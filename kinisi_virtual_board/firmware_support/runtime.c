/* Native boundary only. All protocol, command and control algorithms link from
 * the selected firmware checkout. One loaded library instance owns one board. */
#include "connection.h"
#include "commands_handler.h"
#include "controllers_manager.h"
#include "odometry_manager.h"
#include "platform.h"
#include "hw_config.h"
#if BOARD_HAS_BOARD_IDLE
#include "wake_barriers.h"
#endif
#include "hw_clock.h"
#include "hw_gpio.h"
#include "hw_temperature.h"
#include "hw_imu.h"
#include "hw_current_sense.h"
#include "hw_motor_faults.h"
#include "hw_motor_current_limit.h"
#include "hw_drv8873.h"
#ifdef SIM_CONFIGURATION_STORAGE
#include "hw_fram.h"
#include "controller_configuration.h"
#endif
#include "cmsis_os.h"
#include "semphr.h"
#include <math.h>
#include <setjmp.h>
#include <string.h>
#include "bmi088_model.h"

_Static_assert(sizeof(double) == 8, "Kinisi wire format requires 64-bit doubles");

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API __attribute__((visibility("default")))
#endif

static uint64_t now_us;
static uint8_t tx[65536];
static size_t tx_size;
#ifdef SIM_DUAL_TRANSPORT
#include "dual_transport.h"
#else
static connection_t session;
static uint8_t rx[65536];
static size_t rx_size;
#ifdef SIM_DRIVER_HAL
static size_t bytes_before_wake;
static bool consume_mark(size_t *marked, size_t frame_size) {
    bool stale = *marked != 0;
    *marked = *marked > frame_size ? *marked - frame_size : 0;
    return stale;
}
#if BOARD_HAS_BOARD_IDLE
static size_t bytes_before_board_wake;
#endif
#endif
#endif
static unsigned dropped[256];
static int runtime_error;
static double pwm[4], resolutions[4], temperature = 25., currents[4];
static uint16_t counts[4];
static uint8_t motor_ready[4], motor_reverse[4], motor_mode[4];
static uint8_t encoder_ready[4], encoder_reverse[4], pins[10], led;
static hw_imu_sample_t imu = {.acceleration_z = 9.80665};
static uint64_t imu_accel_us, imu_gyro_us;
static bmi088_model_t imu_model;
static uint32_t imu_seed = 1;
static double imu_temperature_c = 25.;
static bool imu_injected, imu_started;
static double shaft_velocity[4];
static platform_odometry_t previous_body_velocity;
static double body_acceleration[3] = {0., 0., 9.80665}, body_gyro[3];
static hw_motor_faults_t faults[4];
static uint32_t fault_sequence[4];
static hw_motor_current_limit_t limits[4];
static int temperature_status, imu_status, current_status[4];

/* Only the physical storage boundary is simulated. Record selection, CRC,
 * commit markers, save capture and boot restore all run production C code. */
enum { FRAM_BYTES = 8192 };
static uint8_t fram[FRAM_BYTES];
static bool fram_loaded, fram_read_failure;
static int64_t fram_write_remaining = -1;
static uint32_t fram_reads, fram_writes;
bool hw_fram_read(uint16_t address, void *data, uint16_t length) {
    ++fram_reads;
    if (!data || (size_t)address + length > sizeof(fram) || fram_read_failure) return false;
    memcpy(data, fram + address, length);
    return true;
}
bool hw_fram_write(uint16_t address, const void *data, uint16_t length) {
    ++fram_writes;
    if (!data || (size_t)address + length > sizeof(fram)) return false;
    size_t written = length;
    if (fram_write_remaining >= 0 && (uint64_t)fram_write_remaining < written)
        written = (size_t)fram_write_remaining;
    memcpy(fram + address, data, written);
    if (fram_write_remaining >= 0) fram_write_remaining -= (int64_t)written;
    return written == length;
}
API int sim_fram_load(const uint8_t *data, size_t length) {
    if (!data || length != sizeof(fram)) return 0;
    memcpy(fram, data, length); fram_loaded = true; return 1;
}
API size_t sim_fram_read(uint8_t *data, size_t capacity) {
    if (!data || capacity < sizeof(fram)) return 0;
    memcpy(data, fram, sizeof(fram)); return sizeof(fram);
}
API void sim_fram_fault(int64_t write_after_bytes, int read_failure) {
    fram_write_remaining = write_after_bytes; fram_read_failure = read_failure != 0;
}
API uint32_t sim_fram_reads(void) { return fram_reads; }
API uint32_t sim_fram_writes(void) { return fram_writes; }

typedef struct { void (*fn)(void *); void *arg; uint64_t due; int primed; } task_t;
static task_t tasks[4], *active_task;
static unsigned task_count, delay_count, mutex_count;
static task_t *controller_task;
static uint64_t controller_dispatches;
API uint64_t sim_controller_dispatches(void) { return controller_dispatches; }
static uint8_t mutexes[8];
static jmp_buf yield_task;

/* These two production tasks have a delay at the top of their loop and no
 * persistent automatic state beyond their argument/wake tick. Reenter them and
 * yield at the next loop boundary, exactly as the existing firmware host tests.
 * This is deterministic cooperative stepping, not a port of FreeRTOS. */
osThreadId_t osThreadNew(void (*fn)(void *), void *arg, const osThreadAttr_t *attr) {
    if (task_count == 4) { runtime_error = 1; return NULL; }
    task_t *task = &tasks[task_count++];
    *task = (task_t){.fn = fn, .arg = arg, .due = now_us};
    if (attr && attr->name && strcmp(attr->name, "ControllerTask") == 0) controller_task = task;
    return task;
}
osThreadState_t osThreadGetState(osThreadId_t task) { return task ? 0 : osThreadError; }
TickType_t xTaskGetTickCount(void) { return (TickType_t)(now_us / 1000); }
void vTaskDelayUntil(TickType_t *last, TickType_t period) {
    (void)last;
    if (!active_task || !period) { runtime_error = 2; longjmp(yield_task, 1); }
    if (!active_task->primed || delay_count++) {
        active_task->primed = 1;
        active_task->due = now_us + (uint64_t)period * 1000;
        longjmp(yield_task, 1);
    }
}
SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    if (mutex_count == sizeof(mutexes)) { runtime_error = 3; return NULL; }
    return &mutexes[mutex_count++];
}
int xSemaphoreTake(SemaphoreHandle_t handle, uint32_t timeout) {
    (void)timeout;
    if (!handle || *(uint8_t *)handle) { runtime_error = 4; return 0; }
    *(uint8_t *)handle = 1;
    return 1;
}
void xSemaphoreGive(SemaphoreHandle_t handle) {
    if (!handle || !*(uint8_t *)handle) { runtime_error = 5; return; }
    *(uint8_t *)handle = 0;
}
uint64_t hw_clock_microseconds(void) { return now_us; }

#ifndef SIM_DRIVER_HAL
static void motor_stop(motorIndex i) { pwm[i] = 0; motor_mode[i] = 0; }
static uint32_t pwm_frequency = SIM_PWM_DEFAULT;
uint32_t hw_motor_get_pwm_frequency(void) { return pwm_frequency; }
bool hw_motor_pwm_frequency_allowed(uint32_t value) {
    for (unsigned i = 0; i < sizeof(sim_pwm_frequencies) / sizeof(sim_pwm_frequencies[0]); ++i)
        if (value == sim_pwm_frequencies[i]) return true;
    return false;
}
bool hw_motor_set_pwm_frequency(uint32_t value) {
    if (!hw_motor_pwm_frequency_allowed(value)) return false;
    for (unsigned i = 0; i < 4; ++i) motor_stop(i);
    pwm_frequency = value;
    return true;
}
void hw_motor_discard_restored_setup(void) {
    hw_motor_set_pwm_frequency(SIM_PWM_DEFAULT);
    memset(motor_ready, 0, sizeof(motor_ready));
    memset(motor_reverse, 0, sizeof(motor_reverse));
}
static void motor_brake(motorIndex i) { pwm[i] = 0; motor_mode[i] = 1; }
static void motor_init(motorIndex i, bool reverse) {
    motor_stop(i); motor_ready[i] = !(SIM_HAS_DRIVER && faults[i].fault_active); motor_reverse[i] = reverse;
    limits[i] = (hw_motor_current_limit_t){sim_limit_enabled[i], sim_limit_default[i]};
}
static uint8_t motor_initialized(motorIndex i) { return i < 4 && motor_ready[i]; }
static uint8_t motor_reversed(motorIndex i) { return motor_reverse[i]; }
static void motor_speed(motorIndex i, double value) {
    if (!motor_ready[i] || faults[i].fault_active) { motor_stop(i); return; }
    pwm[i] = fmax(-100., fmin(100., value)); motor_mode[i] = 2;
}
const hw_motor_interface_t *get_motor_interface(void) {
    static const hw_motor_interface_t value = {motor_init, motor_reversed, motor_initialized, motor_speed, motor_stop, motor_brake};
    return &value;
}
#else
#ifndef SIM_DUAL_TRANSPORT
static void motor_stop(motorIndex i) { get_motor_interface()->stop(i); }
#endif
#endif
static void encoder_init(encoder_index_t i, double resolution, uint8_t reverse) {
    encoder_ready[i] = 1; resolutions[i] = resolution; encoder_reverse[i] = reverse;
}
bool hw_encoder_try_initialize(encoder_index_t i, double resolution, uint8_t reverse) {
    if (i >= 4) return false;
    encoder_init(i, resolution, reverse); return true;
}
void hw_encoder_discard_restored_setup(void) {
    memset(encoder_ready, 0, sizeof(encoder_ready));
    memset(encoder_reverse, 0, sizeof(encoder_reverse));
    memset(resolutions, 0, sizeof(resolutions));
}
static uint16_t encoder_count(encoder_index_t i) { return encoder_reverse[i] ? (uint16_t)(0 - counts[i]) : counts[i]; }
static uint8_t encoder_direction(encoder_index_t i) { return encoder_reverse[i]; }
uint8_t hw_encoder_is_reversed(encoder_index_t i) { return i < 4 ? encoder_reverse[i] : 0; }
static uint8_t encoder_initialized(encoder_index_t i) { return i < 4 && encoder_ready[i]; }
static double encoder_resolution(encoder_index_t i) { return resolutions[i]; }
const hw_encoder_interface_t *get_encoder_interface(void) {
    static const hw_encoder_interface_t value = {encoder_init, encoder_count, encoder_direction, encoder_initialized, encoder_resolution};
    return &value;
}
static void pin_init(gpio_pin_t i, uint8_t mode) { if (i < NUMBER_GPIO_PINS) pins[i] = mode == GPIO_MODE_INPUT_PULLUP; }
static uint8_t pin_get(gpio_pin_t i) { return i < NUMBER_GPIO_PINS ? pins[i] : 0; }
static void pin_set(gpio_pin_t i, uint8_t value) { if (i < NUMBER_GPIO_PINS) pins[i] = value; }
static void pin_toggle(gpio_pin_t i) { if (i < NUMBER_GPIO_PINS) pins[i] ^= 1; }
static void led_init(void) { led = 0; }
static void led_set(uint8_t value) { led = value; }
static void led_toggle(void) { led ^= 1; }
const gpio_interface_t *get_gpio_interface(void) {
    static const gpio_interface_t value = {pin_init, pin_get, pin_set, pin_toggle, led_init, led_set, led_toggle};
    return &value;
}
static uint64_t temperature_reads, imu_reads;
API uint64_t sim_temperature_reads(void) { return temperature_reads; }
API uint64_t sim_imu_reads(void) { return imu_reads; }
hw_temperature_status_t hw_temperature_read(uint8_t i, double *value) {
    ++temperature_reads;
#if NUMBER_TEMPERATURE_SENSORS == 0
    return HW_TEMPERATURE_UNAVAILABLE;
#else
    if (i >= NUMBER_TEMPERATURE_SENSORS) return HW_TEMPERATURE_UNAVAILABLE;
    if (temperature_status) return (hw_temperature_status_t)temperature_status;
    if (now_us < 300000) return HW_TEMPERATURE_NOT_READY;
    *value = temperature; return HW_TEMPERATURE_OK;
#endif
}
static hw_imu_status_t imu_read_status(uint8_t i) {
    ++imu_reads;
#if NUMBER_IMU_SENSORS == 0
    return HW_IMU_UNAVAILABLE;
#else
    if (i >= NUMBER_IMU_SENSORS) return HW_IMU_UNAVAILABLE;
    if (imu_status) return (hw_imu_status_t)imu_status;
    if (now_us < 50000) return HW_IMU_NOT_READY;
    return HW_IMU_OK;
#endif
}
hw_imu_status_t hw_imu_read(uint8_t i, hw_imu_sample_t *value) {
    hw_imu_status_t status = imu_read_status(i);
    if (status == HW_IMU_OK) *value = imu;
    return status;
}
API hw_imu_status_t hw_imu_read_observation(uint8_t index, hw_imu_sample_t *sample,
                                           uint64_t *accel_us, uint64_t *gyro_us) {
    hw_imu_status_t status = imu_read_status(index);
    if (status != HW_IMU_OK) return status;
    if (!sample || !accel_us || !gyro_us) return HW_IMU_IO_ERROR;
    if (!imu_accel_us || !imu_gyro_us) return HW_IMU_NOT_READY;
    *sample = imu;
    *accel_us = imu_accel_us;
    *gyro_us = imu_gyro_us;
    return HW_IMU_OK;
}
hw_current_sense_status_t hw_current_sense_read(uint8_t i, double *value) {
#if NUMBER_CURRENT_SENSORS == 0
    return HW_CURRENT_SENSE_UNAVAILABLE;
#else
    if (i >= NUMBER_CURRENT_SENSORS) return HW_CURRENT_SENSE_UNAVAILABLE;
    if (current_status[i]) return (hw_current_sense_status_t)current_status[i];
    *value = currents[i]; return HW_CURRENT_SENSE_OK;
#endif
}
#ifndef SIM_DRIVER_HAL
bool hw_drv8873_fault_active(motorIndex i) { return i >= 4 || faults[i].fault_active; }
uint32_t hw_motor_fault_event_sequence(uint8_t i) { return i < 4 ? fault_sequence[i] : 0; }
bool hw_motor_read_faults(uint8_t i, hw_motor_faults_t *value) {
    if (!SIM_HAS_DRIVER || i >= 4) return false;
    *value = faults[i]; value->sleep_requested = !motor_ready[i]; return true;
}
bool hw_motor_clear_faults(uint8_t i, hw_motor_faults_t *value) {
    if (!SIM_HAS_DRIVER || i >= 4 || !motor_ready[i]) return false;
    motor_stop(i); faults[i] = (hw_motor_faults_t){.status_valid = true};
    ++fault_sequence[i]; *value = faults[i]; return true;
}
bool hw_motor_current_limit_allowed(uint8_t i, const hw_motor_current_limit_t *value) {
    if (!SIM_HAS_DRIVER || i >= 4 || !value || (!value->enabled && !sim_limit_can_disable[i])) return false;
    for (unsigned n = 0; n < sim_limit_count[i]; ++n)
        if (sim_limit_values[i][n] == value->threshold_ma) return true;
    return false;
}
bool hw_motor_get_current_limit(uint8_t i, hw_motor_current_limit_t *value) {
    if (!SIM_HAS_DRIVER || i >= 4 || !motor_ready[i]) return false;
    *value = limits[i]; return true;
}
bool hw_motor_set_current_limit(uint8_t i, const hw_motor_current_limit_t *value) {
    if (!hw_motor_current_limit_allowed(i, value) || !motor_ready[i]) return false;
    motor_stop(i); limits[i] = *value; return true;
}

#endif

/* Wiring equivalent to the production command task's connection services,
 * with one native stream instead of USB + I2C queues. */
#ifndef SIM_DUAL_TRANSPORT
static void stop_all(void) {
    platform_stop_velocity_controller();
    for (uint8_t i = 0; i < 4; ++i) { controllers_manager_stop_controller(i); motor_stop(i); }
    rx_size = 0;
#ifdef SIM_DRIVER_HAL
    bytes_before_wake = 0;
#if BOARD_HAS_BOARD_IDLE
    bytes_before_board_wake = 0;
#endif
#endif
}
static bool period_allowed(uint32_t period) { return connection_period_allowed(&session, period); }
static uint32_t event_sequence(uint8_t topic, uint8_t i) {
    return topic == TOPIC_MOTOR_FAULTS ? hw_motor_fault_event_sequence(i) : 0;
}
#endif
bool sim_usb_send(uint8_t *data, uint8_t length) {
    if (dropped[data[1]] && (data[2] || data[3])) { --dropped[data[1]]; return true; }
    if (tx_size + length > sizeof(tx)) return false;
    memcpy(tx + tx_size, data, length); tx_size += length; return true;
}
API void sim_init(void) {
    bmi088_init(&imu_model, imu_seed, imu_temperature_c);
    imu_started = true;
    if (!fram_loaded) { memset(fram, 0xff, sizeof(fram)); fram_loaded = true; }
#ifdef SIM_DRIVER_HAL
    hw_motor_init();
#else
    for (unsigned i = 0; i < 4; ++i) faults[i].status_valid = true;
#endif
#ifdef SIM_CONFIGURATION_STORAGE
    controller_configuration_restore();
#endif
#ifdef SIM_DUAL_TRANSPORT
    sim_transport_init();
#else
    connection_init(&session, command_handler, sim_usb_send, hw_clock_microseconds);
    session.services = (connection_services_t){stop_all, odometry_manager_get_period_ms, period_allowed, event_sequence};
#endif
}
API int sim_feed(const uint8_t *data, size_t length) {
#ifdef SIM_DUAL_TRANSPORT
    return sim_usb_feed(data, length);
#else
    if (length > sizeof(rx) - rx_size) return 0;
    memcpy(rx + rx_size, data, length); rx_size += length; return 1;
#endif
}
static void run_tasks(void) {
    for (volatile unsigned i = 0; i < task_count; ++i) {
        if (tasks[i].due > now_us) continue;
        active_task = &tasks[i]; delay_count = 0;
        if (active_task == controller_task && active_task->primed) ++controller_dispatches;
        if (!setjmp(yield_task)) active_task->fn(active_task->arg);
        active_task = NULL;
    }
}
API int sim_poll(uint64_t time_us) {
    if (time_us < now_us) return 6;
    const double dt = (double)(time_us - now_us) / 1e6;
    now_us = time_us;
    if (!imu_injected) {
        /* Configured platforms assume attached no-slip wheels. Convert actual
         * plant shaft rates, not targets/counts/estimated pose, through the
         * configured forward kinematics. Encoder direction specifies mounting.
         * The external chassis bypasses this through exact observation injection. */
        if (dt > 0.) {
            uint8_t indexes[4] = {0, 1, 2, 3};
            double wheels[4];
            for (unsigned i = 0; i < 4; ++i)
                wheels[i] = encoder_reverse[i] ? -shaft_velocity[i] : shaft_velocity[i];
            platform_odometry_t body = platform_update_odometry(indexes, wheels, 4);
            body_acceleration[0] = (body.x - previous_body_velocity.x) / dt - body.t * body.y;
            body_acceleration[1] = (body.y - previous_body_velocity.y) / dt + body.t * body.x;
            body_gyro[2] = body.t;
            previous_body_velocity = body;
        }
        bmi088_advance(&imu_model, now_us, body_acceleration, body_gyro);
        memcpy(&imu, imu_model.values, sizeof(imu));
        imu_accel_us = imu_model.accel_us;
        imu_gyro_us = imu_model.gyro_us;
    }
#ifdef SIM_DUAL_TRANSPORT
    sim_transport_expire();
    run_tasks();
    for (unsigned turn = 0; turn < 64; ++turn) sim_transport_step();
#else
    /* Expire the watchdog before either controller updates or queued commands. */
    connection_poll(&session);
    run_tasks();
    for (unsigned turn = 0; turn < 64; ++turn) {
        if (rx_size && connection_can_receive(&session)) {
            size_t size = (size_t)rx[0] + 1;
            if (rx_size >= size) {
                uint8_t request[255];
                memcpy(request, rx + 1, size - 1);
                rx_size -= size; memmove(rx, rx + size, rx_size);
#ifdef SIM_DRIVER_HAL
                // As in the firmware ingress queues, a frame whose first byte
                // was buffered at wake remains marked through later fragments.
                bool before_driver_wake = consume_mark(&bytes_before_wake, size);
#if BOARD_HAS_BOARD_IDLE
                bool was_idle = connection_board_idle();
                uint8_t stale = before_driver_wake ? BEFORE_DRIVER_WAKE : 0;
                if (consume_mark(&bytes_before_board_wake, size)) stale |= BEFORE_BOARD_WAKE;
#else
                bool stale = before_driver_wake;
#endif
                bool was_asleep = hw_motor_drivers_sleeping();
                connection_receive_queued(&session, request, size - 1, stale);
                if (was_asleep && !hw_motor_drivers_sleeping()) bytes_before_wake = rx_size;
#if BOARD_HAS_BOARD_IDLE
                if (was_idle && !connection_board_idle()) bytes_before_board_wake = rx_size;
#endif
#else
                connection_receive(&session, request, size - 1);
#endif
            }
        }
        connection_poll(&session);
    }
#endif
    run_tasks(); /* Prime tasks created by this batch of commands at this time. */
    return runtime_error;
}
API size_t sim_read(uint8_t *data, size_t capacity) {
    size_t length = tx_size < capacity ? tx_size : capacity;
    memcpy(data, tx, length); tx_size -= length; memmove(tx, tx + length, tx_size); return length;
}
API uint64_t sim_deadline(void) {
    uint64_t next = UINT64_MAX;
    for (unsigned i = 0; i < task_count; ++i)
        if (tasks[i].due < next) next = tasks[i].due;
    return next;
}
API void sim_disconnect(void) {
#ifdef SIM_DUAL_TRANSPORT
    sim_usb_disconnect();
    tx_size = 0;
#else
    connection_disconnect(&session);
    rx_size = tx_size = 0;
#ifdef SIM_DRIVER_HAL
    bytes_before_wake = 0;
#if BOARD_HAS_BOARD_IDLE
    bytes_before_board_wake = 0;
#endif
#endif
#endif
}
#ifdef SIM_DUAL_TRANSPORT
API int sim_lost(void) { return sim_usb_lost(); }
API int sim_ready(void) { return sim_usb_ready(); }
#else
API int sim_lost(void) { return session.lost; }
API int sim_ready(void) { return session.ready_announced; }
#endif
API void sim_drop(uint8_t command, unsigned count) { dropped[command] = count; }
API void sim_encoder(uint8_t index, uint16_t value) { if (index < 4) counts[index] = value; }
API uint16_t sim_encoder_count(uint8_t i) { return i < 4 ? counts[i] : 0; }
API void sim_shaft_velocity(const double *values) { memcpy(shaft_velocity, values, sizeof(shaft_velocity)); }
API double sim_resolution(uint8_t i) { return i < 4 ? resolutions[i] : 0; }
#ifndef SIM_DRIVER_HAL
API double sim_pwm(uint8_t i) { return i < 4 ? pwm[i] : 0; }
API int sim_reverse(uint8_t i) { return i < 4 ? motor_reverse[i] : 0; }
API int sim_mode(uint8_t i) { return i < 4 ? motor_mode[i] : 0; }
#endif
API void sim_temperature(double value, int status) { temperature = value; temperature_status = status; }
API int sim_imu_configure(uint32_t seed, double temperature_c) {
    if (imu_started || !isfinite(temperature_c) || temperature_c < -40. || temperature_c > 85.) return 0;
    imu_seed = seed;
    imu_temperature_c = temperature_c;
    return 1;
}
API void sim_imu_observation(const double *values, int status, uint64_t accel_us, uint64_t gyro_us) {
    imu_injected = true; /* Exact injected values/status/timestamps stay frozen. */
    memcpy(&imu, values, sizeof(imu));
    imu_status = status;
    imu_accel_us = accel_us;
    imu_gyro_us = gyro_us;
}
API void sim_imu(const double *values, int status) {
    sim_imu_observation(values, status, now_us, now_us);
}
API void sim_current(uint8_t i, double value, int status) { if (i < 4) { currents[i] = value; current_status[i] = status; } }
#ifndef SIM_DRIVER_HAL
API void sim_fault(uint8_t i, int active) {
    if (i < 4) { faults[i].fault_active = faults[i].overcurrent = active != 0; ++fault_sequence[i]; }
}
// Profiles with no physical sleep control always have an inactive latch.
bool hw_motor_drivers_sleeping(void) { return false; }
bool hw_motor_set_driver_sleep(bool sleeping) { (void)sleeping; return false; }
API int sim_sleep_readback_failure(uint8_t i, int active) { return 0; }
API int sim_wake_readback_failure(uint8_t i, int active) { return 0; }
#endif
