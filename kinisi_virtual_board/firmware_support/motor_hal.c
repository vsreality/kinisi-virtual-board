// Host-side peripherals for the production v0.4 motor and DRV8873 sources.
// Pin/timer assignments come from the firmware board configuration.
#include "hw_config.h"
#include "hw_motor.h"
#include "hw_timers.h"
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

GPIO_TypeDef test_ports[5];
TIM_TypeDef test_timers[12];
static TIM_HandleTypeDef handles[12];
static uint8_t registers[NUMBER_MOTORS][6];
static bool corrupt_limit[NUMBER_MOTORS];
static bool sleep_readback_high[NUMBER_MOTORS];
static bool injected_fault[NUMBER_MOTORS];

static bool high(gpio_info_t pin) { return (pin.port->IDR & pin.pin) != 0; }
static bool matches(gpio_info_t pin, GPIO_TypeDef *port, uint16_t mask) {
    return pin.port == port && pin.pin == mask;
}

uint32_t HAL_RCC_GetPCLK1Freq(void) { return 42000000; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return 84000000; }
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *clocks, uint32_t *latency) {
    clocks->APB1CLKDivider = RCC_HCLK_DIV4;
    clocks->APB2CLKDivider = RCC_HCLK_DIV2;
    *latency = 5;
}
TIM_HandleTypeDef *get_timer_handeler(TIM_TypeDef *timer) { return &handles[timer - test_timers]; }
uint32_t get_alternate_function_mapping(TIM_TypeDef *timer) { return 1; }
void rcc_gpio_clk_enable(GPIO_TypeDef *port) {}
void rcc_tim_clk_enable(TIM_TypeDef *timer) {}
void HAL_Delay(uint32_t duration) {}
void Error_Handler(void) { abort(); }
int HAL_TIM_Base_GetState(TIM_HandleTypeDef *timer) { return timer->State; }
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *timer) {
    timer->Instance->PSC = timer->Init.Prescaler;
    timer->State = HAL_TIM_STATE_READY;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *t, TIM_ClockConfigTypeDef *c) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *t) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *t, TIM_MasterConfigTypeDef *c) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *t, TIM_OC_InitTypeDef *c, uint32_t channel) {
    __HAL_TIM_SET_COMPARE(t, channel, c->Pulse);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t, uint32_t channel) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *t, uint32_t channel) { return HAL_OK; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config) {
    if (config->Mode == GPIO_MODE_IT_RISING_FALLING) port->IDR |= config->Pin;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i)
        if (sleep_readback_high[i] && matches(motor_info[i].sleep, port, pin)) return GPIO_PIN_SET;
    return port->IDR & pin ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
    if (state == GPIO_PIN_SET) {
        port->ODR |= pin;
        port->IDR |= pin;
    } else {
        port->ODR &= ~pin;
        port->IDR &= ~pin;
        for (unsigned i = 0; i < NUMBER_MOTORS; ++i)
            if (matches(motor_info[i].sleep, port, pin))
                memset(registers[i], 0, sizeof(registers[i]));
    }
}
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *spi) { return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx, uint8_t *rx,
                                          uint16_t size, uint32_t timeout) {
    unsigned reg = (tx[0] >> 1) & 31;
    if (size != 2 || reg >= 6) return HAL_ERROR;
    unsigned selected = NUMBER_MOTORS;
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i) {
        if (high(motor_info[i].chipSelect)) continue;
        if (selected != NUMBER_MOTORS) return HAL_ERROR;
        selected = i;
    }
    if (selected == NUMBER_MOTORS || !high(motor_info[selected].sleep)) return HAL_ERROR;
    uint8_t *bank = registers[selected];
    rx[0] = 0xc0;
    rx[1] = bank[reg];
    if (reg == 0 && injected_fault[selected]) rx[1] |= 4;
    if (!(tx[0] & 0x40)) {
        bank[reg] = tx[1];
        // A failed IC4 write leaves the following verification read mismatched.
        if (reg == 5 && corrupt_limit[selected]) bank[reg] ^= 4;
        if (reg == 4 && (tx[1] & 0x80)) {
            bank[reg] &= 0x7f;
            injected_fault[selected] = false;
            motor_info[selected].fault.port->IDR |= motor_info[selected].fault.pin;
        }
    }
    return HAL_OK;
}
API int sim_sleep_readback_failure(uint8_t i, int active) {
    if (i >= NUMBER_MOTORS) return 0;
    sleep_readback_high[i] = active != 0;
    return 1;
}
API int sim_wake_readback_failure(uint8_t i, int active) {
    if (i >= NUMBER_MOTORS) return 0;
    corrupt_limit[i] = active != 0;
    return 1;
}
API void sim_fault(uint8_t i, int active) {
    if (i >= NUMBER_MOTORS) return;
    gpio_info_t pin = motor_info[i].fault;
    injected_fault[i] = active != 0;
    if (active) pin.port->IDR &= ~pin.pin;
    else pin.port->IDR |= pin.pin;
    HAL_GPIO_EXTI_Callback(pin.pin);
}
API int sim_reverse(uint8_t i) { return i < NUMBER_MOTORS && get_motor_interface()->is_reversed(i); }
API int sim_mode(uint8_t i) {
    if (i >= NUMBER_MOTORS || !high(motor_info[i].sleep) || high(motor_info[i].disable)) return 0;
    const motor_info_t *motor = &motor_info[i];
    return motor->timer->compare[motor->enable.timerChannel / 4] ? 2 : 1;
}
API double sim_pwm(uint8_t i) {
    if (sim_mode(i) != 2) return 0;
    const motor_info_t *motor = &motor_info[i];
    double duty = 100. * motor->timer->compare[motor->enable.timerChannel / 4] / MOTOR_MAX_SPEED;
    return high(motor->direction) != sim_reverse(i) ? duty : -duty;
}
