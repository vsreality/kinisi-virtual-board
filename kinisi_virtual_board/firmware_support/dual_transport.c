/* Drive the production queue dispatcher one cooperative turn at a time.
 * The host has no concurrent interrupt producers. */
#include "dual_transport.h"
#include <limits.h>
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
#include <commands_manager.c>

message_queue_t CommandQueue;
USBD_HandleTypeDef hUsbDeviceFS = {USBD_STATE_CONFIGURED, &hUsbDeviceFS};

static uint32_t irq_mask;
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1; }
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }
void MX_USB_DEVICE_Init(void) {}
void MX_USB_DEVICE_Process(void) {}  // Virtual USB attachment is supplied by the host.
const os_interface_t *get_os_interface(void) { return NULL; }

static bool usb_busy;
#ifdef _WIN32
__declspec(dllexport)
#endif
void sim_usb_busy(int busy) { usb_busy = busy != 0; }

uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length) {
    return !usb_busy && sim_usb_send(data, (uint8_t)length) ? USBD_OK : 1;
}

void sim_transport_init(void) {
    init_queue(&CommandQueue);
    init_queue(&I2CCommandQueue);
    initialize_external_i2c();
    connection_init(&usb_connection, command_handler, try_usb, hw_clock_microseconds);
    connection_init(&i2c_connection, command_handler, try_i2c, hw_clock_microseconds);
    const connection_services_t services = {stop_on_connection_loss, odometry_manager_get_period_ms,
                                           period_allowed, event_sequence};
    usb_connection.services = i2c_connection.services = services;
}
void sim_transport_expire(void) {
    connection_poll(&usb_connection);
    connection_poll(&i2c_connection);
}
void sim_transport_step(void) {
    process_queue(&usb_connection, &CommandQueue);
    process_queue(&i2c_connection, &I2CCommandQueue);
    sim_transport_expire();
}
int sim_usb_feed(const uint8_t *data, size_t length) {
    if (length > UINT_MAX) return 0;
    // Preserve the native API's all-or-nothing contract when a batch is too large.
    size_t offset = 0;
    unsigned slots = MESSAGE_QUEUE_MAX_SIZE - CommandQueue.count;
    size_t remaining = CommandQueue.incomplete_count
        ? (unsigned char)CommandQueue.incomplete[0] + 1u - CommandQueue.incomplete_count : 0;
    while (offset < length) {
        if (!remaining) remaining = (size_t)data[offset] + 1;
        if (remaining > length - offset) break;
        if (!slots) return 0;
        --slots;
        offset += remaining;
        remaining = 0;
    }
    return enqueue_multi(&CommandQueue, (char *)data, (unsigned)length);
}
void sim_usb_disconnect(void) { stop_on_connection_loss(); }
int sim_usb_lost(void) { return usb_connection.lost; }
int sim_usb_ready(void) { return usb_connection.ready_announced; }
