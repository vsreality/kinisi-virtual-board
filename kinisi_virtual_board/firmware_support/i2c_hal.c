/* One external I2C master. Exercise the firmware address, RX, TX and listen
 * callbacks at transaction boundaries with the firmware's host HAL types. */
#define HAL_GPIO_Init sim_i2c_gpio_init
#include <hardware_i2c.c>
#include <stdbool.h>

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

static uint8_t *rx_target, *tx_source;
static size_t rx_remaining, tx_length;
void sim_i2c_gpio_init(void *port, GPIO_InitTypeDef *config) {}
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t priority, uint32_t subpriority) {}
void HAL_NVIC_EnableIRQ(IRQn_Type irq) {}
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *handle) { return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *handle) { return HAL_OK; }
void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *handle) {}
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *handle) { HAL_I2C_ErrorCallback(handle); }
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *handle,
        uint8_t *data, uint16_t length, uint32_t options) {
    rx_target = data;
    rx_remaining = length;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *handle,
        uint8_t *data, uint16_t length, uint32_t options) {
    tx_source = data;
    tx_length = length;
    return HAL_OK;
}
API int sim_i2c_write(const uint8_t *data, size_t length, int start) {
    // Validate the entire write before consuming bytes so a failed call is retryable.
    bool size_phase = start || (rx_target && receive_state == AWAITING_SIZE);
    if (!start && !rx_target) return 0;
    size_t available = size_phase && length ? (size_t)data[0] + 1 : rx_remaining;
    if (length > available || (size_phase && length && !data[0])) return 0;
    if (length && length == available && is_queue_full(&I2CCommandQueue)) return 0;
    if (start) HAL_I2C_AddrCallback(&hi2c2, I2C_DIRECTION_TRANSMIT, 8);
    while (length) {
        if (!rx_remaining) return 0;
        size_t n = length < rx_remaining ? length : rx_remaining;
        memcpy(rx_target, data, n);
        data += n; length -= n; rx_target += n; rx_remaining -= n;
        if (!rx_remaining) {
            rx_target = NULL;
            HAL_I2C_SlaveRxCpltCallback(&hi2c2);
            if (receive_state == AWAITING_MESSAGE && !rx_target)
                HAL_I2C_ListenCpltCallback(&hi2c2);
        }
    }
    return 1;
}
API void sim_i2c_begin_read(void) {
    rx_target = NULL;
    rx_remaining = 0;
    HAL_I2C_AddrCallback(&hi2c2, I2C_DIRECTION_RECEIVE, 8);
}
API size_t sim_i2c_end_read(uint8_t *data, size_t length) {
    size_t n = length < tx_length ? length : tx_length;
    if (n) memcpy(data, tx_source, n);
    tx_source = NULL;
    tx_length = 0;
    HAL_I2C_ListenCpltCallback(&hi2c2);
    return n;
}
