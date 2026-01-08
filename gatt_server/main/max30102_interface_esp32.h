/**
 * ESP32-specific MAX30102 interface header
 */

#ifndef MAX30102_INTERFACE_ESP32_H
#define MAX30102_INTERFACE_ESP32_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize GPIO interrupt for MAX30102
 * @param[in] irq_handler Pointer to interrupt handler function (not used directly, kept for compatibility)
 * @return status code
 *         - 0 success
 *         - 1 init failed
 */
uint8_t max30102_gpio_interrupt_init(uint8_t (*irq_handler)(void));

/**
 * @brief  Deinitialize GPIO interrupt for MAX30102
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 */
uint8_t max30102_gpio_interrupt_deinit(void);

/**
 * @brief  Get the interrupt queue handle for processing interrupts in a task
 * @return QueueHandle_t queue handle, or NULL if not initialized
 * @note   Use this queue in a task to receive interrupt notifications. 
 *         The queue will receive a uint8_t value (1) when an interrupt occurs.
 */
QueueHandle_t max30102_get_interrupt_queue(void);

#ifdef __cplusplus
}
#endif

#endif /* MAX30102_INTERFACE_ESP32_H */

