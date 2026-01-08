/**
 * ESP32-specific MAX30102 interface implementation
 * 
 * GPIO Configuration:
 * - SCL: GPIO 46
 * - SDA: GPIO 45
 * - INT: GPIO 44
 */

#include "driver_max30102_interface.h"
#include "max30102_interface_esp32.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdarg.h>
#include <string.h>

static const char *TAG = "MAX30102_INTERFACE";

// I2C configuration
#define I2C_MASTER_SCL_IO           46          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           45          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0           /*!< I2C master i2c port number */
#define I2C_MASTER_FREQ_HZ          400000      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0           /*!< I2C master doesn't need buffer */

// GPIO interrupt configuration
#define MAX30102_INT_GPIO           44          /*!< GPIO number for MAX30102 interrupt */

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t i2c_dev_handle = NULL;
static uint8_t (*gpio_irq_handler)(void) = NULL;
static QueueHandle_t interrupt_queue = NULL;

// GPIO interrupt handler - must be fast and non-blocking
static void IRAM_ATTR max30102_gpio_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Send notification to queue (non-blocking from ISR)
    if (interrupt_queue != NULL) {
        uint8_t dummy = 1;  // Dummy value to indicate interrupt
        xQueueSendFromISR(interrupt_queue, &dummy, &xHigherPriorityTaskWoken);
    }
    
    // Yield if a task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t max30102_interface_iic_init(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing I2C bus...");
    
    // Configure I2C bus
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    
    ret = i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
        return 1;
    }
    
    // Configure I2C device (MAX30102 address is 0x57 in 7-bit format, which is 0xAE in 8-bit)
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x57,  // 7-bit address (0xAE >> 1)
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(i2c_bus_handle);
        i2c_bus_handle = NULL;
        return 1;
    }
    
    ESP_LOGI(TAG, "I2C bus initialized successfully (SCL: GPIO%d, SDA: GPIO%d)", 
             I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    
    return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t max30102_interface_iic_deinit(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Deinitializing I2C bus...");
    
    if (i2c_dev_handle != NULL) {
        ret = i2c_master_bus_rm_device(i2c_dev_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to remove I2C device: %s", esp_err_to_name(ret));
            return 1;
        }
        i2c_dev_handle = NULL;
    }
    
    if (i2c_bus_handle != NULL) {
        ret = i2c_del_master_bus(i2c_bus_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete I2C bus: %s", esp_err_to_name(ret));
            return 1;
        }
        i2c_bus_handle = NULL;
    }
    
    ESP_LOGI(TAG, "I2C bus deinitialized successfully");
    
    return 0;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr iic device write address
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t max30102_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    esp_err_t ret;
    
    if (i2c_dev_handle == NULL) {
        ESP_LOGE(TAG, "I2C device handle is NULL");
        return 1;
    }
    
    ret = i2c_master_transmit_receive(i2c_dev_handle, &reg, 1, buf, len, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s (reg: 0x%02X, len: %d)", 
                 esp_err_to_name(ret), reg, len);
        return 1;
    }
    
    return 0;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t max30102_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    esp_err_t ret;
    uint8_t write_buf[256];
    
    if (i2c_dev_handle == NULL) {
        ESP_LOGE(TAG, "I2C device handle is NULL");
        return 1;
    }
    
    if (len > 255) {
        ESP_LOGE(TAG, "Write buffer too large: %d", len);
        return 1;
    }
    
    write_buf[0] = reg;
    if (len > 0) {
        memcpy(&write_buf[1], buf, len);
    }
    
    ret = i2c_master_transmit(i2c_dev_handle, write_buf, len + 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s (reg: 0x%02X, len: %d)", 
                 esp_err_to_name(ret), reg, len);
        return 1;
    }
    
    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void max30102_interface_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void max30102_interface_debug_print(const char *const fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    esp_log_writev(ESP_LOG_INFO, TAG, fmt, args);
    va_end(args);
}

/**
 * @brief     interface receive callback
 * @param[in] type irq type
 * @note      none
 */
void max30102_interface_receive_callback(uint8_t type)
{
    switch (type)
    {
        case MAX30102_INTERRUPT_STATUS_FIFO_FULL :
        {
            ESP_LOGI(TAG, "Interrupt: FIFO full");
            break;
        }
        case MAX30102_INTERRUPT_STATUS_PPG_RDY :
        {
            ESP_LOGI(TAG, "Interrupt: PPG ready");
            break;
        }
        case MAX30102_INTERRUPT_STATUS_ALC_OVF :
        {
            ESP_LOGI(TAG, "Interrupt: Ambient light cancellation overflow");
            break;
        }
        case MAX30102_INTERRUPT_STATUS_PWR_RDY :
        {
            ESP_LOGI(TAG, "Interrupt: Power ready");
            break;
        }
        case MAX30102_INTERRUPT_STATUS_DIE_TEMP_RDY :
        {
            ESP_LOGI(TAG, "Interrupt: Die temperature ready");
            break;
        }
        default :
        {
            ESP_LOGI(TAG, "Interrupt: Unknown code (0x%02X)", type);
            break;
        }
    }
}

/**
 * @brief  Initialize GPIO interrupt for MAX30102
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t max30102_gpio_interrupt_init(uint8_t (*irq_handler)(void))
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing GPIO interrupt (GPIO%d)...", MAX30102_INT_GPIO);
    
    gpio_irq_handler = irq_handler;
    
    // Create queue for interrupt notifications (if not already created)
    if (interrupt_queue == NULL) {
        interrupt_queue = xQueueCreate(10, sizeof(uint8_t));
        if (interrupt_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create interrupt queue");
            return 1;
        }
    }
    
    // Configure GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,  // Falling edge trigger
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << MAX30102_INT_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 1,  // Enable pull-up
    };
    
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return 1;
    }
    
    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return 1;
    }
    
    // Hook ISR handler for specific GPIO pin
    ret = gpio_isr_handler_add(MAX30102_INT_GPIO, max30102_gpio_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO ISR handler: %s", esp_err_to_name(ret));
        return 1;
    }
    
    ESP_LOGI(TAG, "GPIO interrupt initialized successfully (GPIO%d)", MAX30102_INT_GPIO);
    
    return 0;
}

/**
 * @brief  Deinitialize GPIO interrupt for MAX30102
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t max30102_gpio_interrupt_deinit(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Deinitializing GPIO interrupt...");
    
    ret = gpio_isr_handler_remove(MAX30102_INT_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove GPIO ISR handler: %s", esp_err_to_name(ret));
        return 1;
    }
    
    gpio_irq_handler = NULL;
    
    // Clean up queue
    if (interrupt_queue != NULL) {
        vQueueDelete(interrupt_queue);
        interrupt_queue = NULL;
    }
    
    ESP_LOGI(TAG, "GPIO interrupt deinitialized successfully");
    
    return 0;
}

/**
 * @brief  Get the interrupt queue handle for processing interrupts in a task
 * @return QueueHandle_t queue handle, or NULL if not initialized
 * @note   Use this queue in a task to receive interrupt notifications
 */
QueueHandle_t max30102_get_interrupt_queue(void)
{
    return interrupt_queue;
}

