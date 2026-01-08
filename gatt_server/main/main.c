/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
*
* Main application entry point
* Handles MAX30102 sensor initialization and processing
*
****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

// MAX30102 driver includes
#include "driver_max30102_fifo.h"
#include "max30102_interface_esp32.h"

// BLE GATT server includes
#include "gatts.h"

#define MAX30102_TAG "MAX30102"

// MAX30102 variables
static uint32_t max30102_raw_red[32];
static uint32_t max30102_raw_ir[32];
static bool max30102_data_ready = false;
static TaskHandle_t max30102_task_handle = NULL;
static TaskHandle_t max30102_irq_task_handle = NULL;

// MAX30102 receive callback
void max30102_receive_callback(uint8_t type)
{
    switch (type)
    {
        case MAX30102_INTERRUPT_STATUS_FIFO_FULL :
        {
            uint8_t res;
            uint8_t len;
            
            // Read data
            len = 32;
            res = max30102_fifo_read((uint32_t *)max30102_raw_red, (uint32_t *)max30102_raw_ir, (uint8_t *)&len);
            if (res != 0)
            {
                ESP_LOGE(MAX30102_TAG, "Read failed");
            }
            else
            {
                // Log raw decimal values only
                for (int i = 0; i < len; i++) {
                    ESP_LOGI(MAX30102_TAG, "Red=%" PRIu32 ", IR=%" PRIu32, max30102_raw_red[i], max30102_raw_ir[i]);
                }
                
                // Send sensor data over BLE (sample rate is 400 Hz as configured)
                esp_err_t ble_ret = gatts_send_sensor_data(max30102_raw_red, max30102_raw_ir, len, 400.0f);
                if (ble_ret != ESP_OK && ble_ret != ESP_ERR_INVALID_STATE) {
                    ESP_LOGW(MAX30102_TAG, "Failed to send sensor data over BLE: %s", esp_err_to_name(ble_ret));
                }
                
                max30102_data_ready = true;
            }
            break;
        }
        case MAX30102_INTERRUPT_STATUS_PPG_RDY :
        {
            ESP_LOGI(MAX30102_TAG, "PPG ready interrupt");
            break;
        }
        case MAX30102_INTERRUPT_STATUS_ALC_OVF :
        {
            ESP_LOGI(MAX30102_TAG, "Ambient light cancellation overflow interrupt");
            break;
        }
        case MAX30102_INTERRUPT_STATUS_PWR_RDY :
        {
            ESP_LOGI(MAX30102_TAG, "Power ready interrupt");
            break;
        }
        case MAX30102_INTERRUPT_STATUS_DIE_TEMP_RDY :
        {
            uint8_t res;
            uint16_t raw_temp;
            float temperature;
            
            // Read temperature
            res = max30102_fifo_read_temperature(&raw_temp, &temperature);
            if (res != 0)
            {
                ESP_LOGE(MAX30102_TAG, "Failed to read temperature");
            }
            else
            {
                ESP_LOGI(MAX30102_TAG, "Die temperature: %.2f°C (raw: %d)", temperature, raw_temp);
            }
            break;
        }
        default :
        {
            ESP_LOGI(MAX30102_TAG, "Unknown interrupt code: 0x%02X", type);
            break;
        }
    }
}

// Task to process MAX30102 interrupts from queue (runs in task context, not ISR)
static void max30102_irq_task(void *pvParameters)
{
    QueueHandle_t interrupt_queue = max30102_get_interrupt_queue();
    uint8_t dummy;
    
    ESP_LOGI(MAX30102_TAG, "MAX30102 interrupt task started");
    
    if (interrupt_queue == NULL) {
        ESP_LOGE(MAX30102_TAG, "Interrupt queue is NULL");
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        // Wait for interrupt notification from ISR
        if (xQueueReceive(interrupt_queue, &dummy, portMAX_DELAY) == pdTRUE) {
            // Process interrupt in task context (safe to call I2C operations)
            if (max30102_fifo_irq_handler() != 0) {
                ESP_LOGE(MAX30102_TAG, "Failed to process interrupt");
            }
        }
    }
}

// Task to periodically read and log MAX30102 data
static void max30102_task(void *pvParameters)
{
    uint8_t len;
    uint8_t res;
    
    ESP_LOGI(MAX30102_TAG, "MAX30102 task started");
    
    while (1) {
        // Try to read data periodically (even if no interrupt)
        len = 32;
        res = max30102_fifo_read((uint32_t *)max30102_raw_red, (uint32_t *)max30102_raw_ir, (uint8_t *)&len);
        if (res == 0 && len > 0) {

              // Send sensor data over BLE (sample rate is 400 Hz as configured)
            esp_err_t ble_ret = gatts_send_sensor_data(max30102_raw_red, max30102_raw_ir, len, 400.0f);
            if (ble_ret != ESP_OK && ble_ret != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(MAX30102_TAG, "Failed to send sensor data over BLE: %s", esp_err_to_name(ble_ret));
            }
            
            max30102_data_ready = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms (faster polling)
    }
}

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // Initialize BLE GATT server
    ret = gatts_init();
    if (ret != ESP_OK) {
        ESP_LOGE(MAX30102_TAG, "Failed to initialize BLE GATT server: %s", esp_err_to_name(ret));
        return;
    }
    
    // Initialize MAX30102
    ESP_LOGI(MAX30102_TAG, "Initializing MAX30102 sensor...");
    ESP_LOGI(MAX30102_TAG, "GPIO Configuration: SCL=GPIO46, SDA=GPIO45, INT=GPIO44");
    
    // Initialize MAX30102 FIFO first (before setting up interrupts)
    uint8_t res = max30102_fifo_init(max30102_receive_callback);
    if (res != 0) {
        ESP_LOGE(MAX30102_TAG, "Failed to initialize MAX30102 FIFO");
    } else {
        ESP_LOGI(MAX30102_TAG, "MAX30102 FIFO initialized successfully");
        
        // Initialize GPIO interrupt (creates queue for interrupt notifications)
        res = max30102_gpio_interrupt_init(max30102_fifo_irq_handler);
        if (res != 0) {
            ESP_LOGE(MAX30102_TAG, "Failed to initialize GPIO interrupt");
        } else {
            ESP_LOGI(MAX30102_TAG, "GPIO interrupt initialized successfully");
            
            // Create task to process interrupts from queue (runs in task context)
            xTaskCreate(max30102_irq_task, "max30102_irq_task", 4096, NULL, 10, &max30102_irq_task_handle);
            ESP_LOGI(MAX30102_TAG, "MAX30102 interrupt task created");
        }
        
        // Create task to read MAX30102 data periodically
        xTaskCreate(max30102_task, "max30102_task", 4096, NULL, 5, &max30102_task_handle);
        ESP_LOGI(MAX30102_TAG, "MAX30102 task created");
    }

    return;
}

