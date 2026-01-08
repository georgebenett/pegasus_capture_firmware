/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef GATTS_H
#define GATTS_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize BLE GATT server
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gatts_init(void);

/**
 * @brief Send MAX30102 sensor data over BLE
 * @param red_samples Array of red LED samples
 * @param ir_samples Array of IR LED samples
 * @param num_samples Number of samples in arrays
 * @param sample_rate_hz Sample rate in Hz (e.g., 400 for 400Hz)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gatts_send_sensor_data(const uint32_t *red_samples, const uint32_t *ir_samples, uint8_t num_samples, float sample_rate_hz);

#ifdef __cplusplus
}
#endif

#endif // GATTS_H

