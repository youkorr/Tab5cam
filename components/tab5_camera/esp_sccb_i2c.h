/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations for SCCB types that would normally come from ESP-IDF
// We'll use these minimal definitions to avoid header conflicts

typedef struct esp_sccb_io_t* esp_sccb_io_handle_t;

/**
 * @brief SCCB I2C configuration (simplified for ESPHome)
 */
typedef struct {
    uint16_t device_address;             ///< I2C device raw address
    uint32_t scl_speed_hz;               ///< I2C SCL line frequency
    uint32_t addr_bits_width;            ///< Reg address bit-width
    uint32_t val_bits_width;             ///< Reg val bit-width
} sccb_i2c_config_t;

#ifdef __cplusplus
}
#endif
