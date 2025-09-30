/*
 * Driver SC202CS/SC2356 pour ESP32-P4
 * Basé sur le code M5Stack Tab5
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_cam_sensor.h"
#include "esp_err.h"

/**
 * @brief Créer un nouveau capteur SC202CS
 * 
 * @param config Configuration du capteur
 * @param ret_handle Handle du capteur créé
 * @return esp_err_t 
 */
esp_err_t esp_cam_sensor_new_sc202cs(const esp_cam_sensor_config_t *config, 
                                      esp_cam_sensor_device_t **ret_handle);

#ifdef __cplusplus
}
#endif
