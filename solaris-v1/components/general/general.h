#ifndef GENERAL_H
#define GENERAL_H

#include "esp_err.h"
#include "macros.h"  // Para la definición de data_t

// Inicializa la comunicación con IMU y barómetro utilizando los structs pasados como parámetros.
// Devuelve ESP_OK si la inicialización es exitosa, o un código de error en caso contrario.
esp_err_t init_common_sensors(data_t *icm, data_t *baro);

// Realiza las configuraciones específicas de cada sensor
// Despertart sensores, cambios de modo...
esp_err_t configure_common_sensors(data_t *icm, data_t *baro);

// Lee y muestra las mediciones del barómetro.
// Se recibe el puntero al struct del barómetro.
void read_common_sensors(data_t *icm, data_t *baro);

#endif // GENERAL_H
