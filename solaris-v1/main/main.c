#include <stdio.h>
#include <inttypes.h>
#include "bmp390.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "MAIN";

void app_main(void)
{
    spi_device_handle_t spi;
    esp_err_t ret;
    uint8_t id, ifc;
    bmp390_temp_calib_t raw_calib;
    bmp390_temp_params_t temp_params;
    uint32_t raw_temp;
    bmp390_press_calib_t raw_press_calib;
    bmp390_press_params_t press_params;
    uint32_t raw_press;
    float t_lin;

    // Tiempo para estabilizar
    vTaskDelay(pdMS_TO_TICKS(2000));

    //---------Inicializar----------
    ret = bmp390_init(&spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error init BMP390: %d", ret);
        return;
    }

    ret = bmp390_soft_reset(spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error soft reset: %d", ret);
        return;
    }

    ret = bmp390_enable_spi_mode(spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error enable SPI mode: %d", ret);
        return;
    }
    
    // Dejar tiempo al primer ciclo de configuración
    vTaskDelay(pdMS_TO_TICKS(50));

    //-------Comprobación----------
    ret = bmp390_read_chip_id(spi, &id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer CHIP ID: %d", ret);
        return;
    }
    if (id != BMP390_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "CHIP ID inesperado: 0x%02X", id);
        return;
    }
    ESP_LOGI(TAG, "CHIP ID confirmado: 0x%02X", id);

    ret = bmp390_read_if_conf(spi, &ifc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer IF_CONF: %d", ret);
        return;
    }

    //-------Configuración sensor-------
    ret = bmp390_set_mode_normal(spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error modo normal: %d", ret);
        return;
    }
    ret = bmp390_set_osr_temp(spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error OSR: %d", ret);
        return;
    }
    ret = bmp390_set_odr(spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error ODR: %d", ret);
        return;
    }
    ret = bmp390_set_iir(spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error IIR: %d", ret);
        return;
    }

    // Dejar tiempo al primer dato en Normal mode
    vTaskDelay(pdMS_TO_TICKS(50));

    // ---Leer coeficientes de temperatura raw---
    ret = bmp390_read_raw_temp_coeffs(spi, &raw_calib);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Coef raw T1=%u, T2=%d, T3=%d",
                 raw_calib.par_t1,
                 raw_calib.par_t2,
                 raw_calib.par_t3);
    } else {
        ESP_LOGE(TAG, "Error al leer coef. calib.: %d", ret);
        return;
    }

    //---Calibrar parámetros de temperatura---
    ret = bmp390_calibrate_temp_params(spi, &temp_params);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "PAR_T1 calibrado: %.4f",  temp_params.PAR_T1);
        ESP_LOGI(TAG, "PAR_T2 calibrado: %.6e",  temp_params.PAR_T2);
        ESP_LOGI(TAG, "PAR_T3 calibrado: %.6e",  temp_params.PAR_T3);
    } else {
        ESP_LOGE(TAG, "Error al calibrar params: %d", ret);
        return;
    }

    //----Leer coeficientes de presión raw----
    ret = bmp390_read_raw_press_coeffs(spi, &raw_press_calib);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Coef raw P1=%u, P2=%u, P3=%d, P4=%d, P5=%d, P6=%d, P7=%d, P8=%d, P9=%d, P10=%d, P11=%d",
                 raw_press_calib.par_p1,
                 raw_press_calib.par_p2,
                 raw_press_calib.par_p3,
                 raw_press_calib.par_p4,
                 raw_press_calib.par_p5,
                 raw_press_calib.par_p6,
                 raw_press_calib.par_p7,
                 raw_press_calib.par_p8,
                 raw_press_calib.par_p9,
                 raw_press_calib.par_p10,
                 raw_press_calib.par_p11);
    } else {
        ESP_LOGE(TAG, "Error al leer coef. raw press: %d", ret);
    }

    //----Calibrar parámetros de presión----
    ret = bmp390_calibrate_press_params(spi, &press_params);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "PAR_P1 calibrado: %.6f", press_params.PAR_P1);
        ESP_LOGI(TAG, "PAR_P2 calibrado: %.6f", press_params.PAR_P2);
        ESP_LOGI(TAG, "PAR_P3 calibrado: %.6f", press_params.PAR_P3);
        ESP_LOGI(TAG, "PAR_P4 calibrado: %.6f", press_params.PAR_P4);
        ESP_LOGI(TAG, "PAR_P5 calibrado: %.6f", press_params.PAR_P5);
        ESP_LOGI(TAG, "PAR_P6 calibrado: %.6f", press_params.PAR_P6);
        ESP_LOGI(TAG, "PAR_P7 calibrado: %.6f", press_params.PAR_P7);
        ESP_LOGI(TAG, "PAR_P8 calibrado: %.6f", press_params.PAR_P8);
        ESP_LOGI(TAG, "PAR_P9 calibrado: %.6f", press_params.PAR_P9);
        ESP_LOGI(TAG, "PAR_P10 calibrado: %.6f", press_params.PAR_P10);
        ESP_LOGI(TAG, "PAR_P11 calibrado: %.6f", press_params.PAR_P11);
    } else {
        ESP_LOGE(TAG, "Error al calibrar params press: %d", ret);
    }

    //------------------Bucle de lectura-----------------------
       while (1) {
        ret = bmp390_wait_temp_ready(spi);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error wait temp ready: %d", ret);
            break;
        }

        ret = bmp390_read_raw_temp(spi, &raw_temp);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Raw temp: %u", raw_temp);

            float comp_temp = bmp390_compensate_temperature(raw_temp, &temp_params);
            ESP_LOGI(TAG, "Temp compensada: %.2f °C", comp_temp);

        } else {
            ESP_LOGE(TAG, "Error read raw temp: %d", ret);
            break;
        }

        ret = bmp390_wait_press_ready(spi);
        if (ret != ESP_OK) { 
            ESP_LOGE(TAG, "wait press ready: %d", ret); 
            break; 
        }

        ret = bmp390_read_raw_press(spi, &raw_press);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Raw press: %u", raw_press);

            t_lin = bmp390_compensate_temperature(raw_temp, &temp_params);
            float p_pa = bmp390_compensate_pressure(raw_press, t_lin, &press_params);
            ESP_LOGI(TAG, "Presión comp.: %.2f Pa", p_pa);
            float altitude = 44330.0f * (1.0f - powf(p_pa/101325.0f, 1.0f/5.255f));
            ESP_LOGI(TAG, "Altura: %.2f m", altitude);

        }else {
            ESP_LOGE(TAG, "Error read raw press: %d", ret);
            break;
        }

        // Delay para pruebas (5 s)
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}