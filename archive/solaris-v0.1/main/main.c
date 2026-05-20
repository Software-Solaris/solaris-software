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
    //Variables used as parameters in function calls
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

    //Time to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));

    //---------INIT---------
    ret = bmp390_init(&spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error init BMP390: %d", ret);
        return;
    }

    //---------CONFIG & CHECK---------
    ret = bmp390_soft_reset(spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error soft reset: %d", ret);
        return;
    }

    ret = bmp390_enable_spi_mode(spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error enable SPI mode: %d", ret);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(50)); //Allow time for first configuration cycle

    ret = bmp390_read_chip_id(spi, &id);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading CHIP ID: %d", ret);
        return;
    }
    if (id != BMP390_CHIP_ID_VALUE)
    {
        ESP_LOGE(TAG, "Unexpected CHIP ID: 0x%02X", id);
        return;
    }
    ESP_LOGI(TAG, "Confirmed CHIP ID: 0x%02X", id);

    ret = bmp390_read_if_conf(spi, &ifc);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading IF_CONF: %d", ret);
        return;
    }

    //---------PREPARE READ---------
    ret = bmp390_set_mode_normal(spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting normal mode: %d", ret);
        return;
    }
    ret = bmp390_set_osr_temp(spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting OSR: %d", ret);
        return;
    }
    ret = bmp390_set_odr(spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting ODR: %d", ret);
        return;
    }
    ret = bmp390_set_iir(spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting IIR filter: %d", ret);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(50)); //Allow time for first data in Normal mode

    ret = bmp390_read_raw_temp_coeffs(spi, &raw_calib);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Coef raw T1=%u, T2=%d, T3=%d", raw_calib.par_t1, raw_calib.par_t2,
                 raw_calib.par_t3);
    }
    else
    {
        ESP_LOGE(TAG, "Error reading raw temp coeffs: %d", ret);
        return;
    }

    ret = bmp390_calibrate_temp_params(spi, &temp_params);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibrated PAR_T1: %.4f", temp_params.PAR_T1);
        ESP_LOGI(TAG, "Calibrated PAR_T2: %.6e", temp_params.PAR_T2);
        ESP_LOGI(TAG, "Calibrated PAR_T3: %.6e", temp_params.PAR_T3);
    }
    else
    {
        ESP_LOGE(TAG, "Error calibrating temp params: %d", ret);
        return;
    }

    ret = bmp390_read_raw_press_coeffs(spi, &raw_press_calib);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG,
                 "Coef raw P1=%u, P2=%u, P3=%d, P4=%d, P5=%d, P6=%d, P7=%d, P8=%d, P9=%d, P10=%d, "
                 "P11=%d",
                 raw_press_calib.par_p1, raw_press_calib.par_p2, raw_press_calib.par_p3,
                 raw_press_calib.par_p4, raw_press_calib.par_p5, raw_press_calib.par_p6,
                 raw_press_calib.par_p7, raw_press_calib.par_p8, raw_press_calib.par_p9,
                 raw_press_calib.par_p10, raw_press_calib.par_p11);
    }
    else
    {
        ESP_LOGE(TAG, "Error reading raw press coeffs: %d", ret);
    }

    ret = bmp390_calibrate_press_params(spi, &press_params);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibrated PAR_P1 : %.6f", press_params.PAR_P1);
        ESP_LOGI(TAG, "Calibrated PAR_P2 : %.6f", press_params.PAR_P2);
        ESP_LOGI(TAG, "Calibrated PAR_P3 : %.6f", press_params.PAR_P3);
        ESP_LOGI(TAG, "Calibrated PAR_P4 : %.6f", press_params.PAR_P4);
        ESP_LOGI(TAG, "Calibrated PAR_P5 : %.6f", press_params.PAR_P5);
        ESP_LOGI(TAG, "Calibrated PAR_P6 : %.6f", press_params.PAR_P6);
        ESP_LOGI(TAG, "Calibrated PAR_P7 : %.6f", press_params.PAR_P7);
        ESP_LOGI(TAG, "Calibrated PAR_P8 : %.6f", press_params.PAR_P8);
        ESP_LOGI(TAG, "Calibrated PAR_P9 : %.6f", press_params.PAR_P9);
        ESP_LOGI(TAG, "Calibrated PAR_P10 : %.6f", press_params.PAR_P10);
        ESP_LOGI(TAG, "Calibrated PAR_P11 : %.6f", press_params.PAR_P11);
    }
    else
    {
        ESP_LOGE(TAG, "Error calibrating press params: %d", ret);
    }

    //---------READ LOOP---------
    while (1)
    {
        ret = bmp390_wait_temp_ready(spi);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Error wait temp ready: %d", ret);
            break;
        }

        ret = bmp390_read_raw_temp(spi, &raw_temp);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Raw temp: %u", raw_temp);

            float comp_temp = bmp390_compensate_temperature(raw_temp, &temp_params);
            ESP_LOGI(TAG, "Compensated Temp: %.2f °C", comp_temp);
        }
        else
        {
            ESP_LOGE(TAG, "Error readng raw temp: %d", ret);
            break;
        }

        ret = bmp390_wait_press_ready(spi);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "wait press ready: %d", ret);
            break;
        }

        ret = bmp390_read_raw_press(spi, &raw_press);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Raw press: %u", raw_press);

            t_lin = bmp390_compensate_temperature(raw_temp, &temp_params);
            float p_pa = bmp390_compensate_pressure(raw_press, t_lin, &press_params);
            ESP_LOGI(TAG, "Compensated press: %.2f Pa", p_pa);
            float altitude = 44330.0f * (1.0f - powf(p_pa / 101325.0f, 1.0f / 5.255f));
            ESP_LOGI(TAG, "Height: %.2f m", altitude);
        }
        else
        {
            ESP_LOGE(TAG, "Error reading raw press: %d", ret);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); //Delay for testing (using PuTTY)

    } //End read loop
} //End main

//MAIN_C