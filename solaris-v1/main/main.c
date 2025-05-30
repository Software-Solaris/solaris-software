#include <stdio.h>
#include "bmp390.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

void app_main(void)
{
    /* 1) Delay inicial para PuTTY */
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* 2) Init original */
    spi_device_handle_t spi;
    esp_err_t ret = bmp390_init(&spi);
    if (ret != ESP_OK) {
        printf("Error init BMP390: %d\n", ret);
        return;
    }

    /* 3) Soft reset y modo SPI */
    bmp390_soft_reset(spi);
    bmp390_enable_spi_mode(spi);

    /* 4) Leer IF_CONF, CHIP ID y DATA5 */
    uint8_t v;
    bmp390_read_if_conf(spi, &v);   printf("IF_CONF=0x%02X\n", v);
    bmp390_read_chip_id(spi, &v);   printf("CHIP_ID=0x%02X\n", v);
    bmp390_read_data5(spi, &v);     printf("DATA5=0x%02X\n", v);

    /* 5) Cargar calibración de temperatura */
    ret = bmp390_read_calibration(spi);
    if (ret != ESP_OK) {
        printf("Error calibración T: %d\n", ret);
        return;
    }

    /* 6) Deshabilitamos el WDT para esta tarea, por seguridad */
    esp_task_wdt_delete(xTaskGetCurrentTaskHandle());

    /* 7) Bucle de lectura dinámica de temperatura */
    while (1) {
        // Oversampling ×2 para temperatura
        bmp390_write_reg(spi, BMP390_REG_OSR, (1 << 3));

        // Modo forced solo temp: TEMP_EN + MODE=forced (0x05)
        bmp390_write_reg(spi, BMP390_REG_PWR_CTRL, (1<<1)|(1<<4));  // 0x12

        // Esperar drdy_temp con timeout de 500 ms
        uint8_t status = 0;
        const TickType_t start = xTaskGetTickCount();
        while (1) {
            bmp390_read_reg(spi, BMP390_REG_STATUS, &status);
            if (status & (1 << 6)) {
                break;
            }
            if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(500)) {
                ESP_LOGW("MAIN", "drdy_temp timeout");
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        // Leer y compensar
        uint32_t raw_t;
        bmp390_read_raw_temperature(spi, &raw_t);
        float temp_c = BMP390_compensate_temperature(raw_t);
        printf("Temperatura dinámica: %.2f °C\n", temp_c);

        // Esperar ~5 s
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
