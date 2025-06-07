#include <stdio.h>
#include <inttypes.h>
#include "bmp390.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

// Tconv para OSR×2 ≈ 10 ms
#define TCONV_MS        10
// Periodo entre muestras: 5 s
#define SAMPLE_MS     5000

void app_main(void)
{
    spi_device_handle_t spi;
    esp_err_t ret;

    vTaskDelay(pdMS_TO_TICKS(2000));

    // 1) Inicializar SPI y sensor
    ret = bmp390_init(&spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error init BMP390: %d", ret);
        return;
    }
    bmp390_soft_reset(spi);
    bmp390_enable_spi_mode(spi);

    // 2) Leer y mostrar calibración
    bmp390_temp_calib_t rawcal;
    bmp390_read_temp_calibration(spi, &rawcal);
    printf("RAW CAL: par_t1=0x%04X, par_t2=0x%04X, par_t3=0x%02X\n",
           rawcal.par_t1, (uint16_t)rawcal.par_t2, (uint8_t)rawcal.par_t3);

    bmp390_temp_params_t tpar;
    bmp390_temp_params(spi, &tpar);
    printf("CONV CAL: PAR_T1=%.5f, PAR_T2=%.8f, PAR_T3=%.15e\n",
           tpar.PAR_T1, tpar.PAR_T2, tpar.PAR_T3);

    // 3) Configurar rápido: OSR×2 e IIR coef=3
    bmp390_write_reg(spi, BMP390_REG_OSR,    BMP390_OSR_TEMP_x2);
    bmp390_write_reg(spi, BMP390_REG_CONFIG, BMP390_IIR_COEFF_3);

    // 4) Bucle de medidas en Forced Mode
    while (1) {
        // a) disparar conversión puntual
        bmp390_write_reg(spi, BMP390_REG_PWR_CTRL, BMP390_PWRCTRL_FORCED);

        // b) esperar Tconv
        vTaskDelay(pdMS_TO_TICKS(TCONV_MS));

        // c) leer raw24 y desplazar
        uint32_t raw24;
        bmp390_read_raw_temp(spi, &raw24);
        uint32_t raw20 = raw24 >> 4;

        // d) compensar directamente
        float temp_c = bmp390_compensate_temperature(raw20, &tpar);

        // e) mostrar
        printf("RAW24=0x%06" PRIX32
               " RAW20=0x%05" PRIX32
               " → T=%.2f°C\n",
               raw24, raw20, temp_c);

        // f) esperar siguiente ciclo
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
    }
}
