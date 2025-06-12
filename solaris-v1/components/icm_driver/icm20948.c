#include "icm20948.h"
#include "esp_log.h"
#include "driver/spi_common.h"

static const char* TAG = "ICM20948"; 

esp_err_t icm20948_send_message(data_t *p_dev, uint8_t tx[2], uint8_t rx[2]) {
    // Ajuste de parámetros de transacción
    p_dev->trans_desc.length = 16;
    p_dev->trans_desc.tx_buffer = tx;
    p_dev->trans_desc.rx_buffer = rx;

    esp_err_t trans_result = spi_device_transmit(p_dev->handle, &p_dev->trans_desc);
    return trans_result;
}


esp_err_t icm20948_init(data_t *p_dev) {

    // 1. Inicializa la configuración del bus SPI
    p_dev->buscfg.miso_io_num = PIN_NUM_CIPO;
    p_dev->buscfg.mosi_io_num = PIN_NUM_COPI;
    p_dev->buscfg.sclk_io_num = PIN_NUM_CLK;
    p_dev->buscfg.quadwp_io_num = -1;
    p_dev->buscfg.quadhd_io_num = -1;
    p_dev->buscfg.max_transfer_sz = 4096;

    esp_err_t ret = spi_bus_initialize(SPI_HOST_USED, &p_dev->buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error spi_bus_initialize: %d", ret);
        return ret;
    } 
    ESP_LOGI(TAG, "SPI Bus initialized.");

    // 2. Configura el dispositivo SPI (CS, velocidad, modo, etc.)
    p_dev->devcfg.clock_speed_hz = 100000;
    p_dev->devcfg.mode = 3;           // Probar en modo 0 si falla
    p_dev->devcfg.spics_io_num = PIN_NUM_CS;
    p_dev->devcfg.queue_size = 1;
    p_dev->devcfg.address_bits = 0;
    p_dev->devcfg.command_bits = 0;
    p_dev->devcfg.dummy_bits = 0;
    p_dev->devcfg.flags = 0;
    p_dev->devcfg.duty_cycle_pos = 128;
    p_dev->devcfg.pre_cb = NULL;
    p_dev->devcfg.post_cb = NULL;
    vTaskDelay(pdMS_TO_TICKS(100)); // Espera adicional de 100 ms

    ret = spi_bus_add_device(SPI_HOST_USED, &p_dev->devcfg, &p_dev->handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error spi_bus_add_device: %d", ret);
        return ret;
    } 
    ESP_LOGI(TAG, "Device added to SPI bus.");



    // Reset del sensor: escribir 0x80 en PWR_MGMT_1
    uint8_t tx_reset[2] = { (uint8_t) (WRITE_OP | REG_PWR_MGMT_1), BIT_H_RESET };
    uint8_t rx_reset[2] = { 0, 0 };

    ret = icm20948_send_message(p_dev, tx_reset, rx_reset);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor reset returned an error: %d", ret);
        return ret;
    }

    // Lectura del contenido del WHO_AM_i: leer 0x00
    uint8_t tx_who_am_i[2] = { (uint8_t) (READ_OP | REG_WHO_AM_I), EMPTY_MESSAGE };
    uint8_t rx_who_am_i[2] = { 0, 0 };

    ret = icm20948_send_message(p_dev, tx_who_am_i, rx_who_am_i);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I reading failed: %d", ret);
        return ret;
    } else {
        p_dev->sensor_id = rx_who_am_i[1]; // Datos útiles en el segundo bit
        ESP_LOGI(TAG, "WHO_AM_I register has 0x%02X | should be: 0xEA", p_dev->sensor_id);
    }

    return ret;
}

esp_err_t icm20948_config(data_t *p_dev) {
    return ESP_OK;
}

esp_err_t icm20948_get_measurements(data_t *p_dev) {
    ESP_LOGI(TAG, "Structure succed!!");
    return ESP_OK;
}

