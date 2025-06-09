#include "mpu9250.h"
#include "esp_log.h"
#include "driver/spi_common.h"

static const char* TAG = "MPU9250"; 

esp_err_t mpu9250_init(mpu9250_t *p_dev)
{
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
    p_dev->devcfg.mode = 3;           // Puedes probar también con modo 0 si es necesario
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



    // Variables en principio fijas para todas las transacciones
    p_dev->trans_desc.length = 16;
    p_dev->trans_desc.rxlength = 0; //Por defecto, igual que .length
    p_dev->trans_desc.flags = 0;

    // 3. Reset del sensor: escribir 0x80 en PWR_MGMT_1
    uint8_t tx_reset[2] = { REG_PWR_MGMT_1, 0x80 };
    uint8_t rx_reset[2] = { 0, 0 };
    p_dev->trans_desc.tx_buffer = tx_reset;
    p_dev->trans_desc.rx_buffer = rx_reset;

    ret = mpu9250_send_message(p_dev);


    // 4. Despertar el sensor: escribir 0x00 en PWR_MGMT_1
    uint8_t tx_wakeup[2] = { REG_PWR_MGMT_1, 0x00 };
    uint8_t rx_wakeup[2] = { 0, 0 };
    p_dev->trans_desc.tx_buffer = tx_wakeup;
    p_dev->trans_desc.rx_buffer = rx_wakeup;

    ret = mpu9250_send_message(p_dev);

    // 5. Leer el registro WHO_AM_I
    uint8_t tx_data_who[2] = { (uint8_t)(0x80 | REG_WHO_AM_I), 0x00 };
    uint8_t rx_data_who[2] = { 0, 0 };
    p_dev->trans_desc.tx_buffer = tx_data_who;
    p_dev->trans_desc.rx_buffer = rx_data_who;

    ret = mpu9250_send_message(p_dev);

    p_dev->who_am_i = rx_data_who[1];  // El dato real viene en el segundo byte
    ESP_LOGI(TAG, "WHO_AM_I leído: 0x%02X", p_dev->who_am_i);

    return ESP_OK;
}

esp_err_t mpu9250_send_message(mpu9250_t *p_dev) {
    esp_err_t ret= ESP_OK;

    ret = spi_device_transmit(p_dev->handle, &p_dev->trans_desc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al despertar MPU9250: %d", ret);
        return ret;
    } else {
        ESP_LOGI(TAG, "Sensor despertado (PWR_MGMT_1 = 0x00).");
        vTaskDelay(pdMS_TO_TICKS(10));  // Breve espera para estabilizar el sensor
        return ret;
    }
}