#include "icm20948.h"
#include "esp_log.h"
#include "driver/spi_common.h"

static const char* TAG = "ICM20948"; 

// Auxiliar para envío de mensajes por spi_device_transmit
esp_err_t icm20948_send_message(data_t *p_dev, uint8_t tx[2], uint8_t rx[2]) {
    p_dev->trans_desc.tx_buffer = tx;
    p_dev->trans_desc.rx_buffer = rx;

    esp_err_t ret = ESP_OK;
    ret = spi_device_transmit(p_dev->handle, &p_dev->trans_desc);
    return ret;
}

// Auxiliar para petición de datos a la ICM20948
esp_err_t get_data(data_t *p_dev, uint8_t address1, uint8_t address2, int16_t *p_data) {
    esp_err_t ret = ESP_OK;

    uint8_t tx_data_h[2] = { (uint8_t)(READ_OP | address1), EMPTY_MESSAGE };
    uint8_t rx_data_h[2] = { 0, 0 };
    ret = icm20948_send_message(p_dev, tx_data_h, rx_data_h);

    uint8_t tx_data_l[2] = { (uint8_t)(READ_OP | address2), EMPTY_MESSAGE };
    uint8_t rx_data_l[2] = { 0, 0 };
    ret = icm20948_send_message(p_dev, tx_data_l, rx_data_l);

    if (ret != ESP_OK) {ESP_LOGE(TAG, "Alguna extracción de datos tuvo un error");}

    *p_data = (int16_t)(rx_data_h[1] << 8 | rx_data_l[1]);
    return ret;
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
    uint8_t tx_reset[2] = { (uint8_t) (WRITE_OP | REG_PWR_MGMT_1), BIT_H_RESET };
    uint8_t rx_reset[2] = { 0, 0 };

    ret = icm20948_send_message(p_dev, tx_reset, rx_reset);

    // 4. Despertar el sensor: escribir 0x00 en PWR_MGMT_1
    uint8_t tx_wakeup[2] = { (uint8_t) (WRITE_OP | REG_PWR_MGMT_1), START_CONECTION };
    uint8_t rx_wakeup[2] = { 0, 0 };

    ret = icm20948_send_message(p_dev, tx_wakeup, rx_wakeup);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al despertar ICM20948: %d", ret);
        return ret;
    } else {
        ESP_LOGI(TAG, "Sensor despertado (PWR_MGMT_1 = 0x00).");
        vTaskDelay(pdMS_TO_TICKS(10));  // Breve espera para estabilizar el sensor
        return ret;
    }

    // 5. Leer el registro WHO_AM_I: mensaje vacío por ser lectura
    uint8_t tx_data_who[2] = { (uint8_t)(READ_OP | REG_WHO_AM_I), EMPTY_MESSAGE };
    uint8_t rx_data_who[2] = { 0, 0 };

    ret = icm20948_send_message(p_dev, tx_data_who, rx_data_who);

    p_dev->who_am_i = rx_data_who[1];  // El dato real viene en el segundo byte
    ESP_LOGI(TAG, "WHO_AM_I leído: 0x%02X", p_dev->who_am_i);

    return ESP_OK;
}


esp_err_t icm20948_get_measurements(data_t *p_dev) {
    esp_err_t ret = ESP_OK;

    //Todos los datos vienen en 16 bits, por lo que get_data() hace 2 lecturas y devuelve la suma binaria
    
    int16_t x_accel;
    ret = get_data(p_dev, 0x2D, 0x2E, &x_accel);

    ESP_LOGI(TAG, "Aceleración leída en X: %d", x_accel);

    int16_t y_accel;
    ret = get_data(p_dev, 0x2F, 0x30, &y_accel);

    ESP_LOGI(TAG, "Aceleración leída en Y: %d", y_accel);

    //Aqui irán el resto de extracciones de datos

    return ret;
}

