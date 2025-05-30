#include "bmp390.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "BMP390"

// Pines según la conexión física
#define PIN_NUM_MISO 47  // CIPO
#define PIN_NUM_MOSI 38  // COPI
#define PIN_NUM_CLK  48  // SCK
#define PIN_NUM_CS   18  // CS

/**
 * @brief Inicializa el bus SPI y añade el dispositivo BMP390.
 *
 * Se configura con un reloj de 500 kHz, utilizando SPI3_HOST y SPI_DMA_CH_AUTO.
 * Además se establecen 8 bits de comando y 8 bits dummy para que la lectura de registros se realice
 * en una única transacción SPI.
 *
 * @param handle Puntero en el que se devolverá el handle del dispositivo SPI.
 * @return esp_err_t ESP_OK si todo va bien, o error en caso contrario.
 */
esp_err_t bmp390_init(spi_device_handle_t *handle)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al inicializar el bus SPI: %d", ret);
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 500 * 1000,  // 500 kHz
        .mode = 0,                   // SPI modo 0: CPOL=0, CPHA=0
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .command_bits = 8,           // 8 bits para la instrucción (registro con bit de lectura)
        .dummy_bits = 8,             // 8 bits dummy para leer la respuesta
        // address_bits se deja en 0 ya que no se utiliza en esta configuración
    };

    ret = spi_bus_add_device(SPI3_HOST, &devcfg, handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al añadir el dispositivo SPI: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "SPI inicializado correctamente");
    return ESP_OK;
}

/**
 * @brief Realiza un soft reset del BMP390.
 *
 * Envía el comando 0xB6 al registro 0x7E y espera 100 ms.
 *
 * @param handle Handle del dispositivo SPI.
 * @return esp_err_t ESP_OK si se realizó correctamente, u otro error.
 */
esp_err_t bmp390_soft_reset(spi_device_handle_t handle)
{
    uint8_t tx_data[2] = { BMP390_SOFT_RESET_REG & 0x7F, BMP390_SOFT_RESET_CMD };
    // Aquí usamos la transacción tradicional (sin command_bits) para el soft reset
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;  // 16 bits
    t.tx_buffer = tx_data;

    esp_err_t ret = spi_device_transmit(handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al enviar soft reset: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Soft reset enviado, esperando 100 ms");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return ESP_OK;
}

/**
 * @brief Activa el modo SPI del BMP390 escribiendo en el registro IF_CONF (0x1A).
 *
 * Se escribe el valor 0x01 en IF_CONF para poner el bit "spi3" a 1, lo que habilita el modo SPI.
 *
 * @param handle Handle del dispositivo SPI.
 * @return esp_err_t ESP_OK si la operación fue exitosa, u otro error en caso de fallo.
 */
esp_err_t bmp390_enable_spi_mode(spi_device_handle_t handle)
{
    // Para escribir usamos la transacción tradicional (sin utilizar command_bits)
    uint8_t tx_data[2] = { BMP390_IF_CONF_REG & 0x7F, BMP390_IF_CONF_SPI };
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.tx_buffer = tx_data;

    esp_err_t ret = spi_device_transmit(handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al habilitar modo SPI: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Modo SPI activado correctamente (IF_CONF=0x%02X)", BMP390_IF_CONF_SPI);
    return ESP_OK;
}

/**
 * @brief Lee un registro del BMP390 utilizando la nueva configuración de SPI.
 *
 * Se utiliza el campo cmd de la transacción: se pone el registro a leer con el bit de lectura (0x80) y el driver
 * se encarga de enviar 8 bits dummy para recibir el dato.
 *
 * @param handle Handle del dispositivo SPI.
 * @param reg Registro a leer.
 * @param val Puntero donde se almacenará el valor leído (8 bits).
 * @return esp_err_t ESP_OK si la operación fue exitosa, u otro error en caso de fallo.
 */
esp_err_t bmp390_read_reg(spi_device_handle_t handle, uint8_t reg, uint8_t *val)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    // Se usa el campo cmd para enviar el registro a leer con el bit de lectura activado:
    t.cmd = reg | 0x80;
    t.length = 8;      // Se esperan 8 bits (dummy) para la lectura
    t.rxlength = 8;    // Se esperan 8 bits de respuesta
    t.rx_buffer = val;
    return spi_device_polling_transmit(handle, &t);
}

/**
 * @brief Lee el registro IF_CONF del BMP390.
 *
 * @param handle Handle del dispositivo SPI.
 * @param if_conf Puntero donde se almacenará el valor leído del registro IF_CONF.
 * @return esp_err_t ESP_OK si la operación fue exitosa, u otro error.
 */
esp_err_t bmp390_read_if_conf(spi_device_handle_t handle, uint8_t *if_conf)
{
    esp_err_t ret = bmp390_read_reg(handle, BMP390_IF_CONF_REG, if_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer IF_CONF: %d", ret);
    } else {
        ESP_LOGI(TAG, "IF_CONF leído: 0x%02X", *if_conf);
    }
    return ret;
}

/**
 * @brief Lee el valor del registro CHIP ID del BMP390.
 *
 * @param handle Handle del dispositivo SPI.
 * @param chip_id Puntero donde se almacenará el valor leído.
 * @return esp_err_t ESP_OK si la operación fue exitosa, u otro error.
 */
esp_err_t bmp390_read_chip_id(spi_device_handle_t handle, uint8_t *chip_id)
{
    esp_err_t ret = bmp390_read_reg(handle, BMP390_CHIP_ID_REG, chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer CHIP ID: %d", ret);
    } else {
        ESP_LOGI(TAG, "CHIP ID leído: 0x%02X", *chip_id);
    }
    return ret;
}

/**
 * @brief Lee el registro DATA5 (0x09) del BMP390.
 *
 * @param handle Handle del dispositivo SPI.
 * @param data Puntero donde se almacenará el valor leído.
 * @return esp_err_t ESP_OK si la operación fue exitosa, u otro error.
 */
esp_err_t bmp390_read_data5(spi_device_handle_t handle, uint8_t *data)
{
    esp_err_t ret = bmp390_read_reg(handle, BMP390_DATA5_REG, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer DATA5: %d", ret);
    } else {
        ESP_LOGI(TAG, "DATA5 leído: 0x%02X", *data);
    }
    return ret;
}

// —— A PARTIR DE AQUÍ, LO AÑADIDO SIN REDEFINIR NADA MÁS ——

// Bajo nivel: escribir registro
esp_err_t bmp390_write_reg(spi_device_handle_t spi, uint8_t reg_addr, uint8_t data)
{
    spi_transaction_t t = {
        .flags   = SPI_TRANS_USE_TXDATA,
        .tx_data = { reg_addr & 0x7F, data },
        .length  = 16,
    };
    return spi_device_transmit(spi, &t);
}

// Variables internas para calibración de temperatura
static float _par_t1, _par_t2, _par_t3;

// Burst-read de coeficientes de temperatura (0x30…0x35)
esp_err_t bmp390_read_calibration(spi_device_handle_t spi)
{
    uint8_t buf[6];
    for (int i = 0; i < 6; i++) {
        esp_err_t r = bmp390_read_reg(spi, 0x30 + i, &buf[i]);
        if (r != ESP_OK) return r;
    }
    uint16_t raw_t1 = ((uint16_t)buf[1] << 8) | buf[0];
    uint16_t raw_t2 = ((uint16_t)buf[3] << 8) | buf[2];
    int8_t  raw_t3 = (int8_t)buf[5];
    _par_t1 = raw_t1 / 256.0f;               // 2^8
    _par_t2 = raw_t2 / 1073741824.0f;        // 2^30 :contentReference[oaicite:1]{index=1}
    _par_t3 = raw_t3 / 281474976710656.0f;
    ESP_LOGI(TAG, "Calib T: t1=%.6f t2=%.6f t3=%.12f", _par_t1, _par_t2, _par_t3);
    return ESP_OK;
}

// Lectura cruda de temperatura (3 bytes DATA_3…DATA_5)
esp_err_t bmp390_read_raw_temperature(spi_device_handle_t spi, uint32_t *uncomp_temp)
{
    uint8_t raw[3];
    for (int i = 0; i < 3; i++) {
        esp_err_t r = bmp390_read_reg(spi, BMP390_REG_DATA_3 + i, &raw[i]);
        if (r != ESP_OK) return r;
    }
    // Convert 24-bit register data to 20-bit ADC value by discarding 4 LSBs
     uint32_t u24 = ((uint32_t)raw[2] << 16)
                  | ((uint32_t)raw[1] <<  8)
                  |  (uint32_t)raw[0];
     *uncomp_temp = u24 >> 4;  // now a true 20-bit uncomp_temp
    return ESP_OK;
}

// Compensación de temperatura en °C (datasheet §3.10)
float BMP390_compensate_temperature(uint32_t uncomp_temp)
{
    float pd1 = (float)uncomp_temp - _par_t1;
    float pd2 = pd1 * _par_t2;
    return pd2 + pd1 * pd1 * _par_t3;
}