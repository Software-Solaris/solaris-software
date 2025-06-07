#include "bmp390.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "BMP390"

// ------------------Pines según la conexión física------------------

#define PIN_NUM_MISO 47  // CIPO
#define PIN_NUM_MOSI 38  // COPI
#define PIN_NUM_CLK  48  // SCK
#define PIN_NUM_CS   18  // CS

//--------------------Inicialización (8 bits dummy y halfduplex)---------------------------

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
        .flags          = SPI_DEVICE_HALFDUPLEX
    };

    ret = spi_bus_add_device(SPI3_HOST, &devcfg, handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al añadir el dispositivo SPI: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "SPI inicializado correctamente");
    return ESP_OK;
}

esp_err_t bmp390_write_reg(spi_device_handle_t handle, uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2] = { (reg & 0x7F), value };
    spi_transaction_t t = { 0 };
    t.length    = 16;          // 16 bits (8 bits registro + 8 bits dato)
    t.tx_buffer = tx_data;

    return spi_device_transmit(handle, &t);
}

esp_err_t bmp390_soft_reset(spi_device_handle_t handle)
{
    esp_err_t ret = bmp390_write_reg(handle, BMP390_SOFT_RESET_REG, BMP390_SOFT_RESET_CMD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al enviar soft reset: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Soft reset enviado, esperando 100 ms");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return ESP_OK;
}

esp_err_t bmp390_enable_spi_mode(spi_device_handle_t handle)
{
    esp_err_t ret = bmp390_write_reg(handle, BMP390_IF_CONF_REG, BMP390_IF_CONF_SPI);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al habilitar modo SPI: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Modo SPI activado correctamente (IF_CONF=0x%02X)", BMP390_IF_CONF_SPI);
    return ESP_OK;
}

esp_err_t bmp390_read_reg(spi_device_handle_t handle, uint8_t reg, uint8_t *val)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // 1. Comando: registro con bit de lectura (MSB = 1)
    t.cmd = reg | 0x80;

    // 2. fase de datos: no se envía nada en TX
    t.length   = 0;     // 0 bits de TX en fase de datos

    // 3. fase de datos: se leen 8 bits en RX
    t.rxlength = 8;     // 8 bits a recibir
    t.rx_buffer = val;  // val debe apuntar a un uint8_t

    return spi_device_polling_transmit(handle, &t);
}


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

//-------------------------Activar Lecturas-----------------------------

//Modo
esp_err_t bmp390_set_mode_normal(spi_device_handle_t handle)
{
    // Escritura directa del valor pre-combinado (modo Normal + temp + press)
    return bmp390_write_reg(handle, BMP390_REG_PWR_CTRL, BMP390_PWRCTRL_NORMAL);
}

//Oversampling
esp_err_t bmp390_set_osr_temp(spi_device_handle_t handle)
{
    return bmp390_write_reg(handle, BMP390_REG_OSR, BMP390_OSR_TEMP_x16);
}

//ODR
esp_err_t bmp390_set_odr_50hz(spi_device_handle_t handle)
{
    return bmp390_write_reg(handle, BMP390_REG_ODR, BMP390_ODR_SEL_50HZ);
}

//Filtro
esp_err_t bmp390_set_iir(spi_device_handle_t handle)
{
    return bmp390_write_reg(handle, BMP390_REG_CONFIG, BMP390_IIR_COEFF_7);
}

//----------------------------Lectura Temperatura--------------------------------------------

esp_err_t bmp390_read_bytes(spi_device_handle_t handle, uint8_t start_reg, uint8_t *buffer,size_t length)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // En modo half-duplex (devcfg.flags = SPI_DEVICE_HALFDUPLEX, command_bits=8, dummy_bits=8):
    t.cmd       = start_reg | 0x80;  // 8 bits de comando con bit lectura
    t.length    = 0;                 // 0 bits TX en fase de datos
    t.rxlength  = length * 8;        // recibimos 'length' bytes
    t.rx_buffer = buffer;

    return spi_device_polling_transmit(handle, &t);
}

esp_err_t bmp390_read_temp_calibration(spi_device_handle_t handle, bmp390_temp_calib_t *tcalib)
{
    uint8_t raw[5];  // 5 bytes: 0x31..0x35
    esp_err_t ret;

    // 1) Leemos de 0x31 a 0x35 (5 bytes consecutivos)
    ret = bmp390_read_bytes(handle, BMP390_TEMP_CALIB_REG_START, raw, sizeof(raw));
    if (ret != ESP_OK) {
        return ret;
    }

    // 2) Desempaquetamos en little-endian:
    tcalib->par_t1 =  (uint16_t)( raw[1] << 8 | raw[0] );
    tcalib->par_t2 =  (int16_t )( raw[3] << 8 | raw[2] );
    tcalib->par_t3 =  (int8_t  ) raw[4];

    // 3) Inicializamos t_lin a cero
    tcalib->t_lin = 0.0f;

    return ESP_OK;
}

esp_err_t bmp390_temp_params(spi_device_handle_t handle, bmp390_temp_params_t *out)
{
    esp_err_t ret;
    bmp390_temp_calib_t raw;

    // 1) Leemos los valores “raw” desde el BMP390
    ret = bmp390_read_temp_calibration(handle, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    // 2) Convertimos a float según la sección 8.4 de la datasheet:
    //    PAR_T1 = raw.par_t1 / 2^8
    //    PAR_T2 = raw.par_t2 / 2^30
    //    PAR_T3 = raw.par_t3 / 2^48

    out->PAR_T1 = (float) raw.par_t1 / (float)(1 << 8);
    out->PAR_T2 = (float) raw.par_t2 / (float)(1ULL << 30);
    out->PAR_T3 = (float) raw.par_t3 / (float)(1ULL << 48);

    return ESP_OK;
}

esp_err_t bmp390_read_raw_temp(spi_device_handle_t handle, uint32_t *raw_temp)
{
    uint8_t buf[3];
    esp_err_t ret;

    // 1) Leer en ráfaga los 3 bytes de temperatura cruda (0x07..0x09)
    ret = bmp390_read_bytes(handle, 0x07, buf, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer raw_temp: %d", ret);
        return ret;
    }

    // 2) Combinar XLSB, LSB y MSB en un valor de 24 bits:
    //    buf[0] = TEMP_XLSB (bits 7:0)
    //    buf[1] = TEMP_LSB  (bits 15:8)
    //    buf[2] = TEMP_MSB  (bits 23:16)
    *raw_temp = ((uint32_t)buf[2] << 16)  // MSB
              | ((uint32_t)buf[1] <<  8)  // LSB
              |  (uint32_t)buf[0];        // XLSB

    return ESP_OK;
}

float bmp390_compensate_temperature(uint32_t raw_temp, bmp390_temp_params_t *params)
{
    // Según sección 8.4 de la datasheet:
    //   PAR_T1 = stored_par_t1 / 2^8
    //   PAR_T2 = stored_par_t2 / 2^30
    //   PAR_T3 = stored_par_t3 / 2^48
    //
    //   t_lin = PAR_T2 * (raw_temp - PAR_T1)
    //         + PAR_T3 * (raw_temp - PAR_T1)^2

    float partial1 = (float)raw_temp - params->PAR_T1;
    float partial2 = partial1 * params->PAR_T2;
    float t_lin = partial2 + (partial1 * partial1) * params->PAR_T3;

    return t_lin;
}