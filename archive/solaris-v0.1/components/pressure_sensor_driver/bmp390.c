#include "bmp390.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "BMP390"

//-----------------------------PINS (Hardware)-----------------------------
#define PIN_NUM_MISO 47 // CIPO
#define PIN_NUM_MOSI 38 // COPI
#define PIN_NUM_CLK  48 // SCK
#define PIN_NUM_CS   18 // CS

//-----------------------------INIT (8 dummy bits & halfduplex)-----------------------------
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
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error initialazing SPI bus: %d", ret);
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 500 * 1000,  //SPI clock frequency: 500 kHz
        .mode = 0,                     //SPI mode 0 (CPOL = 0, CPHA = 0)
        .spics_io_num = PIN_NUM_CS,    //Chip Select (CS) GPIO pin
        .queue_size = 7,               //Transaction queue depth (max pending transactions)
        .command_bits = 8,             //Number of bits used for command phase (register + R/W bit)
        .dummy_bits = 8,               //Number of dummy cycles (needed for read operations)
        .flags = SPI_DEVICE_HALFDUPLEX //Configure device in half-duplex mode
    };

    ret = spi_bus_add_device(SPI3_HOST, &devcfg, handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error adding the SPI device: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Correct SPI init");
    return ESP_OK;
}

//-----------------------------AUX-----------------------------
esp_err_t bmp390_read(spi_device_handle_t handle, uint8_t reg, uint8_t *dst, size_t len)
{
    spi_transaction_t t = {0};
    t.cmd = reg | 0x80;   //MSB=1 + direction
    t.length = 0;         //TX data phase: 0 bits
    t.rxlength = len * 8; //RX data phase: len bytes * 8 bits
    t.rx_buffer = dst;

    return spi_device_polling_transmit(handle, &t); //Transaction (choose polling or transmit)
}

esp_err_t bmp390_write_reg(spi_device_handle_t handle, uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2] = {(reg & 0x7F), value};
    spi_transaction_t t = {0};
    t.length = 16; //16 bits (8 bits direction + 8 bits value)
    t.tx_buffer = tx_data;

    return spi_device_transmit(handle, &t); //Transaction (choose polling or transmit)
}

//-----------------------------CONFIG & CHECK-----------------------------
esp_err_t bmp390_soft_reset(spi_device_handle_t handle)
{
    esp_err_t ret = bmp390_write_reg(handle, BMP390_SOFT_RESET_REG, BMP390_SOFT_RESET_CMD);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error sending soft reset: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Soft reset sent, wainting 100 ms");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return ESP_OK;
}

esp_err_t bmp390_enable_spi_mode(spi_device_handle_t handle)
{
    esp_err_t ret = bmp390_write_reg(handle, BMP390_IF_CONF_REG, BMP390_IF_CONF_SPI);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error enabling modo SPI: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "SPI mode enabled (IF_CONF=0x%02X)", BMP390_IF_CONF_SPI);
    return ESP_OK;
}

esp_err_t bmp390_read_if_conf(spi_device_handle_t handle, uint8_t *if_conf)
{
    esp_err_t ret = bmp390_read(handle, BMP390_IF_CONF_REG, if_conf, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading IF_CONF: %d", ret);
    }
    else
    {
        ESP_LOGI(TAG, "IF_CONF read: 0x%02X", *if_conf);
    }
    return ret;
}

esp_err_t bmp390_read_chip_id(spi_device_handle_t handle, uint8_t *chip_id)
{
    esp_err_t ret = bmp390_read(handle, BMP390_CHIP_ID_REG, chip_id, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading CHIP ID: %d", ret);
    }
    else
    {
        ESP_LOGI(TAG, "CHIP ID read: 0x%02X", *chip_id);
    }
    return ret;
}

//-------------------------PREPARE READ-----------------------------
//Mode
esp_err_t bmp390_set_mode_normal(spi_device_handle_t handle)
{
    return bmp390_write_reg(handle, BMP390_REG_PWRCTRL, BMP390_VALUE_PWRCTRL);
}

//Oversampling
esp_err_t bmp390_set_osr_temp(spi_device_handle_t handle)
{
    return bmp390_write_reg(handle, BMP390_REG_OSR, BMP390_VALUE_OSR);
}

//ODR
esp_err_t bmp390_set_odr(spi_device_handle_t handle)
{
    return bmp390_write_reg(handle, BMP390_REG_ODR, BMP390_VALUE_ODR);
}

//Filter
esp_err_t bmp390_set_iir(spi_device_handle_t handle)
{
    return bmp390_write_reg(handle, BMP390_REG_IIR, BMP390_VALUE_IIR);
}

//Status
esp_err_t bmp390_read_status(spi_device_handle_t handle, uint8_t *status)
{
    return bmp390_read(handle, BMP390_REG_STATUS, status, 1);
}

esp_err_t bmp390_wait_temp_ready(spi_device_handle_t handle)
{
    uint8_t st = 0;
    esp_err_t ret;
    //Read STATUS until bit DRDY_TEMP (0x40) = 1
    do
    {
        ret = bmp390_read_status(handle, &st);
        if (ret != ESP_OK)
        {
            return ret;
        }
    } while ((st & BMP390_STATUS_DRDY_TEMP) == 0);

    return ESP_OK;
}

esp_err_t bmp390_wait_press_ready(spi_device_handle_t handle)
{
    uint8_t st;
    esp_err_t ret;
    //Read STATUS until bit DRDY_PRESS (0x40) = 1
    do
    {
        ret = bmp390_read_status(handle, &st);
        if (ret != ESP_OK)
            return ret;
    } while ((st & BMP390_STATUS_DRDY_PRES) == 0);
    return ESP_OK;
}

//-----------------------------READ TEMP-----------------------------
esp_err_t bmp390_read_raw_temp_coeffs(spi_device_handle_t handle, bmp390_temp_calib_t *tcalib)
{
    uint8_t raw[5]; //5 bytes: 0x31..0x35
    esp_err_t ret;
    //Read 5 bytes of coefficients in a burst
    ret = bmp390_read(handle, BMP390_TEMP_CALIB_REG_START, raw, sizeof(raw));
    if (ret != ESP_OK)
    {
        return ret;
    }
    //Unpack in little-endian
    tcalib->par_t1 = (uint16_t)(raw[1] << 8 | raw[0]);
    tcalib->par_t2 = (int16_t)(raw[3] << 8 | raw[2]);
    tcalib->par_t3 = (int8_t)raw[4];
    //Initialize t_lin
    tcalib->t_lin = 0.0f;

    return ESP_OK;
}

esp_err_t bmp390_calibrate_temp_params(spi_device_handle_t handle, bmp390_temp_params_t *out)
{
    esp_err_t ret;
    bmp390_temp_calib_t raw;
    //Read Raw Coeffs
    ret = bmp390_read_raw_temp_coeffs(handle, &raw);
    if (ret != ESP_OK)
    {
        return ret;
    }
    //Calibrate as shown in the datasheet (or see in bmp390.h: bmp390_temp_params_t)
    out->PAR_T1 = raw.par_t1 * 256.0f;
    out->PAR_T2 = raw.par_t2 / 1073741824.0f;
    out->PAR_T3 = raw.par_t3 / 281474976710656.0f;

    return ESP_OK;
}


esp_err_t bmp390_read_raw_temp(spi_device_handle_t handle, uint32_t *raw_temp)
{
    uint8_t buf[3];
    esp_err_t ret;
    //Read 3 bytes of raw temp in a burst (regs 0x07..0x09)
    ret = bmp390_read(handle, BMP390_TEMP_RAW_REG, buf, sizeof(buf));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading raw_temp: %d", ret);
        return ret;
    }
    //Combine XLSB, LSB and MSB
    *raw_temp = ((uint32_t)buf[2] << 16)  // MSB
                | ((uint32_t)buf[1] << 8) // LSB
                | (uint32_t)buf[0];       // XLSB

    return ESP_OK;
}

float bmp390_compensate_temperature(uint32_t raw_temp, bmp390_temp_params_t *params)
{
    //Compensate as shown in the datasheet
    float partial1 = (float)raw_temp - params->PAR_T1;
    float partial2 = partial1 * params->PAR_T2;
    float t_lin = partial2 + (partial1 * partial1) * params->PAR_T3;

    return t_lin;
}

//-----------------------------READ PRESS-----------------------------
esp_err_t bmp390_read_raw_press_coeffs(spi_device_handle_t handle, bmp390_press_calib_t *pcalib)
{
    uint8_t raw[16];
    //Read 16 bytes of coefficients in a burst
    esp_err_t ret = bmp390_read(handle, BMP390_PRESS_CALIB_REG_START, raw, sizeof(raw));
    if (ret != ESP_OK)
    {
        return ret;
    }
    //Unpuck in little endian
    pcalib->par_p1 = (uint16_t)((raw[1] << 8) | raw[0]);
    pcalib->par_p2 = (uint16_t)((raw[3] << 8) | raw[2]);
    pcalib->par_p3 = (int8_t)raw[4];
    pcalib->par_p4 = (int8_t)raw[5];
    pcalib->par_p5 = (uint16_t)((raw[7] << 8) | raw[6]);
    pcalib->par_p6 = (uint16_t)((raw[9] << 8) | raw[8]);
    pcalib->par_p7 = (int8_t)raw[10];
    pcalib->par_p8 = (int8_t)raw[11];
    pcalib->par_p9 = (int16_t)((raw[13] << 8) | raw[12]);
    pcalib->par_p10 = (int8_t)raw[14];
    pcalib->par_p11 = (int8_t)raw[15];

    return ESP_OK;
}

esp_err_t bmp390_calibrate_press_params(spi_device_handle_t handle, bmp390_press_params_t *out)
{
    esp_err_t ret;
    bmp390_press_calib_t raw;

    //Read Raw Coeffs
    ret = bmp390_read_raw_press_coeffs(handle, &raw);
    if (ret != ESP_OK)
    {
        return ret;
    }
    //Calibrate as shown in datasheet (or see in bmp390.h: bmp390_press_params_t )
    out->PAR_P1 = (raw.par_p1 - 16384.0f) / 1048576.0f;
    out->PAR_P2 = (raw.par_p2 - 16384.0f) / 536870912.0f;
    out->PAR_P3 = raw.par_p3 / 4294967296.0f;
    out->PAR_P4 = raw.par_p4 / 137438953472.0f;
    out->PAR_P5 = raw.par_p5 * 8.0f;
    out->PAR_P6 = raw.par_p6 / 64.0f;
    out->PAR_P7 = raw.par_p7 / 256.0f;
    out->PAR_P8 = raw.par_p8 / 32768.0f;
    out->PAR_P9 = raw.par_p9 / 281474976710656.0f;
    out->PAR_P10 = raw.par_p10 / 281474976710656.0f;
    out->PAR_P11 = raw.par_p11 / 36893488147419103232.0f;

    return ESP_OK;
}

esp_err_t bmp390_read_raw_press(spi_device_handle_t handle, uint32_t *raw_press)
{
    uint8_t buf[3];
    esp_err_t ret;

    //Read 3 bytes of raw temp in a burst (regs 0x04..0x06)
    ret = bmp390_read(handle, BMP390_PRESS_RAW_REG, buf, sizeof(buf));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading raw_press: %d", ret);
        return ret;
    }
    //Combine XLSB, LSB and MSB
    *raw_press = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];

    return ESP_OK;
}

float bmp390_compensate_pressure(uint32_t raw_press, float t_lin, bmp390_press_params_t *p)
{
    //Compensate as shown in the datasheet
    float partial_data1, partial_data2, partial_data3, partial_data4;
    float partial_out1, partial_out2;
    float comp_press;

    partial_data1 = p->PAR_P6 * t_lin;
    partial_data2 = p->PAR_P7 * (t_lin * t_lin);
    partial_data3 = p->PAR_P8 * (t_lin * t_lin * t_lin);
    partial_out1 = p->PAR_P5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = p->PAR_P2 * t_lin;
    partial_data2 = p->PAR_P3 * (t_lin * t_lin);
    partial_data3 = p->PAR_P4 * (t_lin * t_lin * t_lin);
    partial_out2 = raw_press * (p->PAR_P1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = raw_press * raw_press;
    partial_data2 = p->PAR_P9 + p->PAR_P10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + (raw_press * raw_press * raw_press) * p->PAR_P11;

    comp_press = partial_out1 + partial_out2 + partial_data4;

    return comp_press;
}

//BMP390_C