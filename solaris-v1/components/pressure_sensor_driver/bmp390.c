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

esp_err_t bmp390_read(spi_device_handle_t handle, uint8_t reg, uint8_t *dst, size_t len)
{
    spi_transaction_t t = { 0 };

    // 1) Prepara el comando de lectura: MSB=1 + dirección
    t.cmd       = reg | 0x80;

    // 2) Fase de datos TX: 0 bits
    t.length    = 0;

    // 3) Fase de datos RX: len bytes * 8 bits/byte
    t.rxlength  = len * 8;
    t.rx_buffer = dst;

    // 4) Ejecuta la transacción (polling o puedes usar transmit según convenga)
    return spi_device_polling_transmit(handle, &t);
}


esp_err_t bmp390_read_if_conf(spi_device_handle_t handle, uint8_t *if_conf)
{
    esp_err_t ret = bmp390_read(handle, BMP390_IF_CONF_REG, if_conf, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer IF_CONF: %d", ret);
    } else {
        ESP_LOGI(TAG, "IF_CONF leído: 0x%02X", *if_conf);
    }
    return ret;
}

esp_err_t bmp390_read_chip_id(spi_device_handle_t handle, uint8_t *chip_id)
{
    esp_err_t ret = bmp390_read(handle, BMP390_CHIP_ID_REG, chip_id, 1);
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

//Filtro
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

    // Leer STATUS hasta que el bit DRDY_TEMP (0x40) esté a 1
    do {
        ret = bmp390_read_status(handle, &st);
        if (ret != ESP_OK) {
            return ret;
        }
    } while ((st & BMP390_STATUS_DRDY_TEMP) == 0);

    return ESP_OK;
}

esp_err_t bmp390_wait_press_ready(spi_device_handle_t handle)
{
    uint8_t st;
    esp_err_t ret;
    do {
        ret = bmp390_read_status(handle, &st);
        if (ret != ESP_OK) return ret;
    } while ((st & BMP390_STATUS_DRDY_PRES) == 0);
    return ESP_OK;
}

//----------------------------Lectura Temperatura--------------------------------------------

esp_err_t bmp390_read_raw_temp_coeffs(spi_device_handle_t handle, bmp390_temp_calib_t *tcalib)
{
    uint8_t raw[5];  // 5 bytes: registros 0x31..0x35
    esp_err_t ret;

    // 1) Leer en ráfaga 5 bytes de coeficientes
    ret = bmp390_read(handle, BMP390_TEMP_CALIB_REG_START, raw, sizeof(raw));
    if (ret != ESP_OK) {
        return ret;
    }

    // 2) Desempaquetar en little-endian
    tcalib->par_t1 =  (uint16_t)(raw[1] << 8 | raw[0]);
    tcalib->par_t2 =  (int16_t )(raw[3] << 8 | raw[2]);
    tcalib->par_t3 =  (int8_t  ) raw[4];

    // 3) Inicializar t_lin
    tcalib->t_lin = 0.0f;

    return ESP_OK;
}

esp_err_t bmp390_calibrate_temp_params(spi_device_handle_t handle, bmp390_temp_params_t *out)
{
    esp_err_t ret;
    bmp390_temp_calib_t raw;

    // 1) Leer los coeficientes crudos
    ret = bmp390_read_raw_temp_coeffs(handle, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    //    PAR_T1 = raw.par_t1 / 2^8
    //    PAR_T2 = raw.par_t2 / 2^30
    //    PAR_T3 = raw.par_t3 / 2^48

    out->PAR_T1 = raw.par_t1 * 256.0f;
    out->PAR_T2 = raw.par_t2 / 1073741824.0f;
    out->PAR_T3 = raw.par_t3 / 281474976710656.0f;

    return ESP_OK;
}


esp_err_t bmp390_read_raw_temp(spi_device_handle_t handle, uint32_t *raw_temp)
{
    uint8_t buf[3];
    esp_err_t ret;

    // 1) Leer en ráfaga los 3 bytes de temperatura cruda (regs 0x07..0x09)
    ret = bmp390_read(handle, BMP390_TEMP_RAW_REG, buf, sizeof(buf));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer raw_temp: %d", ret);
        return ret;
    }

    // 2) Combinar XLSB, LSB y MSB en un valor de 24 bits:
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

////------------------------Lectura Presión--------------------------

esp_err_t bmp390_read_raw_press_coeffs(spi_device_handle_t handle, bmp390_press_calib_t *pcalib)
{
    uint8_t raw[16];
    esp_err_t ret = bmp390_read(handle, BMP390_PRESS_CALIB_REG_START, raw, sizeof(raw));
    if (ret != ESP_OK) {
        return ret;
    }

    // PAR_P1, P2: signed 16-bit
    pcalib->par_p1  = (uint16_t)((raw[1] << 8) | raw[0]);
    pcalib->par_p2  = (uint16_t)((raw[3] << 8) | raw[2]);
    // PAR_P3, P4: signed 8-bit
    pcalib->par_p3  = (int8_t)  raw[4];
    pcalib->par_p4  = (int8_t)  raw[5];
    // PAR_P5, P6: unsigned 16-bit
    pcalib->par_p5  = (uint16_t)((raw[7] << 8) | raw[6]);
    pcalib->par_p6  = (uint16_t)((raw[9] << 8) | raw[8]);
    // PAR_P7, P8: signed 8-bit
    pcalib->par_p7  = (int8_t)  raw[10];
    pcalib->par_p8  = (int8_t)  raw[11];
    // PAR_P9: signed 16-bit
    pcalib->par_p9  = (int16_t)((raw[13] << 8) | raw[12]);
    // PAR_P10, P11: signed 8-bit
    pcalib->par_p10 = (int8_t)  raw[14];
    pcalib->par_p11 = (int8_t)  raw[15];

    return ESP_OK;
}



esp_err_t bmp390_calibrate_press_params(spi_device_handle_t handle, bmp390_press_params_t *out)
{
    esp_err_t ret;
    bmp390_press_calib_t raw;

    // 1) Leer los coeficientes crudos
    ret = bmp390_read_raw_press_coeffs(handle, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    // PAR_P1 = (raw.par_p1  - 2^14) / 2^20
    out->PAR_P1  = (raw.par_p1  - 16384.0f) / 1048576.0f;
    // PAR_P2 = (raw.par_p2  - 2^14) / 2^29
    out->PAR_P2  = (raw.par_p2  - 16384.0f) / 536870912.0f;
    // PAR_P3 = raw.par_p3 / 2^32
    out->PAR_P3  = raw.par_p3 / 4294967296.0f;
    // PAR_P4 = raw.par_p4 / 2^37
    out->PAR_P4  = raw.par_p4 / 137438953472.0f;
    // PAR_P5 = raw.par_p5 / 2^-3
    out->PAR_P5  = raw.par_p5 * 8.0f;
    // PAR_P6 = raw.par_p6 / 2^6
    out->PAR_P6  = raw.par_p6 / 64.0f;
    // PAR_P7 = raw.par_p7 / 2^8
    out->PAR_P7  = raw.par_p7 / 256.0f;
    // PAR_P8 = raw.par_p8 / 2^15
    out->PAR_P8  = raw.par_p8 / 32768.0f;
    // PAR_P9 = raw.par_p9 / 2^48
    out->PAR_P9  = raw.par_p9 / 281474976710656.0f;
    // PAR_P10 = raw.par_p10 / 2^48
    out->PAR_P10 = raw.par_p10 / 281474976710656.0f;
    // PAR_P11 = raw.par_p11 / 2^65
    out->PAR_P11 = raw.par_p11 / 36893488147419103232.0f;

    return ESP_OK;
}

esp_err_t bmp390_read_raw_press(spi_device_handle_t handle, uint32_t *raw_press)
{
    uint8_t buf[3];
    esp_err_t ret;

    // 1) Leer en ráfaga los 3 bytes de presión cruda (regs 0x04..0x06)
    ret = bmp390_read(handle, BMP390_PRESS_RAW_REG, buf, sizeof(buf));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer raw_press: %d", ret);
        return ret;
    }

    // 2) Combinar XLSB, LSB y MSB en un valor de 24 bits:
    *raw_press = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];

    return ESP_OK;
}

float bmp390_compensate_pressure(uint32_t raw_press, float t_lin, bmp390_press_params_t *p)
{
    float partial_data1, partial_data2, partial_data3, partial_data4;
    float partial_out1, partial_out2;
    float comp_press;

    partial_data1 = p->PAR_P6 * t_lin;
    partial_data2 = p->PAR_P7 * (t_lin * t_lin);
    partial_data3 = p->PAR_P8 * (t_lin * t_lin * t_lin);
    partial_out1  = p->PAR_P5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = p->PAR_P2 * t_lin;
    partial_data2 = p->PAR_P3 * (t_lin * t_lin);
    partial_data3 = p->PAR_P4 * (t_lin * t_lin * t_lin);
    partial_out2  = raw_press * (p->PAR_P1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = raw_press * raw_press;                               // pres^2
    partial_data2 = p->PAR_P9 + p->PAR_P10 * t_lin;                     // par_p9 + par_p10*t_lin
    partial_data3 = partial_data1 * partial_data2;                      // pres^2 * (...)
    partial_data4 = partial_data3 + (raw_press * raw_press * raw_press) * p->PAR_P11;   // pres^3 * par_p11

    // 4) Suma de todos los términos
    comp_press = partial_out1 + partial_out2 + partial_data4;

    return comp_press;
}


//----Help for general
esp_err_t bmp390_config(void)
{
    //Config
    ret = bmp390_soft_reset(bmp_spi);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error soft reset: %d", ret);
        return ret;
    }

    ret = bmp390_enable_spi_mode(bmp_spi);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error enable SPI mode: %d", ret);
        return ret;
    }//End config

    // Dejar tiempo al primer ciclo de configuración
    vTaskDelay(pdMS_TO_TICKS(50));

    //Check
    ret = bmp390_read_chip_id(bmp_spi, &id);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error al leer CHIP ID: %d", ret);
        return ret;
    }

    ret = bmp390_read_if_conf(bmp_spi, &ifc);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error al leer IF_CONF: %d", ret);
        return ret;
    }//End check

    return ESP_OK; 
}//End BMP config

esp_err_t bmp390_prepare_mode(void)
{
    ret = bmp390_set_mode_normal(bmp_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error modo normal: %d", ret);
        return;
    }
    ret = bmp390_set_osr_temp(bmp_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error OSR: %d", ret);
        return;
    }
    ret = bmp390_set_odr(bmp_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error ODR: %d", ret);
        return;
    }
    ret = bmp390_set_iir(bmp_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error IIR: %d", ret);
        return;
    }

    // Dejar tiempo al primer dato en Normal mode
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}//End BMP prepare mode

esp_err_t bmp390_prepare_temp(void)
{
    //Raw coeffs
    ret = bmp390_read_raw_temp_coeffs(bmp_spi, &raw_calib);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Coef raw T1=%u, T2=%d, T3=%d",
                 raw_calib.par_t1,
                 raw_calib.par_t2,
                 raw_calib.par_t3);
    } else {
        ESP_LOGE(TAG, "Error al leer coef. calib.: %d", ret);
        return;
    }//End raw coeffs

    //Calib
    ret = bmp390_calibrate_temp_params(bmp_spi, &temp_params);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "PAR_T1 calibrado: %.4f",  temp_params.PAR_T1);
        ESP_LOGI(TAG, "PAR_T2 calibrado: %.6e",  temp_params.PAR_T2);
        ESP_LOGI(TAG, "PAR_T3 calibrado: %.6e",  temp_params.PAR_T3);
    } else {
        ESP_LOGE(TAG, "Error al calibrar params: %d", ret);
        return;
    }//End calib

    return ESP_OK;
}//End prepare temp

esp_err_t bmp390_prepare_press(void)
{
    ret = bmp390_read_raw_press_coeffs(bmp_spi, &raw_press_calib);
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
    }//End raw coeffs

    //----Calibrar parámetros de presión----
    ret = bmp390_calibrate_press_params(bmp_spi, &press_params);
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
    }//End calib

    return ESP_OK;
}//End prepare temp


esp_err_t bmp390_prepare_read(void)
{
    ret = bmp390_prepare_mode();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error preparing mode BMP390: %d", ret);
        return ret;    
    }

    ret = bmp390_prepare_temp();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error preparing temp BMP390: %d", ret);
        return ret;    
    }

    ret = bmp390_prepare_press();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error preparing press BMP390: %d", ret);
        return ret;    
    }
    return ESP_OK;
}//End prepare read


esp_err_t bmp390_read_temp(void)
{
    ret = bmp390_wait_temp_ready(bmp_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error wait temp ready: %d", ret);
        break;
    }

    ret = bmp390_read_raw_temp(bmp_spi, &raw_temp);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Raw temp: %u", raw_temp);

        float comp_temp = bmp390_compensate_temperature(raw_temp, &temp_params);
        ESP_LOGI(TAG, "Temp compensada: %.2f °C", comp_temp);

    } else {
        ESP_LOGE(TAG, "Error read raw temp: %d", ret);
        break;
    }

    return ESP_OK;
}//End read temp

esp_err_t bmp390_calc_altitude(void)
{
    ret = bmp390_wait_press_ready(bmp_spi);
    if (ret != ESP_OK) { 
        ESP_LOGE(TAG, "wait press ready: %d", ret); 
        break; 
    }

    ret = bmp390_read_raw_press(bmp_spi, &raw_press);
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

    return ESP_OK;
}//End calc altitude

esp_err_t bmp390_read()
{
    ret = bmp390_read_temp();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error read temp BMP390: %d", ret);
        return ret;    
    }

    ret = bmp390_calc_altitude();
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error calc altitude BMP390: %d", ret);
        return ret;    
    }

    return ESP_OK;
}