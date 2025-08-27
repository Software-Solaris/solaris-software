#include "bmp390.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char* TAG = "BMP390";

//--------------------INIT (8 dummy bits and halfduplex)---------------------------

esp_err_t bmp390_init(data_t *p_dev)
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
        ESP_LOGE(TAG, "Error spi_bus_initialize on BMP390: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "SPI bus initialized on BMP390");

    // 2. Configura el dispositivo SPI (CS, velocidad, modo, etc.)
    p_dev->devcfg.clock_speed_hz = 500 * 1000; // 500kHz
    p_dev->devcfg.mode = 3;           // Probar en modo 0 si falla
    p_dev->devcfg.spics_io_num = PIN_NUM_CS;
    p_dev->devcfg.queue_size = 7;
    p_dev->devcfg.command_bits = 8;
    p_dev->devcfg.dummy_bits = 8;
    p_dev->devcfg.flags = SPI_DEVICE_HALFDUPLEX;
    vTaskDelay(pdMS_TO_TICKS(100)); // Espera adicional de 100 ms

    ret = spi_bus_add_device(SPI_HOST_USED, &p_dev->devcfg, &p_dev->handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error spi_bus_add_device on BMP390: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "BMP390 added to SPI bus.");

    return ESP_OK;
}

//--------------------AUX FUNCTIONS---------------------------

esp_err_t bmp390_write_reg(data_t *p_dev, uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2] = { (reg & 0x7F), value };
    p_dev->trans_desc.length    = 16;          // 16 bits (8 bits registro + 8 bits dato)
    p_dev->trans_desc.tx_buffer = tx_data;

    return spi_device_transmit(p_dev->handle, &p_dev->trans_desc);
}

esp_err_t bmp390_read(data_t *p_dev, uint8_t reg, uint8_t *dst, size_t len)
{
    // 1) Prepara el comando de lectura: MSB=1 + dirección
    p_dev->trans_desc.cmd       = reg | 0x80;
    // 2) Fase de datos TX: 0 bits
    p_dev->trans_desc.length    = 0;
    // 3) Fase de datos RX: len bytes * 8 bits/byte
    p_dev->trans_desc.rxlength  = len * 8;
    p_dev->trans_desc.rx_buffer = dst;
    // 4) Ejecuta la transacción (polling o transmit según convenga)
    return spi_device_polling_transmit(p_dev->handle, &p_dev->trans_desc);
}

//--------------------CONFIG and CHECK---------------------------

esp_err_t bmp390_soft_reset(data_t *p_dev)
{
    esp_err_t ret = bmp390_write_reg(p_dev, BMP390_SOFT_RESET_REG, BMP390_SOFT_RESET_CMD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al enviar soft reset: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Soft reset enviado, esperando 100 ms");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return ESP_OK;
}

esp_err_t bmp390_enable_spi_mode(data_t *p_dev)
{
    esp_err_t ret = bmp390_write_reg(p_dev, BMP390_IF_CONF_REG, BMP390_IF_CONF_SPI);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al habilitar modo SPI: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Modo SPI activado correctamente (IF_CONF=0x%02X)", BMP390_IF_CONF_SPI);
    return ESP_OK;
}


esp_err_t bmp390_read_if_conf(data_t *p_dev, uint8_t *if_conf)
{
    esp_err_t ret = bmp390_read(p_dev, BMP390_IF_CONF_REG, if_conf, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer IF_CONF: %d", ret);
    } else {
        ESP_LOGI(TAG, "IF_CONF leído: 0x%02X", *if_conf);
    }
    return ret;
}

esp_err_t bmp390_read_chip_id(data_t *p_dev, uint8_t *chip_id)
{
    esp_err_t ret = bmp390_read(p_dev, BMP390_CHIP_ID_REG, chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer CHIP ID: %d", ret);
    } else {
        ESP_LOGI(TAG, "CHIP ID leído: 0x%02X", *chip_id);
    }
    return ret;
}

//--------------------PREPARE READ---------------------------

//Modo
esp_err_t bmp390_set_mode_normal(data_t *p_dev)
{
    ret = bmp390_write_reg(p_dev, BMP390_REG_PWRCTRL, BMP390_VALUE_PWRCTRL);
    if (ret != ESP_OK)
    {
        return ESP_FAIL;
    } else 
    {
        return ESP_OK;
    }
}

//Oversampling
esp_err_t bmp390_set_osr_temp(data_t *p_dev)
{
    ret = bmp390_write_reg(p_dev, BMP390_REG_OSR, BMP390_VALUE_OSR);
    if (ret != ESP_OK)
    {
        return ESP_FAIL;
    } else 
    {
        return ESP_OK;
    }
}

//ODR
esp_err_t bmp390_set_odr(data_t *p_dev)
{
    ret = bmp390_write_reg(p_dev, BMP390_REG_ODR, BMP390_VALUE_ODR);
    if (ret != ESP_OK)
    {
        return ESP_FAIL;
    } else 
    {
        return ESP_OK;
    }
}

//Filtro
esp_err_t bmp390_set_iir(data_t *p_dev)
{
    ret = bmp390_write_reg(p_dev, BMP390_REG_IIR, BMP390_VALUE_IIR);
    if (ret != ESP_OK)
    {
        return ESP_FAIL;
    } else 
    {
        return ESP_OK;
    }
}

//Status
esp_err_t bmp390_read_status(data_t *p_dev, uint8_t *status)
{
    return bmp390_read(p_dev, BMP390_REG_STATUS, status, 1);
}

esp_err_t bmp390_wait_temp_ready(data_t *p_dev)
{
    // Leer STATUS hasta que el bit DRDY_TEMP esté a 1
    do {
        ret = bmp390_read_status(p_dev, &st);
        if (ret != ESP_OK) { 
            ESP_LOGE(TAG, "wait temp ready: %d", ret); 
            break; 
        }
    } while ((st & BMP390_STATUS_DRDY_TEMP) == 0);

    return ESP_OK;
}

esp_err_t bmp390_wait_press_ready(data_t *p_dev)
{
    // Leer STATUS hasta que el bit DRDY_PRESS esté a 1
    do {
        ret = bmp390_read_status(p_dev, &st);
        if (ret != ESP_OK) { 
            ESP_LOGE(TAG, "wait press ready: %d", ret); 
            break; 
        }
    } while ((st & BMP390_STATUS_DRDY_PRES) == 0);
    return ESP_OK;
}

//--------------------READ TEMP---------------------------

esp_err_t bmp390_read_raw_temp_coeffs(data_t *p_dev, bmp390_temp_calib_t *tcalib)
{
    uint8_t raw[5];

    // 1) Leer en ráfaga 5 bytes de coeficientes
    ret = bmp390_read(p_dev, BMP390_TEMP_CALIB_REG_START, raw, sizeof(raw));
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

esp_err_t bmp390_calibrate_temp_params(data_t *p_dev, bmp390_temp_params_t *out)
{
    bmp390_temp_calib_t raw;
    
    ret = bmp390_read_raw_temp_coeffs(p_dev, &raw); //Raw coeffs
    if (ret != ESP_OK) {
        return ret;
    }

    out->PAR_T1 = raw.par_t1 * 256.0f; // PAR_T1 = raw.par_t1 / 2^8
    out->PAR_T2 = raw.par_t2 / 1073741824.0f; // PAR_T3 = raw.par_t3 / 2^48
    out->PAR_T3 = raw.par_t3 / 281474976710656.0f; // PAR_T2 = raw.par_t2 / 2^30

    return ESP_OK;
}

esp_err_t bmp390_read_raw_temp(data_t *p_dev, uint32_t *raw_temp)
{
    uint8_t buf[3];

    // 1) Leer en ráfaga los 3 bytes de temperatura cruda (regs 0x07..0x09)
    ret = bmp390_read(p_dev, BMP390_TEMP_RAW_REG, buf, sizeof(buf));
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
    float partial1 = (float)raw_temp - params->PAR_T1;
    float partial2 = partial1 * params->PAR_T2;
    float t_lin = partial2 + (partial1 * partial1) * params->PAR_T3;

    return t_lin;
}

//--------------------READ PRESS---------------------------

esp_err_t bmp390_read_raw_press_coeffs(data_t *p_dev, bmp390_press_calib_t *pcalib)
{
    uint8_t raw[16];

    esp_err_t ret = bmp390_read(p_dev, BMP390_PRESS_CALIB_REG_START, raw, sizeof(raw));
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

esp_err_t bmp390_calibrate_press_params(data_t *p_dev, bmp390_press_params_t *out)
{
    bmp390_press_calib_t raw;
    ret = bmp390_read_raw_press_coeffs(p_dev, &raw); //Rar coeffs
    if (ret != ESP_OK) {
        return ret;
    }

    out->PAR_P1  = (raw.par_p1  - 16384.0f) / 1048576.0f; // PAR_P1 = (raw.par_p1  - 2^14) / 2^20
    out->PAR_P2  = (raw.par_p2  - 16384.0f) / 536870912.0f; // PAR_P2 = (raw.par_p2  - 2^14) / 2^29
    out->PAR_P3  = raw.par_p3 / 4294967296.0f; // PAR_P3 = raw.par_p3 / 2^32
    out->PAR_P4  = raw.par_p4 / 137438953472.0f; // PAR_P4 = raw.par_p4 / 2^37    
    out->PAR_P5  = raw.par_p5 * 8.0f; // PAR_P5 = raw.par_p5 / 2^-3
    out->PAR_P6  = raw.par_p6 / 64.0f; // PAR_P6 = raw.par_p6 / 2^6
    out->PAR_P7  = raw.par_p7 / 256.0f; // PAR_P7 = raw.par_p7 / 2^8
    out->PAR_P8  = raw.par_p8 / 32768.0f; // PAR_P8 = raw.par_p8 / 2^15
    out->PAR_P9  = raw.par_p9 / 281474976710656.0f; // PAR_P9 = raw.par_p9 / 2^48
    out->PAR_P10 = raw.par_p10 / 281474976710656.0f; // PAR_P10 = raw.par_p10 / 2^48
    out->PAR_P11 = raw.par_p11 / 36893488147419103232.0f; // PAR_P11 = raw.par_p11 / 2^65

    return ESP_OK;
}

esp_err_t bmp390_read_raw_press(data_t *p_dev, uint32_t *raw_press)
{
    uint8_t buf[3];

    // 1) Leer en ráfaga los 3 bytes de presión cruda (regs 0x04..0x06)
    ret = bmp390_read(p_dev, BMP390_PRESS_RAW_REG, buf, sizeof(buf));
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
    partial_data1 = p->PAR_P6 * t_lin;
    partial_data2 = p->PAR_P7 * (t_lin * t_lin);
    partial_data3 = p->PAR_P8 * (t_lin * t_lin * t_lin);
    partial_out1  = p->PAR_P5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = p->PAR_P2 * t_lin;
    partial_data2 = p->PAR_P3 * (t_lin * t_lin);
    partial_data3 = p->PAR_P4 * (t_lin * t_lin * t_lin);
    partial_out2  = raw_press * (p->PAR_P1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = raw_press * raw_press;                               
    partial_data2 = p->PAR_P9 + p->PAR_P10 * t_lin;                     
    partial_data3 = partial_data1 * partial_data2;                      
    partial_data4 = partial_data3 + (raw_press * raw_press * raw_press) * p->PAR_P11;   

    comp_press = partial_out1 + partial_out2 + partial_data4;

    return comp_press;
}

//--------------------AUX FUNCTIONS (GENERAL)---------------------------

void bmp390_config(data_t *p_dev)
{
    ret = bmp390_soft_reset(p_dev);

    ret = bmp390_enable_spi_mode(p_dev);

    vTaskDelay(pdMS_TO_TICKS(50));
    
    ret = bmp390_read_chip_id(p_dev, &id);

    ret = bmp390_read_if_conf(p_dev, &ifc);

}//End BMP config

void bmp390_prepare_mode(data_t *p_dev)
{
    ret = bmp390_set_mode_normal(p_dev);

    ret = bmp390_set_osr_temp(p_dev);

    ret = bmp390_set_odr(p_dev);

    ret = bmp390_set_iir(p_dev);

    vTaskDelay(pdMS_TO_TICKS(50)); // Dejar tiempo al primer dato en Normal mode

}//End BMP prepare mode

void bmp390_prepare_temp(data_t *p_dev)
{
    ret = bmp390_read_raw_temp_coeffs(p_dev, &raw_calib); //Raw coeffs
    ESP_LOGI(TAG, "Coef raw T1=%u, T2=%d, T3=%d",
                raw_calib.par_t1,
                raw_calib.par_t2,
                raw_calib.par_t3);

    
    ret = bmp390_calibrate_temp_params(p_dev, &temp_params); //Calib
    ESP_LOGI(TAG, "PAR_T1 calibrado: %.4f",  temp_params.PAR_T1);
    ESP_LOGI(TAG, "PAR_T2 calibrado: %.6e",  temp_params.PAR_T2);
    ESP_LOGI(TAG, "PAR_T3 calibrado: %.6e",  temp_params.PAR_T3);

}//End prepare temp

void bmp390_prepare_press(data_t *p_dev)
{
    ret = bmp390_read_raw_press_coeffs(p_dev, &raw_press_calib); //Raw coeffs

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

    ret = bmp390_calibrate_press_params(p_dev, &press_params); //Calib
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

}//End prepare press

void bmp390_prepare_read(data_t *p_dev)
{
    bmp390_prepare_mode(p_dev); 

    bmp390_prepare_temp(p_dev); 

    bmp390_prepare_press(p_dev);
}


esp_err_t bmp390_read_temp(data_t *p_dev)
{
    ret = bmp390_wait_temp_ready(p_dev);

    ret = bmp390_read_raw_temp(p_dev, &raw_temp);
    ESP_LOGI(TAG, "Raw temp: %u", raw_temp);

    float comp_temp = bmp390_compensate_temperature(raw_temp, &temp_params);
    ESP_LOGI(TAG, "Temp compensada: %.2f °C", comp_temp);

    return ESP_OK;
}//End read temp

esp_err_t bmp390_calc_altitude(data_t *p_dev)
{
    ret = bmp390_wait_press_ready(p_dev);


    ret = bmp390_read_raw_press(p_dev, &raw_press);

    ESP_LOGI(TAG, "Raw press: %u", raw_press);

    t_lin = bmp390_compensate_temperature(raw_temp, &temp_params);
    float p_pa = bmp390_compensate_pressure(raw_press, t_lin, &press_params);
    ESP_LOGI(TAG, "Presión comp.: %.2f Pa", p_pa);

    float altitude = 44330.0f * (1.0f - powf(p_pa/101325.0f, 1.0f/5.255f));
    ESP_LOGI(TAG, "Altura: %.2f m", altitude);

    // Delay para pruebas (5 s)
    vTaskDelay(pdMS_TO_TICKS(5000));

    return ESP_OK;
}//End calc altitude


void bmp390_read_measurements(data_t *p_dev)
{
    ret = bmp390_read_temp(p_dev); //Temp
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error read temp BMP390: %d", ret);

    }

    ret = bmp390_calc_altitude(p_dev); //Press and Alt
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Error calc altitude BMP390: %d", ret);  
    }
}//End BMP read