#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "core/returntypes.h"
#include "core/core.h"
#include "databank.h"
#include "icm20948.h"
#include "spi.h"
#include "bmp390.h"
#include "task.h"

#include "gpio_int.h"      
#include "driver/gpio.h"  

#include "spp_log.h"

#include <unistd.h>

void app_main(void)
{
    retval_t ret = SPP_ERROR;

    // sleep(5);
    icm_data_t s_icm;

    Core_Init();

    SPP_HAL_SPI_BusInit();

    ret = IcmInit((void*)&s_icm);
    if (ret != SPP_OK){
        return;
    }

    ret = IcmConfigDmpInit((void*)&s_icm);
    if (ret != SPP_OK){
        return;
    }

    spp_uint8_t dbg[2] = {0};

    // USER_CTRL
    dbg[0] = READ_OP | 0x03;
    dbg[1] = EMPTY_MESSAGE;
    SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, dbg, 2);
    SPP_LOGI("DBG", "USER_CTRL = 0x%02X", dbg[1]);

    // PWR_MGMT_1
    dbg[0] = READ_OP | 0x06;
    dbg[1] = EMPTY_MESSAGE;
    SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, dbg, 2);
    SPP_LOGI("DBG", "PWR_MGMT_1 = 0x%02X", dbg[1]);

    // PWR_MGMT_2
    dbg[0] = READ_OP | 0x07;
    dbg[1] = EMPTY_MESSAGE;
    SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, dbg, 2);
    SPP_LOGI("DBG", "PWR_MGMT_2 = 0x%02X", dbg[1]);

    // LP_CONFIG
    dbg[0] = READ_OP | 0x05;
    dbg[1] = EMPTY_MESSAGE;
    SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, dbg, 2);
    SPP_LOGI("DBG", "LP_CONFIG = 0x%02X", dbg[1]);

    // Leer DATA_RDY_STATUS del DMP
    // Bank
    dbg[0] = WRITE_OP | REG_MEM_BANK_SEL;
    dbg[1] = (DMP_DATA_RDY_STATUS >> 8);
    SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, dbg, 2);
    // Addr
    dbg[0] = WRITE_OP | REG_MEM_START_ADDR;
    dbg[1] = (DMP_DATA_RDY_STATUS & 0xFF);
    SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, dbg, 2);
    // Read byte 0
    dbg[0] = READ_OP | REG_MEM_R_W;
    dbg[1] = EMPTY_MESSAGE;
    SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, dbg, 2);
    spp_uint8_t drs_hi = dbg[1];
    // Read byte 1
    dbg[0] = WRITE_OP | REG_MEM_BANK_SEL;
    dbg[1] = ((DMP_DATA_RDY_STATUS + 1) >> 8);
    SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, dbg, 2);
    dbg[0] = WRITE_OP | REG_MEM_START_ADDR;
    dbg[1] = ((DMP_DATA_RDY_STATUS + 1) & 0xFF);
    SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, dbg, 2);
    dbg[0] = READ_OP | REG_MEM_R_W;
    dbg[1] = EMPTY_MESSAGE;
    SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, dbg, 2);
    spp_uint8_t drs_lo = dbg[1];

    SPP_LOGI("DBG", "DATA_RDY_STATUS = 0x%02X%02X", drs_hi, drs_lo);

    // Antes del while, después de los otros debug prints:
    dbg[0] = READ_OP | 0x17;  // I2C_MST_STATUS
    dbg[1] = EMPTY_MESSAGE;
    SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, dbg, 2);
    SPP_LOGI("DBG", "I2C_MST_STATUS = 0x%02X", dbg[1]);

    // También leer los datos del EXT_SLV_SENS (0x3A-0x43) para ver si el mag responde
    spp_uint8_t ext[11] = {0};
    ext[0] = READ_OP | 0x3A;
    for (int j = 1; j < 11; j++) ext[j] = EMPTY_MESSAGE;
    SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, ext, 11);
    SPP_LOGI("DBG", "EXT_SLV: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
            ext[1], ext[2], ext[3], ext[4], ext[5],
            ext[6], ext[7], ext[8], ext[9], ext[10]);

    // IcmGetSensorsData((void*)&s_icm);
    while (true)
    {
        spp_uint8_t data[3] = {0};

        /* 1. Leer INT_STATUS (0x19) — esperar bit DMP_INT1 (0x02) */
        data[0] = READ_OP | REG_INT_STATUS;   /* 0x99 */
        data[1] = EMPTY_MESSAGE;
        ret = SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, data, 2);
        if (ret != SPP_OK) return;

        spp_uint8_t int_status = data[1];

        /* 2. Leer DMP_INT_STATUS (0x18) siempre, como hace el tráfico */
        data[0] = READ_OP | REG_DMP_INT_STATUS;  /* 0x98 */
        data[1] = EMPTY_MESSAGE;
        ret = SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, data, 2);
        if (ret != SPP_OK) return;

        /* 3. Solo si DMP generó INT1, leer el FIFO */
        if (int_status & 0x02)
        {
            data[0] = READ_OP | REG_FIFO_COUNTH;
            data[1] = EMPTY_MESSAGE;
            data[2] = EMPTY_MESSAGE;
            ret = SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, data, 3);
            if (ret != SPP_OK) return;

            spp_uint16_t fifo_count = ((spp_uint16_t)data[1] << 8) | data[2];
            SPP_LOGI("MAIN", "The fifo count is: %d", fifo_count);

            if (fifo_count > 512)
            {
                data[0] = WRITE_OP | REG_FIFO_RST;
                data[1] = 0x1F;
                SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, data, 2);
                data[1] = 0x1E;
                SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, data, 2);
                continue;
            }
            spp_uint16_t num_packets = fifo_count / 28;

            for (spp_uint16_t i = 0; i < num_packets; i++)
            {
                spp_uint8_t fifo_buf[29] = {0};
                fifo_buf[0] = READ_OP | REG_FIFO_R_W;
                ret = SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, fifo_buf, 29);
                if (ret != SPP_OK) return;

                uint16_t header = (fifo_buf[1] << 8) | fifo_buf[2];

                int16_t accel_x = (fifo_buf[3] << 8) | fifo_buf[4];
                int16_t accel_y = (fifo_buf[5] << 8) | fifo_buf[6];
                int16_t accel_z = (fifo_buf[7] << 8) | fifo_buf[8];

                int16_t gyro_x = (fifo_buf[9] << 8) | fifo_buf[10];
                int16_t gyro_y = (fifo_buf[11] << 8) | fifo_buf[12];
                int16_t gyro_z = (fifo_buf[13] << 8) | fifo_buf[14];

                // Bytes 15-20: padding/gyro cal (ignorar por ahora)

                int16_t mag_x = (fifo_buf[21] << 8) | fifo_buf[22];
                int16_t mag_y = (fifo_buf[23] << 8) | fifo_buf[24];
                int16_t mag_z = (fifo_buf[25] << 8) | fifo_buf[26];

                uint16_t footer = (fifo_buf[27] << 8) | fifo_buf[28];

                float ax = accel_x / 8192.0f;
                float ay = accel_y / 8192.0f;
                float az = accel_z / 8192.0f;
                float gx = gyro_x / 16.4f;
                float gy = gyro_y / 16.4f;
                float gz = gyro_z / 16.4f;
                float mx = mag_x * 0.15f;
                float my = mag_y * 0.15f;
                float mz = mag_z * 0.15f;

                SPP_LOGI("ICM", "HDR:%04X A:[%.2f %.2f %.2f]g G:[%.1f %.1f %.1f]dps M:[%.1f %.1f %.1f]uT FTR:%04X",
                        header, ax, ay, az, gx, gy, gz, mx, my, mz, footer);
            }

            // for (spp_uint16_t i = 0; i < num_packets; i++)
            // {
            //     spp_uint8_t fifo_buf[35] = {0};
            //     fifo_buf[0] = READ_OP | REG_FIFO_R_W;
            //     ret = SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, fifo_buf, 35);
            //     if (ret != SPP_OK) return;

            //     uint16_t header = (fifo_buf[1] << 8) | fifo_buf[2];

            //     int16_t accel_x = (fifo_buf[3] << 8) | fifo_buf[4];
            //     int16_t accel_y = (fifo_buf[5] << 8) | fifo_buf[6];
            //     int16_t accel_z = (fifo_buf[7] << 8) | fifo_buf[8];

            //     int32_t gyro_x = ((int32_t)fifo_buf[9] << 24) | (fifo_buf[10] << 16)
            //                 | (fifo_buf[11] << 8) | fifo_buf[12];
            //     int32_t gyro_y = ((int32_t)fifo_buf[13] << 24) | (fifo_buf[14] << 16)
            //                 | (fifo_buf[15] << 8) | fifo_buf[16];
            //     int32_t gyro_z = ((int32_t)fifo_buf[17] << 24) | (fifo_buf[18] << 16)
            //                 | (fifo_buf[19] << 8) | fifo_buf[20];

            //     int32_t mag_x = ((int32_t)fifo_buf[21] << 24) | (fifo_buf[22] << 16)
            //                 | (fifo_buf[23] << 8) | fifo_buf[24];
            //     int32_t mag_y = ((int32_t)fifo_buf[25] << 24) | (fifo_buf[26] << 16)
            //                 | (fifo_buf[27] << 8) | fifo_buf[28];
            //     int32_t mag_z = ((int32_t)fifo_buf[29] << 24) | (fifo_buf[30] << 16)
            //                 | (fifo_buf[31] << 8) | fifo_buf[32];

            //     uint16_t footer = (fifo_buf[33] << 8) | fifo_buf[34];

            //     float ax = accel_x / 8192.0f;
            //     float ay = accel_y / 8192.0f;
            //     float az = accel_z / 8192.0f;
            //     float gx = gyro_x / 65536.0f;
            //     float gy = gyro_y / 65536.0f;
            //     float gz = gyro_z / 65536.0f;
            //     float mx = mag_x / 65536.0f;
            //     float my = mag_y / 65536.0f;
            //     float mz = mag_z / 65536.0f;

            //     SPP_LOGI("ICM", "HDR:%04X A:[%.2f %.2f %.2f]g G:[%.1f %.1f %.1f]dps M:[%.1f %.1f %.1f]uT FTR:%04X",
            //             header, ax, ay, az, gx, gy, gz, mx, my, mz, footer);
            // }
        }
    }
    return;
}