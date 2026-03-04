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

            // Overflow: resetear y continuar
            if (fifo_count > 512)
            {
                data[0] = WRITE_OP | REG_FIFO_RST;
                data[1] = 0x1F;
                SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, data, 2);
                data[1] = 0x1E;
                SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, data, 2);
                continue;
            }

            // Calcular cuántos paquetes COMPLETOS hay
            spp_uint16_t num_packets = fifo_count / 34;

            // Leer TODOS los paquetes de una vez
            for (spp_uint16_t i = 0; i < num_packets; i++)
            {
                spp_uint8_t fifo_buf[35] = {0};
                fifo_buf[0] = READ_OP | REG_FIFO_R_W;
                ret = SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, fifo_buf, 35);
                if (ret != SPP_OK) return;

                uint16_t header = (fifo_buf[1] << 8) | fifo_buf[2];

                int16_t accel_x = (fifo_buf[3]  << 8) | fifo_buf[4];
                int16_t accel_y = (fifo_buf[5]  << 8) | fifo_buf[6];
                int16_t accel_z = (fifo_buf[7]  << 8) | fifo_buf[8];

                int32_t gyro_x  = ((int32_t)fifo_buf[9]  << 24) | (fifo_buf[10] << 16)
                                | (fifo_buf[11] << 8)  |  fifo_buf[12];
                int32_t gyro_y  = ((int32_t)fifo_buf[13] << 24) | (fifo_buf[14] << 16)
                                | (fifo_buf[15] << 8)  |  fifo_buf[16];
                int32_t gyro_z  = ((int32_t)fifo_buf[17] << 24) | (fifo_buf[18] << 16)
                                | (fifo_buf[19] << 8)  |  fifo_buf[20];

                int32_t mag_x   = ((int32_t)fifo_buf[21] << 24) | (fifo_buf[22] << 16)
                                | (fifo_buf[23] << 8)  |  fifo_buf[24];
                int32_t mag_y   = ((int32_t)fifo_buf[25] << 24) | (fifo_buf[26] << 16)
                                | (fifo_buf[27] << 8)  |  fifo_buf[28];
                int32_t mag_z   = ((int32_t)fifo_buf[29] << 24) | (fifo_buf[30] << 16)
                                | (fifo_buf[31] << 8)  |  fifo_buf[32];

                uint16_t footer = (fifo_buf[33] << 8) | fifo_buf[34];

                float accel_x_g  = accel_x / 16384.0f;
                float accel_y_g  = accel_y / 16384.0f;
                float accel_z_g  = accel_z / 16384.0f;

                float gyro_x_dps = gyro_x / 65536.0f;
                float gyro_y_dps = gyro_y / 65536.0f;
                float gyro_z_dps = gyro_z / 65536.0f;

                float mag_x_uT   = mag_x / 65536.0f;
                float mag_y_uT   = mag_y / 65536.0f;
                float mag_z_uT   = mag_z / 65536.0f;
            }
        }
    }
    return;
}