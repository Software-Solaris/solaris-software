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

    while (true)
    {
        spp_uint8_t data[3] = {0};

        data[0] = READ_OP | REG_INT_STATUS; 
        data[1] = EMPTY_MESSAGE;
        ret = SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, data, 2);
        if (ret != SPP_OK) return;

        spp_uint8_t int_status = data[1];

        data[0] = READ_OP | REG_DMP_INT_STATUS;
        data[1] = EMPTY_MESSAGE;
        ret = SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, data, 2);
        if (ret != SPP_OK) return;

        if (int_status & 0x02)
        {
            data[0] = READ_OP | REG_FIFO_COUNTH;
            data[1] = EMPTY_MESSAGE;
            data[2] = EMPTY_MESSAGE;
            ret = SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, data, 3);
            if (ret != SPP_OK) return;

            spp_uint16_t fifo_count = ((spp_uint16_t)data[1] << 8) | data[2];

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

                // Bytes 15-20: padding/gyro cal

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
        }
    }
    return;
}