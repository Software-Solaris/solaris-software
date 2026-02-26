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
            /* Leer FIFO_COUNTH + FIFO_COUNTL en una sola transacción */
            data[0] = READ_OP | REG_FIFO_COUNTH;  /* 0xF0 */
            data[1] = EMPTY_MESSAGE;
            data[2] = EMPTY_MESSAGE;
            ret = SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, data, 3);
            if (ret != SPP_OK) return;

            spp_uint16_t fifo_count = ((spp_uint16_t)data[1] << 8) | data[2];

            if (fifo_count >= 16)  /* un paquete Quat6 son exactamente 16 bytes */
            {
                /* Leer FIFO_R_W (0x72) — 1 byte cmd + 16 bytes datos */
                spp_uint8_t fifo_buf[17] = {0};
                fifo_buf[0] = READ_OP | REG_FIFO_R_W;  /* 0xF2 */
                ret = SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, fifo_buf, 17);
                if (ret != SPP_OK) return;

                /* fifo_buf[1..2]  = Header  (debe ser 0x0808) */
                /* fifo_buf[3..6]  = Q1      (4 bytes, big-endian) */
                /* fifo_buf[7..10] = Q2 */
                /* fifo_buf[11..14]= Q3 */
                /* fifo_buf[15..16]= Footer  (gyro count) */
                break;
            }
        }

        SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));
    }


    vTaskDelay(pdMS_TO_TICKS(50));

}