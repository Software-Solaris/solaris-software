#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "core/returntypes.h"
#include "core/core.h"
#include "databank.h"
#include "icm20948.h"
#include "spi.h"
#include "bmp390.h"

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
    while(true){
        spp_uint8_t data[2] = {0};
        data[0] = READ_OP | REG_FIFO_COUNTH;
        data[1] = EMPTY_MESSAGE;
        ret = SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, data, 2);
        if (ret != SPP_OK) return;
        spp_uint8_t count_h = data[1];

        data[0] = READ_OP | REG_FIFO_COUNTL;
        data[1] = EMPTY_MESSAGE;
        ret = SPP_HAL_SPI_Transmit(s_icm.p_handler_spi, data, 2);
        if (ret != SPP_OK) return;
        spp_uint8_t count_l = data[1];

        spp_uint16_t fifo_count = ((spp_uint16_t)count_h << 8) | count_l;

        if (fifo_count > 0)
        {
            return;
        }
    }


    vTaskDelay(pdMS_TO_TICKS(50));

}