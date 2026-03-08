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
        ICM_checkFifoData((void*)&s_icm);
    }
    return;
}