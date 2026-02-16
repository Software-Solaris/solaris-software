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
    retval_t ret;

    // sleep(5);
    icm_data_t s_icm;

    ret = IcmInit((void*)s_icm);
    if (ret != K_RET_OK){
        return ret;
    }

    ret = IcmConfig((void*)s_icm);
    if (ret != K_RET_OK){
        return ret;
    }


    vTaskDelay(pdMS_TO_TICKS(50));

}