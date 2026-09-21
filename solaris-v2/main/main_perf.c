#include "spp/services/fsm/fsm.h"
#include "spp/services/bmp390/bmp390.h"

#include "spp/ports/hal/esp32/halEsp32.h"
#include "spp/hal/hal.h"
#include "spp/hal/time/time.h"

#include "custom.h"

#include <stdio.h>

void app_main(void)
{
    // Get HAL port
    const SPP_HalPort_t *p_halPorts = SPP_PORTS_ESP32S3_getHalPorts();

    // HAL init
    SPP_RetVal_t ret = SPP_HAL_init((void *)p_halPorts);
    if (ret != K_SPP_OK)
    {
        while (1)
        {
            // Infinite loop
        }
    }

    // Get custom FSM table
    const FSM_Transition_t *p_fsmTable = CUSTOM_getFsmTable();

    // FSM init
    ret = FSM_init(p_fsmTable, K_CUSTOM_FSM_TABLE_SIZE);
    if (ret != K_SPP_OK)
    {
        while (1)
        {
            // Infinite loop
        }
    }

    FSM_tick();

    spp_uint32_t t0 = SPP_HAL_TIME_getTimeUs();

    while (CUSTOM_getProcessedSamples() < 10000)
    {
        FSM_tick();
    }

    spp_uint32_t t1 = SPP_HAL_TIME_getTimeUs();

    spp_uint32_t totalTimeUs = t1 - t0;
    spp_uint32_t busyTimeUs = CUSTOM_getBusyTimeUs();
    spp_float32_t utilization = ((spp_float32_t)busyTimeUs * 100.0f) / (spp_float32_t)totalTimeUs;

    printf("Total time: %lu us\n", (unsigned long)totalTimeUs);

    printf("Busy time: %lu us\n", (unsigned long)busyTimeUs);

    printf("Processed samples: %u\n", (unsigned int)CUSTOM_getProcessedSamples());

    printf("SPP processing utilization: %.2f %%\n", (double)utilization);
}