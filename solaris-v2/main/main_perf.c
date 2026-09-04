#include "spp/services/fsm/fsm.h"
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

    spp_uint32_t t0 = 0U;
    spp_uint32_t t1 = 0U;
    spp_uint32_t ticks = 0;

    t0 = SPP_HAL_TIME_getTimeUs();

    while (CUSTOM_isPerformanceFinished() == false)
    {
        FSM_tick();
        ticks++;
    }

    t1 = SPP_HAL_TIME_getTimeUs();
    printf("Total time: %lu us\n", t1 - t0);
    printf("FSM ticks: %lu\n", ticks);
}