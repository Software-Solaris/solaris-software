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

    spp_uint32_t i = 0U;
    spp_uint16_t stored = 0U;
    spp_uint32_t t0 = 0U;
    spp_uint32_t t1 = 0U;
    spp_uint32_t dt = 0U;

    static spp_uint32_t s_dt[500] = {0};

    FSM_tick();
    FSM_tick();

    while (i < 1000000U)
    {
        t0 = SPP_HAL_TIME_getTimeUs();

        FSM_tick();

        t1 = SPP_HAL_TIME_getTimeUs();

        dt = t1 - t0;

        if ((dt > 15U) && (stored < 500U))
        {
            s_dt[stored] = dt;
            stored++;
        }

        i++;
    }

    for (spp_uint16_t j = 0U; j < stored; j++)
    {
        printf("dt[%u] = %lu\n", j, s_dt[j]);
    }
}