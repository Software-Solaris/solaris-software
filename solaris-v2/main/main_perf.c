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

    while (SPP_SERVICES_BMP390_getPerformanceSampleCount() < K_CUSTOM_PERFORMANCE_SAMPLES)
    {
        FSM_tick();
    }

    spp_uint32_t t1 = SPP_HAL_TIME_getTimeUs();

    for (spp_uint16_t i = 0U; i < K_CUSTOM_PERFORMANCE_SAMPLES; i++)
    {
        spp_uint32_t bmpUs = 0U;
        spp_uint32_t tempSpiUs = 0U;
        spp_uint32_t pressSpiUs = 0U;

        if (SPP_SERVICES_BMP390_getPerformanceSample(i, &bmpUs, &tempSpiUs, &pressSpiUs) == K_SPP_OK)
        {
            printf("%lu,%lu,%lu\n", (unsigned long)bmpUs, (unsigned long)tempSpiUs, (unsigned long)pressSpiUs);
        }
    }
    printf("Benchmark time: %lu us\n", (unsigned long)(t1 - t0));
}