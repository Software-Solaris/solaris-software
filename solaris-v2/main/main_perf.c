#include "spp/services/fsm/fsm.h"
#include "spp/ports/hal/esp32/halEsp32.h"
#include "spp/hal/time/time.h"
#include "stdio.h"

void app_main(void)

{
    //Get HAl port
    const SPP_HalPort_t *p_halPorts = SPP_PORTS_ESP32S3_getHalPorts();
    // Pass the HAL port to the FSM
    (void)FSM_init((void *)p_halPorts);


    // Call the Herarchical Finite State Machine

    spp_uint32_t i = 0U;
    spp_uint16_t stored = 0U;
    spp_uint32_t t0 = 0;
    spp_uint32_t t1 = 0;
    spp_uint32_t dt = 0;
    static spp_uint32_t s_dt[500] = {0};

    FSM_tick();
    FSM_tick();

    while (i < 1000000)
    {
        t0 = SPP_HAL_TIME_getTimeUs();
        FSM_tick();
        t1 = SPP_HAL_TIME_getTimeUs();
        dt = t1 - t0;
        if (dt > 15 && (stored < 500U))
        {
            s_dt[stored] = dt;
            stored++;
        }

        i++;
    }

    for (spp_int16_t j = 0; j < stored; j++)
    {
        printf("dt[%d] = %lu\n", j, s_dt[j]);
    }
}
