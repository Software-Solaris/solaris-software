#include "spp/services/fsm/fsm.h"
#include "spp/ports/hal/esp32/halEsp32.h"

void app_main(void)

{
    //Get HAl port
    const SPP_HalPort_t *p_halPorts = SPP_PORTS_ESP32S3_getHalPorts();
    // Pass the HAL port to the FSM
    (void)FSM_init((void *)p_halPorts);

    // Call the Herarchical Finite State Machine
    while (1)
    {
        FSM_tick();
    }
}
