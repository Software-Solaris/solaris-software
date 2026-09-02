#include "spp/services/fsm/fsm.h"
#include "spp/ports/hal/esp32/halEsp32.h"
#include "custom.h"

void app_main(void)

{
    //Get HAL port
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

    // Get the custom FSM
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

    // Call the Herarchical Finite State Machine
    while (1)
    {
        FSM_tick();
    }
}