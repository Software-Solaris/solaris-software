#include "spp/services/fsm/fsm.h"
#include "spp/ports/hal/esp32/halEsp32.h"
#include "custom.h"

void app_main(void)

{
    //Get HAL port
    const SPP_HalPort_t *p_halPorts = SPP_PORTS_ESP32S3_getHalPorts();

    // Get the custom FSM
    const FSM_Transition_t *p_fsmTable = CUSTOM_getFsmTable();
    // Pass the HAL port to the FSM
    SPP_RetVal_t ret = FSM_init((void *)p_halPorts, p_fsmTable, K_CUSTOM_FSM_TABLE_SIZE);
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