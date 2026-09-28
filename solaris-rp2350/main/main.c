/**
 * @file main.c
 * @brief Application entry point for Solaris on RP2350 / RP2350B.
 */

/* ----------------------------------------------------------------
 * INCLUDES
 * ---------------------------------------------------------------- */
#include "pico/stdlib.h"

#include "spp/services/fsm/fsm.h"
#include "spp/ports/hal/rp2350/halRp2350.h"
#include "spp/hal/hal.h"
#include "custom.h"

/* ----------------------------------------------------------------
 * APPLICATION ENTRY POINT
 * ---------------------------------------------------------------- */

int main(void)
{
    /* Initialize Pico standard I/O (UART / USB CDC) */
    stdio_init_all();

    /* 1. Get RP2350 HAL port descriptor */
    const SPP_HalPort_t *p_halPorts = SPP_PORTS_RP2350_getHalPorts();

    /* 2. HAL initialization */
    SPP_RetVal_t ret = SPP_HAL_init((void *)p_halPorts);
    if (ret != K_SPP_OK)
    {
        while (1)
        {
            /* Halt on HAL initialization failure */
        }
    }

    /* 3. Obtain custom FSM table */
    const FSM_Transition_t *p_fsmTable = CUSTOM_getFsmTable();

    /* 4. FSM initialization */
    ret = FSM_init(p_fsmTable, K_CUSTOM_FSM_TABLE_SIZE);
    if (ret != K_SPP_OK)
    {
        while (1)
        {
            /* Halt on FSM initialization failure */
        }
    }

    /* 5. Hierarchical Finite State Machine superloop */
    while (1)
    {
        FSM_tick();
    }

    return 0;
}
