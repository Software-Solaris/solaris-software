/**
 * @file app.c
 * @brief Application layer for Solaris.
 *
 * TODO: describe the purpose and responsibilities of this module.
 */

/* ----------------------------------------------------------------
 * INCLUDES
 * ---------------------------------------------------------------- */
#include "app.h"
#include "spp/services/fsm/fsm.h"
#include "spp/ports/hal/esp32/halEsp32.h"
#include "custom.h"

#include <stdio.h>

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * ---------------------------------------------------------------- */
/**
 * @brief Boot error handler.
 */
static inline void BootError();

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * ---------------------------------------------------------------- */
void APP_initHal(void)
{
    // Get HAL port
    const SPP_HalPort_t *p_halPorts = SPP_PORTS_ESP32S3_getHalPorts();

    // HAL init
    SPP_RetVal_t ret = SPP_HAL_init((void *)p_halPorts);
    if (ret != K_SPP_OK)
    {
        printf("BOOT FAIL: HAL ret=%d\n", (int)ret);
        BootError();
    }
}

void APP_initFsm(void)
{
    // Get the custom FSM table
    const FSM_Transition_t *p_fsmTable = CUSTOM_getFsmTable();

    // Init the FSM with the internal service table
    SPP_RetVal_t ret = FSM_init();
    if (ret != K_SPP_OK)
    {
        printf("BOOT FAIL: FSM init ret=%d\n", (int)ret);
        BootError();
    }

    // Add the application transition table
    ret = FSM_addTable(p_fsmTable, K_CUSTOM_FSM_TABLE_SIZE);
    if (ret != K_SPP_OK)
    {
        printf("BOOT FAIL: FSM addTable ret=%d\n", (int)ret);
        BootError();
    }
}

void APP_fsmLoop(void)
{
    // FSM loop
    while (1)
    {
        FSM_tick();
    }
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS IMPLEMENTATIONS
 * ---------------------------------------------------------------- */
static inline void BootError()
{
    while (1)
    {
        // Update Watchdog (future expansion)
        // Send error bit to ground station (future expansion)
    }
}
