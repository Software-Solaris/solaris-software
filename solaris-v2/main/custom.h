/**
 * @file custom.h
 * @brief Finite State Machine (FSM) module for the SPP core.
 */

#ifndef CUSTOM_H
#define CUSTOM_H

/* ----------------------------------------------------------------
 * INCLUDES
 * ---------------------------------------------------------------- */
#include "spp/services/fsm/fsm.h"

/* ----------------------------------------------------------------
 * DEFINES
 * ---------------------------------------------------------------- */
#define K_CUSTOM_FSM_TABLE_SIZE (spp_uint8_t)(2U)


/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * ---------------------------------------------------------------- */

// TODO: Add Doxygen
const FSM_Transition_t *CUSTOM_getFsmTable(void);


#endif /* CUSTOM_H*/