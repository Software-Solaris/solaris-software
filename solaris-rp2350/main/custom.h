/**
 * @file custom.h
 * @brief Finite State Machine (FSM) module for the SPP core (RP2350 target).
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
 * STRUCTS
 * ---------------------------------------------------------------- */
typedef union
{
    spp_uint16_t errors;
    struct
    {
        spp_uint16_t fsmInitError           : 1;
        spp_uint16_t bmpPubsubError         : 1;
        spp_uint16_t icmPubsubError         : 1;
        spp_uint16_t datalogggerPubsubError : 1;
        spp_uint16_t e22mbl01PubsubError    : 1;
        spp_uint16_t m10mPubsubError        : 1;
        spp_uint16_t bmpInitError           : 1;
        spp_uint16_t icmInitError           : 1;
        spp_uint16_t dataloggerInitError    : 1;
        spp_uint16_t e22mbl01InitError      : 1;
        spp_uint16_t m10mInitError          : 1;
        spp_uint16_t reserved               : 5;
    };
} FsmErrors_t;

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * ---------------------------------------------------------------- */

/**
 * @brief  Returns the pointer to the FSM transition table.
 * @return Pointer to @ref FSM_Transition_t array.
 */
const FSM_Transition_t *CUSTOM_getFsmTable(void);

/**
 * @brief  Returns the total busy time spent processing samples in microseconds.
 * @return Busy time in microseconds.
 */
spp_uint32_t CUSTOM_getBusyTimeUs(void);

/**
 * @brief  Returns the total count of processed sensor samples.
 * @return Sample counter.
 */
spp_uint16_t CUSTOM_getProcessedSamples(void);

#endif /* CUSTOM_H */
