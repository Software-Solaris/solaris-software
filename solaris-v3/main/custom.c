/**
 * @file custom.c
 * @brief Custom FSM transitions and Pub/Sub setup for Solaris on RP2350.
 */

/* ----------------------------------------------------------------
 * INCLUDES
 * ---------------------------------------------------------------- */
#include "custom.h"

#include "spp/core/core.h"
#include "spp/core/commonbit.h"
#include "spp/core/pubsub/pubsub.h"

#include "spp/services/bmp390/bmp390.h"
#include "spp/services/databank/databank.h"
#include "spp/services/kpid.h"
#include "spp/services/service.h"

#include "spp/ports/hal/rp2350/halRp2350.h"
#include "spp/hal/hal.h"
#include "spp/hal/time/time.h"

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS DECLARATIONS
 * ---------------------------------------------------------------- */
static spp_bool_t CUSTOM_registerConsumerProducer(void);
static spp_bool_t CUSTOM_initCheck(void);
static spp_bool_t guard_startInitializationOfModules(void);
static void action_emitTelemetry(void);
static void statefunction_bmpPerformanceLoop(void);

/* ----------------------------------------------------------------
 * STATIC VARIABLES
 * ---------------------------------------------------------------- */
static FSM_Transition_t s_transitionTable[] = {
    {
        .fromState     = FSM_STATE_INIT,
        .fromSubState  = FSM_SUBSTATE_NONE,
        .toState       = FSM_STATE_READY,
        .toSubState    = FSM_SUBSTATE_NONE,
        .guard         = guard_startInitializationOfModules,
        .action        = action_emitTelemetry,
        .stateFunction = NULL,
    },
    {
        .fromState     = FSM_STATE_READY,
        .fromSubState  = FSM_SUBSTATE_NONE,
        .toState       = FSM_STATE_READY,
        .toSubState    = FSM_SUBSTATE_NONE,
        .guard         = NULL,
        .action        = NULL,
        .stateFunction = statefunction_bmpPerformanceLoop,
    },
};

static FsmErrors_t s_fsmErrors = {0};
static const SPP_SERVICE_ProducerContract_t *s_p_bmpProducerContract = NULL;

static spp_uint16_t s_processedSamples = 0U;
static spp_uint32_t s_busyTimeUs = 0U;

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * ---------------------------------------------------------------- */

const FSM_Transition_t *CUSTOM_getFsmTable(void)
{
    return s_transitionTable;
}

spp_uint32_t CUSTOM_getBusyTimeUs(void)
{
    return s_busyTimeUs;
}

spp_uint16_t CUSTOM_getProcessedSamples(void)
{
    return s_processedSamples;
}

/* ----------------------------------------------------------------
 * GUARDS
 * ---------------------------------------------------------------- */

static spp_bool_t guard_startInitializationOfModules(void)
{
    spp_bool_t registered = CUSTOM_registerConsumerProducer();
    if (!registered)
    {
        return false;
    }

    SPP_RetVal_t ret = SPP_CORE_init();
    if (ret != K_SPP_OK)
    {
        return false;
    }

    spp_bool_t initCheck = CUSTOM_initCheck();
    if (!initCheck)
    {
        return false;
    }

    return true;
}

/* ----------------------------------------------------------------
 * ACTIONS
 * ---------------------------------------------------------------- */

static void action_emitTelemetry(void)
{
    SPP_Packet_t *p_packet = SPP_SERVICES_DATABANK_getPacket();
    CommonBitErrors_t *p_commonBit = SPP_CORE_COMMONBIT_getBit();
    if (p_commonBit == NULL)
    {
        return;
    }

    spp_uint16_t errorBitPayload[2] = {s_fsmErrors.errors, p_commonBit->errors};
    SPP_RetVal_t ret = SPP_SERVICES_DATABANK_packetData(p_packet, K_SPP_KPID_FSM, 0U, errorBitPayload,
                                                        (spp_uint16_t)sizeof(errorBitPayload));
    if (ret != K_SPP_OK)
    {
        return;
    }

    ret = SPP_SERVICES_PUBSUB_publish(p_packet);
    if (ret != K_SPP_OK)
    {
        return;
    }
}

/* ----------------------------------------------------------------
 * STATE FUNCTIONS
 * ---------------------------------------------------------------- */

static void statefunction_bmpPerformanceLoop(void)
{
    spp_uint8_t c0 = SPP_SERVICES_PUBSUB_queueDepth();
    spp_uint32_t t0 = SPP_HAL_TIME_getTimeUs();

    SPP_SERVICES_PUBSUB_callProducers();

    spp_uint8_t c1 = SPP_SERVICES_PUBSUB_queueDepth();
    SPP_SERVICES_PUBSUB_callConsumers();

    spp_uint32_t t1 = SPP_HAL_TIME_getTimeUs();

    if (c1 > c0)
    {
        s_busyTimeUs += (t1 - t0);
        s_processedSamples++;
    }
}

/* ----------------------------------------------------------------
 * PRIVATE FUNCTIONS
 * ---------------------------------------------------------------- */

static spp_bool_t CUSTOM_registerConsumerProducer(void)
{
    SPP_Kpid_t bmp390Kpid = {0};
    SPP_RetVal_t ret = K_SPP_ERROR;

    s_p_bmpProducerContract = SPP_SERVICES_BMP390_getProducerContract();
    if (s_p_bmpProducerContract == NULL)
    {
        s_fsmErrors.bmpPubsubError = 1;
    }
    else
    {
        ret = SPP_SERVICES_PUBSUB_registerProducer(s_p_bmpProducerContract, &bmp390Kpid);
        if (ret != K_SPP_OK)
        {
            s_fsmErrors.bmpPubsubError = 1;
        }
    }

    if (s_fsmErrors.bmpPubsubError != 0U)
    {
        return false;
    }

    return true;
}

static spp_bool_t CUSTOM_initCheck(void)
{
    SPP_RetVal_t bmpInitResult = K_SPP_ERROR_NOT_INITIALIZED;
    SPP_RetVal_t ret = SPP_SERVICES_PUBSUB_getProducerInitResult(s_p_bmpProducerContract, &bmpInitResult);
    if ((ret != K_SPP_OK) || (bmpInitResult == K_SPP_ERROR_NOT_INITIALIZED))
    {
        return false;
    }

    if (bmpInitResult != K_SPP_OK)
    {
        s_fsmErrors.bmpInitError = 1;
        return false;
    }

    return true;
}
