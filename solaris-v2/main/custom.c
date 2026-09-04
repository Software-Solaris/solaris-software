/**
 * @file custom.c
 * @brief Custom definitions for the Solaris Packet Protocol
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

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS DECLARATIONS
 * ---------------------------------------------------------------- */
static spp_bool_t CUSTOM_registerConsumerProducer(void);
static spp_bool_t CUSTOM_initCheck(void);
static spp_bool_t guard_startInitializationOfModules(void);
static void action_emitTelemetry(void);
static void statefunction_bmpPerformanceLoop(void);

/* ----------------------------------------------------------------
 * VARIABLES
 * ---------------------------------------------------------------- */
static FSM_Transition_t s_transitionTable[] = {
    {
        .fromState = FSM_STATE_INIT,
        .fromSubState = FSM_SUBSTATE_NONE,
        .toState = FSM_STATE_READY,
        .toSubState = FSM_SUBSTATE_NONE,
        .guard = guard_startInitializationOfModules,
        .action = action_emitTelemetry,
        .stateFunction = NULL,
    },
    {
        .fromState = FSM_STATE_READY,
        .fromSubState = FSM_SUBSTATE_NONE,
        .toState = FSM_STATE_READY,
        .toSubState = FSM_SUBSTATE_NONE,
        .guard = NULL,
        .action = NULL,
        .stateFunction = statefunction_bmpPerformanceLoop,
    },
};

static FsmErrors_t s_fsmErrors = {0};
static const SPP_SERVICE_ProducerContract_t *s_p_bmpProducerContract = NULL;

static spp_uint16_t s_performanceSamples = 0U;
static spp_bool_t s_performanceFinished = false;

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * ---------------------------------------------------------------- */
const FSM_Transition_t *CUSTOM_getFsmTable(void)
{
    return s_transitionTable;
}

spp_bool_t CUSTOM_isPerformanceFinished(void)
{
    return s_performanceFinished;
}

/* ----------------------------------------------------------------
 * PRIVATE FUNCTIONS
 * ---------------------------------------------------------------- */

/* Guards */

static spp_bool_t guard_startInitializationOfModules(void)
{
    spp_bool_t registered = CUSTOM_registerConsumerProducer();
    if (registered == false)
    {
        return false;
    }

    SPP_RetVal_t ret = SPP_CORE_init();

    spp_bool_t initCheck = CUSTOM_initCheck();

    if (ret != K_SPP_OK)
    {
        return false;
    }

    if (initCheck == false)
    {
        return false;
    }
    return true;
}

/* Actions*/

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

    // Publish the packet to the PUBSUB service
    ret = SPP_SERVICES_PUBSUB_publish(p_packet);
    if (ret != K_SPP_OK)
    {
        return;
    }

    // Call the consumers (to force the telemetry to be sent)
    //(void)SPP_SERVICES_PUBSUB_callConsumers();

    return;
}

/* State functions */

static void statefunction_bmpPerformanceLoop(void)
{
    // if (s_performanceFinished == true)
    // {
    //     return;
    // }

    // spp_uint8_t c0 = SPP_SERVICES_PUBSUB_queueDepth();

    (void)SPP_SERVICES_PUBSUB_callProducers();

    // spp_uint8_t c1 = SPP_SERVICES_PUBSUB_queueDepth();

    // if (c1 > c0)
    // {
    //     s_performanceSamples++;
    // }

    (void)SPP_SERVICES_PUBSUB_callConsumers();

    // s_performanceSamples++;

    // if (s_performanceSamples >= K_CUSTOM_PERFORMANCE_SAMPLES)
    // {
    //     s_performanceFinished = true;
    // }
}


/**
* @brief    Register consumers and producers.
* @return   K_SPP_OK on success.
*/
static spp_bool_t CUSTOM_registerConsumerProducer(void)
{
    SPP_Kpid_t bmp390Kpid = {0};
    // SPP_Kpid_t icm20948Kpid = {0};
    // SPP_Kpid_t m10mKpid = {0};

    // SPP_Kpid_t sdSubscription = {0};
    // SPP_Kpid_t e22mbl01Subscription = {0};

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

    // const SPP_SERVICE_ProducerContract_t *p_icmProducerContract = SPP_SERVICES_ICM20948_getProducerContract();
    // if (p_icmProducerContract == NULL)
    // {
    //     s_fsmErrors.icmPubsubError = 1;
    // }
    // ret = SPP_SERVICES_PUBSUB_registerProducer(p_icmProducerContract, &icm20948Kpid);
    // if (ret != K_SPP_OK)
    // {
    //     s_fsmErrors.icmPubsubError = 1;
    // }

    // const SPP_SERVICE_ProducerContract_t *p_m10mProducerContract = SPP_SERVICES_M10M_getProducerContract();
    // if (p_m10mProducerContract == NULL)
    // {
    //     s_fsmErrors.m10mPubsubError = 1;
    // }
    // ret = SPP_SERVICES_PUBSUB_registerProducer(p_m10mProducerContract, &m10mKpid);
    // if (ret != K_SPP_OK)
    // {
    //     s_fsmErrors.m10mPubsubError = 1;
    // }

    // sdSubscription.value = bmp390Kpid.value | icm20948Kpid.value | m10mKpid.value;

    // const SPP_SERVICE_ConsumerContract_t *p_sdConsumerContract = SPP_SERVICES_DATALOGGER_getConsumerContract();
    // ret = SPP_SERVICES_PUBSUB_registerConsumer(p_sdConsumerContract, sdSubscription);
    // if (ret != K_SPP_OK)
    // {
    //     s_fsmErrors.datalogggerPubsubError = 1;
    // }

    // e22mbl01Subscription.value = bmp390Kpid.value | icm20948Kpid.value | K_SPP_KPID_FSM;
    // const SPP_SERVICE_ConsumerContract_t *p_e22mbl01ConsumerContract = SPP_SERVICES_E22MBL01_getConsumerContract();
    // ret = SPP_SERVICES_PUBSUB_registerConsumer(p_e22mbl01ConsumerContract, e22mbl01Subscription);
    // if (ret != K_SPP_OK)
    // {
    //     s_fsmErrors.e22mbl01PubsubError = 1;
    // }

    // spp_bool_t producersFailed =
    //     (s_fsmErrors.bmpPubsubError && s_fsmErrors.icmPubsubError && s_fsmErrors.m10mPubsubError);
    // spp_bool_t consumersFailed = (s_fsmErrors.datalogggerPubsubError && s_fsmErrors.e22mbl01PubsubError);

    if (s_fsmErrors.bmpPubsubError)
    {
        return false;
    }

    return true;
}

static spp_bool_t CUSTOM_initCheck(void)
{
    SPP_RetVal_t bmpInitResult = K_SPP_ERROR_NOT_INITIALIZED;
    SPP_RetVal_t ret = SPP_SERVICES_PUBSUB_getProducerInitResult(s_p_bmpProducerContract, &bmpInitResult);
    if (ret != K_SPP_OK || bmpInitResult == K_SPP_ERROR_NOT_INITIALIZED)
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