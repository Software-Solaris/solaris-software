/**
 * @file custom.c
 * @brief Custom definitions for the Solaris Packet Protocol
 */


/* ----------------------------------------------------------------
 * INCLUDES
 * ---------------------------------------------------------------- */
#include "spp/services/fsm/fsm.h"

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS DECLARATIONS
 * ---------------------------------------------------------------- */
static spp_bool_t CUSTOM_registerConsumerProducer(void);
static spp_bool_t guard_startInitializationOfModules(void);
static void action_emitTelemetry(void);
static void statefunction_pubSubLoop(void);

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
        .toState = FSM_STATE_FLIGHT,
        .toSubState = FSM_SUBSTATE_FLIGHT_ASCENDING,
        .guard = NULL,
        .action = NULL,
        .stateFunction = statefunction_pubSubLoop,
    },
};


/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * ---------------------------------------------------------------- */
FSM_Transition_t *CUSTOM_getFsmTable(void)
{
    return &s_transitionTable;
}
/* ----------------------------------------------------------------
 * PRIVATE FUNCTIONS
 * ---------------------------------------------------------------- */

/* Guards */

static spp_bool_t guard_startInitializationOfModules(void)
{
    // Init the hardware abstraction layer (HAL)
    SPP_RetVal_t ret = SPP_HAL_init(s_p_ports);
    if (ret != K_SPP_OK)
    {
        s_fsmErrors.halInitError = 1;
        return false;
    }
    else
    {
        spp_bool_t initialized = SPP_CORE_FSM_registerConsumerProducer();
        if (initialized == false)
        {
            return false;
        }
        else
        {
            ret = SPP_CORE_init();
            if (ret != K_SPP_OK)
            {
                return false;
            }
        }
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

static void statefunction_pubSubLoop(void)
{
    (void)SPP_SERVICES_PUBSUB_callProducers();
    (void)SPP_SERVICES_PUBSUB_callConsumers();
}


/**
* @brief    Register consumers and producers.
* @return   K_SPP_OK on success.
*/
static spp_bool_t CUSTOM_registerConsumerProducer(void)
{
    SPP_Kpid_t bmp390Kpid = {0};
    SPP_Kpid_t icm20948Kpid = {0};
    SPP_Kpid_t m10mKpid = {0};

    SPP_Kpid_t sdSubscription = {0};
    SPP_Kpid_t e22mbl01Subscription = {0};

    SPP_RetVal_t ret = K_SPP_ERROR;

    const SPP_SERVICE_ProducerContract_t *p_bmpProducerContract = SPP_SERVICES_BMP390_getProducerContract();
    if (p_bmpProducerContract == NULL)
    {
        s_fsmErrors.bmpPubsubError = 1;
    }
    ret = SPP_SERVICES_PUBSUB_registerProducer(p_bmpProducerContract, &bmp390Kpid);
    if (ret != K_SPP_OK)
    {
        s_fsmErrors.bmpPubsubError = 1;
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
