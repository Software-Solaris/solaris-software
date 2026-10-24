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
#include "spp/services/icm20948/icm20948.h"
#include "spp/services/datalogger/datalogger.h"
#include "spp/services/databank/databank.h"
#include "spp/services/uart/uart.h"
#include "spp/services/kpid.h"
#include "spp/services/service.h"

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS DECLARATIONS
 * ---------------------------------------------------------------- */
static spp_bool_t CUSTOM_registerConsumerProducer(void);
static spp_bool_t guard_startInitializationOfModules(void);
static void action_emitTelemetry(void);

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
};

static FsmErrors_t s_fsmErrors = {0};

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * ---------------------------------------------------------------- */
const FSM_Transition_t *CUSTOM_getFsmTable(void)
{
    return s_transitionTable;
}

/* ----------------------------------------------------------------
 * PRIVATE FUNCTIONS
 * ---------------------------------------------------------------- */

/* Guards */

static spp_bool_t guard_startInitializationOfModules(void)
{
    if (CUSTOM_registerConsumerProducer() == false)
    {
        return false;
    }

    return (SPP_CORE_init() == K_SPP_OK);
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

    return;
}

/**
* @brief    Register consumers and producers.
* @return   true on success.
*/
static spp_bool_t CUSTOM_registerConsumerProducer(void)
{
    SPP_Kpid_t bmpKpid = {0};
    SPP_Kpid_t icmKpid = {0};
    SPP_Kpid_t sdSubscription = {0};

    const SPP_SERVICE_ProducerContract_t *p_bmpContract = SPP_SERVICES_BMP390_getProducerContract();
    if (p_bmpContract == NULL || SPP_SERVICES_PUBSUB_registerProducer(p_bmpContract, &bmpKpid) != K_SPP_OK)
    {
        s_fsmErrors.bmpPubsubError = 1;
    }

    const SPP_SERVICE_ProducerContract_t *p_icmContract = SPP_SERVICES_ICM20948_getProducerContract();
    if (p_icmContract == NULL || SPP_SERVICES_PUBSUB_registerProducer(p_icmContract, &icmKpid) != K_SPP_OK)
    {
        s_fsmErrors.icmPubsubError = 1;
    }

    sdSubscription.value = bmpKpid.value | icmKpid.value | K_SPP_KPID_FSM;

    const SPP_SERVICE_ConsumerContract_t *p_sdContract = SPP_SERVICES_DATALOGGER_getConsumerContract();

    SPP_RetVal_t ret = SPP_SERVICES_PUBSUB_registerConsumer(p_sdContract, sdSubscription);
    if (p_sdContract == NULL || ret != K_SPP_OK)
    {
        s_fsmErrors.datalogggerPubsubError = 1;
    }

    if (s_fsmErrors.bmpPubsubError & s_fsmErrors.icmPubsubError)
    {
        return false;
    }

    return true;
}
