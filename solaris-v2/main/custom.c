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
#include "spp/services/sx1262/sx1262.h"
#include "spp/services/kpid.h"
#include "spp/services/service.h"

#include <stdio.h>

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
    spp_bool_t pubsubOk = CUSTOM_registerConsumerProducer();

    printf("PUBSUB REG | BMP=%u ICM=%u SD=%u SX1262=%u\n", s_fsmErrors.bmpPubsubError, s_fsmErrors.icmPubsubError,
           s_fsmErrors.datalogggerPubsubError, s_fsmErrors.sx1262PubsubError);

    if (pubsubOk == false)
    {
        return false;
    }

    SPP_RetVal_t ret = SPP_CORE_init();

    SPP_RetVal_t bmpInit = K_SPP_ERROR;
    SPP_RetVal_t icmInit = K_SPP_ERROR;
    SPP_RetVal_t sdInit = K_SPP_ERROR;
    SPP_RetVal_t sx1262Init = K_SPP_ERROR;

    (void)SPP_SERVICES_PUBSUB_getProducerInitResult(SPP_SERVICES_BMP390_getProducerContract(), &bmpInit);
    if (bmpInit != K_SPP_OK)
    {
        s_fsmErrors.bmpInitError = 1;
    }

    (void)SPP_SERVICES_PUBSUB_getProducerInitResult(SPP_SERVICES_ICM20948_getProducerContract(), &icmInit);
    if (icmInit != K_SPP_OK)
    {
        s_fsmErrors.icmInitError = 1;
    }

    (void)SPP_SERVICES_PUBSUB_getConsumerInitResult(SPP_SERVICES_DATALOGGER_getConsumerContract(), &sdInit);
    if (sdInit != K_SPP_OK)
    {
        s_fsmErrors.dataloggerInitError = 1;
    }

    (void)SPP_SERVICES_PUBSUB_getConsumerInitResult(SPP_SERVICES_SX1262_getConsumerContract(), &sx1262Init);
    if (sx1262Init != K_SPP_OK)
    {
        s_fsmErrors.sx1262InitError = 1;
    }

    printf("INIT | CORE=%d BMP=%d ICM=%d SD=%d SX1262=%d\n", (int)ret, (int)bmpInit, (int)icmInit, (int)sdInit,
           (int)sx1262Init);

    if (ret != K_SPP_OK)
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
    SPP_Kpid_t sx1262Subscription = {0};

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

    if (p_sdContract == NULL || SPP_SERVICES_PUBSUB_registerConsumer(p_sdContract, sdSubscription) != K_SPP_OK)
    {
        s_fsmErrors.datalogggerPubsubError = 1;
    }

    sx1262Subscription.value = bmpKpid.value | icmKpid.value | K_SPP_KPID_FSM;

    const SPP_SERVICE_ConsumerContract_t *p_sx1262Contract = SPP_SERVICES_SX1262_getConsumerContract();

    if (p_sx1262Contract == NULL ||
        SPP_SERVICES_PUBSUB_registerConsumer(p_sx1262Contract, sx1262Subscription) != K_SPP_OK)
    {
        s_fsmErrors.sx1262PubsubError = 1;
    }

    spp_uint8_t producersFailed = (s_fsmErrors.bmpPubsubError && s_fsmErrors.icmPubsubError);
    spp_uint8_t consumersFailed = (s_fsmErrors.datalogggerPubsubError && s_fsmErrors.sx1262PubsubError);

    if (producersFailed || consumersFailed)
    {
        return false;
    }

    return true;
}
