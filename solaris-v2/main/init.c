/**
 * @file init.c
 * @brief System initialisation sequence for the Solaris firmware.
 */

#include "init.h"

#include "spp/spp.h"
#include "spp/hal/hal.h"
#include "spp/ports/hal/esp32/halEsp32.h"
// #include "spp/services/bmp390/bmp390.h"
// #include "spp/services/icm20948/icm20948.h"
#include "spp/services/datalogger/datalogger.h"
#include "spp/services/pubsub/pubsub.h"
#include "spp/services/service.h"

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * ---------------------------------------------------------------- */

SPP_RetVal_t SPP_MAIN_init(void)
{
    SPP_Kpid_t bmp390Kpid = {0};
    SPP_Kpid_t icm20948Kpid = {0};

    SPP_Kpid_t sdSubscription = {0};
    SPP_Kpid_t e22mbl01Subscription = {0};

    SPP_RetVal_t ret = SPP_HAL_init(SPP_PORTS_ESP32S3_getHalPorts());
    if (ret != K_SPP_OK)
    {
        return ret;
    }

    // const SPP_SERVICE_ProducerContract_t *p_bmpProducerContract = SPP_SERVICES_BMP390_getProducerContract();
    // if (p_bmpProducerContract == NULL)
    // {
    //     return K_SPP_ERROR_NULL_POINTER;
    // }
    // ret = SPP_SERVICES_PUBSUB_registerProducer(p_bmpProducerContract);
    // if (ret != K_SPP_OK)
    // {
    //     return ret;
    // }

    // const SPP_SERVICE_ProducerContract_t *p_icmProducerContract = SPP_SERVICES_ICM20948_getProducerContract();
    // if (p_icmProducerContract == NULL)
    // {
    //     return K_SPP_ERROR_NULL_POINTER;
    // }
    // ret = SPP_SERVICES_PUBSUB_registerProducer(p_icmProducerContract);
    // if (ret != K_SPP_OK)
    // {
    //     return ret;
    // }

    const SPP_SERVICE_ConsumerContract_t *p_dataloggerConsumerContract = SPP_SERVICES_DATALOGGER_getConsumerContract();
    if (p_dataloggerConsumerContract == NULL)
    {
        return K_SPP_ERROR_NULL_POINTER;
    }
    ret = SPP_SERVICES_PUBSUB_registerProducer(p_bmpProducerContract, &bmp390Kpid);
    if (ret != K_SPP_OK)
    {
        return ret;
    }

    const SPP_SERVICE_ProducerContract_t *p_icmProducerContract = SPP_SERVICES_ICM20948_getProducerContract();
    if (p_icmProducerContract == NULL)
    {
        return K_SPP_ERROR_NULL_POINTER;
    }
    ret = SPP_SERVICES_PUBSUB_registerProducer(p_icmProducerContract, &icm20948Kpid);
    if (ret != K_SPP_OK)
    {
        return ret;
    }

    // Example of use with subscriptions to differents producers
    sdSubscription.value = bmp390Kpid.value | icm20948Kpid.value;
    e22mbl01Subscription.value = icm20948Kpid.value;

    // TO-DO: add this function to consumer drivers
    // const SPP_SERVICE_ConsumerContract_t *p_sdConsumerContract = SPP_SERVICES_DATALOGGER_getConsumerContract();
    const SPP_SERVICE_ConsumerContract_t *p_sdConsumerContract = NULL;
    ret = SPP_SERVICES_PUBSUB_registerConsumer(p_sdConsumerContract, sdSubscription);
    if (ret != K_SPP_OK)
    {
        return ret;
    }

    // const SPP_SERVICE_ConsumerContract_t *p_e22mbl01ConsumerContract = SPP_SERVICES_E22MBL01_getConsumerContract();
    const SPP_SERVICE_ConsumerContract_t *p_e22mbl01ConsumerContract = NULL;
    ret = SPP_SERVICES_PUBSUB_registerConsumer(p_e22mbl01ConsumerContract, e22mbl01Subscription);
    if (ret != K_SPP_OK)
    {
        return ret;
    }

    ret = SPP_CORE_init();
    return ret;
}
