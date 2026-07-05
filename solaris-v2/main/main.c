#include "spp/core/returnTypes.h"
#include "spp/services/pubsub/pubsub.h"
#include "init.h"

void app_main(void)

{
    SPP_RetVal_t ret = SPP_MAIN_init();
    if (ret != K_SPP_OK)
    {
        return;
    }

    for (;;)
    {
        SPP_SERVICES_PUBSUB_callProducers();
        SPP_SERVICES_PUBSUB_callConsumers();
    }
}
