#include "spp/core/returnTypes.h"
#include "spp/services/pubsub/pubsub.h"
#include "init.h"
#include "stdio.h"

void app_main(void)
{
    printf("------- Entró a main --------\n");
    SPP_RetVal_t ret = SPP_MAIN_init();
    if (ret != K_SPP_OK)
    {
        printf("Init error");
        return;
    }

    printf("________Salió de main_________\n");

    for (;;)
    {
        SPP_SERVICES_PUBSUB_callProducers();
        SPP_SERVICES_PUBSUB_callConsumers();
    }
}
