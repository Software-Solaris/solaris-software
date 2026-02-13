#include "core/returntypes.h"
#include "core/core.h"
#include "databank.h"
#include "services/logging/spp_log.h"
#include "osal/task.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    retval_t ret;

    // 1) Init core (si aquí inicializas cosas comunes del proyecto, déjalo)
    Core_Init();

    // 2) Init databank explícito (para debuggear, luego se puede quitar porque el core ya lo inicializa)
    ret = SPP_DATABANK_init();
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPP_DATABANK_init fallo");
        while (1) { SPP_OSAL_TaskDelay(1000); }
    }

    SPP_LOGI(TAG, "=== TEST DATABANK: START ===");

    // 3) Pedimos varios paquetes 
    spp_packet_t *p0 = SPP_DATABANK_getPacket();
    spp_packet_t *p1 = SPP_DATABANK_getPacket();
    spp_packet_t *p2 = SPP_DATABANK_getPacket();

    if (!p0 || !p1 || !p2) {
        SPP_LOGE(TAG, "No se pudieron obtener 3 punteros del databank");
        while (1) { SPP_OSAL_TaskDelay(1000); }
    }

    // 4) Cambiamos una variable random para luego chekear
    p0->primary_header.version = 0xA0;
    p1->primary_header.version = 0xA1;
    p2->primary_header.version = 0xA2;

    // Log de direcciones para comprobar que no devuelven el mismo puntero
    SPP_LOGI(TAG, "p0=%p (version=0x%02X)", (void*)p0, p0->primary_header.version);
    SPP_LOGI(TAG, "p1=%p (version=0x%02X)", (void*)p1, p1->primary_header.version);
    SPP_LOGI(TAG, "p2=%p (version=0x%02X)", (void*)p2, p2->primary_header.version);

    // 5) Devolvemos en orden (p0, p1, p2)
    ret = SPP_DATABANK_returnPacket(p0);
    if (ret != SPP_OK) SPP_LOGE(TAG, "return p0 fallo");

    ret = SPP_DATABANK_returnPacket(p1);
    if (ret != SPP_OK) SPP_LOGE(TAG, "return p1 fallo");

    ret = SPP_DATABANK_returnPacket(p2);
    if (ret != SPP_OK) SPP_LOGE(TAG, "return p2 fallo");

    // 6) Volvemos a pedir 1 paquete
    spp_packet_t *pX = SPP_DATABANK_getPacket();
    if (!pX) {
        SPP_LOGE(TAG, "Fallo pX despues del return");
        while (1) { SPP_OSAL_TaskDelay(1000); }
    }

    SPP_LOGI(TAG, "pX=%p (esprado p2)", (void*)pX); // por ser LIFO

    // 7) Lo devolvemos y terminamos
    ret = SPP_DATABANK_returnPacket(pX);
    if (ret != SPP_OK) SPP_LOGE(TAG, "return pX fallo");

    SPP_LOGI(TAG, "END TEST DATABANK");
}
