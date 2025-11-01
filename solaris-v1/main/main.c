#include "macros_esp.h"
#include "spi.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "macros.h"


void app_main()
{
    retval_t ret;
    void* p_dev;

    //texto

    ret = SPP_HAL_SPI_BusInit();
    if (ret != SPP_OK) return;

    for(int i = 0; i < MAX_DEVICES; i++)
    {
        p_dev = SPP_HAL_SPI_GetHandler();
        if (p_dev == NULL) break;

        ret = SPP_HAL_SPI_DeviceInit(p_dev);
        if (ret == SPP_ERROR) break; 
    }
}

/*
FLUJO ESPERADO DE INICIALIZACIÓN SPI (BMP → ICM) CON CASOS DE FALLO

1) Bus Init
   main → SPP_HAL_SPI_BusInit()
   - Interno: llama a spi_bus_initialize().
   - Si ESP_OK → SPP_OK. Si falla → SPP_ERROR y main retorna (no seguimos sin bus).

2) Bucle de alta de dispositivos (máx. MAX_DEVICES iteraciones)

   Iteración 1:
   main → p_dev = SPP_HAL_SPI_GetHandler()
   - Interno: recorre device_state[0..n_devices-1] y devuelve &device_ids[i] del PRIMER SLOT con SLOT_EMPTY.
   - Estado inicial típico: [EMPTY, EMPTY] → devuelve &device_ids[0] (BMP).
   - Si no hay slots vacíos → devuelve NULL (main romperá el bucle).

   main → ret = SPP_HAL_DeviceInit(p_dev)
   - Interno: valida p_dev != NULL y que apunte a UNO DE NUESTROS SLOTS (&device_ids[j]) → obtiene flag=j.
   - Comprueba estado paralelo:
       * Si device_state[flag] == SLOT_READY → retorna SPP_OK (idempotencia).
       * Si device_state[flag] == SLOT_EMPTY → compone perfil interno (según flag: BMP o ICM).
   - Llama a spi_bus_add_device(USED_HOST, &devcfg, (spi_device_handle_t*)p_dev)
       * Si ESP_OK → el IDF rellena *p_dev con el handle opaco; MARCA device_state[flag] = SLOT_READY; retorna SPP_OK.
       * Si error → NO marca READY; retorna SPP_ERROR y main rompe el bucle (fallo duro).

   Iteración 2 (camino normal):
   main → p_dev = SPP_HAL_SPI_GetHandler()
   - Interno: ahora device_state típico es [READY, EMPTY] → devuelve &device_ids[1] (ICM).
   main → SPP_HAL_DeviceInit(p_dev)
   - Interno: como SLOT_EMPTY en flag=1 → añade device ICM; si éxito → estado [READY, READY].

   Iteración 3:
   main → p_dev = SPP_HAL_SPI_GetHandler()
   - Interno: ya no hay EMPTY → devuelve NULL; main rompe el bucle y continúa.

CASOS ESPECIALES CONTROLADOS (sin romper la ejecución):

A) Repetición de BMP (llega dos veces seguidas BMP)
   - Si ya se inicializó BMP antes:
       * DeviceInit(p_bmp) detecta device_state[0] == SLOT_READY → SPP_OK.
       * En la siguiente GetHandler(), se ofrecerá ICM (primer EMPTY), por lo que no bloquea al ICM.

B) Intento de tercer dispositivo
   - GetHandler() recorre y ve todos los slots READY → devuelve NULL.
   - main rompe el bucle de forma limpia (no hay hueco, no se intenta añadir un 3.º).

C) Fallo al añadir BMP (por hardware/IDF)
   - Iteración 1: DeviceInit(p_bmp) → spi_bus_add_device() falla → SPP_ERROR → main rompe (no seguimos).

D) Puntero ajeno a nuestros slots
   - DeviceInit(p) valida que p == &device_ids[j] para algún j∈[0..n_devices).
   - Si no coincide con ninguno → SPP_ERROR (estado interno intacto).

E) Idempotencia general
   - Si por cualquier razón DeviceInit() se llama sobre un slot ya READY → SPP_OK.
   - Esto garantiza que “llegadas repetidas” (BMP o ICM) no rompen ni alteran el estado.
*/
