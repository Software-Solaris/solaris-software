#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "core/core.h"
#include "core/returntypes.h"

#include "spi.h"
#include "bmp390.h"

#include "spp_log.h"
#include "osal/eventgroups.h"
#include "osal/task.h"
#include "gpio_int.h"

static const char* TAG = "BMP_INT_TEST";

static bmp_data_t s_bmp;

static void bmp_int_test_task(void *arg)
{
    (void)arg;

    const spp_uint32_t window_ms = 1000;

    while (1)
    {
        spp_uint32_t count = 0;
        spp_uint32_t start_ms = (spp_uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        while (((spp_uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - start_ms) < window_ms)
        {
            osal_eventbits_t bits = 0;

            // Espera hasta 200 ms por una interrupción (si ODR=50Hz deberían llegar antes)
            retval_t ret = OSAL_EventGroupWaitBits(
                s_bmp.p_event_group,
                BMP390_EVT_DRDY,
                1,      // clear_on_exit
                0,      // wait_for_all_bits
                200,    // timeout_ms
                &bits
            );

            if ((ret == SPP_OK) && ((bits & BMP390_EVT_DRDY) != 0u)) {
                count++;
            }
        }

        SPP_LOGI(TAG, "INT events in last %u ms: %lu", (unsigned)window_ms, (unsigned long)count);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    retval_t ret;

    Core_Init();
    SPP_LOGI(TAG, "Starting BMP390 INT test...");

    // 1) SPI bus init
    ret = SPP_HAL_SPI_BusInit();
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPP_HAL_SPI_BusInit failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // 2) SPI device init
    // HAL asigna handlers por orden:
    // - 1ª llamada -> ICM (dummy aquí)
    // - 2ª llamada -> BMP
    void *p_spi_icm_dummy = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_spi_icm_dummy);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPP_HAL_SPI_DeviceInit(ICM dummy) failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    void *p_spi_bmp = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_spi_bmp);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPP_HAL_SPI_DeviceInit(BMP) failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // 3) Config BMP (reset + SPI mode + check ID)
    ret = bmp390_aux_config(p_spi_bmp);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "bmp390_aux_config failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // 4) Setup interrupt context + register ISR (antes de habilitar DRDY)
    s_bmp.int_pin       = (spp_uint32_t)INT_GPIO; // D2 -> GPIO5 (confirmado)
    s_bmp.int_intr_type = 1;                      // POSEDGE
    s_bmp.int_pull      = 0;                      // none (push-pull del BMP390)
    BmpInit(&s_bmp);

    // 5) Prepare measure (ODR/mode)
    ret = bmp390_prepare_measure(p_spi_bmp);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "bmp390_prepare_measure failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // 6) Enable DRDY on INT pin (INT_CTRL.drdy_en = 1)
    ret = bmp390_int_enable_drdy(p_spi_bmp);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "bmp390_int_enable_drdy failed");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // 7) Start counting interrupts
    xTaskCreate(bmp_int_test_task, "bmp_int_test", 4096, NULL, 5, NULL);

    // app_main no debe salir
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}