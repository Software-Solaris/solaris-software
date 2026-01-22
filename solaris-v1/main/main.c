#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "core/returntypes.h"
#include "spi.h"
#include "bmp390.h"

#include "gpio_int.h"      
#include "driver/gpio.h"  

static bmp_data_t s_bmp;

static void bmp_init_task(void *arg)
{
    (void)arg;
    BmpInit(&s_bmp);
    vTaskDelete(NULL); // not necessary because BmpInit deletes the task itself
}

void app_main(void)
{
    retval_t ret;

    // 1) SPI bus
    ret = SPP_HAL_SPI_BusInit();
    if (ret != SPP_OK) while (1) {}

    // 2) 1 DeviceInit -> ICM, 2 DeviceInit -> BMP
    void *p_spi_icm = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_spi_icm);
    if (ret != SPP_OK) while (1) {}

    void *p_spi_bmp = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_spi_bmp);
    if (ret != SPP_OK) while (1) {}

    // 3) Reset + SPI mode + check
    ret = bmp390_aux_config(p_spi_bmp);
    if (ret != SPP_OK) while (1) {}

    // 4) Config
    ret = bmp390_prepare_measure(p_spi_bmp);
    if (ret != SPP_OK) while (1) {}

    // 5) DRDY interrupt for BMP390:
    {
        spp_uint8_t buf[2] = { (spp_uint8_t)0x19u, (spp_uint8_t)(1u << 6) }; // 0x19 = register INT_CTRL ; make 1 the bit 6 (drdy_en)
        ret = SPP_HAL_SPI_Transmit(p_spi_bmp, buf, (spp_uint8_t)sizeof(buf));
        if (ret != SPP_OK) while (1) {}
    }

    // 6) Contexto fot BmpInit
    s_bmp.int_pin       = (spp_uint32_t)INT_GPIO;
    s_bmp.int_intr_type = (spp_uint32_t)GPIO_INTR_POSEDGE;
    s_bmp.int_pull      = 0; //0 none, 1 pullup, 2 pulldown

    // 7) BmpInit (EventGroup + ISR)
    xTaskCreate(bmp_init_task, "bmp_init", BMP_INIT_TASK_STACK_SIZE, NULL, BMP_INIT_PRIO, NULL);

    vTaskDelay(pdMS_TO_TICKS(100));

    volatile float altitude = 0.0f;

    // 8) One read
      ret = bmp390_get_altitude(p_spi_bmp, &s_bmp, (float*)&altitude);
      if (ret != SPP_OK) {
         while (1) {}
      }
}
