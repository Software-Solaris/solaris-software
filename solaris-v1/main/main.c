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
    BmpInit(arg);
}

void app_main(void)
{
    retval_t ret;

    // Step 1: Initialize SPI Bus
    ret = SPP_HAL_SPI_BusInit();
    if (ret != SPP_OK) while (1) {}

    // Step 2: Initialize ICM SPI Device
    void *p_spi_icm = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_spi_icm);
    if (ret != SPP_OK) while (1) {}

    // Step 3: Initialize BMP SPI Device
    void *p_spi_bmp = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_spi_bmp);
    if (ret != SPP_OK) while (1) {}

    // Step 4: Assign SPI Handler to BMP Structure
    s_bmp.p_handler_spi = p_spi_bmp;

    // Step 5: Configure BMP390 Auxiliary Settings
    ret = bmp390_aux_config(p_spi_bmp);
    if (ret != SPP_OK) while (1) {}

    // Step 6: Prepare BMP390 Measurement
    ret = bmp390_prepare_measure(p_spi_bmp);
    if (ret != SPP_OK) while (1) {}

    // Step 7: Configure Control Register
    {
        spp_uint8_t buf[2] = { (spp_uint8_t)0x19u, (spp_uint8_t)(1u << 6) };
        ret = SPP_HAL_SPI_Transmit(p_spi_bmp, buf, (spp_uint8_t)sizeof(buf));
        if (ret != SPP_OK) while (1) {}
    }

    // Step 8: Configure Interrupt Pin Settings
    s_bmp.int_pin       = (spp_uint32_t)INT_GPIO;
    s_bmp.int_intr_type = (spp_uint32_t)GPIO_INTR_POSEDGE;
    s_bmp.int_pull      = 0;

    // Step 9: Create BMP Initialization Task
    xTaskCreate(bmp_init_task, "bmp_init", BMP_INIT_TASK_STACK_SIZE, &s_bmp, BMP_INIT_PRIO, NULL);

    vTaskDelay(pdMS_TO_TICKS(100));

    // Step 10: Read Altitude Measurement
    float altitude = 0.0f;
    ret = bmp390_get_altitude(p_spi_bmp, &s_bmp, &altitude);
    if (ret != SPP_OK) {
        while (1) {}
    }

    vTaskDelay(pdMS_TO_TICKS(50));

}