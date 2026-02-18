#include "core/core.h"
#include "core/returntypes.h"

#include "spi.h"
#include "spp_log.h"
#include "osal/task.h"

#include "services/databank/databank.h"
#include "services/db_flow/db_flow.h"

#include "bmp_service.h"

static const char* TAG = "MAIN";

static bmp_data_t s_bmp;
static icm_data_t s_icm;

static void bmp_init_task(void *arg)
{
    BmpInit(arg);
    SPP_OSAL_TaskDelete(NULL);
}

void app_main(void)
{
    Core_Init();
    retval_t ret;

    Core_Init();
    SPP_LOGI(TAG, "Boot");

    ret = SPP_DATABANK_init();
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "Databank init failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    ret = DB_FLOW_Init();
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "DB_FLOW init failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    ret = SPP_HAL_SPI_BusInit();
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPI bus init failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    // Keep handler order: 1st ICM dummy, 2nd BMP
    void* p_spi_icm_dummy = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_spi_icm_dummy);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPI dev init ICM dummy failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    void* p_spi_bmp = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_spi_bmp);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPI dev init BMP failed");
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    SPP_LOGI(TAG, "BMP service init");
    ret = BMP_ServiceInit(p_spi_bmp);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "BMP_ServiceInit failed ret=%d", (int)ret);
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    SPP_LOGI(TAG, "BMP service start");
    ret = BMP_ServiceStart();
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "BMP_ServiceStart failed ret=%d", (int)ret);
        for (;;) { SPP_OSAL_TaskDelay(1000); }
    }

    //Step 8: Configure Interrupt Pin Settings
    s_bmp.int_pin       = (spp_uint32_t)INT_GPIO;
    s_bmp.int_intr_type = (spp_uint32_t)GPIO_INTR_POSEDGE;
    s_bmp.int_pull      = 0;

    //Step 9: Create BMP Initialization Task
    xTaskCreate(bmp_init_task, "bmp_init", BMP_INIT_TASK_STACK_SIZE, &s_bmp, BMP_INIT_PRIO, NULL);

    
    SPP_LOGI("APP", "Application starting...");
    /** Test all log levels */
    SPP_LOGE("TEST", "Error ejemplo");
    SPP_LOGW("TEST", "Warning ejemplo");
    SPP_LOGI("TEST", "Info ejemplo");
    SPP_LOGD("TEST", "Debug ejemplo");
    SPP_LOGV("TEST", "Verbose ejemplo");

    // Core_Init();

    // // Getting one SPP packet
    spp_packet_t *p_packet_1 = SPP_DATABANK_getPacket();
    p_packet_1->primary_header.version = 0xFA;
    ret = SPP_DATABANK_returnPacket(p_packet_1);

    // Following the logic this will have to return the same address of packet as p_packet_1
    spp_packet_t *p_packet_2 = SPP_DATABANK_getPacket();
    // We can check the new data is being written
    p_packet_2->primary_header.version = 0xFE;
    ret = SPP_DATABANK_returnPacket(p_packet_2);

    

    vTaskDelay(pdMS_TO_TICKS(100));

    // Step 10: Read Altitude Measurement
    float altitude = 0.0f;
    ret = bmp390_get_altitude(p_spi_bmp, &s_bmp, &altitude);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "Failed to read altitude from BMP390");
    }

    vTaskDelay(pdMS_TO_TICKS(50));

}