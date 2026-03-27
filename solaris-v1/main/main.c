#include "storage.h"
#include "spi.h"
#include "macros_esp.h" 
#include "spp_log.h"
#include "osal/task.h"

#include <stdio.h>
#include <string.h>

#define TAG "MAIN"

void app_main(void)
{
    SPP_OSAL_TaskDelay(2000);
    retval_t ret;

    // 1) SPI Bus Init
    ret = SPP_HAL_SPI_BusInit();
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "SPI Bus Init failed");
    }

    // 2) Config
    static SPP_Storage_InitCfg sd_cfg = {
        .base_path = "/sdcard",
        .spi_host_id = USED_HOST,
        .pin_cs = CS_PIN_SDC,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .format_if_mount_failed = false
    };

    // 3) Mount
    ret = SPP_HAL_Storage_Mount((void*)&sd_cfg);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "Storage Mount failed");
    }

    // 4) Write Test
    FILE* f = fopen("/sdcard/test.txt", "w");
    if (f == NULL) {
        (void)SPP_HAL_Storage_Unmount((void*)&sd_cfg);
        SPP_LOGE(TAG, "Failed to open file for writing");
    }

    const char* msg = "hola sd\n";
    size_t wr = fwrite(msg, 1, strlen(msg), f);
    fclose(f);

    if (wr != strlen(msg)) {
        (void)SPP_HAL_Storage_Unmount((void*)&sd_cfg);
        SPP_LOGE(TAG, "Failed to write to file");
    }

    // 5) Unmount
    ret = SPP_HAL_Storage_Unmount((void*)&sd_cfg);
    if (ret != SPP_OK) {
        SPP_LOGE(TAG, "Storage Unmount failed");
    }
}