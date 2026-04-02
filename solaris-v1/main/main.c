#include "storage.h"
#include "spi.h"
#include "macros_esp.h"
#include "osal/task.h"

#include "esp_log.h"

#include <stdio.h>
#include <string.h>

#define TAG "MAIN"

void app_main(void)
{
    retval_t ret;

    SPP_OSAL_TaskDelay(2000);

    static SPP_Storage_InitCfg sd_cfg = {
        .base_path = "/sdcard",
        .spi_host_id = USED_HOST,
        .pin_cs = CS_PIN_SDC,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .format_if_mount_failed = false
    };

    ESP_LOGI(TAG, "Inicio app_main");

    ret = SPP_HAL_SPI_BusInit();
    if (ret != SPP_OK) {
        ESP_LOGE(TAG, "SPI Bus Init failed");
        return;
    }
    ESP_LOGI(TAG, "SPI Bus Init OK");

    ret = SPP_HAL_Storage_Mount((void *)&sd_cfg);
    if (ret != SPP_OK) {
        ESP_LOGE(TAG, "Storage Mount failed");
        return;
    }
    ESP_LOGI(TAG, "Storage Mount OK");

    FILE *f = fopen("/sdcard/test.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        (void)SPP_HAL_Storage_Unmount((void *)&sd_cfg);
        return;
    }
    ESP_LOGI(TAG, "File opened OK");

    const char *msg = "hola sd\n";
    size_t wr = fwrite(msg, 1, strlen(msg), f);
    fclose(f);

    if (wr != strlen(msg)) {
        ESP_LOGE(TAG, "Failed to write to file");
        (void)SPP_HAL_Storage_Unmount((void *)&sd_cfg);
        return;
    }
    ESP_LOGI(TAG, "Write test OK");

    ret = SPP_HAL_Storage_Unmount((void *)&sd_cfg);
    if (ret != SPP_OK) {
        ESP_LOGE(TAG, "Storage Unmount failed");
        return;
    }

    ESP_LOGI(TAG, "Storage Unmount OK");
    ESP_LOGI(TAG, "Fin app_main");
}