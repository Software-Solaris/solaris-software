#include "storage.h"
#include "spi.h"
#include "macros_esp.h" 

#include <stdio.h>
#include <string.h>

void app_main(void)
{
    retval_t ret;

    // 1) SPI Bus Init
    ret = SPP_HAL_SPI_BusInit();
    if (ret != SPP_OK) {
        while (1) { }
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
        while (1) { }
    }

    // 4) Write Test
    FILE* f = fopen("/sdcard/test.txt", "w");
    if (f == NULL) {
        (void)SPP_HAL_Storage_Unmount((void*)&sd_cfg);
        while (1) { }
    }

    const char* msg = "hola sd\n";
    size_t wr = fwrite(msg, 1, strlen(msg), f);
    fclose(f);

    if (wr != strlen(msg)) {
        (void)SPP_HAL_Storage_Unmount((void*)&sd_cfg);
        while (1) { }
    }

    // 5) Unmount
    ret = SPP_HAL_Storage_Unmount((void*)&sd_cfg);
    if (ret != SPP_OK) {
        while (1) { }
    }

    while (1) { }
}