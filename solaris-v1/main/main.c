#include "spp/spp.h"
#include "spp/services/bmp390/bmp390.h"
#include "spp/services/icm20948/icm20948.h"
#include "spp/services/datalogger/datalogger.h"
#include "spp/services/kalman/kalman.h"
#include <stdio.h>
#include <string.h>

/* ----------------------------------------------------------------
 * HAL port (provided by the spp_ports component)
 * ---------------------------------------------------------------- */

extern const SPP_HalPort_t g_esp32HalPort;

/* ----------------------------------------------------------------
 * Private constants
 * ---------------------------------------------------------------- */

static const char *const k_tag = "MAIN";

/* ----------------------------------------------------------------
 * BMP390 service
 * ---------------------------------------------------------------- */

// static BMP390_ServiceCtx_t s_bmpCtx;
// static const BMP390_ServiceCfg_t s_bmpCfg = {
//     .spiDevIdx = 1U,   /* BMP390 = SPI device index 1 */
//     .intPin = 5U,      /* DRDY GPIO */
//     .intIntrType = 1U, /* Rising edge */
//     .intPull = 0U,     /* No pull */
// };

/* ----------------------------------------------------------------
 * ICM20948 service
 * ---------------------------------------------------------------- */

static ICM20948_ServiceCtx_t s_icmCtx;
// static const ICM20948_ServiceCfg_t s_icmCfg = {
//     .spiDevIdx = 0U,   /* ICM20948 = SPI device index 0 */
//     .intPin = 4U,      /* INT GPIO */
//     .intIntrType = 1U, /* Rising edge */
//     .intPull = 0U,     /* No pull */
// };

/* ----------------------------------------------------------------
 * app_main
 * ---------------------------------------------------------------- */

void app_main(void)
{
    /* 1. Register HAL port and initialise core
     *    (SPP_CORE_init also calls SPP_SERVICES_DATABANK_init + SPP_SERVICES_PUBSUB_init) */
    (void)SPP_CORE_setHalPort(&g_esp32HalPort);
    (void)SPP_CORE_init();

    SPP_LOGI(k_tag, "Solaris v1 boot");

    /* 2. Initialise SPI bus and devices (ICM first, then BMP).
     *    Must come before SD card mount which uses the same SPI host. */
    (void)SPP_HAL_spiBusInit();
    (void)SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(0U)); /* ICM20948 */
    (void)SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(1U)); /* BMP390   */


    SPP_LOGI(k_tag, "Services ready — entering superloop");

    /* ----------------------------------------------------------------
     * Kalman init
     * ---------------------------------------------------------------- */

    sensor_data data = {0};
    kalman_state kal;
    //SPP_SERVICES_KALMAN_ekfInit(&kal, &data, 1.0f, NULL, NULL);
    /* ----------------------------------------------------------------
     * Superloop
     * ---------------------------------------------------------------- */
    for (;;)
    {
        float accx, accy, accz, gyrox, gyroy, gyroz;
        SPP_RetVal_t ret = SPP_SERVICES_ICM20948_getMeasurements(&s_icmCtx, &accx, &accy, &accz,
                                                                 &gyrox, &gyroy, &gyroz);
        if (ret != K_SPP_OK)
        {
            SPP_LOGE(k_tag, "Failed to get ICM20948 measurements ret=%d", (int)ret);
        }

        data.acc_data[0] = accx;
        data.acc_data[1] = accy;
        data.acc_data[2] = accz;

        data.gyro_data[0] = gyrox;
        data.gyro_data[1] = gyroy;
        data.gyro_data[2] = gyroz;

        data.acc_new_data = 1;
        data.gyro_new_data = 1;
        // dt = 1/gyro_sample_rate
        // ret = SPP_SERVICES_ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_ODR_GYRO,  0x0000U) -> gyro sample rate = dmp sample rate
        // dmp sample rate = 225Hz -> dt = 1/225s
        float dt = 1.0f / 225.0f;
        SPP_SERVICES_KALMAN_run(&kal, &data, dt);
    }
}
