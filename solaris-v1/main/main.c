#include "spp/spp.h"
//#include "spp/services/bmp390/bmp390.h"
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
 * BMP390 service (not used)
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
static const ICM20948_ServiceCfg_t s_icmCfg = {
    .spiDevIdx = 0U,   /* ICM20948 = SPI device index 0 */
    .intPin = 4U,      /* INT GPIO */
    .intIntrType = 1U, /* Rising edge */
    .intPull = 0U,     /* No pull */
};

/* ----------------------------------------------------------------
 * app_main
 * ---------------------------------------------------------------- */

void app_main(void)
{
    SPP_RetVal_t ret;

    /* ----------------------------------------------------------------
     * Core init
     * ---------------------------------------------------------------- */

    (void)SPP_CORE_setHalPort(&g_esp32HalPort);
    (void)SPP_CORE_init();

    SPP_LOGI(k_tag, "Solaris v1 boot");

    /* ----------------------------------------------------------------
     * SPI init
     * ---------------------------------------------------------------- */

    (void)SPP_HAL_spiBusInit();
    (void)SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(0U)); /* ICM20948 */
    (void)SPP_HAL_spiDeviceInit(SPP_HAL_spiGetHandle(1U)); /* BMP390   */

    /* ----------------------------------------------------------------
     * ICM20948 init
     * ---------------------------------------------------------------- */

    memset(&s_icmCtx, 0, sizeof(s_icmCtx));

    s_icmCtx.p_spi = SPP_HAL_spiGetHandle(s_icmCfg.spiDevIdx);
    s_icmCtx.seq = 0U;

    s_icmCtx.icmData.intPin = s_icmCfg.intPin;
    s_icmCtx.icmData.intIntrType = s_icmCfg.intIntrType;
    s_icmCtx.icmData.intPull = s_icmCfg.intPull;

    SPP_SERVICES_ICM20948_init(&s_icmCtx.icmData);

    ret = SPP_SERVICES_ICM20948_configDmpInit(s_icmCtx.p_spi);
    if (ret != K_SPP_OK)
    {
        SPP_LOGE(k_tag, "ICM20948 configDmpInit failed ret=%d", (int)ret);
        return;
    }

    SPP_LOGI(k_tag, "ICM20948 ready");

    /* ----------------------------------------------------------------
     * Kalman init
     * ---------------------------------------------------------------- */

    sensor_data data = {0};
    kalman_state kal;

    SPP_SERVICES_KALMAN_ekfInit(&kal, &data, 1.0f, NULL, NULL);

    SPP_LOGI(k_tag, "Services ready — entering superloop");

    /* ----------------------------------------------------------------
     * Superloop
     * ---------------------------------------------------------------- */

    for (;;)
    {
        float accx = 0.0f;
        float accy = 0.0f;
        float accz = 0.0f;
        float gyrox = 0.0f;
        float gyroy = 0.0f;
        float gyroz = 0.0f;

        ret = SPP_SERVICES_ICM20948_getMeasurements(&s_icmCtx, &accx, &accy, &accz, &gyrox, &gyroy,
                                                    &gyroz);
        if (ret != K_SPP_OK)
        {
            SPP_LOGE(k_tag, "Failed to get ICM20948 measurements ret=%d", (int)ret);
            continue;
        }

        data.acc_data[0] = accx;
        data.acc_data[1] = accy;
        data.acc_data[2] = accz;

        data.gyro_data[0] = gyrox * DEG_TO_RAD;
        data.gyro_data[1] = gyroy * DEG_TO_RAD;
        data.gyro_data[2] = gyroz * DEG_TO_RAD;

        const float dt = 1.0f / 225.0f;

        SPP_SERVICES_KALMAN_run(&kal, &data, dt);

        SPP_LOGI("EKF", "QUAT, (w=%.4f, x=%.4f, y=%.4f, z=%.4f)", kal.qw, kal.qx, kal.qy, kal.qz);
    }
}