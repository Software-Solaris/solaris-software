#include "icm20948.h"
#include "driver/spi_common.h"
#include "returntypes.h"
#include "spi.h"
#include "task.h"
#include "types.h"
#include <string.h>

/* ── DMP firmware image ──────────────────────────────────────────────── */
static const spp_uint8_t dmp3_image[] = {
    #include "icm20948_img.dmp3a.h"
};

/* ── Helper: write a single register ─────────────────────────────────── */
static retval_t IcmWriteReg(void *spi, spp_uint8_t reg, spp_uint8_t val)
{
    spp_uint8_t buf[2] = { WRITE_OP | reg, val };
    return SPP_HAL_SPI_Transmit(spi, buf, 2);
}

/* ── Helper: read a single register ──────────────────────────────────── */
static retval_t IcmReadReg(void *spi, spp_uint8_t reg, spp_uint8_t *val)
{
    spp_uint8_t buf[2] = { READ_OP | reg, EMPTY_MESSAGE };
    retval_t ret = SPP_HAL_SPI_Transmit(spi, buf, 2);
    *val = buf[1];
    return ret;
}

/* ── Helper: select register bank ────────────────────────────────────── */
static retval_t IcmSetBank(void *spi, spp_uint8_t bank)
{
    return IcmWriteReg(spi, REG_BANK_SEL, bank);
}

/* ── Helper: reset FIFO (assert then deassert) ───────────────────────── */
static retval_t IcmResetFifo(void *spi)
{
    retval_t ret = IcmWriteReg(spi, REG_FIFO_RST, 0x1F);
    if (ret != SPP_OK) return ret;

    return IcmWriteReg(spi, REG_FIFO_RST, 0x1E);
}

/* ── DMP memory write: N bytes, setting bank+address per byte ────────── */
static retval_t IcmDmpWriteBytes(icm_data_t        *p,
                                  spp_uint16_t       addr,
                                  const spp_uint8_t *bytes,
                                  spp_uint8_t        len)
{
    retval_t ret;

    for (spp_uint8_t i = 0; i < len; i++)
    {
        ret = IcmWriteReg(p->p_handler_spi, REG_MEM_BANK_SEL, (spp_uint8_t)((addr + i) >> 8));
        if (ret != SPP_OK) return ret;

        ret = IcmWriteReg(p->p_handler_spi, REG_MEM_START_ADDR, (spp_uint8_t)((addr + i) & 0xFF));
        if (ret != SPP_OK) return ret;

        ret = IcmWriteReg(p->p_handler_spi, REG_MEM_R_W, bytes[i]);
        if (ret != SPP_OK) return ret;
    }

    return SPP_OK;
}

static retval_t IcmDmpWrite32(icm_data_t *p, spp_uint16_t addr, spp_uint32_t v)
{
    spp_uint8_t b[4] = { v >> 24, v >> 16, v >> 8, v };
    return IcmDmpWriteBytes(p, addr, b, 4);
}

static retval_t IcmDmpWrite16(icm_data_t *p, spp_uint16_t addr, spp_uint16_t v)
{
    spp_uint8_t b[2] = { v >> 8, v };
    return IcmDmpWriteBytes(p, addr, b, 2);
}

/* ── LP wake cycle: PWR_MGMT_1 = 0x21 → 0x01 ───────────────────────── */
static retval_t IcmLpWakeCycle(icm_data_t *p)
{
    retval_t ret = IcmWriteReg(p->p_handler_spi, REG_PWR_MGMT_1, 0x21);
    if (ret != SPP_OK) return ret;

    return IcmWriteReg(p->p_handler_spi, REG_PWR_MGMT_1, 0x01);
}

/* ── GYRO_SF calculation (AN-MAPPS Appendix II) ─────────────────────── */
#define BASE_SAMPLE_RATE  1125
#define DMP_RUNNING_RATE  225
#define DMP_DIVIDER       (BASE_SAMPLE_RATE / DMP_RUNNING_RATE)

static spp_int32_t IcmCalcGyroSf(spp_int8_t pll)
{
    spp_int32_t t = 102870L + 81L * (spp_int32_t)pll;
    spp_int32_t a = (1L << 30) / t;
    spp_int32_t r = (1L << 30) - a * t;
    spp_int32_t v = a * 797 * DMP_DIVIDER;

    v += (spp_int32_t)(((spp_int64_t)a * 1011387LL * DMP_DIVIDER) >> 20);
    v += r * 797L * DMP_DIVIDER / t;
    v += (spp_int32_t)(((spp_int64_t)r * 1011387LL * DMP_DIVIDER) >> 20) / t;

    return v << 1;
}

/* ── Helper: write DMP output config block ───────────────────────────── */
static retval_t IcmDmpWriteOutputConfig(icm_data_t  *p,
                                         spp_uint16_t out_ctl1,
                                         spp_uint16_t motion_event)
{
    retval_t ret;

    ret = IcmDmpWrite16(p, DMP_DATA_OUT_CTL1, out_ctl1);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p, DMP_DATA_INTR_CTL, out_ctl1);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p, DMP_DATA_OUT_CTL2, 0x0000);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p, DMP_MOTION_EVENT_CTL, motion_event);
    if (ret != SPP_OK) return ret;

    return SPP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════
   PUBLIC API
   ═══════════════════════════════════════════════════════════════════════ */

retval_t IcmInit(void *p_data)
{
    icm_data_t *p = (icm_data_t *)p_data;
    void *spi = SPP_HAL_SPI_GetHandler();

    retval_t ret = SPP_HAL_SPI_DeviceInit(spi);
    if (ret != SPP_OK) return ret;

    p->p_handler_spi = spi;
    return SPP_OK;
}

/* ── Load DMP firmware into SRAM at 0x0090, then verify ──────────────── */
retval_t IcmLoadDmp(void *p_data)
{
    icm_data_t        *p       = (icm_data_t *)p_data;
    retval_t           ret;
    spp_uint16_t       fw_size = sizeof(dmp3_image);
    spp_uint16_t       addr    = DMP_LOAD_START;
    const spp_uint8_t *fw      = dmp3_image;

    ret = IcmSetBank(p->p_handler_spi, REG_BANK_0);
    if (ret != SPP_OK) return ret;

    /* Write */
    for (spp_uint16_t i = 0; i < fw_size; i++, addr++)
    {
        ret = IcmWriteReg(p->p_handler_spi, REG_MEM_BANK_SEL, (spp_uint8_t)(addr >> 8));
        if (ret != SPP_OK) return ret;

        ret = IcmWriteReg(p->p_handler_spi, REG_MEM_START_ADDR, (spp_uint8_t)(addr & 0xFF));
        if (ret != SPP_OK) return ret;

        ret = IcmWriteReg(p->p_handler_spi, REG_MEM_R_W, fw[i]);
        if (ret != SPP_OK) return ret;
    }

    /* Verify */
    addr = DMP_LOAD_START;
    for (spp_uint16_t i = 0; i < fw_size; i++, addr++)
    {
        ret = IcmWriteReg(p->p_handler_spi, REG_MEM_BANK_SEL, (spp_uint8_t)(addr >> 8));
        if (ret != SPP_OK) return ret;

        ret = IcmWriteReg(p->p_handler_spi, REG_MEM_START_ADDR, (spp_uint8_t)(addr & 0xFF));
        if (ret != SPP_OK) return ret;

        spp_uint8_t rd;
        ret = IcmReadReg(p->p_handler_spi, REG_MEM_R_W, &rd);
        if (ret != SPP_OK) return ret;

        if (rd != fw[i]) return SPP_ERROR;
    }

    p->firmware_loaded = true;
    return SPP_OK;
}

/* ── Full DMP initialization ─────────────────────────────────────────── */
retval_t IcmConfigDmpInit(void *p_data)
{
    icm_data_t *p   = (icm_data_t *)p_data;
    void       *spi = p->p_handler_spi;
    retval_t    ret;
    spp_uint8_t val;

    /* ── 1. WHO_AM_I, clock, SPI-only ──────────────────────────────── */
    ret = IcmSetBank(spi, REG_BANK_0);
    if (ret != SPP_OK) return ret;

    ret = IcmReadReg(spi, REG_WHO_AM_I, &val);
    if (ret != SPP_OK) return ret;
    if (val != 0xEA) return SPP_ERROR;

    ret = IcmWriteReg(spi, REG_PWR_MGMT_1, 0x01);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_USER_CTRL, 0x10);
    if (ret != SPP_OK) return ret;

    /* ── 2. Initial power config for firmware load ─────────────────── */
    ret = IcmWriteReg(spi, REG_PWR_MGMT_2, 0x47);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_LP_CONF, 0x70);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_USER_CTRL, 0x10);
    if (ret != SPP_OK) return ret;

    /* ── 3. Load and verify DMP firmware ───────────────────────────── */
    ret = IcmLoadDmp(p_data);
    if (ret != SPP_OK) return ret;

    /* ── 4. DMP execution start address = 0x1000 ──────────────────── */
    ret = IcmSetBank(spi, REG_BANK_2);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_DMP_ADDR_MSB, 0x10);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_DMP_ADDR_LSB, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmSetBank(spi, REG_BANK_0);
    if (ret != SPP_OK) return ret;

    /* ── 5. Clear DMP control registers ────────────────────────────── */
    ret = IcmDmpWriteOutputConfig(p, 0x0000, 0x0000);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p, DMP_DATA_RDY_STATUS, 0x0000);
    if (ret != SPP_OK) return ret;

    /* ── 6. FIFO watermark ─────────────────────────────────────────── */
    ret = IcmDmpWrite16(p, DMP_FIFO_WATERMARK, 800U);
    if (ret != SPP_OK) return ret;

    /* ── 7. Interrupt config ───────────────────────────────────────── */
    ret = IcmWriteReg(spi, REG_INT_ENABLE, 0x02);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_INT_ENABLE_2, 0x01);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_SINGLE_FIFO_PRIORITY_SEL, 0xE4);
    if (ret != SPP_OK) return ret;

    /* ── 8. HW_FIX_DISABLE ─────────────────────────────────────────── */
    ret = IcmWriteReg(spi, REG_HW_FIX_DISABLE, 0x48);
    if (ret != SPP_OK) return ret;

    /* ── 9. Initial ODR: 1125/(1+19) = 56.25 Hz ───────────────────── */
    ret = IcmSetBank(spi, REG_BANK_2);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_GYRO_SMPLRT_DIV, 0x13);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_ACCEL_SMPLRT_DIV_1, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_ACCEL_SMPLRT_DIV_2, 0x13);
    if (ret != SPP_OK) return ret;

    ret = IcmSetBank(spi, REG_BANK_0);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p, DMP_BAC_RATE, 0x0000);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p, DMP_B2S_RATE, 0x0000);
    if (ret != SPP_OK) return ret;

    /* ── 10. FIFO config and reset ─────────────────────────────────── */
    ret = IcmWriteReg(spi, REG_FIFO_CFG, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmResetFifo(spi);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_FIFO_EN_1, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_FIFO_EN_2, 0x00);
    if (ret != SPP_OK) return ret;

    /* ── 11. Power cycle: LP → sleep → wake ────────────────────────── */
    ret = IcmWriteReg(spi, REG_PWR_MGMT_1, 0x21);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_PWR_MGMT_2, 0x7F);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_PWR_MGMT_1, 0x61);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));

    ret = IcmWriteReg(spi, REG_PWR_MGMT_1, 0x21);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));

    ret = IcmWriteReg(spi, REG_PWR_MGMT_1, 0x01);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));

    /* ── 12. AK09916 WHO_AM_I check via I2C master ────────────────── */
    ret = IcmSetBank(spi, REG_BANK_3);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x05, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x09, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x0D, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x11, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x01, 0x10);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x00, 0x04);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x03, 0x8C);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x04, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x05, 0x81);
    if (ret != SPP_OK) return ret;

    ret = IcmSetBank(spi, REG_BANK_0);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_USER_CTRL, 0x30);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(60));

    ret = IcmWriteReg(spi, REG_USER_CTRL, 0x10);
    if (ret != SPP_OK) return ret;

    /* ── 13. AK09916 power-down via SLV1 ──────────────────────────── */
    ret = IcmSetBank(spi, REG_BANK_3);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x05, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x07, 0x0C);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x08, 0x31);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x0A, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x09, 0x81);
    if (ret != SPP_OK) return ret;

    ret = IcmSetBank(spi, REG_BANK_0);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_USER_CTRL, 0x30);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(60));

    ret = IcmWriteReg(spi, REG_USER_CTRL, 0x10);
    if (ret != SPP_OK) return ret;

    ret = IcmSetBank(spi, REG_BANK_3);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x09, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x05, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x09, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmSetBank(spi, REG_BANK_0);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_USER_CTRL, 0x10);
    if (ret != SPP_OK) return ret;

    /* ── 14. Compass mounting matrix ──────────────────────────────── */
    static const struct { spp_uint16_t addr; spp_uint32_t val; } cpass_mtx[] = {
        { DMP_CPASS_MTX_00, 0x09999999 }, { DMP_CPASS_MTX_01, 0x00000000 },
        { DMP_CPASS_MTX_02, 0x00000000 }, { DMP_CPASS_MTX_10, 0x00000000 },
        { DMP_CPASS_MTX_11, 0xF6666667 }, { DMP_CPASS_MTX_12, 0x00000000 },
        { DMP_CPASS_MTX_20, 0x00000000 }, { DMP_CPASS_MTX_21, 0x00000000 },
        { DMP_CPASS_MTX_22, 0xF6666667 },
    };
    for (spp_uint8_t i = 0; i < 9; i++)
    {
        ret = IcmDmpWrite32(p, cpass_mtx[i].addr, cpass_mtx[i].val);
        if (ret != SPP_OK) return ret;

        ret = IcmLpWakeCycle(p);
        if (ret != SPP_OK) return ret;
    }
    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));

    /* ── 15. B2S mounting matrix (identity) ───────────────────────── */
    static const struct { spp_uint16_t addr; spp_uint32_t val; } b2s_mtx[] = {
        { DMP_B2S_MTX_00, 0x40000000 }, { DMP_B2S_MTX_01, 0x00000000 },
        { DMP_B2S_MTX_02, 0x00000000 }, { DMP_B2S_MTX_10, 0x00000000 },
        { DMP_B2S_MTX_11, 0x40000000 }, { DMP_B2S_MTX_12, 0x00000000 },
        { DMP_B2S_MTX_20, 0x00000000 }, { DMP_B2S_MTX_21, 0x00000000 },
        { DMP_B2S_MTX_22, 0x40000000 },
    };
    for (spp_uint8_t i = 0; i < 9; i++)
    {
        ret = IcmDmpWrite32(p, b2s_mtx[i].addr, b2s_mtx[i].val);
        if (ret != SPP_OK) return ret;

        ret = IcmLpWakeCycle(p);
        if (ret != SPP_OK) return ret;
    }
    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));

    /* ── 16. Accel config: ±4g, bypass DLPF ───────────────────────── */
    ret = IcmSetBank(spi, REG_BANK_2);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_ACCEL_CONFIG, 0x02);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_ACCEL_CONFIG_2, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmSetBank(spi, REG_BANK_0);
    if (ret != SPP_OK) return ret;

    ret = IcmLpWakeCycle(p);
    if (ret != SPP_OK) return ret;

    /* DMP accel scale for ±4g */
    ret = IcmDmpWrite32(p, DMP_ACC_SCALE, 0x04000000);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p, DMP_ACC_SCALE2, 0x00040000);
    if (ret != SPP_OK) return ret;

    ret = IcmLpWakeCycle(p);
    if (ret != SPP_OK) return ret;

    /* ── 17. Gyro config: ±2000dps, DLPF on ──────────────────────── */
    ret = IcmSetBank(spi, REG_BANK_2);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_GYRO_CONFIG, 0x07);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x02, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmSetBank(spi, REG_BANK_0);
    if (ret != SPP_OK) return ret;

    ret = IcmLpWakeCycle(p);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p, DMP_GYRO_FULLSCALE, 0x10000000);
    if (ret != SPP_OK) return ret;

    ret = IcmLpWakeCycle(p);
    if (ret != SPP_OK) return ret;

    /* ── 18. Read PLL trim, calculate GYRO_SF ─────────────────────── */
    ret = IcmSetBank(spi, REG_BANK_1);
    if (ret != SPP_OK) return ret;

    spp_uint8_t pll_raw;
    ret = IcmReadReg(spi, REG_TIMEBASE_CORRECTION_PLL, &pll_raw);
    if (ret != SPP_OK) return ret;

    spp_int8_t pll = (spp_int8_t)pll_raw;

    ret = IcmSetBank(spi, REG_BANK_0);
    if (ret != SPP_OK) return ret;

    ret = IcmLpWakeCycle(p);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p, DMP_GYRO_SF, (spp_uint32_t)IcmCalcGyroSf(pll));
    if (ret != SPP_OK) return ret;

    ret = IcmLpWakeCycle(p);
    if (ret != SPP_OK) return ret;

    /* ── 19. Continuous mode: only I2C master duty-cycled ─────────── */
    ret = IcmWriteReg(spi, REG_LP_CONF, 0x40);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_PWR_MGMT_1, 0x01);
    if (ret != SPP_OK) return ret;

    /* ── 20. Three DMP enable/clear sequences ─────────────────────── */
    for (spp_uint8_t seq = 0; seq < 3; seq++)
    {
        ret = IcmWriteReg(spi, REG_USER_CTRL, (seq == 0) ? 0x10 : 0xD0);
        if (ret != SPP_OK) return ret;

        ret = IcmWriteReg(spi, REG_PWR_MGMT_2, 0x47);
        if (ret != SPP_OK) return ret;

        ret = IcmWriteReg(spi, REG_USER_CTRL, 0xD0);
        if (ret != SPP_OK) return ret;

        ret = IcmDmpWriteOutputConfig(p, 0x0000, 0x0000);
        if (ret != SPP_OK) return ret;

        if (seq < 2)
        {
            ret = IcmWriteReg(spi, REG_PWR_MGMT_2, 0x7F);
            if (ret != SPP_OK) return ret;

            ret = IcmWriteReg(spi, REG_PWR_MGMT_1, 0x41);
            if (ret != SPP_OK) return ret;

            SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));

            ret = IcmWriteReg(spi, REG_PWR_MGMT_1, 0x01);
            if (ret != SPP_OK) return ret;

            ret = IcmWriteReg(spi, REG_PWR_MGMT_2, 0x7F);
            if (ret != SPP_OK) return ret;

            ret = IcmDmpWrite16(p, DMP_DATA_RDY_STATUS, 0x0000);
            if (ret != SPP_OK) return ret;

            ret = IcmLpWakeCycle(p);
            if (ret != SPP_OK) return ret;
        }
    }

    /* ── 21. DMP output config: accel + gyro + compass (16-bit) ───── */
    ret = IcmDmpWriteOutputConfig(p, 0xE000, 0x0300);
    if (ret != SPP_OK) return ret;

    /* ── 22. Accel calibration for 225 Hz ─────────────────────────── */
    ret = IcmDmpWrite32(p, DMP_ACCEL_ONLY_GAIN, 0x00E8BA2E);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p, DMP_ACCEL_ALPHA_VAR, 0x3D27D27D);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p, DMP_ACCEL_A_VAR, 0x02D82D83);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p, DMP_ACCEL_CAL_INIT, 0x0000);
    if (ret != SPP_OK) return ret;

    /* ── 23. Final ODR: 1125/(1+4) = 225 Hz ──────────────────────── */
    ret = IcmSetBank(spi, REG_BANK_2);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_ACCEL_SMPLRT_DIV_1, 0x00);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_ACCEL_SMPLRT_DIV_2, 0x04);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, REG_GYRO_SMPLRT_DIV, 0x04);
    if (ret != SPP_OK) return ret;

    ret = IcmSetBank(spi, REG_BANK_0);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p, DMP_ODR_QUAT6, 0x0000);
    if (ret != SPP_OK) return ret;

    /* Recalculate GYRO_SF for 225 Hz */
    ret = IcmDmpWrite32(p, DMP_GYRO_SF, (spp_uint32_t)IcmCalcGyroSf(pll));
    if (ret != SPP_OK) return ret;

    /* ── 24. Enable sensors, set DATA_RDY_STATUS ──────────────────── */
    ret = IcmWriteReg(spi, REG_PWR_MGMT_2, 0x40);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p, DMP_DATA_RDY_STATUS, 0x000B);
    if (ret != SPP_OK) return ret;

    /* ── 25. Configure I2C SLV0/SLV1 for continuous magnetometer ──── */
    ret = IcmSetBank(spi, REG_BANK_3);
    if (ret != SPP_OK) return ret;

    /* SLV0: read 10 bytes from AK09916 starting at RSV2 (0x03) */
    ret = IcmWriteReg(spi, 0x03, 0x8C);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x04, 0x03);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x05, 0xDA);
    if (ret != SPP_OK) return ret;

    /* SLV1: write Single Measurement to AK09916 CNTL2 (0x31) */
    ret = IcmWriteReg(spi, 0x07, 0x0C);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x08, 0x31);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x0A, 0x01);
    if (ret != SPP_OK) return ret;

    ret = IcmWriteReg(spi, 0x09, 0x81);
    if (ret != SPP_OK) return ret;

    /* I2C Master ODR: 1100/2^4 = 68.75Hz */
    ret = IcmWriteReg(spi, 0x00, 0x04);
    if (ret != SPP_OK) return ret;

    ret = IcmSetBank(spi, REG_BANK_0);
    if (ret != SPP_OK) return ret;

    /* ── 26. Final USER_CTRL: DMP + FIFO + I2C_MST + SPI ─────────── */
    ret = IcmWriteReg(spi, REG_USER_CTRL, 0xF0);
    if (ret != SPP_OK) return ret;

    /* ── 27. Reset DMP and FIFO ───────────────────────────────────── */
    ret = IcmWriteReg(spi, REG_USER_CTRL, 0xF8);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));

    ret = IcmResetFifo(spi);
    if (ret != SPP_OK) return ret;

    /* ── 28. Clock source without LP_EN ───────────────────────────── */
    ret = IcmWriteReg(spi, REG_PWR_MGMT_1, 0x01);
    if (ret != SPP_OK) return ret;

    /* ── 29. Final DMP config (after all resets) ──────────────────── */
    ret = IcmDmpWriteOutputConfig(p, 0xE000, 0x0300);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p, DMP_DATA_RDY_STATUS, 0x000B);
    if (ret != SPP_OK) return ret;

    /* ── 30. Clean FIFO start ─────────────────────────────────────── */
    ret = IcmResetFifo(spi);
    if (ret != SPP_OK) return ret;

    return SPP_OK;
}