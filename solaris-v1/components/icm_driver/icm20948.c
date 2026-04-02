#include "icm20948.h"
#include "macros.h"
#include "spp/core/returntypes.h"
#include "spp/hal/spi/spi.h"
#include "spp/osal/task.h"
#include "spp/core/types.h"

#include <string.h>
#include "spp/services/logging/spp_log.h"
#include "math.h"

/* ----------------------------------------------------------------
 * Private constants
 * ---------------------------------------------------------------- */

/**
 * @brief Base sensor sample rate in Hz used by the DMP gyro scale factor
 *        computation.
 */
#define K_ICM20948_BASE_SAMPLE_RATE 1125L

/**
 * @brief DMP running rate in Hz.
 */
#define K_ICM20948_DMP_RUNNING_RATE 225L

/**
 * @brief Divider between the base sample rate and the DMP running rate.
 */
#define K_ICM20948_DMP_DIVIDER (K_ICM20948_BASE_SAMPLE_RATE / K_ICM20948_DMP_RUNNING_RATE)

/**
 * @brief Expected WHO_AM_I value for the ICM20948.
 */
#define K_ICM20948_WHO_AM_I_VALUE 0xEAU

/**
 * @brief FIFO packet size used by the selected DMP output configuration.
 */
#define K_ICM20948_DMP_PACKET_SIZE_BYTES 42U

/**
 * @brief Maximum FIFO count before forcing a FIFO reset.
 */
#define K_ICM20948_FIFO_RESET_THRESHOLD 512U

/**
 * @brief DMP start address MSB.
 */
#define K_ICM20948_DMP_START_ADDR_MSB 0x10U

/**
 * @brief DMP start address LSB.
 */
#define K_ICM20948_DMP_START_ADDR_LSB 0x00U

/**
 * @brief Tag used in log traces.
 */
#define K_ICM20948_LOG_TAG "ICM"

/* ----------------------------------------------------------------
 * Private types
 * ---------------------------------------------------------------- */

/**
 * @brief Address/value pair used for DMP matrix initialization.
 */
typedef struct
{
    spp_uint16_t addr; /**< DMP memory address. */
    spp_uint32_t val;  /**< Value to write. */
} ICM20948_DmpMatrixEntry_t;

/* ----------------------------------------------------------------
 * DMP firmware image
 * ---------------------------------------------------------------- */

/**
 * @brief DMP firmware image to be loaded into the device SRAM.
 */
static const spp_uint8_t s_dmp3Image[] = {
#include "icm20948_img.dmp3a.h"
};

/* ----------------------------------------------------------------
 * Private helpers
 * ---------------------------------------------------------------- */

/**
 * @brief Writes a raw byte value to an ICM20948 register over SPI.
 *
 * @param[in] p_spi Pointer to the SPI handler.
 * @param[in] reg Register address.
 * @param[in] value Raw byte value to write.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
static retval_t ICM20948_writeReg(void *p_spi, spp_uint8_t reg, spp_uint8_t value)
{
    spp_uint8_t txBuffer[2] = {K_WRITE_OP | reg, value};
    return SPP_HAL_SPI_Transmit(p_spi, txBuffer, 2U);
}

/**
 * @brief Reads a raw byte value from an ICM20948 register over SPI.
 *
 * @param[in]  p_spi Pointer to the SPI handler.
 * @param[in]  reg Register address.
 * @param[out] p_value Pointer where the read value will be stored.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
static retval_t ICM20948_readReg(void *p_spi, spp_uint8_t reg, spp_uint8_t *p_value)
{
    spp_uint8_t txRxBuffer[2] = {K_READ_OP | reg, K_WRITE_OP};
    retval_t ret = SPP_HAL_SPI_Transmit(p_spi, txRxBuffer, 2U);

    if (p_value != NULL)
    {
        *p_value = txRxBuffer[1];
    }

    return ret;
}

/**
 * @brief Selects the active register bank.
 *
 * @param[in] p_spi Pointer to the SPI handler.
 * @param[in] regBank Register bank to select.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
static retval_t ICM20948_setBank(void *p_spi, ICM20948_RegBank_t regBank)
{
    ICM20948_RegBankSel_t regBankSel = {.value = 0U};

    switch (regBank)
    {
        case K_ICM20948_REG_BANK_0:
            regBankSel.bits.bankSel = 0U;
            break;

        case K_ICM20948_REG_BANK_1:
            regBankSel.bits.bankSel = 1U;
            break;

        case K_ICM20948_REG_BANK_2:
            regBankSel.bits.bankSel = 2U;
            break;

        case K_ICM20948_REG_BANK_3:
            regBankSel.bits.bankSel = 3U;
            break;

        default:
            return SPP_ERROR;
    }

    return ICM20948_writeReg(p_spi, K_ICM20948_REG_BANK_SEL, regBankSel.value);
}

/**
 * @brief Resets the FIFO by asserting and then deasserting the FIFO reset bits.
 *
 * @param[in] p_spi Pointer to the SPI handler.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
static retval_t ICM20948_resetFifo(void *p_spi)
{
    ICM20948_RegFifoRst_t fifoResetReg = {.value = 0U};
    retval_t ret;

    fifoResetReg.bits.fifoRst0 = 1U;
    fifoResetReg.bits.fifoRst1 = 1U;
    fifoResetReg.bits.fifoRst2 = 1U;
    fifoResetReg.bits.fifoRst3 = 1U;
    fifoResetReg.bits.fifoRst4 = 1U;

    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_FIFO_RST, fifoResetReg.value);
    if (ret != SPP_OK)
    {
        return ret;
    }

    fifoResetReg.bits.fifoRst0 = 0U;

    return ICM20948_writeReg(p_spi, K_ICM20948_REG_FIFO_RST, fifoResetReg.value);
}

/**
 * @brief Writes a sequence of bytes into DMP memory.
 *
 * This helper writes each byte individually after programming the DMP memory
 * bank and memory start address associated with the target DMP address.
 *
 * @param[in] p_data Pointer to the SPI handler.
 * @param[in] addr Initial DMP memory address.
 * @param[in] p_bytes Pointer to the byte sequence to write.
 * @param[in] len Number of bytes to write.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
static retval_t ICM20948_dmpWriteBytes(void *p_data, spp_uint16_t addr, const spp_uint8_t *p_bytes,
                                       spp_uint8_t len)
{
    void *p_spi = p_data;
    retval_t ret;

    if ((p_spi == NULL) || (p_bytes == NULL))
    {
        return SPP_ERROR_NULL_POINTER;
    }

    for (spp_uint8_t i = 0U; i < len; i++)
    {
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_MEM_BANK_SEL, (spp_uint8_t)((addr + i) >> 8));
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_MEM_START_ADDR,
                                (spp_uint8_t)((addr + i) & 0xFFU));
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_MEM_R_W, p_bytes[i]);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    return SPP_OK;
}

/**
 * @brief Writes a 32-bit big-endian value into DMP memory.
 *
 * @param[in] p_data Pointer to the SPI handler.
 * @param[in] addr DMP memory address.
 * @param[in] value 32-bit value to write.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
static retval_t ICM20948_dmpWrite32(void *p_data, spp_uint16_t addr, spp_uint32_t value)
{
    spp_uint8_t bytes[4] = {(spp_uint8_t)(value >> 24), (spp_uint8_t)(value >> 16),
                            (spp_uint8_t)(value >> 8), (spp_uint8_t)value};

    return ICM20948_dmpWriteBytes(p_data, addr, bytes, 4U);
}

/**
 * @brief Writes a 16-bit big-endian value into DMP memory.
 *
 * @param[in] p_data Pointer to the SPI handler.
 * @param[in] addr DMP memory address.
 * @param[in] value 16-bit value to write.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
static retval_t ICM20948_dmpWrite16(void *p_data, spp_uint16_t addr, spp_uint16_t value)
{
    spp_uint8_t bytes[2] = {(spp_uint8_t)(value >> 8), (spp_uint8_t)value};

    return ICM20948_dmpWriteBytes(p_data, addr, bytes, 2U);
}

/**
 * @brief Executes a low-power wake cycle sequence.
 *
 * This sequence writes PWR_MGMT_1 = 0x21 and then restores PWR_MGMT_1 = 0x01.
 *
 * @param[in] p_data Pointer to the SPI handler.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
static retval_t ICM20948_lpWakeCycle(void *p_data)
{
    void *p_spi = p_data;
    ICM20948_RegPwrMgmt1_t pwrMgmt1Reg = {.value = 0U};
    retval_t ret;

    if (p_spi == NULL)
    {
        return SPP_ERROR_NULL_POINTER;
    }

    pwrMgmt1Reg.bits.clkSel = K_ICM20948_CLK_AUTO;
    pwrMgmt1Reg.bits.lpEn = 1U;

    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_1, pwrMgmt1Reg.value);
    if (ret != SPP_OK)
    {
        return ret;
    }

    pwrMgmt1Reg.value = 0U;
    pwrMgmt1Reg.bits.clkSel = K_ICM20948_CLK_AUTO;

    return ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_1, pwrMgmt1Reg.value);
}

/**
 * @brief Computes the DMP gyro scale factor from the PLL trim value.
 *
 * The implementation follows the method described in AN-MAPPS Appendix II.
 *
 * @param[in] pll Signed PLL trim value.
 *
 * @return Computed gyro scale factor.
 */
static spp_int32_t ICM20948_calcGyroSf(spp_int8_t pll)
{
    spp_int32_t t = 102870L + 81L * (spp_int32_t)pll;
    spp_int32_t a = (1L << 30) / t;
    spp_int32_t r = (1L << 30) - a * t;
    spp_int32_t v = a * 797L * K_ICM20948_DMP_DIVIDER;

    v += (spp_int32_t)(((spp_int64_t)a * 1011387LL * K_ICM20948_DMP_DIVIDER) >> 20);
    v += r * 797L * K_ICM20948_DMP_DIVIDER / t;
    v += (spp_int32_t)(((spp_int64_t)r * 1011387LL * K_ICM20948_DMP_DIVIDER) >> 20) / t;

    return v << 1;
}

/**
 * @brief Writes the DMP output control configuration block.
 *
 * @param[in] p_data Pointer to the SPI handler.
 * @param[in] outCtl1 Primary DMP output control word.
 * @param[in] motionEvent Motion event control word.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
static retval_t ICM20948_dmpWriteOutputConfig(void *p_data, spp_uint16_t outCtl1,
                                              spp_uint16_t motionEvent)
{
    retval_t ret;

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_DATA_OUT_CTL1, outCtl1);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_DATA_INTR_CTL, outCtl1);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_DATA_OUT_CTL2, 0x0000U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_MOTION_EVENT_CTL, motionEvent);
    if (ret != SPP_OK)
    {
        return ret;
    }

    return SPP_OK;
}

/* ----------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------- */

/** @copydoc ICM20948_loadDmp */
retval_t ICM20948_loadDmp(void *p_data)
{
    void *p_spi = p_data;
    retval_t ret;
    spp_uint16_t firmwareSize = (spp_uint16_t)sizeof(s_dmp3Image);
    spp_uint16_t addr = K_ICM20948_DMP_LOAD_START;
    const spp_uint8_t *p_firmware = s_dmp3Image;

    if (p_spi == NULL)
    {
        return SPP_ERROR_NULL_POINTER;
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_0);
    if (ret != SPP_OK)
    {
        return ret;
    }

    for (spp_uint16_t i = 0U; i < firmwareSize; i++, addr++)
    {
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_MEM_BANK_SEL, (spp_uint8_t)(addr >> 8));
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_MEM_START_ADDR, (spp_uint8_t)(addr & 0xFFU));
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_MEM_R_W, p_firmware[i]);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    addr = K_ICM20948_DMP_LOAD_START;

    for (spp_uint16_t i = 0U; i < firmwareSize; i++, addr++)
    {
        spp_uint8_t readValue;

        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_MEM_BANK_SEL, (spp_uint8_t)(addr >> 8));
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_MEM_START_ADDR, (spp_uint8_t)(addr & 0xFFU));
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_readReg(p_spi, K_ICM20948_REG_MEM_R_W, &readValue);
        if (ret != SPP_OK)
        {
            return ret;
        }

        if (readValue != p_firmware[i])
        {
            return SPP_ERROR;
        }
    }

    return SPP_OK;
}

/** @copydoc ICM20948_configDmpInit */
retval_t ICM20948_configDmpInit(void *p_data)
{
    void *p_spi = p_data;
    retval_t ret;
    spp_uint8_t whoAmIValue;
    spp_uint8_t pllRaw;
    spp_int8_t pllTrim;

    static const ICM20948_DmpMatrixEntry_t s_cpassMatrix[] = {
        {K_ICM20948_DMP_CPASS_MTX_00, 0x09999999U}, {K_ICM20948_DMP_CPASS_MTX_01, 0x00000000U},
        {K_ICM20948_DMP_CPASS_MTX_02, 0x00000000U}, {K_ICM20948_DMP_CPASS_MTX_10, 0x00000000U},
        {K_ICM20948_DMP_CPASS_MTX_11, 0xF6666667U}, {K_ICM20948_DMP_CPASS_MTX_12, 0x00000000U},
        {K_ICM20948_DMP_CPASS_MTX_20, 0x00000000U}, {K_ICM20948_DMP_CPASS_MTX_21, 0x00000000U},
        {K_ICM20948_DMP_CPASS_MTX_22, 0xF6666667U}};

    static const ICM20948_DmpMatrixEntry_t s_b2sMatrix[] = {
        {K_ICM20948_DMP_B2S_MTX_00, 0x40000000U}, {K_ICM20948_DMP_B2S_MTX_01, 0x00000000U},
        {K_ICM20948_DMP_B2S_MTX_02, 0x00000000U}, {K_ICM20948_DMP_B2S_MTX_10, 0x00000000U},
        {K_ICM20948_DMP_B2S_MTX_11, 0x40000000U}, {K_ICM20948_DMP_B2S_MTX_12, 0x00000000U},
        {K_ICM20948_DMP_B2S_MTX_20, 0x00000000U}, {K_ICM20948_DMP_B2S_MTX_21, 0x00000000U},
        {K_ICM20948_DMP_B2S_MTX_22, 0x40000000U}};

    if (p_spi == NULL)
    {
        return SPP_ERROR_NULL_POINTER;
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_0);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_readReg(p_spi, K_ICM20948_REG_WHO_AM_I, &whoAmIValue);
    if (ret != SPP_OK)
    {
        return ret;
    }

    if (whoAmIValue != K_ICM20948_WHO_AM_I_VALUE)
    {
        return SPP_ERROR;
    }

    {
        ICM20948_RegPwrMgmt1_t pwrMgmt1Reg = {.value = 0U};
        ICM20948_RegUserCtrl_t userCtrlReg = {.value = 0U};
        ICM20948_RegPwrMgmt2_t pwrMgmt2Reg = {.value = 0U};
        ICM20948_RegLpConf_t lpConfReg = {.value = 0U};

        pwrMgmt1Reg.bits.clkSel = K_ICM20948_CLK_AUTO;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_1, pwrMgmt1Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        userCtrlReg.bits.i2cIfDis = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_USER_CTRL, userCtrlReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        pwrMgmt2Reg.bits.disableAccelX = 1U;
        pwrMgmt2Reg.bits.disableAccelY = 1U;
        pwrMgmt2Reg.bits.disableAccelZ = 1U;
        pwrMgmt2Reg.bits.disableGyroZ = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_2, pwrMgmt2Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        /* Only I2C master in cycled mode. Accel and gyro must NOT be in
         * low-power cycled mode when using the DMP. */
        lpConfReg.bits.i2cMstCyc = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_LP_CONF, lpConfReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_USER_CTRL, userCtrlReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    ret = ICM20948_loadDmp(p_data);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_2);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_DMP_ADDR_MSB, K_ICM20948_DMP_START_ADDR_MSB);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_DMP_ADDR_LSB, K_ICM20948_DMP_START_ADDR_LSB);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_0);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWriteOutputConfig(p_data, 0x0000U, 0x0000U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_DATA_RDY_STATUS, 0x0000U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_FIFO_WATERMARK, 800U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    {
        ICM20948_RegIntEnable_t intEnableReg = {.value = 0U};
        ICM20948_RegIntEnable2_t intEnable2Reg = {.value = 0U};
        ICM20948_RegSingleFifoPrioritySel_t fifoPriorityReg = {.value = 0U};
        ICM20948_RegHwFixDisable_t hwFixDisableReg = {.value = 0U};

        intEnableReg.bits.rawData0Rdy = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_INT_ENABLE, intEnableReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        intEnable2Reg.bits.fifoOverflowEn0 = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_INT_ENABLE_2, intEnable2Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        fifoPriorityReg.value = 0xE4U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_SINGLE_FIFO_PRIORITY_SEL,
                                fifoPriorityReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        hwFixDisableReg.value = 0x48U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_HW_FIX_DISABLE, hwFixDisableReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_2);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_GYRO_SMPLRT_DIV, 0x13U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_ACCEL_SMPLRT_DIV_1, 0x00U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_ACCEL_SMPLRT_DIV_2, 0x13U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_0);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_BAC_RATE, 0x0000U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_B2S_RATE, 0x0000U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    {
        ICM20948_RegFifoCfg_t fifoCfgReg = {.value = 0U};
        ICM20948_RegFifoEn1_t fifoEn1Reg = {.value = 0U};
        ICM20948_RegFifoEn2_t fifoEn2Reg = {.value = 0U};

        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_FIFO_CFG, fifoCfgReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_resetFifo(p_spi);
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_FIFO_EN_1, fifoEn1Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_FIFO_EN_2, fifoEn2Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    {
        ICM20948_RegPwrMgmt1_t pwrMgmt1Reg = {.value = 0U};
        ICM20948_RegPwrMgmt2_t pwrMgmt2Reg = {.value = 0U};

        pwrMgmt1Reg.bits.clkSel = K_ICM20948_CLK_AUTO;
        pwrMgmt1Reg.bits.lpEn = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_1, pwrMgmt1Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        pwrMgmt2Reg.value = 0x7FU;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_2, pwrMgmt2Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        pwrMgmt1Reg.bits.sleep = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_1, pwrMgmt1Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        SPP_OSAL_TaskDelay(1);

        pwrMgmt1Reg.value = 0U;
        pwrMgmt1Reg.bits.clkSel = K_ICM20948_CLK_AUTO;
        pwrMgmt1Reg.bits.lpEn = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_1, pwrMgmt1Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        SPP_OSAL_TaskDelay(1);

        pwrMgmt1Reg.value = 0U;
        pwrMgmt1Reg.bits.clkSel = K_ICM20948_CLK_AUTO;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_1, pwrMgmt1Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        SPP_OSAL_TaskDelay(1);
    }

    /* ----------------------------------------------------------------
     * AK09916 magnetometer: soft-reset, then disable all peripherals.
     * ---------------------------------------------------------------- */
    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_3);
    if (ret != SPP_OK)
    {
        return ret;
    }

    /* Disable all I2C peripherals */
    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_SLV0_CTRL, 0x00U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_I2C_SLV1_CTRL, 0x00U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, 0x0DU, 0x00U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, 0x11U, 0x00U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    /* I2C master: CLK=7 (345.6 kHz), P_NSR=1 */
    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_I2C_CTRL, 0x17U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_I2C_MST_ODR_CONFIG, 0x04U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    /* SLV1: write CNTL3=0x01 (SRST) to AK09916 to soft-reset it */
    ret = ICM20948_writeReg(p_spi, K_ICM20948_I2C_SLV1_ADDR, K_ICM20948_MAG_WR_ADDR);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_I2C_SLV1_REG, 0x32U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_I2C_SLV1_DO, 0x01U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_I2C_SLV1_CTRL, 0x81U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    /* Enable I2C master to send the reset, wait, then disable */
    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_0);
    if (ret != SPP_OK)
    {
        return ret;
    }

    {
        ICM20948_RegUserCtrl_t userCtrlReg = {.value = 0U};

        userCtrlReg.bits.i2cIfDis = 1U;
        userCtrlReg.bits.i2cMstEn = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_USER_CTRL, userCtrlReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        SPP_OSAL_TaskDelay(100);

        userCtrlReg.value = 0U;
        userCtrlReg.bits.i2cIfDis = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_USER_CTRL, userCtrlReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    /* Disable all peripherals after reset */
    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_3);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_SLV0_CTRL, 0x00U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_I2C_SLV1_CTRL, 0x00U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_0);
    if (ret != SPP_OK)
    {
        return ret;
    }

    {
        ICM20948_RegUserCtrl_t userCtrlReg = {.value = 0U};

        userCtrlReg.bits.i2cIfDis = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_USER_CTRL, userCtrlReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    for (spp_uint8_t i = 0U; i < (sizeof(s_cpassMatrix) / sizeof(s_cpassMatrix[0])); i++)
    {
        ret = ICM20948_dmpWrite32(p_data, s_cpassMatrix[i].addr, s_cpassMatrix[i].val);
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_lpWakeCycle(p_data);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    SPP_OSAL_TaskDelay(1);

    for (spp_uint8_t i = 0U; i < (sizeof(s_b2sMatrix) / sizeof(s_b2sMatrix[0])); i++)
    {
        ret = ICM20948_dmpWrite32(p_data, s_b2sMatrix[i].addr, s_b2sMatrix[i].val);
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_lpWakeCycle(p_data);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    SPP_OSAL_TaskDelay(1);

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_2);
    if (ret != SPP_OK)
    {
        return ret;
    }

    {
        ICM20948_RegAccelConfig_t accelConfigReg = {.value = 0U};
        ICM20948_RegAccelConfig2_t accelConfig2Reg = {.value = 0U};

        accelConfigReg.bits.accelFsSel = K_ICM20948_ACCEL_FS_4G;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_ACCEL_CONFIG, accelConfigReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_ACCEL_CONFIG_2, accelConfig2Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_0);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_lpWakeCycle(p_data);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite32(p_data, K_ICM20948_DMP_ACC_SCALE, 0x04000000U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite32(p_data, K_ICM20948_DMP_ACC_SCALE2, 0x00040000U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_lpWakeCycle(p_data);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_2);
    if (ret != SPP_OK)
    {
        return ret;
    }

    {
        ICM20948_RegGyroConfig_t gyroConfigReg = {.value = 0U};

        gyroConfigReg.bits.gyroFchoice = 1U;
        gyroConfigReg.bits.gyroFsSel = K_ICM20948_GYRO_FS_2000DPS;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_GYRO_CONFIG, gyroConfigReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_writeReg(p_spi, 0x02U, 0x00U);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_0);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_lpWakeCycle(p_data);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite32(p_data, K_ICM20948_DMP_GYRO_FULLSCALE, 0x10000000U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_lpWakeCycle(p_data);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_1);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_readReg(p_spi, K_ICM20948_REG_TIMEBASE_CORRECTION_PLL, &pllRaw);
    if (ret != SPP_OK)
    {
        return ret;
    }

    pllTrim = (spp_int8_t)pllRaw;

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_0);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_lpWakeCycle(p_data);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite32(p_data, K_ICM20948_DMP_GYRO_SF,
                              (spp_uint32_t)ICM20948_calcGyroSf(pllTrim));
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_lpWakeCycle(p_data);
    if (ret != SPP_OK)
    {
        return ret;
    }

    {
        ICM20948_RegLpConf_t lpConfReg = {.value = 0U};
        ICM20948_RegPwrMgmt1_t pwrMgmt1Reg = {.value = 0U};

        lpConfReg.bits.i2cMstCyc = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_LP_CONF, lpConfReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        pwrMgmt1Reg.bits.clkSel = K_ICM20948_CLK_AUTO;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_1, pwrMgmt1Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    for (spp_uint8_t seq = 0U; seq < 3U; seq++)
    {
        ICM20948_RegUserCtrl_t userCtrlReg = {.value = 0U};
        ICM20948_RegPwrMgmt2_t pwrMgmt2Reg = {.value = 0U};
        ICM20948_RegPwrMgmt1_t pwrMgmt1Reg = {.value = 0U};

        if (seq == 0U)
        {
            userCtrlReg.bits.i2cIfDis = 1U;
        }
        else
        {
            userCtrlReg.bits.i2cIfDis = 1U;
            userCtrlReg.bits.i2cMstEn = 1U;
            userCtrlReg.bits.fifoEn = 1U;
            userCtrlReg.bits.dmpEn = 1U;
        }

        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_USER_CTRL, userCtrlReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        pwrMgmt2Reg.value = 0x47U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_2, pwrMgmt2Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        userCtrlReg.value = 0U;
        userCtrlReg.bits.i2cIfDis = 1U;
        userCtrlReg.bits.i2cMstEn = 1U;
        userCtrlReg.bits.fifoEn = 1U;
        userCtrlReg.bits.dmpEn = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_USER_CTRL, userCtrlReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        ret = ICM20948_dmpWriteOutputConfig(p_data, 0x0000U, 0x0000U);
        if (ret != SPP_OK)
        {
            return ret;
        }

        if (seq < 2U)
        {
            pwrMgmt2Reg.value = 0x7FU;
            ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_2, pwrMgmt2Reg.value);
            if (ret != SPP_OK)
            {
                return ret;
            }

            pwrMgmt1Reg.bits.clkSel = K_ICM20948_CLK_AUTO;
            pwrMgmt1Reg.bits.sleep = 1U;
            ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_1, pwrMgmt1Reg.value);
            if (ret != SPP_OK)
            {
                return ret;
            }

            SPP_OSAL_TaskDelay(1);

            pwrMgmt1Reg.value = 0U;
            pwrMgmt1Reg.bits.clkSel = K_ICM20948_CLK_AUTO;
            ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_1, pwrMgmt1Reg.value);
            if (ret != SPP_OK)
            {
                return ret;
            }

            ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_2, pwrMgmt2Reg.value);
            if (ret != SPP_OK)
            {
                return ret;
            }

            ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_DATA_RDY_STATUS, 0x0000U);
            if (ret != SPP_OK)
            {
                return ret;
            }

            ret = ICM20948_lpWakeCycle(p_data);
            if (ret != SPP_OK)
            {
                return ret;
            }
        }
    }

    ret = ICM20948_dmpWriteOutputConfig(p_data, 0xE400U, 0x0048U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite32(p_data, K_ICM20948_DMP_ACCEL_ONLY_GAIN, 0x00E8BA2EU);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite32(p_data, K_ICM20948_DMP_ACCEL_ALPHA_VAR, 0x3D27D27DU);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite32(p_data, K_ICM20948_DMP_ACCEL_A_VAR, 0x02D82D83U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_ACCEL_CAL_INIT, 0x0000U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    /* Compass time buffer: 69 matches the I2C master ODR of 68.75 Hz. */
    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_CPASS_TIME_BUFFER, 0x0045U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_2);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_ACCEL_SMPLRT_DIV_1, 0x00U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_ACCEL_SMPLRT_DIV_2, 0x04U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_GYRO_SMPLRT_DIV, 0x04U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_0);
    if (ret != SPP_OK)
    {
        return ret;
    }

    /* Set all DMP ODR registers to 0 (maximum rate = DMP running rate). */
    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_ODR_QUAT9, 0x0000U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_ODR_QUAT6, 0x0000U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_ODR_ACCEL, 0x0000U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_ODR_GYRO, 0x0000U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_ODR_CPASS, 0x0000U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite32(p_data, K_ICM20948_DMP_GYRO_SF,
                              (spp_uint32_t)ICM20948_calcGyroSf(pllTrim));
    if (ret != SPP_OK)
    {
        return ret;
    }

    {
        ICM20948_RegPwrMgmt2_t pwrMgmt2Reg = {.value = 0U};

        pwrMgmt2Reg.value = 0x40U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_2, pwrMgmt2Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_DATA_RDY_STATUS, 0x000BU);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_3);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_SLV0_ADDR, K_ICM20948_MAG_RD_ADDR);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_SLV0_REG, 0x03U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_SLV0_CTRL, 0xDAU);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_I2C_SLV1_ADDR, K_ICM20948_MAG_WR_ADDR);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_I2C_SLV1_REG, K_ICM20948_MAG_CTRL_2);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_I2C_SLV1_DO, 0x01U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_writeReg(p_spi, K_ICM20948_I2C_SLV1_CTRL, 0x81U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    /* I2C master ODR: 0x04 → 1100/2^4 = 68.75 Hz */
    ret = ICM20948_writeReg(p_spi, K_ICM20948_I2C_MST_ODR_CONFIG, 0x04U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_setBank(p_spi, K_ICM20948_REG_BANK_0);
    if (ret != SPP_OK)
    {
        return ret;
    }

    {
        ICM20948_RegUserCtrl_t userCtrlReg = {.value = 0U};

        userCtrlReg.bits.i2cIfDis = 1U;
        userCtrlReg.bits.i2cMstEn = 1U;
        userCtrlReg.bits.fifoEn = 1U;
        userCtrlReg.bits.dmpEn = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_USER_CTRL, userCtrlReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }

        userCtrlReg.bits.dmpRst = 1U;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_USER_CTRL, userCtrlReg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    SPP_OSAL_TaskDelay(1);

    ret = ICM20948_resetFifo(p_spi);
    if (ret != SPP_OK)
    {
        return ret;
    }

    {
        ICM20948_RegPwrMgmt1_t pwrMgmt1Reg = {.value = 0U};

        pwrMgmt1Reg.bits.clkSel = K_ICM20948_CLK_AUTO;
        ret = ICM20948_writeReg(p_spi, K_ICM20948_REG_PWR_MGMT_1, pwrMgmt1Reg.value);
        if (ret != SPP_OK)
        {
            return ret;
        }
    }

    ret = ICM20948_dmpWriteOutputConfig(p_data, 0xE400U, 0x0048U);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_dmpWrite16(p_data, K_ICM20948_DMP_DATA_RDY_STATUS, 0x000BU);
    if (ret != SPP_OK)
    {
        return ret;
    }

    ret = ICM20948_resetFifo(p_spi);
    if (ret != SPP_OK)
    {
        return ret;
    }

    return SPP_OK;
}

/** @copydoc ICM20948_checkFifoData */
void ICM20948_checkFifoData(void *p_data)
{
    void *p_spi = p_data;
    spp_uint8_t txRxData[3] = {0U};
    retval_t ret = SPP_ERROR;

    if (p_spi == NULL)
    {
        return;
    }

    txRxData[0] = K_READ_OP | K_ICM20948_REG_INT_STATUS;
    txRxData[1] = K_WRITE_OP;

    ret = SPP_HAL_SPI_Transmit(p_spi, txRxData, 2U);
    if (ret != SPP_OK)
    {
        return;
    }

    {
        spp_uint8_t intStatus = txRxData[1];

        txRxData[0] = K_READ_OP | K_ICM20948_REG_DMP_INT_STATUS;
        txRxData[1] = K_WRITE_OP;

        ret = SPP_HAL_SPI_Transmit(p_spi, txRxData, 2U);
        if (ret != SPP_OK)
        {
            return;
        }

        if ((intStatus & 0x02U) != 0U)
        {
            txRxData[0] = K_READ_OP | K_ICM20948_REG_FIFO_COUNTH;
            txRxData[1] = K_WRITE_OP;
            txRxData[2] = K_WRITE_OP;

            ret = SPP_HAL_SPI_Transmit(p_spi, txRxData, 3U);
            if (ret != SPP_OK)
            {
                return;
            }

            {
                spp_uint16_t fifoCount = ((spp_uint16_t)txRxData[1] << 8) | txRxData[2];

                if (fifoCount > K_ICM20948_FIFO_RESET_THRESHOLD)
                {
                    (void)ICM20948_resetFifo(p_spi);
                    return;
                }

                {
                    spp_uint16_t numPackets = fifoCount / K_ICM20948_DMP_PACKET_SIZE_BYTES;

                    for (spp_uint16_t i = 0U; i < numPackets; i++)
                    {
                        spp_uint8_t fifoBuffer[K_ICM20948_DMP_PACKET_SIZE_BYTES + 1U] = {0U};

                        fifoBuffer[0] = K_READ_OP | K_ICM20948_REG_FIFO_R_W;
                        ret = SPP_HAL_SPI_Transmit(p_spi, fifoBuffer,
                                                   K_ICM20948_DMP_PACKET_SIZE_BYTES + 1U);
                        if (ret != SPP_OK)
                        {
                            return;
                        }

                        {
                            uint16_t header = ((uint16_t)fifoBuffer[1] << 8) | fifoBuffer[2];

                            int16_t accelX =
                                (int16_t)(((uint16_t)fifoBuffer[3] << 8) | fifoBuffer[4]);
                            int16_t accelY =
                                (int16_t)(((uint16_t)fifoBuffer[5] << 8) | fifoBuffer[6]);
                            int16_t accelZ =
                                (int16_t)(((uint16_t)fifoBuffer[7] << 8) | fifoBuffer[8]);

                            int16_t gyroX =
                                (int16_t)(((uint16_t)fifoBuffer[9] << 8) | fifoBuffer[10]);
                            int16_t gyroY =
                                (int16_t)(((uint16_t)fifoBuffer[11] << 8) | fifoBuffer[12]);
                            int16_t gyroZ =
                                (int16_t)(((uint16_t)fifoBuffer[13] << 8) | fifoBuffer[14]);

                            int16_t magX =
                                (int16_t)(((uint16_t)fifoBuffer[21] << 8) | fifoBuffer[22]);
                            int16_t magY =
                                (int16_t)(((uint16_t)fifoBuffer[23] << 8) | fifoBuffer[24]);
                            int16_t magZ =
                                (int16_t)(((uint16_t)fifoBuffer[25] << 8) | fifoBuffer[26]);

                            int32_t q1Raw =
                                ((int32_t)fifoBuffer[27] << 24) | ((int32_t)fifoBuffer[28] << 16) |
                                ((int32_t)fifoBuffer[29] << 8) | (int32_t)fifoBuffer[30];

                            int32_t q2Raw =
                                ((int32_t)fifoBuffer[31] << 24) | ((int32_t)fifoBuffer[32] << 16) |
                                ((int32_t)fifoBuffer[33] << 8) | (int32_t)fifoBuffer[34];

                            int32_t q3Raw =
                                ((int32_t)fifoBuffer[35] << 24) | ((int32_t)fifoBuffer[36] << 16) |
                                ((int32_t)fifoBuffer[37] << 8) | (int32_t)fifoBuffer[38];

                            int16_t accuracy =
                                (int16_t)(((uint16_t)fifoBuffer[39] << 8) | fifoBuffer[40]);
                            uint16_t footer = ((uint16_t)fifoBuffer[41] << 8) | fifoBuffer[42];

                            float ax = accelX / 8192.0f;
                            float ay = accelY / 8192.0f;
                            float az = accelZ / 8192.0f;

                            float gx = gyroX / 16.4f;
                            float gy = gyroY / 16.4f;
                            float gz = gyroZ / 16.4f;

                            float mx = magX * 0.15f;
                            float my = magY * 0.15f;
                            float mz = magZ * 0.15f;

                            float qx = q1Raw / 1073741824.0f;
                            float qy = q2Raw / 1073741824.0f;
                            float qz = q3Raw / 1073741824.0f;
                            float qwSquared = 1.0f - (qx * qx) - (qy * qy) - (qz * qz);
                            float qw = (qwSquared > 0.0f) ? sqrtf(qwSquared) : 0.0f;

                            (void)header;
                            (void)footer;

                            SPP_LOGI(
                                K_ICM20948_LOG_TAG,
                                "A:[%.2f %.2f %.2f]g G:[%.1f %.1f %.1f]dps M:[%.1f %.1f %.1f]uT",
                                ax, ay, az, gx, gy, gz, mx, my, mz);

                            SPP_LOGI(K_ICM20948_LOG_TAG, "Q:[w=%.4f x=%.4f y=%.4f z=%.4f] acc:%d",
                                     qw, qx, qy, qz, accuracy);
                        }
                    }
                }
            }
        }
    }
}