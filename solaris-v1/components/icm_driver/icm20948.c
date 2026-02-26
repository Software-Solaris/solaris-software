#include "icm20948.h"
#include "driver/spi_common.h"
#include "returntypes.h"
#include "spi.h"
#include "task.h"
#include "types.h"
#include <string.h>

static spp_uint8_t data[2];

static const spp_uint8_t dmp3_image[] = {
    #include "icm20948_img.dmp3a.h"
};

/* Write N bytes to DMP memory, setting bank+address for each byte */
static retval_t IcmDmpWriteBytes(icm_data_t        *p_data_icm,
                                  spp_uint16_t       dmp_addr,
                                  const spp_uint8_t *bytes,
                                  spp_uint8_t        len)
{
    retval_t    ret    = SPP_ERROR;
    spp_uint8_t buf[2] = {0};

    for (spp_uint8_t i = 0; i < len; i++)
    {
        buf[0] = WRITE_OP | REG_MEM_BANK_SEL;
        buf[1] = (spp_uint8_t)((dmp_addr + i) >> 8);
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_MEM_START_ADDR;
        buf[1] = (spp_uint8_t)((dmp_addr + i) & 0xFF);
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_MEM_R_W;
        buf[1] = bytes[i];
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;
    }
    return SPP_OK;
}

static retval_t IcmDmpWrite32(icm_data_t  *p_data_icm,
                               spp_uint16_t dmp_addr,
                               spp_uint32_t value)
{
    spp_uint8_t bytes[4] = {
        (spp_uint8_t)(value >> 24),
        (spp_uint8_t)(value >> 16),
        (spp_uint8_t)(value >> 8),
        (spp_uint8_t)(value)
    };
    return IcmDmpWriteBytes(p_data_icm, dmp_addr, bytes, 4);
}

static retval_t IcmDmpWrite16(icm_data_t  *p_data_icm,
                               spp_uint16_t dmp_addr,
                               spp_uint16_t value)
{
    spp_uint8_t bytes[2] = {
        (spp_uint8_t)(value >> 8),
        (spp_uint8_t)(value)
    };
    return IcmDmpWriteBytes(p_data_icm, dmp_addr, bytes, 2);
}

/* LP/wake cycle seen repeatedly in captured SPI traffic: PWR_MGMT_1 = 0x21 -> 0x01 */
static retval_t IcmLpWakeCycle(icm_data_t *p_data_icm)
{
    retval_t    ret    = SPP_ERROR;
    spp_uint8_t buf[2] = {0};

    buf[0] = WRITE_OP | REG_PWR_MGMT_1;
    buf[1] = 0x21;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_PWR_MGMT_1;
    buf[1] = 0x01;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    return ret;
}

/* GYRO_SF calculation per AN-MAPPS Appendix II */
static spp_int32_t IcmCalcGyroSf(spp_int8_t pll)
{
#define BASE_SAMPLE_RATE  1125
#define DMP_RUNNING_RATE  225
#define DMP_DIVIDER       (BASE_SAMPLE_RATE / DMP_RUNNING_RATE)

    spp_int32_t a, r, value, t;

    t     = 102870L + 81L * (spp_int32_t)pll;
    a     = (1L << 30) / t;
    r     = (1L << 30) - a * t;
    value = a * 797 * DMP_DIVIDER;
    value += (spp_int32_t)(((spp_int64_t)a * 1011387LL * DMP_DIVIDER) >> 20);
    value += r * 797L * DMP_DIVIDER / t;
    value += (spp_int32_t)(((spp_int64_t)r * 1011387LL * DMP_DIVIDER) >> 20) / t;
    value <<= 1;

    return value;
}

retval_t IcmInit(void *p_data)
{
    icm_data_t *p_data_icm = (icm_data_t *)p_data;
    retval_t    ret;
    void       *p_handler_spi;

    p_handler_spi = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_handler_spi);
    if (ret != SPP_OK) return ret;

    p_data_icm->p_handler_spi = p_handler_spi;

    return SPP_OK;
}

/* Load DMP firmware into chip SRAM at address 0x0090, then verify */
retval_t IcmLoadDmp(void *p_data)
{
    icm_data_t        *p_data_icm = (icm_data_t *)p_data;
    retval_t           ret        = SPP_ERROR;
    spp_uint8_t        buf[2]     = {0};
    spp_uint16_t       fw_size    = sizeof(dmp3_image);
    spp_uint16_t       memaddr    = DMP_LOAD_START;
    const spp_uint8_t *fw_ptr     = dmp3_image;

    if (p_data_icm->firmware_loaded) return SPP_OK;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    while (fw_size > 0)
    {
        buf[0] = WRITE_OP | REG_MEM_BANK_SEL;
        buf[1] = (spp_uint8_t)(memaddr >> 8);
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_MEM_START_ADDR;
        buf[1] = (spp_uint8_t)(memaddr & 0xFF);
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_MEM_R_W;
        buf[1] = *fw_ptr;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        fw_ptr++;
        memaddr++;
        fw_size--;
    }

    /* Verify */
    fw_size = sizeof(dmp3_image);
    memaddr = DMP_LOAD_START;
    fw_ptr  = dmp3_image;

    while (fw_size > 0)
    {
        buf[0] = WRITE_OP | REG_MEM_BANK_SEL;
        buf[1] = (spp_uint8_t)(memaddr >> 8);
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_MEM_START_ADDR;
        buf[1] = (spp_uint8_t)(memaddr & 0xFF);
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = READ_OP | REG_MEM_R_W;
        buf[1] = EMPTY_MESSAGE;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        if (buf[1] != *fw_ptr) return SPP_ERROR;

        fw_ptr++;
        memaddr++;
        fw_size--;
    }

    p_data_icm->firmware_loaded = true;
    return SPP_OK;
}

retval_t IcmConfigDmpInit(void *p_data)
{
    icm_data_t *p_data_icm = (icm_data_t *)p_data;
    retval_t    ret        = SPP_ERROR;
    spp_uint8_t buf[2]     = {0};

    /* --- 1. WHO_AM_I check, clock source, SPI-only mode --- */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = READ_OP | REG_WHO_AM_I;
    buf[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;
    if (buf[1] != 0xEA) return SPP_ERROR;

    buf[0] = WRITE_OP | REG_PWR_MGMT_1;
    buf[1] = 0x01;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* --- 2. Disable gyro during firmware load --- */
    buf[0] = WRITE_OP | REG_PWR_MGMT_2;
    buf[1] = 0x47;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = READ_OP | REG_WHO_AM_I;
    buf[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* --- 3. Duty-cycle I2C + accel + gyro during init --- */
    buf[0] = WRITE_OP | REG_LP_CONF;
    buf[1] = 0x70;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* --- 4+5. Load and verify DMP firmware --- */
    ret = IcmLoadDmp((void *)p_data);
    if (ret != SPP_OK) return ret;

    /* --- 6. Set DMP execution start address 0x1000 (load addr is 0x0090) --- */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_DMP_ADDR_MSB;
    buf[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_DMP_ADDR_LSB;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* --- 7. Clear DMP control registers --- */
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_OUT_CTL1,    0x0000);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_OUT_CTL2,    0x0000);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_INTR_CTL,    0x0000);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_MOTION_EVENT_CTL, 0x0000);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_RDY_STATUS,  0x0000);
    if (ret != SPP_OK) return ret;

    /* --- 8. FIFO watermark = 800 bytes --- */
    ret = IcmDmpWrite16(p_data_icm, DMP_FIFO_WATERMARK, 0x0320);
    if (ret != SPP_OK) return ret;

    /* --- 9. Enable DMP interrupt on INT1 pin --- */
    buf[0] = WRITE_OP | REG_INT_ENABLE;
    buf[1] = 0x02;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* --- 10. Enable FIFO overflow interrupt --- */
    buf[0] = WRITE_OP | REG_INT_ENABLE_2;
    buf[1] = 0x01;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_SINGLE_FIFO_PRIORITY_SEL;
    buf[1] = 0xE4;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* --- 11. HW_FIX_DISABLE: read then set bit3 --- */
    buf[0] = READ_OP | REG_HW_FIX_DISABLE;
    buf[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_HW_FIX_DISABLE;
    buf[1] = 0x48;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* --- 12. Initial ODR: 1125/(1+19) = 56.25 Hz --- */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_GYRO_SMPLRT_DIV;
    buf[1] = 0x13;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_ACCEL_SMPLRT_DIV_1;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_ACCEL_SMPLRT_DIV_2;
    buf[1] = 0x13;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p_data_icm, DMP_BAC_RATE, 0x0000);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_B2S_RATE, 0x0000);
    if (ret != SPP_OK) return ret;

    /* --- 13. FIFO config and reset --- */
    buf[0] = WRITE_OP | REG_FIFO_CFG;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_FIFO_RST;
    buf[1] = 0x1F;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_FIFO_RST;
    buf[1] = 0x1E;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* DMP controls FIFO directly */
    buf[0] = WRITE_OP | REG_FIFO_EN_1;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_FIFO_EN_2;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* --- 14. Power cycle: LP -> sleep -> wake --- */
    buf[0] = WRITE_OP | REG_PWR_MGMT_1;
    buf[1] = 0x21;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_PWR_MGMT_2;
    buf[1] = 0x7F;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_PWR_MGMT_1;
    buf[1] = 0x61;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));

    buf[0] = WRITE_OP | REG_PWR_MGMT_1;
    buf[1] = 0x21;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));

    buf[0] = WRITE_OP | REG_PWR_MGMT_1;
    buf[1] = 0x01;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));

    /* --- 15. Configure I2C master and read AK09916 WHO_AM_I (expect 0x48) --- */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_3;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x05;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x09;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x0D;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x11;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x01;
    buf[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x00;
    buf[1] = 0x04;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x03;
    buf[1] = 0x8C;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x04;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x05;
    buf[1] = 0x81;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0x30;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(60));

    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = READ_OP | 0x3B;
    buf[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* --- 16. Put AK09916 into power-down via SLV1 write --- */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_3;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x05;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x07;
    buf[1] = 0x0C;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x08;
    buf[1] = 0x31;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x0A;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x09;
    buf[1] = 0x81;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0x30;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(60));

    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_3;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x09;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x05;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x09;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* --- 17. Compass mounting matrix (adjust per board orientation) --- */
    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_00, 0x09999999);
    if (ret != SPP_OK) return ret;
    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_01, 0x00000000);
    if (ret != SPP_OK) return ret;
    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_02, 0x00000000);
    if (ret != SPP_OK) return ret;
    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_10, 0x00000000);
    if (ret != SPP_OK) return ret;
    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_11, 0xF6666667);
    if (ret != SPP_OK) return ret;
    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_12, 0x00000000);
    if (ret != SPP_OK) return ret;
    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_20, 0x00000000);
    if (ret != SPP_OK) return ret;
    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_21, 0x00000000);
    if (ret != SPP_OK) return ret;
    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_22, 0xF6666667);
    if (ret != SPP_OK) return ret;
    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));

    /* --- 18. B2S mounting matrix, written 4x as per InvenSense SDK --- */
    for (spp_uint8_t repeat = 0; repeat < 4; repeat++)
    {
        ret = IcmDmpWrite32(p_data_icm, DMP_B2S_MTX_00, 0x40000000);
        if (ret != SPP_OK) return ret;
        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        ret = IcmDmpWrite32(p_data_icm, DMP_B2S_MTX_01, 0x00000000);
        if (ret != SPP_OK) return ret;
        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        ret = IcmDmpWrite32(p_data_icm, DMP_B2S_MTX_02, 0x00000000);
        if (ret != SPP_OK) return ret;
        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        ret = IcmDmpWrite32(p_data_icm, DMP_B2S_MTX_10, 0x00000000);
        if (ret != SPP_OK) return ret;
        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        ret = IcmDmpWrite32(p_data_icm, DMP_B2S_MTX_11, 0x40000000);
        if (ret != SPP_OK) return ret;
        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        ret = IcmDmpWrite32(p_data_icm, DMP_B2S_MTX_12, 0x00000000);
        if (ret != SPP_OK) return ret;
        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        ret = IcmDmpWrite32(p_data_icm, DMP_B2S_MTX_20, 0x00000000);
        if (ret != SPP_OK) return ret;
        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        ret = IcmDmpWrite32(p_data_icm, DMP_B2S_MTX_21, 0x00000000);
        if (ret != SPP_OK) return ret;
        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        ret = IcmDmpWrite32(p_data_icm, DMP_B2S_MTX_22, 0x40000000);
        if (ret != SPP_OK) return ret;
        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;
    }

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));

    /* --- 19. Accel config: +/-4g, bypass DLPF (read-modify-write pattern) --- */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = READ_OP | REG_ACCEL_CONFIG;
    buf[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_ACCEL_CONFIG;
    buf[1] = 0x02;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = READ_OP | REG_ACCEL_CONFIG_2;
    buf[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_ACCEL_CONFIG_2;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    /* --- 20. ACC_SCALE values, written 2x as per captured traffic --- */
    for (spp_uint8_t rep = 0; rep < 2; rep++)
    {
        ret = IcmDmpWrite32(p_data_icm, DMP_ACC_SCALE,  0x04000000);
        if (ret != SPP_OK) return ret;
        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        ret = IcmDmpWrite32(p_data_icm, DMP_ACC_SCALE2, 0x00040000);
        if (ret != SPP_OK) return ret;
        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;
    }

    /* --- 21. Gyro config: +/-2000dps, DLPF on, written 3x as per captured traffic --- */
    for (spp_uint8_t rep = 0; rep < 3; rep++)
    {
        buf[0] = WRITE_OP | REG_BANK_SEL;
        buf[1] = REG_BANK_2;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = READ_OP | REG_GYRO_CONFIG;
        buf[1] = EMPTY_MESSAGE;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_BANK_SEL;
        buf[1] = REG_BANK_0;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_BANK_SEL;
        buf[1] = REG_BANK_2;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_GYRO_CONFIG;
        buf[1] = 0x07;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_BANK_SEL;
        buf[1] = REG_BANK_0;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_BANK_SEL;
        buf[1] = REG_BANK_2;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = READ_OP | 0x02;
        buf[1] = EMPTY_MESSAGE;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_BANK_SEL;
        buf[1] = REG_BANK_0;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_BANK_SEL;
        buf[1] = REG_BANK_2;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | 0x02;
        buf[1] = 0x00;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_BANK_SEL;
        buf[1] = REG_BANK_0;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        ret = IcmDmpWrite32(p_data_icm, DMP_GYRO_FULLSCALE, 0x10000000);
        if (ret != SPP_OK) return ret;
        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;
    }

    /* --- 22. Read PLL trim, calculate and write GYRO_SF --- */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_1;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = READ_OP | REG_TIMEBASE_CORRECTION_PLL;
    buf[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    spp_int8_t pll = (spp_int8_t)buf[1];

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    spp_int32_t gyro_sf_init = IcmCalcGyroSf(pll);
    ret = IcmDmpWrite32(p_data_icm, DMP_GYRO_SF, (spp_uint32_t)gyro_sf_init);
    if (ret != SPP_OK) return ret;
    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    /* --- 23. Switch to continuous mode: only I2C master duty-cycled --- */
    buf[0] = WRITE_OP | REG_LP_CONF;
    buf[1] = 0x40;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_PWR_MGMT_1;
    buf[1] = 0x01;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* --- 24-26. Three enable sequences: DMP+FIFO on, clear DMP regs, sleep/wake --- */
    for (spp_uint8_t seq = 0; seq < 3; seq++)
    {
        buf[0] = WRITE_OP | REG_USER_CTRL;
        buf[1] = (seq == 0) ? 0x10 : 0xD0;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_PWR_MGMT_2;
        buf[1] = 0x47;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_USER_CTRL;
        buf[1] = 0xD0;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        ret = IcmDmpWrite16(p_data_icm, DMP_DATA_OUT_CTL1,    0x0000);
        if (ret != SPP_OK) return ret;
        ret = IcmDmpWrite16(p_data_icm, DMP_DATA_INTR_CTL,    0x0000);
        if (ret != SPP_OK) return ret;
        ret = IcmDmpWrite16(p_data_icm, DMP_DATA_OUT_CTL2,    0x0000);
        if (ret != SPP_OK) return ret;
        ret = IcmDmpWrite16(p_data_icm, DMP_MOTION_EVENT_CTL, 0x0000);
        if (ret != SPP_OK) return ret;

        if (seq < 2)
        {
            buf[0] = WRITE_OP | REG_PWR_MGMT_2;
            buf[1] = 0x7F;
            ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
            if (ret != SPP_OK) return ret;

            buf[0] = WRITE_OP | REG_PWR_MGMT_1;
            buf[1] = 0x41;
            ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
            if (ret != SPP_OK) return ret;

            SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1));

            buf[0] = WRITE_OP | REG_PWR_MGMT_1;
            buf[1] = 0x01;
            ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
            if (ret != SPP_OK) return ret;

            buf[0] = WRITE_OP | REG_PWR_MGMT_2;
            buf[1] = 0x7F;
            ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
            if (ret != SPP_OK) return ret;

            ret = IcmDmpWrite16(p_data_icm, DMP_DATA_RDY_STATUS, 0x0000);
            if (ret != SPP_OK) return ret;

            ret = IcmLpWakeCycle(p_data_icm);
            if (ret != SPP_OK) return ret;
        }
    }

    /* --- 27. Final DMP output configuration --- */
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_OUT_CTL1,    0x0808);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_INTR_CTL,    0x0808);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_OUT_CTL2,    0x0000);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_MOTION_EVENT_CTL, 0x0300);
    if (ret != SPP_OK) return ret;

    /* --- 28. Accel calibration parameters for 225 Hz --- */
    ret = IcmDmpWrite32(p_data_icm, DMP_ACCEL_ONLY_GAIN, 0x00E8BA2E);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite32(p_data_icm, DMP_ACCEL_ALPHA_VAR, 0x3D27D27D);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite32(p_data_icm, DMP_ACCEL_A_VAR,     0x02D82D83);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_ACCEL_CAL_INIT,  0x0000);
    if (ret != SPP_OK) return ret;

    /* --- 29. Final ODR: 1125/(1+4) = 225 Hz --- */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_ACCEL_SMPLRT_DIV_1;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_ACCEL_SMPLRT_DIV_2;
    buf[1] = 0x04;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_GYRO_SMPLRT_DIV;
    buf[1] = 0x04;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p_data_icm, DMP_ODR_QUAT6, 0x0000);
    if (ret != SPP_OK) return ret;

    /* --- 30. Recalculate GYRO_SF for final 225 Hz rate --- */
    spp_int32_t gyro_sf_final = IcmCalcGyroSf(pll);
    ret = IcmDmpWrite32(p_data_icm, DMP_GYRO_SF, (spp_uint32_t)gyro_sf_final);
    if (ret != SPP_OK) return ret;

    /* --- 31. Enable accel + gyro --- */
    buf[0] = WRITE_OP | REG_PWR_MGMT_2;
    buf[1] = 0x40;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_RDY_STATUS, 0x0003);
    if (ret != SPP_OK) return ret;

    /* --- 32. Repeat final config write as per captured traffic --- */
    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_OUT_CTL1,    0x0808);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_INTR_CTL,    0x0808);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_OUT_CTL2,    0x0000);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_MOTION_EVENT_CTL, 0x0300);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_ODR_QUAT6,        0x0000);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_PWR_MGMT_2;
    buf[1] = 0x40;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_RDY_STATUS, 0x0003);
    if (ret != SPP_OK) return ret;

    /* --- 33. Start DMP: LP mode on, polling begins after this --- */
    buf[0] = WRITE_OP | REG_PWR_MGMT_1;
    buf[1] = 0x21;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    return SPP_OK;
}