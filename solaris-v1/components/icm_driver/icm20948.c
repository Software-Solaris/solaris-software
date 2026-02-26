#include "icm20948.h"
#include "driver/spi_common.h"
#include "returntypes.h"
#include "spi.h"
#include "task.h"
#include "types.h"
#include <string.h>

/* ============================================================
 * NUEVOS DEFINES REQUERIDOS EN icm20948.h
 * (añadir si no existen)
 *
 * -- Registros hardware Bank 0 --
 * #define REG_INT_ENABLE              0x10
 * #define REG_FIFO_CFG                0x76
 * #define REG_HW_FIX_DISABLE          0x75
 * #define REG_SINGLE_FIFO_PRIORITY_SEL 0x26
 * #define REG_DMP_INT_STATUS          0x18
 * #define REG_INT_STATUS              0x19
 * #define REG_FIFO_COUNTH             0x70
 * #define REG_FIFO_R_W                0x72
 *
 * -- Registros hardware Bank 1 --
 * #define REG_TIMEBASE_CORRECTION_PLL 0x28
 *
 * -- Registros hardware Bank 2 --
 * #define REG_DMP_ADDR_MSB            0x50   // AGB2_REG_PRGM_START_ADDRH
 * #define REG_DMP_ADDR_LSB            0x51   // AGB2_REG_PRGM_START_ADDRL
 * #define REG_ACCEL_CONFIG_2          0x15
 *
 * -- Direcciones DMP (acceso via MEM_BANK_SEL + MEM_START_ADDR + MEM_R_W) --
 * #define DMP_DATA_OUT_CTL1           0x0040   // (4  * 16 + 0)
 * #define DMP_DATA_OUT_CTL2           0x0042   // (4  * 16 + 2)
 * #define DMP_DATA_INTR_CTL           0x004C   // (4  * 16 + 12)
 * #define DMP_MOTION_EVENT_CTL        0x004E   // (4  * 16 + 14)
 * #define DMP_DATA_RDY_STATUS         0x008A   // (8  * 16 + 10)
 * #define DMP_FIFO_WATERMARK          0x01FE   // (31 * 16 + 14)
 * #define DMP_BAC_RATE                0x030A   // (48 * 16 + 10)
 * #define DMP_B2S_RATE                0x0308   // (48 * 16 + 8)
 * #define DMP_GYRO_SF                 0x0130   // (19 * 16)
 * #define DMP_GYRO_FULLSCALE          0x048C   // (72 * 16 + 12)
 * #define DMP_ACC_SCALE               0x01E0   // (30 * 16)
 * #define DMP_ACC_SCALE2              0x04F4   // (79 * 16 + 4)
 * #define DMP_ACCEL_ONLY_GAIN         0x010C   // (16 * 16 + 12)
 * #define DMP_ACCEL_ALPHA_VAR         0x05B0   // (91 * 16)
 * #define DMP_ACCEL_A_VAR             0x05C0   // (92 * 16)
 * #define DMP_ACCEL_CAL_INIT          0x05E4   // (94 * 16 + 4)
 * #define DMP_ODR_QUAT6               0x00AC   // (10 * 16 + 12)
 * #define DMP_CPASS_MTX_00            0x0170   // (23 * 16 + 0)
 * #define DMP_CPASS_MTX_01            0x0174   // (23 * 16 + 4)
 * #define DMP_CPASS_MTX_02            0x0178   // (23 * 16 + 8)
 * #define DMP_CPASS_MTX_10            0x017C   // (23 * 16 + 12)
 * #define DMP_CPASS_MTX_11            0x0180   // (24 * 16)
 * #define DMP_CPASS_MTX_12            0x0184   // (24 * 16 + 4)
 * #define DMP_CPASS_MTX_20            0x0188   // (24 * 16 + 8)
 * #define DMP_CPASS_MTX_21            0x018C   // (24 * 16 + 12)
 * #define DMP_CPASS_MTX_22            0x0190   // (25 * 16)
 * #define DMP_B2S_MTX_00              0x0D00   // (208 * 16)
 * #define DMP_B2S_MTX_01              0x0D04   // (208 * 16 + 4)
 * #define DMP_B2S_MTX_02              0x0D08   // (208 * 16 + 8)
 * #define DMP_B2S_MTX_10              0x0D0C   // (208 * 16 + 12)
 * #define DMP_B2S_MTX_11              0x0D10   // (209 * 16)
 * #define DMP_B2S_MTX_12              0x0D14   // (209 * 16 + 4)
 * #define DMP_B2S_MTX_20              0x0D18   // (209 * 16 + 8)
 * #define DMP_B2S_MTX_21              0x0D1C   // (209 * 16 + 12)
 * #define DMP_B2S_MTX_22              0x0D20   // (210 * 16)
 * ============================================================ */

static spp_uint8_t data[2];

static const spp_uint8_t dmp3_image[] = {
    #include "icm20948_img.dmp3a.h"
};


/* ============================================================
 * Helper: escribe N bytes en memoria DMP.
 * Reescribe banco y direccion en cada byte (no usa auto-increment).
 * ============================================================ */
static retval_t IcmDmpWriteBytes(icm_data_t   *p_data_icm,
                                  spp_uint16_t  dmp_addr,
                                  const spp_uint8_t *bytes,
                                  spp_uint8_t   len)
{
    retval_t    ret  = SPP_ERROR;
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

static retval_t IcmDmpWrite32(icm_data_t *p_data_icm,
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

static retval_t IcmDmpWrite16(icm_data_t *p_data_icm,
                               spp_uint16_t dmp_addr,
                               spp_uint16_t value)
{
    spp_uint8_t bytes[2] = {
        (spp_uint8_t)(value >> 8),
        (spp_uint8_t)(value)
    };
    return IcmDmpWriteBytes(p_data_icm, dmp_addr, bytes, 2);
}

/* ============================================================
 * Helper: ciclo LP/wake que el tráfico SPI muestra repetidamente
 * durante la escritura de matrices de montaje.
 * PWR_MGMT_1 = 0x21 → 0x01
 * ============================================================ */
static retval_t IcmLpWakeCycle(icm_data_t *p_data_icm)
{
    retval_t    ret  = SPP_ERROR;
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

/* ============================================================
 * Helper: calculo de GYRO_SF segun Appendix II del AN-MAPPS.
 * ============================================================ */
static spp_int32_t IcmCalcGyroSf(spp_int8_t pll)
{
#define BASE_SAMPLE_RATE  1125
#define DMP_RUNNING_RATE  225
#define DMP_DIVIDER       (BASE_SAMPLE_RATE / DMP_RUNNING_RATE)  /* 5 */

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


/* ============================================================
 * IcmInit
 * ============================================================ */
retval_t IcmInit(void *p_data)
{
    icm_data_t *p_data_icm = (icm_data_t*)p_data;
    retval_t ret;
    void* p_handler_spi;

    p_handler_spi = SPP_HAL_SPI_GetHandler();
    ret = SPP_HAL_SPI_DeviceInit(p_handler_spi);
    if (ret != SPP_OK) return ret;

    p_data_icm->p_handler_spi = p_handler_spi;

    return SPP_OK;
}


/* ============================================================
 * IcmLoadDmp
 * Carga el firmware DMP en la SRAM del chip y lo verifica.
 * Dirección de carga: DMP_LOAD_START = 0x0090.
 * ============================================================ */
retval_t IcmLoadDmp(void *p_data)
{
    icm_data_t        *p_data_icm = (icm_data_t *)p_data;
    retval_t           ret        = SPP_ERROR;
    spp_uint8_t        buf[2]     = {0};
    spp_uint16_t       fw_size    = sizeof(dmp3_image);
    spp_uint16_t       memaddr    = DMP_LOAD_START;
    const spp_uint8_t *fw_ptr     = dmp3_image;

    if (p_data_icm->firmware_loaded) return SPP_OK;

    /* Banco 0 obligatorio para acceder a MEM_BANK_SEL/MEM_START_ADDR/MEM_R_W */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* Escritura byte a byte */
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

    /* Verificación */
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


/* ============================================================
 * IcmConfigDmpInit
 *
 * Implementa la secuencia EXACTA del tráfico SPI capturado por
 * ZaneL (Teensy-ICM-20948 / InvenSense Nucleo example).
 *
 * Secciones del tráfico:
 *   1.  WHO_AM_I + PWR_MGMT_1 = 0x01 + USER_CTRL = 0x10
 *   2.  PWR_MGMT_2 = 0x47 (disable pressure + gyro)
 *   3.  LP_CONFIG  = 0x70 (I2C + accel + gyro duty-cycled)
 *   4.  USER_CTRL  = 0x10 (SPI mode)
 *   5.  Cargar firmware DMP (0x0090)
 *   6.  Verificar firmware DMP
 *   7.  Bank 2: PRGM_START_ADDR = 0x1000
 *   8.  Bank 0: limpiar DATA_OUT_CTL1/2, DATA_INTR_CTL,
 *               MOTION_EVENT_CTL, DATA_RDY_STATUS a 0x0000
 *   9.  Bank 0x01: FIFO_WATERMARK = 0x0320 (800 bytes)
 *   10. INT_ENABLE = 0x02 (DMP_INT1_EN)
 *   11. INT_ENABLE_2 = 0x01 (FIFO overflow)
 *   12. SINGLE_FIFO_PRIORITY_SEL = 0xE4
 *   13. HW_FIX_DISABLE: leer 0x75, escribir 0x48
 *   14. Bank 2: GYRO_SMPLRT_DIV = 0x13, ACCEL_SMPLRT_DIV = 0x0013
 *   15. Bank 0x03: BAC_RATE = 0, B2S_RATE = 0
 *   16. FIFO_CFG = 0x00
 *   17. FIFO_RST = 0x1F → 0x1E
 *   18. FIFO_EN_1 = 0x00, FIFO_EN_2 = 0x00
 *   19. PWR_MGMT_1 = 0x21 → PWR_MGMT_2 = 0x7F → PWR_MGMT_1 = 0x61
 *   20. PWR_MGMT_1 = 0x21 → 0x01  (ciclos LP/wake)
 *   21. Bank 3: configurar I2C master + magnetómetro (WHO_AM_I)
 *   22. Bank 3: configurar I2C SLV1 para CNTL2 del magnetómetro
 *   23. Compass mounting matrix (DMP CPASS_MTX)
 *   24. B2S mounting matrix (DMP B2S_MTX) — repetida 4 veces por el firmware
 *   25. Bank 2: ACCEL_CONFIG = 0x02 (+/-4g, bypass DLPF)
 *              ACCEL_CONFIG_2 = 0x00
 *   26. DMP: ACC_SCALE = 0x04000000, ACC_SCALE2 = 0x00040000
 *   27. Bank 2: GYRO_CONFIG_1 = 0x07 (2000dps, DLPF on)
 *              GYRO_CONFIG_2 = 0x00
 *   28. Bank 1: leer TIMEBASE_CORRECTION_PLL → calcular GYRO_SF
 *   29. DMP: GYRO_SF, GYRO_FULLSCALE = 0x10000000
 *   30. LP_CONFIG = 0x40 (solo I2C master duty-cycled)
 *   31. PWR_MGMT_1 = 0x01
 *   32. USER_CTRL = 0x10 → PWR_MGMT_2 = 0x47 → USER_CTRL = 0xD0
 *   33. Limpiar DATA_OUT_CTL1/2, DATA_INTR_CTL, MOTION_EVENT_CTL a 0x0000
 *   34. PWR_MGMT_2 = 0x7F → PWR_MGMT_1 = 0x41 → 0x01
 *   35. PWR_MGMT_2 = 0x7F → DATA_RDY_STATUS = 0x0000
 *   36. PWR_MGMT_1 = 0x21 → 0x01
 *   37. USER_CTRL = 0xD0 → PWR_MGMT_2 = 0x47 → USER_CTRL = 0xD0  (x2)
 *   38. Limpiar DATA_OUT_CTL1/2, DATA_INTR_CTL, MOTION_EVENT_CTL a 0x0000
 *   39. PWR_MGMT_2 = 0x7F → PWR_MGMT_1 = 0x41 → 0x01
 *   40. PWR_MGMT_2 = 0x7F → DATA_RDY_STATUS = 0x0000
 *   41. PWR_MGMT_1 = 0x21 → 0x01
 *   42. USER_CTRL = 0xD0 → PWR_MGMT_2 = 0x47 → USER_CTRL = 0xD0
 *   43. DATA_OUT_CTL1 = 0x0808, DATA_INTR_CTL = 0x0808,
 *       DATA_OUT_CTL2 = 0x0000, MOTION_EVENT_CTL = 0x0300
 *   44. ACCEL_ONLY_GAIN, ACCEL_ALPHA_VAR, ACCEL_A_VAR, ACCEL_CAL_INIT
 *   45. Bank 2: ACCEL_SMPLRT_DIV = 0x0004, GYRO_SMPLRT_DIV = 0x04
 *   46. DMP: ODR_QUAT6 = 0x0000
 *   47. DMP: GYRO_SF (recalculado para 225Hz final)
 *   48. PWR_MGMT_2 = 0x40 (accel+gyro habilitados)
 *   49. DATA_RDY_STATUS = 0x0003
 *   50. PWR_MGMT_1 = 0x21  → sistema listo, polling de interrupción
 * ============================================================ */
retval_t IcmConfigDmpInit(void *p_data)
{
    icm_data_t *p_data_icm = (icm_data_t*)p_data;
    retval_t    ret        = SPP_ERROR;
    spp_uint8_t buf[2]     = {0};

    /* ----------------------------------------------------------------
     * 1. WHO_AM_I — debe devolver 0xEA
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = READ_OP | REG_WHO_AM_I;
    buf[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;
    if (buf[1] != 0xEA) return SPP_ERROR;

    /* PWR_MGMT_1 = 0x01 (clock auto) */
    buf[0] = WRITE_OP | REG_PWR_MGMT_1;
    buf[1] = 0x01;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* USER_CTRL = 0x10 (I2C_IF_DIS: SPI only) */
    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 2. PWR_MGMT_2 = 0x47 (disable pressure + all gyro axes)
     *    Giroscopio deshabilitado durante la carga del firmware.
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_PWR_MGMT_2;
    buf[1] = 0x47;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* WHO_AM_I extra (igual que el tráfico capturado) */
    buf[0] = READ_OP | REG_WHO_AM_I;
    buf[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 3. LP_CONFIG = 0x70: I2C + accel + gyro en duty-cycled mode.
     *    Este valor se usa DURANTE la carga y configuración inicial.
     *    Más adelante se cambiará a 0x40.
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_LP_CONF;
    buf[1] = 0x70;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 4. USER_CTRL = 0x10 (SPI mode, otra vez como hace el tráfico)
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 5 + 6. Cargar y verificar firmware DMP
     * ---------------------------------------------------------------- */
    ret = IcmLoadDmp((void *)p_data);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 7. Bank 2: PRGM_START_ADDR = 0x1000 (dirección de ejecución DMP)
     *    NOTA: 0x0090 = dirección de CARGA, 0x1000 = dirección de EJECUCIÓN
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_DMP_ADDR_MSB;   /* Bank 2 offset 0x50 */
    buf[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_DMP_ADDR_LSB;   /* Bank 2 offset 0x51, en el tráfico
                                                se escriben los 2 bytes de una vez */
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* Volver a Banco 0 */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 8. Limpiar registros DMP de control a 0x0000
     *    DATA_OUT_CTL1, DATA_OUT_CTL2, DATA_INTR_CTL,
     *    MOTION_EVENT_CTL, DATA_RDY_STATUS
     * ---------------------------------------------------------------- */
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

    /* ----------------------------------------------------------------
     * 9. FIFO_WATERMARK = 0x0320 (800 bytes)
     *    Registro en Bank DMP 0x01, offset 0xFE → dirección 0x01FE
     * ---------------------------------------------------------------- */
    ret = IcmDmpWrite16(p_data_icm, DMP_FIFO_WATERMARK, 0x0320);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 10. INT_ENABLE = 0x02 (DMP_INT1_EN: interrupción DMP en pin 1)
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_INT_ENABLE;
    buf[1] = 0x02;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 11. INT_ENABLE_2 = 0x01 (FIFO overflow para periférico 0)
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_INT_ENABLE_2;
    buf[1] = 0x01;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 12. SINGLE_FIFO_PRIORITY_SEL = 0xE4
     *     Prioridad máxima al FIFO del DMP
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_SINGLE_FIFO_PRIORITY_SEL;
    buf[1] = 0xE4;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 13. HW_FIX_DISABLE (reg 0x75): leer valor actual y escribir 0x48
     *     El tráfico lee 0x40 y luego escribe 0x48 (activa bit3)
     * ---------------------------------------------------------------- */
    buf[0] = READ_OP | REG_HW_FIX_DISABLE;
    buf[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;
    /* buf[1] contiene el valor leído (típicamente 0x40) — ignorar */

    buf[0] = WRITE_OP | REG_HW_FIX_DISABLE;
    buf[1] = 0x48;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 14. Bank 2: GYRO_SMPLRT_DIV = 0x13 (19), ACCEL_SMPLRT_DIV = 0x0013
     *     Esto configura ODR base a 1125/(1+19) = 56.25 Hz durante setup
     * ---------------------------------------------------------------- */
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

    /* Volver a Banco 0 */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 15. DMP Bank 0x03: BAC_RATE = 0x0000, B2S_RATE = 0x0000
     * ---------------------------------------------------------------- */
    ret = IcmDmpWrite16(p_data_icm, DMP_BAC_RATE, 0x0000);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p_data_icm, DMP_B2S_RATE, 0x0000);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 16. FIFO_CFG = 0x00
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_FIFO_CFG;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 17. FIFO_RST = 0x1F → 0x1E
     *     (reset todos los FIFOs, luego mantener periférico 0 activo)
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_FIFO_RST;
    buf[1] = 0x1F;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_FIFO_RST;
    buf[1] = 0x1E;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 18. FIFO_EN_1 = 0x00, FIFO_EN_2 = 0x00
     *     El DMP controla el FIFO directamente
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_FIFO_EN_1;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_FIFO_EN_2;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 19. Secuencia de power: PWR_MGMT_1=0x21 → PWR_MGMT_2=0x7F → sleep
     *     PWR_MGMT_1 = 0x21 (LP + auto clock)
     *     PWR_MGMT_2 = 0x7F (disable pressure + all accel + all gyro)
     *     PWR_MGMT_1 = 0x61 (sleep + LP + auto clock)
     *     <wait ~175us>
     *     PWR_MGMT_1 = 0x21
     *     <wait ~175us>
     *     PWR_MGMT_1 = 0x01 (solo auto clock, no LP, no sleep)
     *     <wait ~175us>
     * ---------------------------------------------------------------- */
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

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1)); /* ≥175us, usamos 1ms */

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

    /* ----------------------------------------------------------------
     * 20. Bank 3: configurar I2C master para magnetómetro
     *     Verificar WHO_AM_I del AK09916 (debe devolver 0x09 o 0x48)
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_3;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* Deshabilitar todos los canales I2C periférico */
    buf[0] = WRITE_OP | 0x05; /* I2C_PERIPH0_CTRL */
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x09; /* I2C_PERIPH1_CTRL */
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x0D; /* I2C_PERIPH2_CTRL */
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x11; /* I2C_PERIPH3_CTRL */
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* I2C_MST_CTRL = 0x10 (stop entre reads) */
    buf[0] = WRITE_OP | 0x01;
    buf[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* I2C_MST_ODR_CONFIG = 0x04 (ODR = 1.1 kHz) */
    buf[0] = WRITE_OP | 0x00;
    buf[1] = 0x04;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* I2C_PERIPH0_ADDR = 0x8C (READ del magnetómetro 0x0C) */
    buf[0] = WRITE_OP | 0x03;
    buf[1] = 0x8C;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* I2C_PERIPH0_REG = 0x00 (primer registro del AK09916) */
    buf[0] = WRITE_OP | 0x04;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* I2C_PERIPH0_CTRL = 0x81 (enable read, 1 byte) */
    buf[0] = WRITE_OP | 0x05;
    buf[1] = 0x81;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* Volver a Banco 0 y habilitar I2C master */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* USER_CTRL = 0x30: I2C_MST_EN + I2C_IF_DIS */
    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0x30;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(60)); /* espera que el master I2C lea */

    /* USER_CTRL = 0x10 (solo SPI) */
    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* Leer PERIPH_SENS_DATA_00: WHO_AM_I del AK09916 */
    buf[0] = READ_OP | 0x3B; /* REG_PERIPH_SENS_DATA_00 */
    buf[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;
    /* buf[1] debe ser 0x48 (AK09916 WHO_AM_I) — se puede verificar si se desea */

    /* ----------------------------------------------------------------
     * 21. Bank 3: configurar I2C SLV1 para poner AK09916 en modo
     *     continuo 100Hz (CNTL2 = 0x08) vía escritura
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_3;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* Deshabilitar SLV0 */
    buf[0] = WRITE_OP | 0x05;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* I2C_PERIPH1_ADDR = 0x0C (WRITE al magnetómetro) */
    buf[0] = WRITE_OP | 0x07;
    buf[1] = 0x0C;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* I2C_PERIPH1_REG = 0x31 (CNTL2 del AK09916) */
    buf[0] = WRITE_OP | 0x08;
    buf[1] = 0x31;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* I2C_PERIPH1_DO = 0x00 (el tráfico envía 0x00 aquí, modo power-down primero) */
    buf[0] = WRITE_OP | 0x0A;
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* I2C_PERIPH1_CTRL = 0x81 (enable, 1 byte) */
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

    /* Bank 3: deshabilitar SLV1 y SLV0 */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_3;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x09; /* I2C_PERIPH1_CTRL */
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x05; /* I2C_PERIPH0_CTRL */
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | 0x09; /* I2C_PERIPH1_CTRL (otra vez, tal como el tráfico) */
    buf[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* Volver a Banco 0 */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 22. Compass mounting matrix (DMP CPASS_MTX_xx)
     *     El tráfico capturado usa estos valores:
     *       MTX_00 = 0x09999999  (~+0.6, eje X magnetómetro)
     *       MTX_11 = 0xF6666667  (~-0.6, eje Y magnetómetro)
     *       MTX_22 = 0xF6666667  (~-0.6, eje Z magnetómetro)
     *       Resto  = 0x00000000
     *
     *     AJUSTA estos valores según la orientación física de tu placa.
     *     Para matriz identidad usa 0x40000000 en la diagonal.
     * ---------------------------------------------------------------- */
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

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1)); /* ≥175us */

    /* ----------------------------------------------------------------
     * 23. B2S mounting matrix (DMP B2S_MTX_xx)
     *     El tráfico repite esta matriz 4 veces — la implementación de
     *     InvenSense hace múltiples intentos de escritura. Lo replicamos.
     *     Cada escritura de 32 bits va seguida del ciclo LP/wake.
     *     Matriz identidad: diagonal = 0x40000000, resto = 0x00000000
     * ---------------------------------------------------------------- */
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

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1)); /* ≥1ms (tráfico muestra ~1ms aquí) */

    /* ----------------------------------------------------------------
     * 24. Bank 2: ACCEL_CONFIG = 0x02 (+/-4g, bypass DLPF)
     *     El tráfico lee el valor actual y lo escribe dos veces
     *     (patrón de read-modify-write del SDK de InvenSense)
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* Leer valor actual de ACCEL_CONFIG (0x14) */
    buf[0] = READ_OP | REG_ACCEL_CONFIG;   /* Bank 2, reg 0x14 */
    buf[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(1)); /* ≥125us */

    /* Escribir ACCEL_CONFIG = 0x02 */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_ACCEL_CONFIG;
    buf[1] = 0x02;   /* ACCEL_FCHOICE=0 (bypass DLPF), +/-4g, 4500Hz */
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    /* Leer/escribir ACCEL_CONFIG_2 = 0x00 (DEC3_CFG=0, 1x averaging) */
    buf[0] = WRITE_OP | REG_BANK_SEL;
    buf[1] = REG_BANK_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = READ_OP | REG_ACCEL_CONFIG_2;  /* Bank 2, reg 0x15 */
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

    /* ----------------------------------------------------------------
     * 25. DMP: ACC_SCALE = 0x04000000, ACC_SCALE2 = 0x00040000
     *     (dos veces, como hace el tráfico capturado)
     * ---------------------------------------------------------------- */
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

    /* ----------------------------------------------------------------
     * 26. Bank 2: GYRO_CONFIG_1 = 0x07 (+/-2000dps, DLPF on)
     *             GYRO_CONFIG_2 = 0x00 (1x averaging)
     *     (tres veces según el tráfico)
     * ---------------------------------------------------------------- */
    for (spp_uint8_t rep = 0; rep < 3; rep++)
    {
        buf[0] = WRITE_OP | REG_BANK_SEL;
        buf[1] = REG_BANK_2;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        /* Leer GYRO_CONFIG_1 */
        buf[0] = READ_OP | REG_GYRO_CONFIG;  /* Bank 2, reg 0x01 */
        buf[1] = EMPTY_MESSAGE;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_BANK_SEL;
        buf[1] = REG_BANK_0;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        /* Escribir GYRO_CONFIG_1 = 0x07 */
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

        /* Leer GYRO_CONFIG_2 */
        buf[0] = WRITE_OP | REG_BANK_SEL;
        buf[1] = REG_BANK_2;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = READ_OP | 0x02;  /* REG_GYRO_CONFIG_2 */
        buf[1] = EMPTY_MESSAGE;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        buf[0] = WRITE_OP | REG_BANK_SEL;
        buf[1] = REG_BANK_0;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
        if (ret != SPP_OK) return ret;

        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;

        /* Escribir GYRO_CONFIG_2 = 0x00 */
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

        /* GYRO_FULLSCALE = 0x10000000 (en banco DMP 0x04) */
        ret = IcmDmpWrite32(p_data_icm, DMP_GYRO_FULLSCALE, 0x10000000);
        if (ret != SPP_OK) return ret;
        ret = IcmLpWakeCycle(p_data_icm);
        if (ret != SPP_OK) return ret;
    }

    /* ----------------------------------------------------------------
     * 27. Bank 1: leer TIMEBASE_CORRECTION_PLL → calcular GYRO_SF
     *     (primer valor, para la fase de configuración)
     * ---------------------------------------------------------------- */
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

    /* GYRO_SF primera escritura (valor para 56Hz, DMP_DIVIDER=20) */
    spp_int32_t gyro_sf_init = IcmCalcGyroSf(pll);
    ret = IcmDmpWrite32(p_data_icm, DMP_GYRO_SF, (spp_uint32_t)gyro_sf_init);
    if (ret != SPP_OK) return ret;
    ret = IcmLpWakeCycle(p_data_icm);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 28. LP_CONFIG = 0x40 (solo I2C master duty-cycled, accel+gyro CONTINUO)
     *     CRÍTICO: cambio de 0x70 → 0x40 para que DMP funcione
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_LP_CONF;
    buf[1] = 0x40;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_PWR_MGMT_1;
    buf[1] = 0x01;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 29. Primera secuencia de enable:
     *     USER_CTRL = 0x10 → PWR_MGMT_2 = 0x47 → USER_CTRL = 0xD0
     *     Luego limpiar registros DMP y hacer ciclo sleep/wake
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_PWR_MGMT_2;
    buf[1] = 0x47;  /* disable pressure + gyro */
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0xD0;  /* DMP_EN + FIFO_EN + I2C_IF_DIS */
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* Limpiar registros DMP */
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_OUT_CTL1,    0x0000);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_INTR_CTL,    0x0000);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_OUT_CTL2,    0x0000);
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite16(p_data_icm, DMP_MOTION_EVENT_CTL, 0x0000);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_PWR_MGMT_2;
    buf[1] = 0x7F;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    buf[0] = WRITE_OP | REG_PWR_MGMT_1;
    buf[1] = 0x41;  /* sleep + auto clock */
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

    /* ----------------------------------------------------------------
     * 30. Segunda secuencia de enable (idéntica)
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0xD0;
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

    /* ----------------------------------------------------------------
     * 31. Tercera secuencia de enable
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_USER_CTRL;
    buf[1] = 0xD0;
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

    /* ----------------------------------------------------------------
     * 32. Escritura final de registros DMP con valores REALES
     *     DATA_OUT_CTL1  = 0x0808 (Quat6 + Header2)
     *     DATA_INTR_CTL  = 0x0808
     *     DATA_OUT_CTL2  = 0x0000
     *     MOTION_EVENT_CTL = 0x0300 (gyro cal + accel cal)
     * ---------------------------------------------------------------- */
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_OUT_CTL1,    0x0808);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_INTR_CTL,    0x0808);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_OUT_CTL2,    0x0000);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p_data_icm, DMP_MOTION_EVENT_CTL, 0x0300);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 33. ACCEL_ONLY_GAIN, ACCEL_ALPHA_VAR, ACCEL_A_VAR, ACCEL_CAL_INIT
     *     Valores para ODR = 225 Hz
     * ---------------------------------------------------------------- */
    ret = IcmDmpWrite32(p_data_icm, DMP_ACCEL_ONLY_GAIN, 0x00E8BA2E);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p_data_icm, DMP_ACCEL_ALPHA_VAR, 0x3D27D27D);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p_data_icm, DMP_ACCEL_A_VAR,     0x02D82D83);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p_data_icm, DMP_ACCEL_CAL_INIT,  0x0000);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 34. Bank 2: ACCEL_SMPLRT_DIV = 0x0004, GYRO_SMPLRT_DIV = 0x04
     *     ODR final: 1125/(1+4) = 225 Hz
     * ---------------------------------------------------------------- */
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

    /* ----------------------------------------------------------------
     * 35. DMP: ODR_QUAT6 = 0x0000 (ODR máximo, sin decimación)
     * ---------------------------------------------------------------- */
    ret = IcmDmpWrite16(p_data_icm, DMP_ODR_QUAT6, 0x0000);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 36. DMP: GYRO_SF final para 225 Hz
     *     El tráfico recalcula con el mismo pll leído antes
     * ---------------------------------------------------------------- */
    spp_int32_t gyro_sf_final = IcmCalcGyroSf(pll);
    ret = IcmDmpWrite32(p_data_icm, DMP_GYRO_SF, (spp_uint32_t)gyro_sf_final);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 37. PWR_MGMT_2 = 0x40 (solo disable pressure, accel+gyro HABILITADOS)
     *     CRÍTICO: este es el valor que activa los sensores para el DMP.
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_PWR_MGMT_2;
    buf[1] = 0x40;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 38. DATA_RDY_STATUS = 0x0003 (gyro bit0 + accel bit1)
     * ---------------------------------------------------------------- */
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_RDY_STATUS, 0x0003);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------------
     * 39. Repetir una vez más DATA_OUT_CTL1/2, DATA_INTR_CTL,
     *     MOTION_EVENT_CTL, ODR_QUAT6, PWR_MGMT_2, DATA_RDY_STATUS
     *     (el tráfico lo escribe dos veces antes del polling)
     * ---------------------------------------------------------------- */
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

    /* ----------------------------------------------------------------
     * 40. PWR_MGMT_1 = 0x21 (LP + auto clock) — inicio del polling
     *     A partir de aquí el DMP está corriendo y generará
     *     interrupción DMP_INT1 cuando haya datos en el FIFO.
     * ---------------------------------------------------------------- */
    buf[0] = WRITE_OP | REG_PWR_MGMT_1;
    buf[1] = 0x21;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, buf, 2);
    if (ret != SPP_OK) return ret;

    return SPP_OK;
}