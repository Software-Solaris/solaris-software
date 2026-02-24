#include "icm20948.h"
#include "driver/spi_common.h"
#include "returntypes.h"
#include "spi.h"
#include "task.h"
#include "types.h"
#include <string.h>

static spp_uint8_t data[2];
static spp_uint8_t data_2[2];

static const spp_uint8_t dmp3_image[] = {
    #include "icm20948_img.dmp3a.h"
};


/**
 * Nuevos defines necesarios en icm20948.h:
 *
 * -- registros hardware --
 * #define REG_INT_ENABLE              0x10   // Bank 0
 * #define REG_TIMEBASE_CORRECTION_PLL 0x28   // Bank 1
 * #define REG_BANK_1                  0x10
 *
 * -- direcciones DMP (escritura via MEM_BANK_SEL + MEM_START_ADDR + MEM_R_W) --
 * #define DMP_DATA_OUT_CTL1           (4  * 16)       // 0x0040
 * #define DMP_DATA_OUT_CTL2           (4  * 16 + 2)   // 0x0042
 * #define DMP_DATA_INTR_CTL           (4  * 16 + 12)  // 0x004C
 * #define DMP_MOTION_EVENT_CTL        (4  * 16 + 14)  // 0x004E
 * #define DMP_DATA_RDY_STATUS         (8  * 16 + 10)  // 0x008A
 * #define DMP_ODR_ACCEL               (11 * 16 + 14)  // 0x00BE
 * #define DMP_ODR_GYRO                (11 * 16 + 10)  // 0x00BA
 * #define DMP_ODR_CPASS               (11 * 16 + 6)   // 0x00B6
 * #define DMP_GYRO_SF                 (19 * 16)        // 0x0130
 * #define DMP_ACC_SCALE               (30 * 16)        // 0x01E0
 * #define DMP_ACC_SCALE2              (79 * 16 + 4)   // 0x04F4
 * #define DMP_CPASS_MTX_00            (23 * 16)        // 0x0170
 * #define DMP_CPASS_MTX_01            (23 * 16 + 4)   // 0x0174
 * #define DMP_CPASS_MTX_02            (23 * 16 + 8)   // 0x0178
 * #define DMP_CPASS_MTX_10            (23 * 16 + 12)  // 0x017C
 * #define DMP_CPASS_MTX_11            (24 * 16)        // 0x0180
 * #define DMP_CPASS_MTX_12            (24 * 16 + 4)   // 0x0184
 * #define DMP_CPASS_MTX_20            (24 * 16 + 8)   // 0x0188
 * #define DMP_CPASS_MTX_21            (24 * 16 + 12)  // 0x018C
 * #define DMP_CPASS_MTX_22            (25 * 16)        // 0x0190
 */


/* ============================================================
 * Helper: escribe un valor de 32 bits en un registro DMP interno.
 * Los registros DMP de 32 bits se almacenan en big-endian.
 * Se escribe byte a byte reescribiendo la direccion en cada paso.
 * ============================================================ */
static retval_t IcmDmpWrite32(icm_data_t *p_data_icm,
                               spp_uint16_t dmp_addr,
                               spp_uint32_t value)
{
    retval_t    ret     = SPP_ERROR;
    spp_uint8_t data[2] = {0};
    spp_uint8_t bytes[4];

    /* Big-endian: byte mas significativo primero */
    bytes[0] = (spp_uint8_t)(value >> 24);
    bytes[1] = (spp_uint8_t)(value >> 16);
    bytes[2] = (spp_uint8_t)(value >> 8);
    bytes[3] = (spp_uint8_t)(value);

    for (spp_uint8_t i = 0; i < 4; i++)
    {
        data[0] = WRITE_OP | REG_MEM_BANK_SEL;
        data[1] = (spp_uint8_t)((dmp_addr + i) >> 8);
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
        if (ret != SPP_OK) return ret;

        data[0] = WRITE_OP | REG_MEM_START_ADDR;
        data[1] = (spp_uint8_t)((dmp_addr + i) & 0xFF);
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
        if (ret != SPP_OK) return ret;

        data[0] = WRITE_OP | REG_MEM_R_W;
        data[1] = bytes[i];
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
        if (ret != SPP_OK) return ret;
    }

    return SPP_OK;
}


/* ============================================================
 * Helper: escribe un valor de 16 bits en un registro DMP interno.
 * Big-endian, byte a byte.
 * ============================================================ */
static retval_t IcmDmpWrite16(icm_data_t *p_data_icm,
                               spp_uint16_t dmp_addr,
                               spp_uint16_t value)
{
    retval_t    ret     = SPP_ERROR;
    spp_uint8_t data[2] = {0};
    spp_uint8_t bytes[2];

    bytes[0] = (spp_uint8_t)(value >> 8);
    bytes[1] = (spp_uint8_t)(value);

    for (spp_uint8_t i = 0; i < 2; i++)
    {
        data[0] = WRITE_OP | REG_MEM_BANK_SEL;
        data[1] = (spp_uint8_t)((dmp_addr + i) >> 8);
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
        if (ret != SPP_OK) return ret;

        data[0] = WRITE_OP | REG_MEM_START_ADDR;
        data[1] = (spp_uint8_t)((dmp_addr + i) & 0xFF);
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
        if (ret != SPP_OK) return ret;

        data[0] = WRITE_OP | REG_MEM_R_W;
        data[1] = bytes[i];
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
        if (ret != SPP_OK) return ret;
    }

    return SPP_OK;
}


/* ============================================================
 * Helper: calculo de GYRO_SF segun Appendix II del AN-MAPPS.
 * Requiere leer TIMEBASE_CORRECTION_PLL del chip (Bank 1, 0x28).
 * ============================================================ */
static spp_int32_t IcmCalcGyroSf(spp_int8_t pll)
{
#define BASE_SAMPLE_RATE     1125
#define DMP_RUNNING_RATE     225
#define DMP_DIVIDER          (BASE_SAMPLE_RATE / DMP_RUNNING_RATE)  /* 5 */

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
 * IcmDmpStart
 *
 * Completa la puesta en marcha del DMP una vez que el firmware
 * ha sido cargado por IcmLoadDmp(). Implementa las secciones
 * 3.6.3, 3.7, 3.8, 3.9, 3.10 y 4 del AN-MAPPS.
 * ============================================================ */
retval_t IcmDmpStart(void *p_data)
{
    icm_data_t  *p_data_icm = (icm_data_t *)p_data;
    retval_t     ret        = SPP_ERROR;
    spp_uint8_t  data[2]    = {0};

    /* ------------------------------------------------------------------
     * Seccion 3.6.3: Escribir DMP start address y habilitar el DMP.
     * PRGM_STRT_ADDRH/L estan en Banco 2.
     * ------------------------------------------------------------------ */
    data[0] = WRITE_OP | REG_BANK_SEL;
    data[1] = REG_BANK_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    data[0] = WRITE_OP | REG_DMP_ADDR_MSB;
    data[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    data[0] = WRITE_OP | REG_DMP_ADDR_LSB;
    data[1] = 0x90;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* Volver a Banco 0 para el resto de operaciones */
    data[0] = WRITE_OP | REG_BANK_SEL;
    data[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ------------------------------------------------------------------
     * Seccion 3.7: Escalado del acelerometro para el DMP.
     * FSR = 4g → ACC_SCALE = 0x04000000 (alinea 1g = 2^25)
     *            ACC_SCALE2 = 0x00040000 (convierte de vuelta a HW units)
     * ------------------------------------------------------------------ */
    ret = IcmDmpWrite32(p_data_icm, DMP_ACC_SCALE,  0x04000000);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite32(p_data_icm, DMP_ACC_SCALE2, 0x00040000);
    if (ret != SPP_OK) return ret;

    /* ------------------------------------------------------------------
     * Seccion 3.8: Matriz de montaje del magnetometro.
     * Escala: 1 uT = 2^30. Matriz identidad (sin rotacion).
     * Ajusta los valores segun la orientacion fisica de tu hardware.
     * ------------------------------------------------------------------ */
    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_00, 0x40000000); /* 1.0 */
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_01, 0x00000000); /* 0.0 */
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_02, 0x00000000); /* 0.0 */
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_10, 0x00000000); /* 0.0 */
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_11, 0x40000000); /* 1.0 */
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_12, 0x00000000); /* 0.0 */
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_20, 0x00000000); /* 0.0 */
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_21, 0x00000000); /* 0.0 */
    if (ret != SPP_OK) return ret;
    ret = IcmDmpWrite32(p_data_icm, DMP_CPASS_MTX_22, 0x40000000); /* 1.0 */
    if (ret != SPP_OK) return ret;

    /* ------------------------------------------------------------------
     * Seccion 3.9: Reset del FIFO y del DMP.
     * ------------------------------------------------------------------ */
    data[0] = WRITE_OP | REG_FIFO_RST;
    data[1] = 0x1F;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    data[0] = WRITE_OP | REG_FIFO_RST;
    data[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* Reset DMP (bit3) y habilitar DMP (bit7) + I2C_IF_DIS (bit4) */
    data[0] = WRITE_OP | REG_USER_CTRL;
    data[1] = 0x88; /* DMP_EN=1, DMP_RST=1 */
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(10));

    /* Dejar solo DMP_EN e I2C_IF_DIS activos, quitar DMP_RST */
    data[0] = WRITE_OP | REG_USER_CTRL;
    data[1] = 0x90; /* DMP_EN=1, I2C_IF_DIS=1 */
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ------------------------------------------------------------------
     * Seccion 3.10: Habilitar interrupcion del DMP.
     * INT_ENABLE bit3 = DMP_INT1_EN
     * ------------------------------------------------------------------ */
    data[0] = WRITE_OP | REG_INT_ENABLE;
    data[1] = 0x02; /* DMP_INT1_EN */
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ------------------------------------------------------------------
     * Seccion 4.2: Configurar que datos envia el DMP al FIFO.
     * DATA_OUT_CTL1: accel (0x8000) + gyro (0x4000) + compass (0x2000)
     * ------------------------------------------------------------------ */
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_OUT_CTL1,
                        0x8000 |   /* 16-bit accel   */
                        0x4000 |   /* 16-bit gyro    */
                        0x2000);   /* 16-bit compass */
    if (ret != SPP_OK) return ret;

    /* DATA_OUT_CTL2: sin batch mode, sin features extra por ahora */
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_OUT_CTL2, 0x0000);
    if (ret != SPP_OK) return ret;

    /* DATA_INTR_CTL: mismos sensores que DATA_OUT_CTL1 generan interrupcion */
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_INTR_CTL,
                        0x8000 |
                        0x4000 |
                        0x2000);
    if (ret != SPP_OK) return ret;

    /* ------------------------------------------------------------------
     * Seccion 4.2: DATA_RDY_STATUS — indicar al DMP que sensores hay.
     * bit0 = gyro, bit1 = accel, bit3 = secondary (magnetometro)
     * ------------------------------------------------------------------ */
    ret = IcmDmpWrite16(p_data_icm, DMP_DATA_RDY_STATUS,
                        0x0001 |   /* gyro disponible    */
                        0x0002 |   /* accel disponible   */
                        0x0008);   /* secondary (mag)    */
    if (ret != SPP_OK) return ret;

    /* ------------------------------------------------------------------
     * Seccion 4.3: ODR de cada sensor.
     * Formula: valor = (225 / ODR_Hz) - 1
     * Accel y gyro a 56 Hz → (225/56) - 1 ≈ 3
     * Compass a 56 Hz      → (225/56) - 1 ≈ 3
     * ------------------------------------------------------------------ */
    ret = IcmDmpWrite16(p_data_icm, DMP_ODR_ACCEL, 3);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p_data_icm, DMP_ODR_GYRO, 3);
    if (ret != SPP_OK) return ret;

    ret = IcmDmpWrite16(p_data_icm, DMP_ODR_CPASS, 3);
    if (ret != SPP_OK) return ret;

    /* ------------------------------------------------------------------
     * Seccion 4.5: MOTION_EVENT_CTL — habilitar calibraciones.
     * bit8 = gyro cal, bit9 = accel cal, bit7 = compass cal
     * ------------------------------------------------------------------ */
    ret = IcmDmpWrite16(p_data_icm, DMP_MOTION_EVENT_CTL,
                        0x0100 |   /* Gyro calibration   */
                        0x0200 |   /* Accel calibration  */
                        0x0080);   /* Compass calibration*/
    if (ret != SPP_OK) return ret;

    /* ------------------------------------------------------------------
     * Seccion 4.5.10: GYRO_SF — factor de escala del giroscopio.
     * Requiere leer TIMEBASE_CORRECTION_PLL de Bank 1 (0x28).
     * ------------------------------------------------------------------ */
    data[0] = WRITE_OP | REG_BANK_SEL;
    data[1] = REG_BANK_1;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    data[0] = READ_OP | REG_TIMEBASE_CORRECTION_PLL;
    data[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* Volver a Banco 0 antes de escribir en DMP */
    spp_int8_t pll = (spp_int8_t)data[1];

    data[0] = WRITE_OP | REG_BANK_SEL;
    data[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    spp_int32_t gyro_sf = IcmCalcGyroSf(pll);
    ret = IcmDmpWrite32(p_data_icm, DMP_GYRO_SF, (spp_uint32_t)gyro_sf);
    if (ret != SPP_OK) return ret;

    return SPP_OK;
}



/* -------------------------------------------------------------------------- */
/*                               INITIALIZATION                               */
/* -------------------------------------------------------------------------- */

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

/* -------------------------------------------------------------------------- */
/*                               ICM CONFIG                                   */
/* -------------------------------------------------------------------------- */
retval_t IcmLoadDmp(void *p_data)
{
    icm_data_t        *p_data_icm = (icm_data_t *)p_data;
    retval_t           ret        = SPP_ERROR;
    spp_uint8_t        data[2]    = {0};
    spp_uint16_t       fw_size    = sizeof(dmp3_image);
    spp_uint16_t       memaddr    = DMP_LOAD_START;
    const spp_uint8_t *fw_ptr     = dmp3_image;

    /* Si el firmware ya esta cargado no hace falta repetir */
    if (p_data_icm->firmware_loaded) return SPP_OK;

    /* ------------------------------------------------------------------
     * Asegurarse en Banco 0: los registros MEM_BANK_SEL, MEM_START_ADDR
     * y MEM_R_W solo son accesibles desde Banco 0.
     * ------------------------------------------------------------------ */
    data[0] = WRITE_OP | REG_BANK_SEL;
    data[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ------------------------------------------------------------------
     * Escritura del firmware byte a byte en la SRAM del DMP.
     *
     * Para cada byte:
     *   MEM_BANK_SEL   = memaddr >> 8      (pagina de 256 bytes)
     *   MEM_START_ADDR = memaddr & 0xFF    (offset dentro de la pagina)
     *   MEM_R_W        = byte del firmware
     *
     * SPP_HAL_SPI_Transmit solo puede enviar 2 bytes [reg, dato] por
     * llamada, por lo que no es posible un burst real. El auto-incremento
     * del puntero interno del DMP no se usa: se reescribe la direccion
     * completa en cada iteracion para garantizar integridad.
     * ------------------------------------------------------------------ */
    while (fw_size > 0)
    {
        data[0] = WRITE_OP | REG_MEM_BANK_SEL;
        data[1] = (spp_uint8_t)(memaddr >> 8);
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
        if (ret != SPP_OK) return ret;

        data[0] = WRITE_OP | REG_MEM_START_ADDR;
        data[1] = (spp_uint8_t)(memaddr & 0xFF);
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
        if (ret != SPP_OK) return ret;

        data[0] = WRITE_OP | REG_MEM_R_W;
        data[1] = *fw_ptr;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
        if (ret != SPP_OK) return ret;

        fw_ptr++;
        memaddr++;
        fw_size--;
    }

    /* ------------------------------------------------------------------
     * Verificacion: leer de vuelta cada byte y comparar con el original.
     * ------------------------------------------------------------------ */
    fw_size = sizeof(dmp3_image);
    memaddr = DMP_LOAD_START;
    fw_ptr  = dmp3_image;

    while (fw_size > 0)
    {
        data[0] = WRITE_OP | REG_MEM_BANK_SEL;
        data[1] = (spp_uint8_t)(memaddr >> 8);
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
        if (ret != SPP_OK) return ret;

        data[0] = WRITE_OP | REG_MEM_START_ADDR;
        data[1] = (spp_uint8_t)(memaddr & 0xFF);
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
        if (ret != SPP_OK) return ret;

        data[0] = READ_OP | REG_MEM_R_W;
        data[1] = EMPTY_MESSAGE;
        ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
        if (ret != SPP_OK) return ret;

        if (data[1] != *fw_ptr) return SPP_ERROR;

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
 * Secuencia completa de inicializacion segun AN-MAPPS (commands.txt)
 * mas prueba de escritura/lectura 0xFABADA en la SRAM del DMP.
 * ============================================================ */
retval_t IcmConfigDmpInit(void *p_data)
{
    icm_data_t *p_data_icm = (icm_data_t*)p_data;
    retval_t ret = SPP_ERROR;
    spp_uint8_t data[2] = {0};

    /* ----------------------------------------------------------
     * VERIFICACION PREVIA: WHO_AM_I
     * El ICM-20948 debe responder 0xEA.
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_BANK_SEL;
    data[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    data[0] = READ_OP | REG_WHO_AM_I;
    data[1] = EMPTY_MESSAGE;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;
    if (data[1] != 0xEA) return SPP_ERROR;

    /* ----------------------------------------------------------
     * PASO 1: Asegurarse en Banco 0
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_BANK_SEL;
    data[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------
     * PASO 2: Reset completo del chip
     * PWR_MGMT_1 bit7 = DEVICE_RESET. Espera obligatoria 100ms.
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_PWR_MGMT_1;
    data[1] = BIT_H_RESET;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;
    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(100));

    /* ----------------------------------------------------------
     * PASO 3: Despertar el chip y seleccionar clock automatico
     * PWR_MGMT_1: SLEEP=0, CLKSEL=001 (auto-PLL). Espera 20ms.
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_PWR_MGMT_1;
    data[1] = 0x01;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;
    SPP_OSAL_TaskDelay(pdMS_TO_TICKS(20));

    /* ----------------------------------------------------------
     * PASO 4: Habilitar acelerometro y giroscopio
     * PWR_MGMT_2: DISABLE_GYRO=000, DISABLE_ACCEL=000
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_PWR_MGMT_2;
    data[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------
     * PASO 5: Configurar Low Power Mode
     * LP_CONFIG: I2C_MST_CYCLE=1, ACCEL_CYCLE=1, GYRO_CYCLE=1
     * 0x70 = 0b01110000
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_LP_CONF;
    data[1] = 0x70;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------
     * PASO 6: Habilitar interrupcion de overflow de FIFO
     * INT_ENABLE_2 bit0 = FIFO_OVERFLOW_EN = 1
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_INT_ENABLE_2;
    data[1] = 0x01;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------
     * PASO 7: Apagar todos los sensores que van al FIFO
     * El DMP controlara el FIFO cuando este activo.
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_FIFO_EN_1;
    data[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    data[0] = WRITE_OP | REG_FIFO_EN_2;
    data[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------
     * PASO 8: Apagar interrupcion Data Ready
     * INT_ENABLE_1 bit0 = RAW_DATA_0_RDY_EN = 0
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_INT_ENABLE_1;
    data[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------
     * PASO 9: Reset del FIFO
     * Resetear los 5 FIFOs y luego liberar.
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_FIFO_RST;
    data[1] = 0x1F;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    data[0] = WRITE_OP | REG_FIFO_RST;
    data[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------
     * PASO 10: Cambiar a Banco 2
     * Los registros de configuracion de sensores estan en Banco 2.
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_BANK_SEL;
    data[1] = REG_BANK_2;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------
     * PASO 11: Sample rate del giroscopio = 225 Hz
     * GYRO_SMPLRT_DIV: Output = 1125 / (1 + 4) = 225 Hz
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_GYRO_SMPLRT_DIV;
    data[1] = 0x04;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------
     * PASO 12: FSR del giroscopio = 2000 dps
     * GYRO_CONFIG_1: GYRO_FS_SEL=11 (2000dps), FCHOICE=1 (DLPF on)
     * 0x07 = 0b00000111
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_GYRO_CONFIG;
    data[1] = 0x07;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------
     * PASO 13: Sample rate del acelerometro = 225 Hz
     * Divisor de 12 bits: byte alto=0x00, byte bajo=0x04
     * Output = 1125 / (1 + 4) = 225 Hz
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_ACCEL_SMPLRT_DIV_1;
    data[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    data[0] = WRITE_OP | REG_ACCEL_SMPLRT_DIV_2;
    data[1] = 0x04;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------
     * PASO 14: FSR del acelerometro = 4g
     * ACCEL_CONFIG: ACCEL_FS_SEL=001 (4g), FCHOICE=1 (DLPF on)
     * 0x03 = 0b00000011
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_ACCEL_CONFIG;
    data[1] = 0x03;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------
     * PASO 15: Configurar DMP start address = 0x0090
     * PRGM_STRT_ADDRH = 0x00, PRGM_STRT_ADDRL = 0x90
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_DMP_ADDR_MSB;
    data[1] = 0x00;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    data[0] = WRITE_OP | REG_DMP_ADDR_LSB;
    data[1] = 0x90;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------
     * PASO 16: Volver a Banco 0
     * MEM_BANK_SEL, MEM_START_ADDR y MEM_R_W solo son accesibles
     * desde Banco 0.
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_BANK_SEL;
    data[1] = REG_BANK_0;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    /* ----------------------------------------------------------
     * PASO 17: Configurar USER_CTRL
     * bit4 = I2C_IF_DIS = 1 (forzar modo SPI, deshabilitar I2C)
     * ---------------------------------------------------------- */
    data[0] = WRITE_OP | REG_USER_CTRL;
    data[1] = 0x10;
    ret = SPP_HAL_SPI_Transmit(p_data_icm->p_handler_spi, data, 2);
    if (ret != SPP_OK) return ret;

    ret = IcmLoadDmp((void *)p_data);
    if (ret != SPP_OK){
        return ret;
    }

    ret = IcmDmpStart((void*)p_data);
    if (ret != SPP_OK){
        return ret;
    }

    return SPP_OK;
}



// /* -------------------------------------------------------------------------- */
// /*                                 TASK LOOP                                  */
// /* -------------------------------------------------------------------------- */

// void IcmGetSensorsData(void *p_data)
// {
//     for (;;)
//     {
//         IcmReadSensors(p_data);
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }
