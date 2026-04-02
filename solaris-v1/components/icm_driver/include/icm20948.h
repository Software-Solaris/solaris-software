#ifndef ICM20948_H
#define ICM20948_H

#include "spp/core/returntypes.h"
#include "spp/core/types.h"


/**
 * @file icm20948.h
 * @brief Public definitions and register abstractions for the I2C/SPI driver
 *        of the ICM20948 IMU.
 *
 * This header provides:
 * - task priorities and hardware pin definitions,
 * - register addresses and DMP memory map constants,
 * - typed register unions for bit-level register composition,
 * - public driver context definition,
 * - public API declarations.
 *
 * Naming conventions used in this file:
 * - Global constants/macros: K_ICM20948_*
 * - Types: ICM20948_*_t
 * - Public functions: ICM20948_*
 * - Pointer variables/parameters: p_pointerName
 */

/* ----------------------------------------------------------------
 * Task priorities
 * ---------------------------------------------------------------- */

/**
 * @brief Priority of the main ICM20948 task.
 */
#define K_ICM20948_TASK_PRIORITY 4U

/**
 * @brief Priority of the ICM20948 configuration task.
 */
#define K_ICM20948_CONFIG_TASK_PRIORITY 5U

/**
 * @brief Priority of the sensor read task.
 */
#define K_ICM20948_READ_SENSORS_PRIORITY 4U

/* ----------------------------------------------------------------
 * Hardware pins
 * ---------------------------------------------------------------- */


/**
 * @brief Chip-select GPIO number.
 */
#define K_ICM20948_PIN_NUM_CS 21U


/* ----------------------------------------------------------------
 * Register banks
 * ---------------------------------------------------------------- */

/**
 * @brief ICM20948 register bank selector values.
 */
typedef enum
{
    K_ICM20948_REG_BANK_0 = 0x00U, /**< Register bank 0. */
    K_ICM20948_REG_BANK_1 = 0x10U, /**< Register bank 1. */
    K_ICM20948_REG_BANK_2 = 0x20U, /**< Register bank 2. */
    K_ICM20948_REG_BANK_3 = 0x30U  /**< Register bank 3. */
} ICM20948_RegBank_t;

/**
 * @brief Address of the bank selection register.
 */
#define K_ICM20948_REG_BANK_SEL 0x7FU

/* ----------------------------------------------------------------
 * Bank 0 register addresses
 * ---------------------------------------------------------------- */
#define K_ICM20948_REG_WHO_AM_I                 0x00U
#define K_ICM20948_REG_USER_CTRL                0x03U
#define K_ICM20948_REG_LP_CONF                  0x05U
#define K_ICM20948_REG_PWR_MGMT_1               0x06U
#define K_ICM20948_REG_PWR_MGMT_2               0x07U
#define K_ICM20948_REG_INT_ENABLE               0x10U
#define K_ICM20948_REG_INT_ENABLE_1             0x11U
#define K_ICM20948_REG_INT_ENABLE_2             0x12U
#define K_ICM20948_REG_DMP_INT_STATUS           0x18U
#define K_ICM20948_REG_INT_STATUS               0x19U
#define K_ICM20948_REG_SINGLE_FIFO_PRIORITY_SEL 0x26U
#define K_ICM20948_REG_PERIPH_SENS_DATA_00      0x3BU
#define K_ICM20948_REG_FIFO_EN_1                0x66U
#define K_ICM20948_REG_FIFO_EN_2                0x67U
#define K_ICM20948_REG_FIFO_RST                 0x68U
#define K_ICM20948_REG_FIFO_MODE                0x69U
#define K_ICM20948_REG_FIFO_COUNTH              0x70U
#define K_ICM20948_REG_FIFO_COUNTL              0x71U
#define K_ICM20948_REG_FIFO_R_W                 0x72U
#define K_ICM20948_REG_HW_FIX_DISABLE           0x75U
#define K_ICM20948_REG_FIFO_CFG                 0x76U

/* ----------------------------------------------------------------
 * Bank 1 register addresses
 * ---------------------------------------------------------------- */
#define K_ICM20948_REG_TIMEBASE_CORRECTION_PLL 0x28U

/* ----------------------------------------------------------------
 * Bank 2 register addresses
 * ---------------------------------------------------------------- */
#define K_ICM20948_REG_GYRO_SMPLRT_DIV    0x00U
#define K_ICM20948_REG_GYRO_CONFIG        0x01U
#define K_ICM20948_REG_ACCEL_SMPLRT_DIV_1 0x10U
#define K_ICM20948_REG_ACCEL_SMPLRT_DIV_2 0x11U
#define K_ICM20948_REG_ACCEL_CONFIG       0x14U
#define K_ICM20948_REG_ACCEL_CONFIG_2     0x15U
#define K_ICM20948_REG_DMP_ADDR_MSB       0x50U
#define K_ICM20948_REG_DMP_ADDR_LSB       0x51U

/* ----------------------------------------------------------------
 * Bank 3 register addresses
 * ---------------------------------------------------------------- */
#define K_ICM20948_I2C_MST_ODR_CONFIG 0x00U
#define K_ICM20948_REG_I2C_CTRL       0x01U
#define K_ICM20948_REG_SLV0_ADDR      0x03U
#define K_ICM20948_REG_SLV0_REG       0x04U
#define K_ICM20948_REG_SLV0_CTRL      0x05U
#define K_ICM20948_REG_SLV0_DO        0x06U
#define K_ICM20948_I2C_SLV1_ADDR      0x07U
#define K_ICM20948_I2C_SLV1_REG       0x08U
#define K_ICM20948_I2C_SLV1_CTRL      0x09U
#define K_ICM20948_I2C_SLV1_DO        0x0AU

/* ----------------------------------------------------------------
 * DMP memory access
 * ---------------------------------------------------------------- */
#define K_ICM20948_REG_MEM_START_ADDR 0x7CU
#define K_ICM20948_REG_MEM_R_W        0x7DU
#define K_ICM20948_REG_MEM_BANK_SEL   0x7EU
#define K_ICM20948_DMP_LOAD_START     0x0090U

/* ----------------------------------------------------------------
 * DMP memory addresses
 * ---------------------------------------------------------------- */
#define K_ICM20948_DMP_DATA_OUT_CTL1     (4U * 16U)
#define K_ICM20948_DMP_DATA_OUT_CTL2     (4U * 16U + 2U)
#define K_ICM20948_DMP_DATA_INTR_CTL     (4U * 16U + 12U)
#define K_ICM20948_DMP_MOTION_EVENT_CTL  (4U * 16U + 14U)
#define K_ICM20948_DMP_DATA_RDY_STATUS   (8U * 16U + 10U)
#define K_ICM20948_DMP_ODR_QUAT6         (10U * 16U + 12U)
#define K_ICM20948_DMP_ODR_QUAT9         (10U * 16U + 8U)
#define K_ICM20948_DMP_ODR_GYRO          (11U * 16U + 10U)
#define K_ICM20948_DMP_ODR_CPASS         (11U * 16U + 6U)
#define K_ICM20948_DMP_ODR_ACCEL         (11U * 16U + 14U)
#define K_ICM20948_DMP_CPASS_TIME_BUFFER (29U * 16U + 8U)
#define K_ICM20948_DMP_GYRO_SF           (19U * 16U)
#define K_ICM20948_DMP_ACC_SCALE         (30U * 16U)
#define K_ICM20948_DMP_FIFO_WATERMARK    (31U * 16U + 14U)
#define K_ICM20948_DMP_BAC_RATE          (48U * 16U + 10U)
#define K_ICM20948_DMP_B2S_RATE          (48U * 16U + 8U)
#define K_ICM20948_DMP_ACCEL_ONLY_GAIN   (16U * 16U + 12U)
#define K_ICM20948_DMP_ACC_SCALE2        (79U * 16U + 4U)
#define K_ICM20948_DMP_CPASS_MTX_00      (23U * 16U)
#define K_ICM20948_DMP_CPASS_MTX_01      (23U * 16U + 4U)
#define K_ICM20948_DMP_CPASS_MTX_02      (23U * 16U + 8U)
#define K_ICM20948_DMP_CPASS_MTX_10      (23U * 16U + 12U)
#define K_ICM20948_DMP_CPASS_MTX_11      (24U * 16U)
#define K_ICM20948_DMP_CPASS_MTX_12      (24U * 16U + 4U)
#define K_ICM20948_DMP_CPASS_MTX_20      (24U * 16U + 8U)
#define K_ICM20948_DMP_CPASS_MTX_21      (24U * 16U + 12U)
#define K_ICM20948_DMP_CPASS_MTX_22      (25U * 16U)
#define K_ICM20948_DMP_GYRO_FULLSCALE    (72U * 16U + 12U)
#define K_ICM20948_DMP_ACCEL_ALPHA_VAR   (91U * 16U)
#define K_ICM20948_DMP_ACCEL_A_VAR       (92U * 16U)
#define K_ICM20948_DMP_ACCEL_CAL_INIT    (94U * 16U + 4U)
#define K_ICM20948_DMP_B2S_MTX_00        (208U * 16U)
#define K_ICM20948_DMP_B2S_MTX_01        (208U * 16U + 4U)
#define K_ICM20948_DMP_B2S_MTX_02        (208U * 16U + 8U)
#define K_ICM20948_DMP_B2S_MTX_10        (208U * 16U + 12U)
#define K_ICM20948_DMP_B2S_MTX_11        (209U * 16U)
#define K_ICM20948_DMP_B2S_MTX_12        (209U * 16U + 4U)
#define K_ICM20948_DMP_B2S_MTX_20        (209U * 16U + 8U)
#define K_ICM20948_DMP_B2S_MTX_21        (209U * 16U + 12U)
#define K_ICM20948_DMP_B2S_MTX_22        (210U * 16U)

/* ----------------------------------------------------------------
 * Magnetometer (AK09916)
 * ---------------------------------------------------------------- */
#define K_ICM20948_MAG_WR_ADDR 0x0CU
#define K_ICM20948_MAG_RD_ADDR 0x8CU
#define K_ICM20948_MAG_CTRL_2  0x31U

#define K_ICM20948_REG_ACCEL_X_H   0x2DU
#define K_ICM20948_REG_ACCEL_X_L   0x2EU
#define K_ICM20948_REG_ACCEL_Y_H   0x2FU
#define K_ICM20948_REG_ACCEL_Y_L   0x30U
#define K_ICM20948_REG_ACCEL_Z_H   0x31U
#define K_ICM20948_REG_ACCEL_Z_L   0x32U
#define K_ICM20948_REG_GYRO_X_H    0x33U
#define K_ICM20948_REG_GYRO_X_L    0x34U
#define K_ICM20948_REG_GYRO_Y_H    0x35U
#define K_ICM20948_REG_GYRO_Y_L    0x36U
#define K_ICM20948_REG_GYRO_Z_H    0x37U
#define K_ICM20948_REG_GYRO_Z_L    0x38U
#define K_ICM20948_REG_MAGNETO_X_L 0x3CU
#define K_ICM20948_REG_MAGNETO_X_H 0x3DU
#define K_ICM20948_REG_MAGNETO_Y_L 0x3EU
#define K_ICM20948_REG_MAGNETO_Y_H 0x3FU
#define K_ICM20948_REG_MAGNETO_Z_L 0x40U
#define K_ICM20948_REG_MAGNETO_Z_H 0x41U

/* ----------------------------------------------------------------
 * Register unions
 * ---------------------------------------------------------------- */

/**
 * @brief Register layout for the bank selection register.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t reserved0 : 4; /**< Reserved bits [3:0]. */
        spp_uint8_t bankSel   : 2; /**< Bank selector bits [5:4]. */
        spp_uint8_t reserved1 : 2; /**< Reserved bits [7:6]. */
    } bits;
} ICM20948_RegBankSel_t;

/**
 * @brief Register layout for USER_CTRL.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t reserved0 : 1; /**< Reserved bit [0]. */
        spp_uint8_t i2cMstRst : 1; /**< Reset I2C master. */
        spp_uint8_t sramRst   : 1; /**< Reset SRAM/DMP interface. */
        spp_uint8_t dmpRst    : 1; /**< Reset DMP. */
        spp_uint8_t i2cIfDis  : 1; /**< Disable primary I2C interface. */
        spp_uint8_t i2cMstEn  : 1; /**< Enable internal I2C master. */
        spp_uint8_t fifoEn    : 1; /**< Enable FIFO. */
        spp_uint8_t dmpEn     : 1; /**< Enable DMP. */
    } bits;
} ICM20948_RegUserCtrl_t;

/**
 * @brief Register layout for LP_CONF.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t reserved0 : 4; /**< Reserved bits [3:0]. */
        spp_uint8_t gyroCyc   : 1; /**< Duty-cycle gyroscope (bit 4). */
        spp_uint8_t accelCyc  : 1; /**< Duty-cycle accelerometer (bit 5). */
        spp_uint8_t i2cMstCyc : 1; /**< Duty-cycle I2C master (bit 6). */
        spp_uint8_t reserved1 : 1; /**< Reserved bit [7]. */
    } bits;
} ICM20948_RegLpConf_t;

/**
 * @brief Clock source values for PWR_MGMT_1.
 */
typedef enum
{
    K_ICM20948_CLK_INTERNAL_20MHZ = 0U, /**< Internal oscillator. */
    K_ICM20948_CLK_AUTO = 1U,           /**< Automatically select best clock. */
    K_ICM20948_CLK_STOP = 7U            /**< Stops the clock source. */
} ICM20948_ClockSel_t;

/**
 * @brief Register layout for PWR_MGMT_1.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t clkSel      : 3; /**< Clock source selection. */
        spp_uint8_t tempDis     : 1; /**< Disable temperature sensor. */
        spp_uint8_t reserved0   : 1; /**< Reserved bit [4]. */
        spp_uint8_t lpEn        : 1; /**< Low-power enable. */
        spp_uint8_t sleep       : 1; /**< Sleep mode enable. */
        spp_uint8_t deviceReset : 1; /**< Device reset. */
    } bits;
} ICM20948_RegPwrMgmt1_t;

/**
 * @brief Register layout for PWR_MGMT_2.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t disableAccelX : 1; /**< Disable accelerometer X axis. */
        spp_uint8_t disableAccelY : 1; /**< Disable accelerometer Y axis. */
        spp_uint8_t disableAccelZ : 1; /**< Disable accelerometer Z axis. */
        spp_uint8_t disableGyroX  : 1; /**< Disable gyroscope X axis. */
        spp_uint8_t disableGyroY  : 1; /**< Disable gyroscope Y axis. */
        spp_uint8_t disableGyroZ  : 1; /**< Disable gyroscope Z axis. */
        spp_uint8_t reserved0     : 2; /**< Reserved bits [7:6]. */
    } bits;
} ICM20948_RegPwrMgmt2_t;

/**
 * @brief Register layout for INT_ENABLE.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t reserved0   : 1; /**< Reserved bit [0]. */
        spp_uint8_t rawData0Rdy : 1; /**< RAW_DATA_0_RDY interrupt enable. */
        spp_uint8_t reserved1   : 6; /**< Reserved bits [7:2]. */
    } bits;
} ICM20948_RegIntEnable_t;

/**
 * @brief Register layout for INT_ENABLE_2.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t fifoOverflowEn0 : 1; /**< FIFO overflow interrupt enable 0. */
        spp_uint8_t fifoOverflowEn1 : 1; /**< FIFO overflow interrupt enable 1. */
        spp_uint8_t fifoOverflowEn2 : 1; /**< FIFO overflow interrupt enable 2. */
        spp_uint8_t fifoOverflowEn3 : 1; /**< FIFO overflow interrupt enable 3. */
        spp_uint8_t reserved0       : 3; /**< Reserved bits [6:4]. */
        spp_uint8_t dmpInt1En       : 1; /**< DMP interrupt enable. */
    } bits;
} ICM20948_RegIntEnable2_t;

/**
 * @brief Register layout for INT_STATUS.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t reserved0   : 1; /**< Reserved bit [0]. */
        spp_uint8_t rawData0Rdy : 1; /**< RAW_DATA_0_RDY interrupt status. */
        spp_uint8_t reserved1   : 6; /**< Reserved bits [7:2]. */
    } bits;
} ICM20948_RegIntStatus_t;

/**
 * @brief Register layout for DMP_INT_STATUS.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t dmpInt0   : 1; /**< DMP interrupt status bit 0. */
        spp_uint8_t dmpInt1   : 1; /**< DMP interrupt status bit 1. */
        spp_uint8_t dmpInt2   : 1; /**< DMP interrupt status bit 2. */
        spp_uint8_t dmpInt3   : 1; /**< DMP interrupt status bit 3. */
        spp_uint8_t dmpInt4   : 1; /**< DMP interrupt status bit 4. */
        spp_uint8_t reserved0 : 3; /**< Reserved bits [7:5]. */
    } bits;
} ICM20948_RegDmpIntStatus_t;

/**
 * @brief Register layout for SINGLE_FIFO_PRIORITY_SEL.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t accelFifoPri : 2; /**< Accelerometer FIFO priority. */
        spp_uint8_t gyroFifoPri  : 2; /**< Gyroscope FIFO priority. */
        spp_uint8_t dmpFifoPri   : 2; /**< DMP FIFO priority. */
        spp_uint8_t reserved0    : 2; /**< Reserved bits [7:6]. */
    } bits;
} ICM20948_RegSingleFifoPrioritySel_t;

/**
 * @brief Register layout for FIFO_EN_1.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t slv0FifoEn : 1; /**< Enable SLV0 FIFO. */
        spp_uint8_t slv1FifoEn : 1; /**< Enable SLV1 FIFO. */
        spp_uint8_t slv2FifoEn : 1; /**< Enable SLV2 FIFO. */
        spp_uint8_t slv3FifoEn : 1; /**< Enable SLV3 FIFO. */
        spp_uint8_t reserved0  : 4; /**< Reserved bits [7:4]. */
    } bits;
} ICM20948_RegFifoEn1_t;

/**
 * @brief Register layout for FIFO_EN_2.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t accelFifoEn : 1; /**< Enable accelerometer FIFO path. */
        spp_uint8_t gyroXFifoEn : 1; /**< Enable gyroscope X FIFO path. */
        spp_uint8_t gyroYFifoEn : 1; /**< Enable gyroscope Y FIFO path. */
        spp_uint8_t gyroZFifoEn : 1; /**< Enable gyroscope Z FIFO path. */
        spp_uint8_t tempFifoEn  : 1; /**< Enable temperature FIFO path. */
        spp_uint8_t reserved0   : 3; /**< Reserved bits [7:5]. */
    } bits;
} ICM20948_RegFifoEn2_t;

/**
 * @brief Register layout for FIFO_RST.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t fifoRst0  : 1; /**< Reset FIFO channel 0. */
        spp_uint8_t fifoRst1  : 1; /**< Reset FIFO channel 1. */
        spp_uint8_t fifoRst2  : 1; /**< Reset FIFO channel 2. */
        spp_uint8_t fifoRst3  : 1; /**< Reset FIFO channel 3. */
        spp_uint8_t fifoRst4  : 1; /**< Reset FIFO channel 4. */
        spp_uint8_t reserved0 : 3; /**< Reserved bits [7:5]. */
    } bits;
} ICM20948_RegFifoRst_t;

/**
 * @brief Register layout for FIFO_MODE.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t fifoMode  : 1; /**< FIFO mode bit. */
        spp_uint8_t reserved0 : 7; /**< Reserved bits [7:1]. */
    } bits;
} ICM20948_RegFifoMode_t;

/**
 * @brief Register layout for FIFO_CFG.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t fifoCfg   : 1; /**< FIFO configuration bit. */
        spp_uint8_t reserved0 : 7; /**< Reserved bits [7:1]. */
    } bits;
} ICM20948_RegFifoCfg_t;

/**
 * @brief Register layout for HW_FIX_DISABLE.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t bit0 : 1; /**< Bit 0. */
        spp_uint8_t bit1 : 1; /**< Bit 1. */
        spp_uint8_t bit2 : 1; /**< Bit 2. */
        spp_uint8_t bit3 : 1; /**< Bit 3. */
        spp_uint8_t bit4 : 1; /**< Bit 4. */
        spp_uint8_t bit5 : 1; /**< Bit 5. */
        spp_uint8_t bit6 : 1; /**< Bit 6. */
        spp_uint8_t bit7 : 1; /**< Bit 7. */
    } bits;
} ICM20948_RegHwFixDisable_t;

/**
 * @brief Gyroscope full-scale values.
 */
typedef enum
{
    K_ICM20948_GYRO_FS_250DPS = 0U,  /**< ±250 dps. */
    K_ICM20948_GYRO_FS_500DPS = 1U,  /**< ±500 dps. */
    K_ICM20948_GYRO_FS_1000DPS = 2U, /**< ±1000 dps. */
    K_ICM20948_GYRO_FS_2000DPS = 3U  /**< ±2000 dps. */
} ICM20948_GyroFsSel_t;

/**
 * @brief Register layout for GYRO_CONFIG.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t gyroFchoice : 1; /**< Gyro DLPF bypass/choice. */
        spp_uint8_t gyroFsSel   : 2; /**< Gyroscope full-scale selection. */
        spp_uint8_t gyroDlpfCfg : 3; /**< Gyroscope DLPF configuration. */
        spp_uint8_t reserved0   : 2; /**< Reserved bits [7:6]. */
    } bits;
} ICM20948_RegGyroConfig_t;

/**
 * @brief Accelerometer full-scale values.
 */
typedef enum
{
    K_ICM20948_ACCEL_FS_2G = 0U, /**< ±2 g. */
    K_ICM20948_ACCEL_FS_4G = 1U, /**< ±4 g. */
    K_ICM20948_ACCEL_FS_8G = 2U, /**< ±8 g. */
    K_ICM20948_ACCEL_FS_16G = 3U /**< ±16 g. */
} ICM20948_AccelFsSel_t;

/**
 * @brief Register layout for ACCEL_CONFIG.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t accelFchoice : 1; /**< Accel DLPF bypass/choice. */
        spp_uint8_t accelFsSel   : 2; /**< Accelerometer full-scale selection. */
        spp_uint8_t accelDlpfCfg : 3; /**< Accelerometer DLPF configuration. */
        spp_uint8_t reserved0    : 2; /**< Reserved bits [7:6]. */
    } bits;
} ICM20948_RegAccelConfig_t;

/**
 * @brief Register layout for ACCEL_CONFIG_2.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t dec3Cfg   : 2; /**< Accelerometer averaging/decimation config. */
        spp_uint8_t reserved0 : 6; /**< Reserved bits [7:2]. */
    } bits;
} ICM20948_RegAccelConfig2_t;

/**
 * @brief Register layout for I2C_CTRL.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t i2cMstClk : 4; /**< I2C master clock selection. */
        spp_uint8_t i2cMstPnt : 1; /**< I2C master restart control. */
        spp_uint8_t reserved0 : 3; /**< Reserved bits [7:5]. */
    } bits;
} ICM20948_RegI2cCtrl_t;

/**
 * @brief Register layout for slave address registers.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t i2cId        : 7; /**< 7-bit I2C slave address. */
        spp_uint8_t readNotWrite : 1; /**< 1 for read, 0 for write. */
    } bits;
} ICM20948_RegI2cSlvAddr_t;

/**
 * @brief Register layout for slave control registers.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t length   : 4; /**< Number of bytes to transfer. */
        spp_uint8_t group    : 1; /**< Grouping control. */
        spp_uint8_t regDis   : 1; /**< Disable register address phase. */
        spp_uint8_t byteSwap : 1; /**< Byte swap enable. */
        spp_uint8_t enable   : 1; /**< Enable slave channel. */
    } bits;
} ICM20948_RegI2cSlvCtrl_t;

/**
 * @brief Register layout for I2C_MST_ODR_CONFIG.
 */
typedef union
{
    spp_uint8_t value; /**< Full register value. */
    struct
    {
        spp_uint8_t i2cMstOdr : 4; /**< I2C master ODR divider selector. */
        spp_uint8_t reserved0 : 4; /**< Reserved bits [7:4]. */
    } bits;
} ICM20948_RegI2cMstOdrConfig_t;


/* ----------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------- */

/**
 * @brief Performs the generic device configuration sequence.
 *
 * @param[in,out] p_data Pointer to the driver context.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
retval_t ICM20948_config(void *p_data);

/**
 * @brief Performs the DMP configuration sequence.
 *
 * @param[in,out] p_data Pointer to the driver context.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
retval_t ICM20948_configDmp(void *p_data);

/**
 * @brief Performs the full DMP initialization sequence.
 *
 * @param[in,out] p_data Pointer to the driver context.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
retval_t ICM20948_configDmpInit(void *p_data);

/**
 * @brief Loads the DMP firmware image into device memory.
 *
 * @param[in,out] p_data Pointer to the driver context.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
retval_t ICM20948_loadDmp(void *p_data);

/**
 * @brief Starts DMP execution.
 *
 * @param[in,out] p_data Pointer to the driver context.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
retval_t ICM20948_dmpStart(void *p_data);

/**
 * @brief Reads sensor data from the device.
 *
 * @param[in,out] p_data Pointer to the driver context.
 *
 * @return SPP_OK on success, or an error code otherwise.
 */
retval_t ICM20948_readSensors(void *p_data);

/**
 * @brief Retrieves and processes sensor data from the device.
 *
 * @param[in,out] p_data Pointer to the driver context.
 */
void ICM20948_getSensorsData(void *p_data);

/**
 * @brief Checks the FIFO and parses available DMP packets.
 *
 * @param[in,out] p_data Pointer to the driver context.
 */
void ICM20948_checkFifoData(void *p_data);

#ifdef __cplusplus
}
#endif

#endif /* ICM20948_H */