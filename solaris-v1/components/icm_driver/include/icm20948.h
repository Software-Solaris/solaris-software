#ifndef ICM20948_H
#define ICM20948_H

#include "returntypes.h"
#include "general.h"
#include "types.h"

#define ICM_TASK_PRIORITY            4
#define ICM_CONFIG_TASK_PRIORITY     5
#define ICM_READ_SENSORS_PRIORITY    4

/* ----------------------------------------------------------------
 * Hardware pins
 * ---------------------------------------------------------------- */
#define SPI_HOST_USED   SPI2_HOST
#define PIN_NUM_CS      21
#define PIN_NUM_CIPO    47
#define PIN_NUM_COPI    38
#define PIN_NUM_CLK     48

/* ----------------------------------------------------------------
 * SPI operation type (bit 7)
 * ---------------------------------------------------------------- */
#define READ_OP     0x80
#define WRITE_OP    0x00
#define EMPTY_MESSAGE 0x00

/* ----------------------------------------------------------------
 * Bank select
 * ---------------------------------------------------------------- */
#define REG_BANK_SEL    0x7F
#define REG_BANK_0      0x00
#define REG_BANK_1      0x10
#define REG_BANK_2      0x20
#define REG_BANK_3      0x30

/* ----------------------------------------------------------------
 * Bank 0 registers
 * ---------------------------------------------------------------- */
#define REG_WHO_AM_I                0x00
#define REG_USER_CTRL               0x03
#define REG_LP_CONF                 0x05
#define REG_PWR_MGMT_1              0x06
#define REG_PWR_MGMT_2              0x07
#define REG_INT_ENABLE              0x10
#define REG_INT_ENABLE_1            0x11
#define REG_INT_ENABLE_2            0x12
#define REG_DMP_INT_STATUS          0x18
#define REG_INT_STATUS              0x19
#define REG_SINGLE_FIFO_PRIORITY_SEL 0x26
#define REG_PERIPH_SENS_DATA_00     0x3B
#define REG_FIFO_EN_1               0x66
#define REG_FIFO_EN_2               0x67
#define REG_FIFO_RST                0x68
#define REG_FIFO_MODE               0x69
#define REG_FIFO_COUNTH             0x70
#define REG_FIFO_COUNTL             0x71
#define REG_FIFO_R_W                0x72
#define REG_HW_FIX_DISABLE          0x75
#define REG_FIFO_CFG                0x76

/* ----------------------------------------------------------------
 * Bank 1 registers
 * ---------------------------------------------------------------- */
#define REG_TIMEBASE_CORRECTION_PLL 0x28

/* ----------------------------------------------------------------
 * Bank 2 registers
 * ---------------------------------------------------------------- */
#define REG_GYRO_CONFIG             0x01
#define REG_GYRO_SMPLRT_DIV         0x00
#define REG_ACCEL_CONFIG            0x14
#define REG_ACCEL_CONFIG_2          0x15
#define REG_ACCEL_SMPLRT_DIV_1      0x10
#define REG_ACCEL_SMPLRT_DIV_2      0x11
#define REG_DMP_ADDR_MSB            0x50
#define REG_DMP_ADDR_LSB            0x51

/* ----------------------------------------------------------------
 * Bank 3 - I2C master
 * ---------------------------------------------------------------- */
#define I2C_MST_ODR_CONFIG          0x00
#define REG_I2C_CTRL                0x01
#define REG_SLV0_ADDR               0x03
#define REG_SLV0_REG                0x04
#define REG_SLV0_CTRL               0x05
#define REG_SLV0_DO                 0x06
#define I2C_SLV1_ADDR               0x07
#define I2C_SLV1_REG                0x08
#define I2C_SLV1_CTRL               0x09
#define I2C_SLV1_DO                 0x0A

/* ----------------------------------------------------------------
 * DMP memory access (Bank 0: MEM_BANK_SEL + MEM_START_ADDR + MEM_R_W)
 * ---------------------------------------------------------------- */
#define REG_MEM_BANK_SEL            0x7E
#define REG_MEM_START_ADDR          0x7C
#define REG_MEM_R_W                 0x7D
#define DMP_LOAD_START              0x0090

/* ----------------------------------------------------------------
 * DMP memory addresses
 * ---------------------------------------------------------------- */
#define DMP_DATA_OUT_CTL1           (4  * 16)
#define DMP_DATA_OUT_CTL2           (4  * 16 + 2)
#define DMP_DATA_INTR_CTL           (4  * 16 + 12)
#define DMP_MOTION_EVENT_CTL        (4  * 16 + 14)
#define DMP_DATA_RDY_STATUS         (8  * 16 + 10)
#define DMP_ODR_QUAT6               (10 * 16 + 12)
#define DMP_GYRO_SF                 (19 * 16)
#define DMP_ACC_SCALE               (30 * 16)
#define DMP_FIFO_WATERMARK          (31 * 16 + 14)
#define DMP_BAC_RATE                (48 * 16 + 10)
#define DMP_B2S_RATE                (48 * 16 + 8)
#define DMP_ACCEL_ONLY_GAIN         (16 * 16 + 12)
#define DMP_ACC_SCALE2              (79 * 16 + 4)
#define DMP_CPASS_MTX_00            (23 * 16)
#define DMP_CPASS_MTX_01            (23 * 16 + 4)
#define DMP_CPASS_MTX_02            (23 * 16 + 8)
#define DMP_CPASS_MTX_10            (23 * 16 + 12)
#define DMP_CPASS_MTX_11            (24 * 16)
#define DMP_CPASS_MTX_12            (24 * 16 + 4)
#define DMP_CPASS_MTX_20            (24 * 16 + 8)
#define DMP_CPASS_MTX_21            (24 * 16 + 12)
#define DMP_CPASS_MTX_22            (25 * 16)
#define DMP_GYRO_FULLSCALE          (72 * 16 + 12)
#define DMP_ACCEL_ALPHA_VAR         (91 * 16)
#define DMP_ACCEL_A_VAR             (92 * 16)
#define DMP_ACCEL_CAL_INIT          (94 * 16 + 4)
#define DMP_B2S_MTX_00              (208 * 16)
#define DMP_B2S_MTX_01              (208 * 16 + 4)
#define DMP_B2S_MTX_02              (208 * 16 + 8)
#define DMP_B2S_MTX_10              (208 * 16 + 12)
#define DMP_B2S_MTX_11              (209 * 16)
#define DMP_B2S_MTX_12              (209 * 16 + 4)
#define DMP_B2S_MTX_20              (209 * 16 + 8)
#define DMP_B2S_MTX_21              (209 * 16 + 12)
#define DMP_B2S_MTX_22              (210 * 16)

/* ----------------------------------------------------------------
 * Magnetometer (AK09916)
 * ---------------------------------------------------------------- */
#define MAGNETO_WR_ADDR     0x0C
#define MAGNETO_RD_ADDR     0x8C
#define MAGNETO_CTRL_2      0x31
#define REG_ACCEL_X_H       0x2D
#define REG_ACCEL_X_L       0x2E
#define REG_ACCEL_Y_H       0x2F
#define REG_ACCEL_Y_L       0x30
#define REG_ACCEL_Z_H       0x31
#define REG_ACCEL_Z_L       0x32
#define REG_GYRO_X_H        0x33
#define REG_GYRO_X_L        0x34
#define REG_GYRO_Y_H        0x35
#define REG_GYRO_Y_L        0x36
#define REG_GYRO_Z_H        0x37
#define REG_GYRO_Z_L        0x38
#define REG_MAGNETO_X_L     0x3C
#define REG_MAGNETO_X_H     0x3D
#define REG_MAGNETO_Y_L     0x3E
#define REG_MAGNETO_Y_H     0x3F
#define REG_MAGNETO_Z_L     0x40
#define REG_MAGNETO_Z_H     0x41

typedef struct {
    void       *p_handler_spi;
    spp_bool_t  firmware_loaded;
} icm_data_t;

retval_t IcmInit(void *p_data);
retval_t IcmConfig(void *p_data);
retval_t IcmConfigDmp(void *p_data);
retval_t IcmConfigDmpInit(void *p_data);
retval_t IcmLoadDmp(void *p_data);
retval_t IcmDmpStart(void *p_data);
retval_t IcmReadSensors(void *p_data);
void     IcmGetSensorsData(void *p_data);

#endif