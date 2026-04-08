/*
 * test_fifo_data_parsing.c
 *
 * BDD suite — FIFO Data Parsing and Sensor Unit Conversion
 *
 * Feature: The DMP writes 42-byte packets into the ICM20948 FIFO.  Each
 *          packet contains raw sensor data that must be converted to physical
 *          units before use:
 *
 *            Accelerometer : raw (int16) ÷ 8192.0  → g    (±4 g full-scale)
 *            Gyroscope     : raw (int16) ÷ 16.4    → °/s  (±2000 dps FS)
 *            Magnetometer  : raw (int16) × 0.15    → µT   (AK09916 sensitivity)
 *            Quaternion qw : sqrt(1 − qx²−qy²−qz²), clamped to 0 if negative
 *
 *          The converted values are emitted via SPP_LOGI; the test stub
 *          captures each log call in g_logEntries[].
 *
 *          Overflow protection: if the FIFO byte count exceeds 512, the FIFO
 *          is reset and no packet data is parsed.
 *
 * Function under test: ICM20948_checkFifoData
 */

#include "mock_common.h"
#include <string.h>
#include "icm20948.c"

/* ----------------------------------------------------------------
 * Helper: build a 42-byte DMP FIFO packet
 *
 * The 42-byte packet layout (FIFO content, no SPI command byte):
 *   [0-1]   header
 *   [2-7]   accel X/Y/Z  (int16, big-endian)
 *   [8-13]  gyro  X/Y/Z  (int16, big-endian)
 *   [14-19] reserved
 *   [20-25] mag   X/Y/Z  (int16, big-endian)
 *   [26-29] quat q1/X    (int32, big-endian)
 *   [30-33] quat q2/Y
 *   [34-37] quat q3/Z
 *   [38-39] accuracy
 *   [40-41] footer
 * ---------------------------------------------------------------- */
static void build_dmp_packet(spp_uint8_t *buf,
                             int16_t ax,  int16_t ay,  int16_t az,
                             int16_t gx,  int16_t gy,  int16_t gz,
                             int16_t mx,  int16_t my,  int16_t mz,
                             int32_t q1,  int32_t q2,  int32_t q3)
{
    memset(buf, 0, 42U);

    buf[2]  = (spp_uint8_t)(ax >> 8); buf[3]  = (spp_uint8_t)ax;
    buf[4]  = (spp_uint8_t)(ay >> 8); buf[5]  = (spp_uint8_t)ay;
    buf[6]  = (spp_uint8_t)(az >> 8); buf[7]  = (spp_uint8_t)az;

    buf[8]  = (spp_uint8_t)(gx >> 8); buf[9]  = (spp_uint8_t)gx;
    buf[10] = (spp_uint8_t)(gy >> 8); buf[11] = (spp_uint8_t)gy;
    buf[12] = (spp_uint8_t)(gz >> 8); buf[13] = (spp_uint8_t)gz;

    buf[20] = (spp_uint8_t)(mx >> 8); buf[21] = (spp_uint8_t)mx;
    buf[22] = (spp_uint8_t)(my >> 8); buf[23] = (spp_uint8_t)my;
    buf[24] = (spp_uint8_t)(mz >> 8); buf[25] = (spp_uint8_t)mz;

    buf[26] = (spp_uint8_t)((spp_uint32_t)q1 >> 24);
    buf[27] = (spp_uint8_t)((spp_uint32_t)q1 >> 16);
    buf[28] = (spp_uint8_t)((spp_uint32_t)q1 >> 8);
    buf[29] = (spp_uint8_t)(spp_uint32_t)q1;

    buf[30] = (spp_uint8_t)((spp_uint32_t)q2 >> 24);
    buf[31] = (spp_uint8_t)((spp_uint32_t)q2 >> 16);
    buf[32] = (spp_uint8_t)((spp_uint32_t)q2 >> 8);
    buf[33] = (spp_uint8_t)(spp_uint32_t)q2;

    buf[34] = (spp_uint8_t)((spp_uint32_t)q3 >> 24);
    buf[35] = (spp_uint8_t)((spp_uint32_t)q3 >> 16);
    buf[36] = (spp_uint8_t)((spp_uint32_t)q3 >> 8);
    buf[37] = (spp_uint8_t)(spp_uint32_t)q3;
}

/* ================================================================
 * Tests
 * ================================================================ */

Describe(CheckFifoData);
BeforeEach(CheckFifoData) { mock_reset(); }
AfterEach(CheckFifoData)  {}

/*
 * GIVEN a null SPI handle
 * WHEN  checkFifoData is called
 * THEN  it returns immediately without issuing any SPI transaction
 */
Ensure(CheckFifoData, it_does_nothing_when_the_spi_handle_is_null)
{
    ICM20948_checkFifoData(NULL);

    assert_that(s_spiCallCount, is_equal_to(0));
}

/*
 * GIVEN INT_STATUS (reg 0x19) has bit 1 cleared (no raw-data-ready interrupt)
 * WHEN  checkFifoData is called
 * THEN  it reads the interrupt register and DMP status but does not read FIFO data
 */
Ensure(CheckFifoData, it_does_not_access_the_fifo_when_the_interrupt_flag_is_not_set)
{
    s_regs[0][K_ICM20948_REG_INT_STATUS] = 0x00U;  /* interrupt bit = 0 */

    ICM20948_checkFifoData(p_testSpi);

    /* Only INT_STATUS + DMP_INT_STATUS reads: 2 SPI calls; FIFO_COUNT not read */
    assert_that(s_spiCallCount, is_equal_to(2));
}

/*
 * GIVEN an interrupt is pending and the FIFO byte count = 640 (> 512 threshold)
 * WHEN  checkFifoData is called
 * THEN  it resets the FIFO (2 SPI writes to FIFO_RST) and returns without parsing
 */
Ensure(CheckFifoData, it_resets_the_fifo_when_the_count_exceeds_the_overflow_threshold)
{
    s_regs[0][K_ICM20948_REG_INT_STATUS]  = 0x02U;
    s_regs[0][K_ICM20948_REG_FIFO_COUNTH] = 0x02U;
    s_regs[0][K_ICM20948_REG_FIFO_COUNTL] = 0x80U;  /* 640 bytes */

    ICM20948_checkFifoData(p_testSpi);

    /* FIFO_RST should have been written twice (assert + deassert) */
    assert_that(s_regs[0][K_ICM20948_REG_FIFO_RST], is_not_equal_to(0x00));
}

/*
 * GIVEN an interrupt is pending and the FIFO byte count = 0
 * WHEN  checkFifoData is called
 * THEN  no packet data is read (numPackets = 0)
 */
Ensure(CheckFifoData, it_reads_no_packets_when_the_fifo_count_is_zero)
{
    s_regs[0][K_ICM20948_REG_INT_STATUS]  = 0x02U;
    s_regs[0][K_ICM20948_REG_FIFO_COUNTH] = 0x00U;
    s_regs[0][K_ICM20948_REG_FIFO_COUNTL] = 0x00U;

    ICM20948_checkFifoData(p_testSpi);

    assert_that(g_logCallCount, is_equal_to(0));
}

/*
 * GIVEN accel X = 8192 (raw int16) in the FIFO packet
 *       at ±4 g full-scale, 8192 LSB/g
 * WHEN  checkFifoData parses the packet
 * THEN  the log entry contains "1.00" for the X acceleration in g
 */
Ensure(CheckFifoData, it_converts_raw_accelerometer_data_to_g_units_at_4g_full_scale)
{
    spp_uint8_t pkt[42];
    build_dmp_packet(pkt, 8192, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    s_regs[0][K_ICM20948_REG_INT_STATUS] = 0x02U;
    mock_set_fifo(pkt, 42);

    ICM20948_checkFifoData(p_testSpi);

    /* g_logEntries[0] contains the accel/gyro/mag line */
    assert_that(strstr(g_logEntries[0], "1.00"), is_not_null);
}

/*
 * GIVEN gyro X = 164 (raw int16) in the FIFO packet
 *       at ±2000 dps full-scale, 16.4 LSB/(°/s) → 164/16.4 = 10.0 °/s
 * WHEN  checkFifoData parses the packet
 * THEN  the log entry contains "10.0" for the X gyroscope rate in dps
 */
Ensure(CheckFifoData, it_converts_raw_gyroscope_data_to_dps_at_2000dps_full_scale)
{
    spp_uint8_t pkt[42];
    build_dmp_packet(pkt, 0, 0, 0, 164, 0, 0, 0, 0, 0, 0, 0, 0);

    s_regs[0][K_ICM20948_REG_INT_STATUS] = 0x02U;
    mock_set_fifo(pkt, 42);

    ICM20948_checkFifoData(p_testSpi);

    assert_that(strstr(g_logEntries[0], "10.0"), is_not_null);
}

/*
 * GIVEN mag X = 100 (raw int16) in the FIFO packet
 *       AK09916 sensitivity = 0.15 µT/LSB → 100 × 0.15 = 15.0 µT
 * WHEN  checkFifoData parses the packet
 * THEN  the log entry contains "15.0" for the X magnetometer reading in µT
 */
Ensure(CheckFifoData, it_converts_raw_magnetometer_data_to_microtesla)
{
    spp_uint8_t pkt[42];
    build_dmp_packet(pkt, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0);

    s_regs[0][K_ICM20948_REG_INT_STATUS] = 0x02U;
    mock_set_fifo(pkt, 42);

    ICM20948_checkFifoData(p_testSpi);

    assert_that(strstr(g_logEntries[0], "15.0"), is_not_null);
}

/*
 * GIVEN q1=q2=q3 = 0 (all quaternion vector components zero)
 * WHEN  checkFifoData parses the packet
 * THEN  the reconstructed qw = sqrt(1 − 0 − 0 − 0) = 1.0000
 */
Ensure(CheckFifoData, it_reconstructs_quaternion_w_as_1_when_xyz_are_all_zero)
{
    spp_uint8_t pkt[42];
    build_dmp_packet(pkt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    s_regs[0][K_ICM20948_REG_INT_STATUS] = 0x02U;
    mock_set_fifo(pkt, 42);

    ICM20948_checkFifoData(p_testSpi);

    /* g_logEntries[1] contains the quaternion line */
    assert_that(strstr(g_logEntries[1], "w=1.0000"), is_not_null);
}

/*
 * GIVEN q1=q2=q3 = 0x40000000 (each equals 1.0 in Q30)
 *       qx²+qy²+qz² = 3 > 1  →  qwSquared negative
 * WHEN  checkFifoData parses the packet
 * THEN  qw is clamped to 0.0000 (not NaN or negative)
 */
Ensure(CheckFifoData, it_clamps_quaternion_w_to_zero_when_xyz_squared_sum_exceeds_unity)
{
    spp_uint8_t pkt[42];
    /* 0x40000000 = 1073741824 = 1.0 in Q30 */
    build_dmp_packet(pkt, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                     0x40000000, 0x40000000, 0x40000000);

    s_regs[0][K_ICM20948_REG_INT_STATUS] = 0x02U;
    mock_set_fifo(pkt, 42);

    ICM20948_checkFifoData(p_testSpi);

    assert_that(strstr(g_logEntries[1], "w=0.0000"), is_not_null);
}

/*
 * GIVEN two complete 42-byte packets in the FIFO
 * WHEN  checkFifoData is called
 * THEN  both packets are parsed (two log pairs produced)
 */
Ensure(CheckFifoData, it_processes_all_complete_packets_present_in_the_fifo)
{
    spp_uint8_t buf[84];
    build_dmp_packet(&buf[0],  8192, 0, 0, 164, 0, 0, 100, 0, 0, 0, 0, 0);
    build_dmp_packet(&buf[42], 0, 8192, 0, 0, 164, 0, 0, 100, 0, 0, 0, 0);

    s_regs[0][K_ICM20948_REG_INT_STATUS] = 0x02U;
    mock_set_fifo(buf, 84);

    ICM20948_checkFifoData(p_testSpi);

    /* Two packets → two SPP_LOGI pairs → 4 log entries */
    assert_that(g_logCallCount, is_equal_to(4));
}

/*
 * GIVEN the SPI bus fails when reading INT_STATUS (first call)
 * WHEN  checkFifoData is called
 * THEN  it returns gracefully without further SPI access or crash
 */
Ensure(CheckFifoData, it_recovers_gracefully_when_the_interrupt_status_read_fails)
{
    s_spiFailAtCall = 1;

    ICM20948_checkFifoData(p_testSpi);

    assert_that(s_spiCallCount, is_equal_to(1));
}

/*
 * GIVEN an interrupt is pending but the SPI bus fails on the FIFO count read
 * WHEN  checkFifoData is called
 * THEN  it returns gracefully without parsing any packets
 */
Ensure(CheckFifoData, it_recovers_gracefully_when_the_fifo_count_read_fails)
{
    s_regs[0][K_ICM20948_REG_INT_STATUS] = 0x02U;
    s_spiFailAtCall = 3;  /* call 1=INT_STATUS, 2=DMP_INT_STATUS, 3=FIFO_COUNT */

    ICM20948_checkFifoData(p_testSpi);

    assert_that(g_logCallCount, is_equal_to(0));
}

/*
 * GIVEN a valid 42-byte packet in FIFO but SPI fails on the FIFO_R_W read
 * WHEN  checkFifoData is called
 * THEN  it returns gracefully without logging any converted values
 */
Ensure(CheckFifoData, it_recovers_gracefully_when_the_fifo_packet_read_fails)
{
    spp_uint8_t pkt[42];
    build_dmp_packet(pkt, 100, 200, 300, 10, 20, 30, 1, 2, 3, 0, 0, 0);

    s_regs[0][K_ICM20948_REG_INT_STATUS] = 0x02U;
    mock_set_fifo(pkt, 42);
    s_spiFailAtCall = 4;  /* 1=INT_STATUS, 2=DMP_INT, 3=FIFO_COUNT, 4=FIFO_R_W */

    ICM20948_checkFifoData(p_testSpi);

    assert_that(g_logCallCount, is_equal_to(0));
}

/* ================================================================
 * Test runner
 * ================================================================ */

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    TestSuite *suite = create_test_suite();

    add_test_with_context(suite, CheckFifoData, it_does_nothing_when_the_spi_handle_is_null);
    add_test_with_context(suite, CheckFifoData, it_does_not_access_the_fifo_when_the_interrupt_flag_is_not_set);
    add_test_with_context(suite, CheckFifoData, it_resets_the_fifo_when_the_count_exceeds_the_overflow_threshold);
    add_test_with_context(suite, CheckFifoData, it_reads_no_packets_when_the_fifo_count_is_zero);
    add_test_with_context(suite, CheckFifoData, it_converts_raw_accelerometer_data_to_g_units_at_4g_full_scale);
    add_test_with_context(suite, CheckFifoData, it_converts_raw_gyroscope_data_to_dps_at_2000dps_full_scale);
    add_test_with_context(suite, CheckFifoData, it_converts_raw_magnetometer_data_to_microtesla);
    add_test_with_context(suite, CheckFifoData, it_reconstructs_quaternion_w_as_1_when_xyz_are_all_zero);
    add_test_with_context(suite, CheckFifoData, it_clamps_quaternion_w_to_zero_when_xyz_squared_sum_exceeds_unity);
    add_test_with_context(suite, CheckFifoData, it_processes_all_complete_packets_present_in_the_fifo);
    add_test_with_context(suite, CheckFifoData, it_recovers_gracefully_when_the_interrupt_status_read_fails);
    add_test_with_context(suite, CheckFifoData, it_recovers_gracefully_when_the_fifo_count_read_fails);
    add_test_with_context(suite, CheckFifoData, it_recovers_gracefully_when_the_fifo_packet_read_fails);

    return run_test_suite(suite, create_text_reporter());
}
