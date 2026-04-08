/*
 * test_dmp_output_config.c
 *
 * BDD suite — DMP Output Configuration
 *
 * Feature: The DMP is told which sensor channels to include in each FIFO
 *          packet and which ones should raise interrupts via four DMP SRAM
 *          registers:
 *            DATA_OUT_CTL1 (addr 64)  — primary output enable bitmask
 *            DATA_OUT_CTL2 (addr 66)  — always zero (reserved)
 *            DATA_INTR_CTL (addr 76)  — mirrors CTL1 so interrupts fire for
 *                                       the same channels
 *            MOTION_EVENT_CTL (addr 78) — calibration / motion features
 *
 * Function under test: ICM20948_dmpWriteOutputConfig
 */

#include "mock_common.h"
#include "icm20948.c"

Describe(DmpWriteOutputConfig);
BeforeEach(DmpWriteOutputConfig) { mock_reset(); }
AfterEach(DmpWriteOutputConfig)  {}

/*
 * GIVEN outCtl1 = 0xE400 (accel + gyro + compass + 9-axis quaternion)
 * WHEN  dmpWriteOutputConfig is called
 * THEN  DATA_OUT_CTL1 in DMP SRAM contains 0xE400 in big-endian order
 */
Ensure(DmpWriteOutputConfig, it_writes_outctl1_to_the_data_out_ctrl1_dmp_register)
{
    retval_t ret = ICM20948_dmpWriteOutputConfig(p_testSpi, 0xE400U, 0x0048U);

    assert_that(ret, is_equal_to(SPP_OK));
    assert_that(s_dmpMem[K_ICM20948_DMP_DATA_OUT_CTL1 + 0U], is_equal_to(0xE4));
    assert_that(s_dmpMem[K_ICM20948_DMP_DATA_OUT_CTL1 + 1U], is_equal_to(0x00));
}

/*
 * GIVEN outCtl1 = 0xE400
 * WHEN  dmpWriteOutputConfig is called
 * THEN  DATA_INTR_CTL also equals 0xE400
 *       (interrupts fire for the same channels as the output enables)
 */
Ensure(DmpWriteOutputConfig, it_mirrors_outctl1_to_data_intr_ctl_for_interrupt_alignment)
{
    ICM20948_dmpWriteOutputConfig(p_testSpi, 0xE400U, 0x0048U);

    assert_that(s_dmpMem[K_ICM20948_DMP_DATA_INTR_CTL + 0U], is_equal_to(0xE4));
    assert_that(s_dmpMem[K_ICM20948_DMP_DATA_INTR_CTL + 1U], is_equal_to(0x00));
}

/*
 * GIVEN any outCtl1 value
 * WHEN  dmpWriteOutputConfig is called
 * THEN  DATA_OUT_CTL2 is always written as 0x0000
 *       (reserved register — must stay clear)
 */
Ensure(DmpWriteOutputConfig, it_always_zeroes_the_data_out_ctrl2_register)
{
    ICM20948_dmpWriteOutputConfig(p_testSpi, 0xE400U, 0x0048U);

    assert_that(s_dmpMem[K_ICM20948_DMP_DATA_OUT_CTL2 + 0U], is_equal_to(0x00));
    assert_that(s_dmpMem[K_ICM20948_DMP_DATA_OUT_CTL2 + 1U], is_equal_to(0x00));
}

/*
 * GIVEN motionEvent = 0x0048 (compass cal + gyro auto-cal)
 * WHEN  dmpWriteOutputConfig is called
 * THEN  MOTION_EVENT_CTL contains 0x0048 in big-endian order
 */
Ensure(DmpWriteOutputConfig, it_writes_the_motion_event_mask_to_motion_event_ctl)
{
    ICM20948_dmpWriteOutputConfig(p_testSpi, 0xE400U, 0x0048U);

    assert_that(s_dmpMem[K_ICM20948_DMP_MOTION_EVENT_CTL + 0U], is_equal_to(0x00));
    assert_that(s_dmpMem[K_ICM20948_DMP_MOTION_EVENT_CTL + 1U], is_equal_to(0x48));
}

/*
 * GIVEN outCtl1 = 0x0000 and motionEvent = 0x0000
 * WHEN  dmpWriteOutputConfig is called
 * THEN  all four DMP registers are zero (all outputs disabled)
 */
Ensure(DmpWriteOutputConfig, it_can_disable_all_outputs_by_writing_zeros)
{
    retval_t ret = ICM20948_dmpWriteOutputConfig(p_testSpi, 0x0000U, 0x0000U);

    assert_that(ret, is_equal_to(SPP_OK));
    assert_that(s_dmpMem[K_ICM20948_DMP_DATA_OUT_CTL1],    is_equal_to(0x00));
    assert_that(s_dmpMem[K_ICM20948_DMP_DATA_INTR_CTL],    is_equal_to(0x00));
    assert_that(s_dmpMem[K_ICM20948_DMP_DATA_OUT_CTL2],    is_equal_to(0x00));
    assert_that(s_dmpMem[K_ICM20948_DMP_MOTION_EVENT_CTL], is_equal_to(0x00));
}

/*
 * GIVEN the SPI bus fails on the very first DMP write
 * WHEN  dmpWriteOutputConfig is called
 * THEN  the error is returned immediately
 */
Ensure(DmpWriteOutputConfig, it_propagates_failure_from_the_first_dmp_write)
{
    s_spiFailAtCall = 1;

    retval_t ret = ICM20948_dmpWriteOutputConfig(p_testSpi, 0xE400U, 0x0048U);

    assert_that(ret, is_equal_to(SPP_ERROR));
}

/* ================================================================
 * Test runner
 * ================================================================ */

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    TestSuite *suite = create_test_suite();

    add_test_with_context(suite, DmpWriteOutputConfig, it_writes_outctl1_to_the_data_out_ctrl1_dmp_register);
    add_test_with_context(suite, DmpWriteOutputConfig, it_mirrors_outctl1_to_data_intr_ctl_for_interrupt_alignment);
    add_test_with_context(suite, DmpWriteOutputConfig, it_always_zeroes_the_data_out_ctrl2_register);
    add_test_with_context(suite, DmpWriteOutputConfig, it_writes_the_motion_event_mask_to_motion_event_ctl);
    add_test_with_context(suite, DmpWriteOutputConfig, it_can_disable_all_outputs_by_writing_zeros);
    add_test_with_context(suite, DmpWriteOutputConfig, it_propagates_failure_from_the_first_dmp_write);

    return run_test_suite(suite, create_text_reporter());
}
