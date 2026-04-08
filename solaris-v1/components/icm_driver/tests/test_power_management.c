/*
 * test_power_management.c
 *
 * BDD suite — Power Management & FIFO Reset
 *
 * Feature A — Low-power wake cycle (ICM20948_lpWakeCycle):
 *   After each DMP SRAM write the firmware performs a two-step
 *   PWR_MGMT_1 toggle to keep the chip's PLL stable:
 *     1. Write 0x21  (CLK_AUTO | LP_EN)  — enter low-power
 *     2. Write 0x01  (CLK_AUTO only)     — exit low-power
 *
 * Feature B — FIFO reset (ICM20948_resetFifo):
 *   The FIFO is reset by a two-step register write to FIFO_RST (0x68):
 *     1. Assert all five reset bits:   0x1F
 *     2. Deassert bit 0 only:          0x1E
 *
 * Functions under test: ICM20948_lpWakeCycle, ICM20948_resetFifo
 */

#include "mock_common.h"
#include "icm20948.c"

Describe(PowerManagement);
BeforeEach(PowerManagement) { mock_reset(); }
AfterEach(PowerManagement)  {}

/* ---- lpWakeCycle ---------------------------------------------- */

/*
 * GIVEN a valid SPI handle
 * WHEN  lpWakeCycle is called
 * THEN  the first write to PWR_MGMT_1 sets CLK_AUTO and LP_EN (0x21)
 */
Ensure(PowerManagement, lpWakeCycle_first_writes_clkauto_with_lpen_to_enter_low_power_mode)
{
    ICM20948_lpWakeCycle(p_testSpi);

    assert_that(s_txHistReg[0], is_equal_to(K_ICM20948_REG_PWR_MGMT_1));
    assert_that(s_txHistVal[0], is_equal_to(0x21));
}

/*
 * GIVEN a valid SPI handle
 * WHEN  lpWakeCycle is called
 * THEN  the second write to PWR_MGMT_1 clears LP_EN, leaving only CLK_AUTO (0x01)
 */
Ensure(PowerManagement, lpWakeCycle_then_writes_clkauto_only_to_exit_low_power_mode)
{
    ICM20948_lpWakeCycle(p_testSpi);

    assert_that(s_txHistReg[1], is_equal_to(K_ICM20948_REG_PWR_MGMT_1));
    assert_that(s_txHistVal[1], is_equal_to(0x01));
}

/*
 * GIVEN a null SPI handle
 * WHEN  lpWakeCycle is called
 * THEN  SPP_ERROR_NULL_POINTER is returned without any SPI access
 */
Ensure(PowerManagement, lpWakeCycle_returns_null_pointer_error_when_the_handle_is_null)
{
    retval_t ret = ICM20948_lpWakeCycle(NULL);

    assert_that(ret, is_equal_to(SPP_ERROR_NULL_POINTER));
    assert_that(s_spiCallCount, is_equal_to(0));
}

/*
 * GIVEN the SPI bus fails on the first write
 * WHEN  lpWakeCycle is called
 * THEN  the error is returned before the second write occurs
 */
Ensure(PowerManagement, lpWakeCycle_propagates_the_first_write_failure_immediately)
{
    s_spiFailAtCall = 1;

    retval_t ret = ICM20948_lpWakeCycle(p_testSpi);

    assert_that(ret, is_equal_to(SPP_ERROR));
    assert_that(s_spiCallCount, is_equal_to(1));
}

/*
 * GIVEN the SPI bus fails on the second write
 * WHEN  lpWakeCycle is called
 * THEN  the error is returned after the first write succeeds
 */
Ensure(PowerManagement, lpWakeCycle_propagates_the_second_write_failure_to_the_caller)
{
    s_spiFailAtCall = 2;

    retval_t ret = ICM20948_lpWakeCycle(p_testSpi);

    assert_that(ret, is_equal_to(SPP_ERROR));
    assert_that(s_spiCallCount, is_equal_to(2));
}

/* ---- resetFifo ------------------------------------------------ */

/*
 * GIVEN a valid SPI handle
 * WHEN  resetFifo is called
 * THEN  the first write asserts all five FIFO reset bits (0x1F)
 */
Ensure(PowerManagement, resetFifo_asserts_all_five_fifo_reset_bits_in_the_first_write)
{
    ICM20948_resetFifo(p_testSpi);

    assert_that(s_txHistReg[0], is_equal_to(K_ICM20948_REG_FIFO_RST));
    assert_that(s_txHistVal[0], is_equal_to(0x1F));
}

/*
 * GIVEN a valid SPI handle
 * WHEN  resetFifo is called
 * THEN  the second write deasserts only bit 0, leaving bits 1-4 set (0x1E)
 *       This completes the documented reset sequence.
 */
Ensure(PowerManagement, resetFifo_deasserts_only_bit_zero_in_the_second_write)
{
    ICM20948_resetFifo(p_testSpi);

    assert_that(s_txHistReg[1], is_equal_to(K_ICM20948_REG_FIFO_RST));
    assert_that(s_txHistVal[1], is_equal_to(0x1E));
}

/*
 * GIVEN the SPI bus fails on the first write (assert phase)
 * WHEN  resetFifo is called
 * THEN  the error is returned; the deassert write never occurs
 */
Ensure(PowerManagement, resetFifo_propagates_failure_from_the_assert_write)
{
    s_spiFailAtCall = 1;

    retval_t ret = ICM20948_resetFifo(p_testSpi);

    assert_that(ret, is_equal_to(SPP_ERROR));
    assert_that(s_spiCallCount, is_equal_to(1));
}

/*
 * GIVEN the SPI bus fails on the second write (deassert phase)
 * WHEN  resetFifo is called
 * THEN  the error is returned after the assert write succeeds
 */
Ensure(PowerManagement, resetFifo_propagates_failure_from_the_deassert_write)
{
    s_spiFailAtCall = 2;

    retval_t ret = ICM20948_resetFifo(p_testSpi);

    assert_that(ret, is_equal_to(SPP_ERROR));
    assert_that(s_spiCallCount, is_equal_to(2));
}

/* ================================================================
 * Test runner
 * ================================================================ */

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    TestSuite *suite = create_test_suite();

    add_test_with_context(suite, PowerManagement, lpWakeCycle_first_writes_clkauto_with_lpen_to_enter_low_power_mode);
    add_test_with_context(suite, PowerManagement, lpWakeCycle_then_writes_clkauto_only_to_exit_low_power_mode);
    add_test_with_context(suite, PowerManagement, lpWakeCycle_returns_null_pointer_error_when_the_handle_is_null);
    add_test_with_context(suite, PowerManagement, lpWakeCycle_propagates_the_first_write_failure_immediately);
    add_test_with_context(suite, PowerManagement, lpWakeCycle_propagates_the_second_write_failure_to_the_caller);

    add_test_with_context(suite, PowerManagement, resetFifo_asserts_all_five_fifo_reset_bits_in_the_first_write);
    add_test_with_context(suite, PowerManagement, resetFifo_deasserts_only_bit_zero_in_the_second_write);
    add_test_with_context(suite, PowerManagement, resetFifo_propagates_failure_from_the_assert_write);
    add_test_with_context(suite, PowerManagement, resetFifo_propagates_failure_from_the_deassert_write);

    return run_test_suite(suite, create_text_reporter());
}
