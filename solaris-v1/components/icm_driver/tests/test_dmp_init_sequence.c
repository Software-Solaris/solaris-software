/*
 * test_dmp_init_sequence.c
 *
 * BDD suite — Full DMP Initialisation Sequence
 *
 * Feature: configDmpInit runs a ~1000-line hardware initialisation sequence:
 *            1. Verify WHO_AM_I register (must return 0xEA)
 *            2. Load DMP firmware into SRAM
 *            3. Configure sensors, power, interrupts, matrices, ODR, etc.
 *            4. Enable the DMP and reset the FIFO
 *
 *          Every SPI call is guarded; if any call fails, the function must
 *          return the error immediately so the caller can handle it.
 *
 * Function under test: ICM20948_configDmpInit
 */

#include "mock_common.h"
#include "icm20948.c"

Describe(ConfigDmpInit);
BeforeEach(ConfigDmpInit) { mock_reset(); }
AfterEach(ConfigDmpInit)  {}

/*
 * GIVEN a null SPI handle
 * WHEN  configDmpInit is called
 * THEN  SPP_ERROR_NULL_POINTER is returned without any SPI access
 */
Ensure(ConfigDmpInit, it_returns_null_pointer_error_when_the_handle_is_null)
{
    retval_t ret = ICM20948_configDmpInit(NULL);

    assert_that(ret, is_equal_to(SPP_ERROR_NULL_POINTER));
    assert_that(s_spiCallCount, is_equal_to(0));
}

/*
 * GIVEN the device reports WHO_AM_I = 0xEA (correct device)
 * WHEN  configDmpInit is called
 * THEN  it completes the full sequence and returns SPP_OK
 */
Ensure(ConfigDmpInit, it_succeeds_when_the_device_reports_the_correct_who_am_i)
{
    /* mock_reset() pre-loads s_regs[0][0x00] = 0xEA */
    retval_t ret = ICM20948_configDmpInit(p_testSpi);

    assert_that(ret, is_equal_to(SPP_OK));
}

/*
 * GIVEN the device returns WHO_AM_I = 0x00 (wrong device / no device)
 * WHEN  configDmpInit is called
 * THEN  it returns SPP_ERROR after reading the WHO_AM_I register
 *       without continuing the initialisation sequence
 */
Ensure(ConfigDmpInit, it_fails_when_the_device_reports_a_wrong_who_am_i_value)
{
    s_regs[0][K_ICM20948_REG_WHO_AM_I] = 0x00U;  /* wrong device ID */

    retval_t ret = ICM20948_configDmpInit(p_testSpi);

    assert_that(ret, is_equal_to(SPP_ERROR));
}

/*
 * GIVEN the SPI bus can fail at any call during the init sequence
 * WHEN  we inject a failure at every individual SPI call index
 * THEN  every injection causes configDmpInit to return a non-OK result
 *       (every error path is guarded and propagated)
 *
 * This test exercises every `if (ret != SPP_OK) return ret;` branch.
 */
Ensure(ConfigDmpInit, it_surfaces_every_spi_failure_to_the_caller)
{
    /* Count total SPI calls in a successful run */
    mock_reset();
    ICM20948_configDmpInit(p_testSpi);
    int totalCalls = s_spiCallCount;

    int failCount = 0;
    for (int i = 1; i <= totalCalls; i++)
    {
        mock_reset();
        s_spiFailAtCall = i;
        retval_t ret = ICM20948_configDmpInit(p_testSpi);
        if (ret != SPP_OK)
        {
            failCount++;
        }
    }

    assert_that(failCount, is_equal_to(totalCalls));
}

/* ================================================================
 * Test runner
 * ================================================================ */

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    TestSuite *suite = create_test_suite();

    add_test_with_context(suite, ConfigDmpInit, it_returns_null_pointer_error_when_the_handle_is_null);
    add_test_with_context(suite, ConfigDmpInit, it_succeeds_when_the_device_reports_the_correct_who_am_i);
    add_test_with_context(suite, ConfigDmpInit, it_fails_when_the_device_reports_a_wrong_who_am_i_value);
    add_test_with_context(suite, ConfigDmpInit, it_surfaces_every_spi_failure_to_the_caller);

    return run_test_suite(suite, create_text_reporter());
}
