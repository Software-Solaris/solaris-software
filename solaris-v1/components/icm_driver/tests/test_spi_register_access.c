/*
 * test_spi_register_access.c
 *
 * BDD suite — SPI Register Read / Write
 *
 * Feature: The driver communicates with the ICM20948 over SPI using 2-byte
 *          transactions.  Writes assert bit 7 = 0; reads assert bit 7 = 1.
 *          The register address occupies bits[6:0] of the first byte and the
 *          data occupies the second byte.
 *
 * Functions under test: ICM20948_writeReg, ICM20948_readReg
 */

#include "mock_common.h"
#include "icm20948.c"

Describe(SpiRegisterAccess);
BeforeEach(SpiRegisterAccess) { mock_reset(); }
AfterEach(SpiRegisterAccess)  {}

/* ---- writeReg ------------------------------------------------- */

/*
 * GIVEN a valid SPI handle, register 0x06, value 0xAB
 * WHEN  writeReg is called
 * THEN  the value appears in the register model at bank-0 address 0x06
 *       (confirming the write-opcode bit was clear so the mock treated
 *        it as a write transaction)
 */
Ensure(SpiRegisterAccess, writeReg_stores_the_value_at_the_target_register_address)
{
    retval_t ret = ICM20948_writeReg(p_testSpi, 0x06U, 0xABU);

    assert_that(ret, is_equal_to(SPP_OK));
    assert_that(s_regs[0][0x06], is_equal_to(0xAB));
}

/*
 * GIVEN two separate registers written with distinct values
 * WHEN  both are written via writeReg
 * THEN  each register holds its own value (no aliasing)
 */
Ensure(SpiRegisterAccess, writeReg_targets_the_correct_register_address)
{
    ICM20948_writeReg(p_testSpi, 0x03U, 0x11U);
    ICM20948_writeReg(p_testSpi, 0x05U, 0x22U);

    assert_that(s_regs[0][0x03], is_equal_to(0x11));
    assert_that(s_regs[0][0x05], is_equal_to(0x22));
    assert_that(s_regs[0][0x03], is_not_equal_to(s_regs[0][0x05]));
}

/*
 * GIVEN the SPI bus returns an error on the first call
 * WHEN  writeReg is called
 * THEN  the error code is returned to the caller
 */
Ensure(SpiRegisterAccess, writeReg_propagates_the_spi_bus_error_to_the_caller)
{
    s_spiFailAtCall = 1;

    retval_t ret = ICM20948_writeReg(p_testSpi, 0x06U, 0x01U);

    assert_that(ret, is_equal_to(SPP_ERROR));
}

/* ---- readReg -------------------------------------------------- */

/*
 * GIVEN register 0x05 in bank 0 contains 0xAB
 * WHEN  readReg is called for that address
 * THEN  the returned value equals 0xAB
 */
Ensure(SpiRegisterAccess, readReg_retrieves_the_value_stored_in_the_target_register)
{
    s_regs[0][0x05] = 0xABU;

    spp_uint8_t val = 0U;
    retval_t    ret = ICM20948_readReg(p_testSpi, 0x05U, &val);

    assert_that(ret, is_equal_to(SPP_OK));
    assert_that(val, is_equal_to(0xAB));
}

/*
 * GIVEN a null output pointer is passed
 * WHEN  readReg is called
 * THEN  the SPI transaction still completes successfully without crashing
 */
Ensure(SpiRegisterAccess, readReg_succeeds_and_does_not_crash_when_output_pointer_is_null)
{
    retval_t ret = ICM20948_readReg(p_testSpi, 0x00U, NULL);

    assert_that(ret, is_equal_to(SPP_OK));
}

/*
 * GIVEN the SPI bus returns an error on the first call
 * WHEN  readReg is called
 * THEN  the error code is returned to the caller
 */
Ensure(SpiRegisterAccess, readReg_propagates_the_spi_bus_error_to_the_caller)
{
    s_spiFailAtCall = 1;

    spp_uint8_t val = 0U;
    retval_t    ret = ICM20948_readReg(p_testSpi, 0x00U, &val);

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

    add_test_with_context(suite, SpiRegisterAccess, writeReg_stores_the_value_at_the_target_register_address);
    add_test_with_context(suite, SpiRegisterAccess, writeReg_targets_the_correct_register_address);
    add_test_with_context(suite, SpiRegisterAccess, writeReg_propagates_the_spi_bus_error_to_the_caller);

    add_test_with_context(suite, SpiRegisterAccess, readReg_retrieves_the_value_stored_in_the_target_register);
    add_test_with_context(suite, SpiRegisterAccess, readReg_succeeds_and_does_not_crash_when_output_pointer_is_null);
    add_test_with_context(suite, SpiRegisterAccess, readReg_propagates_the_spi_bus_error_to_the_caller);

    return run_test_suite(suite, create_text_reporter());
}
