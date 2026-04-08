/*
 * test_register_bank_selection.c
 *
 * BDD suite — Register Bank Selection
 *
 * Feature: The ICM20948 exposes four register banks selected by writing to
 *          REG_BANK_SEL (0x7F).  After selection, all register reads/writes
 *          target the chosen bank.  An invalid bank value must be rejected
 *          without any SPI access.
 *
 * Function under test: ICM20948_setBank
 */

#include "mock_common.h"
#include "icm20948.c"

Describe(RegisterBankSelection);
BeforeEach(RegisterBankSelection) { mock_reset(); }
AfterEach(RegisterBankSelection)  {}

/*
 * GIVEN bank 0 is requested
 * WHEN  setBank is called
 * THEN  subsequent SPI access targets the bank-0 register space
 */
Ensure(RegisterBankSelection, setBank_activates_bank_0_register_space)
{
    retval_t ret = ICM20948_setBank(p_testSpi, K_ICM20948_REG_BANK_0);

    assert_that(ret, is_equal_to(SPP_OK));
    assert_that(s_currentBank, is_equal_to(0));
}

/*
 * GIVEN bank 1 is requested
 * WHEN  setBank is called
 * THEN  subsequent SPI access targets the bank-1 register space
 */
Ensure(RegisterBankSelection, setBank_activates_bank_1_register_space)
{
    retval_t ret = ICM20948_setBank(p_testSpi, K_ICM20948_REG_BANK_1);

    assert_that(ret, is_equal_to(SPP_OK));
    assert_that(s_currentBank, is_equal_to(1));
}

/*
 * GIVEN bank 2 is requested
 * WHEN  setBank is called
 * THEN  subsequent SPI access targets the bank-2 register space
 */
Ensure(RegisterBankSelection, setBank_activates_bank_2_register_space)
{
    retval_t ret = ICM20948_setBank(p_testSpi, K_ICM20948_REG_BANK_2);

    assert_that(ret, is_equal_to(SPP_OK));
    assert_that(s_currentBank, is_equal_to(2));
}

/*
 * GIVEN bank 3 is requested
 * WHEN  setBank is called
 * THEN  subsequent SPI access targets the bank-3 register space
 */
Ensure(RegisterBankSelection, setBank_activates_bank_3_register_space)
{
    retval_t ret = ICM20948_setBank(p_testSpi, K_ICM20948_REG_BANK_3);

    assert_that(ret, is_equal_to(SPP_OK));
    assert_that(s_currentBank, is_equal_to(3));
}

/*
 * GIVEN an out-of-range bank value (0xFF)
 * WHEN  setBank is called
 * THEN  SPP_ERROR is returned without issuing any SPI transaction
 */
Ensure(RegisterBankSelection, setBank_rejects_invalid_bank_number_without_spi_access)
{
    retval_t ret = ICM20948_setBank(p_testSpi, (ICM20948_RegBank_t)0xFFU);

    assert_that(ret, is_equal_to(SPP_ERROR));
    assert_that(s_spiCallCount, is_equal_to(0));
}

/*
 * GIVEN bank 1 holds value 0xAA and bank 0 holds 0x55 at register 0x28
 * WHEN  setBank switches between them and reads the same address
 * THEN  each read returns the value from the active bank (register spaces isolated)
 */
Ensure(RegisterBankSelection, setBank_isolates_register_contents_between_banks)
{
    s_regs[1][0x28] = 0xAAU;
    s_regs[0][0x28] = 0x55U;

    ICM20948_setBank(p_testSpi, K_ICM20948_REG_BANK_1);
    spp_uint8_t val1 = 0U;
    ICM20948_readReg(p_testSpi, 0x28U, &val1);

    ICM20948_setBank(p_testSpi, K_ICM20948_REG_BANK_0);
    spp_uint8_t val0 = 0U;
    ICM20948_readReg(p_testSpi, 0x28U, &val0);

    assert_that(val1, is_equal_to(0xAA));
    assert_that(val0, is_equal_to(0x55));
}

/* ================================================================
 * Test runner
 * ================================================================ */

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    TestSuite *suite = create_test_suite();

    add_test_with_context(suite, RegisterBankSelection, setBank_activates_bank_0_register_space);
    add_test_with_context(suite, RegisterBankSelection, setBank_activates_bank_1_register_space);
    add_test_with_context(suite, RegisterBankSelection, setBank_activates_bank_2_register_space);
    add_test_with_context(suite, RegisterBankSelection, setBank_activates_bank_3_register_space);
    add_test_with_context(suite, RegisterBankSelection, setBank_rejects_invalid_bank_number_without_spi_access);
    add_test_with_context(suite, RegisterBankSelection, setBank_isolates_register_contents_between_banks);

    return run_test_suite(suite, create_text_reporter());
}
