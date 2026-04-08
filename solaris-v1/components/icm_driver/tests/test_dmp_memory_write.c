/*
 * test_dmp_memory_write.c
 *
 * BDD suite — DMP SRAM Write Operations
 *
 * Feature: The DMP (Digital Motion Processor) has its own 16-KB SRAM
 *          accessed via three registers: MEM_BANK_SEL (0x7E), MEM_START_ADDR
 *          (0x7C), and MEM_R_W (0x7D).  The driver writes data byte-by-byte
 *          and provides 16-bit and 32-bit big-endian helpers.
 *
 * Functions under test: ICM20948_dmpWriteBytes, ICM20948_dmpWrite32,
 *                       ICM20948_dmpWrite16
 */

#include "mock_common.h"
#include "icm20948.c"

Describe(DmpMemoryWrite);
BeforeEach(DmpMemoryWrite) { mock_reset(); }
AfterEach(DmpMemoryWrite)  {}

/* ---- dmpWriteBytes -------------------------------------------- */

/*
 * GIVEN three bytes and DMP address 0x0100
 * WHEN  dmpWriteBytes is called
 * THEN  each byte appears in the DMP SRAM at consecutive addresses
 */
Ensure(DmpMemoryWrite, dmpWriteBytes_writes_each_byte_to_the_correct_sram_address)
{
    spp_uint8_t data[] = {0xAAU, 0xBBU, 0xCCU};

    retval_t ret = ICM20948_dmpWriteBytes(p_testSpi, 0x0100U, data, 3U);

    assert_that(ret, is_equal_to(SPP_OK));
    assert_that(s_dmpMem[0x0100], is_equal_to(0xAA));
    assert_that(s_dmpMem[0x0101], is_equal_to(0xBB));
    assert_that(s_dmpMem[0x0102], is_equal_to(0xCC));
}

/*
 * GIVEN two bytes starting at address 0x01FF (last byte of DMP bank 1)
 * WHEN  dmpWriteBytes is called
 * THEN  the second byte lands at 0x0200 (first byte of bank 2),
 *       confirming MEM_BANK_SEL is updated mid-write
 */
Ensure(DmpMemoryWrite, dmpWriteBytes_updates_bank_select_when_spanning_a_bank_boundary)
{
    spp_uint8_t data[] = {0x11U, 0x22U};

    retval_t ret = ICM20948_dmpWriteBytes(p_testSpi, 0x01FFU, data, 2U);

    assert_that(ret, is_equal_to(SPP_OK));
    assert_that(s_dmpMem[0x01FF], is_equal_to(0x11));
    assert_that(s_dmpMem[0x0200], is_equal_to(0x22));
}

/*
 * GIVEN a null SPI handle
 * WHEN  dmpWriteBytes is called
 * THEN  SPP_ERROR_NULL_POINTER is returned and no SPI access occurs
 */
Ensure(DmpMemoryWrite, dmpWriteBytes_returns_null_pointer_error_for_null_spi_handle)
{
    spp_uint8_t data[] = {0xAAU};

    retval_t ret = ICM20948_dmpWriteBytes(NULL, 0x0100U, data, 1U);

    assert_that(ret, is_equal_to(SPP_ERROR_NULL_POINTER));
    assert_that(s_spiCallCount, is_equal_to(0));
}

/*
 * GIVEN a null data pointer
 * WHEN  dmpWriteBytes is called
 * THEN  SPP_ERROR_NULL_POINTER is returned and no SPI access occurs
 */
Ensure(DmpMemoryWrite, dmpWriteBytes_returns_null_pointer_error_for_null_data_pointer)
{
    retval_t ret = ICM20948_dmpWriteBytes(p_testSpi, 0x0100U, NULL, 1U);

    assert_that(ret, is_equal_to(SPP_ERROR_NULL_POINTER));
    assert_that(s_spiCallCount, is_equal_to(0));
}

/*
 * GIVEN the SPI bus fails on the MEM_BANK_SEL write (call 1)
 * WHEN  dmpWriteBytes is called
 * THEN  SPP_ERROR is returned immediately
 */
Ensure(DmpMemoryWrite, dmpWriteBytes_propagates_failure_on_bank_select_write)
{
    spp_uint8_t data[] = {0xAAU};
    s_spiFailAtCall = 1;

    assert_that(ICM20948_dmpWriteBytes(p_testSpi, 0x0100U, data, 1U), is_equal_to(SPP_ERROR));
}

/*
 * GIVEN the SPI bus fails on the MEM_START_ADDR write (call 2)
 * WHEN  dmpWriteBytes is called
 * THEN  SPP_ERROR is returned immediately
 */
Ensure(DmpMemoryWrite, dmpWriteBytes_propagates_failure_on_start_address_write)
{
    spp_uint8_t data[] = {0xAAU};
    s_spiFailAtCall = 2;

    assert_that(ICM20948_dmpWriteBytes(p_testSpi, 0x0100U, data, 1U), is_equal_to(SPP_ERROR));
}

/*
 * GIVEN the SPI bus fails on the MEM_R_W data write (call 3)
 * WHEN  dmpWriteBytes is called
 * THEN  SPP_ERROR is returned immediately
 */
Ensure(DmpMemoryWrite, dmpWriteBytes_propagates_failure_on_data_write)
{
    spp_uint8_t data[] = {0xAAU};
    s_spiFailAtCall = 3;

    assert_that(ICM20948_dmpWriteBytes(p_testSpi, 0x0100U, data, 1U), is_equal_to(SPP_ERROR));
}

/* ---- dmpWrite32 ----------------------------------------------- */

/*
 * GIVEN value 0xDEADBEEF and DMP address 0x0200
 * WHEN  dmpWrite32 is called
 * THEN  bytes are stored most-significant-first: DE AD BE EF
 */
Ensure(DmpMemoryWrite, dmpWrite32_stores_value_in_big_endian_byte_order)
{
    retval_t ret = ICM20948_dmpWrite32(p_testSpi, 0x0200U, 0xDEADBEEFUL);

    assert_that(ret, is_equal_to(SPP_OK));
    assert_that(s_dmpMem[0x0200], is_equal_to(0xDE));
    assert_that(s_dmpMem[0x0201], is_equal_to(0xAD));
    assert_that(s_dmpMem[0x0202], is_equal_to(0xBE));
    assert_that(s_dmpMem[0x0203], is_equal_to(0xEF));
}

/*
 * GIVEN value 0x04000000 (accel scale for ±4 g in Q30 format)
 * WHEN  dmpWrite32 is called at K_ICM20948_DMP_ACC_SCALE
 * THEN  the four bytes are laid out big-endian
 */
Ensure(DmpMemoryWrite, dmpWrite32_correctly_encodes_q30_sensor_scale_constants)
{
    ICM20948_dmpWrite32(p_testSpi, K_ICM20948_DMP_ACC_SCALE, 0x04000000UL);

    assert_that(s_dmpMem[K_ICM20948_DMP_ACC_SCALE + 0U], is_equal_to(0x04));
    assert_that(s_dmpMem[K_ICM20948_DMP_ACC_SCALE + 1U], is_equal_to(0x00));
    assert_that(s_dmpMem[K_ICM20948_DMP_ACC_SCALE + 2U], is_equal_to(0x00));
    assert_that(s_dmpMem[K_ICM20948_DMP_ACC_SCALE + 3U], is_equal_to(0x00));
}

/*
 * GIVEN the SPI bus fails on the first call
 * WHEN  dmpWrite32 is called
 * THEN  the error propagates to the caller
 */
Ensure(DmpMemoryWrite, dmpWrite32_propagates_spi_failure_to_the_caller)
{
    s_spiFailAtCall = 1;

    assert_that(ICM20948_dmpWrite32(p_testSpi, 0x0200U, 0x12345678UL), is_equal_to(SPP_ERROR));
}

/* ---- dmpWrite16 ----------------------------------------------- */

/*
 * GIVEN value 0xCAFE and DMP address 0x0300
 * WHEN  dmpWrite16 is called
 * THEN  high byte 0xCA is at 0x0300 and low byte 0xFE is at 0x0301
 */
Ensure(DmpMemoryWrite, dmpWrite16_stores_value_in_big_endian_byte_order)
{
    retval_t ret = ICM20948_dmpWrite16(p_testSpi, 0x0300U, 0xCAFEU);

    assert_that(ret, is_equal_to(SPP_OK));
    assert_that(s_dmpMem[0x0300], is_equal_to(0xCA));
    assert_that(s_dmpMem[0x0301], is_equal_to(0xFE));
}

/*
 * GIVEN the SPI bus fails on the first call
 * WHEN  dmpWrite16 is called
 * THEN  the error propagates to the caller
 */
Ensure(DmpMemoryWrite, dmpWrite16_propagates_spi_failure_to_the_caller)
{
    s_spiFailAtCall = 1;

    assert_that(ICM20948_dmpWrite16(p_testSpi, 0x0300U, 0x1234U), is_equal_to(SPP_ERROR));
}

/* ================================================================
 * Test runner
 * ================================================================ */

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    TestSuite *suite = create_test_suite();

    add_test_with_context(suite, DmpMemoryWrite, dmpWriteBytes_writes_each_byte_to_the_correct_sram_address);
    add_test_with_context(suite, DmpMemoryWrite, dmpWriteBytes_updates_bank_select_when_spanning_a_bank_boundary);
    add_test_with_context(suite, DmpMemoryWrite, dmpWriteBytes_returns_null_pointer_error_for_null_spi_handle);
    add_test_with_context(suite, DmpMemoryWrite, dmpWriteBytes_returns_null_pointer_error_for_null_data_pointer);
    add_test_with_context(suite, DmpMemoryWrite, dmpWriteBytes_propagates_failure_on_bank_select_write);
    add_test_with_context(suite, DmpMemoryWrite, dmpWriteBytes_propagates_failure_on_start_address_write);
    add_test_with_context(suite, DmpMemoryWrite, dmpWriteBytes_propagates_failure_on_data_write);

    add_test_with_context(suite, DmpMemoryWrite, dmpWrite32_stores_value_in_big_endian_byte_order);
    add_test_with_context(suite, DmpMemoryWrite, dmpWrite32_correctly_encodes_q30_sensor_scale_constants);
    add_test_with_context(suite, DmpMemoryWrite, dmpWrite32_propagates_spi_failure_to_the_caller);

    add_test_with_context(suite, DmpMemoryWrite, dmpWrite16_stores_value_in_big_endian_byte_order);
    add_test_with_context(suite, DmpMemoryWrite, dmpWrite16_propagates_spi_failure_to_the_caller);

    return run_test_suite(suite, create_text_reporter());
}
