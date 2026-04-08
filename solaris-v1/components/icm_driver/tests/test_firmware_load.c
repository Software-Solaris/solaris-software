/*
 * test_firmware_load.c
 *
 * BDD suite — DMP Firmware Loading
 *
 * Feature: The DMP has no persistent flash; its firmware image must be
 *          written into DMP SRAM on every power-up, starting at address
 *          K_ICM20948_DMP_LOAD_START (0x0090).  After writing, every byte
 *          is read back and compared against the image to detect corruption.
 *
 * Function under test: ICM20948_loadDmp
 */

#include "mock_common.h"
#include "icm20948.c"

Describe(LoadDmp);
BeforeEach(LoadDmp) { mock_reset(); }
AfterEach(LoadDmp)  {}

/*
 * GIVEN a null SPI handle
 * WHEN  loadDmp is called
 * THEN  SPP_ERROR_NULL_POINTER is returned without any SPI access
 */
Ensure(LoadDmp, it_returns_null_pointer_error_when_the_handle_is_null)
{
    retval_t ret = ICM20948_loadDmp(NULL);

    assert_that(ret, is_equal_to(SPP_ERROR_NULL_POINTER));
    assert_that(s_spiCallCount, is_equal_to(0));
}

/*
 * GIVEN a valid SPI handle and an empty DMP SRAM model
 * WHEN  loadDmp is called
 * THEN  it returns SPP_OK
 */
Ensure(LoadDmp, it_succeeds_when_the_spi_bus_is_healthy)
{
    retval_t ret = ICM20948_loadDmp(p_testSpi);

    assert_that(ret, is_equal_to(SPP_OK));
}

/*
 * GIVEN a valid SPI handle
 * WHEN  loadDmp completes successfully
 * THEN  the DMP SRAM starting at address 0x0090 matches the firmware image
 *       byte for byte
 */
Ensure(LoadDmp, it_writes_the_entire_firmware_image_starting_at_the_load_address)
{
    ICM20948_loadDmp(p_testSpi);

    /* The firmware image is s_dmp3Image[], a static array in icm20948.c.
     * Because that file is #included into this TU, we can access it here. */
    spp_uint16_t firmwareSize = (spp_uint16_t)sizeof(s_dmp3Image);
    int mismatch = memcmp(&s_dmpMem[K_ICM20948_DMP_LOAD_START],
                          s_dmp3Image,
                          firmwareSize);

    assert_that(mismatch, is_equal_to(0));
}

/*
 * GIVEN a valid SPI handle
 * WHEN  loadDmp completes successfully
 * THEN  the very first byte in DMP SRAM at 0x0090 matches firmware byte [0]
 *       (confirms the load-start address is honoured)
 */
Ensure(LoadDmp, it_begins_writing_at_dmp_load_address_0x0090)
{
    ICM20948_loadDmp(p_testSpi);

    assert_that(s_dmpMem[K_ICM20948_DMP_LOAD_START], is_equal_to(s_dmp3Image[0]));
    /* Address below 0x0090 must remain untouched */
    assert_that(s_dmpMem[K_ICM20948_DMP_LOAD_START - 1U], is_equal_to(0x00));
}

/*
 * GIVEN the DMP memory at the first load address is corrupted
 *       (the readback returns a wrong byte for that address)
 * WHEN  loadDmp is called
 * THEN  it detects the mismatch during the verify phase and returns SPP_ERROR
 */
Ensure(LoadDmp, it_detects_and_reports_readback_mismatch_as_firmware_corruption)
{
    /* Corrupt the readback of the very first firmware byte */
    s_dmpCorruptAddr = K_ICM20948_DMP_LOAD_START;
    s_dmpCorruptVal  = (spp_uint8_t)(~s_dmp3Image[0]);

    retval_t ret = ICM20948_loadDmp(p_testSpi);

    assert_that(ret, is_equal_to(SPP_ERROR));
}

/*
 * GIVEN the SPI bus fails on the 5th call (during the firmware write loop)
 * WHEN  loadDmp is called
 * THEN  the error is propagated to the caller immediately
 */
Ensure(LoadDmp, it_propagates_spi_failure_during_the_firmware_write_phase)
{
    s_spiFailAtCall = 5;

    retval_t ret = ICM20948_loadDmp(p_testSpi);

    assert_that(ret, is_equal_to(SPP_ERROR));
}

/*
 * GIVEN a full write phase completes but the SPI fails on a read during verify
 * WHEN  loadDmp is called
 * THEN  the error is propagated to the caller
 *
 * Note: the verify phase starts after 3 × firmwareSize write-phase SPI calls
 *       (each byte needs MEM_BANK_SEL + MEM_START_ADDR + MEM_R_W) plus the
 *       initial setBank call.  We inject at the (3N + 2)th call, guaranteed
 *       to land in the verify phase.
 */
Ensure(LoadDmp, it_propagates_spi_failure_during_the_firmware_verify_phase)
{
    spp_uint16_t firmwareSize = (spp_uint16_t)sizeof(s_dmp3Image);
    /* Write phase: 1 (setBank) + 3 * firmwareSize SPI calls.
     * Fail at write-phase-count + 2 to land safely in the verify loop. */
    s_spiFailAtCall = 1 + 3 * (int)firmwareSize + 2;

    retval_t ret = ICM20948_loadDmp(p_testSpi);

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

    add_test_with_context(suite, LoadDmp, it_returns_null_pointer_error_when_the_handle_is_null);
    add_test_with_context(suite, LoadDmp, it_succeeds_when_the_spi_bus_is_healthy);
    add_test_with_context(suite, LoadDmp, it_writes_the_entire_firmware_image_starting_at_the_load_address);
    add_test_with_context(suite, LoadDmp, it_begins_writing_at_dmp_load_address_0x0090);
    add_test_with_context(suite, LoadDmp, it_detects_and_reports_readback_mismatch_as_firmware_corruption);
    add_test_with_context(suite, LoadDmp, it_propagates_spi_failure_during_the_firmware_write_phase);
    add_test_with_context(suite, LoadDmp, it_propagates_spi_failure_during_the_firmware_verify_phase);

    return run_test_suite(suite, create_text_reporter());
}
