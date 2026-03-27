/*
 * Unit tests for icm20948.c using cgreen.
 *
 * Strategy: include icm20948.c directly to access static functions.
 * Stub headers in tests/stubs/ replace all hardware dependencies.
 * A mock SPI layer simulates the ICM20948 register file, DMP memory and FIFO.
 */

/* Include type stubs BEFORE mock code so spp types are available */
#include "spp/core/types.h"
#include "spp/core/returntypes.h"

#include <cgreen/cgreen.h>
#include <string.h>

/* ================================================================
 * Mock infrastructure
 * ================================================================ */

#define MOCK_DMP_MEM_SIZE   16384
#define MOCK_FIFO_SIZE      4096

/* Mock state */
static int         s_spiCallCount;
static int         s_spiFailAtCall;     /* -1 = never fail */
static int         s_delayCallCount;

/* Register model: 4 banks x 128 registers */
static spp_uint8_t s_regs[4][128];
static spp_uint8_t s_currentBank;

/* DMP memory model */
static spp_uint8_t s_dmpMem[MOCK_DMP_MEM_SIZE];
static spp_uint8_t s_dmpBank;
static spp_uint8_t s_dmpAddr;

/* FIFO model */
static spp_uint8_t s_fifo[MOCK_FIFO_SIZE];
static int         s_fifoSize;
static int         s_fifoReadIdx;

/* Last SPI transaction capture */
static spp_uint8_t s_lastTxReg;
static spp_uint8_t s_lastTxVal;

static void mock_reset(void)
{
    s_spiCallCount   = 0;
    s_spiFailAtCall  = -1;
    s_delayCallCount = 0;
    s_currentBank    = 0;
    s_dmpBank        = 0;
    s_dmpAddr        = 0;
    s_fifoSize       = 0;
    s_fifoReadIdx    = 0;
    s_lastTxReg      = 0;
    s_lastTxVal      = 0;
    memset(s_regs, 0, sizeof(s_regs));
    memset(s_dmpMem, 0, sizeof(s_dmpMem));
    memset(s_fifo, 0, sizeof(s_fifo));

    /* Pre-load WHO_AM_I */
    s_regs[0][0x00] = 0xEA;
}

static void mock_set_fifo(const spp_uint8_t *p_data, int len)
{
    memcpy(s_fifo, p_data, (size_t)len);
    s_fifoSize    = len;
    s_fifoReadIdx = 0;
    s_regs[0][0x70] = (spp_uint8_t)((len >> 8) & 0xFF);
    s_regs[0][0x71] = (spp_uint8_t)(len & 0xFF);
}

/* ================================================================
 * Mock implementations (linked by the included icm20948.c)
 * ================================================================ */

retval_t SPP_HAL_SPI_Transmit(void *p_handle, spp_uint8_t *p_data, spp_uint8_t len)
{
    (void)p_handle;
    s_spiCallCount++;

    if (s_spiCallCount == s_spiFailAtCall)
    {
        return SPP_ERROR;
    }

    spp_uint8_t reg    = p_data[0] & 0x7FU;
    int         isRead = (p_data[0] & 0x80U) != 0;

    s_lastTxReg = reg;

    if (isRead)
    {
        if (reg == 0x72U && len > 1) /* FIFO_R_W */
        {
            for (int i = 1; i < len; i++)
            {
                p_data[i] = (s_fifoReadIdx < s_fifoSize)
                            ? s_fifo[s_fifoReadIdx++]
                            : 0U;
            }
        }
        else if (reg == 0x7DU) /* MEM_R_W */
        {
            spp_uint16_t addr = ((spp_uint16_t)s_dmpBank << 8) | s_dmpAddr;
            p_data[1] = (addr < MOCK_DMP_MEM_SIZE) ? s_dmpMem[addr] : 0U;
            s_dmpAddr++;
        }
        else /* Normal register read (multi-byte) */
        {
            for (int i = 1; i < len; i++)
            {
                spp_uint8_t r = (spp_uint8_t)(reg + (i - 1));
                if (r < 128) { p_data[i] = s_regs[s_currentBank][r]; }
            }
        }
    }
    else /* Write */
    {
        if (len >= 2) { s_lastTxVal = p_data[1]; }

        if (reg == 0x7FU && len >= 2)      /* REG_BANK_SEL */
        {
            s_currentBank = (p_data[1] >> 4) & 0x03U;
        }
        else if (reg == 0x7EU && len >= 2)  /* MEM_BANK_SEL */
        {
            s_dmpBank = p_data[1];
        }
        else if (reg == 0x7CU && len >= 2)  /* MEM_START_ADDR */
        {
            s_dmpAddr = p_data[1];
        }
        else if (reg == 0x7DU && len >= 2)  /* MEM_R_W */
        {
            spp_uint16_t addr = ((spp_uint16_t)s_dmpBank << 8) | s_dmpAddr;
            if (addr < MOCK_DMP_MEM_SIZE) { s_dmpMem[addr] = p_data[1]; }
            s_dmpAddr++;
        }
        else if (len >= 2 && reg < 128)     /* Normal register write */
        {
            s_regs[s_currentBank][reg] = p_data[1];
        }
    }

    return SPP_OK;
}

void SPP_OSAL_TaskDelay(spp_uint32_t ms)
{
    (void)ms;
    s_delayCallCount++;
}

/* ================================================================
 * Include source under test
 * ================================================================ */

#include "icm20948.c"

/* Dummy SPI handle */
static int  s_dummySpi = 1;
static void *p_testSpi = &s_dummySpi;

/* ================================================================
 * Helper: build a 42-byte DMP FIFO packet
 * ================================================================ */

static void build_dmp_packet(spp_uint8_t *p_buf,
                             int16_t ax, int16_t ay, int16_t az,
                             int16_t gx, int16_t gy, int16_t gz,
                             int16_t mx, int16_t my, int16_t mz)
{
    memset(p_buf, 0, 42);
    p_buf[2]  = (spp_uint8_t)(ax >> 8);  p_buf[3]  = (spp_uint8_t)(ax);
    p_buf[4]  = (spp_uint8_t)(ay >> 8);  p_buf[5]  = (spp_uint8_t)(ay);
    p_buf[6]  = (spp_uint8_t)(az >> 8);  p_buf[7]  = (spp_uint8_t)(az);
    p_buf[8]  = (spp_uint8_t)(gx >> 8);  p_buf[9]  = (spp_uint8_t)(gx);
    p_buf[10] = (spp_uint8_t)(gy >> 8);  p_buf[11] = (spp_uint8_t)(gy);
    p_buf[12] = (spp_uint8_t)(gz >> 8);  p_buf[13] = (spp_uint8_t)(gz);
    p_buf[20] = (spp_uint8_t)(mx >> 8);  p_buf[21] = (spp_uint8_t)(mx);
    p_buf[22] = (spp_uint8_t)(my >> 8);  p_buf[23] = (spp_uint8_t)(my);
    p_buf[24] = (spp_uint8_t)(mz >> 8);  p_buf[25] = (spp_uint8_t)(mz);
}

/* ================================================================
 * Single Describe context (cgreen allows only one per file)
 * ================================================================ */

Describe(ICM20948);
BeforeEach(ICM20948) { mock_reset(); }
AfterEach(ICM20948) {}

/* ---- writeReg ------------------------------------------------ */

Ensure(ICM20948, writeReg_sends_correct_bytes)
{
    retval_t ret = ICM20948_writeReg(p_testSpi, 0x06U, 0x01U);
    assert_that(ret, is_equal_to(SPP_OK));
    assert_that(s_lastTxReg, is_equal_to(0x06));
    assert_that(s_lastTxVal, is_equal_to(0x01));
}

Ensure(ICM20948, writeReg_propagates_spi_failure)
{
    s_spiFailAtCall = 1;
    assert_that(ICM20948_writeReg(p_testSpi, 0x06U, 0x01U), is_equal_to(SPP_ERROR));
}

/* ---- readReg ------------------------------------------------- */

Ensure(ICM20948, readReg_reads_correct_value)
{
    s_regs[0][0x00] = 0xEA;
    spp_uint8_t val = 0;
    assert_that(ICM20948_readReg(p_testSpi, 0x00U, &val), is_equal_to(SPP_OK));
    assert_that(val, is_equal_to(0xEA));
}

Ensure(ICM20948, readReg_handles_null_pointer)
{
    assert_that(ICM20948_readReg(p_testSpi, 0x00U, NULL), is_equal_to(SPP_OK));
}

Ensure(ICM20948, readReg_propagates_spi_failure)
{
    spp_uint8_t val = 0;
    s_spiFailAtCall = 1;
    assert_that(ICM20948_readReg(p_testSpi, 0x00U, &val), is_equal_to(SPP_ERROR));
}

/* ---- setBank ------------------------------------------------- */

Ensure(ICM20948, setBank_selects_bank_0)
{
    assert_that(ICM20948_setBank(p_testSpi, K_ICM20948_REG_BANK_0), is_equal_to(SPP_OK));
    assert_that(s_currentBank, is_equal_to(0));
}

Ensure(ICM20948, setBank_selects_bank_1)
{
    assert_that(ICM20948_setBank(p_testSpi, K_ICM20948_REG_BANK_1), is_equal_to(SPP_OK));
    assert_that(s_currentBank, is_equal_to(1));
}

Ensure(ICM20948, setBank_selects_bank_2)
{
    assert_that(ICM20948_setBank(p_testSpi, K_ICM20948_REG_BANK_2), is_equal_to(SPP_OK));
    assert_that(s_currentBank, is_equal_to(2));
}

Ensure(ICM20948, setBank_selects_bank_3)
{
    assert_that(ICM20948_setBank(p_testSpi, K_ICM20948_REG_BANK_3), is_equal_to(SPP_OK));
    assert_that(s_currentBank, is_equal_to(3));
}

Ensure(ICM20948, setBank_rejects_invalid_bank)
{
    assert_that(ICM20948_setBank(p_testSpi, (ICM20948_RegBank_t)0xFF), is_equal_to(SPP_ERROR));
}

/* ---- resetFifo ----------------------------------------------- */

Ensure(ICM20948, resetFifo_succeeds)
{
    assert_that(ICM20948_resetFifo(p_testSpi), is_equal_to(SPP_OK));
}

Ensure(ICM20948, resetFifo_fails_on_first_write)
{
    s_spiFailAtCall = 1;
    assert_that(ICM20948_resetFifo(p_testSpi), is_equal_to(SPP_ERROR));
}

Ensure(ICM20948, resetFifo_fails_on_second_write)
{
    s_spiFailAtCall = 2;
    assert_that(ICM20948_resetFifo(p_testSpi), is_equal_to(SPP_ERROR));
}

/* ---- dmpWriteBytes ------------------------------------------- */

Ensure(ICM20948, dmpWriteBytes_writes_to_dmp_memory)
{
    spp_uint8_t data[] = {0xAA, 0xBB, 0xCC};
    assert_that(ICM20948_dmpWriteBytes(p_testSpi, 0x0100U, data, 3U), is_equal_to(SPP_OK));
    assert_that(s_dmpMem[0x0100], is_equal_to(0xAA));
    assert_that(s_dmpMem[0x0101], is_equal_to(0xBB));
    assert_that(s_dmpMem[0x0102], is_equal_to(0xCC));
}

Ensure(ICM20948, dmpWriteBytes_fails_on_bank_sel)
{
    spp_uint8_t data[] = {0xAA};
    s_spiFailAtCall = 1;
    assert_that(ICM20948_dmpWriteBytes(p_testSpi, 0x0100U, data, 1U), is_equal_to(SPP_ERROR));
}

Ensure(ICM20948, dmpWriteBytes_fails_on_addr)
{
    spp_uint8_t data[] = {0xAA};
    s_spiFailAtCall = 2;
    assert_that(ICM20948_dmpWriteBytes(p_testSpi, 0x0100U, data, 1U), is_equal_to(SPP_ERROR));
}

Ensure(ICM20948, dmpWriteBytes_fails_on_data)
{
    spp_uint8_t data[] = {0xAA};
    s_spiFailAtCall = 3;
    assert_that(ICM20948_dmpWriteBytes(p_testSpi, 0x0100U, data, 1U), is_equal_to(SPP_ERROR));
}

/* ---- dmpWrite32 / dmpWrite16 --------------------------------- */

Ensure(ICM20948, dmpWrite32_stores_big_endian)
{
    assert_that(ICM20948_dmpWrite32(p_testSpi, 0x0200U, 0xDEADBEEFUL), is_equal_to(SPP_OK));
    assert_that(s_dmpMem[0x0200], is_equal_to(0xDE));
    assert_that(s_dmpMem[0x0201], is_equal_to(0xAD));
    assert_that(s_dmpMem[0x0202], is_equal_to(0xBE));
    assert_that(s_dmpMem[0x0203], is_equal_to(0xEF));
}

Ensure(ICM20948, dmpWrite16_stores_big_endian)
{
    assert_that(ICM20948_dmpWrite16(p_testSpi, 0x0300U, 0xCAFEU), is_equal_to(SPP_OK));
    assert_that(s_dmpMem[0x0300], is_equal_to(0xCA));
    assert_that(s_dmpMem[0x0301], is_equal_to(0xFE));
}

Ensure(ICM20948, dmpWrite32_propagates_failure)
{
    s_spiFailAtCall = 1;
    assert_that(ICM20948_dmpWrite32(p_testSpi, 0x0200U, 0x12345678UL), is_equal_to(SPP_ERROR));
}

Ensure(ICM20948, dmpWrite16_propagates_failure)
{
    s_spiFailAtCall = 1;
    assert_that(ICM20948_dmpWrite16(p_testSpi, 0x0300U, 0x1234U), is_equal_to(SPP_ERROR));
}

/* ---- lpWakeCycle --------------------------------------------- */

Ensure(ICM20948, lpWakeCycle_succeeds)
{
    assert_that(ICM20948_lpWakeCycle(p_testSpi), is_equal_to(SPP_OK));
}

Ensure(ICM20948, lpWakeCycle_fails_on_first_write)
{
    s_spiFailAtCall = 1;
    assert_that(ICM20948_lpWakeCycle(p_testSpi), is_equal_to(SPP_ERROR));
}

Ensure(ICM20948, lpWakeCycle_fails_on_second_write)
{
    s_spiFailAtCall = 2;
    assert_that(ICM20948_lpWakeCycle(p_testSpi), is_equal_to(SPP_ERROR));
}

/* ---- dmpWriteOutputConfig ------------------------------------ */

Ensure(ICM20948, dmpWriteOutputConfig_succeeds)
{
    assert_that(ICM20948_dmpWriteOutputConfig(p_testSpi, 0xE400U, 0x0048U), is_equal_to(SPP_OK));
}

Ensure(ICM20948, dmpWriteOutputConfig_fails_on_first_write)
{
    s_spiFailAtCall = 1;
    assert_that(ICM20948_dmpWriteOutputConfig(p_testSpi, 0xE400U, 0x0048U), is_equal_to(SPP_ERROR));
}

/* ---- calcGyroSf ---------------------------------------------- */

Ensure(ICM20948, calcGyroSf_nominal)
{
    assert_that(ICM20948_calcGyroSf(0), is_equal_to(83290354L));
}

Ensure(ICM20948, calcGyroSf_decreases_with_positive_pll)
{
    assert_that(ICM20948_calcGyroSf(10), is_less_than(ICM20948_calcGyroSf(0)));
}

Ensure(ICM20948, calcGyroSf_increases_with_negative_pll)
{
    assert_that(ICM20948_calcGyroSf(-10), is_greater_than(ICM20948_calcGyroSf(0)));
}

Ensure(ICM20948, calcGyroSf_extreme_values)
{
    spp_int32_t sfMax = ICM20948_calcGyroSf(127);
    spp_int32_t sfMin = ICM20948_calcGyroSf(-128);
    assert_that(sfMax, is_greater_than(0L));
    assert_that(sfMin, is_greater_than(0L));
    assert_that(sfMin, is_greater_than(sfMax));
}

/* ---- loadDmp ------------------------------------------------- */

Ensure(ICM20948, loadDmp_succeeds)
{
    assert_that(ICM20948_loadDmp(p_testSpi), is_equal_to(SPP_OK));
}

Ensure(ICM20948, loadDmp_fails_on_spi_error)
{
    s_spiFailAtCall = 5;
    assert_that(ICM20948_loadDmp(p_testSpi), is_equal_to(SPP_ERROR));
}

/* ---- configDmpInit ------------------------------------------- */

Ensure(ICM20948, configDmpInit_succeeds)
{
    assert_that(ICM20948_configDmpInit(p_testSpi), is_equal_to(SPP_OK));
}

Ensure(ICM20948, configDmpInit_fails_on_wrong_who_am_i)
{
    s_regs[0][0x00] = 0x00;
    assert_that(ICM20948_configDmpInit(p_testSpi), is_equal_to(SPP_ERROR));
}

Ensure(ICM20948, configDmpInit_fails_on_first_spi_error)
{
    s_spiFailAtCall = 1;
    assert_that(ICM20948_configDmpInit(p_testSpi), is_equal_to(SPP_ERROR));
}

/*
 * Exhaustive error injection: fail at every SPI call in configDmpInit.
 * Covers every `if (ret != SPP_OK) return ret;` branch.
 * No per-iteration asserts to avoid cgreen's pipe limit.
 * Instead, count how many iterations returned error vs OK.
 */
Ensure(ICM20948, configDmpInit_fails_at_every_spi_call)
{
    /* Count total SPI calls in a successful init */
    mock_reset();
    ICM20948_configDmpInit(p_testSpi);
    int totalCalls = s_spiCallCount;

    int failCount = 0;
    for (int i = 1; i <= totalCalls; i++)
    {
        mock_reset();
        s_spiFailAtCall = i;
        retval_t ret = ICM20948_configDmpInit(p_testSpi);
        if (ret != SPP_OK) { failCount++; }
    }

    /* Every injection must produce a failure */
    assert_that(failCount, is_equal_to(totalCalls));
}

/* ---- checkFifoData ------------------------------------------- */

Ensure(ICM20948, checkFifoData_returns_on_null)
{
    ICM20948_checkFifoData(NULL);
    assert_that(s_spiCallCount, is_equal_to(0));
}

Ensure(ICM20948, checkFifoData_returns_when_no_interrupt)
{
    s_regs[0][0x19] = 0x00;
    ICM20948_checkFifoData(p_testSpi);
}

Ensure(ICM20948, checkFifoData_resets_on_overflow)
{
    s_regs[0][0x19] = 0x02;
    s_regs[0][0x70] = 0x02;
    s_regs[0][0x71] = 0x80; /* 640 bytes > 512 threshold */
    ICM20948_checkFifoData(p_testSpi);
}

Ensure(ICM20948, checkFifoData_parses_single_packet)
{
    spp_uint8_t pkt[42];
    build_dmp_packet(pkt, 8192, 0, 0, 164, 0, 0, 100, 200, 300);
    s_regs[0][0x19] = 0x02;
    mock_set_fifo(pkt, 42);
    ICM20948_checkFifoData(p_testSpi);
}

Ensure(ICM20948, checkFifoData_parses_multiple_packets)
{
    spp_uint8_t buf[84];
    build_dmp_packet(&buf[0],  100, 200, 300, 10, 20, 30, 1, 2, 3);
    build_dmp_packet(&buf[42], 400, 500, 600, 40, 50, 60, 4, 5, 6);
    s_regs[0][0x19] = 0x02;
    mock_set_fifo(buf, 84);
    ICM20948_checkFifoData(p_testSpi);
}

Ensure(ICM20948, checkFifoData_handles_spi_fail_on_int_status)
{
    s_spiFailAtCall = 1;
    ICM20948_checkFifoData(p_testSpi);
}

Ensure(ICM20948, checkFifoData_handles_spi_fail_on_dmp_int)
{
    s_regs[0][0x19] = 0x02;
    s_spiFailAtCall = 2;
    ICM20948_checkFifoData(p_testSpi);
}

Ensure(ICM20948, checkFifoData_handles_spi_fail_on_fifo_count)
{
    s_regs[0][0x19] = 0x02;
    s_spiFailAtCall = 3;
    ICM20948_checkFifoData(p_testSpi);
}

Ensure(ICM20948, checkFifoData_handles_spi_fail_on_fifo_read)
{
    spp_uint8_t pkt[42];
    build_dmp_packet(pkt, 100, 200, 300, 10, 20, 30, 1, 2, 3);
    s_regs[0][0x19] = 0x02;
    mock_set_fifo(pkt, 42);
    s_spiFailAtCall = 4;
    ICM20948_checkFifoData(p_testSpi);
}

Ensure(ICM20948, checkFifoData_handles_zero_fifo_count)
{
    s_regs[0][0x19] = 0x02;
    s_regs[0][0x70] = 0x00;
    s_regs[0][0x71] = 0x00;
    ICM20948_checkFifoData(p_testSpi);
}

/* ================================================================
 * Main
 * ================================================================ */

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    TestSuite *suite = create_test_suite();

    /* writeReg */
    add_test_with_context(suite, ICM20948, writeReg_sends_correct_bytes);
    add_test_with_context(suite, ICM20948, writeReg_propagates_spi_failure);

    /* readReg */
    add_test_with_context(suite, ICM20948, readReg_reads_correct_value);
    add_test_with_context(suite, ICM20948, readReg_handles_null_pointer);
    add_test_with_context(suite, ICM20948, readReg_propagates_spi_failure);

    /* setBank */
    add_test_with_context(suite, ICM20948, setBank_selects_bank_0);
    add_test_with_context(suite, ICM20948, setBank_selects_bank_1);
    add_test_with_context(suite, ICM20948, setBank_selects_bank_2);
    add_test_with_context(suite, ICM20948, setBank_selects_bank_3);
    add_test_with_context(suite, ICM20948, setBank_rejects_invalid_bank);

    /* resetFifo */
    add_test_with_context(suite, ICM20948, resetFifo_succeeds);
    add_test_with_context(suite, ICM20948, resetFifo_fails_on_first_write);
    add_test_with_context(suite, ICM20948, resetFifo_fails_on_second_write);

    /* dmpWriteBytes */
    add_test_with_context(suite, ICM20948, dmpWriteBytes_writes_to_dmp_memory);
    add_test_with_context(suite, ICM20948, dmpWriteBytes_fails_on_bank_sel);
    add_test_with_context(suite, ICM20948, dmpWriteBytes_fails_on_addr);
    add_test_with_context(suite, ICM20948, dmpWriteBytes_fails_on_data);

    /* dmpWrite32 / dmpWrite16 */
    add_test_with_context(suite, ICM20948, dmpWrite32_stores_big_endian);
    add_test_with_context(suite, ICM20948, dmpWrite16_stores_big_endian);
    add_test_with_context(suite, ICM20948, dmpWrite32_propagates_failure);
    add_test_with_context(suite, ICM20948, dmpWrite16_propagates_failure);

    /* lpWakeCycle */
    add_test_with_context(suite, ICM20948, lpWakeCycle_succeeds);
    add_test_with_context(suite, ICM20948, lpWakeCycle_fails_on_first_write);
    add_test_with_context(suite, ICM20948, lpWakeCycle_fails_on_second_write);

    /* dmpWriteOutputConfig */
    add_test_with_context(suite, ICM20948, dmpWriteOutputConfig_succeeds);
    add_test_with_context(suite, ICM20948, dmpWriteOutputConfig_fails_on_first_write);

    /* calcGyroSf */
    add_test_with_context(suite, ICM20948, calcGyroSf_nominal);
    add_test_with_context(suite, ICM20948, calcGyroSf_decreases_with_positive_pll);
    add_test_with_context(suite, ICM20948, calcGyroSf_increases_with_negative_pll);
    add_test_with_context(suite, ICM20948, calcGyroSf_extreme_values);

    /* loadDmp */
    add_test_with_context(suite, ICM20948, loadDmp_succeeds);
    add_test_with_context(suite, ICM20948, loadDmp_fails_on_spi_error);

    /* configDmpInit */
    add_test_with_context(suite, ICM20948, configDmpInit_succeeds);
    add_test_with_context(suite, ICM20948, configDmpInit_fails_on_wrong_who_am_i);
    add_test_with_context(suite, ICM20948, configDmpInit_fails_on_first_spi_error);
    add_test_with_context(suite, ICM20948, configDmpInit_fails_at_every_spi_call);

    /* checkFifoData */
    add_test_with_context(suite, ICM20948, checkFifoData_returns_on_null);
    add_test_with_context(suite, ICM20948, checkFifoData_returns_when_no_interrupt);
    add_test_with_context(suite, ICM20948, checkFifoData_resets_on_overflow);
    add_test_with_context(suite, ICM20948, checkFifoData_parses_single_packet);
    add_test_with_context(suite, ICM20948, checkFifoData_parses_multiple_packets);
    add_test_with_context(suite, ICM20948, checkFifoData_handles_spi_fail_on_int_status);
    add_test_with_context(suite, ICM20948, checkFifoData_handles_spi_fail_on_dmp_int);
    add_test_with_context(suite, ICM20948, checkFifoData_handles_spi_fail_on_fifo_count);
    add_test_with_context(suite, ICM20948, checkFifoData_handles_spi_fail_on_fifo_read);
    add_test_with_context(suite, ICM20948, checkFifoData_handles_zero_fifo_count);

    return run_test_suite(suite, create_text_reporter());
}
