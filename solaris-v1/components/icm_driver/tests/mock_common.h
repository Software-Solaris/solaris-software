/*
 * mock_common.h — Shared mock infrastructure for ICM20948 BDD unit tests.
 *
 * Include this header ONCE at the top of each test .c file, before the
 * #include "icm20948.c" line.  Each test binary gets its own copy of the
 * mock state because each .c file is compiled into a separate executable.
 *
 * What is provided:
 *   - Register-file model   : 4 banks × 128 registers (s_regs)
 *   - DMP SRAM model        : 16 KB (s_dmpMem) with optional read corruption
 *   - FIFO model            : 4 KB ring (s_fifo / s_fifoSize / s_fifoReadIdx)
 *   - SPI call tracking     : s_spiCallCount, s_spiFailAtCall, s_txHist*
 *   - Delay tracking        : s_delayCallCount
 *   - Log capture           : g_logEntries / g_logCallCount (used by spp_log.h)
 *   - mock_reset()          : resets all state; pre-loads WHO_AM_I = 0xEA
 *   - mock_set_fifo()       : populates the FIFO model for FIFO-parse tests
 *   - SPP_HAL_SPI_Transmit  : mock SPI implementation
 *   - SPP_OSAL_TaskDelay    : mock delay implementation
 *   - p_testSpi             : a valid non-null handle for test calls
 */
#ifndef MOCK_COMMON_H
#define MOCK_COMMON_H

#include "spp/core/types.h"
#include "spp/core/returntypes.h"
#include <cgreen/cgreen.h>
#include <string.h>
#include <stdio.h>

/* ================================================================
 * Sizes
 * ================================================================ */
#define MOCK_DMP_MEM_SIZE   16384U
#define MOCK_FIFO_SIZE      4096U
#define MOCK_TX_HIST_SIZE   16U

/* ================================================================
 * Log-capture sizes and storage
 * (SPP_LOG_MAX_ENTRIES / SPP_LOG_ENTRY_SIZE are also used by spp_log.h;
 *  define them here first so the array dimension is always known)
 * ================================================================ */
#ifndef SPP_LOG_MAX_ENTRIES
#define SPP_LOG_MAX_ENTRIES 32
#endif
#ifndef SPP_LOG_ENTRY_SIZE
#define SPP_LOG_ENTRY_SIZE  256
#endif

char g_logEntries[SPP_LOG_MAX_ENTRIES][SPP_LOG_ENTRY_SIZE];
int  g_logCallCount;

/* ================================================================
 * SPI call counters
 * ================================================================ */
static int s_spiCallCount;
static int s_spiFailAtCall; /* -1 = never inject a failure */
static int s_delayCallCount;

/* Transaction history — last MOCK_TX_HIST_SIZE register writes */
static spp_uint8_t s_txHistReg[MOCK_TX_HIST_SIZE];
static spp_uint8_t s_txHistVal[MOCK_TX_HIST_SIZE];
static int         s_txHistCount;

/* ================================================================
 * Register model: 4 banks × 128 registers
 * ================================================================ */
static spp_uint8_t s_regs[4][128];
static spp_uint8_t s_currentBank;

/* ================================================================
 * DMP SRAM model
 * ================================================================ */
static spp_uint8_t  s_dmpMem[MOCK_DMP_MEM_SIZE];
static spp_uint8_t  s_dmpBank;
static spp_uint8_t  s_dmpAddr;

/*
 * Optional DMP read-corruption: when the mock reads back DMP byte at
 * s_dmpCorruptAddr, it returns s_dmpCorruptVal instead of the stored
 * value.  Set s_dmpCorruptAddr = 0xFFFFU to disable.
 */
static spp_uint16_t s_dmpCorruptAddr;
static spp_uint8_t  s_dmpCorruptVal;

/* ================================================================
 * FIFO model
 * ================================================================ */
static spp_uint8_t s_fifo[MOCK_FIFO_SIZE];
static int         s_fifoSize;
static int         s_fifoReadIdx;

/* ================================================================
 * Shared SPI handle — a non-null pointer accepted by all functions
 * ================================================================ */
static int  s_dummySpiHandle = 1;
static void *p_testSpi = &s_dummySpiHandle;

/* ================================================================
 * mock_reset — call from BeforeEach to start every test clean
 * ================================================================ */
static void mock_reset(void)
{
    s_spiCallCount   = 0;
    s_spiFailAtCall  = -1;
    s_delayCallCount = 0;
    s_currentBank    = 0U;
    s_dmpBank        = 0U;
    s_dmpAddr        = 0U;
    s_fifoSize       = 0;
    s_fifoReadIdx    = 0;
    s_txHistCount    = 0;
    s_dmpCorruptAddr = 0xFFFFU;
    s_dmpCorruptVal  = 0U;
    g_logCallCount   = 0;

    memset(s_txHistReg,   0, sizeof(s_txHistReg));
    memset(s_txHistVal,   0, sizeof(s_txHistVal));
    memset(s_regs,        0, sizeof(s_regs));
    memset(s_dmpMem,      0, sizeof(s_dmpMem));
    memset(s_fifo,        0, sizeof(s_fifo));
    memset(g_logEntries,  0, sizeof(g_logEntries));

    /* Pre-load WHO_AM_I so init tests pass by default. */
    s_regs[0][0x00] = 0xEAU;
}

/* ================================================================
 * mock_set_fifo — populate FIFO model and set FIFO_COUNT registers
 * ================================================================ */
static void mock_set_fifo(const spp_uint8_t *p_data, int len)
{
    int copy = (len < (int)MOCK_FIFO_SIZE) ? len : (int)MOCK_FIFO_SIZE;
    memcpy(s_fifo, p_data, (size_t)copy);
    s_fifoSize    = copy;
    s_fifoReadIdx = 0;

    /* FIFO_COUNTH (0x70) and FIFO_COUNTL (0x71) — bank 0 */
    s_regs[0][0x70] = (spp_uint8_t)((copy >> 8) & 0xFFU);
    s_regs[0][0x71] = (spp_uint8_t)(copy & 0xFFU);
}

/* ================================================================
 * SPP_HAL_SPI_Transmit — mock SPI bus
 *
 * Behaviour mirrors the real ICM20948 SPI protocol:
 *   - Byte[0] bit7 = 0 → write; bit7 = 1 → read
 *   - Byte[0] bits[6:0] = register address
 *   - Special registers: BANK_SEL, MEM_BANK_SEL, MEM_START_ADDR, MEM_R_W, FIFO_R_W
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
    int         isRead = ((p_data[0] & 0x80U) != 0U);

    if (isRead)
    {
        if (reg == 0x72U && len > 1U) /* FIFO_R_W */
        {
            for (int i = 1; i < (int)len; i++)
            {
                p_data[i] = (s_fifoReadIdx < s_fifoSize)
                            ? s_fifo[s_fifoReadIdx++]
                            : 0U;
            }
        }
        else if (reg == 0x7DU) /* MEM_R_W read */
        {
            spp_uint16_t addr = ((spp_uint16_t)s_dmpBank << 8) | s_dmpAddr;
            spp_uint8_t  val  = (addr < MOCK_DMP_MEM_SIZE) ? s_dmpMem[addr] : 0U;

            /* Optionally corrupt one address to simulate readback mismatch */
            if (addr == s_dmpCorruptAddr)
            {
                val = s_dmpCorruptVal;
            }

            p_data[1] = val;
            s_dmpAddr++;
        }
        else /* Normal register read (may be multi-byte) */
        {
            for (int i = 1; i < (int)len; i++)
            {
                spp_uint8_t r = (spp_uint8_t)(reg + (spp_uint8_t)(i - 1));
                if (r < 128U)
                {
                    p_data[i] = s_regs[s_currentBank][r];
                }
            }
        }
    }
    else /* Write */
    {
        spp_uint8_t val = (len >= 2U) ? p_data[1] : 0U;

        /* Record in history for assertion in tests */
        if (s_txHistCount < (int)MOCK_TX_HIST_SIZE)
        {
            s_txHistReg[s_txHistCount] = reg;
            s_txHistVal[s_txHistCount] = val;
            s_txHistCount++;
        }

        if (reg == 0x7FU && len >= 2U)      /* REG_BANK_SEL */
        {
            s_currentBank = (val >> 4) & 0x03U;
        }
        else if (reg == 0x7EU && len >= 2U) /* MEM_BANK_SEL */
        {
            s_dmpBank = val;
        }
        else if (reg == 0x7CU && len >= 2U) /* MEM_START_ADDR */
        {
            s_dmpAddr = val;
        }
        else if (reg == 0x7DU && len >= 2U) /* MEM_R_W write */
        {
            spp_uint16_t addr = ((spp_uint16_t)s_dmpBank << 8) | s_dmpAddr;
            if (addr < MOCK_DMP_MEM_SIZE)
            {
                s_dmpMem[addr] = val;
            }
            s_dmpAddr++;
        }
        else if (reg < 128U && len >= 2U) /* Normal register write */
        {
            s_regs[s_currentBank][reg] = val;
        }
    }

    return SPP_OK;
}

/* ================================================================
 * SPP_OSAL_TaskDelay — mock delay (counts calls, does not block)
 * ================================================================ */
void SPP_OSAL_TaskDelay(spp_uint32_t ms)
{
    (void)ms;
    s_delayCallCount++;
}

#endif /* MOCK_COMMON_H */
