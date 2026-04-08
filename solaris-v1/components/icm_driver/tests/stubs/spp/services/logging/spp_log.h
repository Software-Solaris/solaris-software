/*
 * stubs/spp/services/logging/spp_log.h
 *
 * Captures SPP log output into a global buffer so BDD tests can verify
 * the exact string values emitted by icm20948.c (e.g., converted sensor
 * readings).  Each call to SPP_LOGI appends one entry; g_logCallCount
 * tracks how many entries have been written.
 *
 * The storage arrays are DEFINED in mock_common.h (one per test binary)
 * and DECLARED here so that icm20948.c can use the macros.
 */
#ifndef STUB_SPP_LOG_H
#define STUB_SPP_LOG_H

#include <stdio.h>

#ifndef SPP_LOG_MAX_ENTRIES
#define SPP_LOG_MAX_ENTRIES 32
#endif
#ifndef SPP_LOG_ENTRY_SIZE
#define SPP_LOG_ENTRY_SIZE  256
#endif

/* Defined in mock_common.h — one definition per test binary. */
extern char g_logEntries[SPP_LOG_MAX_ENTRIES][SPP_LOG_ENTRY_SIZE];
extern int  g_logCallCount;

/* Capture the formatted message; silently drop entries beyond the buffer. */
#define SPP_LOGI(tag, fmt, ...)                                         \
    do {                                                                \
        if (g_logCallCount < SPP_LOG_MAX_ENTRIES) {                     \
            snprintf(g_logEntries[g_logCallCount],                       \
                     SPP_LOG_ENTRY_SIZE, fmt, ##__VA_ARGS__);           \
            g_logCallCount++;                                           \
        }                                                               \
    } while (0)

#define SPP_LOGE(tag, fmt, ...) SPP_LOGI(tag, fmt, ##__VA_ARGS__)
#define SPP_LOGW(tag, fmt, ...) SPP_LOGI(tag, fmt, ##__VA_ARGS__)
#define SPP_LOGD(tag, fmt, ...) SPP_LOGI(tag, fmt, ##__VA_ARGS__)

#endif /* STUB_SPP_LOG_H */
