# Log service

Level-filtered logging service. Provides `SPP_LOG*` macros that route output through a swappable callback function. No default output is set — call `SPP_SERVICES_LOG_setOutput()` to wire up a handler, or use `SPP_CORE_boot()` which automatically bridges log output to the pub/sub bus.

---

## Files

| File | Description |
|---|---|
| `log.h` | Public API, log macros, `SPP_LogLevel_t` |
| `log.c` | Implementation |

---

## Log levels

```c
K_SPP_LOG_NONE    = 0   // Silence all output
K_SPP_LOG_ERROR   = 1   // Critical errors only
K_SPP_LOG_WARN    = 2   // Warnings and errors
K_SPP_LOG_INFO    = 3   // Normal operational messages
K_SPP_LOG_DEBUG   = 4   // Detailed debug output
K_SPP_LOG_VERBOSE = 5   // Full trace (default after init)
```

Messages with a level **higher** than the current filter are suppressed.

---

## API

```c
SPP_RetVal_t    SPP_SERVICES_LOG_init(void);
void            SPP_SERVICES_LOG_setLevel(SPP_LogLevel_t level);
SPP_LogLevel_t  SPP_SERVICES_LOG_getLevel(void);
void            SPP_SERVICES_LOG_setOutput(SPP_LogOutputFn_t fn);
```

---

## Macros

```c
SPP_LOGE("TAG", "SPI error %d", ret);    // error
SPP_LOGW("TAG", "pool at %d%%", pct);    // warning
SPP_LOGI("TAG", "service started");      // info
SPP_LOGD("TAG", "seq=%u ts=%u", s, t);  // debug
SPP_LOGV("TAG", "raw bytes: %02X", b);  // verbose
```

The tag is a short string identifying the module — use a module-level `const char *`:

```c
static const char *const k_tag = "BMP390";
SPP_LOGI(k_tag, "altitude = %.2f m", altitude);
```

---

## Log → pub/sub bridge

`SPP_CORE_boot()` automatically installs a log output function that formats each `SPP_LOG*` call as a `K_SPP_APID_LOG` packet and publishes it on the bus. This lets the SD card logger capture log messages alongside sensor data without any extra setup.

A reentrancy guard (`s_logBusy`) prevents infinite recursion: if a subscriber itself calls `SPP_LOGE()`, the nested call is silently dropped.

---

## Custom output callback

```c
// Redirect to UART on embedded target
static void uartLogOutput(const char *p_tag, SPP_LogLevel_t level,
                           const char *p_msg)
{
    uart_write_bytes(UART_NUM_0, p_msg, strlen(p_msg));
    uart_write_bytes(UART_NUM_0, "\n", 1);
}

SPP_SERVICES_LOG_setOutput(uartLogOutput);
```

Pass NULL to disable log output.
