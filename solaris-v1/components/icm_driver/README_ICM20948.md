# ICM20948 IMU — C Driver (ESP-IDF, ESP32-S3, SPI 4-Wire)

Robust C driver for the TDK InvenSense **ICM20948** IMU using **SPI 4-wire** on **ESP-IDF** (target: **ESP32-S3**). The library focuses solely on the IMU implementation (no shared “General” folder), so naming and some blocks can differ from the multi-sensor “fusion” branch; functionality is equivalent
Link to the **ESP-IDF**: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html

---

## Contents
- [Overview](#overview)
- [Target Platform](#target-platform)
- [File Layout](#file-layout)
- [Driver Architecture](#driver-architecture)
  - [1) SPI Initialization](#1-spi-initialization)
  - [2) Aux I/O Primitives](#2-aux-io-primitives)
  - [3) Configuration & Sanity Checks](#3-configuration--sanity-checks)
  - [4) Read Preparation](#4-read-preparation)
  - [5) Temperature Path](#5-temperature-path)
  - [6) Pressure Path](#6-pressure-path)
- [Example Usage Flow (`main.c`)](#example-usage-flow-mainc)
- [Operational Notes & Caveats](#operational-notes--caveats)
- [Authorship](#authorship)

---

## Overview

This driver provides:
- SPI bus/device bring-up via ESP-IDF.
- Register-level read/write helpers.
- Power-on configuration sequence, including **soft reset** and **SPI 4-wire enable** (device ships in **I²C** by default).
- Data-acquisition pipeline for **acceleration**, **angular velocity** and **magnetic fields**:
  - Read **factory coefficients**.
  - Compute **compensation parameters**, test your own esp32s3 for setting your compensation values on each 9-axis.
  - Read **raw measurements** and **compensate** them.

---

## Target Platform

- **MCU**: ESP32-S3  
- **SDK**: ESP-IDF (uses `spi_bus_initialize()` and `spi_bus_add_device()` under the hood)  
- **Bus**: SPI 4-wire (CS, SCK, COPI/MOSI, CIPO/MISO)

---

## File Layout

```
components
├── general
│   └── macros.h       # The main struct data_t for spi configuration
├── icm_driver         
│   ├── icm20948.h
│   ├── icm20948.c
└── main
    └── main.c
```

---

## Driver Architecture

### 1) SPI Initialization

**Function:** `icm20948_init()`  
- **Bus bring-up:** Configure structures and call **`spi_bus_initialize()`**.  
- **Device attach:** Configure SPI device parameters and add it via **`spi_bus_add_device()`**.  
- **Important:** The implementation assumes a sensor-specific SPI profile.

### 2) Aux I/O Primitives

Low-level **send_message()** helper simplifies the access to icm20948 registers.  
- Use bitwise operations to assemble commands/addresses.  
- Implement transfers with ESP-IDF SPI transactions.

### 3) Configuration & Sanity Checks

Power-on sequence:

1. **Soft reset** via writing on PWR_MGMT_1.
2. **Wake up the sensor from 'sleep mode'**.
3. **Check WHO_AM_I** register (`0xEA`) to verify the device is indeed **ICM-20948**.
4. Set up of used elements from ICM on the USER_CTRL register.


### 4) Read Preparation

- **Calibration** of accelerometer and gyroscope using ACCEL_CONFIG and GYRO_CONFIG.

## Example Usage Flow (`main.c`)

1. **Declarations**
2. **INIT**: bring up SPI.  
3. **CONFIG & CHECK**: soft-reset, **enable SPI 4-wire**, verify **WHO_AM_I**.  
4. **READ loop**:

```c
// Pseudocode sketch
void app_main(void) {
    icm20948_init();                       // SPI bus + device
    icm20948_config();
    icm20948_prepare_read();

    while (1) {
        icm20948_read();
    }
}
```
---

## Authorship

- **Author**: David Rodríguez Dagas
- **Reviewers**: —  
- **Subsystem**: Electronics  
- **Created**: 2025-10-16  
- **Last Modified**: —
