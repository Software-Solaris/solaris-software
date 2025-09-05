# BMP390 Barometer — C Driver (ESP-IDF, ESP32-S3, SPI 4-Wire)

Robust C driver for the Bosch **BMP390** barometer using **SPI 4-wire** on **ESP-IDF** (target: **ESP32-S3**). The library focuses solely on the barometer implementation (no shared “General” folder), so naming and some blocks can differ from the multi-sensor “fusion” branch; functionality is equivalent.

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
- Data-acquisition pipeline for **temperature** and **pressure**:
  - Read **factory coefficients**, reconstruct with correct endianness/sign/bit-width.
  - Compute **compensation parameters**.
  - Read **raw measurements** and **compensate** them.
- Optional **altitude** computation using the **standard atmosphere** relationship (post-compensation).

---

## Target Platform

- **MCU**: ESP32-S3  
- **SDK**: ESP-IDF (uses `spi_bus_initialize()` and `spi_bus_add_device()` under the hood)  
- **Bus**: SPI 4-wire (CS, SCK, COPI/MOSI, CIPO/MISO)

---

## File Layout

```
bmp390/
├─ bmp390.h   # Macros, types, register defs, and function prototypes
├─ bmp390.c   # SPI init, low-level R/W, configuration & measurement pipeline
└─ main.c     # Example: end-to-end init + read loop + altitude computation
```

- `bmp390.h` groups declarations by functional areas:
  - **INIT SPI**: SPI pin selection (CS, SCK, COPI, CIPO) + init prototype.
  - **AUX**: Auxiliary register read/write prototypes.
  - **CONFIG & CHECK**: Register addresses/values + configuration and verification prototypes.
  - **PREPARE READ**: Controls (mode, oversampling, ODR, IIR) + accessors.
  - **READ TEMP / READ PRESS**: Structures/macros/prototypes for raw/coefficient reads and compensation.

---

## Driver Architecture

### 1) SPI Initialization

**Function:** `bmp390_init()`  
- **Bus bring-up:** Configure structures and call **`spi_bus_initialize()`**.  
- **Device attach:** Configure SPI device parameters and add it via **`spi_bus_add_device()`**.  
- **Important:** The implementation assumes a sensor-specific SPI profile; the **first returned byte on transactions is discarded** (electrical/noise behavior), so dummy bits/flags must match the designed R/W routines. **Review these settings** if you alter the SPI config.

### 2) Aux I/O Primitives

Low-level **register read/write** helpers shared by configuration and data path.  
- Use bitwise operations to assemble commands/addresses.  
- Implement transfers with ESP-IDF SPI transactions (one path uses **polling**, another uses **transmit**).

### 3) Configuration & Sanity Checks

Power-on sequence:
1. **Soft reset** via register write (`bmp390_soft_reset`).  
2. **Enable SPI 4-wire** (`bmp390_enable_spi`) — **critical step** because the device boots in **I²C**; skipping this yields errors.  
3. **Read-back checks**:
   - `read_if_conf`: verify selected bus mode (I²C vs SPI).  
   - `read_chip_id`: confirm **BMP390** silicon.

### 4) Read Preparation

Pre-acquisition configuration (via aux write helper):  
- **Mode**: e.g., **normal** for continuous sampling.  
- **Oversampling**: per dynamics/control constraints.  
- **ODR** (Output Data Rate).  
- **IIR filter**: to trade bandwidth vs noise.  

During the loop:
- Poll **status** register; `bmp390_wait_temp_ready()` / `bmp390_wait_press_ready()` block until **DRDY=1** before fetching the next sample.

### 5) Temperature Path

- `bmp390_read_raw_temp_coeffs()`: bulk-read factory **temperature coefficients**; **little-endian** unpacking (LSB first), reconstruct **signed/unsigned** fields with their exact bit-widths into a dedicated struct.  
- `bmp390_calibrate_temp_params()`: compute compensation parameters from raw coeffs as per vendor formulas.  
- `bmp390_read_raw_temp()`: read raw temperature registers and combine into an **uncalibrated °C** value.  
- `bmp390_compensate_temperature()`: apply the equations using calibrated parameters to output **compensated °C**.

### 6) Pressure Path

Follows the **same stages** as temperature but with the **pressure-specific** register map and formulas (see device datasheet for coefficient layout and equations).

---

## Example Usage Flow (`main.c`)

1. **Declarations** (ideally move common symbols to `bmp390.h` in multi-sensor builds).  
2. **INIT**: bring up SPI.  
3. **CONFIG & CHECK**: soft-reset, **enable SPI 4-wire**, verify **IF_CONF** and **CHIP_ID**.  
4. **PREPARE READ**: set **mode/oversampling/ODR/IIR**; read & calibrate **coefficients** before entering the acquisition loop.  
5. **READ loop**:
   - Wait for **DRDY**, fetch **raw T/P**, compensate **T/P**.  
   - Optionally compute **altitude** from compensated pressure using the **standard atmosphere** relation.

```c
// Pseudocode sketch
void app_main(void) {
    bmp390_init();                       // SPI bus + device
    bmp390_soft_reset();
    bmp390_enable_spi();                 // switch from default I2C to SPI 4-wire

    // configure mode/OSR/ODR/IIR
    bmp390_config_mode_normal();
    bmp390_config_osr(...);
    bmp390_config_odr(...);
    bmp390_config_iir(...);

    // read & calibrate coefficients
    bmp390_read_raw_temp_coeffs(&tcoeffs);
    bmp390_calibrate_temp_params(&tcoeffs, &tparams);
    // (pressure path analogous)

    while (1) {
        bmp390_wait_temp_ready();
        bmp390_wait_press_ready();

        int32_t t_raw = bmp390_read_raw_temp();
        float   t_c   = bmp390_compensate_temperature(t_raw, &tparams);

        // pressure: read raw, compensate (same pattern)

        // altitude: standard atmosphere using compensated pressure (optional)
    }
}
```

---

## Operational Notes & Caveats

- **SPI mode is mandatory**: the BMP390 powers up in **I²C**; you **must** set **SPI 4-wire** before any SPI transfers will behave correctly.  
- **Discard first byte** on SPI reads if your electrical setup mandates it; the driver is designed with dummy cycles/flags to drop that initial **garbage** byte. If you change SPI timing/flags, **audit the low-level R/W** paths.  
- **Endianness matters**: coefficient blocks are **little-endian**; reconstruct fields with the **exact sign/bit-width**.  
- **Throughput vs noise**: tune **OSR/ODR/IIR** to meet control deadlines and noise targets.

---

## Authorship

- **Author**: Óscar Barja Lorenzo  
- **Reviewers**: —  
- **Subsystem**: Electronics  
- **Created**: 2025-08-13  
- **Last Modified**: —
