# Roadmap

## How we got here

- **October 2024:** Solaris Software repository created. Early development focused on ESP32-S3 tooling and individual sensor drivers.
- **2025:** Development of the Solaris Packet Protocol (SPP) began, bringing packet-based communication and a modular architecture to the firmware.
- **June 2026:** **Solaris Software v1.0.0** established the first stable SPP-based firmware, integrating sensors, publish-subscribe communication and data logging.
- **September 2026:** Solaris Software and SPP were presented at **New Space España 2026**. The architecture poster ranked **4th out of 12** submissions, providing an opportunity to discuss the project with aerospace industry professionals.

## November 2026 — SURTAM L1

Solaris won the **Go for Launch Award by PLD Space** at the first Spanish university rocketry competition, flying **Viradinha MkII** to an apogee of **426.6 m**.

The flight validated **SPP running on real flight hardware**, including complete onboard data logging and in-flight radio telemetry. Radio transmissions were interrupted at approximately **140 m** when an improvised 3.3 V Dupont connection in the additional telemetry hardware partially disconnected. This was a hardware integration issue, not an identified SPP software failure.

### Solaris Software v2.0.0
**Solaris Software v2.0.0** was released as the firmware version flown on Viradinha MkII, incorporating:

- Modular FSM and improved PUBSUB architecture.
- Expanded HAL and platform ports.
- Sensor acquisition, binary SD card logging and SX1262 radio telemetry.
- Updated source documentation and a complete release archive including the matching SPP submodule revision.

The release preserves the flight-tested baseline for future development.

## EuRoC 2027 — Development roadmap

- **Platform migration:** Port Solaris Software and SPP to the selected **RP2350B** platform using the **Pico SDK**, keeping hardware dependencies behind SPP ports.
- **Efficient I/O:** Evaluate **DMA** and programmable **PIO** to reduce CPU overhead in acquisition and communications, based on measured benefits.
- **Flight supervision:** Develop the rocket's **flight-state machine**, **FDIR** mechanisms and watchdog supervision, with defined responses to faults.
- **Aerobrake control:** Integrate the control algorithm with predictable timing; evaluate dedicating the second core after measuring latency, jitter and resource contention. Keep SPP outside the control-critical path.
- **Subsystem integration:** Define and validate software interfaces for avionics, telemetry, aerobrake actuation and recovery-related commands.
- **Verification:** Test worst-case execution times, memory use, communication failures and fault recovery on the target hardware before flight.

These are planned development objectives. Architecture choices still under evaluation will be finalized through implementation and hardware testing.
