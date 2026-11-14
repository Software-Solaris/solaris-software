# Services

Solaris Software uses SPP services to acquire sensor measurements, transmit telemetry, store packets and provide shared infrastructure. Producers publish packets through PUBSUB; consumers subscribe to the packet types they need.

## Producers

| Service | Description |
|---|---|
| [BMP390](bmp390.md) | Barometric pressure, temperature and relative altitude acquisition. |
| [ICM20948](icm20948.md) | Inertial measurements and orientation data. |

## Consumers

| Service | Description |
|---|---|
| [SX1262](sx1262.md) | LoRa radio transmission of SPP telemetry packets. |
| [Datalogger](datalogger.md) | Binary packet logging to an SD card. |

## Supporting services

| Service | Description |
|---|---|
| [Databank](databank.md) | Static packet allocation, preparation and recycling. |
| [Log](log.md) | Level-filtered diagnostic logging. |

The application registers the required producers and consumers at initialization. The Databank and Log services provide supporting functionality rather than behaving as sensor producers or telemetry consumers.

For packet distribution and service contracts, see [Publish–Subscribe (PUBSUB)](../pubsub.md).
