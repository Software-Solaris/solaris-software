# Databank service

Static packet pool. Maintains a fixed array of `SPP_Packet_t` objects and a free-list stack. Producers call `SPP_SERVICES_DATABANK_getPacket()` to lease a packet, fill it with `SPP_SERVICES_DATABANK_packetData()`, and publish it. After all pub/sub subscribers have processed it, `SPP_SERVICES_PUBSUB_publish()` automatically returns it to the pool. `malloc` is never called.

Pool size is controlled by `K_SPP_DATABANK_SIZE` (default **5**, configurable via CMake).

!!! note
    For the conceptual explanation of what the databank is and why it exists, see [Databank](../databank.md). This page is the API reference for the service itself.

---

## Files

| File | Description |
|---|---|
| `databank.h` | Public API |
| `databank.c` | Implementation |

---

## API

```c
SPP_RetVal_t   SPP_SERVICES_DATABANK_init(void);
SPP_Packet_t  *SPP_SERVICES_DATABANK_getPacket(void);
SPP_RetVal_t   SPP_SERVICES_DATABANK_returnPacket(SPP_Packet_t *p_packet);
SPP_RetVal_t   SPP_SERVICES_DATABANK_packetData(SPP_Packet_t *p_packet,
                                        spp_uint16_t apid,
                                        spp_uint16_t seq,
                                        const void  *p_data,
                                        spp_uint16_t dataLen);
spp_uint32_t   SPP_SERVICES_DATABANK_freeCount(void);
```

---

## `SPP_SERVICES_DATABANK_packetData()`

Fills all packet fields in one call:
- Zeroes the full packet struct (ensures deterministic CRC over padding bytes)
- Sets `primaryHeader`: version, apid, seq, payloadLen
- Sets `secondaryHeader.timestampMs` from `SPP_HAL_TIME_getTimeMs()`
- Copies `dataLen` bytes from `p_data` into `payload`
- Computes CRC-16/CCITT over the full packet minus the CRC field and stores it in `p_packet->crc`

---

## Usage

```c
// Producer (inside a ServiceTask)
SPP_Packet_t *p_pkt = SPP_SERVICES_DATABANK_getPacket();
if (p_pkt == NULL)
{
    SPP_LOGW("MY_SVC", "pool empty, dropping reading");
    return;
}

float data[3] = { altitude, pressure, temperature };
(void)SPP_SERVICES_DATABANK_packetData(p_pkt, K_MY_APID, s_seq++,
                               data, (spp_uint16_t)sizeof(data));
(void)SPP_SERVICES_PUBSUB_publish(p_pkt);
// p_pkt is returned to the pool automatically by SPP_SERVICES_PUBSUB_publish()
```

---

## Rules

- Never call `returnPacket` twice on the same pointer — the double-return guard will return `K_SPP_ERROR_ALREADY_INITIALIZED`.
- Never use a packet after calling `publish()` or `returnPacket()` — the memory may be reused immediately.
- If `getPacket` returns NULL, the pool is exhausted. Either increase `K_SPP_DATABANK_SIZE` or ensure subscribers complete quickly.
- `init` is idempotent — calling it twice returns `K_SPP_ERROR_ALREADY_INITIALIZED` and is harmless.
