# SPP API — a short tour {#spp-api}

A quick orientation to the C API. The full reference — every file, struct,
function and macro — is under <a href="files.html">Files</a> and
<a href="annotated.html">Data Structures</a>.

The application is deliberately tiny: it wires the ESP32-S3 HAL port into the
finite state machine and then ticks it forever.

```c
void app_main(void)
{
    const SPP_HalPort_t *p_halPorts = SPP_PORTS_ESP32S3_getHalPorts();
    (void)FSM_init((void *)p_halPorts);
    while (1) {
        FSM_tick();
    }
}
```

Everything else — sensor sampling, packet routing, logging, downlink — is
driven from the FSM (@ref fsm.h) through the publish/subscribe core
(@ref pubsub.h).
