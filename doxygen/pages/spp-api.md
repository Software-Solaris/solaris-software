# SPP API — a short tour {#spp-api}

A quick orientation to the C API. The full reference — every file, struct,
function and macro — is under <a href="files.html">Files</a> and
<a href="annotated.html">Data Structures</a>.


The application initializes the HAL, configures the FSM and enters the main execution loop.

```c
void app_main(void)
{
    APP_initHal();
    APP_initFsm();
    APP_fsmLoop();
}
```


Everything else — sensor sampling, packet routing, logging, downlink — is
driven from the FSM (@ref fsm.h) through the publish/subscribe core
(@ref pubsub.h).
