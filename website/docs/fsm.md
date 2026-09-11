This is a work in progress. The FSM is where all the pieces we've covered so far — the HAL, the DATABANK, the PUBSUB service — actually get wired together and run, but the flight logic itself is only partially built: right now the FSM can boot the system up and keep it running, but it doesn't yet know how to detect liftoff, apogee, or landing. As that logic gets built, it will show up here as new rows in the transition table below.

Even though the files currently live under services/fsm, the FSM is conceptually part of the SPP core: like the DATABANK and the PUBSUB service, it's mandatory, always-on infrastructure, not an opt-in service — which is why it's grouped here alongside them rather than under Services.

Once the board boots, app_main hands the HAL ports to FSM_init and then just ticks the FSM forever:

```
void app_main(void)
{
    const SPP_HalPort_t *p_halPorts = SPP_PORTS_ESP32S3_getHalPorts();
    FSM_init((void *)p_halPorts);

    while (1)
    {
        FSM_tick();
    }
}
```

Everything else — initializing the core, registering producers and consumers, running the PUBSUB loop, emitting telemetry — happens because FSM_tick decides it should, tick after tick.

## States and sub-states

The FSM has four top-level states:

- INIT: the board has just booted, nothing is initialized yet.
- READY: the core, the HAL and the services are initialized, waiting to fly.
- FLIGHT: the rocket is airborne.
- LANDED: the flight is over.

FLIGHT is further broken down into sub-states, tracked separately from the top-level state:

- NONE: used by every state that doesn't need a sub-state.
- FLIGHT_ASCENDING
- FLIGHT_DESCENDING
- FLIGHT_PARACHUTE_DEPLOYED

At any point, the FSM's position is a (state, sub-state) pair, held in a handle together with the pair it came from:

```
typedef struct
{
    FSM_State_t state;
    FSM_SubState_t subState;
    FSM_State_t prevState;
    FSM_SubState_t prevSubState;
} FSM_Handle_t;
```

## The transition table

Instead of a big switch statement, the FSM is driven by a table of rows, each one describing a possible move from one (state, sub-state) pair to another:

```
typedef struct
{
    FSM_State_t fromState;
    FSM_SubState_t fromSubState;
    FSM_State_t toState;
    FSM_SubState_t toSubState;
    spp_bool_t (*guard)(void);
    void (*action)(void);
    void (*stateFunction)(void);
} FSM_Transition_t;
```

- fromState / fromSubState: the pair the FSM has to currently be in for this row to apply.
- toState / toSubState: the pair the FSM moves into if the row's guard passes.
- guard: a function that returns true once the FSM is allowed to make this transition — for example, "has every module finished initializing successfully".
- action: runs once, right after the transition happens.
- stateFunction: the ongoing work associated with a state.

Every call to FSM_tick walks the table looking for the first row whose fromState/fromSubState matches where the FSM currently is, and then:

- If that row has no guard, its stateFunction runs immediately and the FSM stays exactly where it is — toState/toSubState are simply not used until a real guard gets attached to that row. This is how the READY row works today: with no guard set, its stateFunction runs on every single tick, without the FSM ever actually moving into FLIGHT. Once a liftoff-detection guard is added to that row, it will start transitioning as intended.
- If the row does have a guard and it returns true, the FSM updates prevState/prevSubState, moves into toState/toSubState, runs action if there is one, then runs stateFunction if there is one.
- If the guard returns false, that row doesn't apply this tick, and the search moves on to the next row.

Only one row is acted on per tick — as soon as a match is found, FSM_tick stops looking at the rest of the table.

## What the table looks like today

Right now there are only two rows:

1. INIT/NONE → READY/NONE, guarded by guard_startInitializationOfModules. This guard initializes the HAL, then calls the private registration function mentioned back in the PUBSUB chapter — the one that registers producers and subscribes consumers to them — and finally initializes the core. If all three succeed, the FSM moves to READY and runs an action that packs the FSM's and the common-bit's error flags into a packet and publishes it, as a first bit of telemetry.
2. READY/NONE, with no guard, running the PUBSUB runtime loop on every tick — calling acquireData on every producer and consumeData on every consumer, exactly as described in the Flow of information section of the PUBSUB chapter.

That second row is where the system spends effectively all of its time today: once initialization succeeds, the FSM just sits in READY and keeps the sensors and consumers running, tick after tick, until FLIGHT, LANDED and the flight sub-states are wired in.

One thing worth flagging precisely because this page is about a moving target: the registration function currently reaches for the antenna consumer through a driver that's mid-migration in the SPP repository (the old E22-MBL01 module is being replaced by a newer SX1262 one). If you're working on the FSM and something in that area doesn't build, that's most likely why — check which antenna driver is actually present under spp/services before assuming your own change broke it.

## Error tracking

Errors picked up while ticking the FSM — a failed init, a failed producer or consumer registration, and so on — are tracked in a bitfield, retrievable at any point through SPP_CORE_FSM_getErrorsBit. Nothing halts execution when a bit gets set; the intent is for these to be surfaced as telemetry, the way action_emitTelemetry already does, rather than to stop the rocket mid-flight because a non-critical service failed to register.
