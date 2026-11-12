This is a work in progress. The FSM is where all the pieces we've covered so far — the HAL, the DATABANK, the PUBSUB service — actually get wired together and run, but the flight logic itself is only partially built: right now the FSM can boot the system up and keep it running, but it doesn't yet know how to detect liftoff, apogee, or landing. As that logic gets built, it will show up here as new rows in the transition table below.

Even though the files currently live under services/fsm, the FSM is conceptually part of the SPP core: like the DATABANK and the PUBSUB service, it's mandatory, always-on infrastructure, not an opt-in service — which is why it's grouped here alongside them rather than under Services.


Once the board boots, `app_main()` initializes the HAL, configures the FSM and starts the execution loop:

```c
void app_main(void)
{
    APP_initHal();
    APP_initFsm();
    APP_fsmLoop();
}
```

`APP_initFsm()` initializes the internal FSM table and registers the application-specific transition table.

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

These states and sub-states are defined in the FSM interface, but the current application only implements the transition from INIT to READY. Flight-state transitions remain pending.

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

Every call to `FSM_tick()` iterates through the registered tables in registration order, evaluating rows that match the current state and sub-state:

- **No guard:** The row's `stateFunction` runs, if defined, without changing the current state. Evaluation continues with the remaining rows.
- **Guard returns true:** The FSM updates its current and previous states, executes the optional action and state function, then stops processing the current tick.
- **Guard returns false:** The FSM continues evaluating subsequent rows.

Multiple unguarded rows may execute during a single tick, but at most one guarded transition can occur.

## What the table looks like today

The current implementation uses two transition tables:

**Internal table (`fsm.c`)**

Contains six rows that execute the PUBSUB cycle in the following states:

- READY/NONE
- FLIGHT/NONE
- FLIGHT/ASCENDING
- FLIGHT/DESCENDING
- FLIGHT/PARACHUTE_DEPLOYED
- LANDED/NONE

Each row has no guard and executes `FSM_pubsubCycle()`, which invokes the registered producers and consumers.

**Application table (`custom.c`)**

Currently contains one transition:

`INIT/NONE → READY/NONE`

Its guard, `guard_startInitializationOfModules()`, registers producers and consumers, initializes the SPP core, and checks initialization results.

If registration requirements are satisfied and core initialization succeeds, the FSM transitions to READY and executes `action_emitTelemetry()`, publishing the initialization error flags.

During normal operation, the FSM remains in READY and executes the PUBSUB cycle on every tick. Flight transitions have not yet been implemented.

## Error tracking

Initialization errors are tracked through separate bitfields:

- **Application errors (`FsmErrors_t`):** Track producer and consumer registration and initialization failures.
- **Core errors (`CommonBitErrors_t`):** Track errors encountered during SPP core initialization.
- **FSM internal errors:** Accessible through `SPP_CORE_FSM_getErrorsBit()`.

The application publishes its initialization error flags together with the core error flags through `action_emitTelemetry()`.

A failed initialization guard prevents the transition to READY. Errors do not necessarily terminate execution, but they can prevent normal operation.
