This chapter is the detailed design part of the documentation: one page per component, each one explaining what it's for, how it behaves, and how it talks to the others.

This page is the map for that chapter. If you've just opened main.c for the first time and you're wondering where to go next, start here.

## Where the whole thing starts

main.c is short:

```

void app_main(void)
{
    APP_initHal();
    APP_initFsm();
    APP_fsmLoop();
}

```

The application initializes the HAL, configures the FSM and enters a cooperative execution loop. Each iteration calls `FSM_tick()`, which determines the operations to execute according to the current state.

## The boot sequence, tick by tick

Reading the chapters in this order follows what happens on the board, from power-on to steady state:

1. **Finite State Machine.** `APP_initFsm()` initializes the FSM's internal transition table and registers the application's custom table. On the first tick, the INIT guard registers producers and consumers, then initializes the SPP core. If successful, the FSM transitions to READY, where it remains during normal operation.
2. **HAL.** `APP_initHal()` initializes the hardware abstraction before the FSM starts executing. The HAL provides platform-independent interfaces to the hardware.
3. **Publish-Subscribe (PUBSUB).** The FSM executes the PUBSUB cycle in READY, invoking registered producers and consumers. Published packets are buffered, with one packet dispatched to subscribed consumers per cycle.
4. **Solaris Packet.** Every time a producer publishes something, what moves through the system is one of these — a fixed 64-byte structure. This chapter describes what's inside one.
5. **Databank.** Packets aren't allocated on the fly — there's no dynamic memory in this codebase. This chapter explains the fixed-size pool they're borrowed from and returned to.

Reading those five in order covers how the system boots, how data moves through it, what that data looks like, and where it's stored.

## If you want to make a change

The chapters above explain how things work. This is about where to make a given change, depending on what you're trying to do.

- **Add a new sensor (a producer).** Implement the producer contract from the PUBSUB chapter in a new spp/services/<name>/ folder, register it through CUSTOM_registerConsumerProducer() in main/custom.c alongside the existing producers, and add it to the build following the Build System chapter so it actually gets compiled in.
- **Add a new consumer** — logging somewhere new, a new radio, anything that reacts to data. Same idea: implement the consumer contract, register it with the appropriate KPID subscription bitmask, wire it into the build.
- **Change what's inside a packet, or add a new field.** Start at the Solaris Packet chapter. The packet is a fixed 64 bytes end to end, and every existing consumer already assumes that layout, so this affects all of them — check what reads the fields you're changing before you touch them.
- **Change how many packets can be in flight at once.** That's K_SPP_DATABANK_SIZE, covered in the Databank chapter.
- **Add real flight logic** — detecting liftoff, apogee, parachute deployment, landing. That's the FSM's transition table. Read the Finite State Machine chapter first: it explains exactly what's missing today and why.
- **Remove something to make the binary smaller, or port to a new board.** That's the Build System and Repository structure chapters, under Getting Started.

## Ground rules, wherever you're working

A few constraints apply across every chapter in this section, not just to one component:

- No dynamic memory allocation, anywhere. Producers and consumers borrow packets from the DATABANK's static pool; they never malloc.
- Everything runs bare-metal, on a single core, with no preemption. A consumer's consumeData must never block — no polling, no busy-waits — because there's no mechanism to reclaim CPU time from it if it does.
- Every producer and consumer must follow its contract exactly (see the PUBSUB chapter). The PUBSUB service calls these function pointers unconditionally, so a missing or misbehaving one breaks the loop for every other module too, not just its own.
