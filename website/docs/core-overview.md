This chapter is the detailed design part of the documentation: one page per component, each one explaining what it's for, how it behaves, and how it talks to the others.

This page is the map for that chapter. If you've just opened main.c for the first time and you're wondering where to go next, start here.

## Where the whole thing starts

main.c is short:

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

That while loop is where everything happens. There's no scheduler and no tasks — FSM_tick decides what runs on every iteration. So the actual starting point for understanding the system isn't main.c, it's the Finite State Machine chapter.

## The boot sequence, tick by tick

Reading the chapters in this order follows what happens on the board, from power-on to steady state:

1. **Finite State Machine.** The first tick matches the FSM's INIT row, whose guard initializes the HAL, registers every producer and consumer, and initializes the core. If that succeeds, the FSM moves to READY and stays there — see this chapter for the full transition table, and for why "stays there" is the accurate description right now instead of "moves on to FLIGHT".
2. **HAL** (in the architectural design section). The first thing that guard does is call SPP_HAL_init. This chapter explains why producers and consumers never touch a register directly — they always go through a function pointer that gets wired up to a board-specific implementation at startup.
3. **Publish-Subscribe (PUBSUB).** Registering producers and consumers, and the runtime loop that calls acquireData and consumeData every tick, is covered here.
4. **Solaris Packet.** Every time a producer publishes something, what moves through the system is one of these — a fixed 64-byte structure. This chapter describes what's inside one.
5. **Databank.** Packets aren't allocated on the fly — there's no dynamic memory in this codebase. This chapter explains the fixed-size pool they're borrowed from and returned to.

Reading those five in order covers how the system boots, how data moves through it, what that data looks like, and where it's stored.

## If you want to make a change

The chapters above explain how things work. This is about where to make a given change, depending on what you're trying to do.

- **Add a new sensor (a producer).** Implement the producer contract from the PUBSUB chapter in a new spp/services/<name>/ folder, register it in the FSM's registration function alongside the existing producers, and add it to the build following the Build System chapter so it actually gets compiled in.
- **Add a new consumer** — logging somewhere new, a new radio, anything that reacts to data. Same idea: implement the consumer contract, register it with the right APID subscription, wire it into the build.
- **Change what's inside a packet, or add a new field.** Start at the Solaris Packet chapter. The packet is a fixed 64 bytes end to end, and every existing consumer already assumes that layout, so this affects all of them — check what reads the fields you're changing before you touch them.
- **Change how many packets can be in flight at once.** That's K_SPP_DATABANK_SIZE, covered in the Databank chapter.
- **Add real flight logic** — detecting liftoff, apogee, parachute deployment, landing. That's the FSM's transition table. Read the Finite State Machine chapter first: it explains exactly what's missing today and why.
- **Remove something to make the binary smaller, or port to a new board.** That's the Build System and Repository structure chapters, under Repositories.

## Ground rules, wherever you're working

A few constraints apply across every chapter in this section, not just to one component:

- No dynamic memory allocation, anywhere. Producers and consumers borrow packets from the DATABANK's static pool; they never malloc.
- Everything runs bare-metal, on a single core, with no preemption. A consumer's consumeData must never block — no polling, no busy-waits — because there's no mechanism to reclaim CPU time from it if it does.
- Every producer and consumer must follow its contract exactly (see the PUBSUB chapter). The PUBSUB service calls these function pointers unconditionally, so a missing or misbehaving one breaks the loop for every other module too, not just its own.
