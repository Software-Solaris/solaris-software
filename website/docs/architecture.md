# How the repository is organized

Solaris is actually two repositories glued together: **solaris-software**, the one you clone first, and **solaris-packet-protocol** (SPP), pulled in as a git submodule at solaris-v2/spp. The split follows a simple rule: anything that doesn't care which board it's running on lives in SPP, and anything that does — plus all the tooling to build, flash and debug the real hardware — lives in solaris-software.

## solaris-software — everything platform-dependent

This is the outer repository. It's where you'll find the ESP32-S3 firmware project itself, and all the machinery around it needed to actually get code onto a board:

```
solaris-software/
├── .devcontainer/    Docker image + docker-compose, and the docker-*.sh
│                     scripts that build, flash, monitor and debug the board
│                     from inside the container
├── .vscode/          tasks.json / launch.json — the VS Code build, flash
│                     and debug tasks, wired to the scripts above
├── .github/          CI workflows
├── scripts/          install-linux.sh / install-windows.ps1 — set up
│                     Docker, VS Code and SSH on a new machine
└── solaris-v2/       the ESP32-S3 firmware project
    ├── main/         app_main — where FSM_init and FSM_tick get called
    ├── compiler/      ESP-IDF "component" wrappers (spp/, spp_ports/) that
    │                 pull SPP's sources into the ESP-IDF build — see the
    │                 build system page for details
    ├── CMakeLists.txt, sdkconfig   the ESP-IDF project itself
    └── spp/          <- the SPP submodule mounts here
```

Everything in this list only makes sense once you already know you're targeting an ESP32-S3 inside a Docker dev container: the Dockerfile, the flashing scripts, the debug tasks, the ESP-IDF glue in compiler/. None of it is something SPP itself needs to know about.

## solaris-packet-protocol — everything platform-independent (except ports/)

SPP is its own repository, developed and versioned separately, and checked out as a submodule. The rule here is the mirror image of the one above: nothing in SPP is allowed to know which board it's running on, with a single, deliberate exception — the ports/ folder.

```
spp/
├── core/       packet format, pub/sub, FSM, core init — see the Core section
├── hal/        the HAL contract: structs of function pointers, no board code
├── services/   sensor drivers and other producers/consumers
├── util/       CRC, compile-time flags, small portable helpers
├── external/   optional third-party code (e.g. the encryption module)
├── tests/      Cgreen unit tests — run on a PC, no hardware involved
└── ports/      <- the one platform-dependent part of SPP
    └── hal/
        ├── esp32/   the real ESP32-S3 implementation of the HAL contract
        └── stub/    a no-op implementation, used to build and test on a PC
```

core/, hal/, services/, util/, external/ and tests/ are plain, portable C11 — no ESP-IDF, no FreeRTOS, no board headers. That's exactly what the HAL section explained: the HAL layer only ever calls through function pointers, never touches a register directly. ports/ is where those pointers finally get pointed at real code, one implementation per board (or, for the stub port, at nothing at all — every call just returns K_SPP_OK, which is what makes it possible to build and run the unit tests on a plain Linux machine).

Keeping ports/ inside SPP rather than in solaris-software is what lets SPP be built, tested and reused entirely on its own: clone SPP by itself, point the build at the stub port, and the whole core and every service compiles and runs without an ESP32-S3, a Docker container, or any of the tooling described above.
