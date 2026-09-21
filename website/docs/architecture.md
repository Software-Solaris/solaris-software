# How the repository is organized

The project consists of two repositories: **solaris-software**, the top-level repository, and **solaris-packet-protocol** (SPP), included as a git submodule at `solaris-v2/spp`. The separation follows a single principle: platform-independent code belongs in SPP, while platform-dependent code — along with all tooling required to build, flash and debug the hardware — belongs in solaris-software.

## solaris-software — platform-dependent code

This is the outer repository. It contains the ESP32-S3 firmware project and the infrastructure required to deploy it to hardware:

```
solaris-software/
├── .devcontainer/
├── .vscode/
├── .github/
├── scripts/
└── solaris-v2/
    ├── main/
    ├── compiler/
    ├── CMakeLists.txt, sdkconfig
    └── spp/
```

- `.devcontainer/` — the Docker image and `docker-compose.yml`, plus the `docker-*.sh` scripts that build, flash, monitor and debug the board from inside the container.
- `.vscode/` — `tasks.json` / `launch.json`, the VS Code build, flash and debug tasks, wired to the scripts above.
- `.github/` — CI workflows.
- `scripts/` — `install-linux.sh` / `install-windows.ps1`, which set up Docker, VS Code and SSH on a new machine.
- `solaris-v2/` — the ESP32-S3 firmware project.
    - `main/` — `app_main`, where `FSM_init` and `FSM_tick` are called.
    - `compiler/` — ESP-IDF "component" wrappers (`spp/`, `spp_ports/`) that pull SPP's sources into the ESP-IDF build. See the [Build System](build-system.md) page for details.
    - `CMakeLists.txt`, `sdkconfig` — the ESP-IDF project itself.
    - `spp/` — where the SPP submodule is mounted.

Every component listed here assumes an ESP32-S3 target running inside a Docker development container: the Dockerfile, the flashing scripts, the debug tasks, and the ESP-IDF integration in `compiler/`. None of it is a dependency of SPP itself.

## solaris-packet-protocol — platform-independent code

SPP is developed and versioned as an independent repository, and included here as a submodule. Its rule mirrors the one above: no code within SPP may reference a specific board, with a single deliberate exception — the `ports/` directory.

```
spp/
├── core/
├── hal/
├── services/
├── util/
├── external/
├── tests/
└── ports/
    └── hal/
        ├── esp32/
        └── stub/
```

- `core/` — packet format, pub/sub, FSM, core init. See the [Core](core-overview.md) section.
- `hal/` — the HAL contract: structs of function pointers, no board code.
- `services/` — sensor drivers and other producers/consumers.
- `util/` — CRC, compile-time flags, small portable helpers.
- `external/` — optional third-party code, such as the encryption module.
- `tests/` — Cgreen unit tests, run on a PC, no hardware involved.
- `ports/` — the one platform-dependent part of SPP.
    - `hal/esp32/` — the real ESP32-S3 implementation of the HAL contract.
    - `hal/stub/` — a no-op implementation, used to build and test on a PC.

`core/`, `hal/`, `services/`, `util/`, `external/` and `tests/` contain plain, portable C11, with no dependency on ESP-IDF, FreeRTOS or board-specific headers. This follows directly from the HAL design described above: the HAL layer only invokes function pointers and never accesses hardware registers directly. `ports/` is where those function pointers are bound to a concrete implementation — one per target board, or, in the case of the stub port, to no implementation at all: every call returns `K_SPP_OK`, which allows the unit tests to run on a standard Linux machine.

Because `ports/` resides within SPP rather than in solaris-software, SPP can be built, tested and reused independently of it: cloning SPP alone and building against the stub port compiles and runs the entire core and all services, without requiring an ESP32-S3 target, a Docker container, or any of the tooling described above.
