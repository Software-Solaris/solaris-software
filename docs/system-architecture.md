# System Architecture

This document explains how the entire development setup works — from your laptop to
the physical ESP32-S3 board — so that anyone can replicate it or understand what is
happening under the hood.

---

## The big picture

The ESP32-S3 board is not plugged into anyone's laptop. It lives on a **remote VPS**
connected 24/7. Developers connect to that server over an encrypted tunnel (WireGuard
VPN) and use VS Code on their own machine to build code, flash it to the chip, and
debug it — as if the board were on their desk.

```
  ╔══════════════════════════════════════════════════════╗
  ║  Your laptop                                         ║
  ║                                                      ║
  ║  VS Code                                             ║
  ║  └── Dev Container (Docker)                          ║
  ║       └── ESP-IDF toolchain                          ║
  ║            ├── idf.py build / merge-bin              ║
  ║            ├── Flash  ──── SCP + SSH ────────────┐   ║
  ║            ├── Debug  ──── GDB ──────────────┐   │   ║
  ║            └── Serial ──── SSH ──────────┐   │   │   ║
  ╚══════════════════════════════════════════╪═══╪═══╪═══╝
                                             │   │   │
                      WireGuard VPN tunnel   │   │   │
  ───────────────────────────────────────────┼───┼───┼── VPS ──
                                             │   │   │
  ╔══════════════════════════════════════════╪═══╪═══╪════════╗
  ║  VPS (Fedora)                            │   │   │        ║
  ║                                          │   │   │        ║
  ║  ┌────────────────────────────────┐      │   │   │        ║
  ║  │  openocd.service  (systemd)    │◄─────┘◄──┘   │        ║
  ║  │  GDB server  ·  port 3334      │              │        ║
  ║  │  JTAG → /dev/ttyACM0           │              │        ║
  ║  └────────────────────────────────┘              │        ║
  ║                                                  │        ║
  ║  ┌────────────────────────────────┐              │        ║
  ║  │  /dev/ttyACM0  (USB)           │◄─────────────┘        ║
  ║  │  ESP32-S3 serial  ·  115200    │                       ║
  ║  └────────────────────────────────┘                       ║
  ║                                                           ║
  ║  ┌────────────────────────────────┐                       ║
  ║  │  solaris-web  (Podman)         │                       ║
  ║  │  nginx:alpine  ·  port 9173    ├──► Cloudflare Tunnel  ║
  ║  └────────────────────────────────┘         │             ║
  ╚═════════════════════════════════════════════╪═════════════╝
                                                │
                                                ▼
                                   https://softwaresolaris.com
```

---

## Step 1 — Connect to the VPN

Before you can do anything with the board, you need to be on the VPN.

### What is WireGuard?

Think of it as a locked door. The VPS and all its ports (SSH, OpenOCD, serial) are
invisible to the public internet. Only people with a valid WireGuard key can get in.
Once you are connected, the VPS appears as if it were a machine on your local network —
you can SSH into it, open ports, etc., just like at home.

### How we expose it

Our router exposes WireGuard at:

```
solaris_IP: 443
```

Port 443 is the standard HTTPS port. Using it means the connection works even on
networks that block custom ports (universities, offices, etc.).

```
  Your laptop
  │
  │  WireGuard UDP → vpn.softwaresolaris.com:443
  ▼
  Router (port-forwards 443 to VPS WireGuard daemon)
  │
  ▼
  VPS — now reachable as a local machine
```

### Getting access

To join, a team admin needs to add your public WireGuard key and give you a config file.
Once you have it:

1. Install WireGuard on your machine ([wireguard.com](https://www.wireguard.com/install/))
2. Import the config file
3. Activate the tunnel

From that point on, `ssh raspi` works from your terminal.

---

## Step 2 — What lives on the VPS

Once you are on the VPN, the VPS has three things running:

### OpenOCD (the debug bridge)

OpenOCD is the program that talks to the ESP32-S3's debug hardware. Think of it as a
translator: on one side it speaks JTAG (the physical protocol the chip uses for
debugging), and on the other side it speaks GDB (the debugger protocol your tools
use).

It runs as a systemd service so it starts automatically on boot:

```
openocd
  -f board/esp32s3-builtin.cfg   ← tells OpenOCD what chip it is talking to
  -c "bindto 0.0.0.0"            ← listen on all VPN-visible interfaces
  -c "gdb_port 3334"             ← GDB connects here
```

Logs: `journalctl -u openocd -f`

### solaris-web (the project website)

A container serving the static project website via Cloudflare Tunnel.
See [website/README.md](../website/README.md).

### ESP32-S3 (the hardware)

Connected to the VPS by a single USB cable (`/dev/ttyACM0`). That one cable carries:

| What | How | Details |
|---|---|---|
| Flash | `esptool` over USB | Writes the compiled binary to the chip's flash memory |
| Debug | JTAG over USB | OpenOCD reads/writes CPU registers, sets breakpoints |
| Serial | `screen` over USB | Reads the UART output printed by the firmware |

---

## Step 3 — Remote logic analyser (optional)

A logic analyser lets you watch the signals on the SPI/I2C bus in real time — useful
for diagnosing communication issues at the hardware level.

If a logic analyser is connected to the VPS, you can use it from your laptop over the
network using **USB/IP**, a protocol that forwards a physical USB device over TCP/IP as
if it were plugged into your machine locally.

> **Currently there is no logic analyser physically connected to the VPS.**
> This section documents how to set it up when one is available.

To see how USB/IP is configured on the VPS:

```bash
ssh raspi
# then inspect: lsusb, usbip list -l, systemctl status usbipd
```

On your laptop you would then run:

```bash
# Linux example — attach the remote USB device locally
sudo modprobe vhci-hcd
sudo usbip attach -r <VPS_VPN_IP> -b <bus-id>
```

After attaching, the device appears as a local USB device and you can use it with
your normal analysis software.

---

## Step 4 — Building and flashing from VS Code

### The Dev Container

When you open this repository in VS Code and click "Reopen in Container", Docker
starts a container that has the full ESP-IDF toolchain pre-installed. Your VS Code
terminal is now running _inside_ that container — so `idf.py`, `esptool` and all the
other tools are available without installing anything on your machine.

### Building manually

Open the terminal inside the container and run:

```bash
cd solaris-v1
idf.py build        # compiles everything → produces .elf and .bin files
idf.py merge-bin    # merges bootloader + partition table + app into one .bin
```

### Flashing manually

Once built, run the VS Code task `flash-esp32s3-remote-only`
(`Ctrl+Shift+P` → *Tasks: Run Task*). It does three things automatically:

```
1. Finds the merged .bin in solaris-v1/build/
2. Copies it to the VPS:   scp merged.bin → raspi:/tmp/merged.bin
3. Flashes via SSH:        ssh raspi "esptool write-flash 0x0 /tmp/merged.bin"
```

---

## Step 5 — Debugging with VS Code (F5)

This is where everything comes together. Press **F5** (or *Run → Start Debugging*)
with the **"ESP32-S3 · Build + Flash + Debug"** configuration selected.

Here is exactly what VS Code does, step by step:

### Phase 1 — preLaunchTask: `full-debug-prepare`

VS Code runs this task chain _before_ starting the debugger:

```
full-debug-prepare
├── docker-build-flash          ← build + flash (same as Step 4 above)
└── gdb-prepare                 ← prepare the debug connection
    ├── ssh-tunnel-openocd-up
    │   ├── ssh-tunnel-openocd-open   ← open SSH tunnels to OpenOCD ports
    │   └── ssh-tunnel-openocd-wait   ← wait until the tunnel is ready
    └── openocd-reset-halt            ← pause the CPU at the reset vector
```

**What the SSH tunnel does:**

Even though GDB connects directly to the VPS via the VPN, OpenOCD has additional
control ports (telnet and TCL) that are used to send commands like "reset halt".
The task opens these as local tunnels:

```
localhost:3337  →  raspi:3334   (OpenOCD GDB)
localhost:4447  →  raspi:4444   (OpenOCD telnet — used for reset halt)
localhost:6667  →  raspi:6666   (OpenOCD TCL)
```

**What "reset halt" does:**

It sends a command to OpenOCD that resets the ESP32-S3 and immediately pauses it
before it runs a single instruction. This ensures the debugger attaches to a clean
state at the very start of execution.

### Phase 2 — GDB attaches

VS Code launches `xtensa-esp32s3-elf-gdb` (inside the container, via the wrapper
script `.devcontainer/docker-gdb.sh`) and connects it to OpenOCD at
`<RASPI_IP>:3334` — directly over the WireGuard VPN.

```
  ╔═══════════════════╗              ╔══════════════════╗
  ║  VS Code  (GDB)   ║ ─ VPN:3334 ─►  OpenOCD on VPS  ║
  ╚═══════════════════╝              ╚════════╤═════════╝
                                              │  JTAG
                                              ▼
                                        ESP32-S3
```

### Phase 3 — You are in the debugger

VS Code stops at `app_main` (configured with `thb app_main` in the setup commands).
From here you can:

| Action | How |
|---|---|
| Continue execution | F5 |
| Step over a line | F10 |
| Step into a function | F11 |
| Step out of a function | Shift+F11 |
| Set a breakpoint | Click the gutter to the left of a line number |
| Inspect a variable | Hover over it, or open the Variables panel on the left |
| Evaluate an expression | Type it in the Debug Console at the bottom |
| Restart | Ctrl+Shift+F5 |
| Stop debugging | Shift+F5 |

### Phase 4 — Cleanup: `debug-cleanup`

When you stop the debugger, VS Code runs `debug-cleanup`, which closes the SSH
tunnels to OpenOCD. This frees up the ports for the next debug session.

---

## Serial monitor

To see the output printed by the firmware (`ESP_LOGI`, `printf`, etc.), open the
**Raspi serial** terminal profile — click the `+` dropdown in VS Code's terminal
panel and select it.

It connects via SSH and opens `screen /dev/ttyACM0 115200`:

```
ssh raspi  →  screen /dev/ttyACM0 115200
```

To exit screen: `Ctrl+A` then `K`, then `Y`.

---

## Replicating this setup

If you want to build an identical setup with your own hardware:

1. **A machine with a publicly reachable IP** (VPS, home server, etc.)
2. **WireGuard** installed on that machine — generate a key pair and create a peer config for each developer
3. **A USB connection** from the machine to your ESP32-S3 board
4. **OpenOCD** installed and configured as a systemd service (see `openocd.service` in this repo)
5. **Your router** port-forwarding UDP 443 to the machine's WireGuard port
6. **A DNS record** pointing your domain to your router's public IP
7. Clone this repo, open in the Dev Container, done — `ssh raspi` will be the only thing that needs updating in your `.ssh/config`
