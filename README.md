# Solaris

Open-source firmware framework for amateur rocketry, targeting **ESP32-S3** microcontrollers.
Solaris provides reusable sensor drivers and a software architecture built around the
**Solaris Packet Protocol (SPP)**, designed so that teams without the time or resources
to build their own stack can get flying faster.

---

## Documentation

| Document | Contents |
|---|---|
| [Software Architecture](docs/software-architecture.md) | Repo structure, firmware layers (HAL / SPP), components |
| [System Architecture](docs/system-architecture.md) | Remote development setup: WireGuard VPN, VPS, ESP32, VS Code |
| [Website](website/README.md) | How the project website is hosted and updated |

---

## Quick start

### Step 1 — Get Git

You need Git to clone the repository. If you don't have it:

| OS | Command |
|---|---|
| **Ubuntu / Debian** | `sudo apt install git` |
| **Fedora** | `sudo dnf install git` |
| **Arch** | `sudo pacman -S git` |
| **Windows** | Open PowerShell as Administrator → `winget install Git.Git` → open a new terminal |
| **Any OS** | Download from [git-scm.com](https://git-scm.com) |

### Step 2 — Clone the repository

```bash
git clone --recurse-submodules https://github.com/Software-Solaris/solaris-software.git
cd solaris-software
```

> `--recurse-submodules` is required — ESP-IDF and the SPP libraries are git submodules.
> Use HTTPS for this first clone since SSH keys are configured in the next step.

### Checkout a specific release tag

Solaris uses git submodules. A tag in the main repository stores the exact commit of each
submodule that was used when that tag was created.

To checkout a tagged version of Solaris and make sure all submodules are also placed at
the correct commits, run:

```bash
git fetch --tags
git checkout <tag-name>
git submodule update --init --recursive
```

Do not manually checkout branches inside the submodules unless you are developing them.
For a released tag, the submodules should remain at the commits recorded by the main
repository.

To verify the submodule state:

```bash
git submodule status
```

The commit shown for each submodule is the one associated with the checked-out tag of the
main repository.

### Step 3 — Run the setup script

The script installs Docker and VS Code, generates SSH keys, configures Git identity, and sets up everything for the Dev Container. It is safe to run multiple times — it skips anything already done and offers to pull the latest repo changes on subsequent runs.

The script will ask for your name and email and write them to `~/.gitconfig` (Linux) or `%USERPROFILE%\.gitconfig` (Windows):

```ini
[user]
    name  = Your Name
    email = your.email@example.com
```

**Linux** (Ubuntu / Debian / Fedora / Arch) — run with `sudo`:

```bash
sudo ./scripts/install-linux.sh
```

> The script adds your user to the `docker` group and opens a new session automatically to apply it — no manual logout needed.

**Windows** — open PowerShell **as Administrator**:

```powershell
Set-ExecutionPolicy Bypass -Scope Process -Force
.\scripts\install-windows.ps1
```

> The Windows script checks for common issues (virtualisation disabled, insufficient disk
> space, WSL2 not enabled, missing winget) and gives step-by-step instructions for each.
> A reboot may be required after enabling WSL2; re-run the script afterwards.

### Step 4 — Open in VS Code and build

```bash
code .
```

VS Code will detect the `.devcontainer` configuration and prompt:

> **"Reopen in Container"** → click it.

If the prompt does not appear: `Ctrl+Shift+P` → **Dev Containers: Reopen in Container**.

Docker builds the image on the first run — this takes a few minutes. Subsequent opens are instant.

Once inside the container, open a terminal (`Ctrl+` `` ` ``):

```bash
cd solaris-v1
idf.py build
```

A successful build produces the firmware binaries under `solaris-v1/build/`.

### Step 5 — Flash and debug

Flashing and debugging require access to the remote development station via WireGuard VPN.
See [System Architecture](docs/system-architecture.md) for the full setup.

---

## Versions

| Version | Status | Description |
|---|---|---|
| `solaris-v0.1` | Legacy | BMP390 barometer — standalone ESP-IDF driver |
| `solaris-v0.2` | Legacy | ICM20948 IMU — standalone ESP-IDF driver |
| `solaris-v1` | **Active** | Combined sensors + SPP v2 bare-metal superloop |

---

## Collaboration

Contributions, ideas and improvements are welcome.
Fork the repository, make your changes on a branch, and open a pull request against `main`.

## License

**Solaris Software License v1.0** — key points:

- Free for personal, educational and amateur rocketry use.
- **Commercial use is unconditionally prohibited.** No commercial licence exists or will ever be granted. Any profits generated must be paid in full to Team Solaris.
- **Military use is unconditionally prohibited.**
- All modifications must be published under this same licence (copyleft).
- Academic use (TFG/TFM) is allowed provided the work is public and credits this repository.

See [LICENSE.md](LICENSE.md) for the full terms.