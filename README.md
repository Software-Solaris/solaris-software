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

### Step 2 — Set up an SSH key for GitHub

An SSH key lets you push and pull without entering your password every time.
Skip this step if you already have one configured (`~/.ssh/id_ed25519.pub` exists).

---

#### Linux

**1. Generate the key**

```bash
ssh-keygen -t ed25519 -C "your.email@example.com"
```

Accept the default path (`~/.ssh/id_ed25519`). Set a passphrase if you want extra security (optional but recommended).

**2. Add the key to the SSH agent**

```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```

**3. Copy the public key**

```bash
cat ~/.ssh/id_ed25519.pub
```

Select and copy the entire output (starts with `ssh-ed25519 …`).

---

#### Windows

Open **PowerShell** (no Administrator needed):

**1. Generate the key**

```powershell
ssh-keygen -t ed25519 -C "your.email@example.com"
```

Accept the default path (`C:\Users\YourName\.ssh\id_ed25519`). Set a passphrase if desired.

**2. Copy the public key to the clipboard**

```powershell
Get-Content "$env:USERPROFILE\.ssh\id_ed25519.pub" | Set-Clipboard
```

---

#### Add the key to GitHub

1. Go to **github.com** → click your profile picture (top-right) → **Settings**
2. In the left sidebar click **SSH and GPG keys**
3. Click **New SSH key**
4. Fill in:
   - **Title** — a label for this machine (e.g. `My Laptop` or `Work PC`)
   - **Key type** — `Authentication Key`
   - **Key** — paste the public key you copied above
5. Click **Add SSH key** and confirm with your GitHub password if prompted

**Verify the connection:**

```bash
ssh -T git@github.com
```

Expected response: `Hi <username>! You've successfully authenticated…`

---

### Step 3 — Clone the repository

```bash
git clone --recurse-submodules git@github.com:Software-Solaris/solaris-software.git
cd solaris-software
```

> `--recurse-submodules` is required — ESP-IDF and the SPP libraries are git submodules.
>
> If you skipped the SSH setup, use HTTPS instead:
> `git clone --recurse-submodules https://github.com/Software-Solaris/solaris-software.git`

### Step 4 — Run the setup script

The script installs Docker and VS Code, and configures everything for the Dev Container automatically.

**Linux** (Ubuntu / Debian / Fedora / Arch) — run with `sudo`:

```bash
sudo ./scripts/install-linux.sh
```

The script adds your user to the `docker` group automatically.
**Log out and back in** (or run `newgrp docker`) after it finishes for the change to take effect.

**Windows** — open PowerShell **as Administrator**:

```powershell
Set-ExecutionPolicy Bypass -Scope Process -Force
.\scripts\install-windows.ps1
```

> The Windows script checks for common issues (virtualisation disabled, insufficient disk
> space, WSL2 not enabled, missing winget) and gives step-by-step instructions for each.
> It is safe to run multiple times — it skips anything already installed.
> A reboot may be required after enabling WSL2; re-run the script afterwards.

### Step 5 — Open in VS Code and build

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

### Step 6 — Flash and debug

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
