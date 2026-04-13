#!/bin/bash
# Solaris — Linux setup script
# Installs Git, Docker Engine and VS Code, then configures everything
# for the Solaris Dev Container.
#
# Usage: sudo ./scripts/install-linux.sh
# Supported: Ubuntu / Debian / Fedora / RHEL / Arch Linux

set -euo pipefail

# ── Colours ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

info()    { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()      { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }
section() { echo -e "\n${BOLD}━━━  $*  ━━━${NC}"; }

# ── Privilege check ─────────────────────────────────────────────────────────────
if [ "$EUID" -ne 0 ]; then
    error "Run with sudo:  sudo ./scripts/install-linux.sh"
fi

# Keep track of the real user (not root) so we can configure groups and extensions
REAL_USER="${SUDO_USER:-$USER}"
REAL_HOME=$(eval echo "~$REAL_USER")

# ── Distro detection ────────────────────────────────────────────────────────────
section "Detecting distribution"

if [ ! -f /etc/os-release ]; then
    error "Cannot detect distribution: /etc/os-release not found."
fi

. /etc/os-release
DISTRO="${ID:-unknown}"
DISTRO_LIKE="${ID_LIKE:-}"

info "Distribution: $PRETTY_NAME"

is_debian() { [[ "$DISTRO" == "ubuntu" || "$DISTRO" == "debian" || "$DISTRO_LIKE" == *"debian"* ]]; }
is_fedora() { [[ "$DISTRO" == "fedora" || "$DISTRO" == "rhel" || "$DISTRO" == "centos" || "$DISTRO_LIKE" == *"fedora"* || "$DISTRO_LIKE" == *"rhel"* ]]; }
is_arch()   { [[ "$DISTRO" == "arch" || "$DISTRO" == "manjaro" || "$DISTRO_LIKE" == *"arch"* ]]; }

if ! is_debian && ! is_fedora && ! is_arch; then
    warn "Distribution '$DISTRO' is not explicitly supported."
    warn "The script will attempt to continue but may fail."
fi

# ── Git ─────────────────────────────────────────────────────────────────────────
section "Git"

if command -v git &>/dev/null; then
    ok "Git already installed: $(git --version)"
else
    info "Installing Git..."
    if   is_debian; then apt-get install -y git
    elif is_fedora; then dnf install -y git
    elif is_arch;   then pacman -S --noconfirm git
    fi
    ok "Git installed."
fi

# ── Docker Engine ───────────────────────────────────────────────────────────────
section "Docker Engine"

if command -v docker &>/dev/null; then
    ok "Docker already installed: $(docker --version)"
else
    info "Installing Docker Engine..."

    if is_debian; then
        apt-get install -y ca-certificates curl gnupg lsb-release

        install -m 0755 -d /etc/apt/keyrings
        curl -fsSL https://download.docker.com/linux/"$DISTRO"/gpg \
            | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
        chmod a+r /etc/apt/keyrings/docker.gpg

        echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/$DISTRO \
$(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
            > /etc/apt/sources.list.d/docker.list

        apt-get update
        apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

    elif is_fedora; then
        dnf install -y dnf-plugins-core
        dnf config-manager --add-repo https://download.docker.com/linux/fedora/docker-ce.repo
        dnf install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
        systemctl enable --now docker

    elif is_arch; then
        pacman -S --noconfirm docker docker-compose
        systemctl enable --now docker
    fi

    ok "Docker installed."
fi

# Add the real user to the docker group so Docker works without sudo
if ! groups "$REAL_USER" | grep -q docker; then
    info "Adding $REAL_USER to the 'docker' group..."
    usermod -aG docker "$REAL_USER"
    warn "Group change takes effect after the next login or: newgrp docker"
fi

# Start and enable Docker daemon
if ! systemctl is-active --quiet docker; then
    info "Starting Docker daemon..."
    systemctl enable --now docker
fi
ok "Docker daemon running."

# ── VS Code ─────────────────────────────────────────────────────────────────────
section "Visual Studio Code"

if command -v code &>/dev/null; then
    ok "VS Code already installed: $(code --version | head -1)"
else
    info "Installing VS Code..."

    if is_debian; then
        apt-get install -y wget gpg apt-transport-https

        wget -qO- https://packages.microsoft.com/keys/microsoft.asc \
            | gpg --dearmor > /etc/apt/keyrings/packages.microsoft.gpg
        echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] \
https://packages.microsoft.com/repos/code stable main" \
            > /etc/apt/sources.list.d/vscode.list

        apt-get update
        apt-get install -y code

    elif is_fedora; then
        rpm --import https://packages.microsoft.com/keys/microsoft.asc
        cat > /etc/yum.repos.d/vscode.repo << 'EOF'
[code]
name=Visual Studio Code
baseurl=https://packages.microsoft.com/yumrepos/vscode
enabled=1
gpgcheck=1
gpgkey=https://packages.microsoft.com/keys/microsoft.asc
EOF
        dnf install -y code

    elif is_arch; then
        # VS Code is in the community repo as 'code' (open source build)
        # For the official Microsoft build users can install 'visual-studio-code-bin' from AUR
        if pacman -Si code &>/dev/null 2>&1; then
            pacman -S --noconfirm code
        else
            warn "On Arch, install VS Code manually from AUR: yay -S visual-studio-code-bin"
        fi
    fi

    ok "VS Code installed."
fi

# ── VS Code Dev Containers extension ───────────────────────────────────────────
section "Dev Containers extension"

# Run as the real user, not root
EXTENSION_ID="ms-vscode-remote.remote-containers"

if sudo -u "$REAL_USER" code --list-extensions 2>/dev/null | grep -q "$EXTENSION_ID"; then
    ok "Dev Containers extension already installed."
else
    info "Installing Dev Containers extension..."
    sudo -u "$REAL_USER" code --install-extension "$EXTENSION_ID" || \
        warn "Could not install extension automatically. Install it manually: $EXTENSION_ID"
fi

# ── Summary ─────────────────────────────────────────────────────────────────────
section "All done"

echo ""
echo -e "${GREEN}${BOLD}Solaris development environment is ready.${NC}"
echo ""
echo "  Next steps:"
echo "  1. Log out and back in (or run: newgrp docker) to apply the Docker group."
echo "  2. Clone the repo:"
echo "       git clone --recurse-submodules https://github.com/Software-Solaris/solaris-software.git"
echo "  3. Open it in VS Code:"
echo "       code solaris-software"
echo "  4. Click 'Reopen in Container' when VS Code prompts."
echo "  5. Inside the container terminal:"
echo "       cd solaris-v1 && idf.py build"
echo ""
