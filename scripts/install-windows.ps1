# Solaris - Windows setup script
# Installs Git, Docker Desktop and VS Code, enables WSL2, and verifies
# the full environment for the Solaris Dev Container.
#
# Usage (PowerShell as Administrator):
#   Set-ExecutionPolicy Bypass -Scope Process -Force
#   .\scripts\install-windows.ps1
#
# Safe to run multiple times - skips anything already installed.

#Requires -RunAsAdministrator

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# -- Output helpers -------------------------------------------------------------
function Info    ($msg) { Write-Host "  [INFO]  $msg" -ForegroundColor Cyan }
function Ok      ($msg) { Write-Host "  [OK]    $msg" -ForegroundColor Green }
function Warn    ($msg) { Write-Host "  [WARN]  $msg" -ForegroundColor Yellow }
function Section ($msg) { Write-Host "`n  ======================================" -ForegroundColor DarkGray
                          Write-Host "  $msg" -ForegroundColor White
                          Write-Host "  ======================================" -ForegroundColor DarkGray }

function Fail {
    param(
        [string]$Title,
        [string]$Reason,
        [string[]]$Fix,
        [string]$After = "After fixing, re-run this script."
    )
    Write-Host ""
    Write-Host "  +== PROBLEM DETECTED ===========================+" -ForegroundColor Red
    Write-Host "  |  $Title" -ForegroundColor Red
    Write-Host "  +===============================================+" -ForegroundColor Red
    Write-Host ""
    Write-Host "  Why:" -ForegroundColor Yellow
    Write-Host "    $Reason" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "  How to fix:" -ForegroundColor Cyan
    foreach ($step in $Fix) {
        Write-Host "    $step" -ForegroundColor Cyan
    }
    Write-Host ""
    Write-Host "  $After" -ForegroundColor Gray
    Write-Host ""
    exit 1
}

function RebootRequired {
    param([string]$Reason)
    Write-Host ""
    Write-Host "  +== REBOOT REQUIRED ============================+" -ForegroundColor Yellow
    Write-Host "  |  $Reason" -ForegroundColor Yellow
    Write-Host "  +-----------------------------------------------+" -ForegroundColor Yellow
    Write-Host "  |  Restart your PC, then re-run this script.   |" -ForegroundColor Yellow
    Write-Host "  +===============================================+" -ForegroundColor Yellow
    Write-Host ""
    exit 0
}

Write-Host ""
Write-Host "  +=======================================+" -ForegroundColor Cyan
Write-Host "  |  Solaris - Windows Environment Setup  |" -ForegroundColor Cyan
Write-Host "  +=======================================+" -ForegroundColor Cyan


# ==============================================================================
#  1. WINDOWS VERSION
# ==============================================================================
Section "1/8  Windows version"

$build = [System.Environment]::OSVersion.Version.Build
$caption = (Get-WmiObject Win32_OperatingSystem).Caption
Info "Detected: $caption (build $build)"

if ($build -lt 19041) {
    Fail `
        "Windows is too old for WSL2" `
        "WSL2 requires Windows 10 version 2004 (build 19041) or later.`n    Your build: $build" `
        @(
            "1. Open Settings -> Windows Update",
            "2. Click 'Check for updates' and install everything available",
            "3. Reboot and re-run this script"
        )
}
Ok "Windows $build - compatible."


# ==============================================================================
#  2. CPU VIRTUALISATION
# ==============================================================================
Section "2/8  CPU virtualisation"

$cpu         = Get-WmiObject -Class Win32_Processor | Select-Object -First 1
$virtEnabled = $cpu.VirtualizationFirmwareEnabled
$hypervisor  = (Get-WmiObject -Class Win32_ComputerSystem).HypervisorPresent
$manufacturer = (Get-WmiObject -Class Win32_ComputerSystem).Manufacturer

Info "CPU: $($cpu.Name)"
Info "Manufacturer: $manufacturer"

if (-not $virtEnabled -and -not $hypervisor) {
    # Give manufacturer-specific BIOS key hints
    $biosKey = switch -Wildcard ($manufacturer) {
        "*Dell*"    { "F2 or F12" }
        "*HP*"      { "F10 or Esc -> F10" }
        "*Lenovo*"  { "F1, F2 or Enter -> F1" }
        "*ASUS*"    { "Del or F2" }
        "*MSI*"     { "Del" }
        "*Acer*"    { "F2 or Del" }
        "*Gigabyte*"{ "Del" }
        "*Microsoft*"{ "Volume-Down + Power (Surface)" }
        default     { "Del, F2 or F10 (varies by model - check your manual)" }
    }

    Fail `
        "CPU virtualisation is disabled in BIOS/UEFI" `
        "Docker Desktop requires hardware virtualisation (Intel VT-x / AMD-V).`n    It is currently OFF in your firmware settings." `
        @(
            "1. Restart your PC",
            "2. Enter BIOS/UEFI setup by pressing: $biosKey",
            "3. Look for one of these settings:",
            "     Intel:  'Intel VT-x', 'Intel Virtualization Technology', 'VT-x'",
            "     AMD:    'AMD-V', 'SVM Mode', 'Virtualization'",
            "4. Set it to Enabled",
            "5. Save and exit (usually F10)",
            "6. Boot into Windows and re-run this script"
        )
}
Ok "Virtualisation is enabled."


# ==============================================================================
#  3. DISK SPACE
# ==============================================================================
Section "3/8  Disk space"

$sysDrive = $env:SystemDrive[0]
$freeGB   = [math]::Round((Get-PSDrive $sysDrive).Free / 1GB, 1)
$minGB    = 15
Info "Free space on ${sysDrive}: $freeGB GB  (minimum required: $minGB GB)"

if ($freeGB -lt $minGB) {
    Fail `
        "Not enough free disk space" `
        "At least ${minGB} GB is needed for Docker images, WSL2 and VS Code.`n    Currently free: $freeGB GB on drive ${sysDrive}:" `
        @(
            "1. Open Settings -> System -> Storage",
            "2. Run 'Storage Sense' or delete large files / uninstall unused apps",
            "3. Empty the Recycle Bin",
            "4. Re-run this script once you have ${minGB} GB free"
        )
}
Ok "$freeGB GB free - sufficient."


# ==============================================================================
#  4. INTERNET CONNECTIVITY
# ==============================================================================
Section "4/8  Internet connectivity"

$hosts = @("github.com", "download.docker.com", "aka.ms")
foreach ($h in $hosts) {
    try {
        $result = Test-NetConnection -ComputerName $h -Port 443 -WarningAction SilentlyContinue
        if (-not $result.TcpTestSucceeded) { throw }
        Ok "Reachable: $h"
    } catch {
        Fail `
            "Cannot reach $h" `
            "The installer needs internet access to download Docker, VS Code and winget." `
            @(
                "1. Check that your network cable / Wi-Fi is connected",
                "2. If you are behind a corporate proxy or VPN, disconnect it temporarily",
                "3. Open a browser and verify you can visit https://$h",
                "4. Re-run this script"
            )
    }
}


# ==============================================================================
#  5. WINGET
# ==============================================================================
Section "5/8  Package manager (winget)"

if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
    Info "winget not found - attempting to install App Installer..."
    try {
        $tmp = "$env:TEMP\AppInstaller.msixbundle"
        Invoke-WebRequest -Uri "https://aka.ms/getwinget" -OutFile $tmp -UseBasicParsing
        Add-AppxPackage -Path $tmp
    } catch {
        Fail `
            "winget (App Installer) could not be installed automatically" `
            "winget is the package manager used to install Git, Docker and VS Code." `
            @(
                "Option A - Microsoft Store (easiest):",
                "  1. Open the Microsoft Store",
                "  2. Search for 'App Installer'",
                "  3. Install or update it",
                "  4. Re-run this script",
                "",
                "Option B - GitHub release:",
                "  1. Go to: https://github.com/microsoft/winget-cli/releases/latest",
                "  2. Download the .msixbundle file",
                "  3. Double-click to install",
                "  4. Re-run this script"
            )
    }

    if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
        Fail `
            "winget still not available after installation" `
            "App Installer was downloaded but winget is not in PATH." `
            @(
                "1. Restart PowerShell as Administrator",
                "2. Re-run this script"
            )
    }
}
Ok "winget available: $(winget --version)."


# ==============================================================================
#  6. WSL2
# ==============================================================================
Section "6/8  WSL2"

# Check for conflicting hypervisors (VirtualBox / VMware can block Hyper-V / WSL2)
$conflictingApps = @()
@("Oracle VM VirtualBox", "VMware Workstation") | ForEach-Object {
    if (Get-WmiObject -Class Win32_Product -Filter "Name='$_'" -ErrorAction SilentlyContinue) {
        $conflictingApps += $_
    }
}
if ($conflictingApps.Count -gt 0) {
    Warn "Detected hypervisors that may conflict with WSL2: $($conflictingApps -join ', ')"
    Warn "If WSL2 fails to start later, open those apps and disable their virtualisation engine."
}

$wslFeature = Get-WindowsOptionalFeature -Online -FeatureName "Microsoft-Windows-Subsystem-Linux"
$vmFeature  = Get-WindowsOptionalFeature -Online -FeatureName "VirtualMachinePlatform"

$wslWasDisabled = $wslFeature.State -ne "Enabled"
$vmWasDisabled  = $vmFeature.State -ne "Enabled"

if ($wslWasDisabled) {
    Info "Enabling Windows Subsystem for Linux feature..."
    try {
        Enable-WindowsOptionalFeature -Online -FeatureName "Microsoft-Windows-Subsystem-Linux" -NoRestart | Out-Null
    } catch {
        Fail `
            "Could not enable the WSL Windows feature" `
            "Error: $($_.Exception.Message)" `
            @(
                "1. Open 'Turn Windows features on or off' (search in Start)",
                "2. Check 'Windows Subsystem for Linux'",
                "3. Click OK and let Windows install it",
                "4. Reboot and re-run this script"
            )
    }
}

if ($vmWasDisabled) {
    Info "Enabling Virtual Machine Platform feature..."
    try {
        Enable-WindowsOptionalFeature -Online -FeatureName "VirtualMachinePlatform" -NoRestart | Out-Null
    } catch {
        Fail `
            "Could not enable the Virtual Machine Platform feature" `
            "Error: $($_.Exception.Message)" `
            "This feature is required for WSL2 and Docker." `
            @(
                "1. Open 'Turn Windows features on or off' (search in Start)",
                "2. Check 'Virtual Machine Platform'",
                "3. Click OK, reboot, and re-run this script"
            )
    }
}

if ($wslWasDisabled -or $vmWasDisabled) {
    RebootRequired "WSL2 features were just enabled and require a reboot to activate."
}

# Update WSL2 kernel
Info "Updating WSL2 kernel..."
try {
    wsl --update 2>&1 | Out-Null
    Ok "WSL2 kernel up to date."
} catch {
    Warn "Could not update WSL2 kernel automatically. If Docker fails to start, run: wsl --update"
}

# Set WSL2 as default
try {
    wsl --set-default-version 2 2>&1 | Out-Null
    Ok "WSL2 set as default version."
} catch {
    Fail `
        "Could not set WSL2 as the default version" `
        "Error: $($_.Exception.Message)" `
        @(
            "Run manually in PowerShell (as Administrator):",
            "  wsl --set-default-version 2",
            "If the error mentions the kernel, run first:",
            "  wsl --update"
        )
}


# ==============================================================================
#  7. GIT
# ==============================================================================
Section "7/8  Git"

if (Get-Command git -ErrorAction SilentlyContinue) {
    Ok "Git already installed: $(git --version)"
} else {
    Info "Installing Git..."
    try {
        winget install --id Git.Git --silent --accept-package-agreements --accept-source-agreements
        # Refresh PATH
        $env:Path = [System.Environment]::GetEnvironmentVariable("Path","Machine") + ";" +
                    [System.Environment]::GetEnvironmentVariable("Path","User")
        Ok "Git installed."
    } catch {
        Fail `
            "Git installation failed" `
            "winget could not install Git. Error: $($_.Exception.Message)" `
            @(
                "Install Git manually:",
                "  1. Go to: https://git-scm.com/download/win",
                "  2. Download and run the installer",
                "  3. Re-run this script"
            )
    }
}


# ==============================================================================
#  8. DOCKER DESKTOP
# ==============================================================================
Section "8/8  Docker Desktop"

$dockerPath = @(
    "$env:ProgramFiles\Docker\Docker\Docker Desktop.exe",
    "${env:ProgramFiles(x86)}\Docker\Docker\Docker Desktop.exe"
) | Where-Object { Test-Path $_ } | Select-Object -First 1

if ($dockerPath) {
    Ok "Docker Desktop already installed at: $dockerPath"
} else {
    Info "Installing Docker Desktop..."
    try {
        winget install --id Docker.DockerDesktop --silent --accept-package-agreements --accept-source-agreements
        Ok "Docker Desktop installed."
    } catch {
        Fail `
            "Docker Desktop installation failed" `
            "winget could not install Docker Desktop. Error: $($_.Exception.Message)" `
            @(
                "Install Docker Desktop manually:",
                "  1. Go to: https://www.docker.com/products/docker-desktop/",
                "  2. Download 'Docker Desktop for Windows'",
                "  3. Run the installer",
                "  4. Re-run this script to continue"
            )
    }
}

# Refresh PATH
$env:Path = [System.Environment]::GetEnvironmentVariable("Path","Machine") + ";" +
            [System.Environment]::GetEnvironmentVariable("Path","User")

# Verify docker CLI is reachable
if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    Warn "The 'docker' command is not yet in PATH."
    Warn "This is normal - open a new PowerShell window after Docker Desktop has started."
}

# Remind about WSL2 backend setting
Write-Host ""
Write-Host "  +-----------------------------------------------------+" -ForegroundColor Cyan
Write-Host "  |  ACTION REQUIRED after Docker Desktop starts:       |" -ForegroundColor Cyan
Write-Host "  |                                                      |" -ForegroundColor Cyan
Write-Host "  |  Settings -> General                                 |" -ForegroundColor Cyan
Write-Host "  |    [x] Use the WSL2 based engine                     |" -ForegroundColor Cyan
Write-Host "  |                                                      |" -ForegroundColor Cyan
Write-Host "  |  Settings -> Resources -> WSL Integration            |" -ForegroundColor Cyan
Write-Host "  |    [x] Enable integration with your WSL distro       |" -ForegroundColor Cyan
Write-Host "  +-----------------------------------------------------+" -ForegroundColor Cyan


# ==============================================================================
#  VS CODE + EXTENSION
# ==============================================================================
Section "VS Code + Dev Containers extension"

if (-not (Get-Command code -ErrorAction SilentlyContinue)) {
    Info "Installing Visual Studio Code..."
    try {
        winget install --id Microsoft.VisualStudioCode --silent --accept-package-agreements --accept-source-agreements
        $env:Path = [System.Environment]::GetEnvironmentVariable("Path","Machine") + ";" +
                    [System.Environment]::GetEnvironmentVariable("Path","User")
        Ok "VS Code installed."
    } catch {
        Fail `
            "VS Code installation failed" `
            "Error: $($_.Exception.Message)" `
            @(
                "Install VS Code manually:",
                "  1. Go to: https://code.visualstudio.com",
                "  2. Download and run the installer",
                "  3. Re-run this script"
            )
    }
} else {
    Ok "VS Code already installed."
}

$extensionId = "ms-vscode-remote.remote-containers"
if (Get-Command code -ErrorAction SilentlyContinue) {
    $installed = code --list-extensions 2>$null | Select-String $extensionId
    if ($installed) {
        Ok "Dev Containers extension already installed."
    } else {
        Info "Installing Dev Containers extension..."
        try {
            code --install-extension $extensionId
            Ok "Dev Containers extension installed."
        } catch {
            Warn "Could not install the extension automatically."
            Warn "Install it manually: open VS Code -> Extensions -> search 'Dev Containers'"
        }
    }
} else {
    Warn "VS Code not yet in PATH - open a new terminal after Docker Desktop starts,"
    Warn "then install the extension: code --install-extension $extensionId"
}


# ==============================================================================
#  SSH KEYS & GIT CONFIGURATION
# ==============================================================================
Section "SSH keys and Git configuration"

$sshDir = "$env:USERPROFILE\.ssh"
if (-not (Test-Path $sshDir)) { New-Item -ItemType Directory -Path $sshDir | Out-Null }

Write-Host ""

# --- GitHub SSH key
$inp = (Read-Host "  Generate a GitHub SSH key? [Y/n]").Trim()
if (-not $inp -or $inp -match '^[Yy]') {
    $inp = (Read-Host "  GitHub key name [id_ed25519]").Trim()
    $githubKeyName = if ($inp) { $inp } else { "id_ed25519" }
    $gitEmail = (Read-Host "  Email for key comment (Enter to skip)").Trim()

    $githubKeyPath = Join-Path $sshDir $githubKeyName
    if (Test-Path $githubKeyPath) {
        Ok "GitHub SSH key already exists: $githubKeyPath"
    } else {
        Info "Generating GitHub SSH key: $githubKeyPath"
        $keyComment = if ($gitEmail) { $gitEmail } else { "solaris" }
        ssh-keygen -t ed25519 -C $keyComment -f $githubKeyPath
        Ok "GitHub SSH key created."
        Write-Host ""
        Info "Add this public key to GitHub -> Settings -> SSH keys -> New SSH key:"
        Write-Host ""
        Get-Content "${githubKeyPath}.pub"
        Write-Host ""
    }
} else {
    $gitEmail = ""
    Ok "GitHub SSH key skipped."
}

Write-Host ""

# --- Raspberry Pi SSH key
$inp = (Read-Host "  Generate a Raspberry Pi SSH key? [Y/n]").Trim()
if (-not $inp -or $inp -match '^[Yy]') {
    $raspiKeyName = "raspberry"
    if (-not $gitEmail) { $gitEmail = (Read-Host "  Email for key comment (Enter to skip)").Trim() }

    $raspiKeyPath = Join-Path $sshDir $raspiKeyName
    if (Test-Path $raspiKeyPath) {
        Ok "Raspberry Pi SSH key already exists: $raspiKeyPath"
    } else {
        Info "Generating Raspberry Pi SSH key: $raspiKeyPath"
        $keyComment = if ($gitEmail) { $gitEmail } else { "solaris" }
        ssh-keygen -t ed25519 -C $keyComment -f $raspiKeyPath
        Ok "Raspberry Pi SSH key created."
    }

    # ~/.ssh/config — add raspi block if missing
    $sshConfig  = Join-Path $sshDir "config"
    $hasRaspi   = (Test-Path $sshConfig) -and ((Get-Content $sshConfig -Raw -ErrorAction SilentlyContinue) -match "Host raspi")
    if ($hasRaspi) {
        Ok "SSH config already contains 'Host raspi' entry."
    } else {
        Info "Adding raspi entry to $sshConfig..."
        $block = "`nHost raspi`n    HostName 192.168.20.236`n    User username`n    IdentityFile ~/.ssh/$raspiKeyName`n    IdentitiesOnly yes"
        $utf8NoBom = New-Object System.Text.UTF8Encoding $false
        [System.IO.File]::AppendAllText($sshConfig, $block, $utf8NoBom)
        Ok "SSH config updated."
        Warn "Edit $sshConfig - replace 'username' with your actual Raspberry Pi username."
    }
} else {
    Ok "Raspberry Pi SSH key skipped."
}

Write-Host ""

# --- .gitconfig
$inp = (Read-Host "  Configure Git identity (name + email)? [Y/n]").Trim()
if (-not $inp -or $inp -match '^[Yy]') {
    $gitName = (Read-Host "  Git user name").Trim()
    if (-not $gitEmail) { $gitEmail = (Read-Host "  Git email").Trim() }
    if ($gitName)  { git config --global user.name  $gitName  ; Ok "Git name:  $gitName" }
    if ($gitEmail) { git config --global user.email $gitEmail ; Ok "Git email: $gitEmail" }
} else {
    Ok "Git identity skipped. Configure later: git config --global user.name / user.email"
}


# ==============================================================================
#  REPOSITORY
# ==============================================================================
Section "Repository"

$repoUrl          = "git@github.com:Software-Solaris/solaris-software.git"
$defaultCloneDir  = "$env:USERPROFILE\Documents\solaris-software"

Write-Host ""
$inp = (Read-Host "  Clone/update the repository? [Y/n]").Trim()
if (-not $inp -or $inp -match '^[Yy]') {
    $inp = (Read-Host "  Destination directory [$defaultCloneDir]").Trim()
    $cloneDir = if ($inp) { $inp } else { $defaultCloneDir }

    if (Test-Path (Join-Path $cloneDir ".git")) {
        Info "Repository already exists at $cloneDir - pulling latest changes..."
        git -C $cloneDir pull
        git -C $cloneDir submodule update --init --recursive
        Ok "Repository up to date."
    } else {
        Info "Cloning into $cloneDir..."
        git clone --recurse-submodules $repoUrl $cloneDir
        Ok "Repository cloned."
    }
} else {
    Ok "Repository step skipped."
    $cloneDir = $defaultCloneDir
}


# ==============================================================================
#  SUMMARY
# ==============================================================================
Write-Host ""
Write-Host "  +=========================================================+" -ForegroundColor Green
Write-Host "  |   All tools installed successfully.                     |" -ForegroundColor Green
Write-Host "  +=========================================================+" -ForegroundColor Green
Write-Host ""
Write-Host "  Next steps:" -ForegroundColor White
Write-Host ""
Write-Host "  1. Start Docker Desktop and wait for it to show 'Engine running'." -ForegroundColor Gray
Write-Host "     Confirm the WSL2 backend is enabled (see box above)." -ForegroundColor Gray
Write-Host ""
Write-Host "  2. Open the repo in VS Code:" -ForegroundColor Gray
Write-Host "       code $cloneDir" -ForegroundColor White
Write-Host ""
Write-Host "  3. Click 'Reopen in Container' when VS Code prompts." -ForegroundColor Gray
Write-Host "     The first build takes a few minutes." -ForegroundColor Gray
Write-Host ""
Write-Host "  4. Inside the container terminal:" -ForegroundColor Gray
Write-Host "       cd solaris-v1 && idf.py build" -ForegroundColor White
Write-Host ""
