#!/bin/bash

# ─── Git helpers ──────────────────────────────────────────────────────────────

parse_git_branch() {
    git branch 2>/dev/null | grep '^*' | sed 's/^* //'
}

# ─── Prompt (PROMPT_COMMAND captures $? before anything overwrites it) ────────

_solaris_prompt() {
    local exit_code=$?
    local branch dirty exit_part branch_part

    branch=$(parse_git_branch)
    [[ -n "$(git status --porcelain --ignore-submodules=dirty 2>/dev/null)" ]] && dirty=1

    if [ -n "$branch" ]; then
        if [ -n "$dirty" ]; then
            branch_part=" \[\033[1;35m\](\[\033[1;33m\]${branch} ★\[\033[1;35m\])\[\033[0m\]"
        else
            branch_part=" \[\033[1;35m\](\[\033[1;32m\]${branch}\[\033[1;35m\])\[\033[0m\]"
        fi
    fi

    [ $exit_code -ne 0 ] && exit_part=" \[\033[1;31m\]✗ ${exit_code}\[\033[0m\]"

    PS1="\[\033[1;36m\]╭\[\033[0m\] \[\033[1;34m\]\w\[\033[0m\]${branch_part}${exit_part}\n\[\033[1;36m\]╰❯\[\033[0m\] "
}
PROMPT_COMMAND='_solaris_prompt'

# ─── History ──────────────────────────────────────────────────────────────────

HISTSIZE=10000
HISTFILESIZE=20000
HISTTIMEFORMAT="%Y-%m-%d %H:%M:%S  "
HISTCONTROL=ignoreboth:erasedups
shopt -s histappend

# ─── Autocomplete ─────────────────────────────────────────────────────────────

bind "set completion-ignore-case on"  2>/dev/null
bind "set show-all-if-ambiguous on"   2>/dev/null
bind "set colored-stats on"           2>/dev/null

# ─── ESP-IDF aliases ──────────────────────────────────────────────────────────

alias build='idf.py build && idf.py merge-bin'
alias flash='idf.py flash'
alias monitor='idf.py monitor'
alias fullflash='idf.py build flash monitor'
alias size='idf.py size'
alias clean='idf.py fullclean'

# ─── Navigation ───────────────────────────────────────────────────────────────

SOLARIS_ROOT="/home/user/Documents/solaris-software"

goto() {
    case "$1" in
        root)        cd "$SOLARIS_ROOT" ;;
        v1)          cd "$SOLARIS_ROOT/solaris-v1" ;;
        main)        cd "$SOLARIS_ROOT/solaris-v1/main" ;;
        components)  cd "$SOLARIS_ROOT/solaris-v1/components" ;;
        icm)         cd "$SOLARIS_ROOT/solaris-v1/components/icm_driver" ;;
        bmp)         cd "$SOLARIS_ROOT/solaris-v1/components/pressureSensorDriver" ;;
        datalogger)  cd "$SOLARIS_ROOT/solaris-v1/components/datalogger_driver" ;;
        spp)         cd "$SOLARIS_ROOT/solaris-v1/external/spp" ;;
        ports)       cd "$SOLARIS_ROOT/solaris-v1/external/spp-ports" ;;
        *)
            printf "\n  \033[1;33mUsage:\033[0m goto <destination>\n\n"
            printf "  \033[1;32m%-16s\033[0m %s\n" "root"        "solaris-software/"
            printf "  \033[1;32m%-16s\033[0m %s\n" "v1"          "solaris-v1/"
            printf "  \033[1;32m%-16s\033[0m %s\n" "main"        "solaris-v1/main/"
            printf "  \033[1;32m%-16s\033[0m %s\n" "components"  "solaris-v1/components/"
            printf "  \033[1;32m%-16s\033[0m %s\n" "icm"         "components/icm_driver/"
            printf "  \033[1;32m%-16s\033[0m %s\n" "bmp"         "components/pressureSensorDriver/"
            printf "  \033[1;32m%-16s\033[0m %s\n" "datalogger"  "components/datalogger_driver/"
            printf "  \033[1;32m%-16s\033[0m %s\n" "spp"         "external/spp/"
            printf "  \033[1;32m%-16s\033[0m %s\n" "ports"       "external/spp-ports/"
            echo ""
            ;;
    esac
}

# ─── Unit Testing ─────────────────────────────────────────────────────────────

run_tests() {
    local test_path="$1"

    if [ -z "$test_path" ]; then
        printf "\n  \033[1;33mUsage:\033[0m test <path/to/tests>\n\n"
        printf "  Compiles and runs Cgreen unit tests in the given directory.\n"
        printf "  The directory must contain a CMakeLists.txt.\n\n"
        return 1
    fi

    # Resolve to absolute path
    [[ "$test_path" != /* ]] && test_path="$(pwd)/$test_path"
    test_path="$(realpath "$test_path" 2>/dev/null || echo "$test_path")"

    if [ ! -d "$test_path" ]; then
        printf "\n  \033[1;31m✘\033[0m  Directory not found: %s\n\n" "$test_path"
        return 1
    fi

    if [ ! -f "$test_path/CMakeLists.txt" ]; then
        printf "\n  \033[1;31m✘\033[0m  No CMakeLists.txt in: %s\n\n" "$test_path"
        return 1
    fi

    local build_dir="$test_path/build"
    local L="\033[1;36m  $(printf '─%.0s' {1..54})\033[0m"

    echo -e "\n$L"
    printf "  \033[1;37mSolaris Unit Tests\033[0m\n"
    echo -e "$L"
    printf "  \033[0;37mSource:\033[0m  %s\n" "$test_path"
    printf "  \033[0;37mBuild:\033[0m   %s\n" "$build_dir"
    echo -e "$L\n"

    # ── Configure ────────────────────────────────────────────────────────────
    printf "  \033[1;33m[1/3]\033[0m Configuring...\n"
    rm -rf "$build_dir"
    mkdir -p "$build_dir"
    cmake -S "$test_path" -B "$build_dir" -DCMAKE_BUILD_TYPE=Debug 2>&1 | sed 's/^/       /'
    if [ "${PIPESTATUS[0]}" -ne 0 ]; then
        printf "\n  \033[1;31m✘  cmake failed.\033[0m\n\n"
        return 1
    fi

    # ── Build ────────────────────────────────────────────────────────────────
    echo ""
    printf "  \033[1;33m[2/3]\033[0m Building...\n"
    cmake --build "$build_dir" --parallel 2>&1 | sed 's/^/       /'
    if [ "${PIPESTATUS[0]}" -ne 0 ]; then
        printf "\n  \033[1;31m✘  Build failed.\033[0m\n\n"
        return 1
    fi

    # ── Run tests ────────────────────────────────────────────────────────────
    echo ""
    printf "  \033[1;33m[3/3]\033[0m Running tests...\n\n"

    pushd "$build_dir" > /dev/null
    ctest --output-on-failure --no-compress-output 2>&1 | sed 's/^/  /'
    local exit_code=${PIPESTATUS[0]}
    popd > /dev/null

    echo -e "\n$L"
    if [ "$exit_code" -eq 0 ]; then
        printf "  \033[1;32m✔  All tests passed.\033[0m\n"
    else
        printf "  \033[1;31m✘  Some tests failed  (exit %d).\033[0m\n" "$exit_code"
    fi
    echo -e "$L\n"

    return "$exit_code"
}

# ─── Help ─────────────────────────────────────────────────────────────────────

help() {
    local L="\033[1;36m  $(printf '─%.0s' {1..54})\033[0m"
    echo -e "\n$L"
    echo -e "\033[1;37m  Solaris Dev Container — Quick Reference\033[0m"
    echo -e "$L"

    echo -e "\n  \033[1;33mESP-IDF Aliases\033[0m"
    printf "  \033[1;32m%-14s\033[0m %s\n" "build"     "idf.py build"
    printf "  \033[1;32m%-14s\033[0m %s\n" "flash"     "idf.py flash"
    printf "  \033[1;32m%-14s\033[0m %s\n" "monitor"   "idf.py monitor"
    printf "  \033[1;32m%-14s\033[0m %s\n" "fullflash" "idf.py build flash monitor"
    printf "  \033[1;32m%-14s\033[0m %s\n" "size"      "idf.py size"
    printf "  \033[1;32m%-14s\033[0m %s\n" "clean"     "idf.py fullclean"

    echo -e "\n  \033[1;33mNavigation  →  goto <destination>\033[0m"
    printf "  \033[1;32m%-16s\033[0m %s\n" "goto root"       "solaris-software/"
    printf "  \033[1;32m%-16s\033[0m %s\n" "goto v1"         "solaris-v1/"
    printf "  \033[1;32m%-16s\033[0m %s\n" "goto main"       "solaris-v1/main/"
    printf "  \033[1;32m%-16s\033[0m %s\n" "goto components" "solaris-v1/components/"
    printf "  \033[1;32m%-16s\033[0m %s\n" "goto icm"        "components/icm_driver/"
    printf "  \033[1;32m%-16s\033[0m %s\n" "goto bmp"        "components/pressureSensorDriver/"
    printf "  \033[1;32m%-16s\033[0m %s\n" "goto datalogger" "components/datalogger_driver/"
    printf "  \033[1;32m%-16s\033[0m %s\n" "goto spp"        "external/spp/"
    printf "  \033[1;32m%-16s\033[0m %s\n" "goto ports"      "external/spp-ports/"

    echo -e "\n  \033[1;33mPrompt\033[0m"
    echo -e "  \033[1;35m(branch \033[1;33m★\033[1;35m)\033[0m  Uncommitted changes present"
    echo -e "  \033[1;31m✗ N\033[0m      Last command exited with code N"

    echo -e "\n  \033[1;33mHistory\033[0m"
    echo -e "  10 000 entries with timestamps, no duplicates.  \033[1;32mCtrl+R\033[0m to search."

    echo -e "\n  \033[1;33mUnit Testing  →  run_tests <path>\033[0m"
    printf "  \033[1;32m%-30s\033[0m %s\n" "run_tests <path/to/tests>" "cmake build + ctest (Cgreen)"

    echo -e "\n  \033[1;33mRaspberry Pi  (192.168.20.236)\033[0m"
    echo -e "  \033[1;32mssh raspi\033[0m   SSH into the flashing / OpenOCD station."

    echo -e "\n$L\n"
}

# ─── Welcome banner ───────────────────────────────────────────────────────────

clear

HOUR=$(date +%H)
GIT_USER=$(git config --global user.name 2>/dev/null)
GIT_USER="${GIT_USER:-Developer}"

if   [ "$HOUR" -ge 6  ] && [ "$HOUR" -lt 12 ]; then GREETING="Buenos días"
elif [ "$HOUR" -ge 12 ] && [ "$HOUR" -lt 20 ]; then GREETING="Buenas tardes"
else                                                  GREETING="Buenas noches"
fi

# Source ESP-IDF if IDF_PATH is set but idf.py is not yet in PATH
# (happens in compose+exec mode — postCreateCommand never ran, --init-file skips .bashrc)
if [ -n "$IDF_PATH" ] && ! command -v idf.py >/dev/null 2>&1; then
    source "$IDF_PATH/export.sh" >/dev/null 2>&1
fi

# Collect all status data before printing (avoids interleaved delays)
if [ -n "$IDF_PATH" ]; then
    if [ -n "$IDF_VERSION" ]; then
        IDF_VER="${IDF_VERSION#v}"
    elif command -v idf.py >/dev/null 2>&1; then
        IDF_VER=$(idf.py --version 2>/dev/null | sed 's/ESP-IDF v//')
    else
        IDF_VER="?"
    fi
fi

BRANCH=$(parse_git_branch)
CHANGES=$(git status --porcelain --ignore-submodules=dirty 2>/dev/null | wc -l | tr -d ' ')

timeout 2 bash -c "echo > /dev/tcp/192.168.20.236/22" >/dev/null 2>&1 && RASPI_OK=1 || RASPI_OK=0

# ── Print ──────────────────────────────────────────────────────────────────────

L="\033[1;36m  $(printf '─%.0s' {1..54})\033[0m"

printf "  \033[1;36m%s\033[0m\n" "$(printf '▄%.0s' {1..54})"
printf "  \033[1;37m%s\033[0m\n" "  SOLARIS  ·  Software Development Terminal"
printf "  \033[1;36m%s\033[0m\n" "$(printf '▀%.0s' {1..54})"
printf "  \033[1;33m%s, %s\033[0m\n" "$GREETING" "$GIT_USER"
printf "  \033[0;37m%s\033[0m\n" "$(date '+%A, %d %B %Y  ·  %H:%M:%S')"
echo ""

printf "  \033[0;37m%-10s\033[0m" "ESP-IDF"
if [ -n "$IDF_VER" ]; then
    printf "\033[1;32m✔\033[0m  v%s\n" "$IDF_VER"
else
    printf "\033[1;31m✘\033[0m  not loaded\n"
fi

printf "  \033[0;37m%-10s\033[0m" "Git"
if [ -n "$BRANCH" ]; then
    if [ "$CHANGES" -gt 0 ]; then
        printf "\033[1;34m%s\033[0m  \033[1;33m★ %s change(s)\033[0m\n" "$BRANCH" "$CHANGES"
    else
        printf "\033[1;34m%s\033[0m  \033[1;32m✔ clean\033[0m\n" "$BRANCH"
    fi
else
    printf "\033[0;37mnot a git repository\033[0m\n"
fi

printf "  \033[0;37m%-10s\033[0m" "Raspi"
if [ "$RASPI_OK" -eq 1 ]; then
    printf "\033[1;32m✔\033[0m  192.168.20.236  →  ssh raspi\n"
else
    printf "\033[1;31m✘\033[0m  192.168.20.236 unreachable\n"
fi

echo ""
echo -e "$L"
echo -e "  \033[0;37mType \033[1;32mhelp\033[0;37m for the full command reference.\033[0m"
echo -e "$L\n"
