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
        root)       cd "$SOLARIS_ROOT" ;;
        v1)         cd "$SOLARIS_ROOT/solaris-v1" ;;
        main)       cd "$SOLARIS_ROOT/solaris-v1/main" ;;
        spp)        cd "$SOLARIS_ROOT/solaris-v1/spp" ;;
        ports)      cd "$SOLARIS_ROOT/solaris-v1/spp/ports" ;;
        services)   cd "$SOLARIS_ROOT/solaris-v1/spp/services" ;;
        compiler)   cd "$SOLARIS_ROOT/solaris-v1/compiler" ;;
        tests)      cd "$SOLARIS_ROOT/solaris-v1/spp/tests/unit" ;;
        docs)       cd "$SOLARIS_ROOT/docs" ;;
        *)
            printf "\n  \033[1;33mUsage:\033[0m goto <destination>\n\n"
            printf "  \033[1;32m%-14s\033[0m %s\n" "root"      "solaris-software/"
            printf "  \033[1;32m%-14s\033[0m %s\n" "v1"        "solaris-v1/"
            printf "  \033[1;32m%-14s\033[0m %s\n" "main"      "solaris-v1/main/"
            printf "  \033[1;32m%-14s\033[0m %s\n" "spp"       "solaris-v1/spp/"
            printf "  \033[1;32m%-14s\033[0m %s\n" "ports"     "solaris-v1/spp/ports/"
            printf "  \033[1;32m%-14s\033[0m %s\n" "services"  "solaris-v1/spp/services/"
            printf "  \033[1;32m%-14s\033[0m %s\n" "compiler"  "solaris-v1/compiler/"
            printf "  \033[1;32m%-14s\033[0m %s\n" "tests"     "solaris-v1/spp/tests/unit/"
            printf "  \033[1;32m%-14s\033[0m %s\n" "docs"      "docs/"
            echo ""
            ;;
    esac
}

# ─── Unit Testing ─────────────────────────────────────────────────────────────

test() {
    local input_path="$SOLARIS_ROOT/solaris-v1/spp/tests"

    if [ ! -d "$input_path" ]; then
        printf "\n  \033[1;31m✘\033[0m  Directory not found: %s\n\n" "$input_path"
        return 1
    fi

    # Walk up from input_path to find the CMakeLists.txt root
    local cmake_root="$input_path"
    while [ ! -f "$cmake_root/CMakeLists.txt" ]; do
        [ "$cmake_root" = "/" ] && {
            printf "\n  \033[1;31m✘\033[0m  No CMakeLists.txt found above: %s\n\n" "$input_path"
            return 1
        }
        cmake_root="$(dirname "$cmake_root")"
    done

    local build_dir="$cmake_root/build"
    local L="\033[1;36m  $(printf '─%.0s' {1..54})\033[0m"

    echo -e "\n$L"
    printf "  \033[1;37mSolaris Unit Tests\033[0m\n"
    echo -e "$L"
    printf "  \033[0;37mPath:\033[0m    %s\n" "solaris-v1/spp/tests/core"
    printf "  \033[0;37mBuild:\033[0m   %s\n" "$build_dir"
    echo -e "$L\n"

    # ── Configure ────────────────────────────────────────────────────────────
    printf "  \033[1;33m[1/3]\033[0m Configuring...\n"
    rm -rf "$build_dir"
    mkdir -p "$build_dir"
    cmake -S "$cmake_root" -B "$build_dir" -DCMAKE_BUILD_TYPE=Debug -DSPP_BUILD_TESTS=ON -DSPP_PORT=posix 2>&1 | sed 's/^/       /'
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

# ─── Doxygen Template ─────────────────────────────────────────────────────────

template() {
    local L="\033[1;36m  $(printf '─%.0s' {1..54})\033[0m"
    local filter="${1:-all}"

    echo -e "\n$L"
    echo -e "\033[1;37m  Solaris Doxygen Templates\033[0m"
    echo -e "$L\n"

    if [[ "$filter" == "all" || "$filter" == "h" ]]; then
        echo -e "  \033[1;33m── File header (.h) ──────────────────────────────────\033[0m\n"
        cat <<'EOF'
/**
 * @file module.h
 * @brief One-line description of this header.
 *
 * Extended description if needed. Describe the purpose of the
 * module, not the implementation details.
 *
 * Naming conventions used in this file:
 *  - Constants / macros : K_SPP_MODULE_*
 *  - Types              : SPP_ModuleName_t
 *  - Public functions   : SPP_Module_functionName()
 *  - Pointer params     : p_paramName
 *  - Static module vars : s_varName
 */

#ifndef SPP_MODULE_H
#define SPP_MODULE_H

#include "spp/core/returntypes.h"
#include "spp/core/types.h"

/* ----------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------- */

/** @brief <description of constant>. */
#define K_SPP_MODULE_EXAMPLE  (42U)

/* ----------------------------------------------------------------
 * Types
 * ---------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * Public Functions
 * ---------------------------------------------------------------- */

#endif /* SPP_MODULE_H */
EOF
        echo ""
    fi

    if [[ "$filter" == "all" || "$filter" == "c" ]]; then
        echo -e "  \033[1;33m── File header (.c) ──────────────────────────────────\033[0m\n"
        cat <<'EOF'
/**
 * @file module.c
 * @brief Implementation of <module name>.
 *
 * See module.h for the public API.
 */

#include "spp/module.h"

/* ----------------------------------------------------------------
 * Static Variables
 * ---------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * Private Functions
 * ---------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * Public Functions
 * ---------------------------------------------------------------- */
EOF
        echo ""
    fi

    if [[ "$filter" == "all" || "$filter" == "fn" ]]; then
        echo -e "  \033[1;33m── Function ──────────────────────────────────────────\033[0m\n"
        cat <<'EOF'
/**
 * @brief Short description of what the function does.
 *
 * Optional extended description. Explain preconditions,
 * side effects, or threading constraints if relevant.
 *
 * @param[in]     p_cfg    Pointer to configuration struct. Must not be NULL.
 * @param[in,out] p_ctx    Pointer to context, updated on success.
 * @param[out]    p_result Pointer to output value.
 *
 * @return SPP_OK on success.
 * @return SPP_ERROR_NULL_POINTER if any pointer argument is NULL.
 * @return SPP_ERROR_INVALID_PARAMETER if configuration is invalid.
 */
retval_t SPP_Module_functionName(const SPP_ModuleCfg_t *p_cfg,
                                 SPP_ModuleCtx_t       *p_ctx,
                                 spp_uint32_t          *p_result);
EOF
        echo ""
    fi

    if [[ "$filter" == "all" || "$filter" == "struct" ]]; then
        echo -e "  \033[1;33m── Struct / typedef ──────────────────────────────────\033[0m\n"
        cat <<'EOF'
/**
 * @brief Short description of what this struct represents.
 *
 * Extended description if needed (ownership, lifecycle, etc.).
 */
typedef struct {
    spp_uint32_t  fieldOne;   /**< @brief Description of fieldOne. */
    spp_uint16_t  fieldTwo;   /**< @brief Description of fieldTwo. */
    spp_bool_t    isEnabled;  /**< @brief True if the module is active. */
    void         *p_handle;   /**< @brief Pointer to the underlying resource. */
} SPP_ModuleName_t;
EOF
        echo ""
    fi

    if [[ "$filter" == "all" || "$filter" == "enum" ]]; then
        echo -e "  \033[1;33m── Enum ──────────────────────────────────────────────\033[0m\n"
        cat <<'EOF'
/**
 * @brief Short description of what this enum represents.
 */
typedef enum {
    K_SPP_MODULE_STATE_IDLE    = 0, /**< @brief Module is idle, not started. */
    K_SPP_MODULE_STATE_RUNNING = 1, /**< @brief Module is running normally.  */
    K_SPP_MODULE_STATE_ERROR   = 2, /**< @brief Module encountered an error. */
} SPP_ModuleState_t;
EOF
        echo ""
    fi

    if [[ "$filter" == "all" || "$filter" == "macro" ]]; then
        echo -e "  \033[1;33m── Macro / constant ──────────────────────────────────\033[0m\n"
        cat <<'EOF'
/** @brief Maximum number of items in the module pool. */
#define K_SPP_MODULE_MAX_ITEMS   (16U)

/** @brief Default timeout in milliseconds. */
#define K_SPP_MODULE_TIMEOUT_MS  (5000U)

/**
 * @brief Compute the size of an array at compile time.
 * @param arr  Array whose size is computed (must not be a pointer).
 */
#define SPP_ARRAY_SIZE(arr)  (sizeof(arr) / sizeof((arr)[0]))
EOF
        echo ""
    fi

    echo -e "  \033[0;37mUsage: \033[1;32mtemplate\033[0;37m [h|c|fn|struct|enum|macro]\033[0m"
    echo -e "$L\n"
}

# ─── Help ─────────────────────────────────────────────────────────────────────

help() {
    local L="\033[1;36m  $(printf '─%.0s' {1..54})\033[0m"
    echo -e "\n$L"
    echo -e "\033[1;37m  Solaris Dev Container — Quick Reference\033[0m"
    echo -e "$L"

    echo -e "\n  \033[1;33mESP-IDF Aliases\033[0m"
    printf "  \033[1;32m%-14s\033[0m %s\n" "build"     "idf.py build && idf.py merge-bin"
    printf "  \033[1;32m%-14s\033[0m %s\n" "flash"     "idf.py flash"
    printf "  \033[1;32m%-14s\033[0m %s\n" "monitor"   "idf.py monitor"
    printf "  \033[1;32m%-14s\033[0m %s\n" "fullflash" "idf.py build flash monitor"
    printf "  \033[1;32m%-14s\033[0m %s\n" "size"      "idf.py size"
    printf "  \033[1;32m%-14s\033[0m %s\n" "clean"     "idf.py fullclean"

    echo -e "\n  \033[1;33mNavigation  →  goto <destination>\033[0m"
    printf "  \033[1;32m%-18s\033[0m %s\n" "goto root"      "solaris-software/"
    printf "  \033[1;32m%-18s\033[0m %s\n" "goto v1"        "solaris-v1/"
    printf "  \033[1;32m%-18s\033[0m %s\n" "goto main"      "solaris-v1/main/"
    printf "  \033[1;32m%-18s\033[0m %s\n" "goto spp"       "solaris-v1/spp/"
    printf "  \033[1;32m%-18s\033[0m %s\n" "goto ports"     "solaris-v1/spp/ports/"
    printf "  \033[1;32m%-18s\033[0m %s\n" "goto services"  "solaris-v1/spp/services/"
    printf "  \033[1;32m%-18s\033[0m %s\n" "goto compiler"  "solaris-v1/compiler/"
    printf "  \033[1;32m%-18s\033[0m %s\n" "goto tests"     "solaris-v1/spp/tests/unit/"
    printf "  \033[1;32m%-18s\033[0m %s\n" "goto docs"      "docs/"

    echo -e "\n  \033[1;33mDoxygen Templates  →  template [type]\033[0m"
    printf "  \033[1;32m%-18s\033[0m %s\n" "template"        "All templates"
    printf "  \033[1;32m%-18s\033[0m %s\n" "template h"      "File header (.h)"
    printf "  \033[1;32m%-18s\033[0m %s\n" "template c"      "File header (.c)"
    printf "  \033[1;32m%-18s\033[0m %s\n" "template fn"     "Function"
    printf "  \033[1;32m%-18s\033[0m %s\n" "template struct" "Struct / typedef"
    printf "  \033[1;32m%-18s\033[0m %s\n" "template enum"   "Enum"
    printf "  \033[1;32m%-18s\033[0m %s\n" "template macro"  "Macro / constant"

    echo -e "\n  \033[1;33mUnit Testing\033[0m"
    printf "  \033[1;32m%-14s\033[0m %s\n" "test" "cmake + build + ctest  (solaris-v1/spp/tests/core)"

    echo -e "\n  \033[1;33mPrompt\033[0m"
    echo -e "  \033[1;35m(branch \033[1;33m★\033[1;35m)\033[0m  Uncommitted changes present"
    echo -e "  \033[1;31m✗ N\033[0m      Last command exited with code N"

    echo -e "\n  \033[1;33mHistory\033[0m"
    echo -e "  10 000 entries with timestamps, no duplicates.  \033[1;32mCtrl+R\033[0m to search."

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
