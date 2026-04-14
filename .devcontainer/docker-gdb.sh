#!/bin/bash
# Proxy: runs xtensa GDB inside the Solaris dev container.
# cppdbg communicates over stdin/stdout (GDB MI2) — no TTY needed.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"
GDB_BIN="/opt/esp/tools/xtensa-esp-elf-gdb/16.2_20250324/xtensa-esp-elf-gdb/bin/xtensa-esp32s3-elf-gdb"

CONTAINER_ID=$(docker compose -f "$COMPOSE_FILE" ps -q esp32-dev 2>/dev/null | head -1)
RUNNING=$(docker inspect -f '{{.State.Running}}' "$CONTAINER_ID" 2>/dev/null)

if [ "$RUNNING" != "true" ]; then
    printf 'docker-gdb: esp32-dev container is not running.\nStart it first: docker compose -f %s up -d esp32-dev\n' "$COMPOSE_FILE" >&2
    exit 1
fi

exec docker exec -i "$CONTAINER_ID" "$GDB_BIN" "$@"
