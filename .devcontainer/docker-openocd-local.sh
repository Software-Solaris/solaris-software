#!/bin/bash
# Starts OpenOCD inside the Solaris dev container in detached mode using the
# ESP32-S3 built-in USB-JTAG. The container must be running (privileged) and
# the board connected via USB.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"

CONTAINER_ID=$(docker compose -f "$COMPOSE_FILE" ps -q esp32-dev 2>/dev/null | head -1)
RUNNING=$(docker inspect -f '{{.State.Running}}' "$CONTAINER_ID" 2>/dev/null)

if [ "$RUNNING" != "true" ]; then
    printf 'docker-openocd: esp32-dev container is not running.\nStart it: docker compose -f %s up -d esp32-dev\n' "$COMPOSE_FILE" >&2
    exit 1
fi

# Kill any leftover OpenOCD instance before starting a new one
docker exec "$CONTAINER_ID" pkill openocd 2>/dev/null || true
sleep 0.3

# Start OpenOCD detached — log goes to /tmp/openocd.log inside the container
docker exec -d "$CONTAINER_ID" bash -c \
    "source /opt/esp/idf/export.sh >/dev/null 2>&1 && openocd -f board/esp32s3-builtin.cfg > /tmp/openocd.log 2>&1"

echo "OpenOCD started in background (log: docker exec <container> cat /tmp/openocd.log)"
