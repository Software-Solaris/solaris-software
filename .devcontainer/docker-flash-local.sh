#!/bin/bash
# Flash the already-built merged binary to the ESP32-S3 via local USB.
# Run docker-build-flash-local.sh first if you need to rebuild.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"
PROJECT="/home/user/Documents/solaris-software/solaris-v2"
DEVICE_PORT="/dev/ttyACM0"

SEP="  ──────────────────────────────────────────────────────"

CONTAINER_ID=$(docker compose -f "$COMPOSE_FILE" ps -q esp32-dev 2>/dev/null | head -1)
STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_ID" 2>/dev/null)

if [ -z "$CONTAINER_ID" ] || [ "$STATUS" != "running" ]; then
    echo "  ✘ Container is not running. Start it first."
    exit 1
fi

BIN=$(ls "${PROJECT}/build/merged-"*.bin 2>/dev/null | head -n1)
if [ -z "$BIN" ]; then
    echo "  ✘ No merged binary found in ${PROJECT}/build/  (run build first)"
    exit 1
fi

echo "$SEP"
echo "  Flashing via USB (${DEVICE_PORT})"
echo "  Binary : $(basename "$BIN")"
echo "$SEP"

docker exec "$CONTAINER_ID" /bin/bash -c \
    "source /opt/esp/idf/export.sh >/dev/null 2>&1 && \
     python3 -m esptool --chip esp32s3 --port ${DEVICE_PORT} --baud 460800 \
         --before default_reset --after hard_reset write_flash 0x0 ${BIN}"
FLASH_EXIT=$?

[ $FLASH_EXIT -ne 0 ] && { echo "  ✘ Flash failed (exit ${FLASH_EXIT})."; exit $FLASH_EXIT; }
echo ""
echo "  ✔ Flash complete."
echo "$SEP"
