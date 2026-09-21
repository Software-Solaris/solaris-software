#!/bin/bash
# Build solaris-v2 inside the dev container, then flash to the ESP32-S3 via local USB.
# The board must be connected to DEVICE_PORT before running this script.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"
WORKSPACE="/home/user/Documents/solaris-software"
PROJECT="${WORKSPACE}/solaris-v2"
DEVICE_PORT="/dev/ttyACM0"   # Change if the board appears on a different port

SEP="  ──────────────────────────────────────────────────────"

# ── 1. Ensure the esp32-dev container is running ──────────────────────────────

CONTAINER_ID=$(docker compose -f "$COMPOSE_FILE" ps -q esp32-dev 2>/dev/null | head -1)
STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_ID" 2>/dev/null)

if [ -z "$CONTAINER_ID" ] || [ "$STATUS" != "running" ]; then
    echo "$SEP"
    echo "  Starting Solaris dev container..."
    echo "$SEP"

    docker compose -f "$COMPOSE_FILE" up -d esp32-dev || { echo "  ✘ docker compose up failed."; exit 1; }

    echo "  Waiting for container..."
    for i in $(seq 1 30); do
        CONTAINER_ID=$(docker compose -f "$COMPOSE_FILE" ps -q esp32-dev 2>/dev/null | head -1)
        STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_ID" 2>/dev/null)
        [ "$STATUS" = "running" ] && break
        sleep 1
    done

    [ "$STATUS" = "running" ] || { echo "  ✘ Container did not start in time."; exit 1; }
    echo "  ✔ Container ready."
    echo ""
fi

# ── 2. Build ──────────────────────────────────────────────────────────────────

echo "$SEP"
echo "  idf.py build  (inside container)"
echo "$SEP"

docker exec "$CONTAINER_ID" /bin/bash -c \
    "source /opt/esp/idf/export.sh >/dev/null 2>&1 && cd ${PROJECT} && idf.py build"
BUILD_EXIT=$?

[ $BUILD_EXIT -ne 0 ] && { echo "  ✘ Build failed (exit ${BUILD_EXIT})."; exit $BUILD_EXIT; }

# ── 3. Merge binaries ─────────────────────────────────────────────────────────

echo ""
echo "$SEP"
echo "  idf.py merge-bin  (inside container)"
echo "$SEP"

docker exec "$CONTAINER_ID" /bin/bash -c \
    "source /opt/esp/idf/export.sh >/dev/null 2>&1 && cd ${PROJECT} && idf.py merge-bin"
MERGE_EXIT=$?

[ $MERGE_EXIT -ne 0 ] && { echo "  ✘ merge-bin failed (exit ${MERGE_EXIT})."; exit $MERGE_EXIT; }
echo "  ✔ Binary ready."

# ── 4. Flash via local USB ────────────────────────────────────────────────────

echo ""
echo "$SEP"
echo "  Flashing via USB (${DEVICE_PORT})..."
echo "$SEP"

BIN=$(ls "${PROJECT}/build/merged-"*.bin 2>/dev/null | head -n1)
if [ -z "$BIN" ]; then
    echo "  ✘ No merged binary found in ${PROJECT}/build/"
    exit 1
fi

echo "  Binary : $(basename "$BIN")"

docker exec "$CONTAINER_ID" /bin/bash -c \
    "source /opt/esp/idf/export.sh >/dev/null 2>&1 && \
     python3 -m esptool --chip esp32s3 --port ${DEVICE_PORT} --baud 460800 \
         --before default_reset --after hard_reset write_flash 0x0 ${BIN}"
FLASH_EXIT=$?

[ $FLASH_EXIT -ne 0 ] && { echo "  ✘ Flash failed (exit ${FLASH_EXIT})."; exit $FLASH_EXIT; }
echo ""
echo "  ✔ Flash complete."
echo "$SEP"
