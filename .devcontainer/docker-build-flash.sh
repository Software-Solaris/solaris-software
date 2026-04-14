#!/bin/bash
# Build solaris-v1 inside the dev container, then flash to the ESP32-S3 via Raspi.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"
WORKSPACE="/home/user/Documents/solaris-software"
PROJECT="${WORKSPACE}/solaris-v1"

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

# ── 4. Release serial port ────────────────────────────────────────────────────

echo ""
echo "$SEP"
echo "  Releasing /dev/ttyACM0 on Raspberry Pi..."
echo "$SEP"
# Close SSH ControlMaster session if one exists
ssh -O exit -o "ControlPath=/tmp/raspi-serial.ctl" raspi 2>/dev/null || true
# Kill any process whose controlling terminal is ttyACM0 (screen, minicom, picocom…)
# fuser misses these because they hold it as ctty, not as a regular fd.
ssh raspi "
    pkill -f '/dev/ttyACM0' 2>/dev/null || true
    screen -wipe 2>/dev/null || true
    sleep 0.3
    echo ok
"
echo "  ✔ Port released."

# ── 5. Flash to ESP32-S3 via Raspi ────────────────────────────────────────────

echo ""
echo "$SEP"
echo "  Flashing via Raspberry Pi..."
echo "$SEP"

BIN=$(ls "${PROJECT}/build/merged-"*.bin 2>/dev/null | head -n1)
if [ -z "$BIN" ]; then
    echo "  ✘ No merged binary found in ${PROJECT}/build/"
    exit 1
fi

echo "  Binary : $(basename "$BIN")"
echo "  Copying to raspi..."
scp "$BIN" raspi:/tmp/merged.bin || { echo "  ✘ scp failed."; exit 1; }

echo "  Running esptool..."
ssh raspi "python3 -m esptool --chip esp32s3 --port /dev/ttyACM0 --baud 460800 \
    --before default-reset --after hard-reset write-flash 0x0 /tmp/merged.bin"
FLASH_EXIT=$?

[ $FLASH_EXIT -ne 0 ] && { echo "  ✘ Flash failed (exit ${FLASH_EXIT})."; exit $FLASH_EXIT; }
echo ""
echo "  ✔ Flash complete."
echo "$SEP"

# ── 6. Open serial monitor in a separate terminal window ─────────────────────

echo ""
echo "$SEP"
echo "  Opening serial monitor..."
echo "$SEP"
export DISPLAY="${DISPLAY:-:0}"
setsid qterminal -e "${SCRIPT_DIR}/docker-serial-monitor.sh" </dev/null >/dev/null 2>&1 &
echo $! > /tmp/solaris-serial.pid
disown $!
echo "  ✔ Serial monitor launched."
