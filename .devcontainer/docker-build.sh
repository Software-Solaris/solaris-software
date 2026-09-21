#!/bin/bash
# Ensures the Solaris dev container is running, then builds solaris-v2 inside it.
# Exits with the build's exit code so VS Code can report success or failure.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"
WORKSPACE="/home/user/Documents/solaris-software"
PROJECT="${WORKSPACE}/solaris-v2"

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

# ── 3. Merge binaries (needed by flash-esp32s3-remote-only) ──────────────────

echo ""
echo "$SEP"
echo "  idf.py merge-bin  (inside container)"
echo "$SEP"

docker exec "$CONTAINER_ID" /bin/bash -c \
    "source /opt/esp/idf/export.sh >/dev/null 2>&1 && cd ${PROJECT} && idf.py merge-bin"
MERGE_EXIT=$?

[ $MERGE_EXIT -ne 0 ] && { echo "  ✘ merge-bin failed (exit ${MERGE_EXIT})."; exit $MERGE_EXIT; }

echo "  ✔ Build complete."
