#!/bin/bash
# Opens an interactive shell inside the running Solaris dev container.
# Used by the VSCodium "Solaris Dev Container" terminal profile.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"
INIT_FILE="/home/user/Documents/solaris-software/.devcontainer/setup-terminal.sh"

# ── Find the esp32-dev container ──────────────────────────────────────────────
CONTAINER_ID=$(docker compose -f "$COMPOSE_FILE" ps -q esp32-dev 2>/dev/null | head -1)

if [ -z "$CONTAINER_ID" ]; then
    echo ""
    echo "  ✘  Solaris dev container is not running."
    echo ""
    echo "  Start it with:"
    echo "    docker compose -f .devcontainer/docker-compose.yml up -d"
    echo ""
    exit 1
fi

STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_ID" 2>/dev/null)
if [ "$STATUS" != "running" ]; then
    echo ""
    echo "  ✘  Container found but not running  (status: ${STATUS})"
    echo ""
    echo "  Restart it with:"
    echo "    docker compose -f .devcontainer/docker-compose.yml up -d"
    echo ""
    exit 1
fi

# ── Attach ────────────────────────────────────────────────────────────────────
exec docker exec -it "$CONTAINER_ID" /bin/bash --init-file "$INIT_FILE"
