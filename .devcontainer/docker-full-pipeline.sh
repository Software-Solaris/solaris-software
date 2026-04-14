#!/bin/bash
# Full debug pipeline: build → merge-bin → flash → SSH tunnel → reset halt.
# Runs entirely in one terminal so the output is visible end to end.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SEP="  ──────────────────────────────────────────────────────"

# ── 1. Build + Flash ──────────────────────────────────────────────────────────

"${SCRIPT_DIR}/docker-build-flash.sh" || exit $?

# ── 2. SSH tunnel ─────────────────────────────────────────────────────────────

echo ""
echo "$SEP"
echo "  Opening SSH tunnel to Raspberry Pi..."
echo "$SEP"

ssh \
    -o ControlMaster=auto \
    -o ControlPersist=600 \
    -o "ControlPath=/tmp/raspi-openocd-3337.ctl" \
    -o ExitOnForwardFailure=yes \
    -o StrictHostKeyChecking=accept-new \
    -fN \
    -L 127.0.0.1:3337:127.0.0.1:3333 \
    -L 127.0.0.1:4447:127.0.0.1:4444 \
    -L 127.0.0.1:6667:127.0.0.1:6666 \
    raspi

SSH_EXIT=$?
[ $SSH_EXIT -ne 0 ] && { echo "  ✘ SSH tunnel failed (exit ${SSH_EXIT})."; exit $SSH_EXIT; }
echo "  ✔ Tunnel open."

# ── 3. Wait for OpenOCD ───────────────────────────────────────────────────────

echo "  Waiting for OpenOCD on :3337..."
for i in $(seq 1 40); do
    (echo > /dev/tcp/127.0.0.1/3337) >/dev/null 2>&1 && break
    sleep 0.1
done
(echo > /dev/tcp/127.0.0.1/3337) >/dev/null 2>&1 \
    || echo "  (aviso) OpenOCD aún no escucha :3337" >&2

# ── 4. Reset halt ─────────────────────────────────────────────────────────────

echo "  Sending reset halt..."
(printf "reset halt\n" > /dev/tcp/127.0.0.1/4447) >/dev/null 2>&1 || true

echo "  ✔ Board halted — GDB connecting."
echo "$SEP"
