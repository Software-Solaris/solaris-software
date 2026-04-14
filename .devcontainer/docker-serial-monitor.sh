#!/bin/bash
# Runs inside the serial terminal window.
# 'exec' is replaced by a call that keeps the window open on error.
ssh \
    -o ControlMaster=auto \
    -o ControlPersist=yes \
    -o "ControlPath=/tmp/raspi-serial.ctl" \
    -o StrictHostKeyChecking=accept-new \
    -t raspi 'bash -ic serial'
echo ""
echo "  Serial session ended. Press Enter to close."
read -r
