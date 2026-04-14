#!/bin/bash
# Closes the serial monitor SSH session opened by docker-serial-monitor.sh.
ssh -O exit -o "ControlPath=/tmp/raspi-serial.ctl" raspi 2>/dev/null || true
