#!/bin/bash
# ==============================================================================
# Helper Script: Set UART permissions and start MicroXRCEAgent
# ==============================================================================

DEVICE="/dev/ttyAMA0"
BAUDRATE="921600"

echo "Setting permissions for $DEVICE..."
sudo chmod 666 "$DEVICE"

echo "Starting MicroXRCEAgent on $DEVICE at $BAUDRATE baud..."
MicroXRCEAgent serial --dev "$DEVICE" -b "$BAUDRATE"
