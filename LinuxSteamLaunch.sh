#!/bin/sh

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ARCH=$(uname -m)

if [ "$ARCH" = "x86_64" ]; then
    exec "$SCRIPT_DIR/SpaceCalibratorx64" "$@"
elif [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    if [ -e "$SCRIPT_DIR/SpaceCalibratorArm64" ]; then
        exec "$SCRIPT_DIR/SpaceCalibratorArm64" "$@"
    else
        echo "SpaceCalibrator: aarch64 binary missing, falling back to x64..."
        exec "$SCRIPT_DIR/SpaceCalibratorx64" "$@"
    fi
else
    echo "SpaceCalibrator: Unsupported architecture $ARCH"
    exit 1
fi