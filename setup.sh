#!/usr/bin/env bash
set -e

# -----------------------------
# Defaults
# -----------------------------
INSTALL_TOOLCHAIN=0
ARCH=0
DEBIAN=0
FEDORA=0

# -----------------------------
# Toolchain check
# -----------------------------
if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    echo "✔ ARM toolchain found:"
    arm-none-eabi-gcc --version
    echo "Skipping toolchain installation."
else
    echo "✘ ARM toolchain not found. Installing..."
    INSTALL_TOOLCHAIN=1
fi

# -----------------------------
# Distro detection
# -----------------------------
if command -v apt >/dev/null 2>&1; then
    echo "Detected Debian/Ubuntu (apt)"
    DEBIAN=1
elif command -v dnf >/dev/null 2>&1; then
    echo "Detected Fedora (dnf)"
    FEDORA=1
elif command -v pacman >/dev/null 2>&1; then
    echo "Detected Arch Linux (pacman)"
    ARCH=1
else
    echo "✘ Unsupported distribution"
    exit 1
fi

# -----------------------------
# Install dependencies
# -----------------------------
if [ "$INSTALL_TOOLCHAIN" = "1" ]; then
    if [ "$ARCH" = "1" ]; then
        sudo pacman -S --needed \
            cmake \
            arm-none-eabi-gcc \
            arm-none-eabi-newlib
    fi

    if [ "$DEBIAN" = "1" ]; then
        sudo apt update
        sudo apt install -y \
            cmake \
            gcc-arm-none-eabi \
            libnewlib-arm-none-eabi \
            libstdc++-arm-none-eabi-newlib
    fi

    if [ "$FEDORA" = "1" ]; then
        sudo dnf install -y \
            cmake \
            arm-none-eabi-gcc-cs \
            arm-none-eabi-newlib
    fi
fi

# -----------------------------
# Pico SDK
# -----------------------------
if [ ! -d pico-sdk ]; then
    git clone https://github.com/raspberrypi/pico-sdk.git
    cp pico-sdk/external/pico_sdk_import.cmake .
else
    echo "✔ pico-sdk already exists, skipping clone"
fi

export PICO_SDK_PATH="$PWD/pico-sdk"

# -----------------------------
# Verification
# -----------------------------
arm-none-eabi-gcc --version
cmake --version

echo "✔ Environment setup complete"

