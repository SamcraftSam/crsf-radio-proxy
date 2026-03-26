#!/usr/bin/env bash
set -e


sudo dnf install -y \
  git cmake ninja-build gcc-c++ make pkgconf-pkg-config \
  libusb1-devel libudev-devel

# Check if picotool already exists
if command -v picotool >/dev/null 2>&1; then
    echo "picotool is already installed."
    exit 0
fi


export PICO_SDK_PATH="$PWD/pico-sdk"

# Directory to clone into
REPO_DIR="$PWD/picotool"

# Clone if not already present
if [ ! -d "$REPO_DIR" ]; then
    git clone https://github.com/raspberrypi/picotool.git "$REPO_DIR"
fi

cd "$REPO_DIR"

# Initialize submodules (only what picotool needs)
git submodule update --init

# Build
mkdir -p build
cd build
cmake ..
make

# Install
sudo make install

echo "picotool installed successfully."

