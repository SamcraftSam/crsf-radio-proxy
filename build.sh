#!/usr/bin/env bash
set -e

CLEAN=false

for arg in "$@"; do
  case $arg in
    --clean) CLEAN=true ;;
    *) ;;
  esac
done

mkdir -p build
cd build

if [ "$CLEAN" = true ] && [ -f Makefile ]; then
    make clean
fi

cmake ..
make

if command -v picotool >/dev/null 2>&1; then
    #picotool reboot -u
    picotool load -f main_app.elf
    picotool reboot
fi

