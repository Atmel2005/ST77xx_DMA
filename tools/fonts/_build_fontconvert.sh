#!/usr/bin/env bash
set -euo pipefail
export MSYSTEM=MINGW64
source /etc/profile
cd "$HOME/Adafruit-GFX-Library/fontconvert"
make clean || true
make CC=x86_64-w64-mingw32-gcc CFLAGS="-Wall -I/mingw64/include/freetype2" LIBS="-lfreetype"
install -D fontconvert.exe /c/Users/YULIA/Documents/Arduino/libraries/ST77xx_DMA/tools/fonts/fontconvert.exe
