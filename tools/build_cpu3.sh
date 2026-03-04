#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cmake -S "${ROOT_DIR}/LTD_DISPLAY_CPU3" -B "${ROOT_DIR}/build/LTD_DISPLAY_CPU3" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="${ROOT_DIR}/cmake/toolchain-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Debug

cmake --build "${ROOT_DIR}/build/LTD_DISPLAY_CPU3"
