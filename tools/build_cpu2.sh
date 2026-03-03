#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cmake -S "${ROOT_DIR}/LTD_MAIN_CPU2" -B "${ROOT_DIR}/build/LTD_MAIN_CPU2" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="${ROOT_DIR}/cmake/toolchain-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Debug

cmake --build "${ROOT_DIR}/build/LTD_MAIN_CPU2"
