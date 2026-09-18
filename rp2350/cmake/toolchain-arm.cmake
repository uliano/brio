# ============================================================================
#  toolchain-arm.cmake - the Arm half of the RP2350 build: this chip's
#  Cortex-M33, on the project's own arm-none-eabi-gcc 16.2 at
#  /sw/arm-none-eabi (self-built, never a system-packaged one), pointed at
#  by absolute path.
#
#  The file is the stm32f4 one's twin, because the two families share an
#  ABI: an ARMv8-M/ARMv7-M core with a single-precision FPU built
#  HARD-FLOAT, which means the multilib the compiler picks
#  (thumb/v8-m.main+fp/hard) must exist in the toolchain - it does, and
#  the crt is what enables CP10/CP11 before .data is touched.
#
#  CMAKE_SYSTEM_NAME Generic + STATIC_LIBRARY try_compile: freestanding
#  target, no OS, no working default executable until the linker script
#  and startup are supplied per-app.
#
#  THE ARCHITECTURE IS A PRESET AXIS on this target: the same project
#  configures with this file or with cmake/toolchain-riscv.cmake, and
#  CMakeLists.txt reads RP2350_ARCH (which the preset states beside the
#  toolchain file) to pick the flags, the crt and the include roots.
# ============================================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(ARM_TOOLCHAIN_DIR "/sw/arm-none-eabi" CACHE PATH "Root of the self-built arm-none-eabi toolchain")

set(CMAKE_C_COMPILER   "${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER "${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-gcc")
set(CMAKE_AR           "${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-gcc-ar")
set(CMAKE_RANLIB       "${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-gcc-ranlib")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH "${ARM_TOOLCHAIN_DIR}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
