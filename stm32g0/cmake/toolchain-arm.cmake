# ============================================================================
#  toolchain-arm.cmake - cross toolchain for the STM32G0 family (Cortex-M0+) - the samc
#  file verbatim: same compiler, same core; the armv6m factoring pass may
#  make the two one file.
#
#  The project's own arm-none-eabi-gcc 16.2 at /sw/arm-none-eabi (self-built,
#  same vintage as the AVR and host compilers - never a system-packaged one),
#  pointed at directly by absolute path: no package manifest or download
#  step needed.
#
#  CMAKE_SYSTEM_NAME Generic + STATIC_LIBRARY try_compile: freestanding
#  target, no OS, no working default executable until the linker script and
#  startup are supplied per-app (the standard bare-metal pattern, same as
#  the AVR toolchain file).
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
