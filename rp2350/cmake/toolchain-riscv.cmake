# ============================================================================
#  toolchain-riscv.cmake - the RISC-V half of the RP2350 build: this chip's
#  two Hazard3 cores, on the project's own upstream riscv32-unknown-elf-gcc
#  16.2 at /sw/riscv32-unknown-elf (self-built, with a multilib for each of
#  the two -march strings this core answers to), pointed at by absolute
#  path. NOT WCH's compiler: that one carries an extension of its own and
#  serves the two QingKe families; this core is upstream RISC-V and an
#  upstream compiler is what it takes.
#
#  CMAKE_SYSTEM_NAME Generic + STATIC_LIBRARY try_compile: freestanding
#  target, no OS, no working default executable until the linker script
#  and startup are supplied per-app - the same shape as every other cross
#  toolchain file in this repository.
#
#  THE ARCHITECTURE IS A PRESET AXIS on this target: the same project
#  configures with this file or with cmake/toolchain-arm.cmake, and
#  CMakeLists.txt reads RP2350_ARCH (which the preset states beside the
#  toolchain file) to pick the flags, the crt and the include roots.
# ============================================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR riscv32)

set(RISCV_TOOLCHAIN_DIR "/sw/riscv32-unknown-elf" CACHE PATH "Root of the self-built riscv32-unknown-elf toolchain")

set(CMAKE_C_COMPILER   "${RISCV_TOOLCHAIN_DIR}/bin/riscv32-unknown-elf-gcc")
set(CMAKE_CXX_COMPILER "${RISCV_TOOLCHAIN_DIR}/bin/riscv32-unknown-elf-g++")
set(CMAKE_ASM_COMPILER "${RISCV_TOOLCHAIN_DIR}/bin/riscv32-unknown-elf-gcc")
set(CMAKE_AR           "${RISCV_TOOLCHAIN_DIR}/bin/riscv32-unknown-elf-gcc-ar")
set(CMAKE_RANLIB       "${RISCV_TOOLCHAIN_DIR}/bin/riscv32-unknown-elf-gcc-ranlib")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH "${RISCV_TOOLCHAIN_DIR}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
