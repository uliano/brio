# ============================================================================
#  toolchain-riscv.cmake - cross toolchain for the CH32V00x family
#  (QingKe V2C, RV32EC).
#
#  WCH's own riscv32-wch-elf gcc 15.2.0 at /sw/wch-riscv (out of the
#  MounRiver toolchain package), pointed at by absolute path like every
#  other toolchain here - never a system-packaged one. It is the ONE
#  compiler in this repository that is not self-built and not at the
#  project's usual version: what it brings that upstream gcc does not is
#  a multilib set for rv32e (upstream gcc has to be built with one) and
#  WCH's proprietary "xw" compressed extension. This family USES xw:
#  CMakeLists.txt's default -march is the part's full ISA under this
#  compiler, by choice (the family's smallest part has 16 KB of flash
#  and every per-cent counts there), so an upstream toolchain is not
#  a drop-in for this file - it would build the code, without xw.
#
#  CMAKE_SYSTEM_NAME Generic + STATIC_LIBRARY try_compile: freestanding
#  target, no OS, and no working default executable until a linker
#  script and a crt are supplied per app - the same bare-metal pattern
#  as the AVR and ARM toolchain files.
# ============================================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR riscv32)

set(RISCV_TOOLCHAIN_DIR "/sw/wch-riscv" CACHE PATH "Root of WCH's riscv32-wch-elf toolchain")

set(CMAKE_C_COMPILER   "${RISCV_TOOLCHAIN_DIR}/bin/riscv32-wch-elf-gcc")
set(CMAKE_CXX_COMPILER "${RISCV_TOOLCHAIN_DIR}/bin/riscv32-wch-elf-g++")
set(CMAKE_ASM_COMPILER "${RISCV_TOOLCHAIN_DIR}/bin/riscv32-wch-elf-gcc")
set(CMAKE_AR           "${RISCV_TOOLCHAIN_DIR}/bin/riscv32-wch-elf-gcc-ar")
set(CMAKE_RANLIB       "${RISCV_TOOLCHAIN_DIR}/bin/riscv32-wch-elf-gcc-ranlib")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH "${RISCV_TOOLCHAIN_DIR}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
