# ============================================================================
#  toolchain-riscv.cmake - cross toolchain for the CH32X035 series
#  (QingKe V4C, RV32IMAC).
#
#  WCH's own riscv32-wch-elf gcc 15.2.0 at /sw/wch-riscv, the compiler the
#  ch32v00x and ch32vx03 projects use and for the same reason: it is the
#  only one that emits WCH's proprietary "xw" compressed extension, which
#  the V4C carries (QingKe V4 manual table 1-1: the V4B, V4C and V4F have
#  the XW subset, the V4A has not). Pointed at by absolute path like every
#  other toolchain here.
#
#  THE ISA IS THE CH32V203'S: RV32IMAC with the full register file, the
#  atomic extension and xw, which this gcc resolves to the
#  rv32imac_zaamo_zalrsc_xw/ilp32 multilib. The part table
#  (cmake/ch32x035-parts.cmake) states the pair per part all the same, so
#  that a part of another core would be a row and not an edit here.
#
#  CMAKE_SYSTEM_NAME Generic + STATIC_LIBRARY try_compile: freestanding
#  target, no OS, and no working default executable until a linker script
#  and a crt are supplied per app - the same bare-metal pattern as every
#  other toolchain file here.
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
