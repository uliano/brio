# ============================================================================
#  toolchain-riscv.cmake - cross toolchain for the CH32V203 family
#  (QingKe V4B, RV32IMAC).
#
#  WCH's own riscv32-wch-elf gcc 15.2.0 at /sw/wch-riscv, the same
#  compiler the ch32v00x project uses and for the same reason: it is the
#  only one that emits WCH's proprietary "xw" compressed extension, which
#  this core carries too (misa reads 0x40901105 - I, M, A, C, U and one
#  non-standard extension). Pointed at by absolute path like every other
#  toolchain here.
#
#  THE ABI IS ilp32 AND THE ISA HAS THE FULL REGISTER SET, which is what
#  separates this family from the CH32V00x: that core is RV32EC (sixteen
#  registers, ilp32e), this one is RV32IMAC with the standard thirty-two
#  and an atomic extension. One compiler, two worlds; the project's own
#  -march/-mabi (CMakeLists.txt) state which.
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
