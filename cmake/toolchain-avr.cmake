# ============================================================================
#  toolchain-avr.cmake - cross toolchain for the AVR DA/DB family
#  (mcu-agnostic: CMakeLists.txt's AVR_MCU cache var selects the package
#  per configurePreset - this file only points at the compiler).
#
#  The project's own avr-gcc 16.2 at /sw/avr (never a system-packaged
#  one - see CLAUDE.md "Build, test, debug"), pointed at directly by
#  absolute path: no package manifest or download step needed.
#
#  CMAKE_SYSTEM_NAME Generic + STATIC_LIBRARY try_compile: this is a
#  freestanding target with no OS and no working default main()/exit() until
#  the linker script and crt object are supplied per-executable, so CMake's
#  compiler-identification link test is told to build a static lib instead
#  of an executable (the standard bare-metal pattern).
# ============================================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)

set(AVR_TOOLCHAIN_DIR "/sw/avr" CACHE PATH "Root of the self-built avr-gcc toolchain")

set(CMAKE_C_COMPILER   "${AVR_TOOLCHAIN_DIR}/bin/avr-gcc")
set(CMAKE_CXX_COMPILER "${AVR_TOOLCHAIN_DIR}/bin/avr-g++")
set(CMAKE_ASM_COMPILER "${AVR_TOOLCHAIN_DIR}/bin/avr-gcc")
set(CMAKE_AR           "${AVR_TOOLCHAIN_DIR}/bin/avr-gcc-ar")
set(CMAKE_RANLIB       "${AVR_TOOLCHAIN_DIR}/bin/avr-gcc-ranlib")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH "${AVR_TOOLCHAIN_DIR}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
