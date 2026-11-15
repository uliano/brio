# ============================================================================
#  family.cmake - the RP2350 family's build recipe: how ONE image is built
#  for the RP2350 on either processor pair - the architecture and its
#  compiler check, the flash geometry and the package, the flags, the crt
#  and the runtime every image compiles, the link, the .bin/.hex and the
#  listing, the -upload target.
#
#  Included after project() by this directory's CMakeLists.txt (the apps
#  kept in this tree) and by ../../cmake/brio.cmake (an application kept
#  outside it): both are built by rp2350_add_app() below, so the same
#  source gives the same image in both. Every path is taken from this
#  file's own place, never from the including project's. BRIO_BOARD_DIR,
#  when whoever includes this file sets it, is the directory of the board
#  file an image reads as "board.hpp".
#
#  AND THE COMPILER IS THE NEW AXIS. This chip carries two processor
#  architectures on one die (datasheet 3.9: a pair of Arm Cortex-M33 and a
#  pair of RISC-V Hazard3, and the IMAGE_DEF in the flashed image is what
#  decides which pair runs), so THE ARCHITECTURE IS A PRESET AXIS here,
#  the way the part number is on the stm32f4 project: `rp2350-arm-release`
#  and `rp2350-riscv-release` are two configures of THIS project over the
#  SAME sources, each naming its toolchain file and setting RP2350_ARCH,
#  from which the flags, the crt, the include roots and the family check
#  all follow. An app is written once and built twice.
#
#  WHAT A CONFIGURE TARGETS, besides the architecture: a FLASH GEOMETRY
#  and a PACKAGE. The chip executes in place out of an external quad-SPI
#  flash whose size is the board's (RP2350_FLASH_KB, the linker script),
#  and it comes in two packages whose GPIO counts differ - QFN-60 with 30
#  pins and four ADC inputs, QFN-80 with 48 and eight (datasheet 1.2). The
#  package is passed to the stratum as BRIO_RP2350_PACKAGE_PINS so that a
#  pin the board has not got is a compile error and not a silent write;
#  an image built without it compiles for the larger package and refuses
#  the extra pins at RUN time, against SYSINFO.PACKAGE_SEL.
#
#  NO SECOND-STAGE BOOTLOADER, unlike the RP2040: the bootrom sets the
#  XIP interface up itself while scanning the flash (datasheet 5.9.5), so
#  an image begins with its own first byte - a vector table on Arm, a
#  jump on RISC-V - and what marks it an image is the IMAGE_DEF block the
#  crt places in the first 4 kB (src/glue/).
#
#  Flash/debug: Raspberry Pi's OpenOCD fork through a CMSIS-DAP probe -
#  the one OpenOCD that examines this chip's four cores and flashes from
#  either architecture; the probe (the Raspberry Pi Debug Probe unless
#  BRIO_PROBE names another) and its USB serial are cmake/probe.cmake's.
#  The upload target is THE STATE-INDEPENDENT VERB (see
#  rp2350_add_app()): a rescue over the debug port alone, then
#  programming as core 0 of the Arm pair, then a reset - which works
#  whatever the image already in the flash was doing, and whichever
#  architecture it was running.
#
#  The NVHEAP build-id defsym is a property of util/nv_heap.hpp: an image
#  that never names an NvHeap never references the symbol.
# ============================================================================

include_guard(GLOBAL)

get_filename_component(BRIO_FAMILY_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
get_filename_component(BRIO_ROOT "${BRIO_FAMILY_DIR}/.." ABSOLUTE)

# The RISC-V crt is assembly: an application's project() need not name the
# language.
enable_language(ASM)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

foreach(lang C CXX)
    set(CMAKE_${lang}_FLAGS_RELEASE "" CACHE STRING "" FORCE)
    set(CMAKE_${lang}_FLAGS_DEBUG "" CACHE STRING "" FORCE)
endforeach()

# --- The architecture: the axis this project is built around.
set(RP2350_ARCH "arm" CACHE STRING "which processor pair this image runs on: arm (Cortex-M33) or riscv (Hazard3)")
if(NOT RP2350_ARCH STREQUAL "arm" AND NOT RP2350_ARCH STREQUAL "riscv")
    message(FATAL_ERROR "RP2350_ARCH=${RP2350_ARCH}: the RP2350 has two architectures, arm and riscv")
endif()
if(RP2350_ARCH STREQUAL "arm" AND NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
    message(FATAL_ERROR "RP2350_ARCH=arm with the ${CMAKE_SYSTEM_PROCESSOR} toolchain file: a preset states both, and they must agree")
endif()
if(RP2350_ARCH STREQUAL "riscv" AND NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "riscv32")
    message(FATAL_ERROR "RP2350_ARCH=riscv with the ${CMAKE_SYSTEM_PROCESSOR} toolchain file: a preset states both, and they must agree")
endif()
# GCC 16.2.0 at -Os with Zcmp can drop the li a0,0 of a call's first argument
# and end the function in cm.popretz (PR target/126454, fixed on
# releases/gcc-16 after the tarball). The fixed build names it in its version
# line, and a later release carries the fix anyway.
if(RP2350_ARCH STREQUAL "riscv" AND CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 16.2.0)
    execute_process(COMMAND "${CMAKE_CXX_COMPILER}" --version
                    OUTPUT_VARIABLE RP2350_RISCV_VERSION_LINE ERROR_QUIET)
    if(NOT RP2350_RISCV_VERSION_LINE MATCHES "pr126454")
        message(FATAL_ERROR "${CMAKE_CXX_COMPILER} is GCC 16.2.0 without the fix for PR target/126454: it miscompiles calls under Zcmp. Build it with the patch (--with-pkgversion naming pr126454).")
    endif()
endif()

# THE HAZARD3 ISA, as a cache variable because it is a MEASUREMENT still
# owed. The core implements rv32ima with Zicsr/Zifencei, the bit
# manipulation trio Zba/Zbb/Zbs, Zbkb of the crypto set, and the
# compressed subsets Zca/Zcb/Zcmp (datasheet 3.8.1); the toolchain carries
# a multilib both with and without Zcb/Zcmp, so dropping the last two is
# one -D away and what the shorter string costs or saves gets measured on
# real images, the way the CH32V00x's xw extension was.
set(RP2350_RISCV_ARCH "rv32ima_zicsr_zifencei_zba_zbb_zbs_zbkb_zca_zcb_zcmp"
    CACHE STRING "the -march string for the Hazard3 build")

# --- The flash geometry and the package.
set(RP2350_FLASH_KB "16384" CACHE STRING "size of the board's QSPI flash in KB (the linker script)")
set(RP2350_PACKAGE "b" CACHE STRING "the chip's package: a (QFN-60, 30 GPIO) or b (QFN-80, 48 GPIO)")
if(RP2350_FLASH_KB STREQUAL "16384")
    set(RP2350_LD "rp2350_16m")
else()
    message(FATAL_ERROR "RP2350_FLASH_KB=${RP2350_FLASH_KB}: no linker script for that flash size yet (ld/rp2350_16m.ld is the 16 MB one)")
endif()
if(RP2350_PACKAGE STREQUAL "a")
    set(RP2350_PACKAGE_PINS 60)
elseif(RP2350_PACKAGE STREQUAL "b")
    set(RP2350_PACKAGE_PINS 80)
else()
    message(FATAL_ERROR "RP2350_PACKAGE=${RP2350_PACKAGE}: the RP2350 comes in two packages, a (QFN-60) and b (QFN-80)")
endif()

set(BRIO_COMMON_FLAGS
    -Wall -Wextra -Werror
    -ffunction-sections -fdata-sections
    -DBRIO_RP2350_FLASH_KB=${RP2350_FLASH_KB}       # what the flash chapter sizes the chip by
    -DBRIO_RP2350_PACKAGE_PINS=${RP2350_PACKAGE_PINS}   # what brio/rp2350/device.hpp reads the bank's size from
)

if(RP2350_ARCH STREQUAL "arm")
    # The hard-float ABI, as on the stm32f4 project: the crt enables
    # CP10/CP11 before .data is touched.
    list(APPEND BRIO_COMMON_FLAGS -mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16)
    set(RP2350_STARTUP "${BRIO_FAMILY_DIR}/src/glue/startup_rp2350_arm.cpp")
    set(RP2350_TOOL_PREFIX "${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-")
    set(RP2350_INCLUDE_ROOTS
        ${BRIO_ROOT}/brio
        ${BRIO_ROOT}/third_party/pico-sdk/rp2350/CMSIS
        ${BRIO_ROOT}/third_party/pico-sdk/rp2350
        ${BRIO_ROOT}/third_party/cmsis-core)
else()
    list(APPEND BRIO_COMMON_FLAGS -march=${RP2350_RISCV_ARCH} -mabi=ilp32)
    set(RP2350_STARTUP "${BRIO_FAMILY_DIR}/src/glue/startup_rp2350_riscv.S")
    set(RP2350_TOOL_PREFIX "${RISCV_TOOLCHAIN_DIR}/bin/riscv32-unknown-elf-")
    # THE STUB FIRST: RP2350.h includes core_cm33.h unconditionally, and
    # on this half that include must resolve to nothing
    # (third_party/pico-sdk/rp2350/no_core/core_cm33.h says why). No other
    # build ever sees that directory.
    set(RP2350_INCLUDE_ROOTS
        ${BRIO_ROOT}/third_party/pico-sdk/rp2350/no_core
        ${BRIO_ROOT}/brio
        ${BRIO_ROOT}/third_party/pico-sdk/rp2350/CMSIS
        ${BRIO_ROOT}/third_party/pico-sdk/rp2350)
endif()

find_program(RP2350_OBJCOPY ${RP2350_TOOL_PREFIX}objcopy PATHS "/" NO_DEFAULT_PATH)
if(NOT RP2350_OBJCOPY)
    set(RP2350_OBJCOPY "${RP2350_TOOL_PREFIX}objcopy")
endif()
set(RP2350_SIZE "${RP2350_TOOL_PREFIX}size")
set(RP2350_OBJDUMP "${RP2350_TOOL_PREFIX}objdump")
set(RP2350_OBJCOPY "${RP2350_TOOL_PREFIX}objcopy")

# Raspberry Pi's OpenOCD fork: the one that examines this chip (both Arm
# cores AND both RISC-V harts) and flashes from either side.
set(RP2350_OPENOCD "/sw/openocd-rpi-acff23f/bin/openocd" CACHE FILEPATH "Raspberry Pi's OpenOCD fork, the one that knows the RP2350")
# The probe is cmake/probe.cmake's: BRIO_PROBE picks its kind (the
# Raspberry Pi Debug Probe when empty), BRIO_PROBE_SERIAL one probe of
# several, and RP2350_INTERFACE is the interface part of both OpenOCD
# sessions below, the CMSIS-DAP backend included. bin/brio names the probe
# from its manifest instead.
include("${BRIO_ROOT}/cmake/probe.cmake")
brio_probe_select(DEFAULT debugprobe KINDS cmsis-dap)
brio_probe_openocd(RP2350_INTERFACE)

set(RP2350_NVHEAP_BUILD_ID 0)
file(GLOB_RECURSE _nvheap_id_sources CONFIGURE_DEPENDS
    "${BRIO_ROOT}/brio/*.cpp" "${BRIO_ROOT}/brio/*.hpp"
    "${BRIO_ROOT}/brio/*.c"   "${BRIO_ROOT}/brio/*.h"
    "${BRIO_ROOT}/brio/*.S"
    "${BRIO_FAMILY_DIR}/src/glue/*.cpp" "${BRIO_FAMILY_DIR}/src/glue/*.S"
    "${CMAKE_SOURCE_DIR}/*.cpp" "${CMAKE_SOURCE_DIR}/*.hpp"
    "${CMAKE_SOURCE_DIR}/*.c"   "${CMAKE_SOURCE_DIR}/*.h"
    "${CMAKE_SOURCE_DIR}/*.S"
    "${BRIO_ROOT}/boards/*.hpp"
)
foreach(_f IN LISTS _nvheap_id_sources)
    # A build directory kept inside the source directory is not source.
    string(FIND "${_f}" "${CMAKE_BINARY_DIR}/" _in_build)
    if(_in_build EQUAL 0 OR _f MATCHES "/CMakeFiles/")
        continue()
    endif()
    file(TIMESTAMP "${_f}" _ts "%s")
    if(_ts GREATER RP2350_NVHEAP_BUILD_ID)
        set(RP2350_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()

# --- One app -> one executable target, one .bin/.hex, one -upload target.
function(rp2350_add_app name)
    add_executable(${name}
        ${RP2350_STARTUP}
        ${BRIO_ROOT}/brio/rt/rt.cpp
        ${ARGN}
    )

    target_include_directories(${name} PRIVATE ${RP2350_INCLUDE_ROOTS})
    # The board file, when the includer names one: an app writes
    # #include "board.hpp" and reads the facts of that assembly (the LED,
    # the pads its circuits take), and a pair plan keys on board::is().
    if(BRIO_BOARD_DIR AND EXISTS ${BRIO_BOARD_DIR})
        target_include_directories(${name} PRIVATE ${BRIO_BOARD_DIR})
    endif()

    target_compile_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g -DNDEBUG>  # NDEBUG: no debug check (util/stream.hpp)
        $<$<CONFIG:Debug>:-Og -g3 -ggdb3 -fno-inline>
        $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
        $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++23>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
    )

    # The link takes brio's runtime and libgcc and nothing else
    # (design/runtime.md): -nostartfiles because the crt is ours,
    # -nodefaultlibs so that no C library and no syscall layer reach the
    # link on either half - an accidental printf or malloc fails here with
    # its own name - and libgcc named below, after the objects.
    target_link_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        -nostartfiles
        -nodefaultlibs
        -T${BRIO_FAMILY_DIR}/ld/${RP2350_LD}.ld
        -Wl,--gc-sections
        -Wl,--no-warn-rwx-segments   # .data carries code: .fast_text (placed for speed) and .ram_text (the flash engine's window) (ld/)
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${RP2350_NVHEAP_BUILD_ID}
    )
    target_link_libraries(${name} PRIVATE gcc)
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${BRIO_FAMILY_DIR}/ld/${RP2350_LD}.ld)

    set_target_properties(${name} PROPERTIES SUFFIX ".elf")

    add_custom_command(TARGET ${name} POST_BUILD
        COMMAND ${RP2350_OBJCOPY} -O binary
            $<TARGET_FILE:${name}> $<TARGET_FILE_DIR:${name}>/${name}.bin
        COMMAND ${RP2350_OBJCOPY} -O ihex
            $<TARGET_FILE:${name}> $<TARGET_FILE_DIR:${name}>/${name}.hex
        COMMAND ${RP2350_SIZE} -A -d $<TARGET_FILE:${name}>
        COMMAND ${RP2350_OBJDUMP} -h -S $<TARGET_FILE:${name}> > $<TARGET_FILE_DIR:${name}>/${name}.lst
        COMMENT "${name}.bin/.hex + size + ${name}.lst"
        VERBATIM
    )

    # THE STATE-INDEPENDENT FLASH VERB, in two OpenOCD invocations.
    #
    # (1) THE RESCUE, over the debug port and NOTHING ELSE: a DAP with no
    #     target behind it, RESCUE_RESTART set and cleared in the RP-AP's
    #     control register (datasheet 3.5.8). It works "even when system
    #     clocks are stopped and the switched core power domain is powered
    #     down", and leaves the chip halted in the bootrom, in the Arm
    #     architecture, with the clock tree at its reset state. The target
    #     script's own rescue path is NOT usable for this: it examines the
    #     core first, and a core that cannot be examined is exactly the
    #     case a rescue is for.
    # (2) THE PROGRAMMING, as core 0 of the Arm pair - which after the
    #     rescue is what is running, whatever architecture the image in
    #     the flash names - then `reset run`, which hands the chip back to
    #     the bootrom and so to the new image's own architecture.
    #
    # On the Arm side the session ends by taking halting debug back down
    # (DHCSR with its key and every control bit clear): a core with
    # C_DEBUGEN set HALTS on a breakpoint instead of faulting, and
    # break_here() is where every panic ends. There is no such write on
    # the RISC-V side: that register belongs to a core which is not the
    # one running.
    set(_rescue
        ${RP2350_OPENOCD}
            ${RP2350_INTERFACE}
            -c "adapter speed 1000"
            -c "transport select swd"
            -c "swd newdap rp2350 cpu -expected-id 0x00040927"
            -c "dap create rp2350.dap -adiv6 -chain-position rp2350.cpu"
            -c "init"
            -c "rp2350.dap apreg 0x80000 0 0x80000000"
            -c "rp2350.dap apreg 0x80000 0 0"
            -c "shutdown")
    set(_program
        ${RP2350_OPENOCD}
            ${RP2350_INTERFACE}
            -c "set USE_CORE cm0"
            -c "adapter speed 5000"
            -f target/rp2350.cfg
            -c "program $<TARGET_FILE:${name}> verify"
            -c "reset run")
    if(RP2350_ARCH STREQUAL "arm")
        list(APPEND _program -c "mww 0xE000EDF0 0xA05F0000")
    endif()
    list(APPEND _program -c "exit")
    add_custom_target(${name}-upload
        COMMAND ${_rescue}
        COMMAND ${_program}
        DEPENDS ${name}
        VERBATIM
    )
endfunction()

# The name every family's recipe answers to, which is the one an
# application kept outside the tree calls:
#
#     brio_add_app(<name> <source>...)
function(brio_add_app name)
    rp2350_add_app(${name} ${ARGN})
endfunction()
