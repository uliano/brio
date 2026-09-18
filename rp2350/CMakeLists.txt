# ============================================================================
#  rp2350/CMakeLists.txt - the RP2350 build: a SIBLING CMake project (own
#  toolchain files, own presets) exactly like rp2040/, stm32f4/ and the
#  others - nothing shared, one configure = one compiler.
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
#  One executable target per app: every rp2350/src/apps/*.cpp with a
#  main() is discovered at configure time (CONFIGURE_DEPENDS) from its own
#  "// build:" header comment, the same grammar as the other projects.
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
#  either architecture. The upload target is THE STATE-INDEPENDENT VERB
#  (see rp2350_add_app()): a rescue over the debug port alone, then
#  programming as core 0 of the Arm pair, then a reset - which works
#  whatever the image already in the flash was doing, and whichever
#  architecture it was running.
#
#  The NVHEAP build-id defsym is passed as on the other projects: it is a
#  property of util/nv_heap.hpp, and an image that never names an NvHeap
#  never references the symbol.
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-arm.cmake")
endif()

project(brio_rp2350 CXX C ASM)

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

# THE BOARD TYPES THIS PROJECT SERVES, and why there are two per board:
# the architecture is a preset axis and the manifest names a physical
# board once per architecture (cli/bench/common.py's weact2350b and
# weact2350b-rv are the same piece of hardware), so an app that runs on
# the board is built for both and the default names both.
set(RP2350_BOARDS weact2350b weact2350b-rv)

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
    set(RP2350_STARTUP "src/glue/startup_rp2350_arm.cpp")
    set(RP2350_TOOL_PREFIX "${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-")
    set(RP2350_INCLUDE_ROOTS
        ${CMAKE_SOURCE_DIR}/../brio
        ${CMAKE_SOURCE_DIR}/../third_party/pico-sdk/rp2350/CMSIS
        ${CMAKE_SOURCE_DIR}/../third_party/pico-sdk/rp2350
        ${CMAKE_SOURCE_DIR}/../third_party/cmsis-core)
else()
    list(APPEND BRIO_COMMON_FLAGS -march=${RP2350_RISCV_ARCH} -mabi=ilp32)
    set(RP2350_STARTUP "src/glue/startup_rp2350_riscv.S")
    set(RP2350_TOOL_PREFIX "${RISCV_TOOLCHAIN_DIR}/bin/riscv32-unknown-elf-")
    # THE STUB FIRST: RP2350.h includes core_cm33.h unconditionally, and
    # on this half that include must resolve to nothing
    # (third_party/pico-sdk/rp2350/no_core/core_cm33.h says why). No other
    # build ever sees that directory.
    set(RP2350_INCLUDE_ROOTS
        ${CMAKE_SOURCE_DIR}/../third_party/pico-sdk/rp2350/no_core
        ${CMAKE_SOURCE_DIR}/../brio
        ${CMAKE_SOURCE_DIR}/../third_party/pico-sdk/rp2350/CMSIS
        ${CMAKE_SOURCE_DIR}/../third_party/pico-sdk/rp2350)
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
set(RP2350_PROBE_SERIAL "" CACHE STRING "CMSIS-DAP probe serial to flash/debug through (empty: the only one attached)")
if(RP2350_PROBE_SERIAL STREQUAL "")
    set(RP2350_ADAPTER_SERIAL "")
else()
    set(RP2350_ADAPTER_SERIAL -c "adapter serial ${RP2350_PROBE_SERIAL}")
endif()

set(RP2350_NVHEAP_BUILD_ID 0)
file(GLOB_RECURSE _nvheap_id_sources CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/../brio/*.cpp" "${CMAKE_SOURCE_DIR}/../brio/*.hpp"
    "${CMAKE_SOURCE_DIR}/../brio/*.c"   "${CMAKE_SOURCE_DIR}/../brio/*.h"
    "${CMAKE_SOURCE_DIR}/../brio/*.S"
    "${CMAKE_SOURCE_DIR}/src/*.cpp" "${CMAKE_SOURCE_DIR}/src/*.hpp"
    "${CMAKE_SOURCE_DIR}/src/*.c"   "${CMAKE_SOURCE_DIR}/src/*.h"
    "${CMAKE_SOURCE_DIR}/src/*.S"
)
foreach(_f IN LISTS _nvheap_id_sources)
    file(TIMESTAMP "${_f}" _ts "%s")
    if(_ts GREATER RP2350_NVHEAP_BUILD_ID)
        set(RP2350_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()

# --- One app -> one executable target, one .bin/.hex, one -upload target.
function(rp2350_add_app name)
    add_executable(${name}
        ${RP2350_STARTUP}
        ${ARGN}
    )

    target_include_directories(${name} PRIVATE ${RP2350_INCLUDE_ROOTS})

    target_compile_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        $<$<CONFIG:Debug>:-Og -g3 -ggdb3 -fno-inline>
        $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
        $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++23>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
    )

    # -nostartfiles: the crt is ours; newlib-nano provides memcpy etc.;
    # NO syscall stubs on purpose (an accidental _sbrk/_write fails the
    # link loudly). Same rationale as every other project here.
    target_link_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        --specs=nano.specs
        -nostartfiles
        -T${CMAKE_SOURCE_DIR}/ld/${RP2350_LD}.ld
        -Wl,--gc-sections
        -Wl,--no-warn-rwx-segments   # .data carries .ram_text, the code that runs with the flash disconnected (ld/)
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${RP2350_NVHEAP_BUILD_ID}
    )
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${CMAKE_SOURCE_DIR}/ld/${RP2350_LD}.ld)

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
            -f interface/cmsis-dap.cfg
            -c "cmsis_dap_backend usb_bulk"
            ${RP2350_ADAPTER_SERIAL}
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
            -f interface/cmsis-dap.cfg
            -c "cmsis_dap_backend usb_bulk"
            ${RP2350_ADAPTER_SERIAL}
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

# --- App discovery: every rp2350/src/apps/<app>.cpp - plus every
#     ../experiments/*/rp2350/<app>.cpp - becomes a target, gated by its
#     own "// build: boards = ..." header comment (default: every board
#     type this project serves) against the board types of this configure.
file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/rp2350/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards ${RP2350_BOARDS})
    set(_monitor_speed "")

    file(STRINGS "${_app_path}" _build_lines REGEX "^// build:")
    foreach(_line IN LISTS _build_lines)
        if(_line MATCHES "^// build:[ \t]*([A-Za-z_]+)[ \t]*=[ \t]*(.+)$")
            set(_key "${CMAKE_MATCH_1}")
            string(STRIP "${CMAKE_MATCH_2}" _value)
            if(_key STREQUAL "boards")
                string(REPLACE "," ";" _boards_raw "${_value}")
                set(_boards "")
                foreach(_b IN LISTS _boards_raw)
                    string(STRIP "${_b}" _b)
                    list(APPEND _boards "${_b}")
                endforeach()
            elseif(_key STREQUAL "monitor_speed")
                set(_monitor_speed "${_value}")
            endif()
        endif()
    endforeach()

    if(_monitor_speed STREQUAL "")
        set(_speed_json "null")
    else()
        set(_speed_json "${_monitor_speed}")
    endif()
    set(_boards_json "")
    foreach(_b IN LISTS _boards)
        if(_boards_json STREQUAL "")
            set(_boards_json "\"${_b}\"")
        else()
            set(_boards_json "${_boards_json},\"${_b}\"")
        endif()
    endforeach()
    set(_entry "\"${_app}\":{\"boards\":[${_boards_json}],\"monitor_speed\":${_speed_json}}")
    if(_manifest_entries STREQUAL "")
        set(_manifest_entries "${_entry}")
    else()
        set(_manifest_entries "${_manifest_entries},\n    ${_entry}")
    endif()

    set(_served FALSE)
    foreach(_b IN LISTS _boards)
        if(_b IN_LIST RP2350_BOARDS)
            set(_served TRUE)
        endif()
    endforeach()
    if(_served)
        rp2350_add_app(${_app} ${_app_path})
    endif()
endforeach()

# This project's app roster for bin/brio (apps_rp2350.json): a separate
# file because app names collide across the trees.
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_rp2350.json"
    "{\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
