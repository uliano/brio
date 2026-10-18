# ============================================================================
#  ch32x035/CMakeLists.txt - the CH32X035 build: WCH's CH32X035 and
#  CH32X033 on the QingKe V4C, a SIBLING CMake project (own toolchain file,
#  own presets) exactly like the others - nothing shared, one configure =
#  one compiler.
#
#  One executable target per app: every ch32x035/src/apps/*.cpp with a
#  main() is discovered at configure time (CONFIGURE_DEPENDS) from its own
#  "// build:" header comment, the same grammar as every other project.
#
#  WHAT THIS PROJECT HAS NOT GOT, and the ST ones have: a device header.
#  WCH ships its register definitions inside the EVT package, whose licence
#  is written for software running on WCH parts, so this target compiles
#  against brio/ch32x035/device.hpp instead - the map read off the
#  reference manual. The part is therefore a definition of THIS build's
#  making (cmake/ch32x035-parts.cmake), asked once in device.hpp, which
#  includes the part's own table.
#
#  THE ARCHITECTURE STRING IS THE PART'S FULL ISA UNDER WCH'S GCC, and the
#  part table states it: the QingKe V4C is RV32IMAC (QingKe V4 manual table
#  1-1, with WCH's "xw" compressed subset), so every part asks for
#  rv32imac_xw with the ilp32 ABI - the CH32V203's pair, which this gcc
#  resolves to the rv32imac_zaamo_zalrsc_xw/ilp32 multilib.
#
#  EVERY PART IS THE SMALLEST CHIP. Every part of the series has 62 KB of
#  code flash and 20 KB of SRAM, so a suite that links on the bench part
#  links on all seven - and one that does not fit splits on all seven: a
#  suite with a "// build: groups" line builds as one image per group on
#  every board type of this project (the GROUP axis below).
#
#  Flash/debug: WCH's OpenOCD fork at /sw/wch-openocd, the only OpenOCD
#  that speaks the WCH-Link's SDI transport, over the two-wire debug port
#  this series has on PC18 (SWDIO) and PC19 (SWCLK).
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-riscv.cmake")
endif()

project(brio_ch32x035 CXX C ASM)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

foreach(lang C CXX)
    set(CMAKE_${lang}_FLAGS_RELEASE "" CACHE STRING "" FORCE)
    set(CMAKE_${lang}_FLAGS_DEBUG "" CACHE STRING "" FORCE)
endforeach()

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ch32x035-parts.cmake)

set(CH32X035_MCU "ch32x035f8" CACHE STRING "target CH32X035/CH32X033 part (cmake/ch32x035-parts.cmake)")
ch32x035_part_facts("${CH32X035_MCU}"
    CH32X035_PART_DEFINE CH32X035_FLASH_KB CH32X035_RAM_KB CH32X035_BOARD
    CH32X035_PART_ARCH CH32X035_PART_ABI)

# The part's own ISA and ABI (the table's two last columns) are DERIVED
# at every configure, never cached: the two cache entries are OVERRIDES,
# empty by default - for an experiment - so a changed CH32X035_MCU on an
# existing cache follows the part, and an override that is set wins and
# says so.
# An entry equal to the derived value is no override, and is cleared
# (which is what a cache written while these entries held the default
# carries).
set(CH32X035_ARCH "" CACHE STRING "-march override; empty (the default) = the part's own ISA (cmake/ch32x035-parts.cmake), derived at every configure")
set(CH32X035_ABI "" CACHE STRING "-mabi override; empty (the default) = the part's own ABI")
if(CH32X035_ARCH STREQUAL "${CH32X035_PART_ARCH}")
    set(CH32X035_ARCH "" CACHE STRING "-march override; empty (the default) = the part's own ISA (cmake/ch32x035-parts.cmake), derived at every configure" FORCE)
endif()
if(CH32X035_ARCH STREQUAL "")
    set(_x035_arch "${CH32X035_PART_ARCH}")
else()
    set(_x035_arch "${CH32X035_ARCH}")
    message(STATUS "CH32X035_ARCH override: -march=${_x035_arch} (${CH32X035_MCU}'s own is ${CH32X035_PART_ARCH})")
endif()
if(CH32X035_ABI STREQUAL "${CH32X035_PART_ABI}")
    set(CH32X035_ABI "" CACHE STRING "-mabi override; empty (the default) = the part's own ABI" FORCE)
endif()
if(CH32X035_ABI STREQUAL "")
    set(_x035_abi "${CH32X035_PART_ABI}")
else()
    set(_x035_abi "${CH32X035_ABI}")
    message(STATUS "CH32X035_ABI override: -mabi=${_x035_abi} (${CH32X035_MCU}'s own is ${CH32X035_PART_ABI})")
endif()

# --- The core's hardware prologue/epilogue (HPE): with it on, the crt
#     sets INTSYSCR.HWSTKEN and every handler is declared with WCH's
#     `interrupt("WCH-Interrupt-fast")` attribute (through pfic.hpp's
#     BRIO_CH32_INTERRUPT), so the caller-saved registers are pushed and
#     popped by the hardware instead of by each handler's prologue. ONE
#     SWITCH FOR THE WHOLE IMAGE, on purpose: a fast handler under an HPE
#     that is off corrupts the interrupted program, so the crt bit and the
#     attribute must never disagree. Interrupt NESTING stays off either
#     way (the kernel's rule, docs/design/kernel.md) - which is where this
#     build differs from WCH's own startup file, that enables nesting.
option(CH32X035_HPE "hardware prologue/epilogue for every interrupt handler" ON)
if(CH32X035_HPE)
    set(CH32X035_HPE_DEFINE -DBRIO_CH32_HPE=1)
else()
    set(CH32X035_HPE_DEFINE "")
endif()

set(BRIO_COMMON_FLAGS
    -march=${_x035_arch}
    -mabi=${_x035_abi}
    -D${CH32X035_PART_DEFINE}
    ${CH32X035_HPE_DEFINE}
    -Wall -Wextra -Werror
    -ffunction-sections -fdata-sections
)

find_program(RISCV_OBJCOPY riscv32-wch-elf-objcopy PATHS "${RISCV_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(RISCV_SIZE riscv32-wch-elf-size PATHS "${RISCV_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(RISCV_OBJDUMP riscv32-wch-elf-objdump PATHS "${RISCV_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)

# The NVHEAP build-id defsym, as on every other project: a property of
# util/nv_heap.hpp (the media record it in every map version), the newest
# source mtime, so an unchanged tree relinks to the same bytes; an image
# that never names an NvHeap never references the symbol.
set(CH32X035_NVHEAP_BUILD_ID 0)
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
    if(_ts GREATER CH32X035_NVHEAP_BUILD_ID)
        set(CH32X035_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()

set(CH32X035_OPENOCD "/sw/wch-openocd/bin/openocd" CACHE FILEPATH "WCH's OpenOCD fork (SDI transport)")
set(CH32X035_OPENOCD_CFG "/sw/wch-openocd/bin/wch-riscv.cfg" CACHE FILEPATH "its RISC-V target config")

# --- One app -> one executable target, one .bin/.hex, one -upload target.
function(ch32x035_add_app name)
    add_executable(${name}
        src/glue/startup_ch32x035.S
        ${CMAKE_SOURCE_DIR}/../brio/rt/rt.cpp
        ${ARGN}
    )

    target_include_directories(${name} PRIVATE
        ${CMAKE_SOURCE_DIR}/../brio
    )

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

    # The link takes brio's runtime and libgcc and nothing else
    # (design/runtime.md): -nostartfiles because the crt is ours,
    # -nodefaultlibs so that no C library and no syscall layer reach the
    # link - an accidental printf or malloc fails here with its own name -
    # and libgcc named below, after the objects.
    target_link_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        -nostartfiles
        -nodefaultlibs
        -T${CMAKE_SOURCE_DIR}/ld/${CH32X035_MCU}.ld
        -Wl,--gc-sections
        -Wl,--no-warn-rwx-segments   # .data carries .ram_text, the code placed in SRAM: code copied there is a writable, executable segment (ld/)
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${CH32X035_NVHEAP_BUILD_ID}
    )
    target_link_libraries(${name} PRIVATE gcc)
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${CMAKE_SOURCE_DIR}/ld/${CH32X035_MCU}.ld)

    set_target_properties(${name} PROPERTIES SUFFIX ".elf")

    add_custom_command(TARGET ${name} POST_BUILD
        COMMAND ${RISCV_OBJCOPY} -O binary
            $<TARGET_FILE:${name}> $<TARGET_FILE_DIR:${name}>/${name}.bin
        COMMAND ${RISCV_OBJCOPY} -O ihex
            $<TARGET_FILE:${name}> $<TARGET_FILE_DIR:${name}>/${name}.hex
        COMMAND ${RISCV_SIZE} -A -d $<TARGET_FILE:${name}>
        COMMAND ${RISCV_OBJDUMP} -h -S $<TARGET_FILE:${name}> > $<TARGET_FILE_DIR:${name}>/${name}.lst
        COMMENT "${name}.bin/.hex + size + ${name}.lst"
        VERBATIM
    )

    # RESET THEN RESUME, NOT `reset run` - the sibling project's rule for
    # this OpenOCD fork, which on the CH32V203 leaves the hart at the reset
    # vector after `reset run` (docs/ch32vx03/README.md). Whether it does
    # the same on this part is not measured; the verb that starts the
    # program on both is the one written here.
    add_custom_target(${name}-upload
        COMMAND ${CH32X035_OPENOCD}
            -f ${CH32X035_OPENOCD_CFG}
            -c "program $<TARGET_FILE:${name}> verify"
            -c "reset halt"
            -c "resume"
            -c "exit"
        DEPENDS ${name}
        VERBATIM
    )
endfunction()

# --- App discovery: every ch32x035/src/apps/<app>.cpp - plus every
#     ../experiments/*/ch32x035/<app>.cpp - becomes a target, gated by its
#     own "// build: boards = ..." header comment (default: x035f8, the
#     board type of WCH's evaluation board in its QFN20 edition).
#
#     The GROUP axis (design/overview.md, "A suite's image fits the
#     family's smallest chip"): no part of this series is smaller than
#     another (the file header), so CH32X035_SPLIT_BOARDS is EVERY board
#     type of the part table - a suite whose header names groups builds as
#     <app>-1, <app>-2, ... on every part, each compiled with
#     BRIO_TEST_LETTERS naming its group, and a suite with no groups line
#     is one image everywhere.
set(CH32X035_SPLIT_BOARDS "")
foreach(_row IN LISTS CH32X035_PARTS)
    string(REPLACE ":" ";" _fields "${_row}")
    list(GET _fields 3 _btype)
    list(APPEND CH32X035_SPLIT_BOARDS "${_btype}")
endforeach()

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/ch32x035/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards x035f8)
    set(_groups "")
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
            elseif(_key STREQUAL "groups")
                string(REPLACE "," ";" _groups_raw "${_value}")
                set(_groups "")
                foreach(_g IN LISTS _groups_raw)
                    string(STRIP "${_g}" _g)
                    list(APPEND _groups "${_g}")
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
    if(_groups STREQUAL "")
        set(_groups_json "null")
    else()
        set(_groups_json "")
        foreach(_g IN LISTS _groups)
            if(_groups_json STREQUAL "")
                set(_groups_json "\"${_g}\"")
            else()
                set(_groups_json "${_groups_json},\"${_g}\"")
            endif()
        endforeach()
        set(_groups_json "[${_groups_json}]")
    endif()
    set(_entry "\"${_app}\":{\"boards\":[${_boards_json}],\"groups\":${_groups_json},\"monitor_speed\":${_speed_json}}")
    if(_manifest_entries STREQUAL "")
        set(_manifest_entries "${_entry}")
    else()
        set(_manifest_entries "${_manifest_entries},\n    ${_entry}")
    endif()

    if(CH32X035_BOARD IN_LIST _boards)
        if(CH32X035_BOARD IN_LIST CH32X035_SPLIT_BOARDS AND NOT _groups STREQUAL "")
            # A line that names any upper-case key partitions those keys as it
            # partitions the rest: its groups carry the '.' marker, so an image
            # whose group names none carries none (util/testbench.hpp,
            # upper_mask_of).
            set(_upper_marker "")
            foreach(_g IN LISTS _groups)
                if(_g MATCHES "[A-Z]")
                    set(_upper_marker ".")
                endif()
            endforeach()

            set(_i 0)
            foreach(_g IN LISTS _groups)
                math(EXPR _i "${_i} + 1")
                ch32x035_add_app(${_app}-${_i} ${_app_path})
                target_compile_definitions(${_app}-${_i} PRIVATE "BRIO_TEST_LETTERS=\"${_g}${_upper_marker}\"")
            endforeach()
        else()
            ch32x035_add_app(${_app} ${_app_path})
        endif()
    endif()
endforeach()

# This project's app roster for bin/brio (apps_ch32x035.json): a separate
# file because app names collide across the trees. It names the boards
# that split a suite into its group images - every board type here (the
# discovery comment above) - so the verbs can resolve "<app>-<n>" and
# refuse the bare name of a suite with groups.
set(_split_json "")
foreach(_b IN LISTS CH32X035_SPLIT_BOARDS)
    if(_split_json STREQUAL "")
        set(_split_json "\"${_b}\"")
    else()
        set(_split_json "${_split_json},\"${_b}\"")
    endif()
endforeach()
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_ch32x035.json"
    "{\n  \"split_boards\": [${_split_json}],\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
