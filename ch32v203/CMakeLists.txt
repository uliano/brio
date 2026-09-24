# ============================================================================
#  ch32v203/CMakeLists.txt - the CH32V203 build: brio's second WCH target
#  and its second RISC-V one, a SIBLING CMake project (own toolchain file,
#  own presets) exactly like the six before it - nothing shared, one
#  configure = one compiler. The CH32V303 is built here too, as the third
#  device class of one reference manual (cmake/ch32v203-parts.cmake).
#
#  One executable target per app: every ch32v203/src/apps/*.cpp with a
#  main() is discovered at configure time (CONFIGURE_DEPENDS) from its own
#  "// build:" header comment, the same grammar as every other project.
#
#  ONE APP IS NOT ALWAYS ONE IMAGE. A suite whose image does not fit the
#  family's smallest parts declares its GROUPS of letters in that same
#  header ("// build: groups = abc,de"), and those parts - the 32 KB tier
#  here - build it as one image per group, <app>-1, <app>-2, ..., each
#  carrying its group's letters and the prose that judges them; every
#  other part builds the same source as ONE image with every letter. What
#  bounds a group is the smallest chip's memory, and the linker script is
#  the guard (design/overview.md, "A suite's image fits the family's
#  smallest chip"; the mechanism is util/testbench.hpp and the discovery
#  loop below).
#
#  WHAT THIS PROJECT HAS NOT GOT, and the ST ones have: a device header.
#  WCH ships its register definitions inside the EVT package, whose licence
#  is written for software running on WCH parts, so this target compiles
#  against brio/ch32v203/device.hpp instead - the map read off the
#  reference manual. The part is therefore a definition of THIS build's
#  making (cmake/ch32v203-parts.cmake), asked once in device.hpp, which
#  includes the part's own table.
#
#  THE ARCHITECTURE STRING IS THE PART'S FULL ISA UNDER WCH'S GCC, and
#  the part table states it. The CH32V203's core is the QingKe V4B: misa
#  reads 0x40901105 - RV32 with I, M, A, C, user mode and one non-standard
#  extension, which is WCH's "xw" set - so its parts ask for rv32imac_xw,
#  which their gcc resolves to the rv32imac_zaamo_zalrsc_xw/ilp32
#  multilib. The CH32V303's is the V4F: misa 0x40901125, the same plus F,
#  single-precision floating point - so its parts ask for rv32imafc_xw
#  with the ilp32f ABI (the rv32imafc_zaamo_zalrsc_xw/ilp32f multilib),
#  and the crt enables the FPU before anything runs. This is the same
#  argument the CH32V00x project makes and the same compiler; what is NOT
#  shared is the ABI - ilp32 or ilp32f here against that family's ilp32e,
#  because these cores have the full thirty-two registers.
#
#  Flash/debug: WCH's OpenOCD fork at /sw/wch-openocd, the only OpenOCD
#  that speaks the WCH-Link's SDI transport, over the two-wire debug port
#  (PA13 = SWDIO, PA14 = SWCLK) this family has - not the CH32V00x's
#  single wire.
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-riscv.cmake")
endif()

project(brio_ch32v203 CXX C ASM)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

foreach(lang C CXX)
    set(CMAKE_${lang}_FLAGS_RELEASE "" CACHE STRING "" FORCE)
    set(CMAKE_${lang}_FLAGS_DEBUG "" CACHE STRING "" FORCE)
endforeach()

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ch32v203-parts.cmake)

set(CH32V203_MCU "ch32v203c8" CACHE STRING "target CH32V203/CH32V303 part (cmake/ch32v203-parts.cmake)")
ch32v203_part_facts("${CH32V203_MCU}"
    CH32V203_PART_DEFINE CH32V203_FLASH_KB CH32V203_RAM_KB CH32V203_BOARD
    CH32V203_PART_ARCH CH32V203_PART_ABI)

# The part's own ISA and ABI are the defaults (the table's two last
# columns); a cache entry still overrides them, for an experiment that
# builds a V4F part without its FPU.
set(CH32V203_ARCH "${CH32V203_PART_ARCH}" CACHE STRING "-march for this part: its full ISA under WCH's gcc (xw is theirs alone)")
set(CH32V203_ABI "${CH32V203_PART_ABI}" CACHE STRING "-mabi for this part")

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
option(CH32V203_HPE "hardware prologue/epilogue for every interrupt handler" ON)
if(CH32V203_HPE)
    set(CH32V203_HPE_DEFINE -DBRIO_CH32_HPE=1)
else()
    set(CH32V203_HPE_DEFINE "")
endif()

set(BRIO_COMMON_FLAGS
    -march=${CH32V203_ARCH}
    -mabi=${CH32V203_ABI}
    -D${CH32V203_PART_DEFINE}
    ${CH32V203_HPE_DEFINE}
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
set(CH32V203_NVHEAP_BUILD_ID 0)
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
    if(_ts GREATER CH32V203_NVHEAP_BUILD_ID)
        set(CH32V203_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()

set(CH32V203_OPENOCD "/sw/wch-openocd/bin/openocd" CACHE FILEPATH "WCH's OpenOCD fork (SDI transport)")
set(CH32V203_OPENOCD_CFG "/sw/wch-openocd/bin/wch-riscv.cfg" CACHE FILEPATH "its RISC-V target config")

# --- One app -> one executable target, one .bin/.hex, one -upload target.
function(ch32v203_add_app name)
    add_executable(${name}
        src/glue/startup_ch32v203.S
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

    # -nostartfiles: the crt is ours (src/glue/startup_ch32v203.S). The C
    # library stays linkable for the few things gcc emits calls to, but NO
    # syscall stubs are provided on purpose: an accidental _write or _sbrk
    # must fail the link loudly rather than pull in a heap.
    target_link_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        -nostartfiles
        -T${CMAKE_SOURCE_DIR}/ld/${CH32V203_MCU}.ld
        -Wl,--gc-sections
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${CH32V203_NVHEAP_BUILD_ID}
    )
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${CMAKE_SOURCE_DIR}/ld/${CH32V203_MCU}.ld)

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

    # RESET THEN RESUME, NOT `reset run`. In WCH's OpenOCD fork that
    # verb leaves the hart sitting at the reset vector with every
    # peripheral at its reset value - measured on this bench: the
    # program counter still zero and the clock tree untouched four
    # hundred milliseconds later - so an image flashed that way is in
    # the chip and NOT running, and the board looks dead until something
    # else resumes it. `reset halt` followed by `resume` starts it.
    add_custom_target(${name}-upload
        COMMAND ${CH32V203_OPENOCD}
            -f ${CH32V203_OPENOCD_CFG}
            -c "program $<TARGET_FILE:${name}> verify"
            -c "reset halt"
            -c "resume"
            -c "exit"
        DEPENDS ${name}
        VERBATIM
    )
endfunction()

# --- App discovery: every ch32v203/src/apps/<app>.cpp - plus every
#     ../experiments/*/ch32v203/<app>.cpp - becomes a target, gated by its
#     own "// build: boards = ..." header comment (default: the board type
#     of the part this configure targets).
#
#     The GROUP axis (design/overview.md, "A suite's image fits the
#     family's smallest chip"): a suite whose whole image does not fit the
#     family's 32 KB tier declares its groups of letters in the same
#     header ("// build: groups = abc,de"). On a board of
#     CH32V203_SPLIT_BOARDS the app then builds as one image PER GROUP -
#     <app>-1, <app>-2, ... - each compiled with BRIO_TEST_LETTERS set to
#     its group's keys, which util/testbench.hpp reads to register those
#     letters alone; on every other board the app is ONE image carrying
#     every letter, and a groups line changes nothing there. The roster
#     names the split boards and each app's groups, so bin/brio can
#     resolve "<app>-<n>" and refuse the bare name where it is not an
#     image.
#
#     WHICH BOARDS SPLIT IS READ OFF THE PART TABLE, not listed a second
#     time: every part of 32 KB or less - the C6, the F6, the G6 and the
#     K6. A part added to cmake/ch32v203-parts.cmake is placed by the
#     same memories that give it its linker script, and a part smaller
#     still would join that tier rather than replace it.
set(CH32V203_SPLIT_FLASH_KB 32)
set(CH32V203_SPLIT_BOARDS "")
foreach(_row IN LISTS CH32V203_PARTS)
    string(REGEX REPLACE "=.*$" "" _part "${_row}")
    ch32v203_part_facts("${_part}" _split_define _split_flash _split_ram _split_board
        _split_arch _split_abi)
    if(_split_flash LESS_EQUAL CH32V203_SPLIT_FLASH_KB)
        list(APPEND CH32V203_SPLIT_BOARDS "${_split_board}")
    endif()
endforeach()

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/ch32v203/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards v203c8)
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

    if(CH32V203_BOARD IN_LIST _boards)
        if(CH32V203_BOARD IN_LIST CH32V203_SPLIT_BOARDS AND NOT _groups STREQUAL "")
            set(_i 0)
            foreach(_g IN LISTS _groups)
                math(EXPR _i "${_i} + 1")
                ch32v203_add_app(${_app}-${_i} ${_app_path})
                target_compile_definitions(${_app}-${_i} PRIVATE "BRIO_TEST_LETTERS=\"${_g}\"")
            endforeach()
        else()
            ch32v203_add_app(${_app} ${_app_path})
        endif()
    endif()
endforeach()

# This project's app roster for bin/brio (apps_ch32v203.json): a separate
# file because app names collide across the trees. It names the boards
# that split a suite into its group images, so the verbs can resolve
# "<app>-<n>" there and refuse the bare name.
set(_split_json "")
foreach(_b IN LISTS CH32V203_SPLIT_BOARDS)
    if(_split_json STREQUAL "")
        set(_split_json "\"${_b}\"")
    else()
        set(_split_json "${_split_json},\"${_b}\"")
    endif()
endforeach()
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_ch32v203.json"
    "{\n  \"split_boards\": [${_split_json}],\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
