# ============================================================================
#  ch32v203/CMakeLists.txt - the CH32V203 build: brio's second WCH target
#  and its second RISC-V one, a SIBLING CMake project (own toolchain file,
#  own presets) exactly like the six before it - nothing shared, one
#  configure = one compiler.
#
#  One executable target per app: every ch32v203/src/apps/*.cpp with a
#  main() is discovered at configure time (CONFIGURE_DEPENDS) from its own
#  "// build:" header comment, the same grammar as every other project.
#
#  WHAT THIS PROJECT HAS NOT GOT, and the ST ones have: a device header.
#  WCH ships its register definitions inside the EVT package, whose licence
#  is written for software running on WCH parts, so this target compiles
#  against brio/ch32v203/device.hpp instead - the map read off the
#  reference manual. The part is therefore a definition of THIS build's
#  making (cmake/ch32v203-parts.cmake), asked once in device.hpp, which
#  includes the part's own table.
#
#  THE ARCHITECTURE STRING IS THE PART'S FULL ISA UNDER WCH'S GCC. The
#  core is the QingKe V4B: misa reads 0x40901105 - RV32 with I, M, A, C,
#  user mode and one non-standard extension, which is WCH's "xw" set. The
#  default below asks for all of it, rv32imac_xw, which their gcc resolves
#  to the rv32imac_zaamo_zalrsc_xw/ilp32 multilib. This is the same
#  argument the CH32V00x project makes and the same compiler; what is NOT
#  shared is the ABI - ilp32 here against that family's ilp32e, because
#  this core has the full thirty-two registers.
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

set(CH32V203_MCU "ch32v203c8" CACHE STRING "target CH32V203 part (cmake/ch32v203-parts.cmake)")
ch32v203_part_facts("${CH32V203_MCU}"
    CH32V203_PART_DEFINE CH32V203_FLASH_KB CH32V203_RAM_KB CH32V203_BOARD)

set(CH32V203_ARCH "rv32imac_xw" CACHE STRING "-march for this family: its full ISA under WCH's gcc (xw is theirs alone)")
set(CH32V203_ABI "ilp32" CACHE STRING "-mabi for this family")

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
file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/ch32v203/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards v203c8)
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
    set(_entry "\"${_app}\":{\"boards\":[${_boards_json}],\"groups\":null,\"monitor_speed\":${_speed_json}}")
    if(_manifest_entries STREQUAL "")
        set(_manifest_entries "${_entry}")
    else()
        set(_manifest_entries "${_manifest_entries},\n    ${_entry}")
    endif()

    if(CH32V203_BOARD IN_LIST _boards)
        ch32v203_add_app(${_app} ${_app_path})
    endif()
endforeach()

# This project's app roster for bin/brio (apps_ch32v203.json): a separate
# file because app names collide across the trees.
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_ch32v203.json"
    "{\n  \"split_boards\": [],\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
