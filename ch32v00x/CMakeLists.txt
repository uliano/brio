# ============================================================================
#  ch32v00x/CMakeLists.txt - the CH32V00x build: brio's fourth hardware
#  target and its first RISC-V one, a SIBLING CMake project (own
#  toolchain file, own presets) exactly like avrdx/, samc21/ and
#  stm32g0/ - nothing shared, one configure = one compiler.
#
#  One executable target per app: every ch32v00x/src/apps/*.cpp with a
#  main() is discovered at configure time (CONFIGURE_DEPENDS) from its
#  own "// build:" header comment, the same grammar as the other three
#  projects.
#
#  WHAT THIS PROJECT HAS NOT GOT, and the other three have: a device
#  header. WCH ships its register definitions inside the EVT package,
#  whose licence is written for software running on WCH parts, so this
#  target compiles against brio/ch32v00x/device.hpp instead - the map
#  read off the reference manual. The part is therefore a definition of
#  THIS build's making (CH32V006 or CH32V003, from the part number),
#  asked once in device.hpp, which includes the part's own table.
#
#  THE ARCHITECTURE STRING IS THE PART'S FULL ISA UNDER WCH'S GCC. The
#  core is RV32EC with the multiply-only "m" (misa reads 0x40801014: E,
#  C, M and one non-standard extension); the non-standard extension is
#  WCH's "xw" compressed set, which only their compiler emits, and the
#  default below asks for all of it: rv32ec_zmmul_xw. xw is worth one
#  to two per cent of every image (measured over the suites: 72 to 516
#  bytes each), and the family's smallest part - the CH32V003, 16 KB of
#  flash and 2 KB of RAM - turns those bytes into the choice: this
#  family is built with the vendor compiler and its extension, and an upstream
#  gcc, which has no xw, is not a drop-in here. The part number selects
#  the string: rv32ec_xw on the CH32V003, whose core has no multiplier.
#
#  Flash/debug: WCH's OpenOCD fork at /sw/wch-openocd, the only OpenOCD
#  that speaks the WCH-Link's SDI transport. The probe's own serial
#  pins are the console on this bench, so one cable carries both.
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-riscv.cmake")
endif()

project(brio_ch32v00x CXX C ASM)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

foreach(lang C CXX)
    set(CMAKE_${lang}_FLAGS_RELEASE "" CACHE STRING "" FORCE)
    set(CMAKE_${lang}_FLAGS_DEBUG "" CACHE STRING "" FORCE)
endforeach()

# --- Which part this configure targets: the part number in lower case,
#     without the package and temperature suffix (ch32v006k8, ch32v003f4).
#     It picks the linker script ld/<part>.ld, the part DEFINITION the
#     stratum's device.hpp asks (CH32V006 / CH32V003: the family has no
#     vendor header, so the build states the part) and, unless overridden,
#     the ISA: the CH32V006's QingKe V2C has the multiplier, the
#     CH32V003's V2A has not.
set(CH32V00X_MCU "ch32v006k8" CACHE STRING "target CH32V00x part: ch32v006k8 or ch32v003f4")
string(SUBSTRING "${CH32V00X_MCU}" 0 8 _ch32_part)
string(TOUPPER "${_ch32_part}" CH32V00X_PART_DEFINE)                  # CH32V006 / CH32V003
if(NOT CH32V00X_PART_DEFINE MATCHES "^CH32V00[36]$")
    message(FATAL_ERROR "CH32V00X_MCU '${CH32V00X_MCU}': the part must be a CH32V006 or a CH32V003")
endif()
if(CH32V00X_PART_DEFINE STREQUAL "CH32V003")
    set(_ch32_arch_default "rv32ec_xw")
else()
    set(_ch32_arch_default "rv32ec_zmmul_xw")
endif()
set(CH32V00X_ARCH "${_ch32_arch_default}" CACHE STRING "-march for this part: its full ISA under WCH's gcc (xw is theirs alone); empty = the part's default")
if("${CH32V00X_ARCH}" STREQUAL "")
    set(CH32V00X_ARCH "${_ch32_arch_default}")
endif()
set(CH32V00X_ABI "ilp32e" CACHE STRING "-mabi for this family")

# --- The core's hardware prologue/epilogue (HPE): with it on, the crt
#     sets INTSYSCR.HWSTKEN and every handler is declared with WCH's
#     `interrupt("WCH-Interrupt-fast")` attribute (through pfic.hpp's
#     BRIO_CH32_INTERRUPT), so the ten caller-saved registers are pushed
#     and popped by the hardware instead of by each handler's prologue.
#     ONE SWITCH FOR THE WHOLE IMAGE, on purpose: a fast handler under
#     an HPE that is off corrupts the interrupted program, so the crt
#     bit and the attribute must never disagree. Interrupt NESTING stays
#     off either way (the kernel's rule, docs/design/kernel.md). Needs
#     WCH's gcc, or an upstream gcc with the fast-interrupt patch.
option(CH32V00X_HPE "hardware prologue/epilogue for every interrupt handler" ON)
if(CH32V00X_HPE)
    set(CH32V00X_HPE_DEFINE -DBRIO_CH32_HPE=1)
else()
    set(CH32V00X_HPE_DEFINE "")
endif()

set(BRIO_COMMON_FLAGS
    -march=${CH32V00X_ARCH}
    -mabi=${CH32V00X_ABI}
    -D${CH32V00X_PART_DEFINE}
    ${CH32V00X_HPE_DEFINE}
    -Wall -Wextra -Werror
    -ffunction-sections -fdata-sections
)

find_program(RISCV_OBJCOPY riscv32-wch-elf-objcopy PATHS "${RISCV_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(RISCV_SIZE riscv32-wch-elf-size PATHS "${RISCV_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(RISCV_OBJDUMP riscv32-wch-elf-objdump PATHS "${RISCV_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)

# The NVHEAP build-id defsym, as on the other three projects: a property
# of util/nv_heap.hpp (the media record it in every map version), the
# newest source mtime, so an unchanged tree relinks to the same bytes;
# an image that never names an NvHeap never references the symbol.
set(CH32V00X_NVHEAP_BUILD_ID 0)
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
    if(_ts GREATER CH32V00X_NVHEAP_BUILD_ID)
        set(CH32V00X_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()

set(CH32V00X_OPENOCD "/sw/wch-openocd/bin/openocd" CACHE FILEPATH "WCH's OpenOCD fork (SDI transport)")
set(CH32V00X_OPENOCD_CFG "/sw/wch-openocd/bin/wch-riscv.cfg" CACHE FILEPATH "its RISC-V target config")

# --- One app -> one executable target, one .bin/.hex, one -upload target.
function(ch32_add_app name)
    add_executable(${name}
        src/glue/startup_ch32v00x.S
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

    # -nostartfiles: the crt is ours (src/glue/startup_ch32v00x.S). The
    # C library stays linkable for the few things gcc emits calls to
    # (memcpy for a structure copy, the division helpers this core has
    # no instruction for), but NO syscall stubs are provided on purpose:
    # an accidental _write or _sbrk must fail the link loudly rather
    # than pull in a heap.
    target_link_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        -nostartfiles
        -T${CMAKE_SOURCE_DIR}/ld/${CH32V00X_MCU}.ld
        -Wl,--gc-sections
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${CH32V00X_NVHEAP_BUILD_ID}
    )
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${CMAKE_SOURCE_DIR}/ld/${CH32V00X_MCU}.ld)

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

    add_custom_target(${name}-upload
        COMMAND ${CH32V00X_OPENOCD}
            -f ${CH32V00X_OPENOCD_CFG}
            -c "program $<TARGET_FILE:${name}> verify"
            -c "reset run"
            -c "exit"
        DEPENDS ${name}
        VERBATIM
    )
endfunction()

# --- App discovery: every ch32v00x/src/apps/<app>.cpp - plus every
#     ../experiments/*/ch32v00x/<app>.cpp - becomes a target, gated by
#     its own "// build: boards = ..." header comment (default: v006k8,
#     the bench board).
#
#     The GROUP axis (design/overview.md, "A suite's image fits the
#     family's smallest chip"): a suite whose whole image does not fit
#     the CH32V003's 15 KB declares its groups of letters in the same
#     header ("// build: groups = abg,cdf,e"). On a board of
#     CH32V00X_SPLIT_BOARDS the app then builds as one image PER GROUP -
#     <app>-1, <app>-2, ... - each compiled with BRIO_TEST_LETTERS set to
#     its group's keys, which util/testbench.hpp reads to register those
#     letters alone; on every other board the app is ONE image carrying
#     every letter, and a groups line changes nothing there. The roster
#     names the split boards and each app's groups, so bin/brio can
#     resolve "<app>-<n>" and refuse the bare name where it is not an
#     image.
string(SUBSTRING "${CH32V00X_MCU}" 4 6 CH32V00X_BOARD)          # v006k8
set(CH32V00X_SPLIT_BOARDS v003f4)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/ch32v00x/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards v006k8)
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

    if(CH32V00X_BOARD IN_LIST _boards)
        if(CH32V00X_BOARD IN_LIST CH32V00X_SPLIT_BOARDS AND NOT _groups STREQUAL "")
            set(_i 0)
            foreach(_g IN LISTS _groups)
                math(EXPR _i "${_i} + 1")
                ch32_add_app(${_app}-${_i} ${_app_path})
                target_compile_definitions(${_app}-${_i} PRIVATE "BRIO_TEST_LETTERS=\"${_g}\"")
            endforeach()
        else()
            ch32_add_app(${_app} ${_app_path})
        endif()
    endif()
endforeach()

# This project's app roster for bin/brio (apps_ch32v00x.json): a
# separate file because app names collide across the trees. It names
# the boards that split a suite into its group images, so the verbs can
# resolve "<app>-<n>" there and refuse the bare name.
set(_split_json "")
foreach(_b IN LISTS CH32V00X_SPLIT_BOARDS)
    if(_split_json STREQUAL "")
        set(_split_json "\"${_b}\"")
    else()
        set(_split_json "${_split_json},\"${_b}\"")
    endif()
endforeach()
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_ch32v00x.json"
    "{\n  \"split_boards\": [${_split_json}],\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
