# ============================================================================
#  stm32f4/CMakeLists.txt - the STM32F4 build: brio's sixth hardware target
#  and its first ARMv7-M one, a SIBLING CMake project (own toolchain file,
#  own presets) exactly like the other five - nothing shared, one configure
#  = one compiler.
#
#  One executable target per app: every stm32f4/src/apps/*.cpp with a
#  main() is discovered at configure time (CONFIGURE_DEPENDS) from its own
#  "// build:" header comment, the same grammar as the other projects. One
#  configure targets exactly ONE part (STM32F4_MCU cache var, one value per
#  configurePreset): the F429ZI (the STM32F429I-DISC1), the F446RE (the
#  Nucleo-F446RE), the F411CE (a WeAct black pill) and the F469NI (the
#  32F469IDISCOVERY) have presets; every other header of the pack is
#  compile-checked by brio check stm32f4. What
#  the part decides - the device define, the crt, the linker script, the
#  board type - is cmake/stm32f4-parts.cmake's table.
#
#  THE CORE AND ITS FPU. -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard:
#  the coprocessor is used, not carried. brio has no floats today, so the
#  hard ABI costs nothing and a later program that wants one pays no ABI
#  break across every image; the crt enables the FPU (CPACR) before a line
#  of C++ runs, because under the hard ABI the compiler may put a float in
#  s0 anywhere it likes.
#
#  Flash/debug: OpenOCD over an ST-LINK (the boards' own V2-1 / V2-B, or a
#  standalone V3): interface/stlink.cfg + target/stm32f4x.cfg; `adapter
#  serial` names the probe, which has a real USB serial on every ST-LINK.
#
#  The NVHEAP build-id defsym is passed as on the other projects: it is a
#  property of util/nv_heap.hpp, and an image that never names an NvHeap
#  never references the symbol.
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-arm.cmake")
endif()

project(brio_stm32f4 CXX C ASM)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

foreach(lang C CXX)
    set(CMAKE_${lang}_FLAGS_RELEASE "" CACHE STRING "" FORCE)
    set(CMAKE_${lang}_FLAGS_DEBUG "" CACHE STRING "" FORCE)
endforeach()

# --- Which part this configure targets: the full part number in lower
#     case (stm32f429zi). The table derives the device-select define, the
#     crt and the board type; the linker script is ld/<part>.ld.
include(cmake/stm32f4-parts.cmake)
set(STM32F4_MCU "stm32f429zi" CACHE STRING "target STM32F4 part, e.g. stm32f429zi")
stm32f4_part_facts("${STM32F4_MCU}" STM32F4_DEFINE STM32F4_STARTUP STM32F4_BOARD)

set(BRIO_COMMON_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -D${STM32F4_DEFINE}
    -Wall -Wextra -Werror
    -ffunction-sections -fdata-sections
)

find_program(ARM_OBJCOPY arm-none-eabi-objcopy PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(ARM_SIZE arm-none-eabi-size PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(ARM_OBJDUMP arm-none-eabi-objdump PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)

set(STM32F4_OPENOCD "/sw/openocd/bin/openocd" CACHE FILEPATH "OpenOCD with ST-LINK + stm32f4x support")
# `adapter serial` disambiguates which ST-LINK when more than one board
# is attached; empty = the only one (bin/brio takes the serial from its
# manifest instead).
set(STM32F4_PROBE_SERIAL "" CACHE STRING "ST-LINK serial to flash/debug through (empty: the only one attached)")
if(STM32F4_PROBE_SERIAL STREQUAL "")
    set(STM32F4_ADAPTER_SERIAL "")
else()
    set(STM32F4_ADAPTER_SERIAL -c "adapter serial ${STM32F4_PROBE_SERIAL}")
endif()

set(STM32F4_NVHEAP_BUILD_ID 0)
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
    if(_ts GREATER STM32F4_NVHEAP_BUILD_ID)
        set(STM32F4_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()

# --- One app -> one executable target, one .bin/.hex, one -upload target.
function(stm32_add_app name)
    add_executable(${name}
        ${STM32F4_STARTUP}
        ${ARGN}
    )

    target_include_directories(${name} PRIVATE
        ${CMAKE_SOURCE_DIR}/../brio
        ${CMAKE_SOURCE_DIR}/../third_party/cmsis-device-f4/Include
        ${CMAKE_SOURCE_DIR}/../third_party/cmsis-core
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

    # -nostartfiles: the crt is ours; newlib-nano provides memcpy etc.;
    # NO syscall stubs on purpose (an accidental _sbrk/_write fails the
    # link loudly). Same rationale as samc21/CMakeLists.txt.
    target_link_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        --specs=nano.specs
        -nostartfiles
        -T${CMAKE_SOURCE_DIR}/ld/${STM32F4_MCU}.ld
        -Wl,--gc-sections
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${STM32F4_NVHEAP_BUILD_ID}
    )
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${CMAKE_SOURCE_DIR}/ld/${STM32F4_MCU}.ld)

    set_target_properties(${name} PROPERTIES SUFFIX ".elf")

    add_custom_command(TARGET ${name} POST_BUILD
        COMMAND ${ARM_OBJCOPY} -O binary
            $<TARGET_FILE:${name}> $<TARGET_FILE_DIR:${name}>/${name}.bin
        COMMAND ${ARM_OBJCOPY} -O ihex
            $<TARGET_FILE:${name}> $<TARGET_FILE_DIR:${name}>/${name}.hex
        COMMAND ${ARM_SIZE} -A -d $<TARGET_FILE:${name}>
        COMMAND ${ARM_OBJDUMP} -h -S $<TARGET_FILE:${name}> > $<TARGET_FILE_DIR:${name}>/${name}.lst
        COMMENT "${name}.bin/.hex + size + ${name}.lst"
        VERBATIM
    )

    # The DHCSR write after the reset is the samc21 lesson (a core left with
    # C_DEBUGEN set HALTS on BKPT instead of faulting): every panic ends in
    # a BKPT, and a flashing session must not leave the board deaf to it.
    add_custom_target(${name}-upload
        COMMAND ${STM32F4_OPENOCD}
            -f interface/stlink.cfg
            ${STM32F4_ADAPTER_SERIAL}
            -f target/stm32f4x.cfg
            -c "program $<TARGET_FILE:${name}> verify"
            -c "reset run"
            -c "mww 0xE000EDF0 0xA05F0000"
            -c "exit"
        DEPENDS ${name}
        VERBATIM
    )
endfunction()

# --- App discovery: every stm32f4/src/apps/<app>.cpp - plus every
#     ../experiments/*/stm32f4/<app>.cpp - becomes a target, gated by its
#     own "// build: boards = ..." header comment (default: f429zi, the
#     bench chip; an app that also runs on the other boards lists them).
file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/stm32f4/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards f429zi)
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

    if(STM32F4_BOARD IN_LIST _boards)
        stm32_add_app(${_app} ${_app_path})
    endif()
endforeach()

# This project's app roster for bin/brio (apps_stm32f4.json): a
# separate file because app names collide across the trees.
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_stm32f4.json"
    "{\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
