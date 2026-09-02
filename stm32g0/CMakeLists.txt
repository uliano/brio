# ============================================================================
#  stm32g0/CMakeLists.txt - the STM32G0 build: brio's third hardware target,
#  a SIBLING CMake project (own toolchain file, own presets) exactly like
#  samc/ and avrdx/ - nothing shared, one configure = one compiler, and
#  still no factored "app discovery" module: this is the THIRD instance of
#  the shape, written by hand from the samc one, and the factoring (if it
#  ever happens) happens with all three in hand.
#
#  One executable target per app: every stm32g0/src/apps/*.cpp with a
#  main() is discovered at configure time (CONFIGURE_DEPENDS) from its own
#  "// build:" header comment, the same grammar as the other two projects.
#  One configure targets exactly ONE part (STM32G0_MCU cache var, one value
#  per configurePreset); only the G0B1RE has a preset today - the
#  Nucleo-G0B1RE on the desk - the G071/G031 headers are compile-checked by
#  tools/check_stm32g0.sh instead.
#
#  THE DEVICE SELECT is ST's convention: the umbrella header stm32g0xx.h
#  dispatches on a -DSTM32G0B1xx define, so the define is the part number's
#  first nine characters with the flash-size letter blanked ("stm32g0b1re"
#  -> STM32G0B1xx). The linker script is per PART (the size letter decides
#  flash and RAM), so it keeps the full name: ld/stm32g0b1re.ld. The crt is
#  per DEVICE HEADER (the vector table's names and length follow the
#  header's IRQn list): src/glue/startup_stm32g0b1.cpp.
#
#  Flash/debug: OpenOCD over the Nucleo's on-board ST-LINK/V2.1
#  (interface/stlink.cfg + target/stm32g0x.cfg); `adapter serial` names
#  the probe, which HAS a real USB serial on this family.
#
#  The NVHEAP build-id defsym is passed as on the other two projects: it
#  is a property of util/nv_heap.hpp, and an image that never names an
#  NvHeap never references the symbol.
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-arm.cmake")
endif()

project(brio_stm32g0 CXX C ASM)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

foreach(lang C CXX)
    set(CMAKE_${lang}_FLAGS_RELEASE "" CACHE STRING "" FORCE)
    set(CMAKE_${lang}_FLAGS_DEBUG "" CACHE STRING "" FORCE)
endforeach()

# --- Which part this configure targets: the full part number in lower
#     case (stm32g0b1re). Derived: the device-select define (STM32G0B1xx),
#     the linker script (ld/<part>.ld) and the crt (src/glue/startup_<header>.cpp).
set(STM32G0_MCU "stm32g0b1re" CACHE STRING "target STM32G0 part, e.g. stm32g0b1re")
string(SUBSTRING "${STM32G0_MCU}" 0 9 _header_stem)          # stm32g0b1
string(TOUPPER "${_header_stem}" _header_upper)
set(STM32G0_DEFINE "${_header_upper}xx")                      # STM32G0B1xx
set(STM32G0_STARTUP "src/glue/startup_${_header_stem}.cpp")

set(BRIO_COMMON_FLAGS
    -mcpu=cortex-m0plus
    -mthumb
    -mfloat-abi=soft
    -D${STM32G0_DEFINE}
    -Wall -Wextra -Werror
    -ffunction-sections -fdata-sections
)

find_program(ARM_OBJCOPY arm-none-eabi-objcopy PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(ARM_SIZE arm-none-eabi-size PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(ARM_OBJDUMP arm-none-eabi-objdump PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)

set(STM32G0_OPENOCD "/sw/oss-cad-suite/bin/openocd" CACHE FILEPATH "OpenOCD with ST-LINK + stm32g0x support")
set(STM32G0_PROBE_SERIAL "0670FF534871754867182752" CACHE STRING "ST-LINK serial to flash/debug through")

set(STM32G0_NVHEAP_BUILD_ID 0)
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
    if(_ts GREATER STM32G0_NVHEAP_BUILD_ID)
        set(STM32G0_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()

# --- One app -> one executable target, one .bin/.hex, one -upload target.
function(stm32_add_app name)
    add_executable(${name}
        ${STM32G0_STARTUP}
        ${ARGN}
    )

    target_include_directories(${name} PRIVATE
        ${CMAKE_SOURCE_DIR}/../brio
        ${CMAKE_SOURCE_DIR}/../third_party/cmsis-device-g0/Include
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
    # link loudly). Same rationale as samc/CMakeLists.txt.
    target_link_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        --specs=nano.specs
        -nostartfiles
        -T${CMAKE_SOURCE_DIR}/ld/${STM32G0_MCU}.ld
        -Wl,--gc-sections
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${STM32G0_NVHEAP_BUILD_ID}
    )
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${CMAKE_SOURCE_DIR}/ld/${STM32G0_MCU}.ld)

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

    # The DHCSR write after the reset is the samc lesson (a core left with
    # C_DEBUGEN set HALTS on BKPT instead of faulting): every panic ends in
    # a BKPT, and a flashing session must not leave the board deaf to it.
    add_custom_target(${name}-upload
        COMMAND ${STM32G0_OPENOCD}
            -f interface/stlink.cfg
            -c "adapter serial ${STM32G0_PROBE_SERIAL}"
            -f target/stm32g0x.cfg
            -c "program $<TARGET_FILE:${name}> verify"
            -c "reset run"
            -c "mww 0xE000EDF0 0xA05F0000"
            -c "exit"
        DEPENDS ${name}
        VERBATIM
    )
endfunction()

# --- App discovery: every stm32g0/src/apps/<app>.cpp - plus every
#     ../experiments/*/stm32g0/<app>.cpp - becomes a target, gated by its
#     own "// build: boards = ..." header comment (default: g0b1re, the
#     one board type that exists).
set(STM32G0_BOARD "g0b1re")

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/stm32g0/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards g0b1re)
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

    if(STM32G0_BOARD IN_LIST _boards)
        stm32_add_app(${_app} ${_app_path})
    endif()
endforeach()

# This project's app roster for tools/bench.py (apps_stm32g0.json): a
# separate file because app names collide across the three trees.
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_stm32g0.json"
    "{\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
