# ============================================================================
#  rp2040/CMakeLists.txt - the RP2040 build: brio's fifth hardware target, a
#  SIBLING CMake project (own toolchain file, own presets) exactly like
#  samc21/, stm32g0/ and the others - nothing shared, one configure = one
#  compiler, and still no factored "app discovery" module: the FIFTH
#  instance of the shape, written by hand from the stm32g0 one.
#
#  One executable target per app: every rp2040/src/apps/*.cpp with a
#  main() is discovered at configure time (CONFIGURE_DEPENDS) from its own
#  "// build:" header comment, the same grammar as the other projects.
#
#  ONE CHIP, ONE HEADER, and what a configure targets is not a part but a
#  FLASH: the RP2040 executes in place out of an external quad-SPI flash
#  chip whose size (RP2040_FLASH_KB, the linker script) and whose read
#  protocol (RP2040_BOOT2, the second-stage bootloader in src/glue/) are
#  the board's. A preset therefore serves EVERY board type with that
#  flash geometry and that stage: RP2040_BOARDS lists them, and an app is
#  built here when its "// build: boards =" line names any of them
#  (default: pico, the reference board; a WeAct board is weact2040, a
#  Pico W is picow - the same flash, but four of its pins are the radio
#  chip's and its LED is on the radio, so it is named apart). All three
#  share the rp2040-* presets.
#
#  THE IMAGE BEGINS WITH THE STAGE: src/glue/boot2_<stage>.S is the first
#  256 bytes of every image (ld/ asserts it), the SDK's boot_stage2 for
#  that flash chip assembled, padded and CRC'd once and checked in as
#  bytes (its provenance in its own header comment) - no generation step.
#  The crt is src/glue/startup_rp2040.cpp: the vector table the stage
#  vectors into at flash offset 0x100, with the SDK's own handler names.
#
#  Flash/debug: OpenOCD over a CMSIS-DAP probe - the Raspberry Pi Debug
#  Probe, CMSIS-DAP v2 on the USB bulk backend - through target/rp2040.cfg
#  (the multidrop SWD that reaches both cores, the rp2040 flash driver);
#  `adapter serial` names the probe, which HAS a real USB serial.
#
#  The NVHEAP build-id defsym is passed as on the other projects: it is a
#  property of util/nv_heap.hpp, and an image that never names an NvHeap
#  never references the symbol.
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-arm.cmake")
endif()

project(brio_rp2040 CXX C ASM)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

foreach(lang C CXX)
    set(CMAKE_${lang}_FLAGS_RELEASE "" CACHE STRING "" FORCE)
    set(CMAKE_${lang}_FLAGS_DEBUG "" CACHE STRING "" FORCE)
endforeach()

# --- What this configure targets: a flash geometry and its boot stage.
set(RP2040_FLASH_KB "2048" CACHE STRING "size of the board's QSPI flash in KB (the linker script)")
set(RP2040_BOOT2 "w25q080" CACHE STRING "second-stage bootloader: w25q080 (Winbond-class quad I/O) or generic_03h (any flash, slow)")
if(RP2040_FLASH_KB STREQUAL "2048")
    set(RP2040_LD "rp2040_2m")
    set(RP2040_BOARDS pico weact2040 picow)
else()
    message(FATAL_ERROR "RP2040_FLASH_KB=${RP2040_FLASH_KB}: no linker script for that flash size yet (ld/rp2040_2m.ld is the 2 MB one)")
endif()
set(RP2040_STARTUP "src/glue/startup_rp2040.cpp")
set(RP2040_STAGE "src/glue/boot2_${RP2040_BOOT2}.S")
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/${RP2040_STAGE}")
    message(FATAL_ERROR "RP2040_BOOT2=${RP2040_BOOT2}: ${RP2040_STAGE} does not exist")
endif()

set(BRIO_COMMON_FLAGS
    -mcpu=cortex-m0plus
    -mthumb
    -mfloat-abi=soft
    -Wall -Wextra -Werror
    -ffunction-sections -fdata-sections
)

find_program(ARM_OBJCOPY arm-none-eabi-objcopy PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(ARM_SIZE arm-none-eabi-size PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(ARM_OBJDUMP arm-none-eabi-objdump PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)

set(RP2040_OPENOCD "/sw/openocd/bin/openocd" CACHE FILEPATH "OpenOCD with CMSIS-DAP (USB bulk) + rp2040 support")
# `adapter serial` disambiguates which probe when more than one CMSIS-DAP
# device is attached; empty = the only one (bin/brio takes the serial from
# its manifest instead).
set(RP2040_PROBE_SERIAL "" CACHE STRING "CMSIS-DAP probe serial to flash/debug through (empty: the only one attached)")
if(RP2040_PROBE_SERIAL STREQUAL "")
    set(RP2040_ADAPTER_SERIAL "")
else()
    set(RP2040_ADAPTER_SERIAL -c "adapter serial ${RP2040_PROBE_SERIAL}")
endif()

set(RP2040_NVHEAP_BUILD_ID 0)
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
    if(_ts GREATER RP2040_NVHEAP_BUILD_ID)
        set(RP2040_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()

# --- One app -> one executable target, one .bin/.hex, one -upload target.
function(rp2040_add_app name)
    add_executable(${name}
        ${RP2040_STAGE}
        ${RP2040_STARTUP}
        ${ARGN}
    )

    target_include_directories(${name} PRIVATE
        ${CMAKE_SOURCE_DIR}/../brio
        ${CMAKE_SOURCE_DIR}/../third_party/pico-sdk/CMSIS
        ${CMAKE_SOURCE_DIR}/../third_party/pico-sdk
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
        -T${CMAKE_SOURCE_DIR}/ld/${RP2040_LD}.ld
        -Wl,--gc-sections
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${RP2040_NVHEAP_BUILD_ID}
    )
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${CMAKE_SOURCE_DIR}/ld/${RP2040_LD}.ld)

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
    # USE_CORE 0 keeps OpenOCD off core 1 altogether (the target script's
    # default takes both cores as an SMP pair, halts both to program and
    # leaves core 1 debug-enabled and reset a second time - cli/bench/
    # flash.py says what that did to a two-kernel program). The probe is
    # a CMSIS-DAP v2 device: the USB bulk backend, not the Atmel-ICE's
    # HID one.
    add_custom_target(${name}-upload
        COMMAND ${RP2040_OPENOCD}
            -f interface/cmsis-dap.cfg
            -c "cmsis_dap_backend usb_bulk"
            ${RP2040_ADAPTER_SERIAL}
            -c "set USE_CORE 0"
            -c "adapter speed 5000"
            -f target/rp2040.cfg
            -c "program $<TARGET_FILE:${name}> verify"
            -c "reset run"
            -c "mww 0xE000EDF0 0xA05F0000"
            -c "exit"
        DEPENDS ${name}
        VERBATIM
    )
endfunction()

# --- App discovery: every rp2040/src/apps/<app>.cpp - plus every
#     ../experiments/*/rp2040/<app>.cpp - becomes a target, gated by its
#     own "// build: boards = ..." header comment (default: pico, the
#     reference board) against the board types this configure serves.
file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/rp2040/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards pico)
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
        if(_b IN_LIST RP2040_BOARDS)
            set(_served TRUE)
        endif()
    endforeach()
    if(_served)
        rp2040_add_app(${_app} ${_app_path})
    endif()
endforeach()

# This project's app roster for bin/brio (apps_rp2040.json): a separate
# file because app names collide across the trees.
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_rp2040.json"
    "{\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
