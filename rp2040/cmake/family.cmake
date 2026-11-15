# ============================================================================
#  family.cmake - the RP2040 family's build recipe: how ONE image is built
#  for the RP2040 on a board's flash - the flash geometry and its boot
#  stage, the flags, the crt and the runtime every image compiles, the
#  link, the .bin/.hex and the listing, the -upload target.
#
#  Included after project() by this directory's CMakeLists.txt (the apps
#  kept in this tree) and by ../../cmake/brio.cmake (an application kept
#  outside it): both are built by rp2040_add_app() below, so the same
#  source gives the same image in both. Every path is taken from this
#  file's own place, never from the including project's. BRIO_BOARD_DIR,
#  when whoever includes this file sets it, is the directory of the board
#  file an image reads as "board.hpp".
#
#  ONE CHIP, ONE HEADER, and what a configure targets is not a part but a
#  FLASH: the RP2040 executes in place out of an external quad-SPI flash
#  chip whose size (RP2040_FLASH_KB, the linker script) and whose read
#  protocol (RP2040_BOOT2, the second-stage bootloader in src/glue/) are
#  the board's. A configure therefore serves EVERY board type with that
#  flash geometry and that stage, and RP2040_BOARDS lists them: pico, the
#  reference board; weact2040, a WeAct board; picow, a Pico W - the same
#  flash, but four of its pins are the radio chip's and its LED is on the
#  radio, so it is named apart.
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
#  The NVHEAP build-id defsym is a property of util/nv_heap.hpp: an image
#  that never names an NvHeap never references the symbol.
# ============================================================================

include_guard(GLOBAL)

get_filename_component(BRIO_FAMILY_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
get_filename_component(BRIO_ROOT "${BRIO_FAMILY_DIR}/.." ABSOLUTE)

# The boot stage is assembly: an application's project() need not name the
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

# --- What this configure targets: a flash geometry and its boot stage.
set(RP2040_FLASH_KB "2048" CACHE STRING "size of the board's QSPI flash in KB (the linker script)")
set(RP2040_BOOT2 "w25q080" CACHE STRING "second-stage bootloader: w25q080 (Winbond-class quad I/O) or generic_03h (any flash, slow)")
if(RP2040_FLASH_KB STREQUAL "2048")
    set(RP2040_LD "rp2040_2m")
    set(RP2040_BOARDS pico weact2040 picow)
else()
    message(FATAL_ERROR "RP2040_FLASH_KB=${RP2040_FLASH_KB}: no linker script for that flash size yet (ld/rp2040_2m.ld is the 2 MB one)")
endif()
set(RP2040_STARTUP "${BRIO_FAMILY_DIR}/src/glue/startup_rp2040.cpp")
set(RP2040_STAGE "${BRIO_FAMILY_DIR}/src/glue/boot2_${RP2040_BOOT2}.S")
if(NOT EXISTS "${RP2040_STAGE}")
    message(FATAL_ERROR "RP2040_BOOT2=${RP2040_BOOT2}: ${RP2040_STAGE} does not exist")
endif()

set(BRIO_COMMON_FLAGS
    -mcpu=cortex-m0plus
    -mthumb
    -mfloat-abi=soft
    -Wall -Wextra -Werror
    -ffunction-sections -fdata-sections
    -DBRIO_RP2040_FLASH_KB=${RP2040_FLASH_KB}   # what brio/rp2040/flash.hpp sizes the chip by
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
    "${BRIO_ROOT}/brio/*.cpp" "${BRIO_ROOT}/brio/*.hpp"
    "${BRIO_ROOT}/brio/*.c"   "${BRIO_ROOT}/brio/*.h"
    "${BRIO_ROOT}/brio/*.S"
    "${BRIO_FAMILY_DIR}/src/glue/*.cpp" "${BRIO_FAMILY_DIR}/src/glue/*.S"
    "${CMAKE_SOURCE_DIR}/*.cpp" "${CMAKE_SOURCE_DIR}/*.hpp"
    "${CMAKE_SOURCE_DIR}/*.c"   "${CMAKE_SOURCE_DIR}/*.h"
    "${CMAKE_SOURCE_DIR}/*.S"
)
foreach(_f IN LISTS _nvheap_id_sources)
    # A build directory kept inside the source directory is not source.
    string(FIND "${_f}" "${CMAKE_BINARY_DIR}/" _in_build)
    if(_in_build EQUAL 0 OR _f MATCHES "/CMakeFiles/")
        continue()
    endif()
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
        ${BRIO_ROOT}/brio/rt/rt.cpp
        ${ARGN}
    )

    target_include_directories(${name} PRIVATE
        ${BRIO_ROOT}/brio
        ${BRIO_ROOT}/third_party/pico-sdk/CMSIS
        ${BRIO_ROOT}/third_party/pico-sdk
        ${BRIO_ROOT}/third_party/cmsis-core
    )
    # A board file, when the includer names one: an app writes
    # #include "board.hpp" and reads the facts of that assembly.
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
    # link - an accidental printf or malloc fails here with its own name -
    # and libgcc named below, after the objects.
    target_link_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        -nostartfiles
        -nodefaultlibs
        -T${BRIO_FAMILY_DIR}/ld/${RP2040_LD}.ld
        -Wl,--gc-sections
        -Wl,--no-warn-rwx-segments   # .data carries code: .fast_text (placed for speed) and .ram_text (the flash engine's window) (ld/)
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${RP2040_NVHEAP_BUILD_ID}
    )
    target_link_libraries(${name} PRIVATE gcc)
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${BRIO_FAMILY_DIR}/ld/${RP2040_LD}.ld)

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

# The name every family's recipe answers to, which is the one an
# application kept outside the tree calls:
#
#     brio_add_app(<name> <source>...)
function(brio_add_app name)
    rp2040_add_app(${name} ${ARGN})
endfunction()
