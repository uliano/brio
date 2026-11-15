# ============================================================================
#  family.cmake - the STM32F4 family's build recipe: how ONE image is built
#  for an STM32F4 part - the flags, the crt and the runtime every image
#  compiles, the link, the .bin/.hex and the listing, the -upload target.
#
#  Included after project() by this directory's CMakeLists.txt (the apps
#  kept in this tree) and by ../../cmake/brio.cmake (an application kept
#  outside it): both are built by stm32_add_app() below, so the same source
#  gives the same image in both. Every path is taken from this file's own
#  place, never from the including project's. BRIO_BOARD_DIR, when whoever
#  includes this file sets it, is the directory of the board file an image
#  reads as "board.hpp".
#
#  The part is STM32F4_MCU. What the part decides - the device define, the
#  crt, the linker script, the board type - is stm32f4-parts.cmake's table,
#  beside this file.
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
#  The NVHEAP build-id defsym is a property of util/nv_heap.hpp: an image
#  that never names an NvHeap never references the symbol.
# ============================================================================

include_guard(GLOBAL)

get_filename_component(BRIO_FAMILY_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
get_filename_component(BRIO_ROOT "${BRIO_FAMILY_DIR}/.." ABSOLUTE)

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
include(${CMAKE_CURRENT_LIST_DIR}/stm32f4-parts.cmake)
set(STM32F4_MCU "stm32f429zi" CACHE STRING "target STM32F4 part, e.g. stm32f429zi")
stm32f4_part_facts("${STM32F4_MCU}" STM32F4_DEFINE STM32F4_STARTUP STM32F4_BOARD)
set(STM32F4_STARTUP "${BRIO_FAMILY_DIR}/${STM32F4_STARTUP}")   # the table names it from the family's directory

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
    if(_ts GREATER STM32F4_NVHEAP_BUILD_ID)
        set(STM32F4_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()

# --- One app -> one executable target, one .bin/.hex, one -upload target.
function(stm32_add_app name)
    add_executable(${name}
        ${STM32F4_STARTUP}
        ${BRIO_ROOT}/brio/rt/rt.cpp
        ${ARGN}
    )

    target_include_directories(${name} PRIVATE
        ${BRIO_ROOT}/brio
        ${BRIO_ROOT}/third_party/cmsis-device-f4/Include
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
        -T${BRIO_FAMILY_DIR}/ld/${STM32F4_MCU}.ld
        -Wl,--gc-sections
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${STM32F4_NVHEAP_BUILD_ID}
    )
    target_link_libraries(${name} PRIVATE gcc)
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${BRIO_FAMILY_DIR}/ld/${STM32F4_MCU}.ld)

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

# The name every family's recipe answers to, which is the one an
# application kept outside the tree calls:
#
#     brio_add_app(<name> <source>...)
function(brio_add_app name)
    stm32_add_app(${name} ${ARGN})
endfunction()
