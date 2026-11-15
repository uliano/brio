# ============================================================================
#  family.cmake - the CH32X035 family's build recipe: how ONE image is built
#  for a CH32X035 or CH32X033 part - the part's definition, ISA and ABI,
#  the flags, the crt and the runtime every image compiles, the two linker
#  scripts and the scheduler's placement, the .bin/.hex and the listing,
#  the -upload target.
#
#  Included after project() by this directory's CMakeLists.txt (the apps
#  kept in this tree) and by ../../cmake/brio.cmake (an application kept
#  outside it): both are built by ch32x035_add_app() below, so the same
#  source gives the same image in both. Every path is taken from this
#  file's own place, never from the including project's. BRIO_BOARD_DIR,
#  when whoever includes this file sets it, is the directory of the board
#  file an image reads as "board.hpp".
#
#  The part is CH32X035_MCU; ch32x035-parts.cmake, beside this file, is
#  the table of what it decides.
#
#  WHAT THIS FAMILY HAS NOT GOT, and the ST ones have: a device header.
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
#  Flash/debug: WCH's OpenOCD fork at /sw/wch-openocd, the only OpenOCD
#  that speaks the WCH-Link's SDI transport, over the two-wire debug port
#  this series has on PC18 (SWDIO) and PC19 (SWCLK).
# ============================================================================

include_guard(GLOBAL)

get_filename_component(BRIO_FAMILY_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
get_filename_component(BRIO_ROOT "${BRIO_FAMILY_DIR}/.." ABSOLUTE)

# The crt is assembly: an application's project() need not name the language.
enable_language(ASM)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

foreach(lang C CXX)
    set(CMAKE_${lang}_FLAGS_RELEASE "" CACHE STRING "" FORCE)
    set(CMAKE_${lang}_FLAGS_DEBUG "" CACHE STRING "" FORCE)
endforeach()

include(${CMAKE_CURRENT_LIST_DIR}/ch32x035-parts.cmake)

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
#     BRIO_CH32_VECTOR), so the caller-saved registers are pushed and
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
    if(_ts GREATER CH32X035_NVHEAP_BUILD_ID)
        set(CH32X035_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()

set(CH32X035_OPENOCD "/sw/wch-openocd/bin/openocd" CACHE FILEPATH "WCH's OpenOCD fork (SDI transport)")
set(CH32X035_OPENOCD_CFG "/sw/wch-openocd/bin/wch-riscv.cfg" CACHE FILEPATH "its RISC-V target config")

# --- One app -> one executable target, one .bin/.hex, one -upload target.
#     `scheduler` is ram or flash - an in-tree app's "// build: scheduler"
#     value, which the discovery in ../CMakeLists.txt reads and checks:
#     where the preemptive kernel's scheduler path runs from.
#
#     THE SCHEDULER'S PLACE is two things the value sets for the target.
#     The compile definition BRIO_CH32_SCHEDULER_IN_RAM (1 or 0) tells the
#     code where its request runs, so the request carries the fence.i a
#     store run from SRAM needs and scheduler_init() points mtvec at a copy
#     of the vector table in SRAM (ch32x035/schedule.hpp). And the linker's
#     search path names the directory of the placement fragment ld/
#     sections.ld includes into .data, ld/scheduler_ram/ or ld/
#     scheduler_flash/, which name the path's sections or none. The -L comes
#     before the two scripts: ld looks a script's INCLUDE up in the
#     directories named before it. The kernel knows nothing of the choice,
#     and an image without the preemptive kernel has none of the path's
#     functions, so the default costs it nothing.
function(ch32x035_add_app name scheduler)
    add_executable(${name}
        ${BRIO_FAMILY_DIR}/src/glue/startup_ch32x035.S
        ${BRIO_ROOT}/brio/rt/rt.cpp
        ${ARGN}
    )

    target_include_directories(${name} PRIVATE
        ${BRIO_ROOT}/brio
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
    if(scheduler STREQUAL "ram")
        target_compile_definitions(${name} PRIVATE BRIO_CH32_SCHEDULER_IN_RAM=1)
    else()
        target_compile_definitions(${name} PRIVATE BRIO_CH32_SCHEDULER_IN_RAM=0)
    endif()

    # The part's memories, then the series' sections (ld/sections.ld says
    # why two scripts and in this order).
    target_link_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        -nostartfiles
        -nodefaultlibs
        -L${BRIO_FAMILY_DIR}/ld/scheduler_${scheduler}
        -T${BRIO_FAMILY_DIR}/ld/${CH32X035_MCU}.ld
        -T${BRIO_FAMILY_DIR}/ld/sections.ld
        -Wl,--gc-sections
        -Wl,--no-warn-rwx-segments   # .data carries .ram_text, the code placed in SRAM: code copied there is a writable, executable segment (ld/)
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${CH32X035_NVHEAP_BUILD_ID}
    )
    target_link_libraries(${name} PRIVATE gcc)
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${BRIO_FAMILY_DIR}/ld/${CH32X035_MCU}.ld
        ${BRIO_FAMILY_DIR}/ld/sections.ld
        ${BRIO_FAMILY_DIR}/ld/scheduler_${scheduler}/scheduler_text.ld)

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

# The name every family's recipe answers to, which is the one an
# application kept outside the tree calls:
#
#     brio_add_app(<name> <source>... [SCHEDULER ram|flash])
#
# SCHEDULER is this family's own option: where the preemptive kernel's
# scheduler path runs (ch32x035_add_app() above says what each place
# sets), ram when absent - the in-tree default for an app with no
# "// build: scheduler" line.
function(brio_add_app name)
    cmake_parse_arguments(A "" "SCHEDULER" "" ${ARGN})
    if(NOT DEFINED A_SCHEDULER)
        set(A_SCHEDULER ram)
    endif()
    if(NOT A_SCHEDULER STREQUAL "ram" AND NOT A_SCHEDULER STREQUAL "flash")
        message(FATAL_ERROR "brio_add_app(${name}): SCHEDULER ${A_SCHEDULER} - the scheduler's place is ram or flash")
    endif()
    ch32x035_add_app(${name} ${A_SCHEDULER} ${A_UNPARSED_ARGUMENTS})
endfunction()
