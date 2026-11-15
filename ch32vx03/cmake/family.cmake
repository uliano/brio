# ============================================================================
#  family.cmake - the CH32V203/CH32V303 family's build recipe: how ONE image
#  is built for a part of either series - the part's definition, ISA and
#  ABI, the flags, the crt and the runtime every image compiles, the link,
#  the .bin/.hex and the listing, the -upload target.
#
#  Included after project() by this directory's CMakeLists.txt (the apps
#  kept in this tree) and by ../../cmake/brio.cmake (an application kept
#  outside it): both are built by ch32vx03_add_app() below, so the same
#  source gives the same image in both. Every path is taken from this
#  file's own place, never from the including project's. BRIO_BOARD_DIR,
#  when whoever includes this file sets it, is the directory of the board
#  file an image reads as "board.hpp".
#
#  The part is CH32VX03_MCU; ch32vx03-parts.cmake, beside this file, is
#  the table of what it decides.
#
#  WHAT THIS FAMILY HAS NOT GOT, and the ST ones have: a device header.
#  WCH ships its register definitions inside the EVT package, whose licence
#  is written for software running on WCH parts, so this target compiles
#  against brio/ch32vx03/device.hpp instead - the map read off the
#  reference manual. The part is therefore a definition of THIS build's
#  making (cmake/ch32vx03-parts.cmake), asked once in device.hpp, which
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

include(${CMAKE_CURRENT_LIST_DIR}/ch32vx03-parts.cmake)

set(CH32VX03_MCU "ch32v203c8" CACHE STRING "target CH32V203/CH32V303 part (cmake/ch32vx03-parts.cmake)")
ch32vx03_part_facts("${CH32VX03_MCU}"
    CH32VX03_PART_DEFINE CH32VX03_FLASH_KB CH32VX03_RAM_KB CH32VX03_BOARD
    CH32VX03_PART_ARCH CH32VX03_PART_ABI)

# The part's own ISA and ABI (the table's two last columns) are DERIVED
# at every configure, never cached: the two cache entries are OVERRIDES,
# empty by default - for an experiment that builds a V4F part without
# its FPU - so a changed CH32VX03_MCU on an existing cache follows the
# part, and an override that is set wins and says so.
# An entry equal to the derived value is no override, and is cleared
# (which is what a cache written while these entries held the default
# carries).
set(CH32VX03_ARCH "" CACHE STRING "-march override; empty (the default) = the part's own ISA (cmake/ch32vx03-parts.cmake), derived at every configure")
set(CH32VX03_ABI "" CACHE STRING "-mabi override; empty (the default) = the part's own ABI")
if(CH32VX03_ARCH STREQUAL "${CH32VX03_PART_ARCH}")
    set(CH32VX03_ARCH "" CACHE STRING "-march override; empty (the default) = the part's own ISA (cmake/ch32vx03-parts.cmake), derived at every configure" FORCE)
endif()
if(CH32VX03_ARCH STREQUAL "")
    set(_vx03_arch "${CH32VX03_PART_ARCH}")
else()
    set(_vx03_arch "${CH32VX03_ARCH}")
    message(STATUS "CH32VX03_ARCH override: -march=${_vx03_arch} (${CH32VX03_MCU}'s own is ${CH32VX03_PART_ARCH})")
endif()
if(CH32VX03_ABI STREQUAL "${CH32VX03_PART_ABI}")
    set(CH32VX03_ABI "" CACHE STRING "-mabi override; empty (the default) = the part's own ABI" FORCE)
endif()
if(CH32VX03_ABI STREQUAL "")
    set(_vx03_abi "${CH32VX03_PART_ABI}")
else()
    set(_vx03_abi "${CH32VX03_ABI}")
    message(STATUS "CH32VX03_ABI override: -mabi=${_vx03_abi} (${CH32VX03_MCU}'s own is ${CH32VX03_PART_ABI})")
endif()

# --- The core's hardware prologue/epilogue (HPE): with it on, the crt
#     sets INTSYSCR.HWSTKEN and every vector is bound (through pfic.hpp's
#     BRIO_CH32_VECTOR) with WCH's `interrupt("WCH-Interrupt-fast")`
#     attribute - or, in an image with the F extension, as a naked
#     trampoline to an ordinary function - so the caller-saved integer
#     registers are pushed and popped by the hardware instead of by each
#     handler's prologue. ONE
#     SWITCH FOR THE WHOLE IMAGE, on purpose: a fast handler under an HPE
#     that is off corrupts the interrupted program, so the crt bit and the
#     attribute must never disagree. Interrupt NESTING stays off either
#     way (the kernel's rule, docs/design/kernel.md) - which is where this
#     build differs from WCH's own startup file, that enables nesting.
option(CH32VX03_HPE "hardware prologue/epilogue for every interrupt handler" ON)
if(CH32VX03_HPE)
    set(CH32VX03_HPE_DEFINE -DBRIO_CH32_HPE=1)
else()
    set(CH32VX03_HPE_DEFINE "")
endif()

set(BRIO_COMMON_FLAGS
    -march=${_vx03_arch}
    -mabi=${_vx03_abi}
    -D${CH32VX03_PART_DEFINE}
    ${CH32VX03_HPE_DEFINE}
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
set(CH32VX03_NVHEAP_BUILD_ID 0)
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
    if(_ts GREATER CH32VX03_NVHEAP_BUILD_ID)
        set(CH32VX03_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()

set(CH32VX03_OPENOCD "/sw/wch-openocd/bin/openocd" CACHE FILEPATH "WCH's OpenOCD fork (SDI transport)")
set(CH32VX03_OPENOCD_CFG "/sw/wch-openocd/bin/wch-riscv.cfg" CACHE FILEPATH "its RISC-V target config")
# The probe is cmake/probe.cmake's: a WCH-Link, the one kind this family
# drives, which the target script above opens itself; BRIO_PROBE_SERIAL
# picks one of several, and CH32VX03_INTERFACE is then its `adapter serial`,
# placed before the target script (WCH's fork honours it there:
# cli/bench/flash.py, wch_openocd_args). With no serial it is empty. The
# OpenOCD binary and the target script are this family's.
include("${BRIO_ROOT}/cmake/probe.cmake")
brio_probe_select(DEFAULT wchlink)
brio_probe_openocd(CH32VX03_INTERFACE)

# --- One app -> one executable target, one .bin/.hex, one -upload target.
function(ch32vx03_add_app name)
    add_executable(${name}
        ${BRIO_FAMILY_DIR}/src/glue/startup_ch32vx03.S
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
    target_link_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        -nostartfiles
        -nodefaultlibs
        -T${BRIO_FAMILY_DIR}/ld/${CH32VX03_MCU}.ld
        -Wl,--gc-sections
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${CH32VX03_NVHEAP_BUILD_ID}
    )
    target_link_libraries(${name} PRIVATE gcc)
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${BRIO_FAMILY_DIR}/ld/${CH32VX03_MCU}.ld)

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
        COMMAND ${CH32VX03_OPENOCD}
            ${CH32VX03_INTERFACE}
            -f ${CH32VX03_OPENOCD_CFG}
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
#     brio_add_app(<name> <source>...)
function(brio_add_app name)
    ch32vx03_add_app(${name} ${ARGN})
endfunction()
