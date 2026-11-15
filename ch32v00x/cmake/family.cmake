# ============================================================================
#  family.cmake - the CH32V00x family's build recipe: how ONE image is built
#  for a CH32V006 or a CH32V003 - the part's definitions and ISA, the flags,
#  the crt and the runtime every image compiles, the link, the .bin/.hex
#  and the listing, the -upload target.
#
#  Included after project() by this directory's CMakeLists.txt (the apps
#  kept in this tree) and by ../../cmake/brio.cmake (an application kept
#  outside it): both are built by ch32_add_app() below, so the same source
#  gives the same image in both. Every path is taken from this file's own
#  place, never from the including project's. BRIO_BOARD_DIR, when whoever
#  includes this file sets it, is the directory of the board file an image
#  reads as "board.hpp".
#
#  WHAT THIS FAMILY HAS NOT GOT, and the others have: a device header.
#  WCH ships its register definitions inside the EVT package, whose
#  licence is written for software running on WCH parts, so this
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

# --- Which part this configure targets: the part number in lower case,
#     with its package letter and memory code and without the package
#     type and temperature suffix (ch32v006k8, ch32v006f8, ch32v003f4).
#     It picks the linker script ld/<part>.ld, two DEFINITIONS the
#     stratum's device.hpp asks - the chip (CH32V006 / CH32V003: the
#     family has no vendor header, so the build states the part) and the
#     package (CH32V006F8 and the like: the pads it bonds, parts/<chip>.hpp)
#     - and, unless overridden, the ISA: the CH32V006's QingKe V2C has the
#     multiplier, the CH32V003's V2A has not. A part number whose letter
#     the chip's table does not know is refused by device.hpp.
set(CH32V00X_MCU "ch32v006k8" CACHE STRING "target CH32V00x part: ch32v006k8, ch32v006f8 or ch32v003f4")
string(SUBSTRING "${CH32V00X_MCU}" 0 8 _ch32_part)
string(TOUPPER "${_ch32_part}" CH32V00X_PART_DEFINE)                  # CH32V006 / CH32V003
string(TOUPPER "${CH32V00X_MCU}" CH32V00X_PACKAGE_DEFINE)             # CH32V006K8 / CH32V006F8 / CH32V003F4
if(NOT CH32V00X_PART_DEFINE MATCHES "^CH32V00[36]$")
    message(FATAL_ERROR "CH32V00X_MCU '${CH32V00X_MCU}': the part must be a CH32V006 or a CH32V003")
endif()
if(NOT CH32V00X_PACKAGE_DEFINE MATCHES "^CH32V00[36][A-Z][0-9]$")
    message(FATAL_ERROR "CH32V00X_MCU '${CH32V00X_MCU}': the part number must carry its package letter and memory code (ch32v006f8)")
endif()
# The board type: the part number without "ch32" (v006k8), which names
# the ASSEMBLY the build is for, and its board file's directory.
string(SUBSTRING "${CH32V00X_MCU}" 4 6 CH32V00X_BOARD)          # v006k8
if(CH32V00X_PART_DEFINE STREQUAL "CH32V003")
    set(_ch32_arch_default "rv32ec_xw")
else()
    set(_ch32_arch_default "rv32ec_zmmul_xw")
endif()
# THE ISA AND THE ABI ARE DERIVED, NEVER CACHED: the two cache entries
# are OVERRIDES, empty by default, so every configure re-derives from
# the part (a changed default and a changed CH32V00X_MCU on an existing
# cache both take effect), and an override that is set wins and says so.
# An entry equal to the derived value is no override, and is cleared
# (which is what a cache written while these entries held the default
# carries).
set(CH32V00X_ARCH "" CACHE STRING "-march override; empty (the default) = the part's own ISA under WCH's gcc, derived at every configure")
set(CH32V00X_ABI "" CACHE STRING "-mabi override; empty (the default) = the family's ilp32e")
set(_ch32_abi_default "ilp32e")
if(CH32V00X_ARCH STREQUAL "${_ch32_arch_default}")
    set(CH32V00X_ARCH "" CACHE STRING "-march override; empty (the default) = the part's own ISA under WCH's gcc, derived at every configure" FORCE)
endif()
if(CH32V00X_ARCH STREQUAL "")
    set(_ch32_arch "${_ch32_arch_default}")
else()
    set(_ch32_arch "${CH32V00X_ARCH}")
    message(STATUS "CH32V00X_ARCH override: -march=${_ch32_arch} (the part's own is ${_ch32_arch_default})")
endif()
if(CH32V00X_ABI STREQUAL "${_ch32_abi_default}")
    set(CH32V00X_ABI "" CACHE STRING "-mabi override; empty (the default) = the family's ilp32e" FORCE)
endif()
if(CH32V00X_ABI STREQUAL "")
    set(_ch32_abi "${_ch32_abi_default}")
else()
    set(_ch32_abi "${CH32V00X_ABI}")
    message(STATUS "CH32V00X_ABI override: -mabi=${_ch32_abi} (the family's is ${_ch32_abi_default})")
endif()

# --- The core's hardware prologue/epilogue (HPE): with it on, the crt
#     sets INTSYSCR.HWSTKEN and every handler is declared with WCH's
#     `interrupt("WCH-Interrupt-fast")` attribute (through pfic.hpp's
#     BRIO_CH32_VECTOR), so the ten caller-saved registers are pushed
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
    -march=${_ch32_arch}
    -mabi=${_ch32_abi}
    -D${CH32V00X_PART_DEFINE}
    -D${CH32V00X_PACKAGE_DEFINE}
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
    "${BRIO_ROOT}/brio/*.cpp" "${BRIO_ROOT}/brio/*.hpp"
    "${BRIO_ROOT}/brio/*.c"   "${BRIO_ROOT}/brio/*.h"
    "${BRIO_ROOT}/brio/*.S"
    "${BRIO_FAMILY_DIR}/src/glue/*.cpp" "${BRIO_FAMILY_DIR}/src/glue/*.S"
    "${CMAKE_SOURCE_DIR}/*.cpp" "${CMAKE_SOURCE_DIR}/*.hpp"
    "${CMAKE_SOURCE_DIR}/*.c"   "${CMAKE_SOURCE_DIR}/*.h"
    "${CMAKE_SOURCE_DIR}/*.S"
    "${BRIO_ROOT}/boards/*.hpp"
)
foreach(_f IN LISTS _nvheap_id_sources)
    # A build directory kept inside the source directory is not source.
    string(FIND "${_f}" "${CMAKE_BINARY_DIR}/" _in_build)
    if(_in_build EQUAL 0 OR _f MATCHES "/CMakeFiles/")
        continue()
    endif()
    file(TIMESTAMP "${_f}" _ts "%s")
    if(_ts GREATER CH32V00X_NVHEAP_BUILD_ID)
        set(CH32V00X_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()

set(CH32V00X_OPENOCD "/sw/wch-openocd/bin/openocd" CACHE FILEPATH "WCH's OpenOCD fork (SDI transport)")
set(CH32V00X_OPENOCD_CFG "/sw/wch-openocd/bin/wch-riscv.cfg" CACHE FILEPATH "its RISC-V target config")
# The probe is cmake/probe.cmake's: a WCH-Link, the one kind this family
# drives, which the target script above opens itself; BRIO_PROBE_SERIAL
# picks one of several, and CH32V00X_INTERFACE is then its `adapter serial`,
# placed before the target script (WCH's fork honours it there:
# cli/bench/flash.py, wch_openocd_args). With no serial it is empty. The
# OpenOCD binary and the target script are this family's.
include("${BRIO_ROOT}/cmake/probe.cmake")
brio_probe_select(DEFAULT wchlink)
brio_probe_openocd(CH32V00X_INTERFACE)

# --- One app -> one executable target, one .bin/.hex, one -upload target.
function(ch32_add_app name)
    add_executable(${name}
        ${BRIO_FAMILY_DIR}/src/glue/startup_ch32v00x.S
        ${BRIO_ROOT}/brio/rt/rt.cpp
        ${ARGN}
    )

    target_include_directories(${name} PRIVATE
        ${BRIO_ROOT}/brio
    )
    # The board file of the board type this configure builds for,
    # ../boards/<type>/board.hpp, on the include path when the type has
    # one: an app writes #include "board.hpp" and reads the facts of that
    # assembly (the LED, the crystal, the pads the board's circuits load) -
    # facts of the PCB and what is fitted on it, which the part cannot know.
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
        -T${BRIO_FAMILY_DIR}/ld/${CH32V00X_MCU}.ld
        -Wl,--gc-sections
        -Wl,--no-warn-rwx-segments   # .data carries the code placed in SRAM, .fast_text and .ram_text (ld/)
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${CH32V00X_NVHEAP_BUILD_ID}
    )
    target_link_libraries(${name} PRIVATE gcc)
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${BRIO_FAMILY_DIR}/ld/${CH32V00X_MCU}.ld)

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
            ${CH32V00X_INTERFACE}
            -f ${CH32V00X_OPENOCD_CFG}
            -c "program $<TARGET_FILE:${name}> verify"
            -c "reset run"
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
    ch32_add_app(${name} ${ARGN})
endfunction()
