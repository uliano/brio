# ============================================================================
#  family.cmake - the SAM C21 family's build recipe: how ONE image is built
#  for a SAM C21 part - the flags, the crt and the runtime every image
#  compiles, the link, the .bin/.hex and the listing, the -upload target.
#
#  Included after project() by this directory's CMakeLists.txt (the apps
#  kept in this tree) and by ../../cmake/brio.cmake (an application kept
#  outside it): both are built by sam_add_app() below, so the same source
#  gives the same image in both. Every path is taken from this file's own
#  place, never from the including project's. BRIO_BOARD_DIR, when whoever
#  includes this file sets it, is the directory of the board file an image
#  reads as "board.hpp".
#
#  The part is SAMC21_MCU, the DFP header's basename
#  (third_party/samc21-dfp/include/<SAMC21_MCU>.h).
#
#  What the AVR project needs and this one deliberately does NOT:
#  - avr_predefines(): clang parses -mcpu=cortex-m0plus directly, and the
#    device is selected by an ordinary -D define - no device-specs macro
#    delta exists to feed clangd.
#  - The FLMAP defsym and the ivsel_boot glue: AVR-Dx build invariants
#    with no SAM analog (there is no flash-mapping window here, and the
#    vector table is the crt's). The glue slot on this side is the
#    startup file (vector table + reset path), which every image links
#    instead. The NVHEAP build-id defsym, by contrast, IS shared - it is
#    a property of util/nv_heap.hpp and not of either architecture, and
#    both projects compute it the same way.
# ============================================================================

include_guard(GLOBAL)

get_filename_component(BRIO_FAMILY_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
get_filename_component(BRIO_ROOT "${BRIO_FAMILY_DIR}/.." ABSOLUTE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Replace CMake's own per-config defaults so the ONLY optimization/debug
# flags in effect are the ones set explicitly below (same rule as the AVR
# and host projects).
foreach(lang C CXX)
    set(CMAKE_${lang}_FLAGS_RELEASE "" CACHE STRING "" FORCE)
    set(CMAKE_${lang}_FLAGS_DEBUG "" CACHE STRING "" FORCE)
endforeach()

# --- Which chip variant this configure targets. The value is the DFP
#     header's basename (third_party/samc21-dfp/include/<SAMC21_MCU>.h); the
#     device-select define is derived from it (samc21j18a -> __SAMC21J18A__).
set(SAMC21_MCU "samc21j18a" CACHE STRING "target SAM C21 variant, e.g. samc21j18a (see third_party/samc21-dfp/include)")
string(TOUPPER "${SAMC21_MCU}" _mcu_upper)
set(SAMC21_MCU_DEFINE "__${_mcu_upper}__")

# --- Flags shared by every language + the link step.
set(BRIO_COMMON_FLAGS
    -mcpu=cortex-m0plus
    -mthumb
    -mfloat-abi=soft
    -D${SAMC21_MCU_DEFINE}
    -Wall -Wextra -Werror
    -ffunction-sections -fdata-sections
)

find_program(ARM_OBJCOPY arm-none-eabi-objcopy PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(ARM_SIZE arm-none-eabi-size PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(ARM_OBJDUMP arm-none-eabi-objdump PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(ARM_GDB arm-none-eabi-gdb PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)

# --- Flash over SWD. The probe is cmake/probe.cmake's: BRIO_PROBE picks
#     its kind (the Atmel-ICE as a CMSIS-DAP probe when empty),
#     BRIO_PROBE_SERIAL one probe of several, and SAMC21_INTERFACE is the
#     interface part of the OpenOCD command line. The OpenOCD binary and
#     the target script are this family's: target/at91samdXX.cfg, whose
#     at91samd flash driver auto-probes the geometry from the DSU DID.
#     bin/brio names the probe from its manifest instead. A Black Magic
#     Probe (bmp) is driven by the toolchain's gdb, not by OpenOCD.
set(SAMC21_OPENOCD "/sw/openocd/bin/openocd" CACHE FILEPATH "OpenOCD with CMSIS-DAP + at91samd support")
include("${BRIO_ROOT}/cmake/probe.cmake")
brio_probe_select(DEFAULT atmelice KINDS cmsis-dap jlink bmp)
if(BRIO_PROBE_KIND STREQUAL "bmp")
    if(NOT ARM_GDB)
        message(FATAL_ERROR "brio: BRIO_PROBE=bmp needs arm-none-eabi-gdb, not found in ${ARM_TOOLCHAIN_DIR}/bin")
    endif()
else()
    brio_probe_openocd(SAMC21_INTERFACE)
endif()

# --- The build id every image carries: samc21/nvm_flash.hpp records it in
#     every map version of a flash NvHeap, where it is a DIAGNOSTIC and
#     nothing else - a stored block is judged by its checksum, never by
#     which build wrote it. It is the NEWEST SOURCE TIMESTAMP and not the
#     time of the link, for the same reason as on the AVR side: an
#     unchanged tree must relink to the same bytes, or every reflash of an
#     unchanged app would look like a different image. Passed to every
#     app's link unconditionally - an image that never names NvHeap never
#     references the symbol, so this costs it nothing, not even a byte.
set(SAMC21_NVHEAP_BUILD_ID 0)
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
    if(_ts GREATER SAMC21_NVHEAP_BUILD_ID)
        set(SAMC21_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()

# --- One app -> one executable target, one .bin/.hex, one -upload target.
function(sam_add_app name)
    add_executable(${name}
        ${BRIO_FAMILY_DIR}/src/glue/startup_samc21.cpp
        ${BRIO_ROOT}/brio/rt/rt.cpp
        ${ARGN}
    )

    target_include_directories(${name} PRIVATE
        ${BRIO_ROOT}/brio
        ${BRIO_ROOT}/third_party/samc21-dfp/include
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
    # and libgcc named below, after the objects. No -flto (same DWARF/gdb
    # rationale as the AVR build).
    target_link_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        -nostartfiles
        -nodefaultlibs
        -T${BRIO_FAMILY_DIR}/ld/${SAMC21_MCU}.ld
        -Wl,--gc-sections
        -Wl,--no-warn-rwx-segments   # .data carries .fast_text, the code a program places in SRAM: code copied there is a writable, executable segment (ld/)
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${SAMC21_NVHEAP_BUILD_ID}
    )
    target_link_libraries(${name} PRIVATE gcc)
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${BRIO_FAMILY_DIR}/ld/${SAMC21_MCU}.ld)

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

    # On a Black Magic Probe the upload target is gdb's scripted load
    # (cmake/probe.cmake, brio_probe_bmp()), and OpenOCD's target script
    # and its `program ... verify reset` are not run.
    if(BRIO_PROBE_KIND STREQUAL "bmp")
        brio_probe_bmp(_upload "${ARM_GDB}" "$<TARGET_FILE:${name}>")
        add_custom_target(${name}-upload
            COMMAND ${_upload}
            DEPENDS ${name}
            VERBATIM
        )
    else()
        add_custom_target(${name}-upload
            COMMAND ${SAMC21_OPENOCD}
                ${SAMC21_INTERFACE}
                -f target/at91samdXX.cfg
                -c "program $<TARGET_FILE:${name}> verify reset exit"
            DEPENDS ${name}
            VERBATIM
        )
    endif()
endfunction()

# The name every family's recipe answers to, which is the one an
# application kept outside the tree calls:
#
#     brio_add_app(<name> <source>...)
function(brio_add_app name)
    sam_add_app(${name} ${ARGN})
endfunction()
