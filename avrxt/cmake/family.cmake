# ============================================================================
#  family.cmake - the AVR family's build recipe: how ONE image is built for
#  an AVR DA, DB or EA part - the flags, the glue every image compiles, the
#  link, the .hex and the listing, the -upload target.
#
#  Included after project() by this directory's CMakeLists.txt (the apps
#  kept in this tree) and by ../../cmake/brio.cmake (an application kept
#  outside it): both are built by avr_add_app() below, so the same source
#  gives the same image in both. Every path is taken from this file's own
#  place, never from the including project's.
#
#  The part is AVR_MCU (cmake/avr-mcus.cmake is the table of the parts this
#  recipe knows). BRIO_BOARD_DIR, when whoever includes this file sets it,
#  is the directory of the board file an image reads as "board.hpp".
# ============================================================================

include_guard(GLOBAL)

get_filename_component(BRIO_FAMILY_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
get_filename_component(BRIO_ROOT "${BRIO_FAMILY_DIR}/.." ABSOLUTE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Replace CMake's own per-config defaults (-O3 -DNDEBUG for Release, -g for
# Debug) so the ONLY optimization/debug flags in effect are the ones set
# explicitly below, instead of layering on top of them.
foreach(lang C CXX)
    set(CMAKE_${lang}_FLAGS_RELEASE "" CACHE STRING "" FORCE)
    set(CMAKE_${lang}_FLAGS_DEBUG "" CACHE STRING "" FORCE)
endforeach()

include(${CMAKE_CURRENT_LIST_DIR}/avr-mcus.cmake)

# --- Which AVR package this configure targets - one CMake configure = one
#     package, selected by a configurePreset. avr-mcus.cmake is the
#     reference for valid values.
set(AVR_MCU "avr128db48" CACHE STRING "target AVR MCU, e.g. avr128db48/avr128da48/avr64ea48 (see cmake/avr-mcus.cmake)")
string(TOUPPER "${AVR_MCU}" AVR_MCU_DEFINE)
avr_short_board("${AVR_MCU}" AVR_BOARD)

# --- Flags shared by every language + the link step (CCFLAGS applies to C,
#     C++ and the link step alike).
set(BRIO_COMMON_FLAGS
    -mmcu=${AVR_MCU}
    -D${AVR_MCU_DEFINE}
    -Wall -Wextra -Werror
    -ffunction-sections -fdata-sections
)

# --- The -mmcu device-macro delta, for clangd's benefit. clang does not run
#     avr-gcc's device-specs machinery, so it never learns __AVR_DEVICE_NAME__,
#     __AVR_HAVE_*, __AVR_SFR_OFFSET__ and friends from a bare `-mmcu=...` on
#     the command line - <avr/io.h> fails on device-specific symbols without
#     this (confirmed during the migration POC: CLKCTRL_FRQSEL_1M_gc, gated
#     on a __AVR_HAVE_* macro). --query-driver only recovers the system
#     include paths, not these; the fix is to make them literal -D flags in
#     compile_commands.json, which any compiler (real or clangd's) reads the
#     same way. Diffed against the no-mcu baseline so only the
#     device-specific delta is added - the real build gets the identical
#     values from -mmcu itself, so this never causes a redefinition warning.
#     An AVR peculiarity (device-specs macros; other targets select the
#     device with an explicit -D) that goes away with the second target.
function(avr_predefines out_var)
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} ${ARGN} -x c++ -E -dM -
        INPUT_FILE /dev/null
        OUTPUT_VARIABLE _dump
        RESULT_VARIABLE _rc
    )
    if(NOT _rc EQUAL 0)
        message(FATAL_ERROR "avr-g++ -dM query failed (args: ${ARGN})")
    endif()
    string(REPLACE "\n" ";" _lines "${_dump}")
    set(${out_var} "${_lines}" PARENT_SCOPE)
endfunction()

avr_predefines(_base_lines)
avr_predefines(_mcu_lines -mmcu=${AVR_MCU})

set(_base_defs "")
foreach(_line IN LISTS _base_lines)
    if(_line MATCHES "^#define ([^ ]+)( (.*))?$")
        list(APPEND _base_defs "${CMAKE_MATCH_1}=${CMAKE_MATCH_3}")
    endif()
endforeach()

set(AVR_MCU_DEFINES "")
foreach(_line IN LISTS _mcu_lines)
    if(_line MATCHES "^#define (__AVR[^ ]*)( (.*))?$")
        set(_entry "${CMAKE_MATCH_1}=${CMAKE_MATCH_3}")
        list(FIND _base_defs "${_entry}" _idx)
        if(_idx EQUAL -1)
            if(CMAKE_MATCH_3 STREQUAL "")
                list(APPEND AVR_MCU_DEFINES "${CMAKE_MATCH_1}")
            else()
                list(APPEND AVR_MCU_DEFINES "${_entry}")
            endif()
        endif()
    endif()
endforeach()
message(STATUS "avr mcu define delta for ${AVR_MCU}: ${AVR_MCU_DEFINES}")

find_program(AVR_OBJCOPY avr-objcopy PATHS "${AVR_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(AVR_SIZE avr-size PATHS "${AVR_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(AVR_OBJDUMP avr-objdump PATHS "${AVR_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)

# --- Flash over UPDI (Atmel-ICE). -P usb:<serial> disambiguates which
#     Atmel-ICE when more than one is attached (bin/brio does the same
#     from its manifest, which is the way to flash a desk with several
#     boards). Empty = the only probe attached; set it with
#     -DAVR_PROBE_SERIAL=<serial> (bin/brio list prints them).
set(AVR_PROBE_SERIAL "" CACHE STRING "Atmel-ICE serial to flash/debug through (empty: the only one attached)")
if(AVR_PROBE_SERIAL STREQUAL "")
    set(AVR_PROBE_PORT "usb")
else()
    set(AVR_PROBE_PORT "usb:${AVR_PROBE_SERIAL}")
endif()

# --- The build id every image carries: avrxt/nvm_flash.hpp records it in
#     every map version of a flash NvHeap, where it is a DIAGNOSTIC and
#     nothing else - a stored block is judged by its checksum, never by
#     which build wrote it. It is the NEWEST SOURCE TIMESTAMP, not the time
#     of the link: brio flash reprograms without erasing, which on
#     this part only clears bits, so reflashing is safe only while the image
#     is byte-identical to the one already in the chip - a wall-clock id
#     would break that for every app at every relink. An unchanged tree
#     relinks to the same bytes; a real source change gets a new id (and
#     needs the chip erase anyway). Passed to every app's link
#     unconditionally - an image that never names NvHeap never references
#     the symbol, so this costs it nothing, not even a byte.
set(AVR_NVHEAP_BUILD_ID 0)
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
    if(_ts GREATER AVR_NVHEAP_BUILD_ID)
        set(AVR_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()
message(STATUS "avr nvheap build id (newest source timestamp): ${AVR_NVHEAP_BUILD_ID}")

# --- One app -> one executable target, one .hex, one -upload target. Every
#     app gets the identical flag set; the per-app inputs are its name, its
#     source file, and an optional FLMAP_LOCK override (see
#     avr_discover_apps() below; "// build: flmap_lock = 0" in the app's
#     own header, only test_avr_nvm.cpp uses this today).
function(avr_add_app name)
    cmake_parse_arguments(A "" "FLMAP_LOCK" "" ${ARGN})
    if(NOT DEFINED A_FLMAP_LOCK)
        set(A_FLMAP_LOCK 1)
    endif()

    add_executable(${name}
        ${BRIO_FAMILY_DIR}/src/glue/ivsel_boot.cpp
        ${A_UNPARSED_ARGUMENTS}
    )

    target_include_directories(${name} PRIVATE ${BRIO_ROOT}/brio)
    # The board file of the board type this configure builds for,
    # ../boards/<type>/board.hpp, on the include path when the type has
    # one: an app writes #include "board.hpp" and reads the facts of that
    # assembly (the HF source and its rate, the 32 kHz crystal, what hangs
    # on PA7) - facts of the PCB and what is fitted on it, which the chip's
    # header cannot know. The packages with no board (db28, db32) have no
    # directory, and an app that includes the file fails to build there.
    if(BRIO_BOARD_DIR AND EXISTS ${BRIO_BOARD_DIR})
        target_include_directories(${name} PRIVATE ${BRIO_BOARD_DIR})
    endif()
    target_compile_definitions(${name} PRIVATE ${AVR_MCU_DEFINES})

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

    # No -flto (it splits DWARF line info across partition CUs and avr-gdb
    # then refuses file:line breakpoints). No -fpermissive/
    # -Wno-error=narrowing either (ill-formed code and narrowing conversions
    # must fail, per house rules) - both flags are simply never added here,
    # since this build's flags are set from scratch rather than inherited
    # from a framework default that injected them.
    # The refused C library (design/runtime.md): cmake/avr-refused-libc.rsp
    # wraps every entry into avr-libc's heap and its stdio, and no image
    # defines a wrapper, so a call to one fails the link as an undefined
    # `__wrap_<name>` at the caller's own line. brio check avrxt links a
    # reference to each and expects every one refused.
    target_link_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        -Wl,--gc-sections
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--section-start=.text=0x0
        -Wl,--defsym,__flmap_lock=${A_FLMAP_LOCK}
        -Wl,--defsym,__nvheap_build_id=${AVR_NVHEAP_BUILD_ID}
        -Wl,@${BRIO_FAMILY_DIR}/cmake/avr-refused-libc.rsp
        -lm
    )
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${BRIO_FAMILY_DIR}/cmake/avr-refused-libc.rsp)

    set_target_properties(${name} PROPERTIES SUFFIX ".elf")

    # NOTE: this self-built avr-size is plain binutils size, without the AVR
    # memory-usage patch (`--mcu=... -C`); `-A -d` is the form that works
    # (matches what a per-build percentage-bar report would parse).
    add_custom_command(TARGET ${name} POST_BUILD
        COMMAND ${AVR_OBJCOPY} -O ihex -R .eeprom
            $<TARGET_FILE:${name}> $<TARGET_FILE_DIR:${name}>/${name}.hex
        COMMAND ${AVR_SIZE} -A -d $<TARGET_FILE:${name}>
        COMMAND ${AVR_OBJDUMP} -h -S $<TARGET_FILE:${name}> > $<TARGET_FILE_DIR:${name}>/${name}.lst
        COMMENT "${name}.hex + size + ${name}.lst"
        VERBATIM
    )

    add_custom_target(${name}-upload
        COMMAND ${AVR_TOOLCHAIN_DIR}/bin/avrdude -p ${AVR_MCU} -c atmelice_updi
            -P ${AVR_PROBE_PORT}
            -U flash:w:$<TARGET_FILE_DIR:${name}>/${name}.hex:i
        DEPENDS ${name}
        VERBATIM
    )
endfunction()

# The name every family's recipe answers to, which is the one an
# application kept outside the tree calls:
#
#     brio_add_app(<name> <source>... [FLMAP_LOCK 0|1])
#
# FLMAP_LOCK is this family's own option (1 by default: the image locks
# the flash mapping at start; 0 for an image that moves it).
function(brio_add_app name)
    avr_add_app(${name} ${ARGN})
endfunction()
