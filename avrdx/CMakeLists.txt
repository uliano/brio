# ============================================================================
#  CMakeLists.txt - the AVR build (brio's sole build system; PlatformIO is
#  gone, see git history for the old platformio.ini/apps.ini-based setup).
#
#  One executable target per (app x AVR128DB package): every src/apps/*.cpp
#  with a main() is discovered automatically at configure time
#  (CONFIGURE_DEPENDS), gated by its own "// build: boards = ..." header
#  comment (default: db48 only, matching every app that doesn't name the
#  line explicitly). A CMake configure targets exactly ONE package
#  (AVR_MCU cache var, one value per configurePreset in CMakePresets.json) -
#  that is what replaces PlatformIO's per-(app x board) env multiplication:
#  switching CMake Tools' configurePreset switches the board, and the
#  Target dropdown narrows itself to whatever that package actually builds
#  (verified against the CMake File API codemodel directly during the
#  PlatformIO->CMake migration POC).
#
#  Host unit tests (test/test_*/main.cpp, doctest) are a SEPARATE CMake
#  project under test/ (own CMakeLists.txt + CMakePresets.json, host g++,
#  no cross toolchain) - one CMake configure has exactly one compiler, so
#  the native tests cannot share this configure. This is also the template
#  for the next hardware family (ATSAM C): another sibling CMake project
#  with its own toolchain file, not a branch inside this file. No shared
#  "app discovery" module is factored out ahead of that second real
#  instance - see CLAUDE.md's own rule against generalizing before a
#  second target forces the actual shape.
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-avr.cmake")
endif()

project(brio_avr CXX C ASM)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Replace CMake's own per-config defaults (-O3 -DNDEBUG for Release, -g for
# Debug) so the ONLY optimization/debug flags in effect are the ones set
# explicitly below, matching debug_build_flags' old platformio.ini profile
# exactly instead of layering on top of them.
foreach(lang C CXX)
    set(CMAKE_${lang}_FLAGS_RELEASE "" CACHE STRING "" FORCE)
    set(CMAKE_${lang}_FLAGS_DEBUG "" CACHE STRING "" FORCE)
endforeach()

include(cmake/avr-mcus.cmake)

# --- Which AVR package this configure targets - one CMake configure = one
#     package, selected by a configurePreset. avr-mcus.cmake is the
#     reference for valid values.
set(AVR_MCU "avr128db48" CACHE STRING "target AVR MCU, e.g. avr128db48/avr128db28/avr128db32 (see cmake/avr-mcus.cmake)")
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
#     Atmel-ICE when more than one is attached (tools/bench.py does the same
#     via its manifest); default here is board A's probe (tools/bench.py
#     list). Override with -DAVR_PROBE_SERIAL=... for board B or another desk.
set(AVR_PROBE_SERIAL "J42700051207" CACHE STRING "Atmel-ICE serial to flash/debug through (tools/bench.py list)")

# --- The build id every image carries: avrdx/nvm_flash.hpp records it in
#     every map version of a flash NvHeap, where it is a DIAGNOSTIC and
#     nothing else - a stored block is judged by its checksum, never by
#     which build wrote it. It is the NEWEST SOURCE TIMESTAMP, not the time
#     of the link: tools/bench.py flash reprograms without erasing, which on
#     this part only clears bits, so reflashing is safe only while the image
#     is byte-identical to the one already in the chip - a wall-clock id
#     would break that for every app at every relink. An unchanged tree
#     relinks to the same bytes; a real source change gets a new id (and
#     needs the chip erase anyway). Passed to every app's link
#     unconditionally - an image that never names NvHeap never references
#     the symbol, so this costs it nothing, not even a byte.
set(AVR_NVHEAP_BUILD_ID 0)
file(GLOB_RECURSE _nvheap_id_sources CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/../lib/brio/src/*.cpp" "${CMAKE_SOURCE_DIR}/../lib/brio/src/*.hpp"
    "${CMAKE_SOURCE_DIR}/../lib/brio/src/*.c"   "${CMAKE_SOURCE_DIR}/../lib/brio/src/*.h"
    "${CMAKE_SOURCE_DIR}/../lib/brio/src/*.S"
    "${CMAKE_SOURCE_DIR}/src/*.cpp" "${CMAKE_SOURCE_DIR}/src/*.hpp"
    "${CMAKE_SOURCE_DIR}/src/*.c"   "${CMAKE_SOURCE_DIR}/src/*.h"
    "${CMAKE_SOURCE_DIR}/src/*.S"
)
foreach(_f IN LISTS _nvheap_id_sources)
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
        src/glue/ivsel_boot.cpp
        ${A_UNPARSED_ARGUMENTS}
    )

    target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR}/../lib/brio/src)
    target_compile_definitions(${name} PRIVATE ${AVR_MCU_DEFINES})

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

    # No -flto (it splits DWARF line info across partition CUs and avr-gdb
    # then refuses file:line breakpoints). No -fpermissive/
    # -Wno-error=narrowing either (ill-formed code and narrowing conversions
    # must fail, per house rules) - both flags are simply never added here,
    # since this build's flags are set from scratch rather than inherited
    # from a framework default that injected them.
    target_link_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        -Wl,--gc-sections
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--section-start=.text=0x0
        -Wl,--defsym,__flmap_lock=${A_FLMAP_LOCK}
        -Wl,--defsym,__nvheap_build_id=${AVR_NVHEAP_BUILD_ID}
        -lm
    )

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
            -P usb:${AVR_PROBE_SERIAL}
            -U flash:w:$<TARGET_FILE_DIR:${name}>/${name}.hex:i
        DEPENDS ${name}
        VERBATIM
    )
endfunction()

# --- App discovery: every src/apps/<app>.cpp with a main() becomes a
#     target, gated by its own "// build: boards = ..." header comment
#     (grammar: any "// build: <key> = <value>" line in the file; "boards"
#     is a comma-separated allow-list, default {db48} if absent - matching
#     every app that doesn't name the line today; any other key is
#     collected as per-app metadata for tools/bench.py, via the JSON
#     manifest below, EXCEPT "flmap_lock" which also feeds avr_add_app()'s
#     link flags directly).
file(GLOB APP_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/apps/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards db48)
    set(_monitor_speed "")
    set(_flmap_lock 1)

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
            elseif(_key STREQUAL "flmap_lock")
                set(_flmap_lock "${_value}")
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
    set(_entry "\"${_app}\":{\"boards\":[${_boards_json}],\"monitor_speed\":${_speed_json},\"flmap_lock\":${_flmap_lock}}")
    if(_manifest_entries STREQUAL "")
        set(_manifest_entries "${_entry}")
    else()
        set(_manifest_entries "${_manifest_entries},\n    ${_entry}")
    endif()

    if(AVR_BOARD IN_LIST _boards)
        avr_add_app(${_app} FLMAP_LOCK ${_flmap_lock} src/apps/${_app}.cpp)
    endif()
endforeach()

# Package-agnostic (a scan of source comments, not of what THIS configure
# happens to build) - tools/bench.py reads this instead of parsing apps.ini/
# platformio.ini. Regenerated identically by any one configure/preset.
#
# ONE ROSTER PER PROJECT, named after the project. The samc/ project emits
# its own apps_samc.json next to this one, and it MUST be a separate file:
# app names collide across the two projects (blink, console and probe exist
# in both), so a merged roster keyed by app name could not answer "which
# source tree, which toolchain, which flash mechanism". bench.py picks the
# roster from the board's TYPE, which is what already knows the project.
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_avrdx.json"
    "{\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
