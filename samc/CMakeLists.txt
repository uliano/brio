# ============================================================================
#  samc/CMakeLists.txt - the SAM C21 build: brio's second hardware target,
#  a SIBLING CMake project (own toolchain file, own presets) exactly as the
#  root CMakeLists.txt's header comment prescribes - the same pattern test/
#  already is, with nothing shared: one CMake configure has exactly one
#  compiler, and no "app discovery" module is factored out ahead of the
#  second real instance proving the shape (this file IS that instance; if a
#  third target ever makes the factoring obvious, it happens then).
#
#  One executable target per app: every samc/src/apps/*.cpp with a main()
#  is discovered at configure time (CONFIGURE_DEPENDS) from its own
#  "// build:" header comment, mirroring the AVR project's grammar. One
#  configure targets exactly ONE chip variant (SAMC_MCU cache var, one
#  value per configurePreset); only the J18A variant has a preset today -
#  the board on the desk - the E/G variants' headers are compile-checked by
#  tools/check_samc.sh instead (no hardware to verify them on).
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

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-arm.cmake")
endif()

project(brio_samc21 CXX C ASM)

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
#     header's basename (third_party/samc21-dfp/include/<SAMC_MCU>.h); the
#     device-select define is derived from it (samc21j18a -> __SAMC21J18A__).
set(SAMC_MCU "samc21j18a" CACHE STRING "target SAM C21 variant, e.g. samc21j18a (see third_party/samc21-dfp/include)")
string(TOUPPER "${SAMC_MCU}" _mcu_upper)
set(SAMC_MCU_DEFINE "__${_mcu_upper}__")

# --- Flags shared by every language + the link step.
set(BRIO_COMMON_FLAGS
    -mcpu=cortex-m0plus
    -mthumb
    -mfloat-abi=soft
    -D${SAMC_MCU_DEFINE}
    -Wall -Wextra -Werror
    -ffunction-sections -fdata-sections
)

find_program(ARM_OBJCOPY arm-none-eabi-objcopy PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(ARM_SIZE arm-none-eabi-size PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)
find_program(ARM_OBJDUMP arm-none-eabi-objdump PATHS "${ARM_TOOLCHAIN_DIR}/bin" NO_DEFAULT_PATH)

# --- Flash/debug over SWD: OpenOCD driving the Atmel-ICE as a CMSIS-DAP
#     probe (the at91samd flash driver auto-probes the geometry from the
#     DSU DID). `adapter serial` disambiguates which Atmel-ICE when more
#     than one is attached - same concern as the AVR project's
#     AVR_PROBE_SERIAL; default is the SAM board's probe on this desk.
set(SAMC_OPENOCD "/sw/oss-cad-suite/bin/openocd" CACHE FILEPATH "OpenOCD with CMSIS-DAP + at91samd support")
set(SAMC_PROBE_SERIAL "J42700049508" CACHE STRING "Atmel-ICE serial to flash/debug through")

# --- The build id every image carries: samc/nvm_flash.hpp records it in
#     every map version of a flash NvHeap, where it is a DIAGNOSTIC and
#     nothing else - a stored block is judged by its checksum, never by
#     which build wrote it. It is the NEWEST SOURCE TIMESTAMP and not the
#     time of the link, for the same reason as on the AVR side: an
#     unchanged tree must relink to the same bytes, or every reflash of an
#     unchanged app would look like a different image. Passed to every
#     app's link unconditionally - an image that never names NvHeap never
#     references the symbol, so this costs it nothing, not even a byte.
set(SAMC_NVHEAP_BUILD_ID 0)
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
    if(_ts GREATER SAMC_NVHEAP_BUILD_ID)
        set(SAMC_NVHEAP_BUILD_ID "${_ts}")
    endif()
endforeach()

# --- One app -> one executable target, one .bin/.hex, one -upload target.
function(sam_add_app name)
    add_executable(${name}
        src/glue/startup_samc21.cpp
        ${ARGN}
    )

    target_include_directories(${name} PRIVATE
        ${CMAKE_SOURCE_DIR}/../brio
        ${CMAKE_SOURCE_DIR}/../third_party/samc21-dfp/include
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

    # -nostartfiles: startup_samc21.cpp IS the crt (vector table + reset
    # path); newlib-nano still provides memcpy/__libc_init_array etc. No
    # nosys syscall stubs on purpose: brio allocates nothing and calls no
    # syscalls, so anything dragging _sbrk/_write in fails the link loudly
    # instead of failing silently at run time. No -flto (same DWARF/gdb
    # rationale as the AVR build).
    target_link_options(${name} PRIVATE
        ${BRIO_COMMON_FLAGS}
        $<$<CONFIG:Release>:-Os -g>
        --specs=nano.specs
        -nostartfiles
        -T${CMAKE_SOURCE_DIR}/ld/${SAMC_MCU}.ld
        -Wl,--gc-sections
        -Wl,-Map,$<TARGET_FILE_DIR:${name}>/firmware-${name}.map
        -Wl,--defsym,__nvheap_build_id=${SAMC_NVHEAP_BUILD_ID}
    )
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
        ${CMAKE_SOURCE_DIR}/ld/${SAMC_MCU}.ld)

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

    add_custom_target(${name}-upload
        COMMAND ${SAMC_OPENOCD}
            -f interface/cmsis-dap.cfg
            -c "adapter serial ${SAMC_PROBE_SERIAL}"
            -f target/at91samdXX.cfg
            -c "program $<TARGET_FILE:${name}> verify reset exit"
        DEPENDS ${name}
        VERBATIM
    )
endfunction()

# --- App discovery: every samc/src/apps/<app>.cpp - plus every
#     ../experiments/*/samc/<app>.cpp, the per-experiment app slot
#     (mirroring the AVR project) - becomes a target, gated
#     by its own "// build: boards = ..." header comment (default: c21j -
#     the one board type that exists; the grammar is the AVR project's, the
#     board names are this family's). monitor_speed is carried through to
#     the roster below so tools/bench.py opens a console at the speed the
#     app itself declares; other keys are tolerated as metadata.
set(SAMC_BOARD "c21j")

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/samc/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards c21j)
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

    if(SAMC_BOARD IN_LIST _boards)
        sam_add_app(${_app} ${_app_path})
    endif()
endforeach()

# This project's app roster for tools/bench.py - the twin of the AVR
# project's apps_avrdx.json, and a SEPARATE FILE on purpose: app names
# collide across the two source trees (blink, console, probe exist in
# both), so only a per-project roster can say which tree, which toolchain
# and which flash mechanism an app name means. Variant-agnostic, exactly
# like the AVR one: it is a scan of source comments, not of what this
# configure happens to build.
#
# No flmap_lock field: that key is an AVR-Dx build invariant (FLMAPLOCK)
# with no SAM analog, and bench.py reads it only for db* boards.
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_samc.json"
    "{\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
