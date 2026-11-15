# ============================================================================
#  CMakeLists.txt - the AVR build.
#
#  One executable target per (app x board type): every src/apps/*.cpp
#  with a main() is discovered automatically at configure time
#  (CONFIGURE_DEPENDS), gated by its own "// build: boards = ..." header
#  comment (default: db48 only, matching every app that doesn't name the
#  line explicitly). A CMake configure targets exactly ONE package
#  (AVR_MCU cache var, one value per configurePreset in CMakePresets.json),
#  so there is no target per physical board: switching CMake Tools'
#  configurePreset switches the package, and the Target dropdown narrows
#  itself to whatever that package actually builds.
#
#  Host unit tests (test/test_*/main.cpp, doctest) are a SEPARATE CMake
#  project under test/ (own CMakeLists.txt + CMakePresets.json, host g++,
#  no cross toolchain) - one CMake configure has exactly one compiler, so
#  the native tests cannot share this configure. Every other hardware
#  family is a sibling CMake project too, with its own toolchain file,
#  not a branch inside this file.
#
#  WHAT THIS FILE HOLDS is the discovery of the apps kept in this tree.
#  How one image is built - the flags, the glue, the link, the upload -
#  is cmake/family.cmake, the family's recipe, which this file includes
#  and which an application kept outside the tree gets through
#  ../cmake/brio.cmake: one recipe, so both are built alike.
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-avr.cmake")
endif()

project(brio_avr CXX C ASM)

include(cmake/family.cmake)   # the recipe: flags, glue, link, avr_add_app()

# The board file of the board type this configure builds for,
# ../boards/<type>/board.hpp: the recipe puts the directory on every
# image's include path when it exists (the packages with no board have
# none).
set(BRIO_BOARD_DIR "${BRIO_ROOT}/boards/${AVR_BOARD}")

# --- App discovery: every src/apps/<app>.cpp - plus every
#     ../experiments/*/avrxt/<app>.cpp, the per-experiment app slot (an
#     experiment is a self-contained dir holding both architectures'
#     halves and their shared protocol header) - with a main() becomes a
#     target, gated by its own "// build: boards = ..." header comment
#     (grammar: any "// build: <key> = <value>" line in the file; "boards"
#     is a comma-separated allow-list, default {db48} if absent - matching
#     every app that doesn't name the line today; any other key is
#     collected as per-app metadata for bin/brio, via the JSON
#     manifest below, EXCEPT "flmap_lock" which also feeds avr_add_app()'s
#     link flags directly).
#
#     The GROUP axis (design/overview.md, "A suite's image fits the
#     family's smallest chip"): a suite whose whole image does not fit
#     the SPLIT TIER declares its groups of letters in the same header
#     ("// build: groups = 123456789norsdt,cmliwekx"). On a board type of
#     AVR_SPLIT_BOARDS the app then builds as one image PER GROUP -
#     <app>-1, <app>-2, ... - each compiled with BRIO_TEST_LETTERS set to
#     its group's keys, which util/testbench.hpp reads to register those
#     letters alone; on every other board type the app is ONE image
#     carrying every letter - unless the app names that board type's own
#     groups ("// build: groups.da48 = ..."), which split it on that board
#     type alone, and on a split board type too win over the plain line.
#     The roster names the split board types and each app's groups, so
#     bin/brio can resolve "<app>-<n>" and refuse the bare name where it
#     is not an image.
#
#     THE SPLIT TIER is the smallest chip in view, not the smallest part a
#     preset builds: the ATmega4809's 48 KB of flash and 6 KB of SRAM,
#     below the AVR64EA48's 64 KB and 6 KB. A board type is listed in
#     AVR_SPLIT_BOARDS when its part is the smallest the project builds
#     (the AVR64EA48's ea48 now; the 128 KB DA and DB parts hold every
#     suite whole), and every image built there is held to the tier after
#     its link by cmake/avr-tier-guard.cmake - over the tier's flash or
#     RAM, the build fails, though the part itself would hold the image.
#     The configure also writes the tier into the build directory
#     (tier.json), where brio fit reads it and measures each image's
#     margins against it.
set(AVR_SPLIT_BOARDS ea48)
set(AVR_SPLIT_TIER_FLASH 49152)   # the ATmega4809: 48 KB of flash
set(AVR_SPLIT_TIER_RAM 6144)      # and 6 KB of SRAM
if(AVR_BOARD IN_LIST AVR_SPLIT_BOARDS)
    file(WRITE "${CMAKE_BINARY_DIR}/tier.json"
        "{\"flash\": ${AVR_SPLIT_TIER_FLASH}, \"ram\": ${AVR_SPLIT_TIER_RAM}, \"chip\": \"atmega4809\"}\n")
else()
    file(REMOVE "${CMAKE_BINARY_DIR}/tier.json")
endif()

# One image of an app on this configure: the target, and on a split board
# type the tier's guard after its link.
function(avr_add_image name flmap_lock source)
    avr_add_app(${name} FLMAP_LOCK ${flmap_lock} ${source})
    if(AVR_BOARD IN_LIST AVR_SPLIT_BOARDS)
        add_custom_command(TARGET ${name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -DELF=$<TARGET_FILE:${name}> -DSIZE=${AVR_SIZE}
                -DFLASH=${AVR_SPLIT_TIER_FLASH} -DRAM=${AVR_SPLIT_TIER_RAM}
                -P ${CMAKE_SOURCE_DIR}/cmake/avr-tier-guard.cmake
            COMMENT "${name}: the split tier's flash and RAM"
            VERBATIM
        )
        set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS
            ${CMAKE_SOURCE_DIR}/cmake/avr-tier-guard.cmake)
    endif()
endfunction()

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/avrxt/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards db48)
    set(_groups "")
    set(_board_groups_json "")
    set(_own_groups "")
    set(_own_groups_set FALSE)
    set(_monitor_speed "")
    set(_flmap_lock 1)

    # A key may carry a board type after a dot: "groups.da48 = ..." is
    # the groups line of that board type alone.
    file(STRINGS "${_app_path}" _build_lines REGEX "^// build:")
    foreach(_line IN LISTS _build_lines)
        if(_line MATCHES "^// build:[ \t]*([A-Za-z_]+)(\\.([A-Za-z0-9_-]+))?[ \t]*=[ \t]*(.+)$")
            set(_key "${CMAKE_MATCH_1}")
            set(_qual "${CMAKE_MATCH_3}")
            string(STRIP "${CMAKE_MATCH_4}" _value)
            if(_key STREQUAL "boards" AND _qual STREQUAL "")
                string(REPLACE "," ";" _boards_raw "${_value}")
                set(_boards "")
                foreach(_b IN LISTS _boards_raw)
                    string(STRIP "${_b}" _b)
                    list(APPEND _boards "${_b}")
                endforeach()
            elseif(_key STREQUAL "groups")
                string(REPLACE "," ";" _groups_raw "${_value}")
                set(_line_groups "")
                set(_line_json "")
                foreach(_g IN LISTS _groups_raw)
                    string(STRIP "${_g}" _g)
                    list(APPEND _line_groups "${_g}")
                    if(_line_json STREQUAL "")
                        set(_line_json "\"${_g}\"")
                    else()
                        set(_line_json "${_line_json},\"${_g}\"")
                    endif()
                endforeach()
                if(_qual STREQUAL "")
                    set(_groups "${_line_groups}")
                else()
                    if(_board_groups_json STREQUAL "")
                        set(_board_groups_json "\"${_qual}\":[${_line_json}]")
                    else()
                        set(_board_groups_json "${_board_groups_json},\"${_qual}\":[${_line_json}]")
                    endif()
                    if(_qual STREQUAL AVR_BOARD)
                        set(_own_groups "${_line_groups}")
                        set(_own_groups_set TRUE)
                    endif()
                endif()
            elseif(_key STREQUAL "monitor_speed" AND _qual STREQUAL "")
                set(_monitor_speed "${_value}")
            elseif(_key STREQUAL "flmap_lock" AND _qual STREQUAL "")
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
    if(_groups STREQUAL "")
        set(_groups_json "null")
    else()
        set(_groups_json "")
        foreach(_g IN LISTS _groups)
            if(_groups_json STREQUAL "")
                set(_groups_json "\"${_g}\"")
            else()
                set(_groups_json "${_groups_json},\"${_g}\"")
            endif()
        endforeach()
        set(_groups_json "[${_groups_json}]")
    endif()
    set(_entry "\"${_app}\":{\"boards\":[${_boards_json}],\"groups\":${_groups_json},\"board_groups\":{${_board_groups_json}},\"monitor_speed\":${_speed_json},\"flmap_lock\":${_flmap_lock}}")
    if(_manifest_entries STREQUAL "")
        set(_manifest_entries "${_entry}")
    else()
        set(_manifest_entries "${_manifest_entries},\n    ${_entry}")
    endif()

    # The images THIS configure's board type builds the app as: the
    # groups its own line names, else the plain line's on a board type
    # of the split tier, else one image carrying every letter.
    if(_own_groups_set)
        set(_split "${_own_groups}")
    elseif(AVR_BOARD IN_LIST AVR_SPLIT_BOARDS)
        set(_split "${_groups}")
    else()
        set(_split "")
    endif()
    if(AVR_BOARD IN_LIST _boards)
        if(NOT _split STREQUAL "")
            # A line that names any upper-case key partitions those keys as it
            # partitions the rest: its groups carry the '.' marker, so an image
            # whose group names none carries none (util/testbench.hpp,
            # upper_mask_of).
            set(_upper_marker "")
            foreach(_g IN LISTS _split)
                if(_g MATCHES "[A-Z]")
                    set(_upper_marker ".")
                endif()
            endforeach()

            set(_i 0)
            foreach(_g IN LISTS _split)
                math(EXPR _i "${_i} + 1")
                avr_add_image(${_app}-${_i} ${_flmap_lock} ${_app_path})
                target_compile_definitions(${_app}-${_i} PRIVATE "BRIO_TEST_LETTERS=\"${_g}${_upper_marker}\"")
            endforeach()
        else()
            avr_add_image(${_app} ${_flmap_lock} ${_app_path})
        endif()
    endif()
endforeach()

# Package-agnostic (a scan of source comments, not of what THIS configure
# happens to build) - bin/brio reads it. Regenerated identically by
# any one configure/preset.
#
# ONE ROSTER PER PROJECT, named after the project. The samc21/ project emits
# its own apps_samc21.json next to this one, and it MUST be a separate file:
# app names collide across the two projects (blink, console and probe exist
# in both), so a merged roster keyed by app name could not answer "which
# source tree, which toolchain, which flash mechanism". bench.py picks the
# roster from the board's TYPE, which is what already knows the project.
# It names the board types that split a suite into its group images, so
# the verbs can resolve "<app>-<n>" there and refuse the bare name.
set(_split_json "")
foreach(_b IN LISTS AVR_SPLIT_BOARDS)
    if(_split_json STREQUAL "")
        set(_split_json "\"${_b}\"")
    else()
        set(_split_json "${_split_json},\"${_b}\"")
    endif()
endforeach()
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_avrxt.json"
    "{\n  \"split_boards\": [${_split_json}],\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
