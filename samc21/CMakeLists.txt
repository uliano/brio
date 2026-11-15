# ============================================================================
#  samc21/CMakeLists.txt - the SAM C21 build: a SIBLING CMake project (own
#  toolchain file, own presets) like the other families' - one CMake
#  configure has exactly one compiler.
#
#  WHAT THIS FILE HOLDS is the discovery of the apps kept in this tree.
#  How one image is built - the flags, the crt, the link, the upload - is
#  cmake/family.cmake, the family's recipe, which this file includes and
#  which an application kept outside the tree gets through
#  ../cmake/brio.cmake: one recipe, so both are built alike.
#
#  One executable target per app: every samc21/src/apps/*.cpp with a main()
#  is discovered at configure time (CONFIGURE_DEPENDS) from its own
#  "// build:" header comment, mirroring the AVR project's grammar. One
#  configure targets exactly ONE chip variant (SAMC21_MCU cache var, one
#  value per configurePreset); only the J18A variant has a preset today -
#  the board on the desk - the E/G variants' headers are compile-checked by
#  brio check samc21 instead (no hardware to verify them on).
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-arm.cmake")
endif()

project(brio_samc21 CXX C ASM)

include(cmake/family.cmake)   # the recipe: flags, crt, link, sam_add_app()

# --- App discovery: every samc21/src/apps/<app>.cpp - plus every
#     ../experiments/*/samc21/<app>.cpp, the per-experiment app slot
#     (mirroring the AVR project) - becomes a target, gated
#     by its own "// build: boards = ..." header comment (default: c21j -
#     the one board type that exists; the grammar is the AVR project's, the
#     board names are this family's). monitor_speed is carried through to
#     the roster below so bin/brio opens a console at the speed the
#     app itself declares; other keys are tolerated as metadata.
#
#     The GROUP axis (design/overview.md, "A suite's image fits the
#     family's smallest chip"): a suite whose whole image does not fit
#     declares its groups of letters in the same header ("// build:
#     groups = rmptuei,d"), and the app builds as one image PER GROUP -
#     <app>-1, <app>-2, ... - each compiled with BRIO_TEST_LETTERS set to
#     its group's keys, which util/testbench.hpp reads to register those
#     letters alone. The one board type is the split tier: the three
#     SAM C21 18A parts share its 256 KB of flash and 32 KB of RAM. The
#     roster names the split boards and each app's groups, so bin/brio
#     can resolve "<app>-<n>" and refuse the bare name.
set(SAMC21_BOARD "c21j")
set(SAMC21_SPLIT_BOARDS c21j)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/samc21/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards c21j)
    set(_groups "")
    set(_board_groups_json "")
    set(_own_groups "")
    set(_own_groups_set FALSE)
    set(_monitor_speed "")

    # A key may carry a board type after a dot: "groups.c21j = ..." is
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
                    if(_qual STREQUAL SAMC21_BOARD)
                        set(_own_groups "${_line_groups}")
                        set(_own_groups_set TRUE)
                    endif()
                endif()
            elseif(_key STREQUAL "monitor_speed" AND _qual STREQUAL "")
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
    set(_entry "\"${_app}\":{\"boards\":[${_boards_json}],\"groups\":${_groups_json},\"board_groups\":{${_board_groups_json}},\"monitor_speed\":${_speed_json}}")
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
    elseif(SAMC21_BOARD IN_LIST SAMC21_SPLIT_BOARDS)
        set(_split "${_groups}")
    else()
        set(_split "")
    endif()
    if(SAMC21_BOARD IN_LIST _boards)
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
                sam_add_app(${_app}-${_i} ${_app_path})
                target_compile_definitions(${_app}-${_i} PRIVATE "BRIO_TEST_LETTERS=\"${_g}${_upper_marker}\"")
            endforeach()
        else()
            sam_add_app(${_app} ${_app_path})
        endif()
    endif()
endforeach()

# This project's app roster for bin/brio - the twin of the AVR
# project's apps_avrxt.json, and a SEPARATE FILE on purpose: app names
# collide across the two source trees (blink, console, probe exist in
# both), so only a per-project roster can say which tree, which toolchain
# and which flash mechanism an app name means. Variant-agnostic, exactly
# like the AVR one: it is a scan of source comments, not of what this
# configure happens to build.
#
# No flmap_lock field: that key is an AVR-Dx build invariant (FLMAPLOCK)
# with no SAM analog, and bench.py reads it only for db* boards.
set(_split_json "")
foreach(_b IN LISTS SAMC21_SPLIT_BOARDS)
    if(_split_json STREQUAL "")
        set(_split_json "\"${_b}\"")
    else()
        set(_split_json "${_split_json},\"${_b}\"")
    endif()
endforeach()
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_samc21.json"
    "{\n  \"split_boards\": [${_split_json}],\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
