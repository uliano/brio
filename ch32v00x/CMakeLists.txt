# ============================================================================
#  ch32v00x/CMakeLists.txt - the CH32V00x build: brio's fourth hardware
#  target and its first RISC-V one, a SIBLING CMake project (own
#  toolchain file, own presets) like the other families' - one configure
#  = one compiler.
#
#  One executable target per app: every ch32v00x/src/apps/*.cpp with a
#  main() is discovered at configure time (CONFIGURE_DEPENDS) from its
#  own "// build:" header comment, the same grammar as the other three
#  projects.
#
#  WHAT THIS FILE HOLDS is the discovery of the apps kept in this tree.
#  How one image is built - the part's definitions and ISA, the flags,
#  the crt, the link, the upload - is cmake/family.cmake, the family's
#  recipe, which this file includes and which an application kept outside
#  the tree gets through ../cmake/brio.cmake: one recipe, so both are
#  built alike.
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-riscv.cmake")
endif()

project(brio_ch32v00x CXX C ASM)

include(cmake/family.cmake)   # the recipe: flags, crt, link, ch32_add_app()

# The board file of the board type this configure builds for,
# ../boards/<type>/board.hpp: the recipe puts the directory on every
# image's include path when it exists.
set(BRIO_BOARD_DIR "${BRIO_ROOT}/boards/${CH32V00X_BOARD}")

# --- App discovery: every ch32v00x/src/apps/<app>.cpp - plus every
#     ../experiments/*/ch32v00x/<app>.cpp - becomes a target, gated by
#     its own "// build: boards = ..." header comment (default: v006k8,
#     the bench board).
#
#     The GROUP axis (design/overview.md, "A suite's image fits the
#     family's smallest chip"): a suite whose whole image does not fit
#     the CH32V003's 15 KB declares its groups of letters in the same
#     header ("// build: groups = abg,cdf,e"). On a board of
#     CH32V00X_SPLIT_BOARDS the app then builds as one image PER GROUP -
#     <app>-1, <app>-2, ... - each compiled with BRIO_TEST_LETTERS set to
#     its group's keys, which util/testbench.hpp reads to register those
#     letters alone; on every other board the app is ONE image carrying
#     every letter, and a groups line changes nothing there - unless the
#     app names that board type's own groups ("// build: groups.v006k8 =
#     rtmpde,uw"), for a whole image too near the CH32V006's 40 KB or
#     8 KB: that line splits the app on that board type alone, and on a
#     split board too it wins over the plain line. The roster names the
#     split boards and each app's groups, plain and per board type, so
#     bin/brio can resolve "<app>-<n>" and refuse the bare name where it
#     is not an image.
set(CH32V00X_SPLIT_BOARDS v003f4)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/ch32v00x/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards v006k8)
    set(_groups "")
    set(_board_groups_json "")
    set(_own_groups "")
    set(_own_groups_set FALSE)
    set(_monitor_speed "")

    # A key may carry a board type after a dot: "groups.v006k8 = ..." is
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
                    if(_qual STREQUAL CH32V00X_BOARD)
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
    elseif(CH32V00X_BOARD IN_LIST CH32V00X_SPLIT_BOARDS)
        set(_split "${_groups}")
    else()
        set(_split "")
    endif()
    if(CH32V00X_BOARD IN_LIST _boards)
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
                ch32_add_app(${_app}-${_i} ${_app_path})
                target_compile_definitions(${_app}-${_i} PRIVATE "BRIO_TEST_LETTERS=\"${_g}${_upper_marker}\"")
            endforeach()
        else()
            ch32_add_app(${_app} ${_app_path})
        endif()
    endif()
endforeach()

# This project's app roster for bin/brio (apps_ch32v00x.json): a
# separate file because app names collide across the trees. It names
# the boards that split a suite into its group images, so the verbs can
# resolve "<app>-<n>" there and refuse the bare name.
set(_split_json "")
foreach(_b IN LISTS CH32V00X_SPLIT_BOARDS)
    if(_split_json STREQUAL "")
        set(_split_json "\"${_b}\"")
    else()
        set(_split_json "${_split_json},\"${_b}\"")
    endif()
endforeach()
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_ch32v00x.json"
    "{\n  \"split_boards\": [${_split_json}],\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
