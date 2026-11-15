# ============================================================================
#  ch32vx03/CMakeLists.txt - the CH32V203 build: brio's second WCH target
#  and its second RISC-V one, a SIBLING CMake project (own toolchain file,
#  own presets) like the other families' - one configure = one compiler.
#  The CH32V303 is built here too, as the third device class of one
#  reference manual (cmake/ch32vx03-parts.cmake).
#
#  WHAT THIS FILE HOLDS is the discovery of the apps kept in this tree.
#  How one image is built - the part's definition, ISA and ABI, the flags,
#  the crt, the link, the upload - is cmake/family.cmake, the family's
#  recipe, which this file includes and which an application kept outside
#  the tree gets through ../cmake/brio.cmake: one recipe, so both are
#  built alike.
#
#  One executable target per app: every ch32vx03/src/apps/*.cpp with a
#  main() is discovered at configure time (CONFIGURE_DEPENDS) from its own
#  "// build:" header comment, the same grammar as every other project.
#
#  ONE APP IS NOT ALWAYS ONE IMAGE. A suite whose image does not fit the
#  family's smallest parts declares its GROUPS of letters in that same
#  header ("// build: groups = abc,de"), and those parts - the 32 KB tier
#  here - build it as one image per group, <app>-1, <app>-2, ..., each
#  carrying its group's letters and the prose that judges them; every
#  other part builds the same source as ONE image with every letter. What
#  bounds a group is the smallest chip's memory, and the linker script is
#  the guard; a larger part whose whole image comes too near its own
#  memory names groups of its own (a "groups.<board type>" line) (design/overview.md, "A suite's image fits the family's
#  smallest chip"; the mechanism is util/testbench.hpp and the discovery
#  loop below).
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-riscv.cmake")
endif()

project(brio_ch32vx03 CXX C ASM)

include(cmake/family.cmake)   # the recipe: part, flags, crt, link, ch32vx03_add_app()

# --- App discovery: every ch32vx03/src/apps/<app>.cpp - plus every
#     ../experiments/*/ch32vx03/<app>.cpp - becomes a target, gated by its
#     own "// build: boards = ..." header comment (default: the board type
#     of the part this configure targets).
#
#     The GROUP axis (design/overview.md, "A suite's image fits the
#     family's smallest chip"): a suite whose whole image does not fit the
#     family's 32 KB tier declares its groups of letters in the same
#     header ("// build: groups = abc,de"). On a board of
#     CH32VX03_SPLIT_BOARDS the app then builds as one image PER GROUP -
#     <app>-1, <app>-2, ... - each compiled with BRIO_TEST_LETTERS set to
#     its group's keys, which util/testbench.hpp reads to register those
#     letters alone; on every other board the app is ONE image carrying
#     every letter, and a groups line changes nothing there - unless the
#     app names that board type's own groups ("// build: groups.v203c8 =
#     rmptde,ui"), for a whole image too near a larger part's memory:
#     that line splits the app on that board type alone, and on a split
#     board too it wins over the plain line. The roster names the split
#     boards and each app's groups, plain and per board type, so bin/brio
#     can resolve "<app>-<n>" and refuse the bare name where it is not an
#     image.
#
#     WHICH BOARDS SPLIT IS READ OFF THE PART TABLE, not listed a second
#     time: every part of 32 KB or less - the C6, the F6, the G6 and the
#     K6. A part added to cmake/ch32vx03-parts.cmake is placed by the
#     same memories that give it its linker script, and a part smaller
#     still would join that tier rather than replace it.
set(CH32VX03_SPLIT_FLASH_KB 32)
set(CH32VX03_SPLIT_BOARDS "")
foreach(_row IN LISTS CH32VX03_PARTS)
    string(REGEX REPLACE "=.*$" "" _part "${_row}")
    ch32vx03_part_facts("${_part}" _split_define _split_flash _split_ram _split_board
        _split_arch _split_abi)
    if(_split_flash LESS_EQUAL CH32VX03_SPLIT_FLASH_KB)
        list(APPEND CH32VX03_SPLIT_BOARDS "${_split_board}")
    endif()
endforeach()

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/ch32vx03/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards v203c8)
    set(_groups "")
    set(_board_groups_json "")
    set(_own_groups "")
    set(_own_groups_set FALSE)
    set(_monitor_speed "")

    # A key may carry a board type after a dot: "groups.v203c8 = ..." is
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
                    if(_qual STREQUAL CH32VX03_BOARD)
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
    elseif(CH32VX03_BOARD IN_LIST CH32VX03_SPLIT_BOARDS)
        set(_split "${_groups}")
    else()
        set(_split "")
    endif()
    if(CH32VX03_BOARD IN_LIST _boards)
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
                ch32vx03_add_app(${_app}-${_i} ${_app_path})
                target_compile_definitions(${_app}-${_i} PRIVATE "BRIO_TEST_LETTERS=\"${_g}${_upper_marker}\"")
            endforeach()
        else()
            ch32vx03_add_app(${_app} ${_app_path})
        endif()
    endif()
endforeach()

# This project's app roster for bin/brio (apps_ch32vx03.json): a separate
# file because app names collide across the trees. It names the boards
# that split a suite into its group images, so the verbs can resolve
# "<app>-<n>" there and refuse the bare name.
set(_split_json "")
foreach(_b IN LISTS CH32VX03_SPLIT_BOARDS)
    if(_split_json STREQUAL "")
        set(_split_json "\"${_b}\"")
    else()
        set(_split_json "${_split_json},\"${_b}\"")
    endif()
endforeach()
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_ch32vx03.json"
    "{\n  \"split_boards\": [${_split_json}],\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
