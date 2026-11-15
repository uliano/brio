# ============================================================================
#  ch32x035/CMakeLists.txt - the CH32X035 build: WCH's CH32X035 and
#  CH32X033 on the QingKe V4C, a SIBLING CMake project (own toolchain file,
#  own presets) like the other families' - one configure = one compiler.
#
#  WHAT THIS FILE HOLDS is the discovery of the apps kept in this tree.
#  How one image is built - the part's definition, ISA and ABI, the flags,
#  the crt, the link, the upload - is cmake/family.cmake, the family's
#  recipe, which this file includes and which an application kept outside
#  the tree gets through ../cmake/brio.cmake: one recipe, so both are
#  built alike.
#
#  One executable target per app: every ch32x035/src/apps/*.cpp with a
#  main() is discovered at configure time (CONFIGURE_DEPENDS) from its own
#  "// build:" header comment, the same grammar as every other project.
#
#  EVERY PART IS THE SMALLEST CHIP. Every part of the series has 62 KB of
#  code flash and 20 KB of SRAM, so a suite that links on the bench part
#  links on all seven - and one that does not fit splits on all seven: a
#  suite with a "// build: groups" line builds as one image per group on
#  every board type of this project (the GROUP axis below).
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-riscv.cmake")
endif()

project(brio_ch32x035 CXX C ASM)

include(cmake/family.cmake)   # the recipe: part, flags, crt, link, ch32x035_add_app()

# --- App discovery: every ch32x035/src/apps/<app>.cpp - plus every
#     ../experiments/*/ch32x035/<app>.cpp - becomes a target, gated by its
#     own "// build: boards = ..." header comment (default: x035f8, the
#     board type of WCH's evaluation board in its QFN20 edition).
#
#     "// build: scheduler = ram" or "= flash" says where the preemptive
#     kernel's scheduler path runs (ch32x035_add_app() in
#     cmake/family.cmake); a file without the line is ram, and any other
#     value stops the configure with the app's name.
#
#     The GROUP axis (design/overview.md, "A suite's image fits the
#     family's smallest chip"): no part of this series is smaller than
#     another (the file header), so CH32X035_SPLIT_BOARDS is EVERY board
#     type of the part table - a suite whose header names groups builds as
#     <app>-1, <app>-2, ... on every part, each compiled with
#     BRIO_TEST_LETTERS naming its group, and a suite with no groups line
#     is one image everywhere.
set(CH32X035_SPLIT_BOARDS "")
foreach(_row IN LISTS CH32X035_PARTS)
    string(REPLACE ":" ";" _fields "${_row}")
    list(GET _fields 3 _btype)
    list(APPEND CH32X035_SPLIT_BOARDS "${_btype}")
endforeach()

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/ch32x035/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards x035f8)
    set(_groups "")
    set(_monitor_speed "")
    set(_scheduler "ram")

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
            elseif(_key STREQUAL "groups")
                string(REPLACE "," ";" _groups_raw "${_value}")
                set(_groups "")
                foreach(_g IN LISTS _groups_raw)
                    string(STRIP "${_g}" _g)
                    list(APPEND _groups "${_g}")
                endforeach()
            elseif(_key STREQUAL "monitor_speed")
                set(_monitor_speed "${_value}")
            elseif(_key STREQUAL "scheduler")
                set(_scheduler "${_value}")
            endif()
        endif()
    endforeach()

    if(NOT _scheduler STREQUAL "ram" AND NOT _scheduler STREQUAL "flash")
        message(FATAL_ERROR "${_app_path}: \"// build: scheduler = ${_scheduler}\" - the scheduler's place is ram or flash")
    endif()

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
    set(_entry "\"${_app}\":{\"boards\":[${_boards_json}],\"groups\":${_groups_json},\"monitor_speed\":${_speed_json},\"scheduler\":\"${_scheduler}\"}")
    if(_manifest_entries STREQUAL "")
        set(_manifest_entries "${_entry}")
    else()
        set(_manifest_entries "${_manifest_entries},\n    ${_entry}")
    endif()

    if(CH32X035_BOARD IN_LIST _boards)
        if(CH32X035_BOARD IN_LIST CH32X035_SPLIT_BOARDS AND NOT _groups STREQUAL "")
            # A line that names any upper-case key partitions those keys as it
            # partitions the rest: its groups carry the '.' marker, so an image
            # whose group names none carries none (util/testbench.hpp,
            # upper_mask_of).
            set(_upper_marker "")
            foreach(_g IN LISTS _groups)
                if(_g MATCHES "[A-Z]")
                    set(_upper_marker ".")
                endif()
            endforeach()

            set(_i 0)
            foreach(_g IN LISTS _groups)
                math(EXPR _i "${_i} + 1")
                ch32x035_add_app(${_app}-${_i} ${_scheduler} ${_app_path})
                target_compile_definitions(${_app}-${_i} PRIVATE "BRIO_TEST_LETTERS=\"${_g}${_upper_marker}\"")
            endforeach()
        else()
            ch32x035_add_app(${_app} ${_scheduler} ${_app_path})
        endif()
    endif()
endforeach()

# This project's app roster for bin/brio (apps_ch32x035.json): a separate
# file because app names collide across the trees. It names the boards
# that split a suite into its group images - every board type here (the
# discovery comment above) - so the verbs can resolve "<app>-<n>" and
# refuse the bare name of a suite with groups.
set(_split_json "")
foreach(_b IN LISTS CH32X035_SPLIT_BOARDS)
    if(_split_json STREQUAL "")
        set(_split_json "\"${_b}\"")
    else()
        set(_split_json "${_split_json},\"${_b}\"")
    endif()
endforeach()
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_ch32x035.json"
    "{\n  \"split_boards\": [${_split_json}],\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
