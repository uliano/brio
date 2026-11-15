# ============================================================================
#  rp2350/CMakeLists.txt - the RP2350 build: a SIBLING CMake project (own
#  toolchain files, own presets) like the other families' - one configure
#  = one compiler, and here the compiler is a preset axis: the two
#  architectures of the chip are two configures of the same sources
#  (cmake/family.cmake says how).
#
#  WHAT THIS FILE HOLDS is the discovery of the apps kept in this tree.
#  How one image is built - the architecture, the flash geometry and the
#  package, the flags, the crt, the link, the upload - is
#  cmake/family.cmake, the family's recipe, which this file includes and
#  which an application kept outside the tree gets through
#  ../cmake/brio.cmake: one recipe, so both are built alike.
#
#  One executable target per app: every rp2350/src/apps/*.cpp with a
#  main() is discovered at configure time (CONFIGURE_DEPENDS) from its own
#  "// build:" header comment, the same grammar as the other projects.
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-arm.cmake")
endif()

project(brio_rp2350 CXX C ASM)

include(cmake/family.cmake)   # the recipe: architecture, flash, package, flags, crt, link, rp2350_add_app()

# THE BOARD TYPES A CONFIGURE SERVES, and why there are two per board:
# the architecture is a preset axis and the manifest names a physical
# board once per architecture (cli/bench/common.py's weact2350b and
# weact2350b-rv are the same piece of hardware), so an app that runs on
# the board is built for both and the default names both. The package
# picks the board: the WeAct RP2350B is the QFN-80, the WeAct RP2350A
# the QFN-60, both with a 16 MB flash.
if(RP2350_PACKAGE STREQUAL "a")
    set(RP2350_BOARDS weact2350a weact2350a-rv)
    set(RP2350_BOARD weact2350a)
else()
    set(RP2350_BOARDS weact2350b weact2350b-rv)
    set(RP2350_BOARD weact2350b)
endif()
# Its board file, ../boards/<board>/board.hpp: one directory per physical
# board, which both architectures' board types share; the recipe puts it
# on every image's include path.
set(BRIO_BOARD_DIR "${BRIO_ROOT}/boards/${RP2350_BOARD}")

# --- App discovery: every rp2350/src/apps/<app>.cpp - plus every
#     ../experiments/*/rp2350/<app>.cpp - becomes a target, gated by its
#     own "// build: boards = ..." header comment (default: every board
#     type this project serves) against the board types of this configure.
file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/rp2350/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards ${RP2350_BOARDS})
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

    set(_served FALSE)
    foreach(_b IN LISTS _boards)
        if(_b IN_LIST RP2350_BOARDS)
            set(_served TRUE)
        endif()
    endforeach()
    if(_served)
        rp2350_add_app(${_app} ${_app_path})
    endif()
endforeach()

# This project's app roster for bin/brio (apps_rp2350.json): a separate
# file because app names collide across the trees.
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_rp2350.json"
    "{\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
