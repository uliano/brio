# ============================================================================
#  rp2040/CMakeLists.txt - the RP2040 build: brio's fifth hardware target, a
#  SIBLING CMake project (own toolchain file, own presets) like the other
#  families' - one configure = one compiler.
#
#  WHAT THIS FILE HOLDS is the discovery of the apps kept in this tree.
#  How one image is built - the flash geometry and its boot stage, the
#  flags, the crt, the link, the upload - is cmake/family.cmake, the
#  family's recipe, which this file includes and which an application kept
#  outside the tree gets through ../cmake/brio.cmake: one recipe, so both
#  are built alike.
#
#  One executable target per app: every rp2040/src/apps/*.cpp with a
#  main() is discovered at configure time (CONFIGURE_DEPENDS) from its own
#  "// build:" header comment, the same grammar as the other projects.
#
#  A preset targets a FLASH, not a part, and serves every board type with
#  that flash geometry and that boot stage (RP2040_BOARDS, which the recipe
#  derives from them): an app is built here when its "// build: boards ="
#  line names any of them (default: pico, the reference board). The Pico,
#  the WeAct board and the Pico W share the rp2040-* presets.
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-arm.cmake")
endif()

project(brio_rp2040 CXX C ASM)

include(cmake/family.cmake)   # the recipe: flash, stage, flags, crt, link, rp2040_add_app()

# --- App discovery: every rp2040/src/apps/<app>.cpp - plus every
#     ../experiments/*/rp2040/<app>.cpp - becomes a target, gated by its
#     own "// build: boards = ..." header comment (default: pico, the
#     reference board) against the board types this configure serves.
file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/rp2040/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards pico)
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
        if(_b IN_LIST RP2040_BOARDS)
            set(_served TRUE)
        endif()
    endforeach()
    if(_served)
        rp2040_add_app(${_app} ${_app_path})
    endif()
endforeach()

# This project's app roster for bin/brio (apps_rp2040.json): a separate
# file because app names collide across the trees.
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_rp2040.json"
    "{\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
