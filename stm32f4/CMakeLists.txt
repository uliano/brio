# ============================================================================
#  stm32f4/CMakeLists.txt - the STM32F4 build: brio's sixth hardware target
#  and its first ARMv7-M one, a SIBLING CMake project (own toolchain file,
#  own presets) like the other families' - one configure = one compiler.
#
#  WHAT THIS FILE HOLDS is the discovery of the apps kept in this tree.
#  How one image is built - the flags, the crt, the link, the upload - is
#  cmake/family.cmake, the family's recipe, which this file includes and
#  which an application kept outside the tree gets through
#  ../cmake/brio.cmake: one recipe, so both are built alike.
#
#  One executable target per app: every stm32f4/src/apps/*.cpp with a
#  main() is discovered at configure time (CONFIGURE_DEPENDS) from its own
#  "// build:" header comment, the same grammar as the other projects. One
#  configure targets exactly ONE part (STM32F4_MCU cache var, one value per
#  configurePreset): the F429ZI (the STM32F429I-DISC1), the F446RE (the
#  Nucleo-F446RE), the F411CE (a WeAct black pill) and the F469NI (the
#  32F469IDISCOVERY) have presets; every other header of the pack is
#  compile-checked by brio check stm32f4. What
#  the part decides - the device define, the crt, the linker script, the
#  board type - is cmake/stm32f4-parts.cmake's table.
# ============================================================================

cmake_minimum_required(VERSION 3.21)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-arm.cmake")
endif()

project(brio_stm32f4 CXX C ASM)

include(cmake/family.cmake)   # the recipe: flags, crt, link, stm32_add_app()

# --- App discovery: every stm32f4/src/apps/<app>.cpp - plus every
#     ../experiments/*/stm32f4/<app>.cpp - becomes a target, gated by its
#     own "// build: boards = ..." header comment (default: f429zi, the
#     bench chip; an app that also runs on the other boards lists them).
file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/apps/*.cpp"
    "${CMAKE_SOURCE_DIR}/../experiments/*/stm32f4/*.cpp")

set(_manifest_entries "")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    set(_boards f429zi)
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

    if(STM32F4_BOARD IN_LIST _boards)
        stm32_add_app(${_app} ${_app_path})
    endif()
endforeach()

# This project's app roster for bin/brio (apps_stm32f4.json): a
# separate file because app names collide across the trees.
file(WRITE "${CMAKE_SOURCE_DIR}/../build-cmake/apps_stm32f4.json"
    "{\n  \"apps\": {\n    ${_manifest_entries}\n  }\n}\n")
