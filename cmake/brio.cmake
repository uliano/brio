# ============================================================================
#  brio.cmake - the entry point of an application kept OUTSIDE brio's tree.
#
#      cmake_minimum_required(VERSION 3.25)
#      set(BRIO_MCU avr128db48 CACHE STRING "the part this firmware is for")
#      include(/path/to/brio/cmake/brio.cmake)      # before project()
#      project(my_app)
#      brio_add_app(my_app src/main.cpp src/more.cpp)
#
#  It is included BEFORE project(), because the part decides the compiler
#  and a configure has exactly one. It does three things: it finds the
#  part's family; it names that family's toolchain file; and it asks
#  project() to load the family's recipe, <family>/cmake/family.cmake, as
#  its last step, when the compiler is known. The recipe is the one the
#  apps kept in the tree are built with, and it defines brio_add_app():
#  one image per call, with its .hex or .bin, its listing, its map and an
#  <app>-upload target. What follows project() is ordinary CMake - the
#  application's own include directories and definitions go on the target
#  brio_add_app() made.
#
#  WHAT AN APPLICATION SETS, before the include or from a preset:
#    BRIO_MCU    the part, as the family's own presets spell it:
#                avr128db48, samc21j18a, stm32g0b1re, stm32f446re,
#                ch32v006k8, ch32v203c8, ch32x035f8, rp2040, rp2350a.
#                Required. One build directory per part.
#    BRIO_BOARD  optional: one of brio's board types (boards/<type>/),
#                whose board file the application then reads with
#                #include "board.hpp". An application on a board of its
#                own keeps its board file among its sources and leaves
#                this unset.
#  Each family's further choices are its recipe's own cache variables,
#  set the same way (the RP2350's architecture, RP2350_ARCH, is the one
#  this file reads too, because it picks between two compilers).
# ============================================================================

include_guard(GLOBAL)

if(CMAKE_VERSION VERSION_LESS 3.25)
    message(FATAL_ERROR "brio: CMake 3.25 or newer is required (this is ${CMAKE_VERSION})")
endif()

get_filename_component(BRIO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

if(NOT BRIO_MCU)
    message(FATAL_ERROR "brio: BRIO_MCU is not set - name the part before including brio.cmake, "
                        "e.g. set(BRIO_MCU avr128db48 CACHE STRING \"\") or -DBRIO_MCU=avr128db48")
endif()
string(TOLOWER "${BRIO_MCU}" _brio_mcu)

# The part names its family; the family names its toolchain file and the
# variable its recipe reads the part from. A variable set here wins over
# the recipe's cache default of the same name.
if(_brio_mcu MATCHES "^avr")
    set(BRIO_FAMILY avrxt)
    set(_brio_toolchain toolchain-avr.cmake)
    set(AVR_MCU "${_brio_mcu}")
elseif(_brio_mcu MATCHES "^(at)?samc21")
    set(BRIO_FAMILY samc21)
    set(_brio_toolchain toolchain-arm.cmake)
    string(REGEX REPLACE "^at" "" SAMC21_MCU "${_brio_mcu}")
elseif(_brio_mcu MATCHES "^stm32g0")
    set(BRIO_FAMILY stm32g0)
    set(_brio_toolchain toolchain-arm.cmake)
    set(STM32G0_MCU "${_brio_mcu}")
elseif(_brio_mcu MATCHES "^stm32f4")
    set(BRIO_FAMILY stm32f4)
    set(_brio_toolchain toolchain-arm.cmake)
    set(STM32F4_MCU "${_brio_mcu}")
elseif(_brio_mcu MATCHES "^ch32v00")
    set(BRIO_FAMILY ch32v00x)
    set(_brio_toolchain toolchain-riscv.cmake)
    set(CH32V00X_MCU "${_brio_mcu}")
elseif(_brio_mcu MATCHES "^ch32v[23]03")
    set(BRIO_FAMILY ch32vx03)
    set(_brio_toolchain toolchain-riscv.cmake)
    set(CH32VX03_MCU "${_brio_mcu}")
elseif(_brio_mcu MATCHES "^ch32x03")
    set(BRIO_FAMILY ch32x035)
    set(_brio_toolchain toolchain-riscv.cmake)
    set(CH32X035_MCU "${_brio_mcu}")
elseif(_brio_mcu STREQUAL "rp2040")
    set(BRIO_FAMILY rp2040)
    set(_brio_toolchain toolchain-arm.cmake)
elseif(_brio_mcu MATCHES "^rp2350([ab])?$")
    set(BRIO_FAMILY rp2350)
    # The package is the part's letter (rp2350a the QFN-60, rp2350b the
    # QFN-80); the architecture is the family's own variable, and it
    # decides the compiler.
    if(CMAKE_MATCH_1)
        set(RP2350_PACKAGE "${CMAKE_MATCH_1}")
    endif()
    if(RP2350_ARCH STREQUAL "riscv")
        set(_brio_toolchain toolchain-riscv.cmake)
    else()
        set(_brio_toolchain toolchain-arm.cmake)
    endif()
else()
    message(FATAL_ERROR "brio: no family builds for BRIO_MCU='${BRIO_MCU}' (the families: avr*, samc21*, "
                        "stm32g0*, stm32f4*, ch32v00*, ch32v203*/ch32v303*, ch32x03*, rp2040, rp2350a/rp2350b)")
endif()

set(_brio_recipe "${BRIO_ROOT}/${BRIO_FAMILY}/cmake/family.cmake")
if(NOT EXISTS "${_brio_recipe}")
    message(FATAL_ERROR "brio: the ${BRIO_FAMILY} family has no recipe (${_brio_recipe})")
endif()

# A build directory belongs to one family: the compiler cannot change under it.
if(DEFINED CACHE{BRIO_FAMILY_OF_THIS_BUILD} AND NOT "$CACHE{BRIO_FAMILY_OF_THIS_BUILD}" STREQUAL "${BRIO_FAMILY}")
    message(FATAL_ERROR "brio: this build directory was configured for the $CACHE{BRIO_FAMILY_OF_THIS_BUILD} "
                        "family and BRIO_MCU=${BRIO_MCU} is of ${BRIO_FAMILY}: use one build directory per part")
endif()
set(BRIO_FAMILY_OF_THIS_BUILD "${BRIO_FAMILY}" CACHE INTERNAL "the family this build directory was configured for")

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${BRIO_ROOT}/${BRIO_FAMILY}/cmake/${_brio_toolchain}")
endif()

if(BRIO_BOARD)
    set(BRIO_BOARD_DIR "${BRIO_ROOT}/boards/${BRIO_BOARD}")
    if(NOT EXISTS "${BRIO_BOARD_DIR}/board.hpp")
        message(FATAL_ERROR "brio: BRIO_BOARD='${BRIO_BOARD}' has no board file (${BRIO_BOARD_DIR}/board.hpp)")
    endif()
endif()

# The recipe needs the compiler, so it loads when project() has found it.
list(APPEND CMAKE_PROJECT_INCLUDE "${_brio_recipe}")
