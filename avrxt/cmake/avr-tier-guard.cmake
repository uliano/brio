# ============================================================================
#  avr-tier-guard.cmake - run after the link of every image on a board type
#  of AVR_SPLIT_BOARDS (CMakeLists.txt): the image must fit the SPLIT TIER,
#  the smallest chip in view, whose memories may be smaller than the part
#  the board carries. The part's own linker regions guard the part; this
#  guards the tier, so a suite whose group outgrows the smallest chip fails
#  its build here, on the board that stands in for that chip.
#
#      cmake -DELF=<image.elf> -DSIZE=<avr-size> -DFLASH=<bytes> -DRAM=<bytes>
#            -P avr-tier-guard.cmake
#
#  FLASH is what the image stores: .text, .data's initial values and
#  .rodata (on an avrxmega2 part .rodata is in flash, read through the
#  FLMAP window; on an avrxmega3 part it is in flash too, mapped into data
#  space). RAM is the static RAM: .data, .bss and .noinit - the stack is
#  not in it, as in brio fit's census.
# ============================================================================

execute_process(COMMAND "${SIZE}" -A -d "${ELF}"
                OUTPUT_VARIABLE _out RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "error: avr-size failed on ${ELF}")
endif()

set(_flash 0)
set(_ram 0)
string(REPLACE "\n" ";" _lines "${_out}")
foreach(_line IN LISTS _lines)
    if(_line MATCHES "^\\.(text|data|rodata|bss|noinit)[ \t]+([0-9]+)")
        set(_sec "${CMAKE_MATCH_1}")
        set(_n "${CMAKE_MATCH_2}")
        if(_sec STREQUAL "text" OR _sec STREQUAL "data" OR _sec STREQUAL "rodata")
            math(EXPR _flash "${_flash} + ${_n}")
        endif()
        if(_sec STREQUAL "data" OR _sec STREQUAL "bss" OR _sec STREQUAL "noinit")
            math(EXPR _ram "${_ram} + ${_n}")
        endif()
    endif()
endforeach()

get_filename_component(_image "${ELF}" NAME_WE)
if(_flash GREATER FLASH OR _ram GREATER RAM)
    # NOTICE prints the line as it is (an error message is re-wrapped),
    # so brio fit reads the whole of it.
    message(NOTICE "error: ${_image} does not fit the split tier: flash ${_flash} of ${FLASH} B, "
                   "static RAM ${_ram} of ${RAM} B - regroup its letters (// build: groups)")
    message(FATAL_ERROR "the split tier's guard")
endif()
