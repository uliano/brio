# ============================================================================
#  stm32f4-parts.cmake - the part table of the stm32f4 project: what a full
#  part number (STM32F4_MCU, lower case) decides.
#
#  Three things follow from the part, and on this family they do NOT all
#  follow from the same characters of the name, which is why the G0
#  project's substring arithmetic is a table here:
#
#   - the DEVICE-SELECT DEFINE the umbrella stm32f4xx.h dispatches on -
#     ST's own convention, and an irregular one: STM32F429xx for the whole
#     F429 line, but STM32F411xE for the F411's E-size parts (the C-size
#     shares that header), STM32F401xC / STM32F401xE for the two F401
#     headers, STM32F410Cx / Rx / Tx by PACKAGE for the F410;
#   - the CRT, which is per DEVICE HEADER (the vector table's names and
#     length follow the header's IRQn list): src/glue/startup_<stem>.cpp;
#   - the LINKER SCRIPT, which is per PART (the size letter decides flash
#     and RAM): ld/<part>.ld.
#
#  THE BOARD TYPE an app's "// build: boards =" line names is the part
#  number's last six characters (f446re, f429zi, f411ce): one board per part
#  on this desk, so the part IS the board - the G0 project's rule.
#
#  A part that is not in this table is refused at configure time with the
#  three names it would need; the family's OTHER headers are compile-checked
#  by brio check stm32f4 without any of this (the check calls the compiler
#  with the define directly).
# ============================================================================

# STM32F4_PARTS: <part>=<define>:<startup stem> (a colon, not a semicolon:
# the row is one list element and a semicolon would split it)
set(STM32F4_PARTS
    "stm32f429zi=STM32F429xx:stm32f429"
    "stm32f446re=STM32F446xx:stm32f446"
    "stm32f411ce=STM32F411xE:stm32f411"
)

# stm32f4_part_facts(<part> OUT_DEFINE OUT_STARTUP OUT_BOARD)
function(stm32f4_part_facts part out_define out_startup out_board)
    foreach(_row IN LISTS STM32F4_PARTS)
        string(REPLACE "=" ";" _kv "${_row}")
        string(REPLACE ":" ";" _kv "${_kv}")
        list(GET _kv 0 _name)
        if(_name STREQUAL part)
            list(GET _kv 1 _define)
            list(GET _kv 2 _stem)
            set(${out_define} "${_define}" PARENT_SCOPE)
            set(${out_startup} "src/glue/startup_${_stem}.cpp" PARENT_SCOPE)
            string(SUBSTRING "${part}" 5 6 _board)
            set(${out_board} "${_board}" PARENT_SCOPE)
            return()
        endif()
    endforeach()
    message(FATAL_ERROR "stm32f4: unknown part '${part}' - add it to cmake/stm32f4-parts.cmake "
                        "with its device define, its startup stem and an ld/${part}.ld")
endfunction()
