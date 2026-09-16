# ============================================================================
#  ch32v203-parts.cmake - the part table of the ch32v203 project: what a
#  part number (CH32V203_MCU, lower case) decides.
#
#  THE FAMILY HAS NO VENDOR HEADER (brio/ch32v203/device.hpp), so unlike
#  the three ST projects nothing can be asked of a device header: every
#  per-part fact is STATED, here and in brio/ch32v203/parts/<part>.hpp.
#  This table is the build's half of that - the three things the build
#  itself needs - and the stratum's parts/ file is the code's half.
#
#   - the PART DEFINITION the stratum's device.hpp dispatches on
#     (CH32V203C8 and the like): one macro, one parts/ file;
#   - the LINKER SCRIPT ld/<part>.ld, because flash and RAM are the part's
#     (32K/10K, 64K/20K, 128K/64K - CH32V203 datasheet V2.8 table 2-1);
#   - the BOARD TYPE an app's "// build: boards =" line names, which is
#     the part number without its "ch32" prefix (v203c8), the spelling the
#     bench manifest and bin/brio use.
#
#  The nine parts of the series are listed here whether or not a board
#  exists for them: the table is a statement about the FAMILY, and a part
#  with no ld/<part>.ld is refused at configure time by the message below
#  rather than by a missing file. The CH32V203RB is the family's other
#  DEVICE CLASS (WCH's CH32V20x_D8 against the CH32V20x_D6 of every part
#  above it): its vector table has a different tail, it carries TIM5 and a
#  10M Ethernet, and its HSE is 32 MHz where the rest take 3..25 MHz - so
#  it is named here and its facts belong to its own parts/ file, not to a
#  guess made from this one.
# ============================================================================

# CH32V203_PARTS: <part>=<part definition>:<flash KB>:<RAM KB>:<board type>
set(CH32V203_PARTS
    "ch32v203f6=CH32V203F6:32:10:v203f6"
    "ch32v203f8=CH32V203F8:64:20:v203f8"
    "ch32v203g6=CH32V203G6:32:10:v203g6"
    "ch32v203g8=CH32V203G8:64:20:v203g8"
    "ch32v203k6=CH32V203K6:32:10:v203k6"
    "ch32v203k8=CH32V203K8:64:20:v203k8"
    "ch32v203c6=CH32V203C6:32:10:v203c6"
    "ch32v203c8=CH32V203C8:64:20:v203c8"
    "ch32v203rb=CH32V203RB:128:64:v203rb"
)

# ch32v203_part_facts(<part> OUT_DEFINE OUT_FLASH_KB OUT_RAM_KB OUT_BOARD)
function(ch32v203_part_facts part out_define out_flash out_ram out_board)
    foreach(_row IN LISTS CH32V203_PARTS)
        string(REPLACE "=" ";" _kv "${_row}")
        string(REPLACE ":" ";" _kv "${_kv}")
        list(GET _kv 0 _name)
        if(_name STREQUAL part)
            list(GET _kv 1 _define)
            list(GET _kv 2 _flash)
            list(GET _kv 3 _ram)
            list(GET _kv 4 _board)
            set(${out_define} "${_define}" PARENT_SCOPE)
            set(${out_flash} "${_flash}" PARENT_SCOPE)
            set(${out_ram} "${_ram}" PARENT_SCOPE)
            set(${out_board} "${_board}" PARENT_SCOPE)
            return()
        endif()
    endforeach()
    message(FATAL_ERROR "ch32v203: unknown part '${part}' - add it to cmake/ch32v203-parts.cmake "
                        "with its part definition, its memories and its board type, and give it "
                        "ld/${part}.ld and brio/ch32v203/parts/${part}.hpp")
endfunction()
