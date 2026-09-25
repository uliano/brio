# ============================================================================
#  ch32x035-parts.cmake - the part table of the ch32x035 project: what a
#  part number (CH32X035_MCU, lower case) decides.
#
#  THE SERIES HAS NO VENDOR HEADER in this build (brio/ch32x035/device.hpp),
#  so nothing can be asked of a device header: every per-part fact is
#  STATED, here and in brio/ch32x035/parts/<part>.hpp. This table is the
#  build's half of that - what the build itself needs - and the stratum's
#  parts/ file is the code's half.
#
#   - the PART DEFINITION the stratum's device.hpp dispatches on
#     (CH32X035F8 and the like): one macro, one parts/ file;
#   - the LINKER SCRIPT ld/<part>.ld;
#   - the BOARD TYPE an app's "// build: boards =" line names, which is the
#     part number without its "ch32" prefix (x035f8, x033f8), the spelling
#     the bench manifest and bin/brio use;
#   - the ISA AND THE ABI the compiler is asked for: every part of the
#     series is the QingKe V4C, RV32IMAC, built for its full ISA under WCH's
#     gcc, xw included - the column is stated per part as the sibling
#     project's table does, and here it is one value throughout.
#
#  THE SEVEN PARTS OF THE DATASHEET'S TABLE (CH32X035/X033 datasheet V1.7,
#  the model table before chapter 1): six CH32X035 and the CH32X033F8P6,
#  all with 62 KB of code flash and 20 KB of SRAM. What tells them apart is
#  the PACKAGE - which pads it bonds, which pads it SHORTS together inside
#  (the pin table's notes 4 to 7) - and the blocks a package brings out,
#  all of which are brio/ch32x035/parts/<part>.hpp's. The CH32X035G8U6
#  (QFN28) and the CH32X035G8R6 (QSOP28) share a number and not a
#  bonding, so their keys carry the package's letter; every other key is
#  the number alone.
#
#  A part with no ld/<part>.ld is refused at configure time by the message
#  below rather than by a missing file.
# ============================================================================

# CH32X035_PARTS:
#   <part>=<part definition>:<flash KB>:<RAM KB>:<board type>:<arch>:<abi>
set(CH32X035_PARTS
    "ch32x035r8=CH32X035R8:62:20:x035r8:rv32imac_xw:ilp32"
    "ch32x035c8=CH32X035C8:62:20:x035c8:rv32imac_xw:ilp32"
    "ch32x035g8u=CH32X035G8U:62:20:x035g8u:rv32imac_xw:ilp32"
    "ch32x035g8r=CH32X035G8R:62:20:x035g8r:rv32imac_xw:ilp32"
    "ch32x035f8=CH32X035F8:62:20:x035f8:rv32imac_xw:ilp32"
    "ch32x035f7=CH32X035F7:62:20:x035f7:rv32imac_xw:ilp32"
    "ch32x033f8=CH32X033F8:62:20:x033f8:rv32imac_xw:ilp32"
)

# ch32x035_part_facts(<part> OUT_DEFINE OUT_FLASH_KB OUT_RAM_KB OUT_BOARD
#                     OUT_ARCH OUT_ABI)
function(ch32x035_part_facts part out_define out_flash out_ram out_board out_arch out_abi)
    foreach(_row IN LISTS CH32X035_PARTS)
        string(REPLACE "=" ";" _kv "${_row}")
        string(REPLACE ":" ";" _kv "${_kv}")
        list(GET _kv 0 _name)
        if(_name STREQUAL part)
            list(GET _kv 1 _define)
            list(GET _kv 2 _flash)
            list(GET _kv 3 _ram)
            list(GET _kv 4 _board)
            list(GET _kv 5 _arch)
            list(GET _kv 6 _abi)
            set(${out_define} "${_define}" PARENT_SCOPE)
            set(${out_flash} "${_flash}" PARENT_SCOPE)
            set(${out_ram} "${_ram}" PARENT_SCOPE)
            set(${out_board} "${_board}" PARENT_SCOPE)
            set(${out_arch} "${_arch}" PARENT_SCOPE)
            set(${out_abi} "${_abi}" PARENT_SCOPE)
            return()
        endif()
    endforeach()
    message(FATAL_ERROR "ch32x035: unknown part '${part}' - add it to cmake/ch32x035-parts.cmake "
                        "with its part definition, its memories, its board type, its ISA and its "
                        "ABI, and give it ld/${part}.ld and brio/ch32x035/parts/${part}.hpp")
endfunction()
