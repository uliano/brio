# ============================================================================
#  ch32vx03-parts.cmake - the part table of the ch32vx03 project: what a
#  part number (CH32VX03_MCU, lower case) decides.
#
#  THE FAMILY HAS NO VENDOR HEADER (brio/ch32vx03/device.hpp), so unlike
#  the three ST projects nothing can be asked of a device header: every
#  per-part fact is STATED, here and in brio/ch32vx03/parts/<part>.hpp.
#  This table is the build's half of that - the things the build itself
#  needs - and the stratum's parts/ file is the code's half.
#
#   - the PART DEFINITION the stratum's device.hpp dispatches on
#     (CH32V203C8 and the like): one macro, one parts/ file;
#   - the LINKER SCRIPT ld/<part>.ld, because flash and RAM are the part's
#     (32K/10K, 64K/20K, 128K/64K - CH32V203 datasheet V2.8 table 2-1;
#     128K/32K and 256K/64K - CH32V307 datasheet V3.5 table 2-1-1);
#   - the BOARD TYPE an app's "// build: boards =" line names, which is
#     the part number without its "ch32" prefix (v203c8, v303vc), the
#     spelling the bench manifest and bin/brio use;
#   - the ISA AND THE ABI the compiler is asked for (-march, -mabi): a
#     PART fact and not a family one, because two cores share this
#     stratum - the QingKe V4B of every CH32V203 (RV32IMAC) and the V4F of
#     every CH32V303 (RV32IMAFC, the single-precision FPU) - and each part
#     is built for its own full ISA under WCH's gcc, xw included. The V4F
#     parts take the ilp32f ABI, float arguments in f-registers, which
#     their gcc resolves to the rv32imafc_zaamo_zalrsc_xw/ilp32f multilib.
#
#  The thirteen parts are listed here whether or not a board exists for
#  them: the table is a statement about the FAMILY, and a part with no
#  ld/<part>.ld is refused at configure time by the message below rather
#  than by a missing file. THREE DEVICE CLASSES live in it (WCH's own
#  division, which the reference manual keys its chapters by): the
#  CH32V20x_D6 of every CH32V203 up to the C8; the CH32V20x_D8 of the
#  CH32V203RB, whose vector table has a different tail, which carries a
#  32-bit TIM5 and a 10M Ethernet and whose HSE is 32 MHz where the rest
#  take 3..25 MHz; and the CH32V30x_D8 of the four CH32V303, with a
#  second DMA controller, eight serial ports on the larger two and a
#  vector table 104 words long. Each part's facts belong to its own
#  parts/ file, never to a guess made from a sibling.
#
#  THE FLASH COLUMN IS THE ZERO-WAIT WINDOW. On the CH32V303RC and VC the
#  array is 480 KB and a user option byte divides it: the window the core
#  executes from at full speed (256 KB with 64 KB of SRAM, the split a
#  part leaves the factory in) and a slower tail above it, which
#  ld/<part>.ld names and places nothing in.
# ============================================================================

# CH32VX03_PARTS:
#   <part>=<part definition>:<flash KB>:<RAM KB>:<board type>:<arch>:<abi>
set(CH32VX03_PARTS
    "ch32v203f6=CH32V203F6:32:10:v203f6:rv32imac_xw:ilp32"
    "ch32v203f8=CH32V203F8:64:20:v203f8:rv32imac_xw:ilp32"
    "ch32v203g6=CH32V203G6:32:10:v203g6:rv32imac_xw:ilp32"
    "ch32v203g8=CH32V203G8:64:20:v203g8:rv32imac_xw:ilp32"
    "ch32v203k6=CH32V203K6:32:10:v203k6:rv32imac_xw:ilp32"
    "ch32v203k8=CH32V203K8:64:20:v203k8:rv32imac_xw:ilp32"
    "ch32v203c6=CH32V203C6:32:10:v203c6:rv32imac_xw:ilp32"
    "ch32v203c8=CH32V203C8:64:20:v203c8:rv32imac_xw:ilp32"
    "ch32v203rb=CH32V203RB:128:64:v203rb:rv32imac_xw:ilp32"
    "ch32v303cb=CH32V303CB:128:32:v303cb:rv32imafc_xw:ilp32f"
    "ch32v303rb=CH32V303RB:128:32:v303rb:rv32imafc_xw:ilp32f"
    "ch32v303rc=CH32V303RC:256:64:v303rc:rv32imafc_xw:ilp32f"
    "ch32v303vc=CH32V303VC:256:64:v303vc:rv32imafc_xw:ilp32f"
)

# ch32vx03_part_facts(<part> OUT_DEFINE OUT_FLASH_KB OUT_RAM_KB OUT_BOARD
#                     OUT_ARCH OUT_ABI)
function(ch32vx03_part_facts part out_define out_flash out_ram out_board out_arch out_abi)
    foreach(_row IN LISTS CH32VX03_PARTS)
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
    message(FATAL_ERROR "ch32vx03: unknown part '${part}' - add it to cmake/ch32vx03-parts.cmake "
                        "with its part definition, its memories, its board type, its ISA and its "
                        "ABI, and give it ld/${part}.ld and brio/ch32vx03/parts/${part}.hpp")
endfunction()
