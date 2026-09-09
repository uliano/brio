"""brio fuses: read and write a board's fuses - the AVR-Dx FUSE bytes over
UPDI, the SAM C21 NVM user row over SWD - by the board type."""

import argparse
import json
import os
import re
import subprocess
import sys
import time

from bench.common import *   # noqa: F401,F403
from bench.flash import avrdude_base   # noqa: E402

# ---------------------------------------------------------------------------
#  fuses
#
#  Fuses are PROVISIONING, not build output: they are a property of the chip
#  on the desk, they survive every reflash, and only the programmer can write
#  them (on AVR-Dx the CPU can read them and nothing more - DS40002247B
#  11.3.1.5; on SAM C21 the NVMCTRL could write them but brio's driver
#  deliberately does not). They therefore belong here, next to the manifest
#  that says which chip is which, and not in an env.
#
#  BOTH ARCHITECTURES, ONE VERB, and the board TYPE picks the half:
#  `fuses <board>` reads and decodes, `fuses <board> name=value ...` writes.
#  What differs is the memory underneath - avrdude's FUSE bytes over UPDI on
#  AVR-Dx, the 32-byte NVM User Row over SWD on SAM C21 - and the field names
#  that go with it. A name belonging to the other family is refused by name,
#  because "unknown fuse" would be the wrong diagnosis.
#
#  The AVR names are avrdude's own memory names, which are also the data
#  sheet's register names in FUSE. The values are bytes: decimal, or
#  0x-prefixed. Every write is read back and reported, because a fuse written
#  wrong is a board that no longer boots the way its firmware expects.
# ---------------------------------------------------------------------------

FUSES = {
    "wdtcfg":   "watchdog PERIOD/WINDOW at boot (0 = off, and unlocked)",
    "bodcfg":   "brown-out detector level and sampling mode",
    "osccfg":   "start-up oscillator select and its frequency",
    "syscfg0":  "EESAVE (bit 0), RESET pin mode, CRC source",
    "syscfg1":  "start-up time",
    "codesize": "APPEND in 512-byte blocks (0 = APPCODE runs to FLASHEND)",
    "bootsize": "BOOTEND in 512-byte blocks (0 = the whole Flash is BOOT)",
}


def fuse_read(prog, mcu, names):
    """{name: byte} straight from the chip."""
    argv = avrdude_base(prog, mcu)
    for name in names:
        argv += ["-U", "%s:r:-:h" % name]
    out = subprocess.run(argv, cwd=ROOT, capture_output=True, text=True)
    if out.returncode != 0:
        sys.stderr.write(out.stderr)
        die("avrdude could not read the fuses of this board")
    # avrdude writes each value to stdout, one per -U, in order.
    values = [v.strip() for v in out.stdout.split() if v.strip()]
    if len(values) != len(names):
        sys.stderr.write(out.stdout + out.stderr)
        die("expected %d fuse value(s), got %d" % (len(names), len(values)))
    return dict(zip(names, (int(v, 0) for v in values)))


def cmd_fuses(args):
    entry = board_entry(args.name)
    prog = entry["programmer"]
    # The MCU of the board TYPE: fuses are the chip's, not any one app's.
    btype = entry["board"]
    tspec = board_type(btype)
    if tspec["project"] == "stm32g0":
        die("the STM32G0's option bytes (nBOOT, RDP, BOR, WDG...) have no "
            "bench verb yet - read them with OpenOCD (`stm32l4x option_read 0 "
            "0x20`) until one exists; `fuses` speaks AVR fuses and the SAM's "
            "user row only")
    if tspec["flash"] == "openocd":
        return sam_cmd_fuses(args, prog, tspec)
    return avr_cmd_fuses(args, prog, tspec)


def avr_cmd_fuses(args, prog, tspec):
    mcu = tspec["mcu"]
    # SAM-only syntax, refused by name rather than ignored: both flags exist
    # for the NVM User Row's whole-row erase/write, which has no counterpart
    # in a fuse byte written one at a time over UPDI.
    for flag, on in (("--rewrite", args.rewrite),
                     ("--i-know-what-this-does", args.i_know_what_this_does)):
        if on:
            die("%s belongs to the SAM C21 NVM User Row and board '%s' is an "
                "AVR-Dx: a fuse byte here is written on its own over UPDI, "
                "with no row to erase and no guarded field" % (flag, args.name))

    writes = {}
    for spec in args.assignment:
        if "=" not in spec:
            die("a fuse assignment is <name>=<value>, got '%s'" % spec)
        name, value = spec.split("=", 1)
        name = name.strip().lower()
        if name in SAM_FUSE_BY_NAME:
            die("'%s' is a SAM C21 NVM User Row field and board '%s' is an "
                "AVR-Dx: its fuses are %s"
                % (name, args.name, ", ".join(sorted(FUSES))))
        if name not in FUSES:
            die("unknown fuse '%s' (known: %s)" % (name, ", ".join(sorted(FUSES))))
        try:
            writes[name] = int(value.strip(), 0) & 0xFF
        except ValueError:
            die("fuse %s: '%s' is not a byte" % (name, value))

    if writes:
        before = fuse_read(prog, mcu, sorted(writes))
        argv = avrdude_base(prog, mcu)
        for name in sorted(writes):
            argv += ["-U", "%s:w:%d:m" % (name, writes[name])]
        print("bench: " + " ".join(argv))
        rc = subprocess.call(argv, cwd=ROOT)
        if rc != 0:
            return rc
        after = fuse_read(prog, mcu, sorted(writes))
        bad = 0
        for name in sorted(writes):
            ok = after[name] == writes[name]
            bad += 0 if ok else 1
            print("  %-9s 0x%02X -> 0x%02X  %s"
                  % (name, before[name], after[name], "ok" if ok else "MISMATCH"))
        if bad:
            die("%d fuse(s) did not take the value asked for" % bad)

    names = sorted(FUSES)
    values = fuse_read(prog, mcu, names)
    print("board %s (%s):" % (args.name, mcu))
    for name in names:
        print("  %-9s 0x%02X  %s" % (name, values[name], FUSES[name]))
    boot = values["bootsize"]
    code = values["codesize"]
    print("  -> BOOT %s" % ("the whole Flash (nothing is writable from software)"
                            if boot == 0 else "0x00000..0x%05X" % (boot * 512 - 1)))
    if boot != 0:
        print("     APPCODE %s"
              % ("0x%05X..FLASHEND" % (boot * 512) if code == 0 else
                 ("none" if code <= boot else
                  "0x%05X..0x%05X" % (boot * 512, code * 512 - 1))))
    print("     EESAVE %s (EEPROM %s a chip erase)"
          % ("set" if values["syscfg0"] & 1 else "clear",
             "survives" if values["syscfg0"] & 1 else "is erased by"))
    return 0


# ---------------------------------------------------------------------------
#  fuses, the SAM half: the NVM User Row over SWD
#
#  WHAT THIS MEMORY IS. On SAM C21 the fuses are a flash row like any other:
#  the NVM User Row, read at 0x00804000, whose first 64 bits table 9-4 maps
#  to BOOTPROT, the EEPROM emulation size, the BODVDD detector, the watchdog
#  and the region LOCK word. Peripherals load those bits at power-on and at a
#  user reset, so A CHANGE TAKES EFFECT AT THE NEXT RESET AND NOT BEFORE
#  (9.3), and the row SURVIVES A CHIP ERASE - a wrong word is not undone by
#  reflashing, which is the whole reason this is provisioning and lives here.
#
#  HOW IT IS WRITTEN: BY HAND, THROUGH NVMCTRL, WITH THE CORE HALTED. The
#  sequence is chapter 27's own - Erase Auxiliary Row (CMD 0x05) then, per
#  page, Page Buffer Clear (0x44), 32-bit stores into the row's address range
#  to load the page buffer, and Write Auxiliary Page (0x06) - each command
#  carrying the 0xA5 CMDEX key in the same 16-bit store as CMD (27.8.1) and
#  each followed by a wait on INTFLAG.READY and a read of STATUS. ADDR is a
#  HALF-WORD OFFSET FROM THE SECTION BASE (27.8.8): the auxiliary space
#  starts at 0x00800000, so the user row is ADDR 0x2000 and its four pages
#  0x2000, 0x2020, 0x2040, 0x2060. Note that the field is 22 bits, so the
#  absolute-address/2 form that other people's code uses arrives at the same
#  number only by truncation.
#
#  WHY NOT `at91samd nvmuserrow`. The shipped OpenOCD does carry that helper
#  and its READ half works on this silicon (checked: it reports the same 64
#  bits this code reads). It is not used to WRITE for two reasons. Its
#  interface is 64 bits wide while EAR erases a whole 256-byte ROW, so
#  nothing in its contract says what happens to the rest of that row - and
#  the rule below is that the row is preserved bit-exactly. And the same
#  command group carries `chip-erase` and `set-security`, which this tool
#  must never be one typo away from.
#
#  THE SAFETY RULES, all of them enforced below:
#   - the whole 256-byte ROW is read, modified and written back, because that
#     is the unit EAR erases; pages that come out all-0xFF are left as the
#     erase made them and the tool says which pages it wrote;
#   - the old row is printed in full before anything is written;
#   - the row is read back afterwards and diffed against what was intended,
#     and a mismatch is a nonzero exit;
#   - only the fields in SAM_FUSES can be written. The BODCORE calibration
#     bits table 9-4 marks DO NOT CHANGE, and every Reserved bit, are carried
#     across untouched, AND THERE IS NO RAW BIT ESCAPE: a field this tool
#     does not decode is a field it will not write;
#   - nothing here reaches the security bit (SSB) or a chip erase, in either
#     direction. brio's own samc21/nvm.hpp makes the same ruling from the
#     firmware side and exposes no user-row write at all;
#   - BOOTPROT, the LOCK word and setting WDT ALWAYS-ON all need
#     --i-know-what-this-does: each of them can leave a board that refuses to
#     be programmed the ordinary way.
# ---------------------------------------------------------------------------

SAM_USER_ROW = 0x00804000        # 9.3: where the row is READ
SAM_AUX_BASE = 0x00800000        # ... and what ADDR counts half-words from
SAM_ROW_BYTES = 256              # what one EAR erases: four pages
SAM_PAGE_BYTES = 64              # what one WAP writes

NVM_CTRLA = 0x41004000           # 16-bit: CMD[6:0] and CMDEX[15:8] in ONE store
NVM_INTFLAG = 0x41004014         # 8-bit: READY is bit 0
NVM_STATUS = 0x41004018          # 16-bit: PROGE/LOCKE/NVME are write-one-to-clear
NVM_ADDR = 0x4100401C            # 32-bit: half-words from the section base

NVM_CMDEX = 0xA5
NVM_CMD_EAR = 0x05               # Erase Auxiliary Row
NVM_CMD_WAP = 0x06               # Write Auxiliary Page
NVM_CMD_PBC = 0x44               # Page Buffer Clear
NVM_CMD_INVALL = 0x46            # invalidate the read cache
NVM_STATUS_ERRORS = 0x1C         # PROGE (2) | LOCKE (3) | NVME (4)

DHCSR = 0xE000EDF0               # the debug register brio always leaves off
DHCSR_DEBUG_OFF = 0xA05F0000


def sam_bootprot(v):
    if v == 0x7:
        return "no boot protection"
    rows = 2 << (6 - v)
    return ("the first %d row(s) = %d bytes are write-protected (table 27-2)"
            % (rows, rows * SAM_ROW_BYTES))


def sam_eeprom(v):
    if v == 0x7:
        return "no EEPROM emulation area"
    rows = 1 << (6 - v)
    return ("%d row(s) = %d bytes carved off the top of the Flash (table 27-3)"
            % (rows, rows * SAM_ROW_BYTES))


# The three points table 45-18 actually gives. Everything between them is an
# interpolation neither this tool nor samc21/supc.hpp makes.
SAM_BOD_LEVELS = {8: "2.80 V typ", 9: "2.85 V typ", 44: "4.51 V typ"}


def sam_bodvdd_level(v):
    anchor = SAM_BOD_LEVELS.get(v)
    if anchor:
        return "table 45-18 anchor: %s" % anchor
    return ("no anchor at this level - 45-18 gives only 8, 9 and 44 (the "
            "bench measured the step at 48.7 mV)")


def sam_wdt_cycles(v):
    cycles = 8 << v
    return ("%d CLK_WDT_OSC cycles, about %d ms at OSCULP32K's nominal "
            "1.024 kHz" % (cycles, (cycles * 1000 + 512) // 1024))


def sam_lock(v):
    locked = [i for i in range(16) if not (v >> i) & 1]
    if not locked:
        return "every region unlocked at reset (a ZERO bit means locked)"
    return "region(s) LOCKED at reset: " + ", ".join(str(i) for i in locked)


# name, first bit, width, guarded, the register it is loaded into, decoder.
# The bit positions are table 9-4's and are the same fields samc21/nvm.hpp's
# NvmUserRow reads from the firmware side - the two must move together.
SAM_FUSES = (
    ("bootprot", 0, 3, True, "NA, see table 27-2", sam_bootprot),
    ("eeprom", 4, 3, False, "NA, see table 27-3", sam_eeprom),
    ("bodvdd_level", 8, 6, False, "SUPC.BODVDD.LEVEL", sam_bodvdd_level),
    ("bodvdd_disable", 14, 1, False, "SUPC.BODVDD.ENABLE (inverted)",
     lambda v: "the detector is ENABLED at power-on" if v == 0
               else "the detector is DISABLED at power-on"),
    ("bodvdd_action", 15, 2, False, "SUPC.BODVDD.ACTION",
     lambda v: {0: "none - STATUS.BODVDDDET only",
                1: "reset", 2: "interrupt", 3: "reserved"}[v]),
    ("wdt_enable", 26, 1, False, "WDT.CTRLA.ENABLE",
     lambda v: "the watchdog runs from power-on" if v else "off at power-on"),
    ("wdt_always_on", 27, 1, True, "WDT.CTRLA.ALWAYSON",
     lambda v: "ALWAYS-ON: software can no longer stop the watchdog" if v
               else "software may stop the watchdog"),
    ("wdt_period", 28, 4, False, "WDT.CONFIG.PER", sam_wdt_cycles),
    ("wdt_window", 32, 4, False, "WDT.CONFIG.WINDOW", sam_wdt_cycles),
    ("wdt_ewoffset", 36, 4, False, "WDT.EWCTRL.EWOFFSET", sam_wdt_cycles),
    ("wdt_wen", 40, 1, False, "WDT.CTRLA.WEN",
     lambda v: "window mode" if v else "normal mode"),
    ("bodvdd_hysteresis", 41, 1, False, "SUPC.BODVDD.HYST",
     lambda v: "hysteresis ON" if v else "hysteresis off"),
    ("lock", 48, 16, True, "NVMCTRL.LOCK", sam_lock),
)

SAM_FUSE_BY_NAME = {f[0]: f for f in SAM_FUSES}

# The bits table 9-4 gives a meaning this tool refuses to touch. They are
# carried across every write verbatim; two of them are the BODCORE
# calibration, which the note under table 9-4 and 22.6.3.4 both say must not
# change (samc21/supc.hpp's BodCore makes the same ruling by having no setter).
SAM_PRESERVED = (
    (3, 1, "Reserved"),
    (7, 1, "Reserved"),
    (17, 9, "BODCORE calibration - table 9-4: DO NOT CHANGE"),
    (42, 1, "BODCORE calibration - table 9-4: DO NOT CHANGE"),
    (43, 5, "Reserved"),
)


def _sam_bit_map_is_complete():
    """Every one of the 64 mapped bits is either a writable field or a
    preserved one, exactly once. A field added to one table and not the other
    would otherwise be silently dropped by a write."""
    seen = 0
    for span in [(f[1], f[2]) for f in SAM_FUSES] + [s[:2] for s in SAM_PRESERVED]:
        mask = ((1 << span[1]) - 1) << span[0]
        if seen & mask:
            return False
        seen |= mask
    return seen == (1 << 64) - 1


assert _sam_bit_map_is_complete(), \
    "bench/cli.py: SAM_FUSES + SAM_PRESERVED must tile bits 0..63 of table 9-4"


def sam_bit_span(first, width):
    return ("bit %d" % first) if width == 1 \
        else "bits %d:%d" % (first + width - 1, first)


def sam_get(value, first, width):
    return (value >> first) & ((1 << width) - 1)


def sam_set(value, first, width, new):
    mask = ((1 << width) - 1) << first
    return (value & ~mask) | ((new << first) & mask)


# ---------------------------------------------------------------------------
#  OpenOCD as a register-level instrument
# ---------------------------------------------------------------------------

MDW_RE = re.compile(r"^0x([0-9a-fA-F]{8}):((?:\s+[0-9a-fA-F]{8})+)\s*$", re.M)


def openocd_batch(prog, commands, timeout=180.0):
    """Run one OpenOCD session over the manifest's probe. Returns
    (returncode, all output) - OpenOCD logs and command output both land on
    stderr, so the two streams are joined and parsed together."""
    argv = [manifest.OPENOCD, "-f", "interface/cmsis-dap.cfg",
            "-c", "cmsis-dap backend hid"]
    if prog.get("serial"):
        argv += ["-c", "adapter serial %s" % prog["serial"]]
    argv += ["-f", "target/at91samdXX.cfg"]
    for c in commands:
        argv += ["-c", c]
    try:
        out = subprocess.run(argv, cwd=ROOT, capture_output=True, text=True,
                             timeout=timeout)
    except subprocess.TimeoutExpired:
        die("OpenOCD did not finish within %.0f s - is the probe held by a "
            "debug session? (the Atmel-ICE is single-client)" % timeout)
    return out.returncode, (out.stdout or "") + (out.stderr or "")


def parse_mdw(text, base, words):
    """The words an `mdw` dump reported, or None if the range is incomplete."""
    got = {}
    for m in MDW_RE.finditer(text):
        addr = int(m.group(1), 16)
        for i, word in enumerate(m.group(2).split()):
            got[addr + 4 * i] = int(word, 16)
    out = []
    for i in range(words):
        if base + 4 * i not in got:
            return None
        out.append(got[base + 4 * i])
    return out


def sam_read_row(prog):
    """The whole 256-byte NVM User Row. Read WITHOUT halting the core - the
    debug access port reads flash while the CPU runs - and the session ends
    the way every SAM session here ends, with DHCSR.C_DEBUGEN cleared, so a
    board is left exactly as it was found."""
    rc, text = openocd_batch(prog, [
        "init",
        "mdw 0x%08X %d" % (SAM_USER_ROW, SAM_ROW_BYTES // 4),
        "mww 0x%08X 0x%08X" % (DHCSR, DHCSR_DEBUG_OFF),
        "shutdown",
    ])
    words = parse_mdw(text, SAM_USER_ROW, SAM_ROW_BYTES // 4)
    if words is None:
        sys.stderr.write(text)
        die("could not read the NVM User Row over SWD (rc %d) - probe wired "
            "to this board and free?" % rc)
    row = bytearray()
    for word in words:
        row += word.to_bytes(4, "little")
    return row


def sam_write_script(row, pages):
    """The OpenOCD/Tcl body that erases the auxiliary row and writes `pages`
    of `row` back into it. Every command checks INTFLAG.READY and STATUS, and
    the first failure aborts before the next store - a half-written row is
    worse than an unwritten one."""
    def addr_field(byte_addr):
        return (byte_addr - SAM_AUX_BASE) >> 1

    out = []
    out.append("proc nvm_wait {} {")
    out.append("    for {set i 0} {$i < 100000} {incr i} {")
    out.append("        if {[lindex [read_memory 0x%08X 8 1] 0] & 1} { return 1 }"
               % NVM_INTFLAG)
    out.append("    }")
    out.append("    return 0")
    out.append("}")
    out.append("proc nvm_cmd {name cmd} {")
    out.append("    mwh 0x%08X 0x%04X" % (NVM_STATUS, NVM_STATUS_ERRORS))
    out.append("    mwh 0x%08X [expr {0x%02X00 | $cmd}]" % (NVM_CTRLA, NVM_CMDEX))
    out.append("    if {![nvm_wait]} {")
    out.append("        echo \"FUSES-FAIL $name INTFLAG.READY never returned\"")
    out.append("        return 0")
    out.append("    }")
    out.append("    set st [lindex [read_memory 0x%08X 16 1] 0]" % NVM_STATUS)
    out.append("    if {$st & 0x%02X} {" % NVM_STATUS_ERRORS)
    out.append("        echo \"FUSES-FAIL $name STATUS [format 0x%04X $st]\"")
    out.append("        return 0")
    out.append("    }")
    out.append("    echo \"FUSES-STEP $name STATUS [format 0x%04X $st]\"")
    out.append("    return 1")
    out.append("}")
    out.append("proc nvm_row {} {")
    out.append("    mww 0x%08X 0x%08X" % (NVM_ADDR, addr_field(SAM_USER_ROW)))
    out.append("    if {![nvm_cmd EAR 0x%02X]} { return 0 }" % NVM_CMD_EAR)
    for page in pages:
        base = SAM_USER_ROW + page * SAM_PAGE_BYTES
        out.append("    if {![nvm_cmd PBC-%d 0x%02X]} { return 0 }"
                   % (page, NVM_CMD_PBC))
        # Ascending 32-bit stores: 27.6.4.3 faults an 8-bit one and resets
        # the 64-bit holding register whenever a write crosses a section
        # boundary, so ascending whole words is the one pattern that is
        # immune to both traps.
        for off in range(0, SAM_PAGE_BYTES, 4):
            word = int.from_bytes(bytes(row[page * SAM_PAGE_BYTES + off:
                                            page * SAM_PAGE_BYTES + off + 4]),
                                  "little")
            out.append("    mww 0x%08X 0x%08X" % (base + off, word))
        out.append("    mww 0x%08X 0x%08X" % (NVM_ADDR, addr_field(base)))
        out.append("    if {![nvm_cmd WAP-%d 0x%02X]} { return 0 }"
                   % (page, NVM_CMD_WAP))
    out.append("    nvm_cmd INVALL 0x%02X" % NVM_CMD_INVALL)
    out.append("    return 1")
    out.append("}")
    out.append("init")
    out.append("halt")
    out.append("if {[nvm_row]} { echo \"FUSES-WRITTEN\" }")
    # The row is loaded by the peripherals at reset and not before (9.3), so
    # the board is reset here: after this verb the silicon is running under
    # what the row now says. Then debug off, as after every flash.
    out.append("reset run")
    out.append("mww 0x%08X 0x%08X" % (DHCSR, DHCSR_DEBUG_OFF))
    out.append("shutdown")
    return "\n".join(out) + "\n"


def sam_write_row(prog, row, pages):
    path = os.path.join(ROOT, "build-cmake", "bench_fuses.cfg")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="ascii") as f:
        f.write(sam_write_script(row, pages))
    argv = [manifest.OPENOCD, "-f", "interface/cmsis-dap.cfg",
            "-c", "cmsis-dap backend hid"]
    if prog.get("serial"):
        argv += ["-c", "adapter serial %s" % prog["serial"]]
    argv += ["-f", "target/at91samdXX.cfg", "-f", path]
    print("bench: " + " ".join(argv))
    try:
        out = subprocess.run(argv, cwd=ROOT, capture_output=True, text=True,
                             timeout=180.0)
    except subprocess.TimeoutExpired:
        die("OpenOCD did not finish the user-row write within 180 s")
    text = (out.stdout or "") + (out.stderr or "")
    for line in text.splitlines():
        if line.startswith("FUSES-"):
            print("  " + line)
    if "FUSES-WRITTEN" not in text:
        sys.stderr.write(text)
        die("the NVM User Row write did not complete - the row may be ERASED "
            "or partly written; re-run this verb with the intended values "
            "before resetting the board")


# ---------------------------------------------------------------------------
#  fuses <sam board> [name=value ...]
# ---------------------------------------------------------------------------

def sam_print_row(name, mcu, row):
    value = int.from_bytes(bytes(row[0:8]), "little")
    print("board %s (%s) NVM User Row @ 0x%08X:" % (name, mcu, SAM_USER_ROW))
    for line in range(2):
        print("  0x%02X  %s" % (line * 16, " ".join("%02X" % b for b in
                                                    row[line * 16:line * 16 + 16])))
    print("  words 0..1  0x%08X 0x%08X"
          % (int.from_bytes(bytes(row[0:4]), "little"),
             int.from_bytes(bytes(row[4:8]), "little")))
    tail = row[8:]
    print("  the rest of the 256-byte row (what one EAR erases) is %s"
          % ("all 0xFF" if all(b == 0xFF for b in tail)
             else "NOT all 0xFF and is preserved verbatim"))
    for fname, first, width, guarded, reg, decode in SAM_FUSES:
        field = sam_get(value, first, width)
        print("  %-18s %-6s %-10s %s%s"
              % (fname, "0x%0*X" % ((width + 3) // 4, field),
                 sam_bit_span(first, width), reg,
                 "  [guarded]" if guarded else ""))
        print("  %-18s        -> %s" % ("", decode(field)))
    print("  preserved, never written by this tool:")
    for first, width, what in SAM_PRESERVED:
        print("    %-10s %-6s %s"
              % (sam_bit_span(first, width),
                 "0x%0*X" % ((width + 3) // 4, sam_get(value, first, width)),
                 what))
    print("  a change to any of this takes effect at the NEXT RESET (9.3), "
          "and the row survives a chip erase")


def sam_cmd_fuses(args, prog, tspec):
    mcu = tspec["mcu"]

    writes = {}
    for spec in args.assignment:
        if "=" not in spec:
            die("a user-row assignment is <name>=<value>, got '%s'" % spec)
        fname, value = spec.split("=", 1)
        fname = fname.strip().lower()
        if fname in FUSES:
            die("'%s' is an AVR-Dx fuse byte and board '%s' is a SAM C21: its "
                "NVM User Row fields are %s"
                % (fname, args.name, ", ".join(sorted(SAM_FUSE_BY_NAME))))
        if fname not in SAM_FUSE_BY_NAME:
            die("unknown NVM User Row field '%s' (known: %s). There is no raw "
                "bit escape: a field this tool does not decode is a field it "
                "does not write." % (fname, ", ".join(sorted(SAM_FUSE_BY_NAME))))
        width = SAM_FUSE_BY_NAME[fname][2]
        try:
            field = int(value.strip(), 0)
        except ValueError:
            die("field %s: '%s' is not a number" % (fname, value))
        if field < 0 or field >= (1 << width):
            die("field %s is %d bit(s) wide: %s does not fit"
                % (fname, width, value.strip()))
        writes[fname] = field

    old = sam_read_row(prog)
    sam_print_row(args.name, mcu, old)
    if not writes and not args.rewrite:
        return 0

    old_value = int.from_bytes(bytes(old[0:8]), "little")
    new_value = old_value
    for fname in sorted(writes):
        _, first, width, _, _, _ = SAM_FUSE_BY_NAME[fname]
        new_value = sam_set(new_value, first, width, writes[fname])

    # The guarded fields. ALWAYS-ON is one-way in practice (nothing running
    # on the chip can turn the watchdog off again), BOOTPROT write-protects
    # the bottom of the Flash and LOCK can refuse a region to the programmer,
    # so each of them is a way to hand back a board that will not take new
    # firmware the ordinary way. Only a CHANGE is guarded - restating what is
    # already there costs nothing and needs no ceremony.
    needs_ack = []
    for fname in sorted(writes):
        _, first, width, guarded, _, _ = SAM_FUSE_BY_NAME[fname]
        if not guarded:
            continue
        was, now = sam_get(old_value, first, width), sam_get(new_value, first, width)
        if fname == "wdt_always_on" and not (now and not was):
            continue
        if was != now:
            needs_ack.append("%s 0x%X -> 0x%X" % (fname, was, now))
    if needs_ack and not args.i_know_what_this_does:
        die("refusing to write %s without --i-know-what-this-does: these "
            "fields can leave a board that no longer takes firmware the "
            "ordinary way (BOOTPROT write-protects the bottom of the Flash, "
            "LOCK can refuse a region, ALWAYS-ON makes the watchdog "
            "unstoppable)" % "; ".join(needs_ack))

    new = bytearray(old)
    new[0:8] = new_value.to_bytes(8, "little")
    if new == old and not args.rewrite:
        print("bench: nothing to change - the row already reads that way "
              "(--rewrite writes it back anyway)")
        return 0

    print("")
    print("bench: writing the NVM User Row of board %s" % args.name)
    if new_value == old_value:
        print("  the row is UNCHANGED: this is the erase/write path being "
              "exercised, not a new value (--rewrite)")
    for fname in sorted(writes):
        _, first, width, _, _, decode = SAM_FUSE_BY_NAME[fname]
        was, now = sam_get(old_value, first, width), sam_get(new_value, first, width)
        print("  %-18s 0x%X -> 0x%X   %s"
              % (fname, was, now, decode(now) if was != now else "(unchanged)"))
    # Whole-ROW read-modify-write. EAR erases all four pages, so every page
    # whose content is not simply erased has to be written back; a page that
    # is all-0xFF is exactly what the erase leaves, so writing it would spend
    # a flash operation to store nothing.
    pages = [p for p in range(SAM_ROW_BYTES // SAM_PAGE_BYTES)
             if any(b != 0xFF for b in new[p * SAM_PAGE_BYTES:
                                           (p + 1) * SAM_PAGE_BYTES])]
    print("  page(s) written back after the erase: %s (the rest of the row is "
          "all-0xFF, which is what the erase leaves)"
          % (", ".join(str(p) for p in pages) or "none"))
    sam_write_row(prog, new, pages)

    back = sam_read_row(prog)
    print("")
    print("bench: read back after reset")
    sam_print_row(args.name, mcu, back)
    if back != new:
        for i in range(SAM_ROW_BYTES):
            if back[i] != new[i]:
                print("  MISMATCH at 0x%02X: wanted 0x%02X, read 0x%02X"
                      % (i, new[i], back[i]))
        die("the NVM User Row did not come back as written")
    changed = [i for i in range(SAM_ROW_BYTES) if back[i] != old[i]]
    if changed:
        print("  diff against the row before the write: %s"
              % ", ".join("0x%02X 0x%02X -> 0x%02X" % (i, old[i], back[i])
                          for i in changed))
    else:
        print("  diff against the row before the write: none - bit-identical")
    return 0


