#!/usr/bin/env python3
# ============================================================================
#  bench.py - the multi-board bench ORCHESTRATOR: build, flash, drive.
#
#  Run with any Python 3 that has pyserial installed (pip install --user
#  pyserial):
#
#      python3 tools/bench.py list
#      python3 tools/bench.py flash A test_avr_pin
#      python3 tools/bench.py run A a
#      python3 tools/bench.py console A
#      python3 tools/bench.py duo A:a B:scripts/peer.txt
#      python3 tools/bench.py fuses A
#      python3 tools/bench.py fuses A bootsize=128 codesize=0
#      python3 tools/bench.py fuses C
#      python3 tools/bench.py fuses C bodvdd_hysteresis=1
#
#  Three concerns, kept apart on purpose:
#    1. BUILD   - one CMake target per app x board TYPE, auto-discovered from
#                 each app's own "// build: boards = ..." header comment
#                 (each project's CMakeLists.txt); a configure also (re)writes
#                 that project's build-cmake/apps_<project>.json, which this
#                 file reads instead of parsing apps.ini/platformio.ini (both
#                 gone). Never a target per physical board.
#    2. IDENTITY- tools/bench_boards.py, the bench manifest: which board sits
#                 where, on which console, behind which programmer.
#    3. THIS    - resolves 1 against 2 and drives the hardware.
#
#  TWO ARCHITECTURES SHARE THIS TOOL. The board's TYPE decides everything
#  that differs (BOARD_TYPES below): db* is an AVR-Dx built by avrdx/ and
#  written by avrdude over UPDI, c21j is a SAM C21 built by samc/ and written
#  by OpenOCD over SWD. Everything above that line - the console protocol,
#  the "ALL: N pass, M fail" verdict grammar, the campaign shape - is one
#  story on both, which is the point.
#
#  A board NAME ("A", "B") is a desk position from the manifest. The consoles
#  are OBSERVABILITY ONLY: firmware never goes in through them, they only
#  carry the suite's verdicts back.
#
#  The campaign shape: board A = the DUT running a test suite, board B = a
#  scriptable instrument peer (clock stretching, NACK injection, arbitration).
#  `duo` is that shape; with one board on the desk it cannot yet be exercised.
# ============================================================================
import argparse
import json
import os
import re
import subprocess
import sys
import time

TOOLS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)

import bench_boards as manifest          # noqa: E402

try:
    import serial                        # pyserial
except ImportError:
    sys.exit("bench: pyserial is missing - run this with a Python 3 that has "
             "it installed (pip install --user pyserial)")

# BOARD TYPE -> everything that follows from it. A board type names a chip
# package, and the chip package is what decides which of the sibling CMake
# projects builds for it, which release preset that project uses, which
# app roster to read, and how firmware gets in. Mirrors avrdx/cmake/
# avr-mcus.cmake and samc/CMakePresets.json by hand - keep them in sync.
#
# "project" is also why the rosters are per project: app names COLLIDE
# across the source trees (blink, console and probe exist in both avrdx/
# and samc/), so an app name alone never identifies an app.
BOARD_TYPES = {
    "db28": {"project": "avrdx", "preset": "avr128db28-release",
             "mcu": "avr128db28", "flash": "avrdude"},
    "db32": {"project": "avrdx", "preset": "avr128db32-release",
             "mcu": "avr128db32", "flash": "avrdude"},
    "db48": {"project": "avrdx", "preset": "avr128db48-release",
             "mcu": "avr128db48", "flash": "avrdude"},
    "c21j": {"project": "samc", "preset": "samc21j-release",
             "mcu": "samc21j18a", "flash": "openocd",
             "target_cfg": "target/at91samdXX.cfg"},
    # The Nucleo-G0B1RE: STM32G0B1RE behind its on-board ST-LINK/V2.1 -
    # OpenOCD again, but the ST-LINK interface and the stm32g0x target
    # script (the stm32l4x flash driver underneath, which serves the G0).
    "g0b1re": {"project": "stm32g0", "preset": "stm32g0b1re-release",
               "mcu": "stm32g0b1re", "flash": "openocd",
               "target_cfg": "target/stm32g0x.cfg"},
}


def board_type(btype):
    spec = BOARD_TYPES.get(btype)
    if spec is None:
        die("unknown board type '%s' (known: %s) - a new type needs an entry "
            "in bench.py's BOARD_TYPES" % (btype, ", ".join(sorted(BOARD_TYPES))))
    return spec

PROMPT = "> "
DEFAULT_TIMEOUT = 60.0
SUMMARY_RE = re.compile(r"(\d+)\s+pass,\s*(\d+)\s+fail")


def die(msg):
    sys.exit("bench: " + msg)


# ---------------------------------------------------------------------------
#  Manifest / env resolution
# ---------------------------------------------------------------------------

def board_entry(name):
    entry = manifest.BOARDS.get(name)
    if entry is None:
        die("no board '%s' in the manifest (have: %s) - edit tools/bench_boards.py"
            % (name, ", ".join(sorted(manifest.BOARDS)) or "none"))
    return entry


def apps_manifest(project):
    """One project's app roster as CMake discovered it at its last configure -
    written fresh by any one `cmake --preset ...` of THAT project, regardless
    of which chip variant it targets (the scan is of source comments, not of
    what that configure happens to build). {app: {"boards": [...],
    "monitor_speed": int|None, ...}}."""
    path = os.path.join(ROOT, "build-cmake", "apps_%s.json" % project)
    if not os.path.isfile(path):
        die("build-cmake/apps_%s.json not found - configure the %s/ project "
            "first (cd %s && cmake --preset <a release preset>)"
            % (project, project, project))
    with open(path, encoding="ascii") as f:
        return json.load(f)["apps"]


def resolve_app(name, app):
    """(app info, board type) for an app on a physical board; a clear error
    if the app was never built for that board type (no
    '// build: boards' line naming it)."""
    entry = board_entry(name)
    btype = entry["board"]
    spec = board_type(btype)
    apps = apps_manifest(spec["project"])
    if app not in apps:
        die("no app '%s' under %s/src/apps/ (known to the last cmake configure "
            "of that project)" % (app, spec["project"]))
    info = apps[app]
    if btype not in info["boards"]:
        default = "db48" if spec["project"] == "avrdx" else "c21j"
        hint = ("" if btype == default else
                " - add a '// build: boards = %s' line to %s/src/apps/%s.cpp "
                "and reconfigure" % (btype, spec["project"], app))
        die("app '%s' is not built for board '%s' (type %s)%s"
            % (app, name, btype, hint))
    return info, btype


def env_speed(info):
    return int(info.get("monitor_speed") or manifest.DEFAULT_MONITOR_SPEED)


# Which app was last flashed onto which board: written by `flash`, read by the
# console commands so they open the port at that app's monitor_speed without
# being told again. It is a convenience cache, not identity - --app overrides
# it, and a board never flashed by this tool falls back to the manifest's
# DEFAULT_MONITOR_SPEED.
STATE = os.path.join(ROOT, "build-cmake", "bench_last.json")


def state_read():
    try:
        with open(STATE, encoding="ascii") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def state_write(name, app):
    data = state_read()
    data[name] = app
    try:
        os.makedirs(os.path.dirname(STATE), exist_ok=True)
        with open(STATE, "w", encoding="ascii") as f:
            json.dump(data, f, indent=2, sort_keys=True)
    except OSError:
        pass


def speed_for(name, app):
    """The console speed of a board: the named app's, else the last flashed
    app's, else the manifest default."""
    app = app or state_read().get(name)
    if not app:
        return manifest.DEFAULT_MONITOR_SPEED
    info, _ = resolve_app(name, app)
    return env_speed(info)


def console_path(name):
    entry = board_entry(name)
    path = entry.get("console")
    if not path:
        die("board '%s' has no console in the manifest" % name)
    if not os.path.exists(path):
        die("console %s of board '%s' does not exist - is the board plugged "
            "into the socket the manifest names? (bench.py list)" % (path, name))
    return path


# ---------------------------------------------------------------------------
#  list
# ---------------------------------------------------------------------------

USB_PROGRAMMERS = {
    "03eb": "Atmel",
    "04d8": "Microchip",
    "0483": "STMicro",     # the Nucleo boards' on-board ST-LINK
}


def usb_programmers():
    """Attached EDBG-class probes, read from sysfs (no root needed): the USB
    serial number is what avrdude's -P usb:<serial> selects."""
    found = []
    base = "/sys/bus/usb/devices"
    for dev in sorted(os.listdir(base)):
        def attr(what):
            try:
                with open(os.path.join(base, dev, what), encoding="ascii") as f:
                    return f.read().strip()
            except OSError:
                return ""
        vid = attr("idVendor").lower()
        if vid not in USB_PROGRAMMERS:
            continue
        found.append((vid, attr("idProduct"), attr("product") or "?",
                      attr("serial") or "(no serial)"))
    return found


def cmd_list(args):
    for kind in ("by-path", "by-id"):
        d = "/dev/serial/" + kind
        print("/dev/serial/%s:" % kind)
        if not os.path.isdir(d):
            print("  (none)")
            continue
        entries = sorted(os.listdir(d))
        if not entries:
            print("  (none)")
        for e in entries:
            print("  %-52s -> %s"
                  % (e, os.path.basename(os.path.realpath(os.path.join(d, e)))))
        print("")

    print("USB programmers:")
    progs = usb_programmers()
    if not progs:
        print("  (none)")
    for vid, pid, product, serial_no in progs:
        print("  %s:%s  %-24s %-14s serial %s"
              % (vid, pid, USB_PROGRAMMERS[vid], product, serial_no))
    print("")

    print("Manifest (tools/bench_boards.py):")
    if not manifest.BOARDS:
        print("  (empty)")
    for name, entry in sorted(manifest.BOARDS.items()):
        console = entry.get("console") or "(none)"
        mark = "ok     " if entry.get("console") and os.path.exists(console) \
               else "MISSING"
        prog = entry["programmer"]
        if prog["type"] == "serialupdi":
            how = "serialupdi %s @ %s" % (prog.get("port"), prog.get("baud"))
        else:
            how = prog["type"] + (" usb:%s" % prog["serial"]
                                  if prog.get("serial") else " (only one attached)")
        spec = BOARD_TYPES.get(entry["board"])
        origin = "%s/ via %s" % (spec["project"], spec["flash"]) if spec \
                 else "UNKNOWN TYPE"
        print("  %-4s type %-5s  id %-8s %s  console %s"
              % (name, entry["board"], entry.get("id") or "-", mark, console))
        print("       %-24s programmer %s" % (origin, how))
    return 0


# ---------------------------------------------------------------------------
#  flash
# ---------------------------------------------------------------------------

def avrdude_args(prog, mcu, hexfile, chip_erase=False):
    # THREE ERASE REGIMES, and only two of them are reachable from here.
    # MEASURED on this bench (AVR128DB48 over UPDI, avrdude 8.1), because the
    # option names invite exactly the wrong assumption:
    #
    #   default (what this tool does)  avrdude PAGE-ERASES each page it is
    #       about to write and leaves every other page of the part exactly as
    #       it was. A 12 KB app therefore costs ~24 page cycles out of the
    #       1000 the flash has (DS40002247B table 39-7, lowered from 10k
    #       "based on validation data") instead of the 256 a chip erase would
    #       spend, and anything living outside the image - an NvHeap's blocks
    #       and its map pages under FLASHEND (docs/design/nv-heap.md) -
    #       SURVIVES the reflash. The EEPROM is untouched regardless of
    #       EESAVE.
    #   -D  disables that erase ENTIRELY: the image is programmed into pages
    #       that were never erased, and programming can only clear bits. It is
    #       safe ONLY when the bytes already in the chip are the ones being
    #       written (reflashing an unchanged image); anything else silently
    #       ANDs the two images together and avrdude reports a verification
    #       mismatch. Not used here.
    #   -e  the real chip erase: every page, the EEPROM too (EESAVE clear on
    #       this bench). That is what --erase asks for and what it now passes;
    #       nothing else wipes an NvHeap.
    if prog["type"] == "serialupdi" and not os.path.exists(prog.get("port") or ""):
        die("serialupdi port %s does not exist (bench.py list)" % prog.get("port"))
    erase = ["-e"] if chip_erase else []
    return avrdude_base(prog, mcu) + erase + ["-U", "flash:w:%s:i" % hexfile]


def openocd_args(prog, elffile, target_cfg):
    """OpenOCD over SWD: the probe KIND is the manifest's ("openocd_cmsisdap"
    = an Atmel-ICE, "openocd_stlink" = a Nucleo's on-board ST-LINK), the
    target script the board TYPE's (BOARD_TYPES). Otherwise the same invocation
    samc/CMakeLists.txt's <app>-upload target uses - except that the probe
    is named by THE MANIFEST, not by the CMake cache. Identity is the
    manifest's concern (a second SAM board means a second probe), and this
    mirrors the AVR path, which calls avrdude itself rather than leaning on
    the project's upload target.

    `program ... verify` writes and reads back; the reset then leaves the
    chip RUNNING - the AVR path's end state too, so `run` right after
    `flash` means the same thing on both architectures. The at91samd
    flash driver probes the geometry from the DSU DID, so nothing here
    names a part.

    AND THEN DEBUG IS TURNED OFF AGAIN, which is not cosmetic. Attaching
    a probe sets DHCSR.C_DEBUGEN, and a core with halting debug enabled
    HALTS on a BKPT instead of faulting on it - so `break_here()`, which
    every panic() ends in, stops the board dead with no output instead of
    escalating to HardFault_Handler. Worse, it is sticky: table 18-1 of
    DS60001479M lists the debug logic as reset by a power-on or an
    external reset and NOT by a watchdog reset or a system reset request,
    so once a flash has enabled it every later software reset inherits
    it. Measured the hard way - a suite that reached a panic simply went
    quiet, and the halted core was found parked on the BKPT.

    The write below is DHCSR with its 0xA05F key and every control bit
    zero, which is what a board with no probe attached looks like.

    THE HID BACKEND IS NOT COSMETIC EITHER (every OpenOCD invocation in
    this file carries it). Behind the desk's USB hub (2026-09-02) the
    Atmel-ICE's default usb_bulk transport desynchronizes by one packet
    under sustained traffic - every command then receives the PREVIOUS
    command's response ("CMSIS-DAP command mismatch ... received 0x12"),
    programming dies mid-flash, and the state survives soft recovery
    attempts; the HID transport ran the same flashes clean on the first
    try. A wedged probe is recovered by two USBDEVFS_RESET ioctls five
    seconds apart (or a replug)."""
    argv = [manifest.OPENOCD] + openocd_interface(prog)
    argv += ["-f", target_cfg,
             "-c", "program %s verify" % elffile,
             "-c", "reset run",
             "-c", "mww 0xE000EDF0 0xA05F0000",
             "-c", "exit"]
    return argv


def openocd_interface(prog):
    """The `-f interface/... -c adapter serial ...` half of an OpenOCD
    command line, from the manifest's programmer entry. The HID backend
    is the Atmel-ICE's (see openocd_args); an ST-LINK has one transport
    and is addressed by its real USB serial."""
    kind = prog["type"]
    if kind == "openocd_cmsisdap":
        argv = ["-f", "interface/cmsis-dap.cfg", "-c", "cmsis-dap backend hid"]
    elif kind == "openocd_stlink":
        argv = ["-f", "interface/stlink.cfg"]
    else:
        die("programmer type '%s' is not an OpenOCD probe" % kind)
    if prog.get("serial"):
        argv += ["-c", "adapter serial %s" % prog["serial"]]
    return argv


def cmd_flash(args):
    info, btype = resolve_app(args.name, args.app)
    spec = board_type(btype)
    project, preset = spec["project"], spec["preset"]
    # Refuse an impossible request BEFORE spending a build on it.
    if args.erase and spec["flash"] != "avrdude":
        die("--erase is an avrdude option: on an OpenOCD target, `program "
            "... verify` erases exactly the sectors it writes, and there is "
            "no NvHeap on the SAM or the STM32 yet to protect from a chip "
            "erase")
    print("bench: board %s -> %s/ preset %s, target %s"
          % (args.name, project, preset, args.app))
    # Cheap (a fresh roster too, in case an app's "// build:" lines or the
    # app list itself changed since the last configure). Presets resolve
    # against their own CMakePresets.json, so cmake runs from the owning
    # project's directory - the repo root is not a CMake project.
    projdir = os.path.join(ROOT, project)
    rc = subprocess.call(["cmake", "--preset", preset], cwd=projdir)
    if rc != 0:
        return rc
    rc = subprocess.call(["cmake", "--build", "--preset", preset,
                          "--target", args.app], cwd=projdir)
    if rc != 0:
        return rc

    prog = board_entry(args.name)["programmer"]
    if spec["flash"] == "openocd":
        elffile = os.path.join("build-cmake", preset, args.app + ".elf")
        if not os.path.isfile(os.path.join(ROOT, elffile)):
            die("no %s after the build" % elffile)
        argv = openocd_args(prog, elffile, spec["target_cfg"])
    else:
        hexfile = os.path.join("build-cmake", preset, args.app + ".hex")
        if not os.path.isfile(os.path.join(ROOT, hexfile)):
            die("no %s after the build" % hexfile)
        nvheap_preflight(prog, spec["mcu"], hexfile, chip_erase=args.erase)
        argv = avrdude_args(prog, spec["mcu"], hexfile, chip_erase=args.erase)

    print("bench: " + " ".join(argv))
    rc = subprocess.call(argv, cwd=ROOT)
    if rc == 0:
        state_write(args.name, args.app)
    return rc


# ---------------------------------------------------------------------------
#  NvHeap preflight: what is about to be overwritten
#
#  A flash NvHeap (util/nv_heap.hpp, docs/design/nv-heap.md) keeps its map in
#  the last erase units of the part and its blocks in the free flash between
#  and above the image's sections. Nothing in the toolchain knows that: the
#  linker places code and read-only data wherever they fit, and the first sign
#  that a grown image has landed on a stored block is the loss report at the
#  next mount.
#
#  So before writing, read the chip, find the current map version if there is
#  one, and say plainly which stored blocks the new image would take down.
#  This WARNS AND NEVER BLOCKS - it is information, not a policy, and the
#  application is the one that decides whether losing a table matters. When
#  the chip holds no valid map (the usual case) it says nothing at all.
#
#  The map layout below mirrors util/nv_heap.hpp and must move with it. Two of
#  the heap's template parameters are not recorded in the map, so both are
#  searched rather than assumed: the rotation is looked for in the last few
#  pages, and the entry-table width is whatever makes the checksum come out.
# ---------------------------------------------------------------------------

NVHEAP_MAGIC = 0x5048564E          # "NVHP", little-endian
NVHEAP_FORMAT = 1
NVHEAP_HEADER = 16
NVHEAP_ENTRY = 14
NVHEAP_PAGE = 512                  # the erase unit of every AVR128DA/DB
NVHEAP_SEARCH_PAGES = 8            # map_pages is a template parameter: look
NVHEAP_MAX_BLOCKS = 35             # ... and so is max_blocks (35 fills a page)


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 \
                else (crc << 1) & 0xFFFF
    return crc


def nvheap_version(page):
    """The map version in this page, or None. Returns (seq, entries) with
    entries as (record_id, first_page, size_pages, payload_len)."""
    if len(page) < NVHEAP_HEADER + NVHEAP_ENTRY + 2:
        return None
    magic, fmt, count = (int.from_bytes(page[0:4], "little"), page[4], page[5])
    if magic != NVHEAP_MAGIC or fmt != NVHEAP_FORMAT:
        return None
    for blocks in range(1, NVHEAP_MAX_BLOCKS + 1):
        size = NVHEAP_HEADER + NVHEAP_ENTRY * blocks + 2
        if size > len(page):
            break
        if count > blocks:
            continue          # this table is too narrow to hold that count
        if crc16_ccitt(page[:size - 2]) != int.from_bytes(page[size - 2:size],
                                                          "little"):
            continue
        entries = []
        for i in range(count):
            at = NVHEAP_HEADER + i * NVHEAP_ENTRY
            entries.append((int.from_bytes(page[at:at + 2], "little"),
                            int.from_bytes(page[at + 2:at + 4], "little"),
                            int.from_bytes(page[at + 4:at + 6], "little"),
                            int.from_bytes(page[at + 6:at + 10], "little")))
        return int.from_bytes(page[8:12], "little"), entries
    return None


def nvheap_current(flash):
    """The live blocks of the newest map version in the chip, or None."""
    best = None
    for i in range(1, NVHEAP_SEARCH_PAGES + 1):
        base = len(flash) - i * NVHEAP_PAGE
        if base < 0:
            break
        found = nvheap_version(flash[base:base + NVHEAP_PAGE])
        if found and (best is None or found[0] > best[0]):
            best = found
    return best


def hex_pages(path):
    """The flash PAGES an Intel HEX file writes - which is exactly what
    avrdude erases. Handles both extended-address record types; an AVR image
    is two chunks with a hole between them, so the page set matters and the
    span from zero would be a lie."""
    pages = set()
    base = 0
    with open(path, encoding="ascii") as f:
        for line in f:
            line = line.strip()
            if not line.startswith(":"):
                continue
            raw = bytes.fromhex(line[1:])
            count, offset, kind = raw[0], (raw[1] << 8) | raw[2], raw[3]
            if kind == 0:
                start = base + offset
                for page in range(start // NVHEAP_PAGE,
                                  (start + count - 1) // NVHEAP_PAGE + 1):
                    pages.add(page)
            elif kind == 2:
                base = ((raw[4] << 8) | raw[5]) << 4
            elif kind == 4:
                base = ((raw[4] << 8) | raw[5]) << 16
    return pages


def read_flash(prog, mcu):
    """The whole flash of the chip, or None if it cannot be read."""
    out = os.path.join(ROOT, "build-cmake", "bench_flash.bin")
    try:
        os.makedirs(os.path.dirname(out), exist_ok=True)
        argv = avrdude_base(prog, mcu) + ["-q", "-q",
                                          "-U", "flash:r:%s:r" % out]
        if subprocess.call(argv, cwd=ROOT) != 0:
            return None
        with open(out, "rb") as f:
            return f.read()
    except OSError:
        return None


def nvheap_preflight(prog, mcu, hexfile, chip_erase=False):
    """Warn about stored blocks this flash would destroy. Never blocks."""
    flash = read_flash(prog, mcu)
    if flash is None:
        return
    # avrdude trims trailing erased bytes off a raw read.
    size = NVHEAP_PAGE * ((len(flash) + NVHEAP_PAGE - 1) // NVHEAP_PAGE)
    flash = flash.ljust(size, b"\xff")
    current = nvheap_current(flash)
    if current is None:
        return                       # no heap in this chip: nothing to say
    seq, entries = current
    if not entries:
        return
    if chip_erase:
        print("bench: WARNING - the chip holds an NvHeap map (seq %d) with %d "
              "live block(s): %s" %
              (seq, len(entries), ", ".join("id 0x%04X" % e[0] for e in entries)))
        print("bench:           --erase wipes the whole flash: ALL of them go, "
              "map included.")
        return
    pages = hex_pages(os.path.join(ROOT, hexfile))
    hit = [e for e in entries
           if any(p in pages for p in range(e[1], e[1] + e[2]))]
    if hit:
        print("bench: WARNING - this image lands on %d stored block(s) of the "
              "NvHeap map (seq %d):" % (len(hit), seq))
        for rid, first, span, length in hit:
            print("bench:           id 0x%04X at 0x%05X, %d page(s), %d bytes"
                  % (rid, first * NVHEAP_PAGE, span, length))
        print("bench:           they will fail their checksum at the next "
              "mount and be reported lost.")
    else:
        print("bench: preflight - %d live NvHeap block(s) (map seq %d), none "
              "in this image's pages." % (len(entries), seq))


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


def avrdude_base(prog, mcu):
    argv = [manifest.AVRDUDE, "-p", mcu, "-c", prog["type"]]
    if prog["type"] == "serialupdi":
        port = prog.get("port")
        if not port:
            die("serialupdi programmer without a 'port' in the manifest")
        argv += ["-P", port, "-b", str(prog.get("baud", 230400))]
    elif prog.get("serial"):
        argv += ["-P", "usb:" + prog["serial"]]
    return argv


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
#     direction. brio's own samc/nvm.hpp makes the same ruling from the
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

DHCSR = 0xE000EDF0               # the debug register bench.py always leaves off
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
# interpolation neither this tool nor samc/supc.hpp makes.
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
# The bit positions are table 9-4's and are the same fields samc/nvm.hpp's
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
# change (samc/supc.hpp's BodCore makes the same ruling by having no setter).
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
    "bench.py: SAM_FUSES + SAM_PRESERVED must tile bits 0..63 of table 9-4"


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


# ---------------------------------------------------------------------------
#  console protocol (the bench suites: single-char commands, "> " prompt)
# ---------------------------------------------------------------------------

def open_console_at(name, speed):
    return serial.Serial(console_path(name), speed, timeout=0.05)


def wake(ser, settle=0.4):
    """Newline, then swallow whatever the board says back: the suite reprints
    its prompt and we start the capture from a known-empty line."""
    ser.reset_input_buffer()
    ser.write(b"\n")
    ser.flush()
    deadline = time.time() + settle
    while time.time() < deadline:
        if ser.read(256):
            deadline = time.time() + settle
    ser.reset_input_buffer()


def capture(ser, marker, timeout, echo=True):
    """Read until the marker line has been seen AND the prompt is back.

    MIND THE MARKER'S TAIL: the stop condition is text.endswith(PROMPT),
    checked as soon as the marker has been seen - so a marker whose own
    text ends with the prompt string can stop the capture EARLY,
    truncated mid-line. The concrete trap: --expect="->" for a single
    letter's "  -> N pass" tally ("-> " ends with "> ", the prompt), so
    a read boundary right after the arrow ends the capture before the
    tally. Prefer a marker with no "> " suffix - "fail" matches every
    tally line and is safe."""
    text = ""
    seen = marker == ""
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        piece = chunk.decode("ascii", "replace")
        text += piece
        if echo:
            sys.stdout.write(piece)
            sys.stdout.flush()
        if marker and marker in text:
            seen = True
        if seen and text.endswith(PROMPT):
            return text, True
    return text, False


def wait_prompt(ser, timeout):
    return capture(ser, "", timeout, echo=False)[1]


def send(ser, command):
    ser.write(command.encode("ascii"))
    ser.flush()


def verdict(text, marker):
    """(failures, summary line). The 'ALL:' line wins; without one, the per-test
    '-> N pass, M fail' lines are summed."""
    summary = None
    for line in text.splitlines():
        if marker and marker in line and SUMMARY_RE.search(line):
            summary = line.strip()
    if summary:
        m = SUMMARY_RE.search(summary)
        return int(m.group(2)), summary
    total_p = total_f = 0
    for m in SUMMARY_RE.finditer(text):
        total_p += int(m.group(1))
        total_f += int(m.group(2))
    if total_p or total_f:
        return total_f, "%d pass, %d fail" % (total_p, total_f)
    return 0, None


def cmd_run(args):
    name, command, marker = args.name, args.command, args.expect
    speed = speed_for(name, args.app)
    print("bench: board %s console %s @ %d, command '%s'"
          % (name, console_path(name), speed, command))
    with open_console_at(name, speed) as ser:
        wake(ser)
        send(ser, command)
        text, complete = capture(ser, marker, args.timeout)
    print("")
    if not complete:
        die("timed out after %.0f s waiting for '%s' + prompt on board %s"
            % (args.timeout, marker, name))
    fails, summary = verdict(text, marker)
    if summary:
        print("bench: %s" % summary)
    if fails:
        die("board %s reported %d failure(s)" % (name, fails))
    return 0


# ---------------------------------------------------------------------------
#  console / duo
# ---------------------------------------------------------------------------

def cmd_console(args):
    print("%s %d" % (console_path(args.name), speed_for(args.name, args.app)))
    return 0


def cmd_duo(args):
    """The campaign shape: an instrument peer set up by a script, then the DUT
    driven and judged.

    UNTESTED, PENDING HARDWARE: with a single board on the desk (2026-08-20)
    this path has never run. It is written to the same console protocol as
    `run`, which is exercised daily.
    """
    dut_name, dut_cmd = split_pair(args.dut, "dut")
    ins_name, script = split_pair(args.instrument, "instrument")
    if not os.path.isfile(script):
        die("instrument script %s not found" % script)
    with open(script, encoding="ascii") as f:
        steps = [ln.strip() for ln in f
                 if ln.strip() and not ln.strip().startswith("#")]

    ins_speed = speed_for(ins_name, args.instrument_app)
    dut_speed = speed_for(dut_name, args.app)

    with open_console_at(ins_name, ins_speed) as ins:
        print("bench: instrument %s <- %s (%d step(s))" % (ins_name, script, len(steps)))
        wake(ins)
        for step in steps:
            send(ins, step)
            if not wait_prompt(ins, args.timeout):
                die("instrument %s did not answer step '%s'" % (ins_name, step))
        with open_console_at(dut_name, dut_speed) as dut:
            print("bench: dut %s <- '%s'" % (dut_name, dut_cmd))
            wake(dut)
            send(dut, dut_cmd)
            text, complete = capture(dut, args.expect, args.timeout)
    print("")
    if not complete:
        die("timed out waiting for '%s' + prompt on board %s" % (args.expect, dut_name))
    fails, summary = verdict(text, args.expect)
    if summary:
        print("bench: %s" % summary)
    if fails:
        die("board %s reported %d failure(s)" % (dut_name, fails))
    return 0


def split_pair(text, what):
    if ":" not in text:
        die("%s argument must be <board>:<value>, got '%s'" % (what, text))
    name, value = text.split(":", 1)
    if not name or not value:
        die("%s argument must be <board>:<value>, got '%s'" % (what, text))
    return name, value


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        prog="bench.py",
        description="Multi-board bench orchestrator (manifest: tools/bench_boards.py)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("list", help="serial devices, USB programmers, manifest")
    p.set_defaults(func=cmd_list)

    p = sub.add_parser("flash", help="build an app for a board and flash it over UPDI")
    p.add_argument("name", help="board name from the manifest")
    p.add_argument("app", help="app name (src/apps/<app>.cpp)")
    p.add_argument("--erase", action="store_true",
                   help="real chip erase first, EEPROM included - the only "
                        "way to wipe a flash heap (the default erases just "
                        "the pages of the image: 1k-cycle flash)")
    p.set_defaults(func=cmd_flash)

    p = sub.add_parser("run", help="send a console command and judge the summary")
    p.add_argument("name")
    p.add_argument("command", help="the console command (usually one char)")
    p.add_argument("--app", help="app whose monitor_speed to use "
                                 "(default: the manifest's DEFAULT_MONITOR_SPEED)")
    p.add_argument("--expect", default="ALL:", help="marker line ending the capture")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("fuses",
                       help="read (and optionally write) a board's fuses - "
                            "AVR-Dx FUSE bytes over UPDI, SAM C21 NVM User "
                            "Row over SWD, by board type")
    p.add_argument("name", help="board name from the manifest")
    p.add_argument("assignment", nargs="*", metavar="name=value",
                   help="fields to write first, e.g. bootsize=128 codesize=0 "
                        "on AVR-Dx, bodvdd_hysteresis=1 on SAM C21")
    p.add_argument("--i-know-what-this-does", action="store_true",
                   help="SAM only: allow the guarded fields (bootprot, lock, "
                        "and setting wdt_always_on) - each can leave a board "
                        "that no longer takes firmware the ordinary way")
    p.add_argument("--rewrite", action="store_true",
                   help="SAM only: erase and write the user row back even "
                        "when no field changes - the end-to-end proof of the "
                        "EAR/WAP path at zero risk")
    p.set_defaults(func=cmd_fuses)

    p = sub.add_parser("console", help="print the console device path and speed")
    p.add_argument("name")
    p.add_argument("--app", help="app whose monitor_speed to report")
    p.set_defaults(func=cmd_console)

    p = sub.add_parser("duo", help="script an instrument peer, then run the DUT")
    p.add_argument("dut", help="<board>:<command>")
    p.add_argument("instrument", help="<board>:<script-file>")
    p.add_argument("--app", help="DUT app whose monitor_speed to use")
    p.add_argument("--instrument-app", help="instrument app whose monitor_speed to use")
    p.add_argument("--expect", default="ALL:")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    p.set_defaults(func=cmd_duo)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
