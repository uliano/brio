"""brio flash: build an app for a board and put it in, by the board type -
avrdude over UPDI, OpenOCD over SWD or an ST-LINK, or the ST-LINK's own
mass-storage flasher - with the flash-heap preflight on the AVR."""

import argparse
import json
import os
import re
import subprocess
import sys
import time

from cli.bench.common import *   # noqa: F401,F403


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
        die("serialupdi port %s does not exist (brio list)" % prog.get("port"))
    erase = ["-e"] if chip_erase else []
    return avrdude_base(prog, mcu) + erase + ["-U", "flash:w:%s:i" % hexfile]


def openocd_args(prog, elffile, target_cfg):
    """OpenOCD over SWD: the probe KIND is the manifest's ("openocd_cmsisdap"
    = an Atmel-ICE, "openocd_stlink" = a Nucleo's on-board ST-LINK), the
    target script the board TYPE's (BOARD_TYPES). Otherwise the same invocation
    samc21/CMakeLists.txt's <app>-upload target uses - except that the probe
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
             "-c", "program %s verify" % elffile]
    if "stm32g0" in target_cfg:
        # AND THE DEBUG-IN-STOP BITS ARE TAKEN BACK DOWN, AFTER THE LAST
        # RESET. OpenOCD's own target/stm32g0x.cfg sets DBGMCU_CR.DBG_STOP
        # | DBG_STANDBY in its examine-end hook ("enable debug during low
        # power modes"), and the target is re-examined after every reset
        # OpenOCD issues - so the bits come back on the `reset` that
        # `program` ends with, whatever was cleared before it (measured:
        # cleared to 0 while halted, read 6 again right after `reset
        # run`). With them set HCLK, SysTick and the whole VCORE domain
        # keep running inside a Stop, and DBGMCU_CR survives every reset
        # but a power-on. Measured 2026-09-03: with the bit set a Stop
        # entered with the kernel tick armed lasts 1 ms, with it clear
        # the same Stop lasts its full 250 - and the difference had been
        # recorded as a silicon fact by a whole campaign. So the board is
        # reset and HALTED at its reset vector (no firmware has run, every
        # RCC gate is at its reset value), the DBGMCU's gate is opened,
        # DBGMCU_CR cleared, the gate put back, and only then does the
        # core run: it leaves the bench exactly as a probe-less power-on
        # would leave it.
        argv += ["-c", "reset halt",
                 "-c", "mmw 0x4002103C 0x08000000 0",
                 "-c", "mww 0x40015804 0x00000000",
                 "-c", "mmw 0x4002103C 0 0x08000000",
                 "-c", "resume"]
    else:
        argv += ["-c", "reset run"]
    argv += ["-c", "mww 0xE000EDF0 0xA05F0000",
             "-c", "exit"]
    return argv


def wch_openocd_args(prog, elffile):
    """WCH's OpenOCD fork over a WCH-Link (programmer type "wch_link"):
    the only OpenOCD that speaks the probe's SDI transport, a separate
    binary from the SWD paths' 0.12.0 release, named by the manifest's
    WCH_OPENOCD. Its target script is the vendor's own wch-riscv.cfg,
    which lives beside the binary and not in a scripts tree. The same
    invocation ch32v00x/CMakeLists.txt's <app>-upload target uses.

    `program ... verify` then `reset run` leaves the chip RUNNING, the
    end state of every other path. No debug-enable is taken back down
    afterwards: `break_here()` on this core is an ebreak, which with no
    debugger attached escalates to the fault vector rather than halting
    the core, so a probe-less power-on and a just-flashed board behave
    the same way. Nothing here names the probe: the fork's adapter
    selection is untested with two WCH-Links attached, and the desk has
    one."""
    if prog["type"] != "wch_link":
        die("programmer type '%s' is not a WCH-Link" % prog["type"])
    openocd = getattr(manifest, "WCH_OPENOCD", "/sw/wch-openocd/bin/openocd")
    cfg = os.path.join(os.path.dirname(openocd), "wch-riscv.cfg")
    return [openocd, "-f", cfg,
            "-c", "program %s verify" % elffile,
            "-c", "reset run",
            "-c", "exit"]


def openocd_interface(prog):
    """The `-f interface/... -c adapter serial ...` half of an OpenOCD
    command line, from the manifest's programmer entry. The HID backend
    is the Atmel-ICE's (see openocd_args); an ST-LINK has one transport
    and is addressed by its real USB serial."""
    kind = prog["type"]
    if kind == "openocd_cmsisdap":
        argv = ["-f", "interface/cmsis-dap.cfg", "-c", "cmsis_dap_backend hid"]
    elif kind == "openocd_stlink":
        argv = ["-f", "interface/stlink.cfg"]
    else:
        die("programmer type '%s' is not an OpenOCD probe" % kind)
    if prog.get("serial"):
        argv += ["-c", "adapter serial %s" % prog["serial"]]
    return argv


def msd_flash(prog, binfile, app):
    """THE ST-LINK'S OWN FLASHER. A Nucleo's ST-LINK/V2.1 also enumerates as
    a small mass-storage drive (NODE_<part>), and a .bin dropped on it is
    programmed at 0x08000000 by the probe's firmware - connect under reset,
    program, verify, reset - after which the drive re-enumerates and a
    FAIL.TXT is left on it when anything went wrong (measured on position
    G: a garbage file draws "The application file format is unknown", a
    good image leaves no FAIL.TXT and removes a stale one). The proof an
    image is running is its banner on the console, which is what `run`
    reads anyway.

    This kind exists because a Nucleo-32's debug port can go silent until
    the board is replugged (the manifest carries the record): the
    MSD is then the one way in,
    and it is also why nothing here halts the core, clears DBGMCU_CR or
    reads anything back over SWD - the debug-in-Stop bits that openocd_args
    takes down are set only by an OpenOCD examine, which never happens on
    this path. The drive is mounted through udisks, no root needed."""
    label = prog.get("label")
    if not label:
        die("programmer type stlink_msd needs a \"label\" (the drive's volume "
            "label, e.g. NODE_G031K8)")
    dev = os.path.realpath(os.path.join("/dev/disk/by-label", label))
    if not os.path.exists(dev):
        die("no drive labelled %s - is the board plugged in? (brio list)"
            % label)
    block = os.path.basename(dev)

    def mounted_at():
        out = subprocess.run(["udisksctl", "mount", "-b", dev],
                             capture_output=True, text=True)
        text = (out.stdout + out.stderr).strip()
        m = re.search(r" at (\S+)", text)
        if m:
            return m.group(1).rstrip(".")
        out = subprocess.run(["findmnt", "-n", "-o", "TARGET", dev],
                             capture_output=True, text=True)
        if out.stdout.strip():
            return out.stdout.strip()
        die("cannot mount %s (%s): %s" % (label, dev, text))

    def unmount():
        subprocess.run(["udisksctl", "unmount", "-b", dev],
                       capture_output=True, text=True)

    def drive_size():
        try:
            with open("/sys/class/block/%s/size" % block, encoding="ascii") as f:
                return int(f.read().strip() or "0")
        except OSError:
            return 0

    mnt = mounted_at()
    target = os.path.join(mnt, app + ".bin")
    size = os.path.getsize(binfile)
    with open(binfile, "rb") as src, open(target, "wb") as dst:
        dst.write(src.read())
        dst.flush()
        os.fsync(dst.fileno())
    os.sync()
    unmount()
    print("bench: %s -> %s (%d bytes) via the ST-LINK's mass-storage flasher"
          % (os.path.basename(binfile), label, size))
    # The probe programs, resets the target and re-enumerates the drive
    # (its size reads 0 for a moment): wait for that cycle, bounded.
    t0 = time.time()
    seen_zero = False
    while time.time() - t0 < 20.0:
        if drive_size() == 0:
            seen_zero = True
        elif seen_zero:
            break
        time.sleep(0.2)
    if not seen_zero:
        print("bench: MSD flash FAILED - the drive never re-enumerated, so the "
              "probe did not process %s.bin (measured: a processed file, good "
              "or bad, cycles the drive within a few seconds)" % app)
        return 1
    time.sleep(1.0)
    for attempt in range(5):
        try:
            mnt = mounted_at()
            break
        except SystemExit:
            if attempt == 4:
                raise
            time.sleep(2.0)
    fail = os.path.join(mnt, "FAIL.TXT")
    reason = None
    if os.path.isfile(fail):
        with open(fail, encoding="ascii", errors="replace") as f:
            reason = f.read().strip()
    unmount()
    if reason is not None:
        print("bench: MSD flash FAILED - the probe says: %s" % reason)
        return 1
    print("bench: MSD flash done, no FAIL.TXT - the target was reset by the "
          "probe and is running %s (NB no SWD on this position: DBGMCU_CR is "
          "not touched and nothing is read back)" % app)
    return 0


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
    if prog["type"] == "stlink_msd":
        binfile = os.path.join(ROOT, "build-cmake", preset, args.app + ".bin")
        if not os.path.isfile(binfile):
            die("no %s after the build" % binfile)
        rc = msd_flash(prog, binfile, args.app)
        if rc == 0:
            state_write(args.name, args.app)
        return rc
    if spec["flash"] == "openocd":
        elffile = os.path.join("build-cmake", preset, args.app + ".elf")
        if not os.path.isfile(os.path.join(ROOT, elffile)):
            die("no %s after the build" % elffile)
        argv = openocd_args(prog, elffile, spec["target_cfg"])
    elif spec["flash"] == "wch_openocd":
        elffile = os.path.join("build-cmake", preset, args.app + ".elf")
        if not os.path.isfile(os.path.join(ROOT, elffile)):
            die("no %s after the build" % elffile)
        argv = wch_openocd_args(prog, elffile)
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


