#!/usr/bin/env python3
# ============================================================================
#  bench.py - the multi-board bench ORCHESTRATOR: build, flash, drive.
#
#  RUN IT WITH PLATFORMIO'S PYTHON (it is the interpreter that has pyserial and
#  the `pio` command next to it):
#
#      ~/.platformio/penv/bin/python tools/bench.py list
#      ~/.platformio/penv/bin/python tools/bench.py flash A test_avr_pin
#      ~/.platformio/penv/bin/python tools/bench.py run A a
#      ~/.platformio/penv/bin/python tools/bench.py console A
#      ~/.platformio/penv/bin/python tools/bench.py duo A:a B:scripts/peer.txt
#      ~/.platformio/penv/bin/python tools/bench.py fuses A
#      ~/.platformio/penv/bin/python tools/bench.py fuses A bootsize=128 codesize=0
#
#  Three concerns, kept apart on purpose:
#    1. BUILD   - one PlatformIO env per app x board TYPE (apps.ini, generated
#                 by tools/gen_apps.py). Never an env per physical board.
#    2. IDENTITY- tools/bench_boards.py, the bench manifest: which board sits
#                 where, on which console, behind which programmer.
#    3. THIS    - resolves 1 against 2 and drives the hardware.
#
#  A board NAME ("A", "B") is a desk position from the manifest. The consoles
#  are OBSERVABILITY ONLY: firmware always goes in over UPDI (avrdude), the
#  console only carries the suite's verdicts back.
#
#  The campaign shape: board A = the DUT running a test suite, board B = a
#  scriptable instrument peer (clock stretching, NACK injection, arbitration).
#  `duo` is that shape; with one board on the desk it cannot yet be exercised.
# ============================================================================
import argparse
import configparser
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
from gen_apps import BOARDS, env_name    # noqa: E402

try:
    import serial                        # pyserial
except ImportError:
    sys.exit("bench: pyserial is missing - run this with "
             "~/.platformio/penv/bin/python tools/bench.py ...")

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


def apps_config():
    cfg = configparser.RawConfigParser()
    read = cfg.read([os.path.join(ROOT, "apps.ini")])
    if not read:
        die("apps.ini not found - run python tools/gen_apps.py")
    return cfg


def resolve_env(name, app):
    """(env, board type) for an app on a physical board; a clear error if the
    app was never built for that board type (no '// pio: boards' line)."""
    entry = board_entry(name)
    btype = entry["board"]
    if btype not in BOARDS:
        die("board '%s' has an unknown board type '%s' (known: %s)"
            % (name, btype, ", ".join(sorted(BOARDS))))
    env = env_name(app, btype)
    cfg = apps_config()
    if not cfg.has_section("env:" + env):
        hint = ("" if btype == "db48" else
                " - add a '// pio: boards = %s' line to src/apps/%s.cpp "
                "and re-run tools/gen_apps.py" % (btype, app))
        die("no env '%s' for app '%s' on board '%s' (type %s)%s"
            % (env, app, name, btype, hint))
    return env, btype


def env_option(env, option, default=None):
    cfg = apps_config()
    section = "env:" + env
    if cfg.has_option(section, option):
        return cfg.get(section, option)
    return default


def env_board_json(env):
    """The board JSON of an env: its own 'board' line, else [env] in
    platformio.ini (the default board of the whole project)."""
    board = env_option(env, "board")
    if board is None:
        pio = configparser.RawConfigParser()
        pio.read([os.path.join(ROOT, "platformio.ini")])
        board = pio.get("env", "board", fallback=None)
    if board is None:
        die("cannot tell which board env '%s' builds for" % env)
    path = os.path.join(ROOT, "boards", board + ".json")
    if not os.path.isfile(path):
        die("board file %s not found" % path)
    with open(path, encoding="ascii") as f:
        return json.load(f)


def env_speed(env):
    return int(env_option(env, "monitor_speed", manifest.DEFAULT_MONITOR_SPEED))


# Which app was last flashed onto which board: written by `flash`, read by the
# console commands so they open the port at that app's monitor_speed without
# being told again. It is a convenience cache, not identity - --app overrides
# it, and a board never flashed by this tool falls back to the manifest's
# DEFAULT_MONITOR_SPEED.
STATE = os.path.join(ROOT, ".pio", "bench_last.json")


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
    env, _ = resolve_env(name, app)
    return env_speed(env)


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
        print("  %-4s type %-5s  id %-8s %s  console %s"
              % (name, entry["board"], entry.get("id") or "-", mark, console))
        print("       programmer %s" % how)
    return 0


# ---------------------------------------------------------------------------
#  flash
# ---------------------------------------------------------------------------

def pio_exe():
    exe = os.path.join(os.path.dirname(sys.executable), "pio")
    return exe if os.path.isfile(exe) else "pio"


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


def cmd_flash(args):
    env, _ = resolve_env(args.name, args.app)
    print("bench: board %s -> env %s" % (args.name, env))
    rc = subprocess.call([pio_exe(), "run", "-e", env], cwd=ROOT)
    if rc != 0:
        return rc
    mcu = env_board_json(env)["build"]["mcu"]
    hexfile = os.path.join(".pio", "build", env, "firmware.hex")
    if not os.path.isfile(os.path.join(ROOT, hexfile)):
        die("no %s after the build" % hexfile)
    nvheap_preflight(board_entry(args.name)["programmer"], mcu, hexfile,
                     chip_erase=args.erase)
    argv = avrdude_args(board_entry(args.name)["programmer"], mcu, hexfile,
                        chip_erase=args.erase)
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
    out = os.path.join(ROOT, ".pio", "bench_flash.bin")
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
#  them (the CPU can read them and nothing more - DS40002247B 11.3.1.5). They
#  therefore belong here, next to the manifest that says which chip is which,
#  and not in an env.
#
#  The names are avrdude's own memory names, which are also the data sheet's
#  register names in FUSE. The values are bytes: decimal, or 0x-prefixed.
#  Every write is read back and reported, because a fuse written wrong is a
#  board that no longer boots the way its firmware expects.
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
    # The MCU of the board TYPE: fuses are the chip's, so the board JSON of
    # any env for that type answers it. env_board_json wants an env, and
    # every board type has at least family_probe; take the type's board file
    # directly instead.
    board = BOARDS[entry["board"]]
    path = os.path.join(ROOT, "boards", board + ".json")
    if not os.path.isfile(path):
        die("board file %s not found" % path)
    with open(path, encoding="ascii") as f:
        mcu = json.load(f)["build"]["mcu"]

    writes = {}
    for spec in args.assignment:
        if "=" not in spec:
            die("a fuse assignment is <name>=<value>, got '%s'" % spec)
        name, value = spec.split("=", 1)
        name = name.strip().lower()
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
    """Read until the marker line has been seen AND the prompt is back."""
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

    p = sub.add_parser("fuses", help="read (and optionally write) a board's fuses")
    p.add_argument("name", help="board name from the manifest")
    p.add_argument("assignment", nargs="*", metavar="name=value",
                   help="fuses to write first, e.g. bootsize=128 codesize=0")
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
