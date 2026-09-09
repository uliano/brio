# ============================================================================
#  cli/bench/verbs.py - the bench verbs of brio: build, flash, drive.
#  The command is bin/brio; this module is their argparse front.
#
#  Run with any Python 3 that has pyserial installed (pip install --user
#  pyserial):
#
#      brio list
#      brio flash A test_avr_pin
#      brio run A a
#      brio console A
#      brio duo A:a B:scripts/peer.txt
#      brio fuses A
#      brio fuses A bootsize=128 codesize=0
#      brio fuses C
#      brio fuses C bodvdd_hysteresis=1
#
#  Three concerns, kept apart on purpose:
#    1. BUILD   - one CMake target per app x board TYPE, auto-discovered from
#                 each app's own "// build: boards = ..." header comment
#                 (each project's CMakeLists.txt); a configure also (re)writes
#                 that project's build-cmake/apps_<project>.json, which this
#                 file reads. Never a target per physical board.
#    2. IDENTITY- the bench manifest (cli/bench/manifest.py says where it is
#                 loaded from): which board sits where, on which console,
#                 behind which programmer.
#    3. THIS    - resolves 1 against 2 and drives the hardware.
#
#  TWO ARCHITECTURES SHARE THIS TOOL. The board's TYPE decides everything
#  that differs (BOARD_TYPES below): db* is an AVR-Dx built by avrdx/ and
#  written by avrdude over UPDI, c21j is a SAM C21 built by samc21/ and written
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
import argparse
import json
import os
import re
import subprocess
import sys
import time

from cli.bench.common import *            # noqa: F401,F403
from cli.bench.flash import cmd_flash     # noqa: E402
from cli.bench.fuses import cmd_fuses     # noqa: E402
from cli.bench.console import cmd_run, cmd_console, cmd_duo   # noqa: E402


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

    print("Manifest (%s):" % os.path.relpath(manifest_path(), ROOT))
    if not manifest.BOARDS:
        print("  (empty)")
    for name, entry in sorted(manifest.BOARDS.items()):
        console = entry.get("console") or "(none)"
        mark = "ok     " if entry.get("console") and os.path.exists(console) \
               else "MISSING"
        prog = entry["programmer"]
        if prog["type"] == "serialupdi":
            how = "serialupdi %s @ %s" % (prog.get("port"), prog.get("baud"))
        elif prog["type"] == "stlink_msd":
            how = "stlink_msd usb:%s drive %s (the probe's own flasher; no SWD)" \
                  % (prog.get("serial"), prog.get("label"))
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

# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        prog="brio",
        description="Multi-board bench orchestrator (the manifest: cli/bench/manifest.py)")
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

