"""What every verb needs: the manifest, the board types, the app rosters,
the console paths, the small helpers. Nothing here drives hardware."""

import argparse
import json
import os
import re
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cli.bench.manifest import load as _load_manifest, path as manifest_path   # noqa: E402
manifest = _load_manifest()

try:
    import serial                        # pyserial
except ImportError:
    sys.exit("bench: pyserial is missing - run this with a Python 3 that has "
             "it installed (pip install --user pyserial)")

# BOARD TYPE -> everything that follows from it. A board type names a chip
# package, and the chip package is what decides which of the sibling CMake
# projects builds for it, which release preset that project uses, which
# app roster to read, and how firmware gets in. Mirrors avrdx/cmake/
# avr-mcus.cmake and samc21/CMakePresets.json by hand - keep them in sync.
#
# "project" is also why the rosters are per project: app names COLLIDE
# across the source trees (blink, console and probe exist in both avrdx/
# and samc21/), so an app name alone never identifies an app.
BOARD_TYPES = {
    "db28": {"project": "avrdx", "preset": "avr128db28-release",
             "mcu": "avr128db28", "flash": "avrdude"},
    "db32": {"project": "avrdx", "preset": "avr128db32-release",
             "mcu": "avr128db32", "flash": "avrdude"},
    "db48": {"project": "avrdx", "preset": "avr128db48-release",
             "mcu": "avr128db48", "flash": "avrdude"},
    "c21j": {"project": "samc21", "preset": "samc21j-release",
             "mcu": "samc21j18a", "flash": "openocd",
             "target_cfg": "target/at91samdXX.cfg"},
    # The Nucleo-G0B1RE: STM32G0B1RE behind its on-board ST-LINK/V2.1 -
    # OpenOCD again, but the ST-LINK interface and the stm32g0x target
    # script (the stm32l4x flash driver underneath, which serves the G0).
    "g0b1re": {"project": "stm32g0", "preset": "stm32g0b1re-release",
               "mcu": "stm32g0b1re", "flash": "openocd",
               "target_cfg": "target/stm32g0x.cfg"},
    # The other two G0 boards: the Nucleo-G071RB (a Nucleo-64 like the
    # G0B1RE's, around a single-bank 128 K part) and the Nucleo-G031K8
    # (a Nucleo-32). Same probe, same target script, their own presets,
    # linker scripts and startup files.
    "g071rb": {"project": "stm32g0", "preset": "stm32g071rb-release",
               "mcu": "stm32g071rb", "flash": "openocd",
               "target_cfg": "target/stm32g0x.cfg"},
    "g031k8": {"project": "stm32g0", "preset": "stm32g031k8-release",
               "mcu": "stm32g031k8", "flash": "openocd",
               "target_cfg": "target/stm32g0x.cfg"},
}


def board_type(btype):
    spec = BOARD_TYPES.get(btype)
    if spec is None:
        die("unknown board type '%s' (known: %s) - a new type needs an entry "
            "in cli/cli/bench/common.py's BOARD_TYPES" % (btype, ", ".join(sorted(BOARD_TYPES))))
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
        die("no board '%s' in the manifest (have: %s) - edit the manifest"
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
            "into the socket the manifest names? (brio list)" % (path, name))
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

