"""The console protocol of the bench suites (single-char commands, the
"> " prompt, the "ALL: N pass, M fail" summary, with ", L lost, T torn"
when the suite prints through a LineWriter that lost lines, and the lines
a suite marks "#<T ... :T>#" for the judge to read whole) and the verbs
over it: brio run, brio console, brio duo."""

import argparse
import json
import os
import re
import subprocess
import sys
import time

from cli.bench.common import *   # noqa: F401,F403

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


def losses(text, marker):
    """(lost, torn) the run's tally reports: the summary line's two
    optional fields, or the letters' tallies summed; (0, 0) when no tally
    carries them (a suite printing to a transport, or a writer that lost
    nothing)."""
    for line in text.splitlines():
        if marker and marker in line:
            m = LOSS_RE.search(line)
            if m:
                return int(m.group(3)), int(m.group(4))
    lost = torn = 0
    for m in LOSS_RE.finditer(text):
        lost += int(m.group(3))
        torn += int(m.group(4))
    return lost, torn


def marked_lines(text):
    """The judge's read of the lines a suite marked "#<T ...:T>#": the
    problems found, each a string - a marked line not whole (its two tags
    unmatched, or another line's text inside it), or a count other than
    the one "#count N" announced."""
    problems = []
    whole = 0
    expected = None
    for line in text.splitlines():
        stripped = line.strip()
        m = MARK_COUNT_RE.fullmatch(stripped)
        if m:
            expected = (expected or 0) + int(m.group(1))
            continue
        if "#<" not in line and ">#" not in line:
            continue
        if MARKED_RE.fullmatch(stripped) and stripped.count("#<") == 1 and stripped.count(">#") == 1:
            whole += 1
        else:
            problems.append("marked line not whole: %r" % stripped)
    if expected is not None and whole != expected:
        problems.append("%d whole marked lines, %d announced" % (whole, expected))
    return problems, whole


def judge(text, marker, strict, name):
    """The run's verdict: the suite's own failures, the marked lines read
    whole, and - under --strict - the lines a writer lost. Dies on any."""
    fails, summary = verdict(text, marker)
    if summary:
        print("bench: %s" % summary)
    problems, whole = marked_lines(text)
    if whole or problems:
        print("bench: %d marked line(s) read whole" % whole)
    for p in problems:
        print("bench: %s" % p)
    lost, torn = losses(text, marker)
    if lost or torn:
        print("bench: the writer lost %d line(s), %d torn" % (lost, torn))
    if fails:
        die("board %s reported %d failure(s)" % (name, fails))
    if problems:
        die("board %s: %d marked-line problem(s)" % (name, len(problems)))
    if strict and (lost or torn):
        die("board %s: lines lost under --strict" % name)


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
    judge(text, marker, args.strict, name)
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
    judge(text, args.expect, args.strict, dut_name)
    return 0


def split_pair(text, what):
    if ":" not in text:
        die("%s argument must be <board>:<value>, got '%s'" % (what, text))
    name, value = text.split(":", 1)
    if not name or not value:
        die("%s argument must be <board>:<value>, got '%s'" % (what, text))
    return name, value


