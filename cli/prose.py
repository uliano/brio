#!/usr/bin/env python3
"""cli/prose.py - the prose net: what a comment or a document may not say.

Comments and documents in this repository are a reference for the code
as it is (docs/README.md, docs/design/overview.md). Three things age
mechanically and this tool catches them before a reader does:

  1. HISTORY in prose: an ISO date, a process word (campaign, ruling,
     "the first version", judgment call), a Doxygen tag, a "Validated
     on:" list, the name of a build system this repo no longer uses.
     Every hit is an ERROR.
  2. A DANGLING POINTER: a path such as brio/samc21/dmac.hpp or
     docs/design/spi-bus.md cited in a comment or a document and not
     present in the tree. The rule that a claim about another part of
     the tree is written as a POINTER, never as a statement, only works
     if the pointer is checked. Every hit is an ERROR.
  3. A CLAIM OF ABSENCE: "there is no X driver", "nothing calls it
     yet", "does not exist yet", "future pass". These are sometimes true
     and always age, so they are listed for REVIEW, never an error.

Plus the repo's own ASCII rule (every byte <= 127) on every file it
reads, as an ERROR.

Scope: comments of every .hpp/.cpp under brio/, the three build
projects' src/ trees and experiments/; every .md under docs/ and the
top-level README.md. Not read, on purpose: CLAUDE.md (a working log by
design), private/ (the desk diary), docs/*/vendor/ (datasheet revisions
carry dates of record), third_party/.

Usage: brio prose [paths...]   (no paths = the whole scope)
Exit status: 1 on any error, 0 otherwise. Review items never fail.
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SOURCE_ROOTS = ["brio", "avrdx/src", "samc21/src", "stm32g0/src", "experiments"]
DOC_ROOTS = ["docs"]
DOC_FILES = ["README.md"]
EXCLUDE_DIRS = ("third_party", "build-cmake", ".git", "private")
EXCLUDE_FILES = ()
EXCLUDE_GLOBS = ("/vendor/",)

ERROR_PATTERNS = [
    (re.compile(r"\b20[0-9]{2}-[01][0-9]-[0-3][0-9]\b"), "an ISO date"),
    (re.compile(r"\bcampaign\b", re.I), "'campaign'"),
    (re.compile(r"\bruling\b|\bRULED\b"), "'ruling'"),   # "ruled by the crystal" is English, RULED is process
    (re.compile(r"\bthe first version\b", re.I), "'the first version'"),
    (re.compile(r"\bjudgment call\b", re.I), "'judgment call'"),
    (re.compile(r"@(brief|date|param|return|author|tparam|class|struct|file)\b"), "a Doxygen tag"),
    (re.compile(r"\bValidated on:"), "a per-file validation list"),
    (re.compile(r"platformio", re.I), "PlatformIO"),
]

REVIEW_PATTERNS = [
    (re.compile(r"\bthere is no [A-Za-z0-9_]+ driver\b", re.I), "a claim that a driver is absent"),
    (re.compile(r"\bno [A-Za-z0-9_]+ driver (yet|exists|in this stratum)\b", re.I), "a claim that a driver is absent"),
    (re.compile(r"\bdoes not exist yet\b", re.I), "'does not exist yet'"),
    (re.compile(r"\bnothing calls it yet\b", re.I), "'nothing calls it yet'"),
    (re.compile(r"\bnot built yet\b", re.I), "'not built yet'"),
    (re.compile(r"\bfuture pass\b", re.I), "'future pass'"),
]

PATH_RE = re.compile(
    r"(?<![/A-Za-z0-9_.-])"
    r"(?:brio|docs|tools|bench|cli|bin|avrdx|samc21|stm32g0|armv6m|kernel|util|host|test|experiments)"
    r"/[A-Za-z0-9_][A-Za-z0-9_./-]*(?:\.(?:hpp|cpp|md|py|sh|json|svg|ld|cmake|txt)|(?<=bin/brio))\b"
)


def resolves(cited):
    """A cited path is written the way the code includes it or the way a
    document links it: `samc21/rtc.hpp` is brio/samc21/rtc.hpp (the -I brio
    convention), `avrdx/tcd.md` is docs/avrdx/tcd.md."""
    for base in ("", "brio", "docs"):
        if os.path.exists(os.path.join(ROOT, base, cited)):
            return True
    return False


def excluded(path):
    rel = os.path.relpath(path, ROOT)
    if rel in EXCLUDE_FILES:
        return True
    if any(part in EXCLUDE_DIRS for part in rel.split(os.sep)):
        return True
    return any(g in "/" + rel for g in EXCLUDE_GLOBS)


def walk(roots, exts):
    for root in roots:
        top = os.path.join(ROOT, root)
        if os.path.isfile(top):
            yield top
            continue
        for dirpath, dirnames, filenames in os.walk(top):
            dirnames[:] = [d for d in dirnames if d not in EXCLUDE_DIRS]
            for f in sorted(filenames):
                p = os.path.join(dirpath, f)
                if f.endswith(exts) and not excluded(p):
                    yield p


def comment_spans(text):
    """Yield (line_number, comment_text) for every C++ comment, skipping
    string and character literals (so a '//' inside "..." is not a
    comment and a '/*' inside a string does not open one)."""
    i, n, line = 0, len(text), 1
    while i < n:
        c = text[i]
        if c == "\n":
            line += 1
            i += 1
        elif c in "\"'":
            q = c
            i += 1
            while i < n and text[i] != q:
                if text[i] == "\\":
                    i += 1
                if i < n and text[i] == "\n":
                    line += 1
                i += 1
            i += 1
        elif text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            yield line, text[i:j]
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            body = text[i:j]
            yield line, body
            line += body.count("\n")
            i = j
        else:
            i += 1


def check_text(path, chunks, errors, reviews):
    """chunks: iterable of (line, text). Applies every pattern."""
    for line, chunk in chunks:
        for pat, what in ERROR_PATTERNS:
            for m in pat.finditer(chunk):
                errors.append((path, line + chunk[:m.start()].count("\n"), what))
        for pat, what in REVIEW_PATTERNS:
            for m in pat.finditer(chunk):
                reviews.append((path, line + chunk[:m.start()].count("\n"), what))
        for m in PATH_RE.finditer(chunk):
            cited = m.group(0)
            if not resolves(cited):
                at = line + chunk[:m.start()].count("\n")
                errors.append((path, at, "a dangling path: " + cited))


def main(argv):
    if argv[1:2] in (["-h"], ["--help"]):
        print(__doc__)
        return 0
    targets = argv[1:]
    errors, reviews = [], []
    if targets:
        files = [os.path.abspath(t) for t in targets]
    else:
        files = list(walk(SOURCE_ROOTS, (".hpp", ".cpp"))) + \
                list(walk(DOC_ROOTS, (".md",))) + \
                [os.path.join(ROOT, f) for f in DOC_FILES]
    for path in files:
        with open(path, "rb") as fh:
            raw = fh.read()
        rel = os.path.relpath(path, ROOT)
        bad = [k for k, b in enumerate(raw) if b > 127]
        if bad:
            errors.append((path, raw[:bad[0]].count(b"\n") + 1, "a byte above 127"))
        text = raw.decode("ascii", errors="replace")
        if path.endswith(".md"):
            check_text(rel, ((k + 1, l) for k, l in enumerate(text.split("\n"))), errors, reviews)
        else:
            check_text(rel, comment_spans(text), errors, reviews)
    for rel, line, what in errors:
        print("%s:%d: error: %s" % (rel, line, what))
    for rel, line, what in reviews:
        print("%s:%d: review: %s" % (rel, line, what))
    print("check_prose: %d file(s), %d error(s), %d for review" % (len(files), len(errors), len(reviews)))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
