"""brio gate - the byte-identity gate, and the token-identity check.

A change that claims to move no code proves it here: the reference
commit and the working tree are each built in a tree of their own with
every source's mtime pinned to one instant (the build id some images
carry is the newest source mtime) and the build directories wiped
(ninja relinks stale objects otherwise), and the images are compared
by md5 - per preset, identical count and movers by name.

    brio gate                          HEAD vs the working tree, the four release presets
    brio gate --against <ref>          another reference commit
    brio gate --preset avr128db48-release --preset samc21j-release
    brio gate --keep                   leave the two trees in place for a look

The token check is the finer instrument for a comments-only claim: a
source is lexed with the compiler (g++ -fpreprocessed -E -P, comments
gone), whitespace collapsed, and compared with the reference version;
with --strings the CONTENTS of string literals are blanked too and
adjacent literals folded, so a change that moved only what the
strings say compares equal.

    brio gate --tokens FILE...         token-identical to the reference, or not
    brio gate --tokens --strings FILE... ... ignoring what string literals say

Exit status 0 when every image (or every file) is identical, 1 otherwise."""

import argparse
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PINNED = 1767225600  # 2026-01-01 00:00:00 UTC, any fixed instant will do
DEFAULT_PRESETS = ("avr128db48-release", "samc21j-release", "stm32g0b1re-release", "ch32v006k8-release")


def project_of(preset):
    for prefix, project in (("avr", "avrdx"), ("samc21", "samc21"), ("stm32g0", "stm32g0"), ("ch32v0", "ch32v00x")):
        if preset.startswith(prefix):
            return project
    raise SystemExit("brio gate: no project for preset %r" % preset)


def image_ext(project):
    return ".hex" if project == "avrdx" else ".bin"


def run(cmd, cwd=None, capture=False):
    r = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE if capture else subprocess.DEVNULL,
                       stderr=subprocess.PIPE if capture else subprocess.DEVNULL, text=True)
    return r


def pin(tree):
    for dirpath, dirnames, filenames in os.walk(tree):
        dirnames[:] = [d for d in dirnames if d != ".git"]
        for f in filenames:
            p = os.path.join(dirpath, f)
            if not os.path.islink(p):
                os.utime(p, (PINNED, PINNED))


def make_ref_tree(ref, where):
    r = run(["git", "worktree", "add", "-q", "--detach", where, ref], cwd=ROOT, capture=True)
    if r.returncode != 0:
        raise SystemExit("brio gate: git worktree add failed:\n" + r.stderr)


def drop_ref_tree(where):
    run(["git", "worktree", "remove", "--force", where], cwd=ROOT)
    run(["git", "worktree", "prune"], cwd=ROOT)


def make_work_tree(where):
    """The working tree as it is: tracked and untracked files, minus what
    .gitignore excludes (the build directories among them)."""
    r = run(["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard"], cwd=ROOT, capture=True)
    for rel in r.stdout.split("\0"):
        if not rel:
            continue
        src = os.path.join(ROOT, rel)
        dst = os.path.join(where, rel)
        if not os.path.exists(src):
            continue  # deleted in the working tree
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if os.path.islink(src):
            os.symlink(os.readlink(src), dst)
        else:
            shutil.copy2(src, dst)


def build(tree, preset):
    project = project_of(preset)
    cwd = os.path.join(tree, project)
    if run(["cmake", "--preset", preset], cwd=cwd).returncode != 0:
        return None
    if run(["cmake", "--build", "--preset", preset], cwd=cwd).returncode != 0:
        return None
    out = os.path.join(tree, "build-cmake", preset)
    ext = image_ext(project)
    images = {}
    for f in sorted(os.listdir(out)):
        if f.endswith(ext):
            with open(os.path.join(out, f), "rb") as fh:
                images[f[:-len(ext)]] = hashlib.md5(fh.read()).hexdigest()
    return images


def gate_images(ref, presets, keep):
    scratch = tempfile.mkdtemp(prefix="brio-gate-")
    ref_tree = os.path.join(scratch, "ref")
    work_tree = os.path.join(scratch, "work")
    status = 0
    try:
        make_ref_tree(ref, ref_tree)
        os.makedirs(work_tree)
        make_work_tree(work_tree)
        pin(ref_tree)
        pin(work_tree)
        for preset in presets:
            a = build(ref_tree, preset)
            b = build(work_tree, preset)
            if a is None or b is None:
                print("%s: BUILD FAILED in the %s tree" % (preset, "reference" if a is None else "working"))
                status = 1
                continue
            same = sorted(n for n in a if n in b and a[n] == b[n])
            movers = sorted(n for n in a if n in b and a[n] != b[n])
            new = sorted(n for n in b if n not in a)
            gone = sorted(n for n in a if n not in b)
            line = "%s: %d/%d byte-identical" % (preset, len(same), len(a))
            if movers:
                line += "; MOVERS: " + " ".join(movers)
                status = 1
            if new:
                line += "; new: " + " ".join(new)
            if gone:
                line += "; gone: " + " ".join(gone)
                status = 1
            print(line)
    finally:
        if keep:
            print("trees kept under", scratch)
        else:
            drop_ref_tree(ref_tree)
            shutil.rmtree(scratch, ignore_errors=True)
    return status


STRING_RE = re.compile(r'"(\\.|[^"\\])*"')


def tokens(path, blank_strings):
    r = subprocess.run(["g++", "-fpreprocessed", "-dD", "-E", "-P", "-x", "c++", path],
                       stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    text = r.stdout
    if blank_strings:
        text = STRING_RE.sub('""', text)
        # phase 6 of translation: adjacent literals are ONE literal, on
        # one line or across a line break
        text = re.sub(r'(""\s*)+', '"" ', text)
    return " ".join(text.split())


def gate_tokens(ref, files, blank_strings):
    status = 0
    for f in files:
        rel = os.path.relpath(os.path.abspath(f), ROOT)
        r = run(["git", "show", "%s:%s" % (ref, rel)], cwd=ROOT, capture=True)
        if r.returncode != 0:
            print("%s: not in %s" % (rel, ref))
            status = 1
            continue
        with tempfile.NamedTemporaryFile("w", suffix=os.path.splitext(rel)[1], delete=False) as tmp:
            tmp.write(r.stdout)
            refpath = tmp.name
        try:
            same = tokens(refpath, blank_strings) == tokens(os.path.abspath(f), blank_strings)
        finally:
            os.unlink(refpath)
        if same:
            print("%s: token-identical%s" % (rel, " (strings blanked)" if blank_strings else ""))
        else:
            print("%s: DIFFERENT" % rel)
            status = 1
    return status


def main(argv):
    ap = argparse.ArgumentParser(prog="brio gate", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--against", default="HEAD", metavar="REF", help="the reference commit (default HEAD)")
    ap.add_argument("--preset", action="append", metavar="PRESET",
                    help="a preset to build and compare (repeatable; default: the four release presets)")
    ap.add_argument("--keep", action="store_true", help="keep the two built trees")
    ap.add_argument("--tokens", action="store_true", help="token-identity of FILES instead of images")
    ap.add_argument("--strings", action="store_true", help="with --tokens: blank string literal contents")
    ap.add_argument("files", nargs="*", help="with --tokens: the sources to check")
    args = ap.parse_args(argv[1:])
    if args.tokens:
        if not args.files:
            ap.error("--tokens needs at least one file")
        return gate_tokens(args.against, args.files, args.strings)
    if args.files:
        ap.error("files are only meaningful with --tokens")
    return gate_images(args.against, args.preset or list(DEFAULT_PRESETS), args.keep)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
