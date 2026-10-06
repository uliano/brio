"""brio fit [stratum|all] [--debug] [--into DIR] [--margin PCT] [--quiet]
- what every app's image takes of its part: every app built on every
release preset of the stratum's build project (each preset building the
apps whose `// build: boards` line names its board type, a suite with
groups as its group images where the project splits it), then each
image's flash and RAM against the regions its own linker script gives
them, read off the image's .map, sorted by the smaller margin.

    brio fit                  every build project, every release preset
    brio fit ch32vx03         one stratum's project
    brio fit all --debug      the debug presets as well (what a debugger
                              session flashes; no guard, reported only)
    brio fit --into DIR       build under DIR/<preset> (default:
                              $BRIO_FIT_INTO, else build-cmake/fit/,
                              apart from the directories `brio flash`
                              builds in)
    brio fit --margin 5       the margin, in per cent, under which an
                              image is flagged (default 5)

FLASH is the bytes with contents whose LOAD address lies in the flash
region (.text, the vectors, the initial values of .data, code carried to
RAM); RAM is the bytes allocated at a RUN address in the RAM region
(.data, .bss, .noinit): the static RAM. The stack is not in it - on
every target the stack grows down from the top of that region (on the
RP2040 and the RP2350 it lives in the scratch banks instead), so a RAM
margin is the room the stack has.

`brio check <stratum>` runs the same build after its fixtures - the link
guard: an image that fails to compile or link on any preset fails the
check, and an image under the margin is named. Exit status: 0 when
every image built, 1 otherwise."""

import argparse
import json
import os
import re
import struct
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECTS = ("avrdx", "samc21", "stm32g0", "stm32f4", "ch32v00x", "ch32vx03", "ch32x035", "rp2040", "rp2350")
FLASH_REGIONS = ("rom", "flash", "text")
RAM_REGIONS = ("ram", "data")
SHF_ALLOC = 0x2
SHT_NOBITS = 8
PT_LOAD = 1


def presets(project, debug=False):
    """The project's configure presets: every release one, and the debug
    ones when asked, in the order its CMakePresets.json lists them."""
    with open(os.path.join(ROOT, project, "CMakePresets.json"), encoding="ascii") as f:
        names = [c["name"] for c in json.load(f)["configurePresets"]]
    keep = ("-release", "-debug") if debug else ("-release",)
    return [n for n in names if n.endswith(keep)]


def regions(mapfile):
    """{name: (origin, length)} from the map's "Memory Configuration"."""
    with open(mapfile, encoding="ascii", errors="replace") as f:
        text = f.read()
    m = re.search(r"Memory Configuration\s*\n\s*\nName[^\n]*\n(.*?)\n\s*\nLinker script", text, re.S)
    out = {}
    if m:
        for line in m.group(1).splitlines():
            p = line.split()
            if len(p) >= 3 and p[0] != "*default*":
                out[p[0]] = (int(p[1], 16), int(p[2], 16))
    return out


def elf32_layout(path):
    """(sections, segments) of a little-endian ELF32 image - every target
    brio builds is one: sections as (type, flags, addr, size), the PT_LOAD
    segments as (vaddr, paddr, memsz)."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 1:
        raise ValueError("%s: not a little-endian ELF32 image" % path)
    (e_phoff, e_shoff) = struct.unpack_from("<II", data, 0x1C)
    (e_phentsize, e_phnum, e_shentsize, e_shnum) = struct.unpack_from("<HHHH", data, 0x2A)
    sections = []
    for i in range(e_shnum):
        (_name, s_type, s_flags, s_addr, _off, s_size) = struct.unpack_from("<IIIIII", data, e_shoff + i * e_shentsize)
        sections.append((s_type, s_flags, s_addr, s_size))
    segments = []
    for i in range(e_phnum):
        (p_type, _off, p_vaddr, p_paddr, _filesz, p_memsz) = struct.unpack_from("<IIIIII", data, e_phoff + i * e_phentsize)
        if p_type == PT_LOAD:
            segments.append((p_vaddr, p_paddr, p_memsz))
    return sections, segments


def usage(elf, regs):
    """(flash used, flash length, RAM used, RAM length) of one image, or
    None when its map names no flash or no RAM region."""
    fname = next((n for n in regs if n.lower() in FLASH_REGIONS), None)
    rname = next((n for n in regs if n.lower() in RAM_REGIONS), None)
    if fname is None or rname is None:
        return None
    forg, flen = regs[fname]
    rorg, rlen = regs[rname]
    sections, segments = elf32_layout(elf)
    flash = ram = 0
    for s_type, s_flags, addr, size in sections:
        if not s_flags & SHF_ALLOC or size == 0:
            continue
        lma = addr
        for vaddr, paddr, memsz in segments:
            if vaddr <= addr < vaddr + max(memsz, 1):
                lma = paddr + (addr - vaddr)
                break
        if s_type != SHT_NOBITS and forg <= lma < forg + flen:
            flash += size
        if rorg <= addr < rorg + rlen:
            ram += size
    return flash, flen, ram, rlen


def elf_targets(bdir):
    """The images this configure declares: ninja's executable targets."""
    out = subprocess.run(["ninja", "-C", bdir, "-t", "targets", "all"], capture_output=True, text=True).stdout
    return sorted({line.split(":")[0][:-4] for line in out.splitlines()
                   if line.split(":")[0].endswith(".elf") and "/" not in line.split(":")[0]})


def failures(log):
    """{image: reason} from a build log: a link that overflowed a region
    or failed otherwise, and a translation unit that did not compile."""
    out = {}
    lines = log.splitlines()
    for i, line in enumerate(lines):
        m = re.match(r"FAILED: (?:\[code=\d+\] )?(\S+)", line)
        if not m:
            continue
        target = m.group(1)
        tail = lines[i + 1:i + 60]
        nxt = next((j for j, t in enumerate(tail) if t.startswith("FAILED:")), len(tail))
        tail = tail[:nxt]
        if target.endswith(".elf"):
            image = target[:-4]
            over = [re.search(r"region `(\w+)' overflowed by (\d+) bytes", t) for t in tail]
            over = [o for o in over if o]
            if over:
                out[image] = ", ".join("%s overflowed by %s bytes" % (o.group(1), o.group(2)) for o in over)
            else:
                err = next((t for t in tail if "error" in t), "the link failed")
                out[image] = err.strip()[:160]
        else:
            m2 = re.search(r"CMakeFiles/([^/]+)\.dir/", target)
            image = m2.group(1) if m2 else target
            err = next((t for t in tail if "error:" in t), "a translation unit failed to compile")
            err = err.split("error:", 1)[-1].strip()
            out.setdefault(image, "does not compile: " + err[:160])
    return out


def build(project, preset, into, jobs):
    """Configure and build one preset; (seconds, {image: reason}, rows)."""
    bdir = os.path.join(into, preset)
    t0 = time.monotonic()
    cfg = subprocess.run(["cmake", "--preset", preset, "-B", bdir], cwd=os.path.join(ROOT, project),
                         capture_output=True, text=True)
    if cfg.returncode != 0:
        return time.monotonic() - t0, {"(configure)": cfg.stderr.strip()[-300:]}, []
    res = subprocess.run(["cmake", "--build", bdir, "-j", str(jobs), "--", "-k", "0"],
                         capture_output=True, text=True)
    seconds = time.monotonic() - t0
    failed = failures(res.stdout + res.stderr) if res.returncode != 0 else {}
    if res.returncode != 0 and not failed:
        failed = {"(build)": (res.stdout + res.stderr).strip()[-300:]}
    rows = []
    for image in elf_targets(bdir):
        if image in failed:
            continue
        elf = os.path.join(bdir, image + ".elf")
        mapfile = os.path.join(bdir, "firmware-%s.map" % image)
        if not (os.path.isfile(elf) and os.path.isfile(mapfile)):
            continue
        u = usage(elf, regions(mapfile))
        if u is not None:
            rows.append((preset, image) + u)
    return seconds, failed, rows


def left(used, length):
    return 100.0 * (length - used) / length


def run(projects, debug=False, into=None, margin=5.0, quiet=False, jobs=None):
    """Build and measure; print the table (or, quiet, only what fails or
    is under the margin); return 0 when every image built."""
    into = into or os.environ.get("BRIO_FIT_INTO") or os.path.join(ROOT, "build-cmake", "fit")
    jobs = jobs or os.cpu_count() or 4
    rows, failed, total = [], [], 0.0
    for project in projects:
        for preset in presets(project, debug):
            seconds, bad, got = build(project, preset, into, jobs)
            total += seconds
            rows += got
            failed += [(preset, image, why) for image, why in sorted(bad.items())]
            if not quiet:
                print("brio fit: %-24s %3d images, %d failed, %.1f s" % (preset, len(got), len(bad), seconds))
    rows.sort(key=lambda r: (min(left(r[2], r[3]), left(r[4], r[5])), r[0], r[1]))
    tight = [r for r in rows if min(left(r[2], r[3]), left(r[4], r[5])) < margin]
    if not quiet:
        print()
        print("%-24s %-28s %8s %8s %6s %7s %7s %6s" % ("preset", "image", "flash", "of", "left%", "ram", "of",
                                                     "left%"))
        for p, a, fu, fl, ru, rl in rows:
            flag = "  <- under %g%%" % margin if (p, a) in {(t[0], t[1]) for t in tight} else ""
            print("%-24s %-28s %8d %8d %6.1f %7d %7d %6.1f%s" % (p, a, fu, fl, left(fu, fl), ru, rl, left(ru, rl), flag))
        print()
    elif tight:
        for p, a, fu, fl, ru, rl in tight:
            print("brio fit: %s %s under %g%%: flash %d of %d (%.1f%% left), RAM %d of %d (%.1f%% left)"
                  % (p, a, margin, fu, fl, left(fu, fl), ru, rl, left(ru, rl)))
    for preset, image, why in failed:
        print("brio fit: FAILED %s %s: %s" % (preset, image, why))
    print("brio fit: %s - %d images built, %d failed, %d under %g%%, %.0f s"
          % (", ".join(projects), len(rows), len(failed), len(tight), margin, total))
    return 1 if failed else 0


def main(argv):
    ap = argparse.ArgumentParser(prog=argv[0], description=__doc__.split("\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("which", nargs="?", default="all", help="a stratum's build project, or all")
    ap.add_argument("--debug", action="store_true", help="the debug presets as well")
    ap.add_argument("--into", help="the directory the presets build under")
    ap.add_argument("--margin", type=float, default=5.0, help="flag an image under this per cent")
    ap.add_argument("--quiet", action="store_true", help="only failures and images under the margin")
    args = ap.parse_args(argv[1:])
    if args.which == "all":
        projects = list(PROJECTS)
    elif args.which in PROJECTS:
        projects = [args.which]
    else:
        print("brio fit: unknown stratum %r (one of %s, all)" % (args.which, ", ".join(PROJECTS)))
        return 2
    return run(projects, args.debug, args.into and os.path.abspath(args.into), args.margin, args.quiet)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
