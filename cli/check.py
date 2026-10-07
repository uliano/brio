"""brio check [avrdx|samc21|stm32g0|ch32v00x|ch32vx03|ch32x035|rp2040|rp2350|stm32f4|all]
[filter] - the family compile fixtures: every smoke TU under
test/family*/ compiles for every package or variant of the stratum, and
every negative TU is REFUSED. The fixtures are the shell scripts under
cli/checks/, one per stratum (they call the cross compiler directly,
with no CMake in between - and the rp2350 one calls TWO of them, the
chip having two processor architectures); this module picks and runs
them.

THEN THE LINK GUARD, when no filter is given: every app of the
stratum's build project built on every release preset - the family's
smallest part among them - by `brio fit` (cli/fit.py), so an app that
claims a board type its image does not compile or fit on fails the
check by name, and an image with under 5 per cent of its flash or RAM
left is named. The fixtures prove the strata's headers on every part;
only a link proves an app's image on its part. The presets build under
build-cmake/fit/ (or $BRIO_FIT_INTO), apart from the directories
`brio flash` builds in.

AND ON THE ch32vx03 STRATUM, THE VECTOR GUARD over those images
(cli/vector_guard.py): on the CH32V303's images a vector is a naked
trampoline whose body must reach no floating-point instruction, and the
guard walks every such body's call graph in the linked image and fails
the check on one."""

import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = {
    "avrdx": "check_family.sh",
    "samc21": "check_samc21.sh",
    "stm32g0": "check_stm32g0.sh",
    "ch32v00x": "check_ch32v00x.sh",
    "ch32vx03": "check_ch32vx03.sh",
    "ch32x035": "check_ch32x035.sh",
    "rp2040": "check_rp2040.sh",
    "rp2350": "check_rp2350.sh",
    "stm32f4": "check_stm32f4.sh",
}


# The strata whose linked images the vector guard reads.
GUARDED = ("ch32vx03",)


def vector_guard(project):
    """cli/vector_guard.py over every image the link guard built for the
    project (the guard itself passes over an image without F)."""
    import glob
    from cli import fit, vector_guard as vg
    into = os.environ.get("BRIO_FIT_INTO") or os.path.join(ROOT, "build-cmake", "fit")
    elfs = sorted(e for preset in fit.presets(project) for e in glob.glob(os.path.join(into, preset, "*.elf")))
    return vg.run(elfs, quiet=True)


def main(argv):
    rest = argv[1:]
    if rest and rest[0] in ("-h", "--help"):
        print(__doc__)
        return 0
    which = rest[0] if rest else "all"
    names = list(SCRIPTS) if which == "all" else [which]
    status = 0
    for n in names:
        if n not in SCRIPTS:
            print("brio check: unknown stratum %r (one of %s, all)" % (n, ", ".join(SCRIPTS)))
            return 2
        status |= subprocess.call([os.path.join(ROOT, "cli", "checks", SCRIPTS[n])] + rest[1:])
        if not rest[1:]:
            from cli import fit
            status |= fit.run([n], quiet=True)
            if n in GUARDED:
                status |= vector_guard(n)
            sys.stdout.flush()   # before the next script writes to the same stream
    return status


if __name__ == "__main__":
    sys.exit(main(sys.argv))
