"""brio check [avrdx|samc21|stm32g0|ch32v00x|all] [filter] - the family compile
fixtures: every smoke TU under test/family*/ compiles for every package
or variant of the stratum, and every negative TU is REFUSED. The
fixtures are the four shell scripts under cli/checks/ (they call the
cross compiler directly, with no CMake in between); this module only
picks and runs them."""

import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = {
    "avrdx": "check_family.sh",
    "samc21": "check_samc21.sh",
    "stm32g0": "check_stm32g0.sh",
    "ch32v00x": "check_ch32v00x.sh",
}


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
    return status


if __name__ == "__main__":
    sys.exit(main(sys.argv))
