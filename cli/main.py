"""brio - the one command of the bench and of the repository.

    brio list                      serial devices, USB probes, the manifest
    brio flash <board> <app>       build the app for the board and flash it
    brio run <board> <letter>      drive a suite's console, judge "ALL: N pass, M fail"
    brio console <board>           the console device path and speed
    brio duo <dut>:<cmd> <peer>:<script>
    brio fuses <board> [name=value ...]
    brio stress ...                the host end of the UART suites
    brio check [avrdx|samc21|stm32g0|all] [filter]   the family compile fixtures
    brio prose [paths...]          the prose net (comments and documents)
    brio gate [--against REF] [--preset P]...   images byte-identical to REF? (movers named)
    brio gate --tokens [--strings] FILE...      sources token-identical to REF?

The first six need a board on the desk and live in cli/bench/; the
last three need none. Put bin/ on the PATH, or call bin/brio from
anywhere in the tree."""

import sys

BENCH_VERBS = ("list", "flash", "run", "console", "duo", "fuses")


def main(argv):
    if len(argv) < 2 or argv[1] in ("-h", "--help"):
        print(__doc__)
        return 0
    verb, rest = argv[1], argv[2:]
    prog = argv[0] + " " + verb
    if verb == "prose":
        from cli import prose
        return prose.main([prog] + rest)
    if verb == "gate":
        from cli import gate
        return gate.main([prog] + rest)
    if verb == "check":
        from cli import check
        return check.main([prog] + rest)
    if verb == "stress":
        from cli.bench import stress
        sys.argv = [prog] + rest
        return stress.main()
    if verb in BENCH_VERBS:
        from cli.bench import verbs
        sys.argv = [argv[0]] + argv[1:]
        return verbs.main()
    print("brio: unknown verb %r (brio --help lists them)" % verb)
    return 2
