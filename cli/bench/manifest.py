"""The bench manifest: which board sits where, on which console, behind
which programmer. It is the user's own desk, so it is loaded from the
first of these that exists:

  private/bench_boards.py   the desk that is really there (not published)
  cli/cli/bench/bench_boards.py the one in the repository

Every verb reaches the manifest through load(); nothing imports the file
by name."""

import importlib.util
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CANDIDATES = (
    os.path.join(ROOT, "private", "bench_boards.py"),
    os.path.join(ROOT, "cli", "bench", "bench_boards.py"),
)


def path():
    for p in CANDIDATES:
        if os.path.isfile(p):
            return p
    raise SystemExit("bench: no manifest found (looked for %s)" % ", ".join(CANDIDATES))


def load():
    p = path()
    spec = importlib.util.spec_from_file_location("bench_boards", p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod
