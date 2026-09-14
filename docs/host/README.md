# Target: host (`host/`)

The native build is a target like any other: `brio/host/
platform.hpp` provides `HostPlatform` (and `HostCore<n>`, the
two-core host of `test_inbox`: the same platform told apart by type,
with a test-set current core and a counting doorbell), an implementation of the
kernel's `Platform` concept for a single-threaded test process:

- a depth-counting `CriticalSection` (tests can assert that a lock-free
  path never took the guard, or that a guarded one released it);
- a test-controlled virtual clock behind `now()` - time becomes
  deterministic arithmetic: advance it, call `TimeEvents<P>::process()`,
  assert what was posted;
- recording `idle()` / `break_here()` (counters, no sleeping, no trap);
- `panic_record()` in ordinary static storage, cleared by `reset()`;
- `ticks_per_second = 1000` (the identity case for ms conversions) and
  `atomic_width = 4` (every brio ring index is lock-free here); tests
  that need the other paths (a non-dividing rate, the guarded ring)
  define a small local platform of their own.

Because the kernel and `util/` are pure logic templated on the
platform, everything above the drivers runs here unchanged: queues,
scheduler, FSM contract, time events, panic, `Ring`, `SerialPort`,
`BusMaster` (with a fake bus engine). This is where the things that
are hard to provoke on silicon are tested first: queue overflow and
counters, entry/exit ordering, drift-free periodic re-arm, scan
priority, MPSC stress with simulated producers, wrap-around deadlines.

## Running

```bash
ctest --preset host                 # all suites
ctest --preset host -R test_fsm     # one suite
```

- Framework: [doctest](https://github.com/doctest/doctest) (vendored,
  `third_party/doctest/`), one `test/test_<subject>/main.cpp` per
  suite, each with `DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN`.
- `test/CMakeLists.txt` is a CMake project of its own, a peer of the
  cross-build projects rather than a part of them (its own
  `CMakePresets.json`, host g++, no cross toolchain - a CMake
  configure has exactly one compiler), so no cross build is in reach
  of a probe here in the first place.
- No hardware needed; the host compiler must speak gnu++23 (the same
  standard as the cross builds - the code is identical).

## Simulated devices, and what they are allowed

Beside the platform, this stratum carries simulations of things a test
cannot otherwise provoke: flash with its wear, its power cuts and its
reflash; a USB controller answering scripted control transfers; the
reference implementations a test compares a real one against.

**The whole standard library is allowed inside them, heap included.**
Above the concept, host code is brio code and pays what brio pays; below
it, in a simulation's own guts, modelling a constraint faithfully is
worth more than modelling it cheaply, and none of that code is going
anywhere near a silicon. What brio sees is the CONCEPT the simulation
satisfies, and nothing above that concept can tell how it was reached.

The same reasoning puts a JUDGE here rather than beside the code it
judges - a reference renderer, a reference decoder, anything a test
measures an optimized implementation against. From this stratum it
cannot be linked into a target image even by accident, which is a
stronger guarantee than remembering not to.

## The simulator

A host program can put its pixels on a screen and take its input from a
panel, through a second process that shows the one and drives the other.
Both directions are one mechanism - a named region of memory the program
owns - and the contract between the two processes, the layouts and the
rules that keep it portable are [simulator.md](simulator.md).
