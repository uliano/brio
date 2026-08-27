# Target: host (`host/`)

The native build is a target like any other: `lib/brio/src/host/
platform_host.hpp` provides `HostPlatform`, an implementation of the
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
- `test/CMakeLists.txt` is an entirely separate CMake project from the
  repo root's AVR build (its own `CMakePresets.json`, host g++, no
  cross toolchain - a CMake configure has exactly one compiler), so
  there is no AVR build to accidentally touch a probe from in the
  first place.
- No hardware needed; the host compiler must speak gnu++23 (the same
  standard as the target build - the code is identical).
