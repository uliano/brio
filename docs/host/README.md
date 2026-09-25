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

`sim_dcs_panel.hpp` is a COMMAND PANEL made of RAM, in the two layers
the command tier prescribes ([../design/gfx.md](../design/gfx.md)):
`SimDcsPanel<Traits>`, the core whose seam is the DCS transaction - the
frame memory, the window and the walk its address mode selects, the read
pointer that counts the bytes clocked - and `SimDcsSerial<Core>`, the
framing adapter that puts the four-wire serial interface in front of it
at the level of bytes, with the dummy byte and the one-bit-late stream a
controller's traits ask for. Every rule in it is a bench measurement and
the ones that are not are marked ASSUMPTION beside the code, because a
simulator written from a command table agrees with a driver written from
the same table on the same mistake: what it judges is whether the driver
does what the part was MEASURED to do, never whether that belief is
right.

`sim_spi_host.hpp` is what drives that panel: a SPI HOST MADE OF RAM
whose seam is the REQUEST, offering the arbiter and a device driver the
same static surface a family's `SpiHost<n>` does with no register block
behind it. It is not the same kind of object as `sim_pl022.hpp`, and the
difference is the question each answers. A simulated PL022 has to BE the
chip - the registers at their offsets, their reset values, the order of
the acts of a bring-up - because what it proves is that one driver knows
no chip. This one proves nothing about a driver of the silicon: it is
the WORLD on the other side of a bus, so that a device driver can be
judged against a device made of RAM -
[../design/simulation.md](../design/simulation.md) places it: a device
the program reaches through its own driver, never a channel into the
kernel. It models the two phases of the descriptor in
one select window with the D/C flipping between them, the select active
low, a null `tx` clocking 0xFF and a null `rx` discarding, a frame of
two bytes where the request asks for one, and a select line with nothing
on it reading 0xFF - a bus with no device on it, counted and not an
error. Devices are attached to a select pin by `attach()` and held in a
small type-erased table, so a panel's framing adapter and a counting
stub can sit on one bus. Both completion styles of the contract are
there, because the arbiter distinguishes them: in `immediate` mode every
request completes inside `start()`, and in `deferred` mode an unpolled
one is HELD - `start()` returns false, `finish()` performs it and hands
back the status the app's ISR glue would post, which is the shape a
test of an asynchronous transfer needs. A byte-level TRACE ring records
what the wires carried, which is how a test asserts a D/C choreography,
and each pin remembers when it was last driven on the same ruler the
bytes are stamped with, which is how it asserts that the select fell
before the first byte. What it does NOT model is the wire and the time
on it: no clock rate, no mode, no bit - the `cs_setup_us` is COUNTED and
never spent, and a mode the device disagrees with changes nothing. A
test that wants those wants silicon. `test/test_dcs_link` is its suite,
and the one that drives the DCS link
([../devices/dcs_link.md](../devices/dcs_link.md)) over it.

## The simulator

A host program can put its pixels on a screen and take its input from a
panel, through a second process that shows the one and drives the other.
Both directions are one mechanism - a named region of memory the program
owns - and the contract between the two processes, the layouts and the
rules that keep it portable are [simulator.md](simulator.md).
