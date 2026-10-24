# Target: host (`host/`)

The native build is a target like any other, and this is its
operational page: for a programmer who wants to run brio's test suites
on their own machine, for a driver author who wants to test a driver or
a service against a chip made of RAM before a board is on the desk, and
for whoever writes a host program with a screen and a panel. It says
what the host platform is, what runs on it, how the suites are built and
run, what a simulation is allowed to do, and where the simulator's
contract is.

## The host platform

`brio/host/platform.hpp` provides `HostPlatform`, an implementation of
the kernel's `Platform` concept (the set of requirements a platform
type meets, [../design/kernel.md](../design/kernel.md)) for a
single-threaded test process:

- a depth-counting `CriticalSection`: a test can assert that a
  lock-free path never took the guard, or that a guarded one released
  it;
- a test-controlled virtual clock behind `now()`, so time becomes
  deterministic arithmetic: advance it, call `TimeEvents<P>::process()`,
  assert what was posted;
- recording `idle()` and `break_here()` (counters, no sleeping, no
  trap);
- `in_handler()` answered from a flag, `in_isr`, that a test's harness
  raises around a call it wants treated as an interrupt handler's;
- `panic_record()` in ordinary static storage, cleared by `reset()`;
- `ticks_per_second = 1000` (the identity case for millisecond
  conversions) and `atomic_width = 4` (every brio ring index is
  lock-free here). A test that needs the other paths, a non-dividing
  rate or the guarded ring, defines a small local platform of its own.

The same header provides `HostCore<n>`, the two-core host of
`test_inbox`: the same platform told apart by type, with a test-set
current core and a counting doorbell.

Because the kernel and `util/` are pure logic templated on the
platform, everything above the drivers runs here unchanged: the queues,
the scheduler, the state-machine contract, time events, panic, `Ring`,
`LineReader`, `BusMaster` (with a fake bus engine). This is where the
things that are hard to provoke on silicon are tested first: queue
overflow and its counters, entry/exit ordering, drift-free periodic
re-arm, scan priority, many-producer stress with simulated producers,
wrap-around deadlines.

## Running the suites

```bash
cd test
cmake --preset host                 # configure, once (writes build-cmake/host)
cmake --build --preset host         # build every suite; the test preset does not
ctest --preset host                 # all suites
ctest --preset host -R test_fsm     # one suite
```

- The framework is [doctest](https://github.com/doctest/doctest)
  (vendored, `third_party/doctest/`), one `test/test_<subject>/main.cpp`
  per suite, each with `DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN`.
- `test/CMakeLists.txt` is a CMake project of its own, a peer of the
  cross-build projects rather than a part of them (its own
  `CMakePresets.json`, the host g++, no cross toolchain: a CMake
  configure has exactly one compiler), so no cross build is in reach of
  a probe here in the first place.
- No hardware is needed; the host compiler must speak gnu++23, the same
  standard as the cross builds, since the code is identical.

The host's own programs, the ones that run rather than run and exit
(the front panel of a bench power supply, for instance), are the
`host/` project beside
`test/`, with the same `// build:` grammar and the same flags, and
nothing in it registered with ctest.

## Simulated devices, and what they are allowed

Beside the platform, this stratum carries simulations of things a test
cannot otherwise provoke: a flash with its wear, its power cuts and its
reflash; a USB controller answering scripted control transfers; the
reference implementations a test compares a real one against.

**The whole standard library is allowed inside them, heap included.**
Above the concept, host code is brio code and pays what brio pays;
below it, in a simulation's own guts, modelling a constraint faithfully
is worth more than modelling it cheaply, and none of that code is going
anywhere near a silicon. What brio sees is the concept the simulation
satisfies, and nothing above that concept can tell how it was reached.

The same reasoning puts a judge here rather than beside the code it
judges: a reference renderer, a reference decoder, anything a test
measures an optimized implementation against. From this stratum it
cannot be linked into a target image even by accident, which is a
stronger guarantee than remembering not to.

### A command panel made of RAM

`sim_dcs_panel.hpp` is a command panel in the two layers the command
tier prescribes ([../design/gfx.md](../design/gfx.md)):
`SimDcsPanel<Traits>`, the core whose seam is the DCS transaction (the
frame memory, the window and the walk its address mode selects, the
read pointer that counts the bytes clocked), and `SimDcsSerial<Core>`,
the framing adapter that puts the four-wire serial interface in front of
it at the level of bytes, with the dummy byte and the one-bit-late
stream a controller's traits ask for. Every rule in it is a bench
measurement, down to the window's edges (the counter wraps to the
window's first pixel, for a write and for a read), the two address
counters that only RAMWR and RAMRD move, the pixel cut short at the
close of a write (it is dropped) and the windows the axes cannot hold (a
start beyond the axis is ignored; an end beyond it or a range backwards
is taken and the window is no window); the two corners the bench has not
reached are marked ASSUMPTION beside the code. That discipline is the
point: a simulator written from a command table agrees with a driver
written from the same table on the same mistake, and what this one
judges is whether the driver does what the part was measured to do,
never whether that belief is right.

### A SPI host made of RAM

`sim_spi_host.hpp` is what drives that panel: a SPI host whose seam is
the request, offering the arbiter and a device driver the same static
surface a family's `SpiHost<n>` does with no register block behind it.
It is not the same kind of object as `sim_pl022.hpp`, and the difference
is the question each answers. A simulated PL022 has to be the chip, the
registers at their offsets, their reset values, the order of the acts of
a bring-up, because what it proves is that one driver knows no chip.
This one proves nothing about a driver of the silicon: it is the world
on the other side of a bus, so that a device driver can be judged
against a device made of RAM.
[../design/simulation.md](../design/simulation.md) places it: a device
the program reaches through its own driver, never a channel into the
kernel.

It models the two phases of the descriptor in one select window with
the D/C line flipping between them, the select active low, a null `tx`
clocking 0xFF and a null `rx` discarding, a frame of two bytes where the
request asks for one, and a select line with nothing on it reading
0xFF, a bus with no device on it, counted and not an error. Devices are
attached to a select pin by `attach()` and held in a small type-erased
table, so a panel's framing adapter and a counting stub can sit on one
bus. Both completion styles of the contract are there, because the
arbiter distinguishes them: in `immediate` mode every request completes
inside `start()`, and in `deferred` mode an unpolled one is held,
`start()` returns false, and `finish()` performs it and hands back the
status the app's ISR glue would post, which is the shape a test of an
asynchronous transfer needs. A byte-level trace ring records what the
wires carried, which is how a test asserts a D/C choreography, and each
pin remembers when it was last driven on the same ruler the bytes are
stamped with, which is how it asserts that the select fell before the
first byte. What it does not model is the wire and the time on it: no
clock rate, no mode, no bit; the `cs_setup_us` is counted and never
spent, and a mode the device disagrees with changes nothing. A test that
wants those wants silicon. `test/test_dcs_link` is its suite, and the
one that drives the DCS link
([../devices/dcs_link.md](../devices/dcs_link.md)) over it.

## The simulator

A host program can put its pixels on a screen and take its input from a
panel, through a second process that shows the one and drives the
other. Both directions are one mechanism, a named region of memory the
program owns, and the contract between the two processes, the layouts
and the rules that keep it portable are [simulator.md](simulator.md).
