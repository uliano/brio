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

## A framebuffer another process can watch

`brio/host/sim_display.hpp` publishes a framebuffer into POSIX shared
memory, so that a viewer in another process reads the same physical
pages at its own rate - which is what a display controller does on a
part that has one, scanning memory the program never waits for. It is a
model of the memory-mapped tier rather than an imitation: the drawing
surface is an ordinary `Framebuffer` over the mapped bytes, and nothing
in `brio/gfx/` learns where those bytes live.

**The name is the contract, never a path.** Linux exposes these objects
under `/dev/shm` and macOS exposes them nowhere in the filesystem, so a
viewer attaches with `shm_open` under the same name and the path is a
debugging convenience on one platform only. Two rules follow from the
same place: a name is short, because macOS caps it at 31 characters; and
a segment is SIZED ONCE at creation, because a second `ftruncate` fails
there - so a change of geometry is a new segment and never a resize.

**The layout**, all little-endian, pixels beginning at a fixed offset of
1024 bytes whatever the header grows to inside it:

| field | width | meaning |
|---|---|---|
| magic | 4 | `BRGX` |
| version | 2 | of this layout |
| header_bytes | 2 | where the pixels start |
| boot_id | 8 | drawn afresh at every creation |
| width, height | 2 each | pixels |
| stride | 2 | bytes per row |
| format | 1 | 1 = one bit per pixel, row-major, most significant bit leftmost; 8 = one byte per pixel |
| buffers, front | 1 each | 1 and 0 today |
| frame | 4 | bumped by `publish()` |
| palette_used | 2 | how many entries the format reads |
| palette | 256 x 3 | red, green, blue |

**Why the boot id earns its eight bytes.** After a segment is unlinked
and remade under the same name, a viewer's existing mapping still points
at the OLD object, which stays alive as long as it is referenced - so
nothing written inside that object could ever tell the viewer that
anything changed. A viewer therefore re-opens by name every so often,
compares the id, and remaps when it differs. That is also what lets it
stay open across rebuilds and across different programs at different
resolutions.

**The palette travels** because a byte on its own shows nothing: a
one-bit surface carries two colours, which is what lets an observer
render a panel as it really looks - white on blue, black on green -
instead of an idealized black and white.

**Tearing is honest.** With one buffer a viewer can read a row the
program is part way through writing, exactly as a real memory-mapped
panel tears unless something swaps its buffers at the right moment. The
`buffers` and `front` fields exist for the day that stops being
acceptable; adding them later would have changed the layout under every
viewer already written.
