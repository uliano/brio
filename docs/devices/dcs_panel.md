# The panel driver

This document is for a programmer who wants to draw on a display
controller that keeps its own pixels (the ILI9481 today,
[ili9481.md](ili9481.md)) with the drawing library of `gfx/`, or to
judge a drawing on one without looking at the glass. `DcsPanel<Traits,
Link>` (`devices/dcs_panel.hpp`) is a drawing surface over the panel: a
`Surface` in the sense of `gfx/surface.hpp`, the base every primitive of
the library draws through (a width, a height, a filled rectangle and a
run - one row of pixels, x increasing). It turns the picture to any of
four orientations, answers the module's mounting and colour order, sends
every drawing call as synchronous transactions over a link
([dcs_link.md](dcs_link.md)), and reads the frame memory back in the
picture's own coordinates. `Traits` is the controller's traits type (a
struct of constexpr facts, `brio::Ili9481`), `Link` any `DcsLink`.

The smallest use, a panel woken and turned, then a drawing (an excerpt
of the first two blocks of "How to use it" below, the lines that fill
the two prototype requests left out):

```cpp
using Panel = brio::DcsPanel<brio::Ili9481, PanelLink>;

PanelLink link{PanelLink::Config{write_prototype, read_prototype}};
Panel panel{link, brio::DcsModule{.column_mirror = true,
                                  .bgr = true,
                                  .wants_inversion = true}};
if (!panel.valid()) {
    // the link's mode is not one the controller takes: nothing will reach the bus
}

brio::Pin<'A', 2>::output();            // the reset line is a pad the board wires
auto reset = brio::Pin<'A', 2>::ref();  // named at run time, as a request names a select
// wait_ms is the program's: a void(uint16_t) that spends AT LEAST that many
// milliseconds (the controller's settling times, 10 to 150 ms). brio::delay_us
// is not it on the 32-bit targets: there it refuses a wait of a tick or more
// and spends no time (the AVR's has no cap).
if (!panel.reset_and_wake(reset, wait_ms)) {
    // a link verb was refused or failed: panel.link_failures() says how many
}
panel.rotation(brio::DcsRotation::r90);   // 480 x 320 now

brio::fill_rect(panel, 0, 0, panel.width(), panel.height(), 0x00000000u);
```

Line by line: `Panel` names the driver over the ILI9481's traits and a
link type; `link` is a serial link built from two prototype requests of
the family's SPI host ([dcs_link.md](dcs_link.md)); the `DcsModule` is
the module's own facts, handed over as the pins are - its glass mounted
mirrored, its colour channels crossed, and its need of the inversion
command; `valid()` says whether the link's SPI mode is one the
controller takes. The reset line is an ordinary output pad, passed by a
run-time reference; `reset_and_wake()` drives it low and high, wakes the
controller and sets its formats, spends the controller's four waits (10,
150, 150 and 25 ms, the traits' figures) through `wait_ms`, a callable
the program supplies and the driver never has of its own, and answers
false when a link transaction was refused or ran with a status other
than `spi_ok` - it cannot tell whether `wait_ms` waited. The contract
`wait_ms` must keep: at least the milliseconds asked, every time. A
loop on the platform's tick counter does it when the ticker runs and
interrupts are enabled, waiting `ticks_from_ms<P>(ms) + 1` ticks
(`kernel/time_event.hpp`'s conversion, one tick more because a count of
whole ticks can fall short of the physical interval by up to one);
`brio::delay_us` does not on any 32-bit target, since there it refuses a
wait of a tick or more and spends no time on it (the AVR's has no cap
and does wait). The four waits add up to 335 ms, during which a
dispatch that wakes a panel holds every other active object: the wake
belongs before the kernel runs, in `main()` with interrupts enabled, or
the program accepts the hold. `rotation()`
turns the picture a quarter and sends the new address mode at once, and
`fill_rect` is the library's own primitive,
drawing through the panel with no adapter. The sections below speak the
DCS command names of [dcs.md](dcs.md): CASET and PASET open a window,
RAMWR writes memory into it, MADCTL is the address mode whose three
walk bits (B5, B6, B7) set the order the pixels fill the window in.

What decides correctness even there:

- **Wake the panel before drawing.** `reset_and_wake()` (or `wake()`,
  where the board gives the program no reset line) takes the controller
  out of sleep, turns the display on, and states the pixel format and
  the address mode every later write assumes; a drawing before it
  assumes a format and an orientation nobody stated.
- **A link in a mode the controller does not take makes every verb
  refuse.** The ILI9481 takes SPI modes 0 and 3; a link in mode 1 or 2
  leaves `valid()` false, and the drawing calls, which return nothing,
  only count their refusals in `link_failures()`.
- **Every verb waits for its link transactions.** A full-screen fill is
  one window and a write per row, all on the wire before `fill_rect`
  returns; a program drawing from an active object's handler (an active
  object is a component with its own event queue, served one event at a
  time by the kernel) holds every other active object meanwhile.
- **The object carries its run buffers**, sized by the glass's long
  side: for the ILI9481 the object is 2892 bytes on an AVR128DB48 and
  2900 on a Cortex-M0+, so it is placed
  where that much fits - a static or global object, not on the stack of
  a part whose stack is two kilobytes ("The buffers are members"
  below).

The page goes on with the documents of record, what the design says
and what the driver does (the rotation map, the module, the oracle, the
refusals, the mode, the buffers), the types and verbs, how to use it,
what the host suite establishes, and what is not covered.

## Documents of record

None outside this repository, the driver being
brio's own. What it stands on is stated elsewhere and not repeated here:
the design in [../design/gfx.md](../design/gfx.md)'s "The command tier"
and its surface contract, the words in [dcs.md](dcs.md), what one
silicon makes of them in [ili9481.md](ili9481.md) (every rule of its
frame memory measured), and the bus in [dcs_link.md](dcs_link.md).
Header: `devices/dcs_panel.hpp`. Suite: `test/test_dcs_panel` (host, no
hardware), which stacks the driver over the serial link over a SPI host
made of RAM over a simulated ILI9481, and judges it twice - against that
panel's memory, and against the reference renderer of
`brio/host/gfx_reference.hpp`.

## What the design says, and what the driver does

**It is the fourth layer and it adds one thing: a logical surface over
the glass.** The vocabulary knows the words, the traits know what one
controller makes of them, the link carries bytes - and none of them
knows which way up the picture is. This file does, and that is the whole
of what it contributes besides the two `Surface` verbs.

**The direct shape, and only it.** Every verb is a synchronous link
transaction, complete on return: the shape of a bring-up, of a suite and
of a program that owns its bus. The tiled shape - primitives drawing
into a rectangle of RAM that a display active object streams as one
transfer - is a layer above this one; nothing here knows the kernel.

**A rotation is a window plus a walk, never a transform.** The address
mode's three walk bits reorder the counter inside the window a
CASET/PASET pair opened, and never move that window's origin
([ili9481.md](ili9481.md)). So the driver turns a logical rectangle into
a rectangle of the glass's own columns and pages, and then states the
walk bits that make the counter cross it in the order a logical row
runs. The rule that decides every bit: **a run of the logical surface -
one row, x increasing - must be one contiguous walk of the address
counter.** Each rotation therefore states the bit along its runs and
leaves the bits across them clear:

| Rotation | A row runs along | Walk bits | Straight module | Mirrored module |
|----------|------------------|-----------|-----------------|-----------------|
| r0 | the columns | B6 says which way | 0x00 | 0x40 |
| r90 | the pages | B5, and B7 says which way | 0x20 | 0x20 |
| r180 | the columns | B6 says which way | 0x40 | 0x00 |
| r270 | the pages | B5 + B7 | 0xA0 | 0xA0 |

No BGR bit and no display bit is ever set: the first acts on writes
alone and would leave every read-back crossed, which would cost the
oracle; the second is a wire a module may leave unconnected.

**Why the bits across a run may be left clear**, which is what makes one
code serve both verbs: the two Surface verbs never send a multi-row
block that is not uniform. A fill is one colour, so the order the
counter takes across the rows of a rectangle cannot show; a run has no
second row. A verb that streamed a two-dimensional image would have to
state those bits, and there is no such verb.

**The module's mirror belongs to the rotation map.** A glass mounted so
that memory column `columns - 1` shows at its left cannot be undone by
MADCTL's horizontal flip - measured, that bit does nothing on the module
this was written against - so the driver answers it where it can: the
memory column of a display column `d` is `columns - 1 - d`, which moves
the window under every rotation and flips the walk's direction only
where a row runs along the columns. That is why the mirror changes the
MADCTL code under r0 and r180 and not under r90 and r270.

**The colour order is answered in the bytes and not in a register.** A
module with its channels crossed gets B, G, R so that the eye sees the
logical colour, and a read-back is unpacked the same way - so a drawing
and its read-back agree, on any module, in any orientation.

**The oracle verb is the observer's.** `read_run()` hands back a logical
run in logical coordinates, through the same window and the same walk a
write used, which is what lets one test judge a drawing pixel for pixel
in every orientation with no eye on the glass. `get_pixel()` is that
verb for one pixel, and it exists so a panel satisfies `ReadableSurface`
and the reference renderer judges it with no adapter - **at the price of
a window and a read per pixel** on the wire. No primitive of `gfx/` may use
it, and the base concept having no such verb is the enforcement.

**A refusal is counted, because the Surface verbs are `void`.** They
have nowhere to report one, so `link_failures()` is where a refused
transaction goes instead of becoming a silently wrong picture. The
drawing, reading and turning verbs (`fill_rect()`, `write_run()`,
`read_run()`, `get_pixel()`, `rotation()`) stop at their first refusal -
a window half written is not a window - so a link that refuses
everything counts one for each of them; `wake()`, and so
`reset_and_wake()`, sends all five of its commands whatever each one
answers and counts five. A refusal here is a transaction the panel
asked for and did not get: one the panel itself withheld (`valid()`
false), a link verb answering false for a request the link or the engine
refused, or a transfer whose status was not `spi_ok`. What an update costs
in rectangles, runs and pixels is a different question and
`gfx/counting.hpp` is where it is asked; this driver counts nothing
else.

**The link's SPI mode is judged here, at construction.** Which modes a
controller's serial interface takes is a fact of the controller - the
traits' `serial_mode_accepted(mode)` ([ili9481.md](ili9481.md): modes 0
and 3, from the data sheet's own cycle) - and which mode a link runs in
is the link's (`write_mode()` / `read_mode()`,
[dcs_link.md](dcs_link.md)). This driver is the one layer with both in
hand, so it holds them against each other when it is built: a link whose
mode the traits refuse, on either tenure (the link's write or its read
transaction), leaves the panel with `valid()`
false, and every verb then refuses before it touches the bus - counted in
`link_failures()` as each verb counts a refusal, exactly as over a link
that refuses a wide frame - because a command clocked in a mode the
controller does not sample is not a wrong picture but no picture, and
no verb would ever say so. A link that shows no mode at all (a DSI host, a parallel bus) has
nothing to judge and the panel is valid as it stands; a serial link over a
controller whose traits do not state their modes is a compile error. The
comparison goes through `dcs_spi_mode_number()`, which reads a mode's
number off the spelling of any stratum's enum, so this file names no
family's enum and compiles over every host unchanged. What mode 3 buys
over mode 0 is a block's business and not this driver's: on the PL022 it
removes the 1.5 SCK gap the block leaves between frames in modes 0 and 2
- 19 per cent of an 8-bit transfer, measured in
[../rp2350/spi.md](../rp2350/spi.md) and stated for both families that
carry the block in [../pl022/README.md](../pl022/README.md); the other
blocks have no gap to lose.

**The buffers are members, sized by the glass's long side.** A run of
the rotated surface is as wide as the glass is tall, so a buffer sized
by the short side is correct until the first r90 and then writes past
itself; and a buffer on the stack is one a run can overflow on a part
whose stack is two kilobytes. On the ILI9481 that is 1440 bytes for the
row and 1441 for the read; with the link's reference, the module's
facts, the rotation, the validity flag and the four-byte failure count
the whole object is 2892 bytes on an AVR128DB48 (avr-g++ 16.2, `-Os`),
of that part's sixteen kilobytes, and 2900 on a Cortex-M0+, where the
reference is four bytes and the counter is aligned. They are the
glass's size and not the driver's choice, so a panel of 480 pixels a side is a panel the
smallest parts brio touches cannot drive this way, and a smaller glass
costs proportionally less.

## Types and verbs

| Type | What it is |
|------|------------|
| `DcsRotation` | `r0`, `r90`, `r180`, `r270` - the picture's turn over the glass, named for what an eye sees and not for a register's bits |
| `DcsModule` | The module's own facts, handed over like the pins: `column_mirror`, `bgr`, `wants_inversion` |
| `DcsRgb888` | The surface format: `Color` = `uint32_t` as 0x00RRGGBB, 24 bits. What a panel KEEPS of it is the controller's business |
| `DcsPanel<Traits, Link>` | The driver: a controller's traits over any `DcsLink`, constructed from a `Link&` and a `DcsModule` |

Two free functions, so the map can be read and checked without a bus:

| Function | What it answers |
|----------|-----------------|
| `dcs_rotation_madctl(r, column_mirror)` | The MADCTL code of the table above |
| `dcs_rotation_exchanges(r)` | Whether the rotation exchanges the surface's axes with the glass's |

The driver's verbs, by purpose:

| Verb | What it does |
|------|--------------|
| `valid()` | The link's mode is one the controller's serial interface takes, on both tenures - or the link has no mode to show. Every verb refuses while this is false, counted in `link_failures()` |
| `reset_and_wake(reset, delay_ms)` | The reset line low and high by the traits' times, then `wake()`. `reset` is a pin-like with `set()` and `clear()`, active LOW - every stratum's `PinRef` is one; `delay_ms` must spend at least the milliseconds asked, which the verb cannot check |
| `wake(delay_ms)` | SLPOUT, the wait, DISPON, the wait, the inversion the module asks for, COLMOD as a statement, MADCTL for the rotation in force |
| `rotation(r)` / `rotation()` | Put the logical surface in an orientation, and read the one in force. `width()`/`height()` follow |
| `madctl()` | The code the map produces for the rotation and the module in force |
| `width()`, `height()`, `fill_rect()`, `write_run()` | The `Surface` contract (`gfx/surface.hpp`) |
| `read_run(x, y, out)` | THE ORACLE: a logical run read back in logical order. Pixels outside the surface come back 0 |
| `get_pixel(x, y)` | The `ReadableSurface` refinement, one pixel through one window |
| `link_failures()` / `reset_counters()` | Link transactions asked for and not got |

The wake's numbers are the traits' and the waiting is the application's:
`delay_ms` is any callable `void(uint16_t)` that, on silicon, spends at
least the milliseconds asked every time (the driver cannot check it),
which is what lets a host test count the milliseconds instead of
spending them.

## How to use it

**The wake, on a family, with a real pin and the program's wait.** The link is
[dcs_link.md](dcs_link.md)'s, built over that stratum's `SpiHost` from
two prototype requests the application fills as it fills any request of
that bus - in a mode the controller's traits take; the reset line is a
pad named at run time, and the module's facts come from the board:

```cpp
using Panel = brio::DcsPanel<brio::Ili9481, PanelLink>;

PanelHost::Request write_prototype{};
write_prototype.cs = Cs::ref();
write_prototype.dc = Dc::ref();
write_prototype.clock = brio::SpiClock::div8;    // under the write ceiling
write_prototype.mode = brio::SpiMode::mode3;     // the ILI9481 takes 0 and 3; 3 has no frame gap on a PL022
PanelHost::Request read_prototype = write_prototype;
read_prototype.clock = brio::SpiClock::div32;    // under the READ ceiling, which is lower

PanelLink link{PanelLink::Config{write_prototype, read_prototype}};
Panel panel{link, brio::DcsModule{.column_mirror = true,
                                  .bgr = true,
                                  .wants_inversion = true}};
if (!panel.valid()) {
    // the link's mode is not one the controller takes: nothing will reach the bus
}

brio::Pin<'A', 2>::output();            // the reset line is a pad the board wires
auto reset = brio::Pin<'A', 2>::ref();  // named at run time, as a request names a select
// wait_ms is the program's: a void(uint16_t) that spends AT LEAST that many
// milliseconds (the controller's settling times, 10 to 150 ms). brio::delay_us
// is not it on the 32-bit targets: there it refuses a wait of a tick or more
// and spends no time (the AVR's has no cap).
if (!panel.reset_and_wake(reset, wait_ms)) {
    // a link verb was refused or failed: panel.link_failures() says how many
}
panel.rotation(brio::DcsRotation::r90);   // 480 x 320 now
```

**A drawing, through the library and nothing else.** The panel is a
`Surface`, so every primitive of `gfx/` works on it with no adapter and
no knowledge of what is underneath:

```cpp
brio::fill_rect(panel, 0, 0, panel.width(), panel.height(), 0x00000000u);
brio::round_rect(panel, 4, 4, 200, 60, 8, 0x00FFFFFFu);
brio::text<brio::Font5x7>(panel, 12, 12, "brio", 0x0000FF00u, 0x00000000u);
```

**The oracle, in a suite.** A run read back in logical coordinates is
what makes a drawing judgeable with no eye on the glass - and
`get_pixel()` is what lets the reference renderer do it for the whole
surface:

```cpp
uint32_t back[64];
if (panel.read_run(0, y, back)) {
    // back[i] is the logical pixel at (i, y), whatever the rotation
}

const brio::GfxDiff d = brio::compare(panel, reference);   // host only
REQUIRE(d.agree());
```

## Bench findings

None are this document's yet: every number below was established against
a panel made of RAM, and what a simulator cannot answer is whether the
belief it encodes is true (`../design/gfx.md`, "The three planes of
truth"). `test/test_dcs_panel` establishes, off any hardware:

- **The eight MADCTL codes** of the table above, and that no BGR bit and
  no display bit is ever set.
- **The picture's three corners land on the glass's corners**, under
  each of the four rotations, on a module whose glass is mirrored and
  whose channels are crossed AND on one with neither - read through the
  simulator's own `glass(d, line)`, which is what an eye would see.
- **The wake** drives its line low then high, spends the traits' four
  waits in that order, and leaves the panel out of sleep, displaying,
  inverted per the module, COLMOD 0x66 and MADCTL the map's r0 code.
- **A run written comes back byte-exact** on the six bits a pixel keeps,
  under all four rotations; so does a filled rectangle, row by row; so
  does a run as wide as the rotated surface's long side.
- **The clipping**: a rectangle half outside writes exactly the cells
  that are inside, a run starting left of zero loses its head and keeps
  its tail, a rectangle wholly outside touches the link not at all, and
  a pixel outside answers 0 without asking.
- **The cost**, which is design/gfx.md's own question: a `w` x `h` fill
  is exactly `h + 2` link transactions whatever `w` is (CASET, PASET,
  RAMWR, then a write memory continue a row), a run is 3, a read is 3,
  a pixel is 3.
- **The reference renderer agrees** with a drawing of four shapes - a
  segment, a circle, a filled round rectangle and a word of text - made
  through the panel under r0 and under r90 and read back a pixel at a
  time.
- **A link that refuses** everything: five refusals counted for
  `reset_and_wake()` (SLPOUT, DISPON, the inversion, COLMOD, MADCTL),
  one each for `fill_rect()`, `write_run()`, `read_run()`, `get_pixel()`
  and `rotation()`, nothing on the bus, nothing in the panel.
- **The link's mode against the traits**: a link in mode 0 or 3 leaves
  the panel valid and every verb open; one in mode 1 or 2 - on either
  tenure - is refused at construction, the panel behaving as it does
  over a link that refuses: the reset line still driven and the waits
  still spent, each refusal counted, nothing on the bus. The
  host made of RAM has no wire, so an accepted mode changes nothing in
  it; what the case proves is the gate and not the glass.

## Not covered yet

Driver gaps:

- **The TILED surface shape** - the primitives drawing into a rectangle
  of RAM that a display active object streams to the panel as one
  asynchronous transfer. Born with that active object, because its
  shape is decided by what a tile transfer needs; this file is the
  DIRECT shape and stays synchronous.
- **The memory-mapped shape.** Not missing: it is `gfx/surface.hpp`'s
  `Framebuffer` over the memory a display controller scans, and a
  panel driver has nothing to do with it.
- **A second controller's traits.** The rotation map, the window
  arithmetic and the read framing are all asked of the traits type, so
  a second controller is a second traits type and not a change here -
  but until one exists, "the traits answer it" is a claim with one
  witness. The NT35510 behind a DSI host is the intended second, and
  its memory read is already measured (`../stm32f4/dsi.md`).
- **A pixel format other than eighteen bits.** The driver packs with
  the vocabulary's `dcs_rgb666_pack` and states that in COLMOD, and a
  traits type with another width is refused at compile time. Declined
  until a panel here takes sixteen or twenty-four: the branch is
  cheap, a format nothing writes is a format nothing checks.
- **Partial mode, idle mode, the tearing effect line, brightness,
  gamma.** Each has its code in the vocabulary and no verb here, and
  each is declined until a program needs one: they change what the
  panel does with memory it already holds, which is a different
  question from where a pixel goes. The tearing effect line in
  particular is the tiled shape's business, since it exists to pace a
  transfer nothing here has.
- **The clock ceilings as a refusal.** The rate lives in the link's
  prototype requests and this driver never reads one, so nothing here
  refuses a bus faster than `write_clock_max_hz` or
  `read_clock_max_hz`. Where such a refusal belongs is not decided: a
  link that checked it would have to know which controller is at the
  other end, which is the one thing a link may not know.
- **The backlight and the panel's supply.** Board wires, not the
  driver's: the reset line is here because the CONTROLLER's wake
  sequence needs it and the traits state its times.
- **A judge that reads by the run.** `ReadableSurface` asks for one
  pixel, so the reference renderer's `compare()` costs a window and a
  read PER PIXEL - which is nothing on a framebuffer and a whole
  screen's worth of transactions on a panel, where `read_run()` would
  fetch a row for the same three. Judging a full screen on the glass
  therefore wants a run-shaped verb in the concept, with the per-pixel
  one as the fallback; that is a decision about `gfx/surface.hpp` and
  its reference renderer and not about this file, so it waits for the
  bench run that makes the price real.

Implemented, not bench-verified:

- **The whole of it, against glass.** Every expectation in the suite is
  the bench's own measurement of the controller, but the code that meets
  them has only ever run against RAM - and a driver and a simulator
  written from one reading of a command table agree on the same mistake.
  What would measure it is a bench run of this driver on the module:
  the measurements of [ili9481.md](ili9481.md)'s bench findings repeated
  through it, the memory read back through `read_run()` in each of the
  four rotations and the picture looked at once by a human.
- **Mode 3 on the glass.** The traits take it from the data sheet's own
  cycle and the gate here lets it through, but every byte the bench
  measured was clocked in mode 0 ([ili9481.md](ili9481.md)). What would
  measure it is the same bench run with the host's prototypes in mode 3:
  the
  write and read rate ladders judged by read-back, and the frame read
  back whole.
