# The link a command panel is spoken to over

Documents of record: none outside this repository. The link is a
contract of brio's own, stated in
[../design/gfx.md](../design/gfx.md)'s "The command tier" and shaped, on
its serial realization, by the transaction descriptor of
[../design/spi-bus.md](../design/spi-bus.md). Header:
`devices/dcs_link.hpp`; the vocabulary carried over it is
`devices/dcs.hpp` ([dcs.md](dcs.md)) and a controller's traits are
[ili9481.md](ili9481.md)'s. Suite: `test/test_dcs_link` (host, no
hardware), which drives the link over a SPI host made of RAM
(`brio/host/sim_spi_host.hpp`, [../host/README.md](../host/README.md))
against the simulated ILI9481.

## What the design says

**A link carries a command and the bytes that belong with it, in one
tenure of whatever bus it owns.** That is the whole of its job. It knows
no command code, no pixel format, no window and no dummy byte, which is
what lets one panel driver sit over a four-wire serial bus, an 8080
parallel bus and a DSI host without a branch: the four layers of the
command tier are the vocabulary, a controller's traits, the link, and
the driver that is the three of them together.

**A read comes back RAW.** The bytes as the link clocked them - a dummy
clock, a dummy byte, a line nobody drove, all of it still in the buffer.
What a controller makes of them is that controller's `read_framing`
([dcs.md](dcs.md)'s `DcsReadFraming`) and the driver applies it. A link
that realigned a stream would have to know which controller is on the
other end, and the next link would realign a stream that was never out
of step.

**A memory write is not a command, even where it is the same bytes.**
The two verbs exist because on some links they are two things: a
parallel bus stores a pixel as one word and a parameter as a byte, and
DSI sends a long packet for the one and a short packet for the other.
The serial link implements both with one function and says so; a driver
that spelled them the same would be rewritten at the second link.

**The synchronous face is the whole of what exists.** Every verb
completes on return - `true` = done and accepted, `false` = refused or
failed. That is the DIRECT surface shape of the command tier (a
bring-up, a suite, a program that owns its bus), and the memory-mapped
shape needs no link for its pixels at all. The asynchronous face waits
for the tiled pipeline, for the reason in the gap list below.

**The configuration of the serial link is two PROTOTYPE REQUESTS.** The
strata do not spell a bus transaction the same way - the rate is an enum
on most, a `uint8_t baud` divisor on the SAM, a prescaler pair on the
PL022, and only some Requests carry a frame size - so the link reads
none of those fields. The application fills a request of its own bus
exactly as it would fill any other, and a verb copies it and supplies
only what a transaction of its own needs. There are two because a
controller's read ceiling is not its write ceiling: a third of it on the
ILI9481.

## Types and verbs

| Type | What it is |
|------|------------|
| `DcsLink` | The concept a panel driver is written over: three verbs, all complete on return |
| `DcsSerialLink<Host>` | The four-wire serial realization over ANY stratum's `SpiHost` |
| `DcsSerialLink<Host>::Config` | The two prototype requests: `write` and `read` |

The concept's three verbs, each answering `bool`:

| Verb | What it does |
|------|--------------|
| `command(c, parameters)` | A command with its parameter bytes, of which there may be none |
| `write(c, bytes)` | A memory write: the command, then the pixel bytes |
| `read(c, in)` | The command, then `in.size()` bytes clocked in, RAW |

What the serial realization adds:

| Verb | What it says |
|------|--------------|
| `valid()` | Both prototypes carry a byte-wide frame. A verb refuses while this is false |
| `write_prototype_valid()`, `read_prototype_valid()` | Which of the two the constructor refused |
| `status()` | The engine's own code for the last transaction it ran - what to read after a `false` |

Three rules the realization keeps, each of which a suite checks:

- **A verb is `Host::start(r) && Host::status() == spi_ok`.** A polled
  request completes inside `start()` by the arbiter's contract
  ([../design/spi-bus.md](../design/spi-bus.md)), so a `false` from
  `start()` on one is a REFUSAL, and the status carries what the engine
  made of the transaction it did run - a polled transfer over DMA
  engines can come back `spi_dma_fault` from inside `start()`.
- **The command byte is a member of the link.** A Request lends its
  `cmd` buffer under `Lease::reply`; on a polled request the reply is
  inside `start()`, so the byte must outlive the call and belong to the
  link - not to a function that returns before the loan is over, and not
  to a static two links would share.
- **The frame must be a byte.** Every engine that has a frame size
  counts `cmd_len` and `len` in FRAMES and this link counts BYTES, so a
  prototype whose frame is wider is refused at construction. Where a
  Request has no frame size at all there is nothing to check, and the
  link detects which case it is with a `requires`.

## How to use it

A link over a family's own host - the select and the D/C pin, the setup
time, the rate, the mode and the frame size are that stratum's, and the
link reads none of them:

```cpp
using PanelHost = brio::SpiHost<1, panel_pins>;
using PanelLink = brio::DcsSerialLink<PanelHost>;

PanelHost::Request write{};
write.cs = Cs::ref();
write.dc = Dc::ref();
write.cs_setup_us = 1;
write.clock = brio::SpiClock::div8;       // under the write ceiling
write.mode = brio::SpiMode::mode0;
write.bits = brio::SpiDataSize::bits8;    // where the stratum has one
PanelHost::Request read = write;
read.clock = brio::SpiClock::div32;       // under the READ ceiling, which is lower

PanelLink link{PanelLink::Config{write, read}};
```

The three verbs, with the vocabulary's own packers:

```cpp
link.command(brio::Dcs::slpout, {});                        // no parameters
link.command(brio::Dcs::caset, brio::dcs_window(x0, x1));   // a window's edge
link.write(brio::Dcs::ramwr, std::span<const uint8_t>(pixels, n * 3));

uint8_t raw[1 + 3 * n];
link.read(brio::Dcs::ramrd, raw);   // raw[0] is the dummy byte the traits name
```

The same link over the host made of RAM, which is what a suite does -
one line differs, and it is the Host type:

```cpp
using SimLink = brio::DcsSerialLink<brio::SimSpiHost<0>>;

brio::SimSpiHost<0>::attach(panel_cs, panel_front);   // the device on the line
SimLink link{SimLink::Config{write, read}};           // requests of the sim's bus
```

## Bench findings

None are this document's: the link moves no byte of its own, and what it
adds to a transaction - one command byte, a length, a completion style -
is judged where every byte of it can be seen. `test/test_dcs_link`
establishes, off any hardware, that the link's three verbs reach a
simulated ILI9481 with the bytes the bench measured: the device code
back raw and one dummy clock late (`01 02 4A 40 FF`), the
single-parameter reads aligned, RDDID and RDDST all ones, a block
written and read back pixel for pixel behind its `0x80`, a read memory
continue picking up where the clocks stopped, the BGR bit acting on the
write alone, and a prototype with a wider frame refused with no
transaction reaching the panel.

What the same suite establishes about the host under it - the select
falling before the first byte and rising after the last, the D/C low for
exactly `cmd_len` frames and high for `len`, a null `tx` clocking 0xFF,
a select line with no device reading 0xFF, and the arbiter of
`util/bus_master.hpp` driving it in both completion styles - is
[../host/README.md](../host/README.md)'s.

## Not covered yet

Driver gaps:

- **The asynchronous face** - a memory write answered later instead of
  on return. Born with the TILED surface shape, because its shape is
  decided by what a tile transfer needs and not by what a link could
  offer; on this realization it is the same Request with `polled` false
  and a real `ReplyTo<SpiDone>`, posted to a `SpiBus` arbiter instead of
  handed to the engine, the tile on loan until the reply lands. The
  prototype pair above is already the configuration it would take.
- **The DSI realization and the parallel one.** Each is born with the
  panel that needs it. The DSI host's `dcs_write(command, parameters,
  count)` / `dcs_read(command, buf, len)` pair
  (`brio/stm32f4/dsi.hpp`) satisfies the three verbs through a thin
  adapter that reshapes nothing, which is the check the concept was
  written against; the 8080 bus, where a store IS the transfer, waits
  for a panel wired to one.
- **Where a read's FRAMING belongs.** Today it is a fact of the
  CONTROLLER - `read_framing` is a traits member - and the link is
  silent about it. That is right for the four-wire serial interface,
  where the dummy clock is the controller's own; it may be wrong in
  general, since the same controller behind a DSI host answers a read
  with no dummy at all. Deciding it from one controller on one link
  would be deciding it from the only case there is, so it waits for the
  second traits type over another link, and the driver is where the
  choice would be felt.
- **A frame wider than a byte.** Declined until a panel takes sixteen
  bits a pixel over sixteen-bit frames: a link that counted frames would
  have to know the pixel format to turn a byte span into a length, which
  is the one thing this file may not know. The refusal is a run-time
  answer (`valid()`), not a compile-time one, because the prototype is a
  value the application fills.
- **The reset line, the backlight and the tearing effect pin.** Not on
  the link and not missing from it: they are the panel's wires, not the
  bus's, and they belong to the driver that owns the wake sequence.

Implemented, not bench-verified: the serial link against the glass. Its
every byte is the bench's already - the suite's expectations were
measured on an ILI9481 module over a four-wire serial interface - but
the code that produces them has only ever run against RAM. What would
measure it is the panel driver on that module, where the link replaces
the transaction function a bring-up writes by hand.
