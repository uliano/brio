# IP stratum: the ARM PrimeCell SSP (`pl022/`)

This page is for a programmer who uses an SPI bus on the RP2040 or the
RP2350 and wants to know what the driver under it does, for whoever
reads or changes that driver, and for whoever brings up another chip
that carries the same block. It is both the stratum's operational page
and the document of its one header, `brio/pl022/spi.hpp`. A stratum is
a directory of `brio/` that is one architectural layer; this one is an
IP stratum, named for a peripheral design that several vendors license
into their chips (an "IP block") rather than for a chip, and written
once for every family that carries it.

An application never names this stratum. It writes the family's names,
which are aliases of the three templates here kept in the family's own
`spi.hpp`, and the family supplies the chip's facts through a traits
type - a struct of types, constants and static functions the templates
read, so that they know the chip without including any of its headers:

| Who | Traits type | Where | The public names, aliases of the templates here |
|-----|-------------|-------|---------------------------------------------------|
| the RP2040 | `Rp2040Pl022` | `brio/rp2040/spi.hpp` | `Pl022<n>` = `Pl022Ssp<Rp2040Pl022, n>`; `SpiHost<n, pins, TxEngine, RxEngine, Placement>` = `Pl022Host<Rp2040Pl022, n, pins, ...>`; `SpiClient<n, pins>` = `Pl022Client<Rp2040Pl022, n, pins>` |
| the RP2350, both architectures | `Rp2350Pl022` | `brio/rp2350/spi.hpp` | the same three names over `Rp2350Pl022` |
| the host suite | `SimPl022` | `brio/host/sim_pl022.hpp` | none: the suite names the templates itself |

On both chips the traits give two instances, 8-frame FIFOs and SSPCLK
as `clk_peri` (the clock type's `pclk_hz`), and the figures the host
decides a short request from (`costs`, measured on each family's own
cores: "The short asynchronous request" below). The smallest use, a
host on two DMA channels under the bus arbiter on the RP2040 as its own
document spells it (an excerpt of [../rp2040/spi.md](../rp2040/spi.md),
"How to use it"):

```cpp
constexpr brio::SpiPins pins{.sck = 18, .tx = 19, .rx = 16};
using Bus = brio::SpiHost<0, pins, brio::DmaTxEngine<4, uint16_t>,
                          brio::DmaRxEngine<5, uint16_t>>;
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
using Cs = brio::Pin<17>;

brio::Dma::init();                                    // before the host binds its engines
Bus::init(clock, 8'000'000);                          // a ceiling for the whole bus
Cs::output(true);

Bus::Request r{};
r.cs = Cs::ref();
r.cmd = brio::lend<brio::Lease::reply>(cmd);  r.cmd_len = 1;
r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
r.clock = brio::SpiClocks::div4;                      // 31.25 MHz at 125
r.mode = brio::SpiMode::mode0;
r.reply = brio::reply_to<Display, brio::SpiDone>();
brio::post<Spi>(r);                                   // asynchronous: polled stays false

extern "C" void isr_dma_0() {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

Line by line: `pins` names the clock, transmit and receive pads (no
select: the select is a GPIO the request names); `Bus` is the family's
`SpiHost` on instance 0, the engine, with a transmit and a receive DMA
engine on channels 4 and 5 (half-word engines, the data register's
width), reporting on the controller's line 0; `Dma::init()` releases the
DMA controller, which the host's `init()` binds its engines on; `Spi` is
the arbiter, an active
object (a component with its own event queue, served one event at a
time by the kernel, here over the program's platform `P`) that takes
requests from every active object that uses the bus in arrival order, with room for
four waiting requests; `Cs` is the device's select pad, driven high
(idle) before any request. A request names the select, lends a one-byte command and a
64-byte data buffer (a loan under `Lease::reply`, kernel/borrowed.hpp:
the arbiter and the engine may use them until the reply), sets the
rate as a prescaler pair (`div4` is 31.25 MHz at a 125 MHz SSPCLK, and
the host clamps a request to the bus's ceiling, here 8 MHz), the SPI
mode, and the active object the `SpiDone` reply goes to; it is posted
to the arbiter, and `post` copies the request into the arbiter's queue,
so `r` itself may go out of scope - what it lends may not. The request is
ASYNCHRONOUS (`polled` stays false), and on this host that means the DMA
engines: the 64 data frames ride them, the core free for the wire's time
(the one command frame, too short to repay a block of its own, goes out
polled inside `start()` first). The
handler is the program's, under the name the family's startup code gives
the DMA controller's line 0 (`isr_dma_0` on both RP chips): when the
engine's DMA body returns true, the transaction has just completed
(select released), and the handler posts the engine's status to the
arbiter, which sends the reply.

What decides correctness even there: **the buffers a request lends
stay untouched by the requester until its `SpiDone` arrives**; the select
is high before the first request; for an asynchronous request the DMA
line's vector is bound and posts `TransferDone`, or the arbiter never
learns that a transaction ended (a polled request needs no vector); the
arbiter is in the program's list of active objects; and
`init()` runs after the clock's and after `Dma::init()`, since the
ceiling it is given is turned into a divisor of the rate the clock type
claims for SSPCLK and the engines are bound on a released controller. A
host with no engines bound runs polled requests alone and answers an
asynchronous one `spi_rejected`, at once and with nothing on the wire;
`static_assert(Bus::has_dma)` in the program makes that a compile error
instead. The bus vocabulary itself (the Request, the reply, the status
codes) is
[../design/spi-bus.md](../design/spi-bus.md)'s; what an active object
is and how a program lists them is the learning route's
([../learn/introduction.md](../learn/introduction.md)).

The host runs a request one of the two ways
[../design/spi-bus.md](../design/spi-bus.md) gives every SPI host ("Two
ways to run a request"): a request marked `polled` completes inside
`start()`, the arbiter sending its reply in the same dispatch; any other
is ASYNCHRONOUS, and asynchronous is the DMA engines - `start()` launches
them and returns, and the engines' one interrupt a block ends it. Three
rules complete that, each described below ("The engine slots", "The
short asynchronous request"): with no engines bound the request is
refused (`spi_rejected`, nothing on the wire); a request too short to
repay the engines' fixed cost at its rate runs polled and completes
inside `start()`; and one the engines cannot carry - 16-bit frames on
byte engines, or on half-word engines with a buffer not aligned to a
half-word - runs polled inside `start()` whichever way it asked. A
request marked `polled` runs an explicit loop at any length and whether
engines are bound or not, so it needs no interrupt vector; a long read
that wants the wire's rate asks asynchronously. The slots are empty by
default (`NoDmaEngine` on both RP chips). The host arms no source of the
block's own interrupt line: that vector is a client's.

The page goes on with what the stratum is for, the documents of record,
what lives here, what a family owes (the traits type member by member),
the include contract, the engine slots, the short asynchronous request,
the host above the wire (its cost and shape, measured), what stays per
family, the host suite that proves the file knows no chip, and what is
not covered.

## What the stratum is

A directory under `brio/` named for a peripheral design instead of for a
silicon: the ARM PrimeCell Synchronous Serial Port (PL022), a block
licensed by more than one vendor and dropped into their chips unchanged,
register for register. It sits exactly where a core stratum sits - above
`util/`, below the families ([../design/overview.md](../design/overview.md),
"A core stratum sits between util/ and the families that share a core") -
and it holds the whole of the chapter that is ARM's: the resource over
the block's programmer's model, the host engine `SpiBus` drives and the
client surface, with their measured facts and their ISR bodies.

It exists under the discipline that governs the core stratum (the
stratum a processor core's families share, `cortexm/`): what more than
one family carries is factored out when a second one carries it,
with both copies in hand, and the extraction is gated by the images -
every release image of the family that had it first is byte-identical
before and after. What makes the PL022 worth a stratum of its own, where
a merely similar peripheral would not be, is that its register
description is not similar between the chips that carry it but
identical: the same names, the same offsets, the same bits, so there is
nothing to reconcile and nothing to gain by writing it twice.

## The documents of record

ARM DDI 0194, the *PrimeCell Synchronous Serial Port (PL022) Technical
Reference Manual*, which is what a family's data sheet excerpts when it
says its SPI is a PL022 (the RP2040 data sheet does, in its 4.4, and
names the revision it carries: r1p4). That manual is not on this
project's desk, so nothing here is cited by its section: the facts below
are named by register, and each family's own document carries its data
sheet's chapter and verse for the same thing. A family's peculiarities -
how deep the FIFOs were synthesized, which pads a signal reaches, which
interrupt controller carries the line, what SOD does to a pad - are not
in that manual either, which is exactly why they are the concept below.

The bus vocabulary is not this stratum's and is not restated here: the
Request's arbitration, its reply and its status codes are
[../design/spi-bus.md](../design/spi-bus.md)'s, realized in
`util/spi_bus.hpp` over `util/bus_master.hpp`, and shared verbatim by
every family that drives an SPI. This file takes them.

## What lives here, and what does not

- Here: the mode and framing vocabulary (`SpiMode` with `spi_mode_cpol` /
  `spi_mode_cpha`, `SpiFormat`, `SpiDataSize` with `spi_data_bits`,
  `spi_frame_mask` and `spi_frame_is_halfword`, `SpiRole`), the
  prescaler pair as pure arithmetic (`SpiClock`, `SpiClocks`,
  `spi_sck_hz`, `spi_clock_for`), the whole configuration and the two
  control words it becomes (`SpiConfig`, `spi_config_valid`,
  `spi_cr0_of`, `spi_cr1_of`) and the one word a host compares a
  request against (`spi_control_word`), every register's bit layout as constants
  (`SpiControl0`, `SpiControl1`, `SpiDataField`, `SpiFlag`,
  `SpiInterrupt`, `SpiDmaControl`), the engine's own status code
  `spi_dma_fault`, the short request's arithmetic (`Pl022Costs`,
  `pl022_frame_cycles`, `pl022_async_min_frames`), the resource
  `Pl022Ssp<Chip, n>`, the
  host engine `Pl022Host<Chip, n, pins, TxEngine, RxEngine>` with its
  Request, its two optional DMA engine slots and its one interrupt body
  (the DMA line's), and the client `Pl022Client<Chip, n, pins>`.
- Not here, by design: which pads carry the signals and under which
  function, how the block is taken out of reset, which interrupt
  controller owns the line and what it calls it, which rate of the
  family's clock tree is SSPCLK, what a DMA request is called, how deep
  the FIFOs are, what a pin driven at run time is, and what a
  microsecond busy-wait is made of. Those are the concept, and each
  family answers them in its own header - where the public names also
  live, so an application never spells a chip-traits type.

## What a family owes: the `Pl022Chip` concept

One traits type, stated in the family's own `spi.hpp` and checked there
with a `static_assert` (`Rp2040Pl022` and `Rp2350Pl022`; the host's
`SimPl022` is checked the same way in its own header, the table at the
top of this page). Every member is here because a line of the driver
needs it; nothing is here for symmetry.

The types:

| Member | Why it exists |
|--------|---------------|
| `Regs` | the register block of one instance, with the PL022's own member names (`SSPCR0`, `SSPCR1`, `SSPDR`, `SSPSR`, ...) - the driver reads and writes them by name, so a family whose device description spells them otherwise hands over a struct of its own laid over the block, which costs nothing. What the FIELDS inside them are is not asked: the bit layout is the IP's and is stated here |
| `Irq` | what this family's interrupt controller calls a line: an enumerator on one target, a number on another |
| `Interrupts` | that controller, with `enable` / `disable` - the driver never names an NVIC, because a family may carry one under one architecture and something else under another. It asks for no third verb: nothing here ever pends a line |
| `Pins` | the family's pin-set type, one pad per signal - a pin table is a chip's, never an IP's |
| `PinRef` | a pin named at run time, with `set()` and `clear()`: a Request's chip select and its D/C line are the APPLICATION's pins, chosen per transaction, and never the peripheral's own |
| `DmaRequest` | what this family's DMA calls a peripheral request |
| `DelayRate` | the family's microsecond busy-wait as the precomputed factor a caller stores once, so that no division runs while a chip select settles. The driver calls the verb that belongs to that type - written unqualified and found where the type is - which is how a busy-wait made of a core's own counter reaches a file that must know no core. The verb itself is the second concept below |

The constants:

| Member | Why it exists |
|--------|---------------|
| `instances` | how many of the block this family carries: the resource refuses a number past it |
| `fifo_depth` | the two FIFOs' depth, which is a synthesis parameter of the PL022 and therefore the family's fact, not this file's. It is the polled loop's window (how many frames may be in flight) and the client's `frames_ahead` |
| `no_pad` | what the family's pin set puts in a signal it does not route: a host with no receive pad is a write-only bus, and a client with no transmit pad is a listener |
| `costs` | a `Pl022Costs`: what the host's two ways cost the family's core - E - F for a read and for a write (the engines' fixed cost beyond a polled transaction's fixed part) and the two polled loops' turns - from which a host with engines decides which asynchronous request is too short to repay them ("The short asynchronous request"). MEASURED on the family's own cores and stated with their arithmetic in the family's document: numbers of the core and the flash around the block, never of the block |

The per-instance facts, each a template on the instance number so the
answer is a compile-time constant - a register address, a line, a
request:

| Member | Why it exists |
|--------|---------------|
| `regs<i>()` | where instance i's block sits |
| `irq<i>()` | its interrupt line |
| `reset<i>()`, `released<i>()`, `hold<i>()` | the block from its reset state, whether it is out, and back into it: a reset controller on one family, a clock gate on another |
| `tx_request<i>()`, `rx_request<i>()` | the two requests a host hands its engines |

The verbs over a register, over the pads and over time:

| Member | Why it exists |
|--------|---------------|
| `set_bits(reg, bits)`, `clear_bits(reg, bits)` | one write that sets or clears bits with no read-modify-write: `SSPCR1` carries the enable and the two live bits, and the interrupt mask is touched from two contexts. A family with atomic register aliases uses them; one without does the read-modify-write under its own guard |
| `pins_valid(n, pins)` | is this pin set legal for this instance - the family's pin table, as a constexpr predicate the host's and the client's `static_assert` call |
| `sck_pad(pins)`, `tx_pad(pins)`, `rx_pad(pins)`, `cs_pad(pins)` | which pad number carries each signal, or `no_pad` |
| `Pad<pin>` | the pad as a type, with the three verbs the driver calls on one: hand it over (`function(code, config)`), take it back (`release()`) and read its level (`read()`, which is how a client answers `selected()`) |
| `pad_function` | the function code that routes an SPI to a pad, opaque to this file |
| `no_function`, `dark_pad_config()` | a CLIENT's answer line while it listens dark: handed to no function, so nothing drives it, with a setup that carries no pull. Not the pad's reset state - that carries a pull on the families this file has met (a pull-down on both RP chips), and a far host's receive pad carries a pull-up, so a dark line released to reset sat between the two (measured on the RP pair: the far host read neither level) |
| `sck_pad_config()`, `tx_pad_config()`, `host_rx_pad_config()`, `client_rx_pad_config()`, `cs_pad_config()` | the electrical setup each signal wants. They are functions returning a value rather than constants, so the driver materializes the configuration at the call, exactly as a family's own driver would have written it there - no object's address is taken and the call folds away. The two receive pads are asked separately because they are two electrical situations and not one: a host's answer line may have nothing driving it (no client on the bus, or one listening dark), while a client's command line is driven by the host whenever it matters |
| `engines_distinct<Tx, Rx>()` | two engines of one host must not name one DMA channel; what "the same channel" means is the family's DMA's |
| `delay_rate(hz)` | the busy-wait's factor for a rate, recomputed at every clock change so the `cs_setup_us` of a Request costs no division |

And two things the concept above names but cannot check itself, each
with a concept of its own and a `static_assert` where the thing is in
hand - both of them on the host, which is the only party that needs
either:

| Member | Why it exists |
|--------|---------------|
| `sspclk_hz(clock)` | which rate of this family's tree is SSPCLK - the rate the bit rate is divided from. It is a template over the family's own clock types, of which this file knows none, so it is `Pl022ChipClock<Chip, Clock>`, checked inside `init()` where a clock is in hand |
| `delay_us(rate, us)` | the verb over `DelayRate`. It is not a member of the traits type at all but a free function beside it, found by argument-dependent lookup on the type the traits hand over, so it is `Pl022ChipDelay<Chip>`, checked in the host's own body |

## The include contract

`pl022/spi.hpp` includes `kernel/borrowed.hpp`, `kernel/post.hpp`,
`util/clock.hpp` and `util/spi_bus.hpp`, and nothing else: no vendor
header, no family header, no device include, and in particular no core
stratum - a PL022 may sit on a core `brio/cortexm/` knows nothing about,
which is why the microsecond busy-wait arrives as a type of the family's
rather than as an include. It may therefore be included anywhere, in any
order, and a family's `spi.hpp` includes it beside its own headers.
Apps and family drivers keep including the family's header, never this
one - the family is where the public names are.

It follows that this directory needs no `.clangd` fragment of its own:
the framework default (`brio/.clangd`, the host test project's database)
is the right one, because this file really does compile on the host -
which is also what makes the host suite below possible.

## The engine slots

A transmit and a receive engine, both or neither, and with them the
host's asynchronous way (`has_dma`). A phase moves as one block each
way: the data phase, and the command phase when it repays a block of its
own or is the whole request - its completion then raises D/C and launches
the data block from the DMA line's body - while a shorter command phase
runs polled inside `start()` ahead of the data block. ONE INTERRUPT A
BLOCK - every frame that came back was sent, so the receive block's
completion proves the transmit block's, and the host arms its transmit
engine to report a bus error and nothing else (`arm(data, request,
false, Engine::Report::errors)`); the owner then completes the transmit
block itself. THE BEAT IS THE FRAME: SSPDR is sixteen bits wide, so the
engines' element is `uint8_t` or `uint16_t`, the same on both. Half-word
engines carry 8-bit frames at a byte beat and 16-bit frames at a
half-word beat, the Request's byte buffers handed over as runs of the
frame - two bytes low-first ARE a half-word on a little-endian core - and
a 16-bit request whose buffers are not aligned to a half-word runs
polled inside `start()`; byte engines leave every 16-bit request to the
polled loop. A refusal there would make the answer depend on where a
buffer happens to sit, which is why the family exception
[../design/spi-bus.md](../design/spi-bus.md) allows is taken. The
requests are raised around the engines and dropped at the end of each
block.
THE LEVELS ARE THE RULE'S (docs/design/dma.md): the receive engine at
the family's high level - the host's clock does not wait for a starved
receive channel, and a frame landing on a full receive FIFO is lost
(SSPRIS's RORRIS) - the transmit engine at its normal one, a starved
transmit holding SCK between two frames. Every bind of the receive
engine is one function, `arm_rx_engine()`, so the level is stated once.

WHAT AN ENGINED BLOCK COSTS ON THE WIRE is the block's and not a
family's: with SPH = 0 (modes 0 and 2) the PL022 pulses its frame signal
between the frames of a continuous transfer, and the pulse costs 1.5 SCK
periods a frame - 19 % on 8-bit frames, 9 % on 16-bit ones - where with
SPH = 1 (modes 1 and 3) a block runs at the nominal wire exactly.
Measured on the RP2350 by this engine in the four modes at two rates on
both of that chip's architectures, the mode-0 figure to the cycle the
same by the vendor's own library doing the same transfer on the same
board ([../rp2350/spi.md](../rp2350/spi.md)): a device that takes mode 1
or 3 saves the 19 % by saying so.

What the host asks of an engine: `present`, `channel`, `element`,
`Report` with `errors`, `arm(data, request, high_priority[, report])`,
`start(pointer, count)` at the frame's beat, `start_fixed(cell, count)`
and `start_discard(cell, count)`, `complete()`, `busy()`, `service()`
with `flag_complete` and `flag_error`, `abandon()`, `stop()`.

## The short asynchronous request

On the engines a transaction costs the core a fixed sum, E - `start()`,
the two channels' launch, the one interrupt, the completion - whatever
its length: the wire's time is the core's own. Polled, it costs the
core the whole transaction: a fixed part F and each frame, which is the
wire's frame or the loop's own turn, the longer. A frame on the wire is
its bits in SCK periods and, in modes 0 and 2, the 1.5 periods of the
block's frame gap (below); an SCK period is the divisor's SSPCLK
cycles, scaled to the core's clock by the ratio of the two rates the
host was given (`pl022_frame_cycles`). So a request is cheaper polled
while F and its frames stay under E, and rides the engines from the
smallest count N with F + N x frame >= E, ceil((E - F) / frame)
(`pl022_async_min_frames`), its command phase counted as frames written
and its data phase as frames read or written. Only the difference E - F
enters, one for a read and one for a write, and each family measures it
as one difference through one call (its traits' `costs`): the engines'
32-frame block less its 32 frames on the wire, against the polled
loop's 8-frame request less 8 frames at the loop's own pace.

The host keeps the two frame costs for the configuration in force,
computed where the control word is written and when the clocks move
(`reconfigure()`, `init()`, `rebase()`), never per request: an
asynchronous request pays two multiplies and a compare in `start()`, a
polled one nothing, since it never asks. `async_min_frames(clock, mode,
bits, read)` answers the count for a program or a suite. The figures and
the counts they draw are each family's
([../rp2040/spi.md](../rp2040/spi.md),
[../rp2350/spi.md](../rp2350/spi.md), "The short request").

## The host above the wire

What the host's own code costs around and between the frames, and the
shape it has - written from the block's register description and
measured on the RP2350 in both of its architectures
([../rp2350/spi.md](../rp2350/spi.md) carries the numbers; the vendor's
polled loop on the same board is the figure beside them).

- THE REQUEST IS LENT, AND WHAT COMPLETES INSIDE `start()` KEEPS
  NOTHING. The arbiter's contract lends the request for the call; a
  request that completes inside `start()` is read field by field through
  the reference and no copy is made. An engined one copies what its
  tenure (the time one transaction holds the bus) needs into `tenure_`:
  the request AS WORDS - one load and one store a word, inline on every
  core, because a struct copy of this size is a `memcpy` call on a
  RISC-V build at `-Os` and a byte copy on a core that cannot load a
  misaligned word.
- The request is laid out with nothing padded but the tail: the four words
  first (the two out buffers, the in buffer, the reply), then the two
  half-words (the length, the prescaler pair), then the bytes - fields of
  27 bytes and one of tail padding, 28 where the family's `PinRef` is one
  byte, where the field order of the other strata cost 36 with nine of
  padding. The names are the other strata's and do not move; the order is
  this file's.
- `apply()` IS ONE COMPARE. The prescaler pair, the mode and the width
  are folded into the word the block takes (`spi_control_word`: SSPCPSR
  over SSPCR0, a handful of shifts on the request's three bytes) and
  held against the word last applied; equal, nothing is touched. A
  request whose word differs is clamped to the bus's ceiling, compared
  again, and only then pays the disable/enable pair the block demands
  for its control registers (12.3.4.2 of the RP2350 data sheet). The
  loop-back bit lives in SSPCR1 and outside the word.
- THE FIFOS ARE THE POLLED LOOP'S WRITE-AHEAD. A FIFO's worth of frames
  is in flight at once, written with NO FLAG READ: a transmit FIFO
  holding fewer than its depth cannot be full, and a receive FIFO handed
  back no more than its depth cannot overrun (12.3.3.4, 12.3.3.5) -
  which is what makes the receive-overrun interrupt a thing the host
  need never arm.
- The polled loop has two shapes, chosen per phase. The receive shape
  runs every phase but one: it primes the transmit FIFO with the first
  `fifo_depth` frames and then, per frame, waits for RNE, reads the frame
  and writes the one a FIFO's depth ahead: three accesses of the block a
  frame (SSPSR, SSPDR in, SSPDR out). With a null in buffer it reads and
  throws away. The frames in flight - written and not yet read back - are
  never more than the receive FIFO's depth, so whatever handler holds the
  loop off, the frames that can land in that FIFO are the ones in flight
  and the FIFO holds them all: no handler can overrun this shape, which
  the RP2350's bench proves with its tick handler stretched past eight
  frame times. The transmit-only shape is taken only when all four hold: a
  null in buffer, 8-bit frames, a run longer than the FIFO and a divisor
  under four, the one place the receive shape would fall behind the wire on
  the RP2350's cores (`run_polled`). A command phase of a FIFO's depth or
  less, a write at clk_peri / 4 or slower and every 16-bit write therefore
  run the receive shape, discarding. The transmit-only shape paces on TNF
  and reads nothing per frame, two accesses a frame; the receive FIFO fills
  and overruns by design (a frame pushed on a full receive FIFO is
  dropped and RORRIS raised while the block keeps shifting - harmless
  with RORIM masked, which this host never arms), and the tail waits for
  BSY to clear, drains the receive FIFO and clears the overrun. Both: the
  width a template parameter, the buffers advancing pointers, no accessor
  call, no interrupt asked of anything, the waits bounded by ONE budget for
  the whole transaction - one slowest frame per frame and one more, 200 000
  turns of the wait a frame. Measured on both of the RP2350's cores: the
  write shape at the nominal wire (plus the block's gap in modes 0 and 2) at
  clk_peri / 2, where it is taken; the receive shape at the wire at clk_peri
  / 4 and slower and bound by its three accesses a frame at / 2, where it
  reads ahead of the vendor's full-duplex loop on both cores, the vendor's
  write-only loop reading the same as this one's
  ([../rp2350/spi.md](../rp2350/spi.md)).
- A POLLED WAIT THAT RUNS OUT ANSWERS `spi_stalled` (`util/spi_bus.hpp`),
  with or without engines; `spi_dma_fault` is what the engines answer. The
  select is released and the receive FIFO drained, the frames still in
  the transmit FIFO given a bounded time to leave first. Nothing else is
  done to the block, and `recover()` is the way back: the select released,
  the engines put away and bound again, the block reset and reconfigured
  to the state applied, which throws away what a block that does not clock
  still holds in its transmit FIFO. The arbiter calls it only when a timed
  asynchronous request goes unanswered, so after a `spi_stalled` it is the
  program's to call, with nothing on the engines; before it, a later request
  is promised nothing. That later request does not wait for the program: one
  already in the arbiter's FIFO starts in the same dispatch, before the
  requester's handler can run. On a block that still does not clock, a polled
  one spends its own budget and answers `spi_stalled` too, and an asynchronous
  one goes to the engines and has no completion, which is `spi_timeout` on a
  timed bus and a bus busy for ever on an untimed one; on a block that clocks
  again, the frames left in the transmit FIFO go out first, under the new
  select. No verb says the engines are idle, and `recover()` under a request on
  them takes it away, its completion never coming, so a program calls it when
  it knows: every request it has posted answered, on a bus with one client.
  Each family's suite stages this on its silicon, the
  block's enable taken down under a polled write (letter g of
  `test_rp2040_spi` and `test_rp2350_spi`), and the next polled request
  after `recover()` is exact.
- THE BLOCK'S OWN INTERRUPT LINE IS DECLINED BY THE HOST. Four sources
  on one line (RXIM at the receive FIFO's half, RTIM a frame waiting
  with no clock for 32 bit periods, TXIM, RORIM) are what a per-frame
  interrupt engine would run on; a handler a FIFO level costs the core
  more than the wire's own frames at every rate a program uses, where
  the engines cost a fixed sum
  ([../design/spi-bus.md](../design/spi-bus.md), "Two ways to run a
  request"), so the host masks all four and leaves the line to a client.
- THE PHASE BOUNDARY DRAINS: the first data frame goes out only when the
  last command frame has come back - the polled loop's command turn ends
  on its last echo, a command block on the receive channel's completion
  - so D/C flips between the two phases on the wire. That drain is the
  one idle gap a two-phase request pays, and a one-phase request pays
  none.
- THE RECEIVE FIFO IS NOT FLUSHED AT `start()`. Every transaction takes
  back every frame it wrote, so a frame can stand in the FIFO only after
  a fault, and the fault paths - a polled wait that ran out, `recover()`
  - are where it is drained. The chapter names no other way a frame
  arrives.
- THE PINS are the family's run-time pin, driven by its own one-store
  verb; a null D/C line is skipped inside that verb; `cs_setup_us` is
  tested before any call.
- THE BLOCK'S OWN FRAME SIGNAL, WHEN A HOST'S PIN SET NAMES `cs`
  (`frame_select`): the pad goes to the block after the other three and
  before SSE, and back at `release()`, and the Request's own `cs` is left
  null. It is a select a frame in modes 0 and 2 - the block raises it
  between the frames of a continuous transfer - and one select for a run
  of back-to-back frames in modes 1 and 3 (RP2040 4.4.3.10 to 4.4.3.13),
  which is what a PL022 CLIENT takes in the modes it frames on its select
  and what a device whose every frame is a transaction wants, and not
  what a device reading a multi-frame transaction under one select can
  take - that select stays a GPIO the Request carries. Measured on the RP
  pair (both families' documents): a window a frame in modes 0 and 2 and
  one for 64 back-to-back frames in modes 1 and 3, counted by a PIO
  machine at the far end; the high pulse between two frames about HALF AN
  SCK PERIOD (48 to 53 ns at clk_peri / 16, 96 to 120 ns at / 32, by a
  PIO gauge of two cycles' resolution), inside the gap of 1.5 periods a
  frame the engines' paragraph prices. A run in modes 1 and 3 is as long
  as the transmit FIFO never empties; a host held off past it raises the
  signal between two frames like any end of a run.
- THE HOT PATH'S PLACE IS THE PROGRAM'S, BY A TAG. On a chip that
  executes in place out of a flash behind a cache, a cold transaction
  pays the cache its fill of the host's code - on the RP2350's Cortex-M33
  some 4250 cycles more than the warm figure for a three-byte request,
  69 lines of 8 bytes. A section attribute takes a string literal and this file knows
  no family, so it places nothing by itself: `Pl022Host`'s last template
  parameter is util/placement.hpp's tag, and two paths run through its
  `Hot<P>` - `start()` (the polled loops, the tenure's copy and the
  short-request decision folded in) and the DMA line's body
  (`dma_isr()`). `InFlash`, the default,
  leaves each where it would be without the trampoline - the DMA body
  inline in the app's vector, `start()` a function in the flash; `InRam`
  makes each ONE flattened instance in `.fast_text.brio`, which a family
  that can place code in SRAM collects into `.data`. The engines' launch
  has three call sites (a command block, a data block from `start()`, a
  data block chained from the vector) and stays ONE body under either
  tag (out of line in the flash, or its own instance in SRAM - flattened
  it is some 1 KB of the engines' programming); a change of mode, rate or
  width (`reconfigure()`, the frame costs with it) and a polled wait that
  ran out (`settle_fault()`) stay out of line in the flash under both. `InRam` compiles only where the family's chip
  traits say `places_fast_text` (`HotPlacement`); a third tag does not
  compile. THE ENGINES' DUMMY CELL IS IN RAM, never a constant: the
  transmit engine reads it once a frame of a read with no out buffer, and
  a constant in a flash executed in place is an XIP read every frame,
  competing there with the core's own fetches - measured on the RP2040,
  `test_rp2040_spi` letter d's rate inside an engined block at clk_peri /
  4 read 1046 and 1052 thousandths of the wire in two passes of ten with
  the host's code in the flash, 998 to 1007 in ten of ten with the cell
  in RAM. Both RP families place
  ([../rp2040/platform.md](../rp2040/platform.md) and
  [../rp2350/platform.md](../rp2350/platform.md), "A handler in SRAM",
  have the numbers: on these cores the placement buys the cold path and
  costs the warm one its call).

## What stays per family

The pin table and its function code; the reset or clock gate; the
interrupt line and its controller; the crt's handler name an app binds;
the DMA request numbers and the engine types that fill the slots; which
rate is SSPCLK; the FIFO depth; the run-time pin type and the busy-wait;
what SOD does to a pad on that silicon; whether the family places
`.fast_text` (its traits' `places_fast_text`, the item above); and the public names -
`Pl022<n>`, `SpiHost<n, pins, TxEngine, RxEngine, Placement>` and `SpiClient<n,
pins>`, the family's aliases of the three templates here, with the
family's own defaults for the empty engine slots and the placement. An
application that
says `brio::SpiHost<0, pins>` has no idea this stratum exists, which is
the point.

Each family's document is where its measurements live
([../rp2040/spi.md](../rp2040/spi.md) is the first).

## The proof that it knows no chip

`brio/host/sim_pl022.hpp` is a SECOND realization with no silicon under
it: the register block as an array in RAM (at the PL022's offsets, with
the block's reset values, so a bring-up's flush of the receive FIFO
terminates and a polled loop can write its first batch), a reset that memsets
it, an interrupt controller that counts, pads that remember what they
were handed to and when, a run-time pin that records which way it was
driven, and a busy-wait that COUNTS the microseconds it is asked for
instead of spending them. It is compiled twice - by the host suite
`test_pl022`, which judges the exact words a configuration leaves in the
block, the way each request was run - refused, polled inside `start()`
at any length on a host with engines too, a command block chained to its
data block, the short request's edge, a polled wait that ran out and its
`spi_stalled` - and the ORDER of the acts of a bring-up,
and by a family's compile check (`test/family_rp2040/pl022_ip.cpp`),
which names every verb of the resource, of the host with both engine
slots empty and both filled, and of the client. A driver that needed a
silicon would fail one of the two. The negative TUs beside it stage a
traits type with one member taken away and require the concept to refuse
it by name.

What the fake does NOT model is the FIFOs: a register block that is
plain memory cannot, since a store to `SSPDR` is a store and nothing
watches it. So nothing comes back by itself - a case that runs a polled
loop to its end sets the receive-not-empty flag and reads back whatever
was written last - and the receive half of a transfer, the phase
boundary and the frames on the wire stay the bench's to judge.

## Not covered yet

Driver gaps, each with its reason. They are the same in every family that
carries the block, because they are the block's; what each of them would
COST on a given chip - a wire, a device, a pad - is in that family's own
document.

- The National Semiconductor Microwire transaction: the framing is a
  code `SpiConfig` takes and the resource reads back, but the host
  engine's Request is full duplex and one frame wide, where Microwire is
  a control frame followed by the block's answer. A device with a
  Microwire interface is the first user.
- Frame widths other than 8 and 16 in the Request: the resource takes
  the whole 4..16 the block offers and a client is configured with any
  of them, but `SpiDataSize` is the two widths the other strata's
  Request speaks (`docs/design/spi-bus.md`). A device with a 12-bit
  frame is the first user.
- A client on the DMA engines: the slots are the host's, and a client
  answers from its own interrupt. A client that streams is born with a
  program that needs one.
