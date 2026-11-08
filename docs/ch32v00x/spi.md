# SPI (CH32V00x)

This document is for a programmer who talks to an SPI device - a
display, a flash, a converter - with the CH32V006 or the CH32V003 as
the bus host, or who makes the part a client another host clocks. It
covers the one SPI of RM ch. 16 - both roles, 8- or 16-bit frames, the
four modes, a software or hardware chip select, hardware CRC, two DMA
requests - in three layers: `Spi<1>`, the resource, its registers as
functions; `SpiHost`, the host engine, which runs one request at a time
on the engine the request names, polled inside the call or on its DMA
engines; and
`SpiClient`, the client. A program drives the host through
util/spi_bus.hpp's arbiter, `SpiBus`, an active object of the program (a
component with its own event queue, served one event at a time by the
kernel, [docs/learn/introduction.md](../learn/introduction.md)) that
takes the clients' requests one at a time in arrival order and sends
each its reply ([design/spi-bus.md](../design/spi-bus.md)). The request
is the same struct on every stratum.

The smallest use, a host writing a command byte and 64 pixel bytes to
a device whose select is PC3, from "How to use it" below:

```cpp
using Bus = brio::SpiHost<1, brio::spi1_default_pins,
                          brio::DmaTxEngine<3, uint16_t>, brio::DmaRxEngine<2, uint16_t>>;
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
using Cs = brio::Pin<'C', 3>;
static_assert(Bus::has_dma);                          // it posts requests naming the engines

if (!Bus::init(clock, 8'000'000)) { /* not even HCLK/256 honours the ceiling */ }
Cs::output(true);

Bus::Request r{};
r.cs = Cs::ref();
r.cmd = brio::lend<brio::Lease::reply>(cmd);  r.cmd_len = 1;
r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
r.clock = brio::SpiClock::div4;                       // at 48 MHz: clamped to div8, 6 MHz
r.mode = brio::SpiMode::mode0;
r.reply = brio::reply_to<Display, brio::SpiDone>();
r.engine = brio::SpiEngine::dma;
brio::post<Spi>(r);                                   // the command in the loop, the pixels on the engines

BRIO_CH32_VECTOR(dma1_channel2_handler) {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
BRIO_CH32_VECTOR(dma1_channel3_handler) {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

This is an excerpt: it assumes the program's platform `P`, its clock object
`clock`, an active object `Display` that handles `SpiDone`, the buffers
`cmd` and `pixels`, the headers its names come from (the family's
`ch32v00x/` headers and `util/spi_bus.hpp`), and the statements where they
belong, the setup in `main()` and the post in a handler.

Line by line: `Bus` is the host on SPI1's default pads (SCK PC5, MOSI
PC6, MISO PC7) with its two DMA engines, channel 3 transmitting and
channel 2 receiving, each moving up to a 16-bit frame a beat; `Spi` is
the arbiter over it, an active object listed in the program's `Tenuto`,
with room for four pending requests and no timeout, so a channel that never
completes would leave the bus busy for ever (a limit is its fifth argument,
`brio::SpiBus<Bus, P, 4, brio::BusPassThrough, brio::ticks_from_ms<P>(20)>`,
`ticks_from_ms` from kernel/time.hpp);
`Cs` is the
device's chip select, an ordinary GPIO the request carries and the host
drives around the transaction, not the peripheral's NSS pad. The
`static_assert` is the program's own check that the host it names can
run a request naming the engines at all: a host with no engines refuses one.
`Bus::init()` takes the clock object (`constexpr SysClock clock;`,
[clock.md](clock.md)) and a ceiling for every request's SCK, opens the DMA
controller's clock and enables the two channels' interrupt lines (a program
calls no DMA set-up of its own), and answers false when not even HCLK/256
honours the ceiling;
`Cs::output(true)` makes the select an output driven high, inactive,
before the first request. The request lends the host two buffers
(`lend<Lease::reply>`: the buffers stay the sender's, lent until the
reply comes), names the SCK as a division of HCLK and the mode, and
says where the answer goes: a `SpiDone` event posted to `Display`, the
program's active object that sent it. Its `engine` field names
`SpiEngine::dma`, so the DMA engines run it: the command byte, a command
phase in front of a data phase, goes out in the transmit-only loop inside
the call, the 64 data bytes ride the engines, and the call returns while
they move. The ceiling
clamps and never speeds up: HCLK/4 is 12 MHz at 48 MHz, over the 8 MHz
ceiling, so this request runs at HCLK/8, 6 MHz. The program binds the
two channels' vectors to the host's `dma_isr()`, under the startup
code's names, and posts the host's status to the arbiter when a
transaction ends.

What decides correctness even there. `init()` after the clock's
`SysClock::init()`, since the ceiling and the select's timing are computed from the rate
the clock type claims; a request's SCK is a division of the real HCLK,
so on a chip still at its 8 MHz reset rate every transaction runs six
times slower than its numbers say, and nothing refuses it. The buffers
are untouched by the sender from the post until `SpiDone` arrives (the
host and its engines read and write them in between, and nothing checks
that the sender keeps away). Both DMA vectors must be bound: an unbound
one lands in the startup code's spin the first time its channel
interrupts (the receive channel at every block's end, the transmit
channel on a transfer error). A polled request needs no vector: it runs
its loop inside `start()`, in the arbiter's dispatch, at any length, on a host
with engines or without, and touches no channel; interrupts stay enabled and
every other active object waits under `Tenuto` for its whole wire time, so a
request names the engines when the others must keep running through it. The
host does what the request names at any length; no cost decides for it, and
"Which engine to name" below says from which length the engines cost the core
less. A host with no engines (`SpiHost<1>`) runs polled requests alone and
answers one naming the engines `spi_rejected` with nothing on the wire, and an
engined host answers the same, before it touches anything, to a request its
engines cannot carry: 16-bit frames whose data buffers - or a command that is
the whole request - start at an odd address (a command in front of data runs in
the loop and may sit anywhere), or 16-bit frames over engines of `uint8_t`. A
request over a transfer of the host's own still on the engines is refused the
same way, whichever engine it names - the one busy check, made before
anything moves. A request the host runs ends with the D/C high and its select,
if it names one, high, ok or failed. The client
is narrower than the host: a client that answers needs CPOL = 1, modes 2
and 3 (16.2.3), and `SpiClient` refuses an answering client in modes 0
and 1 - there its window slips one bit at both ends whenever its first
answer bit is 1 and the host's MOSI is low at the select's fall
(measured); a client that only listens is served in all four. And on
the CH32V003 a sleeping core stops the DMA controller, so that part's
`idle()` does not sleep while an engined transaction is in flight
([dma.md](dma.md)).

The page is long because the host's costs are on it. In order: the
documents of record; what the silicon does; the types and verbs; the
host above the wire, cycle by cycle against the wire's figure; which
engine to name; the client's runs; how to use it, the engines and the
client and what each answer asks of the program among the examples; the
bench findings, with this part as the desk's SPI client; and what is not
covered.

## Documents of record

The CH32V00X
reference manual V1.5 (16.2.1 for the pins and the select, 16.2.2 and
16.2.3 for the two roles, 16.2.5 for the CRC, 16.2.7 for the errors,
16.3 for the registers), the CH32V006 datasheet V2.0 (table 2-1-1 for
the pads). The header is
[brio/ch32v00x/spi.hpp](../../brio/ch32v00x/spi.hpp); the reference
suite is `test_ch32_spi`, and the host's costs are `bench_ch32`'s and
that suite's letter g's - E, F and the length from which the engines cost
the core less (a suite's letter, below, is one command key of its console and the group
of verdicts it runs, `z` the key
that runs the image's set of letters, a letter outside it run by its
own key).

## What the silicon does

- **The STM32F1's SPI**, with the I2S half absent and one register of
  its own: HSCR, a "high-speed read" mode whose SCK the manual gives
  as HCLK/(BR + 2).
- **No FIFO.** One transmit buffer, one receive buffer, one shift
  register (figure 16-1): TXE means the buffer moved into the shifter,
  RXNE that a frame has been shifted both ways. A frame written while
  another shifts waits in the buffer and follows it with no gap on SCK
  (16.2.2: "If the TXE flag position bit needs to fill the data register,
  maintain the complete data flow."); and the receive buffer holds one
  frame - a frame that ends while the one before it stands unread is lost,
  OVR set, the transmit side untouched (16.2.7). The host's two loops are
  written from those two sentences: the transmit-only loop uses the
  write-ahead, and the receive loop declines it (The host above the wire).
- **OVR's clear differs by die** (measured, letter v, ten runs on each part).
  16.2.7 clears the flag with a DATAR read followed by a STATR read. On the
  CH32V006K8U6 that is so: after the read the first STATR load still shows the
  flag, RXNE down, and the next does not, with or without a load before the
  read. On the CH32V003F4P6 it is not: after a read with no load before it,
  every load shows the flag, and what clears it is a DATAR read after a load
  that showed it. A zero written to it clears nothing on either (the register
  table's RW0). So `flush_rx()` and `clear_overrun()` read STATR, DATAR, STATR,
  which leaves neither flag on both; the transmit-only loop's tail needs no
  more than its DATAR and STATR, the last load of its wait on BSY coming first.
- **RXNE does not come half a bit early** (measured, `test_ch32_spi` letter w
  and `bench_ch32`'s `spi.frame` table, below). 16.2.2 raises RXNE at "the
  last sampling clock edge of the data word", which in CPHA = 0 is half a
  bit before the frame ends. On this silicon the flag comes with the frame's
  last edge: at the first STATR load that shows RXNE the SCK pad already
  reads its idle level, in all four modes, at /256 and /64, 8 and 16 bits,
  on the CH32V006K8U6 and the CH32V003F4P6, and the receive channel's
  completion flag the same; and in mode 0 a frame carries a dead bus at
  every code, at /256 too, 22 cycles from the flash, where that half bit is
  128 cycles and the loop's turn under 40. So when a request ends the wire
  has ended, on both engines, with no wait added for that edge ("When a
  request ends", below).
- **Frames of 8 or 16 bits** (DFF), SCK from HCLK/2 to HCLK/256 (BR),
  MSB or LSB first, the four CPOL/CPHA modes of table 16-1.
- **The chip select has three arrangements** (16.2.1): software (SSM,
  the level from SSI, the pad free), hardware output (SSOE, driven
  low while SPE is set), hardware input (the multi-host arrangement:
  a low level raises MODF and demotes the host).
- **Nothing is enable-protected** (measured): 16.3.1 says DFF and CRCEN
  "can only be written when SPE is 0" and that BR, CPOL, CPHA, MSTR
  and LSBFIRST "cannot be modified during communication" - and a raw
  write to each of the seven lands with SPE set. The driver's
  refusals and its disable/enable pair are the only protection there
  is.
- **A DMA request that rises while its channel is disabled is
  latched** and served at the channel's next enable (measured: with
  RXDMAEN standing through a command phase written by hand, the
  command's echo landed as the block's first byte; with it standing
  between transactions, the stale frame in DATAR did - every engined
  block after the first came back shifted by one). The host raises
  RXDMAEN only once the receive channel is enabled, TXDMAEN only once
  the transmit one is, and drops both at the block's end.
- **The DMA requests are channels 2 (receive) and 3 (transmit)**,
  table 8-2 - the channel is the request on this family. DATAR is
  sixteen bits, so a channel can move a 16-bit frame in one half-word
  beat - from a half-word boundary only: 8.3.6, a 16-bit access ignores
  the address's bit 0. Every frame that comes back was clocked out
  first, so the receive block ends after the transmit block: the
  receive channel's completion is the one edge a transaction needs.
- **The default pads on the CH32V006** are SCK PC5, MOSI PC6, MISO PC7,
  NSS PC1 (table 2-1-1); the remaps are AFIO's columns, which
  `SpiPins` carries as a remap code (`spi1_pins_for(code)` is a whole
  column, and `init()` writes the code; [pin.md](pin.md)).
- **Two of the chapter's sentences are slips**: DFF's two codes are
  both described as "16-bit" (0 is 8-bit), and 16.2.1 describes CPHA
  with the same words for both values (table 16-1 and the figures
  settle it).
- **A client that answers needs CPOL = 1, as 16.2.3 says.** "If you need
  to send in slave mode, you need to set CPOL to mode 2 or mode 3"; "if
  you only need to receive from the mode, you only need to match the host
  CPOL mode" (the CH32V003's manual, 14.2.3, says the same). Measured
  against a foreign host (the CH32X035's, the bench findings' "THE
  ANSWERING CLIENT NEEDS CPOL = 1"): in modes 0 and 1 a window whose first
  answer bit is 1, with the host's MOSI low when the select falls, slips
  one bit at both ends - the client's SCK pad sees an extra clock at the
  select's fall - and the same window is exact in modes 2 and 3; a
  client that only listens (MISO undriven) is exact in all four. A
  configuration whose correctness depends on the data is refused:
  `SpiClient` refuses an answering client in modes 0 and 1, on both
  parts.

## Types and verbs

[brio/ch32v00x/spi.hpp](../../brio/ch32v00x/spi.hpp), three layers:

- `Spi<1>` is the resource: `configure(SpiConfig)` (refused by
  `spi_config_valid()`: CRC outside full duplex or with a zero
  polynomial, SSOE on a client), `enable()`, `disable()` - the drain
  procedure, TXE then not BSY, bounded -, `data()` both ways,
  `rxne()`/`txe()`/`busy()`, `flush_rx()`, the three errors with their
  clearing sequences (OVR: STATR, DATAR, STATR; MODF: STATR then a CTLR1
  write), the CRC verbs (`crc_next()`, `tx_crc()`, `rx_crc()`),
  `software_select()`, `high_speed_read()`, `dma_requests()`, the
  three interrupt enables and `isr()`, the raised-and-enabled sources.
- `SpiHost<1, pins, TxEngine, RxEngine, Placement>` is the
  engine `SpiBus` (= `BusMaster`) drives. Its `Request` carries the other
  strata's fields name for name: a chip select `PinRef` and a D/C line, a
  command phase (D/C low), a data phase with an optional out buffer (null
  = 0xFF dummies) and an optional in buffer (null = discard), the
  per-request `mode`, `clock` and `bits`, and `engine`, the engine that
  runs it ([design/spi-bus.md](../design/spi-bus.md), "Two ways to run a
  request"), which the host obeys at any length: `SpiEngine::polled` spins
  inside `start()` and completes there, the explicit loops and no channel
  touched; `SpiEngine::dma` runs on the DMA engines - `start()` launches
  them and returns, the receive channel's completion ends the transaction.
  Around it, in the order the host applies them: a request over a transfer
  of the host's own still on the engines is refused, `start()` answering at
  once with `spi_rejected` before it touches the block, the select or the
  D/C - the one busy check, made the same way for a polled request; a host
  with no engines bound refuses it the same way, and `has_dma` (false there)
  lets a program check at compile time; an empty request completes on the
  spot, `spi_ok`; a request the engines cannot carry is refused the same
  way - 16-bit frames with the out buffer, the in buffer or a command alone
  off a half-word boundary (8.3.6), 16-bit frames over engines of `uint8_t`;
  a command phase in front of
  a data phase runs in the transmit-only loop inside `start()`, D/C low,
  then the data phase on the engines - a command is short by nature, and a
  block of its own would cost a second launch and a second interrupt - and a
  stall in that loop ends the request there, `spi_stalled`; a command alone
  rides the engines as the whole request, its end raising D/C. `start()`
  answers true for a polled, an empty and a refused request and false only
  when the engines have it. The order of the fields is this host's: forty
  bytes with no padding, the pads, the buffers and the lengths first, then
  one word holding `clock`, `mode` and `bits` behind `cs_setup_us`, then the
  reply (the next section). `status()` answers `spi_ok`, `spi_rejected` (a request
  over a transfer still on the engines, one naming the engines on a host with
  none, or one they cannot carry), `spi_dma_fault` (a block the engines could
  not finish) or `spi_stalled` (a polled flag never came within the
  transaction's count of polls);
  after the last two the host has put the block back or will at the next
  request that moves something, and the program calls nothing.
  `init(clock, max_sck_hz)`, `rebase()`, `clock_for(hz)`,
  `prime()` for a caller framing its own select, `bit_order()`,
  `dma_isr()`, `recover()`, `release()` (the engines stopped, the drain, SPE
  down, the gate closed, no reset: `init()` starts from the block's reset
  line, [dma.md](dma.md), "A released requester"), `claim_nss_pad()`. The host arms
  no interrupt of the block: SPI1's vector is not its own. `Placement` is
  where the two polled loops run from, util/placement.hpp's `InFlash` (the
  default) or `InRam` - each width of each loop then one instance in SRAM,
  a call a phase ([platform.md](platform.md), "A handler in SRAM"); any
  other tag is refused at compile time. The engine parameters are `DmaTxEngine<3, Elem>` and `DmaRxEngine<2,
  Elem>`, both or neither, on those two channels and no others (refused
  at compile time otherwise). Their `Elem` is the widest beat: `uint8_t`
  engines serve the 8-bit requests; `uint16_t` engines - DATAR's width,
  nothing wider compiles - serve both, the 16-bit request in half-word
  beats when both of its buffers sit on a half-word boundary. The
  transmit engine is armed for its errors alone: one DMA interrupt a
  block, the receive channel's, and `dma_isr()` reads the controller's
  one flag register once for both channels. The receive engine arms at
  very_high and the transmit at high ([../design/dma.md](../design/dma.md)):
  DATAR's receive half holds one frame, and a receive channel the
  transmit one outran would find the next frame on top of its last (OVR,
  16.2.7), while a transmit channel that waits only stops the host's
  clock between two frames. A block's launch is the two engines' block
  starts and the two DMA requests raised around them, no mask (nothing
  completes under it). Frames in a byte buffer: one byte per 8-bit frame,
  two bytes low-first per 16-bit frame - one half-word in memory, which
  is what a 16-bit beat moves.
- `SpiClient<1, pins>`: the target side - `init(clock, Config)`
  refusing a configuration the client is not served in (`config_served()`:
  an answering client - MISO driven - with CPOL = 0,
  `spi_client_may_answer()`) and `init<cfg>(clock)`, where such a
  configuration does not compile; `drive_output(true)` on a client
  configured as a listener in mode 0 or 1 is not checked, the caller's
  duty; the per-frame
  verbs - `enable(first)` with the first answer loaded before the host's
  clock, `enable()` with the buffer empty, `write()` on TXE (one frame
  ahead: no FIFO), `poll()`, `selected()` read off the NSS pad,
  `drive_output()` for a dark listener, `isr()` -; and the runs a
  transaction is served by, its unit (the next section but one):
  `answer(out, spins)`, `exchange(out, in, spins)`, `receive(in, spins)`
  over `uint8_t` or `uint16_t` frames, each returning a `Run` - the
  frames moved and how it ended, `complete`, `closed` (the host raised
  the select) or `stalled` (the spin budget). A run over a wider element,
  and an answering run on a client whose pins name no MISO pad, do not
  compile (`test/family_ch32v00x/neg/spi_client_*.cpp`).

[brio/ch32v00x/pin.hpp](../../brio/ch32v00x/pin.hpp) holds `PinRef` (the
runtime pin the request carries) and `Pad` (the compile-time pad name
the pin tables use).

## The host above the wire

What a transaction costs beyond its frames' time is the host's own:
`start()`, the polled loops, the engines' launch and completion. Each is
written from 16.2.2's two sentences above, each is counted in the
release listings (WCH gcc 15.2, `-Os`, the `xw` ISA, the hardware
prologue on: `test_ch32_spi-2.lst` and `bench_ch32-1.lst` for the
CH32V006, the cost model 2.5 cycles a straight-line instruction, the
prologue's entry 29 and exit 25 cycles measured in platform.md), and
each is MEASURED on the CH32V006K8U6 at 48 MHz by `bench_ch32`'s letters
s and e and `test_ch32_spi`'s letters c and g, MISO floating. A bench
line holds its instrument: the stopwatch's 44 cycles in every wall, and
on an engined line the meter's stamp pair, 113 cycles a handler (letter
r).

- **The request is lent for the call** (design/spi-bus.md): `start()`
  reads it through the reference, and one that goes to the engines has its
  tenure copied inline, the two pins as four word loads and stores, never a
  call - the launch happens inside `start()`, so nothing else of the request
  is read after it returns. The descriptor's layout puts `clock`, `mode` and
  `bits` one word behind `cs_setup_us` and leaves no padding: 40 bytes.
- **`apply()` compares one word.** The request's `clock`, `mode` and
  `bits` lie together behind `cs_setup_us` at a word boundary, so the
  compare with the last request's is one load and a shift against a
  kept word: nine instructions with the ceiling's clamp, nothing
  written. On a mismatch the three are folded into CTLR1 and compared
  with the word in force (two requests under a ceiling may fold to the
  same word); a changed word is written with the drain and the
  disable/enable pair 16.3.1 asks for DFF.
- **Two polled loops.** A phase whose answers nobody wants - the
  command phase, a write with no in buffer - runs the TRANSMIT-ONLY
  loop: a frame written whenever TXE says the buffer is free, which it
  is a whole frame time before the shifter needs it, so the clock never
  pauses; the answers overrun the receive buffer, harmless to the
  transmit side by 16.2.7, and the tail waits TXE then not BSY and
  clears RXNE and OVR with a DATAR read and a STATR read (letter v). Nine
  instructions a frame at 8 bits (twelve at 16), and MEASURED AT THE
  WIRE at HCLK/4 and slower in both widths: 31.9 cycles an 8-bit frame against
  the wire's 32 at HCLK/4, 128.0 and 255.9 to 256.0 at HCLK/16 against 128 and
  256. An interrupt that lands in the loop only pauses the stream, and
  costs what it outlasts the frame's slack: the console's interrupts
  inside a 16-bit run at HCLK/4 cost 3 cycles a frame (67.1 against
  64). A phase with an in buffer runs the RECEIVE loop, ONE FRAME IN
  FLIGHT at every rate: write k, poll RXNE, read k and write k + 1 in the
  next instruction, the next frame's load and the store of the one read
  placed inside the wire time; eleven instructions a frame at 8 bits with
  one taken branch, of which four on the bus's dead time (the poll's last
  turn, the DATAR read, the DATAR write). This host owns the clock and the
  clock waits for the read, so a handler or a preemption of any length is
  a gap on the bus: the loop cannot lose a frame and tests no OVR. Both
  loops spend ONE count of polls for the whole transaction, derived from the
  clocks where a rate is applied and kept as a shift, so a request pays one
  shift of its frames and no multiply - and end with `spi_stalled` when a flag
  never comes within it. A frame is 2^(BR + 4) HCLK cycles at 8 bits and twice
  that at 16; the count is twice the request's frames in cycles, in polls of at
  least four cycles each, plus 256 polls for the block's latency. A poll is five
  instructions in the listing (a load, a test, a count, two branches), measured
  at 14.7 to 25.0 cycles from the flash below. So the count never runs out before
  the wire has had twice its own time, and a handler or a preemption landing in
  a wait costs time and no polls, so it cannot end a healthy transaction. The
  spin is a count and a branch, and no clock is read.
- **The transmit buffer's write-ahead (16.2.2) is used by the
  transmit-only loop and declined by the receive loop.** Storing the next
  frame while one shifts would keep the bus busy through the receive
  loop's turn, and the receive buffer, one frame deep (16.2.7), would then
  lose a frame to any handler longer than a frame. The price of declining
  it is the table: the receive loop's frame over the wire, in HCLK cycles
  at 48 MHz, `bench_ch32` letter e's `spi.frame` (best of five, interrupts
  masked, 8 and 16 bits, modes 0 and 1, every code; three runs identical to
  the hundredth, on `SpiHost<1>` from the flash and its `InRam` twin):

  | the frame over the wire | /2, /32 | /4, /64 | /8, /128 | /16, /256 |
  |---|---|---|---|---|
  | flash, 8-bit, mode 0 | 14 | 22 | 14 | 22 |
  | flash, 8-bit, mode 1 | 38 | 22 | 38 | 22 |
  | flash, 16-bit, modes 0 and 1 | 34 | 26 | 34 | 26 |
  | SRAM, 8-bit, modes 0 and 1 | 12 | 16 | 14 | 10 |
  | SRAM, 16-bit, modes 0 and 1 | 19 | 11 | 19 | 11 |

  The figure does not grow with the frame, so at a slow rate it is a small
  share of the wire: 22 cycles on 512 at /64, 22 on 2048 at /256. At /2 it
  is what limits the rate, 30 cycles a byte against the wire's 16 from the
  flash in mode 0, 54 in mode 1. The 24 cycles between modes 0 and 1 at /2, /8,
  /32 and /128 are one turn of the poll from the flash, whose miss takes two
  taken branches: which turn sees the flag is fixed by the code and the mode.
  The 8 and 16 cycles between the columns of one row are not that turn and have
  not been bisected. A long read that wants the wire's rate names the engines.
  Under the benchmark's console
  and tick the same loop reads 56.7 cycles a byte at HCLK/4, 154.9 to 155.6 at
  HCLK/16 and 31.9 at HCLK/2, and 94.2 to 94.9, 283.2 to 283.5 and 69.4 to 69.5 a
  16-bit frame (letter e, three runs).
- **The loops are aligned to the fetch's word.** From the flash the
  receive loop's frame is a function of where the loop starts modulo
  four: half a word moved it by 6 cycles at HCLK/4 and 3 at HCLK/2, the same
  instructions, the same image otherwise (letter g, two layouts each way).
  `-Os` aligns a
  function to two bytes, so the host aligns both loops' functions to
  four, two bytes of padding at most, and the figures on this page are the
  aligned loops'.
- **The loops from SRAM** (`Placement` = `InRam`, measured in the same
  letters through a twin host): the transmit-only loop at the wire at
  HCLK/2 and slower, 16.0 cycles an 8-bit frame against the flash's 24.9 (the
  wire's 16) and 33.7 a 16-bit one against 37.9 (32); the
  receive loop 29.8 cycles an 8-bit frame at HCLK/2 against 31.9, 50.7 to 50.8
  at HCLK/4 against 56.7, 144.5 to 144.9 at HCLK/16 against 154.9 to 155.6
  (letter e, three runs). The SRAM loop's instance
  sits where the image puts it, util/placement.hpp's trampoline not
  aligning it, and its dead bus moves with that: 8 to 16 cycles at
  HCLK/4 across images. Each width of each loop is one instance in SRAM,
  called once a phase: 460 bytes a host type on the CH32V006 (98 and 108 the
  transmit-only loop's, 106 and 148 the receive loop's, the map's
  `.fast_text.brio`). No image of the CH32V003 carries the placement ("Not
  covered yet").
- **The select and D/C edges** are one store each into BSHR and BCR
  (RM 7.3.1.4, 7.3.1.5), inline, the null D/C a branch; `cs_setup_us`
  is tested before anything is called; no flush of the receive buffer
  at the start of a transaction, because every path leaves it empty at
  its end (the receive loop reads every frame, the transmit-only loop
  flushes after its tail, the receive engine takes every frame), and the
  reset behind a stall or a transfer error, `init()` and `recover()` flush
  what a reset or a `prime()` left.
- **The engines' cost**, `bench_ch32` letter s (five runs): a write on the
  engines of 32 or 256 frames runs at the wire inside its block - 32.0 cycles a
  byte at HCLK/4 and 128.0 at HCLK/16 (32.0 to 32.2 in 16-bit frames at
  HCLK/4, where the 32-frame line's fixed part reads 820 to 931 against the long
  one's 931), one DMA
  interrupt a transaction - and 913 cycles above the wire's time in 8-bit
  frames at both rates and both lengths (931 in 16-bit frames), the line's
  instrument inside: the stopwatch's 44, the handler's stamp pair's 113, and
  the request built inside the stopwatch and `BenchIdle`'s turn, not counted
  apart. The launch is the two engines' block starts,
  five stores and no load each ([dma.md](dma.md)), with SPI1's two
  DMA-request read-modify-writes around them; the completion is the
  receive channel's vector, one INTFR load and the INTFCR store, the
  select released. Which engine to name weighs E against the polled
  path's fixed part, measured as one difference (the next section).
- **The fixed cost of a request**, a 3-byte polled one (a command and
  two data bytes, D/C scripted, the rate unchanged), MEASURED: 340
  cycles above the wire's 384 at HCLK/16 with the stopwatch's 44
  inside, about 300 the host's own; 260 for a lone command byte, 338
  for a command and 15 data bytes (460, 401 and 466 on the CH32V003F4P6).
- **The vendor's own loop, read** (the EVT's `2Lines_FullDuplex`, pinned
  archive 189c91b): it sends on every TXE and reads on every RXNE in one turn
  through its library's calls, each frame written ahead with nothing holding it
  back, at every rate: the write-ahead this host's receive loop declines. This
  host's receive loop takes 150 and 282 cycles a frame at HCLK/16 and 54 and 90
  at HCLK/4 (1.69 and 1.41 times the wire's frame, larger being slower). The
  vendor's loop has not been run beside them on this image ("Not covered yet").
  The vendor's DMA read (the EVT's WUI application,
  a serial flash's read: `SPI_Flash_Read_dma_start` and
  `SPI1_Read_DMA`) sends its command and address bytes polled, one call
  a byte, raises both DMA requests for the data block and drops them at
  its end, and per block disables the two channels, clears a flag and
  writes the counts and the receive address before enabling them: the
  shape this host takes - a command phase in the loop, the data on the
  engines, the requests around the block (What the silicon does: a
  request a disabled channel has not taken is latched).
- **`spi_stalled`, provoked** (letter h): a polled duplex request of 16
  at /16 with SPI1 HELD in reset - its registers ignoring every write,
  RXNE never rising - ends `spi_stalled` after 32 490 to 32 630 cycles (21 680
  to 21 890 on the CH32V003F4P6): the count of 1 280 polls (16 x 64 + 256) at
  25.0 cycles a poll here and 16.6 to 16.8 there, some 580 cycles the reset (430
  there); the same with SPE taken down under the host, a write of sixteen (24 680
  to 24 750, 19 200 to 19 250 there; 18.8 to 18.9 and 14.7 a poll) and a command
  phase in the loop in front of a block on the engines (320 polls: 5 960 to 6 020,
  5 560 to 5 600; 17 and 16 a poll). Never under the polls' lowest time (5 120
  cycles for sixteen frames, 1 280 for the command phase, each poll at four
  cycles), in ten runs on each part, and the D/C and the select high after each.
  The host's own pulse lets the reset line go:
  CTLR1 reads 0x35C, SPE and MSTR up, RXNE and OVR down, and with no `recover()`
  the next polled request and the next on the engines are `spi_ok`, the latter
  with its one interrupt. Exact against the peer in letter t,
  below.
- **The drain the reset goes without**: `Spi::disable()` costs 99 cycles in seven
  runs of ten and 194 to 200 in the others (270 on the CH32V003F4P6) on a block held in reset or behind a closed gate, where
  SPE reads down and neither of its waits starts. Its two waits spend
  something only on a block with SPE up whose TXE or BSY never comes, each
  bounded by a count of polls: two of the longest frame (16 bits at /256, 4096
  cycles) at four cycles a poll and 256 polls, 2 304 polls, and 2^18 polls for
  a client, whose clock is the far host's - no staging here reached them. It gains nothing there or anywhere on this path: the
  select has been released, so a frame the reset cuts reaches no device, and the
  chapter asks the drain for a block that goes on working (16.2.7), not for one
  the RCC is about to reset. So a stall and an owed reset do not drain; `recover()`
  does, and its answer reports the waits.
- **A closed gate is not cured** (letter h): with SPI1's gate shut the registers
  read a stale word (CTLR1 0x02, STATR 0x02), each request stalls on its own
  count of polls, some 32 400 to 32 600 cycles each (21 700 on the CH32V003F4P6), and the reset
  behind it reaches nothing that
  stays. The gate opened, the block has lost its configuration, so the first
  request stalls once more and puts it back, and the next is `spi_ok`, with no
  `recover()`.
- **A real transfer error** (letters h and t): 8.2.1 promises TEIF for a
  reserved address, and it is the addresses just past a memory that give it: a
  block whose buffer starts at the end of the SRAM (0x20002000 here, 0x20000800 on
  the CH32V003F4P6), read or written, and one reading or writing past the end of
  the flash (0x0000F800, 0x00004000). The holes elsewhere in the map complete as
  ordinary blocks; letter h prints each. A store past the SRAM is not stopped by the
  error ([dma.md](dma.md)), so the letters stage it past the flash: on the receive
  channel the first frame's store falls there, one frame into a block of 32 at
  /256; on the transmit channel a buffer whose last eight bytes are the SRAM's stops
  the channel eight frames in. Either way `dma_isr()` answers true once,
  `spi_dma_fault`, the select released, no core fault, no wedge.
- **The reset the interrupt leaves to the next request**: 579 cycles on the
  CH32V006K8U6, 434 on the CH32V003F4P6, read as a `prime()` with the reset owed
  (825, 820) against one with nothing owed (246, 386). The interrupt writes a byte
  and waits for nothing. The request that pays it is the next one that moves
  something: a refused or an empty one touches nothing and leaves the reset owed
  (letter h: CTLR2's ERRIE, set by hand after the fault, stands through a refused
  request and falls to the reset the next `prime()` pays).
- **The next request after a failure, against the second chip** (letter t, on the
  CH32V006K8U6): the exchange commanded first, then the host brought up and
  touched by nothing but the failure and the next request. The failed request
  frames the lone select, the next one the peer's, in mode 3, whose idle SCK the
  reset moves; 16 frames at /256, this end's read and the peer's report judged. Three
  stall paths and the two channels' transfer errors, each followed by a polled
  request and by one on the engines; the transfer error also with a caller framing
  its own select behind `prime()`; a stall and a transfer error with the loops
  in SRAM; and through `SpiBus`, two requests posted from one dispatch, the first
  failing: replies in order, the failure's code then `spi_ok`. Every next request
  exact both ways, 0 mismatches, in all ten runs of 22 cases. With the reset taken
  out of a scratch build every case fails: over a block in reset the next request
  stalls again (polled) or never answers (on the engines), with SPE down the frame
  left standing goes out under the next select and shifts the peer's stream, and
  after a transfer error the frames still on the wire come back in front, all
  sixteen frames read wrong.

- **When a request ends, the wire has ended** (letter w, wireless, thirteen runs
  on each part): the select rises after the last SCK edge, and `start()`'s true
  or `dma_isr()`'s true comes after it too. The letter reads the SCK pad against
  the mode's idle level at the flag the end is read from - the first STATR load
  showing RXNE, the first INTFR load showing the receive channel's completion,
  one frame by hand with interrupts masked - and right after each request: on the
  instruction after `start()` returned true, in the vector after `dma_isr()`
  returned true; all four modes, /256 and /64, read and write, a select and a
  null one, the loops in the flash and (on the CH32V006K8U6) in SRAM. At every
  flag the pad is at its idle level, zero polls to idle, and after every request
  too, 0 of 192 pairs (96 on the CH32V003F4P6). The bound is derived beside the
  letter: half a bit is 128 cycles at /256 and 32 at /64; a poll turn is the
  frame's cycles over the polls a frame the letter prints, some 29 cycles on the
  CH32V003F4P6 (71 polls over a 2048-cycle frame at /256, 17 over 512 at /64)
  and 37 on the CH32V006K8U6 (55 and 14, at RXNE and at the channel's flag).
  So an early flag would leave the pad off idle for some four
  polls at /256 and one at /64 on the CH32V003F4P6, and some three at /256 on
  the CH32V006K8U6, where at /64 a half bit is under a turn and would be seen on
  some passes and not on every one; from /32 up the half bit is under a turn on
  both parts and the instrument cannot see it. The vector reads the pad at
  most 385 to 417 cycles after the flag on the CH32V003F4P6 and 285 to 336 on
  the CH32V006K8U6 (each run's worst), so after the request the letter sees an
  early end only past that. The letter
  has teeth: a scratch host whose transmit-only tail skips the wait on BSY fails
  it, 16 polled requests and 48 on the engines ending with SCK off idle. So the
  host adds no wait for the edge: the receive loop and the vector end at their
  flag, and the transmit-only loop's tail is the wait on TXE and then on BSY it
  needs to know the shifter is empty ("Two polled loops", above).
- **What the engines cannot carry is refused** (letter i, wireless): 16-bit
  frames with the out buffer, the in buffer or a command alone off a half-word
  boundary, and 16-bit frames over engines of `uint8_t` (letter j, an image of
  its own) each answer `spi_rejected` at once, on a select and a D/C that are
  ports of RAM, no store reaching either, CTLR1, SCK and the block as they
  were. An odd command in front of aligned data is not refused (the command
  runs in the loop) and the next requests are `spi_ok`.
- **A request left standing across `release()`** (letter l, wireless, both
  parts, ten runs): `init()` starts from the block's reset line and `release()`
  gates it with no reset, measured with its control on [dma.md](dma.md), "A
  released requester".
- **The one busy check** (letter i, wireless): with a write of 64 frames at
  /256 on the engines, a polled request and one naming the engines each answer
  `spi_rejected` at once, their select and D/C (ports of RAM) untouched and
  CTLR1 as the transfer in flight holds it, and that transfer ends `spi_ok`
  with its one interrupt: the refusal writes its status for its own caller,
  and the completion writes its own over it.

## Which engine to name

The program names the engine; the host runs the request there at any length
([design/spi-bus.md](../design/spi-bus.md), "Two ways to run a request"). What
follows is guidance for that choice in core cycles; nothing in the code reads it.
On the engines a transaction costs the core E, the engines' fixed cost
(`start()`, the launch, the one interrupt, the completion), whatever its length;
polled, it costs a fixed part F and each frame's time, which is the wire's frame
or the loop's own turn, the longer - for a frame of W HCLK cycles at the code,
W plus the receive loop's dead bus for a frame read (the table above), and W,
or the transmit-only loop's turn at HCLK/2, for a frame written. So the engines
cost the core less from the smallest frame count N with F + N x frame >= E,
ceil((E - F) / frame), and only E - F enters. Letter g measures it AS ONE
DIFFERENCE, through one call of the suite on the engined host, so that the call's
own cycles stand on both sides and cancel: E is a 32-frame read block at HCLK/4
less its frames (the frame from a 256-frame block, the best of five each), F the
intercept of a polled line of 6 and one of 14 frames (the best of five), at three
shapes - a read at HCLK/4, a read at HCLK/2, a write at HCLK/2 - and the letter
finds the length itself, the fewest frames whose polled line costs E, and judges
it within a frame of the arithmetic on the same figures (equal in every run).

| measured (letter g, ten runs) | E | F read /4, read /2, write /2 | E - F read, written | from which length, read /4, read /2, write /2 |
|---|---|---|---|---|
| CH32V006K8U6, flash | 787 to 790 | 411, 435, 423 | 376 to 379, 364 to 367 | 7 to 8, 12, 16 |
| CH32V006K8U6, SRAM | 725 | 383, 387, 390 | 342, 335 | 8, 13, 21 |
| CH32V003F4P6, flash | 718 | 482, 478, 492 | 236, 226 | 5, 7, 12 |

The guidance at every code, in byte frames, from those figures (a read's frame
the wire plus the table's dead bus, 22 from the flash and 10 to 16 from SRAM,
14 to 38 by code and mode; a write's the wire, the loop's turn at /2): the
engines cost the core fewer cycles than the loop from

| frames | /2 | /4 | /8 | /16 | /32 | /64 and slower |
|---|---|---|---|---|---|---|
| CH32V006, flash, read | 12 | 7 to 8 | 5 | 3 | 2 | 1 |
| CH32V006, flash, write | 16 | 12 | 6 | 3 | 2 | 1 |
| CH32V006, SRAM, read | 13 | 8 | 5 | 3 | 2 | 1 |
| CH32V006, SRAM, write | 21 | 11 | 6 | 3 | 2 | 1 |
| CH32V003, flash, read | 7 | 5 | 3 | 2 | 1 | 1 |
| CH32V003, flash, write | 12 | 8 | 4 | 2 | 1 | 1 |

In words: one frame already costs the core less on the engines at HCLK/64 and
slower on the CH32V006 and at HCLK/32 and slower on the CH32V003; at HCLK/2 the
loop is the cheaper below 7 to 13 frames read and 12 to 21 written. A 16-bit
frame is twice the wire, so from /4 on the same arithmetic gives about half the
byte count. The figures count core cycles alone: a request on the engines
also leaves the core free through the wire time, which a program that must keep
other work running needs whatever the count says, and its E does not count the
arbiter's round trip ("Not covered yet").

## The client's runs

A client's unit is the run: the answers to a transaction, or what it
brings, served polled in one call. What each costs is counted in the
release listing of `spi_peer`, which inlines all three at both widths
(WCH gcc 15.2, `-Os`, the `xw` ISA), and measured from the far side by
`test_x035_spi`'s letter k: the CH32X035 clocks 32 frames of 8 or 16 bits
under its polled loop - a gap after every frame, the control - and back
to back under its engines, every rung from /64 to /2, and judges each end.

| offer (section) | taken? | why |
|---|---|---|
| TXE, the buffer moved into the shifter at a frame's first sampling edge (16.2.3) | yes | answer k + 1 is written on frame k's TXE: one frame of window, one answer ahead (no FIFO, figure 16-1) |
| RXNE and the one-deep receive buffer (16.2.3, 16.2.7) | yes, read in the middle of a frame | the exchange reads frame k - 1 on its standing RXNE right after frame k's answer is written: a DATAR read at a frame's end lost the next answer, and TXE alone does not say RXNE (both measured, below) |
| the NSS pad (16.2.1) | yes, the window | the chapter has no flag for a client's window end: the pad is read once for every four polls of a flag |
| DMA requests on channels 2 and 3 (table 8-2) | not in a run | a DMA serve is the program's composition of the engines over the resource - its binding, its channels and the latched-request rule (What the silicon does) are the program's - and `spi_peer`'s default serve, at the wire to 24 MHz; a run is the polled serve, with nothing to arm |
| RXNE and TXE interrupts (16.2.8) | not in a run | an interrupt a frame pays the hardware prologue and epilogue, 54 cycles, before a body, against frames of 64 cycles at 6 MHz; `isr()` stands for a program that wants them |
| SRAM execution | the program's | the runs are always inline, so a program places the function that calls them: a plain function with `[[gnu::section(".fast_text")]]`, an inline or template one through `Hot<InRam>::run` ([platform.md](platform.md), "A handler in SRAM"); `spi_peer` keeps its caller in the flash, behind two wait states at 48 MHz (RM 18.3.1), where the fetch is a run's ceiling (measured, below) |

THE COST, in the steady state with each flag found at its first poll,
no call anywhere in a run:

- the EXCHANGE, 13 instructions a frame: the next answer's load, the TXE
  poll's load, test and branch, the DATAR store, the RXNE test's load,
  test and branch (one poll: the flag stands by then), the DATAR
  load, two pointer steps, the frame's store and the loop's branch -
  three taken branches, four peripheral accesses (STATR twice, DATAR
  twice) and two SRAM ones;
- the ANSWER run, 7: the load, the poll's three, the store, the step,
  the branch - two taken, two peripheral accesses;
- the RECEIVE run, 7: the poll's three, the DATAR load, the step, the
  store, the branch - two taken, two peripheral accesses.

A poll that misses is three instructions more, and every fourth a look
at the pad (one load of the port's input register) and the budget's
step. The answer is loaded before the TXE poll, so the store follows the
load that sees the flag with nothing between them.

THE CEILINGS, every rung up to the one named exact and the next one not,
in three runs of the letter on the final images (the tick held, below):

| run | 8-bit, gapped | 8-bit, back to back | 16-bit, gapped | 16-bit, back to back |
|---|---|---|---|---|
| exchange | 6 MHz | 6 MHz | 12 MHz | 12 MHz |
| answer | 24 MHz | 12 MHz | 24 MHz | 24 MHz |
| receive | 24 MHz | 12 MHz | 24 MHz | 24 MHz |

In cycles: the exchange holds a frame of 64 cycles and not one of 32,
the answer and receive runs one of 32 and not one of 16 (the gapped
host's gap after each frame carrying them to 24 MHz in 8-bit frames).
Thirteen instructions in 33 to 64 cycles is 2.5 to 5 cycles an
instruction, the three taken branches refetching from the flash behind
its wait states. The receive run is at the edge of its window there: its
12 MHz back-to-back rung lost one frame in two runs of nine across the
session and its 24 MHz gapped one in one. The wire is not what stops
them: from SRAM the same receive run hears 24 MHz back to back (below).

- **A DATAR READ AT A FRAME'S END LOSES THE NEXT ANSWER.** With frame k
  read on its own RXNE - at the frame's end, a bit before answer k + 1
  moves into the shifter - back to back at 12 MHz in 16-bit frames the
  far host read 56 to 58 of 64 bytes wrong in five runs of five, the
  answer before repeated, while this client heard all 32 frames; with
  frame k - 1 read after the TXE of frame k, the same rung exact in
  every run with the tick held (six). The mechanism inside the block is
  not isolated (a logic analyser on SCK and MISO would show which edge
  the transfer takes).
- **TXE DOES NOT SAY RXNE.** Reading frame k - 1 on the TXE of frame k
  with no RXNE test - one poll a frame - under the gapped host at 12 MHz
  in 8-bit frames, 9 and 10 of the 32 frames read were wrong in two runs
  while every answer was right: the exchange tests RXNE before its read,
  one poll when the flag stands, as it does by then.
- **THE TICK COSTS A FRAME back to back at 6 MHz in 8-bit frames**: its
  handler, a dozen instructions behind the hardware prologue and epilogue
  (about 85 cycles), outlasts a frame of 64. With the tick live two
  letter runs of eight lost one frame at that rung, in the exchange once
  and in a receive run once; `spi_peer` holds its tick for a polled serve,
  and no run since lost one. A program's runs back to back are exact
  while no handler of its image outlasts a frame: a client does not own the
  clock.
- **PLACEMENT**: the same runs from SRAM (a measurement image with the
  peer's serve in `.data`, about 1.1 KB a frame width for the three runs
  inlined) - the exchange exact to 12 MHz in 8-bit frames under both
  hosts, the receive run to 24 MHz back to back, every 16-bit run to 24
  MHz: one rung, a factor of two, over the flash for the exchange. A
  program takes it by placing the function that calls the runs - a
  plain one with `[[gnu::section(".fast_text")]]`, an inline or template
  one through `Hot<InRam>::run` ([platform.md](platform.md), "A handler
  in SRAM"); the runs themselves stay inline in it. THE SPELLING,
  measured: `spi_peer` built with its `polled_serve<T>` marked
  `[[gnu::noinline, gnu::section(".fast_text")]]` - a template whose TU
  places no plain function in that section, so GCC takes it; without
  `noinline` the compiler inlines it back into its caller in the flash -
  1666 bytes of SRAM for both widths, no call out of it, and under letter
  k in two runs the 8-bit exchange back to back at 12 MHz in both (6 MHz
  from the flash), gapped at 12 MHz in one and 6 in the other, the 8-bit
  receive run back to back at 24 MHz in both (12 from the flash), every
  16-bit run at 24 MHz but the back-to-back exchange's 12 in one run.
- **A CLIENT WITH NO SELECT counts bits from SPE and nothing else.**
  `spi_peer` built with its polled serve on a `SpiClient` whose pins name
  no NSS - SSM set, SSI low for the run, each run ending on its count -
  under letter k (two runs): behind the far host's gapped polled loop
  every run exact, the 8-bit exchange to 12 MHz (one rung over the
  pad-watching runs, the look at the pad gone), the answer and receive
  runs to 24 MHz; behind its engines NOTHING exact at any rung, every
  frame one bit off both ways (0xA9 read 0x52, 0x36 heard 0x9B) - an SCK
  edge before the burst, after this client raised SPE, counted as the
  first bit. A selectless run is only as good as the bus's quiet between
  its SPE and the host's first frame; the pad's window is what makes a
  stray edge harmless.

## How to use it

The engined host of the smallest use above, the engines at DATAR's
width so that 8- and 16-bit frames both ride them, the two channels'
vectors bound to `dma_isr()` (the receive channel's is the one that
fires at a block's end; the transmit channel's only on a transfer
error). A complete excerpt of a program's setup and one request; `P` is
the program's platform type, `Display` its active object, `cmd` and
`pixels` its buffers:

```cpp
using Bus = brio::SpiHost<1, brio::spi1_default_pins,
                          brio::DmaTxEngine<3, uint16_t>, brio::DmaRxEngine<2, uint16_t>>;
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
using Cs = brio::Pin<'C', 3>;
static_assert(Bus::has_dma);

if (!Bus::init(clock, 8'000'000)) { /* not even HCLK/256 honours the ceiling */ }
Cs::output(true);

Bus::Request r{};
r.cs = Cs::ref();
r.cmd = brio::lend<brio::Lease::reply>(cmd);  r.cmd_len = 1;
r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
r.clock = brio::SpiClock::div8;
r.mode = brio::SpiMode::mode0;
r.reply = brio::reply_to<Display, brio::SpiDone>();
r.engine = brio::SpiEngine::dma;
brio::post<Spi>(r);

BRIO_CH32_VECTOR(dma1_channel2_handler) {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
BRIO_CH32_VECTOR(dma1_channel3_handler) {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

A host with no engines, `SpiHost<1>`, binds no vector and runs polled
requests alone: each sets `r.engine = SpiEngine::polled` and completes inside the
arbiter's dispatch, its `SpiDone` sent at once; one naming the engines is
answered `spi_rejected`, nothing on the wire. On the engined host a request
naming the engines rides them at any length, and one they cannot carry is
answered `spi_rejected` at once, before anything is touched; the buffers of a
16-bit request on the engines sit on a half-word boundary (`alignas(2)`), and
its engines are of `uint16_t`.

**What each answer asks of the program.** The answer reaches the requester as
the `status` field of its `SpiDone`, in the handler of `Display`:

```cpp
[](const brio::SpiDone& d) {           // a lambda of Display's brio::match
    if (d.status != brio::spi_ok) { /* below; on any status the buffers are the program's again */ }
    return handled();
},
```

| `status` | what happened | what it leaves | who acts |
|---|---|---|---|
| `spi_rejected` | the arbiter's FIFO was full; a transfer of the host's own was still on the engines (a program driving `SpiHost` directly); a host with no engines refused a request naming them; or the engines could not carry it (16-bit frames off a half-word boundary or over engines of `uint8_t`) | nothing on the wire, the select and D/C untouched, a reset still owed left owed | the program: a full FIFO, or a transfer in flight, may be tried again once it has ended - one a lost request left there until `recover()` (below); any other refusal is never cured by posting the request again (`static_assert(Bus::has_dma)`, the buffers aligned, the engines of `uint16_t`) |
| `spi_stalled` | a polled flag never came within the transaction's count of polls | D/C and select released, then the block reset and reconfigured inside `start()`: frames standing or shifting are gone | nobody, for the host: the next request starts on a clean block. The program judges the device |
| `spi_dma_fault` | a channel reported a transfer error | requests down, both channels put away, D/C and select released, a reset owed, which the next request that moves something, or `prime()`, pays before anything else | nobody, for the host: the arbiter answers and goes on, and the next request starts on a clean block. The program judges the device and its buffer |
| `spi_timeout` | a timed arbiter's limit passed with no completion | the arbiter has already called `recover()` | the program judges the device |

A stall holds the core for its count of polls, twice the wire's time for its
frames at four cycles a poll and 256 polls: 1 280 polls for sixteen frames at /16,
and the stall ends some 19 200 to 32 600 cycles after the call (letter h, up to
about 0.7 ms at 48 MHz), every other active object waiting under `Tenuto`. The
D/C and the select go first, then the reset: in modes 2 and 3
the reset moves the SCK pad, and the device must have seen the transaction end.
A transfer error's interrupt decides and does not work: no reset, no configure
and no wait in it, and the next request that moves something resets the block
before it touches anything else. A request already in the arbiter's FIFO starts in the same
dispatch as the failure, before the requester's handler can run, and starts on a
clean block with no help (letter t).

Two things the reset cannot do. It does not open a closed clock gate: there a
polled request stalls for its own bound behind the last one, and one naming the
engines goes to them and has no completion, `spi_timeout` on a timed bus and
a bus busy for ever on an untimed one. And it does not know a select the program
holds by hand (`Request.cs` null): a caller framing its own select calls `prime()`
before it, which pays a reset still owed with the select high, and after a
`spi_stalled` raises its select before the next window, the reset behind the stall
having moved the SCK pad under it in modes 2 and 3.

`recover()` stays for what the host cannot see: a request on the engines that
never answered. Such a request leaves the host's transfer in flight, so the
busy check answers the next request `spi_rejected` rather than a hang, until
`recover()`. The timed arbiter calls it on `spi_timeout` and on no other
answer; it releases the D/C and the select, puts the engines away (the
channels are this host's alone, one owner a channel), drains (its answer is
false when a bounded wait ran out, which only a dead block with SPE up
reaches) and resets. A program may call it. No verb says the engines are idle, and
`recover()` under a request on them takes it away, its completion never coming, so
a program calls it when it knows: every request it has posted answered, on a bus
with one client.

The reset is one cold function in the flash under either placement: it runs at
most once a failure, so `InRam` gives it nothing and pays nothing for it. What
`InRam` costs is the loops' SRAM above; under it `start()`, the launch, the stall's reset and
the owed reset run from the flash as they do by default.

A client answering a transaction in one call, on PC1's hardware select:

```cpp
using C = brio::SpiClient<1>;
C::init<C::Config{.mode = brio::SpiMode::mode3}>(clock);
const auto run = C::exchange(std::span<const uint8_t>(answers), std::span<uint8_t>(heard), spins);
if (run.end != C::RunEnd::complete) { /* run.count frames came */ }
```

## Bench findings

The reference suite is `test_ch32_spi` on the CH32V006K8U6 at 48 MHz, its
SPI1 column 0 (PC5 SCK, PC6 MOSI, PC7 MISO, PC1 the select) wired straight
to the CH32X035F8U6's SPI1 (PA5, PA7, PA6, PA4), the far chip running its
`spi_peer` ([../ch32x035/spi.md](../ch32x035/spi.md)) commanded in band
through the other strata's spi_link protocol - every frame on the wire
witnessed by a second chip, the 16-bit frames, the CRC and a receive-only
client being the protocol's extended exchange. On this part the suite is
five images - whole it passes the 40 KB of program flash -
`test_ch32_spi-1` (the block, the engine a request names, the engines against
the peer, the polled path, the engines, the stalled exit: letters a, i, b, c, d,
h), `-2` (the rate on the wire with the host's costs, the CRC, the kernel, OVR's
clear: g, e, f, v), `-3` (the handler beside the polled loop, the command
channel, the matrix, the ladder, the kernel against the peer, the client, MODF:
k, n, o, p, q, r, m), `-4` (the next request after a failure, the end of the
wire and the same host released with a request standing, then initialized: t,
w, l) and `-5` (the engines of bytes, in an image where theirs is the only
binding armed: j): `z` is 47, 22, 28, 13 and 1 verdicts, green in three runs of
each, letters g, h, i, j, k, l, t and w in ten runs or more: a, c, f, g, h, i, j, l,
v and w with the peer deselected (their select is PC3, a pad with no wire, PC1
held high), the other letters against it; s (the pair's ceiling) and x (the
slips) run by name. On the CH32V003F4P6 the suite is twenty-three group images,
one letter each - the part's 15 KB with no multiplier decide the cut
(design/overview.md) - and with no peer wired to that board the letters that
need none ran: a, c, f and v green in three runs each, g, h, i, j, l and w in
ten runs or more. A letter that needs the peer asks for it first: when no ping
of the boot has been answered it declines with the reason and claims no
verdict, and a peer that answered once and then goes quiet fails it.

- **The reset values are table 16-2's** (STATR 0x0002 with TXE up,
  CRCR 0x0007), and a control word lands as spelled.
- **Seven of seven CTLR1 fields take a write under SPE** - DFF, CRCEN,
  CPOL, CPHA, LSBFIRST, BR and MSTR - so the enable protection the
  chapter describes does not exist in the silicon.
- **A request naming the engines on the host with none is refused**
  (letter a): asked in mode 3 at HCLK/2 after the block was primed in
  mode 0 at HCLK/16, `start()` answers at once `spi_rejected`, the select
  high, CTLR1 the word it was and SCK at its mode-0 idle - nothing reached
  the wire - and the same request naming `SpiEngine::polled` runs `spi_ok`. Through
  the arbiter (letter f): the request naming the engines answered
  `spi_rejected` and the polled one queued behind it `spi_ok`.
- **Every way completes with nothing driving MISO** (letter a): the two
  polled loops, a polled request on the engined host (no DMA interrupt),
  the engines (one DMA interrupt), each releasing its select; no SPI
  interrupt anywhere - SPI1's line disabled, CTLR2's three enables clear,
  its vector never entered.
- **The engine a request names** (letter i, wireless): ONE FRAME at HCLK/2,
  read and written, rides the engines - `start()` false, one DMA interrupt the
  end, `spi_ok` - where a read of one frame in the loop costs the core some 465
  cycles against the engines' 790 on the CH32V006K8U6 and 516 against 718 on the
  CH32V003F4P6 (letter g). The letter has teeth: on the CH32V003F4P6 a scratch
  host that ran a request this short polled whatever it named failed it. THE
  COMMAND RULE, D/C sampled by the tick on PC4, a pad with no wire: a command of
  64 frames at HCLK/256 then 64 data frames, D/C low on the two or three ticks
  inside `start()` and high on the two or three after it while the engines move
  the data, one interrupt; a command of 64 alone, `start()` false with D/C low, low on every
  tick of the block, one interrupt the end, D/C and the select high after it. The
  refusals of "What the engines cannot carry is refused" above, and the next
  requests `spi_ok`.
- **The engines against a foreign client** (letter b): the four
  modes at 8 and 16 bits, sixteen frames each at HCLK/16 on the engines,
  exact both ways with one DMA interrupt a transaction and no SPI one; two
  and eight command frames at HCLK/8 in the loop inside `start()` and 8 and 24
  data frames on the engines (one interrupt each), and eight command frames
  alone on the engines (one) - the client hearing every frame, the data read
  back exact, the command's echo discarded; a read with no out buffer clocking
  the 0xFF cell, heard as such; and one frame read and one written at HCLK/8 on
  the engines, one interrupt each, exact both ways.
- **The polled path at every rate** (letter c, MISO undriven): all eight
  BR codes carry 64 frames to `spi_ok`, none faster than its wire; the
  receive loop's frame 533.9 cycles at HCLK/64 and 1038.2 to 1038.3
  at HCLK/128 in three runs, the bound the wire, 40 cycles of turn (the
  table's worst row, 38, rounded up) and the ticks' 3 and 5.
- **A HANDLER LONGER THAN A FRAME IS A GAP ON THE BUS** (letter k, against
  the peer): the tick's handler dragged by an empty loop to hold main
  context off 2406 to 2440 cycles, past four frames of 512, beside ten
  windows of 128 frames at HCLK/64: ten of ten exact both ways, `spi_ok`, the
  peer's count right, OVR never standing, in ten runs of the letter. A
  window lengthens by 1803 cycles (68 709 undragged, 70 512 dragged, one tick
  landing), where the arithmetic above the letter allows some 1420 to 4960,
  computed from each run's figures.
- **RXNE does not say TXE in modes 1 and 3**: a frame written behind the one
  shifting is still in the buffer when the frame before it raises RXNE, and a
  write made on that RXNE alone replaces it: over the matrix at HCLK/256 against
  the foreign client a loop written so came back wrong in modes 1 and 3, every
  exchange of the one run made so, the client hearing seven frames of eight, one
  bit off, while modes 0 and 2 were exact. The chapter does not order the two
  flags. The same transmit-buffer rule the client's runs learned the other way
  round (TXE does not say RXNE, below). No letter repeats this: the host writes
  nothing ahead on a receive.
- **The engines against the peer** (letter d, HCLK/8): a 128-frame block
  through both engines exact both ways with ONE DMA interrupt and no SPI
  one, no fault on either channel; a command frame in the loop inside
  `start()` then 32 data frames on the engines; a polled request of 64 on
  the engined host running its loop inside `start()`, no interrupt at all, and
  moving its own width after a request on the engines at the other, both
  orders; 32 frames of 16 bits in half-word
  beats, the foreign client hearing 16-bit frames; a 16-bit request off a
  half-word boundary refused with nothing on the wire and the next request
  exact against the foreign client; a read
  with no out buffer clocking the 0xFF cell at both widths, heard as 0xFF.
- **THE RATE ON THE WIRE** (letter g, the peer deselected: ONE FRAME inside
  an engined block, the difference of two block lengths, best of three,
  against its SCK periods at HCLK over the divisor the BR code names,
  16.3.1, with no gap added - a frame written while another shifts
  follows it with none, 16.2.2): from SRAM buffers 1000 to 1002, 999 to 1001,
  1000 and 999 to 1000 thousandths at HCLK/2, /4, /16 and /64 in mode 0,
  1000 at /16 in mode 3, 998 at /4 and 999 to 1000 at /16 in 16-bit frames (ten
  runs); and with no buffer named, on the host's own cells, 999 to 1000 at /2,
  /4 and 16-bit /4. THE CELLS ARE IN SRAM: with the 0xFF cell a constant in
  flash the same block took 1031 thousandths at HCLK/2 - 16.5 cycles
  where 16 are the wire's, the transmit channel reading its source
  behind two wait states (RM V1.5 18.3.1) - and a transmit source in
  SRAM ran at 1000.
- **The host's costs** (letter g): E, F and the length from which the engines
  cost the core less, "Which engine to name" above.
- **The hardware CRC against a foreign CRC unit** (letter e, HCLK/64):
  CRC-8 (0x07) over eight frames and CRC-16 (0x8005) over eight 16-bit
  frames, each end's TXCRCR the bitwise CRC of what it sent (0x62,
  0x5FB6), RXCRCR that of what came (0x5B, 0x2B13), the CRC frame this end
  read the far chip's bitwise CRC of its stream, CRCERR down at both ends;
  and on two polynomials (0x07 here, 0x31 there) CRCERR up at BOTH.
- **The kernel over it, unchanged** (letter f): four transactions through
  SpiBus over the engined host with four replies, the engines and the
  polled loop interleaved on one bus; six posted into a four-deep queue,
  one rejected at once and every request answered exactly once; an idle
  bus votes for the sleep and a busy one - a 64-frame write on the
  engines - against it. Against the peer (letter q): four transactions
  queued from one dispatch in one select window, three on the engines and
  one polled, every byte read back by the far chip.
- **The matrix** (letter o, HCLK/256): the four modes MSb first exact both ways on both of the
  client's serves (its DMA engines and its polled run); this host LSb
  first against the MSb-first client an exact two-way bit reversal in
  every mode; LSb first at both ends heard exact by the client listening
  (it cannot answer so, and refuses to).
- **The ladder** (letter p, /256 to /2): the polled host and the engines
  exact both ways against the client's DMA serve to /4 = 12 MHz; at /2
  the client hears every frame and this host reads its answers wrong -
  the read of an answer at 24 MHz has no timing margin (the pair's
  ceiling, below); this host's transmit-only paths heard whole by a
  listening client at every code to /2; the client's polled run to /8
  behind the polled host and to /16 behind the engines' frames back to
  back - the far chip's run reloads its answers from code in its flash.
- **THIS CHIP AS CLIENT ON ITS HARDWARE NSS** (letter r: PC1 the NSS input,
  SSM clear, under the peer's `host_burst` at 1.5 MHz, one frame a
  request): answering in modes 2 and 3, MSb and LSb first, with a first
  answer bit of 0 and of 1, twelve frames received exact and the far host
  reading the answers exact from the first frame - the preload before the
  select fell; eight 16-bit frames in mode 3 the same; an answering client
  in modes 0 and 1 refused by `SpiClient` before anything is commanded.
- **MODF from a second chip** (letter m): the peer's PA4 held low 2 ms
  demotes a host whose NSS is a hardware input 24 ms after the enable,
  MSTR and SPE cleared, four of four; STATR 0x22 before 16.2.7's clearing
  sequence and 0x02 after it, once the pad has been high a millisecond.
- **THE PAIR'S CEILING, reading an answer at 24 MHz** (letter s, frames on
  this host's engines against the client's DMA serve): to /4 every shape
  is exact both ways - a PRBS each way, MISO quiet low, quiet high,
  toggling every bit under a quiet MOSI, mode 1, mode 3, the client
  listening alone. At /2 the client hears every frame of every shape, and
  this host reads its answers exact while MISO stands still and wrong
  when it toggles - 28 to 32 frames of 32 in modes 0 and 3 (three runs),
  13 in mode 1 in the run printed whole. Table 3-19 of the far
  chip's datasheet gives a client's data valid up to 25 ns after its
  shifting edge and a host 5 ns of setup: 30 ns against the 20.8 ns half
  period of 24 MHz. The far chip's own host closes that gap with its
  high-speed read (HSRXEN; its document); this part's HSCR, the same bit,
  is not used by the host (the gap list). The client's polled run is its
  own ceiling.
- **The host above the wire on the CH32V003F4P6** (`bench_ch32` letters
  s and e, MISO floating, the loop idling through the platform's
  `idle()`, which does not sleep over a working channel, [dma.md](dma.md)):
  the engines at the wire by letter g's measure, 998 to 1000 thousandths of the
  SCK periods at /2, /4, /16 and /64. Letter s's line reads the rate as a
  difference of two walls (five runs): at HCLK/16 128.0 cycles a byte in 8-bit
  frames against 128, the fixed part 1183 cycles at both lengths, and 127.3 to
  127.9 in 16-bit ones; at HCLK/4 30.9 in 8-bit frames and 30.8 to 31.0 in
  16-bit ones against 32. A block cannot beat the clock its BR code makes: the
  walls' fixed parts differ at HCLK/4, 1491 cycles at 32 frames against 1249 at
  256 in 8-bit frames and 1729 against 1219 to 1309 in 16-bit ones, and
  that difference is not explained ("Not covered yet"). The
  polled write 1.05 times the wire at 256 frames of
  HCLK/4; the receive loop's frame over the wire, `spi.frame` from the
  flash, 22 cycles in 8-bit frames and 14 in 16-bit ones, the same at every
  code and in modes 0 and 1. Its E and F are in "Which engine to name" above. And letter c's receive frame 533.9 to 534.0 at HCLK/64 and
  1045.9 to 1046.0 at HCLK/128.

### This part as the desk's instrument: `spi_peer`

`spi_peer` makes the CH32V006K8U6 the desk's SPI client for the
CH32X035's suite (`test_x035_spi`, its document
[../ch32x035/spi.md](../ch32x035/spi.md)): spi_link's command channel in
band over SPI1's column 0 with the HARDWARE select on PC1 (pulled up),
the dark listener, the exchange served on the DMA engines (channels 2 and
3, polled through the receive count, no interrupt) or on the client's
runs (its tick held for the run), `sink_slow`, `ss_pulse` on PC1 as a GPIO,
`host_burst` with PC1 the host's GPIO select; `mspi` refused (no USART of
this part reaches the wires). An answering exchange in modes 0 and 1,
which `SpiClient` refuses, it serves all the same - the block configured
by hand, the resource's verbs on the same pads - so a far suite measures
them; of the
extended exchange it serves the 16-bit frames (the engines in half-word
beats, the runs over `uint16_t`) and the receive-only run (MISO dark) and
refuses the CRC (`report_cfg_failed`, no host on the desk asking it),
raising `report_extended` on every exchange. This die's 24 MHz
crystal runs it (the PLL at 48 MHz), and its supply is read from VREFINT
at boot (3275 mV on the desk). What the far chip measured of this client,
on the four wires:

- **The hardware select frames it**: a client on PC1's NSS input heard
  every exchange the far host clocked in all four modes, both bit orders,
  on both serves - eight frames exact each time - and answered exact in
  modes 2 and 3; the far host's MODF letter took its select low four times
  on command.
- **THE ANSWERING CLIENT NEEDS CPOL = 1** (scratch letters over the far
  host's windows: the select's fall, 30 us, then 8 frames at HCLK/256 or
  32 at HCLK/32): in modes 0 and 1, a window whose first answer bit is 1
  while the host's MOSI is low at the select's fall comes back one bit off
  at BOTH ends - the host reads every byte (t << 1) | the next byte's
  MSB, the client hears the stream one bit late behind a stray bit - and
  the same window is exact with a first answer bit of 0, or with the
  host's MOSI high at the select's fall; in modes 2 and 3 every
  combination is exact. At this pad, an EXTI line on PC5 counted SCK
  edges from the select's fall: none before the host's first clock in an
  exact window, and in a slipped one a stream of edges from the fall on,
  at the rate the handler can count (137 in 20 us), while the host's own
  SCK pad, sampled through its input register after the select's fall,
  read low throughout. Not the wires: with MISO a GPIO held high, or
  raised as a GPIO the instant the select falls, every other line the
  same, the SCK pad counted no edge and the client heard every frame
  exact; with MISO undriven the client heard exact too - the edge needs
  this block's own MISO output rising with the select. Not the host's
  mode change either: with the far host's SCK already at the new idle
  level before this client was configured - no SCK edge from its
  configuration to the select's fall - the same windows slipped and the
  same ones did not. An answer window that follows another one whose
  dummies were zeros meets the condition whenever its frame starts with
  a 1. So the ack, nak, ident and report windows are mode 3 - spi_link's
  alternate answer mode, found by the far host at its rendezvous - and
  `SpiClient` refuses an answering client in modes 0 and 1.
- **The rate ladder**: the DMA serve exact at every code to the far
  host's /2 = 24 MHz, behind its polled loop and its engines; the polled
  serve - the client's exchange run - exact to 6 MHz behind both, the
  run's own ceiling from the flash (The client's runs, above).
- **As a host** (`host_burst`, one polled frame a request, 20 us apart):
  its SCK and MOSI the standard's in all four modes and its frames exact
  at the far client, MSb and LSb first.

## Not covered yet

Driver gaps, each with its reason:

- The client's runs bound their waits by the caller's `spins`, a count the
  program sizes: a client's clock is the far host's, which this chip cannot
  derive a count from; a derived count is born with the first client that serves
  a device with a stated timeout.
- The simplex modes (BIDIMODE/BIDIOE, RXONLY) beyond the resource's
  configuration bits: no user.
- The hardware select arrangements as the ENGINE's select: the engine's
  select is a GPIO on purpose (docs/design/spi-bus.md); the resource
  has SSOE and the multi-host input for a program that wants them.
- The CRC as part of a Request: the resource's verbs exist, born into a
  tenure shape with a device that checks one.
- A verb that says the engines are idle: `recover()` wants nothing on them,
  and the CH32V203's `busy()` has no twin here. A program with one client of
  the bus knows by having nothing outstanding; several clients of one bus that
  recover would want it.
- The arbiter's round trip in E: E counts the host's work through a call
  (the launch, the vector, the completion), not the `TransferDone` the
  program's glue posts and the arbiter's dispatch that answers it; a request
  on the engines through `SpiBus` pays that too, so the guidance
  underestimates the engines' cost to a program - declined as it stands, the
  guidance asking for the order of magnitude; a letter timing a request from
  its post to its `SpiDone` would measure it.

Implemented but not bench-verified, each with what would measure it:

- The client's runs on the CH32V003F4P6: they compile there (`brio check
  ch32v00x`); a CH32V003 wired to the CH32X035 as the CH32V006 is, under
  letter k, would measure them.
- The answering client's CPOL rule on the CH32V003F4P6: `SpiClient`
  refuses an answering client in modes 0 and 1 there as on the CH32V006,
  from its manual's 14.2.3, which says what the CH32V006's 16.2.3 says;
  the slip itself was measured on the CH32V006 alone. A CH32V003 wired to
  the CH32X035 as the CH32V006 is, answering a mode-0 window whose first
  bit is 1 after a window of zeros, would measure it.
- The suite's letters against a peer on the CH32V003F4P6: its twenty-three group
  images link (`brio fit`), the letters that need no peer ran; a CH32V003
  wired to the CH32X035 as the CH32V006 is would run the others. Letter t among
  them, so on that part the next request after a failure is judged by its status
  (letter h) and not by its frames; its image leaves the queued cases out, for
  room, and they would want an image of their own.
- E and F on the CH32V003F4P6 with the host's loops in SRAM: no image of that
  part carries the placement; letter g's second placement built for this part,
  in an image of its own, would measure it.
- The host's read of an answering client at HCLK/2: wrong whenever MISO
  toggles (bench findings, letter s), the timing budget
  of table 3-19 not closing at 24 MHz, so the engines' exchanges at /2
  are judged by what the client hears alone. The far chip's host closes
  the same budget with its high-speed read; letter s run with this part's
  HSRXEN set at /2 would say whether it does here, and the host would
  take it as its option.
- The vendor's loop on this board and this image: the EVT's `2Lines_FullDuplex`
  host side, built over its library with a bound on its turns and run beside the
  host's receive loop at HCLK/4 and HCLK/16, frames counted, would measure what
  its write-ahead buys and what it loses against the bus's one-frame buffer.
- The CH32V003F4P6's engined lines in letter s: why a 32-frame line's fixed part
  runs 242 cycles over the 256-frame line's at HCLK/4 in 8-bit frames, and 420
  to 510 in 16-bit ones. A stamp around
  `start()` and one at the completion on each line would split it.
- HSCR's high-speed read mode: its rate formula against a scope on
  SCK.
