# SPI (CH32V00x)

This document is for a programmer who talks to an SPI device - a
display, a flash, a converter - with the CH32V006 or the CH32V003 as
the bus host, or who makes the part a client another host clocks. It
covers the one SPI of RM ch. 16 - both roles, 8- or 16-bit frames, the
four modes, a software or hardware chip select, hardware CRC, two DMA
requests - in three layers: `Spi<1>`, the resource, its registers as
functions; `SpiHost`, the host engine, which runs one request at a time,
polled inside the call or asynchronously on its DMA engines; and
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
static_assert(Bus::has_dma);                          // it posts asynchronous requests

Bus::init(clock, 8'000'000);                          // a ceiling for the whole bus
Cs::output(true);

Bus::Request r{};
r.cs = Cs::ref();
r.cmd = brio::lend<brio::Lease::reply>(cmd);  r.cmd_len = 1;
r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
r.clock = brio::SpiClock::div4;                       // at 48 MHz: clamped to div8, 6 MHz
r.mode = brio::SpiMode::mode0;
r.reply = brio::reply_to<Display, brio::SpiDone>();
brio::post<Spi>(r);                                   // asynchronous: polled stays false

BRIO_CH32_VECTOR(dma1_channel2_handler) {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
BRIO_CH32_VECTOR(dma1_channel3_handler) {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

Line by line: `Bus` is the host on SPI1's default pads (SCK PC5, MOSI
PC6, MISO PC7) with its two DMA engines, channel 3 transmitting and
channel 2 receiving, each moving up to a 16-bit frame a beat; `Spi` is
the arbiter over it, with room for four pending requests; `Cs` is the
device's chip select, an ordinary GPIO the request carries and the host
drives around the transaction, not the peripheral's NSS pad. The
`static_assert` is the program's own check that the host it names can
run an asynchronous request at all: a host with no engines refuses one.
`Bus::init()` takes the clock task's tag and a ceiling for every
request's SCK, and answers false when not even HCLK/256 honours it;
`Cs::output(true)` makes the select an output driven high, inactive,
before the first request. The request lends the host two buffers
(`lend<Lease::reply>`: the buffers stay the sender's, lent until the
reply comes), names the SCK as a division of HCLK and the mode, and
says where the answer goes: a `SpiDone` event posted to `Display`, the
program's active object that sent it. Its `polled` field is false, so
it runs asynchronously: the command byte, one frame, is short of a DMA
block of its own and goes out polled inside the call, the 64 data bytes
ride the engines, and the call returns while they move. The ceiling
clamps and never speeds up: HCLK/4 is 12 MHz at 48 MHz, over the 8 MHz
ceiling, so this request runs at HCLK/8, 6 MHz. The program binds the
two channels' vectors to the host's `dma_isr()`, under the startup
code's names, and posts the host's status to the arbiter when a
transaction ends.

What decides correctness even there. `init()` after the clock task's,
since the ceiling and the select's timing are computed from the rate
the clock type claims; a request's SCK is a division of the real HCLK,
so on a chip still at its 8 MHz reset rate every transaction runs six
times slower than its numbers say, and nothing refuses it. The buffers
are untouched by the sender from the post until `SpiDone` arrives (the
host and its engines read and write them in between, and nothing checks
that the sender keeps away). Both DMA vectors must be bound: an unbound
one lands in the startup code's spin the first time its channel
interrupts (the receive channel at every block's end, the transmit
channel on a transfer error). A polled request needs no vector: it runs
its loop inside the call at any length, on a host with engines or
without, and touches no channel. A host
with no engines (`SpiHost<1>`) runs polled requests alone and answers
an asynchronous one `spi_rejected` with nothing on the wire. The client
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
host above the wire, cycle by cycle against the wire's figure; the
short request; the client's runs; how to use it, the engines and the
client among the examples; the bench findings, with this part as the
desk's SPI client; and what is not covered.

## Documents of record

The CH32V00X
reference manual V1.5 (16.2.1 for the pins and the select, 16.2.2 and
16.2.3 for the two roles, 16.2.5 for the CRC, 16.2.7 for the errors,
16.3 for the registers), the CH32V006 datasheet V2.0 (table 2-1-1 for
the pads). The header is
[brio/ch32v00x/spi.hpp](../../brio/ch32v00x/spi.hpp); the reference
suite is `test_ch32_spi`, and the host's costs are `bench_ch32`'s and
that suite's letter g's - the short request's figures (a
suite's letter, below, is one command key of its console and the group
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
  (16.2.2: "if the TXE flag is set, fill the data register, maintain
  the complete data flow"); and the receive buffer holds one frame - a
  frame that ends while the one before it stands unread is lost, OVR
  set, the transmit side untouched (16.2.7), the flag cleared by a
  DATAR read followed by a STATR read. The host's two loops are written
  from those two sentences: the transmit-only loop uses the write-ahead,
  and the receive loop declines it (The host above the wire).
- **RXNE does not come half a bit early** (measured; `bench_ch32`'s
  `spi.frame` table, below). 16.2.2 raises RXNE at "the last sampling clock
  edge of the data word", which in CPHA = 0 is half a bit before the frame
  ends, so a write made on it would land in the buffer and follow with no
  gap. It does not: in mode 0 a frame carries a dead bus at every code, on
  the CH32V006K8U6 and the CH32V003F4P6, at /256 too, 22 cycles from the
  flash, where that half bit is 128 cycles and the loop's turn under 40.
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
  clearing sequences (OVR: DATAR then STATR; MODF: STATR then a CTLR1
  write), the CRC verbs (`crc_next()`, `tx_crc()`, `rx_crc()`),
  `software_select()`, `high_speed_read()`, `dma_requests()`, the
  three interrupt enables and `isr()`, the raised-and-enabled sources.
- `SpiHost<1, pins, TxEngine, RxEngine, Placement>` is the
  engine `SpiBus` (= `BusMaster`) drives. Its `Request` carries the other
  strata's fields name for name: a chip select `PinRef` and a D/C line, a
  command phase (D/C low), a data phase with an optional out buffer (null
  = 0xFF dummies) and an optional in buffer (null = discard), the
  per-request `mode`, `clock` and `bits`, and `polled`, the way to run it
  ([design/spi-bus.md](../design/spi-bus.md), "Two ways to run a
  request"): true spins inside `start()` and completes there, the explicit
  loops at any length and no channel touched; false is
  ASYNCHRONOUS, and asynchronous is the DMA engines - `start()` launches
  them and returns, the receive channel's completion ends the
  transaction. Two rules complete the asynchronous way. A host with no
  engines bound refuses it: `start()` answers at once with
  `spi_rejected`, before it touches the block, the select or the D/C, and
  `has_dma` (false there) lets a program check at compile time. And a
  request too short to repay the engines' fixed cost runs polled inside
  `start()` - `async_min_frames(code, bits, read)` is the count from which
  a phase rides them (the short request, below) - as does a 16-bit
  request the engines cannot carry (an odd buffer, or `uint8_t` engines),
  since a refusal would make the answer depend on where a buffer sits. A
  command phase rides the engines on a block of its own when it is the
  whole request or reaches the count by itself, chained to the data block
  from the receive channel's vector; a shorter one goes out polled inside
  `start()` before the data block. The order of the fields is this
  host's: forty bytes with no padding, the eight words an asynchronous
  tenure needs first, then one word holding `clock`, `mode` and `bits`
  behind `cs_setup_us`, then the reply (the next section). `status()`
  answers `spi_ok`, `spi_rejected` (an asynchronous request on a host with
  no engines), `spi_dma_fault` (a block the engines could not finish) or
  `spi_stalled` (a polled flag never came within the transaction's
  budget). `init(clock, max_sck_hz)`, `rebase()`, `clock_for(hz)`,
  `prime()` for a caller framing its own select, `bit_order()`,
  `dma_isr()`, `recover()`, `release()`, `claim_nss_pad()`. The host arms
  no interrupt of the block: SPI1's vector is not its own. `Placement` is
  where the two polled loops run from, util/placement.hpp's `InFlash` (the
  default) or `InRam` - each width of each loop then one instance in SRAM,
  a call a phase ([platform.md](platform.md), "A handler in SRAM"); any
  other tag is refused at compile time; the short request's figures follow
  it. The engine slots are `DmaTxEngine<3, Elem>` and `DmaRxEngine<2,
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

[brio/ch32v00x/pin.hpp](../../brio/ch32v00x/pin.hpp)'s `PinRef` (the
runtime pin the request carries) and `Pad` (the compile-time pad name
the pin tables use) were born with this driver.

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

- **The request is lent for the call** (design/spi-bus.md): a request
  that completes inside `start()` is read through the reference - copied
  nowhere; one that goes to the engines has its tenure copied inline, the
  two pins and the three buffers as seven word loads and stores and the
  two lengths as a half-word and a byte, never a call. The
  descriptor's layout makes those its first eight words and leaves no
  padding: 40 bytes where the same fields in the other strata's order
  take 48 with twelve of padding.
- **`apply()` compares one word.** The request's `clock`, `mode` and
  `bits` lie together behind `cs_setup_us` at a word boundary, so the
  compare with the last request's is one load and a shift against a
  kept word: nine instructions with the ceiling's clamp, nothing
  written. On a mismatch the three are folded into CTLR1 and compared
  with the word in force (two requests under a ceiling may fold to the
  same word); a changed word is written with the drain and the
  disable/enable pair 16.3.1 asks for DFF, and what the code and width
  in force decide beyond CTLR1, the short request's two counts, is
  computed there, never per request.
- **Two polled loops.** A phase whose answers nobody wants - the
  command phase, a write with no in buffer - runs the TRANSMIT-ONLY
  loop: a frame written whenever TXE says the buffer is free, which it
  is a whole frame time before the shifter needs it, so the clock never
  pauses; the answers overrun the receive buffer, harmless to the
  transmit side by 16.2.7, and the tail waits TXE then not BSY and
  clears RXNE and OVR with the chapter's DATAR-then-STATR read. Nine
  instructions a frame at 8 bits (twelve at 16), and MEASURED AT THE
  WIRE from HCLK/4 in both widths: 31.9 cycles an 8-bit frame against
  the wire's 32 at HCLK/4, 127.9 and 255.9 at HCLK/16 against 128 and
  256. An interrupt that lands in the loop only pauses the stream, and
  costs what it outlasts the frame's slack: the console's interrupts
  inside a 16-bit run at HCLK/4 cost 3 cycles a frame (67.3 against
  64). A phase with an in buffer runs the RECEIVE loop, ONE FRAME IN
  FLIGHT at every rate: write k, poll RXNE, read k and write k + 1 in the
  next instruction, the next frame's load and the store of the one read
  placed inside the wire time; eleven instructions a frame at 8 bits with
  one taken branch, of which four on the bus's dead time (the poll's last
  turn, the DATAR read, the DATAR write). This host owns the clock and the
  clock waits for the read, so a handler or a preemption of any length is
  a gap on the bus: the loop cannot lose a frame and tests no OVR. Both
  loops spend ONE bounded budget of poll turns for the whole transaction
  (a frame's cycles per frame, a shift and no multiply) and end with
  `spi_stalled` when a flag never comes.
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
  flash in mode 0, 54 in mode 1. The 24 cycles between a row's best and worst
  are one turn of the poll from the flash, whose miss takes two taken
  branches: which turn sees the flag is fixed by the code and the mode. A
  read that wants the wire asks asynchronously. Under the benchmark's console
  and tick the same loop reads 56.8 cycles a byte at HCLK/4, 154.8 at
  HCLK/16 and 31.9 at HCLK/2, and 94.9, 283.0 and 69.5 a 16-bit frame.
- **The loops are aligned to the fetch's word.** From the flash the
  receive loop's frame is a function of where the loop starts modulo
  four: half a word moved it by 6 cycles at HCLK/4 and 3 at HCLK/2, the same
  instructions, the same image otherwise (letter g, two layouts each way).
  `-Os` aligns a
  function to two bytes, so the host aligns both loops' functions to
  four, two bytes of padding at most, and its constants are the aligned
  loops'.
- **The loops from SRAM** (`Placement` = `InRam`, measured in the same
  letters through a twin host): the transmit-only loop at the wire from
  HCLK/2, 16.0 cycles an 8-bit frame against the flash's 24.9 (the
  wire's 16) and 33.6 a 16-bit one against 37.8 (32); the receive loop
  29.8 cycles an 8-bit frame at HCLK/2 against 31.9, 50.7 at HCLK/4
  against 56.8, 144.3 at HCLK/16 against 154.8. The SRAM loop's instance
  sits where the image puts it, util/placement.hpp's trampoline not
  aligning it, and its dead bus moves with that: 8 to 16 cycles at
  HCLK/4 across images. Each width of each loop is one instance in SRAM,
  called once a phase: 460 bytes a host type on the CH32V006 (98 and 108 the
  transmit-only loop's, 106 and 148 the receive loop's) and 468 on the
  CH32V003, against 2 KB there.
- **The select and D/C edges** are one store each into BSHR and BCR
  (RM 7.3.1.4, 7.3.1.5), inline, the null D/C a branch; `cs_setup_us`
  is tested before anything is called; no flush of the receive buffer
  at the start of a transaction, because every path leaves it empty at
  its end (the receive loop reads every frame, the transmit-only loop
  flushes after its tail, the receive engine takes every frame, the
  fault exits flush) and `init()` and `recover()` flush what a reset or
  a `prime()` left.
- **The engines' cost**, `bench_ch32` letter s: an asynchronous write of
  32 or 256 frames runs at the wire inside its block - 32.0 cycles a
  byte at HCLK/4 and 128.0 at HCLK/16 in both widths, one interrupt a
  transaction - and 943 cycles above the wire's time (946 in 16-bit
  frames), the line's instrument inside (the request built inside the
  stopwatch, `BenchIdle`'s turn, the stopwatch, the handler's stamp
  pair: some 360 cycles). The launch is the two engines' block starts,
  five stores and no load each ([dma.md](dma.md)), with SPI1's two
  DMA-request read-modify-writes around them; the completion is the
  receive channel's vector, one INTFR load and the INTFCR store, the
  select released. What the short request needs of it is E against the
  polled path's fixed part, measured as one difference (the next
  section).
- **The fixed cost of a request**, a 3-byte polled one (a command and
  two data bytes, D/C scripted, the rate unchanged), MEASURED: 374
  cycles above the wire's 384 at HCLK/16 with the stopwatch's 44
  inside, about 330 the host's own; 297 for a lone command byte, 372
  for a command and 15 data bytes.
- **THE VENDOR'S OWN LOOP, run beside this host's on the same board**:
  the EVT's `2Lines_FullDuplex` host side transcribed over the EVT's
  library (pinned archive 189c91b, `-Os`, its calls out of line in their
  own TU as the EVT builds them), 51 instructions a frame through three
  library calls, with one addition - a bound on its turns, a decrement
  and a branch, because the loop has none - in a scratch image with
  this host's polled receive in it, the far client deselected and MISO
  floating, the best of four runs of 16 and 256 frames, the image run
  three times. It sends on every TXE and reads on every RXNE in one turn,
  each frame written ahead with nothing holding it back: at HCLK/16 it
  runs 141 cycles a frame in 8-bit frames (the wire 128) and 256 in
  16-bit ones (the wire 256), every frame received, where this host's
  receive loop, one frame in flight, takes 150 and 282 with interrupts
  masked - the vendor's ahead by a twentieth to a tenth there; at HCLK/4
  it LOSES FRAMES: 8 of every 16 in 8-bit frames in every run, its loop
  ending only on the added bound, and one of 16 or 256 in a third of its
  16-bit runs, where this host's receive loop is exact at 54 and 90
  cycles a frame (1.69 and 1.41 times the wire). The vendor's loop takes
  at every rate the write-ahead this host's receive loop declines.
  The vendor's DMA read (the EVT's WUI application,
  a serial flash's read: `SPI_Flash_Read_dma_start` and
  `SPI1_Read_DMA`) sends its command and address bytes polled, one call
  a byte, raises both DMA requests for the data block and drops them at
  its end, and per block disables the two channels, clears a flag and
  writes the counts and the receive address before enabling them: the
  shape this host takes - a short command phase polled, the data on the
  engines, the requests around the block (What the silicon does: a
  request a disabled channel has not taken is latched).
- **`spi_stalled`, provoked** (letter h): a polled duplex request of 16
  at /16 with SPI1 HELD in reset - its registers ignoring every write,
  RXNE never rising - ends `spi_stalled` after about 50 000 cycles, the
  poll budget against the wire's 2048, and `recover()` puts the next
  request back to `spi_ok`.

## The short request

An asynchronous request rides the engines only when they repay their
fixed cost; a shorter one runs polled inside `start()`
(design/spi-bus.md, "Two ways to run a request"). On the engines a
transaction costs the core E, the engines' fixed cost (`start()`, the
launch, the one interrupt, the completion), whatever its length; polled,
it costs a fixed part F and each frame's time, which is the wire's frame
or the loop's own turn, the longer - for a frame of W HCLK cycles at the
code, max(W + the receive loop's dead bus, its shortest turn) for a
frame read, max(W, the transmit-only loop's shortest turn) for a frame
written. So the request rides the engines from the smallest frame count
N with F + N x frame >= E, ceil((E - F) / frame), and only E - F enters,
one for a read and one for a write. Letter g measures it AS ONE
DIFFERENCE, through one call of the suite on the engined host, so that
the call's own cycles stand on both sides and cancel: E is a 32-frame
read block at HCLK/4 less its frames (the frame from a 256-frame block,
the best of five each), F a polled line's fixed part (6 and 14 frames,
the best of five), at three shapes - a read at HCLK/4 (the receive
loop's dead bus), a read at HCLK/2 (its turn), a write at HCLK/2 (the
transmit-only loop's turn) - and the letter finds the edge itself, the
fewest frames whose polled line costs E, beside the arithmetic on the
same figures.

Measured over several images, the last one ten runs: from the flash E
986 to 1059 cycles through the call (the controller's and the fetch's
wander, by the image), E - F 431 to 504 for a read and 434 to 510 for a
write; the receive loop 54.0 cycles a frame at HCLK/4 (22 of dead bus)
and 30.0 at HCLK/2, the transmit-only loop 24.0 at HCLK/2. From SRAM,
E - F 389 to 457 and 417 to 450, the dead bus 8 or 16 by the layout
(above), the turns 28 to 29 and 16. The host's constants
(`spi_costs_in_flash` {475, 480, 22, 30, 24}, `spi_costs_in_ram` {415,
430, 12, 29, 16}) give, from the flash, a read of byte frames on the
engines from 13, 9, 6, 4 and 2 frames at HCLK/2 to /32 and from one at
/64, a write from 20, 15, 8, 4 and 2. The edge measured stands within a
frame of the arithmetic on the same figures at every shape and run - 8 to
10 frames for a read at HCLK/4, 15 or 16 at HCLK/2, 19 or 20 for a write -
and the host's own count at HCLK/2 reads 13 where 16 are measured: the
dead bus at HCLK/2 is 14 cycles where at HCLK/4 it is 22, the rule taking
the larger (a request of 13 to 15 frames costs the core some 60 cycles
more on the engines than polled - the rule asks for the order of magnitude,
and gets it).

NO MULTIPLY ON THE PATH. The CH32V003's core has no multiplier (a
multiply there is a call into libgcc), so the host keeps no frame cost
to multiply by: it keeps the two COUNTS for the code and width in force
- a read's and a write's, from a table the compiler fills at every code
and width - and compares a request's frames, `cmd_len + len`, with its
data phase's count: five instructions in `start()` on the asynchronous
path, nothing on a polled one. The one approximation: a read's command
frames are counted at a frame read's cost, the dearer turn, so a read
with a command phase may ride the engines a frame early for each
command frame. A command phase rides a block of its own when it is the
whole request or reaches the write count by itself; it is chained to
the data block from the receive channel's vector - two interrupts - and
a shorter one goes out polled inside `start()` before the data block.

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

The asynchronous host of the smallest use above, the engines at DATAR's
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
brio::post<Spi>(r);

BRIO_CH32_VECTOR(dma1_channel2_handler) {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
BRIO_CH32_VECTOR(dma1_channel3_handler) {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

A host with no engines, `SpiHost<1>`, binds no vector and runs polled
requests alone: each sets `r.polled = true` and completes inside the
arbiter's dispatch, its `SpiDone` sent at once; an asynchronous one is
answered `spi_rejected`, nothing on the wire. On the engined host a
request short of `async_min_frames()` runs polled inside `start()`
whichever way it asked, and its `SpiDone` comes at once too.

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
three images - whole it passes the 40 KB of program flash -
`test_ch32_spi-1` (the block, the asynchronous way, the polled path, the
engines, the stalled exit: letters a, b, c, d, h), `-2` (the rate on the
wire with the host's costs, the CRC, the kernel: g, e, f) and `-3` (the
handler beside the polled loop, the command channel, the matrix, the
ladder, the kernel against the peer, the client, MODF: k, n, o, p, q, r, m):
`z` is 33, 18 and 28 verdicts, green in three runs of each on the final
images and letters a, c, d, g and k in ten to fifteen: a, c, f, g and h with
the peer deselected (their select is PC3, a pad with no wire, PC1 held high),
the other letters against it; s (the pair's ceiling) and x (the slips) run
by name. On the CH32V003F4P6 the suite is seventeen group images, one letter
each - the part's 15 KB with no multiplier decide the cut
(design/overview.md) - and with no peer wired to that board the letters that
need none ran: a, c, f and h green three times each, and g, whose constants
are the CH32V006's (below, "Not covered yet").

- **The reset values are table 16-2's** (STATR 0x0002 with TXE up,
  CRCR 0x0007), and a control word lands as spelled.
- **Seven of seven CTLR1 fields take a write under SPE** - DFF, CRCEN,
  CPOL, CPHA, LSBFIRST, BR and MSTR - so the enable protection the
  chapter describes does not exist in the silicon.
- **An asynchronous request on the host with no engines is refused**
  (letter a): asked in mode 3 at HCLK/2 after the block was primed in
  mode 0 at HCLK/16, `start()` answers at once `spi_rejected`, the select
  high, CTLR1 the word it was and SCK at its mode-0 idle - nothing reached
  the wire - and the same request marked polled runs `spi_ok`. Through
  the arbiter (letter f): the asynchronous request answered
  `spi_rejected` and the polled one queued behind it `spi_ok`.
- **Every way completes with nothing driving MISO** (letter a): the two
  polled loops, a polled request on the engined host (no DMA interrupt),
  the engines asynchronous (one DMA interrupt),
  a command phase of 32 frames on a block of its own chained to its data
  block (two channel interrupts) and alone (one), each releasing its
  select; the short request's edge at HCLK/4 for a read, nine frames:
  eight asynchronous frames complete inside `start()`, nine go to the
  engines; no SPI interrupt anywhere - SPI1's line disabled, CTLR2's
  three enables clear, its vector never entered.
- **The asynchronous way against a foreign client** (letter b): the four
  modes at 8 and 16 bits, sixteen frames each at HCLK/16 on the engines,
  exact both ways with one DMA interrupt a transaction and no SPI one; two
  command frames at HCLK/8, short of a block, polled inside `start()` and
  eight data frames on the engines (one interrupt), and eight command
  frames on a block of their own chained to 24 data frames (two) - the
  client hearing every frame, the data read back exact, the command's
  echo discarded; a read with no out buffer clocking the 0xFF cell, heard
  as such; and three frames at HCLK/8, under the edge, run polled inside
  `start()` with no interrupt at all, exact both ways.
- **The polled path at every rate** (letter c, MISO undriven): all eight
  BR codes carry 64 frames to `spi_ok`, none faster than its wire; the
  receive loop's frame 534.0 to 534.1 cycles at HCLK/64 and 1038.1 to 1038.3
  at HCLK/128 in fifteen runs, the bound the wire, 40 cycles of turn (the
  table's worst row, 38, rounded up) and the ticks' 3 and 5.
- **A HANDLER LONGER THAN A FRAME IS A GAP ON THE BUS** (letter k, against
  the peer): the tick's handler dragged by an empty loop to hold main
  context off 2406 to 2440 cycles, past four frames of 512, beside ten
  windows of 128 frames at HCLK/64: ten of ten exact both ways, `spi_ok`, the
  peer's count right, OVR never standing, in eighteen runs of the letter. A
  window lengthens by 2290 to 2298 cycles (68 818 to 68 826 undragged, 71 116
  to 71 124 dragged, one tick landing), where the arithmetic above the letter
  allows some 1450 to 4900, computed from each run's figures.
- **RXNE does not say TXE in modes 1 and 3**: a frame written behind the one
  shifting is still in the buffer when the frame before it raises RXNE, and a
  write made on that RXNE alone replaces it: over the matrix at HCLK/256 against
  the foreign client a loop written so came back wrong in modes 1 and 3, every
  exchange of the one run made so, the client hearing seven frames of eight, one
  bit off, while modes 0 and 2 were exact. The chapter does not order the two
  flags. The same transmit-buffer rule the client's runs learned the other way
  round (TXE does not say RXNE, below). No letter repeats this: the host writes
  nothing ahead on a receive.
- **A status copy taken before the DATAR read misses an overrun**: a frame
  that completes at any moment before the DATAR read of the one before it
  raises an OVR only a STATR load after the read sees - measured with a
  per-frame interrupt handler at HCLK/32 and HCLK/64, its entry swept across
  the completion over 1024 runs a line: with OVR read first, 1 to 3 runs a
  line never completed (the last frame lost uncounted) and most of those that
  lost their last frame left OVR standing for the next transaction; with the
  STATR load after the DATAR read, every run completed with the loss counted
  and no OVR standing. It is for a program that reads frames from its own
  handler with the resource's verbs; DATAR then STATR is 16.2.7's clear, run
  in the same turn.
- **The engines against the peer** (letter d, HCLK/8): a 128-frame block
  through both engines exact both ways with ONE DMA interrupt and no SPI
  one, no fault on either channel; a command frame polled inside
  `start()` then 32 data frames on the engines; a polled request of 64 on
  the engined host running its loop inside `start()`, no interrupt at all, and
  moving its own width after an asynchronous request at the other, both
  orders; 32 frames of 16 bits in half-word
  beats, the foreign client hearing 16-bit frames; a 16-bit request off a
  half-word boundary run polled inside `start()`, no interrupt; a read
  with no out buffer clocking the 0xFF cell at both widths, heard as 0xFF.
- **THE RATE ON THE WIRE** (letter g, the peer deselected: ONE FRAME inside
  an engined block, the difference of two block lengths, best of three,
  against its SCK periods at HCLK over the divisor the BR code names,
  16.3.1, with no gap added - a frame written while another shifts
  follows it with none, 16.2.2): from SRAM buffers 1000 to 1002, 998,
  999 and 999 to 1000 thousandths at HCLK/2, /4, /16 and /64 in mode 0,
  1000 at /16 in mode 3, 1000 at /4 and /16 in 16-bit frames (ten runs);
  and with no buffer named, on the host's own cells, 999 to 1000 at /2,
  /4 and 16-bit /4. THE CELLS ARE IN SRAM: with the 0xFF cell a constant in
  flash the same block took 1031 thousandths at HCLK/2 - 16.5 cycles
  where 16 are the wire's, the transmit channel reading its source
  behind two wait states (RM V1.5 18.3.1) - and a transmit source in
  SRAM ran at 1000.
- **The host's costs** (letter g): the short request's figures and edge,
  "The short request" above.
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
  `idle()`): the engined write at the wire - 31.3 cycles a byte at
  HCLK/4 against 32, 126.5 at HCLK/16 against 128, one DMA interrupt -
  once the V2A's `idle()` stopped sleeping over a working channel
  (dma.md: before it the receive channel overran and the transaction
  never ended); the polled write 1.04 of the wire at 256 frames of
  HCLK/4; the receive loop's frame over the wire, `spi.frame` from the
  flash, 22 cycles in 8-bit frames and 14 in 16-bit ones, the same at every
  code and in modes 0 and 1. Its short-request figures, letter g through the
  same call: E 855, E - F 293 for a read and 283 for a write, the receive loop
  54.0 cycles a byte at HCLK/4 and 38.0 at HCLK/2, the transmit-only loop 20.0
  at HCLK/2 - against constants that are the CH32V006's, so the letter's
  constants verdict fails there, its edge verdict holding (below, "Not covered
  yet"). And letter c's receive frame 534.0 at HCLK/64 and 1045.9 at HCLK/128.

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

- The simplex modes (BIDIMODE/BIDIOE, RXONLY) beyond the resource's
  configuration bits: no user.
- The hardware select arrangements as the ENGINE's select: the engine's
  select is a GPIO on purpose (docs/design/spi-bus.md); the resource
  has SSOE and the multi-host input for a program that wants them.
- The CRC as part of a Request: the resource's verbs exist, born into a
  tenure shape with a device that checks one.
- The arbiter's round trip in the short request's E: E counts the host's
  work through a call (the launch, the vector, the completion), not the
  `TransferDone` the program's glue posts and the arbiter's dispatch that
  answers it; an asynchronous request through `SpiBus` pays that too, so
  the count is an underestimate of the engines' cost to a program -
  declined as it stands, the rule asking for the right order of
  magnitude (design/spi-bus.md); a letter timing a request from its post
  to its `SpiDone` would measure it.

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
- The suite's letters against a peer on the CH32V003F4P6: its seventeen group
  images link (`brio fit`), the letters that need no peer ran; a CH32V003
  wired to the CH32X035 as the CH32V006 is would run the others. Its short
  request's constants are the CH32V006's, measured apart: E - F 293 and 283
  against the constants' 475 and 480 (bench findings), which put a read at
  HCLK/4 on the engines from 9 frames where 6 repay them there. A part's own
  constants would need a place in the host, and a request those few frames from
  the edge costs some tens of cycles either way.
- The host's read of an answering client at HCLK/2: wrong whenever MISO
  toggles (bench findings, letter s), the timing budget
  of table 3-19 not closing at 24 MHz, so the engines' exchanges at /2
  are judged by what the client hears alone. The far chip's host closes
  the same budget with its high-speed read; letter s run with this part's
  HSRXEN set at /2 would say whether it does here, and the host would
  take it as its option.
- The benchmark's engined write's fixed cost on the CH32V003F4P6: `bench_ch32`
  letter s on that part would measure it (its figures above are the engines'
  rate).
- HSCR's high-speed read mode: its rate formula against a scope on
  SCK.
- `spi_dma_fault`: only a transfer error on a channel answers it now, and no
  letter makes one; a block pointed at an address the bus refuses would.
