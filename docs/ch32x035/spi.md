# SPI (CH32X035)

This document is for a programmer who needs an SPI bus: a display, a
flash chip or a converter driven from an active object (a component of
a brio program with its own event queue, served one event at a time by
the kernel), or this chip as a client another host clocks. It covers
the one SPI of RM ch. 16, host and client, 8- or 16-bit frames, the
four clock modes, a software or hardware chip select, hardware CRC, the
simplex arrangements, two DMA requests; and the host engine `SpiHost`,
which the bus arbiter of util/spi_bus.hpp drives on this silicon as on
every other stratum: an active object posts a request, the arbiter
starts one at a time, and the host asserts the request's chip select and
runs the command and data phases - polled inside the call, or
asynchronously on the DMA engines; the arbiter replies with an event when
the last frame is out ([design/spi-bus.md](../design/spi-bus.md)).

The smallest use, a command and 64 frames of pixels to a display:

```cpp
using Bus = brio::SpiHost<1, brio::spi1_default_pins,  // PA5 SCK, PA7 MOSI, PA6 MISO
                          brio::DmaTxEngine<3, uint8_t>, brio::DmaRxEngine<2, uint8_t>>;
using Spi = brio::SpiBus<Bus, P, 4, brio::BusPassThrough,
                         brio::ticks_from_ms<P>(20)>; // the arbiter AO, a 20 ms limit
using Cs = brio::Pin<'A', 4>;
static_assert(Bus::has_dma);                          // its requests are asynchronous

Bus::init(clock, 8'000'000);                          // a ceiling for the whole bus
Cs::output(true);

Bus::Request r{};
r.cs = Cs::ref();
r.cmd = brio::lend<brio::Lease::reply>(cmd);  r.cmd_len = 1;
r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
r.clock = brio::SpiClock::div8;                       // 6 MHz at 48
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

This is an excerpt: it assumes the program's platform `P`, its clock
object `clock`, an active object `Display` that handles `SpiDone`, the
buffers `cmd` and `pixels`, and the statements where they belong, the
setup in `main()` and the post in a handler.

`Bus` is the engine over SPI1 on its default column, the one the QFN20
bonds, with the DMA engines on the two channels this instance's requests
reach; `Spi` the arbiter, an active object listed in the program's
`Tenuto` with room for four requests waiting, and a per-bus timeout
sized above the longest legal transaction, without which a channel that
never completes would leave the bus busy for ever (the fault exits,
below); `Cs` a plain pin the request
carries as its chip select, driven high before anything else so the
display stays deselected; `init()` takes the clock task's tag
([clock.md](clock.md)) and a ceiling for the clock, under which every
request's rate is clamped; the request lends its two buffers
(`brio::lend`) until the `SpiDone` event reaches `Display`, so `cmd`
and `pixels` must stay untouched and in scope until then; the two
`BRIO_CH32_VECTOR` bindings are the channels' interrupt handlers,
`extern "C"` functions as in C under the crt's names
([platform.md](platform.md)), posting the completion to the arbiter.
The request is not marked `polled`, so it runs asynchronously, which on
this host means on the engines: the command byte polled inside
`start()` (one frame repays no DMA block), the 64 pixels a block, and
one interrupt at its end. What decides correctness even there: the
clock task initialized before all this (the ceiling and the chip-select setup time are
computed from the rate the clock type claims); the engines bound and
both vectors too, since an asynchronous request completes on the
receive channel's interrupt and a host with no engines refuses it
(`spi_rejected`, the `static_assert` above says so at compile time), while
a request marked `polled` runs its loop inside the call at any length,
touching no channel and needing no vector; the
select pin driven high before the first request is posted, the host
asserting it low around a transaction and leaving it high after; and
the mode and the rate the device wants, which travel in every request
and are checked by nothing but the device.

The rest of this page, in order: the documents of record; what the
silicon does, with the chapter's offer item by item; the types and
verbs; how to use it, the engines, the placement in SRAM and the client
included; the bench findings, this part as the desk's SPI instrument
and the host's cost above the wire among them; what is not covered.

## Documents of record

The CH32X035 reference manual V1.8 (16.1 for the features, 16.2.1 for
the pins and the select, 16.2.2 and 16.2.3 for the two roles, 16.2.4
for the simplex modes, 16.2.5 for the CRC, 16.2.7 for the errors, 16.3
for the registers; table 8-4 for the pads, 8.3.2.1 for SPI1_RM; RM 9.2.3
for the DMA requests; table 7-1 for the vector), the CH32X035/X033
datasheet V1.7 (table 2-1 for the pads, table 3-19 for the timing, table
3-14 for the pads' speed), and WCH's EVT library (`ch32x035_spi.c` and
the six SPI examples) as the vendor's shape. Driver:
[brio/ch32x035/spi.hpp](../../brio/ch32x035/spi.hpp), the columns in
[brio/ch32x035/afio.hpp](../../brio/ch32x035/afio.hpp); reference suite
`test_x035_spi` (a suite's letter, below, is one command key of its
console and the group of verdicts it runs; `z` runs the single-board
set and `y` the two-board set) against the desk's CH32V006K8U6 running
its `spi_peer` ([../ch32v00x/spi.md](../ch32v00x/spi.md)); this chip's
own `spi_peer` is that board's suite's instrument (below).

## What the silicon does

- **The STM32F1's SPI under WCH's names**, with two registers' worth of
  its own: HSCR (16.3.8, a "high-speed read" mode) and CTLR2.ODEN (16.3.2,
  open-drain NSS, SCK and MOSI).
- **No FIFO.** One transmit buffer, one shift register, one receive buffer
  (figure 16-1). TXE rises "when the data has reached the shift register"
  and a frame written then follows the one shifting with no gap ("fill the
  data register with data to maintain the complete data flow", 16.2.2);
  RXNE rises at "the last sample clock edge of the data word". The
  receive buffer holds one frame: a frame that completes while the one
  before it stands unread is lost, OVR set (16.2.7); the buffer keeps the
  earlier frame, which the chapter does not say (measured, letter o).
- **RXNE does not come half a bit early** (measured, letter b's `spi.frame`
  table, below). 16.2.2 raises RXNE at "the last sample clock edge of the
  data word", which in CPHA = 0 is half a bit before the frame ends, so a
  write made on it would land in the buffer and follow with no gap. It does
  not: a frame's dead bus from the flash is the same in modes 0 and 1 at
  every code, at /256 too, where that half bit is 128 cycles and the loop's
  turn under 50. From SRAM the 16-bit figure is 8 cycles higher in mode 1, the
  8-bit one the same.
- **RXNE leads TXE by less than a load** (measured on the resource, letter d;
  the chapter does not order them). With a frame waiting behind the one
  shifting, frame k's end raises RXNE and moves frame k + 1 out of the buffer.
  Polled from SRAM, in all
  four modes, both widths, /8 to /256, over ten runs of 5 760 frame ends:
  a load saw RXNE with TXE still low at 5 228 of 57 600, and at every one
  of those the very next load, 2.2 cycles on, had TXE up; no load ever saw
  TXE alone. CPHA makes no difference (2 492 of 28 800 with CPHA = 0, 2 736
  of 28 800 with CPHA = 1), and the share at a rate follows the frame's
  length against the poll's 6-cycle turn and not the half bit: 16 to 18 of
  120 at /256 in 8-bit frames, whose half bit is 128 cycles. So this die has
  not the CH32V006K8U6's gap in modes 1 and 3
  ([../ch32v00x/spi.md](../ch32v00x/spi.md), "RXNE does not say TXE"). No
  loop of the host rests on this; it is what a program writing ahead with the
  resource's verbs would need to know.
- **OVR does not clear as 16.2.7 says** (measured, letter e). The chapter:
  "Reading the data register and then reading the status register will
  eliminate this bit". On this die that order leaves it standing, through
  as many loads as follow, RXNE down. What clears it is a DATAR read after
  a STATR load that showed it, the buffer full or empty, at every gap tried
  from back to back to some 200 cycles, from SRAM and from the flash. The
  register table's RW0 (16.3.3) clears nothing: a zero written leaves it
  up, and so does clearing SPE. It falls just after the read: of two loads
  back to back, DATAR then STATR, in compressed instructions, the second still reads it up and the load
  after that does not. Such a late load is itself a load that showed it, and
  in one sequence the next DATAR read then cleared a later overrun with no
  load between, so a program should not treat that read as a guarantee the
  flag was seen.
- **8 or 16 bits** (DFF), SCK = HCLK / 2^(BR+1), /2 to /256 (16.3.1), up
  to "half of FHCLK" (16.1): 24 MHz at 48, table 3-19's ceiling for both
  roles. Every bus of this series runs at HCLK, so HCLK is the reference.
- **The chapter's two mode numberings disagree** and neither is the
  Motorola one. Table 16-1 prints CPOL 0/1/1/1 and CPHA 0/0/0/1 for modes
  0..3, a table no numbering produces; figures 16-2..16-9 number the modes
  CPHA first (mode 1 = CPHA 0, CPOL 1; mode 2 = CPHA 1, CPOL 0). The
  driver's `SpiMode` is the Motorola numbering every stratum speaks, mode =
  CPOL x 2 + CPHA, written into the bits by name - and measured standard
  in all four (letter w, below).
- **Nothing is enable-protected** (measured): 16.3.1 says DFF and CRCEN
  "can only be written when SPE is 0" and that MSTR, CPOL, CPHA and
  LSBFIRST must not change "during communication", and a raw write to
  each of eight fields lands with SPE set (DFF, CRCEN, CPOL, CPHA,
  LSBFIRST, BR, MSTR, BIDIMODE). The driver's disable/enable pair is the
  only protection there is.
- **HSRXEN changes the ladder** (measured). 16.3.8 makes the bit a read
  enable for "CLK greater than or equal to 36MHz", "only valid when the
  clock is divided by 2" and "not readable"; 16.3.1's note on BR says that
  with it set "the SCK frequency is FHCLK/(BR+2)". 16.3.1 is right: with
  the bit set every code gives HCLK/(BR+2) - a second ladder /2, /3 ..
  /9 - and HSCR reads zero whatever was written. The two texts agree at
  BR = 000 alone.
- **At /2 a client's answer needs HSRXEN** (measured, and the datasheet's
  arithmetic). Table 3-19 gives a client's data valid up to 25 ns after
  its shifting edge, tV(SO), and the host 5 ns of setup, tSU(MI): 30 ns
  against the 20.8 ns half period of 24 MHz, no margin. Against the
  CH32V006's client in mode 3, without the bit a 32-frame window on the
  engines read one frame's first bit as 1 where the client sent 0 in 29
  of 70 windows on one image and 2 of 70 on its
  twin, and with the bit none in 28; what the client hears is exact
  either way.
- **The select has three arrangements** (16.2.1): software (SSM, the
  level from SSI, the pad free), hardware output (SSOE), hardware input (a
  low level raises MODF and demotes a host). SSOE drives NSS low "when the
  SPI host sends output" - measured: low from the host's first frame and
  not at SPE, held low between frames, high again only when SPE clears.
  MODF from a second chip's pulse clears MSTR and SPE (measured), and
  16.2.7's clearing sequence - a STATR access, then a CTLR1 write - takes
  once the pad is back high.
- **ODEN** (16.3.2) makes NSS, SCK and MOSI open drain - the only open
  drain this series has outside the I2C (RM 8.3.1.1 gives the GPIO none)
  - and not a client's MISO. Measured: on a wire with no pull-up SCK never
  rises, and NSS comes back through the far end's pull-up.
- **CRC** (16.2.5): two calculators over the frames sent and received,
  CRCNEXT after the last data frame, CRCERR. Measured: TXCRCR over six
  frames sent is what a bitwise CRC-8 over the same polynomial computes,
  and RXCRCR over a foreign chip's eight frames is that of the frames that
  came.
- **The DMA requests are channels 2 (receive) and 3 (transmit)** (RM
  9.2.3): the channel is the request. DATAR is sixteen bits, so a 16-bit
  frame is one half-word beat, from a half-word boundary only (9.3.6).
- **The client's restrictions, measured.** 16.2.3: "If transmitting in
  slave mode is required, CPOL needs to be set and configured as mode 2 or
  mode 3"; 16.3.1, of LSBFIRST: "LSB is only supported by SPI as host".
  Under the CH32V006's host this die receives exact in all four modes and
  both bit orders, answers exact in all four modes MSb first - the first
  restriction does not hold here, not even under the data that makes the
  CH32V006's client slip in modes 0 and 1 (a first answer bit of 1 with
  the host's MOSI low at the select's fall, measured exact in all four
  modes with a scratch letter) - and LSb first its MISO never moves, in
  every mode: the second holds, for what a client sends (letters r and w).
  16-bit frames and the CRC as a client, both ways, measured under the
  CH32V006's host (`spi_peer`, below).
- **As a client, it hears to /2 and is read to /4** (the CH32V006's
  letter s, three runs, and its ladder): under that chip's host every
  pattern is heard exact at 12 and 24 MHz - a PRBS, MISO quiet low or
  high, MISO toggling under a quiet MOSI, modes 1 and 3 - and its answers
  are read exact to 12 MHz; at 24 MHz the far host reads them wrong
  whenever MISO toggles in modes 0 and 3, the timing budget of table 3-19
  (25 ns of a client's output valid time and 5 ns of the host's setup
  against a 20.8 ns half period) not closing for a host without a
  high-speed read.
- **The pads are a column** (8.3.2.1, SPI1_RM[1:0]): 00 PA4/PA5/PA6/PA7
  (NSS, SCK, MISO, MOSI), 01 PB21/PB15/PA8/PA9, 10 PA12/PA11/PA9/PA10, 11
  PC4/PC5/PC6/PC7. The QFN20 bonds the first column alone; the CH32X033F8P6
  bonds PA7 with PB0 on one pin (table 2-1, note 7), so a host's MOSI may
  not be driven there and the host on that column does not compile, where
  a client - MISO on PA6 - does. Table 8-4: a host's MISO a floating or
  pulled-up input, a client's SCK a floating input.
- **The pads have no speed field** (RM 8.3.1.1): port A's outputs reach
  40 MHz at 50 pF (table 3-14), past the SPI's 24. ODEN is the one pad
  class an SPI has here.

### The offer, item by item

| offer (section) | taken? | why |
|---|---|---|
| the transmit buffer in front of the shifter (16.2.2): a frame written while one shifts follows it with no gap | used by the transmit-only loop, declined by the receive loop | the transmit-only loop writes on TXE and never reads, the clock never pausing. The receive loop keeps one frame in flight at every rate: two would keep the bus busy through its turn, and the one-deep receive buffer (16.2.7) would then lose a frame to any handler longer than a frame. This host owns the clock, so one in flight makes a handler or a preemption of any length a gap on the bus; the price is 8 to 44 cycles a frame over the wire ("The host above the wire") |
| DFF, the beat is the frame (16.3.1, 16.3.4) | yes | the Request's `bits`; the engines move a 16-bit request in half-word beats |
| BR, /2../256 (16.3.1) | yes | the Request's `clock`, compared as one word with the last request's |
| HSRXEN at /2 (16.3.8) | yes, as `Config::high_speed_read`, on by default | the host sets it only while /2 is in force and drops it before any other code; without it a client's answer at /2 has no timing margin (table 3-19) and the CH32V006's was read a bit wrong in up to 29 of 70 windows, with it in none of 28 (measured) |
| HSRXEN's second ladder, HCLK/(BR+2) (16.3.1) | declined | 16.3.8 restricts the mode to /2, so no request asks for it, though the ladder carried exact data at /3, /5 and /6 against the CH32V006 (measured) |
| DMA on channels 2 and 3 (RM 9.2.3) | yes, the engine slots | the asynchronous way, and the only one: at the wire at every code, one interrupt a block (the receive block's), a command phase worth a block of its own chained to its data block from the vector; a request too short to repay the engines' fixed cost runs polled inside `start()` instead (a byte read takes the engines from 15 frames up at /2, from 5 up at /16 and from 1 at /128 and slower, below); never for a request marked `polled`; no sleep while a block runs (How to use it) |
| SSOE (16.2.1) | declined as the engine's select | it holds NSS low from the first frame to the next disable, a span no Request's tenure has; `claim_nss_pad()` for a program that wants it |
| the hardware NSS input, MODF (16.2.1, 16.2.7) | the resource's | a multi-host bus has no user; measured against a second chip |
| ODEN (16.3.2) | yes, as `Config::open_drain` | the one pad class, written once |
| CRC (16.2.5) | the resource's verbs | measured both calculators; a tenure shape with a CRC is born with a device that checks one |
| BIDIMODE/BIDIOE, RXONLY (16.2.4) | the resource's configuration | a receive-only host clocks for as long as SPE stands, which no Request bounds; a client that only listens leaves its MISO pad undriven (a dark listener, below) |
| TXE, RXNE and the errors on one vector (16.2.8, table 7-1 entry 33) | declined by the host; the client's | a per-frame interrupt, some 225 to 268 cycles a frame, costs more than the frame at /16 and faster (128 cycles a byte at /16) and lengthens every other line's hold-off ([design/spi-bus.md](../design/spi-bus.md), "Two ways to run a request"); the vector serves a client's runs (`SpiClient::isr()`, letter v) |
| SRAM execution (the instruction fetch) | offered, the program's choice per host, for the paths that run once a frame | the host's placement (util/placement.hpp): under `InRam` the two polled loops run from `.fast_text.brio` - the transmit-only loop at the wire at /2; under `InFlash`, the default, they cost no SRAM, and the polled loops' functions are aligned to the fetch's word, which fixes their cost from the flash (below); the paths that run once a transaction stay in the flash under either (measured, below) |

## Types and verbs

[brio/ch32x035/spi.hpp](../../brio/ch32x035/spi.hpp), three layers:

- `Spi<1>` is the resource: `configure(SpiConfig)` (refused by
  `spi_config_valid()`: CRC outside full duplex or with a zero
  polynomial, SSOE on a client, the high-speed read off /2 or on a
  client), `enable()`, `disable()` - TXE then not BSY, bounded -,
  `data()` both ways, `rxne()`/`txe()`/`busy()`/`status()`, `flush_rx()`,
  the three errors with their clearing sequences, the CRC verbs,
  `software_select()`, `open_drain()`, `high_speed_read()` (refused off
  /2 and on a client; HSCR written whole and its one bit kept in a copy,
  `hscr_register()` the raw read), `dma_requests()`, the three interrupt
  enables and `isr()`.
- `SpiHost<1, pins, TxEngine, RxEngine, Placement>` is the engine
  `SpiBus` (= `BusMaster`) drives. Its `Request` is the other strata's
  field for field (a chip select `PinRef` and a D/C line, a command phase
  and a data phase, the per-request `mode`, `clock` and `bits`, `polled`),
  forty bytes with no padding in this host's order: the eight words an
  asynchronous tenure copies first, then the setup time and the three
  fields the host compares as one word with the last request's, then the
  reply. `init(clock,
  Config)` - `Config` the bus's SCK ceiling, `open_drain` and
  `high_speed_read`, written once - or `init(clock, max_sck_hz)`;
  `rebase()`, `clock_for(hz)`, `sck_hz()`,
  `prime()` for a caller framing its own select, `bit_order()`,
  `dma_isr()` - the one interrupt body, the two channels' -, `status()`
  (`spi_ok`, `spi_rejected`, `spi_dma_fault`, `spi_stalled`),
  `recover()`, `release()` - the block's reset pulse before its gate
  ([../design/dma.md](../design/dma.md)'s release contract) -,
  `claim_nss_pad<arrangement>()`. `has_dma` says whether engines are
  bound: without them every request not marked `polled` is refused,
  `spi_rejected` at once with the block, the select and the D/C
  untouched. `async_min_frames(code, bits, read)` is the short-request
  rule's count: an asynchronous request below it runs polled inside
  `start()` (its arithmetic below, "The short request"). A 16-bit request
  whose buffers sit at an odd address, or over engines of `uint8_t`,
  runs polled whichever way it asked: the engines cannot carry it.
  `Placement` is util/placement.hpp's
  `InFlash` (the default) or `InRam`, where the two polled loops run from;
  the cost constants (`SpiHostCosts`: E - F for a read and a write, the
  receive loop's dead bus, the two loops' turns) and the counts
  drawn from them follow it, and any other tag does not compile. The
  engine slots are `DmaTxEngine<3, Elem>` and `DmaRxEngine<2, Elem>`,
  both or neither, on those two channels alone, the receive at very_high
  and the transmit at high (armed for its errors alone).
- `SpiClient<1, pins>`: `init(clock, Config)` refusing an answering
  client LSb first (`config_served()`, `spi_client_answers()`), the
  per-frame verbs (`enable()`, `write()`, `writable()`, `poll()`,
  `selected()` read off the NSS pad, `drive_output()` for a dark
  listener), and the runs a transaction is served by - `answer()`,
  `exchange()`, `receive()`, polled on TXE and RXNE with the NSS pad as
  the window, returning how many frames moved and how the run ended -
  with `frames_ahead` = 1.

A host's MOSI and SCK, or a client's MISO, on a pad this part shares with
another does not compile; nor does a column whose pads it does not bond,
an engine on another channel, one engine of two, or a beat wider than
DATAR (`test/family_ch32x035/neg/spi_*.cpp`).

## How to use it

The engined host's requests are asynchronous unless marked `polled` (the
smallest use above). The same host's `polled` request runs inside the
call, the arbiter answering in the dispatch that received it, and under
`Tenuto` every other active object waits until it ends. An excerpt, over
the smallest use's `Bus`, `Spi` and `r`:

```cpp
r.polled = true;                                      // inside start(); the caller waits
brio::post<Spi>(r);
```

A polled request runs the explicit loops at any length: it never touches a
channel and needs no vector. A long read that wants the wire asks
asynchronously. An asynchronous request short of the
count below runs polled inside `start()` as well, so `polled = false`
does not promise that the call does not spin.

An engined transfer keeps the core awake. In this die's Sleep the DMA
moves nothing ([dma.md](dma.md)), so the kernel's `idle()` asks the
chip's `bus_masters_working()` (`qingke/platform.hpp`), here whether any
channel is working, and returns at once while one is, the kernel loop
turning. The core is free for other active objects for the block's whole
time but it is not asleep, as it is not in a polled transfer either. The
short-request count below weighs core cycles alone, and no current has
been measured here.

Engines of `uint16_t` move 16-bit frames in half-word beats (a beat is
what the channel moves at a time), from buffers on a half-word boundary. A host with no engines bound,
`brio::SpiHost<1>`, runs polled requests alone - a program that never
posts an asynchronous one needs no DMA channel and no vector - and
answers any other `spi_rejected`. The per-frame paths in SRAM, the
program's choice ([platform.md](platform.md), "A handler in SRAM"):

```cpp
using Bus = brio::SpiHost<1, brio::spi1_default_pins, brio::DmaTxEngine<3, uint8_t>,
                          brio::DmaRxEngine<2, uint8_t>, brio::InRam>;
```

A client:

```cpp
using C = brio::SpiClient<1>;
C::init(clock, {.mode = brio::SpiMode::mode3});      // NSS PA4 the select
C::enable();                                          // SPE up, the buffer empty
const auto run = C::exchange(std::span<const uint8_t>(answers), std::span<uint8_t>(heard), spins);
```

## Bench findings

The reference suite is `test_x035_spi` on the CH32X035F8U6 at 48 MHz, its
SPI1 on PA4..PA7 wired straight to the CH32V006K8U6's column 0 (PC1, PC5,
PC6, PC7). The suite is two images - whole it came within 5 % of the
62 KB every part has - `test_x035_spi-1` (the wireless letters) and `-2`
(the two-board letters and the slip statistics). `z` on the first - the
block, every way a request runs, the ladder and the costs, the refusals,
OVR's clear, the fault exits - runs with the far board's client
deselected: 39 verdicts, three runs green, letters `a`, `b`, `e` and `f` ten
each. `y` on the second against the CH32V006 running `spi_peer` on its
crystal, commanded in band through spi_link: 44 verdicts, three runs green,
its handler letter `h` fifteen, its ladder letter `p` forty. Letter `d`, run by
its own key on the first image and not in `z`, prints a table and judges nothing: the witness of
what the silicon does at a frame's end, above.

- **The reset values are table 16-2's** (STATR 0x0002, CRCR 0x0007, the
  rest zero), a control word lands as spelled, and eight of eight CTLR1
  fields take a write under SPE.
- **The engines run at the rate asked**: a frame inside an engined block,
  the difference of a 256- and a 32-frame block with the select high, is
  1003 to 1011 thousandths of its SCK periods at /2, 998 to 1001 at every
  code from /4 to /256, and 1000 to 1001 in 16-bit frames at /4, 999 to 1000
  at /16.
- **HSRXEN's rate**: 16 017 to 16 196, 23 946 to 24 089, 32 000 to 32 017,
  39 982 to 40 035, 47 964 to 48 035, 56 000 to 56 035, 64 017 to 64 089
  and 72 000 to 72 035 thousandths of a cycle an 8-bit frame at BR 0..7 -
  8 x (BR + 2) cycles at every code, which meets the usual ladder of
  powers of two at BR 0 alone.
- **The waveforms, this chip as the analyser** (letter w: PA4..PA7
  sampled through INDR every 25 cycles at /256, each data change filed
  under the SCK edge before it, the line read at the standard sampling
  edge, this host's frame written from inside the sampling loop - a frame
  written before it began had its first SCK edge in before the first
  sample in some runs, and the line's idle level unread): this host's SCK
  idles at CPOL and its MOSI changes on the
  standard edge and carries the frame in all four modes, alone and inside
  a block back to back; the CH32V006's host is the same in all four; this
  chip as a client drives its answer on the standard edge in all four MSb
  first, and LSb first its MISO never moves.
- **A mode change reaches SCK at the CTLR1 write** (a scratch letter,
  every ordered pair of modes three times at /256 and at /2, the pad read
  through INDR): with SPE cleared the pad holds the old idle level; the
  word carrying the new CPOL, written with SPE still clear, moves it, and
  it stands there 50 us later with SPE up and no frame sent. The host's
  `start()` and `prime()` write that word before the select falls, so a
  window opens on the new idle level and a selected client sees no extra
  edge.
- **Against a foreign client** (letter o, the CH32V006): all four modes,
  both bit orders, on the peer's DMA serve and on its polled run - eight
  frames heard exact by the client every time, and its answers read exact
  by this host in modes 2 and 3, the modes that client answers in (CPOL =
  1, its RM 16.2.3; in modes 0 and 1 it listens alone - its document,
  "THE ANSWERING CLIENT NEEDS CPOL = 1"); a bit-order mismatch an exact
  two-way reversal. And two of the
  client's frames clocked with nothing read leave OVR up and the first frame
  in the buffer, the third read next exact, 8 of 8 (the frame a loss keeps).
- **The ladder against the peer** (letter p, 43 runs): the polled host and
  the engines exact at every code to /2 = 24 MHz against the peer's DMA
  serve, /2 with the high-speed read as the driver's default has it, in
  every run; the same window on the engines with the bit off, printed and
  not judged, read one frame wrong at this end in 3 of the 43, the client's
  own hearing exact in all of them. The peer's polled run is
  its own ceiling, exact to /8 behind both the polled host and the
  engines' back-to-back frames. HSRXEN's /3, /5 and /6 (16, 9.6 and 8
  MHz) exact on the engines.
- **The CH32V006 client's runs** (letter k: its driver's answer, exchange
  and receive runs, 32 frames of 8 and of 16 bits, under this end's
  polled loop - a gap after every frame - and back to back under the
  engines): every run exact to 6 MHz under both hosts at both widths, and above
  that the runs' own ceilings from that chip's flash - the exchange to 12
  MHz in 16-bit frames, the answer and receive runs to 12 MHz back to back
  and 24 MHz gapped in 8-bit frames and to 24 MHz in 16-bit ones; the
  cost and the cause of each in [../ch32v00x/spi.md](../ch32v00x/spi.md),
  "The client's runs". Those ceilings are the far client's code and not the
  wire's: its receive run, from SRAM, hears 24 MHz back to back.
- **The kernel over a second chip** (letter q): four transactions queued
  in one dispatch, one polled among three on the engines - one of them a
  command block of four frames chained from the vector to its data block
  of four - in one select window the letter holds open itself: four `spi_ok`, every byte read back by
  the far chip, ten runs of ten.
- **Every way a request runs** (letter a, wireless): the two polled
  loops; a polled request of 80 frames on the engined host, the loop and no
  channel interrupt; the engines asynchronous; a command phase of 32
  frames on a block of its own, chained to its 16-frame data block from
  the vector - two channel interrupts - and a command-only request on
  one; an asynchronous read one frame under the count `async_min_frames()`
  gives at /4 completing inside `start()` and one at the count going to
  the engines. And the refusal (letter c): an asynchronous request on the
  engineless host answered `spi_rejected` at once, the select never
  falling, CTLR1 the word it was and SCK at its idle level, where the
  same request marked `polled` runs.
- **The SPI stack under the preemptive kernel** (`test_x035_rubato_link`
  key `z`, 120 verdicts, on each scheduler placement, the CH32V006's
  `spi_peer` its client, every data window in a CPOL = 1 mode its client
  answers in): the command channel, the matrix (modes 2 and 3, both
  widths, both bit orders), the ladders (the polled loop, a five-frame
  asynchronous request - on the engines at /16 and slower and polled
  inside `start()` at /8 and faster, each rung's way read at the DMA vector and
  judged against the host's count - and the engines, from /256 to /2),
  the arbiter under pressure (three requesters in one select window in
  mode 3: one on the engines, one whose one-frame asynchronous
  requests run polled inside `start()`, one polled) and the dark
  letter, every object idle and nothing armed at the end: green in three runs of `z` on each scheduler placement, and the
  ladders in fifteen runs more and the arbiter letter in three on both, every
  pass and pressure. The polled loop keeps one frame in flight, so a preemption
  longer than a frame is a gap on the bus and the bound on lost frames is zero:
  2016 polled rungs on each placement in the ladders' eighteen runs, none not exact, the 3000-cycle preemption outlasting a frame at /64 and faster.
- **This chip as client** (letter r, the peer's `host_burst` at 1.5 MHz,
  20 us between frames): received exact in all eight mode-order
  combinations; answered exact in the four MSb-first ones, and LSb first
  the far host read 0xFF or 0x00 - a line that does not move - in all
  four.
- **MODF from a second chip** (letter m): the peer's PC1 held low 2 ms
  demotes a host whose NSS is a hardware input 24 ms after the enable,
  MSTR and SPE cleared, four of four; STATR 0x22 before the clearing
  sequence, 0x02 after it, once the pad has been high a millisecond.
- **OVR and its clear on the resource** (letter e, wireless, ten runs): two
  frames clocked with nothing read raise OVR with RXNE, and what clears it is
  above (What the silicon does); `flush_rx()` leaves neither flag.
- **A handler longer than a frame is a gap on the bus** (letter h, against
  the peer): the tick's handler dragged by an empty loop to hold main context
  off 2244 to 2288 cycles, past four frames of 512, beside ten windows of 128 frames
  at /64: ten of ten exact both ways, `spi_ok`, the peer's count right, OVR never
  standing, in eighteen runs of the letter. A window lengthens by 1758 to 1822 cycles
  (71 411 to 71 459 undragged, 73 215 to 73 239 dragged, one tick landing),
  where the arithmetic above the letter allows some 1100 to 4600, computed from
  each run's figures.
- **The fetch** (letter b): the same bare transmit loop takes 20.0 cycles
  a frame at /2 from the flash and 16.0, the wire, from SRAM. From the
  flash a loop's cost is a function of its address modulo four, the
  fetch's word: the host's polled functions, the same instructions moved
  half a word at a time through 8 bytes, took at offsets 0 and 4 47.8 to
  48.0 cycles a frame at /2, 47.9 at /4 and 148.0 to 148.4 at /16 for the
  receive loop and 24.0 at /2 for the transmit-only loop, and at
  offsets 2 and 6 43.8 to 44.0, 67.9 to 68.0, 171.8 to 172.2 and 28.0: two
  values, nothing between them, half a word saving 4 cycles on the receive
  turn at /2 and costing 20 at /4, 24 at /16 and 4 on the transmit-only loop.
  Code of four taken forward branches showed the same period (68 and 72
  cycles, through 32 bytes), straight-line code and three-instruction loops
  none, and SRAM none within a cycle ([platform.md](platform.md), "A handler
  in SRAM"). A function under -Os with the compressed instructions is aligned
  to two bytes only, so the image's layout would pick the cost, and
  `-falign-loops` does nothing in a function optimized for size. The two
  polled functions are therefore `aligned(4)` in the flash, under `InFlash`:
  two bytes of padding at most, the cost the code's own. The host also offers the program its per-frame
  paths in SRAM, `InRam`, and the table below gives both placements. From
  SRAM the address modulo four decides too, by less: with the loops moved
  through 38 bytes, the receive loop's frame took 24.0 cycles at /2, 40.0
  at /4 and 140.0 at /16 at one of the two, 26.0, 42.0 and 141.0 at the
  other, nothing between them, and the transmit-only loop 16.0 at both. The
  SRAM instances are not aligned, so the image's layout picks the value
  (the gap list).

### This part as the desk's instrument: `spi_peer`

`spi_peer` makes the CH32X035F8U6 the desk's SPI client for the
CH32V006's suite (`test_ch32_spi`, [../ch32v00x/spi.md](../ch32v00x/spi.md)):
spi_link's command channel in band over SPI1's column 0 with the hardware
select on PA4 (pulled up), the dark listener, its answer windows in mode
0, the exchange served on the DMA engines (channels 2 and 3, polled with
no interrupt, the window read off the NSS pad) or on the client's runs,
the extended exchange whole - 16-bit frames (the engines in half-word
beats, the runs over `uint16_t`), the CRC on a polled run (CRCNEXT set
the instant the last answer is in the buffer, the host's CRC frame taken
as one more frame), a receive-only run with MISO undriven -, `sink_slow`,
`ss_pulse` on PA4 as a GPIO, `host_burst` with PA4 the host's GPIO select
(16-bit frames too); an answering exchange LSb first refused, as
`SpiClient` refuses it, and `mspi` refused (no USART of this package
reaches the wires). The voltage detector at 3.0 V gates the link pads at
boot; PB3 is never touched. What the far chip measured of this client:

- **The hardware select frames it**: every exchange the CH32V006 host
  clocked, in all four modes MSb first, on both serves, exact both ways;
  LSb first the client hears exact as a listener and the far host's LSb
  against its MSb is an exact two-way reversal. The far host's MODF
  letter took its select low four times on command.
- **16-bit frames and the CRC**: sixteen 16-bit frames from the far host
  in every mode and 32 on its engines exact both ways; CRC-8
  (0x07) and CRC-16 (0x8005) over eight frames - this client's TXCRCR
  frame the bitwise CRC of its stream, its check of the host's CRC frame
  clear - and on two polynomials its CRCERR set.
- **The rate**: the DMA serve exact both ways at every code to 12 MHz
  behind the far host's polled loop and its engines, and at 24 MHz heard
  exact while the far host reads its answers wrong (What the silicon
  does: the read's timing budget); the receive engine alone, MISO
  undriven, to 24 MHz. The client's polled run - each
  answer reloaded on the TXE of the frame before it by code fetched from
  the flash - to 6 MHz behind a host with gaps between frames and to 3
  MHz behind frames back to back (three runs of the far chip's ladder).
- **As a host** (`host_burst`, one polled frame a request, 20 us apart, 1.5
  MHz): the CH32V006's client on its hardware NSS, answering in modes 2
  and 3 (the modes its driver admits an answering client in), read twelve
  frames exact MSb and LSb first, whatever its first answer bit, and eight
  16-bit frames in mode 3, and this host read its answers exact.

### The host above the wire

What a transaction costs beyond its frames' time, measured by letter b at
48 MHz with the select high, in one image for both placements - the
suite's hosts `InRam`, their twins `InFlash` - each line the best of
three or five, the console quiet, the figures the range over ten runs;
a line going out beside a measurement lengthens it, a 256-frame block at /2
lasting about what a console character does at 115200, and the engines' frame
at /2 read up to 1045 thousandths that way; `fixed` from a
32-frame transaction less 32 frames, the suite's call - a request built,
the call, the wait on the completion - inside it. The call's own cycles
move with where its code sits (a zero-length request through the two
twins' copies of one call takes 436 to 440 and 420 cycles), so a fixed cost is
a figure of one image. The frame costs are not, bar the receive turn's
two values from SRAM (above):

| path | a frame, SRAM / flash | fixed, SRAM / flash | the wire a frame |
|---|---|---|---|
| the engines | the wire (1003 to 1011 thousandths at /2, 998 to 1001 from /4 to /256) | 1320 to 1360 / 1428 to 1440 | 16 at /2, 32 at /4 |
| the receive loop | 23.9 to 24.0 / 47.9 at /2, 39.9 to 40.0 / 47.9 to 48.0 at /4, 139.9 to 140.1 / 147.9 to 148.2 at /16, 524.0 to 524.6 / 532.0 to 532.4 at /64 | 701 to 718 / 720 to 728 at /4 | 16, 32, 128, 512 |
| the transmit-only loop | 16.0 / 23.9 to 24.0 at /2, 31.9 to 32.0 / 32.0 at /4, 64.0 / 63.9 to 64.1 at /8, 127.9 to 128.0 / 127.9 to 128.0 at /16 | - | 16, 32, 64, 128 |
| a 3-frame polled request (a command and two data frames) | 1213 to 1215 / 1216 to 1232 in all at /16 | - | 384 |

- **The transmit-only loop** is seven instructions a frame with one taken
  branch, specialized on whether the out buffer exists (the dummy an
  immediate): from SRAM at the wire at every code; from the flash at the
  wire at /4 and slower, its frame at /2 24.0, the fetch's bound (28.0 when the function sat half a word off, above).
- **The receive loop** keeps one frame in flight at every rate: the poll's
  last turn, the read and the write are the bus's dead time, and the table
  below is what that costs, `spi.frame` of letter b (best of five, interrupts
  masked, every code, 8 and 16 bits, modes 0 and 1, thirteen runs within two
  cycles):

  | the frame over the wire | /2 | /4 | /8 | /16 | /32 | /64 | /128 | /256 |
  |---|---|---|---|---|---|---|---|---|
  | flash, 8-bit, modes 0 and 1 | 32 | 16 | 32 | 20 | 36 | 20 | 36 | 20 |
  | flash, 16-bit, modes 0 and 1 | 40 | 32 | 44 | 36 | 44 | 36 | 44 | 36 |
  | SRAM, 8-bit, modes 0 and 1 | 8 | 8 | 12 | 12 | 12 | 12 | 12 | 12 |
  | SRAM, 16-bit, mode 0 | 14 | 10 | 10 | 10 | 10 | 10 | 10 | 10 |
  | SRAM, 16-bit, mode 1 | 14 | 18 | 18 | 18 | 18 | 18 | 18 | 18 |

  The figure does not grow with the frame, so at a slow rate it is a small
  share of the wire: 20 cycles on 512 at /64 from the flash. At /2 and /4 it is
  what limits the rate: from the flash the loop's own turn, 48 cycles, is the
  frame at both. A read that wants the wire asks asynchronously. A polled read
  is short by necessity: the rule below sends a longer asynchronous one to the
  engines.
- **What `InRam` costs in SRAM** (the map, `.fast_text.brio`): 740 bytes a
  host type - eight instances, the two widths with and without an out
  buffer of the transmit-only loop and the receive loop, of 76 to 110
  bytes each; under `InFlash`, nothing. Two host types on one
  instance (this suite's plain and engined hosts) carry two sets of loops.

### The short request

An asynchronous request rides the engines only where they cost the core
less than running it polled; a shorter one runs polled inside `start()`
(design/spi-bus.md's rule). On the engines a transaction costs the core
a fixed E whatever its length - `start()`, the launch, the one
interrupt, the completion - and the wire is the core's own; polled it
costs a fixed part F and each frame: the wire's frame or the loop's
turn, the longer, the receive loop's dead bus with it. So a request
rides the engines from the smallest count N with F + N x frame >= E,
`async_min_frames()`. Only E - F enters, and letter b measures it as one
difference - the engines' 32-frame block and a polled line's fixed part
through the same call of the suite, so the call's own cycles cancel -
on the engined host of each placement, one for a read and one for a
write, whose loop's fixed part is not the receive loop's:

| figure (`SpiHostCosts`) | measured over ten runs, SRAM / flash | the constant, SRAM / flash |
|---|---|---|
| E - F, a read | 581 to 624 / 700 to 712 | 595 / 705 |
| E - F, a write | 584 to 628 / 732 to 744 | 590 / 740 |
| the receive loop's dead bus a frame at /4 | 7.9 to 8.0 / 15.9 to 16.0 | 8 / 16 |
| the receive loop's turn at /2 | 24.0 / 47.9 to 48.5 | 24 / 48 |
| the transmit-only loop's turn at /2 | 16.0 / 24.0 | 16 / 24 |

From the flash, the default placement, a read of byte frames rides the
engines from 15 frames at /2 and /4, 9 at /8, 5 at /16, 3 at /32, 2 at
/64 and 1 at /128 and slower; a write from 31, 24, 12, 6, 3 and 2 frames at /2 to
/64 (the static_asserts at the end of spi.hpp). The host folds a request's cost from two frame
figures it keeps for the code in force, computed when the code changes:
a multiply for each phase and a compare in `start()`, on the
asynchronous path alone. Letter b also finds the edge itself - the
fewest frames whose polled line costs the engines' measured fixed cost
- at each of the three shapes: within a frame of the arithmetic on that
run's own figures in every run, and within three of the host's
count (from the flash, 15 against 15 read at /2 and /4, 31 against 31 written
at /2; from SRAM, 16 or 17 against 15 read at /4, 26 to 28 against 25 read at
/2, 37 to 40 against 37 written at /2). The count need not be closer: a frame either side of the edge is
some 50 cycles either way at /2, against the thousand and more of
either path.

The figures are the default image's: from SRAM the receive turn takes
one of its two values by where the image puts it, one host type's copy
and the next's apart in one image, and the counts drawn from it move with
it (the gap list).

- **The select and D/C edges are inline**: `PinRef`'s level verbs are
  always inline (pin.hpp), so they add no call to a transaction's path.
- **The fault exits, provoked** (letter f, the select high): a polled
  duplex request of 16 at /16 with SPI1 held in reset ends `spi_stalled`
  after about 58 500 cycles - the poll budget, some 28 times the wire's
  2048 - and `recover()` puts the next request back to `spi_ok`. TIM1's
  compare request held on DMA channel 2 (its channel 1's row) leaves the
  receive channel dead - the next owner takes one stray item ([dma.md](dma.md))
  - and a polled request of 64 on the engined host over it runs its loop to
  `spi_ok`, no channel touched and no channel interrupt: a polled request
  depends on neither. The asynchronous engines over the same channel never
  claim a completion (the suite's bound answers them); with TIM1 reset and
  `recover()` run, both paths complete `spi_ok`.
- **The block's own transmit request outlives its gate while SPE stands**
  (letter f): SPE up as a host with nothing written - TXE standing - and
  TXDMAEN set with no channel enabled; the next owner of channel 3 then
  moved 4 items with the block live, 1 with its gate closed alone, none
  with SPE cleared before the gate, none after `release()` - which clears
  SPE and pulses the reset line before the gate.
- **The client from its vector** (letter v): letter r's bursts, the peer's
  host at 1.5 MHz, served by spi1's vector - RXNEIE and TXEIE, the body
  `SpiClient::isr()`, the glue writing the next answer on TXE and taking
  the frame on RXNE - exact both ways in every combination the client
  admits (modes 0 to 3, MSb first): 25 entries for 12 frames (one a flag,
  the last TXE turning its enable off), no entry finding nothing, the
  longest body some 200 cycles with the suite's stamps and owner switch
  inside it.
- **The /2 frame and the core's poll** (letter s, by name): the engined
  frame at /2 read 998 to 1001 thousandths of the wire in 100 of 100
  measurements with the completion flag polled in SRAM, and 995 to 1004 in
  100 of 100 with the core waiting off the bus (`delay_us` on its own STK,
  the block's end stamped by the DMA vector) - the core's SRAM poll takes
  nothing measurable from the channels. (Sleeping through the block is no
  instrument here: in this die's Sleep the DMA moves nothing.)
- **Against the EVT** (counted, not run): the `2Lines_FullDuplex` host
  loop over WCH's library, compiled with this project's compiler at -Os,
  is 22 instructions and four calls a frame (`SPI_I2S_GetFlagStatus`
  twice, `SendData`, `ReceiveData`), about 34 instructions with the
  callees' - at the three to four cycles an instruction this core's loops
  measure from the flash, some 100 to 140 cycles, three to four times the
  wire at /4, where brio's receive loop is 1.5 times it from the flash and 1.25 from SRAM. Its DMA example
  arms the transmit channel at VeryHigh and the receive one at High, the
  opposite of [the priority rule](../design/dma.md).

## Not covered yet

Driver gaps, each with its reason:

- HSRXEN's second ladder, HCLK/(BR+2), as rates a Request can ask: 16.3.8
  restricts the mode to /2, the driver keeps the powers of two; measured
  exact at /3, /5 and /6 against one client, outside what 16.3.8 allows.
- `start()` and the engines' launch under `InRam`: declined - kept in
  the flash under either placement, since they run once a transaction.
  Measured in a variant that placed them beside the per-frame paths:
  some 230 cycles off a 3-frame polled request, 176 off the receive
  loop's fixed cost and 118 off the engines', for some 1.4 KB more SRAM
  a host with engines (`start()` alone 726 bytes) - a cost a
  transaction, where the placed paths' is a cost a frame; a placement of
  their own waits for a program that needs those cycles.
- The short request's E counts the host's work and not the arbiter's:
  an asynchronous completion adds a `TransferDone` posted from the
  vector and one dispatch of the arbiter, which a request answered
  inside `start()` does not, so near the count the engines cost a
  program more than the arithmetic says and the edge sits a few frames
  later than `async_min_frames()`. Measuring it wants a letter timing one
  request through the arbiter both ways; the rule needs the order of
  magnitude only, which the count has.
- The simplex modes and the hardware select as the engine's: no user; the
  resource has the bits.
- The CRC as part of a Request: born with a device that checks one.
- A second host on the bus (MODF as an event the engine reports): no
  multi-host user; the resource has the flag and its clear.
- **The receive turn's two values from SRAM**: `InRam` leaves the loops'
  instances aligned to two bytes, so the image's layout picks between 24.0
  and 26.0 cycles a frame at /2 (40.0 and 42.0 at /4), and the `InRam`
  constants are the lower pair, the default image's. Aligning the instances
  would fix the value; which one, and what it does to the other users of
  `InRam`, is not measured.

Implemented but not bench-verified, each with what would measure it:

- **The last 3 to 11 thousandths of the engines' frame at /2**: letter b
  reads 1003 to 1011 with the console quiet, and the same rung measured
  5 000 times over in a scratch letter 998 to 1002, so the engines hold the
  wire and something in the letter's first reading does not cancel. Printing
  the two blocks' timings of each run of the letter would find it.
- **`spi_dma_fault`**: only a transfer error on a channel answers it, and no
  letter makes one; a block pointed at an address the bus refuses would.
- **This host's receive in mode 1 against an answering client**: the
  desk's CH32V006 client answers with CPOL = 1 only, so letter o reads it
  in modes 2 and 3, and this chip's own client answers only under a host
  clocking a frame per request; this host's MOSI in mode 1 is the
  standard's (letter w). A client that answers in mode 1 at a frame per
  select - this chip as the peer of a third host - would read it.
- **The other columns and parts**: the other three columns compile on
  every part where they are bonded (`brio check ch32x035`); a board of
  each package would run them.
