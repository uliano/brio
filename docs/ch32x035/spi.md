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
asserts the request's chip select, runs the command and data phases
and replies with an event when the last frame is out
([design/spi-bus.md](../design/spi-bus.md)).

The smallest use, a command and 64 frames of pixels to a display:

```cpp
using Bus = brio::SpiHost<1>;                         // PA5 SCK, PA7 MOSI, PA6 MISO
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
using Cs = brio::Pin<'A', 4>;

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

BRIO_CH32_VECTOR(spi1_handler) {
    if (Bus::isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

`Bus` is the engine over SPI1 on its default column, the one the QFN20
bonds; `Spi` the arbiter, an active object listed in the program's
`Tenuto` with four requests of queue; `Cs` a plain pin the request
carries as its chip select, driven high before anything else so the
display stays deselected; `init()` takes the clock task's tag
([clock.md](clock.md)) and a ceiling for the clock, under which every
request's rate is clamped; the request lends its two buffers
(`brio::lend`) until the `SpiDone` event reaches `Display`, so `cmd`
and `pixels` must stay untouched and in scope until then; the
`BRIO_CH32_VECTOR` binding is the instance's interrupt handler, an
`extern "C"` function as in C under the crt's name
([platform.md](platform.md)), posting the completion to the arbiter.
What decides correctness even there: the clock task first (the ceiling
and the chip-select setup time are computed from the rate the clock
type claims); the vector bound, since a request that is not marked
`polled` completes on the interrupt; the select pin driven high before
the first request is posted, the arbiter asserting it low around a
transaction and leaving it high after; and the mode and the rate the
device wants, which travel in every request and are checked by nothing
but the device.

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
  before it stands unread is lost, OVR set (16.2.7), the flag cleared by
  a DATAR read followed by a STATR read.
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
  restriction does not hold here - and LSb first its MISO never moves, in
  every mode: the second holds, for what a client sends (letters r and w).
  16-bit frames and the CRC as a client, both ways, measured under the
  CH32V006's host (`spi_peer`, below).
- **As a client, its rate depends on its own MISO pad** (measured, cause
  not isolated): under the CH32V006's host every pattern is exact both
  ways to 6 MHz; at 12 and 24 MHz the client loses SCK edges whenever PA6
  stands high - answering 0xFF it heard none of sixteen frames, answering
  0x00 all sixteen at both rates, and listening with PA6 held high as a
  GPIO none at 12 MHz where held low it heard all at 24. Table 3-19's
  24 MHz for a client is reached with that pad low.
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
| the transmit buffer in front of the shifter (16.2.2) | yes | the transmit-only loop writes on TXE and never reads, the clock never pausing; the pump and the receive loop write ahead as far as the one-deep receive buffer allows under the image's hold-off |
| two frames in flight where a frame outlasts the hold-off | yes | the pump and the polled receive loop alike: from /64 on 8-bit frames and /32 on 16-bit ones under the default hold-off, at the wire there (measured) |
| DFF, the beat is the frame (16.3.1, 16.3.4) | yes | the Request's `bits`; the engines move a 16-bit request in half-word beats |
| BR, /2../256 (16.3.1) | yes | the Request's `clock`, compared as one word with the last request's |
| HSRXEN at /2 (16.3.8) | yes, as `Config::high_speed_read`, off by default | the host sets it only while /2 is in force and drops it before any other code; /2 = 24 MHz is exact against the CH32V006 with and without it on these wires (measured) |
| HSRXEN's second ladder, HCLK/(BR+2) (16.3.1) | declined | 16.3.8 restricts the mode to /2; the ladder carried exact data at /3, /5 and /6 against the CH32V006 (measured), and a finer rate is a decision for the user (the report) |
| DMA on channels 2 and 3 (RM 9.2.3) | yes, the engine slots | at the wire at every code; one interrupt a transaction, the receive block's; every data phase rides them when bound, from one frame |
| SSOE (16.2.1) | declined as the engine's select | it holds NSS low from the first frame to the next disable, a span no Request's tenure has; `claim_nss_pad()` for a program that wants it |
| the hardware NSS input, MODF (16.2.1, 16.2.7) | the resource's | a multi-host bus has no user; measured against a second chip |
| ODEN (16.3.2) | yes, as `Config::open_drain` | the one pad class, written once |
| CRC (16.2.5) | the resource's verbs | measured both calculators; a tenure shape with a CRC is born with a device that checks one |
| BIDIMODE/BIDIOE, RXONLY (16.2.4) | the resource's configuration | a receive-only host clocks for as long as SPE stands, which no Request bounds; the client's dark listener is an undriven pad |
| TXE, RXNE and the errors on one vector (16.2.8, table 7-1 entry 33) | RXNE alone | the pump's edge is the frame that came back, the witness that the bus is idle for the next |
| SRAM execution (the instruction fetch) | offered, the program's choice per host, for the paths that run once a frame | the host's placement (util/placement.hpp): under `InRam` the pump's entry and both polled loops run from `.fast_text.brio` - the transmit-only loop at the wire at /2 and the pump's frame a sixth shorter behind a vector in the flash; under `InFlash`, the default, they cost no SRAM, and the two polled loops' functions are aligned to the fetch's word, which fixes their cost from the flash (below); the paths that run once a transaction stay in the flash under either (measured, below) |

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
- `SpiHost<1, pins, TxEngine, RxEngine, hold_off_cycles, Placement>` is the engine
  `SpiBus` (= `BusMaster`) drives. Its `Request` is the other strata's
  field for field (a chip select `PinRef` and a D/C line, a command phase
  and a data phase, the per-request `mode`, `clock` and `bits`, `polled`),
  forty bytes with no padding in this host's order: the eight words an
  asynchronous tenure copies first, then the setup time and the three
  fields `apply()` compares as one word, then the reply. `init(clock,
  Config)` - `Config` the bus's SCK ceiling, `open_drain` and
  `high_speed_read`, written once - or `init(clock, max_sck_hz)`;
  `rebase()`, `clock_for(hz)`, `sck_hz()`, `write_ahead_from(bits)`,
  `prime()` for a caller framing its own select, `bit_order()`, `isr()`,
  `dma_isr()`, `status()` (`spi_ok`, `spi_dma_fault`, `spi_overrun`,
  `spi_stalled`), `recover()`, `release()` - the block's reset pulse
  before its gate ([../design/dma.md](../design/dma.md)'s release
  contract) -, `claim_nss_pad<arrangement>()`. `hold_off_cycles` is the
  image's longest window the host's vector waits behind, 380 by default
  (the one safe with every driver in the flash), zero refused.
  `Placement` is util/placement.hpp's `InFlash` (the default) or `InRam`,
  where the pump's entry and the polled loops run from; the cost
  constants (`pump_frame_cycles`, `polled_gap_cycles` and the fixed
  ones) and the two thresholds drawn from them follow it, and any other
  tag does not compile. The engine slots are `DmaTxEngine<3, Elem>` and
  `DmaRxEngine<2, Elem>`, both or neither, on those two channels alone,
  the receive at very_high and the transmit at high (armed for its errors
  alone).
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

```cpp
using Bus = brio::SpiHost<1>;                         // PA5 SCK, PA7 MOSI, PA6 MISO
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
using Cs = brio::Pin<'A', 4>;

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

BRIO_CH32_VECTOR(spi1_handler) {
    if (Bus::isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

With the engines, `SpiHost<1, spi1_default_pins, DmaTxEngine<3,
uint16_t>, DmaRxEngine<2, uint16_t>>` and `dma1_channel2_handler` /
`dma1_channel3_handler` both calling `Bus::dma_isr()`. The per-frame
paths in SRAM, the program's choice ([platform.md](platform.md), "A
handler in SRAM"):

```cpp
using Bus = brio::SpiHost<1, brio::spi1_default_pins, brio::NoDmaEngine, brio::NoDmaEngine,
                          brio::spi_default_hold_off_cycles, brio::InRam>;
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
block, the ladder and the costs, the refusals, the fault exits - runs with
the far board's client deselected: 34 verdicts. `y` on the second against
the CH32V006 running `spi_peer` on its crystal, commanded in band through
spi_link: 39 verdicts. Each was green whole on the final images.

- **The reset values are table 16-2's** (STATR 0x0002, CRCR 0x0007, the
  rest zero), a control word lands as spelled, and eight of eight CTLR1
  fields take a write under SPE.
- **The engines run at the rate asked**: a frame inside an engined block,
  the difference of a 256- and a 32-frame block with the select high, is
  998 to 1012 thousandths of its SCK periods at every code /2 to /256, and
  1000 in 16-bit frames at /4 and /16.
- **HSRXEN's rate**: 16 178, 24 071, 32 089, 39 964, 48 017, 56 035,
  64 017 and 71 946 thousandths of a cycle an 8-bit frame at BR 0..7 -
  8 x (BR + 2) cycles at every code, the powers of two at BR 0 alone.
- **The waveforms, this chip as the analyser** (letter w: PA4..PA7
  sampled through INDR every 25 cycles at /256, each data change filed
  under the SCK edge before it, the line read at the standard sampling
  edge): this host's SCK idles at CPOL and its MOSI changes on the
  standard edge and carries the frame in all four modes, alone and inside
  a block back to back; the CH32V006's host is the same in all four; this
  chip as a client drives its answer on the standard edge in all four MSb
  first, and LSb first its MISO never moves.
- **Against a foreign client** (letter o, the CH32V006): modes 0 and 3,
  both bit orders, on the peer's DMA serve and on its polled run - eight
  frames heard exact by the client and its answers read exact by this
  host, every time; a bit-order mismatch an exact two-way reversal. Modes
  1 and 2 fail both ways, the CH32V006 client's doing (its own document:
  it samples on the rising edge in every mode).
- **The ladder against the peer** (letter p): the polled host and the
  engines exact at every code to /2 = 24 MHz against the peer's DMA
  serve, /2 with the high-speed read and without; the peer's polled run is
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
  "The client's runs". This direction has no 6 MHz ceiling of the wire:
  the far client's receive run, from SRAM, hears 24 MHz back to back.
- **The kernel over a second chip** (letter q): four transactions queued
  in one dispatch, one polled among three pumped, in one select window -
  four `spi_ok`, every byte read back by the far chip.
- **This chip as client** (letter r, the peer's `host_burst` at 1.5 MHz,
  20 us between frames): received exact in all eight mode-order
  combinations; answered exact in the four MSb-first ones, and LSb first
  the far host read 0xFF or 0x00 - a line that does not move - in all
  four.
- **MODF from a second chip** (letter m): the peer's PC1 held low 2 ms
  demotes a host whose NSS is a hardware input 24 ms after the enable,
  MSTR and SPE cleared, four of four; STATR 0x22 before the clearing
  sequence, 0x02 after it, once the pad has been high a millisecond.
- **The fetch** (letter b): the same bare transmit loop takes 20.8 cycles
  a frame at /2 from the flash and 16.7 - the wire - from SRAM. From the
  flash a loop's cost is a function of its address modulo four, the
  fetch's word, and of no other bit of it: the host's polled functions,
  the same instructions moved through 32 bytes in steps of two, took at
  every offset 0 mod 4 51.9 to 52.0 cycles a frame at /2 and /4 and 152.0
  at /16 for the receive loop and 24.0 at /2 for the transmit-only loop,
  and at every offset 2 mod 4 47.9, 72.0, 176.0 and 28.0 - two values,
  nothing between them. Code of four taken forward branches showed the
  same period (68 and 72 cycles), straight-line code and three-instruction
  loops none, and SRAM none within a cycle ([platform.md](platform.md), "A
  handler in SRAM"). A function under -Os with the compressed
  instructions is aligned to two bytes only, so the image's layout picked
  the cost - a change elsewhere that moved these functions by 30 bytes
  turned the receive loop's 52.0 at /4 into 72.0 - and `-falign-loops`
  does nothing in a function optimized for size. The two polled
  functions are therefore `aligned(4)`: two bytes of padding at most, the
  cost the code's own. The host also offers the program its per-frame
  paths in SRAM - the pump's entry and both polled loops, `InRam` - and
  the table below gives both placements, the suite's own vector in the
  flash.
- **The pump's placement** (letter b at /2, where the handler bounds the
  bus, the suite's vector with its stamps and its owner switch): 256.5
  cycles a frame with the body inline in the vector in the flash
  (`InFlash`), 224.5 with it in SRAM behind that vector (`InRam`); the
  flash body out of line, a call from the vector, took 300.5 - the reason
  `InFlash` is inline (util/placement.hpp). A program that places the
  vector itself (`.fast_text`) runs an `InFlash` body from SRAM with it,
  no call at all; measured on the I2C host's pump, where it is the
  fastest of the three ([i2c.md](i2c.md)).

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
- **16-bit frames and the CRC**: sixteen 16-bit frames through the far
  host's pump in every mode and 32 on its engines exact both ways; CRC-8
  (0x07) and CRC-16 (0x8005) over eight frames - this client's TXCRCR
  frame the bitwise CRC of its stream, its check of the host's CRC frame
  clear - and on two polynomials its CRCERR set.
- **The rate**: the DMA serve exact both ways at every code to 6 MHz
  behind the far host's polled loop and its engines, and the receive
  engine alone, MISO undriven, to 12 MHz - the client's MISO pad decides
  above that (What the silicon does). The client's polled run - each
  answer reloaded on the TXE of the frame before it by code fetched from
  the flash - to 3 MHz behind a host with gaps between frames and to 1.5
  MHz behind frames back to back (3 MHz failing in one run of four).
- **As a host** (`host_burst`, one polled frame a request, 20 us apart, 1.5
  MHz): the CH32V006's client on its hardware NSS read twelve frames exact
  in modes 0 and 3, MSb and LSb first, and eight 16-bit frames in mode 3,
  and this host read its answers exact.

### The host above the wire

What a transaction costs beyond its frames' time, measured by letter b at
48 MHz with the select high, in one image for both placements - the
suite's hosts `InRam`, their twins `InFlash` - each line the best of
three or five; `fixed` from a 32-frame transaction less 32 frames, the
suite's wrapper - a request built, the call, the wait on the completion
- inside it. The two twins' wrappers are one template at two places in
the flash, and a zero-length request through them takes 452 and 348
cycles: the fixed costs carry that band, the frame costs do not. Each is
the constant the host states for its placement, and the suite judges
the frames within 10 per cent of it and the fixed costs within 30:

| path | a frame, SRAM / flash | fixed, SRAM / flash | the wire a frame |
|---|---|---|---|
| the engines | the wire (1000 to 1006 thousandths, 1041 and 1045 in two runs at /2) | 1176 / 1012 | 16 at /2, 32 at /4 |
| the pump | 224.5 / 256.5 at /2, 251.3 / 252.5 at /16 - the handler bounds it | 906 / 708 | - |
| the receive loop, one in flight | 28.9 / 51.9 at /2, 46.9 / 52.0 at /4, 141.0 / 152.0 at /16 | 765 / 580 at /4 | 16, 32, 128 |
| the receive loop, two in flight | 512.0 / 511.9 at /64 | - | 512 |
| the transmit-only loop | 15.9 / 24.0 at /2, 31.9 / 32.0 at /4, 63.9 / 63.9 at /8, 128.0 / 128.0 at /16 | - | 16, 32, 64, 128 |
| a 3-frame polled request (a command and two data frames) | 1272 / 1136 in all at /16 | - | 384 |

- **The transmit-only loop** is seven instructions a frame with one taken
  branch, specialized on whether the out buffer exists (the dummy an
  immediate): from SRAM within 5 per cent of the wire at /2 and /4 and at
  the wire from /8; from the flash within 5 per cent at /4 and at the
  wire from /8, its frame at /2 24.0, the fetch's bound (28.0 when the
  function sat half a word off, above).
- **The receive loop** keeps one frame in flight below the threshold:
  some 15 cycles of dead bus a frame at /4 from SRAM (the poll's last
  turn, the read, the write), 20 from the flash; two in flight from /64,
  at the wire there in both.
- **The pump**: one interrupt a frame (no FIFO), the DATAR read the first
  access after the RXNE test, the OVR test a STATR load after it, the
  frame prepared by the previous handler written next, about 45
  instructions a frame, a leaf - 224.5 cycles in the suite from SRAM with
  the hardware prologue's entry and exit and its vector's own work in the
  flash, 256.5 inline in that vector.
- **What `InRam` costs in SRAM** (the map, `.fast_text.brio`): the pump's
  entry 302 bytes (332 with engines bound), the polled loops 998 bytes a
  host type - eight instances, the two widths with and without an out
  buffer and the two loops, of 76 to 198 bytes each; under `InFlash`,
  nothing. Two host types on one instance (this suite's plain and
  engined hosts) carry two sets of loops.
- **The thresholds those costs give**, by placement: the engines' fixed
  cost against the pump's and their frame the wire's - with engines bound
  a data phase rides them from one frame under `InRam` and from two in
  the flash (`dma_min_frames`); a polled request with an in buffer rides
  them from 28 frames under `InRam` and 22 in the flash
  (`dma_min_frames_polled`), the receive loop's dead bus being the
  placement's.
- **The hold-off** is a fact of the image, not of one host: the default
  is the one safe with every driver in the flash - the family's longest
  measured body so is the I2C host's event pump, 333 cycles with its
  vector's stamps and owner switch ([i2c.md](i2c.md)), and with the 36
  of its exit the default is 380: two in flight from /64 on 8-bit frames
  and /32 on 16-bit ones, the same codes for any hold-off from 199 to 454
  (pinned by static_asserts). This image's handlers measure 151 cycles (the
  console), 67 (the tick) and 222 (this pump, stamps included). A program
  that places its handlers in SRAM shortens them and states its own
  hold-off as the host's template parameter - under 199 it moves both
  codes one faster. The pump with two frames in flight from /64 ran 256
  frames at the wire with a console line draining beside it, `spi_ok`.
- **The select and D/C edges are inline**: `PinRef`'s level verbs are
  always inline (pin.hpp), where an out-of-line call there had put four
  calls on every transaction's path.
- **The fault exits, provoked** (letter f, the select high): a polled
  duplex request of 16 at /16 with SPI1 held in reset ends `spi_stalled`
  after about 58 500 cycles - the poll budget, some 28 times the wire's
  2048 - and `recover()` puts the next request back to `spi_ok`. TIM1's
  compare request held on DMA channel 2 (its channel 1's row) leaves the
  receive channel dead - the next owner takes one stray item ([dma.md](dma.md))
  - and a polled request of 64 on the engines over it ends `spi_dma_fault`,
  but only after `spin_dma()`'s bound, 200 000 spins and 6 000 a frame
  whatever the rate: 9.37 million cycles, 195 ms, against the 8 192 the
  block's wire takes at /16 - the caller blocked that long. The
  asynchronous engines over the same channel never claim a completion
  (the suite's bound answers them); with TIM1 reset and `recover()` run,
  both paths complete `spi_ok`.
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
  wire at /4, where brio's receive loop is 1.6 times it. Its DMA example
  arms the transmit channel at VeryHigh and the receive one at High, the
  opposite of the priority rule.

## Not covered yet

Driver gaps, each with its reason:

- HSRXEN's second ladder, HCLK/(BR+2), as rates a Request can ask: 16.3.8
  restricts the mode to /2, the driver keeps the powers of two; measured
  exact at /3, /5 and /6 against one client - a decision for the user.
- `start()`, the phase openings and the engines' launch under `InRam`:
  declined - kept out of line in the flash under either placement, so
  the pump's trampoline does not flatten them into SRAM. Measured in a
  variant that placed them beside the per-frame paths: some 230 cycles
  off a 3-frame polled request, 176 off the receive loop's fixed cost and
  118 off the engines', for some 1.4 KB more SRAM a host with engines
  (`start()` alone 726 bytes) - a cost a transaction, where the placed
  paths' is a cost a frame; a placement of its own for them is a
  decision for the user.
- The simplex modes and the hardware select as the engine's: no user; the
  resource has the bits.
- The CRC as part of a Request: born with a device that checks one.
- A second host on the bus (MODF as an event the engine reports): no
  multi-host user; the resource has the flag and its clear.
- `spin_dma()`'s bound as the rate's: today 200 000 spins and 6 000 a
  frame at any code - right for 16-bit frames at /256 (4 096 cycles each),
  195 ms for 64 frames at /16 (measured, letter f), the caller blocked
  that long on a dead channel. A bound from the poll budget's arithmetic
  wants a margin for the channels' arbitration under a busy controller,
  which is not measured; until then the bound stays the generous one.
- **The receive loop's overrun**: the polled receive loop watches RXNE
  alone (`poll_duplex()` tests STATR for RXNE and TXE and never for OVR,
  read in the release listing), where the pump reads OVR after DATAR and
  ends the transaction `spi_overrun`. With two frames in flight a reader
  held off longer than a frame loses one to OVR (16.2.7), and the loop
  then waits for a frame that never comes until the transaction's budget
  of poll turns runs out: it ends `spi_stalled`, the receive buffer
  flushed - never `spi_ok` over a run with a hole, since it cannot count
  the frames it asked for without the lost one, but under a status that
  names the wrong cause and after the budget's wait. In main context the
  hold-off covers every reader's delay; under the preemptive kernel the
  loop runs inside the arbiter's dispatch, which a dispatch above it
  holds off for its whole length. An OVR test in the loop is a load and a
  branch a frame on a loop that runs at the wire; a decision for the
  user, inside the preemptive kernel's open question on a polled path
  under preemption ([../design/kernel.md](../design/kernel.md),
  "Rubato").

Implemented but not bench-verified, each with what would measure it:

- **The SPI stack under the preemptive kernel**: `test_x035_rubato_link`'s
  key `z` - the arbiter over the engined host against the CH32V006's
  `spi_peer`, its polled loop, pump and engines climbing /256 to /2,
  modes 0 and 3 in both widths, three requesters on one select window
  at /32 (one frame in flight, so the requester below the arbiter on the
  polled loop sees a preemption as a gap on the bus),
  under an object preempting every 97 to 389 us, in both scheduler
  placements ([platform.md](platform.md), "The drivers under the
  preemptive kernel") - is built and not judged: the pair's SPI link
  fails `test_x035_spi`'s own two-board key (its ladder from /4 and this
  chip as client in modes 0 and 3, where modes 1 and 2 pass), the
  reference the key needs. Key `z` ten times on each image, once that
  reference passes, would measure it, the receive loop's overrun above
  among what it judges.
- **The engines' frame at /2, 1041 and 1045 thousandths of the wire in
  two runs**: letter b's ladder broke its +3 per cent there in one run of
  four and again in a later single run, the others 1006 to 1020, the engines' code the same in every build; it is
  not the core's poll of the completion flag (letter s: 0 of 100 outside
  the bracket either way) and was not seen again in 200 measurements.
  Letter b run many times over, with the bus's other masters - the
  console's transport, the tick - accounted at the outlier, would catch
  its cause.
- **This host's receive in modes 1 and 2 against an answering client**:
  the desk's CH32V006 client does not answer in those modes, and this
  chip's own client answers only under a host clocking a frame per
  request; its MOSI in both modes is the standard's (letter w). A client
  that answers in modes 1 and 2 at a frame per select - this chip as the
  peer of a third host - would read it.
- **The client at 12 and 24 MHz with its MISO driven high** (table 3-19's
  24 MHz): lost edges the client's own PA6 level decides, measured and not
  explained; a scope on PA5 at this pad with PA6 high and low at 12 MHz,
  and the two boards on one rail, would say whether it is the die, the
  two rails or the wiring.
- **Letter w's own mode 2 once read SCK low at its first sample** (one
  run of four: fifteen edges, the frame undecoded; the three after it
  read SCK high and sixteen edges): the prime before the frame, or the
  sampler's start, not isolated; the letter run again under a probe on
  PA5 would say which.
- **The other columns and parts**: the other three columns compile on
  every part where they are bonded (`brio check ch32x035`); a board of
  each package would run them.
