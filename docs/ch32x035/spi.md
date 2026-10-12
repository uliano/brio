# SPI (CH32X035)

The one SPI of RM ch. 16 - host and client, 8- or 16-bit frames, the four
clock modes, a software or hardware chip select, hardware CRC, the simplex
arrangements, two DMA requests - and the host engine util/spi_bus.hpp's
arbiter drives on this silicon as on every other stratum. Documents of
record: the CH32X035 reference manual V1.8 (16.1 for the features, 16.2.1
for the pins and the select, 16.2.2 and 16.2.3 for the two roles, 16.2.4
for the simplex modes, 16.2.5 for the CRC, 16.2.7 for the errors, 16.3 for
the registers; table 8-4 for the pads, 8.3.2.1 for SPI1_RM; RM 9.2.3 for
the DMA requests; table 7-1 for the vector), the CH32X035/X033 datasheet
V1.7 (table 2-1 for the pads, table 3-19 for the timing, table 3-14 for the
pads' speed), and WCH's EVT library (`ch32x035_spi.c` and the six SPI
examples) as the vendor's shape. Driver:
[brio/ch32x035/spi.hpp](../../brio/ch32x035/spi.hpp), the columns in
[brio/ch32x035/afio.hpp](../../brio/ch32x035/afio.hpp); reference suite
`test_x035_spi` against the desk's CH32V006K8U6 running its `spi_peer`
([../ch32v00x/spi.md](../ch32v00x/spi.md)); this chip's own `spi_peer`
is that board's suite's instrument (below).

## What the silicon does

- **The STM32F1's SPI under WCH's names**, with two registers' worth of
  its own: HSCR (16.3.8, a "high-speed read" mode) and CTLR2.ODEN (16.3.2,
  open-drain NSS, SCK and MOSI).
- **No FIFO.** One transmit buffer, one shift register, one receive buffer
  (figure 16-1). TXE rises "when the data has reached the shift register"
  and a frame written then follows the one shifting with no gap ("fill the
  data register with data to maintain the complete data flow", 16.2.2);
  RXNE rises at "the last sample clock edge of the data word". The
  receive buffer holds ONE frame: a frame that completes while the one
  before it stands unread is lost, OVR set (16.2.7), the flag cleared by
  a DATAR read followed by a STATR read.
- **8 or 16 bits** (DFF), SCK = HCLK / 2^(BR+1), /2 to /256 (16.3.1), up
  to "half of FHCLK" (16.1): 24 MHz at 48, table 3-19's ceiling for both
  roles. Every bus of this series runs at HCLK, so HCLK is the reference.
- **THE CHAPTER'S TWO MODE NUMBERINGS DISAGREE** and neither is the
  Motorola one. Table 16-1 prints CPOL 0/1/1/1 and CPHA 0/0/0/1 for modes
  0..3, a table no numbering produces; figures 16-2..16-9 number the modes
  CPHA first (mode 1 = CPHA 0, CPOL 1; mode 2 = CPHA 1, CPOL 0). The
  driver's `SpiMode` is the Motorola numbering every stratum speaks, mode =
  CPOL x 2 + CPHA, written into the bits by name - and measured standard
  in all four (letter w, below).
- **NOTHING IS ENABLE-PROTECTED** (measured): 16.3.1 says DFF and CRCEN
  "can only be written when SPE is 0" and that MSTR, CPOL, CPHA and
  LSBFIRST must not change "during communication", and a raw write to
  each of eight fields lands with SPE set (DFF, CRCEN, CPOL, CPHA,
  LSBFIRST, BR, MSTR, BIDIMODE). The driver's disable/enable pair is the
  only protection there is.
- **HSRXEN CHANGES THE LADDER** (measured). 16.3.8 makes the bit a read
  enable for "CLK greater than or equal to 36MHz", "only valid when the
  clock is divided by 2" and "not readable"; 16.3.1's note on BR says that
  with it set "the SCK frequency is FHCLK/(BR+2)". 16.3.1 is right: with
  the bit set every code gives HCLK/(BR+2) - a second ladder /2, /3 ..
  /9 - and HSCR reads zero whatever was written. The two texts agree at
  BR = 000 alone.
- **The select has three arrangements** (16.2.1): software (SSM, the
  level from SSI, the pad free), hardware output (SSOE), hardware input (a
  low level raises MODF and demotes a host). SSOE drives NSS low "when the
  SPI host sends output" - measured: low from the host's FIRST FRAME and
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
- **THE CLIENT'S RESTRICTIONS, MEASURED.** 16.2.3: "If transmitting in
  slave mode is required, CPOL needs to be set and configured as mode 2 or
  mode 3"; 16.3.1, of LSBFIRST: "LSB is only supported by SPI as host".
  Under the CH32V006's host this die RECEIVES exact in all four modes and
  both bit orders, ANSWERS exact in all four modes MSb first - the first
  restriction does not hold here - and LSb first its MISO never moves, in
  every mode: the second holds, for what a client sends (letters r and w).
  16-bit frames and the CRC as a client, both ways, measured under the
  CH32V006's host (`spi_peer`, below).
- **AS A CLIENT, ITS RATE DEPENDS ON ITS OWN MISO PAD** (measured, cause
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
| SRAM execution (the instruction fetch) | offered, the program's choice per host, for the paths that run once a frame | the host's placement (util/placement.hpp): under `InRam` the pump's entry and both polled loops run from `.fast_text.brio` - the transmit-only loop at the wire at /2, the pump's frame a sixth shorter behind a vector in the flash, and their cost independent of the layout; under `InFlash`, the default, they cost no SRAM; the paths that run once a transaction stay in the flash under either (measured, below) |

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
  IMAGE's longest window the host's vector waits behind, 380 by default
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
  listener), and the RUNS a transaction is served by - `answer()`,
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

extern "C" BRIO_CH32_INTERRUPT void spi1_handler() {
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
PC6, PC7). `z` - the block, the ladder and the costs, the refusals - runs
with the far board dark: 24 verdicts. `y` against the CH32V006 running
`spi_peer` on its crystal, commanded in band through spi_link: 36
verdicts. Each was green whole on the final images.

- **The reset values are table 16-2's** (STATR 0x0002, CRCR 0x0007, the
  rest zero), a control word lands as spelled, and eight of eight CTLR1
  fields take a write under SPE.
- **THE ENGINES RUN AT THE RATE ASKED**: a frame inside an engined block,
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
- **THIS CHIP AS CLIENT** (letter r, the peer's `host_burst` at 1.5 MHz,
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
  flash a loop's cost also moves with where the code around it sits: the
  host's own transmit-only loop took 20.0 and 28.0 cycles a frame at /2
  and its receive loop 76.0 and 52.0 at /4 in two builds that changed
  nothing in the loops (and 32-byte alignment of the loop does not hold
  it: 50.4 in one aligned image, 70.9 in another). So the host offers
  the program its per-frame paths in SRAM - the pump's entry and both
  polled loops, `InRam` ([platform.md](platform.md), "A handler in
  SRAM") - and the table below gives both placements, the suite's own
  vector in the flash.
- **The pump's placement** (letter b at /2, where the handler bounds the
  bus, the suite's vector with its stamps and its owner switch): 268.6
  cycles a frame with the body inline in the vector in the flash
  (`InFlash`), 225.1 with it in SRAM behind that vector (`InRam`); the
  flash body out of line, a call from the vector, took 300.5 - the reason
  `InFlash` is inline (util/placement.hpp). A program that places the
  vector itself (`.fast_text`) runs an `InFlash` body from SRAM with it,
  no call at all; measured on the I2C host's pump, where it is the
  fastest of the three ([i2c.md](i2c.md)).

### This part as the desk's instrument: `spi_peer`

`spi_peer` makes the CH32X035F8U6 the desk's SPI client for the
CH32V006's suite (`test_ch32_spi`, [../ch32v00x/spi.md](../ch32v00x/spi.md)):
spi_link's command channel in band over SPI1's column 0 with the HARDWARE
select on PA4 (pulled up), the dark listener, its answer windows in mode
0, the exchange served on the DMA engines (channels 2 and 3, polled with
no interrupt, the window read off the NSS pad) or on the client's runs,
the EXTENDED EXCHANGE whole - 16-bit frames (the engines in half-word
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
48 MHz with the select high, in ONE IMAGE for both placements - the
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
| the engines | the wire (1000 to 1006 thousandths, 1045 once at /2) | 1176 / 1012 | 16 at /2, 32 at /4 |
| the pump | 225.1 / 268.6 at /2, 254.2 / 264.6 at /16 - the handler bounds it | 1050 / 681 | - |
| the receive loop, one in flight | 28.9 / 51.9 at /2, 46.9 / 52.0 at /4, 141.0 / 152.0 at /16 | 765 / 580 at /4 | 16, 32, 128 |
| the receive loop, two in flight | 512.0 / 511.9 at /64 | - | 512 |
| the transmit-only loop | 15.9 / 28.0 at /2, 31.9 / 32.0 at /4, 63.9 / 63.9 at /8, 128.0 / 128.0 at /16 | - | 16, 32, 64, 128 |
| a 3-frame polled request (a command and two data frames) | 1272 / 1136 in all at /16 | - | 384 |

- **The transmit-only loop** is seven instructions a frame with one taken
  branch, specialized on whether the out buffer exists (the dummy an
  immediate): from SRAM within 5 per cent of the wire at /2 and /4 and at
  the wire from /8; from the flash within 5 per cent at /4 and at the
  wire from /8, its frame at /2 the layout's (20.0 and 28.0 in two
  builds).
- **The receive loop** keeps one frame in flight below the threshold:
  some 15 cycles of dead bus a frame at /4 from SRAM (the poll's last
  turn, the read, the write), 20 from the flash; two in flight from /64,
  at the wire there in both.
- **The pump**: one interrupt a frame (no FIFO), the DATAR read the first
  access after the RXNE test, the OVR test a STATR load after it, the
  frame prepared by the previous handler written next, about 45
  instructions a frame, a leaf - 225.1 cycles in the suite from SRAM with
  the hardware prologue's entry and exit and its vector's own work in the
  flash, 268.6 inline in that vector.
- **What `InRam` costs in SRAM** (the map, `.fast_text.brio`): the pump's
  entry 302 bytes (332 with engines bound), the polled loops 798 bytes a
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

Implemented but not bench-verified, each with what would measure it:

- **The engines' frame at /2, once 1045 thousandths of the wire**: one
  run of letter b in four broke the ladder's +3 per cent there, the other
  three 1006 to 1020, the engines' code the same in every build; what the
  core's poll on the completion flag, a load from SRAM every few cycles,
  takes from the channels' bus share is unmeasured - the ladder repeated
  with the core asleep through the block would separate the two.
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
- **The client's vectors**: the suite's clients are polled runs and DMA;
  an image binding `isr()` to spi1 would measure them.
- **`spi_stalled`, `spi_dma_fault` and `recover()`**: no run raised them;
  a polled request with the block held in reset, or a channel stopped
  under a block, would.
- **`release()`'s reset pulse**: the controller holds a raised request
  ([dma.md](dma.md)), so the pulse is the contract's; a request left on
  channel 2 or 3 by a released SPI, under the next owner's block, would
  stage it.
- **The other columns and parts**: the other three columns compile on
  every part where they are bonded (`brio check ch32x035`); a board of
  each package would run them.
