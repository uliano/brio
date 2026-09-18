# SPI (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 12.3 whole
(12.3.1 what changed from the RP2040, 12.3.2 the overview with the four
interrupt sources, 12.3.3 the functional description - 12.3.3.3 the
prescaler, 12.3.3.4 and 12.3.3.5 the two FIFOs, 12.3.3.7 the combined
interrupt -, 12.3.4 the operation - 12.3.4.2 the disabled-write rule,
12.3.4.4 the clock ratios, 12.3.4.6.1 the bit rate, 12.3.4.7 to 12.3.4.14
the three framings and the four Motorola modes, 12.3.4.16 the DMA
interface -, 12.3.5 the registers), 9.4 (table 645, the GPIO function
table), 12.6.4.1 (the system DREQ table), 7.5 (the reset controller), 3.2
(the interrupt lines), 2.1.3 (the atomic register aliases); Appendix E
names no erratum of this block, and the one that shapes its pads is E9
([pin.md](pin.md)). `docs/design/spi-bus.md` for the Request and the two
completion styles.

The driver is ARM's and not this chip's, so it lives in the IP stratum:
`brio/pl022/spi.hpp` holds the resource, the host engine and the client
([../pl022/README.md](../pl022/README.md)), and `brio/rp2350/spi.hpp`
holds what this chip owes it - the pin table of table 645, the
`Rp2350Pl022` chip traits over `pin.hpp`, `resets.hpp`, `clock.hpp`,
`core.hpp`, `delay.hpp` and `dma_engine.hpp`, and the PUBLIC NAMES
`Pl022<n>` (the resource), `SpiHost<n, pins, ...>` (the engine `SpiBus`
drives) and `SpiClient<n, pins>` (the other end), this chip's aliases of
the three templates there. The reference suite: `test_rp2350_spi`, which
runs on BOTH of this chip's architectures from one source.

This page says what is the RP2350's. What the block itself does - the
prescaler pair, the disabled-write rule, the pump's batching, the phase
boundary, the receive timeout, the engine slots, the client's one frame
per select window in modes 0 and 2 - is the IP stratum's page and is not
restated here.

## What the silicon does

The block is the same one the RP2040 carries, at the SAME REVISION (r1p4,
named in 12.3 on both chips) and with the same eight-deep FIFOs (12.3.3.4,
12.3.3.5). SSPCLK is clk_peri and PCLK is clk_sys, which is why a bit rate
is divided from `Clock::pclk_hz` and never from a second statement of the
rate; clk_peri follows clk_sys undivided unless a program detaches it, and
12.3.4.4's constraint is that SSPCLK may not exceed PCLK. Five things
about it are this chip's:

**ONE FUNCTION COLUMN, and that is worth stating because the UART's is
two.** The pads march in groups of four - RX, CSn, SCK, TX - and the
groups cycle SPI0, SPI0, SPI1, SPI1 over the whole bank, so the instance
is bit 3 of the pad number: GP0..GP7 are SPI0's, GP8..GP15 SPI1's,
GP16..GP23 SPI0's, up to GP47 on the QFN-80. That is the RP2040's rule
over a longer bank. Unlike the UART, which gained a second data pair on
every group's flow-control pads under function 11, the SPI appears in
ONE column of table 645 and one only: F1, on every pad that carries it,
on both packages. So a pin set here is four plain pad numbers.

Every signal of both instances is reachable in the QFN-60 as well: SPI0
clocks on GP2, GP6, GP18 and GP22 there and SPI1 on GP10, GP14 and GP26,
each with its own group's other three signals beside it. The eighteen
pads above GP29 (GP30..GP47) are the QFN-80's alone.

**THE BIT RATE AT THIS CLOCK.** clk_peri / (CPSDVSR x (1 + SCR)), the
prescaler even in 2..254 and SCR in 0..255. At a clk_peri of 150 MHz that
is 75 Mbit/s at the top (CPSDVSR 2, SCR 0) down to 2306 Hz at the bottom
(254 x 256), and a ceiling the pair cannot reach is REFUSED rather than
rounded up past the slowest. A CLIENT needs SSPCLK at least twelve times
its own clock (12.3.4.4: the input is double-synchronized), so 12.5
Mbit/s here. Note that 12.3.4.4's own worked example names 70.5 Mbit/s as
the peak at a clk_peri of 150 MHz with exactly those register values; the
equation printed two paragraphs earlier gives clk_peri / 2, and that is
what the driver's chooser computes.

**WHAT 12.3.1 CHANGED, AND WHAT IT LEADS THIS PAGE TO EXPECT.** On the
RP2040 the block's pad-enable output nSSPOE was not connected to the pad,
so SOD stopped the block driving and left the pad driving whatever it
held - measured there ([../rp2040/spi.md](../rp2040/spi.md)). On this chip
12.3.1 says the output enable of the SSPTXD data output IS controlled by
nSSPOE, and that the peripheral tristates its output when deselected in
client mode, which it offers as the reason software need not manage the
output enable even where several clients share the data lines. Two things
follow from that sentence, and NEITHER IS MEASURED ON THIS SILICON YET:
a client's transmit pad should be undriven while its select is high, and
SOD should release that pad rather than freeze it. The driver depends on
neither - `SpiClient::drive_output(false)` makes the dark listener by
releasing the PAD, which is right on both silicons and is why the IP file
made the pad the lever - and `sod()` stays a bit of the resource, which is
what a suite points at the question.

**THE PADS COME UP ISOLATED** (9.11, and [README.md](README.md)): a pad
answers nothing until the isolation latch is cleared, which every
configuring verb of `pin.hpp` does as it writes. Two of the four signals
are handed over with a PULL-UP, because this chip's pads reset pulled
DOWN: a HOST's receive pad, so an answer line with nothing driving it
reads as the idle ones a bus expects rather than as zeros, and a CLIENT's
select pad, so an unwired select reads not-selected. And because erratum
RP2350-E9 is live on the A2 stepping, an idle LEVEL on a pad of this chip
is history and not a measurement: telling a released transmit pad from a
driven one takes two reads, one under each pull, which is the instrument
the suite's letter h builds.

**THE THREE NUMBER TABLES ARE NOT THE RP2040'S**, and each has its own
reason. The DREQ rows are 24..27 where they were 16..19, because a third
PIO and its twelve rows moved everything above them (12.6.4.1,
`rp2350/dma_engine.hpp`). The reset bits are 18 and 19 where they were 16
and 17, the controller governing twenty-nine blocks against twenty-seven
(7.5). And the interrupt lines are 31 and 32 where they were 18 and 19
(3.2) - shared between the architectures (3.8.4.2), so an app binds
`isr_spi0` / `isr_spi1` once and is bound on both halves. The controller
behind those names is not the same object - an NVIC on the Cortex-M33
half, Hazard3's own on the other - which is exactly why the IP stratum's
concept takes the controller as a TYPE.

## Types and verbs

What the IP stratum names (the mode and framing vocabulary, the prescaler
arithmetic, the register bit layouts, the resource's verbs, the host's
Request and its two ISR bodies, the client's surface) is in
[../pl022/README.md](../pl022/README.md). This stratum adds:

- `SpiPins` - the four pad numbers, each the instance's own or `0xFF` for
  a signal the set leaves out. A host with no receive pad is a write-only
  bus; the host's chip select is NOT here (it is a GPIO the Request
  carries), while the CLIENT's select IS.
- `spi_pad_instance(pin)` - which instance the four pads of that pin's
  group belong to: bit 3 of the pad number.
- `spi_rx_pin(n, pin)`, `spi_cs_pin(n, pin)`, `spi_sck_pin(n, pin)`,
  `spi_tx_pin(n, pin)` - table 645 as constexpr facts, in the table's own
  order.
- `spi_pins_valid(n, pins)` - a legal pin set for instance n. The PACKAGE
  is not asked here: the table is the DIE's, and a pad the QFN-60 has not
  bonded is refused one level down by `Pin<n>`'s own static_assert, which
  says so in those words.
- `Rp2350Pl022` - the chip traits, which nothing above this file names.
  Its `pad_function` is the single code `PinFunction::spi`, and its
  `DelayRate` is [clock.md](clock.md)'s - the microsecond busy-wait of
  this target, which counts on the PLATFORM TIMER and not on a core
  counter, so a Request's `cs_setup_us` is served once `Mtime::start` has
  run and refused (with no time spent) before that.
- `Pl022<n>`, `SpiHost<n, pins, TxEngine, RxEngine>` and `SpiClient<n,
  pins>` - the public names, this chip's aliases of the three IP
  templates, with `NoDmaEngine` in both slots by default.

Beside them, at the foot of the file, is the VOCABULARY CHECK: the IP
file spells the block's bit layout itself, because it may not read a
vendor header, and a block of `static_assert`s holds every one of those
constants against this chip's own device description (`SPI_SSPCR0_*`,
`SPI_SSPCR1_*`, `SPI_SSPSR_*`, `SPI_SSPIMSC_*` with `SSPRIS`, `SSPMIS`
and `SSPICR` over the same layout, `SPI_SSPCPSR_*`, `SPI_SSPDR_*`,
`SPI_SSPDMACR_*`). It costs nothing and it is what makes "the register
description is identical" a checked claim rather than a remembered one.

## How to use it

A host, with the select as an ordinary pin:

```cpp
constexpr brio::SpiPins pins{.sck = 18, .tx = 19, .rx = 16};
using Bus = brio::SpiHost<0, pins>;
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
using Cs = brio::Pin<17>;

Bus::init(clock, 8'000'000);                          // a ceiling for the whole bus
Cs::output(true);

Bus::Request r{};
r.cs = Cs::ref();
r.cmd = brio::lend<brio::Lease::reply>(cmd);   r.cmd_len = 1;
r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
r.clock = brio::SpiClocks::div4;                      // 37.5 MHz at 150
r.mode = brio::SpiMode::mode0;
r.reply = brio::reply_to<Display, brio::SpiDone>();
brio::post<Spi>(r);

extern "C" void isr_spi0() {
    if (Bus::isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

The same source builds for both architectures; `isr_spi0` is a
vector-table slot on one half and a dispatch entry on the other.

With the engines: `SpiHost<0, pins, DmaTxEngine<4>, DmaRxEngine<5>>` and
the line the engines report on calling `Bus::dma_isr()` the same way:

```cpp
extern "C" void isr_dma_0() {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

A client, whose select IS a pad:

```cpp
constexpr brio::SpiPins client_pins{.sck = 10, .tx = 11, .rx = 8, .cs = 9};
using Client = brio::SpiClient<1, client_pins>;
Client::init(clock, {.mode = brio::SpiMode::mode1});
Client::interrupts(brio::SpiInterrupt::rx | brio::SpiInterrupt::rx_timeout, true);
Client::enable(first_answer);
extern "C" void isr_spi1() {
    (void)Client::isr();
    while (const auto f = Client::poll()) { heard(*f); }
    while (Client::writable() && have_answers()) { Client::write(next_answer()); }
}
```

## Not covered yet

Driver gaps, each with its reason. The ones that belong to the block
rather than to this chip - the block's own frame signal as a host select,
the Microwire transaction, frame widths other than 8 and 16 in the
Request, a client on the DMA engines - are in
[../pl022/README.md](../pl022/README.md) and are not repeated here.

- The TI and the Microwire framings ON THE WIRE: both are codes
  `SpiConfig` takes and the resource reads back, and no device on this
  desk speaks either. The pair of instances could speak TI to each other
  on the four wires they share; the letter that would do it is born with
  a reason to.
- A second device on one bus: the Request's per-transaction mode, rate
  and width exist so that two devices can share a host, and the desk
  carries one peripheral instance and no third party, so nothing measures
  the reprogramming between two selects.
- The D/C line of the Request: it is carried and driven, and what makes
  it worth a letter is a display, which this desk has not got.
- Detaching clk_peri from clk_sys: `Clock`'s `PeriSource::crystal` is
  what would hold the bit rate still across a clk_sys change, and this
  target has no dynamic clock yet, so nothing changes clk_sys under a
  running bus.

Implemented but not bench-verified, each with the letter of
`test_rp2350_spi` that will measure it, on both architectures:

- Table 645 as the driver states it, the rate chooser at this clk_peri
  (75 MHz at the top, 12.5 MHz and 1 MHz exact, nothing under 2306 Hz),
  the PrimeCell identification registers against the reset values this
  chip's own description states for them, the block's reset state, the
  four refusals, and the loop-back proven on the pump (letter a).
- The four Motorola modes at 8 and 16 bits on the loop-back, a command
  phase then a data phase, a read with no out buffer, the select released
  after every transaction, an empty request completing on the spot - and
  `cs_setup_us` MEASURED, 200 us asked for between the select and the
  first clock and the two transactions differenced on the system timer
  (letter b). It is the first bench to answer that one on any target.
- The polled path at all eight named rates, div2 to div256, 64 frames
  each, timed, with the per-frame cost beside the wire's own (letter c).
- THE DMA ENGINES on the loop-back: a 128-byte block ISR-completed, a
  command frame on the pump then data on the engines, a polled request
  completing inside `start()`, the 16-bit fallback to the pump, and a
  read with no out buffer through the transmit engine's fixed cell
  (letter d). This letter and letter i are the only two that need
  [dma.md](dma.md)'s engines, and they are apart from the bus verdicts on
  purpose.
- `SpiBus` over `SpiHost` with `util/bus_master.hpp` unchanged: four
  transactions and four replies with pumped and polled interleaved, the
  rejection when the queue is full, and both sleep votes (letter e).
- ON THE WIRE between the two instances: sixteen frames both ways in
  modes 1 and 3 pumped and polled, sixteen 16-bit frames, four frames
  delivered by the receive timeout, the client's select pad reading
  not-selected between transactions, and the PL022 client's one frame per
  select window in modes 0 and 2 (letter f).
- The rate ladder on the wire from div2 down: where the client's
  clk_peri / 12 ceiling really lies on this silicon, which on the RP2040
  turned out to be one rung more generous than the chapter's number
  (letter g).
- THE TRANSMIT PAD'S OUTPUT ENABLE, which is the one verdict this chapter
  expects to differ from the RP2040's: an instrument that calls a line
  floating when a pull-down and a pull-up read it differently - proved
  both ways on a pad driven low and then released, which is also what
  makes it E9-proof - then the same instrument on a client that is
  deselected and on a client with SOD set. 12.3.1 says both should be
  undriven here; on the RP2040 the second was measured DRIVEN. Beside
  them, a dark client answering nothing and hearing everything, and a
  client that does not read overrunning its eight-deep FIFO (letter h).
- The two engines on the wire with the client on its own interrupt, 64
  bytes each way (letter i).
- The roles inverted on the same four wires, SPI1 hosting and SPI0
  listening on its select pad (letter j).
- What one core serving both ends costs: the suite runs its pumped wire
  rounds slower than its polled ones for that reason, and the rung at
  which the client's interrupt stops keeping up is a number letters f and
  g will give.
- The QFN-60's pin table and its compile-time refusals: the stratum
  compiles for that package and refuses the pads it has not got, and no
  QFN-60 part is on the bench.
