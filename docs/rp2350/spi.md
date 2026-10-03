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

**THE CHAPTER SAYS TWO DIFFERENT THINGS ABOUT nSSPOE, AND THE PADS ANSWER
ONE OF THEM.** 12.3.1 says the output enable of the SSPTXD data output IS
controlled by nSSPOE on this chip, and that the peripheral tristates its
output when deselected in client mode, which it offers as the reason
software need not manage the output enable even where several clients
share the data lines. The idle-level list printed under each of the four
Motorola modes and under Microwire (12.3.4.10 to 12.3.4.14) says the
opposite, in the same words all five times: the nSSPOE pad enable signal
is forced high, "this is not connected to the pad in RP2350". MEASURED ON
STEPPING A2 IT IS THE SECOND THAT HOLDS. A deselected client and a client
with SOD set both leave their transmit pad DRIVEN, and driven HIGH - the
same arrangement as the RP2040 ([../rp2040/spi.md](../rp2040/spi.md)),
where nSSPOE reached no pad either. What SOD does do is keep the block's
answers off the wire: the host clocks a steady 0xFF out of a client whose
FIFO is full of frames. The driver depends on none of it -
`SpiClient::drive_output(false)` makes the dark listener by releasing the
PAD, which is right on both silicons and is why the IP file made the pad
the lever - and `sod()` stays a bit of the resource, which is what the
suite points at the question.

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
- `Rp2350Pl022::dma_min_frames`, the data-phase length below which a
  request takes the pump even with engines bound - TEN frames here, the
  quotient of two of this chip's measurements (the bench findings).

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

With the engines: `SpiHost<0, pins, DmaTxEngine<4, uint16_t>,
DmaRxEngine<5, uint16_t>>` - half-word engines, SSPDR's width, so 8-bit
frames ride them at a byte beat and 16-bit frames at a half-word one;
byte engines leave a 16-bit request to the pump, as does a buffer not
aligned to a half-word - and the line the engines report on calling
`Bus::dma_isr()` the same way. The receive block's completion is the
transaction's ONE interrupt: the transmit engine is armed to report a bus
error and nothing else ([dma.md](dma.md)).

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

## Bench findings

All of them on an RP2350 in the QFN-80 package, **stepping A2**, clk_sys
and clk_peri at 150 MHz, and all of them on BOTH architectures, from one
source: `test_rp2350_spi` carries 43 verdicts, the 30 of the loop-back
letters a..e green on the Cortex-M33 pair and on the Hazard3 pair, and
the 13 of the wire letters f..j green on the four self-link wires (letter
i, the engines on the wire, is the one gap below). Where a number differs
between the halves both are given; every verdict reads the same.

- **THE BLOCK IS THE RP2040'S, AND SAYS SO.** SSPPERIPHID 0..3 read 0x22,
  0x10, 0x34, 0x00 - the revision field 3, which is r1p4 - and SSPPCELLID
  the PrimeCell's own 0xB105F00D, on both instances; out of reset SSPSR
  reads 0x03 (both FIFOs empty, the transmit one not full) with the block
  disabled.
- **THE RATE CHOOSER AT THIS clk_peri.** The fastest setting under 75 MHz
  is CPSDVSR 2 with SCR 0, which is **75 MHz exactly** - clk_peri / 2, and
  not the 70.5 Mbit/s 12.3.4.4's worked example names for the same
  register values; 12.5 MHz and 1 MHz come out exact, and 2306 Hz (254 x
  256) is refused as below the pair's reach.
- **cs_setup_us, FOR THE FIRST TIME ON ANY TARGET.** 200 us asked for
  between the select and the first clock, served on the platform timer and
  measured on the system timer - two counters of the same microsecond with
  unrelated phases - lengthens an otherwise identical transaction by
  **203 us on both halves**: sixteen frames at 9.4 MHz cost 21 us bare on
  the Cortex-M33 half and 24 on the Hazard3 one, 224 and 227 with the wait.
  The two transactions have to be told apart from the XIP cache's own
  first-run cost, which is worth some six microseconds on the Hazard3 half
  and under one on the other, so the letter warms the polled path before
  it weighs it.
- **THE PUMP BATCHES.** Sixteen frames on the loop-back cost **three to
  four interrupt entries**, not sixteen: the eight-deep FIFOs are what the
  engine is counting on.
- **THE POLLED LADDER.** 64 frames on the loop-back, Cortex-M33 then
  Hazard3: div2 19 / 17 us, div4 20 / 19, div8 35 / 36, div16 68 / 68, and
  the slower rungs the wire's own time. From div4 down the WIRE sets the
  pace on both halves; at div2 the receive loop's three accesses a frame
  do (the next item). Every rate is byte-exact.
- **THE HOST ABOVE THE WIRE, `bench_rp2350` letter e** - the polled loop
  in both its shapes, the pump, the price of a request - on a host with
  no engines, SPI0 on the same pads with GP17 the select and GP22 the D/C
  line, MISO floating, the receive-overrun flag read after every run and
  clear on every one; Cortex-M33 / Hazard3, the best of five, at clk_peri
  / 2, / 4 and / 16 in mode 0 and in mode 3:
  - THE RECEIVE SHAPE (`spi.poll.rx`, eight frames in flight - written and
    not yet read): 8-bit frames at 37.5 MHz **38.0 / 38.0 cycles a frame in
    mode 0 and 32.0 / 32.0 in mode 3** - the wire with the block's gap, and
    the wire - at 9.375 MHz 152 / 128, and 16-bit frames 70 / 64 and 280 /
    256: the wire's own figures to the hundredth. At 75 MHz its three
    accesses a frame set the pace: **24 / 28 cycles a frame** against a
    16-cycle frame, 16-bit frames at the wire (35 / 32). The loop was 97 /
    93 cycles a frame at 37.5 MHz before this host (the ISR's fill and
    take called per frame, the width reloaded from a static): the same
    request at /4 cost three times the wire.
  - THE WRITE SHAPE (`spi.poll`, no in buffer): at /4 and /16 the same
    figures - the receive loop discarding, since a frame read under the
    wire costs nothing - and at **75 MHz the TNF-paced loop at 19.02 / 19.02
    cycles a frame in mode 0 and 15.99 / 16.00 in mode 3**, the nominal
    wire, which is the one rate the transmit-only shape is taken at, on
    runs longer than the FIFO: its tail (up to eight reads after the wire
    has finished) costs a short request 30 to 130 cycles the receive
    shape spends under the wire instead - measured with the shape taken
    unconditionally, the three-byte request 356 / 354 against 286 / 286.
  - THE RECEIVE SHAPE UNDER A LONG HANDLER: 100 receive requests of 256
    frames on the loop-back at each of the three rates with the tick's
    handler stretched to 1500 cycles - eight frames being 128, 256 and
    1024 cycles - **100 exact and 0 overruns at every rate on both
    halves**, with 8 / 8 / 25 (Cortex-M33) and 8 / 9 / 26 (Hazard3) of the
    stretched ticks landing inside the requests: the frames in flight
    never exceed the receive FIFO's depth, so a handler that holds the
    loop off cannot overrun it, whatever its length.
  - THE PUMP (`spi.pump`, the receive on the interrupt): **40 to 63 cycles
    a frame inside the handler** (was 86 to 119), one interrupt per FIFO
    level - 32 or 33 for 256 frames at 75 and 37.5 MHz, where a handler
    finds the eight in flight all back, 51 or 52 at 9.375 where it finds
    four to five - the overrun flag clear on every run. Wall: 53 / 53
    cycles a frame at 75 and 37.5 MHz, the handler's own entry, stamps and
    refill between batches outlasting the eight frames in flight; the
    wire's own 152 and 128 at 9.375. With the host as it was, the Hazard3
    half RAISED the overrun flag at 37.5 MHz: `start()` armed the receive
    interrupt BEFORE filling the FIFO, so the handler ran inside the fill
    with both sides counting the same frames; the fill now comes first.
  - THE PRICE OF A DCS COMMAND (`spi.req`, a command byte with D/C low and
    0, 2 and 15 data bytes with it high, polled, nothing read back, at
    9.375 MHz): wall less the wire's cycles **196..214 for one byte,
    286..309 for three, 595..633 for sixteen** across the round's builds
    on both halves (the band is the XIP cache's line alignment, which
    every link moves) - of which the block's mode-0 gap is 24 / 72 / 384
    and the phase boundary's drain (the data frames wait for the command
    frame to come back whole, so D/C flips on the wire) one frame's
    latency; the host's own code under 200. Before this host: 379 / 412,
    593 / 636 and 937 / 935. On the engined host the same three requests
    read the same within 15 cycles (they take the polled loop below
    `dma_min_frames`), where before they cost 316, 669 and 979 with an
    interrupt each.
  - THE VENDOR'S LOOP ON THE SAME BOARD (pico-sdk 2.3.1's
    `spi_write_read_blocking` and `spi_write_blocking`, SRAM-resident by
    the SDK's own placement, built by its CMake with its crt, the select a
    GPIO around the call, timed on the same TIMER1): full duplex at 75 MHz
    **27.0 / 38.0 cycles a frame**, at 37.5 MHz 38.0 / 38.0 in mode 0 and
    32.0 / 38.0 in mode 3, at 9.375 MHz the wire's 152 and 128; its
    write-only loop 19.3 / 19.3 cycles a frame at 75 MHz in mode 0. So
    this host's receive shape is ahead of the vendor's loop at 75 MHz on
    both halves and at 37.5 MHz on Hazard3, its write shape reads the
    vendor's at 75, and a 16-frame transfer costs the vendor 61 cycles
    over the wire against this host's write of the same within the wire's
    own time.
- **`dma_min_frames` = 10**, the quotient the chip traits carry: the
  engines' fixed cost per transaction (letter d after this round: 16
  frames at 75 MHz, wall less the wire's cycles and less the block's
  mode-0 gap, **514 / 473 cycles**) over the pump's cost per frame (letter
  e's 49 / 48 cycles a frame in the handler plus the interrupt's own entry
  and exit - the Cortex-M33's 24 cycles, the crt's 48 on Hazard3 - spread
  over the eight frames a batch: 52 / 54) is 9.9 / 8.8, the larger rounded
  up. A data phase shorter than that takes the pump even on a host with
  engines bound.
- **THE ENGINES ON THE LOOP-BACK.** 128 bytes at 37.5 MHz, ISR-completed,
  are exact with **ONE DMA interrupt**, and 64 16-bit frames ride the
  engines at a half-word beat exact with no pump interrupt and one DMA
  interrupt; a 16-bit request on a buffer one byte off a half-word goes
  to the pump and is exact there. A command frame on the pump then data
  on the engines, a polled request on the engines completing inside
  `start()`, and a read with no out buffer through the transmit engine's
  fixed 0xFF cell are all exact. The letter times the 128-byte block, the
  polled request and the 16-bit block twice each - the first run and a
  repeat - with the XIP cache's misses beside each: the block takes
  **112 / 98 us cold, missing 146 / 154 lines, and 40 / 45 us warm**,
  where the wire with mode 0's frame gap is 32 (the next items); the
  polled request, a command frame and 64 data frames at 75 MHz, 21 / 25
  us cold and 14 / 17 warm; the 16-bit block in mode 3, 53 / 49 cold and
  33 / 34 warm against a wire of 27. The repeat still misses 5 to 13
  lines in this image: the cache is two-way, and some lines of this
  image's transaction path share a set; a smaller image of the same
  binding misses none warm.
- **WHAT AN ENGINED TRANSACTION COSTS, `bench_rp2350` letter d**, the
  ruler TIMER1 on clk_sys, MISO floating (and the same on the loop-back),
  the core asleep between the start and the completion; Cortex-M33 /
  Hazard3. `start()` returns after **316 / 267 cycles** - the request's
  words copied into the tenure, one compare for the rate, the select,
  both engines started - and the transaction's ONE interrupt follows the
  last frame; with it a transaction costs the core **about 680 / 440 busy
  cycles** whatever its size, and its FIXED COST, the wall less its
  frames' own time, is **562 / 521 cycles at 16 frames (3.7 / 3.5 us),
  of which 48 the block's gap**, the same at 256 frames and at both
  rates. THE FRAMES' OWN TIME IS NOT THE NOMINAL
  WIRE'S: in mode 0 the PL022 raises its frame signal between two frames
  of a continuous transfer (12.3.4.10, and 12.3.4.12 for mode 2), and the
  gap is 1.5 SCK periods - measured 19 cycles a byte at 75 MHz where the
  bits are 16, 76 at 18.75 MHz where they are 64, 35 and 140 a 16-bit
  frame where they are 32 and 128, x 1.19 and x 1.09 - so with the fixed
  cost 256 frames come out at x 1.10 to x 1.32 of the nominal wire. IN
  MODES 1 AND 3 (SPH = 1) THERE IS NO GAP - measured on a build of the
  same binding in the four modes, warm, by the difference of 16 and 256
  frames: 16.00 and 64.00 cycles a byte at 75 and 18.75 MHz, the nominal
  wire exactly, on both halves, where modes 0 and 2 give 19.00 and 76.00.
  256 16-bit frames at 75 MHz take 9519 / 9499
  cycles, x 1.16, with ONE interrupt; the same request on the pump - byte
  engines, or a misaligned buffer - takes 25836 / 26644, x 3.15 / 3.25,
  with 34 / 33. The core spinning on a flag in RAM or on a register behind
  the APB bridge while the engines run changes a 256-byte block by under
  a hundredth.
- **THE FIRST TRANSACTION PAYS THE XIP CACHE ITS FILL.**
  The very first transaction after the host is brought up takes 10318 /
  8714 cycles in `bench_rp2350` where every later one takes 858 / 837.
  Split on a build of the bench's own binding - 16 frames at 75 MHz, the
  core spinning on the completion's RAM flag, the cache's two counters
  (`Xip::hits()`, `Xip::accesses()`, [flash.md](flash.md)) read around
  each transaction: the first after `init()` costs **11576 / 9354 cycles
  and misses 132 / 126 lines**; the cache invalidated with nothing about
  the SPI changed, 11982 / 11693 and 136 / 165 misses; warm, 794 / 790 and
  none. The SAME SOURCE WITH ITS CODE LINKED INTO SRAM pays the warm wall
  and the rate change and nothing more - 991 / 1160 for its first
  transaction, the remainder exactly what stayed in the flash (one vector
  fetch on the Cortex-M33; the trap entry and the runtime's memcpy on
  Hazard3). So a rate change costs 90 cycles (`apply()`'s SSE pair around
  `configure()`), the DMA's first block after `arm()` and the PL022's
  first frame after SSE cost nothing measurable, and the rest is 126 to
  165 misses of 66 to 82 cycles each: an engined transaction executes
  about 1.1 KB of code (`start()` 420 bytes, the completion's handler
  with `finish_dma()` about 300, the two engine starts 184, `apply()` and
  `clamp()` 164, the launch 84 - Cortex-M33, `-Os`), and the boot's XIP
  setting (EBh at CLKDIV 3, [flash.md](flash.md)) prices a line at that.
  The wire inside is untouched: on the Cortex-M33 at 75 MHz the
  completion's handler is entered before a cold `start()` has even
  returned - the frames were done while its tail was still being fetched.
- **THE VENDOR'S LIBRARY DOING THE SAME TRANSFER ON THE SAME BOARD** is
  the oracle for both numbers: the pico-sdk 2.3.1's `hardware_spi` and
  `hardware_dma`, built by its own CMake with its own crt, SPI0 on the
  same pads at the same rates, two channels with byte beats into SSPDR
  started by one `dma_start_channel_mask()`, timed on the same TIMER1,
  the XIP left as the bootrom set it (the SDK reprograms the QMI only
  when told to: it reads back CLKDIV 3 and EBh too). Inside the block it
  takes **exactly this driver's time - 19.00 cycles a byte at 75 MHz and
  76.00 at 18.75 MHz, on both halves** - so the frame gap is the block's
  and the engines run at the wire. Its warm 16-frame transfer, completed
  by the same interrupt, takes **405 / 452 cycles against this driver's
  794 / 790**: its transfer is two `dma_channel_configure()` calls and
  one store, inlined, where this one carries the arbiter's Request
  through `start()` - stamped, each stamp's own 4 cycles included: the
  Request copied 68 / 91 cycles, an unchanged `apply()` 75 / 64, the D/C
  line, the select and the flush 52 / 56, the two engine starts with
  their two SSPDMACR writes 158 / 123, the completion's handler through
  `finish_dma()` 119 / 110 - a census of the host before the SPI round,
  whose first three items are now the tenure's seven word copies, one
  compare, and no flush. And its first transfer after `spi_init()`,
  completed by its interrupt, pays the same price per line - 14 to 24
  misses, 1158 to 1996 cycles - because it touches six to seven times
  fewer lines; this host's first transaction after `init()` pays 10161 /
  8549.
- **THE CLIENT KEEPS UP ONE RUNG ABOVE THE CHAPTER'S CEILING.** On the
  four wires, the host polled and the client served from its own
  interrupt, 32 frames are exact both ways at **clk_peri / 8, 18.75 MHz**,
  and wrong at 37.5 and 75 - where 12.3.4.4 puts a client's limit at
  clk_peri / 12, 12.5 MHz. That is the same generosity the RP2040 showed.
  The frame period on those rounds is 1125 to 1218 ns whatever the rate,
  because one core is serving both ends and the client's interrupt, not
  the clock, is the pace.
- **WITH SPH = 0 THE PL022 CLIENT TAKES ONE FRAME PER SELECT WINDOW.**
  Sixteen frames clocked under one held select in modes 0 and 2 are heard
  as **one**; in modes 1 and 3 all sixteen arrive. It is the fact a
  multi-frame transaction under a GPIO select must know, and it is the
  block's and not this chip's.
- **THE TRANSMIT PAD'S OUTPUT ENABLE - the one verdict that was expected
  to differ from the RP2040's, and does not.** The instrument is proved
  both ways first on the GP11 -> GP16 wire: a pad driven low reads not
  floating and low, the same pad released reads floating. Then a
  DESELECTED client reads **not floating, level high**, and a client with
  SOD set reads **not floating, level high** while the host clocks
  **0xFF** out of it with 0x80, 0x85, 0x8A waiting in its FIFO. So nSSPOE
  reaches no pad on this stepping, as the modes' own idle lists say and
  12.3.1's paragraph denies; SOD keeps the answers off the wire without
  releasing the line. A dark client - the pad released - reads 0xFF at the
  host and still hears all sixteen frames, and a client nobody reads
  raises its overrun, which clears by writing one.
- **THE ARBITER IS UNTOUCHED.** `util/spi_bus.hpp` and
  `util/bus_master.hpp` carry four transactions with pumped and polled
  interleaved, reject the fifth of a four-deep queue while answering all
  six exactly once, and vote for a sleep when idle and against one when
  busy - on both architectures, with no line changed for either.
- **THE ROLES INVERT** on the same four wires, SPI1 hosting on GP9's
  select and SPI0 listening on GP17, sixteen frames exact both ways; and
  the two engines on the wire carry 64 bytes each way at 2.3 MHz exact
  with the client on its own interrupt.

## Not covered yet

Driver gaps, each with its reason. The ones that belong to the block
rather than to this chip - the block's own frame signal as a host select,
the Microwire transaction, frame widths other than 8 and 16 in the
Request, a client on the DMA engines - are in
[../pl022/README.md](../pl022/README.md) and are not repeated here.

- The transaction path in SRAM: an engined transaction whose lines have
  left the XIP cache pays its lines - 10161 / 8549 cycles for the first
  after `init()` (the bench findings) - which `start()`, the engines'
  starts and the completion's handler placed in `.ram_text` would not
  pay. An attribute takes a string literal and not a trait, so the way
  the IP stratum cannot take is a `Chip::hot_section`; the way this
  family can is its own linker script placing the host's function
  sections (`-ffunction-sections` gives each its own, named for the
  symbol) into `.ram_text` by pattern - about 1.4 KB of SRAM for the two
  shapes, the pump and the engines' launch - measured cold and warm
  before it is taken.
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

Implemented but not bench-verified, each with what would measure it:

- Letter i, the host's engines on the wire against the client, with the
  engines in the half-word shape: the loop-back letters and the bench
  measure them, the wire has not carried them. The four self-link wires
  and the letter.

- The QFN-60's pin table and its compile-time refusals: the stratum
  compiles for that package and refuses the pads it has not got, and no
  QFN-60 part is on the bench. A board carrying one, running the suite's
  letter a against a pin table of thirty pads.
