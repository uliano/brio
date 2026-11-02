# SPI (RP2350)

This document is for a programmer who puts a device on an SPI bus of an
RP2350 - a display, a converter, a memory - or who makes this chip
answer as the device on another chip's bus, on either of the chip's two
processor architectures. The chip carries two ARM PL022 synchronous
serial ports. Their driver is written once for every family that carries
the block, in the PL022 IP stratum ([../pl022/README.md](../pl022/README.md));
this page and `brio/rp2350/spi.hpp` are what the RP2350 adds to it - the
pads, the reset bits, the interrupt lines, the DMA request numbers,
which clock the bit rate is divided from and the busy-wait a select's
setup time is spent on - and the names a program writes: `Pl022<n>`, the
instance; `SpiHost<n, pins, ...>`, the host engine; `SpiClient<n, pins>`,
the other end. A host is not usually driven directly: a program names
one arbiter per bus, `SpiBus` (an active object - a component of a brio
program with its own event queue, served one event at a time by the
kernel, [docs/learn/introduction.md](../learn/introduction.md) - that
takes the transactions of every client of the bus in turn and answers
each with an event, [design/spi-bus.md](../design/spi-bus.md)), and
posts it a request.

The smallest use, a host with its select as an ordinary pin and one
transaction posted to the arbiter, is the first block under "How to use
it" below:

```cpp
constexpr brio::SpiPins pins{.sck = 18, .tx = 19, .rx = 16};
using Bus = brio::SpiHost<0, pins, brio::DmaTxEngine<4, uint16_t>,
                          brio::DmaRxEngine<5, uint16_t>>;
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
using Cs = brio::Pin<17>;

brio::Dma::init();                                    // before the host binds its engines
if (!Bus::init(clock, 8'000'000)) { /* the ceiling cannot be made, or the block did not come up */ }
Cs::output(true);

Bus::Request r{};
r.cs = Cs::ref();
r.cmd = brio::lend<brio::Lease::reply>(cmd);   r.cmd_len = 1;
r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
r.clock = brio::SpiClocks::div4;                      // 37.5 MHz at 150, clamped to the ceiling
r.mode = brio::SpiMode::mode0;
r.reply = brio::reply_to<Display, brio::SpiDone>();
r.engine = brio::SpiEngine::dma;
brio::post<Spi>(r);                                   // asynchronous

extern "C" void isr_dma_0() {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

This is an excerpt: it assumes the program's platform `P`, its clock object
`clock`, an active object `Display` that handles `SpiDone`, the buffers
`cmd` and `pixels`, and the statements where they belong, the setup in
`main()` and the post in a handler.

Line by line: `pins` names the clock, transmit and receive pads of SPI0
by number (a pad that is not the instance's for its signal does not
compile); `Bus` names the host engine over it with two DMA engines,
channels 4 and 5, half-word wide (the data register's width, so both
frame sizes ride them) and reporting on the DMA controller's line 0, the
code in the flash (the placement at its default); `Spi` is the arbiter
for that bus, an active object of the program's `Tenuto` over platform
`P`, with four requests of queue and no timeout, so a completion that never
comes would leave the bus busy for ever (a limit is its fifth argument,
`brio::SpiBus<Bus, P, 4, brio::BusPassThrough, brio::ticks_from_ms<P>(20)>`);
`Cs` is the select, an ordinary pin
the request carries; `Dma::init()` releases the DMA controller the
host's `init()` binds its engines on; `Bus::init()` takes the clock
object (`constexpr SysClock clock;`, [clock.md](clock.md)) and an optional
ceiling for every transaction on the bus; `Cs::output(true)` drives the select high, not
selected, before the first request; the request names the select, lends
its two buffers (`brio::lend<brio::Lease::reply>` - the buffers stay
the caller's and are only read until the reply), the rate as a division
of clk_peri, the mode, and where the reply goes (`Display`, the active
object that posts it, receives an `SpiDone`); `post` hands it to the
arbiter. The request is ASYNCHRONOUS, `SpiEngine::dma`, and
asynchronous is the DMA engines: its 64 data frames ride them, the core
free for their wire time (the one command frame, too short to repay a
block of its own, goes out polled inside `start()` first); and the
`extern "C"` function is the DMA line's handler, under the crt's name
for line 0, the same on both architectures, which tells the arbiter when
the transaction has completed.

What decides correctness even there. `init()` comes after the clock's
`SysClock::init()`: the bit rate is a division of the clk_peri rate the clock type
claims (`Clock::pclk_hz`), not read from the hardware. `init()` answers
false when the ceiling cannot be produced or the block did not come up;
`Dma::init()` comes before it. The
program binds `isr_dma_0` itself; left unbound, the engines' first
completion goes into the crt's default spin and the program stops there
(the arbiter's per-bus timeout cannot answer from inside a spinning
handler); the host's `init()` binds the engines and enables that line in this
core's interrupt controller ([dma.md](dma.md)). A host with no engines (`SpiHost<0, pins>`) runs polled
requests alone and answers an asynchronous one `spi_rejected`, at once
and with nothing on the wire - `static_assert(Bus::has_dma)` in a
program that posts one makes it a compile error; and an asynchronous
request too short to repay the engines at its rate runs polled and
completes inside `start()` ("The short request" below). A request naming
`SpiEngine::polled` runs an explicit loop inside `start()`, in the arbiter's dispatch,
for its whole wire time, holding every other active object meanwhile, with
interrupts enabled; at any length and with engines bound or not, so it needs no
vector. A polled wait that runs out answers `spi_stalled` and a DMA block the
bus refuses `spi_dma_fault`, each with the select released, and the next request
starts on a clean block with nothing asked of the program ("What each answer
asks of the program" below). The two lent buffers must stay
untouched and in scope from the post to the `SpiDone` the arbiter
replies with; nothing checks it. The request is posted from an active
object's handler, which is what the reply is sent back to. `init()`
hands the pads over itself, the isolation latch dropped and a host's
receive pad pulled up ("What the silicon does" below); the select pad
is the program's, configured before the first request. And a request's
`cs_setup_us` is spent on this target's microsecond ruler: served once
`Mtime::start(clock)` has run, refused with no time spent before that.

The rest of this page, in order: the documents of record; what the
silicon does (the one function column, the bit rate at this clock, what
the chapter says about the transmit pad's output enable and what the
pads do, the isolated pads, the three number tables); the types and
verbs this stratum adds; how to use it, the host, its engines and the
client; the bench findings, most of the page, on the loop-back and on
the pair's wire; and what is not covered. What the block itself does is
the IP stratum's page.

## Documents of record

The RP2350 datasheet (build d126e9e), 12.3 whole
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
ways to run one.

The driver is ARM's and not this chip's, so it lives in the IP stratum:
`brio/pl022/spi.hpp` holds the resource, the host engine and the client
([../pl022/README.md](../pl022/README.md)), and `brio/rp2350/spi.hpp`
holds what this chip owes it - the pin table of table 645, the
`Rp2350Pl022` chip traits over `pin.hpp`, `resets.hpp`, `clock.hpp`,
`core.hpp`, `delay.hpp` and `dma_engine.hpp`, and the public names
`Pl022<n>` (the resource), `SpiHost<n, pins, ...>` (the engine `SpiBus`
drives) and `SpiClient<n, pins>` (the other end), this chip's aliases of
the three templates there. The reference suite: `test_rp2350_spi`, which
runs on both of this chip's architectures from one source, on the
loop-back; the wire is the desk partner's, SPI1 on GP30 / GP28 / GP31
with GP41 its select, to the WeAct RP2040
([../boards/weact-rp2350b.md](../boards/weact-rp2350b.md)). A letter of
a suite is one command key of its console and the group of verdicts it
runs; `z` is the key that runs every letter but `x`, the slip
statistics, which runs by name.

This page says what is the RP2350's. What the block itself does - the
prescaler pair, the disabled-write rule, the phase boundary, the engine
slots, the short request's arithmetic, the client's one frame per select
window in modes 0 and 2 - is the IP stratum's page and is not restated
here.

## What the silicon does

The block is the same one the RP2040 carries, at the same revision (r1p4,
named in 12.3 on both chips) and with the same eight-deep FIFOs (12.3.3.4,
12.3.3.5). SSPCLK is clk_peri and PCLK is clk_sys, which is why a bit rate
is divided from `Clock::pclk_hz` and never from a second statement of the
rate; clk_peri follows clk_sys undivided unless a program detaches it, and
12.3.4.4's constraint is that SSPCLK may not exceed PCLK. Five things
about it are this chip's:

**One function column, and that is worth stating because the UART's is
two.** The pads march in groups of four - RX, CSn, SCK, TX - and the
groups cycle SPI0, SPI0, SPI1, SPI1 over the whole bank, so the instance
is bit 3 of the pad number: GP0..GP7 are SPI0's, GP8..GP15 SPI1's,
GP16..GP23 SPI0's, up to GP47 on the QFN-80. That is the RP2040's rule
over a longer bank. Unlike the UART, which gained a second data pair on
every group's flow-control pads under function 11, the SPI appears in
one column of table 645 and one only: F1, on every pad that carries it,
on both packages. So a pin set here is four plain pad numbers.

Every signal of both instances is reachable in the QFN-60 as well: SPI0
clocks on GP2, GP6, GP18 and GP22 there and SPI1 on GP10, GP14 and GP26,
each with its own group's other three signals beside it. The eighteen
pads above GP29 (GP30..GP47) are the QFN-80's alone.

**The bit rate at this clock.** clk_peri / (CPSDVSR x (1 + SCR)), the
prescaler even in 2..254 and SCR in 0..255. At a clk_peri of 150 MHz that
is 75 Mbit/s at the top (CPSDVSR 2, SCR 0) down to 2306 Hz at the bottom
(254 x 256), and a ceiling the pair cannot reach is refused rather than
rounded up past the slowest. A client needs SSPCLK at least twelve times
its own clock (12.3.4.4: the input is double-synchronized), so 12.5
Mbit/s here. Note that 12.3.4.4's own worked example names 70.5 Mbit/s as
the peak at a clk_peri of 150 MHz with exactly those register values; the
equation printed two paragraphs earlier gives clk_peri / 2, and that is
what the driver's chooser computes.

**The chapter says two different things about nSSPOE, and the pads answer
the first.** 12.3.1 says the output enable of the SSPTXD data output is
controlled by nSSPOE on this chip, and that the peripheral tristates its
output when deselected in client mode, which it offers as the reason
software need not manage the output enable even where several clients
share the data lines. The idle-level list printed under each of the four
Motorola modes and under Microwire (12.3.4.10 to 12.3.4.14) says the
opposite, in the same words all five times: the nSSPOE pad enable signal
is forced high, "this is not connected to the pad in RP2350" - which is
what the RP2040's chapter says, and what the RP2040's pads do
([../rp2040/spi.md](../rp2040/spi.md)). On this chip 12.3.1 holds: the
pad's own output enable, read in GPIO_STATUS under a foreign host's
burst, is OFF while the client's select is high and ON while it is low,
and under SOD it is off selected or not - the far host reading its own
pull-up where the RP2040's client repeats its first answer (bench
findings). The driver depends on neither: `SpiClient::drive_output
(false)` makes the dark listener by taking the pad away - to no function
with no pull, the traits' dark config, because the pad's reset state
carries a pull-down that fights the far host's pull-up - and `sod()`
stays a bit of the resource.

**The pads come up isolated** (9.11, and [README.md](README.md)): a pad
answers nothing until the isolation latch is cleared, which every
configuring verb of `pin.hpp` does as it writes. Two of the four signals
are handed over with a pull-up, because this chip's pads reset pulled
down: a host's receive pad, so an answer line with nothing driving it
reads as the idle ones a bus expects rather than as zeros, and a client's
select pad, so an unwired select reads not-selected. And because erratum
RP2350-E9 is live on the A2 stepping, an idle level on a pad of this chip
is history and not a measurement: whether a transmit pad is driven is
read where the pad sees it, in GPIO_STATUS's output enable
(`Pin<n>::status()`), and never from the level a wire holds.

**The three number tables are not the RP2040's**, and each has its own
reason. The DREQ rows are 24..27 where they were 16..19, because a third
PIO and its twelve rows moved everything above them (12.6.4.1,
`rp2350/dma_engine.hpp`). The reset bits are 18 and 19 where they were 16
and 17, the controller governing twenty-nine blocks against twenty-seven
(7.5). And the interrupt lines are 31 and 32 where they were 18 and 19
(3.2) - shared between the architectures (3.8.4.2), so an app binds
`isr_spi0` / `isr_spi1` once and is bound on both halves. The controller
behind those names is not the same object - an NVIC on the Cortex-M33
half, Hazard3's own on the other - which is exactly why the IP stratum's
concept takes the controller as a type.

## Types and verbs

What the IP stratum names (the mode and framing vocabulary, the prescaler
arithmetic, the register bit layouts, the resource's verbs, the host's
Request, `has_dma`, the short request's counts and its one interrupt
body, the DMA line's, the client's surface) is in
[../pl022/README.md](../pl022/README.md). This stratum adds:

- `SpiPins` - the four pad numbers, each the instance's own or `0xFF`
  for a signal the set leaves out. A host with no receive pad is a
  write-only bus; a host's transaction select is not here (it is a GPIO
  the Request carries) - a host whose set names `cs` drives the block's
  own frame signal there (`frame_select`,
  [../pl022/README.md](../pl022/README.md)) - while the client's select
  is its select pad.
- `spi_pad_instance(pin)` - which instance the four pads of that pin's
  group belong to: bit 3 of the pad number.
- `spi_rx_pin(n, pin)`, `spi_cs_pin(n, pin)`, `spi_sck_pin(n, pin)`,
  `spi_tx_pin(n, pin)` - table 645 as constexpr facts, in the table's own
  order.
- `spi_pins_valid(n, pins)` - a legal pin set for instance n. The package
  is not asked here: the table is the die's, and a pad the QFN-60 has not
  bonded is refused one level down by `Pin<n>`'s own static_assert, which
  says so in those words.
- `Rp2350Pl022` - the chip traits, which nothing above this file names.
  Its `pad_function` is the single code `PinFunction::spi`, and its
  `DelayRate` is [clock.md](clock.md)'s - the microsecond busy-wait of
  this target, which counts on the platform timer and not on a core
  counter, so a Request's `cs_setup_us` is served once `Mtime::start` has
  run and refused (with no time spent) before that.
- `Pl022<n>`, `SpiHost<n, pins, TxEngine, RxEngine, Placement>` and
  `SpiClient<n, pins>` - the public names, this chip's aliases of the
  three IP templates, with `NoDmaEngine` in both slots and `InFlash` by
  default (`Placement`: util/placement.hpp's tag, [platform.md](platform.md),
  "A handler in SRAM").
- `Rp2350Pl022::costs`, the figures the short request is drawn from, one
  set per architecture - E - F 230 for a read and 235 for a write on the
  Cortex-M33, 220 and 230 on Hazard3, the receive loop's turn 24 on the
  Cortex-M33 and 26 on Hazard3,
  the transmit-only loop's 16 (measured: "The short request" in the
  bench findings). The host's receive engine arms at HIGH_PRIORITY, a
  frame landing on a full receive FIFO being lost, its transmit engine
  at normal ([dma.md](dma.md)).

Beside them, at the foot of the file, is the vocabulary check: the IP
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
using Bus = brio::SpiHost<0, pins, brio::DmaTxEngine<4, uint16_t>,
                          brio::DmaRxEngine<5, uint16_t>>;
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
using Cs = brio::Pin<17>;

brio::Dma::init();                                    // before the host binds its engines
if (!Bus::init(clock, 8'000'000)) { /* the ceiling cannot be made, or the block did not come up */ }
Cs::output(true);

Bus::Request r{};
r.cs = Cs::ref();
r.cmd = brio::lend<brio::Lease::reply>(cmd);   r.cmd_len = 1;
r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
r.clock = brio::SpiClocks::div4;                      // 37.5 MHz at 150, clamped to the ceiling
r.mode = brio::SpiMode::mode0;
r.reply = brio::reply_to<Display, brio::SpiDone>();
r.engine = brio::SpiEngine::dma;
brio::post<Spi>(r);                                   // asynchronous

extern "C" void isr_dma_0() {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

The same source builds for both architectures; `isr_dma_0` is a
vector-table slot on one half and a dispatch entry on the other. The
engines are half-word engines, SSPDR's width, so 8-bit frames ride them
at a byte beat and 16-bit frames at a half-word one; byte engines leave
a 16-bit request to the polled loop inside `start()`, as does a buffer
not aligned to a half-word. The receive block's completion is the
block's one interrupt: the transmit engine is armed to report a bus
error and nothing else ([dma.md](dma.md)).

Without the engines, `SpiHost<0, pins>`: polled requests alone, each run
inside `start()` (`r.engine = SpiEngine::polled`), an asynchronous one answered
`spi_rejected`.

A client, whose select is a pad, its vector in SRAM (from the flash its first
entry after a quiet spell is the XIP cache's fill, frames late: the client's
vector, bench findings):

```cpp
constexpr brio::SpiPins client_pins{.sck = 10, .tx = 11, .rx = 8, .cs = 9};
using Client = brio::SpiClient<1, client_pins>;
Client::init(clock, {.mode = brio::SpiMode::mode1});
Client::interrupts(brio::SpiInterrupt::rx | brio::SpiInterrupt::rx_timeout, true);
Client::enable(first_answer);
extern "C" [[gnu::section(".fast_text")]] void isr_spi1() {
    (void)Client::isr();
    while (const auto f = Client::poll()) { heard(*f); }
    while (Client::writable() && have_answers()) { Client::write(next_answer()); }
}
```

### What each answer asks of the program

The answer reaches the requester as the `status` field of its `SpiDone`, in
the handler of `Display`:

```cpp
[](const brio::SpiDone& d) {             // a lambda of Display's brio::match
    if (d.status != brio::spi_ok) { /* below; on any status the buffers are the program's again */ }
    return handled();
},
```

| `status` | what happened | what it leaves | who acts |
|---|---|---|---|
| `spi_rejected` | the arbiter's FIFO was full, or a host with no engines refused an asynchronous request | nothing on the wire | the program: a full FIFO may be tried again, the refusal never (`static_assert(Bus::has_dma)`) |
| `spi_stalled` | a polled wait ran out of its budget | select released, then the block reset and reconfigured to the state applied, both FIFOs empty | nobody: the next request starts on a clean block. Remove the cause: a reset does not start a stopped clock |
| `spi_dma_fault` | a channel reported a bus error | requests down, both channels put away and bound again, select released, the block's reset owed to the next `start()`, frames perhaps still shifting until then with the select high | nobody: the arbiter answers and goes on, and the next `start()` resets the block before anything else |
| `spi_timeout` | a timed arbiter's limit passed with no completion | the arbiter has already called `recover()` | the program judges the device |

A stall holds the core for its budget, 200 000 turns of the wait a frame and one
frame more: 2 000 000 turns in 160 / 147 ms for nine frames (letter g, Cortex-M33 / Hazard3), every other active object
waiting under `Tenuto`. The block's reset then throws away what its two FIFOs hold
and costs the request that pays it some tens to a few hundred cycles warm (letter h). A request already in the arbiter's FIFO starts in the same dispatch
as a stall or as the answer to a fault, before the requester's handler can run, and
finds that clean block; the program calls nothing after either. `prime()` pays a reset
a fault left owed, for a caller framing its own select, so that it is paid before the
select falls; with a null select the program's own stands through a stall's reset, and
that caller raises it when `start()` answers. A reset does not
start a stopped clock: on a block whose clock stays off a queued polled request
stalls too, on its own budget, and an asynchronous one goes to the engines and has
no completion, `spi_timeout` on a timed bus and a bus busy for ever on an untimed
one. `recover()` stays for a timed arbiter's unanswered request and for a program
that wants it with the bus idle. No verb says the engines are idle, and `recover()`
under a request on them takes it away, its completion never coming, so a program
calls it when it knows: every request it has posted answered, on a bus with one
client.

## Bench findings

All of them on an RP2350 in the QFN-80 package, **stepping A2**, clk_sys
and clk_peri at 150 MHz, and all of them on BOTH architectures, from one
source: `test_rp2350_spi` carries 77 verdicts, 46 of the loop-back
letters a..h and 31 of nine letters on the pair's wire against the desk
partner's peer (the last group below), green on the Cortex-M33 pair and
on the Hazard3 pair. Where a
number differs between the halves both are given; every verdict reads
the same.

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
  **200 to 201 us on both halves** in ten runs each.
  The two transactions have to be told apart from the XIP cache's own
  first-run cost, which is worth some six microseconds on the Hazard3 half
  and under one on the other, so the letter warms the polled path before
  it weighs it.
- **THE LOOP-BACK ON THE ENGINES.** The four modes at 8 and 16 bits,
  sixteen frames each asynchronous, byte-exact with **ONE DMA interrupt**
  each; a two-frame command phase (polled inside `start()`, too short for
  a block of its own) then 32 data frames, the echo discarded; a read
  with no out buffer clocks 0xFF; an empty request completes on the spot.
- **THE POLLED LADDER.** 64 frames on the loop-back, Cortex-M33 then
  Hazard3: div2 25 to 27 / 25 to 28 us, div4 20 / 21, div8 35 to 37 / 36 to 37,
  div16 68 to 69 / 68, and
  the slower rungs the wire's own time. At div4 and slower the WIRE sets the
  pace on both halves; at div2 the receive loop's three accesses a frame
  do (the next item). Every rate is byte-exact.
- **THE HOST ABOVE THE WIRE, `bench_rp2350` letter e** - the polled loop
  in both its shapes and the price of a request - on a host with
  no engines, SPI0 on the same pads with GP17 the select and GP22 the D/C
  line, MISO floating, the receive-overrun flag read after every run and
  clear on every one; Cortex-M33 / Hazard3, the best of five, at clk_peri
  / 2, / 4 and / 16 in mode 0 and in mode 3:
  - THE RECEIVE SHAPE (`spi.poll.rx`, eight frames in flight - written and
    not yet read): 8-bit frames at 37.5 MHz **38.0 / 38.0 cycles a frame in
    mode 0 and 32.0 / 32.0 in mode 3** - the wire with the block's gap, and
    the wire - at 9.375 MHz 152 / 128, and 16-bit frames 70 / 64 and 280 /
    256: the wire's own figures to the hundredth. At 75 MHz its three
    accesses a frame set the pace: **24 / 26 cycles a frame** against a
    16-cycle frame, 16-bit frames at the wire (35 / 32).
  - THE WRITE SHAPE (`spi.poll`, no in buffer): at /4 and /16 the same
    figures - the receive loop discarding, since a frame read under the
    wire costs nothing - and at **75 MHz the TNF-paced loop at 18.98 / 19.02
    cycles a frame in mode 0 and 16.00 / 16.00 in mode 3**, the nominal
    wire, which is the one rate the transmit-only shape is taken at, on
    runs longer than the FIFO: its tail, up to eight reads after the wire
    has finished, is work the receive shape does under the wire instead
    ("Not covered yet" says what would measure it on a short request).
  - THE FRAME AGAINST THE WIRE AT EVERY DIVISOR (`spi.frame`: 8 and 16 bits,
    modes 0 and 1, a frame by the difference of two lengths, the least of
    five under the mask, the whole table measured before a line of it is
    printed): at clk_peri / 4 and slower every row of both shapes reads the
    wire with mode 0's gap, within 0.4 of a cycle a frame on both halves in
    three runs of the letter, with the overrun flag never raised and no bad
    status. At / 2, in core cycles, Cortex-M33 / Hazard3, the wire with the
    gap in brackets:

    | | mode 0 | mode 1 |
    |---|---|---|
    | read and stored, 8-bit | 24 / 26 (19) | 24 / 26 (16) |
    | read and stored, 16-bit | the wire (35) | the wire (32) |
    | no in buffer, 8-bit | the wire (19) | the wire (16) |
    | no in buffer, 16-bit | the wire (35) | the wire (32) |

    So the one rate and shape with dead bus is the 8-bit read at 75 MHz; a
    long one that wants the wire asks asynchronously, as does any request the
    other active objects must keep running through: on the engines 256
    frames in mode 3 take the wire and the fixed cost, where the loop takes 1.50
    / 1.63 times the nominal wire's time (larger being slower).
  - THE RECEIVE SHAPE UNDER A LONG HANDLER: 100 receive requests of 256
    frames on the loop-back at each of the three rates with the tick's
    handler stretched to 1500 cycles - eight frames being 128, 256 and
    1024 cycles - **100 exact and 0 overruns at every rate on both
    halves**, with 8 / 8 or 9 / 25 or 26 (Cortex-M33) and 7 or 8 / 8 or 9 / 25
    (Hazard3) of the stretched ticks landing inside the requests: the frames in flight
    never exceed the receive FIFO's depth, so a handler that holds the
    loop off cannot overrun it, whatever its length.
  - THE PRICE OF A DCS COMMAND (`spi.req`, a command byte with D/C low and
    0, 2 and 15 data bytes with it high, polled, nothing read back, at
    9.375 MHz): wall less the wire's cycles **195 / 174 for one byte,
    274 / 256 for three, 590 / 573 for sixteen** (the band across builds
    is the XIP cache's line alignment, which every link moves) - of which
    the block's mode-0 gap is 24 / 72 / 384 and the phase boundary's drain
    (the data frames wait for the command frame to come back whole, so
    D/C flips on the wire) one frame's latency; the host's own code under
    200. In mode 3 the same three take the same loop with no gap: 187 / 168, 280 /
    249 and 596 / 569.
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
- **THE SHORT REQUEST** (`test_rp2350_spi` letter f, the IP stratum's
  page, "The short asynchronous request", has the rule). An asynchronous
  request rides the engines from the smallest frame count whose polled
  cost reaches the engines' fixed cost beyond the polled loop's, E - F,
  and runs polled inside `start()` below it. E - F MEASURED AS ONE
  DIFFERENCE THROUGH ONE CALL, the suite's own `xfer()`, so the call's
  cycles cancel: E the 32-frame asynchronous block's wall less its 32
  frames on the wire, F an 8-frame polled request's wall less 8 frames at
  the loop's pace (the difference of a 24- and an 8-frame request), both
  at clk_peri / 4 in mode 3, the best of fifteen in three rounds 3 ms
  apart; Cortex-M33 / Hazard3, sixteen runs each: **E 474 to 481 / 508 to 512,
  F 245 / 291 to 292 and E - F 229 to 236 / 217 to 221 for a read; E 476 to 483 /
  516, F 243 to 245 / 285 and E - F 233 to 240 / 231 for a write**; the two loops'
  turns at clk_peri / 2 on the host with no engines, 24.0 / 26.0 received
  and stored and 16.0 written on TNF - the wire's own frame, so the
  transmit-only loop's turn is no longer. The walls are net of the XIP
  cache's misses, which the letter prints beside them ([the RP2040's
  page](../rp2040/spi.md), "The short request", says why; these images
  counted none in those runs). The traits carry 230 and 235 on the Cortex-M33, 220
  and 230 on Hazard3, and the turns 24 / 26 and 16 (`Rp2350Pl022::costs`);
  letter f judges each within a quarter of its measurement. The counts they
  draw, judged at their edge in ten runs on each half (one frame under the
  count inside `start()` with no DMA interrupt, the count itself on the
  engines with one, every read byte-exact): a read rides the engines from
  **10 / 9 frames at 75 MHz in mode 3, 7 / 6 at 37.5 MHz in mode 0 and 4 / 3
  at 18.75 MHz in mode 0**, a write from 15 at 75 MHz on both. A command
  phase of 15 frames at 75 MHz repays a
  block of its own and is chained to its data block from the vector, two
  DMA interrupts; a one-frame command runs polled ahead of the data
  block, one.
- THE REFUSAL (letter f, both halves): an asynchronous request on the
  host with no engines comes back from `start()` at once with
  `spi_rejected` - the select never driven, the control word as it was
  though the request asked mode 3 over mode 0, both FIFOs empty and the
  block idle, the in buffer untouched, no interrupt - and the same
  request polled runs exact.
- **THE ENGINES ON THE LOOP-BACK.** 128 bytes at 37.5 MHz, ISR-completed,
  are exact with **ONE DMA interrupt**, and 64 16-bit frames ride the
  engines at a half-word beat exact with no SPI interrupt and one DMA
  interrupt; a 16-bit request on a buffer one byte off a half-word runs
  polled inside `start()` and is exact there, no interrupt of either
  line. A command frame polled inside `start()` then data on the engines,
  a polled read of 64 frames at 75 MHz in mode 3 on the same host, the loop
  inside `start()` with no interrupt of either line, and a read with no out
  buffer through the transmit engine's fixed 0xFF cell are all exact.
  The letter times the 128-byte block, the polled read and the 16-bit
  block twice each - the first run and a repeat - with the XIP cache's
  misses beside each, over fifteen runs on each half: the block takes **57
  to 130 / 60 to 114 us cold, missing 41 to 182 / 39 to 181 lines, and 38 to
  46 / 37 to 44 us warm**, where the wire with mode 0's frame gap is 32
  (the next items); the polled read, a command frame and 64 data frames
  at 75 MHz, 26 to 33 / 30 to 36 us cold and 15 to 18 / 16 to 18 warm,
  against a wire of 7; the 16-bit block in mode 3, 53 to 64 / 46 to 60
  cold and 32 to 38 / 33 to 37 warm against a wire of 27. The repeat still
  misses up to 15 lines: the
  cache is two-way, and some lines of the image's transaction path
  share a set.
- **WHAT AN ENGINED TRANSACTION COSTS, `bench_rp2350` letter d**, the
  ruler TIMER1 on clk_sys, MISO floating (and the same on the loop-back),
  the core asleep between the start and the completion; Cortex-M33 /
  Hazard3. `start()` returns after **252 / 209 cycles** - the request's
  words copied into the tenure, one compare for the rate, the
  short-request fold, the select, both engines started - and the
  transaction's ONE interrupt follows the last frame; with it a
  transaction costs the core **614 / 378 busy cycles** whatever its size,
  and its fixed cost, the wall less its frames' own time, is **520 / 490
  cycles at 16 frames (3.5 / 3.3 us), of which 48 the block's gap**, the
  same at 256 frames and at both rates. THE FRAMES' OWN TIME IS NOT THE NOMINAL
  WIRE'S: in mode 0 the PL022 raises its frame signal between two frames
  of a continuous transfer (12.3.4.10, and 12.3.4.12 for mode 2), and the
  gap is 1.5 SCK periods - measured 19 cycles a byte at 75 MHz where the
  bits are 16, 76 at 18.75 MHz where they are 64, 35 and 140 a 16-bit
  frame where they are 32 and 128, 1.19 and 1.09 times the bits' time - so with
  the fixed cost 256 frames come out at 1.10 to 1.30 times the nominal wire's. IN
  MODES 1 AND 3 (SPH = 1) THERE IS NO GAP - measured on a build of the
  same binding in the four modes, warm, by the difference of 16 and 256
  frames: 16.00 and 64.00 cycles a byte at 75 and 18.75 MHz, the nominal
  wire exactly, on both halves, where modes 0 and 2 give 19.00 and 76.00.
  The same by `test_rp2350_spi` letter d against THE RATE ASKED (a frame
  inside a block, from the difference of a 2048- and a 256-frame
  asynchronous block, against clk_peri over the divisor): 999 to 1004
  thousandths on the Cortex-M33 and 999 to 1001 on Hazard3 at clk_peri / 4, / 16
  and / 64 in mode 3, in mode 0 with its gap and in 16-bit frames, fifteen runs
  on each half.
  256 16-bit frames at 75 MHz take 9435 / 9409
  cycles, 1.15 times the nominal wire's, with ONE interrupt; the same request on byte engines,
  or from a misaligned buffer, runs polled inside `start()` at the
  loop's 35 cycles a frame, the core held for it. The core spinning on a
  flag in RAM or on a register behind the APB bridge while the engines
  run changes a 256-byte block by under a hundredth.
- **THE FIRST TRANSACTION PAYS THE XIP CACHE ITS FILL, AND WHERE THE
  HOST'S HOT PATH RUNS IS THE PROGRAM'S CHOICE.** A line of 8 bytes costs
  66 to 82 cycles at the boot's XIP setting (EBh at CLKDIV 3,
  [flash.md](flash.md)) - the price the vendor's library pays too (the
  next item) - and a cold transaction fetches every line of the code it
  runs, while a rate change costs 90 cycles (`apply()`'s SSE pair around
  `configure()`) and the DMA's first block after `arm()` and the PL022's
  first frame after SSE cost nothing measurable, split on a build of the
  bench's binding with the cache's two counters (`Xip::hits()`,
  `Xip::accesses()`) read around each transaction. `SpiHost`'s last
  template parameter is util/placement.hpp's tag: `InFlash`, the
  default, leaves `start()` and the DMA line's body in the flash, `InRam`
  makes each ONE flattened instance in `.fast_text.brio`,
  the engines' verbs and the launch folded in ([platform.md](platform.md),
  "A handler in SRAM", the table and the rule for both architectures).
  `bench_rp2350` letter e's `spi.cold` is the instrument: `spi.req`'s
  three-byte polled request with every line of the cache invalidated
  first (`Xip::invalidate_all()`), under the mask so that no tick's
  handler lands in the one shot, the counters cleared before the start
  and read after the wall - **4956 / 4524 cycles missing 69 / 66 lines
  with the host in the flash (Cortex-M33 / Hazard3), 830 / 742 missing
  5 / 5 with it InRam, against 658 to 768 / 636 to 860 warm from the flash and
  664 / 642 to 809 InRam**; the few left are the
  caller's (the call site with the ruler's reads around it, and on the
  Cortex-M33 the veneer). THE WARM PRICE of InRam is the call: on the
  Cortex-M33 a `bl` reaches 16 MB and the SRAM is 256 MB from the flash,
  so a call across goes through a stub the linker inserts (`ldr.w pc,
  [pc]` and a word), a request 6 to 10 cycles dearer in `spi.req`; on Hazard3
  `auipc` and `jalr` reach, and the request reads 8 to 9 cycles dearer. The
  engined block cold (letter d after letter c, 16 frames) from the flash 13452 /
  10371 cycles, with the host InRam 3978 / 4022, with the vectors placed 8765 /
  7188, with both 1718 / 2619, against 743 to 812 warm.
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
  776 / 746**: its transfer is two `dma_channel_configure()` calls and
  one store, inlined, where this one carries the arbiter's Request
  through `start()`. And its first transfer after `spi_init()`,
  completed by its interrupt, pays the same price per line - 14 to 24
  misses, 1158 to 1996 cycles - because it touches six to seven times
  fewer lines; this host's first transaction after `init()` pays 11315 /
  8527.
- **A POLLED WAIT THAT RUNS OUT** (`test_rp2350_spi` letter g, both
  halves): the block's enable taken down under a polled write of nine frames
  at clk_peri / 2 with three frames left standing in the receive FIFO, so
  the transmit FIFO fills and the loop's budget runs out, 2 000 000 turns in
  160 / 147 ms: `spi_stalled`, the select released, both FIFOs empty and the
  block enabled at the word applied. With no `recover()` the next request is
  exact, polled on the host with no engines, polled and asynchronous on the
  host with them. A command frame polled ahead of an engined block, its enable
  down and its budget 400 000 turns, answers the same inside `start()` with no
  block launched. Through `SpiBus`, two requests from one dispatch, the first
  stalled: the replies `spi_stalled` then `spi_ok` in that order and the second's
  data exact, polled and on the engines. Thirteen runs on each half. That the
  select goes up before the reset is judged on the simulated chip (`test_pl022`);
  no letter here reads the SCK pad across it.
- **A DMA BLOCK THE BUS REFUSES** (`test_rp2350_spi` letter h, both halves): an
  asynchronous request of 64 frames at clk_peri / 128 with its buffer where the fabric
  fails to decode the address (12.6.7): the out or the in buffer at 0xD0000000, the
  SIO's window, which that section says the DMA's bus ports cannot see, and an out
  buffer 16 frames short of the SRAM's end, a read failing with frames already on their
  way. A bus error on the channel and never a fault of the core. `dma_isr()` answers
  true, one interrupt, `spi_dma_fault`, the select released. With frames on their way
  the block is still enabled and its transmit FIFO not empty afterwards, which is what
  the interrupt left for the next `start()`. With no `recover()` that request, polled
  or asynchronous, is exact, no frame of the faulted block in front of it; through
  `SpiBus`, queued behind the read off the end, `spi_dma_fault` then `spi_ok` and its
  data exact both ways. Thirteen runs on each half. It paid the reset first, and what
  that costs it is not pinned: a difference of two walls taken with interrupts on and
  no XIP count beside it, reading most often 95 / 148 in this image and 155 / 148 and
  163 / 150 in two other layouts of this suite, 49 to 273 / 31 to 260 at the extremes.
  A WRITE THE BUS REFUSES
  LANDS NOWHERE IN THE SRAM: 32 half-words of a mark written at the hole, `spi_dma_fault`,
  and across it no kilobyte of the 520 holds the mark once more than it did, 13 runs of
  13 on each half; the same count with the write aimed inside the SRAM finds all 32 in
  their block. A landing in a peripheral it could not see.
- **A WINDOW FRAMED BY HAND AFTER A FAULT** (letter h on the loop-back, letter v on
  the pair's wire): the select null in the Request, `prime()`, the select low,
  `start()`, the select high, behind a fault that left frames on their way, with no
  `recover()`. On the loop-back both FIFOs are clean when `prime()` returns, before the
  select falls, and the window is exact, polled and asynchronous. On the wire the
  foreign client reads sixteen frames exact and answers sixteen exact, in the four
  modes, the fault taken with the select high. Thirteen runs on each half. Without
  `prime()`'s payment the window read wrong both ways in all four modes on both halves, 7
  to 16 of the 16 frames.
- **The arbiter over it, on both halves.** `util/spi_bus.hpp` and
  `util/bus_master.hpp` carry four transactions over the engined host,
  on the engines and polled interleaved, reject the fifth of a four-deep
  queue while answering all
  six exactly once, and vote for a sleep when idle and against one when
  busy - on both architectures, with no line changed for either.

ON THE PAIR'S WIRE (letters n, o, p, q, r, m, s, w and v, the two Raspberry
Pi suites' shared letters, against the WeAct RP2040's peer: its client a
PIO program at 125 MHz, its host that chip's PL022), every figure the
same on both halves:

- THE HOST AGAINST A FOREIGN CLIENT, every window ONE GPIO-held select
  of sixteen frames: the four modes, the client MSb and LSb first (an
  exact two-way bit reversal), 8- and 16-bit frames, both of the
  client's serves - 32 exchanges exact both ways; receive-only (the
  client's answer line dark, this host reading its own pull-up) and
  answer-only exact; the command channel at clk_peri / 128 (1.17 MHz)
  ten pings of ten; noise under a held select wakes nothing.
- THE LADDER AND THE ENGINES ON THE WIRE: the polled host (32 frames) and
  the half-word engines on channels 6 and 7 (64 frames, each window an
  asynchronous request) **exact both ways
  at clk_peri / 14 (10.7 MHz, a half period of 46.7 ns) and slower**; at /12 (12.5
  MHz) this host reads the answers wrong while the client hears it exact
  at /6 (25 MHz) and slower - the answer path, four of the client's cycles (32 ns)
  and the pads, is the bound. Judged at /16 and slower, the derived ceiling (a half
  period of four of the partner's cycles and 15 ns).
- THE KERNEL against the second chip: four transactions in one held
  window, three on the engines and one polled, every byte read back by
  the peer.
- THE ROLES INVERTED: this chip's `SpiClient` under the RP2040's PL022
  host, its select that block's own frame signal: 64 frames back to back
  in every mode, 8- and 16-bit, exact both ways at /64, /32 and /16 of
  the host's clk_peri (1.95, 3.91 and 7.81 MHz); a window a frame in
  modes 0 and 2
  and one for the burst in modes 1 and 3, counted at this end by a PIO
  machine; the frame signal's high pulse half an SCK period (53 ns at
  /16, 120 ns at /32); the burst at the wire's time.
- **AT /16 OF THE RP2040 (7.8 MHz) THIS CLIENT'S OWN ANSWER EDGE TOOK
  FRAMES AWAY, until a series resistor at its pad.** On the bare wire,
  in 16-bit frames in modes 0 and 3 - the two whose data lines change on
  SCK's falling edge - a burst of 64 received 63 and both ends read wrong
  from that frame on: 22 bursts of 278, 8 %; none of 238 in modes 1 and
  2, none of 156 in 8-bit frames, none of some 940 at /32 and /64. In
  some of them this end's PIO gauge caught a high pulse of two machine
  cycles (13 ns) on the select, far shorter than any the frame signal
  makes; with the answer line RELEASED the same mode-3 bursts lost
  nothing (0 of 40, against 2 of 40 driven in the same run), and the
  RP2040 as the client under this chip's host at /16 lost nothing in
  280. The aggressor was this end's own answer line, wire 3, switching
  beside SCK and the select. With 47 Ohm in series at this end of wire 3
  (the board page's wiring) the edge is slowed at its source: 0 of 320
  bursts not exact in the cases that lost 22 of 278 (letter x, four
  runs over both halves, mode 3 with the answer line driven 0 of 40 in
  each), the short pulse gone (the shortest high pulse 8 machine
  cycles, the frame signal's own), and letter r judges /16 as it judges
  /32 and /64 - the three rungs this client admits, its clk_peri at
  least twelve times the far host's SCK.
- THE CLIENT FROM ITS VECTOR, TO THE CEILING (letter s, both halves):
  the client served by its interrupt instead of a loop - RXIM at the
  receive FIFO's half, RTIM, RORIM; every entry drains the receive FIFO
  and refills the transmit FIFO to its depth - 67 frames in modes 0 and
  3, 8- and 16-bit, exact both ways at every even division of the
  RP2040's 125 MHz from /64 down to /10 (12.5 MHz, this client's
  clk_peri / 12, the last rung it admits), ten runs on each half: one
  entry per four frames where the frames are slow enough, the three
  under the half delivered by ONE receive-timeout entry at /64. Past the
  ceiling, /8 (15.6 MHz), the far host read the client's very first
  answer wrong in every setting of every run. The body and the suite's
  vector run from SRAM, for the reason the RP2040's document gives (a
  cold entry from the flash is the XIP cache's fill; on Hazard3 the crt's
  dispatch in front of the vector stays in the flash); here the letter's
  first burst, printed, was exact in all twenty runs.
- THE CLIENT'S OVERRUN (letter s): eight answers preloaded, nothing
  taken, 24 frames from the far host - the receive FIFO keeps the first
  eight exact and the vector is entered on RORIM sixteen times, once a
  frame lost; the far host reads the eight preloaded answers exact.
- THE ARBITER'S PER-BUS TIMEOUT (letter w): a completion that never comes
  (the DMA line's body run, nothing posted) is answered `spi_timeout`
  99.1 to 99.9 ms after the request against 100 ticks, and recover()
  leaves the engine where the next four transactions on the same bus AO
  - three on the engines, one polled - run to `spi_ok`; ten runs on each
  half.
- THE CLIENT'S ANSWER PAD (letter m, its output enable read in
  GPIO_STATUS): OFF while deselected, ON while selected, OFF under SOD
  selected or not - the far host reading its own pull-up, 0xFF x 32 -
  so nSSPOE does reach the pad here (12.3.1), the RP2040's opposite. The
  dark listener takes the pad away and the far host reads its own
  pull-up, the client hearing every frame. Taken to the pad's RESET
  state instead, the pad's pull-down fights the far host's pull-up and
  the host reads 0x02 and the like (measured: sum 0xEBE where a steady
  0xFF gives 0x1FE0), which is why the driver gives it to no function
  with no pull.

## Not covered yet

Driver gaps, each with its reason. The ones that belong to the block
rather than to this chip - the Microwire transaction, frame widths other
than 8 and 16 in the Request, a client on the DMA engines - are in
[../pl022/README.md](../pl022/README.md) and are not repeated here.

- The TI and the Microwire framings ON THE WIRE: both are codes
  `SpiConfig` takes and the resource reads back, and no device on this
  desk speaks either. The desk partner's PL022 could speak TI to this one
  on the pair's wires; the letter that would do it is born with a reason
  to.
- A second device on one bus: the Request's per-transaction mode, rate
  and width exist so that two devices can share a host, and the desk
  carries one peripheral instance and no third party, so nothing measures
  the reprogramming between two selects.
- The D/C line of the Request: it is carried and driven, and what makes
  it worth a letter is a display, which this desk has not got.
- Detaching clk_peri from clk_sys: `Clock`'s `PeriSource::crystal` is
  what would hold the bit rate still across a clk_sys change, and this
  target has no dynamic clock, so nothing changes clk_sys under a
  running bus.

Implemented but not bench-verified, each with what would measure it:

- A stall on a stopped clock: every stall the suite stages is a block whose enable
  went down, which its reset cures. What the reset does with `clk_peri` stopped,
  and what the next request answers, a letter that stops it around a request with
  the console quiet would measure.
- What the transmit-only shape's tail costs a short request: `run_polled` built
  to take that shape on every write, beside letter e's `spi.req`, would measure it.
- Where this driver's 776 / 746 goes against the vendor's 405 / 452: stamps
  through `start()` and the completion's handler in the same transfer would split it.
- The QFN-60's pin table and its compile-time refusals: the stratum
  compiles for that package and refuses the pads it has not got, and no
  QFN-60 part is on the bench. A board carrying one, running the suite's
  letter a against a pin table of thirty pads.
