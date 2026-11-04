# SPI (RP2040)

This document is for a programmer who has an SPI device on an RP2040 -
a display, a converter, a flash chip - and wants to talk to it through
the bus arbiter ([design/spi-bus.md](../design/spi-bus.md) is the
model), or who wants this chip to answer as an SPI client. The chip's
two SPI blocks are ARM's PL022, so the driver is written once in the IP
stratum (`brio/pl022/`) and this stratum supplies the chip's part and
the public names: `Pl022<n>` the instance, `SpiHost<n, pins, ...>` the
host engine `util/spi_bus.hpp` drives, `SpiClient<n, pins>` the client
role. The arbiter, `SpiBus`, is an active object (a component of a
brio program with its own event queue, served one event at a time by
the kernel, [docs/learn/introduction.md](../learn/introduction.md)):
devices post it requests, it runs them one at a time on the host and
replies to each.

The smallest use, a device on the bus through the arbiter, from "How
to use it" below:

```cpp
constexpr brio::SpiPins pins{.sck = 18, .tx = 19, .rx = 16};
using Bus = brio::SpiHost<0, pins, brio::DmaTxEngine<4, uint16_t>,
                          brio::DmaRxEngine<5, uint16_t>>;
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
using Cs = brio::Pin<17>;

if (!brio::Dma::init()) { /* the DMA block did not come out of reset */ }  // before the host binds its engines
if (!Bus::init(clock, 8'000'000)) { /* the ceiling cannot be made, or the block did not come up */ }
Cs::output(true);

Bus::Request r{};
r.cs = Cs::ref();
r.cmd = brio::lend<brio::Lease::reply>(cmd);  r.cmd_len = 1;
r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
r.clock = brio::SpiClocks::div4;                      // asks 31.25 MHz; the ceiling makes it 7.8125
r.mode = brio::SpiMode::mode0;
r.reply = brio::reply_to<Display, brio::SpiDone>();
r.engine = brio::SpiEngine::dma;
brio::post<Spi>(r);                                   // to the arbiter; asynchronous

extern "C" void isr_dma_0() {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

This is an excerpt: it assumes the program's platform `P`, its clock object
`clock`, an active object `Display` that handles `SpiDone`, the buffers
`cmd` and `pixels`, and the statements where they belong, the setup in
`main()` and the post in a handler.

Line by line: `pins` names SPI0's clock, transmit and receive pads,
GP18, GP19 and GP16, which table 279 gives the instance (a pin it does
not give does not compile); `Bus` is the host engine over SPI0 with two
DMA engines, channels 4 and 5, half-word wide (the data register's
width, so both frame sizes ride them) and reporting on the DMA
controller's line 0, and `Spi` the arbiter over it, four requests able
to wait while one is in flight (a further one is answered `spi_rejected` at
once), listed in the program's `Tenuto` like any active object, with no
timeout, so a completion that never comes would leave the bus busy for ever (a
limit is its fifth argument, `brio::SpiBus<Bus, P, 4, brio::BusPassThrough,
brio::ticks_from_ms<P>(20)>`); `Cs` is
the device's select, a plain GPIO driven high (deselected) before the
first request, since the PL022's own select pulses between frames in
modes 0 and 2; `Dma::init()` releases the DMA controller the host's
`init()` binds the engines on; `init()` takes the clock object
(`constexpr SysClock clock;`, [clock.md](clock.md)) and a ceiling for every request on the bus - here
8 MHz, so this request's `div4`, 31.25 MHz at a 125 MHz clk_peri, runs at
the ceiling's setting, 7.8125 MHz; the request names the select, a
one-byte command, 64 data bytes, the clock divisor, the mode, and the
active object the reply goes to (`Display`, which receives an
`SpiDone`); mode 0 puts a gap of 1.5 SCK periods between frames on this block
(the frame gap, bench findings). It names the DMA engines, `SpiEngine::dma`:
its 64 data frames ride them, the core free for their wire time, and the one
command frame goes out in the loop inside `start()` first, as a command ahead
of a data phase always does; `isr_dma_0` - the DMA line's handler under the
pico-sdk's name - tells the arbiter when the transaction completes. The command goes out with a D/C line low and the
data with it high only when the request names that pin in `r.dc` (a
pin's `ref()`, as `r.cs` takes); left at its default, no pin, as here,
the host drives no D/C line and the command is simply the first byte.

What decides correctness even there. The request lends its buffers
(`brio::lend<Lease::reply>`): `cmd` and `pixels` belong to the
transaction from the post until `Display` receives its `SpiDone`, and
stay untouched and in scope until then, which nothing checks. `init()`
after the clock's `SysClock::init()`: the divisor is computed from the rate the clock
type claims for clk_peri, and clk_peri is off until `SysClock::init()`
enables it. The engines need `Dma::init()` before the host's `init()`, which binds them
and enables the DMA line in this core's NVIC ([dma.md](dma.md)). The DMA line's vector is the program's to bind: the
crt gives `isr_dma_0` a weak alias to `Default_Handler`'s spin, so an
unbound vector parks the core there at the engines' first completion,
and the arbiter never hears of one. A host with no engines
(`SpiHost<0, pins>`, both engine parameters at their default) runs polled requests
alone: one naming dma is refused at once, `spi_rejected` and nothing
on the wire, and `static_assert(Bus::has_dma)` makes that a compile
error in a program that posts one. A request naming dma that the engines
cannot carry is refused the same way, at once and with nothing touched:
16-bit frames on byte engines, and 16-bit frames with a buffer off a
half-word on half-word engines such as these
([../pl022/README.md](../pl022/README.md) has the rules in their order). A
request naming `SpiEngine::polled` runs an explicit
loop inside `start()`, in the arbiter's dispatch, for its whole wire time,
holding every other active object meanwhile, with interrupts enabled; at any
length and with engines bound or not, so it needs no vector. A polled wait that
runs out answers `spi_stalled` and a DMA block the bus refuses `spi_dma_fault`,
each with the select and the D/C high, and the next request starts on a clean block
with nothing asked of the program ("What each answer asks of the program"
below, where the other answers the arbiter can bring are too). When a request
ends, on either engine, its wire has ended: in modes 0 and 2 that is a wait
for the block's BSY, up to one SCK period, and on the engines that wait is in
the DMA line's handler, every other interrupt held off for it (bench findings,
"When a request ends").

The page is long because the chapter is. In order: the documents of
record; what the silicon does, with the PL022's two facts as a client;
the types and verbs; how to use it - the host, the engines, a client,
a host whose select is its own frame signal; the bench findings, the
loop-back first, then the cost of the host above the wire, which engine
costs the core less, the engines, the wire's end, the placement, and the
pair's wire; and what is not covered.

## Documents of record

The RP2040 datasheet (build 3184e62), 4.4 (the
ARM PL022: 4.4.2 the functional description - the prescaler, the two
FIFOs, the DMA interface -, 4.4.3 the operation - the clock ratios
of 4.4.3.4, the bit rate of 4.4.3.6.1, the frame formats of 4.4.3.7
-, 4.4.4 the registers), 2.19.2 (table 279: the pins), 2.15.3.1
(clk_peri); docs/design/spi-bus.md for the Request and the two engines a
request names. The driver is ARM's and not this chip's, so it lives
in the IP stratum: `brio/pl022/spi.hpp` holds the resource, the host
engine and the client ([../pl022/README.md](../pl022/README.md)), and
`brio/rp2040/spi.hpp` holds what this chip owes it - the pin table of
table 279, the `Rp2040Pl022` chip traits over `pin.hpp`, `resets.hpp`,
`delay.hpp`, `nvic.hpp` and `dma_engine.hpp`, and the public names
`Pl022<n>` (the resource), `SpiHost<n, pins, ...>` (the engine `SpiBus`
drives) and `SpiClient<n, pins>` (the other end), this chip's aliases of
the three templates there. The reference suite: `test_rp2040_spi`, on
the loop-back; the wire is the desk partner's, SPI1 on GP10 / GP11 /
GP8 with GP29 its select, to the WeAct RP2350B
([../boards/weact-rp2040.md](../boards/weact-rp2040.md)), the board
wired to this one. A suite's letter is one command key of its console
and the group of verdicts it runs.

## What the silicon does

Two PL022s on clk_peri, each a host or a client in Motorola SPI, TI
or Microwire framing, frames of 4 to 16 bits, two FIFOs of eight
frames. The bit rate is clk_peri / (CPSDVSR x (1 + SCR)), the
prescaler even in 2..254 and SCR in 0..255: 62.5 Mbit/s at the top of
a 125 MHz clk_peri for a host; a CLIENT needs clk_peri at least
twelve times its clock (the input is double-synchronized), 10.4
Mbit/s here. Control registers are written with SSE clear. Four
interrupt sources on one line per instance: the receive FIFO at or
above half, the RECEIVE TIMEOUT (a frame waiting and no clock for 32
bit periods), the transmit FIFO at or below half, the receive
overrun; the last two clear by writing, the first two by moving
data. A DMA request per FIFO. LBM loops the transmitter into the
receiver with nothing on the wire. The PL022's own select output,
SSPFSSOUT, pulses between frames in Motorola format - which is why a
multi-frame transaction under one select is a GPIO's business - and
the PL022 as a client frames on SSPFSSIN. The pins are fixed per
instance under function 1: SPI0 clocks on GPIO 2, 6, 18, 22,
transmits on 3, 7, 19, 23, receives on 0, 4, 16, 20, selects on 1, 5,
17, 21; SPI1 on 10, 14, 26 / 11, 15, 27 / 8, 12, 24, 28 / 9, 13, 25,
29.

Two facts of the PL022 as a client. With SPH = 0 (modes 0 and 2) the
select freezes the slave's shift register, so the host must raise
SSPFSSIN between every two frames (4.4.3.10, 4.4.3.12) - a host holding a
GPIO select for a whole transaction reaches a PL022 client for more than
one frame only in modes 1 and 3, and a host whose select is the block's
own frame signal reaches it in all four, the signal pulsing between the
frames in exactly those two modes. And nSSPOE is not connected to the pad
here (4.4.3.14's list of idle levels): a client's transmit pad stays
driven while it is deselected and under SOD, the host then clocking the
client's first answer in every frame (bench findings). The dark listener
is therefore the pad, taken away - to no function with no pull, not to
its reset state, whose pull-down would fight the host's pull-up.

## Types and verbs

- `SpiMode` (the four Motorola modes), `SpiFormat` (motorola, ti,
  microwire), `SpiClock` (`cpsdvsr`, `scr`; `divisor()`, `valid()`) with
  `SpiClocks::div2 .. div256` the named divisors the other strata's
  requests speak, `spi_sck_hz(pclk, clock)`, `spi_clock_for(pclk,
  max_hz)` (the fastest setting under a ceiling, nothing under the
  slowest), `SpiDataSize` (bits8, bits16: what the Request speaks;
  the resource takes 4..16), `SpiRole`, `SpiPins` (`sck`, `tx`, `rx`,
  `cs`, each the instance's own or none) with `spi_sck_pin` and the
  three others as table 279 and `spi_pins_valid`, `SpiConfig` (role,
  mode, format, bits, clock, loop-back, output disabled) with
  `spi_config_valid`, `spi_cr0_of` / `spi_cr1_of`, `SpiControl0`
  (SSPCR0), `SpiControl1` (SSPCR1, with the enable and the two live
  bits), `SpiDataField` (the prescaler's and the data register's
  masks), `SpiFlag` (SSPSR), `SpiInterrupt` (one layout for IMSC/RIS/
  MIS/ICR), `SpiDmaControl` (SSPDMACR). `SpiPins` and its four pin
  predicates are this chip's; everything else on this list is the
  PL022's and lives in the IP stratum.
- `Pl022<n>`: `reset` / `hold` / `released`, `enable(on)`, `configure`
  (refused while enabled), `loopback` and `output_disabled` live, the
  rate and width read back, the flags, `write_data` / `read_data` /
  `flush_rx`, the interrupt trio through the atomic aliases, `isr()`
  (the raised-and-enabled sources, the two clearable ones cleared: the
  client's), `dma_requests`, `dreq_tx` / `dreq_rx`, `irq()`.
- `SpiHost<n, pins, TxEngine, RxEngine, Placement>`: the engine `SpiBus`
  (= `BusMaster`) drives. Its `Request` is the other strata's: a chip
  select `PinRef` and a D/C line, a command phase (D/C low), a data
  phase with an optional out buffer (null = 0xFF dummies) and an
  optional in buffer (null = discard), the per-request `mode`, `clock`
  and `bits`, `cs_setup_us`, and `engine` - `SpiEngine::polled` spins inside `start()`,
  in the receive shape with eight frames in flight, discarding when there is
  no in buffer, and on one case alone (no in buffer, 8-bit frames, a run longer
  than the FIFO, a divisor under four) in a write shape that paces on TNF and
  reads nothing (the IP stratum's page, "The host above the wire");
  `SpiEngine::dma` runs on the DMA engines at any length, refused
  (`spi_rejected`) on a host with none and for what they cannot carry. The
  request is 28 bytes here, laid out words first. `init(clock, max_sck_hz)`,
  `rebase(pclk, sys)`, `clock_for(hz)`, `sck_hz`, `ceiling_clock`,
  `prime()` for a caller framing its own select, `frame_select` (the
  pin set names `cs`: the block's own frame signal drives that pad - a
  select a frame in modes 0 and 2, one a run of back-to-back frames in
  modes 1 and 3 - and a Request's `cs` stays null), `loopback(on)` (kept
  through every re-application: the wireless instrument), `dma_isr()`
  (the host's one interrupt body: it arms no source of SPI0's or SPI1's
  own line), `status()`, `recover()`, `release()`, `has_dma` (engines
  bound: a request naming dma can run). The engine parameters take `dma.hpp`'s `DmaTxEngine` /
  `DmaRxEngine` on any two channels, both or neither, both of one
  element: `uint16_t` - SSPDR's width - carries 8- and 16-bit frames
  alike, each block at its frame's beat, and `uint8_t` engines refuse
  16-bit frames. A request on them moves as one block each way with the
  requests raised around the engines - its data phase, or a command with no
  data phase after it - a command ahead of a data phase running in the loop
  first; the transmit engine is bound to report a bus error alone
  (IRQ_QUIET), so the receive block's completion is the request's ONE DMA
  interrupt. The receive engine arms
  at HIGH_PRIORITY, a frame landing on a full receive FIFO being lost,
  the transmit engine at normal ([dma.md](dma.md)). A 16-bit request
  naming dma whose buffers are not half-word aligned is refused.
  Frames in a byte buffer: one byte per 8-bit frame, two bytes
  low-first per 16-bit frame - on this little-endian core, a half-word
  in memory.
- `Placement`, the last parameter: util/placement.hpp's tag, `InFlash`
  by default, `InRam` making `start()` and the DMA line's body one
  flattened instance each in SRAM ([platform.md](platform.md), "A
  handler in SRAM").
- `SpiClient<n, pins>`: `init(clock, Config)` (mode, format, bits,
  drive_output), `enable(first)` with the first answer in the FIFO
  before the host's clock, `write()` on TNF up to `frames_ahead` = 8,
  `poll()`, `selected()` read off the select pad, `drive_output(on)`
  (the transmit pad handed over, or taken away to no function and no
  pull: the dark listener),
  `sod(on)` (the block's own bit, for a program that wants to see
  it), the overrun flag, `isr()` and the interrupt enables.
- `pin.hpp`'s `PinRef` (a pin named at run time: set, clear, toggle,
  read; `Pin<n>::ref()`); `Pin<n>::status()` (GPIO_STATUS: what reaches the pad, the
  output enable among it) is how the bench reads a pad's drive.

## How to use it

```cpp
constexpr brio::SpiPins pins{.sck = 18, .tx = 19, .rx = 16};
using Bus = brio::SpiHost<0, pins, brio::DmaTxEngine<4, uint16_t>,
                          brio::DmaRxEngine<5, uint16_t>>;
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
using Cs = brio::Pin<17>;

if (!brio::Dma::init()) { /* the DMA block did not come out of reset */ }  // before the host binds its engines
if (!Bus::init(clock, 8'000'000)) { /* the ceiling cannot be made, or the block did not come up */ }
Cs::output(true);

Bus::Request r{};
r.cs = Cs::ref();
r.cmd = brio::lend<brio::Lease::reply>(cmd);  r.cmd_len = 1;
r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
r.clock = brio::SpiClocks::div4;                      // asks 31.25 MHz; the ceiling makes it 7.8125
r.mode = brio::SpiMode::mode0;
r.reply = brio::reply_to<Display, brio::SpiDone>();
r.engine = brio::SpiEngine::dma;
brio::post<Spi>(r);                                   // to the arbiter; asynchronous

extern "C" void isr_dma_0() {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

A polled read on a GPIO select is the same request with `r.engine = SpiEngine::polled` and
`r.rx` lent, on this host or the next: the post returns at once, the loop runs in
the arbiter's dispatch and the buffers stay lent until `SpiDone`.
Without the engines, `SpiHost<0, pins>`: polled requests alone, each
run inside `start()` (`r.engine = SpiEngine::polled`), one naming dma answered
`spi_rejected` - a program that posts one states `static_assert(
Bus::has_dma)` and fails to compile instead. A client, its vector in SRAM (from
the flash its first entry after a quiet spell is the XIP cache's fill, frames
late: the client's vector, bench findings):

```cpp
constexpr brio::SpiPins client_pins{.sck = 10, .tx = 11, .rx = 8, .cs = 9};
using Client = brio::SpiClient<1, client_pins>;
if (!Client::init(clock, {.mode = brio::SpiMode::mode1})) { /* the block did not come up, or the configuration is refused */ }
Client::interrupts(brio::SpiInterrupt::rx | brio::SpiInterrupt::rx_timeout, true);
Client::enable(first_answer);
extern "C" [[gnu::section(".fast_text")]] void isr_spi1() {
    (void)Client::isr();
    while (const auto f = Client::poll()) { heard(*f); }
    while (Client::writable() && have_answers()) { Client::write(next_answer()); }
}
```

A host for such a client in modes 0 and 2 (on another board, or the
other instance), the select its own frame signal:

```cpp
constexpr brio::SpiPins framed{.sck = 18, .tx = 19, .rx = 16, .cs = 17};
using Feeder = brio::SpiHost<0, framed>;              // Feeder::frame_select
if (!Feeder::init(clock)) { /* the block did not come up */ }
Feeder::Request r{};                                  // r.cs left null
r.tx = brio::lend<brio::Lease::reply>(frames); r.len = 64;
r.mode = brio::SpiMode::mode0;
r.engine = brio::SpiEngine::polled;
(void)Feeder::start(r);                               // a polled request answers true, done or not: status() decides
if (Feeder::status() != brio::spi_ok) { /* spi_stalled: "What each answer asks of the program" */ }
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
| `spi_rejected` | the arbiter's FIFO was full; the host refused a request naming dma: it has no engines, or they cannot carry it (16-bit frames on byte engines, or off a half-word); or a program driving the host directly started a request over one on the engines, the one busy check | nothing on the wire, the select and the D/C unmoved, the request in flight going on | the program: a full FIFO may be tried again, and the busy check's refusal once the first request has ended; what the engines lack never as it stands (`static_assert(Bus::has_dma)`; half-word engines, a buffer aligned to a half-word, or the request named polled) |
| `spi_stalled` | a polled wait ran out of its budget, or the DMA line's wait for the end of the last frame (modes 0 and 2) ran out of its bound | select and D/C high; polled, the block reset and reconfigured to the state applied, both FIFOs empty; on the engines, the reset owed to the next `start()` | nobody: the next request starts on a clean block. Remove the cause: a reset does not start a stopped clock |
| `spi_dma_fault` | a channel reported a bus error | requests down, both channels put away and bound again, select and D/C high, the block's reset owed to the next `start()`, frames perhaps still shifting until then with the select high | nobody: the arbiter answers and goes on, and the next request the host does not refuse resets the block before anything else |
| `spi_timeout` | a timed arbiter's limit passed with no completion | the arbiter has already called `recover()` | the program judges the device |

A stall holds the core for its budget, derived from the clocks whenever a rate is
applied: the request's frames and two more at a share of twice a frame's time on the
wire, rounded up to a power of two, in reads of the status register ([the PL022
page](../pl022/README.md)). A read costs 12 core cycles in that loop (letter g, 3520 of
them in 337 to 339 us). So a stall lasts 30 to 60 times a long request's wire time in 8-bit
frames and 27 to 54 times in 16-bit frames, by the divisor, and (frames + 2) / frames of
that on a short request, 3 times at one frame. Nine 8-bit frames at clk_peri / 2, 1.15 us
on the wire, are 704 reads: 73 or 74 us with the code warm and 104 to 175 us the first
time in a letter, the reset included, with every other active object waiting under
`Tenuto`. The block's reset then throws away what its two FIFOs hold
and costs the request that pays it some hundreds of cycles (letter h). A request already in the arbiter's FIFO starts in the same dispatch
as a stall or as the answer to a fault, before the requester's handler can run, and
finds that clean block; the program calls nothing after either. `prime()` pays a reset
a fault left owed, for a caller framing its own select, so that it is paid before the
select falls; with a null select the program's own stands through a stall's reset, and
that caller raises it when `start()` answers. A reset does not
start a stopped clock: on a block whose clock stays off a queued polled request
stalls too, on its own budget, and one naming dma goes to the engines and has
no completion, `spi_timeout` on a timed bus and a bus busy for ever on an untimed
one. `recover()` stays for a timed arbiter's unanswered request and for a program
that wants it with the bus idle. No verb says the engines are idle, and `recover()`
under a request on them takes it away, its completion never coming, so a program
calls it when it knows: every request it has posted answered, on a bus with one
client.

## Bench findings

The reference suite is `test_rp2040_spi`, green on the WeAct board,
clk_sys and clk_peri at 125 MHz: 89 verdicts, 58 of eleven letters on the
loop-back (LBM, no wire) and 31 of nine on the pair's wire against the
desk partner's peer (the last group below).

- The rate chooser: clk_peri / 2 at the top (62.5 MHz), 8.93 MHz under
  a 10 MHz ceiling, 992 kHz under 1 MHz, nothing under 1 kHz. The
  block's reset state: both FIFOs empty, disabled; a 3-bit and a
  17-bit frame, an odd prescaler and a host with its output disabled
  are refused, a 12-bit TI configuration is taken, nothing is
  configured while enabled.
- THE LOOP-BACK ON THE ENGINES: the four modes at 8 and 16 bits,
  sixteen frames each on the engines, byte-exact with ONE DMA interrupt
  each; a two-frame command phase (in the loop inside `start()`, the command
  rule) then 32 data frames, the echo discarded; a read
  with no out buffer clocks 0xFF; the select released after every
  transaction; an empty request completes on the spot.
- THE POLLED PATH at every named rate carries 64 frames byte-exact;
  the suite times the whole request on the system timer, 43 to 50 us at
  clk_peri / 2 and 23 to 26 at / 4 against a wire of 8 and 16 (fifteen
  runs), the
  first request of the letter paying the XIP cache its fill and every
  one its own cost on top of the loop's. The loop's cost a frame is the
  next item's.
- **THE HOST ABOVE THE WIRE, `bench_rp2040` letter e** - the polled loop
  in both its shapes and the price of a request - on a host with
  no engines, SPI0 on the same pads with GP17 the select and GP22 the D/C
  line, MISO floating, the receive-overrun flag read after every run and
  clear on every one; the ruler SysTick's cycles at 125 MHz, the best of
  five, at clk_peri / 2, / 4 and / 16 (62.5, 31.25 and 7.8125 MHz) in
  mode 0 and in mode 3, each figure a frame by the difference of 16 and
  256 frames:
  - THE RECEIVE SHAPE (`spi.poll.rx`, eight frames in flight - written and
    not yet read): **42 cycles an 8-bit frame and 49 a 16-bit one** in this
    image - the loop's three accesses of the block a frame, the frame
    stored, the pointers; the same loop turns in 32 cycles an 8-bit frame
    in `test_rp2040_spi`'s image (letter f), and what this one adds is not
    split - so the loop sets the
    pace at 62.5 MHz in both widths (the wire 16 and 32, 19 and 35 with
    mode 0's gap) and at 31.25 MHz on 8-bit frames in both modes (42
    against 32 and 38); everywhere else it reads the wire: 70 / 64 for
    16-bit frames at 31.25 MHz, 152 / 128 and 280 / 256 at 7.8125. A frame
    read and thrown away instead of stored costs 40 (8-bit) and 44
    (16-bit).
  - THE WRITE SHAPE (`spi.poll`, no in buffer, taken for 8-bit frames at
    clk_peri / 2 on a run longer than the FIFO): the TNF-paced loop at
    **21.00 cycles a frame in both modes** against the wire's 16 and mode
    0's 19 - two accesses a frame, 21.0 in letter f's image too, 1.31 times
    the nominal wire's time where the receive shape would be 2.63 (larger
    being slower).
  - THE FRAME AGAINST THE WIRE AT EVERY DIVISOR (`spi.frame`: 8 and 16 bits,
    modes 0 and 1, a frame by the difference of two lengths, the least of
    five under the mask, the whole table measured before a line of it is
    printed), in core cycles, the wire with mode 0's gap in brackets:

    | | clk_peri / 2 | / 4 | / 8 and slower |
    |---|---|---|---|
    | read and stored, 8-bit, mode 0 | 42 (19) | 42 (38) | the wire |
    | read and stored, 8-bit, mode 1 | 42 (16) | 42 (32) | the wire |
    | read and stored, 16-bit, mode 0 | 49 (35) | the wire | the wire |
    | read and stored, 16-bit, mode 1 | 49 (32) | the wire | the wire |
    | no in buffer, 8-bit, mode 0 | 21 (19) | 40 (38) | the wire |
    | no in buffer, 8-bit, mode 1 | 21 (16) | 40 (32) | the wire |
    | no in buffer, 16-bit, mode 0 | 44 (35) | the wire | the wire |
    | no in buffer, 16-bit, mode 1 | 44 (32) | the wire | the wire |

    "The wire" is within 0.6 of a cycle a frame, in three runs of the
    letter, with the overrun flag never raised and no bad status. So
    the FIFOs leave the bus no dead time at /8 and slower in any row: at
    those rates a polled request costs the core the wire's time and no more.
    The loop's 42 cycles a frame at 62.5 MHz in mode 3 are 2.63 times the
    nominal wire's time, so a long read that wants a fast rate names
    the engines, as does any request the other active objects must keep
    running through: the engines run at the wire, with mode 0's gap there, at
    clk_peri / 4 and slower ("The rate asked" below).
  - THE RECEIVE SHAPE UNDER A LONG HANDLER: 100 receive requests of 256
    frames on the loop-back at each of the three rates with the tick's
    handler stretched to 1500 cycles - eight frames being 128, 256 and
    1024 cycles - **100 exact and 0 overruns at every rate**, with 14 or
    15, 14 or 15 and 32 or 33 of the stretched ticks landing inside the
    requests (ten runs).
  - THE PRICE OF A DCS COMMAND (`spi.req`, a command byte with D/C low and
    0, 2 and 15 data bytes with it high, polled, nothing read back, at
    7.8125 MHz): wall less the wire's cycles **301 for one byte, 425 for
    three, 743 for sixteen**, each the same in three runs of the letter. Of
    these cycles the block's gap between the frames of a phase is 0 / 24 / 336,
    at 1.5 SCK periods (16 frames in mode 0 less mode 3 read 374: fifteen gaps
    and the 14 cycles that follow); the wait for the end of each phase's last
    frame is one SCK period from the receive flag, 16 cycles, 8 of them past
    the frame's last edge; and the phase boundary's drain (the data frames wait
    for the command frame to come back whole and its last edge, so D/C flips on
    the wire) means a second frame start. What remains, 287, 373 and 379, is the
    host's own code and each phase's frame start, which no letter splits; a
    request of 16 frames in one phase in mode 3, with neither gap nor wait, reads
    275 over the wire.
- **WHICH ENGINE COSTS THE CORE LESS** (`test_rp2040_spi` letter f; the IP
  stratum's page, "Which engine costs the core less", has the arithmetic).
  Nothing in the host reads these figures: they are the programmer's guidance.
  E and F are measured through one call, the suite's own `xfer()`, so the
  call's cycles cancel: E the 32-frame block's wall on the engines less its 32
  frames on the wire, F the intercept of two polled lengths (an 8-frame
  request's wall less 8 frames at the loop's own pace, that pace the difference
  of a 24- and an 8-frame request), both at clk_peri / 4 in mode 3, the best of
  fifteen in three rounds 3 ms apart: **E 711 for a read and 712 for a write, F
  490 and 487, the loop's frame there 40.0 and 38.6 cycles**, the same in all
  27 runs (twelve of the letter alone, fifteen inside `z`); and the two loops' turns at clk_peri / 2 on the host with no
  engines, 32.0 received and stored and 21.0 written on TNF. The walls are net
  of the XIP cache's misses, which the letter prints beside them: the cache is
  two-way, so where three lines of one transaction fall in one set at least one
  of them misses on every pass, whatever the cache evicts, and no warm-up cures
  it; what falls in one set is each image's layout (these runs counted none). So E - F is 221 for a read and 225 for a write, and the
  engines cost the core fewer cycles than the loop from ceil((E - F) / frame)
  frames, a frame the wire's or the loop's turn, the longer (mode 3, 8-bit
  frames; in modes 0 and 2 the wire's frame is 9.5 periods, so where the wire
  sets the frame, at / 8 and slower, the counts are the same or smaller, and at
  / 2 and / 4, where the loop's turn outlasts that frame too, the same):

  | rate | a frame read, the count | a frame written, the count |
  |---|---|---|
  | clk_peri / 2, 62.5 MHz | 32 cycles (the loop), 7 | 21 (the loop), 11 |
  | / 4, 31.25 MHz | 40 (the loop), 6 | 38.6 (the loop), 6 |
  | / 8, 15.6 MHz | 64 (the wire), 4 | 64, 4 |
  | / 16, 7.8 MHz | 128, 2 | 128, 2 |
  | / 32 and slower | 256 and more, 1 | 1 |

  In words: at clk_peri / 32 and slower the engines cost the core less from a
  single frame; at / 2 a read needs seven frames to repay them and a write
  eleven. The core's cycles are not all a choice weighs (what else must run
  meanwhile, the vector's wait below), which is why the program chooses.
- **dma IS DMA AT ANY LENGTH** (letter f): one frame, read and written, at
  clk_peri / 2 and / 4, where the loop costs the core less - `start()` answers
  false, `dma_isr()` is the end, one DMA interrupt, the read exact, the select
  released. THE REFUSALS: 16-bit frames naming dma with the out buffer, the in
  buffer, both, a write, a read, a command alone, or a command ahead of such
  data off a half-word, each
  answered `spi_rejected` inside `start()` with the select held low beforehand
  still low, the D/C held high still high, the control word as it was though the
  request asked another mode, the block quiet and the in buffer untouched, no
  interrupt; the next request on aligned buffers exact on the engines. THE
  COMMAND RULE: a command of 1, 2, 8, 9, 128 or 200 frames then 32 data frames
  at clk_peri / 4, the command in the loop and the data on the engines - one
  DMA interrupt every time, D/C high when `start()` answers false, the data
  exact; a command of 16 frames alone rides the engines, `start()` false with
  D/C and the select low on its way, one interrupt, both high after it; a
  command off a half-word ahead of 16-bit data is carried, the command a frame
  at a time in the loop. 27 runs of the letter.
- **WHEN A REQUEST ENDS, THE WIRE HAS ENDED** (`test_rp2040_spi` letter t, the
  IP stratum's page has the rule). The SCK pad, GP18 with the loop-back off, read
  as an input right after the end - after `start()` answers true, or in the DMA
  line's handler right after `dma_isr()` answers true - in the four modes, read
  and write, the request's select and the program's (a null `cs`), on both
  engines, at every rung from clk_peri / 65024 to / 2. THE INSTRUMENT first:
  the pad reads each mode's idle level after `prime()` and both levels inside a
  frame. The pad's read comes some tens of cycles after the loop's last flag
  and some hundreds after the channel's completion, so an early end is seen
  wherever half a period outlasts that. In a control - the same letter, one
  run, over a build of the host that takes the receive FIFO's flag for the
  end - every end in modes 0 and 2 found SCK off its idle
  level at clk_peri / 65024, / 4096 and / 1024, on both engines, the pad coming
  idle 32 306 cycles after the end at / 65024 (half a period is 32 512) and BSY
  clearing 64 814 cycles after it (one period); at / 256 and faster none was
  seen, half a period (128 cycles at / 256) being shorter than the way from the
  end to the read. Modes 1 and 3 were never early. With the wait, **no end in
  any mode, rung, engine or select found SCK off its idle level**, the
  request's select high at its end and the program's still low, 27 runs (twelve
  of the letter alone, fifteen inside `z`). A command's end: one command frame
  at clk_peri / 65024, `start()`'s wall 585 562 to 586 562 cycles in modes 0
  and 2 (BSY's fall, nine periods of 65 024 from the write that starts the
  frame), polled alone and on the way to the engines alike, and in modes 1 and
  3 (the last edge, eight periods) 520 552 to 520 648 polled alone and 520 712
  to 520 806 on the way to the engines, the launch's own cycles on top; so D/C
  rises after the command's last edge. THE VECTOR'S WAIT, the handler's cycles
  around `dma_isr()` less the same in mode 1, which waits for nothing (144
  cycles), the least of four, the same 27 runs:

  | rate | the wait in mode 0, every other interrupt held off |
  |---|---|
  | clk_peri / 65024 | 64 877 cycles, 519 us |
  | / 4096 | 3947 to 3957 cycles, 31.6 us |
  | / 1024 | 877 cycles, 7.0 us |
  | / 256 | 117 cycles, 0.9 us |
  | / 128 and faster | 7 cycles: one read of SSPSR and its test |

  That is one SCK period less the way into the handler: from clk_peri / 128 up
  the vector's own entry covers it. In modes 1 and 3 the handler waits for
  nothing. The polled loop's tail waits the same period in modes 0 and 2, on
  the core and with interrupts on.
- THE ENGINES on the loop: a 128-byte block at 31.25 MHz, ISR-completed
  and exact with ONE DMA interrupt; a command frame in the loop inside
  `start()` then 32 data frames on the engines, one DMA interrupt; a polled read of 64 frames at 62.5 MHz in mode 3 on the same
  host, the loop inside `start()`, exact, no interrupt of either line; a
  read with no out buffer through the transmit engine's fixed 0xFF cell;
  16-bit
  frames riding the half-word engines exact with no SPI interrupt and one
  DMA interrupt; and a 16-bit request naming dma on buffers one byte off a
  half-word refused, its in buffer untouched, no interrupt of either line,
  the same request polled exact. The
  letter's first engined block, timed, takes **98 to 116 us** over fifteen
  runs, where `bench_rp2040` letter d's figures (904 fixed and 38 a byte)
  give 46 warm and the wire 33: the cache's fill of what
  an engined transaction runs from the flash, the host's code included in
  the default placement (the placement item below splits it).
- **WHAT AN ENGINED TRANSACTION COSTS, `bench_rp2040` letter d**, on the
  loop-back, the half-word engines on channels 4 and 5, the core asleep
  in the idle path between the start and the completion, the best of
  four: 16 8-bit frames at 31.25 MHz take **1416 cycles, 726 of them
  busy** in four runs of ten (a fifth carries the tick's second interrupt, and the
  other five read 1507 to 1975, a warm figure from the flash no XIP count has split) -
  the DMA line's handler 204 of those with the meter's
  stamps, its wait for the end of the last frame one read of SSPSR at this
  rate - and the transaction's **fixed cost, the wall less its frames' nominal
  wire, is 904 cycles (7.2 us), of which 90 the block's fifteen mode-0 gaps**: 814,
  at 3.906 MHz too, and 822 for 16-bit frames. Inside the
  block the engines run at the wire with the block's gap, by the
  difference of 16 and 256 frames: **38.00 cycles a byte at 31.25 MHz**
  (32 of bits and 6 of gap) in four runs of ten (35.7 to 38.8 in the
  other six, most of them runs whose 16 frames read high), 304.00 at 3.906 MHz
  (256 and 48) in seven runs of ten (304.4 to 305.5 in the other three),
  35.00 a byte in 16-bit frames (64 and 6 a frame) in six of ten (34.4
  to 35.8 in the other four) - 256 frames at 1.28 and 1.19
  times the nominal wire's time, 4096 16-bit frames at 1.09 with
  the transaction's one interrupt.
- THE RATE ASKED, which a loop cannot tell (`test_rp2040_spi` letter d,
  a frame inside an engined block timed by the difference of a 2048- and
  a 256-frame block on the engines, neither buffer named): 1000, 999 to
  1000 and 999 thousandths of eight SCK periods at clk_peri / 4, / 16 and /
  64 in mode 3, 999 to 1000 of nine and a half at / 16 in mode 0 - the frame
  gap - and 999 of sixteen at / 16 in 16-bit frames, each against
  clk_peri over the divisor asked, over fifteen runs; and no polled rate of
  letter c faster than its wire.
- **WHERE THE HOST'S HOT PATH RUNS IS THE PROGRAM'S CHOICE, and the choice
  is about the COLD request.** `SpiHost`'s last template parameter is
  util/placement.hpp's tag: `InFlash`, the default, leaves `start()` and
  the DMA line's body in the flash (the body inline in the app's vector),
  `InRam` makes each ONE flattened instance in `.fast_text.brio`, copied
  to SRAM with `.data` - the engines' own verbs and the launch folded in,
  so an engined transaction's driver code is in SRAM whole
  ([platform.md](platform.md), "A handler in SRAM", has the table and the
  rule). `bench_rp2040` letter e's `spi.cold` is the instrument for the
  request: `spi.req`'s three-byte polled request after every line of the
  XIP cache is flushed (`Xip::flush()`), under the mask, the cache's
  misses (accesses less hits) read around it - **12140 cycles and 142
  misses with the host in the flash, 2881 and 41 with it InRam, against
  821 and 834 warm**; the 41 are the caller's (the ruler's reads, the call
  site, the veneer). Warm the two read the same but for the veneer, 13
  cycles a request here and 18 in `spi.req`'s line - in nine runs of ten
  with the host in the flash; the tenth read 1379 with two misses. The
  engined block cold (letter d after letter c, 16 frames at 31.25 MHz,
  warm 1399 to 1584) splits the same way: 32588 cycles from the flash,
  17884 with the host InRam, 21820 with the vectors placed, 10620 with
  both - the host's code some 14700 of the difference, the vectors' some
  10800, the caller's the rest. THE SRAM IT COSTS under InRam: `start()`
  824 bytes without engines and 1572 with them, the engines' launch folded
  in, the DMA line's body 684. Every call across the boundary goes through
  a veneer (a push, a literal load, a move into ip, a pop, a `bx`): the
  Cortex-M0+'s `bl` reaches 16 MB, and the SRAM lies 256 MB from the
  flash.
- **A REQUEST'S END AND THE ONE BUSY CHECK** (`test_rp2040_spi` letter j, the
  loop-back, ten runs). Over 64 frames on the engines at clk_peri / 128, a request
  naming either engine, in another mode and rate with a command phase and a D/C of its
  own, is answered `spi_rejected` inside `start()` with the select still low and the
  D/C still high; the first request then ends `spi_ok`, exact, one DMA interrupt. A
  command phase that stalls, the block's enable taken down under it, leaves the select
  and the D/C high with `spi_stalled` inside `start()`: polled with data behind it,
  polled alone, ahead of a block on the engines, and on a host with no engines. A
  command alone on the engines whose buffer the bus refuses, the D/C low when the fault
  comes, leaves both high with `spi_dma_fault` from the DMA line. On the host with
  engines the next request is exact after each; letter g judges the host with none
  after a stalled write. THE COST: one load and a branch a request for the check, and
  the status and the D/C stored at every end in the DMA line's body; on the polled path
  the D/C's extra store sits on the failed paths alone. Nothing of it reaches the
  per-frame loops of `start()`, read in the release listing. That 16-bit frames on byte
  engines are refused however aligned is judged on the simulated chip (`test_pl022`).
- **A RELEASED REQUESTER, THE SAME OWNER AGAIN** (`test_rp2040_spi` letter u, the
  loop-back, ten runs of four rounds): SPI0 released the worst way, both DMA enables set,
  its channels stopped, three frames standing in the receive FIFO, the block held in
  reset with the enables left set. The host's `init()`, then 32 frames on the engines:
  `spi_ok`, one interrupt, exact, both FIFOs clean. THE CONTROL, the reset removed: with
  the block disabled the bound channel moves nothing, and with it enabled the receive
  channel's first three items are the three stale frames, the PL022 keeping its FIFOs
  across a disable. That is the request a reset at `init()` withdraws ([dma.md](dma.md)).
- **A POLLED WAIT THAT RUNS OUT** (`test_rp2040_spi` letter g): the
  block's enable taken down under a polled write of nine frames at clk_peri
  / 2 with three frames left standing in the receive FIFO, so the transmit
  FIFO fills and the loop's budget runs out, 704 reads of the status register:
  104 to 175 us the first time in the letter, 73 or 74 us warm, the reset included.
  Warm again with 64 frames, 4224 reads take 411 or 412 us, so a read costs 12 core
  cycles in the loop as compiled, printed and not judged. The verdicts:
  `spi_stalled`, the select released, both FIFOs empty and the block enabled at
  the word applied. With no `recover()` the next request is exact, polled on
  the host with no engines, polled and on the engines on the host with them. A
  command frame polled ahead of an engined block, its enable down, answers the same inside `start()` with no block
  launched. Through `SpiBus`, two requests from one dispatch, the first stalled:
  the replies `spi_stalled` then `spi_ok` in that order and the second's data
  exact, polled and on the engines. Ten runs and three of `z`. That the select goes up before
  the reset is judged on the simulated chip (`test_pl022`); no letter here reads
  the SCK pad across it.
- **A DMA BLOCK THE BUS REFUSES** (`test_rp2040_spi` letter h): a
  request of 64 frames at clk_peri / 128 with its buffer where 2.2 says a range is
  unmapped: the out or the in buffer at 0x40100000, past the last peripheral of the
  APB, and an out buffer 16 frames short of the SRAM's end, a read failing with frames
  already on their way. A bus error on the channel and never a fault of the core.
  `dma_isr()` answers true, one interrupt, `spi_dma_fault`, the select released. With
  frames on their way the block is still enabled and its transmit FIFO not empty
  afterwards, which is what the interrupt left for the next `start()`. With no
  `recover()` that request, polled or on the engines, is exact, no frame of the faulted
  block in front of it; through `SpiBus`, queued behind the read off the end,
  `spi_dma_fault` then `spi_ok` and its data exact both ways. Ten runs and three of
  `z`. It paid the reset first, and what that costs it is not pinned: a difference of two
  walls taken with interrupts on and no XIP count beside it, reading 224 to 322 cycles
  over the thirteen runs, 228 in ten of them. A WRITE THE BUS
  REFUSES LANDS NOWHERE IN THE SRAM: 32 half-words of a mark written at the hole,
  `spi_dma_fault`, and across it no kilobyte of the 264 holds the mark once more than
  it did, thirteen runs of thirteen; the same count with the write aimed inside the SRAM finds
  all 32 in their block. A landing in a peripheral it could not see.
- **A WINDOW FRAMED BY HAND AFTER A FAULT** (letter h on the loop-back, letter v on
  the pair's wire): the select null in the Request, `prime()`, the select low,
  `start()`, the select high, behind a fault that left frames on their way, with no
  `recover()`. On the loop-back both FIFOs are clean when `prime()` returns, before the
  select falls, and the window is exact, polled and on the engines. On the wire the
  foreign client reads sixteen frames exact and answers sixteen exact, in the four
  modes, the fault taken with the select high. Ten runs and three of `z`. Measured once on
  a build without `prime()`'s payment, which no suite stages: the
  window read 16 of 16 frames wrong both ways in three of the four modes.
- THE KERNEL over it, unchanged: four transactions through SpiBus over
  the engined host with four replies, on the engines and polled
  interleaved on one bus; six
  posted into a four-deep queue, one rejected at once and every
  request answered once; an idle bus votes for the sleep, a busy one
  against.

ON THE PAIR'S WIRE (letters n, o, p, q, r, m, s, w and v, the two Raspberry
Pi suites' shared letters, against the WeAct RP2350B's peer on either
half: its client a PIO program at 150 MHz, its host that chip's PL022):

- THE HOST AGAINST A FOREIGN CLIENT, every window ONE GPIO-held select
  of sixteen frames: the four modes, the client MSb and LSb first (the
  LSb rows an exact two-way bit reversal - the PL022 is MSb first and
  nothing else), 8- and 16-bit frames, the client's DMA and polled
  serves - 32 exchanges exact both ways; a receive-only client (its
  answer line dark) hears every frame and this host reads its own
  pull-up throughout; an answer-only one is read exact. The command
  channel at clk_peri / 128 (977 kHz) carries ten pings of ten, and 32
  bytes of noise under a held select wake nothing.
- THE LADDER, even divisors 256 to 2, the polled host (32 frames) and
  the engines (64 frames, half-word engines on channels 6 and 7): **exact
  both ways at clk_peri / 12 (10.4 MHz) and slower in all fifteen runs**, and at
  / 10 (12.5 MHz, a half period of 40 ns) in eight of fifteen polled and ten of
  fifteen on the engines; from there up to /8 (15.6 MHz) this host reads the
  client's answers wrong while the client
  still hears this host exact up to /4 (31.25 MHz) - the answer path is
  the bound, some four of the client's cycles (26.7 ns) and the pads. The
  letter judges at /12 and slower, the derived ceiling (a half period of four of the
  partner's cycles and 15 ns, derived in the suite); the engines, every
  window a request on the engines, on a wire against a foreign client
  exact wherever the polled loop is.
- THE KERNEL against the second chip: four transactions queued from one
  dispatch inside one held window, one polled among three on the
  engines, every byte read back by the peer.
- THE ROLES INVERTED: this chip's `SpiClient` under the peer's PL022
  host, whose select is its own frame signal: 64 frames back to back in
  every mode, 8- and 16-bit, exact both ways at /64 and /32 of the
  host's clk_peri (2.34 and 4.69 MHz) and at /16 (9.375 MHz), the three
  rungs this client admits (its clk_peri at least twelve times the far
  host's SCK), all judged - more than one frame per select in modes 0
  and 2 on the terms SPH = 0 sets. The
  frame signal, counted at this end by a PIO machine: a window a frame in
  modes 0 and 2, one for the burst in modes 1 and 3; its high pulse
  between two frames half an SCK period (48 ns at /16, 96 ns at /32); the
  burst at the wire's time (430 us for 64 16-bit frames at /64 in mode
  1, 470 in mode 0 with the 1.5-period gap a frame).
- THE CLIENT FROM ITS VECTOR (letter s): the same client served by its
  interrupt instead of a loop - RXIM at the receive FIFO's half, RTIM,
  RORIM; every entry drains the receive FIFO and refills the transmit
  FIFO to its depth, TXIM never needed - 67 frames in modes 0 and 3, 8-
  and 16-bit, exact both ways at /64, /32 and /16 of the far host, every
  rung this client admits: one entry per four frames (16 at /64, fewer
  where the frames come faster than an entry drains them) and the three
  frames under the half delivered by ONE receive-timeout entry at /64.
  Fifteen runs of the letter. The body runs from SRAM (the program's
  `.fast_text`): from the flash, its first entry after a quiet spell is
  the XIP cache's fill and comes several frames late - the receive FIFO
  overflowed at /64 and the transmit FIFO ran dry at /16, the far host
  reading 0x00 from the ninth answer on. The suite's vector runs from
  SRAM too: left in the flash, its own lines missed in an image whose
  layout put them out of the cache, and the overrun item below counted
  15 entries for 16 frames lost in four of six runs, where every run
  counts 16 with the vector placed. The letter's first burst is a
  warm-up, printed. Past the ceiling, printed: /14 (10.7 MHz) and /12
  (12.5 MHz) exact in every setting of fifteen runs.
- THE CLIENT'S OVERRUN (letter s): eight answers preloaded, nothing
  taken, 24 frames from the far host - the receive FIFO keeps the first
  eight exact, and the vector is entered on RORIM sixteen times, once a
  frame lost; the far host reads the eight preloaded answers exact.
- THE ARBITER'S PER-BUS TIMEOUT (letter w, a completion that never comes
  staged by running the DMA line's body and posting nothing): answered
  `spi_timeout` 99.3 to 100.1 ms after the request against the arbiter's
  100 ticks, and recover() leaves the engine where the next four
  transactions on the same bus AO - three on the engines, one polled -
  run to `spi_ok`. Fifteen runs.
- THE CLIENT'S ANSWER PAD (letter m, its output enable read in
  GPIO_STATUS): DRIVEN while deselected as while selected, and under SOD
  - the far host then reading the client's FIRST answer in every frame,
  0x93 x 32 - which is 4.4.3.14's nSSPOE "not connected to the pad"; the
  dark listener takes the pad away and the far host reads its own pull-up
  throughout, the client still hearing every frame.
- The dark listener's pad goes to no function with no pull
  (`drive_output(false)`, the traits' `no_function`, `dark_pad_config()`):
  released to its reset state, the pad keeps its pull-down against the far
  host's pull-up and the line sits between the two, the RP2350 host reading
  0x02 where it should read 0xFF ([../rp2350/spi.md](../rp2350/spi.md)).

## Not covered yet

Driver gaps, each with its reason:

- The transmit-only shape's reach on this core: the host takes it for
  8-bit frames at clk_peri / 2 alone, the rule drawn in the IP stratum from
  the RP2350's two cores, where the receive shape keeps the wire at / 4. On
  this Cortex-M0+ a write with no in buffer reads 40 cycles a frame at / 4
  (the wire 32, 38 with the gap) and 44 in 16-bit frames at / 2 (the wire
  32, 35 with the gap), and the transmit-only loop's 21 would keep the wire
  at both. Its tail still costs a request of a few frames its drain, so the
  rule wants a family's figure and a measurement of that tail, not a
  constant moved.
- The TI and Microwire framings on the wire: configured and read back
  in the resource, no device on the desk speaks them.
- Frame widths other than 8 and 16 in the Request: the other strata's
  Request speaks two, the resource takes 4..16; a device with a
  12-bit frame is the first user.
- The client on the DMA engines: `SpiClient` answers from its
  interrupt; a client that streams is born with a program that needs
  one.

Implemented but not bench-verified, each with what would measure it:

- A stall on a stopped clock: every stall the suite stages is a block whose enable
  went down, which its reset cures. What the reset does with `clk_peri` stopped,
  and what the next request answers, a letter that stops it around a request with
  the console quiet would measure.
- The DMA line's wait for the end of the last frame running out of its bound
  (modes 0 and 2): a block whose clock stops between the last frame's arrival
  and BSY's fall. It answers `spi_stalled` with the select and the D/C released and the
  block's reset owed, judged on the simulated chip (`test_pl022`); a letter that
  stops clk_peri inside that one period would measure it on silicon.
- The vendor's loop on this board: the pinned pico-sdk's
  `spi_write_read_blocking` and `spi_write_blocking`, SRAM-resident by
  the SDK's own placement, against letter e's 42 and 21 cycles a frame
  at clk_peri / 2 - a program built by the SDK's own CMake and timed on
  the same ruler, as on the RP2350 ([../rp2350/spi.md](../rp2350/spi.md)).

- `cs_setup_us`: a device that needs it, or a scope on the select and
  the first clock.
- The engines' rate inside a block at clk_peri / 2: reads on them there
  are exact (letter f), and the rate is timed only at / 4 and slower. A
  `frame_permille` row at / 2 in letter d would measure it.
