# SPI (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 4.4 (the
ARM PL022: 4.4.2 the functional description - the prescaler, the two
FIFOs, the DMA interface -, 4.4.3 the operation - the clock ratios
of 4.4.3.4, the bit rate of 4.4.3.6.1, the frame formats of 4.4.3.7
-, 4.4.4 the registers), 2.19.2 (table 279: the pins), 2.15.3.1
(clk_peri); docs/design/spi-bus.md for the Request and the two
completion styles. The driver is ARM's and not this chip's, so it lives
in the IP stratum: `brio/pl022/spi.hpp` holds the resource, the host
engine and the client ([../pl022/README.md](../pl022/README.md)), and
`brio/rp2040/spi.hpp` holds what this chip owes it - the pin table of
table 279, the `Rp2040Pl022` chip traits over `pin.hpp`, `resets.hpp`,
`delay.hpp`, `nvic.hpp` and `dma_engine.hpp`, and the PUBLIC NAMES
`Pl022<n>` (the resource), `SpiHost<n, pins, ...>` (the engine `SpiBus`
drives) and `SpiClient<n, pins>` (the other end), this chip's aliases of
the three templates there. The reference suite: `test_rp2040_spi`, on
the loop-back; the wire is the desk partner's, SPI1 on GP10 / GP11 /
GP8 with GP29 its select, to the WeAct RP2350B
([../boards/weact-rp2040.md](../boards/weact-rp2040.md)).

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
SSPFSSOUT, pulses BETWEEN FRAMES in Motorola format - which is why a
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
OWN frame signal reaches it in all four, the signal pulsing between the
frames in exactly those two modes. And nSSPOE is NOT CONNECTED TO THE PAD
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
  (the raised-and-enabled sources, the two clearable ones cleared),
  `dma_requests`, `dreq_tx` / `dreq_rx`, `irq()`.
- `SpiHost<n, pins, TxEngine, RxEngine, Placement>`: the engine `SpiBus`
  (= `BusMaster`) drives. Its `Request` is the other strata's: a chip
  select `PinRef` and a D/C line, a command phase (D/C low), a data
  phase with an optional out buffer (null = 0xFF dummies) and an
  optional in buffer (null = discard), the per-request `mode`, `clock`
  and `bits`, `cs_setup_us`, and `polled` - false runs the frames on
  the receive interrupt in batches of up to eight in flight, true
  spins inside `start()` in one of two shapes: a receive with eight
  frames in flight, or a write that paces on TNF and reads nothing
  (the IP stratum's page, "The host above the wire"). The request is
  28 bytes here, laid out words first. `init(clock, max_sck_hz)`,
  `rebase(pclk, sys)`, `clock_for(hz)`, `sck_hz`, `ceiling_clock`,
  `prime()` for a caller framing its own select, `frame_select` (the
  pin set names `cs`: the block's own frame signal drives that pad - a
  select a frame in modes 0 and 2, one a run of back-to-back frames in
  modes 1 and 3 - and a Request's `cs` stays null), `loopback(on)` (kept
  through every re-application: the wireless instrument), `isr()`,
  `dma_isr()`, `status()`, `recover()`, `release()`, and
  `dma_min_frames` - the data-phase length below which a request takes
  the pump even with engines bound, this chip's traits' number. The
  engine slots take `dma.hpp`'s `DmaTxEngine` / `DmaRxEngine` on any
  two channels, both or neither, both of one element: `uint16_t` -
  SSPDR's width - carries 8- and 16-bit frames alike, each block at its
  frame's beat, and `uint8_t` engines leave 16-bit frames to the pump.
  The data phase moves as one block each way with the requests raised
  around the engines; the transmit engine is bound to report a bus
  error alone (IRQ_QUIET), so the receive block's completion is the
  transaction's ONE DMA interrupt. The receive engine arms at
  HIGH_PRIORITY, a frame landing on a full receive FIFO being lost, the
  transmit engine at normal ([dma.md](dma.md)). A 16-bit request whose buffers are
  not half-word aligned goes to the pump. Frames in a byte buffer: one
  byte per 8-bit frame, two bytes low-first per 16-bit frame - on this
  little-endian core, a half-word in memory.
- `Placement`, the last parameter: util/placement.hpp's tag, `InFlash`
  by default, `InRam` making `start()`, the pump and the DMA line's body
  one flattened instance each in SRAM ([platform.md](platform.md), "A
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
  read; `Pin<n>::ref()`) was born with this driver, as on the other
  families; `Pin<n>::status()` (GPIO_STATUS: what reaches the pad, the
  output enable among it) is how the bench reads a pad's drive.

## How to use it

```cpp
constexpr brio::SpiPins pins{.sck = 18, .tx = 19, .rx = 16};
using Bus = brio::SpiHost<0, pins>;
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
using Cs = brio::Pin<17>;

Bus::init(clock, 8'000'000);                          // a ceiling for the whole bus
Cs::output(true);

Bus::Request r{};
r.cs = Cs::ref();
r.cmd = brio::lend<brio::Lease::reply>(cmd);  r.cmd_len = 1;
r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
r.clock = brio::SpiClocks::div4;                      // 31.25 MHz at 125
r.mode = brio::SpiMode::mode0;
r.reply = brio::reply_to<Display, brio::SpiDone>();
brio::post<Spi>(r);

extern "C" void isr_spi0() {
    if (Bus::isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

With the engines: `SpiHost<0, pins, DmaTxEngine<4, uint16_t>,
DmaRxEngine<5, uint16_t>>` (half-word engines, so a display's 16-bit
pixels ride them too) and `isr_dma_0` calling `Bus::dma_isr()` the same
way. A client:

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

A host for such a client in modes 0 and 2 (on another board, or the
other instance), the select its own frame signal:

```cpp
constexpr brio::SpiPins framed{.sck = 18, .tx = 19, .rx = 16, .cs = 17};
using Feeder = brio::SpiHost<0, framed>;              // Feeder::frame_select
Feeder::init(clock);
Feeder::Request r{};                                  // r.cs left null
r.tx = brio::lend<brio::Lease::reply>(frames); r.len = 64;
r.mode = brio::SpiMode::mode0;
r.polled = true;
(void)Feeder::start(r);
```

## Bench findings

The reference suite is `test_rp2040_spi`, green on the WeAct board,
clk_sys and clk_peri at 125 MHz: 52 verdicts, 29 of five letters on the
loop-back (LBM, no wire) and 23 of six on the pair's wire against the
desk partner's peer (the last group below).

- The rate chooser: clk_peri / 2 at the top (62.5 MHz), 8.93 MHz under
  a 10 MHz ceiling, 992 kHz under 1 MHz, nothing under 1 kHz. The
  block's reset state: both FIFOs empty, disabled; a 3-bit and a
  17-bit frame, an odd prescaler and a host with its output disabled
  are refused, a 12-bit TI configuration is taken, nothing is
  configured while enabled.
- THE LOOP-BACK ON THE PUMP: the four modes at 8 and 16 bits, sixteen
  frames each, byte-exact in three or four interrupts (the FIFOs batch
  them); a two-frame command phase then eight data frames, the echo
  discarded; a read with no out buffer clocks 0xFF; the select
  released after every transaction; an empty request completes on the
  spot.
- THE POLLED PATH at every named rate carries 64 frames byte-exact;
  the suite times the whole request on the system timer, 37 us at
  clk_peri / 2 and 32 us at / 4 against a wire of 8 and 16, a request's
  own cost on top of the loop's. The loop's cost a frame is the next
  item's.
- **THE HOST ABOVE THE WIRE, `bench_rp2040` letter e** - the polled loop
  in both its shapes, the pump, the price of a request - on a host with
  no engines, SPI0 on the same pads with GP17 the select and GP22 the D/C
  line, MISO floating, the receive-overrun flag read after every run and
  clear on every one; the ruler SysTick's cycles at 125 MHz, the best of
  five, at clk_peri / 2, / 4 and / 16 (62.5, 31.25 and 7.8125 MHz) in
  mode 0 and in mode 3, each figure a frame by the difference of 16 and
  256 frames:
  - THE RECEIVE SHAPE (`spi.poll.rx`, eight frames in flight - written and
    not yet read): **37 cycles an 8-bit frame and 44 a 16-bit one are the
    Cortex-M0+'s floor** - the loop's three accesses of the block a frame,
    the frame stored, the pointers - so the loop sets the pace at 62.5 MHz
    in both widths (the wire 16 and 32, 19 and 35 with mode 0's gap) and
    at 31.25 MHz on 8-bit frames in mode 3 (37 against 32); everywhere
    else it reads the wire: 37.93 against mode 0's 38 at 31.25 MHz, 70 /
    64 for 16-bit frames there, 152 / 128 and 280 / 256 at 7.8125. A frame
    read and thrown away instead of stored costs 35 (8-bit) and 39
    (16-bit).
  - THE WRITE SHAPE (`spi.poll`, no in buffer, taken for 8-bit frames at
    clk_peri / 2 on a run longer than the FIFO): the TNF-paced loop at
    **21.00 cycles a frame in both modes** against the wire's 16 and mode
    0's 19 - two accesses a frame, the Cortex-M0+'s floor again, x 1.31
    the nominal wire where the receive shape would be x 2.31.
  - THE RECEIVE SHAPE UNDER A LONG HANDLER: 100 receive requests of 256
    frames on the loop-back at each of the three rates with the tick's
    handler stretched to 1500 cycles - eight frames being 128, 256 and
    1024 cycles - **100 exact and 0 overruns at every rate**, with 14, 13
    and 32 of the stretched ticks landing inside the requests.
  - THE PUMP (`spi.pump`, the receive on the interrupt): **87.25 to 87.75
    cycles an 8-bit frame and 92.9 to 93.4 a 16-bit one at 62.5 and 31.25
    MHz**, 75 of them inside the handler (19300 cycles for 256 frames at
    62.5 MHz), one interrupt per FIFO level - 32 for 256 frames, where a
    handler finds the eight in flight all back; at 7.8125 MHz 49 to 51
    interrupts and the wire's own figures (152 / 129, and 280 / 256 for
    16-bit frames); the overrun flag clear on every run. Above 7.8 MHz
    the handler and not the wire is the pace: 256 frames at x 5.55 the
    nominal wire at 62.5 MHz, x 2.76 at 31.25.
  - THE PRICE OF A DCS COMMAND (`spi.req`, a command byte with D/C low and
    0, 2 and 15 data bytes with it high, polled, nothing read back, at
    7.8125 MHz): wall less the wire's cycles **323 for one byte, 462 for
    three, 783 for sixteen**, of which the block's mode-0 gap is 24 / 72 /
    384 and the phase boundary's drain (the data frames wait for the
    command frame to come back whole, so D/C flips on the wire) one
    frame's latency.
- **`dma_min_frames` IS 13 IN THE TRAITS, THIS CHIP'S OWN CROSSOVER.** At
  31.25 MHz an engined data phase costs 902 cycles over its frames, gap
  included, then 38 a byte (letter d, two items down), a pumped one 284
  cycles then 87.25 a frame (letter e): the two are equal at 12.5
  frames. The Cortex-M0+'s handler, not the launch, is the dearer term
  here, which is why the quotient is larger than the RP2350's ten and
  not smaller.
- THE ENGINES on the loop: a 128-byte block at 31.25 MHz, ISR-completed
  and exact with ONE DMA interrupt; a command frame on the pump then 32
  data frames on the engines; a polled request on the engines completing
  inside start() at 62.5 MHz; a read with no out buffer through the
  transmit engine's fixed 0xFF cell; 16-bit frames riding the half-word
  engines exact with no SPI interrupt and one DMA interrupt; and a 16-bit
  request on buffers one byte off a half-word going to the pump, exact
  there. The letter's first engined block, timed, takes **257 us** where
  letter d's walls give 46 warm and the wire 33: the cache's fill of what
  an engined transaction runs from the flash, the host's code included in
  the default placement (the placement item below splits it).
- **WHAT AN ENGINED TRANSACTION COSTS, `bench_rp2040` letter d**, on the
  loop-back, the half-word engines on channels 4 and 5, the core asleep
  in the idle path between the start and the completion, the best of
  four: 16 8-bit frames at 31.25 MHz take **1510 cycles, 826 of them
  busy** - the DMA line's handler 170 of those with the meter's own 47,
  where a 16-byte copy on the same loop is busy 279 with a handler of 79,
  so the SPI request's `start()` costs some 450 cycles more than a copy's
  launch - and the transaction's **FIXED COST, the wall less its frames'
  nominal wire, is 998 cycles (8.0 us), of which 96 the block's mode-0
  gap**: 902, 860 at 3.906 MHz and 919 for 16-bit frames. INSIDE THE
  BLOCK THE ENGINES RUN AT THE WIRE WITH THE BLOCK'S GAP, by the
  difference of 16 and 256 frames: **38.00 cycles a byte at 31.25 MHz**
  (32 of bits and 6 of gap), 304.00 at 3.906 MHz (256 and 48), 35.00 a
  byte in 16-bit frames (64 and 6 a frame) - 256 frames at x 1.29 and x
  1.20 of the nominal wire, 4096 16-bit frames at x 1.09 with the
  transaction's one interrupt.
- THE RATE ASKED, which a loop cannot tell (`test_rp2040_spi` letter d,
  a frame inside an engined block timed by the difference of a 2048- and
  a 256-frame polled block, neither buffer named): 1000, 999 and 999
  thousandths of eight SCK periods at clk_peri / 4, / 16 and / 64 in
  mode 3, 999 of nine and a half in mode 0 - the frame gap - and 999 of
  sixteen in 16-bit frames, each against clk_peri over the divisor
  asked; and no polled rate of letter c faster than its wire.
- **WHERE THE HOST'S HOT PATH RUNS IS THE PROGRAM'S CHOICE, and the
  choice is about the COLD request.** `SpiHost`'s last template parameter
  is util/placement.hpp's tag: `InFlash`, the default, leaves `start()`,
  the pump and the DMA line's body in the flash (the pump inline in the
  app's vector), `InRam` makes each ONE flattened instance in
  `.fast_text.brio`, copied to SRAM with `.data` - the engines' own verbs
  and the launch folded in, so an engined transaction's driver code is in
  SRAM whole ([platform.md](platform.md), "A handler in SRAM", has the
  table and the rule). `bench_rp2040` letter e's `spi.cold` is the
  instrument for the request: `spi.req`'s three-byte polled request
  after every line of the XIP cache is flushed (`Xip::flush()`), under
  the mask, the cache's misses (accesses less hits) read around it -
  **11363 cycles and 136 misses with the host in the flash, 2942 and 41
  with it InRam, against 791 and 803 warm**; the 41 are the caller's
  (the ruler's reads, the call site, the veneer). Warm the two read the
  same but for the veneer, 13 cycles a request. The engined block cold
  (letter d, 16 frames at 31.25 MHz, warm 1450 to 1630) splits the same
  way: 34002 cycles from the flash, 18028 with the host InRam, 22782 with
  the vectors placed, 10768 with both - the host's code some 16000 of
  the difference, the vectors' some 11000, the caller's the rest. THE
  SRAM IT COSTS under InRam: `start()` 944 bytes without engines and 1344
  with them, the shared engine launch 568, the pump 268 to 320, the DMA
  line's body 540. Every call across the boundary goes through
  a veneer (a push, a literal load, a move into ip, a pop, a `bx`): the
  Cortex-M0+'s `bl` reaches 16 MB, and the SRAM lies 256 MB from the
  flash.
- THE KERNEL over it, unchanged: four transactions through SpiBus
  with four replies, pumped and polled interleaved on one bus; six
  posted into a four-deep queue, one rejected at once and every
  request answered once; an idle bus votes for the sleep, a busy one
  against.

ON THE PAIR'S WIRE (letters n, o, p, q, r and m, the two Raspberry Pi
suites' shared letters, against the WeAct RP2350B's peer on either half: its client a
PIO program at 150 MHz, its host that chip's PL022):

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
  both ways to clk_peri / 10 (12.5 MHz, a half period of 40 ns)**; at /8
  (15.6 MHz) this host reads the client's answers wrong while the client
  still hears this host exact up to /4 (31.25 MHz) - the answer path is
  the bound, some four of the client's cycles (26.7 ns) and the pads. The
  letter judges to /12, the derived ceiling (a half period of four of the
  partner's cycles and 15 ns, derived in the suite); the engines
  on a wire against a foreign client exact wherever the polled loop is.
- THE KERNEL against the second chip: four transactions queued from one
  dispatch inside one held window, one polled among three pumped, every
  byte read back by the peer.
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
  Ten runs of the letter. The body runs from SRAM (the program's
  `.fast_text`): from the flash, its first entry after a quiet spell is
  the XIP cache's fill and comes several frames late - the receive FIFO
  overflowed at /64 and the transmit FIFO ran dry at /16, the far host
  reading 0x00 from the ninth answer on. Even with the body in SRAM the
  vector's own lines and the veneer into SRAM miss once: the first burst
  after a flash broke at /64 in 8-bit frames (the ninth answer late),
  the letter's later bursts never - so the letter's first burst is a
  warm-up, printed. Past the ceiling, printed: /14 (10.7 MHz) exact in
  every setting of ten runs, /12 (12.5 MHz) exact but in mode 3 with
  8-bit frames, which failed in each.
- THE CLIENT'S OVERRUN (letter s): eight answers preloaded, nothing
  taken, 24 frames from the far host - the receive FIFO keeps the first
  eight exact, and the vector is entered on RORIM sixteen times, once a
  frame lost; the far host reads the eight preloaded answers exact.
- THE ARBITER'S PER-BUS TIMEOUT (letter w, a completion that never comes
  staged by running the pump's ISR body and posting nothing): answered
  `spi_timeout` 99.2 to 99.9 ms after the request against the arbiter's
  100 ticks, and recover() leaves the engine where the next four
  transactions on the same bus AO - three pumped, one polled - run to
  `spi_ok`. Ten runs.
- THE CLIENT'S ANSWER PAD (letter m, its output enable read in
  GPIO_STATUS): DRIVEN while deselected as while selected, and under SOD
  - the far host then reading the client's FIRST answer in every frame,
  0x93 x 32 - which is 4.4.3.14's nSSPOE "not connected to the pad"; the
  dark listener takes the pad away and the far host reads its own pull-up
  throughout, the client still hearing every frame.
- THE DARK LISTENER'S PAD WAS A DRIVER BUG, found on the RP2350's side of
  this pair: released to its reset state, the pad kept its pull-down
  against the far host's pull-up, and the line sat between the two (the
  RP2350 host read 0x02 where it should read 0xFF); this chip's pulls
  happened to read high. `drive_output(false)` now gives the pad to no
  function with no pull (the traits' `no_function`, `dark_pad_config()`).

## Not covered yet

Driver gaps, each with its reason:

- The TI and Microwire framings on the wire: configured and read back
  in the resource, no device on the desk speaks them.
- Frame widths other than 8 and 16 in the Request: the other strata's
  Request speaks two, the resource takes 4..16; a device with a
  12-bit frame is the first user.
- The client on the DMA engines: `SpiClient` answers from its
  interrupt; a client that streams is born with a program that needs
  one.

Implemented but not bench-verified, each with what would measure it:

- The vendor's loop on this board: the pinned pico-sdk's
  `spi_write_read_blocking` and `spi_write_blocking`, SRAM-resident by
  the SDK's own placement, against letter e's 37 and 21 cycles a frame
  at clk_peri / 2 - a program built by the SDK's own CMake and timed on
  the same ruler, as on the RP2350 ([../rp2350/spi.md](../rp2350/spi.md)).

- `cs_setup_us`: a device that needs it, or a scope on the select and
  the first clock.
