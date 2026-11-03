# DMA (RP2040)

This document is for two readers. Most programs meet the DMA through a
driver: the UART, SPI and I2C transports each take two engine parameters,
template arguments that name a transmit and a receive engine, and a DMA engine
named there moves that driver's data while the core does something else ([uart.md](uart.md), [spi.md](spi.md),
[i2c.md](i2c.md); [design/dma.md](../design/dma.md) is the contract
the engines keep). The other reader drives the controller directly: a
memory copy, a table poured into a register at a timer's pace, a
checksum computed as data goes by. `brio/rp2040/dma.hpp` serves both:
`Dma` the block, `DmaChannel<ch>` one of the twelve channels,
`DmaLine<n>` one of the two interrupt lines, `DmaTimer<n>` a pacing
timer, `DmaSniffer` the checksum unit, and the engines - `DmaTxEngine`,
`DmaRxEngine` and `DmaCopyEngine`, a channel bound once to its job and
started block by block.

The smallest use, a copy; an excerpt, the tag and the claim at namespace scope, the
rest in a function, `src` and `dst` the program's:

```cpp
struct CopyTag {};
static_assert(brio::dma_claim<brio::DmaSlotId<0, 4>, CopyTag>);
using C = brio::DmaChannel<4>;

if (!brio::Dma::init()) { /* the block did not come out of reset */ }    // once
if (!C::load({.read = src, .write = dst, .count = 1024, .config = {.size = brio::DmaSize::word}})) { /* refused */ }
while (C::busy()) {}
```

Line by line: the `static_assert` claims channel 4 for the program's tag, so a
second owner on it does not compile ("One owner a channel" below); `C` names the
channel; `Dma::init()` releases the controller from reset, once per program, and
answers false when it did not come out; `load()` programs the transfer - the
two addresses, the count in items, the width of an item - and starts
it, here 1024 words from `src` to `dst` as fast as the bus allows (no
request is named); the loop waits for the channel to finish. `load()`
answers false, starting nothing, on a channel still busy and for a
transfer `dma_transfer_valid` refuses - an end missing, an address not
aligned to the item, a zero count, a ring of 16 bits or more, a chain
to itself or to a channel that does not exist.

What decides correctness even there. `Dma::init()` comes first and
once: no channel verb and no engine releases the block. The buffers
belong to the transfer until it completes: the controller reads and
writes them on its own, so neither end is touched or goes out of scope
before `busy()` reads false or the completion's interrupt has run. **A
channel has one owner for the whole image and a request one channel**,
which the compiler checks for what claims them ("One owner a channel"
below). A peripheral FIFO a channel serves is the channel's: the processor
does not touch it meanwhile. A channel reports on the line the program routes it to, and a line's
enable acts on the NVIC of the core that calls it - line 0 is core 0's
by convention, line 1 core 1's ([multicore.md](multicore.md)). And a
channel stalled on a request that never comes stays busy until
`abort()`, which carries erratum E13's workaround.

The page is long because the chapter is. In order: the documents of
record; what the silicon does, with what the engines take of it and
the priority bit; the types and verbs, with what a block costs; how to
use it - a copy, a paced transfer, a line, a checksum, the copy engine,
a transmit engine on a timer, a transport with engines; the bench
findings; and what is not covered.

## Documents of record

The RP2040 datasheet (build 3184e62), 2.5 (the
controller: 2.5.1 the four live registers, 2.5.2 the trigger aliases,
chaining and null triggers, 2.5.3 the credit-based data request and
table 119, 2.5.4 the two lines, 2.5.5 the pacing timers, the sniffer
and the abort, 2.5.7 the registers), 4.2.5 (the UART's DMA interface);
Appendix B, RP2040-E12 and E13. The driver: `brio/rp2040/dma.hpp`
(the block, a channel, a line, a timer, the sniffer, the transfer
engines and the copy engine) with the request numbers in
`dma_engine.hpp` beside `NoDmaEngine`, the empty parameter's tag; the transfer engines'
users are the UART's, the SPI's and the I2C's transports (the IP
strata `brio/pl011/`, `brio/pl022/`, `brio/dw_apb_i2c/`) and the ADC,
PIO and PWM suites. The reference suite: `test_rp2040_dma`, whose
console prints through a transmit engine; the engines' cost is
`bench_rp2040`'s letter d.

## What the silicon does

Twelve channels over one read master and one write master, each
channel four live registers - READ_ADDR, WRITE_ADDR, TRANS_COUNT,
CTRL - aliased four times so that any of them is the TRIGGER in one
alias: write the trigger register non-zero with EN set and the
channel starts; a zero written there (a null trigger) starts nothing.
A channel completes and stays ENABLED, its BUSY flag down; clearing
EN pauses it and the abort register ends it. Transfers are 8, 16 or
32 bits wide, the same on both sides, at one read and one write per
clock; each address increments or not, and one side may wrap at a
power-of-two boundary (RING_SIZE, RING_SEL). The transfer request is
a FIELD, TREQ_SEL: any channel takes any of the forty peripheral
requests of table 119, one of four pacing timers, or none (the
transfer as fast as the bus allows). The request is credit-based: a
pulse per unit of room or data at the peripheral, a counter per
channel, and the DMA keeps that many transfers in flight - which is
why a FIFO served by a channel must not be touched by the processor,
and why two channels must not share one request. A completion raises
the channel's raw interrupt bit; two system lines, DMA_IRQ_0 and
DMA_IRQ_1, each with its own enable and masked-status bank, take any
subset of channels. CHAIN_TO names a channel to trigger at
completion; IRQ_QUIET turns the interrupt into "on a null trigger
only", the end of a control-block chain. A bus error halts the
channel, sets AHB_ERROR with READ_ERROR or WRITE_ERROR (cleared by
writing one) and raises the interrupt. The sniffer watches one
channel's data go by and keeps a CRC-32, a CRC-16-CCITT, a sum or a
parity, with bit-reverse, invert and byte-swap on the result.

Two errata. E12: READ_ADDR and WRITE_ADDR read wrong while a
non-incrementing or wrapping sequence is in progress - progress is
TRANS_COUNT's. E13: a channel aborted with transfers in flight
reports a completion when they land - the interrupt enable is cleared
before the abort, BUSY awaited, the status cleared, the enable
restored.

What a channel offers, and what the engines take of it. A channel's
registers are live and keep what was last written: an address that does not
increment is still the next sequence's start (2.5.1.1), a completed
channel keeps CTRL with EN set, and TRANS_COUNT reloads from the last
value written. So an engine binds its channel once - the peripheral's
data register written once, CTRL composed once with the request, the
priority, CHAIN_TO at the channel itself and EN - and a block is the
count and the memory address, the last of the two written through the
alias whose final word is a trigger (2.5.2.1's tuples: TRANS_COUNT then
READ_ADDR_TRIG to a peripheral, WRITE_ADDR then TRANS_COUNT_TRIG from
one): two stores, the third (CTRL) only when the block's width or step
differs from the word the channel holds. DATA_SIZE is one field for
both sides, so one channel carries bytes, half-words and words, block by
block; narrow writes are byte-lane replicated (2.5), which is why a byte
beat into the PL022's half-word data register delivers the frame.
IRQ_QUIET (2.5.2.3) silences a completion while a bus error still
raises the channel's flag (CTRL.AHB_ERROR: the channel "always raises
its channel IRQ flag"): a transmit engine whose completion another
block proves - the SPI host's beside its receive - reports errors
alone, one interrupt a transaction. A zero count is never written: a
zero into a trigger register is a null trigger. Memory to memory is the
controller's own case (TREQ_SEL PERMANENT, one read and one write a
clock). Declined, each for its reason: the address ring and CHAIN_TO in
the engines (one block per start; the circular receive is a ring on the
write side with a second channel chaining back, this chip having no
self-trigger - not built), MULTI_CHAN_TRIGGER (a transport's pair
starts receive first and transmit second with the peripheral's requests
raised between them, an order one mask write would not keep), the
sniffer (not an engine's business). The gate - the block out of
RESETS - is `Dma::init()`, once per program; no engine touches it.

The level is one bit, CTRL.HIGH_PRIORITY (2.5.7): "in each scheduling
round, all high priority channels are considered first, and then only a
single low priority channel, before returning to the high priority
channels". The engines take docs/design/dma.md's rule in that one bit: a
receive engine arms HIGH by default (its peripheral overruns when the
channel is starved), a transmit engine and the copy engine normal, and a
transport names its engines' levels at the call. On this chip the bit
decides no correctness: nothing starved a receive ring at normal (letter
p, below), and the Uart takes no level option.

## One owner a channel

A DMA channel has one owner for the whole image, and a peripheral request
one channel ([design/dma.md](../design/dma.md), "The ownership rule"). The
compiler checks both, through `util/dma_owner.hpp`. What it calls a DMA slot is
the channel, `DmaSlotId<0, ch>` (this controller is not numbered, so the first
number is zero), and the transmit and receive engines publish it as `dma_slot`.

On this chip a request is a field any channel takes, handed at `arm()` (DREQ
numbers, table 119), so an engine's type does not know its owner and cannot claim
for itself. Whoever hands it the request does:

- **A transport claims when its type is instantiated.** `Uart<n, ...>` claims
  each engine's channel for `DmaOwner<DmaPeriph::uart, n, DmaSignal::tx>` or
  `rx`, `SpiHost<n, ...>` for `DmaPeriph::spi`, `I2cHost<n, ...>` for
  `DmaPeriph::i2c`; an engine parameter left at `NoDmaEngine` claims nothing. A program writes
  nothing for these. Two transport types over one instance's request on one
  channel are one owner and pass, whatever their element types and lines.
- **`DmaCopyEngine<ch>` claims its channel for a tag of its own**,
  `DmaCopyOwner<ch>`.
- **An engine a program arms itself, or a channel it programs by hand, is
  claimed by the program**, in the source where it names them:

  ```cpp
  // Excerpt: a level table poured into PWM slice 3's compare register on its wrap.
  using Stream = brio::DmaTxEngine<6, uint32_t>;
  static_assert(brio::dma_claim<brio::DmaSlotId<0, 6>,
                                brio::DmaOwner<brio::DmaPeriph::pwm, 3, brio::DmaSignal::update>>);

  // Excerpt: a channel programmed by hand, under a tag the program names.
  struct ByHand {};
  static_assert(brio::dma_claim<brio::DmaSlotId<0, 4>, ByHand>);
  ```

  The owner names a PIO state machine as its block times four plus its number,
  a pacing timer as `DmaPeriph::tim` with `DmaSignal::update`, the ADC as
  `DmaPeriph::adc`; a request the vocabulary lacks takes a tag (the XIP
  stream's). Nothing checks that a program wrote this claim: an engine armed
  with no claim is invisible to the check, which is the programmer's duty and
  not the compiler's.

A second owner on a claimed channel does not compile. UART1's transmit on
channel 2 beside an SPI0 host on the same channel reads
`ambiguating new declaration of 'constexpr brio::DmaOwner<brio::DmaPeriph::spi, 0, brio::DmaSignal::tx> brio::dma_owner_of(DmaSlotId<0, 2>)'`
with UART1's transmit on the note under it. A second channel for a request does
not compile either, which is this chip's own rule ("multiple channels should not be connected to the same DREQ", 2.5.3.2): the
same transmit on channel 6 after channel 2 reads
`ambiguating new declaration of 'constexpr brio::DmaSlotId<0, 6> brio::dma_slot_of(DmaOwner<brio::DmaPeriph::uart, 1, brio::DmaSignal::tx>)'`
against the old declaration's `DmaSlotId<0, 2>`. A program cannot hand a channel
from one peripheral to another as it runs: where both are claimed, that is the same
refusal, and where they are not, the programmer's duty above. The fixtures
`test/family_rp2040/neg/dma_*` hold each shape. The claims add no byte and no cycle.

**What a release leaves.** `release()` on a UART, SPI or I2C transport stops its
engines, takes its interrupt line down, gives the pads back and holds the block in
reset, and the ADC's stops and holds it too. Its `init()` takes the block through
that reset before it binds a channel, so a request the block raised before the
release does not reach the next run ([design/dma.md](../design/dma.md), "The release
contract"; letter r below measures it). A PWM slice and a PIO state machine have no
reset of their own, the block's being every slice's and every machine's: a stop
releases them, and the next trigger of a channel drops a slice's banked credits.

**The map forces no choice here.** Any request reaches any channel, so nothing
is chosen for a program but the channel numbers, and the only limit is their
count: twelve channels, against the ten a program uses with a console and a second
UART with both engines each, an SPI host's two, an I2C host's two (`I2cHost` takes
both engines or neither), one ADC stream and one paced stream. The one narrower fact
is the DREQ rule above: a converter's or a state machine's one request stays on one
channel, so a program that reads it at two widths uses one engine at the wider beat
and hands it a run of the narrower element where it wants the narrower width (a run's
beat is at most the engine's element). No part of this
family has a forced instance or an unavoidable group.

## Types and verbs

- `DmaSize`, `dma_size_of<Elem>()`, `Dreq` (table 119 by name, the
  four timers, `permanent`), `DmaChannelConfig` (size, the two
  increments, ring bits and side, the request, `chain_to`, high
  priority, byte swap, sniff, IRQ_QUIET), `DmaTransfer` (both ends,
  the count in items, the config) with `dma_transfer_valid` (both
  ends present and aligned to the size, a non-zero count, a legal
  config: a ring under 16 bits, a chain to another existing channel),
  `dma_ctrl_word`, `DmaError`, `DmaProgress`.
- `Dma`: `init()` (the block out of the reset controller, once),
  `channels()` (N_CHANNELS), `raw()` / `clear_raw()`, `trigger(mask)`,
  `abort_raw(mask)` (E13 not handled: the channel's verb is).
- `DmaChannel<ch>` (0..11): `prepare(t)` (programmed, not started),
  `load(t)` (programmed and triggered), `trigger()`, `null_trigger()`,
  `configure`, `set_read` / `set_write` / `set_count`, `enable(on)`
  (pause), `enabled`, `busy`, `count()` live, `progress(programmed)`,
  `errors()` / `clear_errors()`, `raised()` / `clear_raised()`,
  `route(line, on)` / `routed` / `pending(line)` / `clear_pending`,
  `abort()` (E13's workaround as one verb), `stop()`, the debug pair
  `debug_credits()` / `clear_credits()` and `debug_reload()`. Every
  programming verb refuses a BUSY channel.
- `DmaLine<n>` (0, 1): `irq()`, `routed()`, `pending()`, `clear(mask)`,
  `force(mask, on)`, `enable()` / `disable()` in the calling core's
  NVIC - the line a channel reports on is the core's whose kernel
  owns it, line 0 for core 0 by convention.
- `DmaTimer<n>` (0..3): `set(x, y)` (a request every y/x clk_sys
  cycles), `stop()`, `dreq()`.
- `DmaSniffer`: `start(channel, DmaSniffConfig, seed)`, `result()`,
  `stop()`; `DmaSniffCalc`.
- The engines' vocabulary: `DmaReport` (`blocks`: every completed
  block reaches the line; `errors`: IRQ_QUIET, a bus error alone),
  `dma_binding_word` (CTRL's part constant for a binding),
  `dma_beat_bits<T>`, `dma_beat_aligned<T>`, `dma_run_valid<T>` (a
  run, a non-zero count, an address aligned to its beat),
  `dma_handoff_fence` (a compiler fence before every trigger store),
  `DmaBinding<ch>` (the CTRL word the channel holds, kept per channel,
  rewritten only when a block's differs; the shared `service()` body).
- `DmaTxEngine<ch, Elem, line>` / `DmaRxEngine<ch, Elem, line>`: the
  transports' engines. `Elem` is the WIDEST beat the binding allows
  (the data register's width): a run of `uint8_t`, `uint16_t` or
  `uint32_t` up to it is legal, a wider one a compile error, one whose
  address is not aligned to its beat refused at run time, nothing moved
  (the SPI host tests the alignment first and answers `spi_rejected`:
  [spi.md](spi.md)). `arm(data, dreq, high_priority, report)` binds once,
  `high_priority` true by default on a receive engine and false on a
  transmit engine;
  `Report` names `DmaReport` through the engine. Transmit: the run-time verb
  `claim()` (a test-and-set for "is this channel free to start a block", which a
  transport takes under its mask, and no relation of the compile-time claim above),
  `unclaim()`, `launch(run)` / `launch_fixed(cell, n)` once `claim()` has answered true,
  `start(run)` / `start_fixed(cell, n)` = `claim()` and `launch()`, a run being
  a span or a pointer and a count; `complete()` (the beats the block
  carried), `busy()` (claimed or running), `in_flight()`,
  `progress()`. Receive: `start(run)`, `start_discard(cell, n)`,
  `idle()`, `take()` (beats arrived since the last take, from
  TRANS_COUNT), `full`, `capacity`, `taken`. Both: `service()` (this
  channel's status on its line, cleared and handed back as
  `flag_complete` / `flag_error`; no half event on this controller),
  `abandon()`, `faults()`, `stop()`. A receive engine clears the
  channel's request credits before every run.
- `DmaCopyEngine<ch, line>`: `arm(report, high_priority)` (normal by
  default; `blocks`:
  the completion on the line, `service()` its body; `errors`: polled,
  the channel on no line), `copy(dst, src, n)` and `fill(dst, cell, n)`
  in elements of the type the pointers carry, which is the beat;
  `busy()` (CTRL.BUSY), `errors()` / `clear_errors()`, `remaining()`,
  `stop()`. A block over a running one, an empty one and a misaligned
  one are refused.
- In the transports (`uart.hpp`, `spi.hpp`, `i2c.hpp` over the IP
  strata): the engine parameters, `dma_isr()` (the line's ISR body), the UART's
  `harvest()` and `dma_faults()`.

What a block costs, counted in the release listing (Cortex-M0+
timings, one path, the CTRL word unchanged): a transmit block start
from a transport - `claim()`, the ring's run, the block - 51
instructions and about 100 cycles with TWO stores to the DMA block,
the mask held over `claim()` alone (six instructions, about 10
cycles); a receive run 27 instructions, about 60 cycles, one read
(BUSY) and three stores (the credit clear, the address, the count);
an SPI data phase on two engines 89 instructions, about 180 cycles,
and ONE DMA interrupt for the transaction - the block start alone: a
whole engined request of 16 frames at 31.25 MHz costs 904 cycles over
the wire, the host's start(), the completion and the block's mode-0
gap on top of this launch (bench findings below, [spi.md](spi.md)); a copy 25 instructions,
about 40 cycles, one read and three stores.

## How to use it

Each block below is an excerpt on a channel of its own, claimed for one owner, so
all of them can live in one image; one channel does not take these jobs in turn. They
assume `Dma::init()` has run, and the program supplies the buffers, the register and
`clock`. A program has one `isr_dma_0`, so where two blocks show it, its body holds both.

The copy of the first example, then a table poured into a register at one word a
microsecond, on channel 5 under pacing timer 0's request:

```cpp
static_assert(brio::dma_claim<brio::DmaSlotId<0, 5>,
                              brio::DmaOwner<brio::DmaPeriph::tim, 0, brio::DmaSignal::update>>);
using Paced = brio::DmaChannel<5>;

brio::DmaTimer<0>::set(1, 125);                           // one request a microsecond at 125 MHz
if (!Paced::load({.read = table, .write = &pwm_level, .count = 1000,
                  .config = {.incr_write = false, .treq = brio::DmaTimer<0>::dreq()}})) { /* refused */ }
```

A checksum of a load, with its completion on a line:

```cpp
struct SumTag {};
static_assert(brio::dma_claim<brio::DmaSlotId<0, 6>, SumTag>);
using Sum = brio::DmaChannel<6>;

extern "C" void isr_dma_0() {
    if (Sum::pending(0)) { Sum::clear_pending(0); /* ... */ }
}
Sum::route(0, true);
brio::DmaLine<0>::enable();
brio::DmaSniffer::start(6, {.calc = brio::DmaSniffCalc::crc16}, 0xFFFF);
if (!Sum::load({.read = buf, .write = &sink, .count = n,
                .config = {.incr_write = false, .sniff = true}})) { /* refused */ }
```

Copy and fill on one channel, the beat the element type. The engine claims its own
channel:

```cpp
using Copier = brio::DmaCopyEngine<8>;
Copier::arm();                                            // the completion on line 0
extern "C" void isr_dma_0() {
    if ((Copier::service() & Copier::flag_complete) != 0u) { /* done */ }
}
if (!Copier::copy(framebuffer_words, tile_words, 1024u)) { /* refused */ }  // 4 KB as words
// ... and when that block has ended, one 16-bit cell:
if (!Copier::fill(framebuffer_halves, &background, 320u * 480u)) { /* refused */ }
```

A table into one register at a timer's pace through an engine: a transmit engine
whose request is pacing timer 1's, which the program claims since it hands the
request over:

```cpp
static_assert(brio::dma_claim<brio::DmaSlotId<0, 9>,
                              brio::DmaOwner<brio::DmaPeriph::tim, 1, brio::DmaSignal::update>>);
using Wave = brio::DmaTxEngine<9, uint32_t>;

Wave::arm(&pwm_level, brio::DmaTimer<1>::dreq());
brio::DmaTimer<1>::set(1, 400);                           // a word every 400 clk_sys cycles
if (!Wave::start(std::span<const uint32_t>(table))) { /* refused */ }
```

A transport with an engine in each parameter; the transport claims both channels:

```cpp
using Link = brio::Uart<1, pins, 1024, 1024, brio::DmaTxEngine<2>, brio::DmaRxEngine<3>>;
extern "C" void isr_dma_0() { (void)Link::dma_isr(); }    // true on a published run: this program polls harvest()
...
if (!Link::init(clock, 3'000'000)) { /* the rate cannot be made, or the block did not come up */ }
const uint32_t queued = Link::write_bulk(block);          // one DMA block per contiguous run of the ring
(void)Link::harvest();                                    // what arrived, published; a run re-armed
```

## Bench findings

All from `test_rp2040_dma` on the WeAct board, 52 verdicts in its `z`,
with the console itself on a transmit engine (every line below
travelled by DMA):

- The silicon counts twelve channels; a zero count, a word from an
  odd address, a chain to itself or to a thirteenth channel are
  refused. A channel waiting on a request that never comes (a pacing
  timer at X = 0) stays BUSY at its full count with every programming
  verb refusing it, and `abort()` brings BUSY down, raises nothing,
  runs no handler and puts the route back.
- Memory to memory at byte, half-word and word width: exact,
  TRANS_COUNT at zero, the raw status raised and cleared by writing
  one. Four kilobytes as 1024 words take 19 us with this core polling
  BUSY on the same SRAM - not the 8 us of one transfer a cycle.
- A completion routed to line 0 is served there alone, once; the same
  channel routed to line 1 on the other line; under IRQ_QUIET a
  completion is silent and a null trigger raises exactly one.
- Chaining: a prepared channel runs at its predecessor's completion,
  both halves exact, both raised. A sixteen-byte ring read into
  sixty-four bytes lays the pattern four times.
- The pacing timer at X/Y = 1/125 moves one byte a microsecond: a
  thousand in 1009 to 1017 us.
- The sniffer's sum, its CRC-16 (util/crc.hpp's CRC-16-CCITT at the
  same seed) and its CRC-32 (the MSB-first 0x04C11DB7 polynomial from
  the seed given) match their software twins; OUT_REV and OUT_INV
  present the same result bit-reversed and inverted.
- A word block reading 0x40100000, where nothing answers: the channel
  halts at its full count, AHB_ERROR and READ_ERROR set, the
  interrupt raised, BUSY down; the bits clear by writing one and the
  channel copies again.
- A word block WRITING into the same hole: the channel halts with
  AHB_ERROR and WRITE_ERROR set, READ_ERROR clear, its line raised once
  - after two words, TRANS_COUNT reading 14 of 16 in each of ten runs
  (the writes are posted; the read side's error stops a block before it
  moves anything). The read error on a channel bound IRQ_QUIET - the
  SPI host's transmit engine's binding - still raises the line, once: a
  quiet channel is silent on its completion and never on an error
  (letter h, ten runs).
- AN ENGINE ON LINE 1 SERVED BY CORE 1 (`test_rp2040_multicore` letter
  i): the copy engine on channel 6, armed by core 1's kernel - `arm()`
  enables the line in the calling core's NVIC - copies 4 KB exact, its
  completion's vector runs on core 1 once and never on core 0 (core 0's
  NVIC never enabled DMA_IRQ_1), and the report crosses back through
  the inbox. Ten runs of the suite's z.
- THE ENGINES: 4096 bytes through UART1's loop-back at 3 Mbaud with an
  engine in each parameter, harvested by the loop, byte-exact in 13.8 ms
  (the wire's 13.65) with 22 to 23 interrupts on the line. A burst of
  2064 bytes four times the console's ring leaves in 47 blocks of its
  engine, the transport idle after, no fault.
- TWO TRAPS OF THE RECEIVE ENGINE, both now in the transport: a
  receiver enabled over a pad still at its reset pull-down (a wire
  from a silent peer) takes a BREAK, a zero byte the interrupt
  handler drops by its flags and an engine, which moves bytes and not
  flags, delivers at the head of the run - so the RX pad goes over
  with its pull-up before the receiver is enabled and the FIFO is
  emptied before the engine takes it; and THE REQUEST CREDITS
  OUTLIVE A SEQUENCE: while a completed run waited for the loop to
  re-arm it, the FIFO overflowed and kept pulsing, and the next run
  spent those credits reading an empty FIFO - zeros after the 32
  bytes the FIFO held. The completion now re-arms from the line's
  handler and the engine clears the channel's credit count before
  every run (a level request re-pulses for every byte still waiting,
  so nothing is lost).

- THE ENGINES MEASURED ON THE WEACT RP2040: `test_rp2040_dma` judges the
  binding and the two-store block, the beat per block, the run-time
  `claim()`, the copy engine at three widths; `bench_rp2040` letter d - a
  copy of 16 bytes wall 472 and busy 314 cycles with a 110-cycle
  handler in ten runs of ten, and an engined SPI request of 16 frames at
  31.25 MHz wall 1416 with ONE interrupt in four runs of ten (a second, the tick's,
  in a fifth; 1507 to 1975 in the other five), a fixed cost of 904 cycles (7.2 us) of
  which 96 the block's mode-0 gap; inside the block the engines run at the
  wire plus that gap, 38.00 cycles a byte at 31.25 MHz and 304.00 at
  3.906 MHz, 35.00 a byte in 16-bit frames, each in four to seven runs
  of ten ([spi.md](spi.md) has the other runs).
- THE LINE AT THE RATE ASKED, which a loop alone cannot tell (both ends
  share one divisor): 1024 frames through the transmit engine leave in
  1010 to 1016 thousandths of their wire time at 3 Mbaud (letter i), and
  in 1001 to 1004 at 115200 (letter v); on the pad, eight start bits at
  9600 lie seven frames apart within half a per cent (letter t, the
  first edge seen up to a poll late).
- THE LEVEL, letter p (`brio stress` the sender, the console's ring on a
  receive engine over 8 KB, two byte copies of 16 KB back to back beside
  it, a 1.5 s window a leg at 115200, 1 Mbaud and 3 Mbaud): every byte
  in order and no overrun with the ring at HIGH beside two normal copies
  and beside two HIGH_PRIORITY ones - AND with the ring at normal beside
  two HIGH_PRIORITY copies, one, or two normal ones. The arbiter serves
  one normal channel every round, and the bit decides no correctness on
  this chip - where the RP2350's arbiter, the same text over a different
  round, does ([../rp2350/dma.md](../rp2350/dma.md)).
- A RELEASED REQUESTER, THE SAME OWNER AGAIN, letter r (no wire), the release
  contract on UART1, whose channels 2 and 3 are its own for the whole image. The
  block is released the worst way: both DMA enables set, the channels stopped, four
  frames standing in the receive FIFO, the block held in reset with the enables
  left set. The transport's `init()`, then one block of sixteen frames at 1 Mbaud:
  sixteen back in order, none of the four, no fault (175 to 206 us on a wire of 160, ten runs of four rounds). THE
  CONTROL, the reset removed - the block disabled with the enables left set, the
  bring-up by hand, the channels bound as `init()` binds them: with the block still
  disabled the bound channel moves nothing (4.2.5: "all request signals are deasserted"), and with
  it enabled the receive channel's first four items are the four stale frames, four
  rounds of four. The PL011 keeps its FIFOs across a disable, so that is the request
  a reset at `init()` withdraws. Every transport's letter says the same of its own
  block: [spi.md](spi.md), [i2c.md](i2c.md), [adc.md](adc.md), [pwm.md](pwm.md) (a
  slice stopped and not reset, its credits dropped by the next trigger).

## Not covered yet

Driver gaps, each with its reason:

- Control-block lists (a channel programming another through the
  aliases, 2.5.6.2): born with a first stream that wants the DMA to
  reconfigure itself; `prepare()`, the aliases and `chain_to` are its
  building blocks.
- The loop and ping-pong engines of a block stream
  (util/block_stream.hpp), and the circular receive: on this controller
  each is two channels chaining to each other or a ring on one side with
  a second channel re-arming it - there is no self-trigger - and each is
  born with its first user.
- `DmaSniffCalc::crc32_reversed`, `crc16_reversed` and `even_parity`,
  and the BSWAP option: written from 2.5.5.2 and not measured; a
  letter with their software twins.

Implemented but not bench-verified, each with what would measure it:

- A PIO state machine released with its request standing and started again, as letter
  r stages the others: a machine's `init()` drains its FIFOs (`PioSm::init()`) and the
  block's reset is every machine's, so no reset is at issue. Read from the driver and not
  measured; a letter that leaves a word standing, runs `init()` and binds the channel
  on the machine's request would measure it.
