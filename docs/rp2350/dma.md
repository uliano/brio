# DMA (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 12.6 (the
controller: 12.6.1 what changed from the RP2040, 12.6.2 the four live
registers with the count modes and the address steps, 12.6.3 the trigger
aliases, chaining and null triggers, 12.6.4 the credit-based data request
and its table, 12.6.5 the four interrupt lines, 12.6.6 the security
levels and the memory protection unit, 12.6.7 bus errors and the halt
timing, 12.6.8 the pacing timers, the sniffer, the abort and the debug
registers, 12.6.10 the registers), 12.1 (the UART's DMA interface), 7.5
(the reset controller), 3.2 (the interrupt lines); Appendix E,
RP2350-E5 and RP2350-E8. The driver: `brio/rp2350/dma.hpp` (the block, a
channel, a line, a timer, the sniffer, the read-only MPU view, the three
engines) with the request numbers in `dma_engine.hpp` beside the empty
slot's tag. The engines' users are the transports the IP strata drive
with them - the PL011's ([../pl011/README.md](../pl011/README.md)), the
PL022 host's data phase ([spi.md](spi.md)), the DW_apb_i2c host's read
phase ([i2c.md](i2c.md)) - and the PIO, PWM and ADC chapters. The
reference suite: `test_rp2350_dma`, whose console prints through a
transmit engine, and which runs on both of this chip's architectures from
one source; `bench_rp2350`'s letter d prices the engines
([../design/benchmark.md](../design/benchmark.md)).

## What the silicon does

Sixteen channels over one read master and one write master, each channel
four live registers - READ_ADDR, WRITE_ADDR, TRANS_COUNT, CTRL - aliased
four times so that any of them is the TRIGGER in one alias: write the
trigger register non-zero with EN set and the channel starts; a zero
written there (a null trigger) starts nothing. A channel completes and
stays ENABLED, its BUSY flag down; clearing EN pauses it with BUSY still
up, and the abort register ends it. Transfers are 8, 16 or 32 bits wide,
the same on both sides, at one read and one write per clock; one side may
wrap at a power-of-two boundary (RING_SIZE, RING_SEL). The transfer
request is a FIELD, TREQ_SEL: any channel takes any of the peripheral
requests of 12.6.4.1, one of four pacing timers, or none. The request is
credit-based: a pulse per unit of room or data at the peripheral, a
saturating six-bit counter per channel, and the DMA keeps that many
transfers in flight - which is why a FIFO served by a channel must not be
touched by the processor, and why two channels must not share one
request. A completion raises the channel's raw interrupt bit; CHAIN_TO
names a channel to trigger at completion; IRQ_QUIET turns the interrupt
into "on a null trigger only", the end of a control-block chain. A bus
error halts the channel, sets AHB_ERROR with READ_ERROR or WRITE_ERROR
(cleared by writing one), raises the interrupt, and the channel refuses
to restart until the flags are cleared with BUSY read down. The sniffer
watches one channel's data go by and keeps a CRC-32, a CRC-16-CCITT, a
sum or a parity, with bit-reverse, invert and byte-swap on the result.

So far that is the RP2040's block. **Six things here are not.**

**SIXTEEN CHANNELS AND FOUR INTERRUPT LINES**, against twelve and two.
CHAIN_TO is four bits wide, every channel bit map is sixteen bits, and
there are four independent enable/force/status banks - INTE0..INTE3 -
each of which any subset of channels may assert. THE CONVENTION OF THIS
STRATUM, stated because four lines are more than two cores: line 0 is
core 0's and line 1 is core 1's, as on the RP2040, and lines 2 and 3 are
spare - a second line a core may take for a channel whose completion it
wants served apart from the rest.

**THE TOP FOUR BITS OF TRANS_COUNT ARE A MODE.** A count is therefore 28
bits, about 256 million transfers, and a read of the register carries the
mode in its top nibble, which every progress reading must mask off. Mode
0 counts down and halts, which is the RP2040's only behaviour. Mode 1,
TRIGGER_SELF, re-triggers the channel the moment the count reaches zero:
the count reloads and the addresses carry on from where they stand, so a
ring read wrapped by RING_SIZE streams lap after lap with no software
between them - and the completion interrupt and the CHAIN_TO still
happen, which is what makes "tell me when the buffer is half empty".
Mode 0xf, ENDLESS, never decrements the count at all; the register
description is explicit that such a channel triggers no other channel and
raises no interrupt, so an abort is the only thing that ends it.

**AN ADDRESS HAS FOUR STEPS, NOT TWO.** Each side's INCR bit gained a REV
partner, and the four combinations are four behaviours: the same address
every time (a peripheral FIFO), forward by the transfer size, BACKWARD by
it, and forward by TWICE it - stepping over alternate cells. The driver
spells the pair as one value, `DmaStep`, because the four combinations
are four named things and none of them is "both bits happen to be set".

**THE CTRL FIELDS HAVE MOVED** to make room for those two bits: BUSY is
bit 26 here and 24 there, TREQ_SEL sits at 17 and CHAIN_TO at 13. Nothing
of this chapter is retyped from the other chip - every constant in the
driver is the device header's own name - and the DREQ table has not one
row in the RP2040's place (a third PIO and twelve PWM slices moved
everything above the first PIO's; the numbers are in
[uart.md](uart.md)'s account of them and in `rp2350/dma_engine.hpp`).

**BOTH OF THE RP2040'S DMA ERRATA ARE GONE, AND ONE IS REPLACED.**
RP2040-E12 (READ_ADDR and WRITE_ADDR reading wrong during a wrapping or
non-incrementing sequence) is fixed: the in-flight adjustment is disabled
for exactly those transfers now. RP2040-E13 (an aborted channel reporting
a completion when its in-flight transfers land) is answered by a new
guarantee: the ABORT register may be POLLED, and it reads all-zero when
the abort has flushed. In its place stands **RP2350-E5**: aborting an
active channel may fire its CHAIN_TO, and the channel may be re-triggered
on the abort's last cycle and then complete immediately - raising its
interrupt and its chain again. The datasheet's own workaround is the
sequence `abort()` performs: EN cleared AND CHAIN_TO pointed at the
channel itself on every channel of the abort, then the CHAN_ABORT write,
then the poll. **RP2350-E8** - a CHAIN_TO from a channel triggered with a
count of zero fires only by accident - cannot be reached from this
driver: a zero count is refused by every verb that programs or starts a
channel, so no sequence it can begin has length zero.

**EVERY RESOURCE CARRIES A SECURITY LEVEL** (12.6.6), and there is a
memory protection unit of eight regions in front of both masters.
Channels, the four lines, the four pacing timers and the sniffer each
have one of four levels - SP, SU, NSP, NSU, ordered - which decides what
its bus accesses carry, who may touch its registers, which requests it
may observe and which channels it may chain to. At reset every one of
them is SP and no MPU region is enabled, so a program that runs entirely
Secure and Privileged - which is what brio runs here, the bootrom having
handed over in that state - is refused by none of it. One automatic
effect is worth knowing: a successful write to a channel's control
registers LOCKS that channel's assignment, and only the block's reset
(which `Dma::init()` performs) clears the lock again.

## The engines, from this chapter

An engine is ONE CHANNEL BOUND ONCE and fed a block at a time, and every
choice in it answers an item of the controller's own offer:

- **The four live registers and their four aliases** (12.6.2, 12.6.3.1).
  A completed channel KEEPS its CTRL, EN included, and starts only on a
  trigger write; an address that is not reprogrammed is the next
  sequence's start. So `arm()` writes the peripheral side's address once
  and CTRL once - the request, the priority, CHAIN_TO at the channel
  itself, what the channel reports, EN - and a block writes the count and
  the memory address in the TUPLE the chapter names for its direction:
  (TRANS_COUNT, READ_ADDR_TRIG) to gather a run into a peripheral,
  (WRITE_ADDR, TRANS_COUNT_TRIG) to scatter one out of it, (READ_ADDR,
  WRITE_ADDR, TRANS_COUNT_TRIG) for memory to memory. The second store is
  the trigger. CTRL is written again only when the block's beat or step
  differs from the word the channel holds, which the engine keeps per
  channel (`DmaBinding`) and compares, so the question costs a load and
  not a bus read.
- **The width** (12.6.2.3): DATA_SIZE is one field, the same for the read
  and the write, so ONE channel carries 8-, 16- and 32-bit blocks, each
  at the width of the run it is handed. `Elem` is the WIDEST beat the
  binding allows, and a wider run does not compile; a half-word or word
  run not aligned to its beat is refused at run time (12.6.2.1.1: the DMA
  does not enforce alignment).
- **The count's MODE nibble** (12.6.2.2.1): a block's count is checked
  against the 28 bits the nibble leaves - one compare - because a larger
  one written whole would select a mode, TRIGGER_SELF among them.
- **The flags.** A completion is acted on through the line's INTS, which
  clears the raw status with it, and a bus error where it is seen - in
  `service()`, `abandon()` and `stop()`. No block start clears anything:
  a channel still holding a bus error ignores its trigger (12.6.7.1), so a
  block over one never begins rather than misbehaving.
- **What a channel reports** (12.6.5, 12.6.7.1): IRQ_QUIET takes a
  block's completion off the line and leaves a bus error on it - "bus
  errors always cause the channel's interrupt request to be asserted".
  `DmaReport::errors` is that binding, and it is how a pair whose second
  completion proves the first's costs ONE interrupt: the SPI host's
  transmit engine is armed with it ([spi.md](spi.md)).
- **The gate** is the block's reset line, released ONCE by `Dma::init()`;
  there is no per-channel clock to open.
- **The credit counter** (12.6.4.2, 12.6.8.4): a receive engine clears
  its channel's count before every run, because a peripheral that kept
  requesting while the channel sat completed has banked credits the next
  run would spend reading nothing (measured, the transport's account). A
  transmit engine leaves it: a peripheral's transmit request counts room.
- **Memory to memory** is the controller's native case (TREQ_SEL
  permanent, one read and one write a clock): `DmaCopyEngine`, any
  channel, `copy` and `fill` at the beat of their element type.
- **The pacing timers** (12.6.8.1): any engine bound to a timer's request
  is the PACED shape - a table into one cell at a known rate, the
  "memory to a pad register" a timer drives.
- **The abort** (12.6.8.3, RP2350-E5) belongs to `arm()`, `abandon()` and
  `stop()`, never to a block start: a receive run is restarted only when
  it has ended, and a start over a channel still BUSY - a caller's
  misuse, not a transport's path - is answered by binding it again.
- **Declined this round, each for its reason.** The count modes and
  RING_SIZE (a self-re-arming circular receive): the transports' receive
  is still a run re-armed by its owner, and the circular shape over the
  ring's own storage is the next round's, with its ring
  (`docs/design/ring.md`). CHAIN_TO and MULTI_CHAN_TRIGGER: one start of
  an SPI pair in one store is the launch order an open question about
  the engined SPI is being bisected over ([spi.md](spi.md)), so the
  launch keeps its order until that settles.
- **The level** is one bit, CTRL.HIGH_PRIORITY (12.6.10): "in each
  scheduling round, all high priority channels are considered first,
  and then only a single low priority channel" - and 12.6.1 removes the
  idle cycle the RP2040 inserted after a round of high channels. The
  engines take docs/design/dma.md's rule in that bit: a receive engine
  arms HIGH by default, a transmit engine and the copy engine normal, a
  transport naming its engines' levels at the call. HERE THE BIT DECIDES
  CORRECTNESS: two HIGH_PRIORITY channels always ready starve a normal
  one (letter p, below), so the fixed HIGH of the transports' receive
  engines is what keeps their rings, and an engine armed by hand beside
  two HIGH channels that must not lose takes HIGH too.

THE CLAIM. A transmit engine's owner may start blocks from two contexts
- a transport's thread and its completion on the line - and "is the
channel mine" is the one decision they race for: `claim()` is its
test-and-set, taken under the owner's mask; `launch()` programs a
claimed channel with the mask down, because a channel that has not
started cannot complete under its loader; `unclaim()` gives a claim back
with nothing sent; `start()` is `claim()` then `launch()` for an owner
with one context. Before the trigger store a compiler fence
(`dma_handoff_fence`) keeps the block's bookkeeping and its data above
it; both cores issue their accesses in program order, so no barrier
instruction is owed.

WHAT A BLOCK START COSTS, counted in the release listings (the PL011
transport's `pump_tx`, the console of `test_rp2350_dma`): on the
Cortex-M33 7 instructions under the mask - the claim - and about 30
after it with two stores to the controller; on Hazard3 7 under the mask
and about 25 after it. The PL022 host's launch of a data phase is a
receive start and a transmit start: about 70 instructions and six
accesses to the controller, one of them a read (the receive channel's
BUSY).

## Types and verbs

- `DmaSize`, `dma_size_of<Elem>()`, `dma_size_bytes`; `DmaStep` (fixed,
  forward, backward, forward_by_two) with `dma_step_increments` and
  `dma_step_reversed`; `DmaCountMode` (normal, trigger_self, endless)
  with `dma_count_mode_valid` and `dma_count_max`; `Dreq` (12.6.4.1 by
  name, the four timers, `permanent`).
- `DmaChannelConfig` (size, the two steps, ring bits and side, the
  request, `chain_to`, high priority, byte swap, sniff, IRQ_QUIET),
  `DmaTransfer` (both ends, the count in items, the count MODE, the
  config) with `dma_transfer_valid` (both ends present and aligned to the
  size, a count that is neither zero nor past the 28 bits, a legal mode
  and a legal config: a ring under 16 bits, a chain to another existing
  channel), `dma_ctrl_word`, `dma_count_word`, `dma_aligned`, `DmaError`,
  `DmaProgress`, `DmaSecurity`, `DmaFifoLevels`.
- `Dma`: `init()` (the block out of the reset controller, once),
  `channels()` (N_CHANNELS), `raw()` / `clear_raw()`, `trigger(mask)`,
  `abort_raw(mask)` and `abort_pending()` (the register alone),
  `abort(mask)` (THE WHOLE CHAIN AT ONCE, with RP2350-E5's preparation
  and 12.6.8.3's poll), `fifo_levels()`, the security read-back
  `channel_security` / `channel_security_locked` / `line_security` /
  `sniffer_security` / `timer_security`, and the by-index register
  accessors `channel_ctrl` / `enables` / `force_bits` / `status` a verb
  over a mask needs.
- `DmaChannel<ch>` (0..15): `prepare(t)` (programmed, not started),
  `load(t)` (programmed and triggered), `trigger()`, `null_trigger()`,
  `configure`, `set_read` / `set_write` / `set_count(count, mode)`,
  `enable(on)` (pause), `enabled`, `busy`, `count()` live and with the
  mode masked off, `mode()`, `progress(programmed)`, `errors()` /
  `clear_errors()`, `raised()` / `clear_raised()`, `route(line, on)` /
  `routed` / `pending(line)` / `clear_pending`, `abort()` (the routes
  lifted from all four lines, the erratum's sequence, the routes back),
  `stop()`, the debug trio `debug_credits()` / `clear_credits()` /
  `debug_reload()`, and `security()` / `security_locked()`. Every
  programming verb refuses a BUSY channel.
- `DmaLine<n>` (0..3): `irq()`, `routed()`, `pending()`, `clear(mask)`,
  `force(mask, on)`, `enable()` / `disable()` in the calling core's
  interrupt controller.
- `DmaTimer<n>` (0..3): `set(x, y)` (a request every y/x clk_sys cycles),
  `numerator()` / `denominator()`, `stop()`, `dreq()`, `security()`.
- `DmaSniffer`: `start(channel, DmaSniffConfig, seed)`, `result()`,
  `stop()`, `security()`; `DmaSniffCalc`.
- `DmaMpu` (read-only): `regions`, `global_level()`,
  `hides_addresses()`, `region(n)` returning a `DmaMpuRegion`.
- `DmaReport` (`blocks`, `errors`), `dma_binding_word`,
  `dma_beat_bits<T>()`, `dma_beat_aligned<T>`, `dma_run_valid<T>`,
  `dma_handoff_fence()`, and `DmaBinding<ch>` - the CTRL word a channel
  holds as its engine last wrote it, `steer` / `bind` / `release` and the
  ISR body every engine folds in.
- `DmaTxEngine<ch, Elem, line>` / `DmaRxEngine<ch, Elem, line>`: the
  other families' engine surface - `arm(data, dreq, high_priority,
  report)` (`high_priority` true by default on a receive engine, false on
  a transmit engine), `start` over a pointer and a count or a span of `uint8_t`,
  `uint16_t` or `uint32_t` no wider than `Elem`, `start_fixed` /
  `start_discard` (one cell, its own beat), and on the transmit engine
  the claim: `claim()`, `unclaim()`, `launch` / `launch_fixed`;
  `service()` (this channel's status on its line, cleared and handed back
  as `flag_complete` / `flag_error`; no half event on this controller),
  `complete()`, `busy()` / `idle()`, `take()` (beats, from TRANS_COUNT,
  with the mode masked off), `full`, `capacity`, `abandon()`,
  `faults()`, `stop()`, and `Report` - `DmaReport` by a name a template
  reaches through the engine type. A receive engine clears the channel's
  request credits before every run.
- `DmaCopyEngine<ch, line>`: `arm(report, high_priority)` (normal by
  default; TREQ_SEL
  permanent; with `DmaReport::blocks` the completion on `line`, with
  `DmaReport::errors` routed nowhere and polled), `copy(dst, src, n)` and
  `fill(dst, cell, n)` - `n` ELEMENTS of `uint8_t`, `uint16_t` or
  `uint32_t`, the element the beat, the fill's cell in memory and read in
  place - `busy()`, `errors()` / `clear_errors()`, `remaining()`,
  `abandon()` (the abort polled to its end, the channel bound again),
  `service()`, `stop()`.
- In `uart.hpp`'s transport: the two slots, `dma_isr()` (the line's ISR
  body), `harvest()` (the receive verb), `dma_faults()`.

## How to use it

A copy, and a paced one:

```cpp
brio::Dma::init();                                        // once
using C = brio::DmaChannel<4>;
C::load({.read = src, .write = dst, .count = 1024, .config = {.size = brio::DmaSize::word}});
while (C::busy()) {}

brio::DmaTimer<0>::set(1, 150);                           // one request a microsecond at 150 MHz
C::load({.read = table, .write = &pwm_level, .count = 1000,
         .config = {.write_step = brio::DmaStep::fixed, .treq = brio::DmaTimer<0>::dreq()}});
```

A ring that re-arms itself, which is this chip's own: the count reloads
at every zero, the read address wraps inside its window, and the
interrupt marks each lap.

```cpp
C::route(1, true);
brio::DmaLine<1>::enable();
C::load({.read = buffer, .write = &fifo, .count = 64,
         .mode = brio::DmaCountMode::trigger_self,
         .config = {.ring_bits = 6, .write_step = brio::DmaStep::fixed,
                    .treq = brio::Dreq::uart1_tx}});
// ... and it is stopped by clearing EN in the handler, or by C::abort().
```

A completion on a line, and a checksum:

```cpp
extern "C" void isr_dma_0() {
    if (C::pending(0)) { C::clear_pending(0); /* ... */ }
}
C::route(0, true);
brio::DmaLine<0>::enable();

brio::DmaSniffer::start(4, {.calc = brio::DmaSniffCalc::crc16}, 0xFFFF);
C::load({.read = buf, .write = sink, .count = n,
         .config = {.write_step = brio::DmaStep::fixed, .sniff = true}});
```

A transport with an engine in each slot:

```cpp
using Link = brio::Uart<1, pins, 1024, 1024, brio::DmaTxEngine<2>, brio::DmaRxEngine<3>>;
extern "C" void isr_dma_0() { (void)Link::dma_isr(); }
...
Link::init(clock, 3'000'000);
(void)Link::write_bulk(block);        // one DMA block per contiguous run of the ring
(void)Link::harvest();                // what arrived, published; a run re-armed
```

A copy and a fill, polled, and a table poured into one cell at a timer's
pace with its completion on line 0:

```cpp
using Copy = brio::DmaCopyEngine<8>;
Copy::arm(brio::DmaReport::errors);                   // polled: on no line
(void)Copy::copy(dst_words, src_words, 1024);         // 1024 words, one a clock
while (Copy::busy()) {}
static const uint16_t pixel = 0xF800;
(void)Copy::fill(framebuffer, &pixel, 320u * 480u);   // half-words from one cell

using Paced = brio::DmaTxEngine<9, uint32_t>;
brio::DmaTimer<0>::set(1, 150);                       // a request a microsecond at 150 MHz
Paced::arm(&cell, brio::DmaTimer<0>::dreq());
(void)Paced::start(table, 256);
extern "C" void isr_dma_0() {
    if ((Paced::service() & Paced::flag_complete) != 0u) { (void)Paced::complete(); }
}
```

Ending a CHAIN needs the whole chain named at once, which is 12.6.8.3's
closing advice and RP2350-E5's other half:

```cpp
brio::Dma::abort(brio::DmaChannel<4>::bit | brio::DmaChannel<5>::bit);
```

## Bench findings

All of them on an RP2350 in the QFN-80 package, **stepping A2**, clk_sys
at 150 MHz, and all of them on BOTH architectures: `test_rp2350_dma`
reports **49 pass, 0 fail** on the Cortex-M33 pair and on the Hazard3
pair, from one source - and every verdict of it was printed THROUGH a
DMA engine, the console's transport naming one in its transmit slot.

- **Sixteen channels and four lines, as the silicon says.** N_CHANNELS
  reads 16, a channel out of reset reads its reset word, and a
  completion routed to each of the four lines in turn is counted in that
  line's handler and in no other.
- **THE ABORT ANSWERS ERRATUM RP2350-E5.** A channel stalled on a pacing
  timer that never requests stands BUSY with its count untouched and
  refuses every configuring verb; the abort's sequence takes it down with
  **the ABORT register polling to zero in 12 to 13 us**, BUSY down,
  nothing raised, no line left routed, and the channel usable again.
- **IRQ_QUIET, AND THE NULL TRIGGER UNDER IT.** A completion under
  IRQ_QUIET raises **no** interrupt, and a null trigger written after it
  raises **exactly one** - so the null trigger STILL REPORTS through
  CTRL_TRIG on this silicon, which is what makes IRQ_QUIET usable as
  "tell me at the end of the list and not at the end of each block".
- **THE COUNT MODES, which the RP2040 had not.** In TRIGGER_SELF a
  channel re-arms itself at every zero: four laps of 64 bytes paced at
  10 kHz ran in 25 588 us with the processor only counting them, the
  count reloading to 0x10000040 - the mode in the top nibble beside the
  64 - and the destination holding the source four times over. In
  ENDLESS the count never moves at all: after 5 ms of one transfer a
  microsecond it still reads 8, BUSY stands, **zero** interrupts have
  been raised, and an abort is the only thing that ends it.
- **The four address steps.** A sixteen-byte ring read into sixty-four
  bytes lays the pattern down four times; read backward from the last
  byte the copy is the source reversed; read by twos it is every other
  byte.
- **Memory to memory.** Byte, half-word and word runs all copy exactly
  and leave TRANS_COUNT at zero with the raw status raised and cleared by
  writing one. **4096 bytes as 1024 words take 11 us on the Arm half and
  13 us on the RISC-V one** against the 6.8 us one transfer a cycle would
  be - the read and the write masters share the SRAM with a core that is
  polling BUSY over the same bus.
- **A pacing timer** at one request a microsecond delivers a thousand
  bytes in **1004 us**, its X/Y reading back as written.
- **The sniffer** agrees with `util/crc.hpp` on all four functions it was
  asked for: the sum, CRC-16, CRC-32 and the reversed-and-inverted
  CRC-32.
- **A bus error is a bus error.** A word block reading the single-cycle
  IO window stops with ERRORS reading **0xC0000000** - AHB_ERROR and
  READ_ERROR both, WRITE_ERROR clear - its line's handler entered once
  and nothing written to the destination.
- **THE ENGINES ON A TRANSPORT.** 4096 bytes through UART1's own
  loop-back at 3 Mbaud, one engine in each slot, arrive **exact in 13 697
  to 13 702 us** on both halves - the wire's own time for 4096 bytes of
  10 bits at 3 Mbaud is 13.65 ms - with **18 to 22** line-0 interrupts
  for the whole of it, the console's own blocks among them; the same run
  carried on the transport's own interrupt takes 385 to 402
  (`test_rp2350_serial`'s letter e, [uart.md](uart.md)). No framing or
  overrun error, no ring overflow.
- **Byte-exact on every one of sixteen flash-and-run cycles** (ten on
  the Hazard3 half, six on the Cortex-M33 one), the first run after a
  flash included - which is the run that matters: code running cold out
  of the flash widens every window between two register reads. That is
  how this letter found the one rule a receive engine's user must keep,
  and it is the transport's (`brio/pl011/uart.hpp`, harvest()): WHETHER
  A RUN HAS ENDED IS ASKED BEFORE ITS COUNT IS READ. Asked after, a run
  whose last byte lands between the two reads is seen ended with that
  byte uncounted and is re-armed over it - one byte of 4096 gone at the
  end of a short run by the ring's wrap, with no fault, no overrun and no
  ring overflow to show for it (measured, about one cold run in five
  before the rule). The letter reports itself: on a mismatch it names the
  byte it wanted, the byte it got and how far ahead of the stream that
  byte is; on a time-out how many are short and what the engine holds.
- **The console's engine under a burst**: 2064 bytes of it went out in 46
  blocks, no fault, the engine idle after the drain.
- **ONE CHANNEL, THREE BEATS.** A byte, a half-word and a word run poured
  in turn into one cell of RAM by one engine leave it reading 0x00000044,
  0x00007788 and 0xDDEEFF01 - each beat wrote its own lanes - and a
  half-word run one byte off its beat, an empty run and a count past the
  28 bits are refused with the claim given back.
- **ERRORS ALONE.** Under `DmaReport::errors` a clean block raises
  nothing - not one entry of its line's handler - and a block reading
  the SIO window raises the channel's interrupt all the same, served as
  an error; `abandon()` hands the channel back and it runs again.
- **THE COPY ENGINE.** Copy at three beats (1024 words, 100 half-words,
  333 bytes at an odd address) and fill from one cell at three beats are
  exact; a misaligned word run and an empty one are refused; a 1024-word
  copy abandoned in flight stops short and the engine copies again exact.
- **WHAT A BLOCK COSTS, `bench_rp2350` letter d**, Cortex-M33 / Hazard3,
  the ruler TIMER1 on clk_sys. A COPY of 16 bytes as words takes 52 / 51
  cycles start to BUSY down - 48 / 47 over one cycle a word, which is the
  controller's own rate (one read and one write a clock) - and 4096 bytes
  1072 / 1071, x 1.04 against 600 MB/s; the channel verbs the suite's
  letter b uses take 167 for the same 16 bytes. A FILL from one cell
  costs the same at 16 bytes and 1.25 cycles a word at 4096 (1333 /
  1326): the cell sits in one SRAM bank and the destination walks all
  four - 2.2.3 stripes them on address bits 3:2 - so every fourth write
  waits behind a read. A PACED block - 256 words into one cell at one
  request in 150 cycles from `DmaTimer<0>` - runs at the pace, x 1.00,
  ONE interrupt, 224 / 327 busy cycles for its 256 microseconds; stamped
  against the ruler by a polling core, its steps are 150 cycles apart,
  250 of 255 within the poll's own 23-cycle turn.
- **THE LINE AT THE RATE ASKED**, which a loop alone cannot tell (both
  ends share one divisor): 1024 frames through the transmit engine leave
  in 1003 to 1007 thousandths of their wire time at 3 Mbaud (letter j),
  1000 to 1003 at 115200 (letter v), and eight start bits at 9600 lie
  seven frames apart on the pad within two per mille (letter t), on both
  halves.
- **THE LEVEL DECIDES CORRECTNESS HERE**, letter p (`brio stress` the
  sender, the console's ring on a receive engine over 8 KB, byte copies
  of 16 KB back to back beside it, 1.5 s a leg at 115200, 1 Mbaud and
  3 Mbaud). The ring at HIGH loses nothing beside two normal copies or
  two HIGH_PRIORITY ones, and a ring at normal loses nothing beside ONE
  HIGH_PRIORITY copy or two normal ones - but a ring at normal beside
  TWO HIGH_PRIORITY copies is starved: on the Cortex-M33 7717 bytes of
  94 500 lost at 1 Mbaud and 96 068 of 203 992 at 3 Mbaud, on Hazard3
  6956 of 94 500 and 123 414 of 199 490 - run to run 7 to 17 per cent at
  1 Mbaud and two fifths to two thirds at 3 Mbaud - every loss counted
  as an overrun and skipped by the ring. The RP2040 runs the same load with
  nothing lost ([../rp2040/dma.md](../rp2040/dma.md)): the round
  12.6.10 describes is not the RP2040's round once 12.6.1's idle cycle
  is gone, measured and not explained further.
- **A RELEASED REQUESTER**, letter r (no wire), the RP2040's answer on
  both halves: nothing reaches the next owner after the drivers'
  `release()` and its own reset, nor from the PL011 or the PL022 with the
  DMA enable cleared or the block disabled (12.1.5: "all request signals
  are de-asserted"), nor from a stopped PWM slice; ONE stale sample from
  the ADC disabled with DREQ_EN left set. The PWM's pulses bank credits
  on a bound, untriggered channel, `DmaChannel::stop()` and a rebind
  leave them (9 to 12 read on the next owner's channel), and the
  TRIGGER drops them: the next owner, armed on a request that never
  comes, moved nothing.

## Not covered yet

Driver gaps, each with its reason:

- The WRITE side of the security assignment and of the memory protection
  unit (12.6.6): declined while brio runs everything Secure and
  Privileged on this chip. The reset state refuses such a program
  nothing, and a level written low is only undone by the block's reset -
  so the verbs are born with the first program that hands a channel to
  another security domain. The read side is here, so a program can always
  see what it has.
- Control-block lists (a channel programming another through the
  aliases, 12.6.9.2): born with a first stream that wants the DMA to
  reconfigure itself; `prepare()`, the aliases, `chain_to` and the null
  trigger are its building blocks. The compact formats this chip adds -
  completion actions strictly ordered against the last write, so a
  control block need not contain a trigger alias - want that user too.
- The loop and ping-pong engines the other strata keep for a block
  stream (util/block_stream.hpp): born with the ADC chapter, their first
  user here. TRIGGER_SELF is what a loop engine would be built on.
- A circular receive engine - TRIGGER_SELF over a ring the hardware
  writes, its producer index the channel's own count: born with the
  ring's view of a hardware producer (`docs/design/ring.md`) and the
  transport round that adopts it.

Implemented but not bench-verified, each with what would measure it:

- WHY two HIGH_PRIORITY channels always ready starve a normal one here and
  not on the RP2040 (letter p): the measurement stands, the mechanism is
  12.6.1's removed idle cycle by inference only. A letter counting the
  normal channel's transfers against one, two and three HIGH channels of
  known demand, read off its TRANS_COUNT over a fixed window, would place
  the round.
- A PACED block started right after its `arm()`: its first transfer
  comes 700 to 1200 cycles after the trigger at one request in 150,
  where a block started 100 cycles or more after the binding - or after
  the channel's credits are cleared, or the timer written again - begins
  within one pace period; the pace itself is exact either way, and the
  credits a free-running timer banks (the counter saturates at 63) start
  no burst. Measured, not explained; a letter that sweeps the gap
  between the abort `arm()` performs and the trigger, with the
  controller's debug registers read across it, would.
- `DmaCopyEngine` completed on a line (`DmaReport::blocks`): letter l
  polls it; a letter that waits on the line's handler.
- The DW_apb_i2c host's read phase on these engines - the transmit engine
  pouring a half-word command cell, the receive engine collecting the
  bytes: compiled for both halves, and run by the I2C suite's letter on
  the I2C0 x I2C1 self-link with its two pull-ups, which is what would
  measure it ([i2c.md](i2c.md)).

- A WRITE bus error (WRITE_ERROR): letter `h` provokes a read error; a
  block writing into an address the fabric cannot decode would be the
  other half.
- `DmaSniffCalc::crc16_reversed` and `even_parity`, and the sniffer's
  BSWAP option: written from 12.6.8.2 and not measured; a letter with
  their software twins. The sum, the two CRCs and the reversed-and-
  inverted CRC-32 are measured above.
- An engine on line 1 served by core 1, and the two spare lines used as
  the convention above allows: the multicore chapter, with a transport
  on the second core - the routing is the same verb.
- `Dma::abort(mask)` OVER A CHAIN, which is the half of RP2350-E5 a
  single channel cannot answer: the suite aborts one channel at a time
  (letters a and i), so the multi-channel verb and the by-index register
  accessors it walks are exercised only up to channel 5. A letter that
  chains two channels into a loop and then ends both with one mask would
  close it.
