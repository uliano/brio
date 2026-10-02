# DMA (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 2.5 (the
controller: 2.5.1 the four live registers, 2.5.2 the trigger aliases,
chaining and null triggers, 2.5.3 the credit-based data request and
table 119, 2.5.4 the two lines, 2.5.5 the pacing timers, the sniffer
and the abort, 2.5.7 the registers), 4.2.5 (the UART's DMA interface);
Appendix B, RP2040-E12 and E13. The driver: `brio/rp2040/dma.hpp`
(the block, a channel, a line, a timer, the sniffer, the transfer
engines and the copy engine) with the request numbers in
`dma_engine.hpp` beside the empty slot's tag; the transfer engines'
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

THE OFFER, and what the engines take of it. A channel's registers are
LIVE and keep what was last written: an address that does not
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
  address is not aligned to its beat refused at run time (the caller's
  pump takes it). `arm(data, dreq, high_priority, report)` binds once;
  `Report` names `DmaReport` through the engine. Transmit: `claim()`
  (the test-and-set a transport takes under its mask), `unclaim()`,
  `launch(run)` / `launch_fixed(cell, n)` on a claimed channel,
  `start(run)` / `start_fixed(cell, n)` = claim and launch, a run being
  a span or a pointer and a count; `complete()` (the beats the block
  carried), `busy()` (claimed or running), `in_flight()`,
  `progress()`. Receive: `start(run)`, `start_discard(cell, n)`,
  `idle()`, `take()` (beats arrived since the last take, from
  TRANS_COUNT), `full`, `capacity`, `taken`. Both: `service()` (this
  channel's status on its line, cleared and handed back as
  `flag_complete` / `flag_error`; no half event on this controller),
  `abandon()`, `faults()`, `stop()`. A receive engine clears the
  channel's request credits before every run.
- `DmaCopyEngine<ch, line>`: `arm(report, high_priority)` (`blocks`:
  the completion on the line, `service()` its body; `errors`: polled,
  the channel on no line), `copy(dst, src, n)` and `fill(dst, cell, n)`
  in elements of the type the pointers carry, which is the beat;
  `busy()` (CTRL.BUSY), `errors()` / `clear_errors()`, `remaining()`,
  `stop()`. A block over a running one, an empty one and a misaligned
  one are refused.
- In the transports (`uart.hpp`, `spi.hpp`, `i2c.hpp` over the IP
  strata): the slots, `dma_isr()` (the line's ISR body), the UART's
  `harvest()` and `dma_faults()`.

What a block costs, counted in the release listing (Cortex-M0+
timings, one path, the CTRL word unchanged): a transmit block start
from a transport - the claim, the ring's run, the block - 51
instructions and about 100 cycles with TWO stores to the DMA block,
the mask held over the claim alone (six instructions, about 10
cycles); a receive run 27 instructions, about 60 cycles, one read
(BUSY) and three stores (the credit clear, the address, the count);
an SPI data phase on two engines 89 instructions, about 180 cycles,
and ONE DMA interrupt for the transaction; a copy 25 instructions,
about 40 cycles, one read and three stores.

## How to use it

A copy, and a paced one:

```cpp
brio::Dma::init();                                        // once
using C = brio::DmaChannel<4>;
C::load({.read = src, .write = dst, .count = 1024, .config = {.size = brio::DmaSize::word}});
while (C::busy()) {}

brio::DmaTimer<0>::set(1, 125);                           // one request a microsecond at 125 MHz
C::load({.read = table, .write = &pwm_level, .count = 1000,
         .config = {.incr_write = false, .treq = brio::DmaTimer<0>::dreq()}});
```

A completion on a line, and a checksum:

```cpp
extern "C" void isr_dma_0() {
    if (C::pending(0)) { C::clear_pending(0); /* ... */ }
}
C::route(0, true);
brio::DmaLine<0>::enable();

brio::DmaSniffer::start(4, {.calc = brio::DmaSniffCalc::crc16}, 0xFFFF);
C::load({.read = buf, .write = sink, .count = n, .config = {.incr_write = false, .sniff = true}});
```

Copy and fill on one channel, the beat the element type:

```cpp
using Copier = brio::DmaCopyEngine<8>;
Copier::arm();                                            // the completion on line 0
extern "C" void isr_dma_0() {
    if ((Copier::service() & Copier::flag_complete) != 0u) { /* done */ }
}
(void)Copier::copy(framebuffer_words, tile_words, 1024u);         // 4 KB as words
(void)Copier::fill(framebuffer_halves, &background, 320u * 480u); // one 16-bit cell
```

A table into one register at a timer's pace - a transmit engine whose
request is the timer's:

```cpp
using Wave = brio::DmaTxEngine<9, uint32_t>;
Wave::arm(&pwm_level, brio::DmaTimer<0>::dreq());
brio::DmaTimer<0>::set(1, 400);                           // a word every 400 clk_sys cycles
(void)Wave::start(std::span<const uint32_t>(table));
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

## Bench findings

All from `test_rp2040_dma`, green on the Pico and the WeAct board,
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
- THE ENGINES: 4096 bytes through UART1's loop-back at 3 Mbaud with an
  engine in each slot, harvested by the loop, byte-exact in 13.8 ms
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
- The HIGH_PRIORITY arbitration between channels: the suite runs one
  channel at a time; two channels started together, their order read
  off the counts.

Implemented but not bench-verified, each with what would measure it
(the board is not on the desk):

- The engines' binding and their two-store block, the beat per block,
  the claim, the copy engine: `test_rp2040_dma` whole - letter m judges
  copy and fill at three widths and their refusals, and one engine
  writing a byte, a half-word and a word run into one cell with
  DATA_SIZE read back; letters i and j run the transports' block
  start - and the transports' suites (`test_rp2040_serial`,
  `test_rp2040_spi`, `test_rp2040_i2c`, the ADC, PIO and PWM suites).
- A bus error raising the line on a channel bound IRQ_QUIET (the
  SPI host's transmit engine): a block of that engine reading the
  suite's hole in the map.
- The costs above as time: `bench_rp2040` letter d - copy and fill at
  16, 256 and 4096 bytes (the fixed cost is the wall less one beat a
  cycle), a paced table and its jitter against the timer's 400 cycles,
  and an engined SPI transaction of 16 and 256 frames at two rates.
- A write bus error (WRITE_ERROR): the suite provokes a read error;
  a block writing into the hole.
- An engine on line 1 served by core 1: the multicore suite with a
  transport on core 1 - the routing is the same verb.
