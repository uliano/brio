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
channel, a line, a timer, the sniffer, the read-only MPU view, the two
engines) with the request numbers in `dma_engine.hpp` beside the empty
slot's tag; the engines' one user today is `uart.hpp`'s transport. The
reference suite: `test_rp2350_dma`, whose console prints through a
transmit engine, and which runs on both of this chip's architectures from
one source.

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
- `DmaTxEngine<ch, Elem, line>` / `DmaRxEngine<ch, Elem, line>`: the
  other families' engine surface - `arm(data, dreq)`, `start`,
  `start_fixed` / `start_discard`, `service()` (this channel's status on
  its line, cleared and handed back as `flag_complete` / `flag_error`; no
  half event on this controller), `complete()`, `busy()` / `idle()`,
  `take()` (from TRANS_COUNT, with the mode masked off), `full`,
  `capacity`, `abandon()`, `faults()`, `stop()`. A receive engine clears
  the channel's request credits before every run.
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

Ending a CHAIN needs the whole chain named at once, which is 12.6.8.3's
closing advice and RP2350-E5's other half:

```cpp
brio::Dma::abort(brio::DmaChannel<4>::bit | brio::DmaChannel<5>::bit);
```

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
- The engines on the SPI, the I2C, the PWM and the PIO: with their
  chapters; the requests are named already in `dma_engine.hpp`.
- The HIGH_PRIORITY arbitration between channels: the suite runs one
  channel at a time, so the field is written and never contended.

Implemented but not bench-verified, each with the letter that will
measure it:

- The whole chapter: `test_rp2350_dma` is written and builds for both
  architectures, and no letter of it has run on silicon. Letter by
  letter - `a` the channel count, the refusals, the security read-back
  and an abort of a stalled channel (RP2350-E5's sequence and the
  ABORT register's poll); `b` memory to memory at three widths and its
  throughput; `c` all four interrupt lines, IRQ_QUIET and the null
  trigger; `d` chaining; `e` the ring and the backward and by-two
  address steps; `f` a pacing timer at one request a microsecond; `g`
  the sniffer's sum, CRC-16 and CRC-32 against util/crc.hpp; `h` a bus
  error from an address the DMA's ports cannot decode; `i` TRIGGER_SELF
  and ENDLESS; `j` the two engines on the UART's loop-back at 3 Mbaud;
  `k` the console's own transmit engine under a burst.
- A WRITE bus error (WRITE_ERROR): letter `h` provokes a read error; a
  block writing into an address the fabric cannot decode would be the
  other half.
- `DmaSniffCalc::crc32_reversed`, `crc16_reversed` and `even_parity`,
  and the sniffer's BSWAP option: written from 12.6.8.2 and not
  measured; a letter with their software twins.
- An engine on line 1 served by core 1, and the two spare lines used as
  the convention above allows: the multicore chapter, with a transport
  on the second core - the routing is the same verb.
- `Dma::abort(mask)` OVER A CHAIN, which is the half of RP2350-E5 a
  single channel cannot answer: the suite aborts one channel at a time
  (letters a and i), so the multi-channel verb and the by-index register
  accessors it walks are exercised only up to channel 5. A letter that
  chains two channels into a loop and then ends both with one mask would
  close it.
