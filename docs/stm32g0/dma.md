# DMA + DMAMUX (STM32G0)

This document is for a programmer who wants bytes moved with no CPU in
the path: a serial port or a bus whose blocks a channel carries, a
waveform played into a converter for ever, a stream captured into two
buffers in turn, a memory copy. Most programs meet the DMA only by
naming an engine in a transport's template (`Uart`, `SpiHost` and
`I2cHost` take `DmaTxEngine` and `DmaRxEngine` as optional parameters,
called engine parameters on this page); the engines and the raw channel
are here for the rest. `Dma<n>` is a
controller, `DmaChannel<n, ch>` one channel, `DmaMux` the request
multiplexer that routes a peripheral's request to a channel, and the
five engines over them are the shapes a program uses.

The smallest use, a memory-to-memory copy with no peripheral and no
request:

```cpp
using Copy = brio::DmaCopyEngine<1, 1>;
Copy::arm();                                  // the gate, request 0, the word
(void)Copy::copy(dst_words, src_words, 512);  // 512 words, word beats
extern "C" void DMA1_Channel1_IRQHandler() {
    const uint8_t f = Copy::service();
    if ((f & Copy::flag_error) != 0u) { (void)Copy::abandon(); }
    else if ((f & Copy::flag_complete) != 0u) { (void)Copy::complete(); }
}
```

What decides correctness even there. A channel belongs to one owner for
the whole image: the request it serves, or a tag where no peripheral paces
it. A second owner does not compile where both are claimed: a transport
and a copy engine claim by themselves, as the copy above does, and an
engine a program arms itself claims nothing until the program writes the
claim ("One owner a channel", below). `arm()`
runs once before the first block: it opens the clock, routes the
request and keeps the channel's control word, which the register
cannot hold between blocks. The channel's vector is the program's to
bind, and the handler calls the engine's `service()` and then
`complete()` or `abandon()`: a transfer error disables the channel in
hardware and nothing re-enables it until the error flag is cleared,
which `abandon()` does. The element type of the buffers is the beat
(the width of each access), so a span wider than the engine's element
does not compile and a misaligned run is refused at run time. And the
priority an engine arms at follows a rule ([design/dma.md](../design/dma.md)):
a receive engine runs at `very_high` because its peripheral overruns
when starved, and two memory copies alternating with each other can
starve anything below them.

## Documents of record

RM0444 Rev 6 - the DMA ch. 10, the DMAMUX ch. 11,
the RCC's AHB enable and reset bits 5.4.9 / 5.4.5, the interrupt table
12.3 (table 61); errata ES0548 Rev 3 items 2.4.1 and 2.5.1..2.5.4, read
on the bench chip's revision Z column, where all five apply. Driver:
`stm32g0/dma.hpp`; the per-part instance, channel and vector facts come
from `stm32g0/device_tables.hpp` (the reserve: the table of what each
part has, read off the device header), and the request ids from the
peripherals that publish them (`Usart<n>::dma_tx_request()`,
`Tim<n>::dma_update_request()` and their kin). Bench suite:
`test_stm32_dma`, three images on every part, because an image gives a
channel to one owner. The first holds the instrument letters `a` to `n`
and `r`; the second letter `o` and the serial streaming letters `p`, `q`,
`u` and `w`; the third letter `x`, letter `o`'s ring on an LPUART. Across
the three, 17 letters in their `z` and 85 verdicts, no wire (a letter is
one command key of the suite's console and the group of verdicts it runs,
`z` the key that runs them all; letters `p`, `q`, `u` and `w` stay outside
`z` and need `brio stress`). The engines' cost is
`bench_stm32`'s letter `d`. Family fixture
`test/family_stm32g0/dma.cpp` plus its negatives under
`brio check stm32g0` (`dma*.cpp`), three of them the ownership check's.

## What the silicon does

**Two controllers, one multiplexer, three vectors.** The G0B1/G0C1 class
has DMA1 with seven channels and DMA2 with five (the G0B0 too); the
G05x/G06x/G07x/G08x have DMA1's seven alone, the G03x/G04x DMA1's five.
One DMAMUX serves both: its channels 0..6 are DMA1's channels 1..7 and
its 7..11 are DMA2's 1..5 (11.3.2), a hardwired map. Every count above
is COUNTED off the device header's own `DMAn_ChannelK_BASE` /
`DMAMUX1_ChannelK_BASE` symbols, not tabulated. The vectors are table
61's: channel 1 has one to itself, channels 2 and 3 share one, and ONE
line carries DMA1's channels 4..7, every DMA2 channel and the DMAMUX
overrun - and the third line's NAME spells what it serves, in three
spellings, which is why the reserve derives it from the presence of
DMA2 and of DMA1's seventh channel beside `usart_irq()` and
`tim_irq()`.

**A channel is four registers and an enable discipline.** CCR carries
the direction, the circular and memory-to-memory bits, the two
increments, the two access widths, the priority and the three interrupt
enables; CNDTR the number of data items; CPAR and CMAR the two ends.
Everything in CCR but the interrupt enables is READ-ONLY while EN is set
(10.6.3), and CNDTR "must not be written when the channel is enabled"
(10.6.4) - so `configure()` and `set_count()` refuse rather than store
into a register the silicon ignores.

**Suspend and resume is not a thing.** 10.4.5 is explicit: disabling an
active channel and re-enabling it without reprogramming "is not
supported by the DMA hardware". The supported sequence is
abort-and-restart - disable, reconfigure, enable, in SEPARATE writes to
CCR - and `load()` is exactly that. What this driver never offers is the
unsupported one.

**A transfer error disables the channel in hardware** (10.4.7) and EN
cannot be set again until TEIFx is cleared. That is the one place a
`false` from this driver means the SILICON refused rather than the
driver did.

**DIR does not say "peripheral" or "memory": it says which side is the
source.** With DIR = 0 the CPAR side is read and the CMAR side written;
with DIR = 1 it is the other way round, and in memory-to-memory mode the
register names mean nothing at all. `DmaDirection`'s enumerators are
therefore spelled from the source.

**The request vocabulary is not in any header of this pack.** Table 55
numbers 77 request lines by peripheral, and the `DMAMUX_REQ_*` spellings
live in ST's HAL/LL, which this project does not vendor. So `dma.hpp`
owns the fabric and takes a plain request id, and each peripheral
publishes its own codes. The family fixture is where the two publishing
drivers are held to one table.

**A DMAMUX channel can do three more things than route.** With SE it
holds the peripheral's request back until an edge arrives on one of 24
synchronization inputs, then lets NBREQ + 1 of them through; with EGE it
emits a one-cycle event every NBREQ + 1 served requests, and table 56
offers those four events BACK as trigger inputs 16..19; and four
separate request-generator channels turn an edge on one of the same 24
trigger inputs into GNBREQ + 1 DMA requests of their own. Trigger inputs
0..15 ARE the EXTI's lines 0..15.

## Errata

All five DMA/DMAMUX items of ES0548 Rev 3 apply to revision Z.

| item | what | how it is answered here |
|---|---|---|
| 2.4.1 | a CGIFx write coinciding with a transfer error loses BOTH the TEIFx flag and the automatic channel disable | **structural**: `DmaFlag::all` does not contain the global bit and `clear()` masks it out, so no caller can write CGIFx. Measured beside it: clearing the three specific flags clears GIFx anyway (10.6.2), so the workaround costs nothing |
| 2.5.4 | a CxCR write that turns synchronization on while SPOL already holds an edge routes the PREVIOUS request id | **structural**: `request()` writes SPOL = 00 with SE = 0, and `request_synchronized()` writes SE and a non-zero SPOL in ONE word. There is no verb that can leave the illegal combination behind |
| 2.5.1 | a SOFx raised by another channel during a CFR write is lost. No workaround | stated on `DmaMux::clear_overrun()`: do not run two synchronized channels that can both overrun |
| 2.5.3 | the same on the request-generator side, during an RGCFR write. No workaround | stated on `DmaMuxGenerator::clear_overrun()` |
| 2.5.2 | with GNBREQ > 1, a trigger landing on the very last request of a batch loses OFx and makes the NEXT batch one single request | a stated obligation on `configure()`: only the application knows its trigger period. The bench's own overrun leg is staged with the channel DISABLED, so it does not sit in that window |

## Types and verbs

`Dma<n>` is the block (bus clock, reset, the two flag registers, the
channel count, `irq(ch)`). `DmaChannel<n, ch>` is one channel, numbered
as the silicon numbers it - from ONE. `DmaMux` addresses the multiplexer
by DMAMUX channel (`DmaChannel<n, ch>::mux_channel` is the map) and
`DmaMuxGenerator<x>` is one of the four request generators.

Five engines sit on top, each driving one channel and knowing nothing
about the peripheral it serves - the data address and the request id are
handed in at `arm()`:

- `DmaTxEngine<n, ch, Elem>` / `DmaRxEngine<n, ch, Elem>` - the transfer
  pair: one run of memory into a peripheral's register, one run filled
  from it. The receive engine has a SECOND SHAPE on the same binding, the
  CIRCULAR one: a ring's whole storage handed to `arm()`, written lap
  after lap with its count register as the ring's producer index (below,
  "The circular receive"). They are what the engine parameters of `Uart`,
  `SpiHost` and `I2cHost` take - the bus hosts run the
  one-shot receive, the `Uart` the ring. Their `abandon()` is cold and
  out of line, so a driver that flattens its interrupt body into its
  vector or into SRAM ([platform.md](platform.md), "Code in SRAM")
  leaves the error path in the flash.
- `DmaLoopEngine<n, ch, Elem>` - `util/block_stream.hpp`'s
  **BlockPlayer**: one caller-owned table played into a peripheral for
  ever, on a CIRCULAR channel.
- `DmaPingPongEngine<n, ch, Elem>` - `util/block_stream.hpp`'s
  **BlockSource**: fill one caller-owned buffer while the caller drains
  the other, on a NON-circular one.
- `DmaCopyEngine<n, ch>` - memory to memory: `copy(dst, src, count)` and
  `fill(dst, cell, count)`, the element type of the two pointers the beat
  (uint8_t, uint16_t or uint32_t) and the count in elements, the fill's
  cell a word of the caller's MEMORY (the channel reads an address);
  `busy()`, `complete()` from the handler or `poll()`, `abandon()`. This
  surface is every family's.

The level an engine arms at is its peripheral's ([design/dma.md](../design/dma.md)):
`DmaPriority` lives in `stm32g0/dma_engine.hpp`, so a transport names a
level without including the controller, and each engine's `arm()`
defaults to the rule's level for its direction - the receive engine and
the ping-pong source at `very_high` (their peripheral overruns: a USART's
ORE, an SPI's OVR, the ADC's OVR, a capture's CCxOF), the transmit engine
and the player at `high` (theirs waits, or holds its last value a
period), the copy at `low` (no request paces it). The drivers name
theirs at the call with the chapter's reason: the `Uart`'s ring at
`UartOptions::rx_priority` (`very_high` by default) and its transmit at
`high`, the `SpiHost`'s receive at `very_high` and transmit at `high`
(35.5.11: the RXFIFO full, OVR discards the frame and every one after
it), the `I2cHost`'s two at `high` (32.4.7: a byte not taken or not
written stretches SCL, nothing overruns). One paced output on this die is
an exception to "only slows": the DAC, whose trigger finding its request
unacknowledged converts the old data and raises DMAUDR, the stream to be
restarted (16.4.8) - the player stays at `high`, and a program that plays
one beside copies on higher channels ranks it at the call. The raw
channel's `DmaChannelConfig` keeps CCR.PL's reset value, `low`.

`Elem` is the widest beat the binding allows - the peripheral register's
width - and one `sizeof` decides PSIZE, MSIZE and the address arithmetic
together, so they cannot disagree; a type that is not 1, 2 or 4 bytes
wide is a compile error at the engine that named it. The transfer
engines' `start()` is overloaded on spans of uint8_t, uint16_t and
uint32_t and takes its beat from the span: an SPI's engines are uint16_t
and serve a byte frame with a byte beat and a 16-bit frame with a
half-word one, and a span wider than `Elem` is a compile error at the
call. A run misaligned for its beat (10.4.3's note) is refused at run
time, false. A driver that can ask first does, with `dma_engine.hpp`'s
`dma_beat_aligned()`: the SPI host answers such a request `spi_rejected`
before anything moves.

### Two moments: arm() once, five stores a block

`arm()` writes everything that is constant for a binding: the AHB gate
(RCC_AHBENR, opened there and never per block), the channel stopped, the
DMAMUX pointed at the request, CPAR - and the CCR WORD (direction,
priority, the interrupt enables, the circular bit), which the ENGINE
keeps because the register cannot: every CCR field but the enables is
read-only while EN is set (10.6.3) and a block restarts only with EN
down. A block's start is `DmaChannel::restart()`, 10.4.5's abort and
restart reduced to what changes - CCR = the word with EN clear, IFCR =
the channel's three flags, CMAR, CNDTR, CCR = the word with EN - FIVE
STORES AND NO READ; memory to memory adds CPAR, six. The enable is not
read back: the IFCR store clears TEIF before it, and TEIF is the only
thing that makes the silicon refuse EN (10.4.7). ST's own
`HAL_DMA_Start_IT()`, read as the oracle (stm32g0xx-hal-driver 1.4.7),
spends fourteen accesses on the same restart - three read-modify-writes
of CCR, the DMAMUX's flags, CPAR again - and clears the global flag this
driver never writes (ES0548 2.4.1).

The controller's offer, item by item:

| item (RM0444) | used? |
|---|---|
| the register file: CCR, CNDTR, CPAR, CMAR (10.6.3-10.6.6) | CPAR at `arm()`, CMAR and CNDTR a block, CCR's word kept in the engine |
| the width fields PSIZE and MSIZE (10.4.5) | the beat per `start()`, equal on both sides; table 51's unequal widths (packing, zero-extension) stay the channel verbs' and the bench's - an engine moves one item a beat |
| circular mode (10.4.5) | the player's (`DmaLoopEngine`), and the receive engine's second shape: a ring's storage written lap after lap, its count register the ring's producer index (`util/ring.hpp`'s `HardwareRing`), the transfer-complete flag counting laps; the half-transfer flag armed on the ring alone, the edge of a stream with no silence |
| a self-trigger or a chain between channels | none on this controller: a channel is one block; the DMAMUX's channel events (EGE) and request generators chain channels through the fabric, measured below and used by no engine |
| the gate (5.4.9) | opened by `arm()`, once |
| the flags (10.6.1, 10.6.2) | cleared by a write-one IFCR store of the three specific bits, never CGIFx (ES0548 2.4.1): one store a block start, one a completion |
| the interrupt enables (10.5) | in the kept word: `DmaIrq` (`dma_engine.hpp`) - the error always, the completion unless the driver holds another proof of it |
| memory to memory (MEM2MEM, 10.4.5) | `DmaCopyEngine` |
| the request sources: table 55 through the DMAMUX, its synchronization and request generators (11.4.4, 11.4.5) | the request is the peripheral's, handed to `arm()`; a timer's update paces `bench_stm32`'s `paced` |

**One interrupt per transaction.** A driver that holds another edge
arms an engine for its errors alone (`DmaIrq::error_only`). The SPI host
does it to its transmit engine - the receive block's completion is
strictly after the transmit block's last beat, since every frame that
came back was clocked out first - and the I2C host to both of its: the tenure's STOP after NBYTES proves the
transmit block, and the driver completes the engine on that proof
(`complete()`); of the receive block it proves only the wire, so the STOPF entry
reads the channel's count ([i2c.md](i2c.md), "The receive channel at STOPF").

**The mask covers one test-and-set.** A transport starts blocks from two
contexts, its thread and its completion handler, so `DmaTxEngine` splits
a start into three verbs: the run-time `claim()`, a test-and-set of
`busy()` that takes the engine for one block (nothing to do with the
compile-time claim of "One owner a channel"); `launch(span)`, the five
stores; and `unclaim()`. `start()` is the three together, for a driver
with one starting context; `mark_busy()` is the flag alone, for an owner
whose own state says no block of its is in flight (the I2C host, whose
tenures end their blocks). The USART's `pump_tx()` holds its guard around
`claim()` alone - seven instructions - and launches unmasked: a free
engine has no block in flight, so no completion can run under the
programming.

### Reading a channel's progress costs nothing

CNDTR is a live register the controller decrements and software may read
at any time (10.6.4), so there is no harvest ceremony on this silicon:
`DmaRxEngine::take()`
is one register read and a subtraction, and `remaining()` - the circular
shape's producer index - is the register read alone.

### The circular receive: a ring the channel writes

`DmaRxEngine`'s second `arm()` takes a ring's whole storage, an array
whose element is the beat: `arm(data, request, storage, priority)` binds
it for good and keeps a word with CIRC, MINC, the beat, the priority,
TCIE and TEIE beside the storage and its length; `start()` - no run -
is the five stores, and from then on the controller writes the storage
lap after lap and reloads CNDTR and its internal address at every wrap
(10.4.5, 10.6.4) with no CPU in the path. Nothing re-arms it, so nothing
is lost between runs: there are none. The engine is
`util/ring.hpp`'s `RingCounter` for that ring:

- `remaining()` is CNDTR, one load - decremented after each "read
  followed by write" (10.6.4), so it never counts an element whose write
  has not landed;
- `laps()` is the transfer-complete flag turned into a count by
  `complete()` in the channel's handler - so it lags the counter by a
  handler's latency and never leads it.

`HardwareRing<storage, DmaRxEngine<n, ch, Elem>>` is the consumer half
([../design/ring.md](../design/ring.md)): Ring's consumer verbs, and the
accounting of a lap the consumer did not keep up with (`overruns()`),
which is the view's because only the consumer knows where its tail is.
The half-transfer flag is not the view's: it reads the counter whenever
it looks, and the wrap is the one edge its arithmetic needs. It is the
transport's: the ring's word carries HTIE beside TCIE, and `service()`
reports the half mark while the ring runs, so the transport's handler
can tell its consumer twice a lap on a stream that never pauses for the
line's own end-of-burst flag. The one-shot verbs stay usable on a ring binding - `start(run)` runs a block
that replaces the ring until the next `start()` - and a one-shot `arm()`
leaves the ring the circular one bound where it was. The ring's length
is a compile-time fact and is refused past CNDTR's 65535 at the `arm()`,
a ring wider than the engine's element likewise.

**Stopping one has an order.** 10.4.5 stops a circular transfer by
stopping the peripheral's requests FIRST and disabling the channel
after, a channel that never ends having no idle moment of its own; so
the transport sequences it, and `Uart::release()` disables the USART before
it stops its engines.

`stm32g0/usart.hpp`'s `Uart` runs its receive engine in this shape over
its whole receive ring ([usart.md](usart.md)): a byte that lands is
readable at once through `read_span()`, `read_byte()` and
`read_bulk()`, the channel's completion handler counts a lap, and
`harvest()` - run by the transport's two vectors, the USART's on the
line's IDLE and the channel's on the half and full marks - is left with
the housekeeping (the receive errors read and cleared through ICR, a
channel a transfer error stopped started again with the view cleared)
and with the edge a ring with no publish step cannot give by itself:
true when bytes stand in a ring the consumer had looked at and found
empty since the last true. `consume()` there answers whether the run was intact, and
`rx_overruns()` counts the laps a consumer lost.

### The two verbs a full-duplex bus needs

`DmaTxEngine::start_fixed(cell, n)` sends the SAME cell `n` times and
`DmaRxEngine::start_discard(cell, n)` throws `n` elements into one -
the memory pointer simply does not increment, MINC being the one bit
the verb ORs into the kept word. The cell's type is the beat, as a
span's element is. They exist because a full-duplex bus has no
half-transaction: the transmit side of a READ must still put something
on the wire to make the clock run, and the receive side of a WRITE must
still be drained or the peripheral overruns. `stm32g0/spi.hpp`'s engined
host is their first user (a null `tx` feeds all-ones from a held
half-word, a null `rx` drains into a held sink; a byte beat reads and
writes the cell's low byte) and `test_stm32_spi` letter `i` is their
proof.

### One example per use

Each block is an excerpt: the buffers, the timers and the USART's pins are
the program's, brought up as their own pages say, and each vector name is
the program's to bind.

Memory to memory - no peripheral, no request, runs on the enable:

```cpp
using Copy = brio::DmaCopyEngine<1, 1>;
Copy::arm();                                  // the gate, request 0, the word
(void)Copy::copy(dst_words, src_words, 512);  // 512 words, word beats
extern "C" void DMA1_Channel1_IRQHandler() {
    const uint8_t f = Copy::service();
    if ((f & Copy::flag_error) != 0u) { (void)Copy::abandon(); }
    else if ((f & Copy::flag_complete) != 0u) { (void)Copy::complete(); }
}
// or, armed with brio::DmaIrq::error_only: while (!Copy::poll()) {}
```

A waveform played for ever, with the CPU out of the path - an engine a
program arms itself, so the program claims its channel for the request
it hands (below, "One owner a channel"):

```cpp
using Player = brio::DmaLoopEngine<1, 1, uint32_t>;
static_assert(brio::dma_claim<brio::DmaSlotId<1, 1>,
                              brio::DmaOwner<brio::DmaPeriph::tim, 2, brio::DmaSignal::update>>);
Player::arm(Tim2::ccr_address(0), Tim2::dma_update_request());
Tim2::interrupts(Tim2::update_dma, true);
Player::start(duty_table, 8);          // and that is all: CIRC does the rest
extern "C" void DMA1_Channel1_IRQHandler() {
    if ((Player::service() & Player::flag_complete) != 0u) { (void)Player::complete(); }
}
```

A sampled stream into a `BlockRelay`:

```cpp
using Source = brio::DmaPingPongEngine<1, 4, uint32_t>;
static_assert(brio::dma_claim<brio::DmaSlotId<1, 4>,
                              brio::DmaOwner<brio::DmaPeriph::tim, 3, brio::DmaSignal::update>>);
using Relay  = brio::BlockRelay<brio::Stm32g0Platform<>, Subs, Source>;
Source::arm(&TIM2->CNT, Tim3::dma_update_request());
Source::start(buffer_a, buffer_b, 32);
// in the channel's handler, after Source::complete():
brio::post<Relay>(brio::BlockDone{});
```

A USART that moves its bytes without the CPU - the engine parameters are
empty by default and cost an engineless image nothing; the receive side is
the ring above:

```cpp
using Serial = brio::Uart<2, pins, 64, 256,
                          brio::DmaTxEngine<1, 6>, brio::DmaRxEngine<1, 7>>;
extern "C" void DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQHandler() {
    // a block sent, or the ring's half or full mark: the edge on true
    if (Serial::dma_isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
extern "C" void USART2_LPUART2_IRQHandler() {
    // the line's IDLE, a receive error: the edge on true
    if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
```

A receive ring any peripheral fills, read in place:

```cpp
uint8_t rx_ring[256];
using Rx   = brio::DmaRxEngine<1, 5>;
static_assert(brio::dma_claim<brio::DmaSlotId<1, 5>,
                              brio::DmaOwner<brio::DmaPeriph::usart, 3, brio::DmaSignal::rx>>);
using View = brio::HardwareRing<rx_ring, Rx>;
Rx::arm(brio::Usart<3>::rx_data_address(), brio::Usart<3>::dma_rx_request(), rx_ring);
Rx::start();
// the channel's handler: if (Rx::service() & Rx::flag_complete) Rx::complete();
const auto run = View::read_span();   // the bytes where the channel wrote them
const bool intact = View::consume(static_cast<uint32_t>(run.size()));
```

## One owner a channel

The ownership rule ([design/dma.md](../design/dma.md), "The ownership
rule") on this controller. A slot is a channel, `DmaSlotId<c, ch>`: DMA1's
or, on the G0B0/G0B1/G0C1 class, DMA2's, numbered from one as the silicon
numbers it. The request id the DMAMUX routes to the channel is the
owner's, never part of the key: one request id on two channels is the
other half of the check, which RM0444 11.4.4's caution allows only for
"two connected DMA channels ... not simultaneously active" - in-turn use,
which the rule forbids anyway. Who claims, all at compile time through
`util/dma_owner.hpp`, at no byte and no cycle:

- **a transport, for the engines it is given**: `Uart` and `LpUart` claim
  their channels for the instance's transmit and receive
  (`DmaOwner<DmaPeriph::usart, n, ...>`, `lpuart` for an LPUART),
  `SpiHost` and `I2cHost` for theirs, in the class body, so a second owner
  on either channel, or the same request on another channel through a
  second transport type, does not compile. One host used at two element
  widths, or one peripheral under two transport types, is one owner and
  compiles;
- **`DmaCopyEngine<n, ch>`, for itself**: no request paces a copy, so the
  engine claims its channel for its own tag, `DmaCopyOwner<n, ch>`;
- **the program, for every engine it arms itself and every channel it
  programs through `DmaChannel`**: `DmaTxEngine`, `DmaRxEngine`,
  `DmaLoopEngine` and `DmaPingPongEngine` learn their request at `arm()`,
  at run time, so they claim nothing; a program that arms one names the
  request in a claim beside it (the examples above). One that programs a
  channel by hand does the same where a peripheral paces the channel, and
  names a tag of its own, any empty class, one tag a channel, where none
  does.

What the map leaves to choose is the count alone: the DMAMUX routes any
request to any channel (11.4.4), so no instance excludes
another and no choice is forced; the only conflict is more requests than
channels (10.3.1, table 49). Counted over the uses most programs combine -
a console UART with both engines, an SPI host's two (or its transmit
alone), one converter stream, one DAC stream, one timer-paced stream, a
second UART with both engines - the groups that cannot all be on DMA at
once:

| part | channels | what does not fit at once |
|---|---|---|
| STM32G030, G031, G041 | 5 | the console's two, the SPI host's two, the converter and a paced stream: six for five (with the host's transmit alone they take all five) |
| STM32G050, G070 | 7 | the console's two, the SPI host's two, the converter, a paced stream and a second UART's two: eight for seven |
| STM32G051, G061, G071, G081 | 7 | the console's two, the SPI host's two or its transmit alone, the converter, the DAC, a paced stream and a second UART's two |
| STM32G0B0, G0B1, G0C1 | 12 | none of these: the set takes nine |

No choice of instances changes a row, the map being free: what a program
on a five-channel part gives up is a stream, never an instance.

## The block stream

`util/block_stream.hpp` is a fixed point a controller's stream
machinery is measured against, so that friction shows up as "this
concept does not fit" instead of as silent divergence
(`docs/design/block-stream.md`). Both concepts are satisfied here as
written, and the measurements are what say why.

**`BlockPlayer` fits circular mode exactly, and gains by it.** A
controller with no circular mode re-arms a loop from a completion
interrupt: one interrupt per lap, and a window at every lap boundary in
which the peripheral is unserved. Here CCR.CIRC reloads
CNDTR and both current address registers in hardware, so the interrupt at
the wrap only COUNTS the lap. Measured: a circular channel with its
interrupt DISARMED and its NVIC line masked was still enabled 25 ms
later with CNDTR moving, and the captured block was self-checking to the
tick. The sentence in the design doc that says a player "publishes
nothing per lap and `laps()` moving is the one fact that says the stream
is alive" is exactly right here - and cheaper than on a controller that
has to re-arm.

**`BlockSource` does NOT fit circular mode, and the reason is the
doctrine rather than the API.** Circular plus the half-transfer flag
looks like a ping-pong for free: one buffer, two halves, HT and TC as the
two edges. It cannot honour the contract's central trade - "an overrun
means the source SKIPPED a block rather than tear one" - because A
CIRCULAR CHANNEL NEVER STOPS. When the caller still holds the half the
controller is about to write, the only moment software could intervene is
the interrupt AFTER the edge, and by then the controller has already
begun writing that half. The tear happens while the software is deciding
not to allow it, and it is invisible: nothing in the block says which of
its elements belong to this lap and which to the next.

Measured, with a handler whose whole body is "disable the channel and
read CNDTR" at the half-transfer edge:

| request rate | elements already written into the next half when the handler stopped the channel |
|---|---|
| 10 kHz | 0 |
| 2 MHz | 6 |

So under CIRC the guarantee would be a hope about interrupt latency, and
its failure would be silent. `DmaPingPongEngine` therefore uses a
NON-CIRCULAR channel that stops itself at the end of every block: the
swap into the other buffer happens with the channel idle, and the
guarantee is structural. The suite reads CCR back while the engine
streams and finds CIRC clear, so the claim is the silicon's and not the
comment's.

**What the non-circular choice costs, and why it is smaller here.**
Between a completion and the handler's next `start()` the peripheral is
unserved. On this controller 10.4.3's request is a LEVEL the peripheral
holds until it is acknowledged, so a request raised inside that window is
served LATE rather than lost; what can still be lost is a SECOND arrival
inside one window, and that loss is the peripheral's own overrun flag to
report - which is exactly where the contract already puts it.

**The concepts hold here word for word** (`docs/design/block-stream.md`).
What this implementation makes sharper is their own framing: they are
about BLOCKS and not about DMA, and the proof is that the same two
concepts are met by a controller whose natural streaming mode is the one
the contract cannot use.

## Bench findings

Everything below is `test_stm32_dma` on a Nucleo-G0B1RE at 64 MHz, no
wires.

**A REQUEST IS A LEVEL SERVED ON ENABLE, NOT AN EDGE LATCHED ON THE
RISE** - the reason this driver has no `kick()`, on any peripheral: `stm32g0/spi.hpp`'s engined host
kicks nothing either, and its data phase moves whole with the channels
merely enabled. Staged on USART1 brought up with its pads never claimed
(so nothing leaves the die) and TXE therefore standing: a channel armed
over that standing request moved its whole four-byte block with no
software trigger of any kind, and the control - the same channel over the
same standing TXE with CR3.DMAT clear - moved nothing at all.

**Two memory-to-memory channels ALTERNATE, and the software priority
does not enter into it.** 10.4.4 says so in a clause easy to read past:
re-arbitration between every single transfer, and "the DMA arbiter
automatically alternates and grants the other highest-priority requested
channel, WHICH MAY BE OF LOWER PRIORITY than the memory-to-memory
channel". Measured over 512-word blocks: at the first finish the loser
was within 3 of the winner in ALL THREE arrangements - very-high against
low, low against very-high, and equal. What is left is the hardware
tie-break, and it is exact: channel 1 was ahead every time, INCLUDING
when it was the low-priority channel. The lower index wins and no
register configures it.

**PL BETWEEN TWO TIMER-PACED CHANNELS DID NOT SHOW.** Two channels
over-requested at 32 million update events a second EACH - five times the
throughput measured below - still finished within one or two counts of
each other whichever way the two levels were set (letter `c`). PL's
effect is what letters `p` and `q` measure instead, below: the receive
ring's channel read back at CCR.PL 3 and 0, and the pair starving it at
the one and not at the other.

**ONE COPY STARVES NOTHING; TWO DO, AND THE LEVEL IS WHAT KEEPS A RING
FIRST** (`test_stm32_dma` letters `p` and `q`, outside `z`, with `brio
stress` as the sender). The alternation above is between a memory-to-
memory channel and ANY other requester, so a single copy of any length
never starved USART2's receive ring (16 KB copies, 127 character times at
921600, lost nothing). Two copies launched together on channels 4 and 5
alternate with EACH OTHER, and a ring on channel 7 at their level waits
for the pair: 2.6 character times at 115200 with 2 KB copies, 3.7 to 22
at 921600 with 256 B to 2 KB ones (ten runs of each letter on the
STM32G0B1RE, each leg launching 500 or 501 pairs, the verdicts' floor 416).
At the ring's default `very_high` the same pairs starved nothing - every
byte of three 2.5 s legs in sequence, no overrun, no kick of the FIFO's
cure; told `rx_priority = low`, the ring was starved for real and the
receive FIFO's kick absorbed every starvation up to nine characters with
no byte lost, and past that every gap was counted ([usart.md](usart.md),
"The receive FIFO under a starved channel", has the table). The pair ran
its blocks at about 3.5 cycles a byte each, the two together.

**A RELEASED REQUESTER HOLDS NOTHING HERE, AND THE RESET AT `init()`
TAKES THE ONE THING THAT COMES BACK** (`test_stm32_dma` letter `r`, no
wire). docs/design/dma.md's release contract: a transport's `release()`
gates its block and pulses no reset, its `init()` starts from the reset
line. The DMAMUX routes one request line to each channel (11.4.4). Under
one owner a channel, a request held across a release can reach only that
owner's channel, and only when the owner is initialized again.
Measured with USART1's transmit (TXE behind DMAT), SPI1's transmit (TXE
behind TXDMAEN) and TIM3's update (one software update behind UDE), each
brought up with its request standing and no channel listening, then
released, then DMA1's channel 5 routed to the same request and enabled
over four bytes into the requester's own register, the shape of the
owner's channel; where the clock was gated it is opened again afterwards
and the count read once more. One channel routed to three requests in
turn is this instrument's alone: a program may not do it, and the suite's
single tag on channel 5 is why the check does not see it. Items moved,
before / after the clock reopened, on the STM32G0B1RE:

| the release | USART1_TX | SPI1_TX | TIM3_UP |
|---|---|---|---|
| A: the contract - the DMA enable left set, clock gated, no reset, then `init()`'s first step, the clock opened and the reset pulsed, before the channel | 0 / 0 | 0 / 0 | 0 / 0 |
| ... and the owner brought up again, its block on the channel | 4 | 4 | 1 |
| B: DMA enable cleared, clock gated, no reset | 0 / 0 | 0 / 0 | 0 / 0 |
| C: the control - as A with the reset at init removed: the DMA enable left SET, clock gated, then opened, no reset | 0 / 0 | 0 / 4 | 0 / 1 |
| D: DMA enable cleared, clock left on, no reset | 0 / 0 | 0 / 0 | 0 / 0 |
| E: nothing released (the detector's control) | 2 / 2 | 4 / 4 | 1 / 1 |

Unlike the CH32V203's, a gated block here withdraws its request, and so
does a DMA enable cleared with the clock on: nothing reaches a channel
while the gate is closed. What does survive is a DMA ENABLE LEFT SET: the
moment anything opens the block's clock again without a reset, SPI1 asks
again for every frame and TIM3 serves its pending update (USART1 asked
for nothing, its UE being clear) - C, the control. Most transports'
`release()` leaves that enable set: the USART and LPUART tasks, the SPI
host and client, the I2C resource and the ADC, whose `release()` does not
touch CFGR1.DMAEN. The DAC's `release()` clears its channels and
the timers' clears DIER, so letter `r`'s TIM3 column, with UDE left set,
is a worst case staged by hand. Their `init()` opens the clock and pulses
the reset before it configures the block or arms a channel, which clears
every enable: A, the request gone and the owner's block exact. THE
DIRECTION MATTERS TO THE DETECTOR: USART1's standing transmit request
moved nothing into a channel reading a RAM cell into RAM and two items
into one writing TDR (the second into the shift register, then the wire's
pace), so the detector's channel writes where the owner's channel would.

**Throughput.** 512 words (2048 bytes) memory to memory in 2638 cycles =
5.15 cycles a word, about 49.7 MB/s at 64 MHz - the two AHB accesses
10.4.3 describes for every single transfer plus the re-arbitration 10.4.4
places between the transfers of a memory-to-memory block. The chapter
gives no cycle figure, so how the five divide is not claimed;
`bench_stm32`'s letter `d` measures the same 5.00 a word by the
difference of two sizes.

**Table 51's alignment rules hold as printed.** A byte source into a
halfword destination is ZERO-EXTENDED, one item per address step of two
(0xC0 reads back 0x00C0); a word source into a byte destination keeps the
LOW byte of each word (0x11110003 reads back 0x03) - a truncation, not an
error. PINC clear turns a block into a FILL.

**A transfer error is exactly what 10.4.7 promises.** A read of
0x30000000 - between the SRAM and the peripherals, belonging to nobody -
raised TEIF and GIF together and the channel came back DISABLED with
nothing in software having noticed first. EN could not be set again
until TEIF was cleared, and could immediately after.

**The three shared vectors confine themselves.** Channel 3's four flag
bits were clear throughout a block channel 2 ran on the SAME vector, so
an ISR body that reads only its own group cannot consume its
vector-mate's completion. And an UNARMED flag is set by the hardware
anyway: the ISR body reports and clears only what is armed, so a poller
still finds a completion nobody asked to be interrupted for.

**The request generator turns an edge into requests, on a trigger with no
pad.** Table 56's input 22 is TIM14_OC, and TIM14 needs no pin to produce
one: ten triggers at 1 kHz with GNBREQ + 1 = 4 moved exactly 40 words,
never a stray one. With the channel left DISABLED so no request is ever
served, the second trigger set OFx as 11.4.5 warns, and RGCFR cleared it.

**A SOFTWARE EXTI TRIGGER DID NOT REACH THE REQUEST GENERATOR - recorded,
not explained.** Table 56 makes trigger inputs 0..15 the EXTI's lines
0..15, and this family's EXTI has SWIER, so a request generator ought to
be reachable from software with no pad at all. Five SWIER pulses on line
9, with the line's rising trigger selected and BOTH its interrupt and its
event mask set, moved ZERO words on a channel that the TIM14_OC trigger
drove perfectly in the same letter. The data verdict is DECLINED with
the finding printed. What this bench has not separated: whether the
DMAMUX watches a signal SWIER does not drive, or whether a pad left in
analog mode - which blinds an EXTI line ([exti.md](exti.md)) - blinds
this path too.

**EGE makes one channel pace another through the fabric alone.** With a
multiplexer channel set to emit an event every four served requests and a
request generator triggered by `dmamux_evt0`, a paced 64-request stream
on DMA1 channel 1 moved exactly 16 words on channel 5 - which has no
peripheral of its own and never sees an interrupt.

**The console's own transmit engine saturates the wire, and at 115200 the
per-byte feed costs nothing.** A kilobyte took 88.97 ms fed byte by byte
(11510 B/s) and 88.98 ms fed in bulk (11508 B/s), against the 11520 B/s
115200 8N1 carries: the wire is five hundred times slower than the
core feeding it, the ring is always full when a block ends, and every
block the engine gets is a long one. TIMED AGAINST THE WIRE at the rate
ASKED - the kilobyte from its first store to TC against 1024 frames of
ten bits at 115200, which no reader sharing the rate could tell - both
feeds read **1000 and 1000 thousandths** of the wire's time on the
STM32G0B1RE and the STM32G071RB, judged -1 % to +3 %.

**The timer round trip.** An eight-entry duty table played into TIM2's
CCR1 on its own update request, at a 16 kHz PWM, reads 521..522 per mille
off LD4's own pad against the table's mean of 525 - with the CPU touching
not one compare value; the lap count moves at exactly the predicted
2000 laps a second, and stopping the player leaves the pad holding one of
the table's own entries. In the other direction a ping-pong engine
drained TIM16's capture of the LSI - a real ~32 kHz stream reached
through TISEL with no pad at all - four whole blocks of 32, every
consecutive pair 30 us apart, nothing torn and no overrun.

**`BlockRelay` inside a real kernel.** Ten caller-owned blocks of 32
words travelled from the ping-pong engine to a subscriber as
`Lease::dispatch` loans, every sample inside every block exactly one pace
period (32000 ticks at 2 kHz) after the one before it, every SEAM between
blocks the same, zero engine overruns, and the relay's `published()`
equal to the engine's own `laps()`. Then the other half of the contract:
with nobody draining, the engine STALLED and counted an overrun rather
than write into the block the caller held, and one `release()` brought
the stream back.

**The step verdict of that letter is FLAKY at about one run in eight**
(one failure in eight consecutive `z` runs of the same image, the other
seven clean). What fails is the fine-grained one - "no sample is off the
32000-tick step" - and nothing else in the letter: the blocks arrive,
the seams hold, the accounting agrees. The estimator is
the tightest thing in the suite, an equality between consecutive
free-running TIM2 readings taken by the DMA on TIM3's update, so a
single late request shows as a whole failed verdict. It is recorded
rather than tuned: whoever next maintains this suite has the choice of a
tolerance or of finding what is late.

**THE RECEIVE ENGINE AS A RING** (letter `o`). USART1 in single-wire
half duplex hears every byte it sends (33.5.15), so its two engines -
transmit on channel 2, receive on channel 3 in the circular shape over a
64-byte ring - make a loop with no second board, at 1 Mbaud. ON THE
PAIRED DESK PA9 also carries the link wire to the far board's receive
pad, and an open-drain loop's rise on the pull-up into that load gives
out below 1 Mbaud ([usart.md](usart.md)); so the loop's single wire is
DRIVEN (`single_wire_push_pull`: push-pull while a frame goes out, the
pull-up holding it between frames) and every figure below is the paired
desk's, the far board on a console that leaves its pad hi-Z - twelve
runs of the letter there, all green, the stream's wall within 30 us of
its 40960 us of wire in each. The ring under a FOREIGN sender's clock is
`test_stm32_serial`'s letter `6` ([usart.md](usart.md), "Against a far
board"):

- At rest the channel is CIRCULAR, its count the ring's 64, the lap's
  completion, the half lap and the error armed.
- **4096 bytes came back byte-exact in 40986 us against 40960 of wire**,
  read where the channel wrote them, through 64 laps each counted once
  by its own completion (a restart would have zeroed the count), with no
  ring overrun, no ORE and no fault.
- **The loop's rate is the rate asked**, which the loop itself cannot
  tell, one divisor serving both ends: 1024 frames through the transmit
  engine, timed from the first `write_bulk()` to TC against their time
  at 1 Mbaud, read **1001 thousandths** on both parts (judged -1 % to
  +3 %): the 16-byte transmit ring keeps the wire busy.
- **A burst with the consumer away loses nothing.** With the tail at 40
  of the 64, 60 bytes landed with nobody reading, harvesting or
  re-arming, and all 60 read back exact - the ring wrapped the storage by
  itself. The same stimulus on the ONE-SHOT shape (a run to the end of
  the storage, re-armed by a poll) delivered 22 of the 60: the
  run ended with the storage, the receiver's FIFO and RDR took a few
  more, and ORE stood over the rest.
- **A consumer a lap behind is told and the stream goes on.** 80 bytes
  with nobody reading: ONE overrun counted, nothing of the lapped bytes
  delivered (the view skips rather than tears), the next 20 exact - and
  no ORE: the silicon lost nothing, the consumer did.
- **The edge comes from the vectors** over a ring nobody publishes into:
  the line's IDLE tells once for bytes landing on a ring the consumer had
  drained, a `harvest()` asked after it hears nothing new, a
  consumer that has not drained is not told again, the next byte after a
  drain is an edge again.
- **What a harvest costs**, best of eight on the ruler with the ruler's
  own 65..68 cycles taken out: 65..71 cycles with the consumer told and
  still reading, 102..122 on a drained ring (the look at the producer
  included; two builds differing by one store on the restart path read
  102..105 and 122, the slower with the look behind a backward branch -
  the same instructions, so the 20 cycles are the fetch at two wait
  states, measured and not dissected); a poll over the one-shot
  shape on the same stimulus - the
  receive errors, `take()`, the re-arm test - costs 115..125. In the
  release listing (`-Os`, a transport with both engines) `harvest()` is
  252 bytes and runs about 20 instructions on the told path, about 45
  with the look (the ring's `waiting()`, which writes nothing), three
  peripheral reads at most (USART ISR, CCR, CNDTR), against
  244 bytes, a 136-byte re-arm and about 50 instructions on the quiet
  path of a poll over the one-shot shape. The cost moved to the consumer: a run
  read costs two looks, `HardwareRing::read_span()` 112 bytes and
  `consume()` 88 against `Ring`'s index loads, and a lap costs one
  completion interrupt (the ISR load, one IFCR store, the count) - every
  64 bytes at this ring's size.

### The engines' cost (`bench_stm32` letter `d`)

On the Nucleo-G0B1RE at 64 MHz, the lines in
[../design/benchmark.md](../design/benchmark.md)'s grammar, each the best
of eight runs: `wall` from before the start to the completion seen by the
waiting loop, the completion an interrupt and the wait the masked idle
path. `launch` is the start call alone, timed with interrupts masked
(one ruler read, some 62 cycles, is in it). The wire of a memory
operation is one word in two cycles, 10.4.3's read and write - the same
128 MB/s as the core's own copy floor.

| op | n | wall | busy | irq | isr | launch | x |
|---|---|---|---|---|---|---|---|
| copy | 16 | 595 | 421 | 1 | 93 | 166 | 74.37 |
| copy | 256 | 809 | 423 | 1 | 93 | 166 | 6.32 |
| copy | 4096 | 5609 | 423 | 1 | 93 | 166 | 2.73 |
| fill | 16 | 583 | 409 | 1 | 93 | 154 | 72.87 |
| fill | 256 | 797 | 411 | 1 | 93 | 154 | 6.22 |
| fill | 4096 | 5597 | 411 | 1 | 93 | 154 | 2.73 |
| paced | 1024 | 164340 | 776 | 3 | 181 | 125 | 1.00 |

- **5.00 cycles a word**, copy and fill alike, by the difference of the
  4096- and the 256-byte walls: a fill is no cheaper, because the
  channel reads its cell once a beat. That is 0.8 bytes a cycle, where
  the runtime's own memcpy and memset of 4096 bytes (letter `m`, x 1.41
  and x 1.58 of the core's floors) move 1.42 and 2.53: the core copies
  1.8 times and fills 3.2 times faster than the channel. What a copy
  engine buys is the core given back - 423 cycles of busy for a
  4096-byte block that walls 5609.
- **The fixed cost of a block is some 480 cycles**: the launch (~100
  net of the ruler), the completion's entry and handler (93 with the
  meter's stamps), the WFI's wake and the waiting loop's turn with the
  idle window's own two ruler reads. A 16-byte block is all fixed cost;
  its wall is the launch, the window and the wake around a transfer of
  twenty cycles.
- **`paced`**: 256 words from a table into one cell on TIM3's update at
  100 kHz (`DmaTxEngine<1, 4, uint32_t>`): the wall is the 256 periods
  (163 840 cycles) plus 500 - the launch, the completion, the wake and
  the timer stopped. The timer and the ruler count the same HCLK, and
  the spread of the eight walls is 0 cycles: the last word lands on the
  same cycle every run. A run whose completion lands behind a SysTick
  handler reads 93 more - the moment the completion is SEEN moves, not
  the pace. One completion interrupt a block (the other two are the
  SysTick's), 776 cycles of busy for 2.6 ms.
- **The SPI host's engines** have their numbers in [spi.md](spi.md):
  the same 16 cycles a byte as the wire at PCLK/2, with both widths of
  frame, and some 1090 cycles of fixed cost a transaction.

### The VCP's ceiling (letters `u` and `w`, with `brio stress`)

The board's own USART reaches every rate in the ladder; what the
ST-LINK's virtual COM port carries is another question, and only the host
can answer it.

| baud | board -> host (host verified) | host -> board (board verified) |
|---|---|---|
| 115200 | byte-exact, 7103 bytes | byte-exact, 1792 bytes |
| 460800 | byte-exact, 12000 bytes | byte-exact, 6210 bytes |
| 921600 | byte-exact, 12000 bytes | byte-exact, 11040 bytes |
| 2000000 | **corrupt from byte 6145** | **17963 of 18059 bytes wrong** |
| 3000000 | **7 bytes of 12000 arrived** | the host could not even pump a second chunk |

So this VCP is good to 921600 in BOTH directions and not to 2 Mbaud in
either. The board's own transmitter emitted its full window at every rate
including 3 Mbaud (12000 bytes in 39.66 ms = 302600 B/s, 101 % of nominal
- the arithmetic, not the wire), so the ceiling measured here belongs to
the BRIDGE and not to the USART or its engine. `hw_overruns` and the
frame counter stayed at ZERO throughout, at the good rates and the bad
ones alike, which says the corruption happens upstream of the receiver's
own error detection - the bytes that arrive are well-formed frames
carrying the wrong data.

**THE DEFAULT LADDER THEREFORE STOPS AT THE CEILING, on both sides of
the wire.** Letter `u` runs the three rates the bridge is proven to
carry - 115200, 460800, 921600, six legs - and letter `w`, outside `z`
and outside every verdict, keeps the two above it for whoever wants the
numbers again; `brio stress` declines a leg announced above 921600 in
the same way, running it PASSIVELY (the port follows the announced rate
and drains, but nothing is pumped) unless it is given `--beyond-vcp`.
That ceiling is the tool's default only for a Nucleo's ST-LINK port,
which is where it was measured: a CH340 bridge carries 3 Mbaud and gets
none.

The reason is not tidiness but a lost tally. A leg pumped at a rate the
wire will not take leaves the host's operating system delivering the
queue AFTER the board has gone back to 115200, and those bytes land in
the console's receive ring, where the menu loop reads them as LETTERS.
The board's closing "-> N pass" line is then read at the wrong rate and
never seen, and the script reports "tally: None" - reproducibly, on a
letter whose ten legs have all run and printed their figures.

**AND IT IS THE HOST'S PUMP AND NOT THE BOARD'S RATE, staged with a
control.** Letter `w` runs the same two rungs twice: with `--beyond-vcp`
(the script pumping) all four legs run and the tally is LOST; with the
flag absent (the script following the rate and draining, pumping
nothing) the same four legs run over the same rates and the tally
ARRIVES. So the board emitting 12000 bytes at 3 Mbaud costs nothing -
only the host's own queue does. With `u`'s ladder ending at the ceiling
its six legs report and its tally is read every time (three consecutive
runs).

### The synchronization block (letter `k`)

The stimulus this block needs is already on the board: **table 57's
synchronization input 22 is TIM14_OC**, the same signal table 56 offers
the request generator as trigger input 22 and the same signal letter `f`
produces with no pad at all. So TIM3's update event is the REQUEST at
20 kHz and TIM14's OC1REF is the SYNC at 1 kHz.

In a five-millisecond window:

| arrangement | words moved |
|---|---|
| unsynchronized (the control) | 64 - the whole transfer |
| SE, NBREQ 4, rising | 18 |
| SE, NBREQ 4, falling | 20 |
| SE, NBREQ 4, **both** | 39 |
| SE, NBREQ 1, rising | 5 |

Five sync edges times four requests is twenty words where the same
channel unsynchronized saturates. `SPOL` is a real selector - either
single edge serves the same count and BOTH serves twice as many, which
is the only reading of 11.6.1's three codes a square wave can tell
apart - and **NBREQ is the count and not the count minus one**, which is
what `DmaMuxSync::requests` claims and this measures.

**A SYNCHRONIZATION OVERRUN IS REAL AND REPORTED.** With eight requests
asked for per edge and only about half a request arriving between edges,
`SOF` for that multiplexer channel stands and `CFR` clears it; with
`SOIE` set the same arrangement delivered **7 interrupts** on the third
NVIC line. TWO LEGS, and the reason is that line: the flag is read with
the interrupt off and the shared handler's sweep GATED, because a
handler that clears `SOFx` is the thing that would hide it.

**AND THE VECTOR IS NOT THIS LETTER'S TO ARM.** The DMAMUX overrun
shares the third line with DMA1's channels 4..7, which is where this
suite's own console engines live: enabling it is a no-op, and disabling
it afterwards takes the console's transmitter down with it and leaves
the board silent mid-letter.

**The request generator's other two polarities**: over ten TIM14
periods, rising moves 10 words, falling 10, both 20.

- **DMA2 is five channels and not one** (letter `m`): every one of them
  moves a block at every width, byte for byte. With DMA1's seven -
  1..5 the letters above, 6 and 7 the console's own transmit and receive
  engines - all twelve channels of this part carry traffic.
  DMAMUX's channel numbering runs straight through both controllers,
  DMA2's first being DMA1's count.
- **10.4.5's first sense, run**: TIM6's update paces a channel that
  reads TIM3's CCR1 and writes TIM4's CCR1 - a request from one
  peripheral moving data between two OTHERS, with both increments off.
  Nothing in this controller's vocabulary names the arrangement and
  nothing needs to: it is two addresses and a request id.
- **THE SLEEP STORY, and the two halves have opposite answers for one
  reason** (letter `n`): the controller is on HCLK, which Sleep keeps
  and Stop takes away.
  - **A channel keeps running with the CPU asleep.** A 512-word block
    armed with its completion interrupt, then WFI: the transfer finishes,
    its own completion is what wakes the core, and the destination is
    exact. So the DMA is a way to spend a Sleep, not a thing that stops
    with it.
  - **A channel is FROZEN by a Stop, not broken by it.** A TIM6-paced
    stream at 10 kHz, CNDTR read on both sides of a 60 ms Stop 1 woken
    by an LPTIM1 compare on LSE: 391 before, 391 on the wake, 382 a
    millisecond later. Not one request was served while HCLK was down
    and the very same block goes on afterwards.
  - The alarm's oscillator is the letter's to start. A power-on leaves
    the RTC domain's LSE off (RCC_BDCR's reset, 5.4.23), and a Stop whose
    only way out counts on a stopped clock does not come back - this
    suite arms no watchdog. So the letter starts the LSE when it is not
    running (PWR's bus clock, DBP, LSEON), waits for LSERDY bounded at
    4 s, puts DBP back as it found it and touches nothing else of the
    domain - no reset, RTCSEL and the backup registers as they were -
    and skips the Stop leg by name if the crystal never readies.
    Measured on the STM32G071RB with LSEON and DBP cleared through the
    debug port, the power-on state: LSERDY after 146 ms, the Stop leg
    passed, RCC_BDCR back to 0x8103 and DBP clear again.
  - The wake needs its VECTOR BOUND. A Stop left through an EXTI direct
    line lands in the handler, and with none bound it lands in
    `Default_Handler` and never comes back.

## On the STM32G071RB

The three images are built for the Nucleo-G071RB (DEV_ID 0x460, REV_ID
0x2000) and not run there in this form; one `z` of each gives today's
count (the gap list). Run as one image before the split, the suite scored
76 of 76 there against the G0B1RE's 80 of the same run, the missing four
being the two verdicts the part cannot have and the two that skip with
them, and letters `o`, `p`, `q` and `u` passed, the receive FIFO's kick
firing on this die too, with the copy pair then on channels 1 and 2
([usart.md](usart.md)):

- **There is ONE controller.** `dma_present(2)` is false, so `Dma<2>` and
  `DmaChannel<2, n>` do not compile there at all and DMA1's seven channels
  are the whole of it; the DMAMUX has seven channels to match and the same
  four request generators. Letter c's second-bus-master leg and the whole
  of letter m's DMA2 half skip by name.
- **The peripheral-to-peripheral transfer needs a fourth timer.** 10.4.5's
  first sense is staged with TIM6's update moving TIM3's compare into
  TIM4's, and a part with no TIM4 has no free timer left in this suite's
  set, so that leg skips too. The arrangement itself is not per-part: it
  is addresses with their increments off, and nothing in the vocabulary
  names it.
- Everything else is identical, the three vectors included: `dma_irq(1, 4)`
  is the reserve's derivation on both parts and the handler is bound
  through `BRIO_STM32G0_DMA1_CH4_UP_HANDLER`, which expands to the
  G0B1's four-way name and the G071's three-way one.

**AND ONE ERRATUM RIDES ON THE REQUEST GENERATOR.** ES0418 2.2.4
(STM32G071/G081, revision B, "no workaround") says the EXTI-related
DMAMUX synchronization and trigger inputs are routed to `it_exti_per(y)`
instead of to `exti[15:0]`. Letter f's SWIER leg PASSES on that die (five
software events, five words) and the reason is instructive: the trigger
follows the LEVEL of the line's pending interrupt, and the leg's own loop
clears that pending bit after every pulse. A REAL PAD EDGE does not
survive it - `test_stm32_serial`'s boot probe measures both and
[usart.md](usart.md) carries the numbers - so a staging that uses SWIER
alone would report this path healthy. Letter k's synchronization block is
unaffected: its sync input is TIM14_OC (table 56 input 22) and not an
EXTI line.

## On the STM32G031K8

`test_stm32_dma`'s letters `a` to `n` scored **62 of 69** on the
Nucleo-G031K8 (DEV_ID 0x466, REV_ID 0x1003), counted before letter `h`
timed the console's line against the wire - that verdict, and letter `o`,
are built for this board and not run on it (below). The controller here is **DMA1 with FIVE
channels** and a
DMAMUX with five channel multiplexers and four request generators, so
DMA2's five channels (2 verdicts), the second bus master (1) and the
peripheral-to-peripheral leg (1, which needs TIM4 and TIM6 and this part
has neither) skip by name; the geometry verdict is the cross-check
"the multiplexer has exactly as many channels as the controllers have
between them", which is one claim on all three parts.

**AND THE EXTI PATH INTO THE DMAMUX WORKS HERE.** ES0487 lists the
DMAMUX-from-EXTI limitation (its 2.2.4, ES0418 2.2.4's twin) for
revision Z alone, and this board is revision Y: `test_stm32_serial`'s
boot probe moves 4 words for 4 pad edges with the event mask alone (the
G071RB: 0), so letter f's SWIER leg passes here for the right reason.

**THE CONSOLE GIVES ITS ENGINES BACK.** Two of the seven channels are
the console's transmit and receive on the bigger parts; on five, the
letters own all five, so the console runs on the INTERRUPT transport
here and the three verdicts that measure a DMA-fed console at its
ceiling skip by name. That is a real change of arrangement and it left a
mark: with the CPU in the loop the AHB is shared, and four samples of ten
32-word blocks in the BlockRelay letter arrive **2 ticks long** - the
payload being TIM2's counter READ BY THE DMA, whose access a CPU one in
flight delays by a cycle or two. A torn block is off by a whole pace
period or more, so the check allows the bus's own jitter where the
console shares it and nothing at all where it does not.

**8 KB OF SRAM IS THE OTHER LIMIT.** The three big memory-to-memory
buffers drop from 512 to 256 words on this part (every claim restated for
the length that fits and the length printed beside it), which brings the
image to 48168 bytes of flash and 6288 of RAM - the stack living in the
1904 left.

`BlockRelay` and the two block-stream engines run unchanged on this die,
as does `util/` entire.

## Not covered yet

Driver gaps:

- **The per-item cost is measured and not explained to the cycle**:
  5.00 cycles a word memory to memory, against two AHB accesses and a
  re-arbitration the chapter names but does not time. ST's application
  note on this controller's timing (AN2548) is not on the desk.
- **Peripheral-to-peripheral transfers** in the sense of 10.4.5's first
  bullet are reachable through the existing verbs and are bench-run
  (above), and no vocabulary names the arrangement - deliberately, since
  it is two addresses and a request id.
- **`Dma<n>::reset()` resets the DMAMUX with the controller** (the RCC
  bit's own description), so a program with two live controllers cannot
  use it. Nothing here refuses the call; the caveat is stated on the verb
  and the suite never takes it.
- **No linked-list or repeated-block vocabulary**, because this
  controller has none: a channel is one block, and the only repetition it
  offers is CIRC.
- **A one-shot burst vocabulary** in `util/` - a finite capture started
  and stopped - is absent, and nothing on this controller asks for one
  (`docs/design/block-stream.md` says why).
- **`RCC_AHBSMENR.DMAxSMEN`**, the sleep-mode clock enable, has no verb
  in `clock.hpp` at all ([clock.md](clock.md) carries the gap for the
  whole register group). What it buys is therefore stated by the chapter
  and not measured; what IS measured is that a channel runs in Sleep and
  freezes in Stop with the bits at their reset values.

Implemented but not bench-verified: the circular
receive at a half-word or word beat (compiled at all three in the
family fixture; letter `o` runs bytes, the beat a USART gives - a
converter's stream into a ring would measure the wider ones), the
ring's restart after a transfer error (`harvest()` starting a channel
10.4.7 stopped, the view cleared: a transfer error needs an address no
transport hands the engine, so nothing stages it), and the held request
of an I2C, an ADC and a DAC (letter `r` stages USART1, SPI1 and TIM3 -
an I2C request needs a tenure on a wire, a converter's a conversion; the
reset at their init() is applied by reading), and the contract through
the transports' own verbs (letter `r` stages each requester's release and
`init()` at the register level, in the order the transports take them -
read in their code; a run that releases an engined transport mid-block
and initializes it again, its next block judged at both ends, would
measure the verbs themselves). On the STM32G031K8, letter `o` and letter `h`'s rate verdict on the
interrupt console, built and with no board here (one run of the suite
measures them); letters `p` and `q` skip by name there, the console having
no engines. On the STM32G071RB, the three images as they stand, letter `r`
with the reset at `init()` and letters `p` and `q` with the pair on
channels 4 and 5 (one run of each image measures them). On the STM32G031K8
too, letter `r` (one run of its first image). On both, 10.4.5's
peripheral-to-peripheral transfer: the leg is
arranged over TIM6's update moving TIM3's compare into TIM4's, and those
parts have no TIM4 (the STM32G031K8 no TIM6 either), so it skips by name
there; rearranging it over the timers they have would measure it.

Errata not staged, and why: 2.4.1 is a same-cycle coincidence between a
hardware error and a CGIFx write, and the write does not exist in this
driver; 2.5.1 and 2.5.3 need two channels that can both overrun at once,
which is the configuration their own workaround says to avoid; 2.5.2
needs a trigger landing on the last request of a batch above two, which
this suite's overrun leg deliberately steps around.
