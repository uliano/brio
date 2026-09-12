# DMA (CH32V00x)

The one DMA controller of RM ch. 8 - seven channels, each answering a
fixed handful of peripheral requests, an arbiter over four software
priorities, three flags a channel - and the two engines util-level
transports name in their slots, the console's USART among them.
Documents of record: the CH32V00X reference manual V1.5 (8.2.1 for
the channel and its three outcomes, 8.2.2 for the widths, table 8-2
for the request map, 8.3 for the registers), the QingKe V2 manual
V1.3 (3.5 for the vector table the seven lines sit in).

## What the silicon does

- **The channel IS the request.** There is no request multiplexer:
  table 8-2 wires each peripheral event to one channel and an engine
  is named by channel number with that table in hand - the ADC on 1,
  SPI1 receives on 2 and transmits on 3, USART1 transmits on 4 and
  receives on 5, I2C1 transmits on 6 and receives on 7, USART2 on 6
  and 7 as well, and the three timers spread over all of them (TIM3's
  two events reach channels 1 and 4 alone on the CH32V006/007, the
  table's own footnote).
- **A channel is the STM32F1's**: CFGR (direction, circular,
  memory-to-memory, the two increments, the two widths, the priority,
  three interrupt enables, EN), CNTR counting down as items move,
  PADDR and MADDR - where DIR says which side is the source and in
  memory-to-memory mode both are memory. Every one of those fields is
  read-only while EN is set (8.2.1's note), and a store into an
  enabled channel is dropped in silence.
- **Circular mode is not for memory-to-memory** (8.2.1): the one
  configuration the chapter forbids.
- **Seven vectors, one per channel** (Irq::dma1_channel1..7,
  consecutive), so a handler needs no "which channel" question.
- **Four flag bits a channel** in INTFR, read-only - GIF, TCIF, HTIF,
  TEIF at 4 x (channel - 1) - cleared by writing one to the same
  position in INTFCR.
- **EN STAYS SET after a completed block** (measured): CNTR at zero,
  TCIF and GIF up, the channel enabled and idle. Only software clears
  EN, and until it does every configuring store is refused.
- **A read from a hole in the map completes as a normal block**
  (measured, five holes): 8.2.1 promises TEIF and an automatic EN drop
  for a reserved address, and no address the bench could name
  produced either - zeros arrived, TCIF rose.
- **The block's gate, DMA1EN in RCC_HBPCENR, is closed at reset**
  (the register's reset value opens the SRAM alone).

## Types and verbs

[brio/ch32v00x/dma.hpp](../../brio/ch32v00x/dma.hpp) is the
STM32G0 stratum's design minus that family's request multiplexer.
`Dma` is the block (`open()` the gate, `flags()` the whole INTFR).
`DmaChannel<ch>` is one channel, 1..7: `configure(DmaChannelConfig)`,
`set_count()`, `set_peripheral()` / `set_memory()`, `prepare()` and
`load()` over a `DmaTransfer` (both ends, a count, a config - checked
by `dma_transfer_valid()`), `enable()`, `count()` live, the flags
(`flags()`, `flag(mask)`, `clear(mask)` with `DmaFlag`'s four bits),
`arm(mask, on)` for the three interrupt enables, `isr()` - the ISR
body that reads only the ARMED flags that are up, clears exactly
those with GIF and hands them back, deciding nothing -, `progress()`
and `stop()`. Every configuring verb REFUSES while the channel is
enabled and for the one config the chapter forbids, because a store
the silicon ignores would be a lie the code told itself. `Dma::open()`
is the first thing every channel verb does.

The engines: `DmaTxEngine<ch, Elem>` pours a caller-owned run into one
peripheral register (`arm(data)`, `start(buffer, n)`, `start_fixed(cell,
n)` for a full-duplex bus clocking a read, `complete()` handing back
how many elements the block carried so the owner releases exactly that
much of its ring, `abandon()`, `faults()`) and `DmaRxEngine<ch, Elem>`
fills one from a register (`start(buffer, n)`, `start_discard(cell,
n)`, `take()` - how many arrived since last asked, one CNTR read,
nothing suspended -, `idle()`, `full()`). The element type is the
width: 1, 2 or 4 bytes, anything else refused. Both publish `present`,
`channel`, `service()` (the channel's ISR body) and the flag names, and
a transport reaches its engine only through those, so a driver with an
empty slot never includes this file -
[brio/ch32v00x/dma_engine.hpp](../../brio/ch32v00x/dma_engine.hpp)
is the `NoDmaEngine` tag and the one rule that two engines of one
transport must not name the same channel.

[brio/ch32v00x/usart.hpp](../../brio/ch32v00x/usart.hpp)'s `Uart<1, P,
rx, tx, TxEngine, RxEngine>` is the transport with the two slots: the
transmit engine drains the TX ring by contiguous runs, the receive
engine fills the RX ring's free run and `harvest()` publishes what
arrived. `harvest()` is a VERB, not an interrupt: a receive block
completes only when its run fills, which on an idle line is never, so
whoever owns the port decides how often to ask. `dma_isr()` is the ISR
body of whichever channels the transport owns, and `dma_faults()`
counts the blocks thrown away.

## How to use it

The console on both engines - USART1 transmits on channel 4 and
receives on 5 (table 8-2):

```cpp
using Serial = brio::Uart<1, P, 64, 128, brio::DmaTxEngine<4>, brio::DmaRxEngine<5>>;

extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { (void)Serial::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel4_handler() { (void)Serial::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel5_handler() { (void)Serial::dma_isr(); }

// in the loop, or from a TimeEvent every few ticks:
if (Serial::harvest()) { /* the RX ring went from empty to non-empty */ }
```

A memory-to-memory block on an idle channel, polled:

```cpp
using Copier = brio::DmaChannel<1>;   // the ADC's channel, idle here

Copier::stop();                       // EN off, flags clear - it stays set after a block
(void)Copier::load(brio::DmaTransfer{
    .peripheral = src, .memory = dst, .count = 64,
    .config = {.memory_to_memory = true, .peripheral_increment = true,
               .peripheral_width = brio::DmaWidth::word, .memory_width = brio::DmaWidth::word}});
while (!Copier::flag(brio::DmaFlag::complete)) { }
```

A channel's interrupt: `Copier::arm(DmaFlag::complete | DmaFlag::error,
true)`, `Pfic::enable(Copier::irq())`, and a handler on
`dma1_channel1_handler` calling `Copier::isr()` and acting on the
bits it returns.

## Bench findings

The reference suite is `test_ch32_dma` (23 verdicts in `z`, two more
in its interactive letter) on the CH32V006K8U6 and, the same 23, on
the CH32V003F4P6, both at 48 MHz, ITS OWN CONSOLE ON THE TWO ENGINES: every line it printed left the ring on a
transmit block, every keystroke arrived through a harvest.

- **Memory to memory, three widths, exact**: 64 bytes, 32 half-words
  and 16 words copied and compared, CNTR down to zero, TCIF with GIF
  beside it, the write-one clear taking every flag down; a fixed
  source (PINC clear) filling a run with one word.
- **About six HCLK cycles per item at every width**, the core
  polling beside the transfer: 64 bytes take 723 cycles as bytes, 543
  as half-words, 435 as words from `load()` to TCIF, which is a slope
  of six cycles an item over an overhead of some 340 (the load and
  the poll's own latency). The width is the rate: 8, 16 and 32 MB/s.
- **Five blocks in a row counted by their own handler**, the ISR
  body clearing as it went, no error.
- **A store into an enabled channel is refused**, the hardware
  dropping nothing on its own: the letter reconfigures under a
  running copy and gets false back.
- **EN stays set after a completed block** (the finding the engines'
  `complete()` rests on), and **no hole raises TEIF**: reads from
  0x3000 0000, 0x4003 0000, 0x5000 0000, 0x6000 0000 and 0xF000 0000
  each completed as a four-byte block with zeros, EN still set. The
  error path is armed on every engine and reachable by nothing the
  bench found.
- **The transmit engine**: 272 bytes through a 128-byte ring in seven
  blocks, no fault, the ring empty after; **the receive engine**: a
  thirty-character line typed at the host harvested byte-exact, no
  overrun.

## Not covered yet

Driver gaps, each with its reason:

- The loop and ping-pong engines the STM32G0 stratum keeps for a
  block stream (util/block_stream.hpp's two concepts over a circular
  channel and a pair of halves): born with the ADC chapter, their
  first user here.
- The timer-triggered channels (TIM1/TIM2/TIM3's requests): born with
  the timer chapter.
- Half-transfer as a verb of the engines: the flag and its enable are
  the channel's (`DmaFlag::half`, `arm()`), no engine acts on it
  until a stream needs the midpoint.

Implemented but not bench-verified, each with what would measure it:

- **The transfer-error path** (TEIE, the engines' `abandon()` and the
  fault counters): written from 8.2.1 and provoked by no address the
  suite could find; a peripheral that raises it - or a
  write into flash, which the chapter lists as a legal destination
  and the bench has not tried - would measure it.
- Widening and truncation between unequal widths (8.2.2's table 8-1):
  the channel takes two widths and the engines always set them equal;
  a copy with PSIZE and MSIZE apart, compared against the table.
- The priority arbitration between two channels running at once: the
  suite runs one channel at a time; two memory-to-memory blocks
  started together, their order read off the flags.
