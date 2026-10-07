# DMA (STM32F4)

Documents of record: RM0090 Rev 22 ch. 10, and its twins RM0390 Rev 6 ch. 9
and RM0383 Rev 4 ch. 9 - one IP, three manuals, identical register by
register. What differs between them is the REQUEST MAPPING: RM0090's
tables 43 and 44, RM0390's 28 and 29, RM0383's 27 and 28 are three
different tables. Errata: neither ES0206 Rev 24 nor ES0298 Rev 8 has a DMA
section at all (their items with "DMA" in the title are the DAC's request
behaviour, that chapter's business); ES0287 Rev 6 has one item about this
controller, 2.2.11, quoted below. ES0206 2.2.7 / ES0298 2.2.7, the delay
after an RCC clock enable, reaches every peripheral and is answered in
`stm32f4/clock.hpp`. Driver: `stm32f4/dma.hpp`; the per-part request cells
and the presence facts are in `stm32f4/device_tables.hpp`. Bench suite:
`test_stm32f4_dma`. Family fixture `test/family_stm32f4/dma.cpp` plus six
negatives under `brio check stm32f4`.

## What the silicon does

**A stream is the unit, and a channel is only a multiplexer input.** Two
controllers, eight streams each; a stream owns a FIFO, a priority, a pair
of memory pointers, a counter and A VECTOR OF ITS OWN, and it selects ONE
of eight request lines with `SxCR.CHSEL` (10.3.3). That is the opposite
economy of the STM32G0's controller, where a channel is the unit and a
multiplexer points it at any of 77 request lines: here **a peripheral can
be served by the one or two (controller, stream, channel) cells the request
mapping gives it, and by no other**. The mapping is a fixed wiring per part
class and no device header carries it, so it is keyed on the device-select
define in the reserve exactly as the frequency ladders are - and only for
the four part classes whose manual was read. A part outside them has no
table, and an engine on it is refused rather than run on a guessed channel.

**The tables really do differ.** USART1, USART2, USART3, UART4, UART5 and
USART6 sit on the same cells in RM0090 and RM0390; RM0383 adds a second
receive cell for USART2 (DMA1 stream 7, channel 6) that no other manual
shows, and RM0090's channel 5 rows for UART7 and UART8 are marked
"available on STM32F42xxx and STM32F43xxx only". An instance the part does
not bond has no cells whatever its class's table says about the number.

**Only DMA2 can move memory to memory, and no register says so.** DMA1's
AHB peripheral port is not connected to the bus matrix (figures 33 and 34,
note 1 under each), so the port that would be a memory-to-memory transfer's
SOURCE cannot reach a memory at all. Nothing in DMA1's registers refuses
the mode - the stream would simply take a bus error - so the refusal is the
driver's, at compile time where the controller is a template argument and
at run time where the configuration is a value.

**The FIFO is a configuration, not a buffer the caller sees.** Four words
per stream, a threshold of a quarter, a half, three quarters or full, and
table 49's list of which (memory burst, threshold, memory width) triples
the silicon accepts. The rest are not merely unwise: the stream raises
FEIF **at the enable** and clears its own EN (10.3.18). The rule behind the
table is arithmetic - the threshold in bytes must be a whole number of
memory bursts, and a burst must fit in the four-word FIFO - and the driver
carries the arithmetic rather than the table, which is what lets the family
fixture assert all forty-eight cells against it. The note under the table
is the second half: with a PERIPHERAL burst that fills the whole FIFO
(`PBURST x PSIZE` = 16 bytes), a three-quarter threshold underruns for
ever, and it is refused too.

**Direct mode is the FIFO switched off** (`SxFCR.DMDIS` clear, the reset
state), and it costs three things: the two widths must be equal and are
PSIZE's (MSIZE is *don't care* and forced by hardware), neither port may
burst, and memory-to-memory is forbidden outright (10.3.12). What it buys
is the shortest path from a request to a single beat, which is what a byte
transport wants - and what both engines here use.

**A request is a level served on enable.** 10.3.2's handshake is the
STM32G0's: the peripheral drives its request, the controller acknowledges,
the peripheral releases. A stream enabled while its peripheral's request
already stands is served at once, so there is no software-trigger register
on this controller and no verb here to kick a stalled first beat.

**EN is not a switch, it is a request - in both directions.** A write of 0
takes effect only when the current transfer has finished and, on a
peripheral-to-memory or memory-to-memory stream, when the FIFO has been
flushed into the destination (10.3.14) - so `EN` reads back 1 until then,
and **the flush sets TCIF**, which means a caller that reads the flags
after an abort sees a completion that is really an abort. A write of 1 is
not visible to the load that follows it either (measured; see the findings
below).

**SxNDTR counts PERIPHERAL-side items** (10.3.10) whichever side the
peripheral port is on, and it is a live register readable at any time. That
is what makes a receive engine's "how much has arrived" one register read
and a subtraction, with nothing suspended - and, on a circular stream, what
makes the count register a ring's PRODUCER INDEX: it decrements after the
datum is stored (10.3.2's three operations, the post-decrement last), and
in direct mode a datum is "immediately drained and stored into the
destination" (10.3.6), so a count read says only what memory holds. Where
the two ports have different widths the controller PACKS, and table 48 is
the price: with PSIZE below MSIZE the last memory access would be
incomplete unless NDT divides by MSIZE/PSIZE.

**Suspend and resume is supported here** (10.3.14), unlike on the STM32G0's
controller where the chapter forbids it: disable, wait for EN to read 0,
read SxNDTR, advance the addresses, write the remainder, enable again. The
driver offers the pieces and no verb that pretends to do it, because only
the caller knows how its addresses advance.

**Circular mode reloads SxNDTR and both addresses in hardware** (10.3.8),
and the double buffer (10.3.9) is circular mode with two memory pointers
that the hardware SWAPS at every end of transaction - `SxCR.CT` says which
is current, and setting DBM forces CIRC whatever CIRC says. The one write
into an address register a RUNNING stream allows is the half it is not
using: with CT = 0 only M1AR, with CT = 1 only M0AR, and **the wrong one
sets TEIF and disables the stream**. Circular mode is forbidden with
memory-to-memory and with the peripheral flow controller; a circular block
with a memory burst must be a whole number of bursts, or "the DMA behavior
and data integrity are not guaranteed".

**The peripheral can be the flow controller** (`SxCR.PFCTRL`, 10.3.15) for
the one peripheral of this family that can signal an end of transfer: the
SDIO. NDTR is then forced to 0xFFFF at the enable whatever was written, and
the count transferred is 0xFFFF minus what remains.

**Five flags per stream, in two registers and out through two more.**
LISR carries streams 0..3 and HISR streams 4..7, six bits per stream with
bit 1 reserved, the four groups at 0, 6, 16 and 22 - the shifts do not
tile, which is why the driver's shift is a table. LIFCR and HIFCR are the
write-one-to-clear twins, write-only and bit-independent, so clearing one
stream's flags is a single store and never a read-modify-write. There is no
global bit as on the STM32G0, and therefore none of that controller's
erratum about clearing it.

**Errors come in three kinds and only two of them stop the stream**
(10.3.18). TEIF is a bus error on a read or a write, or a write into the
wrong memory address register under the double buffer, and the hardware
clears EN with it; FEIF raised by an illegal burst/threshold pair at the
enable does the same; FEIF raised by an overrun or underrun, and DMEIF (a
direct-mode request arriving before the previous datum reached memory,
peripheral-to-memory with MINC clear), leave the stream running, because no
data is lost when they happen.

**ES0287 2.2.11, the F411's alone**: DMA2 managing concurrent AHB and APB2
requests can perform a transfer over its PERIPHERAL port several times,
corrupting what a QUADSPI, an FSMC or a GPIO register receives. The
workaround is to put such a destination on the MEMORY port instead. This
driver can express that - DIR names which side is the source and the two
ends are separate arguments - but cannot choose it: only the application
knows what its addresses are. Stated as an obligation on the caller.

## Types and verbs

The driver owns the FABRIC - the block, the streams, the FIFO, the flags,
the vectors - and NOT the request vocabulary: it takes a plain channel
number and knows nothing about USARTs. A peripheral publishes its own cells
of the request mapping, and `stm32f4/usart.hpp` checks an engine's
(controller, stream, channel) against them at compile time.

- Vocabulary: `DmaWidth` (`byte`/`half`/`word`) + `dma_width_bytes` +
  `dma_width_of<Elem>()`; `DmaPriority`; `DmaDirection`
  (`peripheral_to_memory` / `memory_to_peripheral` / `memory_to_memory`);
  `DmaBurst` (`single`/`incr4`/`incr8`/`incr16`) + `dma_burst_beats`;
  `DmaFifoThreshold` + `dma_fifo_threshold_bytes`; `DmaFifoStatus`;
  `DmaFlag` (`fifo_error`, `direct_mode_error`, `transfer_error`, `half`,
  `complete`, `all`, `errors`); `dma_flow_control_count`.
- `DmaStreamConfig` and `dma_stream_config_valid(config, controller)` -
  table 50's combinations and table 49's cells as one predicate;
  `dma_fifo_burst_valid(...)` is table 49 on its own.
- `DmaTransfer` (both ends, the double buffer's second memory, the count)
  and `dma_transfer_valid(transfer, controller)` - table 48, the
  alignments, the circular burst rule; `dma_address_aligned`.
- `Dma<n>` - `instance`, `streams`, `memory_to_memory`, `regs()`,
  `bus_clock(on)` / `bus_clock()`, `init()`, `reset()`, `low_flags()` /
  `high_flags()` / `stream_flags(s)`, `flag_shift(s)`, `clear(s, mask)`,
  `irq(s)`.
- `DmaStream<n, s>` - `controller`, `index`, `memory_to_memory_capable`,
  `regs()`, `irq()`; `enabled()` / `enable()` / `abort(spins)`;
  `configure(config)` / `control()` / `fifo_control()`; the read-backs
  `channel()`, `circular()`, `double_buffer()`, `flow_controlled()`,
  `direct_mode()`, `direction()`, `fifo_status()`; `set_count()` /
  `count()`, `set_peripheral()`, `set_memory()`, `set_memory1()`;
  `current_target_is_m1()` / `set_idle_buffer(address)`; `prepare(t)` /
  `load(t)` / `trigger()`; `flags()` / `flag(mask)` / `clear(mask)` /
  `arm(mask, on)` / `armed()` / `isr()`; `progress(programmed)`; `stop()`.
- `dma_control_word(config)`, `dma_fifo_word(config)`, `dma_beat_word(w)`
  - SxCR's configuration fields, SxFCR's two and a beat's PSIZE/MSIZE pair
  as words: `configure()` writes them, an engine computes its own once.
- `DmaTxEngine<n, s, ch, Elem>` and `DmaRxEngine<n, s, ch, Elem>` - the
  slots `stm32f4/usart.hpp`'s `Uart` and the bus hosts declare, with the
  other strata's surface: `present`, `controller`, `stream`, `channel`,
  `width` and `element` (the WIDEST beat the binding takes),
  `flag_complete` / `flag_half` / `flag_error` / `flag_fifo_error`,
  `service()`, `arm(data, priority, interrupts)`, `start()` over a span of
  bytes, half-words or words up to the element, `start_fixed()` (transmit)
  and `start_discard()` (receive) over a cell of each width, `claim()` and
  `start_claimed()` (transmit), `take()` / `idle()` / `full()` /
  `capacity()` / `taken()` (receive), `complete()` / `busy()` /
  `in_flight()` (transmit), `progress()`, `abandon()`, `faults()` /
  `clear_faults()`, `stop()`. The receive engine's CIRCULAR shape:
  `arm(data, ring, priority)` over a whole span of bytes, half-words or
  words up to the element (the binding and the only start), its ISR body
  `service_ring()`, and `remaining()` / `laps()` - the two functions that
  make the engine util/ring.hpp's `RingCounter`, the producer a
  `HardwareRing` over the same storage reads.
- `DmaCopyEngine<n, s>` - copy and fill on a DMA2 stream, the surface
  every family's copy engine has: `arm(priority)`, `copy(dst, src, n)` and
  `fill(dst, cell, n)` over `n` elements of a byte, a half-word or a word
  (the element is the beat, the fill's cell an element in memory),
  `busy()`, `abandon()`, `faults()`; `controller`, `stream`. Refused at
  compile time on DMA1.
- In `stm32f4/dma_engine.hpp`, beside the empty slot's `NoDmaEngine`,
  what a slot's owner names without including the controller:
  `DmaPriority`, `DmaInterrupts` (`completion`, `errors_only`, `none`) and
  `dma_max_items` (65535, SxNDTR's sixteen bits).
- In the reserve (`stm32f4/device_tables.hpp`): `dma_base`, `dma_present`,
  `dma_streams`, `dma_channels`, `dma_fifo_words`, `dma_stream_base`,
  `dma_stream_present`, `dma_clock_mask`, `dma_reset_mask`,
  `dma_stream_irq`, `dma_memory_to_memory_capable`, `DmaPlacement` /
  `DmaPlacements`, `usart_dma_placements`, `usart_dma_placement_valid`.

## The engines against the chapter

**Two moments, and 10.3.17's procedure says which step is which.**
`arm()` is the BINDING and runs once: the controller's gate opened, the
stream stopped, step 2 (SxPAR), step 8 (SxFCR) and steps 5, 7 and 9
(SxCR: the channel, the priority, the direction, the interrupt enables) -
the configuration word kept in the engine, because a block needs it
back. A block start is steps 1, 3, 4 and 10 and nothing else: the flags
cleared in one store, SxM0AR, SxNDTR, EN. SxCR's configuration is
COMPARED with the word the block needs - its beat and its memory
increment are the block's own - and stored only where it differs, so a
run of bytes after a run of bytes stores four registers and reads one.
ST's library stores five a block and reads SxCR three times
(`HAL_DMA_Start_IT`: NDTR, PAR, M0AR, IFCR, and SxCR read-modify-written
for DBM, the enables and EN).

**A block start never waits.** An engine's own stream between two blocks
is already stopped: EN falls in hardware at the end of a non-circular
block (10.3.13) and at a transfer error (10.5.5), and `abandon()` and
`stop()` wait for it themselves. So a start reads SxCR once, BEFORE any
store, and REFUSES while EN stands - no poll, which is what lets a
transport mask its claim alone - and nothing reads SxCR after the EN
store (the wedge below).

**The beat is the element of the run.** `Elem` is the widest beat the
binding allows - the width of the register the stream is pointed at -
and a start takes a span of any narrower element too: one engine pair
carries an SPI's bytes and its half-word frames, the width from the
span's type. An address not aligned on its element is refused (10.3.6),
and the caller takes its pump.

**The receive engine has two shapes, and the stream decides which a
consumer wants.** A BOUNDED block - the data phase of a bus read, a block
of conversions - is the one-shot shape above: a start a block, the
completion its end. An UNBOUNDED stream - a serial line - is the CIRCULAR
shape: `arm(data, ring, priority)` points the stream at the caller's whole
storage with CIRC set, and that binding is the only start there will be,
because under CIRC "the number of data items to be transferred is
automatically reloaded ... and the DMA requests continue to be served"
(10.3.8). Nothing re-arms the stream between two laps, so there is no
moment at which a byte can arrive with the receiver unserved. The engine
is then a `RingCounter`: `remaining()` is SxNDTR read live and `laps()`
the completions its ISR body `service_ring()` has counted - each one the
end of a lap, the flag raised as the count reaches zero, so the lap count
lags the counter by the handler's latency and never leads it, the order
util/ring.hpp's `HardwareRing` is built on. The half-transfer flag is
armed on request (the binding's `half_mark`), and `service_ring()` hands
it back without counting it: a byte transport's receive ring takes it, so
a stream that never falls silent tells its consumer twice a lap. What the
circular shape cannot do is wait for
a consumer: one that falls a lap behind is written over, and the view
counts that as its overrun and skips. Its own ISR body, so the one-shot
shape's owners pay nothing for a count they have no use for.

**What the chapter offers, and what the engines take of it:**

| the controller's offer | taken? | why |
|---|---|---|
| a register file per stream, every field write-protected while EN is set | yes, written at the binding and compared per block | the configuration is constant for a binding (10.3.17's steps 5-9) |
| PSIZE/MSIZE, three widths | yes, per block from the run's element | the frame is the beat; one pair of streams serves both frame sizes |
| direct mode | yes, the transfer engines | "an immediate and single transfer after each DMA request" (10.3.12) is a peripheral register's shape |
| the FIFO, threshold and bursts (table 49) | the copy engine alone | memory to memory forbids direct mode (10.3.12), and sixteen-byte bursts are a third faster; a transport's runs have no length the burst rules could rely on |
| circular mode, NDTR reloaded (10.3.8) | the receive engine's second shape | an unbounded stream (a serial line) wants a ring the stream never stops writing: SxNDTR is the producer index of a `HardwareRing` ([ring.md](../design/ring.md)), the completion the lap; the transmit side and a bounded block keep the one-shot shape |
| the half-transfer flag | on request, the circular binding's `half_mark` | the lap's half and full marks are the receive edge of a stream with no silence in it, a byte transport's; the USART's idle line is the edge of one that pauses ([usart.md](usart.md)); a lap is otherwise one interrupt |
| the double buffer (10.3.9) | declined | a `BlockSource`, born with its first block user (below) |
| a software trigger or a chain | none on this controller | memory to memory starts on EN; no stream triggers another |
| the gate (RCC_AHB1ENR) | once, at the binding | ES0206 2.2.7's read-back costs an AHB round trip every time it is paid |
| write-one-to-clear flag registers (10.5.3) | one store a block | no read-modify-write, safe from any context |
| the interrupt enables | chosen once (`DmaInterrupts`) | the SPI's transmit stream needs only its errors - the receive block's end proves the transmit block's |
| memory to memory (DMA2) | `DmaCopyEngine` | copy and fill, the beat the element, the bursts from the addresses |
| a timer's request through CHSEL | the paced transfer (bench letter d) | TIM1_UP on DMA2 stream 5, channel 6 - a cell of every manual's table |
| the peripheral flow controller (10.3.15) | declined | the SDIO is the one peripheral that can drive it, and there is no SDIO driver |

**The controller's gate is opened by the stream's configuring verbs**, the
way `stm32f4/pin.hpp`'s port clock is: a caller cannot forget it, and a
program that configures no stream pays for none of it. `Dma<n>::init()` is
for the case where the block is wanted before any stream is.

## How to use it

**A copy and a fill** - memory to memory on a DMA2 stream, polled:

```cpp
using Copier = brio::DmaCopyEngine<2, 4>;   // any DMA2 stream: no request is involved
Copier::arm(brio::DmaPriority::high);
(void)Copier::copy(frame_out, frame_in, 1024);          // 1024 uint32_t: word beats
while (Copier::busy()) { }
static uint16_t background = 0x001F;                     // the fill's cell: in memory, kept until done
(void)Copier::fill(pixels, &background, 320 * 40);      // half-word beats
while (Copier::busy()) { }
```

**The same with the stream's verbs** - every field the caller's, the
general and validating `load()`:

```cpp
using Copy = brio::DmaStream<2, 0>;
brio::DmaTransfer t{};
t.peripheral = source;                 // SxPAR is the SOURCE here
t.memory = destination;
t.count = 2048;
t.config.direction = brio::DmaDirection::memory_to_memory;
t.config.peripheral_increment = true;
t.config.memory_increment = true;
t.config.use_fifo = true;              // direct mode is forbidden in this mode
t.config.fifo_threshold = brio::DmaFifoThreshold::full;
t.config.memory_burst = brio::DmaBurst::incr4;
t.config.peripheral_burst = brio::DmaBurst::incr4;
if (Copy::load(t)) {
    while (!Copy::flag(brio::DmaFlag::complete)) { }
    Copy::clear(brio::DmaFlag::all);
}
```

**A console whose bytes leave and arrive without the CPU** - the two
engine slots the `Uart` task has always declared, filled with the cells
USART1's requests are wired to; the receive engine runs circular over the
transport's whole receive ring ([usart.md](usart.md)):

```cpp
using Tx = brio::DmaTxEngine<2, 7, 4>;     // DMA2 stream 7, channel 4
using Rx = brio::DmaRxEngine<2, 2, 4>;     // DMA2 stream 2, channel 4
using Serial = brio::Uart<1, console_pins, 64, 256, Tx, Rx>;

// The edge comes from three vectors: the USART's (an idle line, a
// burst's first frame) and the two streams' (the lap's marks).
extern "C" void USART1_IRQHandler() {
    if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
extern "C" void DMA2_Stream7_IRQHandler() {
    if (Serial::dma_isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
extern "C" void DMA2_Stream2_IRQHandler() {
    if (Serial::dma_isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}

Serial::init(clock, 115200);
brio::print(serial, "this goes out through a stream", brio::crlf);
```

**A ring the stream writes for ever** - the receive engine's circular
shape under any peripheral, read through the view whose producer it is:

```cpp
using Rx = brio::DmaRxEngine<2, 2, 4>;          // USART1's receive cell
static uint8_t ring[256];                       // a power of two, one lap
using Bytes = brio::HardwareRing<ring, Rx>;     // remaining() and laps() are the producer
extern "C" void DMA2_Stream2_IRQHandler() { (void)Rx::service_ring(); }   // a lap counted

brio::Usart<1>::dma_receive(true);
(void)Rx::arm(brio::Usart<1>::data_address(), std::span<uint8_t>(ring));
Bytes::clear();                                 // the stream starts at ring[0], laps at zero
// the consumer, as often as a lap needs:
const auto run = Bytes::read_span();            // contiguous, never wrapping
use(run);
(void)Bytes::consume(run.size());               // false: the run was written over while held
```

A wrong cell is a compile error naming the tables, and so is a part class
whose manual was not read.

**A table played into a peripheral for ever** - circular mode, with nothing
in the CPU's path between laps:

```cpp
brio::DmaTransfer t{};
t.peripheral = brio::Usart<1>::data_address();
t.memory = const_cast<char*>(table);
t.count = sizeof(table);
t.config.channel = 4;
t.config.direction = brio::DmaDirection::memory_to_peripheral;
t.config.circular = true;
t.config.memory_increment = true;
brio::Usart<1>::dma_transmit(true);
(void)brio::DmaStream<2, 7>::load(t);
```

**Two buffers filled in turn** - the double buffer, with the half the
stream is not using replaced under it:

```cpp
t.memory1 = second_half;
t.config.double_buffer = true;         // forces circular
(void)Stream::load(t);
// in the handler, on the completion flag - the target has just changed:
(void)Stream::set_idle_buffer(next_block);
```

**Stopping one, and picking it up again** (10.3.14):

```cpp
if (Stream::abort()) {                       // waits for EN to read 0
    const uint16_t left = Stream::count();   // and this is what is left
    t.memory = &buffer[length - left];
    t.count = left;
    (void)Stream::load(t);
}
```

## Bench findings

Measured on the STM32F429ZI at 180 MHz (over-drive) and on the
STM32F469NI, where the console's engines sit on DMA1 (USART3: stream 3
out, stream 1 in, channel 4 both - RM0386 table 29), before the receive
engine's circular shape; `test_stm32f4_dma` runs 103 verdicts in `z` on
the STM32F446RE with it, and letter u beside `brio stress`. The engines'
costs are `bench_stm32f4`'s letter d on the STM32F446RE at 180 MHz.

**A block start is one read and four stores, 37 cycles.** One
`DmaTxEngine::start()` on an idle stream, timed on the DWT with its read
included: 37 cycles - about twenty instructions inlined into the caller,
SxCR read once, then the flag clear, SxM0AR, SxNDTR and EN, and a fifth
store only when the block's beat or increment differs from the last.
Built from a validated transfer, aborted and reconfigured field by field,
the same start took 647. What that bought where it is spent, measured in
the same letter:

| | the block start rebuilt (647 cycles) | two moments (37 cycles) |
|---|---|---|
| an engined SPI request, wall minus its wire (SCK 22.5 MHz) | 2472 cycles, 2 interrupts | 652 to 664 cycles, 1 interrupt |
| the same, 16-bit frames | the pump: 1 interrupt a frame, 2.35 x the wire at 256 frames | the engines: 1 interrupt, 1.01 x |
| a 16-byte copy, start to completion | 1040 cycles | 86 cycles |
| a transport's masked window | the whole start | the claim: twelve instructions |

The SPI's numbers are spi.md's, the masked window usart.md's.

**The DMA copies memory slower than the core, and frees it.** Word beats
with sixteen-byte bursts at both ports, SRAM to SRAM: 0.88 cycles a byte
at the margin (4096 bytes in 3707 cycles) and a fixed 72 cycles a block,
start to completion; a fill, its source the caller's cell read single,
1.26 cycles a byte and 85 fixed. The runtime's memcpy moves the same 4096
bytes in 2818 and memset in 1540 (`bench_stm32f4` letter m).
Against one beat a cycle of the DMA's clock - four bytes a cycle - the
copy runs at 3.6 x: both ports serve one SRAM, so every word is a read
and a write on the same slave, two cycles at best, and the FIFO's
bursts are the rest (the throughput ladder below says the same at 2048
bytes). What the engine buys is the core's time and not the block's.

**A timer-paced stream holds its pace to the tick.** TIM1's update at 1 MHz
moving the timer's own counter (through TIMx_DMAR, the burst base on CNT)
into 256 words: every word reads 3 ticks of 180 MHz after its update - no
spread at all, one interrupt for the block, 186 busy cycles while the core
idled through its 256 us.

**The enable does not read back in the load that follows its store.** This
is the finding that shaped a verb. A store into SxNDTR is visible to the
very next load - zero stale reads, measured - but the EN bit is not: the
load right after the store reads 0 and the one after it reads 1, so a
read-back taken at once reports a running stream as one that refused to
start - and on the STM32F446 that load WEDGES the stream (below).
`DmaStream::enable()` therefore stores EN and reads nothing back, and
letter b's "it starts on the enable alone" is what stands guard over
that.

**A memory-to-memory block is over in microseconds**, which is the second
reason `enable()` accepts a completion as an answer: 256 bytes take about
1.4 us at 180 MHz, less than the time a print takes to reach the wire.

**The throughput ladder**, 2048 bytes SRAM to SRAM, both ports at the same
width and the same burst, threshold full:

| beat | single | INCR4 | INCR8 |
|------|--------|-------|-------|
| byte | 8748 cycles (0.23 B/cycle) | 6154 (0.33) | 5398 (0.37) |
| half-word | 4632 (0.44) | 3382 (0.60) | 2962 (0.69) |
| word | 2616 (0.78) | 1954 (1.04) | refused by table 49 |

**The width is worth more than the burst, and both are worth having.** A
word beat moves four bytes for the two bus accesses a byte beat pays for -
3.3x - and bursting both ports on top of that buys another 1.3 to 1.6x.
The fastest legal configuration on this silicon moves 2048 bytes in 1954
core cycles, a byte and a fraction per cycle; the slowest legal one takes
4.5 times as long for the same block.

**All forty-eight cells of table 49 answer as the table says**: twelve
single-burst cells plus eleven bursting ones are accepted (seven at the
byte width, three at the half-word, one at the word), the other twenty-five
are refused before the enable, and every accepted one moves its block
byte-exact with no FIFO error.

**Priority arbitration, measured**: a 512-byte memory-to-memory block takes
2337 cycles with the controller to itself (2321 at 100 MHz on the
STM32F411 - the DMA's clock is HCLK). Two of them started a few cycles
apart finish in 2475 and 4300 cycles when one stream is `very_high` and the
other `low` - and the order follows the PRIORITY and not the stream index,
because reversing the two reverses the result (2408 against 4323; on the
STM32F411 2726 against 4312 and 4299 against 3059). So the loser pays 1.8x
for the company - what sharing two AHB ports costs - and what the winner
pays is within the measurement (2626 and 2285 against 2340 alone, the
second figure for the stream enabled first).

**Two enables back to back can STALL the second stream.** On the STM32F411,
a `low` memory-to-memory stream whose EN was stored right after a
`very_high` stream's EN on the same controller started - its FIFO filled,
SxNDTR dropped by 16 - and then never moved again, no flag raised, EN still
set, long after the winner had completed: deterministic over five runs of
two builds with that spacing, gone with eight NOPs between the two stores,
and gone again with a not-taken branch between them - a window a few cycles
wide, which is why the suite's back-to-back probe reports what it sees and
judges nothing. The STM32F429 did not show it at 180 MHz. `stop()` recovers
such a stream like any other. A program that starts two streams of one
controller together gives the second enable a few cycles.

**A load of SxCR in the cycles after the EN store WEDGES the stream - on
the STM32F446 and the STM32F411, every other time.** A memory-to-memory block enabled and
then asked at once whether EN took (a read of SxCR in the instruction
after the store) stops sixteen bytes in on every second block: the FIFO
full, nothing drains, EN standing, no flag, and it stays so until the
controller is reset through the RCC - `abort()` cannot bring EN down.
The same block with the store left alone completes, back to back, as many
times as asked; a read of LISR at once is harmless; ST's library, which
never reads back, never sees it. It looked like a property of the second
block (twenty variants of register sequence, memory, width, stream, clock
and reset all "hung"), until a program built on that library alone ran
the blocks back to back on the same board and the one difference left
was the read-back. The STM32F429 takes the read-back without harm (and
the STM32F411's earlier "stall of a low-priority stream enabled right
after a very-high one" was this: the second enable's read-back). No
errata sheet has an item. `enable()` therefore stores
EN and reads nothing back - the one refusal the silicon could make at
that point, 10.3.18's burst against the threshold, is `configure()`'s
before the store - and a program that wants to know whether a stream is
running asks later, as a handler or a poll would anyway. `Dma<n>::init()`
is the way back for a stream a read-back wedged.

**The abort's wait is real and short**: EN comes down in 33 core cycles on
an idle stream and 74 on one in the middle of a memory-to-memory block -
the flush - and the flush sets TCIF, so an abort looks like a completion to
anything that only reads the flags. SxNDTR then says exactly how much was
left (2007 of 2048 in the suite's run), and 10.3.14's resume from that
count finishes the block.

**Circular mode needs no CPU at all**: five laps of a sixteen-byte table
into the console's transmitter, with nothing re-arming anything, NDTR
reloaded to 16 at every wrap, and the half-transfer flag falling exactly at
NDTR = 8.

**The double buffer swaps in hardware and can be re-pointed while it
runs**: four laps, four CT flips, the first from memory 0; the idle half
replaced at the completion of the second lap appears on the third with no
transfer error - `[buffer A]`, `[buffer B]`, `[buffer A]`, `[buffer C]` on
the console.

**A transfer error is a bus error, and the address at 0x1000 0000 is one**:
the core-coupled memory is on the CPU's own D-bus and no DMA master reaches
it (and on the parts without a CCM nothing is mapped there at all). A
memory-to-memory block writing there raises TEIF alone - no FIFO error, no
direct-mode error - the hardware clears EN, and SxNDTR says where the block
died (47 of 64 items left: the FIFO had already absorbed the rest). What
the enable ANSWERS there is a race the core clock decides - `false` on the
STM32F429 at 180 MHz, where the stream is dead before the read-back poll
ends, `true` on the STM32F411 at 100 MHz - so a caller judges an error by
the flags, never by the enable; the stream takes the next block once the
flags are cleared.

**The console's own transmitter is a DMA console**: 575 bytes through the
transmit stream in 50 ms, which is the LINE's time and not the CPU's (a
byte is ten bits at 115200, so the block's own length says 49 ms), with no
block thrown away.

**Single-wire half duplex is the wire this family does not need**: with
HDSEL the receiver listens on the transmit pad, so everything the port
sends it also hears - 128 bytes out through the transmit stream and 128
back in through the receive stream, byte for byte, on one pad and no wire.

**A buffer a bus master writes must be `volatile`.** The compiler sees
an address handed to a register as an integer nothing dereferences, and
is free to keep the buffer's old bytes in a register across the
transfer: a value that printed as zero compared equal to the pattern in
the same function said so (measured in the accelerator's chapter, true
of every stream here).

**The circular receive loses nothing between laps - and the one-shot one
did lose between runs.** The receive side of an engined `Uart` was a run
of the ring that stopped when it filled and waited for `harvest()` to
start the next; the bytes that arrived in between were lost, the first
one held in DR with ORE raised behind it. Letter u feeds the receive
engine from the host (`brio stress`, its xorshift at three rates for a
second) with the consumer paced by the tick - one look a millisecond, as
a TimeEvent would look - over a ring of 256 bytes, and judges every byte
against the generator, resynchronising over a gap to count it:

| rate | the run re-armed by `harvest()` | the circular stream |
|---|---|---|
| 115200 | 10302 of 10688 in, 386 lost in 73 gaps, the first at byte 257; ORE 73 | 10688 of 10688, no gap, ORE 0 |
| 460800 | 27367 of 30820 in, 3453 lost in 203 gaps; ORE 204 | 30820 of 30820, no gap |
| 921600 | 28705 of 32200 in, 3495 lost in 217 gaps; ORE 218 | 32200 of 32200, no gap |

The run engine's gaps and its ORE counts agree to within one: a gap is a
run that filled before the next look re-armed the stream, the first after
256 bytes (the 255 of the first run and the one DR held while the stream
stood still). The circular stream's received counts are the host's sent
counts to the byte. The half-duplex echo says the same with no wire: 128
bytes through a ring of 64, the run engine reading 127 (the gap at the run
boundary, no flag at all), the circular stream all 128 - two laps counted,
SxNDTR reloaded to 64 by the hardware at the second wrap, and CIRC still
set at the end: nothing re-armed it.

**A lap slept through is counted once and skipped.** 160 bytes through a
ring of 64 with nobody looking: the stream counts three laps, the
consumer's first look finds the tail more than a lap behind the head,
counts ONE overrun and skips to the head (nothing offered), and the 32
bytes sent after it read whole and in order. The run engine in the same
place offered nothing, counted nothing and raised ORE.

**What one `harvest()` costs** (the least of three, the ruler's own pair
taken off): 57 core cycles with nothing new and 84 with bytes waiting -
the restart flag, one look of the view (SxNDTR and the lap count) and the
gate, no call and no register of the USART: the edge is the vectors' now
([usart.md](usart.md)), and `harvest()` the same gate asked from the
consumer's side by an owner that still asks.

**The reset state**: both AHB1 gates closed, every flag register at zero,
every stream's SxCR at zero and SxFCR at 0x21 (the FIFO empty, threshold a
half, direct mode on).

## Not covered yet

Driver gaps, each with its reason:

- **The request mapping beyond the peripherals that use it.** The reserve
  carries the cells of the serial instances, the SPIs, the I2Cs, FMPI2C1,
  the ADCs, the DAC and QUADSPI - the peripherals whose drivers hold an
  engine. The timers' rows, the SDIO's, the SAIs', the DCMI's, CRYP's and
  HASH's and SPDIFRX's are born with the first driver that takes an
  engine there: a row for a peripheral no code can reach is a list
  somebody has to keep. The paced transfer of `bench_stm32f4` names
  TIM1_UP's cell (DMA2 stream 5, channel 6, the same in the three tables)
  by hand.
- **The part classes whose manual is not on the desk** (F401, F410, F412,
  F413/F423) have no request table and an engine on them is refused,
  exactly as a clock above the reset rate is: RM0368, RM0401, RM0402 and
  RM0430 would each add one entry.
- **The block-stream engines** (`util/block_stream.hpp`'s `BlockPlayer` and
  `BlockSource`, which the SAM C21 and the STM32G0 have as
  `DmaLoopEngine` / `DmaPingPongEngine`) are not built here. This
  controller's DOUBLE BUFFER is a better ping-pong than either of theirs -
  the hardware swaps the target, so there is no re-arm window and no race
  between a handler and a running stream - and the shape that uses it
  belongs with its first user, not before it.
- **The peripheral flow controller is configurable and untested**: the SDIO
  is the one peripheral of this family that can signal an end of transfer
  (10.3.15), and there is no SDIO driver in this stratum. The verbs and
  `dma_flow_control_count` are there, and the family fixture prepares such
  a stream; nothing on the bench can make one run.
- **PINCOS with a peripheral burst, and the packing table's exotic rows.**
  `peripheral_increment_fixed4` is written and read back; the increment
  offset it selects only matters for a peripheral whose data registers are
  four bytes apart, which is a peripheral chapter's case.
- **The double buffer's error case is not provoked**: writing the memory
  address register the stream IS using sets TEIF and disables the stream
  (10.3.9). `set_idle_buffer()` makes that unreachable through the driver -
  it reads CT and writes the other one - so there is no path to it, and the
  suite measures the legal write instead.

Implemented but not bench-verified, each with what would measure it:

- **FEIF and DMEIF.** Both are implemented (the flag, the enable, the ISR
  body, the accounting), and neither can be raised through this driver:
  every burst/threshold pair table 49 forbids is refused before the enable,
  and a direct-mode error needs a peripheral requesting faster than the bus
  matrix can grant. A second bus master hammering the same SRAM while a
  direct-mode peripheral-to-memory stream runs with MINC clear would
  measure DMEIF; a driver that could write SxFCR behind `configure()`'s
  back would measure FEIF, and deliberately is not offered.
- **DMA1's streams** are exercised only through the F446's console cells
  and the refusals; every measurement of the fabric above runs on DMA2,
  because memory-to-memory is the instrument and DMA1 has not got it. A
  peripheral on DMA1 with its own suite - a timer, an SPI - is what would
  measure the second controller's own arbitration and flags.
- **The copy engine's narrower beats and its unaligned bursts.** Letter d
  copies and fills words between sixteen-byte boundaries; the half-word
  and byte beats, a port that starts off a sixteen-byte boundary (single
  beats there) and `abandon()` are compiled and checked in the family
  fixture and not run. A copy letter over the three widths at every
  misalignment would measure them, as `test_rt` does for the runtime.
- **The engines on the other three boards.** Letter d and the suites ran on
  the STM32F446RE; the F429's SPI and I2C suites and the F411's and F469's
  `test_stm32f4_dma` (letters m and n among them, the circular receive on
  DMA2 and on DMA1's other cells) are owed a run with the two-moment
  engines and the circular shape.
- **The circular shape's half-word and word rings, and its restart.**
  `arm(data, ring)` over a span of `uint16_t` or `uint32_t` is compiled
  (the family fixture) and run nowhere - a converter's stream would
  measure it ([adc.md](adc.md)); the transport's re-binding after a
  transfer error stopped the stream is never taken here, a receive
  stream's bus error wanting an address no DMA master reaches - a ring
  placed at letter j's 0x1000 0000 under the transport would take it.
- **ES0287 2.2.11** (the F411's DMA2 corruption on concurrent AHB and APB2
  requests) is stated as a caller obligation and not measured: it wants a
  QUADSPI, an FSMC or a GPIO register as a DMA destination on that part.
- **The read-back wedge of the STM32F446 and the STM32F411, characterized.** It is measured,
  reproduced at will and avoided; how many cycles after the store a load
  of SxCR stays dangerous, and why every other block, are not measured
  (what would: a sweep of the load's distance from the store in assembly
  on that part), and no errata sheet has an item.
