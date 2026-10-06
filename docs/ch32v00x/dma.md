# DMA (CH32V00x)

The one DMA controller of RM ch. 8 - seven channels, each answering a
fixed handful of peripheral requests, an arbiter over four software
priorities, three flags a channel - the engines the transports name in
their slots (the USART's, the SPI host's, the I2C host's), and the copy
engine. Documents of record: the CH32V00X reference manual V1.5 (8.2.1
for the channel and its three outcomes, 8.2.2 for the widths, table 8-2
for the request map, 8.3 for the registers), the CH32V003 reference
manual V1.9 (its chapter 8 is the same controller, table 8-2 the same
rows but TIM3's), the QingKe V2 manual V1.3 (3.5 for the vector table
the seven lines sit in). Driver: [brio/ch32v00x/dma.hpp](../../brio/ch32v00x/dma.hpp)
and [brio/ch32v00x/dma_engine.hpp](../../brio/ch32v00x/dma_engine.hpp);
reference suite `test_ch32_dma`; the costs in `bench_ch32`'s letters d
(the controller) and s (the SPI host's engines).

## What the silicon does

- **The channel IS the request.** There is no request multiplexer:
  table 8-2 wires each peripheral event to one channel and an engine
  is named by channel number with that table in hand - the ADC on 1,
  SPI1 receives on 2 and transmits on 3, USART1 transmits on 4 and
  receives on 5, I2C1 transmits on 6 and receives on 7, USART2 on 6
  and 7 as well, TIM1's update on 5 and TIM2's on 2, and the three
  timers' other events spread over all of them (TIM3's two events
  reach channels 1 and 4 alone on the CH32V006/007, the table's own
  footnote). The rows of one channel are an OR: whichever of its
  peripherals has its DMA bit set drives the channel's transfer.
- **A channel is the STM32F1's**: CFGR (direction, circular,
  memory-to-memory, the two increments, the two widths, the priority,
  three interrupt enables, EN), CNTR counting down as items move,
  PADDR and MADDR - where DIR says which side is the source and in
  memory-to-memory mode both are memory. 8.2.1's note makes PADDR,
  MADDR, CNTR and CFGR's DIR, CIRC, PINC and MINC writable only while
  EN is clear, a store into an enabled channel dropped in silence.
- **Circular mode is not for memory-to-memory** (8.2.1): the one
  configuration the chapter forbids.
- **Seven vectors, one per channel** (Irq::dma1_channel1..7,
  consecutive), and **one flag register for all seven**: GIF, TCIF,
  HTIF, TEIF at 4 x (channel - 1) in INTFR, read-only, cleared by
  writing one to the same position in INTFCR. Each enable in CFGR sits
  at its flag's position (TCIE bit 1, HTIE 2, TEIE 3).
- **EN STAYS SET after a completed block** (measured): CNTR at zero,
  TCIF and GIF up, the channel enabled and idle. 8.3.4's note: a
  channel whose count is zero moves nothing, enabled or not. Only
  software clears EN, and until it does every configuring store is
  dropped.
- **The address's low bits are IGNORED for a wide access** (8.3.5,
  8.3.6): with a 16-bit width the module drops bit 0, with a 32-bit
  one bits 1:0 - a half-word block pointed at an odd address moves the
  wrong bytes and completes.
- **A read from a hole in the map completes as a normal block**
  (measured, five holes): 8.2.1 promises TEIF and an automatic EN drop
  for a reserved address, and no address the bench could name
  produced either - zeros arrived, TCIF rose.
- **About six HCLK cycles an item at every width** (measured): the
  controller is a master of the same bus matrix as the core, at HCLK,
  and a memory-to-memory item is a read, a write and the arbitration.
- **The block's gate, DMA1EN in RCC_HBPCENR, is closed at reset**
  (the register's reset value opens the SRAM alone).

### The offer, item by item

What the chapter gives that bears on what a block costs, and what the
engines do with each:

| item | used | how, or why not |
|---|---|---|
| the register file (CFGR, CNTR, PADDR, MADDR) | yes | PADDR and the configuration word at `arm()`, once; a block is FIVE STORES - CFGR with EN clear, CNTR, MADDR, INTFCR, CFGR with EN - and no load. The word is kept in the engine, because the enable store needs it and a read-modify-write would read it back from the bus |
| the write protection of 8.2.1 | yes | the block's first store is the word with EN clear: the disable the note asks for before CNTR and MADDR, which EN standing from the last block makes necessary, and the store that changes the beat or the increment in the same access |
| the two width fields | yes | the BEAT: PSIZE and MSIZE both follow the element of the run a block is handed (8, 16 or 32 bits), so one channel moves bytes for one block and half-words for the next; unequal widths (table 8-1's packing and truncation) declined - no transport's register and buffer differ in width |
| the address alignment of 8.3.5/8.3.6 | yes | a run off its beat's boundary is REFUSED, its caller falling back to whatever moves it otherwise |
| the increments | yes | MINC per block (a run, or one fixed cell a bus write clocks out or a discard cell a bus read fills); PINC for the copy |
| memory to memory | yes | `DmaCopyEngine`: no request, the channel runs from its enable |
| circular mode | the receive engine's circular shape (`arm_ring()`) | a byte transport's receive ring: the channel writes the whole storage lap after lap, its count the producer index of util/ring.hpp's `HardwareRing`, the completion a lap counted by `lap()` (the transport's vector); a transfer engine's block has an end, and a circular player waits for its first block-stream user |
| the half-transfer flag | no | no engine acts on a midpoint before a stream does |
| the completion interrupt | per binding | `arm()` names the flags whose interrupt the binding wants: a transport whose completion another event proves arms the errors alone - the SPI host's transmit block (the receive block's end proves it), the I2C host's (BTF proves it) |
| one flag register for seven channels | yes | a transport serving two channels in one handler body reads INTFR ONCE |
| the priorities | yes | an argument of `arm()`, low by default; ties go to the lower channel number (8.2.1) |
| the software trigger | yes | MEM2MEM is the only one (figure 8-1): there is no trigger register beside it |
| the gate | yes | opened once, by `arm()` |
| the transfer error | yes | armed on every engine; it clears EN by itself (8.2.1) and an engine's `abandon()` halts the channel and counts the fault |

The controller has no FIFO, no burst, no double buffer and no chaining
of one channel to another: there is nothing more to use or decline.

## Types and verbs

[brio/ch32v00x/dma.hpp](../../brio/ch32v00x/dma.hpp):

- `Dma` is the block: `open()` the gate, `flags()` the whole INTFR.
- `DmaChannel<ch>` is one channel, 1..7, with its whole register
  surface as ONE-SHOT verbs for a suite or a rare user:
  `configure(DmaChannelConfig)`, `set_count()`, `set_peripheral()` /
  `set_memory()`, `prepare()` and `load()` over a `DmaTransfer` (both
  ends, a count, a config - checked by `dma_transfer_valid()`, and
  both ends' alignment against their widths), `enable()`, `count()`
  live, the flags (`flags()`, `flag(mask)`, `clear(mask)` with
  `DmaFlag`'s four bits), `arm(mask, on)` for the three interrupt
  enables, `isr()` - the ISR body that reads only the ARMED flags that
  are up, clears exactly those with GIF and hands them back, deciding
  nothing -, `progress()` and `stop()`. Every configuring verb REFUSES
  while the channel is enabled and for the one config the chapter
  forbids, and opens the gate first.
- `DmaTxEngine<ch, Elem>` and `DmaRxEngine<ch, Elem>` are the
  TRANSFER ENGINES, in TWO MOMENTS. `arm(data, interrupts, priority)`
  binds the channel to its peripheral register: the gate opened, the
  channel stopped, PADDR written, the configuration word (direction,
  priority, the interrupt enables the binding wants - `flag_complete`
  and/or `flag_error`, both by default and with `arm(data, priority)`)
  computed and kept, the PFIC line enabled. A block takes a span of
  uint8_t, uint16_t or uint32_t - the BEAT is the element, `Elem` the
  WIDEST the binding allows (the register's width; a wider span does
  not compile) - and writes its five stores; it is refused for an
  empty run, one over 65535 elements or one off its beat's boundary.
  The transmit engine is BUSY from its claim to `complete()`, which
  hands back how many elements the block carried and touches no
  register (EN stands; the next block's first store clears it). THE
  CLAIM AND THE PROGRAMMING ARE TWO VERBS: `claim()` is that flag's
  test-and-set and nothing else, the one step a transport whose
  completion handler starts the next block takes under its mask;
  `launch(run)` programs a claimed engine, outside it (a refused run
  gives the claim back); `unclaim()` gives a claim back unused;
  `start(run)` is the two together, for an owner with no handler to
  race, false on a busy engine; `start_fixed(cell, n)` claims and
  sends `n` copies of one element, what a full-duplex bus clocks a
  read with. The receive engine's `start(run)` fills a run and
  `start_discard(cell, n)` pours `n` elements into one cell (the
  receive side of a bus write); it answers `take()` (how many arrived
  since last asked: one CNTR read, nothing suspended), `full()`,
  `capacity()`, `idle()` (EN clear: stopped, or halted by a transfer
  error). Its CIRCULAR SHAPE, `arm_ring(data, storage, half_mark,
  priority)`, binds the channel to a caller's whole ring storage and
  starts it for good - CIRC, MINC, the beat, the lap's completion and
  the error, and with `half_mark` the half lap - and makes the engine
  util/ring.hpp's `RingCounter`: `remaining()` is CNTR (8.2.1's order:
  the store before the decrement, so the count never counts an element
  not yet in memory), `laps()` the completions `lap()` counted in the
  channel's handler, lagging the count by a handler's latency and never
  leading it. Both publish `present`, `channel`, `width`, `element`, the
  flag names, `service()` - the channel's ISR body over its armed
  flags - with an overload taking an INTFR the caller read once for
  two channels, and `block_flags()` to read it; `abandon()` halts the
  channel and counts the fault, `faults()`, `stop()`. A transport
  reaches its engines only through these names, so a driver with an
  empty slot never includes this file.
- `DmaCopyEngine<ch, Elem>` is memory to memory on any channel (the
  number says where it sits in the arbitration and which vector
  reports it): `arm(priority, interrupt)` once; `copy(dst, src, n)` -
  the source the PADDR side, both increments - and `fill(dst, cell, n)`
  - the source one cell IN MEMORY, the caller's, PINC clear - with `n`
  in elements of the beat the pointers' type names, each refused while
  a block runs and for an `n` of zero or over CNTR's 65535 or an end
  off its beat's boundary; `busy()`, which while a block runs reads the
  flags and frees the engine at the completion or the error it finds
  (the witness when the engine was armed with no interrupt);
  `abandon()`, which stops a running block where it stands (EN
  cleared, CNTR holding what was left) and frees the engine; `isr()`,
  the ISR body that frees it from the vector; `faults()`. Elem
  defaults to uint32_t.
- [brio/ch32v00x/dma_engine.hpp](../../brio/ch32v00x/dma_engine.hpp)
  is the `NoDmaEngine` tag of an empty slot, `dma_engine_channel()`
  and the one rule that two engines of one transport must not name the
  same channel.

What a block costs, counted in the release listings of the CH32V006
images - the instructions on the path and the bus accesses - beside
the WCH examples' restart of a channel: the disable and the enable as
read-modify-writes of CFGR around CNTR, MADDR and the flag's clear,
six or seven accesses through as many library calls, the LCD port of
the WUI example writing the configuration word whole with EN clear
between them - which is this engine's first store:

| path | instructions | accesses to the controller | interrupts |
|---|---|---|---|
| the USART's block start (`pump_tx`, a run in the ring) | 39, no call | 5 stores, no load | - |
| the USART's mask around it | 6, between the csrrci and the csrsi: the claim, a byte in RAM | none | - |
| the USART's completion vector (a block done) | 20 to the next start | one INTFR load, the INTFCR store | one a block |
| the SPI host's launch (both channels) | 59, no call | 10 stores, no load (and SPI1's two DMA-request read-modify-writes) | - |
| the SPI host's completion | about 60 (the vector, `finish_dma()`, the select) | one INTFR load, the INTFCR store | ONE a transaction, the receive channel's |
| a memory-to-memory block (`DmaCopyEngine::copy()`) | about 25 | 6 stores (PADDR the sixth) | one a block, or none polled |

No engine opens the gate, reads CFGR or validates its binding on the
way: those are `arm()`'s, once.

## How to use it

The console on both engines - USART1 transmits on channel 4 and
receives on 5 (table 8-2):

```cpp
using Serial = brio::Uart<1, P, 64, 128, brio::DmaTxEngine<4>, brio::DmaRxEngine<5>>;

// The receive edge from the vectors: the USART's (an idle line, a
// burst's first frame) and the receive channel's (the lap's marks).
extern "C" BRIO_CH32_INTERRUPT void usart1_handler() {
    if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel5_handler() {
    if (Serial::dma_isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel4_handler() { (void)Serial::dma_isr(); }
```

The SPI host with engines at DATAR's width, so that 16-bit frames ride
them in half-word beats (a request whose buffer sits on an odd address
goes to the pump):

```cpp
using Bus = brio::SpiHost<1, brio::spi1_default_pins,
                          brio::DmaTxEngine<3, uint16_t>, brio::DmaRxEngine<2, uint16_t>>;
using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO

extern "C" BRIO_CH32_INTERRUPT void dma1_channel2_handler() {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel3_handler() {
    if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
}
```

A copy and a fill, the completion from the channel's vector:

```cpp
using Copy = brio::DmaCopyEngine<1>;                  // word beats at most
Copy::arm(brio::DmaPriority::high);                  // interrupt on: the line enabled

(void)Copy::copy(frame, tile, 1024);                  // 1024 words, uint32_t* both
// ... later, once the vector said so:
static const uint32_t black = 0;                      // the cell the controller reads
(void)Copy::fill(frame, &black, 1024);

extern "C" BRIO_CH32_INTERRUPT void dma1_channel1_handler() {
    if (Copy::isr() != 0u) { /* the block ended (busy() is false) */ }
}
```

Armed with `interrupt` false, the program polls `busy()` instead.

A block paced by a timer: a transmit engine bound to a register - here
a word in RAM - on the channel table 8-2 gives TIM1's update, the timer
asking for an item at every update:

```cpp
using Paced = brio::DmaTxEngine<5, uint32_t>;        // TIM1_UP is channel 5
using T = brio::Tim<1>;

Paced::arm(&target);
(void)Paced::start(std::span<const uint32_t>(table, 256));
T::init();
(void)T::configure({.prescaler = 0, .period = 4799}); // 10 kHz at 48 MHz
T::interrupts(T::update_dma, true);
T::enable(true);
// dma1_channel5_handler: Paced::service(), then Paced::complete() and T::enable(false)
```

A memory-to-memory block on an idle channel with the one-shot verbs,
polled:

```cpp
using Copier = brio::DmaChannel<1>;   // the ADC's channel, idle here

Copier::stop();                       // EN off, flags clear - it stays set after a block
(void)Copier::load(brio::DmaTransfer{
    .peripheral = src, .memory = dst, .count = 64,
    .config = {.memory_to_memory = true, .peripheral_increment = true,
               .peripheral_width = brio::DmaWidth::word, .memory_width = brio::DmaWidth::word}});
while (!Copier::flag(brio::DmaFlag::complete)) { }
```

## Bench findings

The reference suite is `test_ch32_dma` on the CH32V006K8U6 and on the
CH32V003F4P6, both at 48 MHz, ITS OWN CONSOLE ON THE TWO ENGINES. Its
letters a to d drive the channel's one-shot verbs:

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
  first store rests on), and **no hole raises TEIF**: reads from
  0x3000 0000, 0x4003 0000, 0x5000 0000, 0x6000 0000 and 0xF000 0000
  each completed as a four-byte block with zeros, EN still set. The
  error path is armed on every engine and reachable by nothing the
  bench found.

## Not covered yet

Driver gaps, each with its reason:

- The loop and ping-pong engines a block stream is served by
  (util/block_stream.hpp's two concepts over a circular channel and a
  pair of halves): born with their first block user on this family,
  a source the ADC's stall makes worth measuring first (adc.md).
- Half-transfer as a verb of the transfer engines: the flag and its
  enable are the channel's (`DmaFlag::half`, `arm()`), and the receive
  engine's circular shape takes it on request (`arm_ring()`'s
  `half_mark`, a byte transport's edge); no other engine acts on it
  until a stream needs the midpoint.

Implemented but not bench-verified, each with what would measure it:

- **The engines in their two moments** - the binding at `arm()`, five
  stores a block, the claim under the transport's mask, INTFR read
  once a handler - on every path that names one: `test_ch32_dma`
  letters e and f (the console's own two engines), `test_ch32_serial`
  letter i (USART2's), `test_ch32_spi` letter d on its jumper and
  `test_ch32_i2c` letter g against the peer, on both parts. The
  boards are off the desk; every image builds for both.
- **The beat per block**: the SPI host's 16-bit frames in half-word
  beats, byte-exact on the jumper with no SPI interrupt, and an odd
  buffer's 16-bit request on the pump (`test_ch32_spi` letter d).
- **One DMA interrupt a transaction**: the SPI host's transmit channel
  armed for its errors alone (`test_ch32_spi` letter d counts the
  vector's entries), and the I2C host's write ending on BTF with the
  transmit channel's count at zero and no interrupt from it
  (`test_ch32_i2c` letter g counts both vectors).
- **`DmaCopyEngine`**: copy and fill at the three beats, polled and by
  its interrupt, its refusals and `abandon()` (`test_ch32_dma` letter
  g).
- **The costs in time**: `bench_ch32`'s letter d - copy, fill and
  copy from flash at three sizes with the controller's cycles an item
  and the fixed cost a block, a block of 256 words paced by TIM1's
  update at 10 kHz with its interrupts, busy and the pace's jitter -
  and its letter s, the SPI host's engined write of 16 and 256 frames of 8 and 16
  bits at HCLK/4 and HCLK/16 with MISO floating, the fixed cost a
  transaction against the listing's count above (about 350 cycles at
  the core's two and a half a straight-line instruction, the launch
  and the one vector).
- **The transfer-error path** (TEIE, the engines' `abandon()` and the
  fault counters): written from 8.2.1 and provoked by no address the
  suite could find; a peripheral that raises it - or a write into
  flash, which the chapter lists as a legal destination and the bench
  has not tried - would measure it.
- Widening and truncation between unequal widths (8.2.2's table 8-1):
  the one-shot channel takes two widths and the engines always set
  them equal; a copy with PSIZE and MSIZE apart, compared against the
  table.
- The priority arbitration between two channels running at once: the
  suite runs one channel at a time; two memory-to-memory blocks
  started together, their order read off the flags.
