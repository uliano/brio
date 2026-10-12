# DMA (CH32X035)

The DMA controller of RM ch. 9 - eight channels, each the OR of a fixed
handful of peripheral requests, an arbiter over four software priorities,
three flags a channel - the engines the transports name in their slots
(the USART's and the I2C host's), and the copy engine. Documents of record: the
CH32X035 reference manual V1.8 (9.1 for what the controller has, 9.2.1 for
the arbitration, a channel's three steps and its three outcomes, 9.2.2 for
the widths, 9.2.3's figure 9-1 and the table under it for the request map,
9.3 for the registers; 2.3.2 for the Sleep; table 7-1 for the vectors;
3.4.5 and 3.4.9 for the gate and the reset lines). Driver:
[brio/ch32x035/dma.hpp](../../brio/ch32x035/dma.hpp) and
[brio/ch32x035/dma_engine.hpp](../../brio/ch32x035/dma_engine.hpp); the
guard in [brio/ch32x035/platform.hpp](../../brio/ch32x035/platform.hpp);
reference suite `test_x035_dma`, the transport's engines across a wire in
`test_x035_usart`.

## What the silicon does

- **The channel IS the request.** There is no multiplexer: figure 9-1 ORs
  a fixed handful of peripheral events into each channel - 1 the ADC,
  USART4's transmit and TIM2's channel 3; 2 the SPI's receive, USART3's
  transmit, TIM1's channel 1 and TIM2's update; 3 the SPI's transmit,
  USART3's receive and TIM1's channel 2; 4 USART1's transmit and TIM1's
  channel 4, trigger and commutation; 5 USART1's receive, TIM1's update
  and TIM2's channel 1; 6 USART2's receive, the I2C's transmit and TIM1's
  channel 3; 7 USART2's transmit, the I2C's receive and TIM2's channels 2
  and 4; 8 USART4's receive and TIM2's trigger and commutation. TIM3
  raises no request. The table is `dma_engine.hpp`'s, per part: a USART
  the part does not offer has no channel there. Every TIM1 and TIM2 row
  is measured - each compare and update moving one item on its channel,
  and an edge list on channel 3 taking a value every 10 HCLK cycles
  ([tim.md](tim.md)). The USB host/device
  controller and the USB PD controller carry DMA engines of their own
  (9.1) and touch none of the eight.
- **A channel is the STM32F1's**: CFGR (direction, circular,
  memory-to-memory, the two increments, the two widths, the priority,
  three interrupt enables, EN), CNTR counting down after each item's store
  (9.2.1's three steps), PADDR and MADDR - where DIR says which side is the
  source and in memory-to-memory mode both are memory. 9.2.1's note makes
  PADDR, MADDR, CNTR and CFGR's DIR, CIRC, PINC and MINC writable only
  while EN is clear, and a store into an enabled channel is dropped in
  silence.
- **EN STAYS SET after a completed block** (measured): CNTR at zero, TCIF,
  HTIF and GIF up, the channel enabled and idle; only software clears EN,
  and until it does every configuring store is dropped - PADDR's among
  them (measured, below).
- **Circular mode is not for memory-to-memory** (9.2.1): the one
  configuration the chapter forbids.
- **Eight vectors, not consecutive**: channels 1..7 are table 7-1's
  entries 22..28 and channel 8 is entry 44 (each measured, below). **One
  flag register for all eight**: GIF, TCIF, HTIF, TEIF at 4 x (channel -
  1) in INTFR, the whole word, read-only, cleared by writing one to the
  same position in INTFCR. Each enable in CFGR sits at its flag's
  position.
- **The address's low bits are IGNORED for a wide access** (9.3.5, 9.3.6):
  with a 16-bit width the module drops bit 0, with a 32-bit one bits 1:0 -
  a half-word block pointed at an odd address moves the even address's
  half-words and completes (measured).
- **A read from a hole in the map completes as a normal block** (measured,
  five holes): 9.2.1 promises TEIF and an automatic EN drop for a reserved
  address, and no address the bench could name produced either.
- **Six HCLK cycles an item at every width** from SRAM, eleven and a half
  a word from the flash (two wait states at 48 MHz): the controller is a
  master of the bus matrix at HCLK, and an item is a read, a write and the
  arbitration (measured).
- **IN SLEEP THE CONTROLLER MOVES NOTHING** (measured): RM 2.3.2 says the
  peripherals run in the Sleep, and they do - USART3's transmitter kept
  raising its request - but no item moves until the core wakes. The
  CH32V203's and the CH32V303's answer ([../ch32vx03/dma.md](../ch32vx03/dma.md)).
- **A raised request is HELD** (measured, "A released requester" below):
  the CH32V203's answer again, and the release contract's case.
- **The block's gate, DMA1EN in RCC_AHBPCENR, is closed at reset**
  (measured; the register's reset value opens the USB blocks and SRAMEN),
  and the block has no reset line (3.4.9): a channel is put back by
  zeroing its CFGR.

### The offer, item by item

What the chapter gives that bears on what a block costs, and what the
engines do with each:

| item | used | how, or why not |
|---|---|---|
| the register file (CFGR, CNTR, PADDR, MADDR) | yes | PADDR and the configuration word at `arm()`, once; a block is FIVE STORES - CFGR with EN clear, CNTR, MADDR, INTFCR, CFGR with EN - and no load; the copy engine's SIX, PADDR the second. The word is kept in the engine, because the enable store needs it and a read-modify-write would read it back from the bus |
| the write protection of 9.2.1, and EN standing after a block | yes | the block's first store is the word with EN clear: the disable the note asks for, which EN standing from the last block makes necessary, and the store that changes the beat or the increment in the same access - and every address store comes after it (PADDR stored first was dropped, measured) |
| the two width fields | yes | the BEAT: PSIZE and MSIZE both follow the element of the run a block is handed (8, 16 or 32 bits); unequal widths (table 9-1's packing and truncation) declined - no transport's register and buffer differ in width |
| the address alignment of 9.3.5/9.3.6 | yes | a run off its beat's boundary is REFUSED, its caller falling back to whatever moves it otherwise |
| the increments | yes | MINC per block (a run, or one fixed cell a bus write clocks out or a discard cell a bus read fills); PINC for the copy |
| memory to memory | yes | `DmaCopyEngine`: no request, the channel runs from its enable |
| circular mode | the receive engine's circular shape (`arm_ring()`) | a byte transport's receive ring: the channel writes the whole storage lap after lap, its count the producer index of util/ring.hpp's `HardwareRing`, the completion a lap counted by `lap()`; a transmit ring declined - a circular channel cannot be stopped at the producer's head; a circular player waits for its first block-stream user |
| the half-transfer flag | the receive ring's | the ring's half mark: the edge twice a lap of a stream with no silence; no other engine acts on a midpoint |
| the completion interrupt | per binding | `arm()` names the flags whose interrupt the binding wants |
| one flag register for eight channels | yes | a transport serving two channels in one handler body reads INTFR ONCE |
| the priorities | yes | an argument of `arm()`, and THE LEVEL IS THE PERIPHERAL'S ([../design/dma.md](../design/dma.md)): by default the receive engine at very_high, the transmit engine at high, the copy at low; the USART's ring at very_high (`UartOptions::rx_priority` the knob) and its transmit at high; the I2C host's two at high (it stretches SCL behind a starved channel and loses nothing, [i2c.md](i2c.md)); strict - the level, then the lower number (9.2.1, measured) |
| the software trigger | yes | MEM2MEM is the only one: there is no trigger register beside it |
| the gate | yes | opened once, by `arm()` |
| the transfer error | yes | armed on every engine; it clears EN by itself (9.2.1) and an engine's `abandon()` halts the channel and counts the fault |
| the Sleep | declined | the controller moves nothing in it: the platform's `idle()` does not sleep while a channel works (below) |

The controller has no FIFO, no burst, no double buffer and no chaining of
one channel to another: there is nothing more to use or decline.

## Types and verbs

[brio/ch32x035/dma.hpp](../../brio/ch32x035/dma.hpp) carries the
CH32V006's surface on this controller - the same types, verbs and
defaults - with eight channels:

- `Dma` is the block: `open()` the gate, `is_open()`, `flags()` the whole
  INTFR, and `any_working()` - a channel enabled and circular or with items
  left, the platform's question before it sleeps.
- `DmaChannel<ch>` is one channel, 1..8, with its whole register surface
  as ONE-SHOT verbs for a suite or a rare user: `configure(DmaChannelConfig)`,
  `set_count()`, `set_peripheral()` / `set_memory()`, `prepare()` and
  `load()` over a `DmaTransfer` (both ends, a count, a config - both ends'
  alignment checked against their widths), `enable()`, `count()` live,
  the flags (`flags()`, `flag(mask)`, `clear(mask)` with `DmaFlag`'s four
  bits), `arm(mask, on)` for the three interrupt enables, `isr()` - the
  ISR body that reads only the ARMED flags that are up, clears exactly
  those with GIF and hands them back -, `progress()` and `stop()`. Every
  configuring verb REFUSES while the channel is enabled and for the one
  config the chapter forbids, and opens the gate first.
- `DmaTxEngine<ch, Elem>` and `DmaRxEngine<ch, Elem>` are the TRANSFER
  ENGINES, in the TWO MOMENTS ([../design/dma.md](../design/dma.md)):
  `arm(data, interrupts, priority)` binds the channel to its peripheral
  register - the gate, the channel stopped, PADDR, the configuration word
  kept, the PFIC line - and a block writes its five stores. The beat is
  the run's element, `Elem` the widest the binding allows. The transmit
  engine is BUSY from its `claim()` to `complete()`; `launch(run)`
  programs a claimed engine outside the caller's mask; `start(run)` is
  the two together; `start_fixed(cell, n)` sends `n` copies of one
  element. The receive engine's `start(run)`, `start_discard(cell, n)`,
  `take()`, `full()`, `capacity()`, `idle()`; its CIRCULAR SHAPE,
  `arm_ring(data, storage, half_mark, priority)`, binds a caller's whole
  ring storage and starts it for good, and makes the engine util/ring.hpp's
  `RingCounter`: `remaining()` is CNTR, `laps()` the completions `lap()`
  counted in the channel's handler. Both publish `present`, `channel`,
  `width`, `element`, the flag names, `service()` (with an overload over
  an INTFR read once for two channels) and `block_flags()`; `abandon()`,
  `faults()`, `stop()`.
- `DmaCopyEngine<ch, Elem>` is memory to memory on any channel:
  `arm(priority, interrupt)` once, low unless named; `copy(dst, src, n)`
  and `fill(dst, cell, n)` - the cell IN MEMORY, the caller's -, `busy()`
  (polled, it frees the engine at the completion or the error it finds),
  `abandon()`, `isr()`, `faults()`.
- [brio/ch32x035/dma_engine.hpp](../../brio/ch32x035/dma_engine.hpp) is
  what a transport may know without the controller: the `NoDmaEngine`
  tag, `DmaPriority`, `dma_engine_channel()`, `dma_engines_distinct()`,
  and THE REQUEST TABLE - `DmaRequest`, `dma_request_channel()` per part,
  `DmaRequestOf<r>` refusing a peripheral the part has not got, and the
  USARTs' channels read out of it, which the `Uart` checks its engines
  against at compile time.

What a block costs, counted in the release listings of `test_x035_usart`
(USART4 on channels 1 and 8) beside the WCH examples' restart of a
channel - DMA_Cmd's read-modify-writes of CFGR around
DMA_SetCurrDataCounter and a store of MADDR, seven bus accesses through
three calls in the USART_Idle_Recv example's handlers, which also copy
each half into a ring of their own:

| path | instructions | accesses to the controller | interrupts |
|---|---|---|---|
| the USART's block start (`pump_tx`, a run in the ring) | 58 over three bodies (`pump_tx`, the ring's `read_span`, the engine's `launch`) | 5 stores, no load (and one USART store: TC written 0) | - |
| the USART's mask around it | 6 between the `csrrci` and the `csrsi`: the claim, a byte in RAM | none | - |
| the USART's completion vector (a block done, the next started) | about 90, the ring's `consume` and the block start inside; 372 cycles measured between the body's first and last statement at 48 MHz | one INTFR load, the INTFCR store | one a block |
| the USART's receive ring | none a byte: the vector's two states and the ring's two marks | none a byte | two a burst, two a lap |
| a memory-to-memory block (`DmaCopyEngine::copy()`) | the six stores and the busy flag | 6 stores | one a block, or none polled |

No engine opens the gate, reads CFGR or validates its binding on the
way: those are `arm()`'s, once.

## How to use it

The console on both engines - USART2 transmits on channel 7 and receives
on 6:

```cpp
using Serial = brio::Uart<2, P, 64, 128, brio::UartFormat{}, brio::DmaTxEngine<7>, brio::DmaRxEngine<6>>;

extern "C" BRIO_CH32_INTERRUPT void usart2_handler() {
    if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() {
    if (Serial::dma_isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() { (void)Serial::dma_isr(); }
```

The channel named by its request rather than its number:

```cpp
using Rx = brio::DmaRequestOf<brio::DmaRequest::usart4_rx>;   // channel 8
using Link = brio::Uart<4, P, 512, 256, brio::UartFormat{}, brio::DmaTxEngine<1>,
                        brio::DmaRxEngine<Rx::channel>>;
extern "C" BRIO_CH32_INTERRUPT void dma1_channel8_handler() { (void)Link::dma_isr(); }   // entry 44
```

A copy and a fill, the completion from the channel's vector:

```cpp
using Copy = brio::DmaCopyEngine<5>;                  // word beats at most
Copy::arm();                                          // low, interrupt on

(void)Copy::copy(frame, tile, 1024);                  // 1024 words
static uint32_t black = 0;                            // the cell the controller reads, in SRAM
(void)Copy::fill(frame, &black, 1024);

extern "C" BRIO_CH32_INTERRUPT void dma1_channel5_handler() {
    if (Copy::isr() != 0u) { /* the block ended (busy() is false) */ }
}
```

A block paced by a peripheral's request: the transmit engine bound to
USART3's data register, its transmitter asking for a byte a frame:

```cpp
using Paced = brio::DmaTxEngine<2>;                   // USART3's transmit request
Paced::arm(&brio::Usart<3>::regs().DATAR);
(void)Paced::start(std::span<const uint8_t>(table, 256));
brio::Usart<3>::dma_transmit(true);
```

A program with an engine at work - a receive ring above all - does not
sleep: `idle()` returns at once while `Dma::any_working()` answers true
([platform.md](platform.md)).

## Bench findings

The reference suite is `test_x035_dma` on the CH32X035F8U6 at 48 MHz, ITS
OWN CONSOLE ON THE TWO ENGINES (USART2 on channels 7 and 6), 52 verdicts
in `z`; the paced loads are USART3's transmitter, whose column is handed
no pad. The transport's engines across a wire are `test_x035_usart`'s
([usart.md](usart.md)).

- **Memory to memory, three widths, exact** (letter `b`): 64 bytes, 32
  half-words and 16 words copied, CNTR down to zero, TCIF and HTIF with
  GIF, the write-one clear taking every flag down, EN STANDING after the
  block; a fixed source filling a run with one word; a half-word block
  programmed on an odd address moving the even address's half-words.
- **Six HCLK cycles an item at every width** (letter `b`, every interrupt
  held off): 64 bytes from the enable to TCIF in 408 to 420 cycles as
  bytes, 216 to 224 as half-words, 120 to 128 as words, over three runs -
  six cycles an item, the width the rate.
- **The eight vectors** (letter `c`): a block on each of channels 1..5 and
  8 ended in its own handler, the eighth at entry 44; 6 and 7 are the
  console's engines'.
- **No hole raises TEIF** (letter `d`): reads from 0x3000 0000 (which
  answered a byte that changed from run to run), 0x4003 0000, 0x5000 0000,
  0x6000 0000 and 0xA000 0000 each completed as a four-byte block, EN
  still set.
- **The console's transmit engine at the rate asked** (letter `e`): 272
  bytes in six blocks with no fault, 1001 thousandths of their wire time
  at 115200.
- **`DmaCopyEngine`** (letter `g`): copy and fill at the three beats,
  polled and by its own interrupt, exact; the refusals; `abandon()`
  stopping a 4096-byte fill with 4090 to 4092 left in CNTR. **A PADDR STORED WITH
  EN STANDING IS DROPPED**: written before the restart's disable - the
  order the CH32V006's copy engine uses - a copy after a completed block
  read the PREVIOUS block's source, and the letter's half-word and word
  copies and its fills came out wrong; the engine stores PADDR after the
  disable (`DmaEngineChannel::restart_from()`), and the letter is green.
- **Circular mode** (letter `h`): a block of four bytes from the flash
  into USART3's DATAR, paced by its transmitter at 1 Mbaud, ran twelve
  laps with a completion and a half mark each and no CPU in the path,
  CNTR reloading, EN never dropping.
- **The arbiter is strict** (letter `j`, two blocks of 512 words, the
  counts read after four quiet windows of the core's own loop): channel 1
  at low, first, moved nothing more once channel 8 at very_high came -
  the level decides; at one level channel 8, first, moved nothing once
  channel 1 came, mid-block - the lower number decides; and channel 1,
  first at one level, kept the bus until its block ended. A CPU POLLING
  INTFR in a tight loop let the outranked channel through four to eight
  items in 512 - the verdicts read the quiet windows.
- **IN SLEEP THE CONTROLLER MOVES NOTHING** (letter `k`, the platform's
  WFE sequence entered without its guard): a 4096-byte copy started just
  after a tick had 3862 to 4037 items left when the next interrupt woke
  the core 17 000 to 47 000 cycles later, and 256 bytes paced by USART3 at
  1 Mbaud took 49 to 50 times their pace, moving only while the core was
  awake. THROUGH THE PLATFORM'S `idle()`, which does not sleep over a
  working channel, the same copy ended by its own interrupt in 25 256 to
  25 656 cycles - the controller's rate - and the paced block in 998 to
  1000 thousandths of its pace; and a receive ring, a channel that is
  always working, holds the program awake: with the console's ring
  running, twenty ticks took 2766 to 3189 turns of `idle()`.
- **THE BENCHMARK ROW** (letter `m`, [../design/benchmark.md](../design/benchmark.md)'s
  grammar, every line to the completion interrupt, the best of eight):
  copy 4096 bytes in 6721 to 6729 cycles, x = 6.56 to 6.57 against one word a cycle - 6.0
  cycles an item; fill from a cell in SRAM 6632 to 6640, x = 6.47 to 6.48 - 6.0; copy from
  the flash 12 749, x = 12.45 - 11.6 cycles an item; the fixed cost a block
  about 890 cycles with the instrument inside and the core not sleeping
  (the console's ring); 1024 bytes paced by USART3 at 1 Mbaud at x = 0.99
  to 1.00.

### A released requester

THE RELEASE CONTRACT ([../design/dma.md](../design/dma.md)), measured on
this die (letter `i`): USART3 opened with DMAT and its transmitter on, its
TXE request standing on channel 2 with no channel enabled to take it, then
let go one way, then channel 2 bound by its next owner - a four-byte block
into a RAM cell, request-paced - and its count read 300 us later:

| let go by | the next owner moved |
|---|---|
| nothing ever raised (the control) | 0 |
| DMAT cleared, the block clocked and enabled | 1 |
| UE, TE and DMAT cleared, the clock on | 1 |
| the gate closed with DMAT set | 1 |
| THE RESET LINE, then the gate | 0 |

A raised request outlives every release short of the reset line, and the
next owner of the channel takes one stray item at its first enable - the
CH32V203's answer. So the `Uart`'s `release()` pulses the block's reset
before it closes the gate ([usart.md](usart.md)), the I2C host's and
client's do the same ([i2c.md](i2c.md)), and the suite's own pacer too.

## Not covered yet

Driver gaps, each with its reason:

- The loop and ping-pong engines a block stream is served by
  (util/block_stream.hpp's two concepts over a circular channel and a pair
  of halves): born with their first block user on this series, the ADC,
  which has no driver in this stratum.
- The SPI's engine slots and the timers' requests: born with those
  drivers, each against its row of the request table (the SPI on 2 and
  3). The I2C's are on 6 and 7 ([i2c.md](i2c.md)) - USART2's channels, so
  a program with an engined USART2 and an engined I2C chooses.
- Half-transfer as a verb of the transfer engines: the flag and its enable
  are the channel's (`DmaFlag::half`, `arm()`), and the receive ring takes
  it on request; no other engine acts on it until a stream needs the
  midpoint.
- The completion's restart from SRAM: the transmit engine's restart costs
  372 cycles of body at 48 MHz, about 90 instructions fetched from the
  flash behind two wait states ([usart.md](usart.md)); a body placed in
  SRAM would take most of that off, the linker script and the hot path's
  placement being the change.

Implemented but not bench-verified, each with what would measure it:

- **The transfer-error path** (TEIE, the engines' `abandon()` and the
  fault counters): written from 9.2.1 and provoked by no address the suite
  could find; a peripheral that raises it, or a write into the flash,
  would measure it.
- Widening and truncation between unequal widths (table 9-1): the one-shot
  channel takes two widths and the engines always set them equal; a copy
  with PSIZE and MSIZE apart, compared against the table.
- The six parts other than the CH32X035F8U6: the request table's rows by
  part are compiled for all seven (`brio check ch32x035`); a board of each
  package would measure the USART1 rows the QFN20 does not offer.
