# DMA (CH32V203, CH32V303)

The STM32F1's DMA under WCH's register names, in two sizes. The CH32V203
has ONE controller of EIGHT channels, one more than that ancestor; the
CH32V303 has TWO - a DMA1 of seven channels and a DMA2 of eleven, whose
last four report in a flag register of their own. Every channel is wired
to a fixed handful of peripheral requests, an arbiter over four software
priorities decides between the channels of one controller with the
channel index breaking ties, and every channel has four flag bits.
Documents of record: the CH32F/V20x_V30x_V31x reference manual V2.3
(11.1 for what a channel may reach, 11.2.1 for the channel and its three
outcomes, 11.2.2 and table 11-1 for the widths and their alignment,
11.2.3 for the request maps - table 11-5 for the CH32V20x_D6 class and
11-6 for the D8, and for the CH32V30x_D8 table 11-2 for DMA1 with tables
11-3 and 11-4 for DMA2, and the three notes on DMA1's 64 KB boundary -
11.3 for the registers with table 11-8 for DMA2's, 3.4.6 for the two
gates and 3.4.11 for the reset register that has no bit for either
block, 9.5.1 and table 9-2 for the vectors), the CH32V203 datasheet V2.8
and the CH32V303 datasheet V3.5 (tables 2-1 and 2-1-1 for which
peripherals a part offers, and therefore which requests it can raise).
Three device classes read the chapter: the CH32V20x_D6 for every
CH32V203 up to the CH32V203C8, the CH32V20x_D8 for the CH32V203RB, and
the CH32V30x_D8 for the four CH32V303 parts. Driver:
[brio/ch32vx03/dma.hpp](../../brio/ch32vx03/dma.hpp), with the request
table, the slot a request lands in and the empty slot's tag in
[brio/ch32vx03/dma_engine.hpp](../../brio/ch32vx03/dma_engine.hpp).
Reference suite: `test_vx03_dma`; the engines' costs are letter d of the
bench app `bench_vx03` ([../design/benchmark.md](../design/benchmark.md)).
The receive engine's circular shape is the producer of
[util/ring.hpp](../../brio/util/ring.hpp)'s `HardwareRing`
([../design/ring.md](../design/ring.md)).

## What the silicon does

### The channel is the request

There is no request multiplexer. The tables wire each peripheral event to
ONE channel of ONE controller, and an engine is named with that table in
hand. On the CH32V203 (table 11-5): the converter on 1, SPI1 receiving
on 2 and transmitting on 3, SPI2 on 4 and 5, USART1 transmitting on 4 and
receiving on 5, USART2 the other way round on 6 and 7, USART3 on 2 and 3,
UART4 on 1 and 8, I2C1 on 6 and 7, I2C2 on 4 and 5, and the timers'
updates, captures, triggers and commutations spread across all eight;
the CH32V20x_D8 adds TIM5's six rows (table 11-6) and changes nothing
else.

On the CH32V303 DMA1 carries the same rows on seven channels (table
11-2) - all but UART4's, whose two move to the second controller - and
DMA2 carries everything the bigger part adds (tables 11-3 and 11-4):
TIM5 on channels 1, 2, 4 and 5, TIM6's update on 3 and TIM7's on 4,
TIM8 on 1, 2, 3 and 5, TIM9 and TIM10 on 6..11, UART4 transmitting on 5
and receiving on 3, UART5 on 4 and 2, UART6 on 6 and 7, UART7 on 8 and 9,
UART8 on 10 and 11, SPI3 receiving on 1 and transmitting on 2, SDIO on
4, DAC1 on 3, DAC2 on 4 - and ADC2 on 5, a row whose note gives it to
some lots only, and which the CH32V303VCT6 measured has not got
([adc.md](adc.md)).

A REQUEST LANDS IN A SLOT, NOT ON A CHANNEL NUMBER. With two controllers
a channel number alone names two different channels - UART4 transmits on
DMA2's fifth, which is not DMA1's fifth - so the table answers a
`DmaSlot`, the controller AND the channel, and every check a transport
makes compares the whole slot. Two answers would invite a check that
asks only the second, and a DMA2 engine would then pass for a DMA1
request that happens to share its number: a wedge with no error flag.

WHICH ROWS EXIST IS A PART FACT, not a family one. A count is not a
list: the smallest CH32V203 package offers one usart and it is USART2,
one SPI and no I2C at all, and the 128 KB CH32V303 has no TIM5 and no
TIM8. The driver folds that through the part table and answers the empty
slot - "no channel" - for such a row.

A timer publishes several events and they do not share a channel: an
update is not a capture. TIM1's fourth channel, its trigger and its
commutation ARE one row, because they raise the same request, and so do
the matching events of TIM5, TIM8, TIM9 and TIM10.

A CHANNEL SERVES EVERY REQUEST WIRED TO IT, not the one a program meant.
The rows of a channel are an OR of the peripherals' requests - 11.2.3's
"each channel corresponds to multiple peripheral requests", switched by
each peripheral's own DMA bit - so a peripheral left with its DMA bit set
drives any transfer loaded on its channel. Measured on the CH32V303VCT6:
a UART4 receiver left running with DMAR set held its request standing on
DMA2's channel 3, and a transfer programmed there for TIM6's update moved
all sixteen of its items in a few microseconds instead of one a period.
Stopping a channel does not withdraw a request; turning the peripheral
off does, which is what a transport's `release()` is for.

### A channel

CFGR (direction, circular, memory-to-memory, the two increments, the two
widths, the priority, three interrupt enables and EN), CNTR counting down
as items move, and PADDR and MADDR - where DIR says which side is the
source, and in memory-to-memory mode both are memory and the names mean
nothing. Every one of those fields is read-only while EN is set (11.2.1's
note and each register's own), and a store into an enabled channel is
dropped in silence. The same four words on both controllers: DMA2's
first seven channels sit at the same offsets as DMA1's, and its channels
8 to 11 in a block of their own at offset 0x90, four words apart with no
reserved word between them (table 11-8).

- **The count is in ITEMS, up to 65535** (11.1, 11.3.4), and a count of
  zero moves nothing whether the channel is enabled or not.
- **The widths are 8, 16 and 32 bits** and the fourth code is Reserved
  (11.3.3). Source and destination widths are independent, and table
  11-1 is the padding and truncation between them.
- **THE ADDRESSES ARE ALIGNED BY THE SILICON, SILENTLY**: 11.3.5 and
  11.3.6 say the module ignores bit 0 of a 16-bit access and bits [1:0]
  of a 32-bit one. A half-word transfer pointed at an odd address
  therefore moves the wrong bytes and reports success.
- **Circular mode is not for memory-to-memory** (11.2.1): the one
  configuration the chapter forbids. Memory-to-memory has no request to
  reload against - it runs on the enable.
- **In circular mode the count reloads itself and so do both addresses**
  (11.2.1's cycle mode, 11.3.4): when CNTR reaches zero it is loaded
  with its initial value again and the channel goes on, with no CPU in
  the path. A transfer is the read, the store and THEN the decrement
  (11.2.1's three steps), so the count never counts an item whose store
  has not been made - measured below on a ring read behind the channel
  at three million items a second, no item ever handed out ahead of its
  store. A read can land between the last decrement of a lap and the
  reload and see zero, and the length is seen after the wrap (the
  findings below).
- **Flash is a legal source** (11.1 lists flash, SRAM, the peripheral
  SRAM and all three peripheral buses), which is what lets a program
  copy out of its own image with no RAM cost - on either controller.

### The 64 KB rule of the CH32V303's DMA1

11.2.3's first note bars DMA1 of the CH32V30x_D8 from crossing a
64-kilobyte boundary, at either end of a transfer, on lots whose sixth
digit from the end is zero; its second lifts the rule on every other
lot, and its third says DMA2 has none. A program cannot read its lot, so
the rule is kept on EVERY lot of the class: a transfer whose first and
last access of either end fall in different 64 KB pages is refused on
DMA1 of the CH32V303, nothing written, and taken on DMA2. The note goes
on to relax channels 2 to 5 to a 128 KB boundary on some lots; the driver
does not, because the strict reading is the one no lot contradicts. No
CH32V203 is named by the note, and no part of that series is held to it.

THE RULE IS REAL, AND ITS FAILURE IS A WRAP. On the CH32V303VCT6 the
refusal bypassed - the channel programmed through the piecewise verbs,
which never see a transfer whole - thirty-two bytes of flash straddling
0x0801 0000 arrived on DMA1 with their upper sixteen read from 0x0800 0000
instead: the address counter wrapped inside the 64 KB page the transfer
started in, a transfer that completed and reported nothing. DMA2 moved
the same span byte for byte. That die is a lot the note restricts.

### What the controller offers, item by item

The engines are written from this chapter, and every item of it that
bears on what a block costs is used or declined here with its reason:

- **The register file, four words a channel, EVERY FIELD READ-ONLY WHILE
  EN IS SET** (11.2.1's note). Used as the shape of the engines: what is
  constant for a binding - PADDR, the direction, the increments, the
  priority, the interrupt enables - is written once when the engine is
  armed and kept as a word in the engine, because the register cannot
  keep it across a block (EN is in the same word); per block the engine
  writes MADDR, CNTR, the channel's four flags in INTFCR and CFGR WHOLE
  with EN - four stores, no read. The vendor's own restart of an SPI
  channel (the EVT's `SPI_LCD`) touches the same four registers.
- **One store sets the fields and EN at once.** The fields are read-only
  while EN is SET, and the store that sets it lands on a channel whose EN
  is clear: measured, a word copy, a byte copy and a half-word fill back
  to back, each starting with a width the last had not, every byte where
  it belongs.
- **The width, 8, 16 or 32 bits a side** (11.3.3): used per block. The
  beat of a block is the element of its run - a 16-bit SPI frame is one
  16-bit access to DATAR - and the engine's `Elem` is the widest beat the
  binding takes.
- **Circular mode**: used by the player (`DmaLoopEngine`) and by the
  receive engine's CIRCULAR SHAPE - a transport's whole receive ring as
  one circular block, the count its producer index and the completion a
  lap counted (below) - and declined by the block source for the
  contract's reason (below).
- **The half-transfer flag**: taken on request by the receive engine's
  circular shape (`half_mark`), for a byte transport whose ring wants the
  lap's half and full marks as the receive edge of a stream that never
  falls silent ([usart.md](usart.md)); declined elsewhere - the block
  source stops at every block, and a ring's consumer reads the count
  whenever it looks, the wrap being the one edge the lap count needs.
- **Chaining, a linked descriptor, a self-trigger, a FIFO, a burst, a
  double buffer**: this controller has none of them; a block is restarted
  by software, and the four stores above are the whole of it.
- **Memory to memory** (MEM2MEM, the block running on the enable with no
  request): used by `DmaCopyEngine`.
- **The trigger sources**: a channel's own peripheral rows (table 11-5 -
  no multiplexer) and nothing else; a timer's update request is the pace
  of a paced block (TIM4's on DMA1's seventh channel, measured below), and
  memory to memory is the software trigger.
- **The gate**, DMA1EN and DMA2EN: opened once, when an engine is armed,
  never on the block path. Nothing in this stratum closes it again.
- **The flags**: write-one clears in INTFCR, one store a block for the
  channel's four; an engine's ISR body reads INTFR once and masks it with
  the engine's own armed word - not a second read of CFGR.
- **The interrupts**: one vector a channel, each engine arming only what
  its owner needs - the SPI host's transmit engine its ERROR alone, the
  receive block's completion proving the transmit's.
- **The priorities**: an argument of each engine's `arm()`.
- **The transfer error**: TEIE armed with every engine; the hardware drops
  EN, and the controller's ledger gives the bus-master count back when
  software stops the channel.

### The flags, the vectors and the gates

- **Four flag bits a channel** in INTFR, read-only - GIF, TCIF, HTIF,
  TEIF at 4 x (channel - 1) - cleared by writing one to the same
  position in INTFCR. DMA2's channels 8 to 11 report in a pair of their
  own, DMA2_EXTEM_INTFR and DMA2_EXTEM_INTFCR at offsets 0xD0 and 0xD4,
  four bits a channel from bit 0 (11.3.11, 11.3.12); INTFR's top nibble
  belongs to no channel on that controller and stays clear while those
  four run (measured).
- **The half flag rises when fewer than half the items are left**
  (11.2.1), measured below.
- **One vector per channel.** On the CH32V203 seven are consecutive from
  the vector table's entry 27 and the eighth sits on the tail its DEVICE
  CLASS has - entry 62 on the CH32V20x_D6, 67 on the CH32V203RB, whose
  table carries the Ethernet pair before it. On the CH32V303 DMA1's seven
  are 27 to 33, and DMA2's eleven are split: channels 1 to 5 at 72 to 76,
  channels 6 to 11 at 98 to 103 (all eighteen measured). Either way a
  channel's handler reads only its own flags and needs no "which
  channel" question.
- **The gates, DMA1EN and DMA2EN in RCC_HBPCENR, are closed at reset**
  (the register's reset value opens the SRAM alone).
- **THESE BLOCKS HAVE NO RESET LINE.** RCC_AHBRSTR (3.4.11) reserves bits
  [11:0] and names only the Ethernet MAC, the DVP and the USB OTG core,
  so the pulse every PB1 and PB2 peripheral of this stratum is put back
  with does not exist here: "back to the reset values" is software's
  work, channel by channel.

### In Sleep the bus matrix serves the core alone

This is the fact the chapter opens with, and it is not in the manual. In
the Sleep of RM 2.4 - the core clock gated, "all peripherals still
running" - no bus master but the core gets a cycle: a memory-to-memory
block started right before the core sleeps moves the handful of items
already in flight and then nothing, while the timers on the peripheral
bus and the core's own counter count the whole sleep. The same starvation
is what kills the USB controller's reach into its packet memory in Sleep
([README.md](README.md)), and on the CH32V303 both of its controllers
starve alike (the findings below).

So on this family A DMA-FED TRANSPORT DOES NOT SLEEP, and that is a
MECHANISM and not a rule the programmer keeps: every verb that sets a
channel's EN takes one count of an active bus master
([sleep.md](sleep.md)) and every verb that clears it gives the count
back, on either controller - the controller's LEDGER, one byte a
channel, saying which channel holds one, because the one thing that
drops EN behind software's back (a transfer error) would otherwise leave
a count nobody gives back. The kernel's idle path does not sleep while
the count stands and a sleep site refuses to arm over it. An engine
STOPS ITS CHANNEL AT THE BLOCK'S END for the same reason: EN stays set
after a completed block, and a channel left enabled holds its count. `any_enabled()` is
the same question asked of the registers instead, of every channel of
every controller the part has, for a caller that wants the silicon's own
answer. The alternative the same measurements opened is to slow down
instead: the chip carries data with HCLK as low as 24 MHz.

## Types and verbs

### The request table

[brio/ch32vx03/dma_engine.hpp](../../brio/ch32vx03/dma_engine.hpp) holds
what a TRANSPORT needs without including the controller: `NoDmaEngine`,
the empty slot's tag whose only member is `present`; `DmaSlot`, a
controller and a channel (the empty slot both zero); `dma_engine_slot`
and `dma_engines_distinct`, the two checks a task makes on a named
engine; and the table itself - `DmaRequest`, one enumerator per row of
tables 11-2 to 11-6, `dma_request_channel` answering the slot with the
part folded in (the empty slot where the part has not got the
peripheral), `dma_request_present`, and `DmaRequestOf<r>` with its
`slot`, `controller` and `channel`, the compile-time form that REFUSES a
request this part cannot raise. A driver with an engine slot includes
this file alone, so a program with a console does not carry the
controller.

### The block

`Dma<1|2>`, the second refused on a part with one controller: `open()`
(the gate, which every channel verb calls first), `opened()`,
`channel_count`, `flags()` and `clear()` over INTFR, on DMA2 also
`extended_flags()` and `clear_extended()` over the extended pair (a
compile error on DMA1), `enabled_channels()`, `stop_all()` - every
channel back to the chapter's own values, the work the missing reset line
leaves to software - and `any_enabled()`, the power model's one
question, which asks every controller the part has whichever one it is
spelled on. `dma_channels_of`, `dma_channel_exists` and
`dma_channel_irq` are the same facts as functions of the controller and
the channel, for a program that has them as values.

### The channel

`DmaChannel<c, ch>` - channels 1..8 of the CH32V203's DMA1, 1..7 of the
CH32V303's and 1..11 of its DMA2, anything else refused:
`configure(DmaChannelConfig)` and `configuration()` reading it back,
`set_count()`, `set_peripheral()` / `set_memory()`, `accepts()`,
`prepare()` and `load()` over a `DmaTransfer` (both ends, a count, a
config), `trigger()` (a prepared channel started again), `enable()`,
`remaining()` live, the flags (`flags()`, `flag(mask)`, `clear(mask)`
over `DmaFlag`'s four bits, from INTFR or the extended pair as the
channel's `extended` says), `arm(mask, on)` and `armed()` for the three
interrupt enables, `isr()` - the ISR body that reads only the ARMED
flags that are up, clears exactly those with the global bit and hands
them back, deciding nothing -, `progress()`, `irq()` and `stop()`.
`controller`, `number`, `slot` and `bounded_to_64k` say what the channel
IS.

EVERY CONFIGURING VERB REFUSES while the channel is enabled, every verb
that takes an address refuses one that is not aligned to its own width,
and on the CH32V303's DMA1 `prepare()`, `load()` and `accepts()` refuse
a transfer that crosses a 64 KB boundary: a store the silicon ignores, an
address it rounds down or a span it wraps would be a lie the code told
itself. The piecewise verbs - `configure()`, `set_count()`,
`set_peripheral()` and `set_memory()` - never see a transfer whole and
do not ask the 64 KB rule, which is how the suite hands DMA1 the span
it measures. `dma_channel_config_valid` and `dma_transfer_valid` are the
compile-time half of that, and `dma_transfer_aligned` and
`dma_transfer_crosses_64k` the half only an address can answer.

### The engines

Every engine stands on `DmaBinding<c, ch>`, the channel held: `bind()`
(the gate, the channel stopped, its flags cleared, PADDR - once, at an
engine's `arm()`), `go()` and `rearm()` (a block's four stores, the
second with one store more first for a channel whose EN may still stand),
`halt()` (the block's end: one store, the ledger's count given back) and
`service()` (the ISR body: one load, one store).

`DmaTxEngine<c, ch, Elem>` pours caller-owned runs into one peripheral
register: `arm(data, priority)` and `arm(data, interrupts, priority)` -
the second choosing which flags raise the channel's line, the completion
and the error by default -, `start()` over a span of bytes, of half-words
or (where `Elem` is 32 bits) of words, and over a pointer and a length;
the two halves of `start()`, `claim()` (the busy flag's test-and-set, what
a transport racing its own completion handler masks) and `launch()` (the
programming of a claimed engine, which gives the claim back when it
refuses), `unclaim()`, `start_fixed(cell, n)` for a full-duplex bus
clocking a read, `complete()` (the channel stopped and how many elements
the block carried, so the owner releases exactly that much of its ring),
`kick()`, `abandon()`, `faults()`, `busy()`, `in_flight()`, `progress()`.
`DmaRxEngine<c, ch, Elem>` fills from one register in TWO SHAPES on one
binding. THE ONE-SHOT SHAPE fills a run and stops - a bounded block, the
receive half of a bus transaction: the same two `arm()`s, `start()` over
the mutable spans, `start_discard(cell, n)`, `complete()` (the channel
stopped - EN stays set after a completed block, and a channel left
enabled holds the bus-master count), `take()` - how many arrived since
last asked, one CNTR read, nothing suspended -, `harvest()` - the same
arithmetic without consuming it -, `idle()`, `full()`, `capacity()`,
`taken()`, `kick()`, `abandon()`. THE CIRCULAR SHAPE fills a RING for
ever: `arm(data, storage, priority)` is handed the ring's whole storage -
the caller's array, its element the beat and its length, checked at
compile time, the count every lap reloads - and keeps the address, the
length and the ring's word (CIRC, MINC, the beat, the priority, the lap
and the error armed, never the half), refusing a storage misaligned for
its element or, on the CH32V303's DMA1, one across 64 KB; `start()` with
no run starts the ring from its first element with the lap count at
zero, the binding's five stores; `remaining()` (CNTR, one load) and
`laps()` (what `lap()`, the completion's verb on a ring, counted) make
the engine util/ring.hpp's `RingCounter`, so
`HardwareRing<storage, DmaRxEngine<c, ch, Elem>>` is the ring's
consumer half; and `bounded_to_64k` says whether the channel is held to
the 64 KB rule, so an owner can align the storage to its own size there
and no placement can be refused. Nothing re-arms a ring: the one way it
stops by itself is a transfer error, which `idle()` reports and
`start()` answers. Both engines publish `present`, `controller`,
`channel`, `slot`, `width`, `element`, `service()` and the flag names,
and a transport reaches its engine only through those.

THE BEAT IS THE ELEMENT OF THE RUN. PSIZE and MSIZE follow the span: a
16-bit frame is one 16-bit access to the data register. `Elem` is the
WIDEST beat the binding takes - 16 bits by default, every serial and bus
data register of this family; 32 for a timer's 32-bit compare or a
memory cell - and a wider span is a compile error. A half-word run at an
odd address is refused at run time, the controller rounding it down in
silence otherwise (11.3.6), and so is a run longer than CNTR counts or
one across 64 KB on the CH32V303's DMA1. `arm()` refuses a data register
that is not aligned to `Elem`.

`DmaCopyEngine<c, ch>` is memory to memory: `arm(priority, interrupt)`,
`copy(dst, src, n)` and `fill(dst, cell, n)` - `n` in elements, the
element type the beat, the fill's cell the caller's and in memory,
because the controller reads an ADDRESS every beat -, `busy()` (which,
armed without the interrupt, also finds the completion), `abandon()`,
`service()` (the ISR body that ends the block), `faults()`, `stop()`.

TWO ENGINES OF ONE TRANSPORT NAME TWO CHANNELS: a channel moves data one
way, and the table gives each direction its own.

### The block engines

`DmaLoopEngine<c, ch, Elem>` and `DmaPingPongEngine<c, ch, Elem>` are
this family's realizations of
[util/block_stream.hpp](../../brio/util/block_stream.hpp)'s
`BlockPlayer` and `BlockSource`, and they differ in exactly one thing:
the player RIDES THE CONTROLLER'S CIRCULAR MODE, where CNTR reloads
itself and the lap interrupt does nothing but count, and the source does
NOT. The reason is the contract's and not the API's - "skip rather than
tear" cannot be decided on a channel that never stops - and it is
measured here, on the fastest possible reader: see [adc.md](adc.md),
whose converter is these engines' first user and whose suite carries the
number. On the CH32V303 the player has the peripheral it was written
for: the DAC, whose two requests are DMA2's channels 3 and 4
([dac.md](dac.md)).

### The transport's two slots

[brio/ch32vx03/usart.hpp](../../brio/ch32vx03/usart.hpp)'s `Uart` takes
a transmit engine and a receive engine as two of its template
parameters, `NoDmaEngine` by default: the transmit engine drains the TX
ring by contiguous runs, and the receive engine runs in its circular
shape over the WHOLE receive ring's storage, the transport's receive
ring being the `HardwareRing` over it - a byte that lands is readable at
once, a burst wraps the storage's end with no CPU, and nothing is lost
between runs because there are none. The two bus engines take the same
pair in the same place, in the one-shot shape - `SpiHost` carries a
block's data phase on them ([spi.md](spi.md)) and `I2cHost` a tenure's
([i2c.md](i2c.md)). `dma_isr()` is the ISR body of whichever channels
the transport owns - on the receive channel a completion is a lap,
counted, and the lap's half and full marks are the receive edge, its true
answer the edge `isr()` gives -, `harvest()` the same edge asked from the
consumer's side and the ring's restart after an error
([usart.md](usart.md)), and `dma_faults()` counts the
blocks thrown away. THE MASK COVERS THE CLAIM: the transport's block
start holds its guard over the transmit engine's `claim()` alone - eight
instructions, against the whole channel load before (the listing in the
bench findings) - and programs the claimed channel unmasked, no block
being in flight that could complete under it. An engine is REFUSED on any slot but the instance's
own, controller and channel, which the request table answers; each
resource publishes its two slots as `dma_tx_slot` and `dma_rx_slot`.
Without an engine every branch is compiled out and `init()` does not so
much as touch CTLR3.

The receive edge comes from the vectors - the USART's idle line and a
burst's first frame, the channel's lap marks - and the CPU never reads
DATAR while the channel owns it: what a clear costs on this silicon, and
the bounds it leaves on the error counts, are [usart.md](usart.md)'s.

## How to use it

A copy and a fill on the copy engine, the completion on its interrupt:

```cpp
using Copier = brio::DmaCopyEngine<1, 1>;
Copier::arm();                                   // once: the gate, the mode
BRIO_CH32_VECTOR(dma1_channel1_handler) { (void)Copier::service(); }

alignas(4) uint32_t frame[1024];
static const uint32_t black = 0;
(void)Copier::fill(frame, &black, 1024);         // 1024 word beats
while (Copier::busy()) { }
(void)Copier::copy(frame, saved, 1024);
```

Armed with `Copier::arm(brio::DmaPriority::low, false)` it takes no
interrupt and `busy()` finds the completion itself.

The same block on the raw channel, polled:

```cpp
using Channel = brio::DmaChannel<1, 1>;

Channel::stop();                      // EN stays set after a block: clear it first
(void)Channel::load(brio::DmaTransfer{
    .peripheral = src, .memory = dst, .count = 1024,
    .config = {.memory_to_memory = true, .peripheral_increment = true,
               .peripheral_width = brio::DmaWidth::word,
               .memory_width = brio::DmaWidth::word}});
while (!Channel::flag(brio::DmaFlag::complete)) { }
Channel::stop();
```

A channel's interrupt: `Channel::arm(DmaFlag::complete | DmaFlag::error,
true)`, `Pfic::enable(Channel::irq())`, and a handler on
`dma1_channel1_handler` calling `Channel::isr()` and acting on the bits it
returns. A DMA2 channel is the same with the controller's number first,
`DmaChannel<2, 9>`, and its handler `dma2_channel9_handler`.

A paced block - a table into one cell at a timer's pace, on the transmit
engine of the timer's update slot:

```cpp
using Row = brio::DmaRequestOf<brio::DmaRequest::tim4_up>;
using Paced = brio::DmaTxEngine<Row::controller, Row::channel, uint32_t>;
(void)Paced::arm(&cell, brio::DmaPriority::high);
(void)Paced::start(std::span<const uint32_t>(table, 256));
brio::Tim<4>::interrupts(brio::tim_ude, true);   // the request, then the count
brio::Tim<4>::enable(true);
// dma1_channel7_handler: Paced::service(), and complete() on its flag
```

A peripheral request, named rather than numbered - a timer's update
moving one sample of another timer's counter per period:

```cpp
using Row = brio::DmaRequestOf<brio::DmaRequest::tim2_up>;
using Sampler = brio::DmaChannel<Row::controller, Row::channel>;

(void)Sampler::load(brio::DmaTransfer{
    .peripheral = brio::Tim<3>::cnt_address(), .memory = buffer, .count = 16,
    .config = {.peripheral_width = brio::DmaWidth::half,
               .memory_width = brio::DmaWidth::half}});
brio::Tim<2>::interrupts(brio::tim_ude, true);   // the request enable
brio::Tim<2>::enable(true);
```

The same timer's samples into a RING, read behind the channel for as long
as it runs - the receive engine's circular shape and util/ring.hpp's view
over the same storage:

```cpp
using Row = brio::DmaRequestOf<brio::DmaRequest::tim2_up>;
using Sampler = brio::DmaRxEngine<Row::controller, Row::channel>;
alignas(4) uint16_t samples[256];                // a power of two: the view's mask
using Samples = brio::HardwareRing<samples, Sampler>;

BRIO_CH32_VECTOR(dma1_channel2_handler) {
    const uint8_t f = Sampler::service();
    if ((f & Sampler::flag_error) != 0u) { (void)Sampler::abandon(); }
    else if ((f & Sampler::flag_complete) != 0u) { Sampler::lap(); }
}

(void)Sampler::arm(brio::Tim<3>::cnt_address(), samples, brio::DmaPriority::high);
Samples::clear();
(void)Sampler::start();
brio::Tim<2>::interrupts(brio::tim_ude, true);
brio::Tim<2>::enable(true);
// the consumer, as often as it likes - more than once a lap:
const std::span<const uint16_t> run = Samples::read_span();
/* ... use the run ... */
if (!Samples::consume(run.size())) { /* the channel lapped it while it was held */ }
```

A console on both engines - USART2 transmits on DMA1's channel 7 and
receives on its 6:

```cpp
using Tx = brio::DmaRequestOf<brio::DmaRequest::usart2_tx>;
using Rx = brio::DmaRequestOf<brio::DmaRequest::usart2_rx>;
using Serial = brio::Uart<2, P, 128, 128, brio::UartFormat{},
                          brio::DmaTxEngine<Tx::controller, Tx::channel>,
                          brio::DmaRxEngine<Rx::controller, Rx::channel>>;

// The edge from the vectors: the USART's (an idle line, a burst's first
// frame) and the receive channel's (the lap's half and full marks).
BRIO_CH32_VECTOR(usart2_handler) {
    if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
BRIO_CH32_VECTOR(dma1_channel6_handler) {
    if (Serial::dma_isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
}
BRIO_CH32_VECTOR(dma1_channel7_handler) { (void)Serial::dma_isr(); }
```

The CH32V303's UART4 is the same spelling, and the table puts both of
its engines on DMA2 - the handlers are then `dma2_channel5_handler` and
`dma2_channel3_handler`. With an engine on each side the port arms no
interrupt of its own, and `release()` is what takes its requests away
from the two channels when the port is done with them.

## Bench findings

`test_vx03_dma` measures at 144 MHz, the source of every copy a
four-kilobyte pattern in the image itself and the core's own counter the
ruler. On the CH32V203C8T6 the nine letters of a part with one
controller run with the board bare: **38 verdicts in `z`**. On the
CH32V303VCT6 DMA2's six letters ran beside DMA1's with the evaluation
board's two crossed wires in place (PA2 to PC11, PC10 to PA3), every
verdict passing; the engines as they now are ran there too, **56
verdicts in `z`** with the two wires in place, letter n and the receive
ring of letters g and o among them. Where one number
is given below it is both parts'; where they differ each is named.

- **Six cycles an item at every width, and the width is the rate.** Four
  kilobytes copied flash to RAM take 24809 cycles as 4096 bytes, 12530
  as 2048 half-words and 6392 as 1024 words on the CH32V203C8T6 - 6.05,
  6.11 and 6.24 cycles an item - and 24933, 12649 and 6495 on the
  CH32V303VCT6's DMA1 - 6.08, 6.17 and 6.34. At 144 MHz that is 23.8,
  47.1 and 92.3 megabytes a second, and every byte of all three arrived.
- **The half flag rose with 2049 of 4096 moved** on the CH32V203C8T6 and
  2050 on the CH32V303VCT6, which is 11.2.1's own rule read back; INTFR
  stood at 0x7 (the global, complete and half bits of the channel) and
  one write to INTFCR took all four down.
- **EN stays set after a completed block** - CNTR at zero, TCIF up, the
  channel enabled and idle - and while it is set every configuring verb
  of the driver refuses: the configuration, the count, a whole transfer
  and an address, with CFGR unchanged across all four.
- **NO ADDRESS THE BENCH COULD NAME RAISES A TRANSFER ERROR.** Reads of
  0x3000 0000, 0x4000 8000, 0x4002 4000, 0x6000 0000 and 0xA000 0000 -
  five addresses outside every window of the map - each completed as an
  ORDINARY BLOCK: TCIF up, TEIF down, EN still set, with TEIE armed on
  every one of them. 11.2.1 promises a transfer error and an automatic
  drop of EN for a reserved area, and none of the five produced either.
  The first three read zeros on both parts; on the CH32V303VCT6 the last
  two are the FSMC's bank 1 and its register block, and with the FSMC's
  gate closed each read the word 0x0FFF FFFF. The same is true of the
  CH32V00x's controller.
- **A circular channel reloads its count and its addresses by itself.**
  Eight compares poured into a timer by its own update request, four
  laps: four half flags and four complete ones, the count seen stepping
  down twenty-eight times, the channel still enabled at the end and the
  last value of the run standing in the timer's register. Thirty-two
  updates of 200 us took 6400 us, so the REQUEST and not the bus paces
  the channel.
- **A timer's update request needs no pad and no core**: sixteen samples
  of a free-running one-megahertz counter, taken 500 us apart, came out
  503, 1003, 1503 ... 8003 on the CH32V203C8T6 and 504 ... 8004 on the
  CH32V303VCT6 - a staircase whose step is the period.
- **The burst engine walks four registers per request**: with DBA at
  CH1CVR and DBL at four, a run of four half-words written through the
  single address TIMx_DMAADR landed as 111, 222, 333, 444 in CH1CVR
  through CH4CVR.
- **The software priority outranks the channel number, and with the
  priorities equal the lower number takes the bus outright.** Two
  channels of equal work: the one that started SECOND and asked for the
  very-high priority completed first. With both at medium and the higher
  channel given the head start, channel 1 finished at 3106 cycles and
  channel 2 at 6151 on the CH32V203C8T6, and at 3056 and 6044 on the
  CH32V303VCT6 - channel 1 running at full rate and channel 2 getting
  what was left.
- **THE SLEEP, MEASURED IN BRIO'S OWN CODE.** Four kilobytes that take
  172 us awake moved 4 bytes across one `idle()` and 76 across ten on
  the CH32V203C8T6 - seven a wake at the 1 kHz tick - and 22 and 124 on
  the CH32V303VCT6's DMA1, twelve a wake; then each block finished, byte
  for byte, the moment the core stayed awake. The vendor's own rig put
  the CH32V203's figure at nine to twelve bytes at the entry and eleven a
  wake; this platform's idle is a WFE-shaped `wfi` and its wake handler
  is smaller, which is the difference. Either way the conclusion is the
  same: the clocks run and the bus matrix serves the core alone.
- **AND THE CH32V303's SECOND CONTROLLER STARVES THE SAME WAY**: the same
  four kilobytes on DMA2's first channel took 172 us awake and moved 23
  bytes across one `idle()` and 120 across ten, then finished awake byte
  for byte.
- **All the vectors are the channels' own**: eight on the CH32V203C8T6,
  the eighth on that device class's tail, and on the CH32V303VCT6 DMA1's
  seven and DMA2's eleven - eighteen blocks, eighteen bodies, each
  running exactly once and each seeing its own armed flag and nothing
  else, the four channels of the extended register included.
- **DMA2's eleven channels copy out of flash as DMA1's do.** A kilobyte
  memory to memory on each, byte for byte, in 1731 to 1780 cycles:
  channels 1 to 7 reporting in DMA2_INTFR at 4 x (channel - 1), channels
  8 to 11 in DMA2_EXTEM_INTFR at 4 x (channel - 8), and INTFR's top
  nibble clear while those four ran.
- **DMA2's timer requests**: TIM5's, TIM6's and TIM7's updates each drew
  the same staircase of another timer's counter through DMA2's channels
  2, 3 and 4 - the rows of table 11-3, 503 or 504 ... 8003 or 8004.
- **Both block engines run on DMA2**, each serviced from that
  controller's own handler: the loop engine played eight laps of eight
  compares into TIM5 in 12801 us for 12800 asked, with no fault, and the
  ping-pong engine filled four blocks of eight samples of a counter
  taken 200 us apart (204, 404 ... 1604), with no overrun.
- **The 64 KB rule on the CH32V303VCT6**: DMA1 refused the span across
  0x0801 0000 and took its lower half; DMA2 moved the whole span byte for
  byte; and with the refusal bypassed DMA1 delivered the WRAP described
  above.
- **The transmit engine needs no listener**: twenty-six bytes left the
  ring in one block at 115200 baud, `tx_idle()` true 2177 us after the
  run was queued, with no fault counted, on a pad with nothing attached.
- **THE RECEIVE RING, WITH NO WIRE** (letter g, the CH32V203C8T6). From
  `init()` the receive channel runs circular over the whole 128-byte
  storage, its count at 128 with nothing arrived and one bus master
  counted; stopped by `abandon()` - standing for the transfer error that
  is the one way a ring stops - it was running again from its first byte
  after the consumer's next look, the fault counted. Then bursts banged
  into the receive pad through its pull at 115200 baud, the core doing
  nothing else: 100 bytes into the empty ring read back in order, the
  vectors reporting the edge and `harvest()` asked after it answering
  false; 60 more ACROSS THE STORAGE'S END with no read between their
  bytes, all 60 in order and no overrun of either kind - where the run
  engine this shape replaced, measured with the same letter, delivered
  27 of the 60 and counted one hardware overrun, its run having stopped
  at the storage's end; 200 bytes into the ring with nobody reading,
  ONE overrun counted, nothing of the overwritten lap delivered and the
  ten bytes after it read whole; and a run of 14 held while 140 more
  landed, refused at its release, the second overrun counted. The
  CH32V303VCT6 gave every one of those numbers again, the ring on its
  DMA1 storage aligned to its own size.
- **THE RING AT SPEED** (letter o, the CH32V203C8T6): TIM2's update
  copying TIM3's counter into a ring of 256 half-words, read behind the
  channel through `HardwareRing` with every element judged against the
  one before it. At one update every 288 cycles, 131072 elements across
  512 laps in 230427 looks, every step the pace's, nothing skipped,
  nothing torn - so no count ever ran ahead of its store - and the
  producer's position from `laps()` and the count, 131072, equal to the
  updates the pace made in the consumer's time. At one every 48 cycles,
  three million a second, 131076 elements in 21222 looks, the same. A
  consumer stalling a lap and a half every 4096 elements lost exactly
  that: seven stalls, seven overruns counted and skipped, every step
  between them the pace's; four runs held across a lap and a half were
  each refused at their release. Read in a tight loop across laps the
  count reached 256 again at every wrap and never went above it, and at
  the fast pace a read could land on zero, between a lap's last
  decrement and the reload. On the CH32V303VCT6: 131072 elements in
  270997 looks at the slow pace and 131074 in 27249 at the fast one,
  every step the pace's, nothing skipped or torn, the position 131073
  against 131072 updates; seven stalls and seven overruns; four held
  runs refused; the count never above 256, and zero read once in
  100000 across 98 laps in one run and never in another.
- **TWO CONTROLLERS ON ONE PAIR OF WIRES.** On the CH32V303VCT6, USART2's
  engines on DMA1 against UART4's on DMA2 across the board's crossed
  pair: thirty-six bytes out of USART2's transmit engine into UART4's
  receive engine and twenty-two back the other way, each run byte for
  byte, no fault on either port. Before the runs each receiver's ring
  was drained: zero or one byte, framed while its pad floated between
  the first port's initialization and the second's.
- **A standing request serves the wrong transfer**: the UART4 receiver of
  that letter, left enabled with DMAR set, made a later TIM6 staircase on
  the same DMA2 channel move all sixteen items at once (the section
  above); with the port released at the end of the letter, the staircase
  ran at the timer's pace again.

### The engines' costs

`bench_vx03`'s letter d on the CH32V203C8T6 at 144 MHz, the core's
counter the ruler, every number in HCLK cycles and the instrument's own
cost included as the line carries it (a stopwatch is 28 cycles, a stamp
pair in a vector 79 - the app's letter r). BEFORE is the same letter
built against the engines this document replaced; the transaction's
REQUEST COPY into the SPI host - 130 cycles of a 48-byte `Request` and a
prologue, measured as the cost of `start()` with nothing to send - is
the bus contract's and the same in both.

| operation | BEFORE | AFTER |
|---|---|---|
| copy, 4096 bytes as 1024 words: wall | 6504 | 6385 |
| copy: launch, completion's handler, fixed cost | 208, 94, 360 | 113, 79, 241 |
| paced, 256 words at 100 kHz into one cell: launch, handler | 221, 106 | 84, 80 |
| SPI1 engined write, 8-bit frames at /4: launch, interrupts, handlers, fixed cost | 738, 2, 290, 1087 | 379, 1, 145, 571 |
| SPI1 engined write at /16: fixed cost | 926 | 561 |
| SPI1, 16-bit frames at /4: interrupts for 256 frames, fixed cost | 256 (the pump), 36161 | 1, 582 |
| bus masters still counted after the SPI requests | 1 | 0 |

- **Six cycles an item, by the difference of two sizes**: (wall at 4096
  less wall at 256) over 960 words is 6.00 exactly, BEFORE and AFTER - the
  controller's own rate, independent of the engine; at 4096 bytes a copy
  moves 92 MB/s, against the 576 of one word a cycle (x = 6.2), and the
  runtime's memcpy does the same 4096 bytes in about 3130 cycles, half the
  DMA's time ([../design/benchmark.md](../design/benchmark.md)): the
  engine frees the core, it does not outrun it.
- **The fixed cost of a copy is 241 cycles** of wall above the moving, of
  which the instrument is 107 (the stopwatch and the vector's stamp pair):
  134 cycles of the engine's own - the launch, five stores and the
  ledger's count, and the completion's handler.
- **THE SPI TRANSACTION'S FIXED COST went from 1087 to 514 cycles**, one
  interrupt where there were two, and one idle turn per wake. Less the
  instrument (28 + one stamp pair of 79; BEFORE two) and the request's
  copy (130), what the ENGINES cost a transaction is 277 cycles - their
  launch, the receive channel's one completion, both channels stopped -
  against 771 before. The per-frame
  cost is the wire's: 256 frames at /4 are 8192 cycles of SCK and the
  block adds nothing per frame.
- **A 16-bit frame is one 16-bit access**: a 256-frame write of half-words
  takes 16966 cycles for 16384 of wire, with one interrupt, where the pump
  took 52545 and 256 interrupts (x 3.20 to 1.03 at /4) - and the frames
  are the buffer's, judged with no wire by the SPI's own transmit CRC
  ([spi.md](spi.md)).
- **The pace holds to the cycle**: the 256 words of a 1440-cycle pace
  arrived 1434 to 1446 cycles apart, watched by the core under the mask on
  the counter's position - the watching loop's own turn is the spread -
  and the whole block took 369004 cycles for 368640 due, the first
  update's period and the completion's latency the difference.
- **A ONE-FRAME ENGINED SPI REQUEST WEDGED THE NEXT, before the rework.**
  The transaction completed inside the launch - the receive channel took
  its one frame before the launch's last store - and that store raised
  both DMA requests again after the completion had dropped them; the
  next request's transmit block then "completed" with nothing clocked
  and its receive block never started. The launch now programs both
  channels first and raises both requests in ONE store after them, the
  completion's store always the later; measured, a one-frame request
  ahead of every point of the letter and no stall.
- **AN ENGINED SPI TRANSACTION LEFT ITS RECEIVE CHANNEL COUNTED, before
  the rework**: its EN stays set after the block, nothing stopped it, and
  the bus-master count stood at one from the first transaction on - the
  core never slept again. Every engine now stops its channel at the
  block's end, and the count reads zero after the SPI letter's requests.

## Not covered yet

Driver gaps, each with its reason:

- **Half-transfer as a verb of the engines**: the flag and its enable
  are the channel's (`DmaFlag::half`, `arm()`), and no engine acts on
  the midpoint - the block source stops at every block instead, for the
  reason its own section gives.
- **The 128 KB reading of the 64 KB rule** for channels 2 to 5 of the
  CH32V303's DMA1 on some lots: declined, because the strict 64 KB
  reading is the one every lot satisfies and a program cannot read which
  lot it runs on.

Implemented but not bench-verified, each with what would measure it:

- **The transfer-error path** - TEIE, the engines' `abandon()` on an
  error, their fault counters and the ledger giving the count of a
  channel the hardware stopped back - is written from 11.2.1 and provoked
  by none of the five addresses the suite reads from, on either part. A
  peripheral that raises it, or a write into flash (which 11.1 lists as a
  legal destination and the bench has not tried), would measure it.
- **A ring restarted after a real transfer error**: the consumer's next
  look starts a ring whose channel stopped again, measured with `abandon()` standing
  for the error, which no address the bench can name provokes (above).
- **The CH32V203's serial round trip on one pair of pads.** The engines
  themselves carry a wire's data in the two bus chapters - sixteen bytes
  each way through SPI2's pair and a tenure's shapes through I2C1's,
  byte-exact against a peer board ([spi.md](spi.md), [i2c.md](i2c.md)) -
  and on the CH32V303VCT6 a serial run crosses the two controllers over
  the board's wires. What has no listener on the CH32V203 is USART2's
  pair to itself: a strap between PA2 and PA3 is what the suite's letter
  g detects and what would close it in one run.
- **Widening and truncation between unequal widths** (table 11-1): the
  channel takes two widths and every transfer above sets them equal. A
  copy with PSIZE and MSIZE apart, compared against the table's rows,
  is what would measure it.
- **The parts other than the CH32V203C8 and the CH32V303VC.** The request
  table folds through each part's own instances and the whole stratum
  compiles for all thirteen, both ways the hardware prologue can be built
  (`brio check ch32vx03`); the CH32V203RB's TIM5 rows, the smallest
  parts' absent ones and the 128 KB CH32V303's missing TIM5 and TIM8 are
  asserted at compile time and measured on none of them. What would
  measure them is a board - and for the CH32V303's DMA1, a die of a lot
  the 64 KB note does not restrict, on which the refusal the driver keeps
  would be a caution and not a necessity.
