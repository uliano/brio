# DMA (CH32V203, CH32V303)

This document is for a programmer who wants data moved with no
processor in the path on a CH32V203 or a CH32V303: a serial port's or a
bus's bytes carried by a channel instead of an interrupt a byte, a table
played into a timer or the DAC at a peripheral's pace, a converter's
samples filled into two buffers in turn, a block of memory copied or
filled while the program does something else. Most programs meet the
DMA only through a transport (a driver that moves a peripheral's data
in runs, a run being a contiguous span of elements handed over in one
call: the serial port's `Uart`, the `SpiHost`, the `I2cHost`). Each
transport takes a transmit engine and a receive engine as two of its
template parameters, its engine parameters, `NoDmaEngine` when it takes
none; the program names the engines there, binds the channels' vectors
to the transport's `dma_isr()`, and the transport does the rest
([usart.md](usart.md), [spi.md](spi.md), [i2c.md](i2c.md)).

This page covers what lies under those parameters and what a program may use
directly. `Dma<c>` is a controller, 1 or 2; `DmaChannel<c, ch>` one
channel of it; `DmaRequestOf<r>` names a peripheral's request and
answers the controller and the channel the silicon wires to it. Over a
channel sit five engines, an engine being a channel held for one shape
of transfer, armed once and then started block by block:
`DmaTxEngine<c, ch, Elem>` and `DmaRxEngine<c, ch, Elem>` pour runs
into a peripheral's data register and fill them from one, the second
also in a circular shape that fills a ring for ever; `DmaCopyEngine<c,
ch>` copies and fills memory; and `DmaLoopEngine<c, ch, Elem>` and
`DmaPingPongEngine<c, ch, Elem>` are the two block streams
([design/block-stream.md](../design/block-stream.md): caller-owned
buffers moved in blocks). Every one of those types is a monostate (a
class with only static members, never instantiated: one channel, one
set of state), and every one names its controller first, because on
the CH32V303 a channel number alone names two channels.

The smallest use, the first block of "How to use it" below - a fill and
a copy on the copy engine, the completion on its interrupt:

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

Line by line: `Copier` is a type alias for the copy engine on channel 1
of DMA1, a channel every part of the family has. `Copier::arm()` binds
the engine to that channel, once and before the first block: it opens
DMA1's clock gate, stops the channel and clears its flags, keeps the
channel's configuration word in the engine (memory to memory, the
destination incremented, the default `low` priority, the completion and
the error interrupts) and enables the channel's line in the PFIC, WCH's
interrupt controller. The third line stands at file scope: it binds the
channel's interrupt vector - `dma1_channel1_handler` is the name the
crt (the startup code that runs before `main()`,
`ch32vx03/src/glue/startup_ch32vx03.S`) gives it - to a body that calls
`service()`, which reads the channel's armed flags, clears them, and on
the completion or an error stops the channel and frees the engine; its
answer, the flags it found, is discarded. `frame` is the destination,
1024 32-bit words aligned to a word; `black` is the cell a fill
repeats, which the controller reads again at every beat (a beat is one
bus access, here one word), so it is passed by its address and stays in
memory. `fill()` starts a block that writes 1024 copies of `black` into
`frame` and returns at once, the block running while the core goes on;
the `while` loop waits for its end, `busy()` reading, with the
interrupt armed, the engine's own flag in RAM, which `service()`
clears. `copy()` then starts the second block, 1024 words from `saved`
(an array of the program's own) into `frame`, and a program waits for
it the same way before it reads `frame` or writes `saved`. The lines
after the binding run in `main()` once interrupts are enabled, or in an
active object's handler (an active object is a component of a brio
program with its own event queue, served one event at a time by the
kernel, [docs/learn/introduction.md](../learn/introduction.md)); the
engine's busy flag is tested and set without a mask, so `fill()`, `copy()`
and `busy()` are called from one context.

What decides correctness even there:

- **The vector bound, and interrupts enabled before the wait.** Armed
  with its interrupt, the engine learns that a block has ended only in
  `service()`, so `busy()` stays true until the vector has run. With
  the binding left out the vector is the crt's weak `default_handler`,
  a spin loop, and the program stops there at the first completion with
  no fault and no message; with interrupts still masked
  (`brio::enable_interrupts()` not yet called) the `while` loop never
  ends. Armed with `Copier::arm(brio::DmaPriority::low, false)` the
  engine takes no interrupt and needs no binding, and `busy()` finds the
  completion in the channel's flags itself.
- **The one spelling of a binding.** A vector is bound with
  `BRIO_CH32_VECTOR(name) { body }` or one of its two forms, under the
  crt's name. On the CH32V303, whose images are built with the F
  extension and, by the project's default, the hardware prologue on,
  that makes a naked
  trampoline whose body must do no floating-point work;
  `service()` does none, and `brio check ch32vx03`'s vector guard fails
  an image whose body reaches a floating-point instruction
  ([platform.md](platform.md)).
- **An answer the excerpt discards.** `fill()` and `copy()` answer false
  and start nothing while a block is in flight, and for a run the engine
  refuses: a null address, a count of zero or above the 65535 CNTR
  holds, an address not aligned to the beat (the controller would round
  it down in silence, "The addresses are aligned by the silicon" below)
  and, on the CH32V303's DMA1, a span whose first and last access at
  either end fall in different 64 KB pages - a rule kept on every lot of
  that class, because a program cannot read which lot it runs on ("The
  64 KB rule of the CH32V303's DMA1" below); DMA2 and every CH32V203
  are free of it. A refused call writes nothing, leaves `busy()`
  answering what it answered before and raises no error: a program that
  must know looks at the answer.
- **The buffers stay put until the block ends.** The controller reads
  `black` and `saved` and writes `frame` after the call has returned, so
  all three stay untouched and in scope until `busy()` answers false;
  nothing checks it.
- **No sleep while a block is in flight.** In a sleep of any depth on
  this family no bus master but the core gets a cycle ("In Sleep the bus
  matrix serves the core alone" below), so starting a block takes one
  count of the bus-master count (how many masters other than the core
  are working, `ch32vx03/bus_activity.hpp`, [sleep.md](sleep.md)) and
  `service()` gives it back. While it stands
  the platform's `idle()` returns at once instead of sleeping: the
  program spins in its loop rather than starve the block.
- **The wait reads RAM.** A core that reads a peripheral register back
  to back keeps the channels that need that bus waiting (measured on the
  CH32V303VCT6: "Awake, a core reading a peripheral back to back starves
  them too" below), so a wait on a block reads a flag in SRAM, as
  `busy()` does here.

The engines compute nothing from the clock's rate, so the order against
the clock task does not decide a copy's correctness; the figures below
are at 144 MHz. Two more facts decide correctness. A channel has one
owner for the whole image: the compiler checks every claim, the transports
write theirs, and a channel a program programs by hand is checked only if
the program writes its claim ("One owner a channel" below). And a request
that a USART, an I2C or a timer raised and no channel took (measured on
USART2, UART4, I2C1, TIM1, TIM3 and TIM4) is held until the controller
acknowledges it or the peripheral's reset line is pulsed, whatever its DMA
bit says (an SPI's, measured on all three instances, is not), so every
transport's `init()` pulses its block's reset before it arms an engine and
its `release()` pulses nothing. Without that pulse the channel's next
enable moves one stray item on a USART's or a timer's request, and an I2C1
read never reaches its channel ("A released requester keeps the request
it raised" below).

The rest of this page, in order: the documents of record; what the
silicon does (the channel as the request, a channel's registers, the
64 KB rule of the CH32V303's DMA1, the controller's offer item by item,
the flags, the vectors and the gates, and the starvation of every bus
master but the core, asleep and awake); the types and verbs (the
request table, the block, the channel, the engines, the block engines,
a transport's two engine parameters); how to use it (the copy engine, the raw
channel polled and on its interrupt, a paced block, a request named
rather than numbered, a ring read behind its channel, a console on both
engines); the bench findings, with the engines' costs; and what is not
covered.

The controller is the STM32F1's DMA under WCH's register names, in two
sizes. The CH32V203 has one controller of eight channels, one more than
that ancestor; the CH32V303 has two - a DMA1 of seven channels and a
DMA2 of eleven, whose last four report in a flag register of their own.
Every channel is wired to a fixed handful of peripheral requests, an
arbiter over four software priorities decides between the channels of
one controller with the channel index breaking ties, and every channel
has four flag bits.

## Documents of record

The CH32F/V20x_V30x_V31x reference manual V2.3
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
Three device classes - the manual's groups of parts, each with its own
register notes and request tables - read the chapter: the CH32V20x_D6
for every CH32V203 up to the CH32V203C8, the CH32V20x_D8 for the
CH32V203RB, and the CH32V30x_D8 for the four CH32V303 parts. Driver:
[brio/ch32vx03/dma.hpp](../../brio/ch32vx03/dma.hpp), with the request
table, the slot a request lands in and the empty slot's tag in
[brio/ch32vx03/dma_engine.hpp](../../brio/ch32vx03/dma_engine.hpp).
Reference suites, three because a channel has one owner in an image ("One
owner a channel" below): `test_vx03_dma`, every channel memory to memory
(the widths, the sleep, the transfer error, the priorities, the
vectors, the 64 KB rule); `test_vx03_dma_paced`, the timers' requests;
and `test_vx03_dma_serial`, the USARTs' requests and the engines a
transport takes (a suite's letter is one command key of its console and
the group of verdicts it runs, each letter's name kept across the three;
`z` is the key that runs a suite's single-board letters, and on the
CH32V303 `y` the two-board ones against the far board). The engines'
costs are letter d of the bench app `bench_vx03`
([../design/benchmark.md](../design/benchmark.md)).
The receive engine's circular shape is the producer of
[util/ring.hpp](../../brio/util/ring.hpp)'s `HardwareRing`
([../design/ring.md](../design/ring.md)).

## What the silicon does

### The channel is the request

There is no request multiplexer. The tables wire each peripheral event to
one channel of one controller, and an engine is named with that table in
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

A request lands in a slot, not on a channel number. With two controllers
a channel number alone names two different channels - UART4 transmits on
DMA2's fifth, which is not DMA1's fifth - so the table answers a
`DmaSlot`, the controller and the channel, and every check a transport
makes compares the whole slot. Two answers would invite a check that
asks only the second, and a DMA2 engine would then pass for a DMA1
request that happens to share its number: a wedge with no error flag.

Which rows exist is a part fact, not a family one. A count is not a
list: the smallest CH32V203 package offers one usart and it is USART2,
one SPI and no I2C at all, and the 128 KB CH32V303 has no TIM5 and no
TIM8. The driver folds that through the part table
(`brio/ch32vx03/parts/`, the per-part facts `device::` exposes) and
answers the empty slot - "no channel" - for such a row.

A timer publishes several events and they do not share a channel: an
update is not a capture. TIM1's fourth channel, its trigger and its
commutation are one row, because they raise the same request, and so do
the matching events of TIM5, TIM8, TIM9 and TIM10.

A channel serves every request wired to it, not the one a program meant.
The rows of a channel are an OR of the peripherals' requests - 11.2.3's
"each channel corresponds to multiple peripheral requests", switched by
each peripheral's own DMA bit - so a peripheral left with its DMA bit set
drives any transfer loaded on its channel. Measured on the CH32V303VCT6:
a UART4 receiver left running with DMAR set held its request standing on
DMA2's channel 3, and a transfer programmed there for TIM6's update moved
all sixteen of its items in a few microseconds instead of one a period (a
staging of two owners, no suite reproducing it: "A released requester"
below). Stopping a channel does not withdraw a request, and neither does turning
the peripheral off.

A released requester keeps the request it raised. A peripheral's DMA
request, once raised, is held until the controller acknowledges it or the
peripheral's reset line is pulsed: clearing its DMA enable (DMAT, DMAR,
DMAEN, UDE), its UE or PE, or its counter's CEN leaves it standing, and
gating its clock freezes it on the channel's OR. 11.2.3 says only that a
channel's requests are switched on and off by each peripheral's own DMA
bit, and RM0008 13.3.1 - the STM32F1's controller, this one's lineage -
that a peripheral releases its request on the acknowledge; "switched off"
measures as no new request. A channel's next enable moves one item on the
held request - from or to whatever PADDR names - and then waits for the
request to fall: a requester still clocked drops it (one stray item, then
the stream), a gated one never does (the channel stalls after that item),
and a level that stays up raises it again at once (the block runs to its
end on the stray). The channel itself is neither wedged nor marked: a
memory-to-memory block on it runs whole through a frozen request, and a
USART2 whose channel was never enabled holds one as well.

The measurements behind that paragraph staged a second owner on the
channel. The ownership rule refuses such a staging ("One owner a
channel" below) and no suite reproduces it, so they stand as measured
facts of the silicon and cannot be re-run here. On both parts, with
USART2, I2C1 and TIM4 on DMA1's channels 6 and 7:

- A USART2 initialized with DMAT and released stalled an I2C1 read on
  channel 7 after one item, the I2C's own address byte. So did one whose
  DMAT and DMAR were cleared before the gate, or after it with the clock
  opened again. One released with its reset pulsed, before the gate or
  after it, did not, and a reset pulsed during the stall ended it.
- An I2C1 tenure (one request's hold on the bus, START to STOP) abandoned
  with RxNE standing under DMAEN stalled a USART2 transmit on the same
  channel after one byte, unless the I2C's reset or its SWRST was pulsed.
- A TIM4 stopped with UDE cleared but kept clocked left one stray item.
- SPI1's, SPI2's and, on the CH32V303VCT6, SPI3's requests do not outlive
  a release: a transmit request over TXE and a receive request over a
  standing RXNE, each let go by `SpiHost::release()` with no reset, left a
  timer-paced block on those channels a clean staircase.
- UART4's transmit request on DMA2's channel 5 is held without its reset,
  one stray item and the channel stalled, and let go with it.
- A USART's transmit request is TXE under TE: staged with UE and DMAT
  alone, nothing is held.

Under the ownership rule the one that can meet a request held across a
release is the same owner initialized again. Measured so on both parts: a
USART2 receive request left under DMAR by a frame no channel took, a TIM1
update request left under UDE and an I2C1 receive request left under
DMAEN, each released with its enable set and its gate closed. With no
reset at the next `init()` the ring holds the stale frame ahead of the
next burst, the paced block moves one item before its pace and an I2C
read storms its event vector and never launches (the control in "The same
owner over its own held request" below).

So the contract on this family is the one
[design/dma.md](../design/dma.md) states, "The release contract": every
transport's `init()` - `Uart`'s, `SpiHost`'s and `SpiClient`'s,
`I2cHost`'s and `I2cClient`'s, `Adc`'s, `Dac`'s and `Tim`'s - opens the
gate and pulses the block's reset before it writes a register of its own
or arms a channel, and no `release()` pulses a reset. Every `release()`
closes the gate. The first five also give their pads back and close their
interrupt line; `Adc`'s closes its line, and the pads of an `Adc`, a `Dac`
and a `Tim` are the program's own.

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

- **The count is in items, up to 65535** (11.1, 11.3.4), and a count of
  zero moves nothing whether the channel is enabled or not.
- **The widths are 8, 16 and 32 bits** and the fourth code is Reserved
  (11.3.3). Source and destination widths are independent, and table
  11-1 is the padding and truncation between them.
- **The addresses are aligned by the silicon, silently**: 11.3.5 and
  11.3.6 say the module ignores bit 0 of a 16-bit access and bits [1:0]
  of a 32-bit one. A half-word transfer pointed at an odd address
  therefore moves the wrong bytes and reports success.
- **Circular mode is not for memory-to-memory** (11.2.1): the one
  configuration the chapter forbids. Memory-to-memory has no request to
  reload against - it runs on the enable.
- **In circular mode the count reloads itself and so do both addresses**
  (11.2.1's cycle mode, 11.3.4): when CNTR reaches zero it is loaded
  with its initial value again and the channel goes on, with no CPU in
  the path. A transfer is the read, the store and then the decrement
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
the rule is kept on every lot of the class: a transfer whose first and
last access of either end fall in different 64 KB pages is refused on
DMA1 of the CH32V303, nothing written, and taken on DMA2. The note goes
on to relax channels 2 to 5 to a 128 KB boundary on some lots; the driver
does not, because the strict reading is the one no lot contradicts. No
CH32V203 is named by the note, and no part of that series is held to it.

The rule is real, and its failure is a wrap. On the CH32V303VCT6 the
refusal bypassed - the channel programmed through the piecewise verbs,
which never see a transfer whole - thirty-two bytes of flash straddling
0x0801 0000 arrived on DMA1 with their upper sixteen read from 0x0800 0000
instead: the address counter wrapped inside the 64 KB page the transfer
started in, a transfer that completed and reported nothing. DMA2 moved
the same span byte for byte. That die is a lot the note restricts.

### What the controller offers, item by item

The engines are written from this chapter, and every item of it that
bears on what a block costs is used or declined here with its reason:

- **The register file, four words a channel, every field read-only while
  EN is set** (11.2.1's note). Used as the shape of the engines: what is
  constant for a binding - PADDR, the direction, the increments, the
  priority, the interrupt enables - is written once when the engine is
  armed and kept as a word in the engine, because the register cannot
  keep it across a block (EN is in the same word); per block the engine
  writes MADDR, CNTR, the channel's four flags in INTFCR and CFGR whole
  with EN - four stores, no read. The vendor's own restart of an SPI
  channel (the EVT's `SPI_LCD`) touches the same four registers.
- **One store sets the fields and EN at once.** The fields are read-only
  while EN is set, and the store that sets it lands on a channel whose EN
  is clear: measured, a word copy, a byte copy and a half-word fill back
  to back, each starting with a width the last had not, every byte where
  it belongs.
- **The width, 8, 16 or 32 bits a side** (11.3.3): used per block. The
  beat of a block is the element of its run - a 16-bit SPI frame is one
  16-bit access to DATAR - and the engine's `Elem` is the widest beat the
  binding takes.
- **Circular mode**: used by the player (`DmaLoopEngine`) and by the
  receive engine's circular shape - a transport's whole receive ring as
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
  of a paced block (TIM1's on DMA1's fifth channel, measured below), and
  memory to memory is the software trigger.
- **The gate**, DMA1EN and DMA2EN: opened once, when an engine is armed,
  never on the block path. Nothing in this stratum closes it again.
- **The flags**: write-one clears in INTFCR, one store a block for the
  channel's four; an engine's ISR body reads INTFR once and masks it with
  the engine's own armed word - not a second read of CFGR.
- **The interrupts**: one vector a channel, each engine arming only what
  its user needs - the SPI host's transmit engine its error alone, the
  receive block's completion proving the transmit's.
- **The priorities**: an argument of each engine's `arm()`, and the level
  is the peripheral's ([../design/dma.md](../design/dma.md)): the arbiter
  grants the bus item by item to the highest level asking, the lower
  channel number between equals, and a memory-to-memory channel asks on
  every cycle of its block - so a receive channel it outranks moves
  nothing until the block ends, and one a level above it loses one item
  time at most (measured below). The defaults are the rule's by direction
  - the receive engine and the block source at very_high, the transmit
  engine and the player at high, the copy at low - and each driver names
  its engines' level with its chapter's reason: the USART's ring at
  very_high (DATAR holds one frame) and its transmit at high, the SPI
  host's receive at very_high (OVR, 20.2.7) and transmit at high, the I2C
  host's two at high (the bus stretches, 19.3), the converter's stream at
  very_high (no overrun flag: a sample is written over in silence), the
  DAC's at high (a late trigger converts the value held).
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
  the vector table's entry 27 and the eighth sits on the tail its device
  class has - entry 62 on the CH32V20x_D6, 67 on the CH32V203RB, whose
  table carries the Ethernet pair before it. On the CH32V303 DMA1's seven
  are 27 to 33, and DMA2's eleven are split: channels 1 to 5 at 72 to 76,
  channels 6 to 11 at 98 to 103 (all eighteen measured). Either way a
  channel's handler reads only its own flags and needs no "which
  channel" question.
- **The gates, DMA1EN and DMA2EN in RCC_HBPCENR, are closed at reset**
  (the register's reset value opens the SRAM alone).
- **These blocks have no reset line.** RCC_AHBRSTR (3.4.11) reserves bits
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

So on this family a DMA-fed transport does not sleep, and that is a
mechanism and not a rule the programmer keeps: every verb that sets a
channel's EN takes one count of an active bus master
([sleep.md](sleep.md)) and every verb that clears it gives the count
back, on either controller - the controller's ledger, one byte a
channel, saying which channel holds one, because the one thing that
drops EN behind software's back (a transfer error) would otherwise leave
a count nobody gives back. The kernel's idle path does not sleep while
the count stands and a sleep site refuses to arm over it. An engine
stops its channel at the block's end for the same reason: EN stays set
after a completed block, and a channel left enabled holds its count. `any_enabled()` is
the same question asked of the registers instead, of every channel of
every controller the part has, for a caller that wants the silicon's own
answer. The alternative the same measurements opened is to slow down
instead: the chip carries data with HCLK as low as 24 MHz.

### Awake, a core reading a peripheral back to back starves them too

The same arbitration bites with the core running. A core that spins on
a GPIO input register - a load every few cycles through the peripheral
bus - keeps a channel waiting for that bus: measured on the
CH32V303VCT6, SPI3 as a client on DMA2's channels 1 and 2, the
receive channel lost frames (OVR, 14 to 31 of 64 frames landed) behind a
host clocking 8-bit frames at 18 MHz, 71 to 155 core cycles a frame,
while the core polled the select pad in a tight loop; the
same serve with the core spinning on an SRAM word and reading the pad
once every 256 turns - or with 64 empty instructions between pad reads -
kept every frame, and so did the core polling the SRAM word alone. So a
program that waits on a DMA block waits on SRAM (a completion flag, a
count it reads rarely) or on the core's own instructions, and reads a
peripheral register only now and then; the spi_peer's serve reads its
select pad once every 256 turns ([spi.md](spi.md)). The
CH32V203C8T6's DMA1 under the same loop was not measured apart: its
peer serve carried the change before its ladder ran.

## Types and verbs

### The request table

[brio/ch32vx03/dma_engine.hpp](../../brio/ch32vx03/dma_engine.hpp) holds
what a transport needs without including the controller: `NoDmaEngine`,
the empty slot's tag whose only member is `present`; `DmaSlot`, a
controller and a channel (the empty slot both zero); `dma_engine_slot`
and `dma_engines_distinct`, the two checks a task (a driver layer with
state of its own, served from its peripheral's interrupt, that an
active object or `print` calls) makes on a named engine; and the table
itself - `DmaRequest`, one enumerator per row of tables 11-2 to 11-6,
`dma_request_channel` answering the slot with the part folded in (the
empty slot where the part has not got the peripheral),
`dma_request_present`, and `DmaRequestOf<r>` with its
`slot`, `controller` and `channel`, the compile-time form that refuses a
request this part cannot raise; and `dma_engine_claim`, the claim a
transport makes for each engine it takes (the next section).
A driver with an engine parameter includes this file alone, so a program
with a console does not carry the controller.

### One owner a channel

A channel belongs to one owner for the whole life of an image, and on
this family the compiler checks every claim written
([design/dma.md](../design/dma.md), "The ownership rule"). The slot is the controller and the channel,
`DmaSlotId<c, ch>` - DMA1 is 1 and the CH32V303's DMA2 is 2, so DMA1's
fifth channel and DMA2's fifth are two slots - and its owner is the
peripheral request the channel serves, `DmaOwner<kind, instance,
signal>` (USART2's transmit is `DmaOwner<DmaPeriph::usart, 2,
DmaSignal::tx>`, TIM1's update `DmaOwner<DmaPeriph::tim, 1,
DmaSignal::update>`), or a tag for a channel no request paces.

Who claims:

- **A transport claims its own slots.** `Uart`, `SpiHost` and `I2cHost`
  claim each engine's slot for their instance's request in their class
  body, the I2S face of an SPI raising the SPI's requests; `Adc`'s and
  `Dac`'s stream verbs (`claim_stream()`, `claim_dual_stream()`) claim
  the converter's slot where they are called. A program that names the
  engines in a transport's parameters has nothing more to write.
- **A copy claims its channel under the channel's copy tag.**
  `DmaCopyEngine<c, ch>` claims its slot for `DmaCopyOwner<c, ch>`, and
  a block memory to memory programmed by hand on the same channel claims
  the same tag: both are a copy on that channel.
- **A program claims what it programs by hand.** An engine used without
  a transport - a block paced by a timer, the client side of a link - and
  a `DmaChannel<c, ch>` a program loads itself are claimed by the
  program, beside the type, for the request that paces the channel or
  under a tag of its own, one tag a channel:

  ```cpp
  using Row = brio::DmaRequestOf<brio::DmaRequest::tim1_up>;
  using Paced = brio::DmaTxEngine<Row::controller, Row::channel, uint32_t>;
  static_assert(brio::dma_claim<brio::DmaSlotId<Row::controller, Row::channel>,
                                brio::DmaOwner<brio::DmaPeriph::tim, 1, brio::DmaSignal::update>>);
  ```

  Nothing checks that a program wrote this claim: an engine or a channel
  carries no owner of its own, so one the program forgets to claim is
  invisible to the check.

A second owner on a claimed channel does not compile: a USART2 transport
with its transmit engine beside an I2C1 host with its engines reads, at
the second one's line,
`ambiguating new declaration of 'constexpr brio::DmaOwner<brio::DmaPeriph::i2c, 1, brio::DmaSignal::rx> brio::dma_owner_of(DmaSlotId<1, 7>)'`
with USART2's transmit named on the note under it. The claim's other
direction - one request on two channels - never fires for a transport
here, each request being wired to one channel and the transport
refusing any other; it fires for a program's tag claimed on two
channels. The claims add no byte and no cycle to an image.

The rule leaves no channel to share, so the request tables (RM tables
11-5 and 11-6, and for the CH32V303 tables 11-2, 11-3 and 11-4), which
OR several peripherals' requests onto each channel, force the choice of
an instance wherever two peripherals a program wants on DMA sit on one
channel. The choices the tables force (a peripheral on DMA, and what
that leaves off DMA):

| on DMA | forbids on DMA | the channels, and where |
|---|---|---|
| I2C1, either direction | USART2 with both engines | I2C1's transmit is USART2's receive channel (DMA1's 6), its receive USART2's transmit channel (7); every part with both |
| SPI1 | USART3 | DMA1's 2 and 3; the CH32V203C8 and RB and every CH32V303 (the smaller parts have no USART3) |
| SPI2 | USART1, I2C2 | DMA1's 4 and 5; the parts with SPI2 |
| ADC1 | the fourth port's transmit | DMA1's 1; the CH32V203C8 and RB, where UART4 is DMA1's |
| DAC1 | UART4's receive | DMA2's 3; the CH32V303RC and VC, the parts with UART4 |
| DAC2 | UART5's transmit, the SDIO | DMA2's 4; the CH32V303RC and VC, the parts with UART5 and the SDIO |
| SPI3 | UART5's receive | DMA2's 2; the CH32V303RC and VC |
| ADC2, on a lot that has its request | UART4's transmit | DMA2's 5; the CH32V303RC and VC, the parts with UART4 (table 11-3's note 1 gives the row to some lots only) |

So a console on USART1 with both engines leaves SPI1 for a display on
DMA and a console on USART3 leaves SPI2; and a program that wants I2C1
on DMA keeps its console off USART2. Against any one other use a timer's
request is never the forced one: a free timer event remains for a pace,
short of the full sets in the table below, and only a stream bound to one
timer is held to that timer's channels (TIM1 reaches DMA1's 2 to 6, TIM2
1, 2, 5 and 7, TIM3 2, 3 and 6, TIM4 1, 4, 5 and 7; on the CH32V303 TIM5
DMA2's 1, 2, 4 and 5, TIM8 its 1, 2, 3 and 5, TIM6 its 3, TIM7 its 4,
TIM9 and TIM10 its 6 to 11; on the CH32V203RB TIM5 DMA1's 1, 3, 6, 7 and
8).

Some parts cannot put every use a program might want on DMA at once,
whatever the instances. Written here with the uses as a console with
both engines, a second UART with both engines, an SPI host's two
engines (the host takes both or neither), an I2C transmit, one converter
stream and one timer pace:

| part | the uses that cannot all be on DMA | why |
|---|---|---|
| CH32V203C6, F8, G6, G8, K6, K8 | a console, a second UART and an I2C transmit; or a console, a second UART, an SPI host, a converter stream and a timer pace | USART1 and USART2 are the only UARTs, and USART2's channels are I2C1's; eight requests for seven channels, the eighth serving the absent UART4's receive alone (table 11-5) |
| CH32V203C8 | a console, a second UART, an SPI host, a converter stream and a timer pace, or an I2C transmit in the pace's place | channel 8 serves UART4's receive and nothing else (table 11-5) |
| CH32V203RB | a console, a second UART, an SPI host, a converter stream and an I2C transmit | as the C8, TIM5's update on channel 8 rescuing the pace (table 11-6) |
| CH32V303CB, RB | a console, a second UART, an SPI host and an I2C transmit; on a lot without ADC2's request also a console, a second UART, an SPI host, a converter stream and a timer pace | USART1 to USART3 the only UARTs, each on an SPI's or an I2C's channels; DMA1 has seven channels (tables 11-2 and 11-3, table 11-3's note 1) |

The CH32V203F6 offers one UART and no I2C, and the CH32V303RC and VC
have room for all of them at once (UART5 as the second UART, a TIM9 or
TIM10 event as the pace).

### The block

`Dma<1|2>`, the second refused on a part with one controller: `open()`
(the gate, which the channel's `enable()`, `configure()`, `prepare()`,
`load()` and `stop()` call first; `trigger()` reads CFGR and CNTR
before its `enable()` opens it, and `set_count()`, `set_peripheral()`,
`set_memory()` and `arm(mask, on)` never call it), `opened()`,
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
interrupt enables, `isr()` - the ISR body that reads only the armed
flags that are up, clears exactly those with the global bit and hands
them back, deciding nothing -, `progress()`, `irq()` and `stop()`.
`controller`, `number`, `slot` and `bounded_to_64k` say what the channel
is.

Every configuring verb refuses while the channel is enabled, every verb
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
and the error by default; the level high unless named -, `start()` over a span of bytes, of half-words
or (where `Elem` is 32 bits) of words, and over a pointer and a length;
the two halves of `start()`, `claim()` (the busy flag's test-and-set, what
a transport racing its own completion handler masks) and `launch()` (the
programming of a claimed engine, which takes the flag down when it
refuses), `unclaim()`, `mark_busy()` (the flag's set with no test, before
a `launch()`, for an owner whose own state says no block of its is in
flight: the I2C host's entry that takes ADDR), `start_fixed(cell, n)` for a full-duplex bus
clocking a read, `complete()` (the channel stopped and how many elements
the block carried, so the transport releases exactly that much of its ring),
`kick()`, `abandon()`, `faults()`, `busy()`, `in_flight()`, `progress()`.
`DmaRxEngine<c, ch, Elem>` fills from one register in two shapes on one
binding. The one-shot shape fills a run and stops - a bounded block, the
receive half of a bus transaction: the same two `arm()`s, at very_high
unless named, `start()` over
the mutable spans, `start_discard(cell, n)`, `complete()` (the channel
stopped - EN stays set after a completed block, and a channel left
enabled holds the bus-master count), `take()` - how many arrived since
last asked, one CNTR read, nothing suspended -, `harvest()` - the same
arithmetic without consuming it -, `idle()`, `full()`, `capacity()`,
`taken()`, `kick()`, `abandon()`. The circular shape fills a ring for
ever: `arm(data, storage, priority)` is handed the ring's whole storage -
the caller's array, its element the beat and its length, checked at
compile time, the count every lap reloads - and keeps the address, the
length and the ring's word (CIRC, MINC, the beat, the priority, the lap
and the error armed, the half only when its `half_mark` argument asks
for it), refusing a storage misaligned for
its element or, on the CH32V303's DMA1, one across 64 KB; `start()` with
no run starts the ring from its first element with the lap count at
zero, the binding's five stores; `remaining()` (CNTR, one load) and
`laps()` (what `lap()`, the completion's verb on a ring, counted) make
the engine util/ring.hpp's `RingCounter`, so
`HardwareRing<storage, DmaRxEngine<c, ch, Elem>>` is the ring's
consumer half; and `bounded_to_64k` says whether the channel is held to
the 64 KB rule, so a program can align the storage to its own size there
and no placement can be refused. Nothing re-arms a ring: the one way it
stops by itself is a transfer error, which `idle()` reports and
`start()` answers. Both engines publish `present`, `controller`,
`channel`, `slot`, `width`, `element`, `service()` and the flag names,
and a transport reaches its engine only through those.

The beat is the element of the run. PSIZE and MSIZE follow the span: a
16-bit frame is one 16-bit access to the data register. `Elem` is the
widest beat the binding takes - 16 bits by default, every serial and bus
data register of this family; 32 for a timer's 32-bit compare or a
memory cell - and a wider span is a compile error. A half-word run at an
odd address is refused at run time, the controller rounding it down in
silence otherwise (11.3.6), and so is a run longer than CNTR counts or
one across 64 KB on the CH32V303's DMA1. `arm()` refuses a data register
that is not aligned to `Elem`.

`DmaCopyEngine<c, ch>` is memory to memory: `arm(priority, interrupt)`,
low unless named,
`copy(dst, src, n)` and `fill(dst, cell, n)` - `n` in elements, the
element type the beat, the fill's cell the caller's and in memory,
because the controller reads an address every beat -, `busy()` (which,
armed without the interrupt, also finds the completion), `abandon()`,
`service()` (the ISR body that ends the block), `faults()`, `stop()`.

Two engines of one transport name two channels: a channel moves data one
way, and the table gives each direction its own.

### The block engines

`DmaLoopEngine<c, ch, Elem>` and `DmaPingPongEngine<c, ch, Elem>` are
this family's realizations of
[util/block_stream.hpp](../../brio/util/block_stream.hpp)'s
`BlockPlayer` and `BlockSource`, and they differ in exactly one thing:
the player rides the controller's circular mode, where CNTR reloads
itself and the lap interrupt does nothing but count, and the source does
not. The reason is the contract's and not the API's - "skip rather than
tear" cannot be decided on a channel that never stops - and it is
measured here, on the fastest possible reader: see [adc.md](adc.md),
whose converter is these engines' first user and whose suite carries the
number. On the CH32V303 the player has the peripheral it was written
for: the DAC, whose two requests are DMA2's channels 3 and 4
([dac.md](dac.md)).

### The transport's two engine parameters

[brio/ch32vx03/usart.hpp](../../brio/ch32vx03/usart.hpp)'s `Uart` takes
a transmit engine and a receive engine as two of its template
parameters, `NoDmaEngine` by default: the transmit engine drains the TX
ring by contiguous runs, and the receive engine runs in its circular
shape over the whole receive ring's storage, the transport's receive
ring being the `HardwareRing` over it - a byte that lands is readable at
once, a burst wraps the storage's end with no CPU, and nothing is lost
between runs because there are none. The two bus engines take the same
pair in the same place, in the one-shot shape - `SpiHost` carries a
block's data phase on them ([spi.md](spi.md)) and `I2cHost` a tenure's
([i2c.md](i2c.md)). Each arms its engines at its own level (the offer's
priorities above): the Uart's receive ring at very_high unless its
`UartOptions::rx_priority` says otherwise, its transmit at high.
`dma_isr()` is the ISR body of whichever channels
the transport holds - on the receive channel a completion is a lap,
counted, and the lap's half and full marks are the receive edge, its true
answer the edge `isr()` gives -, `harvest()` the same edge asked from the
consumer's side and the ring's restart after an error
([usart.md](usart.md)), and `dma_faults()` counts the
blocks thrown away. A block's start masks interrupts for a few instructions,
twice at most: on every start around the ledger's count of working bus
masters (ch32vx03/bus_activity.hpp), and on the `Uart`'s transmit around the
engine's `claim()`, the busy flag's test-and-set. It programs the channel
unmasked, no block being in flight that could complete under it. An engine
is refused on any slot but the instance's own, controller and channel, which the request table answers; each
resource publishes its two slots as `dma_tx_slot` and `dma_rx_slot`.
Without an engine every branch is compiled out: `init()`'s reset leaves
CTLR3 zero and nothing writes it, a half duplex, RTS or CTS option apart.

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
// A copy programmed by hand: the channel's copy tag owns it.
static_assert(brio::dma_claim<brio::DmaSlotId<1, 1>, brio::DmaCopyOwner<1, 1>>);

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
static_assert(brio::dma_claim<brio::DmaSlotId<Row::controller, Row::channel>,
                              brio::DmaOwner<brio::DmaPeriph::tim, 4, brio::DmaSignal::update>>);
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
static_assert(brio::dma_claim<brio::DmaSlotId<Row::controller, Row::channel>,
                              brio::DmaOwner<brio::DmaPeriph::tim, 2, brio::DmaSignal::update>>);

(void)Sampler::load(brio::DmaTransfer{
    .peripheral = brio::Tim<3>::cnt_address(), .memory = buffer, .count = 16,
    .config = {.peripheral_width = brio::DmaWidth::half,
               .memory_width = brio::DmaWidth::half}});
brio::Tim<2>::interrupts(brio::tim_ude, true);   // the request enable
brio::Tim<2>::enable(true);
```

The same timer's samples into a ring, read behind the channel for as long
as it runs - the receive engine's circular shape and util/ring.hpp's view
over the same storage:

```cpp
using Row = brio::DmaRequestOf<brio::DmaRequest::tim2_up>;
using Sampler = brio::DmaRxEngine<Row::controller, Row::channel>;
static_assert(brio::dma_claim<brio::DmaSlotId<Row::controller, Row::channel>,
                              brio::DmaOwner<brio::DmaPeriph::tim, 2, brio::DmaSignal::update>>);
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
interrupt of its own. `release()` stops both channels and closes the gate
over whatever request stands, and the port's next `init()` withdraws it
through the block's reset line, the one act that does, before it arms an
engine.

## Bench findings

The three suites measure at 144 MHz, the source of every copy a
four-kilobyte pattern in the image itself and the core's own counter the
ruler; letters a, c, d, f, h, i, j and m are `test_vx03_dma`'s, b, e, o,
k and l `test_vx03_dma_paced`'s, and g, p, q, r, s and n
`test_vx03_dma_serial`'s. On the CH32V203C8T6 the letters of a part with
one controller run with the board bare but for the core board's crossed
pair to the fourth port (PB0 to PA3, which letters q and r take): **24,
13 and 25 verdicts in `z`**. On the CH32V303VCT6 DMA2's single-board
letters run beside DMA1's with the far board dark: **35, 18 and 16
verdicts in `z`**; that board's crossed pair is retired (UART4's default
pads PC10/PC11 are the desk pair's SPI link), and its letters n, q and r
are the two-board key `y` against the far chip's usart_peer - UART4 on
its second column, PB0/PB1, its engines DMA2's channels 5 and 3: **11
verdicts**. Where one number is given below it is both parts'; where
they differ each is named.

- **Six cycles an item at every width, and the width is the rate.** Four
  kilobytes copied flash to RAM take 24809 to 24811 cycles as 4096 bytes,
  12535 to 12536 as 2048 half-words and 6395 as 1024 words on the
  CH32V203C8T6 - 6.05, 6.12 and 6.24 cycles an item - and 24917, 12635 to
  12641 and 6491 to 6500 on the CH32V303VCT6's DMA1 - 6.08, 6.17 and 6.34.
  At 144 MHz the first part's are 23.8, 47.1 and 92.2 megabytes a second,
  and every byte of all three arrived.
- **The half flag rose with 2050 of 4096 moved** on the CH32V203C8T6 and
  2049 on the CH32V303VCT6, which is 11.2.1's own rule read back; INTFR
  stood at 0x7 (the global, complete and half bits of the channel) and
  one write to INTFCR took all four down.
- **Unequal widths move as table 11-1 draws them** (letter a): the nine
  pairings of PSIZE and MSIZE, four items each from the image's pattern
  - each end stepping by its own width, a narrower value zero-extended
  into a wider target, a wider one cut to its low bytes, little-endian at
  both ends, nothing written past the last item - every target byte
  compared against the table on both parts.
- **EN stays set after a completed block** - CNTR at zero, TCIF up, the
  channel enabled and idle - and while it is set every configuring verb
  of the driver refuses: the configuration, the count, a whole transfer
  and an address, with CFGR unchanged across all four.
- **A TRANSFER ERROR COMES FROM THE MEMORY ZONES, AND NOTHING FROM 0x3000 0000
  UP RAISES ONE.** Walked over the map on both parts, read at every width every
  64 MB and at some sixty named points, and written where nothing answers: the
  zones below 0x3000 0000 refuse a channel past the memory that is there.
  A read at 0x1FFF 0000, at 0x0808 0000 and at the first byte past the SRAM (0x2000
  5000 on the CH32V203C8T6, 0x2001 0000 on the CH32V303VCT6) raises TEIF and drops EN,
  as does a read or a write at 0x2FFF F000, the top of the SRAM region (letter d, a
  word read and all three widths written, on the suites' verdicts; the SPI host's
  letters f and r run blocks into it, [spi.md](spi.md)). Above them nothing did:
  reads of 0x3000 0000, 0x4000 8000, 0x4002 4000, 0x6000 0000
  and 0xA000 0000 each completed as an ORDINARY BLOCK, TCIF up, TEIF down, EN still
  set, with TEIE armed; the first three read zeros on both parts; on the
  CH32V303VCT6 the last two are the FSMC's bank 1 and its register block, and with
  the FSMC's gate closed each read the word 0x0FFF FFFF. Writes end the same way
  there (letter d): four items at each width into the same five holes, and into
  the image's own pattern in the flash - at the alias the image is linked at and
  at the array's own address, the FPEC locked as at every boot - all twenty-one
  ordinary completions on both parts, the vector entered for the completion alone
  and the pattern read back unchanged: the locked array drops the store in
  silence. So 11.2.1's "reserved address area" is narrower than the map's own word
  for it. No access by a channel faulted the core, and neither board wedged.
  Where one is raised (letter d, the three writes at 0x2FFF F000, ten runs a part)
  the first item is refused with three left, EN clear, the vector entered once, and
  the ledger keeps the channel's bus-master count until `stop()` gives it back; the
  copy engine's block into it ends with the fault counted, the engine free and no
  master left, and its next block runs whole. A refused store lands nowhere: six of
  them, each width at 0x2FFF F000 and at the first byte past the SRAM, raised the
  error and moved no word of the SRAM below the stack, SRAM+0 among them, on both
  parts' DMA1 and in a walk of the CH32V303VCT6's DMA2 as well; the CH32V00x
  controller does land such a store ([../ch32v00x/dma.md](../ch32v00x/dma.md)).
- **A circular channel reloads its count and its addresses by itself.**
  Eight compares poured into a timer by its own update request, four
  laps: four half flags and four complete ones, the count seen stepping
  down 28 to 30 times, the channel still enabled at the end and the
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
  channel given the head start, channel 1 finished at 3086 cycles and
  channel 2 at 6107 on the CH32V203C8T6, and at 3054 and 6060 to 6061 on the
  CH32V303VCT6 - channel 1 running at full rate and channel 2 getting
  what was left.
- **THE SLEEP, MEASURED IN BRIO'S OWN CODE** (letter c, three runs a part,
  identical). Four kilobytes that take 172 us awake moved 12 bytes across
  one `idle()` and 131 across ten on the CH32V203C8T6, 13 a wake at the
  1 kHz tick, and 18 and 145 on the CH32V303VCT6's DMA1, 14 a wake; then
  each block finished, byte for byte, the moment the core stayed awake.
  The vendor's own library, in a rig no suite here reproduces, moved nine
  to twelve bytes at the entry and eleven a wake on the CH32V203. Both
  say the same: the clocks run and the bus matrix serves the core alone.
  What sets the bytes a wake is not measured ("Not covered yet").
- **AND THE CH32V303's SECOND CONTROLLER STARVES THE SAME WAY**: the same
  four kilobytes on DMA2's first channel took 172 us awake and moved 20
  bytes across one `idle()` and 147 across ten, 14 a wake, then finished
  awake byte for byte.
- **All the vectors are the channels' own**: eight on the CH32V203C8T6,
  the eighth on that device class's tail, and on the CH32V303VCT6 DMA1's
  seven and DMA2's eleven - eighteen blocks, eighteen bodies, each
  running exactly once and each seeing its own armed flag and nothing
  else, the four channels of the extended register included.
- **DMA2's eleven channels copy out of flash as DMA1's do.** A kilobyte
  memory to memory on each, byte for byte, in 1747 to 1802 cycles:
  channels 1 to 7 reporting in DMA2_INTFR at 4 x (channel - 1), channels
  8 to 11 in DMA2_EXTEM_INTFR at 4 x (channel - 8), and INTFR's top
  nibble clear while those four ran.
- **DMA2's timer requests**: TIM5's, TIM6's and TIM7's updates each drew
  the same staircase of another timer's counter through DMA2's channels
  2, 3 and 4 - the rows of table 11-3, 503 ... 8003 for TIM5 and 502 ...
  8002 for the other two.
- **Both block engines run on DMA2**, each serviced from that
  controller's own handler: the loop engine played eight laps of eight
  compares into TIM5 on TIM5's update (channel 2) in 12800 us for 12800
  asked, with no fault, and the ping-pong engine, on TIM8's update
  (channel 1), filled four blocks of eight samples of a counter taken 200
  us apart (201, 401 ... 1601), with no overrun.
- **The 64 KB rule on the CH32V303VCT6**: DMA1 refused the span across
  0x0801 0000 and took its lower half; DMA2 moved the whole span byte for
  byte; and with the refusal bypassed DMA1 delivered the WRAP described
  above.
- **The transmit engine needs no listener**: twenty-six bytes left the
  ring in one block at 115200 baud, `tx_idle()` true 2348 to 2349 us after the
  run was queued, with no fault counted, on a pad with nothing attached.
- **THE STREAMS RUN AT THE RATE ASKED, on the core's counter.** A sender
  and the ring it feeds both divide PB1 by one arithmetic, so the
  streams of letters q and r - and the jumper's loop of letter g, the two
  controllers' pair of letter n - cannot tell a wrong bus rate by
  themselves: each letter times its ports' transmitters from the first
  queue to the last stop bit's end on the STK, which counts HCLK, in
  thousandths of the run's time at the rate ASKED, -1 % to +3 %. USART2's
  transmit engine, 256 frames at 115200: 1000 in letters g and p on both
  parts; UART4's on DMA2 into the far chip's sink, 256 frames at 115200:
  1000 in letter n on the CH32V303VCT6; the CH32V203C8T6's fourth port's
  bare engine pouring one 4 KB block: 1000 at 1 Mbaud and at 4.5 Mbaud
  (letters q and r), where on the CH32V303VCT6 the sender is the far chip,
  which STATES its rate - its PCLK1 over its divisor on its crystal, 0 ppm
  at both; the ring port's own transmit engine through letter q's
  transport, 1024 frames: 1000 at 1 Mbaud on both parts and 1001 at 4.5
  Mbaud on the CH32V203C8T6, 1002 on the CH32V303VCT6 -
  that transport's transmit ring is 64 bytes, so the run goes out in
  sixteen or more blocks, each started by the last one's completion
  vector, and the frames over are what that restart costs a block (not
  measured apart; the CH32V303's completion vector is a trampoline under
  ilp32f, [platform.md](platform.md)). TE's idle frame after an init
  (18.2) is out before a run is timed. Every rate here divides PB1 whole:
  the readings are the wire's, and a ruler that runs from the HSI as the
  buses do cannot see the HSI's own error ([usart.md](usart.md)).
- **THE RECEIVE RING, WITH NO WIRE** (letter g, the CH32V203C8T6). From
  `init()` the receive channel runs circular over the whole 128-byte
  storage, its count at 128 with nothing arrived and one bus master
  counted; stopped by `abandon()` it was running again from its first byte
  after the consumer's next look, the fault counted. The real thing does the
  same: the channel's memory address pointed by register at 0x2FFF F000 and a
  frame banged, the hardware dropped EN, the transport's vector counted one
  fault, and the next look ran the ring again from its first byte, the next 24
  bytes in order, ten runs on each part. Then bursts banged
  into the receive pad through its pull at 115200 baud, the core doing
  nothing else: 100 bytes into the empty ring read back in order, the
  vectors reporting the edge and `harvest()` asked after it answering
  false; 60 more ACROSS THE STORAGE'S END with no read between their
  bytes, all 60 in order and no overrun of either kind; 200 bytes into the ring with nobody reading,
  ONE overrun counted, nothing of the overwritten lap delivered and the
  ten bytes after it read whole; and a run of 14 held while 140 more
  landed, refused at its release, the second overrun counted. The
  CH32V303VCT6 gave every one of those numbers again, the ring on its
  DMA1 storage aligned to its own size.
- **THE RING AT SPEED** (letter o, the CH32V203C8T6): TIM2's update
  copying TIM3's counter into a ring of 256 half-words, read behind the
  channel through `HardwareRing` with every element judged against the
  one before it. At one update every 288 cycles, 131072 elements across
  512 laps in 231656 to 231657 looks, every step the pace's, nothing skipped,
  nothing torn - so no count ever ran ahead of its store - and the
  producer's position from `laps()` and the count, 131072, equal to the
  updates the pace made in the consumer's time. At one every 48 cycles,
  three million a second, 131072 to 131074 elements in 22957 looks, the same. A
  consumer stalling a lap and a half every 4096 elements lost exactly
  that: seven stalls, seven overruns counted and skipped, every step
  between them the pace's; four runs held across a lap and a half were
  each refused at their release. Read in a tight loop across laps the
  count reached 256 again at every wrap and never went above it, and at
  the fast pace a read could land on zero, between a lap's last
  decrement and the reload. On the CH32V303VCT6: 131072 elements in
  247383 to 247825 looks at the slow pace and 131072 to 131073 in 27203 to
  27238 at the fast one, every step the pace's, nothing skipped or torn,
  the position within one of the 131072 updates; seven stalls and seven
  overruns; four held runs refused; the count never above 256, and zero
  read one or two times in 100000 across 98 laps.
- **UART4'S ENGINES ON DMA2 ANSWER REAL REQUESTS** (letter n, the
  CH32V303VCT6): fifty-eight bytes out of the transmit engine on channel
  5, echoed by the far chip and back in through the receive engine on
  channel 3, byte for byte, no fault.
- **A standing request serves the wrong transfer**: a UART4 receiver left
  enabled with DMAR set made a later TIM6 staircase on the same DMA2
  channel move all sixteen items at once (the section above); with the
  port's block reset, the staircase ran at the timer's pace again. A
  staging of two owners, which the check refuses and no suite
  reproduces: a measured fact of the silicon, and why a channel has one.
- **A held request is answered** (`test_vx03_i2c` letter q,
  `test_vx03_dma_serial` letter p, on the CH32V303VCT6 and the
  CH32V203C8T6 alike): a request another peripheral raised on a
  transport's channel and kept behind its closed gate - staged by
  register, the channel given to no one else - meets each transport
  without a storm. TIM3's channel 1 compare with CC1DE, matched once and
  its gate closed without a reset, moved one item of an I2C1 write on
  channel 6 and stalled it (count 31 of 32, the target holding one byte),
  the write answered `i2c_dma_fault`; a read over channel 7 held by
  USART2's transmit request entered the event vector for the START and
  the address alone, the tenure answered by the caller's bound and
  `recover()`; USART2's receive ring over channel 6 held by TIM3 gave the
  channel up within its bound, one DMA fault and no storm. With the
  holder's reset line pulsed, gate closed, each was let go and the same
  write, read and burst landed whole.
- **THE SAME OWNER OVER ITS OWN HELD REQUEST** (`test_vx03_dma_serial`
  letter s and `test_vx03_i2c` letter s, on the CH32V303VCT6 and the
  CH32V203C8T6 alike, ten runs each): the case the reset at `init()`
  answers, each owner released with its DMA enable left set and its gate
  closed. USART2's ring with its channel stopped and one frame banged
  into the pad (STATR 0xE0, RXNE standing, CTLR3 0xC1, DMAR set), the port
  released and initialized again: nothing in the ring, then the sixteen
  bytes banged after it, the burst's own in order. TIM1's update raised
  through 100 us periods with channel 5 disabled, the timer released: its
  own paced block moved nothing before the pace and sixteen samples a
  period apart, its count at zero. I2C1's read stopped under its block
  (STAR1 0x44, RxNE and BTF; CTLR2 0x1930, DMAEN) and the host released:
  the same host initialized again read sixteen consecutive values on
  channel 7, its count at zero. The CONTROL, these images built with the
  pulse taken out of the three `init()`s, three runs each on both parts,
  shows each request: the ring holds the stale 0xA5 before a byte is
  banged and 17 bytes after the burst, the paced block moves one item
  before the pace and has fourteen steps of a period, and the I2C read never
  reaches its channel: the event vector storms, STAR1 reading SB alone,
  until the suite's guard silences it at 60001 entries, SCL and SDA low,
  channel 7 still holding the abandoned block's count of 23. The SPI's `init()` pulses
  as well; its request does not outlive its release, so no such control
  exists there ("A released requester" above).
- **THE PRIORITY RULE ON THE BENCH** (letter q, both parts): 16 KB at 1
  Mbaud and 64 KB at 4.5 Mbaud poured into a ring of 512 while a 2 KB copy
  ran back to back on a lower channel at the copy engine's default level
  - on the CH32V203C8T6 the fourth port's transmitter, fed by a bare
  engine, into USART2's ring on DMA1's channel 6 beside a copy on channel
  3; on the CH32V303VCT6 the far chip's blast into UART4's ring on DMA2's
  channel 3 beside a copy on DMA2's channel 1 - the ring at very_high, the
  transport's default: every byte in order, no overrun of either kind, no
  channel given up, the copy at 0.96 to 1.00 of its rate alone at 1 Mbaud
  and 0.92 to 0.98 at 4.5 Mbaud (the lower figures the CH32V203C8T6's,
  whose sender shares DMA1 with the ring).
  The CONTROL, the ring told `rx_priority = low` - every engine at the
  lowest level, channel 3 winning
  every tie against 6 - lost about seven bytes in eight to some nine
  hundred overruns at 1 Mbaud. The CFGR read back carries the levels: PL
  3 for the ring, 2 for its transmit channel after its first block, 0
  for the ring told low.
- **AN OVERRUN AT A MESSAGE'S TAIL** (letter r, both parts; on the
  CH32V303VCT6 the far chip's messages into UART4's ring on DMA2, the
  staged frame's copy started on its start bit's fall, read off the pad):
  a copy at the ring's own level on a lower channel, the vector held off until the
  channel took the frame DATAR held - no storm, three entries, the
  overrun counted once - and a frame read by the CPU from under a starved
  channel still arrived through the ring, the request it raised served
  when the copy ended ([usart.md](usart.md)).

### The engines' costs

`bench_vx03`'s letter d on the CH32V203C8T6 at 144 MHz, the core's
counter the ruler, every number in HCLK cycles and the instrument's own
cost included as the line carries it (a stopwatch is 28 cycles, a stamp
pair in a vector 79 - the app's letter r). The SPI rows carry the
transaction's copy of the 48-byte `Request` into the host, which is the bus
contract's and not the engines', and which the letter does not print apart.
Three runs of the letter printed the same figures.

| operation | cycles |
|---|---|
| copy, 4096 bytes as 1024 words: wall | 6389 |
| copy: launch, completion's handler, fixed cost | 112, 84, 245 |
| paced, 256 words at 100 kHz into one cell: launch, handler | 82, 81 |
| SPI1 engined write, 8-bit frames at /4: launch, interrupts, handlers, fixed cost | 329, 1, 147, 517 |
| SPI1 engined write at /16: fixed cost | 509 |
| SPI1, 16-bit frames at /4: interrupts for 256 frames, fixed cost | 1, 526 |
| bus masters still counted after the SPI requests | 0 |

- **Six cycles an item, by the difference of two sizes**: (wall at 4096
  less wall at 256) over 960 words is 6.00 exactly - the
  controller's own rate, independent of the engine; at 4096 bytes a copy
  moves 92 MB/s, against the 576 of one word a cycle (x = 6.2), and the
  runtime's memcpy does the same 4096 bytes in about 3130 cycles, half the
  DMA's time ([../design/benchmark.md](../design/benchmark.md)): the
  engine frees the core, it does not outrun it.
- **The fixed cost of a copy is 245 cycles** of wall above the moving, of
  which the instrument is 107 (the stopwatch and the vector's stamp pair):
  138 cycles of the engine's own - the launch, five stores and the
  ledger's count, and the completion's handler.
- **The SPI transaction's fixed cost is 517 cycles**, with one interrupt.
  Less the instrument (28 + one stamp pair of 79) that leaves 410: the
  request's copy and what the engines cost a transaction - their launch, the
  receive channel's one completion, both channels stopped. The per-frame
  cost is the wire's: 256 frames at /4 are 8192 cycles of SCK and the
  block adds nothing per frame.
- **A 16-bit frame is one 16-bit access**: a 256-frame write of half-words
  takes 16910 cycles for 16384 of wire, with one interrupt, and the frames
  are the buffer's, judged with no wire by the SPI's own transmit CRC
  ([spi.md](spi.md)).
- **The pace holds to the cycle**: the 256 words of a 1440-cycle pace
  arrived 1436 to 1447 cycles apart, watched by the core under the mask on
  the counter's position - the watching loop's own turn is the spread -
  and the whole block took 368973 cycles for 368640 due, the first
  update's period and the completion's latency the difference.
- **A one-frame engined SPI request can complete inside the launch**: the
  receive channel takes its one frame before the launch's last store, so a
  request raised by that store would stand after the completion has dropped
  it, and the next request's transmit block would complete with nothing
  clocked and its receive block never start. The launch programs both
  channels first and raises both requests in one store after them, the
  completion's store always the later: a one-frame request runs ahead of
  every point of the letter, and no point stalls.

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

- **The priority rule's reach below 1 Mbaud and under paced loads.**
  Letter q measures the rule at 1 and 4.5 Mbaud against an unpaced copy
  (the bench finding "The priority rule on the bench"). Two claims behind
  the rule ([design/dma.md](../design/dma.md)) are not a suite's
  measurement: that a ring armed at the lowest level loses data from
  115200 baud up as soon as one block of a copy on a lower channel
  outlasts a character time, its receive vector taking most of the core
  while the channel is starved; and that paced loads - an SPI host's two
  engines at 36 MHz, a timer stream at 1 and 4 MHz - starve no ring at
  any level. What would measure them is letter q's control run at
  115200 with a copy block of 4 KB, and letter q with SPI2's two engines
  and a timer stream in the copy's place, each judged on the ring's bytes
  and overrun count.
- **What sets the bytes a sleep moves.** Letter c reads 13 and 14 bytes a
  wake on the two parts, and a vendor's-library rig read eleven on the
  CH32V203; the difference is not explained, and nothing here says how much
  of it is the wake handler's length, the wake latency or the item in
  flight. What would measure it is letter c with a wake handler of stated
  lengths, beside the vendor's rig rebuilt on the same die.
- **The reset at `init()` on the transports letter s does not stage**:
  the ADC's and the DAC's streams, and UART4's two requests on DMA2, keep
  the contract by reading, no run having left a request standing over
  them. UART4's transmit request is measured held on channel 5; its receive
  side wants a frame on its pad, both of that port's columns being the desk
  pair's wires. What would measure them is letter s's stage on each:
  a converter released with DMA set and a conversion unread, a DAC with its
  trigger pending, and on UART4 the far chip's frame, each initialized again
  and its next block read back.
- **The CH32V203's serial round trip on one pair of pads.** The engines
  themselves carry a wire's data in the two bus chapters - sixteen bytes
  each way through SPI2's pair and a tenure's shapes through I2C1's,
  byte-exact against a peer board ([spi.md](spi.md), [i2c.md](i2c.md)) -
  and USART3's and UART4's receive rings take the far chip's blast at 4.5
  Mbaud ([usart.md](usart.md)). What has no listener on the CH32V203 is USART2's
  pair to itself: a strap between PA2 and PA3 is what the suite's letter
  g detects and what would close it in one run.
- **The parts other than the CH32V203C8 and the CH32V303VC.** The request
  table folds through each part's own instances and the whole stratum
  compiles for all thirteen, both ways the hardware prologue can be built
  (`brio check ch32vx03`); the CH32V203RB's TIM5 rows, the smallest
  parts' absent ones and the 128 KB CH32V303's missing TIM5 and TIM8 are
  asserted at compile time and measured on none of them. What would
  measure them is a board - and for the CH32V303's DMA1, a die of a lot
  the 64 KB note does not restrict, on which the refusal the driver keeps
  would be a caution and not a necessity.
