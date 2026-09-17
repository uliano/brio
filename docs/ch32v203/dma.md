# DMA (CH32V203)

One controller with EIGHT channels, each wired to a fixed handful of
peripheral requests, an arbiter over four software priorities with the
channel index deciding ties, and four flag bits a channel - the
STM32F1's DMA1 under WCH's register names, with one channel more than
that ancestor. Documents of record: the CH32F/V20x_V30x_V31x reference
manual V2.3 (11.1 for what a channel may reach, 11.2.1 for the channel
and its three outcomes, 11.2.2 and table 11-1 for the widths and their
alignment, 11.2.3 with table 11-5 for the request map of our device
class and table 11-6 for the other one, 11.3 for the registers, 3.4.6
for the gate and 3.4.11 for the reset register that has no bit for this
block, 9.5.1 and table 9-2 for the vectors) and the CH32V203 datasheet
V2.8 (table 2-1 for which peripherals a part offers, and therefore which
requests it can raise). Ours is the CH32V20x_D6 device class for every
part up to the CH32V203C8 and CH32V20x_D8 for the CH32V203RB; both have
eight channels, and the register notes of 11.3.1..11.3.6 say so by
name. Driver: [brio/ch32v203/dma.hpp](../../brio/ch32v203/dma.hpp) with
the request table and the empty slot's tag in
[brio/ch32v203/dma_engine.hpp](../../brio/ch32v203/dma_engine.hpp).
Reference suite: `test_v203_dma`.

## What the silicon does

### The channel is the request

There is no request multiplexer. Table 11-5 wires each peripheral event
to ONE channel, and an engine is named by channel number with that table
in hand: the converter on 1, SPI1 receiving on 2 and transmitting on 3,
SPI2 on 4 and 5, USART1 transmitting on 4 and receiving on 5, USART2 the
other way round on 6 and 7, USART3 on 2 and 3, UART4 on 1 and 8, I2C1 on
6 and 7, I2C2 on 4 and 5, and the timers' updates, captures, triggers
and commutations spread across all eight. The table is the same for both
device classes: the CH32V20x_D8 adds TIM5's six rows (table 11-6) and
changes nothing else.

WHICH ROWS EXIST IS A PART FACT, not a family one. A count is not a
list: the smallest package offers one usart and it is USART2, one SPI
and no I2C at all, so on it the rows for USART1, USART3, UART4, SPI2,
I2C1 and I2C2 name a request nothing can raise. The driver folds that
through the part table and answers 0 - "no channel" - for such a row.

A timer publishes several events and they do not share a channel: an
update is not a capture. TIM1's fourth channel, its trigger and its
commutation ARE one row, because they raise the same request.

THE MANUAL'S SECOND CONTROLLER IS NOT THIS FAMILY'S. Chapter 11 also
describes a DMA2 of eleven channels with two flag registers of its own
(tables 11-3, 11-4 and 11-8), and 11.2.3 gives it to the V4F and
Cortex-M3 classes alone - the paragraph that introduces our eight
channels names one controller. RCC_HBPCENR declares a DMA2EN beside
DMA1EN with no class note of its own (3.4.6), which is exactly the kind
of bit a device header would let a driver believe in; nothing of DMA2 is
spelled here. The 64-kilobyte-boundary caution of 11.2.3 belongs to
those classes too: it is keyed on lot numbers of parts whose RAM reaches
past that boundary, which no part of this series has.

### A channel

CFGR (direction, circular, memory-to-memory, the two increments, the two
widths, the priority, three interrupt enables and EN), CNTR counting down
as items move, and PADDR and MADDR - where DIR says which side is the
source, and in memory-to-memory mode both are memory and the names mean
nothing. Every one of those fields is read-only while EN is set (11.2.1's
note and each register's own), and a store into an enabled channel is
dropped in silence.

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
- **Flash is a legal source** (11.1 lists flash, SRAM, the peripheral
  SRAM and all three peripheral buses), which is what lets a program
  copy out of its own image with no RAM cost.

### The flags, the vectors and the gate

- **Four flag bits a channel** in INTFR, read-only - GIF, TCIF, HTIF,
  TEIF at 4 x (channel - 1) - cleared by writing one to the same
  position in INTFCR.
- **The half flag rises when fewer than half the items are left**
  (11.2.1), measured below.
- **EIGHT vectors, one per channel.** Seven are consecutive from the
  vector table's entry 27; the eighth sits on the tail this DEVICE CLASS
  has - entry 62 here, 67 on the CH32V203RB, whose table carries the
  Ethernet pair before it. So a channel's handler reads only its own
  flags and needs no "which channel" question.
- **The block's gate, DMA1EN in RCC_HBPCENR, is closed at reset** (the
  register's reset value opens the SRAM alone).
- **THIS BLOCK HAS NO RESET LINE.** RCC_AHBRSTR (3.4.11) reserves bits
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
([README.md](README.md)).

So on this family A DMA-FED TRANSPORT DOES NOT SLEEP. The driver states
it and gives the power chapter the question it will have to ask:
`Dma::any_enabled()` is true while any channel has EN set, and a sleep
site is expected to refuse a mode while it is - or the transport that
owns the engine holds a standing lock. The alternative the same
measurements opened is to slow down instead: the chip carries data with
HCLK as low as 24 MHz.

## Types and verbs

### The request table

[brio/ch32v203/dma_engine.hpp](../../brio/ch32v203/dma_engine.hpp) holds
what a TRANSPORT needs without including the controller: `NoDmaEngine`,
the empty slot's tag whose only member is `present`; `dma_engine_channel`
and `dma_engines_distinct`, the two checks a task makes on a named
engine; and the table itself - `DmaRequest`, one enumerator per row of
tables 11-5 and 11-6, `dma_request_channel` folding the part (0 where
the part has not got the peripheral), `dma_request_present`, and
`DmaRequestOf<r>::channel`, the compile-time form that REFUSES a request
this part cannot raise. A driver with an engine slot includes this file
alone, so a program with a console does not carry the controller.

### The block

`Dma`: `open()` (the gate, which every channel verb calls first),
`opened()`, `flags()` and `clear()` over the whole register,
`stop_all()` - every channel back to the chapter's own values, the work
the missing reset line leaves to software - and `any_enabled()`, the
power model's one question.

### The channel

`DmaChannel<ch>`, 1..8: `configure(DmaChannelConfig)` and
`configuration()` reading it back, `set_count()`, `set_peripheral()` /
`set_memory()`, `prepare()` and `load()` over a `DmaTransfer` (both ends,
a count, a config), `trigger()` (a prepared channel started again),
`enable()`, `remaining()` live, the flags (`flags()`, `flag(mask)`,
`clear(mask)` over `DmaFlag`'s four bits), `arm(mask, on)` and `armed()`
for the three interrupt enables, `isr()` - the ISR body that reads only
the ARMED flags that are up, clears exactly those with the global bit
and hands them back, deciding nothing -, `progress()`, `irq()` and
`stop()`.

EVERY CONFIGURING VERB REFUSES while the channel is enabled, and every
verb that takes an address refuses one that is not aligned to its own
width: a store the silicon ignores, or an address it rounds down, would
be a lie the code told itself. `dma_channel_config_valid` and
`dma_transfer_valid` are the compile-time half of that, and
`dma_transfer_aligned` the half only an address can answer.

### The engines

`DmaTxEngine<ch, Elem>` pours a caller-owned run into one peripheral
register (`arm(data)`, `start(buffer, n)`, `start_fixed(cell, n)` for a
full-duplex bus clocking a read, `complete()` handing back how many
elements the block carried so the owner releases exactly that much of its
ring, `kick()` starting a run again from its beginning, `abandon()`,
`faults()`, `progress()`) and `DmaRxEngine<ch, Elem>` fills one from a
register (`start(buffer, n)`, `start_discard(cell, n)`, `take()` - how
many arrived since last asked, one CNTR read, nothing suspended -,
`harvest()` - the same arithmetic without consuming it -, `idle()`,
`full()`, `kick()`, `abandon()`). The element type is the width: 1, 2 or
4 bytes, anything else refused. Both publish `present`, `channel`,
`service()` (the channel's ISR body) and the flag names, and a transport
reaches its engine only through those.

TWO ENGINES OF ONE TRANSPORT NAME TWO CHANNELS: a channel moves data one
way, and the table gives each direction its own.

### The transport's two slots

[brio/ch32v203/usart.hpp](../../brio/ch32v203/usart.hpp)'s `Uart` takes
a transmit engine and a receive engine as its last two template
parameters, `NoDmaEngine` by default: the transmit engine drains the TX
ring by contiguous runs and the receive engine fills the RX ring's free
run. `harvest()` publishes what arrived and `dma_isr()` is the ISR body
of whichever channels the transport owns; `dma_faults()` counts the
blocks thrown away. An engine is REFUSED on any channel but the
instance's own, which the request table answers. Without an engine every
branch is compiled out and `init()` does not so much as touch CTLR3.

`harvest()` is a VERB, not an interrupt: a receive block completes only
when its run fills, which on an idle line is never, so whoever owns the
port decides how often to ask. With a receive engine RXNE belongs to the
channel, so the error flags are read once per harvest and counted
against the RUN and not the byte - a console that wants exact
attribution takes no receive engine.

## How to use it

A memory-to-memory block, polled:

```cpp
using Copier = brio::DmaChannel<1>;

Copier::stop();                       // EN stays set after a block: clear it first
(void)Copier::load(brio::DmaTransfer{
    .peripheral = src, .memory = dst, .count = 1024,
    .config = {.memory_to_memory = true, .peripheral_increment = true,
               .peripheral_width = brio::DmaWidth::word,
               .memory_width = brio::DmaWidth::word}});
while (!Copier::flag(brio::DmaFlag::complete)) { }
Copier::stop();
```

A channel's interrupt: `Copier::arm(DmaFlag::complete | DmaFlag::error,
true)`, `Pfic::enable(Copier::irq())`, and a handler on
`dma1_channel1_handler` calling `Copier::isr()` and acting on the bits it
returns.

A peripheral request, named rather than numbered - a timer's update
moving one sample of another timer's counter per period:

```cpp
using Sampler = brio::DmaChannel<brio::DmaRequestOf<brio::DmaRequest::tim2_up>::channel>;

(void)Sampler::load(brio::DmaTransfer{
    .peripheral = brio::Tim<3>::cnt_address(), .memory = buffer, .count = 16,
    .config = {.peripheral_width = brio::DmaWidth::half,
               .memory_width = brio::DmaWidth::half}});
brio::Tim<2>::interrupts(brio::tim_ude, true);   // the request enable
brio::Tim<2>::enable(true);
```

A console on both engines - USART2 transmits on channel 7 and receives
on 6:

```cpp
using Serial = brio::Uart<2, P, 128, 128, brio::UartFormat{},
                          brio::DmaTxEngine<brio::DmaRequestOf<brio::DmaRequest::usart2_tx>::channel>,
                          brio::DmaRxEngine<brio::DmaRequestOf<brio::DmaRequest::usart2_rx>::channel>>;

extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { (void)Serial::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() { (void)Serial::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() { (void)Serial::dma_isr(); }

// in the loop, or from a TimeEvent every few ticks:
if (Serial::harvest()) { /* the RX ring went from empty to non-empty */ }
```

## Bench findings

`test_v203_dma`, 25 verdicts in `z`, on the CH32V203C8T6 at 144 MHz with
the board bare - no pad is wired, the source of every copy is a
four-kilobyte pattern in the image itself, and the core's own counter is
the ruler.

- **Six cycles an item at every width, and the width is the rate.** Four
  kilobytes copied flash to RAM take 24809 cycles as 4096 bytes, 12530
  as 2048 half-words and 6392 as 1024 words - 6.05, 6.11 and 6.24 cycles
  an item. At 144 MHz that is 23.8, 47.1 and 92.3 megabytes a second,
  and every byte of all three arrived.
- **The half flag rose with 2049 of 4096 moved**, which is 11.2.1's own
  rule read back; INTFR stood at 0x7 (the global, complete and half bits
  of the channel) and one write to INTFCR took all four down.
- **EN stays set after a completed block** - CNTR at zero, TCIF up, the
  channel enabled and idle - and while it is set every configuring verb
  of the driver refuses: the configuration, the count, a whole transfer
  and an address, with CFGR unchanged across all four.
- **NO ADDRESS THE BENCH COULD NAME RAISES A TRANSFER ERROR.** Reads of
  0x3000 0000, 0x4000 8000, 0x4002 4000, 0x6000 0000 and 0xA000 0000 -
  five addresses outside every window of the map - each completed as an
  ORDINARY BLOCK with zeros: TCIF up, TEIF down, EN still set, with TEIE
  armed on every one of them. 11.2.1 promises a transfer error and an
  automatic drop of EN for a reserved area, and none of the five
  produced either. The same is true of the CH32V00x's controller.
- **A circular channel reloads its count and its addresses by itself.**
  Eight compares poured into a timer by its own update request, four
  laps: four half flags and four complete ones, the count seen stepping
  down twenty-eight times, the channel still enabled at the end and the
  last value of the run standing in the timer's register. Thirty-two
  updates of 200 us took 6400 us, so the REQUEST and not the bus paces
  the channel.
- **A timer's update request needs no pad and no core**: sixteen samples
  of a free-running one-megahertz counter, taken 500 us apart, came out
  503, 1003, 1503 ... 8003 - a staircase whose step is the period, every
  one of the fifteen within one per cent.
- **The burst engine walks four registers per request**: with DBA at
  CH1CVR and DBL at four, a run of four half-words written through the
  single address TIMx_DMAADR landed as 111, 222, 333, 444 in CH1CVR
  through CH4CVR.
- **The software priority outranks the channel number, and with the
  priorities equal the lower number takes the bus outright.** Two
  channels of equal work: the one that started SECOND and asked for the
  very-high priority completed first. With both at medium and the higher
  channel given the head start, channel 1 finished at 3106 cycles and
  channel 2 at 6151 - which is channel 1 running at full rate and
  channel 2 getting what was left.
- **THE SLEEP, MEASURED IN BRIO'S OWN CODE.** Four kilobytes that take
  172 us awake moved 4 bytes across one `idle()` and 76 bytes across ten
  - seven bytes a wake at the 1 kHz tick - and then finished, byte for
  byte, the moment the core stayed awake. The vendor's own rig put the
  same number at nine to twelve bytes at the entry and eleven a wake;
  this platform's idle is a WFE-shaped `wfi` and its wake handler is
  smaller, which is the difference. Either way the conclusion is the
  same: the clocks run and the bus matrix serves the core alone.
- **All eight channels have a vector of their own**, the eighth on this
  device class's tail: eight blocks, eight bodies, each running exactly
  once and each seeing the armed flag and nothing else.
- **The transmit engine needs no listener**: twenty-six bytes left the
  ring in one block in 2327 us at 115200 baud (ten bits a byte is 2257
  us of line time), with no fault counted, on a pad with nothing
  attached. A receive run armed on a quiet line stands open - 127 items
  of the ring's free span, none of them filled - until `abandon()` ends
  it, which hands the channel back and counts the fault.

## Not covered yet

Driver gaps, each with its reason:

- **The block engines** - `DmaLoopEngine` and `DmaPingPongEngine`, the
  two ARMv6-M strata's realizations of
  [util/block_stream.hpp](../../brio/util/block_stream.hpp)'s concepts.
  This controller has a hardware circular mode, which is the player's
  shape on the STM32G0, and the source's two halves would be the same
  design; they are born with their first block user, the ADC, and
  measuring them before there is a stream to judge them against would
  fix the shape against the easy case.
- **The engine slots of every transport but the serial one.** SPI and
  I2C have their rows in table 11-5 and their drivers do not exist on
  this stratum yet; each gets its two slots when its chapter is written.
- **The peripherals whose requests exist and have no driver here yet**:
  the converter's row (channel 1), SPI1's and SPI2's four, I2C1's and
  I2C2's four. The table names them and `DmaRequestOf` will hand out
  their channels; what is missing is the peripheral driver on the other
  end, not the channel.
- **Half-transfer as a verb of the engines**: the flag and its enable
  are the channel's (`DmaFlag::half`, `arm()`), and no engine acts on
  the midpoint until a stream needs it.

Implemented but not bench-verified, each with what would measure it:

- **The transfer-error path** - TEIE, the engines' `abandon()` on an
  error and their fault counters - is written from 11.2.1 and provoked
  by none of the five addresses the suite reads from. A peripheral that
  raises it, or a write into flash (which 11.1 lists as a legal
  destination and the bench has not tried), would measure it.
- **The receive engine on a wire.** The transmit half of the transport
  and the standing-run half of the receive one are measured with the
  board bare; what has no listener is the round trip. A strap between
  PA2 and PA3 - USART2's own two pads - is what the suite's letter g
  detects and what would close it in one run.
- **Widening and truncation between unequal widths** (table 11-1): the
  channel takes two widths and every transfer above sets them equal. A
  copy with PSIZE and MSIZE apart, compared against the table's rows,
  is what would measure it.
- **The eight parts other than the CH32V203C8.** The request table folds
  through each part's own instances and the whole stratum compiles for
  all nine both ways the hardware prologue can be built (`brio check
  ch32v203`); the CH32V203RB's TIM5 rows and the smallest parts' absent
  ones are asserted at compile time and measured on none of them. What
  would measure them is a board.
