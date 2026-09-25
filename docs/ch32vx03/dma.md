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
Reference suite: `test_vx03_dma`.

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
MECHANISM and not a rule the programmer keeps: `DmaChannel::enable()`
counts the EN transition as one active bus master ([sleep.md](sleep.md))
on either controller, the kernel's idle path does not sleep while the
count stands and a sleep site refuses to arm over it. `any_enabled()` is
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

`DmaTxEngine<c, ch, Elem>` pours a caller-owned run into one peripheral
register (`arm(data)`, `start(buffer, n)`, `start_fixed(cell, n)` for a
full-duplex bus clocking a read, `complete()` handing back how many
elements the block carried so the owner releases exactly that much of its
ring, `kick()` starting a run again from its beginning, `abandon()`,
`faults()`, `progress()`) and `DmaRxEngine<c, ch, Elem>` fills one from a
register (`start(buffer, n)`, `start_discard(cell, n)`, `take()` - how
many arrived since last asked, one CNTR read, nothing suspended -,
`harvest()` - the same arithmetic without consuming it -, `idle()`,
`full()`, `kick()`, `abandon()`). The element type is the width: 1, 2 or
4 bytes, anything else refused. Both publish `present`, `controller`,
`channel`, `slot`, `service()` (the channel's ISR body) and the flag
names, and a transport reaches its engine only through those.

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
ring by contiguous runs and the receive engine fills the RX ring's free
run. The two bus engines take the same pair in the same place -
`SpiHost` carries a block's data phase on them ([spi.md](spi.md)) and
`I2cHost` a tenure's ([i2c.md](i2c.md)).
`harvest()` publishes what arrived and `dma_isr()` is the ISR body
of whichever channels the transport owns; `dma_faults()` counts the
blocks thrown away. An engine is REFUSED on any slot but the instance's
own, controller and channel, which the request table answers; each
resource publishes its two slots as `dma_tx_slot` and `dma_rx_slot`.
Without an engine every branch is compiled out and `init()` does not so
much as touch CTLR3.

`harvest()` is a VERB, not an interrupt: a receive block completes only
when its run fills, which on an idle line is never, so whoever owns the
port decides how often to ask. With a receive engine RXNE belongs to the
channel, so the error flags are read once per harvest and counted
against the RUN and not the byte - a console that wants exact
attribution takes no receive engine.

## How to use it

A memory-to-memory block, polled:

```cpp
using Copier = brio::DmaChannel<1, 1>;

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
returns. A DMA2 channel is the same with the controller's number first,
`DmaChannel<2, 9>`, and its handler `dma2_channel9_handler`.

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

A console on both engines - USART2 transmits on DMA1's channel 7 and
receives on its 6:

```cpp
using Tx = brio::DmaRequestOf<brio::DmaRequest::usart2_tx>;
using Rx = brio::DmaRequestOf<brio::DmaRequest::usart2_rx>;
using Serial = brio::Uart<2, P, 128, 128, brio::UartFormat{},
                          brio::DmaTxEngine<Tx::controller, Tx::channel>,
                          brio::DmaRxEngine<Rx::controller, Rx::channel>>;

extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { (void)Serial::isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() { (void)Serial::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel6_handler() { (void)Serial::dma_isr(); }

// in the loop, or from a TimeEvent every few ticks:
if (Serial::harvest()) { /* the RX ring went from empty to non-empty */ }
```

The CH32V303's UART4 is the same spelling, and the table puts both of
its engines on DMA2 - the handlers are then `dma2_channel5_handler` and
`dma2_channel3_handler`. With an engine on each side the port arms no
interrupt of its own, and `release()` is what takes its requests away
from the two channels when the port is done with them.

## Bench findings

`test_vx03_dma` measures on both parts at 144 MHz, the source of every
copy a four-kilobyte pattern in the image itself and the core's own
counter the ruler. On the CH32V203C8T6 the eight letters of a part with
one controller run with the board bare: **25 verdicts in `z`**. On the
CH32V303VCT6 all fourteen run, DMA1's eight and DMA2's six, with the
evaluation board's two crossed wires in place (PA2 to PC11, PC10 to PA3):
**43 pass, 0 fail**, twice. Where one number is given below it is both
parts'; where they differ each is named.

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
  ring in one block in 2327 us at 115200 baud (ten bits a byte is 2257
  us of line time), with no fault counted, on a pad with nothing
  attached. A receive run armed on a quiet line stands open - 127 items
  of the ring's free span, none of them filled - until `abandon()` ends
  it, which hands the channel back and counts the fault.
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
  error and their fault counters - is written from 11.2.1 and provoked
  by none of the five addresses the suite reads from, on either part. A
  peripheral that raises it, or a write into flash (which 11.1 lists as a
  legal destination and the bench has not tried), would measure it.
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
