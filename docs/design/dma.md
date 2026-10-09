# DMA engines: what a program is owed

What every family's DMA engines owe the program above them, by intent,
whatever their controller: the LEVEL each engine arms at in its
controller's arbitration, decided by what its peripheral does when the
channel is starved; the one knob a program has over the level that
matters most, the receive ring's; and the RELEASE CONTRACT - a
peripheral done with a channel leaves no request behind for the next
owner. How a controller is driven - its registers, its engines' two
moments, its costs - is each family's chapter document; this page is
the part of it a program can rely on unchanged from one family to the
next.

## The priority rule

On a controller whose channels arbitrate by a software level, the level
is not a tuning knob left at its reset value: it is decided by what the
channel's PERIPHERAL does when the channel is starved.

- **An engine whose peripheral OVERRUNS when starved arms at the
  HIGHEST level**: a receive ring, an SPI host's receive, a converter's
  stream. Their data registers hold one item, or a FIFO a few items
  deep, the next item arrives on the wire's or the converter's clock
  whether the channel came or not, and an item the channel did not take
  in time is lost - with an overrun flag at best, and in silence on a
  converter that has none.
- **A transmit or paced output engine arms at the NEXT level**: a UART
  or SPI transmit, a DAC or timer stream. A starved transmit leaves the
  line idle a while and loses nothing; a host's clock waits between two
  frames; a paced output holds its last value a period. It underruns, or
  only slows. A receive that cannot overrun belongs here too, and for
  the same reason: an I2C host stretches the clock while its data
  register is full, so a starved channel slows the bus and loses nothing.
- **A memory-to-memory copy arms at the LOWEST level.** No request paces
  it: it asks for the bus on every cycle of its block, so a channel it
  outranks moves nothing until the block is over, and a copy is the one
  load that can starve a ring for longer than a character time.
- **Within a level the silicon's own tie rule stands** - on the
  STM32F1-lineage controllers the lower channel number wins - and the
  rule does not try to number channels.

THE MEASUREMENT behind it, on the CH32V303VC and the CH32V203C8 (the
receive ring USART2's, DMA1's channel 6): with every engine at the
lowest level - the channel NUMBER deciding, the ring losing to channels
1 to 5 - an unpaced copy on a lower channel lost data at 115200 baud and
above as soon as one block lasted longer than a character time, and in
a stream the ring's receive vector took most of the core while it was
starved; paced loads (an SPI host's two engines at 36 MHz, a timer
stream at 1 and 4 MHz) never starved anything. With the ring at the
highest level the same 186 loaded cells were clean - no overrun, no
channel given up, the longest wait one entry - and the copy took 1.6 %
longer. The reference suites carry the rule: `test_vx03_dma`'s letter q
streams 16 KB at 1 Mbaud and 64 KB at 4.5 Mbaud into the ring while a
2 KB copy runs back to back on a lower channel at the copy's level,
every byte delivered on both parts, and the same load against a ring a
program armed at the lowest level overruns it some nine hundred times
in 8 KB ([../ch32vx03/dma.md](../ch32vx03/dma.md)).

WHAT THE RULE COSTS: the items the ring takes from the copy, and no
more. The copy beside a stream at 1 Mbaud ran at 0.96 to 1.00 of its
rate alone, at 4.5 Mbaud at 0.92 to 0.98 - the lower figures where the
stream's sender shared the controller.

A CONTROLLER THAT ARBITRATES DIFFERENTLY still takes the rule. The
STM32G0's controller alternates a memory-to-memory channel with any
other requester (RM0444 10.4.4), so ONE copy of any length never starved
its ring - but two copies alternated with each other and did, and there
the receive FIFO did not absorb the starvation, it wedged the receiver;
the ring at the highest level cured both. The RP2040's arbiter serves
one low-priority channel every round, and no arrangement of copies
starved a low receive channel; its HIGH_PRIORITY bit for the receive
costs nothing measurable there. The rule is the same sentence on every
family; what the level buys depends on the arbiter, and is measured per
family.

### The one knob: the receive ring's level

A program can rank a channel above its serial receive ring - a copy
that must not wait, a second ring it values more - and it says so on
the Uart, in ONE SHAPE on every family: `UartOptions::rx_priority`, a
value of the family's own `DmaPriority` (its levels the silicon's: four
on the STM32-lineage and the SAM's controllers, a bit on the RP
families), defaulting to the family's highest. A Uart with no receive
engine refuses the option at compile time: there is no channel to rank.
Every other engine's level is its driver's, stated beside the call with
the chapter's reason, and an engine armed by hand takes the rule's level
for its direction by default.

## The release contract

A peripheral's DMA request, once raised, is HELD until the controller
acknowledges it or the peripheral's reset line is pulsed - on the
CH32V303 and the CH32V203 clearing its DMA enable, its block enable or a
timer's counter does not withdraw it, and a gated clock freezes it on
the channel's ORed request line. The next owner of the channel then
moves one stray item on it at its first enable and stalls behind it, or,
with the old requester still clocked, gets its stream shifted by one
([../ch32vx03/dma.md](../ch32vx03/dma.md), "A released requester").

So a transport's `release()` resets its peripheral BEFORE it gates its
clock, wherever its block can hold a request, and the incoming owner
needs nothing beyond its engine's `arm()`: the outgoing owner's release
is the whole of the hand-over, because the incoming one cannot tell which
requester holds the line, and its own enable is what would consume the
stray. A block measured not to hold a request (SPI1 on the CH32V203 and
CH32V303 family) says so in its driver and needs no pulse.

### Realizations

Common: on the WCH strata every engine's level is the rule's - the
defaults by direction, each driver's call naming its own - with
`UartOptions::rx_priority` the knob, and every transport whose block
holds a request resets it in `release()`. The rows of the other strata
are this contract's pending waves: what each arms at today, and what
its controller offers.

| stratum | realization | beyond the contract |
|---|---|---|
| avrdx | none | the AVR DA/DB has no DMA controller |
| samc21 | one engine: the Uart's transmit engine (`samc21/dmac.hpp`), at level 0 | PENDING its wave; the DMAC's four levels (LVLEN, CHCTRLB.LVL) and whether a SERCOM holds a raised request across its release are unmeasured; the rest of the controller is declined by erratum 1.10.4 ([../samc21/dmac.md](../samc21/dmac.md)) |
| stm32g0 | every engine at the lowest level (`stm32g0/dma.hpp`) | PENDING its wave; the DMAMUX changes no arbitration (priority, then number), a memory-to-memory channel alternates with any other requester (RM0444 10.4.4); under two copies the receive FIFO wedged the receiver, the ring at the highest level cured it - measured, not yet the default |
| ch32v00x | `DmaPriority` (`ch32v00x/dma_engine.hpp`); the engines' defaults: receive very_high, transmit high, copy low (`ch32v00x/dma.hpp`) | the Uart's ring at `rx_priority`, its transmit at high; the SPI host's receive at very_high and transmit at high; the I2C host's two at high; the Uart and the I2C host reset their blocks in `release()` - the held request is the CH32V203's measurement, applied here by reading, the pulse correct either way ([../ch32v00x/dma.md](../ch32v00x/dma.md)) |
| stm32f4 | every engine at the lowest level (`stm32f4/dma.hpp`) | PENDING its wave: the stream arbiter's four levels, its FIFO and its bursts |
| rp2040 | every channel without HIGH_PRIORITY (`rp2040/dma.hpp`) | PENDING its wave: one bit, one low channel served every round - measured immune to the copies that starve the CH32's ring |
| ch32vx03 | `DmaPriority` (`ch32vx03/dma_engine.hpp`); the engines' defaults: receive and block source very_high, transmit and player high, copy low (`ch32vx03/dma.hpp`) | the Uart's ring at `rx_priority` and its transmit at high; the SPI host's receive very_high, transmit high; the I2C host's two at high; the converter's stream very_high, the DAC's high; the Uart and the I2C host reset their blocks in `release()`, the timers, the ADC and the DAC already did, the SPI host needs not (SPI1 measured) ([../ch32vx03/dma.md](../ch32vx03/dma.md)) |
| ch32x035 | none | the stratum has no DMA driver |
| rp2350 | the RP2040's arrangement (`rp2350/dma.hpp`) | PENDING its wave, as the rp2040 row |
| pl011, pl022, dw_apb_i2c | the IP strata's engine slots, armed at the family's normal priority | PENDING the RP waves: the PL011's receive engine without HIGH_PRIORITY |
| host | none | no DMA controller is simulated |
