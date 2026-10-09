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
the receive FIFO did not absorb the starvation, it wedged the receiver.
The ring at the highest level is never starved by the pair; the wedge
itself is the silicon's, and the Uart's vector cures it so that a ring a
program ranks LOW loses nothing within the FIFO's nine characters and
counts every gap past them ([../stm32g0/usart.md](../stm32g0/usart.md)). The RP2040's arbiter serves
one normal channel every scheduling round after the high ones (2.5.7),
and no arrangement of copies starved a receive ring at normal - two
HIGH_PRIORITY copies of 16 KB back to back at 3 Mbaud, one, or two normal
ones ([../rp2040/dma.md](../rp2040/dma.md)). The RP2350 states the same
round (12.6.10) and has removed the idle cycle the RP2040 inserted after a
round of high channels (12.6.1), and there TWO HIGH_PRIORITY copies starve
a ring at normal - 7 to 17 per cent of its bytes lost at 1 Mbaud and two
fifths to two thirds at 3 Mbaud, run to run, on both architectures - where
one such copy or
two normal ones
starve nothing, and the ring at HIGH loses nothing beside any of them
([../rp2350/dma.md](../rp2350/dma.md)). The STM32F4's arbiter ranks its streams
port by port (RM0090 10.3.4), and there one or two copies ranked above a
receive ring on the same controller never starved it at 11.25 Mbaud,
three or four did - a ring armed low lost up to half its frames - and
the ring at the highest level lost nothing beside four; a copy on the
other controller, DMA2 against a ring on DMA1, reaches it only through
the bus matrix and starved nothing ([../stm32f4/dma.md](../stm32f4/dma.md)).
The rule is the same sentence on every family; what the level buys
depends on the arbiter, and is measured per family.

### The one knob: the receive ring's level

A program can rank a channel above its serial receive ring - a copy
that must not wait, a second ring it values more - and it says so on
the Uart, in ONE SHAPE on every family that offers it:
`UartOptions::rx_priority`, a value of the family's own `DmaPriority`
(its levels the silicon's: four on the STM32-lineage and the SAM's
controllers), defaulting to the family's highest. A Uart with no receive
engine refuses the option at compile time: there is no channel to rank.
THE KNOB IS BORN WHERE THE LEVEL DECIDES CORRECTNESS. On the RP families
the level is one bit, and nothing ranks above a ring at its top: the
Uart there takes no level, its receive engine arms at HIGH_PRIORITY, and
the one thing a knob could do - take the ring down to normal - is the
arrangement the RP2350 starves. Every other engine's level is its
driver's, stated beside the call with the chapter's reason, and an engine
armed by hand takes the rule's level for its direction by default.

## The release contract

A peripheral's DMA request, once raised, is HELD until the controller
acknowledges it or the peripheral's reset line is pulsed - on the
CH32V303 and the CH32V203 clearing its DMA enable, its block enable or a
timer's counter does not withdraw it, and a gated clock freezes it on
the channel's ORed request line. The next owner of the channel then
moves one stray item on it at its first enable and stalls behind it, or,
with the old requester still clocked, gets its stream shifted by one
([../ch32vx03/dma.md](../ch32vx03/dma.md), "A released requester").

THE STM32G0 MEASURED THE OTHER ANSWER, and the contract is the same
sentence. Its DMAMUX routes one request line to each channel, so a stray
could reach only the next owner of the SAME request; and there a gated
block withdraws a raised request, and so does its DMA enable cleared with
the clock on - USART1, SPI1 and TIM3 released every way with a request
standing left the next owner nothing. What survives is the DMA enable
LEFT SET: SPI1 and TIM3 ask again the moment anything opens the clock
without a reset ([../stm32g0/dma.md](../stm32g0/dma.md), "A released
requester holds nothing here"). The STM32F4 gave a third answer: a
cleared DMA enable withdraws the request, but a gated clock does not -
USART6, SPI1, TIM1 and ADC1 released with the enable left set and the
clock gated handed the next owner of their cell a stray item while the
clock was still off ([../stm32f4/dma.md](../stm32f4/dma.md), "A released
requester"). The RP2040 and the RP2350 gave a fourth, the same on both:
a request there is a one-cycle pulse the CHANNEL counts (RP2040 2.5.3.2,
RP2350 12.6.4.2), so it can outlive a release in two places. At the
peripheral, the PL011 and the PL022 withdraw a raised request when their
DMA enable is cleared or the block disabled ("all request signals are
deasserted", RP2040 4.2.5), and a stopped PWM slice pulses nothing - but
the ADC disabled with DREQ_EN left set keeps its sample in the FIFO and
requests it, and the next owner's channel reads a stale sample. In the
channel, the credits a pacing request banks on a bound channel survive its
`stop()` and a rebind to another request, and the next owner's TRIGGER
drops them: nothing reached the next owner of a channel
([../rp2040/dma.md](../rp2040/dma.md), "A released requester"). The reset
pulse clears every one of them, so the rule does not change with the
answer.

So a transport's `release()` resets its peripheral BEFORE it gates its
clock, wherever its block can hold a request, and the incoming owner
needs nothing beyond its engine's `arm()`: the outgoing owner's release
is the whole of the hand-over, because the incoming one cannot tell which
requester holds the line, and its own enable is what would consume the
stray. A block measured not to hold a request (SPI1 on the CH32V203 and
CH32V303 family) says so in its driver and needs no pulse.

### Realizations

Common: on the WCH strata, the STM32G0, the STM32F4 and the two RP
families every engine's level is the rule's - the defaults by direction,
each driver's call naming its own - with `UartOptions::rx_priority` the
knob where the level is more than one bit, and every transport whose
block can raise a request resets it in `release()`. The SAM C21's row is
this contract's pending wave: what it arms at today, and what its
controller offers.

| stratum | realization | beyond the contract |
|---|---|---|
| avrdx | none | the AVR DA/DB has no DMA controller |
| samc21 | one engine: the Uart's transmit engine (`samc21/dmac.hpp`), at level 0 | PENDING its wave; the DMAC's four levels (LVLEN, CHCTRLB.LVL) and whether a SERCOM holds a raised request across its release are unmeasured; the rest of the controller is declined by erratum 1.10.4 ([../samc21/dmac.md](../samc21/dmac.md)) |
| stm32g0 | `DmaPriority` (`stm32g0/dma_engine.hpp`); the engines' defaults: receive and ping-pong source very_high, transmit and player high, copy low (`stm32g0/dma.hpp`) | the Uart's ring at `rx_priority` and its transmit at high; the SPI host's receive very_high, transmit high; the I2C host's two at high; the DMAMUX ranks nothing (priority, then number), and a memory-to-memory channel alternates with any other requester (RM0444 10.4.4), so it takes TWO copies to starve a ring - and behind a starved channel the receive FIFO can WEDGE, which the Uart's vector cures with a kick (DMAR dropped and raised, [../stm32g0/usart.md](../stm32g0/usart.md)); a released block measured to hold NO request, but a DMA enable left set re-raises it when the clock returns, so every transport's `release()` resets its block ([../stm32g0/dma.md](../stm32g0/dma.md)) |
| ch32v00x | `DmaPriority` (`ch32v00x/dma_engine.hpp`); the engines' defaults: receive very_high, transmit high, copy low (`ch32v00x/dma.hpp`) | the Uart's ring at `rx_priority`, its transmit at high; the SPI host's receive at very_high and transmit at high; the I2C host's two at high; the Uart and the I2C host reset their blocks in `release()` - the held request is the CH32V203's measurement, applied here by reading, the pulse correct either way ([../ch32v00x/dma.md](../ch32v00x/dma.md)) |
| stm32f4 | `DmaPriority` (`stm32f4/dma_engine.hpp`); the engines' defaults: receive (both shapes) very_high, transmit high, copy low (`stm32f4/dma.hpp`) | the Uart's ring at `rx_priority` and its transmit at high; the SPI host's receive very_high, transmit high; the I2C and FMPI2C hosts' two at high; a DAC stream the program's to rank, its underrun disabling the transfers (RM0090 14.3.7); the arbiter ranks per port, and it takes THREE copies above a ring on its controller to starve it; a gated clock HOLDS a raised request, a cleared enable withdraws it - the Uart, the SPI host and client, the I2C host and client and the FMPI2C block reset in `release()`, the timers and the DAC already did, the ADC clears CR2 with the clock on (its reset line is every converter's), measured clean ([../stm32f4/dma.md](../stm32f4/dma.md)) |
| rp2040 | HIGH_PRIORITY, one bit (`rp2040/dma.hpp`); the engines' defaults: receive HIGH, transmit and copy normal | the PL011's ring and the PL022 host's receive HIGH, their transmits normal, the DW_apb_i2c host's two normal (SCL held behind a starved channel either way); no level option on the Uart; one normal channel served every round, and nothing starved a ring at normal; the PL011, the PL022 host and client, the DW_apb_i2c host and client and the ADC hold their blocks in reset in `release()`, the next owner's `init()` completing the pulse; the PWM and the PIO share one reset among slices and state machines, a stopped slice pulses nothing (measured) and a state machine's FIFOs are drained by the next `init()` ([../rp2040/dma.md](../rp2040/dma.md)) |
| ch32vx03 | `DmaPriority` (`ch32vx03/dma_engine.hpp`); the engines' defaults: receive and block source very_high, transmit and player high, copy low (`ch32vx03/dma.hpp`) | the Uart's ring at `rx_priority` and its transmit at high; the SPI host's receive very_high, transmit high; the I2C host's two at high; the converter's stream very_high, the DAC's high; the Uart and the I2C host reset their blocks in `release()`, the timers, the ADC and the DAC already did, the SPI host needs not (SPI1 measured) ([../ch32vx03/dma.md](../ch32vx03/dma.md)) |
| ch32x035 | none | the stratum has no DMA driver |
| rp2350 | the RP2040's arrangement (`rp2350/dma.hpp`) | as the rp2040 row, the releases the same and measured the same; but TWO HIGH_PRIORITY channels starve a normal one here, so the receive engine's fixed HIGH is what keeps the ring - one HIGH channel or two normal ones starve nothing ([../rp2350/dma.md](../rp2350/dma.md)) |
| pl011, pl022, dw_apb_i2c | the IP strata's engine slots, each level named at the call as the family's one bit | the PL011's receive engine high and its transmit normal; the PL022 host's receive high (one bind, `arm_rx_engine()`) and its transmit normal; the DW_apb_i2c host's two normal; under the PL011's receive engine an overrun clears UARTRSR and masks its source until the next publish, so every loss is reported and none raises a storm ([../pl011/README.md](../pl011/README.md)) |
| host | none | no DMA controller is simulated |
