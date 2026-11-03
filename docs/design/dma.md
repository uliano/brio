# DMA engines: what a program is owed

This page decides what every family's DMA engines owe the program
above them, whatever their controller: that a DMA channel (a stream on
the STM32F4) has one owner in an image, the peripheral request it
serves, checked when the program is compiled; the level each engine
arms at in its controller's arbitration, decided by what its peripheral
does when the channel is starved; the one knob a program has over the
level that matters most, the receive ring's; and the release contract,
that a transport starts from a reset block whenever it is initialized,
so a request its block held across its own release reaches nothing. It
is written for an embedded programmer who has set a DMA channel's
priority by habit, seen a UART overrun under a memory copy, or handed a
channel from one peripheral to another and found one stray byte in the
next transfer. How a controller is driven (its registers, its engines'
two moments, its costs) is each family's DMA page; this page is the
part of it a program can rely on unchanged from one family to the next.

The smallest use is none: a transport that takes an engine in its slot
(`DmaRxEngine`, `DmaTxEngine`) arms it at the rule's level, and the
image gives each channel to one request. The one knob is
`UartOptions::rx_priority` where a program ranks something above its
serial receive ring. Two prerequisites decide correctness: **one
channel, one owner, for the whole image** - two peripherals that would
take one channel in turn are a design the rule refuses, not a schedule
to get right; and **a receive ring armed below a memory-to-memory copy
can lose bytes** - the findings below say on which controllers, and
under what copy, it did, so the knob is turned down only with that
measurement in hand.

## The ownership rule

A DMA slot belongs to one owner for the whole life of an image. A slot
is what one engine runs on: a channel, or on the STM32F4 a stream - not
the stream's channel cell (CHSEL), because two cells of one stream
cannot run at once (RM0090 10.3.3), so they are one slot. Its owner is
the peripheral request it serves: one peripheral's one direction
(USART2's transmit, SPI1's receive) or one of a timer's events (TIM4's
update). There is no sharing of any kind: no slot with two owners at
once, no request bound to two slots, and no slot handed from one owner
to another in turn as the program runs. A channel no peripheral paces -
a memory-to-memory copy, a channel a program programs by hand - belongs
to a tag its user names, one tag a slot, under the same rule.

The owner is the request, not the engine type and not the transport
type, so two things that look like two users are one. A host that moves
its frames at two element widths over its channel (a display's 8-bit
commands and 16-bit pixels on one SPI) is one owner; so is one
peripheral driven by two transport types over the same request (a
buffered UART and a direct one on USART2's transmit). Both are allowed.

Why one owner and no hand-over:

- **The hand-over is where the faults live.** A request a peripheral
  raised and nobody acknowledged is held, on some controllers across
  every disable but a reset, and the next user of the channel takes a
  stray item from it at its first enable and stalls behind it, or has its
  stream shifted by one ("The release contract", below, and each
  family's DMA page). With one owner the only user that can meet a held
  request is the same owner again, and its own `init()` answers it.
- **The silicon says so where the map is free.** On the RP2040 and the
  RP2350 any channel takes any request, and the datasheets forbid two
  channels on one (RP2040 2.5.3.2, RP2350 12.6.4.2); on the SAM C21
  three channels running at once corrupt their transfers (erratum 1.10.4
  and the measurements beyond it, [the SAM C21 DMAC
  page](../samc21/dmac.md)), so an image is held to two, and with every
  channel given when the program is compiled that budget is a fact the
  compiler can see instead of a claim made at run time.
- **An engine keeps its bookkeeping in its type** (the channel's
  configuration word, whether a block is in flight, its fault count), so
  two engine types on one channel are two accounts of one register
  block, each blind to the other's transfer in flight.

What the rule costs is flexibility. A program cannot take a channel from
one transport for another as it runs; on a controller whose map wires
each request to one channel and ORs several onto it (the WCH families),
or offers a request one or two streams (the STM32F4), choosing an
instance for one peripheral can take another's only channel, and on the
SAM C21 an image has at most two DMA engines. Which choices a part's map
forces is that family's DMA page's to list.

### The check

The rule is checked when the program is compiled, by
`util/dma_owner.hpp`. The type that binds a slot to its request claims
the pair in its class body, and the claim binds it both ways. A second
owner on a slot already claimed does not compile: the error names the
slot and both owners, for DMA1's channel 7 asked for I2C1's receive
after USART2's transmit
`ambiguating new declaration of 'constexpr brio::DmaOwner<brio::DmaPeriph::i2c, 1, brio::DmaSignal::rx> brio::dma_owner_of(DmaSlotId<1, 7>)'`
and, on the note below it, the old declaration with USART2's owner. A
second slot for an owner already bound does not compile either, the
error naming the owner and both slots through `dma_slot_of`: USART2's
transmit on channel 5 after channel 4 reads
`ambiguating new declaration of 'constexpr brio::DmaSlotId<1, 5> brio::dma_slot_of(DmaOwner<brio::DmaPeriph::usart, 2, brio::DmaSignal::tx>)'`
against the old declaration's `DmaSlotId<1, 4>`. The slot is
`DmaSlotId<controller, slot>`, never the element type; the owner is the
request, `DmaOwner<kind, instance, signal>`, or the tag a program names.
A claim counts where the type that makes it is instantiated, so a type a
program names and never uses claims nothing, and the same claim made
again, by the engine and by its transport, is one claim. A claim is a
declaration the compiler checks and never emits: it adds no byte to the
image and no cycle.

The second direction is the silicon's own rule wherever a request can
reach more than one channel. The RP families route any request to any
channel and forbid two channels on one ("multiple channels should not be
connected to the same DREQ", RP2040 2.5.3.2; "must not", RP2350
12.6.4.2): the owner being the request, that is this direction, with no
key of its own. The STM32G0's DMAMUX forbids one request id on two
channels unless "the application guarantees that the two connected DMA
channels are not simultaneously active" (RM0444 11.4.4, the caution) -
in-turn use, which the rule forbids anyway. An STM32F4 request offered
two streams would be served by both if both were bound to it. On the
WCH families each request is wired to one channel, so this direction
never fires there and costs nothing.

The check sees one translation unit, as the compiler does. Every brio
image's application is one source file, its other units (the startup
code, the runtime) owning no channel, and that is why it suffices. A
program split across files would keep the check inside each file and
lose it between them: two files that give one slot to two owners each
compile clean, and the linker sees nothing.

### Realizations

How each family names a slot, and what its request map leaves a
program to choose. No family's engines or transports claim their slots
through the check today; the table is the key the rule gives each
family. The SAM C21 keeps a run-time claim at `init()`, a channel found
held panicking ([the SAM C21 DMAC page](../samc21/dmac.md)).

| stratum | the slot and its key | what the request map leaves to choose |
|---|---|---|
| avrxt | none | the AVR DA, DB and EA have no DMA controller |
| samc21 | the DMAC's channel 0 or 1, `DmaSlotId<0, ch>` | any trigger on any channel, within the image's budget of two |
| stm32g0 | a channel of DMA1 or DMA2 (DMA2 on the STM32G0B1 class alone), `DmaSlotId<c, ch>`; the request id the DMAMUX routes is not part of the key | any request on any channel |
| stm32f4 | a stream of DMA1 or DMA2, `DmaSlotId<c, stream>`, never the cell | the cell: each request on one or two (stream, cell) pairs |
| ch32v00x | a channel of DMA1, `DmaSlotId<1, ch>` | the instance: each request wired to one channel, several ORed onto it |
| ch32vx03 | a channel of DMA1 or DMA2 (DMA2 on the CH32V303 alone), `DmaSlotId<c, ch>` | the instance, as on the CH32V00x |
| ch32x035 | a channel of DMA1, `DmaSlotId<1, ch>` | the instance, as on the CH32V00x |
| rp2040 | a channel, `DmaSlotId<0, ch>` | any DREQ on any channel, one channel a DREQ |
| rp2350 | as the RP2040, over sixteen channels | as the RP2040 |
| host | none | no DMA controller is simulated |

## The priority rule

On a controller whose channels arbitrate by a software level, the
level is not a tuning knob left at its reset value: it is decided by
what the channel's peripheral does when the channel is starved.

- **An engine whose peripheral overruns when starved arms at the
  highest level**: a receive ring, an SPI host's receive, a
  converter's stream. Their data registers hold one item, or a FIFO a
  few items deep; the next item arrives on the wire's or the
  converter's clock whether the channel came or not, and an item the
  channel did not take in time is lost, with an overrun flag at best
  and in silence on a converter that has none.
- **A transmit or paced output engine arms at the next level**: a UART
  or SPI transmit, a DAC or timer stream. A starved transmit leaves
  the line idle a while and loses nothing; a host's clock waits
  between two frames; a paced output holds its last value a period.
  It underruns, or only slows. A receive that cannot overrun belongs
  here too, and for the same reason: an I2C host stretches the clock
  while its data register is full, so a starved channel slows the bus
  and loses nothing.
- **A memory-to-memory copy arms at the lowest level.** No request
  paces it: it asks for the bus on every cycle of its block, so a
  channel it outranks moves nothing until the block is over, and a
  copy is the one load that can starve a ring for longer than a
  character time.
- **Within a level the silicon's own tie rule stands** (on the
  STM32F1-lineage controllers the lower channel number wins), and the
  rule does not try to number channels.

The rule is measured on the CH32V203C8 and the CH32V303VC, whose
controller breaks a tie between equal levels by the lower channel
number ([the CH32V203/CH32V303 DMA page](../ch32vx03/dma.md), "The
priority rule on the bench"): a receive ring at the highest level, fed
16 KB at 1 Mbaud and 64 KB at 4.5 Mbaud while a 2 KB copy ran back to
back on a lower channel at the copy's level, took every byte on both
parts, and the same load against a ring a program armed at the lowest
level, the copy's channel then winning every tie, lost about seven
bytes in eight to some nine hundred overruns at 1 Mbaud. What that page
has not measured - where below 1 Mbaud a ring armed low starts to lose,
and that paced loads starve nothing at any level - is in its "Not
covered yet".

What the rule costs: the items the ring takes from the copy, and no
more. The copy beside a stream at 1 Mbaud ran at 0.96 to 1.00 of its
rate alone, at 4.5 Mbaud at 0.92 to 0.98, the lower figures where the
stream's sender shared the controller.

A controller that arbitrates differently still takes the rule. The
STM32G0's controller alternates a memory-to-memory channel with any
other requester (RM0444 10.4.4), so one copy of any length never
starved its ring, but two copies alternated with each other and did,
and there the receive FIFO did not absorb the starvation, it wedged
the receiver. The ring at the highest level is never starved by the
pair; the wedge itself is the silicon's, and the UART's vector cures
it so that a ring a program ranks low loses nothing within the FIFO's
nine characters and counts every gap past them
([the STM32G0 USART page](../stm32g0/usart.md)). The RP2040's arbiter
serves one normal channel every scheduling round after the high ones
(2.5.7), and no arrangement of copies starved a receive ring at
normal: two HIGH_PRIORITY copies of 16 KB back to back at 3 Mbaud,
one, or two normal ones ([the RP2040 DMA page](../rp2040/dma.md)). The
RP2350 states the same round (12.6.10) and has removed the idle cycle
the RP2040 inserted after a round of high channels (12.6.1), and there
two HIGH_PRIORITY copies starve a ring at normal (7 to 17 per cent of
its bytes lost at 1 Mbaud and two fifths to two thirds at 3 Mbaud, run
to run, on both architectures) where one such copy or two normal ones
starve nothing, and the ring at HIGH loses nothing beside any of them
([the RP2350 DMA page](../rp2350/dma.md)). The STM32F4's arbiter ranks
its streams port by port (RM0090 10.3.4), and there one or two copies
ranked above a receive ring on the same controller never starved it
at 11.25 Mbaud, three or four did (a ring armed low lost up to half
its frames), and the ring at the highest level lost nothing beside
four; a copy on the other controller, DMA2 against a ring on DMA1,
reaches it only through the bus matrix and starved nothing
([the STM32F4 DMA page](../stm32f4/dma.md)). The rule is the same
sentence on every family; what the level buys depends on the arbiter,
and is measured per family.

### The one knob: the receive ring's level

A program can rank a channel above its serial receive ring (a copy
that must not wait, a second ring it values more), and it says so on
the UART, in one shape on every family that offers it:
`UartOptions::rx_priority`, a value of the family's own `DmaPriority`
(its levels the silicon's: four on the STM32-lineage controllers),
defaulting to the family's highest. A UART with no
receive engine refuses the option at compile time: there is no
channel to rank. The knob is born where the level decides correctness.
On the RP families the level is one bit, and nothing ranks above a
ring at its top: the UART there takes no level, its receive engine
arms at HIGH_PRIORITY, and the one thing a knob could do, take the
ring down to normal, is the arrangement the RP2350 starves. On the SAM
C21 the controller has four levels and still nothing to rank: a receive
engine comes only with its transmit engine, and the pair is the image's
whole budget of two channels ([the SAM C21 DMAC page](../samc21/dmac.md)),
so the UART takes no level there either. Every
other engine's level is its driver's, stated beside the call with the
chapter's reason, and an engine armed by hand takes the rule's level
for its direction by default.

## The release contract

Every transport with an engine starts its `init()` from a reset block:
once the block's clock is open, it pulses the peripheral's reset line
before it writes a register of its own and before it arms a channel. Its
`release()` gates the block's clock, gives its pads back and closes its
interrupt line, and pulses no reset. (The RP families' transports stop a
block by holding it in reset, and their `init()` completes the pulse by
letting it go: the same contract, the reset line serving as the gate.)

Why the reset is the first thing `init()` does. A peripheral's DMA
request, once raised, is held until the controller acknowledges it or
the peripheral's reset line is pulsed, and the families differ in what
else withdraws it, each answer measured on its family's DMA page:

- **the CH32V203 and CH32V303, the CH32X035**: nothing else. Clearing
  the DMA enable, the block enable or a timer's counter leaves the
  request standing, and a gated clock freezes it on the channel's ORed
  request line; the channel's next enable then moves one stray item and
  stalls behind it, or, with the requester still clocked, shifts the
  stream by one ([the CH32V203/CH32V303 DMA page](../ch32vx03/dma.md)
  and [the CH32X035 DMA page](../ch32x035/dma.md), "A released
  requester");
- **the STM32G0**: a gated block withdraws it, and so does the DMA
  enable cleared with the clock on; a DMA enable left set asks again the
  moment anything opens the clock without a reset (SPI1, TIM3; [the
  STM32G0 DMA page](../stm32g0/dma.md), "A released requester holds
  nothing here");
- **the STM32F4 and the SAM C21**: a cleared DMA enable withdraws it
  (on the SAM C21, whose SERCOM has none, a disabled instance), but a
  gated clock holds one left enabled, and a channel bound to it took a
  stray item while the clock was still off ([the STM32F4 DMA
  page](../stm32f4/dma.md) and [the SAM C21 DMAC
  page](../samc21/dmac.md), "A released requester");
- **the RP2040 and the RP2350**: a request is a one-cycle pulse the
  channel counts (RP2040 2.5.3.2, RP2350 12.6.4.2); the PL011 and the
  PL022 withdraw theirs when their DMA enable is cleared or the block
  disabled ("all request signals are deasserted", RP2040 4.2.5), but the
  ADC disabled with DREQ_EN left set keeps a sample in its FIFO and
  requests it, and the credits a pacing request banked on a channel
  survive the channel's `stop()` ([the RP2040 DMA page](../rp2040/dma.md),
  "A released requester").

Under the ownership rule nobody else uses the channel, so a request held
across a release can reach only the same owner, when it is initialized
again over the same channel. A reset at that `init()`, with the clock
just opened and no channel armed, withdraws every one of those requests
before an engine could take the stray, whatever left the block that way:
the transport's own release, a program that stopped it some other way,
a debugger. A reset at `release()` served the hand-over between two
owners, the outgoing one cleaning the line for the incoming one that
could not tell who held it; the rule forbids that hand-over, so a
release has nothing to clean for anyone.

What each family's transports do today is in the rows below: they reset
their blocks in `release()`, before the gate. Where a family's `init()`
starts from a reset block, and the measurement of the contract on that
family - a transport released with a request standing, then initialized
again, its channel moving nothing it should not -, is that family's DMA
page's.

### Realizations

Common: on the WCH strata, the STM32G0, the STM32F4, the SAM C21 and
the two RP families every engine's level is the rule's (the defaults
by direction, each driver's call naming its own), with
`UartOptions::rx_priority` the knob where the level is more than one
bit and something can be ranked above a receive engine (not on the SAM
C21, whose receive engine and its transmit engine are the image's
whole budget), and every transport whose block can raise a request
resets it in `release()`, before the gate, but for two kinds of release:
a block measured not to hold a request (the SPI of the CH32V203 and
CH32V303, measured on SPI1, and the CH32V00x's of the same lineage, not
measured on that chip), and the SAM C21's SPI and I2C hosts, which
disable their SERCOM, the disable measured to withdraw a request in the
SERCOM's UART mode and not measured in its I2C mode (DS60001479M
33.6.4.1.2 says when the I2C host's transmit request rises and that
writing DATA clears it, and nothing of a disable).

| stratum | realization | beyond the contract |
|---|---|---|
| avrxt | none | the AVR DA, DB and EA have no DMA controller |
| samc21 | `DmaTxEngine<ch>` at level 2 and `DmaRxEngine<ch>` at level 3 of four (CHCTRLB.LVL), the rule's two, CTRL.LVLEN enabling those (`samc21/dmac.hpp`); the Uart takes the transmit engine or the pair, the SPI host the pair or the transmit engine alone, the I2C host the transmit engine alone | a budget of two channels an image, numbered 0 and 1, a claim that finds its channel held panicking, and no channel suspended or disabled mid-block beside a running one: the controller corrupts or starves the other channel otherwise (erratum 1.10.4 and the measurements beyond it), so a receive engine comes only with its transmit engine; its release answer is the one "The release contract" above gives, the UART resetting its SERCOM before the gate, the SPI host and the I2C host disabling it ([the SAM C21 DMAC page](../samc21/dmac.md)) |
| stm32g0 | `DmaPriority` (`stm32g0/dma_engine.hpp`); the engines' defaults: receive and ping-pong source very_high, transmit and player high, copy low (`stm32g0/dma.hpp`) | the UART's ring at `rx_priority` and its transmit at high; the SPI host's receive very_high, transmit high; the I2C host's two at high; the DMAMUX ranks nothing (priority, then number), and a memory-to-memory channel alternates with any other requester (RM0444 10.4.4), so it takes two copies to starve a ring, and behind a starved channel the receive FIFO can wedge, which the UART's vector cures with a kick (DMAR dropped and raised, [the STM32G0 USART page](../stm32g0/usart.md)); a released block measured to hold no request, but a DMA enable left set re-raises it when the clock returns, so every transport's `release()` resets its block ([the STM32G0 DMA page](../stm32g0/dma.md)) |
| stm32f4 | `DmaPriority` (`stm32f4/dma_engine.hpp`); the engines' defaults: receive (both shapes) very_high, transmit high, copy low (`stm32f4/dma.hpp`) | the UART's ring at `rx_priority` and its transmit at high; the SPI host's receive very_high, transmit high; the I2C and FMPI2C hosts' two at high; a DAC stream the program's to rank, its underrun disabling the transfers (RM0090 14.3.7); the arbiter ranks per port, and it takes three copies above a ring on its controller to starve it; a gated clock holds a raised request, a cleared enable withdraws it: the UART, the SPI host and client, the I2C host and client and the FMPI2C block reset in `release()`, the timers and the DAC already did, the ADC clears CR2 with the clock on (its reset line is every converter's), measured clean ([the STM32F4 DMA page](../stm32f4/dma.md)) |
| ch32v00x | `DmaPriority` (`ch32v00x/dma_engine.hpp`); the engines' defaults: receive very_high, transmit high, copy low (`ch32v00x/dma.hpp`) | the UART's ring at `rx_priority`, its transmit at high; the SPI host's receive at very_high and transmit at high; the I2C host's two at high; the UART and the I2C host reset their blocks in `release()`: the held request is the CH32V203's measurement, applied here by reading, the pulse correct either way; the SPI host disables its block with no pulse, as the CH32V203's does, not measured on this chip ([the CH32V00x DMA page](../ch32v00x/dma.md)) |
| ch32vx03 | `DmaPriority` (`ch32vx03/dma_engine.hpp`); the engines' defaults: receive and block source very_high, transmit and player high, copy low (`ch32vx03/dma.hpp`) | the UART's ring at `rx_priority` and its transmit at high; the SPI host's receive very_high, transmit high; the I2C host's two at high; the converter's stream very_high, the DAC's high; the UART and the I2C host reset their blocks in `release()`, the timers, the ADC and the DAC already did, the SPI host needs not (SPI1 measured) ([the CH32V203/CH32V303 DMA page](../ch32vx03/dma.md)) |
| ch32x035 | `DmaPriority` (`ch32x035/dma_engine.hpp`); the engines' defaults: receive very_high, transmit high, copy low (`ch32x035/dma.hpp`) | the UART's ring at `rx_priority` and its transmit at high; the SPI host's receive very_high, transmit high; the I2C host's two at high; the arbiter strict, the level and then the lower number (measured in quiet windows), and the ring at very_high kept 8192 frames at 3 Mbaud beside back-to-back copies on a lower channel that broke it at its second byte ranked low; a raised request held as on the CH32V203 (DMAT, UE or the gate let go and the next owner takes one stray item, the reset line none), so the UART, the I2C host and client and the SPI host and client reset their blocks in `release()`; in the Sleep the controller moves nothing, so the platform's idle path does not sleep over a working channel ([the CH32X035 DMA page](../ch32x035/dma.md)) |
| rp2040 | HIGH_PRIORITY, one bit (`rp2040/dma.hpp`); the engines' defaults: receive HIGH, transmit and copy normal | the PL011's ring and the PL022 host's receive HIGH, their transmits normal, the DW_apb_i2c host's two normal (SCL held behind a starved channel either way); no level option on the UART; one normal channel served every round, and nothing starved a ring at normal; the PL011, the PL022 host and client, the DW_apb_i2c host and client and the ADC hold their blocks in reset in `release()`, the next owner's `init()` completing the pulse; the PWM and the PIO share one reset among slices and state machines, a stopped slice pulses nothing (measured) and a state machine's FIFOs are drained by the next `init()` ([the RP2040 DMA page](../rp2040/dma.md)) |
| rp2350 | the RP2040's arrangement (`rp2350/dma.hpp`) | as the rp2040 row, the releases the same and measured the same; but two HIGH_PRIORITY channels starve a normal one here, so the receive engine's fixed HIGH is what keeps the ring (one HIGH channel or two normal ones starve nothing) ([the RP2350 DMA page](../rp2350/dma.md)) |
| pl011, pl022, dw_apb_i2c | the IP strata's engine slots, each level named at the call as the family's one bit | the PL011's receive engine high and its transmit normal; the PL022 host's receive high (one bind, `arm_rx_engine()`) and its transmit normal; the DW_apb_i2c host's two normal; under the PL011's receive engine an overrun clears UARTRSR and masks its source until the next publish, so every loss is reported and none raises a storm ([the PL011 page](../pl011/README.md)) |
| host | none | no DMA controller is simulated |
