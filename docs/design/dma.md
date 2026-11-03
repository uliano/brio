# DMA engines: what a program is owed

This page decides what every family's DMA engines owe the program
above them, whatever their controller. A DMA channel (a stream on the
STM32F4) has one owner in an image, the peripheral request it serves.
The compiler checks that for every transport, the driver that moves a
peripheral's data (a `Uart`, an `SpiHost`, an `I2cHost`); a channel a
program arms by hand is checked only as far as "The check" says. Each
engine arms at a level in its controller's arbitration, decided by what
its peripheral does when the channel is starved. A program has one knob
over the level that matters most, the receive ring's. And a transport
starts from a reset block whenever it is initialized, so a request its
block held across its own release reaches nothing; the few blocks with
no reset line of their own are named in "The release contract".

It is written for an embedded programmer who has set a DMA channel's
priority by habit, seen a UART overrun under a memory copy, or handed a
channel from one peripheral to another and found one stray byte in the
next transfer. How a controller is driven (its registers, its engines'
two moments, its costs) is each family's DMA page; this page is the
part of it a program can rely on unchanged from one family to the next.

The smallest use is none: a transport that takes an engine as a template
argument, its engine parameter (`DmaRxEngine`, `DmaTxEngine`), arms it at
the rule's level, and the image gives each channel to one request. The
one knob is `UartOptions::rx_priority`, where a program ranks something
above its serial receive ring. Two prerequisites decide correctness: **one
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
to another in turn as the program runs. A channel no peripheral paces,
a memory-to-memory copy, belongs to a tag its user names, one tag a slot,
under the same rule. A channel a program programs by hand claims the
request it serves or a tag; only the request lets the compiler see that
request on a second channel.

The owner is the request, not the engine type and not the transport
type, so two things that look like two users are one. A host that moves
its frames at two element widths over its channel (a display's 8-bit
commands and 16-bit pixels on one SPI) is one owner; so is one
peripheral driven by two transport types over the same request (a
buffered UART and a direct one on USART2's transmit). Both are allowed.
Two transport types over one request take the channel in turn, each
released before the other's `init()`. Nothing checks that: it is the
programmer's duty.

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
  compiler can see instead of something tested at run time.
- **An engine keeps its bookkeeping in its type** (the channel's
  configuration word, whether a block is in flight, its fault count), so
  two engine types on one channel are two accounts of one register
  block, each blind to the other's transfer in flight.

What the rule costs is flexibility. A program cannot take a channel from
one transport for another as it runs; on a controller whose map wires
each request to one channel and ORs several onto it (the WCH families),
or offers a request a short list of streams (the STM32F4), choosing an
instance for one peripheral can take another's only channel, and on the
SAM C21 an image has at most two DMA engines. Which choices a part's map
forces is that family's DMA page's to list.

### The check

The compiler checks claims, by `util/dma_owner.hpp`. The type that binds a
slot to its request claims the pair, and the claim binds it both ways.
Where it does so differs: in its class body for the transports of the
STM32G0, the STM32F4, the CH32V203 and CH32V303, the SAM C21 and the IP
strata and for the copy engines of those families that have them; in the
verb that programs the channel on the CH32V00x and the CH32X035, and for
the CH32V203 and CH32V303's converter and DAC streams. A second
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
A claim counts when the class or the verb that makes it is instantiated.
So a type a program names and never uses claims nothing, and where the
claim is in a verb, neither does a type whose verb is never called. The
same claim made again, by the engine and by its transport, is one claim.
A claim is a
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
in-turn use, which the rule forbids anyway. An STM32F4 request the map
offers on two or three streams takes one of them. On the WCH families each
request is wired to one channel, so a transport cannot reach this
direction: it fires only on a program that claims one named request or
one tag on two channels by hand.

The check sees one translation unit, as the compiler does. The projects
build every application from one source file (one `main()` per
`src/apps/<app>.cpp`), its image's other units (the startup code, the
runtime) owning no channel, and that is why it suffices here. Nothing
enforces the one file. A program split across files would keep the check
inside each file and lose it between them: two files that give one slot to two owners each
compile clean, and the linker sees nothing.

What the compiler checks and what stays the programmer's are apart. A
transport knows its request, so it claims by itself. An engine a program
arms by hand, or a channel it programs itself, depends on the family:

- **The CH32V00x and the CH32X035:** `arm()` and the `DmaChannel` verbs
  always claim, under the owner the program names or, by default, a tag of
  the engine's kind. Two kinds on one channel do not compile. One engine
  type armed for two requests of its channel under its default tag does:
  the tag does not say which request it serves.
- **The STM32G0, the STM32F4, the CH32V203 and CH32V303 and the RP
  families:** a bare engine claims nothing. The program writes the claim
  beside it, and nothing checks that it did, so two hand-armed engines on
  one channel, for two requests in turn, compile if neither is claimed.
- **The SAM C21:** a program names its engines in a transport and does not
  arm them.

Each family's DMA page gives the form.

### Realizations

How each family names a slot, and what its request map leaves a
program to choose. Every family with a DMA controller claims through the
check, when the program is compiled. The engines' `claim()` verb, on the
families that have one, is another thing: a run-time test-and-set of
`busy()` that decides which of a transport's two contexts starts the next
block.

| stratum | the slot and its key | what the request map leaves to choose |
|---|---|---|
| avrxt | none | the AVR DA, DB and EA have no DMA controller |
| samc21 | the DMAC's channel 0 or 1, `DmaSlotId<0, ch>` | any trigger on any channel, within the image's budget of two |
| stm32g0 | a channel of DMA1 or DMA2 (DMA2 on the STM32G0B1 class alone), `DmaSlotId<c, ch>`; the request id the DMAMUX routes is not part of the key | any request on any channel |
| stm32f4 | a stream of DMA1 or DMA2, `DmaSlotId<c, stream>`, never the cell | the cell: each request on one or two (stream, cell) pairs, three for a few on the STM32F411 |
| ch32v00x | a channel of DMA1, `DmaSlotId<1, ch>` | the instance: each request wired to one channel, several ORed onto it |
| ch32vx03 | a channel of DMA1 or DMA2 (DMA2 on the CH32V303 alone), `DmaSlotId<c, ch>` | the instance, as on the CH32V00x |
| ch32x035 | a channel of DMA1, `DmaSlotId<1, ch>` | the instance, as on the CH32V00x |
| rp2040 | a channel, `DmaSlotId<0, ch>` | any DREQ on any channel, one channel a DREQ |
| rp2350 | as the RP2040, over sixteen channels | as the RP2040 |
| host | none | no DMA controller is simulated |

## The priority rule

On a controller whose channels arbitrate by a software level, the level
of an engine is decided by what the channel's peripheral does when the
channel is starved. (Only a raw channel configuration a program writes
itself keeps the reset value, `low`, on the STM32G0 and the STM32F4.)

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
  paces it: it asks for the bus on every cycle of its block. A channel it
  outranks on a strictly ranked controller waits for about the whole
  block, which makes a copy the load that can starve a ring for longer
  than a character time. Controllers that arbitrate otherwise are below.
- **Within a level the silicon's own tie rule stands** (on the
  STM32F1-lineage controllers the lower channel number wins), and the
  rule does not try to number channels.

The rule is measured on the CH32V203C8 and the CH32V303VC, whose
controller breaks a tie between equal levels by the lower channel
number ([the CH32V203/CH32V303 DMA page](../ch32vx03/dma.md), "The
priority rule on the bench"): a receive ring at the highest level, fed
16 KB at 1 Mbaud and 64 KB at 4.5 Mbaud while a 2 KB copy ran back to
back on a lower channel at the copy's level, took every byte on both
parts. The same load against a ring a program armed at the lowest
level, the copy's channel then winning every tie, lost about seven
bytes in eight at 1 Mbaud, with some nine hundred overruns counted. What
that page
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

A UART, an SPI host or an I2C host with an engine, and a converter or a
timer that feeds a channel, starts its `init()` from a reset block: once
the block's clock is open, it pulses the peripheral's reset line before it
writes a register of its own and before it arms a channel. Three blocks
have no line of their own to pulse. The STM32F4's ADC shares its line with
every converter, so its `init()` clears CR2, CR1 and the flags instead. The
RP families' PWM slices and PIO state machines share theirs, so a stopped
slice pulses nothing and a state machine's FIFOs are drained by the next
`init()`.

`release()` gates the block's clock and pulses no reset; a transport also
gives its pads back and closes any interrupt line it opened. (The RP
families' transports stop a block by holding it in reset, and their
`init()` completes the pulse by letting it go: the same contract, the reset
line serving as the gate.)

Why the reset is the first thing `init()` does. On every family but the RP
families, whose requests are pulses, a peripheral's DMA request, once
raised, stays up until the controller acknowledges it or the peripheral's
reset line is pulsed. The families differ in what else withdraws it, each
answer measured on its family's DMA page:

- **the WCH families**: for a USART's and a timer's request, the CH32V00x's
  SPI1 and the CH32V203's and CH32V303's I2C1, nothing else. Clearing the DMA
  enable, the block enable or a timer's counter leaves the request standing,
  and a gated clock freezes it on the channel's ORed request line. The channel's
  next enable then moves one stray item and stalls behind it, or, with the
  requester still clocked, shifts the stream by one; an I2C read over such a
  request never launches. An SPI's request falls with SPE on the CH32V203,
  the CH32V303 and the CH32X035, and on the CH32V00x and the CH32X035
  I2C1's falls with its gate ([the CH32V00x DMA
  page](../ch32v00x/dma.md), [the CH32V203/CH32V303 DMA
  page](../ch32vx03/dma.md) and [the CH32X035 DMA
  page](../ch32x035/dma.md), "A released requester");
- **the STM32G0**: a gated block withdraws it, and so does the DMA
  enable cleared with the clock on; a DMA enable left set asks again the
  moment anything opens the clock without a reset (SPI1, TIM3; [the
  STM32G0 DMA page](../stm32g0/dma.md), "A released requester holds
  nothing here");
- **the STM32F4 and the SAM C21**: a cleared DMA enable withdraws it
  (on the SAM C21, whose SERCOM has none, a disabled instance), but a
  gated clock holds one left enabled. On the STM32F4 a stream on its cell
  took a stray item while the clock was still off; on the SAM C21 the
  flag reads up behind the gate ([the STM32F4 DMA
  page](../stm32f4/dma.md) and [the SAM C21 DMAC
  page](../samc21/dmac.md), "A released requester");
- **the RP2040 and the RP2350**: a request is a one-cycle pulse the
  channel counts (RP2040 2.5.3.2, RP2350 12.6.4.2); the PL011 and the
  PL022 withdraw theirs when their DMA enable is cleared or the block
  disabled ("all request signals are deasserted", RP2040 4.2.5), but the
  ADC disabled with DREQ_EN left set keeps a sample in its FIFO and
  requests it, and the credits a pacing request banked on a channel
  survive the channel's `stop()`, the next trigger dropping them ([the
  RP2040 DMA page](../rp2040/dma.md), "A released requester", and [the
  RP2040 PWM page](../rp2040/pwm.md)).

Under the ownership rule nobody else uses the channel, so a request held
across a release can reach only the same owner, when it is initialized
again over the same channel. A reset at that `init()`, with the clock
just opened and no channel armed, withdraws such a request before an
engine could take the stray, whatever left the block that way: the
transport's own release, a program that stopped it some other way, a
debugger. On the blocks with no line of their own, the STM32F4 ADC's
register clear and the PWM's next trigger do that job. A reset at
`release()` would serve only a hand-over between two owners, which the
rule forbids.

The measurement of the contract on a family is that family's DMA page's: a
transport released with a request standing (or that state staged at
register level, on the STM32G0, the STM32F4 and the RP families), then
initialized again, its channel moving nothing it should not, and the same
with the reset removed. That control shows a stray item on the WCH
families and the STM32F4, a request asked again as the clock reopens on the
STM32G0's SPI1 and TIM3, and a FIFO's stale frames on the RP families. On
the SAM C21 it shows nothing, `configure()`'s own disable withdrawing the
request before an engine is armed; the reset stays there, the contract
being one.

### Realizations

Common: on the WCH strata, the STM32G0, the STM32F4, the SAM C21 and
the two RP families every engine's level is the rule's (the defaults
by direction, each driver's call naming its own), with
`UartOptions::rx_priority` the knob where the level is more than one
bit and something can be ranked above a receive engine (not on the SAM
C21, whose receive engine and its transmit engine are the image's
whole budget), and every transport that takes an engine follows the
contract. A driver with no engine is outside it and may keep a pulse in
its release (the CH32X035's SPI and I2C clients, the STM32G0's synchronous
modes).

| stratum | the levels | the hosts and the UART |
|---|---|---|
| avrxt | none: the AVR DA, DB and EA have no DMA controller | |
| samc21 | `DmaTxEngine<ch>` at level 2 and `DmaRxEngine<ch>` at level 3 of four (CHCTRLB.LVL), CTRL.LVLEN enabling those (`samc21/dmac.hpp`) | the Uart takes the transmit engine or the pair, the SPI host the pair or the transmit engine alone, the I2C host the transmit engine alone |
| stm32g0 | `DmaPriority` (`stm32g0/dma_engine.hpp`); receive and ping-pong source very_high, transmit and player high, copy low (`stm32g0/dma.hpp`) | the UART's ring at `rx_priority`, its transmit high; the SPI host's receive very_high, transmit high; the I2C host's two high |
| stm32f4 | `DmaPriority` (`stm32f4/dma_engine.hpp`); receive (both shapes) very_high, transmit high, copy low (`stm32f4/dma.hpp`) | as the STM32G0; the FMPI2C host's two high |
| ch32v00x | `DmaPriority` (`ch32v00x/dma_engine.hpp`); receive very_high, transmit high, copy low (`ch32v00x/dma.hpp`) | as the STM32G0 |
| ch32vx03 | `DmaPriority` (`ch32vx03/dma_engine.hpp`); receive and block source very_high, transmit and player high, copy low (`ch32vx03/dma.hpp`) | as the STM32G0; the converter's stream very_high, the DAC's high |
| ch32x035 | `DmaPriority` (`ch32x035/dma_engine.hpp`); receive very_high, transmit high, copy low (`ch32x035/dma.hpp`) | as the STM32G0 |
| rp2040 | HIGH_PRIORITY, one bit (`rp2040/dma.hpp`); receive HIGH, transmit and copy normal | the PL011's ring and the PL022 host's receive HIGH, their transmits normal, the DW_apb_i2c host's two normal (SCL held behind a starved channel either way); no level option on the UART |
| rp2350 | the RP2040's arrangement (`rp2350/dma.hpp`) | as the RP2040 |
| pl011, pl022, dw_apb_i2c | the IP strata's engine parameters, each level named at the call as the family's one bit | as the RP2040; the PL022 host's receive binds once, `arm_rx_engine()` |
| host | none: no DMA controller is simulated | |

What a family adds beyond the contract and the sections above:

- **The SAM C21** ([its DMAC page](../samc21/dmac.md)): its two channels are
  numbered 0 and 1 and held by the compiler. No channel is suspended or
  disabled mid-block beside a running one on a working path, because the
  controller corrupts or starves the other channel otherwise (erratum 1.10.4
  and the measurements beyond it); a dead block's recovery is the exception
  that page lists.
- **The STM32F4** ([its DMA page](../stm32f4/dma.md)): a DAC stream is the
  program's to rank, its underrun disabling the transfers (RM0090 14.3.7).
- **The CH32X035** ([its DMA page](../ch32x035/dma.md)): the arbiter is
  strict, the level and then the lower number (measured in quiet windows).
  The ring at very_high kept 8192 frames at 3 Mbaud beside back-to-back
  copies on a lower channel that broke it at its second byte ranked low. In
  the Sleep the controller moves nothing, so the platform's idle path does
  not sleep over a working channel.
- **The PL011** ([its page](../pl011/README.md)): under its receive engine
  an overrun clears UARTRSR and masks its source until the next publish, so
  every loss is reported and none raises a storm.
