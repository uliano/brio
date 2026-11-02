# The shared SPI bus

This page decides how several active objects share one SPI bus. An
active object (AO) is a component of a brio program with its own event
queue, served one event at a time by the kernel; serving one event is a
dispatch ([learn/introduction.md](../learn/introduction.md)). A request
describes one complete transaction (which chip select, which clock and
mode, a command phase and a data phase). The requester posts it to the
bus active object, the arbiter, and gets a `SpiDone` event back; the
arbiter runs one transaction at a time on the family's `SpiHost`, the
host engine that owns the wire. It runs a request one of two ways, chosen
by the request's `engine` field: `SpiEngine::polled`, an explicit loop inside
`SpiHost::start()`, or `SpiEngine::dma`, asynchronously on the host's DMA
engines (the two DMA channels bound to it). The host never chooses for the
program: a request runs on the engine it names, at any length. It is
the reference form of the bus active-object pattern;
[i2c-bus.md](i2c-bus.md) shares the same arbiter. It is written for an
embedded programmer who has driven an SPI device from a loop and now
has two devices on one bus and two components that want them.

The smallest use: one `SpiHost<n, ...>` of the family, called `Bus`
here, one `SpiBus<Bus, P>` in the list of active objects given to the
kernel (`Tenuto`, the cooperative kernel; `P` is the family's platform
type), and a client that posts `Bus::Request{...}` with its buffers and a
reply channel, `reply_to<Client, SpiDone>()`, then handles `SpiDone` in
its own dispatch, the event's one field, `status`, holding the outcome.
Posting never waits, whichever way the request runs: the post returns at
once and the transaction runs later, in the arbiter's dispatch. A request
naming `SpiEngine::polled`, the field's default, runs its loop inside
`start()` there, for its whole wire time, and needs no DMA and no vector. A
request naming `SpiEngine::dma` is asynchronous: it rides the host's DMA engines, so the host needs engines
bound and the program's DMA vectors bound to `dma_isr()`. A host with no
engines - always the AVR's, and a SAM C21 host whose engines are not named -
refuses it with `spi_rejected` and nothing on the wire, and there every request
runs `SpiEngine::polled`; a host with engines refuses the same way a request
they cannot carry (its section below names what). So
the program decides two things for every request: which way it runs, and
**its buffers are lent until `SpiDone` arrives**, whichever way; the client
must not touch them before, and must not reuse them for another request
before. What else decides correctness (a clock brought up first, a select
pad parked high, a DMA controller initialized, the slew class of pads on
wires) is the family's and stands at the top of its page.

## The stack

```
     client AO                           client AO
        \  post(Request{...})              /
         v                                v
     SpiBus<Bus, P>          util/spi_bus.hpp   arbitration + replies
         |  Bus::start(req)
         v
     SpiHost<n> engine      <stratum>/spi.hpp   CS/DC, the polled loops, the DMA engines
         |
     ISR glue in the app    the DMA vectors: posts TransferDone{status} on completion
```

Layering: `SpiBus` is `util/`'s, pure and host-tested against a fake
bus; the `SpiHost<n>` engine belongs to a target stratum and knows the
silicon. The application's interrupt handler binds the vector, as
always.

`SpiBus` is an alias of `BusMaster<Bus, P>` (`util/bus_master.hpp`),
the arbiter shared with I2C ([i2c-bus.md](i2c-bus.md)); `SpiDone` and
`spi_ok` are the SPI names of `BusDone` and `bus_ok`.

### Realizations

Common to all but the simulated host: the `Request` field for field (`cs`,
`dc`, `cmd`, `cmd_len`, `tx`, `rx`, `len`, `reply`, `mode`, `engine`,
`cs_setup_us`), the engine's functions `init`, `start`, `rebase`, `recover`,
`release`, `sck_hz`, `max_sck_hz`, the `has_dma` constant (whether the host
can run an asynchronous request at all, "Two ways to run a request" below),
on a host with engines the one interrupt body the application's DMA vector
glue calls, `dma_isr`, and the vocabulary `SpiMode`, `SpiDone` and the
`spi_*` status codes. An 8-bit request whose fields are assigned by name
reads the same on every row but for the rate's type; the fields' order
differs from stratum to stratum, so a designated initializer follows its own
host's declaration. Each engine is measured against a real client on the wire
(transactions queued from one dispatch, a rejection when the queue is full,
both answers to the power manager's sleep vote, and, where a host can leave a
transaction open, the per-bus timeout with `recover()`) on the platforms [the
target table](../../README.md) marks supported; the CH32V00x's against the
CH32X035's client ([the CH32V00x page](../ch32v00x/README.md)). What each
stratum adds is in its section under ["The hosts, family by
family"](#the-hosts-family-by-family), at the end of this page; the table
holds the one fact that decides a reader's choice.

| stratum | realization | what decides |
|---|---|---|
| avrxt | `SpiHost<n, route>` (`avrxt/spi.hpp`) | polled only: no DMA controller |
| samc21 | `SpiHost<n, pads, generator, TxEngine, RxEngine, Placement>` (`samc21/spi.hpp`) | engines optional, the pair or the transmit engine alone, inside the DMA controller's budget of two channels an image; the rate is a divisor |
| stm32g0 | `SpiHost<n, pins, TxEngine, RxEngine, Placement>` (`stm32g0/spi.hpp`) | 4- to 16-bit frames, FIFOs, engines optional |
| stm32f4 | `SpiHost<n, pins, TxEngine, RxEngine>` (`stm32f4/spi.hpp`) | one-deep receive register, the rate off the instance's own APB clock, the SCK pad's class under an erratum |
| ch32v00x | `SpiHost<1, pins, TxEngine, RxEngine, Placement>` (`ch32v00x/spi.hpp`) | one-deep receive register, the channels fixed |
| ch32vx03 | `SpiHost<n, pins, TxEngine, RxEngine>` (`ch32vx03/spi.hpp`) | one-deep receive register, the rate off the instance's own bus, the channels fixed per instance |
| ch32x035 | `SpiHost<1, pins, TxEngine, RxEngine, Placement>` (`ch32x035/spi.hpp`) | one-deep receive register, the channels fixed, one pad class |
| rp2040 | `SpiHost<n, pins, TxEngine, RxEngine, Placement>` (`rp2040/spi.hpp`), the family's alias of `pl022/spi.hpp`'s engine | eight-deep FIFOs, any two channels, the rate a pair |
| rp2350 | `SpiHost<n, pins, TxEngine, RxEngine, Placement>` (`rp2350/spi.hpp`), the same engine | as the RP2040, over pads that come up isolated |
| host | `SimSpiHost<n, trace_depth, dma>` (`host/sim_spi_host.hpp`) | no silicon: at the request and not at the register block, so no rate, no mode and no `rebase`, `sck_hz` or `max_sck_hz` |

One of those differences is one thing spelled several ways and is
recorded as such: the rate's unit, a division enum on most strata, a
divider pair on the PL022 and a divisor on the SAM C21, with the ceiling
and the chooser named for it (`ceiling_baud`/`baud_for` against
`ceiling_clock`/`clock_for`). Whether one function with a ceiling in hertz
should replace them is not decided.

The client side is deliberately not one surface. `SpiClient` on each
stratum is the application's protocol over that silicon's own client
half (a shift register with a two-deep buffer, a SERCOM with PLOADEN,
an SPI with a two-deep FIFO, the F1's single data register), and the
peers of the bench converge on one algorithm, answers kept queued
ahead of what the host has clocked, whose only per-silicon parameter
is how many must be queued ahead: one on the AVR, two on the SAM for
its three-SCK-cycle rule, two on the G0 for its FIFO, one on the
CH32V00x, the CH32V203 and CH32V303 and the CH32X035, eight on both
PL022s (the depth of their FIFOs) and one on the STM32F4; the PL022 client also takes one frame
per select window in modes 0 and 2 ([the RP2040 SPI
page](../rp2040/spi.md)). That integer is the one thing a portable
client would need, and each `SpiClient` publishes it as
`frames_ahead`.

The peripherals share almost nothing below the contract: a shift
register with a two-deep buffer, a SERCOM with a pad matrix, an SPI
with a FIFO and a frame size, one with no FIFO at all and a DMA
channel per direction, a PL022 with two eight-deep FIFOs and a request
any channel takes, and an F1-lineage block with no FIFO whose SCK pad
is a correctness parameter. That one descriptor serves them all is the
point worth recording. And where two families carry the same
block, the descriptor stops being a claim: the PL022's driver is one
file both of them alias ([the PL022 page](../pl022/README.md)).

## Why the event queue alone cannot arbitrate

An asynchronous transaction outlives the dispatch that starts it: it
completes on an interrupt later. While the bus is busy the kernel happily delivers
the next request event, which therefore needs a place to wait, and it
waits where `post()` built it. The bus active object holds the
request's slot in its own queue (`Lease::hold`,
[kernel.md, "Payloads"](kernel.md#4-payloads-what-travels-inside-the-event-kernelborrowedhpp)
and [kernel.md, "Queues"](kernel.md#5-queues-kernelevent_queuehpp))
and keeps the slot's number in a small pending FIFO, one byte per
waiting request, touched from main context only and so with no
critical section;
the request on the wire is held the same way until its requester has
been answered. When the FIFO is full the request is answered at once
with `SpiDone{spi_rejected}`, counted, and not held. That answer holds
while the clients keep at most 2 x `pending_depth` + 2 requests
outstanding at once; beyond that the queue itself drops the request with
no reply and its `overflows()` counts it. The control events (the completion, a sleep
vote, a timeout and its marker) are never held. The active object is a
real two-state machine (idle and busy); the request's `ReplyTo<SpiDone>`
capsule is the return channel, read from the held slot before the slot
is released, so the arbiter never knows who its clients are.

The queue's depth is counted in those slots: `pending_depth` requests
held waiting, the one held in flight, `pending_depth` more posted
beyond them and waiting to be answered `spi_rejected`, a completion
and a sleep vote (three more on a timed bus: a timeout or two and the
marker). One spare slot holds the event being dispatched, and a request
the FIFO has no room for is answered from there. The slots
are not reserved by event type: the second `pending_depth` is what
keeps a burst of excess requests from crowding out a completion (an
untimed bus would wedge) or a vote (the power manager would wait for
ever). A held request costs no copy and no time, however
long its transfer lasts.

The request event (two spans, a select, the bus's settings and the
reply capsule) exceeds the size of about 8 bytes the kernel recommends
for an event ([kernel.md, "Events"](kernel.md#3-events-value-semantics-per-ao-variant)):
legal, because the request is the arbitration token and the queues are
per object, so nobody else pays. Its size is paid per copy,
and the arbiter makes none of its own: every request is built once, by
`post()` in a slot of the bus active object's queue (the kernel's copy,
under the producers' mask), and lent from that slot to `start()` for
the call (the contract in `util/bus_master.hpp`), the slot of the
event being dispatched when the bus is idle, its held slot when the
request waited. A polled request completes inside `start()` and is
copied nowhere else: the engine reads every field through the
reference. An asynchronous one outlives the call, so the engine keeps
what its tenure (the request's time on the bus) needs, and on every family
with engines that is two pins, the request's select and its D/C, which the
completion raises: the buffers, the lengths and the width are the channels'
once they are launched, and the rate, the mode, the setup time and the reply
are spent before `start()` returns. The slot stays held until the completion has been answered, so an
engine could read the request there instead; none does, and that is not part
of the contract. A request that waits costs one
byte of FIFO and no copy, and a retrying completion policy starts the
held request again from its slot: no copy and no storage either.

## The transaction descriptor (`SpiHost<n>::Request`)

**The request is the complete script of one bus tenure.** A shared bus
forces per-transaction context (chip select, clock, mode: who you are,
how you talk); that much is the definition of a bus. But the
descriptor also carries protocol structure, the D/C flip, and its
place there is forced by a deeper rule: the request is the atomic unit
of arbitration, so anything that must happen inside the chip-select
window without interleaving must be described in the request. The
alternative, client-side pin toggling between chained requests with a
"hold CS" flag, would let the arbiter interleave another client into
an open chip-select window, or force it to understand linked tenures,
with priority inversion built in. The two-phase command/data script
with one optional pin is the smallest script covering every device on
the bench. The fully general form (a segment list with pin actions
between segments, scatter-gather style) is the known successor, to be
built when a real device breaks two phases (QSPI-style dummy cycles,
three-phase protocols). Splitting the descriptor into per-shape
request types would not even save queue RAM while any display client
shares the bus: every slot of a queue is as large as the largest event
it accepts.

Two phases in one chip-select window, covering every device class in
sight:

1. optional `cmd[cmd_len]` transmitted with D/C low;
2. optional `len` frames with D/C high, full duplex: transmit `tx` (or
   all-ones dummies if null), capture into `rx` (or discard if null).

`cmd_len` and `len` count frames: bytes with the default 8-bit frames, on a
host with wider ones two bytes of the buffer a frame, low byte first.

| Device class | Shape |
|--------------|-------|
| display-style controller with a D/C line | cmd + tx (or cmd + rx for reads), D/C toggles inside the chip-select window |
| rx-only converter | no cmd, tx null, rx set |
| generic transfer / loopback | tx and rx both set |
| block devices (command protocols) | sequences of plain transfers |

Chip select is active low, asserted and released by the engine around
the whole transaction, on a GPIO the request names (a host may offer a
hardware select of its own, the PL022's `frame_select`, and a null `cs`
is a caller framing the window by hand); `dc` may be a null `PinRef`. Both phases are
optional: a device without D/C is a null `dc` plus a phase-2-only
transfer, so plain multibyte transactions (16-bit register devices,
delta-sigma ADC frames of 24-bit groups) are already just `tx`/`rx`
spans, with no display pattern involved. Buffer ownership travels with
the request, and the `cmd`, `tx` and `rx` fields name it: they are
`Lease::reply` loans, so **the client must not touch the buffers until
its `SpiDone` arrives**. That rule is what keeps this free of races:
the client's handler and the engine, its loop or its DMA channel, never
write the same buffer at the same time, and the completion event is the one
boundary.

Word semantics live above the wire, since a driver moves frames and
knows nothing of what they mean: `util/wire.hpp` provides `constexpr` big-endian load and store
for 16, 24 and 32-bit words and the sign-extending `load_be24_signed`
for ADC channel data; the client formats and parses its byte spans at
the edges. A device demanding chip-select-per-word framing would be
the one legitimate engine extension in this area; noted, not built,
since no such device is on the bench.

## Per-transaction clock and mode

On a shared bus every device names its own speed and mode: the
descriptor carries `SpiClock clock` and `SpiMode mode` (defaults div16
and mode 0), and at each `start()` the engine folds them, with the
frame size where the block has one, into the control word the block
takes, compares it with the word in force and writes only on a
change, with whatever disable the chip's reference manual asks around
that write:
nothing on the unchanged path, nothing per frame. `SpiHost<n>::init()`
takes the clock tag and an optional SCK ceiling for the whole bus, with
no per-transaction rate and no mode; a request that asks for more than
the ceiling is slowed to it. The reason: devices on one bus disagree (a
display comfortable at 6 MHz, a touch controller capped below 2.5 MHz),
and a global init-time clock is a bug waiting for the second device.

## Two ways to run a request

`Bus::start(req)` returns a bool: true means the transaction ended
inside `start()`, and the arbiter replies with whatever the engine's
`status()` reports; false means it is still on the wire and a
`TransferDone` will arrive. The way travels per request in the
`engine` field: like the clock, the client knows its transaction.

- **Polled** (`SpiEngine::polled`), and **polled is an explicit loop**, at
  any length and on every host. `start()` runs the whole transaction in
  that loop, in the arbiter's dispatch, and returns done. No DMA channel
  moves a frame and no interrupt vector is needed, on a host with engines
  bound as on one without. What the loop still depends on is the
  peripheral and, for a nonzero `cs_setup_us`, the timer `delay_us()` spins
  on: on the Cortex-M strata SysTick's counter must be running (never its
  interrupt), and with it stopped the delay returns at once, the setup time
  skipped in silence ("Two silicon facts the engine honours" below).

  The peripheral's own interrupts are never armed. Global interrupts stay
  enabled, with one exception: the AVR masks them around the burst's last
  data store and the flag clear behind it. The wait is bounded by a spin
  budget on every host but the AVR's, whose transfer, once written with the
  client select disabled (SSD set), always completes.

  **The poster does not wait; the arbiter's dispatch does**, for the
  loop's whole time. Under the cooperative kernel (`Tenuto`) that holds up
  every other active object too, only interrupt handlers running until
  `start()` returns, and the arbiter goes on through any further polled
  or refused request queued behind it in the same dispatch, so the hold-up
  is the sum of what is queued. Polled is the program's choice for a
  transaction short enough to hold everyone that long, or for a bus with no
  engines. Ask asynchronously when the other objects must keep running
  through it, or when a long read wants the wire's rate.

  The loop's shape follows the request's buffers. A phase with no
  receive buffer (every command phase, a display's pixels) paces on the
  transmit flag, never reads the answers and clears the receive side at
  its tail (the PL022 takes this loop only for a run longer than its FIFO
  at the rate where its receive loop falls behind the wire, 8-bit frames at
  clk_peri / 2, and the receive loop's shape otherwise; the SAM C21 only for
  a data phase longer than its two-level receive buffer, reading its command
  phase back character by character). That loop keeps the wire's
  pace at the family's second-fastest rate and slower on every family
  measured, but for the AVR's and the RP2040's 1.09 and 1.28 of the wire at
  /4, and in mode 3 alone on the PL022, whose modes 0 and 2 keep a gap
  between frames ([benchmark.md](benchmark.md), letter e, the instrument's
  table of the host's cost above the wire). A phase with a receive buffer
  keeps no more frames in flight than the receive side can hold: one on a
  one-deep receive register (the STM32F4, the CH32V00x, the CH32V203 and
  CH32V303, the CH32X035), two on the SAM C21's two-level buffer, a FIFO's
  worth on a FIFO (the STM32G0's, the PL022's). The AVR's keeps three, one
  waiting and two in flight, where a late loop finds the FIFO full and the
  third byte held in the shifter, safe by a silicon fact measured on that
  chip ([the AVR SPI page](../avrxt/spi.md)).

  **So a polled loop cannot lose a frame.** A master owns the clock: SCK
  moves only for a frame the loop has written, and the loop has written
  none that the receive side has no room for, so a handler or a
  preemption that holds the loop off leaves a gap on the bus, however
  long, and nothing else. Measured with a handler longer than a frame
  beside the loop, on the STM32F446RE, the CH32V203C8T6 and CH32V303VCT6,
  the CH32V006K8U6 and the CH32X035F8U6: every window exact, the frame count
  right and the overrun flag never left standing (the loop's own reads clear
  it on the STM32 and CH32 blocks, so the count and the bytes are the
  witness), the window longer by what the arithmetic allows; and under the
  preemptive kernel in [kernel.md,
  "Rubato"](kernel.md#rubato-the-preemptive-kernel-kernelrubatohpp).

  What it costs on a one-deep register is the loop's turn: the bus idles
  from the frame's end until the next write, every frame, measured at most
  68 core cycles a frame on the STM32F446RE at 180 MHz in the lines run,
  modes 0 and 1 at /4 to /256 (SPI2 in mode 1; on SPI1 in mode 0, 30 at /4
  and nothing at /64 and slower, the flag rising half a bit before the
  frame's end), 30 on the CH32V203C8T6 and 32 on the
  CH32V303VCT6 at 144 MHz, 38 on the CH32V006K8U6 and 22 on the
  CH32V003F4P6 at 48 MHz, 44 on the CH32X035F8U6 from the flash and 18 from
  SRAM. Over 256 frames of a byte read, in x, the transaction's duration
  over the wire's, so larger is slower: 1.33 to 1.80 at /4 and 1.07 to 1.21
  at /16 on the STM32F446RE, the CH32V203C8T6, the CH32V303VCT6 and the
  CH32V006K8U6, the CH32V003F4P6 2.00 and 1.33 with its tick's handler in
  the wall ([benchmark.md](benchmark.md), letter e); where a FIFO or a
  two-level buffer stands, 1.01 to 1.34 at /4 and 1.00 to 1.02 at /16. **A
  long read that wants the wire asks asynchronously.** A transmit-only phase
  pays no such price on the wire: it does write ahead, into the transmit
  buffer, having nothing to lose; its price is the hold-up above.

  The polled path reads the lent request in place and copies nothing; a
  request of three frames at /16 costs the host 211 to 539 cycles above
  the wire on the families letter e of [benchmark.md](benchmark.md)
  measures, whose table sets the vendor's own loop beside it on the
  boards where it was run. The CH32X035 has no benchmark suite: its
  figure is `test_x035_spi` letter b's, 1216 to 1232 cycles in all from
  the flash against the wire's 384, the suite's own call inside it.
- **Asynchronous** (`SpiEngine::dma`), and **asynchronous is DMA**. The
  data rides the host's DMA engines at any length, one frame included:
  `start()` arms them and returns false, and the engines' completion
  interrupt, `dma_isr()` returning true, is the edge on which the
  application's glue posts `TransferDone{status}` to the bus active
  object - one interrupt a request, the core free for the wire's whole
  time. `start()` returns false only when the engines have the request.
  It decides, in this order:
  1. **No engines: refused.** On a family without DMA, or a host type with
     no engines bound, `start()` returns true at once with `spi_rejected`
     in `status()`, before it touches the block, the select or the D/C, so
     nothing reaches the wire; the arbiter answers the requester
     `spi_rejected`, the code a full pending FIFO answers too (in both the
     request never reached the wire). The code does not say which: a full
     FIFO clears, a host with no engines never does, so a client that
     retries on `spi_rejected` retries for ever there. The host type
     publishes `has_dma`, a `constexpr`, for the program that posts an
     asynchronous request to assert: `static_assert(Bus::has_dma)`.
  2. **Nothing to move: done.** A request of zero total length answers
     `spi_ok` inside `start()`, the wire untouched.
  3. **What the engines cannot carry: refused**, the same way and before
     the block, the select or the D/C: frames wider than a byte over
     engines that move bytes, a buffer of such frames off a half-word, a
     buffer the channels cannot reach, a transmit channel another owner holds - each family's
     list is in its section below. A request the engines cannot carry is
     not run in the loop instead: that would make the answer to "does
     `start()` hold the dispatch?" depend on where a buffer happens to
     sit. One case is found later on the STM32F4 and the STM32G0: a
     channel a program holds with `claim()` refuses the block at its
     launch, after the select fell, and the request still ends
     `spi_rejected` inside `start()`, the select back up and nothing of its
     data phase clocked.
  4. **A command followed by data:** the command runs in the polled loop
     inside `start()`, then the data phase on the engines. One fixed
     rule, whatever the lengths: a command is short by nature, and a block
     of its own would cost a second launch and a second interrupt. A stall
     in that loop ends the request as a polled one's does.
  5. **A command alone** is the block, on the engines.

  No short request, no threshold and no cost decides anything: the program
  names the engine, because the host can count only its own core cycles
  and knows nothing of what else must run while the wire does. As guidance
  the host never reads, each family's SPI page gives E, the engines' fixed
  cost to the core (`start()`, the launch, the one interrupt, the
  completion), F, a polled request's fixed part, and from which length the
  engines cost the core fewer cycles at each rate: the smallest frame count
  N with F + N x frame >= E, a frame's polled cost being the wire's frame
  or the loop's own turn, whichever is longer. A wrong choice costs time,
  never correctness ("When a request ends, the wire has ended", below).

Why asynchronous is DMA and not a per-frame interrupt: measured on the
CH32X035 at 48 MHz, a handler that writes the next frame when the last
comes back costs some 225 to 268 cycles a frame, where the wire's byte is,
by arithmetic, 32 cycles at 12 MHz, 128 at 3 MHz and 384 at 1 MHz - at
every rate a program uses it frees little or none of the core and idles
the wire between frames, where the engines free all of it after a fixed cost
(some 1400 cycles from the request to its completion in that family's
suite, the suite's own call included). A per-frame interrupt also lengthens every
other line's hold-off by its handler, and under the preemptive kernel
the code that opens such a transaction (the first frames written, then
the interrupt armed) is a window a preemption longer than a frame turns
into a lost frame. The I2C hosts and the UARTs keep their interrupt
engines: a byte at 400 kHz is about a thousand cycles of a 48 MHz
core, several times their handlers.

Time awake does not tell SPI's two ways apart. On a chip whose Sleep
stops the DMA (the CH32V003, the CH32V203 and CH32V303, the CH32X035) the
kernel's idle does not sleep while a channel works
(`qingke/platform.hpp`), so the engines keep the core awake for the
wire's time; a polled transfer keeps it spinning that long too. An I2C
pump, by contrast, lets the core sleep between bytes, so there the choice
weighs time awake as well ([i2c-bus.md](i2c-bus.md), "Which engine moves
a half").

On a synchronous completion the arbiter replies immediately, and
whenever a transfer ends it keeps draining the pending FIFO through
any further synchronous requests (each answered and released in
turn, all in one dispatch), going or staying `busy` only when a transfer
actually stays in flight. Both ways interleave freely on one bus. A
request of zero total length completes on the spot, wire untouched;
the reply still arrives, so nothing hangs silently.

A synchronous completion is not a success by definition. A polled wait
that runs out of its spin budget (a flag that never rose, a peripheral
whose clock does not run: a gate closed, the enable down, a mode fault that
demoted it) ends the request inside `start()` with `spi_stalled` in
`status()` and the select released, and that code is what the requester's
`SpiDone` carries. Every host whose loop has
a budget answers it, one spelling and one value on every family, in
`util/spi_bus.hpp` (the range `util/bus_master.hpp` leaves to engines). The
budget is the hold-up a stall costs: hundreds of milliseconds at a slow rate
on a long request (781.7 ms for 2048 frames on the STM32F446RE, its
[SPI page](../stm32f4/spi.md)), all of it inside the arbiter's dispatch.
On the STM32F4 and the PL022 a request on the engines can answer it too:
the wait for the wire's end in the DMA interrupt has a bound, and when it
runs out the request ends `spi_stalled` there, through `TransferDone`, the
select released ("When a request ends, the wire has ended", below).

`spi_dma_fault` is the engines' alone: a DMA block the controller reports
it could not finish, chip select raised, the code reaching the requester
through `TransferDone`. A polled request never rides the engines, so it
never answers it; a DMA block that simply never completes is the arbiter's
`spi_timeout` on a bus with a timeout ("The per-bus timeout", below).

### What follows each code, and who acts

**The arbiter calls `recover()` on `spi_timeout` and on nothing else.**
Every other code reaches the requester as it is, and the arbiter goes on at
once to the next waiting request, in the same dispatch, before the requester
has run. So:

| code | what the host has done | who acts |
|---|---|---|
| `spi_rejected` | nothing: the request never reached the wire - but for a channel a program holds, found at the launch on the STM32F4 and the STM32G0, where the select fell and rose again and a command before data was clocked | the client: a full FIFO clears, so it may post again; on a host with no engines it never clears |
| `spi_stalled` | select released first, then the block reset and reconfigured, inside `start()`; when the wait for the wire's end ran out in the DMA interrupt (the STM32F4, the PL022), select released and a reset marked as owed, as for `spi_dma_fault` | nobody: the next request starts on a clean block |
| `spi_dma_fault` | in the DMA interrupt: select released, both channels put away, a reset marked as owed. The next `start()` or `prime()` pays it before anything else | nobody: each family with engines provokes a real transfer error in its suite and reads the next request back exact |
| `spi_timeout` | the arbiter has called `recover()`: the stale completion silenced, the select raised, the peripheral reset | nobody for the engine; the wire and the device stay the application's to judge |

`SpiHost` does this itself because nobody else can. A request queued behind
a stall or a fault starts in the same dispatch, before the requester's handler
can call anything, so a program could not reset the block in time; and a
program that drives `SpiHost` directly has no arbiter at all. A stall inside
`start()` resets at once, in main context, the select first because the reset
can move the SCK pad and a device still selected would count the edge. A fault,
or a stall met in the DMA interrupt, only marks the reset, because a DMA
interrupt decides and does not work: it does not reprogram the block. The mark
costs a host with engines at most one load and one branch a request, never one
a frame (the SAM C21 none: its mark is a value of the word its `apply()`
already compares). `prime()` pays it too, because a caller that frames
the select itself must have the block clean before the select goes low. Such a
caller's select, a null one in the request, stands low through a stall's reset:
the driver cannot release what it does not hold.

What a reset cannot do is open a closed clock gate. Behind one, N queued polled
requests stall N times in a row, each on its own budget, and each is answered
`spi_stalled`.

**A program owes `recover()` nothing after a stall or a fault**: the verb
stays for the arbiter's timeout. A program that calls it
anyway does so with the bus idle, and the arbiter publishes no such fact: no verb
says whether a request is in flight or waiting. A client knows the bus is idle only by its own bookkeeping, every
request of every client answered; on a bus with one client that is the
`SpiDone` it holds, having posted nothing since. `recover()` over an
asynchronous request in flight puts its engines away, so its `TransferDone`
never comes: an untimed arbiter then waits for ever, a timed one answers
`spi_timeout`.

Which of the codes a host can answer is in its section at the end of this
page and in its document. The completion policy (`util/bus_master.hpp`) judges asynchronous
completions only: a retry of a failure reported inside `start()` would
run inside the same dispatch, compounding the waiting the polled
client bounded on purpose, so the failure reaches that requester as it
is and the decision to try again is its own.

## When a request ends, the wire has ended

On both engines and for every caller, one that frames the select itself (a
null `cs`) included, the request's select rises after the last SCK edge, and
what tells the caller the request is over - `start()` answering true, or
`dma_isr()` answering true - comes after that edge too. A device therefore
sees the transaction end where it ends, whichever engine the request named,
and a wrong choice of engine costs time, never correctness.

The flags a loop or a vector waits for do not always say so. A receive flag
rises when the frame's last bit is captured, and in the modes whose sampling
edge comes first (CPHA = 0, modes 0 and 2) that is half an SCK period before
the frame's last edge. Each family's suite reads the SCK pad right after
every end, in the four modes, and each family does what its silicon asks:

- **the STM32F4**: BSY falls with RXNE, so no flag and no interrupt marks
  the edge; the host waits for the SCK pad's own input to read the idle
  level, at each polled phase's tail and in the DMA interrupt, the
  interrupt's wait bounded by half a bit at the rate in force;
- **the STM32G0**: only the polled read ends early; its tail waits for the
  SCK pad the same way, and neither the transmit-only tail (FTLVL, then
  BSY) nor the DMA interrupt, whose own body outlasts half a bit at every
  rate, needs a wait;
- **the PL022** (the RP2040, the RP2350): BSY falls one whole SCK period
  after the receive FIFO's flag, in every mode; with SPH = 0 the host waits
  for BSY at each polled phase's tail and in the DMA interrupt, the
  interrupt's wait bounded by one SCK period, and with SPH = 1 the flag
  already marks the last edge;
- **the CH32V00x, the CH32V203 and CH32V303, the CH32X035**: the receive
  flag and the receive channel's completion are not early on these blocks,
  measured in all four modes, and nothing waits;
- **the SAM C21**: RXC comes half a bit early in modes 0 and 2, and TXC
  marks the last frame's end, so every request ends on TXC - the polled
  receive shapes wait for it once after their last RXC, and the engines'
  one interrupt is TXC itself;
- **the AVR**: a burst ends on TXCIF, shifter and buffer empty, which is
  not early (in modes 0 and 1 on a marginal witness, [the AVR SPI
  page](../avrxt/spi.md)); nothing waits.

**A wait in the DMA interrupt is a cost every other interrupt pays.** An
interrupt body runs to completion, so where the host waits there (the
STM32F4; the PL022 with SPH = 0) every other interrupt is held off for up to
half a bit on the STM32F4 and one SCK period on the PL022, less the
interrupt's own entry: nothing at the fast rates, the most at the slowest
divisor. A polled request pays its wait inside `start()`, one read where the
edge has already passed. Each family's SPI page has the measured wait by
rate. A wait whose bound runs out ends the request `spi_stalled` ("What
follows each code", above).

The rule is the select's and the end's; the D/C line between a command and
its data is outside it. On the STM32F4 and the PL022, which wait once a
phase, D/C flips after the command's last edge; the SAM C21 raises it on the
last command character's RXC, half a bit before that character's last edge
in modes 0 and 2 ([the SAM C21 SPI page](../samc21/spi.md)).

## Two silicon facts the engine honours

Found with the MCP3550 on the analyzer, both general:

- **SCK must sit at the request's CPOL before chip select falls.**
  Devices that latch their SPI mode from the SCK level at the
  chip-select edge (the MCP3550: mode 0,0 against 1,1) otherwise start
  the transaction in the wrong mode. The AVR's SPI updates the SCK
  output level when it is enabled and at every transfer, not on a
  CTRLB write while enabled (a known AVR quirk, seen on the analyzer).
  So the engine applies a CPOL change with the peripheral disabled:
  preset the SCK pin's PORT.OUT to the new idle level (what the pin
  shows while the SPI is off, so no glitch), disable, write the mode,
  re-enable (`SpiHost::reprogram()`, four stores, the pad's and three of
  the peripheral's, no clock edges on the bus, only when the polarity
  changes between transactions). A dummy byte without chip select also works and was rejected: a side
  effect on the bus.
- **Chip-select setup time is a device parameter.** `Request::cs_setup_us`
  is the number of microseconds between the chip-select assertion and
  the first SCK edge, spun in `start()`. Most devices need nothing
  beyond what the code path takes; the MCP3550 waking from shutdown needs
  a few microseconds or it drops the frame. Bounded by a byte, spent in
  main context, chosen by the client that knows its device. The spin is
  `delay_us()`: it needs its timer running (on the Cortex-M strata SysTick's
  counter, `cortexm/delay.hpp`), and the host ignores its answer, so with
  the timer stopped the setup time is skipped in silence.

## Transaction economics

The per-request fixed cost is the price of arbitration, not of any
descriptor field: the event round trip (the request copied into the
queue, the dispatch, the reply posted, the requester's dispatch) is the
larger part, while an unused D/C's share is a few instructions. It is paid
once per request and does not depend on the
length, so the defence is making requests big, not fast: batch words
into one span (that is why the descriptor speaks spans) and amortize
the round trip. For a device that demands chip-select-per-word
framing, the noted engine extension would batch N words into one
tenure. And a device that owns a bus alone can skip the arbiter
entirely: with `SpiEngine::polled`, `SpiHost<n>::start()` is a complete
synchronous transfer function usable directly by the owning active
object. The arbitration price is only paid where there is something to
arbitrate.

## The per-bus timeout

`SpiBus` surfaces `BusMaster`'s `timeout_ticks` (the full design and
its rules in [i2c-bus.md](i2c-bus.md): one mechanism, both
vocabularies). On SPI the plausible wedge is not a wire, since the
host clocks itself, but a dead engine whose completion never posts: a
DMA channel that stopped running, a completion interrupt that simply
never fired, a host demoted mid-transfer by a second device on its
select. A host with no engines (the AVR's, and a SAM C21 host whose
engines are not named) never leaves one open, so the timeout never arms
over it. On a
host that can, a wedged transaction comes back `spi_timeout` on the
arbiter's clock. The arbiter itself has called `SpiHost::recover()` by then,
so the requester has nothing to call: it has silenced the stale completion,
put a demoted peripheral back to a host (CH32V203) or run the disable
procedure and the RCC reset (STM32G0), and closed the select window so the
device sees the transaction end. Staged and measured on silicon: on the
STM32G0 a lost completion (the DMA vector's body runs and ends the
DMA block but the `TransferDone` is never posted) is answered `spi_timeout`
in its place, and the very next four transactions run to `spi_ok` on
the same bus active object; on the CH32V203 an engined write whose
peripheral a second device's select demotes 30 ms in is answered
`spi_timeout`, and with the arbiter's recovery behind it the next
transaction runs to `spi_ok`. Size the
limit to the longest legal transaction; polled requests complete
inside `start()` and never arm it. With `timeout_ticks = 0` (the
default) there is no timer at all, **and a DMA channel that never completes
leaves the bus busy for ever**: a program whose requests ride the engines
sizes a limit unless that is a risk it has weighed.

## Multi-client rules of thumb

- Each client owns its chip-select pin (configures it, idles it high)
  and any device-specific pins (D/C, reset). Bench hygiene: deselect
  every device on the bus at application init, even those the
  application never talks to.
- A client keeps at most one request in flight and posts the next from
  its `SpiDone` handler: with N such clients the pending FIFO needs at
  most N - 1 slots and rejects never fire (the default depth of 4 is
  the margin).
- The latency picture: a short request waits at most for the transfer
  in flight (a kilobyte-class transfer at 6 MHz on the engines is a
  millisecond and a half), fine for polling-rate clients. If a client
  ever needs better, that is a scheduling design change, not a FIFO
  change.

Device-specific facts (controllers, wiring, clock caps) live in the
page of the board that carries the device ([docs/boards/](../boards/README.md))
and in the header comment of the application that talks to it, never
here.

## The hosts, family by family

What each stratum adds to the common surface, the same facts in the same
order: the rate, the frame width, the engine slots and what they refuse, how
a request's end waits for the wire's, the codes `status()` answers, what is
this family's alone. The costs - E, F and from which length the engines cost
the core fewer cycles ("Two ways to run a request", above) - are each
family's SPI page's.

### avrxt

The rate is a `SpiClock` division enum, with `ceiling_clock()` the optional
SCK ceiling a rebase re-resolves and `clock_for(hz)` the chooser
(`spi_clock_for(clk_per_hz, hz)` is the same arithmetic with the clock
stated). Polled only: the AVR DA, DB and EA have no DMA controller, so
`has_dma` is false and every request naming `SpiEngine::dma` is refused
(`spi_rejected`, nothing on the wire), the host arming no interrupt and
keeping no transaction's state past `start()`. A burst ends on TXCIF, after
the last edge, so nothing waits for the wire's end. `status()` is
`spi_rejected` after a refusal and `spi_ok` otherwise. `prime(mode, clock)`
serves a caller framing the select by hand.

### samc21

The rate is a `uint8_t baud` divisor (the SERCOM's own register), so the
ceiling is `ceiling_baud()` and the chooser `baud_for(hz)`. Two optional
engine slots, the pair or the transmit engine alone (a receive engine alone
does not compile), `has_dma` whether any is bound; they spend the DMA
controller's budget of two channels an image, which a third claim exceeds
with a panic ([the SAM C21 DMAC page](../samc21/dmac.md)). Without them every
request naming `SpiEngine::dma` is refused (`spi_rejected`), the host arming
no interrupt and leaving its NVIC line disabled. With them, a request on the
transmit engine alone that reads is refused before anything moves; a write
rides the transmit channel alone, a read starts its receive channel first,
both channels armed for their transfer errors alone, and the request's one
interrupt is the SERCOM's TXC, `dma_isr()` bound in the SERCOM's vector and
in the DMA controller's. Every request ends on TXC, polled or not, which
puts the end after the last edge. `status()` is `spi_ok`, `spi_stalled` when
a polled wait ran out, `spi_dma_fault` or `spi_rejected`. The placement
moves the polled transaction and the command loop of a request on the
engines. `prime(mode, baud)` serves a caller framing the select by hand;
`reference_hz()` is the stated GCLK rate ([the SAM C21 SPI
page](../samc21/spi.md)).

### stm32g0

The `SpiClock` enum with `ceiling_clock()` and the chooser `clock_for(hz)`;
`reference_hz()` is PCLK. A frame size in the Request (`bits`, 4 to 16, eight
by default) with `cmd_len` and `len` counted in frames. Two optional DMA
engine slots, `has_dma` whether they are bound: a request naming `dma` rides
them (`dma_isr()` the one interrupt body, the peripheral's own interrupts
never armed and its NVIC line left alone). They refuse frames of 9 to 16
bits on byte engines and a wide buffer off a half-word, and a transmit
channel a program holds refuses the block at its launch (the refusal after
the select, "Two ways to run a request"). A polled read's tail waits for
the SCK pad's idle level; nothing else waits for the wire's end. `status()`
is `spi_ok`, `spi_rejected`, `spi_dma_fault` or
`spi_stalled`. Also `prime(mode, clock, bits)`, `bit_order()`/`lsb_first()` on
the task, `claim_nss_pad()` for a hardware NSS, and the polled transaction
placed by the last parameter, `InFlash` or `InRam` ([the STM32G0 SPI
page](../stm32g0/spi.md)).

### stm32f4

The `SpiClock` enum runs div2..div256 off the instance's own APB clock
(`reference_hz()` is PCLK1 or PCLK2, never SYSCLK), with `ceiling_clock()` and
`clock_for(hz)`. The F1 lineage's peripheral, an older ST design with no FIFO,
so `bits` is 8 or 16 as on the CH32V00x. The engine slots take a (controller,
stream, channel) cell of the request mapping, checked per part class and
refused on a class whose manual was not read, `has_dma` whether they are bound:
a request naming `dma` rides them (`dma_isr()` the one interrupt body). They
refuse a 16-bit frame on byte engines, a 16-bit buffer off a half-word and a
buffer in 0x1000 0000 to 0x1FFF FFFF, the core-coupled memory's region, which
the streams cannot reach; a stream a program holds refuses the block at its
launch (the refusal after the select, "Two ways to run a request"). Each
polled phase's tail and the DMA interrupt wait for the SCK pad's idle level,
BSY falling with RXNE. `status()` is `spi_ok`, `spi_rejected`,
`spi_dma_fault` or `spi_stalled`, the last also from the DMA interrupt's
wait. Also `prime(mode, clock, bits)`,
`bit_order()`/`lsb_first()`, `claim_nss_pad()`, and `sck_speed()`, which the
STM32G0 has too, beside `errata_apb_ceiling_hz()`/`within_errata_ceiling()`,
which only this stratum has, because ES0206 2.12.4 makes the SCK pad's slew
class decide whether the last received bit is captured ([the STM32F4 SPI
page](../stm32f4/spi.md)).

### ch32v00x

The `SpiClock` enum runs div2..div256 with `ceiling_clock()` and `clock_for(hz)`;
`reference_hz()` is HCLK, there being no APB prescaler. The G0's surface on the
F1 lineage's peripheral: `bits` is 8 or 16, the two widths this SPI has. The
engine slots are fixed to DMA channels 3 (TX) and 2 (RX), because on this family
the channel is the request, `has_dma` whether they are bound; `dma_isr()` is the
one interrupt body. They refuse 16-bit frames over byte engines, a 16-bit buffer
at an odd address and a transmit channel another owner holds, before anything
moves. RXNE is not early on this block, so nothing waits for the wire's end.
`status()` is `spi_ok`, `spi_rejected` (a request naming `dma` on a host with no
engines, or one they cannot carry), `spi_dma_fault` or `spi_stalled`. Also `prime(mode, clock, bits)`,
`bit_order()`/`lsb_first()`, `claim_nss_pad()` ([the CH32V00x SPI
page](../ch32v00x/spi.md)).

### ch32vx03

The CH32V00x's surface with a second and, on the CH32V303RC and VC, a third
instance, and so a second reference: `reference_hz()` is the instance's own bus
(PCLK2 for SPI1, PCLK1 for SPI2 and SPI3) and never the system clock, so one
`SpiClock` code is two frequencies on one chip. `ceiling_clock()`/`clock_for(hz)`,
and `SpiRateOf<pclk, hz>`, the compile-time chooser that refuses a ceiling the
bus cannot make where the run-time one only reports it. `bits` is 8 or 16. The
engine slots are fixed per instance (SPI1 on DMA1's channels 3 and 2, SPI2 on its
5 and 4, SPI3 on DMA2's 2 and 1), because on this family the channel is the
request; `has_dma` and the refusal on an engineless host. The engines refuse
16-bit frames over byte engines, a 16-bit buffer at an odd address, a transmit
channel another owner holds and, on the CH32V303's DMA1, a run across a 64 KB
boundary, before anything moves. RXNE is not early on this block, so nothing
waits for the wire's end. `status()` is `spi_ok`, `spi_dma_fault`,
`spi_stalled` or `spi_rejected`. Also `prime(mode, clock, bits)`,
`bit_order()`/`lsb_first()`, `claim_nss_pad()`, `busy()` for the power model,
`pad_speed()`, the slew class of the pads the task drives, a correctness parameter
on a bus of wires, measured in the two modes that sample on the falling edge; and
the resource's high-speed read at BR /2 alone, measured to turn a one-bit-late
read at 36 MHz into an exact one ([the CH32V203 and CH32V303 SPI
page](../ch32vx03/spi.md)).

### ch32x035

The `SpiClock` enum div2..div256 off HCLK (every bus of the series runs at it)
with `ceiling_clock()` and `clock_for(hz)`. The CH32V00x's surface on this series'
copy of the peripheral: `bits` 8 or 16. The engine slots are fixed to DMA channels
3 and 2, `has_dma` whether they are bound; `dma_isr()` is the one interrupt body.
They refuse what the CH32V00x's refuse, before anything moves, and RXNE is not
early here either. `status()` is `spi_ok`, `spi_rejected` (a request naming `dma`
on a host with no engines, or one they cannot carry), `spi_dma_fault` or
`spi_stalled`. Also `prime(mode, clock, bits)`,
`bit_order()`/`lsb_first()`, `claim_nss_pad<arrangement>()`; `init()` takes a
`Config` (the ceiling, `open_drain` for CTLR2.ODEN, the one pad class this GPIO
offers, it having no speed field, and `high_speed_read` for HSRXEN, applied at /2
alone, the bit turning every other code into HCLK/(BR+2), measured) ([the CH32X035
SPI page](../ch32x035/spi.md)).

### rp2040

The PL022: the rate is a `SpiClock` pair (an even prescaler and a serial clock
rate, `SpiClocks::div2..div256` the named ones) with `ceiling_clock()` and
`clock_for(hz)`; `reference_hz()` is clk_peri. `bits` is 8 or 16 in the Request (the
resource takes 4..16). The engine slots go on any two channels told the instance's
requests, `has_dma` whether they are bound; `dma_isr()` is the one interrupt body.
They refuse a 16-bit request on byte engines and a 16-bit buffer off a half-word,
before anything moves. The polled loop keeps eight frames in flight through the
FIFOs. With SPH = 0 each polled phase's tail and the DMA interrupt wait for BSY,
which falls one SCK period after the receive FIFO's flag. `status()` is `spi_ok`,
`spi_rejected` (a request naming `dma` on a host with no engines, or one they
cannot carry), `spi_dma_fault` or `spi_stalled`, the last also from the DMA
interrupt's wait. Also `prime(mode, clock, bits)`, `loopback(on)`
kept through re-application ([the RP2040 SPI page](../rp2040/spi.md), [the PL022
page](../pl022/README.md)).

### rp2350

The same PL022 and the same driver: the resource, the host engine and the client
live in the IP stratum (`pl022/spi.hpp`) and this family writes only a traits type,
so the RP2040's section holds here, its costs apart (one set per architecture, [the
RP2350 SPI page](../rp2350/spi.md)). `reference_hz()` is clk_peri again, 75 Mbit/s
at the top of a 150 MHz one for a host and 12.5 for a client. What is this chip's: the pads repeat in groups of four and the instance is a bit of
the pad number, and unlike its UART this chapter has one function column, so a pin set
is four plain pad numbers; the pads come up isolated, so every configuring function
drops the latch as it writes; and a host's receive pad is handed over with a pull-up,
since an answer line nothing drives must read as the idle a bus expects, which on this
silicon a pull-down would not give ([the RP2350 SPI page](../rp2350/spi.md)).

### host

`SimSpiHost` has no silicon under it: the two phases in one select window, the select
active low, the D/C flipping between them, a null `tx` clocking 0xFF and a select line
with nothing on it reading it back, a `cs_setup_us` counted and never spent, and no wire
at all (no rate, no mode, no bit; its Request's `mode` field is carried and not
applied). Devices are attached to a select pin (`attach`/`detach`) and held type-erased,
so a panel's framing adapter and a stub share one bus; a byte-level trace ring and a
per-pin stamp are what a test asserts a D/C choreography and a select edge against.

Both completion styles are selectable (`completion()`): `immediate` answers every
request inside `start()`, `deferred` holds an asynchronous one (`start()` false,
`finish()` performing it and returning the status the application's DMA glue would
post), which is how an asynchronous transfer is staged with no interrupt anywhere.
`dma` false (`has_dma`) refuses every asynchronous request, `spi_rejected` and the wire
untouched. The simulation still keeps `short_frames()`, a count the test sets below
which a `deferred` host completes a request naming `dma` inside `start()`: no family's
host does that any longer ("Two ways to run a request"), so a test that sets the count
stages a path the families no longer take; left at its default, zero, it changes
nothing.

`test_dcs_link` runs the arbiter over it in both styles, which is what proves it
satisfies this contract and not only its shape, and `test_sim_spi_host` the refusal and
that count at the engine and through the arbiter. Beside it, `SpiBus` is
host-tested over a fake `Bus` (`test_spi_bus`, `test_bus_master`), and the IP stratum's
own driver against a PL022 made of RAM (`test_pl022`, `host/sim_pl022.hpp`), the
realization with no family behind it that proves the driver knows no chip.
