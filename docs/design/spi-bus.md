# The shared SPI bus

The multi-client SPI bus: the "bus AO" pattern in its reference
form ([i2c-bus.md](i2c-bus.md) shares the same arbiter).

## The stack

```
     client AO                           client AO
        \  post(Request{...})              /
         v                                v
     SpiBus<Bus, P>          util/spi_bus.hpp   arbitration + replies
         |  Bus::start(req)
         v
     SpiHost<n> engine      <stratum>/spi.hpp   CS/DC, per-byte ISR pump
         |
     ISR glue in the app    posts TransferDone{status} on completion
```

Layering: `SpiBus` is `util/` (pure, host-testable against a fake Bus);
the `SpiHost<n>` engine belongs to a target stratum and knows the
silicon. The app's ISR binds the vector, as always.

### Realizations

Common to all: the Request field for field (`cs`, `dc`, `cmd`,
`cmd_len`, `tx`, `rx`, `len`, `reply`, `mode`, `polled`, `cs_setup_us`),
the engine verbs `init`, `start`, `isr`, `rebase`, `recover`, `release`,
`sck_hz`, `max_sck_hz`, and the vocabulary `SpiMode` / `SpiDone` /
`spi_*`. An 8-bit request is spelled identically on every row, and each
engine is measured against a real client on the wire - transactions
queued from one dispatch, a rejection when the queue is full, both
sleep votes, and the per-bus timeout with `recover()` - on the three
platforms marked supported; the CH32V00x's wire letters wait for their
jumper ([the target's document map](../ch32v00x/README.md)).

| stratum | realization | beyond the contract |
|---|---|---|
| avrdx | `SpiHost<n, route>` (`avrdx/spi.hpp`) | the rate is a `SpiClock` division enum, with `ceiling_clock()` the optional SCK ceiling a rebase re-resolves and `clock_for(hz)` the chooser (`spi_clock_for(clk_per_hz, hz)` is the same arithmetic with the clock stated); `status()` is always `spi_ok` - no DMA path, no fault of its own; `prime(mode, clock)` as on the other two |
| samc21 | `SpiHost<n, pads, TxEngine, RxEngine>` (`samc21/spi.hpp`) | the rate is a `uint8_t baud` DIVISOR (the SERCOM's own register), so the ceiling is `ceiling_baud()` and the chooser `baud_for(hz)`; two optional DMA engine slots carrying the data phase (`dma_isr`, `status()` = `spi_ok`, `spi_dma_fault`, or `spi_stalled` when a polled wait ran out); `prime(mode, baud)` for a caller framing the select by hand; `reference_hz()` = the stated GCLK rate |
| stm32g0 | `SpiHost<n, pins, TxEngine, RxEngine>` (`stm32g0/spi.hpp`) | a frame size in the Request (`bits`, 4 to 16, eight by default) with `cmd_len`/`len` counted in FRAMES; the `SpiClock` enum with `ceiling_clock()` and the chooser `clock_for(hz)`; the engine slots and `prime(mode, clock, bits)` as the SAM's, `status()` = `spi_ok`, `spi_dma_fault` or `spi_stalled` (no `spi_overrun`: the FIFO is never filled past what the longest handler allows); `bit_order()`/`lsb_first()` on the task; `claim_nss_pad()` for a hardware NSS; `reference_hz()` = PCLK |
| ch32v00x | `SpiHost<1, pins, TxEngine, RxEngine, hold_off_cycles>` (`ch32v00x/spi.hpp`) | the G0's surface on the F1's peripheral: `bits` is 8 or 16 (the two widths this SPI has), the `SpiClock` enum runs div2..div256 with `ceiling_clock()` and `clock_for(hz)`, `prime(mode, clock, bits)`, `bit_order()`/`lsb_first()`, `claim_nss_pad()`, `status()` = `spi_ok`, `spi_dma_fault`, `spi_overrun` (the pump lost a frame to the one-deep receive buffer) or `spi_stalled` (a polled flag never came); the engine slots are FIXED to DMA channels 3 (TX) and 2 (RX), because on this family the channel IS the request; `reference_hz()` = HCLK, there being no APB prescaler |
| ch32vx03 | `SpiHost<n, pins, TxEngine, RxEngine, hold_off_cycles>` (`ch32vx03/spi.hpp`) | the CH32V00x's surface with a SECOND and on the CH32V303RC and VC a THIRD instance, and so a second reference: `reference_hz()` is THE INSTANCE'S OWN BUS (PCLK2 for SPI1, PCLK1 for SPI2 and SPI3) and never the system clock, so one `SpiClock` code is two frequencies on one chip; `bits` 8 or 16, `ceiling_clock()`/`clock_for(hz)` and `SpiRateOf<pclk, hz>` - the compile-time chooser that REFUSES a ceiling the bus cannot make where the runtime one only reports it; `prime(mode, clock, bits)`, `bit_order()`/`lsb_first()`, `claim_nss_pad()`, `busy()` for the power model, `status()` = `spi_ok`, `spi_dma_fault`, `spi_overrun` or `spi_stalled` as on the CH32V00x; the engine slots are FIXED PER INSTANCE (SPI1 on DMA1's channels 3 and 2, SPI2 on its 5 and 4, SPI3 on DMA2's 2 and 1), because on this family the channel IS the request; and `pad_speed()`, the slew class of the pads the task drives - a correctness parameter on a bus of wires, measured in the two modes that sample on the falling edge; and the resource's high-speed read at BR /2 alone, measured to turn a one-bit-late read at 36 MHz into an exact one |
| ch32x035 | none | SPI1 (RM ch. 16) is not written in this stratum |
  
| rp2040 | `SpiHost<n, pins, TxEngine, RxEngine>` (`rp2040/spi.hpp`), the family's alias of the IP stratum's engine (`pl022/spi.hpp`) | the PL022: `bits` 8 or 16 in the Request (the resource takes 4..16), the rate a `SpiClock` PAIR (an even prescaler and a serial clock rate, `SpiClocks::div2..div256` the named ones) with `ceiling_clock()` and `clock_for(hz)`, `prime(mode, clock, bits)`, `loopback(on)` kept through re-application, the engine slots on ANY two channels told the instance's requests, `status()` = `spi_ok` or `spi_dma_fault`; `reference_hz()` = clk_peri; the pump keeps eight frames in flight through the FIFOs |
| stm32f4 | `SpiHost<n, pins, TxEngine, RxEngine, hold_off_cycles>` (`stm32f4/spi.hpp`) | the F1 lineage's block, so `bits` is 8 or 16 as on the CH32V00x; the `SpiClock` enum runs div2..div256 off THE INSTANCE'S OWN APB CLOCK (`reference_hz()` is PCLK1 or PCLK2, never SYSCLK) with `ceiling_clock()` and `clock_for(hz)`, `prime(mode, clock, bits)`, `bit_order()`/`lsb_first()`, `claim_nss_pad()`, `status()` = `spi_ok`, `spi_dma_fault`, `spi_overrun` or `spi_stalled` as on the CH32V00x; the engine slots take a (controller, stream, channel) CELL of the request mapping, checked per part class and refused on a class whose manual was not read; and two verbs no other stratum has - `sck_speed()` beside `errata_apb_ceiling_hz()`/`within_errata_ceiling()`, because ES0206 2.12.4 makes the SCK PAD's slew class decide whether the last received bit is captured |
| rp2350 | `SpiHost<n, pins, TxEngine, RxEngine>` (`rp2350/spi.hpp`) | THE SAME PL022, AND THE SAME DRIVER: the resource, the host engine and the client live in the IP stratum (`pl022/spi.hpp`) and this family writes only a traits type, so every word of the row above holds here. What is this chip's: the pads march in groups of four and the instance is a bit of the pad number, and UNLIKE ITS UART this chapter gained NO second function column, so a pin set is four plain pad numbers; `reference_hz()` is clk_peri again, 75 Mbit/s at the top of a 150 MHz one for a host and 12.5 for a client; and the pads COME UP ISOLATED, so every configuring verb drops the latch as it writes and a host's receive pad is handed over with a pull-UP - an answer line nothing drives must read as the idle a bus expects, which on this silicon a pull-down would not give ([../rp2350/spi.md](../rp2350/spi.md), [../pl022/README.md](../pl022/README.md)) |
| host | `SimSpiHost<n>` (`host/sim_spi_host.hpp`), at the REQUEST and not at the register block | no silicon under it: the two phases in one select window, the select active low, the D/C flipping between them, a null `tx` clocking 0xFF and a select line with nothing on it reading it back, a `cs_setup_us` COUNTED and never spent, and no wire at all - no rate, no mode, no bit. Devices are attached to a select pin (`attach`/`detach`) and held type-erased, so a panel's framing adapter and a stub share one bus; a byte-level TRACE ring and a per-pin stamp are what a test asserts a D/C choreography and a select edge against. BOTH completion styles are selectable (`completion()`): `immediate` answers every request inside `start()`, `deferred` HOLDS an unpolled one - `start()` false, `finish()` performing it and returning the status the app's ISR glue would post - which is how an asynchronous transfer is staged with no interrupt anywhere. `test_dcs_link` runs the arbiter over it in both styles, which is what proves it satisfies this contract and not only its shape. Beside it, `SpiBus` is host-tested over a fake `Bus` (`test_spi_bus`, `test_bus_master`), and the IP stratum's own driver against a PL022 made of RAM (`test_pl022`, `host/sim_pl022.hpp`) - the realization with no family behind it that proves the driver knows no chip |

One of those differences is ONE thing spelled two ways and is
recorded as such: the rate's unit - an enum on two strata, a divisor
on one, with the ceiling and the chooser named for it (`ceiling_baud`/
`baud_for` against `ceiling_clock`/`clock_for`) - an open decision,
since a ceiling in hertz would serve all three.

The client side is deliberately NOT one surface: `SpiClient` on each
stratum is the application's protocol over that silicon's own client
half (a shift register with a two-deep buffer, a SERCOM with PLOADEN, an
SPI with a two-deep FIFO, the F1's single data register), and the peers
of the bench converge on one algorithm - answers kept queued ahead of
what the host has clocked - whose only per-silicon parameter is HOW
MANY must be queued ahead (one on the AVR, two on the SAM for its
three-SCK-cycle rule, two on the G0 for its FIFO, one on the CH32V00x,
eight on both PL022s - the depth of their FIFOs - and one on the
STM32F4; the PL022 client also takes one frame per select window in
modes 0 and 2, [the target's document](../rp2040/spi.md)).
That integer is the one thing a portable client would need, and each
`SpiClient` publishes it as `frames_ahead`.

The peripherals share almost nothing below the contract - a
shift register with a two-deep buffer, a SERCOM with a pad matrix, an
SPI with a FIFO and a frame size, one with no FIFO at all and a DMA
channel per direction, a PL022 with two eight-deep FIFOs and a
request any channel takes, and an F1-lineage block with no FIFO whose
SCK PAD is a correctness parameter - which is what makes the
descriptor's survival worth recording. And where two families carry
THE SAME block, the descriptor stops being a claim: the PL022's driver
is one file both of them alias ([../pl022/README.md](../pl022/README.md)).

`SpiBus` is an alias of `BusMaster<Bus, P>` (`util/bus_master.hpp`),
the arbiter shared with I2C - see [i2c-bus.md](i2c-bus.md).
"SpiDone"/"spi_ok" are the SPI names of `BusDone`/`bus_ok`.

## Why the event queue alone cannot arbitrate

A SPI transaction OUTLIVES the
dispatch that starts it - it completes on interrupts later. While the
bus is busy the kernel happily delivers the next request event, which
therefore needs a place to wait - and it waits where `post()` built it:
the AO HOLDS the request's slot in its own queue (Lease::hold,
[kernel.md](kernel.md) sections 4 and 5) and keeps the slot's number in
a small pending FIFO, one byte per waiting request (main-context only,
no critical sections); the request on the wire is held the same way
until its requester has been answered. Full FIFO = the request is
answered IMMEDIATELY with `SpiDone{spi_rejected}`, counted, and not
held - never silent, never blocking, while the clients keep at most
2 x `pending_depth` + 2 requests outstanding at once (beyond that the
queue itself drops and its `overflows()` counts). The control events (the
completion, a sleep vote, a timeout and its marker) are never held.
The AO is a real 2-state FSM (idle/busy); the request's
`ReplyTo<SpiDone>` capsule is the return channel, read from the held
slot before the slot is released, so the AO never knows who its
clients are.

The queue's depth is counted in those slots: `pending_depth` requests
held waiting, the one held in flight, `pending_depth` more posted
beyond them and waiting to be answered `spi_rejected`, a completion and
a sleep vote (three more on a timed bus: a timeout or two and the
marker), the queue's spare slot being the event being dispatched -
where a request the FIFO has no room for is answered from. The slots
are not reserved by event type: the second `pending_depth` is what
keeps a burst of excess requests from crowding out a completion (an
untimed bus would wedge) or a vote (the manager would wait for ever) -
the headroom the copied FIFO gave when waiting requests lived outside
the queue. A held request costs no copy and no time, however long its
transfer lasts.

The request event (two spans, a select, the bus's settings and the
reply capsule) exceeds the 8-byte envelope guideline: a recorded,
legal deviation - the request IS the arbitration token; the queues are
per-AO, nobody else pays. Its size is paid per COPY, and the arbiter
makes none of its own: every request is built once, by `post()` in a
slot of the bus AO's queue (the kernel's copy, under the producers'
mask), and LENT from that slot to `start()` for the call (the contract
in util/bus_master.hpp) - the slot of the event being dispatched when
the bus is idle, its HELD slot when the request waited. A POLLED
request completes inside `start()` and is copied nowhere else: the
engine reads every field through the reference. An ASYNCHRONOUS one
outlives the call, so the engine copies what its tenure needs - the two
pins, the three buffers, the two lengths, the width and the completion
style, laid out contiguously at the head of the Request so the copy is
a run of word stores - and never the rate, the mode, the setup time or
the reply, which are spent before `start()` returns. (The slot stays
held until the completion has been answered, so an engine could read
the request there instead; whether one does is its own family's
decision, measured on its hot path, and not yet part of the contract.)
A request that waits costs one byte of FIFO and no copy, and a retrying
completion policy starts the held request again from its slot: no copy
and no storage either.

## The transaction descriptor (`SpiHost<n>::Request`)

**The request is the complete script of one bus tenure.** A shared bus
forces per-transaction context (cs, clock, mode: who you are, how you
talk) - that much is the definition of a bus. But the descriptor also
carries protocol STRUCTURE (the D/C flip), and its place there is
forced by a deeper rule: the request is the ATOMIC unit of
arbitration, so anything that must happen inside the CS window without
interleaving must be described in the request. The alternative -
client-side pin toggling between chained requests with a "hold CS"
flag - would let the arbiter interleave another client into an open CS
window, or force it to understand linked tenures (priority inversion
built in). The two-phase cmd/data script with one optional pin is the
smallest script covering every device on the bench; the fully general
form (a segment list with pin actions between segments,
scatter-gather style) is the known successor, to be built when a real
device breaks two phases (QSPI-style dummy cycles, three-phase
protocols). Splitting the descriptor into per-shape request types
would not even save queue RAM while any display client shares the bus:
a queue slot pays the largest variant alternative.

Two phases in ONE chip-select window, covering every device class in
sight:

1. optional `cmd[cmd_len]` transmitted with DC LOW;
2. optional `len` bytes with DC HIGH, full duplex: transmit `tx`
   (or 0xFF dummies if null), capture into `rx` (or discard if null).

| Device class | Shape |
|--------------|-------|
| display-style controller with a D/C line | cmd + tx (or cmd + rx for reads), DC toggles inside the CS window |
| rx-only converter | no cmd, tx null, rx set |
| generic transfer / loopback | tx and rx both set |
| block devices (command protocols) | sequences of plain transfers |

CS is active low, asserted/released by the engine around the whole
transaction; `dc` may be a null PinRef. Both phases are OPTIONAL: a
DC-less device is simply a null `dc` plus a phase-2-only transfer, so
"plain" multibyte transactions (16-bit register devices, delta-sigma
ADC frames of 24-bit groups) are already just tx/rx spans - no display
pattern involved. Buffer ownership travels with the request, and the
cmd/tx/rx fields name it: they are `Lease::reply` loans, so the client
must not touch the buffers until its SpiDone arrives
(run-to-completion makes this race-free).

Word semantics live ABOVE the wire, per the "drivers move bytes"
pillar: `util/wire.hpp` provides constexpr big-endian load/store for
16/24/32-bit words and the sign-extending `load_be24_signed` for ADC
channel data - the client formats/parses its byte spans at the edges.
A device demanding CS-per-word framing would be the one legitimate
engine extension in this area; noted, not built (no such device on the
bench - generalize on the second specimen).

## Per-transaction clock and mode

On a SHARED bus every device names its own speed and mode: the
descriptor carries `SpiClock clock` and `SpiMode mode` (defaults
div16 / mode 0), and at each `start()` the engine folds them - with
the frame size where the block has one - into the control word the
block takes, compares it with the word in force and writes only on a
change, with whatever disable the chapter asks around that write:
nothing on the unchanged path, nothing per frame.
`SpiHost<n>::init()` takes the clock tag and an optional SCK
CEILING for the whole bus - no per-transaction rate, no mode; a
request that asks for more than the ceiling is slowed to it. Rationale: the first two real clients on
the bench already disagreed (a display comfortable at 6 MHz, a touch
controller capped below 2.5 MHz), and a global init-time clock was a
latent bug for every multi-device configuration.

## Two completion styles

`Bus::start(req)` returns bool: FALSE = the transfer runs on the SPI
interrupt and a `TransferDone` will arrive later; TRUE = it completed
SYNCHRONOUSLY inside start(), and the arbiter replies with whatever
the engine's `status()` reports. The choice travels per-request in a
`polled` flag - like the clock, the client knows its transaction.

- **ISR pump** (default): the transaction runs on the block's receive
  interrupt, `SpiHost<n>::isr()` returning true exactly when it
  completed (CS released) - the edge on which the app's ISR glue posts
  `TransferDone{status}` to the bus AO, mirroring the uart edge
  pattern. The kernel keeps dispatching between frames. The handler
  reads the frame that came back FIRST and then writes ahead as far as
  the block's receive side can hold: a FIFO's depth where there is one,
  two frames on a two-level receive buffer, and on a one-deep receive
  register two frames only above a RATE THRESHOLD each family computes
  in its header from named inputs - THE HOLD-OFF, the longest in core
  cycles the image keeps the host's vector from running (the longer of
  its longest handler, since no interrupt nests over another, and its
  longest masked window), the core's exception entry and exit, the
  vector's own entry-to-read - and one frame below it, the bus idle for
  the turnaround. The hold-off is a fact of the IMAGE and not of the
  family, so the host takes it as its last template parameter,
  `hold_off_cycles`, declared by the application or its board file,
  defaulting to the longest the family's own images measure and refused
  at zero; an overrun the declared figure did not foresee ends the
  transaction with `spi_overrun` and is counted - the witness of a
  hold-off declared too short, measured on the three families that take
  it by a host declaring a short one beside the honest default. On a
  one-deep receive register the pump tests the overrun in a status read
  made AFTER the data read: the frame in flight behind the one read may
  complete at any moment before that read, a status copy taken before
  it misses the overrun the completion raises, and on a phase's last
  pair no interrupt follows - a transaction that never completes;
  measured on the CH32V203C8T6, the CH32V303VCT6, the STM32F446RE and
  the CH32V006K8U6 by a sweep of the handler's entry across the frame's
  completion, every run completes at one status load a frame. Right
  where a frame outlasts the handler; faster than that
  the handler bounds the bus and the thread, and the polled style or
  the engines are the bulk path.
- **Polled** (`polled = true`): start() runs the whole transaction in
  a loop and returns done. GLOBAL interrupts stay enabled - the block's
  own interrupt is armed only by an asynchronous tenure; what blocks is
  that one dispatch, bounded by one spin budget per transaction and
  chosen by the client. The loop's SHAPE is the request's buffers': a
  phase with no receive buffer (every command phase, a display's
  pixels) paces on the transmit flag, never reads the answers and
  clears the receive side at its tail - wire-bound from the family's
  second-fastest rate -; a phase with one keeps as many frames in
  flight as the receive side can hold under any handler of the image
  (the same threshold as the pump's on a one-deep register, where one
  frame in flight below it leaves the turnaround on the bus every
  frame), the next frame fetched inside the wire's time. The polled
  path reads the lent request in place and copies nothing; a request
  of a few bytes costs the host a few hundred cycles above the wire on
  every family, measured by letter e of each bench app
  (design/benchmark.md) and stated in each family's SPI document beside
  the vendor's own loop on the same board.

On a synchronous completion the AO replies immediately, and whenever a
transfer ends it keeps draining the pending FIFO through any further
synchronous requests (`begin_waiting`, each answered and released in
turn), going - or staying - `busy` only when a transfer actually stays
in flight. Both styles interleave freely on one bus. A zero-total-length
request completes on the spot, wire untouched - the reply still
arrives (no silent hang).

A synchronous completion is not a success by definition. On an engine
with DMA slots a polled request still moves its bulk over the engines
and `start()` waits on their completion with a bounded budget; a block
that never completes (a channel the silicon stopped, a clock that
stopped, the DMA vector left unbound) ends the request inside
`start()` with `spi_dma_fault` in `status()`, CS raised, and that code
is what the requester's `SpiDone` carries - the same code the
asynchronous path reports through `TransferDone`. Beside it a host may
answer two codes of its own, spelled once in `util/spi_bus.hpp` with
one value on every family (the range `util/bus_master.hpp` leaves to
engines: `spi_dma_fault` first, then these two): `spi_overrun`, a
frame lost to the receive side - a frame landed while the previous one
still stood unread, so every frame went out and one that came back is
missing, and the transaction ends there rather than hand back a run
with a hole in it; and `spi_stalled`, a polled wait that ran out of
the transaction's one spin budget - a flag that never rose, a block
whose clock does not run - with the select released and the receive
side drained. `spi_dma_fault` is defined by the host that has engines,
because only a host with engines can answer it; the two others are
every host's, so a client written over the vocabulary alone tells a
lost frame from a dead block without naming a family. Which of the
three a host can answer is in its row above and in its document. The
completion policy (`util/bus_master.hpp`) judges asynchronous
completions only:
a retry of a failure reported inside `start()` would run inside the
same dispatch, compounding the blocking the polled client bounded on
purpose, so the failure reaches that requester as it is and the
decision to try again is its own.

## Two silicon facts the engine honours

Found with the MCP3550 on the analyzer, both general:

- **SCK must sit at the request's CPOL before CS falls.** Devices that
  latch their SPI mode from the SCK level at the CS edge (the MCP3550:
  mode 0,0 vs 1,1) otherwise start the transaction in the wrong mode.
  The AVR SPI updates the SCK output level when it is ENABLED and at
  every transfer, NOT on a CTRLB write while enabled (a known AVR
  quirk, seen on the analyzer). So the engine applies a CPOL change
  with the peripheral disabled: preset the SCK pin's PORT.OUT to the
  new idle level (what the pin shows while the SPI is off - no glitch),
  disable, write the mode, re-enable - `Spi::apply_mode`, three
  register writes, no clock edges on the bus, only when the polarity
  changes between transactions. (A dummy byte without chip select also
  works; rejected: a side effect on the bus.)
- **CS setup time is a device parameter.** `Request::cs_setup_us`:
  microseconds between CS assertion and the first SCK edge, spun in
  start(). Most devices need nothing beyond the ~1.5 us the code path
  takes; the MCP3550 waking from shutdown needs a few us or it drops
  the frame. Bounded by a byte, main context, chosen by the client
  that knows its device.

## Transaction economics

The per-request fixed cost is the price of ARBITRATION, not of any
descriptor field: the event round trip (request copy into the queue,
dispatch, reply post, requester dispatch) is hundreds of cycles, while
e.g. the unused-D/C share is ~15. It is paid once per request and is
invariant to length - so the defense is making requests BIG, not
fast: batch words into one span (that is why the descriptor speaks
spans), amortize the round trip. For a device that demands CS-per-word
framing, the noted engine extension would batch N words into one
tenure. And a device that owns a bus ALONE can skip the arbiter
entirely: with `polled = true`, `SpiHost<n>::start()` is a complete
synchronous transfer function usable directly by the owning AO - the
arbitration price is only paid where there is something to arbitrate.

## The per-bus timeout

`SpiBus` surfaces `BusMaster`'s `timeout_ticks` (the full design and
its rules in [i2c-bus.md](i2c-bus.md) - one mechanism, both
vocabularies). On SPI the plausible wedge is not a wire - the host
clocks itself - but a DEAD ENGINE whose ISR-style completion never
posts: an AVR host demoted mid-transfer by its SS pin, a DMA channel
stopped by the 1.10.4 class of death, a completion interrupt that
simply never fired. A wedged transaction then comes back `spi_timeout`
on the arbiter's clock, `SpiHost::recover()` having silenced the stale
interrupt, re-armed a demoted host (AVR), put the DMA channels away and
reset the SERCOM (SAM) or run the disable procedure and the RCC reset
(STM32G0), and closed the select window so the device sees the
transaction END. Staged and measured on silicon: a lost interrupt -
the ISR body runs and acknowledges the frames but the `TransferDone` is
never posted - is answered `spi_timeout` in its place, and the very next
four transactions run to `spi_ok` on the same bus AO. Size the limit to the
longest legal transaction; polled requests complete inside `start()`
and never arm it. With `timeout_ticks = 0` (the default) the arbiter
is byte-identical to the untimed one.

## Multi-client rules of thumb

- Each client owns its CS pin (configures it, idles it high) and any
  device-specific pins (DC, RST). Bench hygiene: deselect every device
  on the bus at app init, even those the app never talks to.
- A client keeps at most ONE request in flight and posts the next from
  its SpiDone handler: with N such clients the pending FIFO needs at
  most N-1 slots and rejects never fire (depth 4 default = margin).
- Latency picture: a short request waits at most for the transfer in
  flight (a kilobyte-class transfer at 6 MHz with the ISR pump is a
  few ms) - fine for polling-rate clients; if a client ever needs
  better, that is a scheduling design change, not a FIFO change.

Device-specific facts (controllers, wiring, clock caps) live in the
top-level README bench map and in the header comment of the app that
talks to the device - never here.
