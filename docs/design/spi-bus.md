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

Common to the four: the Request field for field (`cs`, `dc`, `cmd`,
`cmd_len`, `tx`, `rx`, `len`, `reply`, `mode`, `polled`, `cs_setup_us`),
the engine verbs `init`, `start`, `isr`, `rebase`, `recover`, `release`,
`sck_hz`, `max_sck_hz`, and the vocabulary `SpiMode` / `SpiDone` /
`spi_*`. An 8-bit request is spelled identically on all four, and each
engine is measured against a real client on the wire - transactions
queued from one dispatch, a rejection when the queue is full, both
sleep votes, and the per-bus timeout with `recover()` - on the three
platforms marked supported; the CH32V00x's wire letters wait for their
jumper ([the target's document map](../ch32v00x/README.md)).

| stratum | realization | beyond the contract |
|---|---|---|
| avrdx | `SpiHost<n, route>` (`avrdx/spi.hpp`) | the rate is a `SpiClock` division enum, with `ceiling_clock()` the optional SCK ceiling a rebase re-resolves and `clock_for(hz)` the chooser (`spi_clock_for(clk_per_hz, hz)` is the same arithmetic with the clock stated); `status()` is always `spi_ok` - no DMA path, no fault of its own; `prime(mode, clock)` as on the other two |
| samc21 | `SpiHost<n, pads, TxEngine, RxEngine>` (`samc21/spi.hpp`) | the rate is a `uint8_t baud` DIVISOR (the SERCOM's own register), so the ceiling is `ceiling_baud()` and the chooser `baud_for(hz)`; two optional DMA engine slots carrying the data phase (`dma_isr`, `status()` = `spi_ok` or `spi_dma_fault`); `prime(mode, baud)` for a caller framing the select by hand; `reference_hz()` = the stated GCLK rate |
| stm32g0 | `SpiHost<n, pins, TxEngine, RxEngine>` (`stm32g0/spi.hpp`) | a frame size in the Request (`bits`, 4 to 16, eight by default) with `cmd_len`/`len` counted in FRAMES; the `SpiClock` enum with `ceiling_clock()` and the chooser `clock_for(hz)`; the engine slots, `status()` and `prime(mode, clock, bits)` as the SAM's; `bit_order()`/`lsb_first()` on the task; `claim_nss_pad()` for a hardware NSS; `reference_hz()` = PCLK |
| ch32v00x | `SpiHost<1, pins, TxEngine, RxEngine>` (`ch32v00x/spi.hpp`) | the G0's surface on the F1's peripheral: `bits` is 8 or 16 (the two widths this SPI has), the `SpiClock` enum runs div2..div256 with `ceiling_clock()` and `clock_for(hz)`, `prime(mode, clock, bits)`, `bit_order()`/`lsb_first()`, `claim_nss_pad()`, `status()` = `spi_ok` or `spi_dma_fault`; the engine slots are FIXED to DMA channels 3 (TX) and 2 (RX), because on this family the channel IS the request; `reference_hz()` = HCLK, there being no APB prescaler |
| host | none | `SpiBus` is host-tested over a fake `Bus` (`test_spi_bus`, `test_bus_master`) |

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
three-SCK-cycle rule, two on the G0 for its FIFO, one on the CH32V00x).
That integer is the one thing a portable client would need, and each
`SpiClient` publishes it as `frames_ahead`.

The four peripherals share almost nothing below the contract - a
shift register with a two-deep buffer, a SERCOM with a pad matrix, an
SPI with a FIFO and a frame size, and one with no FIFO at all and a
DMA channel per direction - which is what makes the
descriptor's survival worth recording.

`SpiBus` is an alias of `BusMaster<Bus, P>` (`util/bus_master.hpp`),
the arbiter shared with I2C - see [i2c-bus.md](i2c-bus.md).
"SpiDone"/"spi_ok" are the SPI names of `BusDone`/`bus_ok`.

## Why the event queue alone cannot arbitrate

A SPI transaction OUTLIVES the
dispatch that starts it - it completes on interrupts later. While the
bus is busy the kernel happily delivers the next request event, which
therefore needs a place to wait: a small internal pending FIFO in the
AO (main-context only, no critical sections). Full FIFO = the request
is answered IMMEDIATELY with `SpiDone{spi_rejected}` and counted -
never silent, never blocking. The AO is a real 2-state FSM
(idle/busy); the request's `ReplyTo<SpiDone>` capsule is the return
channel, so the AO never knows who its clients are.

The request event (~16-byte descriptor) exceeds the 8-byte envelope
guideline: a recorded, legal deviation - the request IS the
arbitration token; the queues are per-AO, nobody else pays.

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
div16 / mode 0), and the engine reprograms CTRLA/CTRLB at each
`start()` - two register writes between transactions, nothing per
byte. `SpiHost<n>::init()` takes the clock tag and an optional SCK
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

- **ISR pump** (default): one byte per interrupt (no DMA on AVR Dx).
  `SpiHost<n>::isr()` returns true exactly when the transaction completed
  (CS released) - the edge on which the app's ISR glue posts
  `TransferDone{status}` to the bus AO, mirroring the uart edge
  pattern. The kernel keeps dispatching between bytes. Right for slow
  clocks and short transfers; at fast clocks it inverts: a byte at
  div4 flies in 32 CPU cycles while an ISR entry alone costs more, so
  the pump caps the bus near 27% and floods the CPU with interrupt
  overhead (~5 us/byte measured).
- **Polled** (`polled = true`): start() pumps the whole transaction in
  a tight loop and returns done. GLOBAL interrupts stay enabled - only
  the SPI's own IE is silenced (the bound ISR would steal bytes); what
  blocks is that one dispatch, bounded and chosen by the client.
  Measured ~55 cycles/byte at div4 (~2.3 us) with shape-specialized
  loops - the tx/rx null checks are hoisted out because the per-byte
  budget IS the loop body. Known next notch: SPI buffered mode
  (BUFEN + DREIF-gated writes) would close the remaining inter-byte
  gap toward wire speed.

On a synchronous completion the AO replies immediately and keeps
draining the pending FIFO through any further synchronous requests
(`begin_chain`), going `busy` only when a transfer actually stays in
flight. Both styles interleave freely on one bus. A zero-total-length
request completes on the spot, wire untouched - the reply still
arrives (no silent hang).

A synchronous completion is not a success by definition. On an engine
with DMA slots a polled request still moves its bulk over the engines
and `start()` waits on their completion with a bounded budget; a block
that never completes (a channel the silicon stopped, a clock that
stopped, the DMA vector left unbound) ends the request inside
`start()` with `spi_dma_fault` in `status()`, CS raised, and that code
is what the requester's `SpiDone` carries - the same code the
asynchronous path reports through `TransferDone`. The completion
policy (`util/bus_master.hpp`) judges asynchronous completions only:
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
