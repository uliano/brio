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
     SpiHost<n> engine      avrdx/spi.hpp     CS/DC, per-byte ISR pump
         |
     ISR glue in the app    posts TransferDone{status} on completion
```

Layering: `SpiBus` is `util/` (pure, host-testable against a fake Bus);
`SpiHost<n>` is `avrdx/` (knows the silicon). The app's ISR binds the
vector, as always.

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
SYNCHRONOUSLY inside start(). The choice travels per-request in a
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
