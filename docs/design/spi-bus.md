# The shared SPI bus

The first multi-client bus in brio, and the proving ground for the
"bus AO" pattern that TWI/I2C will repeat. Decisions of 2026-08-13.

## The stack

```
     client AO                           client AO
        \  post(Request{...})              /
         v                                v
     SpiBus<Bus, P>          util/spi_bus.hpp   arbitration + replies
         |  Bus::start(req)
         v
     Spi<n> engine          avrdx/spi.hpp     CS/DC, per-byte ISR pump
         |
     ISR glue in the app    posts TransferDone{status} on completion
```

Layering: `SpiBus` is `util/` (pure, host-testable against a fake Bus);
`Spi<n>` is `avrdx/` (knows the silicon). The app's ISR binds the
vector, as always.

Since 2026-08-17 `SpiBus` is an alias of `BusMaster<Bus, P>`
(`util/bus_master.hpp`), the arbiter generalized when I2C arrived as
the second bus - see [i2c-bus.md](i2c-bus.md). Everything below about
arbitration reads unchanged; "SpiDone"/"spi_ok" are the SPI names of
`BusDone`/`bus_ok`.

## Why the event queue alone cannot arbitrate

Design discovery worth remembering: a SPI transaction OUTLIVES the
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

## The transaction descriptor (`Spi<n>::Request`)

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
pattern involved. Buffer ownership travels with the request: the
client must not touch the spans until its SpiDone arrives
(run-to-completion makes this race-free).

Word semantics live ABOVE the wire, per the "drivers move bytes"
pillar: `util/wire.hpp` provides constexpr big-endian load/store for
16/24/32-bit words and the sign-extending `load_be24_signed` for ADC
channel data - the client formats/parses its byte spans at the edges.
A device demanding CS-per-word framing would be the one legitimate
engine extension in this area; noted, not built (no such device on the
bench - generalize on the second specimen).

## Per-transaction clock and mode (2026-08-13, evening)

On a SHARED bus every device names its own speed and mode: the
descriptor carries `SpiClock clock` and `uint8_t mode` (defaults
div16 / mode 0), and the engine reprograms CTRLA/CTRLB at each
`start()` - two register writes between transactions, nothing per
byte. `Spi<n>::init()` lost its clock/mode parameters: pins, interrupt
enable, master enable only. Rationale: the first two real clients on
the bench already disagreed (a display comfortable at 6 MHz, a touch
controller capped below 2.5 MHz), and a global init-time clock was a
latent bug for every multi-device configuration.

## Two completion styles (2026-08-13, late)

`Bus::start(req)` returns bool: FALSE = the transfer runs on the SPI
interrupt and a `TransferDone` will arrive later; TRUE = it completed
SYNCHRONOUSLY inside start(). The choice travels per-request in a
`polled` flag - like the clock, the client knows its transaction.

- **ISR pump** (default): one byte per interrupt (no DMA on AVR Dx).
  `Spi<n>::isr()` returns true exactly when the transaction completed
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
entirely: with `polled = true`, `Spi<n>::start()` is a complete
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
