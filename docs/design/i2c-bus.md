# The I2C bus

The second bus in brio, and the specimen that turned the SPI arbiter
into a generic one.

## BusMaster: the arbiter generalized

`SpiBus` never looked at a byte: it owned the pending FIFO, the
reject-when-full policy, the ReplyTo return channel and the engine
handshake (`Bus::start` -> `TransferDone`). The only SPI thing about
it was its name. When I2C needed exactly the same object, the
"generalize on the second real specimen" rule fired: the class is now
`BusMaster<Bus, P, pending_depth>` in `util/bus_master.hpp`, with
`BusDone{status}`, `bus_ok`, `bus_rejected` and `TransferDone` as its
own vocabulary.

The per-bus names survive as zero-cost aliases, on purpose:
`util/spi_bus.hpp` (`SpiBus`, `SpiDone`, `spi_ok`, `spi_rejected`) and
`util/i2c_bus.hpp` (`I2cBus`, `I2cDone`, `i2c_*`). A display client
that reads `SpiDone` says which wire its bytes took; a `BusDone` in
the same place would be less information for no gain. Measured: the
SPI apps' flash is byte-identical before and after.

Status codes are one byte, split by ownership: `bus_ok = 0` and
`bus_rejected = 1` are the arbiter's; every value from
`bus_engine_status = 2` up belongs to the engine's vocabulary and
travels untouched from `TransferDone` to the requester's reply. SPI
has no wire-level outcome to report (no ACK, no arbitration), so its
vocabulary is exactly the arbiter's; I2C adds the four an I2C master
can observe:

| Code | Meaning |
|------|---------|
| `i2c_nack_addr` | no ACK on the address: nobody home (the probe result an address scanner reads) |
| `i2c_nack_data` | no ACK on a written byte: the device refused or finished early |
| `i2c_arb_lost` | lost arbitration to another master: not our bus, no STOP sent |
| `i2c_bus_error` | protocol violation on the wire: peripheral forced back to idle |

## The stack

```
     client AO                          scanner / DAC client
        \  post(Request{...})              /
         v                                v
     I2cBus<Bus, P>          util/i2c_bus.hpp   = BusMaster: arbitration + replies
         |  Bus::start(req)
         v
     Twi<n> engine           avrdx/twi.hpp     START/Sr/STOP, ACK policy, per-byte ISR
         |
     ISR glue in the app     posts TransferDone{TwiHw::status()} on completion
```

Layering as for SPI: `I2cBus`/`BusMaster` are `util/` (pure, tested on
the host through the SPI alias in `test_spi_bus` - same class); `Twi<n>`
is `avrdx/`. The app's ISR binds `TWIn_TWIM_vect`.

## The transaction descriptor (`Twi<n>::Request`)

`{addr, tx span, rx span, reply, speed}` - 9 bytes, one over the
envelope guideline, the same recorded deviation as SPI (the request IS
the arbitration token). One request is ONE bus tenure from START to
STOP, in the shapes I2C devices actually use:

| Shape | Fields | On the wire |
|-------|--------|-------------|
| write | tx set, rx_len 0 | S addr+W data... P |
| read | tx_len 0, rx set | S addr+R data... P |
| write-then-read | both | S addr+W tx... **Sr** addr+R rx... P |
| probe | both empty | S addr+W P -> `i2c_ok` / `i2c_nack_addr` |

Write-then-read is the register-access idiom (index, repeated START,
value) and it MUST be one request: the repeated START is what keeps
another client from slipping in between - the SPI rule "the request is
the complete script of one bus tenure" holds verbatim. The probe is
what a scanner sends and, unlike the SPI zero-length request, it does
touch the wire (its address phase IS the transaction), so `start()`
is always asynchronous on I2C: every request ends in a `TransferDone`.

Bus speed travels per request (`I2cSpeed::standard_100k` /
`fast_400k`), like clock and mode on SPI: a shared bus can carry a
100 kHz sensor and a 400 kHz DAC, and MBAUD is rewritten at each
start (one register, nothing per byte). MBAUD is solved at compile
time from F_CPU with the spec's worst-case rise time for the mode;
stiffer pull-ups make real edges faster, which only slows SCL a touch
below nominal - the safe side.

## Engine behaviour on the wire

- On any NACK the engine still issues STOP: the bus is released
  before the failure is reported.
- Arbitration lost: the flag is cleared, no STOP (the other master
  owns the bus), `i2c_arb_lost` reported.
- Bus error: flag cleared and bus state forced to idle, `i2c_bus_error`.
- ACK policy is explicit (no Smart Mode): each received byte is
  answered by the ISR with ACK + receive-next or NACK + STOP.
- Pull-ups are external (1.5 k on the bench): the internal ones are
  far too weak for I2C edges. The peripheral drives the pins
  open-drain by itself; init only routes them (PORTMUX).

## Not built, noted

- **Stuck-bus watchdog.** A client holding SDA low forever leaves the
  transaction in flight: the kernel keeps running (nothing blocks) but
  the bus AO stays busy and later requests pile up in the pending
  FIFO until they are rejected - loud, not silent, but not recovered.
  The classic remedy (clock SCL nine times, then STOP) plus a
  per-request timeout via a TimeEvent in the bus AO is the known next
  step, to be built when a real device makes it necessary.
- 10-bit addressing, Fast-mode Plus (FMPEN, 1 MHz), client mode.
- A DAC/sensor client AO with word semantics: `util/wire.hpp` already
  gives the big-endian load/store; the MCP47CVB22 driver is the first
  candidate.

Device-specific facts (the DAC's address strapping, the pull-up value)
live in the top-level README bench map and in the header comment of
the app that talks to the device - never here.
