# The I2C bus

The I2C bus, served by the same generic arbiter as SPI.

## BusMaster: the arbiter generalized

The arbiter never looks at a byte: it owns the pending FIFO, the
reject-when-full policy, the ReplyTo return channel and the engine
handshake (`Bus::start` -> `TransferDone`). Nothing in that is
bus-specific, so one class serves both buses:
`BusMaster<Bus, P, pending_depth>` in `util/bus_master.hpp`, with
`BusDone{status}`, `bus_ok`, `bus_rejected` and `TransferDone` as its
own vocabulary.

The per-bus names are zero-cost aliases, on purpose:
`util/spi_bus.hpp` (`SpiBus`, `SpiDone`, `spi_ok`, `spi_rejected`) and
`util/i2c_bus.hpp` (`I2cBus`, `I2cDone`, `i2c_*`). A display client
that reads `SpiDone` says which wire its bytes took; a `BusDone` in
the same place would be less information for no gain. Measured: the
aliases cost zero flash.

Status codes are one byte, split by ownership: `bus_ok = 0` and
`bus_rejected = 1` are the arbiter's; every value from
`bus_engine_status = 2` up belongs to the engine's vocabulary and
travels untouched from `TransferDone` to the requester's reply. The
engine borrows `i2c_rejected` (= `bus_rejected`) for the one refusal
it makes on its own, a speed the clock in force cannot produce: the
same meaning for the requester either way - nothing was executed, no
byte moved. SPI
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
     I2cHost<n> engine       <stratum>/{twi,i2c}.hpp   START/Sr/STOP, ACK policy, per-byte ISR
         |
     ISR glue in the app     posts TransferDone{I2cHw::status()} on completion
```

Layering as for SPI: `I2cBus`/`BusMaster` are `util/` (pure, tested on
the host through the SPI alias in `test_spi_bus` - same class); the
`I2cHost<n>` engine belongs to a target stratum, as a task over that
silicon's own resource. The app's ISR binds the vector.

### Realizations

Common to the five: the Request field for field (`addr`, `tx`,
`tx_len`, `rx`, `rx_len`, `reply`, `speed`), `I2cSpeed` word for word
(the CH32V00x's stops at `fast_400k`: its peripheral has no Fm+), the
engine verbs `init`, `start`, `isr`, `status`, `rebase`, `recover`,
`release`, `unstick`, the empty probe (both lengths zero) served by
all five - on the RP2040 as a one-byte read, its command FIFO having
no address-only entry -, and the vocabulary `I2cDone` / `i2c_*`
produced ON THE WIRE by each - measured against a second chip on four
and against the chip's other instance on the RP2040. What differs is
the resource under the task and the shape of the rate arithmetic each
chapter imposes.

| stratum | realization | beyond the contract |
|---|---|---|
| avrdx | `I2cHost<n, route>` over `Twi<n>` (`avrdx/twi.hpp`), a resource that carries the client half too | the rate readback is `actual_scl_hz(t_rise_ns)` - what the register in force gives, under the chapter's rise-time budget; `speed_ok(speed)` as on the other two, plus `speed_ok()` for the speed in force; `quick_command(bool)` is the TWI's own hardware MODE (QCEN: every request an address-only frame); `bus_state()` is the WIRE's state from the bus monitor; `speed()`, `baud()` |
| samc21 | `I2cHost<n, pads>` over `I2cm<n>` (`samc21/i2c.hpp`; `I2cs<n>` is the client's resource, two over one SERCOM) | `init` takes the STATED core rate (`reference_hz()`); the rates are a per-speed cache: `speed_ok(speed)`, `scl_hz(speed)`, `baud_of(speed)`; `idle()` is the ENGINE's phase, not the wire's |
| stm32g0 | `I2cHost<n, pins, TxEngine, RxEngine>` over `I2c<n>` (`stm32g0/i2c.hpp`, one TIMINGR word) | the same stated rate and per-speed cache (`speed_ok`, `scl_hz`, `timing_of`, `kernel_hz()` for the kernel-clock multiplexer); two optional DMA engine slots with `dma_isr()`; `fast_plus_drive()` (SYSCFG's Fm+ pad drive); `spurious_bus_errors()` (an erratum's counter); `idle()` as the SAM's |
| ch32v00x | `I2cHost<1, pins, TxEngine, RxEngine>` over `I2c<1>` (`ch32v00x/i2c.hpp`, the F1's event machine with no rise-time register) | the stated bus rate and the per-speed cache as the SAM's (`speed_ok`, `scl_hz`, `timing_of`, `reference_hz()` = HCLK); a `duty` at init (fast mode's 2 or 16/9 shape); TWO vectors, `isr()` for the events and `error_isr()` for AF/ARLO/BERR; the engine slots FIXED to DMA channels 6 (TX) and 7 (RX) with `dma_isr()`, a one-byte read kept on the pump; `unstick()` returns the clocks it took; `start()` WAITS, bounded, for the last STOP to leave (the F1 lineage cannot queue a START behind one) - the one engine whose `start()` spends dispatch time on the bus |
| rp2040 | `I2cHost<n, pins, TxEngine, RxEngine>` over `DwApbI2c<n>` (`rp2040/i2c.hpp`, the Synopsys command FIFO: an entry carries its RESTART and STOP bits, one TX_ABRT carries every failure) | the stated rate (clk_sys, `reference_hz()`) and the per-speed cache as the SAM's (`speed_ok`, `scl_hz`, `timing_of`); `idle()` as the SAM's; `start()` answers a SECOND refusal synchronously, i2c_bus_error for a block that would not disable to take the address (IC_TAR takes a write only with ENABLE clear); the probe is a one-byte read; the engine slots serve the READ phase alone (a transmit engine of uint16_t entries pouring one plain read command from a fixed cell, the receive engine collecting) with `dma_isr()`; `unstick()` returns the clocks it took; `recover()` resets the block through the reset controller, the one way out of a host holding SCL with no STOP in sight |
| host | none | `I2cBus` = `BusMaster`, host-tested through the SPI alias (`test_spi_bus`, `test_bus_master` - the same class) |

None of those differences is a spelling of the same function:
`actual_scl_hz` and `scl_hz`
answer different questions (the register in force against a cached
speed), `bus_state()` and `idle()` look at different things (the wire
against the engine), and `quick_command` is a hardware mode the other
families do not have (their empty probe is the same tenure with no
data, which the AVR serves too). `speed_ok(speed)` is one verb on the
five.

The client side is five surfaces by position (`I2cClient` on each
stratum, the application's protocol over `Twi<n>` / `I2cs<n>` /
`I2c<n>` / `I2c<1>` / `DwApbI2c<n>`): samc21, stm32g0, ch32v00x and
rp2040 speak one vocabulary back (`addressed`, `data_ready`,
`stop_seen`, `give`/`take`, `host_reads`; the CH32V00x and the RP2040
add `service()`, one `I2cClientEvent` per ISR body; the RP2040 has no
`addressed`, its block raising no address-match event - a tenure is
known by its first byte or its first read request, and the clock
stretch of a read request is the block's own), the
AVR's is its own (`addressed`, `data_ready`, `receive`/`respond`,
`complete`) - a convergence of four, not a task.

The five peripherals share almost nothing below the contract - a TWI
with its own baud three-step, a SERCOM with a select-then-use register
file, an I2C with one timing word, the F1's event machine with a
clock register and no rise-time one, and a command FIFO whose entries
carry the bus conditions as bits - which is what makes the
descriptor's survival worth recording.

## The transaction descriptor (`I2cHost<n>::Request`)

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
is asynchronous on I2C whenever the wire moves: every such request
ends in a `TransferDone`. The one synchronous completion is the
REFUSAL: a request naming a speed the clock in force cannot produce
(`speed_ok(speed)` says so) is answered `i2c_rejected` inside
`start()`, through the engine's `status()`, no byte moved - never run
at a rate nobody asked for, and never approximated in silence. The
arbiter replies with `status()` on every synchronous return
(`util/bus_master.hpp`'s contract), so the requester sees the refusal
in the same `I2cDone` as any outcome of the wire; the completion
policy does not judge it, and no retry would change it. Every engine
refuses the same way: status set, nothing armed, true returned.

The tx and rx buffers are `Lease::reply` loans and say so in their
field types: the requester keeps them alive and untouched until its
I2cDone arrives.

Bus speed travels per request (`I2cSpeed::standard_100k` /
`fast_400k` / `fast_plus_1m`), like clock and mode on SPI: a shared bus
can carry a 100 kHz sensor and a 400 kHz DAC. MBAUD may only be written
with the host disabled, so a speed CHANGE costs an ENABLE cycle and a
force-idle at `start()` - paid only when the speed actually moves,
never per byte. How MBAUD is solved, and why the bus's rise and fall
times are arguments rather than assumptions, belongs to the engine:
[twi.md](../avrdx/twi.md).

## Engine behaviour on the wire

- On any NACK the engine still issues STOP: the bus is released
  before the failure is reported.
- Arbitration lost: the flag is cleared, no STOP (the other master
  owns the bus), `i2c_arb_lost` reported.
- Bus error: flag cleared and bus state forced to idle, `i2c_bus_error`.
- ACK policy is the engine's: each received byte is answered with
  ACK + receive-next or NACK + STOP. Smart Mode is an option of the
  engine (the acknowledge then rides the data read), invisible here.
- Pull-ups are external (1.5 k on the bench): the internal ones are
  far too weak for I2C edges. The peripheral drives the pins
  open-drain by itself; init only routes them (PORTMUX).

## The per-bus timeout

A client holding a line low forever - SDA, or the clock in mid-byte -
leaves a tenure in flight: the kernel keeps running (nothing blocks)
but the bus AO stays busy and later requests pile up until rejected -
loud, but not recovered. And no silicon answers a held clock, ON ANY
OF THE FIVE: the SAM SERCOM's SMBus
time-outs police the HOST'S OWN clock hold, not a wire a client wedged
(measured - [i2c.md](../samc21/i2c.md)); the AVR's TWI has none at all;
and the STM32G0's - which RM0444 32.4.12 describes in words that read
otherwise ("if SCL is tied low for longer than ...") - were measured with
a control on each side and answer the same way: the host's own unserved
hold trips TIMEOUTA, a peer's 6 ms hold trips neither TIMEOUTA nor
TIMEOUTB ([i2c.md](../stm32g0/i2c.md)). On that part a wedged SDA does
not even raise an error: the START simply PARKS with BUSY standing,
which is the AVR's and the SAM's behaviour met a third time. The
CH32V00x's F1-lineage I2C has no clock-low timeout either - a client
holding SCL for 45 ms stands until the arbiter answers at its 20 - and
it is the one that answers a wedged SDA by itself: its START into a
busy bus comes back ARLO at once, so the requester reads
`i2c_arb_lost` well before any limit ([i2c.md](../ch32v00x/i2c.md)). The
RP2040's DW_apb_i2c has no clock-low timeout either: a client holding
SCL through a read request it never serves stands until the arbiter
answers at its 20 ms ([i2c.md](../rp2040/i2c.md)). The
same held SDA, two honest replies - parked and timed out on three
strata, refused by the silicon on the fourth - which a requester reads
alike, "the bus is not mine", the recovery ladder staying the
application's. So the timeout is the ARBITER'S - the one object that
knows a completion is owed, living in the kernel that has TimeEvents.

`BusMaster`'s `timeout_ticks` template argument (surfaced here as
`I2cBus`'s) arms a one-shot TimeEvent for every tenure that goes
asynchronous. If it matures first, the engine is declared dead:
`Bus::recover()` puts the PERIPHERAL back where `start()` is legal
(the AVR `I2cHost`'s ENABLE-cycle errata work-around, the SAM's cached
re-init, the G0's PE cycle - which RM0444 32.4.6 offers for
exactly this, "restores the normal operation ... by toggling the PE
bit" -, the RP2040's block reset through the reset controller, because
a host holding SCL with no STOP in sight never disables), the requester is answered `i2c_timeout` in its place, and
the queue moves on. The races with the real completion are closed by
construction (a sequence number and a drain state - the whole story in
`util/bus_master.hpp`, staged deterministically in the host suite).

Three decisions worth their line:

- **Per BUS, not per request.** One wedged device starves every client
  of the wire, so the limit is a property of the bus. It must sit
  ABOVE legal clock stretching - flow control, not a fault - so size
  it to a whole worst-case tenure at the slowest device, and convert
  with `ticks_from_ms<P>()`.
- **The WIRE stays the application's.** The timeout guarantees only
  that the bus AO and its queue survive to be asked; whether to
  `unstick()`, power-cycle a client or re-probe the bus is the
  recovery ladder below, still policy. After a fault, re-verifying
  the clients one by one is the application's decision.
- **A timeout is not a completion**: the retry `Policy` is never
  consulted (the engine never spoke).

With `timeout_ticks = 0` (the default) none of this exists - the
generated code is byte-identical to the untimed arbiter's, the
`never_retries` discipline again.

## Not built, noted

The arbiter carries the HOOK a recovery policy would hang from -
`BusMaster`'s `Policy` template argument, whose `on_done(status,
attempt)` can ask for the same request to be started again (see
`util/bus_master.hpp`); the concrete I2C ladder below and the
multi-host backoff remain on demand.

- **The recovery LADDER.** The mechanical verbs exist at engine level
  (`unstick()` clocks SCL up to nine times until a stuck client
  releases SDA - see [twi.md](../avrdx/twi.md)) and the timeout above
  notices the wedge and reports it; the POLICY connecting them - when
  to unstick, when to retry, when to take the bus out of service - is
  the application's, or a future policy type's, born with the device
  that needs it.
- **Multi-host policy.** The engine reports `i2c_arb_lost` and the
  arbiter passes it to the requester untouched. A bus AO that knows it
  shares the wire - retrying a lost tenure, backing off, refusing to
  start while another host is mid-transaction - is not designed.
- 10-bit addressing (the arbiter's descriptor has no shape for it).
- A DAC/sensor client AO with word semantics: `util/wire.hpp` already
  gives the big-endian load/store; the MCP47CVB22 driver is the first
  candidate.

Device-specific facts (the DAC's address strapping, the pull-up value)
live in the top-level README bench map and in the header comment of
the app that talks to the device - never here.
