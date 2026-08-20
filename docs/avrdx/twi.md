# TWI - the I2C engine (AVR DA/DB)

> **PROVISIONAL.** Not the result of a systematic review of the TWI
> chapter and errata; the driver is the host-mode engine the bus AO
> needed for the MCP47CVB22 DAC. The exhaustive pass is pending.
> Documents consulted: AVR128DB28/32/48/64 data sheet DS40002247B
> (TWI, electricals 39.16), errata DS80000915F (2.15.1 output pin
> override on rev A4/A5; 2.15.2 FLUSH non-functional - the driver never
> flushes, recovery is ENABLE off/on). Driver: `avrdx/twi.hpp`; the
> arbiter above it: [i2c-bus.md](../design/i2c-bus.md) (`util/bus_master.hpp`,
> `util/i2c_bus.hpp`). Bench-exercised against the MCP47CVB22 DAC
> (the apps are mapped in bench.md).

## What the driver does today

`Twi<n, TwiRoute>`: host only, interrupt driven, one `Request` per bus
tenure - {7-bit address, tx span, rx span}: write, read, write-then-
read with a repeated START, or the empty probe (address phase only).
Standard (100 kHz) or fast (400 kHz) per request (MBAUD from
`clock_hz(clock)` with the spec's worst-case rise times, both values
folded at init); outcomes `i2c_ok / nack_addr / nack_data / arb_lost /
bus_error`. Pull-ups external. Routes: default (TWI0 PA2/PA3, TWI1
PF2/PF3) or ALT2. A `ClockUser`: `rebase` recomputes both MBAUDs.

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `Twi<n, TwiRoute::def/alt2>` | `init(clock)`, `rebase(hz)`, `start(const Request&)` -> bool, `isr()` -> bool (done; the app posts TransferDone), `status()` |
| `Twi<n>::Request` | `addr`, `tx`/`tx_len`, `rx`/`rx_len`, `reply` (ReplyTo<I2cDone>), `speed` (`I2cSpeed::standard_100k / fast_400k`) |

## How to use it

Through the bus AO: `post<I2cBus>(request)` with `reply_to<Me, I2cDone>()`
- see [i2c-bus.md](../design/i2c-bus.md). Bound once:
```cpp
using TwiHw = brio::Twi<0>;
ISR(TWI0_TWIM_vect) { if (TwiHw::isr()) brio::post<Bus>(brio::TransferDone{TwiHw::status()}); }
TwiHw::init(clock);
```

## Bench findings

- An address sweep finds the MCP47CVB22 at 0x60; write-then-read with
  the repeated-START sequence verified at 100 and 400 kHz.

## Not covered yet

Client mode (address match, general call, address masking, smart
mode), dual mode (separate host/client pins), multi-host arbitration
beyond reporting `arb_lost`, SMBus time-outs, Fast mode plus (1 MHz,
the FM+ pins PF2/PF3), the input noise filter setting, 10-bit
addressing, recovery from a stuck bus (SCL clocking), errata 2.15.1
on rev A4/A5, the electricals' timing table.
