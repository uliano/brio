# PORT - I/O pins (AVR DA/DB)

> **PROVISIONAL.** Not the result of a systematic review of the PORT
> chapter and errata; the driver covers what every app needs (set,
> clear, read, pull-up, invert, input buffer, PWM-channel duty). The
> exhaustive pass is pending. Documents consulted: AVR128DB28/32/48/64
> data sheet DS40002247B (PORT, PORTMUX where drivers route), errata
> DS80000915F (2.9.1 PD0 floating input on 28/32-pin; 2.2.4 VPORT
> stores after a store to >= 64 - Pin uses SBI/CBI, unaffected).
> Driver: `avrdx/pin.hpp`. Reference tests: every app; `events0` for
> pins as event generators.

## What the driver does today

A pin is a type, `Pin<'D', 2>`: identity (port letter, number, mask) as
constexpr, the VPORT fast path for set/clear/read (single SBI/CBI/
SBIC), the PORT registers for direction and toggle, PINnCTRL for
pull-up, inversion and the digital input buffer. `PinSet<Pins...>`
handles up to 8 pins on any ports as one bit mask. `PinRef` is the
runtime descriptor a pin produces when it must travel inside a request
event (the CS/DC of a bus transaction). A `Pin` is also the degenerate
`PwmChannel` (max 1).

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `Pin<port, n>` | `output()`, `input()`, `set()`, `clear()`, `toggle()`, `read()`, `pullup(bool)`, `invert(bool)`, `disable_digital_input()` / `enable_digital_input()`, `ref()` -> `PinRef`, `duty(v)` (PwmChannel, max = 1); constants `port_letter`, `pin_number`, `mask` |
| `PinSet<Pins...>` | `input(pullup)`, `output()`, `read()` -> mask, `write(mask)`; `count`, `mask` |
| `PinRef` | `set()`, `clear()`; null = no pin (no-ops) |

## How to use it

```cpp
using Led = brio::Pin<'F', 2>;
Led::output(); Led::set(); Led::toggle();

using Keys = brio::PinSet<brio::Pin<'A', 2>, brio::Pin<'A', 3>>;
Keys::input(true);                         // inputs with pull-ups
const uint8_t pressed = ~Keys::read() & Keys::mask;   // active-low buttons

brio::Pin<'D', 1>::disable_digital_input();   // an analog input pin (the ADC driver does it)
```
A pin as an event generator: `EvPin<Pin<'A', 2>>` ([evsys.md](evsys.md)).
A pin inside a request: `Pin<'D', 0>::ref()` ([spi-bus.md](../design/spi-bus.md)).

## Not covered yet

Pin interrupts (ISC: both/rising/falling edges, low level; the PORTx
vectors and INTFLAGS), asynchronous pin-change wake-up from sleep,
slew-rate control per port, input voltage threshold (TTL/schmitt
level, MVIO), the PINCONFIG/PINCTRLUPD multi-pin configuration
registers, PORTMUX as a whole (each driver routes its own pins today:
uart, spi, twi, pwm, evsys), errata 2.9.1 on 28/32-pin parts, the
"one port register write with ST to address < 64" hazard for future
code that does not use SBI/CBI.
