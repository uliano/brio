# GPIO and EXTI (CH32V00x)

The pads of RM ch. 7 and the external interrupt lines of 6.4 - four
ports of at most eight pins, one configuration nibble a pin, pulls
through the output register, and ten lines of which eight are pins
and two the PVD's and the AWU's. Documents of record: the CH32V00X
reference manual V1.5 (7.3.1.1 for the nibble, 7.3.1.3..7.3.1.5 for
the data and bit-set registers, 6.4 and 6.5 for the lines and their
registers), the CH32V006 datasheet V2.0 (2.1 for the ports each
package bonds, table 2-1-1 for the pads).

## What the silicon does

- **One nibble a pin in CFGLR**, and MODE IS ONE BIT: 1 is output (at
  the port's only speed), 0 input; CNF says push-pull or open-drain
  for an output, floating, pulled or analog for an input; the second
  MODE bit of the STM32F1 shape this register looks like is reserved.
  The reset nibble is 0x4, a floating input.
- **Pulls have no register**: an input with CNF 10 is pulled, and the
  direction is that pin's bit in OUTDR - 1 up, 0 down.
- **BSHR sets from its low half and clears from its high half**, BCR
  clears - the atomic stores; there is no toggle register.
- **An analog pad's input buffer is off**: INDR reads zero on it
  whatever the level.
- **Port B bonds seven pins** on the CH32V006 (PB0..PB6): its eighth
  nibble reads zero after a reset.
- **The EXTI lines**: line n is pin n of ONE port, chosen per line in
  AFIO_EXTICR; lines 8 and 9 are the PVD's and the AWU's; the eight
  pin lines share one vector (exti7_0), the two internal ones have
  their own. A line in INTENR raises a flag and its interrupt; a line
  in EVENR raises a wake event that ends a WFE with no handler and no
  flag (measured: eighteen cycles from a latched event to the WFE's
  return); the flags are write-one-clear.
- **SWIEVR raises a line enabled in INTENR** and nothing on a line
  enabled nowhere (measured).

## Types and verbs

[brio/ch32v00x/pin.hpp](../../brio/ch32v00x/pin.hpp): `Pin<'D', 5>`
(`output(level, drive)`, `input(pull)`, `analog()`, `function(drive)`,
`release()`, `set()`/`clear()`/`toggle()`, `read()`/`read_out()`,
`ref()` for the runtime `PinRef` a bus request carries, the constexpr
`Pad` the pin tables use; a PwmChannel with max 1), `Port<'D'>` (the
registers, `clock_on()` by every configuring verb, the mask verbs),
`pin_nibble(mode, drive)`.
[brio/ch32v00x/exti.hpp](../../brio/ch32v00x/exti.hpp): `Exti` (the
line verbs: `interrupt()`, `event()`, `rising()`, `falling()`, `soft()`,
`flag()`, `clear()`, `port()`) and `ExtInt<'D', 4>` (`init(rising,
falling)` routing the line to the pad, the same verbs for one line).

## How to use it

```cpp
using Button = brio::Pin<'D', 4>;
using Line = brio::ExtInt<'D', 4>;

Button::input(brio::PinPull::up);
Line::init(false, true);           // the falling edge
Line::interrupt(true);
brio::Pfic::enable(brio::Irq::exti7_0);

extern "C" BRIO_CH32_INTERRUPT void exti7_0_handler() {
    if (Line::flag()) { Line::clear(); /* ... */ }
}
```

util/input_scanner.hpp's `InputScanner` takes any type with a `read()`
- `struct Sense { static bool read() { return Button::read(); } };` -
and publishes an `InputEdge` per debounced flip.

## Bench findings

The reference suite is `test_ch32_pin` (8 verdicts in `z`, three
letters on the jumper PD2 to PD4) on the CH32V006K8U6. What the desk
has measured so far is the wireless half:

- **The gate, the nibbles, the atomics**: a configuring verb opens the
  port's clock; a reset port reads 0x4 in every nibble it has; the six
  nibbles land as spelled; BSHR's two halves and BCR do what the
  chapter says, `toggle()` flips.
- **The pulls**: four pads read high pulled up and low pulled down,
  and the direction is OUTDR's bit under nibble 0x8.
- **The software trigger** raises the flag of a line in INTENR, none
  of a line enabled nowhere, and one interrupt per trigger with the
  PFIC line open; **the event mode** ends a WFE eighteen cycles after
  the latched event, no flag raised.

## Not covered yet

Driver gaps, each with its reason:

- The alternate-function remaps (AFIO_PCFR1): born with the first
  driver that needs a moved pad.
- The pin-level bonding table per package: the datasheet's until a
  second part gives it something to say.

Implemented but not bench-verified, each with what would measure it:

- **The levels, the edges and the scanner**: a push-pull output driving
  the far pad, an open-drain output against its pull-up, the edge
  counts on line 4, the port select, and util's InputScanner over a
  driven pad - the suite's letters c, d and f on the jumper.
- A pad wake out of a Standby (sleep.md's gap): the same jumper with a
  Standby armed.
