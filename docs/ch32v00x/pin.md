# GPIO and EXTI (CH32V00x)

This document is for a programmer who drives or reads a pad - a LED, a
button, a chip select - who wants an interrupt from a pad's edge, or who
needs a peripheral on another set of pads than its default. It covers
the pads of RM ch. 7 and the external interrupt lines of 6.4: four
ports of at most eight pins, one configuration nibble a pin, pulls
through the output register, and ten lines of which eight are pins
and two the PVD's and the AWU's. A pad is a type here: `Pin<'D', 4>` is
pin PD4, its port and mask constants the compiler folds into each
access, and every function of it is static, so a program names the
type and never makes an object of it. `ExtInt<'D', 4>` is that pad's
external interrupt line, and `afio.hpp`'s tables are the remaps: each
peripheral's pads come as a column, a set of pads one code in
AFIO_PCFR1 selects, and the drivers take their pads from it.

The smallest use, a button on PD4 interrupting on its falling edge,
from "How to use it" below:

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

Line by line: the two `using` lines name the pad and its line (line 4,
since a line is its pin's number); `input(PinPull::up)` makes the pad a
pulled-up input, opening the port's clock on the way (on a port no
earlier call has configured, see the caveat on the first store
below), and there is no
pull register: the pull's direction is the pad's bit in the output
register, which this call writes first; `Line::init(false, true)`
routes line 4 to port D in AFIO_EXTICR and arms the falling edge alone;
`interrupt(true)` lets the line raise its flag and its interrupt;
`Pfic::enable` opens the vector in the interrupt controller
([platform.md](platform.md)), the one the eight pad lines share; the
handler, bound by the program under the startup code's name for that
vector and with this target's one handler attribute
([platform.md](platform.md)), asks the line's flag and clears it by
writing one. An output is `Led::output()` then `set()`, `clear()` or
`toggle()`.

What decides correctness even there. A line is a pin number, not a pad:
PA4 and PD4 share line 4, and only the port `init()` routes it to is
heard. The configuring calls (`input`, `output`, `analog`, `function`,
`release`, and `ExtInt`'s routing and edges) each read, modify and
write a register a whole port or block shares, with no guard, so a
program configures its pads from `main` and not also from an interrupt
handler; `set()` and `clear()` are single stores and `toggle()` a read
of the output register and one store, each atomic against a handler
that touches another pin of the port. A pad must be one the part has:
port B exists on the CH32V006 alone (seven pins there, PB0 to PB6), and
a `Pin` on it that a program uses does not compile for the CH32V003.
And the level a pad starts at: `output(level)` opens the port's clock,
writes the level and only then makes the pad an output, and
`input(pull)` opens the clock, writes the pull's direction (its
output-register bit) and then configures, because a store into a port
whose clock is still gated is lost - measured on the CH32V006K8U6 with
PC3: a `set()` made before the port's first configuration leaves OUTDR's
bit 0 and the pad low, the same store after `Port<'C'>::clock_on()`
lands. The level verbs alone (`set()`, `clear()`, `toggle()`, `read()`)
open no clock and are for a configured port.

The rest of this page, in order: the documents of record; what the
silicon does, the two parts' remap registers and MODE fields among it;
the types and verbs; how to use it; the bench findings, on one jumper
for the levels and edges; and what is not covered.

## Documents of record

The CH32V00X
reference manual V1.5 (7.3.1.1 for the nibble, 7.3.1.3..7.3.1.5 for
the data and bit-set registers, 6.4 and 6.5 for the lines and their
registers), the CH32V006 datasheet V2.0 (2.1 for the ports each
package bonds, table 2-1-1 for the pads). The headers are
[pin.hpp](../../brio/ch32v00x/pin.hpp),
[exti.hpp](../../brio/ch32v00x/exti.hpp) and
[afio.hpp](../../brio/ch32v00x/afio.hpp); the reference suite is
`test_ch32_pin` (a suite's letter, below, is one command key of its
console and the group of verdicts it runs, `z` the key
that runs the image's set of letters, a letter outside it run by its
own key).

## What the silicon does

- **One nibble a pin in CFGLR**, and MODE is one bit: 1 is output (at
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
- **The EXTI lines**: line n is pin n of one port, chosen per line in
  AFIO_EXTICR; lines 8 and 9 are the PVD's and the AWU's; the eight
  pin lines share one vector (exti7_0), the two internal ones have
  their own. A line in INTENR raises a flag and its interrupt; a line
  in EVENR raises a wake event that ends a WFE with no handler and no
  flag (measured: eighteen cycles from a latched event to the WFE's
  return); the flags are write-one-clear.
- **SWIEVR raises a line enabled in INTENR** and nothing on a line
  enabled nowhere (measured) - and a pad's edge obeys the same rule:
  a line enabled nowhere raises no flag on it, a line in INTENR with
  its PFIC line masked raises the flag and leaves it standing for a
  poller (measured).
- **The remaps** (7.2.11, AFIO_PCFR1): one code per peripheral selects
  a column of pads. On the CH32V006, ten columns for TIM1 and USART1,
  eight for TIM2, seven for USART2 and SPI1, five for I2C1, one bit
  for each ADC trigger pad, one for the crystal pads as GPIO, the
  register at offset 0x0C. On the CH32V003 the register is the F1's,
  at offset 0x04: four columns for TIM1, TIM2 and USART1 (whose fourth
  carries the CK pad of its synchronous mode), two for SPI1, three for
  I2C1 (a low bit and a high one), TIM1_IREMAP feeding TIM1_CH1 from
  the LSI, the ADC trigger bits one lower, and the crystal pads' bit
  at 15 with the opposite sense - set for a crystal. SWCFG = 100 turns
  the debug port off until the next reset, on both. USART2's default
  column puts TX on PA7, the CH32V006K8's reset pin; the CH32V003 has
  no USART2.
- **MODE is one bit on the CH32V006 and two on the CH32V003** (its
  RM 7.3.1.1: 01 output at 10 MHz, 10 at 2 MHz, 11 at 30 MHz): the
  stratum drives the CH32V003's outputs at the top speed, so a nibble
  spelled through `pin_nibble()` lands right on both.
- **A timer's forced output level reaches the pad only with the
  counter enabled** (measured: OCxM force modes with CEN clear left
  both pads low; a running PWM at full and zero duty drove them).

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
[brio/ch32v00x/afio.hpp](../../brio/ch32v00x/afio.hpp): the remap
tables as constexpr data (`afio_tim1_pads(code)`, `afio_tim2_pads()`,
`afio_usart1_pads()`, `afio_usart2_pads()`, `afio_spi1_pads()`,
`afio_i2c1_pads()`, the two ADC trigger pads) and `Afio`'s verbs over
PCFR1's fields, `disable_debug_port_until_reset()` spelled long. The
drivers take their pads through it: `SpiPins`/`I2cPins` carry a
`remap` code (`spi1_pins_for(code)`, `i2c1_pins_for(code)` are whole
columns) and init() writes it, `Uart`'s last template parameter is the
code (USART2 refused at 0 on this part), `Tim<n>::remap(code)` moves a
timer and `TimPad` takes a column's pad.

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

The reference suite is `test_ch32_pin` (18 verdicts in `z` on the
CH32V006K8U6; three of its letters sit on the jumper PD2 to PD4 and
decline, claiming no verdict, on a board without it - 10 verdicts on
the CH32V003F4P6 so). On the CH32V003 the port letter takes port C (no
port B there), the nibble letter the part's MODE code, the remap
letter the part's columns - and the remap letter DRAINS THE CONSOLE
before it moves USART1, whose pads are the console's own (measured:
without the drain, the bytes still in the ring leave on the moved pad
and six bytes of line noise take their place). The jumper probe of
every suite pulls the sensing pad AGAINST the level it drives on the
other: a floating pad echoes its neighbour (measured on the
CH32V003F4P6's PC6/PC7), and a probe reading the echo would take it
for the wire:

- **The gate, the nibbles, the atomics**: a configuring verb opens the
  port's clock, and a level asked as a port's first configuration lands
  (the gate closed by hand, `output(true)` on PB0, OUTDR's bit and the
  pad read 1: the verb opens the gate before the store, a store behind a
  shut gate being lost - measured on the CH32V006K8U6; the CH32V003's
  branch of the same letter, on PC3, is compiled and not run); a reset
  port reads 0x4 in every nibble it has; the six nibbles land as
  spelled; BSHR's two halves and BCR do what the chapter says,
  `toggle()` flips.
- **The pulls**: four pads read high pulled up and low pulled down,
  and the direction is OUTDR's bit under nibble 0x8.
- **The software trigger** raises the flag of a line in INTENR, none
  of a line enabled nowhere, and one interrupt per trigger with the
  PFIC line open; **the event mode** ends a WFE eighteen cycles after
  the latched event, no flag raised.
- **The remaps**: every field of PCFR1 reads back as written, and
  TIM1's channel 1 drives PD2 in the default column and PC4 in column
  3 - table 7-8's remap, read on the pads with no wire.
- **The levels, on the jumper**: a push-pull output drives the far pad
  high and low; an open-drain output pulls it low and lets its
  pull-up have it otherwise; an analog pad's input buffer is off
  (INDR zero with the pad high) where an input's is on.
- **The edges, on the jumper**: line 4 counts five rising, five
  falling and ten of both edges of PD2's ten toggles, one interrupt
  each; an edge on a line enabled nowhere raises no flag; the flag of
  a line in INTENR with its vector masked rises on the edge and is
  cleared by writing one; routed to PC4 by AFIO_EXTICR the line hears
  nothing of PD4.
- **The scanner, on the jumper**: util's InputScanner over PD4 with
  PD2 driving it is silent at start and on a held level, publishes
  one InputEdge per flip with the new state, and swallows a glitch
  shorter than its debounce.

## Not covered yet

Driver gaps, each with its reason:

- The pin-level bonding table per package: the part tables state
  ports, not pins, so a `Pin` on a pin of a bonded port that the
  package leaves out (PB7 on the CH32V006, PA0 on the CH32V003)
  compiles. Not written because the build names one package per part
  (the CH32V006K8 and the CH32V003F4), whose pinout the datasheet
  states.
- The remap tables of the CH32V007 (TIM2's and I2C1's differ at one
  column each): this file states the CH32V002/004/005/006 tables and
  the CH32V003's.

Implemented but not bench-verified, each with what would measure it:

- A pad wake out of a Standby (sleep.md's gap): the same jumper with a
  Standby armed.
- The other peripherals on their remapped columns (USART2, which
  exists on this package only remapped; SPI1's and I2C1's alternate
  pads): each is a wire to a peer on the column's pads.
