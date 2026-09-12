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
  enabled nowhere (measured) - and a pad's edge obeys the same rule:
  a line enabled nowhere raises no flag on it, a line in INTENR with
  its PFIC line masked raises the flag and leaves it standing for a
  poller (measured).
- **The remaps** (7.2.11, AFIO_PCFR1): one code per peripheral selects
  a COLUMN of pads. On the CH32V006, ten columns for TIM1 and USART1,
  eight for TIM2, seven for USART2 and SPI1, five for I2C1, one bit
  for each ADC trigger pad, one for the crystal pads as GPIO, the
  register at offset 0x0C. On the CH32V003 the register is the F1's,
  at offset 0x04: four columns for TIM1, TIM2 and USART1 (whose fourth
  carries the CK pad of its synchronous mode), two for SPI1, three for
  I2C1 (a low bit and a high one), TIM1_IREMAP feeding TIM1_CH1 from
  the LSI, the ADC trigger bits one lower, and the crystal pads' bit
  at 15 with the OPPOSITE sense - set for a crystal. SWCFG = 100 turns
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
  port's clock; a reset port reads 0x4 in every nibble it has; the six
  nibbles land as spelled; BSHR's two halves and BCR do what the
  chapter says, `toggle()` flips.
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

- The pin-level bonding table per package: the datasheet's until a
  second part gives it something to say.
- The remap tables of the CH32V007 (TIM2's and I2C1's differ at one
  column each): this file states the CH32V002/004/005/006 tables and
  the CH32V003's.

Implemented but not bench-verified, each with what would measure it:

- A pad wake out of a Standby (sleep.md's gap): the same jumper with a
  Standby armed.
- The other peripherals on their remapped columns (USART2, which
  exists on this package only remapped; SPI1's and I2C1's alternate
  pads): each is a wire to a peer on the column's pads.
