# GPIO - the ports and pins (STM32F4)

This document is for a programmer who needs a pin: an LED, a button, a
pad handed to a peripheral. `Pin<'G', 13>` names pin PG13 as a type (the
letter and the number are template arguments, fixed when the program is
compiled), and its functions configure the pin and drive or read it;
`Port<'G'>` works on a whole port at once, and `PinRef` is a small
run-time handle for a pin that must travel by value inside a request.

The smallest use, the "How to use it" block below: the LED, the user
button and the console's transmit pad of the STM32F429I-DISC1:

```cpp
using Led = brio::Pin<'G', 13>;              // LD3 on the STM32F429I-DISC1
Led::output();                               // push-pull, low speed, no pull
Led::toggle();
using Button = brio::Pin<'A', 0>;            // B1
Button::input(brio::PinPull::down);          // the DISC1's button pulls up when pressed
using Tx = brio::Pin<'A', 9>;                // USART1_TX
Tx::function(brio::PinFunction::af7, {.speed = brio::PinSpeed::high});
```

Line by line: `Led` is a name for the type of PG13, and naming it
configures nothing; `output()` opens GPIOG's clock and makes the pad a
push-pull output at the default options (`PinConfig{}`: no pull, push-pull,
the `low` slew class); `toggle()` flips its level. `Button` is PA0 as an
input with the internal pull-down, so it reads low until the button
drives it high. `Tx` is PA9 handed to USART1 with alternate function 7,
the number the datasheet gives that signal on that pad, at the `high`
slew class (the braces build a `PinConfig` with only `speed` named,
every other field at its default).

What decides correctness even there. A port has a clock and it is off
at reset: with it clear a store into the port is dropped in silence, so
every configuring function (`output`, `input`, `analog`, `function`,
`release`, and `Port<L>::configure_mask` under them) opens the port's
clock before any store it makes (`pull()` is the exception: it rewrites
one field and opens nothing, so it comes after one of them), and the
value functions (`set`, `clear`, `toggle`, `read`) rely on that and do
not: a value function
used on a port nothing has configured yet writes nothing. `output(level)`
opens the clock before it stores the level, so a pad that must start
high is spelled that way. The configuring functions are
read-modify-write sequences on registers the whole port shares: call
them from setup or from an active object's handler (an active object is
a component of a brio program with its own event queue, served one
event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)), never from two
contexts at once, while `set`, `clear` and `toggle` are single BSRR
stores usable from any context. A pad handed to a peripheral is handed
with its alternate-function number, which is the datasheet's and which
nothing in the code can check: the bench is the check. And a pad
nothing configures is an input with no pull, floating, because that is
this family's reset state; `release()` parks a pad in analog mode
instead, which is not the state it came up in.

## Documents of record

RM0090 Rev 22 ch. 8 (RM0390 Rev 6 ch. 7 and RM0383
Rev 4 ch. 8 are its twins), the datasheets' alternate-function tables
(DocID024030 Rev 10 table 12 for the F429, DS10693 Rev 11 table 11 for
the F446, DS10314 Rev 8 table 9 for the F411) for what AFn means on a
pad, and the errata's 2.2.7 for the enable readback; ES0287 2.2.13 and
2.2.14 (PB5's and PA0's input-voltage limits in Standby) are pad facts
of the F411 no verb here can guard. Driver: `stm32f4/pin.hpp` (`Port<L>`,
`Pin<L, n>`, `PinRef`, `PinSet`, the vocabulary `PinMode`, `PinPull`,
`PinSpeed`, `PinFunction`, `PinConfig`, `PinSel`), over the reserve's
`gpio_port_base`/`gpio_port_present`/`gpio_port_clock_mask`
(`stm32f4/device_tables.hpp`, the table of what each part has, read off
the device header). The family fixture is
`test/family_stm32f4/pin.cpp` with the negatives that refuse a port the
header has not got and a seventeenth pin. Bench: the LED and console
pads of the four boards under the blink, console and platform apps.

## What the silicon does

**The STM32G0's register block, without a BRR.** MODER, OTYPER,
OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFRL/AFRH - the same fields at
the same offsets as the G0's. Bitwise reset goes through BSRR's upper
half (8.4.7: bits 31:16 reset, bits 15:0 set, set wins when both are
written), so `clear()` stores `mask << 16` - still one store, still
atomic against a handler touching other pins of the port; a toggle is
one BSRR store with both halves filled from ODR.

**The port has a clock, and it is off at reset.** RCC_AHB1ENR.GPIOxEN
(7.3.10) gates the whole block: with it clear every register of the
port reads as zero and ignores writes, silently. Every configuring verb
but `pull()` switches the port's clock on first (an idempotent read-modify-write of
one RCC bit plus the readback the errata's 2.2.7 asks for); the value
verbs do not, because a pin one has configured has a clocked port by
construction. Nothing turns a port clock OFF - a second pin of the
same port would lose its block; `Port<L>::clock(false)` exists for a
program-wide decision.

**The reset state is input floating, not analog** (8.4.1: MODER resets
to 0 on every pin but PA13/PA14/PA15 and PB3/PB4, the debug port's,
which come up in their alternate function with pulls). The input
buffer is on in input, output and AF modes and off in analog mode
(8.3.12), which is also the lowest-power parking state - `release()`
parks a pad in analog, as on the STM32G0, and that is not this family's
reset state.

**Alternate functions are a per-pin 4-bit number**, and which
peripheral signal AFn means on a given pad is the datasheet's table,
not the reference manual's - a peripheral driver's pin claim names the
AF and no header symbol can check it. Two rows of those tables every
console needs: USART1..3 are AF7, UART4/5/7/8 and USART6 are AF8. The
configuring verb writes the AF nibble before the mode (8.3.2's
procedure), so the pad never spends a cycle on the wrong function.

**The output driver has four slew classes** (OSPEEDR: about 2, 25, 50
and 100 MHz per the datasheets' I/O characteristics, load and supply
dependent); `low` is the reset value of every pin but the debug
port's, and the console's TX runs at it.

**Which ports exist is a bonding fact the header states**: A, B, C and
H on every part; D from the 64-pin F412 up and on every F401/F411; E
from the 100-pin bondings and again on every F401/F411; F and G on the
F405 class, the F412Zx, the F413/F423 and the F446; I on the F405
class and the big packages; J and K on the F42x/F43x and F469/F479
alone. Which pins of a present port a package bonds is finer than that
and stays open.

**There is no pin interrupt in GPIO**: edge and level senses are the
EXTI's (RM0090 ch. 12), through SYSCFG's multiplexer - the EXTI
chapter's ([exti.md](exti.md)).

## Types and verbs

- `port_exists(letter)` (the reserve's presence), `PinRef{port, mask}`
  (a null one whose set/clear are no-ops; `Pin<...>::ref()` builds it),
  `PinMode::input | output | alternate | analog`, `PinPull::none | up |
  down`, `PinSpeed::low | medium | high | very_high`,
  `PinFunction::af0..af15`, `PinConfig{pull, open_drain, speed}`,
  `PinSel{port, pin, function}` with `valid()`.
- `Port<L>` - `regs()`, `clock(on)`/`clock()`, `in()`, `out()`,
  `out_set(mask)`, `out_clear(mask)` (BSRR's upper half),
  `out_toggle(mask)` (one BSRR store), `write_field2(reg, pins, code)`,
  `configure_mask(pins, mode, cfg, fn)` (clock on, AF nibbles, type,
  speed, pull, mode last). Refused at compile time on a letter the
  header has not got.
- `Pin<L, n>` - `mask`, `port_letter`, `pin_number`, `ref()`, the
  PwmChannel pair `max` (1) and `duty(v)`; `set`, `clear`, `toggle`,
  `read`, `read_out`, `is_output`, `has_function`; `output(cfg)`,
  `output(level, cfg)` (the clock opened before the level store),
  `input(pull)`, `analog()`, `function(fn, cfg)`, `release()`
  (analog), `pull(p)`. Refused above pin 15.
- `PinSet<Pins...>::configure(mode, cfg)` - one configure per port for
  a set of pins of several ports.

## How to use it

```cpp
using Led = brio::Pin<'G', 13>;              // LD3 on the STM32F429I-DISC1
Led::output();                               // push-pull, low speed, no pull
Led::toggle();
using Button = brio::Pin<'A', 0>;            // B1
Button::input(brio::PinPull::down);          // the DISC1's button pulls up when pressed
using Tx = brio::Pin<'A', 9>;                // USART1_TX
Tx::function(brio::PinFunction::af7, {.speed = brio::PinSpeed::high});
```

A pin travelling in a bus request: `PinRef cs = Pin<'B', 6>::ref();`
then `cs.clear()` / `cs.set()` from the bus AO - BSRR stores, legal
from any context.

## Bench findings

- LD3 on PG13 (DISC1), LD2 on PA5 (Nucleo-F446RE) and the PC13 LED of
  the black pill blink under the blink app's time events and answer
  the console's `LED ON|OFF|TOG`; the black pill's LED lights when the
  pin is LOW (the LED hangs from 3.3 V).
- The console pads PA9/PA10 at AF7 on the two USART1 boards and PA2/PA3
  at AF7 on the Nucleo: MODER 10 and AFRH nibbles 7 read back over SWD
  (GPIOA MODER 0xA8280000 with the debug pads' 10s beside them, AFR[1]
  0x770), and the consoles run byte-exact - the AF numbers the
  datasheets give are the ones the silicon takes.
- The RX pad's pull-up: an unconnected RX (the black pill before its
  wires were crossed right) read idle, no framing noise, zero received.
- **The port's own options** (`test_stm32f4_exti` letter `q`, PC0 and PC1
  of the Nucleo-F446RE, each read back through its own input buffer):
  the pad follows its pull up AND its pull DOWN; in OPEN DRAIN a 1 is
  released to the pull (it reads 0 over the pull-down and 1 over the
  pull-up) and a 0 is driven, where push-pull drives the 1 over the
  pull-down; `PinSet` configures both pads in one call and one
  `Port<'C'>::out_toggle()` over both flips each; and `release()` parks a
  pad in analog mode, where a pull-up no longer reaches IDR - it reads 0.
  Ten runs of ten.
- `PinRef` from a bus event: the SPI host's Request carries its select
  and its D/C line as `PinRef`s, and `test_stm32f4_spi` drives the select
  through one ([spi.md](spi.md)).

## Not covered yet

Driver gaps:
- Pin interrupts: the EXTI chapter's (`exti.hpp` on this family),
  through SYSCFG_EXTICRx.
- The configuration lock (GPIOx_LCKR's 16-bit key sequence): nothing
  in brio locks a pad; born with its first user.
- A per-package pin-bonding table: the header states ports, not pins,
  and the datasheet's pin table is not carried as data - a Pin on an
  unbonded pad configures a register nobody wired, stated rather than
  refused (the same position as the other STM32 stratum's).
- The compensation cell (SYSCFG_CMPCR, for I/Os toggling at 50 MHz and
  above at 3.3 V - RM0090 9.2.6): nothing here toggles that fast yet.

Implemented, not bench-verified: `PinSpeed` above `low` - the SPI's and
the USART's pads run at `high` and `very_high` on the desk's link
([spi.md](spi.md)), and what a class changes is the edge's slope, which
nothing here can see (a scope on a pad would).