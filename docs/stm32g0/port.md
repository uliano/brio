# GPIO (STM32G0)

> **PROVISIONAL.** Mode, value, pull, output type, speed and the
> alternate-function handoff are implemented, with the port clock
> opened by every configuring verb; what a pin CANNOT do here by
> design - sense edges and raise interrupts - belongs to the EXTI, and
> that peripheral now has its own driver and its own document
> ([exti.md](exti.md)). What is still missing is in "Not covered yet".

Documents of record: RM0444 Rev 6, GPIO ch. 7 (the EXTI is ch. 13 and
has its own driver and document; the AF-number-to-signal tables are the
DATASHEET's,
DS13560 tables 13..24), errata ES0548 Rev 3 item 2.3.1 (GPIO after a
Standby wake-up; Standby is not entered yet). Driver:
`stm32g0/pin.hpp`; the port-presence facts come from
`stm32g0/device_tables.hpp`. The family fixture is
`test/family_stm32g0/pin.cpp` plus two negatives under
`tools/check_stm32g0.sh`.

## What the silicon does

**The port has a clock, and it is off at reset.** RCC_IOPENR.GPIOxEN
gates the whole block (5.2.17): with it clear every register of the
port reads as zero and ignores writes, in silence. So every
CONFIGURING verb opens the port's clock first (an idempotent RCC
read-modify-write plus the readback that covers the two-cycle enable
delay), the VALUE verbs do not (a configured pin has a clocked port by
construction), and nothing ever closes a port clock behind another
pin's back - `Port<L>::clock(false)` exists for the program-wide
decision. Neither the AVR's PORT nor the SAM's has such a gate.

**Five ports everywhere, six on the G0B1/G0C1.** Ports A, B, C, D and
F exist on every STM32G0 (the device header's GPIOx_BASE, read by the
reserve); E is bonded on the G0B1/G0C1 class only; there is no G or H.
A `Pin` on an absent port is refused at compile time on the header
where it is absent - the family fixture proves it on all three. Which
PINS of a present port a package bonds is finer than that and stays
open, as on the other two targets.

**Reset state is ANALOG** - mode 11, input buffer off, no pull, the
lowest-power state - for every pin except PA13/PA14 (SWD, AF0 with
their pulls: MODER 0xEBFFFFFF, PUPDR 0x24000000, OSPEEDR 0x0C000000
for port A). A pin handed back is put there.

**The input buffer follows the mode**: on in input, output and
alternate modes, off in analog (7.3.1) - no INEN to remember (the
SAM's trap), `read()` means the same thing on every target.

**Alternate functions are a per-pin 4-bit NUMBER** (AFRL/AFRH), and
which peripheral signal AFn is on a given pad is the datasheet's table
(PA2 AF1 = USART2_TX, PA3 AF1 = USART2_RX; AF0 on the same pads is
SPI1/SPI2). The device header carries no pin table, so unlike the SAM
DFP's MUX_* macros nothing here can static_assert a driver's AF claim:
the bench is the check. The AF nibble is written BEFORE the mode
switches (7.4.9's note), so a pad never spends a cycle on the wrong
function.

**Set and reset are atomic, the rest is not.** BSRR/BRR set and clear
ODR bits in one store, from any context - `PinRef` and the value verbs
use them, and a mask toggle is one BSRR store carrying both halves.
MODER/OTYPER/OSPEEDR/PUPDR/AFR are read-modify-write fields with no
set/clear twins: configuring two pins of one port from two contexts
can lose a field, which is why the configuring verbs are for setup and
kernel-time FSM actions.

**There is no pin interrupt in GPIO** - edges are the EXTI's
([exti.md](exti.md)), reached through THE EXTI'S OWN port multiplexer
(`EXTI_EXTICRx`, 13.5.11: on this family SYSCFG has nothing to do with
it, unlike the older STM32 lines), and there is no level sense
anywhere. What a pad keeps is its mode: the EXTI watches the port's
INPUT, which stays live in input, output and alternate modes alike
(7.3.1), so a line can watch a pad that an application drives or that a
peripheral owns - measured. Only ANALOG mode, with the input buffer
off, hides a pad from its line.

**FOUR PADS ARE NOT ORDINARY GPIOs OUT OF A POWER-ON.** 7.3.16 and
SYSCFG_CFGR1's strobe bits: "upon power on, internal pull-down resistors
on UCPD1 CC1 and CC2 pins are enabled (connected)". On this family that
is PA8 and PB15 for UCPD1 and PD0 and PD2 for UCPD2 - a USB Type-C
dead-battery Rd of a few kilohms against the tens of kilohms of the
port's own pull-up - and the chapter's own advice is that "in
applications that do not use the UCPD peripheral, disable the internal
pull-down resistor Rd at startup through the strobe bits in SYSCFG
registers". The strobe only ever RELEASES, and `ucpd_dead_battery()` is
that verb. It lives in this file because its subject is a PAD and this
stratum has no UCPD driver to own it.

## Types and verbs

- `Pin<'A', 5>` - `output()` / `output(level)` (the level written
  before the mode switches, so no glitch through the old ODR),
  `input(PinPull)`, `analog()`, `function(PinFunction, PinConfig)`,
  `release()` (= analog), `pull()`; `set`/`clear`/`toggle`/`read`/
  `read_out`/`is_output`/`has_function`; `ref()` (a `PinRef`);
  `PwmChannel` with `max` 1.
- `Port<'A'>` - `clock(on)`/`clock()`, `in`/`out`, `out_set`/
  `out_clear`/`out_toggle(mask)`, `configure_mask(pins, PinMode,
  PinConfig, PinFunction)`, `write_field2`.
- `PinRef {port, mask}` - `set`/`clear` (BSRR/BRR; a null ref is a
  no-op), `valid()`.
- `PinMode` (input/output/alternate/analog), `PinPull` (none/up/down),
  `PinSpeed` (low/medium/high/very_high = OSPEEDR), `PinFunction`
  (af0..af15), `PinConfig {pull, open_drain, speed}`, `PinSel {port,
  pin, function}` (a driver's pin claim, `valid()` = port present and
  pin < 16), `PinSet<Pins...>::configure`, `port_exists(letter)`.

`ucpd_dead_battery(instance, on)` - release the Type-C dead-battery
pull-downs of UCPD1 or UCPD2 (`on` false; the strobe cannot connect
them again, so true is refused). It opens SYSCFG's clock gate, which is
where the register lives.

## How to use it

```cpp
using Led = brio::Pin<'A', 5>;          // LD4
Led::output();                          // opens GPIOA's clock, push-pull, low speed
Led::toggle();

using Button = brio::Pin<'C', 13>;      // B1 (the Nucleo has its own pull-down)
Button::input();
if (Button::read()) { ... }

using Sda = brio::Pin<'B', 9>;          // an open-drain AF line, fast
Sda::function(brio::PinFunction::af6, {.open_drain = true, .speed = brio::PinSpeed::high});
```

## Bench findings

PA5 (LD4) driven high over SWD through IOPENR + MODER + BSRR before a
line of firmware existed, read back on IDR; then toggled by the raw
probe and by the kernel apps (sampled over SWD at 100 ms: the 500 and
250 ms cadences). PA2/PA3 handed to USART2 at AF1 by `Pin::function`
read back as MODER 10 / AFRL 0x1100 with PA3's pull-up in PUPDR, and
the console answered through them (usart.md). Port A's reset values
read as the chapter states them (MODER 0xEBFFFFFF, OSPEEDR
0x0C000000, PUPDR 0x24000000).

An input pad with nothing attached FOLLOWS ITS OWN PULL, which is what
makes a wireless bench possible on this board: PA0, PB0, PB3, PB7 and
PA8 each read high under an internal pull-up and low under a pull-down
(`test_stm32_exti` letter b), and a pad driven in OUTPUT mode is
readable by another peripheral through the same input path (7.3.1),
where analog mode turns that path off. PC13 (the user button B1) is the
one pad of the survey that does NOT follow: the board holds it high
with an external pull-up stronger than the internal pull-down, so a
press is a falling edge. Bonding, measured over SWD on the LQFP64:
every pin of ports A, B and C answers, port D answers on 0..6, 8 and 9,
and port F only on 0 and 1.

- **PA8 and PB15 really do come up holding the dead-battery Rd**,
  measured in `test_stm32_rtc` letter `k`, where PB15 is also RTC_REFIN:
  the pad does NOT follow its own internal pull-up with the Rd
  connected, and does once `ucpd_dead_battery(1, false)` has released
  it. A push-pull driver wins either way. This is very probably the
  mechanism behind the intermittent PA8 pull-up failure
  [bench.md](../bench.md) records as a desk fault, and it is a fact
  about the silicon's reset state rather than about the desk.

## Not covered yet

Driver gaps: the port lock (GPIOx_LCKR), the alternate-function tables
as data (a
per-package pin table, the SAM device-tables shape), the analog switch
control and the 5 V-tolerance map (DS13560's FT pins - read the table
before a mixed-voltage bench, never assume), erratum 2.3.1 (a
Standby-wake pin's configuration).

Implemented, not bench-verified: `PinSpeed` other than low,
open-drain outputs, `PinSet`, `Port::out_toggle` on several pins at
once, `Pin::pull` on its own, port clocks other than A's, and
`ucpd_dead_battery()` on UCPD2 - whose two pads are PD0 and PD2, which
this package does not bond.
