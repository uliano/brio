# Voltage reference (STM32G0)

> **PROVISIONAL, and deliberately so.** The whole of chapter 17 is
> implemented - the four modes of table 91, the two scales, the ready
> bit and the factory trim - and the block is READ on the bench and
> NEVER ENABLED. The reason is in the first section and it is not going
> to change until the board's VREF+ wiring is known from a schematic:
> nothing inside the chip can tell a free VREF+ pin from one tied to
> VDDA, and enabling the buffer on the second is a source driving into
> a regulator. The list is in "Not covered yet".

Documents of record: RM0444 Rev 6 ch. 17; DS13560 Rev 5 section 4
(the LQFP64 pinout, where VREF+ is pin 7 in its own right), section 3.16
and table 67 (the buffer's electrical characteristics); errata ES0548
Rev 3, which has **no item touching VREFBUF**. Driver:
`stm32g0/vref.hpp`, which also carries this target's `brio::Ref` and
`ref_mv()`. Bench suite: `test_stm32_analog` letter a. Family coverage
is in `test/family_stm32g0/adc.cpp`.

## Why `Ref` lives here

`util/analog.hpp` leaves the reference vocabulary to each target -
"every target's vref header defines its own `brio::Ref` under the same
name". On the SAM C21 there was no shared reference block, so each
converter carried its own `REFSEL` enum and `Ref` lived in `adc.hpp`
(`samc/adc.hpp`'s own comment says why). **On this family there IS one
shared rail**: the ADC (15.3.1), the DAC (16.4.6) and the comparators'
VREFINT scaler all work against VREF+, and chapter 17's buffer is the
one thing that can change what that pin is worth. So the enum sits in
the chapter that owns the rail, and `stm32g0/adc.hpp` and
`stm32g0/dac.hpp` include this file.

`Ref` has three values: `external` (VREF+ as the board supplies it -
`ref_mv()` then wants the number, and `Adc::vdda_mv()` is how it is
MEASURED rather than assumed), `buffer_2v048` and `buffer_2v5`.

## Why the buffer is never enabled here

17.1: "When the VREF+ pin is double-bonded with VDDA pin in a package,
the voltage reference buffer is not available and must be kept
disabled." On the LQFP64 the bench chip wears, **VREF+ is a pin of its
own** (DS13560 table 12 gives it pin 7), so the package is not the
problem. The BOARD is: a Nucleo-64 may still tie that pin to VDDA, and
**no register inside the chip can tell the two apart** - a VREF+ at
VDDA reads the same either way, and this project has verified every
board fact it states at the bench rather than from a schematic it does
not have.

So `Vref::enable()` REFUSES unless the caller states, in the config and
by name, that the board leaves the pin free
(`VrefBufConfig::board_vref_pin_is_free`), and the suite never sets it.
What the suite does instead is read the block, which is a real
measurement of the safe default.

The one mode this driver does not offer at all is table 91's ENVR = 0 +
HIZ = 0, "VREF+ pin pulled-down to VSSA": on a board that ties VREF+ to
VDDA that is a path from the supply to ground, and it buys nothing a
disabled buffer does not already give. `disable()` therefore always
leaves HIZ set. Hold mode (ENVR = 1, HIZ = 1) is reachable, because it
drives nothing.

## Bench findings

**THE BLOCK IS BEHIND SYSCFG'S CLOCK GATE, and chapter 17 never says
so.** It names no clock at all; the device header's address map is what
settles it, `VREFBUF_BASE` being `SYSCFG_BASE + 0x30` - the same block
COMP1..COMP3 live in. Measured: **read before RCC_APBENR2.SYSCFGEN is
set, VREFBUF_CSR answers 0x0**, and read after it, 0x2. Zero is not this
register's reset value: it is table 91's OTHER off mode, the one that
pulls VREF+ down to VSSA. So an application that read this register
through the closed gate would conclude the pin was being grounded when
it is not - which is 5.2.17's rule biting in the one place where its
answer is not merely absent but WRONG. `Vref::init()` opens the gate and
every verb assumes it has been called.

**The reset state is the safe one.** With the gate open, VREFBUF_CSR
reads **0x2** out of a cold boot: ENVR clear and HIZ set, table 91's
"external voltage reference mode", VREF+ an input pin. Nothing in this
stratum leaves it.

**The factory trim is loaded**, as 17.3.2 promises: VREFBUF_CCR.TRIM
reads **36** on this die, and the scale bit comes up at 2.048 V.

**`enable()` refuses** without the board acknowledgement, and after the
refusal ENVR is still clear - the refusal writes nothing at all.

**VREF+ is at VDDA on this board**, as far as anything inside the chip
can see: `Adc::vdda_mv()` puts it at 3310 mV, which is what a Nucleo's
3.3 V regulator produces. That is consistent with a bonded-and-bridged
pin and with a separately supplied one, and it is exactly the ambiguity
that keeps the buffer off.

## Not covered yet

**Implemented but not bench-verified - all of it, and on purpose:**

- `enable()` at either scale, and therefore VRR, the start-up time and
  the 2.048 V / 2.5 V levels themselves. Closing this gap does not need
  code: it needs the MB1360 schematic (or a measured VREF+ pin) to say
  whether that pin is free, and then one run.
- Hold mode.
- The ADC, DAC and comparators against a reference that is not the
  supply. Every absolute millivolt this stratum reports today is
  ratiometric to VDDA, measured through VREFINT.

**Deliberately absent:**

- A setter for VREFBUF_CCR.TRIM. 17.3.2's own note makes a user trim an
  ascending sweep from zero, which is a calibration procedure and not a
  setter, and nothing here has a reason to want it.
- Table 91's ENVR = 0 + HIZ = 0 mode, for the reason above.
