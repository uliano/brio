# VREF - the voltage reference selector (AVR DA/DB, AVR EA)

This document is for a programmer who reads the ADC in volts, sets the
DAC to a voltage or gives a comparator a threshold, and so has to say
what full scale is. The VREF block picks, for each analog consumer, the
voltage its full scale stands for. The driver is a vocabulary rather
than a device: `Ref` names this silicon's six references, `ref_mv()`
gives their millivolts to whoever converts counts, and `Vref` is a
class of three setters with no instances (`dac0()`, `ac()`, and on the
DA/DB `adc0()`). A program rarely calls the setters: it names its
reference in the ADC's, the DAC's or the comparator's configuration,
and that driver's `init()` writes it.

The smallest use, a DAC on the 2.048 V reference set to 1.5 V, taken
from "How to use it" below:

```cpp
Dac<0>::init({.reference = Ref::v2048});                         // sets VREF.DAC0REF
Dac<0>::set(dac_code(1500, Dac<0>::steps, ref_mv(Ref::v2048)));  // 1.500 V
```

Line by line: `init()` configures the DAC ([dac.md](dac.md)) and, as
its first act, writes VREF.DAC0REF with the 2.048 V selection;
`ref_mv(Ref::v2048)` is 2048, the millivolts of that reference;
`dac_code()` (`util/analog.hpp`) turns 1500 mV into the code for a
1024-step converter on that reference; `set()` writes it.

What decides correctness even there. The reference named in the
configuration and the one handed to `ref_mv()` are two arguments, and
nothing ties them: the arithmetic is right only when they are the same
(`Dac<0>::reference()` and `Adc<0>::reference()` return what `init()`
chose). An internal reference needs headroom on the supply, the table
below, and nothing in the code checks VDD. For `Ref::vdd` and
`Ref::vrefa` the level is whatever the board supplies: `ref_mv()`
returns the millivolts the caller passes, 0 when it passes none, and
every conversion through 0 yields 0. On the AVR EA the ADC does not
select its reference here at all but in its own CTRLC, with another
encoding (`adc_refsel()` in `avrxt/adc.hpp`).

## Documents of record

AVR128DB28/32/48/64 data sheet DS40002247B,
errata DS80000915F (no VREF items); AVR64EA28/32/48 data sheet
DS40002443A (chapter 20, the ADC's own selection in 31.5.3, table
35-17), errata DS80001048D (no VREF item). Driver: `avrxt/vref.hpp`
(the `Ref` vocabulary is defined there; the counts <-> mV arithmetic
is in `util/analog.hpp`). Reference test: `test_avr_analog` (on the
DA/DB test 1, 14; on the EA letters 1, v). A letter is one command key
of the suite's console and the group of verdicts it runs, `a` the key
that runs them all on either half; the DA/DB half numbers its tests 1
to 15 and types the tenth to the fifteenth as `0`, `w`, `e`, `i`, `v`
and `x`, so its test 14 is key `v`.

## What the silicon does

On the DA/DB, three independent reference selections, one per
consumer - `ADC0REF`, `DAC0REF`, `ACREF` (the three analog comparators
share one). On the EA two - `DAC0REF` and `ACREF`, the same bits and
codes (20.5.1, 20.5.2) - and the ADC selects its reference in its own
CTRLC with another encoding (VDD 0, VREFA 2, 1.024 V 4, 2.048 V 5,
4.096 V 6, 2.5 V 7: `adc.hpp`'s `adc_refsel()`; `Vref::adc0()` does not
exist there). Each selects one of:

| `Ref` | Level | Needs VDD (DA/DB) | Needs VDD (EA) |
|-------|-------|-----------|-----------|
| `v1024` | 1.024 V internal | - | 1.8 V |
| `v2048` | 2.048 V internal | >= 2.5 V | 2.55 V |
| `v2500` | 2.500 V internal | >= 2.95 V | 3.0 V |
| `v4096` | 4.096 V internal | >= 4.55 V | 4.6 V |
| `vdd` | the supply | - | - |
| `vrefa` | external, pin PD7 (1.024 V .. VDD) | - | - |

On the EA the ADC adds its own rule: an internal reference at least
0.5 V under VDD (31.3.3.5).

Internal levels are +-4 % untrimmed over temperature on the DA/DB;
measured on the bench they agree with each other within 1-2 %. On the
EA +-2 % (1.024, 2.048 V) and +-3 % (2.5, 4.096 V) to 85 C, a source
starting in 130 us typical, 180 maximum (tVREF_ST, table 35-17). On the
DA/DB a source turns on by
itself when its consumer asks for it (start-up 10 us when the main
clock is a high-frequency one - internal OSCHF or external
EXTCLK/XOSCHF - and 200 us when it is a 32.768 kHz one, internal or
crystal: Table 39-17 keys on MCLKCTRLA.CLKSEL; 2 us to change level);
`always_on` keeps it powered for ~40-175 uA so the consumer never
waits. VREFA loads the external source with ~50 kOhm (the ADC's
ladder). There is no other behaviour: VREF is a selector.

What it means for code: the reference is part of the meaning of every
conversion (counts <-> volts); the ADC and the DAC may use different
ones; a DAC -> ADC loop is exact only when both are known.

## How to use it

You rarely touch VREF directly: the ADC, DAC and AC configurations
name their reference and their `init()` selects it.

```cpp
#include "avrxt/adc.hpp"
#include "avrxt/dac.hpp"
#include "avrxt/ac.hpp"

Dac<0>::init({.reference = Ref::v2048});                         // sets VREF.DAC0REF
Adc<0>::init(clock, AdcConfig{.reference = Ref::v2048});         // sets VREF.ADC0REF (the EA: ADC.CTRLC)
Ac<0>::init({.negative = AcNeg::dacref, .reference = Ref::v2048}); // sets VREF.ACREF (DACREF)
```

On the AVR EA the `Adc<0>::init()` line above returns false as written:
with an internal reference and no PGA that converter wants
`sample_duration` of at least 4 us x fCLK_ADC (8 at 2 MHz, its default
prescaler's rate on a 20 MHz clock) and refuses less, the default being
2 ([adc.md](adc.md), "The AVR EA's converter").

Converting counts to millivolts: `ref_mv()` gives the level, the
target-independent arithmetic of `util/analog.hpp` does the rest.
(`Ref` is this silicon's: another target's vref header defines its
own `brio::Ref` with its levels - same name, never in one binary.)

```cpp
#include "util/analog.hpp"

const uint16_t mv = adc_mv(Adc<0>::read(), Adc<0>::steps(), ref_mv(Ref::v2048));
Dac<0>::set(dac_code(1500, Dac<0>::steps, ref_mv(Ref::v2048)));  // 1.500 V
```

For `vdd` and `vrefa` the level is whatever it is: pass the known
millivolts (`ref_mv(Ref::vdd, 3300)`) or measure VDD first (VDDDIV10
against an internal reference - [adc.md](adc.md)).

Direct use, to keep a reference always on:

```cpp
#include "avrxt/vref.hpp"

Vref::ac(Ref::v1024);                    // the ACs' reference
Vref::adc0(Ref::v4096, /*always_on=*/true);   // the DA/DB: no start-up wait on the first conversion
```

An external reference: feed VREFA (PD7) and select `Ref::vrefa`. On the
bench the DAC itself can be that source (PD6 -> PD7): the ADC then
reads the DAC's internal path as full scale for any level >= 1.024 V
(`test_avr_analog` test 14).

## Bench findings on the AVR EA (`test_avr_analog`, AVR64EA48 rev B1, 5 V)

- Every pair of the four internal levels at 5 V, the DAC on one and
  the ADC on another, agrees within 0.5 % of its expectation (3058 of
  3072 the worst): the two references' +-2 / +-3 % are far from used.
- VREFA driven from the DAC (PD6 -> PD7 on the board's net) works from
  1.04 V: the ADC on VREFA reads the DAC's internal node at 4095 at
  every level (the pad sits a few millivolts UNDER the node, so the node
  is never short of full scale); VDDDIV10 against it as expected
  (letter v).
- With LOWLAT off a software-started conversion on the 1.024 V
  reference pays 59 us of settling, table 31-5's 60: the reference is
  the ADC's slowest module.

## Bench findings (`test_avr_analog`, rev A5, 3.3 V and 5 V)

- All internal levels cross-check within 1-2 % of each other through
  the DAC -> ADC loop (spec 4 %); at 5 V all four are usable.
- VREFA driven from the DAC output works down to 1.04 V.
