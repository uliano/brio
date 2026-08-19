# VREF - the voltage reference selector (AVR DA/DB)

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B,
errata DS80000915F (no VREF items). Driver: `avrdx/vref.hpp` (the
`Ref` vocabulary is defined there; the counts <-> mV arithmetic is in
`util/analog.hpp`). Reference test: `test_avr_analog` (test 1, 14).

## What the silicon does

Three independent reference selections, one per consumer - `ADC0REF`,
`DAC0REF`, `ACREF` (the three analog comparators share one). Each
selects one of:

| `Ref` | Level | Needs VDD |
|-------|-------|-----------|
| `v1024` | 1.024 V internal | - |
| `v2048` | 2.048 V internal | >= 2.5 V |
| `v2500` | 2.500 V internal | >= 2.95 V |
| `v4096` | 4.096 V internal | >= 4.55 V |
| `vdd` | the supply | - |
| `vrefa` | external, pin PD7 (1.024 V .. VDD) | - |

Internal levels are +-4 % untrimmed over temperature; measured on the
bench they agree with each other within 1-2 %. A source turns on by
itself when its consumer asks for it (start-up 10 us from an internal
main clock, 200 us from an external one; 2 us to change level);
`always_on` keeps it powered for ~40-175 uA so the consumer never
waits. VREFA loads the external source with ~50 kOhm (the ADC's
ladder). There is no other behaviour: VREF is a selector.

What it means for code: the reference is part of the MEANING of every
conversion (counts <-> volts); the ADC and the DAC may use different
ones; a DAC -> ADC loop is exact only when both are known.

## How to use it

You rarely touch VREF directly: the ADC, DAC and AC configurations
name their reference and their `init()` selects it.

```cpp
#include "avrdx/adc.hpp"
#include "avrdx/dac.hpp"
#include "avrdx/ac.hpp"

Dac<0>::init({.reference = Ref::v2048});                         // sets VREF.DAC0REF
Adc<0>::init(clock, AdcConfig{.reference = Ref::v2048});         // sets VREF.ADC0REF
Ac<0>::init({.negative = AcNeg::dacref, .reference = Ref::v2048}); // sets VREF.ACREF (DACREF)
```

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
against an internal reference - see the ADC document).

Direct use, to keep a reference always on:

```cpp
#include "avrdx/vref.hpp"

Vref::ac(Ref::v1024);                    // the three ACs' reference
Vref::adc0(Ref::v4096, /*always_on=*/true);   // no start-up wait on the first conversion
```

An external reference: feed VREFA (PD7) and select `Ref::vrefa`. On the
bench the DAC itself can be that source (PD6 -> PD7): the ADC then
reads the DAC's internal path as full scale for any level >= 1.024 V
(`test_avr_analog` test 14).

## Bench findings (`test_avr_analog`, rev A5, 3.3 V and 5 V)

- All internal levels cross-check within 1-2 % of each other through
  the DAC -> ADC loop (spec 4 %); at 5 V all four are usable.
- VREFA driven from the DAC output works down to 1.04 V.
