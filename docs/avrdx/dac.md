# DAC - the 10-bit digital-to-analog converter (AVR DA/DB)

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B
(DAC chapter, electricals 39.19), errata DS80000915F (2.6.1) and, for
the DA parts, DS80000882C (2.5.1). Driver:
`avrdx/dac.hpp`; `Ref` from `avrdx/vref.hpp`, arithmetic in `util/analog.hpp`. Reference test:
`test_avr_analog` (tests 1-4, 6, 11, 14).

## What the silicon does

One instance, DAC0. A 10-bit value written to `DATA` becomes a
voltage between GND and the selected reference (`Ref`, see
[vref.md](vref.md)). Two outputs of the same voltage:

- the **buffered** output on pin PD6 (`output_pin`): sources 1 mA,
  sinks only ~1 uA, 0.1 V .. VDD-0.1 V, settles in ~10 us full scale
  when rising;
- the **unbuffered** internal output, always available to the ADC
  (input `AdcInput::dac0`) and the comparators, also with the pin
  output off (then PD6 stays a GPIO). The op amps are the exception:
  they tap the DAC at the OUTPUT BUFFER (34.3.2.3, 35.3.2), so an
  op amp fed by the DAC needs `output_pin` on - and PD6 is then not
  free.

There are no events, no interrupts, no modes: the only way to change
the output is to write `DATA`, and the output follows. `run_standby`
keeps it running in standby sleep. Accuracy (ref 3.0 V): INL +-2.3,
DNL +-0.7, offset +-5, gain -3.3..+1.3 LSb, specified for codes
0x030..0x3D0.

Two physical facts to design with (both measured on the bench):

- **it falls slowly on a bare pin**: the buffer cannot sink, so a
  falling step discharges the pin capacitance at ~1 uA - about 20 kV/s,
  2 V in ~100 us and the last tens of millivolts much longer. A load
  that needs fast falling edges wants the datasheet's resistor to
  ground (10 kOhm sinks 200 uA at 2 V: 100 x faster);
- the buffered pin sits ~13 mV above the unbuffered internal path:
  the buffer's offset (spec EOFF +-5 LSb, i.e. +-10 mV at the
  2.048 V reference).

Errata DB 2.6.1 (silicon A4/A5) / DA 2.5.1 (every DA revision): the
buffer's offset drifts over the device's lifetime if the part is
powered with the buffer OFF. The
driver therefore keeps the buffer on once enabled (`disable()` turns
the converter off, not the buffer); an application that wants the
pin free says `output_pin = false` explicitly and owns the drift (or
calibrates the offset against the ADC).

## How to use it

A voltage source on PD6:

```cpp
#include "avrdx/dac.hpp"
using Out = Dac<0>;

Out::init({.reference = Ref::v2048});     // buffered output on PD6, reference 2.048 V
Out::set(512);                            // ~1.024 V (code 0..1023)
Out::set_mv(1500, ref_mv(Ref::v2048));    // 1.500 V
```

Internal use only (feeding the ADC or a comparator, PD6 left free):

```cpp
Out::init({.reference = Ref::v2048, .output_pin = false});
Out::set(700);
Adc<0>::select(AdcInput::dac0);           // the ADC reads it (sample it long: see adc.md)
```

Keep running while the CPU sleeps in standby (a bias voltage):

```cpp
Out::init({.reference = Ref::v1024, .run_standby = true, .reference_always_on = true});
```

A ramp, and the slow fall in practice:

```cpp
for (uint16_t code = 0; code < 1024; code += 64) {
    Out::set(code);
    delay_us(clock, 300);                 // rising: settled (ring ~50 us)
}
Out::set(0);
delay_us(clock, 5000);                    // falling to 0 V on a bare pin: be generous
```

Code <-> millivolts with the reference's value: `dac_code(mv, Out::steps,
ref_mv)` and `dac_mv(code, Out::steps, ref_mv)` in `util/analog.hpp`.

## Bench findings (`test_avr_analog`, rev A5, 3.3 V and 5 V)

On the **AVR128DA48** (rev A7, 5 V) the internal path reads as on the DB
(the unbuffered output 2043 of 2048 with a 32-cycle sample, every
reference pair's DAC0 within a few counts of its expectation); THE PIN
DOES NOT. Wired to PD1 and PD7, a falling step discharges at about
200 V/s and at constant current - 2.0 V to 0.2 V in about 9 ms, the
same with the ADC converting throughout the fall or reading once at
its end - where the DB's pin falls at ~20 kV/s: a sink of a few tens of
nanoamperes on that net, not ~1 uA. A rising step overshoots and the
overshoot comes down at the same rate: from 0 V to 1.536 V the wire
read above 2.048 V at once, and 0.2 V to 2.0 V read full scale against
the 2.048 V reference within two conversions. Whether the part or the
5 V supply makes the difference is not separated. The suite's legs that
read the pin stay owed there until the wire carries the resistor to
ground ([adc.md](adc.md)'s list).

- Ramp 0..1023 against the ADC on the wire: monotonic, error within
  +-7 LSB12 over the range, i.e. well inside the specified INL +
  offset + gain.
- Rising steps settle in ~10-20 us; a ~50 us ring follows before the
  last few LSB settle; falling steps on a bare pin discharge at the
  1 uA sink limit (~20 kV/s), as above.
- Unbuffered internal path vs buffered pin: 13 mV apart (the buffer's
  offset).
- Internal path with the buffer off is exact; the bare pin then
  floats and holds charge for a long time.
