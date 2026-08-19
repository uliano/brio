# ADC - the 12-bit analog-to-digital converter (AVR DA/DB)

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B (ADC
chapter, electricals 39.18), errata DS80000915F (2.3.1, 2.3.2).
Driver: `avrdx/adc.hpp`; `Ref` from `avrdx/vref.hpp`, counts <-> mV arithmetic in `util/analog.hpp`
(host-tested in `test_analog`); event start through `avrdx/evsys.hpp`.
Reference test: `test_avr_analog` (all 14 tests).

## What the silicon does

One instance, ADC0: a 12-bit SAR converter (10-bit optional, 2 cycles
faster), up to 130 ksps. One conversion is always the same sequence -
select an input, trigger, wait for the result, read it - and every
use of the converter is that sequence with different knobs:

- **inputs**: 22 pins (AIN0-7 = PD0-7, AIN8-15 = PE0-7, AIN16-21 =
  PF0-5) and internal sources - GND, the temperature sensor, VDD/10,
  VDDIO2/10, the DAC's unbuffered output, the comparators' DAC
  references. **Single-ended** (0..4095 against the reference) or
  **differential** (a positive and a negative input, -2048..2047; the
  negative may be AIN0-15, GND or DAC0 - not PF0-5);
- **trigger**: software (`start()`), an **event** from the event
  system (rising edge, asynchronous: works in standby sleep - the
  natural pacer is a PIT divider), or **free-running** (each
  conversion starts the next);
- **timing**: CLK_ADC = CLK_PER / {2..256} must stay within 125 kHz ..
  2 MHz (accuracy is specified at 500 kHz); sampling takes 2 CLK_ADC
  cycles plus `sample_length` (0-255 extra cycles, for sources above
  the recommended 10 kOhm) plus `sample_delay` (0-15, to move the
  sampling instant off a noise harmonic); `init_delay` (0..256 cycles)
  is paid once, for the first conversion after enabling, to let the
  reference settle. Conversion time = 2/fPER + n x (13.5 + 2 +
  sample_delay + sample_length)/fADC + 2/fPER (12-bit);
- **accumulation**: 1, 2, 4 .. 128 samples summed in hardware into one
  result (`RES` is the SUM); above 16 samples the sum does not fit 16
  bits and the hardware drops LSBs (32: 1 bit, 64: 2, 128: 3). So 16
  samples give 16 bits of oversampled resolution; 32/64/128 only
  average further - less noise, no more bits. Sum and truncation are
  undone with `result_shift()` / `result_steps()`;
- **result**: 16-bit, right- or left-adjusted (left-adjusted, the
  window thresholds become independent of the accumulation count);
- **window comparator**: the result compared with two thresholds -
  below / above / inside / outside - raising its own flag and
  interrupt (`WCMP`) next to the result-ready one (`RESRDY`); both
  flags are cleared by reading the result;
- **events out**: a result-ready pulse for the event system;
- the **temperature sensor** (2.048 V reference, init delay >= 25 us,
  sample >= 28 us, 40 us after selecting it) with two calibration
  factors in the signature row; **supply monitors** VDD/10 and
  VDDIO2/10 (+-10 %).

Rules that the driver enforces or that the application must respect:
the mode, resolution, prescaler and accumulation must not change
during a conversion (in free-running: stop, wait, change, restart -
`reconfigure()` does); the converter needs ~6 us of warm-up after
enabling before its first conversion is valid (`init()` waits); with
`init_delay != 0`, an input or accumulation change made AFTER enabling
takes effect only after one conversion (errata 2.3.2 - `flush()`
throws one away; `init()` writes the mux before enabling); rev A4
silicon has a -3 mV single-ended offset (2.3.1). Accuracy (12-bit,
500 kHz, 3.0 V reference): INL +-1.8, DNL +-1, offset 2.5 typ / 5 max,
gain +-5 LSB.

## Types and verbs

One type, `Adc<0>` (monostate; the instance number for symmetry with
targets that have several). Its configuration is a struct whose
fields carry the datasheet's names:

| `AdcConfig` field | Values | Default | Effect |
|-------------------|--------|---------|--------|
| `reference` | `Ref::v1024 / v2048 / v2500 / v4096 / vdd / vrefa` | `vdd` | full scale (selects VREF.ADC0REF) |
| `reference_always_on` | bool | false | keep the reference powered (no start-up wait) |
| `resolution` | `AdcRes::bits12 / bits10` | 12 | 0..4095 or 0..1023 per sample |
| `differential` | bool | false | signed (pos - neg), -2048..2047 |
| `prescaler` | `AdcPresc::div2 .. div256` (14 values) | div16 | CLK_ADC = CLK_PER / n, must land in 125 kHz .. 2 MHz |
| `sample_length` | 0..255 | 0 | extra CLK_ADC cycles of sampling (sources > 10 kOhm, the unbuffered DAC, the temperature sensor) |
| `sample_delay` | 0..15 | 0 | CLK_ADC cycles before sampling (move off a noise harmonic) |
| `init_delay` | `AdcInitDelay::none / cycles16 .. cycles256` | none | settling before the FIRST conversion after enable |
| `accumulate` | 1, 2, 4 .. 128 | 1 | samples summed per result (RES = sum, truncated above 16) |
| `left_adjust` | bool | false | result << 4 (thresholds independent of accumulation) |
| `free_running` | bool | false | each conversion starts the next |
| `run_standby` | bool | false | keep converting in standby sleep |

Inputs are types or enumerators: `AnalogIn<Pin<'D', 1>>` (a pin,
PD0-7 / PE0-7 / PF0-5 - the table is checked, PF0-5 refused as a
negative input) and `AdcInput::gnd / temp / vdd_div10 / vddio2_div10 /
dac0 / dacref0..2` (internal; as negative input only `gnd` and `dac0`).

The verbs, by purpose:

| Purpose | Verbs |
|---------|-------|
| configure | `init<cfg>(clock)` (constant, checked), `init(clock, cfg)` (value; `false` if invalid), `reconfigure(clock, cfg)` (under a running program: stops, waits idle, applies), `disable()` |
| what is in effect | `reference()`, `steps()` (4096/1024), `accumulate()`, `result_shift()`, `result_steps()`, `clock_hz_adc()` |
| select the input | `select(AnalogIn<P>{})`, `select(AdcInput)`, `select(pos, neg)` (differential), `flush()` (one throw-away conversion: errata 2.3.2) |
| convert | `start()`, `stop()`, `busy()`, `ready()`, `result()`, `result_signed()`, `read()` (start + wait + result, blocking) |
| window comparator | `window(Window::below/above/inside/outside, low, high)`, `window_off()`, `window_hit()` (about the last result read), `window_flag()` / `clear_window_flag()` (the live flag) |
| interrupts | `enable_resrdy_interrupt(bool)`, `enable_wcmp_interrupt(bool)`; ISR bodies `resrdy()`, `wcmp()` (return the result; the app binds the vectors and posts) |
| events | `start_on(EventChannel<n>{})` (a conversion per rising edge of the channel), `start_on_events(bool)`; generator `EvAdc0Ready`, user `EvAdc0Start` (evsys.hpp) |
| clock | `rebase(hz)` - the `ClockUser` hook a `DynamicClock` calls: keeps the CLK_ADC chosen at init, within range |

Arithmetic: `ref_mv(Ref, known_mv)` (`avrdx/vref.hpp`), `adc_mv(counts,
steps, ref_mv)`, `adc_mv_signed(...)` (`util/analog.hpp`, pure),
`Adc<0>::temp_kelvin(result)` (the signature-row formula); timing: `adc_clock_hz(clk, presc)`,
`adc_conversion_cycles(cfg)`, `adc_presc_for(clk, target_hz)`.

## How to use it

Two forms of configuration, one implementation: a constant as a
template argument (checked at compile time - knobs AND the CLK_ADC
range for a static clock - every register write folded) or a run-time
value (the same writes computed on the fly).

```cpp
#include "avrdx/adc.hpp"
using A = Adc<0>;

// compile-time: static_asserts on the knobs, folded writes
A::init<AdcConfig{.reference = Ref::v2048, .prescaler = AdcPresc::div16}>(clock);

// run-time: same thing from a value (a console command, a state)
AdcConfig cfg{.reference = Ref::v2048};
cfg.accumulate = 16;
A::init(clock, cfg);
```

Defaults: VDD reference, 12 bits, single-ended, CLK_PER/16, no extra
sampling, no accumulation, right-adjusted, not free-running, not in
standby. `init()` selects the reference, writes every knob, enables,
waits the warm-up, leaves the input on GND.

**A single reading** (blocking: fine before the kernel runs, or in a
bench tool):

```cpp
A::select(AnalogIn<Pin<'D', 1>>{});          // AIN1 - a pin is a type, the table is checked
const uint16_t counts = A::read();           // start, wait for the result, read
const uint16_t mv = adc_mv(counts, A::steps(), ref_mv(Ref::v2048));
```

**Internal sources** and a high-impedance one (the unbuffered DAC
output needs sampling time - measured: 3-4 % low with the default 2
cycles, true with 32):

```cpp
A::init(clock, AdcConfig{.reference = Ref::v2048, .sample_length = 32});
A::select(AdcInput::dac0);
A::select(AdcInput::vdd_div10);              // VDD = adc_mv(...) * 10
```

**Differential**:

```cpp
A::init(clock, AdcConfig{.reference = Ref::v2048, .differential = true});
A::select(AnalogIn<Pin<'D', 1>>{}, AnalogIn<Pin<'D', 3>>{});   // AIN1 - AIN3
const int16_t d = A::result_signed();        // after start()/ready(), or static_cast<int16_t>(A::read())
```

**Accumulation** (hardware oversampling):

```cpp
A::init(clock, AdcConfig{.reference = Ref::v2048, .accumulate = 16});
const uint32_t sum = static_cast<uint32_t>(A::read()) << A::result_shift();
const uint16_t mv = adc_mv(sum, A::result_steps(), ref_mv(Ref::v2048));  // or sum / 16 per sample
```

**Inside an active object - results as events** (the shape every
kernel application uses): the RESRDY interrupt body returns the
result, the app's vector binding posts it, the AO decides.

```cpp
struct AdcResult { uint16_t value; };

struct Meter : Fsm<Meter, AdcResult, ...> {
    static void init() {
        Adc<0>::init<AdcConfig{.reference = Ref::v2048, .accumulate = 16}>(clock);
        Adc<0>::select(AnalogIn<Pin<'D', 1>>{});
        Adc<0>::enable_resrdy_interrupt(true);
        start(&running);
    }
    static Status running(const Event& e) {
        return match(e,
            [](AdcResult r) { /* the value, in main context */ return handled(); },
            ...);
    }
};

ISR(ADC0_RESRDY_vect) { post<Meter>(AdcResult{Adc<0>::resrdy()}); }   // target glue
```

**Paced by the event system** (no CPU between samples; also in
standby): a PIT divider on an odd channel starts a conversion 512
times a second:

```cpp
EventChannel<1>::source(EvPitDiv<64>{});     // 32768 / 64 Hz
Adc<0>::start_on(EventChannel<1>{});         // the converter listens, STARTEI set
// ... results arrive through the RESRDY body as above;
Adc<0>::start_on_events(false);              // stop pacing
```

**Free-running** (the converter restarts itself; poll or interrupt):

```cpp
A::init(clock, AdcConfig{.reference = Ref::v2048, .free_running = true});
A::select(AnalogIn<Pin<'D', 1>>{});
A::start();                                  // the first one; then it runs
if (A::ready()) { const uint16_t v = A::result(); }
A::stop();
```

**A threshold watch** (the window comparator; the WCMP body for the
interrupt form, `window_hit()` for the polled one - both tell about
the result just read, because reading the result clears the flag):

```cpp
A::window(A::Window::outside, 1000, 3000);  // result < 1000 or > 3000
// polled:
(void)A::read();
if (A::window_hit()) { /* out of band */ }
// interrupt:
A::enable_wcmp_interrupt(true);
ISR(ADC0_WCMP_vect) { post<Guard>(Threshold{Adc<0>::wcmp()}); }
```

**Temperature and supply**:

```cpp
// 2.048 V reference, init delay >= 25 us, sample >= 28 us: at 24 MHz / 64 = 375 kHz
A::init(clock, AdcConfig{.reference = Ref::v2048, .prescaler = AdcPresc::div64,
                         .sample_length = 12, .init_delay = AdcInitDelay::cycles64});
A::select(AdcInput::temp);
A::flush();                                  // errata 2.3.2: the select came after enable
const uint16_t kelvin = A::temp_kelvin(A::read());   // signature-row factors inside

A::select(AdcInput::vdd_div10);              // VDD in mV = adc_mv(read, steps, ref) * 10
```

**Reconfiguring under a running program** (a console command, a
state's Entry): `reconfigure(clock, cfg)` stops free-running, waits
for the converter to be idle, applies the new configuration, waits the
warm-up; the input is back on GND - `select()` again. The meaning of
the results changes with it (reference, resolution, accumulation):
convert with the new `steps()` / `result_shift()`.

**When the clock changes** (a `DynamicClock`): CLK_ADC moves with
CLK_PER, and a prescaler valid at one rate can be out of range at
another. `Adc<0>` is a `ClockUser`: listed among the clock's users
(`DynamicClock<Boot, Serial, Adc<0>>` - a dynamic clock that does not
list it fails the `init` of the ADC), its `rebase(hz)` re-picks the
prescaler to keep the CLK_ADC chosen at init as close as possible and
within range: it stops, waits for the converter to be idle, rewrites,
and resumes free-running if it was on. Sample and init delays are in
CLK_ADC cycles, so they scale with it. What the owner must do: pause
event-started conversions (`start_on_events(false)`) before
`SysClock::set()`, because an event may start a conversion at any
instant - the general rule of [clock.md](../design/clock.md), here with its
concrete case.

Ownership: one converter, one owner. An AO that owns it configures it
in its states and interprets its results; a second consumer asks that
AO (request/reply), it does not touch the registers.

## Bench findings (`test_avr_analog`, rev A5, 3.3 V and 5 V: 68/68)

- The warm-up after enabling is real: the first conversion before
  ~6 us is garbage. `init()` waits 10 us - that is why it takes the
  clock.
- `init_delay` is paid once, for the first conversion after enabling,
  not per start (2230 one-shots per 100 ms with 256 cycles of delay at
  375 kHz: the plain rate).
- The result and its flags are ready ~2 CLK_PER after the conversion
  bit clears: wait on RESRDY, not on STCONV, or you read the previous
  result.
- Reading RES clears WCMP too: the window verdict must be captured
  before the read (`result()` does).
- Sums above 16 accumulations are truncated exactly as the table
  says; `result_shift()` restores the scale.
- The unbuffered DAC0 input needs `sample_length` (~32 at 1.5 MHz).
- Free-running rates match the timing formula within 1-2 % for every
  prescaler, sample_length and sample_delay tested.
- Event start from PIT/64: 513 conversions per second (the OSC32K's
  tolerance).
- Errata 2.3.2 reproduced: with `init_delay` set, a `select()` after
  enabling returns the old input for one conversion; `flush()` cures
  it.
- VDD/10 reads 3280 mV on a 3.3 V rail and 5170 on a 5 V one; die
  temperature 27-28 C from the signature-row factors; GND reads 5-6
  counts (the offset).
