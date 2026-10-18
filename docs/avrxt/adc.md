# ADC - the 12-bit analog-to-digital converters (AVR DA/DB, AVR EA)

Two converters, two engines, one surface (`avrxt/adc.hpp`, whose
part's header decides which half compiles): the AVR DA/DB's 12-bit SAR
ADC, this page's first half, and the AVR EA's 12-bit differential ADC
with a programmable gain amplifier, its second ([The AVR EA's
converter](#the-avr-eas-converter)).

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B (ADC
chapter, electricals 39.18), errata DS80000915F (2.3.1, 2.3.2);
AVR64EA28/32/48 data sheet DS40002443A (chapter 31, electricals 35.17,
the 16- and 32-bit register access of 8.10), errata DS80001048D (no ADC
item; its 3.4 gives the supply current), and as the oracle of shape
Microchip's TB3254 and TB3256 (written for the tinyAVR 2, whose
TIMEBASE sat in ADC.CTRLC: the EA moved it to CLKCTRL). Driver:
`avrxt/adc.hpp`; `Ref` from `avrxt/vref.hpp`, counts <-> mV arithmetic
in `util/analog.hpp` (host-tested in `test_analog`); event start
through `avrxt/evsys.hpp`. Reference test: `test_avr_analog`, one half
per engine.

## The AVR DA/DB's converter

### What the silicon does

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

### Types and verbs

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
| `debug_run` | bool | false | DBGRUN: keep converting while the CPU is halted |
| `init_delay` | `AdcInitDelay::none / cycles16 .. cycles256` | none | settling before the FIRST conversion after enable (long enough to cover the reference start-up: 10 us with a high-frequency main clock, 200 us with a 32k one - [vref.md](vref.md)) |
| `accumulate` | 1, 2, 4 .. 128 | 1 | samples summed per result (RES = sum, truncated above 16) |
| `left_adjust` | bool | false | MSB-aligned result; the shift depends on the accumulation count (Table 33-1) and the driver's result helpers assume right adjustment |
| `free_running` | bool | false | each conversion starts the next |
| `run_standby` | bool | false | keep converting in standby sleep |

Inputs are types or enumerators: `AnalogIn<Pin<'D', 1>>` (a pin,
PD0-7 / PE0-7 / PF0-5 - the table is checked, PF0-5 refused as a
negative input) and `AdcInput::gnd / temp / vdd_div10 / vddio2_div10 /
dac0 / dacref0..2` (internal; as negative input only `gnd` and `dac0` - `adc_neg_valid` is the fact, and the enum-negative `select` overloads return false for anything else;
`vdd_div10`/`vddio2_div10` exist on the DB only - the DA reserves
those mux codes; the `dacref0..2` inputs need ~30 us of sampling,
tADC_DACREF, errata document clarification 3.7.5).

The verbs, by purpose:

| Purpose | Verbs |
|---------|-------|
| configure | `init<cfg>(clock)` (constant, checked), `init(clock, cfg)` (value; `false` if invalid), `reconfigure(clock, cfg)` (under a running program: stops, waits idle, applies), `disable()` |
| what is in effect | `reference()`, `steps()` (4096/1024), `accumulate()`, `result_shift()`, `result_steps()`, `clock_hz_adc()` |
| select the input | `select(AnalogIn<P>{})`, `select(AdcInput)`, `select(pos, neg)` (differential), `flush()` (one throw-away conversion: errata 2.3.2) |
| convert | `start()`, `stop()`, `busy()`, `ready()`, `result()`, `result_signed()`, `read()` (start + wait + result, blocking) |
| window comparator | `window(Window::below/above/inside/outside, low, high)` (single-ended format), `window_signed(mode, low, high)` (the differential two's-complement format, 33.5.16), `window_off()`, `window_hit()` (about the last result read), `window_flag()` / `clear_window_flag()` (the live flag) |
| interrupts | `enable_resrdy_interrupt(bool)`, `enable_wcmp_interrupt(bool)`; ISR bodies `resrdy()`, `wcmp()` (return the result; the app binds the vectors and posts); `selected()` (the MUXPOS code in effect, to label a result in the glue), `input_code(input)` constexpr |
| events | `start_on(EventChannel<n>{})` (a conversion per rising edge of the channel), `start_on_events(bool)`; generator `EvAdc0Ready`, user `EvAdc0Start` (evsys.hpp) |
| clock | `rebase(hz)` - the `ClockUser` hook a `DynamicClock` calls: keeps the CLK_ADC chosen at init, within range; `clock_ok()` says whether the last rebase FOUND an in-range prescaler (rebase itself stays void and best-effort by the ClockUser contract) |

Arithmetic: `ref_mv(Ref, known_mv)` (`avrxt/vref.hpp`), `adc_mv(counts,
steps, ref_mv)`, `adc_mv_signed(...)` (`util/analog.hpp`, pure),
`Adc<0>::temp_kelvin(result)` (the signature-row formula); timing: `adc_clock_hz(clk, presc)`,
`adc_conversion_cycles(cfg)`, `adc_presc_for(clk, target_hz)`.

### How to use it

Two forms of configuration, one implementation: a constant as a
template argument (checked at compile time - knobs AND the CLK_ADC
range for a static clock - every register write folded) or a run-time
value (the same writes computed on the fly).

```cpp
#include "avrxt/adc.hpp"
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
result, the app's vector binding posts it, the AO decides. The
ready-made owner is `AnalogSampler` (`util/analog_sampler.hpp`,
[analog.md](../design/analog.md)): a list of inputs walked, results
published, paced by software or by any event generator; `selected()`
and `input_code()` are the two verbs it needs of this driver. The
bare form, for an AO with its own logic:

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

### Bench findings (`test_avr_analog`, rev A5, 3.3 V and 5 V)

On the **AVR128DA48** (rev A7, 5 V) - where the DAC's slow edges on the
wire ([dac.md](dac.md)) leave every leg that reads it owed (below) -
the internal paths measure as on the DB: the
reference cross-check through DAC0 within a few counts of every
expectation (3068 of 3072, 1534 of 1536, 2517 of 2516), the DAC
unbuffered path, the prescaler rates, the sampling-time formula, the
DACREF inputs at 997 / 2001 / 2997 of 1000 / 2000 / 3000, the die at 27
C. With no VDDDIV10 input on a DA (no MVIO) the suite reads VDD as DAC0
at 2000 mV from the 2.048 V reference against VDD as the reference:
5022..5028 mV, ten runs, where the probe reads 5.04 V.

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
- The three `dacref0..2` inputs read their AC's DACREF within 7
  counts (64 CLK_ADC of sampling covers tADC_DACREF); the signed
  window hits below -1000 on a -1400 differential result and not
  below -2000; a live 24 -> 12 MHz rebase moves a dacref reading by
  3 counts and `clock_ok()` stays true.
- `run_standby`, in a real STANDBY (`test_avr_sleep` o, 5 V): a
  conversion started by a PIT divider through the event system runs
  with the CPU asleep and its RESRDY interrupt ends the sleep, reading
  OP0's VDD/2 within 2 counts of the awake conversion; without the
  flag nothing converts in standby and the PIT backstop ends the
  sleep - every run of twelve.

## The AVR EA's converter

### What the silicon does

One instance, ADC0 (DS40002443A ch. 31), the tinyAVR 2's design: a
12-bit SAR converter behind a programmable gain amplifier, single-ended
or differential on EVERY conversion, with two output registers and an
accumulator.

- **A conversion is one COMMAND write** (31.5.11): DIFF (single-ended or
  differential), MODE (single 8-bit, single 12-bit, series, series
  scaled, burst, burst scaled - table 31-1) and START (STOP, IMMEDIATE,
  MUXPOS_WRITE, MUXNEG_WRITE, EVENT_TRIGGER) in one byte.
- **Inputs** (3.1, 31.5.13/14): AIN0-7 = PD0-7, AIN8-11 = PE0-3,
  AIN16-21 = PF0-5, AIN22-27 = PA2-7, AIN28-31 = PC0-3, every one a
  positive AND a negative input; internal positives GND, VDDDIV10,
  TEMPSENSE, DAC0 (the DAC's unbuffered output); internal negatives
  GND, DAC0 and the two comparators' DACREF0/1 (negative only).
  `(VINP - VINN) x gain / VREF x 2048` differential, `VINP x gain /
  VREF x 4096` single-ended (31.3.3.3).
- **The PGA** (31.3.3.5.1, 31.5.12): gain 1, 2, 4, 8, 16; bias 100 /
  75 / 50 / 25 % usable to CLK_ADC 6 / 4.5 / 3 / 1.5 MHz; the VIA bits
  route both muxes through it at once (shared, 31.5.13). The ADC samples
  the PGA for a fixed 3 / 5 / 6 / 12 us by bias (4 us at 100 % with an
  internal reference, table 31-6).
- **Two outputs** (31.3.3.3, tables 31-2..31-4): SAMPLE (16 bits, every
  conversion) and RESULT (32 bits: a single conversion, or the SUM of
  an accumulation - 4095 x 1024 is 22 bits, signed 23 in DIFF; SCALING
  right-shifts it by SAMPNUM - 4, left below 16 samples, so a scaled
  result's full scale is 65536). LEFTADJ exists for a 12-bit single
  conversion and a scaled sum.
- **Accumulation** (31.3.3.1, 31.5.10): SAMPNUM 1 .. 1024; BURST runs
  them on one trigger, SERIES takes one trigger per sample; the
  accumulator is reset at every new run. CHOPPING (sign chopping)
  works in the accumulating modes only.
- **The reference** (31.5.3) is selected in ADC.CTRLC with an encoding
  of its own (VDD 0, VREFA 2, 1.024 V 4, 2.048 V 5, 4.096 V 6, 2.5 V 7);
  an internal one wants VDD >= reference + 0.5 V. CLK_ADC = CLK_PER /
  2 .. 64 (sixteen codes, CTRLB) within 300 kHz .. 2 MHz with an
  internal reference, .. 6 MHz with VDD or VREFA (table 35-24).
- **Timing** (31.3.3.6): a conversion is SAMPDUR + 0.5 CLK_ADC of
  sampling (+1 through the PGA) and 13 CLK_ADC of conversion (9 at 8
  bits); with an internal reference and no PGA SAMPDUR >= 4 us x
  fCLK_ADC (the reference buffer's autozero), a burst through the PGA
  SAMPDUR >= 11 (31.3.3.6.6 - the PGA paragraph says 12). Before a
  conversion the hardware waits the initialization of the modules it
  uses (table 31-5: the ADC 10 us, the PGA 20, an internal reference
  60, TEMPSENSE and the DAC input 35), counted in CLKCTRL.MCLKTIMEBASE;
  LOWLAT keeps the modules on and removes it.
- **The input mux is NOT buffered** (31.3.3.5): a change during a
  conversion corrupts it - where the DB buffers and locks both mux
  fields.
- **Triggers and aborts** (31.3.3.2): a trigger during a conversion is
  ignored and sets TRIGOVR; STOP, a REFSEL write, or selecting DAC0,
  TEMPSENSE or VDDDIV10 aborts one; FREERUN repeats single conversions
  and bursts (not series).
- **The window** (31.3.3.8, 31.5.4): below, above, inside (STRICT:
  WINLT < x < WINHT), outside, on RESULT[15:0] or on SAMPLE (WINSRC);
  its flag rides the RESRDY or the SAMPRDY vector and is cleared only by
  writing 1 (31.5.6).
- **Flags and vectors** (table 31-11): RESRDY and SAMPRDY (cleared by
  reading RESULT and SAMPLE), and on the ERROR vector TRIGOVR, SAMPOVR,
  RESOVR. **Events** (table 31-9): RES 0x24, SAMP 0x25, WCMP 0x26
  generators; the START user. The 16- and 32-bit registers share three
  TEMP registers (31.5.17): one owner, one context at a time.
- **The temperature** (31.3.3.7): at 1.024 V with SAMPDUR >= 32 us x
  fCLK_ADC, T = (RESULT + TEMPSENSE1) x TEMPSENSE0 / 4096, both factors
  SIGNED (the DB: (TEMPSENSE1 - RES) x TEMPSENSE0 / 4096 at 2.048 V -
  same names, other meaning).

What it offers that bears on cost, each item used or declined:

| offer | in the driver |
|---|---|
| DIFF, MODE and START in one COMMAND byte | used: the driver keeps the DIFF / MODE image and a start ORs IMMEDIATE into it - one load, one store |
| DIFF per conversion | used: an input VALUE (`AdcDiff<pos, neg>`), so a walk mixes single-ended and differential inputs at no extra store |
| MUXPOS_WRITE start | used: `start_on_select()`, a walk whose select is the start (one store an input) |
| MUXNEG_WRITE start | declined: a pair writes MUXNEG first and MUXPOS last, so MUXPOS_WRITE serves both |
| EVENT_TRIGGER start | used: `start_on()`, no CPU between conversions, async (Idle, and Standby with RUNSTDBY) |
| FREERUN | used: `.free_running` (single and burst) |
| BURST / SERIES / SCALING, SAMPNUM 1..1024 | used: `.accumulate`, `.series`, `.scaled`; a burst costs one start, one interrupt and one read for N samples |
| RESULT 32 bits / SAMPLE 16 | used: `result()` reads two bytes, `sum()` four, `sample()` two |
| SAMPRDY as a per-sample stream | declined: at 312 ksps the budget is 64 CPU cycles a sample - an ISR and a ring push do not fit; a block engine is born with its first user |
| the PGA, its four biases | used: `.pga`, `.gain`, `.pga_bias`, PGACTRL written whole, the bias checked against CLK_ADC |
| CHOPPING | used: `.chopping`, refused in single mode |
| LOWLAT | used: `.low_latency`, OFF by default - the modules' current when idle against 10..60 us per software-started trigger (free-running and bursts pay nothing: measured below) |
| the window on RESULT or SAMPLE | used: `.window`, `.window_source`, `window()`; refused on RESULT where RESULT is wider than 16 bits |
| TRIGOVR / SAMPOVR / RESOVR | used: `error()` (an ISR body and a polled verb) - the hardware's own count of a too-fast pace and a too-slow reader |
| ADCBUSY | used: `busy()` |
| RES / SAMP / WCMP event generators | named in evsys.hpp (`EvAdc0Ready`, `EvAdc0Sample`, `EvAdc0Window`) |
| RUNSTDBY, DBGRUN | used: `.run_standby`, `.debug_run` |
| initialization timed in hardware | used: `init()` waits for nothing - the first conversion waits in hardware |

### Types and verbs

The DA/DB's surface with the EA's own knobs. `AdcConfig` (the EA's):

| field | values | default | effect |
|---|---|---|---|
| `reference` | `Ref::v1024 / v2048 / v2500 / v4096 / vdd / vrefa` | `vdd` | CTRLC, in its own encoding (`adc_refsel()`) |
| `resolution` | `AdcRes::bits12 / bits8` | 12 | single conversions only |
| `prescaler` | `AdcPresc::div2 .. div64` (16 codes) | `div10` | 2 MHz at 20 MHz, the accuracy table's rate |
| `sample_duration` | 0..255 | 2 | SAMPDUR; >= 4 us x fCLK_ADC with an internal reference and no PGA, >= 11 for a burst through the PGA |
| `accumulate` | 1, 2, 4 .. 1024 | 1 | samples per result |
| `series` | bool | false | one trigger per sample (SERIES) |
| `scaled` | bool | false | the sum scaled to 16 bits |
| `pga`, `gain`, `pga_bias` | bool, `AdcGain::x1 .. x16`, `AdcPgaBias::pct100 .. pct25` | off, x1, 100 % | every input through the PGA |
| `chopping` | bool | false | accumulating modes only |
| `low_latency` | bool | false | LOWLAT |
| `left_adjust`, `free_running`, `run_standby`, `debug_run` | bool | false | |
| `window`, `window_source` | `AdcWindow::none / below / above / inside / outside`, `AdcWindowSource::result / sample` | none, result | refused on a RESULT wider than 16 bits |

`init<cfg>(clock)` static_asserts every rule, one message each (the
accumulation, the modes, free-running, the left adjustment, the window,
the burst through the PGA; the CLK_ADC range and SAMPDUR's floor for a
static clock); `init(clock, cfg)` returns false for the same.

Inputs: `AnalogIn<Pin<..>>` (any AIN, a positive or a negative),
`AdcInput::gnd / vdd_div10 / temp / dac0` (positives; `gnd` and `dac0`
negatives too), `AdcDacref::dacref0 / dacref1` (negatives only), and
the PAIR `AdcDiff<pos, neg>{}` whose positions are checked at compile
time - `AdcDiff<AnalogIn<Pin<'D', 1>>{}, AdcInput::gnd>`. `claim(in)`
turns a pad's digital input buffer off once; `select(in)` is then the
mux store alone; `selected()` reports MUXPOS with bit 7 (`adc_diff_flag`)
set for a pair; `input_code(in)` says it at compile time.

| purpose | verbs |
|---|---|
| configure | `init<cfg>(clock)`, `init(clock, cfg)`, `reconfigure(clock, cfg)`, `disable()`, `config()` |
| what is in effect | `reference()`, `steps()` (4096 / 256), `accumulate()`, `result_steps()` (65536 scaled, 4096 x N), `result_shift()` (0: nothing is truncated), `clock_hz_adc()`, `clock_ok()` |
| inputs | `claim(in)`, `select(in)`, `selected()`, `input_code(in)` |
| convert | `start()`, `stop()`, `busy()`, `ready()`, `sample_ready()`, `result()` (RESULT[15:0]), `result_signed()`, `sum()` (all 32 bits), `sum_signed()`, `sample()`, `sample_signed()`, `read()`, `read_sum()` (blocking; one trigger per sample in series) |
| window | `window(mode, low, high, source)` / `window_signed(...)` (false where refused), `window_thresholds(low, high)`, `window_off()`, `window_hit()` (the last read), `window_flag()`, `clear_window_flag()` |
| interrupts | `enable_resrdy_interrupt`, `enable_samprdy_interrupt`, `enable_wcmp_interrupt`, `enable_error_interrupt`; bodies `resrdy()`, `samprdy()`, `error()` |
| paces | `start_on(EventChannel<n>{})`, `start_on_events(bool)`, `start_on_select(bool)` |
| clock | `rebase(hz)` (a `ClockUser`) |
| arithmetic | `temp_kelvin(result)`; `adc_clock_hz`, `adc_presc_for`, `adc_min_sample_duration`, `adc_period_cycles`, `adc_result_bits`; `adc_sum_mv` in `util/analog.hpp` for a sum of any width |

`AdcConfigured<0, cfg>` is the converter with its configuration as a
type: `init(clock)` is `init<cfg>`, and its `sample_bits` (12, 16 scaled,
12 + log2 N for an unscaled sum) is what `AnalogSampler` checks against
its 16-bit `Sampled.value` - an unscaled sum of 32 or more samples is
refused there at compile time.

THE ENGINE OF A SELECT under the event pace (the mux is not buffered):
COMMAND = STOP with the old input's DIFF; a wait of the output latency
(0.5 CLK_ADC + a CLK_MAIN + one CLK_PER: 4 CLK_PER at CLK_PER / 4, 34
at / 64); the mux; COMMAND = EVENT_TRIGGER with the new DIFF. A
conversion past its end when the STOP lands still delivers its result
through that latency, and the RESRDY body must read the OLD label: the
wait lets it land first. The worst case is one aborted conversion or a
trigger that lands on STOP - a lost sample, never a mislabelled one.

Cost, counted in the release listing (CLK_PER cycles on the AVRxt core,
LDS 3, STS 2): `start()` 9; the RESRDY body (`resrdy()`: the window
match taken and cleared, RESULT's two bytes) 14 and `selected()` 9; a
select under the software pace about 20 (one MUXPOS store), under the
event pace 3 x the latency loops more plus two COMMAND stores (12 loops,
about 60 cycles in all at CLK_PER / 64). One conversion is one store
and, in an ISR, 23 cycles of body; a burst of N is the same for N
samples.

### How to use it

```cpp
#include "avrxt/adc.hpp"
using A = Adc<0>;
using Wire = AnalogIn<Pin<'D', 1>>;

// single-ended, 2.048 V: SAMPDUR >= 4 us x 2 MHz = 8 with an internal reference
A::init<AdcConfig{.reference = Ref::v2048, .prescaler = AdcPresc::div10, .sample_duration = 8}>(clock);
A::claim(Wire{});
A::select(Wire{});
const uint16_t counts = A::read();
```

**A walk mixing single-ended and differential inputs** (DIFF is the
input's):

```cpp
using Pair = AdcDiff<Wire{}, AdcInput::gnd>;       // checked: a legal positive, a legal negative
A::claim(Pair{});
A::select(Pair{});   const int16_t d = static_cast<int16_t>(A::read());   // (VINP - VINN) / VREF x 2048
A::select(Wire{});   const uint16_t s = A::read();
```

**The PGA** (every input of the configuration through it):

```cpp
A::init<AdcConfig{.reference = Ref::v4096, .sample_duration = 11, .pga = true,
                  .gain = AdcGain::x16}>(clock);
```

**Accumulation**: a scaled burst gives 16 bits; an unscaled one a sum of
up to 22 bits, read whole:

```cpp
A::init<AdcConfig{.reference = Ref::v2048, .sample_duration = 8, .accumulate = 1024}>(clock);
const uint32_t sum = A::read_sum();
const uint16_t mv = adc_sum_mv(sum, A::result_steps(), ref_mv(Ref::v2048));
```

**Paced by an event, results as events** (the sampler's shape - the
configuration a type, its width checked):

```cpp
constexpr AdcConfig cfg{.reference = Ref::v1024, .sample_duration = 64};  // TEMPSENSE wants 32 us at 1.024 V
using Conv = AdcConfigured<0, cfg>;
using Sampler = AnalogSampler<Conv, P, Subscribers<Monitor>, Wire{}, AdcInput::temp, AdcInput::vdd_div10>;
Conv::init(clock);
EventChannel<1>::source(EvPitDiv<256>{});
Conv::start_on(EventChannel<1>{});
ISR(ADC0_RESRDY_vect) { post<Sampler>(Sampled{Conv::resrdy(), Conv::selected()}); }
```

**The window**, its flag cleared by the reads:

```cpp
A::window(A::Window::outside, 1000, 3000);
(void)A::read();
if (A::window_hit()) { /* out of band */ }
```

**The temperature**: 1.024 V, SAMPDUR >= 32 us x fCLK_ADC (64 at 2 MHz),
`A::temp_kelvin(A::read())` with `AdcInput::temp` selected.

### Bench findings (`test_avr_analog` on the AVR64EA48, rev B1, 5 V, 20 MHz)

The board's analog net (PD6, the DAC's output, wired to PD1 and PD7)
carries 10 kOhm to ground and, measured by its decay through that
resistor, 96 nF - the 1/e time 0.96 ms, the e^-2 time twice that within
2 %: an RC, not a current sink ([../boards/avr64ea48.md](../boards/avr64ea48.md)).

- **References** through the DAC -> ADC loop (DAC on one internal
  reference, ADC on another, the DAC's internal path and the wire):
  every pair within 0.5 % of its expectation (3058 / 3072 the worst), the
  wire 3..10 counts under the internal path (the buffer's offset into
  the 10 k).
- **VDDDIV10** reads 5070 .. 5080 mV where the DAC against VDD says 5085
  (0.3 %; the chapter states no accuracy).
- **TEMPSENSE**: TEMPSENSE0 = -566, TEMPSENSE1 = -4967 (both signed);
  2783 counts at 1.024 V -> 302 K = 29 C by the EA formula, the board's
  room. The DB's formula on the same reading gives nonsense.
- **DACREF as the negative** (A14): the wire at 1.0 V against DACREF0
  at 0.5 V reads +428 at SAMPDUR 8 (4 us), +477 at 32 (16 us), +489 at
  128 (64 us), the same at 255: DACREF wants ~30 us of sampling, as on
  the DB.
- **The PGA**: every gain within -0.3 .. +1.1 % of the direct reading
  times the gain (the +1.1 % at x16 being the direct reading's own
  1.5-LSB offset over its 194 counts); noise 0.8 / 0.8 / 1.0 / 1.4 / 2.0
  LSB12 rms at x1 .. x16 on a 4.096 V reference, i.e. 800 .. 125 uV
  referred to the input - table 35-26's 32 .. 500 uV scale as 32 uV x
  gain, matching neither reading (A2). A burst through the PGA at
  SAMPDUR 11 reads as at 12 (A11). A GND - GND pair through the PGA,
  1024 samples summed: -0.5 LSB a sample at x1 and -1.6 .. -1.8 at x16
  plain, -0.6 and -1.8 .. -1.95 chopped - chopping does not lower it
  here (A3, below).
- **Accumulation**: burst and series sums of 2 .. 1024 agree with the
  single reading per sample within 1 count; scaled results = 16 x the
  single reading within 8 scaled counts; a 1024-sample sum at full
  scale is 4167206 (22 bits); one burst of 64 raises 64 SAMPRDY
  events and one RESRDY.
- **The flags** (A8): a 16-bit read of RESULT (its low byte first)
  clears RESRDY, as does the 32-bit read; a SAMPLE read leaves it. A
  trigger mid-conversion sets TRIGOVR and is ignored; an unread result
  and sample overwritten set RESOVR and SAMPOVR.
- **STOP and the conversion's end** (A9): of 52 STOPs placed 6 us down to
  0 before the end of an 877 us conversion at 312.5 kHz, 32 aborted it
  (no RESRDY, no SAMPRDY), 16 came after the result, and 4 saw the result
  rise AFTER the STOP - at most 23 CLK_PER later: the output latency of
  figure 31-3's note 3. A COMMAND rewritten to EVENT_TRIGGER mid-
  conversion lets it complete.
- **A mux write mid-conversion aborts it** - to another pin as to GND -
  where the chapter names only DAC0, TEMPSENSE and VDDDIV10 as aborting;
  the next conversion is clean.
- **The select engine** (letter s): an 80 us event pace on conversions
  of ~56 us at 312.5 kHz, the RESRDY body labelling each result with
  `selected()`, the select of the other input made after each result
  with a lag swept over the whole period: 0 of 4000 mislabelled with
  the driver's select, 32 of 4000 with a bare MUXPOS store (all at one
  lag, the store landing in the output latency), in each of fifteen runs.
- **The window**: the four modes at the exact codes; WCMP survives a
  later non-matching conversion until written 1; INSIDE is strict (GND
  reads 0 in 8-bit mode and is not inside (0, 10)); the signed window in
  DIFF; on SAMPLE a burst matches per sample.
- **Initialization per trigger** (table 31-5), software-started one-shots
  with LOWLAT off against on: the ADC 9 us, the PGA 20, the 1.024 V
  reference 59, TEMPSENSE and the DAC input 35 - the table's figures. A
  free-running conversion or a burst pays none: their rates are the
  same with LOWLAT off.
- **PGACTRL** reads 0x00 after reset (its register box says 0x04, its
  fields 0: the fields are right, A6). MCLKTIMEBASE 20 (the clock's) and
  the first conversion with an internal reference is right.
- **A live rebase** 20 -> 10 MHz under a DynamicClock: CLK_ADC goes 2
  MHz -> 1.67 MHz (the nearest in range), the reading does not move; back
  at 20 MHz it is init's 2 MHz again (the rebase aims at init's CLK_ADC,
  not the last one).
- **The sampler** (`sampler`, `AdcConfigured<0, cfg>` walking PD1, the
  die temperature and VDDDIV10 at 1.024 V): 512 samples a second under
  the PIT/64 pace with no queue drop and no unknown code; across a clock
  switch 20 -> 10 -> 20 MHz with the pace paused by the owner.

**The conversion rates** of tables 31-7 (5 MHz = 20 MHz / 4, PGA bias
100 %) and 31-8 (312.5 kHz = 20 MHz / 64, bias 25 %), free-running,
counted by the event system over 100 ms (samples per second; the
table's figure, then `adc_period_cycles`'s):

| SAMPDUR, PGA | 8-bit | 12-bit | burst |
|---|---|---|---|
| 20 | 151740 (161290) | 138970 (142857; 138888) | 147140 (147059; 147058) |
| 64 | 65220 (66667) | 62540 (63291; 62500) | 64140 (64103; 64102) |
| 128 | 35610 (35971) | 34740 (34965; 34722) | 35230 (35211; 35211) |
| 255 | 18730 (18797) | 18460 (18519; 18450) | 18590 (18587; 18587) |
| 15, PGA | 114140 (120482) | 106450 (109890; 106382) | 151540 (156250; 151515) |
| 20, PGA | 102470 (107527) | 96210 (99010; 96153) | 131610 (135135; 131578) |
| 64, PGA | 53990 (55249) | 52110 (52910; 52083) | 61000 (61728; 60975) |
| 255, PGA | 17680 (17762) | 17430 (17513; 17421) | 18320 (18382; 18315) |
| 2 at 312.5 kHz | 21620 (24038) | 17830 (18382; 17825) | 19540 (19531; 19531) |
| 11 | 13330 (14205) | 11780 (12019; 11778) | 12500 (12500; 12500) |
| 64 | 4100 (4167) | 3930 (3956; 3929) | 4000 (4006; 4006) |
| 255 | 1170 (1175) | 1150 (1157; 1155) | 1160 (1162; 1161) |
| 2, PGA | 16930 (18116) | 14520 (14706; 14513) | - |
| 11, PGA | 11380 (11905) | 10240 (10331; 10235) | 18370 (18657; 18382) |
| 64, PGA | 3890 (3943) | 3740 (3754; 3741) | 4460 (4480; 4464) |
| 255, PGA | 1150 (1156) | 1130 (1139; 1138) | 1190 (1198; 1197) |

The burst without the PGA runs exactly as the table says. Every other
column runs SLOWER than the table, and each by a law the chapter states
elsewhere or the measurement fixes: a single conversion's free-running
period is the chapter's (SAMPDUR + 15) CLK_ADC PLUS the output latency
of note 3 (0.5 CLK_ADC and a CLK_MAIN) plus one CLK_PER - the next
conversion starts only once the result has landed; and through the PGA
the ADC samples it for WHOLE CLK_ADC cycles, floor(tPGA x fCLK_ADC) + 1
of them (16 at 5 MHz for the 3 us, 4 at 312.5 kHz for the 12), where
the table adds the bare microseconds. `adc_period_cycles()` carries
both, and every 12-bit and burst cell agrees with it within 0.2 % in
each of fifteen runs. The 8-bit column runs 0.4 .. 10 % under the table
and up to 6 % under the same law at (SAMPDUR + 11) - its slope per
SAMPDUR count 0.993 of a CLK_ADC at 5 MHz, the 12-bit's exactly 1: not
explained (below). The
data sheet's 375 ksps needs CLK_ADC 6 MHz (12 MHz / 2); at 20 MHz the
fastest is a burst at 5 MHz and SAMPDUR 0, 357 ksps by the formula the
table's burst rows confirm.

## Not covered yet

Driver gaps:

- `left_adjust` on the DA/DB stays write-only by choice: the result
  helpers assume right adjustment and say so (the shift varies with the
  accumulation count, DS40002247B Table 33-1) - left-adjust-aware
  helpers only if a user appears.
- Per-package input legality (PE/PF pins on 28/32-pin parts, PD0 on
  DB 28/32, PF2-5 on the 28-pin EA) - deferred to the family device
  tables.
- `AdcDiff` on the DA/DB: its differential conversion stays the
  configuration's (`.differential`, CTRLA.CONVMODE, changeable only
  idle) and its two-argument `select`; the pair as an input value is
  the EA's alone until a DA/DB board is on the desk to measure the
  mixed walk there.
- On the EA, the per-sample stream (SAMPRDY into a ring): declined at
  the rates where it matters (64 CPU cycles a sample at 312 ksps); a
  block engine is born with its first user.
- The DA/DB's `rebase()` aims at the LAST CLK_ADC, not init's as its
  comment says (`clk_adc_hz_` rewritten by each rebase): a round trip
  of the clock can leave it slower. The EA's keeps init's target apart;
  the same fix moves every DA/DB image that rebases - owed with a DA/DB
  on the desk to run them.
- The 8-bit single conversion's free-running rate on the EA: 0.4 .. 10 %
  under table 31-7/31-8, up to 6 % under `adc_period_cycles()`, its
  period growing 0.993 CLK_ADC per SAMPDUR count against the 12-bit's 1
  - not explained; what would explain it is the 8-bit conversion state
  timed on its own (TCA capture of the RESRDY event against the start).

Implemented but not bench-verified:

- every leg of the DA/DB half that reads PD1 or takes VREFA from PD7,
  on the AVR128DA48: with the two wires fitted, the DAC's pin falls at
  ~200 V/s and holds a rising step's overshoot for milliseconds there
  ([dac.md](dac.md)) - on the AVR EA's assembly of the same PCB the net
  measures 96 nF, which would make that fall a sink of ~20 uA rather
  than a pin's weak one; the DA's own net is not measured;
- the 200 us reference start-up case on the DA/DB (32.768 kHz main
  clock: a serial console cannot follow it, so it needs a sleepy
  application); `debug_run` (needs a debugger session); on the EA
  `run_standby` in a real Standby (test_avr_sleep's letter o is the
  DA/DB's);
- on the EA, the supply current per PGA bias and with LOWLAT (A13 of
  the survey: no meter on the desk) - the energy side of LOWLAT's
  default;
- on the EA, the ERROR vector bound (`error()` is measured polled) and
  WINSRC = SAMPLE's interrupt on the SAMPRDY vector;
- on the EA, CHOPPING's offset reduction (31.3.3.5.2): on a GND - GND
  pair through the PGA the chopped sum carries no less offset than the
  plain one (letter p) - an input pair with a known, larger offset
  would show what chopping cancels.
