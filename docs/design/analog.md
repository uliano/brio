# The analog block: VREF, DAC, ADC

Second step of the exhaustive-driver track: the three peripherals a
single wire (DAC out -> an ADC pin) turns into a closed, self-measuring
loop. First part: what the silicon offers, read from DS40002247B (ch.
21, 34, 33, electricals 39.12/39.18/39.19, pin table 3.1) and errata
DS80000915F. Second part: the brio representation - one type per
peripheral, and the reasoning that says why the ADC, unlike a timer,
is ONE task with knobs and not several.

## VREF (ch. 21)

Three independent reference SELECTIONS, one per consumer: `ADC0REF`,
`DAC0REF`, `ACREF` (the three comparators share one). Each picks
1.024 / 2.048 / 2.500 / 4.096 V (internal bandgap-derived, +-4 %
untrimmed typ. over temperature; the higher ones need VDD headroom:
2.048 needs VDD >= 2.5 V, 2.5 needs 2.95 V, 4.096 needs 4.55 V), VDD,
or the external VREFA pin (PD7, 1.024 V .. VDD, ~50 kOhm ladder for
the ADC). A source is enabled automatically when its consumer needs
it; `ALWAYSON` keeps it running (start-up 10 us from an internal
main clock, 200 us from an external one; 2 us to change level) at
the cost of ~40-175 uA. Nothing else: this is a selector, not a
device with behaviour. Facts that matter to code: the reference is
part of every conversion's meaning (counts -> volts), the ADC and the
DAC may use different ones, and the DAC->ADC loop is exact only when
they use the same one (or known ones).

## DAC (ch. 34)

One instance, DAC0. 10-bit, `DATA` left-adjusted (`DATA[9:2]` in
DATAH, `DATA[1:0]` in bits 7:6 of DATAL - the output updates when
DATAH is written), range GND .. VDACREF. `CTRLA`: `ENABLE`, `OUTEN`
(the buffered output on pin PD6 - the pin's digital input must be
disabled in PORT), `RUNSTDBY`. The UNBUFFERED output feeds the ADC
(MUXPOS `DAC0` = 0x48), the ACs and the OPAMPs internally with OUTEN
off, leaving PD6 free. Electricals: 0.1 V .. VDD-0.1 V out, 1 mA
source / 1 uA sink (add a resistor to ground if it must sink),
settling ~7-10 us full scale, INL +-2.3 LSB, offset +-5 LSB, gain
+-3 LSB (spec'd for codes 0x030..0x3D0). Errata 2.6.1 (silicon
A4/A5): output buffer offset drifts over lifetime if powered with
OUTEN off - keep OUTEN on, or calibrate the offset with the ADC.
That is all: a value-to-voltage actuator with three bits of
configuration.

## ADC (ch. 33)

One instance, ADC0. 12-bit SAR (10-bit optional, 2 CLK_ADC faster),
up to 130 ksps, CLK_ADC = CLK_PER / {2,4,8,12,16,20,24,28,32,48,64,
96,128,256} within 125 kHz .. 2 MHz (period 0.5-8 us; the accuracy
tables are at 500 kHz), warm-up 6 us after ENABLE (or `INITDLY` =
0/16/32/64/128/256 CLK_ADC cycles, also for reference settling out
of standby). Inputs: `MUXPOS` = AIN0-21 (AIN0-7 = PD0-7, AIN8-15 =
PE0-7 (PE0-3 on 48 pins), AIN16-21 = PF0-5), GND, TEMPSENSE,
VDDDIV10, VDDIO2DIV10, DAC0, DACREF0-2 (the AC DAC references);
`MUXNEG` = AIN0-15 (not AIN16-21), GND, DAC0. Modes:

- **single-ended / differential** (`CONVMODE`): unsigned 0..4095 or
  signed -2048..2047 (sign-extended), full scale = VADCREF, rail to
  rail;
- **single conversion / free-running** (`FREERUN`); a conversion is
  started by software (`COMMAND.STCONV`), by an EVENT (`EVCTRL.
  STARTEI`, rising edge, async - works in standby), or, in
  free-running, by the previous one finishing; `SPCONV` stops;
- **accumulation** (`SAMPNUM` = 1..128 samples per result, summed in
  hardware, result truncated to 16 bits above 16 samples; the
  accumulator resets per conversion; RESRDY once per result) - the
  hardware oversampler;
- **sampling control**: `SAMPLEN` (0-255 extra CLK_ADC cycles of
  sample time, for high-impedance sources: >10 kOhm), `SAMPDLY`
  (0-15, to move the sampling frequency off a noise harmonic),
  `INITDLY`; conversion time = 2/fCLK_PER + n x (13.5 + 2 + SAMPDLY +
  SAMPLEN)/fCLK_ADC + 2/fCLK_PER;
- **result**: `RES` 16-bit, right- or left-adjusted (`LEFTADJ`;
  left-adjusted makes the window thresholds independent of the
  accumulation count), a `TEMP` register for the 16-bit access
  order;
- **window comparator** (`CTRLE.WINCM` = below / above / inside /
  outside `WINLT`..`WINHT`, applied to the accumulated result), its
  own interrupt `WCMP`;
- **interrupts**: `RESRDY`, `WCMP`; **events**: generator `RESRDY`
  (pulse), user `START`; `RUNSTDBY`; `DBGRUN`;
- **temperature sensor**: MUXPOS TEMPSENSE with the 2.048 V reference,
  INITDLY >= 25 us, SAMPLEN >= 28 us, 40 us after switching the mux;
  T[K] = (SIGROW.TEMPSENSE1 - result) x SIGROW.TEMPSENSE0 / 4096;
- **supply monitors**: VDDDIV10 / VDDIO2DIV10 (+-10 %).

Rules from the datasheet and errata: never change CONVMODE/LEFTADJ/
RESSEL/SAMPNUM/PRESC during a conversion (in free-running: disable
FREERUN, wait, change, re-enable); MUX changes are buffered and take
effect at the next conversion; errata 2.3.2 (all silicon): with
INITDLY != 0, a MUXPOS/MUXNEG/SAMPNUM change after enabling or after
a reference change takes effect only after one conversion - so set
them before enabling, or throw a dummy conversion away; errata 2.3.1
(A4): -3 mV single-ended offset. Accuracy (12-bit, 500 kHz, 3.0 V ref):
INL +-1.8, DNL +-1, offset 2.5 typ / 5 max, gain +-5 LSB; source
impedance <= 10 kOhm recommended.

## The brio representation

Following [overview.md](overview.md) "Target strata": tasks named for
what they do, over named resources; no HAL; tables and knobs grow on
demand; apps never touch registers.

### VREF: a vocabulary, not a device

`avrdx/vref.hpp`: `enum class Ref { v1024, v2048, v2500, v4096, vdd,
vrefa }`, `constexpr uint16_t ref_mv(Ref, uint16_t vdd_mv = 0)` (the
one truth for counts <-> millivolts), and three tiny setters
`Vref::adc0(Ref, bool always_on)`, `Vref::dac0(...)`, `Vref::ac(...)`.
The ADC and DAC configs name their `Ref` and their `init()` calls the
setter: users never write VREF registers, and each consumer's
reference is stated where that consumer is configured. Whether the
same `Ref` is used on both sides of the loop is the app's decision,
visible in two lines next to each other.

### DAC: an actuator, rank of Pin

`avrdx/dac.hpp`, `Dac<0>` (the instance is a template parameter for
symmetry with everything else, even if DB has one): `init({.reference,
.output_pin = true, .run_standby = false})` (selects the reference,
disables PD6's digital input, sets OUTEN/RUNSTDBY/ENABLE), `set(code)`
(0..1023, one 16-bit write in the right order), `enable()/disable()`,
and constexpr helpers `dac_code(mv, ref_mv)` / `dac_mv(code, ref_mv)`.
Synchronous, no events - like a PWM channel; a fade is an AO above.
Errata 2.6.1 rule in the header: OUTEN stays on for the life of the
program on A4/A5 silicon. It could satisfy a future `AnalogOut`
concept next to `PwmChannel`; not before a second specimen.

### ADC: ONE task with knobs - and why not several

The question the datasheet had to answer: does the ADC decompose into
tasks the way a TCB does (capture / one-shot / periodic are different
state machines sharing a counter)? No. Every use - single reading,
paced stream, oversampled reading, window watch, temperature, supply
monitor - is the SAME sequence (select input, trigger, wait RESRDY,
read RES) with different knobs (trigger source: software / event /
free-run; accumulation; window; mux) and different consumers of the
same two events. Splitting it into `AdcOneShot`, `AdcSampler`,
`AdcWindow` would give three types that all own the same registers
and cannot coexist anyway (one converter). So:

- `avrdx/adc.hpp`, `Adc<0>`: `init(AdcConfig)` with a `constexpr`
  config struct (designated initializers: `.reference`, `.resolution`,
  `.mode` single/diff, `.prescaler`, `.sample_length`, `.init_delay`,
  `.accumulate` 1..128, `.left_adjust`, `.free_running`,
  `.run_standby`) - the whole configuration owned in one place, the
  errata order respected inside (mux/samples before enable);
- inputs as TYPES: `AnalogIn<Pin<'D', 1>>` (pin -> AINn table: PD0-7,
  PE0-7, PF0-5 - the seed of the device table), `AdcIn::gnd/temp/
  vdd_div10/vddio2_div10/dac0/dacref0..2` for the internal ones;
  `select(pos)`, `select(pos, neg)` (differential; static_assert that
  neg is AIN0-15/GND/DAC0);
- `start()`, `stop()`, `busy()`, `result()`; `window(mode, lo, hi)`;
- ISR bodies `resrdy()` and `wcmp()` returning what the app posts
  (`AdcResult{value}`, `AdcWindowHit{value}`), the vector bound by
  the app - the same pattern as every driver: the ISR condenses, the
  AO consumes; at 130 ksps the accumulator is what turns samples into
  events at a sane rate;
- event start: `Adc<0>` is an event USER - `EvAdc0Start` in
  evsys.hpp (three lines) + `Adc<0>::start_on(EventChannel<n>{})`
  setting STARTEI; and a GENERATOR, `EvAdc0Ready`;
- pure helpers in `util/` (host-tested): counts <-> mV for a
  reference and resolution and accumulation, the temperature formula
  from the two signature-row factors, VDDDIV10 to mV.

The "tasks" of the ADC are therefore CONSUMER patterns, and they live
in the AO that owns the converter: a paced sampler is an AO whose init
routes PIT/64 -> ADC start and whose handler receives `AdcResult`; a
threshold monitor is one that configures `window()` and receives
`AdcWindowHit`; a temperature reader is one that selects TEMPSENSE
with the right delays. Ownership of the one converter belongs to one
AO's FSM (as with a rewired event channel): a second consumer asks it
(request/reply), it does not touch the registers - the shape a
future `AnalogMux` AO takes if two AOs ever want readings.

What is deliberately NOT abstracted: the meaning of the knobs. The
config struct names them as the datasheet does (`sample_length`,
`init_delay`, `accumulate`), the header explains what each buys, and
the app that needs 28 us of sample time for the temperature sensor
computes it from `clock_hz(clock)` and the prescaler with a constexpr
helper. Knob-heavy peripherals get a config struct, not an API that
pretends to understand analog design.

## Bench

- One wire: **PD6 (DAC0 OUT) -> PD1 (AIN1)**. The internal path
  (MUXPOS = DAC0) is the cross-check that costs no wire; PD7 (VREFA)
  free for an external reference experiment; the loop is exact when
  both use the same `Ref` (2.048 V: full DAC range 0-2 V, ADC 0.5 mV
  per LSB).
- Two apps: `test_avr_analog`, the self-test SUITE (bench diagnostic,
  no kernel: 14 tests, one knob group each, PASS/FAIL against the
  datasheet's tolerances - references cross-check, ramp, OUTEN,
  settling, resolution, differential, prescalers vs the timing
  formula, accumulation, sampling knobs, event start from PIT/64,
  window modes on a ramp, errata 2.3.2 shown then cured by flush(),
  internal inputs and temperature, VREFA driven by the DAC itself;
  the supply is measured at start and the reference set follows it -
  run at 3.3 V and at 5 V);
  `analog1`, the kernel integration (results as events from the
  RESRDY body, event-paced sampling, window hits as events,
  reconfiguration under a running program). What one wire cannot
  test: absolute accuracy of the internal references (no trusted
  meter), the DAC's drive current (taken from the datasheet),
  RUNSTDBY/ALWAYSON timing, DACREF0-2 (needs the ACs), the RESRDY
  event as a generator (needs a TCB).

## Bench findings (test_avr_analog, silicon A5, 3.3 V: 54/54)

- The converter's warm-up after ENABLE is real: the first conversion
  without waiting t_ADC_INIT (6 us typ.) is garbage. `Adc::init(clock,
  cfg)` waits 10 us; that is why it takes the clock.
- INITDLY is paid ONCE, for the first conversion after enable - not
  per software start (2240 one-shots/100 ms with INITDLY 256 at 375
  kHz, i.e. the plain rate).
- The unbuffered DAC0 input (MUXPOS DAC0) is high-impedance: with the
  default 2-cycle sampling at 1.5 MHz it reads 3-4 % low; with
  `sample_length` 32 it reads true. Same for any source above the
  10 kOhm the datasheet recommends - the knob exists for this.
- WCMP is cleared by reading RES (33.5.12): a driver that reads the
  result must capture the window verdict first (`result()` does;
  `window_hit()` reports it).
- Accumulation above 16 samples truncates RES (32: 1 bit, 64: 2, 128:
  3) - `result_shift()`.
- The DAC's buffered output RISES in ~10-20 us but FALLS at its sink
  limit: ~1 uA into the pin capacitance = ~20 kV/s (measured on the
  bare PD6->PD1 wire: 2 V lost in ~100 us, the last tens of mV much
  slower). Fast falling edges need the datasheet's resistor to ground.
  A ~50 us ring follows a rising step. The buffered pin sits ~13 mV
  above the unbuffered internal path: the buffer's offset (spec
  +-10 mV).
- References cross-check within 1-2 % (spec 4 %); prescaler rates
  within 1 % of the timing formula; event start from PIT/64: 513/s;
  VDD/10 reads 3290 mV on a 3300 mV rail; die temperature 27 C from
  the signature-row factors; errata 2.3.2 reproduced and cured by
  `flush()`; VREFA driven by the DAC works down to 1.04 V.

## The guiding application: Multislope

uliano/AVR-Multislope (and its SAM-Multislope twin) is the app that
will drive the timer/CCL/AC tasks after the analog block: TCA0 as a
375 kHz heartbeat with two waveform outputs, TCB0 single-shot gate,
TCB1 event counter extended by its OVF interrupt, TCB2->TCB3 cascade
as the modulo-N window counter (a task over TWO resources), CCL LUTs
as a synchronising flip-flop and gates, AC1, five fixed event routes
(the static `EventSystem` sugar's first real user), the ADC started
by the window-end event. Checked against the model: every piece is a
fixed route + a task on a resource + a config struct; the CLEAN /
PREV_CHARGE / NEGATIVE_COUNTS / RESULT_AVAIL state machine that today
spans two ISRs and a superloop over volatiles becomes one Fsm fed
`WindowEnd{counts}` and `AdcResult{value}` in order. The one genuine
timing constraint - snapshot-and-reset of the negative counter within
one heartbeat period (64 CPU cycles) after the window-end event -
stays in the ISR body, as the model prescribes; the kernel queue
carries only the millisecond-scale pair that follows.

## Later, not now

The DB's OPAMP block (three op amps with resistor ladders, DAC as
input, event-controlled) is the natural continuation of this analog
line and the biggest "task over a resource" candidate; the ACs and
ZCDs likewise. Not before the ADC is exhausted.
