# ADC (CH32V00x)

The one 12-bit converter of RM ch. 9 - eight pads and two internal
sources, a rule group of sixteen and an injection group of four, seven
internal triggers a group, three analog watchdogs that can reset the
chip, one DMA request - and util/analog_sampler.hpp's AnalogSampler
running over it unchanged. Documents of record: the CH32V00X reference
manual V1.5 (9.2.2 for the configuration and tCONV, 9.2.3 and tables
9-3/9-4 for the triggers, 9.2.4 for the modes, 9.2.5 for the
watchdog, 9.3 for the registers), the CH32V006 datasheet V2.0 (table
3-5 for VREFINT, the ADC characteristics, table 2-1-1 for the pads).

## What the silicon does

- **The STM32F1's ADC1** under WCH's names: ADON written once wakes
  the converter and written again starts it, SWSTART under EXTSEL =
  111 with EXTTRIG is the other software start, the rule sequence
  in RSQR1..3 and the injection one in ISQR (its slots filling from
  the END, its results counting from the START), a per-slot offset
  and a SIGNED injected result, the flags write-zero-to-clear.
- **No calibration**: no CAL bit, no procedure, the vendor's own init
  runs none.
- **A third control register** (CTLR3): ADC_LP (set at reset, the mode
  for rates under a megasample, and the mode the sample-time table
  depends on), a clock duty-cycle knob, and the three watchdogs'
  results and reset enables.
- **ADCCLK is HCLK through a two-level five-bit code** (RCC_CFGR0.
  ADCPRE): twelve dividers from /2 to /128; the datasheet caps fADC at
  48 MHz, so /2 = 24 MHz at this stratum's 48 MHz.
- **tCONV = sample + 12.5 ADCCLK cycles** (measured to the cycle, four
  settings): 16 cycles at the shortest sample - a megasample and a
  half at 24 MHz.
- **VREFINT is 1.2 V on channel 8** (DS table 3-5), the OPA's output
  on channel 9; the pads are PA2 (0), PA1 (1), PC4 (2), PD2 (3), PD3
  (4), PD5 (5), PD6 (6), PD4 (7).
- **More than one conversion of a group needs SCAN** (9.2.4's table):
  without it the first channel alone converts, in either group.
- **THE WATCHDOG SCAN** (AWD_SCAN, measured, the vendor's own example's
  arrangement): with it set and a SCANNED rule sequence, watchdog 0
  compares the FIRST conversion against WDHTR/WDLTR, watchdog 1 the
  SECOND against WDTR1, watchdog 2 the THIRD against WDTR2, each
  result in CTLR3's AWDx_RES - 9.3.15's "only applicable to watchdog
  channel 1" is that rank. Without the scan, watchdogs 1 and 2 compare
  nothing.
- **A watchdog can RESET THE CHIP** (AWDx_RST_EN): the boot reads
  ADCRSTF in RCC_RSTSCKR and nothing else.
- **THE STALL** (measured): with a rule group paced by a hardware
  trigger and served by the DMA, and the CPU accessing the peripheral
  buses meanwhile, the converter has been seen to stop converting -
  STRT standing, no EOC ever again - within the first conversions of a
  run, at rates from none to a tenth of the runs depending on the
  CPU's access pattern (a poll of GPIOC's INDR in one build of the
  loop, of the ADC's own STATR in another; the same pattern quiet in
  a third), and once in two hundred runs with an EOC interrupt reader
  and no DMA. Neither a read of RDATAR nor a software start revives
  it; a power cycle of ADON does.
- **The console's transmitter perturbs the readings**: VREFINT read
  while a line leaves USART1 on PD5 shows spikes of 40..60 counts;
  with the console quiet the spread over 64 readings is 6 counts.
- **VREFINT wants a slow sample**: at 3.5 cycles of a 24 MHz clock a
  block of 256 readings spreads 430 counts; at 28.5 it is settled.

## Types and verbs

[brio/ch32v00x/adc.hpp](../../brio/ch32v00x/adc.hpp): `Adc`, a
monostate - `init(clock, AdcConfig)` (the prescaler code, alignment,
continuous, scan, JAUTO, the low-power bit, the input buffer, DMA,
the discontinuous lengths; refused by `adc_config_valid()` and for an
ADCCLK above 48 MHz), `power()`, `recover()` (the ADON cycle the stall
wants), `sample_time(ch, AdcSampleTime)`, `sequence(order, n)` and
`select()` (a one-conversion sequence: util/analog_sampler.hpp's
verb, with `start()`, `selected()` and `input_code()`), `read()` /
`read_settled()` / `result_counts()`, `supply_mv(vrefint_counts)` and
`millivolts()` over util/analog.hpp, `trigger(AdcTrigger)` and
`injected_trigger(AdcInjectedTrigger)` (tables 9-3 and 9-4),
`injected_sequence()`, `injected_offset()`, `injected_start()`,
`injected_result(slot)` signed, `watchdog0(AdcWatchdogConfig)` (the
thresholds, the scope, the interrupt, the reset), `watchdog(n, low,
high, reset)` for 1 and 2, `watchdog_scan()`, the results and their
clearing, the flags, the three interrupt enables and `isr()`.
`AnalogIn<Pin>` names a pad and derives its channel (a pad that is no
input is refused); `AdcInput::vrefint` and `AdcInput::opa` are the two
internal sources; `Ref::vdd` and `adc_vrefint_mv` are this target's
util/analog.hpp vocabulary. The arithmetic: `adc_prescaler_for(hclk,
max_hz)`, `adc_prescaler_divider(code)`, `adc_conversion_half_cycles(t,
low_power)`.

## How to use it

```cpp
using Vin = brio::AnalogIn<brio::Pin<'D', 2>>;      // channel 3
brio::Adc::init(clock, {.prescaler_code = brio::adc_prescaler_for(48'000'000, 24'000'000)->code});
brio::Adc::sample_time_all(brio::AdcSampleTime::cycles28_5);
Vin::claim();

brio::Adc::select(brio::AdcInput::vrefint);
const uint16_t vdd_mv = brio::Adc::supply_mv(brio::Adc::read_settled(4));
brio::Adc::select(Vin{});
const uint16_t mv = brio::Adc::millivolts(brio::Adc::read(), vdd_mv);
```

The sampler: `AnalogSampler<Adc, P, Subscribers<...>, AdcInput::vrefint,
Vin{}>` with the EOC handler posting `Sampled{Adc::result_counts(),
Adc::selected()}`.

## Bench findings

The reference suite is `test_ch32_adc` (23 verdicts in `z`, one letter
ending in a real reset, two probe letters outside `z`) on the
CH32V006K8U6 at 48 MHz, nothing wired.

- **The supply from the reference**: 1495..1504 counts of VREFINT at
  the eight sample times, 3275..3294 mV - the probe's 3V3.
- **The rate**: 256 continuous conversions through DMA channel 1 in
  516197 cycles at 6 MHz and 239.5 (516096 computed), 32890 at 6 MHz
  and 3.5 (32768), 21112 at 24 MHz and 28.5 (20992), 8337 at 24 MHz
  and 3.5 (8192): tCONV to the cycle, 1.47 Msps at the top.
- **The triggers**: TIM1 and TIM2's TRGO, CC1 and CC2 at 10 kHz each
  deliver 1000 conversions in 100.0 ms through the DMA; TIM3's CC1
  match paces the injection group at 10 kHz (1000 JEOCs in 100 ms) -
  the streamlined timer's purpose.
- **The injection group**: four slots of VREFINT with offsets 0, 100,
  VREFINT and 4095 read 1497, 1402, 4 and -2597 - signed; under JAUTO
  the injected conversion follows the rule one by itself; left
  alignment puts the datum in bits 15..4.
- **The watchdogs**: watchdog 0 around VREFINT quiet inside its window
  and raising AWD with its interrupt on every conversion outside; the
  scan's three ranks each guarded by their own watchdog, a fault at
  the rank whose window excludes it and nowhere else; armed with
  AWD0_RST_EN and a window above the reading, the next conversion
  REBOOTS THE BOARD with ADCRSTF alone.
- **The pads through their pulls**: PD2 and PC4 read 4095 pulled up
  and 3..4 pulled down.
- **The sampler**: util/analog_sampler.hpp's AnalogSampler over VREFINT
  and PD2 at one conversion per 5 ms delivers 20 samples of each in
  205 ms, every one labelled by its input.
- **The stall and the console** are the two findings above; the stall
  hunt (letter `y`, six arrangements of a hundred runs) is where the
  numbers come from.

## Not covered yet

Driver gaps, each with its reason:

- TouchKey (TKENABLE, TKITUNE, the DRV bits): an application-level
  mode, declined until an application wants it.
- The OPA as a trigger source and as channel 9's signal: the OPA
  chapter's.
- The discontinuous modes beyond their configuration bits: no user.
- The external trigger pads (PD3/PC2, PD1/PA2): a wire to a source.
- The input buffer (BUFEN) as a measurement: a high-impedance source on
  a pad.

Implemented but not bench-verified, each with what would measure it:

- The stall's cause: a scope on the bus arbitration is not on this
  desk; what the suite offers is the recovery and the count. A rule of
  thumb until then: a stream that must not die watches its own
  progress and calls `recover()`.
- A known external voltage on a pad: the pulls are two levels, a
  divider on a jumper would be a third.
