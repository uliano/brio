# ADC (CH32V00x)

The one converter of RM ch. 9 - eight pads and two internal sources, a
rule group of sixteen and an injection group of four, timer triggers a
group, an analog watchdog, one DMA request - and util/analog_sampler.hpp's
AnalogSampler running over it unchanged: 12 bits with a third control
register and two more watchdogs on the CH32V006, 10 bits with the F1's
calibration and a trigger delay register on the CH32V003. Documents of
record: the CH32V00X reference manual V1.5 (9.2.2 for the
configuration and tCONV, 9.2.3 and tables 9-3/9-4 for the triggers,
9.2.4 for the modes, 9.2.5 for the watchdog, 9.3 for the registers),
the CH32V006 datasheet V2.0 (table 3-5 for VREFINT, the ADC
characteristics, table 2-1-1 for the pads); for the CH32V003 its
reference manual V1.9 (the same chapter 9: 9.2.2 for the calibration
and its tCONV, 9.3.15 for DLYR) and its datasheet V1.6 (3.3.14 for the
ADC characteristics).

## What the silicon does

- **The STM32F1's ADC1** under WCH's names: ADON written once wakes
  the converter and written again starts it, SWSTART under EXTSEL =
  111 with EXTTRIG is the other software start, the rule sequence
  in RSQR1..3 and the injection one in ISQR (its slots filling from
  the END, its results counting from the START), a per-slot offset
  and a SIGNED injected result, the flags write-zero-to-clear.
- **No calibration on the CH32V006**: no CAL bit, no procedure, the
  vendor's own init runs none. **The CH32V003 has the F1's**: RSTCAL
  then CAL, each cleared by the hardware, recommended at every
  power-up - `init()` runs it there and the code lands in RDATAR
  (0x200 on the bench, the same reading before and after).
- **A third control register on the CH32V006** (CTLR3): ADC_LP (set at
  reset, the mode for rates under a megasample, and the mode the
  sample-time table depends on), a clock duty-cycle knob, and the
  three watchdogs' results and reset enables. **The CH32V003 spends
  the word at 0x50 on DLYR**, a delay of up to 511 ADCCLK cycles
  between an external trigger and the group it starts, for the rule
  or the injection group - and has none of CTLR3's: no low-power mode,
  no input buffer, one watchdog, no watchdog reset. Its CTLR1 carries
  CALVOL instead, 01 at reset and "invalid" at zero, so `init()` never
  writes the field to zero.
- **ADCCLK is HCLK through a two-level five-bit code** (RCC_CFGR0.
  ADCPRE, the same encoding on both parts): twelve dividers from /2
  to /128; the datasheets cap fADC at 48 MHz on the CH32V006 (/2 =
  24 MHz at this stratum's 48 MHz) and at 12 MHz on the CH32V003 (/4).
- **tCONV = sample + 12.5 ADCCLK cycles on the CH32V006, + 11 on the
  CH32V003** (measured to the cycle, four settings on each): 16
  cycles at the shortest sample on the first - a megasample and a
  half at 24 MHz - and 14 on the second, 836 ksps at 12 MHz. The
  sample-time codes are the same eight, the CH32V006's in half cycles
  (3.5 .. 239.5, two columns by ADC_LP) and the CH32V003's whole (3 ..
  241).
- **VREFINT is 1.2 V on channel 8** (DS table 3-5 of either part).
  Channel 9 is the OPA's output on the CH32V006 and **Vcal on the
  CH32V003**: 2/4 or 3/4 of AVDD by CALVOL, a second known source
  (measured: 511 and 767 counts of 1024). The pads are PA2 (0), PA1
  (1), PC4 (2), PD2 (3), PD3 (4), PD5 (5), PD6 (6), PD4 (7) on both.
- **The CH32V003's injection group has no TIM3 to trigger it** (table
  9-4's codes 100 and 101 blank): TIM2's CC3 and CC4 stand in, and the
  TIM3 codes are refused there. Nor has its CTLR2 the OPA-as-trigger
  bits (TGREGU/TGINJE): refused.
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
- **THE STALL** (measured on both parts): with a rule group paced by a
  hardware trigger and served by the DMA, and the CPU accessing the
  peripheral buses meanwhile, the converter has been seen to stop converting -
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
continuous, scan, JAUTO, the low-power bit and the input buffer on the
CH32V006, the Vcal fraction on the CH32V003, DMA, the discontinuous
lengths; refused by `adc_config_valid()` - which refuses the other
part's fields - and for an ADCCLK above the part's bound; on the
CH32V003 it runs `calibrate()`, false if the calibration never
completed), `power()`, `recover()` (the ADON cycle the stall
wants), `sample_time(ch, AdcSampleTime)` (each part's own enum of the
eight codes; `adc_sample_shortest` and `adc_sample_longest` spell the
two ends on both), `sequence(order, n)` and
`select()` (a one-conversion sequence: util/analog_sampler.hpp's
verb, with `start()`, `selected()` and `input_code()`), `read()` /
`read_settled()` / `result_counts()`, `supply_mv(vrefint_counts)` and
`millivolts()` over util/analog.hpp, `trigger(AdcTrigger)` and
`injected_trigger(AdcInjectedTrigger)` (tables 9-3 and 9-4, each
answering false for a source the part has not), `trigger_delay()`
(DLYR, the CH32V003's), `injected_sequence()`, `injected_offset()`,
`injected_start()`, `injected_result(slot)` signed,
`watchdog0(AdcWatchdogConfig)` (the thresholds, the scope, the
interrupt, the reset - refused on the CH32V003), `watchdog(n, low,
high, reset)` for 1 and 2, `watchdog_scan()`, the results and their
clearing (the CH32V006's; false and nothing written on the CH32V003),
the flags, the three interrupt enables and `isr()`. `AnalogIn<Pin>`
names a pad and derives its channel (a pad that is no input is
refused); `AdcInput::vrefint` is on both parts, `AdcInput::opa` the
CH32V006's channel 9 and `AdcInput::vcal` the CH32V003's; `Ref::vdd`,
`adc_steps` (4096 or 1024) and `adc_vrefint_mv` are this target's
util/analog.hpp vocabulary. The arithmetic: `adc_prescaler_for(hclk,
max_hz)`, `adc_prescaler_divider(code)`, `adc_conversion_half_cycles(t,
low_power)` with the part's own tail.

## How to use it

```cpp
using Vin = brio::AnalogIn<brio::Pin<'D', 2>>;      // channel 3
brio::Adc::init(clock, {.prescaler_code = brio::adc_prescaler_for(48'000'000, 6'000'000)->code});
brio::Adc::sample_time_all(brio::adc_sample_longest);
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

The reference suite is `test_ch32_adc` (23 verdicts in `z` on the
CH32V006K8U6, one letter ending in a real reset, two probe letters
outside `z`; 23 in `z` on the CH32V003F4P6 as three group images), at
48 MHz, nothing wired. Every count the suite judges is a fraction of
the part's full scale, so one source judges both.

- **The supply from the reference**: 1495..1504 counts of VREFINT at
  the eight sample times on the CH32V006, 3275..3294 mV - the probe's
  3V3; 375..376 counts of 1024 on the CH32V003 board, 3268..3277 mV,
  inside the bracket its PVD draws (above 3.15 V, below 3.5 V). The
  PVD is the one independent witness of a wireless board's supply,
  and the verdict judges against it - a cable's drop has moved that
  board's supply by a quarter of a volt between sessions.
- **The rate**: 256 continuous conversions through DMA channel 1 in
  516197 cycles at 6 MHz and 239.5 (516096 computed), 32890 at 6 MHz
  and 3.5 (32768), 21112 at 24 MHz and 28.5 (20992), 8337 at 24 MHz
  and 3.5 (8192) on the CH32V006: tCONV to the cycle, 1.47 Msps at the
  top. On the CH32V003: 516387 at 6 MHz and 241 (516096), 29091 at
  6 MHz and 3 (28672), 42305 at 12 MHz and 30 (41984), 14649 at 12 MHz
  and 3 (14336) - the tail of 11 cycles to the cycle, 838 ksps.
- **The calibration** (CH32V003): completes on its own at init and on
  demand; VREFINT reads the same with the calibration register reset
  and calibrated again (376 and 376).
- **The triggers**: TIM1 and TIM2's TRGO, CC1 and CC2 at 10 kHz each
  deliver 1000 conversions in 100.0 ms through the DMA on both parts;
  TIM3's CC1 match paces the injection group at 10 kHz (1000 JEOCs in
  100 ms) on the CH32V006 - the streamlined timer's purpose - and
  TIM2's CC3 does the same on the CH32V003, whose TIM3 codes are
  refused.
- **The injection group**: four slots of VREFINT with offsets 0, 100,
  VREFINT and 4095 read 1497, 1402, 4 and -2597 on the CH32V006 -
  signed; 375, 350, 0 and -648 for offsets 0, 25, 375 and 1023 on the
  CH32V003; under JAUTO the injected conversion follows the rule one
  by itself; left alignment puts the datum in the top bits of RDATAR
  (15..4, or 15..6 at 10 bits).
- **The watchdogs**: watchdog 0 around VREFINT quiet inside its window
  and raising AWD with its interrupt on every conversion outside, on
  both parts; on the CH32V006 the scan's three ranks each guarded by
  their own watchdog, a fault at the rank whose window excludes it
  and nowhere else, and armed with AWD0_RST_EN and a window above the
  reading, the next conversion REBOOTS THE BOARD with ADCRSTF alone;
  on the CH32V003 the verbs of watchdogs 1 and 2 answer false and a
  reset_on_fault config is refused.
- **The pads through their pulls**: PD2 and PC4 read 4095 pulled up
  and 3..4 pulled down (1023 and 0 of 1024 on the CH32V003).
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

- The CH32V003's trigger delay (DLYR): only an external trigger is
  delayed, and the trigger pads are the wire above.
- The CH32V003's stall hunt (letter `y`) as numbers: the letter builds
  and runs there, its hundreds of runs not yet counted on that board;
  the six-trigger letter met one stall in two runs and recovered it.
- The stall's cause: a scope on the bus arbitration is not on this
  desk; what the suite offers is the recovery and the count. A rule of
  thumb until then: a stream that must not die watches its own
  progress and calls `recover()`.
- A known external voltage on a pad: the pulls are two levels, a
  divider on a jumper would be a third.
