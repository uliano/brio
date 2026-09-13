# ADC (STM32F4)

Documents of record: RM0090 Rev 22 ch. 13, RM0390 Rev 6 ch. 13 and RM0383
Rev 4 ch. 11 - the same converter under three chapter numbers, differing
in how many of it a part has, which channel the temperature sensor is on
and how many of the trigger codes carry a signal. The electrical numbers
- fADC's ceiling, the sampling and stabilization times, and the factory
calibration values with their addresses - are the datasheets': DS10693
Rev 11, the F429 datasheet (DocID024030 Rev 10) and DS10314 Rev 8, which
agree on all of them. Errata: ES0206 Rev 24 2.5.1 and 2.2.8, ES0298 Rev 8
2.6.1 and 2.2.8, ES0287 Rev 6 2.4.1 and 2.2.8 - two items under three
numbers, on every silicon revision of all three parts; both are quoted
below. Driver: `stm32f4/adc.hpp`, with the per-part facts in
`stm32f4/device_tables.hpp`. Bench suite: `test_stm32f4_analog` (one
suite with the DAC, because the DAC's only route to this converter is a
pad they share). Family fixture `test/family_stm32f4/analog.cpp` plus
five negatives under `brio check stm32f4`.

## What the silicon does

**Up to three converters, one clock, one vector.** A part has ADC1 alone
or ADC1, ADC2 and ADC3 (the header says which, and the reserve counts
them); they share the ADCCLK prescaler, the two internal-source switches,
the multi-ADC machinery, one reset line and one NVIC line. Only ADC1 is
the master: the temperature sensor, VREFINT and the battery channel are
its alone (13.3.4's note), and every multi-ADC trigger comes from its
multiplexer.

**ADCCLK is PCLK2 divided by 2, 4, 6 or 8 and by nothing else** (13.3.3):
four codes, no bypass, no asynchronous root - so the fastest possible
converter clock is half the bus, and where the bus is fast that is not a
choice. fADC's ceiling is 36 MHz at 2.4..3.6 V and 18 MHz below (the
datasheets' ADC characteristics table), and its FLOOR is 0.6 MHz. At
90 MHz on APB2 that leaves PCLK2/4 = 22.5 MHz as the fastest legal
division, which is what `init()` picks and what every number below was
measured at.

**There is no calibration and no regulator.** Unlike the STM32G0's
converter this one has no ADCAL and no ADVREGEN: ADON powers it and
tSTAB (2 us typical, 3 us maximum) is the whole bring-up. What ST
measured on the die is elsewhere and is a MEASUREMENT rather than a trim -
VREFINT_CAL at 0x1FFF7A2A, TS_CAL1 at 0x1FFF7A2C and TS_CAL2 at
0x1FFF7A2E, all ADC results at VDDA = 3.3 V, the temperature ones at 30
and 110 degrees (the F4's points are at 3.3 V where the STM32G0's are at
3.0). Nothing is written back anywhere.

**Two groups, and the injected one preempts.** A REGULAR sequence is up
to sixteen conversions into ONE data register - so a sequence longer than
one needs the DMA or a reader fast enough - and an INJECTED sequence is
up to four into four registers of their own, each with a subtractable
12-bit offset and therefore a SIGN. An injected trigger interrupts a
regular conversion, the injected sequence runs, and the regular sequence
resumes where it was (13.3.10). That second group has no counterpart on
the two earlier targets of this project.

**JSQR is filled from the TAIL.** 13.13.12's note: with JL = 0 the
converter runs JSQ4 alone, with JL = 1 it runs JSQ3 then JSQ4, and only a
full four starts at JSQ1. A caller who wrote its list at JSQ1 and set JL
would convert channels it never named. The driver does the placement.

**The status register is rc_w0.** ADC_SR's six flags are cleared by
writing ZERO (13.13.1), the opposite of every other flag register in this
stratum - so a clear stores the COMPLEMENT of the mask (ones everywhere
else, which keep) and never a read-modify-write, which would take down a
flag that rose between the read and the write.

**EOC is cleared by reading the data, and STRT by nobody.** 13.13.1: EOC
goes down on a read of ADC_DR. STRT - "a regular conversion has started" -
goes down on a write and on nothing else, and this converter has NO BUSY
BIT at all, so "started and not yet finished" is the only answer to "is a
conversion in flight". The driver therefore clears STRT inside
`result()`, in the same verb that clears EOC by reading; without that the
bit stands for the rest of the program and every sequence verb refuses
for ever after the first conversion - IN SILENCE, because the selection
verb the sampler's contract asks for returns void.

**Overrun detection is a CHOICE.** OVR exists only with DMA = 1 or
EOCS = 1 (13.13.3). With EOC at the end of the sequence and no DMA there
is none at all, which 13.8.3 offers deliberately: a converter running
continuously for an analog watchdog need not be read and will not be told
it lost anything.

**The multi-ADC modes are the master's**, selected in the common
register, triggered from ADC1's own multiplexer with the slaves' triggers
off, and delivered through the common data register (13.9).

**The reference is a pad and nothing else.** This family has no reference
buffer and no reference selector: VREF+ is an input, tied to VDDA on
every board this stratum has met. So `Ref` has one enumerator and
`ref_mv()` takes the board's millivolts - and `vdda_mv()` measures the
rail through VREFINT rather than believing them.

### The errata, and how they are answered

- **ADC sequencer modification during conversion** (ES0206 2.5.1,
  ES0298 2.6.1, ES0287 2.4.1, every revision): with a SOFTWARE start, a
  write to ADC_SQRx or ADC_JSQR during a conversion resets that
  conversion and the converter does not restart by itself. The hardware
  trigger is spared. ANSWERED STRUCTURALLY: every sequence verb refuses
  while a conversion is in flight, and a deliberately named unchecked
  verb exists so a suite can stage the item. Both halves are measured
  below.
- **Internal noise impacting the ADC accuracy** (2.2.8 in all three,
  every revision): noise on VDD propagates inside whatever the power
  mode. The workaround is a SYSTEM one - the flash accelerator's prefetch
  OFF with both caches on, plus averaging - so it is not the driver's to
  apply: `stm32f4/flash.hpp` owns those bits and the application decides.
  What the choice is worth on this silicon is measured below.

## Types and verbs

### The reserve's facts (`stm32f4/device_tables.hpp`)

| Fact | Where it comes from |
|------|---------------------|
| which converters exist, and how many | `ADC1_BASE`, `ADC2_BASE`, `ADC3_BASE` |
| the common block's base | `ADC123_COMMON_BASE` or `ADC1_COMMON_BASE`, whichever the pack spells |
| the per-instance clock enables, the one reset bit, the one vector | the header's RCC masks and `ADC_IRQn` |
| the internal channels' NUMBERS, the battery divider, whether the sensor shares the battery's channel | the REFERENCE MANUAL, keyed on the device-select define: the F405 class puts the sensor on channel 16 and divides VBAT by two, the F42x/F43x, F446 and F411 put it on 18 beside VBAT and divide by four. A class whose manual is not on the desk gets nothing and its internal channels are refused |
| which timers exist, and therefore which trigger codes carry a signal | the timers' base-address macros |
| where each converter's DMA request sits in the fabric | RM0090 table 44 / RM0390 table 29 / RM0383 table 28, keyed on the part class in the same `DmaPlacement` shape the serial instances use: ADC1 on DMA2's stream 0 or 4, channel 0; ADC2 on stream 2 or 3, channel 1; ADC3 on stream 0 or 1, channel 2 |

### Vocabulary

- **`AdcRes`** - `bits12` (the default), `bits10`, `bits8`, `bits6`. The
  data register is always sixteen bits; what changes is how many are
  significant and how long the SAR takes.
  `adc_sample_steps()` is util/analog.hpp's `steps` for each.
- **`AdcSampleTime`** - `cycles3` (the reset value), `cycles15`,
  `cycles28`, `cycles56`, `cycles84`, `cycles112`, `cycles144`,
  `cycles480`, per channel. `adc_conversion_cycles(res, smp)` is
  13.5's own arithmetic: the sampling time plus one cycle per bit.
- **`AdcPrescaler`** - `div2`, `div4`, `div6`, `div8`.
  `adc_prescaler_for(pclk2, ceiling)` picks the smallest that fits;
  `adc_max_hz`, `adc_max_hz_low_supply` and `adc_min_hz` are the
  datasheets' bounds.
- **`AdcTrigger`** and **`AdcInjectedTrigger`** - tables 87 and 88's
  sixteen codes each, by name; `adc_trigger_valid()` refuses a code whose
  timer this part has not got, and `adc_regular_exti_line` /
  `adc_injected_exti_line` name the two pad lines (11 and 15) so no
  application spells the number.
- **`AdcEdge`** - `none`, `rising`, `falling`, `both`.
- **`AdcInput`** - `temperature`, `vrefint`, `vbat`, as TAGS and not as
  channel numbers, because the sensor's channel differs across the
  family; `adc_input_channel()` is the reserve's answer and
  `adc_input_valid()` says whether there is one.
- **`AdcMulti`** and **`AdcMultiDma`** - the dual and triple modes and
  the three multi-ADC transfer modes; `adc_multi_valid()` refuses a
  Reserved code and a mode this part has too few converters for.
- **`AdcFlag`** - the six status bits as one set of constants, used as a
  flag mask and translated into CR1's scattered enables by the interrupt
  verb.
- **`Ref`** / **`ref_mv()`** - one enumerator, the pad, and the board's
  millivolts as the caller's argument.

### The input types

- **`AnalogIn<Pin, channel>`** - a pad handed to a channel. The channel
  DEFAULTS to the ADC1/ADC2 map every datasheet of this family gives -
  PA0..PA7 are IN0..IN7, PB0 and PB1 are IN8 and IN9, PC0..PC5 are
  IN10..IN15 - so `AnalogIn<Pin<'A', 4>>` is channel 4 with nothing to
  look up, and a pad outside that map states its channel.
  `adc3_channel_of()` is the third converter's own map, whose IN4..IN9,
  IN14 and IN15 live on port F.
- **`AdcFactory`** - a read-only view of the three factory measurements,
  their conditions, and `plausible()`.

### The block: `AdcCommon`

A monostate, and a type of its own rather than a corner of `Adc<n>`
because everything in it belongs to all of them. Verbs: `reset()` (the
ONE line, which pulses every converter), `prescaler()` and `adc_hz()`,
`internal_sources()` (TSVREFE, which wakes the sensor and VREFINT
together), `vbat()`, `sensor_shares_vbat()` and `vbat_divider()`,
`multi()` and `multi_dma()` and `interleave_delay()`, `status()` (the
common status register, read-only), `data()` / `data_low()` /
`data_high()` / `data_address()`, `release()`.

### The converter: `Adc<n>`

- **Bring-up**: `bus_clock()`, `power_on(clock)` (ADON and tSTAB),
  `power_off()`, `configure(AdcConfig)`, `init(clock, config, ceiling)`
  (the bus clock, the block's prescaler for this rate, the config, ADON -
  and refusing when no prescaler keeps fADC legal), `release()`. The
  RESET is deliberately not part of `init()`: one line resets every
  converter together.
- **`AdcConfig`**: resolution, alignment, scan, continuous, EOC per
  conversion or per sequence, both discontinuous modes with the regular
  one's count, auto-injection, the DMA pair, and the two triggers with
  their edges. `adc_config_valid()` refuses exactly what the chapter
  refuses - both discontinuous modes at once, auto-injection with either
  of them or with an injected external trigger, a discontinuous count
  past three bits, a trigger whose timer is absent, DDS without DMA.
- **Sampling times**: `sample_time(channel, t)`, `sample_time_all(t)`,
  `conversion_cycles(channel)`.
- **The regular sequence**: `regular_sequence(order, count)` (up to
  sixteen, refused in flight), `regular_sequence_unchecked()` (the
  erratum's staging ground), `select()` / `select_sync()` /
  `select_channel()` (a sequence of one - `select()` is void because
  util/analog_sampler.hpp's contract asks for that), `sequence_length()`,
  `sequence_channel(slot)`, `selected()`.
- **The injected sequence**: `injected_sequence()` (the tail placement
  hidden), `injected_length()`, `injected_slot_channel(slot)` (the RAW
  slot, so a caller can see the placement), `injected_offset()`,
  `injected_result(k)` (signed), `start_injected()`, `injected_ready()`,
  `read_injected()`.
- **Conversions**: `start()` (SWSTART, void for the sampler),
  `converting()` and `converting_injected()`, `ready()`, `started()`,
  `overrun()`, `result()` (which acknowledges), `read()`,
  `read_settled()`, `stop()`.
- **The arithmetic**: `result_steps()`, `vdda_mv(vrefint_data)`,
  `temperature_centi_c(ts_data, vdda)` (the two-point formula, with the
  rescaling from ST's 3.3 V that the chapter's own text leaves out), and
  `temperature_centi_c_typical(ts_mv)` (the chapter's V25-and-slope
  answer, kept beside it).
- **The analog watchdog**: `watchdog(low, high, regular, injected,
  single, channel)` - table 85's rows as arguments - `watchdog_off()`,
  `watchdog_thresholds()` (the live half), and the three readbacks.
- **Flags and interrupts**: `flags()`, `flag()`, `clear_flags()` (the
  rc_w0 store), `interrupts(flag_mask, on)` (named by the FLAG, since the
  enables sit at other positions), `armed()`, and the ISR body `isr()`,
  which clears exactly the armed flags that are up and DOES NOT read the
  data.
- **The DMA**: `data_address()`, `dma_placements()` (the cells this
  converter's request is wired to) and `engine_placed<E>()` - the same
  question `uart_engine_placed()` asks for a serial instance, offered
  here as a check an application static_asserts rather than as an engine
  SLOT, because a converter has no transport to hold one: its DMA is a
  caller-side composition, a scan into a buffer the caller owns.

## How to use it

One conversion of a pad:

```cpp
using Adc1 = brio::Adc<1>;
using In4 = brio::AnalogIn<brio::Pin<'A', 4>>;   // channel 4 by the datasheet's map

In4::claim();                                    // analog mode, input buffer off
brio::AdcConfig cfg{};                           // 12 bits, one conversion when asked
Adc1::init(clock, cfg);                          // the prescaler comes from the clock
Adc1::sample_time(In4::channel, brio::AdcSampleTime::cycles480);
Adc1::select(In4{});
const uint16_t counts = Adc1::read();
const uint16_t mv = brio::adc_mv(counts, Adc1::result_steps(), 3300);
```

The supply and the junction temperature, with no meter:

```cpp
brio::AdcCommon::internal_sources(true);         // TSVREFE wakes both
brio::delay_us(clock, 20);                       // their start-up time
Adc1::sample_time_all(brio::AdcSampleTime::cycles480);
Adc1::select(brio::AdcInput::vrefint);
const uint16_t vdda = Adc1::vdda_mv(Adc1::read_settled(4));
Adc1::select(brio::AdcInput::temperature);
const int32_t centi_c = Adc1::temperature_centi_c(Adc1::read_settled(4), vdda);
```

A window on one channel, and the flag it raises:

```cpp
Adc1::watchdog(1000, 3000, /*regular=*/true, /*injected=*/false,
               /*single=*/true, In4::channel);
Adc1::interrupts(brio::AdcFlag::watchdog, true);
brio::Nvic::enable(Adc1::irq());
// ... and in the handler the app binds:
extern "C" void ADC_IRQHandler() { const uint32_t hit = Adc1::isr(); /* ... */ }
```

An injected group that preempts a running regular one:

```cpp
static const uint8_t injected[] = {5};
Adc1::injected_sequence(injected, 1);            // lands in JSQ4, as JL = 0 wants
Adc1::injected_offset(1, 500);                   // subtracted, and it may go negative
int16_t value = 0;
Adc1::read_injected(value);
```

Two converters, one trigger:

```cpp
Adc1::select(In4{});
brio::Adc<2>::select_channel(In5::channel);
brio::AdcCommon::multi(brio::AdcMulti::dual_regular);
Adc1::start();                                   // ADC2 starts with it
```

A hardware trigger, which on this family is a timer's output or a pad's
EXTI line:

```cpp
brio::AdcConfig cfg{};
cfg.trigger = brio::AdcTrigger::exti11;
cfg.trigger_edge = brio::AdcEdge::rising;
Adc1::configure(cfg);
// the line is the EXTI chapter's: a pad, a rising sense, and no mask needed
```

A sequence of four channels into memory, which is what the DMA is for:

```cpp
using AdcStream = brio::DmaRxEngine<2, 0, 0, uint16_t>;   // ADC1's own cell
static_assert(Adc1::engine_placed<AdcStream>());
static const uint8_t seq[4] = {4, 5, 17, 18};
static uint16_t block[4];

brio::AdcConfig cfg{};
cfg.scan = true;                     // one conversion per rank
cfg.eoc_per_conversion = false;      // EOC at the end of the sequence
cfg.dma = true;
Adc1::init(clock, cfg);
Adc1::regular_sequence(seq, 4);
brio::Dma<2>::init();
AdcStream::arm(Adc1::data_address());
AdcStream::start(block, 4);
Adc1::start();                       // one trigger, four conversions, four halfwords
```

The sampler over this converter (util/analog_sampler.hpp):

```cpp
using Sampler = brio::AnalogSampler<Adc1, Platform, Subs,
                                    brio::AdcInput::vrefint,
                                    brio::AdcInput::temperature, In4{}>;
Adc1::interrupts(brio::AdcFlag::converted, true);
brio::Nvic::enable(Adc1::irq());
Sampler::start_every(2);
extern "C" void ADC_IRQHandler() {
    if ((Adc1::isr() & brio::AdcFlag::converted) != 0u) {
        brio::post<Sampler>(brio::Sampled{Adc1::result(), Adc1::selected()});
    }
}
```

## Bench findings

`test_stm32f4_analog`, on an STM32F446 at 180 MHz with PCLK2 at 90 MHz -
so ADCCLK at PCLK2/4 = 22.5 MHz throughout - reading a DAC output through
the pad the two converters share.

**The clock.** `init()` picks PCLK2/4: half the bus is 45 MHz, past the
36 MHz ceiling, and the next division is the first that fits. fADC
22.5 MHz.

**The conversion time is the chapter's, to under one per cent, and it was
measured DIFFERENTIALLY** - the same polling loop at two sampling times,
so every cycle of overhead cancels. 477 ADCCLK cycles of extra sampling
(480 against 3) cost 3813, 3822, 3817 and 3809 HCLK cycles at 12, 10, 8
and 6 bits against 3816 predicted. Six SAR cycles (12 bits against 6 at
the same sampling time) cost 44 to 45 HCLK against 48 predicted. One whole
conversion at 3 cycles and 12 bits takes 265 HCLK cycles measured, of
which 120 are the chapter's fifteen ADCCLK cycles and 145 are the polling
loop around it - which is why the difference and not the absolute is what
the verdict rests on.

**VDDA and the temperature.** VREFINT reads 1481..1485 against a
calibration of 1494, giving VDDA 3320..3329 mV on the STM32F446 (1499
against 1507 on the STM32F411, 3318 mV; and 1689 against 1498 on the
STM32F429I-DISC1, whose rail is 2927 mV - the count is where the rail is,
which is why `vdda_mv()` measures it). The sensor reads
953..964, which is 776..780 mV, and gives 37 to 39 C from the two
calibration points against 31 to 33 C from the datasheet's typical slope
and V25 - a spread of six degrees between two formulas for one die, well
inside the up-to-45 degree part-to-part offset the chapter warns about,
and a reason to prefer the calibrated one.

**The battery, and which switch wins the channel it shares with the
sensor.** VBAT/4 reads 1024..1025, giving VBAT 3328..3332 mV against
VDDA's 3324. With TSVREFE and VBATE both set the shared channel reads the
BATTERY's number and not the sensor's on the STM32F446 and the STM32F429,
exactly as 13.11 says - and the two sources are only 58 to 69 LSB apart
on a 3.3 V board, which makes this a close-run reading rather than an
obvious one. THE STM32F411 DOES NOT AGREE: with both switches set its
shared channel reads 978 between the battery's 1015 and the sensor's 957,
neither source alone, as if both were on the pad at once (measured
repeatedly). A program that wants either sets one switch and clears the
other; the suite judges only that the reading is not the sensor's alone.

**The converter is quiet.** 256 conversions of VREFINT at 12 bits spread
3 to 4 LSB with the ART's prefetch on and 2 LSB with it off, at the same
mean (1482 both ways). So the errata item's workaround costs the accuracy
nothing here and buys about one LSB of spread - which is the size of the
effect on this board, at this rate, on this source.

**The DAC read back through the pad they share** (letters c, d and e are
as much the DAC chapter's; they are repeated in
[dac.md](dac.md)'s findings): the round trip is monotonic over the whole
range and stays within 38 LSB of the identity between codes 256 and 3840,
which is a little over one per cent of full scale for the DAC's error,
the ADC's and their shared reference together (38 to 47 LSB run to run).

**The settling of a step, measured by sampling at increasing delays.**
After a 0 -> 4000 code step, a 15-cycle conversion started with no delay
reads 432..475, at +1 us about 2200, at +2 us about 3440, at +5 us 4031,
and 4040..4041 from +10 us - against a settled 4038..4041. So the pad is within one per cent
of its final value between 2 and 5 us after the write, and a program that
converts immediately after a step measures the ramp.

**The analog watchdog.** A window of +/- 600 LSB around a DAC-held
midpoint raises AWD on a step below it and on a step above it and stays
down inside. AND THE THRESHOLDS ARE COMPARED BEFORE THE ALIGNMENT
(13.3.8): with ALIGN set, the same window - written as 12-bit numbers -
still holds a reading whose DATA register shows the code four bits up,
and still fires on a step out of it.

**The injected group preempts, and resumes.** With a continuous regular
run on the pad at code 1000 (EOCS = 0, so no overrun stops it) and an
injected conversion of the other pad at code 3000: the regular register
reads 994..1000, the injected one 3136..3139, and the regular run reads
the same 990..994 afterwards - one converter, two channels, and the interruption costs the
regular sequence its place and nothing else. With JAUTO instead, JEOC
arrives 193 polls after one regular conversion with no injected trigger
of any kind. The injected offset is subtracted (a raw 3137 reads 2637 at
JOFR1 = 500) and CAN go negative (-863 at JOFR1 = 4000), which is the only
signed datapath this converter has.

**Dual regular simultaneous mode, and what the common data register is
for.** One SWSTART on ADC1 converts both: ADC1 reads 1194 on the pad at
code 1200 and ADC2 reads 3138 on the pad at code 3000, and the common
status register shows EOC on both. THE COMMON DATA REGISTER STAYS AT ZERO
while DMA[1:0] is 00, however many pairs convert, and fills with the pair
- the master in the low half - as soon as a multi-ADC DMA mode is
selected, with no stream armed and nothing serving the requests. So
ADC_CDR is the DMA's register and not a second data register, which
13.13.17 does not say. AND THE SLAVE'S OWN EOC DOES NOT STAND after such
a round (its SR reads STRT alone), so a reconfiguration has to clear its
STRT by hand.

**Two converters on ONE pad**, which 13.9.2's note tells an application
not to do ("no overlapping sampling times ... when converting the same
channel"): they read within 1 LSB of each other and within 6 of the
single-converter answer. On a source as stiff as a buffered
DAC output the caution costs nothing measurable; it is about a source
with impedance, which this is not.

**Overrun, both ways.** With EOC at every conversion, continuous, and
nothing read for 200 us, OVR is up; clearing it and triggering again is
the whole recovery and the flag stays down. With EOC at the end of the
sequence and no DMA, the same 200 us raises nothing at all - 13.8.3's
"conversions without DMA and without overrun detection" is a real mode
and not a warning.

**The sequencer erratum, both halves.** STRT rises one poll after
SWSTART, and the driver refuses a sequence write there. Written anyway
through the unchecked verb, INSIDE a 22 us conversion: EOC never arrives
- 200000 polls of it - and one more SWSTART converts the new sequence.
With a HARDWARE trigger (a pad edge on line 11) the conversion the
rewrite landed in is lost in the same way and THE NEXT EDGE converts the
new sequence with no software in between, which is exactly the sparing
the item's own text claims.

**The external trigger path, with no timer and no wire.** A pad driven
high feeds its own EXTI line and starts a conversion on line 11, and an
injected one on line 15. NEITHER EXTI MASK GATES IT: the edge converts
with IMR alone, with EMR alone, and with neither - the converter takes
the edge detector's output. AND THE EXTI's SOFTWARE TRIGGER DOES NOT
REACH IT: SWIER raises the line's own pending bit and no conversion
follows, under either mask, because that write is injected past the
detector. SWSTART is independent of all of it: a converter armed for a
hardware trigger still converts when the software asks.

**A four-rank sequence into memory, and the counter-experiment.** One
SWSTART, four conversions of four different channels (the two DAC pads,
VREFINT and the sensor) and four halfwords in a buffer, in the sequence's
own order, with no overrun and no stream error - the stream's own
completion is what says when. The same four ranks with no stream and EOC
at every conversion overruns and leaves one datum in the register: four
conversions into one register is what the DMA is for, stated as two
measurements rather than as advice.

**DDS clear means the converter STOPS ASKING**, which 13.8.1 says and
this measures: after the first block, a fresh stream over an untouched
converter takes ZERO halfwords - the conversions still run (SR reads
STRT and EOC) and no request is made. Neither cycling CR2's DMA bit nor
re-initializing both ends brought a second block back in this suite's
hands; a stream that keeps running is what the chapter really points at
and it is in the gap list with that reason.

**AnalogSampler runs unchanged on this architecture.** Sixty samples
across three inputs (VREFINT, the sensor, the DAC's pad) at a 2 ms
software pace: twenty each, no queue overflow, and not one sample
mislabelled - on a converter with a hardware sequencer and a DMA the
sampler uses neither of. `selected()` is the driver's own memory of the
last selection, which is exact because the sampler selects one channel at
a time.

**THE ACKNOWLEDGEMENT IS LOAD-BEARING, and it is a trap worth naming.**
STRT is set by the silicon and cleared by nothing but a write, so a
driver that leaves it standing answers "a conversion is in flight" for
ever after the first one and refuses every later selection - IN SILENCE,
because `select()` is void by the sampler's contract. Reading the datum
takes both flags down, and letter a asserts that a selection right after
a completed conversion is accepted.

## Not covered yet

Driver gaps:

- **A SECOND DMA block from one converter.** The first is measured; with
  DDS clear the converter stops asking, and making it ask again was not
  achieved here - not by cycling CR2's DMA bit, not by re-initializing
  both ends. What the chapter points at for a stream that keeps running
  is DDS with a CIRCULAR stream (and the double buffer beside it), and
  the DMA chapter's engines run a block and end, so there is nothing to
  compose with yet. The converter's own side of it is one bit and is
  here.
- **The multi-ADC transfer modes.** All three are selectable and one of
  them was proved to be what loads ADC_CDR at all; none has moved a byte
  through a stream, because their point is a PAIR per request and that is
  the same circular arrangement the item above waits for.
- **A `rebase()` for a dynamic clock.** The ADC prescaler divides PCLK2,
  so a clock that changes rate must recompute it - and this family has no
  `DynamicClock` yet (it is the power chapter's). `init()`
  static_asserts `clock_follows<>`, so the day one arrives the compiler
  will ask for this verb rather than let the division go stale.
- **Interleaved and alternate-trigger multi-ADC modes.** Selectable and
  refused where the part has too few converters; their POINT is a DMA
  stream taking a pair per request, so they are born with the DMA
  chapter. The interleave delay is written and read back and its effect
  on two conversions has not been timed.
- **Triple mode.** The part on the bench has three converters and the
  suite drives two: the third needs its own pad, and the two the DAC owns
  are taken.
- **The 6-bit left-aligned special case** (13.4's byte alignment): the
  alignment bit is written and the 12-bit case measured, the 6-bit one
  only compiled.
- **The discontinuous modes as SEQUENCES.** The bits are configured and
  refused where the chapter refuses them, and nothing here splits a
  sequence into subgroups and counts the triggers it then takes: it wants
  a repeating trigger, which is the timer chapter's.
- **`AdcInput` on the F401, F410, F412, F413/F423 and F469/F479
  classes**: the reserve does not know which channel the sensor is on
  there and the driver refuses all three internal sources rather than
  guess. RM0368, RM0401, RM0402, RM0430 and RM0386 would each add one row
  to `stm32f4/device_tables.hpp`.

Implemented, not bench-verified:

- **ADC3, and the port F pads its own map reaches**: compiled on every
  header and exercised through its verbs; the part on the bench bonds no
  port F, so `adc3_channel_of()`'s numbers are the datasheets' and not
  measured.
- **The low-supply ceiling** (18 MHz below 2.4 V): `init()` takes it as
  an argument and the chooser is tested against it, but every board here
  runs at 3.3 V.
- **The falling and both-edge trigger senses**: the rising one is
  measured on a pad; the other two are one field's value away and need a
  falling edge held the same way.
- **The timer trigger codes.** Fifteen of the sixteen in each table are a
  timer's, refused where the timer is absent and otherwise untried: they
  need the timer chapter.
- **A watchdog INTERRUPT.** The flag is raised, polled and cleared here;
  the vector is shared by every converter and the suite binds it for the
  sampler alone.
