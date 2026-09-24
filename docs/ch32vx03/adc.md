# ADC (CH32V203)

Two 12-bit successive-approximation converters over sixteen pads and two
internal sources, a regular group of up to sixteen conversions and an
injected group of four that preempts it, scan, continuous and
discontinuous modes, an analog watchdog, external triggers from the
timers and from an EXTI line, one DMA request, a calibration the chapter
asks for at every power-up, and the DUAL modes in which one converter
leads and the other follows - the STM32F1's ADC under WCH's register
names, with an input buffer and a programmable gain of WCH's own in
front of it. Documents of record: the CH32F/V20x_V30x_V31x reference
manual V2.3 (12.2.2 for the power-up, the calibration, the sampling
times and the alignments, 12.2.3 with tables 12-1 and 12-2 for the
triggers, 12.2.4 with table 12-3 for the conversion modes, 12.2.5 with
table 12-4 for the watchdog, 12.2.6 for the temperature sensor, 12.2.7
for the dual modes, 12.3 for the registers, 3.4.2 for ADCPRE, 10.2.11.8
for the trigger remaps that belong to other classes, 11.2.3's table 11-5
for the DMA request, and 31 for the electronic signature that carries no
calibration word) and the CH32V203 datasheet V2.8 (table 2-1 for how
many converters and how many channels a part has, 3.2 for the pad map,
table 4-19 for the weak pull, table 4-26 for VREFINT, table 4-27 for the
converter's own ratings, table 4-28 for the source impedance each
sampling time settles, table 4-30 for the temperature sensor). Ours is
the CH32V20x_D6 device class for every part up to the CH32V203C8 and
CH32V20x_D8 for the CH32V203RB. Driver:
[brio/ch32vx03/adc.hpp](../../brio/ch32vx03/adc.hpp), with the two block
engines of [brio/ch32vx03/dma.hpp](../../brio/ch32vx03/dma.hpp) behind
its stream. Reference suite: `test_vx03_adc`.

## What the silicon does

### How many converters, and how many channels

Datasheet table 2-1 counts them together - "9@2", "10@2", "16@1" - and
the two numbers are not independent: every part up to the CH32V203C8 has
TWO converters with nine or ten of the sixteen channels bonded, and the
128 KB part has ONE with all sixteen. So `Adc<2>` does not exist there,
and neither does a dual mode.

The channel map is the family's and the same on every part: ADC_IN0 to
ADC_IN7 are PA0 to PA7, 8 and 9 are PB0 and PB1, 10 to 15 are PC0 to
PC5. What a PACKAGE decides is which of those pads it brings out, and
that is `Pin`'s own refusal - a pad this part does not bond does not
compile - so the driver states the map once and never repeats the part
table. Channels 16 and 17 are internal and reach no pad at all.

### The clock: the one place a legal tree leaves a peripheral out of specification

ADCCLK is PCLK2 divided by 2, 4, 6 or 8 (RCC_CFGR0's ADCPRE) and nothing
else, the converter is rated at 14 MHz, and this family's PCLK2 is HCLK
undivided. Above 112 MHz of HCLK there is therefore NO code that keeps
the converter in range: at the 144 MHz the part is rated for, the
slowest divider still gives 18 MHz.

The clock task programs ADCPRE and publishes both `Clock::adc_hz` and
`Clock::adc_in_spec` ([clock.md](clock.md)); the converter's `init()`
static_asserts the second. A program that wants the ADC therefore picks
a tree that keeps it - the PLL on the HSI at 96 MHz gives 12 MHz, which
is what the suite runs at - and one that does not is refused on the line
that handed over the clock, not at run time and not in silence.

A conversion is the sampling time plus 11 ADCCLK cycles (12.2.2). The
datasheet's table 4-27 gives the two ends as 14 and 252 ADCCLK where the
manual's arithmetic gives 12.5 and 250.5: a cycle and a half at each
end, which is the sample-and-hold the datasheet counts and the manual
does not. The driver follows the manual and says so.

### The power-up and the calibration

ADON written once wakes the converter (tSTAB, a microsecond by the
datasheet); ADON written AGAIN with nothing else changing is a START.
Every verb of the driver therefore writes CTLR2 as a read-modify-write,
and a software start is SWSTART under EXTSEL = 111 with EXTTRIG set,
which `init()` arms - so `start()` is one spelling whatever the group.

12.2.2 asks for a calibration at every power-up: RSTCAL until the
hardware clears it, then CAL until the hardware clears it, the converter
powered for at least two ADCCLK cycles first, and the code left behind
in the regular data register.

THE ORDER IS NOT OBVIOUS, and CTLR1.BUFEN's own note is where it hides:
setting TSVREFE (or the TKEY enable) turns the input buffer ON and it
cannot be turned off again, so the calibration must happen BEFORE them,
with the buffer off. `init()` runs exactly that order - gate, reset, the
control words with the buffer and the gain still clear, the power-up,
tSTAB, the calibration, and only then the buffer, the gain and the
internal sources.

### The two groups

The REGULAR group is up to sixteen conversions named in RSQR1 to RSQR3
with their count in L; its result lands in one register, RDATAR, which
is why a sequence of more than one channel wants the DMA or a reader
fast enough to take each datum before the next. The INJECTED group is up
to four, named in ISQR, and it PREEMPTS the regular one: a trigger for
it resets the regular conversion in flight, runs the injected sequence
and gives the regular one back.

Two asymmetries follow from that. The injected group's four slots FILL
FROM THE END - a length of two uses JSQ3 and JSQ4 - while its results
and its offsets count from the START, in conversion order. And its
result is SIGNED: IDATARx holds the raw datum LESS the per-conversion
offset of IOFRx, with a sign bit above it, so `injected_result()`
returns an `int16_t` where the regular one returns unsigned counts.

The injected group takes no DMA on either converter (12.2.2's note).

### The sampling time, and the source it can settle

Eight codes from 1.5 to 239.5 ADCCLK cycles, one per channel, in two
registers (SAMPTR2 holds channels 0 to 9, SAMPTR1 the rest). What a
sampling time buys is stated in datasheet table 4-28 as a SOURCE
IMPEDANCE: 0.4 kOhm at 1.5 cycles, 5.9 at 7.5, 11.4 at 13.5, 25.2 at
28.5, 37.2 at 41.5, 50 at 55.5, and "invalid" above that - the number
the formula gives is past the 50 kOhm the converter is rated for at all.
`adc_max_source_ohms()` is that table, and it is the one piece of the
analog chapter a digital program can act on: the pads' own weak pull is
30 to 50 kOhm (table 4-19), so a pad held by its own pull is a source
only the longest times settle.

MEASURED, AND WITH A TWIST THE TABLE DOES NOT MENTION: the
sample-and-hold TRACKS the selected channel between conversions. With
the multiplexer parked on a pulled pad for the microseconds a polled
read costs, every sampling time - the shortest included - reads the
pull's own rail. The sampling time only bites when the multiplexer MOVES
at the start of the conversion, which is what a scanned sequence does,
and there the ladder is the physics (the findings below).

### The analog watchdog

One window, two thresholds twelve bits wide, and four bits of scope:
AWDEN and JAWDEN say which GROUPS are guarded, AWDSGL narrows that to
the single channel in AWDCH. A conversion outside the band - above the
high threshold or below the low one - raises AWD and, with AWDIE, the
interrupt. The thresholds may be changed during a conversion and take
effect at the next one (12.3.7). In scan mode the interrupt ABORTS the
scan (12.3.2's note on AWDIE).

### The triggers

Each group has its own list of eight sources, and they are different
lists: the regular group takes TIM1's first three captures, TIM2's
second, TIM3's TRGO, TIM4's fourth capture, an EXTI line and SWSTART;
the injected one takes TIM1's TRGO and fourth capture, TIM2's TRGO and
first capture, TIM3's fourth, TIM4's TRGO, an EXTI line and JSWSTART.
Only the RISING edge of a trigger starts a conversion (12.2.3's note).

CODE 110 IS AN EXTI LINE ON THIS FAMILY AND NOTHING ELSE - line 11 for
the regular group, line 15 for the injected one. The manual writes it
"EXTI line11/TIM8_TRGO" because the same chapter serves families that
have a TIM8, and the four AFIO bits that would select the timer instead
(ADC1/ADC2_ETRGREG_RM and _ETRGINJ_RM) carry a note naming CH32F20x_D8,
CH32F20x_D8C, CH32V30x and CH32V31x - no CH32V20x. So there is no remap
verb in this driver and the enumerators say `exti11` and `exti15`.

What makes an EXTI line reach the converter is its EVENT enable and not
its interrupt enable - measured, and stated nowhere in either chapter.

### The dual modes

With two converters, ADC1 leads and ADC2 follows: CTLR1's DUALMOD on the
MASTER names one of ten arrangements (regular simultaneous, injected
simultaneous, fast and slow interleaved, alternate trigger, and four
combinations), and the field is reserved in ADC2. The follower's datum
arrives in the UPPER HALF of the master's data register. 12.2.7's note
asks for the external trigger to be set on the master and a software
trigger on the slave, so that no spurious edge starts the follower
alone, and for the two groups to take the same time.

`Adc<1>::dual()` is the verb; on the second converter, and on a part
with one converter, it does not compile - the master's field is not a
spelling both instances share. There is NO util shape for this: two
converters in lockstep are a capability no other stratum in this tree
has, and one family does not make a contract.

### What WCH added, and what belongs to another class

CTLR1 carries an input BUFFER (BUFEN) for sources above the impedance
the sampling switch tolerates, and in front of it a PROGRAMMABLE GAIN
(PGA) of 1, 4, 16 or 64; the buffer must be on for the gain to mean
anything, which the configuration refuses without. The same register's
TKEY bits (TKENABLE, TKITUNE) belong to RM ch. 13 and nothing here
writes them.

The AUX register at offset 0x54 - the short sampling times of 2.5 to 5.5
cycles - names CH32F20x_D8, CH32F20x_D8C, CH32V30x_D8, CH32V30x_D8C and
CH32V31x_D8C in its own note, and no CH32V20x: no part of this family
has it. The register map carries the word so a reader can hold table
12-5 beside the code; no verb writes it.

### One vector, two converters

Both converters report on entry 34 of the vector table. A program that
uses both calls the ISR body of each and unions the answers, and the
body returns zero for the converter that had nothing - which is what
makes that safe. The flags are write-zero-to-clear (STATR's RW0, the
timers' discipline), and EOC is also cleared by READING the data
register, which a handler that takes the result does anyway.

### The stream, and why a block source does not ride circular mode

Only ADC1 has a DMA request, on channel 1 (table 11-5), and the channel
IS the request on this controller. `claim_stream<Engine>()` arms an
engine on the data register and sets CTLR2.DMA in one verb, refusing at
compile time an engine on any other channel: an engine on the wrong
channel would wait for a datum that never comes, which is a wedge and
not an error.

The engine behind it is `DmaPingPongEngine`, this family's realization
of [util/block_stream.hpp](../../brio/util/block_stream.hpp)'s
`BlockSource`, and it does NOT use the controller's circular mode even
though the controller has one. The contract's rule is SKIP RATHER THAN
TEAR, and on a channel that never stops the decision could only be taken
after the edge - by which time the controller is already writing the
buffer the caller holds. That was the STM32G0's finding and it is
measured again here (the findings below). So the source stops itself at
every block and its handler re-arms the other buffer, while the PLAYER,
`DmaLoopEngine`, rides circular mode exactly as on the STM32G0: a table
fed for ever, with the lap interrupt doing nothing but count.

AND NO STREAM SLEEPS. In the Sleep of RM 2.4 no bus master but the core
gets a cycle ([dma.md](dma.md)), so a DMA-fed converter stalls for the
whole sleep: a program that runs one holds itself awake.

## Types and verbs

### The vocabulary

`AdcSampleTime` (the eight codes, with `adc_sample_shortest` and
`adc_sample_longest` for a program that wants an end without naming a
count), `adc_sample_half_cycles` and `adc_conversion_half_cycles` (the
arithmetic in halves, so the .5 stays exact), `adc_conversion_ns` (the
same against a stated ADCCLK), `adc_max_source_ohms` (table 4-28),
`AdcGain` with `adc_gain_factor`, `AdcTrigger` and `AdcInjectedTrigger`
(tables 12-1 and 12-2, with `adc_regular_exti_line` and
`adc_injected_exti_line` beside them), `AdcDualMode` (the ten codes),
`AdcFlag` (the five status bits by their meaning), `AdcConfig` and
`AdcWatchdogConfig` with their `*_valid()` predicates, and the constants
a program scales with: `adc_bits`, `adc_steps`, `adc_max_count`,
`adc_channels`, `adc_temperature_channel`, `adc_vrefint_channel`,
`adc_vrefint_mv`, `adc_temperature_v25_mv`,
`adc_temperature_slope_uv_per_c`.

### The resource

`Adc<1|2>`: the gate and the reset line, `init(clock, config)` (which
refuses a clock out of specification at compile time and a configuration
the chapter forbids at run time), `calibrate` and `calibration_code`,
`power`, the sampling times per channel and for all of them at once, the
regular sequence and its read-back, the polled `read` / `read_settled`,
`result` / `result_counts` / `data` / `follower_counts`, the injected
sequence with its offsets and its signed results, the watchdog and its
thresholds, the triggers and their enables, `continuous`, `dma`,
`internal_sources`, `gain`, `dual`, the flags and their interrupts, the
ISR body `isr()`, and `claim_stream<Engine>()`. What an instance HAS is
published beside them: `has_dma`, `has_internal_sources`,
`has_dual_mode`.

### The inputs

`AnalogIn<Pin>` is a pad handed to its channel, refused at compile time
for a pad that carries none; `claim()` puts it in analog mode.
`AdcInput` is the two internal sources as tags on their channel numbers.
Both are `input_code()`-able, which is what the sampler walks.

### The reference, and the arithmetic over it

No package of this series brings out a VREF+ pad, so the converter's
reference IS the analog supply: `Ref` has one enumerator, `vdda`, and
`ref_mv()` takes the board's millivolts. `Adc<1>::vdda_mv(counts)`
measures it instead of assuming it, from a conversion of VREFINT;
`millivolts()` is [util/analog.hpp](../../brio/util/analog.hpp)'s
`adc_mv` over the converter's own full scale; and
`temperature_centi_c(counts, vdda)` is 12.2.6's formula with the
datasheet's typical numbers.

THERE IS NO FACTORY CALIBRATION TO READ. The electronic signature (RM
ch. 31) holds the flash capacity and a 96-bit unique identifier and
nothing else, so there is none of the STM32F4's `AdcFactory` here: the
reference is 1.17 to 1.23 V and the sensor's slope 3.8 to 4.7 mV per
degree, and both are typical values a program may not sharpen.

### The sampler's surface

`start()`, `selected()`, `select(in)` and `input_code(in)` are what
[util/analog_sampler.hpp](../../brio/util/analog_sampler.hpp) asks of a
converter, and `Adc<n>` satisfies them with those spellings.

## How to use it

One conversion of a pad, polled:

```cpp
using Vin = brio::AnalogIn<brio::Pin<'A', 1>>;        // channel 1
using SysClock = brio::Clock<brio::ClockSource::pll, 96'000'000>;   // ADCCLK 12 MHz
constexpr SysClock clock;

Vin::claim();
(void)brio::Adc<1>::init(clock, {.internal_sources = true});
brio::Adc<1>::sample_time_all(brio::adc_sample_longest);
brio::Adc<1>::select(Vin{});
const uint16_t counts = brio::Adc<1>::read();
```

The supply, and a reading in millivolts against it:

```cpp
brio::Adc<1>::select(brio::AdcInput::vrefint);
const uint16_t vdda = brio::Adc<1>::vdda_mv(brio::Adc<1>::read_settled(4));
const uint16_t mv = brio::Adc<1>::millivolts(counts, vdda);
```

A scanned sequence streamed into two caller-owned blocks, lent on by a
relay:

```cpp
using Source = brio::DmaPingPongEngine<
    brio::DmaRequestOf<brio::DmaRequest::adc1>::channel, uint16_t>;
using Relay = brio::BlockRelay<P, brio::Subscribers<Consumer>, Source>;

(void)brio::Adc<1>::init(clock, {.scan = true, .dma = true});
const uint8_t order[4] = {1, 2, 17, 16};
(void)brio::Adc<1>::sequence(order, 4);
brio::Adc<1>::claim_stream<Source>();
(void)Source::start(block_a, block_b, 8);
brio::Adc<1>::continuous(true);
brio::Adc<1>::start();
```

with the channel's vector doing the two things a stream's glue does:

```cpp
extern "C" BRIO_CH32_INTERRUPT void dma1_channel1_handler() {
    const uint8_t f = Source::service();
    if ((f & Source::flag_complete) != 0u) {
        (void)Source::complete();
        brio::post<Relay>(brio::BlockDone{});
    }
    if ((f & Source::flag_error) != 0u) { Source::fail(); }
}
```

A timer pacing the regular group, with the conversions counted in the
converter's own interrupt:

```cpp
(void)brio::Tim<3>::configure({.prescaler = 95, .period = 499});   // 2 kHz
(void)brio::Tim<3>::master(brio::TimMasterMode::update);
brio::Adc<1>::trigger(brio::AdcTrigger::tim3_trgo);
brio::Adc<1>::interrupts(brio::Adc<1>::converted_interrupt, true);
brio::Pfic::enable(brio::Adc<1>::irq());
brio::Tim<3>::enable(true);
```

A pad's own edge pacing it instead - the line's EVENT enable is what
reaches the converter:

```cpp
(void)brio::Exti::select(brio::adc_regular_exti_line, 'B');   // line 11 from port B
(void)brio::Exti::sense(brio::adc_regular_exti_line, brio::ExtiSense::rising);
(void)brio::Exti::event(brio::adc_regular_exti_line, true);
brio::Adc<1>::trigger(brio::AdcTrigger::exti11);
```

The injected group preempting it, with an offset on its first
conversion:

```cpp
const uint8_t injected[2] = {1, 2};
(void)brio::Adc<1>::injected_sequence(injected, 2);
(void)brio::Adc<1>::injected_offset(0, 2000);
brio::Adc<1>::injected_start();
const int16_t first = brio::Adc<1>::injected_result(0);   // signed
```

The watchdog around a level, with its interrupt:

```cpp
(void)brio::Adc<1>::watchdog({.low = 100, .high = 3000, .channel = 1, .interrupt = true});
```

Two converters in lockstep, and both halves out of one register:

```cpp
brio::Adc<1>::select_channel(1);
brio::Adc<2>::select_channel(2);
(void)brio::Adc<1>::dual(brio::AdcDualMode::regular_simultaneous);
brio::Adc<1>::start();
const uint16_t master = brio::Adc<1>::result_counts();
const uint16_t follower = brio::Adc<1>::follower_counts();
```

And the sampler over the converter, walking a list at a software pace:

```cpp
using Sampler = brio::AnalogSampler<brio::Adc<1>, P, brio::Subscribers<Monitor>,
                                    Vin{}, brio::AdcInput::vrefint, brio::AdcInput::temperature>;

extern "C" BRIO_CH32_INTERRUPT void adc1_2_handler() {
    const uint8_t in = brio::Adc<1>::selected();
    const uint16_t v = brio::Adc<1>::result_counts();
    brio::post<Sampler>(brio::Sampled{v, in});
}
```

## Bench findings

`test_vx03_adc`, 57 verdicts in `z`, on the CH32V203C8T6 with the PLL on
the HSI at 96 MHz and ADCCLK at 12 MHz. NOTHING OUTSIDE THE CHIP: the
levels are a pad driven or pulled by its own port, the internal
reference and the temperature sensor, and the core's own counter is the
ruler.

- **The calibration costs eight microseconds and leaves a word behind.**
  ADON was acknowledged within the counter's own resolution, RSTCAL
  cleared itself in under a microsecond and CAL took 8 us - the
  datasheet's tCAL of 100 ADCCLK is 8.3 us at 12 MHz - and the data
  register went from 0 to 2049, which is the calibration code 12.2.2
  says lands there.
- **VREFINT reads 1489 counts, which puts the supply at 3301 mV.** The
  reference is 1.2 V nominal, so the arithmetic is the whole measurement
  of a board whose rail nobody has metered.
- **The temperature sensor reads 1742 counts = 1404 mV**, which the
  datasheet's typical V25 of 1.40 V and slope of 4.3 mV per degree make
  24.7 degrees Celsius, at a conversion of 20.8 us (12.2.6 asks for
  17.1 us of sampling and the longest code gives 20 us of it). The
  spread on those two numbers is worth about twelve degrees, so this is
  a temperature CHANGE to be trusted and an absolute temperature to be
  doubted.
- **A pad driven by its own port is a hard source at either rail**: a
  push-pull output high reads 4095 counts = 3300 mV, the same pad low
  reads 9. A pad held by its own PULL is not: those two facts are what
  the whole of table 4-28 is about.
- **THE SAMPLING LADDER, AND WHEN IT BITES.** The pulled pad converted
  RIGHT AFTER the grounded one in a scanned sequence, so that the
  multiplexer moves at the start of the conversion: 1060, 4010, 4076,
  4091, 4094, 4093, 4095, 4095 counts from 1.5 to 239.5 cycles. That is
  a 40 kOhm source charging an 8 pF sample-and-hold - a quarter of the
  step at 1.5 cycles, within half a per cent of the rail from 13.5 on,
  and at the rail from 71.5. Table 4-28 puts a 40 kOhm source between
  its 41.5-cycle row (37.2 kOhm) and its 55.5-cycle one (50 kOhm),
  which is where the last counts arrive: the table is about a quarter
  of an LSB and this ladder is about a reading.
- **AND THE SAME SOURCE READS FULL SCALE AT EVERY SAMPLING TIME when
  the multiplexer is parked on it**: 4019, 4074, 4087, 4093, 4094, 4094,
  4095, 4095 across the same eight codes, taken with the two channels
  selected separately so a polled read's own microseconds sit between
  the selection and the conversion. The sample-and-hold TRACKS the
  selected channel between conversions; the sampling time is what a
  SEQUENCE pays and what a single parked channel does not.
- **A pad in PULLED INPUT mode is an analog source.** Analog mode takes
  the pull away with the input driver (10.2.7), so a program that wants
  a pad's own pull as its level leaves the pad a pulled input and
  converts it through that - which works, in both directions (the
  pull-down reads 8 counts).
- **The stream runs at the converter's pace and the relay keeps up.**
  Four channels scanned continuously at the longest sampling time, the
  DMA into two blocks of eight: four blocks lent to a subscriber in
  679 us - 170 us a block, 21 us a sample, which is the 20.8 us
  conversion plus the sequence's own overhead - with zero overruns. The
  first block read 4095, 9, 1488, 1744, 4095, 9, 1488, 1743: the rail,
  ground, the reference and the sensor, twice, in the sequence's own
  order.
- **The overrun is the contract's, measured.** With nobody releasing,
  the second block filled, the engine counted one overrun, held two
  buffers and STOPPED rather than write into the block it had lent; one
  `release()` restarted it and the laps went on.
- **AND THIS IS WHY IT IS NOT CIRCULAR.** A block at the CONTROLLER's
  own speed, stopped by the handler of its own half flag - a handler
  whose whole body is read-CNTR-and-disable - found THREE of the next
  thirty-two items already written. On a channel that never stops,
  "skip rather than tear" would be decided three items too late. The
  STM32G0 measured 0 to 6; this is the third controller to say the same
  thing.
- **The injected group preempts and gives back.** With a continuous
  regular conversion running, a two-channel injected group started,
  raised JSTRT, converted both and left the regular conversion running
  afterwards. Its first result was the rail LESS an offset of 2000 -
  2095 counts - and its second, with no offset, was ground.
- **JEOC is the GROUP's flag, not the conversion's**: it reached its
  handler exactly once for a two-conversion injected group.
- **The watchdog is a band and not a ceiling.** A conversion above the
  high threshold raised the flag and its interrupt; one inside the
  window raised nothing; one below the low threshold raised it again.
  With AWDSGL and a channel number, a conversion of ANOTHER channel
  outside the window raised nothing, and with the single-channel bit
  dropped every regular channel was guarded.
- **A timer's TRGO paces the converter at its own rate**: TIM3's update
  at 2 kHz produced 399 conversions in 200 ms, one short of the 400 the
  window holds.
- **AN EXTI LINE REACHES THE CONVERTER THROUGH ITS EVENT ENABLE AND NOT
  ITS INTERRUPT ENABLE.** Twenty rising edges on a pad of the line
  started 0 conversions with the line merely sensed, 20 with EXTI_EVENR
  set, and 0 with EXTI_INTENR set instead - and in that last pass the
  line's own handler ran for every one of the twenty edges. Neither
  chapter says which enable feeds a peripheral; this is the answer.
- **The dual mode carries both converters in one register.** Regular
  simultaneous, the master on a pad at the rail and the follower on one
  at ground: the master's data register read 0x30FFF - 4095 in its low
  half, 3 in its high one. All ten codes were written and read back and
  an eleventh was refused.
- **The follower's datum is there with the DMA bit DROPPED too.**
  12.2.7's note 1 says the DMA must be enabled "to read the slave
  converted data on the master data register"; the upper half carried it
  either way. The note is about getting the pair into memory, not about
  the register.
- **Discontinuous mode steps by DISCNUM and stops.** A sequence of six
  with DISCNUM at two, counted by the DMA: 2 conversions after the first
  trigger, 4 after the second, 6 after the third, and the data was the
  sequence's own channels in order.
- **But its EOC is the SEQUENCE's flag.** The same three triggers with
  the DMA off raised EOC 0, 0 and 1 times: the flag comes once, when the
  last subgroup ends, exactly as 12.2.4's own example draws it - which
  is why a discontinuous group wants the DMA, whose request is per
  conversion.
- **The sampler walks the list and the attribution holds.** Three inputs
  - a pad at the rail, VREFINT, a pad at ground - at a software pace:
  indices 0, 1, 2 with 4095, 1488 and 10 counts, no result carrying a
  code the list does not hold.

## Not covered yet

Driver gaps, each with its reason:

- **TKEY, WCH's capacitive touch sensing (RM ch. 13), DECLINED.** The
  block is an ADC mode - the converter's charge transfer against a pad's
  capacitance, with TKENABLE and TKITUNE in CTLR1 and the thresholds in
  the touch chapter - and what it produces is a key press, which is an
  application and not a driver. The bits are named in the register map
  and nothing writes them.
- **The ADCx_AUX register and its short sampling times** (2.5 to 5.5
  cycles): its own note gives it to CH32F20x_D8, CH32F20x_D8C,
  CH32V30x_D8, CH32V30x_D8C and CH32V31x_D8C, none of which is a
  CH32V20x. Not this family's, so not implemented.
- **The trigger remaps** (AFIO's ADC1/ADC2_ETRGREG_RM and _ETRGINJ_RM):
  the same note, the same reason. On this family trigger code 110 is an
  EXTI line and the alternative does not exist.
- **A real analog source.** Every level this document reports is a rail,
  a weak pull or an internal reference; there is no DAC on this family
  and no wire on the board. A voltage between the rails - a divider, an
  external reference, a filtered PWM pad - is what would measure
  linearity, the offset and gain errors of table 4-29, and the
  converter's actual accuracy rather than its behaviour.
- **The player engine's own driver.** `DmaLoopEngine` is written and
  compiled (and concept-checked against `BlockPlayer`), but the
  peripheral it would feed for ever on this family is a DAC, which this
  series has not got. Its first user will be a timer's compare register
  or a bus transmitter, and until then it is measured only by the
  controller's circular mode under the DMA chapter's own suite.

Implemented but not bench-verified, each with what would measure it:

- **The input buffer and the programmable gain** (BUFEN, PGA). The gain
  amplifies a SMALL signal, and every source this board offers is a rail
  or an internal reference: at a gain of four a 1.2 V reference is past
  full scale and a rail is far past it. A source of a few tens of
  millivolts - the same wire the linearity measurement needs - is what
  would measure them.
- **The interleaved and alternate-trigger dual modes.** All ten codes
  are written and read back; only regular simultaneous is measured
  converting. The others want two channels carrying a CHANGING signal to
  tell a fast interleave from a slow one, which is the analog source
  again.
- **The second converter as a lone instrument.** ADC2 is brought up,
  converts and is read in the dual-mode letter; its own watchdog, its
  own injected group and its own triggers are the same registers at
  another address and are exercised only by the family fixture.
- **JAUTO, the automatic injected group** after the regular one: written
  and refused where the chapter forbids it, and not driven - the
  measurement is the same shape as the trigger letter's and waits for a
  program that wants twenty conversions in one start.
- **The eight parts other than the CH32V203C8.** The converter count,
  the channel count and the pad map fold through each part's own table
  and the whole stratum compiles for all nine both ways the hardware
  prologue can be built (`brio check ch32vx03`); the 128 KB part's six
  extra channels and its single converter are asserted at compile time
  and measured on none of them. What would measure them is a board.
