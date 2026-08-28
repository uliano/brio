# SDADC - Sigma-Delta Analog-to-Digital Converter (SAM C21)

> **PROVISIONAL.** The whole of chapter 39 is implemented and most of it
> is bench-verified, but this board cannot put a DC voltage other than a
> rail across a differential pair: the DAC does not reach an SDADC input
> (it reaches this converter only as a REFERENCE), so every non-rail
> input here is a PWM waveform averaged by the converter's own decimation
> filter. What that cannot answer, plus sleep, the external reference at
> anything but a rail, the interrupts through the NVIC and the analog
> control's undocumented fields, is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 39, the SDADC
characteristics of tables 45-26, 45-27, 45-28 and 45-29, and the event
tables of ch. 29 - and errata DS80000740S, read on the **E/G/J row at
revision F**, where exactly ONE item applies: the device-level 1.8.10
(the DAC as this converter's reference). Driver: `samc/sdadc.hpp`, over
the reserve's SDADC entries in `samc/device_tables.hpp`. The family
fixture is `test/family_samc/sdadc.cpp` plus ten negatives under
`tools/check_samc.sh`; the bench suite is `test_samc_sdadc`.

## What the silicon does

**One 16-bit sigma-delta converter over three DIFFERENTIAL pad pairs**,
up to 1.5 Msps divided by the over-sampling ratio. A first-order-class
modulator feeds a **third-order SINC decimation filter** (39.6.3), and
the filter - not a successive approximation - is what makes the result.
Everything that surprises about this chapter follows from that.

**The input is always differential.** `INPUTCTRL.MUXSEL` names a PAIR of
pads (AINN*k*, AINP*k*), never a single one, and there are three pairs.
"Single-ended mode" in the electrical tables is the same thing with the
negative pad grounded EXTERNALLY (table 45-26 note 3): it costs a bit of
resolution and is not a register setting.

**The result is signed** and spans -VREF .. +VREF. The chapter
contradicts itself four times about this: 39.1 and 39.8.19 say signed,
while 39.6.1, 39.6.3.1 and 39.6.3.4 all say unsigned and the last even
saturates to [0 : 2^16-1]. The silicon is signed - see the findings.

**There is no sample length and no resolution knob.** The output rate is
CLK_SDADC_FS / OSR, the specified resolution is always 16 bits, and OSR
is the only thing trading rate against noise. 39.8.3 adds that "the OSR
must never be changed while the SDADC is running".

**SKPCNT is a warm-up, not an average.** A sigma-delta's filter has to
fill before its output means anything, so a SINGLE conversion runs
SKPCNT + 1 decimation windows and returns the last. The reset value is 2,
which is 39.6.2.3's "the first valid sample starts from the third sample
onward" written as a register - and it is also erratum 1.18.3's own
workaround for the revision where it reset to zero.

### The clock, in three stages

    CLK_SDADC    = GCLK_SDADC / (2 x (PRESCALER + 1))     39.5.3
    CLK_SDADC_FS = CLK_SDADC / 4                          39.5.3
    output rate  = CLK_SDADC_FS / OSR                     free running
                 = CLK_SDADC_FS / (OSR x (SKPCNT + 1))    single

The factor of four between the prescaler's output and the sampling
frequency is easy to read past and is a factor of four in every rate.
Table 45-26 bounds CLK_SDADC at **1 MHz .. 6 MHz**, so at a 48 MHz
generator PRESCALER must be 3..23; `init()` refuses a prescaler that
leaves the range when it is told the generator's rate.

**PRESCALER is eight bits and LINEAR**, and this is where the device
header lies: it declares DIV2, DIV4, DIV8 ... DIV256 for codes 0..7,
which is the SAM D21's power-of-two prescaler. A power-of-two reading
cannot reach 39.5.3's own /512 from an eight-bit field at all. Measured
(below), and `sdadc_prescaler_divisor()` follows the datasheet.

There is no `rebase()` and no `ClockUser`: the converter has its own
generic clock channel, so a main-clock change does not move CLK_SDADC
(the DynamicClock deferral in [clock.md](clock.md)).

### The four register disciplines

1. **Enable-protected.** 39.6.2.1's list is CTRLA.ONDEMAND/RUNSTDBY,
   CTRLB, CTRLC, EVCTRL, ANACTRL; the individual property lines say
   Enable-Protected for REFCTRL, CTRLB and EVCTRL only. **The silicon
   agrees with the property lines** (measured): REFCTRL, CTRLB and EVCTRL
   discard a write made under a running converter, CTRLC and ANACTRL take
   it. The driver writes all five only while the converter is disabled -
   correct under either reading - and the runtime verbs refuse while it
   is enabled.
2. **Write-synchronized, AND THIS CHAPTER THREATENS A BUS ERROR.**
   39.6.8: "If an operation that require synchronization is executed
   while its busy bit is on, the operation is discarded **and a bus error
   is generated**." The ADC's and the DAC's chapters promise a silent
   discard; here the threatened penalty is a HardFault. So every
   synchronized write in `sdadc.hpp` **waits BEFORE writing**, not after,
   and returns false rather than storing into a busy register - which is
   why `select()` is a `bool` where `samc/adc.hpp`'s is a void store.
   The bits that exist: SWRST, ENABLE, CTRLC, INPUTCTRL (SYNCBUSY calls
   it MUXCTRL), WINCTRL, WINLT, WINUT, OFFSETCORR, GAINCORR, SHIFTCORR,
   SWTRIG, ANACTRL.
   **REFCTRL is in 39.6.8's prose list and has NO SYNCBUSY BIT** (the
   register is twelve bits wide and every one is accounted for). One of
   the two statements has to be wrong; the driver treats REFCTRL as
   enable-protected only, as its own property line does.
3. **Write-only.** SWTRIG's two bits are `W` in 39.8.17 and
   self-clearing: there is nothing to read back, and a `false` from
   `start()` means the register was busy, not that the conversion failed.
4. **Neither.** EVCTRL, INTENSET/CLR, INTFLAG, SEQSTATUS, SEQCTRL and
   DBGCTRL - which 39.8.22 also says is not reset by a software reset.

Not PAC-protected (39.5.8): INTFLAG, and nothing else.

### The datapath is twenty-four bits wide

39.8.19 calls RESULT "a signed integer value with 24-bit size" and adds
that the conversion result is "left-adjusted" in it, which reads like
eight bits of padding. **It is not padding.** The register saturates at
+/-2^23, the corrections of 39.6.3.4 act in those raw units, and the low
bits carry real filter output that moves where the specified sixteen are
bit-exact. So the driver has three result verbs:

| verb | what it is |
|------|------------|
| `result()` | the SPECIFIED 16-bit signed datum, the top 16 bits - what `half_steps` (32768) scales and what table 45-26's "Res 16 bits" means |
| `result24()` | the whole signed 24-bit value the datapath carries |
| `result_raw()` | the register, unsigned |

`sdadc_raw_per_count` (256) is the factor between the first two, and it
is the unit OFFSETCORR speaks.

### The post-processing, which is why a Multislope wants this part

    Data = (Data0 + OFFSETCORR) x GAINCORR / 2^SHIFTCORR

with OFFSETCORR signed 24 bits, GAINCORR unsigned 14, SHIFTCORR four
(39.6.3.4, and each register's own description prints the formula
again). Two things follow that no caller should have to discover:

- **GAINCORR's reset value is ONE, not zero**, and a GAINCORR of zero
  multiplies every result to nothing. `sdadc_config_valid()` refuses it.
  Erratum 1.18.3 is exactly that on revision B, where the reset value WAS
  zero - so this revision's printed reset values (GAINCORR 0x0001 and
  CTRLB.SKPCNT 2) ARE that item's workaround baked into the silicon.
- **The gain is an integer over a power of two**, not the SAR's
  fixed-point fraction: a gain of 1.5 is GAINCORR 3 with SHIFTCORR 1.
  `sdadc_gain_permille()` and `sdadc_corrected()` do that arithmetic
  once, in raw units.

**Chopping** (`ANACTRL.ONCHOP`) is the analog half of the same idea:
39.6.3.4 says offset error "can be compensated by setting the Chopper
mode ON", and table 45-27's DC figures are all taken with it on.

### What the header has and the chapter does not

`REFCTRL.REFRANGE`, two bits at 5:4, present in the device header's
register mask (0xB3) and in NO part of chapter 39 - not the bit table,
not the register summary, not the prose. It is exposed because the house
rule is that the header decides what EXISTS, and it is documented as
unknown because nothing in the documents of record says what it selects.
It is a real field (measured: it stays written).

`ANACTRL.CTLSDADC` is drawn as five bits (4:0) in 39.8.21 and declared as
six (`CTRSDADC`, mask 0x3F) in the header. The header is right
(measured). `ANACTRL.BUFTEST` is named by the register description and
given no description at all; it is exposed for completeness and nothing
here can say what it does.

## Types and verbs

`brio/samc/sdadc.hpp` is the reference. In outline:

- `Sdadc` - a **MONOSTATE**, not `Sdadc<n>`: one instance on every C21
  variant, so an index would be a parameter with a single legal value
  (the `Rtc` and `Dac` precedent). `sdadc_count()` in the reserve is what
  makes "and none on a C20" a compile-time fact.
- `SdadcRef` - this converter's OWN reference vocabulary (INTREF, VREFB,
  DAC, VDDANA). Deliberately not `brio::Ref` (the ADC's six codes) nor
  `DacRef` (the DAC's three): this family has no shared reference block,
  so one enum would be a type no register accepts. **Four codes and none
  Reserved** - the only reference field of the three converters with no
  illegal value.
- `SdadcOsr`, `SdadcWindow`, `SdadcEventControl`, `SdadcConfig` and
  `sdadc_config_valid()` - the knobs and the refusals.
- The clock arithmetic: `sdadc_prescaler_divisor()`, `sdadc_clock_hz()`,
  `sdadc_sampling_hz()`, `sdadc_clock_in_range()`, `sdadc_prescaler_for()`,
  `sdadc_conversion_cycles()`, `sdadc_result_hz()`, `sdadc_conversion_us()`.
- The result and correction arithmetic: `sdadc_result_of()`,
  `sdadc_raw_signed()`, `sdadc_threshold_word()`, `sdadc_corrected()`,
  `sdadc_gain_permille()`, and the constants `sdadc_half_steps`,
  `sdadc_raw_half_steps`, `sdadc_raw_per_count`.
- Pads: `pair_exists()`, `negative_port()/negative_pin()`,
  `positive_port()/positive_pin()`, `negative_function()`,
  `positive_function()`, `vrefb_function()`, `claim_negative<P>()`,
  `claim_positive<P>()`, `claim_vrefb<P>()`.
- Conversions: `select()` (runtime and templated), `start()`, `flush()`,
  `ready()`, `overrun()`, `result()/result24()/result_raw()`, `read()`,
  `next()`, `discard()`, `free_running()`.
- The window: `window()`, `window_raw()`, `window_off()`,
  `window_mode()`, `window_hit()`, `window_flag()`.
- The corrections: `offset_correction()`, `gain_correction()`,
  `shift_correction()`.
- The sequencer: `sequence()`, `sequence_busy()`, `sequence_state()`.
- Events: `event_config()`, `start_on()`, `flush_on()`, `stop_events()`
  - the first two REFUSING any channel that is not asynchronous, which
  39.6.6 requires in so many words.
- Interrupts: `arm()/disarm()/armed()/flags()/clear_flags()`, `isr()`,
  `resrdy()`.

### The refusals

`sdadc_config_valid()` returns false, and `init<cfg>()` fails to
compile, for:

- a Reserved OSR code (0x5..0x7) or window mode (0x5..0x7);
- **INTREF or the DAC as reference without `REFCTRL.ONREFBUF`** -
  39.8.2's own Note asks for the buffer with both, and for the DAC that
  bit is erratum 1.8.10's entire workaround;
- **`GAINCORR` = 0**, which multiplies every result away;
- **a single-conversion `SKPCNT` under two** - 39.6.2.3, the reset value
  and erratum 1.18.3's workaround all say those windows are invalid data
  (free running warms up once at the start and is exempt);
- a SEQCTRL bit for a differential pair this package does not bond;
- `SHIFTCORR` or `SKPCNT` past their fields, `CTLSDADC` past six bits,
  `REFRANGE` past two;
- an inverted event input nothing listens to.

`select()` refuses a MUXSEL past the three pairs and a pair this package
does not bond; the templated `select<pair>()` makes both a compile error.
`claim_negative<P>()`/`claim_positive<P>()` refuse a pad of the wrong
polarity: **PA06 is AINN0 and PA07 is AINP0, never the other way round.**

### The pads, per package

| pair | negative | positive | E | G | J |
|------|----------|----------|---|---|---|
| 0 | PA06 | PA07 | yes | yes | yes |
| 1 | PB08 | PB09 | - | yes | yes |
| 2 | PB06 | PB07 | - | - | yes |
| VREFB | PA04 | | yes | yes | yes |

This is the most package-dependent map in the stratum: the E has one
differential input, the G two, the J three. All of these pads are also
SAR analog inputs (PA06/PA07 = ADC0/AIN6, AIN7; PB08/PB09 = ADC0/AIN2,
AIN3 and ADC1/AIN4, AIN5; PA04 = ADC0/AIN4), which is what lets one
converter watch what the other is reading with no wire.

**PA02 - the DAC's VOUT, ADC0/AIN0 and the AC's AIN4 - is NOT an SDADC
pad.** This converter meets the DAC only through `REFCTRL.REFSEL = DAC`.

### One example per use

A single differential reading against the supply:

```cpp
using namespace brio;
Sdadc::init(0, SdadcConfig{
    .reference = SdadcRef::vddana,
    .prescaler = 3,                 // 48 MHz / 8 = 6 MHz
    .osr = SdadcOsr::osr256,
    .chopper = true}, 48'000'000);
(void)Sdadc::select<0>();           // the PA06/PA07 pair
int16_t counts = 0;
if (Sdadc::read(counts)) {
    const int16_t mv = Sdadc::to_mv(counts, vdd_mv);   // signed
}
```

A free-running stream at the finest reference:

```cpp
Sdadc::init(0, SdadcConfig{
    .reference = SdadcRef::intref,
    .reference_buffer = true,       // 39.8.2's Note: required, and refused without
    .prescaler = 3,
    .osr = SdadcOsr::osr1024,
    .free_running = true,
    .chopper = true}, 48'000'000);
int16_t v = 0;
while (Sdadc::next(v)) { /* one result every 683 us */ }
```

Reading more than the specified sixteen bits:

```cpp
const int32_t raw = Sdadc::result24();        // +/- 8388608 is +/- VREF
const int32_t counts = raw / sdadc_raw_per_count;
```

A calibrated channel, in the units the corrections speak:

```cpp
(void)Sdadc::offset_correction(-25600);   // 100 counts of the datum
(void)Sdadc::gain_correction(3);          // x1.5, with...
(void)Sdadc::shift_correction(1);         // ...the power of two under it
```

An automatic sequence over every pair the package bonds:

```cpp
(void)Sdadc::sequence(0x7);
(void)Sdadc::start();                     // ONE start, three results
for (uint8_t i = 0; i < 3; ++i) {
    while (!Sdadc::ready()) { }
    const uint8_t from = Sdadc::sequence_state();
    const int16_t value = Sdadc::result();
}
```

A conversion started by an event, with the result moved by the DMAC:

```cpp
(void)Sdadc::enable(false);               // EVCTRL is enable-protected
(void)Sdadc::start_on(channel, EventChannelConfig{
    .generator = Pacer::overflow_generator,
    .path = EventPath::asynchronous});    // the only path 39.6.6 allows
// ...and a DMA channel on Sdadc::dma_trigger_resrdy reading
// Sdadc::regs().SDADC_RESULT with word beats.
```

A window over a signed result:

```cpp
(void)Sdadc::window(SdadcWindow::outside, -5000, 5000);
Sdadc::arm(Sdadc::flag_winmon);
```

## Errata

DS80000740S, read on the **E/G/J row at revision F**. Four SDADC items
and one device-level item; **one is this silicon**.

- **1.8.10 DAC Output Reference Selection** (device level, ALL
  REVISIONS, B..H): with `REFCTRL.REFSEL = DAC`, starting a conversion
  makes the DAC's OUTPUT voltage noisy. The workaround is
  `REFCTRL.ONREFBUF = 1`, which `sdadc_config_valid()` requires - and
  which 39.8.2's own Note asks for with INTREF as well. **Reproduced
  with a control on both sides** (below).
- **1.8.7 DMA Write Access** (all revisions) names `SDADC: SWTRIG` among
  the registers a SleepWalking DMA write may fail to reach. The
  workaround - use Idle rather than Standby when a DMA channel writes
  SWTRIG - is the application's, and it is stated on `start()`; a driver
  cannot know which sleep is coming.

**NOT this silicon**, and each is the read-the-row trap the errata
document sets over and over:

- **1.18.1** (the APB clock having to be at least twice GCLK_SDADC or the
  first conversion of a sequence is invalid): **revision B only**. Worth
  noting because this stratum runs both at 48 MHz, a ratio of one.
- **1.18.3** (GAINCORR and SKPCNT resetting to zero, so RESULT is zero
  and the first two conversions invalid): **revision B only**, and this
  revision's printed reset values ARE its workaround.
- **1.18.2** (poor INL near VREF; workaround "limit the differential
  input range to +/-0.7 x VREF"): **revisions B..E**. NOTE that table
  45-26 has ABSORBED half of it as a specification - for
  VREF >= VDDANA - 0.3 V the input conversion range IS +/-0.7 x VREF on
  every revision - so a full-scale differential against VDDANA is
  outside specification here whatever the errata say.
- **1.18.4** (power consumption): revisions B..E.

## Bench findings

Board C, ATSAMC21J18A rev F, VDD about 5.15 V (the suite refines it to
5197 mV from the bandgap), wireless. CLK_SDADC 6 MHz off GCLK generator 0
unless stated; times are ruled by the board's 24 MHz **crystal**, since
OSC48M is 5100 ppm slow. Counts are of the SIGNED 16-bit datum
(+/-32768 = +/-VREF) unless the text says "raw", which is the 24-bit
value (+/-8388608 = +/-VREF). Suite `test_samc_sdadc`, `z` 101/101.

### The result, and the width nobody documents

- **THE RESULT IS SIGNED.** A rail-to-rail differential reads
  **0x7FFFFF** one way round and **0x800000** the other, i.e. +32767 and
  -32768 as the specified datum. 39.1 and 39.8.19 are right; 39.6.1,
  39.6.3.1 and 39.6.3.4 are wrong.
- **AND THE REGISTER SATURATES AT +/-2^23, not at a left-shifted
  +/-2^15.** 0x7FFFFF, not 0x7FFF00. So 39.8.19's "left-adjusted" names
  where the SPECIFIED sixteen bits sit and not the width of the datapath.
- **THE EIGHT BITS UNDER THE DATUM ARE REAL FILTER OUTPUT**: on a
  shorted differential at OSR 64 the datum spans 3 counts while the raw
  value spans 760, and at OSR 256 and above the datum is BIT-EXACT over
  64 conversions while the raw value still moves.
- **THE CORRECTIONS ACT IN THOSE RAW UNITS**, which is the campaign's
  central measurement and which 39.6.3.4's "Data0 is an unsigned integer
  defined on 16 bits" denies. On a shorted differential reading -22628
  raw (-89 counts): OFFSETCORR 1000 gives -21456 raw (predicted -21628),
  -1000 gives -23632 (predicted -23628), 8000 gives -14452 (predicted
  -14628) and **25600 gives +2945 raw = +11 counts, i.e. exactly 100
  counts of movement**. With OFFSETCORR 51200 standing, GAINCORR 2 at
  SHIFTCORR 0 gives 57091 raw (predicted 57144), GAINCORR 4 at SHIFTCORR
  1 gives 57497 (the same gain, as it must be), 3/2^1 gives 42817
  (predicted 42858) and 1/2^1 gives 14357 (predicted 14286). **The gain
  really is an integer over a power of two.**
- **GAINCORR = 0 makes every result zero** (span 0), which is erratum
  1.18.3 by arithmetic rather than by accident. The driver refuses it;
  the register was written raw to see it.

### Noise and resolution - the numbers the Multislope work wants

Both pads of a pair driven from PORT to the same rail is an exact analog
zero. Over **64 conversions**, "noise-free bits" = log2(2^24 / raw
peak-to-peak span):

| OSR | raw span | raw rms | noise-free bits of 24 | datum span |
|-----|----------|---------|-----------------------|------------|
| 64 | 1240 | 224 | 14 | 5 |
| 128 | 295 | 55 | 16 | 1 |
| 256 | 151 | 30 | 17 | 0 |
| 512 | 96 | 19 | 18 | 0 |
| 1024 | 79 | 15 | 18 | 0 |

(both pads at GND, reference VDDANA, one raw unit = 157 nV; the high
rail gives 1240 / 335 / 176 / 135 / 88.)

- **THE SPECIFIED SIXTEEN BITS ARE ALL NOISE-FREE ON THIS DIE** from
  OSR 128 up: 64 consecutive conversions of a shorted differential come
  back bit-identical. The noise is only visible in the eight bits below
  them, which is why this document reports it there.
- **THE IMPROVEMENT PER OSR OCTAVE FLATTENS**: the rms falls 224 -> 55
  (2.0 bits) -> 30 (0.9) -> 19 (0.7) -> 15 (0.3). That is a **thermal
  floor**, not a quantization one - a converter whose decimation noise
  has already fallen under its own analog noise by OSR 256.
- At the **1.024 V bandgap**, where one datum count is 31 uV instead of
  157 uV, the same shorted differential spans 3480 / 1105 / 630 / 582 /
  388 raw over the OSR ladder (13 to 16 noise-free bits of 24), and the
  datum spans 14 / 5 / 2 / 2 / 2. Table 45-27's "AC input noise rms
  0.08 mVrms at OSR 256" would be about 2.5 counts at this reference;
  the measured rms of 142 raw is **0.55 counts = 17 uV**, five times
  quieter than the table.
- **THE OFFSET IS COMMON-MODE DRIVEN.** With the chopper on, the same
  zero reads **-89 counts (-13.85 mV) with both pads at GND** and **+108
  counts (+17.03 mV) with both at VDD** - a shift of 197 counts = 31 mV
  for a common-mode step of the whole supply, i.e. a common-mode
  rejection of about 44 dB. The same offsets appear at the 1.024 V
  reference in the same MILLIVOLTS (-13.8 mV), so they are input-referred
  and real. Table 45-27's +/-3.9 mV typical is presumably at a
  mid-supply common mode, which is where the swept measurement below puts
  it: **an intercept of 4 counts, about 0.6 mV**.
- **The chopper moves the offset by a third**: at the low rail, -136
  counts (-21 mV) with `ANACTRL.ONCHOP` clear against -89 counts (-14 mV)
  with it set.

### Linearity, swept with a PWM and the converter's own filter

There is no analog voltage source on this board that reaches an SDADC
pad, so the sweep is made by **TCC1, whose WO0 and WO1 ARE PA06 and
PA07**: two PWM waveforms of 512 GCLK cycles, duties d0 and d1, giving a
mean differential of (d1 - d0)/512 x VDD at a fixed mid-supply common
mode. At OSR 1024 and CLK_SDADC 6 MHz the decimation window is
1024 x 4 x 8 = 32768 GCLK cycles = **exactly 64 PWM periods**, so the
fundamental and every harmonic land on a zero of the third-order SINC.
The source is therefore a duty ratio, whose own linearity is integer
arithmetic.

Fifteen points, k = d1 - d0 from -448 to +448 in steps of 64, so an ideal
converter reads k x 64 counts:

- **Monotonic through zero over the whole sweep**, spread per point
  usually 1..7 counts.
- Best-fit line over |k| <= 320 (eleven points): **slope 65.132 counts
  per k where ideal is 64.000, intercept 4 counts**. That is a **gain of
  +1.7 %** - inside table 45-27's +/-3.4 % maximum, above its +/-1.1 %
  typical - and an **offset of 4 counts (0.6 mV) at mid common mode**.
- **Worst residual against that line: 4 counts of 32768 inside
  |k| <= 320**, where table 45-27 allows an INL of +/-11 LSB at a
  supply-sized internal reference and gives +/-5.3 typical. This is the
  COMBINED nonlinearity of a duty-ratio source and a decimation filter,
  and this document declines to apportion it.
- **THE +/-0.7 x VREF LIMIT DID NOT BITE.** The four points beyond it
  (|k| = 384 and 448, i.e. 0.75 and 0.875 of VREF) leave the same line by
  at most **10 counts** - two and a half times the inner residual, not
  the collapse a modulator overload would give. Erratum 1.18.2, which
  asked for exactly that restriction, is marked revisions B..E and NOT
  this one; table 45-26 keeps it as a specification. Recorded as an
  observation on one die at one OSR.

### Time, crystal-ruled

- **The free-running period is OSR x 4 CLK_SDADC cycles, exactly**:
  measured 1029 / 4116 / 16464 crystal ticks a result at OSR 64 / 256 /
  1024 against 1024 / 4096 / 16384 predicted - i.e. 42 / 171 / 686 us.
  The uniform +0.49 % is OSC48M being 5100 ppm slow against the crystal
  that timed it, which is the two clocks agreeing rather than the
  converter disagreeing.
- **THE PRESCALER IS LINEAR.** PRESCALER 3 / 4 / 7 / 23 give periods of
  1029 / 1286 / 2058 / 6173 ticks, i.e. ratios of **1000 / 1249 / 2000 /
  5999** against 2 x (P + 1)'s 1000 / 1250 / 2000 / 6000. The device
  header's power-of-two enumerators would predict 1000 / 2000 / 16000
  and are simply the SAM D21's.
- **SKPCNT costs a whole decimation window each.** Single conversions at
  SKPCNT 2 / 3 / 5 / 9 take 3784 / 4818 / 6881 / 10987 ticks, i.e.
  differences of 1034 / 3097 / 7203 against a window of 1024 - so a
  single conversion takes (SKPCNT + 1) windows and **table 45-26's
  single-conversion row multiplies the output rate where it should divide
  it**.
- **39.6.2.3's "the first valid sample is the third" is literal.** With
  SKPCNT written raw, the same full-scale differential reads **5478 at
  SKPCNT 0, 27623 at 1 and 32767 at 2** - the SINC filter's step response
  caught in the act, and the reason the driver refuses fewer than two.

### The references

A fixed differential (a PWM at k = 64, about 640 mV) read against four
references at OSR 1024:

| reference | counts | spread | implied |
|-----------|--------|--------|---------|
| INTREF 1.024 V | 20918 | 16 | 654 mV |
| INTREF 2.048 V | 10523 | 12 | 658 mV |
| INTREF 4.096 V | 5270 | 2 | 659 mV |
| VDDANA | 4172 | 3 | 656 mV |

- **The same input measured against four different references agrees to
  0.8 %.** The bandgap's three levels come out a factor of two apart with
  the input cancelling out: **1987 and 1996 per mille** against 2000
  nominal.
- **THE CROSS-CHECK BETWEEN THE TWO ARCHITECTURES.** There is no DC
  voltage both converters can see, so the shared quantity is a RATIO: the
  SDADC's own readings put 4.096 V / VDDANA at **791 per mille**, and the
  SAR ADC, converting the same bandgap as an INPUT against VDDANA, puts
  it at **788 per mille** (3228 counts of 4096, i.e. VDDANA = 5197 mV).
  **Three parts in a thousand**, between a sigma-delta's reference
  multiplexer and a successive-approximation converter's input
  multiplexer, sharing nothing but the bandgap.
- **REFSEL = VREFB reads the pin.** With PA04 driven to the supply from
  PORT, the same differential reads **4171 counts against VDDANA's
  4172** - one count apart, the external and the internal path to the
  same voltage. Driven to GND (below table 45-26's 1 V minimum) the
  reading saturates, as a reference of nothing must.

### Erratum 1.8.10, reproduced with a control

The instrument is the one the DAC campaign built for erratum 1.8.9: the
SAR ADC watching the DAC's own VOUT pad, whose spread is the
disturbance. Spread of 64 SAR readings of 4096, with the DAC at
mid-code:

| arrangement | spread |
|-------------|--------|
| SDADC stopped | 1 |
| SDADC converting, REFSEL = DAC, ONREFBUF **0** | **108** |
| SDADC converting, REFSEL = DAC, ONREFBUF **1** | 1 |
| SDADC converting, REFSEL = VDDANA (the control) | 1 |

The fourth row is what makes it the erratum and not "the SDADC is
noisy": the same converter running just as hard against the supply
leaves the DAC's pad alone. **`REFCTRL.ONREFBUF` is the workaround it
claims to be** - it takes 108 counts of disturbance (about 135 mV) back
to 1. The driver refuses the configuration that produced the middle row,
so the bit was cleared by hand to measure it.

### The rest of the chapter

- **The four window modes behave as 39.8.11 prints them** on a signed
  result, with the thresholds placed in the register the way RESULT
  reports the datum (`sdadc_threshold_word()`), and turning the monitor
  off stops it firing.
- **OVERRUN** sets when a free-running converter writes RESULT before
  the previous value was read.
- **THE NO-CPU CHAIN RUNS BOTH WAYS AT ONCE**: a TC2 overflow crosses an
  asynchronous EVSYS channel into the SDADC's START user, each RESRDY
  pulls one 32-bit DMA beat out of RESULT, and the same RESRDY crosses a
  second channel into TC3 counting them. 16 of 16 results land with the
  polarity the pads held, at both polarities, with 58 and 59 result-ready
  events counted in the same 60 ms window; with the pacer stopped nothing
  moves at all. A SYNCHRONOUS channel into the same user is refused, as
  39.6.6 requires.
- **THE SEQUENCER RUNS THE WHOLE LIST FROM ONE START.** With all three
  pairs enabled and a different differential on each (+VDD, -VDD, zero),
  one `start()` gives **32767 / -32768 / -86 with SEQSTATE 0 / 1 / 2** -
  the order 39.6.2.7 prescribes, each result labelled with the input it
  came from, and SEQBUSY standing from the START until the last
  conversion is done. A one-input sequence converts THAT input whatever
  MUXSEL says, and zero in SEQCTRL gives the multiplexer back. **A
  sequence is over in 385 us at OSR 64**, which is shorter than one
  console line at 115200 - the suite prints nothing between the START and
  the third result, and the first version of the letter that did lost the
  whole sequence.
- **Three differential pairs are three independent inputs**, each
  following its own two pads.
- **The documentary disputes, settled by writing registers raw under a
  running converter**: REFCTRL, CTRLB and EVCTRL discard the write
  (enable-protected, as their property lines say), CTRLC and ANACTRL take
  it (so 39.6.2.1's list is two entries too long); `REFCTRL.REFRANGE`
  stays written (0x33 read back with REFSEL 3 and REFRANGE 3), so the
  field the chapter never mentions is real; `ANACTRL` written 0xFF reads
  back **0xFF**, so its bias field is six bits as the header says and not
  five as 39.8.21's bit table draws; and `DBGCTRL` survives a software
  reset, as 39.8.22 says.
- **The reset values are erratum 1.18.3's own workaround**: out of a
  software reset CTRLB reads 0x2000 (SKPCNT 2) and GAINCORR reads 1.

## Not covered yet

Driver gaps:

- **Sleep.** `CTRLA.RUNSTDBY` and `CTRLA.ONDEMAND` are written and read
  back; table 39-1's four combinations have never been entered, the
  converter has never been a wake source, and erratum 1.8.7's
  SleepWalking obligation on SWTRIG is stated and unexercised.
- **`ANACTRL.CTLSDADC` and `ANACTRL.BUFTEST`.** The first is
  "Debug/Characterization" with no values given, the second has no
  description at all. Both are writable through `SdadcConfig`, both were
  proven to stay written, and NOTHING here says what either does.
- **`REFCTRL.REFRANGE`** is exposed and proven to be a real field, and
  no document of record says what it selects. No measurement has looked
  for an effect.
- **The interrupts through the NVIC.** All three flags are read, cleared
  and used as verdicts; none has ever driven a vector, and `isr()` is
  compile-verified only.
- **`EVCTRL.FLUSHEI` and `SWTRIG.FLUSH`.** Both are implemented,
  `flush_on()` refuses a synchronous channel like `start_on()`, and no
  bench letter has flushed a conversion in flight.
- **`EVCTRL.WINMONEO`.** The window monitor's OUTPUT event is written and
  never routed anywhere.
- **The E and G packages** are compile-only, as everywhere in this
  stratum: only the J has a board.

Implemented but not bench-verified:

- **`DBGCTRL.DBGRUN`** is written and its survival across a software
  reset measured; its effect under a halted debugger is untested.
- **`util/analog_sampler.hpp`** has NOT been given this converter. Its
  `SamplerInput` concept wants a `void select()` and an unsigned reading,
  and this converter's `select()` is a `bool` (39.6.8's bus error) and
  its result is signed - so the adaptation is a real design question and
  not a five-line change. It is named here, not attempted.
- **A DC linearity measurement.** Everything in "Linearity" above is
  swept with a PWM averaged by the converter's own filter, which is a
  legitimate instrument and is not a DC source. INL, DNL and the gain
  error of table 45-27 to their own conditions need a voltage this board
  cannot make.
- **The external reference at anything but a rail.** `SdadcRef::vrefb`
  is proven to read PA04, driven to VDD and to GND. Table 45-26's
  1 V .. VDDANA range in between needs a wire.
- **Erratum 1.8.10's reading half.** The item speaks only of the DAC's
  OUTPUT going noisy, and that is what was measured; whether the SDADC's
  own reading suffers with the buffer off was not isolated.
- **The bus error of 39.6.8.** Every synchronized write in the driver
  waits before storing, precisely so the threatened HardFault cannot
  happen; nothing here has deliberately provoked one, so whether the
  silicon really raises it - or discards silently like the ADC's and the
  DAC's chapters - is unknown.
