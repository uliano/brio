# DAC - Digital-to-Analog Converter (SAM C21)

> **PROVISIONAL.** The whole of chapter 41 is implemented and most of it
> is bench-verified through the ADC and the AC on the same die. What is
> NOT verified is dithering (built, refused without the start event the
> chapter requires, never run), the external reference VREFA (the pin is
> on PA03 and nothing on this board drives it within table 45-30's
> range), and the voltage pump, which switches itself at a supply this
> board never visits. The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 41, the DAC
characteristics of tables 45-30, 45-31 and 45-32, and the event tables
of ch. 29 - and errata DS80000740S, read on the **E/G/J row at revision
F**: the device-level items 1.8.9 (the DAC output as the ADC's positive
multiplexer input) and 1.8.10 (as the SDADC's reference) and the DAC's
own 1.9.2 (the EMPTY flag across a standby) are live, while 1.9.1
(dithering with right-adjusted data) is revision B only. Driver:
`samc/dac.hpp`, over the reserve's DAC entries in
`samc/device_tables.hpp`. The family fixture is
`test/family_samc/dac.cpp` plus five negatives under
`tools/check_samc.sh`; the bench suite is `test_samc_dac`.

## What the silicon does

**One channel, ten bits, up to 350 ksps**, converting DATA into a
voltage between GND and the selected reference: VOUT = DATA / 0x3FF x
VREF (41.6.2.4). One instance on every C21 variant and none on a C20,
which is why `Dac` is a monostate resource and not a template.

**Two ways to start a conversion** (41.6.1): write DATA, or let a START
event copy DATABUF into DATA. **Two outputs**, independently enabled and
usable together (41.6.8.1): the high-drive buffer on the VOUT pad
(`CTRLB.EOEN`) and the internal path to the AC, the ADC and the SDADC
(`CTRLB.IOEN`).

**There is no conversion-done flag.** 41.6.2.4 says so outright: "as
there is no automatic indication that a conversion is done, the sampling
period must be greater than or equal to the specified conversion time".
`STATUS.READY` is about the startup time (3 us, table 45-31) and nothing
else.

**The pads.** VOUT is PA02 and the external reference VREFA is PA03, on
every variant, both under peripheral function B. On this package PA02 is
also **ADC0/AIN0 and the AC's AIN4**, which is what makes the whole
bench suite wireless - and what makes erratum 1.8.9's "wire the DAC VOUT
pin externally to an ADC AINx pin" a wire of zero length.

### The four register disciplines

The chapter mixes them, and it disagrees with itself about one:

1. **Enable-protected** - CTRLB, and 41.6.2.1 also names EVCTRL while
   41.8.3's own property line does not. **Measured: EVCTRL IS
   enable-protected** - a raw write under a running converter reads back
   zero - so 41.6.2.1's list is right and the register description's
   property line is missing a property.
2. **Write-synchronized** - CTRLA.SWRST, CTRLA.ENABLE, DATA, DATABUF
   (41.6.7). Nothing needs synchronization when read.
3. **Neither** - EVCTRL, INTENSET/CLR, INTFLAG, STATUS, and DBGCTRL,
   which 41.8.11 also says is not reset by a software reset.
4. **Write-only** - DATA and DATABUF are `__O` in the device header and
   drawn with W access in 41.8.8 and 41.8.9. **There is no code
   readback**, which is why `code()` reports what the driver last wrote
   and says so.

INTFLAG and DATABUF are the two registers PAC write-protection does not
cover (41.5.8) - the pair a DMA engine needs.

### The reference, and a name that lies in the device header

`CTRLB.REFSEL` has three codes. 41.8.2 calls them INTREF ("supplied by
the bandgap, refer to SUPC.VREF.SEL for voltage level information"),
VDDANA and VREFA. The device header names the same three values
**INT1V**, AVCC and VREFP - INT1V being the SAM D21's fixed 1.0 V
internal reference, which this family does not have. `DacRef` follows
the datasheet for meaning and the header for symbols, and the bench
settles it (below).

### Where the data sits in the register

Two bits decide it together (table 41-1), which is why
`dac_data_word()` exists and no caller shifts by hand:

| DITHER | LEFTADJ | the 10-bit code | the dither bits |
|---|---|---|---|
| 0 | 0 | DATA[9:0] | - |
| 0 | 1 | DATA[15:6] | - |
| 1 | 0 | DATA[13:4] | DATA[3:0] |
| 1 | 1 | DATA[15:6] | DATA[5:2] |

**Dithering is not a mode a CPU-driven converter can use.** 41.6.8.3
requires a periodic START event generating sixteen events per value,
with DATABUF reloaded every sixteen; `dac_config_valid()` refuses
dithering without `EVCTRL.STARTEI` for that reason.

## Types and verbs

`DacRef` is **this converter's own** reference vocabulary, deliberately
not `brio::Ref`, which `samc/adc.hpp` owns for REFCTRL.REFSEL: on this
family there is no shared reference block at all - the ADC's
multiplexer has six codes, this one has three, the SDADC's are different
again - so one enum would be a type no register accepts. `dac_ref_mv()`
is its millivolt table, taking VDDANA or the pin's voltage from the
caller and `samc/supc.hpp`'s `VrefLevel` for INTREF.

`Dac` is the resource: `init(generator, cfg)` and its compile-time twin
`init<cfg>(generator)`, `enable`, `reset`, `release`; `set`, `set_mv`,
`code`, `code_mv`; `buffer`, `buffer_sync`, `buffer_pending`; `ready`,
`wait_ready`; `control_b`, `external_output`, `internal_output`;
`arm` / `disarm` / `flags` / `clear_flags` / `empty` / `underrun` /
`isr`; `event_config`, `start_on`, `stop_events`; `claim_vout<P>()`,
`claim_vrefa<P>()`, `release_pad<P>()`, `vout_function`,
`vrefa_function`.

`DacConfig` carries the whole configuration and `dac_config_valid()` is
the one place every rule of the chapter is written down.
`dac_data_word()` is table 41-1 as arithmetic, clamping rather than
spilling into the neighbouring field.

The EVSYS and DMAC vocabularies this peripheral publishes -
`empty_generator`, `start_event_user`, `dma_trigger_empty` - live here
and not in `evsys.hpp` or `dmac.hpp`, per the ruling in
[evsys.md](evsys.md): those files own the fabric and the channels, a
peripheral owns its own codes. `adc_input_pad_port` /
`adc_input_pad_pin` publish the pad erratum 1.8.9's workaround wants, so
a caller does not have to know the pinout.

**Millivolts go through `util/analog.hpp`**, unchanged from the AVR:
`set_mv()` is a thin wrapper over `dac_code(mv, 1024, ref_mv)`. Note the
half-truth: 41.6.2.4's own formula divides by 0x3FF (1023), so the two
conventions differ by mv/ref of one LSB - nothing at the bottom of the
range and one whole LSB at full scale. Below full scale that is smaller
than this converter's own gain error, which is why the bench declines to
decide between them.

**The linear range is not the whole range.** Table 45-30 gives it as
0.05 V .. VDDANA - 0.05 V, so at a 5 V reference roughly the bottom ten
and the top ten codes are outside specification, and a straight line
fitted through them is fitted through a clipped end.

### One example per use

A voltage on the pad:

```
using Out = brio::Dac;
Out::init(0, brio::DacConfig{.reference = brio::DacRef::vddana,
                             .external_output = true});
Out::set(512);              // about VDDANA / 2
Out::set_mv(2500, vdd_mv);  // or in millivolts
```

A threshold for the comparator, with no pad taken at all:

```
brio::Dac::init(0, brio::DacConfig{.internal_output = true});
brio::Dac::set(256);
brio::AcComparator<0>::configure({.positive = brio::AcPositive::pin0,
                                  .negative = brio::AcNegative::dac});
```

A waveform with no CPU in the path - a timer overflow starts each
conversion and the DMAC refills the buffer:

```
brio::Dac::init(0, cfg_with_empty_event);
brio::Dac::enable(false);                       // EVCTRL is enable-protected
brio::Dac::start_on(channel, {.generator = Pacer::overflow_generator,
                              .path = brio::EventPath::asynchronous});
// DMA channel: trigger = brio::Dac::dma_trigger_empty, one beat per
// request, source = the table in RAM, destination = &DAC_REGS->DAC_DATABUF
brio::Dac::enable(true);
```

## Errata

Read on the **E/G/J row at revision F** - the row, not the column, which
is the trap this document set repeatedly.

- **1.8.9 DAC Output** (device level, every revision): selecting the DAC
  output as the ADC's positive multiplexer input makes both the DAC
  output and the reading noisy. The workaround is to "wire the DAC VOUT
  pin externally to an ADC AINx pin input" and select that pad. **On
  this silicon that wire has zero length**: PA02 is DAC/VOUT and
  ADC0/AIN0 at once, so `CTRLB.EOEN` plus `AnalogIn<Pin<'A',2>>` on ADC0
  IS the workaround. Measured below.
- **1.9.2 Standby Sleep Mode** (every revision): with
  `CTRLA.RUNSTDBY = 0` and DATABUF written but not yet consumed,
  entering standby sets `INTFLAG.EMPTY` on the way out. The workaround
  is the caller's - ignore and clear EMPTY after a standby wake - and a
  driver cannot know that a wake happened, so the obligation is stated
  on `empty()`. **Reproduced**, with its own control, below.
- **1.8.10 DAC Output Reference Selection** (every revision) is live on
  this row and belongs to the OTHER side of the same pair of pins: with
  the SDADC converting against `REFCTRL.REFSEL = DAC`, this DAC's own
  output goes noisy. **Reproduced with a control** by the SDADC campaign
  (a pad spread of 1 count becoming 108 with the reference buffer off,
  and 1 again with it on or with the SDADC running against VDDANA) - see
  [sdadc.md](sdadc.md). Its workaround, `REFCTRL.ONREFBUF = 1`, is
  enforced in `samc/sdadc.hpp`, not here.
- **NOT this silicon**: 1.9.1 (dithering with right-adjusted data giving
  16 LSB of INL) is **revision B only**, so `left_adjust` is not forced
  here.

## Streaming via DMA

A waveform is `DmaLoopEngine<ch, uint16_t>` armed on
`Dac::dma_trigger_empty` with DATABUF as its destination: one
caller-owned table, played for ever, one interrupt per LAP and none per
sample. The contract, the hardening and the shape of the handler are in
[dmac.md](dmac.md); two things belong to THIS chapter.

**The pacing is a START event and not a rate.** 41.6.2.4 gives the
converter no done flag, so nothing in the data path decides when the
next value is due - a periodic event into the START user does, and the
DMA request is only what refills DATABUF behind it. Without a start
event the first DATABUF write stands for ever (see SYNCBUSY.DATABUF
above) and the stream never moves at all.

**INTFLAG.EMPTY is an EVENT, not a state**, and a DMA-fed DAC is where
that bites: on a converter just enabled, whose DATABUF has never been
written, EMPTY reads **zero** even though the buffer is empty - the flag
marks the buffer BECOMING empty. So an owner that waits for the flag
before giving the channel its first software trigger never starts, and
pays an UNDERRUN and one lost period to discover it. What makes that
first `kick()` right is the owner's own knowledge that it has just reset
the converter, not the flag. Measured in `test_samc_analog_dma`.

Streamed at 5 kHz against ADC0 on the shared PA02 pad, a 32-entry table
came back **exact to 3..6 ADC counts** - the converter pair's own noise
floor, against a table step of 120 - with no sample lost at any lap
boundary over thousands of laps.

## Bench findings

Board C, ATSAMC21J18A rev F, VDD about 5.15 V, wireless. Voltages are
ADC counts of 4096 against VDDANA unless stated; times are ruled by the
board's 24 MHz crystal.

- **THE ANALOG OUTPUT REACHES THE PAD WITHOUT THE PIN MULTIPLEXER.**
  Mid-code on PA02 reads 2030 counts with the pad left under PORT and
  2031 with the DAC's peripheral function claimed - one count apart. The
  buffer is connected by `CTRLB.EOEN`, not by PMUX; 41.5.1's "configure
  the I/O pins using PORT" is a recommendation about the digital
  receiver, not a precondition for the output.
- **THE LOOP, END TO END.** Codes 0 / 256 / 512 / 768 / 1023 read
  **0 / 1010 / 2031 / 3057 / 4074** counts of 4096. Code 0 is at the
  ADC's own zero, code 1023 within half a per cent of full scale, and
  the quarter points land within one per cent - which is the two
  converters' combined offset and gain error, not their linearity. Over
  64 readings at mid-code the spread is **1 count**.
- **`util/analog.hpp` NEEDED NO CHANGE.** `set_mv(2000)` at a 5150 mV
  reference writes code 398 and the ADC reads **1982 mV** back through
  `adc_mv()` - 18 mV apart, with the DAC's own quantization at 5 mV a
  code and the pair's gain error dwarfing both.
- **THE DEVICE HEADER'S `INT1V` NAME IS THE SAM D21'S AND IS WRONG
  HERE.** With `REFSEL = INTREF` and the SUPC bandgap at its three
  levels, DAC code 1000 reads 775 / 1564 / 3154 counts, implying full
  scales of **996 / 2011 / 4057 mV** against nominal 1024 / 2048 / 4096.
  The reference follows `SUPC.VREF.SEL`: 41.8.2 is right and the
  header's enumerator name is stale.
- **THE DAC'S REFERENCE PATH DOES NOT NEED `SUPC.VREF.VREFOE`.** Same
  level, bit clear then set: 1564 counts against 1564. That is the
  opposite of what the ADC's bandgap INPUT channel needs (a flat zero
  without the bit, [adc.md](adc.md)) - the reference multiplexer takes
  the bandgap internally.
- **AND NEITHER DOES THE ADC'S**, which is the question
  [adc.md](adc.md) could not answer without a steady mid-range source.
  A 1479 mV DAC level read against
  `REFSEL = INTREF` at 2.048 V gives **2991 counts with VREFOE clear and
  2990 with it set**, i.e. 1496 mV against the 1479 mV VDDANA said. The
  bit is genuinely not in the reference path.
- **`Ref::dac` CONVERTS, AND IT IS RATIOMETRIC.** The quarter-scaled
  analog supply read against a DAC reference at codes 512 / 768 / 1000
  gives **2045 / 1358 / 1040** counts, against 2045 / 1363 / 1047
  predicted by scaling the first as one over the code - under half a per
  cent, with the supply cancelling out of both sides. Absolutely, it
  puts 1/4 VDDANA at **1286 mV** where the supply says 1287.
- **`AdcInput::dac` READS THE DAC**, which 38.8.9's table (marking
  0x1C..0x1F Reserved) says should not exist and the device header says
  should. Code 768 reads **3057** counts through the internal channel
  and **3056** through the pad - the same voltage by two routes.
- **41.6.8.1 SAYS THE OUTPUT BUFFER MUST BE ENABLED FOR THE ADC TO SEE
  THE DAC, AND ON THIS SILICON IT NEED NOT BE**: with `IOEN` alone and
  `EOEN` clear the internal channel reads 3057, the same as with both.
- **ERRATUM 1.8.9'S OUTPUT NOISE IS REAL AND LARGE, AND IT IS THE
  SELECTION AND NOT THE TRAFFIC.** ADC0 watching the pad spreads **3..5
  counts** with ADC1 silent, **71..87 counts** with ADC1 free-running on
  `MUXPOS = DAC`, and **3..4 counts** with ADC1 free-running just as hard
  on another input. That control is what makes it the erratum rather
  than crosstalk: about 100 mV of disturbance appears on the DAC's own
  output while a converter samples it internally.
  The other half of the item - a noisy READING on the internal channel -
  is **declined**: over two interleaved rounds of 128 readings the
  internal channel spreads 3..5 counts and the pad path 4..5, which is
  not decisively apart on a board this quiet. A converter sampling the
  DAC internally does not see the disturbance it causes.
- **`AcNegative::dac` IS REAL, AND THE TWO LADDERS AGREE.** With COMP0's
  positive input on its own 64-step VDD scaler and its negative on the
  DAC, the comparator flips at DAC code **255 / 512 / 769** for scaler
  steps 15 / 31 / 47, against 255 / 511 / 767 predicted. The gaps are
  **257 and 257** codes against 256 predicted - and differencing them
  cancels the comparator's own offset, so that pair of numbers is the
  two dividers' linearity and nothing else.
- **THE TRANSFER CURVE, AND WHAT IT CAN CLAIM.** 31 points from code 32
  to 992 (the ends left out: table 45-30's linear range stops 50 mV from
  each rail), ADC0 with 64x averaging. The transfer is **monotonic**;
  the best-fit slope is **3995 ADC counts per 1000 DAC codes** where an
  exact 4096/1023 would be 4004 and an exact 4096/1024 would be 4000;
  the **worst residual from that line is 2.5..2.9 ADC counts**, about
  0.7 DAC LSB. That residual is the COMBINED nonlinearity of two
  converters in series and this suite does not apportion it: table 45-32
  allows the DAC +/-0.6 LSB (2.4 ADC counts) and table 45-24 allows the
  ADC +/-4. **The slope also settles nothing about the 1023-vs-1024
  question**: the pair's gain error is bigger than the 0.1 % between the
  two conventions.
- Outside the fit, code 1023 reads 4074 where the line predicts 4074 and
  code 0 reads 0 where it predicts -13. There is no visible clipping at
  the top; table 45-30's linear-range limit is about accuracy, not about
  the output stopping.
- **THE STARTUP IS ABOUT SIX MICROSECONDS**: ENABLE to `STATUS.READY`
  measures **141..143 crystal ticks = 5.9..6.0 us**, best of eight, with
  the ENABLE write's own synchronization inside that number. Table 45-31
  gives 3 us for the startup alone.
- **A FULL-SCALE STEP CROSSES MID-SUPPLY FASTER THAN THIS BOARD CAN
  TIME IT.** A single-shot timing is useless here - two synchronized
  stopwatch reads cost 95..98 ticks (4 us) before the DAC does anything
  - so the measurement is a difference: 1000 crossings in a loop that
  waits for the comparator take 21800 ticks against 18950..19460 for the
  same loop waiting for nothing, which is **97..119 ns a crossing**
  against a **166 ns** cost for one poll of the comparator. So the
  answer is an UPPER BOUND of a few hundred nanoseconds - an order of
  magnitude under the 2857 ns table 45-31's 350 ksps conversion rate
  implies, which is a rate and not a settling time.
- **THE NO-CPU CHAIN RUNS.** A TC2 overflow crosses an asynchronous
  event channel into the DAC's START user, which copies DATABUF into
  DATA and converts; DATABUF going empty pulls the next value out of a
  table in RAM through the DMAC; and the same EMPTY crosses a second
  channel into TC3, which counts it. The pad walks between both levels
  of the waveform (0..3585 counts) while TC3 counts 39..40 EMPTY events,
  the DMA block completes, and **with the pacer stopped the output holds
  still** (spread 1 count) - the event really is the only thing starting
  a conversion. A **SYNCHRONOUS** channel into that user is refused by
  the driver, as table 29-3 requires.
- **UNDERRUN IS WHAT 41.6.4 SAYS IT IS**: with the DMA block exhausted
  and the pacer still running, a start event finds DATABUF empty and the
  flag sets. It exists only in the event-driven shape.
- **`SYNCBUSY.DATABUF` IS NOT A BUS CROSSING, and the chapter does not
  say so.** One DATABUF write with no start event configured leaves
  SYNCBUSY reading **0xC - DATABUF *and* DATA - and it stays there**;
  while it stands every later write to either register is discarded
  (41.6.7), so a DAC fed a value nothing will take is stuck until a
  start event or a software reset. That is why `buffer()` is a plain
  void store that never waits and `buffer_sync()` is the separate verb -
  the same shape, for the same reason, that `samc/adc.hpp`'s `select()`
  has over its double-buffered INPUTCTRL.
- **ERRATUM 1.9.2 REPRODUCES, with a control on both sides.** With
  `RUNSTDBY = 0`, DATABUF full and the flag cleared: half a second
  **awake** leaves EMPTY clear, the same half second **in standby**
  comes back with EMPTY set, and the same standby with **`RUNSTDBY = 1`**
  leaves it clear. The item's own precondition and its complement, both
  observed.
- **ERRATUM 1.4.10 (the ADC's) REPRODUCES, and it is worse than its own
  sentence** - a finding that belongs to [adc.md](adc.md) and is
  recorded here because this is the suite that provoked it. Once ADC1
  has been enabled in a power cycle, `ADC0.SYNCBUSY.ENABLE` is stuck at
  one and stays stuck; ADC0 then **does not enable at all** - a plain
  `Adc<0>::init()` returns false and the converter reads zero - and a
  software reset does not clear it (SWRST's own bit joins the stuck
  one). The errata's order is the way out: bring ADC1 up FIRST and ADC0
  second, after which ADC0 converts correctly and keeps converting even
  when ADC1 is released again. The bench suite spends that workaround
  visibly, printing a line each time.

**41.6.6 through a standby.** The DAC's own pad, read by ADC0 on PA02
- the zero-length wire erratum 1.8.9's workaround asks for - holds 2030
counts of 4096 at code 512 before a standby and the same 2030 after,
with CTRLA.RUNSTDBY set. **Erratum 1.9.2 reproduces with its own
control**: with RUNSTDBY CLEAR and DATABUF written and unconsumed,
INTFLAG.EMPTY comes back SET from the sleep; with RUNSTDBY set, in the
same window with the same buffer write, it does not. See
[platform.md](platform.md), "Sleep, peripheral by peripheral".

## Not covered yet

Driver gaps:
- **Dithering** (41.6.8.3): `DacConfig::dither` is written, table 41-1's
  14-bit placement is implemented and fixture-pinned, and the
  configuration is refused without the start event the chapter requires
   - but no sixteen-event sequence has ever run. It needs a pacer at a
  rate chosen against the conversion time and a witness slower than the
  dither period.
- **VREFA** (`DacRef::vrefa`): the pin is PA03 and `claim_vrefa<P>()`
  hands it over, but nothing on this board drives it, and driving it
  from PORT would put it at VDD - outside table 45-30's
  1 V .. VDDANA - 0.6 V range - so the answer would be out of
  specification either way. Needs a wire.
- **The voltage pump** (`voltage_pump_disabled`): the bit is written and
  read back. The pump switches itself by supply voltage and this board
  sits at 5.15 V, where it is off, so disabling it changes nothing that
  can be observed here. 41.6.8.3's "GCLK_DAC at least four times the
  sampling rate" is stated and unenforceable - the header knows neither
  rate.
- **The DAC as a WAKE source**: EMPTY and UNDERRUN have never driven
  the NVIC out of a sleep. (41.6.6's promise about the output buffer IS
  measured - see "Bench findings".)
- **`CTRLB.LEFTADJ`** is implemented in `dac_data_word()` and
  fixture-pinned; no bench letter writes a left-adjusted value.
- **The interrupts.** `EMPTY` and `UNDERRUN` are read, cleared and used
  as verdicts; neither has ever driven the NVIC, and `isr()` is
  compile-verified only.
- **The SDADC's use of `CTRLB.IOEN` as a REFERENCE** (41.6.8.1's third
  consumer) is now built and measured on the other side -
  [sdadc.md](sdadc.md) - including erratum 1.8.10. What is still not
  measured here is what the DAC's output does electrically while the
  SDADC loads it: the disturbance was seen through the ADC watching the
  pad, and the load itself was not characterised.

Implemented but not bench-verified:
- **`DBGCTRL.DBGRUN`** is written and read back at the offset the device
  header gives (0x14, against the register summary's 0x18 - the header
  wins, measured); its effect under a halted debugger is untested.
- **The 1023-vs-1024 scaling question**: `set_mv()` uses
  `util/analog.hpp`'s 1024-step convention while 41.6.2.4's formula
  divides by 1023. The difference is at most one LSB, reached only at
  full scale, and it is smaller than the converter's own gain error -
  so this board cannot decide it and does not pretend to.
- **A settling time to a stated accuracy.** What is measured is a
  large-signal crossing of mid-supply, bounded above by the comparator's
  polling resolution. Settling to 1 LSB needs a faster instrument than
  a CPU poll.
