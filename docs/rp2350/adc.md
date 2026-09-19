# ADC (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 12.4 whole
(12.4 the overview with the two connection diagrams, 12.4.1 what changed
from the RP2040, 12.4.2 the controller with tables 1118 and 1119 - the
channel map of each package -, 12.4.3 the SAR converter and its clock,
12.4.3.1 the one-shot, 12.4.3.2 the free run and the pacing divider,
12.4.3.3 the round-robin, 12.4.3.4 the FIFO, 12.4.3.5 the DMA, 12.4.3.6
the interrupt, 12.4.3.7 the supply, 12.4.4 and 12.4.5 the figures of
merit, 12.4.6 the temperature sensor, 12.4.7 the registers), 6.1.5 (the
ADC supply, which is also its reference), 14.9 table 1438 (the input
voltage range and the ENOB), 8.1.3 and 8.6 (clk_adc and the USB PLL),
9.7 and 9.11.3 (the pad, its isolation latch and its pulls), 12.6.4.1 (the
DREQ table); Appendix E carries no erratum of this block, and RP2350-E9
is the GPIO one this chapter meets at the pad. `util/analog.hpp` and
`util/analog_sampler.hpp` for the arithmetic and the sampler. The
driver: `brio/rp2350/adc.hpp` (`Adc` the resource, `AnalogIn<Pin>`,
`AdcInput`, `Ref`) over `clock.hpp` (`PllUsb`, `Clocks::adc_select`),
`pin.hpp` (`analog`), `resets.hpp`, `sysinfo.hpp` and `dma_engine.hpp`.
The reference suite: `test_rp2350_adc`, wireless.

## What the silicon does

One 12-bit successive-approximation converter behind an input
multiplexer, a conversion of 96 cycles of the converter's own clock
(clk_adc, which the chapter asks for at 48 MHz: 500 ksps, and which only
the USB PLL makes from a crystal). A one-shot start; a free run that
starts a conversion every 1 + INT + FRAC / 256 cycles under a 16.8
divider with a first-order delta-sigma fraction, the converter ignoring
a start that arrives while it converts; a round-robin over any set of
the channels in the free run, AINSEL naming the first and moving after
every conversion; an eight-entry FIFO with a threshold that is the one
interrupt and the one DMA request, an optional shift of each entry to
the result's top eight bits (a byte buffer), an optional error flag per
entry (bit 15: the comparator failed to converge, a sample to discard);
a sticky overflow flag when a conversion completes into a full FIFO. The
temperature sensor is a diode's Vbe, 706 mV at 27 C with -1.721 mV per
degree, its bias switched on apart from the converter and worth about
40 uA. The block sits behind the subsystem reset controller
(`ResetBlock::adc`), and `Adc::init()` CYCLES that line rather than
merely releasing it: a processor reset on this chip leaves every
peripheral as the previous image left it.

THREE THINGS ARE NOT THE RP2040'S, and the first is the one a program
feels.

- **The package decides the input map, and with it the sensor's channel
  number.** The QFN-60 brings out four inputs on GPIO 26..29 as AINSEL
  0..3 with the sensor on AINSEL 4 - the RP2040's arrangement exactly -
  and THE QFN-80 BRINGS OUT EIGHT, on GPIO 40..47 as AINSEL 0..7, with
  the sensor on AINSEL 8 (tables 1118 and 1119). So AINSEL is four bits
  wide here against three, and RROBIN nine against five. The register
  description of AINSEL states that the field "is corrected for the
  package option so only ADC channels which are bonded are available,
  and in the correct order", which is what makes each package's
  numbering contiguous with no hole in it. This stratum reads the
  package from its reserve (`device.hpp`), so the pad table, the
  sensor's code and the round-robin's width are compile-time facts and
  a pad the package has not got is a compile error. Since the die is
  one and an image may be built without stating a package, `Adc::init()`
  reads SYSINFO.PACKAGE_SEL and REFUSES when the silicon disagrees with
  the build: there is no verb that could work around a sensor sitting
  on a channel number the image does not believe in.
- **The precision.** Erratum RP2040-E11's differential-nonlinearity
  spikes at codes 0x200, 0x600, 0xa00 and 0xe00 are gone, "improving
  the ADC's precision by around 0.5 ENOB" (12.4.1), where the RP2040's
  figure was 8.7 bits. The sheet gives two numbers for what is left:
  12.4's own feature list says 9.2 ENOB, table 1438 says 9 minimum and
  9.5 typical, and 12.4.5 says the integral and differential
  nonlinearity are still to come. Appendix E has no ADC erratum at all
  - and the
  erratum numbered E11 in THIS chip's sheet is an XIP one, which is why
  nothing here cites an erratum by its bare number.
- **There is no ADC_VREF pin.** The converter's supply IS its full
  scale: ADC_AVDD, on a pin of its own so that it can be filtered
  (6.1.5, 12.4.3.7), with the input voltage range stated as 0 to
  ADC_AVDD (table 1438). So `Ref` has one enumerator, `avdd_pin`, and
  what that pin carries is the application's to state. The ceiling on
  an ADC pad is IOVDD's and not ADC_AVDD's: above IOVDD the ESD diodes
  leak, whatever the converter is powered at.

THE PAD, AND WHY AN ANALOG CLAIM IS NOT THE RP2040'S. 12.4 asks for a
shared pad's digital functions to be disabled - IE low and OD high in
the pad control register - and `AnalogIn::claim()` does exactly that.
What it also does, and must, is DROP THE ISOLATION LATCH: 9.7 says the
latch holds the output enable, the output level AND THE PULL ENABLES, so
a pull written into an isolated pad never reaches it. The analogue tap
and the digital input are both outside the latch, so the converter would
read the pad either way - the pull would not be there.

ERRATUM RP2350-E9 MEETS THIS CHAPTER AND LOSES. The leakage that makes a
floating pad sit around 2.2 V on stepping A2 needs the pad's input
buffer ENABLED; the errata sheet's own words are that "disabling the
input enable will reset (remove) the leakage" and that the pull-down
"functions normally in this state". An ADC input configured as 12.4 asks
is therefore the one configuration the erratum does not bite, and the
chapter's instruction and the erratum's workaround are the same write.
The converse is worth stating too: this converter is the only
instrument on the chip that can see the leakage as a VOLTAGE rather than
as a logic level, because the analogue mux taps the bond pad while the
digital buffer is what leaks - and it does, measured, at the millivolt
the errata sheet names (below).

**AND THE ERRATUM HAS TO BE ENTERED, WHICH IS WHY EVERY PULLED-DOWN
READING OF THIS CHAPTER IS SOUND.** Its first condition is that the pad
voltage ALREADY be in the undefined logic region when the input buffer is
enabled. Every pad of this chip comes out of reset with its own pull-down
on and its buffer off, so a pad that is merely released and then given an
input buffer is at ground when the buffer is enabled: the leak never
starts, and the pull-down goes on holding it even with the buffer on.
Measured both ways, and it is the difference between a test that provokes
the erratum and one that only thinks it has.

ONE SENTENCE OF THE CHAPTER IS STALE. 12.4.3.3 says each RROBIN bit
"corresponds to one of the five possible values of CS.AINSEL", which is
the RP2040's count; the register description in 12.4.7 gives RROBIN nine
bits and AINSEL four, and tables 1118 and 1119 give the channels. The
driver follows the register description and the tables.

## Types and verbs

- `adc_bits`, `adc_steps` (util/analog.hpp's full scale),
  `adc_max_count`, `adc_fifo_depth`, `adc_conversion_cycles`,
  `adc_nominal_clk_hz`; and the package's own facts, each read from the
  stratum's reserve and never spelled as a digit: `adc_pad_inputs`,
  `adc_first_pin`, `adc_temperature_code`, `adc_input_count`,
  `adc_round_robin_mask`.
- `AdcInput` (ain0..ain7 and `temperature`, a TAG whose code is the
  package's) with `adc_code_of` and `adc_input_bonded` - whose bound is
  the number of PADS and not of channels, because in the smaller
  package `ain4`'s number is the sensor's, and `select()` refuses a pad
  tag rather than hand the caller the diode;
  `Ref::avdd_pin` with `ref_mv(Ref, board_mv)`; `AdcClock` (pll_usb,
  crystal); `AdcDivider` (integer, frac) with
  `adc_divider_for(clk_hz, rate_hz)` (INT at least 96; nullopt above
  clk / 97 - the back-to-back pace is `AdcDivider{}` - or past sixteen
  bits) and `adc_rate_hz(clk_hz, divider)` (a period of 96 cycles or
  less stretched to its first multiple past the conversion);
  `adc_temperature_centi(counts, ref_mv)` (12.4.6's line, in hundredths
  of a degree); `AdcFifoConfig` (enable, dreq, error_flag, shift,
  threshold) with `valid()`; `AdcFlag`; `adc_pin_valid` /
  `adc_input_of`.
- `AnalogIn<Pin>`: `input`, `claim(pull)` (the pad's digital input
  buffer off, its output driver disabled, the isolation dropped, a pull
  optional), `release()`.
- `Adc`: `package_matches()`, `init(clock, source)` (the package
  checked, the USB PLL brought to 48 MHz unless it is locked on that
  ratio already - or the crystal as it is -, clk_adc selected, the
  block cycled through reset, EN, READY waited for), `clock_hz`,
  `enable`, `ready` / `wait_ready`, `temperature_sensor`, `release`;
  the sampler's surface `select(input)` / `select_input()`,
  `selected()` (AINSEL as it stands, moved by the round-robin),
  `selected_input()` (the last select), `input_code`, `start()` (one
  shot), `result()`, `read()` (one conversion waited for, nullopt on a
  failed one); `start_many(on)`, `running`, `divider` (write, restarting
  the pace; read), `rate_hz`, `round_robin(mask)` (false for a mask
  naming a channel this package has not got) with the compile-time
  `round_robin<mask>()`, `stop_many()` (12.4.3.5's order: START_MANY
  cleared, READY polled, the FIFO drained); `error`, `error_seen`,
  `clear_error_seen`; `fifo(config)` (false for a threshold past the
  depth), `fifo_level`, `fifo_threshold`, `fifo_empty` / `fifo_full`,
  `fifo_overflowed` / `fifo_underflowed`, `clear_fifo_flags`, `pop()`
  with `entry_value` / `entry_failed`, `drain()`, `fifo_address()`,
  `dreq`; `interrupt(on)`, `raw_pending` / `pending`, `force`, `isr()`;
  `irq()`, `reset_bit`, `inputs`, `temperature_input`.
- The two write-one-to-clear flags of this block are cleared by a PLAIN
  write-back with the flag's one, never through the atomic set alias:
  a plain write is right whether or not the alias reaches such a bit,
  so nothing in the driver depends on the answer.

## How to use it

```cpp
using Vin = brio::AnalogIn<brio::Pin<40>>;         // input 0 in the QFN-80
brio::Adc::init(clock);                            // the USB PLL at 48 MHz, clk_adc on it
Vin::claim();
brio::Adc::select(Vin{});
if (const auto counts = brio::Adc::read()) {
    const uint16_t mv = brio::adc_mv(*counts, brio::adc_steps, brio::ref_mv(brio::Ref::avdd_pin));
}
brio::Adc::temperature_sensor(true);
brio::Adc::select(brio::AdcInput::temperature);     // AINSEL 8 here, 4 in the QFN-60
const int32_t centi = brio::adc_temperature_centi(*brio::Adc::read(), 3300);
```

A block through the DMA at a paced rate:

```cpp
using Block = brio::DmaRxEngine<10, uint16_t>;
brio::Adc::divider(*brio::adc_divider_for(brio::Adc::clock_hz(), 10'000));
brio::Adc::fifo({.enable = true, .dreq = true, .error_flag = true, .shift = false, .threshold = 1});
brio::Adc::drain();
Block::arm(brio::Adc::fifo_address(), brio::Adc::dreq);
Block::start(samples, 256);
brio::Adc::start_many(true);

extern "C" void isr_dma_0() {
    if (Block::service() & Block::flag_complete) {
        brio::Adc::start_many(false);   // at once: at 500 ksps the FIFO fills microseconds later
        block_done = true;
    }
}
```

Every channel in turn, which on the larger package is a mask the other
chip's field could not hold:

```cpp
brio::Adc::select_input(0);
brio::Adc::round_robin<brio::adc_round_robin_mask>();   // the package's channels, sensor included
brio::Adc::start_many(true);
```

The sampler:
`AnalogSampler<Adc, P, Subscribers<...>, AdcInput::temperature, Vin{}>`
with the FIFO at threshold 1 and its interrupt posting
`Sampled{Adc::entry_value(Adc::pop()), Adc::selected_input()}`. The
handler's name is `isr_adc_fifo` on both architectures.

## Bench findings

On an RP2350 in the QFN-80 package, **stepping A2**, clk_sys at 150 MHz
and clk_adc at 48 MHz off the USB PLL unless another rate is named, on
BOTH architectures: `test_rp2350_adc` reports **32 pass, 0 fail** on the
Cortex-M33 pair and on the Hazard3 pair, from one source, each on three
flash-and-run cycles. Every verdict reads the same on the two halves.

- **THE BLOCK'S RESET DOES NOT COMPLETE WITH clk_adc STOPPED**, which is
  why `init()` takes the clock first: asked to cycle the block with no
  clock on it, RESET_DONE never comes up; with the crystal routed to
  clk_adc the same cycle completes. The reset state it lands in is
  CS 0x0, FCS 0x100, DIV 0x0 - disabled, not ready, the FIFO off.
- **clk_adc COUNTED, not assumed.** The frequency counter reads
  **48 000 000 Hz** with the USB PLL locked and **12 000 000 Hz** on the
  crystal, each within a tenth of a per cent of what `init()` states, and
  CS.READY comes up on both.
- **NINE CHANNELS, AND THE NINTH IS THE SENSOR.** All nine select and
  read back in AINSEL; a select of 9 is refused with AINSEL left at 8;
  `AdcInput::temperature` names 8. The round-robin mask is nine bits wide
  where the RP2040's was five, and 12.4.3.3's sentence about "five
  possible values of CS.AINSEL" is stale on this chip: the mask 0x1FF
  walks every channel, 72 entries of 72 landing where their input's level
  says they should, and a mask naming a tenth channel is refused.
- **THE DIVIDER BOUNDARY THE CHAPTER DOES NOT STATE, SETTLED.** A start
  that arrives while the converter is busy is ignored, so a divider
  period at or below the conversion's own 96 clk_adc cycles stretches to
  a multiple of itself. Measured over a fixed window: **DIV 0 gives 508
  to 512 ksps** (the free run), **DIV 95 - a period of exactly 96 cycles
  - gives 255 ksps, HALF**, and **DIV 96 - 97 cycles - gives 495 to 500
  ksps**. So the period must be strictly GREATER than the conversion, the
  driver's `adc_rate_hz` predicting 250 000 and 494 845 for those two, and
  its `<=` stands as written. DIV 99 gives 480 to 485 against 480 000
  predicted and DIV 119 gives 400 000 exactly.
- **THE RATES INTO A DMA BLOCK.** 256 conversions free-running: **513 to
  514 us at DIV 0** (512 expected), **2556 to 2559 us at 100 ksps**
  (2560), **25 594 to 25 600 us at 10 ksps** (25 600) and **5802 to
  5804 us at a fractional 44.1 ksps**, DIV 1087 + 111/256 (5804) - every
  one inside four per cent of the system timer, none with an overflow.
  On the crystal at DIV 0 a conversion is 96 of its 12 MHz cycles:
  **2050 to 2055 us for 256**, 125 ksps.
- **THE TEMPERATURE SENSOR, ON BOTH CLOCKS.** 64 reads give a mean of
  **866 to 869 counts** - 30.4 to 31.8 C at a stated 3.3 V by 12.4.6's
  line - with a spread of **under eight counts**, and the crystal's
  12 MHz gives the same mean within one count: the clock is the
  conversion's pace and not its result. With the bias off the same input
  reads **705 to 708**, some 160 counts away, so the diode is what the
  bias connects.
- **EVERY PAD THE PACKAGE GIVES THE CONVERTER**, GP40..GP47, read through
  its own pulls with the digital input buffer off: **3894 to 3899 counts
  (3137 to 3141 mV) pulled up, 58 to 60 counts (47 to 48 mV) pulled
  down**, and **397 to 434 counts (320 to 350 mV) floating** - the last
  a number and not a level, and well clear of both pulls.
- **ERRATUM RP2350-E9 IN MILLIVOLTS, WHICH IS WHAT THIS CONVERTER IS FOR.**
  A pad taken to the supply by its own pull-up and then released with its
  input buffer still enabled falls into the undefined region and is held
  there: the converter reads **2348 to 2361 mV**, against the errata
  sheet's "around 2.2 V", and the pad's own digital buffer reads it HIGH.
  Turning the pad's PULL-DOWN on under that leak moves it to **2142 to
  2144 mV** and no further - the internal pull is far weaker than the
  ~120 uA the leak sources, exactly as the sheet says, and 8.2 kOhm is
  the impedance that would win. Clearing the input buffer removes the
  leak and the same pull-down then holds the same pad at **48 mV**. The
  same walk on a plain digital pad of the same bank reads HIGH, HIGH,
  then LOW through a buffer pulsed for the read alone. **And the door
  matters**: the buffer enabled over a pad the reset pull-down is already
  holding at ground reads LOW and stays there, on both kinds of pad.
- **THE FIFO.** A threshold of four drains **200 entries in 50 interrupts
  over 20 ms at 10 ksps**, the level never past four; a threshold past the
  depth of eight is refused instead of written. Fifty conversions into an
  undrained FIFO leave **level 8, FULL set and the sticky overflow set**.
  **THE ATOMIC SET ALIAS DOES NOT CLEAR A WRITE-ONE-TO-CLEAR FLAG HERE**,
  as on the RP2040: after the drain, a set-alias write leaves the overflow
  standing and the driver's plain write-back clears it. FCS.SHIFT into a
  byte block through an eight-bit DMA engine gives a byte mean of **54**
  against a word mean of 866 to 869 - the word over sixteen, within one.
  Of 256 entries at 500 ksps **none** is flagged as a failed conversion
  and CS.ERR_STICKY stays clear.
- **THE SAMPLER.** `AnalogSampler` over the sensor and two pads on a 5 ms
  software pace gives **39 to 40 samples in 200 ms, 13 or 14 of each
  input**, the sensor's near its own count, the pulled-up pad's at 3895
  and the pulled-down one's at 58 to 59, and **none mislabelled**.

## Not covered yet

Driver gaps, each with its reason:

- The sensor's calibration: 12.4.6's line is the typical device and the
  chapter's own note is that 1 % of reference error is over 4 C; a
  calibrated reading needs a reference thermometer on the desk, and
  this chip carries no factory word to sharpen it.
- A hardware pace other than the divider: the converter has no trigger
  input; a program that wants a timer's pace uses the divider or the
  sampler's software one.
- clk_adc from the system PLL, the ring oscillator or a GPIO input:
  `Clocks::adc_select` takes any aux source the generator has, and
  `init()` offers the two that the chapter's rate and a board's crystal
  make.
- A `MeterSource` or a block-stream shape over the FIFO: the DMA
  address and the request are exposed and the suite drives them
  directly; the task that owns a block belongs to a program that
  streams.

Implemented but not bench-verified, each with what would measure it:

- **`Adc::force`** (the interrupt raised by software), **the FIFO
  underflow flag** (a pop of an empty FIFO staged on purpose) and **a
  failed conversion's own flag** (CS.ERR wants an input held right at a
  code's threshold, which needs a source this bench has not got). The
  verbs are there and no letter stages them; the first two are a letter's
  worth of work and the third wants a precision source.
- **The converter read against a KNOWN voltage.** Everything above is
  read against the pad's own pulls, the supply and the sensor's line; no
  reference on this bench says what a code is worth, so the millivolt
  figures carry the ADC's own linearity and the supply's own accuracy. A
  calibrator on an input would close it.
- **EVERYTHING ABOUT THE QFN-60**: four inputs on GPIO 26..29 with the
  sensor on AINSEL 4 and a five-bit round-robin. The stratum compiles
  for that package, refuses the other one's pads and would refuse the
  other one's mask, and the suite's letters are written to walk
  whichever package they find - but no QFN-60 part is on the bench.
