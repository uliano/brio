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
digital buffer is what leaks.

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

Implemented but not bench-verified, each with the letter of
`test_rp2350_adc` that will measure it:

- The rate arithmetic and the temperature line at the chapter's own
  examples, the package the silicon reports against the one the image
  was built for, the reset state after a cycle, every channel selected
  and read back in AINSEL with the one past the last refused, and
  clk_adc COUNTED by the frequency counter at 48 MHz on the USB PLL and
  at 12 MHz on the crystal (letter a).
- That the block's reset does not complete with clk_adc stopped, which
  is why this driver takes the clock first: measured on the other chip,
  carried here as a prediction and printed either way (letter a).
- The temperature sensor: a room temperature with a narrow spread, the
  SAME reading on both clocks - the clock's rate being the conversion's
  pace and not its result - and a different one with the bias off
  (letter b).
- EVERY PAD THE PACKAGE GIVES THE CONVERTER read through its pull-up
  and its pull-down with the digital input buffer off, which is the
  configuration 12.4 asks for and the one erratum RP2350-E9 does not
  bite; the floating reading printed beside them (letter c).
- The free run into a DMA block at 500, 100 and 10 ksps and at a
  fractional divider, timed against the system timer; and THE DIVIDERS
  AROUND THE CONVERSION'S OWN 96 CYCLES - a period at or below it
  stretches to a multiple of itself, and whether the trigger at the
  conversion's LAST cycle counts is the boundary the chapter does not
  state and the one the driver's `adc_rate_hz` predicts (letter d).
- A conversion of 96 crystal cycles when clk_adc is the crystal: 125
  ksps (letter d).
- THE ROUND-ROBIN OVER ALL NINE CHANNELS of the larger package, the
  eight pads carrying alternating pulls and the sensor ninth, every
  entry of the block where its input's level says it should be; and a
  mask naming a channel the package has not got refused (letter e).
- The FIFO: the threshold interrupt draining at four, an undrained FIFO
  filling at eight with its sticky overflow, a threshold past the depth
  refused, FCS.SHIFT into a byte block through an eight-bit DMA engine,
  and no entry of 256 flagged as failed on a steady input (letter f).
- WHAT THE ATOMIC SET ALIAS DOES to a write-one-to-clear flag of this
  block - on the other chip it did not clear one - printed beside the
  plain write-back the driver uses (letter f).
- `AnalogSampler` over three inputs on a software pace, the samples
  counted per input and none of them mislabelled (letter g).
- ERRATUM RP2350-E9 AS A VOLTAGE: the same pulled-down pad read by the
  converter with its digital input buffer enabled and then disabled -
  the errata sheet says the leakage parks a floating pad at about 2.2 V
  at a 3.3 V supply, and nothing on this bench has yet measured that
  number rather than a logic level (letter h).
- `Adc::force` (the interrupt raised by software), the FIFO underflow
  flag (a pop of an empty FIFO staged on purpose) and a failed
  conversion's own flag (CS.ERR needs an input held at a code's
  threshold): the verbs are there and no letter stages them.
- EVERYTHING ABOUT THE QFN-60: four inputs on GPIO 26..29 with the
  sensor on AINSEL 4 and a five-bit round-robin. The stratum compiles
  for that package, refuses the other one's pads and would refuse the
  other one's mask, and the suite's letters are written to walk
  whichever package they find - but no QFN-60 part is on the bench.
