# ADC (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 4.9 (the
ADC and the temperature sensor: 4.9.1 the controller, 4.9.2 the SAR
converter and its clock, 4.9.2.1 the one-shot, 4.9.2.2 the free run
and the divider, 4.9.2.3 the round-robin, 4.9.2.4 the FIFO, 4.9.2.5
the DMA, 4.9.2.6 the interrupt, 4.9.3 and 4.9.4 the measured figures
and erratum RP2040-E11, 4.9.5 the sensor, 4.9.6 the registers), 2.18
(the PLLs), 2.15 (clk_adc), 2.19.6.3 (the pad's control bits);
util/analog.hpp and util/analog_sampler.hpp for the arithmetic and
the sampler. The driver: `brio/rp2040/adc.hpp` (`Adc` the resource,
`AnalogIn<Pin>`, `AdcInput`, `Ref`) over `clock.hpp` (`PllUsb`,
`Clocks::adc_select`), `pin.hpp` (`analog`), `resets.hpp` and
`dma_engine.hpp`. The reference suite: `test_rp2040_adc`, wireless.

## What the silicon does

One 12-bit successive-approximation converter over five inputs: GPIO
26..29 as inputs 0..3 and the temperature sensor as input 4. A
conversion takes 96 cycles of the converter's own clock, clk_adc,
which the chapter asks for at 48 MHz (500 ksps) - the USB PLL's rate;
the generator has an aux mux and no glitchless one, and takes the
crystal as well. A one-shot start; a free run that starts a
conversion every 1 + INT + FRAC / 256 cycles of clk_adc under a 16.8
divider, the converter ignoring a start that arrives while it
converts; a round-robin over any set of the five inputs in the free
run, AINSEL naming the first; an eight-entry FIFO with a threshold
that is the one interrupt and the one DMA request, an optional shift
of each entry to the result's top eight bits (a byte buffer), an
optional error flag per entry (bit 15: the comparator failed to
converge, a sample to discard); a sticky overflow flag when a
conversion completes into a full FIFO. The reference is the ADC_VREF
pin - on the boards here the 3.3 V rail through a filter - and
nothing selectable. The sensor is a diode's Vbe, 706 mV at 27 C with
-1.721 mV per degree, its bias switched on apart from the converter.
Erratum RP2040-E11: the DNL peaks at four codes (512, 1536, 2560,
3584), the ENOB is 8.7 bits, no workaround.

Three facts measured that the chapter states differently or not at
all: the block's reset does not complete without clk_adc running
(RESET_DONE never rises, and a read of its registers then faults), so
the clock comes before the reset; a free run at a divider period of
EXACTLY the conversion's 96 cycles halves the rate (the trigger at the
conversion's last cycle is ignored with the ones inside it: DIV 95 is
250 ksps, DIV 99 is 480 ksps - the chapter's "n will be >= 96"); and
the atomic set alias does not clear a write-one-to-clear flag of this
block (FCS.OVER stays), where a plain write does.

## Types and verbs

- `adc_bits`, `adc_steps` (util/analog.hpp's full scale),
  `adc_max_count`, `adc_conversion_cycles`, `adc_nominal_clk_hz`,
  `AdcInput` (ain0..ain3, temperature), `Ref::vref_pin` with
  `ref_mv(Ref, board_mv)` (what the pin carries is the application's
  to state), `AdcClock` (pll_usb, crystal), `AdcDivider` (integer,
  frac) with `adc_divider_for(clk_hz, rate_hz)` (INT at least 96;
  nullopt above clk / 97 - the back-to-back pace is `AdcDivider{}` -
  or past sixteen bits) and `adc_rate_hz(clk_hz, divider)` (a period
  of 96 cycles or less stretched to its first multiple past the
  conversion), `adc_temperature_centi(counts, ref_mv)` (the chapter's
  line, in hundredths of a degree), `AdcFifoConfig` (enable, dreq,
  error_flag, shift, threshold), `AdcFlag`, `adc_pin_valid` /
  `adc_input_of`.
- `AnalogIn<Pin>`: `input`, `claim()` (the pad's digital input buffer
  off, the output disabled), `release()`.
- `Adc`: `init(clock, source)` (the USB PLL started at 48 MHz from the
  crystal unless locked already, or the crystal as it is; clk_adc
  selected; the block out of reset; EN; READY waited for),
  `clock_hz`, `enable`, `ready` / `wait_ready`, `temperature_sensor`,
  `release`; the sampler's surface `select(input)` / `select_input`,
  `selected()` (AINSEL as it stands, moved by the round-robin),
  `selected_input()` (the last select), `input_code`, `start()`
  (one shot), `result()`, `read()` (one conversion waited for, nullopt
  on a failed one); `start_many(on)`, `running`, `divider` (write,
  restarting the pace; read), `rate_hz`, `round_robin(mask)`,
  `stop_many()` (the chapter's order: START_MANY cleared, READY
  polled, the FIFO drained); `error`, `error_seen`,
  `clear_error_seen`; `fifo(config)`, `fifo_level`, `fifo_empty` /
  `fifo_full`, `fifo_overflowed` / `fifo_underflowed`,
  `clear_fifo_flags` (a plain write), `pop()` with `entry_value` /
  `entry_failed`, `drain()`, `fifo_address()`, `dreq`; `interrupt(on)`,
  `raw_pending` / `pending`, `force`, `isr()`; `irq()`.
- `clock.hpp` grew `PllUsb` (the same block as `PllSys`, at the USB
  PLL's address) and `Clocks::adc_select(aux, div)` with `AdcAux`;
  `pin.hpp` grew `analog(pull)` on `Pin<n>` and `Gpio`.

## How to use it

```cpp
using Vin = brio::AnalogIn<brio::Pin<26>>;        // input 0
brio::Adc::init(clock);                           // the USB PLL at 48 MHz, clk_adc on it
Vin::claim();
brio::Adc::select(Vin{});
if (const auto counts = brio::Adc::read()) {
    const uint16_t mv = brio::adc_mv(*counts, brio::adc_steps, brio::ref_mv(brio::Ref::vref_pin));
}
brio::Adc::temperature_sensor(true);
brio::Adc::select(brio::AdcInput::temperature);
const int32_t centi = brio::adc_temperature_centi(*brio::Adc::read(), 3300);
```

A block through the DMA at a paced rate:

```cpp
using Block = brio::DmaRxEngine<7, uint16_t>;
brio::Adc::divider(*brio::adc_divider_for(brio::Adc::clock_hz(), 10'000));
brio::Adc::fifo({.enable = true, .dreq = true, .error_flag = true, .shift = false, .threshold = 1});
brio::Adc::drain();
Block::arm(brio::Adc::fifo_address(), brio::Adc::dreq);
Block::start(samples, 256);
brio::Adc::start_many(true);
extern "C" void isr_dma_0() {
    if (Block::service() & Block::flag_complete) {
        brio::Adc::start_many(false);   // at once: at 500 ksps the FIFO fills 16 us later
        block_done = true;
    }
}
```

The sampler: `AnalogSampler<Adc, P, Subscribers<...>, AdcInput::temperature, Vin{}>`
with the FIFO at threshold 1 and its interrupt posting
`Sampled{Adc::entry_value(Adc::pop()), Adc::selected_input()}`.

## Bench findings

The reference suite is `test_rp2040_adc`, green on the WeAct board,
every letter wireless: the clocks counted by the frequency counter,
the rates timed on the system timer, the sensor and the pads' pulls
as the known voltages, the DMA as the reader.

- THE CLOCKS: init() on the USB PLL locks it at 48 MHz and clk_adc
  counts 48 MHz, READY up; on the crystal clk_adc counts 12 MHz. The
  block's reset completes only under its clock.
- THE SENSOR reads a room temperature with a spread of five counts
  over 64 reads (863..868 at 3.3 V, 32 C on a board that had run for
  hours), the same reading on the crystal's clock within two counts -
  the clock's rate is the conversion's pace and not its result - and
  a different one with the bias off (about 700 counts).
- THE PADS through their pull-ups read 3910..3918 of 4095, through
  their pull-downs 66..68; the bus keeper is NO KEEPER on an analog
  pad: it latches through the digital input buffer, which the analog
  claim turns off, and reads as the pull-down whatever the pad had.
  GPIO 29 is printed for the record (a Pico's VSYS / 3, a WeAct
  board's bare header pin).
- THE RATE, 256 conversions into a DMA block: 2.003 us each at DIV 0
  (the chapter's 96 cycles), 100 ksps, 10 ksps and a fractional 44.1
  ksps each within a tenth of a per cent of the timer; 8.0 us each on
  the crystal. The DMA keeps up at 500 ksps with one or two entries
  in the FIFO; the FIRST block after a flash overruns (2.09 us a
  conversion and an overflow: the instruction cache cold), the ones
  after are exact, so a warming block precedes the judged ones. A
  loop reading the timer every turn while the DMA pops the FIFO
  starved the pops (the FIFO overflowing every second run at 500
  ksps): the wait keeps off the APB. DIV 95 halves the rate to 250
  ksps, DIV 99 gives 480, DIV 119 400.
- THE ROUND-ROBIN over inputs 0, 1, 2 and the sensor with two pads up
  and one down: every entry of a 64-word block where its input's
  level says it should be.
- THE FIFO: the threshold interrupt at four drains 200 entries in 50
  interrupts over 20 ms at 10 ksps, the level never past four; an
  undrained FIFO fills at eight and raises the sticky overflow,
  cleared by a plain write; FCS.SHIFT delivers the top eight bits
  into a byte block through an eight-bit DMA engine (the byte mean is
  the word mean over sixteen); no entry of 256 at 500 ksps flagged as
  failed on a steady input. AFTER A BLOCK COMPLETES at 500 ksps the
  FIFO fills within 16 us: the converter is stopped from the DMA's
  completion, not from the loop (the chapter's "promptly").
- THE SAMPLER walks the sensor and input 0 on a 5 ms software pace,
  twenty samples each in 200 ms, the sensor's near its count and the
  pulled-up pad's near full scale.

## Not covered yet

Driver gaps, each with its reason:

- The sensor's calibration: the chapter's line is the typical device;
  a calibrated reading needs a reference thermometer on the desk.
- A hardware pace other than the divider: the converter has no
  trigger input; a program that wants a timer's pace uses the
  divider or the sampler's software one.
- clk_adc from the system PLL or a GPIO input: `Clocks::adc_select`
  takes any aux source; init() offers the two the chapter's rate and
  the boards' crystal make.

Implemented but not bench-verified, each with what would measure it:

- A failed conversion (CS.ERR, the entry's flag): the comparator's
  metastability needs an input held at a code's threshold; a slow
  ramp across a full scale with the flags counted.
- The FIFO underflow flag: a pop of an empty FIFO, staged on purpose.
- `Adc::force`: the interrupt raised by software, read in the status.
- The suite on a Raspberry Pi Pico: the same image, its GPIO 29
  reading VSYS / 3.
