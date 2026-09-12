# GPIO (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 2.19
(GPIO: function select and table 279, interrupts, pads, the bus
keeper), 2.3.1.2 (SIO's GPIO control), 2.14 (the two banks' resets);
Appendix B (E6). The driver: `brio/rp2040/pin.hpp`. No reference
suite yet; the pin-check firmware has run on it.

## What the silicon does

Thirty user pins in one bank, GPIO0..GPIO29, and three register
blocks that own each of them. IO_BANK0's CTRL register per pin holds
FUNCSEL, which peripheral drives the pad's direction and level and
reads its input - an SPI, a UART, an I2C, a PWM slice, SIO for
software, a PIO, a clock, USB control - with the function number the
same on every pin a peripheral reaches (2 is UART everywhere, 5 is
SIO everywhere) and the instance and signal fixed per pin by table
279. PADS_BANK0's register per pin holds the pad's electrical setup:
drive strength 2/4/8/12 mA, slew, hysteresis, a pull-up, a pull-down,
BOTH pulls together being the bus-keeper mode that holds the last
level, the input buffer enable, and an output DISABLE that overrides
whoever owns the pad. The pad's reset state is input enabled,
pull-down, 4 mA, hysteresis on (0x56): every pin comes up reading low
unless driven. SIO is the processor's single-cycle path: GPIO_OUT,
GPIO_OE and their SET/CLR/XOR twins move the whole bank in one word,
and GPIO_IN ALWAYS READS THE PADS whatever function owns them. Both
blocks are in reset at power-up.

Pin interrupts exist per pin and per destination (core 0, core 1,
dormant wake), level or edge; GPIO26..29 double as the ADC's inputs
(E6: their digital input, left enabled by the B0/B1 bootrom, is
disabled by the B2 one).

## Types and verbs

- `PinFunction` (the FUNCSEL codes, `none` the reset value),
  `PinPull` (`none`, `up`, `down`, `keeper`), `PinDrive` (2/4/8/12 mA),
  `PinConfig` (pull, drive, slew, schmitt, input enable - the pad's
  reset values by default, pull aside), `PinSel` (a pin and the
  function it is handed to), `pad_value(cfg)` (the register word).
- `Gpio` - the bank: `ready()` releases both blocks from reset (every
  configuring verb calls it); the SIO word-wide verbs `in`, `out`,
  `out_set`, `out_clear`, `out_toggle`, `oe`, `oe_set`, `oe_clear`;
  `ctrl(n)`, `status(n)`, `pad(n)` the per-pin registers by number;
  `function(n, fn, cfg)` (the pad first, then FUNCSEL with every
  override neutral), `release(n)` (back to the reset state),
  `outputs(mask, cfg)` (every pin of a mask a software output at once).
- `Pin<n>` - one pin as a type, no port letter: `output(cfg)`,
  `output(level, cfg)`, `input(pull)`, `function(fn, cfg)`, `release()`,
  `set`, `clear`, `toggle`, `read` (the pad, whoever owns it),
  `read_out`, `is_output`, `function()`, `pull(p)`; a PwmChannel of one
  level (`max` 1, `duty`).

## How to use it

```cpp
using Led = brio::Pin<25>;
Led::output();                       // SIO owns it, driven
Led::toggle();

using Key = brio::Pin<23>;
Key::input(brio::PinPull::up);       // the button to ground reads low when pressed
if (!Key::read()) { ... }

brio::Pin<0>::function(brio::PinFunction::uart);   // hand the pad to UART0 TX (table 279)

brio::Gpio::outputs(0x3f7ffffcu);    // every pin of the mask a software output
brio::Gpio::out_toggle(0x3f7ffffcu); // one word, one cycle
```

## Bench findings

- The pin-check wave on GP2..GP22 and GP24..GP29 (GP0/GP1 the
  console, GP23 the button) runs from `Gpio::outputs` and
  `out_toggle`: GPIO_OE reads the mask, GPIO_IN follows GPIO_OUT on
  every driven pin, read back over SWD with the program running.
- The pads read back as programmed: GP0 (UART TX) at 0x52 - input
  enabled, 4 mA, hysteresis, no pull - and GP1 (UART RX) at 0x5a with
  the pull-up the UART task asks for.
- The LED on GP25 follows `Pin<25>` under the console's heartbeat
  (GPIO_OUT's bit 25 toggling at 2 Hz through SIO).

## Not covered yet

Driver gaps, each with its reason:

- Pin interrupts (IO_BANK0's INTR, PROC0/PROC1_INTE, INTS, INTF; level
  and edge; the per-core destination that is this chip's own): born
  with the first program that wants an edge, as an `exti`-shaped
  driver with `ExtInt<Pin>`; the per-core enable is rule 2 of the
  two-kernel model.
- The QSPI bank's six pins (table 281): the flash's; usable as GPIOs
  only on a board with a single-lane flash or none, which no board
  here is.
- The input, output and interrupt overrides in CTRL (invert, force):
  born with a first user (the datasheet's own is the USB enumeration
  workaround of E5, which brio has no USB to need).
- `PADS_BANK0.VOLTAGE_SELECT` for a 1.8 V IOVDD: a supply no board
  here has.
- `PinRef` and `PinSet` (the other families' run-time descriptor and
  multi-pin set): born with their first portable user on this
  family; `Gpio::outputs` covers the bank-wide case today.

Implemented but not bench-verified, each with what would measure it:

- The pulls and the bus keeper on a floating pad: a meter on the pad
  and `Pin::read()`, in a `test_rp2040_pin` suite.
- The drive strengths and the slew: a load and a scope.
- The button on GP23 of the WeAct board: a `test_rp2040_pin` suite
  with a hand on the button.
