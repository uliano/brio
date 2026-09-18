# GPIO (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), chapter 9
whole - 9.1 (the two banks and what they carry), 9.2 ("changes from
RP2040"), 9.3 (the reset state), 9.4 (function select, the four
overrides and the STATUS register), 9.5 (the interrupts, their twelve
outputs and the summary registers), 9.6 (the pads, with 9.6.1's bus
keeper), 9.7 (THE PAD ISOLATION LATCHES), 9.8 (the processors' own path
through SIO), 9.9 (the Cortex-M33's GPIO coprocessor port) and 9.11 (the
registers) - plus 3.1.3 (SIO's GPIO registers), 7.5 (the reset
controller), 3.2 (the interrupt lines), 12.15.1 (SYSINFO's PACKAGE_SEL)
and appendix E's **RP2350-E9**, which is live on the bench silicon and
changes what a pull-down means. The driver: `brio/rp2350/pin.hpp`. The
reference suite: `test_rp2350_pin`, which runs on BOTH of this chip's
architectures from one source, over four wires and two nets that carry a
real pull-up.

## What the silicon does

**THREE BLOCKS OWN A PIN.** IO_BANK0 has a CTRL register per pad whose
FUNCSEL chooses which peripheral drives it, four OVERRIDES between that
peripheral and the pad, and a STATUS register that reads back what
actually arrived; PADS_BANK0 has a register per pad for the electrical
behaviour - drive strength, slew, hysteresis, the two pulls, the input
buffer, an output disable that outranks whoever owns the pad, and the
ISOLATION LATCH; SIO has the whole bank in two words per direction,
reachable in one cycle from either processor.

**WHAT IS NOT THE RP2040'S** (9.2): eighteen more pads in the QFN-80, a
third PIO among the functions, the USB pads usable as GPIO, a SECOND
FUNCTION COLUMN on every group's flow-control pads (the UART chapter's),
**the isolation latches**, **a different reset state for the pads**,
twice as many interrupt outputs because Secure and Non-secure have their
own, and **summary registers** that say which pad is asking without
reading six words. The bit maps differ too: IO_BANK0 and PADS_BANK0 sit
in RESETS bits 6 and 9 here, 5 and 8 there.

**THE PADS COME UP ISOLATED AND DEAF** (9.3). PADS_BANK0's reset value is
0x116: the ISO latch set, the input buffer DISABLED, a pull-down on,
4 mA, hysteresis on, with FUNCSEL at the null function so the output
buffer is high-impedance. While the latch stands, the pad is cut off from
the digital logic in both directions, so a pad configured as on the
RP2040 - function and level, no pad write - would do nothing at all.
Every configuring verb of this stratum writes the WHOLE pad register with
ISO clear, which is both the documented order (set the pad and the
function up, drop the isolation last) and the reason a configuring verb
is also the way OUT of isolation.

**THE LATCH IS THE NEW IDEA, AND IT IS NOT ONLY FOR SLEEP** (9.7). What
it freezes is the output level, the output enable and the pulls - the
signals crossing from the switched core domain to the pad - so that a
power-down of that domain glitches nothing. The input is never isolated.
It exists so a pad keeps its state through a low-power state, and the
same mechanism answers to software at any time: set ISO, and the pad
ignores SIO until it is cleared.

**ERRATUM RP2350-E9 IS LIVE ON STEPPING A2 AND IT RULES THIS BENCH.**
With its input buffer enabled, a Bank 0 pad sitting in the undefined
logic region leaks about 120 uA out of the input stage, which holds it
around 2.2 V - more than the pad's own pull-down can fight. So on this
stepping A PAD WHOSE BUFFER IS ON AND WHOSE DRIVER HAS LET GO HOLDS ITS
WHOLE NET HIGH, whatever pulls anything else on that net has. The pull-UP
still works (it takes the pad out of the region), and a pad driven low
and released stays low (it never enters the region). The errata sheet's
workaround is the pair this driver offers: keep the buffer OFF on a pad
that must be held by its pull-down, and enable it for the length of a
read alone. Stepping A3 removes the leakage path.

**THE INTERRUPTS ARE PER PAD AND SHARE ONLY THEIR LINE** (9.5). Four
events - two levels, two edges - for each of 48 pads, in six words per
destination, with three destinations (proc 0, proc 1, dormant wake) each
having its own enable, force and status registers over one shared raw
register. The levels are NOT latched: a level event stands while the pad
holds that level and ends when the pad changes, so clearing it does
nothing and a handler that means to return must disarm it. The edges are
latched in INTR and cleared by writing a one. The status register is
"after masking AND FORCING", which has a consequence the datasheet does
not spell out and this bench measured: **a forced event bypasses the
enable**, so disarming a pad does not end a force - only unforcing does.

**THE PACKAGE IS A BOND WIRE AND NOT A DIE.** Every pad's CTRL and PAD
register exists on both packages; what the QFN-60 has not got is the
wire. A build that states its package refuses GP30..GP47 at COMPILE
time; a build that states none compiles them and refuses at run time
against SYSINFO's PACKAGE_SEL.

## Types and verbs

- `Pin<n>` - one pad as a type, `n` 0..47, with no port letter. The
  common surface: `output()` / `output(level)` / `input(PinPull)` /
  `input(PinConfig)` / `function(PinFunction)` / `release()` /
  `analog()`, `set` / `clear` / `toggle` / `read` / `read_out` /
  `is_output` / `function()`, `pull(PinPull)`, `ref()`, `bonded()`, and
  `max` / `duty(v)` - a `Pin` is a `PwmChannel` of one step. Every
  configuring verb answers BOOL, because a pad this package has not got
  is not written and the caller is told.
- Beyond it, this chip's own: `isolated()` / `isolate(bool)` (9.7's
  latch), `read_pulsed()` (E9's workaround: the input buffer enabled for
  the read alone), `input_enable(bool)` / `input_enabled()`,
  `output_disable(bool)` / `output_disabled()`, the four overrides
  `out_override` / `oe_override` / `in_override` / `irq_override` with
  their read-backs, and `status()`, which returns what reached the pad.
- `PinConfig` - the pad in one value: `pull`, `drive`, `slew_fast`,
  `schmitt`, `input_enable`. Its defaults are the pad's reset values
  except that the input buffer is ON and no pull is applied: a pad this
  stratum configures is a pad meant to be used. `pad_value(cfg)` is the
  register word, always with ISO clear.
- `PinFunction` (spi, uart, i2c, pwm, sio, pio0, pio1, pio2, gpck, usb,
  uart_alt, none), `PinPull` (none, up, down, keeper), `PinDrive` (2, 4,
  8, 12 mA), `PinOverride` (pass, invert, low, high), `PinOeOverride`
  (pass, invert, disable, enable), `PadVoltage` (v3v3, v1v8 - read and
  written by nothing here), `PinSel` (a pad and the function it is handed
  to), `PinStatus` (out_to_pad, oe_to_pad, in_from_pad, irq_to_proc).
- `Gpio` - the bank: `ready()`, `bonded(n)`, the SIO words in both halves
  (`in` / `in_hi` / `out` / `out_hi` / `out_set` / `out_clear` /
  `out_toggle` / `oe` / `oe_set` / `oe_clear` / `oe_toggle` and the `_hi`
  twin of each), the per-pad registers `ctrl(n)` / `status(n)` / `pad(n)`,
  the run-time `function(n, fn, cfg)` / `analog(n, pull)` / `release(n)`,
  the four overrides and `pin_status(n)`, `input_enable` /
  `output_disable` / `isolate` by number, `voltage()`, and
  `outputs(mask)` / `outputs_hi(mask)` for a bank of parallel outputs.
- `PinRef` - a pad named at RUN TIME (what a bus request carries as its
  chip select): `valid`, `set`, `clear`, `toggle`, `read`, `read_out`,
  each a SIO word access in whichever half the pad falls.
- `PinEvent` (level_low, level_high, edge_low, edge_high), `PinEvents`
  (a set, with `|` and `has`), `pin_edges` (both edges), `PinIrqTarget`
  (proc0, proc1, dormant_wake).
- `PinIrq` - the bank's interrupt controller: `enable(pin, events,
  target)` / `disable` / `enabled`, `status` (INTS) / `raw` (INTR) /
  `clear`, `force` / `unforce` / `forced`, `summary(target, half)`,
  `disable_all(target)`, `irq()`.
- `ExtInt<Pin, target>` - one pad's interrupt as a type, in the name the
  SAM's EIC and the STM32s' EXTI use for the same thing: `arm(events)`,
  `disarm()`, `armed()`, `pending()`, `raw()`, `clear()`, `force()` /
  `unforce()` / `forced()`, `irq()`, and `served()` - the ISR body, which
  returns what this pad raised and clears the latched half of it. It is
  THINNER than its namesakes on purpose: on those families a LINE is a
  resource several pads compete for and the driver's first job is to
  refuse the second claimant, while here every pad owns its own four
  event bits and nothing is shared but the line out of the block.

## How to use it

```cpp
using Led = brio::Pin<25>;
Led::output();                       // SIO, the pad written, the latch dropped
Led::set();
```

A pad handed to a peripheral, and taken back:

```cpp
Pin<19>::function(brio::PinFunction::pio0);
Pin<19>::release();                  // no owner, isolated again, as at reset
```

An input that a pull-DOWN must hold, on a stepping where E9 is live:

```cpp
using Sense = brio::Pin<22>;
Sense::input({.pull = brio::PinPull::down, .input_enable = false});
const bool level = Sense::read_pulsed();   // the buffer on for the read alone
```

A pin interrupt, with the one vector name both architectures bind:

```cpp
using Button = brio::ExtInt<brio::Pin<8>>;
Button::arm(brio::pin_edges);
brio::Irq::enable(Button::irq());

extern "C" void isr_io_bank0() {
    const brio::PinEvents e = Button::served();
    if (!e.none()) { ... }             // a LEVEL would want Button::disarm()
}
```

A pad frozen across a change of the registers behind it:

```cpp
Pin<19>::isolate(true);              // the level, the direction and the pull latched
Pin<19>::clear();                    // SIO says low; the pad does not move
Pin<19>::isolate(false);             // now it does
```

## Bench findings

All of them on an RP2350 in the QFN-80 package, **stepping A2**, at
3.3 V, and all of them on BOTH architectures: `test_rp2350_pin` reports
**51 pass, 0 fail** on the Cortex-M33 pair and **51 pass, 0 fail** on the
Hazard3 pair, from one source.

- **The reset state, seen**: a pad this image has never configured reads
  0x116, which is the datasheet's own reset value. Configured as an
  output it reads 0x052 - the latch dropped, the input buffer on, 4 mA,
  hysteresis - and `release()` puts 0x116 back with FUNCSEL at the null
  function. The die reports the QFN-80 package, which is what the build
  states, and the bank's input thresholds are the 2.5 to 3.3 V setting.
- **Eight wire directions**: each of the four wires drives and reads
  correctly in BOTH directions - a wire has no direction, and neither
  does a pad.
- **The pulls**: on the two nets that carry a real pull-up to 3V3, both
  pads read high with both ends listening, both read low when one end
  drives low, and both come back high when it lets go. The chip's own
  pull-up holds a free pad high.
- **ERRATUM RP2350-E9, MEASURED FOUR WAYS.** On this A2 die a free pad
  with its input buffer enabled and its own pull-down on reads **1**
  after having been driven high and released, and **0** after having been
  driven low and released - an idle level here is HISTORY and not a
  measurement. With the buffer off and the read taken through
  `read_pulsed()`, the same pad after the same high reads **0**: the
  buffer is what leaks. The erratum reaches further than one pad, and
  the suite ran into it three more times - the far end of a wire, held by
  its pull-down, reads high as long as the NEAR pad's buffer is on and
  its driver has let go. Every letter that lets a driver go now turns
  both buffers off.
- **The input buffer costs nothing to wake**: on a pad its own pull-up
  holds high, the eight SIO reads taken as fast as they can be issued
  after the enable ALL read the pad's level (0xFF, first sample
  included). The enable is an APB write of four cycles and the read a
  single-cycle SIO access, so the buffer is awake before it can be asked
  - which is why `read_pulsed()` needs no settle.
- **The bus keeper** holds the level the pad last had, in both
  directions, on a free pad with nothing else on it.
- **The pad's electrical controls**: all four drive strengths, both slew
  settings and both hysteresis settings read back exactly as written, and
  none of them changes what the far end of a wire reads. The pad's own
  OUTPUT DISABLE takes the driver off the wire over SIO's head, and
  clearing it hands the pad back.
- **The function select and STATUS**: the console's own pads read
  function 2, the UART's first column. A pad driving high reads STATUS
  out 1, oe 1, in 1; the pad at the other end of the wire reads out 0,
  oe 0, in 1 - a listening pad with the level arriving. Handed to PIO0 a
  pad reads function 6, and back under SIO the wire follows again.
- **The four overrides**: with SIO saying LOW, OUTOVER forced high puts
  1 on the wire, forced low 0, inverted 1 and passed 0. OEOVER's
  `disable` takes the driver off a wire whose far pad is pulled up (which
  then reads 1) and `pass` puts it back. INOVER inverts and forces what
  SIO ITSELF READS - the override sits between the pad and the function,
  and SIO is a function - while STATUS's `in_from_pad` keeps reporting
  the pad. That is the one place in this chapter where GPIO_IN and STATUS
  disagree, and it is by design.
- **The interrupts**: thirty-two rising edges are served thirty-two
  times, exactly once each, and the path from the store that made the
  edge to the handler's own timestamp costs **0 to 5 us on the
  Cortex-M33 pair and 0 to 4 us on the Hazard3 pair** - the ruler's grain
  is one microsecond, so the path's own cost is under it and the worst of
  the thirty-two is the one that waited for another handler to finish
  (no interrupt nests over another on this platform). A falling edge that
  is not armed raises nothing; both edges armed serve both transitions
  and the second reports edge_low. A LEVEL armed on a pad that already
  holds that level fires at once and exactly once, because the handler
  DISARMS it - clearing cannot end a level. Two pads armed at once are
  served as two, on one line, and the summary register reads exactly the
  bit of the pad that is asking.
- **A FORCED EVENT BYPASSES THE ENABLE**, measured the hard way: a force
  raised on a pad with EVERY enable cleared still reaches the line, and
  a handler that clears and disarms re-enters for ever. Only unforcing
  ends it. The suite's handler asks INTF and unforces what it served,
  which is the pattern this silicon requires; the forced interrupt then
  arrives exactly once with nothing armed and nothing on the wire.
- **The isolation latch, both halves of it**: a pad driving high, then
  isolated and told through SIO to go low, keeps the wire at 1 while its
  own GPIO_OUT register reads 0, and drops to 0 the moment the latch is
  opened. The same pad isolated and told through SIO to stop driving
  keeps driving - the output ENABLE is latched too - and lets go when the
  latch opens. And a configuring verb of this stratum opens the latch on
  its way past, which is why the direction in that test was changed
  through SIO and not through `Pin::input()`.
- **The package's edges**: `bonded()` answers for every pad of this
  package and refuses the first past the register files; `function(48)`,
  `release(48)` and `analog(48)` write nothing and return false; a
  `PinRef` past the bank is not valid and does nothing. A pad in the HIGH
  half (GP40) is driven and read through GPIO_HI_OUT and GPIO_HI_IN at
  its own bit, eight places down from its pin number.

## Not covered yet

Driver gaps, each with its reason:

- **The QSPI bank** (IO_QSPI and PADS_QSPI, the six flash pads and the
  USB DP/DM pair): its pads carry the chip's own flash, and its verbs
  belong to the flash chapter that owns them - a stray write there is a
  chip that does not boot.
- **The Cortex-M33 GPIO coprocessor port** (9.9): a fast path to the
  same SIO registers that exists on ONE of this chip's two
  architectures, which is exactly what this stratum promises not to
  have. It would be a second spelling of every verb in `Pin`, right on
  one half of the target and absent on the other; declined until
  something measures a GPIO access as a bottleneck, and then behind a
  name that says it is the Arm half's.
- **The pads' input threshold select** (`VOLTAGE_SELECT`): read and
  written by nothing. Both banks share one IOVDD supply, and the
  datasheet's warning is that driving pads above 1.8 V with the 1.8 V
  thresholds selected may damage the chip - the value is a fact of the
  board, not a program's to change.
- **The Non-secure interrupt outputs and the Non-secure GPIO mask**: the
  bank has twelve interrupt outputs and brio runs everything Secure, so
  the driver drives the Secure destinations. Which pads a Non-secure
  context may see at all is ACCESSCTRL's, a chapter this stratum reads
  and does not write.
- **The dormant-wake destination**: `PinIrqTarget::dormant_wake` is
  named and its registers are reachable, but arming a wake without the
  low-power state it wakes from is half a mechanism. It belongs with the
  POWMAN chapter that enters that state.

Implemented but not bench-verified, each with what would measure it:

- **The QFN-60 package**: the stratum compiles for it, the negative tests
  prove it refuses GP30 and a pin interrupt on GP40, and no QFN-60 part
  is on the bench. Erratum RP2350-E3, which is that package's alone,
  is therefore untouched as well.
- **Drive strength and slew as EFFECTS.** The four strengths and both
  slew settings are written and read back, and this chip has no way to
  witness an edge rate or a source impedance from the inside: a scope on
  a pad, or a load that a 2 mA driver cannot hold and a 12 mA one can.
- **The interrupts towards proc 1**: the registers are written and read
  back through `PinIrqTarget::proc1`, and nothing has launched the
  second core on this target yet - that is the multicore chapter.
- **`Gpio::outputs` / `outputs_hi`** over a mask of many pads: used by
  the pin-check tool and not by a letter of this suite, which drives its
  pads one at a time because it has wires to judge.
- **The bank under another function's interrupt**: the pin interrupts are
  proven with SIO owning the pad. A pad owned by a peripheral raises the
  same events (the IRQ path is the pad's, not the function's), and no
  letter shows it.
