# PWM (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 12.5 whole
(12.5.1 the overview, 12.5.1.1 what changed from the RP2040, 12.5.2 the
programmer's model with table 1130 - the slice and channel of every GPIO
-, 12.5.2.1 the counter and the compare, 12.5.2.2 the 0 % and 100 %
levels, 12.5.2.3 the double buffering, 12.5.2.4 the fractional divider,
12.5.2.5 the level and edge input modes, 12.5.2.6 the period, 12.5.2.7
the two interrupt lines and the DMA request, 12.5.2.8 the phase nudges,
12.5.3 the registers), 9.4 (the GPIO function table the pin map also
appears in), 12.6.4.1 (the DREQ table); Appendix E names no erratum of
this block. `util/pwm_channel.hpp` for the PwmChannel contract. The
driver: `brio/rp2350/pwm.hpp` (`Pwm` the block, `PwmSlice<n>` the
resource, the tasks `PwmOutput`, `PwmPair`, `PwmEdgeCounter`,
`PwmLevelCounter`, `PwmPeriodicTick`) over `pin.hpp`, `resets.hpp` and
`dma_engine.hpp`. The reference suite: `test_rp2350_pwm`.

## What the silicon does

TWELVE identical slices, each a 16-bit counter with a wrap value (TOP),
two compare levels (CC A and CC B) driving two outputs, an 8.4 fractional
divider off clk_sys, a phase-correct mode in which the counter runs back
down to 0 (the period doubles, the pulse stays centred), and an INPUT
mode in which the B pin gates or clocks the counter: count while B is
high, or once per rising or falling edge. A level of 0 is a 0 % output
with no pulse and TOP + 1 a 100 % output with no gap. CC and TOP are
double buffered: a write lands at the next wrap (the 0-to-0 turn in
phase-correct mode). A global enable register aliases every slice's
enable bit, so slices started by one write run in lockstep; PH_ADV and
PH_RET move a running slice one count forward or back (an advance needs a
divider above one).

THREE THINGS ARE NOT THE RP2040'S, and they are what 12.5.1.1 names plus
one sentence of 12.5.2.6:

- **Four more slices.** Slices 8..11 are on the die of both packages, but
  their pads are GPIO 32..47, which only the QFN-80 bonds. In the QFN-60
  they are REPEATING TIMERS with no output - the chapter's own words for
  them - and that is what `PwmPeriodicTick` is for there. This stratum
  takes the package from `device.hpp`, so a `PwmOutput` on a pad the
  package has not got is a compile error and `PwmSlice<8>` is legal in
  both.
- **The pin map's second half.** GPIO 0..31 belong to slices 0..7 exactly
  as on the RP2040 (GPIO n to slice (n / 2) mod 8, its A output when n is
  even, its B when odd, repeating at GPIO 16 - the compatibility the
  chapter states); GPIO 32..47 belong to slices 8..11 the same way,
  repeating at GPIO 40. One output selected on two GPIOs appears on both;
  two B pins of one slice selected at once are ORed into its input.
- **A second shared interrupt line.** INTR is one raw register, twelve
  bits wide, but INTE / INTF / INTS come in two sets - IRQ0 and IRQ1 -
  and each is a system interrupt line of its own. So two handlers can own
  disjoint sets of slices, which is what 12.5.1.1 says the line was added
  for: PWM slices as simple repeating timers, beside whatever else the
  block is driving. The raw flag is ONE, so a slice enabled on both lines
  is served by whichever handler runs first and the other finds nothing.
- **The divider stops at 256 exactly.** 12.5.2.6 states that no DIV_FRAC
  bit may be set while DIV_INT is 0, so the fractional part exists up to
  255 and fifteen sixteenths and the last step is a whole 256. The driver
  refuses the combination in `PwmDivider::valid()`, before any register
  moves.

THE PINS: only a B pin is an input, which is why every measuring pad is
an odd GPIO; in the level and edge modes the B pin stops being an output
and CC B is ignored. A frequency measured by edges needs the signal's
high and low periods both longer than one clk_sys period (the edge
detector's rule, 12.5.2.5).

The block is behind the subsystem reset controller (`ResetBlock::pwm`),
and `Pwm::reset()` CYCLES that line rather than merely releasing it: a
processor reset on this chip leaves the peripherals as the previous image
left them, so a driver that wants the reset state has to make it.

## Types and verbs

- `pwm_slice_count` (twelve), `pwm_irq_lines` (two), `pwm_slice_of(pin)`,
  `pwm_pin_is_b(pin)`, `pwm_pin_valid(pin)`, `pwm_slice_has_pads(slice)`
  (the package fact); `PwmDivMode` (free-running, level high, rising
  edge, falling edge), `PwmDivider` (integer, frac; `valid()`,
  `sixteenths()`, `reg()`) with `pwm_divider_of(sixteenths)`,
  `PwmSliceConfig` (mode, divider, top, phase-correct, the two
  inversions) with `pwm_csr_of`, `pwm_period_sixteenths`,
  `pwm_output_hz(sys_hz, config)`, `pwm_config_for(sys_hz, hz, top,
  phase_correct)` (the divider solved to the nearest sixteenth for a TOP;
  nullopt below one or above 256).
- `Pwm`: `reset` / `hold` / `released`, `all_slices` (the twelve-bit
  mask), `start(mask)` / `stop(mask)` / `running()` (the global enable),
  `raw_pending()` / `clear_pending(mask)` (INTR, shared and
  write-one-to-clear), and per LINE - the line a template parameter,
  because what makes a wrap a handler's is the app binding that line's
  vector - `interrupts<line>(mask, on)` / `interrupts<line>()`,
  `pending<line>()`, `force<line>(mask, on)`, `isr<line>()` (the
  raised-and-enabled slices, cleared) and `irq<line>()`.
- `PwmSlice<n>`: `configure(config)` FROM SCRATCH in the vendor's order
  (the slice disabled, its counter at zero, then the divider, TOP and the
  mode; the caller enables) and `config()` read back, `enable(on)` /
  `enabled()`, `level(ch, v)` / `levels(a, b)` / `level(ch)`, `top(v)` /
  `top()`, `counter()` / `counter(v)`, `advance_phase()` /
  `retard_phase()` (true once the pulse was inserted or deleted),
  `interrupt<line>(on)`, `pending<line>()`, `raw_pending()`,
  `clear_pending()`, `bit`, `dreq`, `has_pads`, `cc_address()` /
  `top_address()` for a DMA channel paced by the wrap.
- `PwmOutput<pin, top>`: a `PwmChannel` with `max` = TOP + 1;
  `setup(divider, phase_correct, invert)`, `setup_hz(clock, hz, ...)`
  (the divider solved for this TOP), `attach()` (the pad alone, for the
  second output of a slice already set up, or for the second pad that
  carries the same output), `duty(v)`, `release()`. The frequency belongs
  to the slice, the duty to the channel.
- `PwmPair<pin_a, pin_b, top>`: A and its complement on B, one slice, B
  inverted with a dead time by arithmetic (B's level is A's plus the dead
  time): on one transition in the plain mode, on both in phase-correct
  mode; `setup(divider, dead_time, phase_correct)`, `duty(v)`,
  `dead_time()`.
- `PwmEdgeCounter<pin_b>` / `PwmLevelCounter<pin_b>`: the B pin as the
  counter's clock or gate; `setup(...)`, `run(on)`, `restart()`,
  `count()` - the window is the caller's to time, the count over it is
  the frequency or the high fraction.
- `PwmPeriodicTick<n, line>`: the wrap as a periodic interrupt on one of
  the two lines, no pad; `setup(divider, top, interrupt)`,
  `setup_hz(clock, hz)`, `stop()`, `flag`, `irq()`.

## How to use it

```cpp
using Red = brio::PwmOutput<12, 999>;      // slice 6 A
using Green = brio::PwmOutput<13, 999>;    // slice 6 B, the same period
using Blue = brio::PwmOutput<17, 999>;     // slice 0 B
using Lamp = brio::RgbLamp<Red, Green, Blue>;

brio::Pwm::reset();
Red::setup_hz(clock, 10'000);              // the slice at 10 kHz
Green::attach();                           // the pad alone
Blue::setup_hz(clock, 10'000);
Lamp::show({255, 128, 64});
```

Two repeating timers on two lines, which is what the highest slices are
for (and all they can be in the QFN-60):

```cpp
using Fast = brio::PwmPeriodicTick<8, 0>;  // served by isr_pwm_wrap_0
using Slow = brio::PwmPeriodicTick<9, 1>;  // served by isr_pwm_wrap_1

Fast::setup(brio::PwmDivider{24, 0}, 6249);   // 1 kHz off a 150 MHz clk_sys
Slow::setup_hz(clock, 100);
brio::Irq::enable(Fast::irq());
brio::Irq::enable(Slow::irq());

extern "C" void isr_pwm_wrap_0() { const uint16_t up = brio::Pwm::isr<0>(); /* ... */ }
extern "C" void isr_pwm_wrap_1() { const uint16_t up = brio::Pwm::isr<1>(); /* ... */ }
```

A frequency measured with no capture unit:

```cpp
using Edges = brio::PwmEdgeCounter<15>;    // slice 7's B pin
Edges::setup();
Edges::run(true);
/* a window the system timer times */
Edges::run(false);
const uint32_t hz = Edges::count() * 10u;  // over 100 ms
```

A level table streamed on the wrap wants a DMA channel armed on
`PwmSlice<n>::cc_address()` with `PwmSlice<n>::dreq`, one word of both
levels per period.

## Bench findings

All of them on an RP2350 in the QFN-80 package, **stepping A2**, clk_sys
at 150 MHz, over the board's own wires GP13 -> GP15, GP17 -> GP9 and
GP12 <-> GP14, and all of them on BOTH architectures: `test_rp2350_pwm`
reports **41 pass, 0 fail** on the Cortex-M33 pair and **41 pass, 0
fail** on the Hazard3 pair, from one source. NOTHING IN THIS CHAPTER
DIFFERS BETWEEN THE TWO HALVES - the block is one, its two interrupt
lines are two system lines under one numbering, and the handler names are
the same on both.

- **Twelve slices.** The global enable reads 0xFFF with every slice
  started and zero with every slice stopped. A slice out of reset reads
  CSR 0, DIV 0x10 (one), TOP 0xFFFF, CC 0 and its counter 0; a
  configuration written comes back field by field. The counter at
  divider 2 advances about 7650 counts in 100 us and stands still when
  the slice is disabled.
- **The lockstep start is exact.** Two slices of equal configuration
  started by one write of the global enable read the SAME counter - lag
  **0** at divider 2, and 0 again at divider 100 after 100, 200 and 300
  ms on the second wire's pair. One phase advance puts one of them
  exactly one count ahead and one retard brings it back. (The lag is
  measured by reading the two counters in both orders and averaging, and
  the pass must be a WARM one: run cold, straight out of the XIP cache,
  the same code reports tens of counts of a lag that is not there.)
- **THE TWO INTERRUPT LINES, this chapter's own addition.** A slice's
  wrap on IRQ0 and another's on IRQ1 at 1 kHz give **200 wraps each in
  200 ms**, each in 200 entries of its own handler, with no slice
  appearing in the other line's status; at 100 kHz, **1000 interrupts in
  10 ms**. The two enables are separate registers (INTE0 0x008, INTE1
  0x010), a force on one line shows in that line's masked status alone
  and raises no raw flag, and a slice moved to the other line is served
  there and nowhere else - 50 wraps on line 1 and 0 on line 0.
- **The frequency on the wire**, counted by the far slice's edge
  counter: **1000 Hz, 100 000 Hz and 999 980 Hz** for the three asked
  for, each inside one per cent. **The duty**, counted by level: 0, 250,
  500, 750 and 1000 per mille for the five levels asked - the two ends
  glitch-free and exact.
- **The divider ladder, on the wire**: 750 000 Hz at one, 300 000 at two
  and a half, 75 000 at ten, 2930 at 255 and 15/16 and 2927 at 256 -
  each within a count of the arithmetic's own answer. A fraction on top
  of DIV_INT 0 is refused before any register moves, which 12.5.2.6
  forbids in so many words, so the top of the range is a whole 256.
- **Phase-correct halves the rate** (5000 Hz where the same divider gave
  10 000), **inversion complements the duty** (750 per mille at level
  250), and a level written mid-period is taken AT THE WRAP and not
  before - 8 of 8 trials showed the old duty until the wrap.
- **The pair and its dead time.** At level 400 with 100 counts of dead
  time B's level reads 500 and B is high 500 per mille of the period.
  With A on GP12 -> GP14 and B on GP13 -> GP15 sampled in ONE word read
  through the single-cycle IO, 739 313 samples show A high 388 per
  mille, **both high in 0 samples** and both low in 107 per mille - the
  dead time, as arithmetic and not as a hardware unit.
- **`util/rgb_lamp.hpp` over three outputs**: {255, 128, 64} measures
  1000, 502 and 251 per mille on the three pads.
- **THE FOUR SLICES THIS CHIP ADDED.** Slices 8..11 run as repeating
  timers, two on each interrupt line, **200 wraps each in 200 ms**. Their
  pads are on the half of the map the RP2040 had not: slice 8's A output
  reads 242, 490 and 742 per mille on **GP32 and GP40 alike** for levels
  of 250, 500 and 750 of 1000, and slice 11's the same on **GP38 and
  GP46** - the two pads that carry ONE output never disagreeing in a
  single word read, over some 420 000 samples each.
- **A LEVEL TABLE STREAMED INTO CC, PACED BY THE WRAP.** A DMA channel
  armed on a slice's wrap request moves NOTHING while that slice is
  stopped - the request is the wrap, and there is no wrap. Once the slice
  runs, 64 CC words go in at one word a period: **6405 us for 64 periods
  of 100 us, with 64 wraps counted** and the processor doing nothing
  between them. CC is one register and both levels, so the last word
  stands in A and in B together. And the stream reaches the pad: the same
  table into slice 6 leaves the far end of GP13 -> GP15 reading **839 to
  840 per mille against the 840 the table's last word asks for**.

## Not covered yet

Driver gaps, each with its reason:

- A meter task over `util/meter_sampler.hpp`'s MeterSource: the edge and
  level counters need a window the caller times, and the task that owns
  the window is born with a program that samples.
- A DMA stream into CC or TOP AS A TASK: the suite's own letter drives
  one with a `DmaTxEngine` by hand (above), and what is not here is the
  task that would own the table and the re-arm - born with the program
  that wants a waveform rather than a level.
- Two B pins of one slice selected at once (their OR): a fact of the
  chapter with no use here, and no wire of this desk puts two signals on
  one slice.
- The portable side of "one dimmable output" past `PwmChannel`: a servo
  or a dimmer AO sits above this file and is born with its program.

Implemented but not bench-verified, each with what would measure it:

- A stream into TOP rather than into CC: `top_address()` is exposed and
  the pace would be the same request, but nothing has asked for a
  frequency sweep yet.
- `PwmEdgeCounter` in falling-edge mode, and `pwm_config_for` with
  phase-correct: the arithmetic of both is pinned at compile time, and
  what is missing is an edge count on the wire at the solved divider -
  one more case in letter c's loop.
- A preset counter (`counter(v)` before a lockstep start, the phase
  relationship "determined by the initial counter values"): two outputs
  started with different counters, the offset measured on the two wires.
- Everything about this chapter in the QFN-60, where slices 8..11 have
  no pad: the stratum compiles for that package and refuses those pads,
  and no QFN-60 part is on the bench.
