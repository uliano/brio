# PWM (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 4.5 (the
PWM block: 4.5.2.1 the counter and the compare, 4.5.2.2 the 0 % and
100 % levels, 4.5.2.3 the double buffering, 4.5.2.4 the fractional
divider, 4.5.2.5 the level and edge input modes, 4.5.2.6 the period,
4.5.2.7 the interrupt and the DMA request, 4.5.2.8 the phase nudges,
4.5.3 the registers), 2.19.2 (table 279: the pins) and 4.5.2 (table
515: the slice and channel of each GPIO); util/pwm_channel.hpp for the
PwmChannel contract. The driver: `brio/rp2040/pwm.hpp` (`Pwm` the
block, `PwmSlice<n>` the resource, the tasks `PwmOutput`, `PwmPair`,
`PwmEdgeCounter`, `PwmLevelCounter`, `PwmPeriodicTick`) over
`pin.hpp`, `resets.hpp` and `dma_engine.hpp`. The reference suite:
`test_rp2040_pwm`, wireless and on two wires between the chip's own
slices.

## What the silicon does

Eight identical slices, each a 16-bit counter with a wrap value (TOP),
two compare levels (CC A and CC B) driving two outputs, an 8.4
fractional divider off clk_sys (1 to 255 and fifteen sixteenths; 0 in
the integer field is 256), a phase-correct mode in which the counter
runs back down to 0 (the period doubles, the pulse stays centred), and
an INPUT mode in which the B pin gates or clocks the counter: count
while B is high, or once per rising or falling edge. One wrap event
per slice is a bit on the block's single interrupt line and a DMA
request. A level of 0 is a 0 % output with no pulse and TOP + 1 a 100 %
output with no gap. CC and TOP are double buffered: a write lands at
the next wrap (the 0-to-0 turn in phase-correct mode). A global enable
register aliases every slice's enable bit, so slices started by one
write run in lockstep; PH_ADV and PH_RET move a running slice one
count forward or back (an advance needs a divider above one). The
raw interrupt register is write-one-to-clear; a force register raises
the line without a raw flag.

THE PINS (table 515): every GPIO is a PWM pin - GPIO n belongs to
slice (n / 2) mod 8, its A output when n is even, its B when odd; GPIO
16..29 repeat slices 0..6, and one output selected on two GPIOs
appears on both. ONLY A B PIN IS AN INPUT: in the level and edge modes
the B pin stops being an output and CC B is ignored; two B pins of one
slice selected at once are ORed. A frequency measured by edges needs
the signal's high and low periods both longer than one clk_sys period
(the edge detector's rule).

## Types and verbs

- `pwm_slice_of(pin)`, `pwm_pin_is_b(pin)`, `PwmDivMode` (free-running,
  level high, rising edge, falling edge), `PwmDivider` (integer, frac;
  `sixteenths()`, `reg()`) with `pwm_divider_of(sixteenths)`,
  `PwmSliceConfig` (mode, divider, top, phase-correct, the two
  inversions) with `pwm_csr_of`, `pwm_period_sixteenths`,
  `pwm_output_hz(sys_hz, config)`, `pwm_config_for(sys_hz, hz, top,
  phase_correct)` (the divider solved to the nearest sixteenth for a
  TOP; nullopt below one or above the range).
- `Pwm`: `reset` / `hold` / `released`, `start(mask)` / `stop(mask)` /
  `running()` (the global enable), the interrupt mask, the raw and
  masked status, `clear_pending(mask)`, `force(mask, on)`, `isr()`
  (the raised-and-enabled slices, cleared), `irq()`.
- `PwmSlice<n>`: `configure(config)` FROM SCRATCH in the vendor's order
  (the slice disabled, its counter at zero, then the divider, TOP and
  the mode; the caller enables) and `config()` read back, `enable(on)`,
  `level(ch, v)` / `levels(a, b)` / `level(ch)`, `top(v)` / `top()`,
  `counter()` / `counter(v)`, `advance_phase()` / `retard_phase()`
  (true once the pulse was inserted or deleted), the wrap bit's
  `interrupt(on)`, `raw_pending`, `pending`, `clear_pending`, `dreq`,
  `cc_address()` / `top_address()` for a DMA channel paced by the wrap.
- `PwmOutput<pin, top>`: a `PwmChannel` with `max` = TOP + 1;
  `setup(divider, phase_correct, invert)`, `setup_hz(clock, hz, ...)`
  (the divider solved for this TOP), `attach()` (the pad alone, for the
  second output of a slice already set up), `duty(v)`, `release()`.
  The frequency belongs to the slice, the duty to the channel.
- `PwmPair<pin_a, pin_b, top>`: A and its complement on B, one slice,
  B inverted with a dead time by arithmetic (B's level is A's plus the
  dead time): on one transition in the plain mode, on both in
  phase-correct mode; `setup(divider, dead_time, phase_correct)`,
  `duty(v)`, `dead_time()`.
- `PwmEdgeCounter<pin_b>` / `PwmLevelCounter<pin_b>`: the B pin as the
  counter's clock or gate; `setup(...)`, `run(on)`, `restart()`,
  `count()` - the window is the caller's to time, the count over it
  is the frequency or the high fraction.
- `PwmPeriodicTick<n>`: the wrap as a periodic interrupt with no pad;
  `setup(divider, top, interrupt)`, `setup_hz(clock, hz)`, `stop()`,
  `flag`.

## How to use it

```cpp
using Red = brio::PwmOutput<12, 999>;      // slice 6 A
using Green = brio::PwmOutput<13, 999>;    // slice 6 B, the same period
using Blue = brio::PwmOutput<17, 999>;     // slice 0 B
using Lamp = brio::RgbLamp<Red, Green, Blue>;

Red::setup_hz(clock, 10'000);              // the slice at 10 kHz
Green::attach();                           // the pad alone
Blue::setup_hz(clock, 10'000);
Lamp::show({255, 128, 64});
```

A frequency measured with no capture unit:

```cpp
using Edges = brio::PwmEdgeCounter<15>;    // slice 7's B pin
Edges::setup();
Edges::run(true);
brio::delay_us(clock, 100'000);            // or any window the timer times
Edges::run(false);
const uint32_t hz = Edges::count() * 10u;  // over 100 ms
```

A level table streamed on the wrap: `DmaTxEngine<ch, uint32_t>` armed
on `PwmSlice<n>::cc_address()` with `PwmSlice<n>::dreq`, one word of
both levels per period.

## Bench findings

The reference suite is `test_rp2040_pwm`, green on the Pico and the
WeAct board: two wireless letters, six on two wires between the
chip's own slices - slice 6's B output on GP13 into slice 7's B input
on GP15, slice 0's B output on GP17 into slice 4's B input on GP9 -
with the measuring slice counting edges or the divided clock while
the wire is high over a window the system timer times.

- The block's reset state: every slice disabled, the divider at one,
  TOP 0xFFFF, the levels and the counter at zero. A configuration
  reads back field by field; the counter runs when enabled (about
  6250 counts in 100 us at divider 2) and stands when disabled.
- LOCKSTEP: two slices started by the global enable read counters
  within a count of each other 50 us later, and two outputs at
  divider 100 keep the same relationship over 300 ms - a FIXED one,
  not always zero: each divider keeps its own phase across a stop and
  a start. A phase advance puts one slice one count ahead, a retard
  brings it back; the first counter read after the first nudge of a
  run answers late by some 45 counts, the ones after are current.
  The force register raises the status with no raw flag and leaves
  with the force.
- THE WRAP AS A TICK: 200 interrupts in 200 ms at 1 kHz, 1000 in 10
  ms at 100 kHz. A 64-word level table streamed into CC by a DMA
  channel paced by the wrap at 10 kHz lands in 6.42 ms, one word per
  period, the last level in place.
- ON THE WIRE: 1 kHz, 100 kHz and 1 MHz counted by edges exact; the
  duty at 0, 25, 50, 75 and 100 % counted by level exact, the two ends
  a flat 0 and full; phase-correct mode at the same divider halves the
  frequency (5 kHz) and keeps the duty; the inverted output at level
  250 of 1000 reads 75 % high; a level written mid-period is taken at
  the wrap and not before, eight trials of eight. The second wire:
  slice 0 at 1 kHz and 50 %. The divider ladder at TOP 199 - one, two
  and a half, ten, 255 and 15/16, 256 - within a tenth of a per cent
  of the arithmetic (625000, 250000, 62500, 2440, 2440 Hz).
- THE PAIR at level 400 of 1000 with 100 counts of dead time: B's level
  reads 500 and B is high 50 % of the period, in both modes; at the
  full level B never rises.
- THE LAMP: util/rgb_lamp.hpp over three outputs - 128 of 255 reads
  50 %, 64 reads 25 %, 255 sets the full level, off() is 0 % on both
  wires.
- TWO HAZARDS OF A SLICE RECONFIGURED WHILE RUNNING, measured and the
  reason `configure()` starts from scratch: a counter above a new,
  smaller TOP runs on to 65535 before it wraps (half a millisecond of
  silence at divider one, 1.25 % of a 40 ms window); and a mode
  flipped under the counter (phase-correct to plain) left the output
  stuck at one level.

## Not covered yet

Driver gaps, each with its reason:

- A meter task over util/meter_sampler.hpp's MeterSource: the edge
  and level counters need a window the caller times, and the task
  that owns the window is born with a program that samples.
- A DMA stream into TOP (a frequency sweep): the address is exposed;
  no program asks for one.
- The phase nudges under a fractional divider: verified at divider 2;
  the rule that an advance needs a divider above one is the chapter's.
- Two B pins of one slice selected at once (their OR): a fact of the
  chapter with no use here.

Implemented but not bench-verified, each with what would measure it:

- `PwmPeriodicTick::setup_hz` past 65536 counts (the solved-divider
  path, 984 Hz for 1 kHz asked at TOP 0xFFFF): a wrap count at that
  rate against the timer.
- `PwmEdgeCounter` in falling-edge mode: a count against the rising
  one on the same signal.
- `pwm_config_for` with phase-correct: the arithmetic is pinned; an
  edge count at the solved divider.
- A preset counter (`counter(v)` before a lockstep start, the phase
  relationship "determined by the initial counter values"): two
  outputs started with different counters and the offset measured on
  the two wires.
