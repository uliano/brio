# Clock (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), chapter 8
whole - 8.1 (the generators, their two multiplexers and the switching
sequences of 8.1.3.2, the 16.16 divider, duty cycle correction, the
clock enables of 8.1.3.5, the frequency counter of 8.1.4 with its
interval table, the resus circuit of 8.1.5, and the programmer's model),
8.2 (XOSC, with 8.2.2's "changes from RP2040"), 8.3 (ROSC, with its own
8.3.2 and the randomiser of 8.3.6), 8.4 (the low-power oscillator, whose
registers are POWMAN's), 8.5 (the tick generators) and 8.6 (the two
PLLs) - plus 7.5 (the reset controller), 3.2 (the interrupt lines) and
2.1.3 (the atomic register aliases). Appendix E names no erratum in this
chapter. The driver: `brio/rp2350/clock.hpp`, with the microsecond
busy-wait beside it in `brio/rp2350/delay.hpp`. The reference suite:
`test_rp2350_clock`, which runs on BOTH of this chip's architectures
from one source - every rate counted by the chip's own frequency
counter, the switches timed by the platform timer, the drivers on
clk_sys carried through each of them, and two letters over a clock wire
to a peer board.

## What the silicon does

It is the RP2040's clock tree with four roots instead of three, a
generator more, and a divider one binary point finer. There is no bus
prescaler and no enable bit per peripheral: one **clk_sys** feeds the
cores, the fabric, the memories and every peripheral's bus interface; a
separate **clk_peri** feeds the UARTs and SPIs, so that their bit rates
survive a change of clk_sys; **clk_usb**, **clk_adc** and (new here)
**clk_hstx** have generators of their own; **clk_ref** is the
always-running reference the tick generators and the frequency counter
use. Each generator is an auxiliary multiplexer over the chip's sources
- the crystal oscillator, the ring oscillator, the two PLLs, two GPIO
clock inputs, and for clk_ref the low-power oscillator - followed by a
divider and an enable. clk_ref and clk_sys have a GLITCHLESS
multiplexer in front, because they must never stop, and a SELECTED
register that says which source is in force.

**What is not the RP2040's**, beyond the bigger numbers: there is no
clk_rtc, because there is no RTC - the always-on timer in POWMAN took
its place, with a clock of its own; clk_ref gains the LOW-POWER
OSCILLATOR as a fourth source and is capped at 25 MHz whatever the
crystal is; the dividers are 16.16 where the RP2040's were 24.8, and
clk_peri has one at all; the TICK GENERATORS of 8.5 are a block of their
own, where the RP2040 derived its microsecond from the watchdog; the
ring oscillator can randomise its own frequency; the PLLs report a lock
that was LOST and not only a lock that is present, on an interrupt line
each; and the frequency counter's source numbering is another one.

**The switching sequences are the chapter's whole point** (8.1.3.2). A
generator with a glitchless mux changes its AUX source only while the
glitchless mux is parked elsewhere; a generator without one is stopped,
changed and restarted - and the datasheet asks for its `ENABLED` status
to be POLLED rather than for cycles to be counted, because the generated
clock may be far slower than the one counting them.

**At power-up the chip runs on the RING OSCILLATOR**, at a nominal
11 MHz guaranteed only to sit between 4.6 and 19.6, with the crystal
off, both PLLs in reset and clk_peri disabled. The crystal oscillator
takes 1 to 50 MHz in one of FOUR range codes and needs a startup delay
in units of 256 crystal cycles before STATUS.STABLE rises (47 for 12 MHz
and the 1 ms of the reference design). The system PLL multiplies the
crystal into a VCO of 750..1600 MHz through FBDIV 16..320 and divides it
by two post dividers of 1..7; clk_sys tops out at 150 MHz. The
datasheet's advice is the highest VCO for the least jitter and the
larger post divider first for the least power.

**Two registers in this chapter are passwords and not fields**, and they
are the chapter's one trap. XOSC's and ROSC's `CTRL.ENABLE` and
`CTRL.FREQ_RANGE` are twelve bits wide and take only their enumerated
codes; anything else sets `STATUS.BADWRITE` and is refused. That makes a
read-modify-write through the atomic SET / CLEAR / XOR aliases illegal,
since such a write puts zeros in the field it does not mean to touch -
and it makes carrying ROSC's own reset value over into a write illegal
too, because `FREQ_RANGE` RESETS TO 0xaa0, which is none of its four
codes. Both are measured, and both are why every write of either CTRL in
this driver is a whole word with both passwords made legal.

**The low-power oscillator is the always-on domain's** (8.4): a nominal
32.768 kHz RC oscillator that starts with the core supply, cannot be
stopped from software, and is what the always-on timer counts while the
switched core is powered down. Its accuracy is plus or minus 20 per cent
untrimmed and about 1.5 trimmed, in 63 steps. Its registers live in
POWMAN and are PASSWORD PROTECTED: a write whose top sixteen bits are
not 0x5AFE is ignored and sets BADPASSWD.

**The resus circuit** (8.1.5) is the watchdog of clk_sys itself: with no
edge of clk_sys within TIMEOUT cycles of clk_ref it forces the
glitchless mux back onto clk_ref and raises the CLOCKS interrupt, so
that a program which stopped its own system clock still answers a
debugger. The datasheet calls it a debugging aid and warns that a
clk_sys merely SLOWER than the timeout assumes looks exactly like one
that stopped.

## Types and verbs

- `ClockSource`: `crystal` and `pll` built; `internal` (the ROSC),
  `external` (a clock into XIN) and `gpin` declared and refused - the
  ring oscillator's rate is not a truth `hz` could state.
- `Clock<source, hz, crystal_hz = 12'000'000, peri = PeriSource::sys>`
  - the static main clock, the ONE truth: `hz` = clk_sys, `pclk_hz` =
  clk_peri (clk_sys undivided, or the crystal under
  `PeriSource::crystal`), `ref_hz` = clk_ref = the crystal, `pll` (the
  setting found), `startup_delay`; `init()` runs the whole sequence and
  answers false when the crystal did not start, the PLL did not lock or
  a switch did not take. Called on a running tree it is THE RATE SWITCH,
  and it is written to come back FROM ANY TREE a previous image can
  leave: it parks clk_sys on clk_ref and clk_ref on a ring oscillator it
  started before it touches a PLL. The drivers on clk_sys (the Cortex-M33
  half's timebase, a UART on `PeriSource::sys`) are owed a `rebase(hz)`;
  a UART on a crystal-fed clk_peri is owed nothing. `count_hz(what)`
  counts a source against the crystal.
- `pll_config_for(ref_hz, out_hz)` - the exact ratio at compile time
  under 8.6.1's constraints, REFDIV 1, the highest VCO, the larger post
  divider first; a rate with no exact ratio is a compile error.
  `xosc_startup_delay(crystal_hz, settle_us)`, `xosc_range_code(hz)`,
  `tick_cycles_per_us(ref_hz)`, `fc_interval_us(code)`.
- `Xosc` - the crystal: `init` (the delay and the range in one word; a
  stable crystal is kept), `stable`, `enabled`, `startup_delay`,
  `startup_x4`, `range` and `range_code`, `wait_periods` (the countdown
  register, a short wait in CRYSTAL periods and not in instructions),
  `count`, `badwrite` / `clear_badwrite`, `stop`.
- `Rosc` - the ring oscillator and THE SAFE PLACE TO PARK: `start` (which
  does not write CTRL when it is already running), `stop`, `running`,
  `stable`, `div_running`, `range(RoscRange)` and `range_code`,
  `drive(stage, strength)` behind its password, `randomise(ds0, ds1)` /
  `randomised` / `seed`, `divider` (the divisor, with the block's rule
  that anything outside 1..127 divides by 128), `random_bit`,
  `wait_periods` / `count`, `badwrite` / `clear_badwrite`.
- `Lposc` - the low-power oscillator, in the always-on domain: `trim`
  (0..63), `mode`, `freq_khz_int` / `freq_khz_frac` / `freq_khz` (what
  the always-on timer is TOLD the rate is) and `declared_hz`,
  `bad_password` / `clear_bad_password`.
- `PllSys` and `PllUsb` (one `PllBlock` at two addresses): `init` with a
  `PllConfig`, `locked`, `unlocked`, `config` read back, `bypass` /
  `bypassed`, `stop` / `stopped`, and THE LOCK-LOSS SIDE this chip adds -
  `lock_lost` / `clear_lock_lost`, `lock_lost_interrupt`,
  `interrupt_pending`, `force_interrupt`, `irq()` and an `isr()` body.
- `Clocks` - the generators. clk_ref: `ref_select(RefSource)`,
  `ref_from_aux(RefAux)`, `ref_source`, `ref_aux_source`, `ref_divider`.
  clk_sys: `sys_from_ref`, `sys_from_aux(SysAux)`, `sys_source`,
  `sys_aux_source`, `sys_divider(int, frac)` with both read back. The
  four with an aux mux alone, each `*_select(aux, div)` / `*_stop` /
  `*_enabled` / `*_source` / `*_divider`: `peri_`, `usb_`, `adc_`,
  `hstx_`. And the top-level gates: `wake_enables`, `sleep_enables`,
  `enabled`, over a `SleepClocks` pair of words with `sleep_clocks_all`
  and `sleep_clocks_none`.
- `Resus`: `enable(timeout)` / `disable` / `enabled` / `timeout`,
  `resussed`, `clear`, `force` / `unforce`, the interrupt
  (`interrupt`, `raw`, `pending`, `force_interrupt`, `irq()`, `isr()`).
- `TickGenerator<TickConsumer>` - one per consumer (`proc0`, `proc1`,
  `timer0`, `timer1`, `watchdog`, `riscv`): `start(cycles)` (which stops
  the generator first, as 8.5.1 asks), `stop`, `running`, `cycles`,
  `count`.
- `FreqCounter::count_hz(source, ref_hz, interval = 15)` - a
  `CountSource` (every root and generator) counted against clk_ref, in
  hertz; nullopt when the source died mid-count or the count never
  finished. `measure(...)` is the same count with two bounds and the
  hardware's own PASS / SLOW / FAST / DIED verdict beside the number;
  `start_delay`, `running`, `stop`.
- `ClockOut<n>` (n 0..3 on GP21, GP23, GP24, GP25): `init(GpoutSource,
  div_int, div_frac)`, `stop`, `enabled`, `running`, `source`,
  `divider` / `divider_frac`, and the three controls the always-on
  generators have not - `duty_correction`, `phase`, `nudge`.
  `ClockIn<n>` (n 0..1 on GP20, GP22): `init`, `release`,
  `count_source`, `max_hz`.
- `delay_us(clock, us)` and `delay_us(DelayRate, us)` with
  `delay_rate(hz)`, in `rp2350/delay.hpp`: the microsecond busy-wait,
  AT LEAST and never early, refused at one kernel tick (1000 us) and at
  a rate of zero. It rides the platform timer and not a core counter,
  which is what makes it ONE implementation for two instruction sets -
  and what makes its resolution a microsecond rather than a cycle.

There is NO DYNAMIC CLOCK on this target, as there is none on the
RP2040: this chip has no voltage side to a rate that a program must
sequence, and clk_peri's independence makes a change of clk_sys cheap.
`Clock<...>::init()` called again IS the rate switch, and what it costs
is measured below. A `Rates<>` pack in the STM32G0's shape would be
built when a program wants to scale under the kernel's own users list.

## How to use it

```cpp
using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000>;
constexpr SysClock clock;
const bool clock_ok = SysClock::init();    // first in main()
brio::Mtime::start(clock);                 // the ruler, and delay_us's
Serial::init(clock, 115200);               // divides clk_peri = 150 MHz
```

The crystal alone, for a program that wants 12 MHz and no PLL:

```cpp
using SysClock = brio::Clock<brio::ClockSource::crystal, 12'000'000>;
```

A rate switch under a running program, the UARTs untouched because
clk_peri is on the crystal:

```cpp
using Fast = brio::Clock<brio::ClockSource::pll, 150'000'000, 12'000'000, brio::PeriSource::crystal>;
using Slow = brio::Clock<brio::ClockSource::pll, 48'000'000, 12'000'000, brio::PeriSource::crystal>;
Serial::init(Fast{}, 115200);      // divides clk_peri = 12 MHz, once
...
const bool ok = Slow::init();      // clk_sys to 48 MHz, ~150 us
brio::Ticker::rebase(Slow::hz);    // the Cortex-M33 half's timebase is on clk_sys
```

The USB PLL and the two generators that want its 48 MHz:

```cpp
brio::PllUsb::init(brio::pll_config_for(12'000'000, brio::clk_usb_hz));
brio::Clocks::usb_select(brio::UsbAux::pll_usb);
brio::Clocks::adc_select(brio::AdcAux::pll_usb);
```

A tick for a counter that must keep its microsecond through a rate
change (the platform timer's own, which `Mtime::start` does for you):

```cpp
brio::TickGenerator<brio::TickConsumer::timer0>::start(
    brio::tick_cycles_per_us(SysClock::ref_hz));
```

A clock on a wire, and the wire counted by another board:

```cpp
brio::ClockOut<0>::init(brio::GpoutSource::xosc, 12);      // 1 MHz on GP21
brio::ClockIn<0>::init();                                   // GP20 on the other board
const auto hz = SysClock::count_hz(brio::ClockIn<0>::count_source);
```

A busy-wait, and the one thing it needs first:

```cpp
brio::Mtime::start(clock);            // once, in main(); the ruler
(void)brio::delay_us(clock, 20);      // false if the ruler is not running
```

## Bench findings

All of them on an RP2350 in the QFN-80 package, stepping A2, with a
12 MHz crystal, and all of them on BOTH architectures: `test_rp2350_clock`
reports **114 pass, 0 fail** on the Cortex-M33 pair and **114 pass, 0
fail** on the Hazard3 pair, from one source.

- **The tree after `Clock<pll, 150 MHz>::init()`**: the crystal STABLE
  and ENABLED at delay 47 in range code 0; PLL_SYS locked at REFDIV 1,
  FBDIV 125, post dividers 5 x 2 - a 1500 MHz VCO, which is the ratio
  `pll_config_for` chooses and the datasheet's own recipe; clk_sys on
  its aux undivided, clk_ref on the crystal undivided, clk_peri enabled
  on clk_sys. The counter reads xosc 12 000 000 Hz, clk_ref 12 000 031,
  pll_sys 150 000 000, clk_sys 150 000 000, clk_peri 150 000 000 - every
  one of them within the counting window's own grain of the claim.
- **The other sources, counted**: the ring oscillator at 11.15 to
  11.17 MHz (the datasheet's nominal 11), the low-power oscillator at
  27 906 to 27 937 Hz - 15 per cent slow of its 32 768 nominal, and well
  inside the plus or minus 20 the datasheet allows an untrimmed part.
- **THE SWITCHES**, timed on the platform timer: the PLL to the crystal
  alone 71 us and back 136 us; the PLL re-locked at another rate (48,
  100, 133 MHz - VCOs of 1440, 1500 and 1596 MHz, each at
  `pll_config_for`'s ratio) 115 to 145 us. The crystal is kept
  throughout, which is what keeps every one of them under a
  millisecond.
- **clk_peri on the crystal**: the console re-divided once for 12 MHz
  and then untouched while clk_sys went 150 to 48 and back, and while
  clk_sys was divided by two and by two and a half - every line printed
  across those switches is the proof; clk_peri counted 12 000 000 Hz
  throughout.
- **The 16.16 divider**: clk_sys / 2 counts 75 000 000 Hz and
  clk_sys / 2.5 (INT 2, FRAC 0x8000) counts 60 000 000 Hz, both read
  back as written.
- **`delay_us` at 12, 48, 100, 133 and 150 MHz**: 900 us served as 901
  to 904 us and 100 us as 101 to 108 us. The excess is the ruler's
  one-microsecond grain plus the bracket's two reads, dearest at 12 MHz;
  a whole kernel tick is refused at every rate, a clock with no rate is
  refused, and a wait of nothing is served at once. A hundred 900 us
  waits in a row span 90 kernel ticks - the check that does not share a
  counter with the wait on the Cortex-M33 half.
- **The kernel timebase across rates**: 100 ticks span 100 000 us
  (100 001 at 12 MHz) at 12, 48 and 150 MHz - rebased on the Cortex-M33
  half, where SysTick counts clk_sys, and untouched on the Hazard3 one,
  where the timebase rides a tick divided out of clk_ref and a change of
  clk_sys moves nothing.
- **The ring oscillator, taken apart**: stopped it counts 0 Hz (not
  DIED, which is for a source that dies MID-count) and restarts within
  0.1 per cent of where it was; its divider doubled halves the output
  (11 165 562 -> 5 582 593 Hz); its range stepped LOW to MEDIUM and back
  reads 11 165 562 -> 13 861 937 -> 11 164 687 Hz, which is the
  datasheet's "about 1.33 times"; and THE RANDOMISER raises it by 8 per
  cent with both stages randomised (12 060 562 Hz) and 3 with one
  (11 511 656 Hz), against 11 159 843 Hz with neither - the same
  direction as the datasheet's "up to 22 per cent", and less of it.
- **BADWRITE, and what it caught.** ROSC.BADWRITE stands at every boot
  before brio writes anything: it is not cleared by a processor reset,
  by the debug port's rescue or by the bootrom, so what a program finds
  there is history. Cleared, a whole re-run of `Clock::init()` raises
  NEITHER oscillator's flag. It was raised, during bring-up, by exactly
  the two things the driver now refuses to do: a masked write of CTRL
  through an atomic alias, and carrying ROSC's reset FREQ_RANGE value
  (0xaa0, none of its four codes) into a write.
- **THE PARK-FIRST INIT, from every tree a previous image can leave**:
  from clk_sys on the ring oscillator (11.28 MHz) `init()` takes 96 to
  135 us; from clk_sys on the crystal's aux mux, 95 us; from clk_sys on
  the USB PLL at 48 MHz - a PLL the image did not lock for clk_sys -
  80 us; and from clk_sys parked on a clk_ref that is the 32 kHz
  low-power oscillator, which is a machine running at 32 768 Hz, it
  comes back to 150 MHz with clk_ref on the crystal again. That last one
  is the extreme case of the rule this whole target is written under: a
  debugger's `reset run` resets the cores and nothing else.
- **The low-power oscillator, trimmed**: trim 0 reads 19 656 Hz, trim 63
  reads 35 375 Hz, and the factory 32 comes back to 27 906 Hz - a range
  of 80 per cent over the 63 steps, and the trim restored exactly. What
  the block TELLS the always-on timer (LPOSC_FREQ_KHZ_INT and _FRAC)
  reads 32 kHz + 50332/65536 = 32 768 Hz, its reset value, which is 17
  per cent away from what this die actually runs at: the two are
  independent, and a program that wants the timer to be right measures
  the oscillator and writes the pair.
- **The USB PLL and its generators**: PLL_USB locks at FBDIV 120 over
  6 x 5 (a 1440 MHz VCO) and counts 48 000 000 Hz; clk_usb and clk_adc
  count 48 000 000 Hz each; clk_hstx on clk_sys counts 150 000 031 Hz
  and divided by three 50 000 000 Hz. Stopped, each of them counts 0.
- **The tick generators**: the one the platform timer rides runs at 12
  cycles of clk_ref per tick, which is `tick_cycles_per_us` of this
  crystal; the other five are off in this image. A generator nothing
  uses starts at the count it is given and stops again, and a count that
  does not fit the nine-bit field (512) is refused, as is zero.
- **The top-level gates**: every gate is open at reset (0xffffffff /
  0x7fffffff). Closing one destination's wake gate - the SHA-256 block's
  - clears exactly that bit of ENABLED0 and no other, and opening it
  again puts the register back.
- **The frequency counter itself**: clk_sys counted over 32 us, 1 ms and
  32 ms reads 150 016 000, 150 000 000 and 150 000 000 Hz, each within
  the interval's own grain (64 kHz, 2 kHz, 62 Hz - the table of 8.1.4).
  The test mode answers the bounds it is given: PASS against 149..151
  MHz, FAST against 1..2 MHz, SLOW against 400..500 MHz. A source with
  nothing on it counts 0. Handed back with the NULL source, the counter
  goes idle when the window it is in ends - 3 us or 1002 us, depending
  on where in a 1 ms window the hand-back landed.
- **The resus circuit**: armed with the longest timeout and FORCED, it
  puts clk_sys on clk_ref - counted at 12 000 062 Hz, the crystal - and
  RESUSSED stands. It is POLLED and not read once: a read issued in the
  instruction after the force returned a stale zero, and one poll later
  it stands. Cleared, and with the tree rebuilt, clk_sys counts
  150 000 000 Hz again and the circuit's status is clear.
- **TWO BOARDS, TWO CRYSTALS, BOTH WAYS.** This board's crystal divided
  by 12 on GP21 (`ClockOut<0>`), counted by a peer board on its GP20
  against the peer's own crystal, reads **1 000 000 Hz**; the peer's
  crystal by 12 on its GP21, counted here against this crystal, reads
  **1 000 000 Hz**. At the counter's 62.5 Hz grain on a 1 MHz count that
  puts the two crystals within 63 ppm of each other, with no offset
  visible in either direction - the one external reference this bench
  has, since a chip counting its own oscillators can only ever prove
  their ratios.

## Not covered yet

Driver gaps, each with its reason:

- **A dynamic clock** (`Rates<>` + `DynamicClock`, the STM32G0's shape):
  the section above says why there is none - no voltage side, and a
  cheap switch. Born with the first program on this target that scales
  its rate under load.
- **DORMANT mode** for either oscillator (8.2.6, 8.3.10): entering it
  stops every clock on the chip, and the wake source that brings it back
  is the always-on timer's or a pad's - both of them the POWMAN
  chapter's. The verb belongs with the wake source that makes it safe,
  and is written there.
- **The KILL control** of the four aux generators: the datasheet's own
  note is that it "should not be used", that it can glitch the clock and
  corrupt the logic it drives, and that the design has never been known
  to lock up. Declined, rather than offered with a warning.
- **Duty cycle correction, phase and nudge on clk_usb, clk_adc and
  clk_hstx**: the three controls exist on those generators too and are
  offered on `ClockOut` alone, because that is the one generator whose
  output leaves the chip, where an instrument could see what they did.
  They arrive with a consumer that needs them.
- **`ClockSource::external`** (a clock driven into XIN) and
  **`ClockSource::gpin`** (a generator on a GPIO clock input): the
  resource can already name both - `Clocks::ref_from_aux(RefAux::gpin0)`
  and `SysAux::gpin0` - but no `Clock` task builds a tree on them,
  because no board on this bench has an external oscillator and the
  task's `hz` would be a claim about a wire.
- **A crystal above 25 MHz**, which wants the clk_ref divider the task
  leaves at one: refused at compile time rather than divided in silence,
  and born with a board that carries one.
- **The DFTCLK registers** (`DFTCLK_XOSC_CTRL` and its two siblings):
  production test controls with no programmer's-model section behind
  them.
- **A runtime range check in `PllBlock::init`**: the task's ratio is
  found and checked at compile time, and the resource writes what it is
  given - a program composing its own `PllConfig` owns 8.6.1's
  constraints. What the silicon does outside them is in the findings
  above.

Implemented but not bench-verified, each with what would measure it:

- **The PLLs' lock-loss interrupt.** The sticky flag is read in every
  run (it stands nowhere after `init()`), but nothing has made a locked
  PLL lose its lock: that wants the reference pulled out from under a
  running PLL - stopping the crystal with clk_sys on the PLL - which is
  a state this bench can enter and not one it can leave without the
  rescue. The ISR body and the two lines are compiled and bound by
  nobody.
- **The resus INTERRUPT.** The circuit is armed, forced and cleared on
  the bench; the CLOCKS interrupt line it raises is offered
  (`Resus::irq()`, `isr()`) and no image binds it, because a handler
  that runs after clk_sys has stopped is only useful with a debugger
  attached.
- **The crystal's absolute rate**: the suites prove the PLL's ratio and
  the two boards' crystals against each other, never a crystal against a
  standard. A counter of known accuracy on GP21's output would.
- **The failure paths** - a crystal that does not start, a PLL that does
  not lock: a board with the crystal removed, and a PLL setting the
  silicon refuses. The one tried below its VCO floor (a 192 MHz VCO)
  LOCKED in 64 us, so the range is a promise of the datasheet and not a
  refusal of the silicon; `pll_config_for` is the guard, at compile
  time.
- **`ClockOut`'s fractional divider, and the outputs on GP23..GP25**:
  the integer divider is counted by a peer board, the fraction is not,
  and the other three pins carry no wire - GP25 is this board's LED.
  A counter on those pins would close both.
- **`ClockOut`'s duty cycle correction, phase and nudge**: written and
  read back, and invisible to a frequency counter by construction. An
  oscilloscope on GP21, or a second board with a capture unit.
- **The sleep gates.** `sleep_enables` is written and read back; what it
  does happens when both cores are asleep and the DMA idle, which is the
  power chapter's state and not this one's.
- **A clk_ref that is not 12 MHz**: `tick_cycles_per_us` refuses a
  clk_ref that is not a whole number of megahertz, and only the 12 MHz
  crystal has been on the wire.
- **The QFN-60 package**: the stratum compiles for it and refuses the
  pads it has not got, GP23 and GP24 among them - two of the four clock
  outputs - but no QFN-60 part is on the bench.
