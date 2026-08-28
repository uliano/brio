# OSC32KCTRL - the 32.768 kHz oscillators (SAM C21)

> **PROVISIONAL.** The chapter is built; XOSC32K is written and
> family-compiled but cannot be exercised - the bench board carries no
> 32 kHz crystal - and the clock-failure detector's event output waits
> for an EVSYS driver. The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 21, with the
calibration area in 9.4 - and errata DS80000740S, where **neither item
touching this chapter applies to this silicon**: 1.1.1 (the CFD's
automatic switch fails when XOSC32K is requested by the GCLK) is marked
for E/G/J revision **B only**, and 1.22.1 (the CFD cannot reach the safe
clock when the input is stuck high) is marked only in the **N-family
row**. Both are the shape of trap `dmac.md` already records: read the
row, not the column. Driver: `samc/osc32kctrl.hpp`. Family fixture
`test/family_samc/osc32kctrl.cpp` plus one negative under
`tools/check_samc.sh`; the bench suite is `test_samc_osc32k`, which
measures with `samc/freqm.hpp`.

## What the silicon does

**Three slow roots, and only one of them is free.** OSCULP32K is enabled
by a power-on reset and runs until the next one - there is nothing to
start and nothing to wait for. OSC32K is OFF at reset and must be
enabled, trimmed and waited for. XOSC32K needs a crystal on the board.

**OSC32K IS NOT CALIBRATED UNTIL SOFTWARE CALIBRATES IT**, and this is
the fact worth carrying away. 21.5.9 is blunt: the production value
"must be loaded from the NVM Software Calibration Area into the OSC32K
register (OSC32K.CALIB) by software to achieve specified accuracy". An
oscillator enabled without it runs at whatever an untrimmed RC does -
measured here, **44% fast**. The value lives in the area `samc/nvm.hpp`
already reads, and `Osc32k::factory_calib()` is the one call that closes
the loop.

**An oscillator's output is separate from the oscillator.** EN32K and
EN1K gate the two outputs, and 21.6.4 requires the one a consumer wants
to be enabled *before* the GCLK or the RTC is pointed at it. An enabled
oscillator with both outputs off is a clock nobody can reach - which the
driver refuses as a configuration rather than leaving it to be
discovered.

**CALIB is committed on ready.** "When writing the calibration bits, the
user must wait for the STATUS.OSC32KRDY bit to go high before the new
value is committed" (21.6.4) - so writing a trim and measuring
immediately reads the old frequency.

**A generator cannot be moved off a stopped source.** Not in this
chapter at all, but in GCLK's 16.6.2.6: a generator releases its old
source only once the new one is ready. Stop an oscillator while a
generator still points at it and that generator can never be
re-sourced - the GENCTRL write does not complete. Point the generator
somewhere running *first*, then stop the oscillator. This cost the bench
suite a whole letter before it was understood.

**WRTLOCK is one-way until a power-on reset**, on both internal
oscillators. The worst it does is freeze a working configuration, but
nothing short of unplugging the board undoes it, so every configuring
verb checks the lock and refuses rather than writing into it.

**The RTC's clock is chosen here**, in RTCCTRL.RTCSEL, among all six
oscillator outputs - even though the RTC is another chapter. 21.6.7 asks
for the RTC to be disabled before the selection changes; this driver owns
the register and the ordering belongs to the RTC driver when it exists.

**The watchdog's clock is not chosen at all**: the WDT takes OSCULP32K's
1.024 kHz output, always, internally requested (21.6.6). So trimming
OSCULP32K moves every watchdog timeout with it.

## Types and verbs

**`RtcClock`** - the six RTCSEL codes: `ulp_1k`, `ulp_32k`, `osc_1k`,
`osc_32k`, `xosc_1k`, `xosc_32k`.

**`Osc32kctrl`** - the block: `status` and the decoded `osc32k_ready`,
`xosc32k_ready`, `clock_failing` (STATUS.CLKFAIL), `clock_switched`
(STATUS.CLKSW); `flags`/`armed`/`clear_flags`/`arm`/`disarm`/`isr`; and
`rtc_clock()` both ways. Note the interrupt line is SHARED - IRQ 0
carries MCLK, OSCCTRL, OSC32KCTRL, PAC and SUPC together - so a handler
must ask each block in turn and `isr()` answers only for this one.

**`Osculp32k`** - `calib()` both ways (five bits), `locked()`, `lock()`.
Nothing to enable.

**`Osc32kConfig`** / **`Osc32k`** - `calib` (seven bits),
`enable_32k`/`enable_1k`, `startup`, `run_standby`, `on_demand`.
`factory_calib()` fetches the production trim; `config_valid()` refuses a
field overflow and the both-outputs-off case; `init()` configures, starts
and waits for ready; `retrim()` changes the trim of a running oscillator
and waits for the commit; `stop()`, `lock()`, and the readbacks `reg`,
`enabled`, `locked`, `ready`, `calib`.

**`Xosc32kConfig`** / **`Xosc32k`** - the same shape plus `crystal`
(XTALEN: a crystal, or an external clock on XIN32 alone), and the failure
detector: `failure_detector(on, prescale)`, `switch_back()` (CFDCTRL.
SWBACK). Its `init()` has a generous default bound because a crystal
takes hundreds of milliseconds; a false return means the crystal is not
oscillating, which on a board without one is the correct answer rather
than a fault.

## How to use it

**Start OSC32K properly**, which means with its production trim:

```cpp
Osc32k::init(Osc32kConfig{
    .calib = Osc32k::factory_calib(),   // 21.5.9 - not optional
    .enable_32k = true,                 // or nothing can reach it
});
Gclk<5>::configure(GclkConfig{.source = GclkSource::osc32k});
```

**Give the RTC a clock**, before the RTC is enabled:

```cpp
Osc32kctrl::rtc_clock(RtcClock::ulp_1k);
```

**Stop an oscillator safely** - move whatever points at it first:

```cpp
Gclk<5>::configure(GclkConfig{.source = GclkSource::osculp32k});  // always running
Osc32k::stop();
```

## Bench findings

From `test_samc_osc32k` (3 letters, 32 verdicts, 32/32), measured with
`samc/freqm.hpp` against OSC48M. Nothing to wire.

**THE SCALE, first.** Every absolute frequency below is a ratio against
OSC48M multiplied by a NOMINAL 48 MHz - and OSC48M, weighed against the
crystal after this page was first written, is about **5100 ppm SLOW on
this die** with a +-5% calibration spec and a thermal wander of its own
([clock.md](clock.md)). So these numbers overread by about half a per
cent, they move a little between power-ons, and the suite's near-nominal
verdict carries a 3% band for exactly that reason - the band covers the
REFERENCE more than the oscillator under test. The per-step and
percent-class findings below are unaffected; the crystal-scale values
live in [clock.md](clock.md).

- **The production trim is worth 44%.** OSC32K started with CALIB at
  zero - what a caller who never read 21.5.9 gets - measures
  **47312 Hz** against a nominal 32768. Retrimmed with the production
  value (65 on this die) it measures **32995 Hz**, six per mille off.
  The chapter's insistence is not decoration.
- **OSCULP32K's trim is a coarse knob**: the factory value (14 on this
  die) gives 32979 Hz, and moving it by 8 steps drops the oscillator to
  25801 Hz - about 900 Hz per step. Since the watchdog runs on this
  oscillator, a program that trims it moves every timeout.
- **The two internal RCs are indistinguishable at one operating point.**
  Trimmed, they land within a couple of per mille of each other and of
  nominal - around 32960..33010 Hz for both - and WHICH OF THEM IS
  NEARER FLIPS BETWEEN RUNS. 21.6.5's ordering ("OSCULP32K should be
  preferred whenever power requirements are prevalent over frequency
  stability and accuracy") is about stability across conditions, and a
  single measurement at room temperature cannot see it; a verdict that
  asserted the ordering was a coin toss and has been removed. The
  OSCULP32K figures are consistent with the measurements in
  [freqm.md](freqm.md) and [reset.md](reset.md) - which, as freqm.md
  now records, all share the OSC48M scale rather than witnessing it
  independently.
- **A missing crystal is a false return, not a hang.** XOSC32K started
  on a board with no 32 kHz crystal never raises its ready flag, and the
  bounded wait reports that instead of spinning.
- **A generator cannot be moved off a stopped source** (16.6.2.6),
  which the suite learned by doing it: stopping OSC32K while generator 5
  still pointed at it left that generator unroutable and every later
  measurement empty. The rule is to point the generator at something
  running first.

## Not covered yet

Driver gaps (not built):

- **The CFD's event output** (EVCTRL.CFDEO): no EVSYS driver on this
  target, so the failure detector can raise an interrupt but not an
  event.
- **A ClockUser relationship.** These oscillators feed generators, and
  nothing here tells a driver its clock moved - the same gap
  `clock.md` records for the target as a whole.

Implemented but not bench-verified:

- **XOSC32K entirely**: written and family-compiled, and it cannot be
  exercised on this board, which has no 32 kHz crystal. Its startup
  codes, the external-clock mode (XTALEN clear), the failure detector
  and `switch_back()` are all in that state.
- `run_standby` and `on_demand` on either internal oscillator: set and
  read back, never observed across a sleep - the power pass owns that.
- `lock()` on any of the three: writing it would freeze the
  configuration until someone unplugged the board, so no test sets it.
- Operation on the E and G variants: compile-checked only. Nothing in
  this chapter varies by package.
