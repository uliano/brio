# Clock - OSCCTRL, GCLK, MCLK (SAM C21)

> **PROVISIONAL.** All three OSCCTRL roots are implemented and
> bench-verified as RESOURCES - the internal oscillator, the external
> crystal and the FDPLL - and the CPU has been run from the DPLL and
> brought back. What is still single-rooted is the TASK:
> `Clock<source, hz>` implements `ClockSource::internal` only, because
> which root CLK_MAIN takes, and who is told when it moves, is the
> `DynamicClock` design decision this target has not taken. The list
> is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M - GCLK ch. 16,
MCLK ch. 17, OSCCTRL ch. 20, the DPLL and RC characteristics of tables
45-52 and 45-57, the flash wait-state table 45-41 - and errata
DS80000740S items 1.2.2, 1.2.3, 1.3.3, 1.3.4 and 1.25.1, all encoded
in code, with 1.22.1 and 1.25.2 read and declared inapplicable or out
of scope below. Driver: `samc/clock.hpp`. The family fixture is
`test/family_samc/clock.cpp` (plus four negatives under `neg/`) under
`tools/check_samc.sh`; the bench suite is `test_samc_clock`.

## What the silicon does

**The tree, and where this driver stands in it.** Clock sources
(OSCCTRL and friends) feed nine **generic clock generators** (GCLK);
generator 0 is the main clock, divided by MCLK's CPUDIV into
CLK_CPU/CLK_AHB/CLK_APBx; the other generators feed **peripheral
channels** - each clocked peripheral names one generator through its
own channel (PCHCTRL), indexed by a per-instance constant from the
device header. Out of reset the device already runs: OSC48M divided
by 12 (4 MHz), generator 0 sourced from it and enabled, CPUDIV = 1.
That reset state is why reaching 48 MHz is one divider write - and
why this driver's init is mostly confirmation: it RE-STATES generator
0 and CPUDIV rather than trusting whatever a debugger or bootloader
left behind.

**Flash wait states bound the frequency** (table 45-41, the
conservative VDD > 2.7 V column): 19 MHz at 0 WS, 38 at 1, 64 at 2 -
and NVMCTRL 27.5.2 orders them adapted BEFORE a rise and after a
fall. The register is NVMCTRL's and so is the verb - `FlashWaitStates`
lives in `samc/nvm.hpp` ([nvm.md](nvm.md)) - but the QUESTION is this
driver's, because nothing may raise the frequency without answering it
first. `init()` and `set()` call it on the correct side of the change.

**Synchronization is per-register and real.** The OSC48M divider
write crosses clock domains (OSC48MSYNCBUSY), each generator's
GENCTRL write has its SYNCBUSY bit, and a peripheral channel must be
DISABLED before its generator changes (PCHCTRL has no
synchronization of its own). Every wait in the driver is BOUNDED and
reported - a clock that never becomes ready is a false return, not a
hang.

**The crystal oscillator is one register.** XTALEN chooses between a
crystal amplifier across XIN/XOUT and a digital input on XIN alone;
GAIN is MANDATORY in crystal mode (20.8.5 says so twice, once under
AMPGC), and it names a recommended MAXIMUM crystal frequency rather
than a range - AMPGC's automatic amplitude control does not replace
it. STARTUP masks the output for 2^n OSCULP32K cycles plus three XOSC
cycles; the table's microseconds assume a nominal OSCULP32K, which
this family's is not, so the driver passes the code through and names
it by its field.

**The clock failure detector watches the XOSC against a safe clock**
derived from OSC48M by a power-of-two prescaler, which 20.6.3
requires to be running FIRST and to be no faster than the crystal it
stands in for. In each window of four safe clocks the XOSC must show
at least one rising and one falling edge; otherwise STATUS.XOSCFAIL
(the chapter's CLKFAIL) is asserted, INTFLAG latches, the XOSC output
is replaced by the safe clock and STATUS.XOSCCKSW (CLKSW) says so.
STATUS TRACKS, INTFLAG LATCHES. SWBEN asks to go back once the
crystal recovers and is cleared by the hardware when the switch
happens.

**The DPLL multiplies in sixteenths.** The DCO runs at
f_ref x (LDR + 1 + LDRFRAC/16) and CLK_DPLL is that divided by the
output prescaler, so **table 45-52's 48..96 MHz applies to the DCO,
not to the output** - the prescaler is how a slower output is reached
and is not a way below the floor. The reference must be 32 kHz to
2 MHz after DPLLCTRLB.DIV, which divides the XOSC reference (by
2 x (DIV+1)) and nothing else. DPLLCTRLB is enable-protected;
DPLLRATIO, DPLLPRESC and CTRLA.ENABLE are write-synchronized.
CLKRDY, not LOCK, is the bit that says a consumer will receive a
clock.

**The two OSC48M errata, as code.** 1.2.2: writing OSC48MDIV while the
oscillator runs UNREQUESTED wedges its sync bit - so init clears
ONDEMAND first. 1.2.3: a rare no-start on parts produced before
2025-01 - mitigated by exactly the same setting, ENABLE = 1 with
ONDEMAND = 0, which init writes and leaves.

**GENCTRL's divider has two regimes and one field.** With DIVSEL
clear the source is divided by DIV, and 0 and 1 both mean undivided
(16.6.2.7). With DIVSEL set it is divided by **2^(DIV+1)**. That
second sentence is worth stating flatly because 16.8.3 words it as
"2^(N+1), where N is the Division Factor Bits for the selected
generator", which reads as a FIXED divisor per generator - 512 on the
eight-bit ones, 131072 on generator 1 - with DIV ignored. It is not:
`test_samc_clock` letter f counts generator 5 against generator 0
with both fed by OSC48M, so the count IS the divisor and no reference
error can enter, and gets **2, 16 and 512 for DIV = 0, 3 and 8**;
generator 1, the sixteen-bit one, gives **512 for the same DIV = 8**.
The field's WIDTH decides only which bits are kept (table 16-3:
sixteen on generator 1, eight everywhere else) - bits written past
the range are ignored, so a linear divisor above 255 means something
on generator 1 alone.

**A generator source change is glitch-free on the fly** (16.6.2.6):
the old source is released only once the new one is ready - which is
what lets init re-state generator 0 while executing from it, and what
makes the TEARDOWN ORDER the one trap of this file. A generator still
pointed at a STOPPED oscillator can never be moved again, because the
new source becoming ready is what releases the old one and the old
one is gone. **Point every generator at something running before
stopping an oscillator**, which is what `Xosc::stop()` and
`Fdpll::stop()` state as a caller obligation and what the bench suite
tests on every path.

**Three DPLL errata are live on this silicon and all three are
code.** 1.25.1: below 25 C the loop reports SPURIOUS unlocks, and
since the lock signal gates the output, everything clocked from the
DPLL stops for the duration - so `FdpllConfig::lock_bypass` defaults
TRUE (the erratum's own workaround) and a caller wanting the gate
back has to ask. 1.3.3: an on-the-fly ratio change does not set
STATUS.DPLLLDRTO although INTFLAG.DPLLLDRTO rises, so
`ratio_updated()` reads the flag and nothing reads that status bit.
1.3.4: the same ratio change needs GCLK_DPLL_32K running, which is
`lock_timer_clock()` and can only be a stated caller obligation.
1.25.2 (ONDEMAND non-functional in standby) is live too and is why
`init()` clears ONDEMAND; standby itself is out of this driver's
scope. **1.22.1 does NOT apply**: the CFD's failure to switch with a
stuck-high input is marked in the N-family row only - reading the
column instead of the row is the trap this errata document sets over
and over.

## Types and verbs

Resources (thin typed register views, `samc/clock.hpp`):

- **`FlashWaitStates`** - get/set, and `for_hz()` folding table 45-41
  at compile time.
- **`Oscctrl`** - the block: the STATUS register all three
  oscillators report into, the seven interrupt sources behind the one
  shared IRQ 0 (with `isr()` answering only for this block), the
  CFD's event output enable and the EVSYS generator code it
  publishes.
- **`Osc48m`** - enable/on_demand/run_standby, ready and its bounded
  wait, the 1..16 output divider (ratio in, ratio out) with its sync
  wait, startup time.
- **`Xosc`** - the external multipurpose crystal oscillator, 0.4 to
  32 MHz: crystal or external-clock mode, the GAIN the chapter makes
  mandatory in crystal mode (derived from the frequency the caller
  states, or overridden), automatic amplitude control, the STARTUP
  masking counter, ONDEMAND/RUNSTDBY, and the clock failure detector
  with its safe-clock prescaler and its switch-back. `init()` waits
  for STATUS.XOSCRDY and returns false rather than hanging when the
  crystal does not start; the compile-time `init<cfg>()` twin refuses
  an impossible configuration outright. **The pads are not claimed:**
  OSCCTRL takes XIN and XOUT itself when the oscillator is enabled
  (20.5.1), and gives them back when it is disabled.
- **`Fdpll`** - the fractional 96 MHz DPLL: the three references
  (XOSC32K, XOSC with its own divider, or a generator through
  `GCLK_DPLL`), the LDR/LDRFRAC ratio with `dpll_ratio_for()` as the
  chooser and `dco_hz`/`output_hz` as the readback, the output
  prescaler, lock and CLKRDY, the on-the-fly ratio change and the
  lock timer's own channel `GCLK_DPLL_32K`. Every constraint of table
  45-52 is one `config_valid()` clause, and `init<cfg>()` turns each
  into a static_assert that names the rule it broke.
- **`Gclk<n>`** (n < 9, refused otherwise) - `configure(GclkConfig)`
  writes the whole GENCTRL in one store and waits the sync:
  source, the divider in both regimes (linear, or 2^(DIV+1) under
  DIVSEL), improved duty, output to the GCLK_IO pad, run-standby.
- **`GclkChannel`** - `connect(channel, generator)` with the
  disable-first discipline, disconnect, the one-way WRTLOCK. The
  channel index is the device header's per-instance constant
  (`SERCOM5_GCLK_ID_CORE` and kin) - a fact of the peripheral, passed
  by its driver, never computed (SERCOM5 breaks any formula).
- **`Mclk`** - CPUDIV (one-hot, refused otherwise) and the AHB/APBx
  bus-clock masks, the bit being each driver's own header constant.

Task:

- **`Clock<ClockSource::internal, hz>`** - the static main clock:
  `hz` is the ONE compile-time truth every driver derives from (there
  is no F_CPU in this build). Any exact OSC48M ratio is accepted -
  48, 24, 16, 12, 9.6, 8, 6, 4.8, 4, 3.2, 3 MHz - and the fractional
  ratios are refused at compile time, because `Clock::hz` must not
  lie. `init()` orders wait states correctly around the change,
  applies the errata discipline, re-states generator 0 and CPUDIV,
  and returns whether the rate is really the one `hz` claims. The
  other `ClockSource` enumerators exist so the vocabulary is stable:
  choosing one is a compile error with an explanation, never a
  silently wrong clock.

## How to use it

```cpp
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

int main() {
    const bool ok = SysClock::init();   // false = the rate is NOT hz
    // a peripheral's driver wires its own core clock:
    //   Sercom<5>::core_clock(0);      // channel from ITS header constant
}
```

The crystal, and a generator on it:

```cpp
// The frequency is the board's, not the register's: GAIN and the CFD
// prescaler are chosen from it, and config_valid judges it.
if (brio::Xosc::init<brio::XoscConfig{.hz = 24'000'000, .startup = 4}>()) {
    brio::Gclk<5>::configure({.source = brio::GclkSource::xosc, .div = 250});
}
// ... and to give it back, MOVE THE GENERATOR FIRST:
brio::Gclk<5>::configure({.source = brio::GclkSource::osc48m});
brio::Xosc::stop();
```

The DPLL, locked to that crystal:

```cpp
// 24 MHz / (2 x (5+1)) = 2 MHz reference, x 24 = a 48 MHz DCO.
constexpr brio::FdpllConfig pll{
    .reference = brio::DpllReference::xosc,
    .reference_hz = 24'000'000,
    .xosc_div = 5,
    .ldr = 23,
};
static_assert(brio::Fdpll::output_hz(pll) == 48'000'000);

brio::Fdpll::lock_timer_clock(6);   // erratum 1.3.4, before any ratio change
if (brio::Fdpll::init<pll>()) {     // waits CLKRDY, not LOCK
    brio::Gclk<4>::configure({.source = brio::GclkSource::dpll96m});
}
```

## Bench findings

`test_samc_clock`, 6 letters / 108 verdicts, wireless. The instrument
is `samc/freqm.hpp`; the reference is the crystal divided by 250
(96 kHz), which at REFNUM 255 makes one count about 8 ppm of a 48 MHz
measurand.

- **THE INTERNAL RC IS HALF A PER CENT SLOW, and this is the first
  time anything here could say so.** OSC48M weighed against the
  crystal measures **47.755 MHz, about 5100 ppm slow** (three runs:
  5051, 5076, 5098 ppm). That is comfortably inside table 45-57 -
  +-5% for the standard factory calibration, +-1% for the enhanced
  one - and it is not a fault. It is a SCALE.
- **The scale matters, because every absolute frequency this stratum
  had reported before was a ratio against OSC48M multiplied by a
  nominal 48 MHz.** OSCULP32K read that way looks like 33074 Hz;
  weighed on the crystal it is **32907 Hz**, about 4200 ppm high
  rather than 9200. The same correction applies to
  `osc32kctrl.md`'s figures and to `test_samc_platform`'s
  watchdog timing, which rides SysTick and therefore the same
  oscillator - which also means those "two witnesses sharing no
  mechanism" shared one after all. The crystal is the first scale
  here that does not come from an RC.
- **The crystal starts in 554..576 us** with STARTUP = 4 (a masking
  time table 20-5 puts at 488 us), measured enable-to-XOSCRDY on a
  46875 Hz stopwatch. Reported from the other side, the crystal
  measures 24.122 MHz if OSC48M is believed exact - the same ratio,
  read the other way up.
- **A clock failure was induced with no wire and detected.** Clearing
  XTALEN with a crystal attached leaves XIN a digital input that
  nothing drives; STATUS.XOSCFAIL rises, INTFLAG latches, and
  STATUS.XOSCCKSW shows the output switched to the safe clock.
  Restoring XTALEN clears the failure and SWBEN is consumed by the
  hardware, after which XOSCCKSW is clear again. Erratum 1.22.1
  confirmed inapplicable by behaviour as well as by the row.
- **The DPLL's ratio arithmetic is exact.** Locked to the crystal
  through a 2 MHz reference and measured against the same crystal -
  so the crystal's own error cancels - the counts are the predicted
  ones to the digit: **127500 for LDR 23 (48 MHz), 130156 for
  LDR 23 + 8/16 (49 MHz), 127500 for a 96 MHz DCO divided by two and
  63750 for the same DCO divided by four.** A sixteenth of the
  reference is a real step.
- **Lock takes about 40 us** from ENABLE to CLKRDY at a 2 MHz
  reference, against table 45-52's 25..35 us for the lock alone.
- **INTFLAG.DPLLLTO does not mean what its name says.** With
  LTIME = 8 ms the loop comes up with CLKRDY = 1, LOCK = 1 and
  DPLLLTO = 1 together: the flag marks the lock TIMER reaching zero,
  which in that mode is simply how the output is released
  (table 20-3), not a failure to lock.
- **Erratum 1.3.3 observed directly**: after an on-the-fly ratio
  change INTFLAG.DPLLLDRTO is 1 and STATUS.DPLLLDRTO is 0, in the
  same reading.
- **THE CPU HAS RUN FROM THE DPLL.** Generator 0 was moved onto a
  crystal-locked 48 MHz loop and back to OSC48M with the console
  alive throughout; measured from the crystal's side while it was
  there, CLK_CPU counted 127500 - the crystal ratio, not the RC's -
  and SysTick kept advancing across both switches. It is a proof, not
  a policy: the driver still leaves CLK_MAIN on OSC48M.
- **GENCTRL's DIVSEL settles as 2^(DIV+1)** - see "What the silicon
  does" above for the measurement and for what it corrects.
- The reset-state verification from bring-up still holds: OSC48MDIV =
  0, OSC48MCTRL = ENABLE with ONDEMAND clear, GENCTRL0 = OSC48M +
  GENEN, CPUDIV = 1, RWS = 2, and the SERCOM baud generator
  programmed from `Clock::hz` produces a byte-exact 115200 console.

## Not covered yet

Driver gaps:
- **The main-clock TASK is still OSC48M only.** `Clock<crystal, hz>`
  and `Clock<dpll, hz>` are vocabulary that refuses to compile,
  deliberately: the resources exist and the switch is proven, but
  which root CLK_MAIN takes belongs with the `DynamicClock` design -
  the discrete-rate surface, the rebase fan-out, and the ticker's
  ClockUser question (`samc/ticker.hpp` documents that caveat and
  refuses the combination mechanically).
- Nothing tells a driver that a generator it uses changed source or
  rate. The AVR's `ClockUser`/`clock_follows` pair has no counterpart
  here yet, and on a target where every peripheral has a generator of
  its own it will not be the same shape.
- XOSC in EXTERNAL-CLOCK mode is written and family-compiled but has
  never run: it needs a clock source on XIN, and this board has a
  crystal there.
- The CFD's event output is enabled and disabled but never CONSUMED -
  `samc/evsys.hpp` could carry it, and the generator code is
  published, but no letter routes it to a user.
- OSC48M's CAL48M calibration register is not exposed. Erratum 1.8.12
  (the accuracy that needs it) is revisions B..E, not this silicon.
- Sleep behaviour of all three oscillators - RUNSTDBY, ONDEMAND, the
  DPLL's erratum 1.25.2 - belongs to the power pass.

Implemented but not bench-verified:
- `GclkChannel::lock`, `Osc48m::run_standby`, `Osc48m::startup`,
  `Gclk`'s IDC and its output to a GCLK_IO pad.
- Rates other than 48 MHz for the main clock (the arithmetic is
  compile-checked; only the undivided rate has run).
- The DPLL's `DpllFilter`, `low_power` and `wake_up_fast`, and the
  XOSC32K reference (this board has no 32 kHz crystal). Only the
  default filter, lock-bypass and the XOSC/GCLK references have run.
- `XoscConfig::automatic_gain` is written in a configuration the
  family fixture compiles, but no measurement distinguishes it.
