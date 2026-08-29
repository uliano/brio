# SUPC - Supply Controller (SAM C21)

> **PROVISIONAL.** Configuration, status and the bandgap loop are
> implemented and bench-verified. What is NOT here is anything that
> forces a brown-out - the supply is not a program's to dip - so the
> RESET and INT actions are configured and read back but never fired;
> and the standby behaviour of all three blocks belongs to the power
> pass. The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 22, the
NVM user row mapping of table 9-4, and the BODVDD and regulator
characteristics of tables 45-18 and 45-20 - and errata DS80000740S
item 1.8.14, live on this silicon and named in the code, with 1.8.11
read and declared revision B only. Driver: `samc/supc.hpp`. The family
fixture is `test/family_samc/supc.cpp` plus
`neg/supc_vref_reserved_level.cpp` under `tools/check_samc.sh`; the
bench suite is `test_samc_supc`.

## What the silicon does

**Three of these registers come up with values nobody in the program
wrote.** The BODVDD level, its enable, its action and its hysteresis
are loaded from the NVM user row at every power-on or user reset
(22.6.3.2), exactly as the watchdog's are ([reset.md](reset.md)). So
`samc/nvm.hpp`'s `NvmUserRow` and this driver describe the same four
fields from opposite ends, and `BodVdd::matches_fuses()` exists to say
whether they still agree.

**BODVDD is enable-protected and write-synchronized at once**, a
combination no other register in this stratum has. Writes to the
protected fields are discarded while ENABLE is 1 and raise an APB
error (22.6.3.1); a write of any kind while STATUS.BVDDSRDY is 0
raises a PAC error (22.6.5). Every path in the driver is therefore:
wait for BVDDSRDY, clear ENABLE, wait, write the configuration, wait,
**set ENABLE on its own**, wait. That last step is not cosmetic - see
the bench findings.

**The level is a field, not a voltage.** Table 45-18 gives three
points (level 8 is 2.80 V, level 9 is 2.85 V, level 44 is 4.51 V) and
a "step size" of 60 mV that those points do not support. The driver
passes the field through and names the three anchors; the bench
measures the step rather than trusting either number.

**ACTION = none is not a way of disabling the detector.**
STATUS.BODVDDDET still tracks the comparison, which is what makes a
threshold sweep - raising the level until the detector says the supply
is below it - a measurement instead of a reboot.

**STATUS.BODVDDRDY is never set in sampling mode** (22.8.4), so a wait
on it is only meaningful for a continuous detector. Sampling costs
microamps instead of tens of them (table 45-19) and buys latency; the
sampling clock is OSCULP32K's 1.024 kHz output divided by 2^(PSEL+1).

**BODCORE is not yours.** 22.6.3.4 says it is calibrated in production
and "this configuration must not be changed"; table 9-4 marks its
user-row bits DO NOT CHANGE. The driver exposes it READ-ONLY and
offers no setter at all - the shape is the safety. **Note that the
register exists only on the device header's authority:** chapter 22's
register summary marks offset 0x14 Reserved, and the header also
declares BODCORERDY / BODCOREDET / BCORESRDY at STATUS bits 3..5 that
the chapter does not draw. The bench reads them.

**The main regulator cannot be disabled.** 22.8.6 says VREG.ENABLE
"must never be changed from its reset value of one", so there is no
enable verb - only RUNSTDBY, which erratum 1.8.14 (live on this
silicon) makes the workaround for a standby entry that would otherwise
switch to the low-power regulator and keep requesting GCLK0. Standby
itself is the power pass's business.

**VREF is the bandgap every analog block eventually asks for.** SEL
picks 1.024 V, 2.048 V or 4.096 V - three codes out of sixteen, the
rest Reserved and refused. VREFOE is worded in 22.8.7 as routing the
reference "to an ADC input channel", which undersells it: the ANALOG
COMPARATOR's MUXNEG bandgap selection needs it too, and without this
bit that input is a floating promise. Its absence is what
[ac.md](ac.md)'s gap list was waiting for.

## Types and verbs

- **`Supc`** - the block: STATUS, the six interrupt sources behind the
  shared IRQ 0 (MCLK, OSCCTRL, OSC32KCTRL, PAC and SUPC arrive
  together, so `isr()` answers only for this one), and the APB clock
  verb.
- **`SupcFlag`** - the six flag masks, BODCORE's three included.
- **`BodVdd`** - `configure(BodVddConfig, enable)` spends the whole
  enable-protection dance and refuses rather than writing into a busy
  register; `enable(bool)` starts or stops an already configured
  detector; `level`/`action`/`hysteresis`/`ready`/`detected`/
  `sync_ready` read it back; `matches_fuses()` is the cross-check
  against `NvmUserRow`. `level_2v8`, `level_2v85` and `level_4v51`
  name the three anchors table 45-18 actually gives.
- **`BodAction`** (none/reset/interrupt), **`BodPrescaler`** (div2 to
  div65536) and `bod_sample_mhz()`, the sampling rate in millihertz
  off the nominal 1.024 kHz.
- **`BodCore`** - read-only: `reg`, `enabled`, `action`, `hysteresis`
  and the three status bits the chapter does not draw.
- **`Vreg`** - `enabled()` (a fact, not a switch) and `run_standby`.
- **`Vref`** / **`VrefLevel`** / **`VrefConfig`** - `configure()` in
  runtime and `configure<cfg>()` compile-time flavours (a Reserved
  level is a compile error), `output_enable()` on its own, and
  `vref_mv()` for the nominal millivolts of a level.

## How to use it

Give the analog comparator the bandgap it has been missing:

```cpp
brio::Vref::configure<brio::VrefConfig{.level = brio::VrefLevel::v1_024,
                                       .output_enable = true}>();
brio::Ac::init(0);
brio::AcComparator<0>::configure({.positive = brio::AcPositive::vscale,
                                  .negative = brio::AcNegative::bandgap});
brio::AcComparator<0>::enable(true);
brio::AcComparator<0>::clear_flag();   // erratum 1.5.6, and it is real
```

Watch the supply without consequences:

```cpp
// ACTION = none still sets STATUS.BODVDDDET - a threshold to read,
// not a reset to suffer.
brio::BodVdd::configure({.level = 40, .action = brio::BodAction::none});
const bool below = brio::BodVdd::detected();
```

## Bench findings

`test_samc_supc`, 3 letters / 44 verdicts, 44/44 three times, nothing
wired and nothing forced.

- **THE FUSE ROW AND THE REGISTER AGREE FIELD BY FIELD**: level 8,
  enabled, action RESET, hysteresis off, read from the user row
  through `samc/nvm.hpp` and from SUPC through this driver.
- **THIS BOARD RUNS AT ABOUT 5.1 V, and the bandgap says so three
  times.** The comparator's 64-step VDD scaler crossed INTREF at step
  12 for the 1.024 V level, step 25 for 2.048 V and step 51 for
  4.096 V - VDD of 5251, 5141 and 5090 mV, all within 3% of each
  other, and the crossing step doubles with the reference exactly as a
  real voltage must. This is the first supply measurement on this
  board.
- **THE BODVDD STEP MEASURES 48.7 mV**, which settles table 45-18
  against itself: the table STATES 60 mV typical, while its own three
  anchor points imply about 47.5 mV. The threshold sweep first detects
  at level 56, and with VDD known from the bandgap that is
  (5141 - 2800) / (56 - 8) mV a step. The implied number wins.
- **Enable-protection observed both ways.** A level written to a
  RUNNING detector is discarded and the register keeps the old one;
  the same level through `configure()`, which stops it first, takes.
- **AND THE LAST STEP OF THAT DANCE MATTERS.** A single store carrying
  the configuration AND ENABLE = 1 together sets the bit and leaves
  the protected fields WHERE THEY WERE: the protection is judged on
  the value being written, not on the one already in the register.
  Caught by a restore that did not restore, and now `configure()` sets
  ENABLE on its own.
- **A sampled detector never reports ready**, as 22.8.4 says: 20 ms
  after configuring PSEL = div2 (a 512 Hz nominal sampling clock)
  STATUS.BODVDDRDY still reads 0, where a continuous detector reports
  ready at once.
- **THE CORE DETECTOR IS RUNNING AND THE CHAPTER DOES NOT DRAW IT.**
  SUPC_BODCORE reads 0x0028000A - enabled, action RESET - at an offset
  22.7 marks Reserved, and STATUS bits 3 and 5 (BODCORERDY,
  BCORESRDY) read 1 with BODCOREDET at 0. The device header was right
  and the chapter is incomplete.
- **ERRATUM 1.5.6 IS REAL ON THIS DIE.** Enabling a comparator with
  MUXNEG = bandgap raised a spurious COMP flag at the 2.048 V
  reference (not at 1.024 V) with nothing to flag - which is why
  `ac.hpp` states the clear-before-arming obligation and why this
  suite keeps it.
- The boot BODVDD register is restored bit for bit at the end of the
  letter that moves it, so `z` is re-runnable in one power-on and the
  board is left protected exactly as it was found.

**INTFLAG.BODVDDDET IS A TRANSITION AND NOT A LEVEL**, and that is why
this block could not be made a standby wake source here. With the level
set to 63 - above this board's ~5.1 V - STATUS.BODVDDDET reads 1 and
stays 1, and the detector reports READY; but clearing the flag under
that standing condition does NOT bring it back, awake or across a
standby, and NOT EVEN TO A SAMPLING DETECTOR configured to sample in
standby at the fastest prescaler. So a detection is a crossing, and
nothing on this board can make one with the CPU stopped. ACTION = none
throughout, so nothing was ever forced, and the board's boot BODVDD
word is restored bit for bit.

## Not covered yet

Driver gaps:
- **Nothing forces a brown-out.** `BodAction::reset` and
  `BodAction::interrupt` are written and read back but never fired -
  that needs a supply this program does not control. The BODVDD
  interrupt has an arm/disarm surface and an ISR body it shares with
  the block, and no letter has ever seen it fire.
- **The BODVDD as a standby WAKE SOURCE**, and with it any effect of
  `run_standby` / `sampled_in_standby`: a detection is a SUPPLY
  CROSSING and nothing on this board can make one while the CPU is
  stopped. What that costs is measured rather than assumed - see "Bench
  findings" - and forcing a brown-out stays out of scope.
- **`Vreg::run_standby`** (erratum 1.8.14's workaround) and
  `VrefConfig::run_standby` / `on_demand` (table 22-1): all written and
  read back, none observed across a sleep. The regulator's own bit is
  exercised for a different purpose in [platform.md](platform.md),
  where it moved nothing measurable.
- **BODCORE is read-only by design** and will stay so: its calibration
  is a production value the user row marks DO NOT CHANGE.
- No level-to-millivolt conversion is offered, because the datasheet's
  own two numbers for the step disagree. The measured 48.7 mV is a
  bench finding on one die at one supply, not a driver constant.

Implemented but not bench-verified:
- `BodPrescaler` codes other than div2, and the sampling latency they
  buy.
- `Vref::level()` 4.096 V as a working reference for a converter - it
  was only used as a comparison level here.
- `Supc::bus_clock(false)`, which would make the block unreachable and
  has never been exercised.
- BODVDD hysteresis is configured and read back, never measured: at
  40..75 mV (table 45-18) it is narrower than one level step, so a
  threshold sweep cannot resolve it.
