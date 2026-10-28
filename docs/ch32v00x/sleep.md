# PWR (CH32V00x)

This document is for a programmer who wants a CH32V006 or a CH32V003 to
spend its idle time in Standby, the deepest mode this silicon has, and
wake for the program's next timer; and for one who needs the supply
monitor (the PVD) or the regulator's modes. It covers the stopping half
of the platform: RM ch. 2's two low-power modes, the regulator and the
supply monitor, the auto-wakeup unit (the AWU) that counts the LSI, and
the two sleep sites that run util/power.hpp's model over them, the
model unchanged ([design/power.md](../design/power.md)). In that model
a sleep site is the target's part: it arms a depth of the ladder
(`light`, `standby`, `deep`, each a rung) before the kernel's idle hook
runs, and puts the chip back after the wake; the `PowerManager`, an
active object of the program (a component with its own event queue,
served one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)), decides the
depth from the votes of the components with something at stake. Here
`light` is Sleep, `standby` and `deep` both Standby. The STK, the
kernel's tick, stops in a Standby, so kernel time stands still across
one: the plain site `Ch32SleepSite` arms a Standby whenever it is
asked, and a program on it asks only with no time event armed - a rule
the program keeps, which the plain site does not check and the manager
checks only in part (it refuses a `standby` or `deep` request when the
nearest time event is nearer than its `min_deep_ticks`, two ticks by
default, [util/power.hpp](../../brio/util/power.hpp), and lets a farther
one through); the timed site `Ch32TimedSleepSite` lifts the rule with
the AWU as the alarm for the next deadline.

The smallest use, the timed site under the manager, from "How to use
it" below:

```cpp
using Site = brio::Ch32TimedSleepSite<P, SysClock>;
using Power = brio::PowerManager<P, Site, brio::PowerConfig{}, Voters...>;

BRIO_CH32_VECTOR(awu_handler) { Site::awu_isr(); }

Site::init();                       // the LSI on and measured
brio::Tenuto<P, Power, ...>::run(); // the manager arms, the loop's idle path sleeps
```

Line by line: `Site` names the timed site over the platform `P`
([platform.md](platform.md)) and the clock type, which the site puts
back after each Standby; `Power` is the manager, with the default
configuration and the program's voters (`Voters...` stands for them);
the `extern "C"` function binds the AWU's vector to the site's body, as
the program binds the tick's; `Site::init()` turns the LSI on and
measures it against the STK, the rate every alarm is computed from; the
manager goes in the list of active objects given to the kernel (the
other `...`). The kernel's loop then idles as always, and the platform's
`idle()` sleeps at whatever depth the site armed.

What decides correctness even there. The AWU's vector must be bound:
the site enables the AWU's line in the interrupt controller, so its
wake is a level the sleeping core cannot miss, and an unbound vector
lands in the startup code's spin. The clock comes back on the HSI: the
hardware switches the system clock to the HSI and turns the PLL off in
a Standby, and `disarm()` restores the clock type's tree (a static
`Clock` re-runs its init, a `DynamicClock` re-applies its current
rate), so the first bytes a peripheral moves after
the wake, before `disarm()`, run at the HSI's rate. A wake the AWU did
not cause advances kernel time by nothing: the AWU has no count to
read, so a Standby a pad or the PVD ended leaves the kernel's tick
short by the slept span, late and never early. In real time the
account errs the other way: with the probe attached a Standby
measured 11 to 15 per cent shorter than the span the site hands the
kernel, so a time event across one comes early on a wall clock
though never early in kernel time (the bench findings). And the
probe: a core in debug mode cannot enter any sleep (QingKe V2 manual
5.1), so a
program measured with the probe attached proves that it sleeps and
wakes, not what the sleep draws.

The rest of this page, in order: the documents of record; what the
silicon does, the two parts' differences closing it; the types and
verbs, with what the timed site guarantees of its wake; how to use it;
the bench findings on both parts, the AWU's edge swept to the cycle
among them; and what is not covered.

## Documents of record

The CH32V00X reference manual V1.5 (2.3 for the modes, 2.3.4 and
2.4.3..2.4.5 for the AWU, 6.4 for the EXTI lines the wake sources are),
the CH32V003 reference manual V1.9 (its ch. 2, the same shape with its
own PVD table), the QingKe V2 manual V1.3 (5.1 and 5.2 for what a WFI
and a WFE do). The header is
[brio/ch32v00x/sleep.hpp](../../brio/ch32v00x/sleep.hpp); the reference
suite is `test_ch32_sleep` (a suite's letter, below, is one command key
of its console and the group of verdicts it runs, `z` the key that runs
the image's set of letters; a letter run "by name" is outside `z`).

## What the silicon does

- **Two modes.** Sleep (SLEEPDEEP clear): the core clock stops, every
  peripheral runs, any interrupt ends it. Standby (SLEEPDEEP set in
  PFIC_SCTLR, PDDS set in PWR_CTLR): the HSE, HSI and PLL stop and the
  peripheral clocks with them, the regulator drops to its low-power
  setting, SRAM, registers and pins hold; it ends only through an
  EXTI line - a pad (lines 0..7), the PVD (line 8), the AWU (line 9)
  - or a reset. Both are entered by WFI or WFE.
- **The core wakes from a Standby on the HSI**, the PLL turned off by
  the hardware (2.3.3).
- **SLEEPDEEP with PDDS clear is not a Sleep**, whatever 2.4.1's PDDS
  row says ("0: Enter sleep mode"): measured, the STK and TIM2 stop
  through it and the core wakes on the HSI, as from a Standby. The
  sites set or clear the two bits together.
- **The AWU** counts the 128 kHz LSI through a prescaler (RM 2.4.5,
  fifteen codes from /1 to /61440) up to a six-bit window and raises
  EXTI line 9 at the match - and again at every period while it stays
  enabled (measured); its count cannot be read, and its window
  cannot exceed 2.048 s at /4096 - or 30 s at the /61440 code.
- **Line 9 reaches a WFE three ways** (QingKe V2 manual 5.2, RM 2.3.1
  and 6.4.2): as an EXTI event (item 1, no handler, no flag), as an
  interrupt the PFIC does not enable, through SEVONPEND's "new
  interrupt pending signal" (item 3 - an edge), and as an interrupt
  the PFIC enables, with MIE set (item 2 - a pending line the core
  takes, the handler run first). The edge has the hole platform.md
  measures: one arriving on the cycle the WFE goes to sleep is lost,
  and while line 9's flag stands the AWU's next match makes no new
  one.
- **The LSI is "about 128 kHz"** (the datasheet's own words).
- **PWR's registers answer rubbish until its gate on the PB1 bus is
  open** (PWREN, RCC_PB1PCENR bit 28): 0x3F in every field on the
  bench, which reads as a plausible configuration to code that trusts
  it.
- **A tick that turns pending ends a WFE** with SEVONPEND set, so a
  running STK never lets a Standby begin.
- **No counter runs in a Standby**: the STK and TIM2 stop with HCLK, so
  a wake that came late cannot be timed from inside; only a wake that
  never comes shows.
- **The two parts differ in PWR_CTLR alone**: the AWU's window,
  prescaler table and line are the same, and so is the PVD's flag;
  the PVD's threshold field is two bits from 1.87 to 2.66 V on the
  CH32V006 and three bits from 2.85 to 4.4 V on the CH32V003 (whose
  supply runs to 5.5 V), and the CH32V003 has neither LDO_MODE nor
  FLASH_LP.

## Types and verbs

[brio/ch32v00x/sleep.hpp](../../brio/ch32v00x/sleep.hpp): `Pwr`
(`standby(bool)`, the regulator's `ldo()` modes and `flash_low_power()`
- the CH32V006's, writing nothing and reading the reset state on the
CH32V003 -, `pvd(on, level)` and `supply_low()` with `PvdLevel` naming
each part's own thresholds, `pvd_rising_mv()` / `pvd_falling_mv()`
saying what a level is and `pvd_level_lowest` / `pvd_level_highest`
spelling the ends of either ladder; every verb opens the PB1 gate
first), `Awu` (`init()` turning the LSI on and arming line 9 as an
EXTI interrupt line without the PFIC - a flag to poll and a pending
bit a WFE wakes on -, `arm(prescaler, window)`, `fired()`, `clear()`
clearing the flag and the PFIC's pending bit, `period_us()`), and the
two sites. `Ch32SleepSite<Clock>` is the ladder onto the two modes:
`light` is Sleep, `standby` and `deep` both Standby, the deepest this
silicon has; `disarm()` after a Standby calls `Clock::restore()` (a
static `Clock` re-runs its init, a `DynamicClock` re-applies its
current rate) because the core woke on the HSI.
`Ch32TimedSleepSite<P, Clock>` lifts the no-deadline rule: `init()`
measures the LSI against the STK (64 undivided counts timed in HCLK
cycles), `arm(deep)` converts `TimeEvents<P>::ticks_to_next()` into
LSI counts at the measured rate rounded up, picks the coarsest
prescaler that keeps the window in six bits and enables line 9 in the
PFIC, `awu_isr()` is the vector's body (it closes the line again,
leaving the flag as the witness), and `disarm()` advances
kernel time by the alarm's span less the ticks the STK counted awake
since arm() - when the AWU is what fired; when something else ended
the Standby the span is unknown and nothing is advanced (late
maturation is legal, early is not). [brio/ch32v00x/exti.hpp](../../brio/ch32v00x/exti.hpp)
is the line-level resource under it (`Exti`, `ExtInt<Pin>`).

The platform's `idle()` does the one thing only it can: with SLEEPDEEP
armed it pauses the ticker and clears its pending bit before the WFE
and resumes it after, so kernel time stands still for exactly the
slept span and the tick cannot end the Standby before it begins.

**What the timed site guarantees of its wake.** The AWU's match ends
the Standby by the one path that is a level: line 9 an interrupt the
PFIC enables, the WFE entered with MIE set, so the match is a pending
line the sleeping core takes (5.2's WFE item 2) whenever it arrives -
before the masked check (the check sees the flag and the loop does not
sleep), between the check and the sleep (taken at the unmask, its
latch ending the WFE at once), on the cycle of the sleep entry, or
during the sleep. The EXTI event stays armed beside it. The price is
the vector: an image using the timed site binds `awu_handler` to
`awu_isr()`, as it binds the tick's.

## How to use it

```cpp
using Site = brio::Ch32TimedSleepSite<P, SysClock>;
using Power = brio::PowerManager<P, Site, brio::PowerConfig{}, Voters...>;

BRIO_CH32_VECTOR(awu_handler) { Site::awu_isr(); }

Site::init();                       // the LSI on and measured
brio::Tenuto<P, Power, ...>::run(); // the manager arms, the loop's idle path sleeps
```

A program that wants a Standby lets the kernel loop idle: a stale
latched event (a USART interrupt from just before) ends the first WFE
at once and the loop sleeps again on the next turn. The first bytes a
peripheral moves after the wake, before `disarm()` restores the clock,
are at the HSI's rate.

## Bench findings

The reference suite is `test_ch32_sleep` (36 verdicts in `z` on the
CH32V006K8U6, and letter `l` by name; on the CH32V003F4P6 two images,
letters `a` to `e` in the first, 28 verdicts there, and letters `w` and
`l` in the second) at 48 MHz,
the probe attached (so no current was drawn into a number - the meter
with the probe detached is the desk's next step).

- **The LSI runs at 124 kHz on the CH32V006** (123.7..124.6 kHz across
  measurements, -3% of nominal) and at 125 kHz on the CH32V003
  (124.6..125.8 kHz, -2%), and a prescaled window scales as the
  divider says within 1% on both (8 counts at /128 against 64
  undivided: 391087 cycles for 394432 expected on the first, 388665
  for 392960 on the second).
- **Standby is entered and left, on both parts.** A time event 300 ms
  out, the timed site placing the AWU at prescaler /1024, window 35
  (297 ticks for the 296 that remained at arm time - rounded up, the
  "at least"; window 36 for 303 ticks on the CH32V003 at its LSI),
  two `idle()` turns (the first ended by a stale event: the deep path
  does not consume the latch, see platform.hpp), the AWU
  firing, SYSCLK read as the HSI at the wake and the PLL back after
  `disarm()`, kernel time advanced by 280 ticks (286 on the CH32V003) -
  the span less what the core spent awake printing and waking - and
  the event due, not early, and matured by the next `process()`.
- **The AWU's long windows** (letter `l`, three runs, the lines time-
  stamped by a host reading the console on its NTP-disciplined clock):
  timed awake on the STK against a reference of /4096 x 8 counts, 32
  counts of the /10240 code read 1001 thousandths of the ten times the
  codes name and 64 counts of the /61440 code 1001 thousandths of the
  120 times - neither code divides by a power of two. The host times the
  same windows at 2.620 s and 31.43 to 31.45 s: the LSI at 125.07 kHz,
  -2.3 per cent of nominal (the STK, on the HSI's PLL, said 125.7 kHz -
  [clock.md](clock.md)).
- **A STANDBY IS SHORTER THAN THE SITE ACCOUNTS IT, the probe attached**
  (letters `l` and `d`, three runs each, host-stamped): a 20 s deadline
  placed at the /61440 code, window 40 - 41 counts, 20.30 s at the 124.1
  kHz the site measured awake - ended after 18.15 to 18.16 s of host
  time; letter d's 300 ms, /1024 and window 36 - 0.303 s planned - after
  0.251 to 0.259 s, its wake and the clock's restoring inside that. The
  AWU counts some 11 to 15 per cent faster than the LSI's awake rate
  across the span, so kernel time, advanced by the planned span, runs
  ahead of the host's by as much: a time event in a Standby comes early
  in real time though never early in kernel time. A core in debug mode
  never sleeps (QingKe V2 manual 5.1), so what the regulator or the core
  does to the count with the probe detached is not separated here.
- **Sleep is a light sleep**: armed light, two `idle()` calls cover a
  tick, the tick counts through it.
- **A disarm with no alarm fired advances by nothing**, the clock
  untouched: the accounting's other branch.
- **The AWU's wake, swept to the cycle** (letter `w`): the AWU
  re-matching every 3044 to 3081 cycles (about 63 us), each try locked
  to one match and the next placed D = 0..399 cycles after the wait
  ends, 16 tries a position, the kernel's masked check and `idle()`
  around it. In a Sleep, timed and rescued by TIM2: the pending edge
  alone (line 9 PFIC-disabled, no event) loses 13 of 6400 tries, all
  between D = 234 and 266 and every one woken only by the 4 ms rescue
  - the AWU's later matches never woke it; the event beside it (the
  site's old shape) loses none; the line enabled (the site's shape)
  loses none, its handler entered once per try that slept. In a
  Standby, the clock put back after each try and the enabled line
  ALONE (no event to fall back on), 12800 tries at 32 a position:
  every one that slept (9721) woke, the handler once each. Twelve code
  layouts (the suite's code shifted by 0 to 22 bytes): the pending
  edge lost 4 to 26 tries in each, the level none in a Sleep and in a
  Standby. Fifty Standbys through the timed site, each a 3 ms
  deadline: all fifty ended by the AWU with one handler entry each,
  the line open after every `arm()` and closed after every
  `disarm()`. The event's own cover in a Standby is not measured -
  a wake one AWU period late would leave no trace a counter could
  read there - and the site does not lean on it.
- **The AWU's walk on the CH32V003F4P6** (letter `w`, behind its IWDG
  net): the AWU at 32 counts there (252 us), because a Sleep of the
  V2A lasts at least 66 us (platform.md) and a period shorter than that
  cannot tell a late wake from one on time. The platform's V2A entry
  sleeps MASKED, so in a Sleep: the enabled line with its event (the
  site's shape) loses none in 6400 tries; the pending edge alone loses
  677, every one woken by the rescue (the control); the event beside a
  PFIC-disabled line - the site's old shape - is late in 616, each
  woken by the AWU's next match and none by the rescue: the latch the
  entry empties first can hold that event, and the PFIC question after
  it asks the ENABLED lines only. In a Standby, the platform's V2A
  entry making the line's interrupt an EVENT for the span: 12800 tries,
  11812 slept, none lost - where the entry the V2C uses ended the walk
  in the net's reboot. Fifty Standbys through the timed site: all
  ended by the AWU, its handler once each.
- **The PVD ladder brackets the supply**: on the CH32V006 no level
  reads low - the ladder ends at 2.66 V under a 3.3 V board - and the
  regulator is in its normal 1.2 V mode as found; on the CH32V003
  board the supply reads not low at the 3.3 V level and low from the
  3.5 V one, so it sits between 3.15 and 3.5 V - the ADC's VREFINT
  puts it at 3.27 V (adc.md).

## Not covered yet

Driver gaps, each with its reason:

- A wake through a PAD (EXTI lines 0..7) or the PVD out of a Standby:
  the site's other-source branch is proven only unslept; a pad wake
  needs a wire to a button or a peer, and the PVD a supply that dips.
- The regulator's low-power and energy-saving modes and the flash's
  low-power setting as a policy: what they are worth is a current
  measurement, which is why the verbs exist and no site sets them.
- SLEEPONEXIT: the "sleep as soon as the handler returns" shape has
  no place in a kernel whose loop decides when to sleep.

Implemented but not bench-verified, each with what would measure it:

- **The current**: Sleep and Standby in microamps, the LDO modes' and
  the flash setting's worth, the WFE's latch consumed or spinning -
  the bench meter, the probe detached (a core in debug mode never
  sleeps, QingKe V2 manual 5.1), the energy experiment's method.
- **The Standby's real length with the probe detached**: attached, the
  AWU runs 11 to 15 per cent faster across a Standby than the site's
  awake LSI (above); letters `l` and `d` host-stamped with the probe
  unplugged - a hand on the bench - would say whether a sleeping core's
  span does the same, and so whether the site's account is to be
  calibrated or the debug mode's doing.
- The handler's count in the CH32V003's Standby walk: of the 10853 to
  11143 tries that called `idle()` in a run, 10305 to 10591 entered the
  AWU's handler, so the verdict "its handler runs once per wake" fails
  there by some 480 to 550 in every run (and the Sleep walk's count is
  one short in some runs) - no try was lost, and which ended through
  the loop's own check of the flag rather than the handler is not
  recorded; a stamp of how each such try ended would say whether the
  walk or the entry is short. The same in an image of the suite from
  before the vectors were aligned and the idle store cleared
  SLEEPONEXIT: neither moved it.
- The wake latency (the first instruction after the AWU's match to
  the PLL back): the cycle counter stops in Standby, so it is a
  scope's measurement on a pad toggled at the wake.
