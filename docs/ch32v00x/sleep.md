# PWR (CH32V00x)

The stopping half of the platform: RM ch. 2's two low-power modes, the
regulator and the supply monitor, the auto-wakeup unit that counts the
LSI - and the two sleep sites that run util/power.hpp's model over
them, the model unchanged here as on the other three. Documents of record:
the CH32V00X reference manual V1.5 (2.3 for the modes, 2.3.4 and
2.4.3..2.4.5 for the AWU, 6.4 for the EXTI lines the wake sources are),
the QingKe V2 manual V1.3 (5.1 and 5.2 for what a WFI and a WFE do).

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
- **The AWU** counts the 128 kHz LSI through a prescaler (RM 2.4.5,
  fifteen codes from /1 to /61440) up to a six-bit window and raises
  EXTI line 9 at the match; its count cannot be read, and its window
  cannot exceed 2.048 s at /4096 - or 30 s at the /61440 code.
- **The LSI is "about 128 kHz"** (the datasheet's own words).
- **PWR's registers answer rubbish until its gate on the PB1 bus is
  open** (PWREN, RCC_PB1PCENR bit 28): 0x3F in every field on the
  bench, which reads as a plausible configuration to code that trusts
  it.
- **A tick that turns pending ends a WFE** with SEVONPEND set, so a
  running STK never lets a Standby begin.

## Types and verbs

[brio/ch32v00x/sleep.hpp](../../brio/ch32v00x/sleep.hpp): `Pwr`
(`standby(bool)`, the regulator's `ldo()` modes, `pvd(on, level)` and
`supply_low()`, `flash_low_power()`; every verb opens the PB1 gate
first), `Awu` (`init()` turning the LSI on and arming line 9 as an
EXTI interrupt line WITHOUT the PFIC - a flag to poll and a pending
bit a WFE wakes on -, `arm(prescaler, window)`, `fired()`, `clear()`
clearing the flag AND the PFIC's pending bit, `period_us()`), and the
two sites. `Ch32SleepSite<Clock>` is the ladder onto the two modes:
`light` is Sleep, `standby` and `deep` both Standby, the deepest this
silicon has; `disarm()` after a Standby calls `Clock::restore()` (a
static `Clock` re-runs its init, a `DynamicClock` re-applies its
current rate) because the core woke on the HSI.
`Ch32TimedSleepSite<P, Clock>` lifts the no-deadline rule: `init()`
MEASURES the LSI against the STK (64 undivided counts timed in HCLK
cycles), `arm(deep)` converts `TimeEvents<P>::ticks_to_next()` into
LSI counts at the measured rate rounded UP and picks the coarsest
prescaler that keeps the window in six bits, and `disarm()` advances
kernel time by the alarm's span LESS the ticks the STK counted awake
since arm() - when the AWU is what fired; when something else ended
the Standby the span is unknown and nothing is advanced (late
maturation is legal, early is not). [brio/ch32v00x/exti.hpp](../../brio/ch32v00x/exti.hpp)
is the line-level resource under it (`Exti`, `ExtInt<Pin>`).

The platform's `idle()` does the one thing only it can: with SLEEPDEEP
armed it pauses the ticker and clears its pending bit before the WFE
and resumes it after, so kernel time stands still for exactly the
slept span and the tick cannot end the Standby before it begins.

## How to use it

```cpp
using Site = brio::Ch32TimedSleepSite<P, SysClock>;
using Power = brio::PowerManager<P, Site, brio::PowerConfig{}, Voters...>;

Site::init();                       // the LSI on and measured
brio::Kernel<P, Power, ...>::run(); // the manager arms, the loop's idle path sleeps
```

A program that wants a Standby lets the kernel loop idle: a stale
latched event (a USART interrupt from just before) ends the first WFE
at once and the loop sleeps again on the next turn. The first bytes a
peripheral moves after the wake, before `disarm()` restores the clock,
are at the HSI's rate.

## Bench findings

The reference suite is `test_ch32_sleep` (29 verdicts in `z`) on the
CH32V006K8U6 at 48 MHz, the probe attached (so no current was drawn
into a number - the meter with the probe detached is the desk's next
step).

- **The LSI runs at 124 kHz on this part** (123.7..124.6 kHz across
  measurements, -3% of nominal), and a prescaled window scales as the
  divider says within 1% (8 counts at /128 against 64 undivided:
  391087 cycles for 394432 expected).
- **Standby is entered and left.** A time event 300 ms out, the timed
  site placing the AWU at prescaler /1024, window 35 (297 ticks for
  the 296 that remained at arm time - rounded up, the "at least"), two
  `idle()` turns (the first ended by a stale event), the AWU firing,
  SYSCLK read as the HSI at the wake and the PLL back after `disarm()`,
  kernel time advanced by 280 ticks - the span less the seventeen the
  core spent awake printing and waking - and the event due, not early,
  and matured by the next `process()`.
- **Sleep is a light sleep**: armed light, two `idle()` calls cover a
  tick, the tick counts through it.
- **A disarm with no alarm fired advances by nothing**, the clock
  untouched: the accounting's other branch.
- **The PVD at 2.66 V reads a 3.3 V supply as not low**; the regulator
  is in its normal 1.2 V mode as found.

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
- The AWU's long windows (the /10240 and /61440 codes, tens of
  seconds): the suite sleeps 300 ms; a letter outside `z` would wait
  the thirty seconds and time them against the host.
- The wake latency (the first instruction after the AWU's match to
  the PLL back): the cycle counter stops in Standby, so it is a
  scope's measurement on a pad toggled at the wake.
