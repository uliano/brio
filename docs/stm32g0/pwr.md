# Power control and the sleep sites - the stopping half (STM32G0)

> **PROVISIONAL.** Chapter 4's register surface is built whole and the
> two `util/power.hpp` sites over it are bench-verified, including a
> kernel that meets a deadline THROUGH a Stop. What cannot be staged on
> this desk is the analog half - the PVD's crossing (VDD is the
> ST-LINK's fixed 3.3 V), the wake-up pins (they need a wire) and sleep
> CURRENT (there is no meter) - and what is deliberately not built is the
> VBAT charger and the BOR levels, which are option bytes. The list is in
> "Not covered yet".

Documents of record: RM0444 Rev 6 ch. 4 (PWR) with 5.3 (what the RCC
does to a low-power mode) and table 27 (what each peripheral does in
one), DS13560 Rev 5 table 37 (the wake-up times), and errata ES0548
Rev 3 - **2.2.2 and 2.2.4 are live on this silicon**, 2.2.7 and 2.3.1
are revision A only, which is what makes Standby usable here at all.
Drivers: `stm32g0/pwr.hpp` (`Pwr`, the whole chapter) and
`stm32g0/sleep.hpp` (`Stm32SleepSite`, `Stm32TimedSleepSite`,
`Stm32LptimTimedSleepSite`). Family fixture:
`test/family_stm32g0/sleep.cpp` + seven negatives under
`tools/check_stm32g0.sh`. Bench suites: `test_stm32_sleep` for the first
two sites, `test_stm32_lptim` letter h for the third. The waking half of
the platform is [platform.md](platform.md); the RTC that wakes a deep
sleep at the second site is [rtc.md](rtc.md) and the low-power timer
that wakes it at the third is [lptim.md](lptim.md).

## What the silicon does

**The sleep mode is two registers in two places.** The Cortex-M0+ has
one bit, `SCB->SCR.SLEEPDEEP`, which says "the next WFI is a deep
sleep"; `PWR_CR1.LPMS` says WHICH deep sleep. Neither is the other's and
both must agree before a WFI does anything but Sleep. `Pwr` owns them
both - SLEEPDEEP is written there and nowhere else in this stratum - so
that "what is armed" is one question with one answer, `Pwr::mode()`, and
so that `Stm32Platform::idle()` can stay what it always was.

**THE IDLE HOOK NEEDED NO CHANGE FOR ANY OF THIS**, and that is a fact
about the family worth stating beside the other two targets: the AVR's
`idle()` had to learn to honour a standing SEN bit and the SAM's had to
grow an erratum guard, while this one has always been a DSB, a WFI and
an unmask - and since the depth lives in SCR and PWR_CR1, which it never
writes, it has always taken whatever somebody else armed.

**Seven modes, and only four of them are a ladder a program can resume
from.** Run, Low-power run, Sleep, Low-power sleep, Stop 0, Stop 1,
Standby, Shutdown (4.3). Of the ones a WFI reaches: Sleep keeps every
clock but the CPU's; Stop 0 and Stop 1 stop everything in the VCORE
domain but KEEP SRAM AND REGISTERS, differing only in which regulator
supplies VCORE; Standby and Shutdown power the VCORE domain off and come
back through the RESET VECTOR.

**A Stop entered with the kernel's millisecond tick armed LASTS on a
bare board, and does not under a debugger's DBG_STOP** - and telling
the two apart took two campaigns. 4.3.3: entering a low-power mode
through WFI "is executed only if no interrupt is pending", and a 1 kHz
SysTick raises one every millisecond; but once the WFI is taken HCLK
stops and SysTick with it, so the window is one instruction wide.
Measured: a 250 ms Stop 1 asked for with the tick armed lasts 250 ms to
the RTC's own tick with `DBGMCU_CR.DBG_STOP` clear, and 1 ms with it
set - because with that bit the debug logic keeps HCLK running inside a
Stop, SysTick keeps firing, and every millisecond ends the sleep. The
bit survives every reset but a power-on, and OpenOCD's `stm32g0x.cfg`
sets it at every connection whose examine finds `RCC_APBENR1.DBGEN`
open, which is how the first measurement of this fact read "0..3 ms,
three runs of three" and stood for a campaign. `arm()` still pauses the
ticker for the deep rungs and `disarm()` resumes it: it costs NOTHING
(a Stop stops SysTick anyway and kernel time was going to stand still
for the whole sleep either way), it closes the pending-tick window by
construction, and it makes a Stop last whatever a probe left behind.
`Pwr::debug_in_stop()` reads the bit; `tools/bench.py` clears it after
every flash.

**What comes back from a Stop is not what went in.** 4.3.6 and 5.3: the
system clock on exit is HSISYS and the PLL is off, so a program running
at 64 MHz resumes at 16 with a SysTick reload and a USART divisor meant
for four times that. The site is therefore templated on the Clock task
and restores it - and the place it does so is the power model's own
hook, not a new one: `arm()` runs before the machine stops and
`disarm()` on the FIRST EVENT AFTER THE WAKE, which is exactly "put the
clock back". A program on `ClockSource::internal` pays nothing at all,
because HSIDIV survives a Stop and SWS already reads what the task asked
for.

**A Stop that does not happen is not an error anywhere in the silicon.**
Table 31: with a wake-up flag standing or an EXTI pending bit set, "the
Stop mode entry procedure is ignored and program execution continues".
So the only honest way to judge a sleep is BY TIME ELAPSED, never by a
flag - which is what every letter of this chapter's suite does.

**Standby and Shutdown need their own flags clear to be entered at all**
(tables 33 and 34): the WUFx bits in PWR_SR1, and the RTC flag matching
whichever RTC event is meant to end the sleep. A stale flag from a
previous life is an entry that silently does not happen - the same fact
with a second cause. `Pwr::enter()` performs the sweep.

**The I/O pulls for Standby are a separate set of registers from the
GPIO's.** In Standby and Shutdown the GPIO block is unpowered, so a
pad's state comes from PWR_PUCRx / PWR_PDCRx, applied only while
PWR_CR3.APC is set - and they are NOT reset by a wake from Standby.

**Per-part variability, and it is real here.** The six wake-up pins are
a SPARSE set (the G031 bonds 1, 2, 4 and 6; the G071 adds 5; only the
G0B1/G0C1 has all six), the pull registers follow the GPIO bonding (port
E is the G0B1/G0C1's alone), and the VDDIO2 monitor exists only where
the second I/O supply does. All of it is answered by
`stm32g0/device_tables.hpp`, and every verb that takes a pin number or a
port letter checks it and returns false rather than writing a bit that
is not there.

## The ladder, and why it is not the identity

```
  none    -> Sleep      SLEEPDEEP = 0: the CPU clock stops, HCLK,
                        SysTick and every peripheral keep running
  light   -> Sleep      THE SAME MODE
  standby -> Stop 0     SLEEPDEEP = 1, LPMS = 000: every VCORE clock
                        stops, SRAM and registers retained, the main
                        regulator on for the fastest wake
  deep    -> Stop 1     SLEEPDEEP = 1, LPMS = 001: the same, on the
                        low-power regulator - deeper, slower to leave
```

**`light` and `none` are one code, and that is the honest answer rather
than a shortcut.** `util/power.hpp`'s rule is that a target "maps what
it does not have to the nearest SHALLOWER mode - never deeper than
asked". Between Sleep and Stop 0 this family has exactly one thing,
Low-power SLEEP, and it is not a rung a sleep site may take: 4.3.5
reaches it only from Low-power RUN, which means the regulator in
low-power mode and the system clock at or below 2 MHz - a whole-program
decision an application makes, not something to do behind its back for
the duration of one idle. So `light` maps to Sleep, and `armed()` -
which stays a PURE READ of the silicon, the samc position kept - answers
`none` for it, because that is what the machine will really do.

**Standby and Shutdown are off the ladder on purpose**, and this is the
first target where a mode the silicon has is deliberately not a rung.
The power model is built on the program RESUMING: "the manager's next
dispatch - of ANY event - first disarms the site and publishes a
WakeReport". After this family's Standby or Shutdown there is no next
dispatch (4.3.8: "program execution restarts in the same way as after a
reset"). A site that armed one would leave a manager waiting for a wake
that arrives as a reboot. Both remain reachable through
`Pwr::enter(PwrMode::standby)` as a deliberate one-shot, and their
resumption is the application's boot path reading PWR_SR1 and the TAMP
backup registers.

## The timed site

The plain site keeps the v1 HONEST RESTRICTION: with kernel time frozen
for the whole Stop, a program with armed time events must not take one.
`Stm32TimedSleepSite` LIFTS it, with the RTC in both roles - the ALARM
is the periodic wake-up timer placed on `TimeEvents<P>::ticks_to_next()`
rounded UP, and the WITNESS is the calendar plus the sub-second counter,
whose elapsed milliseconds minus what SysTick itself counted is the
FROZEN span handed to `Ticker::advance()`.

**The rate rule is directional, and on this target both halves want the
same direction**: state an RTCCLK rate NOT BELOW the true one.
Over-estimating makes the prescalers divide too hard, so the witness
under-reports and the resync under-advances; and it makes the alarm
arithmetic ask for more counts than needed, so the wake lands late. Both
errors land where the kernel's contract allows: at least, never early.

**The site takes the OTHER prescaler split**, and the choice is its
whole resolution. 30.3.4's advice - a high asynchronous factor, to save
current - is right for a calendar and wrong here: PREDIV_S is how finely
the sub-second counter divides a second, and that counter is the site's
only way of measuring a span the tick did not count. The chapter's
default at 32.8 kHz gives 328 steps a second - three milliseconds a
step, three times the kernel tick - and a resync quantized that coarsely
can advance one tick too many and mature an event EARLY. Measured, and
it did: a 150 ms deadline came back at 149 ms before
`rtc_prescalers_for_resolution()` existed. The config is refused at
compile time unless its split divides the second at least a thousand
ways.

**The ISR has four acts**, and every one is load-bearing. The last three
are the samc's, learned at that bench; the first is this family's own:
restore the clock (4.3.6), acknowledge the flag, resync the ticker, and
hand the machine back to a TICKING sleep - because the never-early bias
guarantees kernel time is still a shade short of the deadline when the
alarm lands, and an RTC wake posts nothing to any queue.

## The third site: the same lift, without the RTC

`Stm32TimedSleepSite` OWNS THE WHOLE RTC and cannot share it. Its
resolution IS the prescaler split, its witness is the calendar and its
alarm is the wake-up timer - so an application that wants a real
calendar at the chapter's own low-power split, or either alarm, or the
wake-up timer for a periodic of its own, cannot use it at all.

`Stm32LptimTimedSleepSite<P, C, cfg>` lifts the same restriction with a
peripheral almost nothing else wants. 26.5 says an LPTIM on LSE or LSI
is unaffected by Stop 0 and Stop 1 and that its interrupts bring the
device out of them, so ONE BLOCK IS BOTH THE ALARM AND THE WITNESS: a
free-running counter at ARR = 0xFFFF is the witness, its compare
register is the alarm, and its own EXTI direct line (29 or 30) is the
wake path. It is a SIBLING of the RTC site, not a replacement - the same
two verbs, the same four ISR acts, the same directional rate rule, and
NOT ONE LINE of `util/power.hpp`, of `kernel/`, or of the two other
sites changed.

**What it owns**: one LPTIM, whole, plus LSEON or LSION in the RCC
(which it turns on and never off). The RTC, both its alarms, its
wake-up timer and its prescaler split stay the application's.

**The source is refused at compile time unless it runs in Stop.** PCLK
stops with the VCORE domain; HSI16 is a clock REQUEST that a
free-running counter never makes (measured - see [lptim.md](lptim.md))
and that ES0548 2.2.4 breaks on a divided HSI anyway. Only LSE and LSI
are rungs.

**The prescaler must leave the counter FINER than the kernel tick**, and
that too is a compile-time refusal: a resync quantized more coarsely
than the tick it repairs can advance a tick too many and mature an event
EARLY. The default /32 on the crystal gives 1024 counts a second - just
over a 1000 Hz tick - and a lap of 64 seconds.

**ONE COUNT IS SPENT ON EACH SIDE, and that is where "never early" comes
from.** A counter reading is a whole number taken at an unknown phase
inside a counter period, so the integer difference of two readings
OVER-STATES the real interval by up to one count - and one count at the
default rate is very nearly one kernel tick. So `resync()` converts
`elapsed - 1` counts and never `elapsed`, and `place_alarm()` asks for
one count MORE than the deadline needs. Without the pair, a 500 ms
deadline matured anywhere in 499..501 ms of wall, a knife edge either
side of its own deadline; with it, 500..501 and never below.

**Three consequences a user must know**, all of them stated because no
register removes them:

1. THE ALARM IS NEVER CLEARED. Nothing may clear `CR.ENABLE` (ES0548
   2.8.1) and `LPTIM_IER` is a disabled-only register (26.7.3), so
   CMPMIE stands for the life of the site and the last compare value
   stays where it was. A round that places no alarm can therefore be
   woken once per counter lap by the previous round's compare. That wake
   is legal by the model - the manager disarms, the site resyncs, the
   next round re-arms - and it costs one loop pass every 64 seconds.
2. AN ARRM IS A LEGAL EARLY WAKE TOO. The lap interrupt is what keeps
   the high word, so it must be armed; `isr()` accumulates, restores the
   clock and returns WITHOUT downgrading the armed mode, so the loop
   idles straight back into the same Stop with the alarm still standing.
3. A MINIMUM DISTANCE IS ENFORCED, because a compare placed behind a
   counter that has already passed it would not match until the next
   lap. The floor is four counts; the write's own cost is measured at
   about 72 us on an LSE kernel clock, a fourteenth of one count, so the
   floor is generous by an order of magnitude.

## Types and verbs

- `PwrMode` {sleep, stop0, stop1, standby, shutdown} with
  `pwr_mode_valid` (LPMS 010 is Reserved and refused),
  `pwr_mode_resets` (does the program come back through the reset
  vector?) and `pwr_mode_stops_clocks`.
- `PvdRising` / `PvdFalling` / `PvdConfig` with `pvd_config_valid`
  (4.2.2's hysteresis rule as a refusal).
- `Pwr` - `bus_clock`, the six raw register readbacks, `range` both
  ways + `range_changing` (VOSF), `rtc_domain_unlock`/`_unlocked` (DBP,
  spelled here too because this is the register that owns it),
  `flash_power_down_stop` / `_lp_sleep` / `_lp_run` + `flash_ready`,
  `low_power_run` both ways + `on_low_power_regulator` +
  `low_power_regulator_ready`, `deep_sleep` (SLEEPDEEP, written here and
  nowhere else), `lpms`, `arm(PwrMode)`, `mode()` (a pure read that is
  total - the Reserved code reads back as the nearest implemented mode),
  `stop_hsidiv_hazard()` (ES0548 2.2.4 as a predicate), `debug_in_stop()`
  / `debug_in_stop(bool)` (DBGMCU_CR.DBG_STOP through its clock gate,
  the gate put back as found - the bit a probe leaves behind that keeps
  HCLK running inside a Stop), `wakeup_pin` /
  `wakeup_pin_enabled` / `wakeup_flag` / `wakeup_pin_present`,
  `standby_flag` (SBF), `internal_wakeup_flag` (WUFI) and
  `internal_wakeup` (EIWUL), `clear_wakeup_flags`, `sram_retention`
  (RRS), `sampled_supply_monitor` (ENB_ULP, offered and never set),
  `pvd_config` / `pvd_enable` / `pvd_below` + `pvd_exti_line` (16),
  `dac_supply_monitor` / `dac_supply_low`, `has_vddio2`, `apply_pulls`
  (APC) + `standby_pull(port, pin, up, down)`, and `enter(PwrMode)` -
  arm, sweep the flags the chapter demands, DSB, WFI.
- `Stm32SleepSite<Clock>` - `arm` / `disarm` / `armed`, the
  `util/power.hpp` concept, plus `resume_clock()` and
  `expected_source`.
- `TimedSleepConfig` {rtcclk_hz, source, wipe_domain, fast_clock} with
  `timed_sleep_config_valid`.
- `Stm32TimedSleepSite<P, Clock, cfg>` - the same three verbs, plus
  `init()`, `ready()`, `place_alarm(ticks)`, `resync()`, `isr()` (the
  four acts) and the readbacks a suite judges it by: `alarm_armed`,
  `last_advance`, `last_reload`, `last_alarm_was_fast`, `prescalers`,
  `fast_hz`, `fast_span_ticks`.

## How to use it

```cpp
#include "stm32g0/sleep.hpp"

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
using Site = brio::Stm32TimedSleepSite<
    brio::Stm32Platform, SysClock,
    brio::TimedSleepConfig{.rtcclk_hz = 32800,
                           .source = brio::RtcClockSource::lse}>;
using Manager = brio::PowerManager<brio::Stm32Platform, Site,
                                   brio::PowerConfig{}, Blinker>;

extern "C" void RTC_TAMP_IRQHandler() { Site::isr(); }

int main() {
    SysClock::init();
    brio::Ticker::init(clock);
    Site::init();                       // owns the RTC, whole
    brio::enable_interrupts();
    brio::Kernel<brio::Stm32Platform, Blinker, Manager>::run();
}

// ...and somewhere in an AO:
//   post<Manager>(SleepRequested{SleepDepth::deep, reply_to<Self, SleepVote>()});
// After ANY wake, speak to the manager again - a path with nothing to
// say says it with SleepRequested{none}. With this site that convention
// is load-bearing: without it the manager never learns the machine came
// back, so the clock stays at 16 MHz AND the kernel's tick stays paused.
```

## Bench findings

The reference suite is `test_stm32_sleep` (eight letters in `z`, 50
verdicts, **50/50 cold and warm, three runs**; letters `s` and `u`
outside it reboot the board, 6/6 each). NOTHING IS WIRED: the RTC is the
wake-up source and the wall clock, and the backstop is the IWDG, armed
once at about 32 seconds and fed at the top of every letter - it cannot
be turned off again (28.3.1), which is the point.

**THE WALL CLOCK HAS TO BE THE RTC.** Every TIM of this family lives in
the VCORE domain and stops in Stop; SysTick rides HCLK and stops with
it. So the suite runs the calendar on the LSE crystal with the
prescalers split the other way round - PREDIV_A 0 and PREDIV_S 32767,
which puts ck_apre at the crystal's full 32768 Hz and makes the
sub-second counter a 30.5 us stopwatch that keeps counting with every
clock in the chip stopped.

- **A STOP ENTERED WITH THE TICK ARMED LASTS, unless DBGMCU_CR.DBG_STOP
  is set**: 250 ms of 250 with the bit clear and 1 ms with it set - the
  same image, the same letter, the bit written and cleared over SWD;
  with the tick paused, 250 ms in both states. The letter reads the bit
  through its clock gate and judges the state it finds. The site pauses
  the ticker for the deep rungs regardless, which is what makes it
  immune.
- **Kernel time stands still across a Stop**: a 250 ms Stop 1 advanced
  the tick by 0 ms.
- **A Stop drops SYSCLK to HSISYS and stops the PLL**, read at the
  instant of waking (SWS = 0 where the program had been running on the
  PLL), and `resume_clock()` puts it back. Nothing may be printed
  between the WFI and that restore: the console's divisor was computed
  for 64 MHz.
- **Sleep does not stop the timebase**: 32 `idle()` calls advance the
  kernel tick by 32 ms over 30..31 ms of wall.
- **What each rung costs to leave is BELOW THIS DESK'S RESOLUTION, and
  the suite says so instead of guessing.** Thirty-two arm-sleep-wake
  rounds measure 7933 us each for Sleep and 7964 for Stop 0, Stop 1 and
  Stop 1 with the flash powered down - a difference of exactly ONE WALL
  TICK (30 us), where DS13560 table 37 puts these wakes at 5.6 us
  (Stop 0) and 9.0 (Stop 1). So the measurement is a BOUND that agrees
  with the datasheet, and the ORDERING the ladder's mapping rests on -
  Stop 1 deeper than Stop 0 - is DECLINED as unmeasured: it rests on
  4.1.3's regulator argument and on table 37, and every counter fine
  enough to resolve it stops in Stop.
- **A pending interrupt makes a Stop fall through in microseconds**,
  measured at 0..30 us where 250 ms was armed. Staging it takes a mask:
  setting an enabled NVIC line pending with interrupts on does not leave
  it pending, because the handler runs at once.
- **ES0548 2.2.4 DOES NOT REACH AN RTC WAKE.** With HSIDIV at /4
  (SYSCLK 4 MHz) - the erratum's own condition - a Stop woken by the RTC
  lasted 250 ms, exactly as asked. The erratum is about peripherals that
  REQUEST HSI16 while stopped (the USARTs, the LPUARTs, I2C1); the RTC
  wakes through the internal wake-up line and EXTI 19 and asks for
  nothing. `Pwr::stop_hsidiv_hazard()` is the predicate for the
  peripherals it does reach.
- **A Stop through a REAL KERNEL, with the plain site**: the vote round
  runs, the site is armed at the depth the target really took, the first
  event after the wake ends the round with nothing polling - and a
  500 ms time event matures after 500 ms of KERNEL tick but **723..741
  ms of WALL**, late by about the length of the Stop. That is the v1
  restriction, measured rather than asserted.
- **THE TIMED SITE MEETS THE DEADLINE ON THE WALL**: the same 500 ms
  event matures after **501 ms of wall**, with the resync handing back
  500 frozen ticks; and six repeats of a 150 ms deadline come back at
  150..151 ms with NOT ONE EARLY. The alarm lands where the stated
  arithmetic puts it (reload 1025 for 500 ms at the stated 2050 Hz), a
  deadline-less round places no alarm, and a round that never slept
  advances at most a tick.
- **Standby and Shutdown both work, and neither is distinguishable from
  the other by any register this chapter offers.** Both come back
  through the reset vector; both leave PWR_SR1 reading 0x8100 (SBF and
  WUFI standing); and both leave **RCC_CSR carrying NO reset flag at
  all** - not PWRRSTF, which 4.3.9 implies a Shutdown wake should raise,
  and not even the catch-all PINRSTF that every ordinary reset of this
  board sets. So a deep wake is invisible to the reset chapter's own
  register, SBF says only "a deep mode happened", and a program that has
  to know WHICH must leave itself a note. The suite's own note is a TAMP
  backup register - the only storage that survives both - and the RTC,
  which kept counting through the sleep (1156..1825 ms of wall between
  entering and the line printed after the reboot).
- **The PVD's crossing is not stageable here**: VDD is the ST-LINK's
  3.3 V and every threshold this detector has sits below it, so the only
  reading this desk can produce is "above", and that is what it produces.
  What IS measured is the refusal of a backwards hysteresis and that the
  detector's EXTI line is a configurable one.
- **ES0548 2.2.2 cannot reach a caller of this driver**: `wakeup_pin()`
  clears WUFx as part of the configuration, which is the erratum's own
  one-line workaround, and the flag reads clear after arming.
- **The internal wake-up line is enabled out of reset** (PWR_CR3 resets
  to 0x8000), which is why an RTC alarm out of Standby needs nothing set
  in this chapter at all.

### The third site, measured (test_stm32_lptim letter h)

- **A 500 ms deadline through a Stop 1 matures at 501 ms of wall and
  never earlier**, with the LPTIM as both the alarm and the witness and
  the RTC untouched; the resync hands back 500 ticks of frozen span.
  Six repeats of 150 ms all land at 151..152 ms and NOT ONE IS EARLY.
  The letter synchronizes to a SysTick edge before stamping the wall,
  which is what makes "never early" a fair test rather than a lucky one:
  a deadline armed at an arbitrary phase inside a tick is only N - 1 to
  N milliseconds of real time away, so a wall reading of 499 for a
  500 ms deadline would be honest and would prove nothing.
- **The alarm arithmetic is exact**: a 500 ms deadline at 1024 counts a
  second places the compare 513 counts ahead - `ceil(500 x 1024 / 1000)`
  plus the one count that pays for the phase of the reading it is
  measured from.
- **THE SITE REALLY DOES NOT NEED THE RTC.** The same letter puts the
  calendar on 30.3.4's own low-power split (PREDIV_A 127 / PREDIV_S 255)
  - a resolution the RTC-backed site refuses at COMPILE TIME - arms
  ALARM A for the application, and still meets a 300 ms deadline through
  a Stop with the alarm standing untouched afterwards. That is the whole
  reason the third site exists. (The wall is a coarse instrument at that
  split - one sub-second tick is 1/256 s, about 4 ms - so the letter
  prints its own quantum and allows for it; the fine-split rounds above
  are where "never early" is judged.)
- **A deadline-less round places no alarm at all**, and a round that
  never slept advances kernel time by at most one tick.

Three suite-craft lessons paid for here, all of them the samc bench's
own in new dress: a console DRAIN placed between arming a deadline and
stamping the wall puts tens of milliseconds INSIDE the measurement (it
made a 500 ms event look 25 ms early); a verdict printed between `arm()`
and `disarm()` is four milliseconds of frozen span the resync then hands
back; and a shared interrupt vector whose body is chosen by a variable
must have that variable set by every letter that drives the peripheral
by hand, or a warm run inherits the previous letter's behaviour.

## Not covered yet

Driver gaps (this chapter's option space the stratum does not touch):
- **The VBAT charger** (PWR_CR4.VBE/VBRS): it drives current into a
  battery this desk has not got.
- **The BOR levels**: they are OPTION BYTES, and writing one belongs to
  the flash chapter's provisioning verb, not to a register here.
- **SLEEPONEXIT and SEVONPEND**: two Cortex bits whose use is a kernel
  design decision - brio's loop returns to the idle path rather than
  staying in a handler - and not a power one.
- **Low-power run and Low-power sleep as a RUNG**: the verbs exist and
  nothing in this stratum calls them, because putting a whole program at
  2 MHz is not something a sleep site may do behind an application's
  back.

Implemented, not bench-verified: `Pwr::range(2)` (the voltage scaling
change itself - this stratum's flash latency table is the Range 1
column), `sram_retention` across a real Standby, `sampled_supply_monitor`,
the VDDIO2 monitor, the DAC supply monitor, and the Standby pull
registers with APC actually set.

The THIRD site's own gaps: it has been run on LPTIM1 and on LSE only -
LPTIM2 as the site's instance and LSI as its source are configurations
the family fixture compiles and the negatives fence, but no letter has
slept on them; and the once-a-lap wake that consequence 1 predicts (a
standing compare firing 64 seconds after a deadline-less round) is
argued from the registers and has not been sat out on the bench.

Not stageable on this desk, and said so rather than left silent: the PVD
crossing and the wake-up PINS (both want a supply or a wire this bench
does not have), and SLEEP CURRENT, which is the number this whole
chapter exists for and which needs a meter. The
`experiments/energy/` tier is where that measurement belongs when it
comes.
