# Sleep (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 6.5 (the
power reduction strategies: 6.5.1 the top-level clock gates, 6.5.2 the
SLEEP state, 6.5.3 DORMANT and 6.5.3.1 what ends it, 6.5.4 the memory
periphery, 6.5.5 the memory power domains), 8.2.6 and 8.3.10 (the two
oscillators' DORMANT registers), 9.5 (the IO bank's dormant-wake
destination), 6.2.2 (the P1.m states, which are off this ladder and in
[powman.md](powman.md)); `docs/design/power.md` for the model the sites
serve. Appendix E carries no erratum against any of it. The driver:
`brio/rp2350/sleep.hpp` (`DormantWake`, `Rp2350SleepSite`,
`Rp2350TimedSleepSite` and the gate sets) over `clock.hpp` (the gate
registers, the two oscillators' `dormant()`), `powman.hpp` (the alarm and
the witness), `pin.hpp` (the wake destination), `platform.hpp` (the idle
path's `sleep_hook`) and each architecture's core header (the one bit of
6.5.2's condition that belongs to the processor). The reference suite:
`test_rp2350_sleep`, letters f, g, h, j, m and the by-name letters n, o,
p, u.

## What the silicon does

**Three depths, and the deepest of them is not a sleep instruction.**

A core's sleep instruction stops that core and nothing else: every
peripheral, the other core and the DMA run on.

THE SLEEP STATE (6.5.2) is reached when both processors are asleep and
the DMA has no transfer in flight. Then the top-level clock gates switch
from the WAKE_ENx masks to the SLEEP_ENx ones - one bit per clock
endpoint, identical layouts, every gate open at reset - and what a
program leaves out of SLEEP_ENx is unclocked until an interrupt of a
block still clocked wakes a core. A peripheral whose gate closed resumes
where it was; no reset, no reinitialisation. The oscillators and the PLLs
run through it. NOTHING IN 6.5.2'S CONDITION IS A PROCESSOR BIT: the
Cortex-M33's SLEEPDEEP plays no part in it and the Hazard3 half has no
such bit, so this rung IS the program's pruning set and nothing else.

DORMANT (6.5.3) is the other thing: the oscillator the program runs on is
stopped by a keyword written into its DORMANT register, every clock
derived from it stands still - the core included - and it restarts on a
GPIO event the IO bank's dormant-wake destination detects or on the
always-on timer's alarm; execution resumes at the next instruction. The
crystal's restart is the datasheet's "more than 1 ms", the ring
oscillator's about a microsecond. The PLLs are not stopped by the
silicon: a program stops them first, and must, because a VCO whose
reference goes away behaves erratically (8.2.6's note).

**And a dormant's alarm needs clk_ref running** - which is this chapter's
one finding that changes how the driver is written. See the bench
findings below: the always-on block runs off clk_ref while the switched
core is powered (SEQ_CFG.USE_FAST_POWCK), a dormant is not a power-down
and gets no fallback from the power sequencer, so a dormant entered with
clk_ref on the oscillator it is about to stop has no way back but a GPIO
event. Both sites therefore leave clk_ref on something the dormant does
NOT stop and take clk_sys to the stopped oscillator through its own aux
mux.

The P1.m states, where the switched core has no supply at all, are OFF
this ladder: leaving one is a boot and not a wake, so a rung of it would
make `arm()` a lie. `Powman::power_down()` is that door
([powman.md](powman.md)).

## Types and verbs

- The gate sets, composed from `SleepClocks` (clock.hpp's type, with
  `|`, `&`, `~`): `sleep_clocks_core` (the bus fabric and its arbiter,
  every SRAM bank, the ROM and boot RAM, the XIP interface the code is
  fetched through, the SIO, the clock and reset infrastructure, the pads
  and the IO controller, the access controller, both oscillators and the
  system PLL), `sleep_clocks_powman` (the always-on block's two
  endpoints - without them a sleeping program cannot be reached by its
  own alarm), `sleep_clocks_timer`, `sleep_clocks_watchdog`,
  `sleep_clocks_uart0` / `uart1`, `sleep_clocks_dma`, `sleep_clocks_pwm`;
  `sleep_clocks_all` and `sleep_clocks_none` are clock.hpp's.
- `DormantWake`: the IO bank's wake destination, which is
  `PinIrqTarget::dormant_wake` and the same four event bits `pin.hpp`
  already names - `enable(pin, PinEvents)`, `disable`, `enabled`,
  `pending`, `raw`, `acknowledge`, `any_enabled`, `disable_all`.
- `DormantSource` (xosc / rosc).
- `Rp2350SleepSite<Clock, source>`: `arm` / `disarm` / `armed`,
  `standby_clocks(set)` and its readback (the pruning set, written into
  SLEEP_ENx at arm and taken back open at disarm),
  `dormant_wake_ready()`, `dormants()`, `go_dormant()` - what the
  platform's `sleep_hook` runs instead of the sleep instruction.
- `Rp2350TimedSleepSite<P, Clock, source>`: `init()` (the alarm's line;
  false while the always-on timer is not running), `alarm_gates`,
  `standby_clocks` (the program's set with the alarm's gates forced in),
  `arm` / `disarm` / `armed`, `resync()`, `isr()` (the body
  `isr_powman_timer` binds), `ready`, `alarm_armed`, `last_advance`, and
  the two conversions `ticks_to_ms` (up) / `ms_to_ticks` (down).
- `Rp2350Platform<core>::sleep_hook`: the function the idle path calls
  instead of its sleep instruction when it is not null, with interrupts
  masked. A dormant site installs it and nothing else does.
- `sleep_releases_power_request(bool)` and its readback, in each
  architecture's core header (`core_m33.hpp`, `core_hazard3.hpp`): the
  half of 6.5.2's condition that is the PROCESSOR's. Empty on the M33,
  MSLEEP.POWERDOWN on Hazard3. The standby rung is the only caller.

## How to use it

```cpp
using Site = brio::Rp2350TimedSleepSite<P, SysClock, brio::DormantSource::xosc>;
using Power = brio::PowerManager<P, Site, brio::PowerConfig{}, Voters...>;

brio::AonTimer::init(clock);                 // the witness, its divisor measured
Site::standby_clocks(brio::sleep_clocks_core | brio::sleep_clocks_timer |
                     brio::sleep_clocks_uart0);
Site::init();                                // the alarm's line
extern "C" void isr_powman_timer() { (void)Site::isr(); }
brio::Tenuto<P, Power, ...>::run();          // the manager arms, the loop's idle sleeps
```

A standby prunes what the program named: a byte transport whose gates are
pruned loses the byte in flight, so the console's set stays in. A dormant
wants a way back and the site refuses to arm one without it - a GPIO
event through `DormantWake::enable`, or the always-on timer's alarm,
which the crystal site accepts only while the timer is not counting the
crystal it is about to stop. After a dormant the tree is restored by the
program's own `Clock::init()` inside the hook, before the wake's handler
runs, so every driver sees the rate it was told.

## Bench findings

The reference suite is `test_rp2350_sleep`, green on both architectures
on the WeAct RP2350B (stepping A2). The rulers: TIMER0's microseconds
where they run, the always-on timer's milliseconds everywhere, and a PWM
slice counting clk_sys / 256 with its gate pruned as the instrument that
says whether the SLEEP state was reached.

- **AS FOUND:** every gate open in both masks, the idle path's hook empty.
  The ladder through the plain site as stated - light arms and writes
  every gate open, standby arms and writes the program's set, disarm
  takes the gates back open, and DEEP IS REFUSED with no way back.
- **A LIGHT SLEEP** ended by an alarm 300 ms out: 301 idle() turns and
  299 267 us. It is not one long stop - the kernel's own tick ends a turn
  every millisecond, which is what makes this rung the kernel's plain
  idle, named.
- **THE SLEEP STATE IS REACHED.** Over a 500 ms sleep the instrument
  counted 30 504 with every gate open and 178 with its own gate pruned -
  the handful left being the awake instants between the turns. The
  microsecond ruler counted 499 006 us of it (its gates kept) and the
  always-on timer 500 ms (it has no gate to keep).
- **AND THE OTHER HALF OF 6.5.2'S CONDITION IS THE PROCESSOR'S, WHICH IS
  WHERE THE TWO ARCHITECTURES DIFFER.** "Both processors are asleep" is a
  signal the core releases, and a Cortex-M33 releases it on any WFI. A
  Hazard3 does not: with MSLEEP.POWERDOWN clear - which is how this
  stratum leaves it, erratum RP2350-E4 being about the NEIGHBOURING bit -
  the instrument counted 30 666 over a pruned 500 ms sleep against 30 591
  unpruned, the same number twice: the chip never left its WAKE masks
  however thoroughly SLEEP_ENx was cut. With the bit set the same sleep
  prunes as the M33's does. 3.8.9 calls the bit's function
  "platform-defined"; on this platform it is the whole of what tells the
  clock controller a processor is asleep. It is the one thing in this
  chapter that a site asks the CORE for, and `core_m33.hpp`'s half of the
  pair is empty.
- **THE KERNEL'S TIMEBASE COUNTS THROUGH THE SLEEP STATE ON BOTH
  ARCHITECTURES**, 499 ticks of that 500 ms: SysTick is core-private with
  no gate in SLEEP_ENx at all, and the platform timer's SIO and tick
  generator are in the set a program keeps. So kernel time never freezes
  in a standby here and a time event matures on its own - which is why a
  standby is hundreds of turns and not one.
- **A STANDBY THROUGH THE TIMED SITE:** a time event 300 ticks out, the
  alarm placed at that distance in milliseconds rounded up, back after
  304 turns and 300 kernel ticks, THE RESYNC ADVANCING ONE TICK - the two
  rulers' phases and not a frozen timebase. The event matures ONE TICK
  AFTER THE ALARM and not on it, which is what the disarm in the alarm's
  body is for: an alarm posts nothing to any queue, so the machine is
  handed back to a ticking sleep and the tick finishes the job.
- **A DORMANT ENDED BY A PAD** needs no clock anywhere. With the board's
  own standing wire GP19 -> GP8 driven high and a level wake armed on
  GP8, the chip stops and comes back in 1 ms of the always-on timer with
  no alarm armed at all, the crystal stable and the PLL relocked.
- **A DORMANT ENDED BY THE ALARM: THE WAKE PATH NEEDS clk_ref RUNNING.**
  This is the chapter's finding and it cost four measured variants. With
  clk_ref on the crystal and the crystal stopped - the RP2040's own
  sequence - the always-on timer's alarm NEVER ends the dormant: not with
  INTE.TIMER enabled and the line unmasked, not with
  SEQ_CFG.USE_FAST_POWCK written to 0 (which the block obeys at once -
  USING_FAST_POWCK follows the write - and which still does not help),
  and not with TIMER.PWRUP_ON_ALARM set as well. The chip sits there
  until the debug port rescues it. What DOES work is leaving clk_ref on a
  source the dormant does not stop: with clk_ref on the LOW-POWER
  OSCILLATOR and clk_sys taken to the crystal through its own aux mux,
  the same alarm ends the same dormant three seconds after it was placed.
  The ring-oscillator site is the same trick the other way round - clk_ref
  stays on the crystal, clk_sys goes to the ring oscillator - and its
  alarm works for the same reason. 6.5.3.1 says an alarm ends a dormant
  and does not say what the alarm needs to be alive; this is what it
  needs.
- **TIMER0 IS NO WITNESS OF A DORMANT OF EITHER KIND.** Across the
  three-second crystal dormant the microsecond ruler counted 239 us, and
  across the ring-oscillator one 280 us: the tick generators of 8.5 divide
  clk_ref for a block that is not running, so the counter behind them
  stands still whichever oscillator was stopped. The always-on timer
  counted every millisecond of both.
- **THE MANAGER** over the timed site: none accepted and nothing armed,
  light and standby armed with the gates written, deep refused by the
  site with the refusal reaching the requester; a standing PowerLock
  clamps a standby round to light; the deadline guard refuses a deep
  round whose next armed time event is one tick away and takes it once
  the deadline is far enough out; the first event after a wake disarms.
- **THE RESCUE ENDS A DORMANT.** With the chip dormant on the crystal and
  a sixty-second alarm under it, the debug port's DAP-only rescue
  (datasheet 3.5.8) completes in about two seconds and leaves the chip
  halted in the bootrom - proven three times, twice against a dormant
  whose wake never came. It is what makes a dormant safe to write on a
  board nobody can power-cycle. What it does NOT do is end a P1.m state;
  [powman.md](powman.md) has that measurement.

## Not covered yet

Driver gaps, each with its reason:

- **The current of each state.** NO CURRENT IS MEASURABLE ON THIS DESK:
  the board is powered through its USB-C connector with no shunt and no
  meter in the path. This chapter proves which states are entered and
  left and what each keeps running; what each draws is not measured
  anywhere in it.
- **The memory periphery power-down** (6.5.4, SYSCFG's MEMPOWERDOWN): a
  bank the program does not use, powered down around a sleep. No program
  here has one to spare, and the caution that a powered-down memory must
  not be accessed makes it a decision a program takes about its own
  linker script.
- **The XIP cache's own power-down** (CTRL.POWER_DOWN, 6.5.4): the
  chapter's own note is that it is unlikely to save anything while code
  goes on being fetched through the QSPI bus, which is every program of
  this project.
- **A dormant of BOTH oscillators**, the second written with the first
  already stopped: the site stops the one clk_sys runs on and leaves the
  other as the program had it. Stopping the other first is `Rosc::stop()`
  or `Xosc::stop()` before the arm, and no letter does it because the
  ring-oscillator site needs the crystal alive for its alarm.
- **The ring oscillator's frequency drop before a dormant** that 8.3.10's
  neighbours recommend: the site runs on it at its boot settings for the
  microseconds between the mux switch and the keyword.
- **A standby with the other core asleep too.** 6.5.2 wants both
  processors asleep, and the instrument says the state is reached with
  core 1 where the flash verb leaves it - parked in the bootrom's
  wait-for-launch. A program that has launched core 1 must put it to
  sleep as well, and what would measure it is this suite's instrument
  with a second kernel running ([multicore.md](multicore.md)).
- **A power vote across the two cores.** `util/power.hpp`'s round asks
  the voters of ONE kernel; the other core's stakeholders are not in it.
  The bridge that would carry a PrepareSleep across is
  `util/inbox.hpp`'s, and it is the same gap the RP2040's chapter left.

Implemented but not bench-verified, each with what would measure it:

- **The wake enables pruned awake** (`Clocks::wake_enables`): a UART
  whose gate is shut while the program runs, reading its registers as
  static. Nothing here prunes the awake mask.
- **The four dormant-wake events on any pad but GP8, and `pending()`.**
  The level wake is measured on the board's own standing wire; the two
  edges and the low level want a pad something else drives, and on this
  desk both ends of every wire are this chip's own pads - which a dormant
  chip cannot move.
- **A dormant on the ring oscillator ended by a pad**, and a crystal
  dormant with both a pad and an alarm armed at once: each wake path is
  measured on its own site, and the pair has not been.
- **`Rp2350SleepSite` on the ring oscillator with the always-on timer on
  the crystal.** The site accepts it - the crystal goes on running
  through that dormant - and every ring-oscillator letter here runs with
  the timer on the low-power oscillator instead.
