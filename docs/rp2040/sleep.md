# Sleep (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 2.11 (power
control: 2.11.1 the top-level clock gates, 2.11.2 the SLEEP state,
2.11.3 the DORMANT state, 2.11.5 the programmer's model), 2.16.5 and
2.17.7 (the crystal's and the ring oscillator's DORMANT registers),
2.19.6.3 (the IO bank's dormant-wake interrupt), 4.8.5.5 (the RTC as
the wake of a dormant); docs/design/power.md for the model the sites
serve. The driver: `brio/rp2040/sleep.hpp` (`DormantWake`,
`Rp2040SleepSite`, `Rp2040TimedSleepSite`) over `clock.hpp` (the gate
sets, `Clocks::sleep_enables`, the two oscillators' `dormant()`),
`platform.hpp` (the idle path's `sleep_hook`), `timer.hpp` and
`rtc.hpp`. The reference suite: `test_rp2040_sleep`, wireless.

## What the silicon does

A core's WFI stops that core and nothing else. When both cores are
asleep and the DMA has no transfer in flight the chip is in its SLEEP
state, and the top-level clock gates switch from the WAKE_ENx masks
to the SLEEP_ENx masks (one bit per clock endpoint, identical
layouts, all open at reset): what a program leaves out of SLEEP_ENx
is unclocked until an interrupt of a block still clocked wakes a
core, and a peripheral whose gate closed resumes where it was, no
reset needed. The oscillators and the PLLs run through it. DORMANT
is the other thing: the oscillator the program runs on is stopped by
a keyword written into its DORMANT register, every clock derived from
it stands still, the core included, and the oscillator restarts on a
GPIO event the IO bank's dormant-wake logic detects (a level or an
edge, four bits a pin) or on the RTC's interrupt; execution resumes
at the next instruction, the crystal after its startup delay, the
ring oscillator in about a microsecond. The PLLs are not stopped by
the silicon. With no wake configured the keyword is the last
instruction the chip executes.

Three facts measured that the chapter does not state as they stand:
THE SLEEP STATE IS REACHED ON A PLAIN WFI - core 0 in WFI, core 1 in
the bootrom's WFE, SLEEPDEEP clear - exactly as 2.11.2's rule reads,
and the note asking for deep sleep on both cores describes the SDK's
example, not a condition; SYSTICK COUNTS THROUGH THE SLEEP STATE, a
tick a millisecond ending each turn, so kernel time never freezes in
a standby and a time event matures on its own; and the RTC's
interrupt wakes a dormant RING OSCILLATOR while the crystal keeps
clk_rtc, which is the dormant with a deadline this stratum offers.

## Types and verbs

- The gate sets (`clock.hpp`): `SleepClocks` (en0, en1, the operators
  `|`, `&`, `~`), `sleep_clocks_all`, `sleep_clocks_none`,
  `sleep_clocks_core` (the fabric, the memories, the ROM and the
  XIP cache, the SIO, the pads and the IO controller, the clock and
  reset infrastructure, the timer and the watchdog), the sources
  `sleep_clocks_uart0` / `uart1` / `rtc` / `dma` / `pwm`;
  `Clocks::sleep_enables(set)` and its read-back, `wake_enables`,
  `enabled()` (ENABLED0/1: the wake set less the clocks whose
  generator is stopped). `Xosc::dormant()`, `Rosc::dormant()`: the
  keyword, then the wait for STABLE.
- `DormantWake`: `enable(pin, events, on)` with `DormantWakeEvent`
  (level_low, level_high, edge_low, edge_high; `|` combines,
  `dormant_wake_bits`), `enabled(pin)`, `raised(pin)` (the raw flags
  shared with the cores' interrupts), `pending(pin)`,
  `acknowledge(pin, events)` (an edge flag cleared; a level flag
  follows the pad), `any_enabled`, `disable_all`.
- `Rp2040SleepSite<Clock, source>`: the ladder - `light` a WFI with
  SLEEPDEEP clear, `standby` the SLEEP state with SLEEPDEEP set and
  the program's SLEEP_ENx masks, `deep` DORMANT on the oscillator
  `DormantSource` names (`xosc`: everything stops, a GPIO event the
  only way back; `rosc`: clk_rtc survives on the crystal, the RTC's
  alarm is a wake too) - `arm` / `disarm` / `armed`,
  `dormant_wake_ready()` (a GPIO wake enabled, or on the ring-
  oscillator site the calendar running with its alarm armed and its
  interrupt on; arm(deep) refuses without one), `dormants()`,
  `go_dormant()` (what the platform's `sleep_hook` runs instead of
  the WFI: clk_sys onto clk_ref, clk_ref onto the oscillator that
  stays, the PLL stopped, the keyword, then `Clock::init()`).
- `Rp2040TimedSleepSite<P, Clock, source, alarm_n>`: `init()` (the
  alarm's line; false while the timer's tick is not running),
  `timer_gates` (forced into the SLEEP mask by a standby), `arm` (a
  standby: the nearest deadline on the timer alarm, microseconds up;
  a dormant with a deadline: the calendar's alarm at h:m:s whole
  seconds up, refused on the crystal site, without a running
  calendar, or a day or more out), `disarm` (the alarms taken back,
  `resync`), `resync()` (the witness's span down to ticks - the
  timer's microseconds, or the calendar's seconds - less the ticks
  SysTick counted, once), `isr()` (the alarm's vector: acknowledge,
  resync, the plain site disarmed so the next idle ticks),
  `rtc_isr()` (isr_rtc: the RTC's body, then the same),
  `alarm_armed`, `rtc_alarm_armed`, `last_advance`, `ready`.
- `Rp2040Platform<core>::sleep_hook`: the function the idle path
  calls instead of DSB-WFI when it is not null, with interrupts
  masked; a dormant site installs it, and nothing else does.

## How to use it

```cpp
using Site = brio::Rp2040TimedSleepSite<P, SysClock, brio::DormantSource::rosc>;
using Power = brio::PowerManager<P, Site, brio::PowerConfig{}, Voters...>;

brio::Clocks::sleep_enables(brio::sleep_clocks_core | brio::sleep_clocks_uart0 | brio::sleep_clocks_rtc);
Site::init();                            // the timer's alarm line
brio::Rtc::init(clock); brio::Rtc::set(now);   // a dormant with a deadline wants the calendar
extern "C" void isr_timer_3() { Site::isr(); }
extern "C" void isr_rtc() { (void)Site::rtc_isr(); }
brio::Kernel<P, Power, ...>::run();      // the manager arms, the loop's idle path sleeps
```

A standby is a WFI whose SLEEP_ENx masks prune what the program
named: a byte transport whose gates are pruned loses the byte in
flight, so the console's set stays in. A dormant on the crystal wants
a GPIO wake - `DormantWake::enable(pin, ...)` - and a dormant on the
ring oscillator takes the calendar's alarm as well; the site refuses
to arm a dormant with no way back. After a dormant the tree is
restored by the program's own `Clock::init()` before the wake's
handler runs; the first bytes a peripheral moved before the dormant
are the last it moves at the old rate.

## Bench findings

The reference suite is `test_rp2040_sleep`, green on the WeAct board
(52 verdicts in the all-key), every letter wireless: the system timer
as the ruler, a PWM slice with its gate pruned as the instrument that
says whether the SLEEP state was reached.

- AS FOUND: both masks all open, ENABLED the wake set less clk_usb
  (no USB PLL), SLEEPDEEP clear, no dormant wake, the hook empty; the
  ladder through the plain site as stated, deep refused on both sites
  until a GPIO wake is enabled, then armed with the hook installed.
- A LIGHT SLEEP is ended by the tick inside one idle() call.
- THE INSTRUMENT: awake, the slice counts 9766 in 20 ms at clk_sys /
  256; across a 20 ms WFI with its gate pruned it counts 25 with
  SLEEPDEEP clear and core 1 in the bootrom, 8 with SLEEPDEEP set, 8
  with core 1 launched into a deep WFI of its own - the SLEEP state
  in all three, the counts being the awake instants between the
  turns; the timer counts every leg's 20 ms with its gates kept,
  SysTick its 20 ticks, and a tick ends each of the 21 turns.
- A STANDBY THROUGH THE TIMED SITE: a time event 300 ms out places
  alarm 3, the loop takes 258 turns in 257 ms with the instrument at
  120, the alarm's body clears SLEEPDEEP, kernel time from arm to the
  wake covers the deadline with nothing to advance (the tick counted
  through), the event due and matured.
- A STANDBY ENDED BY SOMETHING ELSE: the site's alarm 2 s out, the
  suite's own at 100 ms: 101 turns, kernel time moved 100 ticks, the
  deadline still 1874 ticks out, nothing advanced.
- A DORMANT ON THE RING OSCILLATOR WITH THE CALENDAR AS THE WAKE: a
  deadline 2 s out places the alarm at +2 s; one turn, the calendar
  0 -> 2 s while the timer saw 580 us (clk_ref stood with the ring
  oscillator), the tree back on the PLL, kernel time advanced by 1954
  ticks (the two seconds less the awake ticks), the event due.
- A DORMANT ON THE CRYSTAL WITH A LEVEL WAKE on the console's RX pin
  (idle high) returns in one turn, the crystal restarted, the PLL
  relocked, the console printing at its rate; the same with an EDGE
  wake (the letter outside the all-key) sleeps until a key's start
  bit.
- THE MANAGER over the timed site: none accepted and nothing armed,
  light and standby armed with the register as stated, deep refused
  by the site and reported; a veto ends the round; a deadline one
  tick away is refused by the guard; a 300 ms round arms standby with
  the alarm placed, the kernel's own idle hook takes it a turn a tick
  for the 50 ms a foreign alarm posts a Blip in (50 ticks, 51 turns),
  the site still armed until the first event reaches the manager,
  which disarms and publishes the WakeReport of standby.

## Not covered yet

Driver gaps, each with its reason:

- The current of each state: the bench meter's, with the probe
  detached and both cores idle; this suite proves the states are
  entered and left, not what they draw.
- The memory power-down (2.11.4, SYSCFG's MEMPOWERDOWN): a bank the
  program does not use, powered down around a sleep; no program here
  has one to spare.
- A dormant of BOTH oscillators (the crystal written last with the
  ring oscillator already stopped): the site stops the one it runs on
  and leaves the other as the program had it; the ring oscillator's
  stop is `Rosc::stop()` before the arm.
- The ring oscillator's frequency drop before a dormant that 2.17.7
  recommends: the site runs on it for microseconds at its reset rate.
- The regulator (2.10, VREG_AND_CHIP_RESET) and the brown-out
  detector around a sleep: the reset chapter's registers, no sleep
  here changes them.

Implemented but not bench-verified, each with what would measure it:

- The wake enables pruned awake (`Clocks::wake_enables`): a UART
  whose gate is shut reading its registers as static.
- The four dormant-wake events on any pin but GP1, and
  `DormantWake::pending`: a wire from the second board's output,
  each event in turn.
- The timed site's dormant over a day boundary of the calendar and
  its refusal a day or more out: pinned by the arithmetic, a wake at
  23:59:59 plus two seconds would measure the carry.
- The Pico's own numbers: the same suite on a Pico.
