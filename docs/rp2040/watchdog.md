# Watchdog (RP2040)

This document is for a programmer who wants the chip to reboot itself
when the program stops making progress, or who needs a few words that
survive a reboot. `Watchdog` is the countdown: started with a time-out,
reloaded by `kick()`, and when it runs out it reboots the whole chip
through the bootrom. `Scratch<0..3>` are four reset-surviving words.
`WatchdogTick` is the 1 us tick the countdown and the system timer
both count ([timer.md](timer.md)). Each is a monostate (a type whose
functions are all static, over the one block it stands for).

The smallest use, from "How to use it" below:

```cpp
brio::Watchdog::start(500'000u);      // half a second, the tick running
for (;;) { ...; brio::Watchdog::kick(); }

brio::Scratch<0>::write(state);       // survives a watchdog or a core reset
```

Line by line: `start()` loads the countdown with half a second (500 000
microseconds) and enables it, selecting the whole chip but its two
oscillators as what a time-out resets; `kick()` reloads the countdown
with the same value, and a loop that kicks more often than the time-out
keeps the chip running; `Scratch<0>::write()` stores a word the next
boot can read with `Scratch<0>::read()`.

What decides correctness even there. The countdown runs on the tick,
and nothing starts the tick at boot but software: `Timer::init()`
starts it ([timer.md](timer.md)), or `WatchdogTick::start(12)` for the
12 MHz crystal, before `start()`. A time-out is at most 8.3 s
(`max_timeout_us`; erratum E1 decrements the counter twice a tick), and
a longer one is clamped to it. `start()` holds the count while a core
is halted by a debugger unless told otherwise (its second argument).
This watchdog stops when `stop()` is called, unlike the one-way
watchdogs of other chips. Only the four lower scratch words are
offered: the bootrom reads the upper four at every boot, and
`Scratch<4>` does not compile.

## Documents of record

The RP2040 datasheet (build 3184e62), 4.7 (the
watchdog: the tick generator, the countdown, the scratch registers),
2.13 (the power-on state machine and its WDSEL), 2.8.1.1 (the
bootrom's use of the upper scratch registers); Appendix B, RP2040-E1.
The driver: `brio/rp2040/watchdog.hpp`. The reference suite:
`test_rp2040_platform` (letters a, f and i; a suite's letter is one
command key of its console and the group of verdicts it runs).

## What the silicon does

Three things under one name. The tick generator divides clk_ref by
TICK.CYCLES into a nominal 1 us tick that the countdown uses and that
the system timer counts too; nothing starts it at boot but software.
The countdown is a 24-bit counter loaded through LOAD and decremented
on the tick - twice per tick, erratum RP2040-E1, so the reach is 8.3 s
and not 16.7 - that resets whatever PSM_WDSEL and RESETS_WDSEL select
when it reaches zero, or at once on CTRL.TRIGGER; the pause bits hold
it while a core is halted by a debugger; REASON says whether the last
watchdog reset was the countdown or the trigger. Eight scratch
registers survive every reset but the RUN pin and the supply; the
bootrom reads SCRATCH4..7 at every boot for a magic word that redirects
the boot into user code (2.8.1.1). Unlike the other families' watchdogs
this one is not one-way: ENABLE clears, and a stopped countdown stays
stopped.

## Types and verbs

- `WatchdogTick`: `start(cycles)` (1..511; 12 for the 12 MHz crystal
  on clk_ref), `stop`, `running`, `cycles`, `count`.
- `Watchdog`: `start(timeout_us, pause_on_debug = true)` - LOAD doubled
  for E1, capped at `max_timeout_us` (8.3 s), PSM_WDSEL set to
  everything but the two oscillators so a time-out reboots through the
  bootrom; `kick()`, `stop()`, `running()`, `remaining_us()`,
  `force_reset()` (CTRL.TRIGGER, never returns), `reason()`
  (`WatchdogReason::timer` / `force`).
- `Scratch<n>` for n in 0..3: `read()`, `write()`, `reg()`; the upper
  four are refused at compile time.

## How to use it

```cpp
brio::Watchdog::start(500'000u);      // half a second, the tick running
for (;;) { ...; brio::Watchdog::kick(); }

brio::Scratch<0>::write(state);       // survives a watchdog or a core reset
```

## Bench findings

- A forced reset (CTRL.TRIGGER) and a 100 ms time-out both reboot
  the board through the bootrom, each naming itself in REASON and
  replacing the other's name; the chip-level flags stand as they were;
  the scratch words survive both, and so does the .noinit SRAM
  (`test_rp2040_platform` letter i).
- REASON is a HISTORY: it holds the last watchdog event until a
  chip-level reset, through any number of core resets and reflashes
  over the debug port; a board reads TIMER at every boot after its
  first time-out, FORCE after every `Reset::software()` - the chip's
  reboot IS the trigger ([reset.md](reset.md)).
- The atomic aliases on a scratch word do what 2.1.2 says: set, clear,
  xor and a masked write, each read back exact (letter f).

## Not covered yet

Driver gaps, each with its reason:

- The time-out's real length against a wall clock: every counter on
  the chip resets with it; the host end of the console would time it
  (a `brio run` with timestamps), a measurement to add when the
  chapter's suite exists.
- RESETS_WDSEL (which peripheral blocks a watchdog event resets, as
  opposed to the PSM's selection): `Resets::watchdog_resets()` writes
  it, no program has needed a selection yet.
- The bootrom's boot redirection through SCRATCH4..7: a program that
  wants to reboot into RAM code, born with it.
- The portable watchdog keeper (`util`'s open item): this family's
  spelling is `kick()` and its contract is "LOAD reloads, ENABLE
  clears"; the keeper sits above it.
