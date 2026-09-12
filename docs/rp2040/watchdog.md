# Watchdog (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 4.7 (the
watchdog: the tick generator, the countdown, the scratch registers),
2.13 (the power-on state machine and its WDSEL), 2.8.1.1 (the
bootrom's use of the upper scratch registers); Appendix B, RP2040-E1.
The driver: `brio/rp2040/watchdog.hpp`. The reference suite:
`test_rp2040_platform` (letters a, f and i).

## What the silicon does

Three things under one name. THE TICK GENERATOR divides clk_ref by
TICK.CYCLES into a nominal 1 us tick that the countdown uses and that
THE SYSTEM TIMER COUNTS TOO; nothing starts it at boot but software.
THE COUNTDOWN is a 24-bit counter loaded through LOAD and decremented
on the tick - TWICE per tick, erratum RP2040-E1, so the reach is 8.3 s
and not 16.7 - that resets whatever PSM_WDSEL and RESETS_WDSEL select
when it reaches zero, or at once on CTRL.TRIGGER; the pause bits hold
it while a core is halted by a debugger; REASON says whether the last
watchdog reset was the countdown or the trigger. EIGHT SCRATCH
REGISTERS survive every reset but the RUN pin and the supply; the
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
