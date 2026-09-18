# Watchdog (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 12.9 (the
watchdog: the countdown, what it resets, the scratch registers), 8.5 (the
tick generator that feeds it), 7.4 (the power-on state machine and its
WDSEL), 5.2.4 (the bootrom's use of the upper scratch registers);
Appendix E, RP2350-E19. The driver: `brio/rp2350/watchdog.hpp` over
`clock.hpp` and `resets.hpp`. The reference suite: `test_rp2350_timer`,
letters a, j, m and i.

## What the silicon does

TWO THINGS UNDER ONE NAME, where the RP2040's was three.

THE COUNTDOWN is a 24-bit counter loaded through LOAD and decremented on
its tick, resetting whatever the WDSEL registers select when it reaches
zero, or at once on CTRL.TRIGGER. CTRL.TIME is the live value; the three
pause bits hold the count while a debugger has a core halted or is
driving the bus fabric; ENABLE clears, so unlike the other families'
watchdogs this one is not one-way. **The RP2040's erratum E1 is not this
chip's**: there the counter decremented twice per tick, so every LOAD was
doubled and the reach halved; here LOAD is microseconds as written and
the register's 0xffffff is about 16.8 s, which is what 12.9's LOAD
description states. The suite measures the decrement rate against the
system timer rather than trusting either sentence, and MEASURES ONE
DECREMENT PER MICROSECOND (below).

THE TICK IS NOT THIS BLOCK'S ANY MORE (12.9.2). Every consumer of a
timebase has its own generator in the TICKS block of 8.5, and this one's
is one of the six. So the watchdog and the system timers are independent:
a program can run either without the other, and stopping this generator
stands the countdown still.

EIGHT SCRATCH REGISTERS survive a system or a subsystem reset. The
bootrom reads SCRATCH4..7 at every boot for a magic word, an entry point
and a stack pointer that redirect the boot into user code, and takes
SCRATCH2/3 as that redirection's two parameters when it fires (5.2.4) -
so brio hands out SCRATCH0..3 and never writes the upper four, which is
what leaves 2 and 3 "free for arbitrary user values" in 5.2.4's own
words. THE DATASHEET IS NOT OF ONE MIND about what a watchdog reset does
to them: 7.2's note says they are kept by a system or a subsystem reset
and lost to a chip-level one, while 7.3.1's table of chip-level causes
lists the watchdog's own power-on-state-machine reset among its rows and
says every row resets the watchdog peripheral "including watchdog scratch
registers SCRATCH0 -> SCRATCH7". The MECHANISM argues for 7.2: the
bootrom's own watchdog boot vector exists to divert a reboot into user
code and reads SCRATCH4..7 to do it, which only works if a watchdog
reboot leaves them standing. The suite settles it rather than reasons
about it: it carries its token twice, in the scratch registers and in a
`.noinit` record, and reports which crossed.

WHAT A TIME-OUT RESETS is three registers in three tiers (7.1), and this
driver writes one of them:

| Register | Tier | Who writes it |
|----------|------|---------------|
| `PSM_WDSEL` | system: the stages of the power-on state machine | this driver - every stage but the two oscillators |
| `RESETS_WDSEL` | subsystem: the peripheral blocks | `Resets::watchdog_resets`, on request |
| `POWMAN_WDSEL` | chip: the power manager, the switched core domain | nobody - password-protected, the power chapter's |

The third is new on this chip and the difference matters to a program and
not only to a driver: a CHIP-level watchdog reset also clears the scratch
registers, where the system-level one this driver takes is documented not
to - and, MEASURED, does not even reach the power manager's record of
chip-level resets, because POWMAN_WDSEL is what would carry it there
([reset.md](reset.md)).

**Erratum RP2350-E19 is live on stepping A2**: a reboot hangs in the boot
path if any bit but FRCE_OFF.PROC1 is set in the power-on state machine's
hold register when it happens. Nothing in brio sets one, but a previous
life can - a debugger's scripts do - and the cost of being sure is one
store, so every path here that can end in a reboot clears FRCE_OFF except
PROC1 first.

## Types and verbs

- `Watchdog::init(clock)` - this block's tick generator at one
  microsecond from clk_ref; idempotent, and independent of the system
  timers'. A clk_ref that is not a whole number of megahertz does not
  compile. `Watchdog::Tick` is the generator itself.
- `start(timeout_us, pause_on_debug = true)` - the countdown, clamped at
  `max_timeout_us` (about 16.8 s), with PSM_WDSEL set so a time-out
  reboots through the bootrom; `kick()`, `stop()`, `running()`,
  `remaining_us()`, `timeout_us()`.
- `system_resets(stages)` / `system_resets()` - PSM_WDSEL, for a program
  that wants a narrower reboot than start()'s.
- `force_reset()` - CTRL.TRIGGER under the same selection, never returns:
  the chip's one software reboot, which `Reset::software()` is
  ([reset.md](reset.md)).
- `reason()` against `WatchdogReason::timer` / `force`.
- `Scratch<n>` for n in 0..3: `read()`, `write()`, `reg()`. The upper
  four are refused at compile time.
- Beside them in `brio/rp2350/resets.hpp`: `PsmStage` (every stage of the
  sequence as a mask, with `all`, `oscillators` and `reboot`) and `Psm`
  (`watchdog_resets`, `done`, `hold`, `release`, `held`). FRCE_ON is
  deliberately absent - 7.4.2 calls it a development feature that does
  nothing on a production device.

## How to use it

```cpp
brio::Watchdog::init(clock);             // the tick, once
brio::Watchdog::start(500'000u);         // half a second
for (;;) { /* ... */ brio::Watchdog::kick(); }

brio::Scratch<0>::write(state);          // meant to cross the reboot below
brio::Watchdog::force_reset();           // never returns
```

## Bench findings

On an RP2350 in the QFN-80 package, **stepping A2**, clk_ref on the
board's 12 MHz crystal, and on BOTH architectures: the letters below are
part of `test_rp2350_timer`'s **93 pass, 0 fail** on the Cortex-M33 pair
and **93 pass, 0 fail** on the Hazard3 pair, with the reboot letter run
by name at **28 pass, 0 fail** on the Arm half and **24 pass, 0 fail** on
the RISC-V one (four legs there against five, for the reason
[reset.md](reset.md) gives).

- **IT DECREMENTS ONCE PER MICROSECOND, NOT TWICE.** Measured against the
  system timer over 200 ms: **200 000 counts in 200 000 us**, which is
  1000 per 1000 us on both architectures. The RP2040's erratum E1 is not
  this chip's, so LOAD is microseconds as written, the clamp stands at
  the counter's own reach, and a time-out asked for beyond it is clamped
  rather than wrapped.
- **The tick is the TICKS block's.** Stop this watchdog's generator and
  the countdown stands still - 0 counts over 5 ms - which is the
  mechanical proof that the block no longer owns a tick of its own. A
  kick reloads exactly the time-out that was asked for (8 000 000 of
  8 000 000 us read back).
- **ENABLE clears.** The countdown starts and stops under software,
  unlike the one-way watchdogs of the other families.
- **What a time-out selects.** After `start()` the power-on state
  machine's selection reads **0x1FFFFF3** - every stage the reboot needs,
  the two oscillators left alone - and it is ORed in, so a program may
  widen it and never narrows it by accident.
- **The scratch registers cross a watchdog reboot**, and so does the
  `.noinit` SRAM. Measured on every leg of the reboot letter - five on
  the Arm half, four on the RISC-V one: 7.2's note is what this silicon
  does and 7.3.1's table is not, for the system-level reset this driver
  takes. The two survivors carry the same token every time.
- **Erratum RP2350-E19's condition never arose** and the guard costs one
  store: the power-on state machine's hold register reads clear before
  every reboot, and every reboot of the letter completed through the
  bootrom on both architectures.

## Not covered yet

Driver gaps, each with its reason:

- `POWMAN_WDSEL`, the chip-level tier: its register is password-protected
  and belongs to the power chapter, which owns POWMAN whole. It is also
  the tier that would wipe the scratch registers a suite resumes from, so
  the first program to want it should want it knowingly.
- The bootrom's boot redirection through SCRATCH4..7, and the one-shot
  boot types SCRATCH2/3 parameterise (5.2.4.1): a program that wants to
  reboot into RAM code or into BOOTSEL, born with it. The bootrom's own
  `reboot()` API is the same road, and belongs to the bootrom chapter.
- `RESETS_WDSEL` as a selection: `Resets::watchdog_resets()` writes it, no
  program has needed one - a reboot through the power-on state machine
  takes the whole reset controller down anyway.
- The portable watchdog keeper (`util`'s open item): this family's
  spelling is `kick()` and its contract is "LOAD reloads, ENABLE clears";
  the keeper sits above it.

Implemented but not bench-verified, each with what would measure it:

- `pause_on_debug`: the rate above is measured with the three pause bits
  OFF on purpose, so that it is the silicon's and not the probe's. What
  the bits do while a debugger is halted on a core or driving the bus
  fabric would take a suite that runs under a probe and reports to it,
  which no letter of this target does yet.
- The countdown against a clk_ref that is not 12 MHz: the tick generator
  is set from the clock task's own rate and only the board's crystal has
  been on the wire.
- A time-out taken while the countdown is at its clamp - 16.8 s of
  waiting for one verdict, which no suite spends.
