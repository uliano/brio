# The two watchdogs (CH32V203 and CH32V303)

This document is for a programmer who wants the chip reset when the
program stops making progress, on a CH32V203 or a CH32V303, and wants
to know at the next boot that this is what happened. Both parts carry
the same two blocks: an independent watchdog on its own oscillator,
which nothing but a reset stops, and a window watchdog on the
peripheral bus, which resets the chip both when it is refreshed too
late and when it is refreshed too early. The drivers are two monostates
(classes with only static members, never instantiated: one block, one
set of state), `Iwdg` and `Wwdg`, with the arithmetic that turns a
setting into a time-out as free functions; which reset ended the last
run is read through [reset.hpp](../../brio/ch32vx03/reset.hpp)'s flags.

The smallest use, the first block of "How to use it" below:

```cpp
(void)brio::Iwdg::arm({.prescaler = brio::IwdgPrescaler::div32,
                       .reload = 249});
for (;;) {
    kernel.step();
    brio::Iwdg::refresh();
}
```

Line by line: `arm()` starts the independent watchdog and then writes
the setting, a prescaler of /32 and a reload of 249 (about a fifth of a
second at the oscillator's typical 39 kHz, a quarter at the CH32V203RB's
32 kHz: `lsi_typ_hz` in each part's table,
[brio/ch32vx03/parts/](../../brio/ch32vx03/parts/)), and answers false
only for a setting the fields cannot hold, which the line discards; from
then on only a reset stops it. The loop is the page's sketch of a
program's turn: `kernel.step()` stands for the work done between two
refreshes, and `refresh()`, one store of a key, reloads the counter.

What decides correctness even there: the order, start before setting,
which `arm()` keeps and `configure()` alone does not, because the
watchdog's registers take a write only while its oscillator runs and
the start is what forces that oscillator on. Measured: a setting
written with the oscillator stopped does not land while it stays
stopped (the reload reads back unchanged and both update flags stand
through their bounded wait), and once the oscillator is started the
flags clear, which fits the pending write completing then; which
setting the watchdog would run on after that order is not measured
("The independent watchdog" below). Second, the refresh interval,
computed from the oscillator's fast corner (`device::lsi_max_hz`), since
the oscillator is an uncalibrated RC rated from 25 to 60 kHz (25 to
45 kHz on the CH32V203RB; each part table's `lsi_min_hz` and
`lsi_max_hz`) and the fast corner is the one that resets a program early
(the second block below); and the commitment itself, since nothing in
software stops an armed independent watchdog, so a program that stops
refreshing it is reset - a core halted by a debugger included, unless
the debug module's freeze bits hold the counter, which no verb here sets
("Not covered yet"). The window watchdog adds its own two: a refresh
before the counter has fallen into the window is itself the reset, and
its early-wake-up interrupt leaves one counter tick to act in.

The rest of this page, in order: the documents of record; what the
silicon does, for each watchdog; the types and verbs; how to use it;
the bench findings, on both parts; and what is not covered.

## Documents of record

The CH32F/V20x_V30x_V31x reference manual
V2.3 (chapter 7 for the independent watchdog, chapter 8 for the window
one, 3.3.5.4 for the oscillator the first forces on, 3.4.10 for the
reset flags both raise), the CH32V203 datasheet V2.8 (table 2-1, whose
"2 (WWDG + IWDG)" row spans every part of the series, and table 4-14
for the LSI's rated spread) and the CH32V303/305/307/317 datasheet V3.5
(table 2-1-1 and table 4-15, 25 to 60 kHz with 39 typical, as table
4-14 gives every CH32V203 but the RB, whose LSI is the 32 kHz one,
25 to 45 kHz in its part table).
Both chapters open with "this chapter applies to the whole family" and
carry no device-class note, so the two blocks are the same on the
CH32V20x_D6, the CH32V20x_D8 and the CH32V30x_D8, and neither is a
device-class question. Driver:
[brio/ch32vx03/watchdog.hpp](../../brio/ch32vx03/watchdog.hpp); the
reset flags are [reset.hpp](../../brio/ch32vx03/reset.hpp)'s. Reference
suite: `test_vx03_watchdog` (a suite's letter is one command key of its
console and the group of verdicts it runs; `z` is the key that runs
every letter but the three that reboot the board, `w`, `v` and `p`,
each run by name).

## What the silicon does

### The independent watchdog

- **A twelve-bit down-counter on the LSI**, with an eight-bit prescaler
  of seven codes (/4 to /256) and a period of `reload + 1` prescaled
  ticks (7.2.1). It has no clock gate and no enable bit: three key
  values written into IWDG_CTLR are the whole interface - 0x5555 opens
  the prescaler and reload registers, 0xCCCC starts the watchdog, 0xAAAA
  reloads the counter.
- **Starting it is one way.** Nothing in software stops it again; a
  reset does, and the suite's own letter is what proves the reset
  arrives.
- **The LSI is forced on when it starts** (3.3.5.4) whatever the program
  did with LSION - which is also the only way to tell the watchdog is
  running, there being no status bit that says so. `running()` is that
  reading: the oscillator ready while nothing in the program asked for
  it.
- **The registers live in the oscillator's domain, and the chapter never
  says so.** Measured on both parts: with the LSI stopped, a prescaler
  or reload written behind the unlock key never arrives - the registers
  keep their reset values and STATR's PVU and RVU stand set for ever,
  since the update they report is the crossing into that domain. With
  the LSI running the same write lands and both flags clear within
  microseconds. So the order a program writes them in is not free:
  `arm()` starts the watchdog first, which forces the oscillator on, and
  writes the setting afterwards - the reset setting (0x0FFF at /4, some
  four hundred milliseconds at the nominal rate) being the budget it has
  to get there. A program that wants the registers ready before it
  commits starts the LSI itself.
- **The unlock stands until another key closes it** (7.3.1's "a write
  access to this register with a different value breaks the sequence").
  Measured on both parts: after a keyed configuration a naked store into
  RLDR still lands, and after a refresh the same store changes nothing.
  That is why `arm()` ends with a refresh.
- **Its time-out is the LSI's, and the LSI is an uncalibrated RC.** Both
  datasheets rate it 25 to 60 kHz with 39 kHz typical (the CH32V203RB's
  to 45), a spread of 1.8 to one at the least - so every arithmetic
  helper here takes the rate as an argument (`device::lsi_min_hz`,
  `lsi_typ_hz`, `lsi_max_hz`) and a program that must not be reset
  early computes its refresh interval from the fast corner. There is no
  window register in this chapter: the independent watchdog has one
  edge and not two.
- **It is no way out of a Stop, and it is one out of a Standby.** On
  both parts it does not count through a Stop, against 7.2.1's "can
  still work normally in stop and standby mode" - its reset arrives
  some 200 ms after the wake - so a program that sleeps that deep arms
  the RTC's alarm as its way back, which is what the timed sleep site
  does. Through a Standby it counts, and its reset is the exit RM 2.3.4
  lists: measured on both parts ([sleep.md](sleep.md)).

### The window watchdog

- **A seven-bit down-counter on PCLK1 over 4096**, divided again by
  WDGTB (1, 2, 4 or 8), with the reset at the step from 0x40 to 0x3F -
  so a time-out is `T[5:0] + 1` ticks from a refresh (8.2.1). Its whole
  range is milliseconds where the independent watchdog's is seconds:
  3.6 ms to 29 ms at 72 MHz of PCLK1.
- **The counter does not free-run**, against 8.2.1's own sentence ("no
  matter whether the watchdog function is enabled or not, the counter
  keeps counting down"). Measured on both parts: with WDGA clear the
  counter holds what the last write put in it for twenty milliseconds -
  357 ticks of its own clock - and starts falling the moment WDGA is
  set. The sister family's block behaves the same way against the same
  sentence ([the CH32V00x's own document](../ch32v00x/platform.md)), so
  this is WCH's design and not one part's accident.
- **Its clock gate is half a silence.** With RCC's WWDGEN clear the
  registers still read their reset values (0x007F both) and a write is
  dropped - measured both ways, on both parts. What the gate holds is
  the block's clock, which is what 8.2.1 offers it for: a way to
  suspend a watchdog whose enable bit is one-way. The block's reset
  line is the other way, and the one this driver's `reset()` uses.
- **WDGA is one-way in software** (8.3.1), like the other watchdog's
  start key - but unlike it, a peripheral reset really does clear it,
  which is what lets a suite arm this one and carry on.
- **A refresh outside the window is the reset**: a write into T[6:0]
  while the counter is still above the window value resets the chip on
  the spot. Measured, both deliberately and by accident - narrowing the
  window under a counter that has just been refreshed and refreshing
  again is the accident.
- **The early-wake-up flag is raised at 0x40 whether or not its
  interrupt is enabled** (8.3.3), and what follows it is one tick. With
  the interrupt enabled the vector is this block's own (entry 16 of the
  table on every class), and a handler that does not refresh does not
  prevent the reset - measured, the handler running exactly once.
- **EWI is one-way too**: 8.3.2 says the enable is cleared only by a
  reset, so a configuration that sets it cannot take it back.

## Types and verbs

### The independent watchdog

`Iwdg` is a monostate. `IwdgPrescaler` names the seven dividers,
`IwdgConfig` is the pair (prescaler, reload) and `iwdg_config_valid()`
judges it.

- **Committing**: `start()` (the point of no return), `arm(config)`
  (start, configure, refresh - the order the silicon needs),
  `refresh()` (the verb a loop calls, and what re-locks the two
  registers), `force_reset()` (the shortest setting there is, started
  and refreshed once so the counter takes it).
- **Configuring**: `configure(config)` and its compile-time twin
  `configure<config>()`, which refuses a prescaler code or a reload the
  fields cannot hold; `unlock()` for a caller that writes the registers
  itself.
- **Reading**: `prescaler()` and `reload()` (each waiting for its update
  flag, because a read while one stands is invalid), `status()`,
  `busy(mask)`, `wait_idle(mask, turns)`, `running()`, and
  `timeout_us(lsi_hz)` - what is in the registers now, at a rate the
  caller states.
- **The arithmetic**, as free functions so a program can compute a
  setting at compile time: `iwdg_prescaler_divider()`,
  `iwdg_timeout_us(p, reload, lsi_hz)`, `iwdg_timeout_ms(...)` and
  `iwdg_reload_for(p, us, lsi_hz)` - the smallest reload whose time-out
  reaches what was asked, 0xFFFF when the twelve-bit field cannot.

### The window watchdog

`Wwdg` is a monostate too. `WwdgPrescaler` names the four WDGTB codes
and `WwdgConfig` is (prescaler, window, early wake-up), judged by
`wwdg_config_valid()` - which refuses a window at or below 0x3F, since
no counter value is ever inside one.

- **The block**: `bus_clock(bool)` and its reader, `reset()` (the
  peripheral reset line - the way out of an armed watchdog), `init()`
  (both), `irq()`, `regs()`.
- **Committing**: `start(counter)` (WDGA and the counter in one store),
  `refresh(counter)` (the counter alone, WDGA kept), `force_reset()`.
- **Configuring**: `configure(config)` and `configure<config>()`.
- **Reading**: `enabled()`, `counter()`, `prescaler()`, `window()`,
  `early_wakeup_enabled()`, `in_window()` (whether a refresh would be
  legal right now - a snapshot of a moving counter, so a hint and not a
  lock), `ctlr()`, `cfgr()`, `timeout_us(pclk1_hz, counter)`.
- **The flag and the vector**: `flag()`, `clear_flag()` and `isr()` -
  the body an application binds, true when the early wake-up was this
  peripheral's doing.
- **The arithmetic**: `wwdg_cycles_per_tick(p)`,
  `wwdg_tick_hz(pclk1_hz, p)`, `wwdg_timeout_us(pclk1_hz, p, counter)`
  and `wwdg_window_wait_us(pclk1_hz, p, counter, window)` - the wait
  before a refresh becomes legal.

## How to use it

The independent watchdog, armed for about a fifth of a second and kept
alive:

```cpp
(void)brio::Iwdg::arm({.prescaler = brio::IwdgPrescaler::div32,
                       .reload = 249});
for (;;) {
    kernel.step();
    brio::Iwdg::refresh();
}
```

A refresh interval computed for the fast corner, which is the one that
resets a program early:

```cpp
constexpr uint32_t budget_us =
    brio::iwdg_timeout_us(brio::IwdgPrescaler::div32, 249, brio::device::lsi_max_hz);
static_assert(budget_us > 100'000u, "refresh at least ten times a second");
```

The window watchdog, whose refresh has to wait for its window:

```cpp
brio::Wwdg::init();
(void)brio::Wwdg::configure({.prescaler = brio::WwdgPrescaler::div8,
                             .window = 0x60});
brio::Wwdg::start(0x7F);
for (;;) {
    work();
    if (brio::Wwdg::in_window()) { brio::Wwdg::refresh(0x7F); }
}
```

Its early-wake-up interrupt, which has one counter tick to act in:

```cpp
BRIO_CH32_VECTOR(wwdg_handler) {
    if (brio::Wwdg::isr()) { brio::Wwdg::refresh(0x7F); }
}
```

And which reset the last boot, through the flags reset.hpp keeps:

```cpp
const uint32_t flags = brio::Reset::take_flags();
if ((flags & brio::ResetFlag::independent_watchdog) != 0u) { ... }
```

## Bench findings

`test_vx03_watchdog` at 144 MHz (PCLK1 72 MHz), nothing wired, on the
CH32V203C8T6 and on the CH32V303VCT6 - the same verdicts on both:
**16 pass, 0 fail** in `z`, plus **4 pass** in letter `w`, **8 pass**
in letter `v` and **13 pass** in letter `p`, the three by-name letters
that reboot the board thirteen times between them (`p` ten times on each
part). Every number below is both parts' where one is given.

**The window watchdog's counter is frozen until it is armed.** With
WDGA clear it read 127 and still read 127 twenty milliseconds later -
357 ticks of its own clock. Armed, it left 127 and was at 119 half a
millisecond later.

**Its tick is PCLK1 / 4096 / 2^WDGTB, to the microsecond.** The fall
from 0x7F to 0x50 (47 ticks) measured **2628 us at /1 (2673 nominal),
5346 and 5347 us at /2 (5347), 10694 us at /4 (10695) and 21389 us at
/8 (21390)** - the refresh lands at any phase of the prescaler, so the
fall is 46 to 47 whole ticks and the letter's three per cent covers
both - and the
early-wakeup flag came up **28671 us after a refresh where 63 ticks at
/8 are 28672**.

**Its clock gate holds the block's clock and not the read path.** With
WWDGEN clear, CTLR and CFGR read 0x007F and a write of 0x006A into CFGR
was dropped; with the gate open the same registers take it, and the
block's reset line puts both back to 0x007F.

**Its three resets, each with the flag at the next boot.** Left
unrefreshed at /8 it reset the board **29 ms** after the last refresh,
where the arithmetic says 29; a refresh BEFORE the window opened reset
it in **0 ms** - immediately, not after the counter's own fall; and with
the early-wake-up interrupt enabled the handler ran **once** and the
reset arrived at the same **29 ms**, the interrupt costing nothing.
WWDGRSTF stood alone at all three boots.

**The independent watchdog's registers need their oscillator.** With the
LSI stopped, a keyed write of 0x123 left RLDR at 4095 and STATR at 0x3 -
both update flags standing, and still standing after twenty thousand
polls. With the LSI started the same write read back 0x123, the
prescaler read /16 and STATR read zero.

**Its unlock stands until another key.** A naked store of 0x456 into
RLDR landed while the window was open; after a refresh, a naked store of
0x789 changed nothing.

**Its time-out, and what it says about each die's LSI.** Armed at /32
with a reload of 249, refreshed ten times, then left: the CH32V203C8T6
came back **206 and 207 ms** later in two runs, which puts its LSI at
**38.8 kHz**, and the CH32V303VCT6 **199 ms** later in two runs of one
session and **217 ms** in another, which puts its LSI at **40.2 and
36.9 kHz** - the RC moving eight per cent between sessions, all of it
inside the parts' rated 133 to 320 ms and around the datasheets' 39 kHz
typical. IWDGRSTF stood alone at every boot, and each board had survived
its ten refreshes before that.

**Every prescaler divides the same oscillator by 4 << code.** Letter `p`
arms each of the seven codes for the smallest reload that reaches 400 ms
at the typical LSI, lets the update cross into the LSI's domain, and
leaves it: every leg ended in IWDGRSTF alone, and the LSI each span
implies agrees with the /32 leg's within that leg's window, derived in
the letter from the tick's millisecond, one prescaled tick and half a
per cent of the RC's wander between legs (1.2 % at /4, 2.9 % at /256).
On the CH32V203C8T6 the seven read **38.5 to 38.7 kHz** at /4 to /128
(403 to 405 ms) and **39.0 to 39.1 kHz** at /256 (399 to 400 ms); on
the CH32V303VCT6 **36.79 to 36.83 kHz** at /4 to /64 (424 ms) and
**37.00 kHz** at /128 and /256 (422 ms) - the same figures to the
millisecond in all ten runs on each part. The longest prescaler reads
fast on both, by 0.5 to 1.5 %: its span is short by up to most of one
6.6 ms tick, which is what a refresh that reloads the counter without
restarting the prescaler would do - chapter 7 does not say which, and
the letter's window holds either way.

**Both force_reset()s reset at once, through their own flag.**
`Iwdg::force_reset()` - the start key, /4 with a reload of zero, a
refresh - was back in **0 ms** on the CH32V203C8T6 and **3 ms** on the
CH32V303VCT6 in every run, IWDGRSTF alone: the refresh right after the
configure DID load the new reload (the reset setting would have been 273
ms even at the fast corner), and the span is the LSI's own start, which
the datasheets (CH32V203 table 4-14, CH32V303 table 4-15) give as 230
us with the LSE running and 5 ms with it off, typical, no maximum. The letter's bound is 12 ms (that start taken twice,
the crossing's five prescaled periods at /4, one /4 tick, the tick's
millisecond).
`Wwdg::force_reset()` - WDGA with the counter at zero - was back in
**0 ms** on both, WWDGRSTF alone: T6 clear is the reset condition
itself.

**Through the two deep modes** (`test_vx03_sleep`, letters `g` and `x`,
the RTC's alarm armed first as the way back in both). The same setting,
refreshed immediately before a Stop asked for two seconds, let the Stop
run its full length on both parts and reset the board **215 ms** (the
CH32V203C8T6) and **208 and 209 ms** (the CH32V303VCT6) after the wake - the
counter frozen for the whole sleep. Armed after an alarm placed two
seconds out and followed by a Standby, it ended the Standby on the
CH32V303VCT6 **225 ms** after the alarm was placed, twice, and on the
CH32V203C8T6 **228 to 230 ms**, six times, with IWDGRSTF and PORRSTF
standing at the boot, SBF set and WUF clear - the watchdog's exit and
not the alarm's - and no watchdog running into the new boot (the
CH32V203C8T6's DBGMCU_CR reading 0x0 at it, its freeze bits clear).
The debug module's two watchdog freeze bits read clear there
(DBGMCU_CR 0x0), so neither answer is the probe's.

## Not covered yet

Driver gaps, each with its reason:

- **The option byte that starts the independent watchdog at every boot**
  (IWDG_SW). The option bytes are the flash chapter's and are decoded
  read-only there, by that chapter's own decision
  ([nvm.md](nvm.md)); what this driver states is that a
  hardware-started watchdog needs no start key, which is the chapter's
  own sentence.
- **The debug module's freeze bits** (chapter 34), which hold either
  counter while a probe has the core halted. That register is a core
  CSR the power chapter READS and never writes, because a `csrw` to it
  resets the part on this silicon ([sleep.md](sleep.md)), so no verb
  here sets them; every measurement above was taken with the program
  running free, and the deep modes' on the CH32V303VCT6 with both bits
  read clear.
- **The window watchdog through a low-power mode.** Its counter runs on
  the peripheral bus clock, which a Stop takes away, and what it does
  across one is untested; the independent one is measured there, and
  is no way back out of a Stop (above).
