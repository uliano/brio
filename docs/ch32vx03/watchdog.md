# The two watchdogs (CH32V203 and CH32V303)

An INDEPENDENT watchdog on its own oscillator, which nothing but a reset
stops, and a WINDOW watchdog on the peripheral bus, which resets the
chip both when it is refreshed too late and when it is refreshed too
early. Documents of record: the CH32F/V20x_V30x_V31x reference manual
V2.3 (chapter 7 for the independent watchdog, chapter 8 for the window
one, 3.3.5.4 for the oscillator the first forces on, 3.4.10 for the
reset flags both raise), the CH32V203 datasheet V2.8 (table 2-1, whose
"2 (WWDG + IWDG)" row spans every part of the series, and table 4-14
for the LSI's rated spread) and the CH32V303/305/307/317 datasheet V3.5
(table 2-1-1 and table 4-15, the same 25 to 60 kHz with 39 typical).
Both chapters open with "this chapter applies to the whole family" and
carry no device-class note, so the two blocks are the same on the
CH32V20x_D6, the CH32V20x_D8 and the CH32V30x_D8, and neither is a
device-class question. Driver:
[brio/ch32vx03/watchdog.hpp](../../brio/ch32vx03/watchdog.hpp); the
reset flags are [reset.hpp](../../brio/ch32vx03/reset.hpp)'s. Reference
suite: `test_vx03_watchdog`.

## What the silicon does

### The independent watchdog

- **A twelve-bit down-counter on the LSI**, with an eight-bit prescaler
  of seven codes (/4 to /256) and a period of `reload + 1` prescaled
  ticks (7.2.1). It has no clock gate and no enable bit: three KEY
  VALUES written into IWDG_CTLR are the whole interface - 0x5555 opens
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
- **THE REGISTERS LIVE IN THE OSCILLATOR'S DOMAIN, and the chapter never
  says so.** Measured on both parts: with the LSI stopped, a prescaler
  or reload written behind the unlock key NEVER ARRIVES - the registers
  keep their reset values and STATR's PVU and RVU stand set for ever,
  since the update they report is the crossing into that domain. With
  the LSI running the same write lands and both flags clear within
  microseconds. So the ORDER a program writes them in is not free:
  `arm()` starts the watchdog FIRST, which forces the oscillator on, and
  writes the setting afterwards - the reset setting (0x0FFF at /4, some
  four hundred milliseconds at the nominal rate) being the budget it has
  to get there. A program that wants the registers ready before it
  commits starts the LSI itself.
- **The unlock STANDS until another key closes it** (7.3.1's "a write
  access to this register with a different value breaks the sequence").
  Measured on both parts: after a keyed configuration a naked store into
  RLDR still lands, and after a refresh the same store changes nothing.
  That is why `arm()` ends with a refresh.
- **Its time-out is the LSI's, and the LSI is an uncalibrated RC.** Both
  datasheets rate it 25 to 60 kHz with 39 kHz typical (the CH32V203RB's
  to 45), a spread of 1.8 to one at the least - so every arithmetic
  helper here takes the RATE as an argument (`device::lsi_min_hz`,
  `lsi_typ_hz`, `lsi_max_hz`) and a program that must not be reset
  early computes its refresh interval from the FAST corner. There is no
  window register in this chapter: the independent watchdog has one
  edge and not two.
- **It is no way out of a Stop.** On the CH32V203C8T6 it does not
  count through one ([sleep.md](sleep.md)), and a program that sleeps
  that deep arms the RTC's alarm as its way back, which is what the
  timed sleep site does. Whether it ends a Standby, where RM 2.3.4
  lists its reset among the exits, is not measured on either part
  (below).

### The window watchdog

- **A seven-bit down-counter on PCLK1 over 4096**, divided again by
  WDGTB (1, 2, 4 or 8), with the reset at the step from 0x40 to 0x3F -
  so a time-out is `T[5:0] + 1` ticks from a refresh (8.2.1). Its whole
  range is milliseconds where the independent watchdog's is seconds:
  3.6 ms to 29 ms at 72 MHz of PCLK1.
- **THE COUNTER DOES NOT FREE-RUN**, against 8.2.1's own sentence ("no
  matter whether the watchdog function is enabled or not, the counter
  keeps counting down"). Measured on both parts: with WDGA clear the
  counter holds what the last write put in it for twenty milliseconds -
  357 ticks of its own clock - and starts falling the moment WDGA is
  set. The sister family's block behaves the same way against the same
  sentence ([the CH32V00x's own document](../ch32v00x/platform.md)), so
  this is WCH's design and not one part's accident.
- **Its clock gate is half a silence.** With RCC's WWDGEN clear the
  registers still READ their reset values (0x007F both) and a write is
  DROPPED - measured both ways, on both parts. What the gate holds is
  the block's clock, which is what 8.2.1 offers it for: a way to
  suspend a watchdog whose enable bit is one-way. The block's RESET
  LINE is the other way, and the one this driver's `reset()` uses.
- **WDGA is one-way in software** (8.3.1), like the other watchdog's
  start key - but unlike it, a peripheral reset really does clear it,
  which is what lets a suite arm this one and carry on.
- **A refresh outside the window IS the reset**: a write into T[6:0]
  while the counter is still above the window value resets the chip on
  the spot. Measured, both deliberately and by accident - narrowing the
  window under a counter that has just been refreshed and refreshing
  again is the accident.
- **The early-wake-up flag is raised at 0x40 whether or not its
  interrupt is enabled** (8.3.3), and what follows it is ONE TICK. With
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
  and never refreshed).
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
  REACHES what was asked, 0xFFFF when the twelve-bit field cannot.

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

A refresh interval computed for the FAST corner, which is the one that
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
extern "C" BRIO_CH32_INTERRUPT void wwdg_handler() {
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
**16 pass, 0 fail** in `z`, plus **4 pass** in letter `w` and **8 pass**
in letter `v`, the two by-name letters that reboot the board four times
between them. Every number below is both parts' where one is given.

**The window watchdog's counter is frozen until it is armed.** With
WDGA clear it read 127 and still read 127 twenty milliseconds later -
357 ticks of its own clock. Armed, it left 127 and was at 119 half a
millisecond later.

**Its tick is PCLK1 / 4096 / 2^WDGTB, to the microsecond.** The fall
from 0x7F to 0x50 (47 ticks) measured **2628 us at /1 (2673 nominal),
5346 and 5347 us at /2 (5347) and 21389 us at /8 (21390)**, and the
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
**38.8 kHz**, and the CH32V303VCT6 **199 ms** later in two runs, which
puts its LSI at **40.2 kHz** - both inside the parts' rated 133 to 320
ms and beside the datasheets' 39 kHz typical. IWDGRSTF stood alone at
every boot, and each board had survived its ten refreshes before that.

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
  running free, so none of them depends on which way they stand.
- **The window watchdog through a low-power mode.** Its counter runs on
  the peripheral bus clock, which a Stop takes away, and what it does
  across one is untested; the independent one is measured there, and
  is no way back out of a Stop (above).

Implemented but not bench-verified:

- **The four prescaler codes of the window watchdog**: three of them are
  timed above; /4 is written and read back and nothing times it,
  because the three that are measured already fix the rule its code
  follows.
- **The independent watchdog at prescalers other than /32 on the
  silicon**: every divider is decoded and asserted at compile time, and
  the reset letter arms one setting. The others would cost one reset
  each to measure, and what they would add is the same arithmetic at
  another rate.
- **The independent watchdog through a Standby**: RM 2.3.4 lists its
  reset among Standby's exits. One program on the CH32V303VCT6 that
  armed it for some 200 ms and entered Standby with the debug probe
  attached never answered its console again - an observation and not a
  measurement, because nothing in that program could tell a watchdog
  that never fired from one that did. What would measure it: a Standby
  entered with the RTC's alarm armed as the way back and the watchdog's
  reload shorter than the alarm, the boot reading which flag stands.
- **`force_reset()` on either block**: both are one store away from what
  the by-name letters already prove, and neither is called by a suite -
  a program that wants a deliberate reset has `Reset::software()`
  beside them.
