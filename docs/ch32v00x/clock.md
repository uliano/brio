# RCC (CH32V00x)

The clock tree of RM ch. 3 as this stratum drives it: the static
`Clock` every driver derives its rate from, the `DynamicClock` that
walks the HPRE ladder at run time, and the `Rcc` resource under both -
the roots, the switch, the output, the monitor, the peripheral gates.
Documents of record: the CH32V00X reference manual V1.5 (3.3 for the
tree, 3.4 for the registers), the CH32V006 datasheet V2.0 (the HSI's
and LSI's accuracy, the MCO's pad).

## What the silicon does

- **Three roots for SYSCLK**: the 24 MHz internal RC (HSI, on out of
  reset, factory-calibrated through HSICAL and nudged by a five-bit
  HSITRIM), the HSE (a crystal of 4 to 25 MHz on PA1/PA2, or a clock
  into PA1 with HSEBYP) and the PLL, which has NO ratio to choose - it
  doubles its source, the HSI or the HSE undivided (PLLSRC), so 48 MHz
  comes from the HSI or from a 24 MHz crystal and a PLL from a crystal
  above 24 MHz cannot be. The crystal pads are handed to the
  oscillator by one AFIO bit whose sense differs per part
  ([pin.md](pin.md)); HSERDY says the oscillator runs, and a root
  switch waits for it within a bounded turn count.
- **One divider, HPRE**, between SYSCLK and HCLK: sixteen codes, 0..7
  dividing by 1..8 and 8..15 by 2, 4, 8, 16, 32, 64, 128, 256 - three
  dividers with two spellings. Its reset value is /3, so the chip wakes
  at 8 MHz. There is no APB prescaler: the peripheral buses run at
  HCLK.
- **The 128 kHz LSI**, the watchdog's and the auto-wakeup's root,
  lives with the reset flags in RCC_RSTSCKR (LSION/LSIRDY).
- **Two supervisors, one of them the CH32V006's alone**: the CSS
  (CSSON) watches a ready HSE and, on its failure, switches SYSCLK to
  the HSI, turns the HSE and the PLL off, raises CSSF and the NMI and
  brakes TIM1 - on both parts; the SCM (SYSCM_EN) watches the SYSTEM
  clock, raising SYSCLK_FAILIF, braking TIM1 and, if enabled,
  interrupting on the RCC line - and the CH32V003 has no such monitor
  (its CTLR bits 23:20 are reserved: `Rcc::has_monitor`).
- **MCO on PC4** outputs SYSCLK, the HSI, the HSE or the PLL, on both.
- **The flash wants wait states by rate**, the part's table: 0 to 15
  MHz, 1 to 24, 2 to 48 on the CH32V006 (RM 18.3.1); 0 to 24 and 1 to
  48 on the CH32V003.

## Types and verbs

[brio/ch32v00x/clock.hpp](../../brio/ch32v00x/clock.hpp): `Clock<src,
hz, xtal_hz>` is the static main clock - `internal` (HSI through
HPRE), `crystal` or `external` (the HSE at the rate named third,
through HPRE), `pll` (the HSI doubled, or the crystal named third
doubled: `Clock<ClockSource::pll, 48'000'000, 24'000'000>`), the rate
and the crystal checked at compile time against the divider table and
the HSE's 4..25 MHz, `init()` setting the wait states first, handing
the crystal pads to the oscillator and returning false for a root that
never comes ready. `DynamicClock<Boot,
Users...>` is the runtime regime in the AVR's shape: Boot names the
root at its undivided rate, `set<hz>()` / `set(hz)` name the new rate,
rebase every user in list order, then move HPRE - wait states raised
before a rise and lowered after a fall - and the discrete-rate surface
(`rate_count` 16, `rate_hz(i)`, `rate_index()`) is what
[brio/ch32v00x/delay.hpp](../../brio/ch32v00x/delay.hpp)'s per-rate
table indexes by. `Rcc` is the resource: the HSI and its trim, the
HSE (`hse()`, `hse_ready()`, `hse_bypass()`) and its CSS (`css()`,
`css_failed()`, `clear_css_failed()`), the LSI, the PLL's state and
source (`pll_from_hse()`), the switch and the divider as they stand,
the MCO, the monitor and its failure flag (writing nothing and
reading false on a part without one), and `clock(bus, mask, on)` /
`reset(bus, mask)` for the peripheral gates on the HB, PB2 and PB1
buses - the verbs every configuring driver opens its own gate with.

## How to use it

```cpp
using Boot = brio::Clock<brio::ClockSource::pll, 48'000'000>;
using SysClock = brio::DynamicClock<Boot, brio::Ticker, Serial>;
constexpr SysClock clock;

SysClock::init();                 // 48 MHz, two wait states
Serial::init(clock, 115200);      // asserts it is among the users
brio::Ticker::init(clock);
SysClock::set<6'000'000>();       // the users rebased, then HPRE /8, then 0 waits
```

A program with one rate names `Clock<...>` alone and every driver
folds the rate at compile time.

## Bench findings

The reference suite is `test_ch32_clock` (32 verdicts in `z` on the
CH32V006K8U6, 30 on the CH32V003F4P6), rooted on the 24 MHz crystal
both bench boards carry through the PLL - so every letter is a
measurement of the HSE path, and the console reading clean at 115200
its first proof.

- **The ladder holds at every rung.** 48 -> 24 -> 16 -> 12 -> 6 -> 3 MHz
  and back: at each rate the HPRE code and the wait states are what
  the tables say, twenty 500-us waits are exactly ten ticks (the
  ticker and the delay table rebased), and the console - rebased to
  the new HCLK - prints its line clean, which is the proof that HCLK
  landed within the UART's tolerance of the claim (the actual baud
  reads 115384 or 115107 at every rung). A rate no divider reaches is
  refused with nothing changed.
- **The LSI is ready 17 us after LSION** (862 HCLK cycles, 56 polls),
  and drops its ready bit once stopped.
- **The HSI trim takes a step each way**, and on a crystal root the
  console's rate does not move with it. What a step is worth in hertz
  needs a counter on the MCO.
- **The monitor turns on and sees no failure** on a running clock
  (the CH32V006); on the CH32V003 the verb writes nothing and reads
  false. The MCO takes its selections; a PB1 gate opens, resets and
  closes, on both.
- **The HSE**: on both boards the crystal is on and ready at boot, the
  PLL's source reads the HSE, the crystal pads read as the
  oscillator's in each part's own sense (the CH32V006's bit clear, the
  CH32V003's set), and the CSS arms on the ready HSE and reports no
  failure, then disarms. A failure itself is not provoked (an NMI this
  suite does not bind).
- **The tree at boot** after `Clock<pll, 48 MHz, 24 MHz>::init()`: SWS
  = PLL, HPRE = 1, HSE and HSI on, the trim at 16, PLL locked from the
  HSE, LSI off, the part's wait states.

## Not covered yet

Driver gaps, each with its reason:

- The LSI as SYSCLK: no user, and a program that wanted it would want
  the whole power chapter with it.
- The ADC prescaler and clock mode bits of CFGR0: the ADC chapter's.
- The RCC interrupt line (the ready flags' interrupts, the failure
  interrupt) as an ISR body: nothing waits on a root asynchronously
  yet; `Rcc::failure_interrupt()` exists for the day the monitor gets
  a handler.
- A root switch at run time (PLL on or off under a running program):
  by design not a DynamicClock rate - the divider is what a running
  program changes - and the power phase is where the question of
  running from the HSI alone gets asked.

Implemented but not bench-verified, each with what would measure it:

- The rates in hertz: the ladder is proven by the console reading
  clean at each rung (within UART tolerance) and by tick counts that
  the rate itself clocks; a counter on the MCO (PC4) is what turns
  that into parts per million, and what the trim steps are worth.
- The system clock monitor's failure path and the CSS's: a failure
  needs a crystal that stops, which a soldered one does not; the CSS's
  NMI also needs a handler bound by the program that arms it.
- The HSE as SYSCLK itself (`ClockSource::crystal` and `::external`
  without the PLL) and the bypass: compiled and refused where the
  chapter refuses (the rate outside 4..25 MHz, a PLL past 48 MHz, a
  crystal not named); measured only as the PLL's source, since the
  suites run at 48 MHz.
