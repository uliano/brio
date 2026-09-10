# RCC (CH32V00x)

The clock tree of RM ch. 3 as this stratum drives it: the static
`Clock` every driver derives its rate from, the `DynamicClock` that
walks the HPRE ladder at run time, and the `Rcc` resource under both -
the roots, the switch, the output, the monitor, the peripheral gates.
Documents of record: the CH32V00X reference manual V1.5 (3.3 for the
tree, 3.4 for the registers), the CH32V006 datasheet V2.0 (the HSI's
and LSI's accuracy, the MCO's pad).

## What the silicon does

- **Two roots for SYSCLK**: the 24 MHz internal RC (HSI, on out of
  reset, factory-calibrated through HSICAL and nudged by a five-bit
  HSITRIM) and the PLL, which has NO ratio to choose - it doubles its
  source, so the HSI gives 48 MHz and nothing else. The HSE input
  (PA1/PA2, a crystal or a bypassed clock) is the third root and the
  PLL's other source.
- **One divider, HPRE**, between SYSCLK and HCLK: sixteen codes, 0..7
  dividing by 1..8 and 8..15 by 2, 4, 8, 16, 32, 64, 128, 256 - three
  dividers with two spellings. Its reset value is /3, so the chip wakes
  at 8 MHz. There is no APB prescaler: the peripheral buses run at
  HCLK.
- **The 128 kHz LSI**, the watchdog's and the auto-wakeup's root,
  lives with the reset flags in RCC_RSTSCKR (LSION/LSIRDY).
- **Two supervisors**: the CSS watches an HSE and falls back to the HSI
  on its failure; the SCM (SYSCM_EN) watches the SYSTEM clock, raising
  SYSCLK_FAILIF, braking TIM1 and, if enabled, interrupting on the RCC
  line.
- **MCO on PC4** outputs SYSCLK, the HSI, the HSE or the PLL.
- **The flash wants wait states by rate** (RM 18.3.1): 0 to 15 MHz, 1
  to 24, 2 to 48.

## Types and verbs

[brio/ch32v00x/clock.hpp](../../brio/ch32v00x/clock.hpp): `Clock<src,
hz>` is the static main clock - `internal` (HSI through HPRE) or `pll`
(HSI x2 through HPRE), the rate checked at compile time against the
divider table, `init()` setting the wait states first and returning
false for a root that never comes ready. `DynamicClock<Boot,
Users...>` is the runtime regime in the AVR's shape: Boot names the
root at its undivided rate, `set<hz>()` / `set(hz)` name the new rate,
rebase every user in list order, then move HPRE - wait states raised
before a rise and lowered after a fall - and the discrete-rate surface
(`rate_count` 16, `rate_hz(i)`, `rate_index()`) is what
[brio/ch32v00x/delay.hpp](../../brio/ch32v00x/delay.hpp)'s per-rate
table indexes by. `Rcc` is the resource: the HSI and its trim, the
LSI, the PLL's state, the switch and the divider as they stand, the
MCO, the monitor and its failure flag, and `clock(bus, mask, on)` /
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

The reference suite is `test_ch32_clock` (27 verdicts in `z`) on the
CH32V006K8U6.

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
- **The HSI trim takes a step each way** and the console still reads
  at the nudged rate - one step is inside the UART's tolerance. What a
  step is worth in hertz needs a counter on the MCO.
- **The monitor turns on and sees no failure** on a running clock;
  the MCO takes its selections; a PB1 gate opens, resets and closes.
- **The tree at boot** after `Clock<pll, 48 MHz>::init()`: SWS = PLL,
  HPRE = 1, HSI on with its trim at 16, PLL locked, LSI off, two wait
  states.

## Not covered yet

Driver gaps, each with its reason:

- HSE as a root, in both forms, and the CSS that watches it: the
  module has no crystal, and a root that cannot be tried is not
  offered - `ClockSource::crystal` and `::external` refuse at compile
  time until a board carries one.
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
- The system clock monitor's failure path: no failure can be provoked
  without a root that can be made to fail, which is an HSE.
