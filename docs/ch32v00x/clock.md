# RCC (CH32V00x)

This document is for a programmer bringing a CH32V006 or a CH32V003 to
the rate the program wants, and for one who needs the clock block
itself: the roots, the peripheral gates, the clock output pad, the
clock monitors. It covers the clock tree of RM ch. 3 as this stratum
drives it, where brio's clock model
([design/clock.md](../design/clock.md)) meets a doubling PLL and one
divider. A static `Clock<src, hz, xtal_hz>` names the root and the rate
and is the one truth every driver derives from: a driver asks the clock
type for its rate and computes its own divisors from it, nothing is
read from the hardware. A `DynamicClock` walks the HPRE ladder at run
time with its users rebased first. `Rcc`, under both, is the block
itself: the roots, the switch, the output, the monitor, the peripheral
gates. The program's clock type, with `init()` called once at the top
of `main()`, is what the other pages call the clock task.

The smallest use is the clock type `blink`, the maintained example,
names (this fragment adds a serial port beside it): the HSI doubled by
the PLL to the part's 48 MHz, one rate for the whole run (the "How to
use it" block below shows the moving clock; "a program with one rate
names `Clock<...>` alone"):

```cpp
using SysClock = brio::Clock<brio::ClockSource::pll, 48'000'000>;
constexpr SysClock clock;

int main()
{
    const bool clock_ok = SysClock::init();            // HSI x2 -> 48 MHz
    const bool serial_ok = Serial::init(clock, console_baud);
    const bool tick_ok = brio::Ticker::init(clock);
    ...
}
```

`SysClock` is the type, `clock` an empty tag object of it, the argument
every driver's `init` takes to learn the rate (its functions are all
static: the type has no object of its own to hold); `init()` sets the
flash's wait states for the rate first, writes the divider, starts the
PLL on the HSI and switches the system clock to it. A third template
argument names a crystal instead: `Clock<ClockSource::pll, 48'000'000,
24'000'000>` doubles the bench boards' 24 MHz crystal on PA1/PA2.

Three things decide whether that works. `init()` runs before any
driver's: a driver computes its divisor from the rate the type claims,
so a port or a ticker initialized on a chip still at its 8 MHz reset
rate with a 48 MHz clock type runs six times too slowly until `init()`
has run (its divisor is written once, from the claimed rate), and
nothing refuses it. The rate must be the root's (the HSI's 24 MHz, the
crystal's, or either doubled by the PLL) divided exactly by one of
HPRE's dividers (1 to 8, then 16, 32, 64, 128, 256), and no more than
48 MHz: any other rate is a compile error, never a rounded one, and so
is a crystal outside 4 to 25 MHz. And `init()` answers false when a
root never reports ready or the switch never takes, within a bounded
number of turns; the program decides what to say about it (a program
with a console can print the answer beside the clock's name), and the
drivers it then initializes compute their divisors from a rate the chip
is not running at.

The rest of this page, in order: the documents of record; what the
silicon does, the two parts' differences among it; the types and verbs;
how to use it, the moving clock; the bench findings, the whole ladder
of rates and the crystal against a host's clock among them; and what is
not covered.

## Documents of record

The CH32V00X reference manual V1.5 (3.3 for the
tree, 3.4 for the registers), the CH32V006 datasheet V2.0 (the HSI's
and LSI's accuracy, the MCO's pad). The header is
[brio/ch32v00x/clock.hpp](../../brio/ch32v00x/clock.hpp); the reference
suite is `test_ch32_clock` (a suite's letter, below, is one command key
of its console and the group of verdicts it runs, `z` the key that runs
the image's set of letters, a letter outside it run by its own
key).

## What the silicon does

- **Three roots for SYSCLK**: the 24 MHz internal RC (HSI, on out of
  reset, factory-calibrated through HSICAL and nudged by a five-bit
  HSITRIM), the HSE (a crystal of 4 to 25 MHz on PA1/PA2, or a clock
  into PA1 with HSEBYP) and the PLL, which has no ratio to choose - it
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
  brakes TIM1 - on both parts; the SCM (SYSCM_EN) watches the system
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
its first proof. On the CH32V006K8 it is two images: the second, letters
`h` and `k` (5 verdicts in its `z`), boots on the crystal ITSELF.

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
- **THE CRYSTAL AS SYSCLK ITSELF** (letter `h`, the second image, ten
  runs): `Clock<crystal, 24 MHz, 24 MHz>::init()` from reset leaves SWS
  the HSE, HPRE 1, the HSE on and ready with its pads the oscillator's
  and no bypass, the PLL never turned on, the wait states for 24 MHz,
  no clock failure - and the console at 115200 on it.
- **The roots against a host's clock** (letter `k`, by name: 30 000 ticks
  between two console lines, time-stamped as they arrive by a host on
  an NTP-disciplined clock, the probe's bridge between them): the
  crystal as SYSCLK read 25 to 38 ppm fast of the host over 30 s (three
  runs), the PLL's 48 MHz from it 38 and 40 ppm (two) - one oscillator,
  within the bracket's resolution of a millisecond or so of arrival
  jitter, some 40 ppm. The HSI's PLL at 48 MHz, which the other suites
  run on, reads 0.5 per cent slow: the sleep suite's long AWU windows
  give the LSI 125.71 kHz on its STK and 125.07 kHz on the host's clock
  ([sleep.md](sleep.md)).

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

- The rates of the divided rungs in hertz and the trim steps' worth:
  the ladder is proven by the console reading clean at each rung
  (within UART tolerance) and by tick counts that the rate itself
  clocks, and the roots are measured against the host (above); letter
  `k` at a divided rung, or the trim stepped on the HSI's image, would
  put the rest in parts per million.
- The system clock monitor's failure path and the CSS's: a failure
  needs a crystal that stops, which a soldered one does not; the CSS's
  NMI also needs a handler bound by the program that arms it.
- The bypass (`ClockSource::external`, a clock into PA1 with HSEBYP):
  compiled and refused where the chapter refuses (the rate outside
  4..25 MHz, a PLL past 48 MHz, a rate not named); both bench boards
  carry a crystal on PA1/PA2, so a board with an oscillator's output on
  PA1 instead would measure it.
