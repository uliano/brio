# The clock model

This page decides how a program and its drivers know the rate the
chip runs at: the clock is a type the application names once, its
rate is the one truth every divisor and delay is derived from, and a
rate change at run time reaches every clocked driver before the
hardware moves. It is written for an embedded programmer used to an
`F_CPU` macro and a vendor's clock configuration tool, and says what
replaces each. The shape of a given tree (which oscillators, which
prescalers or PLL parameters, which rates are reachable) is that
target's business, documented in that target's folder.

The smallest use: `using SysClock = brio::Clock<ClockSource::crystal,
24'000'000>;` (the AVR spelling; each family's `clock.hpp` has its
own parameters), `SysClock::init()` first thing in `main()`, and the
tag object `constexpr SysClock clock;` passed to every driver that
needs the rate. The prerequisite that decides correctness: **no
`F_CPU`-style macro exists in the build**, so a vendor header that
needs one stops compiling on purpose, and a driver initialized with a
dynamic clock must be in that clock's list of users or the build
stops.

Contracts and helpers: `util/clock.hpp` (pure, no hardware). The AVR
DA/DB/EA realization: `avrxt/clock.hpp` (`Clock`, `DynamicClock` over
the CLKCTRL resources, [the AVR clock controller page](../avrxt/clkctrl.md))
and `avrxt/delay.hpp` (`delay_us`).

## One truth for the rate

The rate at which the CPU and its peripherals run has exactly one
authoritative source: the clock type the application names. Not a
build macro, and not a number repeated in a driver. Drivers ask the
clock:

- `clock_hz(clock)`: the current rate. A `constexpr` for a static
  clock, so divisors fold at compile time, and a value for a dynamic
  one. Drivers write the same line either way.
- `Clock::init()` first thing in `main()`: brings the tree up and
  reports whether the requested source is running.

The clock is a monostate type (a class with no instances and only
static members), and the application also declares an empty tag
object (`constexpr SysClock clock;`) to pass where a driver needs the
rate (`Serial::init(clock, baud)`, `delay_us(clock, 10)`), the same
idiom as `print(serial, ...)`.

## Two regimes, one shape

- **Static.** `is_static == true`, `hz` a compile-time constant. The
  common case: the tree is configured once and never moves. Divisors
  fold; a wrong assumption is a compile error.
- **Dynamic.** `is_static == false`, `hz()` a value; `set<hz>()` (the
  rate known at compile time: an unreachable rate is a compile error)
  or `set(hz)` (at run time: `false` and nothing changed when
  unreachable) switch under the running program. The application
  speaks hertz; which prescaler or PLL setting produces them is the
  target's detail, never the caller's. A dynamic clock's rate is
  nevertheless one of a discrete set the type knows in full (the boot
  rate over the target's prescalers), and the type says so:
  `rate_count`, `rate_hz(i)` and the run-time `rate_index()` expose the
  set and the current position in it. That is what lets rate-derived
  arithmetic be expanded per rate at compile time and selected by an
  index byte at run time (the target's `delay_us` dispatches this way
  and never divides at wait time) instead of being recomputed from
  `hz()` as a value.

The two are sibling types with the same driver-facing surface: a
driver written for `init(clock)` and `clock_hz(clock)` serves both,
and adds one function, `rebase(hz)`, to serve the dynamic one.

### Realizations

Common to all: `Clock<ClockSource::internal, hz>` and
`Clock<ClockSource::crystal, hz>` spelled identically, `Clock::hz` as
the one rate truth, `clock_hz(clock)` and the `ClockUser` contract
(`util/clock.hpp`), and `delay_us(clock, us)` reading its budget from
it. What differs is the shape of the dynamic regime, which is where
each family's own clock tree shows through, and the sources beyond
the two every part has.

| stratum | realization | beyond the contract |
|---|---|---|
| avrxt | `Clock<src, hz, div, oschf_hz>` and `DynamicClock<Boot, Users...>` (`avrxt/clock.hpp`) | the dynamic set is the boot rate over the twelve main prescalers; `set<hz>()`/`set(hz)` fan `rebase` out then switch; `ClockSource` adds `external`, `osc32k`, `xosc32k`; on the AVR EA OSCHF runs at the rate its OSCCFG fuse chose, so that rate is a claim in the type (`oschf_hz`, the fallback's rate under a crystal), checked against the fuse at boot, and the clock owns MCLKTIMEBASE; the EA has no PLL and its dynamic set boots at the source's rate; `delay_us` returns nothing, having no cap |
| samc21 | `Clock<src, hz>` alone (`samc21/clock.hpp`) | no dynamic clock, by position: every peripheral has a generic clock channel of its own, so "one rate for everything" is not this family's shape (below); `Clock` builds `internal` alone today, the crystal and the DPLL being resources the program runs the CPU from by hand; `ClockSource` adds `dpll` |
| stm32g0 | `Clock<src, hz, regime>` and `DynamicClock<Rates<R0, R1, ...>, Users...>` (`stm32g0/clock.hpp`) | a rate is a tuple (root, VCORE range, regulator) and the dynamic set an explicit pack of such tuples, R0 the boot rate; `PowerRegime` sequences table 13's latency and the range around a switch; `restore()` after a Stop; `ClockSource` adds `pll`, `lsi`, `lse` (the 32 kHz roots under ST's names) |
| stm32f4 | `Clock<src, hz, hse_hz, hse_mode>` and `DynamicClock<Rates<R0, R1, ...>, Users...>` (`stm32f4/clock.hpp`) | the STM32G0's model with the APB prescalers unpinned: `pclk1_hz`/`pclk2_hz` beside `hz` (45 and 90 MHz at 180) and `apb_hz(clock, bus)` for a peripheral's own rate, `clock_hz` still HCLK; the sources spell the root (`hsi`, `hse`, `pll_hsi`, `pll_hse` with the HSE's rate and mode); a rate is a tuple of five (root, regulator scale, over-drive, flash latency, both APB prescalers) and the frequency ladders are keyed on the part class in the device tables, a rate refused where no manual was read; the dynamic switch is not direction-aware because it cannot be: neither the PLL nor the scale nor the over-drive bits may be written while the PLL is SYSCLK, so every switch parks on the HSI and everything after the park is a rise; `restore()` after a Stop re-runs the rate in force with no fan-out |
| ch32v00x | `Clock<src, hz>` and `DynamicClock<Boot, Users...>` (`ch32v00x/clock.hpp`) | the AVR's shape, the simplest of all: one root (the HSI, or `ClockSource::pll`, the PLL being a fixed doubler of it) and one divider, HPRE, so the dynamic set is the boot rate over the divider's ladder; `set<hz>()`/`set(hz)` fan `rebase` out then move the divider, the flash wait states raised before a rise and lowered after a fall; `restore()` puts the rate back after a Standby, from which the core wakes on the HSI; `crystal` is named and refused until a board carries one; `delay_us` on the STK counter, refused at a tick period or more |
| ch32vx03 | `Clock<src, hz, xtal_hz>` and `DynamicClock<Rates<R0, R1, ...>, Users...>` (`ch32vx03/clock.hpp`) | the STM32F4's model: a rate is a tuple and every switch parks on the HSI, here for the family's own reason (PLLMUL, PLLSRC and PLLXTPRE take a write only with the PLL off, and the PLL will not stop while it is SYSCLK), so the static task's `init()` is the dynamic switch and one order serves both directions; the peripheral prescalers are pinned by a cap rather than a ladder, `pclk1_hz`/`pclk2_hz` beside `hz` (PB1 halved above 72 MHz, PB2 never) with `pclk1_hz_at(hclk)` the arithmetic a rebased user derives its own bus rate from, and `timclk1_hz`/`timclk2_hz` for the doubling a divided bus gives its timers; the tuple carries two members the F4's has not, the USB divider (48 MHz or nothing) and the ADC's, the one that can be out of specification at a legal rate; `ClockSource` adds `pll` and `external`, and the PLL's input divider, its input and output ranges and the USB prescaler's fourth code are device-class facts read from the part table, not constants of the file; `restore()` after a wake that dropped the tree to the HSI; `delay_us` on the STK counter, refused at a tick period or more; the CH32V303 (CH32V30x_D8) has the CH32V203C8's tree (PLLXTPRE by one or two, USBPRE's three codes, the same PLL ranges), so its parts state those facts with the same values, and the ADC clock's 75 % duty its class adds is a function that asks the die (`Rcc::adc_duty_75()`), a lot's bit |
| ch32x035 | `Clock<internal, hz>` and `DynamicClock<Boot, Users...>` (`ch32x035/clock.hpp`) | the CH32V00x's shape with less under it: one root, the 48 MHz HSI (no PLL, no crystal oscillator, no clock switch), and one divider, HPRE, sixteen codes from /1 to /256 and /6 out of reset; `set<hz>()`/`set(hz)` fan `rebase` out then move the divider, the flash wait states raised before a rise and lowered after a fall; no bus prescaler at all, so `pclk_hz` is `hz` for every block; `restore()` for a sleep that stopped the tree; `delay_us` on the STK counter, refused at a tick period or more ([the CH32X035 clock page](../ch32x035/clock.md)) |
| rp2040 | `Clock<src, hz, crystal_hz, peri>` alone (`rp2040/clock.hpp`) | no dynamic clock yet, and a tree of a shape none of the others has: not one root feeding prescaled buses but a generator per clock domain, each with its own mux and its own fractional divider, so clk_sys, clk_peri, clk_adc, clk_usb and clk_rtc are five independent rates and there is no bus prescaler at all. The task therefore states two numbers, `hz` for clk_sys and `pclk_hz` for clk_peri, and the fourth template parameter picks which of them clk_peri follows: clk_sys undivided by default, or the crystal, under which the serial ports keep their rate through every change of clk_sys, which is the shape a dynamic clock would be built on here; `ClockSource` is `crystal` and `pll` only (the ring oscillator has no exact rate to be a truth, and `internal` is not declared), the PLL ratio searched exactly at compile time from the crystal; `delay_us` is `cortexm/delay.hpp`'s on clk_sys |
| rp2350 | `Clock<src, hz, crystal_hz, peri>` alone (`rp2350/clock.hpp`) | the RP2040's model with this chip's tree: a generator per domain again, with a fourth root under clk_ref (a low-power oscillator in the always-on domain), one generator more, no clk_rtc at all, and dividers of 16.16 where the RP2040's were 24.8, so the task states three numbers, `hz` for clk_sys, `pclk_hz` for clk_peri and `ref_hz` for clk_ref, that last one being what the tick generators divide; the tick generators are a block of their own here, one per consumer, and what they hand out is the microsecond the timers, the watchdog and the RISC-V platform timer all count, which is why `delay_us` is not `cortexm/delay.hpp`'s on this family but a wait on that counter, one implementation for both instruction sets, immune to a clk_sys switch under it, one microsecond of resolution as the price; two CTRL registers of this tree are passwords, where a masked write is a refused write |
| host | none | the host tests run on the virtual clock of `HostPlatform`; nothing there has a rate |

`delay_us` is one name with one difference of contract: on every
target but the AVR it is capped below one kernel tick and refuses (a
`bool`, nothing spent) a wait that long, a tick or more being
`TimeEvent` territory, where the AVR's has no cap and returns nothing
(`avrxt/delay.hpp`; `cortexm/delay.hpp` for the Cortex-M families
that ride SysTick's counter; `ch32v00x/delay.hpp`, `ch32vx03/delay.hpp`
and `ch32x035/delay.hpp` on the three QingKe families' STK;
`rp2350/delay.hpp` on a timer that is neither core's, because that
chip's two architectures do not share one).

## A rate change is a synchronous fan-out, not an event

When a dynamic clock switches, every driver that derived something
from the old rate must adopt the new one, and it must have done so
before the rate actually changes, or bytes already queued go out at
the wrong baud. That rules out the kernel's event queues: an event
posted to a low-priority active object would be served after the
switch, too late. So:

- the users of a dynamic clock are a compile-time list of types in
  the clock's template arguments (`DynamicClock<Boot, Serial, Twi0>`),
  exactly like the active-object list of `Tenuto` or a
  `Subscribers<...>` list;
- each user satisfies the `ClockUser` concept, a static `rebase(hz)`,
  checked where the list is written (a type without `rebase` does not
  compile there). Today's users on the AVR DA/DB/EA: `Uart` (drains
  its transmit ring, then a new BAUD), `Twi` (both MBAUDs), `Spi` (its
  setup timing base), `Adc` (a prescaler keeping CLK_ADC in range);
- `set()` calls every user's `rebase(next)` in list order,
  synchronously (a fold expression over the pack, unrolled by the
  compiler into direct calls, no table, no RAM), and only then
  reprograms the hardware. A user may drain what it has in flight at
  the old rate inside its `rebase` (the `Uart` waits for its transmit
  ring and one frame time before loading the new baud);
- the reverse check: a driver initialized with a dynamic clock asserts
  `clock_follows<Clock, Driver>()` in its `init(clock)`. It must be
  among that clock's users, or the build stops with a message naming
  it. A forgotten user cannot silently keep the old rate.

Publish semantics with call mechanics: everyone who cares hears about
the change, all of them before it happens, none of them through a
queue.

## What does not move, and what the caller owes

- **The kernel timebase does not move, where the target keeps it off
  the core clock.** Time events, `now()`, timeouts and the idle wake-up
  run from a timebase the CPU rate does not reach (the RTC's PIT on
  the AVR DA/DB, the LPTIM on the crystal of the STM32G0's tickless
  platform): a rate change is invisible to the kernel and to every
  active object's timing (measured: a 50-tick periodic timer through
  ten switches, none early). The SysTick ticker the Cortex-M families
  share rides the core clock instead and is a `ClockUser` that
  restarts its period at a switch, losing the phase of the tick in
  progress, under a tick, late and never early, which is why a
  rescaling program on the STM32G0 is a tickless one; on the SAM C21,
  which has no dynamic clock, the question does not arise.
- **The caller picks the moment.** A driver's `rebase` can quiesce its
  own hardware, but a bus transaction in flight (SPI, I2C) belongs to
  a bus active object's state machine: switching in the middle of one
  corrupts it. The designed shape is a small power-manager active
  object that asks the bus active objects (idle state) through request
  and reply before calling `set()`; it is not built yet, and today the
  callers are console commands with nothing on the wire.
- **`delay_us` follows the clock.** The short-wait role ("at least N
  microseconds", for hardware setup times inside drivers, never for
  waiting in an active object) reads its cycle budget from
  `clock_hz(clock)`: folded for a static clock, computed for a dynamic
  one. How the wait is produced is per target (a cycle-calibrated loop
  where the core is deterministic, a hardware counter where it is
  not).

## The other targets: the SAM C21's static clock, the STM32G0's tuple

**The SAM C21 has no dynamic clock.** Every peripheral there has a
generic clock channel of its own, so "one rate for everything" is an
AVR assumption and no program has asked to rescale; the question
reopens with its first genuine consumer, together with the fan-out
([the SAM C21 clock page](../samc21/clock.md)).

**What one family measured, and what it does not settle.** A family
that can do both was made to choose between a fixed clock plus sleep
and clock adaptation on an event-logging load, in joules: without
voltage scaling the active current is `I0 + k * f` and a slower clock
costs more per cycle, so adaptation cannot win on compute and its one
asset is the wake it avoids; and the same experiment found that how
the program watches for its events is worth a factor of about 1.7,
more than any clock choice (a frugal watcher on a fixed clock beat the
adaptive program by 27 %). On a family without voltage scaling, that
is the argument for leaving the clock alone. The STM32G0 has a second
voltage range and a low-power regulator whose whole point is running
at 2 MHz cheaply, the one lever the AVR lacks, so there the question
is open, not answered.

**The STM32G0 has one, and its rate is a tuple.** The consumer that
opened it is a low-power-run program: SYSCLK at 2 MHz from HSISYS/8 on
the low-power regulator, the kernel clocks on HSI16 and the crystal,
the tickless timebase, waking on the RTC, the LPTIM or a pad and never
on a clock-requesting peripheral (ES0548 2.2.4: a divided HSISYS
cannot be woken from Stop by a USART or an I2C, measured). The shape,
as built ([the STM32G0 clock page](../stm32g0/clock.md)):

- **A rate is a tuple, not a divisor.** On the AVR a dynamic clock's
  rate is the boot rate over one prescaler and the discrete set is an
  array the type indexes. Here a rate is (the SYSCLK source and rate,
  the VCORE range, the regulator): 64 MHz on the PLL in Range 1;
  16 MHz on HSISYS in Range 2; 2 MHz on HSISYS/8 on the low-power
  regulator. The reachable rates come from two disjoint families
  (HSI16 over 2^k, and the exact M/N/R ratios of the PLL) with no
  single prescaler to index, so the discrete set is an explicit pack
  the program names, `DynamicClock<Rates<R0, R1, ...>, Users...>`,
  each member a static `Clock<source, hz, regime>` resolved at compile
  time to its HSIDIV or PLL setting, its flash latency (the Range 1
  and Range 2 columns both exist) and its regime, `R0` the boot rate;
  `rate_count`, `rate_hz(i)` and `rate_index()` keep this page's
  discrete-rate surface unchanged, and `delay_us` dispatches by index
  as on the AVR (`delay_rates`, a table built at compile time over
  that surface).
- **The switch is direction-aware and interleaves two ladders the AVR
  has not got.** The fan-out precedes the switch in both directions
  (this page's contract, unchanged) and the target's own steps sit
  around it. Rising: low-power run left and REGLPF clear, the range up
  and VOSF clear, then the fan-out, then the rate's own `apply()` (the
  wait states up, then the root). Falling: the fan-out, then the
  rate's `apply()` (the root, the wait states down, the range down,
  and the low-power regulator last, because 4.3.2 wants the clock at
  or below 2 MHz before it). A static `Clock<>` in Range 2 or
  low-power run is that falling sequence on its own, so it stands
  alone as a boot clock and its `apply()` is exactly the body a switch
  runs, `init()` being `apply()` plus the one store a boot owns and a
  switch must not repeat, this family's flash prefetch default; a
  Range 1 rate refuses a core it finds in Range 2, because raising it
  is the orderer's step. Measured, every rung as the type claims and
  the CPU at the claimed rate on the crystal's scale; a switch costs
  48 us (64 to 16) to 384 us (2 MHz low-power run to 64: REGLPF, VOSF,
  the PLL lock).
- **The fan-out is small by construction, and that is the G0's own
  gift.** The kernel-clock multiplexers (RCC_CCIPR) take a peripheral
  off SYSCLK: a USART or LPUART on HSI16 or the crystal, an LPTIM on
  the crystal, the ADC on HSI16, the RTC and the IWDG never on it at
  all, and on the tickless platform the kernel timebase is off it too.
  A low-power program's users are therefore few. The inventory as
  built: the `Uart` follows (a `ClockUser` whose `rebase` drains and
  reloads BRR, and folds to a no-op for a port on HSI16 or LSE, so a
  console left there never notices a switch); `BasicTicker` and
  `SysTickCounter` follow (a new reload; the ticker's period restarts
  and loses the phase of the tick in progress, under a tick, late);
  the `Adc` follows in a PCLK mode (the next larger division when the
  configuration's own would put fADC over its ceiling, a pure function
  of configuration and rate, the AVR precedent; a no-op in the
  asynchronous mode); the timers refuse (every period, prescaler and
  capture is in PCLK cycles, and no `rebase` can promise the same
  periods at the prescaler's granularity, so a rescaling program keeps
  its timers on what does not move, the LPTIM and the RTC); the FDCAN
  and the window watchdog take the bus rate as a number and keep it
  (the CAN's bit timing wants a fixed tq clock and its PLLQ/HSE roots
  are not built; the watchdog's timeout scales with PCLK and a
  rescaling program re-arms it). Where the AVR listed every clocked
  driver, the G0 lists the ones it chose not to move off SYSCLK.
- **A Stop is a rate change the fan-out never saw.** The part comes
  out of Stop 0/1 on HSISYS with the PLL off, HSIDIV and LPR kept; the
  dynamic clock's `restore()` re-runs the current rate (`rate_index()`)
  with no fan-out (the users hold the divisors for exactly that rate)
  or finds it in force (every HSISYS rate; a 2 MHz low-power-run
  program wakes in low-power run, measured) and does nothing. The
  plain sleep site calls it at `disarm()`, which the manager reaches at
  the wake convention, after the active object the deadline was for
  has run, at 16 MHz with the PLL off; a program that wants its rate
  back before any active object runs calls `restore()` from the wake's
  own interrupt handler (legal there: it does nothing while a `set()`
  is in progress in thread mode). And HSISYS is never left divided
  behind a PLL rate: the divider a program climbed from is written
  back to 0 after every switch onto the PLL, or a Stop would land on
  2 MHz instead of 16 with ES0548 2.2.4's hazard along.
- **The four steps the design named as unbenched** (the HSIDIV write
  under a running core, the PLL's refusal while running, a wait-state
  decrease, and Range 2 with its own latency column) all ran on
  silicon in the dynamic clock's own suite, and behaved as the
  chapters say.

What the build enforces: a driver initialized with a dynamic clock is
among its users or does not compile (`clock_follows`); a rate outside
a pack's `set<hz>()` does not compile; a Range 2 rate above 16 MHz and
a low-power-run rate above 2 MHz do not compile; the timers and the
three USART personalities without a `rebase` refuse a dynamic clock
at their `init`.

## Reference

| Entity | Header | Role |
|--------|--------|------|
| `ClockUser` (concept) | `util/clock.hpp` | what a dynamic clock's user must offer (`rebase(hz)`) |
| `clock_hz(clock)` | `util/clock.hpp` | the rate of either kind of clock |
| `clock_follows<C, Driver>()` | `util/clock.hpp` | the reverse check for a driver's `init(clock)` |
| `Clock<...>` | each family's `clock.hpp` (the realizations table) | that family's static clock, its parameters the tree's |
| `DynamicClock<...>` | the families with one (the realizations table) | that family's dynamic clock over a boot clock or a pack of rate tuples |
| `delay_us(clock, us)` | `avrxt/delay.hpp`, `cortexm/delay.hpp`, the three QingKe families' `delay.hpp`, `rp2350/delay.hpp` | the short wait, "at least"; `delay_rates<Clock>` in `cortexm/delay.hpp` the per-rate table of a dynamic clock |

Target pages: [the AVR page](../avrxt/README.md),
[the SAM C21 clock page](../samc21/clock.md),
[the STM32G0 clock page](../stm32g0/clock.md),
[the CH32X035 clock page](../ch32x035/clock.md).
