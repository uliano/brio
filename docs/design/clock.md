# The clock model

Every target has a clock tree; every driver of that target derives
divisors, baud rates and delays from it. brio makes the clock a TYPE
and states, once and target-independently, how drivers relate to it -
in particular what happens when the rate changes at run time. This
page is the model; the shape of a given tree (which oscillators, which
prescalers or PLL parameters, which rates are reachable) is that
target's business, documented in that target's folder (`docs/avrdx/`, ...).

Contracts and helpers: `util/clock.hpp` (pure, no hardware). The AVR
DA/DB realization: `avrdx/clock.hpp` (`Clock`, `DynamicClock` over the
CLKCTRL resources - see [clkctrl.md](../avrdx/clkctrl.md)) and `avrdx/delay.hpp`
(`delay_us`).

## One truth for the rate

The rate at which the CPU and its peripherals run has exactly one
authoritative source: the clock type the application names. Not a
build macro (the vendor's `F_CPU`-style define is removed from the
build: two truths cannot diverge, and vendor headers that need it stop
compiling on purpose), not a number repeated in a driver. Drivers ask
the clock:

- `clock_hz(clock)` - the current rate. A `constexpr` for a static
  clock (divisors fold at compile time), a value for a dynamic one.
  Drivers write the same line either way.
- `Clock::init()` first thing in `main()`: brings the tree up and
  reports whether the requested source is running.

The clock is a **monostate type**, and the app also declares an empty
tag object (`constexpr SysClock clock;`) to pass where a driver needs
the rate - `Serial::init(clock, baud)`, `delay_us(clock, 10)` - the
same idiom as `print(serial, ...)`.

## Two regimes, one shape

- **Static.** `is_static == true`, `hz` a compile-time constant. The
  common case: the tree is configured once and never moves. Divisors
  fold; a wrong assumption is a compile error.
- **Dynamic.** `is_static == false`, `hz()` a value; `set<hz>()`
  (rate known at compile time: an unreachable rate is a compile
  error) or `set(hz)` (run time: `false`, nothing changed, when
  unreachable) switch under the running program. The app speaks Hz;
  which prescaler or PLL setting produces them is the target's detail,
  never the caller's. A dynamic clock's rate is nevertheless one of a
  DISCRETE SET the type knows in full (the boot rate over the target's
  prescalers), and the type says so: `rate_count`, `rate_hz(i)` and
  the runtime `rate_index()` expose the set and the current position
  in it. That is what lets rate-derived arithmetic be expanded per
  rate at compile time and selected by an index byte at run time -
  the target's `delay_us` dispatches this way and never divides at
  wait time - instead of being recomputed from `hz()` as a value.

The two are sibling types with the same driver-facing surface: a
driver written for `init(clock)` + `clock_hz(clock)` serves both, and
adds one function - `rebase(hz)` - to serve the dynamic one.

## A rate change is a synchronous fan-out, not an event

When a dynamic clock switches, every driver that derived something
from the old rate must adopt the new one - and it must have done so
BEFORE the rate actually changes, or bytes already queued go out at
the wrong baud. That rules out the kernel's event queues: an event
posted to a low-priority AO would be served after the switch, too
late. So:

- the users of a dynamic clock are a **compile-time list of types** in
  the clock's template arguments (`DynamicClock<Boot, Serial, Twi0>`),
  exactly like the AO pack of the Kernel or a `Subscribers<...>` list;
- each user satisfies the **`ClockUser` concept**: a static
  `rebase(hz)`, checked where the list is written (a type
  without `rebase` does not compile there). Today's users on AVR DA/DB:
  `Uart` (drains TX, new BAUD), `Twi` (both MBAUDs), `Spi` (setup
  timing base), `Adc` (a prescaler keeping CLK_ADC in range);
- `set()` calls every user's `rebase(next)` **in list order,
  synchronously** - a fold expression over the pack, unrolled by the
  compiler into direct calls, no table, no RAM - and **only then**
  reprograms the hardware. A user may drain what it has in flight at
  the old rate inside its `rebase` (the Uart waits for its TX ring and
  one frame time before loading the new baud);
- **the reverse check**: a driver initialized with a dynamic clock
  asserts `clock_follows<Clock, Driver>()` in its `init(clock)` - it
  must be among that clock's users, or the build stops with a message
  naming it. A forgotten user cannot silently keep the old rate.

Publish semantics, call mechanics: everyone who cares hears about the
change, all of them before it happens, none of them through a queue.

## What does not move, and what the caller owes

- **The kernel timebase does not move - where the target keeps it off
  the core clock.** Time events, `now()`, timeouts and the idle wake-up
  run from a timebase the CPU rate does not reach (the RTC/PIT on AVR
  DA/DB, the LPTIM on the crystal of the STM32G0's tickless platform):
  a rate change is invisible to the kernel and to every AO's timing.
  The two SysTick tickers (SAM C21, the STM32G0's default) ride the
  core clock instead, and REFUSE a dynamic clock by construction - their
  `init(clock)` asserts `clock_follows`, and no `rebase` exists to
  satisfy it - until the day one is written; on the STM32G0 that day
  need not come, the LPTIM timebase being the right road (below).
- **The caller picks the moment.** A driver's `rebase` can quiesce
  its own hardware, but a bus transaction in flight (SPI, I2C) belongs
  to a bus AO's FSM: switching in the middle of one corrupts it. The
  designed shape is a small power-manager AO that asks the bus AOs
  (idle state) through request/reply before calling `set()` - not
  built yet; today the callers are console commands with nothing on
  the wire.
- **`delay_us` follows the clock.** The short-wait role ("at least
  N microseconds", for hardware setup times inside drivers, never for
  waiting in an AO) reads its cycle budget from `clock_hz(clock)`:
  folded for a static clock, computed for a dynamic one. How the wait
  is produced is per target (a cycle-calibrated loop where the core
  is deterministic, a hardware counter where it is not).

## The other targets: the deferral, and the shape the STM32G0 takes

**The SAM C21 has no dynamic clock, by ruling.** Every peripheral there
has a generic clock channel of its own, so "one rate for everything" is
an AVR assumption and no program has asked to rescale; the question
reopens with its first genuine consumer, together with the fan-out and
the ticker's `ClockUser` question ([../samc21/clock.md](../samc21/clock.md)).

**What the first target measured, and what it does not settle.** The
family that CAN do both was made to choose between a fixed clock plus
sleep and clock adaptation on an event-logging load, in joules: without
voltage scaling active current is `I0 + k * f` and a slower clock costs
MORE per cycle, so adaptation cannot win on compute and its one asset is
the wake it avoids - and the same experiment found that HOW the program
watches for its events is worth a factor ~1.7, more than any clock
choice (a frugal watcher on a fixed clock beat the adaptive program by
27 %). That vindicates the SAM ruling on a family without voltage
scaling. The STM32G0 HAS a second voltage range and a low-power
regulator whose whole point is running at 2 MHz cheaply - the one lever
the first target lacked - so there the question is open, not answered.

**The STM32G0: deferred with its design written, until a consumer.**
Nothing rescales on the third target today (`Clock<source, hz>` is
static, one root at a time), and the two consumers that would open it
are named: a LOW-POWER-RUN program - SYSCLK at 2 MHz from HSISYS/8,
VCORE Range 2, the regulator in low-power mode, the kernel clocks on
HSI16 and the crystal, the tickless timebase, waking on the RTC, the
LPTIM or a pad and NEVER on a clock-requesting peripheral (ES0548 2.2.4:
a divided HSISYS cannot be woken from Stop by a USART or an I2C,
measured) - or the energy experiment's own STM32G0 instance, with the
SAM at 3.3 V as its meter. Whichever comes first brings the campaign.
The shape it takes, so that the consumer starts from a design and not
from a blank page:

- **A rate is a TUPLE, not a divisor.** On the AVR a dynamic clock's
  rate is the boot rate over one prescaler and the discrete set is an
  array the type indexes. Here a rate is (SYSCLK source and rate, the
  VCORE range, the regulator mode): 64 MHz on the PLL in Range 1;
  16 MHz on HSISYS; 2 MHz on HSISYS/8 in Range 2 on the low-power
  regulator. The reachable rates come from two disjoint families
  (HSI16 over 2^k, and the exact M/N/R ratios of the PLL) with no single
  prescaler to index, so the discrete set is an EXPLICIT PACK the
  program names - `DynamicClock<Boot, Rate<...>, Rate<...>, Users...>`
  - each rate resolved at compile time to its HSIDIV or PLL setting,
  its flash latency (the Range 1 and Range 2 columns both exist) and
  its range; `rate_count`, `rate_hz(i)` and `rate_index()` keep this
  page's discrete-rate surface unchanged, and `delay_us` dispatches by
  index as on the AVR.
- **The switch is direction-aware and interleaves two ladders the AVR
  has not got.** The fan-out precedes the switch in BOTH directions -
  this page's contract, unchanged - and the target's own steps sit
  around it: rising = the range up and its VOSF wait, then the wait
  states up, then the fan-out, then the switch; falling = the fan-out,
  the switch, the wait states down, the range down, and the low-power
  regulator last, because 4.3.2 wants the clock at or below 2 MHz
  BEFORE it and REGLPF clear before any rise. `Clock<>::init()` is
  already that sequence with compile-time settings (the PLL parked on
  HSISYS before it is touched, the switch read back), which is the
  body `set()` would run with runtime ones.
- **The fan-out is small by construction, and that is the G0's own
  gift.** The kernel-clock multiplexers (RCC_CCIPR) take a peripheral
  OFF SYSCLK: a USART or LPUART on HSI16 or the crystal, an LPTIM on the
  crystal, the ADC on HSI16, the RTC and the IWDG never on it at all -
  and on the tickless platform the kernel timebase is off it too. A
  low-power program's users are therefore few: the SysTick cycle
  counter (`delay_us`'s), a console left on PCLK, the general-purpose
  timers. The inventory as the drivers stand: the `Uart` FOLLOWS (a
  `ClockUser` whose `rebase` drains and reloads BRR, and folds to a
  no-op for a port on HSI16 or LSE); `BasicTicker` and `SysTickCounter`
  would restart their reload (the SysTick ticker loses ticks across the
  restart, one more reason a rescaling program is a tickless one); the
  `Adc` and the timers REFUSE a dynamic clock at compile time until they
  grow a `rebase` (the ADC's is a prescaler that keeps its clock in
  range, the AVR precedent; a timer's a new PSC for the same period);
  the FDCAN and the window watchdog take the bus rate as a NUMBER and
  keep it (the CAN's bit timing wants a fixed tq clock and its PLLQ/HSE
  roots are not built; the watchdog's timeout scales with PCLK and a
  rescaling program re-arms it). Where the AVR listed every clocked
  driver, the G0 lists the ones it chose not to move off SYSCLK.
- **A Stop is a rate change the fan-out never saw.** The part comes out
  of Stop 0/1 on HSISYS at 16 MHz with the PLL off; today the sleep
  site re-runs the static clock task at the first thing that executes
  after the wake ([../stm32g0/pwr.md](../stm32g0/pwr.md)). With a
  dynamic clock the site restores the CURRENT rate (`rate_index()`),
  not the boot one, and a program parked on a divided HSISYS keeps its
  clock across the Stop (HSIDIV survives) but keeps ES0548 2.2.4 too.
- **What a campaign benches first**, because every one of these sits on
  the switch's critical path and none has run on silicon: the HSIDIV
  write under a running core, the PLL's refusal while running (the
  teardown before a reconfigure), a wait-state DECREASE, and Range 2
  with its own latency column.

The refusals above are code today, so the design states nothing the
build does not enforce: a static clock is what every STM32G0 driver
but the `Uart` and the timebase accepts, and a dynamic one is a compile
error naming this page.

## Reference

| Entity | Header | Role |
|--------|--------|------|
| `ClockUser` (concept) | `util/clock.hpp` | what a dynamic clock's user must offer (`rebase(hz)`) |
| `clock_hz(clock)` | `util/clock.hpp` | the rate of either kind of clock |
| `clock_follows<C, Driver>()` | `util/clock.hpp` | the reverse check for a driver's `init(clock)` |
| `Clock<source, hz, div>` | `avrdx/clock.hpp` | AVR DA/DB static clock |
| `DynamicClock<Boot, Users...>` | `avrdx/clock.hpp` | AVR DA/DB dynamic clock over a static Boot |
| `delay_us(clock, us)` | `avrdx/delay.hpp` | AVR DA/DB short wait, "at least" |
| `Clock<source, hz>` | `samc21/clock.hpp`, `stm32g0/clock.hpp` | the SAM C21's and the STM32G0's static clocks (no dynamic one: the deferral above) |
| `delay_us(clock, us)` | `armv6m/delay.hpp` | both Cortex-M0+ families' short wait, on SysTick's counter |

Target pages: [../avrdx/README.md](../avrdx/README.md),
[../samc21/clock.md](../samc21/clock.md),
[../stm32g0/clock.md](../stm32g0/clock.md).
