# The clock model

Every target has a clock tree; every driver of that target derives
divisors, baud rates and delays from it. brio makes the clock a TYPE
and states, once and target-independently, how drivers relate to it -
in particular what happens when the rate changes at run time. This
page is the model; the shape of a given tree (which oscillators, which
prescalers or PLL parameters, which rates are reachable) is that
target's business, documented in `docs/targets/`.

Contracts and helpers: `util/clock.hpp` (pure, no hardware). The AVR
DA/DB realization: `avrdx/clock.hpp` (`Clock`, `DynamicClock`) and
`avrdx/delay.hpp` (`delay_us`).

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
  never the caller's.

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
- each user satisfies the **`ClockUser` concept**: `static void
  rebase(uint32_t hz)`, checked where the list is written (a type
  without `rebase` does not compile there);
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

- **The kernel timebase does not move.** Time events, `now()`,
  timeouts and the idle wake-up run from a timebase the target keeps
  independent of the CPU clock (the RTC/PIT on AVR DA/DB, a low-power
  timer elsewhere): a rate change is invisible to the kernel and to
  every AO's timing.
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

## Reference

| Entity | Header | Role |
|--------|--------|------|
| `ClockUser` (concept) | `util/clock.hpp` | what a dynamic clock's user must offer (`rebase(hz)`) |
| `clock_hz(clock)` | `util/clock.hpp` | the rate of either kind of clock |
| `clock_follows<C, Driver>()` | `util/clock.hpp` | the reverse check for a driver's `init(clock)` |
| `Clock<source, hz, div>` | `avrdx/clock.hpp` | AVR DA/DB static clock |
| `DynamicClock<Boot, Users...>` | `avrdx/clock.hpp` | AVR DA/DB dynamic clock over a static Boot |
| `delay_us(clock, us)` | `avrdx/delay.hpp` | AVR DA/DB short wait, "at least" |

Target pages: [../targets/avrdx.md](../targets/avrdx.md).
