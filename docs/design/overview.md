# brio: philosophy, rules, layering

`brio` (from "con brio", the musical marking - deliberately not tied to
any silicon) is a modern-C++ bare-metal framework growing on the
AVR128DB48 testbed, aiming to cover at least the AVR DA/DB families and
to stay portable beyond them. Its core is a QP/QV-style active-object
kernel, written clean-room (concepts from Samek's book, never the QP
source).

## Governing rule

**Nothing is settled.** Reusing existing code is fine only as long as
it does not limit the design above it: whenever a limitation could be
overcome by rewriting the architecture of what sits below, the rewrite
wins. Every stage of work requires critical analysis at ALL levels of
the stack, not just the layer being added.

## Design pillars

- **Everything resolves at compile time.** Peripheral drivers are
  static (monostate) class templates (`Uart<2, Route::alt1>`,
  `BasicTicker<1024>`); services are templated on their transport and
  constrained by concepts. No virtual interfaces, no runtime
  singletons. Monostate drivers double as zero-cost tags
  (`constexpr Serial serial;` usable as a `print()` argument).
- **Drivers move bytes, they do not format.** Text output is
  `brio::print(sink, ...)` - variadic free functions over any ByteSink;
  new types become printable via an ADL `print_one` overload. print
  BLOCKS until the sink accepts: no silent truncation.
- **Protocol layer is push-based.** LineAssembler is fed bytes and
  returns completed lines; no reference to any transport,
  host-testable.
- **One flat namespace `brio`.** Namespaces prevent clashes, they are
  not architecture. Types PascalCase, functions/constants snake_case.
- **Use the freestanding libstdc++** the toolchain ships
  (type_traits, concepts, bit, span, optional, expected - no
  chrono/charconv/iostream) instead of hand-rolled traits.
- **gnu++23** project-wide; C++26 features already in gcc 16 may be
  used when genuinely needed. Idioms in active use:
  `static_assert(false, ...)` in discarded if-constexpr branches for
  "not present on this device" errors, `std::to_underlying` for enum
  codes.

## Generalization rule

Today it is AVR Dx, tomorrow it could be anything else (candidate
targets: STM32G0x0/x1, ATSAMC/D, CH32V00x). No `#ifdef` where a
template/concept boundary can do the job; no AVR includes outside the
platform/driver layer; target-specific facts live behind the Platform
concept or in drivers. The kernel must never know which silicon it
runs on. Family differences inside AVR (DA vs DB, package sizes) are
handled with device-macro guards so the same headers build everywhere.

## Layering: four strata

Directories under `lib/brio/src/`; includes always carry the stratum
prefix (`#include "avrdx/uart.hpp"`) so an app's portability is
readable at a glance.

| Stratum | May include | Content |
|---------|-------------|---------|
| `kernel/` | nothing of brio | pure logic: queues, scheduler, FSM, time events, panic, delivery |
| `util/` | `kernel/` | pure services: print, stream concepts, line parser, Ring, SerialPort, SpiBus, timestamp |
| `avrdx/` | `kernel/`, `util/` | everything that knows `avr/io.h`: drivers + AvrPlatform |
| `host/` | `kernel/`, `util/` | the test "target": HostPlatform |

Two targets never meet in one binary, so a future `ch32v00x/` is a
sibling of `avrdx/` and the flat namespace stays collision-free. One
PlatformIO library, not one per stratum: discipline comes from this
rule, multi-library would add ceremony without enforcement. Shared
pure types produced by a target and consumed by util live in `util/`
(e.g. `util/timestamp.hpp`).

## Target strata: drivers by role, not a HAL

Pins, timers, PWM, clocks stay files of their target, each with its
own peculiarities stated in the open. brio does not build a HAL and
never a multi-platform one: the common factor between targets is HOW a
driver is made and WHAT it produces upward, not what the peripheral is.

- **A driver is a peripheral in a ROLE.** Types are named for what
  they do (`Ticker` = timebase, `TcaPwm` = waveform, a future
  `TcbCapture` = measurement); the peripheral appears in the type only
  where the app must pick the instance (`TcaPwm<0, 'C'>`). A timer
  used in a role belongs to that role entirely; "the same timer, half
  in another role" is another type with another name, not a flag.
- **What is common lives at the role level and is small**: a timebase
  is the Platform's `now()`/`ticks_per_second`; a PWM channel is a
  `PwmChannel` concept (`max` + `duty()`, raw counts, no frequency, no
  polarity - those belong to the timer instance and the target); a
  measurement is an EVENT posted by value with its timestamp. Register
  layouts, modes, clock trees, prescaler sets, event-system wiring are
  never unified: the target speaks its native vocabulary.
- **The brio properties every target driver has**: monostate type
  chosen at compile time, existence and routing checked with
  `static_assert(false)` in the instantiated branch, one `init()` that
  owns the WHOLE configuration of the role (no registers touched by
  halves around the app), ISR handler bodies exposed for the app to
  bind, the model explained in the header comment.
- **No driver allocates a resource the app did not name.** The kernel
  tick runs on the RTC/PIT precisely so that every general-purpose
  timer stays available to the app; a framework that silently takes a
  timer for `millis()` is what brio refuses to be.
- **PWM is an actuator, not a bus.** Continuous state, set and forget,
  synchronous, no completion, no contention worth an arbiter: rank of
  `Pin`, called from handlers. Fades and sequences are AOs in `util/`
  above the channel, never inside the driver.
- **The clock is a type with two regimes** (full model:
  [clock.md](clock.md)). Compile-time: `Clock<...>`
  owns the tree, `Clock::hz` is the ONE truth the same target's drivers
  derive their divisors from (no vendor `F_CPU`-style macro exists in
  the build: a second truth is unflagged, and vendor headers that need
  it stop compiling on purpose); "always at maximum" is not a rule, a
  low rate is a legitimate choice. Runtime (only when an application needs it): a
  change is a synchronous compile-time fan-out to every clocked driver
  (`Users::rebase(hz)`, applied to all before the next byte - publish
  semantics, fold mechanics, never a queued event that would arrive
  late) coordinated with the bus AOs so nothing changes mid-transfer;
  the RTC-based tick does not move. The kernel never knows the clock.
- **Clock and delay, concretely.** `Clock` is templated on the
  PARAMETERS of the tree (source, multiplier, dividers), never on a
  list of allowed frequencies: `hz` (one per clock domain where the
  target has several) is constexpr ARITHMETIC on those parameters,
  with `static_assert`s carrying the datasheet limits (VCO range, max
  system clock, flash wait states) - the CubeMX-style calculation done
  by the compiler, with errors instead of red boxes. The complexity of
  a rich tree stays inside that target's `clock.hpp`. `Clock::is_static`
  says whether `hz` is a compile-time constant or a runtime value.
  `delay_us<Clock>(us)` ("at least us microseconds") is a per-target
  driver of the short-wait role, for hardware setup times inside
  drivers - never a substitute for time events: a cycle-calibrated loop
  where the core is deterministic (AVR), a hardware counter (DWT
  CYCCNT, SysTick, a free timer, mcycle) where pipelines, prefetch and
  wait states make cycle counting a lottery. Same name and semantics,
  no shared algorithm.
- **Every addition to the AVR stratum is designed with the other
  targets in mind.** Before a new avrdx/ type or a change to one, ask
  what its shape would be on a Cortex-M0+ with a rich clock tree, on
  a RISC-V core, on a chip with per-pin alternate functions instead of
  per-timer routing: the answer decides what goes above the concept
  boundary (little) and what stays in the target file (most). Nothing
  written for AVR may make the second target harder than it already is.
- **Board facts vs device facts.** Which timer reaches which port,
  which USART sits on which pins per route, are facts of the DEVICE
  (today tables inside `pwm.hpp`, `uart.hpp` - the seed of a per-family
  header). Which timer drives which LEDs, which pins are buttons, which
  vectors bind to which driver bodies, are facts of the BOARD: they
  belong in a per-board unit that can also list resource claims and
  reject a double use at compile time. Both are built on the second
  target, when there is something to compare.

## Style rulings

- No `Ao` suffix on active-object class names: in an AO framework
  every service is an active object, so the suffix is noise - name
  the class for WHAT it is (`SpiBus`, `SerialPort`, `Blinker`). The
  term stays where it is information: the `ActiveObject` concept and
  the prose.
- Private members: trailing underscore (`head_`), not `m_`.
- Queues speak push/pop.
- No `*_from_isr` API doubling: one always-safe operation, revisit
  only with measurements (see [ring.md](ring.md)).
- No redundant `inline` on in-class definitions.
- `std::optional` returns instead of bool + out-parameter.
- Header comments explain the concurrency model and the WHY of each
  tradeoff - that comment IS the API documentation.
- **Apps never touch registers.** `PORTx.`, `VPORTx.`, peripheral
  structs and vendor bit masks appear only inside the target stratum's
  drivers; an app that needs an operation the driver lacks adds it to
  the driver (`Pin::pullup`, `PinSet::read`), never works around it.
  The only vendor glue an app may contain is the ISR vector binding.

## ISR binding pattern

Drivers expose the handler BODY (`rxc()`, `dre()`, `pit()`, `isr()`);
the app binds the vector (`ISR(USART2_RXC_vect) { Serial::rxc(); }`).
Vector names are irreducibly target-specific glue and live in the app,
never in portable code. The bodies are `[[gnu::always_inline]]`: an
ISR body has exactly one call site by construction, so inlining costs
no flash, and with the body visible the compiler saves only the
registers actually used (measured: 16 pushes -> 8/9 per ISR).
