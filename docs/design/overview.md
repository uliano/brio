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

## ISR binding pattern

Drivers expose the handler BODY (`rxc()`, `dre()`, `pit()`, `isr()`);
the app binds the vector (`ISR(USART2_RXC_vect) { Serial::rxc(); }`).
Vector names are irreducibly target-specific glue and live in the app,
never in portable code. The bodies are `[[gnu::always_inline]]`: an
ISR body has exactly one call site by construction, so inlining costs
no flash, and with the body visible the compiler saves only the
registers actually used (measured: 16 pushes -> 8/9 per ISR).
