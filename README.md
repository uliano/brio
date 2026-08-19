# brio

A small modern-C++ framework for bare-metal microcontrollers, built
around a cooperative **active-object kernel**: event queues, flat state
machines, time events, one loop, one stack, no heap, no virtual
functions, nothing resolved at run time that the compiler could resolve
first. Written in C++23 (gnu++23), header-only, in one flat namespace
`brio` ("con brio" - the musical marking for liveliness).

The kernel knows nothing about the silicon it runs on. Today it runs on
**AVR DA/DB** (the bench target, an AVR128DB48); the design keeps the
door open, by construction, to whatever comes next.

## What an application looks like

```cpp
using P = brio::AvrPlatform;             // the one place the target is named

struct Toggle {};                        // events are small plain structs
struct SetPeriod { uint16_t ticks; };

struct Blinker : brio::Fsm<Blinker, Toggle, SetPeriod> {
    static inline brio::EventQueue<Event, 4, P> queue;                 // its own queue
    static inline brio::TimeEvent<P, Blinker, Toggle> heartbeat{Toggle{}};

    static void init() { Led::output(); start(&running); }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) { heartbeat.arm_every(brio::ticks_from_ms<P>(500));
                              return handled(); },
            [](Toggle)      { Led::toggle(); return handled(); },
            [](SetPeriod p) { heartbeat.arm_every(p.ticks); return handled(); },
            [](auto)        { return unhandled(); });
    }
};

int main() {
    /* clock, timebase, sei() - target glue */
    brio::Kernel<P, Blinker, Supervisor>::run();   // priority = pack order
}
```

Somewhere else, `brio::post<Blinker>(SetPeriod{...})` changes the
cadence - from another active object, from a time event, or from an
ISR: same call, always safe, never blocking.

## The ideas

- **Active objects, run-to-completion.** An AO owns a queue and reacts
  to one event at a time; it never blocks, waits or polls. The kernel
  serves the highest-priority non-empty queue, one event per turn, and
  sleeps when nothing is pending. The only concurrency is ISR vs main
  loop, and an ISR may do exactly one kernel thing: post an event.
- **Everything is a type, resolved at compile time.** AOs, drivers and
  the kernel are monostate classes selected by type; priorities,
  subscriptions and reply channels are template parameters. No tables
  walked at run time; RAM is exactly what is declared.
- **Contracts are concepts.** What the kernel needs from an AO
  (`ActiveObject`), from the machine (`Platform`), what a text sink
  must offer (`ByteSink`) - stated as C++20 concepts, checked where a
  type is used, with errors that name the requirement.
- **Events are values, per-AO variants.** Each AO declares its own
  `std::variant` of small trivially-copyable structs; events are copied
  into queues (a few bytes, one brief critical section). Plain shared
  structs are the lingua franca between publisher and subscriber; no
  global signal enum. Handlers dispatch with `match(e, lambdas...)`.
- **Timers post events; failures leave a breadcrumb.** A time event
  posts to its owner AO in main context (never runs user code in an
  ISR); a full queue is a sizing mistake counted, never blocking;
  `panic()` writes a reset-surviving record before any LED blinks.

The full rationale, decision by decision, is in
[docs/design/](docs/design/) - start with
[overview.md](docs/design/overview.md) and
[kernel.md](docs/design/kernel.md).

## Layering and portability

![brio strata](docs/design/architecture.svg)

`lib/brio/src/` has four strata; the include prefix makes a file's
portability readable at a glance:

| Stratum | Contains | Depends on |
|---------|----------|------------|
| `kernel/` | queues, scheduler, FSM, delivery, time events, panic - pure logic | nothing of brio |
| `util/` | services on top of the kernel: `SerialPort`, `BusMaster` (SPI/I2C arbiter), `print`, `Ring`, line parsers | `kernel/` |
| `avrdx/` | everything that knows `avr/io.h`: clock, pins, UART, SPI, TWI, ticker, `AvrPlatform` | `kernel/`, `util/` |
| `host/` | `HostPlatform`: the native test "target" (virtual clock, recording idle/break) | `kernel/` |

Targets are siblings, never meet in one binary, and are the only place
where hardware headers, ISR vector names and tick rates live. Nothing
above them uses `#ifdef` to tell targets apart: where behaviour must
differ, the target states a fact (`ticks_per_second`, `atomic_width`)
and generic code chooses with `if constexpr` or a concept.

| Target | State | Notes |
|--------|-------|-------|
| AVR DA/DB (`avrdx/`) | on the bench | AVR128DB48, avr-gcc 16.2, see [docs/avrdx/README.md](docs/avrdx/README.md) |
| host (`host/`) | in use | doctest suites, `pio test -e native`, see [docs/host/README.md](docs/host/README.md) |
| STM32G0, ATSAMC/D, CH32V00x | candidates | SysTick timebases (1000 Hz): the reason the kernel tick is opaque |

## Building and testing

The repo is a PlatformIO project: the framework in `lib/brio/` (a
private library, auto-linked), one `main()` per `src/apps/<app>.cpp`
turned into a pair of envs (`<app>` release, `<app>-debug`) by
`tools/gen_apps.py` (an app may pin env options such as its console
baud with `// pio: monitor_speed = 115200` header lines), and host unit
tests in `test/`.

```bash
pio test -e native            # host tests: kernel, queues, FSM, time events, buses, ring...
pio run -e <app>              # build one app for the target (release, -Os)
pio run -e <app> -t upload    # flash it (target-specific probe: see docs/<target>/)
pio debug -e <app>-debug      # debug build + gdb session
```

Everything target-specific - toolchain, board, probe, debugger, its
quirks - is documented per target in each target's folder under [docs/](docs/).
The apps are the framework's test bench: disposable by design, they
document themselves in their own header comment, and their current
wiring lives in [docs/bench.md](docs/bench.md).

## Status

Bench-tested on the AVR128DB48: kernel loop, time events, serial
console over `SerialPort`, arbitrated SPI (display + touch on one bus)
and I2C (DAC written and read back, ADC measured through SPI). Under
continuous, deliberately radical revision: nothing below the kernel
contract is considered done (see the governing rule in
[overview.md](docs/design/overview.md)). Clean-room with respect to
QP: the concepts come from Samek's book, never the QP source.
