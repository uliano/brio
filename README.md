# brio

A small modern-C++ framework for bare-metal microcontrollers, built
around a cooperative **active-object kernel**: event queues, flat state
machines, time events, one loop, one stack, no heap, no virtual
functions, nothing resolved at run time that the compiler could resolve
first. Written in C++23 (gnu++23), header-only, in one flat namespace
`brio` ("con brio" - the musical marking for liveliness).

The kernel knows nothing about the silicon it runs on. Today it runs
on **AVR DA/DB** (an AVR128DB48 on the bench), on **SAM C21**
(Cortex-M0+, an ATSAMC21J18A) and on **STM32G0** (Cortex-M0+ from the
other vendor, an STM32G0B1RE on a Nucleo-64) - each of the two
Cortex-M0+ families compiled the kernel and its services unchanged,
which was the design's promise.

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

<sub>[open the diagram full size](https://raw.githubusercontent.com/uliano/brio/main/docs/design/architecture.svg) (zoomable in the browser)</sub>

`brio/` has four strata; the include prefix makes a file's
portability readable at a glance:

| Stratum | Contains | Depends on |
|---------|----------|------------|
| `kernel/` | queues, scheduler, FSM, delivery, time events, panic - pure logic | nothing of brio |
| `util/` | services on top of the kernel: `SerialPort`, `BusMaster` (SPI/I2C arbiter), `print`, `Ring`, line parsers | `kernel/` |
| `avrdx/` | everything that knows `avr/io.h`: clock, pins, UART, SPI, TWI, ticker, `AvrPlatform` | `kernel/`, `util/` |
| `armv6m/` | what ARM designed into every Cortex-M0/M0+ and both ARM families share: NVIC + PRIMASK guard, the SysTick ticker | `util/` (and the including family's device header) |
| `samc21/` | everything that knows `sam.h` (Cortex-M0+): clock tree, pins, SERCOM UART, `SamPlatform`; its NVIC and ticker are `armv6m/`'s | `kernel/`, `util/`, `armv6m/` |
| `stm32g0/` | everything that knows `stm32g0xx.h` (Cortex-M0+): RCC/PLL, GPIO, USART, `Stm32g0Platform`; its NVIC and ticker are `armv6m/`'s | `kernel/`, `util/`, `armv6m/` |
| `host/` | `HostPlatform`: the native test "target" (virtual clock, recording idle/break) | `kernel/` |

Targets are siblings, never meet in one binary, and are the only place
where hardware headers, ISR vector names and tick rates live. Nothing
above them uses `#ifdef` to tell targets apart: where behaviour must
differ, the target states a fact (`ticks_per_second`, `atomic_width`)
and generic code chooses with `if constexpr` or a concept.

A target is either **supported** - its peripheral chapters are
implemented and bench-verified, and everything `kernel/` and `util/`
claim holds there - or **in bring-up**, where the stratum exists, part
of it is proven on silicon, and the rest is still to be validated. This
table is the one place that question is answered: no source file carries
its own list of the targets it was tried on. Where a contract's
realizations differ - and where they are one interface - is the
REALIZATIONS TABLES, one per design page, indexed in
[docs/design/overview.md](docs/design/overview.md) ("One interface
where it can, its exceptions where a reader looks").

| Target | State | Bench silicon | Notes |
|--------|-------|---------------|-------|
| AVR DA/DB (`avrdx/`) | supported | AVR128DB48 | avr-gcc 16.2, see [docs/avrdx/README.md](docs/avrdx/README.md) |
| SAM C21 (`samc21/`) | supported | ATSAMC21J18A | arm-none-eabi-gcc 16.2, SysTick tick at 1000 Hz against the AVR's 1024 - the kernel tick's opacity, exercised for real; see [docs/samc21/README.md](docs/samc21/README.md) |
| STM32G0 (`stm32g0/`) | supported | STM32G0B1RE, STM32G071RB, STM32G031K8 | arm-none-eabi-gcc 16.2, HSI16 x PLL at 64 MHz, the third clock model (shared bus prescalers + per-peripheral enables) and a tickless timebase option; see [docs/stm32g0/README.md](docs/stm32g0/README.md) |
| host (`host/`) | supported | - | doctest suites, `cd test && ctest --preset host`, see [docs/host/README.md](docs/host/README.md) |

## Building and testing

The framework in `brio/` is header-only, included directly. The
builds are four sibling CMake projects, one per toolchain, all peers
(the repo root is not a CMake project): `avrdx/`, `samc21/` and
`stm32g0/` each auto-discover one `main()` per `src/apps/<app>.cpp` at configure time
- an app may pin build options such as its console baud with
`// build: monitor_speed = 115200` header lines - and `test/` holds
the host unit tests (a configure has exactly one compiler).

```bash
(cd test    && ctest --preset host)                                       # host tests: kernel, queues, FSM, time events, buses, ring...
(cd avrdx   && cmake --build --preset avr128db48-release --target <app>)  # build one AVR app (release, -Os)
(cd avrdx   && cmake --build --preset avr128db48-release --target <app>-upload)   # flash it over UPDI
(cd samc21  && cmake --build --preset samc21j-release --target <app>)     # build one SAM app
(cd samc21  && cmake --build --preset samc21j-release --target <app>-upload)      # flash it over SWD
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>)  # build one STM32G0 app
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>-upload)   # flash it over the ST-LINK
```

With more than one board on the desk the bench has one command,
`bin/brio` (put `bin/` on the PATH): `brio list` shows the boards the
manifest knows and which are plugged in, `brio flash <board> <app>`
builds for that board's type and flashes it whatever its probe,
`brio run <board> <letter>` drives a test suite's console and judges
its summary, `brio check <stratum>` runs the family compile fixtures
and `brio prose` the prose net. `brio --help` lists the rest.

Everything target-specific - toolchain, board, probe, debugger, its
quirks - is documented per target in each target's folder under [docs/](docs/).
The apps are the framework's test bench: disposable by design, they
document themselves in their own header comment (`brio apps` lists
them), and what a suite needs wired is in that comment. The boards and
the probes brio is tested with are [docs/boards/](docs/boards/) and
[docs/probes/](docs/probes/).

## Status

The kernel and the services above it compile unchanged on every
supported target and are bench-tested on all three families: the kernel
loop and time events, a serial console over `SerialPort`, an arbitrated
SPI bus (a display and a touch controller sharing one) and an
arbitrated I2C bus (a DAC written and read back, an ADC measured
through SPI), the power model with its sleep sites, the flash storage
classes, and the analog and metering services. Each target's own
chapters are covered by the reference suites its documentation names.

brio is still revised freely: nothing below the kernel contract is
promised stable, and where a limitation can be removed by rewriting
what sits below, the rewrite wins (the governing rule in
[overview.md](docs/design/overview.md)). What has settled is measured
rather than declared: the kernel and the services above it reached each
new family unchanged, and the contracts that survived three
realizations - the AO contract, the bus vocabularies, the power model,
the storage classes, the analog and metering services, the clock model -
are the ones a program can lean on. Clean-room with respect to QP: the
concepts come from Samek's book, never the QP source.
