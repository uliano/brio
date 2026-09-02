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
other vendor, an STM32G0B1RE on a Nucleo-64) - the second and the
third architecture each compiled the kernel and its services
unchanged, which was the design's promise.

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
| `samc/` | everything that knows `sam.h` (Cortex-M0+): clock tree, pins, SERCOM UART, SysTick ticker, `SamPlatform` | `kernel/`, `util/` |
| `stm32g0/` | everything that knows `stm32g0xx.h` (Cortex-M0+): RCC/PLL, GPIO, USART, SysTick ticker, `Stm32Platform` | `kernel/`, `util/` |
| `host/` | `HostPlatform`: the native test "target" (virtual clock, recording idle/break) | `kernel/` |

Targets are siblings, never meet in one binary, and are the only place
where hardware headers, ISR vector names and tick rates live. Nothing
above them uses `#ifdef` to tell targets apart: where behaviour must
differ, the target states a fact (`ticks_per_second`, `atomic_width`)
and generic code chooses with `if constexpr` or a concept.

| Target | State | Notes |
|--------|-------|-------|
| AVR DA/DB (`avrdx/`) | on the bench | AVR128DB48, avr-gcc 16.2, see [docs/avrdx/README.md](docs/avrdx/README.md) |
| SAM C21 (`samc/`) | on the bench | ATSAMC21J18A, arm-none-eabi-gcc 16.2, SysTick tick at 1000 Hz (vs the AVR's 1024: the kernel tick's opacity, exercised for real), see [docs/samc/README.md](docs/samc/README.md) |
| STM32G0 (`stm32g0/`) | on the bench, bring-up | STM32G0B1RE (Nucleo-G0B1RE), arm-none-eabi-gcc 16.2, HSI16 x PLL at 64 MHz, the third clock model (shared bus prescalers + per-peripheral enables), see [docs/stm32g0/README.md](docs/stm32g0/README.md) |
| host (`host/`) | in use | doctest suites, `cd test && ctest --preset host`, see [docs/host/README.md](docs/host/README.md) |

## Building and testing

The framework in `brio/` is header-only, included directly. The
builds are four sibling CMake projects, one per toolchain, all peers
(the repo root is not a CMake project): `avrdx/`, `samc/` and
`stm32g0/` each auto-discover one `main()` per `src/apps/<app>.cpp` at configure time
- an app may pin build options such as its console baud with
`// build: monitor_speed = 115200` header lines - and `test/` holds
the host unit tests (a configure has exactly one compiler).

```bash
(cd test  && ctest --preset host)                                       # host tests: kernel, queues, FSM, time events, buses, ring...
(cd avrdx && cmake --build --preset avr128db48-release --target <app>)  # build one AVR app (release, -Os)
(cd avrdx && cmake --build --preset avr128db48-release --target <app>-upload)   # flash it over UPDI
(cd samc  && cmake --build --preset samc21j-release --target <app>)     # build one SAM app
(cd samc  && cmake --build --preset samc21j-release --target <app>-upload)      # flash it over SWD
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>)  # build one STM32G0 app
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>-upload)   # flash it over the ST-LINK
```

Everything target-specific - toolchain, board, probe, debugger, its
quirks - is documented per target in each target's folder under [docs/](docs/).
The apps are the framework's test bench: disposable by design, they
document themselves in their own header comment, and their current
wiring lives in [docs/bench.md](docs/bench.md).

## Status

Bench-tested on the AVR128DB48: kernel loop, time events, serial
console over `SerialPort`, arbitrated SPI (display + touch on one bus)
and I2C (DAC written and read back, ADC measured through SPI).
Bench-tested on the ATSAMC21J18A: the same kernel and the same
services, recompiled untouched - blink under time events and a full
serial console at 115200, byte-exact on the wire. Bench-tested on the
STM32G0B1RE: the same two apps, recompiled untouched again, on the
third clock model and the third crt. Under continuous,
deliberately radical revision: nothing below the kernel contract is
considered done (see the governing rule in
[overview.md](docs/design/overview.md)). Clean-room with respect to
QP: the concepts come from Samek's book, never the QP source.
