# The first program, read in stages: blink

`blink` is the maintained teaching example of brio: two active objects
and a blinking LED. This page reads its source for the STM32G0 family,
[stm32g0/src/apps/blink.cpp](../../stm32g0/src/apps/blink.cpp), one
mechanism at a time, in the order the
[introduction](introduction.md) presented them. Every code block below
is an excerpt of that file, not a program of its own; the complete,
compilable source is the file itself. The same application exists for
the AVR, SAM C21, STM32F4 and CH32V00x projects under the same name;
only the few lines marked "target glue" differ.

The notation that may be new to a C programmer (angle brackets, `using`,
`static inline`, lambdas, `match`) is explained the first time it
appears, with a link to [the C++ in blink](cpp-notation.md) for the
longer version.

## What the program does

The board's user LED changes state at a steady rate. Every three
seconds the rate changes: half a second per toggle, then a quarter of a
second, then a tenth, then half a second again. Nothing else happens;
the program prints nothing and reads no input. Between two events the
processor sleeps.

The board this route documents is the ST Nucleo-G0B1RE
([its page](../boards/nucleo-g0b1re.md)): the LED is LD4 on pin PA5,
nothing has to be wired, and the board's own ST-LINK is the programmer.
The same source builds for the Nucleo-G071RB (same LED, same pin) and
the Nucleo-G031K8 (LD3 on PC6); the pin is chosen by the part, as the
"target glue" stage shows.

Verification status: `blink` is one of the applications the STM32G0
project builds on every preset and the bench flashes
([docs/stm32g0/README.md](../stm32g0/README.md), "The Nucleo-G071RB").
Its compilation was not repeated while this page was written; the local
handoff in the pull request that introduced this page lists it.

## Two active objects

The application has two components:

- `Blinker` owns the LED. Its own periodic timer sends it a `Toggle`
  event; on each one it flips the LED. It also accepts a `SetPeriod`
  command that changes the timer's period.
- `Supervisor` owns a slower periodic timer. Every three seconds it
  picks the next period from a table of three and sends `SetPeriod` to
  the `Blinker`.

The two share no variable. The only thing that passes between them is
the `SetPeriod` event.

## Stage 1: the events

```cpp
// excerpt: the three event types
struct Toggle {};                      // Blinker's own heartbeat
struct SetPeriod { uint16_t ticks; };  // command: change the blink period
struct Cycle {};                       // Supervisor's own heartbeat
```

An event is a plain struct. `Toggle` and `Cycle` carry no data: their
arrival is the whole message. `SetPeriod` carries the new period, in
kernel ticks (one millisecond each on this target; the "Time" stage
explains the unit). These are ordinary C structs; the empty ones are
legal in C++ and occupy no useful space inside the queue.

## Stage 2: the Blinker and its queue

```cpp
// excerpt: the Blinker
struct Blinker : brio::Fsm<Blinker, Toggle, SetPeriod> {
    static inline brio::EventQueue<Event, 4, P> queue;
    static inline brio::TimeEvent<P, Blinker, Toggle> heartbeat{Toggle{}};

    static void init() {
        Led::output();
        start(&running);
    }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                heartbeat.arm_every(brio::ticks_from_ms<P>(500));
                return handled();
            },
            [](Toggle) {
                Led::toggle();
                return handled();
            },
            [](SetPeriod p) {
                heartbeat.arm_every(p.ticks);  // restart with the new cadence
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }
};
```

Read the first line as a declaration of what this active object
accepts. `brio::Fsm` is a template: a type the compiler builds from the
choices written between the angle brackets. The first choice is the
active object's own type, `Blinker` (why it names itself is explained in
the "State" stage). The remaining choices, `Toggle` and `SetPeriod`, are
the events this object accepts. From them the template defines a nested
type, `Event`, which can hold one `Toggle` or one `SetPeriod` (or one of
two reserved events, `Entry` and `Exit`, met below). A `std::variant` is
the C++ form of a tagged union whose tag the language maintains; see
[variants and match](cpp-notation.md#stdvariant-one-value-one-of-several-types).

`static inline brio::EventQueue<Event, 4, P> queue;` is the object's
queue: a buffer of four waiting `Event`s, guarded by the platform `P`
(named at the top of the file). `static` means there is one queue for
the type, not one per instance; brio active objects have no instances
at all, every member is static, and the type itself is the object.
`inline` on a static member lets the header-only framework define the
storage here, in the class, with no separate `.cpp` file
([static members](cpp-notation.md#static-inline-members-one-per-type-no-cpp-file)).
The 4 is the programmer's sizing decision; the "Capacities" stage
returns to it.

`heartbeat` is a time event: a timer whose payload, `Toggle{}`, is
posted to `Blinker` when it expires. It is declared next to the object
that owns it and arms nothing yet.

`init()` is called once by the kernel before the first event is served.
It configures the LED pin as an output and starts the state machine in
the `running` state. `start` delivers the `Entry` event to `running`
immediately, so the first thing `running` does is arm the heartbeat at
500 ms.

`running` is the event handler, and the only state this object ever has.
It receives the event, decides by type what to do, and returns a
`Status`: `handled()` means done. `match` is the dispatch: it calls the
one lambda whose parameter type matches the event currently stored. A
lambda is a small function written where it is passed; `[](Toggle) {
... }` reads "a function taking a `Toggle` and doing this". The last
lambda, `[](auto)`, takes anything else; the variant dispatch must cover
every alternative, and this catch-all does. Without it the file does not
compile. More in
[lambdas](cpp-notation.md#lambdas-a-function-written-where-it-is-used)
and [match](cpp-notation.md#match-choose-the-lambda-by-the-event-type).

## Stage 3: one complete event path

Follow one `Toggle` from the hardware to the LED. Everything in this
list is code you can open.

1. The SysTick timer interrupt fires every millisecond. Its handler is
   one line of target glue in blink.cpp: `SysTick_Handler()` calls
   `brio::Ticker::tick()`, which increments the tick counter
   (`brio/cortexm/ticker.hpp`). Nothing of the application runs here.
2. The interrupt also wakes the processor, which was sleeping in the
   kernel's idle step. The kernel loop (`Tenuto::run()` in
   `brio/kernel/tenuto.hpp`) starts its next turn.
3. The loop's first action is `TimeEvents<P>::process()`
   (`brio/kernel/time_event.hpp`): it compares the current tick count
   with every armed deadline. When `heartbeat`'s deadline has passed,
   it re-arms it one period later and posts the payload: `Toggle{}` is
   copied into `Blinker::queue` by `post<Blinker>()`
   (`brio/kernel/post.hpp`). The copy happens with interrupts masked
   for a few instructions, in the loop, not in the interrupt handler.
4. The loop's second action is to scan the queues in priority order.
   `Blinker` is first and its queue holds one event, so the kernel
   takes it and calls `Blinker::dispatch(e)`, a function `Fsm` provides.
5. `dispatch` calls the current state's handler, `running(e)`. `match`
   selects the `Toggle` lambda, `Led::toggle()` flips PA5, and the
   lambda returns `handled()`.
6. The kernel releases the queue slot. The turn ends; the queues are
   empty, so the loop checks once more with interrupts masked and puts
   the processor to sleep until the next interrupt.

Steps 3 to 6 run with interrupts enabled, on the main stack, in the
loop. The interrupt handler's whole job was step 1.

## Stage 4: scheduling and priority

```cpp
// excerpt: the kernel is started with the two active objects, in priority order
brio::Tenuto<P, Blinker, Supervisor>::run();  // never returns
```

`Tenuto` is the kernel. The list after the platform `P` is the list of
active objects, highest priority first: `Blinker` before `Supervisor`.
The compiler generates the scan over this list; there is no table of
pointers consulted at run time, and an active object that is not in
this list never runs. `run()` calls every object's `init()` in list
order, then loops forever.

What priority means here, in one scenario. Suppose `Supervisor`'s
handler is running and the tick interrupt fires. The handler is
interrupted by the interrupt handler for the time it takes to count one
tick, then resumes and finishes. On the next turn the loop posts
`Toggle` to `Blinker` and scans from the top: `Blinker`'s queue is
served before `Supervisor`'s, whatever `Supervisor` still has waiting.
**Priority chooses the next event to serve. It never interrupts the
handler that is running.** A handler that takes long delays every
other active object by that long.

In `blink` both handlers take microseconds, so the question is
academic. It stops being academic the first time a handler is given a
loop that waits for a peripheral.

## Stage 5: state

`Blinker` and `Supervisor` each have one state, `running`, for their
whole life. The state mechanism is still there, and three parts of it
are visible in the excerpt:

- `start(&running)` in `init()` makes `running` the current state and
  delivers `Entry` to it. The current state is a pointer to the handler
  function; that pointer is the whole of the object's state variable.
- `Entry` is a reserved event the kernel delivers when a state is
  entered; `Exit` when it is left. Here `Entry` is where the timer is
  armed, so the timer is armed exactly once, when the state begins, and
  not in `init()` by hand. Neither is ever posted through a queue; the
  state machine delivers them directly.
- `handled()`, `unhandled()` and `transition(&next)` are the three
  things a handler can return. `blink` never transitions. A program
  that did would write `return transition(&other_state);`, and the
  kernel would deliver `Exit` to `running`, switch the pointer, and
  deliver `Entry` to `other_state`.

Why `Fsm<Blinker, ...>` names `Blinker` itself: the current-state
pointer is a static member of the `Fsm` type. If `Supervisor` also
accepted `Toggle` and `SetPeriod`, `Fsm<Toggle, SetPeriod>` without the
object's name would be one type with one state pointer shared by both
objects. Passing the object's own type as the first argument makes
`Fsm<Blinker, ...>` and `Fsm<Supervisor, ...>` two different types with
two different state pointers. This form is often called CRTP; here its
purpose is to keep each object's state separate, and the base does not
use it to call functions on `Blinker`
([the derived type as template argument](cpp-notation.md#the-derived-type-as-template-argument)).

## Stage 6: time

```cpp
// excerpt: arming the heartbeat
heartbeat.arm_every(brio::ticks_from_ms<P>(500));
```

Kernel time is counted in ticks. The tick rate is a constant of the
platform: 1000 per second on this target (SysTick at 1 kHz,
`brio/stm32g0/ticker.hpp`), 1024 on the AVR. `ticks_from_ms<P>(500)`
converts milliseconds to that platform's ticks, rounding up so that a
delay is never shorter than asked; with a constant argument the compiler
does the arithmetic and the result is a literal in the binary
(`brio/kernel/time.hpp`). `arm_every` arms the timer periodically: the
first `Toggle` 500 ticks from now, then one every 500 ticks.

Where the timer is checked, and where the user code runs, is the point
of the mechanism: the tick interrupt counts; the kernel loop compares
counts with deadlines at the start of every turn and posts the payload;
the handler runs when the event is served. A periodic timer is re-armed
from its previous deadline, not from the moment it was noticed, so the
period does not drift when the loop is late.

`SetPeriod` re-arms the same timer with a new period. Re-arming a
running timer restarts it: the next `Toggle` comes one new period after
the `SetPeriod` was handled.

## Stage 7: two active objects talking

```cpp
// excerpt: the Supervisor
struct Supervisor : brio::Fsm<Supervisor, Cycle> {
    static inline brio::EventQueue<Event, 2, P> queue;
    static inline brio::TimeEvent<P, Supervisor, Cycle> cadence{Cycle{}};

    static constexpr uint16_t periods[] = {
        static_cast<uint16_t>(brio::ticks_from_ms<P>(500)),
        static_cast<uint16_t>(brio::ticks_from_ms<P>(250)),
        static_cast<uint16_t>(brio::ticks_from_ms<P>(100)),
    };
    static inline uint8_t index = 0;

    static void init() { start(&running); }

    static Status running(const Event& e) {
        return brio::match(e,
            [](brio::Entry) {
                cadence.arm_every(brio::ticks_from_secs<P>(3));
                return handled();
            },
            [](Cycle) {
                index = static_cast<uint8_t>((index + 1) % 3);
                brio::post<Blinker>(SetPeriod{periods[index]});
                return handled();
            },
            [](auto) { return unhandled(); }
        );
    }
};
```

`Supervisor` has the same shape as `Blinker`: a queue (two slots here),
a periodic timer with a `Cycle` payload, one state. Its persistent data
is `index`, a `static inline` byte that survives between events, and
`periods`, a `constexpr` table computed by the compiler. `static_cast`
is C++'s spelled-out conversion, used here because `ticks_from_ms`
returns 32 bits and the event carries 16.

The line that connects the two objects is
`brio::post<Blinker>(SetPeriod{periods[index]});`. `post` copies the
event into `Blinker`'s queue and returns; `Supervisor`'s handler
continues and finishes. On the next turn the kernel finds `Blinker`'s
queue non-empty and serves the `SetPeriod`. `post` can be called from a
handler, as here, or from an interrupt handler: the same function, the
same brief masked copy, and it never waits. The compiler checks that
`Blinker` accepts `SetPeriod`: posting an event an object does not list
is a compile error at that line.

This is the first of the three delivery forms, a command addressed to
one object. `publish` (one event to a list of subscribers) and the
reply channel (a result returned to whoever asked) are described in
[kernel.md, "Delivery"](../design/kernel.md#7-delivery-kernelposthpp)
and used by the services in `brio/util/`.

## Capacities, overload, and how to see a failure

`Blinker`'s queue holds four waiting events, `Supervisor`'s two. In this
program the queues never hold more than one event: each timer posts one
event per period and the handler is served within the same turn. The
depth is still a decision the programmer makes, so it is worth knowing
what happens when it is wrong.

A post into a full queue drops the event and increments a saturating
counter in that queue. Nothing blocks and nothing panics. The counter
is `Blinker::queue.overflows()`; a debugger can read it by name, and a
program with a console can print it. A persistent overflow means a
producer outruns a consumer: a queue too small for a burst, or a
handler too slow for the rate. The kernel's own failure path for
errors a program cannot recover from is `panic()`, which writes a
record that survives a reset and halts
([kernel.md, "Failures: overflow and panic"](../design/kernel.md#10-failures-overflow-and-panic-kernelpanichpp));
`blink` never calls it.

Latency in this model is the time an event waits in its queue: at most
the longest handler in the program plus the handlers of
higher-priority events that were also waiting. `blink`'s handlers are a
few dozen instructions, so a `Toggle` is served within microseconds of
its tick.

## The target glue

Everything above runs unchanged on every target. What is this target's
is a handful of lines:

```cpp
// excerpt: what names the target
using P = brio::Stm32g0Platform<>;

using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

#if defined(STM32G031xx)
using Led = brio::Pin<'C', 6>;  // PC6 = LD3 on the Nucleo-32
#else
using Led = brio::Pin<'A', 5>;  // PA5 = LD4 on the Nucleo-64s
#endif

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }  // tick + idle wakeup

int main()
{
    SysClock::init();             // HSI16 -> PLL -> SYSCLK = 64 MHz
    brio::Ticker::init(clock);    // SysTick timebase (runs through WFI sleep)
    brio::enable_interrupts();

    brio::Tenuto<P, Blinker, Supervisor>::run();  // never returns
}
```

`using P = brio::Stm32g0Platform<>;` names the platform: the type that
gives the kernel its critical section, its idle instruction and its
tick counter. It is the one place the application says which chip it
runs on; everything else takes `P` as a parameter
([using aliases](cpp-notation.md#using-a-name-for-a-type)).

`using Led = brio::Pin<'A', 5>;` names pin PA5. The letter and the
number between the angle brackets are template arguments: fixed choices
the compiler uses to form this pin's type, so the port and the mask are
constants in the generated code. This line only names the pin;
`Led::output()` in `Blinker::init()` is what configures the hardware,
and `Led::toggle()` changes the level while the program runs. The
`#if defined(STM32G031xx)` is the C preprocessor you know: the STM32G0
project defines the part's macro, and the Nucleo-32 has its LED on
another pin.

`SysClock::init()` runs the chip at 64 MHz from the internal oscillator
through the PLL; `brio::Ticker::init(clock)` programs SysTick for 1000
interrupts per second from that rate; `brio::enable_interrupts()` is the
`sei()` of this target. `SysTick_Handler` is the vector name the
project's startup file uses; binding it to the ticker is the one piece
of vendor glue an application contains. Then the kernel runs and never
returns.

The three "init" calls happen in this order for a reason: the ticker
derives its reload value from the clock rate, so the clock is set up
first, and interrupts are enabled after both so that the first tick
finds a configured counter.

## Where to go from here

- Build it, flash it and watch the LED: [from the command
  line](command-line.md) or [from VS Code](vscode.md).
- The C++ used above, collected: [the C++ in blink](cpp-notation.md).
- The design behind every mechanism on this page, with its reasons and
  the exact contracts: [docs/design/kernel.md](../design/kernel.md). The
  active-object contract, the queue, delivery, the scheduler and time
  each have their section there, and the headers under `brio/kernel/`
  are the reference for every function.
- What `blink` does not show, each with its own design page: events
  that carry a pointer to a buffer and the rules on how long the buffer
  stays valid ([kernel.md, "Payloads"](../design/kernel.md#4-payloads-what-travels-inside-the-event-kernelborrowedhpp));
  the services over the kernel, starting with the serial port and
  `print` ([serial.md](../design/serial.md)) and the shared SPI and I2C
  buses ([spi-bus.md](../design/spi-bus.md),
  [i2c-bus.md](../design/i2c-bus.md)); sleeping deeper than the idle
  instruction ([power.md](../design/power.md)); and two cores running
  two kernels ([kernel.md, "Two cores"](../design/kernel.md#12-two-cores-two-kernels-and-a-bridge-utilinboxhpp)).
  There is no hands-on example for these in the learning route yet; the
  pages above and the test suites named in each target's documentation
  are where they are exercised.
