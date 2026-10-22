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

To see the program run before reading it, build and flash it unchanged
from [the command line](command-line.md) or
[from VS Code](vscode.md), then come back here with the LED blinking.

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
project builds on every preset, and the bench has flashed it on the
Nucleo-G071RB ([docs/stm32g0/README.md](../stm32g0/README.md), "The
Nucleo-G071RB"); whether this revision of the file has run on the
Nucleo-G0B1RE is not on record, as [the command-line
page](command-line.md) states step by step.
The excerpts below are copied from the file as it stands in the same
revision as this page; where the two differ, the file is the authority.

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
explains the unit). These are ordinary C structs, except that C++
allows an empty one. A queue slot holds one `Event`, the variant built
from these types (next stage): it stores the largest alternative, a
tag saying which alternative is live, and whatever padding the
compiler's alignment rules add. For `Blinker` with the compiler the
STM32G0 project uses (arm-none-eabi-gcc 16.2, Cortex-M0+),
`sizeof(SetPeriod)` is 2 and `sizeof(Event)` is 4. The empty events
add nothing here, but that is a fact about this variant, not a formula
for others; when the size matters, read `sizeof(Event)` in the program
itself.

`Toggle{}` is how C++ writes a value of type `Toggle` with nothing in
it, and `SetPeriod{250}` a `SetPeriod` whose field is 250: braces after
a type name build a value of that type, field by field in order, the
way a C99 compound literal `(SetPeriod){250}` does
([brace initialization](cpp-notation.md#braces-after-a-type-building-a-value)).

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
accepts. `brio::Fsm` (for finite state machine) is a template: a type
the compiler builds from the choices written between the angle
brackets. The first choice is the active object's own type, `Blinker`
(why it names itself is explained in the "State" stage). The remaining
choices, `Toggle` and `SetPeriod`, are the events this object accepts.
From them the template defines a nested type, `Event`, which can hold
one `Toggle` or one `SetPeriod` (or one of two reserved events, `Entry`
and `Exit`, met below). A `std::variant` is the C++ form of a tagged
union whose tag the language maintains; see
[variants and match](cpp-notation.md#stdvariant-one-value-one-of-several-types).
Writing `: brio::Fsm<...>` after the struct's name gives `Blinker` every
member `Fsm` defines: `Event`, `Status` (the type a handler returns),
`start`, `handled`, `unhandled`, `transition` and `dispatch`
([inheritance](cpp-notation.md#structs-with-functions-inheritance)).

`static inline brio::EventQueue<Event, 4, P> queue;` is the object's
queue: a buffer in which up to four `Event`s wait while one is being
handled. `P` is the platform, named at the top of the file: the type
that supplies the interrupt masking the queue uses while an event is
copied in. `static` means there is one queue for the type, not one per
instance; brio active objects have no instances at all, every member is
static, and the type itself is the object. `inline` on a static member
lets the header-only framework define the storage here, in the class,
with no separate `.cpp` file
([static members](cpp-notation.md#static-inline-members-one-per-type-no-cpp-file)).
The 4 is the programmer's sizing decision; the "Capacities" stage
returns to it.

`heartbeat` is a time event: a timer whose payload, `Toggle{}`, is
posted to `Blinker` when it expires. It is declared next to the object
that owns it and arms nothing yet. The braces in `heartbeat{Toggle{}}`
initialize the timer with its payload, as `SetPeriod{250}` builds an
event.

`running` takes `const Event& e`. The `&` in a parameter declaration
does not mean "address of": it declares a reference, a name for the
caller's object without a copy, used like the object itself (`e`, not
`*e`). `const` says the handler only reads it. It is the C++ spelling of
"pass by const pointer, but use it as a value"
([references](cpp-notation.md#const-event-e-a-reference-parameter)).

`init()` is called once by the kernel before the first event is served.
It configures the LED pin as an output and starts the state machine in
the `running` state. `start` delivers the `Entry` event to `running`
immediately, so the first thing `running` does is arm the heartbeat at
500 ms. In `start(&running)` the `&` is the C one, address-of: the
argument is a pointer to the handler function, exactly as in C, and the
current state is that pointer.

`running` is the event handler, and the only state this object ever has.
It receives the event, decides by type what to do, and returns a
`Status`: `handled()` means done; `unhandled()`, returned by the
catch-all, means "not for this state", and in the flat model the state
machine ignores it (nothing is logged and nothing panics; it is the hook
a hierarchical extension would use to ask a parent state). `Entry` and
`Exit` are brio's own types, hence the `brio::` prefix, where `Event`
and `Status` are `Blinker`'s inherited members. `match` is the dispatch: it calls the
one lambda whose parameter type matches the event currently stored. A
lambda is a small function written where it is passed; `[](Toggle) {
... }` reads "a function taking a `Toggle` and doing this". The last
lambda, `[](auto)`, takes anything else; the variant dispatch must cover
every alternative, and this catch-all does. Without it the file does not
compile, because `Entry` and `Exit` are alternatives too and `running`
names only `Entry`. The other side of the catch-all: a listed event
that a handler forgets to name goes to it and is ignored, with no
error at compile time or at run time. The compiler's check is that
every alternative reaches some lambda, not that each has one of its
own. More in
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
   for a few instructions: the mask is against the other producers,
   since a post from the loop and a post from an interrupt handler
   share the producer's cursor. Note where this `post` runs: in the loop,
   because a timer's expiry is detected by the loop. An interrupt
   handler that detects something itself (a byte received, a pin edge)
   calls the same `post` from the interrupt handler: the event lands in
   the queue before the handler returns, and the loop's next scan (step
   4) serves it. `blink` has no such interrupt. The taking side needs
   no mask of its own: the producers and the consumer write different
   one-byte cursors, and a byte is one uninterruptible access on every
   core brio runs on, so a post landing in the middle of a take leaves
   both consistent (`brio/kernel/event_queue.hpp`, "The consumer's
   side, which never masks").
4. The loop's second action is to scan the queues in priority order.
   `Blinker` is first and its queue holds one event, so the kernel
   takes it and calls `Blinker::dispatch(e)`, a function `Fsm` provides.
5. `dispatch` calls the current state's handler, `running(e)`. `match`
   selects the `Toggle` lambda, `Led::toggle()` flips PA5, and the
   lambda returns `handled()`.
6. The kernel releases the queue slot. The turn ends; the queues are
   empty, so the loop checks once more with interrupts masked and puts
   the processor to sleep until the next interrupt, through the
   platform's `idle()` (`brio/stm32g0/platform.hpp`): on this core the
   wait-for-interrupt instruction ends on a pending interrupt even
   while interrupts are masked, and the platform unmasks right after
   it, so a tick that lands between the check and the sleep is not
   lost.

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
handler is running and the tick interrupt fires, and suppose this tick
is the one `heartbeat`'s deadline falls on (most ticks are not: the
loop posts a `Toggle` once per period, not once per tick). The handler
is interrupted by the interrupt handler for the time it takes to count
one tick, then resumes and finishes. On the next turn the loop posts
`Toggle` to `Blinker` and scans from the top: `Blinker`'s queue is
served before `Supervisor`'s, whatever `Supervisor` still has waiting.
**Priority chooses the next event to serve. It never interrupts the
handler that is running.** A handler that takes long delays every
other active object by that long, and that is the only promise the
kernel makes about response time. How long an event waits also depends
on the interrupt handlers that run, on the timer check at the start of
the turn, and on the events ahead of it: older ones in the same queue,
and any in a higher-priority queue, including ones posted while it
waits. An object whose higher-priority peers always have something
waiting is never served, and nothing in the kernel prevents that. One
interrupt handler never interrupts another on any brio platform, so
the tick handler above runs whole, whatever else is armed.

In `blink` both handlers take microseconds and no queue ever holds
more than one event, so the question is academic. It stops being
academic the first time a handler is given a loop that waits for a
peripheral, or a producer posts faster than its consumer is served.

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
converts milliseconds to that platform's ticks, rounding up to whole
ticks so that the count is never shortened by truncation; with a
constant argument the compiler does the arithmetic and the result is a
literal in the binary (`brio/kernel/time.hpp`). `arm_every` arms the
timer periodically: the first `Toggle` 500 ticks from now, then one
every 500 ticks. Its sibling `arm` posts the payload once, after a
delay, and is the one a program uses in place of a one-shot delay loop.

A count of ticks is not a promise of physical time. The timer is
scheduled against the platform's tick counter, which changes once per
tick interval (a millisecond on this target), and arming can happen
anywhere between two changes; so a delay of N ticks matures between
N - 1 and N tick intervals of wall time later, and the event is
handled when the loop gets to it after that. For a 500 ms heartbeat the
difference is invisible; for a one-tick delay it means "almost at
once". The exact rules (the arming phase, the wrap of the 32-bit
counter, the catch-up when the loop falls behind a periodic timer) are
in the opening comment of `brio/kernel/time_event.hpp`.

Where the timer is checked, and where the user code runs, is the point
of the mechanism: the tick interrupt counts; the kernel loop compares
counts with deadlines at the start of every turn and posts the payload;
the handler runs when the event is served. When a periodic timer
matures, the kernel sets its next deadline one period after the
previous deadline, not one period after the moment it was noticed, so
the period does not drift when the loop is late.

`SetPeriod` calls `arm_every` again on the same timer with a new
period. That is a different thing from the kernel's own re-scheduling
above: a program's `arm` or `arm_every` on a running timer restarts it
from now, so the next `Toggle` comes one new period after the
`SetPeriod` was handled, whatever was left of the old period.

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

`Blinker`'s queue holds four waiting events, `Supervisor`'s two. The
depth counts the events that can wait while one is being handled: the
event being dispatched keeps its slot until its handler returns, and
the queue has one slot more than its depth for that reason, so four
posts can land during a `Blinker` handler and the fifth is the one
dropped; while no handler of `Blinker` is running the spare slot takes
a waiting event too, and five fit (`brio/kernel/event_queue.hpp` states
the rule). Size by the depth: a sender cannot know whether the
receiver's handler is running, so the fifth slot is not to be counted
on. In this program the queues never hold more than one event: each
timer posts one event per period and the handler is served within the
same turn. The depth is still a decision the programmer makes, so it is
worth knowing what happens when it is wrong.

A post into a full queue drops the event and increments a saturating
counter in that queue. Nothing blocks and nothing panics, and `post`
returns nothing: the sender is not told, and the drop is visible only
in the counter. The counter is `Blinker::queue.overflows()`; a
debugger can read it by name, and a program with a console can print
it. A persistent overflow means a
producer outruns a consumer: a queue too small for a burst, or a
handler too slow for the rate. The kernel's own failure path for
errors a program cannot recover from is `panic()`, which writes a
record that survives a reset and halts
([kernel.md, "Failures: overflow and panic"](../design/kernel.md#10-failures-overflow-and-panic-kernelpanichpp));
`blink` never calls it.

Latency in this model is the time an event waits in its queue, and the
kernel bounds only one part of it: the handler running when the event
arrives finishes first. The rest is the interrupt handlers that run in
the meantime, the timer check at the start of the turn, and every event
served before it, older ones in the same queue and any in a
higher-priority queue, those posted while it waits included. There is
no formula; the bound is whatever the program's own handlers and
traffic make it. In `blink` the handlers are a few dozen instructions
and no queue ever holds more than one event, so a `Toggle` is expected
to be served within microseconds of its tick; that is read from the
code, not measured.

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
first, and the ticker's own contract is to be initialized after the
clock and before interrupts are enabled (`brio/cortexm/ticker.hpp`,
`init()`), so that the first tick interrupt finds a programmed counter
and a handler bound to it. `brio::Ticker::init(clock)` returns false,
and starts nothing, when the rate asked for does not fit SysTick's
24-bit reload at this clock; `blink` does not check the return because
1000 Hz at 64 MHz fits by a wide margin, and a program on an unusual
clock should.

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
