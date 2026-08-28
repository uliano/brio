# The active-object kernel

Headers: `brio/kernel/`. Everything in this stratum is pure
logic, templated on a `Platform` and never on a specific machine -
host-testable, no target includes; the reference index at the end
maps every entity to its header. Where a concrete number appears
below it is a worked example from one platform, marked as such, never
an assumption of the kernel.

How to read this: sections 1-2 give the model and the contract; 3-10
follow the life of an event (born, carried, queued, dispatched,
delivered, scheduled, timed, lost); 11 is what the machine must
provide; 12 is the index.
Paragraphs marked **C++ note** explain the C++17/20/23 idiom the code
relies on - brio is deliberately written in modern C++ and those
idioms are part of the design, not decoration.

## 1. The model in one page

An **active object** (AO) is a piece of logic that owns a queue of
events and reacts to them one at a time. It never blocks, never
waits, never polls: it is *given* an event by the kernel, runs a
short piece of code to completion (**run-to-completion**, RTC),
possibly posts events to other AOs, arms a timer, or changes state,
and returns. Between two dispatches it holds no stack: all its data
is static.

The **kernel** is a cooperative loop on the single main stack. Each
turn it fires matured time events, then serves ONE event from the
highest-priority non-empty queue, then starts again from the top. If
every queue is empty it sleeps until an interrupt. There is no
preemption between AOs: while one dispatch runs, no other AO runs.
The only concurrency in the system is **ISR versus main loop**, and
ISRs are allowed to do exactly one kernel thing: `post()` an event
(a few bytes copied inside a brief critical section).

This is the "QV" flavor of active objects (cooperative, one stack,
priority by scan order) described in Samek's book, written from
scratch (clean room: concepts only, never the QP source).

Everything is a **monostate**: AOs, drivers, queues' owners, the
kernel itself are classes with no instances and only static members,
selected by type. Priority, wiring and subscriptions are types and
template parameters, resolved at compile time. Nothing is looked up
at run time; the RAM footprint is exactly the sum of what is
declared.

**C++ note - monostate and `static inline`.** A monostate class is
one where every data member is `static inline` (C++17): the variable
is defined in the header, once per program, with no .cpp file and no
initialization-order problem. `Fsm() = delete;` on the constructor
makes "no instances" explicit. Choosing a monostate by *type* is what
lets `Kernel<P, Traffic, Buttons>` visit the AOs with a fold
expression instead of walking a table of pointers at run time.

## 2. The AO contract (`kernel/active_object.hpp`)

The kernel does not derive AOs from a base class (a virtual base would
force a system-wide event type and cost an indirect call per event).
It states what it needs as a **concept**, `ActiveObject`:

- a nested type `Event` - the AO's own event variant;
- a static member `queue` whose `pop()` yields `std::optional<Event>`
  and whose `empty()` yields `bool` (an `EventQueue`);
- static `init()` - called once by `Kernel::init_all()` in pack order,
  before the first event is served; an Fsm-based AO calls
  `start(&initial)` here;
- static `dispatch(const Event&)` - run ONE event to completion.

That is the **formal half**: names, signatures, return types, checked
by the compiler where an AO enters `Kernel<...>`. The **informal
half** is the set of rules the compiler cannot see and that the
kernel is written assuming: dispatch is RTC and is only ever called
from the loop, interrupts enabled, never re-entered; the AO's static
data therefore needs no locking against other AOs (only ISRs are
concurrent, and they touch nothing but the queue, through `post()`);
`init()` must leave the AO in a real state; the queue is really an
`EventQueue`. Wherever this document says "contract" it means both
halves.

`Fsm<Derived, Alts...>` (section 5) is *one way* to satisfy the
contract - it gives you `Event` and `dispatch` - not the contract
itself. An AO with a hand-written variant and a switch is a legal
citizen. That is why `active_object.hpp` includes nothing of
`fsm.hpp`, and why the `queue` member is always declared by the AO
itself: its depth is that AO's sizing decision.

**C++ note - concepts and `requires`.** A `concept` (C++20) is a
named compile-time predicate on types. `ActiveObject<Buttons>` is
just `true` or `false`; nothing happens until a template *applies*
it: `template <Platform P, ActiveObject... Aos> class Kernel` (the
short form: the concept name replaces `typename`), or a trailing
`requires ActiveObject<Ao>` clause, or a `static_assert`. When a
constrained template is instantiated with a type that fails, the
error names the concept and the requirement that failed - which is
the whole point over "duck typing" by plain templates, where the
error would be an unreadable failure deep inside the body. In brio
the concepts ARE the contracts between strata: `ActiveObject` and
`Platform` here, `ByteSink`/`ByteSource` for the drivers.

## 3. Events: value semantics, per-AO variant

An event is a small, trivially copyable struct - `struct Tick {}`,
`struct Pressed { uint8_t which; }`, `struct TempChanged { int16_t
c10; }`. Each AO declares its own **event type** as a `std::variant`
of the structs it accepts (`Fsm` builds it for you: `Event =
std::variant<Entry, Exit, Alts...>`). There is no global signal enum
and no system-wide event type: plain shared structs are the lingua
franca between publishers and subscribers, and two AOs that accept
the same struct simply both list it in their variant.

Events are **copied** into per-AO queues - no pools, no reference
counting, no shared ownership. The 8-byte size target is a
**guideline** for the event envelope, not a law: every slot of a
queue pays that AO's largest alternative, and the push copies it
with interrupts masked (on an 8-bit core at 24 MHz, ~1 us per 8
bytes). Per-AO deviations are legal with numbers in hand, and nobody
else pays for them (the queues are per-AO). Measured on one platform
(avr-gcc 16.2, -Os): `std::visit` on a 4-alternative variant compiles
to a plain switch, +30 bytes flash and identical RAM vs a hand-tagged
union.

Two alternatives are reserved and always first: `Entry` and `Exit`
(empty structs, so they do not enlarge the slots). They are
delivered synchronously by the state-machine machinery and are never
posted; `post()` refuses them at compile time.

**C++ note - `std::variant`, `std::visit`, `match`.**
`std::variant<A, B, C>` (C++17) is a type-safe tagged union: it holds
exactly one of A, B or C and remembers which. Its size is the largest
alternative plus one index byte - which is why the queue slot size is
"the AO's largest alternative". `std::visit(f, v)` calls `f` with
the alternative currently held; `f` must therefore be callable with
*every* alternative. Handlers in brio write

    return brio::match(e,
        [](Tick)  { return transition(&green); },
        [](Entry) { lamp_red(); return handled(); },
        [](auto)  { return unhandled(); });

subject first, then one lambda per case, a `[](auto)` catch-all for
the rest (variant dispatch is exhaustive: without it the code does
not compile) - the shape of `match` in Rust/ML and of the pattern
matching proposed for C++ (P2688, not adopted for C++26; `match` here
is a four-line library stand-in in `kernel/fsm.hpp`).
Underneath sits the classic C++17 building block, public as
`brio::Overloaded`:

    template <class... Ts> struct Overloaded : Ts... {
        using Ts::operator()...;
    };

it inherits from all the lambdas and pulls all their `operator()`
into one overload set, so `std::visit(Overloaded{...}, e)` dispatches
by overload resolution at compile time (class template argument
deduction + pack expansion of using-declarations). `match` forwards
to exactly that and compiles to the same switch (measured on two
bench apps: flash identical to spelling out the `std::visit` +
`Overloaded` dispatch by hand). `std::variant` needs its alternatives to
be complete types and, for the queue, trivially copyable -
`EventQueue` static-asserts this: an event may be copied by an ISR,
byte-wise, and no destructor will ever run for it.

## 4. Payloads: what travels inside the event (`kernel/borrowed.hpp`)

The event envelope is copied, so what the struct *contains* decides
the cost - and, for pointers, the rules. The whole payload rule fits
in three lines:

1. **Copy what is read once** - descriptors, commands, results,
   facts: by value. Size is a per-AO budget (section 3); deviations
   are legal with numbers in hand.
2. **Borrow only what must be shared** - storage that hardware or a
   producer writes into and that would be absurd to copy: by
   reference, and every borrow is one of exactly two leases:
   - `Lease::dispatch` - valid during the receiving dispatch only;
     correct by construction when the borrower PRECEDES the lender in
     the Kernel pack;
   - `Lease::reply` - valid until the borrower posts the agreed
     completion event; the reply IS the return of the loan.
3. **Published payloads travel by value** (or by const reference at
   most): N subscribers see the same struct in N sequential dispatches,
   a mutable loan would let each modify what the next one sees.

The case-by-case then reduces to one question per field: is this data
*read once by one receiver*, or *written by someone else while it is
in flight*? The first has one answer, the second has two.

**Why the distinction is safe under this kernel.** There is no
concurrency between AOs, so two AOs can never write the same buffer at
the same time; the only hazard of a loan is *aliasing in time* - the
lender reusing storage the borrower still holds. Both leases pin that
down: for `dispatch` the pack order guarantees the borrower runs
before the lender is dispatched again (and under a preemptive kernel
the same order makes the borrower preempt the lender right at the
post - the rule survives QK unchanged); for `reply` the boundaries are
two explicit events. At every instant exactly one party has the right
to write: ownership moves, it is never shared.

**How it is written down.** `Borrowed<T, Lease>` is a plain pointer
with the lease in its type: zero cost, trivially copyable, and the
contract becomes readable at the field (`struct LineReceived {
Borrowed<char, Lease::dispatch> line; }`). The lender of a `dispatch`
loan declares its borrowers - `using LendsTo = Subscribers<Sink>;` -
and `Kernel` static_asserts that each precedes it in the pack: the
scheduling contract of the serial stack is a compile-time fact, not a
comment. What C++ cannot do is stop a receiver from stashing the raw
pointer past its window; the planned debug-build addition is a lender
epoch inside `Borrowed` compared on access (stale loan -> panic on the
guilty instruction), built when a host test simulating preemption
needs it. `lend<Lease::reply>(buf)` is the maker that names a loan at
the call site, spelled like `reply_to<Ao, Payload>()` - the lease is
the explicit argument, the pointee type is deduced; a field left out
(or `{}`) is a null loan. Bus tx/rx/command buffers and the source
bytes of a nonvolatile write are `reply` loans and say so in their
field types; the engine that walks such a buffer calls `.get()` and
indexes the raw pointer, because a loan is a view and not a container.

**In use today.** `LineReceived` lends the line buffer for one
dispatch (mutable: in-place tokenization is the point). Bus requests
copy the ~14-16-byte descriptor by value (read once by the bus AO; the
request IS the arbitration token, its size is paid once in the bus
queue) and lend the data buffers until `BusDone` - the borrowed
surface is the structurally necessary minimum. Nothing is ever
chopped into several events: one event = one envelope, the cargo
stays put.

**Fan-out cost.** `publish()` is one `post()` per subscriber: N
subscribers = N copies of the envelope, in N queues, one critical
section each. Fine for small notifications; a large published payload
would be N times wrong, which is why "facts" are small structs.
**Replies** (section 7) are ordinary posted events too: the service
copies the small result struct into the requester's queue.

**One more discipline, stated for the future.** AOs share nothing but
events: an AO's own statics are safe because its dispatch is never
re-entered, but a global touched by two AOs outside events is safe
only because this kernel is cooperative - under a preemptive one it is
a one-way race. The rule costs nothing today and keeps the QK door
open.

## 5. Queues (`kernel/event_queue.hpp`)

One `EventQueue<E, depth, Platform>` per AO, **multi-producer** (any
ISR, any main-loop code, any timer) **single-consumer** (the
scheduler). The kernel assumes no atomic read-modify-write from the
machine (the smallest candidate cores have none), so the honest
primitive is a brief interrupts-off section - the platform's
`CriticalSection`: `push` enters it, copies the event, leaves; `pop`
does the same around the index update. There
are deliberately no `*_from_isr` twins: one always-safe API. Depth is
a per-AO template parameter sized on that AO's real burst; it may be
any number (no power-of-two rounding, no sacrificed slot: rounding a
depth of 5 to 8 would waste real RAM to speed up a wrap that is
already two instructions).

`Ring` (SPSC, see [ring.md](ring.md)) is NOT the event queue: it stays
at the BYTE level inside drivers - the ISR pushes bytes lock-free,
the driver AO condenses them into few events (bytes at high rate,
events at low rate).

**Overflow** is a sizing mistake, not a runtime condition. `push`
never blocks and never returns failure: a full queue drops the event
and bumps a **saturating per-queue counter** (a named static symbol,
readable from the debugger, nearly free in release). The reaction is
a compile-time knob at the kernel level - `count` (drop, keep
running: release default) or `panic` (debug builds, where
`break_here()` stops the debugger on the undersized queue). No
bool-returning post spreading untested error branches.

**C++ note - `std::optional` returns.** `pop()` returns
`std::optional<E>` (C++17): "an E, or nothing". The caller writes `if
(auto e = q.pop()) dispatch(*e);` and cannot forget to test, where a
`bool pop(E& out)` would leave a half-written out-parameter around. A
project style ruling: optional returns instead of bool + out-param.

## 6. State machines (`kernel/fsm.hpp`)

HSM-ready contract, flat implementation. `Fsm<Derived, Alts...>` is
the CRTP-monostate base an AO derives from; it gives the AO its
`Event` (variant with `Entry`, `Exit` prepended) and its `dispatch()`.

- A **state is a handler function** `Status handler(const Event&)`;
  the current state of the machine IS a pointer to it (one function
  pointer of RAM, one indirect call per dispatch).
- The handler returns `handled()`, `unhandled()`, or
  `transition(&next)`. `unhandled` today means "ignore"; tomorrow it
  means "ask the parent state" - it IS the hook that makes a
  hierarchical extension additive rather than breaking.
- A **transition** delivers `Exit` to the old state, switches,
  delivers `Entry` to the new one. Entry may itself return
  `transition()`: the machinery follows the chain (pass-through
  states for free). Exit's return is deliberately ignored: exit is an
  action, not a decision.
- `start(&initial)` arms the machine and delivers the first Entry, so
  no state ever runs half-initialized. The AO calls it from `init()`,
  which the kernel calls before the loop ("kernel-delivered init").
- Trivial AOs may legally keep a plain switch inside one everlasting
  state.

Full HSM (parent pointers, bubbling, LCA entry/exit chains) gets
built only when a real AO demands it; states-as-types (sml-style) is
rejected as foundation (heaviest machinery, hostile errors,
readability drops).

**C++ note - CRTP monostate.** `struct Buttons : brio::Fsm<Buttons,
Tick>`: the derived class passes *itself* as the first template
argument (Curiously Recurring Template Pattern). Here it is not used
to call back into the derived class; it exists so that the `static
inline Handler state_` inside `Fsm<Buttons, Tick>` is a *different*
variable from the one inside `Fsm<Traffic, Tick>` - two AOs with the
same alternatives get distinct machines. The state pointer type
`Status (*)(const Event&)` is a plain function pointer: handlers are
static functions, no `this`, no virtual table.

## 7. Delivery (`kernel/post.hpp`)

Three primitives, one rule each:

- `post<Ao>(ev)` - **commands are addressed**: the event goes to one
  named AO. Requires only that `Ao::Event` can be constructed from
  `ev`'s type; Entry/Exit are refused by `static_assert`. Safe from
  ISRs and from the loop alike, never blocks.
- `publish(Subscribers<A, B...>{}, ev)` - **facts are published**: one
  `post` per listed subscriber. The subscriber list is a type built
  where the publisher is declared, so a subscription that cannot be
  received (variant lacks the struct) fails to compile at that line.
- `ReplyTo<Payload>` - **replies return to sender**: a one-function-
  pointer capsule (trivially copyable) that travels
  INSIDE the request event, built at compile time by
  `reply_to<RequesterAo, Payload>()`. The service calls
  `req.reply.send(payload)` without knowing who asked; a
  default-constructed capsule is null and `send()` is a no-op, so
  fire-and-forget requests are free. A requester whose variant cannot
  hold `Payload` fails to compile at the `reply_to` site. This is the
  return channel of every bus AO, whose request queue doubles as the
  bus arbiter.

**C++ note - fold expressions and thunks.** `publish` is one line:
`(post<Aos>(e), ...);` - a C++17 fold over the parameter pack with
the comma operator, expanding to `post<A>(e), post<B>(e), ...` at
compile time; there is no loop and no table. `Kernel::step()` uses
the same device with `||` (`(try_one<Aos>() || ...)`), which is what
makes priority = pack order: the fold short-circuits at the first
non-empty queue. `ReplyTo` stores a pointer to a *thunk* - a static
function template `thunk_for<Ao>` whose only body is `post<Ao>(p)`;
instantiating it for a given AO bakes the destination into the code,
so the capsule needs no data beyond the pointer. `TimeEvent` fires
its payload with the same trick (`do_fire`), and the FSM state
pointer is the third instance of "a function pointer instead of a
virtual". `requires std::constructible_from<typename Ao::Event, Ev>`
on `post` is a *trailing requires-clause* constraining a single
function template rather than a named concept - used when the
requirement is local to one function.

## 8. Scheduler (`kernel/kernel.hpp`)

`Kernel<P, Ao1, Ao2, ...>`: AOs are the pack, **priority IS the pack
order**, first = highest; no separate priority table to keep coherent.
Because the order is a type, the pack answers ordering questions at
compile time (`Pack<Aos...>::index<Ao>()`), and Kernel uses that to
enforce the one ordering fact the payload rule needs: every
`Lease::dispatch` borrower precedes its lender (section 4).
The loop (`run()`), one turn:

1. `TimeEvents<P>::process()` - post every matured time event (main
   context, see section 9);
2. `step()` - pop ONE event from the highest-priority non-empty queue
   and dispatch it, run-to-completion; an urgent event arriving during
   a slow dispatch is served right after it because the next turn
   rescans from the top;
3. if `step()` found nothing, `idle_if_empty()`: re-check every queue
   with interrupts masked and, if still empty, `P::idle()` - which
   re-enables interrupts and sleeps in one breath, so no wake-up can
   slip between the check and the sleep. HOW that breath is atomic is
   the platform's business (the classic sequence is "enable, then
   sleep" where the core guarantees enable takes effect only after
   the following instruction; a core with a wait-for-interrupt that
   is itself the wake condition does it differently); the kernel only
   relies on the contract.

`init_all()` runs every AO's `init()` in pack order before the first
turn. Host tests drive `init_all()`/`step()` directly - `run()` never
returns. Starvation of low-priority AOs under fixed priority is by
definition a sizing/design error; the overflow counters make it
visible. The shallowest sleep keeps peripherals alive; deeper modes are
app policy, not kernel business - negotiated ABOVE the kernel by the
power model ([power.md](power.md)), which arms a mode and lets this same
hook take it, so the loop is unchanged and a program without a power
manager pays nothing.

## 9. Time (`kernel/time.hpp`, `kernel/time_event.hpp`)

**Timers post events; expiry runs in the loop, never in the ISR.** In
the AO world a timer never runs user code: `TimeEvent<P, Ao, Ev>` is
a static object declared next to its owner AO holding a payload; when
it matures, the payload is `post`ed to that AO. The tick ISR only
advances the counter and, by firing, wakes the CPU; `process()`, once
per loop turn, compares `P::now()` with the armed deadlines and posts
matured events in main context (the ISR-side
alternative is rejected: an ISR whose duration grows with armed
timers is the antithesis of short-dumb-ISR, and buys no precision
since a posted event waits for the current RTC step anyway).

- Armed events form an **intrusive singly-linked list** - no
  allocation, RAM = what is declared; arm/disarm/process run in the
  loop only, so the list needs no critical section by construction.
  An ISR that wants a delay posts an event to an AO, which arms.
- One-shot (`arm(delay)`) or periodic (`arm_every(period)`).
  Periodic re-arm is **drift-free**: next = previous deadline +
  period, never now + period; if processing lags, it fires at most
  once per turn and catches up.
- Deadline arithmetic is **wrap-safe** via signed difference:
  `(int32_t)(now - deadline) >= 0` works across the 32-bit wrap.
- RAII: a dying `TimeEvent` disarms itself (free on target, where
  they are static and never die; matters in host tests).
- `TimeEvents<P>::ticks_to_next()` asks the armed list the one question
  the list can answer and nobody else can: how long until the next
  thing this program has to do - 0 when something is already due,
  nothing at all when nothing is armed, the same wrap-safe arithmetic
  `process()` uses. It is a QUESTION and changes no state; the loop
  still fires the events. Its reason to exist is the power model
  ([power.md](power.md)): a stop that costs more to leave than the wait
  it saves is a bad trade, and only the armed list knows the wait.

**The tick is opaque, the rate is a platform constant.** Any given
timebase peripheral has its own natural rates (a 32 kHz-derived
periodic timer gives powers of two, a core systick gives 1000 Hz);
none of them is a truth of the world. The kernel reasons in ticks and
assumes nothing about the rate; `ticks_from_ms<P>()` /
`ticks_from_secs<P>()` are `constexpr` with CEIL semantics ("at least
this long": a timeout never fires early; identity folding when tps =
1000). `TimeStamp`'s fraction is milliseconds, self-describing across
platforms. Each target ships its own ticker driver, declaredly its
own; no preemptive "generic ticker" - the concept is the abstraction,
generalize on the second real specimen.

**C++ note - `constexpr` conversions.** `ticks_from_ms<P>(500)` is a
`constexpr` function template: with a constant argument it is
evaluated by the compiler and the 64-bit intermediate costs nothing
at run time - the deadline next to the AO is a literal in flash. It
also works at run time (then the 64-bit multiply is real: prefer
precomputing on 8-bit targets).

## 10. Failures: overflow and panic (`kernel/panic.hpp`)

Overflow is section 5's counter. **Panic** is the one hook for
unrecoverable failures: `panic<P, Reporter>(code, ctx)` is
`[[noreturn]]`: mask interrupts for good -> write the breadcrumb
(`PanicRecord{magic, code, context}`) into the platform's
reset-surviving storage -> `P::break_here()` (stops the debugger if
one is attached, does nothing otherwise) -> hand over to the
app-chosen `Reporter`. The kernel knows no LED: `HaltReporter`
(interrupts masked + forever loop) is the stock default;
blinkers, watchdog resetters and their compositions are target/app
code. Because the breadcrumb is written BEFORE any reporter runs, the
information is safe whatever the manifestation does; at boot
`take_panic_record<P>()` returns it once and clears it (cross-check
the reset-cause register for the full story).

## 11. Platform: what the machine provides (`kernel/platform.hpp`)

The kernel's only hardware touchpoints, stated as the `Platform`
concept - no `#ifdef`, no default (a default would hardwire one
target's include: the app names its platform once):

- `CriticalSection`: RAII guard, constructor masks interrupts,
  destructor restores the previous state (nests); enter/leave are
  also compiler memory barriers, so shared data needs no `volatile`;
- `idle()`: called with interrupts masked; re-enables and sleeps with
  no lost-wakeup window;
- `break_here()`, `now()`, `ticks_per_second` (positive constant),
  `atomic_width` (widest single-access load/store in bytes: 1 on an
  8-bit core, 4 on a 32-bit one - `Ring` chooses lock-free vs guarded
  with `if constexpr` on it);
- `panic_record()`: a reference to a `PanicRecord` in storage that
  survives reset without being zeroed by startup code.

`PanicRecord` is defined in `platform.hpp`, not in `panic.hpp`, on
purpose: it is the one kernel data type a Platform must host, so the
concept has to name it and `panic.hpp` (which owns the semantics)
sits above the concept in the include graph.

Every target stratum ships its implementation (today
`avrdx/platform_avr.hpp`, see [../avrdx/README.md](../avrdx/README.md));
`HostPlatform` (`host/platform_host.hpp`) gives a depth-counting
critical section, a test-controlled virtual clock and recording
idle/break - time becomes deterministic arithmetic in tests
(`ctest --preset host`), which is why the host tests cover first what
is hard to provoke on real hardware: queue overflow, entry/exit
ordering, drift-free re-arm, scan priority, MPSC stress.

**C++ note - `if constexpr`.** `if constexpr (cond)` (C++17) with a
compile-time condition discards the untaken branch entirely - it is
not even required to compile for the current types. brio uses it to
select code paths on platform facts (`atomic_width`) and, with
`static_assert(false, ...)` in the discarded branch (legal since
C++23), to turn "peripheral not present on this device" into a clear
compile error instead of a template failure.

## 12. Reference index

| Entity | Header | Role |
|--------|--------|------|
| `ActiveObject` (concept) | `active_object.hpp` | what Kernel requires of an AO |
| `Platform` (concept), `PanicRecord` | `platform.hpp` | what the kernel requires of the machine |
| `EventQueue<E, depth, P>` | `event_queue.hpp` | per-AO MPSC queue, overflow counter |
| `Overloaded`, `match`, `Entry`, `Exit`, `Fsm<Derived, Alts...>` | `fsm.hpp` | variant dispatch helpers, state machine base, Event, Status |
| `post`, `Subscribers`, `publish`, `ReplyTo`, `reply_to` | `post.hpp` | delivery primitives |
| `Borrowed<T, Lease>`, `Lease` | `borrowed.hpp` | pointer payloads with their lease in the type |
| `Pack<Aos...>`, `Kernel<P, Aos...>` | `kernel.hpp` | pack ordering questions (index, lends_ok); the loop: init_all/step/idle_if_empty/run |
| `TimeEvents<P>` (incl. `ticks_to_next`), `TimeEvent<P, Ao, Ev>` | `time_event.hpp` | armed list + owned time events |
| `ticks_from_ms`, `ticks_from_secs` | `time.hpp` | constexpr tick conversions |
| `PanicCode`, `panic_magic`, `HaltReporter`, `panic`, `take_panic_record` | `panic.hpp` | unrecoverable failures |

Include graph (arrows = includes; nothing here includes anything
outside `kernel/` and the standard freestanding library):

    platform.hpp        <- event_queue.hpp, time.hpp, panic.hpp,
                           time_event.hpp, kernel.hpp
    fsm.hpp             <- post.hpp
    active_object.hpp   <- kernel.hpp
    post.hpp            <- time_event.hpp, kernel.hpp
    time_event.hpp      <- kernel.hpp
    borrowed.hpp        <- (util/ producers of loans; nothing in kernel/)

An app that only posts includes `kernel/post.hpp`; an app that runs
includes `kernel/kernel.hpp` and gets the contract with it.
