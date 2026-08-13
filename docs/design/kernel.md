# The active-object kernel

Decisions of 2026-08-13 unless noted. Headers: `lib/brio/src/kernel/`.
Everything in this stratum is pure logic - host-testable, no AVR
includes.

## Platform concept (`kernel/platform.hpp`)

The kernel's only hardware touchpoints, declared as a concept - no
`#ifdef`, no default (a default would hardwire an AVR include): the
app names its platform once.

- a RAII critical-section type whose enter/leave are also compiler
  memory barriers (shared data needs no `volatile`);
- `idle()`, `break_here()`, `now()`;
- `ticks_per_second` as a positive compile-time constant.

`AvrPlatform` (`avrdx/platform_avr.hpp`) implements them with
intrinsics; `HostPlatform` (`host/platform_host.hpp`) gives a
depth-counting critical section, a test-controlled virtual clock and
recording idle/break - time becomes deterministic arithmetic in tests
(`pio test -e native`). Host tests cover first what is hard to provoke
on silicon: queue overflow, entry/exit ordering, drift-free re-arm,
scan priority, MPSC stress.

## Events: value semantics, per-AO variant

Events are COPIED into per-AO queues - no pools, no reference
counting. Each AO declares its own event type: a `std::variant` of
small trivially-copyable structs. Plain shared structs are the lingua
franca between publishers and subscribers - no global signal enum, no
system-wide event type. Measured on avr-gcc 16.2 -Os: `std::visit` on
a 4-alternative variant compiles to a plain switch, +30 bytes flash,
identical RAM vs a hand-tagged union.

The 8-byte size target is a GUIDELINE for the event envelope, not a
law: every queue slot pays that AO's largest alternative and the push
copies it with interrupts masked (~1 us per 8 bytes at 24 MHz).
Per-AO deviations are legal with numbers in hand - e.g. the SPI
request descriptor (~16 bytes): the request IS the arbitration token.
Payloads above the budget travel BY REFERENCE (pointer/span with an
ownership rule) when the data already lives in a structurally
necessary buffer, BY VALUE when lifetime simplicity is worth the RAM.
One event = one envelope; cargo is never chopped across events.

## Queues (`kernel/event_queue.hpp`)

One MPSC `EventQueue<E, depth, Platform>` per AO. Under the
cooperative scheduler the only real concurrency is ISR vs main loop
(AVR has no CAS - the honest primitive IS the brief cli section):
push saves SREG, disables interrupts, copies, restores; pop protects
only the index update. Posting from an ISR never blocks. Queue depth
is a per-AO template parameter sized on that AO's real burst.

`Ring` (SPSC, no cli, byte indices) is NOT the event queue: it stays
at the BYTE level inside drivers - the ISR pushes bytes lock-free, the
driver AO condenses them into few events (bytes at high rate, events
at low rate).

**Overflow**: a full queue is a sizing mistake, not a runtime
condition to handle. Saturating per-queue overflow counters are ALWAYS
maintained (named static symbols, inspectable from gdb); the reaction
is a compile-time knob: `OverflowPolicy::count` (drop, keep running -
release default) or `panic` (debug). No bool-returning post spreading
untested error branches.

## Scheduler (`kernel/kernel.hpp`)

`Kernel<P, Ao1, Ao2, ...>`: AOs are monostate classes visited via fold
expressions - no virtual base. **Priority IS the pack order**, first =
highest. One event per iteration: pop the highest-priority non-empty
queue, dispatch run-to-completion, rescan from the top. Starvation
under fixed priority is by definition a sizing error - the overflow
counters make it visible. All queues empty -> IDLE sleep via the
cli-check / sei+sleep_cpu idiom (sei takes effect after the following
instruction: the lost-wakeup race is closed by the silicon). Deeper
sleep modes are app policy, not kernel business.

## State machines (`kernel/fsm.hpp`)

HSM-ready contract, flat (QP-style) implementation. The current state
is a handler function `Status handler(const Event&)`; outcomes are
`handled | unhandled | transition(target)`. `unhandled` today means
"ignore", tomorrow "bubble to parent" - it IS the HSM hook. Entry/Exit
are reserved kernel structs PREPENDED to the variant, delivered
synchronously by the transition machinery (never posted, never
hand-written at call sites); they are empty so queue slots do not
grow. `start()` arms the initial state and delivers the first Entry;
Entry may chain `transition()` (pass-through states for free); Exit's
return is deliberately ignored (exit is an action, not a decision).
`Fsm<Derived, Alts...>` is CRTP so two AOs with identical alternatives
get distinct machines. Full HSM (parent pointers, bubbling, LCA
chains) gets built only when a real AO demands it. Trivial AOs may
legally keep a plain switch inside one handler.

## Delivery primitives (`kernel/post.hpp`)

- `post<Ao>(ev)` - addressed: commands go to a named AO.
- `publish(Subscribers<A, B...>{}, ev)` - facts are published: one
  copy per subscriber, compile-time subscriber lists, a concept checks
  each subscriber's variant accepts the event.
- `ReplyTo<Payload>` - replies return to sender: a one-function-
  pointer capsule (2 bytes on AVR, trivially copyable, travels INSIDE
  the request event) built via `reply_to<RequesterAo, Payload>()`. A
  service calls `req.reply.send(payload)` without knowing the
  requester; a default-constructed capsule is null and `send()` is a
  no-op (fire-and-forget for free). A requester whose variant cannot
  hold Payload fails to compile at the `reply_to` site. This is the
  return channel every bus AO uses.

## Time (`kernel/time.hpp`, `kernel/time_event.hpp`)

Timers post events; expiry runs in the kernel loop, never in the ISR
(a periodic ISR whose duration grows with armed timers is the
antithesis of short-dumb-ISR, and buys no real precision since any
posted event waits for the current run-to-completion step anyway).
The PIT ISR just ticks and wakes the CPU; each loop turn compares the
tick with armed deadlines and posts matured events in main context.
`TimeEvent<P, Ao, Ev>` objects are statics owned by their AO, linked
into an intrusive list when armed - no allocation, RAII disarm.
Periodic re-arm is drift-free: next = previous deadline + period.
Deadline arithmetic over wrapping counters uses signed difference.

**The tick is opaque, the rate is a target constant.** Power-of-two
tick rates are a truth of the AVR PIT, not of the world (future
targets tick at 1000 Hz from SysTick). The kernel reasons in opaque
ticks and assumes nothing about the rate; conversions are constexpr
with CEIL semantics ("at least this long" - a timeout never fires
early; identity folding when tps = 1000). `TimeStamp`'s fraction is
MILLISECONDS (0..999), self-describing across targets; the producing
driver converts with its known rate. `BasicTicker` stays declaredly
AVR (pow2 asserts, PIT) in `avrdx/`; no preemptive "generic ticker" -
the concept is the abstraction, generalize on the second real
specimen.

## Panic (`kernel/panic.hpp`)

`brio::panic(code, ctx)` is `[[noreturn]]`: cli -> breadcrumb (cause +
queue/AO id) into a `.noinit` variable that survives reset
(cross-checked against RSTCTRL at next boot, fetched with
`take_panic_record<P>()`) -> `asm("break")` (halts in gdb if the OCD
is active, plain NOP otherwise - a free breakpoint that does not
consume the 2 hw slots) -> app-pluggable Reporter. The kernel knows no
LED: the app plugs a reporter from the stock library (halt - default,
blink_forever, reset_now, blink_then_reset) or writes its own. The
breadcrumb is written BEFORE any reporter runs.
