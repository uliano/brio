# From the main loop to active objects

This page explains the execution model of a brio program before any
C++ appears. It is written for someone who has programmed a
microcontroller in C with interrupt handlers and a main loop, and who
has not met active objects before. The next page,
[the first program read in stages](first-program.md), shows the same
ideas in the source of a real application.

## The program you already know

A typical bare-metal program has two kinds of code. Interrupt handlers
run when hardware demands it: a timer expired, a byte arrived, a pin
changed. The main loop runs the rest of the time, polling flags the
handlers set and doing the work those flags announce. Waiting is done
with a delay loop or by spinning on a flag.

This works, and for a small program it is the right tool. It gets hard
to extend in a recognizable way. Each new feature adds flags, and each
flag has a writer (a handler) and a reader (somewhere in the loop) that
must agree on when the flag is cleared. A delay loop in one feature
stalls every other feature for its duration. A piece of the loop that
needs to do something "in 200 ms" must either count loop iterations or
keep its own timestamp and compare it on every pass. When two features
need the same peripheral, the loop becomes the place where their turns
are negotiated by hand. None of this is a flaw of C or of the main loop;
it is the cost of coordinating by shared variables.

brio keeps the two kinds of code (handlers and a loop) and replaces the
shared flags with three things: events, queues and active objects.

## An event is a record of something that happened

An event is a small struct that says what happened and carries the data
that goes with it: "the tick timer expired", "change the blink period to
250 ms", "a line of text arrived". It is a value, a few bytes, copied
like any struct. An event carries data; it does not carry code. The
code that reacts to the event lives in the component that receives it.

Separating the record from the reaction is the first step away from
flags. A flag says "something happened" and leaves the loop to work out
what to do and whether it has already done it. An event says exactly
what happened, once, and is consumed exactly once.

## A queue lets an event wait

Events are put into queues. A queue is a small fixed-size buffer of
events in the order they were posted. A handler that has detected
something posts an event into a queue and returns at once; it does not
wait for anyone to react. The reaction runs later, from the loop, when
the component that owns the queue is given the event.

This is the second step away from flags: a queue holds several events,
so nothing is lost when two things happen before the loop gets around
to them, and nothing has to be cleared. Posting an event into a queue
copies a few bytes with interrupts briefly masked. It never waits. If
the queue is full, the event is dropped and counted (that count is how
you discover the queue was too small; see
[capacities and overload](first-program.md#capacities-overload-and-how-to-see-a-failure)).

## An active object is a responsibility with its own queue and state

An active object is one component of the application: the thing that
blinks the LED, the thing that supervises it, the thing that owns the
serial port. Each active object has its own queue of events and its own
persistent data. It reacts to one event at a time by running a short
piece of code, the event handler, which may change the object's data,
post events to other active objects, arm a timer, or change the
object's state. Then the handler returns.

An active object is not a thread. It has no stack of its own, it never
blocks, and the kernel does not switch between active objects in the
middle of their handlers. All active objects run on the one main stack,
one handler at a time. Between two events an active object holds no
local variables; everything it needs to remember is in its persistent
data.

## The kernel: a loop that serves queues

The kernel is the loop. On each turn it does three things:

1. It checks the armed timers and posts an event for each one that has
   expired.
2. It looks at the queues of the active objects in priority order and
   takes the oldest event from the first queue that is not empty. It
   hands that event to its active object's handler and lets the handler
   run to completion.
3. If every queue was empty, it puts the processor to sleep until the
   next interrupt. The check and the sleep are arranged so that an
   interrupt arriving between them is not lost: the kernel checks the
   queues with interrupts masked, and the platform's sleep code is
   written so that a pending interrupt still ends the sleep (on a
   Cortex-M the wait-for-interrupt instruction wakes on a pending
   interrupt even while masked, and interrupts are unmasked right after
   it; each platform's header says how its core does it). You write
   none of this; it is the kernel's and the platform's.

Step 2 is the heart of the model, and its rule is the one rule to
remember:

**A handler runs to completion. Nothing else in the application runs
until it returns.** A higher-priority event that arrives in the
meantime does not interrupt the handler; it waits in its queue and is
served on the very next turn, before anything of lower priority.

The consequences for the programmer follow directly:

- Keep handlers short. A handler that spins for 50 ms stalls every
  other active object for 50 ms. When work must continue later, arm a
  timer or post an event instead of waiting inside the handler.
- A handler never needs a lock against another handler. Two handlers
  never run at the same time, so an active object's data is touched by
  one handler at a time.
- Interrupt handlers still interrupt. "Cooperative" describes the
  relationship between active objects, not between an active object and
  the hardware. An interrupt handler can run in the middle of an event
  handler. What an interrupt handler may do with the kernel is exactly
  one thing: post an event. It must not touch an active object's data
  directly, because the handler it interrupted may be in the middle of
  changing that data.
- One interrupt handler does not interrupt another. brio requires this
  of every platform it runs on (each keeps the promise its own way: the
  AVR by its architecture, the Cortex-M families by giving every
  interrupt line the same priority). So there are exactly two contexts,
  interrupt and loop, and one boundary between them, and posting an
  event is the one operation built for that boundary: two interrupt
  handlers that post to the same queue never run at the same time.
  The full statement is
  [kernel.md, "The model in one page"](../design/kernel.md#1-the-model-in-one-page).

## Priority

The application lists its active objects in a fixed order. The first is
the highest priority. When the kernel looks for the next event, it scans
the queues in that order and serves the first non-empty one. Priority
therefore decides who goes next when several events are waiting; it
never preempts a running handler.

The diagram follows one scenario. `Low` is a low-priority active object
with a slow handler. `High` is a high-priority one. An interrupt posts an
event to `High` while `Low`'s handler is running.

```
time -->

interrupt     : ........ post(High) ..............................
                              |
High's queue  : [empty] ..... [E1] ............................ [empty]
                                                               ^ taken
Low's handler : |===== running, not interrupted by E1 =====|
                                                           |
kernel loop   :                                            | next turn:
                                                           | scan from
                                                           | the top,
                                                           | High first
High's handler:                                              |== E1 ==|
```

E1 waits until `Low`'s handler returns. It is then served before any
other waiting event of lower priority, because the kernel rescans from
the top on every turn. The latency of a high-priority event is bounded
by the longest handler in the program, which is why handlers are kept
short.

## State

Many active objects behave differently depending on what happened
before: a connection is open or closed, a motor is stopped, running or
braking. brio lets an active object name those situations as states.
A state is a handler function; the current state of the object is a
pointer to the handler that will receive the next event. Changing state
means changing that pointer, and the kernel delivers an `Exit` event to
the old state and an `Entry` event to the new one on the way, so that
each state can set up and tear down what it needs.

The model is flat: a state is one handler, with no parent states. An
active object whose behaviour does not depend on history keeps one
state for its whole life, and that is normal. The first program uses one
state per object; the mechanism is explained there at the point where
the code uses it.

## Time

A delay loop has no place inside a handler, because it would stall every
other active object. Instead an active object arms a timer, called a
time event, with a payload and a delay or a period. When the time
expires, the kernel posts the payload to the owning active object's
queue, in the loop, like any other event. The tick interrupt only counts;
no user code runs in it. The handler that reacts to the timer runs in
the loop, with interrupts enabled, like every other handler.

## Talking between active objects

Active objects share no variables. One active object tells another
something by posting an event into its queue. The receiver reacts in its
own handler, in its own turn. Because the sender does not wait for the
receiver, there is no deadlock and no ordering puzzle: the sender's
handler returns, and the receiver's runs on a later turn.

Three delivery forms exist. The first program uses the first one; the
other two are introduced where a program needs them:

- post: send one event to one named active object (a command);
- publish: send one event to a list of subscribers (a notification);
- reply: a service returns a result to whoever asked, through a return
  channel carried inside the request.

## What the model costs and what it does not promise

- Posting never waits, but a handler can. A handler that calls a
  library function which waits for hardware stalls the application for
  that long. In brio the text output function `print` is such a call:
  it waits until the serial transport has accepted every byte
  (util/print.hpp, "Delivery policy"), so a long message on a slow port
  costs the whole application that time. The first program prints
  nothing.
- A queue is fixed-size and sized by the programmer. A full queue drops
  the event and counts it; nothing blocks and nothing is reallocated.
- Static storage. Every active object, queue and timer is a static
  object whose size is known when the program is compiled. There is no
  heap in the kernel. This is what makes the RAM footprint readable from
  the source, and it also means an active object cannot be created at
  run time.
- One core. Everything on this page describes one processor core
  running one kernel loop. A chip with two cores runs two kernels, one
  per core, with a bridge between them
  ([kernel.md, "Two cores: two kernels and a bridge"](../design/kernel.md#12-two-cores-two-kernels-and-a-bridge-utilinboxhpp));
  the single-core model is the one to learn first.

## Where this comes from

The model is the cooperative ("QV") flavour of active objects described
in Miro Samek's book on practical statecharts. brio is a clean-room
implementation: the concepts come from the book, the code from nobody
else's. The full design, with every decision and its reasons, is
[docs/design/kernel.md](../design/kernel.md); read it after the first
program, when the names mean something.
