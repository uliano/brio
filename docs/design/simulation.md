# Simulation

The model of the WORLD a brio program runs against when the machine
under it is the host: what a simulated device is, how the world and the
program exchange state, who owns time, and what a test built this way
may honestly conclude.

The pieces that exist are documented where they are built - the host
target's [front page](../host/README.md) and the
[viewer contract](../host/simulator.md). This page is the model above
them, and "What stands today" at the end says which parts of it are
code.

## Why this exists before its implementation

The order is deliberate, as it was for [block streams](block-stream.md)
and for [graphics](gfx.md), and the risk it answers is specific. The
first simulated device built was a display with a viewer watching it,
because a panel is the one device whose channel IS a picture. A model
extracted from that would put a graphical interface at its centre - and
a graphical interface is the exception, not the case. The general case
is a TABLE OF VALUES, most of which nobody wants to look at: a contact,
a shaft, a level in microvolts, a code driven out, a quantity computed
from another one. Fixing the model first makes the viewer what it
actually is: one source among three, and an ancillary one.

## The world never speaks to the program

On silicon nothing pushes state into a program. A contact is a level a
scanner polls, a converter leaves a result an interrupt body reads, an
incoming byte waits in a register. A simulator that invented an inbound
channel would model a machine that does not exist, and a program written
against it would not run on a target.

So the rule the rest of this page rests on: **the world writes values,
and the program reads them at its own pace.** Three consequences.

- The seam of every simulated device is a LEVEL, not a gesture. A level
  survives any compression or dilation of time; a queue of gestures
  would have to be delivered at instants nobody chose.
- While time does not advance, writing the world is legal and produces
  nothing - as pressing a button while the clock is stopped does. A
  pulse shorter than the program's own sampling is invisible, which is
  what the hardware would do with it too.
- Nothing in `kernel/` or `util/` knows any of this, and the host
  platform stays honestly single-threaded.

## The channel table

The world and the program share a table of CHANNELS. A channel has a
name, a kind, a unit and a range, and carries its value as an integer
with a declared scale - integer because the quantity is quantized
anyway, and because the same table is read by another process and
written by a file, where a floating-point value would invite a
precision nobody has.

A channel has a DIRECTION: *in* for what the program reads (a contact, a
shaft, a measured quantity), *out* for what it drives (a code, a duty, a
level, a lamp). And it has a SOURCE: it is either **stimulated** - some
source outside puts a value in it - or **computed** - a function of
other channels and of time.

Six rules make a computed channel deterministic. They are the whole of
the mechanism.

1. **It is evaluated in the world's turn**, never lazily inside the
   program's read. A value computed at read time would depend on when
   the program happened to look, and two reads in one turn could
   disagree - which destroys the snapshot semantics the whole model
   rests on.
2. **It reads the state at the START of the turn and writes the state at
   its end.** The world is a synchronous machine clocked by the turn, so
   a chain of computed channels costs one turn per stage. That delay is
   not a defect: a real plant has one.
3. **It may carry state**, so its shape is a STEP and not an expression:
   the current instant, the elapsed ticks, its own state, the table.
4. **It integrates with the elapsed ticks**, never "once per turn",
   because the turn rate is a policy and not a constant - see below,
   where a jump can make one turn worth a second of simulated time.
5. **A channel with dynamics declares its own next deadline** - the
   largest step its physics tolerates - so that a jumping clock cannot
   step over it.
6. **Behaviour is written in C++; the scenario file carries only
   numbers.** The moment a file can express behaviour there is a
   grammar, a parser and an interpreter to maintain, and the simulated
   world has become the project.

A computed channel is a MODEL, and what separates a model from an empty
stub is the four things it keeps: quantization to the device's least
significant bit, saturation at the rails, noise from a SEEDED generator
(an unseeded one is not a test), and delay - a conversion time is part
of the model, not an implementation detail of it.

From outside, **a computed channel is indistinguishable from a
stimulated one**: the program reads a level, a viewer shows a channel, a
trace records a value. Only the world knows the difference. That is what
makes a CLOSED LOOP possible with no new mechanism - a program that
drives an output and reads it back through a model of the wire is three
pieces, none of which knows the other two exist.

## The world, as a type

Two verbs:

- `next_deadline()` - the instant of its next change, empty when it has
  none;
- `turn(now)` - apply everything due at or before that instant.

A file has deadlines because it has lines; a plant has them because it
has a time constant. One interface, two reasons.

Values reach the table from three sources, and they are
interchangeable because they all use the same seam: direct calls from a
test, a scenario file, a live viewer. **Seam first, binding second** -
every simulated device has an in-process seam, and a binding to a
mechanism outside the process is optional. An in-process source follows
the program's clock and works under any time policy; a peer across a
process boundary follows the wall clock, and needs the program run at
real time.

## Time, and who owns it

There is one entrance - `P::now()` - and one place where time becomes
events: the kernel loop's `TimeEvents<P>::process()`. That is true on
silicon too, where the tick interrupt does no user work but advance a
counter. Because of that, a virtual clock needs no conditional
compilation anywhere: on the host, `now()` reads a variable.

| policy | who advances `now()` | what it is for |
|---|---|---|
| stepped | the harness, tick by tick | suites, goldens, reproducibility |
| jumping | the platform, to the next deadline, through the kernel's optional `idle_until()` | long scenarios: nothing can happen between two deadlines, so the empty time is skipped |
| paced | a real clock | the interactive case, and the only one another process may join |

Two notes on the jumping policy, because it is the one that is not
obvious. It costs nothing to support: it is the same platform hook built
for silicon whose timebase counts through sleep. And the world is what
bounds the jump - the loop offers the kernel's next deadline, the world
offers its own, and the jump goes to the nearer.

**The world's turn happens between two of the kernel's steps, never
inside a dispatch.** A simulated device whose silicon twin interrupts is
its driver's interrupt BODY, called from the turn: it runs to
completion, like the interrupt it stands for. It cannot land inside a
dispatch, which is one of the things the last section is about.

## The scenario file

A line-oriented ASCII file: an instant in ticks, a channel by name, a
value. It is A SOURCE and not a mechanism - it feeds the seam a test
calls directly, so nothing has to open a file in order to be tested.

Beyond values it carries three things:

- **outcomes, which are stimuli like any other.** A timeout, a corrupt
  frame, a device that does not answer are lines in the file; on a bench
  they are a wire pulled out of a breadboard.
- **the parameters of the computed channels** - gains, offsets, time
  constants, the noise seed - and never their behaviour.
- **the degrees of freedom.** Stimuli declared independent at the same
  instant may be permuted. This is what lets an exploration of arrival
  orders run without inventing orders that cannot physically happen (a
  completion before its request), which is the difference between a
  useful exploration and a flood of false reports.

What a file cannot do is a world that LISTENS: it is open loop by
construction. A plant that reacts to what the program drives is a
computed channel, and the two compose - the file stimulates the model.

## What a judgement is

Two kinds, and confusing them is how a suite rots.

A **golden** is an exact sequence: the trace of a run, compared line by
line with a recorded one. It says "nothing changed", it is legible to a
human, and it is an excellent regression net - but it holds only while
the program is unchanged AND while one order of arrival is the only one.

An **invariant** is a property computed OVER the trace: no edge lost,
the total conserved, every request answered, nothing arriving before its
cause, no queue overflowed, the loop converging with the error under a
bound. It is what survives permuted arrival orders, and it is the only
thing that can be asserted about a closed loop, where the output is a
function of the program's own behaviour.

brio's services already publish the vocabulary invariants are written
in, because in every one of them the accounting IS the API: queue
overflow counters, edges counted, lost steps counted, overwrites
counted, rejections counted, the panic breadcrumb.

## What the host does not tell you

Stated, so that no one reads a green suite as more than it is.

- **An interrupt at an arbitrary instruction boundary.** The world's
  turn calls an interrupt body between two dispatches; silicon fires it
  wherever it likes, including in the middle of a driver's register
  sequence. Races between an interrupt and the loop are invisible here.
- **Stack depth.** The host has megabytes where a small target has two
  kilobytes.
- **Timing budgets and latency** - the host's are its own.
- **The narrow machine.** A 16-bit `int`, the guarded paths a platform
  with an atomic width of one byte takes.
- **Compiler independence.** The host builds with the same compiler
  family as the targets.
- **The silicon**, which is the third plane of truth and is never on
  this side.
- **The latency a shared bus adds**, where no bus is modelled: model the
  CONSTRAINT - a number the bench measured - rather than the mechanism.

## What is deliberately absent

- **Bus mechanics and wire timing.** A simulated device is modelled at
  the level the application consumes it, not at its transfers. What a
  fake bus proves is that the driver agrees with our reading of a data
  sheet, which is not what a driver has to agree with.
- **A viewer that commands.** Every channel carries state, never orders;
  a command is an event and would need a mechanism of its own, with a
  reason of its own.
- **An external owner of time.** Lockstep with another process is not
  offered; the in-process harness is the lockstep.
- **An expression language in the scenario file** (rule 6 above).
- **Empty stubs.** A simulated device that answers instantly, exactly
  and always is a machine no one will ever have to program against.
- **The host as the principal development platform.** The easy case
  would shape the code, which is the trap graphics avoided by settling
  its contract before its primitives.

## What stands today

The model above is the fixed point; these are the pieces of it that are
code, each born with its first user, the rest waiting for theirs.

- `brio/host/platform.hpp` - the platform, with the clock as a variable
  a harness advances, and the two-core form of it.
- `brio/host/sim_input.hpp` - a contact and a shaft as levels, with the
  seam that is one call.
- `brio/host/sim_panel.hpp`, `brio/host/shared_segment.hpp` - the
  world's side as a region another process writes, named channels and
  fixed slots: the channel table in its first, specialized form.
- `brio/host/sim_display.hpp` - the one channel that is a picture.
- `brio/host/sim_flash.hpp`, `brio/host/sim_usb.hpp` - two simulated
  devices that predate this page and already obey its rule: a seam in
  process, injectable failures, no invented channel inward.
- `brio/host/gfx_reference.hpp` - a judge, which is a different thing
  from a simulation and is kept in a different file for that reason.

Not code yet: the general channel table with its kinds and units, the
computed channels and their step, the scenario file, the world as a type
with two verbs, and the jumping time policy - the kernel's side of which
(`idle_until()`) exists, since it was built for tickless silicon.
