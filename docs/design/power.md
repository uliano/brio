# The power model

A microcontroller spends most of its life with nothing to do. Deciding
that the program may STOP - and how far down - is not a driver's
business and not the kernel's either: it is a statement about the whole
application, because every deep mode gates clock domains and shortens
the list of things able to wake the machine. This page is the model that
makes that decision explicit, negotiated and target-independent. The
mechanism it stands on - which modes exist, what each gates, what wakes
it - is each target's, documented in that target's folder
(`docs/avrdx/platform.md`).

Contracts and services: `util/power.hpp`. The kernel question it needs:
`TimeEvents<P>::ticks_to_next()` (`kernel/time_event.hpp`). The
realizations: `AvrSleepSite` over `Sleep` in `avrdx/sleep.hpp`, and
`SamSleepSite` over `Pm` in `samc/sleep.hpp`.

**The model has a second silicon under it, and it needed no change.**
On the SAM C21 the vote round, the unanimity rule, the `PowerLock`
ceilings, the deadline guard and the first-event-after-wake contract
all run on the real kernel with two voters, with `util/power.hpp`
untouched. Two things the second target did do is prove out: the
never-deeper mapping is no longer the identity (that family's deepest
stop is STANDBY, so `deep` maps to it and `armed()` reports what was
really taken), and the duty the section below places on a platform's
idle path turned out to cost nothing there - PM.SLEEPCFG already IS
the armed mode, so the hook takes it by not touching it. What the
second target adds is a target-level restriction the model does not
express: its kernel tick stops in standby, so an application there may
only ask for standby with no time event armed (`docs/samc/platform.md`).

## The ladder

`SleepDepth` is an ordered ladder of four rungs, shallowest first:

- `none` - not a sleep mode. It is the ABSENCE of a request: the
  kernel's own idle hook keeps behaving exactly as it always does, which
  is what every brio program already does when its queues run dry.
- `light` - the shallowest real stop the target has. Everything but the
  CPU keeps running; the wake-up list is complete.
- `standby` - clock domains are gated; only the peripherals that ask
  keep running, and only they can wake the machine.
- `deep` - the deepest stop. Almost nothing survives.

**A target need not have all four**, and the ones it lacks it maps to
the nearest SHALLOWER rung - never deeper than asked. That direction is
the whole safety of the rule: a portable application asking for `deep`
gets the deepest stop that machine really offers, and never one that
loses more than the application agreed to lose. Where a target does have
every rung the mapping is the identity, and the model still reads back
what was actually taken rather than what was asked.

## The site: arming and sleeping are two different acts

`SleepSite` is what a target must provide: `arm(depth)`, `disarm()`,
`armed()`. That is all - and in particular, **not the sleep itself**.

The split is not an accident of one instruction set; it is the load-
bearing piece of the design. On every machine brio targets, the mode
lives in a register and the CPU stops when the idle path executes its
stop instruction. So the manager arms and returns; its dispatch ends;
the kernel loop finds every queue empty and calls the platform's
`idle()`; and THAT is the sleep. No new kernel hook exists, `Kernel::run()`
is untouched, and an application without a power manager pays nothing.

The one duty this places on a target is stated in the `Platform`
contract's neighbourhood rather than in it: a platform's idle path must
let an already-armed deeper mode STAND instead of imposing its own
shallow one. A platform that re-armed its default every turn would
silently undo the decision every stakeholder had just voted on.

## The round: unanimity among stakeholders

A sleep request is not an order, it is a question put to everyone with
something at stake:

1. someone posts `SleepRequested{depth, reply}` to the manager - an
   idle-detecting AO, a console command, a supervisor;
2. the manager clamps the depth (below), guards it (below), and posts
   `PrepareSleep{depth, reply}` to every **voter** in its pack;
3. each voter answers `SleepVote{ok}` - always, exactly once;
4. **one not-ok ends the round.** Unanimity arms the site and the
   requester is told ok; anything else leaves nothing armed and the
   requester is told not-ok.

Why ask at all: a bus engine mid-transfer, a converter mid-conversion, a
driver whose oscillator is about to stop each hold a fact the requester
does not have. Why unanimity rather than a majority or a veto-by-
priority: the cost of being wrong is asymmetric. A refused sleep wastes
power; an accepted one loses data.

The manager does not time out. A voter that never replies stalls the
round, and that is a bug to fix, not a condition to recover from -
which is also why a voter's queue must have a slot for the vote
(`BusMaster` sizes one in, deliberately).

The requester learns the outcome through the same `SleepVote` a voter
sends. One vocabulary, both directions.

`BusMaster` (`util/bus_master.hpp`) is the first voter in the framework
and the model of the role: it answers ok only when nothing is in flight
and its pending FIFO is empty, because a transfer outlives the dispatch
that started it and completes on an interrupt a gated domain would
swallow.

## Standing restrictions: the stakeholders that cannot be asked

Voting works for stakeholders the manager can ask - active objects,
dispatched in the loop. A stakeholder that lives in an INTERRUPT cannot
answer a `PrepareSleep`, and asking it afterwards would be asking too
late. It takes a `PowerLock` instead:

    static brio::PowerLock burst;
    burst = Pm::restrict(brio::SleepDepth::light);   // start of the burst
    ...
    burst.release();                                 // or scope exit

While a lock lives, no round arms deeper than the depth it names; the
effective ceiling is the SHALLOWEST live restriction. The counters are
kept under the platform's critical section, so both ends are safe from
an ISR. The lock is an owned right: non-copyable, movable, released by
its destructor.

A request deeper than the ceiling is CLAMPED, not refused - the program
asked to stop and it stops, as far as it is currently allowed to. A lock
taken while a round is already armed applies from the next request on:
it does not retroactively disarm. That is not a gap, because the
interrupt that takes the lock is itself a wake, and the first event
after a wake disarms.

## The deadline guard

Leaving a deep mode is not free. Whatever the target, the bill is the
same shape: the clock source has to come back, and the supply has to
reach its working point. Stopping for less than that is a losing trade,
so a request for `standby` or deeper is REFUSED - not clamped - when the
nearest armed time event is nearer than `PowerConfig::min_deep_ticks`.

This is the one thing the model needs from the kernel, and it is a
question, not a scheduling decision: `TimeEvents<P>::ticks_to_next()`
walks the armed list with the same wrap-safe signed arithmetic
`process()` uses and answers "how long until the next thing this program
has to do" - 0 if something is already due, nothing at all if nothing is
armed. Calling it changes no state.

The default, two ticks, is the worst wake-up bill measured on the first
target (a crystal's 1.77 ms restart) expressed in the coarsest timebase
brio runs on (1024 Hz). An application with a faster tick, a cheaper
wake or a different tolerance sets its own; the knob is a compile-time
configuration struct, so the comparison folds.

Nothing here guards `light`: the shallowest stop costs a handful of
cycles to leave and its wake-up list is complete.

## The first event after a wake disarms

The manager does not sleep, and it does not wake anyone. The machine
comes back on an interrupt; that interrupt's handler posts what it
always posts; and the manager's next dispatch - **of any event** -
first disarms the site and publishes `WakeReport{was}` before doing
anything else. Nothing polls, and the round ends exactly where the
program resumes.

Two consequences worth stating:

- A wake that has nothing to say to the manager leaves the mode armed,
  and the program stops again on the next empty turn. That is correct:
  the application's decision to be asleep has not been revisited.
- A wake path that DOES want the manager to know, without asking for
  anything new, says so with `SleepRequested{none}` - "awake, no new
  request". `none` is a no-op that still replies ok, so this is one
  posted event and no special case.

`WakeReport` reaches the voters whose event variant declares it. It is
a notification, not a subscription: a stakeholder is in the list because
it has a say in the decision, which is not the same as wanting the news
afterwards, and forcing the alternative into every voter's variant would
make a bus engine pay queue slots for a fact it has no use for.

**And a second finding from the same target, one campaign later: a
site can LIFT a target restriction with the model still unchanged.**
The SAM's v1 restriction - standby only with no armed time event,
because its tick freezes there - fell entirely INSIDE the two verbs the
concept always had: the timed site (`SamTimedSleepSite`) places an RTC
alarm on `ticks_to_next()` in `arm()` and catches the tick counter up
in `disarm()` (or in the alarm's own ISR), and the manager never
learned a new word. The one thing the site DOES lean on is already in
this file: the after-a-wake convention - a wake path with nothing to
say sends `SleepRequested{none}` - which on the AVR is a courtesy (the
PIT tick re-wakes the machine regardless) and with a timed site is
LOAD-BEARING, since a woken program that idles again without speaking
re-enters a standby whose alarm may already be spent. The site's own
documentation carries that obligation.

## What is deliberately not here

- **No idle detection.** Nothing in the model decides *when* the
  program has nothing to do; something has to post the request. That is
  application knowledge (a UI gone quiet, a shift ended, a command),
  and inventing a rule for it would be inventing an application.
- **No per-driver RUNSTDBY policy.** Which peripherals must survive a
  given depth is a target-and-application matter; the drivers expose the
  flags, the voters know what they need, and the model carries the
  negotiation between them - not the answers.
- **No current numbers.** The model is about who decides and when.
  What a given depth actually saves is a bench-supply measurement,
  recorded in the target's own document.
