# The power model

This page decides how a brio program stops: the ladder of sleep depths
a program asks for, the vote among the components with something at
stake, the lock an interrupt handler holds instead of a vote, the
guard that refuses a deep stop the next timer would make a bad trade,
and what the program may assume on the first event after a wake. It
is written for an embedded programmer who has put a chip to sleep
with a vendor function and wants the decision made once, for the
whole application, by the parts that know what is in flight. The
mechanism underneath (which modes exist, what each gates, what wakes
it) is each target's, documented in that target's folder (the AVR's is
[the AVR platform page](../avrxt/platform.md)).

A microcontroller spends most of its life with nothing to do. Deciding
that the program may stop, and how far down, is not a driver's
business and not the kernel's either: it is a statement about the
whole application, because every deep mode gates clock domains and
shortens the list of things able to wake the machine. The model here
makes that decision explicit, negotiated and target-independent.

The smallest use: a `PowerManager` in the list of active objects the
application gives Tenuto, the kernel ([kernel.md](kernel.md)), with
the voters named, a target's sleep site, and something that posts
`SleepRequested{depth, reply}` when the application knows it is idle.
The prerequisite that decides correctness: **the wake path that
resumes the program posts `SleepRequested{none}`**, which tells the
manager the program is awake without asking for a new depth. The rule
holds on every target, so a program need not know which: it costs one
event, it is a no-op where the target does not need it, and on a
target whose tick stops in the chosen mode a woken program that idles
again without saying so re-enters a sleep whose alarm may already be
spent.

In this page a rung is one depth of the ladder below; a timed site is
a sleep site that, before a sleep in which the kernel's tick stops,
arms a hardware alarm for the kernel's next deadline; its witness is
the counter it reads afterwards to resynchronize (advance) the
kernel's tick count by the span that was slept.

Contracts and services: `util/power.hpp`. The kernel question it
needs: `TimeEvents<P>::ticks_to_next()` (`kernel/time_event.hpp`).

### Realizations

Common to all: `SleepDepth`, the `SleepSite` concept (`arm`, `disarm`,
`armed`), `PowerManager`, `PowerLock`, the vote round and the deadline
guard. None of `util/power.hpp` is per target, and every realization
below ran it unchanged. What differs is the ladder each family can
offer, and whether the kernel tick survives the rung.

| stratum | realization | beyond the contract |
|---|---|---|
| avrxt | `AvrSleepSite` over `Sleep` (`avrxt/sleep.hpp`) | the ladder is the identity: `light` is IDLE, `standby` is STANDBY, `deep` is POWER-DOWN, on the AVR DA, DB and EA alike; the tick (the RTC's PIT on the 32 kHz oscillator) runs through every rung, so nothing needs resyncing. What a deep rung costs at the wake is the die's: on the DA and DB a regulator start-up once every oscillator is let go (125 and 290 us, which `PMODE` can buy back), on the EA none at all (an OSCHF restart is its 9.2 us whatever else stopped), and the main clock's own start-up on all three ([the AVR platform page](../avrxt/platform.md)) |
| samc21 | `SamSleepSite` and `SamTimedSleepSite` over `Pm` (`samc21/sleep.hpp`) | `light` is IDLE2 (IDLE0 is the reset value, so `armed()` could not read it back), `standby` and `deep` are both STANDBY (nothing deeper exists; `armed()` reports `standby` for a `deep` request); the tick stops in standby, so the plain site restricts standby to a program with no time event armed, and the timed site lifts that with the RTC as alarm and witness (below) |
| stm32g0 | `Stm32g0SleepSite`, `Stm32g0TimedSleepSite` and `Stm32g0LptimTimedSleepSite` over `Pwr` (`stm32g0/sleep.hpp`) | `none` and `light` are both Sleep (the only mode between Sleep and Stop wants the whole program at 2 MHz, which a site may not do), `standby` is Stop 0, `deep` is Stop 1; Standby and Shutdown are off the ladder because the program does not resume from them; a SYSCLK restore and a ticker pause ride the two functions; the timed sites take the RTC or the LPTIM as alarm and witness, and both refuse a `Tickless` platform, whose timebase counts through the Stop by itself |
| stm32f4 | `Stm32f4SleepSite` and `Stm32f4TimedSleepSite` over `Pwr` (`stm32f4/sleep.hpp`) | `none` and `light` are both Sleep (this family has nothing between Sleep and Stop), `standby` is a Stop on the main regulator, `deep` the same Stop on the low-power one with the flash in power-down, one mode with two prices, so `armed()` answers from the mirror the site wrote and `deep` for a Stop somebody else armed; Standby is off the ladder and enforced off it, every `arm()` clearing PDDS, because its wake is the reset vector; a SYSCLK restore rides `disarm()` (a Stop leaves the core on the HSI with the PLL, the HSE and over-drive gone) and the ticker is paused across a deep rung, which is also ES0298 2.2.1's workaround; the timed site takes the RTC whole, the wake-up timer as alarm and the calendar and sub-second counter as witness, and the Stop is left only through the NVIC line the application must enable |
| ch32v00x | `Ch32SleepSite` and `Ch32TimedSleepSite` over `Pwr` (`ch32v00x/sleep.hpp`) | `light` is Sleep, `standby` and `deep` are both Standby, the deepest this silicon has (`armed()` reports `standby` for a `deep` request, the SAM's rule); the core wakes from a Standby on the HSI with the PLL off, so `disarm()` restores the clock before anything clocked from HCLK runs right; the tick (the STK) stops with the core, so the plain site restricts a Standby to no armed deadline and the timed site lifts that with the AWU as the alarm, an alarm with no readable count, so the slept span is the programmed interval when the AWU is what woke the core and nothing otherwise, and the LSI it runs on is measured at init against the STK, the counts rounded up so an alarm lands late and never early; the AWU's line is enabled in the PFIC so its wake is a level the core cannot miss on the cycle it goes to sleep (an edge-only wake can, measured), which makes the AWU's vector one the program binds to the site's body |
| ch32vx03 | `Ch32vx03SleepSite` and `Ch32vx03TimedSleepSite` over `Pwr` (`ch32vx03/sleep.hpp`) | `none` and `light` are both Sleep (this family has nothing between Sleep and Stop), `standby` is a Stop on the main regulator, `deep` the same Stop on the low-power one, one mode with two prices (measured: the second costs 30 us more at the wake on the CH32V203C8T6 and 61 us on the CH32V303VCT6), so `armed()` answers from the mirror the site wrote and `deep` for a Stop somebody else armed; Standby is off the ladder and enforced off it, every `arm()` clearing PDDS, because every exit measured on this silicon is a power reset (the RTC alarm's line as an interrupt, the same line as an event alone, even a pad's line as an event, which the chapter does not list as an exit at all, and on the CH32V303VCT6 the independent watchdog's time-out); a SYSCLK restore rides `disarm()` (a Stop leaves the core on the HSI with the PLL off, and putting a PLL rate back takes 1.4 to 2 ms of crystal start-up) and the tick is paused by the platform's idle hook rather than by the site, because with this core's WFE idiom a merely pending tick would end the sleep before it began, at the price of one dropped tick, which the timed site's witness repairs. And the one thing no other target has: a sleep of any depth starves every bus master but the core here (measured on both parts), so the site refuses every rung but `none` while `bus_masters_active()` is not zero (a DMA channel of either controller while its EN is up and a USB controller from its pull-up to its detach each hold one count), and the kernel's idle path makes the same test before it sleeps at all, which is what turns "a program with USB never idles" from a rule into a mechanism. The timed site takes the RTC's counter as alarm and witness on a 1024 Hz tick out of the crystal, the counts rounded up and the span down so a wake is late and a resync short: a 500 ms deadline met at 502 ms (the CH32V203C8T6) and 501 ms (the CH32V303VCT6) of wall and of kernel tick |
| ch32x035 | none | not written: this series has no RTC and no low-speed oscillator, so the auto-wakeup on SYSCLK/1024 is the one wake a Stop or a Standby has, and the site is born with it; until then `idle()` is the plain Sleep and nothing in the stratum sets SLEEPDEEP ([the CH32X035 page](../ch32x035/README.md)) |
| rp2040 | `Rp2040SleepSite` and `Rp2040TimedSleepSite` (`rp2040/sleep.hpp`) | `light` is a WFI (which already reaches the chip's SLEEP state when core 1 sleeps too, measured), `standby` the SLEEP state with the program's SLEEP_ENx gates pruned, `deep` DORMANT on the crystal or on the ring oscillator, which is not a WFI, so the site installs the platform's `sleep_hook` and the idle path runs it: the clocks onto the oscillator, the PLL off, the keyword, the tree restored by `Clock::init()` after the wake; `arm(deep)` refuses with no way back configured (a GPIO event, or the RTC's alarm with the ring oscillator the one stopped and the crystal keeping clk_rtc), because the keyword would be the chip's last instruction; the tick (SysTick) counts through a standby, so nothing freezes there, and the timed site's timer alarm is the deadline's belt over that; a dormant freezes everything and the calendar is the alarm and the witness, in whole seconds |
| rp2350 | `Rp2350SleepSite` and `Rp2350TimedSleepSite` (`rp2350/sleep.hpp`) | `light` is a plain sleep instruction with every gate open, `standby` the chip's SLEEP state, which on this silicon is the program's SLEEP_ENx pruning set and no processor bit at all (no SLEEPDEEP takes part in its condition) plus the one thing the two architectures do not share: "a processor is asleep" is a signal the core releases, and a Cortex-M33 releases it on any WFI while a Hazard3 releases it only with MSLEEP.POWERDOWN set (measured: without it the pruned masks are never applied), so the standby rung arms `sleep_releases_power_request()` and each half's core header spells it; `deep` is DORMANT on the crystal or on the ring oscillator, not a sleep instruction, so the site installs the platform's `sleep_hook`, and `arm(deep)` refuses with no way back configured. The way back is where this chip surprises: the always-on timer's alarm ends a dormant only while clk_ref is still running (measured four ways over), so both sites move clk_sys to the oscillator they are about to stop through its own aux mux and leave clk_ref on one that keeps going, the low-power oscillator for a crystal dormant and the crystal for a ring-oscillator one; a GPIO event needs no clock at all and is the other way out. The P1.m states, where the switched core has no supply, are off the ladder and behind `Powman::power_down()`, because leaving one is a boot and not a wake: the only state that crosses is the always-on block's eight scratch words, and on the bench the debug port's rescue does not reach a powered-down core, so the driver's refusal without an armed wake is the only way back. The timed site takes the always-on timer as alarm and witness on every rung, milliseconds against a 1000 Hz tick, the deadline rounded up and the span down; the kernel timebase counts through a standby on both halves, so the resync has at most a tick to do there and the whole of a dormant afterwards ([the RP2350 sleep page](../rp2350/sleep.md), [the RP2350 power manager page](../rp2350/powman.md)) |
| host | none | the model's round, guard and locks are host-tested against a scripted site (`test_power`) |

Three things the realizations proved about the model, each with no
change to `util/power.hpp`. On the SAM C21 the vote round, the
unanimity rule, the `PowerLock` ceilings, the deadline guard and the
first-event-after-wake rule ran on the real kernel with two voters;
there the never-deeper mapping stopped being the identity (the
family's deepest stop is STANDBY, so `deep` maps to it and `armed()`
reports what was really taken), the duty the site section places on a
platform's idle path cost nothing (PM.SLEEPCFG already is the armed
mode, so the hook takes it by not touching it), and a target-level
restriction the model does not express appeared: the kernel tick stops
in standby, so a plain site may only arm standby with no time event
armed ([the SAM C21 platform page](../samc21/platform.md)).

On the STM32G0 the site's two functions absorbed everything that
family asks of a sleeper, a SYSCLK restore after a Stop (the part comes
back on HSI16, 5.3 of its manual) and a kernel-ticker pause across it,
with no new member of the concept. The first-event-after-wake rule is
more load-bearing there than anywhere: without the wake path's
`SleepRequested{none}` the clock would stay at 16 MHz and the tick
paused. Two frictions recorded, neither a contradiction: the
never-deeper rule can collapse two rungs onto one silicon state
(`light` maps to Sleep like `none`, because the only mode between
Sleep and Stop wants the whole program at 2 MHz), and `armed()` then
reports the shallower truth; and a mode the silicon has can be
unrepresentable in the ladder not because it is too deep but because
the program does not resume from it (Standby and Shutdown come back
through the reset vector, so no site can honour the
first-event-after-wake rule for them, and they stay reachable only as
a deliberate act outside the model,
[the STM32G0 power page](../stm32g0/pwr.md)). The timed site over the
RTC's wake-up timer lifts the tick-stops restriction there as it does
on the SAM: a 500 ms deadline through a Stop matures at 723 ms of wall
with the plain site and at 501 with the timed one, never early.

A third site on the same target showed what a witness close to the
tick's own rate demands. The RTC-backed timed site owns the whole RTC
(its prescaler split is its resolution), so a program that wants a
real calendar cannot use it; the STM32G0 grew a second timed site over
a low-power timer that counts through a Stop, with one peripheral as
both alarm and witness and the RTC left to the application, and the
model, the manager and the two older sites were not touched by a
line. Two timed sites can coexist on one target, differing only in the
peripheral each owns, and an application picks by what else it needs.
The rule it recorded, which the earlier sites never had to face
because their witnesses tick far finer than the kernel: **when a
witness ticks at close to the kernel's own rate, the integer
difference of two readings over-states the elapsed time by up to one
unit**, and a site that hands that difference to the ticker matures
events early (measured: a 500 ms deadline landing anywhere in 499 to
501 ms of wall). So a resync converts `elapsed - 1` witness units, an
alarm asks for one unit more than the deadline needs, and both errors
then land on the side the time contract allows. The same rule reaches
any site whose witness resolution approaches the tick, the RTC site at
its coarsest permitted split included, and a test of "never early" is
only fair if the deadline is armed on a tick edge, since a deadline
armed at an arbitrary phase is honestly N - 1 to N milliseconds away
([the STM32G0 power page](../stm32g0/pwr.md)).

## The ladder

`SleepDepth` is an ordered ladder of four rungs, shallowest first:

- `none`: not a sleep mode. It is the absence of a request: the
  kernel's own idle hook keeps behaving exactly as it always does,
  which is what every brio program already does when its queues run
  dry.
- `light`: the shallowest real stop the target has. Everything but the
  CPU keeps running; the wake-up list is complete.
- `standby`: clock domains are gated; only the peripherals that ask
  keep running, and only they can wake the machine.
- `deep`: the deepest stop. Almost nothing survives.

**A target need not have all four**, and the ones it lacks it maps to
the nearest shallower rung, never deeper than asked. That direction is
the whole safety of the rule: a portable application asking for `deep`
gets the deepest stop that machine really offers, and never one that
loses more than the application agreed to lose. Where a target does
have every rung the mapping is the identity, and the model still reads
back what was actually taken rather than what was asked.

## The site: arming and sleeping are two different acts

`SleepSite` is what a target must provide: `arm(depth)`, `disarm()`,
`armed()`. That is all, and in particular **not the sleep itself**.

The split is not an accident of one instruction set; it is the
load-bearing piece of the design. On every machine brio targets, the
mode lives in a register and the CPU stops when the idle path executes
its stop instruction. So the manager arms and returns; its dispatch
ends; the kernel loop finds every queue empty and calls the platform's
`idle()`; and that is the sleep. The power model needs no kernel hook
of its own, `Tenuto::run()` is untouched, and an application without a
power manager pays nothing.

The kernel's one optional idle hook is the timebase's, not the
model's, and the two do not know each other. A platform whose kernel
time runs on a counter that keeps counting while the core sleeps
offers `idle_until(deadline)`
([kernel.md, "Platform: what the machine provides"](kernel.md#11-platform-what-the-machine-provides-kernelplatformhpp)):
the loop hands it the nearest armed deadline and the platform places
its own wake there (the LPTIM compare on the STM32G0's tickless
platform), whatever depth a site has armed. It exists whether or not a
manager exists, the site still only arms, the vote round and the guard
below run unchanged, and the site's other duty on such a platform is
simply absent: there is no tick to pause and no time to resynchronize
([the STM32G0 power page](../stm32g0/pwr.md)).

The one duty this places on a target is stated in the `Platform`
contract's neighbourhood rather than in it: a platform's idle path
must let an already-armed deeper mode stand instead of imposing its
own shallow one. A platform that re-armed its default every turn would
silently undo the decision every stakeholder had just voted on.

## The round: unanimity among stakeholders

A sleep request is not an order; it is a question put to everyone with
something at stake:

1. someone posts `SleepRequested{depth, reply}` to the manager: an
   idle-detecting active object, a console command, a supervisor;
2. the manager clamps the depth (below), guards it (below), and posts
   `PrepareSleep{depth, reply}` to every **voter** in its list;
3. each voter answers `SleepVote{ok}`, always, exactly once;
4. **one not-ok ends the round.** Unanimity arms the site and the
   requester is told ok; anything else leaves nothing armed and the
   requester is told not-ok.

Why ask at all: a bus engine in the middle of a transfer, a converter
in the middle of a conversion, a driver whose oscillator is about to
stop, each holds a fact the requester does not have. Why unanimity
rather than a majority or a veto by priority: the cost of being wrong
is asymmetric. A refused sleep wastes power; an accepted one loses
data.

The manager does not time out. A voter that never replies stalls the
round, and that is a bug to fix, not a condition to recover from,
which is also why a voter's queue must have a slot for the vote
(`BusMaster` sizes one in, deliberately).

The requester learns the outcome through the same `SleepVote` a voter
sends. One vocabulary, both directions.

`BusMaster` (`util/bus_master.hpp`) is the first voter in the framework
and the model of the role: it answers ok only when nothing is in
flight and its pending FIFO is empty, because a transfer outlives the
dispatch that started it and completes on an interrupt a gated domain
would swallow.

## Standing restrictions: the stakeholders that cannot be asked

Voting works for stakeholders the manager can ask: active objects,
dispatched in the loop. A stakeholder that lives in an interrupt
handler cannot answer a `PrepareSleep`, and asking it afterwards would
be asking too late. It takes a `PowerLock` instead:

    static brio::PowerLock burst;
    burst = Pm::restrict(brio::SleepDepth::light);   // start of the burst
    ...
    burst.release();                                 // or scope exit

While a lock lives, no round arms deeper than the depth it names; the
effective ceiling is the shallowest live restriction. The counters are
kept under the platform's critical section, so both ends are safe from
an interrupt handler. The lock is an owned right: it cannot be copied,
it can be moved, and its destructor releases it.

A request deeper than the ceiling is clamped, not refused: the program
asked to stop and it stops, as far as it is currently allowed to. A
lock taken while a round is already armed applies from the next
request on; it does not retroactively disarm. That is not a gap,
because the interrupt that takes the lock is itself a wake, and the
first event after a wake disarms.

## The deadline guard

Leaving a deep mode is not free. Whatever the target, the bill is the
same shape: the clock source has to come back, and the supply has to
reach its working point. Stopping for less than that is a losing
trade, so a request for `standby` or deeper is refused, not clamped,
when the nearest armed time event is nearer than
`PowerConfig::min_deep_ticks`: nothing is armed, and the requester is
told not-ok through the same `SleepVote{false}` a failed round sends.

This is the one thing the model needs from the kernel, and it is a
question, not a scheduling decision: `TimeEvents<P>::ticks_to_next()`
walks the armed list with the same wrap-safe signed arithmetic
`process()` uses and answers "how long until the next thing this
program has to do", 0 if something is already due, nothing at all if
nothing is armed. Calling it changes no state.

The default, two ticks, is the worst wake-up bill measured on the
AVR128DB48 bench board (a crystal main clock's restart, under 2 ms,
[the AVR platform page](../avrxt/platform.md)) expressed in the
coarsest timebase brio runs on (1024 Hz). An application with a faster tick, a
cheaper wake or a different tolerance sets its own; the knob is a
compile-time configuration struct, so the comparison folds.

Nothing here guards `light`: the shallowest stop costs a handful of
cycles to leave and its wake-up list is complete.

## The first event after a wake disarms

The manager does not sleep, and it does not wake anyone. The machine
comes back on an interrupt; that interrupt's handler posts what it
always posts; and the manager's next dispatch, **of any event**, first
disarms the site and publishes `WakeReport{was}` before doing anything
else. Nothing polls, and the round ends exactly where the program
resumes.

Two consequences worth stating:

- A wake that has nothing to say to the manager leaves the mode armed,
  and the program stops again on the next empty turn. That is correct:
  the application's decision to be asleep has not been revisited.
- A wake path that does want the manager to know, without asking for
  anything new, says so with `SleepRequested{none}`: "awake, no new
  request". `none` is a no-op that still replies ok, so this is one
  posted event and no special case.

`WakeReport` reaches the voters whose event variant declares it. It is
a notification, not a subscription: a stakeholder is in the list
because it has a say in the decision, which is not the same as wanting
the news afterwards, and forcing the alternative into every voter's
variant would make a bus engine pay queue slots for a fact it has no
use for.

**A site can lift a target restriction with the model still
unchanged.** The SAM's restriction, standby only with no armed time
event because its tick freezes there, fell entirely inside the two
functions the concept always had: the timed site (`SamTimedSleepSite`)
places an RTC alarm on `ticks_to_next()` in `arm()` and catches the
tick counter up in `disarm()` (or in the alarm's own interrupt
handler), and the manager never learned a new word. The one thing the
site does lean on is already on this page: the after-a-wake rule, a
wake path with nothing to say sends `SleepRequested{none}`, which on
the AVR is a courtesy (the PIT tick re-wakes the machine regardless)
and with a timed site is load-bearing, since a woken program that
idles again without speaking re-enters a standby whose alarm may
already be spent. The site's own documentation carries that
obligation.

## What is deliberately not here

- **No idle detection.** Nothing in the model decides when the program
  has nothing to do; something has to post the request. That is
  application knowledge (a UI gone quiet, a shift ended, a command),
  and inventing a rule for it would be inventing an application. The
  loop's own `idle_until`, where a platform has it, decides how long an
  idle may last (the nearest deadline), never when the program is done
  or how deep it may stop: those stay the request's and the round's.
- **No per-driver policy for which peripherals run in standby.** Which
  peripherals must survive a given depth is a target-and-application
  matter; the drivers expose the flags, the voters know what they
  need, and the model carries the negotiation between them, not the
  answers.
- **No current numbers.** The model is about who decides and when.
  What a given depth actually saves is a bench-supply measurement,
  recorded in the target's own document.
