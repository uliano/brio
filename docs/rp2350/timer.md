# Timer (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 12.8 (the
system timers: the 64-bit counter, the latching pair, the four alarms,
and the two registers this chip added) and 8.5 (the tick generators that
feed them). Appendix E carries no erratum against this block. The driver:
`brio/rp2350/timer.hpp` over `clock.hpp` and `resets.hpp`. The reference
suite: `test_rp2350_timer`, letters b, c, d, e, f, g, h - and every
timing letter of that suite, which `Timer<0>` is the ruler of.

## What the silicon does

TWO INSTANCES, TIMER0 and TIMER1, where the RP2040 had one: identical
hardware at two addresses, put there so that two security domains can
each own a timebase. Each is a 64-bit counter of microseconds - it cannot
overflow in practice, so it is monotonic - with four alarms that match
the LOW 32 bits, at most 2^32 us (about 71 minutes) ahead. Writing an
ALARMn register arms it (ARMED's bit), the match clears ARMED and raises
INTR's bit - a level, cleared by writing 1 to it - and each alarm has one
interrupt line of its own: TIMER0's are lines 0..3 and TIMER1's lines
4..7, one numbering for both architectures (3.8.4.2). INTE masks, INTF
forces, INTS is the masked status.

WHAT THE COUNTER COUNTS COMES FROM ANOTHER BLOCK. On the RP2040 the tick
was the watchdog's and was lent to the timer; here every consumer of a
timebase has its own generator in the TICKS block of 8.5, and each system
timer has one. A generator divides clk_ref by a CYCLE COUNT, so with
clk_ref on the crystal the counter reads microseconds whatever clk_sys is
doing - and stopping the generator stops the counter behind it, the tick
being what the counter counts rather than a clock the block owns. Nothing
starts a generator at boot but software.

The 64-bit value is read either through TIMELR/TIMEHR, where reading the
low half LATCHES the high one until it is read (right for one reader,
wrong for two contexts - and this chip has two cores), or through the raw
halves TIMERAWL/TIMERAWH with a re-read of the high half.

TWO REGISTERS THE RP2040 HAD NOT (12.8.1.1):

- **SOURCE** takes the counter off the tick and onto clk_sys cycles. It
  is not a timebase then - it moves with every rate change - but it is
  the finest counter this chip offers a program.
- **LOCKED** disables every write to the block, and IT CANNOT BE CLEARED:
  the datasheet's own "without a reset" means the subsystem reset
  controller, which is why `init` CYCLES this block's reset line instead
  of merely releasing it. Reads go on answering and the counter goes on
  running while it stands.

PAUSE stops the count where it is; it is taken up again from there, with
nothing caught up. DBGPAUSE's two bits, SET AT RESET, stop the count
while a debugger has a core halted - and a halted core is one of four
here, so with the bits set a breakpoint on one core freezes the ruler of
another, whose every timed wait then never ends.

## Types and verbs

- `Timer<n>` for n = 0, 1. A third instance does not compile.
  `alarm_count` is 4; `tick_consumer` and `reset_block` name this
  instance's generator and reset line, and `Timer<n>::Tick` is the
  generator itself.
- `init(clock)` - the block's reset CYCLED, DBGPAUSE cleared, SOURCE on
  the tick and the generator started at the crystal's megahertz. A
  clk_ref that is not a whole number of megahertz does not compile.
- `now()` (64 bits, the raw-pair discipline, any context), `now_low()`
  (the low half: 71 minutes of unsigned span, what a bracket wants),
  `now_latched()` (the latching pair, one context only).
- The alarms, with the index a TEMPLATE PARAMETER because an alarm is
  claimed at build time - what claims it is the app binding its vector by
  name: `alarm<a>(at)`, `alarm_in<a>(us)`, `alarm_at<a>()`, `armed<a>()`,
  `disarm<a>()`, `interrupt<a>(on)`, `raised<a>()`, `pending<a>()`,
  `clear<a>()`, `force<a>(on)`, `irq<a>()`. A fifth alarm does not
  compile.
- `pause(on)`, `paused()`, `debug_pause(core0, core1)`,
  `debug_paused(core)`.
- `TimerSource` (`tick`, `sysclk`) with `source(src)` and `source()`.
- `locked()` and `lock()` - the one-way verb, whose way back is
  `init(clock)` again.

## How to use it

```cpp
brio::Timer<0>::init(clock);                    // after SysClock::init()
const uint32_t t0 = brio::Timer<0>::now_low();
// ...
const uint32_t took_us = brio::Timer<0>::now_low() - t0;

extern "C" void isr_timer0_2() { brio::Timer<0>::clear<2>(); /* ... */ }
brio::Timer<0>::interrupt<2>(true);
brio::Irq::enable(brio::Timer<0>::irq<2>());
brio::Timer<0>::alarm_in<2>(2000u);             // fires in 2 ms, once

// The second instance as a cycle counter, while the first stays the ruler
brio::Timer<1>::source(brio::TimerSource::sysclk);
```

## Not covered yet

Driver gaps, each with its reason:

- Writing the time (TIMEHW/TIMELW): 12.8.2's own caution is that other
  software expects the value to increase monotonically, and nothing in
  brio wants a non-monotonic timebase; declined.
- A kernel timebase on a system timer - a TICKLESS platform in the shape
  of the STM32G0's LPTIM one, with an alarm as the wake: born with the
  first program that wants no periodic interrupt; the alarm verbs are its
  building blocks. On this target the RISC-V half's timebase already
  stands on a 64-bit comparator (`brio/rp2350/mtime.hpp`), which is the
  nearer road.
- An alarm task (a `TimeEvent`-like one-shot over an alarm): born with
  its first user.
- The security domains the second instance exists for (ACCESSCTRL, the
  secure and non-secure views of a peripheral): everything this stratum
  runs is Secure, as the bootrom hands over, so the second timer is a
  second ruler and nothing more.

Implemented but not bench-verified, each with the letter of
`test_rp2350_timer` that will measure it:

- The two counters against each other and against the SIO's platform
  timer over 200 ms, three generators of one clk_ref (letter b).
- The raw-pair read against the latching pair, and 2000 reads of each
  instance that never go backwards (letter c).
- The four alarms of TIMER0 and the four of TIMER1, each on its own
  interrupt line, with the handler's entry latency after the match, the
  disarm before the time, and INTF forcing the masked status without
  touching INTR (letters d and e).
- PAUSE stopping the count and taking it up where it stopped, and
  DBGPAUSE's two bits cleared by `init` (letter f).
- SOURCE on clk_sys: the counts per microsecond against the ruler, which
  is clk_sys in megahertz (letter g).
- A tick generator stopped and restarted with its counter watched, and
  the refusal of a divisor of zero or one past the nine-bit field
  (letter h).
- LOCKED refusing a write in silence while the counter runs on, and
  `init` as the one way back (letter k).
- Both instances on the QFN-60 package, which the stratum compiles for
  and no board of this bench carries.
