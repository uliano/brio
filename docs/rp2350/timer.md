# Timer (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 12.8 (the
system timers: the 64-bit counter, the latching pair, the four alarms,
and the two registers this chip added) and 8.5 (the tick generators that
feed them). Appendix E carries no erratum against this block. The driver:
`brio/rp2350/timer.hpp` over `clock.hpp` and `resets.hpp`. The reference
suite: `test_rp2350_timer`, letters b, c, d, e, f, g, h and k - and every
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
- **LOCKED** is described as disabling write access to the block. IT
  DISABLES NOTHING ON THIS STEPPING (measured, below): the bit takes its
  write and reads back set, and every register of the block goes on
  taking writes under it. What is true of it is the other half of the
  datasheet's sentence - only a reset takes it down - which is why `init`
  CYCLES this block's reset line instead of merely releasing it. No verb
  of the driver is gated on the bit, and a program must not gate one
  either.

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
- `locked()` and `lock()` - the bit written and read back, and no guard:
  the way to a block that has it set is `init(clock)` again.

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

## Bench findings

All of them on an RP2350 in the QFN-80 package, **stepping A2**, at
3.3 V, clk_sys on the PLL at 150 MHz and clk_ref on the board's 12 MHz
crystal, and all of them on BOTH architectures: `test_rp2350_timer`
reports **93 pass, 0 fail** on the Cortex-M33 pair and **93 pass, 0
fail** on the Hazard3 pair, from one source.

- **Three counters, one crystal.** Over a nominal 200 ms TIMER0 counts
  200003 us, TIMER1 200004 us and the SIO's platform timer 200003 us on
  the Arm half (200001 / 200001 / 200002 on the RISC-V one): three
  generators of the TICKS block dividing one clk_ref, agreeing to within
  4 us over 200 000. The crystal's own accuracy is nobody's verdict here
  - they all share it.
- **The 64-bit read.** The raw pair and the latching pair agree within a
  microsecond of each other on both instances, and 2000 raw-pair reads of
  each never go backwards. The high half is still zero, so the low half
  alone is an honest span for the 71 minutes it covers.
- **The eight alarms are eight interrupt lines**, TIMER0's 0..3 and
  TIMER1's 4..7, and each fires exactly once, clears ARMED by itself and
  is taken down by the handler's write-one. **The handler's entry after
  the match is 2 to 3 us on the Cortex-M33 and under 1 us on Hazard3** -
  the one number of this chapter that differs between the halves, and it
  differs in the RISC-V half's favour. The suite's ceiling is 20 us and
  both are far inside it. An alarm disarmed before its time never fires
  and leaves no raw flag; INTF forces the masked status without touching
  INTR.
- **PAUSE loses the time it stops.** A paused TIMER1 counts 0 us over a
  5 ms wait and takes up 5000 us over the next 5 ms: nothing is caught
  up, so the pause is missing from the count for good.
- **The counter on clk_sys.** With SOURCE on clk_sys TIMER1 counts
  149 996 to 150 000 per millisecond against the 150 000 its rate
  nominally gives, measured against the microsecond ruler over 20 ms -
  the residue being the slack of reading the two spans one after the
  other, not the silicon's. One write puts it back on the tick and it
  counts microseconds again.
- **The tick is what the counter counts.** Stop a system timer's
  generator and its counter stands still - 0 us over 5 ms - and
  restarting the generator at 12 cycles takes the count up again at
  5000 us per 5 ms. A divisor of zero, and one past the nine-bit field,
  are refused. Of the six generators only those the program starts run:
  with the kernel timebase elsewhere, neither core's SysTick generator is
  running on either architecture, while the two system timers', the
  watchdog's and the RISC-V platform timer's all divide clk_ref by 12.
- **LOCKED REFUSES NOTHING.** The bit takes its write and reads back
  **1**, and with it standing an alarm register, PAUSE, SOURCE and a
  register written through an atomic alias ALL TAKE THEIR WRITES; the
  counter goes on running (5001 us over 5 ms) and every read goes on
  answering. What 12.8 says of it is not what this stepping does. Cycling
  the block's reset line is what takes the bit down, and the counter
  comes back at zero on its microsecond tick.
- **DBGPAUSE** reads back both ways, and `init` clears both bits: a core
  halted in a debugger no longer freezes the ruler another core is
  waiting on.

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

Implemented but not bench-verified, each with what would measure it:

- Both instances on the QFN-60 package, which the stratum compiles for
  and no board of this bench carries.
- `lock()` against a bus master that is not the processor - a DMA channel
  writing the block, or a second core: the bit refuses nothing from the
  core that set it, and whether it refuses anything at all on this
  stepping would take one of those to say.
- The alarms under a clk_ref that is not 12 MHz: the arithmetic refuses a
  clk_ref that is not a whole number of megahertz, and only the board's
  crystal has been on the wire.
