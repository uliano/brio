# Timer (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 4.6 (the
system timer: the 64-bit counter, the latching pair, the four alarms)
and 4.7.2 (the watchdog's tick generator it counts). The driver:
`brio/rp2040/timer.hpp` over `resets.hpp` and `watchdog.hpp`. The
reference suite: `test_rp2040_platform`, letter j, and every timing
letter of that suite, which this timer is the ruler of.

## What the silicon does

One 64-bit counter of microseconds - it cannot overflow in practice,
so it is monotonic - counting the nominal 1 us tick the watchdog block
derives from clk_ref (4.7.2): with clk_ref on the crystal, as
rp2040/clock.hpp puts it, the count is the crystal's own microseconds,
independent of clk_sys and of whatever the PLL does. Four alarms match
the LOW 32 bits (at most 2^32 us, about 71 minutes, ahead): writing an
ALARMn register arms it (ARMED's bit), the match clears ARMED and
raises INTR's bit - a level, cleared by writing 1 to it - and each
alarm has its own NVIC line, TIMER_IRQ_0..3. INTE masks, INTF forces,
INTS is the masked status. The 64-bit value is read either through
TIMELR/TIMEHR, where reading the low half LATCHES the high one until
it is read (right for one reader, wrong for two contexts), or through
the raw halves TIMERAWL/TIMERAWH with a re-read of the high half.
PAUSE stops the count; DBGPAUSE's two bits, SET AT RESET, stop it
while either core is halted by a debugger. The block is one of the
reset controller's, and nothing starts the tick but software.

## Types and verbs

- `Timer::init(clock)` - the block out of reset and the tick started
  at the crystal's megahertz (a crystal that is not a whole number of
  megahertz does not compile).
- `now()` (64 bits, the raw-pair discipline, any context), `now_low()`
  (the low half: 71 minutes of unsigned span, what a bracket wants),
  `now_latched()` (the latching pair, one context only).
- `alarm(n, at)`, `alarm_in(n, us)`, `armed(n)`, `disarm(n)`,
  `interrupt(n, on)`, `raised(n)`, `pending(n)`, `clear(n)`,
  `force(n, on)`, `irq(n)`.
- `pause(on)`, `paused()`, `debug_pause(core0, core1)`.

## How to use it

```cpp
brio::Timer::init(clock);                     // after SysClock::init()
const uint32_t t0 = brio::Timer::now_low();
...
const uint32_t took_us = brio::Timer::now_low() - t0;

extern "C" void isr_timer_0() { brio::Timer::clear(0); /* ... */ }
brio::Timer::interrupt(0, true);
brio::Nvic::enable(brio::Timer::irq(0));
brio::Timer::alarm_in(0, 2000u);              // fires in 2 ms, once
```

## Bench findings

- The tick starts at the crystal's 12 cycles and the count runs at
  1 us: 200 SysTick periods of clk_sys read 199999 us on it, 5 ppm
  from the PLL's exact ratio (`test_rp2040_platform` letter c).
- The raw-pair read and the latched read agree within a few
  microseconds; 2000 raw-pair reads never go backwards (letter j).
- An alarm set 2 ms ahead fires once, and its handler is entered
  3 us after the match; ARMED clears on the match; a disarmed alarm
  never fires; INTF forces the masked status with the counter
  untouched (letter j).
- DBGPAUSE is CLEARED by `init`: a core halted by a debugger no
  longer stops the count - with the bits at their reset value a BKPT
  taken by core 1 under a probe's leftover debug enable froze the
  ruler of core 0, whose 10 ms wait never ended
  ([multicore.md](multicore.md)). `debug_pause(true, true)` is the
  datasheet's default back, for a single-kernel program that wants a
  value read over SWD at a halt to be a stopped one.

## Not covered yet

Driver gaps, each with its reason:

- Writing the time (TIMEHW/TIMELW): the datasheet discourages it and
  nothing in brio wants a non-monotonic timebase; declined.
- A kernel timebase on this timer - a TICKLESS platform in the shape
  of the STM32G0's LPTIM one, with an alarm as the wake: born with the
  first program that wants no periodic interrupt; the alarm verbs are
  its building blocks.
- An alarm task (a `TimeEvent`-like one-shot over an alarm): born with
  its first user.

Implemented but not bench-verified, each with what would measure it:

- `pause()`: a `test_rp2040_clock` letter reading the count across a
  pause.
- The second core's reads of the raw pair while the first core reads
  too: the multicore suite times its crossings on core 0's reads
  alone; a letter with both cores reading.
