# Platform - what the kernel stands on (STM32G0)

This document is for a programmer putting a brio program on an STM32G0.
It describes what the kernel stands on here, the platform, and the two
timebases a program can run its time events on. A brio kernel asks the
chip for a handful of things, stated by the kernel's `Platform` concept
(a concept is a compiler-checked set of requirements on a type;
[design/kernel.md, "Platform: what the machine provides"](../design/kernel.md#11-platform-what-the-machine-provides-kernelplatformhpp)):
a way to mask interrupts for a critical section, a way to sleep until
the next interrupt without losing one that arrives at the wrong moment,
a source of time, a breakpoint for a panic, and a record a panic leaves
for the next boot. `Stm32g0Platform<TB>` is this family's answer, and
`TB` is its timebase: the SysTick `Ticker`, a 1000 Hz interrupt that
stops when the chip enters a Stop mode, or `LptimTicker`, the low-power
timer on the 32.768 kHz crystal, which keeps counting through a Stop and
lets the kernel sleep straight to its next deadline with no periodic
interrupt at all (a tickless program).

The smallest use is the SysTick timebase: the lines of `blink`'s target
glue that concern the platform, quoted from the file
([the first program](../learn/first-program.md)):

```cpp
using P = brio::Stm32g0Platform<>;
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }  // tick + idle wakeup
...
    SysClock::init();             // HSI16 -> PLL -> SYSCLK = 64 MHz
    brio::Ticker::init(clock);    // SysTick timebase (runs through WFI sleep)
    brio::enable_interrupts();
    brio::Tenuto<P, Blinker, Supervisor>::run();  // never returns
```

Three things decide whether that works. The clock task must have run
first (`SysClock::init()`, [clock.md](clock.md)): the ticker computes
its reload from the clock's rate. The SysTick vector must be bound to
`Ticker::tick()` by the program, as above; the kernel binds no vector,
and a program that forgets it compiles, runs and never advances time.
And interrupts must be enabled before `run()`
(`brio::enable_interrupts()`), since the kernel sleeps between events
and the tick is what brings it back. On the tickless timebase the
crystal must be running: `LptimTicker::init()` returns false when it is
not, and a program that ignores that answer has a kernel with no time.

The rest of this page, in order: the documents of record; what the
silicon does (the interrupt masking, the vector table and its shared
lines, the idle hook, the two timebases, the SRAM parity option, the
breakpoint); the types and verbs; how to use it, with the tickless
variant; the bench findings, measured to the cycle; "Code in SRAM", an
opt-in placement of two drivers' hot paths; what the two smaller parts
change; and what is not covered.

## Documents of record

RM0444 Rev 6 - the Cortex-M0+ summary ch. 12
(NVIC) with ARM's ARMv6-M ARM behind it, and PWR ch. 4 for what a WFI
here really enters ([pwr.md](pwr.md) owns that chapter) - and errata
ES0548 Rev 3
(2.8.2 and 26.4.11 shape the tickless timebase's arming; nothing else
touches this chapter on revision Z). Drivers:
`stm32g0/platform.hpp` (`Stm32g0Platform<TB>`, this target's
realization of the kernel's `Platform` concept, templated on its
timebase, and the `Tickless` concept), `stm32g0/ticker.hpp` (the
SysTick `Ticker`: this family's include of the core stratum's
`cortexm/ticker.hpp` - `BasicTicker`, `SysTickCounter`),
`stm32g0/lptim_ticker.hpp` (`LptimTicker`, the tickless timebase over
[lptim.md](lptim.md)'s driver), `stm32g0/delay.hpp` (the microsecond
busy-wait), `stm32g0/reset.hpp` ([reset.md](reset.md)),
`stm32g0/nvic.hpp` (`cortexm/nvic.hpp` - `InterruptGuard`, `Nvic`;
[../cortexm/README.md](../cortexm/README.md)). The
crt is `stm32g0/src/glue/startup_stm32g0b1.cpp` +
`stm32g0/ld/stm32g0b1re.ld` in the build project. The family fixtures
are `test/family_stm32g0/platform.cpp`, `lptim_ticker.cpp` and
`rubato.cpp` (the preemptive platform under the Rubato kernel) under
`brio check stm32g0`, with the negatives that refuse a timed sleep
site on a tickless platform, the tickless timebase off the value
line and the Rubato kernel on the Tenuto platform.
`Stm32g0RubatoPlatform<TB>`, in the same header, is the platform of the
preemptive kernel over the core stratum's `cortexm/schedule.hpp` (the
ARMv6-M ARM's PendSV and SVCall; the Cortex-M0+ TRM, DDI 0484C, for
the exception's cycles).

## What the silicon does

**Interrupt masking is PRIMASK, and nothing finer.** ARMv6-M has no
BASEPRI: masking is all-or-nothing, one bit, saved and restored. The
NVIC orders PREEMPTION between handlers with two priority bits (four
levels, 0 most urgent); it never limits the reach of a critical
section. Core exceptions (SysTick among them) have no NVIC enable bit.

**The vector table is 16 + 32 entries and shared lines are the rule.**
RM0444 table 61 gives 32 positions to more peripherals than that:
USART2 shares position 28 with LPUART2, USART3/4/5/6 share 29 with
LPUART1, TIM3 with TIM4, TIM6 with DAC and LPTIM1, and so on. A
handler for a shared line asks each of its peripherals in turn; the
device header gives the line's IRQn (`USART2_LPUART2_IRQn`) and ST's
startup template the handler's spelling (`USART2_LPUART2_IRQHandler`)
- the header declares NO handler names, so the crt cites the template
for that one thing. The vector table lives at the start of flash
(0x0800_0000) and is fetched through the boot alias at 0; VTOR exists
on this core and is not written.

**WFI wakes on a pending interrupt even under PRIMASK**, so the idle
hook sleeps first and unmasks after, closing the lost-wakeup window by
construction - and a WFI is not an event latch: an interrupt that wakes
it leaves nothing behind for the next one, so the loop turns once per
interrupt. Both are measured to the cycle (letters `w` and `k`, below). What the WFI enters is WHATEVER IS ARMED: SCR.SLEEPDEEP is
0 out of reset and this file never writes it, so a bare WFI is Sleep -
HCLK, SysTick and every peripheral keep running (5.3) - and with
`stm32g0/sleep.hpp`'s site having armed a Stop, the same WFI is that
Stop. **The idle hook knows nothing of the depth**, and that is worth
stating beside the other two targets: the AVR's `idle()` must honour a
standing SEN bit and the SAM C21's carries an erratum guard, while this
hook writes neither SCR.SLEEPDEEP nor PWR_CR1.LPMS - this silicon
selects the depth there, and the arming is the sleep site's business
alone.

**SysTick rides HCLK.** The reload is `Clock::hz / 1000 - 1`
(63999 at 64 MHz, read back over SWD); in Stop the core clocks stop and
so does the kernel's time - the samc21 standby situation, and it has the
same two answers here: `Stm32g0SleepSite` keeps the honest restriction and
`Stm32g0TimedSleepSite` lifts it, resynchronizing from the RTC through
`advance()` ([pwr.md](pwr.md)). The tick itself does not defeat a deep
sleep, though a debugger can make it look so: 4.3.3 makes a WFI a no-op
with an interrupt pending, but a Stop once taken stops HCLK and SysTick
with it, and a 250 ms Stop asked for with a 1 kHz SysTick armed lasts
its 250 ms (measured). What cuts it to 1 ms is `DBGMCU_CR.DBG_STOP`,
which keeps HCLK running inside a Stop for the debug logic's sake -
and OpenOCD sets that bit at every connection that finds the DBGMCU's
clock gate open. The site pauses the ticker for the deep rungs anyway
and resumes it on disarm: it costs nothing and makes the Stop last in
both states ([pwr.md](pwr.md)).

**And a timebase that does not stop: the LPTIM on the crystal.** The
platform is ONE class templated on its timebase, `Stm32g0Platform<TB =
Ticker>`, and the other argument is `LptimTicker<>`: LPTIM1 clocked
from the 32768 Hz LSE undivided, which keeps counting through Stop 0
and Stop 1 (26.5), with the kernel tick its count shifted right by
five - 1024 ticks a second, a power of two, so `millis()` is shifts
and exact. A program on it is tickless: no periodic interrupt at all,
the kernel's optional `idle_until(deadline)` hook
([design/kernel.md, "Platform: what the machine provides"](../design/kernel.md#11-platform-what-the-machine-provides-kernelplatformhpp))
places the loop's next deadline in the LPTIM's compare and one WFI
covers the whole wait at whatever depth a sleep site armed, kernel time
runs through the Stop, and the plain sleep site is the only site it takes
(the timed ones refuse it at compile time: nothing stands still). What
it gives up: one LPTIM and its vector, bound by the app to the ticker's
`isr()`; SysTick keeps counting interrupt-less as `delay_us`'s cycle
counter (`SysTickCounter`), and nothing is bound to `SysTick_Handler`.
The compare's arming is where the silicon and two errata dictate the
shape - a write lands 2..3 counts after the store, a match fires at
the count edge AFTER equality, ES0548 2.8.2 forbids clearing a flag
outside the handler and 26.4.11 a second write before the first one's
CMPOK - and the four rules that follow are in `lptim_ticker.hpp`'s
header and measured below.

**SRAM survival across a reset is promised nowhere**, and the SRAM has
a parity check the factory option byte leaves disabled
(FLASH_OPTR.RAM_PARITY_CHECK = 1): with it enabled, the first read of
a never-written word - the panic breadcrumb's, after a power-on -
would raise an NMI. The breadcrumb's magic word is what makes the
default case harmless; the option-byte pass has to remember the other.

**A BKPT with C_DEBUGEN set halts the core in silence**, as on the
samc21: `break_here()` cannot ask whether a debugger is attached
(ARMv6-M), so `bin/brio` clears DHCSR after every flash and a
BKPT with no debugger escalates to the crt's distinct
`HardFault_Handler` spin.

## Types and verbs

- `Stm32g0Platform<TB = Ticker>` - `Timebase` (= TB), `CriticalSection`
  (= `InterruptGuard`), `idle()` (DSB, WFI, unmask),
  `interrupts_enabled()`, `in_handler()` (kernel/platform.hpp's optional
  member: IPSR's exception number, zero in thread mode, ARMv6-M ARM
  B1.4.2, cortexm/nvic.hpp's read), `break_here()` (BKPT), `now()`/
  `ticks_per_second` (the timebase's), `atomic_width` 4,
  `places_fast_text` (the three linker scripts collect `.fast_text`, so
  a driver may be given util/placement.hpp's `InRam` - "Code in SRAM"
  below), `panic_record()` in `.noinit` - and, on a `Tickless` timebase only,
  `idle_until(std::optional<uint32_t> deadline)`: the kernel's optional
  hook, entered masked with the absolute tick of the nearest armed time
  event; a due deadline or a wake the timebase could not place this turn
  means no sleep (interrupts back on, return), otherwise the wake is
  placed and the same DSB/WFI/unmask follows. `Tickless<TB>` requires
  `ticks()`, `ticks_per_second`, `arm_wake(now, deadline) -> bool`; the
  SysTick `Ticker` is statically not one.
- `Stm32g0RubatoPlatform<TB = Ticker>` - `Stm32g0Platform<TB>` whole
  (`idle_until()` with it where `TB` is `Tickless`) plus the three
  members of a preemptive platform, the one the preemptive kernel
  `Rubato` runs on ([design/kernel.md](../design/kernel.md), "Rubato"):
  `preemptive`, `ceiling` (the index of the object being dispatched, 255
  at the loop's base) and `pend_schedule()`, which is
  `cortexm/schedule.hpp`'s request: PendSV pended, at the lowest of the
  core's four levels (the byte 0xC0), below every NVIC line at its reset
  priority 0. PendSV returns into the scheduler in thread mode and the
  scheduler leaves through SVC, so a program on this platform binds two
  vectors more than a Tenuto one, `PendSV_Handler` and `SVC_Handler`
  (the crt's weak aliases), both naked, to `scheduler_exception<K>()`
  and `scheduler_return()`, calls `scheduler_init()` before it enables
  interrupts, and executes no `svc` of its own. The critical section
  stays PRIMASK, and a post made inside one serves its receiver when the
  section ends; the peripheral handlers still never nest among
  themselves.
- `LptimTicker<cfg>` (`LptimTickerConfig{instance = 1, shift = 5,
  source = lse, lsi_hz = 34000}`) - `init(clock)` (the LSE through the
  RTC domain's gate, or LSI through RCC; the LPTIM undivided with ARRM
  and CMPM armed, the compare parked on the lap, the wake line and the
  vector open, the counter started, SysTick as the cycle counter; false
  when the oscillator never came), `ticks()` (the count shifted; exact
  inside a masked window under one second), `count()` (the 32-bit
  count itself), `millis()`/`secs()`/`now()` (exact by shifts on the
  crystal, two divisions on LSI), `arm_wake(now, deadline)` (the four
  rules: store only with CMPOK clear and no write in flight, else pend
  the sweep once and decline; a deadline nearer than six counts is
  declined and spun through; before a Stop a store is not slept on
  until it has landed - every turn before CMPOK is declined, with
  interrupts back on, so nothing in the ticker waits masked; a
  deadline a lap or more away parks the compare on the lap and the
  register is mirrored), `park()` (no deadline at all: the compare on
  the lap - what the platform's `idle_until(nullopt)` calls), `isr()`
  (the ordered clear, the lap carry, the completion note; returns the
  mask served), the readbacks `laps`/`cmp_reg`/`write_pending`/
  `deferrals`/`floor_declines`/`stores`/`write_timeouts`/`stop_waits`,
  `irq()`; `on_crystal`, `count_hz`. `shift` 0..10 (32768 down to 32
  ticks a second), a lap of two seconds at every shift and on either
  clock. **LSI is a stated rate**: `source = lsi` takes `lsi_hz`,
  refused outside DS13560 table 46's 29.5..34 kHz, and the DIRECTIONAL
  RULE picks what to state - a tick is exact by count, the statement
  governs only the conversion of milliseconds into ticks, and N ticks
  of a clock faster than stated are fewer real milliseconds than asked,
  i.e. EARLY; so the statement must not sit below the true rate, the
  default is the band's ceiling (never early on any part, up to 15 %
  late on a slow one), and a program that has measured its LSI (the
  rtc suite's TIM16 capture: 32586 Hz on the bench die) states that.
- `InterruptGuard`, `enable_interrupts()`, `disable_interrupts()`,
  `interrupts_enabled()`, `irq_priority_levels` (4) - cortexm/nvic.hpp
  through stm32g0/nvic.hpp.
- `Nvic` - `enable`/`disable`/`enabled`, `set_pending`/`clear_pending`/
  `pending`, `priority` (refuses a level the core does not have), by
  the header's IRQn value.
- `BasicTicker<tps>` / `Ticker` (1000 Hz) - `init(clock)` (false when
  the reload does not fit 24 bits), `tick()` (the ISR body), `ticks`/
  `millis`/`secs`/`now`, `advance(n)` (the RTC resync's landing point -
  `Stm32g0TimedSleepSite` is its user), `pause`/`resume` (which the same
  site calls around every deep sleep). A rate that does not divide 1000
  is refused at compile time. `SysTickCounter` - `start(clock)`/`stop()`/
  `running()`: SysTick counting with no interrupt, the same reload, for
  a program whose timebase is the LPTIM (`LptimTicker::init` calls it).
- `delay_us(clock, us)` / `delay_us(DelayRate, us)` +
  `delay_rate(hz)` - stm32g0/delay.hpp, which is `cortexm/delay.hpp`
  plus this family's measured facts: a busy-wait of AT LEAST `us`
  microseconds on SysTick's VAL, CAPPED BELOW ONE SYSTICK PERIOD - one
  millisecond, one kernel tick on the SysTick timebase and 1.024 on the
  LPTIM one (a millisecond or more is TimeEvent territory and is
  refused with `false`, spending nothing), and `false` with no time
  spent when SysTick is not running at all. No division runs at wait
  time: the cycles-per-microsecond factor is rounded UP at compile time
  so every error lands late. SysTick's one writer (the ticker, or the
  counter) owns it IN WRITING; this file only reads VAL, and folds the
  reload across a wrap, so it is correct inside a masked window too,
  and needs the counter running, never its interrupt.
- The crt: `Reset_Handler` (copy .data, zero .bss, walk .init_array,
  call main; .noinit untouched), weak `Default_Handler` aliases for
  every vector, a distinct weak `HardFault_Handler`, `abort()` as a
  spin.

## How to use it

```cpp
using P = brio::Stm32g0Platform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 64'000'000>;
constexpr SysClock clock;

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main() {
    SysClock::init();
    brio::Ticker::init(clock);
    brio::enable_interrupts();
    brio::Tenuto<P, Blinker, Supervisor>::run();
}
```

The same program TICKLESS - the timebase as the platform's argument,
the LPTIM's vector instead of SysTick's, nothing else:

```cpp
using Tb = brio::LptimTicker<>;                  // LPTIM1 on LSE, 1024 Hz
using P = brio::Stm32g0Platform<Tb>;
using Site = brio::Stm32g0SleepSite<SysClock, Tb>;   // the only site it takes

extern "C" void TIM6_DAC_LPTIM1_IRQHandler() { Tb::isr(); }

int main() {
    SysClock::init();
    const bool tick_ok = Tb::init(clock);        // false: no crystal
    brio::enable_interrupts();
    brio::Tenuto<P, Blinker, Supervisor>::run(); // sleeps TO each deadline
}
```

The same program under the preemptive kernel: the platform type, the
two naked vectors and one call more.

```cpp
#include "kernel/rubato.hpp"

using P = brio::Stm32g0RubatoPlatform<>;
using K = brio::Rubato<P, Sensor, Logger>;   // Sensor preempts Logger

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" [[gnu::naked]] void PendSV_Handler() { brio::scheduler_exception<K>(); }
extern "C" [[gnu::naked]] void SVC_Handler() { brio::scheduler_return(); }

int main() {
    SysClock::init();
    brio::Ticker::init(clock);
    brio::scheduler_init();         // PendSV at the lowest priority
    brio::enable_interrupts();
    K::run();
}
```

## Bench findings

The reference suite is `test_stm32_platform` (ten letters in `z`, 71
verdicts; letters `i` and `j` outside it reboot the board six times
each - [reset.md](reset.md) carries the reset and watchdog half). A
letter is one command key of the suite's console and the group of
verdicts it runs; `z` is the key that runs them all.
What it measures of THIS chapter:

- **PRIMASK nesting is nesting**: leaving an INNER critical section does
  not unmask, because the guard saves and restores the bit rather than
  clearing it.
- **The NVIC's two priority bits order and preempt** (letter `n`, on
  FLASH and I2C1, two lines pended by software): the field holds levels
  0..3 and `priority()` refuses a fifth with the level standing; two
  lines pending at one level are served lower number first, each
  handler to its end; with the higher-numbered line the more urgent, it
  is served first; and a more urgent line pended from INSIDE a handler
  preempts it, where one at the same level waits for it to return -
  which is what every brio program runs with, every line left at its
  reset level 0 (ten runs of `z`).
- **A masked window loses ticks and does not lose time**: 5 ms with
  interrupts masked advances the tick by 1 ms - SysTick's interrupt is
  a pending BIT, so the coalesced ticks are gone, and a timebase that
  must survive long masked windows needs more than the pending bit.
- **`idle()` sleeps and the tick brings it back**: with the console
  drained, ONE `idle()` call covers the 960..990 us to the next tick,
  and it returns with interrupts enabled. (With the console still
  draining it returns in 85 us on the USART's own interrupt, which is
  the hook working, not failing.)
- **No edge position loses the wake** (letter `w`): SysTick's next edge
  placed D cycles after a write, D = 1..400 - LOAD set to D around the
  VAL write that reloads it and put back at once, every placement
  checked - and the kernel's shape run against it: a masked check,
  `idle()`, until one tick, until two (the second edge a whole period
  later, the sleep entered after a tick was taken), and a third pass with
  the edge on an NVIC LINE (TIM14's compare on HCLK, the tick's interrupt
  held off so that a lost wake would be lost for good). TIM2 at 1 MHz
  times each try and rescues a lost one: 0 of 400 lost in each pass and
  none late - the slowest try 9 us for one tick, 1009 for two. The walk
  crosses the sleep: from D = 39 (the tick) and D = 24 (the line) on,
  the edge found the core asleep; every nearer edge came before the WFI,
  pending under PRIMASK, and the WFI fell through on it.
- **What a wake costs**, in HCLK cycles at 64 MHz (two flash wait
  states, the prefetch and the cache on): an edge that finds the core
  asleep enters the tick's handler at 36 cycles against 31 when the core
  is awake and spinning unmasked, reaches the handler's last statement
  at 69 and the caller's loop at 135 (an awake spin is out at 95); a line
  edge enters at 36 against 31 and reaches the loop at 113 against 88.
  The handler's first statement is a VAL or count read, so each entry
  figure carries the instructions before that read. An `idle()` that a
  pending line returns at once costs 85 cycles, the line's handler round
  trip included. The idle path is the masked check and `idle()`'s three
  instructions, DSB, WFI and CPSIE (a call of its own in the suite's
listing) - ST's
  `HAL_PWR_EnterSLEEPMode` is a bare WFI run unmasked, the lost-wake
  window left to its caller.
- **One kernel turn per interrupt** (letter `k`): a Tenuto pack of three
  quiet AOs, two with a periodic time event, turned as `run()` turns it
  with the tick the only interrupt: 100 turns over 100 ticks. A quiet
  turn whose `idle()` a pending line returns at once costs 281 cycles,
  85 of them that `idle()` and its handler and 196 the turn's own -
  `process()`, `step()` over three empty queues, the masked check.
- **SysTick's VAL arithmetic is exact enough to build a delay on**:
  accumulating VAL deltas with the reload folded in across a wrap tracks
  the INTERRUPT count to 350..3100 ppm over 200 ticks (12.8 M cycles) -
  the residue is the loop's own start and end boundaries, and an error
  in the wrap handling would be a whole period.
- **`delay_us` is at least, never early**: 5 / 30 / 100 / 500 / 900 us
  measure 6 / 32 / 102 / 501 / 902; two hundred 50 us waits land at
  51..53 us with NOT ONE below 50, whatever SysTick phase they start
  in; a thousand 100 us waits are 101 ms of kernel tick against 100
  due, so the per-call overhead is about 1%. The CAP is real: 1000 us
  is refused in 1 us and 999 us is served, and with SysTick stopped the
  answer is `false` in 0 us.


**The tickless timebase** - the reference suite is `test_stm32_tickless`
(nine letters in `z`, 47 verdicts; `u` outside it needs a keystroke and
`x` is the diagnostic that measures the latencies). What it measures:

- **No periodic interrupt**: `SysTick_Handler`, bound on purpose, never
  runs; SysTick counts with TICKINT clear and the 1 ms reload; the
  LPTIM raises one interrupt per deadline and ONE per two-second lap -
  the compare parked at 0xFFFF matches at the counter's own wrap and
  its CMPM rides the ARRM's interrupt. Parked mid-lap it would be a
  SECOND wake every lap: a ten-second Stop costs ten interrupts there
  where parking on the lap costs six - five laps and the deadline.
- **What a rare-event program pays for the lap**: ten seconds in Stop 1
  with one deadline at the end cost five lap wakes, every one on HSISYS
  with the PLL never re-locked (no AO ran, the manager was never asked:
  the loop found nothing to do and went straight back to the WFI), and
  each kept the part awake for 169..173 us measured ISR to ISR on TIM2
  clocked from MCO = HSI16/8 - a clock that runs only while the part is
  awake - that is the Stop exit, the handler, one empty loop turn and
  the WFI, at 16 MHz with the PLL's two wait states and the prefetch
  still in FLASH_ACR (201..203 us with the prefetch off); 1.5 ms awake
  in the whole ten seconds, a 10^-4 duty. It
  is the price of a 16-bit counter that must not be divided (below),
  and it does not grow with how rare the events are.
- **The rate and the arithmetic**: 2 s of TIM2 on the PLL move the tick
  by 2043 (2048 due: HSI16 against the crystal, coherence not
  metrology) and `millis()` by 1995; `secs()`/`now()` are the shifts
  they claim; 200000 reads never step backwards.
- **The lap carry, under a mask too**: a second `LptimTicker` on LPTIM2
  at shift 0 (the crystal's own rate, a lap of two seconds - LPTIM2's
  counter on silicon for the first time) is monotonic across wraps in
  a tight loop, and with PRIMASK held across a wrap the read under the
  mask already carries the lap (ARRM standing, the handler not yet
  run): the unmasked read follows it by one count, not by 65536.
- **A compare equal to ARR matches** like any other value - 0xFFFF
  needs no special case.
- **THE LATENCIES SCALE WITH THE COUNTER'S CLOCK, NOT THE KERNEL
  CLOCK**: a CMP write reaches CMPOK
  in 74..88 us on the undivided counter (2..3 counts, lptim.md's
  figure) and in 2.0..2.8 ms at prescaler /32 (2..3 PRESCALED counts,
  a match tried three counts out is missed for a whole lap), and CMPM
  fires at the count edge AFTER equality (k = 4, 5, 6, 10, 40 counts
  out all wake at CMP + 1; two counts out lands at CMP + 2, the write
  having arrived on top of the match). Hence the undivided counter
  with a shifted tick, the compare at the deadline's first count LESS
  ONE, and the six-count floor.
- **`idle_until` by hand**: 2 / 3 / 10 / 100 / 500 ticks asked sleep
  1956 / 2938 / 9786 / 97862 / 489276 us on TIM2's scale (1953 / 2930 /
  9766 / 97656 / 488281 nominal, the PLL 0.23 % fast against the
  crystal), one CMPM each, `ticks()` at return equal to the deadline
  every time; a deadline one tick away sleeps 980 us and lands on it;
  a due deadline returns in 7 us with no interrupt; with nothing armed
  the sleep lasts until a foreign wake (the RTC's wake-up timer,
  232 ms) and the LPTIM never speaks.
- **The handshake the errata force**: two arms 150 us apart - the
  second finds CMPOK standing, pends the vector once and declines; the
  handler's clear is visible on the APB side in 0 us; the third arm
  stores the new value (two stores, one completion between them).
- **A store immediately before a Stop 1 LANDS with PCLK stopped**: the
  compare fires at its 292 ms on the RTC's wall with no wait at all,
  three runs of three - the APB-to-kernel transfer completes on the
  kernel clock alone, and the ticker's wait before a Stop (rule 3) is
  insurance the silicon does not need, kept at 72..93 us awake a round
  - spent in declined loop turns with interrupts enabled, not masked.
- **A real kernel**: three periodics at 7, 30 and 250 ticks pumped
  through `process()/step()/idle_if_empty()` for three seconds fire
  438 / 102 / 12 times, every interval exactly 7 / 30 / 250 ticks in
  the timebase's own units (6829..6871 / 29326..29384 / 244543..244712
  us on TIM2's scale), NOT ONE EARLY, and the LPTIM woke the loop 535
  times for 534 CMPM (one interrupt served a deadline and a lap
  together) with no deferral: the kernel slept TO its deadlines.
- **Stop 1 through the manager with the plain site**: a 500-tick
  deadline matures after 488 ms of RTC wall with 500 kernel ticks
  elapsed - time RAN through the Stop, no resync, no timed site - one
  LPTIM interrupt, the round closed by the convention and SYSCLK back
  on the PLL; six rounds of 150 ticks all at 146 ms of wall (146.5
  nominal) and not one early.
- **`delay_us` on the interrupt-less SysTick**: 5 / 30 / 100 / 500 /
  900 us measure 6 / 31 / 101 / 501 / 901 on TIM2; 200 x 50 us never
  below 51; 1000 us refused, 999 served.
- **An LSI-clocked ticker** on LPTIM2 at the stated 32586 Hz (shift 5:
  1018 ticks a second by the statement): 2 s of the crystal count 2041
  of its ticks (2036 by the statement) and 2004 of its milliseconds,
  the oscillator's own 0.2 % that day; a 100-tick wake placed on it
  lands at its tick, never early in its own units, one interrupt,
  97.6 ms on TIM2; and the default statement (34000) would make every
  millisecond asked land 43 per mille late on this die, never early -
  the directional rule's price, paid by a program that does not
  measure.
- **What a two-second lap asks of a suite over this timebase**: a leg
  that judges "one interrupt" or "the LPTIM never spoke" over a window
  under a second, or that expects CMPOK unswept between two arms, is
  wrong once in four to eight runs when a lap wraps inside it - the
  handler runs for the carry and the loop turns once more, which costs
  a program nothing and a naive verdict its truth; such legs wait for
  the first half of a lap. And a print in flight when a Stop is
  entered is a garbled line (the console's clock stops mid-character):
  drain first.

**The preemptive kernel** - the reference suite is `test_stm32_rubato`
(nine letters in `z`, 39 verdicts, every one green in ten runs of
`z`): a Rubato pack
of five objects on `Stm32g0RubatoPlatform` on the Nucleo-G0B1RE at
64 MHz, the Console the lowest and the letters' runner, TIM14's compare
the interrupt the road-1 letters land, nothing wired and nothing lit.
The ruler is TIM2 free-running at HCLK: two back-to-back reads of its
CNT are 4 cycles apart, against 65 for the kernel ticker's composed
`cycles()` (SysTick's position and the tick count, five reads and a
consistency check), and the two agree within a cycle over a millisecond
and a reload. HCLK cycles, each figure beside the bound its letter
derives from the core's own counts over the release listing:

- **The ceiling, both roads** (letters a, b, e): an interrupt's post to
  High inside Low's handler runs High to its end before Low resumes,
  every handler reading its own index as the ceiling; the UART's
  interrupt reads the loop's ceiling as the letter's key arrives, and
  the verdict accepts both values the loop holds below every object -
  255 at its base, 0 while its base turn walks the time events
  (kernel/rubato.hpp holds it there so that a timer's fire only
  queues; no dispatch of Top, whose index is also 0, can be open then,
  nothing in the suite being able to post to Top when a key arrives) -
  and read 255 in every run, twenty of letter `a` alone and ten of
  `z`; Low posting to Mid and Mid to High nests High inside Mid inside
  Low, each inside the post that asked, High's post to Low served
  after Low's own dispatch ends; and at depth 2 an
  interrupt over High's handler, started by either road, runs Top
  before High resumes, Top's first statement 188 to 189 cycles after
  the handler's last (one set of ten events in five of the twenty sets
  of ten runs reaching 234 to 252, by either road, unattributed: the
  letter reads no tick around them).
- **The latency of road 1** (letter c): from the interrupt handler's
  last statement to High's handler's first, 204 cycles min, 205 median
  and max over 100 events, every run; 251 to 267 for the one event in
  about a hundred and fifty a kernel tick lands in (seven of the
  thousand events of ten runs, a tick inside each). The path counts 161
  cycles at zero wait states, 83 instructions - the handler's exit, the
  tail-chain into PendSV, its thirteen instructions and the return
  through the fake frame, the entry (the ceiling read and 0 stored while
  PendSV's mask still holds, the unmask, base kept in the filler word),
  `activate()`'s read of the count of quiet posts, its looks at Top and
  High with each queue's two cursors compared in line, the raise of the
  ceiling and the count read again, `take()` - and 207 with every flash
  refill paid; the bound, 319, adds one tick (70) and a fifth.
- **The scan and the post** (letter d): `activate()` with four empty
  queues below the ceiling, entered and left by hand as the platform's
  entry and exit do, 170 cycles in every run (bound 213: 123 at zero
  wait states, 177 with every refill); a post that asks for nothing 108
  (bound 140), the count of quiet posts moved on that path (a literal,
  a load, the add, a store, a branch back: nine cycles at zero wait
  states); a post that runs High's empty dispatch inside it 575 (bound
  692: 433 at zero wait states, 576 with every refill), the
  scheduling's share 467 - two exception entries from thread mode
  (PendSV's and the SVC's, DDI 0484C's 15 cycles each) and two returns,
  the entry's masked store and the exit's count compared in the SVC's
  eleven instructions, and a line of a flash at two wait states
  refilled at every call and taken branch on the way.
- **Time events under preemption** (letter f): three timers matured in
  one turn are posted with the ceiling held at 0 and served High, Mid,
  Low; High preempting Low's loop of `arm()` and `disarm()` by road 1
  (in the loop's tenth turn of 200) leaves the list whole, every timer
  due fired once, the disarmed one never; one `arm()` 93 cycles (bound
  135) with its masked window inside.
- **The request under PRIMASK** (letter g, the core having no FPU to
  test): a post to a higher object inside a critical section pends
  PendSV - PENDSVSET reads set inside the section, the receiver has not
  run - and the receiver runs when the section's guard restores the
  mask, before the statement after it, from the Console's dispatch and
  from inside Low's, ten runs each; the section's last statement to the
  receiver's first 224 to 225 cycles on every event no kernel tick
  landed in (bound 273: the path counts 175 cycles at zero wait states
  and 227 with every flash refill paid). The letter reads the tick
  around each event, as letter c does: an event a tick lands in is
  reported and judged against 343, the bound with one tick's 70 on it,
  and every event over 227 must have had a tick in it. In ten runs of
  `z`, a hundred events, a tick fell inside one, which took 288 cycles,
  every verdict green. The nested guard of the trace's own
  `note()` inside the section restores PRIMASK set and takes nothing.
- **The stack** (letter h): painted from main's frame down to the end of
  `.bss` and `.noinit` (0x20000730..0x20023F80), 500 to 520 bytes of
  high-water for the suite's deepest chain (the paint is laid once at
  boot, so the mark is the deepest since the flash), the deepest sampled
  stack pointer at 464 bytes in a dispatch and 336 to 344 in the
  interrupt, the mark
  within the 156 bytes an action's frames, a basic exception frame and
  a handler's or a PendSV level's frames add. The SVC frame of the exit
  stands where a returned `activate()`'s frame stood, and the one run a
  request made in PendSV's first instruction can start at an entry
  (kernel/rubato.hpp, "The stack") reaches 44 bytes below the dispatch
  of the run it delays, inside the same 156.
- **Dark** (letter i): every object idle, no timer armed, TIM14 quiet,
  PA5 not an output, the ceiling the Console's, interrupts enabled.
- **The hot path in the release listing**: PendSV's body is thirteen
  instructions, every one a 16-bit form, 32 bytes with its literal, and
  no barrier (the DSB is the Cortex-M4's alone, for its erratum,
  [../cortexm/README.md](../cortexm/README.md)); the thread-mode entry
  twelve, the first four under PendSV's mask (the ceiling's literal,
  its load, the 0 and its store); the SVC's eleven on the path that
  finds the count of quiet posts still, five more on the path that
  pends PendSV again; the request seven (the mask and the address, the
  store to ICSR, DSB, ISB, the return), a call the compiler cannot see
  into; `post<Low>()` eighteen on the path that asks nothing, five of
  them the count's, and thirteen and one call on the path that asks;
  `activate()` reads each queue's two cursors and compares them in
  line, no call per queue (`EventQueue::empty()` is forced inline), the
  chosen index, the scan's limit and the count in registers, and
  `idle_if_empty()`'s masked window, from the `cpsid` to the WFI, is 28
  instructions and no call, the store of the base's ceiling, 255, among
  them. The rest of this paragraph
  was measured in diagnostic builds of the suite (not kept) whose
  `activate()` calls the queue's `empty()` out of line. Two that differ
  only in comments and
  constants put the road-1 path at 197..198 and 193..194 cycles, the
  scan at 184 and 165, the road-2 post at 546 and 531: a path's cost
  from this flash moves with where it sits, as "Code in SRAM" below
  finds for the drivers. Three that differ only in PendSV's body tell
  the barrier from the place. With
  the DSB, the body fourteen instructions and 36 bytes: road 1 at
  200..201 cycles (min..median), the road-2 post 551, the section's end
  to the receiver 223..224 (three runs). With two NOPs in the DSB's four
  bytes, the code after the body where it was and the barrier gone:
  199..200, 550 and 222..223 (three runs). Without either, the body as
  it is: 202..203, 557 and 224..225 (ten runs). So the barrier costs
  this core about one cycle, and the four bytes the body gave up move
  the code after it and cost about three on this flash: a second
  instance of the same finding.

**The drivers under the preemptive kernel** - the suite is
`test_stm32_rubato_link`, two keys against the Nucleo-F446RE's unchanged
`spi_peer` (`z`, 140 verdicts) and `usart_peer` (`y`, 111 verdicts), three
runs of each alike, every verdict green. It runs the real SPI and UART
stacks inside a Rubato pack of seven objects: `util/spi_bus.hpp`'s
arbiter over the engined `SpiHost<1>` ([spi.md](spi.md); half-word
engines on DMA1 channels 1 and 2), `util/line_reader.hpp`'s
`LineReader` over `Uart<1>` and its receive ring on DMA1 channel 3
([usart.md](usart.md)), a borrower of the reader's lines above it, and
on top an object that TIM14's update interrupt posts to every 97, 193
or 389 us, whose dispatch spins 300 or 3000 cycles - at 97 us x 3000,
48 % of the core. Every letter runs once with TIM14 silent as its
control and six times under that pressure, and every pass judges the
pressure itself: one dispatch per period over the pass, none dropped.
The target-free half of the suite (the pack, the passes, the numbered
lines, the window client, the UART legs) is the link suites' shared
body, `avrxt/src/apps/rubato_link.hpp`. What it established:

- **The polled SPI loop under preemption loses nothing.** The loop runs
  inside the arbiter's dispatch, so a dispatch above the arbiter holds
  it off for its whole length - TIM14 landed there up to 353 times a
  pass. With the RXFIFO as the write-ahead (no more frames in flight
  than it holds, [spi.md](spi.md)), a held-off loop leaves a gap on the
  bus and never a lost frame: the polled loop packed (8 frames) and
  frame by frame (5), the RXNE pump in byte frames (7) and in 16-bit
  frames (16, from an odd address) and the DMA engines (32) all
  byte-exact at both ends at every code from PCLK/256 to PCLK/8, under
  every pressure as in the control, and the host's OVR flag read clear
  at every one of the replies (the receive shapes never let it rise).
  This host has no `spi_overrun` and needs none.
- **PCLK/4 = 16 MHz is this fitting's, not the service's.** The same
  rungs at PCLK/4 slip in up to one window in ten of a run, with TIM14
  silent as well as under it and on the DMA engines too, which no
  preemption reaches
  (the suite's letter `x`, forty windows a shape and a pressure, four
  runs: 16-bit frames on the engines 2 of 160 windows silent and 4 of
  160 pressed, on the pump 2 and 4, on the polled loop 1 and 0; byte
  frames on the pump 0 and 0; 16-bit frames at PCLK/8 0 and 0). Letter
  `p` therefore judges to PCLK/8 and prints PCLK/4 (exact on 105 of 105
  path rungs over three runs) and PCLK/2 (above the F446 client's
  22.5 MHz, wrong on every path, as [spi.md](spi.md)'s peer ladder
  finds).
- **The matrix and the arbiter.** The four modes in both widths, LSb
  first and a DORD mismatch on the engines at PCLK/32, byte-exact in
  every pass, the data windows at 725 to 1000 per mille of the control's
  rate under pressure; three requesters of three priorities on one
  select window (the Client above the arbiter on the engines, a
  requester below it polled, the preempting object itself on the pump)
  answered every one `spi_ok`, to its requester, in its order, with no
  timeout, no rejection and the peer counting every frame exact.
- **The UART receive side under preemption.** Numbered lines echoed by
  the F446 at 460800 and 921600, assembled by `LineReader` from the
  ring and lent to the borrower: 200 a leg, none lost, out of order or
  corrupt, every port counter zero - the RXFT kick (`rx_kicks()`)
  included -, 640 to 801 per mille of the wire at 460800 and 460 to 670
  at 921600 (the writer paces each line to its own last stop bit and
  two frame times); 300 lines blasted into the peer's sink counted and
  summed exact, 998 to 999 per mille of the wire at 460800 and 1004 to
  1006 at 921600 - the divisor's +0.64 % (64 MHz / 69), the ruler and
  the USART counting one HSI16.
- **The loan under preemption.** At 460800 with the borrower spinning
  6000 cycles a line and the lines streamed five ahead, TIM14 landed
  inside the borrower's dispatch 99 to 621 times a pass; every line
  arrived exact, each lent alone, the two buffers strictly alternating,
  and the receive stream moved inside the borrower's dispatch 21 to 32
  times a leg. The rate is the echo's: the F446's echo has no FIFO and
  holds two characters of a writer's lead, this end runs 1.27 % ahead of
  it at 921600, and a streamed run of some 427 characters would leave 5
  there; at 460800 the lead is 0.04 %.
- **Dark** (letter `i`): every object idle, TIM14 stopped, no time event
  armed, SPI1 idle with its engines idle (the engined host keeps CR2's
  DMA enables set for its life, and a finished channel stays enabled
  with its count at zero), USART1 given back, PA5 not an output, the
  ceiling the Console's.

**The I2C stack under the preemptive kernel** - the suite is
`test_stm32_rubato_i2c`, one key (`z`, 195 verdicts) against the
Nucleo-F446RE's unchanged `twi_peer`, every verdict green on every run
([i2c.md](i2c.md) says how many of each letter). A Rubato pack of five objects runs `util/i2c_bus.hpp`'s arbiter
over the engined `I2cHost<1>` ([i2c.md](i2c.md)) with three requesters
of three priorities, under the same TIM14 pressure and the same seven
passes as the link suite; the target-free half (the pack, the command
channel through the arbiter, the letters) is the I2C suites' shared body,
`avrxt/src/apps/rubato_i2c.hpp`, and the waits and passes both bodies
share are `avrxt/src/apps/rubato_passes.hpp`. Every tenure shape on both
of the host's paths, both speeds, the vocabulary, the commanded stretch
and the arbiter's ordering held under every pressure as in the control;
a preemption inside the host's `start()` is a later START and never a
stretch, and the tenure cycle locks to the pressure's period (the
findings are [i2c.md](i2c.md)'s, "The stack under the preemptive
kernel").

Two kernel programs on the Nucleo-G0B1RE: two AOs under time events at
500/250/100 ms (PA5 sampled over SWD every 100 ms shows the 500 ms cadence
and the switch to 250 ms after three seconds), and three AOs over USART2
(see usart.md). The kernel tick against the PC's clock over ten seconds:
+0.24 % (HSI16 is factory-trimmed to 1 %; the host side of that
measurement is a serial round trip, so the figure is coherence, not
metrology). SysTick CTRL 0x7 / LOAD 0xF9FF read back. Between events the
core sits in WFI (the PC read over SWD is the instruction after the WFI,
inside the kernel's idle path), which is the state README.md's HLA read
caveat is about.

## Code in SRAM

**What the silicon does.** At 64 MHz the flash answers at two wait
states (RM0444 3.3.4, table 13) behind the prefetch and the
instruction cache (3.3.5), both on as [clock.md](clock.md) sets them;
the SRAM answers at none (2.3), and the Cortex-M0+ fetches both over
its one bus (2.1). The prefetch and the cache hide a fifth of the
flash's cost on a driver's hot body and no more: the SPI pump's entry,
inline and unflattened in its vector, measured 416 cycles with both on
and 507 with both off, and 305 from SRAM with either. SRAM sits 512 MB
above the flash and a Thumb `bl` reaches 16 MB, so A CALL BETWEEN THE
TWO MEMORIES GOES THROUGH A VENEER the linker puts beside the caller:
`push {r0}; ldr r0, [pc, #8]; mov ip, r0; pop {r0}; bx ip`, a pad and
the address - 16 bytes, nine cycles beyond the `bl`'s three by the
core's table (the RP2040 datasheet's 2.4.3.3, table 81, the copy of the
Cortex-M0+'s figures this desk has), fetched from the caller's memory.
The vector table holds absolute addresses, so a vector placed in SRAM
is entered with no veneer at all.

**The option and its spelling: opt-in, per driver.** The three linker
scripts (`stm32g0/ld/`) collect the input sections `.fast_text` and
`.fast_text.*`, then `.ram_text`, first in `.data`, so the crt's copy of
`.data` moves them to SRAM before `main`, and the platform says so
(`Stm32g0Platform::places_fast_text`) - on every part, the STM32G031's
8 KB included: the placement is the program's, its cost the program's
to weigh, and a link that does not fit is refused by the script's ram
region. `.fast_text` is code that MAY run from SRAM, for speed;
`.ram_text` is code that MUST, for correctness - nothing of this
stratum's. An image that places nothing gets zero bytes from either.
Two drivers take util/placement.hpp's tag as their last template
parameter, `InFlash` by default:

- `SpiHost` - the pump's entry (`isr()`) and the polled transaction,
  each ONE flattened instance under `InRam` ([spi.md](spi.md));
- `UartTask` (`Uart`, `LpUart`, `Rs485`) - `isr()` and `dma_isr()`, each
  one instance ([usart.md](usart.md)).

```cpp
using Display = brio::SpiHost<1, display_pins, brio::NoDmaEngine, brio::NoDmaEngine, brio::InRam>;
using Serial = brio::Uart<2, console_pins, 64, 256, brio::NoDmaEngine, brio::NoDmaEngine,
                          brio::UartOptions{}, brio::InRam>;
```

Under `InFlash` each body is what it would be without the trampoline,
inline in its vector or in `start()`; both placements FLATTEN it, so
what it calls on its common path is inline in it - the transmit
engine's completion carries the next block's launch (`pump_tx()`) - and
what it calls once a burst or on an error stays out of line in the
flash (the SPI's `launch_dma()` and `spin_dma()`, the USART's
`harvest()`, the engines' `abandon()`). The I2C host takes no placement:
measured, its event pump as an instance behind a vector in the flash
came out no shorter ([i2c.md](i2c.md)). A third tag, and a placement on
the I2C host, do not compile (`test/family_stm32g0/neg/`). THE NAME IS
TWO NAMES: the trampoline's instances are COMDAT functions in
`.fast_text.brio`, and a program's own placed function names
`.fast_text` - one name for both in one translation unit is GCC's
"section type conflict" (util/placement.hpp); the family fixtures
compile a placed function beside the `InRam` instances in one unit
(`test/family_stm32g0/spi.cpp`, `usart.cpp`).

A program may also place a VECTOR it binds, with
`[[gnu::section(".fast_text")]]` on the handler: what the vector inlines
moves with it and runs with no call, and an `InRam` body called from a
placed vector is a call where the inline one had none.

**What it buys, measured** by `bench_stm32` (every driver `InFlash`) and
`bench_stm32_ram` (the same source, every driver that takes a placement
`InRam`, the vectors in the flash in both) on the STM32G0B1RE at 64 MHz -
HCLK cycles, a handler's figure per entry with the meter's stamps inside
it (an empty pair is 42 in the flash, [clock.md](clock.md)), a loop's the
wall of 256 frames:

| path | flash | SRAM (`InRam`, the vector in the flash) | SRAM bytes |
|---|---|---|---|
| SPI pump, an entry (three 8-bit frames; PCLK/2 and /4 alike, the handler is the bound) | 397 | 347 | 624 an engineless host's, 684 an engined one's |
| the same, 16-bit frames (two an entry) | 356 | 308 | |
| SPI pump, the thread's busy a frame at PCLK/16 (256 frames read back: `pump_cycles_per_frame`) | 138 | 122 | |
| SPI pump, 256 8-bit frames at PCLK/2, wall (the wire 4096) | 43824 | 39125 | |
| SPI receive loop, 256 8-bit frames at PCLK/2 (the wire 4096) | 6470 | 6189 | 1644 a host's polled transaction, both widths (1612 engined) |
| the same at PCLK/4 (the wire 8192) | 8609 | 8535 | |
| SPI receive loop, 256 16-bit frames at PCLK/2 (the wire 8192) | 10536 | 10028 | |
| SPI transmit-only loop, 256 8-bit frames at PCLK/2 (the wire 4096) | 4687 | 4467 | |
| a polled request of 1, 3 and 16 bytes at PCLK/16 (wall) | 444, 797, 2504 | 429, 765, 2476 | |
| USART transmit engine's completion, a block at 2 Mbaud | 232 | 212 | 264 (340 with a receive engine) |
| USART transmitter's FIFO refill, eight bytes an entry at 115200 | 323 | 318 | 404 an engineless port's `isr()` |
| USART receive, four bytes an entry at 115200 | 355 | 316 | 332 the `isr()` of a port whose transmitter is on its engine |
| the same at 2 Mbaud: entries for 256 bytes, cycles an entry | 51, 408 | 64, 316 | |
| the console's interrupt, a print of 4096 bytes | 295 | 275 | 436 |
| USART receive engine: the line's IDLE / the ring's marks | 206 / 226 | 225 / 226 | 84 / 340 |

What the rows say:

- THE PUMP PAYS MOST: an eighth off every entry, 4700 cycles off 256
  frames at PCLK/2. The polled transaction is a function of its own in
  either memory (spi.md says why), and from SRAM its receive loop is a
  twentieth shorter at PCLK/2 (x 1.58 of the wire to 1.51), its 16-bit
  loop likewise, a polled request 15 to 32 cycles. Where a loop is the
  wire's already (everything at PCLK/4 and below) SRAM changes nothing.
- THE USART'S BODIES PAY LESS, because both placements flatten them:
  the completion's launch is inline in the flash too, so SRAM takes a
  tenth off it; the receiver a ninth (at 2 Mbaud the flash's slower
  entry takes five bytes an entry where SRAM's takes four); the
  transmitter's refill a sixtieth at 115200.
- THE ONE ROW SRAM LOSES is the receive engine's IDLE entry, whose work
  is `harvest()` out of line: from SRAM that is a veneer back into the
  flash, once a burst.
- The SPI engines' transaction takes no placement: letter d reads
  1093..1115 cycles above the wire in the flash image and 1069..1083 in
  the SRAM one, the difference the bench's DMA vector, which binds
  letter u's ports beside the SPI host's completion - its entry 267
  cycles where their bodies are inline, 246 where they are calls into
  SRAM ([spi.md](spi.md)).
- A path's cost from the flash moves with where it sits: two builds of
  `bench_stm32` that differed in the SPI host alone measured the USART
  receiver's entry at 372 and 355 cycles and the refill at 320 and 323;
  the same bodies from SRAM moved by two cycles at most.

**What it costs.** The bytes in the last column - of the G0B1's 144 KB,
the G071's 36 and the G031's 8, the load image staying in the flash
beside them - plus 16 bytes of flash for every function the flash calls
in SRAM (twelve in `bench_stm32_ram`) and 16 of SRAM for every function
an instance calls back in the flash (five there: `launch_dma()`,
`spin_dma()`, `harvest()` and two `abandon()`). No code: the crt's copy
of `.data` already moves the section, a word a turn of its loop. A
placed function exists only from that copy on - a vector taken earlier
would enter SRAM the copy has not written yet. A default image has no
byte of code in SRAM. The whole of `bench_stm32_ram` is 7040 bytes of
SRAM above `bench_stm32`'s. On the STM32G031K8, where it counts, a
console's `isr()` is 324 bytes (its USART2 is a BASIC instance, no FIFO)
and every image of the stratum's suites with every eligible driver
`InRam` grows by 324 to 2840 bytes: `test_stm32_serial`'s fourth image
3396 to 6236 of 8192, `spi_peer` 1552 to 3520, and `test_stm32_spi`
past the part by 496 bytes and `bench_stm32`'s first two images by 3604
and 4532 - which is why `bench_stm32_ram` is not built there.

## On the STM32G071RB

Every platform-level claim of this document holds on the Nucleo-G071RB
(DEV_ID 0x460, REV_ID 0x2000), letters `w` and `k` aside (measured on
the G0B1RE alone): `test_stm32_platform` runs in
`z` (the reset flags, the critical section, the idle hook, SysTick's
arithmetic, `delay_us` and both watchdogs), `test_stm32_sleep` likewise
(all four depths and both sites - [pwr.md](pwr.md)), and
`test_stm32_tickless` runs the LPTIM timebase and `idle_until` there.
`flash.hpp`'s `DeviceIdcode` reads DBGMCU_IDCODE's DEV_ID and
REV_ID through the APB gate 5.2.17 keeps closed at reset (and puts the
gate back as it found it, the way `Pwr::debug_in_stop()` does): every
bench suite prints it at boot, because a measurement that differs between
two boards is only a finding once the die it was taken on is on the
record.

ONE LETTER OF `test_stm32_tickless` IS PART-DEPENDENT: letter `i`
measures what a lap wake COSTS
with a meter built out of TIM2's ETR taking MCO, and RM0444 22.4.25 gives
that ETRSEL code to the G0B1/G0C1 alone. On a part without it the meter
does not count - and `spin_us()`, this suite's own microsecond wait,
rides the same TIM2, so a meter that does not count is a wait that never
ends and the IWDG reboots the board. The meter and the awake-time verdict
go together behind `tim_etrsel_has_mco()`; the lap count and the
never-re-lock-the-PLL claim need no meter and stay.

## On the STM32G031K8

`test_stm32_platform` runs on the Nucleo-G031K8 (DEV_ID 0x466,
REV_ID 0x1003), letter `i` included over its six real resets, with the
same letters and the same verdicts as on the LQFP64 parts, letters `w`
and `k` aside (measured on the G0B1RE alone). The
board's user LED is on PC6 (a Nucleo-32 fact), which nothing here
judges.

**THIS DIE'S LSI IS THE SLOWEST OF THE THREE**: the IWDG time-out of
letter `i` implies **31400 Hz** where the G0B1RE reads 32536 and the
G071RB 32295 - all three inside DS12992/DS13560 table 46's 29.5..34 kHz,
and none of them each other. The board's crystal runs, so this is the
rate of the LSI witness letters alone ([../boards/nucleo-g031k8.md](../boards/nucleo-g031k8.md)).

**AND LSIRDY IS A READING ABOUT THE RTC DOMAIN AND NOT ABOUT THE
WATCHDOG**: LSIRDY stands with LSION clear exactly where RCC_BDCR holds
RTCEN with RTCSEL = LSI, and is clear otherwise - on this board it reads
0 with LSION 0 and the domain on the crystal (RCC_BDCR 0x8103), as it
read 0 with the domain holding no RTCEN at all. Letter `a` claims the
EQUIVALENCE, which is what makes it a statement about 5.4.24 rather than
about one desk.

## Not covered yet

Driver gaps (this chapter's option space the stratum does not touch):
- VTOR relocation, which nothing here needs while the vector table is
  the crt's and sits where the part boots; born with its first user (a
  bootloader).
- NVIC priorities: exposed and measured (letter `n`), and nothing in
  the stratum assigns one to a peripheral line: under both kernels every
  line stays at level 0, one hardware level above the loop, and the
  preemptive one puts only PendSV below them (`scheduler_init()`);
  priorities among the lines would break the promise that no handler
  nests over another ([design/kernel.md](../design/kernel.md),
  "Rubato"), and none is born until a design asks for one.
- A per-package pin-bonding table - [port.md](port.md) says why not.
- The veneer between the flash and SRAM: every `InRam` call from the
  flash pays the linker's long-branch stub; GCC's ARM `long_call` on
  util/placement.hpp's `Hot<InRam>` would call the instance by address
  instead (measured on the SAM C21: 8 to 18 cycles a call). That header
  is every family's, so the choice is made there, not here.

Implemented, not bench-verified: `in_handler()` on this core (IPSR is
the architecture's, ARMv6-M ARM B1.4.2, and the same read is measured
on the STM32F4's Cortex-M4, [../stm32f4/usart.md](../stm32f4/usart.md)
letter h - a letter that asks it from a vector, from the main loop and
under a critical section would measure it here); a VECTOR the program
places in `.fast_text` over the flattened bodies ("Code in SRAM" gives the pump's
entry inline and unflattened, 305 cycles) - a `bench_stm32` built with
its vectors in `.fast_text` would measure it;
`bench_stm32_ram` on the STM32G071RB (the same image, run there, is the
measurement); letters `w` and `k` on the
STM32G071RB and the STM32G031K8 (the same image, run there, is the
measurement); the idle path of `idle_until()` on the tickless
timebase, whose arming adds instructions between the masked check and
the WFI - the same WFI under PRIMASK closes it, and letter `w`'s walk
over `Stm32g0Platform<LptimTicker>` with the LPTIM's compare as the
edge is what would measure it; the preemptive platform over the
tickless timebase, `Stm32g0RubatoPlatform<LptimTicker<>>` (its
`idle_until()` is the Tenuto platform's, inherited, and compiled by the
family fixture; `test_stm32_rubato` runs on the SysTick `Ticker`, and
the same pack on the LPTIM's platform with its vector bound is what
would measure it); `test_stm32_rubato` on the STM32G071RB and the
STM32G031K8 (built for the G0B1RE alone; TIM2, TIM14 and every verb it
uses exist on both, so its boards line and a run there are the
measurement); the preemptive kernel's stack bound under a load that
never lets the pack drain - at most one run per object, and the one
empty run a request made in PendSV's first instruction can start at an
entry (kernel/rubato.hpp, "The stack") - which no letter of this
family drives: letter h measures the chain the suite builds, and the
I2C suite's flood is paced; its letter h run unpaced, with the stack
painted and read, is what would measure it; `abort()`'s and `HardFault_Handler`'s spins (the fault VECTOR is
exercised, by `hard_fault_reset` - [reset.md](reset.md); the spins are
what runs when no body is bound, and every suite binds one). The
cortex-debug launch entry ([README.md](README.md)) has not been driven
end to end either; halt-and-dump through OpenOCD's own console is what
this stratum's findings were taken with.

The tickless timebase's two CONTRACTS, stated rather than checked
because no register can check them: a masked window (PRIMASK held)
must stay under half a lap, one second - a double wrap under a mask is
one ARRM flag, indistinguishable from a single one, and `ticks()` then
loses a lap; in a brio program the longest legitimate masked window is
microseconds (`delay_us` refuses at a millisecond), so this is a bug's
name and not a regime. And a deadline nearer than six counts (183 us)
is not slept for: `idle_until` declines and the loop spins it through
`process()`, which fires it on time - the part stays awake for under
183 us instead of taking a Sleep, and only when a deadline is already
that near when the loop reaches its idle; a SysTick one-shot for those
would be a second timebase for a case this cheap, and is not built.
The tickless timebase's own gaps: the keystroke letter `u` (a UART
byte as the foreign wake of a long `idle_until`) needs an operator and
is outside `z`; an LSI-rooted ticker as the platform's OWN timebase -
the suite's `tb_cfg` selects the crystal, which every bench board
carries, so the LSI root runs only as the LPTIM2 witness of letter `h`
(its arithmetic and its wake measured there); setting `tb_cfg.source`
to `lsi` is the whole of the measurement, and the same letters, Stops
included, then run on it. And on the STM32G071RB and the STM32G031K8,
**what a lap wake COSTS** (letter `i`): its meter is TIM2's ETR taking
MCO, an ETRSEL code of the G0B1/G0C1 alone, so the lap COUNT and the
never-re-lock-the-PLL claim are measured on those parts and the
microseconds are not.
