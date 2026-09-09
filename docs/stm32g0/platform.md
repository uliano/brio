# Platform - what the kernel stands on (STM32G0)

> **PROVISIONAL.** All three halves are here and bench-verified: the
> WAKING half below (the critical section, the two idle hooks, the two
> timebases - SysTick, and the tickless LPTIM one that counts through a
> Stop - the NVIC, the crt), the FAILING half next door in
> [reset.md](reset.md) (which reset happened, both watchdogs, the
> HardFault breadcrumb across a real reset), and the STOPPING half in
> [pwr.md](pwr.md) (PWR's mode ladder and the `SleepSite`s). What is
> left is small and listed in "Not covered yet".

Documents of record: RM0444 Rev 6 - the Cortex-M0+ summary ch. 12
(NVIC) with ARM's ARMv6-M ARM behind it, and PWR ch. 4 for what a WFI
here really enters ([pwr.md](pwr.md) owns that chapter) - and errata
ES0548 Rev 3
(2.8.2 and 26.4.11 shape the tickless timebase's arming; nothing else
touches this chapter on revision Z). Drivers:
`stm32g0/platform.hpp` (`Stm32g0Platform<TB>`, this target's
realization of the kernel's `Platform` concept, templated on its
timebase, and the `Tickless` concept), `stm32g0/ticker.hpp` (the
SysTick `Ticker`: this family's include of the core stratum's
`armv6m/ticker.hpp` - `BasicTicker`, `SysTickCounter`),
`stm32g0/lptim_ticker.hpp` (`LptimTicker`, the tickless timebase over
[lptim.md](lptim.md)'s driver), `stm32g0/delay.hpp` (the microsecond
busy-wait), `stm32g0/reset.hpp` ([reset.md](reset.md)),
`stm32g0/nvic.hpp` (`armv6m/nvic.hpp` - `InterruptGuard`, `Nvic`;
[../armv6m/README.md](../armv6m/README.md)). The
crt is `stm32g0/src/glue/startup_stm32g0b1.cpp` +
`stm32g0/ld/stm32g0b1re.ld` in the build project. The family fixtures
are `test/family_stm32g0/platform.cpp` and `lptim_ticker.cpp` under
`tools/check_stm32g0.sh`, with the negatives that refuse a timed sleep
site on a tickless platform and the tickless timebase off the value
line.

## What the silicon does

**Interrupt masking is PRIMASK, and nothing finer.** ARMv6-M has no
BASEPRI: masking is all-or-nothing, one bit, saved and restored. The
NVIC orders PREEMPTION between handlers with two priority bits (four
levels, 0 most urgent); it never limits the reach of a critical
section. Core exceptions (SysTick among them) have no NVIC enable bit.

**The vector table is 16 + 32 entries and SHARED LINES ARE THE RULE.**
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
construction. What the WFI enters is WHATEVER IS ARMED: SCR.SLEEPDEEP is
0 out of reset and this file never writes it, so a bare WFI is Sleep -
HCLK, SysTick and every peripheral keep running (5.3) - and with
`stm32g0/sleep.hpp`'s site having armed a Stop, the same WFI is that
Stop. **THE IDLE HOOK KNOWS NOTHING OF THE DEPTH**, and that is worth
stating beside the other two targets: the AVR's `idle()` must honour a
standing SEN bit and the SAM C21's carries an erratum guard, while this
hook writes neither SCR.SLEEPDEEP nor PWR_CR1.LPMS - this silicon
selects the depth there, and the arming is the sleep site's business
alone.

**SysTick rides HCLK.** The reload is `Clock::hz / 1000 - 1`
(63999 at 64 MHz, read back over SWD); in Stop the core clocks stop and
so does the kernel's time - the samc21 standby situation, and it has the
same two answers here: `Stm32g0SleepSite` keeps the honest restriction and
`Stm32g0TimedSleepSite` LIFTS it, resynchronizing from the RTC through
`advance()` ([pwr.md](pwr.md)). THE TICK ITSELF DOES NOT DEFEAT A DEEP
SLEEP, though a debugger can make it look so: 4.3.3 makes a WFI a no-op
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
and exact. A program on it is TICKLESS: no periodic interrupt at all,
the kernel's optional `idle_until(deadline)` hook
([../design/kernel.md](../design/kernel.md) section 11) places the
loop's next deadline in the LPTIM's compare and one WFI covers the
whole wait at whatever depth a sleep site armed, kernel time RUNS
through the Stop, and the plain sleep site is the only site it takes
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
a PARITY CHECK the factory option byte leaves DISABLED
(FLASH_OPTR.RAM_PARITY_CHECK = 1): with it enabled, the first read of
a never-written word - the panic breadcrumb's, after a power-on -
would raise an NMI. The breadcrumb's magic word is what makes the
default case harmless; the option-byte pass has to remember the other.

**A BKPT with C_DEBUGEN set halts the core in silence**, as on the
samc21: `break_here()` cannot ask whether a debugger is attached
(ARMv6-M), so `tools/bench.py` clears DHCSR after every flash and a
BKPT with no debugger escalates to the crt's distinct
`HardFault_Handler` spin.

## Types and verbs

- `Stm32g0Platform<TB = Ticker>` - `Timebase` (= TB), `CriticalSection`
  (= `InterruptGuard`), `idle()` (DSB, WFI, unmask),
  `interrupts_enabled()`, `break_here()` (BKPT), `now()`/
  `ticks_per_second` (the timebase's), `atomic_width` 4,
  `panic_record()` in `.noinit` - and, on a `Tickless` timebase only,
  `idle_until(std::optional<uint32_t> deadline)`: the kernel's optional
  hook, entered masked with the absolute tick of the nearest armed time
  event; a due deadline or a wake the timebase could not place this turn
  means no sleep (interrupts back on, return), otherwise the wake is
  placed and the same DSB/WFI/unmask follows. `Tickless<TB>` requires
  `ticks()`, `ticks_per_second`, `arm_wake(now, deadline) -> bool`; the
  SysTick `Ticker` is statically not one.
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
  declined and spun through; before a Stop the store is waited for; a
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
  `interrupts_enabled()`, `irq_priority_levels` (4) - armv6m/nvic.hpp
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
  `delay_rate(hz)` - stm32g0/delay.hpp, which is `armv6m/delay.hpp`
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
    brio::Kernel<P, Blinker, Supervisor>::run();
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
    brio::Kernel<P, Blinker, Supervisor>::run(); // sleeps TO each deadline
}
```

## Bench findings

The reference suite is `test_stm32_platform` (six letters in `z`, 53
verdicts; letter `i` outside it reboots the board six times -
[reset.md](reset.md) carries the reset and watchdog half).
What it measures of THIS chapter:

- **PRIMASK nesting is nesting**: leaving an INNER critical section does
  not unmask, because the guard saves and restores the bit rather than
  clearing it.
- **A masked window loses ticks and does not lose time**: 5 ms with
  interrupts masked advances the tick by 1 ms - SysTick's interrupt is
  a pending BIT, so the coalesced ticks are gone, and a timebase that
  must survive long masked windows needs more than the pending bit.
- **`idle()` sleeps and the tick brings it back**: with the console
  drained, ONE `idle()` call covers the 960..990 us to the next tick,
  and it returns with interrupts enabled. (With the console still
  draining it returns in 85 us on the USART's own interrupt, which is
  the hook working, not failing.)
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
  each kept the part awake for 214..215 us measured ISR to ISR on TIM2
  clocked from MCO = HSI16/8 - a clock that runs only while the part is
  awake - that is the Stop exit, the handler, one empty loop turn and
  the WFI, at 16 MHz with the PLL's two wait states still in
  FLASH_ACR; 1.8 ms awake in the whole ten seconds, a 10^-4 duty. It
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
  insurance the silicon does not need, kept at 93 us a round.
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

## On the second silicon

Every platform-level claim of this document holds on the Nucleo-G071RB
(DEV_ID 0x460, REV_ID 0x2000): `test_stm32_platform` runs whole in
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

## On the third silicon

`test_stm32_platform` runs whole on the Nucleo-G031K8 (DEV_ID 0x466,
REV_ID 0x1003), letter `i` included over its six real resets, with the
same letters and the same verdicts as on the other two dies. The
board's user LED is on PC6 (a Nucleo-32 fact), which nothing here
judges.

**THIS DIE'S LSI IS THE SLOWEST OF THE THREE**: the IWDG time-out of
letter `i` implies **31400 Hz** where the G0B1RE reads 32536 and the
G071RB 32295 - all three inside DS12992/DS13560 table 46's 29.5..34 kHz,
and none of them each other. The board's crystal runs, so this is the
rate of the LSI witness letters alone ([bench.md](../bench.md)).

**AND LSIRDY IS A READING ABOUT THE RTC DOMAIN AND NOT ABOUT THE
WATCHDOG**: LSIRDY stands with LSION clear exactly where RCC_BDCR holds
RTCEN with RTCSEL = LSI, and is clear otherwise - on this board it reads
0 with LSION 0 and the domain on the crystal (RCC_BDCR 0x8103), as it
read 0 with the domain holding no RTCEN at all. Letter `a` claims the
EQUIVALENCE, which is what makes it a statement about 5.4.24 rather than
about one desk.

## Not covered yet

Driver gaps (this chapter's option space the stratum does not touch):
- VTOR relocation; NVIC priorities are exposed but nothing assigns one.
- A per-package pin-bonding table.

Implemented, not bench-verified: `Nvic::priority`, `abort()`'s and `HardFault_Handler`'s spins
(the fault VECTOR is exercised, by `hard_fault_reset` - reset.md),
the cortex-debug launch entry.

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
has not been run unattended; an LSI-clocked ticker has been the WITNESS
on LPTIM2 and not yet the platform's own timebase in a program (the
arithmetic and the wake are measured; a Stop on it is not).

On the second silicon (the Nucleo-G071RB), NOT COVERED: **what a lap wake
COSTS** in `test_stm32_tickless` letter `i`, whose meter is TIM2's ETR
taking MCO. The lap COUNT and the never-re-lock-the-PLL claim are measured
there; the microseconds are not.
