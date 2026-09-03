# Platform - what the kernel stands on (STM32G0)

> **PROVISIONAL.** All three halves are here and bench-verified: the
> WAKING half below (the critical section, the idle hook, the SysTick
> timebase, the NVIC, the crt), the FAILING half next door in
> [reset.md](reset.md) (which reset happened, both watchdogs, the
> HardFault breadcrumb across a real reset), and the STOPPING half in
> [pwr.md](pwr.md) (PWR's mode ladder, the two `SleepSite`s and the
> RTC-backed timebase that survives a Stop). What is left is small and
> listed in "Not covered yet".

Documents of record: RM0444 Rev 6 - the Cortex-M0+ summary ch. 12
(NVIC) with ARM's ARMv6-M ARM behind it, and PWR ch. 4 for what a WFI
here really enters ([pwr.md](pwr.md) owns that chapter) - and errata
ES0548 Rev 3
(no item touches this chapter on revision Z). Drivers:
`stm32g0/platform_stm32.hpp` (`Stm32Platform`, this target's
realization of the kernel's `Platform` concept), `stm32g0/delay.hpp`
(the microsecond busy-wait), `stm32g0/reset.hpp`
([reset.md](reset.md)), `stm32g0/nvic.hpp`
and `stm32g0/ticker.hpp` (this family's includes of the core stratum's
`armv6m/nvic.hpp` - `InterruptGuard`, `Nvic` - and `armv6m/ticker.hpp`
- `BasicTicker` - plus the `Ticker` alias;
[../armv6m/README.md](../armv6m/README.md)). The
crt is `stm32g0/src/glue/startup_stm32g0b1.cpp` +
`stm32g0/ld/stm32g0b1re.ld` in the build project. The family fixture
is `test/family_stm32g0/platform.cpp` under `tools/check_stm32g0.sh`.

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
Stop. **THIS FILE NEEDED NO CHANGE FOR THE SLEEP CAMPAIGN**, and that is
worth stating beside the other two targets: the AVR's `idle()` had to
learn to honour a standing SEN bit and the SAM's had to grow an erratum
guard, while this hook was already right, because this silicon selects
the depth in SCR.SLEEPDEEP and PWR_CR1.LPMS and it writes neither. The
proof is a byte-identity gate: all ten pre-existing STM32G0 images are
unchanged by the campaign that added the sites.

**SysTick rides HCLK.** The reload is `Clock::hz / 1000 - 1`
(63999 at 64 MHz, read back over SWD); in Stop the core clocks stop and
so does the kernel's time - the samc standby situation, and it has the
same two answers here: `Stm32SleepSite` keeps the honest restriction and
`Stm32TimedSleepSite` LIFTS it, resynchronizing from the RTC through
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

**SRAM survival across a reset is promised nowhere**, and the SRAM has
a PARITY CHECK the factory option byte leaves DISABLED
(FLASH_OPTR.RAM_PARITY_CHECK = 1): with it enabled, the first read of
a never-written word - the panic breadcrumb's, after a power-on -
would raise an NMI. The breadcrumb's magic word is what makes the
default case harmless; the option-byte pass has to remember the other.

**A BKPT with C_DEBUGEN set halts the core in silence**, as on the
samc: `break_here()` cannot ask whether a debugger is attached
(ARMv6-M), so `tools/bench.py` clears DHCSR after every flash and a
BKPT with no debugger escalates to the crt's distinct
`HardFault_Handler` spin.

## Types and verbs

- `Stm32Platform` - `CriticalSection` (= `InterruptGuard`), `idle()`
  (DSB, WFI, unmask), `interrupts_enabled()`, `break_here()` (BKPT),
  `now()`/`ticks_per_second` (the ticker's), `atomic_width` 4,
  `panic_record()` in `.noinit`.
- `InterruptGuard`, `enable_interrupts()`, `disable_interrupts()`,
  `interrupts_enabled()`, `irq_priority_levels` (4) - armv6m/nvic.hpp
  through stm32g0/nvic.hpp.
- `Nvic` - `enable`/`disable`/`enabled`, `set_pending`/`clear_pending`/
  `pending`, `priority` (refuses a level the core does not have), by
  the header's IRQn value.
- `BasicTicker<tps>` / `Ticker` (1000 Hz) - `init(clock)` (false when
  the reload does not fit 24 bits), `tick()` (the ISR body), `ticks`/
  `millis`/`secs`/`now`, `advance(n)` (the RTC resync's landing point -
  `Stm32TimedSleepSite` is its user), `pause`/`resume` (which the same
  site calls around every deep sleep). A rate that does not divide 1000
  is refused at compile time.
- `delay_us(clock, us)` / `delay_us(DelayRate, us)` +
  `delay_rate(hz)` - stm32g0/delay.hpp: a busy-wait of AT LEAST `us`
  microseconds on SysTick's VAL, CAPPED BELOW ONE KERNEL TICK by
  contract (a tick or more is TimeEvent territory and is refused with
  `false`, spending nothing), and `false` with no time spent when
  SysTick is not running at all. No division runs at wait time: the
  cycles-per-microsecond factor is rounded UP at compile time so every
  error lands late. The ticker owns SysTick IN WRITING; this file only
  reads VAL, and folds the reload across a wrap, so it is correct
  inside a masked window too.
- The crt: `Reset_Handler` (copy .data, zero .bss, walk .init_array,
  call main; .noinit untouched), weak `Default_Handler` aliases for
  every vector, a distinct weak `HardFault_Handler`, `abort()` as a
  spin.

## How to use it

```cpp
using P = brio::Stm32Platform;
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

## Bench findings

The reference suite is `test_stm32_platform` (six letters in `z`, 53
verdicts, 53/53 cold and warm; letter `i` outside it reboots the board
six times - [reset.md](reset.md) carries the reset and watchdog half).
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


Two kernel apps on the Nucleo-G0B1RE, both in the tree: blink (two
AOs, time events at 500/250/100 ms; PA5 sampled over SWD every
100 ms shows the 500 ms cadence and the Supervisor's switch to 250 ms
after three seconds) and console (three AOs over USART2; see
usart.md). The kernel tick against the PC's clock over ten seconds:
+0.24 % (HSI16 is factory-trimmed to 1 %; the host side of that
measurement is a serial round trip, so the figure is coherence, not
metrology). SysTick CTRL 0x7 / LOAD 0xF9FF read back. Between events
the core sits in WFI (the PC read over SWD is the instruction after
the WFI, inside the kernel's idle path) - which is also how the HLA
read caveat in README.md was found.

## Not covered yet

Driver gaps (this chapter's option space the stratum does not touch):
- VTOR relocation; NVIC priorities are exposed but nothing assigns one.
- A per-package pin-bonding table.
- `stm32g0/delay.hpp` and `samc/delay.hpp` are the same file but for
  their includes and one paragraph of reasoning; neither has been
  factored into `armv6m/`, which is where the pair belongs once a
  reviewer says so.

Implemented, not bench-verified: `Nvic::priority`, `abort()`'s and `HardFault_Handler`'s spins
(the fault VECTOR is exercised, by `hard_fault_reset` - reset.md),
the cortex-debug launch entry.

A DESIGN QUESTION, not a driver gap: **a kernel timebase on a counter
that does not stop.** SysTick rides HCLK and HCLK stops in Stop, which
is why kernel time stands still there and why two sleep sites exist to
repair it after the fact ([pwr.md](pwr.md)). An LPTIM on LSE or LSI
keeps counting through Stop 0 and Stop 1 ([lptim.md](lptim.md)), so a
TICKLESS platform is now physically available on this target: `now()`
read from a free-running LPTIM counter, and the loop's next deadline
placed in its compare register instead of counted out in ticks. That
would change the Platform's tick SOURCE and RATE for every program on
this family, and `BasicTicker`'s own header already names the day it
must either become a `ClockUser` or move off the core clock. It is a
decision for `docs/design/`, and it is stated here and not built.
