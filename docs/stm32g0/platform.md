# Platform - what the kernel stands on (STM32G0)

> **PROVISIONAL.** The WAKING half is here and bench-verified - the
> critical section, the idle hook, the SysTick timebase, the NVIC, the
> crt. The FAILING half (which reset happened, the watchdogs, the
> HardFault breadcrumb) and the STOPPING half (PWR's Sleep / Low-power
> sleep / Stop 0/1 / Standby / Shutdown ladder and the `SleepSite`
> over it) are not built. The list is in "Not covered yet".

Documents of record: RM0444 Rev 6 - the Cortex-M0+ summary ch. 12
(NVIC) with ARM's ARMv6-M ARM behind it, PWR ch. 4 (for what this
stratum deliberately does not touch yet) - and errata ES0548 Rev 3
(no item touches this chapter on revision Z). Drivers:
`stm32g0/platform_stm32.hpp` (`Stm32Platform`, this target's
realization of the kernel's `Platform` concept), `stm32g0/nvic.hpp`
(`InterruptGuard`, `Nvic`), `stm32g0/ticker.hpp` (`BasicTicker`). The
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
construction. What the WFI enters is SLEEP MODE and nothing deeper:
SCR.SLEEPDEEP is 0 out of reset and this stratum never writes it, so
HCLK, SysTick and every peripheral keep running (5.3). The deeper
modes are PWR's and arrive with the sleep site.

**SysTick rides HCLK.** The reload is `Clock::hz / 1000 - 1`
(63999 at 64 MHz, read back over SWD); in Stop the core clocks stop
and so does the kernel's time - the samc standby situation, with the
same two answers waiting for the PWR pass (ticker.hpp).

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
  `interrupts_enabled()`, `irq_priority_levels` (4) - nvic.hpp.
- `Nvic` - `enable`/`disable`/`enabled`, `set_pending`/`clear_pending`/
  `pending`, `priority` (refuses a level the core does not have), by
  the header's IRQn value.
- `BasicTicker<tps>` / `Ticker` (1000 Hz) - `init(clock)` (false when
  the reload does not fit 24 bits), `tick()` (the ISR body), `ticks`/
  `millis`/`secs`/`now`, `advance(n)` (the future resync's landing
  point), `pause`/`resume`. A rate that does not divide 1000 is
  refused at compile time.
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
- The FAILING half: RCC_CSR's reset-cause flags, SYSRESETREQ, IWDG and
  WWDG, a `ResetReporter` and a HardFault body that writes the
  breadcrumb (the samc/reset.hpp shape) - today a fault spins.
- The STOPPING half: PWR's five modes, SLEEPDEEP + LPMS, the wake-up
  pins, the voltage-range setter (Range 2), the `SleepSite` adapter and
  the RTC-backed timebase resync - `advance()` exists for it.
- VTOR relocation; NVIC priorities are exposed but nothing assigns one.
- `delay_us` (the samc SysTick-VAL busy-wait has no twin here yet).
- A per-package pin-bonding table.

Implemented, not bench-verified: `Nvic::priority`, `advance()`,
`pause()`/`resume()`, `abort()`'s and `HardFault_Handler`'s spins,
the cortex-debug launch entry.
