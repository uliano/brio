# Platform - what the kernel stands on (STM32F4)

Documents of record: PM0214 Rev 10 (the Cortex-M4 core: PRIMASK and
BASEPRI, the NVIC, SysTick, the FPU's CPACR and lazy stacking, the
ISB), RM0090 Rev 22 / RM0390 Rev 6 / RM0383 Rev 4 for the vector
tables (RM0090 table 62 and its twins) and for what a WFI here enters
(PWR ch. 5, which [clock.md](clock.md)'s regulator half touches today
and the power chapter will own), and the three errata sheets ES0206
Rev 24, ES0298 Rev 8, ES0287 Rev 6 - the core items 2.1.1 (interrupted
loads to SP), 2.1.2 (VDIV/VSQRT under very short ISRs) and 2.1.3
(store immediate overlapping an exception return) are the Cortex-M4
r0p1's and stand on every part; none of the three shapes this chapter's
code, and 2.1.2 is a note for the day a float divides in a handler.
Drivers: `stm32f4/platform.hpp` (`Stm32f4Platform<TB>`, this target's
realization of the kernel's `Platform` concept, templated on its
timebase), `stm32f4/ticker.hpp` (the SysTick `Ticker`: this family's
include of the core stratum's `armv6m/ticker.hpp` - `BasicTicker`,
`SysTickCounter`), `stm32f4/delay.hpp` (the microsecond busy-wait),
`stm32f4/nvic.hpp` (`armv6m/nvic.hpp` - `InterruptGuard`, `Nvic`;
[../armv6m/README.md](../armv6m/README.md)). The crt is
`stm32f4/src/glue/startup_stm32f4{29,46,11}.cpp` + `stm32f4/ld/<part>.ld`
in the build project. The family fixture is
`test/family_stm32f4/platform.cpp` under `brio check stm32f4`. The
reference suite is `test_stm32f4_platform` (letters a..e, g), run on
the three boards.

## What the silicon does

**Interrupt masking is PRIMASK here, though the core has more.**
ARMv7-M adds BASEPRI - "mask everything at or below priority N" - and
sixteen NVIC priority levels (four bits) where the M0+ had four. brio
uses neither: the kernel promise (design/kernel.md section 11) is that
no interrupt nests over another, and this family keeps it the way the
SAM C21 does - every NVIC line at the same priority (the reset value,
0), PRIMASK the one mask, `Nvic::priority()` the one door and no driver
opens it. BASEPRI is the preemptive kernel's tool and waits for it.
Core exceptions (SysTick among them) have no NVIC enable bit.

**The vector table is 16 + 91 entries on the F429, 97 on the F446, 86
on the F411, and on this family a line is mostly ONE peripheral's.**
The sharing the table shows is the advanced timers' (TIM1_BRK_TIM9,
TIM1_UP_TIM10, TIM1_TRG_COM_TIM11, TIM8's with TIM12/13/14), TIM6 with
the DAC, the EXTI lines 5..9 and 10..15 grouped, the I2C's two vectors
per instance; every serial instance has a line of its own. The device
header gives the line's IRQn and ST's startup template the handler's
spelling (`USART1_IRQHandler`) - the header declares NO handler names,
so the crt cites the template for that one thing. The table has holes
where another part of the family has a peripheral this one has not
(the F446's positions 61-62, 79-80, 82-83, 85-86, 88-90), filled with
null. It lives at the start of flash (0x0800_0000) and is fetched
through the boot alias at 0 (BOOT0 low, the boards' default); VTOR
exists and is not written.

**The FPU is enabled by the crt before anything else runs.** The
project builds with the hard-float ABI, so the compiler may keep a
float in an s register anywhere - including the initializers
`.init_array` runs; with CPACR's CP10/CP11 fields at their reset value
(no access) the first such instruction is a UsageFault. Reset_Handler
writes full access, then DSB and ISB (PM0214 4.6.1), then copies .data,
zeroes .bss and walks the constructors. Lazy stacking (FPCCR.ASPEN and
LSPEN, set at reset) is left as it is: an exception stacks the FPU
context only if the handler uses the FPU, so a handler that does not
pays nothing.

**The three configurable faults are disabled at reset** (SHCSR's
MEMFAULTENA, BUSFAULTENA, USGFAULTENA clear) and escalate to HardFault,
which the crt gives its own spin so a BKPT with no debugger - what
every `panic()` ends in - is legible in a backtrace. The fault-handling
pass (the reset chapter) gives them bodies; until then their weak
aliases spin in Default_Handler.

**WFI wakes on a pending interrupt even under PRIMASK**, so the idle
hook sleeps first and unmasks after, closing the lost-wakeup window by
construction. What the WFI enters is WHATEVER IS ARMED: SCR.SLEEPDEEP is
0 out of reset and this file never writes it, so a bare WFI is Sleep -
HCLK, SysTick and every peripheral keep running (RM0090 5.3.4) - and
with a sleep site having armed a Stop, the same WFI is that Stop.

**A pending interrupt is taken AFTER the instruction that lifts the
mask, not inside it** (measured, letter b): a SysTick tick left pending
under a 5-period critical section is delivered exactly once, and a read
of the tick count in the instruction after the guard's `msr PRIMASK`
sees it NOT YET delivered - 0 ticks advanced - while the same read
after an ISB sees 1. PM0214 2.1.3's rule ("an ISB ... ensures that the
effect of the change is seen by the following instructions"); on the
M0+ families the same suites read the count after a ruler read, which
took the cycles an ISB takes here. A masked window LOSES ticks: the
pending bit is one bit, five periods under it are one interrupt.

**SysTick rides HCLK.** The reload is `Clock::hz / 1000 - 1` (179999 at
180 MHz, 99999 at 100 MHz, read back by the suite), well inside the
24-bit field; in Stop the core clocks stop and KERNEL TIME STANDS STILL
for the sleep's duration - the sleep sites of the power chapter are
where that gets repaired.

**`delay_us` reads SysTick's VAL** (`armv6m/delay.hpp`: at least, never
early, capped below one tick, no division at wait time - a rule that
costs this core nothing, ARMv7-M having UDIV, and is kept for the one
code path). VAL is 5.6 ns of resolution at 180 MHz; the call's own
overhead is about 150 cycles at 180 MHz and 50 at 100 MHz (measured:
1057 cycles for a 5 us wait at 180 MHz, 553 at 100 MHz).

## Types and verbs

- `Stm32f4Platform<TB = Ticker>` - `Timebase` (= TB), `CriticalSection`
  (= `InterruptGuard`), `idle()` (DSB, WFI, unmask),
  `interrupts_enabled()`, `break_here()` (BKPT, unconditional - ARMv7-M
  lets software read DHCSR's C_DEBUGEN, unlike ARMv6-M, and a conditional
  break is possible here; it is not taken, so the three families' panics
  end the same way), `now()`/`ticks_per_second` (the timebase's),
  `atomic_width` 4, `panic_record()` in `.noinit`. No `idle_until()`: no
  timebase of this family counts through Stop yet.
- `Ticker` = `BasicTicker<1000>` (`armv6m/ticker.hpp`): `init(clock)`
  (the reload from the clock's rate, refused when it does not fit),
  `tick()` (the ISR body the app binds to `SysTick_Handler`), `ticks()`,
  `millis()`, `secs()`, `now(TimeStamp&)`, `rebase(hz)` for a dynamic
  clock, `advance(n)` for a sleep site's resync.
- `delay_us(clock, us)` / `delay_us(DelayRate, us)`, `delay_rate(hz)`
  (`armv6m/delay.hpp`).
- `InterruptGuard`, `enable_interrupts()`, `disable_interrupts()`,
  `interrupts_enabled()`, `Nvic` (enable/disable/enabled, set_pending/
  clear_pending/pending, priority get/set over sixteen levels),
  `irq_priority_levels` = 16 (`armv6m/nvic.hpp`).
- The crt: `Reset_Handler` (CPACR, .data, .bss, .preinit_array,
  .init_array, main), `Default_Handler` and the weak `HardFault_Handler`
  spins, `abort()`, every peripheral vector as a weak alias the app
  overrides by defining the strong symbol.

## How to use it

```cpp
#include "stm32f4/clock.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/ticker.hpp"

using P = brio::Stm32f4Platform<>;
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
constexpr SysClock clock;

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main() {
    SysClock::init();
    brio::Ticker::init(clock);
    brio::enable_interrupts();
    brio::Tenuto<P, MyAo>::run();   // WFI between events, woken by the tick
}
```

A vector is bound by defining its strong symbol beside `main()`:
`extern "C" void USART1_IRQHandler() { ... }`. A short wait inside a
dispatch: `brio::delay_us(clock, 20);` - a millisecond or more is a
`TimeEvent`.

## Bench findings

`test_stm32f4_platform`, letters a..e and g, 36 verdicts on each of the
three boards:

- **a**: DEV_ID 0x419 REV_ID 0x2003 with 2048 KB on the DISC1, 0x421 /
  0x1000 / 512 KB on the Nucleo-F446RE, 0x431 / 0x1000 / 512 KB on the
  black pill; the unique ids read; no breadcrumb pending on a clean
  boot.
- **b**: the guard masks, nests and restores (an inner scope's exit
  leaves the mask on); five SysTick periods under the mask advance the
  tick by ONE, read after an ISB (0 read before it - the finding above);
  with the console silent, two `idle()` calls reach the next tick and
  the hook returns unmasked.
- **c**: LOAD 179999 / 99999 as `Clock::hz / 1000 - 1`; CLKSOURCE,
  TICKINT and ENABLE set; millis() == ticks(), secs() and now()
  coherent; 200 ticks accumulated by VAL deltas against the interrupt
  count within 6, 62 and 51 cycles of 36 000 000 / 20 000 000 - the
  arithmetic `delay_us` is built on holds across 200 wraps.
- **d**: `delay_us` of 5, 30, 100, 500 and 900 us at least on its own
  counter on all three (1057 / 5450 / 18054 / 90058 / 162059 cycles at
  180 MHz); a thousand 100 us waits are 101 ms of kernel tick; a wait
  of one period or of 65536 us is refused.
- **e**: on all three SWS reads PLL, the PLL is locked, the HSE is
  ready in the board file's mode (bypass on the Nucleo, crystal on the
  other two), VOS is scale 1, over-drive on where the rate needs it
  (the two 180 MHz boards) and off on the F411, the latency 5 / 5 / 3,
  the accelerator on, PPRE1 /4 /4 /2 and PPRE2 /2 /2 /1, HPRE 1,
  PLLCFGR holding M 4 N 180 P 2 Q 8 (8 MHz roots), M 16 N 128 P 2 Q 5
  (25 MHz) - every register the task's constant.
- **g**: the record at 0x200002E8 in the main SRAM; nothing pending; a
  written record taken once with its code and context, then gone.

Two desk facts outside the letters: OpenOCD's HLA transport returns
garbage for memory read while the core sleeps in WFI (the vector table
as ASCII, 0x101 in every USART register) and exact values after a halt;
and a console whose clock init failed (a bypass on a board with a
crystal) prints at the WRONG BAUD - the divisor for a 90 MHz bus on a
16 MHz one - which reads as one byte of noise on the host; the CLK
verb, or a halted read of RCC_CR, tells the two apart.

## Not covered yet

Driver gaps (this chapter's option space the stratum does not touch):
- VTOR relocation, which nothing here needs while the vector table is
  the crt's and sits where the part boots; born with its first user (a
  bootloader).
- NVIC priorities and BASEPRI: exposed and unassigned, respectively
  unused, because the kernel is flat - every interrupt is one hardware
  level above the loop, and a priority scheme is the preemptive
  kernel's question ([design/kernel.md](../design/kernel.md)).
- The three configurable faults' enables and bodies, the MPU, the FPU's
  exception flags: the reset chapter's, born with `reset.hpp`.
- The reset causes, the watchdogs and the breadcrumb across a real
  reset: the reset chapter's suite.
- A per-package pin-bonding table - [port.md](port.md) says why not.

Implemented, not bench-verified: `Nvic::priority` (nothing assigns one,
above); `abort()`'s and `HardFault_Handler`'s spins (no suite has
faulted the part yet); the cortex-debug launch entries
([README.md](README.md)), not driven end to end - halt-and-dump through
OpenOCD's own console is what the findings were taken with;
`delay_us` against an INDEPENDENT ruler (the suite judges it on
SysTick's own counter and the kernel tick, both derived from HCLK; a
timer on the crystal or the RTC is the ruler the M0+ suites use, and
the timer chapter brings it); the FPU itself - no brio program executes
a float instruction yet, so CPACR's enable is proven only by the
absence of a UsageFault, not by a computed result.
