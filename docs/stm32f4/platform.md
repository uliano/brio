# Platform - what the kernel stands on (STM32F4)

This document is for a programmer putting a brio program on an STM32F4.
It describes what the kernel stands on here, the platform, and the
timebase a program runs its time events on. A brio kernel asks the chip
for a handful of things, stated by the kernel's `Platform` concept (a
concept is a compiler-checked set of requirements on a type;
[design/kernel.md, "Platform: what the machine provides"](../design/kernel.md#11-platform-what-the-machine-provides-kernelplatformhpp)):
a way to mask interrupts for a critical section, a way to sleep until
the next interrupt without losing one that arrives at the wrong moment,
a source of time, a breakpoint for a panic, and a record a panic leaves
for the next boot. `Stm32f4Platform<TB>` is this family's answer, and
`TB` is its timebase: the SysTick `Ticker`, a 1000 Hz interrupt from the
core's own timer, which stops when the chip enters Stop mode. The kernel
never enters Stop by itself (its idle writes no sleep depth, so a bare
idle is Sleep, where the tick runs); a Stop is armed by a sleep site, the
object a program's power policy uses to choose a deeper mode, and the
timed sleep site of [pwr.md](pwr.md) repairs kernel time afterwards. The Cortex-M4
has BASEPRI and sixteen interrupt priority levels, and this platform uses
neither: every interrupt line stays at one priority, so no interrupt
handler runs inside another, which is what the kernel is written against
([design/kernel.md, "The model in one page"](../design/kernel.md#1-the-model-in-one-page)).

The smallest use, the "How to use it" block below without its includes
(the maintained example `blink`, `stm32f4/src/apps/blink.cpp`, has the
same glue with its two active objects and each board's clock):

```cpp
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

Line by line: `P` names the platform with its default timebase, the
SysTick `Ticker`; `SysClock` is the clock task, the PLL at 180 MHz from
an 8 MHz HSE, a crystal by the type's default fourth argument (a board
whose HSE is a driven clock names `HseMode::bypass`, [clock.md](clock.md)),
and `clock` an empty tag object of
that type, which the drivers take to read the rate; the `extern "C"`
function is the SysTick interrupt handler, as in C, under the name ST's
startup code gives the vector, and its body is the ticker's; `main()`
brings the clock up, programs SysTick from the clock's rate, enables
interrupts and hands the core to the kernel. `Tenuto` takes the platform
first and the program's active objects after it, from the highest
priority to the lowest (`MyAo` here stands for them: an active object is
a component of a brio program with its own event queue, served one
event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)); `run()` never
returns. A short wait inside a handler is `delay_us()`, which answers
false and waits nothing for a millisecond or more ("How to use it"
below).

Three things decide whether that works. The clock task must have run
first: `Ticker::init()` computes SysTick's reload from the rate the
clock type claims and not from the hardware, so a ticker initialized
while the chip still runs at its 16 MHz reset rate under a 180 MHz clock
type ticks about eleven times too slowly, and nothing refuses it. The
SysTick vector must be bound to `Ticker::tick()` by the program, as
above: the kernel binds no vector, and the crt's `SysTick_Handler` is a
weak alias of `Default_Handler`, a silent spin, so a program that
forgets the binding links and stops in that spin at its first tick. And
interrupts must be enabled before `run()` (`brio::enable_interrupts()`),
since the kernel sleeps between events and the tick is what brings it
back.

The rest of this page, in order: the documents of record; what the
silicon does (the interrupt masking, the vector table, the FPU's enable
in the crt, the fault vectors, the idle hook, where a pending interrupt
is taken, SysTick, `delay_us`); the types and verbs; how to use it; the
bench findings, measured to the cycle, with the case against running
code from SRAM on this family; and what is not covered.

## Documents of record

PM0214 Rev 10 (the Cortex-M4 core: PRIMASK and
BASEPRI, the NVIC, SysTick, the FPU's CPACR and lazy stacking, the
ISB, the sleep's entry and wake-up), ARM's ARMv7-M ARM (DDI 0403E.e:
the WFI's wake-up events, SysTick's reload) and the Cortex-M4 TRM (DDI
0439B: the exception entry's and return's cycles), RM0090 Rev 22 / RM0390 Rev 6 / RM0383 Rev 4 for the vector
tables (RM0090 table 62 and its twins) and for what a WFI here enters
(PWR ch. 5, which [clock.md](clock.md)'s regulator half touches and
[pwr.md](pwr.md) owns), and the three errata sheets ES0206
Rev 24, ES0298 Rev 8, ES0287 Rev 6 - the core items 2.1.1 (interrupted
loads to SP), 2.1.2 (VDIV/VSQRT under very short ISRs) and 2.1.3
(store immediate overlapping an exception return) are the Cortex-M4
r0p1's and stand on every part; none of the three shapes this chapter's
code, and 2.1.2 is a note for the day a float divides in a handler.
Drivers: `stm32f4/platform.hpp` (`Stm32f4Platform<TB>`, this target's
realization of the kernel's `Platform` concept, templated on its
timebase), `stm32f4/ticker.hpp` (the SysTick `Ticker`: this family's
include of the core stratum's `cortexm/ticker.hpp` - `BasicTicker`,
`SysTickCounter`), `stm32f4/delay.hpp` (the microsecond busy-wait),
`stm32f4/nvic.hpp` (`cortexm/nvic.hpp` - `InterruptGuard`, `Nvic`;
[../cortexm/README.md](../cortexm/README.md)). The crt is
`stm32f4/src/glue/startup_stm32f4{29,46,11}.cpp` + `stm32f4/ld/<part>.ld`
in the build project. The family fixture is
`test/family_stm32f4/platform.cpp` under `brio check stm32f4`. The
reference suite is `test_stm32f4_platform` (letters a..e, g, w, k, t),
run on the four boards; a letter is one command key of the suite's
console and the group of verdicts it runs, and `z` is the key that runs
them all.

## What the silicon does

**Interrupt masking is PRIMASK here, though the core has more.**
ARMv7-M adds BASEPRI - "mask everything at or below priority N" - and
sixteen NVIC priority levels (four bits) where the M0+ had four. brio
uses neither: the kernel promise
([design/kernel.md, "Platform: what the machine provides"](../design/kernel.md#11-platform-what-the-machine-provides-kernelplatformhpp)) is that
no interrupt nests over another, and this family keeps it the way the
SAM C21 does - every NVIC line at the same priority (the reset value,
0), PRIMASK the one mask, `Nvic::priority()` the one door and no driver
opens it. BASEPRI is the preemptive kernel's tool and waits for it.
Core exceptions (SysTick among them) have no NVIC enable bit.

**The vector table is 16 + 91 entries on the F429, 97 on the F446, 86
on the F411, 93 on the F469 (QUADSPI and the DSI host after the DMA2D),
and on this family a line is mostly one peripheral's.**
The sharing the table shows is the advanced timers' (TIM1_BRK_TIM9,
TIM1_UP_TIM10, TIM1_TRG_COM_TIM11, TIM8's with TIM12/13/14), TIM6 with
the DAC, the EXTI lines 5..9 and 10..15 grouped, the I2C's two vectors
per instance; every serial instance has a line of its own. The device
header gives the line's IRQn and ST's startup template the handler's
spelling (`USART1_IRQHandler`) - the header declares no handler names,
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
every `panic()` ends in - is legible in a backtrace. `stm32f4/reset.hpp`
has their switch (`Faults`) and a body a program may bind to them
(`hard_fault_reset<P>()`, [reset.md](reset.md)); a fault vector the
program leaves unbound is a weak alias that spins in Default_Handler.

**WFI wakes on a pending interrupt even under PRIMASK**, so the idle
hook sleeps first and unmasks after, closing the lost-wakeup window by
construction: the processor "ignores the value of PRIMASK in
determining whether an asynchronous exception is a WFI wakeup event"
(ARMv7-M ARM B1.5.19, PM0214 2.5.2), and a WFI is not WFE - no event
latch is left standing by a wake, so the loop turns once per interrupt.
Both are measured to the cycle (letters `w` and `k`, below). What the
WFI enters is whatever is armed: SCR.SLEEPDEEP is
0 out of reset and this file never writes it, so a bare WFI is Sleep -
HCLK, SysTick and every peripheral keep running (RM0090 5.3.4) - and
with a sleep site having armed a Stop, the same WFI is that Stop.

**A pending interrupt is taken after the instruction that lifts the
mask, not inside it** (measured, letter b): a SysTick tick left pending
under a 5-period critical section is delivered exactly once, and a read
of the tick count in the instruction after the guard's `msr PRIMASK`
sees it not yet delivered - 0 ticks advanced - while the same read
after an ISB sees 1. PM0214 2.1.3's rule ("an ISB ... ensures that the
effect of the change is seen by the following instructions"); on the
M0+ families the same suites read the count after a ruler read, which
took the cycles an ISB takes here. A masked window loses ticks: the
pending bit is one bit, five periods under it are one interrupt.

**SysTick rides HCLK.** The reload is `Clock::hz / 1000 - 1` (179999 at
180 MHz, 99999 at 100 MHz, read back by the suite), well inside the
24-bit field; in Stop the core clocks stop and kernel time stands still
for the sleep's duration - the sleep sites of [pwr.md](pwr.md) are
where that gets repaired.

**`delay_us` reads SysTick's VAL** (`cortexm/delay.hpp`: at least, never
early, capped below one tick, no division at wait time - a rule that
costs this core nothing, ARMv7-M having UDIV, and is kept for the one
code path). VAL is 5.6 ns of resolution at 180 MHz; the call's own
overhead is about 150 cycles at 180 MHz and 50 at 100 MHz (measured:
1057 cycles for a 5 us wait at 180 MHz, 553 at 100 MHz).

## Types and verbs

- `Stm32f4Platform<TB = Ticker>` - `Timebase` (= TB), `CriticalSection`
  (= `InterruptGuard`), `idle()` (DSB, WFI, unmask),
  `interrupts_enabled()`, `in_handler()` (kernel/platform.hpp's
  optional member: IPSR's exception number, zero in thread mode,
  ARMv7-M ARM B1.4.2, cortexm/nvic.hpp's read - measured true in a
  USART vector and false in thread mode and under a critical section
  by the serial suite's letter h, [usart.md](usart.md)),
  `break_here()` (BKPT, unconditional - ARMv7-M
  lets software read DHCSR's C_DEBUGEN, unlike ARMv6-M, and a conditional
  break is possible here; it is not taken, so the three families' panics
  end the same way), `now()`/`ticks_per_second` (the timebase's),
  `atomic_width` 4, `panic_record()` in `.noinit`. No `idle_until()`: no
  timebase of this family counts through Stop yet.
- `Stm32f4RubatoPlatform<TB = Ticker>` - `Stm32f4Platform<TB>` whole plus
  the three members of a preemptive platform, the one the preemptive
  kernel `Rubato` runs on ([design/kernel.md](../design/kernel.md),
  "Rubato"): `preemptive`, `ceiling` (the index of the object being
  dispatched, 255 at the loop's base) and `pend_schedule()`, which is
  `cortexm/schedule.hpp`'s request: PendSV pended, at the lowest of the
  sixteen levels, below every NVIC line at its reset priority 0. PendSV
  returns into the scheduler in thread mode and the scheduler leaves
  through SVC, so a program on this platform binds two vectors more
  than a Tenuto one, `PendSV_Handler` and `SVC_Handler`, both naked,
  to `scheduler_exception<K>()` and `scheduler_return()`, calls
  `scheduler_init()` before it enables interrupts, and executes no
  `svc` of its own. The critical section stays PRIMASK; the peripheral
  handlers still never nest among themselves.
- `Ticker` = `BasicTicker<1000>` (`cortexm/ticker.hpp`): `init(clock)`
  (the reload from the clock's rate, refused when it does not fit),
  `tick()` (the ISR body the app binds to `SysTick_Handler`), `ticks()`,
  `millis()`, `secs()`, `now(TimeStamp&)`, `rebase(hz)` for a dynamic
  clock, `advance(n)` for a sleep site's resync.
- `delay_us(clock, us)` / `delay_us(DelayRate, us)`, `delay_rate(hz)`
  (`cortexm/delay.hpp`).
- `InterruptGuard`, `enable_interrupts()`, `disable_interrupts()`,
  `interrupts_enabled()`, `Nvic` (enable/disable/enabled, set_pending/
  clear_pending/pending, priority get/set over sixteen levels),
  `irq_priority_levels` = 16 (`cortexm/nvic.hpp`).
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
dispatch: `if (!brio::delay_us(clock, 20)) { ... }` - it answers false,
spending no time, when SysTick is not counting or the request is a
millisecond or more, which is a `TimeEvent`'s work.

The same program under the preemptive kernel: the platform type, the
two naked vectors and one call more.

```cpp
#include "kernel/rubato.hpp"

using P = brio::Stm32f4RubatoPlatform<>;
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

`test_stm32f4_rubato`, 37 verdicts on the Nucleo-F446RE, the preemptive
kernel on `Stm32f4RubatoPlatform` with the scheduler in thread mode;
CYCCNT the ruler at 180 MHz, TIM9's compare the interrupt the road-1
letters land, nothing wired and nothing lit:

- **The ceiling, both roads** (letters a, b, e): an interrupt's post to
  High inside Low's handler runs High to its end before Low resumes,
  every handler reading its own index as the ceiling, ten runs alike;
  the UART's interrupt reads the loop's ceiling as the letter's key
  arrives, and the verdict accepts both values the loop holds below
  every object - 255 at its base, 0 while its base turn walks the time
  events (kernel/rubato.hpp holds it there so that a timer's fire only
  queues; no dispatch of Top, whose index is also 0, can be open then,
  nothing in the suite being able to post to Top when a key arrives) -
  and read 255 in every run, twenty of letter `a` alone and ten of
  `z`; Low posting to Mid and Mid to High nests High inside Mid inside
  Low, each inside the post that asked; High's post to Low is served
  after High returns and after Low's own dispatch ends; and at depth 2,
  an interrupt over High's
  handler (started by either road) runs Top before High resumes, Top's
  first statement 163 to 168 cycles after the handler's last in every
  set of ten events, twenty sets over ten runs of the suite, one by each
  road; the letter does not read the kernel tick around its events, so
  an event a tick lengthened would go unattributed.
- **The latency of road 1** (letter c): from the interrupt handler's
  last statement to High's handler's first, 173 to 186 cycles on every
  event no kernel tick lands in, a hundred events in each of ten runs
  (173 the least, 186 the median and the most); 231 and 248 when a tick
  lands in the path (one event in three of the ten runs). The sum of the path's own counts in
  the listing (the handler's exit, PendSV's entry by tail-chaining, the
  fake frame's return, the entry with its masked store of the ceiling,
  `activate()`'s read of the count of quiet posts, its looks and the
  dispatch's call, every refill at its most) is 165 cycles, 81
  instructions, short of the measurement. A diagnostic build whose one
  difference is a print in this letter's own code - every function on
  the path at the same address, instruction for instruction - measures
  162 on nearly every event: the up to 21 cycles the suite's image adds
  are the flash accelerator's line refills, decided by the code around
  the path - the finding the STM32G0's page makes of its own flash
  ([../stm32g0/platform.md](../stm32g0/platform.md), "Code in SRAM"): a
  path's cost from flash moves with where the code sits. So the bound,
  256, is the quiet path's slowest event (186) plus one tick (58) and
  the flash accelerator's misses (12).
- **The scan and the post** (letter d): `activate()` with four empty
  queues below the ceiling, entered and left by hand as the platform's
  entry and exit do, 124 cycles (213 at worst; the listing's count 140),
  each queue's two cursors loaded and compared in line, since the
  queue's header forces `empty()` inline; a post that asks for nothing
  95 (the count 85, eleven of it the count of quiet posts moved); a post
  that runs High's empty dispatch inside it 472 (the count 475), the
  scheduling's share 377.
- **Time events under preemption** (letter f): three timers matured in
  one turn, the lowest object's armed last, are posted with the ceiling
  held at 0 and served High, Mid, Low; High preempting Low's loop of
  `arm()` and `disarm()` by road 1 leaves the list whole, every timer
  due fired once, the disarmed one never; one `arm()` 79 cycles with
  its masked window inside.
- **The FPU across a preemption** (letter g): Low's float series
  interrupted at a mark, High's computed inside, the handler computing
  a float too: every sum bit-exact against the same code run without
  preemption, the lazy context save proved by the values.
- **The stack** (letter h): 712 bytes of high-water for the suite's
  deepest chain, the deepest sampled stack pointer at 568 bytes in a
  dispatch and 512 in the interrupt, the mark within the 350 bytes the
  deepest action's calls (a line through the writer), an exception frame
  with the FPU's words and the handler's frames add. The SVC frame of
  the exit is basic and stands where a returned `activate()`'s frame
  stood, and the one run a request made in PendSV's first instruction
  can start at an entry (kernel/rubato.hpp, "The stack") reaches 44
  bytes below the dispatch of the run it delays, inside the same 350.
- **Dark** (letter i): every object idle, no timer armed, TIM9 quiet,
  PA5 not an output, the ceiling the Console's, interrupts enabled.
- **A compiler hazard on road 2, found and closed**: GCC 16 at `-Os`
  deleted as dead a store the receiving dispatch inside `post()` read,
  the dispatch being reached through an exception the compiler cannot
  see and the DSB's memory clobber not stopping it; the request is
  a call the compiler cannot see into (`noinline`, `noipa`), one call
  and return on the asking path only.
- **A second compiler hazard, in the kernel, found by the link suite
  and closed**: the same compiler moved the queue's two cursor loads
  ahead of the ceiling's raise in the scheduler's pass; a request
  landing in that gap served the event in a nested scheduler and the
  outer pass took it again and released the slot twice, wedging the
  queue after half a second to nine seconds of pressure. The raise and
  the restore are compiler barriers (`kernel/rubato.hpp`), and the
  suite below runs clean over them.

`test_stm32f4_rubato_link`, 230 verdicts on the Nucleo-F446RE (its
target-free half the link suites' shared body,
`avrxt/src/apps/rubato_link.hpp`) against
the Nucleo-G0B1RE's unchanged `spi_peer` and `usart_peer` over the desk's
SPI2 and USART1 wires: the real SPI and UART stacks under Rubato with
an object above them preempting on TIM2 at 97, 193 and 389 us, its
dispatch 300 then 3000 cycles, every letter run first without pressure
as its control; three runs of each key, every verdict green. The peer's
command channel through the arbiter, the
matrix on the DMA engines (152 kB/s, 963 to 1000 per mille of the
control under pressure), the arbiter with three requesters of three
priorities (every reply to its requester in order, no timeout), the
UART's numbered lines echoed at 460800 and 921600 through `LineReader`
into a borrower above it (no line lost, out of order or corrupt, every
port counter zero, 876 to 926 per mille of the wire), the one-writer
blast judged by the peer, the loan with a slow borrower preempted
inside its dispatch 19 to 114 times a pass: all byte-exact. The one
limit found is the polled loop's, [spi.md](spi.md)'s "Not covered
yet". Also observed: the peer's command window sent right after an
answer window is not heard until the peer is back in command mode
(the suite waits between windows); the receive engine tells the port
of a stream only at an idle line or a half lap, up to 1024 characters.

`test_stm32f4_rubato_i2c`, 195 verdicts on the Nucleo-F446RE, and its
twin `test_stm32f4_rubato_fmpi2c`, 174 (their target-free half the I2C
suites' shared body, `avrxt/src/apps/rubato_i2c.hpp`), against the
Nucleo-G0B1RE's unchanged `twi_peer` over the desk's I2C node: the I2C
stack - `util/i2c_bus.hpp`'s arbiter over `I2cHost<1>` on I2C1 and, in
the twin, over `FmpI2cHost<1>`, each with its DMA engines - under Rubato
with the same pressure on TIM2, every letter run first without it as
its control; every verdict green on every run ([i2c.md](i2c.md) and
[fmpi2c.md](fmpi2c.md) say how many of each letter). The command channel through the arbiter, the
tenure shapes on every path of each host, the speeds (100 and 400 kHz,
and 1 MHz on the FMPI2C1) and the partner's commanded stretch, three
requesters of three priorities, and a held wire with requests arriving
in the arbiter's draining window from above it: every request answered
once, every byte exact - and the status vocabulary on both blocks. A
preemption inside a host's `start()` delays the START and never
stretches a tenure. An address the classic block takes as a 10-bit
header wedges that block ([i2c.md](i2c.md)).

`test_stm32f4_platform`, letters a..e and g, 36 verdicts on each of the
four boards, and letters `w`, `k` and `t` beside them in `z` (49
verdicts on the Nucleo-F446RE):

- **a**: DEV_ID 0x419 REV_ID 0x2003 with 2048 KB on the DISC1, 0x421 /
  0x1000 / 512 KB on the Nucleo-F446RE, 0x431 / 0x1000 / 512 KB on the
  black pill, 0x434 / 0x1000 / 2048 KB on the 32F469IDISCOVERY; the
  unique ids read; no breadcrumb pending on a clean boot.
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
  counter on all four (1057 / 5450 / 18054 / 90058 / 162059 cycles at
  180 MHz on the DISC1, 968 / 5450 / 18054 / 90055 / 162061 on the
  32F469IDISCOVERY); a thousand 100 us waits are 100..101 ms of kernel
  tick; a wait of one period or of 65536 us is refused.
- **e**: on all four SWS reads PLL, the PLL is locked, the HSE is
  ready in the board file's mode (bypass on the Nucleo, crystal on the
  other three), VOS is scale 1, over-drive on where the rate needs it
  (the three 180 MHz boards) and off on the F411, the latency 5 / 5 /
  3 / 5, the accelerator on, PPRE1 /4 /4 /2 /4 and PPRE2 /2 /2 /1 /2,
  HPRE 1, PLLCFGR holding M 4 N 180 P 2 Q 8 (8 MHz roots), M 16 N 128
  P 2 Q 5 (25 MHz) - every register the task's constant.
- **g**: the record at 0x200002E8 in the main SRAM; nothing pending; a
  written record taken once with its code and context, then gone.
- **w**, on the Nucleo-F446RE at 180 MHz: SysTick's next edge placed D
  cycles after a write, D = 1..400 - LOAD set to D around the VAL write
  that reloads it and put back at once (PM0214 4.5.2's single-shot
  reload), every placement checked - and the kernel's shape run against
  it: a masked check, `idle()`, until one tick, until two (the second
  edge a whole period later), and a third pass with the edge on an NVIC
  LINE (TIM9's compare, its timer clock HCLK, the tick's interrupt held
  off so that a lost wake would be lost for good). TIM2 at 1 MHz times
  each try and rescues a lost one: 0 of 400 lost in each pass and none
  late - the slowest try 3 us for one tick, 1003 for two. The walk
  crosses the sleep: from D = 16 (the tick) and D = 8 (the line) on the
  edge found the core asleep, every nearer one came before the WFI and
  the WFI fell through on it. Two facts of this core shaped the letter:
  the first edge after other code ran pays the ART's misses (a line edge
  entered at 38 cycles cold against 24 warm), so each walk opens with
  one try it judges and does not measure; and a timer enabled by a store
  starts counting where the write buffer delivers the store, which the
  DSB in `idle()` waits for - so the walk reads the timer back after the
  enable, or every line edge lands after the WFI.
- **What a wake costs**, HCLK cycles at 180 MHz: an edge that finds the
  core asleep enters the tick's handler at 20 cycles against 16 when the
  core is awake and spinning unmasked (the Cortex-M4 TRM, DDI 0439B
  3.9.1, gives at most 12 to the handler's first instruction and 12 for
  the return, at zero wait states), reaches the handler's last statement at 52
  and the caller's loop at 90 (an awake spin is out at 69); a line edge
  enters at 25 against 21, the TIM9 count read through APB2 one cycle
  apart by the bus clock's phase, and reaches the loop at 78..79 against
  67. The handler's first statement is a VAL or count read, so each
  entry figure carries the instructions before that read. An `idle()`
  that a pending line returns at once costs 62 cycles, the line's
  handler round trip included. ST's `HAL_PWR_EnterSLEEPMode` is a bare
  WFI run unmasked, the lost-wake window left to its caller; this hook
  is DSB, WFI, CPSIE.
- **k**: a Tenuto pack of three quiet AOs, two with a periodic time
  event, turned as `run()` turns it with the tick the only interrupt:
  100 turns over 100 ticks. A quiet turn whose `idle()` a pending line
  returns at once costs 195 cycles, 62 of them that `idle()` and its
  handler and 133 the turn's own - `process()`, `step()` over three
  empty queues, the masked check.

**Code in SRAM, measured: it does not pay here.** At 180 MHz the flash
answers at five wait states (RM0390 3.4) behind the ART accelerator
(3.5: the prefetch, a 64-line instruction cache, an 8-line data cache),
all three on as the clock task leaves them. Whether a hot path runs
faster from SRAM - the question
[../../brio/util/placement.hpp](../../brio/util/placement.hpp) asks of
a family - was measured on the Nucleo-F446RE with `bench_stm32f4`'s
letters `e`, `u`, `i` and `p` in builds whose linker script collected
`.fast_text` into `.data`, each path three ways: in the flash as the
stratum builds it; its VECTOR placed (`.fast_text` on the program's
handler, every body inline in it moved with it); and the driver's body
alone as a `Hot<InRam>` instance called from a vector left in the
flash. The flash column was run again with PRFTEN, ICEN and DCEN
clear, to show what the accelerator hides. HCLK cycles, a handler's
figure per entry with the meter's stamps inside it:

| path | flash | flash, ART off | SRAM, vector placed | SRAM, body `InRam` behind a flash vector | SRAM bytes |
|---|---|---|---|---|---|
| SPI frame pump, an entry (one 8-bit frame, SCK 45 MHz) | 103 | 152 | 123 | 158 | 368 the body, 880 an engined host's |
| the same pump, 256 frames, the wall (the wire 8192) | 38199 | 56807 | 46386 | 51502 | |
| SPI receive loop, 256 8-bit frames at SCK 45 / 22.5 MHz (the wire 8192 / 16384) | 19110 / 26278 | 17714 / 25394 | - | 21764 / 28420 | 500 a host's polled loops |
| SPI transmit-only loop, 256 8-bit frames at SCK 45 / 22.5 MHz | 9392 / 16568 | 10012 / 16659 | - | 8472 / 16681 | |
| USART per-byte transmit, 5.625 Mbaud | 64 | 97 | 83 | 111 | 316 |
| USART per-byte receive, 1 Mbaud | 100 | 171 | 118 | 144 | 316 |
| USART transmit engine's completion, 5.625 Mbaud (the tick's entries mixed in alike) | 123 | 216 | 145 | 146 | 364 |
| the console's interrupt, a print of 4096 bytes (the tick's entries mixed in alike) | 61 | 88 | 67 | 101 | 312 |
| I2C1 event pump, a byte written / read | 103 / 122 | - | 91 / 118 | - | 2104 the suite's vector |
| FMPI2C1 event pump, a byte written / read | 136 / 151 | - | 164 / 166 | - | 1060 the suite's vector |

The accelerator hides the five wait states: a warm handler runs from
the flash faster than from SRAM, the pump a fifth faster. The core's
own bus map says why SRAM cannot win it back: code at 0x20000000 is
fetched over the System interface, which also carries every SRAM and
peripheral data access the handler makes, data first (DDI 0439B
2.2.1), where code in the flash comes over ICode beside them. The
exceptions are small and mixed with losses on the same paths - the
transmit-only loop at 45 MHz 9392 -> 8472 (the wire 8192) against the
receive loop 19110 -> 21764, I2C1's byte 103 -> 91 against FMPI2C1's
136 -> 164 - and the flash loops themselves move with layout (the
receive loop is FASTER with the accelerator off, 17714). A body behind
a flash vector pays the call besides: SRAM lies out of a BL's 16 MB
reach, so GNU ld puts an 8-byte stub in the path, `ldr.w pc, [pc]` and
the target's address - LDR to PC 2+P cycles against the BL's 1+P
(DDI 0439B 3.3.1, table 3-1, P 1 to 3) - and the pump's body there is
35 cycles an entry above the vector placed.

RECOMMENDATION: decline util/placement.hpp's opt-in on this stratum -
the ART already hides the wait states, and SRAM execution shares the
System bus with the handler's own data.

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
- NVIC priorities and BASEPRI: the peripheral lines stay at one level
  under both kernels (the preemptive one puts only PendSV below them),
  and BASEPRI is unused: a zero-latency tier above the critical section
  is a later decision ([design/kernel.md](../design/kernel.md),
  "Rubato").
- The three configurable faults' enables and bodies, the MPU, the FPU's
  exception flags: the reset chapter's. `reset.hpp` has the enables
  (`Faults`) and a body (`hard_fault_reset<P>()`); the MPU and the FPU's
  own fault sources are [reset.md](reset.md)'s gaps, with their reasons.
- The reset causes, the watchdogs and the breadcrumb across a real
  reset: the reset chapter's suite, `test_stm32f4_reset`
  ([reset.md](reset.md)).
- A per-package pin-bonding table - [port.md](port.md) says why not.
- Code run from SRAM aliased at address 0 (SYSCFG_MEMRMP), which the
  core would fetch over ICode rather than the System interface the
  Bench findings' "Code in SRAM" measured: not measured, because what
  is mapped at 0 is a boot decision and `syscfg.hpp` only reads it.

Implemented, not bench-verified: letters `w` and `k` on the F429ZI, the
F411CE and the F469NI (the same image, run there, is the measurement);
`Nvic::priority` (nothing assigns one,
above); `abort()`'s and `HardFault_Handler`'s spins (the fault vectors
are taken by `test_stm32f4_reset` with `hard_fault_reset` bound,
[reset.md](reset.md); the crt's spins, what runs where no body is bound,
no suite has run); the cortex-debug launch entries
([README.md](README.md)), not driven end to end - halt-and-dump through
OpenOCD's own console is what the findings were taken with;
`delay_us` against an INDEPENDENT ruler (the suite judges it on
SysTick's own counter and the kernel tick, both derived from HCLK; a
timer on the crystal or the RTC is the ruler the M0+ suites use, and
the timer chapter brings it); a scheduling request made with PRIMASK
set, served when the mask lifts, which this
family's suite does not exercise (the STM32G0's letter g does, on the
same core stratum's code); the preemptive kernel's stack bound under a
load that never lets the pack drain - at most one run per object, and
the one empty run a request made in PendSV's first instruction can
start at an entry (kernel/rubato.hpp, "The stack") - which no letter of
this family drives: letter h measures the chain the suite builds, and
the I2C suites' flood is paced; their letter h run unpaced, with the
stack painted and read, is what would measure it.
