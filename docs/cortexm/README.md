# Core stratum: Cortex-M (`cortexm/`)

`brio/cortexm/` is one of the directories under `brio/` that is neither
target-independent nor a target: what ARM designed into every Cortex-M
and every vendor ships unchanged, written once and included by each
Cortex-M family after its own device header. This page is for a reader
of a family's headers who meets an include from `cortexm/` and wants to
know what it holds, what stays per family, which cores it has been
proven on and the one rule for including it. The stratum has four
files; nothing in them knows a vendor, and a program never includes them
directly.

The cores it covers are the M0/M0+ of three families (the SAM C21, the
STM32G0 and the RP2040), the M4 of the STM32F4 and the M33 half of the
RP2350: one programmer's model over the ARMv6-M, ARMv7-M and ARMv8-M
architectures.

## The five files

- **`cortexm/nvic.hpp`**: the NVIC and PRIMASK. `InterruptGuard` (the
  critical section every brio platform on these cores rides on), the
  global enable, disable and readback verbs, `in_handler()` (IPSR's
  exception number, zero in thread mode: the platforms' answer to the
  kernel's optional question), `Nvic` and `irq_priority_levels`.
- **`cortexm/ticker.hpp`**: the SysTick timebase. `BasicTicker`, the
  kernel's ticker, whose `cycles()` composes a cycle count from five
  reads in `util/cycle_count.hpp`'s order (the tick count, the
  exception's pending bit, VAL, the pending bit and the tick count
  again), judged by its `ticker_consistent()` and composed by its
  `ticker_compose()`, the pending bit covering the window between the
  reload and the handler; with LOAD read for the period, some thirty
  Thumb instructions on an M0+ and no call. And `SysTickCounter`,
  SysTick as a bare cycle counter with no interrupt, for a program whose
  kernel timebase is elsewhere, such as the STM32G0's tickless LPTIM
  one. Both are `ClockUser`s whose `rebase(hz)` reprograms the reload
  for a dynamic clock, a new LOAD and a restarted period with CTRL
  untouched by either, so the ticker loses the phase of the tick in
  progress: under a tick, late and never early. Both were measured at
  every rung of the STM32G0's ladder, the ticker holding 1000 Hz against
  the crystal at 64, 16 and 2 MHz.
- **`cortexm/delay.hpp`**: the microsecond busy-wait on SysTick's own
  counter. `delay_us`, `delay_rate` and `DelayRate` wait "at least" and
  never early, are capped below one SysTick period of one millisecond,
  and divide nothing at wait time: the arithmetic is folded for a
  static clock, and for a dynamic one selected by its rate index out of
  `delay_rates`, a per-rate table built at compile time over the clock's
  discrete-rate surface. It needs the counter running, never the
  interrupt, so it serves both writers of SysTick alike.
- **`cortexm/dwt.hpp`**, for the cores that have it: the DWT's cycle
  counter, `CycleCounter`. CYCCNT is the free-running count of processor
  cycles that ARMv7-M and ARMv8-M main define and ARMv6-M has not; it is
  enabled by TRCENA and then CYCCNTENA, and only where each reads clear,
  because a probe may have set them and nothing promises it leaves
  them; the count is never written; `now()` is one load; the file is
  refused by `static_assert` below the M3. On the STM32F4, measured, it
  counts through a WFI whatever DBG_SLEEP holds (`stm32f4/dwt.hpp`).
- **`cortexm/schedule.hpp`**: the scheduling request of the preemptive
  kernel, Rubato ([design/kernel.md](../design/kernel.md), "Rubato"),
  and the place its scheduler runs. `request_scheduler()` pends PendSV
  (PENDSVSET, then DSB and ISB, so a request from thread mode is taken
  before the next instruction and one from a handler at the handler's
  exit); `scheduler_init()` puts PendSV at the lowest priority the core
  implements, below every peripheral line; `scheduler_exception<K>()`,
  the naked body a program binds to `PendSV_Handler`, builds a second
  exception frame and returns through it into `scheduler_thread<K>()`,
  which runs the kernel's `activate()` in thread mode with PendSV no
  longer active, so an interrupt's post preempts a dispatch at any
  depth; `scheduler_return()`, the body bound to `SVC_Handler`
  (`SVCall_Handler` on the SAM C21, whose crt names the core exceptions
  as its device header does), steps over the SVC's frame and returns
  into the real one. The frames, the
  FPU's lazy context and EXC_RETURN's bits are the header's account.
  The assembly is the ARMv6-M subset, so one source serves the M0+, the
  M4 and the M33, and five platforms have run it: the STM32F4's
  Cortex-M4 (`stm32f4/platform.hpp`'s `Stm32f4RubatoPlatform`, measured
  on the Nucleo-F446RE, [../stm32f4/platform.md](../stm32f4/platform.md)),
  the STM32G0's Cortex-M0+ (`stm32g0/platform.hpp`'s
  `Stm32g0RubatoPlatform`, measured on the Nucleo-G0B1RE - PendSV at
  0xC0, the frames basic, a request under PRIMASK served at the unmask,
  [../stm32g0/platform.md](../stm32g0/platform.md)), the SAM C21's
  Cortex-M0+ (`samc21/platform.hpp`'s `SamRubatoPlatform`, measured on
  the SAMC21J18A: road 1 from an interrupt's last statement to the
  preempting dispatch's first in 232 cycles, the SVC vector bound by
  that crt's name `SVCall_Handler`, a PendSV pending at the idle's WFI
  ending it, [../samc21/platform.md](../samc21/platform.md)), the
  RP2040's (`rp2040/platform.hpp`'s `Rp2040RubatoPlatform`, one per
  core, each core pending and taking its own PendSV, the one vector
  table's entry bound to `rp2040/schedule.hpp`'s pair body when both
  cores run the kernel, measured on the WeAct RP2040: road 1 in 197
  cycles, a Rubato on each core preempting at once through that entry,
  [../rp2040/platform.md](../rp2040/platform.md)) and the RP2350's
  Cortex-M33 (`rp2350/platform.hpp`'s `Rp2350RubatoPlatform`, measured
  on the WeAct RP2350B: road 1 in 103 cycles,
  [../rp2350/platform.md](../rp2350/platform.md)). The ARMv8-M facts the
  M33 rests on are checked against the documents there: EXC_RETURN's
  security bits (ES, DCRS and S, all 1 in the Secure state the image
  runs in) are carried unchanged from the value PendSV entered with,
  MSPLIM is zeroed by that family's crt, and PRI_14's write lands on
  the core's four bits as 0xF0. The header's lazy FP context save
  assumes FPCCR.ASPEN and LSPEN at 1, the architecture's reset value;
  the RP2350's own register table gives both a reset value of 0, and the
  register read after the bootrom is 0xC000001C, both bits 1, the FP
  registers bit-exact across a preemption there - the measurement
  contradicts the table. One barrier in the PendSV body is the
  Cortex-M4's alone: erratum 838869 (the STM32F4's r0p1) wants a DSB
  before the exception return, and the body carries it on that core and
  on no other (the device header's `__CORTEX_M`), the rule being no
  barrier where no erratum asks for one.

## Why the stratum exists

brio's naming rule factors a core stratum at the second ARM family,
never earlier. `samc21/` and `stm32g0/` carried these files as twins,
line for line: the first two until the STM32G0's bring-up, the third
until the STM32G0's fillers were done. Every extraction was gated by the
images: every SAM C21 and STM32G0 release image was byte-identical
before and after.

## What lives here, and what does not

Here: code that reads only CMSIS-Core symbols (`__get_PRIMASK`,
`NVIC_*`, `SysTick`, `IRQn_Type`, `__NVIC_PRIO_BITS`) and nothing of any
vendor header. Each family's `delay.hpp` keeps what was measured on its
silicon (the division's cost, the counter's resolution, which sleep
stops it) beside the include.

Not here, by design: the platform type (`SamPlatform`, `Stm32g0Platform`
and the others stay per family, because their idle hooks differ where
the families' sleep controllers differ, and the STM32G0's takes its
timebase as a template parameter), the clock, the pins, every
peripheral, the crt (vector names are the vendor's), the errata (SAM
erratum 1.8.13's `SysTickInterruptGuard` stays in `samc21/ticker.hpp`),
and the project-wide `Ticker` alias (each family's `ticker.hpp` states
its rate).

## Three architectures, one stratum

The STM32F4 stratum (Cortex-M4, ARMv7-M) includes the first three files
unchanged: SysTick, the NVIC's enable, pend and priority registers and
PRIMASK have the same programmer's model on ARMv7-M, and the files read
nothing else, which is why the stratum is named for the core family and
not for an architecture. What ARMv7-M adds (BASEPRI, the sixteen
priority levels, the FPU, the configurable faults) stays in `stm32f4/`,
unused by the cooperative kernel, until another ARMv7-M family proves
what is shared, by the rule this stratum was born under at the second
Cortex-M0+ family: an `armv7m/` would sit beside this one, holding what
ARMv7-M adds, as an `armv8m/` would for the M33s. The fourth file is the
one exception, and it is here on the same rule read the other way: the
DWT's cycle counter is what the M3, the M4, the M7 and the M33 share,
one unit and one programmer's model in ARMv7-M and ARMv8-M main alike,
so it is a file of the core family that two of its profiles carry and
the third refuses at compile time, not a thing one architecture adds.
The STM32F4 takes it through `stm32f4/dwt.hpp`, which states the
family's facts (the counter runs on HCLK, a Stop stops it); no M33
family includes it yet.

The RP2350's Arm half (Cortex-M33, ARMv8-M) reaches two of the files
under a condition worth stating, because that chip is the one target
here with two processor architectures over one set of peripherals.
`brio/rp2350/core_m33.hpp`, the Arm branch of that stratum's single
architecture question, is the device header plus `cortexm/nvic.hpp`,
exactly as `samc21/nvic.hpp` and `stm32f4/nvic.hpp` are, and
`brio/rp2350/ticker.hpp` takes `cortexm/ticker.hpp`'s `BasicTicker` on
that half alone, writing the same surface over a different counter on
the other. What that family does not take is `cortexm/delay.hpp`: its
microsecond wait must be one implementation for both instruction sets,
so its ruler is a timer neither core owns. The M33's own extensions (the
MPU, the SAU and TrustZone, MSPLIM, the coprocessor ports, BASEPRI) are
untouched there for the reason BASEPRI is untouched on the M4.

The guards of `nvic.hpp` and `ticker.hpp` accept the M0, M0+, M4 and
M33 core headers; `delay.hpp`'s accepts the first three, which are its
users.

## The include-order contract

A cortexm header does not include a device header, since it cannot know
which, and it refuses to be included before one (`#error`): the CMSIS
core header it is written against is brought in by the device header,
after the device has declared its IRQn enumerators and priority width.
Each family's `nvic.hpp` and `ticker.hpp` is exactly that: the device
include, then the core file, then what is the family's own. Apps and
family drivers keep including the family's headers, never these
directly.

## Editor

`brio/cortexm/.clangd` parses these files against the SAM project's
database (the oldest of the users), so the CMSIS symbols resolve; every
other project that includes them compiles them with its own flags at
build time.
