# Core stratum: Cortex-M (`cortexm/`)

One of the directories under `brio/` that is neither target-independent
nor a target: what ARM designed into every Cortex-M - the M0/M0+ of
three families, the M4 of a fourth and the M33 half of a fifth, one
programmer's model over ARMv6-M, ARMv7-M and ARMv8-M
- and every vendor ships unchanged - the NVIC and PRIMASK (`cortexm/nvic.hpp`:
`InterruptGuard`, the global enable/disable/readback verbs, `Nvic`,
`irq_priority_levels`), the SysTick timebase (`cortexm/ticker.hpp`:
`BasicTicker`, and `SysTickCounter` - SysTick as a bare cycle counter
with no interrupt, for a program whose kernel timebase is elsewhere,
such as the STM32G0's tickless LPTIM one; both are `ClockUser`s whose
`rebase(hz)` reprograms the reload for a dynamic clock - a new LOAD and
a restarted period, CTRL untouched by either, so the ticker loses the
phase of the tick in progress, under a tick, late and never early;
both measured at every rung of the STM32G0's ladder, the ticker holding
1000 Hz against the crystal at 64, 16 and 2 MHz)
and the microsecond busy-wait on SysTick's own counter
(`cortexm/delay.hpp`: `delay_us`, `delay_rate`, `DelayRate` - "at
least", never early, capped below one SysTick period of one
millisecond, no division at wait time: folded for a static clock,
and for a dynamic one selected by its rate index out of `delay_rates`,
a per-rate table built at compile time over the clock's discrete-rate
surface; it needs the counter running, never the interrupt, so it
serves both writers alike). It exists because brio's naming rule says a core stratum is
factored at the SECOND ARM family: `samc21/` and `stm32g0/` carried
these files as twins line for line - the first two until the STM32G0's
bring-up, the third until the STM32G0's fillers were done - and every
extraction was gated by the images: every SAM C21 and STM32G0 release
image byte-identical before and after.

## What lives here, and what does not

- Here: code that reads only CMSIS-Core symbols (`__get_PRIMASK`,
  `NVIC_*`, `SysTick`, `IRQn_Type`, `__NVIC_PRIO_BITS`) and nothing of
  any vendor header. Each family's `delay.hpp` keeps what was MEASURED
  on its silicon (the division's cost, the counter's resolution, which
  sleep stops it) beside the include.
- Not here, by design: the Platform (`SamPlatform`, `Stm32g0Platform`
  stay per family - their idle hooks differ where the families' sleep
  controllers differ, and the STM32G0's takes its timebase as a
  template parameter), the clock, the pins, every peripheral, the crt
  (vector NAMES are the vendor's), the errata (SAM erratum 1.8.13's
  `SysTickInterruptGuard` stays in `samc21/ticker.hpp`), and the
  project-wide `Ticker` alias (each family's ticker.hpp states its
  rate).

## Three architectures, one stratum

The STM32F4 stratum (Cortex-M4, ARMv7-M) includes these three files
unchanged: SysTick, the NVIC's enable/pend/priority registers and
PRIMASK have the same programmer's model on ARMv7-M, and the files
read nothing else - which is why the stratum is named for the CORE
family and not for an architecture. What ARMv7-M adds - BASEPRI, the
sixteen priority levels, the FPU, the configurable faults - stays in
`stm32f4/`, unused by the cooperative kernel, until another ARMv7-M
family proves what is shared (the rule by which this stratum was born
at the second Cortex-M0+ family): an `armv7m/` would sit BESIDE this
one, holding what v7-M adds, as an `armv8m/` would for the M33s.

The RP2350's Arm half (Cortex-M33, ARMv8-M) reaches the same two of
them under a condition worth stating, because that chip is the one
target here with TWO processor architectures over one set of
peripherals: `brio/rp2350/core_m33.hpp` - the Arm branch of that
stratum's single architecture question - is the device header plus
`cortexm/nvic.hpp`, exactly as `samc21/nvic.hpp` and `stm32f4/nvic.hpp`
are, and `brio/rp2350/ticker.hpp` takes `cortexm/ticker.hpp`'s
`BasicTicker` on that half alone, writing the same surface over a
different counter on the other. What that family does NOT take is
`cortexm/delay.hpp`: its microsecond wait must be one implementation
for both instruction sets, so its ruler is a timer neither core owns.
The M33's own extensions - the MPU, the SAU and TrustZone, MSPLIM, the
coprocessor ports, BASEPRI - are untouched there for the reason
BASEPRI is untouched on the M4.

The guards of `nvic.hpp` and `ticker.hpp` accept the M0, M0+, M4 and
M33 core headers; `delay.hpp`'s accepts the first three, which are its
users.

## The include-order contract

An cortexm header does not include a device header - it cannot know
which - and it refuses to be included before one (`#error`): the CMSIS
core header it is written against is brought in BY the device header,
after the device has declared its IRQn enumerators and priority width.
Each family's `nvic.hpp` / `ticker.hpp` is exactly that: the device
include, then the core file, then what is the family's own. Apps and
family drivers keep including the FAMILY's headers, never these
directly.

## Editor

`brio/cortexm/.clangd` parses these files against the SAM project's
database (the oldest of the users), so the CMSIS symbols resolve; every
other project that includes them compiles them with its own flags at
build time.
