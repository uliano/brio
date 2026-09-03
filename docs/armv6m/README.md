# Core stratum: ARMv6-M (`armv6m/`)

The one directory under `brio/` that is neither target-independent
nor a target: what ARM designed into every Cortex-M0/M0+ and every
vendor ships unchanged - the NVIC and PRIMASK (`armv6m/nvic.hpp`:
`InterruptGuard`, the global enable/disable/readback verbs, `Nvic`,
`irq_priority_levels`), the SysTick timebase (`armv6m/ticker.hpp`:
`BasicTicker`) and the microsecond busy-wait on SysTick's own counter
(`armv6m/delay.hpp`: `delay_us`, `delay_rate`, `DelayRate` - "at
least", never early, capped below one kernel tick, no division at wait
time). It exists because brio's naming rule says a core stratum is
factored at the SECOND ARM family: `samc/` and `stm32g0/` carried
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
- Not here, by design: the Platform (`SamPlatform`, `Stm32Platform`
  stay per family - their idle hooks differ where the families' sleep
  controllers differ), the clock, the pins, every peripheral, the crt
  (vector NAMES are the vendor's), the errata (SAM erratum 1.8.13's
  `SysTickInterruptGuard` stays in `samc/ticker.hpp`), and the
  project-wide `Ticker` alias (each family's ticker.hpp states its
  rate).

## The include-order contract

An armv6m header does not include a device header - it cannot know
which - and it refuses to be included before one (`#error`): the CMSIS
core header it is written against is brought in BY the device header,
after the device has declared its IRQn enumerators and priority width.
Each family's `nvic.hpp` / `ticker.hpp` is exactly that: the device
include, then the core file, then what is the family's own. Apps and
family drivers keep including the FAMILY's headers, never these
directly.

## Editor

`brio/armv6m/.clangd` parses these files against the SAM project's
database (the older of the two users), so the CMSIS symbols resolve;
the STM32G0 project compiles them with its own flags at build time.
