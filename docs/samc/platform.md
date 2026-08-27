# Platform - what the kernel stands on (SAM C21)

> **PROVISIONAL.** What exists is the waking half of the platform:
> the critical section, the idle hook, the SysTick timebase, the NVIC
> verbs and the crt that carries them - each bench-verified as far as
> a bring-up exercises it. What is missing is the whole stopping and
> failing half: the deeper sleep modes and their SleepSite, the
> reset/watchdog story, the BKPT-with-no-debugger path. The list is
> in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M - the
Cortex-M0+ processor summary ch. 4 with ARM's ARMv6-M ARM behind it,
PM (power manager) ch. 12, RSTC ch. 18 - and errata DS80000740S
(1.8.13, SysTick + standby back-bias, is recorded for the power pass:
the kernel tick IS SysTick). Drivers: `samc/platform_sam.hpp`
(`SamPlatform`, this target's realization of the kernel's `Platform`
concept), `samc/nvic.hpp` (`InterruptGuard`, `Nvic`),
`samc/ticker.hpp` (`BasicTicker`); the crt is `samc/src/glue/
startup_samc21.cpp` + `samc/ld/samc21j18a.ld` in the build project.

These are one story - how a program on this silicon runs one event at
a time, waits, and keeps time - told across one stratum header each
and the two crt files every image links.

## What the silicon does

**Interrupt masking is PRIMASK, and nothing finer.** ARMv6-M has no
BASEPRI: masking is all-or-nothing, one bit, saved and restored like
the AVR's SREG. The NVIC orders PREEMPTION between handlers with
`__NVIC_PRIO_BITS` = 2 high bits of priority (four levels, 0 most
urgent); it never limits the reach of a critical section. Core
exceptions (SysTick among them) have no NVIC enable bit - the
peripheral's own interrupt enable is the gate.

**WFI wakes on a pending interrupt even under PRIMASK.** The handler
does not run until PRIMASK clears, but the core wakes - which is what
lets the idle hook sleep FIRST and unmask AFTER, closing the
lost-wakeup window by construction (an interrupt that lands between
the kernel's queue check and the WFI simply makes the WFI fall
through). This is simpler than the AVR idiom, where `sei` takes
effect one instruction late and `sleep` must follow immediately.

**SysTick counts CPU cycles.** A 24-bit down-counter reloaded from
LOAD, clocked (as configured here) from the processor clock; reading
CTRL clears COUNTFLAG, and the exception entry is what acknowledges
the interrupt - there is no flag for the handler to clear. Because it
is core-private, no application could ever use it for PWM or capture:
claiming it for the kernel tick costs the application nothing - the
same reasoning that gave the AVR tick to the RTC's PIT. The reload is
a function of the CPU clock, and that is the one caveat that outlives
the bring-up: the day this target grows a DynamicClock, the ticker
must either become a ClockUser or move to the RTC (`ticker.hpp`
refuses to compile with a dynamic clock that does not list it - the
caveat is mechanical, not a comment).

**SRAM survival is promised nowhere.** RSTC's reset table (18) has no
SRAM row for any reset cause - exactly the AVR situation - so the
panic breadcrumb's magic word is necessary, not merely prudent.

## Types and verbs

**`SamPlatform`** (platform_sam.hpp) realizes the kernel's `Platform`
concept: `CriticalSection` is nvic.hpp's `InterruptGuard` (save
PRIMASK, `cpsid i`, restore on scope exit; the CMSIS intrinsics carry
the "memory" clobbers the contract requires, verified in the vendored
cmsis_gcc.h); `idle()` is WFI-then-enable as above, with SCR.SLEEPDEEP
untouched - out of reset that means plain sleep, everything clocked,
and whatever deeper mode a future power manager arms will stand,
exactly as the AVR idle() honors a standing SEN; `break_here()` is
`BKPT #0` with the ARMv6-M caveat that the core cannot ask whether a
debugger is attached (DHCSR is debugger-access-only), so with none
the BKPT escalates to the crt's HardFault spin; `now()` reads the
ticker; `ticks_per_second` = 1000; `atomic_width` = 4 (an aligned
word moves in one access - what lets `util/ring.hpp` take its
lock-free path here); `panic_record()` lives in `.noinit`.

**`BasicTicker<tps>`** (ticker.hpp): monostate over SysTick. The rate
must divide 1000 exactly - `millis()` is then EXACT, ticks times a
constant, with none of the AVR's skip-correction - and `Ticker` is
the 1000 Hz alias. `init(clock)` computes the reload from the clock
tag (refusing a reload that does not fit 24 bits), `tick()` is the
handler body, `ticks()/millis()/secs()/now()` read the counters -
through a volatile access, because word atomicity alone is not
visibility (see the header's Concurrency note; the bench finding
below is why that sentence exists). `pause()/resume()` gate only the
interrupt: SysTick has no pause, so the first tick after resume can
be short - the escape hatch, not a metrology verb.

**`Nvic`** (nvic.hpp): enable/disable/pend one line, priority with
refusal of levels this core does not have; `enable_interrupts()` is
this target's `sei()`. Everything in the header is ARMv6-M, not chip:
only the IRQn values come from the device header.

**The crt** (startup_samc21.cpp + samc21j18a.ld): a C++ vector table
of exactly 47 entries in `.vectors` at address 0, every handler a
weak alias of a spin - **the app binds a vector by defining the
strong symbol** (`extern "C" void SysTick_Handler() { ... }`), the
ARM edition of the AVR rule that vector names never appear in
portable code. `Reset_Handler` copies `.data`, zeroes `.bss`, walks
`.init_array` itself (newlib's walker would drag crti.o back past
`-nostartfiles`) and calls `main`. `.noinit` is NOLOAD and skipped by
the zero-fill - the PanicRecord's home. The crt also owns the one
libc symbol brio images really reference: with `-fno-exceptions`,
libstdc++ compiles throw sites into `abort()` calls that `-Og` cannot
prove dead, and newlib's abort would defeat the no-syscalls rule - so
the crt defines `abort()` as a spin (not a BKPT: with no debugger a
BKPT becomes a HardFault and the frame that got there is gone).

## How to use it

The whole platform surface an app touches:

```cpp
using P = brio::SamPlatform;
using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main() {
    SysClock::init();
    brio::Ticker::init(clock);
    brio::enable_interrupts();
    brio::Kernel<P, /* AOs */>::run();
}
```

## Bench findings

- The SysTick timebase holds against the wall clock: two uptime
  readings 2 s apart by the host's own timing measured 1.991 s of
  tick time (OSC48M tolerance and host-side slack absorb the rest);
  reload read back over SWD is exactly 47999.
- The visibility clause in ticker.hpp is not theoretical: with the
  getters reading the counters bare (non-volatile, no barrier), gcc
  at -Os DELETED a `while (ticks() < t)` polling loop outright - the
  function compiled to an immediate return with no load at all. The
  volatile-access read (ring.hpp's own technique) restores the load
  inside the loop; the kernel path had been safe only by the accident
  of crossing InterruptGuard barriers every turn.
- WFI-then-enable ran continuously under both bring-up firmwares (a
  1 kHz tick is a permanent lost-wakeup stress); no stall observed.
  The instrumented version of that claim - loop-turn counts asleep vs
  awake, wake latency in cycles - is the power pass's stopwatch work.
- The vendored CMSIS 5.9.0 really does put "memory" clobbers on
  `cpsid`/`cpsie`/`msr primask` and even `wfi` - the InterruptGuard's
  barrier claim is the intrinsics', verified, not assumed.

## Not covered yet

Driver gaps (not built):
- The stopping half: STANDBY (PM.SLEEPCFG, SCR.SLEEPDEEP), the
  SleepSite adapter for util/power.hpp, performance levels, and
  erratum 1.8.13's SysTick-vs-standby interaction - the power pass.
- The failing half: RSTC reset-cause verbs, the WDT, a HardFault
  handler that writes the panic breadcrumb and resets - the samc
  analog of avrdx/reset.hpp.
- MTB trace, the MPU, DIVAS (the memory-mapped divider; gcc emits
  software division unless taught otherwise).

Implemented but not bench-verified:
- `break_here()` with no debugger attached (documented to escalate to
  the HardFault spin; never provoked deliberately).
- `Nvic::set_pending` as a software interrupt source; priorities
  other than the reset default (every line runs at 0 today, so
  handler-vs-handler preemption is unexercised).
- A `BasicTicker` rate other than 1000 (the 125 Hz instantiation is
  compile-checked by the family TU only).
