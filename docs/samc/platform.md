# Platform - what the kernel stands on (SAM C21)

> **PROVISIONAL.** The waking half (critical section, idle hook,
> SysTick timebase, NVIC, crt) and the STOPPING half (PM's three sleep
> modes, the `SleepSite` and the power model above it) are both here;
> the FAILING half has its own document, [reset.md](reset.md). What
> remains is in "Not covered yet" - above all a measurement this bench
> cannot make (sleep current) and the RTC-backed timebase that would
> lift the standby restriction stated below.

Documents of record: SAM C20/C21 data sheet DS60001479M - the
Cortex-M0+ processor summary ch. 4 with ARM's ARMv6-M ARM behind it,
PM (power manager) ch. 19 - and errata DS80000740S (1.8.13, 1.8.14,
1.8.7 and 1.8.5 all touch this chapter; see "What the silicon does").
Drivers: `samc/platform_sam.hpp` (`SamPlatform`, this target's
realization of the kernel's `Platform` concept), `samc/nvic.hpp`
(`InterruptGuard`, `Nvic`), `samc/ticker.hpp` (`BasicTicker`,
`SysTickInterruptGuard`), `samc/sleep.hpp` (`Pm`, `SamSleepSite`); the
target-independent power model above the last of these is
[design/power.md](../design/power.md). The crt is `samc/src/glue/
startup_samc21.cpp` + `samc/ld/samc21j18a.ld` in the build project.

These are one story - how a program on this silicon runs one event at
a time, waits, keeps time and stops - told across one stratum header
each and the two crt files every image links.

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

**SRAM survival is promised nowhere.** RSTC's reset table (18-1) has
no SRAM row for any reset cause - exactly the AVR situation - so the
panic breadcrumb's magic word is necessary, not merely prudent. That
it survives in practice is measured in [reset.md](reset.md), which
also owns the reset causes themselves.

**The PM has three sleep modes and no interrupt.** Chapter 19 is two
registers. SLEEPCFG.SLEEPMODE selects what the next WFI takes -
**IDLE0** (0x0, the reset value: the CPU stops, MCLK and generator 0
and their source stay up, every peripheral that asks keeps its clock,
the CAN included), **IDLE2** (0x2, the same minus the CAN's clock, so
the CAN can no longer wake the device) and **STANDBY** (0x4, the CPU
and the peripherals stop; what survives is what asks to). 0x1, 0x3 and
0x5..0x7 are Reserved. STDBYCFG carries the two knobs standby has:
VREGSMOD (which regulator supplies VDDCORE) and BBIASHS (back-bias the
RAM, and note the DMAC cannot reach a back-biased RAM). The block has
no interrupt, no event, no DMA connection and cannot be reset; its APB
clock can be turned off but **only a system reset turns it back on**
(19.5.2).

**The mode register lands LATE.** 19.6.3.3 warns of "a small latency
between the store instruction and actual writing of the SLEEPCFG
register due to bridges" and requires software to read the register
back before issuing WFI. The latency is measured at about five
microseconds on a 48 MHz CPU - not a formality, and `Pm` spends the
readback on every arming.

**One RUNSTDBY bit is the whole request.** 19.6.3.3.2 calls it
SleepWalking: a peripheral running in standby requests its clock, and
that request wakes the GCLK generator and the clock source behind it.
Measured on this silicon, the peripheral's own `CTRLA.RUNSTDBY` is the
ONLY bit an application has to set - the generator's `GENCTRL.RUNSTDBY`
and the source's own are both unnecessary, and OSCULP32K, whose
register carries no such bit at all, serves a generator through standby
just the same.

**The kernel tick STOPS in standby.** SysTick is clocked from the CPU
clock and the CPU clock stops, so kernel time stands still for exactly
as long as the sleep lasts and every armed time event matures late by
that amount. This does not travel from the AVR, where the tick is the
RTC's PIT on a 32 kHz oscillator and runs through every sleep mode.
The consequences are a rule and a restriction, both below.

**Erratum 1.8.13 is live and its preconditions are the default.** With
the standby back-bias option set - and STDBYCFG comes up at 0x0400,
i.e. BBIASHS SET - a SysTick interrupt coinciding exactly with a
standby entry can raise a hard fault. The workaround (disable that
interrupt around standby entry) costs nothing here precisely because
the tick is frozen across a standby either way. **1.8.14** (standby
with VREGSMOD in performance mode switches to the low-power regulator
and keeps requesting GCLK0; the workaround is SUPC.VREG.RUNSTDBY)
and **1.8.7** (a DMA write performed while sleepwalking may not land
on RTC.COUNT, the TC/TCC control and count registers, or ADC/SDADC
SWTRIG) are live and stated where they belong; **1.8.5** (higher
standby current, no workaround) is a number, not a behaviour. NOT this
silicon, revision B only: 1.8.1, 1.8.11, 1.8.6.

**A debugger changes what standby is.** 19.5.6: with the CPU halted in
debug mode the PM keeps operating, and a standby requested while a
debugger is attached does not turn the power domains off. Every number
below was taken with DHCSR.C_DEBUGEN cleared, which `tools/bench.py`
does at the end of every SAM flash.

## Types and verbs

**`SamPlatform`** (platform_sam.hpp) realizes the kernel's `Platform`
concept: `CriticalSection` is nvic.hpp's `InterruptGuard` (save
PRIMASK, `cpsid i`, restore on scope exit; the CMSIS intrinsics carry
the "memory" clobbers the contract requires, verified in the vendored
cmsis_gcc.h); `idle()` is DSB-WFI-then-enable as above, and **it takes
whatever PM.SLEEPCFG holds** - this family selects the depth in that
register and not in SCR.SLEEPDEEP, which is never written, so a mode
armed above simply stands and the power model needs no new kernel hook.
The one thing the hook does spend is a read of SLEEPCFG per idle, and
it buys erratum 1.8.13's workaround: when the armed mode is STANDBY the
SysTick interrupt is held off across the WFI. `break_here()` is
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

**`SysTickInterruptGuard`** (ticker.hpp, next to the register it
guards): an RAII scope that holds the SysTick interrupt off and puts
it back exactly as it found it. It exists for erratum 1.8.13 and has
two users, `SamPlatform::idle()` and `Pm::sleep()`.

**`Pm`** (sleep.hpp): the whole of chapter 19. `sleep_mode()` and
`set_sleep_mode(m)` (which spends the readback rule and refuses a
Reserved code, with a `set_sleep_mode<m>()` twin that refuses at
compile time); `configure_standby(cfg)` / `regulator_mode()` /
`back_bias()` over STDBYCFG, again with a compile-time twin;
`bus_clock()`; and the two verbs that stop the CPU - `sleep()` (WFI at
the armed mode, taking the erratum guard when that mode is STANDBY)
and `enter(m)` (arm, then sleep). `VregStandbyMode` names the three
implemented VREGSMOD codes and its `performance` enumerator carries
erratum 1.8.14's obligation; this header does not write SUPC's
register for the caller.

**`SamSleepSite`** (sleep.hpp): the `util/power.hpp` adapter. `none`
maps to IDLE0, `light` to IDLE2, `standby` to STANDBY and `deep` to
STANDBY as well - **this family has nothing deeper, so the model's
map-an-absent-rung-shallower rule is not the identity here as it was
on AVR DA/DB, and `armed()` reports `standby` for a `deep` request:
what the target really took**. `light` is IDLE2 rather than IDLE0
because there is no SEN bit on this silicon: IDLE0 is both a sleep
mode and the reset value, so mapping `light` there would make "armed at
light" indistinguishable from "nothing armed" without a variable
shadowing the register. The price is that the CAN cannot wake the
device from `light`; brio has no CAN driver, and a program that grows
one arms IDLE0 through `Pm::enter()` directly.

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

And the whole of stopping, which is one AO more in the pack and one
rule to respect:

```cpp
using Pm = brio::PowerManager<P, brio::SamSleepSite, brio::PowerConfig{},
                              /* voters */>;
using K = brio::Kernel<P, /* AOs */, Pm>;

// Somewhere that decides the program has nothing to do. THE RULE OF
// THIS TARGET: kernel time stops in standby, so ask for it only when
// nothing is waiting on the tick.
const auto next = brio::TimeEvents<P>::ticks_to_next();
brio::post<Pm>(brio::SleepRequested{
    next ? brio::SleepDepth::light : brio::SleepDepth::standby,
    brio::reply_to<MyAo, brio::SleepVote>()});
```

The manager arms the site; the kernel loop finds every queue empty and
calls `SamPlatform::idle()`; THAT is the sleep. Before asking for
standby the application owes itself the rest of what chapter 19
requires: the wake source must be configured, enabled and REACHABLE IN
THAT MODE (an asynchronous interrupt, or a peripheral with its own
`RUNSTDBY` set - the RTC and the watchdog on OSCULP32K are the two
this stratum has), and any peripheral that must keep running through
the sleep needs that same bit of its own.

## Bench findings

The suite is `test_samc_sleep`, wireless. Its standby ruler is the RTC
on OSCULP32K (whose clock is not a GCLK, so it is not itself
sleepwalking); its fine ruler is a TC pair on the board's 24 MHz
crystal with RUNSTDBY set, which IS a sleepwalking task and therefore
holds the main regulator up - every wake number below is the bill with
the supply already at working point, and is stated as such.

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
  Instrumented since: over a quarter second the idle loop turns
  109000 times spinning and 250 times sleeping in IDLE0, i.e. exactly
  once per SysTick tick.
- The vendored CMSIS 5.9.0 really does put "memory" clobbers on
  `cpsid`/`cpsie`/`msr primask` and even `wfi` - the InterruptGuard's
  barrier claim is the intrinsics', verified, not assumed.

Stopping:

- **The SLEEPCFG bridge latency is about 5 us** on a 48 MHz CPU (mean
  of sixteen alternating armings, the ruler's own cost subtracted).
  19.6.3.3's "small latency" is real and the readback rule is not a
  formality: a WFI issued straight after the store would sleep in the
  mode that was there before.
- **STDBYCFG is three bits wide.** Storing 0xFFFF reads back 0x04C0,
  the device header's own register mask, and its reset value is
  0x0400 - back-bias ON, regulator AUTO.
- **Waking from IDLE0 costs nothing measurable** over a polled wait on
  the same RTC compare: under a microsecond either way across runs,
  i.e. a WFI and an exception entry. **IDLE2 costs 24 to 30 us more
  than IDLE0** on a board with no CAN traffic at all - chapter 19
  presents IDLE2 as IDLE0 with one more clock gated and says nothing
  about paying for it at the wake. (THE ABSOLUTES HERE WERE CORRECTED
  on 2026-08-29: the campaign's original 3.5..4.4 us was measured
  through tc.hpp's one-behind READSYNC defect, whose differences
  telescoped onto the inter-round ARMING time instead of the wait
  window - tc.md carries the mechanism and the fix. Every structural
  conclusion below survived the correction; the absolute bills did
  not.)
- **Waking from STANDBY costs about 106 us** more than waking from
  IDLE0 (mean of 64 single wakes on the crystal ruler; originally
  reported as 16.6..17.8 us through the same defect). That is the
  whole bill: **nothing in STDBYCFG or SUPC.VREG moves it**. Six
  combinations of VREGSMOD (AUTO / PERFORMANCE / LP) x SUPC.VREG.
  RUNSTDBY x BBIASHS spread under 2 us, inside twice the scatter of
  the same measurement repeated first and last. This family therefore
  has NO separate regulator bill, unlike AVR DA/DB where the regulator
  was a distinct ~290 us item on top of the oscillator's - and even
  the corrected 106 us is far below the AVR's 290 us + 1.77 ms
  crystal restart.
- Erratum 1.8.14's predicted fingerprint - PERFORMANCE alone keeping
  GCLK0 requested, hence a cheap wake - **is not visible in the wake
  time**. Reported, not claimed: the erratum is about which regulator
  is used and about a clock left requested, and only the second of
  those could show up here.
- **The kernel tick freezes, exactly.** A standby of 499 ms of wall
  clock advanced `Ticker::ticks()` by 0 ms; the same sleep in IDLE0
  advanced it by 496. A time event 50 ms away, slept over for 249 ms,
  matured 199 ms late and the kernel never knew.
- **The peripheral's own RUNSTDBY is the whole request.** A TC pair
  counting through a 31 ms standby: with `CTRLA.RUNSTDBY` clear it
  counts 13 ticks of 1024 whatever the generator and the source say;
  with it set it counts all 1024 - with the generator's own RUNSTDBY
  CLEAR, and with the source's (OSC32K's) CLEAR, and on OSCULP32K,
  whose register has no such bit at all.
- **XOSC keeps running through a standby whatever RUNSTDBY says.**
  Asked of the crystal's own counter (a status flag cannot answer:
  XOSCRDY reads set at every wake, and stays set across a deliberate
  stop) in three arrangements - a TC still running on it, its
  generator disabled for the whole sleep, and its own RUNSTDBY set -
  it is already ticking at every wake, where table 19-2 gives a source
  with ONDEMAND = 0 and RUNSTDBY = 0 as stopped. The same shape errata
  1.3.1 records for the FDPLL and marks revision B. The practical
  consequence is large: **a standby on this board costs no crystal
  restart**, where the same crystal on the first target cost 1.77 ms
  out of every deep sleep.
- **The watchdog runs through standby and its early warning wakes the
  device** - 123 ms measured for a 128-cycle offset on its nominal
  1.024 kHz. It is also this suite's anti-wedge backstop: a wake that
  never arrives costs a reboot and a banner instead of a mute board.
- **The power model needed no change at all.** `util/power.hpp`
  compiled and ran UNTOUCHED on this second architecture: the vote
  round, the unanimity rule, the `PowerLock` ceilings, the deadline
  guard and the first-event-after-wake contract all pass on the real
  kernel with two voters. A two-voter round costs 153 us post-to-armed.
- The deviation the model asks of a platform is one line here rather
  than the AVR's: `SamPlatform::idle()` already took whatever was
  armed. Making it read SLEEPCFG (for the erratum guard) and adding a
  DSB cost **+52 bytes on blink and +44 on console**; the other twelve
  SAM images that do not use `idle()` are byte-identical.

## Not covered yet

Driver gaps (not built):
- **A timebase that survives standby.** The v1 policy is an honest
  restriction and not a correction: standby is legitimate when the
  kernel has no armed time event, and `TimeEvents<P>::ticks_to_next()`
  is the question that answers it. Lifting the restriction means
  moving the kernel tick to the RTC, or resynchronizing SysTick's
  counters from the RTC after every wake - designed work, not a patch,
  and deliberately not hidden inside `sleep.hpp`.
- **EIC as a standby wake source** is compile-only: erratum 1.11.6
  (with asynchronous edge detection in standby only the FIRST edge
  wakes) is stated in `eic.hpp`, and exercising either the wake or the
  erratum needs an external edge this wireless bench cannot make.
- **PAC.** Chapter 19's registers are optionally PAC write-protected
  (19.5.7) and brio has no PAC driver, so the protection is left as
  reset leaves it - off.
- **SLEEPONEXIT**, the ARM feature 19.6.3.3.1 mentions (re-enter the
  armed mode when the CPU leaves the lowest-priority ISR). It would be
  a different shape of idle path and no AO wants it yet.
- MTB trace and the MPU.
- DIVAS (the memory-mapped divider; gcc emits software division
  unless taught otherwise).

Implemented but not bench-verified:
- `break_here()` with no debugger attached. It is now REACHED
  deliberately - `test_samc_platform` letter i runs a panic() through
  it - but what it does depends on DHCSR.C_DEBUGEN: with a probe
  attached the core HALTS on the BKPT instead of faulting, and that
  bit survives every software reset (table 18-1 resets the debug logic
  only on a power-on or external reset). See [reset.md](reset.md),
  "the BKPT hazard".
- `Nvic::set_pending` as a software interrupt source; priorities
  other than the reset default (every line runs at 0 today, so
  handler-vs-handler preemption is unexercised).
- A `BasicTicker` rate other than 1000 (the 125 Hz instantiation is
  compile-checked by the family TU only).
- **Sleep CURRENT.** Everything above is time, because this bench has
  no supply meter on the board. What a standby actually saves - and
  what BBIASHS, VREGSMOD and erratum 1.8.5 do to it - is a manual
  measurement with an ammeter in the supply, and nothing here claims
  it.
- **`Pm::bus_clock(false)`**, deliberately not exercised: 19.5.2 makes
  it one-way (only a system reset turns CLK_PM_APB back on), so a
  suite that tried it would spend a reboot to learn what the chapter
  states.
- **Whether the FDPLL stops in standby.** Its CLKRDY reads set at
  every wake with RUNSTDBY clear and nothing requesting it, but this
  suite counts no DPLL-derived clock and the crystal above showed
  exactly what such a flag is worth after a wake. Consistent with
  erratum 1.3.1 (which marks that behaviour revision B); not evidence
  for it.
- **Sleeping under a debugger.** 19.5.6 says the power domains are not
  turned off then; everything here was measured with C_DEBUGEN
  cleared, and the debug-attached behaviour is trusted to the chapter.
