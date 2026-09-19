# Platform (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 3.1 (the
processor subsystem and SIO, with 3.1.2's CPUID and 3.1.8's platform
timer), 3.2 (the interrupt lines, the six spare ones among them), 3.8
(Hazard3: the CSRs, the interrupt controller, and 3.8.1.23 on `wfi`),
3.8.4.2 (the interrupt numbering the two architectures share), 7.5 (the
subsystem reset controller), 9.11 (the pad registers and their isolation
latch), 12.15.1 (SYSINFO) and 12.15.3 (TBMAN), 2.1.3 (the atomic register aliases), appendix C
(the hardware revisions) and appendix E (the errata, RP2350-E9 among
them). The drivers: `brio/rp2350/platform.hpp` (the brio Platform
itself), `core.hpp` with `core_m33.hpp` and `core_hazard3.hpp` (the one
file of the stratum that asks which processor is in the socket, and its
two halves), `ticker.hpp` (the kernel timebase, one per core),
`mtime.hpp` (the microsecond ruler), `resets.hpp`, `sysinfo.hpp` and
`device.hpp`. The reference suite: `test_rp2350_platform`, which runs on
BOTH architectures from one source and is judged the same way on each.

## What the silicon does

**ONE PLATFORM TYPE PER CORE, AND ONE PLATFORM TYPE FOR TWO
ARCHITECTURES.** `Rp2350Platform<core, TB>` is keyed by the core because
the kernel's statics are keyed by the platform type, so two cores are two
kernels with two packs, two timebases and two breadcrumbs. It is NOT
keyed by the architecture: everything architecture-specific in it is a
name from `core.hpp`, so the same file, the same kernel and the same
`util/` sit above the Cortex-M33 pair and the Hazard3 pair alike. What
differs is one level down - PRIMASK against mstatus.MIE, WFI against
`wfi`, BKPT against `ebreak`, SysTick against the platform timer's
comparator.

**THE IDLE PATH HAS NO LOST-WAKEUP WINDOW ON EITHER HALF, AND FOR TWO
DIFFERENT REASONS.** The kernel's loop masks interrupts, looks at its
queues, sleeps and unmasks; an interrupt that becomes pending between the
look and the sleep must not put the core to sleep. On the Cortex-M33 that
is ARM's rule, that a pending interrupt wakes WFI even through PRIMASK;
on Hazard3 it is 3.8.1.23's, that `wfi` ignores mstatus.MIE and respects
every other interrupt control. The two are the same promise in two
spellings, which is why the kernel needs no target knowledge - and it is
NOT the behaviour of the QingKe cores brio's other RISC-V strata run on,
where the same sequence deadlocks.

**THE RULER IS THE PLATFORM TIMER, AND IT BELONGS TO EVERYBODY.** 3.1.8's
64-bit counter is named for RISC-V and the datasheet is explicit that it
is usable equally by either architecture; brio runs it on a tick divided
out of clk_ref so that it counts MICROSECONDS and a change of clk_sys
does not move it. It is the ruler every measurement here is taken with,
and on the Hazard3 half it is ALSO the kernel timebase, its comparator
raising the trap the tick handler serves. Its DBGPAUSE bits are cleared
at init: a ruler one core's breakpoint can freeze for the other core is
not a ruler.

**RESET MEANS THREE DIFFERENT THINGS ON THIS CHIP**, and only one of them
is the one a program's memories survive. A PROCESSOR reset (the debug
port's, SYSRESETREQ) resets the cores and leaves the SRAM, the clock tree
and the peripherals exactly as they were - measured: a `.noinit` word and
a panic breadcrumb cross it intact. A rescue over the debug port, a
watchdog event or a power cycle do reset them. And the SUBSYSTEM reset
controller of 7.5 is a third thing again: every peripheral is held in
reset at power-up, so "open the peripheral's clock" is "release its
reset" here, and RESET_DONE is what says the release has taken.

**ERRATUM RP2350-E9 IS LIVE ON STEPPING A2**, and it changes what an
input means. With its input buffer enabled, a pad that nothing drives
leaks enough current through the input stage to sit HIGH against its own
pull-down; a pad driven low and then released stays low. An idle level on
this silicon is HISTORY and not a measurement. A program that must read a
floating pad through its pulls drives it first, or uses the pull-UP and
reads a LOW as its signal.

**THERE ARE TWO REGISTERS CALLED PLATFORM, AND THE ONE IN SYSINFO IS NOT
THE ONE THAT ANSWERS.** SYSINFO's (12.15.1, offset 0x08) is a
PRE-PRODUCTION indicator - its own description says the platform "is
always ASIC, non-SIM" post-production - and on production silicon the
whole register reads zero, ASIC bit included (measured). The register
that answers whether this is an ASIC is TBMAN's, whose ASIC bit the data
sheet gives a reset value of 1 and which reads 1 here; `ChipId::asic()`
reads that one.

**THE PACKAGE IS A BUILD FACT AND A SILICON FACT AT ONCE.** The die is
one and every pin's registers exist on both packages; what the QFN-60 has
not got is the bond wire. A build that states its package refuses an
absent pad at COMPILE time; a build that states none compiles for the
larger package and refuses at RUN time against SYSINFO's PACKAGE_SEL.

## Types and verbs

- `Rp2350Platform<core = 0, TB = CoreTicker<core>>` - the brio Platform:
  `CriticalSection` (the RAII mask, PER CORE in both spellings, saved and
  restored rather than cleared), `idle()` (sleep then unmask, through
  `sleep_hook` when a low-power state that is not a sleep instruction has
  armed one), `sleep_hook` (null until the power chapter), `break_here()`,
  `now()`, `ticks_per_second`, `atomic_width` (4), `core_id()`,
  `on_own_core()` (what `EventQueue::push` checks before it copies),
  `Doorbell` (the bell a send to an AO of this core rings -
  [multicore.md](multicore.md)), and `panic_record()` - one `.noinit`
  record per core, a static of the template.
- `core.hpp`'s names, the same on both halves: `InterruptGuard`,
  `enable_interrupts()`, `disable_interrupts()`, `interrupts_enabled()`,
  `Irq` (the per-line controller over the device header's `IRQn_Type`:
  `enable`, `disable`, `enabled`, `set_pending`, `clear_pending`,
  `pending`), `irq_priority_levels`, `wait_for_interrupt()`,
  `debug_break()`, `core_id()`. Plus `CoreKind` and `core_kind`, a VALUE,
  so a driver with something genuinely different to do branches on it
  with `if constexpr` instead of asking the preprocessor.
- `Ticker` / `CoreTicker<core>` - the kernel timebase at 1000 Hz, one per
  core: `init(clock)`, `tick()` (the handler's body, bound as
  `isr_systick` on both halves), `ticks`, `millis`, `secs`, `now`,
  `advance`, `pause`, `resume`, `rebase`, `ticks_per_second`.
- `Mtime` - the ruler: `start(clock)` (idempotent; false when clk_ref is
  not a whole number of megahertz), `running()`, `now()` (64 bits, read
  against a rollover of its low half), `micros()` (the low half alone,
  one bus read and 71 minutes of range), `set_compare`, `compare`,
  `disarm`.
- `Resets` and `ResetBlock` - the subsystem reset controller:
  `release(blocks)` (with its bounded RESET_DONE wait), `hold`, `cycle`,
  `released`, `held`, `watchdog_resets`.
- `ChipId` - SYSINFO: `read()` (manufacturer, part, the STEPPING the
  errata are keyed by, with `revision_a2` / `a3` / `a4` beside it),
  `package_sel()`, `gitref()`, and `asic()`, which reads the testbench
  manager's register and not SYSINFO's of the same name.
- `device.hpp`'s `hw_set` / `hw_clear` / `hw_xor` / `hw_write_masked` -
  2.1.3's three write-only aliases of every APB and AHB register, one bus
  write and no read-modify-write, which is what lets a bit be flipped
  from a handler or from the other core with no critical section; and
  `Package`, `package_gpio_count`, `gpio_count`, `gpio_count_max`,
  `package_known`.

## How to use it

An app names the platform, the timebase and the clock, and binds its
vectors by the crt's names - the SAME names on both architectures:

```cpp
using P = brio::Rp2350Platform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 150'000'000UL>;
constexpr SysClock clock;

extern "C" void isr_systick() { brio::Ticker::tick(); }

int main() {
    SysClock::init();
    brio::Mtime::start(clock);      // the ruler; the Hazard3 ticker starts it too
    brio::Ticker::init(clock);
    brio::enable_interrupts();
    brio::Tenuto<P, MyAo>::run();
}
```

A measurement, on the ruler rather than on the tick:

```cpp
const uint32_t t0 = brio::Mtime::micros();
do_the_thing();
const uint32_t took = brio::Mtime::micros() - t0;
```

A peripheral out of its reset state before its registers are touched:

```cpp
if (!brio::Resets::release(brio::ResetBlock::pwm)) { /* it never came ready */ }
```

Reading a pad that nothing drives, on a part where E9 is live:

```cpp
(void)Pad::input(brio::PinPull::up);   // and a LOW is the signal
```

## Bench findings

`test_rp2350_platform` on the bench part (a QFN-80 stepping A2, clk_sys
150 MHz from the 12 MHz crystal through the system PLL), green on both
architectures from one source. Where the halves differ, both numbers:

| What | Cortex-M33 | Hazard3 |
|------|-----------|---------|
| the suite | 60 pass, 0 fail | 60 pass, 0 fail |
| 200 kernel ticks on the ruler | 200000 us, 0 ppm | 200000 us, 0 ppm |
| `idle()` calls covering 200 ticks | 200 | 200 |
| awake in that span | 3 us of 199960 | 1 us of 199397 |
| a wakeup raised INSIDE the mask | back in 3 us | back in 1 us |
| a 5 ms masked window's ticks | 1 | 5 |
| `Resets::release()` to RESET_DONE | 1 us | 0 us |

- The chip reports manufacturer 0x493, part 0x0004 and STEPPING 0x2 - A2,
  which is what makes RP2350-E9 live here - with PACKAGE_SEL saying
  QFN-80, the package the image was built for.
- THE KERNEL TIMEBASE IS EXACT ON BOTH HALVES, and for different reasons.
  On the Cortex-M33 SysTick rides clk_sys off the PLL and the ruler rides
  clk_ref off the crystal, so 200 ticks against the ruler measure the
  PLL's ratio and the reload's rounding - and both are exact at 150 MHz
  over 1000 Hz. On Hazard3 the timebase IS a comparator on the very
  counter the ruler reads, so the letter measures the comparator's
  arithmetic instead, and it too is exact.
- A MASKED WINDOW COSTS THE KERNEL TICK ON ONE HALF AND NOT ON THE OTHER,
  which is the one place the two timebases are not interchangeable: five
  milliseconds with interrupts masked advanced the tick by 1 on the
  Cortex-M33 and by 5 on Hazard3. The ruler counts through the window on
  both - it is hardware, not an interrupt - and what differs is what the
  masked tick events leave behind. SysTick leaves ONE pending bit however
  many periods elapsed, so the coalesced ticks are gone; the platform
  timer leaves a COMPARATOR still behind the counter, and the handler,
  which moves it on by one period, is re-entered until it catches up, so
  none are. A program that masks for longer than a tick keeps real time
  on the Hazard3 half and loses it on the Arm half.
- THE LOST-WAKEUP WINDOW IS NOT THERE, measured rather than assumed: a
  line made pending with interrupts masked and `idle()` entered
  immediately after comes back in 3 us on one half and 1 us on the other,
  with the handler served once after the unmask - where a core that
  slept through it would have waited for the next tick, up to a
  millisecond.
- ERRATUM RP2350-E9, SHOWN: a free pad (GP22) driven low and released to
  its own pull-down reads 0; driven HIGH and released to the same
  pull-down it reads 1 and stays there; driven low and released to a
  pull-UP it reads 1. So the pull-up wins over a pad's history and the
  pull-down does not, on this stepping.
- THE INTERRUPT PATH END TO END on a line that reaches no hardware (a
  spare one of 3.2): enabled in the controller and raised in software, it
  is served exactly once by a handler an app bound BY NAME - the same
  name, `isr_spare_0`, through the Cortex-M vector table on one half and
  Hazard3's own dispatch on the other. A DISABLED line still latches its
  pending bit and does not run, and `clear_pending` takes it back down.
- THE ATOMIC ALIASES of 2.1.3, on a register a driver hands out: the SET
  alias ORs the written bits in, CLR clears them, XOR flips them, and a
  masked write changes only its mask - each read back.
- The reset controller: a block held has its RESET bit set and its
  RESET_DONE clear; `release()` reports it ready within a microsecond;
  `cycle()` takes a released block through reset and back; releasing a
  block twice is a confirmation and not an error.
- THE KERNEL'S TIME EVENTS on this timebase: `ticks_to_next()` answers
  the deadline just armed and nothing when the list is empty, a one-shot
  fires exactly once and disarms itself, and a 10 ms periodic fires ten
  times in 105 ms - drift-free, the deadline computed from the last one.
- THE PANIC BREADCRUMB: taken once and gone within a boot; and ACROSS A
  PROCESSOR RESET from the debug port, both the breadcrumb and a
  `.noinit` canary arrive at the next boot with their contents intact -
  the SRAM, the clock tree and the peripherals survive a reset of the
  cores alone.
- SYSINFO's PLATFORM register reads 0x00000000 on this part, ASIC bit
  included, while TBMAN's reads 0x00000001; the two registers and what
  each is for are in the findings above.

## Not covered yet

Driver gaps, each with its reason:

- `delay_us`. The microsecond busy-wait wants a measured fact per family
  and a ruler both halves share; the ruler exists and the fact does not,
  and nothing yet needs the verb - every wait in the suite is a spin on
  `Mtime::micros()`.
- The reboot and the causes of a boot: they belong to another chapter -
  [reset.md](reset.md) for the causes and the two reset verbs,
  [watchdog.md](watchdog.md) for the countdown and the scratch registers
  - which is why the breadcrumb letter HERE is driven by a reset from the
  debug port and is outside `z`. The `.noinit` survival of a watchdog
  reset and of a rescue is that chapter's to measure.
- Interrupt priorities. Both halves have sixteen levels and neither uses
  them: the kernel's promise is that no interrupt nests over another, and
  it is kept structurally here (every line at the reset priority on the
  Cortex-M33, mstatus.MIE never set inside a handler on Hazard3). The two
  conventions run opposite ways - numerically higher is more urgent on
  Hazard3 - which is why `Irq` carries no priority verb at all.
- `idle_until()`. The timebase runs while the core sleeps on either
  half, so the plain idle path is the whole story until a power chapter
  offers a state that stops it.
- The Cortex-M33's own extensions - the MPU, the SAU and TrustZone,
  MSPLIM, BASEPRI, the coprocessor ports: everything here runs Secure, as
  the bootrom hands over, and nothing has needed one.
- Which architecture is running, asked of the CHIP rather than of the
  build. The suite reports `core_kind`, which is the image's own claim,
  and the chip's ARCHSEL_STATUS lives in the OTP block's register window,
  which no driver of this stratum reaches yet; read over the debug port
  it is 0 after an Arm image and 3 after a RISC-V one.

Implemented but not bench-verified, each with what would measure it:

- CORE 1, on either architecture. The platform is per core and the
  tickers are per core by construction, and the suite proves that
  `on_own_core()` agrees with CPUID on core 0; nothing has launched a
  second core here. The launch, the bell and the bridge are
  [multicore.md](multicore.md)'s, whose own list names the letter that
  will measure each.
- The QFN-60's run-time refusals: the stratum compiles for that package
  and the suite checks the die's own PACKAGE_SEL against the build's, but
  no QFN-60 part is on the bench.
- `Ticker::advance`, `pause` and `resume`: born with the first sleep site
  that stops the timebase, which is the power chapter's.
- `Mtime` under a clk_ref that is not 12 MHz: the arithmetic refuses one
  that is not a whole number of megahertz, and only the 12 MHz crystal
  has been on the wire.
- `break_here()` without a probe attached. Under a probe with halting
  debug enabled it halts the core; the flash verb of this target takes
  halting debug back down afterwards so that it faults instead, and the
  fault path itself is the reset chapter's to catch.
