# Platform (RP2350)

This document is for a programmer putting a brio program on an RP2350,
on either of its two processor architectures. It describes what the
kernel stands on here: the platform, the kernel timebase, the
microsecond ruler, the subsystem reset controller and the registers the
chip identifies itself by. A brio kernel asks the chip for a handful of
things, stated by the kernel's `Platform` concept (a concept is a
compiler-checked set of requirements on a type;
[design/kernel.md, "Platform: what the machine provides"](../design/kernel.md#11-platform-what-the-machine-provides-kernelplatformhpp)):
a way to mask interrupts for a critical section, a way to sleep until
the next interrupt without losing one that arrives at the wrong moment,
a source of time, a breakpoint for a panic, and a record a panic leaves
for the next boot. `Rp2350Platform<core, TB>` is this chip's answer, and
`Rp2350RubatoPlatform<core, TB>` the preemptive kernel's, the same type
with the three members of a preemptive platform ("Types and verbs"). It
is one type per core, because the two cores of a pair run two kernels
([multicore.md](multicore.md)), and one type for both architectures: the
same source compiles for the Cortex-M33 pair and for the Hazard3 pair,
and only `core.hpp` asks the compiler which one it is built for
(`__riscv`); the three headers that must include a different file or run
different instructions per architecture - `ticker.hpp`, `multicore.hpp`
and `reset.hpp` - ask the macro `core.hpp` defines from that answer,
`BRIO_RP2350_CORE_M33`. `TB` is the core's timebase, a 1000 Hz tick:
SysTick on the Cortex-M33, a comparator of the RISC-V platform timer on
Hazard3, under one name, `Ticker`. The ruler is that platform timer
itself, `Mtime`, a 64-bit counter of microseconds that both
architectures read. The platform's suite measures with it and with the
two system timers of [timer.md](timer.md): `Timer<0>` in microseconds,
the idle letter's rescue clock among its uses, and `Timer<1>` on clk_sys
counting cycles, the unit of the rows of letters `w` and `o`.

The smallest use, the start of a program, is an excerpt of the block
under "How to use it" below:

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

Line by line: `P` names core 0's platform with its default timebase,
what a single-core program names; `SysClock` names the clock tree, here
the system PLL at 150 MHz from the board's 12 MHz crystal
([clock.md](clock.md)), and its `init()` is the clock task, the call
that brings the tree up; `clock` is an empty tag object of that type,
the argument the ruler and the ticker take to learn the rates;
`isr_systick` is the timebase's interrupt handler, written `extern "C"`
so that it carries the plain C name the crt - the startup code that
holds the vector table - binds, and that name is the same on both
architectures (SysTick's vector on the Cortex-M33, the machine timer
trap the RISC-V crt routes there on Hazard3); `SysClock::init()` brings
the tree up; `Mtime::start(clock)` starts the ruler, and may be called
again: a second start keeps the count, restarts the tick generator and
writes the counter's control register back to its enable bit alone,
clearing FULLSPEED and both debug-pause bits if anything had set them;
`Ticker::init(clock)` starts the kernel's tick; `enable_interrupts()`
unmasks interrupts on this core; and `Tenuto<P, MyAo>::run()` starts the
kernel over the program's active objects (`MyAo` stands for them: an
active object is a component of a brio program with its own event queue,
served one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)) and never
returns.

Four things decide whether that works. The clock task must have run
first: the SysTick reload is computed from the rate the clock type
claims and the ruler's divisor from the reference clock's, neither read
from the hardware. The program must bind `isr_systick` itself, as above;
the kernel binds no vector, and a program that forgets it compiles,
links and stops in the crt's default spin at the first tick. Interrupts
must be enabled before `run()`, the precondition `Tenuto::run()` states
(`brio/kernel/tenuto.hpp`): the kernel never unmasks them itself outside
its idle, and the critical section around that idle restores the mask
the loop entered with, so a loop entered masked would take the tick and
every driver's interrupt only in the instant each idle returns, and not
at all while events keep coming. And the ruler refuses a reference clock
that is not a whole number of megahertz: `Mtime::start()` answers false
then, and so does the Hazard3 ticker's `init()`, which stands on it (the
Cortex-M33 ticker's `init()` answers false instead when the SysTick
reload does not fit); the example discards both answers, and a Hazard3
program on such a clock has a kernel with no time. A critical section
here masks this core alone, so anything two kernels share goes through
the bridge of [multicore.md](multicore.md) - an inbox each way and the
SIO doorbell
that rings the other core - and not through `CriticalSection`.

The rest of this page, in order: the documents of record; what the
silicon does (one platform type per core and for both architectures,
the idle path, the ruler, the three meanings of reset, erratum
RP2350-E9, the two registers called PLATFORM, the package); the types
and verbs; how to use it; the bench findings, both halves side by side;
"A handler in SRAM", an opt-in placement of a handler or a driver's hot
path, with its measured cost; and what is not covered.

## Documents of record

The RP2350 datasheet (build d126e9e), 3.1 (the processor subsystem and
SIO, with 3.1.2's CPUID and 3.1.8's platform timer), 3.2 (the interrupt
lines, the six spare ones among them), 3.8 (Hazard3: the CSRs, the
interrupt controller, and 3.8.1.23 on `wfi`), 3.8.4.2 (the interrupt
numbering the two architectures share), 7.5 (the subsystem reset
controller), 9.11 (the pad registers and their isolation latch), 12.15.1
(SYSINFO) and 12.15.3 (TBMAN), 2.1.3 (the atomic register aliases),
appendix C (the hardware revisions) and appendix E (the errata,
RP2350-E9 among them). For the preemptive platform: 3.7.2 (the
Cortex-M33's configuration: the Security Extension, the FPU, four
priority bits), the M33 register list's FPCCR table (3.7.5), 3.8.4
(Hazard3's trap entry and `mret`), 3.8.4.2.1 (the software interrupt and
the SIO's RISCV_SOFTIRQ register, 3.1.11), 3.8.6.1 (Xh3irq's preemption,
which is not used), 3.8.7 (Hazard3's instruction cycle counts) and 3.8.8
(its configuration); and ARM's ARMv8-M Architecture Reference Manual
(DDI 0553) for EXC_RETURN. The drivers: `brio/rp2350/platform.hpp` (the
brio Platform itself, and the Rubato platform beside it), `core.hpp`
with `core_m33.hpp` and `core_hazard3.hpp` (the one file of the stratum
that asks which processor is in the socket, and its two halves),
`schedule.hpp` with `schedule_hazard3.hpp` (the preemptive kernel's
request, the core stratum's on one half and this stratum's on the
other), `ticker.hpp` (the kernel timebase, one per core), `mtime.hpp`
(the microsecond ruler), `resets.hpp`, `sysinfo.hpp` and `device.hpp`;
4.4.1 (the XIP cache) and `brio/util/placement.hpp` for the placement.
The reference suites: `test_rp2350_platform`, which runs on both
architectures from one source and is judged the same way on each (a
letter is one command key of the suite's console and the group of
verdicts it runs; `z` is the key that runs every letter but two, which
run by name: `r`, the breadcrumb across a processor reset, and `x`,
`break_here()`, which stops the console), and `test_rp2350_rubato` for
the preemptive kernel, the same on both ("Bench findings").

## What the silicon does

**One platform type per core, and one platform type for two
architectures.** `Rp2350Platform<core, TB>` is keyed by the core because
the kernel's statics are keyed by the platform type, so two cores are two
kernels with two packs (a pack is the program's list of active objects,
the arguments of its `Tenuto`), two timebases and two breadcrumbs. It is
not keyed by the architecture: everything architecture-specific in it is a
name from `core.hpp`, so the same file, the same kernel and the same
`util/` sit above the Cortex-M33 pair and the Hazard3 pair alike. What
differs is one level down - PRIMASK against mstatus.MIE, WFI against
`wfi`, BKPT against `ebreak`, SysTick against the platform timer's
comparator.

**The idle path has no lost-wakeup window on either half, and for two
different reasons.** The kernel's loop masks interrupts, looks at its
queues, sleeps and unmasks; an interrupt that becomes pending between the
look and the sleep must not put the core to sleep. On the Cortex-M33 that
is ARM's rule, that a pending interrupt wakes WFI even through PRIMASK;
on Hazard3 it is 3.8.1.23's, that `wfi` ignores mstatus.MIE and respects
every other interrupt control. The two are the same promise in two
spellings, which is why the kernel needs no target knowledge - and it is
not the behaviour of the QingKe cores brio's other RISC-V strata run on,
where the same sequence deadlocks. Neither sleep instruction has an
event latch to leave behind (Hazard3's latch belongs to `h3.block`,
3.8.6.3.1; the M33's event register to WFE), so a wake is one return of
`idle()` and the kernel's loop turns once per interrupt - both halves of
the contract measured below.

**The ruler is the platform timer, and it belongs to everybody.**
3.1.8's 64-bit counter is named for RISC-V and the datasheet is explicit
that it is usable equally by either architecture; brio runs it on a tick
divided out of clk_ref so that it counts microseconds and a change of
clk_sys does not move it. It is this page's ruler beside the two system
timers named at the top, and on the Hazard3 half it is also the kernel
timebase, its comparator raising the trap the tick handler serves. Its
DBGPAUSE bits are cleared at init: a ruler one core's breakpoint can
freeze for the other core is not a ruler.

**Reset means three different things on this chip**, and a program's
memories survive more than one of them. A processor reset (the debug
port's, SYSRESETREQ) resets the cores and leaves the SRAM, the clock tree
and the peripherals exactly as they were - measured: a `.noinit` word and
a panic breadcrumb cross it intact. A watchdog event, a rescue over the
debug port or a power cycle reset the system or the whole chip; of
those, the watchdog's reboot and its time-out keep the `.noinit` SRAM
and the watchdog's scratch registers (measured in [reset.md](reset.md),
"Bench findings"), a power cycle keeps nothing, and what a rescue keeps
is not measured ("Not covered yet" below). And the subsystem reset
controller of 7.5 is a third thing again: every peripheral is held in
reset at power-up, so "open the peripheral's clock" is "release its
reset" here, and RESET_DONE is what says the release has taken.

**Erratum RP2350-E9 is live on stepping A2**, and it changes what an
input means. With its input buffer enabled, a pad that nothing drives
leaks enough current through the input stage to sit high against its own
pull-down; a pad driven low and then released stays low. An idle level on
this silicon is history and not a measurement. A program that must read a
floating pad through its pulls drives it first, or uses the pull-up and
reads a low as its signal; for a pad that must be held by its pull-down,
[pin.md](pin.md) gives the erratum's own workaround, the input buffer
kept off and switched on for the read alone (`read_pulsed()`).

**There are two registers called PLATFORM, and the one in SYSINFO is not
the one that answers.** SYSINFO's (12.15.1, offset 0x08) is a
pre-production indicator - its own description says the platform "is
always ASIC, non-SIM" post-production - and on production silicon the
whole register reads zero, ASIC bit included (measured). The register
that answers whether this is an ASIC is TBMAN's, whose ASIC bit the data
sheet gives a reset value of 1 and which reads 1 here; `ChipId::asic()`
reads that one.

**The package is a build fact and a silicon fact at once.** The die is
one and every pin's registers exist on both packages; what the QFN-60 has
not got is the bond wire. A build that states its package refuses an
absent pad at compile time; a build that states none compiles for the
larger package and refuses at run time against SYSINFO's PACKAGE_SEL.

**THE PREEMPTIVE KERNEL'S REQUEST IS TWO ENGINES UNDER ONE NAME.** Rubato
([design/kernel.md](../design/kernel.md), "Rubato") asks its platform for
a way to run the scheduler at once from a dispatch and at the handler's
exit from an interrupt handler, and a place where the scheduler then
runs outside every handler. On the Cortex-M33 that is the core
stratum's PendSV and SVC, the same code the STM32F4 and the STM32G0 run
([../cortexm/README.md](../cortexm/README.md)): PendSV at the lowest of
the core's sixteen levels (3.7.2: four priority bits) returns into the
scheduler in thread mode through a second exception frame, and the
scheduler leaves through SVC. On Hazard3 it is a realization of this
stratum's own, written from 3.8.4:

- The request is the machine software interrupt. mip.MSIP is wired to
  the SIO's RISCV_SOFTIRQ register, one flag per hart, raised by a
  write of CORE0_SET or CORE1_SET and lowered by a write of the
  matching CLR bit (3.1.11,
  3.8.4.2.1): a level, which the scheduler's body lowers before
  anything else. The request raises the flag of the hart that asks.
- The datasheet does not say how many instructions retire between the
  store that raises the flag and the trap, so the request does not count
  on a latency: with mstatus.MIE set it waits until the scheduler has
  been served (a count the body moves in mscratch), and with MIE clear
  it returns at once, the flag taken when the mask lifts. Measured, one
  to two stores retire behind the raising store before the trap, and
  none behind the request ("Bench findings").
- Nothing nests, and nothing needs to. A trap entry clears mstatus.MIE
  (3.8.4) and the crt never sets it inside a handler, so every handler
  still runs masked to its end; what Rubato preempts is a dispatch, which
  runs outside every trap with MIE set. Xh3irq's preemption levels
  (3.8.6.1) are for an interrupt interrupting another, and stay unused.
- The vector is reached through the crt's trampoline, which saves the
  sixteen caller-saved registers in a 64-byte frame, calls
  `isr_riscv_softirq`, restores them and executes `mret`. The body bound
  there does not run the scheduler: it stores the interrupted code's
  mepc and mstatus in the two words just below the trampoline's frame,
  clears MPIE so that the trampoline's `mret` lands with MIE still
  clear, and points mepc at the scheduler's entry. The entry builds an
  80-byte frame whose words at offsets 8 and 12 ARE those two stores,
  saves the sixteen registers, sets MIE, runs the scheduler, restores,
  writes mstatus (the mask back on) and mepc from the two words and
  executes `mret` itself. `mret` is legal without a prior trap (3.8.4),
  and this core pushes nothing at a trap and keeps no per-interrupt
  state, so there is nothing to pair it with: no exit vector, no
  `ecall`, where the QingKe V4 of the CH32V203 needs one for its
  hardware prologue's pop.
- Hazard3 here has no floating-point unit (3.8.8), so the entry saves no
  f-register.

**WHAT THE CORTEX-M33'S HALF RESTS ON, CHECKED AGAINST THE DOCUMENTS.**
The core stratum's code was written for ARMv6-M and ARMv7-M; on this
ARMv8-M core with the Security Extension (3.7.2), running Secure as the
bootrom hands over:

- EXC_RETURN. The return value PendSV builds is the one it entered
  with, bit 4 set. Entered from thread mode in the Secure state on the
  main stack, that is 0xFFFFFFF9 (or 0xFFFFFFE9 with an FP frame):
  bit 0 ES = 1, the exception taken to the Secure state; bit 2 SPSEL =
  0, the main stack; bit 3 Mode = 1, thread; bit 4 FType = 1, a basic
  frame; bit 5 DCRS = 1, the default rules for the callee registers
  (none stacked); bit 6 S = 1, the frame on the Secure stack (DDI 0553,
  the EXC_RETURN payload). Bit 4 is the only one the code changes, and
  the three security bits are carried from the value the core wrote.
  This chip's SFSR.INVER (the M33 register list) names the two ways a
  return violates them - DCRS clear, or ES set returning to Non-secure -
  and neither applies.
- MSPLIM is zeroed by the crt (`startup_rp2350_arm.cpp`) and nothing
  sets it, so the fake frame, written below the real one, is never under
  a stack limit.
- PendSV's priority: `scheduler_init()` writes every bit of SHPR3's
  PRI_14 field, and the core implements four (3.7.2), so the byte is
  0xF0, the sixteenth level, below every NVIC line at its reset value 0.
- The FPU's lazy context save, which the core stratum leans on (an FP
  frame reserved by every exception over FP code, its words written
  only when an FP instruction runs before the return), is FPCCR's ASPEN
  and LSPEN. ARM's architecture resets both to 1, while this chip's
  FPCCR table gives both a reset value of 0, and so does the SVD it is
  generated from. **Measured, the register table is contradicted:**
  FPCCR reads 0xC000001C after the bootrom, ASPEN 1 and LSPEN 1, so
  every exception taken over FP code stacks the FP context lazily, under
  Tenuto as under Rubato, and the FP registers come back bit-exact
  across a preemption, a float-computing handler's included
  (`test_rp2350_rubato`'s letter j, "Bench findings").

## Types and verbs

- `Rp2350Platform<core = 0, TB = CoreTicker<core>>` - the brio Platform:
  `CriticalSection` (the RAII mask - a guard object that masks
  interrupts while it is in scope - per core in both spellings, saved and
  restored rather than cleared), `idle()` (sleep then unmask, through
  `sleep_hook` when a low-power state that is not a sleep instruction has
  armed one), `sleep_hook` (null unless a dormant site has armed one,
  [sleep.md](sleep.md)), `break_here()`,
  `now()`, `ticks_per_second`, `atomic_width` (4), `core_id()`,
  `on_own_core()` (what `EventQueue::push` checks before it copies),
  `Doorbell` (the bell a send to an AO of this core rings -
  [multicore.md](multicore.md)), and `panic_record()` - one `.noinit`
  record per core, a static of the template.
- `Rp2350RubatoPlatform<core = 0, TB = CoreTicker<core>>` - the platform
  of the preemptive kernel: `Rp2350Platform` whole, plus `preemptive`,
  `ceiling` (the pack index being dispatched, 255 at the loop's base) and
  `pend_schedule()`, the same text on both halves. Core 0 only - a
  `Rp2350RubatoPlatform<1>` does not compile: both cores of a pair enter
  one table of vectors, and which kernel a scheduler vector serves is
  the two-kernel question not yet answered.
- `schedule.hpp`'s names, the same on both halves: `request_scheduler()`
  (what `pend_schedule()` calls), `scheduler_init()` (PendSV to its
  level on one half; the hart's flag lowered and mie.MSIE set on the
  other), `scheduler_thread<K>()` (the entry the scheduler runs in), and
  the binding `BRIO_RP2350_SCHEDULER_VECTORS(K)`, which defines
  `isr_pendsv` and `isr_svcall` on the Cortex-M33 and
  `isr_riscv_softirq` on Hazard3; behind it the bodies
  `scheduler_exception<K>()` and `scheduler_return()` (the core
  stratum's) and `scheduler_trap<K>()` (Hazard3's).
- `core.hpp`'s names, the same on both halves: `InterruptGuard`,
  `enable_interrupts()`, `disable_interrupts()`, `interrupts_enabled()`,
  `in_handler()` (IPSR's exception number on the Cortex-M33;
  `mstatus.MIE` clear on Hazard3, which a trap's entry clears and the
  crt never sets again before the return, so a critical section reads
  as a handler too and no service asks it under a guard), `Irq` (the per-line controller over the device header's `IRQn_Type`:
  `enable`, `disable`, `enabled`, `set_pending`, `clear_pending`,
  `pending`), `irq_priority_levels`, `wait_for_interrupt()`,
  `debug_break()`, `core_id()`. Plus `CoreKind` and `core_kind`, a value,
  so a driver with something genuinely different to do branches on it
  with `if constexpr` instead of asking the preprocessor. `core_kind` is
  the image's own claim, and it is what this suite reports; the chip's
  answer to which architecture is running is ARCHSEL_STATUS, in the OTP
  block's register window, which `Otp::archsel_status()` reads
  ([otp.md](otp.md), measured there); read over the debug port it is 0
  after an Arm image and 3 after a RISC-V one.
- `Ticker` / `CoreTicker<core>` - the kernel timebase at 1000 Hz, one per
  core: `init(clock)`, `tick()` (the handler's body, bound as
  `isr_systick` on both halves), `ticks`, `millis`, `secs`, `now`,
  `advance`, `pause`, `resume`, `rebase`, `ticks_per_second`. This
  suite runs `pause` and `resume` around letter `w`'s passes and judges
  neither; `Ticker::advance` is the sleep sites' resync, measured in
  [sleep.md](sleep.md), and `pause` and `resume` are judged per core in
  [multicore.md](multicore.md).
- `delay_us` - the microsecond busy-wait over the ruler, in
  `rp2350/delay.hpp`: at least the time asked, refused at run time (it
  answers false and waits nothing) from a whole kernel tick up;
  documented and measured in [clock.md](clock.md). This page's suite
  does not use it: every wait there is a spin on `Mtime::micros()`.
- `Mtime` - the ruler: `start(clock)` (idempotent; false when clk_ref is
  not a whole number of megahertz), `running()`, `now()` (64 bits, read
  against a rollover of its low half), `micros()` (the low half alone,
  one bus read and 71 minutes of range), `set_compare`, `compare`,
  `disarm`.
- `Resets` and `ResetBlock` - the subsystem reset controller:
  `release(blocks)` (with its bounded RESET_DONE wait), `hold`, `cycle`,
  `released`, `held`, `watchdog_resets`.
- `ChipId` - SYSINFO: `read()` (manufacturer, part, the stepping the
  errata are keyed by, with `revision_a2` / `a3` / `a4` beside it),
  `package_sel()`, `gitref()`, and `asic()`, which reads the testbench
  manager's register and not SYSINFO's of the same name.
- `device.hpp`'s `hw_set` / `hw_clear` / `hw_xor` - 2.1.3's three
  write-only aliases of every APB and AHB register, one bus write and no
  read-modify-write, which is what lets a bit be flipped from a handler
  or from the other core with no critical section - and `hw_write_masked`
  over them, which reads the register first and then makes one XOR-alias
  write: a read-modify-write again, fine for configuration and not for a
  field two contexts race on; and
  `Package`, `package_gpio_count`, `gpio_count`, `gpio_count_max`,
  `package_known`.

## How to use it

An app names the platform, the timebase and the clock, and binds its
vectors by the crt's names - the same names on both architectures:

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

Reading a pad that nothing drives, on a part where E9 is live (`Pad`
is any `brio::Pin<n>`, [pin.md](pin.md)):

```cpp
(void)Pad::input(brio::PinPull::up);   // and a LOW is the signal
```

The preemptive kernel on core 0, one source for both architectures: the
macro binds PendSV and SVCall on the Cortex-M33 and the machine software
interrupt on Hazard3, and `scheduler_init()` comes before the unmask:

```cpp
using P = brio::Rp2350RubatoPlatform<>;
using K = brio::Rubato<P, Sensor, Logger>;   // highest priority first

BRIO_RP2350_SCHEDULER_VECTORS(K)
extern "C" void isr_systick() { brio::Ticker::tick(); }

int main() {
    SysClock::init();
    brio::Ticker::init(clock);
    brio::scheduler_init();
    brio::enable_interrupts();
    K::run();
}
```

Binding the vectors by hand is the macro's expansion: on the Cortex-M33

```cpp
extern "C" [[gnu::naked]] void isr_pendsv() { brio::scheduler_exception<K>(); }
extern "C" [[gnu::naked]] void isr_svcall() { brio::scheduler_return(); }
```

and on Hazard3

```cpp
extern "C" [[gnu::naked]] void isr_riscv_softirq() { brio::scheduler_trap<K>(); }
```

A program on this platform executes no `svc` of its own on the
Cortex-M33, leaves mscratch alone on Hazard3, and never writes the
hart's RISCV_SOFTIRQ flag.

## Bench findings

`test_rp2350_platform` on the bench part (a QFN-80 stepping A2, clk_sys
150 MHz from the 12 MHz crystal through the system PLL), green on both
architectures from one source. Where the halves differ, both numbers:

| What | Cortex-M33 | Hazard3 |
|------|-----------|---------|
| the suite | 70 pass, 0 fail | 70 pass, 0 fail |
| 200 kernel ticks on the ruler | 200000 us, 0 ppm | 200000 us, 0 ppm |
| `idle()` calls covering 200 ticks | 200 | 200 |
| awake in that span | 3 us of 199960 | 1 us of 199397 |
| a wakeup raised INSIDE the mask | back in 3 us | back in 1 us |
| a 5 ms masked window's ticks | 1 | 5 |
| `Resets::release()` to RESET_DONE | 1 us | 0 us |
| edge positions walked, lost / a period late | 4800, 0 / 0 | 3200, 0 / 0 |
| the comparator's edge, the core asleep, to the handler / to the loop | 31 / 79 cycles | 31 / 90 (36 / 95 in standby) |
| the PWM line's edge, the same | 31 / 76 cycles | 36 / 92 (41 / 97 in standby) |
| SysTick's edge, the same (the Cortex-M33's tick) | 26 / 88 cycles | - |
| kernel turns over 100 quiet ticks | 100 | 100 |
| `idle()` with a line pending / a quiet turn that finds one | 57 / 167 cycles | 82 / 175 cycles |

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
  millisecond (letter `j`: a wake pending AT the call, on the
  microsecond ruler; the edge at every instruction is letter `w`'s).
- NO WAKE IS LOST WHEREVER THE INTERRUPT LANDS, on either half (letter
  `w`). An edge is placed D cycles after a placement, D walking 400
  consecutive cycles, and the kernel's own shape runs against it - a
  masked check, `idle()`, until one edge was served, and in a second pass
  two (the sleep entered right after a handler) - with TIMER0's alarm as
  a rescue 50 ms out and TIMER1, switched to clk_sys (12.8.1.1's SOURCE),
  timing each try in cycles on both halves. Two edges, placed the same
  way on both: the PLATFORM TIMER'S COMPARATOR, the counter run on clk_sys
  for the letter (MTIME_CTRL.FULLSPEED, 3.1.8) so that a counter write
  places it - the kernel's own tick on Hazard3, the system line
  SIO_IRQ_MTIMECMP on the Cortex-M33 -, and a PERIPHERAL LINE, PWM slice
  0 counting clk_sys to its wrap, placed by a counter write, through the
  NVIC on one half and Xh3irq on the other. Each in the light rung and in
  standby through `Rp2350SleepSite`, which on Hazard3 releases the core's
  power request (MSLEEP.POWERDOWN) and on the Cortex-M33 changes nothing
  the core does. And on the Cortex-M33 alone a THIRD edge, its own tick:
  SysTick, a core exception, placed by a reload of D (LOAD = D, a VAL
  write, LOAD back) - SysTick has no name on the other half. 4800 tries
  on the Cortex-M33 and 3200 on Hazard3 (ten runs of the letter on the
  Arm half): none took the rescue, none ended a period late; SysTick's
  edge finds the core asleep from D = 88, 81 cycles after the earliest. The edge crosses the whole path: the earliest lands 80 to
  89 cycles before the sleep takes it (a fixed lead of bus reads sits
  between the placement and the loop, longer than the path from the
  loop's masked check to the sleep instruction, about 20 cycles in
  either listing), and from D = 81 to 90 on the edge finds the core asleep.
- WHAT A WAKE COSTS: from an edge that finds the core asleep to the
  handler's first statement 31 cycles on the Cortex-M33 for a line and 26
  for SysTick - its exception entry, the vector fetched with the state
  saved - and 31 to 36 on
  Hazard3, whose dispatch is the crt's own (the trap, the caller-saved
  registers, `meinext` for a line); releasing the power request adds 5
  cycles a wake there, the core waiting for its power-up acknowledge as
  3.8.9 says. To the caller's loop, the handler whole and the return: 76
  to 88 cycles on one half, 90 to 97 on the other. The datasheet gives
  neither core's entry in cycles.
- ONE TURN PER INTERRUPT, on both halves (letter `o`): a Tenuto pack of
  three quiet AOs, two of them with a periodic time event, turned as
  `run()` turns it with the tick the only interrupt - 100 turns over 100
  ticks, a turn more or fewer allowed at the window's ends. On TIMER1's
  cycles, `idle()` with a line already pending (the sleep falls through,
  the unmask takes the line's handler) costs 57 cycles on the Cortex-M33
  and 82 on Hazard3, and a quiet turn entered masked whose `idle()` finds
  one 167 and 175: the loop's own share - the time events' `process()`,
  the empty `step()`, the masked check - is 110 and 93 cycles. Letter `h`
  (200 `idle()` calls over 200 ticks) proves the same of a bare `idle()`
  loop, without the kernel.
- WHAT THE FULL-SPEED COMPARATOR COSTS THE LETTER: the platform timer is
  the ruler of both halves and the timebase of one, so the letter puts
  it back on its microsecond tick at the value it would have reached
  (TIMER0 counted the span) and the comparator a period out. Kernel time
  on Hazard3 runs 150 times fast meanwhile - a tick every thousand
  cycles - and the letters after it judge spans, never an absolute
  count.
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

**The preemptive kernel** - the reference suite is `test_rp2350_rubato`,
one source and two images on the WeAct RP2350B at 150 MHz: 44 verdicts
in `z` on the Cortex-M33 and 46 on Hazard3, ten runs of `z` alike on
each half. A Rubato pack of five objects on `Rp2350RubatoPlatform<0>`,
the Console the lowest and the letters' runner, TIMER1's alarm 0 the
interrupt the road-1 letters land, nothing wired and nothing lit. The
ruler is TIMER1 itself with its source on clk_sys, two back-to-back
reads 3 cycles apart on both halves; beside it the core's own counter -
on the Cortex-M33 the kernel ticker's composed `cycles()`, two reads 41
cycles apart, agreeing with TIMER1 over 1 to 1.06 ms within 0 to 1
cycle on every clean span, and on Hazard3 `mcycle`, two reads 1 cycle
apart, agreeing within 0 cycles on every clean span. A span with an
XIP refill inside a pair of reads is reported and not judged - the
first of every run has one, its second pair's code cold, and reads 0
to 50 cycles off -, so spans are taken until four are clean (five in
every run). That is the suite's method throughout: an event, a run or a
span that a kernel tick, an XIP refill or the console line's handler
lengthened is told apart by counters read around it (the tick count,
the XIP cache's miss counter, the console vector's entries) and
reported, and each bound judges the ones nothing lengthened.

On both halves road 1 preempts at any depth: an interrupt's post to
High inside Low's dispatch runs High to its end before Low resumes,
every object reading its own index as the ceiling and the UART's
interrupt the base's 255; road 2 nests High inside Mid inside Low, each
inside the post that asked; at depth 2 an interrupt over High's
dispatch, started by either road, runs Top before High resumes; three
timers matured in one turn are posted with the ceiling at 0 and served
highest first, and High preempting Low's loop of `arm()` and `disarm()`
leaves the list whole; a post under the mask (PRIMASK on one half,
mstatus.MIE on the other) leaves the request pending and is served when
the section ends, before the statement after it; dark at the end, the
request as `scheduler_init()` set it. clk_sys cycles, the ruler's own
cost taken off:

| clk_sys cycles | Cortex-M33 | Hazard3 |
|---|---|---|
| road 1: an interrupt's last statement to the preempting dispatch's first, min / median / max over 100 events | 103 / 103 / 132 | 175 / 175 / 175 |
| road 1: the path's count by the release listing, and the bound | 126, 210 | 172, 277 |
| road 1 at depth 2, over a preempting dispatch | 93 | 167 |
| `activate()` over four empty queues | 65 | 68 |
| a post that asks nothing | 59 | 54 |
| a road-2 post, the receiver's empty dispatch inside (the scheduling's share) | 295 (236) | 319 (265) |
| one `arm()`, its masked window inside | 61 | 45 to 51 |
| a post under the mask: the section's end to the receiver's first | 114 | 158 |
| stack high-water of the suite's deepest chain (bytes), the deepest sample in a dispatch / in the interrupt | 552 to 656 (544 / 488) | 904 (864 / 736) |

The Cortex-M33's road-1 events over 103 are the ones a tick or a refill
landed in, and its measured path runs under the count drawn with the
Cortex-M4's table, the model the suite takes for want of the M33's own
figures: the count is 23 cycles over the measurement on road 1 and 15
to 79 over it on the other paths. On Hazard3 the model, 3.8.7's counts
with the interrupt's entry (which the section does not give) taken at
`ecall`'s, is 3 cycles under the measurement on road 1, depth 2 and the
post under the mask, and 1 to 2 over it on the scan and the road-2
post. Each queue's emptiness is its two cursors compared in line, no
call. A post under the mask lengthened by a kernel tick or an XIP
refill reached 746 cycles on the Cortex-M33 and 1015 on Hazard3, and is
reported, not judged. The
Cortex-M33's high-water mark read 552 bytes in the first three runs
after the flash and 656 from the fourth on: the paint is laid once at
boot, so the figure is the deepest since the flash, and both are inside
the bound, 544 + 312. The two halves' rows part where their mechanisms
do: on the Cortex-M33 the hardware stacks the interrupted code's
registers, on Hazard3 the crt's trampoline and the scheduler's entry
each save sixteen in software ("What the silicon does"), and the
stack's levels carry those frames. The Cortex-M33's PendSV body is
thirteen instructions and 32 bytes, without the DSB that is the
Cortex-M4's alone ([../cortexm/README.md](../cortexm/README.md)); a
build that kept it measured road 1 two cycles longer, the barrier's
cost on this core.

- **The FPU across a preemption** (letter j, the Cortex-M33 alone):
  FPCCR reads 0xC000001C, ASPEN 1 and LSPEN 1, where the datasheet's
  register table and the SVD give both a reset value of 0; Low's float
  series, preempted at a mark inside its loop by High's own series, and
  in a second run by an interrupt handler that computes a float too,
  ends with every value bit-exact against the same functions run without
  preemption.
- **The software interrupt's own facts** (letter k, Hazard3 alone):
  after the raw store that raises the hart's flag, 1 to 2 of the eight
  stores behind it retire before the trap - the latency the request
  waits out - and behind `request_scheduler()` none; each raise serves
  Top exactly once (20 raises); the caller-saved registers, loaded with
  patterns, are found again across an entry and across a nested one,
  10 of 10 each; inside a dispatch the software interrupt started,
  the flag is low, mstatus.MIE set and mscratch moved by one.

## A handler in SRAM

**What the silicon does.** Both architectures execute the flash in place
through the XIP cache (4.4.1): 16 kB, two-way set-associative, lines of
8 bytes, a hit in one cycle, in front of the QSPI flash the bootrom left
at EBh and CLKDIV 3 ([flash.md](flash.md)). Warm code runs at about the
SRAM's pace; a MISS fetches its line over the QSPI bus - 66 to 82 cycles
a line at 150 MHz ([spi.md](spi.md)). A path runs cold the first time
after `init()`, after an invalidate, or once other code has evicted its
lines. On the Cortex-M33 a `bl` reaches 16 MB and the SRAM lies 256 MB
from the flash, so a call between the two goes through a linker veneer
(`ldr.w pc, [pc]` and the address); Hazard3's `auipc` and `jalr` reach
anywhere, and a call across is an ordinary call. A vector placed in
SRAM is entered with no veneer on either: the Arm table and the Hazard3
dispatch's table (`startup_rp2350_riscv.S`) hold absolute addresses -
but on Hazard3 THE DISPATCH ITSELF, the trap entry, the register save
and the `meinext` loop, stays in the flash.

**The option and its spelling** are the RP2040's
([../rp2040/platform.md](../rp2040/platform.md), "A handler in SRAM"):
`rp2350_16m.ld` collects `.fast_text` then `.ram_text` first in `.data`,
copied by both crts, the platform says `places_fast_text`, and the
program either places the vector it binds or gives `SpiHost`, `Uart` or
`I2cHost` util/placement.hpp's `InRam` - one flattened instance in
`.fast_text.brio` a path, `InFlash` the default.

**What it buys, measured** (`bench_rp2350` and `bench_rp2350_ram` on
each architecture; cycles at 150 MHz, each cell WARM / COLD, cold read by
letter c, every line of the XIP cache invalidated before every timed
run; F the default, R the drivers InRam, V the vectors placed, B both):

| Cortex-M33 | F | R | V | B |
|---|---|---|---|---|
| SPI polled request, a command byte and 2 data bytes | 671 / 5333 | 676 / 1241 | 671 / 5404 | 676 / 1241 |
| SPI pump, 16 frames at clk_peri / 2, wall | 945 / 8810 | 980 / 3294 | 946 / 5088 | 973 / 1812 |
| ... its handler, two entries | 608 / 3785 | 647 / 1577 | 607 / 610 | 639 / 642 |
| SPI on the engines, 16 frames, wall | 1433 / 12525 | 791 / 4192 | 770 / 7800 | 788 / 1776 |
| ... the DMA line's handler | 342 / 5010 | 139 / 1877 | 116 / 116 | 135 / 135 |
| UART interrupt receiver, 16 bytes at 1 Mbaud, handler | 1278 / 6390 | 549 / 3160 | 487 / 487 | 543 / 543 |
| UART transmit engine, 256 bytes at 1 Mbaud, completions | 1107 / 4129 | 286 / 1956 | 290 / 1124 | 314 / 1148 |
| I2C one-byte write at 400 kHz, `start()` | 182 / 4354 | 189 / 477 | 184 / 4311 | 189 / 477 |
| ... its longest handler entry | 88 / 2960 | 114 / 1158 | 88 / 88 | 110 / 110 |
| I2C 16-byte read on the engines at 400 kHz, wall | 58903 / 66330 | 58935 / 61564 | 58905 / 62598 | 58930 / 59548 |

| Hazard3 | F | R | V | B |
|---|---|---|---|---|
| SPI polled request, a command byte and 2 data bytes | 633 / 4621 | 637 / 1034 | 639 / 4610 | 637 / 1034 |
| SPI pump, 16 frames at clk_peri / 2, wall | 948 / 8490 | 988 / 3731 | 964 / 5481 | 988 / 2590 |
| ... its handler, two entries | 603 / 3192 | 635 / 1302 | 615 / 1261 | 633 / 636 |
| SPI on the engines, 16 frames, wall | 844 / 9980 | 749 / 3700 | 737 / 7072 | 748 / 2439 |
| ... the DMA line's handler | 100 / 2657 | 119 / 1104 | 99 / 99 | 119 / 762 |
| UART interrupt receiver, 16 bytes at 1 Mbaud, handler | 2214 / 5421 | 460 / 2840 | 454 / 454 | 460 / 460 |
| UART transmit engine, 256 bytes at 1 Mbaud, completions | 1296 / 3385 | 269 / 1827 | 222 / 894 | 272 / 991 |
| I2C one-byte write at 400 kHz, `start()` | 178 / 4009 | 185 / 1831 | 176 / 3963 | 185 / 1831 |
| ... its longest handler entry | 226 / 1669 | 72 / 488 | 70 / 70 | 71 / 71 |
| I2C 16-byte read on the engines at 400 kHz, wall | 58924 / 64414 | 58945 / 61864 | 58923 / 62333 | 58945 / 60751 |

What the rows say - the RP2040's reading on both halves:

- WARM, the placement buys nothing on a path the cache holds and costs
  the call: 4 to 40 cycles under InRam. A warm figure that differs more
  is one image's set conflicts, not the place: the F column's engined
  SPI block (1433 against 770 to 791) and its UART and I2C handlers miss
  lines warm that the other three images do not.
- COLD it pays on every path, on both cores: a three-byte request 5333
  cycles from the flash against 1241 with the host InRam on the M33, a
  handler cold from the flash 2000 to 5000 cycles, from SRAM what it
  costs warm.
- The vector placed reaches what the vector runs; InRam reaches
  `start()`; BOTH is the image whose cold transaction is closest to its
  warm one. On Hazard3 the dispatch in front of a placed vector still
  misses, and two of its placed bodies read cold above warm (the pump's
  handler under V 1261 against 615, the DMA line's under B 762 against
  119) with no call into the flash in their listings - a residue this
  chapter has not split.

## Not covered yet

Driver gaps, each with its reason:

- PINNED XIP CACHE LINES (4.4.1: a line marked never to be evicted, and
  4.4.1.3's cache-as-SRAM): the other way to keep a path warm, by set and
  way at run time, every line pinned a line of the 16 kB no other path
  can use. `.fast_text` is the one placement this
  stratum offers; a pin is born with a program whose hot code will not
  fit the SRAM it can spare.
- The Hazard3 dispatch in SRAM: the crt's trap entry, its register save
  and its `meinext` loop run from the flash in front of every vector, a
  placed one too ("A handler in SRAM"), and the residue measured there on
  two placed bodies is not split. A placed copy of the dispatch is born
  with a program whose handlers must answer cold within a deadline.
- The reboot and the causes of a boot: they belong to another chapter -
  [reset.md](reset.md) for the causes and the two reset verbs,
  [watchdog.md](watchdog.md) for the countdown and the scratch
  registers - which is why the breadcrumb letter HERE is driven by a reset from
  the debug port and is outside `z`. The `.noinit` survival of a
  watchdog reset is measured there (across a reboot and a time-out,
  reset.md's findings); a rescue's is measured nowhere, a rescue being
  the flash verb's own path, and a letter that banks a token in
  `.noinit` before one and reads it after would measure it.
- Interrupt priorities. Both halves have sixteen levels and no driver
  uses them: the kernel's promise is that no interrupt nests over
  another, and it is kept structurally here (every line at the reset
  priority on the Cortex-M33, mstatus.MIE never set inside a handler on
  Hazard3). The one priority written anywhere is PendSV's, by the
  preemptive kernel's `scheduler_init()` on the Cortex-M33, below every
  line. The two conventions run opposite ways - numerically higher is
  more urgent on Hazard3 - which is why `Irq` carries no priority verb
  at all.
- Rubato on core 1, on either half: both crts give a pair one table, so
  the scheduler's vectors would serve two kernels through a body that
  asks the core which kernel to run, the way the RP2040 takes
  ([../rp2040/platform.md](../rp2040/platform.md), "How to use it"). That
  body is not written for either half yet, and until it is
  `Rp2350RubatoPlatform<1>` does not compile.
- `idle_until()`. The timebase runs while the core sleeps on either
  half, in the plain idle path and through a standby; the one rung that
  stops it, DORMANT, is repaired afterwards only under the timed sleep
  site, `Rp2350TimedSleepSite`, whose resync from the always-on timer
  restores the time ([sleep.md](sleep.md)) - under the plain site the
  deep rung has no witness and the time a dormant spent is lost - and
  the platform offers no tickless idle.
- The Cortex-M33's own extensions - the MPU, the SAU and TrustZone,
  MSPLIM, BASEPRI, the coprocessor ports: everything here runs Secure, as
  the bootrom hands over, and nothing has needed one.

Implemented but not bench-verified, each with what would measure it:

- CORE 1, on either architecture. The platform is per core and the
  tickers are per core by construction, and the suite proves that
  `on_own_core()` agrees with CPUID on core 0; this suite launches no
  second core (core 1's launch, its ticker, its kernel and its panic are
  measured in [multicore.md](multicore.md), with the bell and the
  bridge). The idle contract there would want letters `w` and `o`
  launched on core 1 with `Rp2350Platform<1>` and `CoreTicker<1>` (its
  own SysTick, its own comparator), the rescue enabled in core 1's
  controller, and the SIO doorbell as a third edge placed from core 0.
- The QFN-60's run-time refusals: the stratum compiles for that package
  and the suite checks the die's own PACKAGE_SEL against the build's, but
  no QFN-60 part is on the bench.
- The idle path through the DORMANT hook: its wake list is the dormant's
  own (a GPIO event, the always-on timer's alarm), and a wake already
  latched when the oscillator stops is not shown to end it at once. A
  pad the chip drives itself, its dormant-wake event armed, edges walked
  across the hook with the always-on timer as the rescue.
- `Mtime` under a clk_ref that is not 12 MHz: the arithmetic refuses one
  that is not a whole number of megahertz, and only the 12 MHz crystal
  has been on the wire.
- `break_here()` without a probe attached. Under a probe with halting
  debug enabled it halts the core; the flash verb of this target takes
  halting debug back down afterwards so that it faults instead, and the
  fault path itself is the reset chapter's to catch.
