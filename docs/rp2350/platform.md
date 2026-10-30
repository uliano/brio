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
([multicore.md](multicore.md)) - a Tenuto or a Rubato on each - and one
type for both architectures: the
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
counting cycles, the unit of the rows of letters `w`, `o` and `i` - the
last the cost models the preemptive kernel's bounds are drawn with.

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
priority bits), the M33 register list's FPCCR table (3.7.5), 3.1.2 (SIO's
CPUID, which the two-kernel body asks, and its note that MHARTID answers
the same), 2.2.6 (the SIO's zero-wait access), 3.8.4
(Hazard3's trap entry and `mret`), 3.8.4.2.1 (the software interrupt and
the SIO's RISCV_SOFTIRQ register, 3.1.11), 3.8.6.1 (Xh3irq's preemption,
which is not used), 3.8.7 (Hazard3's instruction cycle counts) and 3.8.8
(its configuration); ARM's ARMv8-M Architecture Reference Manual
(DDI 0553) for EXC_RETURN; and ARM's Cortex-M33 Technical Reference
Manual (100230, issue 08) for 1.4.1 (the core's limited dual issue of
16-bit instruction pairs) and 3.4 (ACTLR and its DISFOLD bit) - the
manual gives no instruction timing, so the cycle figures of this core
are the suite's own ("Bench findings"). The drivers: `brio/rp2350/platform.hpp` (the
brio Platform itself, and the Rubato platform beside it), `core.hpp`
with `core_m33.hpp` and `core_hazard3.hpp` (the one file of the stratum
that asks which processor is in the socket, and its two halves),
`schedule.hpp` with `schedule_hazard3.hpp` (the preemptive kernel's
request, the core stratum's on one half and this stratum's on the
other), `ticker.hpp` (the kernel timebase, one per core), `mtime.hpp`
(the microsecond ruler), `resets.hpp`, `sysinfo.hpp` and `device.hpp`;
4.4.1 (the XIP cache), 4.4.5 (its registers, the two counters in
tables 452 and 453) and `brio/util/placement.hpp` for the placement.
The reference suites: `test_rp2350_platform`, which runs on both
architectures from one source and is judged the same way on each (a
letter is one command key of the suite's console and the group of
verdicts it runs; `z` is the key that runs every letter but two, which
run by name: `r`, the breadcrumb across a processor reset, and `x`,
`break_here()`, which stops the console), `test_rp2350_rubato` for
the preemptive kernel, and `test_rp2350_rubato_link` for the drivers
under it, the same on both ("Bench findings").

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

**TWO KERNELS SHARE ONE SCHEDULER ENTRY, AND THE ENTRY ASKS THE CORE.**
Both crts give the two cores of a pair one table: core 1 is launched with
core 0's vector base (VTOR on one half, mtvec on the other,
[multicore.md](multicore.md)), so PendSV and SVCall, or the software
interrupt's trampoline, are one entry each for both cores. A program with
a Rubato on each core binds the pair body, which reads which core took
the entry and indexes a table of two scheduler entries placed after it -
no branch, one path of one length on both cores. On the Cortex-M33 the
question is SIO's CPUID (3.1.2), one load over the core's AHB port to the
SIO, zero wait states (2.2.6; this core has no single-cycle IO port of
the kind the RP2040's Cortex-M0+ had), its address one MOV.W of a
modified immediate, and the entry one load with the index scaled in the
instruction: four instructions in place of the single body's literal
load. On Hazard3 it is the hart's mhartid, the same answer (3.1.2's
note), which the body reads anyway to lower the hart's flag, and the
entry is one `sh2add` (Zba) and one load placed where no instruction
waits on it. The core's own CPUID register in the System Control Space
reads the same part number on both cores and cannot be the question; the
stack pointer's bank would tell the cores apart only while each stack
stayed where the linker script puts it. Everything else is per core
already: PendSV's priority in each M33's SHPR3 (so `scheduler_init()`
runs on each core), mie, mscratch and the software interrupt flag of each
hart, each core's stack and each kernel's ceiling. A vector table per
core would spare the question at every request for a second table every
bound vector must be kept in step with; it is declined.

**THE REQUEST IS MADE ONLY FROM THE PLATFORM'S OWN CORE.** `post()` asks
for the scheduler after the queue's push whenever the receiver's index is
below its platform's ceiling, and a post from the other core is one the
push has already refused and counted. Asking then would raise the posting
core's request - PendSV in its own ICSR, its own hart's flag - and with a
Rubato on core 0 alone and core 1 under Tenuto, the single body taken on
core 1 would run core 0's kernel there (the Cortex-M33), or core 1, its
software interrupt not enabled, would spin in the request's wait for ever
(Hazard3). `pend_schedule()` is therefore `request_scheduler_on<core>()`,
which asks only when the calling core is the platform's: SIO's CPUID
compared first on the Cortex-M33, the hart id the request reads anyway on
Hazard3.

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
  FPCCR reads 0xC000001C after the bootrom (0xC0000004 in the first run
  after a flash), ASPEN 1 and LSPEN 1 either way, so
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
  of the preemptive kernel, one per core: `Rp2350Platform` whole, plus
  `preemptive`, `ceiling` (the pack index being dispatched on that core,
  255 at the loop's base) and `pend_schedule()`, which is
  `request_scheduler_on<core>()`; the same text on both halves.
- `schedule.hpp`'s names, the same on both halves: `request_scheduler()`
  (the calling core's request, unchecked), `request_scheduler_on<core>()`
  (the same, made only when the calling core is `core`: what
  `pend_schedule()` calls), `scheduler_init()` (for the core that calls
  it: PendSV to its level on one half; the hart's flag lowered and
  mie.MSIE set on the other), `scheduler_thread<K>()` (the entry the
  scheduler runs in), and the two bindings:
  `BRIO_RP2350_SCHEDULER_VECTORS(K)` for one kernel, core 0's (a kernel
  of core 1 is refused at compile time), and
  `BRIO_RP2350_SCHEDULER_VECTORS_PAIR(K0, K1)` for a Rubato on each core
  (the two in the other order, or either not a Rubato of its core, are
  refused at compile time). Each defines `isr_pendsv` and `isr_svcall`
  on the Cortex-M33 and `isr_riscv_softirq` on Hazard3; behind them the
  bodies `scheduler_exception<K>()` and `scheduler_return()` (the core
  stratum's), `scheduler_exception_pair<K0, K1>()` (this stratum's, the
  Cortex-M33), and `scheduler_trap<K>()` and
  `scheduler_trap_pair<K0, K1>()` (Hazard3's).
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

A Rubato on each core binds the pair body instead, and each core calls
`scheduler_init()` in its own start-up. An excerpt - the clock, core 0's
pack and core 0's `main()` as above; `Motor` and `Watch` stand for core
1's active objects, `Motor` one that receives from core 0 through its
inbox:

```cpp
using P0 = brio::Rp2350RubatoPlatform<0>;
using P1 = brio::Rp2350RubatoPlatform<1>;
using K0 = brio::Rubato<P0, Sensor, Logger>;
using K1 = brio::Rubato<P1, Motor, Watch>;

BRIO_RP2350_SCHEDULER_VECTORS_PAIR(K0, K1)

[[noreturn]] void core1_main() {
    brio::CoreTicker<1>::init(clock);
    brio::scheduler_init();          // core 1's own PendSV priority, or its own hart's MSIE
    brio::Inboxes<Motor>::enable();  // core 1's doorbell, in its own controller
    brio::enable_interrupts();
    K1::run();
}
```

The excerpt discards two answers: `CoreTicker<1>::init()` is false when
the ticker cannot be set (the SysTick reload not fitting on one half,
the reference clock not a whole number of megahertz on the other, as
the single-core example says), and `Inboxes<Motor>::enable()` is false on any core but Motor's,
which `core1_main()` runs on by construction; a program that cannot
rule the first out checks it before the unmask.

Core 0's `main()` calls `brio::scheduler_init()` before its own unmask
and `brio::Core1::launch(&core1_main)`, whose answer says whether core 1
came up ([multicore.md](multicore.md) has the launch and the doorbell's
vector, which asks the core that took it). A post to an active object of
the other core is refused and counted by its queue and raises no
request on either core; an event crosses with `send()` through the
inbox, whose doorbell handler posts on the receiving core - road 1
there, taken at that handler's exit.

A program on this platform executes no `svc` of its own on the
Cortex-M33, leaves mscratch alone on Hazard3, and never writes a
hart's RISCV_SOFTIRQ flag.

## Bench findings

`test_rp2350_platform` on the bench part (a QFN-80 stepping A2, clk_sys
150 MHz from the 12 MHz crystal through the system PLL), green on both
architectures from one source. Where the halves differ, both numbers:

| What | Cortex-M33 | Hazard3 |
|------|-----------|---------|
| the suite | 75 pass, 0 fail | 71 pass, 0 fail |
| 200 kernel ticks on the ruler | 200000 us, 0 ppm | 199999 to 200000 us, 0 to 5 ppm |
| `idle()` calls covering 200 ticks | 200 | 200 |
| awake in that span (about 199.0 to 200.0 ms) | 1 to 3 us | 1 to 3 us |
| a wakeup raised INSIDE the mask | back in 0 to 4 us | back in 3 to 4 us |
| a 5 ms masked window's ticks | 1 | 5 |
| `Resets::release()` to RESET_DONE | 0 to 1 us | 0 to 1 us |
| edge positions walked, lost / a period late | 4800, 0 / 0 | 3200, 0 / 0 |
| the comparator's edge, the core asleep, to the handler / to the loop | 31 / 79 cycles | 30 / 88 (35 / 93 in standby) |
| the PWM line's edge, the same | 31 / 76 cycles | 35 / 92 (40 / 97 in standby) |
| SysTick's edge, the same (the Cortex-M33's tick) | 26 / 88 cycles | - |
| kernel turns over 100 quiet ticks | 100 | 100 |
| `idle()` with a line pending / a quiet turn that finds one | 57 / 179 cycles | 84 / 185 cycles |

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
  immediately after comes back in 0 to 4 us on one half and 3 to 4 us on
the other,
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
  92 cycles before the sleep takes it (a fixed lead of bus reads sits
  between the placement and the loop, longer than the path from the
  loop's masked check to the sleep instruction, about 20 cycles in
  either listing), and from D = 81 to 93 on the edge finds the core asleep.
- WHAT A WAKE COSTS: from an edge that finds the core asleep to the
  handler's first statement 31 cycles on the Cortex-M33 for a line and 26
  for SysTick - its exception entry, the vector fetched with the state
  saved - and 30 to 35 on
  Hazard3, whose dispatch is the crt's own (the trap, the caller-saved
  registers, `meinext` for a line); releasing the power request adds 5
  cycles a wake there, the core waiting for its power-up acknowledge as
  3.8.9 says. To the caller's loop, the handler whole and the return: 76
  to 88 cycles on one half, 88 to 97 on the other. The datasheet gives
  neither core's entry in cycles.
- ONE TURN PER INTERRUPT, on both halves (letter `o`): a Tenuto pack of
  three quiet AOs, two of them with a periodic time event, turned as
  `run()` turns it with the tick the only interrupt - 100 turns over 100
  ticks, a turn more or fewer allowed at the window's ends. On TIMER1's
  cycles, `idle()` with a line already pending (the sleep falls through,
  the unmask takes the line's handler) costs 57 cycles on the Cortex-M33
  and 84 on Hazard3, and a quiet turn entered masked whose `idle()` finds
  one 179 and 185: the loop's own share - the time events' `process()`,
  the empty `step()`, the masked check - is 122 and 101 cycles. Letter `h`
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

**The cost models** - letter `i`, four verdicts on the Cortex-M33 and
one on Hazard3, ten runs of `z` on each half with every figure the same
in all ten: the cycle figures `test_rp2350_rubato`'s bounds are drawn
with. Each primitive is a run of K identical units of hand-written
assembly (K 32 for a one-cycle unit, 16 or 8 for a longer one) in the
flash with the XIP cache warm - the memory the preemptive kernel's paths
run from - between two loads of TIMER1's TIMERAWL, the suite's cycle
ruler: the run called once uncounted, then 16 times masked, the least of
the 16 less the ruler's own two loads (3 cycles). An exception or a trap
is one event between two loads, the second the handler's first
instruction or the instruction its return lands on, the least of 16. A
run's two ends can hide a cycle or two behind the bridge's loads (sixteen
taken branches read 31, not 32), so a figure is a FINDING where it is
half a cycle a unit or more off the published table a count of the core
would otherwise borrow - the Cortex-M4's (DDI 0439B table 3-1), the
nearest, on one half, the datasheet's 3.8.7 on the other - and the
letter's verdict is that every run lies
within one count of the cost it states as measured. ARM's manual for the
Cortex-M33 (100230) gives no instruction timing: these rows are the
core's figures on this chip. On the Cortex-M33 every row runs twice, with
ACTLR.DISFOLD clear, as every program runs, and set, which turns off the
core's "limited dual-issue of common 16-bit instruction pairs" (TRM
1.4.1, 3.4); ACTLR is put back and read back. The letter's code lies
after every other function of the image and the letter runs last in
`z`: run in its place, it lengthened the wake of letter `j` after it by 2
to 3 us on both halves.

| Cortex-M33, cycles a unit | DISFOLD clear | set | Cortex-M4 table |
|---|---|---|---|
| an ALU instruction, 16-bit or 32-bit | 1 | 1 | 1 |
| a load from SRAM, byte or word, 32 in a run | 1 | 1 | 2 |
| a load and an instruction that does not read it | 2 | 2 | 3 |
| a load and an instruction that reads it | 3 | 3 | 3 |
| a literal load, 32 in a run; with an add.w | 1.21; 2.12 | the same | 2; 3 |
| a literal load and a load through it | 3.5 | 3.5 | 4 |
| a store to SRAM | 1 | 1 | 1 |
| a taken branch (beq, b) onto a 16-bit instruction on a word | 2 | 2 | 3 |
| a taken beq.w onto a 32-bit instruction off its word | 3 | 3 | 3 |
| a not-taken bne after a 32-bit add.w (the pair) | 2 | 2 | 2 |
| a not-taken bne after a 16-bit cmp (the pair) | 1 | 2 | 2 |
| a not-taken bne, 32 in a run | 0.5 | 1 | 1 |
| an IT and its one instruction (the pair) | 1.06 | 2 | 2 |
| a BL; a BL and its BX LR | 2; 4 | the same | 4; 6 |
| a BL, a push of two and a pop of two with the PC | 9 | 9 | 13 |
| a push or a pop of two; of eight | 2; 8 | the same | 3; 9 |
| an MRS and an MSR of PRIMASK (the pair) | 2 | 2 | 4 |
| a DSB; an ISB; a CPSID | 2; 2; 1 | the same | 4; 4; 1 |
| a load of SIO's CPUID and an add.w | 2 | 2 | 3 |
| the request's store to ICSR, DSB and ISB | 5.56 | 5.56 | 9 |
| two 16-bit ALU instructions; two 16-bit byte loads | 2; 2 | the same | 2; 4 |
| a queue visit in line: ldrb, ldrb, cmp, bne not taken | 4 | 5 | 6 |
| the same 32-bit, its queue's address a literal | 6 | 7 | 8 |
| a queue visit through a call to `empty()` out of line | 12.5 | 14.5 | 18 |
| an SVC to its handler's first instruction | 13 | 13 | 13 |
| a BX LR of EXC_RETURN, and the return to thread mode | 10 | 10 | 14 |
| a pop of two with the PC loading EXC_RETURN, and the return | 12 | 12 | 18 |
| a BX LR into a pending PendSV (the tail-chain) | 7 | 7 | 8 |
| a pop of two with the PC into a pending PendSV | 9 | 9 | 12 |
| PendSV pended under PRIMASK: the MSR that lifts it, to the handler | 12 | 12 | 15 |
| PendSV requested: the store to ICSR, DSB, ISB, to the handler | 14 | 14 | 21 |
| PendSV's BX LR back onto the instruction after that ISB, and eight adds | 21 | 21 | 22 |

What the rows say of the Cortex-M33:

- A load costs one cycle, and a second only when the next instruction
  reads what it loaded, as an operand or an address; a store's data is
  read late and waits for nothing. A literal load reads the flash through
  the XIP cache, as the fetch does, and costs 1.21 cycles in a run of 32
  and about 1 beside other instructions. The Cortex-M4's table charges 2
  for every load.
- A taken branch, a BL and a BX cost 2, one more onto a 32-bit instruction
  off its word; PUSH and POP of N registers N, a POP into the PC N + 3;
  MRS and MSR 1 each; DSB and ISB 2, the DSB after a store to the System
  Control Space about 2.5. Every one of these is under the Cortex-M4's
  figure.
- THE DUAL ISSUE IS A FOLD. With DISFOLD clear, a not-taken conditional
  branch after a 16-bit instruction costs nothing, and so does an IT;
  two 16-bit ALU instructions, or two 16-bit loads, never issue
  together. With DISFOLD set each of those costs its cycle, and nothing
  else in the table moves. A queue visit as the kernel compiles it in
  line pays its cmp and bne one cycle together.
- THE QUEUE VISIT IN LINE SAVES 6.5 CYCLES A VISIT, read directly: 12.5
  through a call to `empty()` against 6 for the 32-bit in-line visit
  `activate()` makes.
  The measured model counts 13 against 6, 7; the Cortex-M4's table 18
  against 8, 10. Built with the visit out of line and in line, the
  preemptive suite's road 1, scan, post under the mask, road-2 post and
  depth 2 differ by 13 cycles over two visits, 29 over four, 22 over
  three, 31 over four and 5 over one: 5 to 7.75 a visit, where the
  measured model counts 17, 33, 25, 35 and 7, two to four cycles over on
  each path, and the Cortex-M4's table 22, 46, 34, 44 and 8, half again
  the measurement.
- THE EXCEPTIONS: an SVC reaches its handler's first instruction in 13
  cycles, its own and the entry's 12, the Cortex-M4's figure. The return
  is faster: 10 from the BX LR that loads EXC_RETURN to the thread's next
  instruction (the table's 2 and 12 make 14), N + 10 from a POP of N with
  the PC; the tail-chain 7 from the BX, N + 7 from such a POP. PendSV
  pended under PRIMASK is taken behind one more instruction after the MSR
  that lifts the mask, 12 cycles from the MSR to the handler; requested by
  the store to ICSR, its DSB and its ISB, it is taken with nothing behind
  the ISB, 14 cycles from the store, and the return onto the instruction
  after that ISB takes 3 cycles longer than any other return measured.

| Hazard3, cycles a unit | measured | 3.8.7 |
|---|---|---|
| an ALU instruction, 32-bit or 16-bit | 1 | 1 |
| a load from SRAM, byte or word, 32 in a run | 1 | 1 |
| a load and an instruction that reads it: an addi, a load's address, a branch | 3 | 3 |
| a load and a store of its value (the pair) | 2 | 2 |
| a store to SRAM | 1 | 1 |
| a taken branch or a `j` onto a 32-bit instruction on its word | 1.93 | 2 |
| the same off its word | 2.87 | 3 |
| a not-taken branch | 1 | 1 |
| a `jal` and its `c.jr ra`, the return onto a word; off a word | 3.93; 4.93 | 4; 5 |
| cm.push or cm.pop of three; of thirteen | 4; 14 | 4; 14 |
| a `jal`, a cm.push and a cm.popret of one; of three; of thirteen | 7.93; 10.93; 30.93 | 8; 11; 31 |
| a csrrci of mstatus; a csrsi and a csrci (the pair) | 1; 2 | 1; 2 |
| a load from the SIO, a store to it, each with an addi | 2 | 2 |
| a queue visit in line: lbu, lbu, bne not taken | 4 | 4 |
| a queue visit through a call to `empty()` out of line | 11 | 11 |
| an `ecall` to its handler's first instruction, through the table's jump | 5 | 5 |
| an `mret` to the code it points at | 3 | 2 |
| an `mret` into a pending software interrupt, through the jump (the entry taken at `ecall`'s 3) | 6 | 7 |
| the `csrsi` that finds it pending, to its handler through the jump (the same) | 5 | 6 |
| the store that raises it with MIE set, to its handler through the jump (the same) | 7 | 6 |

What the rows say of Hazard3: 3.8.7 holds, its footnotes included - a
load stalls the next instruction only when that instruction reads it in
its second stage (an addi, a load's address, a branch), not when it
stores it; a cm.popret of N costs N + 2, 4 for one; and every transfer
onto a 32-bit instruction off its word pays one cycle. One figure parts
from the table: `mret` costs 3, not 2. The section gives no interrupt
entry; measured with the table's jump, the software interrupt reaches its
handler 6 cycles after the `mret` it lands behind, 5 after the `csrsi`
that finds it pending and 7 after the store that raises it with MIE set,
two instructions retiring behind that store (letter k of the preemptive
suite) - an entry of 1 behind an `mret` and 2 behind the others, where
`ecall`'s own is 3.

**The preemptive kernel** - the reference suite is `test_rp2350_rubato`,
one source and two images on the WeAct RP2350B at 150 MHz: 59 verdicts
in `z` on the Cortex-M33 and 60 on Hazard3, ten runs of `z` alike on
each half. A Rubato pack of five objects on `Rp2350RubatoPlatform<0>`,
the Console the lowest and the letters' runner, TIMER1's alarm 0 the
interrupt the road-1 letters land, nothing wired and nothing lit; the
scheduler's entry is the pair body in the whole image, and one letter
launches core 1 into a Rubato pack of its own (below). The
ruler is TIMER1 itself with its source on clk_sys, two back-to-back
reads 3 cycles apart on both halves; beside it the core's own counter -
on the Cortex-M33 the kernel ticker's composed `cycles()`, two reads 41
cycles apart, agreeing with TIMER1 over 1 to 1.06 ms within 0 to 1
cycle on every clean span, and on Hazard3 `mcycle`, two reads 1 cycle
apart, agreeing within 0 cycles on every clean span. A span with an
XIP refill inside a pair of reads is reported and not judged - the
first of every run has one, its second pair's code cold, and reads 0
to 120 cycles off -, so spans are taken until four are clean (five in
every run). That is the suite's method throughout: an event, a run or a
span that a kernel tick, an XIP refill or the console line's handler
lengthened is told apart by counters read around it (the tick count,
the XIP cache's miss counter, the console vector's entries) and
reported, and each bound judges the ones nothing lengthened (road 1's
bound allows one kernel tick, and judges the events no refill and no
console line lengthened). The miss counter is the difference of the
cache's two counters, which saturate at 32 bits (4.4.5, tables 452 and
453), so every letter starts by clearing them: left to saturate - some
225 s of letter `l` run back to back did it - the difference stops
moving and a refill goes unseen.

On both halves road 1 preempts at any depth: an interrupt's post to
High inside Low's dispatch runs High to its end before Low resumes,
every object reading its own index as the ceiling and the UART's
interrupt the loop's - 255 at its base, or 0 while the base turn walks
the time events (kernel/rubato.hpp holds it there so that a timer's
fire only queues; no dispatch of Top, whose index is also 0, can be
open then, nothing posting to Top between letters); road 2 nests High inside Mid inside Low, each
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
| road 1: an interrupt's last statement to the preempting dispatch's first, min / median / max over 100 events | 106 / 106 / 135 | 179 / 179 / 407 |
| road 1: the path's count by the release listing, and the bound | 102, 177 | 175, 284 |
| road 1 at depth 2, over a preempting dispatch | 96 | 170 |
| `activate()` over four empty queues | 65 | 68 |
| a post that asks nothing | 60 | 53 |
| a road-2 post, the receiver's empty dispatch inside (the scheduling's share) | 303 (243) | 323 (270) |
| one `arm()`, its masked window inside | 61 | 45 to 51 |
| a post under the mask: the section's end to the receiver's first | 117 | 160 |
| stack high-water of the suite's deepest chain (bytes), the deepest sample in a dispatch / in the interrupt | 652 (632 / 488) | 904 to 992 (928 / 736) |
| core 1, road 1 from its doorbell's handler at its base, a send from core 0 | 110 | 185 |
| core 1, road 1 from its alarm over a dispatch | 102 | 175 |
| core 1's paths' counts, and the bounds | 107 and 100; 129 and 120 | 181 and 171; 218 and 206 |
| core 1's stack high-water (bytes), the deepest sample in a dispatch / in an interrupt | 236 (216 / 88) | 348 (352 / 192) |

The pair body costs core 0 three cycles a scheduler entry on the
Cortex-M33 by the model and three measured (road 1, depth 2 and the
post under the mask each three longer than in an image with the single
body bound: 103, 93 and 114 there), two on Hazard3 by the model and two
to four measured, where the paths' other one-cycle differences are
branches that land on a 32-bit instruction off its 4-byte alignment in
one image and not in the other. A road-2 post pays the request's core
check too: four cycles on the Cortex-M33 (the SIO's address, CPUID, the
branch), seven in all with the body, measured eight longer (295 with the
single body); one on Hazard3, whose request reads the hart id anyway,
three in all with the body, measured four longer (319 with the single
body). The quiet post, the scan and `arm()` ask nothing of the body and
differ by a cycle at most between the two images, by layout. Road 1's
events over the fastest are the ones a tick, a refill or the console
line landed in, and the bound judges the others: on Hazard3 one or two
of the hundred events of a run were lengthened so in six runs of ten
(246 to 407 cycles), none in the four others. Every count is drawn with
the figures `test_rp2350_platform`'s letter `i` measures ("The cost
models", above). Against the measurement the counts lie within a cycle
on the paths that start and end in thread mode - the scan, the quiet
post, the road-2 post and `arm()` - but for Hazard3's road-2 post,
counted 325 and measured 323, and 2 to 4 cycles under it on those that start in an interrupt
handler or under the mask and end at a dispatch's first statement: road
1, depth 2, the post under the mask and core 1's two paths, on both
halves. That residue is not placed: every primitive those paths add over
the others - the tail-chain from a handler's POP of the PC, PendSV taken
at the MSR, the software interrupt behind an `mret` and behind a
`csrsi` - measures at its model in `test_rp2350_platform`'s letter `i`.
Counted with the
Cortex-M4's table instead, the Cortex-M33's paths come out 24 cycles
over the measurement on road 1 and 15 to 79 over on the others; with the
core's dual issue turned off (ACTLR.DISFOLD set in a build of this suite
kept off the tree) road 1 measured 111, the scan 69, the quiet post 62,
the road-2 post 317, depth 2 100 and the post under the mask 124, where
the measured model without the fold counts 105, 68, 60, 313, 95 and 119:
the fold is 2 to 14 cycles of each path, and the Cortex-M4's counts
(130, 80, 80, 382, 118, 144) are not reached with it off either. On
Hazard3 the measured model is 3.8.7 with `mret` at 3 and the interrupt's
entries measured, and its counts lie within a cycle or two of 3.8.7's.
Each queue's emptiness is its two cursors compared in line, no call. A
post under the mask
lengthened by a kernel tick or an XIP refill reached 278 cycles on the
Cortex-M33 and 394 on Hazard3, and is reported, not judged. Core 0's
high-water marks are those of core 1's letter, whose Low, High and
Top nest three deep over the Console; on Hazard3 the paint's edge
stands 24 bytes above the deepest sample in seven runs of ten (904
against 928), the lowest words of that `activate()`'s frame being ones
no path below it writes, and the letter allows that frame's 44 such
bytes; in one run it stood at the sample and in two 64 bytes below it
(992). All are inside the bound below the deepest sample, 312 and 400
bytes. The two halves' rows part
where their mechanisms do: on the Cortex-M33 the hardware stacks the
interrupted code's registers, on Hazard3 the crt's trampoline and the
scheduler's entry each save sixteen in software ("What the silicon
does"), and the stack's levels carry those frames. The Cortex-M33's
pair body is sixteen instructions and 44 bytes with its table, the
single body's thirteen and 32, both without the DSB that is the
Cortex-M4's alone ([../cortexm/README.md](../cortexm/README.md)); a
build of the single body that kept it measured road 1 two cycles
longer, the barrier's cost on this core. Hazard3's pair body is
nineteen instructions in the listing, the single body's seventeen.

- **The FPU across a preemption** (letter j, the Cortex-M33 alone):
  FPCCR reads 0xC000001C (0xC0000004 in the first run after a flash),
  ASPEN 1 and LSPEN 1 either way, where the datasheet's
  register table and the SVD give both a reset value of 0; Low's float
  series, preempted at a mark inside its loop by High's own series, and
  in a second run by an interrupt handler that computes a float too,
  ends with every value bit-exact against the same functions run without
  preemption.
- **The software interrupt's own facts** (letter k, Hazard3 alone):
  after the raw store that raises the hart's flag, two of the eight
  stores behind it retire before the trap - the latency the request
  waits out - and behind the platform's request none; each raise serves
  Top exactly once (20 raises); the caller-saved registers, loaded with
  patterns, are found again across an entry and across a nested one,
  10 of 10 each; inside a dispatch the software interrupt started,
  the flag is low, mstatus.MIE set and mscratch moved by one.
- **A Rubato on each core** (letter l, both halves): core 1 launched into
  a pack of three of its own on `Rp2350RubatoPlatform<1>`, its request
  set up by its own `scheduler_init()`, every preemption of both cores
  through the one pair body. Core 0's Low sends to core 1's Low1 (road
  1 there from the doorbell's handler, at core 1's base), and both cores
  then run the same shape at once - a spin, the core's own alarm posting
  to its High, which posts to its Top (road 2): ten runs alike on both
  halves, each core's trace with its own ceilings, each core's
  preemption landing inside the other core's preempted dispatch, both
  settled at their bases after. Run alone on Hazard3 420 times back to
  back, 286 s with the cache's counters cleared at each run's start,
  the letter settled in all 4620 of its runs and warm-ups, every verdict
  green in every run. Inside Low1's dispatch a post to core
  0's High is refused and counted by High's queue, runs nothing, and
  leaves core 1's request down - PendSV not pending, the hart's flag low
  - in every run. Core 1's two road-1 latencies, the table's rows, are
  read with core 0 asleep. In a diagnostic build of the letter (not
  kept) with core 0 spinning on the ruler and on the flash meanwhile,
  they read 113 to 117 and 110 cycles against that build's 111 and 105
  with core 0 asleep on the Cortex-M33, and 196 and 189 against 185 and
  175 on Hazard3 - the two cores' fetches and bus accesses meet, by 2 to
  6 cycles on one half and 11 to 14 on the other; which port they meet
  at is not separated. Core 1's FPCCR reads 0xC0000004,
  ASPEN 1 and LSPEN 1, as core 0's does in its first run after a flash.
  Sent before core 1 has left the scheduler entry of the previous
  crossing, a doorbell lands in that entry's exit and nests a new entry
  over it, as the Hazard3 file's re-entrancy section allows: back to
  back, the crossings took core 1's stack 80 bytes deeper at every one
  that landed so (1516 bytes of high-water in one run against 348
  settled), each level unwinding when the one above it ended - the
  letter waits 20 us after each crossing's end, many times the rest of
  core 1's way back to its base.

**The drivers under the preemptive kernel** - the suite is
`test_rp2350_rubato_link`, one source and two images, two keys against
the WeAct RP2040's unchanged `spi_peer` (`z`, 147 verdicts) and
`uart_peer` (`y`, 55 verdicts), three runs of each key on each half alike,
every verdict green. It runs the real SPI and UART stacks inside a Rubato
pack of seven objects on core 0: `util/spi_bus.hpp`'s arbiter over the
engined `SpiHost<1>` ([spi.md](spi.md); half-word engines on DMA channels
6 and 7, line 1), `util/line_reader.hpp`'s `LineReader` over `Uart<1>`
and its interrupt receiver ([uart.md](uart.md)), a borrower of the
reader's lines above it, and on top an object that TIMER1's alarm 0 posts
to every 97, 193 or 389 us - the timer on clk_sys and re-armed in its
handler at the last match plus the period, so the period does not drift -
whose dispatch spins 300 or 3000 cycles: at 97 us x 3000, 21 % of the
core. Every letter runs once with the alarm silent as its control and six
times under that pressure, and every pass judges the pressure itself, one
dispatch per period over the pass, none dropped. Every letter that drives
a link pad reads the partner's rail first (`pair_power.hpp`). The
target-free half of the suite is the link suites' shared body
(`avrxt/src/apps/rubato_link.hpp`), what the two Raspberry Pi chips share
over the PL022 and the PL011 is `rp2040/src/apps/rp_rubato_link.hpp`; the
eighth pass fed by core 1 is the RP2040's suite's alone. What it
established, on both halves:

- **The polled SPI loop under preemption loses nothing.** The loop runs
  inside the arbiter's dispatch, so a dispatch above the arbiter holds it
  off for its whole length - the alarm landed there 39 to 192 times a
  pass in letter `p` on the Cortex-M33 and 40 to 190 on Hazard3. The
  FIFOs are the write-ahead on every path (no more than 8 frames in
  flight, `brio/pl022/spi.hpp`), so a held-off loop or pump leaves a gap
  on the bus and never a lost frame: the polled loop in 8-bit frames (9)
  and in 16-bit frames from an odd address (32, off the engines), the
  pump the same two ways, the engines (32 byte frames) and the engines
  under a polled request (32 half-words), all byte-exact at both ends at
  every even divisor from 256 to 16 (9.4 MHz, the PIO client's ceiling as
  [spi.md](spi.md) derives it against the RP2040's 125 MHz), under every
  pressure as in the control, the block's overrun flag clear at every
  reply. Past that ceiling, /14 is exact on all six paths in every pass
  of both halves and /12 on none to three, varying, the control's as the
  pressures'.
- **The matrix and the arbiter.** The four modes in both widths, the
  client MSb and LSb first (the LSb rows an exact two-way reversal
  against this MSb-only host) on the engines at clk_peri / 32, byte-exact
  in every pass, the data windows at 939 to 1003 per mille of the
  control's rate under pressure on the Cortex-M33 and 925 to 1005 on
  Hazard3, the cache's misses over a pass as the control's (1750 to 1970 on
  the Cortex-M33, 2152 to 2415 on Hazard3, the counters cleared at each
  pass's start); three requesters of three priorities on one select
  window answered every one `spi_ok`, to its requester, in its order,
  with no timeout, no rejection and the peer counting every frame exact.
- **The UART receive side under preemption, at 115200.** Numbered lines
  echoed by the RP2040's `uart_peer`, assembled by `LineReader` from the
  interrupt receiver's ring and lent to the borrower: 200 a leg, none
  lost, out of order or corrupt, every port counter zero (ring overruns,
  OE, FE, PE, BE, the ring's skips), 949 to 953 per mille of the wire (the writer
  paces each line to its own last stop bit). The peer has one rate and no
  report, so the key is reduced to this rate and to what this end judges
  (Not covered yet).
- **The loan under preemption.** With the borrower holding each line
  sixteen character times (one receive level of the FIFO) and the lines
  streamed five ahead, the alarm landed inside the borrower's dispatch
  709 to 2934 times a pass and the link's vector inside every one of its 200
  dispatches a leg; every line arrived exact, each lent alone, the two
  buffers strictly alternating, the leg at 995 to 996 per mille of the wire.
- **Dark** (letter `i`): every object idle, TIMER1's alarm 0 disarmed with
  its flag low and its line quiet, no time event armed, SPI1 and UART1
  given back with their pads at no function, the mode line released,
  GP25 not an output, the ceiling the Console's.

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
- The SPI and UART stacks under preemption with the UART above 115200,
  and a blast judged by the far end: `test_rp2350_rubato_link`'s key `y`
  runs at 115200 alone and judges at this end, the partner's `uart_peer`
  being a fixed-role echo at that rate with no report
  (rp2040/src/apps/rp_uart_peer.hpp). A peer speaking the usart_link
  protocol on the two Raspberry Pi chips would run the other families'
  full key, its 460800 and 921600 legs and the blast into its sink.
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
