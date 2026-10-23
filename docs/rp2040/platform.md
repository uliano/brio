# Platform (RP2040)

This document is for a programmer putting a brio program on an RP2040.
It describes what the kernel stands on here, the platform, and the
chip's start-up around it. A brio kernel asks the chip for a handful
of things, stated by the kernel's `Platform` concept (a concept is a
compiler-checked set of requirements on a type;
[design/kernel.md, "Platform: what the machine provides"](../design/kernel.md#11-platform-what-the-machine-provides-kernelplatformhpp)):
a way to mask interrupts for a critical section, a way to sleep until
the next interrupt without losing one that arrives at the wrong moment,
a source of time, a breakpoint for a panic, and a record a panic leaves
for the next boot. `Rp2040Platform<core, TB>` is this family's answer.
The chip has two cores, and the first template argument says which one
the platform is: `Rp2040Platform<>` is core 0's, the one a single-core
program names, and a program that runs a second kernel on core 1 names
`Rp2040Platform<1>` for it ([multicore.md](multicore.md)). `TB` is the
timebase, by default that core's SysTick ticker at 1000 Hz, which keeps
counting through every sleep short of the chip's DORMANT state.

The smallest use, from "How to use it" below - the lines of a kernel
program's start-up that concern the platform:

```cpp
using P = brio::Rp2040Platform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

extern "C" void isr_systick() { brio::Ticker::tick(); }

int main() {
    SysClock::init();
    brio::Ticker::init(clock);
    brio::enable_interrupts();
    brio::Tenuto<P, Console, SerialLines, Lamp>::run();
}
```

Line by line: `P` names the platform of core 0 with its default
timebase; `SysClock` names the clock tree the program runs on (the
crystal through the PLL to 125 MHz, [clock.md](clock.md)) and `clock`
is an empty tag object of that type, which the drivers' `init()` take
to learn the rate; `isr_systick` is the SysTick exception's handler,
under the pico-sdk's name for it, and its one statement advances kernel
time; `main()` starts the clock tree, starts the ticker from the
clock's rate, enables interrupts and hands the core to `Tenuto`, the
kernel's scheduler, with the program's active objects in priority
order, the list these pages call the pack (an active object is a
component of a brio program with its own event queue, served one event
at a time by the kernel: a handler runs to its end before the kernel
serves another event on that core, whatever its priority,
[docs/learn/introduction.md](../learn/introduction.md); `Console`,
`SerialLines` and `Lamp` are the console program's). `run()` never
returns.

What decides correctness even there. The clock comes first:
`Ticker::init()` computes the SysTick reload from the rate the clock
type claims and does not read the hardware, so a tick is a millisecond
only once clk_sys runs at that rate - a ticker started before
`SysClock::init()` ticks some twenty times too slowly on the ring
oscillator's 6.5 MHz until the tree reaches 125 MHz, and at the
millisecond from then on, and a `SysClock::init()` that answered false
has left the tree short of the rate (on clk_ref after a crystal or PLL
failure, [clock.md](clock.md)), with every rate computed from the type
wrong (the program looks at that answer). `Ticker::init()` itself
answers false, starting nothing, only when the reload is zero (a clock
slower than the tick rate) or does not fit SysTick's 24 bits. The vector
is the program's to bind: the crt gives every handler name but
`isr_hardfault` (a weak spin of its own) a weak alias to
`Default_Handler`, a spin, so a program that forgets `isr_systick` stops
in that spin at its first tick. The peripheral handler names - not
`isr_systick` or `isr_hardfault` - are macros of the vendored device
header that every header of this stratum includes (`isr_uart0` is
`isr_irq20`), and the crt includes the same header, so a program that
includes a brio header spells the same slot the crt declares; a handler
defined without that header names a symbol no slot holds and leaves the
vector on the spin ("Bench findings", the handler names). Interrupts are
enabled before `run()`, since the kernel sleeps between events and the
tick is what brings it back. And the critical section is per core: it
masks this core's interrupts and nothing on the other one, so anything
two kernels share goes through the bridge of
[multicore.md](multicore.md), never through a `CriticalSection`. A
`panic()` writes its record and then executes a breakpoint: under a
debugger the core halts there, and with none the breakpoint escalates to
HardFault, whose handler the crt gives a spin of its own - a program
that wants the record read at the next boot binds `isr_hardfault` to
[reset.md](reset.md)'s `fault_reset<P>()`, which keeps a record that
stands and reboots the chip, and reads it with `take_panic_record<P>()`.
One more fact meets every program at its first driver: every peripheral
the reset controller governs is held in reset at power-up, and each
driver's `init()` releases its own block - except the DMA, the PIO and
the PWM, whose release is a verb of its own the program calls once
before the block's first user (`Dma::init()`, `Pio<n>::reset()`,
`Pwm::reset()`).

The rest of this page, in order: the documents of record; what the
silicon does (the two cores, the boot chain, the reset controller, the
SRAM banks); the types and verbs; how to use it; the bench findings,
the idle path measured to the cycle on both cores; "A handler in
SRAM", an opt-in placement of the drivers' hot paths and the vectors,
with its cost table; and what is not covered.

## Documents of record

The RP2040 datasheet (build 3184e62), 2.3
(processor subsystem: SIO, the two NVICs, table 2.3.2's interrupt
lines), 2.6 (memory: the SRAM banks), 2.7 and 2.8 (boot sequence,
bootrom, the second stage), 2.14 (subsystem resets), 2.20 (SYSINFO);
Appendix B. The drivers: `brio/rp2040/platform.hpp`, `nvic.hpp`,
`ticker.hpp`, `delay.hpp`, `resets.hpp`, `sysinfo.hpp` over
`brio/cortexm/`; the crt `rp2040/src/glue/startup_rp2040.cpp`, the
second stage `rp2040/src/glue/boot2_*.S`, the linker script
`rp2040/ld/rp2040_2m.ld`; `brio/util/placement.hpp` for the placement.
The preemptive kernel's platform, `Rp2040RubatoPlatform`, is in the same
`platform.hpp`, over the core stratum's `brio/cortexm/schedule.hpp`
(ARM's PendSV and SVCall, [../cortexm/README.md](../cortexm/README.md))
and this stratum's `brio/rp2040/schedule.hpp` (the one PendSV entry two
kernels share).
The reference suite: `test_rp2040_platform`
(the boot state, the ticker against the system timer, `delay_us`, the
reset controller, the SIO, the atomic aliases, the WFI idle, the idle
path against an interrupt's edge placed to the cycle, the kernel's turn
per interrupt, the breadcrumb, five real resets); a suite's letter is
one command key of its console and the group of verdicts it runs, and
`z`, the all-key, runs the suite's set; the preemptive kernel's,
`test_rp2040_rubato` ("Bench findings"). The family fixture
`test/family_rp2040/rubato.cpp` instantiates both platforms, a pack on
each core and both bindings of PendSV, and
`neg/rubato_on_the_tenuto_platform.cpp` refuses the preemptive kernel
on the Tenuto platform. The failing half of the platform - the
causes, the watchdog, the timer - has its own documents:
[reset.md](reset.md), [watchdog.md](watchdog.md), [timer.md](timer.md).

## What the silicon does

Two Cortex-M0+ cores share one bus fabric, one flash and one SRAM;
each has its own NVIC, its own SysTick and its own PRIMASK. Every one
of the 32 interrupt lines reaches both NVICs and each core enables
what it wants; a line enabled on both interrupts both cores. Out of
reset only core 0 runs code; core 1 sits in the bootrom's launch
protocol, waiting on the inter-core FIFO (2.8.2).

The boot chain: the bootrom (in ROM, on the ring oscillator at about
6.5 MHz) reads the first 256 bytes of the flash into SRAM, checks
their CRC32 and runs them; that second stage programs the XIP serial
interface for the flash chip in use and then vectors through the
table it expects at flash offset 0x100 - it writes VTOR with that
address and loads SP and PC from the table's first two words. So an
image is the stage, then the vector table, then the load image of
.data - which carries the code that runs from SRAM, the flash engine's
window and whatever a program places ("A handler in SRAM", below) -
then the code, and nothing in brio writes VTOR again. Every peripheral the reset controller
governs (2.14: the UARTs, SPIs, I2Cs, the timer, the PWM, the ADC,
both PLLs, both PIOs, the DMA, the IO and pad banks, USB, SYSINFO) is
held in reset at power-up: a driver's first act is releasing its
block and waiting for RESET_DONE, the way the other families open a
clock gate.

The SRAM is 264 KB in six banks: four of 64 KB striped word by word
over 0x2000_0000..0x2003_FFFF, so consecutive words fall in different
banks and two masters seldom contend, and two unstriped banks of 4 KB
at 0x2004_0000 and 0x2004_1000 - where the two cores' stacks go, one
each, so a core's stack traffic never contends with the other core's
or the DMA's. Kernel time is the core's SysTick on clk_sys, which
nothing short of the DORMANT state stops.

## Types and verbs

- `Rp2040Platform<core = 0, TB = CoreTicker<core>>` - the kernel's
  Platform, one type per core ([multicore.md](multicore.md)); `Rp2040Platform<>`
  is core 0's on `Ticker`:
  `CriticalSection` = the PRIMASK guard of `cortexm/nvic.hpp`, PER CORE
  (it excludes this core's handlers and nothing of the other core);
  `idle()` = DSB, WFI, unmask - the core's clock stops, everything
  else runs, and with both cores and the DMA asleep the clock enables
  switch to their SLEEP_EN set (all enabled at reset; the SLEEP state
  and the `sleep_hook` a dormant site installs in place of the WFI
  are [sleep.md](sleep.md)'s). The WFI path keeps the kernel's idle
  contract - no wake lost wherever the interrupt lands, one return per
  interrupt - measured below; the dormant hook's wake list is the
  DORMANT state's own (a GPIO event, the RTC), so a line that is not
  one of those waits for one; `now()`,
  `ticks_per_second` 1000, `atomic_width` 4, `panic_record()` in
  `.noinit`, `break_here()` = BKPT; `core_id()` = SIO's CPUID.
- `Rp2040RubatoPlatform<core = 0, TB = CoreTicker<core>>` - the platform
  of the preemptive kernel, `Rubato`
  ([design/kernel.md](../design/kernel.md), "Rubato"), one type per core:
  `Rp2040Platform<core, TB>` whole plus the three members of a
  preemptive platform - `preemptive`, `ceiling` (the index of the object
  being dispatched, 255 at the loop's base; a static of the core's own
  type, so each core has its own) and `pend_schedule()`, which is
  `cortexm/schedule.hpp`'s request with one test first: PendSV pended in
  the ICSR of the core that asks, at the lowest of that core's four
  levels (the byte 0xC0), below every NVIC line at its reset priority 0,
  and only when the asking core is the platform's own. A post from the
  other core is a mispost the queue refuses and counts; without the test
  `post()` would still ask, pend the posting core's PendSV and, under the
  single body below, run this core's kernel on the other core. The test
  is SIO's CPUID read and compared inside the request's one opaque call,
  five cycles on the asking path alone (by the release listing). PendSV
  returns into the
  scheduler in thread mode on that core's stack and the scheduler leaves
  through SVC, every frame basic (no FPU). The critical section stays
  PRIMASK, per core, and a post made inside one serves its receiver when
  the section ends; a send from the other core lands through the
  doorbell's handler as a local post, road 1 on the receiving core
  ([multicore.md](multicore.md)).
- `scheduler_exception_pair<K0, K1>()` (`rp2040/schedule.hpp`) - the
  body `isr_pendsv` takes when each core runs a Rubato: the core stratum's
  body with the core's identity read first (SIO's CPUID, one cycle on the
  IO port) and used to pick `scheduler_thread<K0>` or
  `scheduler_thread<K1>` from a two-word table, the same path on both
  cores; and no DSB, as the single body has none on this core (the
  barrier is the Cortex-M4's alone, for its erratum 838869,
  [the core stratum's page](../cortexm/README.md)). By the release
  listing eighteen instructions and 44 bytes with the table, against the
  single body's thirteen and 32: two cycles more at zero wait states.
  `K0` must be a Rubato on `Rp2040RubatoPlatform<0>` and `K1` one on
  `Rp2040RubatoPlatform<1>`, refused at compile time otherwise.
- `Ticker` = `CoreTicker<0>`, the core stratum's `BasicTicker<1000>`
  on core 0's SysTick (`CoreTicker<1>` is core 1's), `SysTickCounter` for a
  program whose timebase is elsewhere, `delay_us(clock, us)` - the
  core stratum's, unchanged.
- `Nvic`, `InterruptGuard`, `enable_interrupts()` - the core stratum's,
  acting on the NVIC and PRIMASK of the calling core.
- `ResetBlock` (the 25 blocks as masks) and `Resets`: `release(blocks)`
  clears the bits and waits, bounded, for RESET_DONE; `hold`, `cycle`
  (hold then release: a block from its reset state), `released`,
  `held`, `watchdog_resets` (WDSEL).
- `ChipId::read()` - manufacturer, part and silicon revision off
  SYSINFO.CHIP_ID (the block released first); `gitref()`.
- The handler names an app binds: `isr_systick`, `isr_nmi`,
  `isr_hardfault`, `isr_svcall`, `isr_pendsv`, and `isr_<block>` for
  the 32 lines (`isr_uart0`, `isr_timer_0`, `isr_dma_0`,
  `isr_sio_proc0`, ...): the pico-sdk's spelling, every Pico user's.
- `hw_set` / `hw_clear` / `hw_xor` / `hw_write_masked` on any peripheral
  register: the atomic aliases of 2.1.2, one bus write and no
  read-modify-write (`device.hpp`).

## How to use it

A kernel program: the clock first, the ticker, interrupts on, the pack.

```cpp
using P = brio::Rp2040Platform<>;
using SysClock = brio::Clock<brio::ClockSource::pll, 125'000'000>;
constexpr SysClock clock;

extern "C" void isr_systick() { brio::Ticker::tick(); }

int main() {
    SysClock::init();
    brio::Ticker::init(clock);
    brio::enable_interrupts();
    brio::Tenuto<P, Console, SerialLines, Lamp>::run();
}
```

A driver's first act, the reset release:

```cpp
if (!brio::Resets::cycle(brio::ResetBlock::uart0)) { /* the block never came ready */ }
```

The same program under the preemptive kernel, on core 0, with core 1
under Tenuto or not launched: the platform type, the two naked vectors
and one call more. Core 1 never pends PendSV - a Tenuto kernel never
asks - so the core stratum's body for one kernel is the binding:

```cpp
#include "kernel/rubato.hpp"

using P = brio::Rp2040RubatoPlatform<>;
using K = brio::Rubato<P, Sensor, Logger>;   // Sensor preempts Logger

extern "C" void isr_systick() { brio::Ticker::tick(); }
extern "C" [[gnu::naked]] void isr_pendsv() { brio::scheduler_exception<K>(); }
extern "C" [[gnu::naked]] void isr_svcall() { brio::scheduler_return(); }

int main() {
    SysClock::init();
    brio::Ticker::init(clock);
    brio::scheduler_init();         // PendSV at the lowest priority, this core's
    brio::enable_interrupts();
    K::run();
}
```

A Rubato on each core: one vector table serves both cores (core 1 is
launched with core 0's, [multicore.md](multicore.md)), so the one
`isr_pendsv` entry takes the pair body, and each core sets PendSV's
priority in its own registers before it enables its interrupts:

```cpp
#include "rp2040/schedule.hpp"

using P0 = brio::Rp2040RubatoPlatform<0>;
using P1 = brio::Rp2040RubatoPlatform<1>;
using K0 = brio::Rubato<P0, Sensor, Logger>;   // core 0's pack
using K1 = brio::Rubato<P1, Motor, Watch>;     // core 1's pack

extern "C" [[gnu::naked]] void isr_pendsv() { brio::scheduler_exception_pair<K0, K1>(); }
extern "C" [[gnu::naked]] void isr_svcall() { brio::scheduler_return(); }

[[noreturn]] void core1_main() {
    brio::CoreTicker<1>::init(clock);
    brio::scheduler_init();         // core 1's own PendSV priority
    brio::Inboxes<Motor>::enable();
    brio::enable_interrupts();
    K1::run();
}
```

Core 0's `main()` is the single-kernel one above with `K0`, and
`Core1::launch(&core1_main)` before it enables its interrupts.

An open decision: how one table serves two kernels. The pair body is
one of two ways. The other is a vector table per core: core 1 launched
with a table of its own (the launch protocol's VTOR word), identical to
core 0's but for PendSV's entry, which would hold the single body bound
to `K1` - each core's PendSV then the core stratum's thirteen
instructions with nothing asked. What that costs: a second table of 48
words, 192 bytes in the flash or a copy in SRAM aligned as VTOR
requires, every vector a program binds present in both; a crt that
defines it and a launch that takes it (today the crt holds one table and
`Core1::launch` passes core 0's VTOR); a second binding of PendSV a
program must keep apart from the first. What it saves: the pair's two
cycles and its question at every request. The pair body is what is
built and measured ("Bench findings"); which road a two-kernel program
takes is not decided ("Not covered yet").

## Bench findings

- The chip on the WeAct board is B2 silicon: CHIP_ID 0x20002927,
  read over SWD and by `ChipId::read()`; both debug ports answer the
  multidrop select (instance ids 0 and 1), each a Cortex-M0+ r0p1 with
  four breakpoints and two watchpoints.
- The image boots as the chain says: the second stage (the SDK's
  `boot2_w25q080`, CLKDIV 4) runs the Zetta ZD25Q16 flash, VTOR reads
  0x1000_0100 afterwards, SP is the top of the upper unstriped bank,
  and the program counter sits in `main()`'s loop with the pin-check
  wave running.
- THE HANDLER NAMES ARE MACROS, AND THE TRAP IS REAL. The SDK's vector
  slots are the symbols `isr_irq0..31`; `hardware/regs/intctrl.h`
  spells `isr_uart0` as `#define isr_uart0 isr_irq20`. A crt that
  declared its weak `isr_uart0` WITHOUT that header and an app that
  defined its strong `isr_uart0` WITH it produced two different
  symbols, the vector stayed on the weak spin, and the first receive
  interrupt parked the core in `Default_Handler` for ever (found with
  the debugger: PC on the spin, LR 0xFFFFFFF9, UARTRIS.RTRIS set). The
  crt now includes the header, so both sides spell the slot.
- `reset run` over SWD (SYSRESETREQ) restarts core 0 through the
  bootrom and the stage - core 0 alone: SYSRESETREQ is one core's
  ([reset.md](reset.md)); the DHCSR write after it leaves the core
  faulting on BKPT as a probe-less power-on would.
  AND IT RESETS NO PERIPHERAL: a block the previous image brought up and
  the next does not touch runs on under it. Measured on the WeAct
  RP2040: under a console image, UART1 found enabled (UARTCR 0x301, out
  of reset) with GP4 under its function and idling high, left by a serial
  peer flashed before it - the desk partner's receiver saw a driven line
  where a dark board would have left its pad pulled down. Every brio
  driver resets its own block at `init()`; what an image does not use is
  the previous image's until a power-on or a chip reset - which is why
  `brio flash` restarts the power-on state machine through the rescue
  debug port before it programs (cli/bench/flash.py).
- The kernel console runs: three active objects over UART0, the
  SysTick ticker at 1000 Hz, WFI between events, the `LED` command
  toggling GP25 (read back through SIO), the uptime advancing with the
  host's clock.
- The reference suite is green on the WeAct (62 verdicts in the
  all-key, 18 in the reset letter) and, letters `w` and `k` apart, on
  the Pico. What it measured, on the system timer as the ruler:
  - the ticker: 200 SysTick periods at 1000 Hz span 199999 us of the
    crystal's microseconds - the PLL's ratio exact to 5 ppm;
  - `delay_us` is exact when warm, and THE FIRST CALL OF A COLD ROUTINE
    COSTS THE XIP CACHE FILL: a 5 us delay took about 17 us the first
    time and 5 us after, so a program's first delay of a code path is
    long, never short. The suite measures the cold call apart;
  - `Resets::cycle` sees RESET_DONE within 3 us of the release, the
    PWM block among the ones the bootrom leaves in reset;
  - the WFI idle: 200 ticks of an empty loop keep the core awake 4 us
    (the WeAct) to 202 us (the Pico, with the console's traffic) of
    199 ms - 200 `idle()` calls, one per tick (letter `h`, a bare
    `idle()` loop, no kernel);
  - `.noinit` SRAM survives a core reset, a watchdog reset and a
    HardFault's reset - a fact the datasheet does not promise;
  - the breadcrumb: a `panic()` through `ResetReporter` and a
    HardFault through `fault_reset()` each read back at the next boot
    with their code and context; the causes word is a history
    ([reset.md](reset.md)).

- THE IDLE CONTRACT HOLDS ON CORE 0, both halves of it, measured on
  the WeAct at 125 MHz (letters `w` and `k`):
  - NO WAKE IS LOST, wherever the interrupt lands. Letter `w` places an
    edge D cycles after a placement, D walking 400 consecutive cycles,
    then runs the kernel's own shape against it - a masked check,
    `idle()`, until one edge was served, and in a second pass two (the
    sleep entered right after a handler) - with alarm 1 of the system
    timer as a rescue 50 ms out. Two edges: the TICK's own, SysTick
    placed by a reload of D (a VAL write reloads on the next cycle,
    ARMv6-M B3.3), and a PERIPHERAL LINE, PWM slice 0 counting clk_sys
    to its wrap and placed by a counter write; each in the light rung
    (SLEEPDEEP clear) and the standby rung (set, through
    `Rp2040SleepSite`) - every SLEEP_EN gate left open, and again with
    the set PRUNED to the core's (`sleep_clocks_core`: the fabric, the
    memories, SIO, the timer and its tick), the PWM's and the console's
    UART. 4800 tries: none
    took the rescue, none ended a period late, the slowest one-edge try
    4 us. The edge crosses the whole path: the earliest lands 141 to 143
    cycles before the sleep takes it (a fixed lead of bus reads sits
    between the placement and the loop, longer than the path from the
    loop's masked check to the WFI, about 25 cycles in the listing), and
    from D = 150 (the tick) and D = 142 (the line) on the edge finds the
    core asleep.
  - WHAT A WAKE COSTS, in clk_sys cycles: from an edge that finds the
    core asleep to the handler's first statement 23 (SysTick) and 26
    (the line), against the core's 15-cycle worst-case entry of a
    zero-wait system (2.4.3.6.1); to the caller's loop, the handler
    whole and the WFI's return, 94 and 93. The standby rung measures
    the same: with the gates open, SLEEPDEEP changes nothing the core
    does here - the SysTick edge, which 2.4.2.8.4's wake-up interrupt
    controller does not take, ends a standby WFI as it ends a light one;
    and with the gates pruned the same again, to the cycle (ten runs of
    the letter).
  - ONE TURN PER INTERRUPT. A Tenuto pack of three quiet AOs, two of
    them with a periodic time event, turned as `run()` turns it with the
    tick the only interrupt: 100 turns over 100 ticks (letter `k`; a
    turn more or fewer allowed at the window's ends). This core's WFI
    has no event latch to leave behind - the WFE's event register is a
    different instruction's - so nothing makes a turn find nothing.
  - WHAT A TURN COSTS, on SysTick's counter: `idle()` with a line
    already pending - the WFI falls through, the unmask takes the line's
    handler - 63 cycles; a quiet turn entered masked whose `idle()`
    finds one 196, so the loop's own share - the time events'
    `process()`, the empty `step()`, the masked check - is 133 cycles.
  - The first try of a sweep is discarded: the code runs from flash,
    and the first pass is the XIP cache's fill (1786 cycles from the
    edge to the loop against 91, measured); a lone try further on can
    meet a refill the same way, which is why the asleep plateau is read
    as three positions in a row.
  - THE SAME CONTRACT ON CORE 1 (`test_rp2040_multicore` letter k, ten
    runs of ten): the walk run by core 1 itself on `Rp2040Platform<1>`
    and `CoreTicker<1>`, inside its kernel's dispatch - its own SysTick
    placed by a reload, PWM slice 0's wrap in its own NVIC, and THE
    DOORBELL, rung by core 0 a counted number of its own cycles after
    core 1 asks; every position of 400, a plain WFI and with SLEEPDEEP
    set, the timer's alarm 1 the rescue in core 1's NVIC. None lost,
    none a period late; asleep, 34 cycles from the tick's edge to its
    handler (the shared vector asks the core first) and 28 from the
    line's, 110 and 102 to the loop. The doorbell's positions are
    counted on core 0's clock: the two cores share none, so where the
    bell lands in core 1's path is not read back, only that no position
    of 400 lost it. A quiet pack on core 1 turns 100 times over 100 of
    its ticks.

**The preemptive kernel** - the reference suite is `test_rp2040_rubato`
(55 verdicts in `z`, ten runs of `z` alike) on the WeAct RP2040 at
125 MHz: a Rubato pack of five objects on `Rp2040RubatoPlatform<0>`,
the Console the lowest and the letters' runner, alarm 0 of the system
timer the interrupt the road-1 letters land, the one PendSV entry bound
to the pair body for every letter (letter k runs a Rubato on each core),
nothing wired and nothing lit. The ruler is core 0's composed `cycles()`
(SysTick's position and the tick count): two back-to-back reads are 55
cycles apart, a cost taken off every span below. Against the system
timer over four spans of 1 ms, a span whose pairs of reads missed no
XIP line agrees within 15 to 134 cycles - the microsecond's two floors,
125 cycles, and the skew between the pairs - while a span with a cache
refill inside a pair of reads is off by up to 573 and is reported, not
judged. That is the suite's method throughout: an event, a run or a
span that a kernel tick, an XIP refill or the console line's handler
lengthened is told apart by counters read around it (the tick count,
the XIP cache's miss counter, the console vector's entries) and
reported, and each bound judges the ones nothing lengthened; the first
span after a console drain always carries a refill. clk_sys cycles:

- **The ceiling, both roads** (letters a, b, e): an alarm's post to High
  inside Low's handler runs High to its end before Low resumes, every
  handler reading its own index as the ceiling and the base's 255 in the
  UART's interrupt; Low posting to Mid and Mid to High nests High inside
  Mid inside Low, each inside the post that asked; at depth 2 an
  interrupt over High's handler, started by either road, runs Top before
  High resumes, Top's first statement 186 to 187 cycles after the
  handler's last.
- **The latency of road 1** (letter c): from the alarm handler's last
  statement to High's handler's first, 209 cycles min, 210 median and
  max over 100 events, none missed.
- **The scan and the post** (letter d): `activate()` with four empty
  queues below the ceiling 124 cycles; a post that asks for nothing 80;
  a post that runs High's empty dispatch inside it 491, the scheduling's
  share 411.
- **Time events under preemption** (letter f): three timers matured in
  one turn are posted with the ceiling held at 0 and served High, Mid,
  Low; High preempting Low's loop of `arm()` and `disarm()` by road 1
  leaves the list whole, every timer due fired once, the disarmed one
  never; one `arm()` 85 cycles with its masked window inside.
- **The request under PRIMASK** (letter g): a post to a higher object
  inside a critical section pends PendSV and is served when the
  section's guard restores the mask, before the statement after it,
  from the Console's dispatch and from inside Low's; the section's last
  statement to the receiver's first 233 cycles in every run nothing
  lengthened (in one run of ten a kernel tick or an XIP refill
  lengthened one, told apart by the counters).
- **Road 1 across the cores** (letter j, core 1 under Tenuto running one
  object, Relay, that answers core 0's send with a send to High): a post
  from core 1 to core 0's object is refused and counted as a mispost and
  pends no PendSV on core 1; Low's send crosses to core 1 and Relay's
  answer crosses back, core 0's doorbell handler posts to High, and
  High's dispatch starts after that handler's last statement and before
  Low resumes, 7 to 11 us from Low's send to High's first statement. From
  the doorbell handler's last statement to High's first, 209 cycles
  min, 210 median and max over 100 crossings, none over 292: the road
  1 of letter c, a doorbell's handler in place of an alarm's.
- **A Rubato on each core** (letter k, core 1's pack of three on
  `Rp2040RubatoPlatform<1>`, both cores through the one pair body):
  core 0 traces `Ls3 Lm3 Hs1 Ts0 Te0 He1 Lr3 Le3` and core 1
  `ls2 lm2 hs1 ts0 te0 he1 lr2 le2` (who, what, the ceiling read) in 10
  of 10 runs, road 1 then road 2 on each core with every ceiling its own
  object's index;
  the two cores preempt at once, each preemption landing inside the
  other core's preempted dispatch on the system timer, in 10 of 10;
  core 1's ceiling reads 255 at its base after; core 1's stack
  high-water mark 236 bytes, its deepest sampled stack pointer 216 in a
  dispatch and 104 in its interrupt.
- **Dark** (letter i): every object of core 0 idle, core 1 parked in the
  bootrom, no time event and no alarm armed, GP25 not an output, the
  ceiling the Console's, interrupts enabled.
- **The stack** (letter h): core 0's high-water mark 532 bytes for the
  suite's deepest chain, its deepest sampled stack pointer 512 in a
  dispatch and 368 in an interrupt.

## A handler in SRAM

**What the silicon does.** The core executes the flash in place through
the XIP cache (2.6.3.1): 16 kB, two-way set-associative, lines of 8
bytes, a hit in one cycle, in front of a quad-SPI flash the second stage
runs at clk_sys / 4. So code that is warm runs at the SRAM's pace, and a
MISS stalls the core for the line's fetch over the QSPI bus - 50 to 80
cycles a line at 125 MHz by this chapter's figures. What the cache does
not hold is every line a path runs for the first time after `init()`,
after a flush, or after other code has evicted it: the larger images
here are three times the cache, and a program's paths share its sets. SRAM sits 256 MB above the flash and a
Thumb `bl` reaches 16 MB, so A CALL BETWEEN THE TWO MEMORIES GOES
THROUGH A VENEER the linker puts beside the caller (`push {r0}; ldr r0,
[pc, #8]; mov ip, r0; pop {r0}; bx ip` and the address, 16 bytes). The
vector table holds absolute addresses, so a vector placed in SRAM is
entered with no veneer.

**The option and its spelling: opt-in, per driver.** `rp2040_2m.ld`
collects `.fast_text` and `.fast_text.*` first in `.data`, then
`.ram_text` and `.ram_text.*`, so the crt's copy of `.data` moves them
to SRAM before `main()`, and the platform says so
(`Rp2040Platform::places_fast_text`). `.fast_text` is code that MAY run
from SRAM, for speed; `.ram_text` is code that MUST - the flash engine's
window ([flash.md](flash.md)), spelled `.ram_text.brio` as a header's
function must be (the COMDAT rule of util/placement.hpp). An image that
places nothing pays nothing. Two ways, each path the one that reaches
it:

- THE PROGRAM PLACES THE VECTOR IT BINDS, one attribute, and the
  driver's inline body moves with it:

  ```cpp
  extern "C" [[gnu::section(".fast_text"), gnu::flatten]] void isr_i2c1() {
      if (Bus::isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
  }
  ```

- A DRIVER IS GIVEN util/placement.hpp's `InRam`, its last template
  parameter, for a path no vector reaches - a host's `start()`, in main
  context - or a program that keeps its vectors in the flash: `SpiHost`
  (`start()`, the pump, the DMA line's body), `Uart` (the vector's body,
  the DMA line's), `I2cHost` (`start()`, the vector's body, the DMA
  line's). Each path is ONE flattened instance in `.fast_text.brio`, a
  call from the flash through a veneer; `InFlash`, the default, leaves
  each body where it was, inline in its vector or in the flash:

  ```cpp
  using Display = brio::SpiHost<1, display_pins, brio::NoDmaEngine, brio::NoDmaEngine, brio::InRam>;
  ```

  A third tag does not compile (`test/family_rp2040/neg/`).

**What it buys, measured** (`bench_rp2040` and `bench_rp2040_ram`, and
the two images a define gives between them; cycles at 125 MHz, each
cell WARM / COLD - cold read by letter c, the XIP cache flushed before
every timed run). F: every driver InFlash and every vector in the flash
(the default); R: the drivers InRam; V: the vectors placed; B: both.

| path | F | R | V | B |
|---|---|---|---|---|
| SPI polled request, a command byte and 2 data bytes (`spi.req`) | 788 / 11364 | 801 / 2944 | 788 / 11364 | 801 / 2944 |
| SPI pump, 16 frames at clk_peri / 2, wall | 1527 / 20936 | 1598 / 9968 | 1527 / 11100 | 1580 / 4280 |
| ... its handler, two entries | 929 / 7889 | 997 / 3790 | 929 / 929 | 979 / 979 |
| SPI on the engines, 16 frames, wall | 1631 / 34002 | 1473 / 18028 | 1447 / 22782 | 1464 / 10768 |
| ... the DMA line's handler | 190 / 8622 | 202 / 4664 | 190 / 190 | 193 / 193 |
| UART interrupt receiver, 16 bytes at 1 Mbaud, handler | 5274 / 16628 | 1826 / 9865 | 823 / 2897 | 1391 / 2964 |
| UART transmit engine, 256 bytes at 1 Mbaud, completions | 1109 / 9738 | 2081 / 6449 | 459 / 2032 | 452 / 2025 |
| I2C one-byte write at 400 kHz, `start()` | 349 / 12429 | 366 / 5209 | 349 / 15865 | 366 / 5209 |
| ... its longest handler entry | 155 / 6182 | 190 / 3162 | 155 / 155 | 181 / 181 |
| I2C 16-byte read on the engines at 400 kHz, wall | 49498 / 69460 | 49553 / 63504 | 49499 / 60096 | 49544 / 54576 |

What the rows say:

- WARM, THE PLACEMENT BUYS NOTHING ON THIS CORE AND COSTS THE VENEER:
  a polled request 13 cycles dearer under InRam, a pump entry some 35,
  an I2C entry 26-35 - the cache serves a warm path at the SRAM's pace,
  and the polled loops read the same to the cycle in both places
  (`spi.poll` of 256 frames 5744 against 5757). A warm figure that
  differs more is the cache's set conflicts in that one image (the UART
  rows: the ops of letter u each run once, after other code).
- COLD IS WHERE IT PAYS, and it pays on every path: a three-byte request
  11364 cycles (136 misses) from the flash against 2944 (41, the
  caller's) with the host InRam; a handler cold from the flash 6000 to
  8600 cycles, from SRAM what it costs warm.
- THE VECTOR PLACED REACHES WHAT A VECTOR RUNS, and nothing else: the
  handlers read their warm figures cold, but a host's `start()` stays
  where it is. InRam reaches `start()` and halves a handler's cold cost
  behind a vector left in the flash (the vector's own lines and the
  veneer still miss). BOTH is the image that runs a cold transaction
  closest to a warm one; what it still fetches is the caller's code, the
  idle path and the tick.
- THE SRAM IT COSTS, one instance a path and host type: the SPI host's
  `start()` 944 bytes without engines and 1344 with them, its shared
  engine launch 568, the pump 268 to 320, the DMA line's body 540; the
  I2C host's 592 to 1652; the UART's 156 to 720 - the bench's four
  drivers, in all their types, 11.6 KB.

So: a program that cannot afford the cold path - a first transaction
after `init()` on a deadline, a handler that must answer within a frame
whatever ran before - places the vectors it binds and gives `InRam` to
the hosts whose `start()` is on that deadline; every other program
leaves both alone, since a warm path gains nothing here.

## Not covered yet

Driver gaps, each with its reason:

- The SIO's spinlocks, dividers and interpolators: a program's
  business (the FIFOs are the doorbell's, [multicore.md](multicore.md));
  nothing portable wants them today.
- The bus fabric's priorities and performance counters (BUSCTRL): a
  measurement tool, born with the two-core contention question.
- A vector table per core, the other way two Rubato kernels could share
  PendSV ("How to use it"): not built while the choice between it and the
  pair body is open.

Implemented but not bench-verified, each with what would measure it:

- The single-kernel binding, the core stratum's `scheduler_exception<K>()`
  bound to `isr_pendsv` with core 1 under Tenuto or not launched ("How
  to use it"): it runs in no image, `test_rp2040_rubato` binding the pair
  body for every letter. Measured by the suite's letters but k, built
  with the single body bound.
- The preemptive kernel on the Pico: `test_rp2040_rubato` is built for
  that board too, and a `z` there would measure it.
- Whether a pending PendSV ends a WFI as a pending NVIC line does: the
  kernel's idle path never sleeps with PendSV pending in the suite's
  shapes (a post made at the base is served before the masked check),
  so no letter asks it; a WFI entered masked with PENDSVSET written
  first, timed on the system timer, would.

- The idle path through the DORMANT hook: a wake that is already
  latched when the keyword is written (a GPIO dormant-wake event, the
  RTC's alarm) is not shown to end the state at once. A pad the chip
  drives itself, its dormant-wake event armed, PWM-placed edges walked
  across `go_dormant()` with the RTC on the ring oscillator as the
  rescue.
- Letters `w` and `k` on the Pico: a `z` there.

- `Resets::hold` on a running block and the block's state afterwards:
  a `test_rp2040_clock` letter cycling a UART and reading its
  registers at their reset values.
