# Platform (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 2.3
(processor subsystem: SIO, the two NVICs, table 2.3.2's interrupt
lines), 2.6 (memory: the SRAM banks), 2.7 and 2.8 (boot sequence,
bootrom, the second stage), 2.14 (subsystem resets), 2.20 (SYSINFO);
Appendix B. The drivers: `brio/rp2040/platform.hpp`, `nvic.hpp`,
`ticker.hpp`, `delay.hpp`, `resets.hpp`, `sysinfo.hpp` over
`brio/cortexm/`; the crt `rp2040/src/glue/startup_rp2040.cpp`, the
second stage `rp2040/src/glue/boot2_*.S`, the linker script
`rp2040/ld/rp2040_2m.ld`. The reference suite: `test_rp2040_platform`
(the boot state, the ticker against the system timer, `delay_us`, the
reset controller, the SIO, the atomic aliases, the WFI idle, the idle
path against an interrupt's edge placed to the cycle, the kernel's turn
per interrupt, the breadcrumb, five real resets). The failing half of the platform - the
causes, the watchdog, the timer - has its own documents:
[reset.md](reset.md), [watchdog.md](watchdog.md), [timer.md](timer.md).

## What the silicon does

Two Cortex-M0+ cores share one bus fabric, one flash and one SRAM;
each has its own NVIC, its own SysTick and its own PRIMASK. Every one
of the 32 interrupt lines reaches BOTH NVICs and each core enables
what it wants; a line enabled on both interrupts both cores. Out of
reset only core 0 runs code; core 1 sits in the bootrom's launch
protocol, waiting on the inter-core FIFO (2.8.2).

The boot chain: the bootrom (in ROM, on the ring oscillator at about
6.5 MHz) reads the first 256 bytes of the flash into SRAM, checks
their CRC32 and runs them; that SECOND STAGE programs the XIP serial
interface for the flash chip in use and then vectors through the
table it expects at flash offset 0x100 - it writes VTOR with that
address and loads SP and PC from the table's first two words. So an
image is the stage, then the vector table, then the load image of
.data - which carries the code that runs from SRAM, the flash engine's
and the SPI host's hot path ([flash.md](flash.md), [spi.md](spi.md)) -
then the code, and nothing in brio writes VTOR again. Every peripheral the reset controller
governs (2.14: the UARTs, SPIs, I2Cs, the timer, the PWM, the ADC,
both PLLs, both PIOs, the DMA, the IO and pad banks, USB, SYSINFO) is
HELD IN RESET at power-up: a driver's first act is releasing its
block and waiting for RESET_DONE, the way the other families open a
clock gate.

The SRAM is 264 KB in six banks: four of 64 KB striped word by word
over 0x2000_0000..0x2003_FFFF, so consecutive words fall in different
banks and two masters seldom contend, and two UNSTRIPED banks of 4 KB
at 0x2004_0000 and 0x2004_1000 - where the two cores' stacks go, one
each, so a core's stack traffic never contends with the other core's
or the DMA's. Kernel time is the core's SysTick on clk_sys, which
nothing short of the DORMANT state stops.

## Types and verbs

- `Rp2040Platform<TB = Ticker>` - the kernel's Platform:
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
- `Ticker` = `BasicTicker<1000>` on SysTick, `SysTickCounter` for a
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
    `Rp2040SleepSite`, every SLEEP_EN gate left open). 3200 tries: none
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
    controller does not take, ends a standby WFI as it ends a light one.
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

## Not covered yet

Driver gaps, each with its reason:

- The SIO's spinlocks, dividers and interpolators: a program's
  business (the FIFOs are the doorbell's, [multicore.md](multicore.md));
  nothing portable wants them today.
- The bus fabric's priorities and performance counters (BUSCTRL): a
  measurement tool, born with the two-core contention question.

Implemented but not bench-verified, each with what would measure it:

- The idle contract on CORE 1. The type is the same template and the
  WFI path the same instructions, but core 1 has its own SysTick, its
  own NVIC and one wake of its own - the SIO FIFO's doorbell
  (`SIO_IRQ_PROC1`), which is how an event sent to it arrives
  ([multicore.md](multicore.md)). Letters `w` and `k` launched on core
  1 with `Rp2040Platform<1>` and `CoreTicker<1>`, the rescue alarm
  enabled in core 1's NVIC, the doorbell as a third edge placed by a
  FIFO write from core 0, the counts handed back through SRAM.
- The idle path through the DORMANT hook: a wake that is already
  latched when the keyword is written (a GPIO dormant-wake event, the
  RTC's alarm) is not shown to end the state at once. A pad the chip
  drives itself, its dormant-wake event armed, PWM-placed edges walked
  across `go_dormant()` with the RTC on the ring oscillator as the
  rescue.
- The standby rung with a pruned SLEEP_EN set: letter `w` walks it
  with every gate open. The same letter with the program's own set,
  the tick's and the rescue's clocks kept.
- Letters `w` and `k` on the Pico: a `z` there.

- `Resets::hold` on a running block and the block's state afterwards:
  a `test_rp2040_clock` letter cycling a UART and reading its
  registers at their reset values.
