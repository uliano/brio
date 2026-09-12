# Platform (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 2.3
(processor subsystem: SIO, the two NVICs, table 2.3.2's interrupt
lines), 2.6 (memory: the SRAM banks), 2.7 and 2.8 (boot sequence,
bootrom, the second stage), 2.14 (subsystem resets), 2.20 (SYSINFO);
Appendix B. The drivers: `brio/rp2040/platform.hpp`, `nvic.hpp`,
`ticker.hpp`, `delay.hpp`, `resets.hpp`, `sysinfo.hpp` over
`brio/armv6m/`; the crt `rp2040/src/glue/startup_rp2040.cpp`, the
second stage `rp2040/src/glue/boot2_*.S`, the linker script
`rp2040/ld/rp2040_2m.ld`. The reference suite: `test_rp2040_platform`
(the boot state, the ticker against the system timer, `delay_us`, the
reset controller, the SIO, the atomic aliases, the WFI idle, the
breadcrumb, five real resets). The failing half of the platform - the
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
image is the stage, then the vector table, then the code, and nothing
in brio writes VTOR again. Every peripheral the reset controller
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
  `CriticalSection` = the PRIMASK guard of `armv6m/nvic.hpp`, PER CORE
  (it excludes this core's handlers and nothing of the other core);
  `idle()` = DSB, WFI, unmask - the core's clock stops, everything
  else runs, and with both cores and the DMA asleep the clock enables
  switch to their SLEEP_EN set (all enabled at reset); `now()`,
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
    brio::Kernel<P, Console, SerialLines, Blinker>::run();
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
  SysTick ticker at 1000 Hz, WFI between events, the heartbeat time
  event toggling GP25 at 2 Hz (read back through SIO), the uptime
  advancing with the host's clock.
- The reference suite is green on both boards, the Pico and the WeAct
  (52 verdicts in the all-key, 18 in the reset letter). What it
  measured, on the system timer as the ruler:
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
    199 ms;
  - `.noinit` SRAM survives a core reset, a watchdog reset and a
    HardFault's reset - a fact the datasheet does not promise;
  - the breadcrumb: a `panic()` through `ResetReporter` and a
    HardFault through `fault_reset()` each read back at the next boot
    with their code and context; the causes word is a history
    ([reset.md](reset.md)).

## Not covered yet

Driver gaps, each with its reason:

- The power modes (the clock enables in sleep, DORMANT, the voltage
  regulator's VSEL): `util/power.hpp`'s sites, born with the first
  power-aware program.
- The SIO's spinlocks, dividers and interpolators: a program's
  business (the FIFOs are the doorbell's, [multicore.md](multicore.md));
  nothing portable wants them today.
- The bus fabric's priorities and performance counters (BUSCTRL): a
  measurement tool, born with the two-core contention question.

Implemented but not bench-verified, each with what would measure it:

- The clock enables in sleep (SLEEP_EN0/1) taking effect: a current
  meter on the supply with both cores and the DMA idle.
- `Resets::hold` on a running block and the block's state afterwards:
  a `test_rp2040_clock` letter cycling a UART and reading its
  registers at their reset values.
