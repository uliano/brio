# Platform (RP2040)

Documents of record: the RP2040 datasheet (build 3184e62), 2.3
(processor subsystem: SIO, the two NVICs, table 2.3.2's interrupt
lines), 2.6 (memory: the SRAM banks), 2.7 and 2.8 (boot sequence,
bootrom, the second stage), 2.14 (subsystem resets), 2.20 (SYSINFO);
Appendix B. The drivers: `brio/rp2040/platform.hpp`, `nvic.hpp`,
`ticker.hpp`, `delay.hpp`, `resets.hpp`, `sysinfo.hpp` over
`brio/cortexm/`; the crt `rp2040/src/glue/startup_rp2040.cpp`, the
second stage `rp2040/src/glue/boot2_*.S`, the linker script
`rp2040/ld/rp2040_2m.ld`; `brio/util/placement.hpp` for the placement.
The reference suite: `test_rp2040_platform`
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
window and whatever a program places ("A handler in SRAM", below) -
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

Implemented but not bench-verified, each with what would measure it:

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
