# Target: RP2350 (`rp2350/`)

The operational page for brio's RP2350 target: the silicon Raspberry Pi
put beside the RP2040, and the only one this project has met that
carries TWO PROCESSOR ARCHITECTURES over one set of peripherals - a pair
of Arm Cortex-M33 and a pair of RISC-V Hazard3, of which exactly one
pair runs at a time. Which one is not a switch on the board and not a fuse: it is
a property of THE IMAGE IN THE FLASH, so the architecture is an axis of
the build here, the way the part number is on the STM32F4 project, and
every suite of this target is written once and run twice.

`kernel/`, `util/` and `gfx/` compile here as written, through both
compilers (`test/family_rp2350/util_all.cpp` is the whole of them over
the platform, and the family check builds it four times).

Peripheral documents live next to this page, one per chapter - the map
below; the documents of record are in
[vendor/README.md](vendor/README.md).

## The documents

| Document | Content |
|----------|---------|
| [vendor/README.md](vendor/README.md) | The RP2350 datasheet by build, the hardware design guide, the vendored pico-sdk subset and the SVD, the bench chip's identity, the errata this stratum answers |
| [platform.md](platform.md) | The platform on two instruction sets: one platform type per core and one for both architectures, the idle path and the lost-wakeup window it does not have (two rules, one promise), the microsecond ruler that is also the Hazard3 half's timebase, the subsystem reset controller, the atomic register aliases, the panic breadcrumb across a processor reset, erratum RP2350-E9 shown on a pad, and the two registers called PLATFORM |
| [clock.md](clock.md) | The clock tree: four roots where the RP2040 had three (the low-power oscillator joins clk_ref), a generator more and none for an RTC, the 16.16 dividers, the tick generators as a block of their own, the ring oscillator's randomiser, the PLLs' lost lock, the resus circuit - and two CTRL registers that are passwords, where a masked write is a refused write |
| [pin.md](pin.md) | GPIO: the pads that come up isolated and the latch that holds them, the four overrides and the STATUS behind them, the pin interrupts with their summary registers and the forced event that bypasses the enable, and erratum RP2350-E9 measured four ways |
| [uart.md](uart.md) | The UART: what this chip owes the PL011 that its predecessor did not - a SECOND FUNCTION COLUMN that makes every group's flow-control pads a second data pair, pads that come up isolated, a DREQ table with not one row in the old place, and one interrupt name bound on both architectures |
| [i2c.md](i2c.md) | The I2C: the Synopsys block this chip carries unchanged, register for register, and the five things around it that are not - one function column over forty-eight pads where the UART has two, pads that come up isolated, a bank of two words that an open-drain drive by hand must tell apart, a DREQ table and a reset controller whose bits have all moved, and a bus clear by hand paced by the one ruler both architectures have |
| [timer.md](timer.md) | The two system timers: a 64-bit counter of microseconds each, four alarms with an interrupt line apiece, the tick generator of 8.5 behind each one, and the two registers this chip added - the counter taken off the tick and onto clk_sys, and a lock that refuses every write until the block is reset |
| [watchdog.md](watchdog.md) | The countdown and the four scratch registers a program may use: the tick that now comes from the TICKS block, the RP2040's double decrement that is not this chip's, the three WDSEL registers in their three tiers, and erratum RP2350-E19's guard before every reboot |
| [reset.md](reset.md) | Chapter 7 whole: the three tiers, the causes recorded in the always-on power manager beside the watchdog's REASON, the power-on state machine, the subsystem controller, the reboot both architectures have and the processor reset only one of them has |
| [dma.md](dma.md) | The DMA: sixteen channels and four interrupt lines, a MODE in the top nibble of the transfer count that makes a channel re-arm itself or run forever, an address step that can go backward or by twos, the abort that answers erratum RP2350-E5, a security level on every resource - and the two engines a transport names in its slots |
| [pwm.md](pwm.md) | The PWM: twelve slices where the RP2040 had eight, the pin map's second half on GPIO 32..47 and what the four highest slices are in the package that bonds none of those pads, a second shared interrupt line with a register set of its own, and a fractional divider whose last step is a whole 256 |
| [pio.md](pio.md) | The programmable I/O, chapter 11: three blocks of four machines over one instruction set, and the six things this chip added to it - a version field, a window that says which thirty-two pads a block can see, a CTRL write that reaches the blocks either side of it, all eight flags on the interrupt lines, a masked input count, and a receive FIFO that can be four registers instead of a queue |
| [sha256.md](sha256.md) | The SHA-256 accelerator: the compression function in hardware and nothing else, the padding software owes it, the byte swap that reconciles a little-endian bus with a big-endian standard, the handshake and the flag that catches a missed one - and the constexpr twin the silicon is judged against |
| [trng.md](trng.md) | The true random number generator: a ring oscillator with no tie to the clock tree, three entropy checks of which one is fatal until the block is reset, a generation time that is not deterministic, and the raw-sample path the bootrom takes instead |
| [otp.md](otp.md) | The one-time programmable array, READ SIDE ONLY and deliberately so: four read windows of which two fault instead of lying, page locks that only climb, the error correction decoded in software with the one sentence of the chapter that must be read carefully, and the flags a program must never set - printed, not written |
| [bootrom.md](bootrom.md) | The mask ROM's public function table: two sets of well-known words, one per architecture, and two different lookups over them; what is wrapped, and the three entry points that deliberately are not |

## The two architectures, and what decides between them

The bootrom scans the flash for a block loop with a valid IMAGE_DEF and
enters the image it finds (datasheet 5.9.5). One word of that block
names the architecture, and flashing an image whose word names the other
one makes the bootrom reset both cores into it: no button, no OTP write,
no second board. Measured on the bench chip in both directions -
ARCHSEL_STATUS (0x4012015c) reads 0 after an Arm image and 3 after a
RISC-V one.

That is why `rp2350/` has four presets and not two:

```bash
(cd rp2350 && cmake --preset rp2350-arm-release)                      # configure (once, or after adding an app)
(cd rp2350 && cmake --build --preset rp2350-arm-release --target <app>)
(cd rp2350 && cmake --build --preset rp2350-riscv-release --target <app>)
(cd rp2350 && cmake --build --preset rp2350-arm-release --target <app>-upload)
brio check rp2350 [name]                                              # family smoke, both compilers, no hardware
```

and why the bench manifest names the same physical board twice, once per
architecture (`weact2350b` and `weact2350b-rv` in
[../boards/README.md](../boards/README.md)'s board types): `brio flash`
takes a board type and the board type carries the preset.

An app is written ONCE. It names `brio::Rp2350Platform<>`,
`brio::Ticker`, `brio::Clock<...>` and binds its vectors by the crt's
names - `isr_systick`, `isr_uart0`, `isr_dma_0` - and those names are
bound on both halves: the Arm crt puts them in a Cortex-M vector table,
the RISC-V crt binds the same symbols to the machine timer trap and to
Hazard3's own dispatch.

## Toolchains

TWO, both self-built under `/sw`, both by absolute path:

- **arm-none-eabi-gcc 16.2** at `/sw/arm-none-eabi` for the Cortex-M33
  half, with the HARD-FLOAT ABI (`-mcpu=cortex-m33 -mthumb
  -mfloat-abi=hard -mfpu=fpv5-sp-d16`, the `thumb/v8-m.main+fp/hard`
  multilib) - the crt enables CP10 and CP11 before `.data` is touched,
  as on the STM32F4.
- **riscv32-unknown-elf-gcc 16.2** at `/sw/riscv32-unknown-elf` for the
  Hazard3 half, `-march=rv32ima_zicsr_zifencei_zba_zbb_zbs_zbkb_zca_
  zcb_zcmp -mabi=ilp32`. Upstream RISC-V, NOT the WCH compiler the two
  QingKe families use: this core carries no vendor extension, and the
  `-march` string sits in a cache variable because the toolchain also
  has the multilib without Zcb and Zcmp and what those two cost or save
  on real images is a measurement this stratum still owes.

What is ARMv8-M and not Raspberry Pi's comes from the `cortexm/` core
stratum (`brio/rp2350/core_m33.hpp` is the device header and then
`cortexm/nvic.hpp`); what is RISC-V and not Raspberry Pi's is
`brio/rp2350/core_hazard3.hpp`, which is this stratum's own until a
second RISC-V family of this shape earns a core stratum. `core.hpp` is
the ONE file that asks which processor is in the socket, and both halves
export the same names, so nothing above it knows.

The device description is vendored from the pico-sdk at tag 2.3.1
(`third_party/pico-sdk/rp2350/`, BSD-3-Clause), in its own include root
because the RP2040's files have the same names. On the RISC-V build the
device header's unconditional `#include "core_cm33.h"` resolves to a
stub of this project's own (`third_party/pico-sdk/rp2350/no_core/`):
the register map, the instance pointers and the IRQ numbers are the
chip's whichever core reads them - the system interrupt numbering is
SHARED between the architectures (3.8.4.2) - and only the core
description is not.

## Board and build

The board on the bench is a **WeAct RP2350B core board**
([../boards/weact-rp2350b.md](../boards/weact-rp2350b.md)): an RP2350 in
the QFN-80 package (48 GPIO, eight ADC inputs), stepping **A2**, behind
a 16 MB Winbond W25Q128 quad-SPI flash, a 12 MHz crystal, USB-C, a user
LED on **GP25** and a user KEY on **GP23**, at 3.3 V.

What a configure targets, besides the architecture, is a FLASH GEOMETRY
and a PACKAGE: `RP2350_FLASH_KB` picks the linker script, `RP2350_PACKAGE`
(a for the QFN-60, b for the QFN-80) becomes the define the stratum
reads its bank size from - so a pad the package has not got is a COMPILE
error, and an image built without the define compiles for the larger
package and refuses the extra pads at run time against SYSINFO's
PACKAGE_SEL.

THERE IS NO SECOND-STAGE BOOTLOADER, unlike the RP2040: the bootrom sets
the XIP interface up itself while scanning the flash (03h serial reads
at CLKDIV 12, 5.1.4), so an image begins with its own first byte - a
vector table on Arm, one jump instruction on RISC-V - and what marks it
an image is the twenty-byte IMAGE_DEF block the crt places right behind
that, well inside the first 4 kB the bootrom reads. A program that wants
the flash faster reprograms the QMI itself, which is a later chapter.

Build outputs land in `build-cmake/rp2350-{arm,riscv}-{release,debug}`:
`<app>.elf/.bin/.hex`, `firmware-<app>.map`, `<app>.lst`.

## The probe, and the three OpenOCDs

The board is reached by a **Raspberry Pi Debug Probe** (CMSIS-DAP v2,
the USB bulk backend): port D on the four-pad SWD header, port U crossed
onto GP0 and GP1 for a console when a program has one.

Three OpenOCD builds live under `/sw`, and this target uses the third:

| Build | What it is for |
|-------|----------------|
| `/sw/openocd` (0.12.0 release) | every SWD path of the other targets; does not know this chip |
| `/sw/openocd-git-bedefa2` | the RP2040 boards whose flash chip the release does not know; it can attach to an RP2350's Arm cores but CANNOT debug Hazard3 (its RISC-V driver takes no DAP) |
| `/sw/openocd-rpi-acff23f` (Raspberry Pi's fork) | THIS TARGET. It examines all four cores - `set USE_CORE cm0` or `rv0` before `-f target/rp2350.cfg` - serves gdb on either, and flashes from either side |

The fork warns that the probe's firmware is old and falls back to a
"low-performance workaround"; that is cosmetic here and no probe is
updated for it.

The debugger is the same fork with a gdb per architecture:
`/sw/arm-none-eabi/bin/arm-none-eabi-gdb` for the M33 half,
`/sw/riscv-gdb/bin/riscv32-wch-elf-gdb` for the Hazard3 one (generic
under that name, whatever its prefix says).

## The flash verb, and why it is what it is

`brio flash <board> <app>` runs TWO OpenOCD sessions, and this pair is
THE STATE-INDEPENDENT VERB of this target:

1. **The rescue**, over the debug port and nothing else: a DAP with no
   target behind it, RESCUE_RESTART set and cleared in the RP-AP's
   control register (3.5.8). It works "even when system clocks are
   stopped and the switched core power domain is powered down" and
   leaves the chip halted in the bootrom, in the Arm architecture, with
   the clock tree at its reset state and the SRAM gone. The target
   script's own rescue path is NOT usable for this: it examines a core
   first, and a core that cannot be examined is exactly the case a
   rescue is for.
2. **The programming**, as core 0 of the Arm pair - which after the
   rescue is what is running, whatever the image in the flash named -
   then `reset run`, which hands the chip back to the bootrom and so to
   the new image's own architecture. After an Arm image the session ends
   by taking halting debug back down (DHCSR with its key and every
   control bit clear), so that a `break_here()` faults instead of
   halting the core; there is no such write after a RISC-V image, that
   register belonging to a core which is not the one running.

So a board is recoverable from any state a program can put it in, with
no button and no replug - which is what makes the sleep chapters of this
target safe to write.

## What the silicon does to a program, before any driver

Four facts shape every chapter of this stratum, and all four are
measured on the bench chip:

- **`reset run` is a PROCESSOR reset, not a chip reset.** It resets the
  cores and leaves the clock tree, the pads and the peripherals exactly
  as the previous image left them, so an image may start on a PLL it did
  not lock. Nothing in this stratum assumes a reset state it did not
  make itself: `Clock::init()` walks clk_sys onto clk_ref and clk_ref
  onto a ring oscillator it started, before it touches a PLL. Measured:
  with the program running on the system PLL at 150 MHz, a `reset run`
  restarts it and it reaches 150 MHz again with the tree never having
  stopped. (A rescue, a watchdog event or a power cycle DO reset them.)
- **The pads come up ISOLATED.** PADS_BANK0's reset value is 0x116: the
  ISO latch set, the input buffer DISABLED, a pull-down on. While the
  latch stands the pad is cut off from the digital logic in both
  directions, so a pad configured as on the RP2040 would do nothing at
  all. Every configuring verb of `pin.hpp` writes the pad with ISO
  clear.
- **THE BIT MAPS ARE NOT THE RP2040'S.** IO_BANK0 and PADS_BANK0 sit in
  RESETS bits 6 and 9 here (5 and 8 there), the reset controller governs
  twenty-nine blocks against twenty-seven, the clock generator's
  dividers are 16.16 where the RP2040's were 24.8, there is no clk_rtc
  at all, and the frequency counter's source numbering differs. Nothing
  of this chip is retyped from the other one: every constant comes from
  this chip's own headers.
- **Erratum RP2350-E9 is live on stepping A2.** With its input buffer
  enabled, a pad that nothing drives leaks enough current through the
  input stage to sit HIGH against its own pull-down; a pad driven low
  and then released stays low. An idle level on this silicon is
  HISTORY, not a measurement: a program that wants to read a floating
  pad through its pulls must drive it first, or use the pull-UP and read
  a low as its signal. Stepping A3 removes the leakage path.

And two more the RISC-V half adds:

- **`wfi` ignores mstatus.MIE** (3.8.5) and respects every other
  interrupt control, so the kernel's idle path - mask, look at the
  queues, sleep, unmask - has no lost-wakeup window here, exactly as on
  the Arm half. This is NOT the QingKe cores' behaviour, where the same
  sequence deadlocks.
- **Hazard3's clock-gated sleep is never armed.** MSLEEP.DEEPSLEEP would
  stall a debugger's system-bus reads on core 1 until that core wakes
  (erratum RP2350-E4), and the datasheet's own note is that the saving
  over a plain `wfi` is minimal.

## First light, measured

The `blink` app is one active object toggling GP25 from a periodic time
event, built from one source for both architectures and reporting
through globals a debugger reads out of the ELF while the program runs
(the fork reads memory under either architecture without halting). On
the bench chip, the same numbers on both halves:

| What | Cortex-M33 | Hazard3 |
|------|-----------|---------|
| ARCHSEL_STATUS after the flash | 0 | 3 |
| clk_sys by the chip's own frequency counter | 150 000 000 Hz | 150 000 000 Hz |
| clk_ref (the crystal) | 12 000 031 Hz | 12 000 031 Hz |
| kernel ticks over ten host seconds | 10 001 in 10 001 ms | 10 002 in 10 001 ms |
| LED toggles in the same span | 40 (4.00 Hz) | 40 (4.00 Hz) |
| `idle()` calls covering 200 ticks | 200 | 200 |
| awake in that span | 4 us of 200 480 | 4 us of 200 422 |
| a spare interrupt line raised in software and served | once | once |

GP25 sampled over the debug port every 125 ms reads 1 0 0 1 1 0 0 1 1 0
0 1: a 250 ms half period, the programmed one. The pad register reads
0x052 - the isolation latch dropped, the input buffer on, 4 mA - and
FUNCSEL reads 5, SIO.

The last row is the whole interrupt path on a line that reaches no
hardware (one of the six spare ones, 3.2): enabled in the controller,
raised in software, served by a handler an app bound BY NAME -
`isr_spare_0`, which is the slot `isr_irq46` on both halves, through the
Arm vector table on one and through Hazard3's `meinext` dispatch on the
other.

The awake time is under the microsecond ruler's resolution in nearly
every call on both halves. A run in which one wake covers two ticks
shows fewer `idle()` calls than ticks - that is the measuring loop's own
race between the tick it reads and the sleep it then enters, not a
difference in the sleeping.

## Not covered yet

Driver gaps, each with its reason:

- **Most of the chip.** The SPI, the ADC, the flash and the QMI,
  the USB, POWMAN with its always-on timer and the sleep states, the
  second core, and two of the blocks the RP2040 never had - the HSTX and
  the M33's coprocessors - have no driver here yet. Each arrives with its
  chapter, its suite on both architectures and its document.
- **The programming side of OTP**, in any form: declined permanently
  ([otp.md](otp.md)), and the absence is asserted by the family check
  rather than promised.
- **The QFN-60 package.** The stratum compiles for it and refuses its
  absent pads, but no QFN-60 part is on the bench: everything below the
  compile check is untested there.
- **Interrupt priorities.** Both halves have sixteen levels and neither
  uses them: the kernel's promise is that no interrupt nests over
  another, and here it is kept structurally (every line at the reset
  priority on the M33, mstatus.MIE never set inside a handler on
  Hazard3). The two conventions run opposite ways - numerically higher
  is more urgent on Hazard3 - which is why `Irq` carries no priority
  verb at all.

Implemented but not bench-verified:

- **The QFN-60's compile-time refusals**, above.
- **Core 1, on either architecture.** The platform is per core and the
  tickers are per core by construction, but nothing has launched a
  second core here: that is the multicore chapter, with the bootrom's
  protocol and the inbox bridge.
- **`Mtime` and the tick generators under a clk_ref that is not 12 MHz.**
  The arithmetic refuses a clk_ref that is not a whole number of
  megahertz, and only the 12 MHz crystal has been on the wire.
- **The machine software interrupt** (`isr_riscv_softirq`, the RISC-V
  crt's third trap): the doorbell a second kernel will ring, bound and
  never raised. The external and the timer traps beside it are both
  proven on the bench.
- **The Arm crt's fault vectors.** The four configurable faults have
  their names in the table and their weak spins. `isr_hardfault` and the
  RISC-V crt's `isr_riscv_exception` are what `fault_reset()` binds to
  ([reset.md](reset.md)); nothing has faulted on purpose yet.
