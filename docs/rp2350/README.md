# Target: RP2350 (`rp2350/`)

The operational page of brio's RP2350 target, for a programmer who has
a WeAct RP2350B board on the desk and wants to build, flash and debug a
brio program on it, on either of its two processor architectures, and
then find the document of the peripheral the program needs. It says
what the stratum is, what decides which architecture runs, how the two
toolchains, the build, the upload, the debugger and the console work
and where each has a trap, what the silicon does to a program before
any driver, and it ends with the map of the stratum's own documents:
one line per peripheral driver, which is where the state of the driver
work is read.

A stratum is a directory of `brio/` holding one architectural layer;
`brio/rp2350/` holds everything that includes the chip's vendored
device description. The RP2350 is the silicon Raspberry Pi put beside
the RP2040, and the only chip this project has met that carries two
processor architectures over one set of peripherals: a pair of Arm
Cortex-M33 and a pair of RISC-V Hazard3 cores, of which exactly one
pair runs at a time. Which one is not a switch on the board and not a
fuse: it is a property of the image in the flash, so the architecture
is an axis of the build here, the way the part number is on the STM32F4
project, and every suite of this target is written once and run twice.
The kernel (`kernel/`), the services (`util/`) and the drawing library
(`gfx/`) compile here as written, through both compilers
(`test/family_rp2350/util_all.cpp` is the whole of them over the
platform, and the family check builds it four times). The maintained
example `blink` builds for both architectures, and "First light,
measured" below is what it showed. Every driver's suite passes on both
halves; the target's maturity is stated in the root README's table
alone, and what is still owed is "Not covered yet" at the end.

The documents of record (the datasheet by build, the hardware design
guide, the vendored pico-sdk subset and the SVD), the bench chip's
identity and the errata this stratum answers are in
[vendor/README.md](vendor/README.md).

## The board

The board on the bench is a WeAct RP2350B core board
([../boards/weact-rp2350b.md](../boards/weact-rp2350b.md)): an RP2350
in the QFN-80 package (48 GPIO, eight ADC inputs), stepping A2, behind a
16 MB Winbond W25Q128 quad-SPI flash, a 12 MHz crystal, USB-C, a user
LED on GP25 and a user KEY on GP23, at 3.3 V. The Raspberry Pi Debug
Probe is its probe and its console, and it is wired to a WeAct RP2040
as its peer; the board's page lists the wires.

## The two architectures, and what decides between them

The bootrom scans the flash for a block loop with a valid IMAGE_DEF and
enters the image it finds (datasheet 5.9.5). One word of that block
names the architecture, and flashing an image whose word names the
other one makes the bootrom reset both cores into it: no button, no OTP
write, no second board. Measured on the bench chip in both directions:
ARCHSEL_STATUS (0x4012015c) reads 0 after an Arm image and 3 after a
RISC-V one.

That is why the bench manifest names the same physical board twice,
once per architecture (`weact2350b` and `weact2350b-rv` in
[../boards/README.md](../boards/README.md)'s board types): `brio flash`
takes a board type and the board type carries the preset.

An app is written once. It names `brio::Rp2350Platform<>`,
`brio::Ticker` and `brio::Clock<...>` and binds its vectors by the
crt's names, `isr_systick`, `isr_uart0`, `isr_dma_0`, and those names
are bound on both halves: the Arm crt puts them in a Cortex-M vector
table, the RISC-V crt binds the same symbols to the machine timer trap
and to Hazard3's own dispatch.

## Toolchains

Two, both self-built under `/sw`, both named by absolute path:

- **arm-none-eabi-gcc 16.2** at `/sw/arm-none-eabi` for the Cortex-M33
  half, with the hard-float ABI (`-mcpu=cortex-m33 -mthumb
  -mfloat-abi=hard -mfpu=fpv5-sp-d16`, the `thumb/v8-m.main+fp/hard`
  multilib); the crt enables CP10 and CP11 before `.data` is touched, as
  on the STM32F4.
- **riscv32-unknown-elf-gcc 16.2** at `/sw/riscv32-unknown-elf` for the
  Hazard3 half, `-march=rv32ima_zicsr_zifencei_zba_zbb_zbs_zbkb_zca_
  zcb_zcmp -mabi=ilp32`. Upstream RISC-V, not the WCH compiler the two
  QingKe families use: this core carries no vendor extension, and the
  `-march` string sits in a cache variable because the toolchain also
  has the multilib without Zcb and Zcmp, and what those two cost or save
  on real images is a measurement this stratum still owes.

What is ARMv8-M and not Raspberry Pi's comes from the `cortexm/` core
stratum (`brio/rp2350/core_m33.hpp` is the device header and then
`cortexm/nvic.hpp`, [../cortexm/README.md](../cortexm/README.md)); what
is RISC-V and not Raspberry Pi's is `brio/rp2350/core_hazard3.hpp`,
which is this stratum's own until a second RISC-V family of this shape
earns a core stratum. `core.hpp` is the one file that asks which
processor is in the socket, and both halves export the same names, so
nothing above it knows.

The device description is vendored from the pico-sdk at tag 2.3.1
(`third_party/pico-sdk/rp2350/`, BSD-3-Clause), in its own include root
because the RP2040's files have the same names. On the RISC-V build the
device header's unconditional `#include "core_cm33.h"` resolves to a
stub of this project's own (`third_party/pico-sdk/rp2350/no_core/`): the
register map, the instance pointers and the IRQ numbers are the chip's
whichever core reads them, the system interrupt numbering being shared
between the architectures (3.8.4.2), and only the core description is
not.

## Build

`rp2350/` is its own CMake project, the only one whose axis is the
compiler; the repository root is not a CMake project, so every CMake
command runs from `rp2350/`. There are four presets and not two,
`rp2350-arm-release`, `rp2350-arm-debug`, `rp2350-riscv-release` and
`rp2350-riscv-debug` (CMake Tools shows them as `RP2350 on its
Cortex-M33 pair, 16 MB flash, QFN-80 (release, -Os)` and the Hazard3
and debug variants), each naming its toolchain file and setting
`RP2350_ARCH`:

```bash
(cd rp2350 && cmake --preset rp2350-arm-release)                      # configure (once, or after adding an app)
(cd rp2350 && cmake --build --preset rp2350-arm-release --target <app>)
(cd rp2350 && cmake --build --preset rp2350-riscv-release --target <app>)   # the same source, Hazard3
(cd rp2350 && cmake --build --preset rp2350-arm-release --target <app>-upload)
brio check rp2350 [name]                                              # family smoke, both compilers, no hardware
```

What a configure targets, besides the architecture, is a flash geometry
and a package: `RP2350_FLASH_KB` picks the linker script, and
`RP2350_PACKAGE` (`a` for the QFN-60, `b` for the QFN-80) becomes the
define the stratum reads its bank size from, so a pad the package has
not got is a compile error, and an image built without the define
compiles for the larger package and refuses the extra pads at run time
against SYSINFO's PACKAGE_SEL.

**There is no second-stage bootloader**, unlike the RP2040: the bootrom
sets the XIP interface up itself while scanning the flash. It tries
EBh, BBh, 0Bh and 03h reads at SCK divisors 3, 6, 12 and 24, sixteen
attempts (5.2.7's table 452), and keeps the first that works, so an
image begins with its own first byte, a vector table on Arm and one
jump instruction on RISC-V, and what marks it an image is the
twenty-byte IMAGE_DEF block the crt places right behind that, well
inside the first 4 kB the bootrom reads. On the bench board that is the
first of the sixteen, EBh quad-I/O at CLKDIV 3, 50 MHz of SCK once a
program has raised clk_sys to 150 MHz; how a program keeps it across a
flash operation is [flash.md](flash.md).

Applications are discovered at every configure from
`rp2350/src/apps/*.cpp` by the `// build:` lines in each file's header
comment, with `weact2350b` and `weact2350b-rv` as the board names. Build
outputs land in `build-cmake/rp2350-{arm,riscv}-{release,debug}` at the
repository root: `<app>.elf`, `.bin` and `.hex`, `firmware-<app>.map`
and `<app>.lst`, the source-interleaved disassembly.

## Upload (the Raspberry Pi Debug Probe, and the three OpenOCDs)

The board is reached by a Raspberry Pi Debug Probe (CMSIS-DAP v2, the
USB bulk backend,
[../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)):
port D on the four-pad SWD header, port U crossed onto GP0 and GP1 for a
console when a program has one.

Three OpenOCD builds live under `/sw`, and this target uses the third:

| Build | What it is for |
|-------|----------------|
| `/sw/openocd` (the 0.12.0 release) | every SWD path of the other targets; it does not know this chip |
| `/sw/openocd-git-bedefa2` | the RP2040 boards whose flash chip the release does not know; it can attach to an RP2350's Arm cores but cannot debug Hazard3 (its RISC-V driver takes no DAP) |
| `/sw/openocd-rpi-acff23f` (Raspberry Pi's fork) | this target. It examines all four cores (`set USE_CORE cm0` or `rv0` before `-f target/rp2350.cfg`), serves gdb on either, and flashes from either side; the project's `RP2350_OPENOCD` cache variable names it |

The fork warns that the probe's firmware is old and falls back to a
"low-performance workaround"; that is cosmetic here and no probe is
updated for it.

**The flash verb, and why it is what it is.** `brio flash <board>
<app>` runs two OpenOCD sessions, and the pair is the state-independent
verb of this target:

1. **The rescue**, over the debug port and nothing else: a DAP with no
   target behind it, RESCUE_RESTART set and cleared in the RP-AP's
   control register (3.5.8). It works "even when system clocks are
   stopped and the switched core power domain is powered down" and
   leaves the chip halted in the bootrom, in the Arm architecture, with
   the clock tree at its reset state and the SRAM gone. The target
   script's own rescue path is not usable for this: it examines a core
   first, and a core that cannot be examined is exactly the case a
   rescue is for.
2. **The programming**, as core 0 of the Arm pair, which after the
   rescue is what is running whatever the image in the flash named,
   then `reset run`, which hands the chip back to the bootrom and so to
   the new image's own architecture. After an Arm image the session
   ends by taking halting debug back down (DHCSR with its key and every
   control bit clear), so that a `break_here()` faults instead of
   halting the core; there is no such write after a RISC-V image, that
   register belonging to a core which is not the one running.

So a board is recoverable from any state a program can put it in, with
no button and no replug, which is what makes the sleep chapters of this
target safe to write. One state the rescue does not reach: a chip whose
switched core is powered down ([powman.md](powman.md)).

## Debugging

The debugger is the same fork with a gdb per architecture:
`/sw/arm-none-eabi/bin/arm-none-eabi-gdb` for the M33 half and
`/sw/riscv-gdb/bin/riscv32-wch-elf-gdb` for the Hazard3 one (a generic
build under that name, whatever its prefix says). The fork reads memory
under either architecture without halting, which is how "First light,
measured" below read `blink`'s globals. `.vscode/launch.json` has no
entry for this target, and no editor session has been run on it.

## Serial console

The Debug Probe's UART bridge is the console: its port U, crossed onto
GP0 (TX) / GP1 (RX), which is UART0, enumerates as a CDC port under the
probe's own USB serial, so the console is addressed by
`/dev/serial/by-id` and never moves with the socket. The board's own
USB-C is a second console's road, through the chip's own USB device
controller ([usb.md](usb.md)).

## Family coverage

One chip in two packages and two architectures: `brio check rp2350` is
the one fixture that crosses two compilers. Every positive translation
unit of `test/family_rp2350/` is built four times, for the Cortex-M33
and for Hazard3, each for the QFN-80 and for the QFN-60, and every
`neg/` TU, a program that must be refused, must fail; `util_all.cpp`
goes through both compilers. The bench proves the QFN-80 on both
architectures; the QFN-60 has the compile check alone ("Not covered
yet" below).

## What the silicon does to a program, before any driver

Four facts shape every chapter of this stratum, and all four are
measured on the bench chip:

- **`reset run` is a processor reset, not a chip reset.** It resets the
  cores and leaves the clock tree, the pads and the peripherals exactly
  as the previous image left them, so an image may start on a PLL it
  did not lock. Nothing in this stratum assumes a reset state it did
  not make itself: `Clock::init()` walks clk_sys onto clk_ref and
  clk_ref onto a ring oscillator it started, before it touches a PLL.
  Measured: with the program running on the system PLL at 150 MHz, a
  `reset run` restarts it and it reaches 150 MHz again with the tree
  never having stopped. (A rescue, a watchdog event or a power cycle do
  reset them.)
- **The pads come up isolated.** PADS_BANK0's reset value is 0x116: the
  ISO latch set, the input buffer disabled, a pull-down on. While the
  latch stands the pad is cut off from the digital logic in both
  directions, so a pad configured as on the RP2040 would do nothing at
  all. Every configuring verb of `pin.hpp` writes the pad with ISO
  clear.
- **The bit maps are not the RP2040's.** IO_BANK0 and PADS_BANK0 sit in
  RESETS bits 6 and 9 here (5 and 8 there), the reset controller
  governs twenty-nine blocks against twenty-seven, the clock generator's
  dividers are 16.16 where the RP2040's were 24.8, there is no clk_rtc
  at all, and the frequency counter's source numbering differs. Nothing
  of this chip is retyped from the other one: every constant comes from
  this chip's own headers.
- **Erratum RP2350-E9 is live on stepping A2.** With its input buffer
  enabled, a pad that nothing drives leaks enough current through the
  input stage to sit high against its own pull-down; a pad driven low
  and then released stays low. An idle level on this silicon is history,
  not a measurement: a program that wants to read a floating pad through
  its pulls must drive it first, or use the pull-up and read a low as
  its signal. Stepping A3 removes the leakage path.

And two more the RISC-V half adds:

- **`wfi` ignores mstatus.MIE** (3.8.1.23) and respects every other
  interrupt control, so the kernel's idle path (mask, look at the
  queues, sleep, unmask) has no lost-wakeup window here, exactly as on
  the Arm half. This is not the QingKe cores' behaviour, where the same
  sequence deadlocks.
- **Hazard3's clock-gated sleep is never armed.** MSLEEP.DEEPSLEEP
  would stall a debugger's system-bus reads on core 1 until that core
  wakes (erratum RP2350-E4), and the datasheet's own note is that the
  saving over a plain `wfi` is minimal.

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
0x052 (the isolation latch dropped, the input buffer on, 4 mA) and
FUNCSEL reads 5, SIO.

The last row is the whole interrupt path on a line that reaches no
hardware (one of the six spare ones, 3.2): enabled in the controller,
raised in software, served by a handler an app bound by name,
`isr_spare_0`, which is the slot `isr_irq46` on both halves, through the
Arm vector table on one and through Hazard3's `meinext` dispatch on the
other.

The awake time is under the microsecond ruler's resolution in nearly
every call on both halves. A run in which one wake covers two ticks
shows fewer `idle()` calls than ticks: that is the measuring loop's own
race between the tick it reads and the sleep it then enters, not a
difference in the sleeping.

## Not covered yet

Driver gaps, each with its reason:

- **The HSTX** (12.11), born with its first user. It is an output-only
  streaming serialiser for a display or a fast link, eight pads, a
  command expander and a bit crossbar behind one FIFO a DMA channel
  feeds, and the display link that would use it is not in this project
  yet. No instrument is missing: a PIO block reads a pad another
  function drives ([pio.md](pio.md)), so its suite can sample the
  block's own pads with no wire.
- **The Cortex-M33's coprocessors** (3.6), declined as drivers. They
  exist on one of this chip's two processor architectures, and this
  stratum is written once for both. The GPIO coprocessor (3.6.1) is a
  shorter road to the same bank verbs SIO gives both halves; the
  double-precision one (3.6.2) belongs under the compiler's arithmetic
  routines and not in a peripheral driver, and would come with a
  program that measures double arithmetic as its bottleneck; the
  redundancy coprocessor (3.6.3) is the bootrom's own hardening against
  fault injection, whose failed assertion halts the processor, nothing
  a framework hands out as a verb.
- **The programming side of OTP**, in any form: declined permanently
  ([otp.md](otp.md)), and the absence is asserted by the family check
  rather than promised.
- **The QFN-60 package.** The stratum compiles for it and refuses its
  absent pads, but no QFN-60 part is on the bench: everything below the
  compile check is untested there.
- **Interrupt priorities.** Both halves have sixteen levels and no
  driver uses them: the kernel's promise is that no interrupt nests over
  another, and here it is kept structurally (every line at the reset
  priority on the M33, mstatus.MIE never set inside a handler on
  Hazard3); the one priority written is PendSV's, below every line, by
  the preemptive kernel on the M33. The two conventions run opposite
  ways (numerically higher is more urgent on Hazard3), which is why
  `Irq` carries no priority verb at all.

Implemented but not bench-verified, each with what would measure it:

- **The QFN-60's compile-time refusals**, above: a QFN-60 board.
- **A second console for core 1.** Core 1 itself is measured on both
  architectures (the launch, the two tickers, the bridge, the panic and
  the relaunch, [multicore.md](multicore.md)), but it speaks only
  through the bridge, there being one probe and one UART on this board;
  the chip's own USB CDC port would be the second console.
- **`Mtime` and the tick generators under a clk_ref that is not
  12 MHz.** The arithmetic refuses a clk_ref that is not a whole number
  of megahertz, and only the 12 MHz crystal has been on the wire.
- **The Arm crt's four configurable fault vectors.** They have their
  names in the table and their weak spins, and nothing has raised one
  of those four. The entry `fault_reset()` binds to is measured on both
  halves, `isr_hardfault` on one and `isr_riscv_exception` on the other,
  reached by a breakpoint instruction with no debugger attached
  ([reset.md](reset.md)).

## The documents

One document per peripheral driver, in the shape
[../README.md](../README.md) prescribes: the reader's orientation, the
documents of record, what the silicon does, the types and verbs, how to
use it, the bench findings and, where the driver has gaps, "Not covered
yet" as two lists. The line beside each names what the document is
about and the finding or two most worth knowing before using the
driver; the findings themselves, with their figures and conditions, are
in the document.

| Document | Content |
|----------|---------|
| [platform.md](platform.md) | The platform on two instruction sets: one platform type per core and one for both architectures, the idle path and the lost-wakeup window it does not have (two rules, one promise), the microsecond ruler that is also the Hazard3 half's timebase, the subsystem reset controller, the atomic register aliases, the panic breadcrumb across a processor reset, erratum RP2350-E9 shown on a pad, and the two registers called PLATFORM; a handler in SRAM on both halves: `.fast_text`, `places_fast_text`, the vectors placed or the drivers given InRam, measured warm and cold, the placement buying the cold path; on Hazard3 the crt's dispatch stays in the flash; the preemptive kernel's platform on both halves, measured by `test_rp2350_rubato`: the core stratum's PendSV and SVC on the Cortex-M33 with the ARMv8-M facts it rests on checked, road 1 in 113 cycles from the interrupt's last statement to the preempting dispatch's first, a quiet post 60, a road-2 post 319, FPCCR read as ASPEN 1 and LSPEN 1 where the register table says 0, the FP registers bit-exact across a preemption; the machine software interrupt through the crt's trampoline on Hazard3, its entry leaving by an `mret` of its own, road 1 in 188 cycles, a quiet post 54, a road-2 post 346, one to two stores retiring behind the raising one before the trap |
| [clock.md](clock.md) | The clock tree: four roots where the RP2040 had three (the low-power oscillator joins clk_ref), a generator more and none for an RTC, the 16.16 dividers, the tick generators as a block of their own, the ring oscillator's randomiser, the PLLs' lost lock, the resus circuit, and two CTRL registers that are passwords, where a masked write is a refused write |
| [pin.md](pin.md) | GPIO: the pads that come up isolated and the latch that holds them, the four overrides and the STATUS behind them, the pin interrupts with their summary registers and the forced event that bypasses the enable, and erratum RP2350-E9 measured four ways |
| [uart.md](uart.md) | The UART: what this chip owes the PL011 that its predecessor did not, a second function column that makes every group's flow-control pads a second data pair, pads that come up isolated, a DREQ table with not one row in the old place, and one interrupt name bound on both architectures; on the pair's wires against the RP2040's `uart_peer`, an echo of 512 bytes in the two crystals' common time and the error attribution of 7E1 under 8N1 |
| [spi.md](spi.md) | The SPI: the PL022 this chip carries at the same revision as its predecessor, over a pin table with one function column where the UART gained a second, and the one sentence of 12.3.1 that the rest of the chapter contradicts five times over, the transmit pad's output enable, which the pads answer on the pair's wire (released while deselected and under SOD: 12.3.1 holds); the host against the partner's PIO client exact to clk_peri / 14, the engines on the wire, this chip's client under the partner's frame-signal host, and a rare glitch on the select at /16 of the partner |
| [i2c.md](i2c.md) | The I2C: the Synopsys block this chip carries unchanged, register for register, and the five things around it that are not: one function column over forty-eight pads where the UART has two, pads that come up isolated, a bank of two words that an open-drain drive by hand must tell apart, a DREQ table and a reset controller whose bits have all moved, and a bus clear by hand paced by the one ruler both architectures have; measured on the wire against the desk partner as the peer |
| [timer.md](timer.md) | The two system timers: a 64-bit counter of microseconds each, four alarms with an interrupt line apiece, the tick generator of 8.5 behind each one, and the two registers this chip added, the counter taken off the tick and onto clk_sys, and a lock that refuses every write until the block is reset |
| [watchdog.md](watchdog.md) | The countdown and the four scratch registers a program may use: the tick that now comes from the TICKS block, the RP2040's double decrement that is not this chip's, the three WDSEL registers in their three tiers, and erratum RP2350-E19's guard before every reboot |
| [reset.md](reset.md) | Chapter 7 whole: the three tiers, the causes recorded in the always-on power manager beside the watchdog's REASON, the power-on state machine, the subsystem controller, the reboot both architectures have and the processor reset only one of them has |
| [dma.md](dma.md) | The DMA: sixteen channels and four interrupt lines, a MODE in the top nibble of the transfer count that makes a channel re-arm itself or run forever, an address step that can go backward or by twos, the abort that answers erratum RP2350-E5, a security level on every resource; the engines: the two a transport names in its slots, the copy and fill engine over a channel in memory-to-memory, each binding its channel once and starting a block with two stores, the second the trigger; the level as one bit, a receive engine HIGH by default, and here two HIGH_PRIORITY channels starve a normal one; what a released requester leaves the next owner |
| [pwm.md](pwm.md) | The PWM: twelve slices where the RP2040 had eight, the pin map's second half on GPIO 32..47 and what the four highest slices are in the package that bonds none of those pads, a second shared interrupt line with a register set of its own, and a fractional divider whose last step is a whole 256 |
| [pio.md](pio.md) | The programmable I/O, chapter 11: three blocks of four machines over one instruction set, and the six things this chip added to it: a version field, a window that says which thirty-two pads a block can see, a CTRL write that reaches the blocks either side of it, all eight flags on the interrupt lines, a masked input count, and a receive FIFO that can be four registers instead of a queue |
| [adc.md](adc.md) | The ADC: one converter over five channels or nine, because the package decides the input map and the temperature sensor's channel number with it; a four-bit AINSEL and a nine-bit round-robin where the RP2040 had three and five; no ADC_VREF pin, the converter's own supply being its full scale; the RP2040's differential-nonlinearity spikes gone; and an analog claim that must drop the pad's isolation latch for its pull to exist at all, which is also the one configuration erratum RP2350-E9 does not bite |
| [sha256.md](sha256.md) | The SHA-256 accelerator: the compression function in hardware and nothing else, the padding software owes it, the byte swap that reconciles a little-endian bus with a big-endian standard, the handshake and the flag that catches a missed one, and the constexpr twin the silicon is judged against |
| [trng.md](trng.md) | The true random number generator: a ring oscillator with no tie to the clock tree, three entropy checks of which one is fatal until the block is reset, a generation time that is not deterministic, and the raw-sample path the bootrom takes instead |
| [otp.md](otp.md) | The one-time programmable array, read side only and deliberately so: four read windows of which two fault instead of lying, page locks that only climb, the error correction decoded in software with the one sentence of the chapter that must be read carefully, and the flags a program must never set, printed and not written |
| [bootrom.md](bootrom.md) | The mask ROM's public function table: two sets of well-known words, one per architecture, and two different lookups over them; what is wrapped, and the three entry points that deliberately are not |
| [usb.md](usb.md) | The USB device controller: the RP2040's block with one new duty, the PHY's isolation latch, set at reset and lifted last of all, three reset values moved under it, two pads that are bank 1's and could be GPIO, an erratum that makes the system clock's rate part of the contract, and a shelf of diagnostics that are a report and never a path |
| [flash.md](flash.md) | The external quad-SPI chip and the two blocks between it and the bus: a memory interface with two windows, a transfer described phase by phase, four translation panes a window and a direct mode that disconnects them all; a cache whose flush is an address and not a register, with one erratum in its clean sweep; and an engine whose way back into execute-in-place is the bootrom's own function out of boot RAM, where its predecessor had a second stage |
| [multicore.md](multicore.md) | The second core: a kernel of its own on either instruction set, the doorbell registers this chip has and its predecessor had not, which free the mailbox FIFO for the launch alone, one bell line with one number for both cores, the bootrom's six-word protocol and the entry shim that gives core 1 what the ROM does not hand it |
| [powman.md](powman.md) | The always-on block: five power domains and twelve states of which eight have no processor in them, a password on every register and a read of it that does not return the password, the core regulator and the brown-out detector decoded and never written, the eight scratch words that are the only state a power-down keeps, and the millisecond timer in the same block, whose divisor is a number software writes and which therefore has to be told what the low-power oscillator really runs at |
| [sleep.md](sleep.md) | The three depths a program can arm: a plain sleep instruction, the SLEEP state that is the program's pruning set and no processor bit at all, and DORMANT, whose alarm needs the reference clock left running, which is what makes both sites move clk_sys and clk_ref apart before the keyword; the always-on timer as alarm and witness on every rung, and the memory power-down states kept off the ladder because leaving one is a boot |
| [vendor/README.md](vendor/README.md) | The RP2350 datasheet by build, the hardware design guide, the vendored pico-sdk subset and the SVD, the bench chip's identity, the errata this stratum answers |
