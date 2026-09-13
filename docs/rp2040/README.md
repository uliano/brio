# Target: RP2040 (`rp2040/`)

The operational page for brio's RP2040 target: Raspberry Pi's chip
with two ARM Cortex-M0+ cores, an ARMv6-M family that shares the
`armv6m/` core stratum with the SAM C21 and the STM32G0. brio runs
on core 0 today - one kernel, one SysTick, the other core asleep in
the bootrom - and the second core, the design point the stratum was
opened for, runs a kernel of its own (design/kernel.md section 12,
[multicore.md](multicore.md)): two kernels, one per core, and the
inbox bridge between them. `kernel/` and `util/` compile here as
written (`test/family_rp2040/util_all.cpp` is the whole of both over
the platform).

Peripheral documents live next to this page, one per chapter - the
map below; the documents of record are in [vendor/README.md](vendor/README.md)
together with the errata pass and the bench chip's identity (B2
silicon, SYSINFO.CHIP_ID read over SWD).

## The documents

One document per peripheral driver, in the shape
[../README.md](../README.md) prescribes; the vendor page carries the
documents of record.

| Document | Content |
|----------|---------|
| [platform.md](platform.md) | Platform (RP2040): `Rp2040Platform<TB>` on the SysTick `Ticker` (1000 Hz), the PRIMASK critical section that is PER CORE, WFI idle; the boot chain (bootrom, the 256-byte second stage, the vector table at flash offset 0x100 the stage vectors into), the crt with the pico-sdk's handler names and THE MACRO TRAP behind them, the two unstriped SRAM banks as the two cores' stacks; the reset controller that gates every peripheral; SYSINFO's chip id; the suite's measures (the tick to 5 ppm, the cold XIP cost of a first call, .noinit through every reset) |
| [reset.md](reset.md) | Reset (RP2040): three kinds of reset with three signatures - chip-level (HAD_POR, HAD_RUN, HAD_PSM_RESTART, standing for the life of the supply), watchdog (REASON, the last event), the core's own (SYSRESETREQ: ONE CORE through the bootrom, no mark) - so the causes word is a history read by comparison, and the chip's reboot is the watchdog's trigger; `Reset::software()` (the chip), `Reset::core()`, `ResetReporter`, `fault_reset<P>()` |
| [dma.md](dma.md) | DMA (RP2040): twelve channels with their trigger aliases, the request as a FIELD any channel takes (table 119, credit-based), two interrupt lines (one per core by convention), chaining, address wrapping, pacing timers, the checksum sniffer, the abort with erratum E13's workaround, progress from TRANS_COUNT (E12); the two engines in a UART's slots - the suite's console prints through one - and the two traps of a receive engine |
| [i2c.md](i2c.md) | I2C (RP2040): the Synopsys DW_apb_i2c behind `DwApbI2c<n>` - a COMMAND FIFO whose entries carry the RESTART and STOP bits, an empty FIFO with no STOP holding SCL, one TX_ABRT with the reason decoded, the SCL counts with the cycles the block adds subtracted so the asked rate is the measured one -, the `I2cHost` engine with the other strata's Request (the probe as a one-byte read: no address-only entry exists), the engines serving a read phase (the request a level the DMA spends in bursts), `I2cClient` on RD_REQ (the stretch is the block's own); measured: 99 / 391 / 960 kHz, the timed bus at the arbiter's 20 ms, the flush at the host's NACK |
| [adc.md](adc.md) | ADC (RP2040): one 12-bit converter behind `Adc` over four pads and the temperature sensor - its own 48 MHz clock from the USB PLL (or the crystal, a conversion of 96 of its cycles either way), the one-shot start, the free run paced by a 16.8 divider whose period must exceed the conversion (DIV 95 halves the rate, measured), the round-robin, the eight-entry FIFO as one interrupt and one DMA request, the eight-bit shift, the error flag per entry -, `AnalogIn<Pin>` on GPIO 26..29, util/analog_sampler.hpp over it; measured wireless: the sensor, the pads through their pulls (the keeper is no keeper on an analog pad), 2.003 us a conversion, the FIFO overflowing 16 us after a block ends unless the converter is stopped from the completion |
| [flash.md](flash.md) | Flash (RP2040): the external quad-SPI chip the core executes out of, erased and programmed through THE BOOTROM'S OWN FUNCTIONS behind `Flash` - every operation a window with the flash disconnected, run from `.ram_text` with interrupts masked and closed by re-entering the second stage's copy in SRAM -, `command()` for the chip's ids (9Fh, 4Bh, 05h), `Xip` the cache with its counters; `QspiFlashPartition` the constant top 64 KB the linker never reaches, `QspiFlash` and `QspiFlashJournalZone` the two FlashMedia (4096 / 256) for the heap and the journal; measured: the stage returns to a caller, a page takes a second program, a 5 ms window costs the ticker its ticks but one |
| [pio.md](pio.md) | PIO (RP2040): two blocks of four state machines behind `Pio<n>` and `PioSm<n, sm>`, the nine instructions as constexpr encoders with the side-set and delay folded in, a program relocated at load, the FIFOs as interrupt sources and DMA requests, the eight flags and the two lines; the chapter's own programs as tasks (a serial port on any pin, a square wave, a PWM that is a PwmChannel); measured on the wires: 64 bytes at 3 Mbaud, a 31.25 MHz wave, the PWM's duties, a one-instruction sampler as a logic analyser |
| [pwm.md](pwm.md) | PWM (RP2040): eight slices behind `PwmSlice<n>` - a 16-bit counter with a wrap value and two levels, the 8.4 divider, phase-correct mode, the B pin as the counter's gate or clock (a duty or a frequency measured with no capture unit), the wrap as one interrupt bit and one DMA request, the global enable for lockstep, the phase nudges -, `configure()` from scratch because a slice reconfigured while running keeps its counter (measured: half a millisecond of silence, an output stuck); the tasks `PwmOutput` (a PwmChannel, max = TOP + 1), `PwmPair` (a dead time by arithmetic), the edge and level counters, `PwmPeriodicTick`; measured on two wires between the chip's own slices |
| [rtc.md](rtc.md) | RTC (RP2040): a calendar of seven binary fields behind `Rtc` - clk_rtc from the crystal over 256 with CLKDIV_M1 for the second, a leap year of "divisible by four" with the bit for the century years, one alarm on any subset of the fields (a level: the ISR body masks the line and disarms), the day of the week the silicon does not compute -, `RtcDateTime` and the calendar arithmetic; measured: the enable's own tick (a set value reads one second on unless undone, which set() does), the 1.000 s second, eight boundaries including 2100, the alarm's 37 us latency |
| [spi.md](spi.md) | SPI (RP2040): the ARM PL022 behind `Pl022<n>`, the `SpiHost` engine with the other strata's Request (a GPIO select held for the tenure, the pump keeping eight frames in flight through the FIFOs, the engines on any two channels, the loop-back as the instrument) and `SpiClient` framing on its select pad; measured: 62.5 Mbit/s polled, the client's clk_peri / 12 ceiling, ONE frame per select window in modes 0 and 2, SOD not releasing the pad |
| [sleep.md](sleep.md) | Sleep (RP2040): the SLEEP state - reached on a plain WFI with core 1 in the bootrom (measured, SLEEPDEEP or not), the top-level gates pruned to SLEEP_ENx, SysTick counting through it - and DORMANT on either oscillator (the keyword, a GPIO event or the RTC the way back, the ring oscillator dormant with the crystal keeping the calendar as the dormant with a deadline), behind `Rp2040SleepSite` and `Rp2040TimedSleepSite` (the timer as alarm and witness, the calendar for a dormant) and the platform's `sleep_hook`; `DormantWake`, the gate sets; measured: a two-second dormant the timer never saw, the manager's rounds through the kernel's own idle |
| [multicore.md](multicore.md) | Multicore (RP2040): two kernels on two cores - `Rp2040Platform<core>` and `CoreTicker<core>`, the SIO FIFO doorbell behind `util/inbox.hpp`'s bridge, the bootrom's launch protocol and the power-on state machine's reset of core 1 (`Core1`), the mispost check on the queues; measured: a crossing and back in 10 us, fifty thousand a second each way lossless, the accounts balancing under saturation, the probe trap that halts core 1 and freezes the timer |
| [watchdog.md](watchdog.md) | Watchdog (RP2040): the tick generator the system timer counts too, the 24-bit countdown that decrements TWICE per tick (erratum E1, 8.3 s of reach), the PSM selection that makes a time-out a reboot through the bootrom, ENABLE that clears, `Scratch<0..3>` surviving every reset but the chip-level one (the upper four are the bootrom's, refused) |
| [timer.md](timer.md) | Timer (RP2040): the 64-bit microsecond counter on the watchdog tick, the raw-pair read for any context and the latching pair for one, four alarms on the low word each with its own line, DBGPAUSE set at reset; `Timer::now()`, `alarm_in`, the ruler of every timing verdict on this target |
| [clock.md](clock.md) | Clock (RP2040): the FOURTH clock model - no bus prescaler and no enable bit per peripheral, but a generator per clock domain with an aux multiplexer and a divider, a glitchless mux in front of the two that must never stop, and a separate clk_peri so the UARTs keep their rates (`PeriSource::crystal`, measured); the boot on the ring oscillator, the crystal's startup delay, the system PLL's exact-ratio search at compile time (125 MHz = 1500 MHz / 6 / 2), `init()` as the rate switch (~200 us), the switching sequences of 2.15.3.2 as code; the frequency counter as the measure, the GPIO clock outputs and inputs, the ring oscillator's start and stop |
| [pin.md](pin.md) | GPIO (RP2040): thirty pins of one bank owned by THREE blocks - IO_BANK0's function select, PADS_BANK0's electrical setup with the bus-keeper mode, SIO's single-cycle word-wide path - both blocks in reset at power-up and released by every configuring verb; the pad's reset state (input enabled, pull-down); `Pin<n>` with no port letter |
| [uart.md](uart.md) | UART (RP2040): the ARM PL011 behind `Pl011<n>` and the `Uart` task - 32-deep FIFOs storing the error flags with each byte (the overrun read from UARTRSR, a live bit in the entries), a transmit interrupt that is a TRANSITION and not a level (so the task pends its own line to start a transmission), the receive timeout against the level (measured: 371 us for a lone byte, the sixteenth frame for a burst), the 16-bit + 6-bit divisor off clk_peri from 120 baud to 7.8125 Mbaud, the loop-back as the instrument, pins fixed per instance by table 279 |
| [vendor/README.md](vendor/README.md) | The RP2040 datasheet by build, the Pico datasheet and the hardware design guide, the vendored pico-sdk subset and the SVD, the bench chip's identity, the errata pass |

## Toolchain

Self-built **arm-none-eabi-gcc 16.2** at `/sw/arm-none-eabi` - the
same compiler, flags and linker discipline as the samc21 and stm32g0
projects (`rp2040/cmake/toolchain-arm.cmake` is that file verbatim:
`CMAKE_SYSTEM_NAME Generic`, `STATIC_LIBRARY` try-compile,
`--specs=nano.specs -nostartfiles`, deliberately NO syscall stubs so
an accidental `_sbrk`/`_write` fails the link). What is ARMv6-M and not
Raspberry Pi's lives in the `armv6m/` core stratum, so `nvic.hpp`,
`ticker.hpp` and `delay.hpp` here are the device header plus that core
file plus this family's own facts.

The device description is vendored, from the pico-sdk at tag 2.3.1
(`third_party/pico-sdk/`, BSD-3-Clause): the CMSIS device header
`RP2040.h` generated from the chip's SVD (the register blocks, the
instance pointers, the IRQn enumerators, `core_cm0plus.h` from the
shared `third_party/cmsis-core/`) and the `hardware/regs/*.h` bit-field
headers generated from the same SVD - the two halves of one map,
because the CMSIS header names no field. Nothing of the SDK's runtime
is built: brio has its own crt, linker script and drivers, and the
SDK is read as a reference and cited. One chip, one package, no
device-select define: `brio check rp2040` compiles every positive TU
of `test/family_rp2040/` once and requires every `neg/` TU to fail.

## Board and build

Two boards are on the bench: a **Raspberry Pi Pico** ([../boards/pico.md](../boards/pico.md)),
the reference board - the Winbond W25Q16JV flash the SDK's second
stage was written for, the recommended crystal, the documented power
path - and a **WeAct RP2040 board** ([../boards/weact-rp2040.md](../boards/weact-rp2040.md)):
an RP2040 B2 behind a 2 MB Zetta ZD25Q16 quad-SPI flash, a 12 MHz
crystal, USB-C, BOOTSEL and NRST buttons, a user LED on **GP25**, a
user button on **GP23**, every user GPIO on its headers and a 4-pin
SWD header, running at **3.3 V**. Verified at the bench: the chip id
and both debug ports over SWD, the image booting through the
second-stage bootloader into the vector table at flash offset 0x100,
the crystal and the PLL locked at 125 MHz, the pin-check wave on
GP2..GP29 read back through SIO, the console over the probe's UART
bridge in both directions (the kernel console answers HELP, LED,
UPTIME and ERR at 115200 with every error counter at zero), the LED
on GP25 under the console's heartbeat, and every suite of the
document map green on both boards (their documents'
findings; the clock link between the two boards counts one crystal
against the other, the UART cross link carries the serial suite's
peer letters both ways). NOT yet verified: the user button on GP23.

`rp2040/` is its own CMake project, a sibling and peer of the other
five. Apps are auto-discovered from `rp2040/src/apps/*.cpp` - plus
`experiments/*/rp2040/*.cpp` - by their `// build:` header comment.
WHAT A CONFIGURE TARGETS IS A FLASH, not a part: the chip executes in
place out of an external quad-SPI flash whose size (`RP2040_FLASH_KB`,
the linker script) and read protocol (`RP2040_BOOT2`, the second stage
in `src/glue/`) are the board's, so a preset serves every board type
with that geometry - `rp2040-release` serves `pico` (a Raspberry Pi
Pico / Pico H), `picow` (a Pico W: the same flash, four pins the
radio's) and `weact2040` (the 2 MB WeAct board), and an app is built
when its `// build: boards =` line names any of them (default:
`pico`). Every image begins with the 256-byte stage
(`boot2_w25q080.S` for the Winbond-class chips the Pico and the WeAct
board carry, `boot2_generic_03h.S` the slow universal fallback), the
SDK's `boot_stage2` source assembled, padded and CRC'd once and
checked in as bytes with its provenance in its header - no generation
step.

```bash
(cd rp2040 && cmake --preset rp2040-release)                      # configure (once, or after adding an app)
(cd rp2040 && cmake --build --preset rp2040-release --target <app>)
(cd rp2040 && cmake --build --preset rp2040-release --target <app>-upload)
brio check rp2040 [name]                                     # family smoke, no hardware
```

Build outputs land in `build-cmake/rp2040-{release,debug}` at the
repo root: `<app>.elf/.bin/.hex`, `firmware-<app>.map`, `<app>.lst`. A
configure also writes this project's app roster,
`build-cmake/apps_rp2040.json`, which `bin/brio` reads: the board
types `pico`, `picow` and `weact2040` are what tell that tool to build here and
to flash through OpenOCD's CMSIS-DAP interface (`brio flash <board>
<app>`).

## Upload (OpenOCD, the Raspberry Pi Debug Probe)

Flashing goes through OpenOCD driving a **Raspberry Pi Debug Probe**
([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)),
a CMSIS-DAP v2 device on the USB bulk backend (`cmsis_dap_backend
usb_bulk`; it has no HID interface), through `target/rp2040.cfg`: the
multidrop SWD that reaches both cores as two targets, the `rp2xxx`
flash driver that identifies the QSPI chip through the bootrom's
functions, then `program <app>.elf verify`, `reset run` and a write of
DHCSR that clears C_DEBUGEN (a core left with halting debug enabled
HALTS on a BKPT instead of faulting, and every `panic()` ends in one).

WHICH OPENOCD IS A FACT OF THE FLASH CHIP. The 0.12.0 release
(`/sw/openocd`) carries the rp2040 target script and drives the probe,
but its flash driver refuses the WeAct board's Zetta chip - "Unknown
flash device (ID 0x001560ba)" - while a build from OpenOCD's git
(`/sw/openocd-git-bedefa2`, `/sw/src/build-openocd-git.sh`, master at
commit bedefa238) identifies it by name and programs it. The release
build identifies and programs a Pico's Winbond W25Q16JV (measured:
`brio flash` runs the Pico on it with no override). So the bench manifest names the
OpenOCD per PROGRAMMER (`"openocd"` in the entry, `bin/brio`'s
`flash.py`) and nothing else on the desk moves; the project's
`RP2040_OPENOCD` cache variable defaults to the release build. The
git tree also brings `rp2040.cfg`'s RESCUE mode (a debug port that
resets the power-on state machine and halts in the bootrom, for a
firmware that killed SWD) and the SMP mode (`USE_CORE=SMP`, one GDB
seeing both cores as threads). `brio flash` and the `-upload` target
set `USE_CORE 0`: OpenOCD then knows core 0 alone, and never leaves
core 1 debug-enabled, halted or reset a second time - what the
default SMP pair did to a two-kernel program is in
[multicore.md](multicore.md).

Recovery needs no probe at all: BOOTSEL held at power-up (or a
blank flash, which the bootrom treats the same) puts the bootrom's
USB mass-storage loader up as a drive named RPI-RP2, and a UF2
dropped on it is programmed and run - a board cannot be bricked by
an image. Measured: the pin-check image's `.bin`, wrapped into a UF2
(512-byte blocks of 256 payload bytes at their flash addresses,
family id 0xE48BFF56), booted the Pico with no probe attached; a
`brio flash` mechanism for that path is a matter of writing it.

## Debugging (cortex-debug + OpenOCD)

Not yet exercised on this target: the same OpenOCD serves cortex-debug
with the two cores as two targets (or as SMP threads), the SVD in
`rp2040/svd/` for the Peripheral Viewer; the launch entry and the
first session are still owed.

## Editor (clangd)

`brio/rp2040/.clangd` and `rp2040/.clangd` route the stratum and the
project to `build-cmake/rp2040-release`. The repo-root `.clangd`
rules apply unchanged.

## Serial console

The Debug Probe's UART bridge is the console: its port U, crossed
onto GP0 (TX) / GP1 (RX) = UART0 under function 2, enumerates as a
CDC port under the probe's own USB serial, so the console is
addressed by `/dev/serial/by-id` and never moves with the socket.
Console apps run 115200 8N1. Measured: the divisor at 125 MHz is
67 + 52/64 (the datasheet's own example, 115207 baud), the console's
banner, echo and command replies pass in both directions with the
frame, parity, break and overrun counters at zero, and the kernel's
uptime advances with the host's clock over the seconds a session
lasts.
