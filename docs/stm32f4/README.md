# Target: STM32F4 (`stm32f4/`)

The operational page for brio's STM32F4 target: an ARM Cortex-M4F
(STM32F429ZI on the bench, on an ST STM32F429I-DISC1; an STM32F446RE on
a Nucleo-F446RE and an STM32F411CE on a WeAct black pill beside it) -
brio's first ARMv7-M family, which includes the `cortexm/` core stratum
unchanged because SysTick, the NVIC's enables and PRIMASK are the same
programmer's model on both architectures. `kernel/` and `util/` run
here as written: time events pacing a pin, and the full console over
each board's own serial bridge.

Peripheral documents live next to this page, one per chapter - the map
below; the documents of record are in [vendor/README.md](vendor/README.md)
together with the three parts' identities (DEV_ID, REV_ID, the errata
sheet each keys into).

## The documents

One document per peripheral driver, in the shape
[../README.md](../README.md) prescribes; the vendor page carries the
documents of record.

| Document | Content |
|----------|---------|
| [platform.md](platform.md) | Platform (STM32F4): `Stm32f4Platform<TB>` - PRIMASK critical section on a core that HAS BASEPRI and does not use it (the kernel promise kept as on the SAM C21: one priority for every line), WFI idle in Sleep mode, the 16 + 91/97/86-entry vector table with the FPU enabled before a line of C++, SysTick at 1000 Hz, `delay_us`; measured on three parts: the pending tick delivered after an ISB, not inside the unmask |
| [clock.md](clock.md) | Clock (STM32F4): the STM32G0's model with the APB prescalers NO LONGER PINNED - `pclk1_hz`/`pclk2_hz` beside `hz`, `apb_hz(clock, bus)` for a peripheral's own rate - and the way up in the chapter's order: the regulator scale, the PLL, over-drive between the PLL's start and the switch, the flash latency read back; the frequency ladders KEYED ON THE PART CLASS and refused where no manual was read; 180 MHz in over-drive on two boards (an MCO in bypass, a crystal), 100 MHz on the third |
| [port.md](port.md) | GPIO (STM32F4): the STM32G0's register block without a BRR (reset through BSRR's upper half), the port clock on AHB1 and off at reset, INPUT FLOATING as the reset state, AF numbers the datasheets' (AF7 the USARTs, AF8 the UARTs), ports A, B, C and H on every part and the rest by bonding |
| [reset.md](reset.md) | Reset and the watchdogs (STM32F4): RCC_CSR's seven flags as an ACCUMULATING history with PINRSTF raised by every internal source, the IWDG with NO window whose keyed registers do not update until the start key (and whose own reset does stop it), the WWDG on PCLK1 measurable with WDGA never set, the four fault vectors of a Cortex-M4 with the three configurable ones disabled at reset, and the panic breadcrumb across a real reset; the LSI weighed at 33.5 kHz by a watchdog time-out |
| [exti.md](exti.md) | EXTI + SYSCFG (STM32F4): where this family keeps its pin interrupts - sixteen lines numbered by the PIN NUMBER and multiplexed by SYSCFG's EXTICR (another block, another clock gate, closed at reset), the rest peripheral wake-ups whose existence the reserve reads off the PERIPHERALS; one rc_w1 pending bit per line and only while its interrupt is unmasked, the software trigger that needs no pad and does not clear itself, the seven pin vectors and the ISR body that clears first, and an EXTI event out of WFE in 0 us |
| [usart.md](usart.md) | USART (STM32F4): the classic SR/DR/BRR block - flags cleared by read sequences, the divisor in sixteenths or eighths, ten instances at most with the U(S)ART spelling deciding what a FULL one has - the resource over the whole chapter (mute, LIN, IrDA, smartcard, synchronous, flow control, DMA requests) and the `Uart` task with the other strata's surface; the console byte-exact on the three boards at 115200 |
| [rtc.md](rtc.md) | RTC and the backup domain (STM32F4): a BCD calendar in a power domain of its own behind two locks - PWR_CR.DBP over all of it, the RTC's own key over most of it - with RTCSEL one-way and BDRST the way back; the shadow registers and the three errata around them, both alarms, the wake-up timer, the smooth and coarse calibrators and their interlock, the sub-second shift, and the timestamp and tamper that share RTC_AF1 with the two outputs; twenty backup registers that outlive every reset but the domain's own, and the crystal weighed at 32769.18 Hz with a 1 Hz counted on the pad with no instrument |
| [dma.md](dma.md) | DMA (STM32F4): two controllers of eight STREAMS, each with its own FIFO, its own priority and its own vector, choosing between eight request lines with CHSEL - so a peripheral reaches only the one or two (controller, stream, channel) cells the request mapping gives it, and the mapping is a per-part-class fact no header carries; table 49's burst-and-threshold arithmetic refused before the enable, the double buffer swapped in hardware, memory-to-memory on DMA2 alone as the wireless instrument (2048 bytes in 1954 core cycles at 180 MHz), and the two engines the Uart task's slots had been holding empty |
| [tim.md](tim.md) | Timers (STM32F4): fourteen instances of one register block whose geometry is the manual's and not the header's (two 32-bit counters, one break unit per advanced-control timer, TIM9/TIM12 slaving without an encoder, TIM9..TIM14 with no CR2 at all), the counter clock that is HCLK or TWICE its APB clock (and RCC_DCKCFGR's TIMPRE), the rc_w0 status register, four vectors on TIM1 and TIM8 with three of them shared with a small timer, and the two things that make the chapter measurable with nothing attached - the internal triggers and the option registers that put the LSE, the LSI or HSE_RTC on a capture channel; the crystal weighed at 32769.18 Hz and the dead time at 39.9 ns per hundred |
| [adc.md](adc.md) | ADC (STM32F4): up to three converters sharing one PCLK2 prescaler, one reset and one vector - the regular sequence of sixteen ranks and the injected four that PREEMPT it with a signed offset, the four resolutions and their conversion times measured to the ADCCLK cycle, the watchdog whose thresholds are compared before the alignment, the internal channels the MANUAL and not the header numbers, the multi-ADC modes and what really loads their common data register, and the DMA cell each converter's request sits on; VDDA and the junction temperature from the factory measurements, and both errata staged |
| [dac.md](dac.md) | DAC (STM32F4): two 12-bit channels whose outputs reach a PAD and nothing else - which is what lets the ADC read them back with no wire - the holding register and the output register a cycle apart, the buffer measured against a real load, both wave generators with the LFSR's first two words exactly, and a table played by a hardware trigger through the channel's own DMA cell with the underrun a spent stream leaves behind; absent on the F401, F411 and F412, where the resource does not exist |
| [spi.md](spi.md) | SPI and I2S (STM32F4): one block wearing two faces - the F1 lineage's SPI with no FIFO and eight rates off the instance's OWN APB clock, three chip-select arrangements, the simplex and bidirectional line modes, the CRC unit whose polynomial must be ODD, and flags cleared by read SEQUENCES; `SpiHost` with the other strata's Request verbatim over the request mapping's own (controller, stream, channel) cells, `SpiClient` beside it, and the audio face with its own PLL and the I2SDIV/ODD arithmetic; measured against the gyroscope and the display controller a board carries on SPI5 - the rate ladder with the polled pump's constant 240..295 ns a frame, a mode fault raised by SSI alone and visible fifty core cycles later, and the bidirectional read that needs the pad's output stage let go by hand |
| [i2c.md](i2c.md) | I2C (STM32F4): the F1 lineage's EVENT MACHINE - a tenure that advances only when software runs the sequence each flag prescribes, and a RECEIVE PROCEDURE that is three different choreographies by count - over three instances with TWO vectors each and three timing registers (FREQ, CCR under F/S and DUTY, and TRISE, which is the wire's rise time and not a guess); `I2cHost` with the other strata's Request verbatim over the request mapping's own (controller, stream, channel) cells, all of them on DMA1, `I2cClient` beside it, and the errata as code - a controller's BUS ERROR counted and never acted on, SWRST the way back, the repeated start's setup time published where it is at risk; measured against the STMPE811 the board carries on I2C3 - one address of 112 answering, the three receive procedures byte-exact against each other, and SCL counted on its own pad while the DMA carries a 32-byte read. THE TRAP: BUSY stands out of reset over an idle wire, because it watches the peripheral's own inputs and those read low while the pads are not yet in their alternate function |
| [usb.md](usb.md) | USB OTG, device mode (STM32F4): a Synopsys DWC2 core - up to two of them, the full-speed one on PA11/PA12 and the high-speed one's own full-speed PHY on PB14/PB15 - whose whole interface is FIFOS reached through push and pop registers, with one shared receive queue for every OUT endpoint and a slice of RAM per IN endpoint the program maps by hand and the silicon never checks; the address that must be in DCFG BEFORE the status stage of SET_ADDRESS, against chapter 9 and measured; the OUT transfer of several packets that is this core's answer to the NAK a hub charges a frame for; 658 KB/s out and 440 KB/s in on the black pill's own connector, with the kernel console over it |
| [pwr.md](pwr.md) | PWR, the sleep sites and Standby (STM32F4): the mode as two registers in two places (SLEEPDEEP and PDDS), a Stop whose price is four more bits (LPDS, FPDS, the low-voltage pair, under-drive) and whose exit puts the core back on the HSI with the PLL, the HSE and over-drive gone - but the VOS field untouched; `util/power.hpp`'s ladder over it with Standby ENFORCED off it, and the timed site that meets a deadline through a Stop on the RTC's wake-up timer; the flash's 92 us of wake reproducing the datasheet's typical, a 213 us restore to 180 MHz, and a 500 ms event met at 500 ms of wall |

## Toolchain

Self-built **arm-none-eabi-gcc 16.2** at `/sw/arm-none-eabi` - the
same compiler and linker discipline as the other ARM projects
(`stm32f4/cmake/toolchain-arm.cmake` is the stm32g0 file verbatim:
`CMAKE_SYSTEM_NAME Generic`, `STATIC_LIBRARY` try-compile,
`--specs=nano.specs -nostartfiles`, deliberately NO syscall stubs so an
accidental `_sbrk`/`_write` fails the link), with this family's core
flags: `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard` - THE FPU
IS USED, NOT CARRIED: brio has no floats, so the hard ABI costs
nothing today, and a program that wants one pays no ABI break across
every image; the crt enables the coprocessor (CPACR) before the first
C++ instruction because under the hard ABI a float may sit in an s
register anywhere. What is Cortex-M and not ST lives in the `cortexm/`
core stratum, so `nvic.hpp`, `ticker.hpp` and `delay.hpp` here are the
device header plus that core file plus this family's own facts.

The device headers are vendored: `third_party/cmsis-device-f4/` (ST's
cmsis-device-f4 v2.6.9, every F4 part) and the shared
`third_party/cmsis-core/` (with `core_cm4.h`). The device is selected
ST's way, by a plain `-DSTM32F429xx` define that the umbrella
`stm32f4xx.h` dispatches on - no device-specs machinery, so clangd
needs no macro-delta feed. The define's spelling is irregular across
the pack (`STM32F411xE`, `STM32F410Cx`), which is why the project keeps
a PART TABLE (`stm32f4/cmake/stm32f4-parts.cmake`) instead of the G0
project's substring arithmetic.

## Boards and build

Three boards, three parts, no two alike - each with its page in
[../boards/](../boards/README.md):

- **STM32F429I-DISC1** - the bench chip, the superset part (2 MB, FMC
  with an SDRAM, LTDC with a display, ports up to K); 8 MHz crystal on
  HSE, the PLL at **180 MHz in over-drive**; the ST-LINK/V2-B's VCP on
  USART1 PA9/PA10; LD3 on PG13.
- **Nucleo-F446RE** - the second part on the bench, another package and
  the family's extras; the ST-LINK's 8 MHz MCO into HSE in bypass, the PLL
  at 180 MHz in over-drive; the V2-1's VCP on USART2 PA2/PA3; LD2 on PA5.
- **a WeAct STM32F411CE black pill** - the small part (100 MHz, no
  over-drive, one ADC, ports A..C and H); 25 MHz crystal, the PLL at
  **100 MHz**; a standalone STLINK-V3 on SWD with its UART bridge on
  USART1 PA9/PA10; the LED on PC13, lit when low.

All three run at 3.3 V (the DISC1's rail reads 3.0 V). Verified at the
bench, each by its own experiment: the LED pins (the blink app), the
console pads and instances (the console answers on all three), the HSE
modes (a bypass on the DISC1 never sees HSERDY - the MCO route is a
solder bridge left open, UM1670 7.12.1), the whole clock tree as the
registers hold it (the platform suite's letter `e`).

`stm32f4/` is its own CMake project, a sibling and peer of the other
five. Apps are auto-discovered from `stm32f4/src/apps/*.cpp` - plus
`experiments/*/stm32f4/*.cpp` - by their `// build:` header comment,
the other projects' grammar with this family's board names (`boards =
f429zi`, the default; an app that also runs on the other two lists
`f446re` and `f411ce`). One configure targets one part (`STM32F4_MCU`,
the full part number: the table derives the device define, the crt
`src/glue/startup_<header>.cpp` and the board name, the part's last six
characters - one board per part on this desk, so the part IS the board;
the linker script is `ld/<part>.ld`); the three parts have presets.
Every other part of the family is compile-checked by `brio check
stm32f4`, which sweeps every positive TU in `test/family_stm32f4/`
across ALL TWENTY-THREE device headers the CMSIS pack ships and requires
every `neg/` TU to fail for the variants its `// mcu:` line names - see
"Family coverage" below.

```bash
(cd stm32f4 && cmake --preset stm32f429zi-release)                 # the bench chip
(cd stm32f4 && cmake --preset stm32f446re-release)                 # the Nucleo
(cd stm32f4 && cmake --preset stm32f411ce-release)                 # the black pill
(cd stm32f4 && cmake --build --preset stm32f429zi-release --target console)
brio flash <board> console        # OpenOCD over the board's ST-LINK, by serial
brio run <board> z                # a suite's whole run, judged
```

## Family coverage

The stratum is written for the whole STM32F4 family: twenty-three
device headers, from the F401 to the F479, and `brio check stm32f4`
compiles the smoke TUs on every one of them. What differs is read off
the DEVICE HEADER and never off a device name where the header can
say it - `brio/stm32f4/device_tables.hpp` (the reserve) probes the
header's own base-address and bit-mask macros and exports what it
finds as constexpr data: which GPIO ports exist (A, B, C and H
everywhere; D and E from the 64- and 100-pin bondings; F and G on the
144-pin classes; I, J and K on the big packages), which serial
instances (USART1 and USART2 everywhere, USART6 everywhere but the
36-pin F410Tx, USART3 and UART4/5 from the F405 class, UART7/8 on the
F42x/F43x, F413 and F469 classes, UART9/10 on the F413 alone), whether
the regulator has one VOS bit or two and whether it has the over-drive
pair, how wide the flash latency field is.

ONE CLASS OF FACT IS NOT IN THE HEADER: how fast a part may run at each
regulator scale, how many flash wait states a rate needs, how fast each
APB may go. Those are the reference manual's, and they differ per part
class, so the reserve keys them on the device-select define and states
them ONLY for the classes whose manual is on the desk (RM0090 for the
F405 and F42x/F43x classes, RM0390 for the F446, RM0383 for the F411).
On the F401, F410, F412, F413/F423 and F469/F479 headers
`sysclk_ladder().known` is false and a `Clock` above the 16 MHz reset
rate is a compile error naming the manual to read - proven by
`neg/clock_ladder_unknown.cpp`; the 16 MHz reset rate compiles
everywhere.

## Upload (OpenOCD, ST-LINK)

Flashing goes through OpenOCD driving an ST-LINK: the boards' own
V2-1 and V2-B, or the standalone V3 on the black pill -
`interface/stlink.cfg` + `target/stm32f4x.cfg` (the stm32f2x flash
driver underneath, which identifies the part and its size itself:
`device id = 0x20036419, flash size = 2048 KiB`) + `program <app>.elf
verify`, then `reset run` and a write of DHCSR that clears C_DEBUGEN: a
core left with halting debug enabled HALTS on a BKPT instead of
faulting, and every `panic()` ends in one. OpenOCD - the 0.12.0 release
at `/sw/openocd` - drives all three probes; each carries a REAL USB
serial, so `adapter serial` names it and the same serial names the
console under `/dev/serial/by-id`. Single-client: close the debug
session before flashing.

TWO SWD FACTS worth knowing, both the STM32G0's too: memory reads
THROUGH THE HLA TRANSPORT WHILE THE CORE SLEEPS IN WFI ARE UNRELIABLE -
a running console answered ASCII text for the vector table and 0x101
for every USART register, values those addresses cannot hold, while
the same reads after `halt` were exact (halt first, read, resume). And
`target/stm32f4x.cfg`'s examine-end hook writes `DBGMCU_CR.DBG_SLEEP |
DBG_STOP | DBG_STANDBY` and freezes both watchdogs under halt
(DBGMCU_APB1_FZ), bits that survive every reset but a power-on; the
power chapter's suite is where their effect on a Stop will be measured
and `bin/brio` will take them down as it does on the G0.

The black pill is reached by SWD alone (attach without reset works on
a firmware that leaves the debug pads and never sleeps with debug off);
a program that does either needs the NRST wire and a connect under
reset.

## Debugging (cortex-debug + OpenOCD)

Not yet exercised on this target: one launch config per part in
`.vscode/launch.json` ("Debug STM32F429ZI (OpenOCD, STM32F429I-DISC1)"
and its F446RE and F411CE siblings), the G0 entries' shape with the two
ST config files and the part's own SVD from `stm32f4/svd/` for the
Peripheral Viewer; no probe named, so the one ST-LINK attached is the
one used - with three attached, the entry needs an `adapter serial`
line. Halt-and-dump through OpenOCD's own console is what this
stratum's findings were taken with.

## Editor (clangd)

`brio/stm32f4/.clangd` and `stm32f4/.clangd` route the stratum and the
project to `build-cmake/stm32f429zi-release`. The repo-root `.clangd`
rules apply unchanged.

## Serial console

Each board's ST-LINK bridge is the console at 115200 8N1, addressed by
`/dev/serial/by-id` (a real USB serial on every ST-LINK); the console
apps and the suites print their part's DEV_ID and REV_ID at boot. The
line assembler completes a line on `\n` (`\r` is ignored), which a
terminal sends and a hand-rolled script must too.
