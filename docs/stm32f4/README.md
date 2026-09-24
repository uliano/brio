# Target: STM32F4 (`stm32f4/`)

The operational page for brio's STM32F4 target: an ARM Cortex-M4F
(STM32F429ZI on the bench, on an ST STM32F429I-DISC1; an STM32F446RE on
a Nucleo-F446RE, an STM32F411CE on a WeAct black pill and an STM32F469NI
on a 32F469IDISCOVERY beside it) -
brio's first ARMv7-M family, which includes the `cortexm/` core stratum
unchanged because SysTick, the NVIC's enables and PRIMASK are the same
programmer's model on both architectures. `kernel/` and `util/` run
here as written: time events pacing a pin, and the full console over
each board's own serial bridge.

Peripheral documents live next to this page, one per chapter - the map
below; the documents of record are in [vendor/README.md](vendor/README.md)
together with the four parts' identities (DEV_ID, REV_ID, the errata
sheet each keys into).

## The documents

One document per peripheral driver, in the shape
[../README.md](../README.md) prescribes; the vendor page carries the
documents of record.

| Document | Content |
|----------|---------|
| [platform.md](platform.md) | Platform (STM32F4): `Stm32f4Platform<TB>` - PRIMASK critical section on a core that HAS BASEPRI and does not use it (the kernel promise kept as on the SAM C21: one priority for every line), WFI idle in Sleep mode, the 16 + 91/97/86/93-entry vector table with the FPU enabled before a line of C++, SysTick at 1000 Hz, `delay_us`; measured on four parts: the pending tick delivered after an ISB, not inside the unmask |
| [clock.md](clock.md) | Clock (STM32F4): the STM32G0's model with the APB prescalers NO LONGER PINNED - `pclk1_hz`/`pclk2_hz` beside `hz`, `apb_hz(clock, bus)` for a peripheral's own rate - and the way up in the chapter's order: the regulator scale, the PLL, over-drive between the PLL's start and the switch, the flash latency read back; the frequency ladders KEYED ON THE PART CLASS and refused where no manual was read; 180 MHz in over-drive on three boards (an MCO in bypass, two crystals), 100 MHz on the fourth |
| [port.md](port.md) | GPIO (STM32F4): the STM32G0's register block without a BRR (reset through BSRR's upper half), the port clock on AHB1 and off at reset, INPUT FLOATING as the reset state, AF numbers the datasheets' (AF7 the USARTs, AF8 the UARTs), ports A, B, C and H on every part and the rest by bonding |
| [reset.md](reset.md) | Reset and the watchdogs (STM32F4): RCC_CSR's seven flags as an ACCUMULATING history with PINRSTF raised by every internal source, the IWDG with NO window whose keyed registers do not update until the start key (and whose own reset does stop it), the WWDG on PCLK1 measurable with WDGA never set, the four fault vectors of a Cortex-M4 with the three configurable ones disabled at reset, and the panic breadcrumb across a real reset; the LSI weighed at 33.5 kHz by a watchdog time-out |
| [exti.md](exti.md) | EXTI + SYSCFG (STM32F4): where this family keeps its pin interrupts - sixteen lines numbered by the PIN NUMBER and multiplexed by SYSCFG's EXTICR (another block, another clock gate, closed at reset), the rest peripheral wake-ups whose existence the reserve reads off the PERIPHERALS; one rc_w1 pending bit per line and only while its interrupt is unmasked, the software trigger that needs no pad and does not clear itself, the seven pin vectors and the ISR body that clears first, and an EXTI event out of WFE in 0 us |
| [usart.md](usart.md) | USART (STM32F4): the classic SR/DR/BRR block - flags cleared by read sequences, the divisor in sixteenths or eighths, ten instances at most with the U(S)ART spelling deciding what a FULL one has - the resource over the whole chapter (mute, LIN, IrDA, smartcard, synchronous, flow control, DMA requests) and the `Uart` task with the other strata's surface; the console byte-exact on the four boards at 115200 |
| [rtc.md](rtc.md) | RTC and the backup domain (STM32F4): a BCD calendar in a power domain of its own behind two locks - PWR_CR.DBP over all of it, the RTC's own key over most of it - with RTCSEL one-way and BDRST the way back; the shadow registers and the three errata around them, both alarms, the wake-up timer, the smooth and coarse calibrators and their interlock, the sub-second shift, and the timestamp and tamper that share RTC_AF1 with the two outputs; twenty backup registers that outlive every reset but the domain's own, and the crystal weighed at 32769.18 Hz with a 1 Hz counted on the pad with no instrument |
| [dma.md](dma.md) | DMA (STM32F4): two controllers of eight STREAMS, each with its own FIFO, its own priority and its own vector, choosing between eight request lines with CHSEL - so a peripheral reaches only the one or two (controller, stream, channel) cells the request mapping gives it, and the mapping is a per-part-class fact no header carries; table 49's burst-and-threshold arithmetic refused before the enable, the double buffer swapped in hardware, memory-to-memory on DMA2 alone as the wireless instrument (2048 bytes in 1954 core cycles at 180 MHz), and the two engines the Uart task's slots had been holding empty |
| [tim.md](tim.md) | Timers (STM32F4): fourteen instances of one register block whose geometry is the manual's and not the header's (two 32-bit counters, one break unit per advanced-control timer, TIM9/TIM12 slaving without an encoder, TIM9..TIM14 with no CR2 at all), the counter clock that is HCLK or TWICE its APB clock (and RCC_DCKCFGR's TIMPRE), the rc_w0 status register, four vectors on TIM1 and TIM8 with three of them shared with a small timer, and the two things that make the chapter measurable with nothing attached - the internal triggers and the option registers that put the LSE, the LSI or HSE_RTC on a capture channel; the crystal weighed at 32769.18 Hz and the dead time at 39.9 ns per hundred |
| [adc.md](adc.md) | ADC (STM32F4): up to three converters sharing one PCLK2 prescaler, one reset and one vector - the regular sequence of sixteen ranks and the injected four that PREEMPT it with a signed offset, the four resolutions and their conversion times measured to the ADCCLK cycle, the watchdog whose thresholds are compared before the alignment, the internal channels the MANUAL and not the header numbers, the multi-ADC modes and what really loads their common data register, and the DMA cell each converter's request sits on; VDDA and the junction temperature from the factory measurements, and both errata staged |
| [dac.md](dac.md) | DAC (STM32F4): two 12-bit channels whose outputs reach a PAD and nothing else - which is what lets the ADC read them back with no wire - the holding register and the output register a cycle apart, the buffer measured against a real load, both wave generators with the LFSR's first two words exactly, and a table played by a hardware trigger through the channel's own DMA cell with the underrun a spent stream leaves behind; absent on the F401, F411 and F412, where the resource does not exist |
| [spi.md](spi.md) | SPI and I2S (STM32F4): one block wearing two faces - the F1 lineage's SPI with no FIFO and eight rates off the instance's OWN APB clock, three chip-select arrangements, the simplex and bidirectional line modes, the CRC unit whose polynomial must be ODD, and flags cleared by read SEQUENCES; `SpiHost` with the other strata's Request verbatim over the request mapping's own (controller, stream, channel) cells, `SpiClient` beside it, and the audio face with its own PLL and the I2SDIV/ODD arithmetic; measured against the gyroscope and the display controller a board carries on SPI5 - the rate ladder with the polled pump's constant 240..295 ns a frame, a mode fault raised by SSI alone and visible fifty core cycles later, and the bidirectional read that needs the pad's output stage let go by hand |
| [i2c.md](i2c.md) | I2C (STM32F4): the F1 lineage's EVENT MACHINE - a tenure that advances only when software runs the sequence each flag prescribes, and a RECEIVE PROCEDURE that is three different choreographies by count - over three instances with TWO vectors each and three timing registers (FREQ, CCR under F/S and DUTY, and TRISE, which is the wire's rise time and not a guess); `I2cHost` with the other strata's Request verbatim over the request mapping's own (controller, stream, channel) cells, all of them on DMA1, `I2cClient` beside it, and the errata as code - a controller's BUS ERROR counted and never acted on, SWRST the way back, the repeated start's setup time published where it is at risk; measured against the STMPE811 the board carries on I2C3 - one address of 112 answering, the three receive procedures byte-exact against each other, and SCL counted on its own pad while the DMA carries a 32-byte read. THE TRAP: BUSY stands out of reset over an idle wire, because it watches the peripheral's own inputs and those read low while the pads are not yet in their alternate function |
| [fmpi2c.md](fmpi2c.md) | FMPI2C (STM32F4): the SECOND I2C design of this family, on the F410, F412, F413/F423 and F446 - the STM32G0's register file under the name FMPI2C1 (one TIMINGR word of five fields, a byte counter with RELOAD and AUTOEND, ISR/ICR, two own addresses with a mask), so the driver is that driver with an `Fmp` prefix, TWO vectors instead of one, a KERNEL CLOCK OF ITS OWN (the APB, SYSCLK or the HSI, which is what buys Fast-mode Plus at 1 MHz and a bus that survives a core rate change) and no wake from Stop, table 127's one dash; the errata as code - a controller's BUS ERROR counted and never acted on, RELOAD never used, the transmit-stall band published on the ratio between the two clocks; `FmpI2cHost` with the other strata's Request verbatim over the one (controller, stream, channel) cell the mapping gives each direction. Measured on a bus with NOTHING ON IT: 112 addresses of 112 NACKed with the STOP the peripheral sends by itself seen on the wire, SCL counted on its own pad at three speeds and three kernel clocks, and the idle time-out fired at 51.1 us where 50 was asked. THE FINDING: the manual's assumed SCL detection delay is generous - 50..500 ns really - so a bus solved against the defaults runs a few per cent FASTER than the speed asked for |
| [usb.md](usb.md) | USB OTG, device mode (STM32F4): a Synopsys DWC2 core - up to two of them, the full-speed one on PA11/PA12 and the high-speed one's own full-speed PHY on PB14/PB15 - whose whole interface is FIFOS reached through push and pop registers, with one shared receive queue for every OUT endpoint and a slice of RAM per IN endpoint the program maps by hand and the silicon never checks; the address that must be in DCFG BEFORE the status stage of SET_ADDRESS, against chapter 9 and measured; the OUT transfer of several packets that is this core's answer to the NAK a hub charges a frame for; 658 KB/s out and 440 KB/s in on the black pill's own connector, with the kernel console over it |
| [pwr.md](pwr.md) | PWR, the sleep sites and Standby (STM32F4): the mode as two registers in two places (SLEEPDEEP and PDDS), a Stop whose price is four more bits (LPDS, FPDS, the low-voltage pair, under-drive) and whose exit puts the core back on the HSI with the PLL, the HSE and over-drive gone - but the VOS field untouched; `util/power.hpp`'s ladder over it with Standby ENFORCED off it, and the timed site that meets a deadline through a Stop on the RTC's wake-up timer; the flash's 92 us of wake reproducing the datasheet's typical, a 213 us restore to 180 MHz, and a 500 ms event met at 500 ms of wall |
| [fmc.md](fmc.md) | FMC, the external memory controller (STM32F4): the one peripheral that adds ADDRESS SPACE instead of driving a wire, and the one chapter with three answers to "which block does this part have" - the FMC on the F42x/F43x, the F446 and the F469/F479, the FSMC (another peripheral, no SDRAM half) on the F405 class, the F412 and the F413/F423, and neither on seven parts of the pack; six windows of the Cortex's own map whose decoder answers before the memory does, so a read of an SDRAM bank that has not run its sequence HANGS THE MACHINE with no fault and `initialize()` is one verb that runs the whole of it or answers false; the SDRAM controller in the chapter's words - the geometry, the seven delays in memory cycles, the command machine, the refresh counter's arithmetic, self-refresh and power-down - and the NOR/PSRAM half with both timing registers and the extended mode; measured against the 64-Mbit SDRAM a board carries: 8 MB byte-exact in three access widths at 80 MB/s written and 33 MB/s read, a DMA stream at twice the CPU's rate going in, a single READ enough to leave self-refresh, TRC the one delay with a measured floor, and an array that keeps its bits for seconds with nothing refreshing it. THE TWO TRAPS, both against the manual: a COUNT of zero does not reach SDRTR (the silicon leaves 40 and the timer runs on) and SDCR1.SDCLK = 00 does not stop FMC_SDCLK - only the block's own reset leaves the device unclocked |
| [ltdc.md](ltdc.md) | LTDC, the LCD-TFT display controller (STM32F4): the one peripheral of this family whose output is a PICTURE - two layers, eight pixel formats, a colour table each, blending and colour keying, out of whatever memory holds the frame buffer - with a PLL of its own for the pixel clock that has no input divider of its own (it divides the MAIN PLL's M, so a pixel clock is an exact triple or nothing), THREE clock domains that turn out to be visible from the bus (every pixel-clock register reads ZERO with no pixel clock, whatever its reset value), a shadow register behind every layer register but the colour table (LxCR included, so a layer's enable is atomic with its geometry) and a read straight after the reload store that RACES the reload - which `reload()` covers with one access of its own. Measured against the 240x320 panel this board carries, taken into its RGB mode over SPI5: the pixel clock exact to 0 per mille, the line event once a frame, 16.4.1's own example caught printing the two TOTAL fields without the minus one its own rule asks for, an underrun flag that exists only while its enable is set and is raised PER PIXEL when it does, and the number the display tier lives on - two layers of 32-bit pixels at 65 Hz plus the accelerator on the same SDRAM, no starvation, some 160 MB/s over the bus in every configuration |
| [quadspi.md](quadspi.md) | Quad-SPI memory interface (STM32F4): a command engine of five phases on one, two or four lines in front of a serial flash, with three faces - indirect (the program pumps a 32-byte FIFO), automatic status polling (the block repeats a read and matches it) and memory-mapped (a load from 0x9000 0000 is the read command); a command that starts on the last register it needs, the flash's size and NCS's high time as part of the contract, the window and the DMA cell the manual gives and the header does not; four of the five errata as code, and the flag a poller must not share with a handler. Measured against the 32F469IDISCOVERY's MT25QL128: the identity and the SFDP table, the last subsector erased through status polling and programmed a page at a time, five read commands byte-exact (quad I/O at 36 MB/s on the pump, 44.5 MB/s through the window), the dummy cycles against the device's own table, the FIFO stalling the clock, the abort, the error past the size, deep power-down |
| [dsi.md](dsi.md) | DSI host (STM32F4): the MIPI Display Serial Interface in front of the LTDC on the F469/F479 class alone - the wrapper's regulator and PLL, the two-lane D-PHY on dedicated pads, the host's two ways to a panel (video mode packing the LTDC's frame into pixel packets, the ADAPTED COMMAND MODE turning each frame into DCS memory writes launched by a refresh or by the tearing effect pulse) and the generic interface a panel is spoken to through; the PLL's ranges as two documents state them and the solver on their intersection, the unit interval in quarter nanoseconds, the frame converted into lane byte clocks with its rounding, the lane transition times from the data sheet's maxima, two of the three errata as code. THE PANEL DECIDES BETWEEN THE TWO WAYS, and the rule the manual does not state: in video mode the generic interface lives inside the stream. Measured against the 32F469IDISCOVERY's panel, whose controller is READ over the link (an NT35510 by its three ID bytes, a command interface by its datasheet): every DCS register written and read back, one refresh of the frame in exactly one LTDC frame and 61 a second, THE PICTURE READ BACK out of the panel's memory pixel for pixel, a second of video-mode frames leaving that memory untouched, the tearing effect pulses counted on the pin and by the wrapper alike and the automatic refresh paced by them, ULPS, and what the panel reports at the first turnaround after a re-enable |
| [dma2d.md](dma2d.md) | DMA2D, the Chrom-Art accelerator (STM32F4): a DMA that knows what a pixel is - rectangles and not runs, four modes over eleven input formats and five output ones, two colour tables, a blender with a stated rounding, and a bus master of its own, which is what makes its reads of external memory the ones ES0206 2.3.5 allows. Present on two parts of the class that have no display interface at all, so its presence is a WIDER fact than the controller's. Every mode measured byte-exact against a software model - 16.4.2's channel expansion, 11.3.11's blend to the unit, the five output packings - with the configuration error raised by the silicon where the driver's own refusals do not reach; the colour table loaded both by the CPU and by the engine; and the bandwidth beside a running display, where the AHB dead time is the one knob. THE TRAP is the compiler's: a buffer this engine writes must be volatile, because its address reaches it as an integer nothing dereferences |
| [crc.md](crc.md) | CRC calculation unit (STM32F4): three registers and one function wired in - the Ethernet polynomial with no knobs, which is CRC-32/MPEG-2 exactly, a word at a time because the register takes nothing narrower, and the software model beside it that the silicon is judged against; the reset that lands ONE READ LATE and swallows a word written in the instruction after it, and CRC_IDR, eight bits of scratch the reset spares and the RCC line clears |
| [rng.md](rng.md) | Random number generator (STM32F4): ring oscillators feeding an LFSR on the 48 MHz domain, with the manual's only clock constraint being a RATIO (f(RNG_CLK) below f(HCLK)/16 raises CECS) and not a rate, the first value discarded and every next one compared as FIPS PUB 140-2 asks, two error statuses of different kinds whose latched flags are rc_w0, the gate that moves to AHB1 on the F410 and the vector that wears two names; absent on the F401, F411 and F446, measured on the STM32F429 - a thousand words a millisecond at 45 MHz, the ones 499 per mille, and a computed word that survives the disable |
| [can.md](can.md) | bxCAN (STM32F4): up to three instances and ONE filter block for the first two, split at CAN2SB - 28 banks in four shapes whose match index is not the bank number - the bit time searched exactly at compile time from PCLK1, three mailboxes ordered by identifier or by request, two FIFOs of three whose overrun policy is a choice, the error counters walked to bus-off and back in 1407 us, and the loopback that still needs eleven recessive bits ON ITS RECEIVE PAD; a frame type of this stratum's own, because the util CAN vocabulary waits for a second family |
| [flash.md](flash.md) | The embedded flash memory interface (STM32F4): an array of UNEVEN sectors - four of 16 Kbytes, one of 64, then 128 to the end of the bank, a rule the driver computes the map from rather than tabulating it - with the program width declared TWICE (PSIZE and the store instruction, so the parallelism is a template parameter and x64 is refused for want of a VPP supply), a keyed lock whose wrong sequence is an imprecise bus error and a lockout until reset, and the option bytes read whole with nWRP the one of them a running program may write; the two facts this family turns on, both measured: a 128 Kbyte erase FREEZES the core for a second (the kernel tick advances 1 ms, and with the ART off not one poll turn completes) and a second program into a programmed word is accepted and ANDs into the array - while the D-cache answers with what was written |

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

Four boards, four parts, no two alike - each with its page in
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
- **32F469IDISCOVERY** - the F469/F479 class's part, the F42x/F43x's
  superset (2 MB, 320 KB of SRAM, the FMC with a 32-bit SDRAM, the LTDC
  behind a DSI host driving the 800x480 panel, the QUADSPI with a 16 MB flash); 8 MHz crystal on HSE, the PLL at
  **180 MHz in over-drive** (168 MHz for the USB apps, the rate whose
  VCO divides to 48 MHz); the ST-LINK/V2-1's VCP on USART3 PB10/PB11;
  LD1 on PG6, lit when low; its OTG FS connector cabled to the host.

All four run at 3.3 V (the DISC1's rail reads 3.0 V). Verified at the
bench, each by its own experiment: the LED pins (the blink app), the
console pads and instances (the console answers on all four), the HSE
modes (a bypass on the DISC1 never sees HSERDY - the MCO route is a
solder bridge left open, UM1670 7.12.1), the whole clock tree as the
registers hold it (the platform suite's letter `e`).

`stm32f4/` is its own CMake project, a sibling and peer of the other
five. Apps are auto-discovered from `stm32f4/src/apps/*.cpp` - plus
`experiments/*/stm32f4/*.cpp` - by their `// build:` header comment,
the other projects' grammar with this family's board names (`boards =
f429zi`, the default; an app that also runs on the others lists
`f446re`, `f411ce` and `f469ni`). One configure targets one part (`STM32F4_MCU`,
the full part number: the table derives the device define, the crt
`src/glue/startup_<header>.cpp` and the board name, the part's last six
characters - one board per part on this desk, so the part IS the board;
the linker script is `ld/<part>.ld`); the four parts have presets.
Every other part of the family is compile-checked by `brio check
stm32f4`, which sweeps every positive TU in `test/family_stm32f4/`
across ALL TWENTY-THREE device headers the CMSIS pack ships and requires
every `neg/` TU to fail for the variants its `// mcu:` line names - see
"Family coverage" below.

```bash
(cd stm32f4 && cmake --preset stm32f429zi-release)                 # the bench chip
(cd stm32f4 && cmake --preset stm32f446re-release)                 # the Nucleo
(cd stm32f4 && cmake --preset stm32f411ce-release)                 # the black pill
(cd stm32f4 && cmake --preset stm32f469ni-release)                 # the 32F469IDISCOVERY
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
F405 and F42x/F43x classes, RM0390 for the F446, RM0383 for the F411,
RM0386 for the F469/F479). On the F401, F410, F412 and F413/F423
headers `sysclk_ladder().known` is false and a `Clock` above the 16 MHz
reset rate is a compile error naming the manual to read - proven by
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
at `/sw/openocd` - drives all four probes; each carries a REAL USB
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
(DBGMCU_APB1_FZ), bits that survive every reset but a power-on; a
program that measures a sleep takes them down itself at boot
(`Pwr::debug_low_power(false)`, [pwr.md](pwr.md)) - `bin/brio` does
not.

The black pill is reached by SWD alone (attach without reset works on
a firmware that leaves the debug pads and never sleeps with debug off);
a program that does either needs the NRST wire and a connect under
reset.

## Debugging (cortex-debug + OpenOCD)

One launch entry per part in `.vscode/launch.json` ("Debug STM32F429ZI
(OpenOCD, STM32F429I-DISC1)" and its F446RE, F411CE and F469NI
siblings), the G0
entries' shape: cortex-debug launches `/sw/openocd/bin/openocd` itself
with the two ST config files, puts `arm-none-eabi-gdb` in front of it and
hands the part's own SVD from `stm32f4/svd/` to the Peripheral Viewer.
CMake Tools' Active Folder must be `stm32f4/`, its configure preset the
part's `-debug` one (`-Og -g3 -ggdb3 -fno-inline`) and its launch target
the app; `runToEntryPoint` lands on `main`.

The same thing from a shell, which is what the entry does - the server,
then gdb against the DEBUG build of a real app:

```bash
/sw/openocd/bin/openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "adapter serial <the probe's serial>"       # gdb server on 3333
/sw/arm-none-eabi/bin/arm-none-eabi-gdb ../build-cmake/stm32f411ce-debug/console.elf
  (gdb) target extended-remote localhost:3333
  (gdb) monitor reset halt
  (gdb) load
  (gdb) break console.cpp:217        # a line inside a command handler
  (gdb) continue                     # then type UPTIME on the console
  (gdb) bt                           # the whole AO chain, kernel included
  (gdb) print ts                     # $1 = {seconds = 2, millis = 300}
  (gdb) finish
  (gdb) monitor mdw 0x40023808       # a register at the SVD's own address
  (gdb) monitor resume
  (gdb) detach
```

NAME THE PROBE. With more than one ST-LINK attached and no `adapter
serial`, OpenOCD takes the first one it enumerates, which is another
board; the entries carry no serial, so one needs
`"openOCDPreConfigLaunchCommands": ["adapter serial <s>"]`. Either
ordering selects it - before the config files, after them, or between
the two, where `brio flash` puts it.

What a session does, what it costs and what it leaves behind, measured
on the black pill:

- **The cost.** OpenOCD from launch to "Listening on port 3333" 0.12 s;
  gdb's connect to it 0.02..0.05 s; `monitor reset halt` 0.03 s; `load`
  of a 14696-byte image 0.56 s at 25 KB/s. A whole launch - server,
  connect, reset halt, load, run to `main` - is under a second. The
  debug preset costs 2.7x the image: `console` is 14672 bytes of text
  against 5472 at `-Os`, and 584 of bss against 576.
- **`monitor reset halt` needs no NRST wire.** Under the HLA transport
  `target/stm32f4x.cfg` selects `cortex_m reset_config sysresetreq`, so
  the core resets itself: the halt lands at `Reset_Handler` with xPSR
  0x01000000 and MSP at the top of SRAM even where the probe has only
  the four SWD wires and cannot pull the reset pin.
- **Connecting halts the running program**, which is what makes a debug
  session's memory reads exact where a poking one's are not (the HLA
  trap above). A kernel that idles in WFI is caught in the same place
  every time: the `__enable_irq()` after the `__WFI()` of
  `Stm32f4Platform::idle()`.
- **Six hardware breakpoints, and the refusal comes at the RESUME.**
  Flash is read-only to gdb, so every breakpoint is a hardware one - no
  flash wear, and no software-breakpoint escape either. gdb accepts a
  seventh and an eighth `break` without a word; the following `continue`
  answers `Cannot insert hardware breakpoint N: Remote failure reply:
  0E`, aborts, and leaves the core halted.
- **The kernel survives a halt, the clock does not.** After a `continue`
  the console answers the next line and the suites run to their usual
  verdict. But SysTick is the core's own counter and a halted core does
  not count it: `UPTIME` under-reports the wall by the time spent
  halted - 11.3 s lost across a halt held 10 s. Time events do not
  mature during a halt, so a deadline under a debugger is the
  debugger's and not the wall's.
- **`detach` does not resume.** Detaching from a halted core leaves it
  halted and the board silent; `monitor resume` (or `monitor reset run`)
  before the detach is what leaves it running, and the console answers
  again at once.
- **The Peripheral Viewer's read path** is `monitor mdw` at the SVD's
  own base addresses, and it agrees with what the firmware prints. With
  the console's `CLK` line reading sysclk 100 MHz, pclk1 50 MHz, pclk2
  100 MHz, SWS 2, the PLL locked, HSE a crystal, scale 1, no over-drive
  and 3 wait states: RCC_CFGR reads 0x0000100A (SW and SWS both the PLL,
  PPRE1 /2, PPRE2 /1), RCC_CR 0x03036F83 (HSE on, ready and not
  bypassed; the PLL locked), PWR_CR 0x0000C000 (VOS scale 1), FLASH_ACR
  0x00000703 (three wait states, the prefetch and both caches) and
  USART1_BRR 0x00000364 = 868, which is 115207 baud off a 100 MHz APB2.
  DBGMCU_IDCODE reads 0x10006431, the part's DEV_ID and REV_ID.
- **What it leaves behind**: DBGMCU_CR 0x00000007 and DBGMCU_APB1_FZ
  0x00001800 - the target script's examine-end hook, DBG_SLEEP |
  DBG_STOP | DBG_STANDBY and both watchdogs frozen under halt. They
  survive every reset but a power-on, and a `brio flash` sets them
  again; a Stop measured just after a session is measuring them and not
  the silicon.

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
