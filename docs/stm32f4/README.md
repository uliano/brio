# Target: STM32F4 (`stm32f4/`)

The operational page of brio's STM32F4 target, for a programmer who has
one of the four STM32F4 boards on the desk and wants to build, flash and
debug a brio program on it, and then find the document of the
peripheral the program needs. It says what the stratum is, which boards
and parts it is proven on, how the build, the upload, the debugger and
the console work and where each has a trap, and it ends with the map of
the stratum's own documents: one line per peripheral driver, which is
where the state of the driver work is read.

A stratum is a directory of `brio/` holding one architectural layer;
`brio/stm32f4/` holds everything that includes ST's device header
`stm32f4xx.h`. The STM32F4 is an ARM Cortex-M4F family (the ARMv7-M
architecture with a single-precision FPU), brio's one ARMv7-M family,
and it includes the `cortexm/` core stratum unchanged
([../cortexm/README.md](../cortexm/README.md)), because SysTick, the
NVIC's enables and PRIMASK are the same programmer's model on ARMv6-M
and ARMv7-M. The kernel (`kernel/`) and the services (`util/`) run here
as written: time events pacing a pin, and the full console over each
board's own serial bridge. The bench chip is the STM32F429ZI on an ST
STM32F429I-DISC1, with an STM32F446RE on a Nucleo-F446RE, an STM32F411CE
on a WeAct black pill and an STM32F469NI on a 32F469IDISCOVERY beside
it; the rest of the family is covered by a compile check over every
device header ST ships ("Family coverage" below). The maintained
example `blink` builds for all four boards.

The documents of record (the reference manuals, datasheets and errata
by revision) and the four parts' identities (DEV_ID, REV_ID, the errata
sheet each keys into) are in [vendor/README.md](vendor/README.md).

## The boards

Four boards, four parts, no two alike; the part selects the board in the
build (below), and each board has a page of its own under
[../boards/](../boards/README.md) with its connectors and its probe.

| Board | Part | Clock | Console | LED |
|---|---|---|---|---|
| [STM32F429I-DISC1](../boards/stm32f429i-disc1.md), the bench board | STM32F429ZI, the superset part: 2 MB, the FMC with an SDRAM, the LTDC with a display, ports up to K | an 8 MHz crystal on HSE, the PLL at 180 MHz in over-drive | the ST-LINK/V2-B's virtual COM port on USART1 PA9/PA10 | LD3 on PG13 |
| [Nucleo-F446RE](../boards/nucleo-f446re.md) | STM32F446RE, another package and the family's extras | the ST-LINK's 8 MHz MCO into HSE in bypass, the PLL at 180 MHz in over-drive | the V2-1's virtual COM port on USART2 PA2/PA3 | LD2 on PA5 |
| [a WeAct STM32F411CE black pill](../boards/blackpill-f411ce.md) | STM32F411CE, the small part: 100 MHz, no over-drive, one ADC, ports A..C and H | a 25 MHz crystal, the PLL at 100 MHz | a standalone STLINK-V3's UART bridge on USART1 PA9/PA10 | PC13, lit when low |
| [32F469IDISCOVERY](../boards/stm32f469i-discovery.md) | STM32F469NI, the F469/F479 class's part, the F42x/F43x's superset: 2 MB, 320 KB of SRAM, the FMC with a 32-bit SDRAM, the LTDC behind a DSI host driving the 800x480 panel, the QUADSPI with a 16 MB flash | an 8 MHz crystal on HSE, the PLL at 180 MHz in over-drive (168 MHz for the USB apps, the rate whose VCO divides to 48 MHz) | the ST-LINK/V2-1's virtual COM port on USART3 PB10/PB11 | LD1 on PG6, lit when low; its OTG FS connector cabled to the host |

All four run at 3.3 V (the DISC1's rail reads 3.0 V). Verified at the
bench, each by its own experiment: the LED pins (the blink app), the
console pads and instances (the console answers on all four), the HSE
modes (a bypass on the DISC1 never sees HSERDY: the MCO route is a
solder bridge left open, UM1670 7.12.1), the whole clock tree as the
registers hold it (the platform suite's letter `e`).

## Toolchain

The compiler is a self-built arm-none-eabi-gcc 16.2 at
`/sw/arm-none-eabi`, the same compiler and linker discipline as the
other ARM projects: `stm32f4/cmake/toolchain-arm.cmake` is the stm32g0
file verbatim (`CMAKE_SYSTEM_NAME Generic`, a `STATIC_LIBRARY`
try-compile, and a link that takes brio's runtime and libgcc and no C
library, so an accidental `printf` or `malloc` fails it,
[../design/runtime.md](../design/runtime.md)), with this family's core
flags: `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`. **The FPU
is used, not carried**: brio has no floats, so the hard ABI costs
nothing today, and a program that wants one pays no ABI break across
every image; the crt enables the coprocessor (CPACR) before the first
C++ instruction, because under the hard ABI a float may sit in an s
register anywhere. What is Cortex-M and not ST lives in the `cortexm/`
core stratum, so `nvic.hpp`, `ticker.hpp` and `delay.hpp` here are the
device header plus that core file plus this family's own facts.

The device headers are vendored: `third_party/cmsis-device-f4/` holds
ST's cmsis-device-f4 v2.6.9 (every F4 part) and the shared
`third_party/cmsis-core/` the CMSIS-Core headers (with `core_cm4.h`).
The device is selected ST's way, by a plain `-DSTM32F429xx` define that
the umbrella header `stm32f4xx.h` dispatches on; there is no
device-specs machinery, so clangd needs no macro feed of its own. The
define's spelling is irregular across the pack (`STM32F411xE`,
`STM32F410Cx`), which is why the project keeps a part table
(`stm32f4/cmake/stm32f4-parts.cmake`) instead of the G0 project's
substring arithmetic.

## Build

`stm32f4/` is its own CMake project, a sibling and peer of the other
cross projects and of `test/`; the repository root is not a CMake
project, so every CMake command runs from `stm32f4/`:

```bash
(cd stm32f4 && cmake --preset stm32f429zi-release)                 # the bench chip; configure once, or after adding an app
(cd stm32f4 && cmake --preset stm32f446re-release)                 # the Nucleo
(cd stm32f4 && cmake --preset stm32f411ce-release)                 # the black pill
(cd stm32f4 && cmake --preset stm32f469ni-release)                 # the 32F469IDISCOVERY
(cd stm32f4 && cmake --build --preset stm32f429zi-release --target <app>)
(cd stm32f4 && cmake --build --preset stm32f429zi-release --target <app>-upload)
brio check stm32f4 [name]                                     # family smoke, no hardware
```

One configure targets one part. The cache variable `STM32F4_MCU` holds
the full part number, and the part table derives from it the device
define, the crt `src/glue/startup_<header>.cpp` (the vector table's
names and length follow the header's IRQn list) and the board name, the
part's last six characters (`f429zi`, `f446re`, `f411ce`, `f469ni`): one
board per part on this desk, so the part is the board; the linker
script is `ld/<part>.ld`. The four parts have presets, a release and a
debug one each (`stm32f429zi-release`, `stm32f429zi-debug` and the
three other pairs; CMake Tools shows them as `STM32F429ZI (release,
-Os)` and `STM32F429ZI (debug, -Og)`). A release build is `-Os`, the one
to flash for a measurement; a debug build is `-Og -g3 -ggdb3
-fno-inline`, the one a debugger wants.

Applications are discovered at every configure from
`stm32f4/src/apps/*.cpp`, plus `experiments/*/stm32f4/*.cpp`, by the
`// build:` lines in each file's header comment, the other projects'
grammar with this family's board names: `// build: boards = f429zi` is
the default, and an app that also runs on the others lists `f446re`,
`f411ce` and `f469ni`.

Build outputs land in `build-cmake/<preset>/` at the repository root:
`<app>.elf`, `.bin` and `.hex`, `firmware-<app>.map` and `<app>.lst`,
the source-interleaved disassembly. A configure also writes this
project's app roster, `build-cmake/apps_stm32f4.json`, which `bin/brio`
reads: the four board types are what tell the tool to build here and to
flash through OpenOCD's ST-LINK interface, by the probe's serial.
`brio flash <board> console` builds and flashes the console in one
step, and `brio run <board> z` runs a suite's whole set and judges it
([../boards/README.md](../boards/README.md)).

## Upload (OpenOCD, ST-LINK)

Flashing goes through OpenOCD driving an ST-LINK, the boards' own V2-1
and V2-B or the standalone V3 on the black pill
([../probes/st-link.md](../probes/st-link.md)): `interface/stlink.cfg`
plus `target/stm32f4x.cfg` (the stm32f2x flash driver underneath, which
identifies the part and its size itself: `device id = 0x20036419, flash
size = 2048 KiB`), `program <app>.elf verify`, then `reset run` and a
write of DHCSR that clears C_DEBUGEN. That last write matters: a core
left with halting debug enabled halts on a BKPT instruction instead of
faulting, and every `panic()` ends in one. OpenOCD, the 0.12.0 release
at `/sw/openocd`, drives all four probes; another path is named with
the cache variable `STM32F4_OPENOCD`. Each probe carries a real USB
serial, so `adapter serial` names it (`-DSTM32F4_PROBE_SERIAL=<serial>`
at configure time) and the same serial names the console under
`/dev/serial/by-id`. The probe serves one client at a time: close the
debug session before flashing.

Two SWD facts worth knowing before reading a register through the
debugger, both the STM32G0's too.

**Memory reads through the HLA transport while the core sleeps in WFI
are unreliable** (HLA is the high-level adapter interface the ST-LINK
speaks). A running console answered ASCII text for the vector table and
0x101 for every USART register, values those addresses cannot hold,
while the same reads after `halt` were exact. Halt first, read, then
resume.

**The target script leaves the debug-in-low-power bits set.**
`target/stm32f4x.cfg`'s examine-end hook writes `DBGMCU_CR.DBG_SLEEP |
DBG_STOP | DBG_STANDBY` and freezes both watchdogs under halt
(DBGMCU_APB1_FZ), bits that survive every reset but a power-on. A
program that measures a sleep takes them down itself at boot
(`Pwr::debug_low_power(false)`, [pwr.md](pwr.md)); `bin/brio` does not.

The black pill is reached by SWD alone: an attach without reset works
on a firmware that leaves the debug pads to the debug port and never
sleeps with debug off. A program that does either needs the NRST wire
and a connect under reset.

## Debugging (cortex-debug + OpenOCD)

`.vscode/launch.json` has one launch configuration per part: "Debug
STM32F429ZI (OpenOCD, STM32F429I-DISC1)" and its F446RE, F411CE and
F469NI siblings, the G0 entries' shape: cortex-debug launches
`/sw/openocd/bin/openocd` itself with the two ST config files, puts
`arm-none-eabi-gdb` in front of it and hands the part's own SVD from
`stm32f4/svd/` to the Peripheral Viewer. Three choices in CMake Tools
must agree before a session starts: the Active Folder `stm32f4/`, the
configure preset the part's `-debug` one, and the launch target the app
to debug; `runToEntryPoint` lands on `main`.

**Name the probe.** With more than one ST-LINK attached and no `adapter
serial`, OpenOCD takes the first one it enumerates, which is another
board; the entries carry no serial, so one needs
`"openOCDPreConfigLaunchCommands": ["adapter serial <s>"]`. Either
ordering selects it: before the config files, after them, or between
the two, where `brio flash` puts it.

The same thing from a shell, which is what the entry does: the server,
then gdb against the debug build of a real app:

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

What a session does, what it costs and what it leaves behind, measured
on the black pill:

- **The cost.** OpenOCD from launch to "Listening on port 3333" 0.12 s;
  gdb's connect to it 0.02..0.05 s; `monitor reset halt` 0.03 s; `load`
  of a 14696-byte image 0.56 s at 25 KB/s. A whole launch (server,
  connect, reset halt, load, run to `main`) is under a second. The debug
  preset costs 2.7x the image: `console` is 14672 bytes of text against
  5472 at `-Os`, and 584 of bss against 576.
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
- **Six hardware breakpoints, and the refusal comes at the resume.**
  Flash is read-only to gdb, so every breakpoint is a hardware one: no
  flash wear, and no software-breakpoint escape either. gdb accepts a
  seventh and an eighth `break` without a word; the following `continue`
  answers `Cannot insert hardware breakpoint N: Remote failure reply:
  0E`, aborts, and leaves the core halted.
- **The kernel survives a halt, the clock does not.** After a `continue`
  the console answers the next line and the suites run to their usual
  verdict. But SysTick is the core's own counter and a halted core does
  not count it: `UPTIME` under-reports the wall by the time spent
  halted, 11.3 s lost across a halt held 10 s. Time events do not mature
  during a halt, so a deadline under a debugger is the debugger's and
  not the wall's.
- **`detach` does not resume.** Detaching from a halted core leaves it
  halted and the board silent; `monitor resume` (or `monitor reset
  run`) before the detach is what leaves it running, and the console
  answers again at once.
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
  0x00001800, the target script's examine-end hook, DBG_SLEEP |
  DBG_STOP | DBG_STANDBY and both watchdogs frozen under halt. They
  survive every reset but a power-on, and a `brio flash` sets them
  again; a Stop measured just after a session is measuring them and not
  the silicon.

## Editor (clangd)

`brio/stm32f4/.clangd` and `stm32f4/.clangd` route the stratum and the
project to `build-cmake/stm32f429zi-release`, which one configure of
that preset creates. The repository root's `.clangd` rules apply
unchanged.

## Serial console

Each board's ST-LINK bridge is the console at 115200 8N1, addressed by
`/dev/serial/by-id` (a real USB serial on every ST-LINK); the console
apps and the suites print their part's DEV_ID and REV_ID at boot. The
line assembler completes a line on `\n` (`\r` is ignored), which a
terminal sends and a hand-rolled script must too.

## Family coverage

The stratum is written for the whole STM32F4 family: twenty-three
device headers, from the F401 to the F479, and `brio check stm32f4`
compiles the smoke TUs on every one of them. What differs is read off
the device header and never off a device name where the header can say
it: `brio/stm32f4/device_tables.hpp`, which the stratum's documents
call the reserve, probes the header's own base-address and bit-mask
macros and exports what it finds as constexpr data. The reserve is where
a per-part fact lives when no register of the peripheral states it:
which GPIO ports exist (A, B, C and H everywhere; D and E from the 64-
and 100-pin bondings; F and G on the 144-pin classes; I, J and K on the
big packages), which serial instances (USART1 and USART2 everywhere,
USART6 everywhere but the 36-pin F410Tx, USART3 and UART4/5 from the
F405 class, UART7/8 on the F42x/F43x, F413 and F469 classes, UART9/10 on
the F413 alone), whether the regulator has one VOS bit or two and
whether it has the over-drive pair, how wide the flash latency field
is.

One class of fact is not in the header: how fast a part may run at each
regulator scale, how many flash wait states a rate needs, how fast each
APB may go. Those are the reference manual's, and they differ per part
class, so the reserve keys them on the device-select define and states
them only for the classes whose manual is on the desk (RM0090 for the
F405 and F42x/F43x classes, RM0390 for the F446, RM0383 for the F411,
RM0386 for the F469/F479). On the F401, F410, F412 and F413/F423
headers `sysclk_ladder().known` is false and a `Clock` above the 16 MHz
reset rate is a compile error naming the manual to read, proven by
`neg/clock_ladder_unknown.cpp`; the 16 MHz reset rate compiles
everywhere.

`brio check stm32f4` proves it without hardware: every positive
translation unit in `test/family_stm32f4/` compiles for all
twenty-three device headers, and every `neg/` TU, a program that must
be refused, fails to compile for each variant its `// mcu:` line names;
the check ends with the link guard of every app on every release preset.
The bench proves four of the twenty-three; for the other nineteen the
sweep is the only check there is.

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
| [platform.md](platform.md) | What the kernel stands on: `Stm32f4Platform<TB>`, a PRIMASK critical section on a core that has BASEPRI and does not use it (the kernel promise kept as on the SAM C21: one priority for every line), WFI idle in Sleep mode, the 16 + 91/97/86/93-entry vector table with the FPU enabled before a line of C++, SysTick at 1000 Hz, `delay_us`; measured on four parts: the pending tick delivered after an ISB, not inside the unmask; on the F446RE the idle contract to the cycle (no edge position of 400 loses its wake, the tick's or an NVIC line's, letter `w`; one kernel turn per tick, letter `k`; a wake from Sleep 20 cycles to the handler against 16 awake); and `Stm32f4RubatoPlatform<TB>`, the preemptive kernel's platform over `cortexm/schedule.hpp` (PendSV at the lowest level returning into the scheduler in thread mode, SVC the way back), measured on the Nucleo-F446RE by `test_stm32f4_rubato` (32 verdicts: road 1 in 145 cycles, a quiet post 79, a road-2 post 403, the FPU bit-exact across a preemption, 664 bytes of stack for the deepest chain) |
| [clock.md](clock.md) | The STM32G0's clock model with the APB prescalers no longer pinned: `pclk1_hz`/`pclk2_hz` beside `hz`, `apb_hz(clock, bus)` for a peripheral's own rate, and the way up in the chapter's order (the regulator scale, the PLL, over-drive between the PLL's start and the switch, the flash latency read back); the frequency ladders keyed on the part class and refused where no manual was read; 180 MHz in over-drive on three boards (an MCO in bypass, two crystals), 100 MHz on the fourth; on the STM32F446 the dynamic ladder walks regulator scale 2 (144 MHz) and a PLL fed by the HSI, and the ART accelerator's three switches and two cache resets are each timed |
| [port.md](port.md) | GPIO: the STM32G0's register block without a BRR (reset through BSRR's upper half), the port clock on AHB1 and off at reset, input floating as the reset state, AF numbers the datasheets' (AF7 the USARTs, AF8 the UARTs), ports A, B, C and H on every part and the rest by bonding; open drain against push-pull, the pull-down, PinSet and a two-pin toggle, and release() blinding the input buffer, on two free pads |
| [reset.md](reset.md) | Reset and the watchdogs: RCC_CSR's seven flags as an accumulating history with PINRSTF raised by every internal source, the IWDG with no window whose keyed registers do not update until the start key (and whose own reset does stop it), the WWDG on PCLK1 measurable with WDGA never set, the four fault vectors of a Cortex-M4 with the three configurable ones disabled at reset, and the panic breadcrumb across a real reset; the LSI weighed at 33.5 kHz (STM32F429ZI) and 31.4 kHz (STM32F446RE) by a watchdog time-out, both deliberate resets timed, a BusFault and an unaligned access taken at their own vectors, and the IWDG's reload written only after the prescaler has crossed, or a short one is in force before the start's own count |
| [exti.md](exti.md) | EXTI + SYSCFG: where this family keeps its pin interrupts, sixteen lines numbered by the pin number and multiplexed by SYSCFG's EXTICR (another block, another clock gate, closed at reset), the rest peripheral wake-ups whose existence the reserve reads off the peripherals; one rc_w1 pending bit per line and only while its interrupt is unmasked, the software trigger that needs no pad and does not clear itself, the seven pin vectors and the ISR body that clears first, and an EXTI event out of WFE in 0 us |
| [usart.md](usart.md) | USART: the classic SR/DR/BRR block (flags cleared by read sequences, the divisor in sixteenths or eighths, ten instances at most with the U(S)ART spelling deciding what a full one has), the resource over the whole chapter (mute, LIN, IrDA, smartcard, synchronous, flow control, DMA requests) and the `Uart` task with the other strata's surface; the console byte-exact on the four boards at 115200. Against the desk's STM32G0B1RE running `usart_peer` on the crossed USART1 pair (the second all-key `t` of `test_stm32f4_serial`, 150 verdicts): two crystals within 2 ppm measured and 10 derived, every rate of the two dividers' common ladder to 6.4 Mbaud timed by the far receiver within 25 ppm of its divisor and the far blast within 30 of the rate the far end states, both receivers whole, the frame matrix, the FE onset, the start bit's vote; ES0298 2.13.2 reproduced in the erratum's order and absent from the task, 2.13.1 reproduced in its own case (one idle frame of a sender 3 % fast not told, the far STM32G0 reading the same table though its sheet lists nothing) while the receive engine's edge held at a gap of a frame, 2.13.3 reproduced after a software mute and absent after the hardware's own, mute mode's two wakes with the 4-bit address on 8- and 9-bit words, the far engine's start threshold and operational range, three copies starving a ring ranked low with a wire as the sender, one open-drain wire between the two chips, LIN's break at LBDL's two lengths (and with LIN mode off, against 25.4.8), IrDA SIR both ways against the far ST ENDEC, `set_baud()` moving a running task to 9600, 921600 and 2 Mbaud, TCIE and LBDIE timed. This stratum's own `usart_peer` is the instrument a far board drives |
| [rtc.md](rtc.md) | RTC and the backup domain: a BCD calendar in a power domain of its own behind two locks (PWR_CR.DBP over all of it, the RTC's own key over most of it), RTCSEL one-way and BDRST the way back; the shadow registers and the three errata around them, both alarms, the wake-up timer, the smooth and coarse calibrators and their interlock, the sub-second shift, the timestamp and tamper that share RTC_AF1 with the two outputs; twenty backup registers that outlive every reset but the domain's own, the crystal weighed at 32769.18 Hz with a 1 Hz counted on the pad with no instrument; the 8- and 16-second calibration cycles at the 32-second scale, CALP refused under PREDIV_A 3 (the silicon drops it in silence), the hour, date and weekday alarm matches, the alarm output inverted and in open drain |
| [dma.md](dma.md) | DMA: two controllers of eight streams, each with its own FIFO, priority and vector, choosing between eight request lines with CHSEL, so a peripheral reaches only the one or two (controller, stream, channel) cells the request mapping gives it, and the mapping is a per-part-class fact no header carries; table 49's burst-and-threshold arithmetic refused before the enable, the double buffer swapped in hardware, memory-to-memory on DMA2 alone as the wireless instrument (2048 bytes in 1954 core cycles at 180 MHz), and the two engines the Uart task's slots had been holding empty; every engine at the level its peripheral's starvation decides (a receive ring armed low beside three copies on its controller loses frames, at very_high none) and a raised request held through a gated clock, which every transport's release takes away with its block's reset |
| [tim.md](tim.md) | Timers: fourteen instances of one register block whose geometry is the manual's and not the header's (two 32-bit counters, one break unit per advanced-control timer, TIM9/TIM12 slaving without an encoder, TIM9..TIM14 with no CR2 at all), the counter clock that is HCLK or twice its APB clock (and RCC_DCKCFGR's TIMPRE), the rc_w0 status register, four vectors on TIM1 and TIM8 with three of them shared with a small timer, and the two things that make the chapter measurable with nothing attached, the internal triggers and the option registers that put the LSE, the LSI or HSE_RTC on a capture channel; the crystal weighed at 32769.18 Hz and the dead time at 39.9 ns per hundred; on the STM32F446 the six instances the F411 has not (TIM6, TIM7, TIM8 and TIM12..14) counted at their own lines, the centre-aligned modes told apart by their flags' direction, BDTR.LOCK's levels, the one-pulse task's pulse one count too wide, fixed; a polling read of a timer's pad dithered, because a tight loop locks onto the waveform it samples |
| [adc.md](adc.md) | ADC: up to three converters sharing one PCLK2 prescaler, one reset and one vector; the regular sequence of sixteen ranks and the injected four that preempt it with a signed offset, the four resolutions and their conversion times measured to the ADCCLK cycle, the watchdog whose thresholds are compared before the alignment, the internal channels the manual and not the header numbers, the multi-ADC modes and what really loads their common data register, the DMA cell each converter's request sits on; VDDA and the junction temperature from the factory measurements, both errata staged; every timer trigger code of both tables measured on its timer, the edge senses, the watchdog interrupt |
| [dac.md](dac.md) | DAC: two 12-bit channels whose outputs reach a pad and nothing else, which is what lets the ADC read them back with no wire; the holding register and the output register a cycle apart, the buffer measured against a real load, both wave generators with the LFSR's first two words exactly, a table played by a hardware trigger through the channel's own DMA cell with the underrun a spent stream leaves behind; absent on the F401, F411 and F412, where the resource does not exist; the underrun interrupt on TIM6's shared line, every trigger after the first with no stream |
| [spi.md](spi.md) | SPI and I2S: one block wearing two faces, the F1 lineage's SPI with no FIFO and eight rates off the instance's own APB clock, three chip-select arrangements, the simplex and bidirectional line modes, the CRC unit whose polynomial must be odd, flags cleared by read sequences; `SpiHost` with the other strata's Request verbatim over the request mapping's own cells, `SpiClient` beside it, and the audio face with its own PLL and the I2SDIV/ODD arithmetic. Measured against the gyroscope and the display controller a board carries on SPI5 (the rate ladder with the polled pump's constant 241..300 ns a frame, the engines byte-exact in 8- and 16-bit frames with one interrupt for an engined data phase, a mode fault raised by SSI alone and visible sixty core cycles later, the bidirectional read that needs the pad's output stage let go by hand) and on the Nucleo-F446RE's SPI2 against the STM32G0B1RE paired with it: `SpiClient`'s polled runs written on TXE exact to 16 MHz from that host, the host to PCLK1/2 = 22.5 MHz polled and on DMA1, a mode fault from a second chip pulling NSS low, the arbiter's timeout and `recover()`, the pump's two-in-flight overwrite measured and closed, the pads' slew classes as configuration of both tasks and correctness parameters on jumper wires; through spi_link's extended exchange both ways (16-bit frames, the half-word streams at the wire's own time to 22.5 MHz, the transmit-only shapes witnessed by a listening client, `receive()` and odd counts, the CRC unit against the G0's with a corrupted CRC frame caught, the same CRCNEXT window as the G0's, ES0298 2.14.5's CRC signature at PCLK1/2 at every SCK class, the received CRC wrong and every data frame exact); a client with no select pad (SSM) and SSOE framing the far client, a stall ending `spi_stalled` with ERRIE's vector on the same MODF, a client answering on TXEIE alone, and the hold-off declared honestly: a 375-cycle handler overruns the default's two in flight in every window and none on a host declaring 400 |
| [i2c.md](i2c.md) | I2C: the F1 lineage's event machine, a tenure that advances only when software runs the sequence each flag prescribes, and a receive procedure that is three different choreographies by count, over three instances with two vectors each and three timing registers (FREQ, CCR under F/S and DUTY, and TRISE, which is the wire's rise time and not a guess); `I2cHost` with the other strata's Request verbatim over the request mapping's own cells, all on DMA1, `I2cClient` beside it, the errata as code (a controller's BUS ERROR counted and never acted on, SWRST the way back, the repeated start's setup time published where it is at risk); measured against the STMPE811 the STM32F429I-DISC1 carries on I2C3 and the FocalTech touch controller of the 32F469IDISCOVERY's display board on I2C1 (one address of 112 answering on each, the three receive procedures byte-exact against each other, SCL counted on its own pad while the DMA carries a 32-byte read, a finger's coordinates read with its INT line counted). The trap: BUSY stands out of reset over an idle wire, because it watches the peripheral's own inputs and those read low while the pads are not yet in their alternate function. Against the desk partner's `twi_peer` (an STM32G0 on the Nucleo-F446RE's wire): stretching as flow control, `unstick()` against a client really holding SDA, the per-bus timeout over this engine, and `rebase()`, which derives the APB rate from the new SYSCLK |
| [fmpi2c.md](fmpi2c.md) | FMPI2C: the second I2C design of this family, on the F410, F412, F413/F423 and F446, the STM32G0's register file under the name FMPI2C1 (one TIMINGR word of five fields, a byte counter with RELOAD and AUTOEND, ISR/ICR, two own addresses with a mask), so the driver is that driver with an `Fmp` prefix, two vectors instead of one, a kernel clock of its own (the APB, SYSCLK or the HSI, which buys Fast-mode Plus at 1 MHz and a bus that survives a core rate change) and no wake from Stop; the errata as code, `FmpI2cHost` with the other strata's Request verbatim. Measured on a bus with nothing on it: 112 addresses of 112 NACKed with the STOP the peripheral sends by itself, SCL counted at three speeds and three kernel clocks, the idle time-out at 51.1 us where 50 was asked. The finding: the manual's assumed SCL detection delay is generous (160..390 ns really on a 2.2 kOhm wire) and so is ST's own timing utility's, so the chooser charges the floor under it unless told the wire's, and a speed asked is a ceiling: no row of eight measured above it, a stated wire within one per cent of it, and the classic I2C1 beside it at its own stated rate. Against the desk partner's `twi_peer`: Fast-mode Plus with every byte acknowledged, TIMEOUTA firing at 25 ms on a clock the other chip holds, `unstick()`, the per-bus timeout and `rebase()` at three core rates |
| [usb.md](usb.md) | USB OTG, device mode: a Synopsys DWC2 core, up to two of them (the full-speed one on PA11/PA12, the high-speed one's own full-speed PHY on PB14/PB15), whose whole interface is FIFOs reached through push and pop registers, with one shared receive queue for every OUT endpoint and a slice of RAM per IN endpoint the program maps by hand and the silicon never checks; the address that must be in DCFG before the status stage of SET_ADDRESS, against chapter 9 and measured; the OUT transfer of several packets that is this core's answer to the NAK a hub charges a frame for; 658 KB/s out and 440 KB/s in on the black pill's own connector, with the kernel console over it |
| [pwr.md](pwr.md) | PWR, the sleep sites and Standby: the mode as two registers in two places (SLEEPDEEP and PDDS), a Stop whose price is four more bits (LPDS, FPDS, the low-voltage pair, under-drive) and whose exit puts the core back on the HSI with the PLL, the HSE and over-drive gone but the VOS field untouched; `util/power.hpp`'s ladder over it with Standby enforced off it, and the timed site that meets a deadline through a Stop on the RTC's wake-up timer; the flash's 92 us of wake reproducing the datasheet's typical, a 213 us restore to 180 MHz, a 500 ms event met at 500 ms of wall |
| [fmc.md](fmc.md) | FMC, the external memory controller: the one peripheral that adds address space instead of driving a wire, and the one chapter with three answers to "which block does this part have" (the FMC on the F42x/F43x, the F446 and the F469/F479; the FSMC, another peripheral with no SDRAM half, on the F405 class, the F412 and the F413/F423; neither on seven parts of the pack); six windows of the Cortex's own map whose decoder answers before the memory does, so a read of an SDRAM bank that has not run its sequence hangs the machine with no fault and `initialize()` is one verb that runs the whole of it or answers false; the SDRAM controller and the NOR/PSRAM half in the chapter's words. Measured against the 64-Mbit SDRAM a board carries: 8 MB byte-exact in three access widths at 80 MB/s written and 33 MB/s read, a DMA stream at twice the CPU's rate going in, a single read enough to leave self-refresh, TRC the one delay with a measured floor, an array that keeps its bits for seconds with nothing refreshing it. The two traps, both against the manual: a COUNT of zero does not reach SDRTR (the silicon leaves 40 and the timer runs on), and SDCR1.SDCLK = 00 does not stop FMC_SDCLK, only the block's own reset leaves the device unclocked |
| [ltdc.md](ltdc.md) | LTDC, the LCD-TFT display controller: the one peripheral of this family whose output is a picture, two layers, eight pixel formats, a colour table each, blending and colour keying, out of whatever memory holds the frame buffer, with a PLL of its own for the pixel clock that has no input divider of its own (it divides the main PLL's M, so a pixel clock is an exact triple or nothing), three clock domains visible from the bus (every pixel-clock register reads zero with no pixel clock), a shadow register behind every layer register but the colour table, and a read straight after the reload store that races the reload, which `reload()` covers with one access of its own. Measured against the 240x320 panel this board carries, taken into its RGB mode over SPI5: the pixel clock exact to 0 per mille, the line event once a frame, 16.4.1's own example caught printing the two TOTAL fields without the minus one its own rule asks for, an underrun flag that exists only while its enable is set and is raised per pixel, and the number the display tier lives on: two layers of 32-bit pixels at 65 Hz plus the accelerator on the same SDRAM, no starvation, some 160 MB/s over the bus in every configuration |
| [quadspi.md](quadspi.md) | Quad-SPI memory interface: a command engine of five phases on one, two or four lines in front of a serial flash, with three faces (indirect, automatic status polling, memory-mapped: a load from 0x9000 0000 is the read command); a command that starts on the last register it needs, the flash's size and NCS's high time as part of the contract, the window and the DMA cell the manual gives and the header does not; four of the five errata as code, and the flag a poller must not share with a handler. Measured against the 32F469IDISCOVERY's MT25QL128: the identity and the SFDP table, the last subsector erased through status polling and programmed a page at a time, five read commands byte-exact (quad I/O at 36 MB/s on the pump, 44.5 MB/s through the window), the dummy cycles against the device's own table, the FIFO stalling the clock, the abort, the error past the size, deep power-down |
| [dsi.md](dsi.md) | DSI host: the MIPI Display Serial Interface in front of the LTDC on the F469/F479 class alone, the wrapper's regulator and PLL, the two-lane D-PHY on dedicated pads, the host's two ways to a panel (video mode packing the LTDC's frame into pixel packets, the adapted command mode turning each frame into DCS memory writes launched by a refresh or by the tearing effect pulse) and the generic interface a panel is spoken to through; the PLL's ranges as two documents state them and the solver on their intersection, the unit interval in quarter nanoseconds, the frame converted into lane byte clocks with its rounding, two of the three errata as code. The panel decides between the two ways, and the rule the manual does not state: in video mode the generic interface lives inside the stream. Measured against the 32F469IDISCOVERY's panel, whose controller is read over the link (an NT35510 by its three ID bytes): every DCS register written and read back, one refresh of the frame in exactly one LTDC frame and 61 a second, the picture read back out of the panel's memory pixel for pixel, a second of video-mode frames leaving that memory untouched, the tearing effect pulses counted on the pin and by the wrapper alike, ULPS, the module's touch frame solved against the display from ten taps on a square, and the trap: a panel reset under a clock lane already in high speed takes no high-speed packet afterwards, so the clock lane goes to high speed after the panel's bring-up |
| [dma2d.md](dma2d.md) | DMA2D, the Chrom-Art accelerator: a DMA that knows what a pixel is, rectangles and not runs, four modes over eleven input formats and five output ones, two colour tables, a blender with a stated rounding, and a bus master of its own, which is what makes its reads of external memory the ones ES0206 2.3.5 allows; present on two parts of the class that have no display interface at all. Every mode measured byte-exact against a software model (16.4.2's channel expansion, 11.3.11's blend to the unit, the five output packings) with the configuration error raised by the silicon where the driver's own refusals do not reach; the colour table loaded both by the CPU and by the engine; the bandwidth beside a running display, where the AHB dead time is the one knob. The trap is the compiler's: a buffer this engine writes must be volatile, because its address reaches it as an integer nothing dereferences |
| [crc.md](crc.md) | CRC calculation unit: three registers and one function wired in, the Ethernet polynomial with no knobs, which is CRC-32/MPEG-2 exactly, a word at a time because the register takes nothing narrower, and the software model beside it that the silicon is judged against; the reset that lands one read late and swallows a word written in the instruction after it, and CRC_IDR, eight bits of scratch the reset spares and the RCC line clears |
| [rng.md](rng.md) | Random number generator: ring oscillators feeding an LFSR on the 48 MHz domain, with the manual's only clock constraint being a ratio (f(RNG_CLK) below f(HCLK)/16 raises CECS) and not a rate, the first value discarded and every next one compared as FIPS PUB 140-2 asks, two error statuses of different kinds whose latched flags are rc_w0, the gate that moves to AHB1 on the F410 and the vector that wears two names; absent on the F401, F411 and F446; measured on the STM32F429: a thousand words a millisecond at 45 MHz, the ones 499 per mille, a computed word that survives the disable |
| [can.md](can.md) | bxCAN: up to three instances and one filter block for the first two, split at CAN2SB (28 banks in four shapes whose match index is not the bank number), the bit time searched exactly at compile time from PCLK1, three mailboxes ordered by identifier or by request, two FIFOs of three whose overrun policy is a choice, the error counters walked to bus-off and back in 1407 us (by ABOM or by the software request), the transmit, FIFO 0 and status vectors under traffic (and a master reset that leaves completed mailboxes standing), rates from 10 kbit/s to 3 Mbit/s in loopback, and the loopback that still needs eleven recessive bits on its receive pad; a frame type of this stratum's own, because the util CAN vocabulary waits for a second family |
| [flash.md](flash.md) | The embedded flash memory interface: an array of uneven sectors (four of 16 Kbytes, one of 64, then 128 to the end of the bank, a rule the driver computes the map from rather than tabulating it), the program width declared twice (PSIZE and the store instruction, so the parallelism is a template parameter and x64 is refused for want of a VPP supply), a keyed lock whose wrong sequence is an imprecise bus error and a lockout until reset, the option bytes read whole with nWRP the one of them a running program may write; the two facts this family turns on, both measured: a 128 Kbyte erase freezes the core for a second (the kernel tick advances 1 ms, and with the ART off not one poll turn completes), and a second program into a programmed word is accepted and ANDs into the array while the D-cache answers with what was written |
| [vendor/README.md](vendor/README.md) | The reference manuals, datasheets and errata by revision, the vendored cmsis-device-f4 headers and SVDs, the four bench parts' identities (DEV_ID, REV_ID, the errata sheet each keys into) |
