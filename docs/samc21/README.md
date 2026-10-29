# Target: SAM C21 (`samc21/`)

The operational page of brio's SAM C21 target, for a programmer who has
the SAM C21 board on the desk and wants to build, flash and debug a
brio program on it, and then find the document of the peripheral the
program needs. It says what the stratum is, which board and part it is
proven on, how the build, the upload, the debugger and the console work
and where each has a trap, and it ends with the map of the stratum's own
documents: one line per peripheral driver, which is where the state of
the driver work is read.

A stratum is a directory of `brio/` holding one architectural layer;
`brio/samc21/` holds everything that includes Microchip's device header
`sam.h`. The SAM C21 is an ARM Cortex-M0+ family (the ARMv6-M
architecture), and what is the core's rather than Microchip's lives one
layer below, in the `cortexm/` stratum the STM32G0 and the RP2040 share
([../cortexm/README.md](../cortexm/README.md)). The kernel (`kernel/`)
and the services (`util/`) run here unchanged; the crt (the linker
script, the start-up code and the vector table) is the project's own
work rather than the toolchain's. The bench chip is the ATSAMC21J18A on
a self-built board; the E and G variants of the part are covered by a
compile check ("Family coverage" below). The maintained example `blink`
builds for the board.

The documents of record (the datasheet and the errata by number), the
errata pass item by item and the identity of the bench die (silicon
revision F, the DSU DID read over SWD) are in
[vendor/README.md](vendor/README.md).

## The board

The bench board is a self-built C21J rev 1.1, not the Xplained Pro
([../boards/samc21j.md](../boards/samc21j.md)): an ATSAMC21J18A with the
CPU on the internal OSC48M at 48 MHz; a 24 MHz crystal on PA14/PA15,
which is not the default clock root but a real oscillator the suites
start and measure against ([clock.md](clock.md)); the console on a
CH340 bridge on PB30/PB31 (SERCOM5 PAD0/PAD1); the user button on PB22,
the LED on PB23, SWD on PA30/PA31.

## Toolchain

The compiler is a self-built arm-none-eabi-gcc 16.2 at
`/sw/arm-none-eabi`, the same vintage as the AVR and host compilers and
never a system-packaged one, named by absolute path in
`samc21/cmake/toolchain-arm.cmake` (`CMAKE_SYSTEM_NAME Generic`, a
`STATIC_LIBRARY` try-compile: the standard bare-metal pattern, under
which no executable links until the linker script and the start-up code
are supplied per app).

Unlike avr-gcc, this toolchain ships no device headers, so the CMSIS
device pack is vendored in the repository and a fresh clone builds:
`third_party/samc21-dfp/` (Microchip's SAMC21_DFP include tree, every
C21 E/G/J variant) and `third_party/cmsis-core/` (the five CMSIS-Core
headers the device header needs). The device is selected by an ordinary
define (`-D__SAMC21J18A__`, derived from the `SAMC21_MCU` cache
variable) that `sam.h` dispatches on; there is no device-specs
machinery and no `-mmcu` equivalent, which is also why clangd needs no
macro feed on this target.

The link is `-nostartfiles -nodefaultlibs` with brio's runtime and
libgcc: no C library and no syscall layer. brio allocates nothing and
calls no syscalls, so anything dragging `printf`, `malloc` or a syscall
in fails the link loudly instead of failing at run time; `abort()` and
the four `mem*` functions the compiler calls are the runtime's
([../design/runtime.md](../design/runtime.md)).

## Build

`samc21/` is its own CMake project, a sibling and peer of `avrxt/`,
`stm32g0/` and `test/` (one configure has exactly one compiler; the
repository root is not a CMake project), so every CMake command runs
from `samc21/`:

```bash
(cd samc21 && cmake --preset samc21j-release)                       # configure (once, or after adding an app)
(cd samc21 && cmake --build --preset samc21j-release --target <app>)
(cd samc21 && cmake --build --preset samc21j-release --target <app>-upload)
brio check samc21 [name]                                        # family smoke, no hardware
```

One configure targets one chip variant (`SAMC21_MCU`); only the J18A
has presets today, `samc21j-release` and `samc21j-debug` (CMake Tools
shows them as `SAMC21J18A (release, -Os)` and `SAMC21J18A (debug,
-Og)`), because the E and G variants have no board on the desk and are
compile-checked instead ("Family coverage" below).

Applications are discovered at every configure from
`samc21/src/apps/*.cpp`, plus `experiments/*/samc21/*.cpp`, the SAM half
of any top-level experiment directory (which documents itself in its
own README), by the `// build:` lines in each file's header comment,
the same grammar as the AVR project with this family's names:

- `// build: boards = c21j` names the board type; `c21j` is also the
  default when the line is absent.
- `// build: groups = rmptuei,d` splits a test suite whose whole image
  would leave its stack too little of the 32 KB of RAM into one image
  per group of letters, `<app>-1`, `<app>-2`
  ([../design/overview.md](../design/overview.md), "A suite's image
  fits the family's smallest chip").

Build outputs land in `build-cmake/samc21j-{release,debug}` at the
repository root: `<app>.elf`, `.bin` and `.hex`, `firmware-<app>.map`
and `<app>.lst`, the source-interleaved disassembly. A configure also
writes this project's app roster, `build-cmake/apps_samc21.json`, which
`bin/brio` reads.

`bin/brio` is the repository's one command for boards and builds; a
verb is the action word after `brio`. The board's type in the bench
manifest, `c21j`, is what tells the tool to build in this project and to
flash through OpenOCD instead of avrdude, so a SAM suite is driven
exactly like an AVR one: `brio flash <board> <app>`, `brio run <board>
z` ([../boards/README.md](../boards/README.md)). The `-upload` target
remains the flash path that needs no manifest.

## Upload (OpenOCD, SWD)

Flashing goes through OpenOCD driving the Atmel-ICE as a CMSIS-DAP
probe ([../probes/atmel-ice.md](../probes/atmel-ice.md)):
`interface/cmsis-dap.cfg` plus `target/at91samdXX.cfg` (the at91samd
flash driver auto-probes the geometry from the DSU DID) and `program
<app>.elf verify reset exit`. OpenOCD is the 0.12.0 release, built from
its tarball into `/sw/openocd-0.12.0` (reached as `/sw/openocd`) by
`/sw/src/build-openocd.sh` the way the toolchains are, with the
CMSIS-DAP adapter on its hidapi backend; it drives the ICE without
incident. With more than one Atmel-ICE attached, `adapter serial` is
what tells them apart (the `SAMC21_PROBE_SERIAL` cache variable, empty
by default, which means the only one attached; `bin/brio` takes the
serial from its manifest instead). The probe serves one client at a
time: close the debug session before flashing.

The probe page carries two facts of this path that a reader of a
silent board wants: the HID backend is chosen because the USB bulk one
loses its place behind a hub, and every SAM flash ends by clearing
DHCSR.C_DEBUGEN, without which a `panic()`'s breakpoint halts the core
in silence instead of faulting ([reset.md](reset.md)).

## Debugging (cortex-debug + OpenOCD)

The launch configuration is "Debug SAM (OpenOCD, C21J board)" in
`.vscode/launch.json`: `servertype: openocd` (cortex-debug launches the
server itself and parses its output natively, so none of the buffering
shims PyAvrOCD needs on the AVR), the same two `-f` config files, the
`adapter serial` through `openOCDPreConfigLaunchCommands` when more
than one probe is attached, gdb from the cross toolchain,
`runToEntryPoint: main`, and `svdPath` at `samc21/svd/ATSAMC21J18A.svd`
feeding the same mcu-debug Peripheral Viewer the AVR side uses. Two
choices in CMake Tools must agree before a session starts: the Active
Folder `samc21/` and the launch target the app to debug (the
executable comes from
`${command:cmake.launchTargetPath}`, exactly as on the AVR entry).

Verified at the bench: the stop at `main`, user breakpoints hit. By
policy the verification is light: cortex-debug and OpenOCD are mature
tooling, where PyAvrOCD on the AVR side needs exhaustive checking, and
quirks get documented here as they are found. One SWD technique worth
knowing: with the CPU halted, `mdw` and `mdb` from OpenOCD read any
peripheral register, which settles a wrong-looking serial line in one
pass where guessing host baud rates does not.

## Editor (clangd)

clangd routes every file to its own architecture through `.clangd`
fragments, independent of which project CMake Tools has active:
`brio/.clangd` sends the framework default (`kernel/`, `util/`, `host/`)
to the host database, `brio/samc21/.clangd` and `samc21/.clangd` send
the samc21 stratum and project to `build-cmake/samc21j-release`, and
`test/family_samc21/.clangd` lets the script-compiled family TUs borrow
flags from that same database. The repository root's `.clangd`
suppresses the two clang-only diagnostics that the inherited `-Werror`
would otherwise render as editor errors on gcc-clean code, and tells the
include-cleaner that `sam.h` is the intended umbrella (including the
device header directly is exactly what the code must not do).

## Serial console

The board's CH340 enumerates without a USB serial number, like every
CH340 on this desk, and `/dev/serial/by-path` names move with the USB
socket: identify the SAM console by listening on the candidates during
an OpenOCD `reset run` and seeing which one prints the boot banner.
Console apps run at 115200 8N1.

## Family coverage

The stratum is written for the SAM C21's three variants, E, G and J,
which differ in package and bonding. `brio check samc21` proves it
without hardware: every positive translation unit in
`test/family_samc21/` compiles across the E, G and J 18A headers, and
every `neg/` TU, a program that must be refused, fails to compile for
the variants its `// mcu:` line names. The check ends with the link
guard of every app on the project's release preset. The bench proves the
J18A alone; for the E and G variants the sweep is the only check there
is.

## The documents

One document per peripheral driver, in the shape
[../README.md](../README.md) prescribes: the reader's orientation, the
documents of record, what the silicon does, the types and verbs, how to
use it, the bench findings and, where the driver has gaps, "Not covered
yet" as two lists. The line beside each names what the document is
about and the finding or two most worth knowing before using the
driver; the findings themselves, with their figures and conditions, are
in the document. The power manager (PM) lives in `platform.md` with the
rest of what the kernel stands on.

| Document | Content |
|----------|---------|
| [platform.md](platform.md) | What the kernel stands on: `SamPlatform` (the PRIMASK critical section, a WFI idle that takes whatever PM.SLEEPCFG holds, BKPT and its ARMv6-M caveat, the `.noinit` breadcrumb, atomic_width 4) and `SamRubatoPlatform`, the same for the preemptive kernel (the ceiling and cortexm/schedule.hpp's PendSV request, the SVC vector bound as this crt names it, `SVCall_Handler`), `BasicTicker` over SysTick, `Nvic`, the hand-written crt and its `.fast_text`, where a program places the vector it binds and a driver given `InRam` its path (a handler in SRAM: a third off every entry with the vector placed, the console's 149 cycles to 100 and the tick's 40 to 28, a tenth at most for an `InRam` body behind a vector in the flash); and the stopping half, PM ch. 19 (`Pm`, three sleep modes, the readback rule, the regulator and back-bias knobs) with `SamSleepSite` running util/power.hpp unchanged. Measured: the idle hook walked by an edge at every cycle of its path in IDLE0, IDLE2 and STANDBY with no wake lost or late, one kernel turn per tick and a quiet turn about 250 cycles; the SLEEPCFG bridge latency about 5 us; IDLE2 costing 24..30 us more to leave (15 us of it the flash's sleep power reduction, which SLEEPPRM = DISABLED removes) and STANDBY about 106 us, nothing in the regulator or back-bias settings moving it; the kernel tick frozen in standby exactly, so the plain site's rule is "no armed time event, then standby" and `SamTimedSleepSite` lifts it with the RTC as alarm and witness (a 500 ms event through a standby matures at 507 ms of wall, six rounds and not one early); a peripheral's own RUNSTDBY the whole clock request; XOSC running through standby whatever RUNSTDBY says, so no crystal-restart bill; the preemptive platform on the SAMC21J18A by `test_samc_rubato`: road 1 in 232 cycles from the interrupt's last statement to the preempting dispatch's first, a quiet post 107, a road-2 post 654, a post under PRIMASK served when the section ends, a PendSV pending at the idle's WFI ending it in IDLE0 and IDLE2, 580 bytes of stack for the suite's deepest chain; and by `test_samc_rubato_link`, the SPI and UART stacks under an object preempting at prime periods against the AVR64EA48, byte-exact on both SPI paths to 2 MHz with the polled loop held off for any length losing no frame, a handler-bound pump at 3 MHz holding TC4 off past its period, both UART transmitters and the line loans clean. Not covered: sleep current (no meter), chapter 19's write protection, SLEEPONEXIT, whether a clock runs in standby with nothing requesting it, the BODVDD as a wake source, the SRAM handler's full gain until the console's per-byte path is inline, a PendSV pending at a STANDBY WFI |
| [clock.md](clock.md) | All three OSCCTRL roots as resources (OSC48M, the external crystal with its clock-failure detector, the fractional DPLL with the LDR/LDRFRAC arithmetic and an actual-hertz readback), GCLK/MCLK, bounded waits everywhere, flash wait states ordered around the change, five errata as code (two OSC48M, three DPLL). Measured: the internal RC 5100 ppm slow against the board's 24 MHz crystal, the first non-RC scale this stratum has had, which corrects every absolute frequency measured before it; a clock failure induced with no wire and the safe-clock switch observed; the DPLL's ratios exact at 48, 49 and 96/2 MHz; the CPU run from the crystal-locked DPLL and brought back; GENCTRL's DIVSEL is 2^(DIV+1). Not covered: the main-clock task implements `internal` only, DynamicClock, external-clock mode, sleep |
| [port.md](port.md) | PORT: two groups on every variant, the off-by-default input buffer, the directional pull, the PMUX handoff, WRCONFIG; not covered: PORT events (senses and pin interrupts are the EIC's, [eic.md](eic.md)) |
| [sercom.md](sercom.md) | SERCOM USART: the one-vector reality and the INTFLAG-and-INTENSET discipline, pads against pins (TxD on PAD0/PAD2 only), the LSB-first default, enable protection and both sync waits, the 16x baud arithmetic byte-exact on the wire, every SAMPR regime, the synchronous host and client, the IrDA coder, the four TXPO pinouts (RS485's TE with GTIME, RTS/CTS), the LIN host's header and auto-baud, collision detection under erratum 1.17.12's clock rule, start-of-frame detection; `Uart` with its one optional DMA engine slot, the transmit one (the family's one DMA user, a second owner a panic; no receive slot, erratum 1.10.4), its release that resets the SERCOM before its gate, and its placement (`InRam`: the transmit completion 220 cycles to 197 behind a vector in the flash, 145 with the vector placed). Measured on the loop-back through the pad and across the wire against an AVR64EA48: every regime's rate judged by the far crystal (2400 baud to 2 Mbaud, 998 to 1003 thousandths to 3 Mbaud on the loop), the frame matrix and the asymmetry of a frame mismatch, the receiver's votes on the pad, auto-baud learning foreign rates, the LIN header's gap, TE exact, a collision that wedges the transmitter until a reconfiguration, a disabled transmitter still driving its pad, `tx_idle()` the wire's, the two-level receive buffer an entry each (3 Mbaud kept). Not covered: CTS high on the bench, the asynchronous mode on an external clock, a receive engine (declined, [dmac.md](dmac.md)) |
| [spi.md](spi.md) | SERCOM SPI: `Spi<n>` over one register view for both roles, DOPO refused as the triple it is, `SpiHost` with the shared Request on the polled loop or the byte pump (no DMA: declined, [dmac.md](dmac.md)) and its placement (`InRam`: the receive loop 42.4 cycles a frame to 38.9 at 12 MHz), `SpiClient` with its polled runs (`answer`, `exchange`, `receive`, written to the three-SCK-cycle rule's five-cycle window). Measured against two peers: a mode change is a CPOL flip on the wire (hence `prime()`), the three-SCK-cycle rule counts only while SCK runs, MSSEN raises SS between every character, the client's runs exact against an AVR host to 5 MHz in every ladder and at 10 MHz in ten of eleven; the SCK ladder on the wire, a frame in 1006 thousandths of its SCK periods from 93.75 kHz to 8 MHz on the crystal |
| [i2c.md](i2c.md) | SERCOM I2C: `I2cm<n>` and `I2cs<n>` (two resources, the register sets differing), the SYSOP wait-before-store, force_idle as the UNKNOWN exit, the chapter's BAUD arithmetic solved both ways, `I2cHost` with the shared Request (no placement: the program places its vector, a third off an entry) and `I2cClient` with the errata discipline built in. The headline is a wire finding: this I2C samples on GCLK_CORE with no input filter, so a bundle's crosstalk reads as false Starts (a ladder by core rate) while a clean pair runs both cores at 48 MHz with Fm+ on the wire; the SMBus time-outs police the host's own hold and not the wire, which is why the per-bus timeout of util/bus_master.hpp exists (letter l is its silicon witness); SCL sampled on the pad at the three speeds, and a rise budget longer than the bus's running SCL fast; the three SMBus counts as this silicon makes them (MEXT 14.7 ms, SEXT with LOWTOUT 29.3), the quick command counted on SCL, the address-match wake from STANDBY, two live hosts colliding on the wire |
| [dmac.md](dmac.md) | DMAC: one use, the Uart's transmit engine on channel 0, and every other use declined. Erratum 1.10.4 (live on E/G/J at rev E, F, H) corrupts the live write-back of concurrently triggered channels, measured: the SPI full-duplex pair clean alone in 24.8 M requests and hung in 4..8 % beside any third channel with silent bad completions, two channels clean in every shape and three faulted in every shape; the one-user rule enforced by structure and by the claim (a second owner's init() a panic); the engine at level 2; a released requester measured: a disable withdraws DRE's request, a gated clock holds one left standing and hands the next binding one stray beat, the channel keeps none |
| [reset.md](reset.md) | RSTC + WDT: RCAUSE as the exclusive one-cause register it is (the AVR's accumulating habit does not travel), what each reset does and does not clear, the watchdog whose reset values are fuses and whose CLEAR key resets whether it runs or not, the early-warning interrupt used to measure OSCULP32K by difference, and the panic breadcrumb proven across six real resets including a HardFault; not covered: SUPC behind two of the causes, always-on mode (one-way), VTOR |
| [nvm.md](nvm.md) | NVMCTRL: the two arrays and the one that does not stall the CPU (about 3950 polling turns survive an RWWEE row erase, one survives a main-array one), erase-by-row/program-by-page, the page-buffer rule measured narrower than the chapter's (one 64-bit section at a time), ADDR proven section-relative, region locks, the fuse row and the factory calibration typed, the RWWEE array partitioned between `RwweeFlash` (util/nv_heap.hpp's SAM realization) and `RwweeJournalZone` (util/nv_journal.hpp's first), the user row written by the bench tool over SWD and never from firmware; not covered: the security bit, a main-array backend deliberately not built |
| [ac.md](ac.md) | AC: the four comparators, the VDD scaler, filters, both output routings, window mode over the comparator pairs, both event directions with the EVSYS codes published here, per-package input legality (the E/G variants bond only AIN[5:4] for COMP2/3). Measured: a synchronized output edge costs the fraction to the next GCLK_AC edge plus two whole periods, all three window states reached with no analog source, a comparator flip and a window transition each counted by a TC on the event fabric, a pin edge starting a comparison through the AC's SOC0 user; not covered: sleep, the DAC and bandgap inputs |
| [evsys.md](evsys.md) | EVSYS: the one place the AVR shape does not transfer: twelve identical channels with numeric generator/user codes and an allocator where the AVR has a typed table, so this driver owns the fabric and each peripheral publishes its own codes; the user multiplexer's channel+1, the ordering rule, the three paths and the path/edge legality both ways, three live errata as code; proven end to end by a TC counting the events exactly, and a software event on an asynchronous channel reaching a TC's input every time (8/8, 8/8) as it reaches a LUT's edge-detecting input (16/16); not covered: the generator/user tables (deliberate), SleepWalking |
| [eic.md](eic.md) | EIC: where this family keeps its pin interrupts, since PORT has none: sixteen lines plus an unmaskable NMI, the pad-to-line map read out of the device header (irregular and per-package), the five senses, the majority filter, asynchronous detection, one NVIC vector for all sixteen, every line an EVSYS generator; five of the six errata are not this silicon. Measured: a sampled line cannot be enabled without a clock while a clockless one can, a hardware generator does cross an asynchronous EVSYS channel where a software event does not, all sixteen lines generate events whatever 26.6.7's prose says, the crt's core-exception vectors spelled the CMSIS way; not covered: the N-variant debouncer, sleep/wake |
| [tc.md](tc.md) | TC: five timers, three counter resolutions with the 32-bit pairing and the shared generic clock channels read out of the device header, four waveform modes, capture with its event actions, reading COUNT as a command (READSYNC); erratum 1.20.3 as code, 1.20.1 and 1.20.2 revision B only; the tasks PwmChannel and the capture meters behind MeterSource. Measured: the prescaler ratio exact to 4.00x, COUNT32 nine million counts past a 16-bit range, PWM duty and frequency read off the pad by a second timer, capture exact at 2344/937 ticks, a synchronized read proven one behind (0, 196, 201, 205), a MeterSampler running live in the kernel with not one line of util/ changed; not covered: the N-variant capture modes, DMA (declined), ONDEMAND's clock request, erratum 1.20.2 (no controlled edge reaches a muxed WO pad with no wires) |
| [tcc.md](tcc.md) | TCC: the family's richest timer, three instances that are not copies of each other (24/24/16-bit counters, 4/2/2 channels, 8/4/2 outputs, five optional waveform-extension units per instance, all read out of the device header through the reserve), the pad map keyed by pad and function, dead-time insertion, the output matrix, swap, pattern generation, ramps, dithering and both fault systems; seven of the eleven errata live (1.21.10 a compile-time refusal, 1.21.6 code, 1.21.11 unreachable, three caller obligations, 1.21.8 not reproduced). Measured: dead time exact to a stopwatch tick and independent of the prescaler, the dual-slope formula right, a fault from a pin clamping an output, MSYNC moving the channels and not COUNT, a buffered write's SYNCBUSY standing until the UPDATE takes it (so the setters refuse instead of waiting), a CCx read returning the buffered value while the pad shows the old one; not covered: DMA (declined), sleep, the debug fault, the advanced capture modes |
| [osc32kctrl.md](osc32kctrl.md) | OSC32KCTRL: the three 32 kHz roots, the RTC's clock select (which lives here and not in the RTC), the crystal's failure detector, and the fact the bench confirms: OSC32K is not calibrated until software calibrates it (44 % fast without its production trim, six per mille with it); the GCLK rule that a generator cannot be moved off a stopped source; XOSC32K started and weighed on a board that carries the crystal (32.5 kHz on OSC48M's scale) and a false return on one that does not; not covered: XOSC32K's other startup codes, its external-clock mode and failure detector, the crystal's accuracy in ppm, the CFD event output |
| [freqm.md](freqm.md) | FREQM: the hardware ratio counter (f_msr = VALUE/REFNUM x f_ref between two GCLK generators) with erratum 1.24.1 as code (START written and never read), the 24-bit overflow budget, two documentation findings decided by data (CFGA has no DIVREF whatever 44.8.3 draws; the channels must be routed before the software reset); OSCULP32K cross-checked against the watchdog measurement to 3 Hz, its DONE interrupt ending a standby; not covered: the GCLK_IO pins as inputs (a wire) |
| [supc.md](supc.md) | SUPC: the two brown-out detectors, the regulator that cannot be turned off, the bandgap every analog block asks for, the BODVDD whose reset value is a fuse, VREF's VREFOE (what ac.md's gap list was waiting for). Measured: this board runs at about 5.1 V, located three ways through the comparator's VDD scaler; the BODVDD level step 48.7 mV; a store carrying configuration and enable together leaves the protected fields untouched; a sampled detector never reports ready; SUPC_BODCORE real and running at an offset ch. 22 marks Reserved; erratum 1.5.6's spurious comparator flag observed. Not covered: nothing forces a brown-out, standby, BODCORE stays read-only by design |
| [rtc.md](rtc.md) | RTC: one counter wearing three faces (a 32-bit counter, a 16-bit one with PER as its top, a clock/calendar with a masked alarm) behind three overlaid register views, the clock chosen in another chapter (OSC32KCTRL.RTCCTRL, never written here), the read synchronization off at reset; erratum 1.16.3 answered structurally, 1.16.1 and 1.16.2 B and B..E items. Measured: the counter tick for tick on all four clock selects against the board's crystal, the prescaler exact to 70..300 ppm, PRESCALER = OFF dividing by one while silencing every periodic event, a compare and a periodic event each counted by a TC, MATCHCLR raising the compare and the OVERFLOW together, mode 1's period PER + 1, the calendar's leap rule and its year-63 wrap, an alarm a whole second after its match, a synchronized read trailing by a constant four ticks while an unsynchronized one is frozen; not covered: tasks (deliberate), sleep, the frequency correction's per-step linearity |
| [adc.md](adc.md) | ADC: both 12-bit converters over the whole chapter: the reference multiplexer (this target's `brio::Ref` and `ref_mv()`), the prescaler and sampling arithmetic with table 45-22's cycle counts as constexpr, the RESSEL/AVGCTRL interplay whose full-scale arithmetic is util/analog.hpp's `steps`, the window monitor, the digital corrections, the sequencer, both event directions, the factory calibration copied by init(); five of the ten errata live, 1.4.10 live and worse than its own sentence (once ADC1 has run, ADC0 will not enable at all and no reset clears it). Measured: the bandgap channel dead without SUPC's VREFOE while the bandgap as a reference needs no such thing; VDD located at 5201..5276 mV; ADC0 and ADC1 exact on a shared pad and 6 counts of 4096 apart on an internal divider; conversion time exact to the tick in six configurations; MODE4 the complement of MODE3; the correction's 13-cycle latency not observed; util/analog_sampler.hpp running in a real kernel unchanged. Not covered: the dual pair beyond one shared pad, the converter as a wake source, VREFA, the gain correction and LEFTADJ on silicon, INL/DNL |
| [dac.md](dac.md) | DAC: the whole of a small chapter as a monostate resource (one instance on every variant): the three references with `DacRef`, table 41-1's four data placements, the two outputs, the START/EMPTY event pair, the refusals. The geometric gift: VOUT, ADC0/AIN0 and the AC's AIN4 are all PA02, so erratum 1.8.9's loop has zero length and the whole suite is wireless. Measured: the device header's `INT1V` name is the SAM D21's and wrong (the internal reference follows SUPC.VREF.SEL, 996/2011/4057 mV); `Ref::dac` ratiometric to half a per cent; `AdcInput::dac` reads the DAC to one count in 4096 though the code is marked Reserved; erratum 1.8.9's output noise real (3 counts of spread becoming 71..87) while its reading half is declined as beneath the noise floor; `AcNegative::dac` flips at 255/512/769 against 255/511/767; the transfer curve monotonic with a worst residual of 2.5 counts; startup 5.9 us; an event-paced waveform with UNDERRUN when it runs dry; SYNCBUSY.DATABUF not a bus crossing (it stands until a start event consumes the value and every later write is discarded, which is why buffer() never waits); erratum 1.9.2 reproduced with a control. Not covered: VREFA, the voltage pump, the DAC as a wake source, the SDADC's use of the output as a reference, a settling time to a stated accuracy |
| [sdadc.md](sdadc.md) | SDADC: this family's third converter, C21-only: a 16-bit sigma-delta over three differential pad pairs behind a third-order SINC filter, as a monostate resource with its own `SdadcRef`; the whole chapter with the refusals that are its own rules. The central measurement: the datapath is twenty-four bits and "left-adjusted" is not padding (RESULT saturates at +/-2^23, OFFSETCORR 25600 is exactly 100 counts). Also measured: the prescaler linear, 2 x (P + 1), against the device header's SAM D21 enumerators; CTRLC and ANACTRL not enable-protected though 39.6.2.1 lists them; REFRANGE a real field chapter 39 never mentions; the sixteen specified bits noise-free from OSR 128 up; the offset common-mode driven (-13.9 mV at GND, +17.0 at VDD, 0.6 mV at mid-supply) and the chopper taking a third off it; a PWM sweep linear to 4 counts of 32768 with +1.7 % gain error; a conversion OSR x 4 CLK_SDADC cycles exactly and SKPCNT a whole window each; erratum 1.8.10 reproduced with a control. Not covered: the converter as a wake source, the analog control's undocumented fields, a DC linearity source, an external reference off the rails, AnalogSampler |
| [tsens.md](tsens.md) | TSENS: the on-die temperature sensor, the one chapter that is not an ADC channel: it counts a temperature-dependent oscillator against GCLK_TSENS, so the measurement is a clock ratio and the generic clock its ruler; the factory calibration read out of nvm.hpp's calibration area, the signed 24-bit datum in centi-degrees, the window monitor, both event directions, the two escapes for an uncalibrated clock (`tsens_gain_for()`, `tsens_rescale()`). Measured: the same die on OSC48M (5003 ppm slow) and on a crystal-locked DPLL differs by a median of -35 centi-C against -37 predicted, a thermometer-free verification of the whole formula; GAIN's reset value 2^24 and not zero (a 699 ms conversion handing back a plausible -16000 C); a measurement 2 x GAIN + 2020 GCLK periods; the rail at -2^23 located to 250 counts in eight million; an overflowed VALUE wraps rather than saturating; OUTSIDE the complement of INSIDE; CAL.TCAL/FCAL worth 10.08 C; erratum 1.19.1 reproduced and worse than it says (a PAC-protected write to CTRLB starts nothing and raises no PAC flag). Not covered: a sampler adapter, the block as a wake source, the hysteresis modes' own hysteresis, absolute accuracy (no thermometer) |
| [ccl.md](ccl.md) | CCL: four three-input look-up tables, two sequential sub-modules, the synchronizer, filter and edge detector, no interrupt and no DMA at all; every INSEL source, the truth table from a predicate (the AVR's `lut_truth()` convention), both output stages, all four sequencers, both event directions, the pad map in the reserve (LUT3's pads the J's alone). The enable-protection story settled raw against three documents: a LUTCTRL write lands only with both CTRL.ENABLE and LUTCTRLn.ENABLE clear, a store into an enabled LUT is dropped in silence, 37.6.2.1's one-store escape is real. The headline, at a GCLK slowed to 11.719 kHz: a combinational LUT costs 0.05 periods, a LUT pair as a DFF the fraction to the next edge, the synchronizer the fraction + 1, the comparator's own synchronized output the fraction + 2, the filter's fraction + 3. Also measured: the device header missing INSEL 0x8 (TCC) on every variant, errata 1.8.3 and 1.7.1 revision B, 1.7.2 reproduced and its workaround coded, 1.7.4 confirmed, the edge strobe one GCLK period to 0.5 %, a software event crossing an asynchronous channel into a LUT (16/16). Not covered: standby, usage types, the E/G variants |
| [pac.md](pac.md) | PAC: write protection for every peripheral on every bridge, and the position is mechanism only: no guard type, no policy, no util contract (11.5.2.6's balance rule, and erratum 1.13.3 shows the guarantee is not uniform). Measured: protection off out of reset except the DSU; STATUSC bit 25 (PERID 89) outside every drawing and answering WRCTRL; sixteen peripherals across three bridges all flagging a protected write; erratum 1.13.2 confirmed with a control, 1.13.3 confirmed (the same DIRTGL write lands through the IOBUS unflagged); the IOBUS window not a plain mirror; a lock lasting until the next reset of any kind. Not covered: any guard or policy, the ERR interrupt as a wake source, the C21N's fourth bridge |
| [dsu.md](dsu.md) | DSU: the three services chapter 13 offers to the CPU: DID decoded into the fields the errata matrix is keyed by, a hardware CRC32 over anything the bus matrix reaches, and MBIST, the March LR test that destroys what it tests; chip erase deliberately absent; the one peripheral that comes out of reset PAC-protected. Measured: the die serial printed in the manifest's format; the CRC32 the standard reflected-0xEDB88320 answer, 6 cycles per word against 387 for a bitwise reference (55x), LENGTH a byte count and a working counter; MBIST 1165 cycles per word; the CoreSight ROM's two entries resolving to the M0+'s own table and the MTB; the header's two undocumented registers read. Not covered: chip erase, everything that is a probe's business, the DCC's DMA, MBIST's failure path (nothing here makes a bad bit) |
| [divas.md](divas.md) | DIVAS: the 32-bit divider and square-root engine the Cortex-M0+ has not got, on two buses (the AHB at 0x48000000 and the IOBUS alias at 0x60000200, named by the data sheet alone). Measured against gcc's own division: 0xFFFFFFFF/7 costs 332 cycles in software and 19.5 through the engine (17x), 683 against 22.6 with the remainder (30x); the IOBUS not the faster path; DLZ=0 8.3 cycles for a small dividend against 20.8 for a full-width one; a square root 20.8; 14.5.8's unnamed error is PAC.INTFLAGAHB.DIVAS. The decision the numbers exist for, making this the toolchain's division through `__aeabi_uidiv`, is named open and not taken; not covered: that adoption, 16-bit operation |
| [mtb.md](mtb.md) | MTB: the Cortex-M0+ Micro Trace Buffer, whose bit layout the device header alone states locally; a program can point the block at its own SRAM and read its own hardware backtrace with no debugger. Measured: 12 packets over a known chain with all three functions' linked addresses, bit 0 of the destination word marking the start of trace, a full buffer wrapping while FLOW.AUTOSTOP stops it, the device header's event-user numbers right and table 12-3's wrong (only user 45 starts a trace, 46 stops one), neither AUTOHALT nor HALTREQ stopping a core whose C_DEBUGEN is clear; the trace carried across a reset beside the panic breadcrumb is `samc21/postmortem.hpp`'s. Not covered: any decoder above the packet pair, the CoreSight management registers |
| [vendor/README.md](vendor/README.md) | The datasheets and errata the stratum is written against, the bench chip's silicon revision, the targeted errata pass (PDFs kept local, not in git) |
