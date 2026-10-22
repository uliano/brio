# Target: STM32G0 (`stm32g0/`)

The operational page of brio's STM32G0 target, for a programmer who has
a Nucleo board on the desk and wants to build, flash and debug a brio
program on it, and then find the document of the peripheral the program
needs. It says what the stratum is, which boards and parts it is proven
on, how the build, the upload, the debugger and the console work and
where each has a trap, and it ends with the map of the stratum's own
documents: one line per peripheral driver, which is where the state of
the driver work is read.

A stratum is a directory of `brio/` holding one architectural layer;
`brio/stm32g0/` holds everything that includes ST's device header
`stm32g0xx.h`. The STM32G0 is an ARM Cortex-M0+ family (the ARMv6-M
architecture), and what is the core's rather than ST's lives one layer
below, in the `cortexm/` stratum the SAM C21 shares
([../cortexm/README.md](../cortexm/README.md)). The kernel (`kernel/`)
and the services (`util/`) run here as written: time events pacing a
pin, and the full console over the board's own virtual COM port. The
bench chip is the STM32G0B1RE on an ST Nucleo-64, with the Nucleo-G071RB
and the Nucleo-G031K8 beside it; the rest of the family is covered by a
compile check over every device header ST ships ("Family coverage"
below).

If you are here from the learning route: the step-by-step build, flash
and debug of the maintained example `blink` is
[docs/learn/command-line.md](../learn/command-line.md) and
[docs/learn/vscode.md](../learn/vscode.md). This page is their
reference: the same tools, with every board, every caveat and the
whole family.

The documents of record (reference manual, datasheet, errata, by
revision), the errata pass item by item and the identity of each bench
die (the silicon revision, read from DBGMCU_IDCODE over SWD) are in
[vendor/README.md](vendor/README.md).

## The boards

Three Nucleo boards, one per part; the part selects the board in the
build (below), and each board has a page of its own under
[../boards/](../boards/README.md) with its connectors and its probe.

| Board | Part | Memory | LED | Console | Die |
|---|---|---|---|---|---|
| [Nucleo-G0B1RE](../boards/nucleo-g0b1re.md) (MB1360), the bench board | STM32G0B1RE, LQFP64 | 512 KB dual-bank flash, 144 KB SRAM | LD4 on PA5 | USART2 PA2 (TX) / PA3 (RX), AF1 | DEV_ID 0x467, REV_ID 0x1001: silicon revision Z |
| [Nucleo-G071RB](../boards/nucleo-g071rb.md) | STM32G071RB, LQFP64 | 128 KB single-bank flash, 36 KB SRAM | LD4 on PA5 | the same | DEV_ID 0x460, REV_ID 0x2000: ES0418's revision B |
| [Nucleo-G031K8](../boards/nucleo-g031k8.md) (a Nucleo-32) | STM32G031K8, LQFP32 | 64 KB single-bank flash, 8 KB SRAM | LD3 on PC6 | the same | DEV_ID 0x466, REV_ID 0x1003 |

Every bench suite prints that DEV_ID and REV_ID pair at boot, through
`DeviceIdcode::read()` ([platform.md](platform.md)), because a
measurement that differs between two boards is only a finding once the
die it was taken on is on the record.

The bench board runs at 3.3 V. The following facts of it were each
verified at the bench by an experiment of their own, not taken from the
user manual: LD4 on PA5 (driven over SWD), the ST-LINK virtual COM port
on USART2 PA2 (TX) / PA3 (RX) at AF1 (the console answers through it),
the CPU on HSI16 through the PLL at 64 MHz, and the LSE 32.768 kHz
crystal fitted and running (X2 is populated on this board: LSERDY rises
and the period measures 32703 Hz against the core, which is the core's
own 1 % trim and not the crystal's, [rtc.md](rtc.md)). Not yet verified:
the user button B1 on PC13, and the absence of an HSE crystal (X3 is not
fitted by default, and the ST-LINK's 8 MHz MCO reaches HSE only through
solder bridges; the HSE root is unbuilt anyway).

The Nucleo-G071RB has the same board layout as the G0B1RE's and is tied
to it by the six-wire bus link the two bus suites' header comments
describe. The Nucleo-G031K8 carries the same six bus wires to the
Nucleo-G0B1RE. What each of the two smaller parts has and has not got,
and what the suites measure on them, is in "The Nucleo-G071RB" and "The
Nucleo-G031K8" below.

## Toolchain

The compiler is a self-built arm-none-eabi-gcc 16.2 at
`/sw/arm-none-eabi`, the same compiler, flags and linker discipline as
the samc21 project: `stm32g0/cmake/toolchain-arm.cmake` is that
project's file verbatim (`CMAKE_SYSTEM_NAME Generic`, a `STATIC_LIBRARY`
try-compile, and a link that takes brio's runtime and libgcc and no C
library, so an accidental `printf` or `malloc` fails the link,
[../design/runtime.md](../design/runtime.md)). A toolchain installed
elsewhere is named at configure time with the cache variable
`ARM_TOOLCHAIN_DIR`, the root that contains `bin/`
([the command-line page](../learn/command-line.md) shows the form).

The device headers are vendored: `third_party/cmsis-device-g0/` holds
ST's cmsis-device-g0 v1.4.5 (every G0 part) and `third_party/cmsis-core/`
the CMSIS-Core headers it includes. The device is selected ST's way, by
a plain `-DSTM32G0B1xx` define that the umbrella header `stm32g0xx.h`
dispatches on; there is no device-specs machinery, so clangd needs no
macro feed of its own. What is ARMv6-M and not ST lives in the
`cortexm/` core stratum: `nvic.hpp`, `ticker.hpp` and `delay.hpp` here
are the device header plus that core file plus this family's own facts.

## Build

`stm32g0/` is its own CMake project, a sibling and peer of `avrxt/`,
`samc21/` and `test/`; the repository root is not a CMake project, so
every CMake command runs from `stm32g0/` (or in a subshell that enters
it, as the examples below do):

```bash
(cd stm32g0 && cmake --preset stm32g0b1re-release)                      # configure (once, or after adding an app)
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>)
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>-upload)
(cd stm32g0 && cmake --preset stm32g071rb-release)                      # the Nucleo-G071RB; stm32g031k8-release the Nucleo-32
brio check stm32g0 [name]                                          # family smoke, no hardware
```

One configure targets one part. The cache variable `STM32G0_MCU` holds
the full part number (`stm32g0b1re`), and from it the project derives
the device define (`STM32G0B1xx`), the linker script `ld/<part>.ld`, the
start-up code `src/glue/startup_<header>.cpp` and the board name, the
part's last six characters (`g0b1re`): one Nucleo carries one part on
this desk, so the part is the board. Six presets name the three parts
twice each: `stm32g0b1re-release` and `stm32g0b1re-debug`, and the
`stm32g071rb-*` and `stm32g031k8-*` pairs (CMake Tools shows them by
their display names, `STM32G0B1RE (release, -Os)` and
`STM32G0B1RE (debug, -Og)`). A release build is `-Os`, the one to flash
for a measurement; a debug build is `-Og -g3 -fno-inline`, the one a
debugger wants.

Applications are discovered at every configure from
`stm32g0/src/apps/*.cpp`, plus `experiments/*/stm32g0/*.cpp`, by the
`// build:` lines in each file's header comment, the same grammar as the
other projects with this family's board names:

- `// build: boards = g0b1re,g071rb,g031k8` lists the boards an app
  builds for; `g0b1re` is the default when the line is absent. A
  configure creates a target only for the apps that list its board.
- `// build: groups = abcdefghij,klmnopqvwy` splits a test suite into
  one image per group of letters on a board whose part cannot hold it
  whole (the G031K8 here): `<app>-1`, `<app>-2` and so on
  ([design/overview.md](../design/overview.md), "A suite's image fits
  the family's smallest chip").

Build outputs land in `build-cmake/stm32g0b1re-{release,debug}` at the
repository root: `<app>.elf`, `.bin` and `.hex`, `firmware-<app>.map`
and `<app>.lst`, the source-interleaved disassembly. A configure also
writes this project's app roster, `build-cmake/apps_stm32g0.json`, which
`bin/brio` reads.

`bin/brio` is the repository's one command for boards and builds; a
verb is the action word after `brio`. `brio flash <board> <app>`
configures, builds and flashes in one step, and the board type `g0b1re`
in the bench manifest is what tells it to build in this project and to
flash through OpenOCD's ST-LINK interface. `brio run` speaks the bench
suites' single-letter grammar (one key per command, no line terminator);
the line-oriented `console` app is driven with any serial monitor, or
pyserial, at 115200 8N1.

All three boards have their presets, linker scripts, start-up files and
`bin/brio` board types, and `blink`, `console` and `probe` build for all
three.

## Upload (OpenOCD, ST-LINK)

Flashing goes through OpenOCD driving the Nucleo's on-board
ST-LINK/V2.1 ([../probes/st-link.md](../probes/st-link.md)):
`interface/stlink.cfg` plus `target/stm32g0x.cfg` (the stm32l4x flash
driver underneath), `program <app>.elf verify`, then `reset run`, then a
write of DHCSR that clears C_DEBUGEN. That last write matters: a core
left with halting debug enabled halts on a BKPT instruction instead of
faulting, and every `panic()` ends in one, so without it a panic would
stop the board in silence. OpenOCD is the 0.12.0 release, built from its
tarball into `/sw/openocd-0.12.0` and reached as `/sw/openocd`
([the SAM page](../samc21/README.md) describes the build); another path
is named with the cache variable `STM32G0_OPENOCD`. It drives the
ST-LINK (firmware V2J46M31) without incident. The probe carries a real
USB serial number, so `adapter serial` names one probe among several
(`-DSTM32G0_PROBE_SERIAL=<serial>` at configure time), and the same
serial names the console under `/dev/serial/by-id`. The probe serves one
client at a time: close the debug session before flashing.

Two caveats of this probe and this family are worth knowing before
reading a register through the debugger.

**Memory reads through the ST-LINK while the core sleeps in WFI are
unreliable.** A running console (WFI between events) answered
`0xffffffb7` for FLASH_ACR and zeros for RCC_CR, values those registers
cannot hold, while the same reads after `halt` were exact. Halt first,
read, then resume; the CMSIS-DAP probe on the SAM C21 board does not do
this. (The transport is OpenOCD's HLA, the high-level adapter interface
the ST-LINK speaks.)

**A debugger leaves DBG_STOP set, and it reads as a silicon fact until
it is found.** `target/stm32g0x.cfg`'s examine-end hook writes
`DBGMCU_CR.DBG_STOP | DBG_STANDBY` ("enable debug during low power
modes"), and OpenOCD re-examines the target after every reset it issues,
so the bits are back on the reset a `program` ends with, whatever was
cleared before it (measured: cleared to 0 while halted, read 6 again
right after `reset run`). The register survives every reset but a
power-on. With DBG_STOP set the debug logic keeps HCLK, and SysTick with
it, running inside a Stop, which cuts a Stop entered with the kernel
tick armed from its full 250 ms to one tick. `bin/brio` therefore ends
every G0 flash with `reset halt`, a clear of DBGMCU_CR through its clock
gate (the gate put back to its reset value) and a `resume`, so a board
leaves the bench as a power-on would leave it. A cortex-debug session
sets the bits again, and `Pwr::debug_in_stop()` ([pwr.md](pwr.md)) is
how firmware tells. Both states are measured in `test_stm32_sleep`
letter c. For a program that never uses the Stop modes, `blink` among
them, the bit is of no consequence.

## Debugging (cortex-debug + OpenOCD)

`.vscode/launch.json` has one launch configuration per part: "Debug
STM32G0B1RE (OpenOCD, Nucleo-G0B1RE)" and its G071RB and G031K8
siblings, each the SAM entry's shape with the two ST config files above
and the part's own SVD from `stm32g0/svd/` for the Peripheral Viewer. No
probe is named, so the one ST-LINK attached is taken; with several,
`adapter serial` goes through `openOCDPreConfigLaunchCommands`. Three
choices in CMake Tools must agree before a session starts: the Active
Folder `stm32g0/`, the configure preset the part's `-debug` one, and the
launch target the app to debug. The sequence is verified on the
Nucleo-G071RB, from VS Code and from the command line with the same
OpenOCD and gdb: load, reset-halt, a breakpoint at `main` hit, source
lines stepped; the step-by-step procedure, with each step's
verification status, is [docs/learn/vscode.md](../learn/vscode.md). A
session leaves DBGMCU_CR.DBG_STOP set (the target script's doing, above)
until a power-on or a `brio flash`, which clears it; a Stop measured
right after a debug session does not stop the clocks.

## Editor (clangd)

`brio/stm32g0/.clangd` and `stm32g0/.clangd` route the stratum's headers
and the project's sources to the compile database of the release preset,
`build-cmake/stm32g0b1re-release`, which one configure of that preset
creates; `test/family_stm32g0/.clangd` lets the family check's TUs, which
no CMake project compiles, borrow flags from the same database. The
repository root's `.clangd` rules apply unchanged.

## Serial console

The ST-LINK's virtual COM port enumerates with the probe's own USB
serial number, so the console is addressed by `/dev/serial/by-id` and
never moves with the socket (the CH340-based boards of other targets
need `/dev/serial/by-path` instead). Console apps run at 115200 8N1.
Measured: BRR 556 at 64 MHz gives 115107 baud (the arithmetic to the
hertz), 300 lines of 50 bytes exchanged with zero errors of any kind, and
the kernel tick +0.24 % against the PC's clock (inside HSI16's 1 %
calibration).

## Family coverage

The stratum is written for the whole STM32G0 family, both lines:

- the x1 line: G031/G041, G051/G061, G071/G081, G0B1/G0C1, which is
  what the bench chip belongs to (the G0B1 is its superset), and
- the x0 value line: G030, G050, G070, G0B0, each of whose headers is a
  strict subset of its x1 twin: the same IP under the same register
  names, minus a list of peripherals (no LPUART, no LPTIM, no DAC, no
  comparator, no VREFBUF, no TIM2, no PVD/PVM, no programmable BOR, no
  PCROP or securable memory, no CEC/CRS/UCPD/FDCAN).

What differs between parts is read off the device header and never off
a device name. The header is the authority, and
`brio/stm32g0/device_tables.hpp` is the table of what each part has: it
probes the header's own base-address and bit-mask macros and exports
what it finds as constexpr data. The stratum's documents call that
header "the reserve", a name worth knowing because every peripheral
document uses it: the reserve is where a per-part fact lives when no
register of the peripheral states it.

In particular, the interrupt vectors are derived from presence. An
interrupt line's enumerator name in the header lists what shares it
(`USART2_LPUART2_IRQn`, `TIM6_DAC_LPTIM1_IRQn`, `ADC1_COMP_IRQn`,
`DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn`, and so on), a header declares
the shared spelling exactly when it declares the sharer, and IRQn values
are enumerators the preprocessor cannot probe. So each vector verb asks
for the sharer's base macro and names the enumerator that follows. A
header whose naming did not follow the rule would fail to compile, never
bind a wrong line in silence.

An absence is spelled one of two ways, and each peripheral document says
which applies to it:

- a peripheral the part has not got is a driver that does not exist
  there: `Dac`, `Comp<n>`, `Vref`, `Lptim<n>` and its sleep site,
  `Lpuart<n>` and `Fdcan<n>` compile their register-facing half only
  where the header declares the block, while their vocabulary (enums,
  config structs, validity checks and arithmetic) is every part's;
  spelling the driver on such a part is a compile error naming the
  reason, and `Tim<2>` on a value-line part is the "no such timer"
  refusal;
- a register or bit a present block has not got is a verb that refuses
  at run time and a constexpr flag that says so at compile time:
  `Pwr::has_pvd` / `has_sram_retention` / `has_sampled_supply_monitor` /
  `has_dac_supply_monitor`, `Flash::has_debug_gate`,
  `FlashOptions::has_programmable_bor` / `has_shutdown_reset_option` /
  `has_nrst_mode`, and the reserve's `flash_pcrop_capable` and
  `flash_securable_capable`.

`brio check stm32g0` proves it on every header the CMSIS pack ships,
without hardware: every positive translation unit in
`test/family_stm32g0/` compiles for all twelve device headers (the x1
line and the x0 line), and every `neg/` TU, a program that must be
refused, fails to compile for each variant its `// mcu:` line names.
The bench proves two of the twelve beyond the G0B1 (`brio check stm32g0
[name]` filters by name; the check ends with the link guard of every app
on every release preset). For the nine headers no board here carries,
the sweep is the only check there is.

## The Nucleo-G071RB

The STM32G071RB is the bench board's smaller sibling, an LQFP64 with the
same board layout. Fourteen of the seventeen test suites build for it
(the three that do not, `test_stm32_nvm`, `test_stm32_journal` and
`test_stm32_fdcan`, need a second flash bank or an FDCAN the part has
not got), and the same fourteen build for the Nucleo-G031K8 below. An
app that builds for more than one board says so in its
`// build: boards = g0b1re,g071rb,g031k8` line. What a suite cannot do
on a board it skips by name, printing the reserve's own fact and
claiming no verdict, so a smaller count is a shorter list of claims and
never a weaker one.

In the tables below a suite's figure is its count of verdicts (the
"pass" lines its console reports); a letter is one command key of a
suite's console, the group of verdicts it runs, and `z` is the all-key
that runs every letter in turn; a figure in parentheses is a letter
outside `z`.

| Suite | G0B1RE | G071RB | What skips there, and why |
|---|---|---|---|
| `test_stm32_crc` | 33 | **27** | nothing - the one driver that needs no fact from the reserve |
| `test_stm32_platform` | 71 (+ `i` 26, `j` 20) | **53** (+ `i` 26) | nothing; letters `w` and `k` (the idle contract) are measured on the G0B1RE alone ([platform.md](platform.md)) |
| `test_stm32_sleep` | 50 | **50** | nothing in `z`; letter `u` measures a different Shutdown wake ([pwr.md](pwr.md)) |
| `test_stm32_exti` | 95 | **89** | nothing; IMR1's reset value differs ([exti.md](exti.md)) |
| `test_stm32_rtc` | 126 | **125** | nothing in `z`; two tamper inputs instead of three, and arming one does not take its pad ([rtc.md](rtc.md)) |
| `test_stm32_lptim` | 85 | **82** | nothing |
| `test_stm32_tickless` | 47 | **45** | letter `i`'s awake-time meter needs TIM2's ETR taking MCO, which this part's ETRSEL has not got |
| `test_stm32_clock` | 42 | **40** | letter `h` (`delay_us` at 20 us) needs that same 4 us wall; the rest runs on an LSE wall instead |
| `test_stm32_tim` | 128 | **114** | TIM4: its counter, its four channels on PB6..PB9, and TIM3's shared vector |
| `test_stm32_dma` | 85 (+ `p` 3, `q` 4, `u` 3) | **76** (+ `p` 3, `q` 4, `u` 3) | DMA2's five channels, and letter `m`'s peripheral-to-peripheral destination ([dma.md](dma.md)) |
| `test_stm32_analog` | 139 | **136** | COMP3's register block and its whole signal path |
| `test_stm32_serial` | 120 (+ y/w/v/x; + `t` 174, the STM32F446RE as peer) | **112** (+ y/w/v) | LPUART2 (two verdicts), and four to ES0418 2.2.4 ([usart.md](usart.md)) |
| `test_stm32_spi` | 86 (the STM32F446RE as peer) | - | the same self-link letters that skip on the G0B1RE; SPI3 and SPI2's I2S are absent; the peer letters want a board paired with this one, and none is ([spi.md](spi.md)) |
| `test_stm32_i2c` | 155 (+ `w` 35, the STM32F446RE as peer) | **54** (peer) | the same self-link letters, and the peer set's `s` and `t`, whose third node is the self-link's I2C2; I2C3 is absent and I2C2 has no independent clock |
| `test_stm32_nvm` | 85 | - | one flash bank: no storage attic ([nvm.md](nvm.md)) |
| `test_stm32_journal` | 52 | - | the same |
| `test_stm32_fdcan` | 96 | - | no FDCAN on this part ([fdcan.md](fdcan.md)) |

The I2C suite's figures on this part were measured with the other board
as its peer, roles exchanged, in one set (the peer letters have since
moved to their own all-key, `w`); the SPI and I2C suites' G0B1RE figures
are against the STM32F446RE it is paired with on the desk.

What the part has not got, all of it read off the header by the
reserve: TIM4, USART5, USART6, LPUART2, I2C3, SPI3 (and with it SPI2's
I2S), DMA2 (DMA1 keeps seven channels and the DMAMUX seven), COMP3,
FDCAN, USB, CRS, GPIOE, the second flash bank, the EXTI's second-group
trigger registers, WKUP3, TAMP_IN3, PWR's PUCRE/PDCRE, and
`RCC_CCIPR_I2C2SEL`, which is what takes I2C2's independent clock, its
SMBus and its wake with it. Its USART3 is a basic instance where the
G0B1's is a full one (the two classes of USART, [usart.md](usart.md)), so
it has one wake line fewer; its MCOSEL and MCOPRE are three bits rather
than four. The ETRSEL list is a per-part fact of its own in the reserve,
`tim_etrsel_has_mco()`, because RM0444 22.4.25 gives codes 0100, 0101
and 0110 to the G0B1/G0C1 sales types alone and no TIM register says so
([tim.md](tim.md)).

## The Nucleo-G031K8

The STM32G031K8 is the smallest part on the desk, and its 64 KB of flash
and 8 KB of SRAM are part of the design: every suite's image on that
board fits, with the cuts named in its own suite. Its LSE crystal runs
(LSERDY in about 900 ms at the lowest drive, 32719..32753 Hz against the
core) with the oscillator bridges at UM2591's default, so the RTC, the
tickless timebase, the timed sleep sites and every wall run on the
crystal exactly as on the Nucleo-64s and the six suites those touch are
configured identically for all three boards; its LSI weighs
31403..31496 Hz on a TIM16 capture and 31400 by the watchdog, the
slowest of the three dies. Two board facts shape suites on it: PA0 and
PA4 lean high when left floating (the Nucleo-64s' free pads drift down),
which costs the tamper letter its TAMPPUDIS contrast; and its debug port
can go silent, a state of the board's ST-LINK half that only unplugging
the board clears, in which case the ST-LINK's own mass-storage flasher
is the way in (`bin/brio`'s `stlink_msd` programmer kind) and nothing
can be halted or read over SWD until the replug
([../probes/st-link.md](../probes/st-link.md)).

Fourteen of the seventeen suites build for it, the same fourteen as for
the G071RB:

| Suite | G0B1RE | G071RB | G031K8 | What skips on the G031K8, and why |
|---|---|---|---|---|
| `test_stm32_crc` | 33 | 27 | **27** | nothing - the boards line is the whole change |
| `test_stm32_platform` | 71 (+ `i` 26, `j` 20) | 53 (+ `i` 26) | **53** (+ `i` 26) | nothing; the LED moves to PC6 and LSI reads 31400 Hz; letters `w` and `k` are measured on the G0B1RE alone |
| `test_stm32_sleep` | 50 (+ `s` 6, `u` 6) | 50 (+ `s` 6, `u` 6) | **50** (+ `s` 6, `u` 6) | nothing; a Shutdown wake is a literal power-on reset here as on the G071 ([pwr.md](pwr.md)) |
| `test_stm32_exti` | 95 | 89 | **85** | IMR2's reset value (no second register group at all) and the user button (PC13 is not bonded) |
| `test_stm32_rtc` | 126 (+ `w` 11, `v` 4) | 125 | **109** (+ `w` 11, `v` 4) | RTC_REFIN (PB15 unbonded), two tamper legs whose level is a board's, and the TAMPPUDIS contrast (a free pad that leans high) ([rtc.md](rtc.md)) |
| `test_stm32_lptim` | 85 | 82 | **78** | the two comparator routes, the comparator trigger row and the two-owner vector (no COMP, no TIM7); the LSE rows are measured |
| `test_stm32_tickless` | 47 | 45 | **45** (+ `u` 1) | the awake-time meter, which needs TIM2's ETR taking MCO |
| `test_stm32_clock` | 42 | 40 | **40** | letter `h`: the 4 us wall needs TIM2's ETR taking MCO, a G0B1 code; the LSE code is the wall here, as on the G071 ([clock.md](clock.md)) |
| `test_stm32_tim` | 128 | 114 | **108** | TIM4, TIM6, TIM7 and TIM15 - four instances the part has not got - and PB13..PB15, which the package does not bond |
| `test_stm32_dma` | 85 | 76 | **62** (+ `u` 3, `w` 0) | DMA2, the console's DMA-fed rate (five channels, and the letters own all five) and the peripheral-to-peripheral leg; letters `o` (6) and `r` (4) are built and not run here, nor letter `h`'s rate verdict on the interrupt console, and `p` and `q` skip by name (no console engines) |
| `test_stm32_analog` | 139 | 136 | **67** | the DAC's letters and the comparators' - this part has neither block, so eight of the eighteen letters are compiled out ([adc.md](adc.md)) |
| `test_stm32_serial` | 118 (+ y/w/v) | 112 | **79** (+ `y` 1, `v` 2, `w` 0), counted on the one image it was before it outgrew the part and before its letters timed the wire | USART2 is a basic instance here (no kernel-clock multiplexer, no wake), LPUART1's only bonded pads are the console's, and there is no LPUART2 |
| `test_stm32_spi` | 86 (the STM32F446RE as peer) | - | - | the eleven self-link letters, compiled out: SPI2's four pads are bonded to no pin of the LQFP32, so that instrument cannot exist here at any wiring; the peer letters want a paired board, and none is |
| `test_stm32_i2c` | 155 (+ `w` 35) | 54 (peer) | **54** (peer) | the eleven self-link letters, skipped on the probe's answer like everywhere else - this package does bond I2C2's PA11/PA12, so what rules them out is the desk and not the plastic |
| `test_stm32_nvm`, `test_stm32_journal` | 85, 52 | - | - | one flash bank: no storage attic |
| `test_stm32_fdcan` | 96 | - | - | no FDCAN |

How the images fit: `brio check stm32g0` links each, `brio fit stm32g0`
lists what each takes, and one suite does not fit whole.
`test_stm32_serial` is built as five group images on the G031K8 (its
groups line): 47.4, 40.4, 20.0, 45.1 and 39.2 KB, the tightest with 26 %
left; on the G071RB as two (93.6 and 69.5 KB of its 128). The hungriest
in RAM is `test_stm32_dma`, at about 6.4 KB of the 8, which leaves the
stack some 1.7 KB (the DMA suite's deepest letter's frame is 120 bytes,
by `-fstack-usage`). One suite makes a real cut rather than losing
letters: `test_stm32_dma` gives its three memory-to-memory buffers 256
words there instead of 512 (every claim restated for the length that
fits) and gives the console's two DMA channels back to the letters.
`test_stm32_spi` compiles its eleven self-link letters out (a package
fact, not a budget one) and, the peer half grown past 95 % of the part,
builds as two group images (its groups line: 42.4 and 37.2 KB);
`test_stm32_i2c` keeps every letter it has anywhere.

What the part has not got, all of it read off the header by the
reserve: TIM4, TIM6, TIM7, TIM15, USART3..6, LPUART2, I2C3, SPI3, DMA2
(DMA1 keeps five channels and the DMAMUX five with four generators), the
DAC and every comparator, FDCAN, USB, CRS, UCPD (so PA8 and PB15 carry no
dead-battery Rd), GPIOE, the second flash bank, the EXTI's entire second
register group, WKUP3 and WKUP5, `RCC_CCIPR_I2C2SEL` and
`RCC_CCIPR_USART2SEL`, the last of which is what takes USART2's kernel
clock, its FIFO and its wake with it. What the package does not bond is
a separate list the suites own, never the reserve: PB10..PB15, PC0..PC5,
PC7, PC13, PD0..PD3, PF0 and PF1 (GPIOD is declared by the header and
reaches no pin at all). One per-part fact in the reserve covers that
difference: `ucpd_present(n)`, probed on SYSCFG's own dead-battery strobe
bit, so a pad that will not follow its pull can be told from one that
never had an Rd on it.

**Vector names across the boards.** The start-up code spells ST's own
handler names, and a shared line's name changes with what shares it:
USART2's is `USART2_LPUART2_IRQHandler` on the G0B1 and
`USART2_IRQHandler` on the G071/G031. An app for one board binds the
bare name; an app for more than one binds the name the reserve derives
from the header's presence macros, `BRIO_STM32G0_USART2_HANDLER` and its
siblings (TIM3, TIM6, TIM7, TIM16, TIM17, LPTIM1, LPTIM2, USART3,
LPUART1, the DMA's upper line, the ADC); `test/family_stm32g0/handlers.cpp`
proves every one expands on every header and names the line the
reserve's verb answers. A name outside that set on the wrong board lands
in `Default_Handler`'s silent spin.

## The documents

One document per peripheral driver, in the shape
[../README.md](../README.md) prescribes: the reader's orientation
(purpose, the smallest use, the prerequisite that decides correctness),
the documents of record, what the silicon does, the types and verbs, how
to use it, the bench findings and, where the driver has gaps, "Not
covered yet" as two lists. The line beside each names what the document
is about and the finding or two most worth knowing before using the
driver; the findings themselves, with their figures and conditions, are
in the document.

| Document | Content |
|----------|---------|
| [platform.md](platform.md) | What the kernel stands on: `Stm32g0Platform<TB>`, the PRIMASK critical section, WFI idle, the vector table with its shared lines, the start-up code, and its two timebases - the SysTick `Ticker` (1000 Hz, stops in Stop) and `LptimTicker` (the LPTIM on the LSE crystal, 1024 Hz), which makes a program tickless: no periodic interrupt, every deadline placed in the compare, kernel time running through a Stop. Measured to the cycle: the idle contract (no edge position of 400 loses its wake, one kernel turn per tick, a wake from Sleep 36 cycles to the handler) and the LPTIM compare's latency, the finding that put the counter undivided. "Code in SRAM": the opt-in `InRam` placement of the SPI host's and the UART task's hot paths, with its gain and its cost per path |
| [reset.md](reset.md) | The reset flags of RCC_CSR as the accumulating history they are, the independent and the window watchdog with their traps (keyed registers that do not update until the start key clocks the block; a window counter that free-runs), the deliberate resets, and the panic breadcrumb proven across six real resets including a HardFault |
| [clock.md](clock.md) | One SYSCLK, shared bus prescalers, per-peripheral enable bits, the kernel-clock multiplexers and the MCO clock output; HSI16 and the PLL to 64 MHz, the flash latency sequence, the three voltage regimes as static rates; the dynamic clock over a pack of rates, every rung measured and a switch 48..384 us; `LseTracker`, the core tracked against the LSE crystal with no CPU |
| [port.md](port.md) | GPIO: a port has a clock and it is off at reset, analog is the reset state, the AF numbers are the datasheet's, BSRR/BRR are atomic, and four pads (PA8/PB15, PD0/PD2) are not ordinary GPIOs out of a power-on until SYSCFG's strobe releases the UCPD dead-battery pull-downs |
| [exti.md](exti.md) | Where this family keeps its pin interrupts: sixteen lines whose number is the pin number and whose port is the one choice (PA3 and PB3 cannot both be armed), edges only, rising and falling in separate pending registers, three shared vectors; a pending bit is set only for an unmasked interrupt, so a line cannot be polled without arming it |
| [nvm.md](nvm.md) | The flash program/erase engine (page 2048, row 256, double word 8), the option bytes as a read-only decode, the ECC status; the storage design that gives bank 2 to the NV heap and journal while the image stays in bank 1; a malformed program found to wedge CFGBSY until the next reset |
| [tim.md](tim.md) | Ten timers behind one register struct, so what a timer is comes from the documents' table and every verb naming a feature an instance lacks refuses; the `rc_w0` status register (the one register of this stratum where the write-one-to-clear reflex is wrong); the input multiplexer that makes a capture measurable with no pad; PWM, complementary pairs with dead time, the util contracts `PwmChannel` and `MeterSampler`, the DMA requests and the burst engine |
| [dma.md](dma.md) | DMA and DMAMUX: two controllers behind one request multiplexer, three vectors for twelve channels, the request vocabulary each peripheral publishes; a request is a level served on enable, not an edge; the engines' two moments (`arm()` once, five stores per block) and the circular receive ring; the priority rule measured; what a Stop does to a running channel |
| [usart.md](usart.md) | RM0444 chapter 33 whole, almost all of it measured on one board with no wires through the single-wire loop-back; full and basic instances compared on three authorities; the FIFO as the transport's default (a 4096-byte print in 515 interrupts where the one-register transport took 4098); every frame format, both oversamplings, the prescaler, auto-baud, LIN, smartcard, IrDA, RS-485, the wake from Stop; the receive FIFO's wedge behind a starved DMA channel and the kick that cures it; every rate timed against the wire; the errata reproduced with their controls; the far-board measurements |
| [lpuart.md](lpuart.md) | Chapter 34, which is chapter 33 minus several modes plus a baud generator of its own; `LpUart` is the USART's own task over the other resource; a whole console moved onto LPUART1 at 9600 on the 32768 Hz crystal, and an LPUART on the crystal bringing the part out of Stop 1 on a start bit |
| [irtim.md](irtim.md) | The infrared interface: one pad and an AND gate, TIM17 as the carrier and a timer or a USART as the envelope, neither needing a pad; the USART envelope is active low where TIM16's is active high |
| [adc.md](adc.md) | The one 12-bit converter, a monostate: the regulator start-up and the self-calibration `init()` spends, the three factory measurements it reads; the sequencer's two faces and its CCRDY handshake; VDDA and the junction temperature with no meter; conversion time exact to the cycle; the oversampler against a measured noise floor; the watchdogs as disabled-state verbs because a forbidden write is not one thing on this converter |
| [dac.md](dac.md) | The two 12-bit channels, absent on the G031/G041; the DAC reaches the ADC through the pad and nothing else, so PA4 is a zero-length wire and the shape of every wireless analog experiment; DHR is not DOR, a trigger is an edge, the buffer cannot reach the rails, the DMA underrun caught |
| [vref.md](vref.md) | The voltage reference buffer, implemented whole and never enabled: driving VREF+ on a board that ties it to VDDA is forbidden and nothing inside the chip can tell, so `enable()` refuses unless the caller states the pin is free; the block sits behind SYSCFG's clock gate and reads wrong through it |
| [comp.md](comp.md) | The comparators (three on the G0B1 class, two on the G071, none on the G031), each living inside SYSCFG's block; the inverting input reaches internal references with no pad while the non-inverting one is a pad and nothing else; the precharged pad as the wireless stimulus; the blanking window and the window mode measured |
| [rtc.md](rtc.md) | The clock that outlives everything else: its own power domain behind two independent locks with different reach, RTCSEL one-way until a domain reset; the Nucleo's crystal fitted and running; the calendar, alarms, wake-up timer, smooth calibration, RTC_REFIN, the shift register and tamper detection measured, with the findings no chapter carries |
| [pwr.md](pwr.md) | The stopping half of the platform: chapter 4 and `util/power.hpp`'s depth ladder realized here unchanged; the sleep mode as two registers in two places; Standby and Shutdown off the ladder on purpose; a Stop's length on a bare board and under a debugger, the timed sleep site's resync from the RTC, the tickless platform's plain site through a Stop 1 |
| [lptim.md](lptim.md) | The two low-power timers, which are not copies of each other; the opposite enable rules of the two register halves; both errata answered by construction (no verb ever clears ENABLE; flags cleared only where the erratum allows); the waveform arithmetic the chapter never prints; through a Stop only LSE and LSI keep counting |
| [crc.md](crc.md) | The CRC unit, chapter 14 whole: words, half-words and bytes through one register, the programmable polynomial with the two rules the register does not enforce, big-endian feeding; 2056 cycles per 1000 bytes against the bitwise loop's 109216, and the DMA feeding it with no CPU |
| [i2c.md](i2c.md) | Chapter 32 whole, both roles, over the Nucleo's own self-link; the `I2cHost` request the other targets share verbatim; the timing register's arithmetic both ways; a repeated START as one CR2 store, RXNE before STOPF, a held SDA as a park and not an error; the errata's floors against a 2 MHz core; the far-board tenures and the wake from Stop by the other chip's address |
| [spi.md](spi.md) | Chapter 35 whole, both roles, over the self-link; the silicon enforces none of the configuration rule, so the driver's refusals are the only protection; the disable procedure as the erratum's workaround; every baud code to PCLK/2 with the host pacing and PCLK/4 with both ends on DMA; hardware NSS against a GPIO chip select; the CRC and its window; I2S; the client's polled runs; the extended exchange against the F446 |
| [fdcan.md](fdcan.md) | The Bosch M_CAN, the first chapter of this stratum a part can be missing entirely; one enable, reset and divider for both modules; measured with no transceiver, no wire and no second node through the chapter's own loop-backs; the 0xCC padding paragraph that does not describe this part; bus-off recovery timed |
| [vendor/README.md](vendor/README.md) | RM0444, DS13560, ES0548 (with ES0418 and ES0487 for the STM32G071RB and the STM32G031K8) by revision, the vendored cmsis-device-g0 headers and SVD, the bench chips' IDCODEs, the errata pass |
