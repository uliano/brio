# Target: STM32G0 (`stm32g0/`)

The operational page for brio's STM32G0 target: an ARM Cortex-M0+
(STM32G0B1RE on the bench, on an ST Nucleo-64), an ARMv6-M family that
shares the `armv6m/` core stratum with the SAM C21. `kernel/` and
`util/` run here as written: time events pacing a pin, and the full
console over the board's own virtual COM port.

Peripheral documents live next to this page, one per chapter; the
documents of record are in [vendor/README.md](vendor/README.md)
together with the errata pass and the bench chip's identity (silicon
revision Z, DBGMCU_IDCODE read over SWD).

## Toolchain

Self-built **arm-none-eabi-gcc 16.2** at `/sw/arm-none-eabi` - the
same compiler, flags and linker discipline as the samc21 project
(`stm32g0/cmake/toolchain-arm.cmake` is that file verbatim:
`CMAKE_SYSTEM_NAME Generic`, `STATIC_LIBRARY` try-compile,
`--specs=nano.specs -nostartfiles`, deliberately NO syscall stubs so
an accidental `_sbrk`/`_write` fails the link). What is ARMv6-M and not
ST lives in the `armv6m/` core stratum, so `nvic.hpp`, `ticker.hpp` and
`delay.hpp` here are the device header plus that core file plus this
family's own facts.

The device headers are vendored: `third_party/cmsis-device-g0/`
(ST's cmsis-device-g0 v1.4.5, every G0 part) and the shared
`third_party/cmsis-core/`. The device is selected ST's way, by a plain
`-DSTM32G0B1xx` define that the umbrella `stm32g0xx.h` dispatches on -
no device-specs machinery, so clangd needs no macro-delta feed.

## Board and build

The bench board is an **ST Nucleo-G0B1RE** (MB1360): STM32G0B1RE
(LQFP64, 512 KB dual-bank flash, 144 KB SRAM), silicon revision Z,
running at **3.3 V**. Verified at the bench, each by its own experiment
and not by the user manual: LD4 on **PA5** (driven over SWD), the
ST-LINK virtual COM port on **USART2 PA2 (TX) / PA3 (RX), AF1** (the
console answers through it), the CPU on **HSI16 through the PLL at
64 MHz**, and the **LSE 32.768 kHz crystal fitted and running** (X2 is
populated on this board: LSERDY rises and the period measures 32703 Hz
against the core, which is the core's own 1 % trim and not the
crystal's - [rtc.md](rtc.md)). NOT yet verified: the user button B1 on
PC13, and the absence of an HSE crystal (X3 is not fitted by default
and the ST-LINK's 8 MHz MCO reaches HSE only through solder bridges -
the HSE root is unbuilt anyway).

`stm32g0/` is its own CMake project, a sibling and peer of `avrdx/`,
`samc21/` and `test/`. Apps are auto-discovered from
`stm32g0/src/apps/*.cpp` - plus `experiments/*/stm32g0/*.cpp` - by
their `// build:` header comment, the other two projects' grammar with
this family's board names (`boards = g0b1re`, the default; an app that
also runs on the other two Nucleos lists `g071rb` and `g031k8`). One
configure targets one part (`STM32G0_MCU`, full part number: it decides
the device define `STM32G0B1xx`, the linker script `ld/<part>.ld`, the
crt `src/glue/startup_<header>.cpp` and the board name, the part's last
six characters - one Nucleo per part on this desk, so the part IS the
board); three parts have presets - the G0B1RE, and the two other Nucleos
below. Every other part of the family is compile-checked by
`brio check stm32g0`, which sweeps every positive TU in
`test/family_stm32g0/` across ALL TWELVE device headers the CMSIS pack
ships and requires every `neg/` TU to fail for the variants its
`// mcu:` line names - see "Family coverage" below.

## Family coverage

The stratum is written for the whole STM32G0 family, both lines:

- the **x1 line** - G031/G041, G051/G061, G071/G081, G0B1/G0C1 - which
  is what the bench chip belongs to (the G0B1 is its superset), and
- the **x0 value line** - G030, G050, G070, G0B0 - each of whose
  headers is a strict subset of its x1 twin: the same IP under the same
  register names, minus a list of peripherals (no LPUART, no LPTIM, no
  DAC, no comparator, no VREFBUF, no TIM2, no PVD/PVM, no programmable
  BOR, no PCROP or securable memory, no CEC/CRS/UCPD/FDCAN).

What differs is read off the DEVICE HEADER and never off a device
name: `brio/stm32g0/device_tables.hpp` (the reserve) probes the
header's own base-address and bit-mask macros and exports what it
finds as constexpr data. In particular THE VECTORS ARE DERIVED FROM
PRESENCE: an interrupt line's enumerator NAME lists what shares it
(`USART2_LPUART2_IRQn`, `TIM6_DAC_LPTIM1_IRQn`, `ADC1_COMP_IRQn`,
`DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn`...), a header declares the
shared spelling exactly when it declares the sharer, and IRQn values
are enumerators the preprocessor cannot probe - so each vector verb
asks for the sharer's base macro and names the enumerator that
follows. A header whose naming did not follow the rule would fail to
compile, never bind a wrong line in silence.

An absence is spelled one of two ways, and the docs say which:

- a peripheral the part has not got is a driver that DOES NOT EXIST
  there - `Dac`, `Comp<n>`, `Vref`, `Lptim<n>` and its sleep site,
  `Lpuart<n>`, `Fdcan<n>` compile their register-facing half only where
  the header declares the block, while their vocabulary (enums, config
  structs, validity and arithmetic) is every part's; spelling the
  driver on such a part is a compile error naming the reason, and
  `Tim<2>` on a value-line part is the "no such timer" refusal;
- a REGISTER or BIT a present block has not got is a verb that REFUSES
  at run time and a constexpr flag that says so at compile time:
  `Pwr::has_pvd` / `has_sram_retention` / `has_sampled_supply_monitor` /
  `has_dac_supply_monitor`, `Flash::has_debug_gate`,
  `FlashOptions::has_programmable_bor` / `has_shutdown_reset_option` /
  `has_nrst_mode`; the reserve's `flash_pcrop_capable` and
  `flash_securable_capable`.

The sweep proves it on every header the pack ships (22 positive TUs x
12 headers, and every negative refused on each variant it names); the
bench proves two of them.

## The second silicon: the Nucleo-G071RB

The **Nucleo-G071RB** (STM32G071RB, LQFP64, 128 KB single-bank flash,
36 KB SRAM, the same board layout as the G0B1RE's - LD4 on PA5, the VCP
on USART2 PA2/PA3) sits at manifest position `F`, tied to the G0B1RE by
the six-wire bus link of [../bench.md](../bench.md). Its die reports
**DEV_ID 0x460, REV_ID 0x2000** - ES0418 silicon revision B - and every
bench suite prints that pair at boot through `DeviceIdcode::read()`
([platform.md](platform.md)), because a measurement that differs between
two boards is only a finding once the die it was taken on is on the
record.

**FOURTEEN OF THE SEVENTEEN SUITES RUN ON IT**, and the same fourteen
on the third silicon below. An app that builds for more than one board says so
in its `// build: boards = g0b1re,g071rb,g031k8` line; what a suite
cannot do on a board it SKIPS BY NAME, printing the reserve's own fact
and claiming no verdict, so a smaller count is a shorter list of claims
and never a weaker one.

| Suite | G0B1RE (E) | G071RB (F) | What skips there, and why |
|---|---|---|---|
| `test_stm32_crc` | 27 | **27** | nothing - the one driver that needs no fact from the reserve |
| `test_stm32_platform` | 53 (+ `i` 26) | **53** (+ `i` 26) | nothing |
| `test_stm32_sleep` | 50 | **50** | nothing in `z`; letter `u` measures a DIFFERENT Shutdown wake ([pwr.md](pwr.md)) |
| `test_stm32_exti` | 89 | **89** | nothing; IMR1's reset value differs ([exti.md](exti.md)) |
| `test_stm32_rtc` | 125 | **125** | nothing in `z`; two tamper inputs instead of three, and arming one does not take its pad ([rtc.md](rtc.md)) |
| `test_stm32_lptim` | 82 | **82** | nothing |
| `test_stm32_tickless` | 47 | **45** | letter `i`'s awake-time meter needs TIM2's ETR taking MCO, which this part's ETRSEL has not got |
| `test_stm32_clock` | 42 | **40** | letter `h` (`delay_us` at 20 us) needs that same 4 us wall; the rest runs on an LSE wall instead |
| `test_stm32_tim` | 118 | **114** | TIM4: its counter, its four channels on PB6..PB9, and TIM3's shared vector |
| `test_stm32_dma` | 69 | **65** | DMA2's five channels, and letter `m`'s peripheral-to-peripheral destination |
| `test_stm32_analog` | 139 | **136** | COMP3's register block and its whole signal path |
| `test_stm32_serial` | 88 | **83** | LPUART2, and four verdicts to ES0418 2.2.4 ([usart.md](usart.md)) |
| `test_stm32_spi` | 38 (peer) | **38** (peer) | the same self-link letters that skip on E; SPI3 and SPI2's I2S are absent |
| `test_stm32_i2c` | 54 (peer) | **54** (peer) | the same self-link letters; I2C3 is absent and I2C2 has no independent clock |
| `test_stm32_nvm` | 85 | - | one flash bank: no storage attic ([nvm.md](nvm.md)) |
| `test_stm32_journal` | 52 | - | the same |
| `test_stm32_fdcan` | 96 | - | no FDCAN on this part ([fdcan.md](fdcan.md)) |

The two bus suites run with the OTHER board as their peer, roles
exchanged, on the same six wires.

WHAT THE PART HAS NOT GOT, all of it read off the header by the reserve:
TIM4, USART5, USART6, LPUART2, I2C3, SPI3 (and with it SPI2's I2S), DMA2
(DMA1 keeps seven channels and the DMAMUX seven), COMP3, FDCAN, USB, CRS,
GPIOE, the second flash bank, the EXTI's second-group TRIGGER registers,
WKUP3, TAMP_IN3, PWR's PUCRE/PDCRE, and `RCC_CCIPR_I2C2SEL` - which is
what takes I2C2's independent clock, its SMBus and its wake with it. Its
USART3 is BASIC where the G0B1's is FULL, so it has one wake line fewer;
its MCOSEL and MCOPRE are three bits rather than four. The ETRSEL list
is a per-part fact of its own in the reserve, `tim_etrsel_has_mco()`,
because RM0444 22.4.25 gives codes 0100, 0101 and 0110 to the G0B1/G0C1
sales types alone and no TIM register says so ([tim.md](tim.md)).

All three boards have their presets (`stm32g0b1re-*`, `stm32g071rb-*`,
`stm32g031k8-*`), linker scripts, crts and `bin/brio` board types,
and `blink`, `console` and `probe` build for all three.

## The third silicon: the Nucleo-G031K8

The **Nucleo-G031K8** (STM32G031K8, a Nucleo-32: **LQFP32, 64 KB
single-bank flash, 8 KB SRAM**, LD3 on PC6, the VCP on USART2 PA2/PA3)
sits at manifest position `G`, carrying the six bus wires to the
Nucleo-G0B1RE at E. Its die reports **DEV_ID 0x466, REV_ID 0x1003**.
Its **LSE crystal runs** (LSERDY in about 900 ms at the lowest drive,
32719..32753 Hz against the core) with the oscillator bridges at
UM2591's default, so the RTC, the tickless timebase, the timed sleep
sites and every wall run on the crystal exactly as on the Nucleo-64s
and the six suites those touch are configured identically for all three
boards; its **LSI** weighs 31403..31496 Hz on a TIM16 capture and 31400
by the watchdog, the slowest of the three dies. Two board facts shape
suites on it: **PA0 and PA4 lean HIGH when left floating** (the
Nucleo-64s' free pads drift down), which costs the tamper letter its
TAMPPUDIS contrast; and **its debug port can go silent** - a state of
the board's ST-LINK half that only unplugging the board clears - in
which case the ST-LINK's own mass-storage flasher is the way in
(`bin/brio`'s `stlink_msd` programmer kind) and nothing can be
halted or read over SWD until the replug ([../bench.md](../bench.md)).

**FOURTEEN OF THE SEVENTEEN SUITES RUN ON IT.**

| Suite | E | F | G | What skips on G, and why |
|---|---|---|---|---|
| `test_stm32_crc` | 27 | 27 | **27** | nothing - the boards line is the whole change |
| `test_stm32_platform` | 53 (+ `i` 26) | 53 (+ `i` 26) | **53** (+ `i` 26) | nothing; the LED moves to PC6 and LSI reads 31400 Hz |
| `test_stm32_sleep` | 50 (+ `s` 6, `u` 6) | 50 (+ `s` 6, `u` 6) | **50** (+ `s` 6, `u` 6) | nothing; a Shutdown wake is a literal power-on reset here as on the G071 ([pwr.md](pwr.md)) |
| `test_stm32_exti` | 89 | 89 | **85** | IMR2's reset value (no second register group at all) and the user button (PC13 is not bonded) |
| `test_stm32_rtc` | 125 (+ `w` 11, `v` 4) | 125 | **109** (+ `w` 11, `v` 4) | RTC_REFIN (PB15 unbonded), two tamper legs whose level is a board's, and the TAMPPUDIS contrast (a free pad that leans high) ([rtc.md](rtc.md)) |
| `test_stm32_lptim` | 82 | 82 | **78** | the two comparator routes, the comparator trigger row and the two-owner vector (no COMP, no TIM7); the LSE rows are measured |
| `test_stm32_tickless` | 47 | 45 | **45** (+ `u` 1) | the awake-time meter, which needs TIM2's ETR taking MCO |
| `test_stm32_clock` | 42 | 40 | **40** | letter `h`: the 4 us wall needs TIM2's ETR taking MCO, a G0B1 code; the LSE code is the wall here, as on the G071 ([clock.md](clock.md)) |
| `test_stm32_tim` | 118 | 114 | **108** | TIM4, TIM6, TIM7 and TIM15 - four instances the part has not got - and PB13..PB15, which the package does not bond |
| `test_stm32_dma` | 69 | 65 | **62** (+ `u` 3, `w` 0) | DMA2, the console's DMA-fed rate (five channels, and the letters own all five) and the peripheral-to-peripheral leg |
| `test_stm32_analog` | 139 | 136 | **67** | the DAC's letters and the comparators' - this part has neither block, so eight of the eighteen letters are compiled out ([adc.md](adc.md)) |
| `test_stm32_serial` | 88 (+ y/w/v) | 83 | **79** (+ `y` 1, `v` 2, `w` 0) | USART2 is BASIC here (no kernel-clock multiplexer, no wake), LPUART1's only bonded pads are the console's, and there is no LPUART2 |
| `test_stm32_spi` | 38 (peer) | 38 (peer) | **38** (peer) | the eleven self-link letters, COMPILED OUT: SPI2's four pads are bonded to no pin of the LQFP32, so that instrument cannot exist here at any wiring |
| `test_stm32_i2c` | 54 (peer) | 54 (peer) | **54** (peer) | the eleven self-link letters, skipped on the PROBE's answer like everywhere else - this package does bond I2C2's PA11/PA12, so what rules them out is the desk and not the plastic |
| `test_stm32_nvm`, `test_stm32_journal` | 85, 52 | - | - | one flash bank: no storage attic |
| `test_stm32_fdcan` | 96 | - | - | no FDCAN |

**64 KB AND 8 KB ARE PART OF THE DESIGN HERE.** Every image on that
board fits with the cuts named in its own suite: the largest is
`test_stm32_serial` at **61928 bytes of 65536**, then `test_stm32_i2c`
52676, `test_stm32_lptim` 50336 and `test_stm32_dma` 42392; the
hungriest in RAM is `test_stm32_dma` at **6108 bytes**, which leaves the
stack the 2 KB reserved for it. One suite makes a real cut rather than
losing letters: `test_stm32_dma` gives its three memory-to-memory
buffers 256 words there instead of 512 (every claim restated for the
length that fits) and gives the console's two DMA channels back to the
letters. `test_stm32_spi` compiles its eleven self-link letters out - a
package fact, not a budget one - and comes to 31104 bytes with the peer
half in; `test_stm32_i2c` keeps every letter it has anywhere.

**WHAT THE PART HAS NOT GOT**, all of it read off the header by the
reserve: TIM4, TIM6, TIM7, TIM15, USART3..6, LPUART2, I2C3, SPI3, DMA2
(DMA1 keeps five channels and the DMAMUX five with four generators),
**the DAC and every comparator**, FDCAN, USB, CRS, UCPD (so PA8 and PB15
carry no dead-battery Rd), GPIOE, the second flash bank, the EXTI's
entire second register group, WKUP3 and WKUP5, `RCC_CCIPR_I2C2SEL` and
`RCC_CCIPR_USART2SEL` - the last of which is what takes USART2's kernel
clock, its FIFO and its wake with it. And what the PACKAGE does not bond
is a separate list the SUITES own, never the reserve: PB10..PB15,
PC0..PC5, PC7, PC13, PD0..PD3, PF0 and PF1 (GPIOD is declared by the
header and reaches no pin at all). One per-part fact in the reserve
covers that difference: `ucpd_present(n)`, probed on SYSCFG's own
dead-battery strobe bit, so a pad that will not follow its pull can be
told from one that never had an Rd on it.

**Vector names across the boards.** The crt spells ST's own handler
names, and a SHARED line's name changes with what shares it - USART2's
is `USART2_LPUART2_IRQHandler` on the G0B1 and `USART2_IRQHandler` on
the G071/G031. An app for one board binds the bare name; an app for
more than one binds the name the reserve derives from the header's
presence macros, `BRIO_STM32G0_USART2_HANDLER` and its siblings (TIM3,
TIM6, TIM7, TIM16, TIM17, LPTIM1, LPTIM2, USART3, LPUART1, the DMA's
upper line, the ADC) - `test/family_stm32g0/handlers.cpp` proves every
one expands on every header and names the line the reserve's verb
answers. A name outside that set on the wrong board lands in
`Default_Handler`'s silent spin.

```bash
(cd stm32g0 && cmake --preset stm32g0b1re-release)                      # configure (once, or after adding an app)
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>)
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>-upload)
(cd stm32g0 && cmake --preset stm32g071rb-release)                      # the Nucleo-G071RB; stm32g031k8-release the Nucleo-32
brio check stm32g0 [name]                                          # family smoke, no hardware
```

Build outputs land in `build-cmake/stm32g0b1re-{release,debug}` at the
repo root: `<app>.elf/.bin/.hex`, `firmware-<app>.map`, `<app>.lst`. A
configure also writes this project's app roster,
`build-cmake/apps_stm32g0.json`, which `bin/brio` reads: the
board TYPE `g0b1re` is what tells that tool to build here and to flash
through OpenOCD's ST-LINK interface (`bench.py flash E <app>`). NB
`bench.py run` speaks the bench SUITES' single-letter grammar (no
line terminator): the line-oriented `console` app is driven with any
serial monitor, or pyserial, at 115200 8N1.

## Upload (OpenOCD, ST-LINK)

Flashing goes through OpenOCD driving the Nucleo's on-board
ST-LINK/V2.1: `interface/stlink.cfg` + `target/stm32g0x.cfg` (the
stm32l4x flash driver underneath) + `program <app>.elf verify`, then
`reset run` and a write of DHCSR that clears C_DEBUGEN: a core left with
halting debug enabled HALTS on a BKPT instead of faulting, and every
`panic()` ends in one. OpenOCD - the 0.12.0 release built from its
tarball into `/sw/openocd-0.12.0` (`/sw/openocd`), see
[the SAM page](../samc21/README.md) - drives the ST-LINK (firmware
V2J46M31) without incident; the probe carries a REAL USB
serial, so `adapter serial` names it and the same serial names the
console under `/dev/serial/by-id`. Single-client: close the debug
session before flashing.

ONE SWD CAVEAT worth knowing: memory reads THROUGH THE HLA TRANSPORT
WHILE THE CORE SLEEPS IN WFI ARE UNRELIABLE - a running console
(WFI between events) answered `0xffffffb7` for FLASH_ACR and zeros
for RCC_CR, values those registers cannot hold, while the same reads
after `halt` were exact. Halt first, read, resume; the CMSIS-DAP probe
on the SAM C21 board does not do this.

A SECOND SWD CAVEAT, and one that reads as a silicon fact until it is
found: `target/stm32g0x.cfg`'s examine-end hook writes
`DBGMCU_CR.DBG_STOP | DBG_STANDBY` ("enable debug during low power
modes"), and OpenOCD re-examines the target after every reset it
issues, so the bits are back on the reset a `program` ends with
whatever was cleared before it (measured: cleared to 0 while halted,
read 6 again right after `reset run`). The register survives every
reset but a power-on, and with DBG_STOP set the debug logic keeps
HCLK - and SysTick - running inside a Stop, which cuts a Stop entered
with the kernel tick armed from its full 250 ms to one tick.
`bin/brio` therefore ends every G0 flash with `reset halt`, a
clear of DBGMCU_CR through its clock gate (the gate put back to its
reset value) and a `resume`, so a board leaves the bench as a power-on
would leave it; a cortex-debug session sets the bits again, and
`Pwr::debug_in_stop()` ([pwr.md](pwr.md)) is how firmware tells. Both
states are measured in `test_stm32_sleep` letter c.

## Debugging (cortex-debug + OpenOCD)

The launch config is "Debug STM32G0 (OpenOCD, Nucleo-G0B1RE)" in
`.vscode/launch.json`: the SAM entry's shape with the two ST config
files, `adapter serial` through `openOCDPreConfigLaunchCommands`, and
`svdPath` at `stm32g0/svd/STM32G0B1.svd`. CMake Tools' Active Folder
must be `stm32g0/` and its launch target the app to debug. Not yet
exercised at the bench (mature tooling gets the light verification
policy).

## Editor (clangd)

`brio/stm32g0/.clangd` and `stm32g0/.clangd` route the stratum and the
project to `build-cmake/stm32g0b1re-release`;
`test/family_stm32g0/.clangd` lets the script-compiled family TUs
borrow flags from the same database. The repo-root `.clangd` rules
apply unchanged.

## Serial console

The ST-LINK's virtual COM port enumerates with the probe's own USB
serial, so the console is addressed by `/dev/serial/by-id` and never
moves with the socket - the first console on this desk that does not
need the by-path dance. Console apps run 115200 8N1. Measured: BRR 556
at 64 MHz gives 115107 baud (the arithmetic to the hertz), 300 lines
of 50 bytes exchanged with zero errors of any kind, and the kernel
tick +0.24 % against the PC's clock (inside HSI16's 1 % calibration).
