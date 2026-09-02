# Target: STM32G0 (`stm32g0/`)

The operational page for brio's third hardware target: an ARM
Cortex-M0+ from the other big vendor (STM32G0B1RE on the bench, on an
ST Nucleo-64), the second ARMv6-M family in the tree - and the target
that proved, for the third time, that the kernel and util strata
compile UNCHANGED on a new architecture: blink under time events and
the full console over the board's own virtual COM port, with not one
line of `kernel/` or `util/` touched.

Peripheral documents live next to this page (`platform.md`,
`clock.md`, `port.md`, `usart.md` - all PROVISIONAL, this is a
bring-up); the documents of record are in
[vendor/README.md](vendor/README.md) together with the errata pass
and the bench chip's identity (silicon revision Z, DBGMCU_IDCODE read
over SWD).

## Toolchain

Self-built **arm-none-eabi-gcc 16.2** at `/sw/arm-none-eabi` - the
same compiler, flags and linker discipline as the samc project
(`stm32g0/cmake/toolchain-arm.cmake` is that file verbatim:
`CMAKE_SYSTEM_NAME Generic`, `STATIC_LIBRARY` try-compile,
`--specs=nano.specs -nostartfiles`, deliberately NO syscall stubs so
an accidental `_sbrk`/`_write` fails the link). The `armv6m/` core
stratum the naming rule calls for at the second ARM family is factored
AFTER this bring-up, with both implementations in hand and a
byte-identity gate on every samc image; until then `nvic.hpp` and
`ticker.hpp` are the samc files' twins by discipline.

The device headers are vendored: `third_party/cmsis-device-g0/`
(ST's cmsis-device-g0 v1.4.5, every G0 part) and the shared
`third_party/cmsis-core/`. The device is selected ST's way, by a plain
`-DSTM32G0B1xx` define that the umbrella `stm32g0xx.h` dispatches on -
no device-specs machinery, so clangd needs no macro-delta feed.

## Board and build

The bench board is an **ST Nucleo-G0B1RE** (MB1360): STM32G0B1RE
(LQFP64, 512 KB dual-bank flash, 144 KB SRAM), silicon revision Z,
running at **3.3 V**. Verified at the bench, each by its own
experiment and not by the user manual: LD4 on **PA5** (driven over
SWD before a line of firmware, then by `probe`), the ST-LINK virtual
COM port on **USART2 PA2 (TX) / PA3 (RX), AF1** (the console answers
through it), the CPU on **HSI16 through the PLL at 64 MHz**. NOT yet
verified: the LSE 32.768 kHz crystal the Nucleo-64 ships with (no
consumer yet), the user button B1 on PC13, and the absence of an HSE
crystal (X3 is not fitted by default and the ST-LINK's 8 MHz MCO
reaches HSE only through solder bridges - the HSE root is unbuilt
anyway).

`stm32g0/` is its own CMake project, a sibling and peer of `avrdx/`,
`samc/` and `test/`. Apps are auto-discovered from
`stm32g0/src/apps/*.cpp` - plus `experiments/*/stm32g0/*.cpp` - by
their `// build:` header comment, the other two projects' grammar with
this family's board names (`boards = g0b1re`, the default). One
configure targets one part (`STM32G0_MCU`, full part number: it decides
the device define `STM32G0B1xx`, the linker script `ld/<part>.ld` and
the crt `src/glue/startup_<header>.cpp`); only the G0B1RE has a preset
today - the G071 and G031 (Nucleo boards in the drawer) are
compile-checked by `tools/check_stm32g0.sh`, which sweeps every
positive TU in `test/family_stm32g0/` across the three device headers
and requires every `neg/` TU to fail for the variants its `// mcu:`
line names.

```bash
(cd stm32g0 && cmake --preset stm32g0b1re-release)                      # configure (once, or after adding an app)
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>)
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>-upload)
tools/check_stm32g0.sh [name]                                          # family smoke, no hardware
```

Build outputs land in `build-cmake/stm32g0b1re-{release,debug}` at the
repo root: `<app>.elf/.bin/.hex`, `firmware-<app>.map`, `<app>.lst`. A
configure also writes this project's app roster,
`build-cmake/apps_stm32g0.json`, which `tools/bench.py` reads: the
board TYPE `g0b1re` is what tells that tool to build here and to flash
through OpenOCD's ST-LINK interface (`bench.py flash E <app>`). NB
`bench.py run` speaks the bench SUITES' single-letter grammar (no
line terminator): the line-oriented `console` app is driven with any
serial monitor, or pyserial, at 115200 8N1.

## Upload (OpenOCD, ST-LINK)

Flashing goes through OpenOCD driving the Nucleo's on-board
ST-LINK/V2.1: `interface/stlink.cfg` + `target/stm32g0x.cfg` (the
stm32l4x flash driver underneath) + `program <app>.elf verify`, then
`reset run` and a write of DHCSR that clears C_DEBUGEN - the samc
lesson, kept: a core left with halting debug enabled HALTS on a BKPT
instead of faulting, and every `panic()` ends in one. The oss-cad-suite
OpenOCD at `/sw/oss-cad-suite/bin/openocd` drives the ST-LINK
(firmware V2J46M31) without incident; the probe carries a REAL USB
serial, so `adapter serial` names it and the same serial names the
console under `/dev/serial/by-id`. Single-client: close the debug
session before flashing.

ONE SWD CAVEAT worth knowing: memory reads THROUGH THE HLA TRANSPORT
WHILE THE CORE SLEEPS IN WFI ARE UNRELIABLE - a running console
(WFI between events) answered `0xffffffb7` for FLASH_ACR and zeros
for RCC_CR, values those registers cannot hold, while the same reads
after `halt` were exact. Halt first, read, resume; the samc board
(CMSIS-DAP) never showed this.

## Debugging (cortex-debug + OpenOCD)

The launch config is "Debug STM32G0 (OpenOCD, Nucleo-G0B1RE)" in
`.vscode/launch.json`: the SAM entry's shape with the two ST config
files, `adapter serial` through `openOCDPreConfigLaunchCommands`, and
`svdPath` at `stm32g0/svd/STM32G0B1.svd`. CMake Tools' Active Folder
must be `stm32g0/` and its launch target the app to debug. Not yet
exercised at the bench (the samc entry is the proven twin; the light
verification policy for mature tooling applies).

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
