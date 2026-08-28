# Target: SAM C21 (`samc/`)

The operational page for brio's second hardware target: an ARM
Cortex-M0+ (ATSAMC21J18A on the bench), the first target whose crt -
linker script, startup, vector table - is the project's own work
rather than the toolchain's gift, and the target that proved the
kernel and util strata compile UNCHANGED on a second architecture.

Peripheral documents live next to this page (`platform.md`,
`clock.md`, `port.md`, `sercom.md`, `dmac.md`, `ac.md`, `nvm.md`,
`reset.md`, `freqm.md`, `osc32kctrl.md`, `evsys.md`, `eic.md`,
`tc.md`); the
datasheets of record are in
[vendor/README.md](vendor/README.md) together with the targeted
errata pass and the bench chip's identity (silicon rev F, DSU DID
read over SWD).

## Toolchain

Self-built **arm-none-eabi-gcc 16.2** at `/sw/arm-none-eabi` (same
vintage as the AVR and host compilers, never a system-packaged one),
pointed at by absolute path in `samc/cmake/toolchain-arm.cmake`
(`CMAKE_SYSTEM_NAME Generic`, `STATIC_LIBRARY` try-compile - the
standard bare-metal pattern: no executable links until the linker
script and startup are supplied per app).

Unlike avr-gcc, this toolchain ships **no device headers**, so the
CMSIS device pack is vendored in the repository and a fresh clone
builds: `third_party/samc21-dfp/` (Microchip.SAMC21_DFP include tree,
every C21 E/G/J variant) and `third_party/cmsis-core/` (the five
CMSIS-Core headers the device header needs). The device is selected
by an ordinary define (`-D__SAMC21J18A__`, derived from the
`SAMC_MCU` cache variable) which `sam.h` dispatches on - no
device-specs macro machinery and no `-mmcu` equivalent, which is also
why clangd needs no macro-delta feed on this target.

Link: `--specs=nano.specs -nostartfiles` and deliberately **no
syscall stubs**: brio allocates nothing and calls no syscalls, so
anything dragging `_sbrk`/`_write` in fails the link loudly instead
of failing at run time. The one libc symbol every image does define
itself is `abort()` (see `platform.md`, "The crt").

## Board and build

The bench board is the user's own C21J rev 1.1 (custom, not the
Xplained Pro): ATSAMC21J18A, CPU on the internal OSC48M at 48 MHz, a
24 MHz crystal on PA14/PA15 reserved for future use (NOT enabled),
console CH340 on PB30/PB31 (SERCOM5 PAD0/PAD1), user button PB22,
LED PB23, SWD on PA30/PA31.

`samc/` is its own CMake project, a sibling and peer of `avrdx/` and
`test/` (one configure has exactly one compiler; the repo root is not
a CMake project). Apps are auto-discovered from
`samc/src/apps/*.cpp` by their `// build:` header comment - the same
grammar as the AVR project, board names of this family (`boards =
c21j`; c21j is also the default). One configure targets one chip
variant (`SAMC_MCU`); only the J18A has a preset today - the E/G
variants have no board on the desk and are compile-checked by
`tools/check_samc.sh` instead, which sweeps every positive TU in
`test/family_samc/` across the E/G/J 18A headers and requires every
`neg/` TU to fail for the variants its `// mcu:` line names.

```bash
(cd samc && cmake --preset samc21j-release)                       # configure (once, or after adding an app)
(cd samc && cmake --build --preset samc21j-release --target <app>)
(cd samc && cmake --build --preset samc21j-release --target <app>-upload)
tools/check_samc.sh [name]                                        # family smoke, no hardware
```

Build outputs land in the shared `build-cmake/samc21j-{release,debug}`
at the repo root: `<app>.elf/.bin/.hex`, `firmware-<app>.map`,
`<app>.lst`. A configure also writes this project's app roster,
`build-cmake/apps_samc.json`, which `tools/bench.py` reads: the board's
TYPE (`c21j`) is what tells that tool to build here and to flash
through OpenOCD instead of avrdude, so a SAM suite is driven exactly
like an AVR one (`bench.py flash C <app>`, `bench.py run C z`). The
`-upload` target remains the flash path that needs no manifest.

## Upload (OpenOCD, SWD)

Flashing goes through OpenOCD driving the Atmel-ICE as a CMSIS-DAP
probe: `interface/cmsis-dap.cfg` + `target/at91samdXX.cfg` (the
at91samd flash driver auto-probes the geometry from the DSU DID) +
`program <app>.elf verify reset exit`. The oss-cad-suite OpenOCD
build at `/sw/oss-cad-suite/bin/openocd` drives the ICE flawlessly.
Two Atmel-ICE probes live on this desk, so `adapter serial` is
mandatory (the `SAMC_PROBE_SERIAL` cache variable, default the SAM
board's probe); flashing requires the debug session to be closed -
the ICE is single-client.

## Debugging (cortex-debug + OpenOCD)

The launch config is "Debug SAM (OpenOCD, C21J board)" in
`.vscode/launch.json`: `servertype: openocd` (cortex-debug launches
the server itself and parses its output natively - none of the
buffering shims PyAvrOCD needed), the same two `-f` configs plus
`adapter serial` via `openOCDPreConfigLaunchCommands`, gdb from the
cross toolchain, `runToEntryPoint: main`, and `svdPath` at
`samc/svd/ATSAMC21J18A.svd` feeding the same mcu-debug Peripheral
Viewer the AVR side uses. CMake Tools' Active Folder must be `samc/`
and its launch target the app to debug - the executable comes from
`${command:cmake.launchTargetPath}` exactly as on the AVR entry.

Verified at the bench: stop at `main`, user breakpoints hit. By
policy the verification is light - cortex-debug and OpenOCD are
mature tooling, unlike PyAvrOCD which earned its exhaustive
treatment; quirks get documented here as they are found. One SWD
technique worth knowing: with the CPU halted, `mdw`/`mdb` from
OpenOCD reads any peripheral register - it diagnosed a wrong-looking
serial line in one pass where guessing host baud rates went nowhere.

## Editor (clangd)

clangd routes every file to its own architecture through `.clangd`
fragments, independent of which project CMake Tools has active:
`brio/.clangd` sends the framework default (kernel/, util/,
host/) to the host database, `brio/samc/.clangd` and
`samc/.clangd` send the samc stratum and project to
`build-cmake/samc21j-release`, `test/family_samc/.clangd` lets the
script-compiled family TUs borrow flags from that same database. The
repo-root `.clangd` suppresses the two clang-only diagnostics that
the inherited `-Werror` would otherwise render as editor errors on
gcc-clean code, and tells the include-cleaner that `sam.h` is the
intended umbrella (including the device header directly is exactly
what the code must not do).

## Serial console

The board's CH340 enumerates without a USB serial number, like every
CH340 on this desk, and `/dev/serial/by-path` names move with the USB
socket - identify the SAM console by listening on the candidates
during an OpenOCD `reset run` and seeing which one prints the boot
banner. Console apps run 115200 8N1.
