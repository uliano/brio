# Working in VS Code

The graphical route for building, flashing and debugging a brio
application, documented for the [first program](first-program.md) on the
ST Nucleo-G0B1RE. The repository carries a VS Code configuration
(`.vscode/settings.json`, `.vscode/launch.json`,
`.vscode/extensions.json`); this page explains what it sets up, how to
drive it, and for each step whether it has been performed on this
configuration or only configured.

## Verification status of this page

Every step below is marked with one of four words:

- verified: performed interactively on the named configuration. A
  result obtained on a sibling board with a sibling launch entry is
  reported beside the mark as what it is, and never counted as
  verified for this board;
- configured: the files set it up, and nothing on the desk has
  contradicted it, but the interactive check on this board is still
  owed;
- terminal: there is no documented graphical action; the command line
  does it;
- blocked: not available, with the reason.

The evidence behind the marks is the configuration itself and
[docs/stm32g0/README.md](../stm32g0/README.md), whose "Debugging"
section records a debug session on the Nucleo-G071RB from VS Code with
this launch configuration's sibling (load, reset-halt, a breakpoint at
`main` hit, source lines stepped). A configuration file is not proof of
a working session, so nothing on this page about the G0B1RE is marked
verified until the sequence has been run there. Every "Expected result"
below describes what the configuration asks the tools to do; where it
goes beyond what the G071RB session recorded, it says so.

## The matrix

| Step | Status | Where |
|---|---|---|
| Open the repository folder | configured | the folder is the workspace; no multi-root file |
| Install the recommended extensions | configured | `.vscode/extensions.json` |
| Code navigation and diagnostics (clangd) | configured | needs a configured project; `.clangd` fragments route each stratum |
| Select the active CMake project (`stm32g0/`) | configured | CMake Tools status bar, "Active Folder" |
| Select the configure preset (`stm32g0b1re-debug`) | configured | CMake Tools status bar |
| Configure and build `blink` | configured | CMake Tools status bar |
| Flash (`blink-upload` as the build target) | configured | the same build button, another target |
| Select the executable to debug | configured | CMake Tools launch target |
| Start a debug session, stop at `main` | configured (the same sequence was performed on the Nucleo-G071RB with its own entry) | Run and Debug, "Debug STM32G0B1RE (OpenOCD, Nucleo-G0B1RE)" |
| Peripheral registers in the debugger | configured | the Peripheral Viewer, `svdPath` in the launch entry |
| Serial output | terminal for brio's console apps; not applicable to `blink` | any serial terminal; the Serial Monitor extension is not among the recommendations |
| Host tests | configured | CTest from the status bar with `test/` as the active folder |

The rest of the page is the procedure, step by step.

## Open the right folder

Open the repository root as the workspace folder (File > Open Folder;
a plain folder, there is no `.code-workspace` file). The root is not a
CMake project on purpose: it holds eleven sibling CMake projects, one
per target plus `test/` and `host/`, and `.vscode/settings.json` lists
them all in `cmake.sourceDirectory`. Opening a subdirectory such as
`stm32g0/` alone loses the workspace settings and the launch
configurations.

## Extensions

`.vscode/extensions.json` recommends four extensions, and VS Code
offers to install them when the folder is opened:

| Extension | Role on this route |
|---|---|
| CMake Tools (`ms-vscode.cmake-tools`) | selects the project, the preset, the build target, the debug executable; builds |
| clangd (`llvm-vs-code-extensions.vscode-clangd`) | code navigation and diagnostics, from each project's compile database |
| Cortex-Debug (`marus25.cortex-debug`) | the debugger for the ARM targets, driving OpenOCD and arm-none-eabi-gdb |
| Peripheral Viewer (`mcu-debug.peripheral-viewer`) | the register view beside the debugger, from the part's SVD file |

Two things the recommendations do not supply. Microsoft's C/C++
extension (`ms-vscode.cpptools`) is not recommended, and its own code
engine is disabled in the settings in favour of clangd; the launch
entries for the AVR and the WCH RISC-V boards use its `cppdbg` debugger
type, so those routes need it installed while the STM32G0 route does
not. And the Serial Monitor extension is referenced by one setting
(`vscode-serial-monitor.customBaudRates`) without being recommended;
`blink` has no serial output, so this route does not need it either.

The tools the extensions drive are not installed by them. Cortex-Debug
is configured with absolute paths to OpenOCD (`/sw/openocd/bin/openocd`)
and to the toolchain's gdb and objdump (`/sw/arm-none-eabi/bin/`); CMake
Tools uses the toolchain file the project names, which expects the
compiler under `/sw/arm-none-eabi`. The [command-line
page](command-line.md) lists what to install. A toolchain elsewhere is
told to the build the way the command line does it, through the cache
variable `ARM_TOOLCHAIN_DIR` (and `STM32G0_OPENOCD` for the flash
target): in VS Code, CMake Tools' setting `cmake.configureSettings` in
the workspace's or the user's `settings.json` adds cache variables to
every configure, or a preset of one's own can set them; the launch
entries carry their paths literally, so `.vscode/launch.json` is edited
too, and so is clangd's `--query-driver` (below). None of these
relocations has been performed for this page.

## clangd

clangd reads each project's `compile_commands.json`, which every
configure writes (`CMAKE_EXPORT_COMPILE_COMMANDS` is on). The `.clangd`
fragments in the repository route each stratum to its own project's
database, so a header under `brio/stm32g0/` parses with the ARM
compiler's flags whatever project is active. Until the project has been
configured once there is no database and clangd shows errors on every
include; configure first. The `--query-driver` setting lets clangd ask
the real cross compiler for its include paths; its globs are under
`/sw/`, so a toolchain elsewhere needs that setting edited.

## Select the project and the preset

CMake Tools shows its controls in the status bar (the setting
`cmake.options.statusBarVisibility` forces them visible, since recent
versions hide them by default). Three choices are made there before
anything is built, and the order matters:

1. Active folder: `stm32g0`. With several projects in one workspace,
   CMake Tools keeps one active and every other control refers to it.
   The command palette entry is "CMake: Select Active Folder". Expected
   result: the status bar names the folder.
2. Configure preset: `STM32G0B1RE (debug, -Og)`, which is the preset
   `stm32g0b1re-debug`. The display names come from
   `stm32g0/CMakePresets.json`; a release preset builds with `-Os` and
   is the one to flash for a measurement, the debug one is what the
   debugger wants. Expected result: CMake Tools configures, and the
   output panel shows the ARM compiler under `/sw/arm-none-eabi` and
   the build tree `build-cmake/stm32g0b1re-debug`.
3. Build target: `blink`. The target list is the applications the
   configure discovered for this board, each with its `-upload` twin.
   Expected result: the status bar shows the chosen target's name next
   to the build button.

The wrong active folder is the most common cause of a confusing
failure: a build target list with AVR or host names in it, or a debug
session that loads an ELF from another family's build tree, means the
active folder is not `stm32g0`. Check it before anything else.

The status bar items and the palette entries named here are CMake
Tools' own; their exact labels are to be confirmed against the
installed version in the interactive check this page still owes.

## Build

With the three choices made, the status bar's build button (or
"CMake: Build", F7) builds the selected target. Expected result: the
same output as the command-line build, ending with the size table, and
`blink.elf` in `build-cmake/stm32g0b1re-debug/`.

## Flash

Flashing goes through the application's `-upload` target, not through
CMake Tools' own run button (the settings hide CMake Tools' launch and
debug buttons on purpose, since neither knows how to program a board).
Select `blink-upload` as the build target and build: OpenOCD programs
and verifies the image through the board's ST-LINK, resets it and
exits, and the LED starts blinking. Then select `blink` again as the
build target, or leave `blink-upload`, which rebuilds `blink` when its
sources changed. Like the command-line `-upload`, this leaves the
DBG_STOP bit set in the chip (OpenOCD's target script sets it at every
examination, and the reset that ends the programming is followed by
one); it changes nothing for `blink` and matters for a program that
measures the Stop modes ([the command-line page](command-line.md),
step 3).

What to check when nothing happens: the output panel shows OpenOCD's
messages. "open failed" or a permission error means the probe could not
be reached (USB permission, cable); "no device connected" means the
probe answered but sees no target, which on a Nucleo is the ST-LINK
half of the board gone silent: replug the board as a whole before any
other diagnosis ([docs/probes/st-link.md](../probes/st-link.md)). A
debug session and a flash cannot share the probe: stop the session
first.

## Debug

Three things must agree before a session starts, and they are chosen
in three different places:

- the active folder and configure preset (above): `stm32g0`,
  `stm32g0b1re-debug`, so that the ELF carries debug information
  (`-Og -g3 -fno-inline`) and matches the target script;
- the launch target of CMake Tools: `blink`. The launch configuration
  reads the executable from `${command:cmake.launchTargetPath}`, which
  is this choice and not the build target. The two usually coincide;
  after flashing with `blink-upload` as the build target, the launch
  target is still `blink`. The command palette entry is "CMake: Set
  Launch/Debug Target"; expected result: the status bar shows the
  launch target's name;
- the launch configuration, in the Run and Debug view: "Debug
  STM32G0B1RE (OpenOCD, Nucleo-G0B1RE)". It is a Cortex-Debug entry:
  it starts OpenOCD with the ST-LINK interface and the `stm32g0x`
  target script, loads the ELF, resets, and runs to `main` (its
  `runToEntryPoint` setting, which names the function the session
  stops at after loading). The G071RB and G031K8 entries differ only in
  the SVD file; with one Nucleo attached, any of the three reaches it,
  but the Peripheral Viewer then shows another part's registers.

Start the session (F5). What was recorded, on the Nucleo-G071RB with
its own entry: the program loaded, the chip reset and halted, a
breakpoint at `main` hit, source lines stepped. What the configuration
asks for and this page therefore expects, none of it recorded: the
Debug Console showing OpenOCD connecting to the ST-LINK and the part,
then the ELF being loaded, with the ELF's path in those first lines (a
path under `build-cmake/stm32g0b1re-debug/` is the right one, and any
other build tree there means the wrong active folder or launch target);
the editor stopping at the first line of `main` in `blink.cpp` with the
call stack showing `main`; a breakpoint on the `Led::toggle()` line
inside `Blinker::running` hit within half a second of continuing; the
Peripheral Viewer, in the Run and Debug side bar, showing the
STM32G0B1's registers from `stm32g0/svd/STM32G0B1.svd`, with GPIOA's ODR
bit 5 following the LED.

If the session loads an executable from another family, or fails to
find the ELF: check the active folder and the launch target before
suspecting the probe. If OpenOCD cannot open the probe, a `blink-upload`
from the same workspace fails the same way, which separates a probe
problem from a configuration one. With several ST-LINKs attached, the
entry takes the first one; the comment above the entry in
`.vscode/launch.json` says how to name a serial
(`openOCDPreConfigLaunchCommands` with `adapter serial`).

What a session leaves behind: OpenOCD's target script sets the
DBG_STOP bit in DBGMCU_CR (as a flash does, above), which keeps the
clocks running in the deeper Stop modes until a power-on or a `brio
flash` clears it. `blink` sleeps in plain Sleep, where the bit changes
nothing; a program measuring Stop right after a debug session or a flash
does not measure Stop ([docs/stm32g0/README.md](../stm32g0/README.md),
"Upload").

## Serial output

`blink` prints nothing. The applications that do (the console and the
test suites) speak through the ST-LINK's virtual COM port at 115200
8N1, which appears on Linux under `/dev/serial/by-id/` with the probe's
own serial number in its name. Any serial terminal reads it; `brio
console <position>` prints the device path and speed for a board in the
bench manifest. There is no graphical step documented for this: the
Serial Monitor extension is configured by one setting and not among
the recommended extensions, and no session with it is on record.

## Host tests

The `test/` project builds the kernel's and the services' test suites
for the host and registers them with CTest. From VS Code: make `test`
the active folder, select its one preset (`host`), build, and run the
tests from CMake Tools' CTest item in the status bar (the one the
settings keep visible for this purpose; its label is the extension's
and is confirmed with the rest in the interactive check). Expected
result: every suite passes, and the status bar item reports the count. The equivalent on the command line is in the [command-line
page](command-line.md). The host compiler is the preset's
`/sw/gcc/bin/g++`.

## What VS Code adds, and what stays on the command line

The editor adds navigation with real compile flags, a build and flash
button, and a debugger with a register view. The bench verbs
(`brio flash`, `brio run`, `brio check`, `brio gate`) have no graphical
counterpart and are not meant to have one: they are the route for
measurements and for the checks every change runs.
