# An application in a directory of its own

brio's own applications live inside brio's tree, one file each under
`<family>/src/apps/`, found by the family's build project when it is
configured. A program of yours does not have to live there. It can sit
in a directory of its own, anywhere on the disk, under version control
of its own, with a `CMakeLists.txt` of five lines that includes brio's
`cmake/brio.cmake`. Its image is then built by the same recipe as
brio's own applications - the family's `cmake/family.cmake`: the same
compiler, flags, startup code, linker script and runtime - so the same
source gives the same bytes, and it gets the same `-upload` target.

This page builds a copy of `blink` that way for the ST Nucleo-G0B1RE,
the board of the [command-line route](command-line.md), and then for
other parts and boards. It assumes that route's toolchain and its first
two steps; the [first program](first-program.md) explains what `blink`
does.

## What has been verified

Every step was performed on the host this repository is developed on,
with the toolchains named in [the command-line route](command-line.md);
building needs no board. A step is verified when it was performed as
written, configured when the files set it up and the check is still
owed:

| Step | Status |
|---|---|
| 1. The directory and its `CMakeLists.txt` | verified as written: configure and build of a copy of `stm32g0/src/apps/blink.cpp` for the STM32G0B1RE |
| 2. One build directory per part | verified as written: the same copy built for the STM32G071RB in a second directory, and the refusal of a directory configured for another family |
| 3. A board file | verified on the CH32V006K8 with a copy of `ch32v00x/src/apps/blink.cpp`: brio's `v006k8` board file through `BRIO_BOARD`, and a copy of it kept in the application's directory (the two images byte-identical); the refusal of an unknown board type and the compile error of a board file whose part is not the build's |
| 4. Presets of your own | verified as written for the release and debug presets of the STM32G0B1RE, the release image byte-identical to step 1's |
| 5. The build directory | verified: the files listed, after every build above |
| 6. Flash with the `-upload` target | verified on an AVR128DB48 board through a PICkit 4, `-DBRIO_PROBE=pickit4` with and without `-DBRIO_PROBE_SERIAL`: the image written and read back by avrdude. On the STM32G0B1RE as written, and for every other family and probe kind, configured, not run: the command was read in the build directory's `build.ninja` |
| What `brio check` does | verified: `brio check all` ends with the external guard on every family |

"Expected" paragraphs describe what the tools print by design, not a
transcript.

## What you need

- brio's tree, checked out anywhere. The application names its
  `cmake/brio.cmake` by path; nothing is installed.
- The family's cross compiler, exactly as for brio's own applications.
  `cmake/brio.cmake` names the family's own toolchain file
  (`stm32g0/cmake/toolchain-arm.cmake` for this board), so the same
  override applies: `-DARM_TOOLCHAIN_DIR=/opt/arm` at the first
  configure for a compiler under `/opt/arm/bin`. A
  `CMAKE_TOOLCHAIN_FILE` given on the command line is used instead.
- CMake 3.25 or newer: `cmake/brio.cmake` stops the configure on an
  older one. A generator: every step on this page was performed with
  Ninja, the one brio's own presets name.
- For flashing, the family's programmer and OpenOCD, as for brio's own
  applications (step 6).

## Step 1: the directory and its CMakeLists.txt

The smallest application is a directory with two files:

```text
my_app/
    CMakeLists.txt
    src/main.cpp
```

Start from a maintained example rather than an empty file: from brio's
tree, `cp stm32g0/src/apps/blink.cpp my_app/src/main.cpp`. The
`CMakeLists.txt` is complete as it stands, with the path to brio
replaced by yours:

```cmake
cmake_minimum_required(VERSION 3.25)
set(BRIO_MCU stm32g0b1re CACHE STRING "the part this firmware is for")
include(/path/to/brio/cmake/brio.cmake)
project(my_app)
brio_add_app(my_app src/main.cpp)
```

**The include comes before `project()`.** The part decides the
compiler, and one CMake configure has exactly one compiler, chosen when
`project()` runs. `cmake/brio.cmake` reads `BRIO_MCU`, finds the part's
family, names that family's toolchain file, and asks `project()` to
load the family's recipe as its last step. The recipe defines
`brio_add_app(<name> <source>...)`: one image per call, from as many
sources as you list, relative to the application's directory. What
follows `project()` is ordinary CMake: your own include directories and
definitions go on the target that `brio_add_app()` made, for example
`target_include_directories(my_app PRIVATE include)`.

Two families take one option of their own after the sources: the AVR's
`FLMAP_LOCK 0|1` and the CH32X035's `SCHEDULER ram|flash`. The comment
above `brio_add_app()` at the end of the family's recipe
(`avrxt/cmake/family.cmake`, `ch32x035/cmake/family.cmake`) says what
each sets and its default.

Configure and build, from the application's directory:

```bash
cd my_app
cmake -S . -B build/stm32g0b1re -G Ninja
cmake --build build/stm32g0b1re
```

Expected: the configure reports the compiler it found under the
toolchain directory; the build compiles `src/main.cpp`, the startup
code and brio's runtime, links `my_app.elf` and ends with the section
table `arm-none-eabi-size` prints for it. The build type is release
(`-Os`) unless you ask for another: `-DCMAKE_BUILD_TYPE=Debug` at the
configure gives the debug flags (`-Og -g3 -fno-inline`) that the
command-line route's debug preset uses.

If it fails: "no family builds for BRIO_MCU" means a part name
`cmake/brio.cmake` does not know; its header lists how each family's
parts are spelled. A part whose family is known but which that family
has not got stops the configure, naming what the recipe looked for:
the startup code it derives from the part on the STM32G0, the part's
row of the part table on the STM32F4. A
compile error inside a brio header is the same as for brio's own
applications ([the command-line route](command-line.md), step 2).

## Step 2: BRIO_MCU, and one build directory per part

`BRIO_MCU` is the part as the family's own presets spell it:
`avr128db48`, `samc21j18a`, `stm32g0b1re`, `stm32f446re`, `ch32v006k8`,
`ch32v203c8`, `ch32x035f8`, `rp2040`, and `rp2350a` or `rp2350b` (the
letter is the package). The `set(... CACHE ...)` line in the
`CMakeLists.txt` is the default; a configure with `-DBRIO_MCU=<part>`
overrides it.

A family's further choices are its recipe's own cache variables, given
the same way: the RP2350's `RP2350_ARCH` (`arm` or `riscv`, which picks
the compiler, so it is given at the first configure of a build
directory), the flash of the two Raspberry Pi chips (`RP2040_FLASH_KB`
and its boot stage `RP2040_BOOT2`, `RP2350_FLASH_KB`), the QingKe
families' hardware prologue switch (`CH32V00X_HPE` and its siblings).
Each family's recipe, `<family>/cmake/family.cmake`, declares its own.

Each part gets a build directory of its own:

```bash
cmake -S . -B build/stm32g071rb -G Ninja -DBRIO_MCU=stm32g071rb
cmake --build build/stm32g071rb
```

This builds the same `blink` for the Nucleo-G071RB; its source picks the
LED's pin from the part. `cmake/brio.cmake` refuses a build directory
first configured for another family ("this build directory was
configured for the stm32g0 family and BRIO_MCU=avr128db48 is of avrxt"),
because the compiler cannot change under it. A second part of the same
family in the same directory is not refused, and is not what this page
verified: one directory per part keeps each part's image.

## Step 3: a board file

A board file states the facts of one assembly to the compiler - the
LED's pin, the crystal, what hangs on which pad - as constants an
application reads with `#include "board.hpp"`
([docs/boards/README.md](../boards/README.md), "The board file"). There
are two ways to give one to an application outside the tree.

**One of brio's boards.** `BRIO_BOARD` names a board type with a
directory under `boards/`, and that directory goes on the image's
include path. The CH32V006's `blink` reads its LED from the board file:

```bash
cmake -S . -B build/v006k8 -G Ninja -DBRIO_MCU=ch32v006k8 -DBRIO_BOARD=v006k8
```

A board type with no `boards/<type>/board.hpp` stops the configure
("BRIO_BOARD='...' has no board file"). Every board file checks that the
build's part is the one fitted on its board, so `-DBRIO_MCU=ch32v003f4
-DBRIO_BOARD=v006k8` configures and then fails to compile at that check.

**A board of your own.** Keep its `board.hpp` among the application's
files, leave `BRIO_BOARD` unset, and add its directory after
`brio_add_app()`:

```cmake
target_include_directories(my_app PRIVATE board)
```

brio's own board files are the model to copy: the namespace `board`,
`board::type`, and the check of the part. An application that includes
`board.hpp` with neither fails to compile with "board.hpp: No such file
or directory".

## Step 4: presets of your own

The command lines of steps 1 to 3 can be written once, as presets in a
`CMakePresets.json` beside the application's `CMakeLists.txt` - the
same mechanism brio's own projects use, with the part as a cache
variable:

```json
{
    "version": 6,
    "configurePresets": [
        {
            "name": "g0b1re",
            "generator": "Ninja",
            "binaryDir": "${sourceDir}/build/${presetName}",
            "cacheVariables": { "BRIO_MCU": "stm32g0b1re", "CMAKE_BUILD_TYPE": "Release" }
        },
        {
            "name": "g0b1re-debug",
            "generator": "Ninja",
            "binaryDir": "${sourceDir}/build/${presetName}",
            "cacheVariables": { "BRIO_MCU": "stm32g0b1re", "CMAKE_BUILD_TYPE": "Debug" }
        }
    ],
    "buildPresets": [
        { "name": "g0b1re", "configurePreset": "g0b1re" },
        { "name": "g0b1re-debug", "configurePreset": "g0b1re-debug" }
    ]
}
```

```bash
cmake --preset g0b1re
cmake --build --preset g0b1re
```

Expected: the same configure and build as step 1, into
`build/g0b1re`, and the same image. A board type or a family variable
is one more entry in `cacheVariables`.

## Step 5: what the build directory holds

After a build, the build directory holds, for an image named `my_app`:

- `my_app.elf`, the linked image with its debug information - the file
  a debugger loads;
- `my_app.hex`, and on every family but the AVR `my_app.bin` too, the
  image a programmer writes;
- `firmware-my_app.map`, the linker's map: where every section and
  symbol landed, and the memory regions of the part's linker script;
- `my_app.lst`, the disassembly interleaved with the source;
- `compile_commands.json`, the compile database a language server such
  as clangd reads.

## Step 6: flash it with the -upload target

Every image `brio_add_app()` makes has an `<name>-upload` target, the
same command the family's in-tree applications use. For this board it
programs through OpenOCD and the Nucleo's ST-LINK, as step 3 of the
[command-line route](command-line.md) describes:

```bash
cmake --build build/stm32g0b1re --target my_app-upload
```

The probe is chosen at the configure by three cache variables, the
same in every family (`cmake/probe.cmake` says what each does):

- `BRIO_PROBE`, the probe's kind; empty means the family's own probe,
  the first of its row below.
- `BRIO_PROBE_SERIAL`, the probe's USB serial, which picks one probe
  when more than one of the kind is attached; empty means the only one.
- `BRIO_PROBE_PORT`, the serial port of a probe that is one: the
  SerialUPDI adapter (`serialupdi`, whose rate is `BRIO_PROBE_BAUD`,
  230400 when empty) and the Black Magic Probe (`bmp`).

| Family | Its own probe | Other kinds |
|---|---|---|
| AVR DA, DB, EA | `atmelice`: avrdude, the Atmel-ICE over UPDI | `pickit4`, `pickit5`, `snap`, `nedbg`, `serialupdi` |
| SAM C21 | `atmelice`: OpenOCD, the Atmel-ICE over SWD | `cmsis-dap`, `jlink`, `bmp` |
| STM32G0, STM32F4 | `stlink`: OpenOCD, the ST-LINK | `cmsis-dap`, `atmelice`, `jlink`, `bmp` |
| RP2040 | `debugprobe`: OpenOCD, the Raspberry Pi Debug Probe | `cmsis-dap`, `atmelice`, `bmp` |
| RP2350 | `debugprobe`: Raspberry Pi's OpenOCD, the Raspberry Pi Debug Probe | `cmsis-dap` |
| CH32V00x, CH32V203/CH32V303, CH32X035 | `wchlink`: WCH's OpenOCD, the WCH-Link | none |

[The probes' index](../probes/README.md), "The probe of an -upload
target", says what each kind is and what drives it. Every kind but a
family's own and the PICkit 4 is configured, not run, through an
`-upload` target: its command was read in `build.ninja`, and no board
was programmed with it.

A board on another probe names its kind at the configure, for example
an AVR128DB48 on a PICkit 4:

```bash
cmake -S . -B build/avr128db48 -G Ninja -DBRIO_MCU=avr128db48 -DBRIO_PROBE=pickit4
```

or, when the board is always on that probe, in the `CMakeLists.txt`
before the include of `cmake/brio.cmake`:

```cmake
set(BRIO_PROBE pickit4 CACHE STRING "")
```

A probe's serial belongs to the desk, not to the project: give
`-DBRIO_PROBE_SERIAL=<serial>` on the configure line, or put it in a
`CMakeUserPresets.json` beside the project's `CMakePresets.json` (the
file CMake reserves for one developer's own presets, which is not
committed), never in the `CMakeLists.txt` or the shared presets.

OpenOCD's path is the family's `<FAMILY>_OPENOCD` variable
(`STM32G0_OPENOCD` here), as for the in-tree applications.

**From an application outside the tree this step was run on one
board**, an AVR128DB48 on a PICkit 4: `-DBRIO_PROBE=pickit4`, with and
without `-DBRIO_PROBE_SERIAL`, wrote the image and avrdude read it
back. On the STM32G0B1RE of this page the target and its command are
the in-tree ones (the family's recipe is the one place both are
written), and the command generated with
`-DBRIO_PROBE_SERIAL=<serial>` was read in `build.ninja`; that board
was not programmed this way.

## What brio check does for your application

Nothing. `brio check` compiles brio's own fixtures and builds brio's
own applications; an application outside the tree is built only when
its owner builds it, so a change to brio that breaks it shows at the
owner's next build. brio's other tools do not know it either: `brio
apps`, `brio fit` and `brio flash <board> <app>` read the families'
in-tree applications.

What the check does guard is the claim this page rests on: that the
recipe outside the tree is the recipe inside. Its last step, the
external guard, builds one application of each family - `blink`, or
`console` where the family has no `blink` - from a project written as
an application outside the tree (`test/external_app/`), for every
release preset on which that application is built in the tree, and
fails if the image differs by one byte from the in-tree image of the
same preset (`cli/check.py`).
