# Build, flash and debug blink from the command line

The terminal route for the [first program](first-program.md) on the
ST Nucleo-G0B1RE. Each step says what to type, what to expect and what
to check first when it fails. The same steps apply to the other STM32G0
Nucleos with their preset names; other families have the same shape
with their own project directory, preset and probe
([the target table](../../README.md#targets) names them). The
graphical route with the same steps is [Working in VS Code](vscode.md).

## What has been verified

Every step describes the mechanism the repository's build and bench
tools use (`stm32g0/CMakeLists.txt`, `cli/bench/flash.py`,
[docs/stm32g0/README.md](../stm32g0/README.md)). What has been
performed, and where:

| Step | Status |
|---|---|
| 1. Configure | configured: the configure every build of this project runs; not repeated from a fresh checkout for this page |
| 2. Build | configured: as step 1; the size figures quoted are from one debug build of this revision |
| 3. Flash with the `-upload` target | configured: the OpenOCD sequence `brio flash` runs on this board; the target itself not run for this page |
| The bench tool | verified on this board with the desk's own manifest entry: how every STM32G0 suite reaches the Nucleo-G0B1RE; writing a first manifest entry not walked through |
| 4. Observe | configured: the nine-second cycle is read from the source, not from a run of this revision |
| 5. Debug from a terminal | verified on the Nucleo-G071RB with the same OpenOCD, target script and gdb (load, reset-halt, a breakpoint at `main`, source lines stepped); not performed on the G0B1RE |
| 6. Host tests | verified on the host, no board |

"Expected" paragraphs below describe what the tools print by design,
not a transcript.

## What you need

- The board: an ST Nucleo-G0B1RE, connected to the host by its USB
  connector. The on-board ST-LINK is both the programmer and a serial
  port; `blink` uses only the programmer.
- The cross compiler: arm-none-eabi-gcc, version 16.2 as the project
  is built with. The toolchain file `stm32g0/cmake/toolchain-arm.cmake`
  expects it under `/sw/arm-none-eabi`, the root that contains `bin/`;
  a toolchain elsewhere is named when configuring, by adding the cache
  variable to the preset command:
  `cmake --preset stm32g0b1re-debug -DARM_TOOLCHAIN_DIR=/opt/arm` for a
  compiler at `/opt/arm/bin/arm-none-eabi-g++`. The debugging step
  below spells the `/sw` paths; substitute yours.
  An older compiler may refuse the C++23 the code uses. The toolchains
  are self-built ([docs/stm32g0/README.md](../stm32g0/README.md),
  "Toolchain"); no package manager installs this exact one.
- CMake 3.25 or newer, and Ninja, the generator every preset names.
  Both come from the distribution's packages or from their own sites;
  the project does not fix their location.
- OpenOCD 0.12.0 with ST-LINK support, expected at
  `/sw/openocd/bin/openocd`; another path is given the same way,
  `-DSTM32G0_OPENOCD=/path/to/openocd`. Flashing and debugging both go
  through it.
- On Linux, permission to open the ST-LINK's USB device without root.
  OpenOCD's source tree ships the rules file `contrib/60-openocd.rules`;
  copied to `/etc/udev/rules.d/` and loaded (`sudo udevadm control
  --reload-rules && sudo udevadm trigger`, or a replug of the board), it
  gives the `plugdev` group access to every probe OpenOCD knows, the
  ST-LINK included.
- For debugging from a terminal: arm-none-eabi-gdb from the same
  toolchain.

Nothing has to be wired. The LED is on the board.

## Step 1: configure the STM32G0 project

The repository root is not a CMake project. Each family is its own
project directory; `stm32g0/` is the one for this board. Configure it
with a preset: the preset names the part and whether the build is
release (`-Os`) or debug (`-Og -g3 -fno-inline`, the one a debugger
wants).

```bash
cd stm32g0
cmake --preset stm32g0b1re-debug
```

Expected: CMake reports the compiler it found under the toolchain
directory and writes the build tree to `build-cmake/stm32g0b1re-debug`
at the repository root (every project's build trees live under
`build-cmake/`, one per preset). The configure also discovers every
application under `stm32g0/src/apps/` from its `// build:` header line
and creates one target per application for this board.

If it fails: a missing compiler is reported by name with the path
tried; fix `ARM_TOOLCHAIN_DIR`. A preset not found means the command
was run from the wrong directory; presets resolve against the project
directory, so `cd stm32g0` first (or run the whole command in a
subshell, `(cd stm32g0 && cmake --preset stm32g0b1re-debug)`, as the
README's examples do).

## Step 2: build blink

```bash
cmake --build --preset stm32g0b1re-debug --target blink
```

Expected: the build ends with the section table `arm-none-eabi-size`
prints for `blink.elf`, and the build directory holds `blink.elf`,
`blink.bin`, `blink.hex`, `firmware-blink.map` and `blink.lst` (the
source-interleaved disassembly, worth opening once to see what the
kernel compiled to).

The table has one line per section: `.text` is the code and the
constants, in flash; `.data` the variables with a non-zero initial
value, stored in flash and copied to RAM at start; `.bss` the
zero-initialised variables, RAM only. One debug build of this revision
measured, in bytes:

| section | size |
|---|---|
| `.text` | 6244 |
| `.data` | 48 |
| `.bss` | 76 |

Another compiler version or a changed source gives other figures, and
the release build is smaller.

If it fails: a compile error names a file and a line as usual. An error
inside a brio header after a change to the application usually comes
from a constraint the kernel checks, and its message names the rule
(for example a type in the list given to `brio::Tenuto` that does not
meet the active-object contract, or a posted event the receiver does
not accept).

## Step 3: flash it

Every application has an `-upload` target that runs OpenOCD with the
ST-LINK interface and the STM32G0 target script, programs and verifies
the ELF, resets the board, and clears the core's halting-debug enable
so that a later `panic()` faults instead of halting silently on its
breakpoint instruction:

```bash
cmake --build --preset stm32g0b1re-debug --target blink-upload
```

Expected: OpenOCD prints the probe's firmware version, the part it
found, the programming and verification of the image, and exits. The
LED starts changing state at once: every half second for three seconds,
then faster. The three rates cycle with a period of nine seconds.

With more than one ST-LINK attached, OpenOCD takes the first it
enumerates; name the right one with
`-DSTM32G0_PROBE_SERIAL=<serial>` at configure time (the `-upload`
target then passes `adapter serial`).

What this leaves behind: OpenOCD's target script sets a bit in the
chip's debug configuration register, DBG_STOP, at every examination of
the chip, and the reset that ends the programming is followed by one,
so the bit stays set until a power-on. It changes nothing for `blink`,
which never enters a Stop mode; what it does to a program that measures
Stop, and how `brio flash` clears it, is in
[docs/stm32g0/README.md, "Upload"](../stm32g0/README.md).

If it fails. Two messages look alike and mean different things. "open
failed" (or a libusb permission error) means OpenOCD could not reach the
probe: check the USB permission, then the cable. "no device connected"
(a status the probe itself reports) means the probe answered but sees
no target: on a Nucleo this is the ST-LINK half of the board gone
silent, and the first thing to do is to replug the board as a whole,
before any other diagnosis
([docs/probes/st-link.md](../probes/st-link.md)). A verification error
after a successful program is rare and points at a flash protection
option; the board page and the target's flash document say what the
part's option bytes mean.

## The bench tool, as an alternative

`bin/brio` is the repository's one command for boards and builds. With
`bin/` on the PATH and a board described in the bench manifest
(`cli/bench/bench_boards.py` holds an example entry per board type;
the real manifest is private to each desk), `brio flash <position>
blink` configures, builds and flashes in one step through the same
OpenOCD sequence, and ends the flash with a halt, a clear of DBG_STOP
and a resume, so the board leaves the bench as a power-on would leave it
([docs/boards/README.md](../boards/README.md), "How a board joins the
bench"). The `flash` verb is the action name after `brio`; `brio
--help` lists the others. The first-time route above does not need it;
a measurement of the Stop modes does.

## Step 4: observe

For `blink` the observation is the LED, LD4 next to the user button:
half a second per toggle for three seconds, then a quarter, then a
tenth, then half a second again. There is no serial output to watch.

A debugger adds a second way to observe, without stopping the program
for long: the queue overflow counters and `Supervisor::index` are
static variables readable by name (step 5).

## Step 5: debug from a terminal

Two processes: OpenOCD as the GDB server, gdb as the client. In one
terminal:

```bash
/sw/openocd/bin/openocd -f interface/stlink.cfg -f target/stm32g0x.cfg
```

Expected: OpenOCD stays running, listening for gdb on port 3333. In a
second terminal, from the repository root (`cd ..` from `stm32g0/`; the
ELF's path below is relative to the root):

```bash
/sw/arm-none-eabi/bin/arm-none-eabi-gdb build-cmake/stm32g0b1re-debug/blink.elf
(gdb) target extended-remote :3333
(gdb) monitor reset halt
(gdb) load
(gdb) break main
(gdb) continue
```

Expected: the program is loaded, runs from reset and stops at `main`.
From there, `break Blinker::running` and `continue` stop at the next
event the Blinker handles, within half a second; `print
Supervisor::index` prints the current period's index, 0 to 2; `monitor
reset run` and `quit` leave the board running.

What the queue looks like. `print Blinker::queue` prints the queue
object field by field: its slots (each a variant, printed with the
implementation's own members), its cursors and the counters. After
events have passed, the cursors and the slot contents are not zero,
and that is normal: the queue is a ring, and old slots keep their last
event until they are reused. The one field that must stay zero in a
healthy `blink` is the overflow counter, `overflows_`;
`print Blinker::queue.overflows_` reads it alone. Read the stored field
while the core is halted rather than calling the accessor
`overflows()` from gdb, which would run code on the target. A count
that is not zero means a producer outran this queue
([capacities and overload](first-program.md#capacities-overload-and-how-to-see-a-failure)).

Halt before reading memory. A memory read through the ST-LINK while
the core sleeps in its wait-for-interrupt instruction can return wrong
values, and `blink` sleeps that way between events, so a halted core
(a breakpoint hit, or `monitor halt`) is the only reliable moment to
read ([docs/stm32g0/README.md, "Upload"](../stm32g0/README.md)). The
DBG_STOP bit of step 3 is a separate matter: a debug session sets it
too, and it changes only the deeper Stop modes, which `blink` does not
use.

## Step 6: the host tests, without a board

The kernel and the services are tested on the host, with no hardware,
by the `test/` project. From the repository root:

```bash
cd test
cmake --preset host
cmake --build --preset host
ctest --preset host
```

The three commands are configure, build and run; the test preset runs
the suites and does not build them, so `ctest` alone on a fresh
checkout reports every suite as not run.

Expected: every suite passes (`test_kernel`, `test_fsm`,
`test_event_queue`, `test_time_event` and the rest; `ctest --preset
host -R test_fsm` runs one). The host compiler must speak gnu++23; the
preset names `/sw/gcc/bin/g++`, and another compiler is given the same
way as the cross toolchain,
`cmake --preset host -DCMAKE_CXX_COMPILER=/path/to/g++`
([docs/host/README.md](../host/README.md)). These tests exercise the
queue, the scheduler, the state machine and the time events the first
program uses, with a simulated clock the test advances by hand; they
are the place to read how the kernel behaves in cases a board makes hard
to provoke (a full queue, a late timer).
