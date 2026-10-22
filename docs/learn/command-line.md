# Build, flash and debug blink from the command line

The terminal route for the [first program](first-program.md) on the
ST Nucleo-G0B1RE. Each step says what it needs, what to type, what to
expect and what to check first when it fails. The same steps apply to
the other STM32G0 Nucleos with their preset names; other families have
the same shape with their own project directory, preset and probe
([the target table](../../README.md#targets) names them).

Every step of this page describes the mechanism the repository's build
and bench tools use (`stm32g0/CMakeLists.txt`, `cli/bench/flash.py`,
[docs/stm32g0/README.md](../stm32g0/README.md)). The page itself was
written without a board at hand; the pull request that introduced it
lists the steps to perform once on the desk.

## What you need

- The board: an ST Nucleo-G0B1RE, connected to the host by its USB
  connector. The on-board ST-LINK is both the programmer and a serial
  port; `blink` uses only the programmer.
- The cross compiler: arm-none-eabi-gcc, version 16.2 as the project
  is built with. The toolchain file `stm32g0/cmake/toolchain-arm.cmake`
  expects it under `/sw/arm-none-eabi`; a toolchain elsewhere is named
  on the configure command line with
  `-DARM_TOOLCHAIN_DIR=/path/to/toolchain`. An older compiler may
  refuse the C++23 the code uses. The toolchains are self-built
  ([docs/stm32g0/README.md](../stm32g0/README.md), "Toolchain"); no
  package manager installs this exact one.
- CMake 3.25 or newer (the presets file is version 6) and Ninja, the
  generator every preset names.
- OpenOCD 0.12.0 with ST-LINK support, expected at
  `/sw/openocd/bin/openocd`; another path is set with
  `-DSTM32G0_OPENOCD=/path/to/openocd` at configure time. Flashing and
  debugging both go through it.
- On Linux, permission to open the ST-LINK's USB device without root
  (a udev rule for the ST-LINK's vendor ID, as the OpenOCD distribution
  ships).
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

Expected: the build ends with a size table for `blink.elf`, and the
build directory holds `blink.elf`, `blink.bin`, `blink.hex`,
`firmware-blink.map` and `blink.lst` (the source-interleaved
disassembly, worth opening once to see what the kernel compiled to).

If it fails: a compile error names a file and a line as usual. An error
inside a brio header after a change to the application usually comes
from a constraint the kernel checks, and its message names the rule
(for example an active object missing from the `Tenuto` list, or a
posted event the receiver does not accept).

## Step 3: flash it

Every application has an `-upload` target that runs OpenOCD with the
ST-LINK interface and the STM32G0 target script, programs and verifies
the ELF, resets the board and clears the debugger-enable bit so that a
`panic()` later faults instead of halting silently:

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

If it fails: "no device connected" or "open failed" from OpenOCD means
the probe is not reachable. Check the USB permission first, then the
cable, then replug the board as a whole: the ST-LINK half of a Nucleo
can go silent in a way only a replug clears
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
OpenOCD sequence, and ends the flash with the extra reset that clears
the DBG_STOP bit a debug session leaves behind
([docs/boards/README.md](../boards/README.md), "How a board joins the
bench"). The `flash` verb is the action name after `brio`; `brio
--help` lists the others. The first-time route above does not need it.

## Step 4: observe

For `blink` the observation is the LED, LD4 next to the user button.
There is no serial output to watch.

A debugger adds a second way to observe, without stopping the program
for long: the queue overflow counters and `Supervisor::index` are
static variables readable by name.

## Step 5: debug from a terminal

Two processes: OpenOCD as the GDB server, gdb as the client. In one
terminal:

```bash
/sw/openocd/bin/openocd -f interface/stlink.cfg -f target/stm32g0x.cfg
```

Expected: OpenOCD stays running, listening for gdb on port 3333. In a
second terminal, from the repository root:

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
event the Blinker handles; `print Blinker::queue` shows the queue's
cursors and counters; `print Supervisor::index` the current period's
index. `monitor reset run` and `quit` leave the board running.

Two facts of this probe and this family to know before reading memory
from gdb ([docs/stm32g0/README.md](../stm32g0/README.md), "Upload"):
a memory read through the ST-LINK while the core is in its WFI sleep
can return wrong values, so halt first; and OpenOCD's target script
sets the DBG_STOP bit, which keeps clocks running in the deeper Stop
modes until a power-on or a `brio flash` clears it. `blink` sleeps in
plain Sleep, where neither matters, but a program that uses the Stop
modes will measure differently right after a debug session.

Debugging from a terminal with these exact tools was performed on the
Nucleo-G071RB ([docs/stm32g0/README.md](../stm32g0/README.md),
"Debugging"); the Nucleo-G0B1RE uses the same probe, target script and
gdb, and has not been walked through this page's exact sequence yet.

## Step 6: the host tests, without a board

The kernel and the services are tested on the host, with no hardware,
by the `test/` project:

```bash
cd test
cmake --preset host
ctest --preset host
```

Expected: every suite passes (`test_kernel`, `test_fsm`,
`test_event_queue`, `test_time_event` and the rest; `ctest --preset
host -R test_fsm` runs one). The host compiler must
speak gnu++23; the preset names `/sw/gcc/bin/g++`
([docs/host/README.md](../host/README.md)). These tests exercise the
queue, the scheduler, the state machine and the time events the first
program uses, with a simulated clock the test advances by hand; they
are the place to read how the kernel behaves in cases a board makes hard
to provoke (a full queue, a late timer).
