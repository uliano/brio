# Build, flash and debug blink from the command line

The terminal route for the [first program](first-program.md) on the
ST Nucleo-G0B1RE. Each step says what it needs, what to type, what to
expect and what to check first when it fails. The same steps apply to
the other STM32G0 Nucleos with their preset names; other families have
the same shape with their own project directory, preset and probe
([the target table](../../README.md#targets) names them).

Every step of this page describes the mechanism the repository's build
and bench tools use (`stm32g0/CMakeLists.txt`, `cli/bench/flash.py`,
[docs/stm32g0/README.md](../stm32g0/README.md)), and each step ends
with its status: whether its sequence has been performed on the
Nucleo-G0B1RE as written here, performed on a sibling board, or is the
documented mechanism not yet walked through on this board. The page was
written without a board at hand, so the "Expected" paragraphs describe
what the tools print by design, not a transcript.

## What you need

- The board: an ST Nucleo-G0B1RE, connected to the host by its USB
  connector. The on-board ST-LINK is both the programmer and a serial
  port; `blink` uses only the programmer.
- The cross compiler: arm-none-eabi-gcc, version 16.2 as the project
  is built with. The toolchain file `stm32g0/cmake/toolchain-arm.cmake`
  expects it under `/sw/arm-none-eabi`; a toolchain elsewhere is named
  when configuring, by adding the cache variable to the preset command:
  `cmake --preset stm32g0b1re-debug -DARM_TOOLCHAIN_DIR=/path/to/toolchain`.
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

Status: the mechanism every build of this project uses, on this board
among others (every STM32G0 test suite was built this way); not walked
through as a first-time setup for this page.

## Step 2: build blink

```bash
cmake --build --preset stm32g0b1re-debug --target blink
```

Expected: the build ends with the section table `arm-none-eabi-size`
prints for `blink.elf` (one line per section, `.text`, `.data`, `.bss`
and the rest, with its size in bytes; a few kilobytes of `.text` and a
few hundred bytes of `.bss` for this program), and the build directory
holds `blink.elf`, `blink.bin`, `blink.hex`, `firmware-blink.map` and
`blink.lst` (the source-interleaved disassembly, worth opening once to
see what the kernel compiled to).

If it fails: a compile error names a file and a line as usual. An error
inside a brio header after a change to the application usually comes
from a constraint the kernel checks, and its message names the rule
(for example a type in the list given to `brio::Tenuto` that does not
meet the active-object contract, or a posted event the receiver does
not accept).

Status: as step 1.

## Step 3: flash it

Every application has an `-upload` target that runs OpenOCD with the
ST-LINK interface and the STM32G0 target script, programs and verifies
the ELF, resets the board, and clears the core's debug-enable bit
(C_DEBUGEN in the Cortex-M's DHCSR register) so that a `panic()` later
faults instead of halting silently on its breakpoint instruction:

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

What this leaves behind: OpenOCD's STM32G0 target script sets a second,
unrelated bit at every examination of the chip, DBG_STOP in DBGMCU_CR,
and it re-examines the chip after the reset that ends the programming,
so a board flashed this way keeps DBG_STOP set until a power-on. With
it set the clocks keep running inside the chip's Stop modes. `blink`
sleeps in plain Sleep, where the bit changes nothing; a program that
measures a Stop after this kind of flash does not measure Stop
([docs/stm32g0/README.md](../stm32g0/README.md), "Upload"). The bench
tool below clears the bit; this target does not.

If it fails. Two messages look alike and mean different things. "open
failed" (or a libusb permission error) means OpenOCD could not reach the
probe: check the USB permission, then the cable. "no device connected"
(the probe's status 5) means the probe answered but sees no target: on
a Nucleo this is the ST-LINK half of the board gone silent, and the
first thing to do is to replug the board as a whole, before any other
diagnosis ([docs/probes/st-link.md](../probes/st-link.md)). A
verification error after a successful program is rare and points at a
flash protection option; the board page and the target's flash document
say what the part's option bytes mean.

Status: the mechanism `brio flash` and the bench use on this board; the
exact command above not walked through for this page.

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

For `blink` the observation is the LED, LD4 next to the user button.
There is no serial output to watch.

A debugger adds a second way to observe, without stopping the program
for long: the queue overflow counters and `Supervisor::index` are
static variables readable by name.

Status: the LED's behaviour is the program's as written; it has been
seen on the bench boards this application was run on, not re-observed
for this page.

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
event the Blinker handles; `print Blinker::queue` prints the queue
object field by field (its slots, its three one-byte cursors and the
`overflows_` counter, all zero in a healthy `blink`); `print
Supervisor::index` prints the current period's index, 0 to 2. `monitor
reset run` and `quit` leave the board running.

Two facts of this probe and this family to know before reading memory
from gdb ([docs/stm32g0/README.md](../stm32g0/README.md), "Upload"):
a memory read through the ST-LINK while the core is in its WFI sleep
can return wrong values, so halt first; and OpenOCD's target script
sets the DBG_STOP bit (step 3), which a debug session sets again. `blink`
sleeps in plain Sleep, where neither matters, but a program that uses
the Stop modes will measure differently right after a debug session or
a flash, until a power-on or a `brio flash`.

Status: debugging from a terminal with these exact tools (OpenOCD, this
target script, this gdb: load, reset-halt, a breakpoint at `main`,
source lines stepped) was performed on the Nucleo-G071RB
([docs/stm32g0/README.md](../stm32g0/README.md), "Debugging"). The
Nucleo-G0B1RE uses the same probe, script and gdb and has not been
walked through this page's sequence; the gdb commands beyond `break
main` describe what gdb does with the symbols the debug build carries,
not a transcript.

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
speak gnu++23; the preset names `/sw/gcc/bin/g++`, and another compiler
is given the same way as the cross toolchain,
`cmake --preset host -DCMAKE_CXX_COMPILER=/path/to/g++`
([docs/host/README.md](../host/README.md)). These tests exercise the
queue, the scheduler, the state machine and the time events the first
program uses, with a simulated clock the test advances by hand; they
are the place to read how the kernel behaves in cases a board makes hard
to provoke (a full queue, a late timer).

Status: the suites run on every change to the kernel and the services;
nothing here depends on a board.
