# Boards

The boards brio is tested on, one page each, for a programmer who has
one of them on the desk or is choosing one. A board page says what the
board is and which part it carries, what it offers a brio program (the
clocks, the LED and the button, the console bridge, the probe, the
wires a two-board test needs), how it is connected and flashed, the
traps its silicon or its layout sets for a program, and what has been
tested on it. This page is the index of those pages and the explanation
of the one piece of vocabulary they all use: a board's type, the key
the bench tool `bin/brio` and the build know a board by.

Nothing here is a desk. Which board is plugged into which socket, with
which probe, is the bench manifest, a file each desk keeps for itself
(below, "How a board joins the bench"). A page names the stratum a
board belongs to: a stratum is a directory of `brio/` holding one
architectural layer, and a target's stratum is its family's driver set
([../README.md](../README.md)).

| Board | Part | Type | Page |
|-------|------|------|------|
| a self-built AVR128DB48 board | AVR128DB48 | `db48` | [avr128db48.md](avr128db48.md) |
| a self-built 48-pin board, REV 1.2 of a DA/DB/EA design, fitted with an AVR128DA48 | AVR128DA48 | `da48` | [avr128da48.md](avr128da48.md) |
| the same REV 1.2 board fitted with an AVR64EA48 | AVR64EA48 | `ea48` | [avr64ea48.md](avr64ea48.md) |
| a self-built ATSAMC21J18A board | ATSAMC21J18A | `c21j` | [samc21j.md](samc21j.md) |
| ST Nucleo-G0B1RE | STM32G0B1RE | `g0b1re` | [nucleo-g0b1re.md](nucleo-g0b1re.md) |
| ST Nucleo-G071RB | STM32G071RB | `g071rb` | [nucleo-g071rb.md](nucleo-g071rb.md) |
| ST Nucleo-G031K8 | STM32G031K8 | `g031k8` | [nucleo-g031k8.md](nucleo-g031k8.md) |
| a CH32V006K8U6 module | CH32V006K8U6 | `v006k8` | [ch32v006k8.md](ch32v006k8.md) |
| WCH's CH32V003F4P6 evaluation board | CH32V003F4P6 | `v003f4` | [ch32v003f4.md](ch32v003f4.md) |
| a WeAct CH32V203C8T6 core board | CH32V203C8T6 | `v203c8` | [weact-ch32v203c8.md](weact-ch32v203c8.md) |
| WCH's CH32V303 evaluation board (CH32xx0xV-R0-1v0) | CH32V303VCT6 | `v303vc` | [ch32v303-evt.md](ch32v303-evt.md) |
| WCH's CH32X035 evaluation board, QFN20 edition | CH32X035F8U6 | `x035f8` | [ch32x035-evt-f8u6.md](ch32x035-evt-f8u6.md) |
| Raspberry Pi Pico | RP2040 | `pico` | [pico.md](pico.md) |
| a WeAct RP2040 board | RP2040 | `weact2040` | [weact-rp2040.md](weact-rp2040.md) |
| a WeAct RP2350B core board | RP2350 (QFN-80) | `weact2350b` for its Cortex-M33 pair, `weact2350b-rv` for its Hazard3 pair | [weact-rp2350b.md](weact-rp2350b.md) |
| ST STM32F429I-DISC1 | STM32F429ZI | `f429zi` | [stm32f429i-disc1.md](stm32f429i-disc1.md) |
| ST Nucleo-F446RE | STM32F446RE | `f446re` | [nucleo-f446re.md](nucleo-f446re.md) |
| a WeAct STM32F411CE black pill | STM32F411CE | `f411ce` | [blackpill-f411ce.md](blackpill-f411ce.md) |
| ST 32F469IDISCOVERY | STM32F469NIH6 | `f469ni` | [stm32f469i-discovery.md](stm32f469i-discovery.md) |

## How a board joins the bench

Three things have to be true before `brio flash <board> <app>` can put
a program into a board, and they are kept apart on purpose: the build
must know how to compile the program for that kind of board, the desk
must know which physical board is which, and the tool must put the two
together. A verb is the action word after `brio` in a command.

**The build knows board types, never physical boards.** A board type
names a chip package and everything that follows from it: which of the
sibling CMake projects compiles for it, which release preset that
project uses, and how firmware gets in. Every application under a
project's `src/apps/` becomes one CMake target per board type it names
in its own header comment, discovered at configure time: an app that
builds for more than its project's default board says so with a line
such as `// build: boards = db28,db32,db48` or `// build: boards =
g0b1re,g071rb,g031k8`. A configure targets exactly one package (one
configure preset per package), so switching the preset switches which
apps have a target. There is never a target per physical board.

A test suite whose whole image does not fit the smallest part of its
family declares its groups of letters, `// build: groups = abg,cdf,e`
(a letter is one command key of a suite's console, the group of
verdicts it runs). On a board type its project lists as splitting, the
suite builds as one image per group, `<app>-1`, `<app>-2` and so on,
each carrying the letters its group names and registering those alone
(`util/testbench.hpp` does the selection), while on every other board
the suite is one image with every letter. The splitting board types
are the CH32V00x project's `v003f4`; the CH32V203 project's `v203c6`,
`v203f6`, `v203g6` and `v203k6`; the STM32G0's `g031k8`; the SAM C21's
`c21j`; every CH32X035 board type, all seven parts having the same
62 KB; and the AVR project's `ea48`, whose images are held to the
ATmega4809's 48 KB and 6 KB. A suite may also name one board type's own
groups, `// build: groups.v006k8 = rtmpdes,uw`, which split it on that
board type alone and replace the plain line there, on a splitting board
too. The rule and its reasons are in
[design/overview.md](../design/overview.md), "A suite's image fits the
family's smallest chip"; those six projects are the ones with the axis.
`brio check` links every app on every preset of its project, and
`brio fit` lists each image's flash and RAM against its part's, or
against the smaller tier its project holds it to.

**The desk knows physical boards: the bench manifest.** The manifest is
a plain Python dict naming each board on the desk by an entry name, its
type, the label the chip is expected to carry, its console and its
programmer. The file in the repository,
`cli/bench/bench_boards.py`, is an example with entries for ten of the
board types over four of the programmer kinds the tool drives (the
others appear in its comments); the
desk that is really there is `private/bench_boards.py`, which
`bin/brio` loads first when it exists and which is never published.

**The tool puts the two together.** `brio flash <board> <app>` builds
the app for the board's type and flashes it through the board's
programmer; `brio run <board> z` drives the board's console and judges
the suite's `ALL: N pass, M fail` line; `brio console <board>` prints
the device path and speed for a monitor of your own. On a board that
splits a suite, `brio flash` takes the group image's name
(`test_ch32_tim-2`) and refuses the bare name with the list of its
images, and `brio run` judges each image's own `ALL:` line.

## The board type decides the project and the flash mechanism

Each type belongs to one project and one way of writing firmware:

| Types | Project | Written by |
|-------|---------|------------|
| `db28`, `db32`, `db48`, `da48`, `ea48` | `avrxt/` | avrdude over UPDI, through an EDBG-class probe |
| `c21j` | `samc21/` | OpenOCD over SWD, through a CMSIS-DAP probe |
| `g0b1re`, `g071rb`, `g031k8` | `stm32g0/` | OpenOCD through the Nucleo's own ST-LINK |
| `v006k8`, `v003f4` | `ch32v00x/` | WCH's OpenOCD fork through a WCH-Link |
| `v203c8`, `v303vc` | `ch32vx03/` | WCH's OpenOCD fork through a WCH-Link |
| `x035f8` | `ch32x035/` | the same fork through a WCH-LinkE |
| `pico`, `picow`, `weact2040` | `rp2040/` | OpenOCD through a CMSIS-DAP probe, the Raspberry Pi Debug Probe |
| `weact2350b`, `weact2350b-rv` | `rp2350/` | Raspberry Pi's OpenOCD fork through the same probe, in two sessions |
| `f429zi`, `f446re`, `f411ce`, `f469ni` | `stm32f4/` | OpenOCD through an ST-LINK, the board's own or a standalone STLINK-V3 |

The table of types in the code is `BOARD_TYPES` in
`cli/bench/common.py`, and the probes and their mechanisms are
[../probes/README.md](../probes/README.md)'s.

**On one chip the type carries the processor architecture as well.**
The RP2350 runs a pair of Cortex-M33 cores or a pair of Hazard3 RISC-V
cores over one set of peripherals, and which pair runs is decided by
the image in the flash. One physical board therefore wears two types:
`weact2350b` builds it for Arm and `weact2350b-rv` for RISC-V, the same
hardware and the same probe, differing only in the preset the type
names. Both are written by the same verb, which does not depend on the
state the board is in: a rescue over the debug port alone, then
programming as core 0 of the Arm pair, then a reset that hands the chip
to whatever the new image asks for. So the board is recoverable from any
state a program can put it in, whichever architecture was running
before ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md);
the sequences are `cli/bench/flash.py`'s).

## Consoles and programmers

A console is for observing a board; firmware never goes in through it.
How a console is addressed depends on whether its USB bridge carries a
serial number. A bridge with none (a CH340) is addressed by
`/dev/serial/by-path`, which names the USB socket it is plugged into,
because two CH340s collide in `/dev/serial/by-id`. A bridge with a real
serial (a Nucleo's ST-LINK, a WCH-Link, the Raspberry Pi Debug Probe) is
addressed by `/dev/serial/by-id`, which is stable across sockets. macOS
has no `/dev/serial` tree: there a CH340 is `/dev/cu.usbserial-<location>`,
named after the USB location it is plugged into, which plays the role
of `by-path`, and a board used from both systems names its console per
system in the manifest (`{"linux": ..., "darwin": ...}`). `brio list`
reads the attached probes from sysfs on Linux and from the I/O Registry
on macOS.

A programmer is addressed by its own USB serial, and needs one only
when two of a kind are attached. The kinds the manifest knows,
`atmelice_updi`, `serialupdi`, `openocd_cmsisdap`, `openocd_stlink`,
`stlink_msd` and `wch_link`, are described on
[../probes/README.md](../probes/README.md).

## Identity in the chip

Two boards of one kind are indistinguishable by hardware, so each
carries something a test suite can print in its banner and a person can
compare against the manifest: an AVR board carries a label in its
USERROW, written once over UPDI
([../avrxt/userrow.md](../avrxt/userrow.md)); a SAM board is told by its
factory die serial; an STM32 by its 96-bit unique device ID.

## What a suite needs wired

Each test suite's own header comment is the map of what its letters
exercise and what must be wired for them. A single-board suite says
"nothing to wire"; a two-board suite names the peer firmware the other
board runs and the pins at both ends by their GPIO names. A peer board
carries the same brio build and the same bench tool; nothing
distinguishes the board under test from the instrument but the
application it runs.

## The apps

Every application under a project's `src/apps/` is one `main()` and one
target, documented in its own header comment; `brio apps` lists them
all with that comment's first line. Three kinds share the directory:
demos and bench tools, which are disposable; the peers (`spi_peer`,
`twi_peer`, `usart_peer`, `sleep_peer`), the far end of a two-board
suite; and the bench test suites named `test_<target>_<subject>`, the
reference tests of the drivers they cover, which must keep passing
through every restructuring of the driver under them. Breadboard
experiments that were assembled once live under `experiments/`, each
with its own README.
