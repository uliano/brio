# Boards

The boards brio is tested on, one page each - what a board is, what it
carries that a suite depends on (crystals, LED, button, console
bridge, probe), its TYPE in the bench manifest and the documents that
describe it. Nothing here is a desk: which board is plugged in where is
the user's own manifest, below.

| Board | Type | Page |
|-------|------|------|
| an AVR128DB48 board (self-built) | `db48` | [avr128db48.md](avr128db48.md) |
| an ATSAMC21J18A board (self-built) | `c21j` | [samc21j.md](samc21j.md) |
| ST Nucleo-G0B1RE | `g0b1re` | [nucleo-g0b1re.md](nucleo-g0b1re.md) |
| ST Nucleo-G071RB | `g071rb` | [nucleo-g071rb.md](nucleo-g071rb.md) |
| ST Nucleo-G031K8 | `g031k8` | [nucleo-g031k8.md](nucleo-g031k8.md) |
| a CH32V006K8U6 module | `v006k8` | [ch32v006k8.md](ch32v006k8.md) |
| WCH's CH32V003F4P6 evaluation board | `v003f4` | [ch32v003f4.md](ch32v003f4.md) |

## How a board joins the bench

Three concerns, deliberately kept apart:

1. **Build** - one CMake target per app x board TYPE, auto-discovered
   from each app's own header comment at configure time. An app that
   builds for more than its project's default board says so directly
   (`// build: boards = db28,db32,db48`; `// build: boards =
   g0b1re,g071rb,g031k8`); a configure targets exactly one package
   (one `configurePreset` per package), so switching preset switches
   the board. Never a target per physical board. A suite whose whole
   image does not fit a part of its family declares its GROUPS of
   letters (`// build: groups = abg,cdf,e`): on a board type the
   project lists as splitting (the CH32V00x's `v003f4`) the suite
   builds as one image per group - `<app>-1`, `<app>-2`, ... - each
   carrying the letters its group names and registering those alone
   (`util/testbench.hpp`'s selection), while on every other board the
   suite is one image with every letter (design/overview.md, "A
   suite's image fits the family's smallest chip").
2. **Identity** - the bench MANIFEST, `cli/bench/bench_boards.py`: a
   plain dict naming each board on the desk by a POSITION (a letter),
   its type, the label the chip is expected to carry, its console and
   its programmer. The file in the repository is an example, one entry
   per board type; the desk that is really there is
   `private/bench_boards.py`, which `bin/brio` loads first when it
   exists and which is never published.
3. **Orchestration** - `bin/brio`, which resolves 1 against 2:
   `brio flash A test_avr_pin` builds the app for A's type and flashes
   it through A's programmer, `brio run A z` drives A's console and
   judges the suite's `ALL: N pass, M fail` line, `brio console A`
   prints the device path and speed for a monitor of your own. On a
   board that splits a suite, `brio flash` takes the group image's
   name (`test_ch32_tim-2`) and refuses the bare name with the list of
   its images; `brio run` judges each image's own ALL: line.

**The board type carries the architecture**: `db28`/`db32`/`db48` are
AVR DA/DB parts built by the `avrdx/` project and written by avrdude
over UPDI; `c21j` a SAM C21 built by `samc21/` and written by OpenOCD
over SWD; `g0b1re`/`g071rb`/`g031k8` the STM32G0 Nucleos built by
`stm32g0/` and written by OpenOCD through the board's own ST-LINK. The
table of types is `BOARD_TYPES` in `cli/bench/common.py`.

**Consoles** are observability only - firmware never goes in through
them. A bridge with no USB serial (a CH340) is addressed by
`/dev/serial/by-path`, i.e. by the USB socket it is plugged into (two
CH340s collide in `by-id`); a bridge with a real serial (a Nucleo's
ST-LINK) by `/dev/serial/by-id`, stable across sockets. **Programmers**
are addressed by their own USB serial, and only need one when two of
a kind are attached. The kinds - `atmelice_updi`, `serialupdi`,
`openocd_cmsisdap`, `openocd_stlink`, `stlink_msd` - are
[../probes/README.md](../probes/README.md)'s.

**Identity in the chip.** Boards of one kind are indistinguishable by
hardware, so each carries something a suite can print in its banner
and a human can compare against the manifest: an AVR board a label in
its USERROW, written once over UPDI
([../avrdx/userrow.md](../avrdx/userrow.md)); a SAM board its factory
die serial; an STM32 its 96-bit unique device ID.

**What a suite needs wired** is in the suite's own header comment, the
map of what each of its letters exercises: a single-board suite says
"nothing to wire", a two-board one names the peer firmware and the pins
at both ends by their GPIO names. A peer board carries the same brio
build and the same bench tool; nothing distinguishes a DUT from an
instrument but the app it runs.

## The apps

Every app under a project's `src/apps/` is one `main()` and one
target, documented in its own header comment - `brio apps` lists them
all with that comment's first line. Three kinds share the directory:
demos and bench tools, disposable; the PEERS (`spi_peer`, `twi_peer`,
`usart_peer`, `sleep_peer`), the far end of a two-board suite; and the
bench test SUITES named `test_<target>_<subject>`, the reference tests
of the drivers they cover, which must keep passing through every
restructuring of the driver under them. Breadboard experiments that
were assembled once live under `experiments/`, each with its own
README.
