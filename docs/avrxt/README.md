# Target: AVR DA, DB and EA (`avrxt/`)

The operational page of brio's AVR target, for a programmer who has one
of the three AVR boards on the desk and wants to build, flash and debug
a brio program on it, and then find the document of the peripheral the
program needs. It says what the stratum is, which boards and parts it
is proven on, how the build, the upload, the debugger and the console
work and where each has a trap, and it ends with the map of the
stratum's own documents: one line per peripheral driver, which is where
the state of the driver work is read.

A stratum is a directory of `brio/` holding one architectural layer;
`brio/avrxt/` holds everything in brio that knows `avr/io.h`: the clock
and its init, `Pin`, `Uart`, `Spi`, `Twi`, `BasicTicker`, and
`AvrPlatform`, the kernel's `Platform` concept implemented with AVR
intrinsics. The stratum is named for the AVRxt core the AVR DA, DB and
EA families share, and it claims the AVR128DA and AVR128DB in their four
packages and the AVR64EA28, 32 and 48. The differences inside the three
families are read off the device header in one place,
`avrxt/device_tables.hpp`, which the stratum's documents call the
reserve: each feature is probed from the symbol that is it
(`CLKCTRL_XOSCHFCTRLA` exists only on the DB; `PORTB`, `PORTE` and
`PORTG` depend on the package), the die from the compiler's device
macro, the errata keyed on the die, so the same headers build for every
claimed part and a part the stratum does not claim is a compile error.
The design side (why the ticker is declaredly AVR, the ISR binding
pattern) is in [../design/](../design/overview.md), and the kernel's
own vocabulary (an active object, a time event, a monostate: a class
whose state is static, with no instances) is the learning route's
([../learn/introduction.md](../learn/introduction.md)). The maintained
example `blink` builds for the AVR128DB48 board. A test suite's console
takes one command key per group of verdicts, which the pages call a
letter; `z` runs a suite's single-board set and `y` the set that needs
a partner board.

**On the AVR EA** every chapter the part carries is written from
DS40002443A and DS80001048D, compiles on the AVR64EA28, 32 and 48, and is
on the bench on the AVR64EA48 board: the clock (the fuse's OSCHF rate as
a claim checked at boot, MCLKTIMEBASE owned by the clock), the pins
(every pin fully asynchronous), the event system's pin and PIT slots,
the USART, SPI and TWI with their route tables per PORTMUX layout, the
TCA, the TCB, the CCL (AC1 on input 2), the RTC, reset, sleep and the
platform, NVMCTRL's EEPROM engine of NVM version 3 (its flash and User
Row self-programming declined), and the differential ADC with its PGA,
an engine of its own behind the ADC's surface, with the DAC, the VREF
and the comparators against the board's analog net. Every suite image of
the board runs green there but for the letters that time on the board's
32.768 kHz crystal, which does not start (its page); what each chapter
leaves open is its document's two lists, and the AVR64EA28 and 32 are
compiled, not run.

## The boards

Three self-built boards, one per part, each with a page of its own
under [../boards/](../boards/README.md) with its connectors and its
probe. All three are programmed and debugged with an Atmel-ICE over
UPDI, at firmware 1.46 or later for the EA: 1.41 acknowledges every
write over UPDI to an AVR64EA (fuses, flash, a chip erase) and loses it
([../probes/atmel-ice.md](../probes/atmel-ice.md)).

| Board | Part | Memory | Clock | Console | LED |
|---|---|---|---|---|---|
| [an AVR128DB48 board](../boards/avr128db48.md), the bench board | AVR128DB48, 48 pins | 128 KB flash, 16 KB SRAM | a 24 MHz crystal on PA0/PA1, no 32 kHz crystal | USART2 ALT1, PF4/PF5, a CH340 at 460800 | PF2 through a solder jumper, bridged on this board |
| [an AVR128DA48 board](../boards/avr128da48.md) | AVR128DA48, the same memories, no crystal oscillator and no MVIO | 128 KB flash, 16 KB SRAM | a 24 MHz external clock into PA0, a 32.768 kHz crystal on PF0/PF1 | the same pads, an isolated CH340B | PF2, a button on PF3 |
| [an AVR64EA48 board](../boards/avr64ea48.md) | AVR64EA48: OSCHF at the rate its OSCCFG fuse chooses, no PLL, no TCD, no OPAMP, NVM version 3, a differential ADC | 64 KB flash, 6 KB SRAM, 512 B EEPROM | a 20 MHz crystal on PA0/PA1, a 32.768 kHz crystal that does not start | the same pads, an isolated CH340B | PF2, a button on PF3 |

The bench board's crystal is the clock type every app of this project
states: `Clock<ClockSource::crystal, 24'000'000>`, PA0 and PA1 therefore
not GPIOs. It carries no 32.768 kHz crystal: the Ticker runs on the
internal OSC32K, and XOSC32K (PF0/PF1) is never enabled there ("Clock,
delay and timebase" below).

## Toolchain

Self-built avr-gcc 16.2 with avr-libc, avr-gdb 17.2 and avrdude 8.1
(built by `/sw/src/build-avr.sh`), named by absolute path in
`cmake/toolchain-avr.cmake` (`AVR_TOOLCHAIN_DIR`, default `/sw/avr`, a
symlink to `/sw/avr-16.2`): no package manifest or download step, unlike
a build-tool-managed toolchain package.

The toolchain ships the freestanding libstdc++ (type_traits, concepts,
bit, span, optional, expected, variant; no chrono, charconv or
iostream), and brio uses it instead of hand-rolled traits.

`-std=gnu++23` is set once, explicitly, in `CMakeLists.txt`'s
`avr_add_app()`; nothing else in this build adds a competing `-std`
flag, so there is no "last one wins" hazard to guard against.

## Build

`avrxt/` is its own CMake project, a sibling and peer of the other
cross projects and of `test/`; the repository root is not a CMake
project, so every CMake command runs from `avrxt/`:

```bash
(cd avrxt && cmake --preset avr128db48-release)                        # configure (once, or after adding an app)
(cd avrxt && cmake --build --preset avr128db48-release --target <app>)
(cd avrxt && cmake --build --preset avr128db48-release --target <app>-upload)   # flash via the Atmel-ICE
(cd avrxt && cmake --build --preset avr128db48-debug --target <app>)   # the debug build, then F5
brio check avrxt [name]                                           # the family compile check, no hardware
brio run <board> z                                                # a suite's single-board set, judged
```

`<app>` is a file of `src/apps/` by its name: `blink` for the
maintained example.

- **One configure targets one package.** `cmake/avr-mcus.cmake` is the
  table from board type to avr-gcc mcu name (the AVR128DA/DB with 128K
  of flash and 16K of RAM, the AVR64EA with 64K and 6K); `db48` is the
  default board when an app names none, the others are selected by the
  app's `boards` line below. The AVR64EA is `avrxmega2`: no RAMPZ, and
  FLMAP selects the 32 KB flash section the data space shows (`.rodata`
  lives at 0x8000 while the code is shorter than 32 KB). No `-DF_CPU` is
  ever produced: the clock rate is `brio::Clock<...>::hz` ("Clock, delay
  and timebase" below).
- **The presets.** `CMakePresets.json` has one configure and build
  preset pair per bench package and build type: the AVR128DB's three
  packages, the AVR128DA48 and the AVR64EA48 (`avr128db48-release`,
  `avr128db48-debug`, `avr128db28-release`, `avr128db32-release`,
  `avr128da48-release`, `avr64ea48-release` and their `-debug` twins;
  CMake Tools shows them as `AVR128DB48 (release, -Os)` and `AVR128DB48
  (debug, -Og -g3, PyAvrOCD)`). Each points `AVR_MCU` and the toolchain
  file at that package. The Atmel-ICE upload is a per-app `<app>-upload`
  target; the debug wiring is `.vscode/launch.json` ("Debugging" below).
- **App discovery.** Every `src/apps/*.cpp` with a `main()` becomes an
  executable target, discovered at configure time (`file(GLOB ...
  CONFIGURE_DEPENDS ...)`: a new or removed app is picked up on the next
  configure, with no generation step); the glob also covers
  `../experiments/*/avrxt/*.cpp`, a top-level experiment directory
  holding both architectures' app halves plus their shared protocol
  header and documenting itself in its own README (the same `// build:`
  grammar, names unique within the architecture). What an app can add to
  its own header comment is "Per-app build options" below.
- **The flags.** `avr_add_app()` sets per-language AVR flags, aware of
  the build type: `-Os -g` on the Release config and `-Og -g3 -ggdb3
  -fno-inline` on the Debug one (generator expressions; each configure
  preset picks one `CMAKE_BUILD_TYPE`, so the two profiles never mix),
  `-std=gnu++23` plus `-fno-exceptions -fno-rtti -fno-threadsafe-statics
  -fno-use-cxa-atexit`, and the `-mmcu` macro delta made explicit
  (`avr_predefines()` asks avr-gcc for its predefines with and without
  `-mmcu` and appends the `__AVR*` difference as `-D`s, the same values
  the real compile already implies). On the link the C library is
  refused: `cmake/avr-refused-libc.rsp` wraps avr-libc's heap and stdio
  with no wrapper defined ([../design/runtime.md](../design/runtime.md)).
  LTO is never added in the first place: it makes the DWARF problem
  below worse, buys nothing at this firmware size and yields an
  unreadable `.lst`. **Never add `-mrelax`**: PyAvrOCD refuses ELF files
  built with it (distorted line-number info).
- **The post-build step** writes the source-interleaved disassembly
  `<app>.lst`, the linker map `firmware-<app>.map` and `<app>.hex` into
  the target's own build directory, `build-cmake/<preset>/`.
- **`src/glue/ivsel_boot.cpp` is compiled into every image** (every
  `avr_add_app()` call lists it beside the app's own source). It holds
  build invariants that must hold before any app code runs and that no
  app may be trusted to remember. Today that is one file: a
  four-instruction `.init3` fragment that sets `CPUINT.CTRLA.IVSEL` under
  CCP, so the interrupt vector table is looked for at address 0. Without
  it, any image on a chip whose `BOOTSIZE` fuse is not 0 jumps into
  erased flash on its first interrupt: a reset loop, not a crash.
  Setting it is correct under every geometry, because the BOOT section
  always starts at 0; the details, and the run-time check that pairs
  with it, are in [nvm.md](nvm.md).
- **The board type reaches every app.** `avr_add_app()` defines
  `BRIO_BOARD_<type>` (the configure's board type, `AVR_BOARD`), and
  `src/apps/avr_board.hpp`, the board file's first draft, maps it to
  what the suites read of the assembly: `BoardClock` (the HF source, its
  rate and the OSCHF rate behind it), the 32 kHz crystal, what hangs on
  PA7 (CLKOUT allowed or not), whether a crystal pin may be driven,
  jumpered pin pairs, the console's USART and route. A board type the
  table does not know is a compile error; the packages with no board
  (db28, db32) have no row, their one app reading no board fact.
- **FLMAPLOCK is set in every image, by default.** gcc 16 places
  `.rodata` in a 32 KB flash section reached through the data-space
  window and emits a write of `NVMCTRL.CTRLB.FLMAP` in `.init`; the
  linker script ORs the lock bit into that write when the symbol
  `__flmap_lock` is non-zero, and `avr_add_app()` appends
  `-Wl,--defsym,__flmap_lock=<value>` (default 1) to every AVR link.
  brio's flash verbs never use the window (ELPM/SPM with a 24-bit
  address only), so the window is a mode nothing uses, and a mode
  nothing uses can only change by accident. On the AVR64EA the crt's
  write reads (`__do_flmap_init`, in `.init3` right after
  `brio_vectors_in_boot`): it loads NVMCTRL.CTRLB (reset 0x30: FLMAP 3,
  which on the 64 KB part aliases section 1, DS40002443A 11.5.2), keeps
  every bit but FLMAP, writes FLMAP = 1 (the upper 32 KB, where the
  linker put `.rodata`) ORed with FLMAPLOCK under CCP (`lds; andi 0xCF;
  ori 0x90; out CCP; sts`, read in every EA listing), so the map is the
  linker's and locked till the next reset. An app that must exercise the
  field opts out with `// build: flmap_lock = 0` in its header.
- **Fuses are provisioning, not build output.** The CPU can read them
  and nothing more; only the programmer writes them, so they are a
  property of the chip on the desk and live behind `brio fuses <board>
  [name=value ...]` ([../boards/avr128db48.md](../boards/avr128db48.md)
  has the standing geometry). The one that matters to the build is
  `BOOTSIZE`: with its shipping default of 0 the whole flash is one BOOT
  section and no software can write any flash at all.
- **Two ISRs with one body are folded by gcc into a call.** avr-gcc's
  identical code folding (at -Os and -O2 alike, measured on 16.2) turns
  the second of two `ISR()` bindings with the same body into a handler
  that saves its registers, calls the first and restores them. The first
  ends in `reti`, which returns into the caller with the I flag already
  set, so the caller's epilogue runs with interrupts enabled, a window of
  some forty cycles in which another interrupt can nest, against the
  kernel's one-boundary promise ([../design/kernel.md](../design/kernel.md),
  "The model in one page"), and every such interrupt pays two prologues.
  No image of this project carries a folded vector today (every release
  image scanned: no `__vector_N` calls another), so two bindings of one
  body are written with `__attribute__((no_icf))` as the ISR's second
  argument, which is what the RISC-V strata carry in their handler
  attribute.

### Per-app build options: `// build:` header lines

There is no generated file to hand-edit: `CMakeLists.txt` scans every
`src/apps/<app>.cpp` itself at configure time for comment lines of the
shape

    // build: monitor_speed = 115200

Any key other than `boards` (below), `groups` (the group axis below)
and `flmap_lock` (FLMAPLOCK above) is collected as metadata and written
into `build-cmake/apps_avrxt.json` at every configure; `bin/brio` reads
it instead of a build-tool manifest, for instance to pick a console's
`monitor_speed` (a console fact, not a compiler flag: nothing in
`CMakeLists.txt` interprets it beyond passing it through). One line per
key, any key name: an unrecognized one becomes manifest metadata nothing
currently reads, and there is no validation to complain about it. No
regeneration step: adding, removing or editing such a line takes effect
on the next configure (CMake Tools reconfigures automatically on save;
`cmake --preset ...` by hand otherwise).

`// build: boards = db28,db32,db48` is reserved and consumed directly:
it declares the board types the app is built for (default, if absent:
`db48` only). A CMake configure targets exactly one package (`AVR_MCU`,
one value per configure preset), so the app becomes a target only when
the currently configured package is in its list, and switching the
configure preset in the CMake Tools status bar switches which apps show
up in the Target dropdown. A target is a build, never a physical board:
the boards on the desk live in the bench manifest
([../boards/README.md](../boards/README.md)).

### The group axis: suites sized for the smallest AVR in view

A suite's image is held to the smallest chip of the family
([../design/overview.md](../design/overview.md), "A suite's image fits
the family's smallest chip"), and on the AVR that chip is one in view
rather than one the project builds: the ATmega4809, 48 KB of flash and
6 KB of SRAM, below the AVR64EA48's 64 KB and 6 KB. `CMakeLists.txt`
names it as the split tier (`AVR_SPLIT_TIER_FLASH`,
`AVR_SPLIT_TIER_RAM`) and lists the board types that stand in for it
(`AVR_SPLIT_BOARDS`: `ea48`, the smallest part the project builds; the
ATmega4809's own board type joins the list when it is born). On those
board types:

- a suite that declares its groups of letters (`// build: groups =
  123456789norsdt,cmliwekx`) builds as one image per group,
  `test_avr_timer-1`, `test_avr_timer-2`, each compiled with
  `BRIO_TEST_LETTERS` naming its group, which `util/testbench.hpp` reads
  to register those letters alone; the menu of such an image says which
  letters it carries. A group names lower-case letters, digits and, once
  its line names any, upper-case keys (`util/testbench.hpp`,
  `upper_mask_of()`): the build appends a '.' marker to every group of
  such a line, so a group naming none carries none;
- every image, grouped or not, is held to the tier after its link:
  `cmake/avr-tier-guard.cmake` sums its flash (`.text`, `.data`,
  `.rodata`) and its static RAM (`.data`, `.bss`, `.noinit`) and fails
  the build over 48 KB or 6 KB, though the AVR64EA48 would hold the
  image. The configure writes the tier to `tier.json` in the build
  directory, and `brio fit` measures that preset's margins against it:
  an image with under 5 % of the tier left is named, and is regrouped.

On the 128 KB DA and DB board types a suite stays one image with every
letter, unless it names that board type's own groups (`// build:
groups.da48 = ...`), which split it there alone. The roster
(`build-cmake/apps_avrxt.json`) carries `split_boards` and each app's
groups, so `brio flash <board> test_avr_timer-2` names an image on an EA
board and the bare name is refused there with the list; `brio run`
judges each image's own `ALL:` line, and a two-key suite (`z` alone, `y`
against the partner) answers each key with the letters of that set the
image carries, an `ALL: 0 pass, 0 fail` where it carries none, which is
no verdict at all and not a pass.

The suites grouped, each split along a line its letters already draw,
with every image's margin against the tier read by `brio fit avrxt`:

| suite | groups | what each group is |
|---|---|---|
| `test_avr_timer` | `123456789norsdt`, `cmliwekx` | the TCA and TCB tasks; the CCL, the AC, the event system and the EA's own letter |
| `test_avr_sleep` | `abcdefgnop`, `hijklmq` | the single-board set `z`; the two-board set `y` |
| `test_avr_serial` | `abcdefghiITF`, `jklmnopq`, `rstuvwxB` | the single-board set (F the EA's own); the two-board set in two halves, the by-name letters with the second |
| `test_avr_spi` | `abcdefghij`, `tklmnop`, `qrsuvw` | the single-board set but letter t; t with the first half of the two-board set; its second half |
| `test_avr_twi` | `abcdefghijuv`, `klmnopqrst` | the single-board set (u and v the EA's own); the two-board set |

Every other suite fits the tier whole; the benchmark skeleton (the app
that measures the drivers' costs, [../design/benchmark.md](../design/benchmark.md))
sizes its two memcpy buffers from the part's SRAM (2048 bytes each on
the EA, its letter m's largest run) and stays one image.

## Clock, delay and timebase

The target-independent model (one rate truth, static and dynamic
regimes, the rebase fan-out and its checks) is
[../design/clock.md](../design/clock.md); this section is the AVR
realization.

The main clock is a type, `brio::Clock<source, source_hz, div>`
(`avrxt/clock.hpp`), and `Clock::hz` is the one truth every driver of
this target derives its divisors from: `Uart::init(clock, baud)`,
`Twi::init(clock)`, `Spi::init(clock)` and `delay_us(clock, us)` all
take the app's clock tag. The sources are `internal` (OSCHF at
1/2/3/4/8/12/16/20/24 MHz), `crystal` (XOSCHF on PA0/PA1, the DB and the
EA) and `external` (EXTCLK on PA0); `div` is the main prescaler.
`Clock::init()`, the first line of `main()`, brings CLK_PER up and
returns true when running from the requested source, false when an
external source failed and OSCHF runs at the same rate (which is why an
external rate must be one OSCHF can produce: `hz` stays true either
way). `is_static` is true.

The runtime regime is `brio::DynamicClock<Boot, Users...>`: `Boot` is a
static `Clock<...>` naming the source, `Users` types satisfying the
`ClockUser` concept (a static `rebase(hz)`, checked where the list is
written). `set<hz>()` and `set(hz)` name the new rate in Hz (the
prescaler that produces it is the silicon's detail; an unreachable rate
is a compile error or a false), fan it out to every listed user
(`Uart`, `Twi` and `Spi` expose `rebase(hz)`; `Uart` drains its TX at
the old rate first) and then reprogram the main prescaler; `hz()` is a
value, `is_static` false, and `clock_hz(clock)` reads either kind.
`delay_us` takes the runtime path. The RTC/PIT timebase does not move.
The subscription is explicit and checked: a clocked driver initialized
with a DynamicClock that does not list it fails to compile at its
`init(clock)` (`clock_follows<Clock, Driver>()`), so forgetting a user
cannot leave it silently at the old rate. Do not switch while a bus
transaction is in flight (ask the bus active objects); RX bytes during
the switch may be garbled. `Uart::can_baud(hz, baud)` tells whether a
rate can still hit a baud (BAUD >= 64, which is CLK_PER >= 16 x baud).
Bench-verified: 24 -> 12 -> 2 MHz under a running 115200 console, 1 MHz
refused as expected.

**On the AVR EA** the internal oscillator's rate is the OSCCFG fuse's,
16 or 20 MHz with no FRQSEL, and the `Clock` type names it as a claim
the init checks with one read of the fuse ([clkctrl.md](clkctrl.md));
CLK_MAIN tops at 20 MHz (from 4.5 V), and the clock owns
CLKCTRL.MCLKTIMEBASE (cycles per microsecond, rounded up), rewritten at
every rate change. The EA board carries a 20 MHz quartz on PA0/PA1, and
its `BoardClock` is
`Clock<ClockSource::crystal, 20'000'000, ClockDiv::div1, 20'000'000>`.
Vectors are bound by name in the apps and
the EA lacks several the DA/DB have (`USART3..5_*`, `TCD0_*`, `ZCD*`,
`AC2_AC`, `NVMCTRL_EE` and others): an app binding one fails the EA
build (`-Werror=misspelled-isr`), so an app is built for `ea48` only
once it binds the EA's names.

`F_CPU` is not defined in this project: nothing in `CMakeLists.txt`
ever adds a `-DF_CPU`, since the flags are built up from scratch rather
than inherited from a framework default that would produce one. The
rate has one truth, `Clock::hz`; avr-libc's `util/delay.h` and
`util/setbaud.h`, which need `F_CPU`, therefore do not compile, on
purpose: nothing may assume a rate the clock type does not state. (In a
build that still defines `F_CPU`, `Clock` static_asserts it equal to
`hz`.) The bench board's clock is `Clock<ClockSource::crystal,
24'000'000>`, a 24 MHz crystal on PA0/PA1.

`brio::delay_us(clock, us)` (`avrxt/delay.hpp`) busy-waits at least `us`
microseconds: a folded `__builtin_avr_delay_cycles` when `us` is a
compile-time constant (what `_delay_us` does, minus F_CPU), a 4-cycle
`_delay_loop_2` loop otherwise (`delay_us_runtime(cycles_per_us, us)`
for drivers holding a runtime setup time, such as `Spi`'s cs_setup_us).
It is for hardware setup times in drivers and pre-kernel init only:
anything measured in milliseconds inside an active object is a time
event. `_delay_us` and `_delay_ms` are not used anywhere in brio or the
apps.

There is no 32.768 kHz crystal on the bench board: `Ticker::init()`
picks the RTC clock automatically (XOSC32K if MCLKSTATUS reports a
running 32k crystal, the internal OSC32K otherwise; `Clock` never
touches XOSC32K), and `Ticker::init(source)` names it instead. Do not
enable XOSC32K (PF0/PF1) unless a 32k crystal is fitted. The Ticker owns
that clock select for the whole RTC block; the counter half
(`brio::Rtc`, [rtc.md](rtc.md)) shares it.

`brio::Ticker` is `BasicTicker<1024>`: the RTC/PIT timebase at 1024 Hz
(the PIT's power-of-two dividers, a truth of this silicon, which is why
the kernel tick rate is a platform constant), alive in IDLE sleep,
monostate, wired by the app as:

    ISR(RTC_PIT_vect) { brio::Ticker::pit(); }
    ...
    brio::Ticker::init();   // after clock init, before sei()

ISR vector bindings always live in the app (or a future board file),
never in portable code: drivers expose `[[gnu::always_inline]]` handler
bodies (`rxc()`, `dre()`, `pit()`), and the app binds the vector.

## Upload (Atmel-ICE, UPDI)

`cmake --build --preset <pkg>-release --target <app>-upload` drives
avrdude 8.1 through the Atmel-ICE (`avr_add_app()` in `CMakeLists.txt`;
`-P usb:<serial>` disambiguates when more than one Atmel-ICE is
attached, through `AVR_PROBE_SERIAL`). `brio flash <board> <app>` does
the same, resolved against the bench manifest; the probe page
([../probes/atmel-ice.md](../probes/atmel-ice.md)) says what each erase
regime leaves in the chip.

- Plug the cable into the Atmel-ICE **AVR** port, not the SAM port. The
  SAM port fails silently: avrdude still sees the ICE on USB, but
  `Vtarget` reads about 1.71 V (parasitic) and the UPDI sign-on returns
  `0xa0` / "initialization failed".
- The programming header is the standard 6-pin AVR-ISP (2x3, 2.54 mm).
  UPDI uses three pins: pin 2 = VCC, pin 5 = UPDI, pin 6 = GND (1, 3 and
  4 unused).
- Flash endurance on these parts is 1000 cycles: uploads and software
  breakpoints (avoided, "Debugging" below) come out of the same budget.

## Debugging (PyAvrOCD + Atmel-ICE)

Debug sessions use [PyAvrOCD](https://pyavrocd.io) as the GDB server
with the toolchain's avr-gdb 17.2, driven by VS Code's generic `cppdbg`
debug adapter (ms-vscode.cpptools), not a target-specific debug
extension: PyAvrOCD speaks the standard GDB remote protocol, so any
GDB-driving frontend works. Always debug a `*-debug` build (the release
configs are the pure `-Os` build): build the preset (`cmake --build
--preset avr128db48-debug --target <app>`), pick `<app>` as CMake Tools'
launch target, then F5 on the one generic entry of
`.vscode/launch.json`, which uses `${command:cmake.launchTargetPath}` to
follow whichever target is selected. The entry's end-to-end run
(connect, `monitor reset`, `load`, the stop at `main()`, the Peripheral
Viewer reading live CLKCTRL and PORTF values) is recorded in the file's
own comment.

### One-time host setup

```bash
# The GDB server (lands in ~/.local/bin/pyavrocd). Installed as a uv tool on
# a uv-managed CPython pinned to 3.12, so it survives distro Python bumps.
# Without --python-preference only-managed, uv would silently bind the venv
# to the DISTRO python if a matching one exists.
uv tool install --python-preference only-managed --python 3.12 pyavrocd
# (equivalent, distro-python-bound alternative: pipx install pyavrocd)

# udev rules for the EDBG probes (Atmel-ICE, PICkit 4, Snap, ...),
# then unplug/replug the probe:
wget https://pyavrocd.io/99-edbg-debuggers.rules
sudo cp 99-edbg-debuggers.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

### The entry in .vscode/launch.json

PyAvrOCD is launched by cppdbg itself (`debugServerPath` and
`debugServerArgs`), so no separate step is needed. The entry is "Debug
(PyAvrOCD, db48)", and its substance is:

```jsonc
{
    "type": "cppdbg",
    "request": "launch",
    "name": "Debug (PyAvrOCD, db48)",
    "program": "${command:cmake.launchTargetPath}",
    "stopAtEntry": true,
    "svdPath": "${workspaceFolder}/avrxt/svd/avr128db48.svd",
    "miMode": "gdb",
    "miDebuggerPath": "/sw/avr/bin/avr-gdb",
    "debugServerPath": "/usr/bin/env",
    "debugServerArgs": "PYTHONUNBUFFERED=1 pyavrocd --breakpoints hardware -s nop -p 40044 -m all -d avr128db48 -i updi -t atmelice -F 24000000 -P 2000",
    "serverStarted": "Listening on port",
    "setupCommands": [
        { "text": "file ${command:cmake.launchTargetPath}" },
        { "text": "target remote localhost:40044" },
        { "text": "monitor reset" },
        { "text": "load" },
        { "text": "set breakpoint always-inserted off" }
    ]
}
```

The flags: `--breakpoints hardware` (hardware breakpoints only, below),
`-s nop` (no GUI), `-p` the GDB server port, `-m all` (let PyAvrOCD
manage the relevant fuses), `-d` the MCU (`pyavrocd -d '?'` lists
them), `-i updi`, `-t` the probe kind, and `-P` the UPDI programming
clock in kHz. With more than one probe attached add `-u <serial>`. The
entry is the AVR128DB48's: for another board duplicate it with its
`-d` (`avr128db28`, `avr128db32`, `avr128da48`, `avr64ea48`), and note
that `avrxt/svd/` holds the AVR128DB48's SVD alone, so on another part
the Peripheral Viewer has no register map to read. The server is
launched through `/usr/bin/env` with
`PYTHONUNBUFFERED=1` because PyAvrOCD's "Listening on port" line is a
plain print that does not flush under a pipe, and the connection is made
by hand in `setupCommands` because cppdbg runs them before its own
target selection (the file's comment records both findings).
`svdPath` is read directly by the mcu-debug Peripheral Viewer extension
from the active session's launch config (its `svdPathConfig` setting
defaults to trying `svdPath` then `svdFile`, confirmed in the
extension's own `package.json`).

`-F` and the CPU clock: for a UPDI target `-F` is inert. PyAvrOCD uses
it only to derive the default JTAG debug clock (megaAVR JTAG sessions)
and to start simavr; the UPDI session sets its own link speed and never
reads it. UPDI has its own clock (`UPDICLKSEL`, independent of CLK_PER)
and reaches memory through the ASI without the CPU, so breakpoints and
stepping work at any CPU rate: a `DynamicClock` app running at 2 MHz
debugs exactly like one at 24 MHz. (debugWIRE, on classic AVRs, is the
interface whose speed is F_CPU/128: that is what the option exists for.)
24000000 is kept as the boot rate for tidiness.

### Breakpoints: effectively one free breakpoint

The UPDI OCD has two hardware breakpoints; single-stepping is native and
costs neither a breakpoint nor flash. GDB borrows one slot for its
temporary breakpoints (`tbreak main` opening every session, `next` over
a call, `finish`, run-to-line). **The practical rule: one breakpoint in
code plus one slot free for GDB**; two in code only while stepping with
`step`/`stepi` or `continue`.

Extra breakpoints would normally become software breakpoints, that is,
flash rewrites. The server therefore runs with `--breakpoints
hardware`: a breakpoint that does not fit is refused ("Cannot insert
breakpoint" at the next continue; delete one and go on) instead of
silently wearing flash. The session's escape hatch is `monitor
breakpoints all`; `monitor breakpoints` shows the mode.

### Making line breakpoints bind (the GCC 16 DWARF caveat)

GCC 16.x (verified 16.1 and 16.2) emits a DWARF5 line table with a
duplicate file entry for the main source and file switches around
inlined code; avr-gdb 17.2 and host gdb 15.1 then silently drop part of
it: at any -O level above 0, file:line breakpoints fail with "No
compiled code for line N" while function breakpoints work. The fix is
in `avr_add_app()`: LTO is never added ("Build" above) and the Debug
config is `-Og -g3 -ggdb3 -fno-inline`.

`-fno-inline` removes the mid-function file switches (always_inline
code such as `_delay_ms` stays inlined and keeps its timing); dropping
LTO removes the DWARF partitioning that makes it worse. The residue: a
line whose only content is a call into an always_inline system-header
function (a bare `_delay_ms(500);`) still cannot take a line breakpoint;
break on a neighbouring line. Re-test after any avr-gcc or avr-gdb
rebuild; a candidate for an upstream bug report (minimal repro: any -O1
build of a blink program).

### Peripheral registers (SVD and monitor commands)

Microchip ships no SVD for the AVR; PyAvrOCD generates them from the
ATDFs. `avrxt/svd/avr128db48.svd` is committed and wired through
`svdPath` in `.vscode/launch.json` (above), which the mcu-debug
Peripheral Viewer extension reads to populate its PERIPHERALS panel
(values read while stopped; the panel refresh can be flaky, and CLion
renders the same SVD better). The server side is always reliable from
the Debug Console:

```
monitor info
monitor reset
monitor ioregister PORTA.OUT          # read, with bitfield breakdown
monitor ioregister PORTA.OUT 0x20     # write
monitor ioregister PORTA.*            # whole peripheral
info registers                        # CPU: r0-r31, SREG, SP, PC
p/x PORTA                             # via avr/io.h macros (needs -g3)
```

With the probe-rs VS Code extension enabled, every stop and continue may
spam "unknown custom event" in the Debug Console (it eavesdrops on DAP
custom events regardless of debug adapter type). Harmless; disable
probe-rs per workspace if it does.

## Editor (clangd)

clangd works over each CMake project's own `compile_commands.json`
(`CMAKE_EXPORT_COMPILE_COMMANDS`, regenerated automatically on every
configure; no manual step). `.vscode/settings.json` enables clangd with
`--query-driver` pointed at the cross compiler, which supplies the
include search path and the `avr` target, while the `-D` delta above
supplies the device macros clang does not define
(`__AVR_DEVICE_NAME__` drives avr-libc's computed `<avr/io.h>`);
`brio/avrxt/.clangd` and `avrxt/.clangd` route the stratum and the
project to the project's database, and `test/.clangd` points straight at
the host project's own. cpptools' own engine is disabled in the same
settings file: its clang-based parser (1.33+) runs in an x86-64 model
against the full gcc macro set, and the AVR-configured libstdc++ then
demands gcc-only types (`__int24`, `_Float32`) it cannot have,
structurally unparsable, and no define feed can fix it.

## Serial console

The console is a CH340 bridge on USART2 ALT1, PF4 (TX) / PF5 (RX), on
every board of the stratum, at 460800 baud in the console apps (the
bench manifest's default `monitor_speed`); a CH340 without a USB serial
is addressed by `/dev/serial/by-path`, the two isolated CH340Bs by
`/dev/serial/by-id` ([../boards/README.md](../boards/README.md)).

## Family coverage

`brio check avrxt` compiles every smoke TU (a translation unit, one
source file that includes a driver and uses it, written to prove that
it compiles) in `test/family_avrxt/` for all eight AVR128 DA/DB
packages (28, 32, 48 and 64 pins, both families) and the three AVR64EA
packages (28, 32 and 48 pins; a chapter the EA has no driver for
compiles to nothing there, its refusal a negative), and requires every
`test/family_avrxt/neg/` TU, a negative, a program that must be refused,
to fail for the MCUs its `// mcu:` line names. Seconds, no hardware;
the packages without a board (every 64-pin one, and the 28- and
32-pin DA, DB and EA) have this sweep as their only check. Part of every driver's
definition of done, because the bench chip alone masks the missing
ports, instances, registers and enum values of the other packages. The
check ends with the link guard: every app linked on every release
preset of the project, a failure fatal.

## The documents

One document per peripheral driver, in the shape
[../README.md](../README.md) prescribes: the reader's orientation, the
documents of record, what the silicon does, the types and verbs, how to
use it, the bench findings and, where the driver has gaps, "Not covered
yet" as two lists. The line beside each names what the document is
about and the finding or two most worth knowing before using the
driver; the findings themselves, with their figures and conditions, are
in the document.

| Document | Content |
|----------|---------|
| [device_tables.hpp](../../brio/avrxt/device_tables.hpp) | The reserve, with no document of its own; its header comment is the reference: the features and instance counts each probed from its own header symbol (a macro, an enumerator or a struct member), the part (die and pin count) keyed on the compiler's device macro with every unclaimed part refused, the PORTMUX layout and route fields (mask, position, which register holds them) and the route codes per layout, and the errata and family facts keyed on the die: the DA's EVSYS pins (DS80000882C 2.6.1), its TCA1 ALT2/ALT3 (2.11.1), its swapped SDA hold (2.14.2), its data-space window (2.7.1), the DB's XDIR on USART2's ALT1, CLK_MAIN's maximum, the EA's fuse rates and its every-pin asynchrony; checked part by part by the `device_tables` fixture |
| [platform.md](platform.md) | What the kernel stands on (DA, DB, EA): `AvrPlatform` (the critical section, idle and the SLPCTRL erratum, the timebase, atomic_width, the `.noinit` breadcrumb), the short-wait role with both delay paths measured, `Sleep`/`Vreg` over all three sleep modes and the regulator (`Vreg` the DA/DB's: the EA has no regulator control) plus `AvrSleepSite`, the adapter that carries the power model onto them, and `Reset`/`Watchdog` over RSTCTRL and the WDT; the idle contract and the sleep modes measured on an AVR128DB48, an AVR128DA48 and an AVR64EA48, the EA's offer inventoried, its wake paying no regulator start-up. Not covered: idle detection and the RUNSTDBY policy, the BOD, the sleep current, the wake-up sources a lone board cannot produce |
| [clkctrl.md](clkctrl.md) | CLKCTRL: the oscillators, the PLL, the main clock mux, prescaler and CLKOUT, clock failure detection (DB, EA) as resources; `Clock` and `DynamicClock` as tasks; on the AVR128DA48 the external clock, XOSC32K as the main clock, the auto-tune and the PLL from EXTCLK measured, OSCHF found pulled onto the external clock while that clock is CLK_PER; the AVR EA's chapter (OSCHF at the fuse's rate as a claim checked at boot, MCLKTIMEBASE owned by the clock, the offer inventoried, its reset state read: OSCHFCTRLA 0x00, not the 0x0C on paper). Not covered: the DB's external-clock paths, the PLL's RUNSTDBY, A4 silicon, the EA's letters timed on a 32 kHz crystal (the bench board's does not start) |
| [evsys.md](evsys.md) | EVSYS: the full typed vocabulary (generators and users, package-gated) plus run-time connect and disconnect; the AVR EA's six channels and its pin and PIT slots named in the generator's type, the PIT slots measured (every divider DIV4..DIV32768, both at once, PITEN gating them). Not covered: static allocation, the vocabulary no driver exercises yet, a slot write's latency |
| [vref.md](vref.md) | VREF (DA, DB, EA): the reference selector, its levels, headroom and tolerance per die, how the ADC and DAC name it (the EA's ADC in a register of its own) |
| [dac.md](dac.md) | DAC (DA, DB, EA): the 10-bit actuator, buffered and unbuffered outputs, the OUTEN policy keyed on the errata, the EA's OUTRANGE, the drive measured into the board's 96 nF net; usage |
| [adc.md](adc.md) | ADC (DA, DB, EA): one task with knobs, two engines (the DA/DB's SAR converter, and the EA's differential converter with PGA: pairs as input values, burst and series accumulation to 1024 with a 32-bit sum, the window written 1 to clear, LOWLAT, the three start modes, the select engine under an event pace on an unbuffered mux, the conversion rates of tables 31-7/31-8 measured). Not covered: pin-level input legality, the standby paths, the EA's 8-bit rate, its supply current |
| [nvm.md](nvm.md) | NVMCTRL, two generations. On the DA/DB the `Nvm` resource: the four memories and what each is for, the flash sections as a compile-time claim cross-checked against the fuses, ELPM/SPM only (never the data-space window), the command model, the one-way protections, the vector-table invariant every image carries, the multi-page-erase erratum guarded page by page, plus the services over it (a typed record, an interrupt-paced writer active object, a persistent panic record), and the DA's window erratum reproduced on an AVR128DA48. On the AVR EA the EEPROM engine of NVM version 3: the page buffer's command model as the silicon runs it (fill, then command), the page run and the cheapest-operation chooser, EEWP, the offer inventoried, the cost counted in the listing and measured on an AVR64EA48, the supply guard of DS80001048D 2.2.1 with BODCFG 0x4D, its flash and User Row self-programming declined; the EEPROM store takes a run on the EA (a record, a writer request, the panic record an operation per page). Not covered: a boot loader, any flash policy, the DA's lower endurance (a page worn out to measure it), the store's run verb on the DA/DB |
| [userrow.md](userrow.md) | USERROW: the board identity label, which survives a chip erase, is written once over UPDI and is read into every suite banner by `board_id()`; 64 bytes on the AVR EA, its CPU write declined there |
| [port.md](port.md) | PORT: the `Port<L>` resource, `Pin` (a one-store PinConfig, senses, flags), `PinSet` across ports on the multi-pin engine, `PinRef`; every pin fully asynchronous on the AVR EA, one edge of each of twelve pins waking it from power-down on the bench. Not covered: INLVL and slew measurements, the DA/DB's fully-async wake on a Px6 pad, the EA's sub-CLK_PER pulse and the pins no edge reaches on its board |
| [usart.md](usart.md) | USART: the `Usart<n>` resource (the routes including the pinless one, and the AVR EA's route table of another shape: per-instance codes, routes on other ports, USART2 in USARTROUTEB, every USART0/USART1 route read back on an AVR64EA48; every frame format, the receiver modes, the errata verbs, the EA's SFDEN-in-Active and open-drain halves measured) and the tasks over it (`Uart`, `OneWire`, `Rs485`, `SyncHost`/`SyncClient`, `MspiHost`, `IrdaLink`, `AutoBaud`), the transport's cost measured at 24 and 20 MHz; the SAM C21 its second board. Not covered: LIN above auto-baud, the polled tasks' interrupt versions |
| [spi.md](spi.md) | SPI: the `Spi<n>` resource (the per-package route table with the errata that beats it; the AVR EA's SPI0 ALT1..ALT6, its UPDI-pin ALT6 and two-port ALT3 refused, NONE at code 7 and ALT4/ALT5 read back on an AVR64EA48; both roles, the seven rates, both INTFLAGS layouts and their clear disciplines, the host demotion), the tasks over it (the `SpiHost` transfer engine in buffer mode, its costs counted in the listing and measured by the benchmark skeleton's letter `e` at 24 and 20 MHz; `SpiClient`), and the read the silicon does not take when it empties the FIFO as a byte enters it, which the engine reads around; the SAM C21 the second device on the wire. Not covered: a client-side active object, multi-host as a protocol |
| [twi.md](twi.md) | TWI: the `Twi<n>` resource (the per-package route table with its dual pin pairs, the AVR EA's ALT3 included; the three errata as code; the chapter's own baud arithmetic with the bus's edges as arguments, on the AVR EA its own SCL duty, measured: FMEN used with Fm, the low phase's model, the MBAUD floor, Fm+'s 12 MHz; the time-out on MCLKTIMEBASE; SDAHOLD straight) and the tasks over it (the `I2cHost` transfer engine, its cost measured by the benchmark skeleton's letter `i` at 24 and 20 MHz; `I2cClient` including Dual mode, SDASETUP at eight clocks by default, four losing a Smart client in a long read on the AVR EA's dual loop); its second chip on the wire the SAM C21. Not bench-verified: the engine's `i2c_arb_lost` as the DUT, the DA/DB's client at SDASETUP eight, INPUTLVL as an electrical, the EA's Fm+ pads, TWI1 and the routes other than TWI0's |
| [rtc.md](rtc.md) | RTC/PIT: the `RtcClock`, `Rtc` and `Pit` resources (one clock for both functions, a counter with period and compare, crystal error correction, the busy flags) plus `BasicTicker` over the PIT; the 32.768 kHz crystal as CLK_RTC measured on the AVR128DA48 board, the AVR EA's PIT event slots from the RTC's side (`Pit::event_divider<slot>`), the AVR64EA48's OSC32K and busy flags measured. Not covered: XOSC32K on the AVR EA (the bench board's crystal does not start), the external-clock sources, the standby and debug-run paths |
| [tca.md](tca.md) | TCA: the `Tca` resource (normal mode, buffered compares, event inputs, commands) plus the tasks `TcaPwm`, `TcaPwm16`, `FrequencyGenerator`, `Heartbeat` and `EventCounter`; the route table per layout (the AVR EA's TCA1 on every package, its three-channel PORTA/PORTD routes). Not covered: the split halves' counters as verbs |
| [tcb.md](tcb.md) | TCB: the `Tcb` resource (eight modes, event clock and capture, cascade, routes) plus the tasks `PeriodicTick`, `Timeout`, `OneShotPulse`, `PulseCounter`, `CascadedCounter`, the meters and `Pwm8`. Not covered: pin-level bonding within a port |
| [tcd.md](tcd.md) | TCD: the `Tcd<0>` resource, the full chapter with its three synchronization disciplines enforced by the verbs, the per-package route table, the input-mode validity table and the errata that shrink it, the 12-bit captures and their read discipline, dithering; plus `TcdPwm`, the complementary pair with dead time, and the PLL made observable through it, from OSCHF and, on the DA, from its external clock. Not covered: the usage types waiting for their first user, the DB's external clock source, errata that would not reproduce on either die |
| [ccl.md](ccl.md) | CCL: the `Ccl` and `Lut<n>` resources (the inputs menu, the truth table, filter and edge, clocks, pins, the whole-block reconfiguration erratum) plus `ToggleFlipFlop`; the AVR EA's four LUTs and its input menu (AC1 on input 2) measured. Not covered: typed per-input instance legality |
| [ac.md](ac.md) | AC (DA, DB, EA): the `Ac<n>` resource (inputs and DACREF, hysteresis and power, pin, event and interrupt, window; the EA's AINP4/AINN3) plus `Threshold` and `Window`. Not covered: pin-level bonding (PD0, PC6 on small packages) |
| [opamp.md](opamp.md) | OPAMP (DB only): the `OpampSystem` block (the one ENABLE, the TIMEBASE that makes a settle time mean microseconds, and the ClockUser hook that keeps it true across a rebase) plus `Opamp<n>`: both input multiplexers with their per-instance link codes, the 16R ladder and its eight exact gains, the output driver, the three enable regimes, the internal timer and READY, the four event users and the offset trim; plus the tasks `OpampFollower`, `OpampPga`, `OpampInvertingPga` and the chapter's three-op-amp `InstrumentationAmp`. Not covered: the integrator usage type (external R and C, and a DUMP policy), RUNSTBY, IRSEL's electrical effect |
| [vendor/README.md](vendor/README.md) | The datasheets and errata the stratum is written against, by document number (PDFs kept local, not in git) |
