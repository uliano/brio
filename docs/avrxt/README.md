# Target: AVR DA, DB and EA (`avrxt/`)

The stratum `brio/avrxt/` is everything in brio that knows
`avr/io.h`: clock init, `Pin`, `Uart`, `Spi`, `Twi`, `BasicTicker`,
and `AvrPlatform` (the kernel's `Platform` concept implemented with
AVR intrinsics). Family differences inside DA/DB/EA are read off the
device header in one place, `avrxt/device_tables.hpp` (the reserve):
each feature from the symbol that IS it (`CLKCTRL_XOSCHFCTRLA` exists
only on DB, `PORTB`/`PORTE`/`PORTG` depend on the package), the die
from the compiler's device macro and the errata keyed on the die, so
the same headers build for both and a part the stratum does not claim
is a compile error. This page is the operational side: toolchain, board,
probe, debugger, and their quirks. The design side (why the ticker
is declaredly AVR, the ISR binding pattern, ...) is in
[../design/](../design/).

Bench MCUs: **AVR128DB48** (48-pin, 128 KB flash, 16 KB SRAM),
**AVR128DA48** (the same memories, no crystal oscillator and no MVIO) and
**AVR64EA48** (64 KB flash, 6 KB SRAM, 512 B EEPROM: OSCHF at a rate its
OSCCFG fuse chooses, no PLL, no TCD, no OPAMP, NVM version 3, a
differential ADC; the stratum claims the AVR64EA28/32/48), each on a
board of its own, programmed and debugged with an **Atmel-ICE** over UPDI
(at firmware 1.46 or later for the EA: 1.41 acknowledges and loses every
NVM version 3 write). Board wiring and what each board carries:
[../boards/avr128db48.md](../boards/avr128db48.md),
[../boards/avr128da48.md](../boards/avr128da48.md),
[../boards/avr64ea48.md](../boards/avr64ea48.md).

**On the AVR EA, in bring-up**: the build, the clock (the fuse's OSCHF
rate as a claim, MCLKTIMEBASE owned by the clock), the pins (every pin
fully asynchronous), the event system's pin and PIT slots, the USART,
SPI and TWI route tables re-expressed per PORTMUX layout, reset, sleep
and the platform are written from DS40002443A and compile on the three
packages; the TCA (its own route table), the TCB, the CCL (its input
menu read from the header, AC1 on input 2) and the RTC (the PIT's event
slots) and NVMCTRL's EEPROM engine (NVM version 3: the page buffer, the
three page operations, EEWP, the supply guard of DS80001048D 2.2.1) are
on the bench, its flash and User Row self-programming declined; the
differential ADC with its PGA has an engine of its own behind the
ADC's surface, and it, the DAC, the VREF and the comparators are on the
bench against the board's analog net. What the silicon has confirmed is
each document's.

## The documents

One document per peripheral driver, in the shape
[../README.md](../README.md) prescribes; the vendor page carries the
documents of record.

| Document | Content |
|----------|---------|
| [device_tables.hpp](../../brio/avrxt/device_tables.hpp) | The reserve - no document of its own, its header comment is the reference: the features and instance counts each probed from its own header symbol (a macro, an enumerator or a struct member), the part (die and pin count) keyed on the compiler's device macro with every unclaimed part refused, the PORTMUX layout and route fields (mask, position, which register holds them) and the route CODES per layout, and the errata and family facts keyed on the die - the DA's EVSYS pins (DS80000882C 2.6.1), its TCA1 ALT2/ALT3 (2.11.1), its swapped SDA hold (2.14.2), its data-space window (2.7.1), the DB's XDIR on USART2's ALT1, CLK_MAIN's maximum, the EA's fuse rates and its every-pin asynchrony; checked part by part by the `device_tables` fixture |
| [platform.md](platform.md) | Platform (DA, DB, EA): what the kernel stands on - `AvrPlatform` (critical section, idle and the SLPCTRL erratum, timebase, atomic_width, the `.noinit` breadcrumb), the short-wait role with both delay paths measured, `Sleep`/`Vreg` over all three sleep modes and the regulator (`Vreg` the DA/DB's: the EA has no regulator control) plus `AvrSleepSite`, the adapter that carries the power model onto them, and `Reset`/`Watchdog` over RSTCTRL and the WDT, the idle contract and the sleep modes measured on an AVR128DB48, an AVR128DA48 and an AVR64EA48 - the EA's offer inventoried, its wake paying no regulator start-up; not covered: idle detection and the RUNSTDBY policy, the BOD, the sleep current, and the wake-up sources a lone board cannot produce |
| [clkctrl.md](clkctrl.md) | CLKCTRL: oscillators, PLL, main clock mux/prescaler/CLKOUT, clock failure detection (DB, EA) as resources; Clock/DynamicClock as tasks; on the AVR128DA48 the external clock, XOSC32K as the main clock, the auto-tune and the PLL from EXTCLK measured, OSCHF found pulled onto the external clock while that clock is CLK_PER; the AVR EA's chapter - OSCHF at the fuse's rate as a claim checked at boot, MCLKTIMEBASE owned by the clock, the offer inventoried, its reset state read (OSCHFCTRLA 0x00, not the 0x0C on paper); not covered: the DB's external-clock paths, the PLL's RUNSTDBY, A4 silicon, the EA's suite on the bench |
| [evsys.md](evsys.md) | EVSYS: the full typed vocabulary (generators and users, package-gated) + run-time connect/disconnect; the AVR EA's six channels and its pin and PIT SLOTS named in the generator's type, the PIT slots measured (every divider DIV4..DIV32768, both at once, PITEN gating them); not covered: static allocation, the vocabulary no driver exercises yet, a slot write's latency |
| [vref.md](vref.md) | VREF (DA, DB, EA): the reference selector - levels, headroom and tolerance per die, how the ADC/DAC name it (the EA's ADC in a register of its own) |
| [dac.md](dac.md) | DAC (DA, DB, EA): the 10-bit actuator - buffered/unbuffered outputs, the OUTEN policy keyed on the errata, the EA's OUTRANGE, the drive measured into the board's 96 nF net; usage |
| [adc.md](adc.md) | ADC (DA, DB, EA): one task with knobs, two engines - the DA/DB's SAR converter, and the EA's differential converter with PGA: pairs as input values, burst and series accumulation to 1024 with a 32-bit sum, the window written 1 to clear, LOWLAT, the three start modes, the select engine under an event pace on an unbuffered mux, the conversion rates of tables 31-7/31-8 measured; not covered: pin-level input legality, the standby paths, the EA's 8-bit rate, its supply current |
| [nvm.md](nvm.md) | NVMCTRL, two generations: on the DA/DB the `Nvm` resource - the four memories and what each is for, the Flash sections as a compile-time claim cross-checked against the fuses, ELPM/SPM only (never the data-space window), the command model, the one-way protections, the vector-table invariant every image carries, and the multi-page-erase erratum guarded page by page - plus the services over it (typed record, interrupt-paced writer AO, persistent panic record), and the DA's window erratum reproduced on an AVR128DA48; on the AVR EA the EEPROM engine of NVM version 3 - the page buffer's command model as the silicon runs it (fill, then command), the page run and the cheapest-operation chooser, EEWP, the offer inventoried, the cost counted in the listing and measured on an AVR64EA48, the supply guard of DS80001048D 2.2.1 with BODCFG 0x4D - its flash and User Row self-programming declined; the EEPROM store takes a run on the EA (a record, a writer request, the panic record an operation per page); not covered: a boot loader, any flash policy, the DA's lower endurance (a page worn out to measure it), the store's run verb on the DA/DB |
| [userrow.md](userrow.md) | USERROW: the board identity label - survives chip erase, written once over UPDI, read into every suite banner by board_id(); 64 bytes on the AVR EA, its CPU write declined there |
| [port.md](port.md) | PORT: Port<L> resource, Pin (one-store PinConfig, senses, flags), PinSet across ports on the multi-pin engine, PinRef; every pin fully asynchronous on the AVR EA, one edge of each of twelve pins waking it from power-down on the bench; not covered: INLVL/slew measurements, the DA/DB's fully-async wake on a Px6 pad, the EA's sub-CLK_PER pulse and the pins no edge reaches on its board |
| [usart.md](usart.md) | USART: the `Usart<n>` resource - routes incl. the pinless one (and the AVR EA's route table of another shape: per-instance codes, routes on other ports, USART2 in USARTROUTEB), every frame format, the receiver modes, the errata verbs - and the tasks over it (`Uart`, `OneWire`, `Rs485`, `SyncHost`/`SyncClient`, `MspiHost`, `IrdaLink`, `AutoBaud`); not bench-verified: everything that needs a second board |
| [spi.md](spi.md) | SPI: the `Spi<n>` resource - the per-package route table with the errata that beats it (the AVR EA's SPI0 ALT1..ALT6, its UPDI-pin ALT6 and two-port ALT3 refused), both roles, the seven rates, both INTFLAGS layouts and their clear disciplines, the host demotion - and the tasks over it (`SpiHost` transfer engine in buffer mode, its costs counted in the listing and measured by `bench_avr`'s letter `e`, `SpiClient`), and the read the silicon does not take when it empties the FIFO as a byte enters it, which the engine reads around; not bench-verified: everything that needs a second device on the wire |
| [twi.md](twi.md) | TWI: the `Twi<n>` resource - the per-package route table with its dual pin pairs (the AVR EA's ALT3 included), the three errata as code, the chapter's own baud arithmetic with the bus's edges as arguments, both halves - and the tasks over it (`I2cHost` transfer engine, `I2cClient` including Dual mode); its second chip on the wire the SAM C21; not bench-verified: the engine's `i2c_arb_lost` as the DUT, the address-match wake timed from outside, SDASETUP and INPUTLVL as electricals, TWI1 and the routes other than TWI0's |
| [rtc.md](rtc.md) | RTC/PIT: RtcClock/Rtc/Pit resources (one clock for both functions, counter with period and compare, crystal error correction, the busy flags) + BasicTicker over the PIT, the 32.768 kHz crystal as CLK_RTC measured on the AVR128DA48 board, the AVR EA's PIT event slots from the RTC's side (`Pit::event_divider<slot>`), the AVR64EA48's OSC32K and busy flags measured; not covered: XOSC32K on the AVR EA (the bench board's crystal does not start), the external-clock sources, the standby and debug-run paths |
| [tca.md](tca.md) | TCA: the Tca resource (normal mode, buffered compares, event inputs, commands) + tasks TcaPwm/TcaPwm16/FrequencyGenerator/Heartbeat/EventCounter; the route table per layout (the AVR EA's TCA1 on every package, its three-channel PORTA/PORTD routes); not covered: the split halves' counters as verbs |
| [tcb.md](tcb.md) | TCB: the Tcb resource (eight modes, event clock/capture, cascade, routes) + tasks PeriodicTick/Timeout/OneShotPulse/PulseCounter/CascadedCounter/meters/Pwm8; not covered: pin-level bonding within a port |
| [tcd.md](tcd.md) | TCD: the `Tcd<0>` resource - the full chapter with its three synchronization disciplines enforced by the verbs, the per-package route table, the input-mode validity table and the errata that shrink it, the 12-bit captures and their read discipline, dithering - plus `TcdPwm`, the complementary pair with dead time, and the PLL made observable through it, from OSCHF and, on the DA, from its external clock; not covered: the usage types waiting for their first user, the DB's external clock source, errata that would not reproduce on either die |
| [ccl.md](ccl.md) | CCL: Ccl + Lut<n> resources (inputs menu, truth table, filter/edge, clocks, pins, the whole-block reconfiguration erratum) + ToggleFlipFlop; the AVR EA's four LUTs and its input menu (AC1 on input 2) measured; not covered: typed per-input instance legality |
| [ac.md](ac.md) | AC (DA, DB, EA): the Ac<n> resource (inputs and DACREF, hysteresis/power, pin/event/interrupt, window; the EA's AINP4/AINN3) + Threshold/Window; not covered: pin-level bonding (PD0, PC6 on small packages) |
| [opamp.md](opamp.md) | OPAMP (**DB only**): the `OpampSystem` block (the one ENABLE, the TIMEBASE that makes a settle time mean microseconds, and the ClockUser hook that keeps it true across a rebase) + `Opamp<n>` - both input multiplexers with their per-instance link codes, the 16R ladder and its eight exact gains, the output driver, the three enable regimes, the internal timer and READY, the four event users and the offset trim - plus the tasks `OpampFollower`, `OpampPga`, `OpampInvertingPga` and the chapter's three-op-amp `InstrumentationAmp`; not covered: the integrator usage type (external R and C, and a DUMP policy), RUNSTBY, IRSEL's electrical effect |
| [vendor/README.md](vendor/README.md) | The datasheets/errata the stratum is written against, by document number (PDFs kept local, not in git) |

## Toolchain

Self-built **avr-gcc 16.2** + avr-libc + avr-gdb 17.2 + avrdude 8.1
(built by `/sw/src/build-avr.sh`), pointed at directly by absolute path
in `cmake/toolchain-avr.cmake` (`AVR_TOOLCHAIN_DIR`, default `/sw/avr`,
a symlink to `/sw/avr-16.2`) - no package manifest or download step
needed, unlike a build-tool-managed toolchain package.

The toolchain ships the freestanding libstdc++ (type_traits, concepts,
bit, span, optional, expected, variant - no chrono/charconv/iostream);
brio uses it instead of hand-rolled traits.

`-std=gnu++23` is set once, explicitly, in `CMakeLists.txt`'s
`avr_add_app()` - nothing else in this build adds a competing `-std`
flag, so there is no "last one wins" hazard to guard against.

## Board and build

- `cmake/avr-mcus.cmake`: board type -> avr-gcc mcu name table (the
  AVR128DA/DB 128K flash / 16K RAM, the AVR64EA 64K / 6K; `db48` is the
  default board when an app names none, the others are selected by the
  app's `boards` line below). The AVR64EA is `avrxmega2`: no RAMPZ, and
  FLMAP selects the 32 KB flash section the data space shows
  (`.rodata` lives at 0x8000 while the code is shorter than 32 KB). No `-DF_CPU` is ever produced - the clock rate is
  `brio::Clock<...>::hz`, see below.
- `CMakePresets.json`: one configure + build preset pair per bench
  package x {release, debug} - the AVR128DB's three, the AVR128DA48 and
  the AVR64EA48 (`avr128db48-release`, `avr128db48-debug`,
  `avr128db28-release`, ..., `avr128da48-release`, `avr64ea48-release`); each points `AVR_MCU` and the toolchain
  file (`cmake/toolchain-avr.cmake`) at that package. Atmel-ICE upload
  is a per-app `<app>-upload` target (`CMakeLists.txt`); debug wiring
  is `.vscode/launch.json` (below).
- App discovery (`CMakeLists.txt`): every `src/apps/*.cpp` with a
  `main()` becomes an executable target, auto-discovered at configure
  time (`file(GLOB ... CONFIGURE_DEPENDS ...)` - a new or removed app
  is picked up on the next configure, no generation step); see
  "Per-app build options" below for what an app can add to its own
  header comment. The glob also covers `../experiments/*/avrxt/*.cpp`:
  a top-level experiment directory holds both architectures' app
  halves plus their shared protocol header and documents itself in its
  own README (same `// build:` grammar, names unique within the
  architecture).
- `CMakeLists.txt`'s `avr_add_app()`: per-language AVR flags,
  build-type aware (`-Os -g` only on the Release config, `-std=gnu++23`
  plus `-fno-exceptions -fno-rtti -fno-threadsafe-statics
  -fno-use-cxa-atexit`), plus the -mmcu macro delta made explicit
  (`avr_predefines()`: asks avr-gcc for its predefines with and without
  `-mmcu` and appends the `__AVR*` difference as `-D`s - same values
  the real compile already implies), and on the link the refused C
  library (`cmake/avr-refused-libc.rsp`: avr-libc's heap and stdio
  wrapped with no wrapper defined, design/runtime.md). The host project under `test/` is
  an entirely separate CMake project (its own `CMakeLists.txt`/
  `CMakePresets.json`, host g++, no cross toolchain - a CMake configure
  has exactly one compiler).
- Editor: clangd over each CMake project's own compile_commands.json
  (`CMAKE_EXPORT_COMPILE_COMMANDS`, regenerated automatically on every
  configure - no manual step); `.vscode/settings.json` enables clangd
  with `--query-driver` pointed at the cross compiler, which supplies
  the include search path and the `avr` target, while the `-D` delta
  above supplies the device macros clang does not define
  (`__AVR_DEVICE_NAME__` drives avr-libc's computed `<avr/io.h>`).
  `test/.clangd` points straight at the host project's own database.
  cpptools' own engine is disabled in the
  same file: its clang-based parser (1.33+) runs in an x86-64 model
  against the full gcc macro set, and the AVR-configured libstdc++ then
  demands gcc-only types (`__int24`, `_Float32`) it cannot have -
  structurally unparsable, no define feed can fix it.
- `avr_add_app()`'s post-build step: source-interleaved disassembly
  `<app>.lst`, the linker map `firmware-<app>.map` and `<app>.hex`, all
  written directly into that target's own build dir
  (`build-cmake/<preset>/`).
- **`src/glue/ivsel_boot.cpp`: compiled into every image** (every
  `avr_add_app()` call lists it alongside the app's own source - see
  `CMakeLists.txt`). It holds build invariants that must hold before
  any app code runs and that no app may be trusted to remember. Today
  that is one file: a four-instruction `.init3` fragment that sets
  `CPUINT.CTRLA.IVSEL` under CCP, so the interrupt vector table is
  looked for at address 0. Without it, any image on a chip whose
  `BOOTSIZE` fuse is not 0 jumps into erased Flash on its first
  interrupt - a reset loop, not a crash. Setting it is correct under
  every geometry, because the BOOT section always starts at 0. Details
  and the run-time twin: [nvm.md](nvm.md).
- **The board type reaches every app.** `avr_add_app()` defines
  `BRIO_BOARD_<type>` (the configure's board type, `AVR_BOARD`), and
  `src/apps/avr_board.hpp` - the board file's first draft - maps it to
  what the suites read of the ASSEMBLY: `BoardClock` (the HF source, its
  rate and the OSCHF rate behind it), the 32 kHz crystal, what hangs on
  PA7 (CLKOUT allowed or not), whether a crystal pin may be driven,
  jumpered pin pairs, the console's USART and route. A board type the
  table does not know is a compile error; the packages with no board
  (db28, db32) have no row, their one app reading no board fact.
- **FLMAPLOCK is set in every image, by default.** gcc 16 places
  `.rodata` in a 32 KB Flash section reached through the data-space
  window and emits a write of `NVMCTRL.CTRLB.FLMAP` in `.init`; the
  linker script ORs the lock bit into that write when the symbol
  `__flmap_lock` is non-zero, and `avr_add_app()` appends
  `-Wl,--defsym,__flmap_lock=<value>` (default 1) to every AVR link.
  brio's Flash verbs never use the window (ELPM/SPM with a 24-bit
  address only), so the window is a mode nothing uses - and a mode
  nothing uses can only change by accident. On the AVR64EA the crt's
  write reads (`__do_flmap_init`, in `.init3` right after
  `brio_vectors_in_boot`): it loads NVMCTRL.CTRLB (reset 0x30: FLMAP 3,
  which on the 64 KB part aliases section 1, DS40002443A 11.5.2), keeps
  every bit but FLMAP, writes FLMAP = 1 (the upper 32 KB, where the
  linker put `.rodata`) ORed with FLMAPLOCK under CCP - `lds; andi 0xCF;
  ori 0x90; out CCP; sts`, read in every EA listing - so the map is the
  linker's and locked till the next reset. An app that must exercise
  the field opts out with `// build: flmap_lock = 0` in its header.
- **Fuses are provisioning, not build output.** The CPU can read them
  and nothing more; only the programmer writes them, so they are a
  property of the chip on the desk and live behind
  `brio fuses <board> [name=value ...]` (see
  [../boards/avr128db48.md](../boards/avr128db48.md) for the standing geometry). The one that
  matters to the build is `BOOTSIZE`: with its shipping default of 0
  the whole Flash is one BOOT section and no software can write any
  Flash at all.
- Release and debug flags are fully separated in `avr_add_app()`
  (`$<$<CONFIG:Release>:-Os -g>` / `$<$<CONFIG:Debug>:-Og -g3 -ggdb3
  -fno-inline>` generator expressions - each configurePreset picks one
  `CMAKE_BUILD_TYPE`, so the two profiles never mix). LTO is never
  added in the first place (no flag to unflag): it makes the DWARF
  problem below worse and buys nothing at this firmware size (and
  yields an unreadable `.lst`).
- Do NOT add `-mrelax`: PyAvrOCD refuses ELF files built with it
  (distorted line-number info).
- **Two ISRs with one body are folded by gcc into a call.** avr-gcc's
  identical code folding (in -Os and -O2 alike, measured on 16.2)
  turns the second of two `ISR()` bindings with the same body into a
  handler that saves its registers, CALLS the first and restores
  them: the first ends in `reti`, which returns into the caller with
  the I flag already set, so the caller's epilogue runs with
  interrupts enabled - a window of some forty cycles in which another
  interrupt can nest, against the kernel's one-boundary promise
  ([../design/kernel.md](../design/kernel.md), section 1) - and every
  such interrupt pays two prologues. No image of this project carries
  a folded vector today (every release image scanned: no `__vector_N`
  calls another), so two bindings of one body are written with
  `__attribute__((no_icf))` as the ISR's second argument, which is
  what the other RISC-V strata carry in their handler attribute.
- **Family compile check**: `brio check avrxt` compiles every
  smoke TU in `test/family_avrxt/` for all eight AVR128 DA/DB packages
  (28/32/48/64 pins, both families) and the three AVR64EA packages
  (28/32/48 pins; a chapter the EA has no driver for compiles to
  nothing there, its refusal a negative) and requires every
  `test/family_avrxt/neg/` TU to FAIL for the MCUs its `// mcu:` line
  names. Seconds, no hardware; part of every driver's definition of
  done - the bench chip alone masks missing ports, instances,
  registers and enum values of the other packages.

## Per-app build options: `// build:` header lines

There is no generated file to hand-edit: `CMakeLists.txt` scans every
`src/apps/<app>.cpp` itself at configure time for comment lines of the
shape

    // build: monitor_speed = 115200

Any key other than `boards` (below) and `flmap_lock` (see FLMAPLOCK
above) is collected as metadata and written into
`build-cmake/apps_avrxt.json` at every configure - `bin/brio`
reads it instead of a build-tool manifest, e.g. to pick a console's
`monitor_speed` (a console fact, not a compiler flag: nothing in
`CMakeLists.txt` interprets it beyond passing it through). Rules: one
line per key, any key name (unrecognized ones just become manifest
metadata nothing currently reads - there is no validation to complain
about an unknown one). No regeneration step: adding, removing or
editing such a line takes effect on the next configure (CMake Tools
reconfigures automatically on save; `cmake --preset ...` by hand
otherwise).

One key is RESERVED and consumed directly instead of being passed
through: `// build: boards = db28,db32,db48` declares the board TYPES
the app is built for (default, if absent: `db48` only). A CMake
configure targets exactly one package (`AVR_MCU`, one value per
configurePreset) - the app becomes a target only when the currently
configured package is in its list, so switching configurePreset in the
CMake Tools status bar switches which apps show up in the Target
dropdown. A target is a build, never a physical board: the boards on
the desk live in the bench manifest, see [../boards/README.md](../boards/README.md).

### The group axis: suites sized for the smallest AVR in view

A suite's image is held to the smallest chip of the family
(design/overview.md, "A suite's image fits the family's smallest
chip"), and on the AVR that chip is one IN VIEW rather than one the
project builds: the **ATmega4809, 48 KB of flash and 6 KB of SRAM**,
below the AVR64EA48's 64 KB and 6 KB. `CMakeLists.txt` names it as
the SPLIT TIER (`AVR_SPLIT_TIER_FLASH`, `AVR_SPLIT_TIER_RAM`) and lists
the board types that stand in for it (`AVR_SPLIT_BOARDS`: `ea48`, the
smallest part the project builds; the ATmega4809's own board type joins
the list when it is born). On those board types:

- a suite that declares its groups of letters (`// build: groups =
  123456789norsdt,cmliwekx`) builds as one image per group -
  `test_avr_timer-1`, `test_avr_timer-2` - each compiled with
  `BRIO_TEST_LETTERS` naming its group, which `util/testbench.hpp`
  reads to register those letters alone; the menu of such an image says
  which letters it carries. A group names lower-case letters, digits
  and, once its line names any, upper-case keys (`util/testbench.hpp`,
  `upper_mask_of()`);
- EVERY image, grouped or not, is held to the tier after its link:
  `cmake/avr-tier-guard.cmake` sums its flash (`.text`, `.data`,
  `.rodata`) and its static RAM (`.data`, `.bss`, `.noinit`) and fails
  the build over 48 KB or 6 KB, though the AVR64EA48 would hold the
  image. The configure writes the tier to `tier.json` in the build
  directory, and `brio fit` measures that preset's margins against it:
  an image with under 5 % of the tier left is named, and is regrouped.

On the 128 KB DA and DB board types a suite stays ONE image with every
letter - unless it names that board type's own groups (`// build:
groups.da48 = ...`), which split it there alone. The roster
(`build-cmake/apps_avrxt.json`) carries `split_boards` and each app's
groups, so `brio flash <board> test_avr_timer-2` names an image on an
EA board and the bare name is refused there with the list; `brio run`
judges each image's own `ALL:` line, and a two-key suite (`z` alone,
`y` against the partner) answers each key with the letters of that set
the image carries - an `ALL: 0 pass, 0 fail` where it carries none.

The suites grouped, each split along a line its letters already draw,
with every image's margin against the tier (`brio fit avrxt`):

| suite | groups | what each group is |
|---|---|---|
| `test_avr_timer` | `123456789norsdt`, `cmliwekx` | the TCA and TCB tasks; the CCL, the AC, the event system and the EA's own letter |
| `test_avr_sleep` | `abcdefgnop`, `hijklmq` | the single-board set `z`; the two-board set `y` |
| `test_avr_serial` | `abcdefghiIT`, `jklmnopq`, `rstuvwxB` | the single-board set; the two-board set in two halves, the by-name letters with the second |
| `test_avr_spi` | `abcdefghij`, `tklmnop`, `qrsuvw` | the single-board set but letter t; t with the first half of the two-board set; its second half |
| `test_avr_twi` | `abcdefghij`, `klmnopqrst` | the single-board set; the two-board set |

The serial, SPI and TWI suites do not build for `ea48`: the EA's
USART, SPI and TWI engines are gaps of usart.md, spi.md and twi.md. Their
groups are measured on the AVR128DB48's build of each group, every one
of which leaves 18 % or more of the tier. Every other suite fits the
tier whole.

## Clock, delay and timebase

(The target-independent model - one rate truth, static and dynamic
regimes, the rebase fan-out and its checks - is
[../design/clock.md](../design/clock.md); this section is the AVR
DA/DB realization.)

The main clock is a TYPE, `brio::Clock<source, source_hz, div>`
(`avrxt/clock.hpp`), and `Clock::hz` is the one truth every driver of
this target derives its divisors from: `Uart::init(clock, baud)`,
`Twi::init(clock)`, `Spi::init(clock)`, `delay_us(clock, us)` all take
the app's clock tag. Sources: `internal` (OSCHF at 1/2/3/4/8/12/16/20/
24 MHz), `crystal` (XOSCHF on PA0/PA1, DB only), `external` (EXTCLK on
PA0); `div` is the main prescaler. `Clock::init()` - first line of
`main()` - brings CLK_PER up and returns true when running from the
requested source, false when an external source failed and OSCHF runs
at the SAME rate (which is why an external rate must be one OSCHF can
produce: `hz` stays true either way). `is_static` is true.

The runtime regime is `brio::DynamicClock<Boot, Users...>`: `Boot` is a
static `Clock<...>` naming the source, `Users` types satisfying the
`ClockUser` concept (a static `rebase(hz)`, checked where the list is
written); `set<hz>()` / `set(hz)` name the new RATE (Hz - the prescaler
that produces it is the silicon's detail; an unreachable rate is a
compile error / a false) and fan it out to every listed user (`Uart`,
`Twi`, `Spi` expose `rebase(hz)`; `Uart` drains its TX at the old rate
first) and THEN reprograms the main prescaler; `hz()` is a value,
`is_static` false, `clock_hz(clock)` reads either kind. `delay_us` takes
the runtime path. The RTC/PIT timebase does not move. The subscription
is explicit and checked: a clocked driver initialized with a
DynamicClock that does not list it fails to compile at its `init(clock)`
(`clock_follows<Clock, Driver>()`) - forgetting a user cannot leave it
silently at the old rate. Do not switch while a bus transaction is in
flight (ask the bus AOs); RX bytes during the switch may be garbled.
`Uart::can_baud(hz, baud)` tells whether a rate can still hit a baud
(BAUD >= 64, i.e. CLK_PER >= 16 x baud). Bench-verified: 24 -> 12 -> 2
MHz under a running 115200 console, 1 MHz refused as expected.

**On the AVR EA** the internal oscillator's rate is the OSCCFG fuse's -
16 or 20 MHz, no FRQSEL - and the `Clock` type names it as a claim the
init checks with one read of the fuse ([clkctrl.md](clkctrl.md)); CLK_MAIN
tops at 20 MHz (from 4.5 V), and the clock owns CLKCTRL.MCLKTIMEBASE
(cycles per microsecond, rounded up), rewritten at every rate change.
The EA board: **a 20 MHz quartz on PA0/PA1**, `BoardClock` =
`Clock<ClockSource::crystal, 20'000'000, ClockDiv::div1, 20'000'000>`.
Vectors are bound by NAME in the apps and the EA lacks several the DA/DB
have (`USART3..5_*`, `TCD0_*`, `ZCD*`, `AC2_AC`, `NVMCTRL_EE`, ...): an
app binding one fails the EA build (`-Werror=misspelled-isr`), so an app
is built for `ea48` only once it binds the EA's names.

`F_CPU` is NOT defined in this project: nothing in `CMakeLists.txt`
ever adds a `-DF_CPU`, since flags here are built up from scratch
rather than inherited from a framework default that would produce one.
The rate has one truth, `Clock::hz`; avr-libc's `util/delay.h` and
`util/setbaud.h`, which need `F_CPU`, therefore do not compile - on
purpose: nothing may assume a rate the clock type does not state. (In
a build that still defines `F_CPU`, `Clock` static_asserts it equal to
`hz`.) The bench board: **24 MHz crystal on PA0/PA1** (PA0/PA1
therefore not GPIO), `Clock<ClockSource::crystal, 24'000'000>`.

`brio::delay_us(clock, us)` (`avrxt/delay.hpp`) busy-waits AT LEAST
`us` microseconds: a folded `__builtin_avr_delay_cycles` when `us` is
a compile-time constant (what `_delay_us` does, minus F_CPU), a 4-cycle
`_delay_loop_2` loop otherwise (`delay_us_runtime(cycles_per_us, us)`
for drivers holding a runtime setup time, e.g. `Spi`'s cs_setup_us).
For hardware setup times in drivers and pre-kernel init only:
anything measured in milliseconds inside an AO is a time event.
`_delay_us`/`_delay_ms` are not used anywhere in brio or the apps.

There is NO 32.768 kHz crystal on the bench board: `Ticker::init()`
picks the RTC clock automatically (XOSC32K if MCLKSTATUS reports a
running 32k crystal, internal OSC32K otherwise; `Clock` never touches
XOSC32K), and `Ticker::init(source)` names it instead. Do not enable
XOSC32K (PF0/PF1) unless a 32k crystal is fitted. The Ticker owns that
clock select for the whole RTC block - the counter half (`brio::Rtc`,
[rtc.md](rtc.md)) shares it.

`brio::Ticker` = `BasicTicker<1024>`: RTC/PIT timebase at 1024 Hz
(the PIT's power-of-two dividers - a truth of this silicon, which is
why the kernel tick rate is a platform constant), alive in IDLE
sleep, monostate, wired by the app as:

    ISR(RTC_PIT_vect) { brio::Ticker::pit(); }
    ...
    brio::Ticker::init();   // after clock init, before sei()

ISR vector bindings always live in the app (or a future board file),
never in portable code: drivers expose `[[gnu::always_inline]]`
handler bodies (`rxc()`, `dre()`, `pit()`), the app binds the vector.

## Upload (Atmel-ICE, UPDI)

`cmake --build --preset <pkg>-release --target <app>-upload` drives
avrdude 8.1 through the Atmel-ICE (`avr_add_app()` in `CMakeLists.txt`;
`-P usb:<serial>` disambiguates when more than one Atmel-ICE is
attached, see `AVR_PROBE_SERIAL`). `brio flash <board> <app>`
does the same, resolved against the bench manifest.

- Plug the cable into the Atmel-ICE **AVR** port, NOT the SAM port.
  The SAM port fails silently: avrdude still sees the ICE on USB, but
  `Vtarget` reads ~1.71 V (parasitic) and the UPDI sign-on returns
  `0xa0` / "initialization failed".
- Programming header: standard 6-pin AVR-ISP (2x3, 2.54 mm). UPDI uses
  three pins: **pin 2 = VCC, pin 5 = UPDI, pin 6 = GND** (1/3/4 unused).
- Flash endurance on UPDI parts is 1000 cycles: uploads and (avoided,
  see below) software breakpoints come out of the same budget.

## Debugging (PyAvrOCD + Atmel-ICE)

Debug sessions use [PyAvrOCD](https://pyavrocd.io) as GDB server with
the toolchain's avr-gdb 17.2, driven by VS Code's generic `cppdbg`
debug adapter (ms-vscode.cpptools) - not a target-specific debug
extension: PyAvrOCD speaks the standard GDB remote protocol, so any
GDB-driving frontend works. Always debug a `*-debug` build (the
release configs are the pure `-Os` build): build the preset
(`cmake --build --preset avr128db48-debug --target <app>`), pick
`<app>` as CMake Tools' launch target, then F5 (`.vscode/launch.json`'s
one generic entry, using `${command:cmake.launchTargetPath}` to follow
whichever target is selected).

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

### Wiring in .vscode/launch.json

PyAvrOCD is launched by cppdbg itself (`debugServerPath` +
`debugServerArgs`), no separate step needed:

```jsonc
{
    "type": "cppdbg",
    "request": "launch",
    "program": "${command:cmake.launchTargetPath}",
    "svdPath": "${workspaceFolder}/avrxt/svd/avr128db48.svd",
    "miMode": "gdb",
    "miDebuggerPath": "/sw/avr/bin/avr-gdb",
    "debugServerPath": "/home/<user>/.local/bin/pyavrocd",
    "debugServerArgs": "--breakpoints hardware -s nop -p 40044 -m all -d avr128db48 -i updi -t atmelice -u <probe serial> -F 24000000 -P 2000",
    "serverStarted": "Listening on port",
    "miDebuggerServerAddress": "localhost:40044",
    "setupCommands": [{ "text": "monitor reset" }]
}
```

(the flags: hardware breakpoints only, no GUI,
GDB server port, let PyAvrOCD manage the relevant fuses, MCU -
`pyavrocd -d '?'` lists them, UPDI, `-t`/`-u` needed with more than one
probe attached, UPDI programming clock in kHz). `-F` (below) stays
inert for a UPDI target either way. `svdPath` is read directly by the
mcu-debug Peripheral Viewer extension from the active session's launch
config (its `svdPathConfig` setting defaults to trying `svdPath` then
`svdFile`, confirmed in the extension's own `package.json`).

`-F` and the CPU clock: for a UPDI target `-F` is inert. PyAvrOCD uses
it only to derive the default JTAG debug clock (megaAVR JTAG sessions)
and to start simavr; the UPDI session sets its own link speed and
never reads it. UPDI has its own clock (`UPDICLKSEL`, independent of
CLK_PER) and reaches memory through the ASI without the CPU, so
breakpoints and stepping work at any CPU rate - a `DynamicClock` app
running at 2 MHz debugs exactly like one at 24 MHz. (debugWIRE, on
classic AVRs, is the interface whose speed is F_CPU/128: that is what
the option exists for.) 24000000 is kept as the boot rate for tidiness.

### Breakpoints: effectively ONE free breakpoint

The UPDI OCD has **2 hardware breakpoints**; single-stepping is native
(costs neither a breakpoint nor flash). GDB borrows one slot for its
temporary breakpoints (`tbreak main` opening every session, `next`
over a call, `finish`, run-to-line). Practical rule: **one breakpoint
in code + one slot free for GDB**; two in code only while stepping
with `step`/`stepi` or `continue`.

Extra breakpoints would normally become SOFTWARE breakpoints, i.e.
flash rewrites. The server therefore runs with `--breakpoints
hardware`: a breakpoint that does not fit is refused ("Cannot insert
breakpoint" at the next continue - delete one and go on) instead of
silently wearing flash. Session escape hatch: `monitor breakpoints
all`; `monitor breakpoints` shows the mode.

### Making line breakpoints bind (GCC 16.x DWARF caveat)

GCC 16.x (verified 16.1 and 16.2) emits a DWARF5 line table with a
duplicate file entry for the main source and file switches around
inlined code; avr-gdb 17.2 and host gdb 15.1 then silently drop part
of it: at any -O level above 0, file:line breakpoints fail with "No
compiled code for line N" while function breakpoints work. Fixed in
`avr_add_app()` (`CMakeLists.txt`): LTO is never added (see "Board and
build" above) and the Debug config is `-Og -g3 -ggdb3 -fno-inline`.

`-fno-inline` removes the mid-function file switches (always_inline
code such as `_delay_ms` stays inlined and keeps its timing); dropping
LTO removes the DWARF partitioning that makes it worse. Residual: a
line whose ONLY content is a call into an always_inline system-header
function (a bare `_delay_ms(500);`) still cannot take a line
breakpoint - break on a neighbouring line. Re-test after any
avr-gcc/avr-gdb rebuild; candidate for an upstream bug report
(minimal repro: any -O1 build of a blink program).

### Peripheral registers (SVD and monitor commands)

Microchip ships no SVD for AVR; PyAvrOCD generates them from the
ATDFs. `avrxt/svd/avr128db48.svd` is committed and wired via `svdPath` in
`.vscode/launch.json` (above), which the mcu-debug Peripheral Viewer
extension reads to populate its PERIPHERALS panel (values read while
stopped; the panel refresh can be flaky - CLion renders the same SVD
better). The server side is always reliable from the Debug Console:

```
monitor info
monitor reset
monitor ioregister PORTA.OUT          # read, with bitfield breakdown
monitor ioregister PORTA.OUT 0x20     # write
monitor ioregister PORTA.*            # whole peripheral
info registers                        # CPU: r0-r31, SREG, SP, PC
p/x PORTA                             # via avr/io.h macros (needs -g3)
```

With the probe-rs VS Code extension enabled, every stop/continue may
spam "unknown custom event" in the Debug Console (it eavesdrops on DAP
custom events regardless of debug adapter type). Harmless; disable
probe-rs per-workspace if it does.
