# Target: AVR DA/DB (`avrdx/`)

The stratum `brio/avrdx/` is everything in brio that knows
`avr/io.h`: clock init, `Pin`, `Uart`, `Spi`, `Twi`, `BasicTicker`,
and `AvrPlatform` (the kernel's `Platform` concept implemented with
AVR intrinsics). Family differences inside DA/DB are handled with
device-macro guards (`CLKCTRL_XOSCHFCTRLA` exists only on DB;
`PORTB`/`PORTE`/`PORTG` depend on the package), so the same headers
build for both. This page is the operational side: toolchain, board,
probe, debugger, and their quirks. The design side (why the ticker
is declaredly AVR, the ISR binding pattern, ...) is in
[../design/](../design/).

Bench MCU: **AVR128DB48** (48-pin, 128 KB flash, 16 KB SRAM),
programmed and debugged with an **Atmel-ICE** over UPDI. Board wiring
and external chips: [../bench.md](../bench.md).

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

- `cmake/avr-mcus.cmake`: package -> avr-gcc mcu name table (128K
  flash / 16K RAM each; `avr128db48` is the default board when an app
  names none, the others are selected by the app's `boards` line
  below). No `-DF_CPU` is ever produced - the clock rate is
  `brio::Clock<...>::hz`, see below.
- `CMakePresets.json`: one configure + build preset pair per AVR128DB
  package x {release, debug} (`avr128db48-release`, `avr128db48-debug`,
  `avr128db28-release`, ...); each points `AVR_MCU` and the toolchain
  file (`cmake/toolchain-avr.cmake`) at that package. Atmel-ICE upload
  is a per-app `<app>-upload` target (`CMakeLists.txt`); debug wiring
  is `.vscode/launch.json` (below).
- App discovery (`CMakeLists.txt`): every `src/apps/*.cpp` with a
  `main()` becomes an executable target, auto-discovered at configure
  time (`file(GLOB ... CONFIGURE_DEPENDS ...)` - a new or removed app
  is picked up on the next configure, no generation step); see
  "Per-app build options" below for what an app can add to its own
  header comment. The glob also covers `../experiments/*/avrdx/*.cpp`:
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
  the real compile already implies). The host project under `test/` is
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
  `test/.clangd` points straight at the host project's own database
  instead of the root AVR one. cpptools' own engine is disabled in the
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
- **FLMAPLOCK is set in every image, by default.** gcc 16 places
  `.rodata` in a 32 KB Flash section reached through the data-space
  window and emits a write of `NVMCTRL.CTRLB.FLMAP` in `.init`; the
  linker script ORs the lock bit into that write when the symbol
  `__flmap_lock` is non-zero, and `avr_add_app()` appends
  `-Wl,--defsym,__flmap_lock=<value>` (default 1) to every AVR link.
  brio's Flash verbs never use the window (ELPM/SPM with a 24-bit
  address only), so the window is a mode nothing uses - and a mode
  nothing uses can only change by accident. An app that must exercise
  the field opts out with `// build: flmap_lock = 0` in its header.
- **Fuses are provisioning, not build output.** The CPU can read them
  and nothing more; only the programmer writes them, so they are a
  property of the chip on the desk and live behind
  `tools/bench.py fuses <board> [name=value ...]` (see
  [../bench.md](../bench.md) for the standing geometry). The one that
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
- **Family compile check**: `tools/check_family.sh` compiles every
  smoke TU in `test/family/` for all eight AVR128 DA/DB packages
  (28/32/48/64 pins, both families) and requires every
  `test/family/neg/` TU to FAIL for the MCUs its `// mcu:` line
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
`build-cmake/apps_manifest.json` at every configure - `tools/bench.py`
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
the desk live in the bench manifest, see [../bench.md](../bench.md).

## Clock, delay and timebase

(The target-independent model - one rate truth, static and dynamic
regimes, the rebase fan-out and its checks - is
[../design/clock.md](../design/clock.md); this section is the AVR
DA/DB realization.)

The main clock is a TYPE, `brio::Clock<source, source_hz, div>`
(`avrdx/clock.hpp`), and `Clock::hz` is the one truth every driver of
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
`ClockUser` concept (a static `rebase(hz)`, checked where
the list is written); `set<hz>()` / `set(hz)` name the new RATE (Hz - the prescaler that produces it is the silicon's detail;
an unreachable rate is a compile error / a false) and fan it out to
every listed user (`Uart`, `Twi`, `Spi` expose `rebase(hz)`; `Uart`
drains its TX at the old rate first) and THEN reprograms the main
prescaler; `hz()` is a value, `is_static` false, `clock_hz(clock)`
reads either kind. `delay_us` takes the runtime path. The RTC/PIT
timebase does not move. The subscription is explicit and checked: a
clocked driver initialized with a DynamicClock that does not list it
fails to compile at its `init(clock)` (`clock_follows<Clock, Driver>()`)
- forgetting a user cannot leave it silently at the old rate. Not while a bus transaction is in flight (ask
the bus AOs); RX bytes during the switch may be garbled. `Uart::
can_baud(hz, baud)` tells whether a rate can still hit a baud (BAUD >=
64, i.e. CLK_PER >= 16 x baud). Bench-verified: 24 -> 12 -> 2 MHz
under a running 115200 console, 1 MHz refused as expected.

`F_CPU` is NOT defined in this project: nothing in `CMakeLists.txt`
ever adds a `-DF_CPU`, since flags here are built up from scratch
rather than inherited from a framework default that would produce one.
The rate has one truth, `Clock::hz`; avr-libc's `util/delay.h` and
`util/setbaud.h`, which need `F_CPU`, therefore do not compile - on
purpose: nothing may assume a rate the clock type does not state. (In
a build that still defines `F_CPU`, `Clock` static_asserts it equal to
`hz`.) The bench board: **24 MHz crystal on PA0/PA1** (PA0/PA1
therefore not GPIO), `Clock<ClockSource::crystal, 24'000'000>`.

`brio::delay_us(clock, us)` (`avrdx/delay.hpp`) busy-waits AT LEAST
`us` microseconds: a folded `__builtin_avr_delay_cycles` when `us` is
a compile-time constant (what `_delay_us` did, minus F_CPU), a 4-cycle
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
attached, see `AVR_PROBE_SERIAL`). `tools/bench.py flash <board> <app>`
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
    "svdPath": "${workspaceFolder}/avrdx/svd/avr128db48.svd",
    "miMode": "gdb",
    "miDebuggerPath": "/sw/avr/bin/avr-gdb",
    "debugServerPath": "/home/<user>/.local/bin/pyavrocd",
    "debugServerArgs": "--breakpoints hardware -s nop -p 40044 -m all -d avr128db48 -i updi -t atmelice -u <probe serial> -F 24000000 -P 2000",
    "serverStarted": "Listening on port",
    "miDebuggerServerAddress": "localhost:40044",
    "setupCommands": [{ "text": "monitor reset" }]
}
```

(the flags mean the same as before: hardware breakpoints only, no GUI,
GDB server port, let PyAvrOCD manage the relevant fuses, MCU -
`pyavrocd -d '?'` lists them, UPDI, `-t`/`-u` needed with more than one
probe attached, UPDI programming clock in kHz). `-F` (below) stays
inert for a UPDI target either way. `svdPath` is read directly by the
mcu-debug Peripheral Viewer extension from the active session's launch
config (its `svdPathConfig` setting defaults to trying `svdPath` then
`svdFile`, confirmed in the extension's own `package.json`) - the same
mechanism Cortex-Debug's `svdFile` used to provide before it moved SVD
support out to this same extension.

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
ATDFs. `avrdx/svd/avr128db48.svd` is committed and wired via `svdPath` in
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
