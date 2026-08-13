# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Interaction

Claude will interact with the user in italian but all the edits in the files, including comments, will be done in english.

## Allowed text symbols

Only ASCII <=127 characters will be allowed in the documents.

## Project Overview

Bare-metal C++ experiments on an **AVR128DB48** (48-pin, 128 KB flash, 16 KB
SRAM), programmed/debugged with an **Atmel-ICE** over UPDI. No Arduino
framework: just avr-gcc + avr-libc.

Long-term goal: grow `lib/core` into a small **modern-C++ framework covering
at least the AVR DA and DB families**. Design decided on 2026-07-21 (after a
full critical review of the AVR-Multislope heritage):

- **everything resolves at compile time**: peripheral drivers are static
  (monostate) class templates (`Uart<2, Route::alt1>`, `BasicTicker<1024>`),
  services are templated on the transport and constrained by concepts
  (`ByteSink`/`ByteSource` in stream.hpp) - NO virtual interfaces, no
  runtime singletons (the old ByteStream base class is gone);
- **drivers move bytes, they do not format**: text output is
  `dx::print(sink, ...)` in print.hpp, variadic free functions over any
  ByteSink; new types become printable via a `print_one(sink, value)`
  overload (found by ADL). print BLOCKS until the sink accepts (no silent
  truncation) - so print only after init + sei();
- **protocol layer is push-based**: proto/line_parser.hpp's LineAssembler
  is fed bytes and returns completed lines - no reference to any transport,
  host-testable; parsers/router are plain static code;
- **one flat namespace `dx`**, no nesting (namespaces prevent clashes, they
  are not architecture); apps may `using namespace dx;`, headers always
  qualify. Types PascalCase, functions/constants snake_case;
- **monostate drivers double as zero-cost tags**: `constexpr Serial serial;`
  is an empty object usable as `print(serial, ...)` argument;
- the toolchain ships the **freestanding libstdc++** (type_traits, concepts,
  bit, span, optional, expected, ... - no chrono/charconv/iostream): use it
  instead of hand-rolled traits;
- the project standard is **gnu++23** (set in tools/pio_flags.py); C++26
  features already implemented by gcc 16 may be used when genuinely needed
  (bump the -std flag in that case). C++23 idioms in active use:
  `static_assert(false, ...)` in discarded if-constexpr branches turns
  "peripheral/port not present on this device" into a clear compile error
  (pin.hpp, uart.hpp), `std::to_underlying` for enum codes (clock.hpp).

Family differences are handled with device-macro guards (e.g.
`CLKCTRL_XOSCHFCTRLA` exists only on DB, `PORTB`/`PORTE`/`PORTG` depend on
the package) so the same headers build for both; the USART pin/PORTMUX
table inside uart.hpp is the designated seed of a future per-family device
header.

The general structure (toolchain wiring, custom board JSON, Atmel-ICE upload,
the disassembly post-build script) and the `lib/core` library are inherited
from [uliano/AVR-Multislope](https://github.com/uliano/AVR-Multislope).

The **multi-app** system is inherited from
[uliano/blackpill-experiments](https://github.com/uliano/blackpill-experiments):
every `src/apps/<app>.cpp` has its own `main()` and becomes its own pair of
envs `[env:<app>]` / `[env:<app>-debug]`. Shared code lives in `lib/core`
(a PlatformIO private library: the LDF compiles and links it automatically
for every env whose app includes one of its headers - no src_filter entry).

There are currently NO external peripherals on the board except the serial
link (CH340 on USART2 ALT1, PF4/PF5). Device drivers for external chips will
be (re)added one by one when the corresponding hardware gets wired up.

## Toolchain

Self-built avr-gcc 16.2 + avr-libc + avr-gdb 17.2 + avrdude 8.1 (built by
/sw/src/build-avr.sh), used in place via `symlink://`:

    /sw/avr        (symlink to /sw/avr-16.2)

A minimal `package.json` manifest inside /sw/avr-16.2 makes the folder usable
as a PlatformIO `symlink://` package; build-avr.sh's finalize stage generates
it on every (re)build, so toolchain upgrades keep it (without it, PlatformIO
fails with MissingPackageManifestError). PlatformIO's bundled toolchain-atmelavr /
avrdude 7.1 are NOT used (too old for AVR-Dx).

## Debugging (PyAvrOCD + Atmel-ICE over UPDI)

Debugging uses [PyAvrOCD](https://pyavrocd.io) as GDB server (installed via
`uv tool install --python-preference only-managed --python 3.12 pyavrocd`,
exposed at ~/.local/bin/pyavrocd, venv on a uv-managed CPython 3.12 so it is
independent of the distro Python) together with the toolchain's avr-gdb 17.2. It is wired in platformio.ini as `debug_tool = custom`
(server on port 40044), mirroring felias-fogg's platform-atmelavr wiring;
switch to `debug_tool = pyavrocd` once the integration lands in the
platform-atmelmegaavr fork referenced by `platform =`.

- Always debug the `<app>-debug` env: start from the IDE (Run and Debug,
  after selecting the `<app>-debug` project env) or with
  `pio debug -e <app>-debug`. The plain `<app>` env is the release build.
- udev rules for the EDBG probes: /etc/udev/rules.d/99-edbg-debuggers.rules
  (unplug/replug the probe after installing them).
- Do NOT add `-mrelax` to the build flags: PyAvrOCD refuses ELF files built
  with it (distorted line-number info).
- Useful GDB console commands: `monitor info`, `monitor reset`,
  `monitor ioregister <name>` (read/write an I/O register by name).
- Breakpoints: the UPDI OCD has 2 hardware breakpoints; GDB borrows one for
  its temporary breakpoints (tbreak main, next over a call, finish), so plan
  for ONE free user breakpoint. Single-stepping is native OCD, costs nothing.
  The server runs with `--breakpoints hardware` (set in debug_server): extra
  breakpoints are refused ("Cannot insert breakpoint") instead of becoming
  software breakpoints that rewrite flash (1000-cycle endurance on UPDI
  parts). Session escape hatch: `monitor breakpoints all`.

### Toolchain DWARF caveat (GCC 16.x vs gdb)

GCC 16.x (verified on 16.1 and 16.2) emits a DWARF5 line table with a
duplicate file entry for the main source and switches file mid-sequence
around inlined code. Both avr-gdb 17.2
and host gdb 15.1 then silently drop part of the line table: file:line
breakpoints fail to bind ("No compiled code for line N") at ANY -O level
above 0, while function breakpoints (e.g. `tbreak main`) still work. This is
why platformio.ini sets:

- `build_unflags = -flto -fuse-linker-plugin` (LTO makes it worse and buys
  nothing at this firmware size), and
- `debug_build_flags = -Og -g3 -ggdb3 -fno-inline` (no inlining -> no
  mid-function file switches -> line breakpoints bind; always_inline code
  such as `_delay_ms` stays inlined and keeps correct timing).

Debug and release flags are fully separated: `debug_build_flags` only applies
to `build_type = debug`, i.e. to the generated `[env:<app>-debug]` envs, and
tools/pio_flags.py adds `-Os -g` only to release builds (it checks
`env.GetBuildType()`). Release firmware therefore never carries `-Og` or
`-fno-inline`; the only global concession is the LTO disable, which is a
deliberate choice (unreadable .lst, zero size benefit) rather than a debug
leftover.

Residual limitation: a line whose only content is a call into an
always_inline system-header function (e.g. a bare `_delay_ms(500);`) still
cannot take a line breakpoint; break on a neighbouring line instead. Worth
re-testing after any avr-gcc / avr-gdb rebuild; candidate for an upstream
bug report (minimal repro: any -O1 build of blink.cpp).

## Multi-app workflow

Each experiment is one `src/apps/<name>.cpp` with its own `main()`. The env
blocks are AUTO-GENERATED into `apps.ini`, TWO per app:

- `[env:<name>]` - release build (`-Os`), for production uploads;
- `[env:<name>-debug]` - `build_type = debug` (`-Og -g3 -ggdb3 -fno-inline`
  via `debug_build_flags`), for every debug session.

```bash
# After adding/removing a file in src/apps/, regenerate the env list:
python tools/gen_apps.py
# (or the VS Code task "PIO: regen apps"), then reload the PlatformIO project.
```

`apps.ini` is committed so a fresh clone already has the envs.

## Build and Development Commands

```bash
# Build the default app (blink, release)
pio run

# Build a specific app (release / debug)
pio run -e blink
pio run -e blink-debug

# Build and upload via Atmel-ICE (UPDI)
pio run -e blink -t upload

# Debug (builds and flashes the -debug env, then attaches)
pio debug -e blink-debug

# Clean
pio run -t clean
```

`pio run` (build only) needs NO hardware connected; the Atmel-ICE is only
touched on `-t upload`.

Upload gotcha: connect the Atmel-ICE **AVR** port (not the SAM port). Wrong port
-> avrdude still detects the ICE on USB but `Vtarget` reads ~1.71 V and the UPDI
sign-on fails with `Bad response to AVR sign-on command: 0xa0`.

## Layout

```
platformio.ini          base [env], toolchain, Atmel-ICE upload, debug wiring
apps.ini                generated: [env:<app>] + [env:<app>-debug] per app
boards/AVR128DB48.json   custom bare-metal board (128K flash / 16K RAM)
tools/gen_apps.py        scans src/apps/*.cpp -> apps.ini
tools/pio_flags.py       per-language AVR flags (build-type aware) +
                         IntelliSense include paths
tools/gen_lst.py         post-build: firmware.lst (disassembly) + firmware.map
src/apps/<app>.cpp       one main() per experiment
lib/core/                the dx framework (auto-linked by the LDF), all in
                         namespace dx, header-only:
  src/stream.hpp           ByteSink / ByteSource / ByteTransport concepts
  src/clock.hpp            DA/DB clock init: init_clocks() probing generic +
                           init_clock_24mhz() deterministic DB crystal path
  src/pin.hpp              Pin<'A',5> compile-time GPIO (VPORT fast paths)
  src/ring.hpp             Ring<T,size> SPSC ring (index type auto-derived)
  src/uart.hpp             Uart<n, Route, rx, tx> static interrupt-driven
                           byte transport, RXDATAH error counters
  src/print.hpp            print(sink, ...) variadic formatting, hex/fixed/
                           sci wrappers, crlf; extend via print_one + ADL
  src/ticker.hpp           BasicTicker<tps> static RTC/PIT timebase
                           (alias Ticker = BasicTicker<1024>)
  src/timer.hpp            Timer<Millis|Secs|Ticks> soft timers, no vtables,
                           member callbacks via dx::bind<&Class::method>
  src/proto/line_parser.hpp  LineAssembler (push) + console/SCPI parsers +
                           CommandRouter<Sink>
```

## Build Artifacts

- ELF / HEX / MAP / LST: `.pio/build/<env>/`
- `firmware.lst`: source-interleaved disassembly (from tools/gen_lst.py)

## Clock note

The board has a **24 MHz crystal on PA0/PA1** (XOSCHF, a DB-family feature;
PA0/PA1 are therefore not available as GPIO). `lib/core/src/clock.hpp`
(ported from AVR-Multislope's src/clocks.h) offers two entry points:

- `init_clocks()` - generic DA/DB probing init: OSCHF 24 MHz baseline, then
  probes DB crystal on PA0/PA1, EXTCLK on PA0, and a 32k crystal on PF0/PF1
  (with OSCHF autotune); returns a `ClockInitCode` describing what it found.
  For boards whose clock fixture is unknown.
- `init_clock_24mhz()` - deterministic DB path used by the apps in THIS repo:
  the crystal is a known fixture, start it (SELHF_XTAL, FRQRANGE 24M,
  CSUTHF 4k) and switch CLK_PER, falling back to the internal OSCHF @ 24 MHz.
  Returns true when running from the crystal. It does NOT touch XOSC32K.

Each app calls one of them as the first line of main(). `F_CPU=24000000`
comes from the board JSON and is correct for both sources.

There is NO 32.768 kHz crystal on this board: `Ticker::init()` selects the
RTC clock automatically (XOSC32K only if the clock init reports it running,
internal OSC32K otherwise). Do NOT enable XOSC32K (PF0/PF1) unless a 32k
crystal is actually fitted; the serial link uses USART2 ALT1 on PF4/PF5, so
PF0/PF1 are free.

## Ticker / timers note

`dx::Ticker` is a STATIC (monostate) class template - `BasicTicker<tps>`
with `using Ticker = BasicTicker<1024>` - not a runtime singleton: all state
is C++17 `static inline` in .bss, all methods are static, header-only, no
init order issues and no pointer indirection in the ISR. Apps wire it as:

    ISR(RTC_PIT_vect) { dx::Ticker::pit(); }
    ...
    dx::Ticker::init();   // after clock init, before sei()

Two bugs of the original AVR-Multislope ticker are fixed here (worth
back-porting): the H/L union word order made ticks() advance by 65536 per
tick, and millis() ran 0.7% fast because window position 0x00 was not
skipped. See the NOTE in lib/core/src/ticker.hpp.

`dx::Timer<Unit>` has no virtual functions (the old TimerBase vtable and the
operator delete stubs are gone); member callbacks use the compile-time
trampoline `dx::bind<&Class::method>(&object)`.

## Toolchain std gotcha

The platform's `_bare.py` appends `-std=gnu++11` AFTER the flags added by
extra_scripts `pre:` scripts, so it would override the `-std=gnu++23` from
tools/pio_flags.py (the last -std on the command line wins). This is why
`build_unflags` also lists `-std=gnu++11`. Symptom if it regresses:
"'concept' only available with '-std=c++20'" errors from stream.hpp.
