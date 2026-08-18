# Target: AVR DA/DB (`avrdx/`)

The stratum `lib/brio/src/avrdx/` is everything in brio that knows
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
(built by `/sw/src/build-avr.sh`), used in place via `symlink://`:

    /sw/avr        (symlink to /sw/avr-16.2)

A minimal `package.json` manifest inside the folder makes it usable as
a PlatformIO `symlink://` package; the build script's finalize stage
regenerates it on every rebuild (without it PlatformIO fails with
`MissingPackageManifestError`). PlatformIO's bundled toolchain-atmelavr
and avrdude 7.1 are NOT used: too old for AVR Dx.

The toolchain ships the freestanding libstdc++ (type_traits, concepts,
bit, span, optional, expected, variant - no chrono/charconv/iostream);
brio uses it instead of hand-rolled traits.

**`-std` gotcha.** The platform's `_bare.py` appends `-std=gnu++11`
AFTER the flags added by `extra_scripts` `pre:` scripts, which would
override the `-std=gnu++23` from `tools/pio_flags.py` (last `-std`
wins). This is why `build_unflags` also lists `-std=gnu++11`. Symptom
if it regresses: "'concept' only available with '-std=c++20'".

## Board and build

- `boards/AVR128DB48.json`: custom bare-metal board (128K flash / 16K
  RAM). Its `f_cpu` field is PlatformIO's manifest entry only: the
  `-DF_CPU` it would produce is unflagged (see below), the clock rate is
  `brio::Clock<...>::hz`.
- `platformio.ini`: `platform =` the felias-fogg fork of
  atmelmegaavr (a plain mirror of upstream today, where PyAvrOCD's
  integration will land); toolchain via `symlink://`; Atmel-ICE
  upload; `debug_tool = custom` wiring (below).
- `tools/gen_apps.py`: scans `src/apps/*.cpp` into `apps.ini`, two envs
  per app (`<app>` release, `<app>-debug`); see "Per-app env options"
  below for what an app can add to its own envs.
- `tools/pio_flags.py`: per-language AVR flags, build-type aware:
  `-Os -g` only on release builds, `-std=gnu++23`, IntelliSense
  include paths (skips `[env:native]`).
- `tools/gen_lst.py`: post-build source-interleaved disassembly
  `firmware.lst` + `firmware.map` in `.pio/build/<env>/`.
- Release and debug flags are fully separated: `debug_build_flags =
  -Og -g3 -ggdb3 -fno-inline` applies only to `build_type = debug`,
  i.e. the generated `[env:<app>-debug]` envs. The one global
  concession is `build_unflags = -flto -fuse-linker-plugin`: LTO makes
  the DWARF problem below worse and buys nothing at this firmware
  size (and yields an unreadable `.lst`).
- Do NOT add `-mrelax`: PyAvrOCD refuses ELF files built with it
  (distorted line-number info).

## Per-app env options: `// pio:` header lines

`apps.ini` is generated, so an app cannot be edited into it by hand;
instead an app declares the `[env:]` options it needs as comment lines
in its own header:

    // pio: monitor_speed = 115200
    // pio: monitor_filters = time

`python tools/gen_apps.py` copies every `// pio: <option> = <value>`
line verbatim into BOTH of that app's envs (`<app>`, `<app>-debug`), so
an app-specific fact - a console at a different baud, an extra
`build_flags = -DFOO`, an `upload_speed` - lives next to the code it
belongs to and travels with it. Rules: one line per option (no
multi-line values), any option PlatformIO accepts in an env (it is not
validated here - PlatformIO complains about unknown ones), later lines
of the same option follow INI semantics (last wins). Rerun gen_apps
after adding, removing or changing such a line, then reload the
project (VS Code task "PIO: regen apps").

Then `pio device monitor -e <app>` uses that app's speed;
`clock_console` is the first user (115200, to keep talking down to
2 MHz).

## Clock, delay and timebase

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
`ClockUser` concept (`static void rebase(uint32_t hz)`, checked where
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
64, i.e. CLK_PER >= 16 x baud). Bench-verified with the `clock_console`
app: 24 -> 12 -> 2 MHz under the running console at 115200, 1 MHz
refused as expected.

`F_CPU` is NOT defined in this project: `platformio.ini` unflags the
`-DF_CPU` PlatformIO would pass from the board manifest (by name -
`build_unflags = -DF_CPU`; the platform keeps it as the pair
`("F_CPU", "$BOARD_F_CPU")`, a valued form would not match). The rate
has one truth, `Clock::hz`; avr-libc's `util/delay.h` and
`util/setbaud.h`, which need `F_CPU`, therefore do not compile - on
purpose: nothing may assume a rate the clock type does not state. (In
a build that still defines `F_CPU`, `Clock` static_asserts it equal to
`hz`.) The board JSON's `f_cpu` field is only PlatformIO's manifest
entry. The bench board: **24 MHz crystal on PA0/PA1** (PA0/PA1
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
XOSC32K). Do not enable XOSC32K
(PF0/PF1) unless a 32k crystal is fitted.

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

`pio run -e <app> -t upload` drives avrdude 8.1 through the Atmel-ICE.

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
the toolchain's avr-gdb 17.2. Always debug the `<app>-debug` env
(the plain env is the pure `-Os` release build): from the IDE (Run and
Debug with the `<app>-debug` project env selected) or with
`pio debug -e <app>-debug`.

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

### Wiring in platformio.ini

The atmelmegaavr fork does not integrate PyAvrOCD natively yet, so it
is wired as a custom debug tool:

```ini
debug_tool = custom
debug_port = :40044
debug_server =
    /home/<user>/.local/bin/pyavrocd
    --breakpoints hardware   ; hardware breakpoints only (see below)
    -s nop                   ; no GUI
    -p 40044                 ; GDB server port
    -m all                   ; let PyAvrOCD manage the relevant fuses
    -d avr128db48            ; MCU (pyavrocd -d '?' lists them)
    -i updi
    -t atmelice              ; only needed with several probes attached
    -F 24000000              ; F_CPU in Hz
    -P 2000                  ; UPDI programming clock in kHz
```

(one argument per line in the real file) plus the `debug_init_cmds`
block in `platformio.ini`. Once the fork gains native support this
collapses to `debug_tool = pyavrocd`.

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
compiled code for line N" while function breakpoints work. Fixed by
two `platformio.ini` lines:

```ini
build_unflags     = -flto -fuse-linker-plugin
debug_build_flags = -Og -g3 -ggdb3 -fno-inline
```

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
ATDFs. `svd/avr128db48.svd` is committed and wired with
`debug_svd_path = svd/avr128db48.svd`, which enables the PERIPHERALS
panel in VS Code (values read while stopped; the panel refresh can be
flaky - CLion renders the same SVD better). The server side is always
reliable from the Debug Console:

```
monitor info
monitor reset
monitor ioregister PORTA.OUT          # read, with bitfield breakdown
monitor ioregister PORTA.OUT 0x20     # write
monitor ioregister PORTA.*            # whole peripheral
info registers                        # CPU: r0-r31, SREG, SP, PC
p/x PORTA                             # via avr/io.h macros (needs -g3)
```

With the probe-rs VS Code extension enabled, every stop/continue spams
"unknown custom event" in the Debug Console (it eavesdrops on
PlatformIO's DAP events). Harmless; disable probe-rs per-workspace.
