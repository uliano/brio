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

The general structure (toolchain wiring, custom board JSON, Atmel-ICE upload,
the disassembly post-build script) is inherited from
[uliano/AVR-Multislope](https://github.com/uliano/AVR-Multislope).

The **multi-app** system is inherited from
[uliano/blackpill-experiments](https://github.com/uliano/blackpill-experiments):
every `src/apps/<app>.cpp` has its own `main()` and becomes its own
`[env:<app>]`. Shared code lives in `src/common/`.

## Toolchain

Self-built avr-gcc 16.1 + avr-libc + avr-gdb 17.2 + avrdude 8.1 (built by
/sw/src/build-avr.sh), used in place via `symlink://`:

    /sw/avr        (symlink to /sw/avr-16.1)

A minimal `package.json` manifest inside /sw/avr-16.1 makes the folder usable
as a PlatformIO `symlink://` package. PlatformIO's bundled toolchain-atmelavr /
avrdude 7.1 are NOT used (too old for AVR-Dx).

## Debugging (PyAvrOCD + Atmel-ICE over UPDI)

Debugging uses [PyAvrOCD](https://pyavrocd.io) as GDB server (installed via
`uv tool install --python-preference only-managed --python 3.12 pyavrocd`,
exposed at ~/.local/bin/pyavrocd, venv on a uv-managed CPython 3.12 so it is
independent of the distro Python) together with the toolchain's avr-gdb 17.2. It is wired in platformio.ini as `debug_tool = custom`
(server on port 40044), mirroring felias-fogg's platform-atmelavr wiring;
switch to `debug_tool = pyavrocd` once the integration lands in the
platform-atmelmegaavr fork referenced by `platform =`.

- Start from the IDE (Run and Debug) or with `pio debug -e <app>`.
- udev rules for the EDBG probes: /etc/udev/rules.d/99-edbg-debuggers.rules
  (unplug/replug the probe after installing them).
- Do NOT add `-mrelax` to the build flags: PyAvrOCD refuses ELF files built
  with it (distorted line-number info).
- Useful GDB console commands: `monitor info`, `monitor reset`,
  `monitor ioregister <name>` (read/write an I/O register by name).

### Toolchain DWARF caveat (GCC 16.1 vs gdb)

GCC 16.1 emits a DWARF5 line table with a duplicate file entry for the main
source and switches file mid-sequence around inlined code. Both avr-gdb 17.2
and host gdb 15.1 then silently drop part of the line table: file:line
breakpoints fail to bind ("No compiled code for line N") at ANY -O level
above 0, while function breakpoints (e.g. `tbreak main`) still work. This is
why platformio.ini sets:

- `build_unflags = -flto -fuse-linker-plugin` (LTO makes it worse and buys
  nothing at this firmware size), and
- `debug_build_flags = -Og -g3 -ggdb3 -fno-inline` (no inlining -> no
  mid-function file switches -> line breakpoints bind; always_inline code
  such as `_delay_ms` stays inlined and keeps correct timing).

Residual limitation: a line whose only content is a call into an
always_inline system-header function (e.g. a bare `_delay_ms(500);`) still
cannot take a line breakpoint; break on a neighbouring line instead. Worth
re-testing after any avr-gcc / avr-gdb rebuild; candidate for an upstream
bug report (minimal repro: any -O1 build of blink.cpp).

## Multi-app workflow

Each experiment is one `src/apps/<name>.cpp` with its own `main()`. The list of
`[env:<name>]` blocks is AUTO-GENERATED into `apps.ini`:

```bash
# After adding/removing a file in src/apps/, regenerate the env list:
python tools/gen_apps.py
# (or the VS Code task "PIO: regen apps"), then reload the PlatformIO project.
```

`apps.ini` is committed so a fresh clone already has the envs.

## Build and Development Commands

```bash
# Build the default app (blink)
pio run

# Build a specific app
pio run -e blink

# Build and upload via Atmel-ICE (UPDI)
pio run -e blink -t upload

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
platformio.ini          base [env], toolchain, Atmel-ICE upload
apps.ini                generated: one [env:<app>] per src/apps/<app>.cpp
boards/AVR128DB48.json   custom bare-metal board (128K flash / 16K RAM)
tools/gen_apps.py        scans src/apps/*.cpp -> apps.ini
tools/pio_flags.py       per-language AVR flags + IntelliSense include paths
tools/gen_lst.py         post-build: firmware.lst (disassembly) + firmware.map
src/apps/<app>.cpp       one main() per experiment
src/common/              shared code: pin, clock, uart+ring+bytestream, ticker,
                         spi (SPI1), ads131m02, mcp3550 (ADC drivers)
```

## Build Artifacts

- ELF / HEX / MAP / LST: `.pio/build/<env>/`
- `firmware.lst`: source-interleaved disassembly (from tools/gen_lst.py)

## Clock note

The board has a **24 MHz crystal on PA0/PA1** (XOSCHF, a DB-family feature;
PA0/PA1 are therefore not available as GPIO). Each app calls
`init_clock_24mhz()` (src/common/clock.hpp) as the first line of main(): it
starts the crystal (SELHF_XTAL, FRQRANGE 24M, CSUTHF 4k - same proven
constants as AVR-Multislope's src/clocks.h, but deterministic instead of
probing) and switches CLK_PER to it, falling back to the internal OSCHF @
24 MHz if the crystal does not start. The function returns true when running
from the crystal. `F_CPU=24000000` comes from the board JSON and is correct
for both sources.

There is NO 32.768 kHz crystal: the RTC/ticker uses the internal OSC32K -
init_ticker() falls back to it automatically. Do NOT enable XOSC32K (the
32 kHz crystal input, PF0/PF1) unless a 32k crystal is actually fitted; on
this board the serial link uses USART2 ALT1 on PF4/PF5, so PF0/PF1 are free.
