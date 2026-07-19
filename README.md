# avr128db48_experiments

Bare-metal C++ experiments on an **AVR128DB48**, built with PlatformIO against a
local self-built **avr-gcc 16.1** toolchain (at `/sw/avr`, with avr-gdb 17.2 and
avrdude 8.1) and flashed with an **Atmel-ICE** over UPDI. No Arduino framework.

- General structure inherited from [AVR-Multislope](https://github.com/uliano/AVR-Multislope)
  (toolchain wiring, custom board JSON, Atmel-ICE upload, disassembly script).
- Multi-app system inherited from [blackpill-experiments](https://github.com/uliano/blackpill-experiments):
  one `main()` per `src/apps/<app>.cpp`, each becoming its own `[env:<app>]`.
- Cloned from [avr128db28_experiments](https://github.com/uliano/avr128db28_experiments)
  (same structure and debug setup, retargeted to the 48-pin part; apps trimmed
  down to `blink`).

## Quick start

```bash
# Build the default app (blink: LED on PF2)
pio run

# Build + flash over the Atmel-ICE
pio run -e blink -t upload
```

## Debugging (PyAvrOCD + Atmel-ICE over UPDI)

Debug sessions use [PyAvrOCD](https://pyavrocd.io) as GDB server together with
the toolchain's `avr-gdb`. Start them from the IDE (Run and Debug) or with
`pio debug -e <app>`.

### One-time host setup

```bash
# The GDB server (lands in ~/.local/bin/pyavrocd). Installed as a uv tool on
# a uv-managed CPython pinned to 3.12, so it survives distro Python bumps
# (rolling-release proof). Without --python-preference only-managed, uv would
# silently bind the venv to the DISTRO python if a matching one exists.
uv tool install --python-preference only-managed --python 3.12 pyavrocd
# (equivalent, distro-python-bound alternative: pipx install pyavrocd)

# udev rules for the EDBG probes (Atmel-ICE, PICkit 4, Snap, ...),
# then unplug/replug the probe:
wget https://pyavrocd.io/99-edbg-debuggers.rules
sudo cp 99-edbg-debuggers.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

### Enabling PyAvrOCD in platformio.ini

The `platform-atmelmegaavr` fork referenced by `platform =` does not integrate
PyAvrOCD natively yet (only the classic-AVR `platform-atmelavr` fork does), so
this repo wires it manually as a custom debug tool:

```ini
debug_tool = custom
debug_port = :40044
debug_server =
    /home/<user>/.local/bin/pyavrocd
    -s
    nop
    -p
    40044
    -m
    all
    -d
    avr128db48
    -i
    updi
    -t
    atmelice
    -F
    24000000
    -P
    2000
```

Argument meaning: `-s nop` = do not spawn a GUI; `-p` = GDB server port;
`-m all` = let PyAvrOCD manage the relevant fuses; `-d` = MCU
(`pyavrocd -d '?'` lists all supported ones); `-i` = physical interface;
`-t` = probe, only needed with several probes attached; `-F` = F_CPU in Hz;
`-P` = UPDI programming clock in kHz.

plus the `debug_init_cmds` block already present in `platformio.ini`. Once the
platform fork gains native support, all of this collapses to
`debug_tool = pyavrocd`.

### Making line breakpoints actually bind

GCC 16.1 emits a DWARF5 line table (duplicate file entry for the main source,
file switches around inlined code) that both avr-gdb 17.2 and host gdb drop
parts of: with plain `-Og` (or any -O level above 0) file:line breakpoints
fail with "No compiled code for line N", while function breakpoints work.
Two platformio.ini lines fix it:

```ini
build_unflags     = -flto -fuse-linker-plugin
debug_build_flags = -Og -g3 -ggdb3 -fno-inline
```

`-fno-inline` removes the mid-function file switches (always_inline code such
as `_delay_ms` stays inlined and keeps correct timing); dropping LTO removes
the DWARF partitioning that makes things worse. Remaining caveats:

- a line whose ONLY content is a call to an always_inline system-header
  function (e.g. a bare `_delay_ms(500);`) still cannot take a line
  breakpoint: break on a neighbouring line instead;
- never add `-mrelax`: PyAvrOCD refuses ELF files built with it.

### Peripheral registers (SVD and monitor commands)

Microchip does not ship SVD files for AVR; the PyAvrOCD project generates them
from the official ATDFs for every supported MCU. Grab the right one from
<https://github.com/felias-fogg/PyAvrOCD/tree/main/svd> (this repo commits
`svd/avr128db48.svd`) and point PlatformIO at it:

```ini
debug_svd_path = svd/avr128db48.svd
```

This enables the **PERIPHERALS** panel in the VS Code Run and Debug sidebar
(values are read while the target is stopped; the PIO panel refresh can be
flaky - CLion renders the same SVD better). From the Debug Console the server
side is always reliable:

```
monitor ioregister PORTA.OUT          # read, with bitfield breakdown
monitor ioregister PORTA.OUT 0x20     # write
monitor ioregister PORTA.*            # whole peripheral
info registers                        # CPU: r0-r31, SREG, SP, PC
p/x PORTA                             # via avr/io.h macros (needs -g3)
```

Note: with the probe-rs VS Code extension enabled, every stop/continue spams
"unknown custom event" errors in the Debug Console (it eavesdrops on
PlatformIO's custom DAP events). Harmless; disable probe-rs per-workspace.

## Adding an experiment

1. Create `src/apps/<name>.cpp` with its own `int main()`.
2. Regenerate the env list: `python tools/gen_apps.py`
   (or the VS Code task **PIO: regen apps**), then reload the PlatformIO project.
3. Build/upload it: `pio run -e <name> -t upload`.

## Apps

| App       | What it does                                             |
|-----------|----------------------------------------------------------|
| `blink`   | Toggles an LED on **PF2** at ~1 Hz (24 MHz internal osc) |

## Hardware

- MCU: AVR128DB48 (48-pin, 128 KB flash, 16 KB SRAM)
- Programmer/debugger: Atmel-ICE over UPDI. **Plug the cable into the Atmel-ICE
  `AVR` port, NOT the `SAM` port.** The SAM port is for ARM/SAM and fails
  silently: avrdude still sees the ICE on USB, but `Vtarget` reads ~1.71 V
  (parasitic) and the UPDI sign-on returns `0xa0` / "initialization failed".
- Programming header: standard 6-pin AVR-ISP (2x3, 2.54 mm). For UPDI only three
  pins are used: **pin 2 = VCC, pin 5 = UPDI, pin 6 = GND** (1/3/4 unused).
- Clock: **24 MHz crystal on PA0/PA1** (XOSCHF, DB-only feature), with
  automatic fallback to the internal OSCHF @ 24 MHz if the crystal fails to
  start - `init_clock_24mhz()` returns which source won. PA0/PA1 are therefore
  NOT available as GPIO. RTC on internal OSC32K (no 32 kHz crystal).
- `blink` wiring: PF2 -> ~330 ohm -> LED -> GND
