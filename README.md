# avr128db48_experiments

Bare-metal C++ experiments on an **AVR128DB48**, built with PlatformIO against a
local self-built **avr-gcc 16.2** toolchain (at `/sw/avr`, with avr-gdb 17.2 and
avrdude 8.1) and flashed with an **Atmel-ICE** over UPDI. No Arduino framework.

- General structure and the `lib/brio` shared library inherited from
  [AVR-Multislope](https://github.com/uliano/AVR-Multislope) (toolchain wiring,
  custom board JSON, Atmel-ICE upload, disassembly script, clock/uart/ticker/
  timer/parser modules).
- Multi-app system inherited from [blackpill-experiments](https://github.com/uliano/blackpill-experiments):
  one `main()` per `src/apps/<app>.cpp`, each becoming two envs:
  `[env:<app>]` (release, `-Os`) and `[env:<app>-debug]` (`-Og -g3
  -fno-inline`, for debugging only - debug flags never reach release builds).
- `lib/brio` is the **brio framework**: a small modern-C++ bare-metal layer for
  the AVR **DA/DB** families. Everything resolves at compile time: static
  (monostate) drivers (`brio::Uart<2, brio::Route::alt1>`, `brio::Ticker`,
  `brio::Pin<'F',2>`), C++20 concepts instead of virtual interfaces,
  `brio::print(sink, ...)` variadic formatting, push-based line/command
  parsing (`proto/`), and a QV-style active-object kernel (event queues,
  flat state machines, time events). One flat `brio` namespace; family
  differences handled with device-macro guards. Design docs in `docs/`.

## Quick start

```bash
# Build the default app (blink: LED on PF2)
pio run

# Build + flash over the Atmel-ICE
pio run -e blink -t upload

# Debug build of the same app (separate env and build dir)
pio run -e blink-debug
```

## Debugging (PyAvrOCD + Atmel-ICE over UPDI)

Debug sessions use [PyAvrOCD](https://pyavrocd.io) as GDB server together with
the toolchain's `avr-gdb`. Always debug the `<app>-debug` env (the release
env keeps pure `-Os` production flags): start from the IDE (Run and Debug,
with the `<app>-debug` project env selected) or with
`pio debug -e <app>-debug`.

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
    --breakpoints
    hardware
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

Argument meaning: `--breakpoints hardware` = hardware breakpoints only (see
below); `-s nop` = do not spawn a GUI; `-p` = GDB server port;
`-m all` = let PyAvrOCD manage the relevant fuses; `-d` = MCU
(`pyavrocd -d '?'` lists all supported ones); `-i` = physical interface;
`-t` = probe, only needed with several probes attached; `-F` = F_CPU in Hz;
`-P` = UPDI programming clock in kHz.

plus the `debug_init_cmds` block already present in `platformio.ini`. Once the
platform fork gains native support, all of this collapses to
`debug_tool = pyavrocd`.

### Breakpoints: effectively ONE free breakpoint

The UPDI OCD provides **2 hardware breakpoints** and single-stepping is a
native OCD operation (it consumes neither a breakpoint nor flash). GDB
however silently borrows one slot for its temporary breakpoints: the
`tbreak main` that opens every PlatformIO session, `next` over a function
call, `finish`, run-to-line. The practical rule is therefore:

- **1 breakpoint placed in code + 1 slot left free for GDB**;
- 2 breakpoints in code are fine only while stepping with `step`/`stepi`
  or `continue` (no `next` over calls / `finish` in that state).

Extra breakpoints beyond the hardware slots would normally become SOFTWARE
breakpoints, i.e. flash rewrites on a die whose guaranteed endurance is
1000 cycles (same budget the uploads come out of). The server is therefore
started with `--breakpoints hardware`: a breakpoint that does not fit is
refused (GDB reports "Cannot insert breakpoint" at the next continue/next -
delete one and go on) instead of silently wearing flash. To allow software
breakpoints for one session, type `monitor breakpoints all` in the Debug
Console; `monitor breakpoints` shows the current mode.

### Making line breakpoints actually bind

GCC 16.x emits a DWARF5 line table (duplicate file entry for the main source,
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
   (or the VS Code task **PIO: regen apps**), then reload the PlatformIO
   project. This creates BOTH `[env:<name>]` (release) and
   `[env:<name>-debug]` (debug).
3. Build/upload it: `pio run -e <name> -t upload`; debug it:
   `pio debug -e <name>-debug`.

Shared code goes into `lib/brio/src/`: any header included from an app is
compiled and linked automatically by PlatformIO's Library Dependency Finder,
no `build_src_filter` changes needed.

## Apps

| App       | What it does                                                               |
|-----------|----------------------------------------------------------------------------|
| `blink`   | The minimal kernel app: Blinker toggles **PF2** on its periodic time event, Supervisor cycles the period (500/250/100 ms) every 3 s by posting a command - no delay loops, CPU in IDLE sleep between events |
| `console` | Interactive command console @ 460800 (HELP, LED, UPTIME, ERR): SerialPort (RX bytes -> line events, ping-pong buffers), Console (parse/route/reply), Blinker (heartbeat FSM + LED commands via posted events), zero polling |
| `spi_loopback` | SPI stack test: jumper **PA4(MOSI) -> PA5(MISO)**, a full-duplex 8-byte transaction per second through SpiBus + the Spi<0> engine, verdict on the serial console (no jumper = FAIL 0xFF, by design) |
| `display_id` | Reads the display controller's DCS registers (RDDPM, RDDID, ID4, 0xBF device code) and prints the raw answers: the aliveness/identification probe that unmasked the 3.5" module as an **ILI9481** |
| `display_fill` | ILI9481 full-screen solid fill cycling red/green/blue (18-bit pixels, CASET/PASET + RAMWR/3C row writes, INVON for the 9481 panel polarity) |
| `spi_duo` | **Two devices, one arbitrated bus**: ILI9481 fill (960-byte rows @ 6 MHz) + XPT2046 touch polling (3-byte conversions @ 1.5 MHz, T_CS on PD5) through the same SpiBus; touch steers the fill palette, per-request clock switching |

## Hardware

- MCU: AVR128DB48 (48-pin, 128 KB flash, 16 KB SRAM)
- Supply: jumper-selectable **3.3 V / 5 V**; VDDIO2 is powered, so PORTC
  (the MVIO domain) is usable - and could one day talk 3.3 V logic while
  the rest of the chip runs 5 V, no level shifters.
- Bench (SPI/I2C experiments, **3.3 V rail** - the 3.5" module has no
  level shifter on the display signals): SPI0 on PA4(MOSI)/PA5(MISO)/
  PA6(SCK); PD0=CS display, PD1=RS/DC, PD2=RST display, PD3=CS MCP3550,
  PD4 reserved for the module's SD_CS, PD5=T_CS (XPT2046 touch, same
  shared bus, PEN unused); TWI0 reserved on PA2(SDA)/PA3(SCL)
  with 4.7k pull-ups for the MCP47CVB22 (dual 12-bit DAC, addr 0x60).
  Display: 3.5" red module **HST035003-A**, controller **ILI9481**
  (320x480, SPI = 18-bit pixels only, panel needs INVON), XPT2046
  resistive touch on board (its U2; U1 is the LDO), BL tied high.
  The older 2.4" module (frozen white through every protocol despite
  verified signals) is parked as defective-suspect.
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
