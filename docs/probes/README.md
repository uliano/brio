# Probes

The probes brio flashes and debugs through, one page each, for a
programmer who has one on the desk or is choosing one for a board. A
probe page says what the probe is and which targets it reaches, how
`bin/brio` drives it, and the traps found on the way: a transport that
desynchronizes, a firmware that does not know a part, a register a
debugger leaves set. This page is the index of those pages and the
table of the flash mechanisms the tool knows, which is the `"type"` of
a board's `programmer` entry in the bench manifest
([../boards/README.md](../boards/README.md), "How a board joins the
bench").

| Programmer kind | Probe | Wire | Boards |
|-----------------|-------|------|--------|
| `atmelice_updi`, `pickit4_updi` | an EDBG-class probe, driven by avrdude | UPDI | the AVR DA, DB and EA boards |
| `pickit5_updi` | an MPLAB PICkit 5, driven by avrdude | UPDI | the AVR DA, DB and EA boards |
| `serialupdi` | a USB-serial adapter, driven by avrdude | UPDI | the AVR DA, DB and EA boards |
| `openocd_cmsisdap` | an [Atmel-ICE](atmel-ice.md) (the HID backend) or a [Raspberry Pi Debug Probe](raspberry-pi-debug-probe.md) (the USB bulk backend, with a UART bridge on board), driven by OpenOCD | SWD, multidrop on the RP2040 | the SAM C21 board; the RP2040 boards; the RP2350 board |
| `openocd_stlink` | an [ST-LINK](st-link.md), a Nucleo's or a Discovery's own or a standalone STLINK-V3, driven by OpenOCD | SWD | the STM32G0 Nucleos, the STM32F4 boards |
| `stlink_msd` | the ST-LINK's mass-storage flasher | a USB drive | the STM32G0 Nucleos, when the debug port does not answer |
| `openocd_jlink` | a SEGGER J-Link, driven by OpenOCD | SWD | the SAM C21 board, the STM32G0 Nucleos, the STM32F4 boards |
| `bmp` | a Black Magic Probe, driven by the toolchain's gdb through the probe's own gdb server (the manifest's `ARM_GDB`) | SWD | the boards OpenOCD writes over SWD, the RP2350 board excepted |
| `wch_link` | a [WCH-Link](wch-link.md), driven by WCH's OpenOCD fork | SDI, one wire on the CH32V00x, two wires on the CH32V203, CH32V303 and CH32X035 | the CH32V00x, CH32V203, CH32V303 and CH32X035 boards |

On the RP2350 the board type, not the programmer kind, selects the
flash mechanism (`rp2350_openocd` in `cli/bench/common.py`'s
`BOARD_TYPES`): the same `openocd_cmsisdap` entry as an RP2040's, run
by Raspberry Pi's OpenOCD fork (SWD, ADIv6) as two sessions, a rescue over the debug
port alone and then the programming
([raspberry-pi-debug-probe.md](raspberry-pi-debug-probe.md)).

`brio flash` drives every UPDI probe but the SerialUPDI adapter at one
UPDI clock, 500 kHz (avrdude's `-B`): the Atmel-ICE's own default, five
times the PICkit 4's (`UPDI_CLOCK` in `cli/bench/flash.py`). An
`-upload` target gives the same clock: `-B 500kHz` to every probe on
USB but the Atmel-ICE, whose own it is.

## Rules that hold for every probe

- **A probe serves one client at a time.** Close the debug session
  before flashing.
- **A probe is addressed by its own USB serial**, and needs it only
  when two of a kind are attached.
- **Every OpenOCD flash ends with `** Verified OK **` or it did not
  happen.** A failed program leaves a partly erased image that may
  still print another app's banner, or nothing.

## Which OpenOCD

Which OpenOCD build drives a board is a property of the target, not of
the probe. The bench has three under `/sw`, each named by absolute path
in the manifest: the 0.12.0 release for every SWD path of the SAM C21
and STM32 targets and for the Pico; a build from git for the RP2040 boards
whose flash chip the release does not identify; and Raspberry Pi's fork
for the RP2350, the only one of the three that can reach that chip at
all ([../rp2350/README.md](../rp2350/README.md) says why, and
`cli/bench/flash.py` is where each is chosen). WCH's fork, a fourth
build, is the WCH-Link's alone.

## The probe of an -upload target

A build directory's `<app>-upload` target programs through the probe
its configure names. `cmake/probe.cmake` reads the same four cache
variables in every family and writes the probe's half of the command;
the family's recipe writes the rest (the part, the target script, the
resets):

- `BRIO_PROBE`, the probe's kind, from the first table below; empty
  means the family's own probe.
- `BRIO_PROBE_SERIAL`, the probe's USB serial, which picks one probe
  when several of the kind are attached; empty means the only one.
- `BRIO_PROBE_PORT`, the serial port of a probe that is one
  (`serialupdi`, `bmp`). It names the probe, so those two kinds require
  it and refuse a USB serial, and every other kind refuses a port.
- `BRIO_PROBE_BAUD`, the SerialUPDI adapter's rate, 230400 when empty;
  every other kind refuses it.

A kind the family does not drive stops the configure with the list of
the kinds it does, and so does a probe serial under a family's own
name such as `AVR_PROBE_SERIAL`, which nothing reads.

| `BRIO_PROBE` | Probe | Driven by | Manifest type for `brio flash` |
|---|---|---|---|
| `atmelice` | Atmel-ICE | avrdude `-c atmelice_updi` on the AVRs; OpenOCD with `interface/cmsis-dap.cfg` on the Cortex-M families | `atmelice_updi`; `openocd_cmsisdap` |
| `pickit4` | MPLAB PICkit 4 | avrdude `-c pickit4_updi -B 500kHz` | `pickit4_updi` |
| `pickit5` | MPLAB PICkit 5 | avrdude `-c pickit5_updi -B 500kHz` | `pickit5_updi` |
| `snap` | MPLAB Snap | avrdude `-c snap_updi -B 500kHz` | `snap_updi` |
| `nedbg` | the nEDBG of a Curiosity Nano | avrdude `-c pkobn_updi -B 500kHz` | `pkobn_updi` |
| `serialupdi` | a USB-serial adapter wired for UPDI | avrdude `-c serialupdi`, on `BRIO_PROBE_PORT` at `BRIO_PROBE_BAUD` | `serialupdi` |
| `stlink` | ST-LINK | OpenOCD with `interface/stlink.cfg` | `openocd_stlink` |
| `cmsis-dap` | any CMSIS-DAP probe | OpenOCD with `interface/cmsis-dap.cfg` | `openocd_cmsisdap` |
| `debugprobe` | Raspberry Pi Debug Probe | OpenOCD with `interface/cmsis-dap.cfg` on the USB bulk backend | `openocd_cmsisdap` with `"backend": "usb_bulk"` |
| `jlink` | SEGGER J-Link | OpenOCD with `interface/jlink.cfg`, the transport set to SWD | `openocd_jlink` |
| `bmp` | Black Magic Probe | the toolchain's gdb on `BRIO_PROBE_PORT`, no OpenOCD | `bmp` |
| `wchlink` | WCH-Link | WCH's OpenOCD fork | `wch_link` |

The manifest's UPDI types are avrdude's own programmer names: `brio
flash` passes the type to avrdude's `-c` as it is.

| Family | Its own probe (`BRIO_PROBE` empty) | Other kinds it drives |
|---|---|---|
| AVR DA, DB, EA (`avrxt`) | `atmelice` | `pickit4`, `pickit5`, `snap`, `nedbg`, `serialupdi` |
| SAM C21 | `atmelice` | `cmsis-dap`, `jlink`, `bmp` |
| STM32G0, STM32F4 | `stlink` | `cmsis-dap`, `atmelice`, `jlink`, `bmp` |
| RP2040 | `debugprobe` | `cmsis-dap`, `atmelice`, `bmp` |
| RP2350 | `debugprobe` | `cmsis-dap` |
| CH32V00x, CH32V203/CH32V303, CH32X035 | `wchlink` | none |

A family's own probe gives the command its target page describes, with
what was verified there. The PICkit 4 is measured on the bench through
`brio flash` (`UPDI_CLOCK` in `cli/bench/flash.py`) and was run through
an `-upload` target on an AVR128DB48, from an application outside the
tree: named by kind alone and with its serial, the image written and
read back. Every other kind - `pickit5`, `snap`, `nedbg`, `serialupdi`,
`cmsis-dap`, `atmelice` on an STM32 or an RP2040, `jlink`, `bmp` - and
a serial for `wchlink` through the `-upload` target is configured, not
run: its command was read in a build directory's `build.ninja`, and no
board was programmed with it.

A Black Magic Probe runs a gdb server of its own, so its `-upload`
target is not an OpenOCD session: none of the family's OpenOCD steps
runs (no target script, no reset sequence, no write that turns halting
debug off). gdb attaches, loads the image, compares every section with
the target's memory and prints each as `matched` or `MIS-MATCHED`.
gdb's exit status reports its last command only (measured with gdb
17.2 and no probe attached): it is not 0 when the closing `kill` finds
nothing attached - no probe on the port, no target, a refused attach -
and it is 0 after a section that did not match. `brio flash` reads the
lines and counts a board as flashed only when every section compared
is `matched`; an `-upload` target has gdb's status alone, so there the
lines are the verdict.

## The J-Link and the Black Magic Probe

Both have a command and no bench run: the manifest types
`openocd_jlink` and `bmp` and the `-upload` kinds `jlink` and `bmp`
are configured, not run. The J-Link reaches an RP2040 only through an
OpenOCD built from git, the release's J-Link driver lacking the
multidrop select, and the RP2040's `-upload` target does not offer
it. Each probe gets a page when it has a bench run.
