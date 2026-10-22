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
| `serialupdi` | a USB-serial adapter, driven by avrdude | UPDI | the AVR DA, DB and EA boards |
| `openocd_cmsisdap` | an [Atmel-ICE](atmel-ice.md) (the HID backend) or a [Raspberry Pi Debug Probe](raspberry-pi-debug-probe.md) (the USB bulk backend, with a UART bridge on board), driven by OpenOCD | SWD, multidrop on the RP2040 | the SAM C21 board; the RP2040 boards; the RP2350 board |
| `openocd_stlink` | an [ST-LINK](st-link.md), a Nucleo's or a Discovery's own or a standalone STLINK-V3, driven by OpenOCD | SWD | the STM32G0 Nucleos, the STM32F4 boards |
| `stlink_msd` | the ST-LINK's mass-storage flasher | a USB drive | the STM32G0 Nucleos, when the debug port does not answer |
| `wch_link` | a [WCH-Link](wch-link.md), driven by WCH's OpenOCD fork | SDI, one wire on the CH32V00x, two wires on the CH32V203, CH32V303 and CH32X035 | the CH32V00x, CH32V203, CH32V303 and CH32X035 boards |

On the RP2350 the board type, not the programmer kind, selects the
flash mechanism (`rp2350_openocd` in `cli/bench/common.py`'s
`BOARD_TYPES`): the same `openocd_cmsisdap` entry as an RP2040's, run
by Raspberry Pi's OpenOCD fork as two sessions, a rescue over the debug
port alone and then the programming
([raspberry-pi-debug-probe.md](raspberry-pi-debug-probe.md)).

The two EDBG-class probes are driven at one UPDI clock, 500 kHz
(avrdude's `-B`): the Atmel-ICE's own default, five times the PICkit 4's
(`UPDI_CLOCK` in `cli/bench/flash.py`).

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

## Probes brio does not drive yet

SEGGER J-Link (RP2040-capable only through an OpenOCD built from git,
the release's J-Link driver lacking the multidrop select) and the Black
Magic Probe. Each gets a page when it does.
