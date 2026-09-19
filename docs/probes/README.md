# Probes

The probes brio flashes and debugs through, one page each, and the
flash MECHANISMS `bin/brio` knows - the `"type"` of a board's
`programmer` in the bench manifest ([../boards/README.md](../boards/README.md)).

| Mechanism | Probe | Wire | Boards |
|-----------|-------|------|--------|
| `atmelice_updi`, `pickit4_updi` | an EDBG-class probe, avrdude | UPDI | the AVR DA/DB boards |
| `serialupdi` | a USB-serial adapter, avrdude | UPDI | the AVR DA/DB boards |
| `openocd_cmsisdap` | an [Atmel-ICE](atmel-ice.md) (HID backend) or a [Raspberry Pi Debug Probe](raspberry-pi-debug-probe.md) (USB bulk backend, a UART bridge on board), OpenOCD | SWD, multidrop on the RP2040 | the SAM C21 boards; the RP2040 boards |
| `rp2350_openocd` | a [Raspberry Pi Debug Probe](raspberry-pi-debug-probe.md), Raspberry Pi's OpenOCD fork | SWD, ADIv6 | the RP2350 boards - and the ONE mechanism that is two OpenOCD sessions, a rescue over the debug port alone and then the programming |
| `openocd_stlink` | an [ST-LINK](st-link.md) - a Nucleo's or a Discovery's own, or a standalone V3 - OpenOCD | SWD | the STM32G0 Nucleos, the STM32F4 boards |
| `stlink_msd` | the ST-LINK's mass-storage flasher | USB drive | the STM32G0 Nucleos, when the debug port does not answer |
| `wch_link` | a [WCH-Link](wch-link.md), WCH's OpenOCD fork | SDI (1-wire) on the CH32V00x, two-wire on the CH32V203 | the CH32V00x and CH32V203 boards |

Rules that hold for every probe: a probe is SINGLE-CLIENT (close the
debug session before flashing); a probe is addressed by its own USB
serial and needs it only when two of a kind are attached; every
OpenOCD flash ends with `** Verified OK **` or it did not happen - a
failed program leaves a partly erased image that may still print
another app's banner, or nothing.

**Which OpenOCD** is a property of the target and not of the probe, and
the bench has three under `/sw`, each named by absolute path in the
manifest: the 0.12.0 release for every SWD path of the older targets, a
build from git for the RP2040 boards whose flash chip the release does
not identify, and Raspberry Pi's fork for the RP2350, which is the only
one of the three that can reach that chip at all
([../rp2350/README.md](../rp2350/README.md) says why, and
`cli/bench/flash.py` is where each is chosen).

Probes brio does not drive yet: SEGGER J-Link (RP2040-capable only
through an OpenOCD built from git, the release's J-Link driver
lacking the multidrop select), Black Magic Probe. Each gets a page when
it does.
