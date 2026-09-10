# Probes

The probes brio flashes and debugs through, one page each, and the
flash MECHANISMS `bin/brio` knows - the `"type"` of a board's
`programmer` in the bench manifest ([../boards/README.md](../boards/README.md)).

| Mechanism | Probe | Wire | Boards |
|-----------|-------|------|--------|
| `atmelice_updi`, `pickit4_updi` | an EDBG-class probe, avrdude | UPDI | the AVR DA/DB boards |
| `serialupdi` | a USB-serial adapter, avrdude | UPDI | the AVR DA/DB boards |
| `openocd_cmsisdap` | an [Atmel-ICE](atmel-ice.md) (or any CMSIS-DAP probe), OpenOCD | SWD | the SAM C21 boards |
| `openocd_stlink` | a Nucleo's own [ST-LINK](st-link.md), OpenOCD | SWD | the STM32G0 Nucleos |
| `stlink_msd` | the ST-LINK's mass-storage flasher | USB drive | the STM32G0 Nucleos, when the debug port does not answer |
| `wch_link` | a [WCH-Link](wch-link.md), WCH's OpenOCD fork | SDI (1-wire) | the CH32V00x board |

Rules that hold for every probe: a probe is SINGLE-CLIENT (close the
debug session before flashing); a probe is addressed by its own USB
serial and needs it only when two of a kind are attached; every
OpenOCD flash ends with `** Verified OK **` or it did not happen - a
failed program leaves a partly erased image that may still print
another app's banner, or nothing.

Probes brio does not drive yet: SEGGER J-Link, ST-LINK/V3 as a
standalone probe (for a self-built STM board), WCH-LinkE (for the
CH32V00x, when that stratum comes). Each gets a page when it does.
