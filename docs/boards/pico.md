# Raspberry Pi Pico

Raspberry Pi's own board around the RP2040 (B2 silicon on the one at
the bench): a 2 MB Winbond W25Q16JV quad-SPI flash (JEDEC 0x1540ef),
the recommended 12 MHz crystal (Abracon ABM8-272-T3, the Pico
datasheet's part), an RT6150 buck-boost regulator, micro-USB, and
twenty-six of the thirty GPIOs on its two header rows. It is the
reference board of the `rp2040/` stratum
([../rp2040/README.md](../rp2040/README.md)): the flash chip the SDK's
second stage and OpenOCD's release were written for, and the documented
power path. The design files are the schematic in the Pico datasheet's
appendix B and the pinout in its section 1.4
([../rp2040/vendor/README.md](../rp2040/vendor/README.md)). What the
board offers a brio program is an RP2040 with no console of its own:
the Raspberry Pi Debug Probe supplies the probe and the console on one
cable.

## The board

- **Clock**: a 12 MHz crystal on XIN/XOUT; brio runs it through the
  system PLL at 125 MHz.
- **Supply**: micro-USB into VBUS, VSYS through a diode, 3.3 V from the
  buck-boost; 3V3_EN on the header. Three pins are the board's: GP23
  drives the regulator's power-save select (high = PWM mode, low = the
  default PFM), GP24 reads VBUS through a divider, GP29 is ADC3 on
  VSYS/3; the pin-check tool treats them apart.
- **Buttons**: BOOTSEL only (the bootrom's mass-storage loader when
  held at power-up). No reset button: the RUN pin is on the header.
- **LED**: the user LED on GP25. No power LED, so a board with nothing
  driving GP25 shows no light at all.
- **Console**: none on board. The Debug Probe's UART bridge on UART0,
  GP0 (pin 1, TX) / GP1 (pin 2, RX) under function 2, crossed, GND on
  pin 3 ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)),
  addressed by `/dev/serial/by-id` under the probe's serial.
- **Probe**: the three DEBUG pads at the board's end, SWCLK, GND and
  SWDIO, with pins soldered on this one (a Pico H carries the JST-SH
  connector instead). Reset over SWD; no reset wire.
- **Identity**: the RP2040 has no die serial; the flash chip's unique
  id (`Flash::unique_id()`, 4Bh over the SSI) is the board's. The
  manifest's `id` is empty.

## Flashing

The Winbond chip is the one the SDK's default second stage was written
for and the one OpenOCD's 0.12.0 release identifies and programs
(measured: `brio flash` runs this board on the release build with no
override). The board type in the manifest is `pico`, and the same type
serves a Pico H; the 2 MB flash shares the `rp2040-*` presets with the
WeAct board. The bootrom's USB drive takes a UF2 made from the image's
`.bin`, the recovery path, measured too.

## Documents

The datasheets are in [../rp2040/vendor/README.md](../rp2040/vendor/README.md);
the target's page is [../rp2040/README.md](../rp2040/README.md).
