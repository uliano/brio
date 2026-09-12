# Raspberry Pi Pico

Raspberry Pi's own board around the **RP2040** (B2 silicon on the one
at the bench): a **2 MB Winbond W25Q16JV** quad-SPI flash (JEDEC
0x1540ef), the recommended 12 MHz crystal (Abracon ABM8-272-T3, the
Pico datasheet's part), an RT6150 buck-boost regulator, micro-USB,
and twenty-six of the thirty GPIOs on its two header rows. Design
files: the schematic in the Pico datasheet's appendix B, the pinout in
its section 1.4 ([../rp2040/vendor/README.md](../rp2040/vendor/README.md)).

- **Clock**: 12 MHz crystal on XIN/XOUT; brio runs it through the
  system PLL at 125 MHz.
- **Supply**: micro-USB into VBUS, VSYS through a diode, 3.3 V from
  the buck-boost; 3V3_EN on the header. GP23 drives the regulator's
  power-save select (high = PWM mode, low = the default PFM), GP24
  reads VBUS through a divider, GP29 is ADC3 on VSYS/3 - the three
  pins the pin-check tool treats apart.
- **Buttons**: BOOTSEL only (the bootrom's mass-storage loader when
  held at power-up). No reset button: the RUN pin on the header.
- **LED**: the user **LED on GP25**. No power LED - a board with
  nothing driving GP25 shows no light at all.
- **Console**: none on board; the Debug Probe's UART bridge on
  **UART0 GP0 (pin 1, TX) / GP1 (pin 2, RX)**, function 2, crossed,
  GND on pin 3 ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)),
  addressed by `/dev/serial/by-id` under the probe's serial.
- **Probe**: the three DEBUG pads at the board's end - SWCLK, GND,
  SWDIO - with pins soldered on this one (a Pico H carries the JST-SH
  connector instead). Reset over SWD; no reset wire.
- **Flash**: the Winbond chip is the one the SDK's default second
  stage was written for and the one OpenOCD's 0.12.0 release
  identifies and programs - measured, `brio flash` runs this board on
  the release build with no override. The bootrom's USB drive takes
  a UF2 made from the image's `.bin` (the recovery path, measured
  too).
- **Identity**: the RP2040 has no die serial; the flash chip's unique
  id, read over the SSI, is the identity to come. The manifest's `id`
  is empty.
- Manifest type `pico`; the same type serves a Pico H. The 2 MB
  flash shares the `rp2040-*` presets with the WeAct board.

Documents: the datasheets in
[../rp2040/vendor/README.md](../rp2040/vendor/README.md); the target's
page [../rp2040/README.md](../rp2040/README.md).
