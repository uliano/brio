# A WeAct RP2040 board

WeAct Studio's RP2040 core board: an **RP2040** (dual Cortex-M0+,
B2 silicon on the one at the bench) behind a **2 MB Zetta ZD25Q16**
quad-SPI flash (JEDEC 0x1560ba - the board is also sold with 4, 8
and 16 MB chips), a 12 MHz crystal, USB-C, and every user GPIO on its
two header rows. Design files: WeAct's own repository
(WeActStudio.RP2040CoreBoard on GitHub, schematic and pinout).

- **Clock**: 12 MHz crystal on XIN/XOUT (dedicated pins, not GPIOs);
  brio runs the crystal through the system PLL at 125 MHz.
- **Supply**: USB-C, 3.3 V logic (IOVDD 3.3 V); VBUS, VSYS and 3V3 on
  the header.
- **Buttons**: BOOTSEL (the bootrom's mass-storage loader when held at
  power-up), NRST (the RUN pin), and a user **KEY on GP23** - a driven
  output on GP23 would meet the button, so the pin-check wave leaves
  it out.
- **LEDs**: a power LED and a user **LED on GP25**, the pin a Pico's
  LED is on too (the console's heartbeat blinks it).
- **Console**: none on board; the Debug Probe's UART bridge on
  **UART0 GP0 (TX) / GP1 (RX)**, function 2, crossed, is the console
  ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)),
  addressed by `/dev/serial/by-id` under the probe's serial.
- **Probe**: the 4-pin header at the board's end - GND, SWDIO, SWCLK,
  3V3 - takes the Debug Probe's port D on three wires; THE 3V3 PIN
  STAYS UNCONNECTED (each board on its own USB, two regulators never
  in parallel). Reset over SWD (SYSRESETREQ); no reset wire.
- **Pins a Pico uses otherwise**: GP24 and GP29 are plain header pins
  here (a Pico's VBUS sense and VSYS/3), GP23 is the button (a Pico's
  regulator PS pin). ADC_VREF is on the header.
- **Identity**: the RP2040 has no die serial; the flash chip's unique
  id, read over the SSI, is the identity to come. The manifest's `id`
  is empty.
- Manifest type `weact2040`; the 2 MB flash shares the `rp2040-*`
  presets with a Pico. A 16 MB variant would want its own linker
  script and preset.

Documents: the datasheets in
[../rp2040/vendor/README.md](../rp2040/vendor/README.md); the target's
page [../rp2040/README.md](../rp2040/README.md).
