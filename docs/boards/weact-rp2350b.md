# A WeAct RP2350B board

WeAct Studio's RP2350B core board: an **RP2350** in the QFN-80 package
(48 GPIO, eight ADC inputs; stepping A2 on the one at the bench) behind
a **16 MB Winbond W25Q128** quad-SPI flash, a 12 MHz crystal, USB-C, and
every user GPIO on its header rows. Design files: WeAct's own repository
(WeActStudio.RP2350BCoreBoard on GitHub, schematic and pinout).

- **Two architectures, one board.** The chip carries a pair of Cortex-M33
  and a pair of Hazard3 RISC-V cores, and the image in the flash decides
  which pair runs, so THIS BOARD HAS TWO MANIFEST TYPES: `weact2350b`
  (built for Arm) and `weact2350b-rv` (built for RISC-V). They are the
  same piece of hardware and the same probe; only the preset differs -
  the way one board carries two consoles elsewhere on this bench.
- **Clock**: 12 MHz crystal on XIN/XOUT (dedicated pins, not GPIOs);
  brio runs the crystal through the system PLL at 150 MHz, the chip's
  maximum.
- **Supply**: USB-C, 3.3 V logic; VBUS, VSYS and 3V3 on the header, and
  a VREF pin for the converter.
- **Buttons**: BOOT (the bootrom's mass-storage loader when held at
  power-up), RESET (the RUN pin), and a user **KEY on GP23** - a driven
  output on GP23 would meet the button, so a pin-check wave leaves it
  out.
- **LEDs**: a power LED and a user **LED on GP25**.
- **A footprint for a second memory**: the board has pads for a second
  flash or a PSRAM whose chip select would be GP0. NOTHING IS FITTED on
  the one at the bench, so GP0 is free and carries the console.
- **Console**: none on board; the Debug Probe's UART bridge on **UART0
  GP0 (TX) / GP1 (RX)**, crossed, is the console
  ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)),
  addressed by `/dev/serial/by-id` under the probe's serial. The board's
  own USB-C is a second console's road, through the chip's USB
  controller, when that chapter arrives.
- **Probe**: the 4-pad header - 3V3, SWDIO, SWCLK, GND - takes the Debug
  Probe's port D on three wires; THE 3V3 PAD STAYS UNCONNECTED (each
  board on its own USB, two regulators never in parallel). No reset
  wire: the board is reset, and RECOVERED FROM ANY STATE, by the chip's
  rescue reset over the debug port alone ([../rp2350/README.md](../rp2350/README.md)).
- **The package is a build fact here**: `RP2350_PACKAGE=b` is what makes
  GP30..GP47 legal and puts the ADC's inputs on GP40..GP47 rather than
  on GP26..GP29.
- **Identity**: no die serial; the flash chip's unique id is the board's,
  when the flash chapter exists to read it. The manifest's `id` is empty.
- Manifest types `weact2350b` and `weact2350b-rv`; the 16 MB flash and
  the QFN-80 package are what the `rp2350-*` presets are configured for.

Documents: the datasheets in
[../rp2350/vendor/README.md](../rp2350/vendor/README.md); the target's
page [../rp2350/README.md](../rp2350/README.md).
