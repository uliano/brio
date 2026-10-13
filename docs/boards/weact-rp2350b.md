# A WeAct RP2350B board

WeAct Studio's RP2350B core board: an **RP2350** in the QFN-80 package
(48 GPIO, eight ADC inputs; stepping A2 on the one at the bench) behind
a **16 MB Winbond W25Q128** quad-SPI flash (its JEDEC id reads EF 40 18,
and the bootrom finds it in quad-I/O mode - EBh at clock divisor 3, the
first combination it tries - with no status write by anybody,
[../rp2350/flash.md](../rp2350/flash.md)), a 12 MHz crystal, USB-C, and
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
  own USB-C is a second console's road, through the chip's own USB
  device controller ([../rp2350/usb.md](../rp2350/usb.md)).
- **Probe**: the 4-pad header - 3V3, SWDIO, SWCLK, GND - takes the Debug
  Probe's port D on three wires; THE 3V3 PAD STAYS UNCONNECTED (each
  board on its own USB, two regulators never in parallel). No reset
  wire: the board is reset, and RECOVERED FROM ANY STATE, by the chip's
  rescue reset over the debug port alone ([../rp2350/README.md](../rp2350/README.md)).
- **The desk partner**: this board is wired to a WeAct RP2040
  ([weact-rp2040.md](weact-rp2040.md)) by ten signal wires and two
  grounds, each board on its own USB-C, both on one hub, no 3V3 wire
  between them:

  | wire | here | RP2040 | carries |
  |---|---|---|---|
  | 1 | GP30 | GP10 | SPI1 SCK |
  | 2 | GP28 | GP11 | the RP2040's SPI1 TX -> this SPI1 RX |
  | 3 | GP31 | GP8 | this SPI1 TX -> the RP2040's SPI1 RX |
  | 4 | GP41 | GP29 | SPI1 CSn, 1.5 kOhm to the RP2040's 3V3 |
  | 5 | GP36 | GP2 | I2C SDA (I2C0 here), 1.5 kOhm to the RP2040's 3V3 |
  | 6 | GP37 | GP3 | I2C SCL, 1.5 kOhm to the RP2040's 3V3 |
  | 7 | GP5 | GP4 | the RP2040's UART1 TX -> this UART1 RX |
  | 8 | GP4 | GP5 | this UART1 TX -> the RP2040's UART1 RX |
  | 9 | GP20 | GP21 | the RP2040's GPOUT0 -> this GPIN0 |
  | 10 | GP21 | GP20 | this GPOUT0 -> the RP2040's GPIN0 |

  THIS BOARD'S GP29 DOES NOT DRIVE: its output reads 1 while driven low
  at 12 mA, with or without a wire on it, so the select is on GP41 (an
  SPI1 CSn pad as well, and the converter's input 1); nothing here
  drives GP29. GP41 READS THE SELECT'S PULL-UP, which is why the
  converter's suite prints input 1 and never judges it. The I2C link sits
  above GP31 on purpose: its pads are the high half of the bank. AN
  RP2040 PAD IS NOT FAILSAFE where this chip's is (RP2040 datasheet
  5.5.3.1), so a link pad driven high while the RP2040 is unpowered feeds
  that chip: every image here that drives a link pad first reads the
  RP2040's rail through the I2C link's pull-ups
  (`rp2350/src/apps/pair_power.hpp`, erratum E9's own read: the pad's
  pull-down, its buffer pulsed) and drives nothing when it reads low.
  ERRATUM E9 ON A RELEASED WIRE: a link line nobody drives keeps its
  last level on a pad whose buffer is on, so an undriven wire is judged
  after driving it, or with the buffer pulsed. ONE DRIVER PER WIRE: while
  one board runs a suite on a link pad the other runs a dark image (its
  pads inputs) or the peer of the same subject. No other wire is on this
  board: every wire between two of its own pads is retired, and the
  letters that judged one run by name only.
- **The package is a build fact here**: `RP2350_PACKAGE=b` is what makes
  GP30..GP47 legal and puts the ADC's inputs on GP40..GP47 rather than
  on GP26..GP29.
- **Identity**: no die serial; the flash chip's 64-bit unique id is the
  board's, read through the flash chapter
  ([../rp2350/flash.md](../rp2350/flash.md)). The manifest's `id` is
  empty.
- Manifest types `weact2350b` and `weact2350b-rv`; the 16 MB flash and
  the QFN-80 package are what the `rp2350-*` presets are configured for.

Documents: the datasheets in
[../rp2350/vendor/README.md](../rp2350/vendor/README.md); the target's
page [../rp2350/README.md](../rp2350/README.md).
