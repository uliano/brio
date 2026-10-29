# A WeAct RP2350B board

WeAct Studio's RP2350B core board: an RP2350 in the QFN-80 package (48
GPIO, eight ADC inputs; stepping A2 on the one at the bench) behind a
16 MB Winbond W25Q128 quad-SPI flash, a 12 MHz crystal, USB-C, and every
user GPIO on its header rows. The flash's JEDEC id reads EF 40 18, and
the bootrom finds it in quad-I/O mode, EBh at clock divisor 3, the first
combination it tries, with no status write by anybody
([../rp2350/flash.md](../rp2350/flash.md)). The design files are WeAct's
own repository (WeActStudio.RP2350BCoreBoard on GitHub, schematic and
pinout). What the board offers a brio program is the one chip of this
project with two processor architectures, and so the one board with two
manifest types; it is also one half of a two-board pair with a WeAct
RP2040, and the ten wires between them are listed here.

**Two architectures, one board.** The chip carries a pair of Cortex-M33
and a pair of Hazard3 RISC-V cores, and the image in the flash decides
which pair runs, so this board has two manifest types: `weact2350b`
(built for Arm) and `weact2350b-rv` (built for RISC-V). They are the
same piece of hardware and the same probe; only the preset differs, the
way one board carries two consoles elsewhere on this bench
([../rp2350/README.md](../rp2350/README.md)).

## The board

- **Clock**: a 12 MHz crystal on XIN/XOUT (dedicated pins, not GPIOs);
  brio runs the crystal through the system PLL at 150 MHz, the chip's
  maximum.
- **Supply**: USB-C, 3.3 V logic; VBUS, VSYS and 3V3 on the header, and
  a VREF pin for the converter.
- **Buttons**: BOOT (the bootrom's mass-storage loader when held at
  power-up), RESET (the RUN pin), and a user KEY on GP23. A driven
  output on GP23 would meet the button, so a pin-check wave leaves it
  out.
- **LEDs**: a power LED and a user LED on GP25.
- **A footprint for a second memory**: the board has pads for a second
  flash or a PSRAM whose chip select would be GP0. Nothing is fitted on
  the one at the bench, so GP0 is free and carries the console.
- **Console**: none on board. The Debug Probe's UART bridge on UART0,
  GP0 (TX) / GP1 (RX), crossed, is the console
  ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)),
  addressed by `/dev/serial/by-id` under the probe's serial. The board's
  own USB-C is a second console's road, through the chip's own USB
  device controller ([../rp2350/usb.md](../rp2350/usb.md)).
- **Probe**: the 4-pad header, 3V3, SWDIO, SWCLK and GND, takes the
  Debug Probe's port D on three wires. **The 3V3 pad stays
  unconnected**: each board is on its own USB, and two regulators are
  never put in parallel. No reset wire: the board is reset, and
  recovered from any state, by the chip's rescue reset over the debug
  port alone ([../rp2350/README.md](../rp2350/README.md)).
- **The package is a build fact**: `RP2350_PACKAGE=b` is what makes
  GP30..GP47 legal and puts the ADC's inputs on GP40..GP47 rather than
  on GP26..GP29.
- **Identity**: no die serial; the flash chip's 64-bit unique id is the
  board's, read through the flash chapter
  ([../rp2350/flash.md](../rp2350/flash.md)). The manifest's `id` is
  empty.

## The wires to the WeAct RP2040 board

This board is wired to a WeAct RP2040 ([weact-rp2040.md](weact-rp2040.md))
by ten signal wires and two grounds, each board on its own USB-C, both
on one hub, no 3V3 wire between them:

| wire | here | RP2040 | carries |
|---|---|---|---|
| 1 | GP30 | GP10 | SPI1 SCK |
| 2 | GP28 | GP11 | the RP2040's SPI1 TX to this SPI1 RX |
| 3 | GP31 | GP8 | this SPI1 TX to the RP2040's SPI1 RX, 47 Ohm in series at this end |
| 4 | GP41 | GP29 | SPI1 CSn, 1.5 kOhm to the RP2040's 3V3 |
| 5 | GP36 | GP2 | I2C SDA (I2C0 here), 1.5 kOhm to the RP2040's 3V3 |
| 6 | GP37 | GP3 | I2C SCL, 1.5 kOhm to the RP2040's 3V3 |
| 7 | GP5 | GP4 | the RP2040's UART1 TX to this UART1 RX |
| 8 | GP4 | GP5 | this UART1 TX to the RP2040's UART1 RX |
| 9 | GP20 | GP21 | the RP2040's GPOUT0 to this GPIN0 |
| 10 | GP21 | GP20 | this GPOUT0 to the RP2040's GPIN0 |

**The 47 Ohm on wire 3** sits at this board's GP31, the driving end of
this chip's SPI answer line, and is part of the wiring: without it this
chip as the SPI client lost a frame now and then at /16 of the RP2040's
clk_peri, its own answer edge reaching the select and SCK beside it
([../rp2350/spi.md](../rp2350/spi.md)).

**This board's GP29 does not drive**: its output reads 1 while driven
low at 12 mA, with or without a wire on it, so the select is on GP41
(an SPI1 CSn pad as well, and the converter's input 1); nothing here
drives GP29. GP41 reads the select's pull-up, which is why the
converter's suite prints input 1 and never judges it. The I2C link sits
above GP31 on purpose: its pads are the high half of the bank.

**An RP2040 pad is not failsafe** where this chip's is (RP2040 datasheet
5.5.3.1), so a link pad driven high while the RP2040 is unpowered feeds
that chip. Every image here that drives a link pad first reads the
RP2040's rail through the I2C link's pull-ups
(`rp2350/src/apps/pair_power.hpp`, erratum E9's own read: the pad's
pull-down, its buffer pulsed) and drives nothing when it reads low.

**Erratum E9 on a released wire**: a link line nobody drives keeps its
last level on a pad whose buffer is on, so an undriven wire is judged
after driving it, or with the buffer pulsed.

**One driver per wire.** While one board runs a suite on a link pad the
other runs a dark image (its pads inputs) or the peer of the same
subject: the two I2C suites, `test_rp2350_i2c` and `test_rp2040_i2c`,
each against the other board's peer of the twi_link protocol; the two
SPI suites, `test_rp2350_spi` and `test_rp2040_spi`, each against the
other board's spi_link peer (a PIO client, the roles inverted on the
PL022 with its own frame signal on the select); and the two serial
suites, `test_rp2350_serial` and `test_rp2040_serial`, each against the
other board's `uart_peer` (its role chosen on wire 9 or 10, the driving
board's GP21 into the peer's GP20; this board's peer reads its GP20 with
the buffer pulsed, E9's way, and brings UART1 up only once the RP2040's
rail reads up); and the two link suites under the preemptive kernel,
`test_rp2350_rubato_link` and `test_rp2040_rubato_link`, each against
the other board's `spi_peer` (key `z`) and `uart_peer` (key `y`). No other wire is on this board: every wire between two
of its own pads is retired, and the letters that judged one run by name
only.

## Flashing

`brio flash <board> <app>` writes the board through Raspberry Pi's
OpenOCD fork in two sessions, a rescue over the debug port and then the
programming, whichever architecture the image already in the flash was
running ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)).
The manifest types are `weact2350b` and `weact2350b-rv`; the 16 MB flash
and the QFN-80 package are what the `rp2350-*` presets are configured
for.

## Documents

The datasheets are in [../rp2350/vendor/README.md](../rp2350/vendor/README.md);
the target's page is [../rp2350/README.md](../rp2350/README.md).
