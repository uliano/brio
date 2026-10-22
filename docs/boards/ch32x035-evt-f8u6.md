# WCH's CH32X035 evaluation board, QFN20 edition

WCH's own evaluation board for the CH32X035 series in the edition that
carries a CH32X035F8U6 (QFN20, 62 KB of code flash and 20 KB of SRAM,
the QingKe V4C core, RV32IMAC, at up to 48 MHz on its internal
oscillator): the bench part of the `ch32x035/` stratum, the directory of
`brio/` that holds this family's drivers
([../ch32x035/README.md](../ch32x035/README.md)). What the board offers
a brio program is every pad the package bonds on two headers, a USB-C
connector on the chip's own USB pads, two LEDs reached by jumper wires,
and a two-wire debug port whose probe also carries the console. The
package has no reset pin and the series has no crystal, which shape the
traps below.

The documents of record are the CH32X035 evaluation board reference
V1.3 (its QFN20 page) and the schematic `CH32X035SCH.pdf` in WCH's
CH32X035EVT package (the QFN20 edition's sheet, its sources under
`SCHPCB/CH32X035F8U6-R0`, the PCB `CH32X035F8U-R0-1v1`), with the
CH32X035/X033 datasheet V1.7 for the pins
([../ch32x035/vendor/README.md](../ch32x035/vendor/README.md)). The
chip's own identity is what the platform test suite prints at its
first letter (a letter is one command key of a suite's console): 62 KB
of code flash, the unique identifier, and the word WCH's library reads
as the chip identifier, 0x035E0611 on the board's CH32X035F8U6, the
library's 0x035E06x1 for this part
([../ch32x035/platform.md](../ch32x035/platform.md)). The rest of the
page is the schematic's and the reference's, with what the suites
confirmed on the board: the debug port and the console on the pads
below, PC14 reading low under its pull-up against the CC1 line's
5.1 kOhm, and the jumper the pin suite detects.

## The board

- **Clock**: no crystal, the series having no oscillator for one, so
  the apps run the 48 MHz HSI undivided
  ([../ch32x035/clock.md](../ch32x035/clock.md)).
- **Supply**: the USB-C connector P7 brings VBUS straight onto the
  board's 5 V input, and the second USB connector P8 (+5V, D-, D+, GND)
  through a diode; from there a 500 mA fuse, an HT7533 regulator to
  3.3 V, and the switch S2, which connects the regulator's output to the
  3.3 V rail (open, the rail is whatever the headers' VCC pins are
  given). A red LED says the rail is up. P3 is two pins on the 5 V
  input, and a 0 Ohm link beside the regulator is not fitted.
- **Debug**: the two-wire port on the header P1, PC18 = SWDIO on pin 6
  and PC19 = SWCLK on pin 8, with VCC on pin 2 and GND on pin 4 (the
  schematic's sheet; the reference calls the port "SDI"). A WCH-LinkE
  drives it ([../ch32x035/README.md](../ch32x035/README.md),
  [../probes/wch-link.md](../probes/wch-link.md)).
- **Console**: USART2 on PA2 (TX, P1 pin 7) and PA3 (RX, P1 pin 5), the
  reference's "Serial port 2", wired to the probe's own serial: one
  cable carries the debug port and the console, at 115200. WCH's own
  examples print on USART1, whose default TX pad, PB10, this package
  does not bring out ([../ch32x035/usart.md](../ch32x035/usart.md)).
- **USB**: P7's D+ and D- are PC17 (UDP) and PC16 (UDM), behind a
  CH412K protection device, and P8 carries the same pair. P7's two CC
  lines are each tied to ground by 5.1 kOhm (the board is a USB-C sink)
  and reach PC14 (CC1) and PC15 (CC2) through 0 Ohm links, the USB PD
  controller's pads; so a program reading PC14 as an input reads the
  pull-down, and a program that drives either pad drives into it.
- **LEDs**: two, LED1 and LED2, each from 3.3 V through 1 kOhm to a pin
  of the header P4. A LED reaches a pad only through a jumper wire, and
  lights with that pad driven low. The apps of the project drive PA0
  (P1 pin 11) as their LED, the pad WCH's own GPIO example toggles, so
  with LED1 jumpered from P4 pin 1 to P1 pin 11 **the console app's
  `LED ON` command darkens it**. The schematic's net LED_G is not on
  this edition's sheet:
  it belongs to another board of the EVT package, the USB PD one
  (`SCHPCB/CH32X035USBPD_CH211`).
- **Buttons**: S4 (`Download`) ties PC17 to 3.3 V through 4.7 kOhm
  while pressed, which at power-up selects WCH's boot loader (the
  reference: the chip "powers up and detects USB_P (PC17) high level to
  enter BOOT"). There is no reset button: the QFN20 has no reset pin,
  and a reset is the probe's or the supply's.

## The headers

Two of 7x2, carrying every pad the package bonds:

| P1 pin | signal | P1 pin | signal | | P2 pin | signal | P2 pin | signal |
|---|---|---|---|---|---|---|---|---|
| 1 | VBUS | 2 | VCC | | 1 | VCC | 2 | GND |
| 3 | VCC | 4 | GND | | 3 | PC14 | 4 | PC15 |
| 5 | PA3 | 6 | PC18 | | 5 | PB11 | 6 | PB12 |
| 7 | PA2 | 8 | PC19 | | 7 | PB1 | 8 | PB3 |
| 9 | PA1 | 10 | PA4 | | 9 | PA7 | 10 | PB0 |
| 11 | PA0 | 12 | PC16 | | 11 | PA5 | 12 | PA6 |
| 13 | PC15 | 14 | PC17 | | 13 | GND | 14 | GND |

## What a program meets on this board

- **Any pull-up on PC17 selects the boot loader at power-up**, exactly
  as S4 does: a resistor from PC17 to a live rail (an I2C pull-up on
  the pad, I2C1's SDA on its column 2) holds it high at every power-up.
  Such a pull-up therefore hangs from a pad the program drives high only
  once it runs; the bench hangs its two from PB3.
- **With no supply on the rail** (neither the USB nor the probe's VCC
  on P1's pin 2) the board still boots, fed through the protection
  diodes of the probe's and the console's pads, and restarts about
  seventy times a second at 48 MHz whenever the core stays busy (a
  suite's banner, a burst of console lines), the restarts going on with
  the core halted by the probe; it runs clean at 8 MHz. With either
  supply connected, the USB with S2 closed or the probe's 3V3 on P1's
  pin 2, the suites run at 48 MHz.

## The jumpers the suites want

None is required: each suite detects its wire before it judges and says
"no wire" otherwise. PB11 to PB12 (P2 pin 5 to P2 pin 6) serves the pin
suite, and LED1 to PA0 lets a hand at the bench see the console app's
`LED` command. A jumper never sits on a pad wired to another board:
PA4..PA7, PB0, PB1, PB3, PC16 and PC17 carry the bench's links to a
CH32V006K8U6 (SPI1, USART4, I2C1 and its pull-up rail), and the USART
suite's two-board letters run across PB0/PB1 against that board's peer,
the program on the far end that answers the suite.

## Flashing

`brio flash <board> <app>` writes the board through WCH's OpenOCD fork
and a WCH-LinkE; the board type in the manifest is `x035f8`, the project
`ch32x035/` and the preset `ch32x035f8-release`
([../ch32x035/README.md](../ch32x035/README.md)). Every CH32X035 board
type splits a suite with a groups line into one image per group
([README.md](README.md), "How a board joins the bench").
