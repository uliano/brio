# A WeAct RP2350A board

WeAct Studio's RP2350A core board, version V2.1: an RP2350 in the QFN-60
package (30 GPIO, four ADC inputs; stepping A4 on the one at the bench)
behind a 16 MB quad-SPI flash, a 12 MHz crystal, USB-C, and the
Raspberry Pi Pico 2's pinout. WeAct publishes the design files of the V10
and the V20 of this board (WeActStudio.RP2350ACoreBoard on GitHub,
schematics and pinout), not of the V2.1; what this page says of a circuit
only a schematic shows is the V20's and is marked so, unmeasured on the
V2.1. What the board offers a brio program is the RP2350's smaller
package, with both of the chip's processor architectures, and so two
manifest types; it is one half of a two-board pair with a WeAct RP2040,
and the ten wires between them are listed here. Its facts for the
compiler are its board file, `boards/weact2350a/board.hpp`.

**Two architectures, one board.** The chip carries a pair of Cortex-M33
and a pair of Hazard3 RISC-V cores, and the image in the flash decides
which pair runs, so this board has two manifest types: `weact2350a`
(built for Arm) and `weact2350a-rv` (built for RISC-V). They are the
same piece of hardware and the same probe; only the preset differs
([../rp2350/README.md](../rp2350/README.md)).

## The board

- **Clock**: a 12 MHz crystal on XIN/XOUT (dedicated pins, not GPIOs;
  the V20 schematic's Y1); brio runs the crystal through the system PLL
  at 150 MHz, the chip's maximum.
- **Supply**: USB-C, 3.3 V logic; VBUS, VSYS and 3V3 on the header, and
  a VREF pin for the converter. On the V20 the 3V3 rail comes from a
  switching regulator whose mode pin is GP23 (below).
- **Buttons**: BOOT (the bootrom's mass-storage loader when held at
  power-up) and RESET (the RUN pin). There is no user key: neither
  button reaches a GPIO.
- **LEDs**: a power LED and a user LED on GP25, lit by a high pad (on
  the V20, a green LED through 5.1 kOhm to ground).
- **Pads the board takes**: GP23, GP24 and GP29 are not on the header
  rows. On the V20 GP23 is the regulator's mode pin, pulled to ground by
  100 kOhm (low its power-saving mode, high its fixed-frequency mode);
  GP24 senses VBUS through a divider of two 100 kOhm resistors; GP29
  senses VSYS through another, passed through a MOSFET whose gate is the
  3V3 rail, with 100 kOhm from the pad to ground - and GP29 is ADC input
  3. None of the three is free: the pin-check tool's wave leaves them
  out, and the converter's suite prints input 3 and never judges it.
- **Console**: none on board. The Debug Probe's UART bridge on UART0,
  GP0 (TX) / GP1 (RX), crossed, is the console
  ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)).
  The V20 has a footprint that would join GP0 to the flash's select,
  not fitted. The board's own USB-C is a second console's road, through
  the chip's own USB device controller ([../rp2350/usb.md](../rp2350/usb.md)).
- **Probe**: the board's SWD pads (3V3, SWDIO, SWCLK and GND on the
  V20's pinout) take the
  Debug Probe's port D on three wires. **The 3V3 pad stays unconnected**:
  each board is on its own USB, and two regulators are never put in
  parallel. No reset wire: the board is reset, and recovered from any
  state, by the chip's rescue reset over the debug port alone
  ([../rp2350/README.md](../rp2350/README.md)).
- **The package is a build fact**: `RP2350_PACKAGE=a` is what refuses
  GP30..GP47 at compile time and puts the ADC's inputs on GP26..GP29.
- **Erratum E9 does not apply**: the pad leakage RP2350-E9 describes is
  stepping A2's, and stepping A3 modified the pad circuit to remove it
  (the datasheet's appendix C.2.1), so on this board's A4 a pad whose
  input buffer stays on does not hold its history against its
  pull-down. The pads are still read the way the A2 needs, with the
  buffer enabled for the read alone, which costs nothing here.

## The wires to the WeAct RP2040 board

This board is wired to a WeAct RP2040 ([weact-rp2040.md](weact-rp2040.md))
by ten signal wires and two grounds, each board on its own USB-C, both
on one hub, no 3V3 wire between them. The plan is
`rp2350/src/apps/rp2350_pair.hpp`'s, keyed on the board file, and each
pad is checked there against the datasheet's function table (9.4):

| wire | here | RP2040 | carries |
|---|---|---|---|
| 1 | GP10 | GP10 | SPI1 SCK |
| 2 | GP8 | GP11 | the RP2040's SPI1 TX to this SPI1 RX |
| 3 | GP11 | GP8 | this SPI1 TX to the RP2040's SPI1 RX, 47 Ohm in series at this end |
| 4 | GP9 | GP29 | SPI1 CSn, 4.7 kOhm to the RP2040's 3V3 |
| 5 | GP16 | GP2 | I2C SDA (I2C0 here), 1.5 kOhm to the RP2040's 3V3 |
| 6 | GP17 | GP3 | I2C SCL, 1.5 kOhm to the RP2040's 3V3 |
| 7 | GP5 | GP4 | the RP2040's UART1 TX to this UART1 RX |
| 8 | GP4 | GP5 | this UART1 TX to the RP2040's UART1 RX |
| 9 | GP20 | GP21 | the RP2040's GPOUT0 to this GPIN0 |
| 10 | GP21 | GP20 | this GPOUT0 to the RP2040's GPIN0 |

The RP2040 end is the same as in the plan of the WeAct RP2350B
([weact-rp2350b.md](weact-rp2350b.md)); only this end moves, both links
into the low pads: the QFN-60 stops at GP29, and the board takes GP23,
GP24 and GP29 for itself.

**The 47 Ohm on wire 3** sits at this board's GP11, the driving end of
this chip's SPI answer line, and is part of the wiring, as on the
RP2350B, where it was measured to stop a lost frame of this chip as the
SPI client ([../rp2350/spi.md](../rp2350/spi.md)).

**An RP2040 pad is not failsafe** where this chip's is (RP2040 datasheet
5.5.3.1), so a link pad driven high while the RP2040 is unpowered feeds
that chip. Every image here that drives a link pad first reads the
RP2040's rail through the I2C link's pull-ups on GP16 and GP17
(`rp2350/src/apps/pair_power.hpp`: the pad's pull-down, its buffer
pulsed) and drives nothing when it reads low.

**One driver per wire.** While one board runs a suite on a link pad the
other runs a dark image (its pads inputs) or the peer of the same
subject, as on the RP2350B's page: the two I2C suites against the other
board's `twi_peer`, the two SPI suites against the other board's
`spi_peer`, the two serial suites against the other board's `uart_peer`
(its role chosen on wire 9 or 10, the driving board's GP21 into the
peer's GP20), and the two pairs of suites under the preemptive kernel
against the same peers.

**Letters this board does not register.** The wires between two of a
board's own pads are retired on every desk, and some of them end on
pads that are this board's links: the pin suite's four wires (GP8,
GP9, GP10, GP11, GP16 and GP17 among their ends), the PWM suite's and
the PIO suite's GP17 -> GP9, and the sleep suite's GP19 -> GP8. The
letters on those wires - the pin suite's `b` and `f` to `j`, the PWM
suite's `f` and `h`, the PIO suite's `e`, the sleep suite's `o` - are
not registered on this board, so no letter drives a link pad as if it
were a free one; each suite's header says why. The SPI suite's and the
bench app's loop-back host, whose receive pad and select are GP16 and
GP17 on the RP2350B, has no receive pad here and its select on GP26.

## Flashing

`brio flash <board> <app>` writes the board through Raspberry Pi's
OpenOCD fork in two sessions, a rescue over the debug port and then the
programming, whichever architecture the image already in the flash was
running ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)).
The manifest types are `weact2350a` and `weact2350a-rv`; the 16 MB flash
and the QFN-60 package are what the `rp2350a-*` presets are configured
for.

## Documents

The datasheets are in [../rp2350/vendor/README.md](../rp2350/vendor/README.md);
the target's page is [../rp2350/README.md](../rp2350/README.md).
