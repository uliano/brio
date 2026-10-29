# A WeAct RP2040 board

WeAct Studio's RP2040 core board: an RP2040 (two Cortex-M0+ cores, B2
silicon on the one at the bench) behind a 2 MB Zetta ZD25Q16 quad-SPI
flash (JEDEC 0x1560ba; the board is also sold with 4, 8 and 16 MB
chips), a 12 MHz crystal, USB-C, and every user GPIO on its two header
rows. The design files are WeAct's own repository
(WeActStudio.RP2040CoreBoard on GitHub, schematic and pinout). What the
board offers a brio program beyond a Pico is a flash chip OpenOCD's
release does not know, a reset button and a user button, and a wired
partner: it is one half of a two-board pair with a WeAct RP2350B, and
the ten wires between them are listed here.

## The board

- **Clock**: a 12 MHz crystal on XIN/XOUT (dedicated pins, not GPIOs);
  brio runs the crystal through the system PLL at 125 MHz.
- **Supply**: USB-C, 3.3 V logic (IOVDD 3.3 V); VBUS, VSYS and 3V3 on
  the header.
- **Buttons**: BOOTSEL (the bootrom's mass-storage loader when held at
  power-up), NRST (the RUN pin), and a user KEY on GP23. A driven output
  on GP23 would meet the button, so the pin-check tool's wave over the
  free pads leaves it out.
- **LEDs**: a power LED and a user LED on GP25, the pin a Pico's LED is
  on too, dark until the console's `LED` command drives it.
- **Console**: none on board. The Debug Probe's UART bridge on UART0,
  GP0 (TX) / GP1 (RX), crossed, is the console; UART0 is code 2 of
  those pads' function select
  ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)),
  and the port is addressed by `/dev/serial/by-id` under the probe's
  serial.
- **Probe**: the 4-pin header at the board's end, GND, SWDIO, SWCLK and
  3V3, takes the Debug Probe's SWD port (its port D) on three wires.
  **The 3V3 pin stays unconnected**: each board is on its own USB, and
  two regulators are never put in parallel. A reset is the core's own
  request over SWD (SYSRESETREQ); there is no reset wire, and what such
  a reset leaves running is below.
- **Pins a Pico uses otherwise**: GP24 and GP29 are plain header pins
  here (a Pico's VBUS sense and VSYS/3; GP29 carries the partner's
  select, below), GP23 is the button (a Pico's regulator PS pin).
  ADC_VREF is on the header.
- **Identity**: the RP2040 has no die serial; the flash chip's unique
  id (`Flash::unique_id()`, 4Bh over the SSI) is the board's. The
  manifest's `id` is empty.

## The wires to the WeAct RP2350B board

This board is wired to a WeAct RP2350B ([weact-rp2350b.md](weact-rp2350b.md))
by ten signal wires and two grounds, each board on its own USB-C, both
on one hub, and no 3V3 wire between them:

| wire | here | RP2350B | carries |
|---|---|---|---|
| 1 | GP10 | GP30 | SPI1 SCK |
| 2 | GP11 | GP28 | this SPI1 TX to the RP2350B's SPI1 RX |
| 3 | GP8 | GP31 | the RP2350B's SPI1 TX to this SPI1 RX, 47 Ohm in series at the RP2350B's end (its reason is on that board's page) |
| 4 | GP29 | GP41 | SPI1 CSn, 1.5 kOhm to this board's 3V3 |
| 5 | GP2 | GP36 | I2C SDA (I2C1 here), 1.5 kOhm to this board's 3V3 |
| 6 | GP3 | GP37 | I2C SCL, 1.5 kOhm to this board's 3V3 |
| 7 | GP4 | GP5 | this UART1 TX to the RP2350B's UART1 RX |
| 8 | GP5 | GP4 | the RP2350B's UART1 TX to this UART1 RX |
| 9 | GP21 | GP20 | this GPOUT0 to the RP2350B's GPIN0 |
| 10 | GP20 | GP21 | the RP2350B's GPOUT0 to this GPIN0 |

**The three pull-ups hang from this board's 3V3 header pin.** They are a
load on this board's regulator (6.6 mA at most, the three lines held
low), not a feed between the boards, so the 3V3 pin still joins
nothing. They sit on this side because an RP2040 pad is not failsafe
(datasheet 5.5.3.1: no pad above IOVDD + 0.5 V) where an RP2350's is, so
a line pulled to the other board's rail would feed this chip whenever it
is unpowered; the RP2350B reads these pull-ups to know this board is
powered before it drives a link pad.

**One driver per wire.** While one board runs a suite on a link pad the
other runs a dark image (one that configures none of the link pads and
leaves them inputs) or the peer of the same subject, the program that
answers the suite from the far end: the two I2C suites,
`test_rp2040_i2c` and `test_rp2350_i2c`, each against the other board's
peer speaking the twi_link protocol; the two SPI suites,
`test_rp2040_spi` and `test_rp2350_spi`, each against the other board's
spi_link peer (an SPI client written on the PIO, and for the other
direction the roles inverted, the PL022 as the host with its own frame
signal on the select); and the two serial suites, `test_rp2040_serial`
and `test_rp2350_serial`, each against the other board's `uart_peer`
(its role chosen on wire 9 or 10, the driving board's GP21 into the
peer's GP20); and the two link suites under the preemptive kernel,
`test_rp2040_rubato_link` and `test_rp2350_rubato_link`, each against
the other board's `spi_peer` (key `z`) and `uart_peer` (key `y`).

`brio flash` rescues the chip before programming it, so a freshly
flashed image starts from the power-on state: core 1 in the bootrom,
every block in reset. An SWD reset alone is core 0's and resets no
peripheral, so a block the previous image brought up (a UART1 driving
GP4) runs on under one that does not use it
([../rp2040/platform.md](../rp2040/platform.md)). No other wire is on
this board: every wire between two of its own pads is retired, and the
letters that judged one (a letter is one command key of a suite's
console) run by name only, answering "no wire".

## Flashing

`brio flash <board> <app>` writes the board through OpenOCD and the
Debug Probe; the board type in the manifest is `weact2040`, the 2 MB
flash shares the `rp2040-*` presets with a Pico, and a 16 MB variant
would want its own linker script and preset. The Zetta chip is one
OpenOCD's 0.12.0 release refuses, so this board's programmer entry
names the build from OpenOCD's git
([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md),
[../rp2040/README.md](../rp2040/README.md)).

## Documents

The datasheets are in [../rp2040/vendor/README.md](../rp2040/vendor/README.md);
the target's page is [../rp2040/README.md](../rp2040/README.md).
