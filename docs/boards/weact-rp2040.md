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
  LED is on too, dark until the console's `LED` command drives it.
- **Console**: none on board; the Debug Probe's UART bridge on
  **UART0 GP0 (TX) / GP1 (RX)**, function 2, crossed, is the console
  ([../probes/raspberry-pi-debug-probe.md](../probes/raspberry-pi-debug-probe.md)),
  addressed by `/dev/serial/by-id` under the probe's serial.
- **Probe**: the 4-pin header at the board's end - GND, SWDIO, SWCLK,
  3V3 - takes the Debug Probe's port D on three wires; THE 3V3 PIN
  STAYS UNCONNECTED (each board on its own USB, two regulators never
  in parallel). Reset over SWD (SYSRESETREQ); no reset wire.
- **The desk partner**: this board is wired to a WeAct RP2350B
  ([weact-rp2350b.md](weact-rp2350b.md)) by ten signal wires and two
  grounds, each board on its own USB-C, both on one hub, NO 3V3 wire
  between them. The links and this board's pads:

  | wire | here | RP2350B | carries |
  |---|---|---|---|
  | 1 | GP10 | GP30 | SPI1 SCK |
  | 2 | GP11 | GP28 | this SPI1 TX -> the RP2350B's SPI1 RX |
  | 3 | GP8 | GP31 | the RP2350B's SPI1 TX -> this SPI1 RX |
  | 4 | GP29 | GP41 | SPI1 CSn, 1.5 kOhm to this board's 3V3 |
  | 5 | GP2 | GP36 | I2C SDA (I2C1 here), 1.5 kOhm to this board's 3V3 |
  | 6 | GP3 | GP37 | I2C SCL, 1.5 kOhm to this board's 3V3 |
  | 7 | GP4 | GP5 | this UART1 TX -> the RP2350B's UART1 RX |
  | 8 | GP5 | GP4 | the RP2350B's UART1 TX -> this UART1 RX |
  | 9 | GP21 | GP20 | this GPOUT0 -> the RP2350B's GPIN0 |
  | 10 | GP20 | GP21 | the RP2350B's GPOUT0 -> this GPIN0 |

  THE THREE PULL-UPS HANG FROM THIS BOARD'S 3V3 HEADER PIN: a load on
  this board's regulator (6.6 mA at most, the three lines held low), not
  a feed between the boards, so the 3V3 pin still joins nothing. They sit
  on this side because an RP2040 pad is NOT failsafe (datasheet 5.5.3.1:
  no pad above IOVDD + 0.5 V) where an RP2350's is, so a line pulled to
  the other board's rail would feed this chip whenever it is unpowered;
  the RP2350B reads these pull-ups to know this board is powered before
  it drives a link pad. ONE DRIVER PER WIRE: while one board runs a
  suite on a link pad the other runs a dark image (its pads inputs) or
  the peer of the same subject - the two I2C suites, `test_rp2040_i2c`
  and `test_rp2350_i2c`, each against the other board's peer of the
  twi_link protocol, and the two SPI suites, `test_rp2040_spi` and
  `test_rp2350_spi`, each against the other board's spi_link peer (a PIO
  client, the roles inverted on the PL022 with its own frame signal on
  the select). No other wire is on this board: every
  wire between two of its own pads is retired, and the letters that
  judged one run by name only.
- **Pins a Pico uses otherwise**: GP24 and GP29 are plain header pins
  here (a Pico's VBUS sense and VSYS/3; GP29 carries the partner's
  select above), GP23 is the button (a Pico's
  regulator PS pin). ADC_VREF is on the header.
- **Identity**: the RP2040 has no die serial; the flash chip's unique
  id (`Flash::unique_id()`, 4Bh over the SSI) is the board's. The
  manifest's `id` is empty.
- Manifest type `weact2040`; the 2 MB flash shares the `rp2040-*`
  presets with a Pico. A 16 MB variant would want its own linker
  script and preset.

Documents: the datasheets in
[../rp2040/vendor/README.md](../rp2040/vendor/README.md); the target's
page [../rp2040/README.md](../rp2040/README.md).
