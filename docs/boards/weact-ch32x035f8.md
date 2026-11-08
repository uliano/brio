# A WeAct CH32X035F8U6 core board

WeAct Studio's CH32X035 core board (V1.0) with a CH32X035F8U6 (QFN20,
62 KB of code flash and 20 KB of SRAM, the QingKe V4C core, RV32IMAC, at
up to 48 MHz on its internal oscillator): a small blue board, not WCH's
own evaluation board ([ch32x035-evt-f8u6.md](ch32x035-evt-f8u6.md), the
same part), and the bench board of the `ch32x035/` stratum
([../ch32x035/README.md](../ch32x035/README.md)). What it offers a brio
program is a 48 MHz RISC-V core with no crystal and no reset pin, a LED,
a USB-C connector on the chip's own USB pads, and a two-wire debug port
whose probe also carries the console. It is one half of a two-board pair
with a WeAct CH32V006F8U6 board, and the wires between them are listed
here; they take the chip's USB pads, which shapes the one trap below.

The documents of record are the CH32X035/X033 datasheet V1.7 for the
pins and the reference manual for the chip
([../ch32x035/vendor/README.md](../ch32x035/vendor/README.md)); the
board's own facts are WeAct's schematic (`WeAct-CH32X035_CoreBoard_V10`
in WeAct's CH32X035CoreBoard repository) and what the suites measured on
it. The project has no board file for this board type. The silicon's
identity, read over the debug port: the word WCH's library reads as the
chip identifier, `0x035E0611` (the evaluation board's CH32X035F8U6 read
the same), `misa` `0x40901105`, 62 KB in the flash capacity register,
and option bytes `0x00ff5aa5` (unprotected, the defaults).

## The board

- **Clock**: no crystal, the series having no oscillator for one, so
  the apps run the 48 MHz HSI undivided
  ([../ch32x035/clock.md](../ch32x035/clock.md)).
- **Supply**: a 3.3 V regulator of its own, no 5 V flavour. On the bench the board runs from the probe's 3V3 on its VDD33
  pin and the USB-C stays unplugged (below).
- **Debug**: the two-wire port, PC18 = SWDIO and PC19 = SWCLK, with
  GND, on a WCH-LinkE ([../probes/wch-link.md](../probes/wch-link.md)).
- **Console**: USART2 on PA2 (TX) / PA3 (RX), wired to the probe's own
  serial: one cable carries the debug port and the console, at 115200.
- **LED**: blue, on PB12 through 10 kOhm (R2) to ground, lit by a high
  pad. The project's apps drive PA0 as their LED, the pad of WCH's own
  examples, which on this board drives nothing, so the console app's
  `LED` command lights nothing here. The pin suite judges PB12 taking
  its pull-up against the LED's load: it passes, with a margin no letter
  measures.
- **Key**: one, WeAct's DOWNLOAD key, which ties PC17 (UDP) to 3.3 V
  through 5.1 kOhm (R1) while pressed: at power-up a high PC17 starts
  WCH's boot loader. There is no user key and no reset key: the QFN20
  has no reset pin, and a reset is the probe's or the supply's.
- **USB**: the USB-C's D+ and D- are PC17 and PC16, straight to the
  connector and to two test points, DP (PC17) and DN (PC16); its CC1
  and CC2 lines reach PC14 and PC15 through the solder bridges SB2 and
  SB3, CC1 carrying the sink's 5.1 kOhm (R4) to ground at the connector.
  SB2 is closed on the bench board: PC14 read as an input under its
  pull-up reads low.

## The wires to the WeAct CH32V006F8U6 board

The two boards ([weact-ch32v006f8.md](weact-ch32v006f8.md)) are each
other's peers, on a breadboard: eight signal wires and a ground.

| this board | the CH32V006 | carries |
|---|---|---|
| PA4 | PC0 | SPI1's select (this board's one SPI column on the QFN20) |
| PA5 | PC5 | SCK |
| PA6 | PC7 | MISO |
| PA7 | PC6 | MOSI |
| PC16 (test point DN) | PC2 | I2C1's SCL on its column 010 |
| PC17 (test point DP) | PC1 | I2C1's SDA |
| PB0 | PD3 | USART4's TX into USART2's RX |
| PB1 | PD2 | USART2's TX into USART4's RX |

I2C1 sits on the USB pads because the QFN20 offers it nowhere else off
the debug port ([../ch32x035/i2c.md](../ch32x035/i2c.md)). Each line
carries a 1.5 kOhm pull-up to this board's PB3, which the I2C images
drive high for their whole run and nothing drives at a reset.

**The USB-C stays unplugged, and no pull-up goes to the rail.** A
pull-up on PC17 to a live rail holds it high at every power-up, exactly
as the DOWNLOAD key does, and the chip starts WCH's loader instead of
the program; hung from PB3, the pull-ups are live only once an I2C
image runs. With PB3 floating at power-up, PC16 and PC17 read low.

**Neither board has a crystal.** The far chip's `usart_peer` times this
board's frames against its own HSI, so every timed leg across the
serial link is the ratio of two RC oscillators. This board's HSI runs
0.35 to 0.54 % fast of the far one, and two legs taken back to back read
up to about 0.1 % apart, both oscillators wandering. So the USART suite
judges the bytes and the rate error a receiver tolerates, prints
without a verdict the spans it would judge within 500 ppm of a
reference leg, and declines the claims that need an absolute clock,
the HSI's error and its trim; the Rubato link suite's echo letters
decline too, this end being the faster writer
([../ch32x035/usart.md](../ch32x035/usart.md)).

## The jumper the suites want

PB11 to PB12, the pin suite's wire: PB11 sees PB12's LED through it,
and the letter judges eight edges across it. The suite detects the
wire before it judges and says "no wire" without it. No jumper sits on
a pad wired to the other board.

## Flashing

`brio flash <board> <app>` writes the board through WCH's OpenOCD fork
and the WCH-LinkE; the board type in the manifest is `x035f8`, the same
as the evaluation board's, the project `ch32x035/` and the preset
`ch32x035f8-release` ([../ch32x035/README.md](../ch32x035/README.md)).
Every CH32X035 board type splits a suite with a groups line into one
image per group ([README.md](README.md), "How a board joins the bench").
