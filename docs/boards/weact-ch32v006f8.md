# A WeAct CH32V006F8U6 core board

WeAct Studio's CH32V00x core board (V1.0, the 3.3 V flavour) fitted with
a CH32V006F8U6 (QFN20, 62 KB of code flash and 8 KB of SRAM, the QingKe
V2C core, RV32EC): a small blue board, not WCH's own, and the 20-pin
package of the `ch32v00x/` stratum's larger part
([../ch32v00x/README.md](../ch32v00x/README.md)). WeAct uses the same
PCB for its CH32V003F4U6 board, so the board brings out the QFN20's
eighteen GPIO pins: PA1, PA2, PC0 to PC7 and PD0 to PD7, PA4 sharing
PD7's pin. What it offers a brio program is the
smallest-class RISC-V core brio runs on, a LED, and a one-wire debug
port whose probe also carries the console; it has no crystal, and three
of its pads carry a USB circuit that a program must know about (below).
It is one half of a two-board pair with a WeAct CH32X035F8U6 board, and
the wires between them are listed here.

The documents of record are the CH32V006 datasheet V2.0 (the QFN20
column of its pin table) and the CH32V00X reference manual V1.5
([../ch32v00x/vendor/README.md](../ch32v00x/vendor/README.md)); the
board's own facts are WeAct's schematic and what the suites measured on
it. The board file `boards/v006f8/board.hpp` states the same facts to
the compiler ([README.md](README.md), "The board file"). The silicon's
identity, read over the debug port: the chip identifier `0x00620620`
(the CH32V006K8U6's is `0x00600620`), `misa` `0x40801014` and 62 KB in
the flash capacity register.

## The board

- **Clock**: HSI 24 MHz, and the PLL at 48 MHz, which every app runs.
  No crystal is fitted: PA1 and PA2 go to the header alone, so an app
  that would start one runs the HSI's PLL instead, and the clock
  suite's crystal letters are not registered on this board type
  ([../ch32v00x/clock.md](../ch32v00x/clock.md)).
- **Supply**: 3.3 V from the probe's 3V3 on the board's VCC pin. The
  USB-C connector carries power alone and stays unplugged on the bench.
- **Debug**: PD1 = SWDIO, the one-wire SDI, with GND, on a WCH-LinkE
  ([../probes/wch-link.md](../probes/wch-link.md)).
- **Console**: USART1 on PD5 (TX) / PD6 (RX), the default alternate
  function, wired to the probe's own serial: one cable carries the
  debug port and the console, at 115200.
- **LED**: blue, on PC4 through 10 kOhm to ground, lit by a high pad;
  every app of the project drives it on this board (`board::Led`). A
  letter that reads a pad through its own pull-up therefore takes PA2,
  not PC4: the LED's load pulls PC4 well below the rail.
- **Key**: WeAct wires it as the reset, on PD7. It does nothing on the
  bench part: its option bytes read `0xffff5aa5`, USER and its
  complement both 0xFF (an area never programmed), and FLASH_OBR loads
  USER as 0xFF with no error flagged (RM 18.5): RST_MODE 11, so PD7 is
  an ordinary pad and not a reset; START_MODE 1, IWDG_SW 1 and
  STANDBY_RST 1 with it. Driving PD7 low as an output pulls the pin low
  and no pin reset follows. The option bytes are left so, because a
  reset on PD7 would cost the package's pin 1, which PD7 shares with
  PA4, the OPA's input (the datasheet's note 4 to its pin table).
- **PA4 and PD7 share one pin**: brio refuses to make either an output
  or an alternate-function pad (`brio/ch32v00x/pin.hpp`). With both
  pads' pull-downs on, the pin reads high (measured over the probe):
  something on the board holds it up against the internal pulls, and
  `test_ch32_opa`'s sign verdict declines when it cannot pull the pin
  to both rails.

## The software-USB circuit

WeAct populates a USB circuit for software USB on three pads, and the
resistors stay fitted: PD3 through 33 Ohm (SB2) to the connector's D+,
PD4 through 33 Ohm (SB3) to D-, and PD5 through 1.5 kOhm (SB4) to D-.
So **PD4 is joined to PD5, the console's transmit pad, through about
1.5 kOhm** (measured with a meter on two boards): PD4's level follows
PD5 and its own pulls cannot hold it. PD4 is not a free pad on this
board (`board::pd4_free`), and every letter that drives PD4, reads its
level or its pull is left unregistered on this board type; the timer
and serial suites' jumper letters, which ran over PD2 to PD4 on the
CH32V006K8U6 module ([ch32v006k8.md](ch32v006k8.md)), have no free pad
to move to here. PD3 carries a 33 Ohm stub to the unplugged connector
and is the serial link's receive pad (below).

## The wires to the WeAct CH32X035F8U6 board

The two boards ([weact-ch32x035f8.md](weact-ch32x035f8.md)) are each
other's peers, on a breadboard: eight signal wires and a ground, the
two I2C lines pulled up by 1.5 kOhm each to the other board's PB3. The
pads on this side are stated once, in
`ch32v00x/src/apps/ch32_pair.hpp`.

| this board | the CH32X035 | carries |
|---|---|---|
| PC0 | PA4 | SPI1's select (this side's column 1) |
| PC5 | PA5 | SCK |
| PC7 | PA6 | MISO |
| PC6 | PA7 | MOSI |
| PC2 | PC16 | I2C1's SCL (this side's column 0) |
| PC1 | PC17 | I2C1's SDA |
| PD2 | PB1 | USART2's TX (column 3) into USART4's RX |
| PD3 | PB0 | USART4's TX into USART2's RX |

The pull-ups hang from the CH32X035's PB3, never from the rail: a high
level on that chip's PC17 at power-up starts WCH's boot loader there.
With no I2C image on the CH32X035, PB3 floats and both lines read low,
which is how this board's I2C suite finds no peer and declines its wire
letters. The SPI suite declines its peer letters the same way when no
ping of the boot is answered.

**Neither board has a crystal**, so every timed leg across the serial
link is the ratio of two RC oscillators: `usart_peer` on this board
times the CH32X035's frames against this board's HSI, and the
CH32X035's suites judge only what that ratio can carry
([weact-ch32x035f8.md](weact-ch32x035f8.md),
[../ch32x035/usart.md](../ch32x035/usart.md)).

## The jumper the suites want

PA2 to PA1, the pin suite's levels, edges and scanner: two pads nothing
else on this board uses (no crystal, no pair wire, not the LED, not the
USB pads). The pin suite looks for the wire first, and without it each
of those letters declines with the reason, no verdict claimed.

## What the suites measured on it

- **The LSI runs at 105.2 to 106.8 kHz**, 17 % under its nominal 128
  kHz and 15 % under the CH32V006K8U6 module's; a program that needs
  its rate measures it ([../ch32v00x/sleep.md](../ch32v00x/sleep.md)).
- **A fast page erase takes 2925 to 2927 us** and a page program 1894
  to 1898 us, inside the datasheet's maxima
  ([../ch32v00x/nvm.md](../ch32v00x/nvm.md)).

## Flashing

`brio flash <board> <app>` writes the board through WCH's OpenOCD fork
and the WCH-LinkE; the board type in the manifest is `v006f8`, the
project `ch32v00x/` and the preset `ch32v006f8-release`
([../ch32v00x/README.md](../ch32v00x/README.md)). The CH32V006 builds a
suite whole where it fits; the suites too large for its 40 KB of
program flash name their own groups for this board type
([README.md](README.md), "How a board joins the bench").
