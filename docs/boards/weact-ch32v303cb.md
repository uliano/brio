# A WeAct CH32V303CBT6 core board

WeAct Studio's "BluePill Plus" core board fitted with a CH32V303CBT6
(LQFP48, 128 KB of code flash and 32 KB of SRAM, the QingKe V4F core,
RV32IMAFC with a single-precision floating-point unit): the same printed
circuit board as the [WeAct CH32V203C8T6 board](weact-ch32v203c8.md),
about 53 x 21 mm, and the CH32V303 part of the `ch32vx03/` stratum,
whose device class is the reference manual's CH32V30x_D8
([../ch32vx03/README.md](../ch32vx03/README.md)). What the board offers
a brio program is a 144 MHz RISC-V core with an FPU, two crystals, the
host/device USB controller on its own connector, a LED, a button, and a
two-wire debug port whose probe also carries the console. It is the
CH32V303's bench board and one half of a two-board pair with the WeAct
CH32V203C8T6 board, and the wires between them are listed here.

The documents of record are the CH32V303/305/307/317 datasheet V3.5 and
the CH32F/V20x_V30x_V31x reference manual V2.3
([../ch32vx03/vendor/README.md](../ch32vx03/vendor/README.md)); the
board's own facts below are the vendor's page and what the suites
measured on it. The silicon's identity, read over the debug port: the
chip identifier at 0x1FFFF704 reads `0x303305A4`, `misa` `0x40901125`
(I, M, A, F, C, user mode and WCH's own extension), the flash-capacity
signature 128 KB, and the unique identifier's two factory words
`0xF432ABCD` `0x5C0FBC10`.

## The board

- **Clock**: HSI 8 MHz and a PLL to 144 MHz, which is what most apps
  run. An 8 MHz crystal on PD0/PD1 (the LQFP48's oscillator pins), which
  the clock suite starts in about 1 ms, and a 32.768 kHz crystal on
  PC14/PC15, measured by the RTC suite at 32765.93 Hz (63 ppm slow)
  against the core's crystal-rooted counter
  ([../ch32vx03/clock.md](../ch32vx03/clock.md),
  [../ch32vx03/rtc.md](../ch32vx03/rtc.md)).
- **Supply**: the board's own USB-C, through its regulator to 3.3 V.
  The probe's 3V3 is not wired: its supply pins and the USB-C would meet
  on the rail.
- **Debug**: the two-wire port on the board's four-pin header, PA13 =
  SWDIO and PA14 = SWCLK, with GND, on a WCH-Link of the CH549 kind at
  firmware 2.12 ([../probes/wch-link.md](../probes/wch-link.md)). A
  console that stays noise with the wires right is that probe's serial
  bridge, and a reset of the probe over USB clears it.
- **Console**: USART1 on PA9 (TX) / PA10 (RX), the instance's default
  pads, wired to the probe's own serial: one cable carries the debug
  port and the console, at 115200.
- **USB**: the USB-C's data pair is PA11/PA12, the pair of the
  host/device controller of RM ch. 23. The CH32V303 has no USB device
  controller of ch. 21, so a USB console on this board drives the
  host/device controller in device mode
  ([../ch32vx03/usbfs.md](../ch32vx03/usbfs.md)), and the USB suite
  enumerates through this connector.
- **LED and KEY**: a blue LED on PB2, driven active high, every app's
  LED; a KEY button on PA0 (the vendor's page). PB2 is BOOT1 as well.
- **What it shipped with**: WeAct's RTC demo, the PLL on the crystal at
  144 MHz and a date line printed once a second on USART1 at 115200.
- **Option bytes** as found: RDPR `0xA5`, the flash unprotected; USER
  `0x9F`.

**The die is of a lot the lot notes do not restrict.** The registers
the reference manual gives to some lots of the CH32V30x_D8 only are
there on this die: ADCx_AUX takes and reads back its sampling-time
bits, EXTEN_CTR2's four amplifier speed bits set and clear one by one,
and DMA1 carries a span across a 64 KB boundary byte for byte (the
driver refuses that span on every lot all the same, because a program
cannot read its lot). Where the manual leaves a behaviour open and this
die answers differently from the CH32V303VCT6 of
[WCH's evaluation board](ch32v303-evt.md) - DMA in Sleep, an
enumeration with the core asleep, the RTC alarm's count - the suites
print the answer as a finding and judge only what both dies share
([../ch32vx03/README.md](../ch32vx03/README.md), "What the silicon
taught the stratum").

## The wires to the WeAct CH32V203C8T6 board

The two boards ([weact-ch32v203c8.md](weact-ch32v203c8.md)) are each
other's peers: eight signal wires and their grounds, every wire between
pads of the same name, the two I2C lines pulled up to the other board's
3.3 V by 2.2 kOhm each. On this part the pair's SPI link is SPI2 and its
serial link USART3, because the CH32V303CB has neither SPI3 nor a fourth
port (the rule is `ch32vx03/src/apps/vx03_pair.hpp`'s).

| this board | the CH32V203C8T6 | carries |
|---|---|---|
| PB12 | PB12 | SPI2's NSS to SPI2's NSS, both hardware NSS |
| PB13 | PB13 | SCK |
| PB14 | PB14 | MISO |
| PB15 | PB15 | MOSI |
| PB6 | PB6 | I2C1's SCL to I2C1's SCL |
| PB7 | PB7 | I2C1's SDA to I2C1's SDA |
| PB10 | PB11 | USART3's TX into USART3's RX |
| PB11 | PB10 | USART3's RX from USART3's TX |

The four SPI wires are a cable of their own, a flat about 20 cm long:
SCK with 33 Ohm in series at each end, MOSI with 56 Ohm at this board's
PB15, MISO with 56 Ohm at the other board's PB14. Through it the SPI suite's two-board
letters pass with either board as the host. With no series resistor at
all, the same letters failed at 1125 kHz: none of the eight
combinations of mode and bit order came back exact with the CH32V203's
pads at the medium slew class and four at the slow one, the far client
counting anywhere between 0 and 32 frames of 32 - the signature of an
SCK edge that rings. Checked wire by wire with both cores halted: the
eight wires and the jumper below, and no short.

## The jumper on the board itself

SPI1's loopback node, PA8 to PA6 and PA8 to PA7, kept because the
letters no peer can judge ride it (SPI1 above any peer's client
ceiling, a timer's output into another timer's capture and into an
amplifier's input). The WeAct CH32V203C8T6 board's other two jumpers,
USART2 crossed with that part's fourth port, have no counterpart here:
the CH32V303CB has three serial ports, and PA2 and PB0 are free.

## Flashing

`brio flash <board> <app>` writes the board through WCH's OpenOCD fork
and the WCH-Link; the board type in the manifest is `v303cb`, the
project `ch32vx03/` and the preset `ch32v303cb-release`, built with F
and the ilp32f ABI ([../ch32vx03/README.md](../ch32vx03/README.md)).
The fork's `reset run` does not start a program on this family; the
flash verb ends with `reset halt` and `resume`, which does.
