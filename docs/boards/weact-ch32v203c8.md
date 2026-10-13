# A WeAct CH32V203C8T6 core board

WeAct Studio's core board for the **CH32V203C8T6** (LQFP48, 64 KB of
code flash and 20 KB of SRAM, the QingKe V4B core - RV32IMAC): a blue
board in the black-pill shape, not WCH's own EVT, and the CH32V203 part
of the `ch32vx03/` stratum, whose device class is the reference
manual's CH32V20x_D6. Documents of record: the CH32V203 datasheet V2.8
and the CH32F/V20x_V30x_V31x reference manual V2.3
([../ch32vx03/vendor/README.md](../ch32vx03/vendor/README.md)); the
board's own facts below are the vendor's page and what the suites
measured on it. Silicon identity: the chip identifier at 0x1FFFF704
reads `0x20310500` and the unique identifier's two factory words
`0x865DABCD` `0xEF0CBCE2` - its third reads the array's erased pattern
([../ch32vx03/nvm.md](../ch32vx03/nvm.md)).

- **Clock**: HSI 8 MHz and a PLL to 144 MHz, which is what most apps
  run. An **8 MHz crystal on PD0/PD1** (the LQFP48's oscillator pins) and
  a **32.768 kHz crystal on PC14/PC15**; the clock suite and `twi_peer`
  run the PLL from the 8 MHz one
  ([../ch32vx03/clock.md](../ch32vx03/clock.md)).
- **Supply**: the board's own **USB-C**, through its regulator to
  3.3 V. The probe's 3V3 is not wired: its supply pins and the USB-C
  would meet on the rail.
- **Debug**: the two-wire port, **PA13 = SWDIO, PA14 = SWCLK**, with
  GND, on a WCH-LinkE ([../probes/wch-link.md](../probes/wch-link.md)).
- **Console**: USART1 on **PA9 (TX) / PA10 (RX)**, the instance's default
  pads, wired to the probe's own serial: one cable carries the debug
  port and the console, at 115200.
- **USB**: the USB-C's data pair is **PA11/PA12**, the device controller
  of RM ch. 21 (the host's pull-downs measured on those two pads; PB6/PB7,
  the host/device controller's, are free) - `console_usb`'s port.
- **LED and KEY**: a blue LED on **PB2** driven active high, every app's
  LED; a KEY button on **PA0** (the vendor's page; the pad carries no
  external resistor, so a press is what would prove it). PB2 is BOOT1 as
  well.
- **The wires to the CH32V303 evaluation board**
  ([ch32v303-evt.md](ch32v303-evt.md)), the two boards being each other's
  peers - eight signal wires and two grounds, the two I2C lines pulled up
  to THIS board's 3.3 V by 2.2 kOhm each:

  | this board | the CH32V303 | carries |
  |---|---|---|
  | PB12 | PA4 | SPI2's NSS - SPI3's NSS on its second column, both hardware NSS |
  | PB13 | PC10 | SCK |
  | PB14 | PC11 | MISO |
  | PB15 | PC12 | MOSI |
  | PB6 | PB6 | I2C1's SCL - I2C1's SCL |
  | PB7 | PB7 | I2C1's SDA - I2C1's SDA |
  | PB10 | PB1 | USART3's TX into UART4's RX on its second column |
  | PB11 | PB0 | UART4's TX on its second column into USART3's RX |

  The CH32V303's PA4 is not 5 V tolerant (the CH32V303 datasheet's pin
  table) and this board's SPI2 select idles high, so this board does not
  run an SPI2 image while the other board is unpowered. As an SPI CLIENT
  on these wires this board's own MISO edge (PB14) reaches its SCK and
  select inputs: it answers at the slowest slew class, and even so loses
  frames in modes 1 and 2 below 9 MHz while answering - exact while only
  listening ([../ch32vx03/spi.md](../ch32vx03/spi.md), the pair's
  section).
- **The jumpers on the board itself**, each kept because a letter no peer
  can judge rides it: USART2 crossed with UART4 on their default pads
  (PA2 to PB1, PB0 to PA3: the fourth port and DMA1's eighth channel),
  and SPI1's loopback node, PA8 to PA6 and PA8 to PA7 (SPI1 above any
  peer's client ceiling, TIM1 into TIM3's capture, TIM1 into OPA2's
  input). `wire_check` finds exactly these and the eight wires above,
  with no stray.
- Manifest type `v203c8`; the project is `ch32vx03/`, preset
  `ch32v203c8-release` ([../ch32vx03/README.md](../ch32vx03/README.md)).
