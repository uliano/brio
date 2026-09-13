# A WeAct STM32F411CE "black pill"

A WeAct Studio black pill with an **STM32F411CE** (UFQFPN48, 512 KB
flash, 128 KB SRAM), the small part of the `stm32f4/` stratum: the
100 MHz ladder with no over-drive, one ADC, no DAC and no CAN, ports A,
B, C and H alone. No probe on board: it is driven through a standalone
STLINK-V3 on its four-pin SWD header.

- **Clock**: the part boots on HSI at 16 MHz; the kernel apps run the
  board's **25 MHz crystal on HSE** through the PLL to **100 MHz**
  (M 16, N 128, P 2 - a 1.5625 MHz PLL input), PCLK1 50 MHz, PCLK2
  100 MHz. A **32.768 kHz crystal** is fitted on the LSE pads.
- **Supply**: 3.3 V from the probe's target pin or the USB-C connector
  (the target rail reads about 3.27 V).
- **LED** on PC13, which LIGHTS WHEN LOW (the LED hangs from 3.3 V and
  the pin sinks it); **KEY** button on PA0; a BOOT0 button that pulls
  the pin high while pressed (the ROM bootloader's DFU over USB).
- **Console**: none on board - the STLINK-V3's own UART bridge, two
  wires to USART1: the board's PA9 (TX, AF7) to the bridge's RX, PA10
  (RX) to its TX. The V3's connector labels for the pair proved
  ambiguous on the desk: with no banner at boot, swap them (two 3.3 V
  UART pins, nothing to break). 115200 8N1, addressed by
  `/dev/serial/by-id` (the V3 carries a real USB serial).
- **Probe**: the STLINK-V3 (firmware V3J16M9B5S1) on SWDIO/SWCLK/GND/
  3V3, attached with no NRST wire - the part is reached by SWD alone as
  long as the firmware leaves PA13/PA14 to the debug port and does not
  sleep with debug off; a program that does either needs the NRST wire
  and a connect under reset - [../probes/st-link.md](../probes/st-link.md).
- **USB**: the USB-C connector is the F411's OTG FS port, CABLED to the
  host on the bench; the kernel console runs over it as a CDC ACM device
  ([../stm32f4/usb.md](../stm32f4/usb.md)).
- **Identity**: DBGMCU_IDCODE 0x10006431 (DEV_ID 0x431 = STM32F411,
  REV_ID 0x1000 = silicon markings A, 1 and 2 in ES0287's table 2), the
  flash size register reads 512 KB.
- Manifest type `f411ce`.

Documents: RM0383, DS10314 and ES0287 by revision in
[../stm32f4/vendor/README.md](../stm32f4/vendor/README.md); the
target's page [../stm32f4/README.md](../stm32f4/README.md).
