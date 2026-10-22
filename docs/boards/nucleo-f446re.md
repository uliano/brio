# ST Nucleo-F446RE

ST's Nucleo-64 board MB1136 (revision C-03 or C-04) with an STM32F446RE
(LQFP64, 512 KB of flash, 128 KB of SRAM): the board of the `stm32f4/`
stratum ([../stm32f4/README.md](../stm32f4/README.md)) that carries the
family's extras (QUADSPI, SAI2, SPDIFRX, FMPI2C, CEC) in the smaller
package. The user manual is UM1724. What the board offers a brio
program is the Nucleo arrangement, a LED, a button and an ST-LINK that
is both probe and console, with the ST-LINK's clock feeding the part and
a running LSE crystal; on the desk it is also the STM32F4 end of the
bus links to the Nucleo-G0B1RE.

## The board

- **Clock**: the part boots on HSI at 16 MHz; the kernel apps run the
  ST-LINK's 8 MHz MCO into HSE in bypass (SB16 and SB50 fitted, the
  board's default from revision C-02) through the PLL to 180 MHz in
  over-drive, PCLK1 45 MHz, PCLK2 90 MHz. No HSE crystal is fitted (X3
  is not provided); the LSE crystal X2 is fitted and runs (32769.3 Hz,
  +40 ppm against the core clock, ready 280 ms after LSEON, measured).
- **Supply**: 3.3 V (the target rail reads about 3.26 V).
- **LED** LD2 on PA5; **button** B1 on PC13.
- **Console**: the on-board ST-LINK's virtual COM port on USART2, PA2
  (TX) / PA3 (RX) at AF7 (SB13/SB14 fitted), 115200 8N1; the ST-LINK
  carries a real USB serial, so the console is addressed by
  `/dev/serial/by-id` and never moves with the socket.
- **Probe**: the on-board ST-LINK/V2-1 (firmware V2J43M28), the same
  USB device as the console ([../probes/st-link.md](../probes/st-link.md)).
- **Identity**: DBGMCU_IDCODE 0x10006421 (DEV_ID 0x421 = STM32F446,
  REV_ID 0x1000 = silicon markings A and 1 in ES0298's table 2); the
  flash size register reads 512 KB; the 96-bit unique ID is `DeviceUid`
  in `stm32f4/flash.hpp`; the platform suite prints all three at letter
  a.

## What a program meets on this board

PA11 and PB5 are free and serve the CAN suite as CAN1_RX and CAN2_RX:
inputs pulled up and never driven, because a loopback node still wants
eleven recessive bits on its receive pad.

## Flashing

`brio flash <board> <app>` writes the board through OpenOCD and the
ST-LINK; the board type in the manifest is `f446re`.

## Documents

RM0390, DS10693 and ES0298 by revision are in
[../stm32f4/vendor/README.md](../stm32f4/vendor/README.md); the target's
page is [../stm32f4/README.md](../stm32f4/README.md).
