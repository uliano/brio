# ST STM32F429I-DISC1

ST's Discovery kit MB1075 with an STM32F429ZI (LQFP144, 2 MB of
dual-bank flash, 192 KB of SRAM + 64 KB of CCM + 4 KB of backup SRAM),
the bench chip of the `stm32f4/` stratum
([../stm32f4/README.md](../stm32f4/README.md)) and the superset part of
its family: the FMC with an SDRAM on board, the LTDC with a 2.4"
ILI9341 display on board, the DMA2D, the I/O ports up to K. The user
manual is UM1670. This is the DISC1 revision of the board, with an
ST-LINK/V2-B on board that has a virtual COM port; the older
32F429IDISCOVERY had the V2 without one. What the board offers a brio
program is the memory, the display and the sensors every big chapter of
the stratum is measured against, with the probe and the console on one
cable.

## The board

- **Clock**: the part boots on HSI at 16 MHz; the kernel apps run the
  8 MHz crystal X3 on HSE through the PLL to 180 MHz in over-drive,
  PCLK1 45 MHz, PCLK2 90 MHz. The ST-LINK's MCO reaches PH0 only through
  SB18, open by default (UM1670 7.12.1): a bypass configuration never
  sees HSERDY on this board, measured. No LSE crystal (X2 is not
  provided).
- **Supply**: 3.0 V (the target rail reads about 2.88 V through the
  probe).
- **LEDs** LD3 (green) on PG13, LD4 (red) on PG14; **button** B1 on PA0
  (SB2 fitted); B2 is reset.
- **Console**: the on-board ST-LINK's virtual COM port on USART1, PA9
  (TX) / PA10 (RX) at AF7, through SB11 and SB15, fitted by default on
  the DISC1, at 115200 8N1; addressed by `/dev/serial/by-id`.
- **Probe**: the on-board ST-LINK/V2-B (firmware V2J46M32), the same
  USB device as the console, on the mini-B connector CN1 with the two
  CN4 jumpers fitted ([../probes/st-link.md](../probes/st-link.md)).
  The micro-B connector on the other side is the F429's own USB OTG HS
  port (in full-speed mode on this board's PHY-less connector), cabled
  to the host on the bench for the day the USB device controller lands
  on it.
- **Identity**: DBGMCU_IDCODE 0x20036419 (DEV_ID 0x419 = STM32F42x/F43x,
  REV_ID 0x2003 = silicon markings 4, 5 and B in ES0206's table 2); the
  flash size register reads 2048 KB; the platform suite prints them at
  letter a.

## What else is on the board

Each a chapter's peer when that chapter comes: the ILI9341 LCD (SPI5
for commands, the LTDC for pixels), the STMPE811 touch controller
(I2C3), the L3GD20 gyroscope (SPI5), the IS42S16400J 64 Mbit SDRAM on
FMC SDRAM bank 2 (SDNE1/SDCKE1, 0xD0000000; 38 pads at AF12 across
ports B, C, D, E, F and G), a USB OTG HS connector.

## What a program meets on this board

- PA4 is the display's VSYNC and something on the board pulls it high:
  an unbuffered DAC output there reads 2906 of 4095 at code 0. PA5 is
  free.
- The rail is 2.93 V (VDDA measured through VREFINT), not 3.3.

## Flashing

`brio flash <board> <app>` writes the board through OpenOCD and the
ST-LINK; the board type in the manifest is `f429zi`.

## Documents

RM0090, DocID024030 and ES0206 by revision are in
[../stm32f4/vendor/README.md](../stm32f4/vendor/README.md); the target's
page is [../stm32f4/README.md](../stm32f4/README.md).
