# ST 32F469IDISCOVERY

ST's Discovery kit **MB1189** with an **STM32F469NIH6** (UFBGA216, 2 MB
dual-bank flash, 320 KB SRAM + 64 KB CCM + 4 KB backup SRAM), the
fourth board of the `stm32f4/` stratum and the F469/F479 class's bench
part: the F42x/F43x's superset - the same FMC with a wider SDRAM, the
LTDC feeding a DSI host instead of an RGB panel, QUADSPI, ports up to K.
User manual UM1932; the display is a separate board, the **MB1166**, on
connector CN10.

- **Clock**: the part boots on HSI at 16 MHz; the kernel apps run the
  **8 MHz crystal X2 on HSE** through the PLL to **180 MHz in
  over-drive**, PCLK1 45 MHz, PCLK2 90 MHz (UM1932 4.3.1: the ST-LINK's
  MCO reaches HSE only with R35 fitted, R131 removed and SB19 closed -
  not the default). The **32.768 kHz crystal X3** is fitted on LSE. The
  USB apps run 168 MHz instead, the rate of this ladder whose VCO
  divides to the controller's 48 MHz (180 MHz leaves it 45).
- **Supply**: 3.3 V (the target rail reads 3.23 V through the probe).
- **LEDs** LD1 (green) on PG6, LD2 (orange) on PD4, LD3 (red) on PD5,
  LD4 (blue) on PK3, all LIT WHEN LOW (UM1932 4.15); **button** B2 (blue)
  on PA0, reading 1 when pressed; B1 (black) is reset.
- **Console**: the on-board ST-LINK/V2-1's virtual COM port on USART3,
  PB10 (TX) / PB11 (RX) at AF7 (UM1932 4.11, the MB1189's STLK_RX and
  STLK_TX nets), at 115200 8N1; addressed by `/dev/serial/by-id`.
- **Probe**: the on-board ST-LINK/V2-1 (firmware V2J40M27), the same USB
  device as the console, on the mini-B connector CN1 -
  [../probes/st-link.md](../probes/st-link.md). The micro-AB connector
  CN13 is the F469's own **USB OTG FS** port (PA11/PA12, VBUS on PA9, ID
  on PA10, the STMPS2151 power switch driven by PB2 and its over-current
  flag on PB7), CABLED to the host on the bench: the USB suite and the
  USB console enumerate through it.
- **Identity**: DBGMCU_IDCODE 0x10006434 (DEV_ID 0x434 = STM32F469/F479,
  REV_ID 0x1000 = silicon markings A and 1 in ES0321's table 2), the
  flash size register reads 2048 KB; the platform suite prints them at
  letter a.
- **What else is on the board**, each a chapter's peer when that chapter
  comes: the ISSI **IS42S32400F** 128 Mbit SDRAM - 4 M x 32 bits, 16 MB -
  on FMC SDRAM bank 1 (SDNE0/SDCKE0, 0xC0000000) over 54 pads at AF12
  across ports C, D, E, F, G, H and I (the FMC suite's board block lists
  them from the MB1189's MCU sheet); the 4" 800x480 DSI panel of the
  MB1166 with its capacitive touch controller on I2C1 (PB8/PB9, the
  board's 1.5 k pull-ups), reset on PH7 and tearing effect on PJ2; a
  128 Mbit Micron Quad-SPI NOR flash (UM1932 names the N25Q128A13; the
  JEDEC id 0x20BA18 it answers with is the MT25QL128's, its successor)
  on PF6..PF10 with PB6 as its select, found blank; the CS43L22 audio
  DAC on I2C2 (PH4/PH5) and SAI1; two
  MP34DT05 MEMS microphones; a microSD socket on SDIO with its detect
  on PG2; Arduino Uno headers and the 16-pin extension connector CN12.
- **Pads a suite uses that are the board's own**: PD0 (the SDRAM's D2,
  high impedance with the memory controller off) as CAN1_RX for the CAN
  letters, PB5 (CN12 pin 9) as CAN2_RX; PA6, PA8, PB13, PD12 and PD13 for
  the timer suite's channels; PA4 and PA5 for the analog suite - NEITHER
  DAC pad carries a load here (PA4 goes to the Arduino header's A5, PA5
  to CN12 pin 7), which the suite says by name.
- Manifest type `f469ni`.

Documents: RM0386, DS11189 and ES0321 by revision in
[../stm32f4/vendor/README.md](../stm32f4/vendor/README.md); the
target's page [../stm32f4/README.md](../stm32f4/README.md).
