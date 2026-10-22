# ST Nucleo-G071RB

ST's Nucleo-64 board with an STM32G071RB (LQFP64, 128 KB of
single-bank flash, 36 KB of SRAM): the same layout as the Nucleo-G0B1RE's
(LD4 on PA5, B1 on PC13, the virtual COM port on USART2 PA2/PA3) around
the x1 line's other LQFP64 part. The user manual is UM2324. What the
board offers a brio program is the Nucleo-G0B1RE's arrangement on a
smaller part, which is how the `stm32g0/` stratum proves that a driver
written for the bench chip holds on a part with less.

## The board

- **Clock**: HSI16, the PLL at 64 MHz for the kernel apps; no HSE
  crystal fitted; the LSE crystal is fitted and runs.
- **Supply**: 3.3 V.
- **Console and probe**: the on-board ST-LINK/V2.1, by its USB serial
  ([../probes/st-link.md](../probes/st-link.md)).
- **Identity**: the 96-bit unique device ID; the boot line prints
  `DEV_ID 0x460`. The stm32g0 peers put the ID's first word on the wire
  as their ident label.

## What the part has not got

All of it read off the device header by the stratum's reserve (the
table of what each part has, [../stm32g0/README.md](../stm32g0/README.md),
"Family coverage"): TIM4, USART5/6, LPUART2, I2C3, SPI3, DMA2, COMP3,
FDCAN, a second flash bank, port E. So fourteen of the seventeen stm32g0
suites run here, and the rest say why not; the counts per suite are in
the target's page.

## Flashing and documents

`brio flash <board> <app>` writes the board through OpenOCD and the
ST-LINK; the board type in the manifest is `g071rb`. Its errata sheet is
ES0418, read against the letters in
[../stm32g0/vendor/README.md](../stm32g0/vendor/README.md).
