# ST Nucleo-G0B1RE

ST's Nucleo-64 board MB1360 with an STM32G0B1RE (LQFP64, 512 KB of
dual-bank flash, 144 KB of SRAM), the bench chip of the `stm32g0/`
stratum and the board the learning route builds `blink` for. The user
manual is UM2324. What the board offers a brio program is everything
the route needs with no wiring: a LED, a button, a crystal for the RTC,
and an on-board ST-LINK that is both the probe and the serial console.

## The board

- **Clock**: the part boots on HSI16; the kernel apps run HSI16 through
  the PLL at 64 MHz. No HSE crystal is fitted by default; the LSE
  32.768 kHz crystal is fitted and runs, and the RTC, the tickless
  timebase and the timed sleep sites use it.
- **Supply**: 3.3 V (the target rail reads about 3.23 V).
- **LED** LD4 on PA5; **button** B1 on PC13, held high by the board's
  own pull-up, so a press is a falling edge.
- **Console**: the on-board ST-LINK's virtual COM port on USART2, PA2
  (TX) / PA3 (RX) at AF1, 115200 8N1. The ST-LINK carries a real USB
  serial, so the console is addressed by `/dev/serial/by-id` and never
  moves with the socket.
- **Probe**: the on-board ST-LINK/V2.1, the same USB device as the
  console ([../probes/st-link.md](../probes/st-link.md)).
- **Identity**: the 96-bit unique device ID at 0x1FFF7590, read over
  SWD (`DeviceUid` in `stm32g0/flash.hpp`); every suite prints
  `DEV_ID 0x467` and its `REV_ID` at boot.

## What a program meets on this board

Two facts of the silicon's reset state that a pad test meets:

- PA8 (UCPD1_CC1) and PB15 (UCPD1_CC2) come out of a power-on holding
  the Type-C dead-battery pull-down until SYSCFG's strobe releases it
  (`ucpd_dead_battery(1, false)`,
  [../stm32g0/port.md](../stm32g0/port.md)).
- Physical **bank 2 of the flash is brio's storage**, not code: the
  linker script gives the program bank 1 alone, and a reflash leaves the
  NvHeap's blocks and the NvJournal's values in place
  ([../stm32g0/nvm.md](../stm32g0/nvm.md)).

## Flashing

`brio flash <board> <app>` writes the board through OpenOCD and the
ST-LINK; the board type in the manifest is `g0b1re`. The step-by-step
build, flash and debug of `blink` on this board is
[../learn/command-line.md](../learn/command-line.md) and
[../learn/vscode.md](../learn/vscode.md).

## Documents

RM0444, DS13560 and ES0548 by revision are in
[../stm32g0/vendor/README.md](../stm32g0/vendor/README.md); the
target's page is [../stm32g0/README.md](../stm32g0/README.md).
