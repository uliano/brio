# ST Nucleo-G0B1RE

ST's Nucleo-64 board **MB1360** with an **STM32G0B1RE** (LQFP64, 512 KB
dual-bank flash, 144 KB SRAM), the bench chip of the `stm32g0/`
stratum. User manual UM2324.

- **Clock**: the part boots on HSI16; the kernel apps run HSI16 through
  the PLL at 64 MHz. No HSE crystal is fitted by default; the **LSE
  32.768 kHz crystal is fitted** and runs (the RTC, the tickless
  timebase and the timed sleep sites use it).
- **Supply**: 3.3 V (the target rail reads about 3.23 V).
- **LED** LD4 on PA5; **button** B1 on PC13, held HIGH by the board's
  own pull-up (a press is a falling edge).
- **Console**: the on-board ST-LINK's virtual COM port on USART2, PA2
  (TX) / PA3 (RX) at AF1, 115200 8N1; the ST-LINK carries a real USB
  serial, so the console is addressed by `/dev/serial/by-id` and never
  moves with the socket.
- **Probe**: the on-board ST-LINK/V2.1, the same USB device as the
  console - [../probes/st-link.md](../probes/st-link.md).
- **Identity**: the 96-bit unique device ID at 0x1FFF7590, read over
  SWD (`DeviceUid` in `stm32g0/flash.hpp`); every suite prints
  `DEV_ID 0x467`, `REV_ID` at boot.
- **Two facts of the silicon's reset state that a pad test meets**:
  PA8 (UCPD1_CC1) and PB15 (UCPD1_CC2) come out of a power-on holding
  the Type-C dead-battery pull-down until SYSCFG's strobe releases it
  (`ucpd_dead_battery(1, false)`, [../stm32g0/port.md](../stm32g0/port.md));
  and physical **bank 2 is brio's storage**, not code - the linker
  script gives the program bank 1 alone and a reflash leaves the
  NvHeap's blocks and the NvJournal's values in place
  ([../stm32g0/nvm.md](../stm32g0/nvm.md)).
- Manifest type `g0b1re`.

Documents: RM0444, DS13560 and ES0548 by revision in
[../stm32g0/vendor/README.md](../stm32g0/vendor/README.md); the
target's page [../stm32g0/README.md](../stm32g0/README.md).
