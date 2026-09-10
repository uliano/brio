# ST Nucleo-G031K8

ST's Nucleo-32 board **MB1455** with an **STM32G031K8** (LQFP32, 64 KB
single-bank flash, 8 KB SRAM), the smallest part of the family brio
runs on. User manual UM2591 (the connector map: PA15 = D2 = CN3-5, PB8
= D8 = CN3-11, PB9 = D10 = CN3-13, PB5 = D11 = CN3-14, PB4 = D12 =
CN3-15, PB3 = D13 = CN4-15).

- **Clock**: HSI16, the PLL at 64 MHz for the kernel apps. The **LSE
  crystal X2 on PC14/PC15 runs with the oscillator bridges at
  UM2591's default** (SB7 off - the ST-LINK's MCO not routed to PC14 -
  SB8 and SB9 on); a board shipped with SB7 on and X2 cut off has no
  32 kHz source but LSI, which Shutdown powers down (an RTC on LSI
  cannot end a Shutdown, DS12992 3.7.4).
- **Supply**: 3.3 V.
- **LED** LD3 on PC6; no user button (PC13 is not bonded on this
  package).
- **Console**: the on-board ST-LINK's VCP on USART2 PA2/PA3 at AF1,
  115200. USART2 is a BASIC instance here (no kernel-clock
  multiplexer), so the suites whose subject is the clock move their
  console to LPUART1 on the same two pads at AF6.
- **Probe**: the on-board ST-LINK, by its USB serial -
  [../probes/st-link.md](../probes/st-link.md), which also says what to
  do when its debug port goes silent and how the mass-storage flasher
  serves meanwhile.
- **Board facts a pad test meets**: PA0 and PA4 lean HIGH when left
  floating (the Nucleo-64s' free pads drift down); the shipped HW1
  shunt between CN3-4 (GND) and CN3-5 (PA15) is to be pulled before
  PA15 is used, as UM2591's getting-started says; this part has no
  UCPD, so PA8 carries no dead-battery pull-down.
- **What the part has not got**: TIM4/6/7/15, USART3..6, LPUART2, I2C3,
  SPI3, DMA2, the DAC, every comparator, the EXTI's second register
  group, port E; the LQFP32 bonds PA0..PA15, PB0..PB9, PC6, PC14, PC15
  and PF2 alone. Fourteen of the seventeen stm32g0 suites run here
  ([../stm32g0/README.md](../stm32g0/README.md)); the two storage
  suites and the FDCAN's do not.
- Its errata sheet is **ES0487**, read against the letters in
  [../stm32g0/vendor/README.md](../stm32g0/vendor/README.md).
- Manifest type `g031k8`.
