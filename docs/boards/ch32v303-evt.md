# WCH's CH32V303 evaluation board

WCH's own evaluation board for the **CH32V303VCT6** (LQFP100, 256 KB
of zero-wait code flash of a 480 KB array and 64 KB SRAM as the factory
splits them, the QingKe V4F core - RV32IMAFC, a single-precision FPU),
silkscreen `CH32xx0xV-R0-1v0`: the CH32V303 part of the `ch32vx03/`
stratum, whose device class is the reference manual's CH32V30x_D8.
Documents of record: the CH32V303/305/307/317 datasheet V3.5 and the
CH32F/V20x_V30x_V31x reference manual V2.3
([../ch32vx03/vendor/README.md](../ch32vx03/vendor/README.md)); the
board itself is the sheet `CH32xx0xV_EVT` of `CH32V30xSCH.pdf` in WCH's
CH32V307EVT package, whose `SCHPCB/CH32V303VCT6-R0` holds its sources.
Silicon identity, read over the debug port: the chip id at 0x1FFFF704
reads `0x30300514`, `misa` `0x40901125` (I, M, A, F, C, user mode and
WCH's own extension), the flash-capacity signature 288 KB, and the
unique identifier's two factory words `0x0C9BABCD` `0x74B0BC48` - its
third reads the array's erased pattern
([../ch32vx03/nvm.md](../ch32vx03/nvm.md)).

- **Clock**: HSI 8 MHz and a PLL to 144 MHz, which is what the apps
  run. An **8 MHz crystal Y1 on OSC_IN/OSC_OUT** (pins 12 and 13 of the
  LQFP100, which are not port D's: PD0 and PD1 are pins 81 and 82 of
  their own, the V3.5 datasheet's note 4) and a **32.768 kHz crystal Y2 on PC14/PC15**; the clock
  suite runs the PLL from Y1 as well.
- **Supply**: the USB connector P14 (USB-C, the two CC pads pulled down
  by 5.1 kOhm) or P4 in parallel with it, through a 500 mA fuse, the
  switch **S1** and a 1117 regulator (U1) to 3.3 V; a red LED (D1) says
  the rail is up. P3 brings 5 V, 3.3 V and GND out, and P11 selects
  what supplies the chip's VDD. P10 carries the 3.3 V and 5 V rails as
  well, so a probe's supply pins and a cable on P14 meet on the board
  unless one of them is left off.
- **Debug**: the 2x5 header **P10** (`LINK`): PA14 = SWCLK on pin 1,
  PA13 = SWDIO on pin 3, the two-wire port this family has, with GND,
  3.3 V and 5 V on the pins below them. The probe brio drives it with
  is a WeAct-branded WCH-Link of the CH549 kind at firmware 2.12
  ([../probes/wch-link.md](../probes/wch-link.md)), on the two debug
  pads, the two serial ones and GND.
- **Console**: USART1 on PA9 (TX, P10 pin 2) and PA10 (RX, pin 4), the
  instance's default pads, wired to the probe's own serial: one cable
  carries the debug port and the console, at 115200.
- **USB**: P14 and P4 both carry PA11/PA12, which the schematic calls
  USBFS and the datasheet's alternate-function table gives the OTG_FS
  pair - the host/device controller of RM ch. 23, and the pair it
  drives (measured). The CH32V303 has no USB device controller of
  ch. 21 (its clock gate is not there,
  [../ch32vx03/clock.md](../ch32vx03/clock.md)), so a USB console on
  this board drives the host/device controller in device mode
  ([../ch32vx03/usbfs.md](../ch32vx03/usbfs.md)) on P14 - the same
  stack, class and descriptors as the CH32V203's.
- **LEDs and the KEY**: the row **P1** carries LED1 (blue), LED2 (red)
  and KEY, and a jumper takes each to a pad. Both LEDs hang from 3.3 V
  through 1 kOhm, so a LED lights with its pad driven LOW; the KEY S2
  ties its pin to GND through 10 kOhm when pressed and the board gives
  it no pull-up. Every app of the project drives **PB2** as its LED,
  active high as on the WeAct CH32V203 board, and this board's LED1 is
  jumpered there - PB2 being BOOT1 as well, held low by the board's own
  10 kOhm - with the KEY jumpered to PA0; so on this board an app's
  `LED ON` darkens the LED.
- **Buttons**: S3 is the reset; S4 (`Download`) raises BOOT0 for WCH's
  serial boot loader.
- **What it shipped with**: WCH's demo, which prints
  `SystemClk:144000000` and `this is a test` on USART1 at 115200 after
  a reset through the probe.
- **Option bytes** as found: RDPR `0xA5`, the flash unprotected; USER
  `0x9F`, whose top three bits choose the 256 KB + 64 KB split of code
  flash and SRAM (RM table 32-4's note).
- **What its chip adds to the stratum**, each with its document: the
  V4F's floating-point unit ([../ch32vx03/platform.md](../ch32vx03/platform.md)),
  a second DMA controller ([../ch32vx03/dma.md](../ch32vx03/dma.md)),
  the DAC on PA4/PA5, read back by the converter on the same pads
  ([../ch32vx03/dac.md](../ch32vx03/dac.md)), the RNG
  ([../ch32vx03/rng.md](../ch32vx03/rng.md)), four amplifiers
  ([../ch32vx03/opa.md](../ch32vx03/opa.md)), TIM5 to TIM10
  ([../ch32vx03/tim.md](../ch32vx03/tim.md)) and five ports
  ([../ch32vx03/pin.md](../ch32vx03/pin.md)); what its die lacks - the
  registers a lot of its class may have - is in
  [../ch32vx03/README.md](../ch32vx03/README.md).
- **Storage**: the image gets the zero-wait window less its top 4 KB,
  which are the flash medium's zone
  ([../ch32vx03/nvm.md](../ch32vx03/nvm.md)); the 224 KB tail above the
  window is named in the linker script and holds nothing.
- **The jumpers the suites want**, every one on the board itself - SPI2
  against SPI3 (PB12-PA15, PB13-PB3, PB14-PB4, PB15-PB5, the same four
  pads carrying I2S2 against I2S3), USART2 crossed with UART4 on its
  default pads (PA2 to PC11, PC10 to PA3), TIM3's channel 1 into TIM2's
  channel 2 (PA6 to PA1, the same wire an EXTI edge), TIM8's channel 1
  into TIM4's channel 3 (PC6 to PB8), and I2C1 against I2C2 (PB6-PB10
  and PB7-PB11, each pair with a 4.7 kOhm pull-up to 3.3 V). `wire_check`
  finds exactly these ten over the twenty pads, and no stray.
- Manifest type `v303vc`; the project is `ch32vx03/`, preset
  `ch32v303vc-release` ([../ch32vx03/README.md](../ch32vx03/README.md)).
