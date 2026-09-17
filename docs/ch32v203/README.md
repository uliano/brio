# Target: CH32V203 (`ch32v203/`)

The operational page for the CH32V203 target: WCH's QingKe V4B, a
RISC-V core with the **RV32IMAC** instruction set - thirty-two
registers, a multiplier and a divider, the atomic extension, user mode
and a 64-bit system counter - on the CH32V203C8T6 of the bench (64 KB
flash, 20 KB SRAM, 144 MHz). brio's second WCH family and its second
RISC-V one, and the fact this target exists to keep true is the same as
every other: the kernel and util strata compile here unchanged.

Two things shape the stratum and are stated up front. **There is no
vendor header in the build**: WCH ships its register definitions inside
the EVT package, whose licence is written for software running on WCH
parts, so [brio/ch32v203/device.hpp](../../brio/ch32v203/device.hpp)
carries the map, read off the reference manual and answerable to it -
and per-part variability, which the ST strata ask the device header
for, is stated in [parts/](../../brio/ch32v203/parts/). And **this is
not the CH32V00x with a bigger core**: the peripheral generation is a
different one, the STM32F1's under WCH's names - sixteen pins a port
with the F1's two-bit MODE, a USB device controller that is ST's
register for register, a bxCAN, an F1 timer set - where the CH32V00x is
a reduced design of its own.

## The documents

One document per peripheral driver is the shape
[../README.md](../README.md) prescribes; on this target the drivers are
young enough that each header is still its own document of record, and
this page carries what the bench has measured so far. A row becomes a
page as its driver is measured.

| Driver | State |
|--------|-------|
| [device.hpp](../../brio/ch32v203/device.hpp) + [parts/](../../brio/ch32v203/parts/) | the register map and the part table (memories, bonded pads, instances, the device class) |
| [platform.md](platform.md) | Platform: `Ch32v203Platform` (the csrrci critical section, the WFE-shaped `idle()`, `ebreak`, the `.noinit` breadcrumb), `Pfic` and the one handler attribute `BRIO_CH32_INTERRUPT` (the core's hardware prologue MEASURED: 53 cycles of round trip against 63, 152 bytes of flash and SIXTY-FOUR BYTES OF USER STACK the internal hardware stack carries instead - which is what parts this core from the CH32V00x's), the 64-bit STK `BasicTicker` (its CNT arithmetic 1 ppm against the interrupt count over 200 reloads) and `delay_us` on that counter (100 us in 14434 cycles of 14400 asked, a tick period and above refused), corecfgr's 0x1F measured at two cycles in 36811, and the tick rate against the host's clock - the HSI half a per cent fast where the board's crystal is exact; then the failing half, [reset.hpp](../../brio/ch32v203/reset.hpp): the six flags as the history they are, `Reset::software()` through the core's keyed PFIC_CFGR reading back as SFTRSTF alone, `ResetReporter`, `fault_reset<P>()` carrying the cause the core left in mcause, and the ebreak that lands on the BREAKPOINT vector and not the exception one; three real resets in the suite |
| [clock.hpp](../../brio/ch32v203/clock.hpp) | RCC: the HSI, a crystal, the PLL and the three prescalers, with the USB divider part of the tree; measured at 144 MHz from the HSI and 48 MHz from the board's crystal |
| [pin.hpp](../../brio/ch32v203/pin.hpp) | GPIO: the F1's four-bit nibbles over two registers, the pull that lives in the output register, the pad bonding from the part table |
| [usart.hpp](../../brio/ch32v203/usart.hpp) | USART: the frame, the divisor, the flags and the two rings of the transport; measured: the console at 115200 with the divisor exact against PCLK2 |
| [usb.hpp](../../brio/ch32v203/usb.hpp) | USBD: ST's device controller under WCH's names, realizing util/usb's UsbController; measured: the kernel console over a CDC ACM port on the board's own USB-C, enumerated, configured and carrying bytes with no overflow - and the core must not sleep while it does (below) |

## Toolchain

WCH's own `riscv32-wch-elf` gcc 15.2.0 at `/sw/wch-riscv`, the same
compiler the CH32V00x target uses and for the same reason: it is the
only one that emits WCH's proprietary `xw` compressed extension, which
this core carries too (misa reads `0x40901105` - I, M, A, C, user mode
and one non-standard extension). What is NOT shared with that family is
the ABI: **ilp32** here against its ilp32e, because this core has the
full thirty-two registers. The project's `-march` is `rv32imac_xw`,
which their gcc resolves to the `rv32imac_zaamo_zalrsc_xw/ilp32`
multilib.

## Board and build

A **WeAct Studio CH32V203C8T6 core board** - a blue board in the
black-pill shape, not WCH's own EVT: an 8 MHz crystal and a 32.768 kHz
one, a blue LED on **PB2** driven active high, a KEY button on PA0 (the
vendor's page; the pad carries no external resistor, so a press is what
would prove it), and a USB-C connector wired to the chip's own USBD
pads.

```bash
(cd ch32v203 && cmake --build --preset ch32v203c8-release --target console)
(cd ch32v203 && cmake --build --preset ch32v203c8-release --target console-upload)
brio flash P console        # the same thing through the bench's one command
```

The part number selects the part definition `device.hpp` asks for, the
linker script and the board type, from one table
([cmake/ch32v203-parts.cmake](../../ch32v203/cmake/ch32v203-parts.cmake)):
the nine parts of the series are named there whether or not a board
exists for them, because the table is a statement about the family.

## The probe and the upload

A WCH-LinkE (firmware 2.16) over the **two-wire** debug port this
family has - PA13 = SWDIO, PA14 = SWCLK - not the CH32V00x's single
wire. WCH's OpenOCD fork at `/sw/wch-openocd` is the only OpenOCD that
speaks the probe's SDI transport.

**`reset run` does not start the program.** In that fork the verb
leaves the hart sitting at the reset vector with every peripheral at
its reset value - measured here: the program counter still zero and the
clock tree untouched four hundred milliseconds later - so an image
flashed that way is in the chip and NOT running, and the board looks
dead. The upload target and `brio flash` both use `reset halt` followed
by `resume`.

## Serial console

The probe's own serial pins go to **PA9 (USART1_TX)** and **PA10
(USART1_RX)**, the instance's default mapping, so one cable carries the
debug port and the console. 115200 8N1:

```bash
brio console P
```

## What the silicon taught the stratum

- **The clock task must park on the HSI before it touches the PLL.**
  PLLMUL, PLLSRC and PLLXTPRE are writable only while the PLL is off,
  and the PLL refuses to stop while it is the system clock - so a tree
  that arrives at `init()` with the PLL already running takes every one
  of those writes in silence and keeps the rate it had, while every
  divisor in the program is computed for the rate it asked for.
  Measured both ways: the failure staged on purpose, and the fix
  recovering 144 MHz within a millisecond of re-entry.
- **The PLL's input divider for the HSI is in another block.** RCC says
  only which root feeds the PLL; whether the HSI arrives whole or
  halved is `EXTEN_CTR.HSIPRE`, which the RCC chapter never mentions
  and which is 0 out of reset - so a program that only wrote RCC
  registers would find every rate half of what it asked for.
- **The USB pads are still GPIO pads.** D- and D+ are PA11 and PA12 and
  the USB device controller reaches them through port A: with that
  port's clock closed - its reset state - the pull-up still works (it
  lives in EXTEN), so the host sees a device attach and starts
  enumerating, and then every packet fails. WCH's own `USB_Port_Set()`
  opens the port's clock and leaves the two pads floating inputs before
  it touches the pull-up.
- **PB1 is not the datasheet's 144 MHz.** The block diagram rates both
  peripheral buses at the core's own ceiling; WCH's own clock code sets
  PPRE1 = /2 at every rate it offers, and only the bare-HSE path leaves
  it undivided. This stratum caps PB1 at 72 MHz. Where the real ceiling
  lies between those two numbers is not known.
- **The reference manual's vector table is the union of four
  families.** Its table 9-2 names TIM8 at entries 59..62, which belong
  to the CH32V30x; the CH32V203's tail is USBFS, its wake-up, UART4 and
  the eighth DMA channel, and the CH32V203RB's is different again. The
  crt states the per-class truth
  ([startup_ch32v203.S](../../ch32v203/src/glue/startup_ch32v203.S)).
- **UART4's pads are the part's, not the family's.** The manual has two
  remap tables for it; the one that starts at PC10/PC11 belongs to the
  bigger classes, and the CH32V203C8 is named explicitly in the other,
  whose default pads are PB0 and PB1.
- **THE USB CONTROLLER DOES NOT SURVIVE THE CORE'S SLEEP**, and the
  manual says it should. Chapter 2's table gives Sleep as "core clock
  off, no effect on other clocks" and its prose as "the core stops
  running and all peripherals are still running"; measured here, with
  the core in WFI or in the platform's WFE idle, the controller cannot
  reach its packet memory. An armed bulk endpoint receives nothing -
  the host's bytes are lost, `cdc_rx` stays at zero and the
  packet-memory overflow counter climbs one per attempt - and an
  enumeration never gets past its first control transfer. The same
  image with a loop that only `step()`s enumerates, configures and
  carries bytes with not one overflow. Both idle forms fail and a
  spinning loop works, so it is the core being asleep and not the
  idiom. A program that uses USB therefore does not idle, and
  [usb_probe](../../ch32v203/src/apps/usb_probe.cpp) is the instrument
  that says so: it switches between SPIN, WFI and WFE from its own
  console while the host tries to enumerate.
- **`ebreak` with no debugger goes to the BREAKPOINT vector, not the
  exception one.** The table has both - the exception entry at index 3
  and the breakpoint one at index 9 - and the silicon takes the second,
  while mcause reports exception code 3, this core's number for a
  breakpoint: the vector index and the exception code are different
  numbers here. A program that wants a crash recorded binds both.
- **corecfgr (CSR 0xBC0) is zero at the reset vector.** The QingKe V4
  manual says it configures the pipeline and branch prediction, gives
  no bit table, and says the products set it in their startup file;
  WCH's own writes 0x1F, and so does this crt.

## Not covered yet

Driver gaps, each with its reason:

- **Every chapter but the six named above.** DMA, the timers, SPI, I2C,
  the ADC, the OPA, CAN, the RTC and the backup domain, PWR and the
  sleep modes, the flash engine, CRC, TKEY, EXTI, the watchdogs and the
  second USB block (USBFS, on PB6/PB7, a different peripheral): this
  target is at first light and each arrives with its own document and
  its own suite.
- **The remaps (AFIO_PCFR1/PCFR2)**: every driver is on its default
  pads, which is where this board's console is; they arrive with the
  first program that needs a moved pad.
- **The double-buffered and isochronous USB endpoints**: a CDC console
  needs neither, and the double buffer changes what DTOG means on every
  access.

Implemented but not bench-verified:

- **The eight other parts of the family.** Only the CH32V203C8 has a
  part table and a linker script; the others are named in the build's
  table and nothing has compiled for them. The family check fixture
  (`brio check ch32v203`) is not written.
- **The KEY button on PA0**: the vendor's claim, and the pad is free.
