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
| [clock.md](clock.md) | RCC and the EXTEN bits that belong to the tree: the two high-speed roots, the PLL whose input divider is per DEVICE CLASS (the HSI's in EXTEN, the HSE's dividing by one or two here and by four or eight on the CH32V203RB) and whose input and output RANGES are part facts, the whole prescaler table with PB1 capped and the timers' doubling rule stated, the USB divider written before any USB gate can open, the ADC's divider - the one place a legal rate leaves a peripheral out of specification - the LSI, the clock security system as the non-maskable interrupt's body, the ready interrupts, the output pad two packages have not got, and the peripheral gates; `Clock` static and `DynamicClock` over a pack of rate tuples, every switch parking on the HSI. Measured: ten trees entered, read back and bracketed against the host's clock (every HSI-rooted rate 0.42 to 0.46 % fast, every crystal-rooted one within 0.05 % of exact), the pack walked up and down with the tick and the console rebased at each step, the LSI ready 3.1 ms after LSION and the board's crystal 1.7 ms after HSEON with its ready interrupt reaching the RCC vector, the HSI stopped under a tree that runs off the crystal, the security system armed over a healthy crystal, and twenty-six gates opened and closed - the bits of blocks this part has not got reading back zero |
| [pin.md](pin.md) | GPIO, the remaps and the EXTI: the F1's four-bit nibble over two registers with a MODE that carries a speed, the pull that lives in the output register and belongs to input mode alone, the whole-port verbs and the one-way configuration lock; the remap columns of AFIO_PCFR1/PCFR2 as constexpr pad tables judged by the DEVICE CLASS and by this package's bonding, the debug port's own field read and never written, the event output whose source no document of this family names; and the twenty-two EXTI lines - sixteen pin lines through AFIO_EXTICR with the one-pad-per-line rule refused by `select()` and overridden by `steal()`, the PVD's, the RTC alarm's and the two USB wake-ups, the senses, the two enables, the software trigger and the write-one flags over five single vectors and two shared ones. Measured: all fifteen configurations read back in both registers, a reset port's unbonded nibbles reading ZERO, an open-drain output that drives nothing high, nine remap columns written and restored and two the silicon holds at zero against the manual's own notes, five/five/ten edges of a pad on its own line, and a line event ending idle() in 15 core cycles |
| [tim.md](tim.md) | The timers: one advanced-control block and three general-purpose ones (the 32-bit TIM5 the 128 KB part's alone, no basic timer anywhere in the series), the F1's register file under WCH's names - the two shadow registers, the four channels in both faces with CCyS writable only off, the slave controller with its three encoder modes and the master TRGO, the internal trigger table folded through what the part HAS, TIM1's repetition counter, complementary outputs, dead-time generator and break input, the DMA burst engine as data for the chapter that will use it, and TIM1's FOUR unshared vectors against one line for every other timer; the pads taken from afio.hpp's remap columns, and the nine tasks. Measured: the counter and both shadows against the core's counter, a PWM captured by its own timer to the microsecond at four duties, a dead band of 27.9 us against 28 asked, the interval and period meters behind a MeterLatch, two timers counting each other over an internal trigger (201 updates, a 25 % duty gated to 5000 us), a one pulse 501 us wide for 500 asked, forty encoder counts for ten quadrature cycles - and two facts the relatives do not share: a pad in PLAIN OUTPUT mode reaches a timer's capture input, and in an encoder mode a channel's output stage does not reach its pad |
| [watchdog.md](watchdog.md) | The two watchdogs: the independent one on the LSI (three keys, no way back but a reset, the oscillator forced on - and its two registers taking a write only while that oscillator RUNS, which is why arm() starts before it configures and ends with the refresh that re-locks them) and the window one on PCLK1/4096/2^WDGTB (the counter that does not run unarmed against its own chapter, the clock gate that holds the counter while the registers still read, the window whose early refresh IS the reset, the early wake-up one tick before it, the block's reset line as the only way back). Measured: the tick at three prescalers to the microsecond (2628/5346/21389 us for 47 ticks), EWIF at 28671 us of 28672, three real WWDG resets (29 ms, 0 ms for an early refresh, 29 ms with the interrupt having run once) and the IWDG's own at 206 and 207 ms - which puts this die's LSI at 38.8 kHz |
| [usart.hpp](../../brio/ch32v203/usart.hpp) | USART: the frame, the divisor, the flags and the two rings of the transport; measured: the console at 115200 with the divisor exact against PCLK2 |
| [usb.hpp](../../brio/ch32v203/usb.hpp) | USBD: ST's device controller under WCH's names, realizing util/usb's UsbController; measured: the kernel console over a CDC ACM port on the board's own USB-C, enumerated, configured and carrying bytes with no overflow - and the core must not sleep while it does (below) |
| [vendor/README.md](vendor/README.md) | The documents of record with their revisions - the reference manual that covers four families and the CLASS RULE that divides them, the datasheet's table 2-1 and pin tables, the QingKe V4 manual, the probe's, and the EVT as the vendor's only voice on quirks; the no-errata statement |

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
- **The PLL's input divider is in another block - and on the other class it
  is not even the same divider.** RCC says only which root feeds the PLL;
  whether the HSI arrives whole or halved is `EXTEN_CTR.HSIPRE`, which the
  RCC chapter never mentions and which is 0 out of reset - so a program that
  only wrote RCC registers would find every rate half of what it asked for.
  The HSE's own divider IS an RCC bit (PLLXTPRE), and what it divides BY
  follows the device class: one or two on every part up to the CH32V203C8,
  four or eight on the CH32V203RB, whose oscillator is 32 MHz.
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
  WCH's own reference implementation fails the same way, which is
  what settles whose the finding is: the EVT's SimulateCDC example,
  built with the vendor's compiler and flags and unchanged but for
  one `wfi` at the end of its loop, never reaches CONFIGURED -
  SET_ADDRESS gets through, the descriptor reads do not, the overflow
  counted 18 to 26 times - and the vendor's own `__WFE()` fails the
  same, while a busy-wait of the same length in the same loop works;
  a variant that sleeps only once configured loses every one of the
  4096 bytes the host writes; and the loss happens with the core
  woken at least once per USB frame (the frame interrupt at 1 kHz
  and a 10 kHz timer armed), so it is neither the depth nor the
  length of the sleep. Measured on the host-to-device direction; the
  absence of an erratum is, once again, evidence of nothing. The
  debug module's keep-HCLK-in-Sleep bit could not be tried: a `csrw`
  to CSR 0x7C0 from the running program resets the part.
- **In Sleep, no bus master but the core gets a cycle.** A memory-to-
  memory DMA started right before a `wfi` moves nine to twelve bytes -
  the pipeline between the enable and the sleep - and nothing more
  until the core wakes, while a timer on the peripheral bus and the
  core's own counter count the whole sleep; woken once a millisecond
  it moves eleven bytes a wake. The clocks run; the bus matrix serves
  the core alone. The USB controller's reach into its packet memory
  is such an access, which is why a transfer with a payload fails in
  Sleep and one without (SET_ADDRESS) passes - and why no software
  mitigation short of staying awake works, measured on the vendor's
  own example: unmasking the overflow interrupt wakes the core but
  the packet is gone and the host does not retry a failed bulk burst;
  staying awake for ten or a hundred milliseconds after the overflow
  recovers nothing of the burst in flight; holding the endpoint at
  NAK across the sleep still overflows; re-validating the endpoint
  finds it valid. What does work is not sleeping and dividing HCLK
  instead: the device enumerates and carries data with HCLK at 96, 48
  and 24 MHz and the USB clock at 48, and fails at 12 MHz with the
  sleeping case's own signature. So on this family a program that
  moves data through the bus - USB, a DMA-fed transport - does not
  sleep; it slows down, to no less than 24 MHz of HCLK.
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

- **Every chapter but the eight named above.** DMA, SPI, I2C,
  the ADC, the OPA, CAN, the RTC and the backup domain, PWR and the
  sleep modes, the flash engine, CRC, TKEY, and the
  second USB block (USBFS, on PB6/PB7, a different peripheral): this
  target is at first light and each arrives with its own document and
  its own suite.
- **The double-buffered and isochronous USB endpoints**: a CDC console
  needs neither, and the double buffer changes what DTOG means on every
  access.

Implemented but not bench-verified:

- **The eight parts other than the CH32V203C8.** Every one has its
  table, its linker script and its preset, and the whole stratum
  compiles for all nine both ways the hardware prologue can be built
  (`brio check ch32v203`); the CH32V203C6 preset links every image
  and is the 32 KB / 10 KB tier's guard, and the CH32V203RB's console
  links with its seventy-word vector table. What would measure them is
  a board: only the C8 exists on the desk.
- **The KEY button on PA0**: the vendor's claim, and the pad is free.
