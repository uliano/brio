# Target: CH32X035 (`ch32x035/`)

The operational page for the CH32X035 target: WCH's **QingKe V4C**, a
RISC-V core with the **RV32IMAC** instruction set - the CH32V203's core
generation with a five-cycle divider and a physical memory protection
unit (the reference manual's core table, the QingKe V4 manual's table
1-1) - on a series of seven parts, six CH32X035 and the CH32X033F8P6:
one die with 62 KB of code flash and 20 KB of SRAM, one 48 MHz internal
oscillator as its only clock root, and seven packages. The bench part is
the **CH32X035F8U6** (QFN20) on WCH's evaluation board
([../boards/ch32x035-evt-f8u6.md](../boards/ch32x035-evt-f8u6.md)). What
this target exists to keep true is what every other one does: the
kernel and util strata compile here unchanged.

Two things shape the stratum and are stated up front. **There is no
vendor header in the build**, for the reason the two sibling WCH strata
give (WCH ships its register definitions inside the EVT package, whose
licence is written for software running on WCH parts): the map is
[brio/ch32x035/device.hpp](../../brio/ch32x035/device.hpp), read off the
reference manual and cross-checked against the EVT's header, and every
per-part fact is stated in [parts/](../../brio/ch32x035/parts/). And
**this is not a CH32V203 with fewer pins**: a GPIO port runs to 24 pins
over three configuration registers with no speed and no open-drain
output, there is no PLL, no crystal oscillator and no peripheral-bus
prescaler, and beside the CH32V303's USB host/device controller the
series carries two blocks no other stratum drives - a USB PD controller
and the PIOC co-processor.

A statement of behaviour in this folder is the reference manual's, the
datasheet's, the vendor library's, or a sibling stratum's measurement
named as such, unless it is marked measured: what is measured is the
CH32X035F8U6's, with its numbers in each document's bench findings, and
each document's last list names what is still owed and what would
measure it.

## The documents

One document per peripheral driver is the shape
[../README.md](../README.md) prescribes.

| Driver | State |
|--------|-------|
| [device.hpp](../../brio/ch32x035/device.hpp) + [parts/](../../brio/ch32x035/parts/) | the register map and the part table: the seven parts, each with its memories, its package, the pads it bonds and the pads it SHORTS together inside (the datasheet's notes 4 to 7), the USARTs it offers beside the model table's count, and what else it carries; RCC, GPIO, AFIO, the USART and the core's STK and PFIC mapped for the drivers, the EXTI and PWR mapped for the reader, the flash interface's option bytes and the electronic signature decoded READ-ONLY; the interrupt numbers are the vector table's word indices |
| [platform.md](platform.md) | Platform: `Ch32x035Platform` (the `csrrci` critical section, the WFE-shaped `idle()`, `ebreak`, the `.noinit` breadcrumb, the stack ledger), `Pfic` and `BRIO_CH32_INTERRUPT` with `no_icf`, the 64-bit STK `Ticker`, `delay_us`, and what the crt writes into the core - measured on the CH32X035F8U6 by `test_x035_platform` (42 verdicts): the WFE idle, called masked, waking on the tick, the STK's compare and its CNT arithmetic within 30 ppm of the interrupt count over 200 reloads, `delay_us` 112 to 124 cycles late and never early, an interrupt round trip of 100 to 104 cycles at 48 MHz with the hardware prologue, corecfgr's 0x1F making a timed loop 7.3 per cent SLOWER, INTSYSCR and mtvec as the crt wrote them and an MPP no MRET changes, and the reset flags, the signature and the chip-identifier word read at boot |
| [clock.md](clock.md) | RCC: ONE root (the 48 MHz HSI) and ONE divider (HPRE, /6 out of reset), the user trim, the flash's wait states sequenced around every change, the clock output on PB9 where the package bonds it, the gates, the resets and the reset flags; `Clock<internal, hz>` and `DynamicClock<Boot, Users...>` - measured on the CH32X035F8U6 by `test_x035_clock` (21 verdicts): the whole HPRE ladder from 48 MHz down to 3 MHz and back with the tick, a busy-wait and the console exact at every rung and nothing of the tree written but HPRE and the wait states, the trim a step each way, RCC_AHBPCENR's reset value, fifteen gates opened and closed; RMVF a LEVEL, and the power controller's reset line refused after a pulse on it cut the part off its debug port |
| [pin.md](pin.md) | GPIO and AFIO: the F1's nibble with NO speed and NO open-drain output, 24 pins a port over THREE configuration registers, CFGHR written whole from a RAM copy as WCH's library does it, BSHR and BSXR for the two halves of a port, the pull in the output register and a pull-down on PA0..PA15, PC16 and PC17 alone, the bonding and the shorted pads refused at compile time, the one-way lock; the remap fields of AFIO_PCFR1 with the USARTs' columns as data, the debug port's field read and written only by a verb nobody reaches by accident, the EXTI port multiplexer - measured on the CH32X035F8U6 by `test_x035_pin` (21 verdicts): every nibble in each of the three registers, CFGHR reading back what its copy wrote on a die the vendor's caution is not for, the levels, BSXR on PC14, the pulls, a remap field, the debug port's field and the EXTI multiplexer's three codes, and the levels across a jumper; a port read through its shut gate answering with the last word the bus carried |
| [usart.md](usart.md) | The four USARTs: every one a full USART counting its divisor in HCLK, and which of them a part offers read off its pins (the QFN20 offers USART2, USART3 and USART4, and no USART1); the frame, mute, LIN, half duplex, IrDA, the smartcard, the synchronous clock and the flow-control pair as the resource's verbs, the chapter's exclusions refused; the `Uart` transport with the sibling strata's surface, its engine slots taking the empty tag alone - measured on the CH32X035F8U6 by `test_x035_usart` (27 verdicts): the divisor in HCLK, every frame and mode register by register with the chapter's exclusions refused, a frame's length and the break's timed, the debug port's column refused, and USART4 across a jumper at 115200, 1 Mbaud and 3 Mbaud with 8E1, 7O1 and 9N1 frames and a LIN break detected |
| [vendor/README.md](vendor/README.md) | The documents of record with their revisions - the reference manual V1.8, the datasheet V1.7, the QingKe V4 manual, the probe's manual, and the EVT package as the vendor's only voice on quirks; the no-errata statement; where the documents disagree, and which of those items the CH32X035F8U6 has answered |

[afio.hpp](../../brio/ch32x035/afio.hpp) and
[dma_engine.hpp](../../brio/ch32x035/dma_engine.hpp) have no document
of their own: the first is [pin.md](pin.md)'s second half, the second
the empty engine slot and the request channels [usart.md](usart.md)
names.

## Toolchain

WCH's own `riscv32-wch-elf` gcc 15.2.0 at `/sw/wch-riscv`, the compiler
of the two sibling WCH strata and for their reason: it is the one that
emits WCH's `xw` compressed extension, which the QingKe V4C carries (the
QingKe V4 manual's table 1-1). The ISA and the ABI are the CH32V203's,
`-march=rv32imac_xw -mabi=ilp32`, which that gcc resolves to its
`rv32imac_zaamo_zalrsc_xw/ilp32` multilib - two columns of each part's
row in the part table, one value throughout. The core's hardware
prologue is the project's `CH32X035_HPE` option, ON by default: the crt
sets INTSYSCR.HWSTKEN and `BRIO_CH32_INTERRUPT` becomes WCH's fast
attribute, one switch for the whole image ([platform.md](platform.md)).

## Board and build

WCH's CH32X035 evaluation board in its QFN20 edition, a CH32X035F8U6
([../boards/ch32x035-evt-f8u6.md](../boards/ch32x035-evt-f8u6.md)): no
crystal - the series has no oscillator for one - no reset button - the
QFN20 has no reset pin - two LEDs that reach a pad only through a
jumper, the chip's USB on a USB-C connector, and every bonded pad on two
7x2 headers.

```bash
(cd ch32x035 && cmake --preset ch32x035f8-release)       # configure (once, or after adding an app)
(cd ch32x035 && cmake --build --preset ch32x035f8-release --target console)
(cd ch32x035 && cmake --build --preset ch32x035f8-release --target console-upload)
brio flash <board> console  # the same thing through the bench's one command
```

The part number selects the part definition `device.hpp` asks for, the
linker script, the board type, the ISA and the ABI, from one table
([cmake/ch32x035-parts.cmake](../../ch32x035/cmake/ch32x035-parts.cmake)):
the seven parts of the datasheet's model table, whether or not a board
exists for them, each with a release preset and the CH32X035F8 with the
one debug preset. The two 28-pin parts share a number and not a bonding,
so their keys carry the package's letter (`ch32x035g8u`, `ch32x035g8r`).

The image is linked at `0x0000 0000`, the alias of the code flash at
`0x0800 0000` that the core boots from, and it gets the whole 62 KB: no
zone is set aside for a flash store here, which is the NV stack review's
question and not this target's
([ld/ch32x035f8.ld](../../ch32x035/ld/ch32x035f8.ld)). Every part has
the same two memories, so the family's smallest chip is every chip and
no suite splits into groups: a `// build: groups` line is read for the
roster and changes nothing ([../design/overview.md](../design/overview.md),
"A suite's image fits the family's smallest chip").

WCH's EVT package ships no SVD for this series, so the project has no
`svd/` and the debugger no peripheral view.

## The probe and the upload

A WCH-LinkE over the **two-wire** debug port this series has - **PC18 =
SWDIO, PC19 = SWCLK** (the WCH-Link user manual's table 6) - which the
board brings to its header P1, pins 6 and 8. WCH's OpenOCD fork at
`/sw/wch-openocd` (v2.10 by its own `version.txt`) is the one OpenOCD
that speaks the probe's SDI transport. Its `wch-riscv.cfg` selects the
`wlinke` adapter at 6000 kHz, the `sdi` transport, a `wch_riscv` target
and a flash bank at address 0 whose size is 0 - found by probing the
chip - and its binary carries a `ch32x` flash driver and the names
CH32X033, CH32X034 and CH32X035. It programs and verifies the
CH32X035F8U6 through a WCH-LinkE: every image the four suites ran from
was written that way.

The upload target and `brio flash` both write the image with
`program ... verify` and start it with `reset halt` followed by
`resume`: on the CH32V203 the fork's `reset run` leaves the hart at the
reset vector ([../ch32vx03/README.md](../ch32vx03/README.md)), and the
pair that starts the program there starts it here too - whether
`reset run` would on this part is not measured. `brio flash` names the
probe by the USB serial the manifest gives; the CMake upload target, as
on the sibling projects, takes the one probe attached.

## Serial console

**USART2 on PA2 (TX) and PA3 (RX)**, the board's "Serial port 2" (P1
pins 7 and 5), wired to the probe's own serial, so one cable carries
the debug port and the console. 115200 8N1:

```bash
brio console <board>
```

WCH's own examples print on USART1's default pads, PB10 and PB11. The
QFN20 bonds PB11 and not PB10, and no column of USART1 has both its TX
and its RX pads on this package, so the part table does not offer USART1
on it at all ([usart.md](usart.md)).

A program that enables a line and binds no handler for it looks DEAD:
the crt's weak default handler is a spin loop, with no fault and no
message. The exception entry and the breakpoint entry have weak spins of
their own, so a probe that halts the core names the wreck from the
program counter - `default_handler`, `fault_handler` or
`breakpoint_handler` - and `mcause` gives the interrupt number or the
exception code.

## What the documents say that shapes the stratum

What the reference manual, the datasheet and the vendor's library say
that the stratum is built around; what the CH32X035F8U6 answered is the
next section, and what is still owed the list at the end of this page.

- **One root and no bus prescaler.** SYSCLK is the 48 MHz HSI and
  nothing else (RM 3.3); HCLK is SYSCLK through HPRE, which resets to
  /6 - the chip wakes at 8 MHz - and every peripheral, the four USARTs
  included, runs at HCLK (14.3). So a clock type is a rate and a
  divider, and `pclk_hz` is `hz` ([clock.md](clock.md)).
- **HPRE's own register asks for a prefetch buffer that has no bit.**
  RM 3.4.2 closes the divider's description with "when the prescaler
  factor of the HB clock source is greater than 1, the prefetch buffer
  must be turned on"; no register of the manual has such a bit - the
  flash's access control register holds the wait states alone (20.3.1) -
  and WCH's own clock code divides HCLK without touching anything else.
  The driver does what the vendor does.
- **A port runs to 24 pins, over three configuration registers**:
  CFGLR, CFGHR and CFGXR (8.3.1.1, 8.3.1.2, 8.3.1.8), with BSHR setting
  and resetting pins 0..15 and BSXR pins 16..23 (8.3.1.5, 8.3.1.9), so a
  mask across both halves is two stores ([pin.md](pin.md)).
- **WCH's library does not read CFGHR on some dies.** Where the word at
  0x1FFFF704 - which that library reads as the chip identifier - has
  zero in bits 7:4, it keeps a RAM copy of each port's CFGHR and writes
  the register whole from it; the reference manual says nothing. This
  stratum does the same on every die ([pin.md](pin.md)).
- **A pin has no speed and no open drain.** MODE's three non-zero codes
  are all "output" and CNF names push-pull and alternate push-pull alone
  (8.3.1.1); the pull is the output register's bit under a pulled input,
  and only PA0..PA15, PC16 and PC17 have a pull-down (the opening of
  RM ch. 8).
- **Some package pins carry two pads.** On every part but the
  CH32X035F8U6, PC16 shares its pin with PC11 and PC17 with PC10; on the
  two 28-pin parts PB1 shares one with PB5; on the QSOP28 PA12 shares
  one with PC14 and PA13 with PC15; on the CH32X033F8P6 PA7 with PB0 -
  and the datasheet forbids both pads of such a pair as outputs (table
  2-1, notes 4 to 7). The part table names them, and an output on one
  does not compile.
- **The debug port's two pads are GPIO pads a remap can reach.** PC18
  and PC19 are SWDIO and SWCLK from reset, AFIO_PCFR1.SW_CFG hands them
  back to GPIO (8.3.2.1), and USART3's column 1 and USART4's columns 4
  and 6 put a USART signal on them: the transport refuses those columns
  while the probe's port is alive, and the one verb that writes SW_CFG
  is `disable_debug_port_until_reset()`.
- **One vector table for the series**: 55 words, seven core entries and
  thirty-nine peripheral ones with entry 19 reserved (RM table 7-1), the
  same on every part whatever its package brings out, and WCH's own
  startup file for the series word for word. mtvec takes it with both
  mode bits, absolute and vectored (7.5.3.2).
- **The crt parts company with WCH's in two places.** WCH's startup
  file writes INTSYSCR = 3 - the hardware stack AND two-level nesting -
  and enters main() through an mret with mstatus 0x88, whose MPP of zero
  is user mode; this crt writes the hardware stack alone (nesting never:
  [../design/kernel.md](../design/kernel.md), section 1) and calls
  main() in machine mode with interrupts masked (mstatus 0x1880).
  corecfgr gets WCH's 0x1F, which neither manual explains
  ([platform.md](platform.md)).
- **The USB blocks' clocks are on out of reset**: RCC_AHBPCENR resets to
  0x00021004 - the USB PD controller's gate, the USB host/device
  controller's, and SRAMEN, the SRAM's clock in Sleep (3.4.5).
- **The EXTI's port codes are not the sibling families' order**:
  AFIO_EXTICR's two bits a line are 00 port A, 10 port B and 11 port C,
  with 01 reserved (8.3.2.2).

## What the silicon taught the stratum

What the CH32X035F8U6 answered on WCH's evaluation board - the four
suites' `z`, each green whole, and a read over the debug port; the
numbers are each document's bench findings.

- **RMVF is a level, not a pulse.** Written 1 it reads back 1 -
  RCC_RSTSCKR read 0x01000000 once the flags had cleared - where the
  CH32V203's bit clears itself; `Rcc::clear_reset_flags()` writes it and
  then clears it, and after that the register reads zero
  ([clock.md](clock.md)).
- **A reset pulse on the power controller's line cut the part off its
  debug port.** Pulsed through RCC_APB1PRSTR.PWRRST after every gate of
  the chapter had opened and closed, it silenced the console before the
  next line drained, and the debug port answered "failed to connect"
  until the supply was cycled - on a package with no reset pin, where a
  hand on the supply is the only way back. Observed once and not
  repeated on purpose: `Rcc::reset()` refuses that line and answers
  false ([clock.md](clock.md)).
- **A port read through its shut gate answers with the last word the bus
  carried.** Port A's clock gate is closed out of reset, and its INDR,
  read over the debug port while the gate was shut, returned the last
  word the bus had moved and not the pads' levels. A pad is read after
  its port is configured: every configuring verb of `Pin` and `Port`
  opens the gate first, and a read does not ([pin.md](pin.md)).
- **Of the manual's two reset values for RCC_RSTSCKR, 3.4.8's is the
  QFN20's.** The boots measured read SFTRSTF and PORRSTF, a power-on and
  the probe's resets behind them, and never PINRSTF: the register
  table's 0x0C000000, with PINRSTF beside PORRSTF, is not this
  package's, which has no reset pin ([clock.md](clock.md)).
- **The vendor library's CFGHR caution is not this die's.** The word at
  0x1FFFF704 reads 0x035E0611 - the library's 0x035E06x1 for the
  CH32X035F8U6, with 1 in bits 7:4 - so WCH's own `GPIO_Init` reads
  CFGHR here, and the register reads back what the stratum's copy wrote
  for every nibble of the series. The stratum configures CFGHR from its
  copy on every die regardless, for the dies with zero there
  ([pin.md](pin.md)).
- **A divided HCLK needs nothing but HPRE.** With nothing of the tree
  written but HPRE and the flash's wait states - no prefetch buffer, the
  bit 3.4.2 asks for being in no register - the clock suite's ladder ran
  the tick, a busy-wait and the console exact at every rung from 48 MHz
  down to 3 MHz and back, the console's divisor recomputed as HCLK /
  baud at each: no bus prescaler stands between the clock and a USART
  ([clock.md](clock.md)).
- **corecfgr's 0x1F makes this core slower.** The crt writes WCH's
  value, and a timed loop with a load and a data-dependent branch runs
  7.3 to 7.4 per cent slower with it than with the register cleared,
  where the CH32V203's V4B gained two cycles in 36811 from the same bits
  and the CH32V303's V4F lost half to eight tenths of a per cent
  ([platform.md](platform.md)).
- **The WFE idle wakes on the V4C.** `idle()`, called with interrupts
  masked as the kernel calls it, returns on the next tick with them
  enabled - the form the sibling strata use because the CH32V00x's WFI
  does not wake for an interrupt its core cannot take
  ([platform.md](platform.md)).
- **Where the suites reached it, the documents' word held**: the USB
  blocks' gates open out of reset (RCC_AHBPCENR 0x00021004), the three
  configuration registers and BSXR, the EXTI multiplexer's 00, 10 and
  11, the debug port's field at 0 and the transport's refusal of the
  columns on its pads, INTSYSCR and mtvec as the crt writes them and
  main() in machine mode, and the vector table's entries for the STK
  (12), the software interrupt (14), USART2 (39) and USART4 (43).

## Not covered yet

Driver gaps, each with its reason:

- **The failing half of the platform - `Reset`, the fault record and
  the two watchdogs (RM ch. 5, 6)**: born with their first user, the
  watchdog chapter, in one file as on the sibling strata. Until then the
  reset flags are `Rcc`'s verbs ([clock.md](clock.md)), and a trap
  lands in the crt's weak spins and leaves no record for the next boot.
- **The power chapter - PWR, the auto-wakeup and a sleep site (RM
  ch. 2, 4)**: this series has no RTC, so the AWU on the HSI divided by
  1024 is the only wake a Stop or a Standby has, and a sleep site is
  born with it as its wake; until then `idle()` is the plain Sleep and
  nothing in the stratum sets SLEEPDEEP.
- **The DMA (RM ch. 9)**: born with its first user. The USART's engine
  slots take `NoDmaEngine` alone, and the channel each USART direction
  requests on is stated beside it
  ([dma_engine.hpp](../../brio/ch32x035/dma_engine.hpp)).
- **The external interrupts (RM 7.4)**: the six EXTI registers are
  mapped and AFIO's port multiplexer has its verbs ([pin.md](pin.md));
  an `ExtInt` is born with its first user.
- **The ADC and the touch keys (RM ch. 10, 11), the timers TIM1, TIM2
  and TIM3 (RM ch. 12, 13), the amplifiers and comparators (RM
  ch. 17)**: born with their first user. The timers' remap fields are
  named in `afio.hpp`; their columns come with the driver.
- **I2C and SPI (RM ch. 15, 16)**: a peer on the wire, or the board's
  own jumpers, and a first user.
- **The USB host/device controller (RM ch. 18)**: the CH32V303's block,
  which [brio/ch32vx03/usbfs.hpp](../../brio/ch32vx03/usbfs.hpp) drives
  in device mode; a second family carrying it is the moment an IP
  stratum is factored out of that file, under the byte-identity gate,
  and that is a change of its own.
- **The USB PD controller (RM ch. 21)**: a PD source on the other end
  of the cable, and a first user.
- **The PIOC (RM ch. 22)**: an 8-bit co-processor with an instruction
  set of its own, whose programs want an assembler - a project of its
  own, the RP2040's PIO assembler the model.
- **Flash programming and the option bytes (RM ch. 20)**: decoded
  read-only; the engine is born with the NV stack review's answer for
  this series, and the linker gives the image the whole array until
  then.
- **The reset pin's configuration** (the RST_MODE option bits, and the
  RST function the datasheet gives PA21, PC3 or PB7 by package):
  decoded read-only with the option bytes; the QFN20 has no reset pin.
- **The debug module's freeze bits (DBGMCU_CR, CSR 0x7C0, RM 23.2.1)**:
  read by nothing and written by nothing - on the CH32V203 a `csrw` to
  that CSR from the running program resets the part
  ([../ch32vx03/sleep.md](../ch32vx03/sleep.md)).
- **A `qingke/` core stratum**: `pfic.hpp`, `ticker.hpp`,
  `platform.hpp` and `delay.hpp` are the third copy of files the
  CH32V00x and the CH32V203/CH32V303 strata hold, kept as close to the
  latter's as the silicon allows; folding the three into one is a
  change of its own, made with every image of the three strata
  byte-identical across it.

Implemented but not bench-verified, each with what would measure it:

- **What the four suites have not measured**, each in its document's
  list with what would measure it: what the hardware prologue buys (the
  platform suite's letter `e` in an image built without it), the HSI's
  accuracy (a host that time-stamps the brackets of the platform suite's
  `g` and the clock suite's `d`), the trim's step in hertz and the clock
  output's waveform (a package that bonds PB9, and a counter), the reset
  flags one event at a time, the idle hook's power, the lock (the pin
  suite's `l`, by name), half a stop bit, and what the USART's modes do
  on a wire (a logic analyser, or a letter across the jumper).
- **The six parts other than the CH32X035F8U6.** Every one has its
  table, its linker script and its preset, and the whole stratum
  compiles for all seven both ways the hardware prologue can be built
  (`brio check ch32x035`); what would measure them is a board of each
  package.
