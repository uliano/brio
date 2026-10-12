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
| [platform.md](platform.md) | Platform: `Ch32x035Platform` (the `csrrci` critical section, the WFE-shaped `idle()` that does not sleep over a working DMA channel, `ebreak`, the `.noinit` breadcrumb, the stack ledger), `Pfic` and `BRIO_CH32_INTERRUPT` with `no_icf`, the 64-bit STK `Ticker`, `delay_us`, and what the crt writes into the core - measured on the CH32X035F8U6 by `test_x035_platform` (51 verdicts): the WFE idle entered unmasked and the latch its wake leaves consumed - no edge position of 600 losing the wake or serving it a tick late, one kernel turn per tick where the unconsumed latch gave two (at 48 MHz and at 8 MHz), the STK's compare and its CNT arithmetic within 30 ppm of the interrupt count over 200 reloads, `delay_us` 112 to 124 cycles late and never early, an interrupt round trip of 100 to 104 cycles at 48 MHz with the hardware prologue, corecfgr's 0x1F making a timed loop 7.3 per cent SLOWER, INTSYSCR and mtvec as the crt wrote them and an MPP no MRET changes, and the reset flags, the signature and the chip-identifier word read at boot |
| [clock.md](clock.md) | RCC: ONE root (the 48 MHz HSI) and ONE divider (HPRE, /6 out of reset), the user trim, the flash's wait states sequenced around every change, the clock output on PB9 where the package bonds it, the gates, the resets and the reset flags; `Clock<internal, hz>` and `DynamicClock<Boot, Users...>` - measured on the CH32X035F8U6 by `test_x035_clock` (21 verdicts): the whole HPRE ladder from 48 MHz down to 3 MHz and back with the tick, a busy-wait and the console exact at every rung and nothing of the tree written but HPRE and the wait states, the trim a step each way and, against the desk's CH32V006K8U6 crystal, the HSI at -0.25 per cent and the trim's step at 116 to 118 kHz on average from 7 to 28, RCC_AHBPCENR's reset value, fifteen gates opened and closed; RMVF a LEVEL, and the power controller's reset line refused after a pulse on it cut the part off its debug port |
| [pwr.md](pwr.md) | PWR: the programmable voltage detector alone - PLS's four thresholds and PVD0, no enable bit - the check every image that drives a link pad makes at boot (3.0 V), reading "not low" on the CH32X035F8U6's 3.3 V rail |
| [pin.md](pin.md) | GPIO and AFIO: the F1's nibble with NO speed and NO open-drain output, 24 pins a port over THREE configuration registers, CFGHR written whole from a RAM copy as WCH's library does it, BSHR and BSXR for the two halves of a port, the pull in the output register and a pull-down on PA0..PA15, PC16 and PC17 alone, the bonding and the shorted pads refused at compile time, the one-way lock; the remap fields of AFIO_PCFR1 with the USARTs' columns as data, the debug port's field read and written only by a verb nobody reaches by accident, the EXTI port multiplexer - measured on the CH32X035F8U6 by `test_x035_pin` (19 verdicts in `z`, two more on its jumper): every nibble in each of the three registers, CFGHR reading back what its copy wrote on a die the vendor's caution is not for, the levels, BSXR on PC14, the pulls, a remap field, the debug port's field and the EXTI multiplexer's three codes, and the levels across a jumper; a port read through its shut gate answering with the last word the bus carried |
| [usart.md](usart.md) | The four USARTs: every one a full USART counting its divisor in HCLK, and which of them a part offers read off its pins (the QFN20 offers USART2, USART3 and USART4, and no USART1); the frame, mute, LIN, half duplex, IrDA, the smartcard, the synchronous clock and the flow-control pair as the resource's verbs, the chapter's exclusions refused; the `Uart` transport with the sibling strata's surface and a DMA engine in either slot, checked against the request table, the receive ring at `rx_priority`, the block reset in `release()` - measured on the CH32X035F8U6 by `test_x035_usart` (23 verdicts single-board, 80 against the desk's CH32V006K8U6 on its crystal across USART4's PB0/PB1): the divisor in HCLK, every frame and mode register by register, a frame's length and the break's timed, the debug port's column refused, half duplex deaf to its own frames and hearing a second chip's; across the wire every rate from 115200 to 3 Mbaud echoed and timed by the far receiver within 500 ppm of HSI / BRR, nine frame formats with half and one-and-a-half stop bits, PE, FE onset, glitches, ORE and the break from a crystal sender, mute woken both ways, IrDA, LIN's two detection lengths; the interrupt transport's sustained receive clean to 2 Mbaud and ON ITS ENGINES 8192 frames clean at every rate to 3 Mbaud, the wire's ceiling, two interrupts a burst and two a lap; the error flags' clear performed by the channel's read, the ring at very_high kept beside copies on a lower channel that starve it ranked low, and the transmit engine gapless to 1.5 Mbaud, its block boundary's idle above that explained by the restart's 372 cycles; a polled receiver losing a frame between its STATR and DATAR reads; and `usart_peer`, this part as the desk's USART instrument for the CH32V006's suite - calibrated by that suite's crystal, its echo 28 instructions a character with no call, its waveforms on TIM1, its collide a wired AND it reads back |
| [i2c.md](i2c.md) | I2C1: the F1's event machine on two vectors with no rise-time register, FREQ's 4..60 MHz window under a 48 MHz HCLK, six columns and which of them each package can use, the lot note asked of the block itself (`present()`), the host engine with the sibling strata's Request, the first two bytes in the ADDR entry, the repeated START on the last byte's TxE, engines on channels 6 and 7, the client polled or on its vectors - measured on the CH32X035F8U6 by `test_x035_i2c` (84 verdicts against the desk's CH32V006K8U6 running `twi_peer`, the bus pulled up from PB3): the block PRESENT, the scan, the shapes and the four receive procedures, the vocabulary, the rate asked at 100 and 400 kHz on the pump and on the engines, the kernel with a held SDA and a held clock, arbitration both ways and this board as a foreign host's client written to and read from (`flush()` dropping the byte asked beyond each closing NACK), the stuck bus, the repeated START against a polled CH32 target, the USB pads' pulls absent under the alternate function, and the cost - 184 cycles an entry a byte on the pump, three event entries and no DMA interrupt for an engined write of any length; and `twi_peer`, this part as the desk's I2C instrument for the CH32V006's suite, its polled target's habits measured from that host |
| [spi.md](spi.md) | SPI1: the F1's block with no FIFO, the chapter's two mode numberings and neither of them the driver's, HSRXEN and ODEN, four columns of which the QFN20 bonds one; the host engine with the sibling strata's Request, the transmit-only loop, the receive loop one or two frames in flight, the pump, engines on channels 2 and 3, the client's runs - measured on the CH32X035F8U6 by `test_x035_spi` (24 verdicts single-board, 32 against the desk's CH32V006K8U6 running `spi_peer`): the engines at the rate asked at every code to 24 MHz, HSRXEN's second ladder HCLK/(BR+2), nothing enable-protected, SSOE low from the first frame, ODEN open drain, both CRCs the arithmetic, MODF from a second chip; both hosts' waveforms the standard's in all four modes (this chip as the analyser), the CH32V006 client served in modes 0 and 3; this chip as a client receiving in every mode and order and answering in every mode MSb first and never LSb first; the costs beside the wire, the loops aligned so they do not move with the code around them; and `spi_peer`, this part as the desk's SPI client for the CH32V006's suite - the extended exchange served (16-bit frames, the CRC, a receive-only run), exact both ways to 6 MHz, and above that losing edges whenever its own MISO pad stands high (measured, not explained) |
| [tim.md](tim.md) | TIM1 and TIM2: one advanced-control block twice, in the subset a waveform engine needs - the time base, every compare mode, the complementary outputs behind MOE, one pulse, the compare's and the update's DMA request, the remap with both timers' columns as data - measured on the CH32X035F8U6 by `test_x035_tim` (33 verdicts, no wire of its own): CTLR1 reading 0x2000 out of reset, its "reserved" bit 13 deciding whether an output MOE disables floats or is driven, the outputs HOLDING while the counter is stopped and a never-run timer holding them low, a software compare event raising nothing, every request of figure 9-1, an edge list streamed by DMA to its intervals and kept down to edges ten cycles apart, one pulse to the cycle; and `usart_peer`'s waveforms on C2N and C3N |
| [dma.md](dma.md) | The DMA controller: eight channels where the channel is the request, the request table per part as the transports' check, the engines in two moments (five stores a block, six for the copy) - `DmaTxEngine`, `DmaRxEngine` with its circular ring, `DmaCopyEngine` - the levels by the rule - measured on the CH32X035F8U6 by `test_x035_dma` (52 verdicts, its own console on two engines): three widths at six cycles an item, EN standing after a block and a PADDR stored under it dropped, the eight vectors (the eighth at 44), no hole raising TEIF, circular laps paced by a USART, the arbiter strict, a raised request HELD until the block's reset, THE CONTROLLER IDLE IN SLEEP, and the benchmark row |
| [vendor/README.md](vendor/README.md) | The documents of record with their revisions - the reference manual V1.8, the datasheet V1.7, the QingKe V4 manual, the probe's manual, and the EVT package as the vendor's only voice on quirks; the no-errata statement; where the documents disagree, and which of those items the CH32X035F8U6 has answered |

[afio.hpp](../../brio/ch32x035/afio.hpp) and
[dma_engine.hpp](../../brio/ch32x035/dma_engine.hpp) have no document
of their own: the first is [pin.md](pin.md)'s second half, the second
the request table and the engine slot's vocabulary [dma.md](dma.md)
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
CH32X035F8U6 through a WCH-LinkE: every image the suites ran from was
written that way.

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
- **A lot may have no I2C**: table 2-1's note 3 (and the notes of 1.4.12
  and 1.4.16) - "the channel 3, channel 7, channel 11, channel 15 and
  I2C functions ... are not applicable to products with a lot number
  with the penultimate bit 5 being 0". Every part of the series, and no
  register names a lot: the I2C driver asks its block
  (`I2c<1>::present()`, and both `init()`s refuse a block that does not
  hold its timing, [i2c.md](i2c.md)); the ADC's four channels wait for
  that driver.
- **The EXTI's port codes are not the sibling families' order**:
  AFIO_EXTICR's two bits a line are 00 port A, 10 port B and 11 port C,
  with 01 reserved (8.3.2.2).

## What the silicon taught the stratum

What the CH32X035F8U6 answered on WCH's evaluation board - the suites'
`z` and the USART suite's `y`, each green whole, and a read over the
debug port; the numbers are each document's bench findings.

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
- **The WFE idle keeps the kernel's promise on the V4C.** `idle()`,
  called with interrupts masked as the kernel calls it, unmasks, sleeps
  as a WFE and consumes the latch its wake leaves: the STK's edge placed
  to the cycle across the whole path loses no wake and serves none a
  tick late, and a quiet kernel loop turns once per tick - twice with
  the latch left set, as the CH32V203's V4B did - the form the sibling
  strata use because the CH32V00x's WFI does not wake for an interrupt
  its core cannot take ([platform.md](platform.md)).
- **In the Sleep the DMA controller moves nothing.** RM 2.3.2 keeps the
  peripherals running in it, and USART3's transmitter kept asking: a copy
  started before a sleep stood still until the core woke, and a block
  paced at 1 Mbaud ran at a fiftieth of its pace - the CH32V203's and the
  CH32V303's answer. The platform's `idle()` does not sleep while a
  channel works, so a program with an engine at work - a receive ring
  above all - holds itself awake ([dma.md](dma.md),
  [platform.md](platform.md)).
- **A raised DMA request is held** until the channel takes it or the
  block's reset line is pulsed - its DMA enable, its enable and a closed
  gate notwithstanding - and the next owner of the channel takes one
  stray item: the release contract's case, which the `Uart`'s `release()`
  answers ([dma.md](dma.md)).
- **A PADDR stored with EN standing is dropped**, and EN stands after
  every completed block: the copy engine stores its source after the
  restart's disable ([dma.md](dma.md)).
- **The bench die has its I2C.** The lot note's question, put to the
  block over the debug port before any code and again by the I2C suite:
  FREQ, CKCFGR and OADDR1 hold what they are given and a START on the
  idle bus raises SB - whatever the unreadable marking says, this lot is
  not the one note 3 withdraws the I2C from ([i2c.md](i2c.md)).
- **The USB pads' pull-ups do not reach the alternate function.**
  AFIO_CTLR's 10 kOhm and 1.5 kOhm modes pull PC16 and PC17 up as pulled
  inputs and do nothing under I2C1's open-drain alternate function: an
  I2C on those pads wants resistors of its own, and on the evaluation
  board they cannot go to the supply - PC17 high at power-up is the
  loader's boot condition ([i2c.md](i2c.md)).
- **A port's output register behind a shut gate drops a store.** The
  same gate that makes a read meaningless swallows a write: a pad set
  high before its port's first configuration came up driving low, and
  `Pin::output(level)` and `Pin::input(pull)` open the gate before the
  output-data store ([pin.md](pin.md)).
- **The flash fetch bounds the core's loops, and placement moves it.**
  Code runs from a flash with two wait states at 48 MHz: the SPI's
  transmit loop takes 20.7 cycles a frame from it and 16.8 - the wire at
  /2 - from SRAM, the pump's handler half as long from SRAM, and a loop's
  cost a frame moved by up to a third between two builds whose code
  merely sat elsewhere; aligned to 32 bytes it does not ([spi.md](spi.md)).
- **HSRXEN is a second rate ladder.** With the bit set every BR code
  gives HCLK/(BR+2) - 16.3.1's note, against 16.3.8's "only valid at /2" -
  and the bit reads zero whatever was written ([spi.md](spi.md)).
- **A stopped timer holds its outputs, and a never-run one holds them
  low.** Chapter 12 is silent; with CEN clear every TIM1 and TIM2 output
  keeps driving the level it had, a forced level written meanwhile
  included, and the reset's level is low - so a waveform engine runs the
  counter before it hands a pad to the timer ([tim.md](tim.md)).
- **CTLR1's "reserved" bit 13 reads one and acts.** Set out of reset on
  both advanced timers, it lets an output MOE disables float; cleared,
  the output is driven at its idle level - the CH32V00x's OE_MODE
  position. The driver keeps it as it reads, as WCH's library does
  ([tim.md](tim.md)).
- **A software compare event does nothing to an output channel**: 12.4.6's
  CCxG raised no CCxIF and no DMA request on either timer; a match does
  both, and UG raises the update's request ([tim.md](tim.md)).
- **An edge list keeps edges ten HCLK cycles apart**: TIM1's compare
  request reloading CHxCVR through the DMA controller took every value of
  a list spaced 10 cycles and one of eight at 8 ([tim.md](tim.md)).
- **A polled receiver loses a frame that lands between its STATR and its
  DATAR reads** - with ORE cleared by the same sequence, unseen: measured
  at 3 Mbaud with a time stamp taken between the two ([usart.md](usart.md)).
- **The SPI client's restrictions are half true.** 16.2.3's "mode 2 or
  mode 3" for a transmitting client does not hold - this die answers in all
  four modes - and 16.3.1's "LSB is only supported by SPI as host" holds
  for what a client sends: LSb first its MISO never moves
  ([spi.md](spi.md)).
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
- **The power chapter's low-power half - the sleep modes, the
  auto-wakeup and a sleep site (RM ch. 2, 4)**: the voltage detector is
  [pwr.md](pwr.md)'s; this series has no RTC, so the AWU on the HSI divided by
  1024 is the only wake a Stop or a Standby has, and a sleep site is
  born with it as its wake; until then `idle()` is the plain Sleep and
  nothing in the stratum sets SLEEPDEEP.
- **The external interrupts (RM 7.4)**: the six EXTI registers are
  mapped and AFIO's port multiplexer has its verbs ([pin.md](pin.md));
  an `ExtInt` is born with its first user.
- **The ADC and the touch keys (RM ch. 10, 11), TIM3 (RM ch. 13), the
  amplifiers and comparators (RM ch. 17)**: born with their first user.
  TIM3's remap field is named in `afio.hpp`; its columns come with its
  driver. TIM1 and TIM2 are [tim.md](tim.md)'s, in the subset its gap
  list names.
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

- **What the suites have not measured**, each in its document's
  list with what would measure it: what the hardware prologue buys (the
  platform suite's letter `e` in an image built without it), the clock
  output's waveform (a package that bonds PB9, and a counter), the reset
  flags one event at a time, the idle hook's power, the lock (the pin
  suite's `l`, by name), and what the USART's smartcard, clock and
  flow-control pair do on a wire (a logic analyser, or pads this package
  does not bond).
- **The six parts other than the CH32X035F8U6.** Every one has its
  table, its linker script and its preset, and the whole stratum
  compiles for all seven both ways the hardware prologue can be built
  (`brio check ch32x035`); what would measure them is a board of each
  package.
