# Target: CH32X035 (`ch32x035/`)

The operational page of brio's CH32X035 target, for a programmer who
has WCH's CH32X035 evaluation board on the desk and wants to build,
flash and debug a brio program on it, and then find the document of the
peripheral the program needs. It says what the stratum is, which board
and part it is proven on, how the toolchain, the build, the upload and
the console work and where each has a trap, what the documents and the
silicon taught the stratum, what is not covered yet, and it ends with
the map of the stratum's own documents: one line per peripheral driver,
which is where the state of the driver work is read. The platform, the
clock, the pins, the USARTs, the I2C, the SPI, two timers and the DMA
have their documents and their suites, and the rest is "Not covered
yet" at the end; the target's maturity is stated in the root README's
table alone.

A stratum is a directory of `brio/` holding one architectural layer;
`brio/ch32x035/` holds everything that knows WCH's CH32X035 register
map. The core is WCH's QingKe V4C, a RISC-V core with the RV32IMAC
instruction set: the CH32V203's core generation with a five-cycle
divider and a physical memory protection unit (the reference manual's
core table, the QingKe V4 manual's table 1-1). The series is seven
parts, six CH32X035 and the CH32X033F8P6: one die with 62 KB of code
flash and 20 KB of SRAM, one 48 MHz internal oscillator as its only
clock root, and seven packages. The bench part is the CH32X035F8U6
(QFN20) on WCH's evaluation board. What this target exists to keep true
is what every other one does: the kernel (`kernel/`) and the services
(`util/`) compile here unchanged. There is no maintained teaching
example in this project: the console and the test suites are its
applications, and `blink` here is a stated gap.

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
series carries two blocks no other stratum drives, a USB PD controller
and the PIOC co-processor.

A statement of behaviour in this folder is the reference manual's, the
datasheet's, the vendor library's, or a sibling stratum's measurement
named as such, unless it is marked measured: what is measured is the
CH32X035F8U6's, with its numbers in each document's bench findings, and
each document's last list names what is still owed and what would
measure it. The documents of record and their revisions are in
[vendor/README.md](vendor/README.md).

## The board

WCH's CH32X035 evaluation board in its QFN20 edition, a CH32X035F8U6
([../boards/ch32x035-evt-f8u6.md](../boards/ch32x035-evt-f8u6.md)): no
crystal (the series has no oscillator for one), no reset button (the
QFN20 has no reset pin), two LEDs that reach a pad only through a
jumper, the chip's USB on a USB-C connector, and every bonded pad on two
7x2 headers. The apps run the 48 MHz HSI undivided, and the console is
USART2 on PA2/PA3 on the probe's own serial. On the desk the board is
wired to a CH32V006K8U6, the peer of its two-board suites
([../boards/ch32v006k8.md](../boards/ch32v006k8.md)).

## Toolchain

The compiler is WCH's `riscv32-wch-elf` gcc 15.2.0 at `/sw/wch-riscv`,
the compiler of the two sibling WCH strata and for their reason: it is
the one that emits WCH's `xw` compressed extension, which the QingKe
V4C carries (the QingKe V4 manual's table 1-1,
[../ch32v00x/README.md](../ch32v00x/README.md), "Toolchain"). The ISA
and the ABI are the CH32V203's, `-march=rv32imac_xw -mabi=ilp32`,
which that gcc resolves to its `rv32imac_zaamo_zalrsc_xw/ilp32`
multilib; they are two columns of each part's row in the part table,
one value throughout. The core's hardware prologue is the project's
`CH32X035_HPE` option, on by default: the crt sets INTSYSCR.HWSTKEN and
`BRIO_CH32_VECTOR` writes WCH's fast attribute, one switch for the
whole image ([platform.md](platform.md)).

## Build

`ch32x035/` is its own CMake project, a sibling and peer of the other
cross projects and of `test/`; the repository root is not a CMake
project, so every CMake command runs from `ch32x035/`:

```bash
(cd ch32x035 && cmake --preset ch32x035f8-release)       # configure (once, or after adding an app)
(cd ch32x035 && cmake --build --preset ch32x035f8-release --target <app>)
(cd ch32x035 && cmake --build --preset ch32x035f8-release --target <app>-upload)
brio flash <board> <app>                                 # the same thing through the bench's one command
brio check ch32x035 [name]                               # family smoke, no hardware
```

One configure targets one part. The part number selects the part
definition `device.hpp` asks for, the linker script, the board type,
the ISA and the ABI, from one table
([cmake/ch32x035-parts.cmake](../../ch32x035/cmake/ch32x035-parts.cmake)):
the seven parts of the datasheet's model table, whether or not a board
exists for them, each with a release preset (`ch32x035r8-release`,
`ch32x035c8-release`, `ch32x035g8u-release`, `ch32x035g8r-release`,
`ch32x035f8-release`, `ch32x035f7-release`, `ch32x033f8-release`) and
the CH32X035F8 with the one debug preset, `ch32x035f8-debug` (CMake
Tools shows them as `CH32X035F8 (release, -Os)` and `CH32X035F8 (debug,
-Og)`). The two 28-pin parts share a number and not a bonding, so their
keys carry the package's letter (`ch32x035g8u`, `ch32x035g8r`).

The image is linked at `0x0000 0000`, the alias of the code flash at
`0x0800 0000` that the core boots from, and it gets the whole 62 KB: no
zone is set aside for a flash store here, which is the NV stack review's
question and not this target's
([ld/ch32x035f8.ld](../../ch32x035/ld/ch32x035f8.ld)). Applications are
discovered at every configure from `ch32x035/src/apps/*.cpp` by the
`// build:` lines in each file's header comment, with the seven board
types of the part table as the board names and `x035f8` the one the
bench manifest knows ([../boards/README.md](../boards/README.md)).
Every part has the same two memories, so the family's smallest chip is
every chip: a suite whose header names groups (`// build: groups =
...`) builds as one image per group on every part, `<app>-1`, `<app>-2`
and so on (the project lists each board type of its part table as a
split board, and the roster says so), and a suite with no groups line
is one image everywhere ([../design/overview.md](../design/overview.md),
"A suite's image fits the family's smallest chip"). The I2C and SPI
suites are two images each.

**Code runs from the flash, and a hot path from SRAM where the program
says so.** The flash answers behind two wait states at 48 MHz. An image
is linked with two scripts, the part's memories (`ld/<part>.ld`) and
the series' section layout
([ld/sections.ld](../../ch32x035/ld/sections.ld)), which carries
`.fast_text` and `.fast_text.*` in `.data`, which the crt copies into
SRAM before `main()`: zero bytes in an image that places nothing. The
preemptive kernel's scheduler path goes there too unless the program's
source carries the build line `// build: scheduler = flash`
([platform.md](platform.md), "The scheduler's place"). A driver's
placement is opt-in: the SPI host, the I2C host and the USART transport
take util/placement.hpp's `InFlash` (the default: the hot body inline
in its vector, the image as if no placement existed) or `InRam` (each
hot path one instance in `.fast_text.brio`). The paths that move are
the SPI host's pump entry and polled loops, the I2C host's event pump,
and the USART's transmit completion and its per-byte body; the drivers'
cost constants follow the placement. A program places a function of
its own, the vector it binds above all, with `.fast_text`, and
everything inline in that vector moves with it. The section has two
names, `.fast_text` for a program's function and `.fast_text.brio` for
the drivers', because GCC refuses one section name for a header's
function and a plain one in the same unit. The measurements, the bytes
each placement costs of the 20 KB, and which of the two buys what are
[platform.md](platform.md), "A handler in SRAM".

WCH's EVT package ships no SVD for this series, so the project has no
`svd/` and a debugger would have no peripheral view.

## The probe and the upload (WCH-LinkE, WCH's OpenOCD)

A WCH-LinkE over the two-wire debug port this series has, PC18 = SWDIO
and PC19 = SWCLK (the WCH-Link user manual's table 6), which the board
brings to its header P1, pins 6 and 8
([../probes/wch-link.md](../probes/wch-link.md)). WCH's OpenOCD fork at
`/sw/wch-openocd` (v2.10 by its own `version.txt`) is the one OpenOCD
that speaks the probe's SDI transport. Its `wch-riscv.cfg` selects the
`wlinke` adapter at 6000 kHz, the `sdi` transport, a `wch_riscv` target
and a flash bank at address 0 whose size is 0, found by probing the
chip, and its binary carries a `ch32x` flash driver and the names
CH32X033, CH32X034 and CH32X035. It programs and verifies the
CH32X035F8U6 through a WCH-LinkE: every image the suites ran from was
written that way.

The upload target and `brio flash` both write the image with
`program ... verify` and start it with `reset halt` followed by
`resume`: on the CH32V203 the fork's `reset run` leaves the hart at the
reset vector ([../ch32vx03/README.md](../ch32vx03/README.md)), and the
pair that starts the program there starts it here too; whether `reset
run` would on this part is not measured. `brio flash` names the probe
by the USB serial the manifest gives; the CMake upload target, as on the
sibling projects, takes the one probe attached.

## Debugging

`.vscode/launch.json` has no entry for this target, and no editor
session has been run on it. What the bring-up used is the fork's own
console over the same probe (`halt`, `reg pc`, `reg mcause`, `mdw`),
which is how a program that looks dead is named: the crt's weak
default handler is a spin loop, with no fault and no message, and the
exception entry and the breakpoint entry have weak spins of their own,
so a probe that halts the core names the wreck from the program counter
(`default_handler`, `fault_handler` or `breakpoint_handler`) and
`mcause` gives the interrupt number or the exception code.

## Editor (clangd)

`brio/ch32x035/.clangd` and `ch32x035/.clangd` route the stratum and
the project to the project's own compilation database, so a header of
this stratum parses with the RISC-V compiler's real command line
whatever project CMake Tools has active.

## Serial console

USART2 on PA2 (TX) and PA3 (RX), the board's "Serial port 2" (P1 pins
7 and 5), wired to the probe's own serial, so one cable carries the
debug port and the console. 115200 8N1; `brio console <board>` prints
the device path and the speed.

WCH's own examples print on USART1's default pads, PB10 and PB11. The
QFN20 bonds PB11 and not PB10, and no column of USART1 has both its TX
and its RX pads on this package, so the part table does not offer
USART1 on it at all ([usart.md](usart.md)).

## Family coverage

The stratum is written for the seven parts of the series, one die in
seven packages: every one has its table, its linker script and its
preset. `brio check ch32x035` compiles every smoke TU under
`test/family_ch32x035/` for all seven, both ways the hardware prologue
can be built, and demands that every `neg/` TU, a program that must be
refused, fail; `util_all.cpp` among them includes every kernel and util
header and instantiates each service over the platform. The check ends
with the link guard of every app on every release preset. The bench
proves the CH32X035F8U6 alone; for the other six the sweep is the only
check there is.

## What the documents say that shapes the stratum

What the reference manual, the datasheet and the vendor's library say
that the stratum is built around; what the CH32X035F8U6 answered is the
next section, and what is still owed the list at the end of this page.

- **One root and no bus prescaler.** SYSCLK is the 48 MHz HSI and
  nothing else (RM 3.3); HCLK is SYSCLK through HPRE, which resets to /6
  (the chip wakes at 8 MHz), and every peripheral, the four USARTs
  included, runs at HCLK (14.3). So a clock type is a rate and a
  divider, and `pclk_hz` is `hz` ([clock.md](clock.md)).
- **HPRE's own register asks for a prefetch buffer that has no bit.**
  RM 3.4.2 closes the divider's description with "when the prescaler
  factor of the HB clock source is greater than 1, the prefetch buffer
  must be turned on"; no register of the manual has such a bit (the
  flash's access control register holds the wait states alone, 20.3.1),
  and WCH's own clock code divides HCLK without touching anything else.
  The driver does what the vendor does.
- **A port runs to 24 pins, over three configuration registers**:
  CFGLR, CFGHR and CFGXR (8.3.1.1, 8.3.1.2, 8.3.1.8), with BSHR setting
  and resetting pins 0..15 and BSXR pins 16..23 (8.3.1.5, 8.3.1.9), so a
  mask across both halves is two stores ([pin.md](pin.md)).
- **WCH's library does not read CFGHR on some dies.** Where the word at
  0x1FFFF704, which that library reads as the chip identifier, has zero
  in bits 7:4, it keeps a RAM copy of each port's CFGHR and writes the
  register whole from it; the reference manual says nothing. This
  stratum does the same on every die ([pin.md](pin.md)).
- **A pin has no speed and no open drain.** MODE's three non-zero codes
  are all "output" and CNF names push-pull and alternate push-pull alone
  (8.3.1.1); the pull is the output register's bit under a pulled input,
  and only PA0..PA15, PC16 and PC17 have a pull-down (the opening of RM
  ch. 8).
- **Some package pins carry two pads.** On every part but the
  CH32X035F8U6, PC16 shares its pin with PC11 and PC17 with PC10; on the
  two 28-pin parts PB1 shares one with PB5; on the QSOP28 PA12 shares
  one with PC14 and PA13 with PC15; on the CH32X033F8P6 PA7 with PB0;
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
  file writes INTSYSCR = 3, the hardware stack and two-level nesting,
  and enters main() through an mret with mstatus 0x88, whose MPP of
  zero is user mode; this crt writes the hardware stack alone (nesting
  never: [../design/kernel.md](../design/kernel.md), "The model in one
  page") and calls main() in machine mode with interrupts masked
  (mstatus 0x1880). corecfgr gets WCH's 0x1F, which neither manual
  explains ([platform.md](platform.md)).
- **The USB blocks' clocks are on out of reset**: RCC_AHBPCENR resets to
  0x00021004, the USB PD controller's gate, the USB host/device
  controller's, and SRAMEN, the SRAM's clock in Sleep (3.4.5).
- **A lot may have no I2C**: table 2-1's note 3 (and the notes of 1.4.12
  and 1.4.16), "the channel 3, channel 7, channel 11, channel 15 and
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

What the CH32X035F8U6 answered on WCH's evaluation board: the suites'
`z` and the USART suite's `y`, each green whole, and a read over the
debug port; the numbers are each document's bench findings.

- **RMVF is a level, not a pulse.** Written 1 it reads back 1
  (RCC_RSTSCKR read 0x01000000 once the flags had cleared), where the
  CH32V203's bit clears itself; `Rcc::clear_reset_flags()` writes it and
  then clears it, and after that the register reads zero
  ([clock.md](clock.md)).
- **A reset pulse on the power controller's line cut the part off its
  debug port.** Pulsed through RCC_APB1PRSTR.PWRRST after every gate of
  the chapter had opened and closed, it silenced the console before the
  next line drained, and the debug port answered "failed to connect"
  until the supply was cycled, on a package with no reset pin, where a
  hand on the supply is the only way back. Observed once and not
  repeated on purpose: `Rcc::reset()` refuses that line and answers
  false ([clock.md](clock.md)).
- **A port read through its shut gate answers with the last word the
  bus carried.** Port A's clock gate is closed out of reset, and its
  INDR, read over the debug port while the gate was shut, returned the
  last word the bus had moved and not the pads' levels. A pad is read
  after its port is configured: every configuring verb of `Pin` and
  `Port` opens the gate first, and a read does not ([pin.md](pin.md)).
- **Of the manual's two reset values for RCC_RSTSCKR, 3.4.8's is the
  QFN20's.** The boots measured read SFTRSTF and PORRSTF, a power-on and
  the probe's resets behind them, and never PINRSTF: the register
  table's 0x0C000000, with PINRSTF beside PORRSTF, is not this
  package's, which has no reset pin ([clock.md](clock.md)).
- **The vendor library's CFGHR caution is not this die's.** The word at
  0x1FFFF704 reads 0x035E0611, the library's 0x035E06x1 for the
  CH32X035F8U6 with 1 in bits 7:4, so WCH's own `GPIO_Init` reads CFGHR
  here, and the register reads back what the stratum's copy wrote for
  every nibble of the series. The stratum configures CFGHR from its copy
  on every die regardless, for the dies with zero there
  ([pin.md](pin.md)).
- **A divided HCLK needs nothing but HPRE.** With nothing of the tree
  written but HPRE and the flash's wait states (no prefetch buffer, the
  bit 3.4.2 asks for being in no register), the clock suite's ladder ran
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
  tick late, and a quiet kernel loop turns once per tick, twice with the
  latch left set, as the CH32V203's V4B did; the form the sibling strata
  use because the CH32V00x's WFI does not wake for an interrupt its core
  cannot take ([platform.md](platform.md)).
- **In the Sleep the DMA controller moves nothing.** RM 2.3.2 keeps the
  peripherals running in it, and USART3's transmitter kept asking: a
  copy started before a sleep stood still until the core woke, and a
  block paced at 1 Mbaud ran at a fiftieth of its pace, the CH32V203's
  and the CH32V303's answer. The platform's `idle()` does not sleep
  while a channel works, so a program with an engine at work, a receive
  ring above all, holds itself awake ([dma.md](dma.md),
  [platform.md](platform.md)).
- **A raised DMA request is held** until the channel takes it or the
  block's reset line is pulsed, its DMA enable, its enable and a closed
  gate notwithstanding, and the next owner of the channel takes one
  stray item: the release contract's case, which the `Uart`'s
  `release()` answers ([dma.md](dma.md)).
- **A PADDR stored with EN standing is dropped**, and EN stands after
  every completed block: the copy engine stores its source after the
  restart's disable ([dma.md](dma.md)).
- **The bench die has its I2C.** The lot note's question, put to the
  block over the debug port before any code and again by the I2C suite:
  FREQ, CKCFGR and OADDR1 hold what they are given and a START on the
  idle bus raises SB; whatever the unreadable marking says, this lot is
  not the one note 3 withdraws the I2C from ([i2c.md](i2c.md)).
- **The USB pads' pull-ups do not reach the alternate function.**
  AFIO_CTLR's 10 kOhm and 1.5 kOhm modes pull PC16 and PC17 up as pulled
  inputs and do nothing under I2C1's open-drain alternate function: an
  I2C on those pads wants resistors of its own, and on the evaluation
  board they cannot go to the supply, PC17 high at power-up being the
  loader's boot condition ([i2c.md](i2c.md)).
- **A port's output register behind a shut gate drops a store.** The
  same gate that makes a read meaningless swallows a write: a pad set
  high before its port's first configuration came up driving low, and
  `Pin::output(level)` and `Pin::input(pull)` open the gate before the
  output-data store ([pin.md](pin.md)).
- **The flash fetch bounds the core's hot paths, and SRAM lifts it.**
  Code runs from a flash with two wait states at 48 MHz: the SPI's bare
  transmit loop takes 20.8 cycles a frame from it and 16.7, the wire at
  /2, from SRAM; the driver's own loop 24.0 a frame at /2 and 16.0 from
  SRAM; the USART transmit engine's DMA body 285 to 288 cycles from the
  flash and 129 to 132 from SRAM; and a handler's body two fifths to a
  half of its flash cycles from SRAM with its vector placed
  ([platform.md](platform.md), "A handler in SRAM"). From the flash,
  code with taken branches costs what its address modulo four makes it:
  the SPI receive loop 52.0 or 72.0 cycles a frame at /4 as its function
  starts on a word or half a word into one, and nothing above that bit
  moving it, so a driver function that keeps a stated flash cost is
  aligned to a word, gcc giving two bytes under -Os ([spi.md](spi.md),
  "The fetch").
- **HSRXEN is a second rate ladder.** With the bit set every BR code
  gives HCLK/(BR+2), 16.3.1's note against 16.3.8's "only valid at /2",
  and the bit reads zero whatever was written ([spi.md](spi.md)).
- **A stopped timer holds its outputs, and a never-run one holds them
  low.** Chapter 12 is silent; with CEN clear every TIM1 and TIM2 output
  keeps driving the level it had, a forced level written meanwhile
  included, and the reset's level is low, so a waveform engine runs the
  counter before it hands a pad to the timer ([tim.md](tim.md)).
- **CTLR1's "reserved" bit 13 reads one and acts.** Set out of reset on
  both advanced timers, it lets an output MOE disables float; cleared,
  the output is driven at its idle level, the CH32V00x's OE_MODE
  position. The driver keeps it as it reads, as WCH's library does
  ([tim.md](tim.md)).
- **A software compare event does nothing to an output channel**:
  12.4.6's CCxG raised no CCxIF and no DMA request on either timer; a
  match does both, and UG raises the update's request ([tim.md](tim.md)).
- **An edge list keeps edges ten HCLK cycles apart**: TIM1's compare
  request reloading CHxCVR through the DMA controller took every value
  of a list spaced 10 cycles and one of eight at 8 ([tim.md](tim.md)).
- **A polled receiver loses a frame that lands between its STATR and
  its DATAR reads**, with ORE cleared by the same sequence, unseen:
  measured at 3 Mbaud with a time stamp taken between the two
  ([usart.md](usart.md)).
- **The SPI client's restrictions are half true.** 16.2.3's "mode 2 or
  mode 3" for a transmitting client does not hold (this die answers in
  all four modes), and 16.3.1's "LSB is only supported by SPI as host"
  holds for what a client sends: LSb first its MISO never moves
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

- **The failing half of the platform: `Reset`, the fault record and the
  two watchdogs (RM ch. 5, 6)**, born with their first user, the
  watchdog chapter, in one file as on the sibling strata. Until then the
  reset flags are `Rcc`'s verbs ([clock.md](clock.md)), and a trap lands
  in the crt's weak spins and leaves no record for the next boot.
- **The power chapter's low-power half: the sleep modes, the auto-wakeup
  and a sleep site (RM ch. 2, 4).** The voltage detector is
  [pwr.md](pwr.md)'s; this series has no RTC, so the AWU on the HSI
  divided by 1024 is the only wake a Stop or a Standby has, and a sleep
  site is born with it as its wake; until then `idle()` is the plain
  Sleep and nothing in the stratum sets SLEEPDEEP.
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
- **The USB PD controller (RM ch. 21)**: a PD source on the other end of
  the cable, and a first user.
- **The PIOC (RM ch. 22)**: an 8-bit co-processor with an instruction
  set of its own, whose programs want an assembler, a project of its
  own with the RP2040's PIO assembler as the model.
- **Flash programming and the option bytes (RM ch. 20)**: decoded
  read-only; the engine is born with the NV stack review's answer for
  this series, and the linker gives the image the whole array until
  then.
- **The reset pin's configuration** (the RST_MODE option bits, and the
  RST function the datasheet gives PA21, PC3 or PB7 by package): decoded
  read-only with the option bytes; the QFN20 has no reset pin.
- **The debug module's freeze bits (DBGMCU_CR, CSR 0x7C0, RM 23.2.1)**:
  read by nothing and written by nothing; on the CH32V203 a `csrw` to
  that CSR from the running program resets the part
  ([../ch32vx03/sleep.md](../ch32vx03/sleep.md)).
- **A maintained teaching example**: no `blink` builds in this project.

Implemented but not bench-verified, each with what would measure it:

- **What the suites have not measured**, each in its document's list
  with what would measure it: what the hardware prologue buys (the
  platform suite's letter `e` in an image built without it), the call
  into a driver's `InRam` body in such an image (the SPI suite's letter
  `b`), the clock output's waveform (a package that bonds PB9, and a
  counter), the reset flags one event at a time, the idle hook's power,
  the lock (the pin suite's `l`, by name), and what the USART's
  smartcard, clock and flow-control pair do on a wire (a logic
  analyser, or pads this package does not bond).
- **The six parts other than the CH32X035F8U6.** Every one has its
  table, its linker script and its preset, and the whole stratum
  compiles for all seven both ways the hardware prologue can be built
  (`brio check ch32x035`); what would measure them is a board of each
  package.

## The documents

One document per peripheral driver, in the shape
[../README.md](../README.md) prescribes: the reader's orientation, the
documents of record, what the silicon does, the types and verbs, how to
use it, the bench findings and, where the driver has gaps, "Not covered
yet" as two lists. The line beside each names what the document is
about and the finding or two most worth knowing before using the
driver; the findings themselves, with their figures and conditions, are
in the document.

| Document | Content |
|----------|---------|
| [device.hpp](../../brio/ch32x035/device.hpp) + [parts/](../../brio/ch32x035/parts/) | The register map and the part table: the seven parts, each with its memories, its package, the pads it bonds and the pads it shorts together inside (the datasheet's notes 4 to 7), the USARTs it offers beside the model table's count, and what else it carries; RCC, GPIO, AFIO, the USART and the core's STK and PFIC mapped for the drivers, the EXTI and PWR mapped for the reader, the flash interface's option bytes and the electronic signature decoded read-only; the interrupt numbers are the vector table's word indices |
| [platform.md](platform.md) | Platform, its code the QingKe core stratum's over this family's facts ([../qingke/README.md](../qingke/README.md)): `Ch32x035Platform` (the `csrrci` critical section, the WFE-shaped `idle()` that does not sleep over a working DMA channel, `ebreak`, the `.noinit` breadcrumb, the stack ledger), `Ch32x035RubatoPlatform` and the preemptive kernel's scheduling request (`qingke/schedule.hpp`: the software interrupt one preemption level below every peripheral line, returning into the scheduler in thread mode, the machine-mode `ecall` the way back), the scheduler's place (`// build: scheduler = ram` or `= flash`, ram the default: the path and a copy of the vector table in SRAM, the request ending with a `fence.i` there), `Pfic` and `BRIO_CH32_VECTOR` with `no_icf`, the 64-bit STK `Ticker`, `delay_us`, what the crt writes into the core, and a handler in SRAM: the drivers' placement opted into per driver (util/placement.hpp's `InRam`, `.fast_text.brio`), `.fast_text` for a program's own vector, which of the two buys what, and what each costs. Measured on the CH32X035F8U6 by `test_x035_platform` (209 verdicts): the fetch and trap cost model the preemptive kernel's bounds are drawn with, each figure to the cycle with corecfgr as the crt leaves it and cleared, from the flash (a word every 4 cycles, a taken transfer 8 or 12, a trap's entry 20, an MRET 8 or 24, a wrong guess of the branch predictor up to 8) and from SRAM (an instruction a cycle, a taken transfer 2 or 3, a load 2, a store 1 to 3 by its neighbours, a trap's entry 10 through a vector table in SRAM and 18 through one in the flash, an MRET 5, a wrong guess 3, and the crossings between the two memories), the WFE idle entered unmasked and the latch its wake leaves consumed (no edge position of 600 losing the wake or serving it a tick late, one kernel turn per tick at 48 MHz and at 8 MHz, where the unconsumed latch gave two in a probe image with the consume removed), the STK's compare and its CNT arithmetic within 128 cycles of the interrupt count over 200 reloads, `delay_us` 108 to 140 cycles late and never early, an interrupt round trip of 96 to 100 cycles at 48 MHz with the hardware prologue and 120 to 128 in an image built without it (where an InRam body's call is some 65 cycles dearer), every vector aligned to four, corecfgr's 0x1F making a timed loop 7.3 per cent slower at one placement of the loop and 25 per cent at another, INTSYSCR and mtvec as the crt wrote them and an MPP no MRET changes, and the reset flags, the signature and the chip-identifier word read at boot; and by `test_x035_rubato` and `test_x035_rubato_flash` (40 verdicts each, ten runs of each image): with the scheduler path in SRAM, the default, road 1 (an interrupt handler's post preempting the running object) in 154 to 160 cycles from the interrupt's last statement to the preempting dispatch's first, a quiet post 108 to 112, a road-2 post (one object's post to a higher-priority one, which runs inside it) 406 to 408, the vector table's copy in SRAM saving 8 cycles at each trap it serves, for 1724 bytes of SRAM in the suite's five-object pack (a program's `activate()` as large as the dispatches inlined into it); with the path in the flash road 1 in 307 to 311, a quiet post 141 and a road-2 post 785, the fetch's 4 cycles a word over the CH32V203's instructions; and by `test_x035_rubato_link` and `test_x035_rubato_link_flash` (key `y`, 115 verdicts, three runs of each image) against the CH32V006K8U6's `usart_peer`: the UART stack under an object preempting every 97 to 389 us, the lines received through the ring and lent to a borrower above the reader and the lines blasted through the transmit engine byte-exact in both placements, the HSI's lead over the far crystal measured before every echo letter; the SPI key built and not judged ([spi.md](spi.md), "Not covered yet") |
| [clock.md](clock.md) | RCC: one root (the 48 MHz HSI) and one divider (HPRE, /6 out of reset), the user trim, the flash's wait states sequenced around every change, the clock output on PB9 where the package bonds it, the gates, the resets and the reset flags; `Clock<internal, hz>` and `DynamicClock<Boot, Users...>`. Measured on the CH32X035F8U6 by `test_x035_clock` (26 verdicts): the whole HPRE ladder from 48 MHz down to 3 MHz and back with the tick, a busy-wait and the console exact at every rung and nothing of the tree written but HPRE and the wait states, the tick HCLK / 1000 to the cycle at every rung against a timer, the trim a step each way and, against the desk's CH32V006K8U6 crystal, the HSI at -0.25 per cent and the trim's step at 116 to 118 kHz on average from 7 to 28, the HSI moved by the load (some 400 ppm faster with the core asleep than spinning at 48 MHz, the rungs below faster than 48 MHz, by a host's clock), RCC_AHBPCENR's reset value, every gate opened and closed, the console's own two under a drained console; the reset flags one event at a time (the probe's reset, a programming and PFIC_CFGR's SYSRST each SFTRSTF alone), RMVF a level, and the power controller's reset line refused after a pulse on it cut the part off its debug port |
| [pwr.md](pwr.md) | PWR: the programmable voltage detector alone, PLS's four thresholds and PVD0, no enable bit; the check every image that drives a link pad makes at boot (3.0 V), reading "not low" on the CH32X035F8U6's 3.3 V rail |
| [pin.md](pin.md) | GPIO and AFIO: the F1's nibble with no speed and no open-drain output, 24 pins a port over three configuration registers, CFGHR written whole from a RAM copy as WCH's library does it, BSHR and BSXR for the two halves of a port, the pull in the output register and a pull-down on PA0..PA15, PC16 and PC17 alone, the bonding and the shorted pads refused at compile time, the one-way lock; the remap fields of AFIO_PCFR1 with the USARTs' columns as data, the debug port's field read and written only by a verb nobody reaches by accident, the EXTI port multiplexer. Measured on the CH32X035F8U6 by `test_x035_pin` (19 verdicts in `z`, two more on its jumper): every nibble in each of the three registers, CFGHR reading back what its copy wrote on a die the vendor's caution is not for, the levels, BSXR on PC14, the pulls, a remap field, the debug port's field and the EXTI multiplexer's three codes, and the levels across a jumper; a port read through its shut gate answering with the last word the bus carried |
| [usart.md](usart.md) | The four USARTs: every one a full USART counting its divisor in HCLK, and which of them a part offers read off its pins (the QFN20 offers USART2, USART3 and USART4, and no USART1); the frame, mute, LIN, half duplex, IrDA, the smartcard, the synchronous clock and the flow-control pair as the resource's verbs, the chapter's exclusions refused; the `Uart` transport with the sibling strata's surface and a DMA engine in either slot, checked against the request table, the receive ring at `rx_priority`, the block reset in `release()`. Measured on the CH32X035F8U6 by `test_x035_usart` (23 verdicts single-board, 98 against the desk's CH32V006K8U6 on its crystal across USART4's PB0/PB1): the divisor in HCLK, every frame and mode register by register, a frame's length and the break's timed, the debug port's column refused, half duplex deaf to its own frames and hearing a second chip's; across the wire every rate from 115200 to 3 Mbaud echoed and timed by the far receiver within 500 ppm of HSI / BRR, nine frame formats with half and one-and-a-half stop bits, PE, FE onset, glitches, ORE and the break from a crystal sender, mute woken both ways, IrDA in normal and low-power mode, LIN's two detection lengths; the interrupt transport's sustained receive clean to 3 Mbaud with its vector in SRAM (2.4 Mbaud with the port's body alone placed, 2 from the flash) and on its engines 8192 frames clean at every rate to 3 Mbaud, the wire's ceiling, two interrupts a burst and two a lap; the error flags' clear performed by the channel's read and a run of errored frames counted every other one (a status read on the wait for a frame, with no transmit engine, counted none of eight), a status read that found IDLE down arming no clear of it, the receive vector's two guards staged by a block above the ring (the overrun with nothing to take in four entries, the stall bound giving a starved channel up after 257), the ring at very_high kept beside copies on a lower channel that starve it ranked low, and the transmit engine gapless to 3 Mbaud with its DMA vector's body placed in SRAM, 129 to 132 cycles where the flash's 285 to 288 idle the line above 1.5 Mbaud; a polled receiver losing a frame between its STATR and DATAR reads; and `usart_peer`, this part as the desk's USART instrument for the CH32V006's suite, calibrated by that suite's crystal, its echo 28 instructions a character with no call, its waveforms on TIM1, its collide a wired AND it reads back |
| [i2c.md](i2c.md) | I2C1: the F1's event machine on two vectors with no rise-time register, FREQ's 4..60 MHz window under a 48 MHz HCLK, six columns and which of them each package can use, the lot note asked of the block itself (`present()`), the host engine with the sibling strata's Request, the first two bytes in the ADDR entry, the repeated START on the last byte's TxE, engines on channels 6 and 7, the client polled or on its vectors. Measured on the CH32X035F8U6 by `test_x035_i2c` (two images, 107 verdicts against the desk's CH32V006K8U6 running `twi_peer`, the bus pulled up from PB3): the block present, the scan, the shapes and the four receive procedures, the vocabulary, the rate asked at 100 and 400 kHz on the pump and on the engines, the kernel with a held SDA and a held clock, arbitration both ways and this board as a foreign host's client written to and read from (`flush()` dropping the byte asked beyond each closing NACK), the stuck bus, the repeated START against a polled CH32 target, the USB pads' pulls absent under the alternate function, fast mode at DUTY 16/9, a BUSY left over an idle wire taken out by `start()`, the dead channel (a timer's held request on channel 6 or 7 answered `i2c_dma_fault` on a write and by the caller's bound on a read, the block's own request not outliving its gate), the client served from its vectors, one entry an event, and the cost: 205 cycles an entry a byte on the pump from the flash behind the suite's vector, 140 with the pump placed in SRAM and 79 with the vector a program places there, three event entries and no DMA interrupt for an engined write of any length; and `twi_peer`, this part as the desk's I2C instrument for the CH32V006's suite, its polled target's habits measured from that host |
| [spi.md](spi.md) | SPI1: the F1's block with no FIFO, the chapter's two mode numberings and neither of them the driver's, HSRXEN and ODEN, four columns of which the QFN20 bonds one; the host engine with the sibling strata's Request, the transmit-only loop, the receive loop one or two frames in flight, the pump, engines on channels 2 and 3, the client's runs. Measured on the CH32X035F8U6 by `test_x035_spi` (two images, 34 verdicts single-board, 39 against the desk's CH32V006K8U6 running `spi_peer`): the engines at the rate asked at every code to 24 MHz, HSRXEN's second ladder HCLK/(BR+2), nothing enable-protected, SSOE low from the first frame, ODEN open drain, both CRCs the arithmetic, MODF from a second chip; both hosts' waveforms the standard's in all four modes (this chip as the analyser), the CH32V006 client served in modes 0 and 3; this chip as a client receiving in every mode and order and answering in every mode MSb first and never LSb first, served from its runs or from its vector; the fault exits provoked (`spi_stalled` over a block in reset, `spi_dma_fault` over a dead channel, `recover()`) and the block's transmit request outliving its gate until SPE falls; the costs beside the wire in both placements, the pump's entry and both polled loops in the flash or in SRAM (from SRAM the transmit-only loop at the wire at /2 and their cost no longer moving with the code around them); and `spi_peer`, this part as the desk's SPI client for the CH32V006's suite: the extended exchange served (16-bit frames, the CRC, a receive-only run), exact both ways to 6 MHz, and above that losing edges whenever its own MISO pad stands high (measured, not explained) |
| [tim.md](tim.md) | TIM1 and TIM2: one advanced-control block twice, in the subset a waveform engine needs: the time base, every compare mode, the complementary outputs behind MOE, one pulse, the compare's and the update's DMA request, the remap with both timers' columns as data. Measured on the CH32X035F8U6 by `test_x035_tim` (47 verdicts, no wire of its own): CTLR1 reading 0x2000 out of reset, its "reserved" bit 13 deciding whether an output MOE disables floats or is driven, the outputs holding while the counter is stopped and a never-run timer holding them low, a software compare event raising nothing, every request of figure 9-1, an edge list streamed by DMA to its intervals and kept down to edges ten cycles apart, one pulse to the cycle, the repetition counter edge- and centre-aligned on both timers, the three centre-aligned flag policies, the down counter, UDIS and URS (a UG moving a running counter only and loading nothing under UDIS, so `configure()` writes the start and stores UDIS after its UG), and TIM2's C2N on PB12 under two codes; and `usart_peer`'s waveforms on C2N and C3N |
| [dma.md](dma.md) | The DMA controller: eight channels where the channel is the request, the request table per part as the transports' check, the engines in two moments (five stores a block, six for the copy), `DmaTxEngine`, `DmaRxEngine` with its circular ring, `DmaCopyEngine`, the levels by the rule. Measured on the CH32X035F8U6 by `test_x035_dma` (55 verdicts, its own console on two engines): three widths at six cycles an item, the nine unequal pairs as table 9-1 spells them, EN standing after a block and a PADDR stored under it dropped, the eight vectors (the eighth at 44), no hole read or written nor a write into the flash raising TEIF, circular laps paced by a USART, the arbiter strict, a raised request held until the block's reset, the controller idle in Sleep, and the benchmark row |
| [vendor/README.md](vendor/README.md) | The documents of record with their revisions: the reference manual V1.8, the datasheet V1.7, the QingKe V4 manual, the probe's manual, and the EVT package as the vendor's only voice on quirks; the no-errata statement; where the documents disagree, and which of those items the CH32X035F8U6 has answered |

[afio.hpp](../../brio/ch32x035/afio.hpp) and
[dma_engine.hpp](../../brio/ch32x035/dma_engine.hpp) have no document
of their own: the first is [pin.md](pin.md)'s second half, the second
the request table and the engine slot's vocabulary [dma.md](dma.md)
names.
