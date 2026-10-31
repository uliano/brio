# Target: CH32V203 and CH32V303 (`ch32vx03/`)

The operational page of brio's CH32V203 and CH32V303 target, for a
programmer who has the WeAct CH32V203 board or WCH's CH32V303
evaluation board on the desk and wants to build, flash and debug a brio
program on it, and then find the document of the peripheral the program
needs. It says what the stratum is, which boards and parts it is proven
on, how the toolchain, the build, the upload, the debugger and the
console work and where each has a trap, what the silicon taught the
stratum across the chapters, what is not covered yet, and it ends with
the map of the stratum's own documents: one line per peripheral
driver, which is where the state of the driver work is read. Each
part's maturity is stated in one place, the target table of the root
README; every chapter is green on both boards, with what is excluded
and why in "Not covered yet".

A stratum is a directory of `brio/` holding one architectural layer;
`brio/ch32vx03/` holds everything that knows the register map the
CH32V203 and the CH32V303 share. A test suite's console takes one
command key per group of verdicts, which the pages call a letter, `z`
running a suite's single-board set. The core is WCH's QingKe V4B, a RISC-V
core with the RV32IMAC instruction set (thirty-two registers, a
multiplier and a divider, the atomic extension, user mode and a 64-bit
system counter), on the CH32V203C8T6 of the bench (64 KB of flash,
20 KB of SRAM, 144 MHz); and the QingKe V4F of the CH32V303, the same
core with a single-precision floating-point unit (RV32IMAFC), on the
CH32V303VCT6 (256 KB of zero-wait flash, the part of its 480 KB array
the core reads with no wait state, the rest being the tail; 64 KB of
SRAM; 144 MHz), the reference manual's CH32V30x_D8, the third device
class of this stratum beside the CH32V203's two. The fact this target
exists to keep true is
the same as every other's: the kernel (`kernel/`) and the services
(`util/`) compile here unchanged. There is no maintained teaching
example in this project: the console and the test suites are its
applications, and `blink` here is a stated gap.

Two things shape the stratum and are stated up front. **There is no
vendor header in the build**: WCH ships its register definitions inside
the EVT package, whose licence is written for software running on WCH
parts, so [brio/ch32vx03/device.hpp](../../brio/ch32vx03/device.hpp)
carries the map, read off the reference manual and answerable to it,
and per-part variability, which the ST strata ask the device header
for, is stated in [parts/](../../brio/ch32vx03/parts/). And **this is
not the CH32V00x with a bigger core**: the peripheral generation is a
different one, the STM32F1's under WCH's names (sixteen pins a port
with the F1's two-bit MODE, a USB device controller that is ST's
register for register on the CH32V203, where the CH32V303 carries WCH's
own host/device block instead, a bxCAN, an F1 timer set), where the
CH32V00x is a reduced design of its own.

The documents of record and their revisions, the class rule of the
reference manual that covers four families, and what the manual's V2.5
changes against what the silicon said are in
[vendor/README.md](vendor/README.md).

## The boards

Two boards, one per part, each with a page of its own under
[../boards/](../boards/README.md) with its connectors, the wires between
the two (they are each other's peers) and its probe.

| Board | Part | Memory | Clock | Console | LED |
|---|---|---|---|---|---|
| [a WeAct CH32V203C8T6 core board](../boards/weact-ch32v203c8.md), the bench board | CH32V203C8T6, LQFP48, the QingKe V4B | 64 KB flash, 20 KB SRAM | HSI 8 MHz and a PLL to 144 MHz; an 8 MHz crystal and a 32.768 kHz one | USART1 PA9/PA10 on the probe's own serial, 115200; the USB-C on the device controller | a blue LED on PB2, active high; a KEY on PA0 (the vendor's page) |
| [WCH's CH32V303 evaluation board](../boards/ch32v303-evt.md) | CH32V303VCT6, LQFP100, the QingKe V4F | 256 KB zero-wait flash of a 480 KB array, 64 KB SRAM | the same; an 8 MHz crystal on the LQFP100's own oscillator pins (PD0 and PD1 are pads of their own there) and a 32.768 kHz one on PC14/PC15 | the same; two USB connectors on PA11/PA12, the host/device controller's | two LEDs and a KEY on a row of pins a jumper takes to a pad (PB2 and PA0 as on the WeAct board), the LEDs lit with the pad low |

Both run at 3.3 V. The CH32V303 has no USB device controller, so a USB
console there runs on the host/device controller in device mode
([usbfs.md](usbfs.md)).

## Toolchain

The compiler is WCH's own `riscv32-wch-elf` gcc 15.2.0 at
`/sw/wch-riscv`, the same compiler the CH32V00x target uses and for the
same reason: it is the only one that emits WCH's proprietary `xw`
compressed extension, which this core carries too (`misa` reads
`0x40901105`: I, M, A, C, user mode and one non-standard extension;
[../ch32v00x/README.md](../ch32v00x/README.md), "Toolchain"). What is
not shared with that family is the ABI: ilp32 here against its ilp32e,
because this core has the full thirty-two registers. The project's
`-march` is `rv32imac_xw`, which their gcc resolves to the
`rv32imac_zaamo_zalrsc_xw/ilp32` multilib. The ISA and the ABI are two
columns of each part's row in the part table, and the CH32V303's are
its V4F's own: `rv32imafc_xw` with ilp32f, float arguments and results
in the single-precision registers, which the same gcc resolves to the
`rv32imafc_zaamo_zalrsc_xw/ilp32f` multilib (`misa` reads `0x40901125`
there, F beside the rest). One project and one crt serve both: the crt
switches the unit on exactly when the compiler says the image was built
with F (`__riscv_flen`), never by a part name.

## Build

`ch32vx03/` is its own CMake project, a sibling and peer of the other
cross projects and of `test/`; the repository root is not a CMake
project, so every CMake command runs from `ch32vx03/`:

```bash
(cd ch32vx03 && cmake --preset ch32v203c8-release)                      # configure (once, or after adding an app)
(cd ch32vx03 && cmake --build --preset ch32v203c8-release --target <app>)
(cd ch32vx03 && cmake --build --preset ch32v203c8-release --target <app>-upload)
(cd ch32vx03 && cmake --build --preset ch32v303vc-release --target <app>)   # the CH32V303: rv32imafc_xw, ilp32f
brio flash <board> <app>      # the same thing through the bench's one command
brio run <board> z            # a suite's single-board set, judged
brio check ch32vx03 [name]    # the family compile check, no hardware
```

One configure targets one part. The part number selects the part
definition `device.hpp` asks for, the linker script, the board type, the
ISA and the ABI, from one table
([cmake/ch32vx03-parts.cmake](../../ch32vx03/cmake/ch32vx03-parts.cmake)):
the thirteen parts, the CH32V203's nine and the CH32V303's four, are
named there whether or not a board exists for them, because the table
is a statement about the family. Every part has a release preset
(`ch32v203f6-release` to `ch32v303vc-release`), and the two bench parts
a debug one each, `ch32v203c8-debug` and `ch32v303vc-debug` (CMake Tools
shows them as `CH32V203C8 (release, -Os)` and `CH32V203C8 (debug,
-Og)`). The board types an app's `// build: boards =` line names are
`v203c8` and `v303vc`, the two the bench manifest knows, and `v203c6`
and `v203rb` for the parts without a board (the C6 preset is the 32 KB
tier's link guard) ([../boards/README.md](../boards/README.md)).

The four 32 KB parts split a suite too big for them: a suite that
declares its groups of letters (`// build: groups = abc,de`) builds
there as one image per group, `<app>-1`, `<app>-2` and so on, each
registering its own letters and carrying only their verdict prose,
while every other part builds the same source as one image with every
letter, unless the suite names a part's own groups (`// build:
groups.v203c8 = abcdefgh,ijklnopqr`) because its whole image comes
within a few per cent of that part's memory: the I2C suite and the
benchmark skeleton are two images on the CH32V203C8
([../design/overview.md](../design/overview.md), "A suite's image fits
the family's smallest chip").

## The probe and the upload (WCH-Link, WCH's OpenOCD)

A WCH-LinkE (firmware 2.16) over the two-wire debug port this family
has, PA13 = SWDIO and PA14 = SWCLK, not the CH32V00x's single wire; on
the CH32V303 board a WCH-Link of the CH549 kind on the same two pads,
which forwards its serial bridge in blocks and can fall behind after a
burst sent into a closed port; a WCH-LinkE at firmware 2.10 attaches
that board too, its attach rewriting RCC_CFGR0 and setting the debug
module's low-power bits on a running chip
([../probes/wch-link.md](../probes/wch-link.md)). WCH's OpenOCD fork at
`/sw/wch-openocd` is the only OpenOCD that speaks the probes' SDI
transport. The upload target and `brio flash` run the same thing:
`openocd -f wch-riscv.cfg -c "program <app>.elf verify" -c "reset
halt" -c resume -c exit`, the probe named by its USB serial when the
manifest gives one.

**`reset run` does not start the program.** In that fork the verb
leaves the hart sitting at the reset vector with every peripheral at
its reset value, measured here: the program counter still zero and the
clock tree untouched four hundred milliseconds later. An image flashed
that way is in the chip and not running, and the board looks dead. The
upload target and `brio flash` both use `reset halt` followed by
`resume`.

## Debugging

`.vscode/launch.json` carries a cppdbg entry per bench part, "Debug
CH32V203C8 (WCH OpenOCD, WCH-Link)" and "Debug CH32V303VC (WCH OpenOCD,
WCH-Link)", the CH32V006 entry's shape: the fork launched by the
editor and listening on 3333, the gdb built from source under
`/sw/riscv-gdb` in front of it, `reset halt`, `load` through the fork's
flash driver and a stop on `main()`; the CH32V303 entry wants the
`ch32v303vc-debug` preset, built with F and the ilp32f ABI, so gdb shows
the f-registers and fcsr beside the integer file. Both are configured,
not verified: neither has been driven from the editor. What the
bring-up used is the fork's own command console (OpenOCD's telnet
console: `halt`, `reg pc`, `reg mcause`, `mdw`, `resume`) over the same
probe.

A program that enables a line and binds no handler for it looks dead:
the crt's weak default handler is a spin loop, with no fault and no
message. What names the line in a minute is the probe: `openocd ... -c
halt -c "reg pc" -c "reg mcause"` shows the program counter in
`default_handler` and the interrupt number in mcause.

## Serial console

The probe's own serial pins go to PA9 (USART1_TX) and PA10
(USART1_RX), the instance's default mapping, so one cable carries the
debug port and the console, at 115200 8N1; `brio console <board>`
prints the device path and the speed.

The chip's own USB carries the console too, as a CDC ACM port with
pid.codes' test identity 1209:0001: on the WeAct board through the
device controller on its USB-C ([usb.md](usb.md)), on the evaluation
board through the host/device controller on its USB-C connector P14
([usbfs.md](usbfs.md)), one stack and one class over two controllers,
the part table choosing which.

## Family coverage

The stratum is written for the thirteen parts of the two series under
two ISAs, the V4B's and the V4F's. `brio check ch32vx03` compiles every
smoke TU (a translation unit, one source file that includes a driver
and uses it, written to prove that it compiles) under
`test/family_ch32vx03/` for all thirteen, both ways the hardware
prologue can be built (the project's `CH32VX03_HPE` option, on by
default: with it the crt sets INTSYSCR.HWSTKEN and every handler takes
WCH's fast attribute, the core saving the caller-saved registers
itself; off, a handler is a plain `[[gnu::interrupt]]` function), and
demands that every `neg/` TU, a program that must be refused, fail for
the parts its `// mcu:` line names; `util_all.cpp` among them is the
whole of kernel/ and util/ over both ABIs, ilp32 and ilp32f. The check
ends with the link guard (every app linked on every release preset, a
failure fatal), and adds the vector guard (`cli/vector_guard.py`): on
the F images every naked vector's body is walked in the ELF, and a
floating-point instruction or an unresolved indirect call in one is
fatal ("What the silicon taught the stratum", the V4F's vectors). The
bench proves the CH32V203C8 and the CH32V303VC; for the other eleven
the sweep and the link guard are the only checks there are.

## What the silicon taught the stratum

- **`ebreak` with no debugger goes to the breakpoint vector, not the
  exception one.** The table has both, the exception entry at index 3
  and the breakpoint one at index 9, and the silicon takes the second,
  on the V4B and the V4F alike, while mcause reports exception code 3,
  this core's number for a breakpoint: the vector index and the
  exception code are different numbers here. A program that wants a
  crash recorded binds both.
- **corecfgr (CSR 0xBC0) is zero at the reset vector.** The QingKe V4
  manual says it configures the pipeline and branch prediction, gives
  no bit table, and says the products set it in their startup file;
  WCH's own writes 0x1F, and so does this crt. What those five bits buy
  is not the same on the two cores: two cycles in 36811 of a timed loop
  on the CH32V203C8T6's V4B, and on the CH32V303VCT6's V4F the same
  loop half to eight tenths of a per cent slower with them than without
  ([platform.md](platform.md)).
- **The V4F's floating-point unit is off out of reset, and its vectors
  are trampolines.** mstatus.FS is 00 at the reset vector and every FP
  instruction traps in that state, so the crt of an image built with F
  switches it on, and its own write to fcsr leaves the unit Dirty,
  which is what main() finds. The hardware prologue saves the sixteen
  integer caller-saved registers and nothing else (the V4 manual's 3.4,
  note 3): under the ilp32f ABI the compiler saves in an
  interrupt-attributed handler's own code the f-registers it clobbers,
  and all twenty of them as soon as it calls a function, whatever the
  callee. So an image with F binds every vector as a naked trampoline
  (`BRIO_CH32_VECTOR`: a handler with no prologue of its own, a call of
  an ordinary function and MRET, whose body saves only what a callee
  must), and `brio check ch32vx03` walks every such body in the linked
  images and fails on a floating-point instruction it reaches; a body
  that needs float binds with the float trampoline
  (`BRIO_CH32_VECTOR_FLOAT`), which saves the twenty and fcsr itself,
  its body starting from fcsr's reset state; and a leaf, a handler that
  calls no function and so saves nothing under the interrupt attribute,
  binds as the attributed handler still (`BRIO_CH32_LEAF_VECTOR`), the
  same check failing a vector bound leaf that is none, failing any
  binding but the three macros, and reporting a trampoline whose body
  is one ([platform.md](platform.md)). Measured on the CH32V303VCT6: a posting
  receive vector 65 cycles a byte against 105 as an attributed handler,
  a handler that calls out 65 cycles of round trip against 100, a leaf
  53 to 66 bound leaf against 57 to 70 as a trampoline; and the I2C
  suite's refused last byte at 8 MHz and 400 kHz, which the attributed
  handlers' saves cost this die's own target, kept ([i2c.md](i2c.md)).
- **The global mask leaves a critical section whole; a line's own
  disable does not.** The manual's V2.5 revision adds a note asking for
  a `fence.i` after a mask. Measured with a line pended by hand and
  masked at every distance from the pend: on the CH32V303VCT6 an
  interrupt not yet taken when the `csrrci` on mstatus.MIE retires is
  never taken after it (0 of 5000 races, with and without a `fence.i`,
  with and without the hardware prologue), and on the CH32V203C8T6 one
  arriving at a single lead is taken at the boundary right after the
  `csrrci`, the mask already in force and no masked instruction run
  (250 of 5000, the same both ways); while a store into the line's own
  PFIC_IRER lets it through up to three instructions later on both, 750
  races of 750, a `fence.i` after the store changing nothing. The
  platform's guard is the `csrrci` ([platform.md](platform.md)).
- **A `wfi` with the global mask clear sleeps past every pending
  interrupt**, as on the CH32V00x: measured on the CH32V203C8T6 with
  the STK and TIM2 pending and enabled, woken only by a debugger's
  halt. The platform sleeps as a WFE with the mask lifted first, and the
  sleep entry swept to the cycle against the tick's edge loses no wake
  in either order on that die; with the event latch the wake sets
  consumed after it, none lost and one kernel turn per interrupt on
  both parts ([platform.md](platform.md)).
- **The reference manual's vector table is the union of four
  families.** Its table 9-2 names TIM8 at entries 59..62, which belong
  to the CH32V30x; the CH32V203's tail is USBFS, its wake-up, UART4 and
  the eighth DMA channel, and the CH32V203RB's is different again. The
  crt states the per-class truth
  ([startup_ch32vx03.S](../../ch32vx03/src/glue/startup_ch32vx03.S)),
  and for the CH32V303 it is WCH's own startup file for the class in
  every number but two: that file leaves entries 58 and 84 zero, and the
  silicon raises both. EXTI line 18's software trigger pends PFIC line
  58 and line 20's pends 84, read over the debug port with the core
  halted, where line 17's pends the RTC alarm's 57 as the table says.
  The manual's V2.5 revision draws one table per class, and what it
  gets right and wrong against the silicon is in
  [vendor/README.md](vendor/README.md).
- **The clock task must park on the HSI before it touches the PLL.**
  PLLMUL, PLLSRC and PLLXTPRE are writable only while the PLL is off,
  and the PLL refuses to stop while it is the system clock, so a tree
  that arrives at `init()` with the PLL already running takes every one
  of those writes in silence and keeps the rate it had, while every
  divisor in the program is computed for the rate it asked for.
  Measured both ways: the failure staged on purpose, and the fix
  recovering 144 MHz within a millisecond of re-entry.
- **The PLL's input divider is in another block, and on the other
  class it is not even the same divider.** RCC says only which root
  feeds the PLL; whether the HSI arrives whole or halved is
  `EXTEN_CTR.HSIPRE`, which the RCC chapter never mentions and which is
  0 out of reset, so a program that only wrote RCC registers would find
  every rate half of what it asked for. The HSE's own divider is an RCC
  bit (PLLXTPRE), and what it divides by follows the device class: one
  or two on every part up to the CH32V203C8 and on every CH32V303, four
  or eight on the CH32V203RB, whose oscillator is 32 MHz.
- **PB1 is not the datasheet's 144 MHz.** The block diagram rates both
  peripheral buses at the core's own ceiling; WCH's own clock code sets
  PPRE1 = /2 at every rate it offers, and only the bare-HSE path leaves
  it undivided. This stratum caps PB1 at 72 MHz. Where the real ceiling
  lies between those two numbers is not known.
- **UART4's pads are the part's, not the family's.** The manual has two
  remap tables for it; the one that starts at PC10/PC11 belongs to the
  bigger classes, and the CH32V203C8 is named explicitly in the other,
  whose default pads are PB0 and PB1. The CH32V303 reads the first, and
  its UART4 carries the board pair's serial link (the two boards are
  wired to each other, each the far chip of the other's suites) on that
  table's second column, PB0/PB1, its engines on DMA2
  ([usart.md](usart.md)).
- **A pad two peripherals share goes to an enabled transmitter, and a
  timer's forced level waits for a running counter.** With a USART's TX
  pad in alternate-function push-pull and a timer's channel enabled on
  the same pad, a port with UE and TE set holds the pad and the timer's
  low does not reach it, while with TE clear the timer drives it; and
  an output channel's forced level or toggle is evaluated only while
  its counter runs, a counter that never ran holding the output low:
  the two facts the board pair's serial instrument hands a pad between a
  port and a timer by ([usart.md](usart.md), [tim.md](tim.md)).
- **Two AFIO fields the manual gives the whole family are the class's.**
  USART3's remap and TIM2's internal-trigger bit are read-only at zero
  on the CH32V203C8T6 and take a write on the CH32V303VCT6, which is
  where the driver refuses and offers them ([pin.md](pin.md)); on the
  CH32V303VCT6 TIM2's ITR1 carries TIM8's TRGO whichever value the field
  holds ([tim.md](tim.md)).
- **An EXTI line reaches a peripheral through its event enable, not its
  interrupt enable.** Twenty edges on a pad started no conversion with
  the line merely sensed, twenty with EXTI_EVENR set, and none with
  EXTI_INTENR set instead, while the line's own handler ran for all
  twenty, on both parts. Neither chapter says so; the next block that
  takes an EXTI trigger will meet it, and the CH32V303's DAC takes its
  EXTI line 9 trigger the same way ([dac.md](dac.md)).
- **The USB pads are still GPIO pads.** D- and D+ are PA11 and PA12 and
  the USB device controller reaches them through port A: with that
  port's clock closed, its reset state, the pull-up still works (it
  lives in EXTEN), so the host sees a device attach and starts
  enumerating, and then every packet fails. WCH's own `USB_Port_Set()`
  opens the port's clock and leaves the two pads floating inputs before
  it touches the pull-up.
- **The CH32V303 has no USB device controller**, against RM ch. 21's
  opening, which applies that controller to the whole family. Its clock
  gate, bit 23 of RCC_APB1PCENR, is not there: that register written
  with all ones reads back 0x3A7EC9FF, every gate the manual names for
  this part answering and bit 23 alone of them silent, and the block's
  registers read zero. The part's one full-speed controller is the
  host/device one of ch. 23 (its control register reads its reset value,
  0x06), on PA11/PA12, which the datasheet's alternate-function table
  gives the OTG_FS pair and WCH's own board labels USBFS. So `has_usbd`
  is false in its part table, and a USB console here runs on the
  host/device controller in device mode ([usbfs.md](usbfs.md)): the
  same stack, class and descriptors over another controller, which
  drives exactly that pair (measured).
- **The USB controllers do not survive the core's sleep**, and the
  manual says they should. Chapter 2's table gives Sleep as "core clock
  off, no effect on other clocks" and its prose as "the core stops
  running and all peripherals are still running"; measured on the
  CH32V203C8T6, with the core in WFI or in the platform's WFE idle, the
  device controller cannot reach its packet memory. An armed bulk
  endpoint receives nothing (the host's bytes are lost and the
  packet-memory overflow counter climbs one per attempt), and an
  enumeration never gets past its first control transfer. The same
  image with a loop that only `step()`s enumerates, configures and
  carries bytes with not one overflow; both idle forms fail and a
  spinning loop works, so it is the core being asleep and not the
  idiom. WCH's own reference implementation fails the same way, which
  is what settles whose the finding is: the EVT's SimulateCDC example,
  built with the vendor's compiler and flags and unchanged but for one
  `wfi` at the end of its loop, never reaches CONFIGURED (SET_ADDRESS
  gets through, the descriptor reads do not, the overflow counted 18 to
  26 times), and the vendor's own `__WFE()` fails the same, while a
  busy-wait of the same length in the same loop works; a variant that
  sleeps only once configured loses every one of the 4096 bytes the
  host writes; and the loss happens with the core woken at least once
  per USB frame (the frame interrupt at 1 kHz and a 10 kHz timer
  armed), so it is neither the depth nor the length of the sleep. The
  debug module's keep-HCLK-in-Sleep bit could not be tried: a `csrw` to
  CSR 0x7C0 from the running program resets the part. On the
  CH32V303VCT6 the host/device controller, whose buffers are the
  program's own RAM, fails the other way round ([usbfs.md](usbfs.md)):
  with the core in a bare `wfi` between packets the host's OUT packets
  landed whole, but the IN packets the block read out of RAM carried
  zeros, some thirty per cent of the bytes of a counting stream broken
  with no error at either end, and an enumeration stopped at the
  descriptor reads after SET_ADDRESS. A program that uses USB therefore
  does not sleep, on either part, and it is the platform that enforces
  it: an attached controller counts itself a bus master (the platform
  keeps a count of the DMA channels and USB controllers at work, the
  bus-master count of [sleep.md](sleep.md)) and `idle()` does not sleep
  while the count stands.
- **In Sleep, no bus master but the core gets a cycle.** A
  memory-to-memory DMA started right before a `wfi` on the CH32V203C8T6
  moves nine to twelve bytes, the pipeline between the enable and the
  sleep, and nothing more until the core wakes, while a timer on the
  peripheral bus and the core's own counter count the whole sleep;
  woken once a millisecond it moves eleven bytes a wake. The clocks
  run; the bus matrix serves the core alone. On the CH32V303VCT6 the
  same, on both of its controllers: a memory-to-memory block moves
  48010 words in 2 ms with the core spinning and 37 across 1.9 ms of the
  platform's idle() with the bus-master count set aside, six of them
  before the core slept, and DMA2's first channel 23 bytes across one
  idle() and 120 across ten. The CH32V203's USB device controller
  reaching into its packet memory is such an access, which is why a
  transfer with a payload fails in Sleep and one without (SET_ADDRESS)
  passes, and why no software mitigation short of staying awake works,
  measured on the vendor's own example: unmasking the overflow
  interrupt wakes the core but the packet is gone and the host does not
  retry a failed bulk burst; staying awake for ten or a hundred
  milliseconds after the overflow recovers nothing of the burst in
  flight; holding the endpoint at NAK across the sleep still overflows;
  re-validating the endpoint finds it valid. What does work is not
  sleeping and dividing HCLK instead: the device enumerates and carries
  data with HCLK at 96, 48 and 24 MHz and the USB clock at 48, and
  fails at 12 MHz with the sleeping case's own signature. So on this
  family a program that moves data through the bus (USB, a DMA-fed
  transport) does not sleep; it slows down, to no less than 24 MHz of
  HCLK under the CH32V203's device controller and 12 MHz under the
  CH32V303's host/device one ([usbfs.md](usbfs.md)). And that is a
  mechanism and not a rule the programmer keeps: [sleep.md](sleep.md).
- **A DMA channel serves every request wired to it.** A channel's rows
  are an OR of the peripherals' requests, switched by each peripheral's
  DMA bit: a UART4 receiver left enabled made a timer's transfer on
  DMA2's channel 3 move all sixteen items at once. Stopping the channel
  withdraws nothing, and neither does turning the peripheral off: a
  request once raised is held until the channel acknowledges it or the
  peripheral's reset line is pulsed (DMA bits, UE, PE and CEN cleared
  leave it standing, a gated clock freezes it on the OR, and the next
  owner's first enable moves one stray item and then stalls behind it;
  measured on both series with USART2, I2C1 and TIM4 on channels 6 and
  7). So the transports' `release()` pulses the block's reset before
  the gate ([dma.md](dma.md)).
- **A conversion takes 12.5 cycles beyond its sampling time, not the
  manual's 11**: both datasheets, and the CH32V303VCT6 at all four long
  sampling codes ([adc.md](adc.md)).
- **On the CH32V203C8 the sample-and-hold tracks the selected channel
  between conversions; on the CH32V303VCT6 it does not.** A 40 kOhm
  pull-up converted right after a grounded channel in one scanned
  sequence reads 1060 counts at 1.5 sampling cycles and 4095 at 239.5;
  parked on that same channel it reads full scale at every sampling
  time. The sampling time is what a sequence pays for its multiplexer
  moving, while the CH32V303VCT6 reads 1053 at 1.5 cycles and the same
  after 200 us of park.
- **A break input held at its level raises its flag again as soon as it
  is cleared**, which the chapter does not say: on the CH32V303VCT6's
  TIM8 a break vector that returns while BKIN still stands is taken
  again at once, and nothing else runs until the pad falls. A handler
  that may see a standing break masks its own interrupt
  ([tim.md](tim.md)).
- **The RNG's words are not random bits, and SYSCLK is its clock.** On
  the CH32V303VCT6 four thousand words hold 900 to 3350 distinct values
  at every rate and pace tried, a chi-square over their bytes lands near
  49000 on 255 degrees of freedom and FIPS PUB 140-2's poker and runs
  tests fail, while neither of the block's two monitors fires; and the
  block runs on SYSCLK, not on the PLL48CLK chapter 29 names, its word
  time the same number of core cycles at 144, 96 and 48 MHz and unmoved
  by the USB prescaler. The driver hands the words out unconditioned
  and says so ([rng.md](rng.md)).
- **The CH32V303VCT6 of the bench behaves as a lot the manual's lot
  notes restrict.** Five registers the CH32V30x_D8's notes give only to
  lots whose "penultimate sixth digit" (the notes' words for a digit of
  the lot number) is not zero are absent on it
  (TIMx_AUX's dual-edge capture, ADCx_AUX's short sampling times,
  ADC_DUTY_SEL, EXTEN_CTR2's high-speed bits and ADC2's DMA request;
  USART_CTLR4's MARK and SPACE parity, CTLR1's M_EXT, STATR's MS_ERR and
  RX_BUSY; and SPI's HSRXEN2), and the one restriction the notes give
  the other lots is present: DMA1 wraps a span crossing a 64 KB boundary
  inside the page it started in. A program cannot read its lot (the
  notes key on digits of a lot number no register states), so each
  feature is a verb that asks the die, writing its bit and reading it
  back before anything else (TIMx_AUX kept no bit on TIM1 and TIM2
  alike, and the dual-edge verb answered false), and the restriction is
  kept on every lot ([dma.md](dma.md), [adc.md](adc.md),
  [tim.md](tim.md), [opa.md](opa.md), [usart.md](usart.md),
  [spi.md](spi.md)). CTLR4 and M_EXT were asked of the wire as well,
  stored raw past the verbs: the parity bit stayed computed and the word
  eight bits long, so they are absent and not write-only.
- **An absent register can be a mirror.** On that die EXTEN_CTR2's
  address reads and writes EXTEN_CTR: a high-speed bit set there lands
  in EXTEN_CTR's USB bits and a whole word rewrites the regulator trims.
  Reading a bit back cannot tell that from a register that is there;
  reading the word can, and the amplifier driver does it before any
  write ([opa.md](opa.md)).
- **PWR_CTLR's RAM-retention bits read back zero on the CH32V303VCT6,
  and act.** Bits 16 to 20 (R2KSTY, R30KSTY, their two VBAT twins and
  RAMLV) read 0x0 whatever was written, from the program and over the
  debug link alike, while a Standby keeps exactly the banks they name.
  So every read-modify-write of the register writes them away: with the
  bits set by one store and the next store made from a read, a Standby
  kept not one word in any bank. The register therefore has one store
  in the stratum, which carries the program's choice from a copy in RAM
  (the RTC chapter's DBP included), and the retention verbs answer that
  copy. On the CH32V203C8T6 the same bits read back
  ([sleep.md](sleep.md)).
- **A SETUP's status byte names no endpoint.** On the CH32V303VCT6's
  host/device controller the endpoint field of the status byte at a
  SETUP read 6 (whatever the last IN or OUT left there, or its reset
  garbage) and the receive length beside it 1022; a driver that
  believed the field dropped every SETUP of an enumeration, the setup
  packet itself sitting whole in the buffer the whole time. The chapter
  does not say so, and WCH's own sequence never looks
  ([usbfs.md](usbfs.md)).
- **The independent watchdog stops in a Stop and ends a Standby.** On
  both parts it does not count through a Stop (its reset arrives some
  200 ms after the wake, and on the CH32V303VCT6 with the debug
  module's freeze bits read clear), against 7.2.1's promise that it
  keeps working there; on both parts it ends a Standby as a reset, some
  230 ms into one an alarm would have ended at two seconds, IWDGRSTF
  beside PORRSTF, as 2.3.4 lists it. A program that sleeps deep still
  arms the RTC's alarm as its way back ([watchdog.md](watchdog.md),
  [sleep.md](sleep.md)).
- **gcc folds two identical interrupt handlers into one that calls the
  other.** Two bindings with one body (a transport's two DMA channels,
  each vector calling the same dma_isr()) are merged by gcc's identical
  code folding (upstream 16.2 and WCH's 15.2 alike, at -Os and not at
  -O2, with the fast attribute or the plain one, where clang folds the
  same only with its merging pass switched on) into a handler that
  saves what a caller saves and calls the first; the first ends in MRET,
  so the call never returns and the caller's frame stays on the stack.
  On the V4B with the hardware prologue that frame is empty and the
  merge is harmless by luck; on the V4F it is twenty f-registers, and
  the interrupted program resumed eighty bytes below its frame
  (measured on the CH32V303VCT6: its next return jumped to a saved
  float). The attributed binding of BRIO_CH32_VECTOR and
  BRIO_CH32_LEAF_VECTOR carries no_icf; a trampoline needs none, two
  bodies that fold being ordinary functions
  ([platform.md](platform.md)).
- **A CH32V303VCT6 I2C target loses the last byte before a late
  repeated START.** With the host's START requested after BTF (the last
  written byte acknowledged and SCL held low) that target never raised
  RxNE for the byte, in silence (the chip's two controllers on one bus,
  the target polled by the core that also serves the host, at every
  count and both speeds); a CH32V203C8T6 target on another board, under
  a CH32V303VCT6 host, took it in all twelve of the suite's late-START
  probes, whether it took a standing byte before answering the address
  or after. Requested while that byte still shifts, as WCH's own
  example does, every byte arrives at both, and that is where the
  engine requests it ([i2c.md](i2c.md)).
- **An I2C START on a busy bus leaves at the controller's tick, not at
  the STOP.** With SDA held low and SCL standing high, a pending START
  reaches the wire about 85 us after the edge that made the bus busy,
  or a multiple of 80 us after that; one set on a free bus, or pending
  when a STOP frees it, reaches SDA some 5 us later whatever the wire
  does meanwhile. Two controllers' STARTs meet only where one STOP
  releases both or the second is set within those 5 us, and a tenure
  into a line a stuck target holds is answered at once instead of
  parked (measured on the CH32V303VCT6, a CH32V203C8T6's START on the
  same beat; [i2c.md](i2c.md)).

## Not covered yet

Driver gaps, each with its reason:

- **CAN**: the one chapter still open, waiting for the pass across every
  platform that brings a second transceiver to the desk; it arrives with
  its own document and its own suite.
- **TKEY, WCH's capacitive touch sensing (ch. 13), declined**: the block
  is an ADC mode (the converter's charge transfer against a pad's
  capacitance), and what it produces is a key press, which is an
  application and not a driver. Its two bits are named in the register
  map and nothing writes them.
- **SDIO, the CH32V303RC's and VC's card interface (ch. 28)**: the
  evaluation board has no card socket, so a driver would have nothing
  to answer to; the block's base, its clock gate and its vector are in
  the register map and nothing drives them.
- **The FSMC, the CH32V303VC's static memory controller (ch. 26)**: a
  memory on the bus is what the chapter wants and the board carries
  none; the base and the gate are in the map, and its windows are read
  only by the DMA chapter, as addresses that raise no transfer error
  ([dma.md](dma.md)).
- **The CH32V305 and the CH32V307**, the CH32V30x_D8C class the same
  datasheet covers: another device class (a second PLL and a third, the
  high-speed USB, the Ethernet, a second CAN) that no board here
  carries, so the part table does not name them.
- **A maintained teaching example**: no `blink` builds in this project.

Implemented but not bench-verified, each with what would measure it:

- **The eleven parts other than the CH32V203C8 and the CH32V303VC.**
  Every one has its table, its linker script and its preset, and the
  whole stratum compiles for all thirteen both ways the hardware
  prologue can be built (`brio check ch32vx03`); the CH32V203C6 preset
  links every image (the check's link guard) and is the 32 KB / 10 KB
  tier's guard, and the CH32V203RB's console links with its
  seventy-word vector table. What would measure them is a board.
- **The KEY buttons on PA0**: no program has pressed either. The WeAct
  board's is the vendor's claim, and there the pad is USART2's CTS and
  carries the wake edge of [sleep.md](sleep.md)'s suite when a second
  board drives it; the evaluation board's ties its pin to ground
  through 10 kOhm when pressed and the board gives it no pull-up (its
  schematic). What would read either is a press, which
  [pin.md](pin.md)'s suite keeps a letter for, by name.
- **The lot-keyed features on a die of the other lots.** TIMx_AUX's
  dual-edge capture, ADCx_AUX's short sampling times, ADC_DUTY_SEL,
  EXTEN_CTR2's high-speed bits, ADC2's DMA request, USART_CTLR4's MARK
  and SPACE parity, M_EXT's short words, MS_ERR, RX_BUSY and SPI's
  HSRXEN2 are verbs that ask the die, and the CH32V303VCT6 answered no
  to each; what would measure them is a CH32V303 of a lot whose
  penultimate sixth digit is not zero.
- **The host/device controller on the CH32V203's pads.** Five CH32V203
  parts carry it on PB6/PB7, the CH32V203C8 among them beside its device
  controller, and `Usbfs` builds for all five; the WeAct board wires its
  one connector to the device controller's pads, so what would measure
  it is a board with a connector on PB6/PB7 ([usbfs.md](usbfs.md)).

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
| [device.hpp](../../brio/ch32vx03/device.hpp) + [parts/](../../brio/ch32vx03/parts/) | The register map and the part table: thirteen parts under three device classes (the CH32V203's nine in the CH32V20x_D6 and the CH32V20x_D8, the CH32V303's four in the CH32V30x_D8), each with its memories (the zero-wait window, the tail above it and the whole array), its bonded pads, its instances, its core's FPU or none, its vector table's tail and the interrupt lines its class has not got |
| [platform.md](platform.md) | Platform, its code the QingKe core stratum's over this family's facts ([../qingke/README.md](../qingke/README.md)): `Ch32vx03Platform` (the csrrci critical section, the WFE-shaped `idle()`, `ebreak`, the `.noinit` breadcrumb), `Pfic` and the one vector binding `BRIO_CH32_VECTOR` with its float and leaf forms (the core's hardware prologue measured on the CH32V203C8T6: 52 cycles of round trip against 60, 144 bytes of flash and the user stack a handler's saves take, which the internal hardware stack carries instead and which is what parts this core from the CH32V00x's; every vector aligned to four, 52 cycles against 56), the 64-bit STK `BasicTicker` (its CNT arithmetic 1 ppm against the interrupt count over 200 reloads) and `delay_us` on that counter (100 us in 14434 cycles of 14400 asked, a tick period and above refused), corecfgr's 0x1F measured at two cycles in 36811, the tick rate against the host's clock (the HSI half a per cent fast where the board's crystal is exact) and the idle hook's sleep entry swept to the cycle against the tick's edge, no position losing the wake and a masked `wfi` never waking; then the failing half, [reset.hpp](../../brio/ch32vx03/reset.hpp): the six flags as the history they are, `Reset::software()` through the core's keyed PFIC_CFGR reading back as SFTRSTF alone, `ResetReporter`, `fault_reset<P>()` carrying the cause the core left in mcause, and the ebreak that lands on the BREAKPOINT vector and not the exception one; three real resets in the suite. On the CH32V303VCT6 the same letters (a round trip of 52 to 62 cycles, corecfgr's bits slower there by half to eight tenths of a per cent) and the V4F's floating-point unit: off at reset and switched on by the crt of an image built with F, left Dirty by the crt's own fcsr write, recording its exceptions and never trapping on them, twenty float accumulators exact through 23262 interrupts whose handler does float work of its own, every vector of an image built with F bound as a naked trampoline (the check walking every such body for a floating-point instruction), a round trip of 48, 125 and 66 cycles for a leaf, with float work under the float trampoline's twenty saves and fcsr, and calling out (54, 67 and 100 as attributed handlers), a posting receive vector 65 cycles a byte against 105; the bus in sleep asked again of that die, a DMA block moving 37 words across 1.9 ms of idle() against 48010 awake; and the mask's shadow (none behind the guard's csrrci, 0 of 5000 races; up to three instructions behind a store into a line's own PFIC_IRER, 750 of 750, which a fence.i does not close); and the attributed binding carrying no_icf, because gcc folds two handlers of one body into a call of the first, which ends in MRET: on the CH32V303VCT6 the caller's twenty f-register saves stayed on the stack and the interrupted program's next return jumped to a saved float; and `Ch32vx03RubatoPlatform`, the preemptive kernel's ([schedule.hpp](../../brio/qingke/schedule.hpp): the software interrupt as the request, one preemption level below every peripheral line with nesting turned on at run time, returning into the scheduler in thread mode with the trap's mepc and mstatus in a block on the stack, the machine-mode `ecall` the way back, the f-registers and fcsr saved by the entry where FS is Dirty): road 1 in 144 cycles from the handler's last statement to the dispatch's first on the CH32V203C8T6 and 149 on the CH32V303VCT6, a quiet post 57, a road-2 post 370 with the receiver's empty dispatch inside, the stack at most a level per object of the pack, the f-registers bit-exact across a preemption and their save 27 to 35 cycles in and 25 to 29 out; and by `test_vx03_rubato_link`, the SPI and UART stacks under Rubato with an object preempting at prime periods, each part against the other's peers (231 verdicts on the CH32V203C8T6, 232 on the CH32V303VCT6): byte-exact, the polled SPI loop cut short by a detected overrun where a preemption outlasts a frame with two in flight and never handing back a run with a hole; and by `test_vx03_rubato_i2c`, the I2C stack the same way at 96 MHz (202 verdicts on each part), byte-exact, a preemption reaching the host only before its START |
| [clock.md](clock.md) | RCC and the EXTEN bits that belong to the tree: the two high-speed roots, the PLL whose input divider is per device class (the HSI's in EXTEN, the HSE's dividing by one or two here and by four or eight on the CH32V203RB) and whose input and output ranges are part facts, the whole prescaler table with PB1 capped and the timers' doubling rule stated, the USB divider written before any USB gate can open, the ADC's divider (the one place a legal rate leaves a peripheral out of specification), the LSI, the clock security system as the non-maskable interrupt's body, the ready interrupts, the output pad two packages have not got, the peripheral gates; `Clock` static and `DynamicClock` over a pack of rate tuples, every switch parking on the HSI. Measured on the CH32V203C8T6: ten trees entered, read back and bracketed against the host's clock (every HSI-rooted rate 0.42 to 0.46 % fast, every crystal-rooted one within 0.05 % of exact), the pack walked up and down with the tick and the console rebased at each step, the LSI ready 3.1 ms after LSION and the board's crystal 1.7 ms after HSEON with its ready interrupt reaching the RCC vector, the HSI stopped under a tree that runs off the crystal, the security system armed over a healthy crystal, twenty-six gates opened and closed (the bits of blocks this part has not got reading back zero). On the CH32V303VCT6 the same ten trees and 49 verdicts: the HSI-rooted rates a third of a per cent fast and the crystal's within a tenth, the LSI ready 4.4 ms after LSION, the board's crystal 1.05 ms after HSEON, and forty-two gates, with the USB device controller's bit among those that read back zero, that part having no such controller |
| [pin.md](pin.md) | GPIO, the remaps and the EXTI: the F1's four-bit nibble over two registers with a MODE that carries a speed, the pull that lives in the output register and belongs to input mode alone, the whole-port verbs and the one-way configuration lock, over the five ports the LQFP100 bonds; the remap columns of AFIO_PCFR1/PCFR2 as constexpr pad tables judged by the device class and by the package's bonding (the CH32V303's own among them: USART1's four columns, UART4's other table, TIM8/TIM9/TIM10, UART5..UART8, SPI3 with I2S3, the ADC trigger remaps, FSMC_NADV; CAN2's and the Ethernet's refused), the debug port's own field read and never written, the event output whose source no document of this family names; and the twenty-two EXTI lines (sixteen pin lines through AFIO_EXTICR with the one-pad-per-line rule refused by `select()` and overridden by `steal()`, the PVD's, the RTC alarm's and the peripheral wake-ups each part's table states, the senses, the two enables, the software trigger and the write-one flags over five single vectors and two shared ones). Measured: all fifteen configurations read back in both registers on both parts, the LQFP48's unbonded nibbles reading zero where the LQFP100's full port reads the manual's value, every free pad of ports C, D and E driven and pulled both ways, an open-drain output that drives nothing high, nine remap columns written and restored on the CH32V203C8T6 and thirty-one on the CH32V303VCT6, two fields the CH32V203 holds at zero against the manual's own notes and the CH32V303 takes, five/five/ten edges of a pad on its own line, an edge over a wire reaching its handler in 17 to 20 cycles, and a line event ending idle() in 15 and 34 core cycles |
| [tim.md](tim.md) | The timers: TIM1 and TIM2..TIM4 on every part, a 32-bit TIM5 on the CH32V203RB, and on the CH32V303RC and VC the whole chapter set (three more advanced timers, TIM8, TIM9 and TIM10 with four unshared vectors each, a sixteen-bit TIM5 and the two basic timers TIM6 and TIM7, on which a channel verb does not compile); the F1's register file under WCH's names (the two shadow registers, the four channels in both faces with CCyS writable only off, the slave controller with its three encoder modes and the master TRGO, the internal trigger table folded through what the part has, the advanced timers' repetition counter, complementary outputs, dead-time generator and break input, the DMA burst engine and the class's DMA2 requests as data, the CH32V30x_D8's dual-edge capture as a verb that asks the die); the pads taken from afio.hpp's remap columns, and the nine tasks. Measured on both parts: the counter and both shadows against the core's counter, a PWM captured by its own timer to the microsecond at four duties, a dead band of 27.9 us against 28 asked, the interval and period meters behind a MeterLatch, two timers counting each other over an internal trigger (201 updates, a 25 % duty gated to 5000 us), a one pulse 501 us wide for 500 asked, forty encoder counts for ten quadrature cycles, a pad in plain output mode reaching a capture input and a channel's output stage cut off from its pad in an encoder mode, the repetition counter's one update in RPTCR + 1 periods within 42 cycles edge- and centre-aligned (every odd-count update at an underflow), the three centre-aligned modes raising the compare flag counting down, up, or both (600 and 400 us apart); on the CH32V303VCT6 a second timer reading the first over a wire (999 and 299 us for 1000 and 300, and the centre-aligned wave 999 and 399 for 1000 and 400 in every mode), TIM8's PWM captured by TIM4 to the count at nine duties and rates up to 100 kHz, TIM8's dead band (27.8 us), four vectors and break from BKIN, TIM9 and TIM10 with no wire, TIM5 sixteen bits wide, TIM6 and TIM7 keeping the core's time with their TRGO counted on TIM9, nineteen internal-trigger links counting 201 each, TIM3's external trigger on PD2 in both external clock modes, a break held at its level re-raising its flag without end, and a die whose lot has no dual-edge register; on both parts, under the serial peer's waveform engine, a forced level or a toggle not evaluated while the counter is stopped (a counter that never ran holding its output low), a compare request keeping up with edges 14 core cycles apart and missing one 12 apart, and the one pulse returning its line high by itself |
| [watchdog.md](watchdog.md) | The two watchdogs, the same blocks on every device class: the independent one on the LSI (three keys, no way back but a reset, the oscillator forced on, and its two registers taking a write only while that oscillator runs, which is why arm() starts before it configures and ends with the refresh that re-locks them) and the window one on PCLK1/4096/2^WDGTB (the counter that does not run unarmed against its own chapter, the clock gate under which the registers still read their reset values and drop a write, the window whose early refresh is the reset, the early wake-up one tick before it, the block's reset line as the only way back). Measured on the CH32V203C8T6 and the CH32V303VCT6 with the same verdicts: the tick at all four prescalers to the microsecond (2628/5347/10694/21389 us for 47 ticks), EWIF at 28671 us of 28672, three real WWDG resets (29 ms, 0 ms for an early refresh, 29 ms with the interrupt having run once) and the IWDG's own at 206 and 207 ms on the CH32V203C8T6 and 199 to 217 ms on the CH32V303VCT6, which puts those dies' LSIs at 38.8 and 36.9 to 40.2 kHz; the IWDG at all seven prescalers implying one LSI per die within each leg's derived window; both force_reset()s back at once under their own flag (0 to 3 ms); and through the two deep modes, the independent watchdog frozen in a Stop on both parts (its reset 215 ms after a two-second Stop on the CH32V203C8T6, 208 and 209 ms on the CH32V303VCT6) and ending a Standby on both, 225 ms (the CH32V303VCT6) and 228 to 230 ms (the CH32V203C8T6) after the alarm meant to end it was placed, with IWDGRSTF beside PORRSTF |
| [dma.md](dma.md) | The DMA controllers: one of eight channels on the CH32V203 and two on the CH32V303 (a DMA1 of seven and a DMA2 of eleven whose last four report in a flag pair of their own), where the channel is the request: no multiplexer, the tables wiring each peripheral event to one channel of one controller, so a request lands in a slot, controller and channel, and every type names its controller first; the part's own table deciding which rows exist, a channel's rows an OR (a peripheral left with its DMA bit set drives whatever the channel holds); the configuration word, the count read-only while the channel runs, the addresses the silicon would align in silence and this driver refuses, the CH32V303 DMA1's 64 KB boundary refused on every lot, four flags and one vector a channel, blocks with no reset line in RCC, `any_enabled()` asked of both controllers; then the four engines on either controller. Measured on both parts: six cycles an item at all three widths (23.8, 47.1 and 92.3 MB/s at 144 MHz), the half flag at 2049 and 2050 of 4096, every configuring verb refused under a running channel, the nine pairings of two unequal widths as table 11-1 draws them, five addresses outside the map read or written and the locked flash written all completing as ordinary blocks instead of raising the promised transfer error, SPI2's and SPI3's requests leaving nothing held after `SpiHost::release()` where UART4's transmit request on DMA2 is held until its reset, a circular run reloading itself at a timer's pace, a staircase of another timer's counter 500 us a step, the burst engine's four compares, the software priority beating the channel number and the channel number beating a head start, and the sleep: a handful of bytes a wake; on the CH32V303VCT6 DMA2's eleven channels copying out of flash with 8 to 11 in the extended pair, all eighteen vectors each the channel's own, TIM5's, TIM6's and TIM7's updates as DMA2 requests, both block engines on DMA2, DMA2 starving in Sleep as DMA1 does, UART4's engines on DMA2 byte for byte round the far chip's echo, a receiver left running serving a timer's transfer sixteen items at once, and the 64 KB rule real on that die (with the refusal bypassed, DMA1 read the upper half of a span across 0x0801 0000 from the bottom of the same page); and every engine at its peripheral's level ([../design/dma.md](../design/dma.md)): a copy back to back on a lower channel starving no byte of a ring at 1 and 4.5 Mbaud (on the CH32V303VCT6 UART4's ring on DMA2 fed by the far chip), the same ring told to arm low overrun some nine hundred times in 8 KB |
| [adc.md](adc.md) | The two converters: the STM32F1's ADC under WCH's names with an input buffer and a gain of 1, 4, 16 or 64 in front of it; how many converters and bonded channels a part has (two and nine or ten up to the CH32V203C8, one and sixteen on the 128 KB part, two and ten or sixteen on the CH32V303), the one place a legal clock tree leaves a peripheral out of specification (ADCCLK is PCLK2 over 2, 4, 6 or 8 against a 14 MHz rating, refused at compile time above 112 MHz of HCLK), the calibration and the order that hides in BUFEN's note, the regular group of sixteen and the injected four with a signed offset, the eight sampling times against the source impedance each settles and a conversion 12.5 cycles longer than its sampling time (the datasheets' figure, not the manual's 11), the analog watchdog, code 110 an EXTI line that the CH32V303RC and VC can hand to TIM8, the DMA request as a block stream, the dual modes, one vector for both units, and the CH32V303's lot-keyed additions (four short sampling times, ADC2's own DMA request, a 75 % ADC clock), each asked of the die. Measured: the calibration at 8 us leaving 2049 (3 us and 2044 on the CH32V303VCT6), VREFINT at 1489 counts = 3301 mV, the sensor at 1404 mV, a 40 kOhm pull read 1060 counts at 1.5 cycles and 4095 at 239.5 in a scan (full scale at every sampling time when the multiplexer is parked on the CH32V203C8T6, a quarter of the scale however long it is parked on the CH32V303VCT6), four blocks lent through a relay in 679 us with the overrun staged, two or three items past a half flag before the fastest handler can stop the channel, a timer's TRGO pacing 399 conversions in 200 ms, an EXTI line reaching the converter through its event enable and not its interrupt enable, both converters' data in one register, a discontinuous sequence stepping 2, 4, 6 with one EOC, the 12.5-cycle tail at four sampling times on both parts, ADC2 alone answering as ADC1 does, JAUTO's twenty conversions from one start, the fast and slow interleaves timed at 7 and 14 ADCCLK with the slow one restarting itself every 28, injected simultaneous, alternate trigger and its turn outliving the mode (ADC1's next injected start wedged until ADC1's init()); on the CH32V303VCT6 the part's own DAC as the source (thirty-one codes on a line of gain 0.9984 with no residual above 2.6 counts, the PGA at 4.01, 16.18 and 65.79), code 110 handed to TIM8 and paced by it at 200 conversions in 100 ms, and a die whose lot has none of the three lot-keyed additions |
| [dac.md](dac.md) | The CH32V303's digital-to-analog converter: two 12-bit channels on PA4 and PA5, which are also the ADC's inputs 4 and 5, so the converter reads the DAC through the bond pad they share; nine spellings of a holding register that is not the output register, the buffer disabled by a one, table 17-1's eight triggers with the absent timers refused, the noise and triangle generators, the DMA request a hardware trigger raises on DMA2's channels 3 and 4 handed to a block engine in one verb, the dual register fed by one request, and no status register, no interrupt and no underrun. Measured on the CH32V303VCT6, read back by the ADC: six placements in one register per channel, seven codes on each pad within 13 counts buffered and 10 unbuffered with the rails at 0 and some 10 mV under the supply, the TRGO of all six timers holding a datum until the update, EXTI line 9 reaching the converter through its event enable as the ADC's does, the noise generator equal to figure 17-5's register for sixty-four triggers at three widths, the triangle turning at both ends (both generators and the software trigger on each channel, and both channels from one store), a table played for ever by the loop engine on DMA2's channel 3 and found on the pad level by level, and into DAC2 on its own request on DMA2's channel 4, both channels fed a word a trigger from one request, and each channel following its own configuration with both enabled (V2.3's note otherwise does not hold) |
| [opa.md](opa.md) | The operational amplifiers, two on the CH32V203 and four on the CH32V303: four bits each in one register inside the EXTEN block's window (an enable, one of two positive pads, one of two negative ones, one of two outputs) and nothing else: no key, no lock, no gain, no internal feedback, so with no wire strapped the block is an open-loop stage, a comparator. Every first output pad is an ADC input pad; the CH32V303's second outputs are port-E pads no converter reads, and its high-speed bits live in EXTEN_CTR2, a lot's register whose address mirrors EXTEN_CTR on a die without it, so the driver reads the word before it writes a bit; the twenty-pin CH32V203 has OPA2 alone. Measured on both parts: saturation to the rail and to ground on every amplifier and input pair, a selection moving the level to the new output pad and off the old, the high rail 1 to 4 mV under the supply, the four-bit fields independent, the amplifier winning against its pad's 40 kOhm pull; on the CH32V303VCT6 OPA3 and OPA4 on both input pairs, the four port-E outputs read as levels both ways, a differential step followed in about a microsecond, and a die with no EXTEN_CTR2 whose address read EXTEN_CTR's word |
| [usart.md](usart.md) | The serial ports: up to eight instances on two peripheral buses (USART1 on PB2, every other port on PB1), every divisor asked of the instance's own bus, and which instances a part offers taken from the datasheets' tables (one on the smallest CH32V203, four on the CH32V203C8 and RB, three on the 128 KB CH32V303, all eight on the CH32V303RC and VC, UART6..UART8 below USART2 in the map); the frame in every shape M and STOP allow, mute mode with both wakes, LIN's break, single-wire half duplex, IrDA, the smartcard and the synchronous clock of a full instance (the fourth port a USART4 on the CH32V203C8 alone, the full-USART verbs compile errors on a UART), the flow-control pair, the two DMA requests as slots (UART4..UART8's on DMA2 on the CH32V303), every flag with the sequence that clears it and both interrupt sources; the pads are afio.hpp's columns, a remap code of 0 writes no register at all and the two columns on the debug port's pads are refused while the probe's port is alive; the CH32V303's lot-keyed CTLR4, M_EXT, MS_ERR and RX_BUSY as verbs that ask the die, CTLR4's address read as a word first; then the transport, with its frame, its two engine slots and its trailing options. Measured on the CH32V203C8T6: twenty-eight baud rows within one per mille on both buses with 1200 refused from the 144 MHz one (the register's sixteen bits, not the generator's floor), a start bit that is the divisor to the clock at four rates from 9600 to 3 Mbaud, the word length as a frame's low run, the two half stop codes coming out as whole ones, TE's idle frame at 1146 us with TXE back in 104 us on an idle transmitter, PE delivering its byte and ORE keeping the data register's, both mute wakes with 18.10.4's two notes, a break of ten bits outside LIN and thirteen inside it with the detector at ten and eleven, the infrared decoder taking the mirror of its own encoder's pulse, seven clock pulses a byte with LBCL clear and eight with it set (the F1's sense and not this manual's words), a card clock of 6001500 Hz for the 6 MHz asked and only with the transmitter enabled, and the CTS hold with the RTS hand; on the CH32V303VCT6 the same letters with the same numbers, UART5..UART8 each with its gate, divisor, pads and vector (69, 87..89) and no wire, the debug-port columns refused with SW_CFG reading the probe's port alive, and a die with none of the lot's four features (the probe finding CTLR4's address empty and moving nothing and the wire agreeing: the parity bit computed and the word eight bits long with both stored raw); on both parts the receive engine at very_high (`UartOptions::rx_priority`) and an overrun with nothing to take cleared on its wait's second unmoved entry (three vector entries and the overrun counted once, where the storm took 256, a DMA fault and the next message). The pair, the CH32V203C8T6's USART3 crossed to the CH32V303VCT6's UART4 on its second column, each against the other's usart_peer on its crystal: the two crystals agreeing within nine ppm from both sides, every rate PCLK1 over BRR to a few ppm from 9600 to 4.5 Mbaud as the far receiver times it, the half stop codes rounded up on the wire, the receiver clean from -4 to +5 per cent and its start filter between half a bit and three quarters, three sampling instants a sixteenth apart whose bit under partial noise is not their majority in every case, PE, FE, a ten-bit break and ORE from the far engine's drawn trains, half duplex hearing a second chip and the wired AND of an open drain, IrDA between two WCH codecs, the receive engine taking a second of 4.5 Mbaud with two interrupts a lap, each ring counting a run of the far chip's errored frames every other one with every byte kept (one in eight where the wait for a frame read STATR for an idle transmitter: why it reads CTLR1) and telling a burst of sixteen within 1.1 frames on one interrupt, `rx_pending()` exact on the ring and the interrupt receiver, UART4's engines on DMA2 carrying four kilobytes each way at 921600, and the lot's absent registers confirmed by the far receiver; the peer's engine a timer's toggle list streamed by DMA, an enabled transmitter owning the pad it shares with the timer |
| [spi.md](spi.md) | SPI and I2S: up to three synchronous serial ports (SPI1 on PB2, SPI2 and SPI3 on PB1, so one BR code is two frequencies and a program asks the instance), the STM32F1's SPI under WCH's names with no FIFO at all (one buffer each way, which is what makes a host's pump run on RXNE and a client answer one frame ahead), the four modes, both widths, both bit orders, the three chip-select arrangements, the simplex and one-wire line modes, the hardware CRC, the two DMA rows a slot each (SPI3's on DMA2), the pads as afio.hpp's columns, the high-speed read at BR /2 alone (the one code no lot of any class reads differently) with the lot's HSRXEN2 a verb that asks the die, and on the CH32V303RC and VC the I2S face of SPI2 and SPI3 (the four standards, the widths, both roles and directions, the divider on SYSCLK, the flags and requests the faces share); then the host engine with the other strata's Request verbatim, the client with its dark listener, and a pad-slew verb on both. Measured on the CH32V203C8T6: eight rising edges per frame at every BR code, counted by a timer on the SPI's own clock pad; a strap clean to 36 MHz and breaking at 72; four kilobytes byte-exact; the engines at 36 core cycles a frame against the wire's 32; the arbiter's four transactions, its rejection and both votes; a polled wait that runs out ending `spi_stalled` within its spin budget (12 cycles a spin, 10 on the CH32V303VCT6), the transmit interrupt feeding a host's stream back to back, the write-ahead threshold one code lower on a PB1 instance (SPI2's oracle losing frames at 128 cycles and none at 256, as SPI1's); and that 20.2.7's recipe for clearing MODF does not clear it (fourteen sequences measured, WCH's own library on the same board among them; SPE never takes while the flag stands, and only the block's reset line puts it down). The pair, the CH32V203C8T6's SPI2 against the CH32V303VCT6's SPI3 on its second column, each against the other's spi_peer: the ladder exact both ways to 18 MHz on every host shape and both far serves (at 36 MHz the far client hears every frame and the host reads the answers one bit late, the client's answer valid 25 ns after its edge against a 13.9 ns half period), 16-bit frames, the CRC against a foreign unit with a mismatch caught at both ends, this end's write witnessed by a listening client, each chip's client runs under the far host, MODF raised by a second chip and SSOE's select falling with the first frame, a host the far chip demoted mid-transaction answered `spi_timeout` by the timed arbiter and put back by `recover()`, the engine rebased at three rates of a DynamicClock with its ceiling and write-ahead codes the new bus's, and what the chapter does not say: a late first answer loads mid-frame and the client restarts its frame one bit behind, and over loose wires a client's answer edge reaches its own SCK and select inputs (the pair's cable, series resistors on SCK, MISO and MOSI and two grounds beside SCK, carries every slew class exact, and the pair runs the medium one, the slowest whose edge fits /4's half period, each client answering in all four modes). On the CH32V303VCT6, over four wires between SPI2 and SPI3: the high-speed read turning 36 MHz from 254 bytes of 256 wrong (the client's answer read one bit late) to none; half-words through four channels on two controllers under one per cent over the wire's own cost at every rate; and the I2S face word for word in all four standards, both roles, both directions, every width and on DMA at 48 kHz, its clock SYSCLK to the frame (16015 counted for 16014), the first word the left channel's, and TXE reading 1 with I2SE clear against 20.3.6.2, a datum written there going out first |
| [i2c.md](i2c.md) | The two-wire ports: the STM32F1's I2C under WCH's names, host and target, with a rise-time register the CH32V00x's block has not, so the SCL timing is three registers (FREQ, CKCFGR under F/S and DUTY, RTR from the wire's rise) and the chapter's own ceiling is FREQ's six bits, 4 to 60 MHz of PB1, which makes it the one peripheral of this family that cannot be used at the top of the tree. The event machine's every sequence, the receive procedure by count, 7- and 10-bit own addresses with the dual address and the general call, ACK and POS, SMBus and PEC as bits, the two DMA rows a channel each (a one-byte read stays on the pump), two vectors an instance, the pads as afio.hpp's columns with the SMBus alert pad outside them; then the host engine with the other strata's Request verbatim, the arbiter over it, and the client with a polled option and a flush(). Measured on the CH32V203C8T6 and the CH32V303VCT6, each the other's peer on I2C1, every letter of `z` both ways ten times: both speeds exact at 48 MHz of PB1 and SCL read off its own pad by a timer, no enable protection at all (CKCFGR, FREQ and RTR all take a write with PE set, against 19.12.9's own sentence), an absent address as i2c_nack_addr with BUSY back in 6 us, a held SDA answered i2c_arb_lost by the silicon at the first START into it and unstick()'s three clocks counted on the pad, the four receive procedures byte-exact, a polled CH32 target's stretch landing span by span where its model puts it, both DMA engines, the kernel's arbiter with a held clock answered i2c_timeout at its 20 ms limit, arbitration between two CH32 controllers in both directions, the target as a transmitter read by the far controller with flush() taking the extra byte each read asks for, the dual address and the general call, a far target stuck mid-byte clocked out in 4 pulses, the refused last byte answered at three core clocks with each tenure's mean rate inside a derived bracket, 25 thousand tenures in ten seconds with no error at either end; two facts a recovery has to know (a DMA-served read can leave BUSY standing over an idle wire, 19.12.1's own case for SWRST, and SWRST takes CTLR2's interrupt enables with it; clearing PE does not let a stretching machine's pads go, only the block's reset line does); the repeated START taken while the last byte goes out (requested after BTF, a target served by the core that drives the host lost the last written byte of a write-then-read in silence on both parts, where each part's target on its own core took every one, so the engine requests it a byte earlier, where WCH's own example does); and a START on a busy bus leaving at the controller's tick, about 85 us after the edge that made the bus busy and every 80 us after, not at the STOP, while one set on a free bus is committed 5 us ahead; the stack under the preemptive kernel (`test_vx03_rubato_i2c`): every tenure shape exact on both paths under every pressure, and on a held line the first START losing and the next ones parking until the arbiter's timeout; and an address outside 0x08..0x77 refused by `start()`, the engine ending a tenure on the ADD10 a 0x78 to 0x7B makes instead of spinning on it, the STOP and a DATAR store clearing it (measured with the refusal compiled out) |
| [nvm.md](nvm.md) | The flash memory, the user option bytes and the electronic signature: two programming methods and four erase grains in one chapter (a half-word behind PG and a whole 256-byte page behind FTPG, with 256 bytes, 4 KB, 32 KB and the whole user array as the things that can be erased), three locks each wanting its own key pair in order, a wrong one holding until the next system reset, and an erased pattern that is not all ones (0xE339E339, stated four times in the chapter and the opposite of what the sister family promises); the rate as part of the contract, because the access clock may not exceed 60 MHz and SCKMOD already halves the system clock, so above 120 MHz an erase or a program wants HCLK divided around it, which the engine refuses rather than doing behind its caller's back (at compile time under a static clock and with a code under a dynamic one); the enhanced read mode that would fail an erase in silence, the option bytes decoded read-only with RDP written by no verb, and the signature; on both series the array is a zero-wait window with a non-zero-wait tail above it ((224K - R0WAIT) on every CH32V203, (480K - R0WAIT) on the CH32V303RC and VC), which both methods write and which a read reaches through the array's own address under the same rate contract, the option byte's memory split decoded read-only (FLASH_OBR holding the USER byte whole at [9:2]) and ESIG_FLACAP the largest window the split can select; plus MainFlash, the FlashMedia in the last 4 KB of every part's zero-wait window (one write-protection unit, the page as the cell, the clock in the medium's own type) with no heap and no journal above it. Measured on both parts: the core running out of the array throughout an erase, the kernel's tick advancing by what the ruler measured, so an erase is a wait and not a stall; the erased pattern on the zone and in the third word of each die's own identifier; a half-word cell of the window taking pass after pass between erases, each read back exactly, where a page programmed twice with no erase holds neither pattern nor their AND; a wrong key locking the engine until the next reset and raising no bus error; the enhanced read mode never engaging; FLASH_ADDR not needed by the page program; SCKMOD written and read back in both positions with neither an instruction fetch nor a data read of the window measurably faster at 48 MHz; an erase at 144 MHz refused with a code and nothing written; the medium's cell erased, programmed, read back and found again after a reset. On the CH32V203C8T6 a page erased in 9770 us and programmed in 1540 and a half-word in 2603, its tail's sector and block erased in some 9.8 ms and a word of its tail costing 36 core cycles at 96 MHz (eighteen access-clock periods where the CH32V303VCT6 takes ten); on the CH32V303VCT6 every erase 16.85 ms whatever its grain, a word of the tail costing 20 core cycles against the window's 5; on both, SCKMOD moving the tail's price and not the window's, both methods writing the tail, a tail cell given a second pass not dependable (the second value in most runs, one bit off it in two of eleven on the CH32V303VCT6) where a window cell keeps the second across a system reset, a page of the tail read exact at 144 MHz outside the rating the driver keeps; and on the CH32V303VCT6 the split's code read off FLASH_OBR with FLACAP answering 288 over a window of 256 |
| [rtc.md](rtc.md) | The real-time clock and the backup domain: not a calendar but a number, a 32-bit counter behind a 20-bit prescaler with a second, an alarm and an overflow, kept in a domain that survives a system reset and, on a battery, the loss of VDD; three gates before a register answers (two bus clocks and PWR's DBP), the write window and the read synchronization the chapter demands of every access, a prescaler reload and an alarm that cannot be read back at all, the counter's two halves read against a carry the chapter is silent about, the alarm's second path through EXTI line 17 (the one that survives a low-power mode), RTCSEL one-way with a domain reset the only way back, and an HSE division that is 512 or 128 by lot number on the CH32V20x_D6 (stated as a pair and measured) and one number on the other two classes, 512 on the CH32V203RB and 128 on every CH32V303; then the backup registers, ten on the CH32V20x_D6 and forty-two on the CH32V20x_D8 and the CH32V30x_D8, the tamper input that wipes them and remembers an edge it was not watching for, and the three things the one pad PC13 can carry. Measured on the CH32V203C8T6: the low-speed crystal at 32765.93 Hz (63 ppm slow) counted over ten of its own seconds against a crystal-rooted core; this die dividing the HSE by 128 (RTCCLK 62500 Hz), which the manual leaves to the lot number; the LSI as RTCCLK at 38.81 kHz (36.96 on the CH32V303VCT6), the rate the watchdog implies; the read synchronization back in 14 us; the alarm arriving as the counter leaves the value it was armed at and reaching EXTI line 17 with the block's own interrupt enable clear; the overflow flag standing before the count the bus reads has wrapped, and that count stepping back by up to three ticks when it is read faster than it ticks; the ten data registers, the crystal, the clock select and the counter all surviving a system reset while the two bus clocks and DBP come up shut; BDRST wiping the domain while the block's own reset line moves nothing in it; the tamper input unreachable from the pad's own side, TPE taking the pin whole; on the CH32V303VCT6 the same letters again with the second block of backup registers, BKP_DATAR11..42, under two patterns, the HSE division one number (128) where the CH32V203C8's is a pair by lot, and the alarm's handler reading the armed count itself (the event raised as the counter leaves it on both parts, this part's bus reading the old count) |
| [crc.md](crc.md) | The cyclic redundancy check unit: three registers, no options, and the Ethernet polynomial wired in, so what the block computes is exactly one function, CRC-32/MPEG-2, whose constexpr twin lives in util/crc.hpp because more than one target carries this block and one function keeps one home. The data register is the operation (a write feeds a word, a read returns the running result, the computation stalling the bus instead of raising a flag), a word is the only grain so a byte stream is the caller's to pack and a length that is not whole words is a compile error, the reset bit is the only control, and the eight-bit scratch register is the one piece of state the reset spares. No interrupt, no DMA row, no per-part fact: the same block on both series. Measured on the CH32V203C8T6: eight lengths and a packed byte stream equal to util/crc.hpp's twin to the bit; the reset landing before the next instruction can look, with a word stored behind it taken and not swallowed (where the same block on an STM32F4 needed cycles for both); the scratch taking a byte store, surviving RST and every fed word, and cleared by nothing short of a system reset, this block having no line in RCC_AHBRSTR; a thousand words costing 12578 core cycles at 96 MHz against 81636 for the twin; a block behind a shut clock gate answering a read with the last word the bus carried instead of the register asked for; and the same fourteen verdicts on the CH32V303VCT6 |
| [rng.md](rng.md) | The random number generator of the CH32V303RC and VC, the only parts of the stratum that carry one: an analog seed behind an LFSR, three registers and one vector, two monitors whose errors are not the same kind (a clock error stops the words and leaves the last one good, a seed error spoils the word and wants 29.2.2's sequence), the two latched flags cleared by a zero because a one sets neither, no reset line, and FIPS PUB 140-2's first-word discard and continuous comparison inside `read()`. Measured on the CH32V303VCT6: SYSCLK runs the block and not the PLL48CLK the chapter names (the word time the same number of core cycles at 144, 96 and 48 MHz, unmoved by USBPRE, and the words coming on the bare HSI), so no clock is refused; the first word after an enable zero every time and a stale word standing across a disable, so a restart discards two; DRDY rising over an unchanged word; and the words not to be taken as random bits (900 to 3350 distinct values in 4096 at every rate and pace, a chi-square near 49000 on 255 degrees of freedom over their bytes, the poker and runs tests failing, neither monitor firing), which the driver states and does not condition, and which the suite asserts so that a die whose words pass turns it red |
| [sleep.md](sleep.md) | PWR and the two sleep sites: three low-power modes behind one pair of bits (SLEEPDEEP, which is the core's, and PDDS, which is this block's, written and read as one pair), a Stop that is one mode with two prices (the regulator's low-power setting and the RAM's low-voltage one, whose interlock the chapter states and the driver enforces) and a Standby that is off the ladder because every way out of it is a reset; the supply monitor whose eight thresholds are worth two different tables of millivolts by a bit of the die and not of the part number, the wake-up pad PA0 with the one flag it shares with the RTC alarm, what a Standby keeps of the RAM (the class's two readings of one bit, a number that is the class's largest array and not every part's, and on the CH32V303VCT6 bits that read back zero and still act, so the driver keeps the program's choice and every store of the register carries it), the regulator trims that live in the extended register, and the debug module's three low-power bits read and never written because a store to that CSR resets the part. Then the mapping onto [the power model](../design/power.md) (light to Sleep, standby and deep to a Stop on each regulator, the clock tree put back after one and the tick paused across it by the idle path) and the count of active bus masters, this family's answer to a sleep that starves every master but the core: a DMA channel of either controller and an attached USB controller each hold one, the kernel's idle path sleeps only at zero and a site refuses to arm above it, so "a program with USB never idles" becomes a mechanism. Plus the timed site, the RTC's alarm as the wake and its counter as the witness of the frozen span. Measured on both parts: an idle() that returns at once over a working master in thirty to forty core cycles, a 65535-item block running on across eight of them (on the CH32V303VCT6 a channel of DMA2 counted and refused as well), and the CDC console enumerating and carrying bytes over each part's own controller with the ordinary kernel loop, where a bare WFI loses every packet on the CH32V203C8T6 and sends zeros on the CH32V303VCT6; a 200 ms Stop woken by the alarm with sixty microseconds of wake (the block raising its event one count of the ruler late, awake as well on the CH32V203C8T6, in the Stop alone on the CH32V303VCT6, measured and subtracted), 21 to 30 us of RTC resynchronization owed at every wake, the tick advancing by none of the sleep and the tree back at the PLL rate within two milliseconds; 30 us (the CH32V203C8T6) and 61 us (the CH32V303VCT6) more at the wake for the low-power regulator, over sixteen Stops of each; a 500 ms deadline met at 502 and 501 ms of wall and of kernel tick through the timed site under a real kernel; the vote round, the standing lock, the deadline guard and the refusal that is the silicon's over a running channel; a pad's line ending a Stop from outside on the CH32V203C8T6, and on the CH32V303VCT6 a wire's edge ending a Sleep in 501 us with its handler 30 timer cycles behind the edge, a Stop at every rate of a DynamicClock with the rate in force put back (on the CH32V203C8T6 too), and all thirty-two f-registers and fcsr through a Stop bit for bit; three Standbys all ending in a reset on each part (the alarm's line as an interrupt, the same as an event alone, and on the CH32V203C8T6 a pad's line as an event, which 2.3.4 does not list as an exit at all) with SBF at every boot, WUF at the alarm's alone, PORRSTF and never LPWRRSTF, and the .noinit token surviving; what a Standby keeps (on the CH32V203C8T6 its one bank whole under R2KSTY and nothing without it; on the CH32V303VCT6 each retention bit its own bank whole and nothing above 32 KB; nothing at all with the bits written by separate read-modify-write stores); and the independent watchdog frozen through a Stop on both parts, its reset arriving 215, 208 and 209 ms after two-second sleeps it never interrupted, and ending a Standby on both, 225 ms (the CH32V303VCT6) and 228 to 230 ms (the CH32V203C8T6) after the alarm meant to end it was placed, IWDGRSTF beside PORRSTF |
| [usb.md](usb.md) | The CH32V203's USB device controller (the CH32V303 has none, its one full-speed controller being [usbfs.md](usbfs.md)'s): ST's F1/F0 device peripheral under WCH's names, realizing util/usb's UsbController at the packet: eight endpoint registers whose status and toggle fields are written by XOR while the two completion flags in the same word clear on a zero, a packet memory of 512 bytes seen through a 32-bit window (one halfword every four bytes of address space) whose top 128 the CAN's filter table takes, so the budget is a template parameter and a CDC port spends 328 of the 384 that are left; COUNTn_RX both the size the program writes and the count the hardware writes back over it, which is why arming a reception rewrites it, why the bus reset guards against one already standing, and why an overflow is a repair and not only a count (the lost packet leaving the size field zero and no completion behind it); the host's suspend answered with FSUSP, because a module left running never hears the wake-up; the pads that are still port A's GPIO pads with the pull-up in another chapter's register; one of the two vectors used and the other left to the double buffer this driver does not offer; and two compile-time refusals, the 48 MHz the tree's USBPRE must make and the measured floor of the bus under it. Measured: an enumeration to CONFIGURED in 362 to 628 ms with 328 of 384 bytes spent, ten frames in 10 ms and no overflow; a thousand idle() calls costing 0 to 1 ms over the bus-master count an attachment holds; a reconnect answered with three bus resets and the same address; 2.87 million bytes echoed byte-exact in ten seconds, 767 KB/s out of the bulk pair and 461 KB/s into it with no ring overrun; the line coding read back as 19200 7E2 with DTR and RTS raised; a host's suspend and its resume both counted, where a module left running reported the suspend alone; staged with a bare wfi under a full-speed pour, 616 packet-memory overflows in 50 ms that carried nothing where the awake window carried 22 kB (the endpoint repaired and taking 95 kB in the 200 ms after); the PLL at 48 MHz with USBPRE /1 enumerating; two CDC ports in the whole 512 bytes, the second's buffers above the 384 line, both echoing; and a host's halt and its clear on the bulk pair, which found the toggles surviving a bus reset and a clear releasing to NAK, both fixed in the driver (a configuration and a clear start at DATA0, and a clear gives back the armed status) |
| [usbfs.md](usbfs.md) | The USB host/device controller of ch. 23 in device mode, the CH32V303's one full-speed controller and five CH32V203 parts' second, on the pads the part table names (PA11/PA12, PB6/PB7), realizing util/usb's UsbController at the packet: WCH's own design and nothing of ST's, a register file of bytes with eight endpoint numbers whose mode nibbles pack two to a byte, a buffer address, a transmit length and two response bytes each, and the endpoint buffers in the program's own RAM, reached by DMA out of a pool the driver owns and hands out in claim order; one status byte and one receive-length register for every endpoint, kept standing by the automatic pause the driver sets; a SETUP taken as endpoint zero's whatever the status byte's endpoint field says; a response the silicon never changes, so every completion is answered NAK before its flag is cleared; the data toggle kept in the endpoint's register and flipped by the driver, a repeated OUT dropped; the pull-up that is also the device enable, and every event of a detached block dropped; the wake-up line of each class; and the 48 MHz and a measured floor of HCLK refused at compile time. Measured on the CH32V303VCT6 on the board's own connector: the pair PA11/PA12, reached with port A's clock shut; an enumeration to CONFIGURED in 533 to 565 ms with 256 of the 960 pool bytes spent; the status byte naming endpoint 6 at a SETUP (a driver that believed it dropped every SETUP of an enumeration) and RX_LEN reading 1022 there, and the buffer register reading back the address's low bits; no bus reset reported over the SE0 a detach leaves once the driver drops a detached block's events, and a reconnect in 362 to 570 ms with the same address; the floor at 12 MHz of HCLK (96 to 12 carrying an enumeration and a pour whole, 6 writing about 3 % of a pour's packets wrong with no flag and none missing whole, as a pour of numbered packets tells, 1.5 carrying neither); the host's frames counted one a millisecond through the interrupt bit the manual calls reserved; three million bytes echoed byte-exact at 339 kB/s, 768 kB/s out and up to 609 kB/s in; a staged toggle mismatch costing exactly one packet; the pause holding the status byte through a held vector where without it 248 to 297 packets overran it in 20 ms; with the core in a bare wfi the OUT packets landing whole while the IN packets carried zeros and an enumeration stopped at its descriptors, which is why the controller holds a bus-master count from its pull-up to its detach; a host's resume raising EXTI line 18 once and line 20 never; the PLL at 48 MHz with USBPRE /1 enumerating; a host's halt and its clear on the bulk pair, the clear now giving back the armed response at DATA0 (a release to NAK lost the class's reception); a number's halves moved by the claim order, a move over an armed half refused; and a device on Usbfs<256>, filled to the byte, echoing |
| [vendor/README.md](vendor/README.md) | The documents of record with their revisions: the reference manual that covers four families and the class rule that divides them, both datasheets' resource and pin tables, the QingKe V4 manual for the V4B and the V4F, the probe's, and the two EVT packages as the vendor's only voice on quirks; the no-errata statement; the candidate revisions waiting to be promoted, and what the reference manual's V2.5 changes, chapter by chapter, beside what the silicon said where it was measured |
