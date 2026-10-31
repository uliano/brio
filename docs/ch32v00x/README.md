# Target: CH32V00x (`ch32v00x/`)

The operational page of brio's CH32V00x target, for a programmer who
has the CH32V006 module or the CH32V003 evaluation board on the desk
and wants to build, flash and debug a brio program on it, and then find
the document of the peripheral the program needs. It says what the
stratum is, which boards and parts it is proven on, how the toolchain,
the build, the upload, the debugger and the console work and where each
has a trap, what the silicon taught the stratum, what is not covered
yet, and it ends with the map of the stratum's own documents: one line
per peripheral driver, which is where the state of the driver work is
read.

A stratum is a directory of `brio/` holding one architectural layer;
`brio/ch32v00x/` holds everything that knows WCH's CH32V00x register
map. The core is WCH's QingKe V2C (the V2A on the CH32V003), a RISC-V
core with the RV32EC instruction set: sixteen registers, compressed
instructions, a multiply and no divide (no multiply on the V2A). The
bench part is the CH32V006K8U6 (62 KB of flash, 8 KB of SRAM, 48 MHz),
and the second and last part of the stratum is the CH32V003F4P6 (16 KB,
2 KB), the smallest silicon brio runs on and its most extreme point.
The kernel (`kernel/`) and the services (`util/`) run on both
unchanged, which is the fact this target exists to keep true. The
maintained example `blink` builds for both boards.

Two things shape the stratum and are stated up front. **There is no
vendor header in the build**: WCH ships its register definitions inside
the EVT package, whose licence is written for software running on WCH
parts, so [brio/ch32v00x/device.hpp](../../brio/ch32v00x/device.hpp)
carries the map, read off the reference manual and answerable to it,
and per-part variability, which the other 32-bit strata ask the device
header for, is stated in the part's own table under
`brio/ch32v00x/parts/`. And **the core is not the RISC-V of the
privileged specification in two places that matter to a kernel**: a WFI
that only wakes for an interrupt it can take, and an MIE that is not
cleared on interrupt entry, both of which the platform header answers
("What the silicon taught the stratum" below).

The documents of record and their revisions are in
[vendor/README.md](vendor/README.md).

## The boards

Two boards, one per part, each with a page of its own under
[../boards/](../boards/README.md) with its pin numbers, its jumpers and
its probe.

| Board | Part | Memory | Clock | Console | LED |
|---|---|---|---|---|---|
| [a CH32V006K8U6 module](../boards/ch32v006k8.md), the bench board | CH32V006K8U6, QFN32, the QingKe V2C | 62 KB flash, 8 KB SRAM | HSI 24 MHz, the PLL at 48 MHz; a 24 MHz crystal on PA1/PA2 | USART1 PD5/PD6 on the probe's own serial, 115200 | PC0, jumpered |
| [WCH's CH32V003F4P6 evaluation board](../boards/ch32v003f4.md) | CH32V003F4P6, TSSOP20, the QingKe V2A, no multiplier | 16 KB flash, 2 KB SRAM | the same, a 24 MHz crystal soldered on PA1/PA2 | the same | two on a row of pins, jumpered |

Both run at 3.3 V. The chip wakes up at 8 MHz on both parts (HSI 24 MHz
with HPRE at its reset value of /3); `Clock<ClockSource::pll,
48'000'000>` takes it to the top of the range (the PLL only doubles;
HPRE divides), setting the flash wait states first from the part's own
table: 0 to 15 MHz, 1 to 24, 2 to 48 on the CH32V006; 0 to 24 and 1 to
48 on the CH32V003. There is no APB prescaler on this family, so
`pclk_hz` is `hz`.

## Toolchain

The compiler is WCH's `riscv32-wch-elf` gcc 15.2.0 at `/sw/wch-riscv`
(a symlink to `wch-riscv-15.2.0`, extracted from the MounRiver toolchain
package), named by absolute path from
[ch32v00x/cmake/toolchain-riscv.cmake](../../ch32v00x/cmake/toolchain-riscv.cmake).
It is the one compiler in this repository that is not self-built and
not at the project's usual version. What it brings that an upstream gcc
does not: a multilib set for rv32e (`rv32ec`, `rv32ec_zmmul` and their
`_xw` twins, each with newlib in full and nano form), which an upstream
gcc has to be built with.

**The architecture string is the part's full ISA under WCH's gcc.**
`misa` on the bench part reads `0x40801014`: E, C, M and one
non-standard extension. The M is the multiply-only "m" of the
datasheet's RV32EmC; the non-standard extension is WCH's `xw` compressed
set, which only their compiler emits. The project compiles with
`-march=rv32ec_zmmul_xw -mabi=ilp32e` for the CH32V006 and
`-march=rv32ec_xw` for the CH32V003, whose V2A core has no multiplier
(derived from the part at every configure; the `CH32V00X_ARCH` cache
entry is an override, empty by default, that a configure announces when
it is set). The reason for `xw` is the family's smallest part: `xw` is
worth one to two per cent of every image, 72 to 516 bytes over the
suites, measured, and on a 16 KB CH32V003 those bytes are the
difference between a group of letters that links and one that does not
(the rule: [../design/overview.md](../design/overview.md), "A suite's
image fits the family's smallest chip"). The consequence is accepted:
this family is built by the vendor compiler, and an upstream gcc, which
has no `xw`, is not a drop-in for the toolchain file. The hardware
prologue (below) is the other saving the compiler offers, 8 to 220
bytes an image on top of its nine cycles.

**Interrupt handlers are bound with one macro, `BRIO_CH32_VECTOR(name)
{ body }`**, and which handler attribute it writes belongs to the image: with the project's
`CH32V00X_HPE` option (on by default) the crt sets INTSYSCR.HWSTKEN and
the handlers are declared with WCH's `WCH-Interrupt-fast`, the core
pushing and popping the caller-saved registers itself; with it off, a
handler is a plain `[[gnu::interrupt]]` function. One option for the
whole image, because a fast handler under an HPE that is off corrupts
the program it interrupted. Both spellings carry `no_icf`: gcc's
identical code folding otherwise turns the second of two handlers of
one body into a handler that calls the first, which ends in MRET, so
the call never returns and the caller's frame stays on the stack: empty
under the hardware prologue, where the merge was harmless by luck, and
the caller-saved registers under a plain `interrupt` (measured on the
CH32V303VCT6, [../ch32vx03/platform.md](../ch32vx03/platform.md)).
Interrupt nesting is never turned on. What the hardware prologue is
worth is measured: six cycles on a minimal round trip, 21 of 144 on
the console's USART handler ([platform.md](platform.md)); the price is
the vendor attribute, which WCH's gcc has and an upstream gcc gets from
the fast-interrupt patch.

## Build

`ch32v00x/` is its own CMake project, a sibling and peer of the other
cross projects and of `test/` (its own toolchain file, its own presets,
one configure = one compiler); the repository root is not a CMake
project, so every CMake command runs from `ch32v00x/`:

```bash
(cd ch32v00x && cmake --preset ch32v006k8-release)                 # configure (once, or after adding an app)
(cd ch32v00x && cmake --build --preset ch32v006k8-release --target <app>)
(cd ch32v00x && cmake --build --preset ch32v006k8-release --target <app>-upload)
(cd ch32v00x && cmake --preset ch32v003f4-release)                 # the second part
brio flash <board> <app>        # the bench way: board type v006k8 or v003f4 in the manifest
brio console <board>            # its console: the probe's own serial port, 115200
brio check ch32v00x [name]      # family smoke, no hardware
```

One configure targets one part. The cache variable `CH32V00X_MCU` names
it (`ch32v006k8` or `ch32v003f4`), and from it the project derives the
linker script ([ch32v00x/ld/](../../ch32v00x/ld/)), the part definition
the stratum asks (`CH32V006` or `CH32V003`: with no vendor header the
build states the part, and `device.hpp` includes the part's own table
from `brio/ch32v00x/parts/`) and, unless `CH32V00X_ARCH` says
otherwise, the ISA: `rv32ec_zmmul_xw` for the CH32V006's V2C core,
`rv32ec_xw` for the CH32V003's V2A. Four presets name the two parts
twice each: `ch32v006k8-release`, `ch32v006k8-debug`,
`ch32v003f4-release` and `ch32v003f4-debug` (CMake Tools shows them as
`CH32V006K8 (release, -Os)` and `CH32V006K8 (debug, -Og)`).

The CH32V006's script gives the linker the first 40 KB of the 62, the
top 22 KB being the storage partition ([nvm.md](nvm.md)); the
CH32V003's gives it 15 KB of the 16, the top kilobyte the journal's
attic; `__brio_rom_end` is the boundary the media read back. The crt is
[ch32v00x/src/glue/startup_ch32v00x.S](../../ch32v00x/src/glue/startup_ch32v00x.S),
compiled into every image, its table two entries shorter on the
CH32V003 (no USART2, no OPCM); after `.data` and `.bss` it paints the
free RAM up to the stack top, the ledger `stack_untouched()` reads
([platform.md](platform.md)). Both scripts place `.fast_text` and
`.ram_text` first in `.data`, copied to SRAM with it: code a program
opts into running from SRAM for speed, and code that must run from
SRAM (the CH32V003's sleep entry); [platform.md](platform.md), "A
handler in SRAM".

Applications are discovered at every configure from
`ch32v00x/src/apps/*.cpp` by the `// build:` lines in each file's
header comment, the same grammar as the other projects; the board types
an app's `boards =` line names are `v006k8` (the default) and `v003f4`.
Every chapter is tiered for both parts; an app names the second when
its image fits there (the group axis below) and its wires exist on that
board; the serial suite stays on the CH32V006, whose USART2 is its
instrument. The board's type in the bench manifest is what tells
`bin/brio` to build here and to flash through WCH's OpenOCD fork
([../boards/README.md](../boards/README.md)).

**The CH32V003 splits a suite into group images.** Its 15 KB of program
flash and 2 KB of RAM do not hold every suite whole, so a suite
declares its groups of letters (`// build: groups = abcd,efg`) and the
`ch32v003f4` presets build it as one image per group,
`test_ch32_tim-1`, `test_ch32_tim-2` and so on, each compiled with
`BRIO_TEST_LETTERS` naming its group and registering those letters
alone; the menu of such an image says which letters it carries and that
the others are in its sibling images. The `ch32v006k8` presets build the
same source as one image with every letter, unless the suite names the
CH32V006's own groups (`// build: groups.v006k8 = rtmpdes,uw`) because
its whole image comes within a few per cent of that part's 40 KB or
8 KB, which the benchmark skeleton does. `brio flash <board>
test_ch32_tim-2` names an image, and the bare name is refused there
with the list. The rule and its reasons are
[../design/overview.md](../design/overview.md), "A suite's image fits
the family's smallest chip"; the mechanism is `util/testbench.hpp` and
`ch32v00x/CMakeLists.txt`'s discovery.

**The image starts at address 0 and its first word is an instruction.**
The core fetches address 0 as code, so the vector table's entry 0 is
`j reset_handler` and every entry after it is a handler address (mtvec
is set with both mode bits: absolute addresses, vectored by interrupt
number). The interrupt number is the table's word index, the STK at 12
and USART1 at 32, which is why `device.hpp`'s `Irq` enum and the crt's
table are the same list. The handler names are the project's own
(`systick_handler`, `usart1_handler` and so on): with no vendor header
there is no vendor spelling to honour, and an app binds a vector by
defining the strong symbol.

## The probe and the upload (WCH-Link, WCH's OpenOCD)

A WCH-Link in RISC-V mode (USB `1a86:8010`), driven by WCH's OpenOCD
fork at `/sw/wch-openocd`, the only OpenOCD that speaks the probe's SDI
transport and a different program from the `/sw/openocd` the SWD paths
use. Its target script `wch-riscv.cfg` sits beside the binary, not in a
scripts tree. The probe's own page,
[../probes/wch-link.md](../probes/wch-link.md), carries the traps met on
the way in: an old probe firmware refusing the part in words that read
like a wiring fault, a vendor binary linked against a library the
distribution has not got, and the bridges' habits.

The wires: PD1 = SWDIO (the 1-wire SDI, the chip's default debug
mode), GND, 3.3 V; PB3 = SWCLK for the 2-wire mode this family also
has. The debug interface is on after every reset (AFIO_PCFR1.SWCFG
resets to 0) and a program can turn it off, which is why WCH's serial
ISP on USART1 (PD5/PD6), the same pins the console rides on this bench,
stays the escape route.

`brio flash <board> <app>` and the project's `<app>-upload` target run
the same programming: `openocd -f wch-riscv.cfg -c "program <app>.elf
verify"`, then `exit`; between the two, the `-upload` target runs
`reset run`, which starts the program on this family, and `brio flash`
runs `reset halt` and `resume`, the sequence it uses on every
WCH-Link because `reset run` leaves a CH32V203 at its reset vector
(`cli/bench/flash.py`). The fork's flash
driver reads the part's device id and flash size itself (`device id =
0x5dd3abcd`, `flash size = 62kbytes` on the bench part); nothing names
the part. The chip is left running, the end state of every other path.
No debug-enable is taken back down afterwards: `break_here()` on this
core is an `ebreak`, which with no debugger attached escalates to the
fault vector instead of halting the core, so a just-flashed board and a
probe-less power-on behave the same way.

## Debugging

The fork starts a gdb server on port 3333 (`Info : starting gdb server
for wch_riscv.cpu.0 on 3333`). The toolchain carries a
`riscv32-wch-elf-gdb` that links libpython3.8 and does not start on a
current distribution; the debugger is a stock gdb built from source for
the same target under `/sw/riscv-gdb` (WCH's gcc stays, only the
debugger is replaced). What the bring-up used, and is enough for one,
is OpenOCD's own console: `halt`, `reg pc`, `reg mstatus`, `reg
mcause`, `mdw`, `step`, `resume`. `.vscode/launch.json` carries a
cppdbg entry over the same server, "Debug CH32V006K8 (WCH OpenOCD,
WCH-Link)", the AVR entry's shape: the fork launched by the editor, gdb
connecting by hand in `setupCommands`, `load` through the fork's flash
driver, a stop on `main()`. It is configured, not verified: written
from the console dialogue and not yet driven from the editor. Two facts
about the core in a session: **a core in debug mode cannot enter any
sleep** (QingKe V2 manual 5.1), so the idle path's behaviour is not
observable with the probe halted on it; and **the debugger's `step`
does not take pending interrupts**, so a single-step through an unmask
instruction proves nothing about whether an interrupt would be taken.

## Editor (clangd)

`brio/ch32v00x/.clangd` and `ch32v00x/.clangd` route both trees at the
project's own compilation database (`build-cmake/ch32v006k8-release`),
so a header of this stratum parses with the RISC-V compiler's real
command line whatever project CMake Tools has active.

## Serial console

USART1 on PD5 (TX) and PD6 (RX), the default alternate function, wired
to the WCH-Link's TX/RX pins, so the probe's CDC port
(`/dev/serial/by-id/usb-wch.cn_WCH-Link_<serial>-if01`, a real USB
serial) is the console, at 115200. The line assembler completes a line
on `\n`; a `\r` alone is ignored (`util/proto/line_parser.hpp`), which
is what a terminal sending only carriage returns runs into.

## Family coverage

`brio check ch32v00x` compiles every smoke TU under
`test/family_ch32v00x/` for both parts with each part's own flags, its
definition and its ISA, the two facts the build derives from the part
number, and demands that every `neg/*.cpp` be refused on the parts its
`// mcu:` line names (`cli/checks/check_ch32v00x.sh`; no CMake, no
hardware, seconds). A positive TU may carry a `// mcu:` line too: the
fixtures of the chapters still closed on the CH32V003 compile for the
CH32V006 alone, and the ones that pin a part's values (the GPIO nibble,
the wait-state table, the flash geometry) pin each part's under its
definition. What the sweep also proves is the toolchain: `util_all.cpp`
includes every kernel and util header and instantiates each service
over this platform, so a construct WCH's gcc 15.2 rejected would show
here first; one did (a loop-analysis false positive in
`util/nv_heap.hpp`'s mount, answered by a bound the compiler can see,
byte-identical on the other targets). The check ends with the link
guard of every app on every release preset.

## What the silicon taught the stratum

Each of these is enforced or answered in the code it names; they are
here because the manual states them quietly and the bench found them
loudly.

- **WFI wakes only for an interrupt the core can take** (QingKe V2
  manual 5.2: the sleep is ended by "the interrupt source responded by
  the interrupt controller"). A `wfi` executed with mstatus.MIE clear
  sleeps past every pending interrupt for ever: the tick and the USART
  both pending, the core asleep, the console silent. The "sleep first,
  unmask after" idle of the two Cortex-M0+ targets is therefore a
  deadlock here, and `Ch32v00xPlatform::idle()` sleeps as a WFE instead,
  unmasked: PFIC_SCTLR.WFITOWFE turns `wfi` into a wait-for-event and
  SEVONPEND makes every interrupt entering the pending state a latched
  event, so an interrupt taken between the unmask and the sleep makes
  the `wfi` return at once.
- **A WFE with MIE clear loses an edge on the cycle it goes to sleep.**
  With interrupts masked the only wake left is SEVONPEND's "new
  interrupt pending signal" (5.2, WFE item 3), an edge; one that arrives
  on the very cycle the WFE enters its sleep is neither latched for it
  nor seen by the sleeping core, the pending bit stands, and no new edge
  comes: measured to the cycle, one position in every sleep entry, WCH's
  doubled-`wfi` sequence included. A kernel that slept masked hung on
  it, the tick pending and enabled; an enabled interrupt with MIE set
  wakes the WFE by its level (item 2), which is why the unmask comes
  before the sleep ([platform.md](platform.md)), and why the timed sleep
  site enables the AWU's line in the PFIC, its wake a level and its
  vector one the program binds ([sleep.md](sleep.md)).
- **The waking edge is latched too**, so the next WFE would return at
  once and a kernel loop turn twice per interrupt: `idle()` consumes the
  latch after the wake (SETEVENT and one more `wfi`), 101 turns over
  100 ticks for 6 to 15 cycles a wake.
- **The CH32V003's sleep entry wedges the core**: an edge within a
  four-cycle window at the QingKe V2A's `wfi` leaves it in a state no
  interrupt, event or debugger halt ends, only a reset, in every entry
  shape tried but the vendor's own (the `wfi` masked and run from SRAM,
  a spin there after it), which that part's `idle()` therefore uses
  ([platform.md](platform.md)); the V2A's Sleep also lasts at least
  66 us at 48 MHz and stops the DMA controller, so that `idle()` does
  not sleep while a channel works ([dma.md](dma.md)).
- **MIE is not cleared on interrupt entry** (QingKe V2 manual 2.2, for
  the sake of its nesting mode). With nesting disabled in INTSYSCR, as
  this crt leaves it, a handler still runs with MIE set and the
  controller holds the next line until MRET; the guard inside a handler
  behaves as anywhere else.
- **The STK flag must be cleared by the handler** (RM 6.5.4.2, CNTIF is
  "write 0 to clear"): unlike an ARM SysTick exception, entry does not
  clear it, and a handler that returns with it standing is re-entered
  for ever. `BasicTicker::tick()` clears it first.
- **The GPIO nibble looks like the STM32F1's and is not** (RM 7.3.1.1):
  MODE is one bit here (output at the port's only speed, or input), the
  second bit of the field reserved. An F1 nibble such as 0xB (AF
  push-pull "50 MHz") lands as AF push-pull with a reserved bit set,
  right by accident. `pin.hpp` spells CNF and MODE apart.
- **A namespace-scope reference to a register is dynamic
  initialization.** `auto& reg = *reinterpret_cast<...>(addr);` at
  namespace scope goes to `.init_array` and reads as zero in anything
  that runs before the constructors; on the first image of this
  bring-up every register access went through a null pointer. The
  stratum reaches registers through inline functions, and the crt walks
  `.init_array` anyway.
- **The PFIC's status banks are named backwards**: the manual's ISR is
  the enable status and its IPR the pending status; the enable
  registers themselves (IENR/IRER) are write-only and read as zero.
  `Pfic::enabled()` and `Pfic::pending()` read the right bank.
- **PINRSTF names the pin alone.** On the STM32 register this one
  descends from the pin flag is raised beside every system reset; here
  a software reset boots to SFTRSTF and nothing else (measured). The
  flags mean what they say, and `reset.hpp` says so.
- **The hardware prologue is worth six cycles on a minimal handler**
  (78 against 84 for the whole round trip), not the order of magnitude
  a "fast interrupt" suggests: the ten registers reach the stack either
  way, and what the hardware saves is the prologue's fetch. The numbers
  and the reasoning are in [platform.md](platform.md).
- **A DMA request that is never cleared is served once**: TIM3's
  channel 3 and 4 matches move one transfer after a reset of the block
  and never another, whatever is rewritten or toggled
  ([tim.md](tim.md)); a DMA read or write of a hole in the map, or a
  write into the flash, completes as a normal block and no TEIF comes
  ([dma.md](dma.md)).
- **The ADC can stop converting under a hardware trigger served by the
  DMA** while the CPU works the peripheral buses, STRT standing and no
  EOC, and only an ADON cycle revives it ([adc.md](adc.md),
  `Adc::recover()`); the cause is not found.
- **A START into a busy bus is answered, not parked**: against a peer
  holding SDA low the I2C's START comes back ARLO at once, where the
  other three strata's peripherals park it until the bus frees (a
  tenure another controller is clocking is waited out); and no CTLR1
  write may happen while the host's own STOP stands (the F1 lineage's
  rule), so this engine's `start()` is the one that waits, bounded, for
  that STOP to leave ([i2c.md](i2c.md)).
- **A DMA request that rises while its channel is disabled is latched**
  and served at the channel's next enable: the SPI's RXNE under a
  standing RXDMAEN put a stale frame at the head of every engined block
  after the first. The host arms the requests around the engines and
  nowhere else.
- **USART2's gate is PB2's** (RCC_PB2PCENR bit 13, and its reset
  RCC_PB2PRSTR bit 13) while its registers answer in the PB1 address
  space; on the F1's bit of PB1 the block reads zero everywhere. The bus
  a block answers on and the register that clocks it are two facts.
- **Half duplex is a bus, not a loop**: HDSEL puts the receiver on the
  TX pad through the AF mux, and a frame driven onto that wire is heard,
  but not the instance's own frames, in any pad configuration. The
  single-wire loop-back the STM32G0's serial suite is built on does not
  exist here; the CH32 suite bit-bangs the RX pad and captures TX on a
  timer instead.
- **The chapters promise protections the silicon does not keep**: no
  SPI control field is enable-protected, the I2C's timing registers take
  a write under PE, the WWDG's counter does not run until armed and its
  BIF cannot be cleared while the input stands. Each is a refusal in the
  driver where the chapter's rule matters.

## Not covered yet

Driver gaps, each with its reason:

- **The CH32V003 tier's last pieces.** The part's table
  (`brio/ch32v00x/parts/ch32v003.hpp`) tiers every chapter: the core,
  the clock (the HSE included), the pads and every remap column, EXTI,
  USART1, the DMA, the timers (no TIM3, no dead time on TIM2), SPI, I2C
  (the same block), the ADC (10 bits, the calibration, Vcal, DLYR, one
  watchdog), the OPA (three bits of EXTEND_CTR), PWR (its PVD table, no
  LDO modes), the flash constants and the reset/watchdog block; each
  document says where the part differs, and every suite but the serial
  and the SPI ones runs green on the board, the larger ones as group
  images, the I2C one on the wire against a peer; the SPI suite's
  fifteen group images link and wait for a CH32V003 on the CH32X035's
  wires ([spi.md](spi.md)). The USART's synchronous mode, which the
  CH32V003 has and the CH32V006 has not, is measured there with no wire
  ([usart.md](usart.md)); what stays declared, each with its reason: the
  smartcard (no card on the desk), the I2C's kernel and two-host
  letters, each image alone over the part's 15 KB and carried by no
  group of that build (the CH32V006's runs of them stand for the block
  both parts share, [i2c.md](i2c.md)), and the serial suite's other
  letters, whose instrument is USART2.
- The part's SVD for a register viewer, and the editor debug entry
  driven for real: the entry is written, the SVD is not fetched (the
  MounRiver package may carry one), and neither was needed to bring the
  target up.
- The DMA burst of the timers (DMACFGR/DMAADR), the front-end polling
  of the OPA, TouchKey, the DBG freezes of the timers, EXTEN's lock-up
  monitor: each a mode with no user, each named in its chapter's
  document.

Implemented but not bench-verified, each with what would measure it:

- The CH32V003's regrouped images: the benchmark skeleton's letter s
  (the SPI host's engines, moved out of letter d so that the part's
  image of letter d fits it) in an image of its own, the part's new
  groups of the I2C suite, and the DMA suite's two images (its letters h
  and i, unequal widths and the level against the channel number, and
  letter d's writes, measured on the CH32V006K8U6 alone). `brio check
  ch32v00x` links each; one run of each image on that board, off the
  desk, measures it (the CH32V006's two benchmark images ran whole).
- The idle path's power, and Standby's: both are proven to sleep and
  wake (`test_ch32_platform` letter b, `test_ch32_sleep` letter d) but
  not to sleep cheaply; a current measurement on a bench meter, with the
  probe detached (a core in debug mode never sleeps), would.
- `Pin` open-drain outputs and the levels a pad drives: written from
  the chapter and measured only through their pulls; the pad suite's
  jumper letters would.

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
| [clock.md](clock.md) | RCC: the two roots (the HSI, the doubling PLL), the one divider, `Clock` and the `DynamicClock` that walks the HPRE ladder with the users rebased and the wait states following (measured: 48 to 3 MHz and back, the console clean at every rung; the crystal as SYSCLK itself in an image of its own; the crystal's roots within some 40 ppm of a host's clock and the HSI's PLL 0.5 per cent slow), `Rcc` (the HSI trim, the LSI, ready in 17 us, the MCO, the system clock monitor, the peripheral gates) |
| [sleep.md](sleep.md) | PWR: Sleep and Standby through the site contract (light is Sleep, standby and deep both Standby), the AWU as the alarm of a timed site with the LSI measured against the STK (124 and 125 kHz on the two parts) and the frozen span handed back less the ticks counted awake, each part's own PVD ladder (two bits to 2.66 V, three to 4.4 V) and the LDO modes the CH32V006 alone has; measured on both: a 300 ms Standby ended by the AWU, the core waking on the HSI and the PLL restored, the event due and never early in kernel time, the PVD bracketing the board's supply; on the CH32V006 the /10240 and /61440 codes' windows (31 s) as their names divide, a Standby, the probe attached, 11 to 15 per cent shorter on a host's clock than the span the site hands the kernel, the AWU's edge swept to the cycle across the sleep entry (the PFIC-disabled line's pending edge losing tries, the enabled line the site arms, a level, none, in a Sleep and in a Standby), and the PWR gate that answers rubbish until opened |
| [nvm.md](nvm.md) | FLASH: the engine (fast page program as the only way to write, the two locks, three erase grains), the constant partition (the linker's 40 KB, the heap's 16 KB, the journal's 6 KB attic) and the two media with the page as the cell; measured on a microsecond stopwatch: a page erase 2.9 ms, a program 1.9 ms and a sector erase 3.1 ms with the core stalled, the kernel's tick counting one across a stall of three periods, a journal save 2 ms and a collecting one 76 to 78 ms, and a page that accepts a second program between erases, the finding a smaller cell could rest on, not yet taken |
| [dma.md](dma.md) | DMA: seven channels where the channel is the request (table 8-2, no multiplexer), the STM32F1's channel with its fields read-only while enabled, one vector a channel and one flag register for all seven; the engines in two moments (the binding at `arm()`, five stores and no load a block, the beat the run's element, the claim alone under a transport's mask) and the copy engine, each armed at its peripheral's level ([../design/dma.md](../design/dma.md): an overrunning receive highest, a transmit next, a copy lowest); measured with the one-shot verbs: three widths exact at about six HCLK cycles an item, the nine unequal pairs as table 8-1 spells them, the level outranking the channel number, EN staying set after a completed block, and no hole in the map, read or written, nor a write into the flash, raising the transfer error the chapter promises; the costs in time, 6.0 and 8.0 cycles an item, a block's fixed cost moving with its image's layout, the SPI host's 846 cycles above the wire split between the bench's instrument and the host's launch |
| [spi.md](spi.md) | SPI: the F1's SPI with no FIFO and one register of its own (HSCR), 8/16-bit frames, the four modes, the select as a GPIO the request carries, `SpiHost` with the other strata's Request verbatim (pump or polled, engines on channels 2 and 3 at both frame widths with one DMA interrupt a transaction, the engines' 0xFF cell in SRAM), `SpiClient` refusing an answering client with CPOL = 0 (16.2.3, measured: a window whose first answer bit is 1 can slip one bit at both ends) and serving a transaction in one call (its answer, exchange and receive runs at both widths); measured by `test_ch32_spi` against the CH32X035's `spi_peer` (two images, 64 verdicts): no field is enable-protected (seven of seven take a write under SPE), the pump in the four modes at both widths, the engines one interrupt a transaction and a polled request at its own frame width after an asynchronous one at the other, the rate on the wire from HCLK/64 to HCLK/2, the hardware CRC at 8 and 16 bits against a foreign CRC unit (a polynomial mismatch caught at both ends), the ladder to the pair's ceiling (12 MHz both ways, 24 MHz for what this host sends: a read at 24 MHz has no timing margin, table 3-19), the kernel over a second chip, the client on PC1's hardware NSS answering in modes 2 and 3, MODF from a second chip, and a DMA request latched while its channel is disabled, served at the next enable, which is why the host arms the requests around the engines; the host above the wire on the CH32V006 (the polled write at the wire at HCLK/4 and /16, the receive loop at 1.69 and 1.13 times it, the pump two in flight at the wire from HCLK/64, a 3-byte request 391 cycles above its wire time, the engines about 520 cycles a transaction against the 355 counted); `spi_peer`, this part as the desk's SPI client on PC1's hardware select (hearing in all four modes and answering with CPOL = 1, so its answer windows go in mode 3); the client's runs measured from the far side (`test_x035_spi` letter k: the exchange to 6 MHz in 8-bit frames and 12 MHz in 16-bit ones from the flash, the answer and receive runs to 12 MHz back to back and 24 MHz behind a gapped host, a rung more from SRAM, a DATAR read at a frame's end losing the next answer and TXE no witness of RXNE); the host's polled loops from SRAM under `InRam` (the transmit-only loop at the wire from HCLK/2, the receive loop's dead bus 11 cycles a frame against 25), the pump inline in the vector under either placement; `spi_stalled` provoked over a block held in reset; the run's caller placed by `[[gnu::noinline, gnu::section(".fast_text")]]` buying its rung, a selectless client exact behind a quiet bus and one bit off behind a stray edge; and the vendor's own loop run beside the host, ahead by a tenth to an eighth at HCLK/16 on its unguarded write-ahead, losing frames at HCLK/4 where the host is exact |
| [i2c.md](i2c.md) | I2C: the F1's event machine on two vectors, the receive procedures by count, no rise-time register on either part (the prose names one, the register lists do not), `I2cHost` with the other strata's Request verbatim and i2c_bus.hpp's outcomes (engines on channels 6 and 7, a write ending on BTF with no DMA interrupt, a one-byte read on the pump), `I2cClient` polled or on its vectors with `flush()`, the unstick; measured on column 3 (PB5, PB6) against the desk's CH32X035: every tenure shape byte-exact, the two speeds, the arbiter with a held SDA answered ARLO by the silicon and a held clock by the per-bus timeout, a STOPF the slave half raises on a STOP it was not addressed in, arbitration both ways against a second controller, a START into its live tenure held until the STOP, the client written to and read from by it, BTF under a requested STOP and the withdrawal's race measured raw; OADDR1's bit 14 reserved, the timing registers writable under PE; `twi_peer`, this part as the desk's I2C instrument, its polled target's habits measured from the CH32X035's host (the byte asked one more, the stretch one byte late, the refusal held until its window closes); the event pump takes no placement (as an SRAM instance behind a vector in the flash it lost the call's cycles), the vector placed by the program taking an entry from 154 cycles to 92; a timer's compare request held on channel 6 or 7 staging a dead channel (a write over it `i2c_dma_fault`, a read over it answered by the caller's bound with no storm) and the block's own request not outliving its gate; the client from its vectors, one entry an event; the peer's counted serve ended on the wire at rest, this block raising no Stop flag after a read |
| [usart.md](usart.md) | USART: the F1's USART minus two personalities on the CH32V006 (no synchronous mode, no smartcard; the reserved bits measured) and whole on the CH32V003 (the synchronous mode's verbs, its clock counted off the CK pad by TIM2 with no wire, the receiver sampling on that clock; the smartcard declined), `Usart<n>` with the whole of chapter 14 as verbs and the transport `Uart` on it with options that cost nothing (byte-identical), USART2's gate on PB2 while its registers answer on PB1; measured: every frame format both ways, a start bit that is the divisor to the cycle at eight rates to 3 Mbaud, the three sampling points of the receiver and its tolerance, the break sent and detected, both mute wakes, the IrDA pulse in both modes and the decoder, CTS and RTS, one interrupt per flag, and half duplex as a bus, not a loop (the receiver hears the TX pad and never its own frames); the receive engine at very_high (`UartOptions::rx_priority`), an overrun with nothing to take cleared on its wait's second unmoved entry (three vector entries and the overrun counted once, where the storm took 256, a DMA fault and the next burst); USART2's released requests held on channels 6 and 7 until its reset line, the receive ring over a channel a timer's held request froze given up by its stall bound after 256 entries and alive again once the timer's reset line falls, a frame's request outliving the CPU's read of DATAR, the CTS flag on both edges and its vector once a toggle, the transport's `rts` and `cts` options run as a port; on a wire to a second chip (column 4, PB0/PB1, the desk's link to a CH32X035) every rate to 3 Mbaud and every format echoed, a TX pad left on the function through a reset or handed back as TE rises showing the far receiver a low pulse; column 4 as the DUT under `test_ch32_serial`'s two-board key (60 verdicts, its own image on the CH32V006K8, the module's crystal the reference): the far CH32X035 calibrated by this crystal, every rate to 3 Mbaud timed by the far receiver within 200 ppm of HCLK over BRR, the odd stops' lengths, PE, the rate tolerance from a calibrated edge engine, glitches, ORE and LIN, the receive engine under a second of blast at 1, 2 and 3 Mbaud with every byte in sequence and a run of errored frames counted every other one (a status read on the wait for a frame, with no transmit engine, counted one of eight), a one-wire bus with a second node and the wired AND, mute and IrDA; the crystal's pad PA1 being column 3's RTS; the DMA vector's body from SRAM under `InRam` (196 to 197 cycles against 238 to 248 flattened in the flash), the block restart inside its window at 3 Mbaud from both with more to spare from SRAM, `isr()` placed only with its vector (76 cycles against 135) |
| [tim.md](tim.md) | TIM: the F1's TIM1 and TIM2 under WCH's names (three bits the F1 has not, TIM2's dead-time pairs through DTCR, the 32-bit CHxCVR carrying a captured level) and TIM3, a streamlined block with no pad; the tasks (PwmChannel through TimPwm/TimPairPwm, the meters, one timer counting or gated by another); measured: the time base exact against the STK, a centre-aligned period 2 x ATRLR, TIM2 counting TIM1 and TIM1 counting TIM2 with no wire, a duty measured internally, the break staged from a pad's pull (BIF unclearable while it stands), and TIM3's DMA request one-shot, probed nine ways, re-armed by the RCC pulse alone; on the jumper, the captures to the count, one pulse of 100 us captured at 101, a CHCVR read clearing the capture flag, and a slave mode that outlives every verb not naming it (which is why configure() zeroes SMCFGR); a stopped counter holds its outputs (a forced level reaches the pad only once CEN has run), and TIM1's CH2 and CH3 requests pacing a DMA channel one item a match, edge lists on PB0 and PB1 a second chip's USART received byte for byte |
| [adc.md](adc.md) | ADC: the F1's converter, 12 bits with a third control register (low power, three watchdogs that can reset the chip) on the CH32V006, 10 bits with the F1's calibration, Vcal on channel 9 and a trigger delay register on the CH32V003; the pads deriving their channel, VREFINT as the supply's ruler judged against the PVD's bracket, both groups, the timer triggers a group, the DMA, util's AnalogSampler over it unchanged; measured on both: tCONV to the cycle at four settings (1.47 Msps at the top of the first, 836 ksps of the second), six timer triggers and a seventh pacing the injection group (TIM3's CC1, or TIM2's CC3 where there is no TIM3), the signed injected offsets, the calibration and Vcal at 2/4 and 3/4 of AVDD, the watchdog scan (one watchdog per rank), the watchdog reset judged at the next boot, and the stall: a triggered, DMA-served run that occasionally stops converting until ADON is cycled, seen on both parts |
| [pin.md](pin.md) | GPIO, EXTI and the remaps: the one-bit MODE nibble, pulls through OUTDR, BSHR/BCR, the ten lines (eight pads by port select, the PVD's and the AWU's), interrupt or event, AFIO_PCFR1's columns as constexpr tables every driver takes its pads from; measured: the pulls, the nibbles, the atomics, the software trigger raising a flag only on a line in INTENR, a line in event mode ending a WFE in eighteen cycles, port B's seven nibbles, TIM1's channel moved from PD2 to PC4 by its remap; on one jumper the levels, the edges (a pad's edge raising no flag on a line enabled nowhere, the software trigger's rule) and the scanner |
| [opa.md](opa.md) | OPA: on the CH32V006 one amplifier behind a key pair, four positive pads, a negative pad or a programmable gain with the feedback switched in, the differential PGA and its bias, the output always on the ADC's channel 9; on the CH32V003 three bits of EXTEND_CTR, two pads a side, the output on PD4 (channel 7 through the pad) and no loop but a resistor; measured: the lock and the keys, the PGA reaching channel 9 at both rails, the differential sign, the differential input network a pull cannot drive, CMP2 not enabling on the CH32V006 (the comparators are the CH32V007's), and the CH32V003's amplifier open-loop as a comparator of its pads |
| [platform.md](platform.md) | Platform, its code the QingKe core stratum's over this family's facts ([../qingke/README.md](../qingke/README.md)): `Ch32v00xPlatform` (the csrrci critical section, the WFE-shaped `idle()` entered unmasked, the WFI rule that forces the WFE and the lost edge that forces the unmask, measured to the cycle, `ebreak`, the `.noinit` breadcrumb), `Pfic` and the one vector binding `BRIO_CH32_VECTOR` (the hardware prologue measured: 78 vs 84 cycles round trip on a minimal handler, 123 vs 144 on the console's USART handler, the default on; every vector aligned to four), the STK `BasicTicker`, `delay_us` on the STK counter, and the failing half: `Reset` (the flags as history, PINRSTF naming the pin alone on this family), `ResetReporter`, `fault_reset<P>()`, and the two watchdogs `Iwdg` and `Wwdg` (the IWDG biting at 255 ms for 258 computed at the measured LSI, the WWDG's step exact and its counter not running unarmed against the chapter's word); three real resets in the platform suite, three more in the watchdogs'; and a handler in SRAM, opt-in on both parts (`places_fast_text`, `.fast_text` and `.ram_text` in both scripts): what a driver places (`SpiHost`'s polled loops, the `Uart`'s DMA vector body for its block restart) and what a program places (its vector), measured flash against SRAM on the CH32V006: behind a vector in the flash an SRAM body pays only where it is long or loops, the call and the return eating a short one |
| [vendor/README.md](vendor/README.md) | The documents of record and their revisions |

The headers not yet behind a document of their own:

| Header | Content |
|--------|---------|
| [brio/ch32v00x/device.hpp](../../brio/ch32v00x/device.hpp) | The register map in the chapter's words: the buses, RCC, GPIO, USART, FLASH, the core's STK and PFIC, the interrupt numbers (the other blocks' maps live in their own headers) |
| [brio/ch32v00x/usart.hpp](../../brio/ch32v00x/usart.hpp) | `Usart<n>`, the resource over the whole of chapter 14, and `Uart<n, P, ...>`: the interrupt-driven byte transport on it (two rings, TXEIE armed and disarmed, errors read then cleared), its two optional DMA engine slots ([dma.md](dma.md)), its remap code ([pin.md](pin.md)) and its trailing options (the frame, a single wire, the flow-control pair); USART2 at codes 1..6 ([usart.md](usart.md)) |
