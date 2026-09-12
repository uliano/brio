# Target: CH32V00x (`ch32v00x/`)

The operational page for the CH32V00x target: WCH's QingKe V2C, a
RISC-V core with the **RV32EC** instruction set - sixteen registers,
compressed instructions, a multiply and no divide - on the
CH32V006K8U6 of the bench (62 KB flash, 8 KB SRAM, 48 MHz). The
smallest machine brio runs on, and its first RISC-V one: the kernel
and util strata run on it unchanged, which is the fact this target
exists to keep true.

Two things shape the stratum and are stated up front. **There is no
vendor header in the build**: WCH ships its register definitions inside
the EVT package, whose licence is written for software running on WCH
parts, so [brio/ch32v00x/device.hpp](../../brio/ch32v00x/device.hpp)
carries the map, read off the reference manual and answerable to it -
and per-part variability, which the other 32-bit strata ask the device
header for, has to be stated there when the second part arrives. And
**the core is not the RISC-V of the privileged specification in two
places that matter to a kernel** - a WFI that only wakes for an
interrupt it can take, and an MIE that is not cleared on interrupt
entry - both of which the platform header answers (below, "What the
silicon taught the stratum").

## The documents

One document per peripheral driver is the shape
[../README.md](../README.md) prescribes; on this target the drivers are
young enough that each header is still its own document of record.
The map below names them, and each row becomes a page as its
driver is measured on the bench.

| Document | Content |
|----------|---------|
| [clock.md](clock.md) | RCC: the two roots (HSI, the doubling PLL), the one divider, `Clock` and the `DynamicClock` that walks the HPRE ladder with the users rebased and the wait states following (measured: 48 to 3 MHz and back, the console clean at every rung), `Rcc` (the HSI trim, the LSI - ready in 17 us -, the MCO, the system clock monitor, the peripheral gates) |
| [sleep.md](sleep.md) | PWR: Sleep and Standby through the site contract (light is Sleep, standby and deep both Standby), the AWU as the alarm of a timed site with the LSI MEASURED against the STK (124 and 125 kHz on the two parts) and the frozen span handed back less the ticks counted awake, each part's own PVD ladder (two bits to 2.66 V, three to 4.4 V) and the LDO modes the CH32V006 alone has; measured on both: a 300 ms Standby ended by the AWU, the core waking on the HSI and the PLL restored, the event due and never early, the PVD bracketing the board's supply - and the PWR gate that answers rubbish until opened |
| [nvm.md](nvm.md) | FLASH: the engine (fast page program as the ONLY way to write, the two locks, three erase grains), the constant partition (the linker's 40 KB, the heap's 16 KB, the journal's 6 KB attic) and the two media with THE PAGE AS THE CELL; measured: an erase and a program each under a millisecond with the core stalled, a sector erase seven times faster than a page's, and a page that ACCEPTS a second program between erases - the finding a smaller cell could rest on, not yet taken |
| [dma.md](dma.md) | DMA: seven channels where THE CHANNEL IS THE REQUEST (table 8-2, no multiplexer), the STM32F1's channel with every field read-only while enabled, one vector a channel, and the two engines the console's own USART runs on in the suite; measured: three widths exact at about six HCLK cycles an item, EN STAYING SET after a completed block, and no hole in the map raising the transfer error the chapter promises |
| [spi.md](spi.md) | SPI: the F1's SPI with no FIFO and one register of its own (HSCR), 8/16-bit frames, the four modes, the select as a GPIO the request carries, `SpiHost` with the other strata's Request VERBATIM (pump or polled, engines on channels 2 and 3), `SpiClient`; measured: NO FIELD IS ENABLE-PROTECTED (seven of seven take a write under SPE), every path completing with nothing on MISO, and on one jumper the four modes at both widths through the pump, all eight BR codes timed on the polled path, the engines with and without a command phase and polled, the hardware CRC equal to a bitwise reference, the arbiter's replies and votes - and A DMA REQUEST LATCHED WHILE ITS CHANNEL IS DISABLED, served at the next enable, which is why the host arms the requests around the engines; against a SAM C21 peer on both parts: the four modes and both bit orders byte-exact, the BR ladder to 12 MHz, four kernel transactions in one select window read by the peer chip, and this board as a software-selected CLIENT under a foreign host, enabled on the select edge |
| [i2c.md](i2c.md) | I2C: the F1's event machine on two vectors, the receive procedures by count, NO RISE-TIME REGISTER on either part (the prose names one, the register lists do not), `I2cHost` with the other strata's Request VERBATIM and i2c_bus.hpp's outcomes (engines on channels 6 and 7, a one-byte read on the pump), `I2cClient`, the unstick; measured against a peer board: every tenure shape byte-exact, the two speeds, the DMA engines, the arbiter with a held SDA answered ARLO by the silicon and a held clock by the per-bus timeout, a STOPF the slave half raises on a STOP it was not addressed in; OADDR1's bit 14 reserved, the timing registers writable under PE |
| [usart.md](usart.md) | USART: the F1's USART minus two personalities on the CH32V006 (no synchronous mode, no smartcard - the reserved bits measured) and whole on the CH32V003 (the synchronous mode's verbs, its clock counted off the CK pad by TIM2 with no wire, the receiver sampling on that clock; the smartcard declined), `Usart<n>` with the whole of chapter 14 as verbs and the transport `Uart` on it with options that cost nothing (byte-identical), USART2's gate on PB2 while its registers answer on PB1; measured: every frame format both ways, a start bit that IS the divisor to the cycle at eight rates to 3 Mbaud, the three sampling points of the receiver and its tolerance, the break sent and detected, both mute wakes, the IrDA pulse in both modes and the decoder, CTS and RTS, the two DMA engines, one interrupt per flag - and HALF DUPLEX AS A BUS, NOT A LOOP: the receiver hears the TX pad and never its own frames |
| [tim.md](tim.md) | TIM: the F1's TIM1 and TIM2 under WCH's names (three bits the F1 has not, TIM2's dead-time pairs through DTCR, the 32-bit CHxCVR carrying a captured level) and TIM3, a streamlined block with no pad; the tasks (PwmChannel through TimPwm/TimPairPwm, the meters, one timer counting or gated by another); measured: the time base exact against the STK, a centre-aligned period 2 x ATRLR, TIM2 counting TIM1 and TIM1 counting TIM2 with no wire, a duty measured internally, the break staged from a pad's pull (BIF unclearable while it stands), and TIM3'S DMA REQUEST ONE-SHOT - probed nine ways, re-armed by the RCC pulse alone; on the jumper, the captures to the count, one pulse of 100 us captured at 101, a CHCVR read clearing the capture flag, and a slave mode that outlives every verb not naming it (which is why configure() zeroes SMCFGR) |
| [adc.md](adc.md) | ADC: the F1's converter - 12 bits with a third control register (low power, three watchdogs that can RESET THE CHIP) on the CH32V006, 10 bits with the F1's calibration, Vcal on channel 9 and a trigger delay register on the CH32V003 - the pads deriving their channel, VREFINT as the supply's ruler judged against the PVD's bracket, both groups, the timer triggers a group, the DMA, util's AnalogSampler over it unchanged; measured on both: tCONV to the cycle at four settings (1.47 Msps at the top of the first, 836 ksps of the second), six timer triggers and a seventh pacing the injection group (TIM3's CC1, or TIM2's CC3 where there is no TIM3), the signed injected offsets, the calibration and Vcal at 2/4 and 3/4 of AVDD, THE WATCHDOG SCAN (one watchdog per rank), the watchdog reset judged at the next boot - and THE STALL: a triggered, DMA-served run that occasionally stops converting until ADON is cycled, seen on both parts |
| [pin.md](pin.md) | GPIO, EXTI and the REMAPS: the one-bit MODE nibble, pulls through OUTDR, BSHR/BCR, the ten lines (eight pads by port select, the PVD's and the AWU's), interrupt or event, AFIO_PCFR1's columns as constexpr tables every driver takes its pads from; measured: the pulls, the nibbles, the atomics, the software trigger raising a flag only on a line in INTENR, a line in event mode ending a WFE in eighteen cycles, port B's seven nibbles, TIM1's channel moved from PD2 to PC4 by its remap; on one jumper the levels, the edges (a pad's edge raising no flag on a line enabled nowhere, the software trigger's rule) and the scanner |
| [opa.md](opa.md) | OPA: on the CH32V006 one amplifier behind a key pair, four positive pads, a negative pad or a programmable gain with the feedback switched in, the differential PGA and its bias, the output always on the ADC's channel 9; on the CH32V003 three bits of EXTEND_CTR, two pads a side, the output on PD4 (channel 7 through the pad) and no loop but a resistor; measured: the lock and the keys, the PGA reaching channel 9 at both rails, the differential sign, THE DIFFERENTIAL INPUT NETWORK a pull cannot drive, CMP2 not enabling on the CH32V006 - the comparators are the CH32V007's - and the CH32V003's amplifier open-loop as a comparator of its pads |
| [platform.md](platform.md) | Platform: `Ch32v00xPlatform` (the csrrci critical section, the WFE-shaped `idle()` and the WFI rule that forces it, `ebreak`, the `.noinit` breadcrumb), `Pfic` and the one handler attribute `BRIO_CH32_INTERRUPT` (the hardware prologue/epilogue MEASURED: 83 vs 92 cycles round trip, the default ON), the STK `BasicTicker`, `delay_us` on the STK counter, and the failing half - `Reset` (the flags as history, PINRSTF naming the pin alone on this family), `ResetReporter`, `fault_reset<P>()`, and the two watchdogs `Iwdg` and `Wwdg` (the IWDG biting at 255 ms for 258 computed at the measured LSI, the WWDG's step exact and ITS COUNTER NOT RUNNING UNARMED against the chapter's word); three real resets in the platform suite, three more in the watchdogs' |

The headers not yet behind a document of their own:

| Header | Content |
|--------|---------|
| [brio/ch32v00x/device.hpp](../../brio/ch32v00x/device.hpp) | The register map in the chapter's words: buses, RCC, GPIO, USART, FLASH, the core's STK and PFIC, the interrupt numbers (the other blocks' maps live in their own headers) |
| [brio/ch32v00x/usart.hpp](../../brio/ch32v00x/usart.hpp) | `Usart<n>`, the resource over the whole of chapter 14, and `Uart<n, P, ...>`: the interrupt-driven byte transport on it (two rings, TXEIE armed and disarmed, errors read then cleared), its two optional DMA engine slots (dma.md), its remap code (pin.md) and its trailing options (the frame, a single wire, the flow-control pair); USART2 at codes 1..6 - usart.md |

The documents of record and their revisions: [vendor/README.md](vendor/README.md).

## Toolchain

WCH's `riscv32-wch-elf` gcc **15.2.0** at `/sw/wch-riscv` (a symlink to
`wch-riscv-15.2.0`, extracted from the MounRiver toolchain package),
pointed at by absolute path from
[ch32v00x/cmake/toolchain-riscv.cmake](../../ch32v00x/cmake/toolchain-riscv.cmake).
The one compiler in this repository that is not self-built and not at
the project's usual version. What it brings that an upstream gcc does
not: a multilib set for rv32e (`rv32ec`, `rv32ec_zmmul` and their `_xw`
twins, each with newlib in full and nano form), which upstream gcc has
to be built with.

**The architecture string is the part's full ISA under WCH's gcc.**
`misa` on this part reads `0x40801014`: E, C, M and one non-standard
extension. The M is the multiply-only "m" of the datasheet's RV32EmC;
the non-standard extension is WCH's `xw` compressed set, which only
their compiler emits. The project compiles with
`-march=rv32ec_zmmul_xw -mabi=ilp32e` for the CH32V006 and
`-march=rv32ec_xw` for the CH32V003, whose V2A core has no multiplier
(`CH32V00X_ARCH` in the cache variables, derived from the part), and
the reason for `xw` is the family's smallest part: `xw` is
worth one to two per cent of every image - 72 to 516 bytes over the
suites, measured - and on a 16 KB CH32V003 those bytes are the
difference between a group of letters that links and one that does
not (the rule: [design/overview.md](../design/overview.md), "A
suite's image fits the family's smallest chip"). The consequence is
accepted: this family is built by the vendor compiler, and an
upstream gcc, which has no `xw`, is not a drop-in for the toolchain
file. The hardware prologue (below) is the other saving the compiler
offers, 8 to 220 bytes an image on top of its nine cycles.

**Interrupt handlers carry ONE attribute, `BRIO_CH32_INTERRUPT`**,
and which attribute it is belongs to the image: with the project's
`CH32V00X_HPE` option (ON by default) the crt sets INTSYSCR.HWSTKEN
and the handlers are declared with WCH's `WCH-Interrupt-fast`, the
core pushing and popping the caller-saved registers itself; with it
off, a handler is a plain `[[gnu::interrupt]]` function. One option for
the whole image, because a fast handler under an HPE that is off
corrupts the program it interrupted. Interrupt nesting is never turned
on. What the hardware prologue is worth was measured before it became
the default - nine cycles on a minimal round trip
([platform.md](platform.md)); the price is the vendor attribute, which
WCH's gcc has and an upstream gcc gets from the fast-interrupt patch.

## Board and build

Two boards, one per part: a CH32V006K8U6 module -
[../boards/ch32v006k8.md](../boards/ch32v006k8.md) - and WCH's
CH32V003F4P6 evaluation board -
[../boards/ch32v003f4.md](../boards/ch32v003f4.md). The project is
`ch32v00x/`, a sibling of the other four (own toolchain file, own
presets, one configure = one compiler): `CH32V00X_MCU` names the part
(`ch32v006k8` or `ch32v003f4`), and from it the project derives the
linker script ([ch32v00x/ld/](../../ch32v00x/ld/)), the PART
DEFINITION the stratum asks (`CH32V006` / `CH32V003` - with no vendor
header the build states the part, and `device.hpp` includes the
part's own table from `brio/ch32v00x/parts/`) and, unless
`CH32V00X_ARCH` says otherwise, the ISA: `rv32ec_zmmul_xw` for the
CH32V006's V2C core, `rv32ec_xw` for the CH32V003's V2A, which has no
multiplier. The CH32V006's script gives the linker the first 40 KB of
the 62, the top 22 KB being the storage partition
([nvm.md](nvm.md)); the CH32V003's gives it 15 KB of the 16, the top
kilobyte the journal's attic; `__brio_rom_end` is the boundary the
media read back. The crt is
[ch32v00x/src/glue/startup_ch32v00x.S](../../ch32v00x/src/glue/startup_ch32v00x.S),
compiled into every image, its table two entries shorter on the
CH32V003 (no USART2, no OPCM); after .data and .bss it paints the free
RAM up to the stack top, the ledger `stack_untouched()` reads
([platform.md](platform.md)).

```bash
(cd ch32v00x && cmake --preset ch32v006k8-release)
(cd ch32v00x && cmake --build --preset ch32v006k8-release --target console)
(cd ch32v00x && cmake --build --preset ch32v006k8-release --target console-upload)
(cd ch32v00x && cmake --preset ch32v003f4-release)                 # the second part
brio flash I console        # the bench way: board type v006k8 in the manifest
brio flash J console        # ... or v003f4, the probe named by its serial
brio console I              # its console: the probe's own serial port, 115200
```

Apps are auto-discovered from `ch32v00x/src/apps/*.cpp` with the same
`// build:` header grammar as the other projects; the board types an
app's `boards =` line names are `v006k8` (the default) and `v003f4`.
Every chapter is tiered for both parts; an app names the second when
its image fits there (the group axis below) and its wires exist on
that board - the serial suite stays on the CH32V006, whose USART2 is
its instrument.

**The CH32V003 splits a suite into group images.** Its 15 KB of
program flash and 2 KB of RAM do not hold every suite whole, so a
suite declares its groups of letters (`// build: groups = abcd,efg`)
and the `ch32v003f4` presets build it as one image per group,
`test_ch32_tim-1`, `test_ch32_tim-2`, ..., each compiled with
`BRIO_TEST_LETTERS` naming its group and registering those letters
alone - the menu of such an image says which letters it carries and
that the others are in its sibling images. The `ch32v006k8` presets
build the same source as one image with every letter. `brio flash J
test_ch32_tim-2` names an image; the bare name is refused there with
the list. The rule and its reasons: design/overview.md, "A suite's
image fits the family's smallest chip"; the mechanism:
`util/testbench.hpp` and `ch32v00x/CMakeLists.txt`'s discovery.

**The image starts at address 0 and its first word is an
instruction.** The core fetches address 0 as code, so the vector
table's entry 0 is `j reset_handler` and every entry after it is a
handler address (mtvec is set with both mode bits: absolute addresses,
vectored by interrupt number). The interrupt number IS the table's
word index - the STK at 12, USART1 at 32 - which is why
`device.hpp`'s `Irq` enum and the crt's table are the same list. The
handler names are the project's own (`systick_handler`,
`usart1_handler`, ...): with no vendor header there is no vendor
spelling to honour, and an app binds a vector by defining the strong
symbol.

**The clock the chip wakes up with is 8 MHz**: HSI 24 MHz with HPRE at
its reset value of /3, on both parts. `Clock<ClockSource::pll,
48'000'000>` takes it to the top of the range (the PLL only doubles;
HPRE divides), setting the flash wait states first - the part's own
table: 0 to 15 MHz, 1 to 24, 2 to 48 on the CH32V006; 0 to 24 and 1
to 48 on the CH32V003. There is no APB prescaler on this family, so
`pclk_hz` is `hz`.

## The probe (WCH-Link, WCH's OpenOCD)

A WCH-Link in RISC-V mode (USB `1a86:8010`), driven by **WCH's OpenOCD
fork** at `/sw/wch-openocd` - the only OpenOCD that speaks the probe's
SDI transport, a different program from the `/sw/openocd` the SWD
paths use. Its target script `wch-riscv.cfg` sits beside the binary,
not in a scripts tree. The probe's own page:
[../probes/wch-link.md](../probes/wch-link.md), which carries the two
traps met on the way in (an old probe firmware refusing the part; a
vendor binary linked against a library the distribution has not got).

The wires: **PD1 = SWDIO** (the 1-wire SDI, the chip's default debug
mode), GND, 3.3 V; PB3 = SWCLK for the 2-wire mode this family also
has. The debug interface is on after every reset (AFIO_PCFR1.SWCFG
resets to 0) and a program can turn it off, which is why WCH's serial
ISP on USART1 (PD5/PD6) - the same pins the console rides on this
bench - stays the escape route.

## Upload

`brio flash <board> <app>` and the project's `<app>-upload` target run
the same thing: `openocd -f wch-riscv.cfg -c "program <app>.elf
verify" -c "reset run" -c exit`. The fork's flash driver reads the
part's device id and flash size itself (`device id = 0x5dd3abcd`,
`flash size = 62kbytes` on the bench part); nothing names the part.
The chip is left RUNNING, the end state of every other path. No
debug-enable is taken back down afterwards: `break_here()` on this core
is an `ebreak`, which with no debugger attached escalates to the fault
vector instead of halting the core, so a just-flashed board and a
probe-less power-on behave the same way.

## Debugging

The fork starts a gdb server on port 3333 (`Info : starting gdb server
for wch_riscv.cpu.0 on 3333`), and the toolchain carries
`riscv32-wch-elf-gdb`. What the bring-up used, and is enough for one,
is OpenOCD's own console: `halt`, `reg pc`, `reg mstatus`, `reg
mcause`, `mdw`, `step`, `resume`. `.vscode/launch.json` carries a
cppdbg entry over the same server ("Debug CH32V006K8"), the AVR entry's
shape: the fork launched by the editor, gdb connecting by hand in
setupCommands, `load` through the fork's flash driver, a stop on
main() - written from the console dialogue and not yet driven from
the editor. Two facts about the core in a session: **a core in debug
mode cannot enter any sleep** (QingKe V2 manual 5.1), so the idle
path's behaviour is not observable with the probe halted on it; and
**the debugger's `step` does not take pending interrupts**, so a
single-step through an unmask instruction proves nothing about whether
an interrupt would be taken.

## The family check

`brio check ch32v00x` compiles every smoke TU under
`test/family_ch32v00x/` for BOTH parts with each part's own flags -
its definition and its ISA, the two facts the build derives from the
part number - and demands that every `neg/*.cpp` be refused on the
parts its `// mcu:` line names (cli/checks/check_ch32v00x.sh, the
RISC-V twin of the other three scripts; no CMake, no hardware,
seconds). A positive TU may carry a `// mcu:` line too: the fixtures
of the chapters still closed on the CH32V003 compile for the CH32V006
alone, and the ones that pin a part's values (the GPIO nibble, the
wait-state table, the flash geometry) pin each part's under its
definition. What the sweep also proves is the toolchain: `util_all.cpp`
includes EVERY kernel and util header and instantiates each service
over this platform, so a construct WCH's gcc 15.2 rejected would show
here first - one did (a loop-analysis false positive in
util/nv_heap.hpp's mount, answered by a bound the compiler can see,
byte-identical on the other three targets).

## Editor (clangd)

`brio/ch32v00x/.clangd` and `ch32v00x/.clangd` route both trees at the
project's own compilation database (`build-cmake/ch32v006k8-release`),
so a header of this stratum parses with the RISC-V compiler's real
command line whatever project CMake Tools has active.

## Serial console

USART1 on **PD5 (TX) and PD6 (RX)**, the default alternate function,
wired to the WCH-Link's TX/RX pins - so the probe's CDC port
(`/dev/serial/by-id/usb-wch.cn_WCH-Link_<serial>-if01`, a real USB
serial) is the console, at 115200. The line assembler completes a line
on `\n`; a `\r` alone is ignored (util/proto/line_parser.hpp), which is
what a terminal sending only carriage returns runs into.

## What the silicon taught the stratum

Each of these is enforced or answered in the code it names; they are
here because the manual states them quietly and the bench found them
loudly.

- **WFI wakes only for an interrupt the core can TAKE** (QingKe V2
  manual 5.2: the sleep is ended by "the interrupt source responded by
  the interrupt controller"). A `wfi` executed with mstatus.MIE clear
  sleeps past every pending interrupt for ever - the tick and the
  USART both pending, the core asleep, the console silent. The
  "sleep first, unmask after" idle of the two Cortex-M0+ targets is
  therefore a deadlock here, and `Ch32v00xPlatform::idle()` sleeps as
  a WFE instead: PFIC_SCTLR.WFITOWFE turns the next `wfi` into a
  wait-for-event and SEVONPEND makes every interrupt entering the
  pending state a LATCHED event, so an interrupt that turned pending
  between the kernel's queue check and the sleep makes the `wfi`
  return at once. The unmask that follows is what lets it be taken.
  WCH's own `__WFI()` is a WFE as well.
- **MIE is not cleared on interrupt entry** (QingKe V2 manual 2.2,
  for the sake of its nesting mode). With nesting disabled in
  INTSYSCR, as this crt leaves it, a handler still runs with MIE set
  and the controller holds the next line until MRET; the guard inside
  a handler behaves as anywhere else.
- **The STK flag must be cleared by the handler** (RM 6.5.4.2, CNTIF is
  "write 0 to clear"): unlike an ARM SysTick exception, entry does not
  clear it, and a handler that returns with it standing is re-entered
  for ever. `BasicTicker::tick()` clears it first.
- **The GPIO nibble looks like the STM32F1's and is not** (RM
  7.3.1.1): MODE is ONE bit here (output at the port's only speed, or
  input), the second bit of the field reserved. An F1 nibble such as
  0xB (AF push-pull "50 MHz") lands as AF push-pull with a reserved
  bit set - right by accident. `pin.hpp` spells CNF and MODE apart.
- **A namespace-scope reference to a register is dynamic
  initialization.** `auto& reg = *reinterpret_cast<...>(addr);` at
  namespace scope goes to `.init_array` and reads as zero in anything
  that runs before the constructors; on the first image of this
  bring-up every register access went through a null pointer. The
  stratum reaches registers through inline functions, and the crt
  walks `.init_array` anyway.
- **The PFIC's status banks are named backwards**: the manual's ISR
  is the ENABLE status and its IPR the PENDING status; the enable
  registers themselves (IENR/IRER) are write-only and read as zero.
  `Pfic::enabled()` and `Pfic::pending()` read the right bank.
- **PINRSTF names the pin alone.** On the STM32 register this one
  descends from the pin flag is raised beside every system reset; here
  a software reset boots to SFTRSTF and nothing else (measured). The
  flags mean what they say, and `reset.hpp` says so.
- **The hardware prologue is worth nine cycles on a minimal handler**
  (83 against 92 for the whole round trip), not the order of magnitude
  a "fast interrupt" suggests: the ten registers reach the stack either
  way, and what the hardware saves is the prologue's fetch. The numbers
  and the reasoning are in [platform.md](platform.md).
- **A DMA request that is never cleared is served ONCE**: TIM3's
  channel 3 and 4 matches move one transfer after a reset of the block
  and never another, whatever is rewritten or toggled ([tim.md](tim.md));
  a DMA read of a hole in the map completes as a normal block and no
  TEIF comes ([dma.md](dma.md)).
- **The ADC can stop converting under a hardware trigger served by
  the DMA** while the CPU works the peripheral buses - STRT standing,
  no EOC - and only an ADON cycle revives it ([adc.md](adc.md),
  `Adc::recover()`); the cause is not found.
- **A START into a busy bus is answered, not parked**: against a peer
  holding SDA low the I2C's START comes back ARLO at once, where the
  other three strata's peripherals park it until the bus frees; and no
  CTLR1 write may happen while the host's own STOP stands (the F1
  lineage's rule), so this engine's `start()` is the one that waits,
  bounded, for that STOP to leave ([i2c.md](i2c.md)).
- **A DMA request that rises while its channel is disabled is
  latched** and served at the channel's next enable: the SPI's RXNE
  under a standing RXDMAEN put a stale frame at the head of every
  engined block after the first. The host arms the requests around
  the engines and nowhere else.
- **USART2's gate is PB2's** (RCC_PB2PCENR bit 13, and its reset
  RCC_PB2PRSTR bit 13) while its registers answer in the PB1 address
  space; on the F1's bit of PB1 the block reads zero everywhere. The
  bus a block answers on and the register that clocks it are two
  facts.
- **Half duplex is a bus, not a loop**: HDSEL puts the receiver on the
  TX pad through the AF mux, and a frame driven onto that wire is
  heard - but not the instance's own frames, in any pad configuration.
  The single-wire loop-back the STM32G0's serial suite is built on does
  not exist here; the CH32 suite bit-bangs the RX pad and captures TX on
  a timer instead.
- **The chapters promise protections the silicon does not keep**: no
  SPI control field is enable-protected, the I2C's timing registers
  take a write under PE, the WWDG's counter does not run until armed
  and its BIF cannot be cleared while the input stands. Each is a
  refusal in the driver where the chapter's rule matters.

## Not covered yet

Driver gaps, each with its reason:

- **The CH32V003 tier's last pieces.** The part's table
  (`brio/ch32v00x/parts/ch32v003.hpp`) tiers every chapter: the core,
  the clock (the HSE included), the pads and every remap column, EXTI,
  USART1, the DMA, the timers (no TIM3, no dead time on TIM2), SPI,
  I2C (the same block), the ADC (10 bits, the calibration, Vcal, DLYR,
  one watchdog), the OPA (three bits of EXTEND_CTR), PWR (its PVD
  table, no LDO modes), the flash constants and the reset/watchdog
  block - each document says where the part differs, and every suite
  but the serial one runs green on the board, the larger ones as
  group images, the SPI and I2C ones on the wire against a peer. The
  USART's synchronous mode, which the CH32V003 has and the CH32V006
  has not, is measured there with no wire (usart.md); what stays
  declared, each with its reason: the smartcard (no card on the desk),
  the I2C's kernel letter, whose image alone is 216 bytes over the
  part's 15 KB and which no group of that build carries - the CH32V006's
  run of it stands for the block both parts share (i2c.md) - and the
  serial suite's other letters, whose instrument is USART2.
- The part's SVD for a register viewer, and the editor debug entry
  driven for real: the entry is written, the SVD is not fetched (the
  MounRiver package may carry one), and neither was needed to bring
  the target up.
- The DMA burst of the timers (DMACFGR/DMAADR), the front-end polling
  of the OPA, TouchKey, the DBG freezes of the timers, EXTEN's lock-up
  monitor: each a mode with no user, each named in its chapter's
  document.
- A `qingke/` core stratum: factored at the second RISC-V family,
  never earlier (the armv6m rule).

Implemented but not bench-verified, each with what would measure it:

- The idle path's power, and Standby's: both are proven to sleep and
  wake (`test_ch32_platform` letter b, `test_ch32_sleep` letter d)
  but not to sleep cheaply - a current measurement on a bench meter,
  with the probe detached (a core in debug mode never sleeps).
- `Pin` open-drain outputs and the levels a pad drives: written from
  the chapter and measured only through their pulls - the pad suite's
  jumper letters.
