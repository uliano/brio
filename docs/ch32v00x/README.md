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

| Header | Content |
|--------|---------|
| [brio/ch32v00x/device.hpp](../../brio/ch32v00x/device.hpp) | The register map in the chapter's words: buses, RCC, GPIO, USART, FLASH_ACTLR, the core's STK and PFIC, the interrupt numbers |
| [brio/ch32v00x/platform.hpp](../../brio/ch32v00x/platform.hpp) | `Ch32v00xPlatform`: the CriticalSection on mstatus.MIE, the WFE-shaped `idle()` and why, `ebreak`, the `.noinit` breadcrumb, atomic_width 4 |
| [brio/ch32v00x/pfic.hpp](../../brio/ch32v00x/pfic.hpp) | `InterruptGuard` (csrrci read-and-clear), the enable/disable/readback verbs, `Pfic` per-line enables |
| [brio/ch32v00x/ticker.hpp](../../brio/ch32v00x/ticker.hpp) | `BasicTicker` over the core's STK counter (up-count, auto-reload, the flag the handler must clear), `Ticker` at 1000 Hz |
| [brio/ch32v00x/clock.hpp](../../brio/ch32v00x/clock.hpp) | `Clock<internal or pll, hz>`: HSI 24 MHz, the doubling PLL, the HPRE divider table, the flash wait states |
| [brio/ch32v00x/pin.hpp](../../brio/ch32v00x/pin.hpp) | `Pin<'D', 5>`, `Port<'D'>`: the one-bit MODE this family has, pulls through OUTDR, the port clock opened by every configuring verb |
| [brio/ch32v00x/usart.hpp](../../brio/ch32v00x/usart.hpp) | `Uart<1, P>`: the interrupt-driven byte transport (two rings, TXEIE armed and disarmed, errors read then cleared) |

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

**The architecture string is a choice.** `misa` on this part reads
`0x40801014`: E, C, M and one non-standard extension. The M is the
multiply-only "m" of the datasheet's RV32EmC, so the project compiles
with `-march=rv32ec_zmmul -mabi=ilp32e`, which any RISC-V gcc can
produce. The non-standard extension is WCH's `xw` compressed
extension, worth a few per cent of code size and available only from
WCH's compiler - deliberately not the default (`CH32V00X_ARCH` in the
cache variables), so a self-built upstream toolchain can take this
file's place when it exists and the code will not care.

**Interrupt handlers are plain gcc.** The crt leaves INTSYSCR at its
reset value - hardware stacking and interrupt nesting off - so a
handler is a `[[gnu::interrupt]]` function that saves what it uses in
software and ends in MRET. WCH's own startup turns both on and needs
their compiler's `WCH-Interrupt-fast` attribute; the hardware stack is
an optimization to measure, not an assumption to build on.

## Board and build

The bench board is a CH32V006K8U6 module -
[../boards/ch32v006k8.md](../boards/ch32v006k8.md). The project is
`ch32v00x/`, a sibling of the other three (own toolchain file, own
presets, one configure = one compiler): `CH32V00X_MCU` selects the part
(`ch32v006k8`), which selects only the linker script
[ch32v00x/ld/ch32v006k8.ld](../../ch32v00x/ld/ch32v006k8.ld) - there is
no device-select define to pass. The crt is
[ch32v00x/src/glue/startup_ch32v00x.S](../../ch32v00x/src/glue/startup_ch32v00x.S),
compiled into every image.

```bash
(cd ch32v00x && cmake --preset ch32v006k8-release)
(cd ch32v00x && cmake --build --preset ch32v006k8-release --target console)
(cd ch32v00x && cmake --build --preset ch32v006k8-release --target console-upload)
brio flash I console        # the bench way: board type v006k8 in the manifest
brio console I              # its console: the probe's own serial port, 115200
```

Apps are auto-discovered from `ch32v00x/src/apps/*.cpp` with the same
`// build:` header grammar as the other projects; the board type an
app's `boards =` line names is `v006k8`, the default.

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
its reset value of /3. `Clock<ClockSource::pll, 48'000'000>` takes it
to the top of the range (the PLL only doubles; HPRE divides), setting
the flash wait states first (0 to 15 MHz, 1 to 24, 2 to 48). There is
no APB prescaler on this family, so `pclk_hz` is `hz`.

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
`riscv32-wch-elf-gdb`. What was used so far, and is enough for a
bring-up, is OpenOCD's own console: `halt`, `reg pc`, `reg mstatus`,
`reg mcause`, `mdw`, `step`, `resume`. Two facts about it: **a core in
debug mode cannot enter any sleep** (QingKe V2 manual 5.1), so the
idle path's behaviour is not observable with the probe halted on it;
and **the debugger's `step` does not take pending interrupts**, so a
single-step through an unmask instruction proves nothing about whether
an interrupt would be taken. A `launch.json` entry is not written yet.

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

## Not covered yet

Driver gaps, each with its reason:

- USART2 and the alternate-function REMAPS (AFIO_PCFR1): USART1 on its
  default pads is the one port the bench needs; USART2's default pads
  and every remap arrive with the first program that needs a second
  port or a moved pad.
- EXTI, TIM1/TIM2, ADC, I2C, SPI, DMA, the OPA, the watchdogs, the
  flash program/erase engine, the power modes (Sleep/Standby, the AWU)
  and a reset.hpp that says which reset happened: each is a chapter
  of the reference manual with no user yet, and each is born with
  its first user and its bench measurements, the way the other three
  strata's were.
- The family tiering (which parts have USART2 and the OPCM, which
  pins each package bonds): `device.hpp` states the CH32V006K8 alone,
  and the table that tells the parts apart needs a second part on the
  desk to be written against something real.
- `brio check ch32v00x`, the family smoke check: a fixture over one
  part is the part's own build; it is written with the second part.
- A debug configuration for the editor (`launch.json`) and the part's
  SVD for a register viewer: neither was needed to bring the target
  up, and MounRiver's package may carry the SVD.
- HSE (the crystal on PA1/PA2) and LSI as clock roots: named in
  `ClockSource`, refused at compile time, built with a board that
  carries a crystal.

Implemented but not bench-verified, each with what would measure it:

- The idle path's power: the WFE-shaped sleep is proven functionally
  (the console wakes on every byte and every tick) but not measured -
  whether the latched event is consumed by the `wfi` or leaves the
  loop spinning is a current measurement on a bench meter, with the
  probe detached (a core in debug mode never sleeps).
- `Pin` pulls, open-drain outputs and `analog()`: written from the
  chapter, exercised by no pad test yet - a pad test in the manner of
  the other targets' probe suites.
- `Clock` at the HSI rates (24, 12, 8, 6, 4, 3 MHz and below) and its
  `rebase` path: the console runs the PLL at 48 MHz; a clock suite
  that steps through the divider table and reads the rate back on a
  timer would cover the rest.
- The `.noinit` panic breadcrumb across a reset: the section is placed
  and the crt skips it, but no program has yet written a record,
  reset, and read it back.
- The USART's error counters (framing, noise, parity, hardware
  overrun): the paths are written; the stress suites that provoke
  each condition from the host side are what verifies them.
