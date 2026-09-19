# CLAUDE.md

The operating manual of this repository, for the assistant (Claude
Code loads it) and for a contributor alike: where the truth lives, what
is settled, how work is done, what is open. It does NOT duplicate the
design docs - it points at them. Personal settings (the language of the
conversation, the desk, the private notes) belong in `CLAUDE.local.md`,
which is not published.

Every edit in the files - code, comments, docs, commit messages - is in
English.

## Allowed text symbols

Only ASCII <= 127 in every file of the repo (code, docs, this file).

## Where the truth lives

- `README.md` - brio's shopfront: what it is, an application snippet,
  the ideas, layering and the target table. No history, no apps.
- `docs/design/*.md` - the target-independent design, by intent: WHY
  and contracts (`architecture.svg` = the strata diagram, hand-written
  SVG). `overview.md` (philosophy, governing rule, layering, style),
  `kernel.md` (the AO kernel: model, contract, events, payloads,
  queues, FSM, delivery, scheduler, time, panic, platform, index),
  `clock.md` (the clock model), `serial.md`, `spi-bus.md`, `i2c-bus.md`,
  `can.md` (the CAN vocabulary three controllers share: the classic
  frame, the timing in human units with a controller's limits as a
  value and the exact search, the error codes and the error state as
  one observable; no bus AO, for reasons the page gives),
  `ring.md`, `analog.md` (the sampler usage type + arithmetic),
  `nv-heap.md` (the flash block allocator: FlashMedia contract, map
  pair, survival-aware mount), `nv-journal.md` (the small-value store
  over the SAME contract: two halves that ping-pong wholesale, seq
  decides and CRC judges, read-only mount, the panic reserve, and the
  decision that NvRecord and NvJournal stay two spellings), `power.md`
  (the sleep-depth ladder, the
  site that only arms, the vote round, standing locks, the deadline
  guard, the first-event-after-wake contract), `meters.md` (the
  MeterLatch bridge out of a capture ISR and the MeterSampler that
  paces publication, not capture - a stale source publishes nothing),
  `usb.md` (the USB device side: the controller contract drawn at
  the packet, the control-endpoint machine written once, the class
  contract, CDC ACM as a byte transport; no host side),
  `block-stream.md` (block streams: BlockSource/BlockPlayer concepts
  over caller-owned buffers - blocks, not DMA - and the BlockRelay AO
  lending each filled block for one dispatch; built BEFORE its second
  implementation as the fixed point the next platform is measured
  against), `gfx.md` (drawing: the three kinds of surface told apart by
  where a pixel's truth lives, the write-only base the library draws
  through and nothing else, the two verbs panels are good at, the
  conventions that would otherwise become one-pixel errors, no
  compositing, and THE THREE PLANES OF TRUTH - a reference renderer for
  the primitives, a differential oracle for the pipeline, the bench for
  the silicon; the contract settled before the primitives so the easy
  case cannot shape it), `simulation.md` (the world a program runs
  against when the machine under it is the host: the world never speaks
  to the program, so every device's seam is a LEVEL and not a gesture;
  the channel table in both directions, whose columns are either
  stimulated or COMPUTED - six rules that make a computed one
  deterministic, and the closed loop they buy with no new mechanism; the
  world as a type with two verbs; the three time policies and who may
  join each; the scenario file, where outcomes are stimuli and the
  degrees of freedom are declared; goldens against invariants; and what
  the host does not tell you - written before its implementation, for
  gfx.md's reason).
- `docs/<target>/` - one folder per target, mirroring
  `brio/<target>/` (`avrdx/`, `samc21/`, `stm32g0/`, `stm32f4/`,
  `ch32v00x/`, `ch32v203/`, `rp2040/`, `rp2350/`, `host/`), plus one
  per stratum that sits between `util/` and the targets (`cortexm/`,
  and the IP strata `pl011/`, `pl022/`, `dw_apb_i2c/`): `README.md` is
  the
  operational page (toolchain, board, probe, debugger and their
  quirks); next to it ONE document per peripheral in the shape
  docs/README.md prescribes (documents of record -> what the silicon
  does -> types and verbs -> how to use it, one example per use ->
  bench findings -> "Not covered yet"). No document carries a banner:
  maturity is a property of the PLATFORM, stated once in README.md's
  target table (`supported` / `in bring-up`). Every document closes
  with "Not covered yet" as TWO LISTS - driver gaps, each with its
  REASON (a wire, a peer board, a meter, a supply; born with its first
  user; declined because ...) kept distinct from implemented-but-not-
  bench-verified, each with what would measure it - and a "gap" the
  document's own findings cover is deleted; a document with nothing in
  either list has no such section. The public voice names a silicon by
  its PART and never by an ordinal ("the Nth silicon"), and no desk
  letter (the manifest's positions) appears outside private/. The
  state of the driver work is readable in each target's document map,
  the second section of its README.md (docs/README.md is one row per
  target and the rules). The Multislope assessment (every acrobatic
  piece maps to fixed routes + tasks on resources + config structs; the
  64-cycle snapshot stays in the ISR body) lives in memory and in the
  private track log, not in docs.
- `docs/boards/` and `docs/probes/` - one page per board brio is
  tested on (what it carries, its manifest type, its documents) and
  one per probe (the flash mechanisms and their traps); `brio apps`
  lists the apps from their own headers. THE DESK - which board is
  plugged in where, the incidents, the end state - is `private/`, a
  git-ignored nested repository: `private/bench_boards.py` (the real
  manifest, loaded before the public example), `private/bench.md`
  (the diary), `private/TODO.md`.
- `docs/avrdx/vendor/README.md` - the datasheets/errata by document
  number and the chapters we use; PDFs are local symlinks
  (git-ignored), cite by SECTION as "DS40002247B 16.5.2" (pages move
  between revisions). Documents of record: datasheet DS40002247B
  (2023) and errata DS80000915F (2025), fetched into
  docs/avrdx/vendor/ (git-ignored); the copies in
  ~/Documenti/Elettronica/AVR/ are the older rev. A of both (and the
  file named AVR64DB...Errata there IS the AVR128DB errata). Check
  revisions before trusting a local PDF.
- Headers - the canonical API reference; header comments explain the
  concurrency model and the WHY of each tradeoff.

Rules (full text in `docs/README.md`): any change that alters a
documented decision updates the matching doc in the same change; docs
say today's truth only (no change history, no dates, no renames);
design/ and the target folders never reference individual apps; never duplicate
signatures into docs; new decisions go into `docs/design/`, not here.
This file has no decision log any more: the former log was migrated to
`docs/design/` and `docs/design/*` is authoritative.

## The project in one paragraph

`brio` (`brio/`) is a header-only C++23 (gnu++23) framework for
bare-metal MCUs built around a cooperative active-object kernel,
written clean-room after Samek's book (never the QP source). One flat
namespace `brio`; the strata under `brio/`, one directory each, are
`kernel/` (pure
logic, includes nothing of brio), `util/` (services over the kernel),
`gfx/` (drawing: pure, target-independent, and needing nothing of the
kernel - three kinds of surface told apart by where a pixel's truth
lives, and a library that draws through the write-only base whatever
lies beneath, design/gfx.md),
`cortexm/` (the CORE stratum every Cortex-M family includes after its
device header - the M0+ families, the M4 one and the Arm half of the
RP2350's M33 pair, one programmer's model for all of them: NVIC +
PRIMASK guard, the SysTick ticker, the microsecond busy-wait on
SysTick's counter - and the nvic/ticker guards accept the M33 core
header where the delay one, which the RP2350 does not use, does not),
`pl011/`, `pl022/` and `dw_apb_i2c/` (the IP STRATA: ARM's PrimeCell
UART and SSP and Synopsys's DesignWare I2C, each a peripheral DESIGN
written once, knowing no chip and including nothing of a family - what
a family owes one is a TRAITS type satisfying the concept the IP file
states, `Pl011Chip` and its two siblings: register block, reset,
interrupt line and controller, guard, pin legality, clock, DMA
requests - while the family's own header keeps the PUBLIC NAMES the
apps write. Born under the core stratum's rule, at the SECOND family
that carries the block, gated by the images; what earns one is that
the register description is not similar between the chips but
IDENTICAL),
`avrdx/` (everything that knows `avr/io.h`: AVR DA/DB, bench chip
AVR128DB48), `samc21/` (everything that knows `sam.h`: SAM C21,
Cortex-M0+, bench chip ATSAMC21J18A), `stm32g0/` (everything that
knows `stm32g0xx.h`: STM32G0, Cortex-M0+, bench chip STM32G0B1RE on
a Nucleo-64), `ch32v00x/` (everything that knows the CH32V00x: WCH's
QingKe V2C, RV32EC, bench chip CH32V006K8U6 - NO vendor header, the
register map is the stratum's own device.hpp), `ch32v203/` (everything
that knows the CH32V203: WCH's QingKe V4B, RV32IMAC with the full
register file, bench chip CH32V203C8T6 on a WeAct core board - no
vendor header either, and the STM32F1's peripheral generation under
WCH's names, which is what separates it from the CH32V00x),
`rp2040/` (everything
that knows the RP2040: Raspberry Pi's dual Cortex-M0+, bench chip an
RP2040 B2 on a Raspberry Pi Pico and on a WeAct board, the pico-sdk's CMSIS header and register
definitions vendored, a kernel per core), `rp2350/` (everything that
knows the RP2350: Raspberry Pi's silicon with TWO PROCESSOR
ARCHITECTURES over one set of peripherals - a Cortex-M33 pair and a
Hazard3 RISC-V pair, of which exactly one runs, chosen by the
IMAGE_DEF block in the image the bootrom finds and by nothing else, so
the architecture is an axis of the build and every suite is written
once and run twice; bench chip an RP2350 in the QFN-80 package,
stepping A2, on a WeAct RP2350B core board; `core.hpp` is THE ONE FILE
of the stratum that asks `__riscv`, and both halves export the same
names; the pico-sdk's rp2350 device description vendored in an include
root of its own, with a stub for the core header the RISC-V build must
not take; no second-stage bootloader - the bootrom sets the XIP
interface up itself), `stm32f4/` (everything that
knows `stm32f4xx.h`: STM32F4, Cortex-M4F - brio's first ARMv7-M family,
built with the hard-float ABI, the FPU enabled by the crt; bench chips
STM32F429ZI on an STM32F429I-DISC1, STM32F446RE on a Nucleo-64,
STM32F411CE on a WeAct black pill), `host/` (the native test
target). Includes carry the stratum prefix
(`#include "avrdx/usart.hpp"`). The builds are sibling CMake
projects, PEERS - the repo root is not a CMake project: the CROSS ones
`avrdx/`, `samc21/`, `stm32g0/`, `stm32f4/`, `ch32v00x/`, `ch32v203/`,
`rp2040/` and `rp2350/` (each with its own toolchain file
and presets, Ninja, emitting into the shared `build-cmake/`)
auto-discover one `main()` per `src/apps/<app>.cpp` at configure time
from its own `// build:` header comment; host tests in `test/` are a
project of their own (host g++, no cross toolchain), run via `ctest`,
and `host/` another - the host's own apps, which RUN rather than run
and exit, and which ctest never sees. The `rp2350/` project is the
only one with a second axis besides the chip: the ARCHITECTURE, two
toolchain files and two presets per build type. ONE NAME PER ARCHITECTURE,
the same key on three axes: `brio/<arch>/` (stratum),
`docs/<arch>/` (docs), `<arch>/` (build project); chip precision
lives in preset names, per-chip ld/svd files and the `*_MCU` cache
variables. Names are claims: a stratum is named for exactly the family
it has been proven on, and a name widens only when a real chip proves
it shares the stratum. So `samc21` is final (the C21 is the only SAM
this stratum has known; a D21 would not share GCLK/PM/SYSCTRL and would
earn its own stratum); the one landing name still pending is avrdx ->
avrxt (Microchip's sigla for the modern-AVR core) when an EA/mega0 part
proves it shares the stratum; and stm32g0 SHARES its name with the G0x0
value line, decided on the headers (every x0 header is a strict subset
of its x1 twin: the same IP under the same register names) - the
stratum compiles on all twelve G0 headers of the pack with the reserve
deriving every vector from PERIPHERAL PRESENCE and no device name
spelled anywhere, the bench proof on x0 silicon pending a board. The
`cortexm/` core stratum (nvic, ticker, delay) is what the Cortex-M
families share whatever the vendor and whatever the profile - the same
programmer's model for
SysTick, the NVIC's enables and PRIMASK - factored with the first two in
hand and every image byte-identical before and after, renamed from its
architecture to its core family when the M4 joined, under the same
gate; a RISC-V core stratum would be
factored the same way, at its second family, never earlier - and
Hazard3 is not a QingKe, so `rp2350/core_hazard3.hpp` stays this
stratum's own until a second RISC-V family of that shape earns one.

## Governing rule and stability hierarchy

**Nothing is settled.** Reusing existing code is fine only while it
does not limit the design above it; when a limitation can be overcome
by rewriting what sits below, the rewrite wins. Every stage of work
requires critical analysis at ALL levels of the stack, not just the
one being added.

Within that, three levels of stability guide how much a change should
disturb:

- **kernel ideas** - fairly stable: the AO contract (`Event`, `queue`,
  `init`, `dispatch`), value events per-AO variant, the two loans
  (`Lease::dispatch` / `Lease::reply`), post/publish/reply, priority =
  pack order, timers post events, panic breadcrumb, Platform concept,
  TWO CONTEXTS AND ONE BOUNDARY (ISR over main, and an ISR body runs to
  completion too - no interrupt nests over another, on any target;
  design/kernel.md section 1 and the platform promise in section 11);
- **util/ services and target drivers** - important, here to stay,
  but expected to change (possibly radically) as targets are added;
- **apps** - incidental test tools; they will not survive in their
  current form. Nothing in the foundations or their docs may depend on
  an app; apps document themselves in their own header comment.

## Working discipline (read this first, every session)

The failure mode to guard against is EFFORT PARSIMONY: solving the one
concrete problem on the bench chip instead of building the framework.
It produces drivers that do not compile on half the family, docs marked
complete that list their own gaps, and false comments justifying wrong
restrictions. The antidote, in practice:

- **Framework, not application.** The target is the whole AVR DA/DB
  range (and future targets), the AVR128DB48 is only the test vehicle.
  Cover the chapter's FULL option space - every instance, mode, route
  from the register description, both errata documents (DB
  DS80000915F and DA DS80000882C differ). Leave something out only
  knowingly and declare it in the doc's "Not covered yet".
- **Definition of done for a driver**: (1) systematic pass over the
  chapter's register description + errata; (2) a smoke TU compiled for
  every package - `avr-g++ -mmcu=avr128d{a,b}{28,48,64} -std=gnu++23
  -Os -c -I brio` takes seconds, no hardware; (3) negative
  tests: what must be refused must FAIL to compile; (4) the
  `test_<target>_<subject>` suite on the bench; (5) `brio prose`
  clean over the files touched, and `brio gate` for every change that
  claims to move no image (a comments-only edit, a rename, a move) - a comment or a document is a reference
  for the code as it is, and a claim about another part of the tree is
  a POINTER the tool can check, never a statement that ages. The bench chip alone
  masks half the family (SWEVENTB, TCA1, PORTB proved it).
- **Package variability pattern** (full rule: overview.md "Target
  strata"; model code: tcb.hpp/pin.hpp/evsys.hpp): device header =
  authority. Missing instance -> `#if defined(TCB4)` tiers. Missing
  pin POSITION -> instance stays usable: `port_exists` +
  `if constexpr` compile the branch out (a runtime `if` on a missing
  Pin kills the instance), `init<cfg>` static_asserts, `init(cfg)`
  returns false. Missing register/enum -> gate on its header symbol.
  Pin-level bonding inside an existing port -> device tables (open).
- **Never state what is not enforced**: a fact in a doc or comment
  either has a guard in the code or sits in "Not covered yet".
- **Docs are a reference for the CURRENT version** (rules:
  docs/README.md): no history, no dates, no work narrative, no app
  names (test suites excepted); no banner, "Not covered yet" as two
  lists with a reason on every item (driver gaps separate from
  implemented-but-not-bench-verified), nothing listed that the
  document's own findings cover; doc and code move in the same
  change.
- **When every variant of OUR code fails on the bench, consult the
  vendor's reference implementation - as an ORACLE, never as a source.**
  The trigger is a written list of measured variants (sequence, memory,
  width, instance, clock, reset, interrupts...) all failing the same way,
  not an impression. Then, in order: read the vendor's sequence and
  compare it register for register with ours; if reading does not
  discriminate, build a test on the vendor's library ALONE and run it on
  the SAME board doing the SAME operation; if it passes, bisect between
  the two sequences down to the ONE difference and record that fact
  (with its reason) in the driver and the document - never the vendor's
  code or its shape; if it fails too, it is the silicon or the board, and
  worth a question to the vendor. The corollary that makes this a rule: a
  behaviour the reference implementation never triggers never reaches an
  errata sheet, so the absence of an erratum is evidence of nothing.
  (The STM32F4 DMA's read-back wedge is the case that taught it.)
- **When the user refines the method, write it to memory in the same
  session** - the next context must start from the agreed method, not
  regress to the instinctive minimum.
- **Commit messages speak the public voice**: the change and its
  verification (the family checks, the host tests, the gate's count and
  its movers, the suites re-run) - never a desk position, an ordinal
  silicon or a chronicle; what happened on the desk goes to the private
  diary.

## Standing style rulings

Types PascalCase, functions/constants snake_case; private members
trailing underscore; no `Ao` suffix on AO class names; queues speak
push/pop; `std::optional` returns instead of bool + out-param; no
`*_from_isr` API doubling; no redundant `inline` on in-class
definitions; concepts instead of virtual interfaces; use the
freestanding libstdc++ (variant, optional, span, concepts, bit,
expected...) instead of hand-rolled traits; C++26 features already in
gcc 16 may be used when genuinely needed (bump the -std flag then).
Explicit-size integer types for every stored or exchanged value;
arithmetic that can exceed 16 bits names its width (UL literal,
explicit-width accumulator, cast on an OPERAND never on the result) -
`int`/`auto` stay fine for ephemeral locals; native tests run under
non-recovering UBSan. Handlers dispatch with `brio::match(e,
lambdas...)`. Drivers expose
ISR handler BODIES (`[[gnu::always_inline]]`), the app binds the
vector - vector names never appear in portable code. No `#ifdef` where
a template/concept boundary can do the job - and where only the
preprocessor can ask (does this vendor macro exist?), a probe that
yields a VALUE lives in the family's device-tables header
(samc21/device_tables.hpp), never in a driver; a driver keeps `#ifdef`
only to select per-instance CODE (full text in overview.md
"Generalization rule"); no target includes outside
the target strata; the kernel must never know which silicon it runs
on. Apps never touch registers (PORTx/VPORTx/peripheral structs live
only in target drivers; the ISR vector binding is the one vendor glue an
app may contain). Full text: `docs/design/overview.md`.

## Open items and horizon

Roughly ordered by proximity. None of these is a decision yet; each
gets its home in `docs/design/` when taken.

- **The harmonization pass** - the rules under which everything born
  after is born: the rename (the task speaks brio's words, the resource
  the chapter's), the voice pass, the prose net, the gap lists as two
  reasoned lists with no banner, the realizations tables (the one home
  of the cross-target view, indexed in overview.md), the true synonyms
  reconciled and the traps recorded. What remains: the public/private
  cut's last pieces, then a first public release with a fresh root at
  tag 0.1 (the development history stays whole in a private archive).
- **The first portable example application** on the three platforms
  (candidates: an ILI9481 display over SPI, an MCP47CVB22/MCP3550 DAC-ADC
  loop): application logic target-free over util and tasks, one thin
  board file per target. It gives birth to the board-file shape, to
  chapter one of a learning track, and it is where the open design point
  on TRANSFER GRANULARITY closes - the frame (8 fixed on the AVR, 8 or 9
  on the SAM, 4..16 on the G0), the access granularity and the machine
  word are three widths, the cost per frame is the interrupt turnaround,
  and "the element type is the beat" (the DMA engines' rule) is the
  answer the SPI Request does not yet spell; measured on a block of
  16-bit pixels, not imagined.
- **The CH32V00x stratum's second part.** `brio/ch32v00x/` and
  `ch32v00x/` are `supported` on the CH32V006K8U6 (README.md's table):
  every chapter of the reference manual has its document and its
  suite green on the module, the bus chapters on the wire (I2C against
  a peer board, SPI and the USART on the module's own jumpers), on
  WCH's gcc 15.2 and WCH's OpenOCD fork through a WCH-Link. What
  remains: the CH32V003F4P6 as the SECOND AND LAST part of the family
  (these two parts and no more, by decision) - its tier is OPEN: the
  part table, the two-part check matrix, the presets, the console and
  the platform suite are on the board, and six chapters (ADC, OPA,
  sleep, the timers, SPI, I2C) are closed on it by name until written
  against its own register description (docs/ch32v00x/README.md's gap
  list; its reference manual is not on the desk). It is the most
  extreme point brio can touch - 16 KB of flash and 2 KB of RAM, the
  QingKe V2A with no multiplier. For its sake, this family is built
  by WCH's gcc with the `xw` extension
  and the hardware prologue (both measured: one to two per cent and
  8..220 bytes an image), so no upstream gcc is sought for it; and
  its suites are shaped by the smallest-chip rule (design/overview.md,
  "A suite's image fits the family's smallest chip"). A `qingke/` core
  stratum only at a second QingKe family.
- **The STM32F4 stratum.** `brio/stm32f4/` and `stm32f4/` are
  `supported` on the STM32F429ZI, the STM32F446RE and the STM32F411CE
  (README.md's table) on three boards (STM32F429I-DISC1, Nucleo-F446RE,
  an STM32F411CE black pill on a standalone STLINK-V3) with kernel/ and
  util/ untouched - the platform,
  the clock (the regulator scale and over-drive sequenced, the APB
  prescalers unpinned), the pins and the USART with their documents,
  the family check over all twenty-three headers; then chapter by
  chapter, each with its document and its suite green on every board it
  builds for: reset and the watchdogs, EXTI + SYSCFG, the RTC and the
  backup domain, the DMA (the Uart's engine slots filled), the timers,
  the ADC and the DAC (read back on the pad they share), SPI/I2S and I2C
  (against the gyroscope and the touch controller a board carries), the
  USB OTG core for util/usb (the console on the black pill's own
  connector), PWR with the sleep sites and the dynamic clock, and the
  flash interface as the ENGINE alone (no FlashMedia, by the NV review's
  decision below), the CRC unit, the RNG (written for the parts that
  have one, measured on the STM32F429), the bxCAN in loopback and the FMPI2C1 of the
  F410/F412/F413/F446 - the STM32G0's I2C under another name, measured
  on a bus with no device on it, the FMC with the SDRAM a board
  carries (8 MB byte-exact, the two traps against the manual), and on
  the F429 alone the LTDC and the DMA2D - the memory-mapped display
  tier over that SDRAM, the panel driven in its RGB mode and the
  bandwidth the tier lives on measured (two layers of 32-bit pixels at
  65 Hz plus the accelerator, some 160 MB/s over one bus). What remains
  is in the documents' gap lists, and three things outside them: the
  OTG HS core in full-speed mode on the STM32F429 (UsbHs compiled, its
  connector cabled, never enumerated); the frequency ladders of the five
  part classes whose manuals are not on the desk (a rate above 16 MHz
  refused there); the debugger driven from the command line as
  cortex-debug would (docs/stm32f4/README.md) and not yet from the
  editor. Tenuto only, the equal-priority promise kept; Rubato and
  BASEPRI are another type and another day.
- **The RP2040 stratum.** `brio/rp2040/` and `rp2040/` are
  `supported` on the RP2040 (README.md's table) on a Raspberry Pi
  Pico and a WeAct board: every chapter of the datasheet's plan has
  its document and its suite green on both boards - the platform, the
  clock, the PL011 UART, the timer, the watchdog and the resets, the
  DMA, the SPI, the I2C, the PWM, the ADC, the RTC, the PIO, the
  flash with the heap and the journal at the top of the QSPI chip,
  the power chapter with the SLEEP state and DORMANT - and THE SECOND
  CORE RUNS A KERNEL OF ITS OWN: one platform type per core,
  util/inbox.hpp's bridge, the launch through the bootrom's protocol,
  a console per core over two probe bridges (design/kernel.md section
  12, docs/rp2040/multicore.md). The pico-sdk's device description is
  vendored, the boot stage checked in as bytes, the crt its own; the
  WeAct board's Zetta flash wants the OpenOCD built from git. What
  remains is in the documents' gap lists (a power vote across the
  cores, the bus fabric's counters). The USB device stack
  (util/usb/) was born here, with its CDC console on the chip's own
  connector. Its PL011, PL022 and DW_apb_i2c drivers live in the IP
  strata now (`brio/pl011/`, `brio/pl022/`, `brio/dw_apb_i2c/`), the
  family header holding the traits and the public names.
- **The RP2350 stratum.** `brio/rp2350/` and `rp2350/` are
  `in bring-up` on an RP2350 in the QFN-80 package (README.md's table)
  on a WeAct RP2350B core board, and the whole of it is written ONCE
  and run TWICE: a Cortex-M33 pair and a Hazard3 RISC-V pair over one
  set of peripherals, the architecture chosen by the IMAGE_DEF block in
  the image the bootrom finds - no button, no fuse, no second stage -
  so it is an axis of the build, `rp2350/` carries two toolchain files
  and two presets per build type, and a suite is green when it is green
  on both halves. `core.hpp` is the one file that asks `__riscv`;
  `core_m33.hpp` is the device header plus `cortexm/nvic.hpp` as the
  other ARM families' are, `core_hazard3.hpp` is this stratum's own.
  Which chapters have their document and their suite is
  docs/rp2350/README.md's document map, and what each still owes is
  its own gap list. What remains as whole chapters: THE POWER CHAPTER
  (POWMAN with its always-on timer and the sleep states) is not
  written, and the HSTX and the M33's coprocessors - the other two
  blocks the RP2040 never had - are declared gaps. The bench verb here
  is STATE-INDEPENDENT by construction (a rescue through the RP-AP over
  the debug port alone, then programming as core 0 of the Arm pair,
  then a reset), which is what will make the sleep chapters safe to
  write; Raspberry Pi's OpenOCD fork is the only build that reaches
  this chip, upstream's RISC-V target driver taking no DAP. The three
  IP strata were born with this family, at the second chip that carries
  each block.
- **The NV stack reviewed as a whole.** The flash heap and the journal
  were born where flash was cheap to partition (a zone off the end of
  the image on the AVR, the top of the array elsewhere) and carried to
  every target since for coherence; the STM32F4 is the first family
  where the geometry pushes back (16 KB sectors only at the bottom of
  the bank, 128 KB above), and the question it raises is prior to the
  design point it seemed to pose: WHICH programs need a flash-backed
  store, which need the heap's blocks against the journal's small
  values, and whether a zone at a fixed address is worth a partition
  every image pays. Until that review, no FlashMedia on the STM32F4.
- **The CH32V203 stratum's chapters.** `brio/ch32v203/` and `ch32v203/`
  are `in bring-up` on the CH32V203C8T6 (README.md's table), with the
  nine parts of the series in the part table and the family check on
  all of them: measured on the silicon are the platform with its failing
  half, the clock tree whole, GPIO with the remaps and the EXTI, the
  timers and the two watchdogs, the DMA, the two converters and the two
  amplifiers, the USART chapter whole (the frame, both buses' divisors,
  mute mode, LIN, half duplex, IrDA, the smartcard and the synchronous
  clock) and THE USB DEVICE CONTROLLER - the kernel console runs both on
  the probe's serial and on the chip's own USB-C, a CDC ACM port over
  util/usb, and the chapter has its document and its suite.
  Open: CAN (its driver written against the manual and kept for a pass
  across every platform), TKEY and the second USB block. What the
  silicon taught is in docs/ch32v203/README.md, and one finding shapes
  the power model: IN SLEEP THE BUS MATRIX SERVES THE CORE ALONE - the
  USB controller cannot reach its packet memory and a DMA stalls
  (measured with the vendor's own example as the oracle, and no software
  mitigation short of staying awake works), so a program that moves data
  through the bus does not SLEEP here and slows down instead, to no less
  than 24 MHz of HCLK; the sleep sites and the platform's idle path both
  read a COUNT of active bus masters - a DMA channel while its EN is up,
  the USB controller from its pull-up - and neither sleeps above zero.
- **Test consolidation per platform** when its chapters are closed: a
  two-level TestBench (groups over letters), few units per platform by
  domain, one logical unit on the host side, an .md per unit - the
  group being the unit an image carries, sized by the family's
  smallest chip (the rule in design/overview.md; the CH32V00x is where
  it bites first).
- **Queued**: the SAM's CAN (two transceivers; its M_CAN would be the
  second FD controller design/can.md's shared FD frame waits for), the
  energy experiment's G0 instance,
  Multislope (an application), avrdx -> avrxt when a part proves it.
- **Borrowed, phase 2 (debug epoch).** `Borrowed<T, Lease::dispatch>`
  is a plain pointer today. Planned: in debug builds an 8-bit lender
  epoch travels with the loan (one byte and no padding on 8-bit targets;
  the pointer's padding on 32-bit ones) and is compared on every access
  - a stale loan panics on the guilty instruction. Built together with
  the first host test that simulates preemption; not before.
- **HSM.** The FSM contract is HSM-ready (`unhandled` = future
  bubble-to-parent); parent pointers, bubbling and LCA entry/exit chains
  get built only when a real AO demands them.
- **A watchdog keeper.** The three strata kick their watchdog under two
  names and three contracts (design/kernel.md's table); the portable
  verb is a util concept one level above them, born with the first
  portable program that keeps a watchdog alive.
- **QK-style preemption (far horizon, probably not on AVR).** A
  preemptive non-blocking kernel would be an EXTENSION of the present
  one: the AO contract and the three delivery primitives unchanged,
  `Lease::dispatch` loans still correct (the borrower-precedes-lender
  order makes the borrower preempt the lender right at the post),
  `Lease::reply` loans ordering-independent. What would change: `post()`
  triggering the scheduler when it readies a higher AO, an ISR-exit
  hook on the Platform (PendSV-like, irreducibly target-specific), the
  loop as the idle context, time-event maturation revisited. It would
  be a SECOND kernel type beside `Tenuto`, named `Rubato`, chosen per
  board file. The discipline kept NOW so the door stays open: AOs
  share nothing but events.
- **C++ modules: considered, not now.** The prize would be macro
  isolation (`import brio.avrdx` would not leak `avr/io.h` above the
  target stratum), not build speed; the blocker is the language server.
  Revisit with the board files.

## Build, test, debug (the must-knows)

```bash
# Sibling CMake projects, PEERS (none is the repo root): one per target
# plus test/. cmake presets resolve against their own project dir -
# run cmake FROM that dir (or let bin/brio do it).
(cd test  && ctest --preset host)                                  # host tests (doctest); no hardware needed
brio check avrdx [name]         # every avrdx smoke TU compiles for all 8 DA/DB packages;
                                # neg/ TUs must FAIL (definition of done)
brio check samc21 [name]        # same for the samc21 stratum (E/G/J 18A headers)
brio check stm32g0 [name]       # same for the stm32g0 stratum (ALL TWELVE G0 headers, x1 + x0)
brio check ch32v00x [name]      # same for the ch32v00x stratum (one part today; util_all.cpp = the
                                # whole of kernel/ and util/ through WCH's gcc 15.2)
brio check ch32v203 [name]      # same for the ch32v203 stratum (ALL NINE parts of the series, both
                                # HPE ways; util_all.cpp = the whole of kernel/ and util/ over the
                                # ilp32 ABI)
brio check rp2040 [name]        # same for the rp2040 stratum (one chip: every header's verbs, util_all.cpp)
brio check stm32f4 [name]       # same for the stm32f4 stratum (ALL TWENTY-THREE F4 headers; the ladder
                                # refused by name where no manual was read)
brio check rp2350 [name]        # same for the rp2350 stratum - the one fixture that crosses TWO
                                # COMPILERS: every TU built four times (Cortex-M33 and Hazard3, each
                                # for the QFN-80 and the QFN-60), util_all.cpp through both
brio check all                  # every stratum above, in a row
brio prose [paths...]           # the prose net: no dates/process words/Doxygen tags in
                                # comments and docs, every cited path exists, ASCII only;
                                # "review" lines are claims of absence to re-read, not errors
brio gate [--against REF]       # THE BYTE-IDENTITY GATE: reference and working tree each built
                                # with mtimes pinned and build dirs wiped, images compared per
                                # preset, movers named (the release presets cli/gate.py's
                                # DEFAULT_PRESETS names: every build project, two of them on the
                                # ch32v00x - one per part - and two on the rp2350, one per architecture)
brio gate --tokens [--strings] FILE...   # a source token-identical to REF? (--strings: ignoring
                                # what string literals say) - the gate for a comments-only claim
(cd avrdx && cmake --build --preset avr128db48-release --target <app>)         # AVR release build (-Os)
(cd avrdx && cmake --build --preset avr128db48-release --target <app>-upload)  # flash via Atmel-ICE (UPDI)
(cd avrdx && cmake --build --preset avr128db48-debug --target <app>)           # AVR debug build, then F5
(cd samc21 && cmake --build --preset samc21j-release --target <app>)            # SAM release build
(cd samc21 && cmake --build --preset samc21j-release --target <app>-upload)     # flash via OpenOCD (SWD)
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>)          # STM32G0 release build
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>-upload)   # flash via OpenOCD (ST-LINK)
(cd ch32v00x && cmake --build --preset ch32v006k8-release --target <app>)          # CH32V00x release build (WCH gcc 15)
(cd ch32v00x && cmake --build --preset ch32v006k8-release --target <app>-upload)   # flash via WCH's OpenOCD fork (WCH-Link)
(cd ch32v203 && cmake --build --preset ch32v203c8-release --target <app>)          # CH32V203 release build (WCH gcc 15, ilp32)
(cd ch32v203 && cmake --build --preset ch32v203c8-release --target <app>-upload)   # flash it: `reset run` does NOT start the program here, `reset halt` + `resume` does
(cd rp2040 && cmake --build --preset rp2040-release --target <app>)              # RP2040 release build
(cd rp2040 && cmake --build --preset rp2040-release --target <app>-upload)       # flash via OpenOCD (the Debug Probe, CMSIS-DAP)
(cd stm32f4 && cmake --build --preset stm32f429zi-release --target <app>)        # STM32F4 release build (hard-float)
(cd stm32f4 && cmake --build --preset stm32f429zi-release --target <app>-upload) # flash via OpenOCD (the board's ST-LINK)
(cd rp2350 && cmake --preset rp2350-arm-release)                                 # configure (once, or after adding an app)
(cd rp2350 && cmake --build --preset rp2350-arm-release --target <app>)          # RP2350 release build, Cortex-M33
(cd rp2350 && cmake --build --preset rp2350-riscv-release --target <app>)        # the SAME source, Hazard3 RISC-V
(cd rp2350 && cmake --build --preset rp2350-arm-release --target <app>-upload)   # flash via Raspberry Pi's OpenOCD fork: a
                                                                                 # rescue over the debug port, then program as
                                                                                 # core 0 of the Arm pair, then reset run
# apps are auto-discovered from <project>/src/apps/*.cpp - plus
# experiments/*/{avrdx,samc21}/*.cpp, each experiment's per-arch app
# halves - at every configure; no generation step; a new/removed app
# or a changed "// build: opt = value" line takes effect on the next
# configure

# The bench and the repository have ONE command, bin/brio (put bin/ on
# the PATH); its guts are the cli/ package. The verbs:
brio list                  # serial devices, USB probes, the bench manifest
brio flash A test_avr_pin  # cmake --build --target <app>, then avrdude/UPDI
brio flash C test_samc_dma # ... or OpenOCD/SWD - the BOARD TYPE decides both
brio flash E console       # ... or OpenOCD/ST-LINK (a Nucleo in the example manifest)
                           # the project to build in and the flash mechanism
brio run C z               # drive the console, judge "ALL: N pass, M fail"
brio console A             # device path + speed, for your own monitor
brio duo A:a B:script.txt  # instrument peer scripted, then the DUT
brio stress --letters ...  # the host end of the UART suites
brio fuses A bootsize=128  # read/write fuses over UPDI (fuses are
                                             # provisioning: UPDI-only, survive reflash)
```

- The multi-board bench, three separate concerns (detail:
  `docs/boards/README.md`): BUILD = one CMake target per app x board TYPE
  (`// build: boards = db28,db32,db48` in the app header; `db48` is the
  default when the line is absent; a configure targets exactly one
  package, so switching `configurePreset` switches which apps' targets
  exist; `// build: groups = abg,cdf` splits a suite into one image per
  group on a board type its project lists as splitting - the CH32V003,
  and the CH32V203's four 32 KB parts - and changes nothing elsewhere),
  IDENTITY = the manifest `cli/bench/bench_boards.py` (which board
  sits where, its console by `/dev/serial/by-path` because the CH340s
  have no USB serial, its programmer), ORCHESTRATION = `bin/brio`.
  Never a target per physical board. `family_probe` carries the matrix
  and is the first firmware for a new board.

- Toolchains: self-built avr-gcc 16.2 at `/sw/avr`
  (`avrdx/cmake/toolchain-avr.cmake`), arm-none-eabi-gcc 16.2 at
  `/sw/arm-none-eabi` (`samc21/cmake/toolchain-arm.cmake`) and OpenOCD
  0.12.0 at `/sw/openocd` (`/sw/src/build-openocd.sh`, from the release
  tarball: CMSIS-DAP on hidapi + ST-LINK; the manifest's OPENOCD, the
  two ARM projects' `*_OPENOCD` cache variables and launch.json all
  point there), each by absolute path; never a system-packaged one.
  A second OpenOCD built from git at a pinned commit
  (`/sw/openocd-git-bedefa2`, `/sw/src/build-openocd-git.sh`) writes the
  RP2040 boards whose flash chip the release does not know, named per
  programmer in the manifest; and a THIRD, Raspberry Pi's own fork
  (`/sw/openocd-rpi-acff23f`, `/sw/src/build-openocd-rpi.sh`, the
  manifest's `RPI_OPENOCD`), is the RP2350's and nobody else's: the
  release does not know that chip at all and the git build cannot debug
  Hazard3, its RISC-V target driver taking no DAP, so the fork is the
  one build that examines all four cores and flashes from either side.
  The RP2350's Hazard3 half is built by a self-built upstream
  riscv32-unknown-elf-gcc 16.2 at `/sw/riscv32-unknown-elf`
  (`/sw/src/build-riscv32-elf.sh`, `rp2350/cmake/toolchain-riscv.cmake`)
  - NOT WCH's compiler, which the two QingKe families use and which
  carries a vendor extension this core has not; its Arm half takes the
  same arm-none-eabi-gcc as the other ARM targets, with the hard-float
  ABI. The release build stays everyone else's. Never add
  `-mrelax` on AVR (PyAvrOCD refuses the ELF).
  No `-flto` (never added, so nothing to strip) and no `-DF_CPU` (never
  added either: the clock rate has one truth, `Clock::hz`; avr-libc's
  util/delay.h / setbaud.h do not compile here, on purpose - use
  `brio::delay_us(clock, us)`).
- Atmel-ICE: cable in the **AVR** port, not SAM (symptom: Vtarget
  ~1.71 V, sign-on `0xa0`).
- Debugging: PyAvrOCD as GDB server, launched by cppdbg itself
  (`.vscode/launch.json`'s `debugServerPath`/`debugServerArgs`, port
  40044); effectively ONE free hardware breakpoint (GDB borrows the
  second); `--breakpoints hardware` refuses extras instead of wearing
  flash; line breakpoints need `-fno-inline` (GCC 16 DWARF caveat) -
  hence the Debug config's flags in `CMakeLists.txt`'s `avr_add_app()`.
  `monitor ioregister <name>` reads/writes peripheral registers; the
  same SVD also drives the mcu-debug Peripheral Viewer extension's
  panel (`svdPath` in `launch.json`).
- The bench board: 24 MHz crystal on PA0/PA1 (not GPIO) -
  `Clock<ClockSource::crystal, 24'000'000>` - no 32k crystal (do not
  enable XOSC32K), serial on USART2 ALT1 PF4/PF5.
- Full detail and rationale: `docs/avrdx/README.md`,
  `docs/host/README.md`, `docs/boards/README.md`, `docs/probes/README.md`.

## Layout

```
avrdx/                   the AVR build project (a PEER of samc21/ and test/ -
                         the repo root is not a CMake project):
  CMakeLists.txt           app auto-discovery ("// build:" header comments),
                           avr_predefines() (clangd's -mmcu macro delta),
                           avr_add_app() (flags, .hex/.lst/.map, per-app
                           -upload target, FLMAPLOCK/build-id defsyms)
  CMakePresets.json        one configure+build preset pair per AVR128DB
                           package x {release, debug}; binaryDir under the
                           shared ../build-cmake/
  cmake/toolchain-avr.cmake  the cross toolchain file (avr-gcc 16.2 at /sw/avr)
  cmake/avr-mcus.cmake     package -> mcu name table (128K flash / 16K RAM each)
  src/apps/<app>.cpp       one main() per app (ISR vector bindings live HERE);
                           "// build: <option> = <value>" header lines
                           ("boards = db28,..." gates which package builds it,
                           default db48 only; "flmap_lock = 0" opts out of the
                           FLMAPLOCK default; anything else is just metadata
                           for bin/brio, e.g. "monitor_speed = 115200")
  src/glue/                build invariants compiled into EVERY image (every
                           avr_add_app() call lists ivsel_boot.cpp alongside
                           the app's own source - the .init3 IVSEL store,
                           vectors at BOOT start)
  svd/avr128db48.svd       the debug Peripheral Viewer's register map
samc21/                    the SAM C21 build project, same shape (CMakeLists +
                         presets + cmake/toolchain-arm.cmake + ld/ linker
                         script + src/apps + src/glue startup crt + svd/) -
                         its own header comments are the reference
stm32g0/                 the STM32G0 build project, same shape again (the
                         part number selects define + ld + crt + board
                         name: presets for the G0B1RE, the G071RB and the
                         G031K8, ld/<part>.ld and src/glue/startup_<header>
                         .cpp for each; ST-LINK upload target; svd/ with
                         ST's SVD per part)
ch32v00x/                the CH32V00x build project, the fifth of the shape
                         (cmake/toolchain-riscv.cmake on /sw/wch-riscv,
                         CH32V00X_MCU naming the part and deriving from it
                         the PART DEFINITION (CH32V006 / CH32V003, what
                         device.hpp asks - no vendor header, so the build
                         states the part), the ISA (rv32ec_zmmul_xw / rv32ec_xw,
                         each part's full ISA under WCH's gcc, xw theirs
                         alone) and ld/<part>.ld; two preset pairs, one
                         per part; src/glue/startup_ch32v00x.S - the table
                         whose first word is an INSTRUCTION and whose
                         handler names are the project's own, two entries
                         shorter on the CH32V003 - and the upload target on
                         WCH's OpenOCD fork, the probe named by serial; the
                         GROUP axis: on the CH32V003 a suite with a
                         "// build: groups" line builds as one image per
                         group, <app>-<n>, the crt PAINTS the free RAM so a
                         suite reports how much stack it never touched
ch32v203/                the CH32V203 build project, the seventh of the shape:
                         cmake/toolchain-riscv.cmake on the same /sw/wch-riscv
                         but the ilp32 ABI and the full register file, a PART
                         TABLE (cmake/ch32v203-parts.cmake: the nine parts of
                         the series, each with its part definition, its
                         memories and its board type) instead of substring
                         arithmetic, ld/<part>.ld and src/glue/startup_ch32v203.S
                         - the table whose first word is an INSTRUCTION again,
                         with a TAIL THAT IS THE DEVICE CLASS'S
                         - ld/<part>.ld and a release preset for each of the nine
                         parts, the C8's the only debug one; the C6 preset (the
                         F6's memories, the C8's bonding) is the 32K/10K tier's
                         LINK GUARD, because the F6 bonds neither USART1 nor the
                         board's LED; the GROUP axis, as on the CH32V00x:
                         the four 32 KB parts (the C6, F6, G6 and K6,
                         read off the part table by their memories and
                         never listed twice) build a suite with a
                         "// build: groups" line as one image per group,
                         <app>-<n>, and every other part builds it whole
stm32f4/                 the STM32F4 build project, the sixth of the shape: a
                         PART TABLE (cmake/stm32f4-parts.cmake: the part number
                         -> ST's irregular device define, the crt stem, the
                         board type) instead of substring arithmetic; presets
                         for the F429ZI, the F446RE and the F411CE; the hard-
                         float flags; ld/<part>.ld (the F429's CCM a named
                         region nothing is placed in); src/glue/startup_stm32f4
                         {29,46,11}.cpp - ST's handler names, the FPU's CPACR
                         enabled before .data, holes where another part has a
                         peripheral; svd/ with ST's SVD per part
rp2350/                  the RP2350 build project, and THE ONLY ONE WHOSE AXIS
                         IS THE COMPILER: one chip, two processor architectures,
                         so cmake/toolchain-arm.cmake (arm-none-eabi, hard float,
                         a Cortex-M33) and cmake/toolchain-riscv.cmake (upstream
                         riscv32-unknown-elf on /sw/riscv32-unknown-elf, ilp32,
                         the -march string a cache variable) are two configures
                         of the SAME sources, each setting RP2350_ARCH, from
                         which the flags, the crt, the include roots and the
                         family check follow - four presets, arm and riscv x
                         release and debug. Besides the architecture a configure
                         targets a FLASH GEOMETRY (RP2350_FLASH_KB -> ld/) and a
                         PACKAGE (RP2350_PACKAGE, a for the QFN-60 and b for the
                         QFN-80, handed to the stratum as the pin count, so a pad
                         the package has not got is a COMPILE error and an image
                         built without it refuses the extra pads at run time
                         against SYSINFO.PACKAGE_SEL); src/glue/startup_rp2350
                         _arm.cpp and _riscv.S - two crts binding THE SAME
                         handler names, one through a Cortex-M vector table and
                         one through Hazard3's own dispatch, each placing the
                         twenty-byte IMAGE_DEF block that tells the bootrom which
                         architecture to enter, THERE BEING NO SECOND STAGE;
                         svd/RP2350.svd; and an -upload target that is the
                         STATE-INDEPENDENT VERB - a rescue, then programming as
                         core 0 of the Arm pair, then a reset
host/                    the HOST build project - programs that RUN, as
                         opposed to the suites in test/ which run and exit.
                         The three axes finally agree (brio/host/, docs/host/,
                         host/); one main() per src/apps/<app>.cpp, the same
                         "// build:" grammar as the cross projects, and
                         NOTHING registered with ctest - an interactive
                         program cannot be a test. Flags identical to test/'s
                         so a program and the suites cannot disagree about the
                         language. First app: supply_panel, the front panel of
                         a bench supply, which is where the drawing library,
                         the published framebuffer, the panel a viewer writes
                         and the quadrature decoder first meet
test/CMakeLists.txt      the host test project (independent - one CMake
                         configure has exactly one compiler):
                         one executable + ctest entry per test_*/main.cpp
test/CMakePresets.json   the "host" configure/build/test preset (native g++, UBSan)
test/test_*/main.cpp     host unit tests (doctest), cd test && ctest --preset host
test/family_samc21/        samc21 family smoke TUs + neg/, brio check samc21 runs them
third_party/doctest/     vendored doctest.h (MIT, upstream doctest/doctest)
third_party/samc21-dfp/  vendored Microchip.SAMC21_DFP include tree (Apache-2.0)
third_party/cmsis-device-g0/  vendored ST cmsis-device-g0 v1.4.5 Include/ (Apache-2.0)
test/family_stm32g0/     stm32g0 family smoke TUs + neg/, brio check stm32g0
test/family_ch32v00x/    ch32v00x family smoke TUs + neg/, brio check ch32v00x
                         (util_all.cpp: every kernel/util header over the platform)
test/family_rp2040/      rp2040 family smoke TUs + neg/, brio check rp2040 (one
                         chip, one header: every verb of every header, util_all.cpp)
test/family_stm32f4/     stm32f4 family smoke TUs + neg/, brio check stm32f4 (ALL
                         TWENTY-THREE F4 headers; the reset rate everywhere, the
                         ladder-dependent rates where the reserve knows the ladder)
test/family_ch32v203/    ch32v203 family smoke TUs + neg/, brio check ch32v203
                         (every part of the series, both HPE ways; util_all.cpp)
test/family_rp2350/      rp2350 family smoke TUs + neg/, brio check rp2350 - the
                         one fixture that crosses TWO COMPILERS: every TU built
                         four times (Cortex-M33 and Hazard3 x the two packages),
                         a negative built for the package its refusal is about
                         (the `qfn60_` prefix names the smaller one), and
                         util_all.cpp through both compilers
test/test_pl011/, test/test_pl022/, test/test_dw_apb_i2c/
                         the IP strata's host suites: each driver against its
                         simulated chip (brio/host/sim_pl011.hpp and its two
                         siblings), so the block's own logic is judged off the
                         silicon it was extracted from
test/test_sha256/        util/sha256.hpp against FIPS 180-4's own vectors, at
                         compile time and at run time, plus the tail the
                         hardware accelerators are owed
third_party/cmsis-device-f4/  vendored ST cmsis-device-f4 v2.6.9 Include/ (Apache-2.0):
                         every F4 part's header, the umbrella stm32f4xx.h
third_party/cmsis-core/  vendored ARM CMSIS-Core headers (Apache-2.0), core_cm33.h
                         among them - the RP2350's Arm half reads it
third_party/pico-sdk/    vendored pico-sdk 2.3.1 subset (BSD-3), TWO include roots
                         because the two chips' files have the same names: the
                         RP2040's CMSIS device header and hardware/regs bit-field
                         headers at the top, the RP2350's under rp2350/ - plus
                         rp2350/no_core/core_cm33.h, this project's own stub for
                         the header the RISC-V build must not take. Never the SDK
                         runtime; each chip's SVD is its build project's (rp2040/
                         svd/, rp2350/svd/)
bin/brio                 THE ONE COMMAND of the bench, dispatching on its first
                         argument; put bin/ on the PATH
cli/                     its guts, a Python package: main.py dispatches on the
                         verb; gate.py, prose.py and check.py need no board
  gate.py                `brio gate`, the byte-identity gate over the images and
                         the token-identity check over sources
  prose.py               `brio prose`, the prose net (see the definition of done)
  check.py               `brio check <stratum> [filter]` over cli/checks/, the
                         four family compile fixtures kept as shell scripts
                         (zero CMake coupling, they call the cross compiler
                         directly)
  bench/                 THE BENCH HALF: the verbs that need a board
    verbs.py             the argparse front of list / flash / run / console /
                         duo / fuses
    common.py            what every bench verb needs: the manifest, BOARD_TYPES
                         (a board type -> its project, preset, mcu and flash
                         mechanism: db* -> avrdx/avrdude/UPDI, c21j ->
                         samc21/OpenOCD/SWD, g0* -> stm32g0/OpenOCD/ST-LINK,
                         v006k8/v003f4 -> ch32v00x/WCH's OpenOCD fork/WCH-Link,
                         v203c8 -> ch32v203/the same fork/the same probe on two
                         wires, pico/picow/weact2040 -> rp2040/OpenOCD/the Debug
                         Probe, f429zi/f446re/f411ce -> stm32f4/OpenOCD/an
                         ST-LINK, and weact2350b + weact2350b-rv -> rp2350/
                         Raspberry Pi's OpenOCD fork/the Debug Probe: ONE BOARD
                         UNDER TWO TYPES, because the type carries the preset
                         and on this chip the preset carries the ARCHITECTURE),
                         the per-project app rosters build-cmake/apps_<project>
                         .json (each project writes its own at
                         every configure - separate files because app NAMES
                         COLLIDE across the trees), the console paths
    manifest.py          loads the bench MANIFEST from private/bench_boards.py
                         if it exists, else cli/bench/bench_boards.py: the
                         physical boards on the desk (type, console by-path,
                         programmer) - not a target list; no verb imports
                         the file by name
    bench_boards.py      the manifest in the repository
    flash.py             `brio flash`: build, then avrdude / OpenOCD / the
                         ST-LINK's mass-storage flasher by board type, with
                         the flash-heap preflight on the AVR - and the
                         RP2350's own mechanism, TWO OpenOCD sessions in
                         order: a DAP with no target behind it setting and
                         clearing the RP-AP's rescue-restart bit (the one
                         reset that works with the clocks stopped and the
                         core domain down), then the programming as core 0
                         of the Arm pair and a reset run that hands the chip
                         to whatever architecture the new image names
    fuses.py             `brio fuses`: the AVR FUSE bytes over UPDI, the SAM
                         user row over SWD; refuses what a type cannot do
    console.py           the suites' console protocol and `brio run`,
                         `brio console`, `brio duo` over it
    stress.py            `brio stress`, the host end of the UART suites: the
                         same xorshift the firmware generates, plus the baud
                         and frame changes only an OUTSIDE sender can make
experiments/             one SELF-CONTAINED directory per cross-cutting bench
                         experiment (deliberately not "examples" - no
                         maintenance promise): both
                         architectures' app halves (<name>/avrdx/*.cpp and
                         <name>/samc21/*.cpp, globbed by the respective build
                         projects; app names unique per arch), the shared
                         wire-protocol header beside them, its own README
                         (rationale, pre-registered predictions, wiring,
                         protocol, log format - docs/ never references it),
                         python driver + analysis, logs/ git-ignored.
                         energy/ = the clock-strategy energy experiment
                         (DynamicClock-deferral verdict; the SAM as
                         stimulus + judge + meter for an AVR DUT)
docs/                    README (map + rules), design/, one folder per target
                         (avrdx/, samc21/, stm32g0/, stm32f4/, ch32v00x/,
                         ch32v203/, rp2040/, rp2350/, host/), one per shared
                         stratum (cortexm/, pl011/, pl022/, dw_apb_i2c/),
                         boards/, probes/
brio/.clangd             per-stratum clangd routing: the framework default is
                         the host database; a target stratum carries a fragment
                         of its own (brio/<arch>/.clangd, and one in the project
                         dir beside it) pointing at that architecture's own
                         database, so a header always
                         parses with its own compiler regardless of CMake
                         Tools' active project. Each CMake project carries its
                         own fragment too - test/.clangd on build-cmake/host,
                         host/.clangd on build-cmake/host-apps - and a project
                         listed in .vscode/settings.json's
                         cmake.sourceDirectory array is what the status
                         bar's project picker offers
brio/                    the framework, one directory per stratum:
  kernel/                pure kernel logic - includes NOTHING of brio
    platform.hpp           Platform concept (CriticalSection, idle,
                           break_here, now, ticks_per_second, atomic_width,
                           panic_record) + PanicRecord (hosted by the platform)
                           + the OPTIONAL idle_until(deadline) a platform on a
                           timebase that counts through sleep may offer
    active_object.hpp      ActiveObject concept: what Tenuto requires of an
                           AO (Event, queue, init, dispatch) + the informal
                           half of the contract
    event_queue.hpp        EventQueue<E, depth, P>: per-AO MPSC queue,
                           saturating overflow counter, optional pop
    fsm.hpp                Fsm<Derived, Alts...> flat HSM-ready machines
                           (Entry/Exit reserved, transition chaining, start);
                           match(e, lambdas...) + Overloaded
    post.hpp               post<Ao>(ev), publish(Subscribers<...>{}, ev),
                           ReplyTo<Payload> / reply_to<Ao, Payload>()
    borrowed.hpp           Lease {dispatch, reply}, Borrowed<T, Lease>:
                           pointer payloads with the lease in the type
    time.hpp               constexpr tick conversions (ceil, never early)
    time_event.hpp         TimeEvents<P> armed list + TimeEvent<P, Ao, Ev>
                           (drift-free periodics, wrap-safe, RAII disarm);
                           ticks_to_next() = how long until the next
                           deadline, the power model's one kernel question;
                           next_deadline() = its absolute tick, the loop's
                           question for a tickless platform
    tenuto.hpp             Pack<Aos...> (index, lends_ok) + Tenuto<P, Aos...>:
                           the cooperative run-to-completion kernel (a note
                           held for its full value) -
                           init_all/step/idle_if_empty/run, static_asserts
                           borrowers before lenders; idle_if_empty takes the
                           optional idle_until branch by requires
    panic.hpp              panic<P, Reporter>(), PanicCode, HaltReporter,
                           take_panic_record<P>()
  util/                  pure services - may include kernel/, never a target
    stream.hpp             ByteSink / ByteSource / ByteTransport concepts
    print.hpp              print(sink, ...) + hex/fixed/sci wrappers, crlf;
                           extend via print_one + ADL
    timestamp.hpp          TimeStamp (ms fraction)
    wire.hpp               constexpr big-endian load/store (16/24/32, be24s)
    analog.hpp             adc_mv/adc_mv_signed, dac_code/dac_mv: pure counts<->mV
                           arithmetic (host-tested); Ref/ref_mv are each target's
    analog_sampler.hpp     AnalogSampler<Converter, P, Subscribers, inputs...>: the
                           owner AO walking a list of inputs, Sampled in (ISR glue,
                           labelled by the converter's selected code), AnalogSample
                           published; software pace or any hardware generator
    meter_sampler.hpp      MeterLatch<T, P, id> (the one-cell bridge out of a
                           capture ISR: store/take/missed) + MeterSource
                           concept + MeterSampler<P, Subscribers, Sources...>:
                           the AO that paces PUBLICATION, not capture -
                           MeterSample per FRESH source, a stale one is silent
    block_stream.hpp       BlockSource/BlockPlayer concepts (blocks, not
                           DMA: caller-owned buffers, overruns skip laps,
                           the accounting IS the API) + BlockRelay<P, Subs,
                           Sources...>: the event-driven AO that lends each
                           filled block for ONE dispatch (Lease::dispatch,
                           LendsTo-checked) and returns it on its next -
                           every block delivered exactly once, the opposite
                           economy of MeterSampler's discard-stale
    input_scanner.hpp      ScannedInput concept (read() = active) +
                           InputScanner<P, Subscribers, ScanConfig, Inputs...>:
                           periodic poll, N-sample debounce, InputEdge on each
                           flip, silent at startup; polarity is the input's
    quadrature.hpp         quadrature_step() (the 4x4 transition table as
                           an optional: nothing means a state was missed) +
                           Turned{detents} + QuadratureConfig (counts per
                           detent = a FACT OF THE PART; the lost-step policy,
                           conservative by default) + Quadrature<P, Subs, A,
                           B, config>: the POLLED SOFTWARE DECODER over two
                           ScannedInputs, right on every target because a
                           knob does not justify a timer. The button is
                           InputScanner's; the accumulator never leaves
    trace.hpp              Trace<N, P, enabled>: ring of {t, tag, arg} stamps
                           from ISRs or the loop, overwrite-oldest, dump(sink);
                           the disabled specialization is EMPTY - no storage,
                           no code
    clock.hpp              ClockUser concept, clock_hz(clock), clock_follows:
                           the target-independent clock contracts
    power.hpp              the power model: SleepDepth ladder, SleepSite
                           concept (arm only - the kernel loop's idle path
                           still does the sleeping), PowerManager AO with
                           the vote round, PowerLock standing restrictions,
                           the ticks_to_next deadline guard, WakeReport
    pwm_channel.hpp        PwmChannel concept: max + duty(v), the role-level
                           "one dimmable output" (Pin satisfies it, max 1)
    rgb_lamp.hpp           RgbLamp<R, G, B> over three PwmChannels, levels
                           scaled per channel max; Rgb triple
    crc.hpp                crc16_byte/crc16 (the record checksum) and
                           crc32_ethernet* (CRC-32/MPEG-2: the one function
                           a hardware block with the Ethernet polynomial
                           wired in computes, and what its bench suite
                           judges it against) - both bitwise, no table
    sha256.hpp             SHA-256 in constexpr C++ (FIPS 180-4), here for
                           crc.hpp's reason: an accelerator for this function
                           has nothing to configure, so what it computes has
                           one home. sha256() over a message, and
                           sha256_tail() - the padding an accelerator that
                           digests whole blocks and pads nothing is owed, so
                           the arithmetic that decides where a message ends is
                           written once and shared by both paths. No HMAC, no
                           key derivation: they are built ON this and are not
                           here
    nv_record.hpp          NvStore/NvPacedStore concepts + NvRecord<T, S>
                           (magic+version+CRC-16 header, store() writes
                           only changed bytes)
    nv_writer.hpp          NvWriter AO: one byte per ready interrupt,
                           BusMaster-style pending FIFO + ReplyTo
    nv_heap.hpp            FlashMedia concept + NvHeap<Media, max_blocks,
                           map_pages>: flash block allocator - ping-pong
                           map pair under FLASHEND, headerless payload
                           blocks, survival-aware mount, alloc/append/
                           seal/rewrite/find, no free
    nv_journal.hpp         NvJournal<Media, max_ids, max_payload,
                           half_pages> over the SAME FlashMedia: small
                           values in flash where there is no EEPROM -
                           two halves ping-ponging wholesale, entries
                           appended cell by cell with a seq and a CRC,
                           latest-seq-wins, a READ-ONLY mount that only
                           reports a torn tail or an unfinished
                           collection, and the PANIC RESERVE an ordinary
                           save always leaves so save_reserved() is one
                           bounded program with no erase; + JournalPanic
                           (the reporter over it, reaching the journal
                           through a reference template parameter
                           because a journal is an object)
    persistent_panic.hpp   PersistentPanic<S>: panic Reporter into an
                           NvStore + boot-side take()
    ring.hpp               Ring<T, size, P> SPSC FIFO, lock-free when the
                           index fits P::atomic_width, guarded otherwise
    inbox.hpp              THE BRIDGE BETWEEN TWO KERNELS ON TWO CORES
                           (design/kernel.md sec. 12): Inbox<Ao> (a ring of
                           Ao::Event written by the sending core under its
                           own guard, read by the receiving core, a fence
                           pair and no shared counter), send<Ao>(ev) beside
                           post, Inboxes<Aos...>::isr() (the doorbell's ISR
                           body: bells popped FIRST, then every inbox drained
                           into local posts), send_reply_to<Ao, Payload>();
                           nothing here is instantiated on a single-core
                           target
    usb/device.hpp         the USB device stack: UsbSetup and chapter 9's
                           codes, the descriptors as constexpr bytes
                           (usb_device_descriptor, usb_configuration_head,
                           usb_interface/endpoint/string_descriptor,
                           usb_concat), the UsbController and UsbClass
                           contracts, UsbDevice<Controller, Descriptors,
                           Classes...> - the control-endpoint machine as
                           an ISR body (the stages, the address after its
                           status, the configuration claiming the classes'
                           endpoints, the routing by interface and endpoint)
    usb/cdc.hpp            UsbCdcAcm<Controller, P, interface, ep_notify,
                           ep_data, rx, tx>: the serial-port class as a
                           ByteTransport (write_byte / write / read_byte /
                           tx_idle, the rings, the OUT endpoint armed only
                           while a packet fits - USB's NAK as flow control -,
                           the zero-length packet closing a full run), the
                           line coding and DTR reported, not obeyed
    testbench.hpp          TestBench<Sink, max_letters>: the bench suite
                           grammar in one place - letter registry, verdict
                           lines, per-letter tally and the ALL: total
                           bin/brio parses
    serial_port.hpp        SerialPort<Transport, P, LineSink>: RX bytes ->
                           LineReceived (Lease::dispatch loan, LendsTo)
    bus_master.hpp         BusMaster<Bus, P, depth, Policy>: bus arbiter
                           (pending FIFO, reject-when-full, ReplyTo
                           completion, BusDone, PrepareSleep voter) + the
                           completion-policy hook (BusAction pass/retry,
                           BusPassThrough default declaring never_retries -
                           the opt-out that makes the hook cost zero)
    spi_bus.hpp            SPI vocabulary: SpiBus/SpiDone/spi_*
    i2c_bus.hpp            I2C vocabulary: I2cBus/I2cDone/i2c_* + outcomes
    can.hpp                the CAN vocabulary three controllers share:
                           CanFrame (classic: the natural id, the length in
                           bytes, the receive-side filter index and stamp),
                           CanTiming in human units + CanTimingLimits (a
                           controller's register widths as a VALUE) +
                           can_timing_search (exact, never rounded; a
                           stratum binds its limits into its own
                           can_timing_for), CanError (the LEC codes),
                           CanErrorState/can_error_state, CanErrorCounters;
                           no filters and no bus AO (design/can.md)
    proto/line_parser.hpp  LineAssembler + console/SCPI parsers +
                           CommandRouter<Sink>
  avrdx/                 everything that knows avr/io.h (AVR DA/DB)
    platform.hpp           AvrPlatform (idle() sleeps in IDLE unless a
                           deeper mode is already armed - see sleep.hpp)
    clock.hpp              CLKCTRL: resources Oschf/Osc32k/Xosc32k/Xoschf/Pll/
                           MainClock/ClockFailure (typed register views) +
                           tasks Clock<source, hz, div> (constexpr hz, the ONE
                           rate truth: no F_CPU) and DynamicClock<Boot, Users...>
                           (set<hz>()/set(hz) rebases the users then switches;
                           discrete-rate surface rate_count/rate_hz/rate_index)
    delay.hpp              delay_us(clock, us) "at least", never a division at
                           wait time: folded cycles when constant, per-rate
                           Q4.12 fixed point otherwise (dynamic clocks
                           dispatched by rate index); delay_cycles,
                           delay_us_runtime (stored-byte), cycles_per_us
    reset.hpp              RSTCTRL + WDT: Reset (RSTFR flags read-and-clear,
                           software reset) and Watchdog (PERIOD/WINDOW, WDR,
                           SYNCBUSY, the one-way LOCK)
    nvm.hpp                NVMCTRL: Nvm (flash ELPM/SPM 24-bit, page/multi-
                           page erase with the whole-range errata guard,
                           EEPROM writes + EEREADY ISR body, USERROW,
                           protections, vectors_in_boot), FlashLayout,
                           Sigrow, EepromStore (the util NvStore backend)
    nvm_flash.hpp          NvmFlash: the FlashMedia backend over Nvm -
                           zones from the *_load_* linker symbols with the
                           BOOT-section floor, build id from the link
                           defsym (CMakeLists.txt's avr_add_app())
    sleep.hpp              SLPCTRL: Sleep (arm/disarm/sleep/enter, the three
                           modes, the errata-2.2.4 NOP discipline), Vreg
                           (PMODE under CCP, HTLLEN with the TWI/CCL
                           interlock enforced) and AvrSleepSite, the
                           util/power.hpp adapter (depth ladder -> SMODE)
    pin.hpp                Pin<'A',5> compile-time GPIO (also a PwmChannel,
                           max 1) + PinRef descriptor + PinSet<Pins...> mask
                           + port_by_letter/pinctrl_of (run-time port lookup)
    usart.hpp              USART: Usart<n> resource (routes incl. NONE with
                           full teardown, every frame format, receiver modes,
                           the baud arithmetic, STATUS W1C, ISR bodies) +
                           tasks Uart<n, Route, rx, tx> (interrupt-driven
                           transport), OneWire, Rs485, SyncHost/SyncClient,
                           MspiHost, IrdaLink, AutoBaud
    spi.hpp                SPI: Spi<n> resource (the per-package route table
                           with the errata gate, both roles incl. the host
                           demotion, the seven rates and their chooser, both
                           INTFLAGS layouts, two ISR bodies) + tasks
                           SpiHost<n, route> (two-phase descriptor, per-byte
                           ISR pump or polled, CS owned by the engine,
                           optional SCK ceiling) and SpiClient<n, route>
    twi.hpp                TWI: Twi<n> resource (the per-package route table
                           with its dual pin pairs, the three errata as code,
                           the chapter's baud arithmetic with the bus edges as
                           arguments, host and client halves, two ISR bodies)
                           + tasks I2cHost<n, route> (the I2C transfer engine
                           driven by util/i2c_bus.hpp) and I2cClient<n, route,
                           on_dual_pins>
    tca.hpp                TCA: Tca<n> resource (normal mode: PER, three
                           buffered CMP, waveform modes, event inputs A/B,
                           commands, ISR bodies) + tasks TcaPwm<n, port>
                           (split mode, six 8-bit PwmChannels), TcaPwm16,
                           FrequencyGenerator, Heartbeat, EventCounter
    tcb.hpp                TCB: Tcb<n> resource (eight modes, event clock/
                           capture, cascade, routes) + tasks PeriodicTick,
                           Timeout, OneShotPulse, PulseCounter,
                           CascadedCounter, Frequency/PulseWidth/DutyMeter,
                           Pwm8
    tcd.hpp                TCD: Tcd<0> resource (the whole chapter with its three
                           synchronization disciplines enforced by the verbs, the
                           per-package route table, the input-mode validity table
                           and the errata refusals, 12-bit compares/captures,
                           delay block, dither, two ISR bodies) + task
                           TcdPwm<route> (the complementary pair with dead time;
                           the PLL's only consumer)
    ccl.hpp                CCL: Ccl (block, sequencers, one vector) + Lut<n>
                           (three typed inputs, lut_truth(), filter/edge,
                           clock, pin, sense) + ToggleFlipFlop<pair>
    ac.hpp                 AC: Ac<n> (inputs, DACREF via Vref::ac, hysteresis,
                           power, pin/event/interrupt, window) + Threshold,
                           Window
    rtc.hpp                RTC: RtcClock (the CLKSEL both functions share),
                           Rtc counter (prescaler, PER/CMP/CNT with the busy
                           waits, OVF/CMP flags + ISR body, CALIB trim with the
                           DIV2 rule, RUNSTDBY/DBGRUN) + Pit (PERIOD, PITEN,
                           PI flag + ISR body)
    ticker.hpp             BasicTicker<tps> timebase over Pit (Ticker = 1024);
                           owns the block's clock select
    userrow.hpp            board_id(): the USERROW identity label (written
                           once over UPDI, survives chip erase)
    vref.hpp               Ref + ref_mv (this silicon's levels) + Vref::adc0/dac0/ac
    dac.hpp                Dac<0>: init(DacConfig), set(code)/set_mv - actuator
    adc.hpp                Adc<0>: init<cfg>()/init(cfg)/reconfigure, AnalogIn<Pin>
                           + AdcInput, select/start/stop/read/result, window,
                           resrdy()/wcmp() ISR bodies, start_on(channel),
                           ClockUser (rebase keeps CLK_ADC in range)
    evsys.hpp              EVSYS: EventChannel<n> (source/off/pulse), generators
                           EvPitDiv/EvRtcOvf/EvRtcCmp/EvPin/EvTcaOvf/EvTcaCmp/
                           EvTcbCapt/EvTcbOvf/EvLut/EvAcOut (code + legality),
                           users EvOut<Pin>/EvAdc0Start/EvTcaCntA/B/
                           EvTcbCaptIn/CountIn/EvLutIn + EventUserBase
                           (listen/unlisten); concepts EventGenerator/
                           EventUser; tables on demand
  samc21/                  everything that knows sam.h (SAM C21, Cortex-M0+)
    nvic.hpp               "sam.h" + cortexm/nvic.hpp (the guard and Nvic live there)
    platform.hpp           SamPlatform (idle takes whatever PM.SLEEPCFG holds -
                           SCR.SLEEPDEEP is never written - with erratum
                           1.8.13's guard around a standby WFI; BKPT, .noinit
                           breadcrumb, atomic_width 4)
    ticker.hpp             cortexm/ticker.hpp's BasicTicker (Ticker = 1000 Hz,
                           advance(n) the standby resync's landing point) +
                           SysTickInterruptGuard, erratum 1.8.13's workaround
                           in the file that owns the register. The tick stops
                           in standby (SysTick rides the CPU clock) and the
                           TIMED SITE in sleep.hpp is what lifts that
    clock.hpp              OSCCTRL/GCLK/MCLK: Oscctrl (the block, the shared
                           IRQ 0, the CFD event code), Osc48m, Xosc (crystal
                           or external clock, the mandatory gain, the startup
                           counter, the clock failure detector + safe clock),
                           Fdpll (three references, dpll_ratio_for in
                           sixteenths, dco_hz/output_hz, the lock timer's own
                           channel, three live errata as code), Gclk<n> +
                           GclkChannel + Mclk, and the task Clock<internal,
                           hz>; calls nvm.hpp's FlashWaitStates around a
                           change
    nvm.hpp                NVMCTRL: Nvm (both arrays, the CMDEX command
                           discipline, erase-by-row/program-by-page with the
                           page-buffer rules, region locks, PARAM geometry) +
                           FlashWaitStates + the read-only factory views
                           (NvmUserRow = this family's fuses, NvmCalibration,
                           NvmTemperatureCalibration, DeviceSerial)
    nvm_flash.hpp          RwweePartition (the array's 32 rows split once:
                           0..27 blocks, 28..31 the journal's attic) +
                           RwweeFlash and RwweeJournalZone, the two
                           FlashMedia backends over it - writing the RWWEE
                           array does not stall the CPU, and BOTH bounds are
                           constants because no linker section reaches there
    pin.hpp                Pin<'A',5>, PinConfig, the WRCONFIG multi-pin engine
    device_tables.hpp      THE RESERVE: the one file where vendor-macro
                           #ifdef probing is allowed - pad/instance facts
                           (EIC pad->line, TC WO pads + instance ids, TCC
                           instance geometry + its pad map keyed by pad AND
                           FUNCTION, AC AIN bonding) read from the device
                           header's own symbols and exported as constexpr data
    sercom.hpp             Sercom<n> resource + Uart task with two OPTIONAL
                           DMA engine slots
    dmac.hpp               Dmac block + DmaDescriptor + DmaChannel<n> +
                           DmaTxEngine/DmaRxEngine
    eic.hpp                EIC: the family's pin interrupts - Eic block
                           (per-line sense/filter/async, the optional clock
                           and the enable that synchronizes against it,
                           EVSYS generators published here) + ExtInt<Pin> /
                           ExtNmi<Pin> reached through the pad
    tc.hpp                 TC: Tc<n> resource (three widths incl. the paired
                           COUNT32, READSYNC discipline, capture, events both
                           ways, erratum 1.20.3 as code) + TcWo<Pin> + tasks
                           TcPwm/TcPwm8 (PwmChannel) and TcPeriodMeter/
                           TcPulseWidthMeter (MeterSource feeders)
    tcc.hpp                TCC: Tcc<n> resource over the whole of ch. 36 -
                           three instances that are NOT copies of each other
                           (width, channels, outputs and five optional
                           extension units all per instance, from the
                           reserve), seven waveform modes, ramps, dithering,
                           the waveform extension (output matrix, dead time,
                           swap, pattern) and BOTH fault systems, whose
                           inputs ARE the event inputs; buffered setters
                           REFUSE instead of waiting, because SYNCBUSY stands
                           until the update takes the buffer. + TccWo<Pin,
                           function> (the map needs both) + tasks TccPwm
                           (PwmChannel) and TccPairPwm (the complementary
                           pair with dead time)
    ac.hpp                 AC: Ac block + AcComparator<n> + AcWindow<w>
                           (window mode, the event surface both ways with
                           published codes, per-package input legality where
                           the PAIR owns the pads)
    evsys.hpp              EVSYS: the event fabric - twelve channels, the
                           three paths, the user multiplexer (channel+1 hidden),
                           the software event, three live errata as code. Owns
                           the FABRIC, not the generator/user tables: each
                           peripheral publishes its own codes
    osc32kctrl.hpp         OSC32KCTRL: the three 32 kHz roots (Osculp32k,
                           Osc32k with factory_calib() closing the loop to
                           nvm.hpp, Xosc32k + its failure detector), the RTC's
                           clock select, the shared IRQ 0
    freqm.hpp              FREQM: the hardware ratio counter between two GCLK
                           generators (refnum_for = the 24-bit overflow budget,
                           to_hz, measure); CTRLB written and NEVER read
                           (erratum 1.24.1), CFGA.DIVREF refused (absent on
                           this silicon), channels routed BEFORE the reset
    reset.hpp              RSTC + WDT: Reset (RCAUSE as ONE cause, not a
                           history; software() through SYSRESETREQ), Watchdog
                           (the shared period encoding, enable-protection vs
                           synchronization, early warning, clear() vs
                           force_reset()), ResetReporter and
                           hard_fault_reset<P>() (the HardFault body an app
                           binds; it never clobbers a record panic() wrote)
    supc.hpp               SUPC: Supc block + BodVdd (level/action/hysteresis,
                           continuous or sampled, the enable-protection AND
                           synchronization dance, matches_fuses() against
                           nvm.hpp's user row) + BodCore (READ-ONLY by design:
                           its calibration is a production value) + Vreg (no
                           enable verb - 22.8.6 forbids the change) + Vref
                           (the bandgap, and the VREFOE the AC's bandgap input
                           needs)
    rtc.hpp                RTC: one counter wearing three faces (COUNT32,
                           COUNT16 with PER, the CLOCK/calendar with its
                           masked alarm) over three overlaid register views,
                           the prescaler that is also the periodic-event
                           source, the read synchronization COUNT and CLOCK
                           need, FREQCORR, the event codes this peripheral
                           publishes and the one vector. It NEVER writes the
                           clock select - that is osc32kctrl.hpp's RTCCTRL -
                           and erratum 1.16.3 is answered structurally: no
                           verb writes COUNT or CLOCK in pieces
    sleep.hpp              PM: Pm (the three sleep modes with SLEEPCFG's
                           readback rule, STDBYCFG's regulator and RAM
                           back-bias, the guarded WFI) + SamSleepSite, the
                           util/power.hpp adapter - and the first place the
                           model's never-deeper rule is NOT the identity:
                           deep maps to standby because nothing is deeper -
                           and SamTimedSleepSite, the v2 site that LIFTS the
                           standby restriction (the RTC as alarm on
                           ticks_to_next() and as witness for the resync,
                           the model untouched, the after-a-wake convention
                           load-bearing)
    adc.hpp                ADC: Adc<n> over both converters (the same
                           peripheral at two addresses, with the header's own
                           host/client roles enforced), the two input muxes,
                           the six reference codes behind THIS TARGET'S Ref +
                           ref_mv() (util/analog.hpp's vocabulary; there is no
                           shared reference block on this family), the
                           prescaler and sampling arithmetic, the
                           RESSEL/AVGCTRL interplay whose full scale IS
                           util/analog.hpp's `steps`, the window monitor, the
                           digital corrections, the sequencer, both event
                           directions and the DMA trigger - with the factory
                           calibration copied by init() and erratum 1.4.4 as
                           code (an ADC event user takes the asynchronous
                           path or nothing)
    dac.hpp                DAC: `Dac`, a MONOSTATE resource (one instance on
                           every variant), with its OWN reference enum
                           DacRef, table 41-1's four data placements as
                           dac_data_word(), both outputs, the START/EMPTY
                           event pair and the DMA trigger - and buffer() a
                           plain store that never waits, because
                           SYNCBUSY.DATABUF stands until a start event
                           consumes the value
    sdadc.hpp              SDADC: `Sdadc`, a MONOSTATE 16-bit sigma-delta over
                           THREE DIFFERENTIAL PAD PAIRS behind a third-order
                           SINC decimation filter, with its OWN SdadcRef (four
                           codes, none Reserved), the three-stage clock
                           (GCLK / 2(P+1) / 4 / OSR), OSR and SKPCNT, the
                           SIGNED result in all three of its widths
                           (result() / result24() / result_raw() - the datapath
                           is 24 bits and the corrections speak those units),
                           the window, the post-processing, the sequencer, both
                           event directions and the DMA trigger; every
                           synchronized write WAITS BEFORE STORING, because
                           39.6.8 threatens a BUS ERROR where the ADC's and the
                           DAC's chapters promise a silent discard
    tsens.hpp              TSENS: `Tsens`, a MONOSTATE temperature sensor that
                           is NOT an ADC channel - it counts a temperature-
                           dependent oscillator against GCLK_TSENS, so the
                           GENERIC CLOCK IS THE RULER and VALUE is a signed
                           24-bit datum in CENTI-DEGREES CELSIUS only on the
                           48 MHz the factory calibration assumes;
                           TsensCalibration::factory() copies GAIN/OFFSET/
                           TCAL/FCAL out of nvm.hpp, tsens_gain_for() and
                           tsens_rescale() are the two escapes for any other
                           rate (both taking it as a caller argument), a zero
                           GAIN is REFUSED because it is 2^24 and not none,
                           and every synchronized write waits before storing
    ccl.hpp                CCL: `Ccl` (the block - one ENABLE, one software
                           reset, ONE generic clock for every filter, edge
                           detector and sequencer in it, the two sequencer
                           selectors) + `Lut<n>` (three input multiplexers,
                           the TRUTH table from `lut_truth()`, the
                           synchronizer/filter, the edge detector, both event
                           enables) + `CclIn<Pin>`/`CclOut<Pin>`. No
                           interrupt and no DMA exist here. Every configuring
                           verb refuses while the BLOCK is enabled and drops
                           LUTCTRLn.ENABLE for the store, because the two
                           enable gates are an AND (measured) and a write
                           into an enabled LUT is dropped in silence
    pac.hpp                PAC: `Pac`, MECHANISM AND NO CONCEPT - set/clear/
                           lock a peripheral's write protection by the id it
                           publishes (the keyed WORD-WISE WRCTRL store,
                           PERID = 32 x bridge + index), the per-bridge
                           status, the four read-and-clear flag banks, the
                           ACCERR event and the shared IRQ 0. No RAII guard
                           and no util contract: 11.5.2.6's balance rule
                           makes nesting a design decision, and erratum
                           1.13.3 means "protected" is not uniform
    dsu.hpp                DSU: `Dsu` - DID decoded (the fields the errata
                           matrix is keyed by), the hardware CRC32 over
                           anything the bus matrix reaches (chainable through
                           crc32_raw), MBIST WHICH DESTROYS WHAT IT TESTS,
                           the CoreSight ROM and the two debug channels. It
                           is the one peripheral that comes up PAC-protected,
                           so init() clears that through pac.hpp and
                           release() puts it back. Chip erase deliberately
                           absent
    divas.hpp              DIVAS: `Divas` - 32-bit signed/unsigned division
                           and unsigned square root, on TWO buses (the AHB,
                           whose RESULT read stalls, and the IOBUS alias at
                           0x60000200 that only the DATA SHEET names, whose
                           caller must poll). The operation starts on the
                           operand write; divide-by-zero does not trap
    mtb.hpp                MTB: `Mtb` - the Cortex-M0+ Micro Trace Buffer
                           pointed at a buffer of the program's own, so the
                           CPU reads its OWN hardware backtrace with no
                           debugger; MASTER.MASK as log2(bytes) - 4, POSITION
                           and FLOW as offsets from BASE, `MtbPacket` with
                           bit 0 of each word left an unnamed flag (10.3
                           defers to a TRM this project has not got, so the
                           device header is the only local authority);
                           `freeze()` and `snapshot()` are the post-mortem
                           pair - stop the trace BEFORE reading it, then
                           copy its tail oldest-first, bounded and legal
                           with interrupts dead
    postmortem.hpp         WHERE the program died, beside WHAT killed it:
                           `MtbPostMortem<bytes, keep>` freezes the MTB,
                           copies its last packets into a CRC-16'd .noinit
                           record and hands them to the next boot once
                           (`take()`), refusing to overwrite a diagnosis
                           that already stands; `TracingReporter` and
                           `hard_fault_trace_reset<P, Store>()` are the two
                           entry paths, composed with reset.hpp and
                           kernel/panic.hpp without touching either. A
                           SIBLING of the kernel's PanicRecord and not an
                           extension of it - a hardware trace is silicon
                           this stratum happens to have
  cortexm/               the CORE stratum: what the Cortex-M families share,
                         whatever the vendor and whatever the profile
    nvic.hpp               InterruptGuard (PRIMASK) + Nvic + irq_priority_levels
                           - reads CMSIS-Core only, #errors if included before
                           a device header (the family's nvic.hpp does both);
                           the guard accepts the M0, M0+, M4 and M33 core
                           headers
    ticker.hpp             BasicTicker<tps> over SysTick with advance/pause/
                           resume; each family's ticker.hpp adds its alias
                           and its own guards; SysTickCounter = SysTick as a
                           bare cycle counter (no interrupt) for delay_us
                           where the kernel timebase is elsewhere
    delay.hpp              delay_us / delay_rate / DelayRate: the microsecond
                           busy-wait on SysTick's VAL - at least, never early,
                           capped below one millisecond, no division at wait
                           time; each family's delay.hpp is the device include
                           plus this file plus its measured facts - the RP2350
                           is the one Cortex-M family that does NOT take it,
                           its ruler being a timer both its architectures read
  pl011/                 IP STRATUM: ARM's PrimeCell UART, written once for
                         every family that carries it - a directory named for
                         a peripheral DESIGN and not for a silicon, sitting
                         where a core stratum sits
    uart.hpp               the frame vocabulary, the fractional divider as pure
                           arithmetic, every register's bits as constants,
                           Pl011Uart<Chip, n> the resource and
                           Pl011Transport<Chip, n, pins, ...> the byte
                           transport (two rings, the error counters, two
                           optional DMA engine slots, the ISR bodies) - and
                           `Pl011Chip`, the concept the family's traits type
                           satisfies: where the registers are, the reset, the
                           interrupt line, the guard, the platform, the pads
                           and which rate is UARTCLK. The pad's function code
                           stays OPAQUE here, because a family may have more
                           than one column for it
  pl022/                 IP STRATUM: ARM's PrimeCell SSP (the SPI block), the
                         same arrangement
    spi.hpp                the resource, the host engine util/spi_bus.hpp's
                           SpiBus drives with the other strata's Request
                           verbatim, and the client - over `Pl022Chip`, which
                           adds to the UART's list the run-time select pin, the
                           busy-wait a select setup is spent on and the PAD as
                           the lever a dark listener releases, because a
                           silicon may leave the block's own pad-enable output
                           unconnected
  dw_apb_i2c/            IP STRATUM: Synopsys's DesignWare I2C, the same
                         arrangement
    i2c.hpp                the resource over a COMMAND FIFO whose entries carry
                           their own RESTART and STOP, the host engine
                           util/i2c_bus.hpp's I2cBus drives, and the client -
                           over `DwApbI2cChip`, which adds the open-drain verbs
                           a bus clear drives a line with, the microsecond
                           ruler it paces itself by and which rate is ic_clk
  stm32g0/               everything that knows stm32g0xx.h (STM32G0, Cortex-M0+)
    device_tables.hpp      THE RESERVE: GPIO ports, USART instances, APB
                           enables, CCIPR multiplexers and the SHARED
                           VECTORS, read off the device header - the
                           vectors DERIVED FROM PRESENCE (a line's
                           enumerator names what shares it, and the
                           sharer's base macro is what the preprocessor
                           can probe); no device-select macro anywhere.
                           Also the handler NAMES an app on more than
                           one board binds (BRIO_STM32G0_*_HANDLER), the
                           same presence rule as macros
    nvic.hpp               "stm32g0xx.h" + cortexm/nvic.hpp
    ticker.hpp             cortexm/ticker.hpp + the Ticker alias (1000 Hz)
    platform.hpp           Stm32g0Platform<TB = Ticker> (WFI = Sleep mode,
                           SLEEPDEEP never written; BKPT; .noinit breadcrumb;
                           atomic_width 4) + the Tickless<TB> concept and the
                           idle_until() the kernel calls on such a timebase
    flash.hpp              FlashWaitStates (table 13, the read-back rule),
                           FlashAccel, the program/erase engine over both
                           banks, the option bytes decoded read-only,
                           DeviceUid / DeviceIdcode
    clock.hpp              Rcc (HSI16/HSIDIV, the PLL, SYSCLK switch, bus
                           prescalers, the per-peripheral ENABLES, CCIPR,
                           the MCO clock output), PowerRegime,
                           Clock<internal|pll, hz, regime> with the
                           compile-time PLL ratio search and the Range 2 /
                           low-power-run sequences, Rates<> +
                           DynamicClock<Rates<...>, Users...> (a pack of
                           rate TUPLES, the direction-aware switch, the
                           fan-out, restore() after a Stop) - the third
                           clock model
    pin.hpp                Pin<'A',5> / Port<'A'> / PinRef over GPIOx: the
                           port clock opened by every configuring verb,
                           MODER/OTYPER/OSPEEDR/PUPDR/AFR, BSRR/BRR values
    usart.hpp              Usart<n> resource + Uart<n, pins> task, the
                           other two targets' Uart surface verbatim
    lptim_ticker.hpp       LptimTicker<cfg>: the TICKLESS kernel timebase -
                           the LPTIM on the LSE crystal undivided, the tick
                           its count shifted (1024 Hz), arm_wake() placing
                           the loop's next deadline in CMP under the four
                           rules the silicon and ES0548 2.8.2 dictate; the
                           platform's template argument for a program with
                           no periodic interrupt
  ch32v00x/              everything that knows the CH32V00x (WCH QingKe V2C,
                         RV32EC) - and NO vendor header: the map is the
                         stratum's own
    device.hpp             the register map in the chapter's words (RCC, GPIO,
                           USART, FLASH, the core's STK and PFIC), the
                           interrupt numbers = the vector table's word
                           indices; asks the build's part definition ONCE
                           and includes parts/ch32v003.hpp or
                           parts/ch32v006.hpp - the RESERVE of this family:
                           the memories, the flash page, the bonded ports,
                           the instances, the GPIO mode width, AFIO's
                           register order, what the USART has, as constexpr
                           facts every driver branches on with if constexpr
                           (two macros for per-instance code)
    pfic.hpp               InterruptGuard (csrrci read-and-clear of
                           mstatus.MIE), enable/disable/readback, Pfic per-
                           line enables (write-one registers; the manual's
                           ISR bank is the ENABLE status, IPR the pending)
    ticker.hpp             BasicTicker over the core's STK (up-count, STRE
                           auto-reload, CNTIF cleared by the handler),
                           Ticker = 1000 Hz
    clock.hpp              Rcc (the HSI trim, the LSI, the PLL, the HPRE
                           divider, the MCO, the clock monitor, the
                           peripheral gates) + Clock<internal|pll, hz>
                           (flash wait states first; pclk_hz = hz, no APB
                           prescaler) + DynamicClock<Boot, Users...> over the
                           HPRE ladder, restore() after a Standby
    delay.hpp              delay_us(clock, us) on the STK counter: at least,
                           never early, refused at one tick and beyond
    pin.hpp                Pin<'D',5> / Port<'D'>: the ONE-BIT MODE of this
                           family (an F1 nibble is right by accident), pulls
                           through OUTDR, the port clock opened by every
                           configuring verb
    usart.hpp              Usart<n>: the RESOURCE over the whole of RM ch. 14
                           (the frame, the divisor, mute with both wakes,
                           LIN's break, half duplex - a BUS here, not a loop
                           -, IrDA, CTS/RTS, DMA, every flag; no synchronous
                           mode and no smartcard on this silicon, stated as
                           facts) + Uart<1|2, P, rx, tx, TxEngine, RxEngine,
                           remap, opts>: the interrupt-driven byte transport
                           on it (two rings, TXEIE armed/disarmed, errors
                           read then cleared, BRR = pclk/baud whole) with
                           two OPTIONAL DMA engine slots (harvest() the verb
                           that publishes a receive run) and trailing
                           options that cost nothing; USART2 on columns 1..6
    dma_engine.hpp         NoDmaEngine, the empty slot's tag and nothing else -
                           the STM32G0's arrangement, so a driver with an engine
                           slot never includes the controller
    dma.hpp                Dma + DmaChannel<1..7> (THE CHANNEL IS THE
                           REQUEST: no multiplexer, table 8-2 names the
                           channel; every store refused while EN is set,
                           which stays set after a completed block) +
                           DmaTxEngine/DmaRxEngine<ch, Elem>
    spi.hpp                Spi<1> resource (the F1's SPI, no FIFO, HSCR) +
                           SpiHost<1, pins, TxEngine, RxEngine> (the other
                           strata's Request VERBATIM: pump on RXNE or polled,
                           engines on channels 3/2 only) + SpiClient
    i2c.hpp                I2c<1> resource (the F1's event machine, NO rise-
                           time register, CCR rounded up) + I2cHost<1, pins,
                           TxEngine, RxEngine> (one tenure = write / read /
                           write-then-read / probe, the receive procedure by
                           count, two vectors, engines on channels 6/7, the
                           unstick) + I2cClient (a polled surface + two ISR
                           bodies reporting I2cClientEvent)
    tim.hpp                Tim<1|2> (the F1's timers under WCH's names, TIM2's
                           dead-time pairs via DTCR, CAPLVL/CAPOV/OE_MODE) +
                           Tim3 (the streamlined block: no pad, no interrupt,
                           a ONE-SHOT DMA request) + TimPad + the tasks
                           (TimPwm/TimPairPwm, TimPeriodMeter/TimIntervalMeter,
                           TimEventCounter/TimGatedCounter, TimPeriodicTick,
                           TimOnePulse)
    adc.hpp                Adc (the F1's converter + CTLR3: low power, three
                           watchdogs that can reset the chip, THE WATCHDOG
                           SCAN = one per rank; the STALL and recover()),
                           AnalogIn<Pin> deriving the channel, AdcInput
                           (VREFINT, the OPA), the sampler's converter surface
    afio.hpp               the remap tables (7-8..7-15) as constexpr data +
                           Afio's verbs over PCFR1; SpiPins/I2cPins carry the
                           code, Uart takes it as a template parameter,
                           Tim<n>::remap(code); USART2 refused at code 0 (its
                           default TX is the K8's reset pin)
    opa.hpp                Opa (the key-locked amplifier: four positive pads,
                           a negative pad or a PGA gain, the differential
                           PGA, the bias, the output always on ADC channel 9;
                           CMP2's verbs answering false on the V006)
    exti.hpp               Exti (ten lines: eight pads via AFIO_EXTICR, the
                           PVD, the AWU; interrupt or event, edges, the
                           software trigger) + ExtInt<Pin>
    reset.hpp              Reset (RSTSCKR's flags as history - PINRSTF names
                           the pin ALONE here -, software() through
                           PFIC_CFGR), ResetReporter, fault_reset<P>() + the
                           two watchdogs: Iwdg (a key starts it, only a
                           reset stops it, the update flags waited for) and
                           Wwdg (whose counter does NOT run unarmed, against
                           the chapter; the RCC pulse is the way back)
    nvm.hpp                Flash: the engine (fast page program as the ONLY
                           write, two locks, three erase grains), refused
                           as a code beside STATR's errors
    nvm_flash.hpp          MainFlashPartition (the linker's 40 KB, the heap's
                           16 KB, the journal's 6 KB attic) + MainFlash and
                           MainFlashJournalZone, the two FlashMedia with THE
                           PAGE AS THE CELL (256 B)
    sleep.hpp              Pwr (Sleep/Standby, the LDO, the PVD; every verb
                           opens the PB1 gate first), Awu (the LSI alarm on
                           EXTI line 9), Ch32SleepSite (light = Sleep,
                           standby and deep = Standby, restore() after) and
                           Ch32TimedSleepSite (the LSI MEASURED at init, the
                           span handed back less the ticks counted awake)
    platform.hpp           Ch32v00xPlatform<TB = Ticker>: idle() is a WFE,
                           not a WFI - this core's WFI wakes only for an
                           interrupt it can TAKE, so "sleep then unmask"
                           deadlocks; WFITOWFE + SEVONPEND latch the wake
                           instead, and with SLEEPDEEP armed the ticker is
                           paused across the sleep; ebreak; .noinit
                           breadcrumb; atomic_width 4
  ch32v203/              everything that knows the CH32V203 (WCH QingKe V4B,
                         RV32IMAC, ilp32): the STM32F1's peripheral generation
                         under WCH's names, and NO vendor header - the map is
                         the stratum's own
    device.hpp             the register map read off the reference manual -
                           the blocks more than one chapter reaches, RCC and
                           PWR among them, live here and nowhere twice;
                           asks the build's part definition ONCE and includes
                           parts/<part>.hpp - THE RESERVE of this family:
                           memories, bonded pads, instances, the device class
                           and its vector tail, as constexpr facts every
                           driver branches on with if constexpr
    pfic.hpp               InterruptGuard (csrrci on mstatus.MIE), the PFIC's
                           per-line verbs, BRIO_CH32_INTERRUPT - the handler
                           attribute the CH32V203_HPE option decides
    ticker.hpp             BasicTicker over the core's 64-bit STK (Ticker =
                           1000 Hz)
    delay.hpp              delay_us on the STK counter: at least, never early,
                           refused at one tick and beyond
    platform.hpp           Ch32v203Platform<TB = Ticker>: the WFE-shaped idle
                           (WFITOWFE + SEVONPEND, this core's WFI wakes only
                           for a takeable interrupt), ebreak, the .noinit
                           breadcrumb, atomic_width 4
    clock.hpp              Rcc + Clock<internal|pll|crystal|external, hz,
                           xtal_hz> + Rates<>/DynamicClock<Rates<...>,
                           Users...>: PARKS ON THE HSI before touching the PLL
                           (PLLMUL/PLLSRC/PLLXTPRE take a write only with the
                           PLL off, and it will not stop while it is SYSCLK),
                           the HSI's PLL divider in EXTEN and the HSE's per
                           DEVICE CLASS, the PLL's input and output ranges part
                           facts, PB1 capped at 72 MHz with pclk1_hz_at() the
                           arithmetic a rebased user uses, the USB and ADC
                           dividers part of the tree, the LSI, the clock
                           security system as the NMI's body, the ready
                           interrupts, and Mco over the pad two packages have
                           not got
    pin.hpp                Pin<'A',5> / Port<'A'>: the F1's two-bit MODE over
                           two registers, the pull in the output register, the
                           bonding from the part table, the whole-port verbs
                           (a mask configured in one store per register, the
                           port written whole) and LCKR - the configuration
                           lock whose only way back is a reset
    afio.hpp               the remap columns of AFIO_PCFR1/PCFR2 as constexpr
                           pad tables (the manual's, one per peripheral), with
                           Remap + afio_remap_has_code() judging a column by
                           the DEVICE CLASS and by the part's own bonding -
                           refused by static_assert where the code is a
                           constant, by false where it is not; AFIO_EXTICR for
                           exti.hpp, the event output, and SW_CFG read but
                           never written: the debug port is the probe's
    exti.hpp               Exti (twenty-two lines: sixteen by PIN NUMBER with
                           the port chosen per line, the PVD's, the RTC
                           alarm's and the two USB wake-ups; edges, the two
                           enables, the software trigger whose bit stands
                           until the flag is cleared, write-one flags, five
                           single vectors and two shared ones with isr() +
                           served()) + ExtiLine<n> + ExtInt<Pin> (claim,
                           select refusing a line another port holds, steal)
    usart.hpp              the serial ports (ch. 18): Usart<n> the resource
                           over the whole chapter - four instances on two
                           buses with every divisor asked of the instance's
                           OWN bus, and WHICH instances a part offers taken
                           from the datasheet's table; the frame in every
                           shape M and STOP allow, mute mode with both wakes,
                           LIN's break, single-wire half duplex, IrDA, and
                           THE SMARTCARD AND THE SYNCHRONOUS CLOCK, which
                           exist because the fourth port of this part is a
                           USART4 and not a UART4; the flow-control pair, the
                           two DMA requests, every flag with the sequence
                           that clears it and both interrupt sources; the
                           pads are afio.hpp's COLUMNS and a remap code of 0
                           writes no register at all - + Uart<n, P, rx_size,
                           tx_size, format, TxEngine, RxEngine, remap, opts>:
                           the interrupt-driven transport with the other
                           strata's surface, the frame as a template
                           parameter of its own and two OPTIONAL DMA engine
                           slots (harvest() the verb that publishes a receive
                           run)
    usb.hpp                the USB DEVICE controller (ch. 21): Usbd<pma_bytes>,
                           ST's F1/F0 device peripheral under WCH's names,
                           realizing util/usb's UsbController at the packet -
                           eight endpoint registers whose status and toggle
                           fields are WRITTEN BY XOR while the two completion
                           flags in the same word clear on a zero, a packet
                           memory of 512 bytes seen through a 32-bit window
                           whose top 128 the CAN's filter table takes (so the
                           budget is a template parameter and an endpoint past
                           it is refused), COUNTn_RX both the size the program
                           writes and the count the hardware writes back over
                           it - which is why a reception rewrites it, why the
                           bus reset guards against one already standing, and
                           why an OVERFLOW IS REPAIRED and not only counted (it
                           leaves that field zero and raises no completion, so
                           the endpoint would lose every packet after it) - the
                           host's suspend answered with FSUSP, which is what
                           arms the wake-up at all, the pads that are port A's
                           GPIO pads and the pull-up that lives in EXTEN - and
                           TWO COMPILE-TIME REFUSALS: the 48 MHz the tree's
                           USBPRE must make, and usbd_min_hclk_hz, the measured
                           floor of the bus (96, 48 and 24 MHz carry data, 12
                           does not). An attached controller holds one count in
                           bus_activity.hpp from its pull-up to its detach,
                           which is what keeps the core awake for it
    reset.hpp              Reset (RSTSCKR's six flags as a HISTORY, software()
                           through the keyed PFIC_CFGR), ResetReporter,
                           fault_reset<P>() carrying the cause byte from
                           mcause - bound to BOTH trap entries, because an
                           ebreak lands on the breakpoint vector and not the
                           exception one
    tim.hpp                the timers (ch. 14, 15): Tim<1..4> over the F1's
                           blocks under WCH's names - the time base with its
                           two shadow registers, the channels in both faces
                           (CCyS writable only with the channel off), the
                           slave controller and the master TRGO, the internal
                           trigger table folded through what the PART has,
                           the repetition counter, complementary outputs,
                           dead time and break of TIM1 alone, the DMA burst
                           engine, the rc_w0 flags and TIM1's FOUR UNSHARED
                           vectors + TimPad from afio.hpp's remap columns and
                           the nine tasks (TimPwm/TimPairPwm, TimPeriodMeter/
                           TimIntervalMeter, TimEventCounter/TimGatedCounter,
                           TimPeriodicTick, TimOnePulse, TimEncoder); no
                           basic timer exists on this series and the 32-bit
                           TIM5 is the 128 KB part's
    watchdog.hpp           IWDG + WWDG (ch. 7, 8), a file of their own beside
                           reset.hpp's flags: Iwdg (the three keys, the
                           prescaler and reload that take a write only while
                           the LSI RUNS - so arm() starts the watchdog first
                           and ends with the refresh that re-locks them -,
                           running() read off the forced oscillator, the
                           time-out arithmetic taking the LSI rate as an
                           argument) and Wwdg (PCLK1/4096/2^WDGTB, the counter
                           that does NOT run unarmed, the window whose early
                           refresh IS the reset, the early wake-up flag and
                           its vector, the block's reset line as the only way
                           back)
    dma_engine.hpp         NoDmaEngine, the empty slot's tag - and THE REQUEST
                           TABLE (11.2.3's tables 11-5 and 11-6): DmaRequest,
                           one enumerator a row, dma_request_channel() folding
                           the PART (0 where the part has not got the
                           peripheral that raises it) and DmaRequestOf<r>
                           refusing it at compile time, so a transport can
                           check a named engine against the table without
                           including the controller
    dma.hpp                the DMA (ch. 11): Dma (the gate, the flags,
                           stop_all() because RCC_AHBRSTR has no bit for this
                           block, any_enabled() the power model's one
                           question) + DmaChannel<1..8> (THE CHANNEL IS THE
                           REQUEST: prepare/load/trigger, every store refused
                           while EN is set - which stays set after a completed
                           block - and every address refused unless it is
                           aligned to its own width, because 11.3.5 would
                           align it in silence; remaining(), the four flags
                           and the ISR body, one vector a channel with the
                           eighth on the device class's own tail) +
                           DmaTxEngine/DmaRxEngine<ch, Elem> for the
                           transports' slots, and the rule that two engines of
                           one transport name two channels; + the BLOCK ENGINES
                           util/block_stream.hpp asks for - DmaLoopEngine (a
                           BlockPlayer on the controller's own circular mode,
                           the lap interrupt only counting) and
                           DmaPingPongEngine (a BlockSource that does NOT use
                           it: "skip rather than tear" cannot be decided on a
                           channel that never stops, measured at three items
                           past the edge, so it stops at every block and the
                           handler re-arms the other buffer)
    adc.hpp                the two converters (ch. 12): Adc<1|2> over the F1's
                           ADC under WCH's names - the calibration run BEFORE
                           the buffer and the internal sources (TSVREFE forces
                           BUFEN on for good), the regular group of sixteen and
                           the INJECTED four that preempt it with a signed
                           offset, scan/continuous/discontinuous, the analog
                           watchdog, the timers' triggers and AN EXTI LINE
                           (code 110 is a line on this family; the TIM8
                           alternative and the four AFIO bits that would select
                           it are another class's), the DMA request handed to a
                           block engine by claim_stream<Engine>() - refused at
                           compile time off table 11-5's row -, the rc_w0 flags
                           and ONE VECTOR FOR BOTH units, the DUAL modes as
                           ADC1's verb alone (no util shape: one family does
                           not make a contract), and WCH's own input buffer
                           with a PGA of 1/4/16/64 + AnalogIn<Pin> from the
                           family's pad map, AdcInput (the sensor on 16,
                           VREFINT on 17, one bit waking both) and Ref/ref_mv -
                           no package brings out a VREF+ pad, so the reference
                           IS VDDA and vdda_mv() measures it
    opa.hpp                the two amplifiers (ch. 30): Opa<1|2> over FOUR BITS
                           EACH in one register that lives in the EXTEN block's
                           window - an enable, one of two positive pads, one of
                           two negative ones, one of two outputs, and nothing
                           else: no key, no lock, no gain, no internal
                           feedback, so with no wire strapped the block is an
                           open-loop stage + OpaIn<n, which> / OpaOut<n, which>
                           (the datasheet's pad map, every output pad an ADC
                           input pad, which is this block's whole route to the
                           converter); OPA3 and OPA4 belong to other device
                           classes and the twenty-pin part has OPA2 alone
    spi.hpp                the two synchronous ports (ch. 20): Spi<1|2> the resource
                           over the whole chapter - TWO INSTANCES ON TWO BUSES, so one
                           BR code is two frequencies (SPI1 divides PB2, SPI2 PB1) and
                           a program asks the INSTANCE; no FIFO at all, which is what
                           makes a host's pump run on RXNE and a client answer ONE
                           FRAME AHEAD; the four modes, both widths, both bit orders,
                           the three NSS arrangements, the simplex and one-wire line
                           modes, the hardware CRC, every flag with the sequence that
                           clears it - and MODF with the measured fact that 20.2.7's
                           recipe does NOT clear it here, the block's reset line being
                           the way back - the high-speed read mode confined to BR = /2
                           on this device class, and NO I2S (the datasheet gives this
                           series none, and the register is only asked whether it
                           answers) + SpiPins carrying afio.hpp's COLUMN (SPI1 has two,
                           SPI2 no remap field at all), SpiRateOf<pclk, hz> the
                           compile-time rate chooser, and SpiHost<n, pins, TxEngine,
                           RxEngine> with the other strata's Request VERBATIM, its
                           engines fixed to the channels table 11-5 wires to the
                           instance + SpiClient<n, pins>, one frame ahead, the dark
                           listener releasing MISO - and pad_speed() on both, the slew
                           class of the pads a task drives
    i2c.hpp                the two-wire ports (ch. 19): I2c<1|2> the resource over
                           the whole chapter - the F1's event machine under WCH's
                           names WITH the rise-time register the CH32V00x has not,
                           so the SCL timing is THREE registers and FREQ's six bits
                           are the chapter's own ceiling (4..60 MHz of PB1: the one
                           peripheral this family cannot run at the top of its
                           tree); the receive procedure by count, 7- and 10-bit own
                           addresses with the dual address and the general call,
                           SMBus and PEC as bits, the two DMA rows with LAST, two
                           vectors an instance, and BUSY as the WIRE (a START set
                           into a busy bus is held by the hardware, and a tenure
                           that ends with no STOP seen leaves BUSY standing over an
                           idle wire - 19.12.1's own case, taken out of the way by
                           SWRST and only when both lines read high) + I2cPins
                           carrying afio.hpp's COLUMN (I2C1 has two, I2C2 none) and
                           I2cHost<n, pins, TxEngine, RxEngine> with the other
                           strata's Request VERBATIM, its engines fixed to the
                           channels table 11-5 wires to the instance, unstick()
                           counting the clocks a stuck target took + I2cClient<n,
                           pins> with a POLLED option and flush(), the PE cycle
                           that drops a byte the controller never clocked
    nvm.hpp                the flash memory and the user option bytes (ch. 32)
                           with the electronic signature beside them (ch. 31):
                           Flash, the engine - TWO programming methods (a
                           half-word behind PG, a whole 256-byte page behind
                           FTPG) and FOUR erase grains (the page, the 4 KB
                           sector that is also the write-protection unit, a
                           32 KB block, the chip behind a policy argument no
                           suite passes), three locks each wanting its own key
                           pair with a wrong one holding until the next system
                           reset and raising no bus error (measured), AN ERASED
                           PATTERN THAT IS NOT ALL ONES (0xE339E339) over cells
                           that take pass after pass between erases, the
                           enhanced read mode that would fail an erase in
                           silence and that does not engage on this class, the
                           status flags and the one error this family carries,
                           the interrupt - and THE RATE AS PART OF THE CONTRACT:
                           the access clock may not exceed 60 MHz and SCKMOD
                           already halves the system clock, so above 120 MHz an
                           erase or a program wants HCLK divided around it,
                           which the engine REFUSES instead of doing behind its
                           caller's back, at compile time under a static Clock
                           and with a code under a dynamic one + FlashOptions
                           and FlashOptionArea, every option byte decoded
                           READ-ONLY with RDP written by no verb, and DeviceUid
                           / flash_size_kbytes
    nvm_flash.hpp          MainFlashPartition (the LAST 4 KB of every part's
                           array - sixteen pages, one write-protection unit -
                           with all nine linker scripts stopping that far short)
                           + MainFlash<Clock>, the FlashMedia over it with THE
                           PAGE AS THE CELL (256 B) and THE CLOCK IN THE
                           MEDIUM'S TYPE, because the contract's program() and
                           erase() take an address and nothing else while an
                           erase above 120 MHz is illegal; no heap and no
                           journal stand on it here, by decision - and the
                           journal could not without being taught an erased
                           pattern that is not 0xFF. A write is a WAIT and not a
                           stall: the core goes on running out of the array
                           while the engine works
    crc.hpp                CRC (ch. 5): Crc, a monostate over three registers -
                           the Ethernet polynomial wired in (no polynomial, no
                           initial value, no reversal: the function IS
                           CRC-32/MPEG-2), a WORD the only grain and a byte run
                           that is not whole words a compile error, the reset
                           that lands before the next instruction can look
                           (measured, where the STM32F4's same block does not),
                           and the eight-bit scratch that RST spares and only a
                           SYSTEM reset clears, this block having no line in
                           RCC_AHBRSTR; word_be is the packing, spelled as the
                           STM32F4 stratum spells it, and util/crc.hpp's
                           crc32_ethernet* is what the silicon is judged
                           against. No interrupt, no DMA row, no per-part fact
    rtc.hpp                the real-time clock and the backup domain (ch. 6, 4,
                           with 3.4.9's clock select and 2.4.1's write enable):
                           RtcDomain (PWR_CTLR's DBP read back, the whole of
                           RCC_BDCTLR - the LSE with its bypass, RTCSEL one-way
                           with BDRST the way back, RTCEN - and the HSE division
                           that is 512 OR 128 BY LOT NUMBER, so the part table
                           states the pair and a program measures which it has),
                           Rtc (a NUMBER and not a calendar: a 32-bit counter
                           behind a 20-bit prescaler, the CNF write window and
                           the RSF read synchronization on every access, a
                           prescaler reload and an alarm that cannot be read
                           back, the counter's two halves read against a carry
                           the chapter is silent about - and which the bus can
                           answer with a copy up to three ticks stale - the
                           second, alarm and overflow on one vector and the
                           ALARM AGAIN on EXTI line 17, which fires with this
                           block's own interrupt enable clear) and Bkp (the data
                           registers, ten on this device class and forty-two on
                           the other, wiped by a domain reset or a tamper and
                           NOT by the block's own reset line, which moves
                           nothing at all; the tamper input that REMEMBERS an
                           edge it was not watching for and that no pad of this
                           board can raise; and the three things PC13 can carry)
    bus_activity.hpp       the count of BUS MASTERS OTHER THAN THE CORE, kept
                           by the drivers that make one work and read by the
                           idle path and by the sleep sites: in a sleep of any
                           depth here no other master gets a cycle (measured),
                           so a DMA channel while EN is up and the USB
                           controller from its pull-up to its detach each hold
                           one count, idle() sleeps only at zero and a site
                           refuses to arm above it - the old rule "a program
                           with USB never idles" as a mechanism
    pwr.hpp                the power controller (ch. 2): Pwr, the three modes
                           behind SLEEPDEEP and PDDS (the core's bit and this
                           block's, written and read as ONE PAIR), the Stop
                           that is one mode with TWO PRICES (LPDS and the RAM's
                           low-voltage mode, whose interlock the chapter
                           states), the two flags a boot reads and their
                           write-one clears - WUF being the WAKE's flag and not
                           the event's, measured - the WKUP pad PA0, the supply
                           monitor whose eight thresholds are worth TWO
                           DIFFERENT TABLES of millivolts by a bit of the DIE
                           and not of the part (FEATURE_SIGN.VLEVEL, read at
                           run time and believed only if it inverts), what a
                           Standby keeps of the RAM per device class - the
                           manual's "20K" being the class's largest array and
                           not every part's - the regulator's two trims that
                           live in EXTEN, and the debug module's three
                           low-power bits READ AND NEVER WRITTEN, because a
                           csrw to that CSR resets the part
    sleep.hpp              Ch32v203SleepSite (light -> Sleep, standby -> a Stop
                           on the main regulator, deep -> the same on the
                           low-power one, STANDBY OFF THE LADDER with
                           enter_standby() the deliberate door - every exit
                           measured here being a reset, an ordinary EXTI line
                           among them - the clock tree restored after a Stop)
                           and Ch32v203TimedSleepSite (the RTC's alarm on EXTI
                           line 17 as the wake and its counter as the WITNESS,
                           a 1024 Hz tick out of the crystal, the counts
                           rounded up and the span down so a wake is late and a
                           resync short, the four-act ISR) - both REFUSING
                           every rung but none while a bus master other than
                           the core is working
  stm32f4/               everything that knows stm32f4xx.h (STM32F4, Cortex-M4F):
                         brio's first ARMv7-M family on the cortexm/ core files
    device_tables.hpp      THE RESERVE: GPIO ports A..K, the serial instances
                           1..10 (bus, gate, vector, FULL by the U(S)ART name),
                           the regulator's VOS width and over-drive pair, and
                           THE FREQUENCY LADDERS keyed on the device-select
                           define - known for the F405, F42x/F43x, F446 and
                           F411 classes, refused elsewhere; the watchdogs'
                           and the EXTI's per-part facts (implemented lines,
                           port codes, per-line vectors) appended by chapter;
                           the backup-register count read off RTC_TypeDef
                           itself and the RTC's pad facts keyed on the part
                           class (ST declares TAMP2E on every header, so the
                           count of tamper inputs is the manual's); and the
                           flash interface's per-class facts, the chapter where
                           the device header is wrong in BOTH directions (a
                           second bank's bits declared on a part that has one
                           bank, a PCROP bit omitted on a part that has it)
    nvic.hpp / ticker.hpp / delay.hpp  the device header + the cortexm/ file:
                           PRIMASK the one mask on a core that has BASEPRI,
                           SysTick at 1000 Hz, delay_us on VAL
    platform.hpp           Stm32f4Platform<TB>: WFI = Sleep, SLEEPDEEP never
                           written; BKPT; .noinit breadcrumb; atomic_width 4
    pwr.hpp                Pwr: the whole power controller - the APB1 gate,
                           VoltageScale (one bit or two), the over-drive pair
                           (ODEN/ODRDY, ODSWEN/ODSWRDY, and an exit that waits
                           ODSWRDY down), the Sleep/Stop/Standby ladder as
                           SLEEPDEEP + PDDS with StopConfig's four price bits
                           (LPDS, FPDS, the low-voltage pair, under-drive), the
                           wake-up pins and the ONE flag they share with the
                           RTC, ES0298 2.2.4's Standby sequence, the PVD with
                           its part-class thresholds, the backup regulator, and
                           the three DBGMCU bits a probe leaves behind
    sleep.hpp              Stm32f4SleepSite (light -> Sleep, standby -> a Stop
                           on the main regulator, deep -> the low-power one
                           with the flash in power-down; Standby off the ladder
                           and every arm() CLEARING PDDS to keep it there; the
                           SYSCLK restore after a Stop and the ticker paused
                           across one - ES0298 2.2.1) and Stm32f4TimedSleepSite
                           (the RTC's wake-up timer as alarm and its sub-second
                           counter as witness, the frozen span handed to
                           Ticker::advance(), the four-act ISR)
    flash.hpp              the whole of chapter 3. For the clock task:
                           FlashWaitStates (the readback rule) and FlashAccel
                           (the ART's prefetch, both caches, and the flush an
                           erase asks for). Then the write side: the sector map
                           COMPUTED from the size register (four of 16 Kbytes,
                           one of 64, 128 to the end of the bank), the keyed
                           lock, the program and erase engine with the
                           PARALLELISM AS A TEMPLATE PARAMETER because it is
                           the store instruction (x64 refused for want of VPP),
                           the four malformed sequences a suite stages on
                           purpose and the fifth that locks the engine until
                           reset, the interrupt - and FlashOptions, every option
                           byte decoded with nWRP the only one written, through
                           a half-word store that keeps the RDP byte out of the
                           data path; no FlashMedia, by the NV review's decision.
                           DeviceUid / flash_size_kbytes / DeviceIdcode
    clock.hpp              Rcc (HSI, HSE crystal or bypass, the main PLL, the
                           switch, the APB prescalers, the enables with the
                           errata readback, the resets, MCO1/2) + Clock<src,
                           hz, hse_hz, hse_mode>: the exact PLL ratio at
                           compile time, the regulator scale and over-drive
                           sequenced in the manual's order, pclk1_hz/pclk2_hz
                           beside hz, apb_hz(clock, bus) for a peripheral's
                           own rate + Rates<> and DynamicClock<Rates<...>,
                           Users...>: the pack of rate TUPLES, the switch that
                           PARKS ON THE HSI (neither the PLL nor the scale nor
                           the over-drive bits may be written otherwise, so one
                           order serves both directions), restore() after a Stop
    pin.hpp                Pin<'A',5> / Port<'A'> / PinRef over GPIOx: the G0's
                           block without a BRR (BSRR's upper half), the port
                           clock on AHB1 opened by every configuring verb,
                           input floating as the reset state
    reset.hpp              Reset (RCC_CSR's seven flags as a HISTORY - PINRSTF
                           raised by every internal source too -, RMVF, and
                           software() through SYSRESETREQ) + Iwdg (NO window on
                           this family; the keyed registers that do not update
                           until the start key, so arm() starts first; the LSI
                           as the only witness) + Wwdg (PCLK1/4096/2^WDGTB with
                           a TWO-bit WDGTB, the free-running counter, in_window()
                           and EWIF all measurable with WDGA clear) + Faults (the
                           three configurable fault vectors, the CCR traps, and
                           CFSR/HFSR/MMFAR/BFAR as a twelve-byte FaultRecord the
                           APPLICATION banks) + ResetReporter and
                           hard_fault_reset<P>()
    syscfg.hpp             Syscfg: the block the EXTI's pin multiplexer lives
                           in - the APB2 gate that is CLOSED at reset and
                           that every verb (reads included) opens, EXTICR,
                           the memory map read-only, the I/O compensation
                           cell, the Ethernet PHY select
    exti.hpp               EXTI: Exti (the 23 lines - sixteen pin lines
                           numbered by the PIN NUMBER, the rest peripheral
                           wake-ups derived from the peripherals' presence -
                           the edge senses, ONE rc_w1 pending bit that
                           exists only while the interrupt is unmasked, the
                           software trigger that obeys the mask and does not
                           self-clear, the vectors and the ISR body that
                           clears first) + ExtInt<Pin> (the pad's face,
                           REFUSING a line another port is using, steal()
                           the override) + ExtiLine<n> (a wake-up named as a
                           constant, refused where the part has none)
    dma_engine.hpp         NoDmaEngine, the empty slot's tag
    dma.hpp                the two DMA controllers (ch. 10): Dma<1|2> (the AHB1
                           gate, the flag banks decoded per stream, the
                           write-one clears) + DmaStream<n, 0..7> - a STREAM is
                           the unit and CHSEL only picks one of its eight
                           request lines: the enable discipline of 10.3.17 with
                           the read-back slack measured, table 49's burst and
                           threshold arithmetic refused BEFORE the enable,
                           direct mode, the double buffer with its
                           current-target readback, circular, the peripheral
                           flow controller, five flags and one ISR body -
                           + DmaTxEngine/DmaRxEngine<n, stream, channel, Elem>
                           for the transports' slots, checked against the
                           request mapping the reserve keys per part class
    usart.hpp              Usart<n> resource over the classic SR/DR/BRR chapter
                           (mute, LIN, IrDA, smartcard, synchronous, flow
                           control, DMA requests, every flag) + Uart<n, pins,
                           ...> task with the other strata's surface, the
                           divisor from the instance's own APB clock
    rtc.hpp                RTC + the backup domain (ch. 17): RtcDomain (PWR_CR's
                           DBP with the manual's read back, the whole of RCC_BDCR
                           - LSE, its bypass and drive, RTCSEL one-way with BDRST
                           the way back, RTCEN -, RCC_CFGR's RTCPRE; the LSI
                           stays Rcc's) + Rtc (the keys, initialization mode with
                           ES0287 2.8.4's workaround on every exit, the BCD
                           calendar read under 2.8.2's re-read, both alarms with
                           their sub-second match, the wake-up timer, the smooth
                           and coarse calibrators and the interlock between them,
                           the shift, the timestamp, one or two tamper inputs, the
                           RTC_OUT pad all of them share, the twenty backup
                           registers, and three ISR bodies that clear their own
                           EXTI line through exti.hpp and loop over the standing
                           flags - 2.8.3's workaround)
    tim.hpp                the timers (ch. 17..20): Tim<n> over one TIMx block
                           whose GEOMETRY is the manual's and not the header's
                           (two 32-bit counters, the break unit and RCR on the
                           advanced pair alone, TIM9/TIM12 slaving with no
                           encoder and no ETR, TIM9..TIM14 with no CR2 and so
                           no TRGO, TIM6/TIM7 with no channel) - the time base
                           with its two shadow registers, the channels in both
                           faces (CCyS writable only with the channel off), the
                           slave controller and the master TRGO, the internal
                           trigger table with the TIM8 entries derived from
                           presence, the option registers of TIM2/TIM5/TIM11
                           that reach the LSE, the LSI, the RTC wake-up and
                           HSE_RTC with no pad, the break and dead time, the
                           DMA burst engine, the rc_w0 flags and FOUR vectors
                           on TIM1/TIM8 three of which are shared + TimPad and
                           the nine tasks (TimPwm, TimPairPwm, TimPeriodMeter,
                           TimIntervalMeter, TimEventCounter, TimGatedCounter,
                           TimPeriodicTick, TimOnePulse, TimEncoder)
    adc.hpp                ADC: AdcCommon (the block up to three converters
                           share: the PCLK2 prescaler with its 36 MHz ceiling,
                           TSVREFE and VBATE, the multi-ADC modes and their
                           DMA modes, the ONE reset line) + Adc<1..3> (the
                           regular sequence of sixteen and the INJECTED four
                           that preempt it with a signed offset, the four
                           resolutions, per-channel sampling times, scan and
                           discontinuous, the analog watchdog, the timer and
                           EXTI triggers, the rc_w0 status register whose
                           result() acknowledges STRT too - there is no busy
                           bit - the ISR body, the DMA cell check) +
                           AnalogIn<Pin, channel> (the channel from the
                           family's own pad map by default) + AdcInput (the
                           three internal sources as TAGS: the sensor sits on
                           channel 16 or 18 by part class) + AdcFactory
                           (VREFINT_CAL and the two temperature points) +
                           Ref/ref_mv (a PAD: no buffer, no selector)
    dac.hpp                DAC: Dac, a MONOSTATE with two 12-bit channels
                           whose only route out is a PAD (no MCR, no internal
                           path - so the ADC reads it back through the bond
                           pad they share), the three data formats and the
                           three dual ones, the buffer DISABLED by a one, the
                           eight triggers gated on timer presence, the noise
                           and triangle generators, the DMA request only a
                           HARDWARE trigger raises and the underrun it leaves,
                           and the ISR body on TIM6's vector; absent where the
                           header declares no DAC_BASE
    spi.hpp                SPI and I2S (ch. 28): Spi<n> the resource over the
                           whole chapter - the F1 lineage's block with no FIFO,
                           eight rates off the instance's OWN APB clock, three
                           NSS arrangements, the simplex and bidirectional line
                           modes, the CRC unit whose polynomial must be ODD
                           (ES0206 2.12.3), the TI framing, flags cleared by
                           READ SEQUENCES and a disable that ignores BSY in
                           master receive-only (2.12.1) - + I2s<n> / I2sExt<n>,
                           the same block in its audio face (the four standards,
                           PCM's two frames, the I2SDIV/ODD arithmetic against
                           the audio PLL, the extension block the F446 has not)
                           + SpiHost<n, pins, TxEngine, RxEngine> with the other
                           strata's Request VERBATIM (the pump on RXNE, the
                           engines on the request mapping's own cells,
                           sck_speed() because 2.12.4 makes the SCK pad's slew
                           class a correctness parameter) + SpiClient<n, pins>
    i2c.hpp                I2C (ch. 27): I2c<n> the resource over the whole
                           chapter - the F1 lineage's event machine, the
                           clock arithmetic in THREE registers (FREQ, CCR
                           under F/S and DUTY, TRISE from the wire's rise
                           time), the two noise filters where the part has
                           them, 7- and 10-bit own addresses with the dual
                           address and the general call, SMBus and PEC, the
                           DMA requests with LAST, every flag with the
                           SEQUENCE that clears it, TWO vectors - +
                           I2cHost<n, pins, TxEngine, RxEngine> with the
                           other strata's Request VERBATIM (the receive
                           procedure by count, a BUS ERROR counted and never
                           acted on per ES0206 2.10.1, recover() the SWRST
                           2.10.3 prescribes, the pads handed over BEFORE
                           that reset because BUSY watches the peripheral's
                           own inputs) + I2cClient<n, pins>
    fmpi2c.hpp             FMPI2C (RM0390 ch. 23): the SECOND I2C design some
                           parts of this family carry - the STM32G0's register
                           file under another name, so FmpI2c<n> is that
                           driver's resource with an Fmp prefix (one TIMINGR
                           word solved both ways against tables 134/135, the
                           byte counter with RELOAD and AUTOEND, two own
                           addresses with a mask, the SMBus half with both
                           time-outs) - and this family's own differences: TWO
                           vectors, a kernel clock chosen in RCC_DCKCFGR2 (the
                           APB, SYSCLK or the HSI), the Fm+ pad drive in
                           SYSCFG, and NO wake from Stop (table 127's dash,
                           and the header has no WUPEN) + FmpI2cHost<n, pins,
                           TxEngine, RxEngine> with I2cHost's parameter list
                           and Request VERBATIM, so a program changes one type
                           name to move between the two blocks + FmpI2cClient
    usb.hpp                USB OTG in DEVICE MODE (ch. 22 / 31 / 34-35):
                           UsbOtg<core> - ONE template over the two DWC2
                           cores this family may carry (UsbFs on PA11/PA12,
                           UsbHs through its OWN full-speed PHY on PB14/
                           PB15) - realizing util/usb's UsbController at
                           the packet: the core reset and the 25 ms after
                           device mode is forced, the FIFO map the program
                           writes by hand out of a 320-word budget (a claim
                           past it REFUSED), the shared receive FIFO drained
                           to its first data packet a pass, an OUT transfer
                           of as many packets as a class has armed
                           (out_slots) because this core has no double
                           buffer, one packet per IN transfer with the
                           empty-level interrupt behind it, the address
                           written BEFORE the status stage because 22.17.5
                           and not chapter 9 is what the silicon obeys, and
                           the two device-mode errata as code (the guarded
                           FIFO write, the address never read back)
    crc.hpp                CRC: `Crc`, a monostate over three registers - the
                           CRC-32 Ethernet polynomial wired in (no polynomial,
                           no initial value, no reversal: the function IS
                           CRC-32/MPEG-2), a WORD the only grain the register
                           takes, the reset that lands a read late and
                           swallows a word written behind it, and CRC_IDR the
                           one piece of state a reset spares + `word_be`, the
                           packing that turns a byte stream into the words this
                           register takes; the polynomial in constexpr C++
                           (util/crc.hpp's crc32_ethernet*) is what the
                           silicon is judged against
    rng.hpp                RNG: `Rng`, a monostate where the part has one (not
                           the F401, F411 or F446) - the 48 MHz domain checked
                           against the chapter's RATIO and not against 48 MHz,
                           the FIPS first-value discard and the continuous
                           comparison inside read(), the seed error whose
                           recovery is a sequence and the clock error that is
                           not, two latched flags that are rc_w0, and one
                           vector under two names
    can.hpp                bxCAN: `Can<1|2|3>` over the whole chapter - the
                           three modes with their acknowledges, the bit timing
                           searched EXACTLY at compile time from PCLK1 with
                           its sample point, three transmit mailboxes ordered
                           by identifier or by request, two receive FIFOs of
                           three with the overrun policy RFLM chooses, the 28
                           filter banks of a block CAN1 owns and CAN2 shares at
                           CAN2SB (refused on CAN3, whose count RM0430 states
                           and this project has not read), the error counters
                           and LEC, four vectors and three ISR bodies, and the
                           TTCM the errata forbid REFUSED by part class -
                           over util/can.hpp's shared frame, timing and error
                           codes, with can_timing_limits binding CAN_BTR's
                           widths into the shared search, and CanFilter /
                           CanTxResult the bxCAN's own; no bus AO
                           (design/can.md)
    fmc.hpp                the FLEXIBLE MEMORY CONTROLLER (ch. 37 / 11), the
                           one peripheral that adds ADDRESS SPACE instead of
                           driving a wire - and the one chapter with two
                           different blocks in it, the FMC here and the FSMC
                           (the static banks alone, other register names) on
                           the F405 class, the F412 and the F413/F423, where
                           nothing of this file exists: Fmc (the AHB3 gate and
                           the reset that is the ONLY thing stopping the
                           memory clock, the six windows of figure 457, the
                           one shared vector, what this part has) +
                           FmcSdram<1|2> (the geometry and the seven delays in
                           the chapter's words, 37.7.3's sequence as ONE verb
                           because a read of a bank that has not run it HANGS
                           THE MACHINE with no fault, the refresh counter's
                           arithmetic and the two places the manual is wrong
                           about stopping it, self-refresh and power-down, the
                           status flags and the refresh-error ISR body) +
                           FmcNorPsram<1..4> (four sub-banks with a chip
                           select each, BCR/BTR/BWTR whole, the extended mode)
                           and the tasks FmcSram<n> / FmcNor<n>; the NAND and
                           PC Card halves are presence facts and no verbs
    ltdc.hpp               the LCD-TFT DISPLAY CONTROLLER (ch. 16), on the
                           parts with a panel interface alone: the pixel
                           packers (argb8888/rgb888/rgb565/argb1555/argb4444,
                           pure arithmetic, everywhere), LtdcTiming (a panel's
                           own eight numbers, the accumulation and the minus
                           ones this file's) with ltdc_frame_pixels /
                           ltdc_frame_rate_mhz / ltdc_fetch_bytes_per_second,
                           LtdcFramebuffer<Pixel> (a rectangle of memory as a
                           typed surface, WRITTEN and never read by the CPU -
                           ES0206 2.3.5), Ltdc (the block: the pixel clock off
                           PLLSAI's R output, the four timing registers, the
                           background, the dithering whose widths are READ-
                           ONLY, the shadow-reload discipline with the domain
                           race it covers for its caller, the four events over
                           TWO vectors and one ISR body taking the mask of the
                           pair its vector carries, the position counter) and
                           LtdcLayer<1|2> (the window counted from the back
                           porch, the eight formats, the frame buffer's three
                           registers with 16.7.23's "+ 3", the constant alpha
                           and the two legal blending codes of each factor, the
                           colour key, the default colour and the CLUT - the
                           one layer register that is not shadowed)
    dma2d.hpp              the CHROM-ART ACCELERATOR (ch. 11), on a WIDER set
                           of parts than the display: Dma2d over the four modes
                           as four verbs (fill, copy, convert, blend, with copy
                           refusing a pair of unequal WIDTH because that mode
                           does not convert), Dma2dSource / Dma2dOutput /
                           Dma2dArea (offsets in PIXELS, addresses in BYTES
                           aligned to the format), the eleven input formats and
                           five output ones, both colour tables loaded by the
                           engine or by the CPU, the six events, the abort and
                           the suspend, and the AHB dead time - the one knob a
                           program has for sharing a memory with a display
                           controller's fetch. 11.3.11's blend and the output
                           packings are constexpr beside the registers, which
                           is what a test judges the silicon against
  rp2040/                everything that knows the RP2040 (Raspberry Pi's dual
                         Cortex-M0+): the pico-sdk's CMSIS header + regs headers
                         are the device description (third_party/pico-sdk/)
    device.hpp             RP2040.h + every hardware/regs header but addressmap.h
                           (its *_BASE macros are the CMSIS header's), and the
                           atomic register aliases of 2.1.2 as hw_set/hw_clear/
                           hw_xor/hw_write_masked
    nvic.hpp               device.hpp + cortexm/nvic.hpp: TWO NVICs, every line
                           reaching both, a line enabled by one core
    ticker.hpp             cortexm/ticker.hpp + CoreTicker<core> (1000 Hz, one
                           per SysTick through BasicTicker's tag) and Ticker =
                           core 0's
    delay.hpp              cortexm/delay.hpp on clk_sys
    platform.hpp           Rp2040Platform<core = 0, TB = CoreTicker<core>>:
                           ONE PLATFORM TYPE PER CORE (the kernel statics are
                           keyed by it, so the type is the core), the PRIMASK
                           critical section that is PER CORE, WFI idle, a
                           .noinit breadcrumb per core, atomic_width 4,
                           core_id(), and the two optional members of a core
                           of several: on_own_core() (CPUID: a post to the
                           other core's queue is refused and counted) and
                           Doorbell (the SIO FIFO towards this core)
    multicore.hpp          SioDoorbell<core> (the inter-core FIFO as the
                           bridge's bell: ring from the other core, pop_all /
                           enable on this one) + Core1 (launch = the power-on
                           state machine's reset of core 1 THEN the bootrom's
                           six-word FIFO protocol; protocol alone; reset) -
                           SYSRESETREQ resets one core, so a launch resets
                           first and a probe's reset of core 0 is not a reboot
    timer.hpp              Timer: the 64-bit microsecond ruler on the watchdog
                           tick (raw-pair read from any context, the latched
                           pair for one), four alarms with their lines,
                           DBGPAUSE CLEARED at init (a halted core would freeze
                           the other core's ruler)
    watchdog.hpp           WatchdogTick + Watchdog (E1's double decrement as
                           code, PSM WDSEL selecting everything but the
                           oscillators, force_reset = the chip's one software
                           reboot) + Scratch<0..3> (the bootrom's four refused)
    reset.hpp              Reset (the causes as a HISTORY: chip-level flags
                           for the life of the supply, REASON the last watchdog
                           event; software() = the watchdog's trigger, the
                           CHIP; core() = SYSRESETREQ, this core alone, no
                           mark), ResetReporter, fault_reset<P>()
    resets.hpp             RESETS: the gate on every peripheral (held in reset
                           at power-up, released with a RESET_DONE wait);
                           ResetBlock masks
    sysinfo.hpp            ChipId (manufacturer, part, the silicon revision
                           the errata key on)
    clock.hpp              the FOURTH clock model: Xosc (startup delay),
                           PllSys and PllUsb (one PllBlock; the exact-ratio
                           search at compile time), Clocks (the glitchless
                           and aux muxes of clk_ref / clk_sys, clk_peri,
                           clk_adc and clk_rtc) + Clock<crystal|pll, hz, xtal>
    pin.hpp                Gpio (the bank: SIO word-wide verbs, the per-pin
                           CTRL and PAD registers, both blocks released by
                           every configuring verb) + Pin<n> (no port letter)
    sleep.hpp              DormantWake (the IO bank's dormant-wake events)
                           + Rp2040SleepSite (light = WFI, standby = the
                           SLEEP state under the program's SLEEP_ENx gates,
                           deep = DORMANT through the platform's sleep_hook,
                           refused with no way back) + Rp2040TimedSleepSite
                           (the timer as alarm and witness, the calendar for
                           a dormant on the ring oscillator)
                           + PinRef (a pin named at run time: a bus request's
                           select)
    spi.hpp                the SPI (4.4): TWO ARM PL022s, and THE DRIVER IS NOT
                           HERE either - brio/pl022/spi.hpp holds the resource,
                           the host engine and the client. Here: the pin table
                           of table 279 with its function code, Rp2040Pl022 the
                           CHIP TRAITS (register block, reset bits, NVIC lines,
                           atomic aliases, DREQs, the pad setup, the run-time
                           select pin a Request carries, the busy-wait
                           cs_setup_us is spent on, and clk_peri as SSPCLK) -
                           and the PUBLIC NAMES Pl022<n>, SpiHost<n, pins, ...>
                           and SpiClient<n, pins>. The measured fact this chip
                           adds: SOD DOES NOT RELEASE THE PAD, the block's
                           pad-enable output reaching no pad, which is why the
                           dark listener releases the PAD instead
    i2c.hpp                the I2C (4.3): TWO Synopsys DW_apb_i2c controllers,
                           the driver written once in brio/dw_apb_i2c/i2c.hpp.
                           Here: the pin table of table 279 with its function
                           code and the pad setup 4.3.1.3 asks for, and
                           Rp2040DwApbI2c the CHIP TRAITS - register block,
                           reset bits, NVIC lines, atomic aliases, DREQs, the
                           five open-drain verbs the unstick drives a line
                           with, the microsecond ruler it paces itself by, and
                           clk_sys as ic_clk - + the PUBLIC NAMES DwApbI2c<n>,
                           I2cHost<n, pins, ...> and I2cClient<n, pins>
    adc.hpp                the ADC (4.9): Adc, a monostate (its own clock from
                           the USB PLL or the crystal, the one-shot and the
                           free run paced by the 16.8 divider under the
                           conversion's 96-cycle floor, the round-robin, the
                           eight-entry FIFO as interrupt and DMA request, the
                           shift, the error flag; the sampler's converter
                           surface) + AnalogIn<Pin> (GPIO 26..29) + AdcInput
                           (the sensor the fifth) + Ref::vref_pin, the board's
                           millivolts stated by the application
    rtc.hpp                the RTC (4.8): Rtc, a monostate (clk_rtc from the
                           crystal over 256, the set that waits for the read
                           path and undoes the enable's own tick, the read
                           RTC_0 then RTC_1, the load while running, one alarm
                           on any subset of the seven fields whose ISR body
                           masks the line and disarms the match) +
                           RtcDateTime / RtcAlarm and the calendar arithmetic
                           the silicon has not (both leap rules, the weekday)
    pio.hpp                the PIO (chapter 3): the nine instructions as
                           constexpr encoders (side-set and delay folded in,
                           JMPs relocated at load), PioProgram<N>, Pio<n> (the
                           memory with a first-fit placer, the enables and
                           restarts, the flags, the two lines), PioSm<n, sm>
                           (the configuration, the FIFOs, an instruction on
                           the side, the pins claimed by a SET) + the
                           chapter's programs as tasks: PioUartTx / PioUartRx,
                           PioSquareWave, PioPwm (a PwmChannel)
    pwm.hpp                the PWM block (4.5): Pwm (the global enable for
                           lockstep, the four interrupt registers) +
                           PwmSlice<n> (a 16-bit counter, TOP, two levels,
                           the 8.4 divider, phase-correct, the B pin as gate
                           or clock, the phase nudges; configure() from
                           scratch, the vendor's order) + the tasks PwmOutput
                           (PwmChannel, max = TOP + 1), PwmPair (a dead time
                           by arithmetic), PwmEdgeCounter / PwmLevelCounter
                           (a frequency and a duty with no capture unit),
                           PwmPeriodicTick
    dma_engine.hpp         NoDmaEngine, the empty slot's tag, and the request
                           numbers (Dreq, table 119) a transport names - a
                           request being a FIELD any channel takes here, so a
                           transport names its own without including the block
    dma.hpp                the DMA (2.5): Dma (the block), DmaChannel<0..11>
                           (prepare/load/trigger, the abort with E13's
                           workaround, progress from TRANS_COUNT per E12),
                           DmaLine<0|1> (one per core by convention),
                           DmaTimer<n>, DmaSniffer, and DmaTxEngine/DmaRxEngine
                           <ch, Elem, line> for the transports' slots, and the rule
                           that two engines of one transport name two channels
    flash.hpp              the external QSPI chip: Flash (the bootrom's
                           six functions found by code, every erase /
                           program / raw command a WINDOW with the flash
                           disconnected - .ram_text, interrupts masked,
                           the second stage's SRAM copy re-entered -,
                           the JEDEC / unique / status ids) + Xip (the
                           cache, its counters)
    nvm_flash.hpp          QspiFlashPartition (the top 64 KB the linker's
                           flash region stops short of) + QspiFlash and
                           QspiFlashJournalZone, the two FlashMedia
                           (4096 / 256)
    usb.hpp                Usb: the chip's device controller as the stack's
                           endpoint controller - the dual-port RAM's endpoint
                           and buffer control words, the two-step offer, the
                           bulk OUT double buffered with the halves completing
                           in turn, the PIDs per endpoint and direction, the
                           events from BUFF_STATUS and the SIE, clk_usb from
                           the USB PLL
    uart.hpp               the UART (4.2): TWO ARM PL011s, and THE DRIVER IS
                           NOT HERE - it is written once in brio/pl011/uart.hpp.
                           This file is what that file asks of a family: the
                           pin table of table 279 with its function code,
                           Rp2040Pl011 the CHIP TRAITS (register block, reset
                           bits, NVIC lines, guard, platform, the atomic
                           aliases, the DREQs, the pad setup, and clk_peri as
                           UARTCLK so the divisor comes from Clock::pclk_hz and
                           never from a second statement of the rate) - and
                           the PUBLIC NAMES Pl011<n> and Uart<n, pins, ...>
  rp2350/                everything that knows the RP2350 (Raspberry Pi's
                         dual Cortex-M33 AND dual Hazard3 RISC-V over one
                         set of peripherals, exactly one pair running at
                         a time): the pico-sdk's rp2350 device description
                         vendored in an include root of its own, the
                         ARCHITECTURE an axis of the build, and no second
                         stage - the bootrom sets the XIP interface up
                         itself while it scans the flash
    device.hpp             the CMSIS header generated from the SVD + every
                           hardware/regs bit-field header but addressmap.h,
                           the atomic register aliases of 2.1.3 as hw_set/
                           hw_clear/hw_xor/hw_write_masked, and THE PACKAGE
                           as a build fact
                           and a silicon fact at once (QFN-60 = 30 GPIO and
                           four ADC inputs, QFN-80 = 48 and eight). The SAME
                           FILE ON BOTH ARCHITECTURES: the IRQ numbering is
                           shared (3.8.4.2), and the RISC-V build resolves
                           the header's core_cm33.h to a stub of this
                           project's own
    core.hpp               THE ONE FILE OF THE STRATUM THAT ASKS WHICH
                           PROCESSOR IS IN THE SOCKET. It asks `__riscv`
                           once, includes one of the two halves below, and
                           both export the same names - InterruptGuard, the
                           global mask verbs, Irq over the device header's
                           IRQn_Type, wait_for_interrupt, debug_break,
                           core_id - so no driver, no header above it and no
                           app ever knows; it also defines the macro
                           (BRIO_RP2350_CORE_M33 / _HAZARD3) that the two
                           files which cannot choose an #include by a
                           constant ask instead of the compiler
    core_m33.hpp           the Arm half: the device header + cortexm/nvic.hpp
                           as samc21/nvic.hpp and stm32f4/nvic.hpp are, plus
                           this core's instruction verbs. Everything runs
                           Secure, as the bootrom hands over; the MPU, the
                           SAU and TrustZone, MSPLIM, BASEPRI and the
                           coprocessor ports are deliberately untouched
    core_hazard3.hpp       the RISC-V half, this stratum's own until a second
                           family of the shape earns a core stratum: the
                           Xh3irq controller over the ARM interrupt numbers,
                           its ARRAY CSR idiom (a 16-bit window selected by
                           the low half of the value written), MEIPRA left at
                           reset because no interrupt nests, MSLEEP never
                           written (erratum RP2350-E4), and the fact the
                           idle path rests on - wfi IGNORES mstatus.MIE
                           (3.8.5), which is NOT the QingKe cores' wfi
    mtime.hpp              Mtime: the RISC-V platform timer (3.1.8), a 64-bit
                           counter in the SIO with a comparator per core that
                           the datasheet calls usable equally by either
                           architecture; counting the TICK and not cycles, so
                           it is a microsecond ruler a rate change cannot
                           move; DBGPAUSE cleared for both cores, and the
                           read and the comparator write as 3.1.8's own
                           sequences
    ticker.hpp             ONE SURFACE OVER TWO CLOCKS: cortexm/ticker.hpp's
                           BasicTicker on SysTick for the Arm half,
                           MtimeTicker on the platform timer for the RISC-V
                           one, written to the same verbs; CoreTicker<core>
                           (1000 Hz, one per core) and Ticker = core 0's, and
                           the vector name isr_systick bound on both halves
    delay.hpp              delay_us(clock, us) on the PLATFORM TIMER and not
                           on a core counter, so one implementation serves
                           both instruction sets: at least, never early,
                           no arithmetic on clk_sys at all, one microsecond
                           of resolution as the price, capped below one
                           kernel tick and REFUSED beyond it + DelayRate /
                           delay_rate, the pair the IP strata ask for
    platform.hpp           Rp2350Platform<core = 0, TB = CoreTicker<core>>:
                           ONE PLATFORM TYPE PER CORE and ONE FOR BOTH
                           ARCHITECTURES - everything target-specific is a
                           name from core.hpp - the per-core critical
                           section, the idle path free of a lost-wakeup
                           window in both spellings, a .noinit breadcrumb
                           per core, atomic_width 4, on_own_core() and
                           Doorbell; no idle_until(), and sleep_hook for a
                           low-power state that is not a sleep instruction
    multicore.hpp          SioDoorbell<core> - THE DOORBELL REGISTERS (3.1.6)
                           and not the mailbox FIFO, so the bell never fails,
                           never blocks, is correct rung from either core and
                           shares no channel with the launch - + Core1
                           (reset through the power-on state machine's
                           FRCE_OFF.PROC1, the bootrom's six-word protocol
                           on both architectures, and the entry shim that
                           gives core 1 what the ROM does not hand it: the
                           global pointer on one half, MSPLIM and CPACR on
                           the other); erratum RP2350-E2 costs the bridge
                           nothing, brio taking no SIO spinlock here
    timer.hpp              the system timers, TWO of them where the RP2040
                           had one: Timer<n>, a 64-bit counter of the
                           microsecond TICK its own generator in the TICKS
                           block makes, read as a raw triple, with four
                           alarms whose INDEX IS A TEMPLATE PARAMETER because
                           an alarm is claimed by the app binding its vector
                           + the two registers this chip added - SOURCE (the
                           counter off the tick and onto clk_sys cycles) and
                           LOCKED, which locks nothing (measured), so init()
                           CYCLES the block's reset line; DBGPAUSE cleared,
                           a halted core freezing no other core's ruler
    watchdog.hpp           Watchdog: the 24-bit countdown on its OWN tick
                           generator (the block no longer owns one), the
                           three WDSEL registers in their three tiers with
                           only the system one written here, REASON, and
                           erratum RP2350-E19's guard - FRCE_OFF cleared but
                           for PROC1 - before every reboot + Scratch<0..3>,
                           the four the bootrom's boot redirection leaves
                           free
    reset.hpp              Reset: chapter 7's three tiers, the chip-level
                           cause kept in the always-on power manager beside
                           the watchdog's REASON - and the measured fact that
                           a reboot does not appear in the chip-level word at
                           all, a watchdog event through the power-on state
                           machine being a SYSTEM reset; software() = the
                           watchdog's trigger, core() = the processor reset
                           the Arm half alone has, refused at compile time on
                           the other; ResetReporter, fault_reset<P>() bound
                           to isr_hardfault or isr_riscv_exception
    resets.hpp             RESETS: the gate on every peripheral (held in
                           reset at power-up, released through the SET/CLR
                           aliases with a bounded RESET_DONE wait) -
                           ResetBlock masks read from the device header, the
                           map NOT the RP2040's - + Psm, the power-on state
                           machine read for the watchdog's stage selection
                           and for the erratum's guard, FRCE_ON deliberately
                           absent
    sysinfo.hpp            ChipId (manufacturer, part, the REVISION the
                           errata are keyed by) + PACKAGE_SEL, how an image
                           that was not told its package finds out - and
                           asic(), which reads TBMAN's PLATFORM and not
                           SYSINFO's, the chip having two registers of that
                           name and SYSINFO's being the pre-production one
    clock.hpp              the RP2040's clock model with this chip's tree:
                           Xosc, Rosc (with the frequency randomiser),
                           Lposc (a fourth root of clk_ref, in the always-on
                           domain), PllSys/PllUsb over one PllBlock with the
                           STICKY LOCK-LOSS, Clocks (the glitchless and aux
                           muxes, 16.16 dividers, no clk_rtc), Resus,
                           TickGenerator<consumer> (8.5 as a block of its
                           own, one generator per timebase), FreqCounter,
                           ClockOut<n>/ClockIn<n> + the task Clock<source,
                           hz, crystal_hz, peri>; two CTRL registers are
                           PASSWORDS, where a masked write is a refused write
    pin.hpp                Gpio (SIO's word-wide verbs over TWO words, the
                           bank being 48 pins wide) + Pin<n> (no port
                           letter), the per-pin CTRL with its four overrides
                           and STATUS, the pad register - and THE ISOLATION
                           LATCH, this chip's own: PADS_BANK0 resets to 0x116
                           with the latch set and the input buffer off, so
                           every configuring verb writes the pad whole with
                           ISO CLEAR or the pad answers nothing. The pin
                           interrupts of 9.5, and erratum RP2350-E9's own
                           workaround as a verb pair (the buffer kept off,
                           read_pulsed() enabling it for the read alone)
    spi.hpp                the SPI (12.3): TWO ARM PrimeCell SSPs, and THE
                           DRIVER IS NOT HERE - the PL022 is written once in
                           brio/pl022/spi.hpp. This file is what that file
                           asks of a family: the pin table of 9.4 with its
                           ONE function column (unlike the UART, this chapter
                           gained no second), Rp2350Pl022 the CHIP TRAITS
                           (register block, reset bits, interrupt lines, the
                           atomic aliases, the DREQs, the pad setup, the
                           run-time select pin, the busy-wait, and clk_peri
                           as SSPCLK) - and the PUBLIC NAMES Pl022<n>,
                           SpiHost<n, pins, ...>, SpiClient<n, pins>
    i2c.hpp                the I2C (12.2): TWO Synopsys DW_apb_i2c
                           controllers, the driver written once in
                           brio/dw_apb_i2c/i2c.hpp. Here: the pin table's one
                           function code over forty-eight pads, and
                           Rp2350DwApbI2c the CHIP TRAITS - register block,
                           reset bits, interrupt lines, atomic aliases,
                           DREQs, the pad setup 12.2.1.3 asks for, the five
                           open-drain verbs the unstick drives a line with
                           over a BANK THAT IS TWO WORDS, the microsecond
                           ruler it paces itself by, and clk_sys as ic_clk -
                           + the PUBLIC NAMES DwApbI2c<n>, I2cHost<n, pins,
                           ...>, I2cClient<n, pins>
    adc.hpp                the ADC (12.4): Adc, a monostate - the package
                           decides the input map, so FIVE INPUTS OR NINE, a
                           four-bit AINSEL and a nine-bit round-robin where
                           the RP2040 had three and five, and init() reads
                           SYSINFO.PACKAGE_SEL and refuses when the silicon
                           disagrees with the build; the 16.8 divider under
                           the 96-cycle conversion, the eight-entry FIFO as
                           interrupt and DMA request, the sampler's converter
                           surface + AnalogIn<Pin> (whose claim DROPS THE
                           ISOLATION LATCH, the one configuration erratum
                           RP2350-E9 does not bite) + AdcInput (the sensor
                           LAST, its number the package's) + Ref::avdd_pin,
                           there being no ADC_VREF pin: the converter's own
                           supply is the full scale
    pio.hpp                the PIO (chapter 11): the nine instructions as
                           constexpr encoders with this chip's pio_put /
                           pio_get, PioProgram<N>, Pio<n> over THREE blocks
                           that form a ring, PioSm<n, sm> + the chapter's
                           programs as tasks (PioUartTx / PioUartRx,
                           PioSquareWave, PioPwm). What this chip added and
                           every item of it a verb: a VERSION field, GPIOBASE
                           - the window that says which thirty-two pads a
                           block sees, and which changes the meaning of every
                           pin number - a CTRL write reaching the blocks
                           either side, all eight flags on the lines, a
                           masked input count, and a receive FIFO that can be
                           four random-access registers instead of a queue
    pwm.hpp                the PWM (12.5): Pwm the block, PwmSlice<n> over
                           TWELVE slices where the RP2040 had eight - the
                           four highest reaching a pad on the QFN-80 alone
                           and being repeating timers on the QFN-60 - a
                           SECOND shared interrupt line with a register set
                           of its own (the line a template parameter, an app
                           binding isr_pwm_wrap_0 or _1), and a divider whose
                           last step is a whole 256, 12.5.2.6 forbidding a
                           fraction there + the tasks PwmOutput (PwmChannel,
                           max = TOP + 1), PwmPair, PwmEdgeCounter /
                           PwmLevelCounter, PwmPeriodicTick<n, line>
    dma_engine.hpp         NoDmaEngine, the empty slot's tag - and THE
                           REQUEST NUMBERS (12.6.4.1), which on this chip is
                           a FIELD any channel takes, so a transport names
                           its peripheral's request without including the
                           controller; not one row of the table is the
                           RP2040's
    dma.hpp                the DMA (12.6): Dma (the block, the security
                           assignment read back) + DmaChannel<0..15> (the
                           four aliases of which any may be the TRIGGER, the
                           MODE in the top nibble of the count that makes a
                           channel re-arm itself or run forever, an address
                           step backward or by twos, the abort with erratum
                           RP2350-E5's workaround) + DmaLine<0..3> (a line
                           belongs to the core whose kernel serves it) +
                           DmaTimer<n>, DmaSniffer, DmaMpu (read-only) +
                           DmaTxEngine/DmaRxEngine<ch, Elem, line> for the
                           transports' slots
    bootrom.hpp            the mask ROM's public function table (5.4): two
                           sets of well-known words, one per architecture,
                           and TWO DIFFERENT LOOKUPS over them - a pointer
                           on the Arm half, a jump instruction on the other,
                           chosen by core_kind and not by the preprocessor.
                           get_sys_info, the partition table and the ROM's
                           own revision are wrapped; the flash functions
                           belong beside the flash driver, reboot() would be
                           a second spelling of the reset chapter's, and
                           otp_access() is the OTP PROGRAMMING entry point
                           and so is not here at all
    flash.hpp              the external quad-SPI chip and the two blocks
                           between it and the bus: Qmi + QmiWindow<0|1> (the
                           SSI's replacement and not its rename - two 16 MB
                           windows, a transfer described phase by phase, four
                           translation panes a window, and a DIRECT MODE that
                           disconnects them all), Xip (the cache whose flush
                           is a MAINTENANCE ADDRESS and not a register, with
                           erratum RP2350-E11 answered in clean_all()) and
                           Flash, the engine over the bootrom's own functions
                           - every erase, program and raw command a WINDOW
                           with the memory interface disconnected. THERE IS
                           NO SECOND STAGE: the way back is the ROM's own XIP
                           setup function, copied out of boot RAM into SRAM
    nvm_flash.hpp          QspiFlashPartition (the top 64 KB the linker's
                           flash region stops short of, a CONSTANT floor
                           read back against __brio_flash_end) + QspiFlash,
                           the FlashMedia over it with THE PAGE AS THE CELL
                           (256 under an erase of 4096, the bootrom's
                           program function taking whole pages). A MEDIUM
                           AND NOTHING ABOVE IT: neither the heap nor the
                           journal is instantiated on this family, by the NV
                           stack review's decision, so a program gets a band
                           of flash and whatever it keeps there is its own
                           structure. A write is a HOLE IN THE PROGRAM and
                           not a wait
    sha256.hpp             Sha256, a monostate: the compression function of
                           FIPS 180-4 IN HARDWARE AND NOTHING ELSE - no
                           length register, no padding, no context to save -
                           so the padding is software's and hash() builds it
                           with util/sha256.hpp's own tail, the two unable to
                           drift apart; BSWAP left set and every word loaded
                           little-endian, which reconciles a little-endian
                           bus with a big-endian standard and makes a word
                           write and four byte writes one operation; the
                           WDATA_RDY handshake polled before every write with
                           ERR_WDATA_NOT_RDY a first-class verb, because a
                           suite that does not read it cannot tell a lost
                           word from a wrong answer; the DMA request is for a
                           WHOLE BLOCK, there being no FIFO
    trng.hpp               Trng, a monostate: a ring oscillator with no tie
                           to the clock tree, sampled every SAMPLE_CNT1
                           system cycles, three entropy checks of which
                           AUTOCORR_ERR is FATAL until the block is reset
                           (recover() the only way back) and two are the
                           block refusing entropy it does not trust, so
                           read_blocking() goes round again and COUNTS; a
                           generation time that is not deterministic, so
                           every wait is bounded and says when it ran out;
                           the sample interval stated as a TIME and turned
                           into cycles, a shorter one having been measured to
                           stop the block
    otp.hpp                Otp, THE READ SIDE ONLY AND DELIBERATELY SO: no
                           SBPI, no bootrom otp_access, not even a soft lock
                           written, because every bit of this array climbs
                           once and among them are the ones that remove a
                           debug port, a processor architecture or unsigned
                           code for ever. The four read windows of 13.1, two
                           of which FAULT instead of lying, with read()
                           asking the page's lock first so a caller gets an
                           empty optional and never a fault; the ECC of 13.6
                           decoded in software off the raw row, the hardware
                           correcting transparently and telling nobody; the
                           critical flags printed and never written
    usb.hpp                Usb: the chip's device controller as the stack's
                           endpoint controller - the RP2040's block with one
                           new duty, MAIN_CTRL.PHY_ISO, lifted LAST OF ALL
                           after everything else is configured - the
                           dual-port RAM's endpoint and buffer control words
                           with the two-step store 12.7.3.7.1 prescribes,
                           three reset values moved (so the registers are
                           written WHOLE), DP and DM as bank 1 pads that
                           could be GPIO and are kept from it by two facts,
                           erratum RP2350-E12 making CLK_SYS AT LEAST TEN PER
                           CENT ABOVE CLK_USB a compile-time assertion, and a
                           shelf of diagnostics that are a report and never a
                           path
    uart.hpp               the UART (12.1): TWO ARM PL011s, and THE DRIVER IS
                           NOT HERE - it is written once in
                           brio/pl011/uart.hpp. Here: the pin table of 9.4
                           with its TWO FUNCTION COLUMNS, every group's
                           flow-control pads being a second data pair (which
                           is why the IP file keeps the pad's function code
                           OPAQUE and this file makes it a TYPE),
                           Rp2350Pl011 the CHIP TRAITS - register block,
                           reset bits, interrupt lines, guard, platform,
                           atomic aliases, DREQs, pad setup, and clk_peri as
                           UARTCLK, so the divisor comes from Clock::pclk_hz
                           and never from a second statement of the rate -
                           + the PUBLIC NAMES Pl011<n> and Uart<n, pins, ...>
  gfx/                   drawing, pure and target-independent
    surface.hpp            Coord/Extent/Rect + clip() (16 bits over the WHOLE
                           domain, because the far edge is never formed) +
                           coord_max, the formats Mono and Indexed8, the
                           Surface concept (two verbs: a filled rectangle and
                           a run) and its ReadableSurface refinement NO
                           PRIMITIVE TAKES, Framebuffer over caller-owned
                           storage, Viewport as a view and not a mode
    draw.hpp               the primitives as free functions over the
                           write-only base: clear, set_pixel, fill_rect,
                           hline/vline, line, rect, circle, round_rect and the
                           filled forms, plus isqrt
    font.hpp               the Font concept: a font is a TYPE, so the cell is
                           a constant and the linker drops what is unnamed
    font_5x7.hpp           the whole printable ASCII in a six-by-eight cell,
                           stored BY ROWS (text is drawn as runs), a hollow
                           box outside the range
    text.hpp               text and text_field - opaque glyphs, a field padded
                           to a fixed width so "the old value is gone" is a
                           verb; generated a row at a time into a bounded
                           buffer and flushed as runs
    pen.hpp                Pen<S>: a cursor for TRACING (move_to/line_to, the
                           text run) and the colours; the shapes take their
                           own coordinates and leave it alone
    counting.hpp           Counting<S>: a Surface that forwards and counts
                           rectangles, runs and pixels. What an update costs
                           on a PANEL is a different question from what it
                           costs in memory, and this makes the answer a
                           measurement (design/gfx.md's cost section)
  host/                  the test target
    platform.hpp           HostPlatform (virtual clock, recording idle/break)
                           + HostCore<n> (the two-core host: the same platform
                           by type, a test-set current core, a counting
                           doorbell) for test_inbox
    sim_usb.hpp            SimUsb: a UsbController the host tests play the
                           host against, one packet at a time (NAKs, stalls
                           and refusals counted)
    sim_flash.hpp          SimFlash: FlashMedia over RAM for the host tests
                           (configurable geometry, power-cut injection,
                           simulated reflash, wear counters)
    sim_pl011.hpp          THE IP STRATA'S SECOND REALIZATION, one file each
    sim_pl022.hpp          (sim_dw_apb_i2c.hpp beside them): a PL011, a PL022
    sim_dw_apb_i2c.hpp     and a DW_apb_i2c MADE OF RAM, each with the chip
                           traits its IP file asks of a family answered by NO
                           FAMILY AT ALL - the register block an array at the
                           block's own offsets with the block's own reset
                           values, a reset that memsets it, an interrupt
                           controller that counts, pads that remember what
                           they were handed to, a busy-wait that counts
                           microseconds instead of spending them. That is what
                           proves the claim "this driver knows no chip", and
                           each header is compiled twice - by the host suite
                           and by a family's compile check - so the proof is
                           made once per compiler. What is NOT modelled is the
                           wire: a test that wants a received byte wants
                           silicon
    shared_segment.hpp     SharedSegment: a named region two processes
                           reach, OWNED BY THE PROGRAM - the name is the
                           contract and never a path (macOS has no
                           /dev/shm), at most 31 characters, sized ONCE at
                           creation, and carrying a boot id because after
                           an unlink a viewer's old mapping still points
                           at the old object (docs/host/simulator.md)
    sim_panel.hpp          SimPanel: the other direction - the contacts and
                           shafts a VIEWER writes and the program reads in
                           its idle path. A snapshot and not a stream, which
                           is why it is a region and not a socket: nothing
                           to frame, latest-wins by construction, and no
                           question about an absent viewer
    sim_display.hpp        SimDisplay: a framebuffer published into POSIX
                           shared memory for a viewer in another process to
                           read at its own rate - NOT a surface, it hands out
                           the bytes an ordinary Framebuffer draws into; the
                           name is the contract and never a path, and the
                           boot id is what tells a viewer the segment was
                           remade (docs/host/README.md)
    sim_input.hpp          SimButton<id> and SimEncoder<id> (its two pads as
                           ScannedInputs, the shaft turned by step()/spin(),
                           force() the door for injecting bounce and skipped
                           states): the world's side of a contact, whose
                           whole interface is set()
    gfx_reference.hpp      THE JUDGE of the drawing primitives: every shape
                           computed from its definition, per pixel, sharing
                           no arithmetic with what it judges; plus the ASCII
                           dump and the diff map a failure prints
```

## Build artifacts

`build-cmake/<preset>/`: `<app>.elf` / `<app>.hex` / `firmware-<app>.map`
/ `<app>.lst` (source-interleaved disassembly), all written directly by
`avr_add_app()`'s post-build step - one set per app, in the preset's own
build dir (the SAM project's `sam_add_app()` does the same, plus a
`.bin`). Host test binaries live in `build-cmake/host/`, and the
per-project app rosters `apps_avrdx.json` / `apps_samc21.json` in
`build-cmake/` itself.
