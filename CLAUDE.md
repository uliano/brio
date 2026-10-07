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

- `README.md` - brio's shopfront: what it is, a snippet, the ideas,
  the layering and the target table (`supported` / `in bring-up`: the
  ONE place a platform's maturity is stated). No history, no apps.
- `docs/README.md` - the map of the documentation and its rules.
- `docs/design/*.md` - the target-independent design, by intent: WHY
  and contracts. `overview.md` (philosophy, the governing rule, the
  layering, the style, "a driver is written from its chapter", "a
  contract's unit is the run"), `kernel.md`, `clock.md`, `serial.md`,
  `spi-bus.md`, `i2c-bus.md`, `can.md`, `ring.md`, `analog.md`,
  `nv-heap.md`, `nv-journal.md`, `power.md`, `meters.md`, `usb.md`,
  `runtime.md`, `benchmark.md` (the instrument a driver's cost is read
  with), `block-stream.md`, `gfx.md`, `simulation.md`; `architecture.svg`
  is the strata diagram. Each page says in its first paragraph what it
  decides.
- `docs/<target>/` - one folder per target, mirroring `brio/<target>/`
  (`avrdx/`, `samc21/`, `stm32g0/`, `stm32f4/`, `ch32v00x/`, `ch32vx03/`,
  `ch32x035/`, `rp2040/`, `rp2350/`, `host/`), plus one per stratum that
  is not a target (`cortexm/`, `pl011/`, `pl022/`, `dw_apb_i2c/`,
  `devices/`). `README.md` is the operational page (toolchain, board,
  probe, debugger and their quirks) and carries the DOCUMENT MAP - the
  state of the driver work is read there; beside it ONE document per
  peripheral in the shape docs/README.md prescribes (documents of
  record -> what the silicon does -> types and verbs -> how to use it,
  one example per use -> bench findings -> "Not covered yet"). No
  document carries a banner. "Not covered yet" is TWO LISTS - driver
  gaps, each with its REASON (a wire, a peer board, a meter; born with
  its first user; declined because ...), kept apart from implemented-
  but-not-bench-verified, each with what would measure it - and a gap
  the document's own findings cover is deleted; a document with nothing
  in either list has no such section. The public voice names a silicon
  by its PART and never by an ordinal, and no desk letter (the
  manifest's positions) appears outside private/.
- `docs/boards/`, `docs/probes/` - one page per board brio is tested on
  and one per probe (the flash mechanisms and their traps); `brio apps`
  lists the apps from their own headers. THE DESK - which board is
  plugged in where, the incidents, the end state - is `private/`, a
  git-ignored nested repository: `private/bench_boards.py` (the real
  manifest, loaded before the public example), `private/bench.md` (the
  diary), `private/TODO.md`, `private/briefs/` (the agents' briefs and
  reports), `private/memory/` (the assistant's memory).
- `docs/<target>/vendor/README.md` - the data sheets, manuals and
  errata by document number and the chapters used; PDFs are local and
  git-ignored; cite by SECTION ("DS40002247B 16.5.2"), never by page
  (pages move between revisions), and check a PDF's revision before
  trusting it.
- Headers - the canonical API reference; a header's comment explains
  the concurrency model and the WHY of each tradeoff. This file holds
  no inventory of them.

Rules (full text in `docs/README.md`): any change that alters a
documented decision updates the matching doc in the same change; docs
say today's truth only (no change history, no dates, no renames);
design/ and the target folders never reference individual apps; never
duplicate signatures into docs; new decisions go into `docs/design/`,
not here. This file has no decision log: `docs/design/*` is
authoritative.

## The project

`brio` (`brio/`) is a header-only C++23 (gnu++23) framework - bar the
one source file of its runtime - for bare-metal MCUs built around a
cooperative active-object kernel, written clean-room after Samek's book
(never the QP source). One flat namespace `brio`; one directory per
STRATUM under `brio/`; includes carry the stratum prefix
(`#include "avrdx/usart.hpp"`). A stratum includes only what lies below
it: the kernel includes nothing of brio, util/ includes kernel/ and
never a target, no target include leaves the target strata, and the
kernel never knows which silicon it runs on. What each stratum holds is
its headers' business (their comments) and its documents' (the map in
`docs/<stratum>/README.md`); here is the roster alone.

| stratum | what it is |
|---|---|
| `kernel/` | the AO kernel: the Platform concept, per-AO queues, the FSM, post/publish/reply, the two loans, time events, `Tenuto`, panic |
| `rt/` | the runtime every 32-bit image compiles - memcpy, memmove, memset, memcmp, strlen, memchr, abort - in the framework's one source file; libgcc and no C library (design/runtime.md) |
| `util/` | services over the kernel: streams and print, the benchmark instrument, rings, the bus arbiter with the SPI, I2C and CAN vocabularies, the serial port, the NV heap and journal, power, meters, block streams, the two-core inbox, the USB device stack, the test-bench grammar |
| `gfx/` | drawing, pure and kernel-free: surfaces, primitives, fonts, text, the counting surface (design/gfx.md) |
| `devices/` | what sits OFF the chip over a link the chip provides: the DCS vocabulary, a traits type per display controller, the serial DCS link over any family's SpiHost, the panel driver; includes kernel/, util/ and gfx/, never a family |
| `cortexm/` | the CORE stratum every Cortex-M family includes after its device header: NVIC + PRIMASK guard, the SysTick ticker, delay_us, the DWT cycle counter |
| `pl011/`, `pl022/`, `dw_apb_i2c/` | the IP STRATA: ARM's PrimeCell UART and SSP, Synopsys's DesignWare I2C - a peripheral DESIGN written once over a traits type the family supplies (register block, reset, interrupt line, guard, pads, clock, DMA requests), the family keeping the public names; born at the SECOND family whose register description is IDENTICAL |
| `avrdx/` | everything that knows `avr/io.h`: AVR DA/DB; bench chips AVR128DB48 and AVR128DA48 |
| `samc21/` | everything that knows `sam.h`: SAM C21, Cortex-M0+; bench chip ATSAMC21J18A |
| `stm32g0/` | everything that knows `stm32g0xx.h`: STM32G0, Cortex-M0+; bench chip STM32G0B1RE; compiles on all twelve G0 headers with the reserve deriving every vector from peripheral presence, the x0 value line pending a board |
| `stm32f4/` | everything that knows `stm32f4xx.h`: STM32F4, Cortex-M4F with the hard-float ABI; bench chips STM32F429ZI, STM32F446RE, STM32F411CE, STM32F469NI |
| `ch32v00x/` | the CH32V006 and the CH32V003, QingKe V2, RV32EC, NO vendor header (the map is the stratum's `device.hpp`), WCH's gcc with the `xw` extension and the hardware prologue |
| `ch32vx03/` | the CH32V203 and the CH32V303, QingKe V4B and V4F, two ISAs and ABIs, the STM32F1's peripheral generation under WCH's names, three device classes in one part reserve |
| `ch32x035/` | the CH32X035 and the CH32X033, QingKe V4C: one die in seven packages, a peripheral generation of WCH's own |
| `rp2040/` | the RP2040, dual Cortex-M0+, a kernel per core; the pico-sdk's device description vendored, never its runtime |
| `rp2350/` | the RP2350: a Cortex-M33 pair OR a Hazard3 pair over one set of peripherals, chosen by the image the bootrom finds, so the ARCHITECTURE is an axis of the build and every suite is written once and run twice; `core.hpp` the one file that asks `__riscv` |
| `host/` | the native target: the host platform, the chips made of RAM (a PL011, a PL022, a DW_apb_i2c, a flash, a USB controller, a SPI host at the request level, a DCS panel), the shared-memory display and panel, the gfx reference renderer |

Names are claims: a stratum is named for exactly the family it has
been proven on and widens only when a real chip proves it shares the
stratum (`samc21` is final; avrdx -> avrxt when an EA/mega0 part proves
it; stm32g0 shares its name with the G0x0 line, decided on the headers).
ONE NAME PER ARCHITECTURE on three axes: `brio/<arch>/` (stratum),
`docs/<arch>/` (docs), `<arch>/` (the build project); chip precision
lives in preset names, per-chip ld/svd files and the `*_MCU` cache
variables. A core or IP stratum is factored at the SECOND family that
carries the block, every image byte-identical before and after, never
earlier - Hazard3 is not a QingKe, so the RP2350's RISC-V core file
stays its own until a second family of that shape earns one, and a
`qingke/` core stratum is due at the CH32X035, the third QingKe family.

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
restrictions.

Its twin is DERIVATION BY COPY: a new chip's engine written from the
nearest existing stratum's, its shape justified by a chip it no longer
runs on. It produces drivers that pass every suite while running a FIFO
one entry deep, programming a DMA channel whole for every block, keeping
one SPI frame in flight - and measured rates several times slower than
the wire filed as facts instead of findings. The definition of done
judged behaviour and never cost, so nothing caught it. The antidote to
both, in practice:

- **Framework, not application.** The target is the whole AVR DA/DB
  range (and future targets), the AVR128DB48 is only the test vehicle.
  Cover the chapter's FULL option space - every instance, mode, route
  from the register description, both errata documents (DB
  DS80000915F and DA DS80000882C differ). Leave something out only
  knowingly and declare it in the doc's "Not covered yet".
- **Silicon, not sibling.** The SURFACE (task names, the Request, the
  verbs, the events) is copied between strata on purpose; the ENGINE is
  written from THIS chip's chapter, starting from the inventory of what
  the silicon offers that bears on cost - FIFOs and thresholds, DMA
  requests and circular mode, the idle-line and time-out edges, byte
  counters and automatic STOP/RESTART, set/clear aliases, timer widths,
  the instruction fetch - each item used or declined with its reason in
  the document. A comment justifies an engine choice from the manual's
  section, never from "as the other strata do". The vendor's library is
  read BEFORE the engine is written, as the oracle of shape and cost
  (how it does each operation, what that costs; brio need not win, a gap
  over about a fifth is a finding). The hot path is read in the release
  disassembly (no call per byte or frame, the request's fields outside
  the loop, configuration at `arm()`/`init()` and only address, count
  and enable per operation, a critical section around the decision and
  never around the work). The byte-identity gate proves that a change
  moves nothing; it is never the reason a better default stays off.
  Contracts are drawn against the most capable silicon in view: the unit
  is the run, the byte its degenerate case. None of this fixes an answer
  (a DMA block forbids the CH32V203's sleep; a FIFO threshold starves a
  console's tail): it forbids choosing without asking. Full text:
  design/overview.md, "A driver is written from its chapter" and the
  three bullets after it, and "A contract's unit is the run".
- **Definition of done for a driver**: (1) systematic pass over the
  chapter's register description + errata; (2) a smoke TU compiled for
  every package - `avr-g++ -mmcu=avr128d{a,b}{28,48,64} -std=gnu++23
  -Os -c -I brio` takes seconds, no hardware - and `brio check` then
  LINKS every app on every preset of the family (`brio fit` the census
  of each image's flash and RAM, under 5 % left regrouped); (3) negative
  tests: what must be refused must FAIL to compile; (4) the
  `test_<target>_<subject>` suite on the bench; (5) `brio prose`
  clean over the files touched, and `brio gate` for every change that
  claims to move no image (a comments-only edit, a rename, a move) - a comment or a document is a reference
  for the code as it is, and a claim about another part of the tree is
  a POINTER the tool can check, never a statement that ages; (6) the
  silicon's offer inventoried in the document, each feature used or
  declined with its reason; (7) THE COST of every transport and engine -
  cycles per byte or frame, interrupts per byte, cycles masked - counted
  in the release disassembly, measured by a suite letter and stated in
  the document beside the wire's or the bus's figure, a rate far from
  that figure explained or listed as a gap. The bench chip alone
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
  (Its first use is earlier and routine - read before an engine is
  written, for shape and cost, "Silicon, not sibling" above; this is its
  use as a debugging instrument.) The trigger is a written list of measured variants (sequence, memory,
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
push/pop (take, hold and release for an element handed over in place); `std::optional` returns instead of bool + out-param; no
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

Roughly ordered by proximity. None is a decision yet; each gets its
home in `docs/design/` when taken. The state of every stratum is its
README's document map and the target table of `README.md`; the gaps
are each document's two lists; the running log and the decisions
posed to the user are `private/memory/`'s. What stays here is the
horizon itself, one entry each.

- **The harmonization pass** - the rules everything born after is born
  under (the rename, the voice pass, the prose net, the gap lists, the
  realizations tables indexed in overview.md). What remains: the
  public/private cut's last pieces, then a first public release with a
  fresh root at tag 0.1, the development history kept whole in a
  private archive.
- **The first portable example application** (an ILI9481 display over
  SPI, or an MCP47CVB22/MCP3550 DAC-ADC loop): application logic
  target-free over util and tasks, one thin board file per target. It
  births the board-file shape and chapter one of a learning track, and
  closes the open point on TRANSFER GRANULARITY (the frame, the access
  and the machine word are three widths; "the element type is the
  beat" is the engines' rule the SPI Request does not yet spell),
  measured on a block of 16-bit pixels.
- **The display stack** (design/gfx.md, "The command tier"): the DCS
  vocabulary, the traits, the simulated panel, the link over any
  SpiHost and the simulated SPI host, the panel driver in its DIRECT
  shape are code with their host suites (brio/devices/, brio/host/,
  test/test_dcs*, test/test_ili9481); next the tiled shape, the touch
  and the viewer's mouse, then the black pill against the glass, then
  the F469's DSI link as a DcsLink and the NT35510 as a second traits
  type.
- **The CH32V00x stratum's second part**: `supported` on the
  CH32V006K8U6; the CH32V003F4P6 is the SECOND AND LAST part (by
  decision), its tier OPEN - six chapters closed on it by name until
  written against its own register description (docs/ch32v00x/README.md).
  It is the most extreme point brio touches (16 KB, 2 KB, no
  multiplier), the reason for WCH's gcc and the smallest-chip rule
  (design/overview.md).
- **The STM32F4 stratum**: `supported` on four boards, every chapter
  with its document and suite (docs/stm32f4/README.md). Outside the gap
  lists: the OTG HS core in full-speed mode on the F429 (never
  enumerated), the frequency ladders of the four part classes whose
  manuals are not on the desk, the debugger from the command line, and
  the 32F469IDISCOVERY's audio side and card socket (SAI, I2S receive,
  SDIO: no driver, stated on the board's page). Tenuto only; Rubato and
  BASEPRI are another type and another day.
- **The RP2040 stratum**: `supported` on two boards, every chapter;
  THE SECOND CORE RUNS A KERNEL OF ITS OWN (design/kernel.md section
  12, docs/rp2040/multicore.md); the USB device stack was born here.
  What remains is in the documents' gap lists (a power vote across the
  cores, the bus fabric's counters).
- **The RP2350 stratum**: `in bring-up` on the WeAct RP2350B, written
  ONCE and run TWICE (M33 and Hazard3), every chapter green on both
  halves, the power chapter included; two declared gaps above the lists:
  the HSTX (born with its first user) and the M33's coprocessors
  (declined: one architecture of two has them). The bench verb is
  state-independent (a rescue over the debug port) and does NOT reach a
  chip whose switched core is powered down (docs/rp2350/powman.md).
- **The NV stack reviewed as a whole**: the flash heap and the journal
  were born where flash was cheap to partition and carried everywhere
  since; the STM32F4's geometry pushes back, and the question is prior:
  WHICH programs need a flash-backed store, blocks or small values, and
  whether a zone at a fixed address is worth a partition every image
  pays. Until that review, no FlashMedia on the STM32F4 and no heap or
  journal on the RP2350. Stalled by the user's call, taken up later.
- **The CH32V203/CH32V303 stratum**: `supported` on the CH32V203C8T6,
  the CH32V303VCT6 `in bring-up` with every chapter green there too
  (the promotion is one word; what is excluded and why is in
  docs/ch32vx03/README.md). Findings that shape it: in a sleep of any
  depth the bus matrix serves the core alone (the bus-master count),
  the CH32V303 has no USB device controller, the bench CH32V303VCT6 is
  a lot its class's notes restrict (every lot-keyed register is a verb
  that asks the die), gcc folds two handlers of one body (no_icf on both
  WCH strata). Owed on a CH32V203C8 board: the suites the CH32V303's
  changes moved. Decisions for the user: THE FPU TAX under ilp32f (a
  non-leaf handler saves twenty f-registers: a leaf ISR path, a
  soft-float configure, or the tax as it stands), the kept copy of
  PWR_CTLR's retention bits starting at zero at every boot, and the
  reference manual's V2.5 for promotion (docs/ch32vx03/vendor/README.md).
- **The CH32X035 stratum**: `in bring-up` on the CH32X035F8U6, the
  register map, the part table, the platform, the clock, the pins and
  the USARTs with their suites green; what the silicon taught is in
  docs/ch32x035/README.md. Owed: the hardware prologue's pair, the
  HSI's accuracy, the lock, every part but the F8U6. Then, by the user's
  pick, one chapter new to brio (the USB PD sink, or the host/device
  controller as the console - the CH32V303's block, to be factored into
  an IP stratum at its second family). The PIOC is out; the rest is
  born with its first user. A `qingke/` core stratum is due.
- **The optimization rounds after the retrospective review** (the
  method and the state in private/memory/'s critical-review note): the
  rulers and the platform, the util contracts, the DMA engines, the
  SPI hosts, the UART and I2C transports and the kernel (the index
  carousel, the idle promise measured on every core, the image's
  hold-off) are done, each with its letter in `benchmark.md` or its
  suite. What remains: `benchmark.md`'s skeleton table re-measured
  whole, and a recovery session on the parts the rounds did not run
  (the AVR128DA48, the G071RB and G031K8, the F429's spi suite, the
  F411CE and F469NI, the Pico).
- **Test consolidation per platform** when its chapters are closed: a
  two-level TestBench (groups over letters), few units per platform by
  domain, one logical unit on the host side, an .md per unit - the
  group being the unit an image carries, sized by the family's smallest
  chip.
- **Queued**: the SAM's CAN (its M_CAN would be the second FD
  controller design/can.md's shared FD frame waits for), the energy
  experiment's G0 instance, Multislope (an application), avrdx -> avrxt
  when a part proves it.
- **Borrowed, phase 2 (debug epoch)**: `Borrowed<T, Lease::dispatch>` is
  a plain pointer today; in debug builds an 8-bit lender epoch compared
  on every access would panic a stale loan on the guilty instruction.
  Built with the first host test that simulates preemption; not before.
- **HSM**: the FSM contract is HSM-ready (`unhandled` = future
  bubble-to-parent); parent pointers, bubbling and LCA chains are built
  only when a real AO demands them.
- **A watchdog keeper**: the strata kick their watchdog under two names
  and three contracts (design/kernel.md's table); the portable verb is a
  util concept born with the first portable program that keeps one.
- **QK-style preemption (far horizon, probably not on AVR)**: a SECOND
  kernel type beside `Tenuto`, named `Rubato`, chosen per board file -
  the AO contract and the three delivery primitives unchanged, the
  loans still correct, `post()` triggering the scheduler, an ISR-exit
  hook on the Platform. The discipline kept NOW so the door stays open:
  AOs share nothing but events.
- **C++ modules: considered, not now**: the prize would be macro
  isolation, not build speed; the blocker is the language server.
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
brio check ch32vx03 [name]      # same for the ch32vx03 stratum (the thirteen parts of the two
                                # series under TWO ISAs - the V4B's and the V4F's -, both
                                # HPE ways; util_all.cpp = the whole of kernel/ and util/ over
                                # both ABIs, ilp32 and ilp32f)
brio check ch32x035 [name]      # same for the ch32x035 stratum (the seven parts of the series - one die,
                                # seven packages - both HPE ways; util_all.cpp = the whole of kernel/
                                # and util/ over the platform)
brio check rp2040 [name]        # same for the rp2040 stratum (one chip: every header's verbs, util_all.cpp)
brio check stm32f4 [name]       # same for the stm32f4 stratum (ALL TWENTY-THREE F4 headers; the ladder
                                # refused by name where no manual was read)
brio check rp2350 [name]        # same for the rp2350 stratum - the one fixture that crosses TWO
                                # COMPILERS: every TU built four times (Cortex-M33 and Hazard3, each
                                # for the QFN-80 and the QFN-60), util_all.cpp through both
brio check all                  # every stratum above, in a row; each check ends with the
                                # LINK GUARD: every app built on every release preset of
                                # the stratum's project (build-cmake/fit/), a failure fatal;
                                # ch32vx03's adds the VECTOR GUARD (cli/vector_guard.py): on
                                # the F images every naked vector's body walked in the ELF,
                                # any f-register instruction or unresolved indirect call fatal
brio fit [stratum|all]          # that build alone, and the census: each image's flash and
                                # RAM against its linker script's regions, by margin
brio prose [paths...]           # the prose net: no dates/process words/Doxygen tags in
                                # comments and docs, every cited path exists, ASCII only;
                                # "review" lines are claims of absence to re-read, not errors
brio gate [--against REF]       # THE BYTE-IDENTITY GATE: reference and working tree each built
                                # with mtimes pinned and build dirs wiped, images compared per
                                # preset, movers named (the release presets cli/gate.py's
                                # DEFAULT_PRESETS names: every build project, two on the ch32v00x -
                                # one per part - two on the ch32vx03 - one per series - and two on
                                # the rp2350, one per architecture)
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
(cd ch32vx03 && cmake --build --preset ch32v203c8-release --target <app>)          # CH32V203 release build (WCH gcc 15, ilp32)
(cd ch32vx03 && cmake --build --preset ch32v203c8-release --target <app>-upload)   # flash it: `reset run` does NOT start the program here, `reset halt` + `resume` does
(cd ch32vx03 && cmake --build --preset ch32v303vc-release --target <app>)          # CH32V303 release build (the V4F: rv32imafc_xw, ilp32f)
(cd ch32x035 && cmake --build --preset ch32x035f8-release --target <app>)          # CH32X035 release build (WCH gcc 15, rv32imac_xw/ilp32)
(cd ch32x035 && cmake --build --preset ch32x035f8-release --target <app>-upload)   # flash via WCH's OpenOCD fork (a WCH-LinkE on PC18/PC19), `reset halt` + `resume`
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
  the CH32V203's four 32 KB parts, the G031K8, the SAM C21 - and
  `// build: groups.v006k8 = ...` on that board type alone),
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
  (`/sw/src/build-riscv-elf.sh`, `rp2350/cmake/toolchain-riscv.cmake`)
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

The repo root is not a CMake project: the builds are sibling CMake
projects, PEERS, one per target plus `test/` and `host/`. Every CROSS
project has the same shape - `CMakeLists.txt` (app auto-discovery:
one `main()` per `src/apps/<app>.cpp`, each app's `// build: <option> =
<value>` header lines read at configure, `<project>_add_app()` with
the flags, the `.hex`/`.lst`/`.map` post-build and a per-app `-upload`
target), `CMakePresets.json` (one configure+build preset pair per bench
chip x {release, debug}, binaryDir under the shared `build-cmake/`),
`cmake/toolchain-*.cmake` (the cross toolchain by absolute path),
`ld/<part>.ld`, `src/apps/`, `src/glue/` (the crt and what every image
compiles), `svd/` (the debugger's register map) - and the particulars
below. `// build: boards = ...` gates which board types build an app;
`// build: groups = ...` splits a suite into one image per group on a
board type whose project lists it as splitting (the CH32V003, the
CH32V203's four 32 KB parts, the G031K8, the SAM C21) and
`// build: groups.<board type> = ...` on that board type alone.

| path | what it holds |
|---|---|
| `brio/` | the framework, one directory per stratum (the roster above); the headers' comments are the reference, `docs/<stratum>/README.md` the map. `brio/.clangd` routes each stratum to its own compile database; a fragment in each stratum and each project beside it |
| `avrdx/` | the AVR project: `cmake/avr-mcus.cmake` (package -> mcu), `cmake/avr-refused-libc.rsp` (the linker's --wrap list refusing avr-libc's heap and stdio), `src/glue/ivsel_boot.cpp` in every image (vectors at BOOT start), the FLMAPLOCK and build-id defsyms; presets for the AVR128DB's three packages and the AVR128DA48 |
| `samc21/`, `stm32g0/` | the SAM C21 and STM32G0 projects in the shared shape; the G0's presets for the G0B1RE, G071RB and G031K8 with one ld and crt per part; the GROUP axis on the G031K8 and the SAM's one part |
| `stm32f4/` | a PART TABLE (`cmake/stm32f4-parts.cmake`: part -> ST's device define, crt stem, board type), presets for the F429ZI, F446RE, F411CE and F469NI, the hard-float flags |
| `ch32v00x/` | WCH's gcc at /sw/wch-riscv; `CH32V00X_MCU` names the part and derives its definition, ISA and ld; `src/glue/startup_ch32v00x.S` (the table whose first word is an instruction, two entries shorter on the CH32V003); the GROUP axis on the CH32V003 (and per board type); the crt paints the free RAM |
| `ch32vx03/` | the same compiler with the full register file; a PART TABLE of the thirteen parts with their memories, board type, ISA and ABI (`cmake/ch32vx03-parts.cmake`); one ld per part, each stopping 4 KB short of the zero-wait window; a crt with a vector tail per device class and the FPU enabled under F; the GROUP axis on the four 32 KB parts (and per board type); the C6 preset as the 32K tier's link guard |
| `ch32x035/` | the shape of the two sibling WCH projects: a part table of the seven packages, one ld giving the image the whole 62 KB at the alias, a 55-word crt; no group axis, no svd |
| `rp2040/` | the RP2040 project: the boot stage checked in as bytes, its own crt, `ld/` with `.ram_text` carried by `.data` (the flash engine and the PL022 host's hot path); the WeAct board's flash wants the OpenOCD built from git |
| `rp2350/` | THE ONLY PROJECT WHOSE AXIS IS THE COMPILER: `cmake/toolchain-arm.cmake` and `cmake/toolchain-riscv.cmake` are two configures of the same sources, each setting `RP2350_ARCH`; a flash geometry and a PACKAGE (QFN-60 or QFN-80, a pad the package has not got a compile error) per configure; two crts binding the same handler names, each placing the IMAGE_DEF block - there is no second stage; the `-upload` target a rescue over the debug port, then programming as core 0 of the Arm pair, then a reset |
| `host/` | the host's own programs, which RUN rather than run and exit (first: `supply_panel`); the same `// build:` grammar, nothing registered with ctest; flags identical to `test/`'s |
| `test/` | the host test project (native g++, non-recovering UBSan): one executable and ctest entry per `test_*/main.cpp` - the kernel, util, gfx, devices and IP strata against their simulated chips, the runtime's cases (`test_rt`); and the family compile fixtures `family_<stratum>/` with their `neg/` TUs (`brio check`) - every fixture directory of a family with an SpiHost compiles the DCS link and the panel over that host |
| `third_party/` | vendored, licensed: doctest, the SAM C21 DFP, ST's cmsis-device-g0 and -f4, ARM's CMSIS-Core, the pico-sdk's device descriptions in TWO include roots (the RP2040's and the RP2350's, same file names) with a stub core header for the RISC-V build - never the SDK runtime |
| `bin/brio`, `cli/` | THE ONE COMMAND of the bench and the repository, dispatching on its verb: `gate.py` (the byte-identity gate over the images and the token-identity check over sources), `prose.py` (the prose net), `vector_guard.py` (the CH32V303's naked vectors kept float-free), `check.py` over `cli/checks/` (the family compile fixtures as shell scripts, each 32-bit one ending with the runtime's checks); `cli/bench/` the verbs that need a board - `verbs.py`, `common.py` (BOARD_TYPES: a board type -> its project, preset, mcu and flash mechanism; the app rosters `build-cmake/apps_<project>.json`), `manifest.py` (private/bench_boards.py before the public example), `bench_boards.py`, `flash.py` (build, then avrdude / OpenOCD / the ST-LINK by board type, the RP2350's two sessions), `fuses.py`, `console.py` (the suites' console protocol: run, console, duo), `stress.py` (the host end of the UART suites) |
| `experiments/` | one self-contained directory per cross-cutting bench experiment (app halves per architecture globbed by the projects, the wire protocol, its own README, a python driver, logs git-ignored); `docs/` never references one |
| `docs/` | README (map + rules), `design/`, one folder per target and per shared stratum, `boards/`, `probes/` |
| `private/` | the desk (git-ignored nested repository): the manifest, the diary, the TODO, the briefs and reports, the memory |

Build artifacts: `build-cmake/<preset>/<app>.elf`, `.hex` (`.bin` on the
ARM projects), `firmware-<app>.map` and `<app>.lst` (the source-
interleaved disassembly), one set per app in the preset's own build
dir; host test binaries in `build-cmake/host/`; the per-project app
rosters in `build-cmake/` itself.
