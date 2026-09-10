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
  `block-stream.md` (block streams: BlockSource/BlockPlayer concepts
  over caller-owned buffers - blocks, not DMA - and the BlockRelay AO
  lending each filled block for one dispatch; built BEFORE its second
  implementation as the fixed point the next platform is measured
  against).
- `docs/<target>/` - one folder per target, mirroring
  `brio/<target>/` (`avrdx/`, `samc21/`, `stm32g0/`, `ch32v00x/`,
  `host/`): `README.md` is the
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
namespace `brio`; eight strata under `brio/` - `kernel/` (pure
logic, includes nothing of brio), `util/` (services over the kernel),
`armv6m/` (the CORE stratum both Cortex-M0+ families include after
their device header: NVIC + PRIMASK guard, the SysTick ticker),
`avrdx/` (everything that knows `avr/io.h`: AVR DA/DB, bench chip
AVR128DB48), `samc21/` (everything that knows `sam.h`: SAM C21,
Cortex-M0+, bench chip ATSAMC21J18A), `stm32g0/` (everything that
knows `stm32g0xx.h`: STM32G0, Cortex-M0+, bench chip STM32G0B1RE on
a Nucleo-64), `ch32v00x/` (everything that knows the CH32V00x: WCH's
QingKe V2C, RV32EC, bench chip CH32V006K8U6 - NO vendor header, the
register map is the stratum's own device.hpp), `host/` (the native test
target). Includes carry the stratum prefix
(`#include "avrdx/usart.hpp"`). The builds are five sibling CMake
projects, PEERS - the repo root is not a CMake project: `avrdx/`,
`samc21/`, `stm32g0/` and `ch32v00x/` (each with its own toolchain file
and presets, Ninja, emitting into the shared `build-cmake/`)
auto-discover one `main()` per `src/apps/<app>.cpp` at configure time
from its own `// build:` header comment; host tests in `test/` are the
fifth project (host g++, no cross toolchain), run via `ctest`. ONE NAME PER ARCHITECTURE,
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
`armv6m/` core stratum (nvic, ticker, delay) is what the two Cortex-M0+
families share, factored with both in hand and every image of both
byte-identical before and after; a RISC-V core stratum would be
factored the same way, at its second family, never earlier.

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
- **The CH32V00x stratum, from bring-up to supported.** `brio/ch32v00x/`
  and `ch32v00x/` exist (`in bring-up` in README.md's table): the kernel
  console runs on the CH32V006K8U6 at 48 MHz over USART1, on WCH's gcc
  15.2 and WCH's OpenOCD fork through a WCH-Link. What remains, in
  docs/ch32v00x/README.md's gap lists: the chapters (EXTI, TIM, ADC,
  I2C, SPI, DMA, flash, the power modes, reset), the family tiering and
  `brio check ch32v00x` with a second part, a self-built upstream gcc 16
  for riscv32 with an rv32ec/ilp32e multilib (the stratum compiles with
  plain rv32ec_zmmul on purpose - WCH's `xw` extension is worth a few
  per cent and only their compiler emits it), and the CH32V003 at 16 KB
  / 2 KB as the most extreme point brio can touch. A `qingke/` core
  stratum only at a second QingKe family.
- **Test consolidation per platform** when its chapters are closed: a
  two-level TestBench (groups over letters), few units per platform by
  domain, one logical unit on the host side, an .md per unit.
- **Queued**: the SAM's CAN (two transceivers and the util vocabulary a
  shared frame type must carry), the energy experiment's G0 instance,
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
  loop as the idle context, time-event maturation revisited. The
  discipline kept NOW so the door stays open: AOs share nothing but
  events.
- **C++ modules: considered, not now.** The prize would be macro
  isolation (`import brio.avrdx` would not leak `avr/io.h` above the
  target stratum), not build speed; the blocker is the language server.
  Revisit with the board files.

## Build, test, debug (the must-knows)

```bash
# Three sibling CMake projects, PEERS (none is the repo root): avrdx/,
# samc21/, test/. cmake presets resolve against their own project dir -
# run cmake FROM that dir (or let bin/brio do it).
(cd test  && ctest --preset host)                                  # host tests (doctest); no hardware needed
brio check avrdx [name]         # every avrdx smoke TU compiles for all 8 DA/DB packages;
                                # neg/ TUs must FAIL (definition of done)
brio check samc21 [name]        # same for the samc21 stratum (E/G/J 18A headers)
brio check stm32g0 [name]       # same for the stm32g0 stratum (ALL TWELVE G0 headers, x1 + x0)
brio check ch32v00x [name]      # same for the ch32v00x stratum (one part today; util_all.cpp = the
                                # whole of kernel/ and util/ through WCH's gcc 15.2)
brio check all                  # the four in a row
brio prose [paths...]           # the prose net: no dates/process words/Doxygen tags in
                                # comments and docs, every cited path exists, ASCII only;
                                # "review" lines are claims of absence to re-read, not errors
brio gate [--against REF]       # THE BYTE-IDENTITY GATE: reference and working tree each built
                                # with mtimes pinned and build dirs wiped, images compared per
                                # preset, movers named (the three release presets, ~30 s)
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
  exist), IDENTITY = the manifest `cli/bench/bench_boards.py` (which board
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
  point there), each by absolute path; never a system-packaged one. Never add
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
                         CH32V00X_ARCH = rv32ec_zmmul by choice, one preset
                         pair for the CH32V006K8, ld/ch32v006k8.ld,
                         src/glue/startup_ch32v00x.S - the table whose
                         first word is an INSTRUCTION and whose handler
                         names are the project's own - and the upload
                         target on WCH's OpenOCD fork); no vendor header,
                         so no device-select define
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
third_party/cmsis-core/  vendored ARM CMSIS-Core headers (Apache-2.0)
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
                         v006k8 -> ch32v00x/WCH's OpenOCD fork/WCH-Link),
                         the per-project app rosters build-cmake/apps_{avrdx,
                         samc21,stm32g0}.json (each project writes its own at
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
                         the flash-heap preflight on the AVR
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
docs/                    README (map + rules), design/, <target>/ (avrdx/, samc21/,
                         host/), boards/, probes/
brio/.clangd             per-stratum clangd routing: the framework default is
                         the host database; avrdx/.clangd and samc21/.clangd
                         (in brio/ AND in each project dir) override with
                         their own architecture's database, so a header always
                         parses with its own compiler regardless of CMake
                         Tools' active project
brio/                    the framework, four strata:
  kernel/                pure kernel logic - includes NOTHING of brio
    platform.hpp           Platform concept (CriticalSection, idle,
                           break_here, now, ticks_per_second, atomic_width,
                           panic_record) + PanicRecord (hosted by the platform)
                           + the OPTIONAL idle_until(deadline) a platform on a
                           timebase that counts through sleep may offer
    active_object.hpp      ActiveObject concept: what Kernel requires of an
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
    kernel.hpp             Pack<Aos...> (index, lends_ok) + Kernel<P, Aos...>:
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
    crc.hpp                crc16_byte/crc16: the record checksum (bitwise,
                           no table)
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
    nvic.hpp               "sam.h" + armv6m/nvic.hpp (the guard and Nvic live there)
    platform.hpp           SamPlatform (idle takes whatever PM.SLEEPCFG holds -
                           SCR.SLEEPDEEP is never written - with erratum
                           1.8.13's guard around a standby WFI; BKPT, .noinit
                           breadcrumb, atomic_width 4)
    ticker.hpp             armv6m/ticker.hpp's BasicTicker (Ticker = 1000 Hz,
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
  armv6m/                the CORE stratum: what both Cortex-M0+ families share
    nvic.hpp               InterruptGuard (PRIMASK) + Nvic + irq_priority_levels
                           - reads CMSIS-Core only, #errors if included before
                           a device header (the family's nvic.hpp does both)
    ticker.hpp             BasicTicker<tps> over SysTick with advance/pause/
                           resume; each family's ticker.hpp adds its alias
                           and its own guards; SysTickCounter = SysTick as a
                           bare cycle counter (no interrupt) for delay_us
                           where the kernel timebase is elsewhere
    delay.hpp              delay_us / delay_rate / DelayRate: the microsecond
                           busy-wait on SysTick's VAL - at least, never early,
                           capped below one millisecond, no division at wait
                           time; each family's delay.hpp is the device include
                           plus this file plus its measured facts
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
    nvic.hpp               "stm32g0xx.h" + armv6m/nvic.hpp
    ticker.hpp             armv6m/ticker.hpp + the Ticker alias (1000 Hz)
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
                           USART, FLASH_ACTLR, the core's STK and PFIC), the
                           interrupt numbers = the vector table's word
                           indices; states the CH32V006K8 alone until a
                           second part gives the tiering something to say
    pfic.hpp               InterruptGuard (csrrci read-and-clear of
                           mstatus.MIE), enable/disable/readback, Pfic per-
                           line enables (write-one registers; the manual's
                           ISR bank is the ENABLE status, IPR the pending)
    ticker.hpp             BasicTicker over the core's STK (up-count, STRE
                           auto-reload, CNTIF cleared by the handler),
                           Ticker = 1000 Hz
    clock.hpp              Clock<internal|pll, hz>: HSI 24 MHz, the doubling
                           PLL (48 MHz), the HPRE divider table, flash wait
                           states first; pclk_hz = hz (no APB prescaler)
    pin.hpp                Pin<'D',5> / Port<'D'>: the ONE-BIT MODE of this
                           family (an F1 nibble is right by accident), pulls
                           through OUTDR, the port clock opened by every
                           configuring verb
    usart.hpp              Uart<1, P>: the interrupt-driven byte transport
                           (two rings, TXEIE armed/disarmed, errors read then
                           cleared, BRR = pclk/baud whole); USART1 on its
                           default pads PD5/PD6, USART2 refused until the
                           remaps exist
    platform.hpp           Ch32v00xPlatform<TB = Ticker>: idle() is a WFE,
                           not a WFI - this core's WFI wakes only for an
                           interrupt it can TAKE, so "sleep then unmask"
                           deadlocks; WFITOWFE + SEVONPEND latch the wake
                           instead; ebreak; .noinit breadcrumb; atomic_width 4
  host/                  the test target
    platform.hpp           HostPlatform (virtual clock, recording idle/break)
    sim_flash.hpp          SimFlash: FlashMedia over RAM for the host tests
                           (configurable geometry, power-cut injection,
                           simulated reflash, wear counters)
```

## Build artifacts

`build-cmake/<preset>/`: `<app>.elf` / `<app>.hex` / `firmware-<app>.map`
/ `<app>.lst` (source-interleaved disassembly), all written directly by
`avr_add_app()`'s post-build step - one set per app, in the preset's own
build dir (the SAM project's `sam_add_app()` does the same, plus a
`.bin`). Host test binaries live in `build-cmake/host/`, and the
per-project app rosters `apps_avrdx.json` / `apps_samc21.json` in
`build-cmake/` itself.
