# CLAUDE.md

Guidance for Claude Code when working in this repository. This file is
the assistant's operating manual: how to interact, where the truth
lives, what is settled, what is open. It does NOT duplicate the design
docs - it points at them.

## Interaction

Claude interacts with the user in Italian; every edit in the files -
code, comments, docs, commit messages - is in English.

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
  pair, survival-aware mount), `power.md` (the sleep-depth ladder, the
  site that only arms, the vote round, standing locks, the deadline
  guard, the first-event-after-wake contract), `meters.md` (the
  MeterLatch bridge out of a capture ISR and the MeterSampler that
  paces publication, not capture - a stale source publishes nothing).
- `docs/<target>/` - one folder per target, mirroring
  `brio/<target>/` (`avrdx/`, `samc/`, `host/`): `README.md` is the
  operational page (toolchain, board, probe, debugger and their
  quirks); next to it ONE document per peripheral in the shape
  docs/README.md prescribes (documents of record -> what the silicon
  does -> types and verbs -> how to use it, one example per use ->
  bench findings -> for provisional ones, "Not covered yet"). Only
  INCOMPLETE docs are marked: PROVISIONAL banner + closing "Not
  covered yet" (driver gaps kept distinct from implemented-but-not-
  bench-verified); a complete doc has NO banner and no gap list -
  never mark a doc complete while it still lists gaps. The state of
  the driver work is readable in the docs map. The Multislope assessment (every
  acrobatic piece maps to fixed routes + tasks on resources + config
  structs; the 64-cycle snapshot stays in the ISR body) lives in
  memory and in the track entry below, not in docs.
- `docs/bench.md` - the board, the wiring and the apps as they are
  today. The volatile end.
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
namespace `brio`; five strata under `brio/` - `kernel/` (pure
logic, includes nothing of brio), `util/` (services over the kernel),
`avrdx/` (everything that knows `avr/io.h`: AVR DA/DB, bench chip
AVR128DB48), `samc/` (everything that knows `sam.h`: SAM C21,
Cortex-M0+, bench chip ATSAMC21J18A), `host/` (the native test
target). Includes carry the stratum prefix
(`#include "avrdx/usart.hpp"`). The builds are three sibling CMake
projects, PEERS - the repo root is not a CMake project: `avrdx/` and
`samc/` (each with its own toolchain file and presets, Ninja,
emitting into the shared `build-cmake/`) auto-discover one `main()`
per `src/apps/<app>.cpp` at configure time from its own `// build:`
header comment; host tests in `test/` are the third project (host
g++, no cross toolchain), run via `ctest`. ONE NAME PER ARCHITECTURE,
the same key on three axes: `brio/<arch>/` (stratum),
`docs/<arch>/` (docs), `<arch>/` (build project); chip precision
lives in preset names, per-chip ld/svd files and the `*_MCU` cache
variables. Names are claims, extended only when a real chip extends
the family - the known landing names, never used early: avrdx ->
avrxt (Microchip's sigla for the modern-AVR core) when an EA/mega0
part proves it shares the stratum; samc -> sam0 if a D21 arrives; an
`armv6m/` core stratum factored at the SECOND ARM family.

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
  pack order, timers post events, panic breadcrumb, Platform concept;
- **util/ services and target drivers** - important, here to stay,
  but expected to change (possibly radically) when the second real
  target arrives;
- **apps** - incidental test tools; they will not survive in their
  current form. Nothing in the foundations or their docs may depend on
  an app; apps document themselves in their own header comment.

## Working discipline (read this first, every session)

The documented failure mode of past sessions is EFFORT PARSIMONY:
solving the one concrete problem on the bench chip instead of building
the framework. It produced drivers that did not compile on half the
family, docs marked complete that listed their own gaps, and false
comments justifying wrong restrictions. The antidote, in practice:

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
  `test_<target>_<subject>` suite on the bench. The bench chip alone
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
  names (test suites excepted); only INCOMPLETE docs are marked
  (PROVISIONAL + "Not covered yet", driver gaps separate from
  implemented-but-not-bench-verified); doc and code move in the same
  change. Never mark a doc complete while it lists gaps.
- **When the user refines the method, write it to memory in the same
  session** - the next context must start from the agreed method, not
  regress to the instinctive minimum.

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
(samc/device_tables.hpp), never in a driver; a driver keeps `#ifdef`
only to select per-instance CODE (ruling 2026-08-28, full text in
overview.md "Generalization rule"); no target includes outside
the target strata; the kernel must never know which silicon it runs
on. Apps never touch registers (PORTx/VPORTx/peripheral structs live
only in target drivers; the ISR vector binding is the one vendor glue an
app may contain). Full text: `docs/design/overview.md`.

## Open items and horizon

Roughly ordered by proximity. None of these is a decision yet; each
gets its dated home in `docs/design/` when taken.

- **Borrowed, phase 2 (debug epoch).** `Borrowed<T, Lease::dispatch>`
  is a plain pointer today. Planned: in debug builds an 8-bit lender
  epoch travels with the loan (on 8-bit targets it costs one byte and
  no padding; on 32-bit ones it sits in the pointer's padding for
  free) and is compared on every access - a stale loan panics on the
  guilty instruction. To be built together with the first host test
  that simulates preemption; not before.
- **Payload/ownership pass DONE.** Every reply-class pointer payload
  now names its lease in the field type: `NvWrite::data`,
  `TwiHost::Request::tx/rx` and `SpiHost::Request::cmd/tx/rx` are
  `Borrowed<..., Lease::reply>`, built at the call sites with the new
  `lend<Lease::reply>(buf)` maker in kernel/borrowed.hpp (spelled like
  `reply_to<>`; `{}` is the null loan, and a loan converts to the same
  loan over a more qualified pointee so a writable buffer lends to a
  read-only field with no ceremony). Borrowed is a zero-cost typed
  pointer: the release images of test_avr_twi / test_avr_spi /
  test_avr_nvm / bus_mv are BYTE-IDENTICAL before and after, which is
  how the retyping was verified.
- **HSM.** The FSM contract is HSM-ready (`unhandled` = future
  bubble-to-parent); parent pointers, bubbling and LCA entry/exit
  chains get built only when a real AO demands them.
- **Second target: SAM C21 - bring-up step 1 DONE (2026-08-27).**
  The samc/ stratum (nvic, clock, pin, ticker over SysTick at
  1000 Hz, platform_sam) and the samc/ build project (own toolchain
  file, presets, hand-written crt: 47-entry vector table +
  ld script with NOLOAD .noinit; OpenOCD upload over SWD,
  cortex-debug) exist and are live-verified on the user's C21J rev
  1.1 board (ATSAMC21J18A, silicon rev F): kernel blink under time
  events, and a full SERCOM5 console at 115200 driven end-to-end
  over the CH340 - byte-exact baud (BAUD 63019, actual_baud readback
  115219 = the arithmetic to the hertz), tick coherent with the wall
  clock. THE PROMISE HELD: kernel/ and util/ compiled UNTOUCHED on
  the second architecture (ring's lock-free path, print's hosted
  branch, serial_port, line_parser, the clock contracts - first real
  exercise for each). Docs: docs/samc/*.md (all PROVISIONAL with
  honest gap lists), vendor pass in docs/samc/vendor/README.md.
  Findings that cost blood and are now pinned by asserts: SERCOM has
  ONE interrupt vector (isr() masks INTFLAG with INTENSET - DRE is a
  level), CTRLA.DORD resets MSB-first while a UART is LSB-first (the
  first banner arrived bit-reversed; family static_assert guards the
  default), TxD exists only on PAD0/PAD2, PORT_GROUPS = 2 on every
  variant, the ticker's getters need volatile reads (gcc -Os DELETED
  a bare polling loop), -Og keeps abort() alive so the crt must
  define it. Deliberately still open (each doc's "Not covered yet"):
  the E/G variants are compile-only, bench.py knows no SAM, sleep/
  reset/EIC are future passes. STM32G0x0/x1 remains the candidate
  third target.
  **DMAC campaign DONE (2026-08-27, Opus delegation, hand-verified
  after the session hosting the agent crashed between code and docs -
  the docs were reconstructed from the agent transcript plus a fresh
  bench pass).** samc/dmac.hpp NEW - Dmac block (CHID select-then-use
  guarded structurally: with_channel private, DmaChannel the only
  friend; INTPEND take_pending() as the selector-free ISR dispatch;
  the two 384-byte SRAM descriptor tables), DmaDescriptor/
  dma_descriptor() (start + beats in, END-ADDRESS arithmetic out -
  constexpr, fixture-pinned, and the datasheet disagrees with itself:
  25.6.2.7 is right, the register descriptions' "+ 1" is not),
  DmaChannel<n> (bounded disable wait because SWRST is IGNORED
  silently while ENABLE drains; harvest() = suspend/read/resume in
  ONE critical section, every write-back reading VALIDATED against
  the loaded copy and refused on mismatch - the erratum 1.10.4
  answer, since its workaround forbids the concurrency a duplex port
  IS), DmaTxEngine/DmaRxEngine (peripheral-agnostic: data address +
  trigger code handed in at arm). Uart gained two OPTIONAL engine
  template slots (NoDmaEngine default, if constexpr throughout,
  sercom.hpp never includes dmac.hpp): TX drains the ring's
  read_span in blocks from the DMAC completion, RX fills write_span
  and is published by the harvest() VERB - pacing is the port
  owner's, per-byte error attribution traded away and said so.
  util/ring.hpp gained the span/commit bulk API (read_span/consume,
  write_span/publish; spans never wrap, clamped commits, SPSC
  contract unchanged - design/ring.md). Zero-cost PROVEN: SAM
  blink/console/probe and all 40 AVR hexes byte-identical. NEW SUITE
  test_samc_dma z 112/112 (agent x5 + hand x2; runner-driven r 7/7,
  i 9/9), first testbench.hpp user on SAM. Bench facts in dmac.md:
  1.10.4 CAUGHT (340 corrupted write-backs refused in 210852
  readings under engined churn, a MIXTURE of two channels' fields;
  zero at low trigger density; transfers byte-exact throughout), a
  bus-errored channel deterministically loses the FIRST beat of its
  next block (documented nowhere), the device header's LVLEN
  group-vs-per-level macro trap (levels 1-3 silently never enabled),
  take_pending() could steal SUSP from a concurrent harvest (~1 per
  70k - hence the one critical section), DMA buffers need volatile
  BOTH directions (gcc sank a zeroing store past the transfer),
  m2m 12 MB/s, chains 59700 blocks/s, harvest ~10 us. Errata: 1.10.4
  live on rev F; 1.10.1..3 NOT this chip (the matrix's N-family row
  is the trap). Deliberately not built (dmac.md): CRC engine, linked
  lists, event hooks (no EVSYS driver), RUNSTDBY/standby, non-SERCOM
  trigger codes.
  **AC sync-latency probe DONE (2026-08-27, user's curiosity,
  answered same day).** samc/ac.hpp NEW (minimal by design: Ac block
  + AcComparator<n>, config enable-protected per comparator, the
  probe's surface only - window/events/sleep wait for the full
  campaign) + probe app ac_sync_probe (7 letters, 30/30 twice,
  wireless: GPIO-driven PA04 vs the comparator's own VDD scaler,
  CMP0 pad read back, GCLK_AC at 11.7 kHz, SysTick stopwatch). THE
  ANSWER ch. 40 never gives: a synchronized output edge costs the
  fraction to the next GCLK_AC edge + TWO whole periods (the user's
  bench intuition confirmed; staircase + 1000 randomized shots +
  independent OSCULP32K all agree). Also measured: STATE is the
  sampled path even with OUT=ASYNC; INTFLAG fires on the SAME
  period as the flip; a mid-stream edge through the majority filter
  pays (N-1)/2 periods (the chapter's N-1 is the from-idle cost);
  single-shot START->READY 5.1..6.5 periods. (A GCLK claim this probe
  also made - that DIVSEL divides by a FIXED 2^(field_width+1) - was
  CORRECTED on 2026-08-28 by test_samc_clock letter f: it is
  2^(DIV+1), the DIV value counts, and the probe's own OSC48M/4096 is
  what that gives for DIV = 11. The probe's measurements are
  unaffected; the explanation was wrong.) Docs: docs/samc/ac.md
  (PROVISIONAL).
  **Peripheral + util campaign PLANNED AND PART-EXECUTED 2026-08-28,
  REVIEWED AND COMMITTED the same day**: Fable re-ran every gate and
  every suite by hand (incl. platform letter i across its six resets)
  and ACCEPTED all seven judgment calls the session had queued (index:
  memory samc-session-2026-08-28). The plan itself is in memory
  samc-peripheral-plan. The chapters split in two and the sequence alternates
  them: chapters that give a util contract its SECOND implementation
  and thereby validate it (NVMCTRL -> FlashMedia/NvHeap, SERCOM SPI+
  I2C -> spi_bus/i2c_bus/BusMaster, TC/TCC -> PwmChannel +
  MeterSource, ADC/SDADC -> analog/AnalogSampler, PM -> SleepSite),
  and chapters where the AVR shape is simply wrong and brio must grow
  (EVSYS is a 12-CHANNEL ALLOCATOR with numeric generator/user ids,
  three propagation paths and a per-channel GCLK, not the AVR's
  per-generator types; per-peripheral GCLK channels make
  DynamicClock's "one rate for everything" an AVR assumption; the
  NVIC's four priority levels against a flat kernel; PAC has no brio
  concept at all; CAN has no util vocabulary). Phases: A bench
  tooling for SAM (bench.py is avrdude-only, so test_samc_dma is not
  re-runnable and no SAM suite can be judged - and the app roster
  must become per-project because names COLLIDE across avrdx/ and
  samc/: blink, console, probe exist in both); B nvm.hpp + nvm_flash
  (also taking back the flash wait-state verb clock.hpp squats on)
  and reset.hpp + a real HardFault breadcrumb; C EVSYS + EIC (SAM's
  pin interrupts live in the EIC, PORT has none) and then AC's event/
  window gaps; D TC then TCC; E OSC32KCTRL/XOSC/FDPLL96M/SUPC/RTC,
  then the DynamicClock question (RULED 2026-08-28: DEFERRED - no
  DynamicClock on this target for now, Clock<> stays static; per-
  peripheral GCLK channels make "one rate for everything" an AVR
  assumption, and the question reopens with its first real consumer)
  and PM's SleepSite (erratum 1.8.13 is that phase's landmine); F SERCOM SPI and I2C, whose two-board
  halves should use an AVR128DB as the peer - spi_peer and twi_peer
  already exist, so the bus vocabulary gets a CROSS-ARCHITECTURE
  proof for free; G ADC/DAC/SDADC/TSENS; H CCL, FREQM, DSU, PAC,
  DIVAS, MTB and CAN last (new vocabulary plus a second C21 node).
  **Phase A DONE 2026-08-28: the bench tool speaks both
  architectures.** bench.py's BOARD_TYPES maps a board type to its
  project, preset, mcu and flash mechanism (db* -> avrdx/avrdude/UPDI,
  c21j -> samc/OpenOCD/SWD), so flash/run/console are
  architecture-blind at the command line while `fuses` and `--erase`
  refuse a SAM board and say why. The app roster became PER PROJECT
  (apps_avrdx.json + apps_samc.json) because names collide across the
  trees. The SAM board is desk position C in the manifest, with its
  factory 128-bit die serial recorded as the identity an AVR board has
  to be GIVEN by hand (no USERROW provisioning on this family).
  Regression through the new path: test_samc_dma z 112/112. NB the
  environment's python had NO pyserial - installed into the venv that
  `python3` already resolves to, so the documented invocation works.
  **Phase B1 DONE 2026-08-28: NVMCTRL, and the heap's second
  silicon.** samc/nvm.hpp NEW (the whole of ch. 27: both arrays behind
  one page buffer and one command register, the CMDEX key discipline
  with sticky-error reporting, erase-by-row/program-by-page, region
  locks, PARAM geometry cross-checked against the device header, and
  the read-only factory views - NvmUserRow IS this family's fuse row,
  NvmCalibration/NvmTemperatureCalibration are what future ADC/OSC32K/
  OSC48M/TSENS drivers must copy into their peripherals, DeviceSerial
  is board identity); FlashWaitStates MOVED here out of clock.hpp,
  closing that file's own declared squat. samc/nvm_flash.hpp NEW:
  RwweeFlash, and the design choice IS the RWWEE array - writing it
  does not stall the CPU (measured: ~3950 polling turns survive a row
  erase there against ONE on the main array), it is 4x more durable
  (100k vs 25k cycles), and its zone is a CONSTANT because no linker
  section can reach it, which deletes the AVR backend's hardest
  problem. util/nv_heap.hpp compiled and ran UNCHANGED on it: the
  erase_size/write_cell split (256/64 here vs 512/2 on AVR) is exactly
  what made that work, and design/nv-heap.md's "second target" gap is
  closed. NEW SUITE test_samc_nvm z 52/52 plus letter m 8/8 outside z
  (it costs one main-array row of endurance). Findings in nvm.md: ADDR
  is section-relative half-words (measured over SWD before a line was
  written, then again from firmware); THE PAGE-BUFFER RULE IS NARROWER
  THAN THE CHAPTER'S - not "write ascending" but "write both words of
  a 64-bit section back to back", proven by descending loads coming
  out exact while even-then-odd loses all eight low words; RWWEE row
  erase 989 us / page write 190 us against the 6 ms / 2.5 ms maxima;
  table 45-42's footnote limits a row to 8 consecutive writes before
  an erase is mandatory (nowhere in ch. 27); and a stalled operation
  CANNOT be timed by the software clock - where the tempting
  explanation is wrong, since eight erases report eight milliseconds
  of ticks, so no tick is lost and the counter is merely STALE for one
  reading after a stall. NVMCTRL has NO errata (1.14.1 is Reserved,
  its one historical item deprecated as resolved). Deliberately not
  built: writing the user row (it is the fuses, survives chip erase,
  carries WDT ALWAYSON - provisioning, wanting a bench.py verb over
  SWD), the SSB security bit (one-way), the header's two undocumented
  commands SF/WL, a main-array FlashMedia backend.
  **Phase B2 DONE 2026-08-28: RSTC + WDT + the fault breadcrumb,
  closing platform.md's "failing half".** samc/reset.hpp NEW: Reset
  (RCAUSE decoded as the EXCLUSIVE one-cause register it is - the AVR's
  accumulating RSTFR habit does NOT travel here - plus table 18-1's two
  groups and software()), Watchdog (the whole of ch. 23: the shared
  8<<n period encoding for PER/WINDOW/EWOFFSET, enable-protection vs
  synchronization, the early-warning interrupt, clear() and
  force_reset() spelled apart), ResetReporter (a panic Reporter that
  resets so the breadcrumb is read at the next boot) and
  hard_fault_reset<P>() (the HardFault BODY an app binds - it does NOT
  go through panic(), because a BKPT taken from inside HardFault is a
  LOCKUP, and it does NOT overwrite a valid record, because a fault
  after a panic is a consequence of something already diagnosed).
  util/testbench.hpp gained resume_tally() - the other half of the
  already-public end_letter(), which its own comment promised
  reset-spanning tests and never gave them a way back. NEW SUITE
  test_samc_platform z 34/34 plus letter i 20/20 outside z (it reboots
  the board six times). Findings in reset.md: the fuse row and the WDT
  registers agree field by field (nvm.hpp and reset.hpp describing the
  same fuses from opposite ends); OSCULP32K measures 1030.4 Hz BY
  DIFFERENCE, the two-point method being what removes the constant 3 ms
  arming cost a single measurement hides; A WRONG CLEAR KEY RESETS
  WHETHER THE WATCHDOG RUNS OR NOT (23.6.2.4's sentence sits in the
  Normal-mode section and reads narrower than the silicon) and is NOT
  immediate - CLEAR is write-synchronized, and the first version of the
  test ran past its own trigger into the next leg; the breadcrumb
  survives a system reset, code and context intact; SYST and WDT are
  distinguishable so two intentions cross a reset with no RAM at all.
  THREE THINGS BIT AND ARE NOW FIXED: the crt's HardFault_Handler was
  NOT weak although its own comment said so (an app binding it failed
  to link); an unaligned volatile load does NOT fault, because gcc
  emits four byte loads with shifts - only UDF is beyond the
  compiler's reach; and THE BKPT HAZARD - a core with DHCSR.C_DEBUGEN
  set HALTS on a BKPT instead of faulting, so break_here() stops the
  board silently, and table 18-1 makes it STICKY (the debug logic is
  reset only by a power-on or external reset, never by a watchdog or
  system reset), so bench.py now clears C_DEBUGEN as the last step of
  every SAM flash.
  **Serial-speed probe DONE 2026-08-28 (the user's curiosity, answered
  the same day; full story in memory samc-serial-speed).** New probe app
  serial_speed. THE WIRE (ADuM1201 isolator + CH340) IS GOOD TO 3 MBAUD
  - the SERCOM's own 16x ceiling at 48 MHz - proven by a raw polled
  transmit moving 64 KB byte-exact at 299251 B/s, 99.75% of nominal and
  6.5x the AVR bench's 460800. 2.5 Mbaud is a HOLE and not a ceiling
  (both neighbours work; the bridge has no exact divisor for it and the
  nearest is ~4% off). THE LIMIT WAS OUR OWN API: write()/write_byte()
  nudges the transport PER BYTE - arm DRE, or pump_tx() - and plateaus
  at 98.4 kB/s at every rate from 1 Mbaud up; fed that way the DMA
  engines are SLOWER than the interrupt (57-64 kB/s at 92% CPU, a block
  started for one byte). SO sercom.hpp GAINED write_bulk()/read_bulk(),
  which fill or drain the ring's own contiguous run (util/ring.hpp's
  span/commit API, added by the DMAC campaign and never exposed) and
  nudge ONCE: at 3 Mbaud the engines then deliver 297890 B/s - 99.3% of
  the wire - at 9% CPU, against 169343 B/s at 75% through the interrupt;
  at 1 Mbaud they saturate the wire at 5% CPU against 70%. Echo is
  lossless to 1 Mbaud through the interrupt transport both ways, and
  above it loses in the SOFTWARE ring (hw_overruns stays 0). Console
  policy this suggests: 1 Mbaud as the safe default, 3 Mbaud for
  transmit-heavy work with DMA + bulk. OPEN AND LOOKS LIKE A REAL BUG:
  the RX engine in FULL DUPLEX loses most of the stream and can wedge
  the transport (print blocked on a TX ring that stops draining, the
  halted core in Ring::push) - while TX-only through the engine is
  flawless and test_samc_dma's duplex letter passes 112/112, so one of
  the two shapes misses it. Not isolated; sercom.md records it.
  **FREQM DONE 2026-08-28 (ch. 44, a one-hour peripheral).**
  samc/freqm.hpp NEW: the hardware ratio counter, f_msr = VALUE/REFNUM x
  f_ref between two GCLK generators, with the 24-bit overflow budget
  (refnum_for) and erratum 1.24.1 AS CODE - reading CTRLB is a PAC
  protection error on EVERY silicon revision with no workaround, so
  START is written and never read and BUSY/DONE are the only evidence a
  measurement began. NEW SUITE test_samc_freqm 25/25, wireless by
  construction, and the first thing in this stratum to run a GCLK
  generator other than 0 on silicon (clock.md updated). THREE FINDINGS:
  (1) the CROSS-CHECK - OSCULP32K measured here against OSC48M reads
  32957 Hz while test_samc_platform letter c, through the watchdog and
  SysTick, implies 32960, 3 Hz apart (CORRECTED 2026-08-28 by the clock
  campaign: the two routes SHARED the OSC48M scale after all - SysTick
  rides CLK_MAIN - so the agreement proves the chains' consistency, not
  the frequency; on the crystal's scale both readings become ~32907 Hz),
  and OSCULP32K runs fast of nominal (every watchdog timeout inherits
  that); (2) CFGA HAS NO DIVREF whatever 44.8.3 draws - the bit
  does not even stay written (CFGA reads back 0x0001) and changes no
  measurement, so the device header's 8-bit CFGA_Msk is right and the
  chapter's 16-bit drawing is not; the driver REFUSES a config asking
  for it; (3) A REAL ORDERING BUG, caught on the bench: resetting the
  block before its GCLK channels are connected leaves SYNCBUSY.SWRST
  standing forever (SWRST synchronizes into a domain those channels
  feed) - every measurement returned nothing until init() was reordered
  to route first. Also measured: REFNUM scales the count to 6 parts in
  10000 at REFNUM 64/128, while a 4-cycle window misses by 0..34 parts
  in 10000 between runs - the reference RC's short-term wander, so a
  ratio test is immune to the reference's absolute error but NOT to its
  drift between the two measurements.
  **OSC32KCTRL DONE 2026-08-28 (ch. 21, measured with the meter built
  an hour earlier).** samc/osc32kctrl.hpp NEW: the three 32 kHz roots
  (Osculp32k always-on and only trimmable, Osc32k off-at-reset and
  needing its production trim, Xosc32k with the clock-failure detector),
  the RTC's clock select - which lives in THIS chapter and not in the
  RTC - and the shared IRQ 0 caveat. NEW SUITE test_samc_osc32k 32/32,
  wireless, using samc/freqm.hpp as its instrument: LETTER B IS WHERE
  THREE DRIVERS MEET - nvm.hpp reads the production trim out of the NVM
  calibration area, osc32kctrl.hpp writes it into the oscillator and
  freqm.hpp says what it was worth. THE ANSWER: 47312 Hz untrimmed
  against 32995 trimmed, i.e. an OSC32K enabled without reading 21.5.9
  runs 44% FAST. Also measured: OSCULP32K's trim is a ~900 Hz-per-step
  knob (and the WATCHDOG rides on that oscillator, so trimming it moves
  every timeout); both internal RCs land six per mille high once
  trimmed, OSC32K marginally closer as 21.6.5 implies; a missing crystal
  is a false return and not a hang. ONE MORE ORDERING RULE LEARNED THE
  HARD WAY, and it belongs to GCLK rather than to this chapter: A
  GENERATOR CANNOT BE MOVED OFF A STOPPED SOURCE (16.6.2.6 releases the
  old source only once the new one is ready), so stopping an oscillator
  while a generator still points at it leaves that generator unroutable
  forever - point it somewhere running FIRST. Errata: NEITHER item
  touching ch. 21 applies here (1.1.1 is rev B only, 1.22.1 is the
  N-family row) - the read-the-row trap again.
  **EVSYS DONE 2026-08-28 (ch. 29) - PHASE C's FIRST HALF, and the
  adaptation the plan was built around.** samc/evsys.hpp NEW, and THE
  DESIGN POSITION IS THE POINT: on the AVR the event system is a typed
  table (per-generator types, compile-time legality); here it is an
  ALLOCATOR - twelve identical channels, numeric codes from tables of 95
  generators and 47 users, a GCLK per channel. Reproducing the AVR shape
  would mean 95 types encoding a table this header has no business
  owning, so THIS DRIVER OWNS THE FABRIC AND NOT THE VOCABULARY: a
  peripheral that generates events publishes its generator codes, one
  that consumes them publishes its user index. That is what keeps the
  file short and is the thing for Fable to accept or overrule. Built:
  the three paths with the path/edge legality enforced BOTH ways, the
  user multiplexer's channel+1 hidden in two lines, connect() taking
  user AND channel together so 29.6.2.3's ordering cannot be got wrong,
  the status surface, and THREE LIVE ERRATA as code (1.12.1 refuses a
  synchronous channel with a free-running clock - spurious overruns;
  1.12.3 and 1.12.4 are waits the caller must spend and the header says
  so, since it cannot know the channel clock's rate; 1.12.2 is not this
  silicon). NEW SUITE test_samc_evsys 37/37, wireless, WITH THE DMAC AS
  ITS EVENT USER: a DMA channel armed with NO hardware trigger, so the
  only thing that can move its bytes is an event - the transfer is the
  witness, which also retires dmac.md's caveat that every EVACT value
  but none was untested silicon. THE FINDING THE CHAPTER DOES NOT HAVE:
  A SOFTWARE EVENT DOES NOT CROSS AN ASYNCHRONOUS CHANNEL. 29.6.2.12
  says a software event "can be serviced as any event generator" with no
  mention of the path; measured, EIGHT of them on an async channel move
  nothing while ONE on a clocked path moves a block - the async path has
  no clock and no edge detector, and a register write has no width to
  propagate. Deliberately NOT claimed: anything about a hardware
  generator on the async path, which this suite has none to test.
  SEVEN JUDGMENT CALLS from this session were REVIEWED AND ACCEPTED
  at commit (none was forced by the code; full note in memory
  samc-peripheral-plan): EVSYS owning the fabric and not the
  vocabulary; write_bulk/read_bulk added to sercom.hpp on measurement
  rather than on request; testbench.hpp's resume_tally() as
  end_letter()'s reset-spanning other half; and the four below: the
  NvHeap living in the RWWEE array; the SAM board took
  desk position "C" (letters stay positions, so a second C21 is "D");
  the AVR roster was RENAMED apps_manifest.json -> apps_avrdx.json for
  symmetry with apps_samc.json; and nvm.hpp's refusal to expose user-row
  writes means BOOTPROT and the EEPROM-emulation size are readable only.
  **EIC DONE 2026-08-28 (ch. 26) - PHASE C CLOSED.** samc/eic.hpp NEW:
  the peripheral where this family keeps its PIN INTERRUPTS (PORT has
  none), sixteen lines plus the unmaskable NMI, the five senses, the
  majority filter, asynchronous detection, EVCTRL, one NVIC vector for
  all sixteen (26.6.6 describes per-line request lines; the device
  header gives EIC_IRQn, and the header wins - the SERCOM shape again),
  every configuration register enable-protected so every verb touching
  one RETURNS FALSE while enabled rather than storing into a register
  the silicon ignores. THE PAD-TO-LINE MAP IS THE DEVICE HEADER'S OWN
  TABLE and nothing else: PIN_P<pad>A_EIC_EXTINT_NUM, one guarded probe
  per pad (51 on the J, 37 on the G, 25 on the E; since the 2026-08-28
  sanitation the probes live in samc/device_tables.hpp, the one file
  where vendor-macro #ifdef walls are allowed), because the map is
  IRREGULAR (PA16 -> 0, PA24 -> 12, PA27 -> 15, PB30 -> 14) and no
  formula can stand in for it; ExtInt<Pin> refuses to compile on a pad
  the package does not bond, which is the per-package gate with no
  hand-kept table behind it, and ExtNmi<Pin> is the same for PA08.
  ERRATA, AND THIS IS THE CHAPTER WHERE READING THE ROW MATTERS MOST:
  FIVE OF SIX ARE NOT THIS SILICON (1.11.1/1.11.2 rev B only,
  1.11.3/1.11.4 revs B..E, and 1.11.5 - the one everybody would apply -
  is the N FAMILY ONLY); the single live item is 1.11.6, asynchronous
  edge detection in Standby catching only the FIRST edge on every E/G/J
  revision, which the driver cannot enforce (it does not know if the app
  sleeps) and therefore carries as a stated obligation on
  EicLineConfig::asynchronous. NEW SUITE test_samc_eic z 85/85 twice,
  plus n 9/9 (the NMI) and u 4/4 (the button) outside z - WIRELESS on a
  chapter whose subject is EXTERNAL pins, and letter b is where that
  technique is ESTABLISHED rather than assumed: PMUXEN takes a pad away
  from PORT's OUTPUT DRIVER (driving OUT under the mux moves nothing)
  but NOT from its INTERNAL PULL, whose direction is still the OUT bit
  (28.6.3.2) - so a pad first proven electrically free walks between the
  rails on its own and every sense, filter and event path sees a real
  edge on a real pin. FOUR FINDINGS THE CHAPTER DOES NOT HAVE: (1) THE
  ENABLE NEEDS THE CLOCK ITS LINES ASKED FOR - 26.6.3 says the EIC
  requests GCLK_EIC/CLK_ULP32K in the sampled modes and CTRLA.ENABLE
  synchronizes against THAT clock, so with GCLK_EIC disconnected a block
  of clockless lines enables/detects/disables perfectly while ONE line
  asking to be sampled leaves CTRLA.ENABLE written, readable and
  SYNCBUSY.ENABLE standing forever, with nothing detected; the write is
  PENDING not lost (connecting the channel afterwards, CTRLA untouched,
  completes it and the line starts detecting), and enable() returning
  false is the only warning there is; (2) A HARDWARE GENERATOR DOES
  CROSS AN ASYNCHRONOUS EVSYS CHANNEL - an EXTINT edge on an async
  channel moves a whole DMA block where eight software events move
  nothing, which ANSWERS the question evsys.md explicitly declined
  (the async path carries what has WIDTH; a register write has none);
  (3) EVERY LINE IS AN EVENT GENERATOR, not the "EXTINT0-7" of 26.6.7's
  prose - EXTINT9 (code 0x17) moves a block, and clearing EVCTRL.EXTINTEO
  is the gate, not the line number; (4) A TRAP IN OUR OWN CRT, found by
  the first NMI: the device header declares NonMaskableInt_Handler and
  SVCall_Handler while startup_samc21.cpp spelled them the CMSIS way
  (NMI_Handler, SVC_Handler) although all 31 PERIPHERAL names already
  matched the header - an app binding the header's name compiled, linked
  and left the vector at Default_Handler, a silent spin; the crt now
  spells both the header's way (a JUDGMENT CALL - shared build glue
  changed on a bench finding - ACCEPTED at Fable's review, with the
  header verified as the declaring authority and the tree grepped clean
  of the old names). Also measured: the filter is a real low-pass (a
  few-hundred-cycle excursion is rejected at a ~32 kHz EIC clock, a
  settled one passes), a cleared LEVEL flag comes straight back while an
  edge flag does not, an unarmed line flags without interrupting, the
  NMI fires with the EIC DISABLED (26.6.4.1) and sense NONE is the only
  way to turn one off, and PB22 on this board does NOT follow its own
  internal pull (it rests LOW with the pull-up on). Doc:
  docs/samc/eic.md (PROVISIONAL: the N-variant debouncer, sleep/wake).
  Regressions after the crt edit: test_samc_dma z 112/112,
  test_samc_platform z 34/34 and i 20/20, check_samc OK, host 22/22.
  **AC COMPLETED 2026-08-28 (ch. 40) - PHASE C CLOSED FOR GOOD.**
  samc/ac.hpp grew from the probe's minimal surface to the whole
  chapter bar sleep: AcWindow<w> (the comparator PAIRS - WINCTRL is
  write-synchronized but NOT enable-protected, so a window turns on
  under a running block; WSTATE above/inside/below, the four WINTSEL
  conditions, and pair_consistent() asking the SILICON the two things
  40.6.4 requires and no register enforces - same measurement mode,
  same positive input); the whole EVENT SURFACE both ways with the
  codes PUBLISHED HERE per the accepted EVSYS ruling (COMP0..3 gens
  0x49..0x4C, WIN0/1 gens 0x4D/0x4E, SOC0..3 users 34..37 which table
  29-3 marks ASYNCHRONOUS PATH ONLY), EVCTRL refused while the block
  is enabled; and PER-PACKAGE INPUT LEGALITY, where the finding is
  that THE PAIR OWNS THE PADS - the same pin2 code means AIN2 on COMP0
  and AIN6 on COMP2, and AIN6/AIN7 (PB05/PB06) are J-only, so
  ac_config_valid() takes the COMPARATOR INDEX as an argument and the
  device header's own PIN_P<pad>B_AC_AIN<k> symbols are the authority.
  Two more chapter rules became refusals: hysteresis is continuous-mode
  only (40.6.6) and the end-of-comparison interrupt is single-shot only
  (40.8.12). ERRATA on the E/G/J row at rev F: only TWO of seven apply
  and neither is fixable in a register - 1.5.3 (AC and PTC share pads)
  and 1.5.6 (a spurious COMP flag when enabling with MUXNEG = bandgap,
  the caller's to clear; the driver does not refuse a real input) -
  while 1.5.2 makes low-power WITH hysteresis LEGAL here, and the
  device-level 1.8.2 (GCLK_AC not functional, borrow GCLK_ADC1) is
  REVISION B ONLY. NEW SUITE test_samc_ac z 94/94 THREE TIMES,
  wireless; ac_sync_probe stays a PROBE and is untouched. THE TEST
  DESIGN PROBLEM AND ITS ANSWER: a window needs a signal that sits
  BETWEEN two limits and this board has no analog source (no DAC
  driver, and the bandgap needs SUPC.VREF which nothing here turns
  on), so the ROLES ARE SWAPPED - the SIGNAL is each comparator's VDD
  scaler (both at step 31, so the pair sees one level) and the two
  LIMITS are the two rail-driven pads, which reaches all three WSTATE
  values; the chapter's own shared-input-PIN shape is exercised too and
  reaches the two states a rail can. Measured: STATE and the CMP0 pad
  follow the driven pad both ways and the 64-step ladder is a real
  divider; all four WINTSEL conditions fire on their own transition and
  stay silent otherwise; a COMPARATOR FLIP and a WINDOW TRANSITION each
  move a DMA block, the window's event generated from the
  inside/outside state whatever WINTSEL says (40.6.13 confirmed); and
  letter f closes the loop with eic.hpp - A PIN EDGE STARTS A
  SINGLE-SHOT COMPARISON through SOC0 on the asynchronous path, with
  EVCTRL.COMPEI as the proven gate. TWO INCIDENTAL FINDINGS: PA04 and
  PA05 do NOT follow their own weak internal pull on this board though
  they reach both rails under PORT (so the suite's precondition is
  "does the pad go where PORT drives it"); and RE-POINTING AN EVENT
  CHANNEL AT A NEW GENERATOR LEAVES AN EVENT STANDING - reproducibly,
  the first DMA arming after Evsys::connect() moves a block the window
  never asked for, which reads as the re-route looking like an edge
  while the not-yet-ready user makes the channel HOLD it (29.2's
  USRRDY handshake); the suite arms twice, rests its verdict on the
  second and prints the first. Doc: docs/samc/ac.md (still PROVISIONAL:
  sleep, the DAC/bandgap inputs, COMP2/3 and window 1 on silicon).
  **TC DONE 2026-08-28 (ch. 35) - PHASE D OPENS, and it is the campaign's
  "assecondare" case: util's contracts get their SECOND implementation
  and are validated by it.** samc/tc.hpp NEW: five instances, the three
  counter resolutions, four waveform modes, capture with all its event
  actions, commands, buffered registers, status, flags, ISR body - plus
  TcWo<Pin>, TcPwm/TcPwm8 (both PwmChannel) and TcPeriodMeter/
  TcPulseWidthMeter (the shapes MeterSource is fed from). THE GEOMETRY IS
  THE DEVICE HEADER'S, ALL OF IT, and two facts of it matter: TC0/TC1
  SHARE generic clock channel 30 and TC2/TC3 share 31 (35.5.3: a shared
  pair "cannot be set to different clock frequencies"), and
  TCn_MASTER_SLAVE_MODE says which instances PAIR into a 32-bit counter -
  TC0+TC1, TC2+TC3, never TC4 - so a 32-bit mode on a client is a
  compile-time refusal read out of the header rather than out of
  35.6.2.4's sentence. The pad-to-(TC,WO) map is the same story as the
  EIC's: 26 pads on the J, 18 on the G, 8 on the E, PA22/PB08/PB12 all
  TC0/WO0, PB23 the board's LED = TC3/WO1, one guarded probe per pad
  (in samc/device_tables.hpp since the sanitation, with the TC instance
  data - gclk/pairing/DMAC ids - beside it). READING COUNT IS A COMMAND (READSYNC then two waits, 35.6.8) and
  the raw accessors are spelled raw. ERRATA: 1.20.1 and 1.20.2 are
  REVISION B ONLY (1.20.2 - "input capture on I/O pins does not work" -
  is the trap); 1.20.3 is EVERY revision and is code, clear_buffer_valid()
  writing the flag TWICE as the erratum's own workaround demands. NEW
  SUITE test_samc_tc z 77/77 THREE TIMES, wireless. LETTER F IS THE POINT:
  a MeterSampler AO inside a REAL KERNEL, fed by a MeterLatch that TC2's
  capture ISR fills from a pin edge routed EIC -> EVSYS -> TC - 41 ISR
  captures, 19 samples published, 19 received, last value 2344 ticks
  against 2343 exact, 21 overwrites counted - which is meter_sampler.hpp's
  whole design (the AO paces PUBLICATION, not capture) measured on a
  second architecture with NOT ONE LINE OF util/ CHANGED. Also measured:
  the prescaler ratio exactly 4.00x (and /1024 within 0.4% of exact over
  a SysTick-timed window); COUNT32 advancing 9587352 counts in 200 ms,
  146 wraps past a 16-bit range, with TC1 reporting STATUS.SLAVE; one-shot
  stopping itself; PWM on the LED with duty 0/199/100/50 reading
  0/994/488/255 per mille off the pad AND ITS FREQUENCY READ BY A SECOND
  TIMER COUNTING ITS OVERFLOW EVENTS (937 in one second against 937.5 Hz -
  a timer counting a timer is a frequency meter with no wire in it);
  capture through an EIC line sensed on a LEVEL (so its event is a COPY
  of the pad) exact at 2344/937 ticks; and READING CCx PROVEN TO BE THE
  ACKNOWLEDGEMENT - four unread periods raise INTFLAG.ERR and lose
  captures, four drained ones raise nothing. TWO PIN FACTS that complete
  the picture the other suites started: A PAD HANDED TO A DRIVING
  PERIPHERAL FUNCTION DOES NOT MOVE UNDER ITS OWN PULL (function E takes
  the output driver, where the EIC's input-only function A leaves the
  pull in charge), and A PAD LEFT UNDER PORT IS NOT SEEN BY A DIGITAL
  CAPTURE INPUT (where the AC's ANALOG input reaches the pad with no mux
  at all). Those two together are why THE SUITE DECLINES TO JUDGE ERRATUM
  1.20.2: no controlled edge can reach a muxed WO pad from inside the
  chip, so it prints what it can (CC0 did not stay zero after the
  handover) and claims nothing - one wire would settle it. A LESSON WORTH
  KEEPING: a bench.verdict() line is ~4 ms of console at 115200, and the
  first version of letter b measured 4849 ticks where 4687 were due
  because a print sat inside the measurement window; a window is now
  opened and closed by two reads with nothing between them, and it is
  sized to fit the counter that reads it (at /256 a 16-bit counter wraps
  in 350 ms). Doc: docs/samc/tc.md (PROVISIONAL: the N-variant capture
  modes, DMA, sleep, and 1.20.2 unjudged).
  **TCC DONE 2026-08-28 (ch. 36) - PHASE D CLOSED, the family's richest
  timer and the first driver BORN under the ifdef-reserve rule.**
  samc/tcc.hpp NEW, with all of its per-instance and per-pad data probed
  in samc/device_tables.hpp and NOT ONE vendor-macro #ifdef in the driver
  itself. THE POINT OF THE CHAPTER IS THAT THE THREE INSTANCES ARE NOT
  COPIES OF EACH OTHER, where the TC's five differ only in which pads
  they reach: TCC0 is 24-bit with 4 channels, 8 outputs and all five
  extension units, TCC1 is 24-bit with 2/4 and only pattern + dithering,
  TCC2 is 16-bit with 2/2 and nothing - every number a TCCn_* constant,
  and TCCn_EXT confirmed to be exactly those five bits (31/24/0). TCC0
  and TCC1 SHARE gclk channel 28. AND THE PAD MAP NEEDS TWO KEYS: PA08 is
  TCC0/WO0 under function E and TCC1/WO2 under function F, so TccWo<Pin,
  function> takes the function and the reserve keeps two maps. Built: the
  whole register surface, seven waveform modes, RAMP1/2/2A (RAMP2C
  refused as variant-L, which also puts erratum 1.21.11 out of reach),
  dithering with tcc_dither() packing the shared low bits, the four
  waveform-extension stages, both fault systems, and tasks TccPwm
  (PwmChannel with a caller-chosen max, util's THIRD implementation) and
  TccPairPwm (the complementary pair - the TcdPwm analog in scale).
  ERRATA: eleven items, SEVEN LIVE, the most of any chapter here - 1.21.10
  is a COMPILE-TIME REFUSAL (ALOCK is not functional and has no
  workaround), 1.21.6 is code (clear the buffer-valid flag twice),
  1.21.11 unreachable by construction, 1.21.5/7/9 stated caller
  obligations, and 1.21.8 DID NOT REPRODUCE. NEW SUITE test_samc_tcc z
  143/143 THREE TIMES, wireless: five of TCC0's outputs land on free pads
  (PA08/PA09 on function E, PA22/PA12 on F), PA16 is the EIC fault
  stimulus, a TC at 3 MHz is the stopwatch and another counts events.
  TWO FINDINGS NO CHAPTER CARRIES, both of which CHANGED THE DRIVER'S API:
  (1) A BUFFERED WRITE'S SYNCBUSY BIT STANDS UNTIL THE UPDATE CONSUMES
  THE BUFFER - waiting it out took 256/480/1299 us of a 1333 us period on
  three runs and NEVER returned with LUPD set, and a second write inside
  that window is DISCARDED by the silicon - so set_cc_buffer/
  set_period_buffer/pattern_buffer REFUSE instead of waiting and return
  at once (a waiting duty() would have put a whole PWM period inside a
  PwmChannel); (2) A READ OF CCx OR PER WHILE A BUFFERED WRITE IS PENDING
  RETURNS THE BUFFERED VALUE, not the one the waveform is using - the
  register and the pad disagree and the pad is right, which is exactly
  the trap that makes erratum 1.21.8 look real. A THIRD found the same
  way: THE TWO HALVES OF CTRLB ARE NOT A SET/CLEAR PAIR FOR THE COMMAND
  FIELDS (writing zero has no effect on either), so a command is issued
  through CTRLBSET and cancelled only through CTRLBCLR - caught by a
  RAMP2 index held forever. Also measured: the two dead times exact at
  899/2701 stopwatch ticks against 900/2700 AND UNMOVED by a fourfold
  prescaler change (36.8.7's "GCLK_TCC cycles" confirmed against the
  obvious alternative); a complementary pair never both high in 400000
  paired samples, the two duties summing to 888 where 880 is the dead
  time removed; THE DUAL-SLOPE PERIOD IS EXACTLY 2 x PER (942 overflows
  in 2 s against 942 predicted, where 2 x (PER+1) gives 937) - the AVR
  TCD's printed formula was off by that one and this chapter's is not;
  dithering delivering a fractional period (938/933/928 against
  937/932/928); the output matrix and a LIVE swap (WAVE is the one
  configuration register this chapter does not enable-protect); pattern
  generation beating every upstream stage, with STATUS.PATTBUFV LAGGING
  its own write where CCBUFVx does not; a pin level through the EIC and
  an ASYNCHRONOUS channel clamping an output, halting the counter in both
  halt modes and timestamping itself, with EVCTRL.MCEIx PROVEN to be the
  gate 36.6.3.5 never names; ERRATUM 1.21.9 MEASURED (the same generator
  on a SYNCHRONOUS channel does nothing at all); a non-recoverable fault
  stopping the counter and forcing every enabled output to its NRV level,
  the state a LATCH and not a level; RAMP2's two interleaved cycles seen
  as duty (242/243 per mille from a 50 % compare) and IDX toggling at 504
  per mille; PPW capture exact at 2344/938 ticks; and CTRLA.MSYNC MOVING
  THE CHANNELS AND NOT COUNT - the client's matches jump 47 -> 937 a
  second while its own overflows stay at 47, which is what 36.6.4 says
  and not what a reader expecting two counters to track would guess. ONE
  MEASUREMENT LESSON, twice: at 1 kHz a counter wraps every millisecond,
  so TWO READS OF COUNT ARE NOT A WITNESS for "is it halted" (a halted
  counter gave 0 then 116, READSYNC being a command that must cross the
  domain the halt stopped) - the OVERFLOW FLAG is; and the pad sampler
  needs ~30 periods, so a 100 Hz waveform wants ten times the samples a
  1 kHz one does. Doc: docs/samc/tcc.md (PROVISIONAL: DMA, sleep, the
  debug fault, the advanced capture modes, and 1.21.7/1.21.8 not judged).
  **XOSC + FDPLL96M DONE 2026-08-28 (ch. 20) - PHASE E's FIRST HALF, and
  THE BOARD FINALLY HAS A SCALE. NOT COMMITTED; awaiting Fable's
  review.** samc/clock.hpp GREW rather than gained a neighbour, closing
  its own declared gap: Oscctrl (the block - the STATUS register all
  three roots report into, the seven interrupt sources behind the shared
  IRQ 0, the CFD's event output with its published EVSYS code), Xosc
  (crystal or external clock, the GAIN 20.8.5 makes mandatory in crystal
  mode and which xosc_gain_for() derives from the frequency the caller
  states, AMPGC, the STARTUP masking counter, ONDEMAND/RUNSTDBY, and the
  clock failure detector with its safe-clock prescaler and SWBEN) and
  Fdpll (three references, dpll_ratio_for() choosing LDR/LDRFRAC in
  SIXTEENTHS of the reference with an exact/inexact answer,
  dco_hz/output_hz, the output prescaler, CLKRDY as the bit that matters,
  the on-the-fly ratio change and the lock timer's OWN GCLK channel).
  Both have a compile-time init<cfg>() twin whose static_asserts name the
  rule a configuration broke (four new negatives). ERRATA AS CODE, all
  live on rev F: 1.25.1 (spurious unlocks below 25 C gate the output
  clock away, so FdpllConfig::lock_bypass DEFAULTS TRUE - the erratum's
  own workaround), 1.3.3 (ratio_updated() reads INTFLAG because
  STATUS.DPLLLDRTO does not rise) and 1.3.4 (lock_timer_clock() is the
  verb, and set_ratio() states the obligation it cannot enforce); 1.22.1
  verified NOT this silicon by the row AND by behaviour. NEW SUITE
  test_samc_clock z 108/108 three times, wireless. THE HEADLINE: THE
  24 MHz CRYSTAL RAN FOR THE FIRST TIME on this board (start-up
  554..576 us with STARTUP = 4) and weighed against it OSC48M IS
  47.755 MHz - 5100 ppm SLOW, inside table 45-57 and a SCALE that
  corrects every absolute frequency this stratum ever reported, all of
  which were ratios against OSC48M multiplied by a nominal 48 MHz
  (OSCULP32K: 33074 Hz that way, 32907 Hz against the crystal - and
  test_samc_platform's watchdog figure rides SysTick, so those "two
  witnesses sharing no mechanism" shared one after all). Also: A REAL
  CLOCK FAILURE INDUCED WITH NO WIRE - clearing XTALEN leaves XIN a
  digital input nothing drives - with XOSCFAIL, the latched INTFLAG and
  the safe-clock switch all observed and recovered through SWBEN; the
  DPLL's ratios EXACT to the count (127500 for LDR 23 = 48 MHz, 130156
  for LDR 23 + 8/16 = 49 MHz, 63750 for a 96 MHz DCO divided by four),
  lock in ~40 us, erratum 1.3.3 seen in one reading (INTFLAG 1,
  STATUS 0), and INTFLAG.DPLLLTO PROVEN NOT TO MEAN WHAT ITS NAME SAYS
  (with LTIME = 8 ms the loop comes up with CLKRDY = 1, LOCK = 1 and the
  flag SET - it marks the timer finishing, which in that mode is how the
  output is released). THE CPU RAN FROM THE CRYSTAL-LOCKED DPLL and came
  back, console alive throughout - a proof, not a policy: Clock<> still
  implements ClockSource::internal only, because which root CLK_MAIN
  takes is the DynamicClock design decision, reserved. AND A CORRECTION:
  GENCTRL's DIVSEL divides by 2^(DIV+1), NOT by a fixed 2^(field
  width+1) - measured by counting generator 5 against generator 0 with
  both fed by OSC48M (2, 16, 512 for DIV 0, 3, 8) and confirmed on the
  16-bit generator 1 (512 for the same DIV 8); the AC campaign's note in
  GclkConfig, ac.md and ac_sync_probe's comments are corrected, the
  probe's own measurements unaffected. Docs: docs/samc/clock.md grown
  (still PROVISIONAL: the main-clock TASK, DynamicClock, external-clock
  mode, sleep).
  **SUPC DONE 2026-08-28 (ch. 22) - PHASE E's SECOND HALF. NOT
  COMMITTED.** samc/supc.hpp NEW: Supc (the block, the six flags
  including the three BODCORE ones the chapter does not draw), BodVdd
  (level/action/hysteresis, continuous or sampled, the enable-protection
  AND write-synchronization dance in one verb, matches_fuses() against
  nvm.hpp's user row - which GREW a bodvdd_hysteresis() accessor for bit
  41), BodCore (READ-ONLY BY DESIGN: 22.6.3.4 and table 9-4 both say its
  calibration must not change, so there is no setter to call), Vreg (no
  enable verb at all - 22.8.6 forbids the change - only RUNSTDBY, which
  erratum 1.8.14 makes a workaround) and Vref (the bandgap: three levels
  out of sixteen codes, and VREFOE, which is what docs/samc/ac.md's gap
  list was waiting for). NEW SUITE test_samc_supc z 44/44 three times,
  wireless AND NOTHING FORCED - every threshold carries ACTION = none, so
  STATUS.BODVDDDET still tracks and a sweep is a measurement rather than
  a reboot. THE MEASUREMENTS: THIS BOARD RUNS AT ~5.1 V, located three
  ways through the AC's own VDD scaler against INTREF (5251/5141/5090 mV
  at 1.024/2.048/4.096 V, the crossing step doubling with the reference
  as a real voltage must) - the first supply measurement here, and the
  closing of ac.md's bandgap gap; THE BODVDD LEVEL STEP IS 48.7 mV,
  settling table 45-18 against itself (it STATES 60 mV while its own
  three anchors imply 47.5); enable-protection observed both ways, and
  the finding that cost a restore: A STORE CARRYING CONFIGURATION AND
  ENABLE = 1 TOGETHER SETS THE BIT AND LEAVES THE PROTECTED FIELDS ALONE,
  so the protection is judged on the value being WRITTEN (configure()
  now sets ENABLE on its own); a sampled detector NEVER reports ready
  (22.8.4, at 20 ms with a 512 Hz sampling clock); SUPC_BODCORE reads
  0x0028000A - enabled, action RESET - at an offset ch. 22's summary
  marks Reserved, with its two undrawn status bits set, so the device
  header was right and the chapter is incomplete; and ERRATUM 1.5.6 IS
  REAL (a spurious COMP flag on a bandgap enable, seen at the 2.048 V
  reference). The fuse row and the register agree field by field, and the
  boot BODVDD is restored bit for bit. Doc: docs/samc/supc.md
  (PROVISIONAL: nothing forces a brown-out, standby, BODCORE stays
  read-only).
  **RTC DONE 2026-08-28 (ch. 24) - PHASE E's DRIVER HALF CLOSED. NOT
  COMMITTED.** samc/rtc.hpp NEW: one counter wearing three faces
  (COUNT32 with one 32-bit compare, COUNT16 with PER as its top and two
  compares, the CLOCK/calendar with its masked alarm) over the THREE
  OVERLAID REGISTER VIEWS, handled the way tc.hpp handles COUNT8/16/32 -
  the control surface written once against MODE0, the width-carrying
  verbs in explicit flavours, nothing dispatching at run time on a mode
  the caller chose at compile time. THE DESIGN POSITION HELD: the driver
  NEVER writes the clock select - OSC32KCTRL.RTCCTRL is osc32kctrl.hpp's
  register and rtc.hpp only states 21.6.7's disable-first obligation and
  takes the chosen RATE where arithmetic needs one. No TASKS built, the
  avrdx/rtc.hpp precedent (alarm clocks and slow periodics are policies,
  born with their first user); the kernel Ticker stays on SysTick and
  ticker.hpp was not touched, nor was one line of util/ or kernel/.
  Erratum 1.16.3 (write corruption on a partial access, LIVE on every
  revision) is answered STRUCTURALLY - there is no verb that writes
  COUNT or CLOCK in pieces - while 1.16.1 (rev B) and 1.16.2 (rev B..E,
  its F and H marks belonging to the N-family row) are named and the
  second is DISPROVED BY BEHAVIOUR at the bench. NEW SUITE test_samc_rtc
  z 125/125, eight letters, wireless, 40 s. THE INSTRUMENT IS THE
  CRYSTAL: a TC0+TC1 pair as a 32-bit stopwatch clocked from generator 2
  off the 24 MHz crystal, and FREQM measuring the RTC's source against
  the same crystal, so this is the first suite in the stratum whose
  every absolute frequency is on a non-RC scale. Findings in rtc.md:
  THE COUNTER COUNTS ITS SOURCE tick for tick on all four clock selects
  (90..730 ppm, and the residue is the RC's own wander between the two
  instruments' windows, printed as a spread of 350..1100 ppm); the
  prescaler exact to 70..300 ppm across DIV2/DIV32/DIV1024, measurable
  at all only because the rate windows are EDGE-ALIGNED at both ends;
  PRESCALER = OFF divides by one AND SILENCES every periodic event
  (24.8.1's sentence, and the reason both codes are named); a COMP0
  event and a PER3 event each moving a DMA block over an ASYNCHRONOUS
  channel; MATCHCLR raising the compare AND THE OVERFLOW together
  (INTFLAG 0x81FF on a counter that never approaches its top); MODE 1's
  PERIOD IS PER + 1 SOURCE TICKS (1001 measured against 999+1, with the
  counter never seen above 998) and A MODE CHANGE DOES NOT CLEAR COUNT -
  a 16-bit counter left above PER never meets it and runs to 0xFFFF, a
  trap nowhere in the chapter that cost the suite a letter, and beside it
  A REAL DRIVER BUG THE BENCH CAUGHT - the device header's compare-EVENT
  group mask is ONE bit in the mode 0 view and TWO in the mode 1 view at
  the same position, so writing the shared control surface through the
  natural mode-0 macro silently drops CMPEO1 (fixed, with a verdict and a
  family static_assert on both widths); the
  calendar's every boundary in one second, its leap rule as the CHAPTER
  states it (YEAR[1:0] == 0 - 29 February in year+0, 1 March in year+1)
  and the year-63 wrap with OVF; THE ALARM ARRIVES A WHOLE COUNTER
  PERIOD AFTER ITS MATCH (989 ms measured on a 1 Hz counter - 24.6.2.5
  says so in a sentence easy to read past, and the first version of the
  letter reported a working alarm as broken); and CLKREP is a READING
  and not a format, the same CLOCK word being 11 PM or hour 27. THE
  READ-SYNCHRONIZATION ANSWER: a synchronized COUNT read costs 2.2 us
  against 0.19 us raw (so COUNTSYNC is background synchronization and
  not a per-read handshake into the 32 kHz domain), the readable value
  trails the counter by a CONSTANT four ticks - exactly four on all
  eight repetitions, which is the fact that matters - an unsynchronized
  read is FROZEN rather than wrong, toggling the bit costs ~5 ms, and a
  COUNT write takes ~190 us to appear in the shadow. TWO THINGS
  DECLINED, both printed: FREQCORR's per-step linearity, because THE
  TRIM'S WHOLE RANGE (129 ppm) IS SMALLER THAN THE WANDER OF EVERY CLOCK
  THIS BOARD CAN GIVE THE RTC (both selects are internal RCs moving
  100..300 ppm between windows) - a lock-in of seven ABBA blocks with a
  MEDIAN estimator does establish the sign and put the FULL SWING at
  415..620 ppm where 24.6.8.2's formula predicts 258, a factor of
  1.6..2.4 that the doc records and the driver does NOT bake in; and
  what one "count in the prescaler" is worth, since the DIV16 control
  moves with the window length too. Also found: 24.6.5 names a PERD
  event that no register summary, no register description and no header
  symbol implements. Family fixture test/family_samc/rtc.cpp + SEVEN
  negatives (MATCHCLR in COUNT16 - through the compile-time
  configure<cfg> twin - CLKREP outside mode 2, a Reserved prescaler
  code, a periodic event with the prescaler OFF, a compare event past
  the mode's channel count, the Reserved alarm mask, an impossible
  date). Doc docs/samc/rtc.md (PROVISIONAL: no tasks, sleep, the
  overflow event through EVSYS, the intermediate alarm masks, the
  12-hour rollover, XOSC1K/XOSC32K).
  **PM + SleepSite DONE 2026-08-28 (ch. 19) - PHASE E CLOSED. NOT
  COMMITTED.** samc/sleep.hpp NEW, the avrdx/sleep.hpp twin: `Pm` over
  the whole of chapter 19 (which is TWO registers - SLEEPCFG's three
  implemented modes IDLE0/IDLE2/STANDBY with the Reserved codes refused
  both ways, STDBYCFG's VREGSMOD and BBIASHS, the readback rule spent on
  every arming, and the one-way bus clock stated rather than hidden) and
  `SamSleepSite`, which gives util/power.hpp its SECOND SILICON WITH NOT
  ONE LINE CHANGED - the victory condition, and it held. THE LADDER
  MAPPING IS THIS TARGET'S OWN DECISION and the first place brio's
  never-deeper rule is not the identity: none -> IDLE0, light -> IDLE2,
  standby AND deep -> STANDBY, with armed() reporting standby for a deep
  request. `light` is IDLE2 rather than IDLE0 because there is no SEN bit
  here: IDLE0 is both a mode and the reset value, so armed() could not
  stay a pure read of the silicon otherwise (the price is a CAN wake from
  light, and brio has no CAN driver). platform_sam.hpp's ONE MANDATORY
  DEVIATION turned out to be nearly free - idle() ALREADY took whatever
  SLEEPCFG held, because this family selects the depth there and not in
  SCR.SLEEPDEEP, which is never written - so what it gained is a DSB and
  a SLEEPCFG read that buys ERRATUM 1.8.13's workaround (a SysTick
  interrupt coinciding with standby entry can hard fault while BBIASHS is
  set, and BBIASHS is SET AT RESET): the new SysTickInterruptGuard in
  ticker.hpp - the file that owns the register - is held across a standby
  WFI in both idle() and Pm::sleep(). Cost measured: +52 bytes on blink,
  +44 on console, the other twelve SAM images BYTE-IDENTICAL. THE TICK
  RULE, this campaign's central position: SysTick rides the CPU clock and
  the CPU clock stops, so KERNEL TIME STANDS STILL across a standby and
  the v1 policy is HONEST RESTRICTION - standby is legitimate when
  TimeEvents::ticks_to_next() is empty; the doc, ticker.hpp's grown
  caveat and the suite show the pattern, and an RTC-backed resync that
  would lift it is named as future work, not built. clock.hpp gained
  FdpllConfig::run_standby (defaulting false, so every image but
  test_samc_clock stayed byte-identical) and reset.hpp gained
  Watchdog::irq(), without which the early-warning interrupt the driver
  already had could not reach the NVIC. NEW SUITE test_samc_sleep z
  87/87 three times, eight letters, wireless, 6 s; every sleeping letter
  arms the WATCHDOG first so a lost wake costs a reboot and a banner
  rather than a mute board (it fired once, exactly as designed). THE
  NUMBERS: the SLEEPCFG bridge latency 19.6.3.3 warns about is ~5 us on a
  48 MHz CPU (so the readback rule is not a formality); leaving IDLE0
  costs NOTHING measurable over a polled wait while IDLE2 costs 3.5..4.4
  us more, repeatably, on a board with no CAN traffic - which chapter 19
  does not mention; leaving STANDBY costs 16.6..17.8 us, and SIX
  combinations of VREGSMOD x SUPC.VREG.RUNSTDBY x BBIASHS spread 2.1 us
  against 1.4 us of scatter in the same measurement repeated, so THIS
  FAMILY HAS NO SEPARATE REGULATOR BILL (the AVR's was a distinct 290 us
  item); a 499 ms standby advanced the kernel tick by 0 ms and a time
  event 50 ms away slept over for 249 ms matured 199 ms late; THE
  PERIPHERAL'S OWN RUNSTDBY IS THE WHOLE CLOCK REQUEST (a TC counted all
  1024 ticks of a standby with its generator's RUNSTDBY clear, with its
  SOURCE's clear, and on OSCULP32K which has no such bit at all - and
  counted 13 with its own clear); XOSC KEEPS RUNNING THROUGH A STANDBY
  whatever RUNSTDBY says and whoever is or is not asking, measured on the
  crystal's own counter in three arrangements with a deliberate-stop
  control, where table 19-2 says it should stop - so a standby here costs
  NO crystal restart against the AVR's 1.77 ms; and the watchdog runs
  through standby with its early warning as a second wake source (123 ms
  for a 128-cycle offset). METROLOGY LESSON WORTH KEEPING: the obvious
  wake measurement - N rounds of arm/sleep/wake differenced between IDLE
  and STANDBY on the RTC - MEASURES NOTHING, because the loop locks to
  the RTC and 512 rounds took 10240 ticks in both modes to the tick; a
  sub-tick overhead is quantized away, and the answer had to be timed
  single-shot on a through-standby crystal counter (which is itself
  sleepwalking, hence "the bill with the supply already up", said in
  print). A TC.HPP DEFECT FOUND ON THE WAY, documented in tc.md and
  tc.hpp and NOT fixed here: a synchronized COUNT read is ONE BEHIND -
  four consecutive count32() calls on a counter running for six
  milliseconds returned 0, 196, 201, 205 - so read_sync()'s waits return
  before the value THIS command latched is readable. Family fixture
  test/family_samc/sleep.cpp + two negatives (a Reserved SLEEPMODE code,
  the Reserved VREGSMOD code). Docs: platform.md GROWS its stopping half
  and keeps PROVISIONAL (sleep CURRENT is the big gap - no meter on this
  bench - plus the RTC-backed timebase, EIC wake, PAC, SLEEPONEXIT);
  design/power.md records the second target holding the contract
  unchanged; tc.md gains the one-behind read.
  **Build tooling is DONE, not part of this milestone any more**
  (2026-08-27): PlatformIO was stretched past its design use case (the
  env-per-app-x-board list would only have grown worse per family) and
  the whole repo migrated to CMake - `CMakePresets.json` (one configure
  + build preset pair per package x {release, debug}), app
  auto-discovery from `// build:` header comments
  (`CMakeLists.txt`, no generation step), the independent host-test
  project under `test/`, `tools/bench.py` retargeted, every PlatformIO
  file removed. Verified byte-identical `.hex` output against the old
  PlatformIO build before committing to the migration. The EDITOR half
  moved early (2026-08-25, forced, and unaffected by the later
  migration): cpptools' clang-based parser (1.33+) is structurally
  unable to parse the AVR-configured libstdc++ (x86-64 model +
  gcc-only types __int24/_Float32), so the editor is clangd over
  `compile_commands.json` - regenerated automatically on every CMake
  configure now (`CMAKE_EXPORT_COMPILE_COMMANDS`, no manual step),
  --query-driver for include paths and target, `CMakeLists.txt`'s
  `avr_predefines()` supplying the `-mmcu` device-macro delta clang
  lacks, `test/.clangd` pointing at the host project's own database
  (detail in docs/avrdx/README.md). The delta feed is an AVR
  peculiarity (device-specs macros) - confirmed by the second target:
  samc selects the device with a plain -D__SAMC21J18A__ and needs
  none of it. clangd routing is now per-stratum .clangd fragments
  (framework default = host DB; avrdx/samc override with their own
  database), fully decoupled from CMake Tools' Active Folder, with
  the repo-root .clangd suppressing the two clang-only diagnostics
  -Werror would turn into editor errors on gcc-clean code.
- **QK-style preemption (far horizon, probably not on AVR).** A
  preemptive non-blocking kernel (higher-priority AO preempts a
  lower one mid-dispatch, single stack, LIFO nesting) would be an
  EXTENSION of the present kernel, not a rewrite of what sits above
  it. What already holds: the AO contract and the three delivery
  primitives are unchanged; `Lease::dispatch` loans stay correct
  because the borrower-precedes-lender order (static_asserted by
  Kernel) makes the borrower preempt the lender right at the post;
  `Lease::reply` loans are ordering-independent. What would change:
  `post()` must trigger the scheduler when it readies a higher AO;
  the Platform gains an ISR-exit / pending-scheduler hook (PendSV-
  like: irreducibly target-specific); the loop becomes the idle-
  priority context; time-event maturation in the loop (T2) must be
  revisited (loop is idle priority then - candidates: tick ISR, or a
  top-priority pseudo-AO woken by the tick). Discipline to keep NOW
  so the door stays open: AOs share nothing but events (an AO's own
  statics are safe; a global touched by two AOs outside events is a
  one-way race under preemption).
- **Exhaustive-driver track (started 2026-08-19).** Done: EVSYS, VREF/
  DAC/ADC, CLKCTRL (clock.hpp rewritten as resources + tasks; `test_avr_clock`
  15/15 on the scope via CLKOUT/PA7: tune curve asymmetric, CFD
  fallback really 4 MHz, status follows the request - in clkctrl.md).
  TCA/TCB/CCL/AC done: tca.hpp (Tca resource + TcaPwm/TcaPwm16/
  FrequencyGenerator/Heartbeat/EventCounter), tcb.hpp (Tcb resource +
  PeriodicTick/Timeout/OneShotPulse/PulseCounter/CascadedCounter/
  meters/Pwm8), ccl.hpp (Ccl/Lut<n>/ToggleFlipFlop), ac.hpp (Ac<n>/
  Threshold/Window); `test_avr_timer` 82/82 on A5 (closed loop through
  EvPin generators, no wires; findings: capture = interval - 1 CLK_PER
  at div1, SEQCTRL before the even LUT, OSC32K +0.94 %, AC hysteresis
  17 mV). TCD done (see below). Multislope next when wanted. A 2026-08-20
  family-completeness review (all drivers vs their chapters, both
  errata docs, the device headers of every package) found real gaps
  and a few bugs; only VREF and DAC came out complete. Everything
  else is PROVISIONAL and each doc's "Not covered yet" (driver gaps
  kept distinct from bench gaps) is the shopping list; the code fixes
  are listed there too (TCA RESET-from-split without CMDEN, tasks
  missing rebase/clock_follows, CCL LUT3-ALT and per-package gating,
  ADC rebase failure path, AC package legality). Fixed 2026-08-20:
  TCB4 end-to-end, Tcb usable on every package (port_exists in
  pin.hpp + if constexpr on missing pin positions - the pattern for
  the other drivers), event_channels/SWEVENTB/TcbClock::tca1 gated by
  the device header, evsys_pulse(ch) helper; compile-verified for
  db28/32/48/64 and da28/64, test_avr_timer to re-run at the bench. Original order:
  EVSYS (docs/avrdx/evsys.md; avrdx/evsys.hpp primitives built and
  `events0` VERIFIED on the scope 2026-08-19: 512 Hz PIT/64 on PD2,
  button level, off, 4 Hz LED with no CPU; EventSystem static sugar
  when an app has several fixed routes) -> VREF, DAC,
  ADC exhaustively (docs/avrdx/adc.md; vref.hpp, dac.hpp, adc.hpp
  and `test_avr_analog`, the bench test SUITE (54/54 on rev A5 at
  3.3 V; measures VDD at start, run it at 5 V too) - a suite named
  test_<target>_<subject> is a reference test to keep passing through
  every restructuring; docs vref.md/dac.md/adc.md; 68/68 at 5 V too;
  `sampler` = the ADC inside the kernel via util/analog_sampler.hpp,
  bench-verified 512 samples/s no drops) -> TCB/TCA/CCL/AC (done as
  above; `test_avr_timer` is their reference suite) -> the Multislope
  app (the 64-cycle snapshot stays in the ISR body; EventSystem static
  sugar gets its first user there) -> TCD. Datasheet DS40002247B chapters: EVSYS 16, PORTMUX 17,
  VREF 21, TCA 23, TCB 24, TCD 25, CCL 31, AC 32, ADC 33, DAC 34
  (errata F: ADC, DAC, CCL 2.4, TCA 2.12, TCB 2.13, TCD 2.14 items).
- **Sleep pass DONE 2026-08-25** (both phases in one day, B's dead
  crystal notwithstanding - the peer runs on OSCHF): avrdx/sleep.hpp
  (Sleep arm/disarm/sleep/enter with the errata-2.2.4 NOP; Vreg PMODE
  under CCP, HTLLEN refusing while TWI client/CCL are enabled) + NEW
  SUITE test_avr_sleep z 72/72 (single-board) and y 49/49 (two-board,
  sleep_peer on B: PE0 one-wire commands, PE2 stimulus, PE3 echo into
  B's hardware TCB capture). Findings in platform.md: the peripheral's
  RUNSTDBY alone revives its whole clock chain; an oscillator's
  RUNSTDBY buys wake latency only; the regulator is a separate ~290 us
  bill and AUTO drops it only when OSC32K is the last clock left (so
  PMODE moves nothing beside a live crystal, everything in PD: 313 vs
  24 us OSCHF); crystal restart 1.77 ms in both deep modes; RTC.CNT
  stale ~1 ms at wake; SFD does not latch (line must still be low when
  the clock returns; the frame survives only if the clock is back by
  mid-start-bit; RXSIF never fires from Idle) - usart.md's deferral
  closed; TWI: only the ADDRESS MATCH wakes, a standby client's
  tenure is not measurably stretched, PD adds the crystal restart;
  CCL PD wake follows the LUT's CLOCK (OSC32K-clocked filtered LUT
  wakes). Remaining (platform.md "Not covered yet"): MVIO/BOD-VLM
  wakes, sleep current, the power-manager AO (util pass).
- **NVMCTRL campaign DONE 2026-08-25** (board A only, no wires):
  avrdx/nvm.hpp NEW - Nvm resource over the whole ch. 11 (CCP-SPM
  command discipline with NOCMD between commands; flash read = ELPM,
  write = SPM with 24-bit addresses, NEVER the FLMAP data-space window
  - DA errata 2.7.1 inapplicable by construction; page/multi-page
  erase with a WHOLE-RANGE protection guard because both families'
  2.7.x errata make the silicon check only the FIRST page - POSITIVELY
  OBSERVED via erase_ignoring_protection: a 2-page erase over an
  APPDATAWP'd page erases it with NO error; EEPROM
  EEWR/EEERWR/EEBER/EEMBER, EEREADY as the LEVEL flag it is + ISR
  body; USERROW runtime writes (flash commands + ST); SIGROW typed
  view; FlashLayout<boot,code> compile-time geometry claim +
  matches_fuses(); scratch_region() = the hole in the MIDDLE of the
  part, 65536..98304 under the bench geometry, because gcc puts
  .rodata at 96K and code low; one-way protection verbs;
  vectors_in_boot()) + EepromStore backend; util/ NEW: crc.hpp,
  nv_record.hpp (magic+version+CRC-16 record, store() writes ONLY
  changed bytes - 0 for an unchanged value, host-tested),
  nv_writer.hpp (writer AO, one byte per EEREADY interrupt, BusMaster
  shape), persistent_panic.hpp (polled panic Reporter + boot-side
  take()). TWO BUILD INVARIANTS now in every image:
  src/glue/ivsel_boot.cpp ([common] base_src_filter) arms
  CPUINT.IVSEL in .init3 (with BOOTSIZE != 0 the default vector
  location is the APP section start - every ISR would jump into
  erased flash; the store is correct under both geometries) and
  pio_flags.py links -Wl,--defsym,__flmap_lock=1 (FLMAPLOCK set by
  crt; per-app escape `// pio: custom_flmap_lock = 0`). Fuse policy:
  board A BOOTSIZE=128 (all code in BOOT), CODESIZE=0, written by the
  NEW bench.py `fuses` verb - fuses are UPDI-only (11.3.1.5). NEW
  SUITE test_avr_nvm z 112/112 (a..f; u USERROW 9/9 and g
  APPDATA+erratum 11/11 sit OUTSIDE z: each costs wear or a fuse
  round-trip), re-runnable in one power-on (letter a skips its
  FLMAP-mobility half when the one-way lock stands from a prior run).
  Findings (nvm.md): flash word write measures 83-84 us, ABOVE the
  datasheet's 75 us max; a page erase halts the CPU for the full
  10.1 ms - interrupt latency through it 9078 us, and a 1024 Hz
  software timebase advances ONE tick across it, not ten (the PIT
  interrupt is serviced once); an EEPROM write does NOT halt the CPU
  (10476 polling turns observed during one); multi-byte/multi-page
  erases cost single-unit time; CMD is 6 implemented bits drawn as 7
  (0x7F reads back 0x3F) and a store with NO command selected is the
  only way to see invalid_command; ERROR has 5 codes in the headers
  vs 3 in the datasheet; DB errata 2.7.2's "EEWP" bit does not exist
  - this family's EEPROM has NO write protection at all; EESAVE
  verified both ways over UPDI (chip erases leave fuses and USERROW
  intact); NVMCTRL is byte-identical across all 8 device headers (no
  package gating needed). CHER/EECHER and software fuse writes
  deliberately not exposed. Family 16 TUs x 8 + 69 negatives (10 new),
  native 15 suites (test_nv_record NEW); regressions under the new
  fuses/glue/defsym re-verified by hand: platform z 96, sleep z 72;
  twi z DEFERRED until the office I2C bus is re-jumpered. nvm.md
  PROVISIONAL: bootloader not built (door open: BOOTSIZE=1 fits
  Optiboot-DX), flash journaling/wear-leveling policy waits for its
  first user, DA silicon facts datasheet-trusted, power-loss
  mid-write unexercised. End state: A = test_avr_nvm (fuses
  BOOTSIZE=128 CODESIZE=0), B = sleep_peer but UNPLUGGED from the
  desk (manifest notes it; console by-path re-verified: A moved to
  the hub socket the manifest gave to B).
- **NvHeap DONE 2026-08-26** (design in design/nv-heap.md, full story
  in memory flash-alloc-design): util/nv_heap.hpp (FlashMedia concept
  + the allocator: ping-pong map pair in the last 2 pages, headerless
  blocks, survival-aware mount judging by CRC not build-id,
  alloc/append/seal/rewrite/find, max-clearance placement, no free) +
  host/sim_flash.hpp + avrdx/nvm_flash.hpp; suites test_nv_heap
  (host, 44 cases / 2656 assertions, power-cut sweep at every write
  boundary, both 512/2 and 2048/8 geometries) and test_avr_nvheap
  (bench, on BOARD B for wear rebalancing, z 51/51 + the reflash
  choreography: blocks survive a default reflash, --erase wipes
  clean). THE ERASE-REGIME REVERSAL, measured twice: avrdude's
  DEFAULT is the page-selective erase (only the image's pages), `-D`
  disables erasing entirely and ANDs the image into old bytes (a
  trap), `--erase` now passes a real `-e`; bench.md carries the
  measured three-regime table, bench.py flash gained the NvHeap
  preflight (reads the chip's map, warns which blocks the new image
  lands on, never blocks). __nvheap_build_id defsym = newest source
  mtime (deterministic rebuilds). B provisioned BOOTSIZE=128
  CODESIZE=0. nv-heap.md PROVISIONAL: ring/log journaling, payload
  wear levelling, per-image flag, zone hint, silicon power-loss
  atomicity (host-swept only), second target.
- **OPAMP campaign DONE 2026-08-26** (board B only, no wires; the last
  unclaimed low-level chapter): avrdx/opamp.hpp NEW - `OpampSystem`
  (the block: the one ENABLE, TIMEBASE = ceil(CLK_PER/1 MHz) - 1, and
  the chapter's single ClockUser hook so a settle time keeps meaning
  microseconds across a rebase; PWRCTRL/IRSEL, DBGCTRL) + `Opamp<n>`
  (both input muxes with their PER-INSTANCE link codes refused
  otherwise, the 16R ladder as eight EXACT rationals - naming 16/15
  "1" would be a lie - the output driver, the three enable regimes of
  35.3.2.7 as one enum, the internal timer, the four event users and
  the offset trim; pads claimed and released by hand) + tasks
  `OpampFollower`/`OpampPga`/`OpampInvertingPga`/`InstrumentationAmp`
  (the integrator deliberately NOT built: it needs an external R and C
  and a DUMP policy, and is born with Multislope). DB-ONLY: the whole
  header is gated on the device header's OPAMP symbol and the NEW
  `opamp_count` in evsys.hpp (a concept over `OPAMP_t::OP2CTRLA` - a
  requires-expression on a non-dependent type is a hard error, so it
  had to be a concept), which also FIXED a latent evsys bug:
  `EvOpampReady<2>`/`EvOpampCtl<2, ...>` indexed past the EVSYS struct
  on 28/32-pin parts and now refuse. NEW SUITE test_avr_opamp z 96/96
  on B, WIRELESS - the DAC's buffered output is the source, the ADC
  reads every OUT pad, a TCB latches READY through EVSYS, PD0 supplies
  the LEVEL the DUMP/DRIVE users need, and both converters on VDD make
  it ratiometric. Findings in opamp.md: the follower tracks its source
  to 0 mV over a nine-point sweep and the NON-INVERTING ladder is exact
  to a permille at all eight wipers, while the INVERTING one (its
  bottom driven by the DAC buffer) runs up to 5 % high in the middle;
  ONE SETTLE UNIT IS EXACTLY ONE TIMEBASE MICROSECOND (deltas 723/960/
  1128 against 720/960/1128 CLK_PER ticks) but the WARM-UP on a cold
  ENABLE is 365 ticks = 15 us where 39-27's TON is 1 us, and a
  restart() of a running op amp pays only 21; READY is issued in
  EVENT_ENABLED mode ONLY (35.3.2.6 is exact where 35.3.3 reads wider,
  measured both ways); a RUNNING op amp HOLDS its OUT pad against a
  pull-up even with OUTMODE OFF, and DRIVE raises the driver on top of
  that; OUTMODE is ONE implemented bit drawn as two (0x3 reads back
  0x4) and OPnINMUX's bits 3 and 7 do not exist (0xFF -> 0x77); CAL's
  step measures 594 uV vs 500 and its DIRECTION is the opposite of the
  plain reading of 35.5.10 (a rising CAL lowers the output), the
  production 0x81 trimmed to 0x80 taking the residual from -490 uV to
  ~100; IRSEL is WRITABLE on A5, so errata 2.8.2 is confirmed rev.-A4
  only; the instrumentation recipe makes all seven of table 35-14's
  gains and only seven exist (both R1 values must be wiper positions);
  and the DUMP switch turns the floating INN pad into a visible
  integrator - the node keeps the dumped charge for milliseconds and
  then walks away. Family 18 TUs x 8 + 78 negatives (6 new: OP2 on a
  32-pin part, any Opamp on DA, MUXPOS LINKOUT on OP0, LINKWIP on OP1,
  MUXBOT LINKOUT on OP0 without OP2, OpampSystem not rebased); native
  16 suites green. LESSON RE-LEARNED THE HARD WAY: `int` is SIXTEEN
  bits here - `4096u * 16u` is 0 and gcc turned the whole suite into
  one `abort()`, which showed up only as a 2.7 KB image. The builds
  DID carry -Wall -Wextra all along (pio_flags.py, since the first
  kernel commit): unsigned wrap is DEFINED behavior and NO warning
  level reports it, so the only defenses are 32-bit literals (UL) and
  suspicion of a suddenly tiny image. What the sweep DID fix
  (2026-08-26, user-approved): -Werror added to pio_flags common and
  [env:native] (80/80 envs + native measured zero-warning first), and
  build_unflags now strips the platform-injected -fpermissive and
  -Wno-error=narrowing, so ill-formed code and narrowing fail the
  build as house rules demand.
  End state: B = test_avr_opamp with test_avr_nvheap's five blocks
  intact (verified by its letter v after the campaign's five
  reflashes), A untouched on test_avr_serial.
- **Util pass (2026-08-26, six steps agreed; 0-2 DONE).** Step 0,
  payload/ownership: lend<L>() maker + Borrowed converting ctor, the
  six reply-class raw pointers typed (NvWrite/TwiHost/SpiHost
  requests), 80 call sites, byte-identity proof on six images (the
  gate caught one +10-byte hoisting). Step 1: util/testbench.hpp -
  the suite grammar's ONE implementation + host suite test_testbench
  (bench.py's own regex re-run on the emitted bytes), test_avr_opamp
  migrated as exemplar (z 96/96 unchanged); others migrate when
  touched. Step 2, PowerManager: kernel gained
  TimeEvents::ticks_to_next() (wrap-safe, host-tested);
  util/power.hpp - SleepDepth ladder {none,light,standby,deep} with
  the never-deeper mapping rule, SleepSite concept (arm/disarm/
  armed), PowerLock RAII ceilings, PrepareSleep/SleepVote unanimity
  round, WakeReport delivered opportunistically (if constexpr on the
  voter's variant - publish would tax every queue); BusMaster is a
  voter (ok iff idle+empty; +1 queue slot so a dropped vote cannot
  hang unanimity; measured cost +48 flash/+11 RAM where a bus is
  actually arbitrated, ZERO elsewhere - spi image byte-identical);
  AvrSleepSite in sleep.hpp; THE ONE MANDATORY DEVIATION:
  AvrPlatform::idle() was unconditionally re-arming IDLE and
  overwrote any armed mode - it now honors a standing SEN (armed
  above = sei+sleep+return, the arming is the manager's to clear;
  a managerless program pays one bit-test per idle). Contract:
  the FIRST EVENT AFTER WAKE disarms and reports. NEW SUITE
  test_avr_power on B, z 44/44 twice (standby round through the
  kernel 157 us with two voters; 32 turns asleep over 32 PIT ticks
  vs ~13400 awake; standby wake +10..12 cycles - crystal kept alive
  by the RUNSTDBY stopwatch; deadline guard refuses deep with no
  voter asked; unanimity not first-no; nested locks shallowest-wins);
  design/power.md NEW; platform.md gaps updated (sleep current stays
  a manual bench task; a PDOWN round and an ISR-taken PowerLock
  unexercised). Steps 3-6 DONE same day: util/meter_sampler.hpp
  (MeterLatch<T, P, id> ISR bridge + MeterSampler AO - the AO paces
  PUBLICATION, capture-rate events would flood queues; design/
  meters.md NEW; suite test_avr_meter on B z 38/38 twice, wireless:
  1/5/20 kHz to the tick, ~19989 ISR captures vs 8 published per
  1024 ticks, missed+published accounts for every capture, dual
  source labeled by pack order; suite development also re-observed
  tcb.md's arming-edge fact), util/trace.hpp (stamp/dump ring,
  disabled specialization PROVEN zero storage - an object, not
  statics, exactly so that proof is real), util/input_scanner.hpp
  (N-consecutive debounce, power-on establishes state without an
  edge; bench waits for the traffic testbed's buttons),
  bus_master.hpp policy hook (BusAction pass/retry, attempt counter,
  BusPassThrough::never_retries compiles the whole branch out - twi/
  spi/power hexes byte-identical; a retrying master votes not-ok to
  PrepareSleep; the I2C recovery ladder and multi-host backoff stay
  on demand per i2c-bus.md) + NEW host suites test_meter_sampler/
  test_trace/test_input_scanner/test_bus_master (native = 22).
  Follow-up fix found by the new suite and applied at review:
  BusMaster::init() now resets FIFO/tallies/reply like every other
  AO's init (+24 bytes where a bus is arbitrated). AO inits that
  need runtime config default their init() args - Kernel::init_all
  needs the no-arg form; the app arms them after. THE UTIL PASS IS
  CLOSED.
- **Low-level review track (planned 2026-08-20, full plan in memory
  low-level-review-plan).** Everything in avrdx/ gets the Working
  discipline treatment, device by device. Phase 0 DONE 2026-08-20:
  family-compile fixture (tools/check_family.sh, all 8 DA/DB
  packages, negatives included) and bench baseline re-run after the
  TCB edits - test_avr_timer 82/82, test_avr_analog 68/68 at 5 V
  (VDD measured 5190 mV), test_avr_clock 15/15 (the old "14/14" note
  lagged a suite addition).
  Phase 1, the ex-EXHAUSTIVE set (findings in each doc's "Not covered
  yet"): TCB DONE 2026-08-20 (tasks are ClockUsers, Timeout TOP =
  ticks bench-proven exact +2-tick constant path offset, snapshot_on
  edge/filter, 0x10000 truncation fixed, CFD block header-gated so
  clock.hpp compiles on DA; suite now 14 tests, 117/117 incl. live
  rebase 24->12->24 under a running meter, noise canceler = +3
  CLK_PER differential, ALT1 verified on TCB2/PB4, CCMPINIT, div2/
  tca1 clocks; findings: TIMEOUT re-fires CAPT at every CNT wrap
  through TOP, arming a sync CAPT-in on a high channel reads a
  spurious edge. Queued: util/ meter-AO usage type; console move to
  free PF4/PF5 for TCB0/1 ALT1; RUNSTDBY bench pass, LOW priority,
  needs a sleeping app) -> TCA DONE 2026-08-20 (reset carries CMDEN
  through the split view - the escape bench-proven by test s; routes
  = exactly the device header's codes, PORTG/TCA1-PORTE on 64-pin,
  gated by PORT instance macros; CTRLC output_value verbs; split
  flags/interrupts/ISR bodies + HunfEvent; PORTMUX_TCA1_gm and the
  enum-OR latents fixed; suite 15 tests 124/124; TcaPwm16 PER=MAX
  restated as the deliberate endpoint policy; PLUS TcaPwmCentered -
  dual-slope center-aligned task, OVF-at-centre bench-proven by the
  stamp technique, suite 16 tests 129/129. MeterSampler (meter AO,
  util/) APPROVED by the user, scheduled for the util pass) -> CCL
  DONE 2026-08-20 (lut_count/has_pins from the device header - LUT5
  works pinless on 48-pin, another dead-instance latent like Tcb<2>;
  LUT3-ALT refused; pin release on re-init; INTCTRL1 gated; DFF
  comment/doc corrected: clocked, the latch is the transparent one;
  suite tests l+i, 18 tests 141/141; findings: LINK/DFF/RS verified,
  ALT1 on PC6 + release, TCA WO and AC as LUT inputs, filter delays
  sync +2 / filter +4 CLK_PER differential, OSC32K filter ~4 cycles
  = 2871 ticks; open: slow-domain-proof stamp protocol) -> AC DONE
  2026-08-20 (ac_config_valid + port_exists refuse the PORTE
  positives on small packages, runtime init returns false;
  ac_dacref_* delegate to util/analog.hpp; state() comment fixed;
  suite test w, 19 tests 159/159: pin-vs-pin with a GPIO negative -
  no wires, AC1 + its OUT event consumed, INVERT, PA7 and PC6 OUT
  pins, one interrupt per window-sense entry) -> ADC/DAC/VREF, CLKCTRL,
  EVSYS DONE 2026-08-20, closing Phase 1: ADC gained debug_run,
  window_signed, adc_neg_valid (enum-negative selects return false),
  clock_ok() after rebase, init<cfg> returns bool, vdd_div10/
  vddio2_div10 gated by #ifdef MVIO (DA reserves the codes); DAC
  code() readback; test_avr_analog test x, 15 tests 81/81 (dacref
  inputs +-7 counts, signed window, live rebase 24->12 within 3
  counts). CLKCTRL: set_hz -> bool guard, config latches handled
  (start_* stop an enabled oscillator first), DA external clock
  implemented DATASHEET-TRUSTED (EXTS waited before selecting), task
  restrictions documented as deliberate; test_avr_clock still 15/15.
  EVSYS: the whole generator/user table filled (UPDI/MVIO/ZCD/OPAMP/
  USART XCK/SPI SCK/TCD0 gens; IRDA/TCD/OPAMP users; EVOUTG; PORTG
  pins; codes verified against the header's own enums; usart_count
  tiers), EvOut family ALT1 rule + full unlisten teardown;
  test_avr_timer still 159/159. Family fixture now 7 TUs x 8 MCUs +
  20 negatives, all green. Phase 2: PORT DONE 2026-08-20 - Port<L>
  resource (take_flags ISR body, slew, mask verbs, multi-pin engine),
  PinConfig one-store configure (dodges the INVEN/ISC same-cycle
  hazards), PinSense incl. level_low/input_disable, INLVL gated
  (#ifdef PORT_INLVL_bm, DB only), Pin::flag/clear_flag with the W1C
  plain-store discipline, fully_async fact (Px2/Px6),
  PinSet::configure grouped by port at compile time; NEW SUITE
  test_avr_pin 22/22 (findings: W1C clear lands one cycle after the
  store - back-to-back read sees old flags; level_low re-fires
  continuously; INVEN inverts the sense; input_disable freezes IN at
  the last value); errata 2.9.1 (PD0 floating on DB 28/32)
  documented, deliberately NOT wrapped (user ruling: silicon kludges
  stay visible); port.md rewritten to shape; family TU pin.cpp + 3
  negatives (INLVL on DA refused). RTC DONE 2026-08-20 - new
  avrdx/rtc.hpp with three resources (RtcClock owns the CLKSEL both
  functions share - in a brio program the Ticker is its owner; Rtc the
  counter; Pit the periodic timer), BasicTicker migrated onto Pit with
  its public API unchanged (test_avr_timer still 159/159), the whole
  chapter-26 register description exposed, a negative CALIB trim at
  DIV1 refused at compile time and at run time; NEW SUITE test_avr_rtc
  78/78, no wires (a TCB cascade at CLK_PER latched by the RTC's own
  OVF/CMP events is the stopwatch). Findings: the compare fires exactly
  CMP + 1 ticks after the overflow; the busy flags live ~2.8 CLK_RTC
  (2005..2107 crystal ticks); the first PIT interrupt after an enable
  falls at 4845..15902 of a 23424-tick period with the prescaler free
  and 14484..16525 with it stopped; PIT_DIV64 does not move with the
  counter's PRESCALER (evsys.hpp's comment corrected); exactly 1024 PIT
  ticks per 32768-cycle counter period; OSC32K +9000..+9800 ppm and
  wandering 100..300 ppm, which is the noise floor of everything here;
  the +-127 ppm trim measures +105..+148 / -110..-150 ppm and is
  GRANULAR (one whole CLK_RTC cycle every 1e6/ERROR cycles), so it
  shows up only by alternating trimmed and untrimmed periods and
  averaging. Tasks (alarm, slow periodic) deliberately NOT built: born
  with their first user. USART phase U1 DONE (single-board half):
  uart.hpp REPLACED by usart.hpp - Usart<n> resource (per-package route
  table incl. the pinless NONE + full teardown, every frame format and
  receiver mode, the baud arithmetic with actual_baud, STATUS W1C, the
  three errata as code: ODME pin direction, SFD as arm/disarm verbs,
  recover_from_isf) + tasks Uart (same public API, every app's console)
  / OneWire / Rs485 / SyncHost / SyncClient / MspiHost / IrdaLink /
  AutoBaud; NEW SUITE test_avr_serial 108/108 (menu letters a..i, z runs
  all). Findings: LBME is taken at the TXD PAD - a pinless loop-back
  receives nothing, now refused by the driver; the baud generator
  measures exact to the tick at 9600/115200/460800/1M and CLK2X; the RX
  FIFO keeps three frames and the third is the NEWEST, BUFOVF marking it
  alone; GENAUTO writes a BAUD value directly (+0.06 % at 19200),
  LINAUTO needs a real >= 12-bit break (produced by driving TXD from
  PORT with TXEN off), errata 2.16.3 confirmed; STATUS.WFB is
  write-only. USART phase U2 DONE (two-board half): usart_peer on B
  driven IN-BAND over the link (src/apps/usart_link.hpp: magic/opcode/
  checksum, ack-before-act, bounded actions with autonomous return to
  command mode 8N1/115200); both apps DISCOVER the wiring topology
  (crossed pair vs one shared TXD-TXD wire - the desk turned out to
  carry the shared wire, caught by the edge-count probe, suite v /
  peer console 2); test_avr_serial grew j..u/v/w, sets y 103/103 and
  w 6/6 (both close with the ALL: line), z still 108/108. Findings in
  usart.md: FERR onset +5/-6 % vs table 27-4's ~4 %, glitch rejection
  at half a bit, ABW windows measured at 14-16/16-18/20-22/>22 % (doc
  15/18/21/25), auto-baud learns a foreign clock to its real offset
  and accepts BAUD 80 below the documented 0x64 floor, LINAUTO only
  re-locks near the BAUD in force while GENAUTO learns anything, MPCM
  5..8-bit flavour is receiver-only, RS-485 XDIR = 1 baud-clock guard
  + frame (exact), RXPL counts CLK_PER cycles not samples, TXPL=0xFF
  makes only the TRANSMITTER plain async, an LBME receiver DOES hear
  an external driver on its TXD pad (one-wire collision detection is
  real, proven with a bench collision), RXCIF leads the sender's
  TXCIF by half a bit (turnaround guard documented), board B's 24 MHz
  crystal does not start (runs OSCHF +0.24 %). Driver fixes: Uart::
  init drains stale rings/counters, wait_line_idle re-commented (one
  call per burst). Crossed pair refitted: sync roles VERIFIED (exact
  at 100 kHz and 1 MHz, both INVEN phases - host and client invert
  TOGETHER; the client's CLK_PER/4 ceiling is real, 12 MHz garbles
  every run), y 110/110; the fix round killed the peer's topology
  latch (un-latches after 3 s of silence, discovery drives nothing
  until proven, console 0 = manual escape) and q's FIFO-drain suite
  bug (collect live - the 3-deep FIFO made the ceiling test pass
  vacuously). USART campaign closed but for the declared deferrals:
  MSPI electrical (SPI campaign), SFD from standby (sleep pass),
  DBGRUN, IREI, the LIN protocol layer. SPI phase S1 DONE (single-board
  half): spi.hpp REBUILT from chapter 28 - Spi<n> resource (the route
  table per package with the FIRST errata-beats-header gate: SPI1 ALT2
  refused on 48 pins, DB 2.11.1; a pinless host must set SSD, DA 2.10.1;
  both roles, the seven rates + the spi_clock_for chooser, both INTFLAGS
  layouts with their differing clear disciplines, the host demotion
  readback and re-arm, two ISR bodies, full pin teardown) + tasks
  SpiHost<n, route> (the old engine's public behaviour intact: two-phase
  descriptors, ISR pump vs polled, CS/DC, cs_setup_us, the CPOL preset
  applied with the peripheral disabled; new: an optional SCK ceiling that
  rebase re-resolves) and SpiClient<n, route> (polled surface + the ISR
  bodies, ClockUser for the CLK_PER/6 ceiling). Apps and spi-bus.md
  follow the rename Spi -> SpiHost, and SpiMode replaces the raw
  SPI_MODE_x_gc apps were passing. NEW SUITE test_avr_spi 148/148 (10
  tests, no wires, SPI0 ALT1 only - DEFAULT is the 3.3 V display
  cabling). Findings: all seven rates exact on SCK (CLK_PER/2 included);
  a host's MISO direction override is LATCHED AT ENABLE (a DIRSET under a
  running SPI does nothing); MOSI parks HIGH between transfers; a W1C
  store to IF leaves WRCOL standing (only the read-then-DATA sequence
  clears it) and a store to DREIF does not clear it; BUFOVF waits for the
  NEXT transfer; an SS pin driven low as an OUTPUT does not demote (table
  28-2) - the suite forces the demotion with the pin's own INVEN. DESK
  CHANGE found by the wiring probe: PORTE is now wired STRAIGHT THROUGH
  (A.PEn - B.PEn, n = 0..3), not crossed - so test_avr_serial y is
  103/103 with q skipping itself, w is 6/6, and S2's host/client link
  needs no re-jumpering. SPI phase S2 DONE (two-board half): spi_peer
  on B driven IN-BAND over the SPI bus itself (src/apps/spi_link.hpp:
  magic 0xB8, checksum, bounded actions; the peer is a DARK LISTENER -
  MISO driven only for one answer window after a checksummed frame
  decodes, so test_avr_spi z scores its 148 with the peer attached);
  test_avr_spi grew k..s, y = 92/92. Findings in spi.md: the client
  matrix (4 modes x dord x 3 regimes) exact both ways at div32; the
  buffer-no-BUFWR dummy is the shifter (0x00 after init); the client
  ceiling is the errata's CLK_PER/6 with OBSERVED asymmetric
  corruption at /4 and /2 (client mis-samples first); CPOL/CPHA
  mismatches corrupt totally, DORD mismatch = exact two-way bit
  reversal; SS raised mid-byte resets the client and un-drives its
  pad; an undrained normal-mode client keeps the LAST byte, buffer
  keeps FIFO-first-two + shifter-last; BUFOVF needs transmit data
  (28.5.5) so a receive-only buffered client cannot see its losses; a
  missed load echoes the received byte (shared shifter); WRCOL is
  about the BOUNDARY - a mid-transfer write is ignored (WRCOL, frame
  intact), an in-gap write is accepted (a race first seen as a flaky
  verdict, pinned by having the peer watch its own SCK leave the CPOL
  idle before writing; NB a spin mixing INTFLAGS reads with DATA
  writes clears IF and loses completions); a REAL demotion by B
  driving shared SS follows the LEVEL (restore_host does not stick
  while the wire is held); MspiHost vs a real SPI client exact in all
  four modes and both orders (SpiMode = {invert_sck, sample_trailing}
  bit for bit) - usart.md's MSPI deferral closed; rebase 24->12->24
  under two-board traffic exact. One driver fix, usart.hpp:
  MspiHost::release now clears the XCK INVEN that invert_sck set (a
  PORT bit the resource teardown cannot know; bench-proven to invert
  the next owner's SCK). serial z 108/108 after the edit; family +
  native green. TWI phase T1 DONE (driver rewrite +
  single-board half, on the office I2C bus: SDA = A.PA2+A.PC2+B.PA2,
  SCL = A.PA3+A.PC3+B.PA3, 1.5k to +5V, VDDIO2 = 5 V; the PORTB tap is
  wired but measured NOT on the node; the PORTE link is GONE, so the
  serial/spi y halves need re-jumpering): twi.hpp REBUILT as Twi<n>
  resource (full ch.29 register description, per-package route table
  with the dual pairs - TWI1 absent on 28-pin, ALT2 absent at 32, dual
  pinless where the header says so; THREE errata as code: DA 2.14.2
  SDAHOLD 50/300 SWAPPED so TwiSdaHold speaks true ns and the encoding
  swaps on DA (family told by MVIO), FLUSH never exposed with recover()
  = ENABLE cycle, OUT=0 before enable; the chapter's baud three-step
  29-3/-4/-5 with rise AND fall as arguments - the bench proved a
  rise-only budget lands tLOW below the spec floor by exactly tOF;
  actual_scl readback; both halves, both ISR bodies) + tasks
  TwiHost<n, route> (the old engine's Request/start/isr/status intact,
  I2cBus/BusMaster unchanged, TwiSpeed {100k,400k,1M} replaces
  I2cSpeed, MBAUD written only with the host disabled per 29.5.7) and
  TwiClient<n, route, on_dual_pins> (NEW: address/general call/mask or
  second address/PMEN, smart mode, S1..S4 verbs, dual via DUALCTRL,
  refused where the dual pair is pinless). NEW SUITE test_avr_twi
  175/175 (a..j, z = all; k..s/y reserved for T2). Findings in twi.md:
  tR 166 ns on this bus at every speed while FMPEN's x10 drive
  collapses tOF from 125 ns to <42 ns; timeouts react at 32/81/183 us
  for the 50/100/200 settings; a STOP injected MID-BYTE is a BUSERR
  and the held START waits, after a clean frame it is legal; a host
  asked to start on a Busy bus holds the START until STOP/timeout/
  force-idle; smart mode reads 4 bytes on 1 MCMD strobe vs 4; a quick
  command is 10 SCL rises = one 9-bit frame + the STOP's own rise
  (writes are 9N+1); six-byte write = 7 host / 8 client interrupts,
  idle bus = zero; combined mode is order-independent (the later init
  owns the shared CTRLA). Family TUs twi.cpp + 5 negatives (TWI1 on
  28-pin, TWI1 ALT2 at 32 - ALT1 exists on da48, the header beat the
  plan - dual on pinless pairs, FM+ without FMPEN, host not in the
  DynamicClock users). serial z 108 and spi z 148 re-verified. TWI
  phase T2 DONE (two-board half): twi_peer on B commanded IN-BAND over
  TWI itself (src/apps/twi_link.hpp, magic 0xC3, command address 0x6B
  - an I2C client is inherently dark, a write carries the command and
  a read serves the response; bounded actions, autonomous restore);
  test_avr_twi grew k..s, y = 174/174, z = 175/175 WITH the peer
  attached. Findings in twi.md: commanded clock stretching lands
  within model + the peer's 5..9 us/byte polled turnaround (even
  unstretched, a polled client paces the bus: SCL low 208 ticks vs
  the register's 120); address/data NACK injected at a commanded byte
  report nack_addr/nack_data with MSTATUS 0x72 and the client-side
  RXACK is only valid SAMPLED AT THE CLOSING NACK (a later copy
  carries the previous tenure's); multi-master arbitration made
  DETERMINISTIC - both hosts hold a START into a Busy bus, the
  injected STOP releases them on one hardware edge and the lower
  address byte wins: loser MSTATUS 0x4B (WIF+ARBLOST, Busy, nothing
  landed), winner 0x62, retry-after-idle works, 20+20 losses over the
  T sweep with zero exceptions; COLL/S4 both ways with two clients on
  one address (wire = AND, only the loser raises COLL, cleared by the
  next Start); NEW resource verb Twi<n>::unstick() - the classic
  nine-clocks-and-a-STOP, open-drain by hand, returns the pulse count
  (0 healthy / 4 when the peer released on the 4th falling edge /
  unstick_failed against a line nothing releases; recover() fixes the
  PERIPHERAL, unstick() the WIRE, the noticing POLICY stays not built
  per i2c-bus.md, which also now names the multi-host policy gap);
  with SDA held low the state machine reads Busy (a falling SDA under
  high SCL IS a Start); general call reaches both clients in one
  tenure and SADDR[0]=0 makes the peer deaf to it while its exact
  address still answers; the three speeds with B attached measure the
  SAME min SCL periods as solo T1 (244/82/34 ticks - the second tap
  adds no measurable rise), the cost is latency (longest SCL low 70/
  126/142 us on read tenures); rebase 24->12->24 exact under two-board
  traffic. End state: A = test_avr_twi, B = twi_peer; serial/spi y
  need the PORTE jumpers AND their peer firmware back. delay/platform
  sweep DONE, CLOSING PHASE 2 of the low-level review (only TCD's own
  track remains untouched): NEW avrdx/reset.hpp (RSTCTRL + WDT - the
  campaign proved panic.hpp asked apps to cross-check the reset cause
  with no verb in the stratum; Reset::take_flags/software, Watchdog
  arm/sync/lock with the two bench-proven WDT facts: arm()'s config is
  NOT in force until sync() - the CTRLA write crosses into the
  1.024 kHz domain - and the FIRST WDR after enabling window mode only
  ACTIVATES the window, 22.3.3.2); AvrPlatform gained sleep_armed()/
  interrupts_enabled() readbacks; delay.hpp UNTOUCHED - nothing
  falsified. NEW SUITE test_avr_platform 96/96 (a..i, z; test i spans
  FOUR real resets - WDT timeout, WDT window violation, SW reset from
  a panic reporter, plain SW reset - with a .noinit phase token, and
  bench.py's judge survives the reboots). The DynamicClock delay was
  then REDESIGNED (same session): a dynamic clock's rate is one of a
  discrete set the TYPE knows (boot rate over the twelve prescalers),
  so DynamicClock exposes rate_count/rate_hz(i)/rate_index() (the
  discrete-rate surface, now in design/clock.md) and delay_us
  dispatches on the index into per-rate branches folded at compile
  time - no division ever runs at wait time, and each branch's Q4.12
  loops-per-us factor (delay_mult, rounded up at compile time) makes
  runtime-us waits exact at every rate incl. sub-MHz. Measured after
  the redesign: folded delay_us = 0 cycles overhead (exact at every
  length), static-rate runtime us +122 constant, DynamicClock +157
  runtime us (was +693; suite caps at 200 to bar the division's
  return) and +6 with a constant us, the old 1.5 MHz 4/3 ceil
  overshoot GONE (slope exact 1001 us; cycles_per_us keeps the
  whole-cycle rounding for the stored-byte pattern only),
  delay_cycles +39 (+10 crossing the 0xFFFF chunk), CriticalSection
  3 cycles (8 nested), IDLE wake = 6 cycles exactly per 13.3.3.2 with
  the sleep proven (loop counter frozen 32 ticks), Ring 50000 elements
  at 20 kHz ISR-producer clean with the overrun holding exactly
  capacity oldest-first, timebase +8843 ppm (OSC32K, matches RTC).
  Chapter corrections: SLPCTRL is ch. 13 / RSTCTRL 14 / WDT 22 (9/11/
  12 is tinyAVR numbering); errata 2.2.4 verified verbatim and it has
  a SECOND half (a following store below 0x40 is lost too - all built
  images emit none, dodged by construction) and NO DA twin (the DA
  errata doc predates the item; the NOP is deliberate on both);
  SLPCTRL.CTRLA is NOT under CCP, SWRR and WDT.CTRLA/LOCK are; SRAM
  survival is promised NOWHERE (table 14-1 lists no SRAM domain) so
  the breadcrumb's magic word is necessary, not prudent; RSTFR
  ACCUMULATES history and take_flags() is the read-and-clear boot
  verb. New doc avrdx/platform.md (PROVISIONAL: standby/power-down +
  VREGCTRL belong to a future sleep pass, break_here's with-OCD half,
  three unreachable reset flags). TCD DONE (the last never-built
  driver - the low-level review has now covered EVERY chapter the
  framework claims): avrdx/tcd.hpp NEW - Tcd<0> resource with the
  three sync disciplines first-class (ENABLE under ENRDY, CTRLE
  strobes/AUPDATE under CMDRDY, static registers only disabled),
  FAULTCTRL under CCP, capture read-L-then-H (the H read releases the
  buffer - a fresh sequence needs one discarded read, it returns a
  STALE value), the input-mode-vs-wgmode validity table as compile-
  time refusals, blanking XOR PROGEV, route table header-gated
  (DEFAULT PA4-7; ALT1 PB4/5 on 48-pin with WOC/WOD pinless, PB4-7 on
  64; ALT2 PF0-3, PF0/1 only on 28-pin - the 32-pin headers bond all
  four; ALT3 PG4-7 64-pin) + task TcdPwm (complementary pair with
  dead time; other usage types wait for Multislope). NEW SUITE
  test_avr_tcd 250/250 x5, wireless. Findings: DUAL SLOPE IS
  2x(CMPBCLR+1), one tick longer than the chapter's printed formula
  (WOB = 2x(CMPBCLR-CMPBSET+1) likewise); PWMACTA/B watch the
  WAVEFORM GENERATOR, not the pad (set even while a fault holds the
  pins still); pin-event capture offset = +3 counter ticks on both
  captures so the PWM-capture example is exact; ASYNC override
  latency is -3 CLK_PER vs +14 sync (= the chapter's 2-3 SYNC
  cycles); all cycle formulas and the prescaler products exact;
  dither DITHER=8 -> 32 cycles sum +16 exactly, dead-time placement
  adds 0 and shortens the neighbour per table 25-7; TCD on OSCHF is
  IMMUNE to a CLK_PER rebase, on CLK_PER it follows. THE PLL GAP
  CLOSED: multipliers measured x1000 = 2000/2999/2001 (OSCHF 16x2,
  16x3, 24x2 - 48 MHz on the counter), PLLS sets only with TCD
  requesting (errata 2.5.3 observed), Pll::start -> bool REFUSES the
  XOSCHF-crystal source (2.5.4; DA has no twin, gated); errata 2.14.2
  POSITIVELY observed (ALT2 + CMPBEN alone = 0 edges, + CMPAEN = 402);
  2.14.1 and 2.14.3 NOT REPRODUCED after systematic sweeps (refusals
  and comments stand, tcd.md records the negatives honestly).
  Suite-glue lesson: an unbound vector is a RESET LOOP (jmp 0), not a
  crash - the suite binds all four TCB vectors as a net. Family 15
  TUs x 8 + 58 negatives (8 new); baselines re-run twi 175, platform
  96, clock 15 (Pll surface grew: source_ok/multiplier/pll_output_hz).
  End state: A = test_avr_tcd, B = twi_peer. -> CCL -> AC ->
  ADC/DAC/VREF -> CLKCTRL (DA must compile) -> EVSYS tables. Phase 2,
  never-reviewed: USART
  (jumper cross-loopback, new suite test_avr_serial) -> SPI (host ->
  client on SPI0/SPI1, 4 jumpers) -> TWI (host -> client TWI0/TWI1) ->
  delay/platform sweep. Jumper tests
  always check bench.md collisions first.
  USART/SPI/TWI re-planned (2026-08-20) as the MULTI-BOARD protocol
  campaign: more boards arrive 2026-08-22; board A = DUT suite,
  board B = scriptable instrument peer (clock stretching, NACK
  injection, multi-master arbitration, bus recovery, auto-baud
  against a foreign sender); two-port orchestrator script; desk work
  (chapter reviews, client-mode driver surfaces, family TUs,
  single-board halves) can precede the boards. Full plan in memory
  low-level-review-plan.
- **Target strata, positions taken (overview.md "Target strata" and
  the diagram docs/design/architecture.svg).** Tasks over resources
  (thin handles + task types named for what they do; explicit handle
  stratum on the second task per peripheral); PwmChannel concept +
  generic actuators (done); Clock as a type (done, both regimes);
  interrupts condense / DMA inside the engines (not on AVR); event
  system = typed vocabulary + runtime connect/disconnect primitives +
  optional static allocation (not built, tables on demand); per-family
  device tables and per-board claim files on the second target.
  Nothing of this is a HAL.
- **Clock: dynamic regime - built and bench-verified (24->12->2 MHz
  under the running console).**
  `DynamicClock<Boot, Users...>` (avrdx/clock.hpp): set<hz>()/set(hz)
  (the app speaks Hz, the prescaler is resolved by div_for) fan
  rebase(hz) out to the users, then switch; Uart/Twi/Spi have
  rebase(); the users list is constrained by the ClockUser concept
  (has rebase) and a driver init(clock)'d with a DynamicClock that does
  not list it is a compile error (clock_follows) - both directions
  checked; delay_us takes the runtime path; `clock_hz(clock)` reads
  either kind. Coordination with bus AOs (switch only when idle) is
  the caller's job - a power-manager AO with request/reply is the
  designed shape, not built. `clock_console` is the bench test
  (CLOCK 4M / 2M / 24M at 115200). F_CPU is not defined in this build
  (see below).
- **Design rule for all AVR work: think the other targets first.**
  Before adding/changing anything in avrdx/, ask what shape it takes
  on Cortex-M0+ (rich clock tree, hardware cycle counters, per-pin
  AF), RISC-V (mcycle, group remap): that decides what crosses the
  concept boundary. Written in overview.md "Target strata".
- **C++ modules: considered, not now.** The real prize would be macro
  isolation (`import brio.avrdx` would not leak `avr/io.h` macros
  above the target stratum - the layering rule made mechanical), not
  build speed. Blocker: the language server (clangd follows modules
  only partially) - CMake itself has grown C++20 module dependency
  scanning since the SCons-based build (untested here, and clangd's own
  gap is unrelated to the build system). Not a cure for the ISR glue either: the
  vector bindings are configuration (which USART, which route, which
  sink) and would live in a per-board unit under modules exactly as
  under headers. Revisit with the second target / board files.
- **Housekeeping.** No LICENSE file yet (repo is public at
  github.com/uliano/brio).

## Build, test, debug (the must-knows)

```bash
# Three sibling CMake projects, PEERS (none is the repo root): avrdx/,
# samc/, test/. cmake presets resolve against their own project dir -
# run cmake FROM that dir (or let tools/bench.py do it).
(cd test  && ctest --preset host)                                  # host tests (doctest); no hardware needed
tools/check_family.sh [name]                                       # every avrdx smoke TU compiles for all 8 DA/DB
                                                                    # packages; neg/ TUs must FAIL (definition of done)
tools/check_samc.sh [name]                                         # same for the samc stratum (E/G/J 18A headers)
(cd avrdx && cmake --build --preset avr128db48-release --target <app>)         # AVR release build (-Os)
(cd avrdx && cmake --build --preset avr128db48-release --target <app>-upload)  # flash via Atmel-ICE (UPDI)
(cd avrdx && cmake --build --preset avr128db48-debug --target <app>)           # AVR debug build, then F5
(cd samc  && cmake --build --preset samc21j-release --target <app>)            # SAM release build
(cd samc  && cmake --build --preset samc21j-release --target <app>-upload)     # flash via OpenOCD (SWD)
# apps are auto-discovered from <project>/src/apps/*.cpp at every
# configure - no generation step; a new/removed app or a changed
# "// build: opt = value" line takes effect on the next configure

python3 tools/bench.py list                  # serial devices, USB probes, the bench manifest
python3 tools/bench.py flash A test_avr_pin  # cmake --build --target <app>, then avrdude/UPDI
python3 tools/bench.py flash C test_samc_dma # ... or OpenOCD/SWD - the BOARD TYPE decides both
                                             # the project to build in and the flash mechanism
python3 tools/bench.py run C z               # drive the console, judge "ALL: N pass, M fail"
python3 tools/bench.py console A             # device path + speed, for your own monitor
python3 tools/bench.py duo A:a B:script.txt  # instrument peer scripted, then the DUT
python3 tools/bench.py fuses A bootsize=128  # read/write fuses over UPDI (fuses are
                                             # provisioning: UPDI-only, survive reflash)
```

- The multi-board bench, three separate concerns (detail:
  `docs/bench.md`): BUILD = one CMake target per app x board TYPE
  (`// build: boards = db28,db32,db48` in the app header; `db48` is the
  default when the line is absent; a configure targets exactly one
  package, so switching `configurePreset` switches which apps' targets
  exist), IDENTITY = the manifest `tools/bench_boards.py` (which board
  sits where, its console by `/dev/serial/by-path` because the CH340s
  have no USB serial, its programmer), ORCHESTRATION = `tools/bench.py`.
  Never a target per physical board. `family_probe` carries the matrix
  and is the first firmware for a new board.

- Toolchains: self-built avr-gcc 16.2 at `/sw/avr`
  (`avrdx/cmake/toolchain-avr.cmake`) and arm-none-eabi-gcc 16.2 at
  `/sw/arm-none-eabi` (`samc/cmake/toolchain-arm.cmake`), each pointed
  at by absolute path; never a system-packaged one. Never add
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
  `docs/host/README.md`, `docs/bench.md`.

## Layout

```
avrdx/                   the AVR build project (a PEER of samc/ and test/ -
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
                           for tools/bench.py, e.g. "monitor_speed = 115200")
  src/glue/                build invariants compiled into EVERY image (every
                           avr_add_app() call lists ivsel_boot.cpp alongside
                           the app's own source - the .init3 IVSEL store,
                           vectors at BOOT start)
  svd/avr128db48.svd       the debug Peripheral Viewer's register map
samc/                    the SAM C21 build project, same shape (CMakeLists +
                         presets + cmake/toolchain-arm.cmake + ld/ linker
                         script + src/apps + src/glue startup crt + svd/) -
                         its own header comments are the reference
test/CMakeLists.txt      the host test project (independent - one CMake
                         configure has exactly one compiler):
                         one executable + ctest entry per test_*/main.cpp
test/CMakePresets.json   the "host" configure/build/test preset (native g++, UBSan)
test/test_*/main.cpp     host unit tests (doctest), cd test && ctest --preset host
test/family_samc/        samc family smoke TUs + neg/, tools/check_samc.sh runs them
third_party/doctest/     vendored doctest.h (MIT, upstream doctest/doctest)
third_party/samc21-dfp/  vendored Microchip.SAMC21_DFP include tree (Apache-2.0)
third_party/cmsis-core/  vendored ARM CMSIS-Core headers (Apache-2.0)
tools/check_family.sh    family compile check over test/family/ (see above) -
                         zero CMake coupling, calls avr-g++ directly
tools/check_samc.sh      the samc twin over test/family_samc/
tools/bench_boards.py    the bench MANIFEST: the physical boards on the desk
                         (type, console by-path, programmer) - not a target list
tools/bench.py           the bench orchestrator: list / flash / run / console /
                         duo / fuses, over the manifest and the per-project app
                         rosters build-cmake/apps_{avrdx,samc}.json (each project
                         writes its own at every configure - separate files
                         because app NAMES COLLIDE across the trees: blink,
                         console and probe exist in both). BOARD_TYPES maps a
                         board type to its project, preset, mcu and flash
                         mechanism (db* -> avrdx/avrdude/UPDI, c21j ->
                         samc/OpenOCD/SWD); `fuses` and --erase are AVR-only
                         and refuse a SAM board instead of pretending
docs/                    README (map + rules), design/, <target>/ (avrdx/, samc/,
                         host/), bench.md
brio/.clangd             per-stratum clangd routing: the framework default is
                         the host database; avrdx/.clangd and samc/.clangd
                         (in brio/ AND in each project dir) override with
                         their own architecture's database, so a header always
                         parses with its own compiler regardless of CMake
                         Tools' active project
brio/                    the framework, four strata:
  kernel/                pure kernel logic - includes NOTHING of brio
    platform.hpp           Platform concept (CriticalSection, idle,
                           break_here, now, ticks_per_second, atomic_width,
                           panic_record) + PanicRecord (hosted by the platform)
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
                           deadline, the power model's one kernel question
    kernel.hpp             Pack<Aos...> (index, lends_ok) + Kernel<P, Aos...>:
                           init_all/step/idle_if_empty/run, static_asserts
                           borrowers before lenders
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
    persistent_panic.hpp   PersistentPanic<S>: panic Reporter into an
                           NvStore + boot-side take()
    ring.hpp               Ring<T, size, P> SPSC FIFO, lock-free when the
                           index fits P::atomic_width, guarded otherwise
    testbench.hpp          TestBench<Sink, max_letters>: the bench suite
                           grammar in one place - letter registry, verdict
                           lines, per-letter tally and the ALL: total
                           tools/bench.py parses
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
    platform_avr.hpp       AvrPlatform (idle() sleeps in IDLE unless a
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
                           + tasks TwiHost<n, route> (the I2C transfer engine
                           driven by util/i2c_bus.hpp) and TwiClient<n, route,
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
  samc/                  everything that knows sam.h (SAM C21, Cortex-M0+)
    nvic.hpp               InterruptGuard (PRIMASK) + Nvic (enable/pend/priority)
    platform_sam.hpp       SamPlatform (idle takes whatever PM.SLEEPCFG holds -
                           SCR.SLEEPDEEP is never written - with erratum
                           1.8.13's guard around a standby WFI; BKPT, .noinit
                           breadcrumb, atomic_width 4)
    ticker.hpp             BasicTicker<tps> over SysTick (Ticker = 1000 Hz) +
                           SysTickInterruptGuard, erratum 1.8.13's workaround
                           in the file that owns the register. THE TICK STOPS
                           IN STANDBY: SysTick rides the CPU clock
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
    nvm_flash.hpp          RwweeFlash: the FlashMedia backend over the RWWEE
                           array - writing it does not stall the CPU, and the
                           zone is a constant because no linker section reaches
                           there
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
                           deep maps to standby because nothing is deeper
  host/                  the test target
    platform_host.hpp      HostPlatform (virtual clock, recording idle/break)
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
per-project app rosters `apps_avrdx.json` / `apps_samc.json` in
`build-cmake/` itself.
