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
  pair, survival-aware mount).
- `docs/<target>/` - one folder per target, mirroring
  `lib/brio/src/<target>/` (`avrdx/`, `host/`): `README.md` is the
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

`brio` (`lib/brio/`) is a header-only C++23 (gnu++23) framework for
bare-metal MCUs built around a cooperative active-object kernel,
written clean-room after Samek's book (never the QP source). One flat
namespace `brio`; four strata under `lib/brio/src/` - `kernel/` (pure
logic, includes nothing of brio), `util/` (services over the kernel),
`avrdx/` (everything that knows `avr/io.h`, the only target today:
AVR DA/DB, bench chip AVR128DB48), `host/` (the native test target).
Includes carry the stratum prefix (`#include "avrdx/usart.hpp"`). The
repo is a PlatformIO project: one `main()` per `src/apps/<app>.cpp`
becomes envs `<app>` (release) and `<app>-debug`; host tests in
`test/` run on `[env:native]`.

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
  -Os -c -I lib/brio/src` takes seconds, no hardware; (3) negative
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
a template/concept boundary can do the job; no target includes outside
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
- **Second target.** Candidates STM32G0x0/x1, ATSAMC/D, CH32V00x
  (SysTick timebases at 1000 Hz). It will exercise the Platform
  concept, the per-target ticker/driver strata and the layering rule
  for real; expect radical revision of util/ and drivers then. The
  build tooling is part of this milestone: PlatformIO is already
  stretched past its design use case; the keep-or-replace analysis is
  scheduled with the second target (CMake the only named candidate so
  far). The EDITOR half moved early
  (2026-08-25, forced): cpptools' clang-based parser (1.33+) is
  structurally unable to parse the AVR-configured libstdc++ (x86-64
  model + gcc-only types __int24/_Float32), so the editor is clangd
  over compile_commands.json already - `pio run -e <app> -t compiledb`,
  --query-driver for include paths and target, the pio_flags.py -mmcu
  define-delta feeding the device macros clang lacks, test/.clangd for
  the host tests (detail in docs/avrdx/README.md). The delta feed is
  an AVR peculiarity (device-specs macros; other targets select the
  device with an explicit -D) and goes away with the second target.
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
  build speed. Blockers: the language server (PlatformIO's IntelliSense
  does not follow modules; clangd only partially) and SCons has no
  module dependency scanning. Not a cure for the ISR glue either: the
  vector bindings are configuration (which USART, which route, which
  sink) and would live in a per-board unit under modules exactly as
  under headers. Revisit with the second target / board files.
- **Housekeeping.** No LICENSE file yet (repo is public at
  github.com/uliano/brio).

## Build, test, debug (the must-knows)

```bash
pio test -e native            # host tests (doctest); no hardware needed
tools/check_family.sh [name]  # every avrdx smoke TU compiles for all 8 DA/DB
                              # packages; neg/ TUs must FAIL (definition of done)
pio run -e <app>              # release build (-Os)
pio run -e <app> -t compiledb # refresh compile_commands.json (clangd, the
                              # editor engine; any app env, they share flags)
pio run -e <app> -t upload    # flash via Atmel-ICE (UPDI)
pio debug -e <app>-debug      # ALWAYS debug the -debug env
python tools/gen_apps.py      # after adding/removing src/apps/*.cpp (or a "// pio: opt = value" line)

PY=~/.platformio/penv/bin/python         # pyserial + pio live in PlatformIO's venv
$PY tools/bench.py list                  # serial devices, USB probes, the bench manifest
$PY tools/bench.py flash A test_avr_pin  # build the app's env + avrdude over UPDI
$PY tools/bench.py run A a               # drive the console, judge "ALL: N pass, M fail"
$PY tools/bench.py console A             # device path + speed, for your own monitor
$PY tools/bench.py duo A:a B:script.txt  # instrument peer scripted, then the DUT
$PY tools/bench.py fuses A bootsize=128  # read/write fuses over UPDI (fuses are
                                         # provisioning: UPDI-only, survive reflash)
```

- The multi-board bench, three separate concerns (detail:
  `docs/bench.md`): BUILD = one env per app x board TYPE
  (`// pio: boards = db28,db32,db48` in the app header -> also
  `[env:<app>-db28]`; `db48` is the bare `[env:<app>]`), IDENTITY = the
  manifest `tools/bench_boards.py` (which board sits where, its console
  by `/dev/serial/by-path` because the CH340s have no USB serial, its
  programmer), ORCHESTRATION = `tools/bench.py`. Never an env per
  physical board. `family_probe` carries the matrix and is the first
  firmware for a new board.

- Toolchain: self-built avr-gcc 16.2 at `/sw/avr` via `symlink://`;
  never PlatformIO's bundled one. Never add `-mrelax` (PyAvrOCD
  refuses the ELF). `build_unflags` must keep `-std=gnu++11` (the
  platform appends it after our `-std=gnu++23`), `-flto`, and
  `-DF_CPU` (by name: the clock rate has one truth, `Clock::hz`;
  avr-libc's util/delay.h / setbaud.h do not compile here, on purpose -
  use `brio::delay_us(clock, us)`).
- Atmel-ICE: cable in the **AVR** port, not SAM (symptom: Vtarget
  ~1.71 V, sign-on `0xa0`).
- Debugging: PyAvrOCD as GDB server (`debug_tool = custom`, port
  40044); effectively ONE free hardware breakpoint (GDB borrows the
  second); `--breakpoints hardware` refuses extras instead of wearing
  flash; line breakpoints need `-fno-inline` (GCC 16 DWARF caveat) -
  hence the debug flags in platformio.ini. `monitor ioregister <name>`
  reads/writes peripheral registers.
- The bench board: 24 MHz crystal on PA0/PA1 (not GPIO) -
  `Clock<ClockSource::crystal, 24'000'000>` - no 32k crystal (do not
  enable XOSC32K), serial on USART2 ALT1 PF4/PF5.
- Full detail and rationale: `docs/avrdx/README.md`,
  `docs/host/README.md`, `docs/bench.md`.

## Layout

```
platformio.ini          base [env], toolchain, Atmel-ICE upload, debug wiring
apps.ini                generated: [env:<app>] + [env:<app>-debug] per app
boards/AVR128DB{28,32,48}.json  custom bare-metal boards (128K flash / 16K RAM)
tools/check_family.sh    family compile check over test/family/ (see above)
tools/gen_apps.py        scans src/apps/*.cpp -> apps.ini; copies each app's
                         "// pio: <option> = <value>" header lines into its envs;
                         "// pio: boards = db28,..." adds the board-type envs
tools/bench_boards.py    the bench MANIFEST: the physical boards on the desk
                         (type, console by-path, programmer) - not an env list
tools/bench.py           the bench orchestrator: list / flash / run / console /
                         duo, over the manifest and the generated envs
tools/pio_flags.py       per-language AVR flags (build-type aware) + the
                         -mmcu macro delta as -Ds (feeds clangd; skips
                         [env:native])
tools/gen_lst.py         post-build: firmware.lst (disassembly) + firmware.map
src/apps/<app>.cpp       one main() per app (ISR vector bindings live HERE)
src/glue/                build invariants compiled into EVERY image via
                         [common] base_src_filter (today: ivsel_boot.cpp,
                         the .init3 IVSEL store - vectors at BOOT start)
test/test_*/main.cpp     host unit tests (doctest), pio test -e native
docs/                    README (map + rules), design/, <target>/ (avrdx/, host/), bench.md
lib/brio/src/            the framework, four strata:
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
                           (drift-free periodics, wrap-safe, RAII disarm)
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
    clock.hpp              ClockUser concept, clock_hz(clock), clock_follows:
                           the target-independent clock contracts
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
    bus_master.hpp         BusMaster<Bus, P>: bus arbiter (pending FIFO,
                           reject-when-full, ReplyTo completion, BusDone)
    spi_bus.hpp            SPI vocabulary: SpiBus/SpiDone/spi_*
    i2c_bus.hpp            I2C vocabulary: I2cBus/I2cDone/i2c_* + outcomes
    proto/line_parser.hpp  LineAssembler + console/SCPI parsers +
                           CommandRouter<Sink>
  avrdx/                 everything that knows avr/io.h (AVR DA/DB)
    platform_avr.hpp       AvrPlatform
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
                           defsym (pio_flags.py)
    sleep.hpp              SLPCTRL: Sleep (arm/disarm/sleep/enter, the three
                           modes, the errata-2.2.4 NOP discipline) and Vreg
                           (PMODE under CCP, HTLLEN with the TWI/CCL
                           interlock enforced) - mechanism only, policy is a
                           future power-manager AO's
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
  host/                  the test target
    platform_host.hpp      HostPlatform (virtual clock, recording idle/break)
    sim_flash.hpp          SimFlash: FlashMedia over RAM for the host tests
                           (configurable geometry, power-cut injection,
                           simulated reflash, wear counters)
```

## Build artifacts

`.pio/build/<env>/`: ELF / HEX / MAP / `firmware.lst` (source-
interleaved disassembly from tools/gen_lst.py).
