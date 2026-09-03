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
  pair, survival-aware mount), `nv-journal.md` (the small-value store
  over the SAME contract: two halves that ping-pong wholesale, seq
  decides and CRC judges, read-only mount, the panic reserve, and the
  ruling that NvRecord and NvJournal stay two spellings), `power.md`
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
namespace `brio`; seven strata under `brio/` - `kernel/` (pure
logic, includes nothing of brio), `util/` (services over the kernel),
`armv6m/` (the CORE stratum both Cortex-M0+ families include after
their device header: NVIC + PRIMASK guard, the SysTick ticker),
`avrdx/` (everything that knows `avr/io.h`: AVR DA/DB, bench chip
AVR128DB48), `samc/` (everything that knows `sam.h`: SAM C21,
Cortex-M0+, bench chip ATSAMC21J18A), `stm32g0/` (everything that
knows `stm32g0xx.h`: STM32G0, Cortex-M0+, bench chip STM32G0B1RE on
a Nucleo-64), `host/` (the native test
target). Includes carry the stratum prefix
(`#include "avrdx/usart.hpp"`). The builds are four sibling CMake
projects, PEERS - the repo root is not a CMake project: `avrdx/`,
`samc/` and `stm32g0/` (each with its own toolchain file and presets,
Ninja, emitting into the shared `build-cmake/`) auto-discover one
`main()` per `src/apps/<app>.cpp` at configure time from its own
`// build:` header comment; host tests in `test/` are the fourth
project (host g++, no cross toolchain), run via `ctest`. ONE NAME PER ARCHITECTURE,
the same key on three axes: `brio/<arch>/` (stratum),
`docs/<arch>/` (docs), `<arch>/` (build project); chip precision
lives in preset names, per-chip ld/svd files and the `*_MCU` cache
variables. Names are claims, extended only when a real chip extends
the family - the known landing names, never used early: avrdx ->
avrxt (Microchip's sigla for the modern-AVR core) when an EA/mega0
part proves it shares the stratum; samc -> sam0 if a D21 arrives; stm32g0 shares its name with
the G0x0 value line if a chip ever proves it; an `armv6m/` core
stratum factored at the SECOND ARM family - DONE 2026-09-02 the day
the stm32g0 arrived: brio/armv6m/{nvic,ticker}.hpp, the two families'
nvic/ticker headers reduced to device-include + core-include + their
own extras, every one of the 42 samc + stm32g0 images byte-identical
before and after (worktree gate with pinned mtimes).

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
- **THIRD TARGET: STM32G0 - BRING-UP DONE 2026-09-02 (phases 0-3 of
  memory stm32g0-bringup-plan, BY FABLE'S OWN HAND in one session,
  from the plan to a console answering on the third architecture).**
  PHASE 4 DONE the same evening: the armv6m/ factoring (above).
  PHASE 5 RUNNING autonomously the same night (the user away, their
  instruction: everything the single board allows; Opus delegations
  per chapter, Fable's brief, re-verification by hand and commit; the
  campaign order and status live in memory stm32g0-peripheral-plan):
  campaign 1 reset/IWDG/WWDG/breadcrumb/delay DONE (test_stm32_platform
  z 53/53 + i 26/26 over six reboots; THE IWDG IS STOPPED BY THE RESET
  IT CAUSES, its keyed registers do not update until started, LSI
  32536 Hz, the EWI interrupt needs WDGA); campaign 2 FLASH DONE
  (flash.hpp grown to ch. 3 bar the one-way writes, BANK 2 = the storage
  attic with the ld narrowed to bank 1, NvHeap + NvJournal UNCHANGED on
  their third silicon at 2048/8: test_stm32_nvm z 85/85, test_stm32_
  journal z 52/52; EOP does not rise with EOPIE clear, an orphan first
  word wedges CFGBSY until reset, fast row 21 us/dword, RWW 174260 loop
  turns inside an erase); campaign 3 EXTI DONE (test_stm32_exti z
  89/89; the EXTI sees a pad its owner drives, a pending bit exists
  only for an unmasked interrupt, IMR1 resets to implemented &
  ~configurable, PC13 has an EXTERNAL PULL-UP - a press is a falling
  edge); campaign 4 TIM DONE (tim.hpp over all ten timers with eight
  tasks, test_stm32_tim z 105/105; the geometry is a table in the
  reserve because one TIM_TypeDef serves ten timers; centre-aligned
  period = 2 x ARR; PwmChannel + MeterSource on their third silicon);
  campaign 5 DMA+DMAMUX DONE (dma.hpp, test_stm32_dma z 54/54; THE
  BLOCK-STREAM FIXED POINT ANSWERED - BlockPlayer fits the hardware
  circular mode, BlockSource cannot (skip-rather-tear is undecidable
  after the edge), nothing in util/ changed; a request is a LEVEL served
  on enable, the SAM's opposite; the VCP ceiling is 921600); campaign 6
  ADC/DAC/COMP/VREFBUF DONE (four drivers, test_stm32_analog z 94/94,
  util/analog + analog_sampler unchanged; VDDA 3310 mV via VREFINT,
  conversion time exact to the cycle, the DAC reaches the ADC only
  through PA4, VREFBUF sits behind SYSCFG's clock gate, pulls are
  disabled in analog mode so the comparator's stimulus is a precharged
  pad; Ref/ref_mv live in vref.hpp - one shared VREF+ rail); campaign 7
  RTC+PWR DONE (rtc.hpp with RCC_BDCR, pwr.hpp, sleep.hpp with the two
  sites; test_stm32_rtc z 77/77, test_stm32_sleep z 50/50 + Standby/
  Shutdown reboot letters; the ladder none/light -> Sleep, standby ->
  Stop 0, deep -> Stop 1, Standby/Shutdown OFF it because the program
  does not resume; the LSE crystal is fitted and runs; util/power.hpp
  unchanged on its third silicon). THE WIRELESS SINGLE-BOARD ROSTER IS
  CLOSED; what remains is the fillers (CRC, LPTIM, IRTIM, the USART
  tail) and what needs a peer or wires (the buses against a SAM at
  3.3 V, FDCAN, USB, UCPD, the option-byte bench verb).
  brio/stm32g0/ NEW: device_tables.hpp (THE RESERVE from day one -
  GPIO ports, USART instances, their APB enables, their CCIPR
  multiplexers and their SHARED VECTORS, the last read off the device
  select macro because IRQn values are enumerators the preprocessor
  cannot probe), nvic.hpp + ticker.hpp (the samc files' twins line for
  line - the armv6m/ factoring candidates, deliberately NOT factored
  yet), platform_stm32.hpp (`Stm32Platform`: WFI = Sleep mode,
  SLEEPDEEP never written), flash.hpp (FlashWaitStates with the
  read-back rule and Range-1 table 13, FlashAccel with PRFTEN left at
  reset because of erratum 2.2.10), clock.hpp (THE THIRD CLOCK MODEL:
  one SYSCLK, shared HPRE/PPRE pinned at 1 with `pclk_hz` stated
  beside `hz`, per-peripheral enable bits with the readback stall,
  CCIPR multiplexers; `Clock<internal, hz>` = HSI16/HSIDIV and
  `Clock<pll, hz>` = HSI16 x PLL with the exact ratio searched at
  compile time - 64 MHz is M1/N8/R2), pin.hpp (a port has a CLOCK and
  it is off at reset, so every configuring verb opens it; analog is
  the reset state; AF numbers are the DATASHEET's with no header
  table to check them against; BSRR/BRR atomic values), usart.hpp
  (`Usart<n>` + `Uart<n, pins>` with the SAME public surface as the
  other two targets' Uart; non-FIFO view; the integer baud divisor;
  ORE cleared through ICR or the handler storms). stm32g0/ build
  project NEW (the samc shape; `STM32G0_MCU` = full part number ->
  device define STM32G0B1xx + ld/<part>.ld + startup_<header>.cpp;
  the crt's handler NAMES come from ST's startup template because the
  header declares none; OpenOCD stlink.cfg + stm32g0x.cfg + the DHCSR
  clear). third_party/cmsis-device-g0/ (v1.4.5, Apache-2.0) + the SVD
  vendored. Apps probe/blink/console; test/family_stm32g0/ (4 TUs x
  g0b1/g071/g031 + 8 negatives) + tools/check_stm32g0.sh green;
  bench.py grew the g0b1re board type and the openocd_stlink probe
  kind (the SAM argv proven unchanged), manifest position E by-id.
  THE VICTORY CONDITION HELD FOR THE THIRD TIME: kernel/ and util/
  untouched - blink under time events (PA5 sampled over SWD at both
  cadences), console over the ST-LINK VCP (300 lines byte-exact, all
  counters zero, baud 115107 = 64e6/556, tick +0.24 % vs the PC).
  Findings: PWR is behind an APB enable and its range register read
  the right reset value THROUGH THE CLOSED GATE once (luck, not a
  contract - init opens it); SWD reads through the ST-LINK's HLA
  transport are UNRELIABLE while the core sleeps in WFI (halt first);
  bench.py `run` speaks the suites' letter grammar, not the console's
  lines. Docs: docs/stm32g0/{README,platform,clock,port,usart}.md all
  PROVISIONAL with honest gap lists, vendor/README.md (RM0444 Rev 6,
  DS13560 Rev 5, ES0548 Rev 3 - revision Z, the errata pass). NEXT:
  phase 4 = the armv6m/ factoring pass (its own session), then phase 5
  = the campaign plan (FLASH -> NvJournal on its pre-validated
  geometry, EXTI, DMA+DMAMUX as block-stream's second implementation,
  TIM, ADC/DAC, PWR, the buses against a SAM peer at 3.3 V, RTC,
  watchdogs, FDCAN). Session memory: stm32g0-session-2026-09-02.
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
  transmit-heavy work with DMA + bulk. The duplex bug this probe found
  is DIAGNOSED AND FIXED - see the UART campaign entry below.
  **UART TRANSPORT CAMPAIGN DONE 2026-08-29 (Opus delegation, REVIEWED
  BY FABLE and COMMITTED f6fdd8c): the duplex wedge diagnosed, fixed and fenced, and the
  transport matrix given a suite.** THE MECHANISM, in one paragraph:
  a DMA-fed SERCOM direction stops dead when its channel is armed
  while the peripheral's own request LEVEL is already high - a trigger
  is latched on the RISE (25.6.2 / 25.8.8), so the channel sits enabled
  with CHSTATUS empty, the peripheral's DRE (or RXC) standing, and not
  one beat moving; on top of that, erratum 1.10.4 is not a bad READING
  but a destroyed TRANSFER, because 25.6.2.6 makes the write-back the
  controller's LIVE descriptor for an ongoing block, so a corrupted one
  leaves the channel running someone else's transfer for ever. Either
  way DmaTxEngine::busy() stayed true, pump_tx() returned at its first
  line every time, the ring filled, and print() spun in Ring::push with
  the board silent - the exact wedge serial_speed reported. CAUGHT IN
  THE ACT three times over the halt-and-dump and the suite's own
  snapshot: the transmit channel enabled, DRE and TXC both set, and its
  write-back holding the RX channel's descriptor (BTCTRL 0x809, SRCADDR
  = the SERCOM's DATA register, where its own says 0x409 and a RAM
  address). THE FIX, three pieces, each named to its measurement:
  dmac.hpp's engines gained kick() (one software trigger, safe by
  construction - a channel has one pending bit and SWTRIGCTRL raises it
  only if clear, so a kick racing a real trigger is LOST, never
  doubled), DmaTxEngine::abandon()/faults() (throw away a block the
  silicon has stopped running; what it loses is stated, not pretended
  away) and a harvest that answers "nothing started" instead of
  suspending a channel whose write-back is still the zeros reset() put
  there - because 25.6.2.8's second clause sets FERR when a RESUME
  fetches a next descriptor with a null DESCADDR, which every
  single-block descriptor here has, and the first version of the
  recovery livelocked on exactly that. sercom.hpp asks the SERCOM
  whether its flag is already standing and kicks; makes a REFUSED byte
  still nudge (write_byte's false is the state in which nothing is
  draining, and print() answers it by trying for ever); re-arms the
  receiver when the SILICON says the channel is not running rather than
  when the engine's beat count says so; and exposes dma_faults().
  THREE GUESSED FIXES WERE REFUTED BY THE DATA before the right one
  stood: that the completed write-back's addresses are rewritten (they
  are not - the case that looked like it was two different blocks), that
  consistent() was giving false positives (it was not - the refusals
  were the symptom of a dead channel), and that a repeated-refusal
  give-up would recover it (it made things deterministically worse).
  THE ISOLATION that settled it: with only ONE engine and one unrelated
  churning channel the same death appears, so "the two engines interact"
  was never the point - two concurrently triggered channels are enough,
  and test_samc_dma letter j passes because nothing is being RECEIVED
  during it, so its RX channel takes no triggers at all. NEW SUITE
  test_samc_uart (z 27/27; eleven host letters, 62 verdicts, green twice
  over) + NEW TOOL tools/uart_stress.py. Measured there: all four
  transports carry 11840 bytes byte-exact except at the RX engine's
  block boundaries (11813..11827, and WHERE it loses is its contract -
  a filled block has no run to continue into until a harvest re-arms
  it); 8E1/8O1/8N2/7E1/7N2 all byte-exact; 3 Mbaud echo loses in the
  HARDWARE and the loss is accounted for, never silent; and A FRAME
  MISMATCH IS ASYMMETRIC - a host at 8E1 into an 8N1 receiver raises
  198 framing errors in 1210 characters, while a host at 8N1 into an
  8E1 receiver raises NOTHING (2432 characters, both counters zero: the
  receiver reads the sender's stop bit as parity and finds the idle line
  where its stop bit belongs). COST, measured: +4 bytes on every
  engineless console image (the refusal path's one branch), +292 on
  serial_speed and +600 on test_samc_dma, blink and probe unchanged.
  The headline throughput is UNTAXED - raw 3 Mbaud 299251 B/s, DMA+bulk
  297890 B/s at 9% CPU, 1 Mbaud DMA+bulk 99902 B/s at 5% - identical to
  the recorded figures. Canaries: test_samc_dma z 112/112 (the erratum
  still caught and refused, 280 in 153654), check_samc OK, check_family
  OK. JUDGMENT CALLS QUEUED: see memory samc-session-2026-08-29-uart.
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
  A SOFTWARE EVENT ON AN ASYNCHRONOUS CHANNEL DOES NOT REACH THE DMAC.
  29.6.2.12 says a software event "can be serviced as any event
  generator" with no mention of the path; measured, EIGHT of them on an
  async channel move nothing while ONE on a clocked path moves a block.
  (The first reading - that the async PATH drops what has no width -
  was CORRECTED by the CCL campaign on 2026-08-29: the limit is the
  USER's input stage. A CCL LUT's edge-detecting event input catches
  16/16 software events on the same async channel; the DMAC's trigger
  stage is what a register write has no width for. evsys.md carries
  the reconciliation.)
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
  (a claim the CCL campaign later sharpened: the async path carries a
  software event too - it is the DMAC's own trigger stage that needs
  width, evsys.md);
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
  THE BOARD FINALLY HAS A SCALE. REVIEWED BY FABLE next session and COMMITTED (3d7d2e6), all eight judgment calls accepted - the review's canary re-run caught and fixed the osc32k 1% verdict the rescaling had doomed (details: memory samc-session-2026-08-28-clocks); the rest of Fable's
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
  costs NOTHING measurable over a polled wait while IDLE2 costs more -
  ORIGINALLY 3.5..4.4 us, CORRECTED 2026-08-29 to 24..30 us (the
  original was measured through tc.hpp's one-behind READSYNC defect,
  whose deep_leg differences telescoped onto the arming time; the
  double-READSYNC fix and the mechanism are in tc.md) - on a board with
  no CAN traffic, which chapter 19 does not mention; leaving STANDBY
  costs ~106 us (originally 16.6..17.8 through the same defect), and
  SIX combinations of VREGSMOD x SUPC.VREG.RUNSTDBY x BBIASHS still
  spread under 2 us inside the repeat's own scatter, so THIS FAMILY HAS
  NO SEPARATE REGULATOR BILL holds with the corrected absolute (the
  AVR's was a distinct 290 us item); a 499 ms standby advanced the kernel tick by 0 ms and a time
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
  **ADC DONE 2026-08-28 (ch. 38, BOTH converters) - PHASE G's first
  chapter, and util/analog.hpp + analog_sampler.hpp VALIDATED UNCHANGED
  on their second architecture. REVIEWED BY FABLE same day and COMMITTED (a79e039), all seven judgment calls accepted; two false claims caught at review - the build-id determinism holds, and VREFOE-vs-REFSEL-INTREF was an open gap later CLOSED by the DAC campaign.** samc/adc.hpp NEW over
  the whole chapter: `Adc<n>` for two instances that are the same
  peripheral at two addresses but NOT a symmetric pair (the device
  header's ADCn_MASTER_SLAVE_MODE gives ADC0 the host role and DUALSEL,
  ADC1 the client role and SLAVEEN, and a knob on the wrong instance is
  a compile error), both input muxes with every internal channel, the
  six reference codes, the prescaler and sampling arithmetic with table
  45-22's cycle counts as constexpr, all four resolutions, the
  accumulation/averaging/oversampling unit, free-running, the window
  monitor, the offset and gain corrections, the sequencer, both event
  directions, the DMA trigger and the one vector - with THE FOUR
  REGISTER DISCIPLINES SPELLED PER REGISTER (enable-protected,
  write-synchronized, DOUBLE-BUFFERED as well, and neither), because
  the mixture is where this chapter's traps are: a double-buffered
  write made mid-conversion holds its SYNCBUSY for the whole
  conversion, which is why select() is a void store with no wait and
  select_sync() is a separate verb. THE PAD MAPS WENT INTO THE RESERVE
  and needed TWO of them keyed by INSTANCE, because they OVERLAP: PA08
  is ADC0/AIN8 and ADC1/AIN10 at once, PB08 is ADC0/AIN2 and ADC1/AIN4
  - and the E bonds NO PORT B pad to either converter, leaving ADC1
  there with AIN10 and AIN11 and nothing else (device_tables.hpp grew
  eleven ADC probes; the growth is proven a pure addition - 18 of 19
  pre-existing SAM images byte-identical, the nineteenth being
  test_samc_nvm, whose newest-source-mtime defsym changes by design).
  THE RESSEL/AVGCTRL TRAP IS REFUSED: accumulation above one sample
  requires RESSEL = 16BIT, which three separate Notes say and nothing
  enforces, and the full-scale arithmetic that falls out of tables 38-1
  and 38-2 IS util/analog.hpp's `steps` argument, computed at compile
  time by adc_result_steps() and at run time by result_steps().
  CALIBRATION IS NOT OPTIONAL and init() copies it, keeping the promise
  nvm.hpp's comment has carried since phase B1. This target's
  `brio::Ref` + `ref_mv()` live in adc.hpp rather than in a vref header,
  because on this family there is no shared reference block - ADC, DAC
  and SDADC each have their own REFSEL vocabulary. Errata: FIVE of the
  ten are live at rev F and 1.4.4 IS CODE (start_on()/flush_on() refuse
  a non-asynchronous channel, since a synchronized event during a
  conversion stalls the whole channel), 1.4.5 is why start() waits for
  nothing, 1.4.6 is the five conversions init() discards, 1.4.9 is a
  stated caveat, 1.4.10 is an obligation - and it DID NOT REPRODUCE;
  1.4.1/1.4.2/1.4.3 are rev B and 1.4.7/1.4.8 are rev B..E (the
  read-the-row trap again). NEW SUITE test_samc_adc z 97/97 four times,
  9 letters, wireless, ~25 s. THE FINDINGS: the BANDGAP CHANNEL IS DEAD
  WITHOUT SUPC.VREF.VREFOE - MUXPOS INTREF reads a FLAT ZERO until that
  bit is set and 795 counts of 4096 after, a connection between chapters
  22 and 38 that neither states; VDD from the ADC's side is 5276/5233/
  5201 mV at the three bandgap levels where the COMPARATOR independently
  said 5251/5141/5090 (two peripherals, no shared mechanism, agreeing to
  under 2 % and sloping the same way with the level, which puts the
  slope on the bandgap and not on either instrument); ADC0 and ADC1 read
  a shared pad EXACTLY alike at both rails and differ by 5..6 counts of
  4096 on an internal divider; CONVERSION TIME IS EXACT TO THE
  STOPWATCH TICK in six configurations ruled by the crystal (13/33/10/
  16/13/208 CLK_ADC cycles predicted, every one measured to the tick, so
  table 45-22's four rows are right and the accumulation multiplies
  exactly); THE WINDOW MONITOR'S MODE4 IS THE COMPLEMENT OF MODE3, which
  settles the device header against 38.8.10's own table (with WINLT
  below WINUT the table's reading is an empty band that can never fire
  and the header's must fire at both rails - it fires at both rails);
  and 38.6.2.14's PER-CONVERSION 13-CYCLE correction latency IS NOT
  OBSERVED (117 ticks uncorrected against 119 corrected, at either
  offset, where thirteen cycles would be 104) although the correction is
  demonstrably in the path, an OFFSETCORR of 100 taking exactly 100
  counts off - so adc_conversion_cycles() keeps charging the 13 in
  single mode deliberately, a pacing prediction being safe when
  generous. The no-CPU chain runs BOTH WAYS AT ONCE (a TC overflow
  starting the conversion over an asynchronous channel, the DMAC taking
  RESULT one beat per conversion, a second TC counting the result-ready
  events - and nothing moving at all with the pacer stopped), and it
  caught that THE DMA REQUEST IS THE RESRDY FLAG: 38.6.4's "cleared when
  the RESULT register is read" means clearing the flag is not enough and
  a stale result moves a stale beat. Letter i is the campaign's point:
  AnalogSampler INSIDE A REAL KERNEL walking an internal channel and a
  rail-driven pad, 49 interrupts, 49 events received, zero mislabelled,
  with NOT ONE LINE of util/ changed - the file's own comment doubted
  the shape would survive a target with a sequencer and DMA, and the
  answer is that it does because the sampler uses neither. The averaging
  letter MEASURES THE NOISE BEFORE CLAIMING ANYTHING ABOUT IT (three
  internal sources spanning 1, 4 and 5 counts over 64 readings; the
  noisiest is the measurand and the reduction verdict is declined in
  print if even that one spans under four counts). Family fixture
  test/family_samc/adc.cpp + NINE negatives; suite regressions sleep z
  87, tc z 77, supc z 44; check_samc, check_family, host 22/22. Docs:
  adc.md new PROVISIONAL, with the honest gap list - the host/client
  pair, the sequencer, sleep, differential mode, VREFA and everything
  needing a voltage that is not a rail, which the DAC campaign owns.
  **DAC DONE 2026-08-28 (ch. 41) - PHASE G's SECOND CHAPTER AND THE
  SESSION THAT CLOSES THE ANALOG LOOP. REVIEWED BY FABLE same day and COMMITTED (f2d550f): JC1 ruled no-code-change (init()'s false IS the truth where 1.4.10 kills ADC0), and the review hunted down the suite's one flaky verdict (letter h's one-count knife edge, rewritten).** samc/dac.hpp NEW:
  the whole small chapter as a MONOSTATE `Dac` (one instance on every
  variant - the Rtc precedent, against Adc<n>'s two), with its OWN
  reference enum `DacRef` (adc.hpp keeps `Ref` for its own REFSEL - the
  ADC campaign's accepted judgment call 1 applied), table 41-1's four
  data placements as `dac_data_word()` arithmetic, both outputs, the
  START/EMPTY event pair and the DMA trigger published as codes, and the
  refusals that are the chapter's rules (a Reserved REFSEL; DITHERING
  WITHOUT A START EVENT, since 41.6.8.3 makes the sixteen sub-conversions
  the event's job). THE GEOMETRIC GIFT: PA02 is DAC/VOUT, ADC0/AIN0 and
  the AC's AIN4 AT ONCE, so erratum 1.8.9's "wire the DAC VOUT pin
  externally to an ADC AINx pin input" is a wire of ZERO LENGTH and the
  suite is wireless - which is also what let three drivers' unvalidated
  enumerators finally be measured. NEW SUITE test_samc_dac z 108/108
  (three cold runs from a fresh flash, six warm), 11 letters. MEASURED:
  the device header's `INT1V` name for REFSEL 0 is the SAM D21's and
  WRONG here - the reference follows SUPC.VREF.SEL (996/2011/4057 mV at
  its three levels); NEITHER the DAC's nor the ADC's reference path needs
  SUPC.VREF.VREFOE, which closes the open gap Fable's review of the ADC
  campaign left (the ADC's bandgap INPUT channel is dead without it, the
  REFERENCE path reads 2991 vs 2990 counts with the bit clear and set);
  `Ref::dac` converts and is ratiometric to half a per cent, putting
  1/4 VDDANA at 1286 mV against 1287; `AdcInput::dac` reads the DAC to
  one count in 4096 though 38.8.9's table marks the code Reserved, and
  41.6.8.1's "the output buffer must be enabled" is NOT true on this
  silicon; ERRATUM 1.8.9's OUTPUT half is real and large AND ISOLATED BY
  A CONTROL (the pad's spread goes 3 -> 71..87 counts while another
  converter samples MUXPOS = DAC, and back to 3 with that converter
  free-running on any other input) while its READING half is DECLINED as
  beneath this board's noise floor; `AcNegative::dac` flips a comparator
  at 255/512/769 against 255/511/767 predicted with gaps of EXACTLY 257
  and 257 (differencing cancels the comparator's offset); the transfer
  curve is monotonic with a worst residual of 2.5 ADC counts from the
  best-fit line, reported as the two converters' COMBINED nonlinearity
  and DELIBERATELY NOT APPORTIONED (and the same measurement declines the
  1023-vs-1024 scaling question, whose 0.1 % is smaller than the pair's
  gain error); startup ENABLE-to-READY 5.9 us; a full-scale step crosses
  mid-supply FASTER THAN A COMPARATOR POLL CAN RESOLVE (an upper bound of
  a few hundred ns against the 2857 ns the 350 ksps rate implies - a rate
  is not a settling time), measured as a 1000-crossing DIFFERENCE because
  a single shot costs 4 us in synchronized stopwatch reads alone; the
  no-CPU waveform runs (timer event -> START, DMAC -> DATABUF on EMPTY, a
  second timer counting EMPTY) with UNDERRUN when it runs dry and nothing
  moving when the pacer stops; and TWO DOCUMENTARY DISPUTES SETTLED BY
  EXPERIMENT - EVCTRL IS enable-protected (41.6.2.1's list is right,
  41.8.3's property line is missing a property) and DBGCTRL IS AT OFFSET
  0x14 (the device header's, not the register summary's 0x18).
  THE FINDING THAT CHANGED THE DRIVER: SYNCBUSY.DATABUF IS NOT A BUS
  CROSSING - it stands until a START EVENT CONSUMES the value, SYNCBUSY.
  DATA stands with it, and every later write to either register is
  discarded (41.6.7), so one DATABUF write in a DAC with no start event
  wedges the data path until a start event or a reset. `buffer()` is
  therefore a plain void store that never waits and `buffer_sync()` the
  separate verb - exactly the shape adc.hpp's `select()` has over its
  double-buffered INPUTCTRL, and neither chapter says so. Erratum 1.9.2
  REPRODUCES with a control on both sides (half a second awake leaves
  EMPTY clear, the same half second in standby sets it, and RUNSTDBY = 1
  leaves it clear). AND A CROSS-CAMPAIGN CORRECTION: ERRATUM 1.4.10 IS
  LIVE AND WORSE THAN ITS OWN SENTENCE - the ADC campaign's narrow probe
  saw nothing, but once ADC1 has been enabled in a power cycle
  ADC0.SYNCBUSY.ENABLE is stuck at one, ADC0 WILL NOT ENABLE AT ALL
  (Adc<0>::init() returns false and the converter reads zero) and a
  software reset does not clear it; the errata's own order (ADC1 up
  first, ADC0 second) is the way out and the suite spends it visibly.
  adc.hpp's and ac.hpp's comments were corrected from these measurements
  (comment-only, the established precedent); their docs lost exactly the
  gap lines this closed. Family fixture test/family_samc/dac.cpp + FIVE
  negatives; docs/samc/dac.md new PROVISIONAL (gaps: dithering, VREFA,
  the voltage pump, sleep, LEFTADJ on silicon, the interrupts through the
  NVIC, the SDADC's share). THREE JUDGMENT CALLS QUEUED - see memory
  samc-session-2026-08-28-dac.
  **SDADC DONE 2026-08-28 (ch. 39) - PHASE G's THIRD CHAPTER, THE
  CONVERTER THE MULTISLOPE WORK WILL LEAN ON. REVIEWED BY FABLE same day and COMMITTED (see memory samc-session-2026-08-28-sdadc for the rulings).**
  samc/sdadc.hpp NEW: the whole chapter as a MONOSTATE `Sdadc` (the Rtc
  and Dac precedent) with its OWN `SdadcRef` - four codes and NONE
  Reserved, the only reference field of the three converters with no
  illegal value. A 16-bit sigma-delta over THREE DIFFERENTIAL PAD PAIRS
  (the most package-dependent map in the stratum: the E bonds pair 0
  alone, the G adds pair 1, only the J carries pair 2) behind a
  THIRD-ORDER SINC DECIMATION FILTER, and almost everything that
  surprises follows from the filter rather than from a converter.
  THE FIRST DESIGN POSITION IS THE BUS ERROR: 39.6.8 says a synchronized
  write made while its busy bit stands is "discarded AND A BUS ERROR IS
  GENERATED" where the ADC's and the DAC's chapters promise a silent
  discard - a bus error on this core is a HardFault - so EVERY
  synchronized write in this file WAITS BEFORE STORING and reports, which
  is why `select()` is a bool where adc.hpp's is a void store.
  THE CENTRAL MEASUREMENT, and it changed the driver: THE DATAPATH IS
  TWENTY-FOUR BITS WIDE and 39.8.19's "left-adjusted" is NOT eight bits
  of padding - RESULT saturates at 0x7FFFFF/0x800000 and not at a shifted
  +/-2^15; the corrections of 39.6.3.4 act in RAW 24-BIT UNITS (an
  OFFSETCORR of 25600 moves the reading by EXACTLY 100 counts of the
  specified datum, and GAINCORR 2/2^0, 4/2^1, 3/2^1, 1/2^1 all land
  within tens of raw units of the formula); and the eight bits under the
  specified datum are REAL FILTER OUTPUT - they move where the datum is
  bit-exact. So the driver has three result verbs (result / result24 /
  result_raw) and `sdadc_corrected()` works in raw units.
  MEASURED: THE PRESCALER IS LINEAR, 2 x (P + 1) - PRESCALER 3/4/7/23
  give period ratios 1000/1249/2000/5999 against the datasheet's
  1000/1250/2000/6000 and the DEVICE HEADER'S SAM D21 power-of-two
  enumerators' 1000/2000/16000, which are simply wrong here; the
  free-running period is OSR x 4 CLK_SDADC cycles EXACT (the uniform
  +0.49 % being OSC48M's 5100 ppm against the crystal that timed it);
  SKPCNT costs a whole decimation window each, so table 45-26's
  single-conversion row DIVIDES where it should multiply, and 39.6.2.3's
  "the first valid sample is the third" is LITERAL (a full-scale
  differential reads 5478 / 27623 / 32767 at SKPCNT 0 / 1 / 2 - the SINC
  step response caught in the act, and the reason the driver REFUSES a
  single conversion skipping fewer than two); CTRLC and ANACTRL are NOT
  enable-protected though 39.6.2.1 lists them, while REFCTRL is (and has
  NO SYNCBUSY bit though 39.6.8's prose lists it); REFCTRL.REFRANGE is a
  REAL FIELD CHAPTER 39 NEVER MENTIONS; ANACTRL's bias field is six bits
  as the header says and not five as 39.8.21 draws.
  THE NOISE NUMBERS THE MULTISLOPE WANTS: on a shorted differential the
  SPECIFIED SIXTEEN BITS ARE ALL NOISE-FREE from OSR 128 up (64
  consecutive conversions bit-identical), so the noise is only visible in
  the raw byte below - 14/16/17/18/18 noise-free bits of 24 over the OSR
  ladder, rms 224 -> 55 -> 30 -> 19 -> 15, an improvement per octave that
  FLATTENS onto a thermal floor rather than a quantization one; at the
  1.024 V bandgap the rms is 17 uV, five times quieter than table 45-27's
  0.08 mVrms. THE OFFSET IS COMMON-MODE DRIVEN (-13.9 mV with both pads
  at GND, +17.0 at VDD, a 44 dB common-mode rejection, and 0.6 mV at
  mid-supply) and the chopper takes a third off it.
  THE LINEARITY SWEEP HAD TO BE INVENTED, because the DAC cannot reach an
  SDADC input at all (it is a REFERENCE here and nothing else): TCC1's
  WO0 and WO1 ARE PA06 and PA07, i.e. this pair's own two pads, so two
  PWM waveforms give a swept differential at a fixed mid-supply common
  mode with the converter's own SINC as the reconstruction filter - and
  at OSR 1024 the decimation window is EXACTLY 64 PWM periods, so the
  fundamental and every harmonic land on a filter zero. Fifteen points:
  monotonic throughout, best-fit slope 65.132 counts per duty step
  against 64.000 ideal (a gain of +1.7 %, inside table 45-27's +/-3.4 %
  max), intercept 4 counts, and a WORST RESIDUAL OF 4 COUNTS OF 32768
  where 45-27 allows an INL of +/-11 LSB - reported as the COMBINED
  nonlinearity of a duty ratio and a decimation filter and not
  apportioned. THE +/-0.7 x VREF LIMIT DID NOT BITE at 0.875 of VREF
  (10 counts of deviation, not a collapse), consistent with erratum
  1.18.2 being marked revisions B..E and not F.
  THE CROSS-CHECK between the two architectures is a RATIO and not a
  voltage, there being no voltage both can see: the SDADC's reference
  multiplexer puts 4.096 V / VDDANA at 791 per mille and the SAR's input
  multiplexer at 788 - THREE PARTS IN A THOUSAND, sharing nothing but the
  bandgap. Also: the same input against four references agrees to 0.8 %,
  the bandgap's levels come out 1987 and 1996 per mille against 2000
  nominal, and REFSEL = VREFB with PA04 driven to the supply reads 4171
  where VDDANA reads 4172 - one count.
  ERRATA: exactly ONE applies at rev F and it REPRODUCES WITH A CONTROL -
  1.8.10 (the DAC as this converter's reference) makes the DAC's pad
  spread go 1 -> 108 counts with ONREFBUF clear, 1 with it set, and 1
  with the same converter running against VDDANA; the driver REQUIRES the
  buffer for both internal references (39.8.2's own Note), so the bit was
  cleared by hand to see it. 1.8.7's SleepWalking obligation on SWTRIG is
  stated. 1.18.1, 1.18.2, 1.18.3 and 1.18.4 are all B or B..E - and THIS
  REVISION'S PRINTED RESET VALUES (GAINCORR 1, SKPCNT 2) ARE 1.18.3'S OWN
  WORKAROUND baked into the silicon.
  NEW SUITE test_samc_sdadc z 101/101 (twice warm, once cold), 11
  letters, WIRELESS; family fixture test/family_samc/sdadc.cpp + TEN
  negatives; canaries dac z 108 and adc z 97; md5 gate on the reserve's
  growth: ALL 21 pre-existing SAM images BYTE-IDENTICAL. docs/samc/
  sdadc.md NEW PROVISIONAL, dac.md's 1.8.10 and SDADC gap lines closed.
  All seven judgment calls RULED AND ACCEPTED at Fable's review (memory samc-session-2026-08-28-sdadc).
  **TSENS DONE 2026-08-28 (ch. 43) - PHASE G's FOURTH AND LAST CHAPTER,
  and the one that is NOT a converter. REVIEWED BY FABLE and COMMITTED
  (b65fa51), all seven judgment calls accepted (memory
  samc-session-2026-08-28-tsens).**
  samc/tsens.hpp NEW: the whole chapter as a MONOSTATE `Tsens` (the Rtc /
  Dac / Sdadc precedent - one instance on every C21 variant, and NO PADS
  AT ALL, 43.5.1 being "Not applicable"). THE DESIGN POSITION IS THAT
  THIS IS A CLOCK RATIO AND NOT AN ADC CHANNEL: a temperature-dependent
  oscillator is counted against GCLK_TSENS, so THE GENERIC CLOCK IS THE
  MEASUREMENT'S RULER and the factory GAIN/OFFSET belong to one
  particular rate (43.6.1's note: OSC48M undivided). Under that clock
  VALUE IS CENTI-DEGREES CELSIUS - 43.8.10's own example, 2500 = 25 C -
  which is the unit the header speaks throughout with no float anywhere;
  under any other rate the two escapes are `tsens_gain_for()` on the way
  in and `tsens_rescale()` on the way out, BOTH TAKING THE RATE AS A
  CALLER ARGUMENT (the freqm reference_hz pattern - a ratio meter cannot
  know what its own reference is worth). `TsensCalibration::factory()`
  reads GAIN/OFFSET/TCAL/FCAL out of samc/nvm.hpp's
  NvmTemperatureCalibration, KEEPING THE PROMISE that file's comment has
  carried since phase B1 (the adc.hpp load_calibration precedent). Every
  synchronized write WAITS BEFORE STORING and returns bool - 43.6.7
  threatens a BUS ERROR in the same words 39.6.8 does, the sdadc position
  taken again. THE CENTRAL MEASUREMENT, and one no other chapter in this
  stratum could offer a bench with no thermometer: THE SAME DIE READ ON
  TWO REFERENCES. FREQM weighs OSC48M against the board's crystal in the
  same letter (47759811 Hz, 5003 ppm SLOW), a crystal-locked DPLL
  supplies a true 48 MHz, and an INTERLEAVED A-B-B-A comparison repeated
  four times with a MEDIAN estimator (the test_samc_rtc FREQCORR
  technique - a linear drift of the die's own temperature cancels exactly
  out of four equally spaced batches) measures a median of -35 centi-C
  against -37 PREDICTED, with a cycle-to-cycle spread of 7 where a single
  reading's own spread is 42. AND THE ERROR RIDES ON THE SPAN FROM THE
  OFFSET, not on the temperature: 5000 ppm of a 26 C reading would be
  0.13 centi-C, and it is 35. The 1/f law itself is confirmed structurally
  (24 MHz with the factory GAIN reads -4667 against -4673 predicted) and
  both escapes land within 5 centi-C. OTHER FINDINGS: GAIN'S RESET VALUE
  IS 2^24 AND NOT ZERO - the field is 24 bits, so an uncalibrated TSENS
  does not report a benign zero but waits 699 ms (2 x 2^24 periods at
  48 MHz, TO THE MILLISECOND) and hands back the gain term amplified two
  hundredfold, looking like -16000 C, which is why config_valid() refuses
  it; a measurement costs 2 x GAIN + 2020 GCLK periods, a CONVERSION TIME
  CH. 43 STATES NOWHERE, the constant identical across a fourfold GAIN
  range once the reference's error is taken out; the VALUE rail is at
  -2^23 located to 250 counts in eight million (convicting 43.6.4's "more
  than 16 bits" against 43.8.7's "more than 24") and AN OVERFLOWED VALUE
  WRAPS RATHER THAN SATURATING - a plausible number of the WRONG SIGN,
  the opposite of what sdadc.hpp's converter does; WINMODE OUTSIDE is the
  COMPLEMENT of INSIDE, settling the device header against 43.8.3's
  printed "WINUT < VALUE < WINLT"; CAL.TCAL/FCAL are worth 10.08 C; the
  gain term is NEGATIVE at room temperature (the OFFSET sits above the
  reading), which the chapter never says. THE EVENT FINDING: table 29-3
  grants THIS user - user 0 - all three propagation paths where the DAC's
  and the SDADC's are asynchronous-only, and all three move DMA blocks -
  BUT A SAMPLED PATH SAMPLES, and with the pacer's rate held at 1 kHz and
  only the pulse width and the channel clock changed, a 21 ns event
  reaches an asynchronous channel clocked at 32 kHz while a synchronous
  one keeps it only at the GENERATOR'S OWN RATE (24 MHz against 48 is no
  better than 32 kHz), and widening the pulse does not save a slow
  synchronous channel either. ERRATUM 1.19.1 REPRODUCED AND WORSE THAN
  ITS OWN SENTENCE: with PAC write protection set for the TSENS a CTRLB
  write starts nothing AND raises no PAC interrupt flag, so it is dropped
  in COMPLETE SILENCE - and it does NOT fault this core, so 11.5.2.4's
  "access error" is not a bus error here. NEW SUITE test_samc_tsens z
  168/168 (three warm, two cold), 10 letters plus `p` outside z (8/8),
  WIRELESS; family fixture test/family_samc/tsens.cpp + FIVE negatives;
  canary sdadc z 101; md5 gate on the reserve's growth: ALL 22
  pre-existing SAM images BYTE-IDENTICAL. Two traps this suite paid for
  and that belong to other drivers: TC2 AND TC3 SHARE GCLK CHANNEL 31, so
  Tc<2>::release() silently stops TC3; and bench.py's --expect="->" can
  truncate a capture, because "  -> " ends with the prompt "> ".
  docs/samc/tsens.md NEW PROVISIONAL. JUDGMENT CALLS QUEUED - see memory
  samc-session-2026-08-28-tsens.
  **CCL DONE 2026-08-29 (ch. 37) - PHASE H's FIRST CHAPTER, and the one
  that CLOSES AC.MD'S OPEN LEAD. COMMITTED 931211f (see memory
  samc-session-2026-08-29-ccl).**
  samc/ccl.hpp NEW: the whole chapter in the AVR's own two strata,
  because for once the two families really do have the same peripheral -
  `Ccl` (one ENABLE, one software reset, ONE generic clock for every
  filter/edge/sequencer in the block, the two sequencer selectors, the
  EVSYS codes it publishes) and `Lut<n>` (three input multiplexers, an
  eight-bit TRUTH table built by `lut_truth()` on the AVR's own IN[0]-is-
  the-LSB convention, the synchronizer/filter, the edge detector, both
  event enables), plus `CclIn<Pin>`/`CclOut<Pin>` over six new reserve
  probes. NO INTERRUPT AND NO DMA AT ALL here (37.5.4 and 37.5.5 are both
  "Not applicable"), so the only ways out are a pad and an event. THE
  PACKAGE FACT: LUT3's four pads are the J's ALONE, so on the E and the G
  that LUT exists (CCL_LUT_NUM is 4 everywhere) with NO PIN OF ITS OWN,
  reachable only through events, a link or a sequencer - `if constexpr`
  on `Lut<3>::has_output_pad`, two negatives, and the E bonds no PORT B
  pad to the CCL at all. THE ENABLE-PROTECTION STORY IS THE DESIGN, and
  it was settled RAW against three documents that disagree (37.6.2.1 says
  LUTCTRLn.ENABLE, 37.8.2 says CTRL.ENABLE, erratum 1.7.3 says the
  silicon swapped them): four cells of a truth table say IT IS AN AND -
  a LUTCTRL write lands only with BOTH bits clear - so every configuring
  verb refuses while the block is up, and reconfiguring one LUT drops
  every other LUT's output (bench-proven, the AVR errata-2.4.1 protocol
  reached by another road). THE BENCH CAUGHT A REAL DRIVER BUG THERE: a
  store into an ENABLED LUT is dropped IN COMPLETE SILENCE, so
  configure() and truth() now clear LUTCTRLn.ENABLE in a store of their
  own first (37.6.2.1 forbids writing the protected bits together with
  ENABLE = 0), and 37.6.2.1's one-store escape is separately proven. THE
  HEADLINE, and the answer ac-sync-latency's open lead asked for, on
  ac_sync_probe's own instrument (GCLK_AC and GCLK_CCL both at
  OSC48M/4096 = 11.719 kHz, one period = 4096 CPU cycles, 52 shots per
  row): a COMBINATIONAL LUT costs 0.05 periods (207 cycles, eight above
  the comparator's own ASYNC pad - 40.8.13's note is about COMPCTRL.OUT
  and not about a pad, so a LUT dodges the AC's sampler entirely), a LUT
  PAIR AS A DFF costs the fraction to the next edge and NO WHOLE PERIOD,
  ONE LUT with FILTSEL=SYNCH costs the fraction + 1, THE AC'S OWN
  SYNCHRONIZED OUTPUT - the fraction + 2 - IS THE SLOWEST CLOCKED PATH OF
  THE FOUR, and FILTSEL=FILTER costs the fraction + 3 (so 37.6.2.5's "two
  to five GCLK cycles" is a range over the two OPTIONS and each option's
  own cost is exact). The application the AC killed does not revive as
  "no CCL at all" - the +2 is silicon - but the CCL is strictly cheaper
  than it, which is the answer with numbers. TWO MORE DOCUMENTARY
  DISPUTES SETTLED BY EXPERIMENT: INSEL 0x8 (TCC) HAS NO ENUMERATOR IN
  ANY DEVICE HEADER of this pack though 37.8.3 lists it for every variant
  - written as a literal it works, with TCC0's own pad as the control -
  and ERRATUM 1.8.3 IS REVISION B (TC0 drives the default TC input and
  TC4 does not, two controls). ERRATA: 1.7.1 confirmed revision B (the RS
  latch's reset works), 1.7.2 REPRODUCED WITH A CONTROL ON BOTH SIDES and
  its workaround coded (Lut<n>::enable(true) on an EVEN LUT re-states
  CTRL.ENABLE), 1.7.3 as above, 1.7.4 CONFIRMED - CTRL.SWRST really does
  raise PAC INTFLAGC bit 23, which is the OPPOSITE of what TSENS's 1.19.1
  did (that one was silent). Also measured: the edge strobe is one
  GCLK_CCL period to half a per cent (4086..4108 cycles against 4096);
  37.5.3 is exact (a combinational LUT decodes with GCLK_CCL
  DISCONNECTED, a synchronized one passes nothing); tables 37-2..37-5 all
  four, with the lesson that THE GATE MUST COME DOWN FIRST when both
  stimuli are pull-driven pads; FEEDBACK proven by a difference; and A
  SOFTWARE EVENT *DOES* CROSS AN ASYNCHRONOUS CHANNEL - sixteen of
  sixteen into a LUT, with a disconnected-user control and a DMA block
  moved through the LUT as a second witness - which CORRECTS what
  test_samc_evsys concluded from the DMAC alone: the limit is the USER's
  input stage, not the path, so evsys.hpp's comment and evsys.md's
  finding were rewritten (the one comment-only change to another driver).
  NEW SUITE test_samc_ccl z 141/141 (three warm, one cold; about two
  seconds), 7 letters, WIRELESS; family fixture test/family_samc/ccl.cpp
  + TEN negatives; canaries tsens z 168, evsys z 37, ac z 94; md5 gate on
  the reserve's growth with source mtimes PINNED: all 23 pre-existing SAM
  images BYTE-IDENTICAL. docs/samc/ccl.md NEW PROVISIONAL, ac.md's open
  lead CLOSED, evsys.md's software-event finding corrected. JUDGMENT
  CALLS QUEUED - see memory samc-session-2026-08-29-ccl.
  **PHASE H'S TAIL DONE 2026-08-29 - PAC (11), DSU (13), DIVAS (14) and
  MTB (10.3) in one session, WHICH CLOSES EVERY WIRELESS CHAPTER OF THE
  PLAN. REVIEWED BY FABLE and COMMITTED same day, all eight judgment
  calls accepted (memory samc-session-2026-08-29-debug).**
  Four small chapters, four new headers, ONE suite - test_samc_debug,
  because three of the four meet each other: the DSU comes out of reset
  PAC-protected, DIVAS's only error report is a bit in the PAC's AHB flag
  register, and the MTB's PAC identifier is a number only the PAC's
  register map states. samc/pac.hpp NEW and THE POSITION IS MECHANISM
  ONLY - the keyed word-wise WRCTRL, PERID = 32 x bridge + index, the
  four flag banks, the ACCERR event, the shared IRQ 0 - with NO guard
  type, NO policy and NO util concept, for three reasons stated in the
  header: nothing in brio protects anything yet, 11.5.2.6's balance rule
  (a double set or a double clear is itself an error) makes a nestable
  guard a design decision rather than a detail, and erratum 1.13.3 proves
  the guarantee is not uniform. samc/dsu.hpp NEW (DID decoded, the
  hardware CRC32, MBIST, the CoreSight ROM, the two debug channels; chip
  erase DELIBERATELY not exposed, the AVR CHER precedent). samc/divas.hpp
  NEW (both buses, the AHB's wait-state read and the IOBUS's mandatory
  poll). samc/mtb.hpp NEW (the four registers 10.3 names before deferring
  to a TRM this project does not have - so the DEVICE HEADER is the only
  local authority on the layout). THE CAMPAIGN'S HEADLINE is letter b's
  CONTRAST MAP: sixteen peripherals across all three bridges, each
  written back to itself with protection off (the control) and on, and
  ALL SIXTEEN REPORT - which relocates erratum 1.19.1's silence onto
  TSENS.CTRLB rather than TSENS (CTRLA flags in the same run) and leaves
  erratum 1.7.4's flag-with-no-protection on the CCL as the opposite
  pole, so AN ABSENT FLAG IS NOT EVIDENCE AND A PRESENT ONE IS NOT PROOF.
  Also measured: protection IS off out of reset except the DSU, which
  alone comes up protected (STATUSB reset 0x2, table 12-3's one Y); and
  STATUSC COMES UP WITH BIT 25 SET - PERID 89, outside 11.7.12's drawing
  AND outside the header's own PAC_STATUSC_Msk, past ID_PERIPH_MAX 87 -
  which answers WRCTRL like any other protection bit, so this device
  protects something at reset that no document names. Erratum 1.13.2
  CONFIRMED WITH A CONTROL (the same illegal access flags on the MCLK and
  raises nothing on the PORT) and erratum 1.13.3 CONFIRMED with controls
  both sides (the same DIRTGL write lands through the IOBUS unflagged and
  is dropped and flagged through the APB) - and the IOBUS window turns
  out NOT to be a plain mirror: DIR and OUT read through it, IN and CTRL
  read zero. Erratum 1.23.1 names MCLK.CTRLA, a register THIS FAMILY DOES
  NOT IMPLEMENT (ch. 17 opens at INTENCLR, the header reserves offset
  0x00) - and a read of that unimplemented byte DOES raise the illegal-
  access error 11.5.2.4 promises. THE LOCK: 11.5.2.5's "only a hardware
  reset" and table 18-1's silence left it open, and a SYSTEM reset AND a
  watchdog reset each CLEAR IT - a PAC lock lasts until the next reset of
  any kind, and letter c is re-runnable because of it. DSU: the die
  serial and DID match tools/bench_boards.py's record for board C (that
  manifest comment's "recorded, NOT yet checked" now names the letter
  that checks it); the CRC32 is the standard reflected-0xEDB88320 answer
  over flash and SRAM, chains through a raw seed, refuses a bus error,
  and costs 6 cycles per word against 387 for a bitwise software
  reference (55x); LENGTH is written as a BYTE count and is a working
  COUNTER the engine consumes to zero, which 13.14.5 never says; MBIST
  costs 1165 cycles per word and provably destroys its buffer; and the
  CoreSight ROM's two entries resolve to 0xE00FF000 (the M0+'s own table)
  and 0x41008000 - THE MTB, so ch. 13 and 10.3 are one debug system -
  which also settles 13.14.10's two self-contradicting EPRES sentences at
  "1 means present". DIVAS against gcc's own division as reference:
  0xFFFFFFFF/7 costs 332 cycles in software and 19.5 through the engine
  (17x), 683 against 22.6 with the remainder (30x); THE IOBUS IS NOT THE
  FASTER PATH (its mandatory BUSY poll costs more than the AHB's
  wait-state stall); DLZ=0 gives 8.3 cycles for a small dividend against
  20.8 full width and DLZ=1 gives 20.8 for both; a square root costs
  20.8; and 14.5.8's unnamed "error" for an operand write while busy IS
  PAC.INTFLAGAHB.DIVAS and nothing else. ADOPTING DIVAS AS THE
  TOOLCHAIN'S DIVISION IS NAMED OPEN AND NOT TAKEN (re-entrancy, clocking
  and 14.5.2's "will not operate in any sleep mode" make it a
  whole-image decision). MTB: a self-hosted trace with NO PROBE ATTACHED
  - 12 packets from a known chain, all three functions' linked addresses
  in the decoded destinations; bit 0 of the DESTINATION word marks the
  START OF TRACE (one packet of twelve, the first) and the source word's
  is set on none; WRAP and AUTOSTOP both exact; THE DEVICE HEADER'S
  EVENT-USER NUMBERS ARE RIGHT AND TABLE 12-3'S ARE NOT (only user 45
  starts a trace and 46 stops one, 44 does nothing); and neither AUTOHALT
  nor HALTREQ stops a core whose DHCSR.C_DEBUGEN bench.py has cleared.
  SUITE test_samc_debug z 117/117 five times after the last fix (one cold, four warm - the runs before it caught a flaky DLZ verdict in letter h, an ordering comparison between two measurements that are EQUAL), plus
  c 11/11 (two real resets) and k 2/2 outside z; family fixtures
  pac/dsu/divas/mtb + SEVEN negatives; the reserve grew five probes
  (pac/dsu/mtb ids, the bridge count, the two MTB event users). Docs
  pac.md / dsu.md / divas.md / mtb.md NEW PROVISIONAL; the board-C
  comment in tools/bench_boards.py updated (comment only). JUDGMENT CALLS
  QUEUED - see memory samc-session-2026-08-29-debug.
  **ANALOG STREAMING DMA DONE 2026-08-29 (Opus delegation, the user's
  own question "il dma possiamo usarlo anche per dac e adc?", REVIEWED
  BY FABLE and COMMITTED same day, all seven judgment calls accepted
  plus one supervisor fix; memory samc-session-2026-08-29-analog-dma).**
  samc/dmac.hpp GREW: the element type IS the beat (dma_beat_of -
  one sizeof feeds BEATSIZE and the end-address arithmetic so they
  cannot disagree; DmaTx/RxEngine<ch, Elem = uint8_t>, every existing
  spelling unchanged, all 27 pre-existing SAM images BYTE-IDENTICAL -
  the gate re-run by Fable's own worktree comparison) and TWO NEW
  streaming engines, same monostate shape, same kick/abandon/faults/
  harvest hardening: DmaLoopEngine (one caller-owned table played for
  ever, re-armed from TCMPL because THIS CONTROLLER HAS NO HARDWARE
  CIRCULAR MODE - 25.6.3.1 offers only a self-linked descriptor and
  1.10.4 corrupts the write-back that 25.6.2.6 makes the LIVE
  descriptor, so a self-linked chain has no second copy to judge the
  first against) and DmaPingPongEngine (two caller-owned buffers, THE
  ACCOUNTING IS THE API - laps/overruns/stalled/pending; an overrun
  SKIPS the lap rather than write into the buffer the caller holds, so
  everything handed over is untorn, and the count of LOST samples is
  the peripheral's OVERRUN flag, never the engine's). NEITHER STREAMING
  ENGINE KICKS ON ITS RE-ARM - a correctness rule: the beat that ended
  the block SERVED the request, so a kick there overwrites an
  unconsumed value or duplicates a sample; kick() is the OWNER's verb
  for the first arm and the arm after abandon(). NEW SUITE
  test_samc_analog_dma z 72/72 (agent x5, Fable x3 incl. one cold), 10
  letters, WIRELESS - the chain has no CPU in the sample path: TC0
  overflow -> EVSYS -> DAC START while one channel refills DATABUF,
  TC0 CC0 -> EVSYS -> ADC0 START on the SHARED PA02 PAD while another
  drains RESULT, 5 kHz. THE HEADLINE: a SOFTWARE-CLOSED LOOP LOSES
  NOTHING AT ITS SEAMS - table 32 and block 24 chosen not to divide
  each other, the entry each block starts on steps by exactly 24 (mod
  32) through every block of every run, worst residual 3..6 counts
  where the ADC's own noise is 4..6 and a table step is 120. FINDINGS
  NO CHAPTER CARRIES: INTFLAG.EMPTY IS AN EVENT, NOT A STATE (a
  freshly enabled DAC with an empty DATABUF reads EMPTY = 0, so a
  stream that waits for the flag never starts - the owner's knowledge
  that it just reset the converter is what makes the first kick
  right); kick() ONLY WHERE THE STANDING REQUEST BELONGS TO THE STREAM
  (the ADC's is an erratum-1.4.6 warm-up conversion and must be
  DRAINED - kicked, it lands in slot zero and shifts the whole
  capture); SELECTING TRIGSRC IS ITSELF AN EDGE (the mux output rises
  from the DISABLE code's zero, and a rise during a disable is
  latched) so neither late-arm arrangement wedged on RESRDY - which
  WEAKENED sercom.md's categorical wedge sentence to "can, not must"
  (Fable's reconciliation; every wedge the UART suite caught in the
  act carried a corrupted write-back in hand, and the kick stays as
  never-doubling insurance); 1.10.4'S DENSITY IS THE CONCURRENCY, NOT
  THE TRAFFIC (bounded-wait churn 43000 blocks, ZERO refusals; sprayed
  churn 2733/2744 refused in ~36000 harvests, every one refused and
  never believed, samples exact throughout); abandon() IS NOT ALWAYS
  ENOUGH - SWRST is ignored SILENTLY while ENABLE stands (25.8.18) and
  ENABLE does not clear until the buffer drains, so the recovery
  ladder is TWO RUNGS and the second (Dmac::init()) is deliberately
  not automated - resetting the block stops every other channel, a
  program-wide decision; A 24-BIT DATUM NEEDS A WORD BEAT (SDADC
  RESULT[15:0] is a DIFFERENT NUMBER, not a narrower reading - at a
  rail the raw 8388607 does not fit a halfword at all; TSENS streamed
  on the same word engine). SUPERVISOR FIX at review: laps_/overruns_
  are ISR-written and thread-polled, so they became volatile reads
  (the ticker doctrine - gcc -Os deleted a bare polling loop once);
  re-verified at the bench after the edit. Rate 4980/s against 5000
  nominal, both rulers OSC48M (checks the arithmetic, not the
  oscillator). DELIBERATELY NOT BUILT and said where: a util streaming
  AO (born-with-users - Multislope will dictate the contract), linked
  descriptors, automatic ladder escalation. Canaries test_samc_dma z
  112/112 and test_samc_uart z 27/27 (agent AND Fable); check_samc,
  check_family, host green; 2 new negatives; family dmac.cpp now
  cross-checks the four analog trigger codes. Docs: dmac.md grown,
  dac.md/adc.md/sdadc.md each gained a "Streaming via DMA" section,
  bench.md row + board C firmware line.
  **BLOCK-STREAM CONTRACT DONE 2026-08-29 (same day, BY FABLE'S OWN
  HAND on the user's ruling): the util level the streaming campaign had
  deferred was OVERRULED - brio must be versatile beyond Multislope,
  and the contract built BEFORE its second implementation is the FIXED
  POINT the stm32g0x1 platform will be measured against, so friction
  shows up as "this concept does not fit" instead of silent
  divergence.** util/block_stream.hpp NEW: BlockSource/BlockPlayer
  concepts that speak BLOCKS AND NOT DMA (caller-owned buffers,
  overruns skip laps, the accounting IS the API; satisfiable by an
  interrupt handler on a machine with no DMA - stated, not built) and
  BlockRelay<P, Subs, Sources...>, the event-driven AO that is
  MeterSampler's opposite economy (every block delivered exactly once
  where the meter discards stale): each filled block travels as a
  Lease::dispatch loan (LendsTo-checked - borrowers precede the relay
  in the pack), is verified/consumed INSIDE the receiving dispatch, and
  is returned to its source on the relay's NEXT dispatch, whose
  self-post also guarantees the restart of a stalled source
  (SerialPort's two-buffer contract, generalized). design/
  block-stream.md NEW records the inverted doctrine. NEW HOST SUITE
  test_block_stream (host 23/23 total): a scripted ping-pong source
  honest to the SAM engine's contract - loan timing, storage reused
  only after release, stall drained oldest-first with release
  restarting, coalesced wakeups neither lose nor duplicate, accounting
  passed through. SILICON: test_samc_analog_dma grew letter k (z now
  78/78, four runs incl. one cold) - the SAME DAC-to-ADC chain through
  a REAL kernel, 12 blocks, worst residual 3 of band 10, every loan
  home, zero engine overruns, DmaPingPongEngine/DmaLoopEngine
  concept-checked in the family fixture at every width plus a negative
  (DmaTxEngine refused as a BlockSource). THE LETTER RE-TAUGHT THE
  PRINT LESSON at a new scale: a verdict printed between chain_up()
  and the pump overran the engine ONCE, DETERMINISTICALLY, in every z
  run and never solo - the console ring full of the previous letters'
  output blocks print() for milliseconds and the engine's slack is two
  blocks (9.6 ms), so the letter measures everything first and prints
  after. Zero-cost held: 26 pre-existing SAM images byte-identical
  (test_samc_nvm's build-id defsym moved because new FILES entered the
  tree - the standing by-design exception); check_samc, check_family,
  host all green. Deliberately absent and stated in the design doc: a
  playback AO, one-shot burst vocabulary, gap policy, the AVR
  implementation (born with its first user).
  **THE TRANSVERSAL SLEEP/SLEEPWALKING PASS DONE 2026-08-29 (Opus
  delegation, REVIEWED BY FABLE and COMMITTED same day, all six
  judgment calls accepted plus one supervisor comment alignment;
  memory samc-session-2026-08-29-sleepwalk).** The gap FIFTEEN chapter
  docs deferred to "the power pass": what every peripheral does while
  the CPU is stopped (the PLATFORM half stays test_samc_sleep's). NEW
  SUITE test_samc_sleepwalk z 76/76 (agent x5 + Fable x3, cold runs
  both hands; ~6 s) + letter p 5/5 outside z. THE INSTRUMENT IS ITSELF
  A FINDING: a pad cannot be pull-walked by a sleeping CPU, so the
  stimulus is HARDWARE - TC on OSCULP32K -> combinational CCL LUT ->
  asynchronous EVSYS -> PORT EVENT INPUT with EVACT=OUT, the one
  action 28.6.4 says survives a standby (28.6.5 separated and
  measured: OUT bypasses the OUT register and moves the pad even under
  PMUXEN; TGL writes the OUT bit - the pull's direction - at half rate
  awake and DEAD in standby, which doubles as the suite's
  APB-is-really-down control). samc/pin.hpp GREW the PORT event-user
  surface port.md had declared a gap (PortEventAction/Config,
  event_user published per the EVSYS ruling, evsys.hpp NOT included -
  table 29-3's async-only rule a stated obligation, the ac.hpp SOC
  precedent); 2 negatives; all 28 pre-existing images BYTE-IDENTICAL
  (re-proven by Fable's worktree gate). THE HEADLINES: (1) ERRATUM
  1.11.6 REFUTED AT REV F against a matrix that marks every revision -
  an ASYNCHRONOUS EXTINT detected 100 edges of 100 inside ONE standby
  (counted by its own event generator into a RUNSTDBY TC, interrupt
  disarmed so the window was one sleep; kernel tick frozen and the TGL
  row dead as controls) and with the interrupt armed all 100 WOKE the
  device; eic.hpp's comments now carry claim AND measurement (Fable's
  alignment - the sampled fallback stays documented for silicon where
  it bites). (2) THE FDPLL DOES NOT STOP IN STANDBY for a peripheral
  that asks, RUNSTDBY clear or set - count across the sleep = count
  awake tick for tick (1414 vs 1413), platform.md's open question
  answered OPPOSITE to its guess; CLKRDY/LOCK read set at wake either
  way so neither is evidence. (3) CHANNELn.RUNSTDBY GATES THE
  ASYNCHRONOUS EVSYS PATH TOO (32 crossings with, 0 without) - table
  29-1's three SYNC rows invite the wrong reading, and this was the
  bug that stalled the suite's first version. (4) 19.5.2's "can only
  be re-enabled by a system reset" is about the clock INSIDE the
  block, not MCLK's mask: Pm::bus_clock(false) comes back (letter p,
  outside z because the chapter's claim could become true on other
  silicon). TABLES ENTERED: ADC 38-4 with a REAL SleepWalking chain
  (RTC periodic event, async channel per 1.4.4, 31/32 conversions in a
  30 ms standby, no CPU; erratum 1.4.5 DOES NOT REPRODUCE), SDADC 39-1
  (89/87/1; 1.8.7 sidestepped by free-running, its own escape), TSENS
  43-1 (4/4/0, and THE WITNESS HAD TO BE WINMON - TSENS publishes no
  result-ready generator), TCC 36.6.6 (16/15/0), CCL 37.6.4 EXACT BOTH
  HALVES (combinational 100/100 with GCLK_CCL not running; filtered/
  synched 1 = the wake's seam, 100 with RUNSTDBY), AC 40.6.14 both
  sequences (continuous RUNSTDBY wakes 8/8 at ~14 ms vs a 91 ms
  backstop; RUNSTDBY clear wakes NOTHING even with GCLK_AC force-fed -
  the bit gates the COMPARATOR, not its clock; one stray wake in 32
  recorded, not rounded away; single-shot SleepWalking via RTC->SOC0
  runs), DAC 41.6.6 (pad holds 2030/4096 across standby; 1.9.2
  reproduces WITH control). MORE: EXTINT wakes in 7 us; the EIC has NO
  RUNSTDBY bit and detects sampled edges in standby even on a
  generator with RUNSTDBY CLEAR (its clock request is honoured, not
  what table 19-4 predicts); a peripheral's own RUNSTDBY carries the
  whole chain (OSC48M/OSC32K/DPLL measured, ONDEMAND changes nothing
  when something asks); FREQM finishes a measurement ASLEEP and DONE
  wakes (reference on OSCULP32K, the meter's roles inverted); RTC
  compare/periodic/alarm all wake with no RUNSTDBY anywhere. NOT
  STAGEABLE, said so: BODVDD as wake - INTFLAG.BODVDDDET is a
  TRANSITION not a level (a standing condition cannot re-fire, not
  even to a sampling detector), so a detection needs a supply
  crossing; the unrequested-clock question (every witness is itself a
  request); sleep current (no meter). ERRATA: 1.9.2 live+reproduced;
  1.11.6 and 1.4.5 refuted; 1.25.2 unreachable by construction (no
  ONDEMAND verb); 1.3.1 declined twice over (rev B AND a consumption
  claim); 1.4.4/1.8.13/1.2.3 applied as code. METROLOGY: an unbound
  vector is a WATCHDOG RESET mid-letter (AC_Handler missing - the
  banner saying WDT with the watchdog "disabled" was the clue);
  APBCMASK resets to ZERO here (Evsys::bus_clock(true) is not
  optional); a free-running stimulus makes a wake test a coin toss
  (retrigger a slow wave right before each sleep, judge
  nothing-woke-it legs by the SLEEP'S LENGTH never by interrupt
  counts); the backstop belongs in the sleep primitive (an RTC compare
  in standby_until_wake, the watchdog only where the compare register
  IS the alarm register). Canaries sleep z 87, eic z 85, ccl z 141
  (both hands); check_samc/check_family/host green. Docs: platform.md
  gained "Sleep, peripheral by peripheral"; port.md + 14 chapter docs
  each lost exactly the gap this closed; bench.md row + firmware line.
  **TIMER DMA + THE TIMERS' ADVANCED MODES DONE 2026-08-29 (Opus
  delegation, REVIEWED BY FABLE and COMMITTED same day, all six
  judgment calls accepted): the trigger ids both timer drivers had
  published and nothing had ever used, and the gaps tc.md and tcc.md had
  been carrying since their own campaigns.** NEW SUITE
  test_samc_timer_dma z 101/101 THREE TIMES (twice warm, once cold from a
  fresh flash) plus a fourth green run after the canaries, 10 letters,
  under four seconds, WIRELESS - and NOT ONE LINE OF DRIVER CODE
  CHANGED: every edit to brio/ in this pass is a COMMENT, the md5 gate
  showing all 29 pre-existing SAM release images byte-identical with the
  new suite the only addition. THE INSTRUMENT IS THE FIRST FINDING: a
  TCC or TC waveform reaches a capture channel through a COMBINATIONAL
  CCL LUT published as an EVSYS generator, so both its edges are delayed
  alike and a period and a pulse width come out untouched - Lut<0>'s
  INSEL "TC" is TC0's WO[0] and its "TCC" is TCC0's, so one fabric
  carries either, with no pad, no pull and no wire (the sleepwalk
  campaign's stimulus technique, reused as a MEASURING instrument).
  THE HEADLINE is the ROUND TRIP: a DmaLoopEngine plays an eight-entry
  duty table into TCC0's CCBUF0 on the OVF trigger while TWO
  DmaPingPongEngines drain the capture meter's CC0 and CC1, and over 192
  judged samples of each stream the captured widths ARE the played table,
  in order, with a WORST ERROR OF ZERO TICKS and a phase holding across
  every lap boundary of the loop and every block boundary of both streams
  - the period not moving by a single tick, no overrun, no 1.10.4
  refusal. So the streaming engines have their THIRD peripheral family
  and util/'s stream contract was not touched. THE DOCTRINE THE TIMERS
  QUALIFY: 25.8.8 makes a DMA trigger the RISE of a request the
  peripheral holds up, which is why a SERCOM's TX engine needs kick() and
  why the ADC stream drains RESULT before arming - but A TC CAPTURE'S
  REQUEST IS NOT A LEVEL WAITING TO BE RE-RISEN. A stream armed with
  INTFLAG.MCx already standing filled two whole blocks in fifty periods,
  and - the half that rules out "selecting TRIGSRC over a high level
  looked like a rise" - a stream STALLED to a dead stop and re-enabled
  with CHCTRLB UNTOUCHED and the flag still up picked straight up again.
  Every capture asks again, read or not; and a DMA beat is the
  acknowledgement a CPU read would have been (eight blocks each on both
  channels with INTFLAG.ERR never rising, where thirty unread periods
  raised it). THE OTHER MEASUREMENTS, driver by driver. TC: the captured
  period is the waveform's LESS ONE TICK every time (the capture edge
  both latches COUNT and clears it); MFRQ toggles on every match (4799
  captured for CC0 = 2399, pad 499 per mille); MPWM SPENDS CC0 AS THE
  PERIOD AND CHANNEL 0'S OWN OUTPUT DEGENERATES - WO[0] matches only at
  TOP and is high for all but one tick, pad 999 per mille and the LUT
  agreeing, while WO[1] on PA23 carries the duty at 250 against 250; the
  16-bit TcPwm task runs (242 per mille, captured period 65535); INVEN
  inverts (757); PWP is PPW with the two registers exchanged and PW needs
  one channel where PPW needs two; STAMP walks the counter by 18 or 19
  counts against 4800/256 = 18.75; PRESCSYNC = GCLK STARTS THE COUNTER A
  WHOLE PRESCALED TICK EARLIER than either prescaler-synchronized option
  (means 17.96 / 16.90 / 17.10 over 300 retrigger phases, reproducible to
  a hundredth of a tick) while PRESC and RESYNC are DECLINED as
  indistinguishable from the CPU, since a RETRIGGER through CTRLBSET is
  itself taken on a prescaled clock; and CTRLA.ALOCK holds a buffered
  write until the UPDATE command, with the LED pad as the witness (247 ->
  754 free, 245 -> 245 -> 745 locked). TCC: a COMPARE REGISTER IS A WORD
  AND A DUTY STREAM'S BEAT MUST BE ONE (a halfword write into CCBUF lands
  in the low half alone - 0x00ABCDEF then 0x1234 reads 0x00AB1234); a
  flooded loop ran 1229 laps in 20 ms against 25 paced by the TCC and
  reported NOTHING wrong, because the loss is a STORE the peripheral
  discarded and a discarded buffered write loses a value without
  corrupting one; THE HARDWARE CIRCULAR BUFFER (WAVE.CICCEN0, set under a
  running timer) plays two values for ever with no CPU and no DMA, and
  CIPEREN does the same for the PERIOD, against the software loop's one
  interrupt per lap - so the circular buffer wins at two values and loses
  at three; NFRQ toggles on the PERIOD and CC0 moves nothing at all,
  where MFRQ makes CC0 the top; DUAL-SLOPE CRITICAL's arithmetic, which
  36.6.2.5.7 never prints, is width = (PER - CC0) + (PER - CC2), exact to
  the tick at three settings; RAMP2A PAIRS TWO COUNTER CYCLES (the period
  doubles, the width does not move) and does NOT give two duties as its
  name invites; recoverable fault B is A's mirror on channel 1's event
  input; FCTRLn.FILTERVAL COUNTS GCLK_TCC CYCLES AND NOT PRESCALED ONES -
  the dead-time unit's story again and NOT what 36.8.5 or the driver's
  own comment said - measured as a THRESHOLD on one 93 kHz generator (20
  us bare, 160 us at FILTERVAL 15, UNMOVED by a sixty-fourfold prescaler
  change, where fifteen cycles of that clock are 160.9); a blanking
  window gates the INPUT and not its edge; FCTRLn.QUAL ties the fault to
  THE FAULT'S OWN CHANNEL output (CC1 for fault B, and the first version
  moved CC0 and measured a qualifier that never fired at any duty);
  EVACT0 = INCREMENT gives COUNT = 20 for twenty pulses exactly and
  EVACT0 = COUNT turns the counter into a gate (0 counts low, 1863
  against 1864 high). ERRATUM 1.21.7 STAGED AND DID NOT REPRODUCE: a
  dithered duty shows as EXACTLY TWO pulse widths, and under a periodic
  hardware RETRIGGER arriving at a rate that never cuts into the pulse it
  still showed exactly those two - with a CONTROL proving the instrument
  sensitive (move the retrigger so its landing point walks through the
  pulse and the distinct-width count rises at once, dithered and
  undithered alike, which is a retrigger's own doing). Recorded as
  unreproduced, not disproved - the standing 1.21.8 already has. 1.21.5
  (advanced capture) is DELIBERATELY NOT ATTEMPTED and stays stated: it
  is a property of the whole channel set and wants a varying capture
  stimulus this letter set has no free channel for. A CLOCK CORRECTION
  FELL OUT ON THE WAY, and it reconciles rather than overturns: DIVSEL's
  divisor is 2^(DIV+1) SATURATED AT 2^(field width + 1) - on generator 7,
  whose DIV field is eight bits, DIV = 8 and DIV = 9 give the SAME 512 -
  which agrees with test_samc_clock letter f where the two overlap and
  fixes only the extrapolation past the width. TWO METHOD LESSONS PAID
  FOR IN FAILED VERDICTS: a TC CAPTURE REGISTER IS TWO REGISTERS, so one
  read taken after the signal under test has been reconfigured hands back
  the PREVIOUS arrangement's capture (the first waveform-mode letter
  "measured" MPWM's period as MFRQ's), and A VERDICT LINE IS FOUR
  MILLISECONDS OF CONSOLE where a block of this stream is two and a half,
  so a print between arming a stream and draining it overruns the engine
  - which passed alone and failed inside z, the worst way to find it.
  Docs: tc.md and tcc.md lose exactly the gap lines this pass closes and
  gain the numbers, dmac.md gains the trigger-doctrine qualification and
  the timers as the engines' third family, clock.md gains the DIVSEL
  ceiling, bench.md gains the suite row and board C's firmware. Canaries
  re-run by hand: tc z 77/77, tcc z 143/143, analog_dma z 78/78,
  sleepwalk z 76/76, dma z 112/112 (agent AND Fable - the reviewer also
  re-proved the comment-only claim by worktree md5, 29/29, and ran the
  new suite's cold z); check_samc OK, check_family OK, host 23/23. All
  six judgment calls ACCEPTED at review (the DIVSEL ceiling verified
  measured, not inferred); memory samc-session-2026-08-29-timer-dma.
  **THE ANALOG COMPLETION DONE 2026-08-29 (Opus delegation, REVIEWED BY
  FABLE and COMMITTED same day, all judgment calls accepted; memory
  samc-session-2026-08-29-analog).** One suite for five chapters -
  test_samc_analog z 136/136 (agent x8 incl. two cold, Fable x3 incl.
  one cold), 12 letters, ~3.5 s, WIRELESS, and again NOT ONE LINE OF
  DRIVER CODE: the two brio/ edits are measurement-anchored comments
  (30/30 pre-existing images byte-identical by the reviewer's worktree
  gate). THE HOST/CLIENT ADC PAIR run for real: one host trigger starts
  both converters, agreeing to ZERO counts on the shared pad - and THE
  CLIENT'S OWN ENABLE BIT DOES NOT STAND (ADC1.CTRLA reads 0x20, init()
  writes ENABLE and it does not stick, the converter converts anyway -
  38.6.3.1's "enabled by accessing the CTRLA register of Host ADC" is
  literal and Adc<1>::enabled() lies); INTERLEAVE buys the RATE OF ONE
  SIGNAL (2495 results for 2500 triggers = exactly twice what one
  converter sustains) where BOTH buys two inputs at the old lateness;
  A MISSED TRIGGER IS NOT AN OVERRUN (a converter drops half its
  triggers with the flag CLEAR - 38.6.5 is about an unread RESULT, so
  OVERRUN is no witness for over-pacing); and ONLY ONE OF 38.6.3.1'S
  THREE RESTART OPTIONS RESTARTS ANYTHING - trigger parity is the
  witness (software triggers alternate strictly), FLUSH and a
  disable/enable cycle both carry the parity straight through, only a
  SOFTWARE RESET defines it. The SEQUENCE walked over known-distinct
  inputs; DIFFERENTIAL finally measured with the DAC as the swept
  mid-scale source; NEITHER R2R NOR OFFCOMP MOVES THE READING (worst 2
  counts of 4096; the 1.3 mV floor stated, not an absence claimed) and
  OFFCOMP IS SHORTER - it REPLACES SAMPLEN (28 ticks saved). DAC
  DITHERING RUN AT LAST: 1024 samples hitting each of the 16 sub-slots
  exactly 64 times, means 32487 -> 32548, SWING 61 WHERE 15/16 LSB IS
  60 (4.1 counts per bit vs 4.0 ideal), the undithered control stepping
  a whole LSB; LEFTADJ on silicon; EMPTY/UNDERRUN through a real
  handler. THE AC COMPLETED: COMP2/COMP3 and WINDOW 1 on silicon at
  last; THE HYSTERESIS MEASURED - 118 mV high-speed / 113 mV low-power
  (both inside table 45-34, both edges moving, erratum 1.5.2's pairing
  legal and behaving); THE BANDGAP NEGATIVE INPUT DOES NOT NEED VREFOE
  (measured both ways - ac.hpp's own comment corrected: 22.6.2.2 is
  about the ADC INPUT CHANNEL path, the one that really is dead
  without the bit) and the bandgap weighed by the DAC lands 1044/2083/
  4178 mV, a third independent route; ERRATUM 1.5.6 REPRODUCES RARELY
  (1 enable in 64, the workaround as the control at zero -
  pass-as-declined); THE SWAP RECIPE RETURNS A BOUND, NOT A NUMBER
  (SWAP inverts terminals AND output so the sense is unchanged -
  gotten backwards it produced a nonsense 246 mV; corrected: offset
  under one DAC code). SDADC: WINMONEO moves a real witness; ONE
  SWTRIG.FLUSH COSTS NO WINDOW but FLUSHEI events every 20 us stop
  every result dead (recorded, deliberately not reconciled);
  ANACTRL's CTLSDADC/BUFTEST DECLINED with the reason (factory/test
  bits). TSENS: STARTINV needs a LEVEL (a comparator output - same
  fact as the AC's INVEIx); the window hysteresis DECLINED (one-way
  self-heating is no stimulus). A DEVIATION THE SILICON FORCED: this
  family's COMPCTRL has HYSTEN only, NO HYSTSEL, so "both hysteresis
  levels" became both SPEEDS. Errata re-read by the row: 1.4.10 spent
  visibly (the ADC1-first order again), 1.5.1/1.9.1/1.4.7/1.5.4 not
  this silicon by row AND measurement where reachable. A citation fix:
  the DAC's dithering is 41.6.8.4, not .3 (dac.hpp's comments). At
  review Fable also retired ac.hpp's stale sleep line (the 40.6.14
  sequences ARE measured - sleepwalk letter h). Canaries adc 97 / dac
  108 / ac 94 / sdadc 101 / tsens 168 both hands; check_samc,
  check_family, host green; adc.md/dac.md/ac.md/sdadc.md/tsens.md
  moved in the same change, bench.md row + board C firmware. Still
  open by honest necessity: VREFA (a wire), the voltage pump (the
  supply), the util adapters (born with Multislope).
  **NVJOURNAL + THE FUSES VERB DONE 2026-08-30 (Opus delegation with
  the explicit util carve-out - the campaign's SUBJECT was a new util
  file, the NvHeap precedent - REVIEWED BY FABLE and COMMITTED same
  day, all nine judgment calls accepted; memory
  samc-session-2026-08-30-journal).** THE SAM'S EEPROM CLASS EXISTS
  NOW: util/nv_journal.hpp NEW - NvJournal<Media, max_ids,
  max_payload, half_pages> over the SAME FlashMedia concept as NvHeap,
  two halves at the media's top ping-ponging WHOLESALE, entries
  appended cell-granular with seq + CRC-16, HIGHEST-SEQ-WITH-VALID-CRC
  WINS and the newer half is a collection's destination - two rules
  that resolve ALL FOUR power-cut positions (torn append, torn
  collection, torn source erase, torn DESTINATION erase) with no
  special case in the code. Mount READ-ONLY (a boot costs no cycle,
  158 us measured). THE PANIC RESERVE is the centerpiece: an ordinary
  save collects EARLY so room for one max-size entry always stands in
  pre-erased cells - save_reserved() does no GC and no erase, one
  bounded polled program, legal from a panic handler; JournalPanic +
  take() give the SAM the breadcrumb that survives a POWER LOSS (the
  .noinit one survives only resets). THE GEOMETRY INVARIANT IS
  (max_ids + 2) x entry <= half - the agent CORRECTED the brief's
  max_ids + 1, one term short (the entry being written), and the
  reviewer verified the arithmetic before accepting. NEW HOST SUITE
  test_nv_journal (host now 24/24): 59 cases / 51567 assertions over
  THREE geometries - 256/64 (RWWEE), 2048/8 (THE STM32G0'S DOUBLE-WORD
  FLASH - the journal validated on the third target's geometry before
  that target exists, the block-stream doctrine applied to storage),
  512/2 (the one shape where a header spans cells) - with power cuts
  at EVERY program unit of a save and of a collection, and the reserve
  guarantee across 3000 randomized saves. THE RWWEE PARTITIONED ONCE
  (RwweePartition: rows 0..27 the heap, 28..31 the journal's attic;
  the old on-chip map reported lost by the survival-aware mount, WHICH
  IS THE DESIGN WORKING - test_samc_nvm's letter e adapted, the sole
  md5 mover). NEW BENCH SUITE test_samc_journal z 58/58 (agent x3 +
  Fable x3 incl. cold), p 11/11 across a REAL panic + reset, v 6/6
  across a reflash of blink and back (Fable re-ran the whole
  choreography by hand). NUMBERS: a save 347..357 us (the delta over
  the bare 190 us page program is the entry image and the bitwise
  CRC), a collection 5.8 ms, NO STALL (~2500 polling turns inside one
  991 us row erase), coexistence proven (a 300-byte heap block
  byte-exact while the journal collects over it), wear 168..184 row
  erases per z run. THE PANIC FINDING: break_here()'s BKPT escalates
  to HardFault BEFORE any reporter runs on a board whose C_DEBUGEN
  bench.py cleared, so the breadcrumb is written by the HardFault
  BODY, not the reporter - the suite binds both and records which ran.
  bench.py's `fuses` verb LEARNED THE SAM: read/decode of the 32-byte
  user row (BOOTPROT, EEPROM size, BODVDD, WDT, LOCK - dangerous
  fields marked [guarded]), write via hand-driven EAR + WAP with
  WHOLE-ROW read-modify-write always (factory bits preserved
  bit-exact), old row printed before any write, read-back verify,
  refusal of unknown fields and of WDT-always-on/BOOTPROT without
  explicit acknowledgement; validated by a write-identical round trip
  and a real bodvdd_hysteresis flip confirmed in SUPC after reset, the
  row back at production default. nvm.md's user-row gap CLOSED; the
  main-array FlashMedia backend RULED not-built (born with its first
  user - the RWWEE serves both storage classes; the stall, the 25k
  endurance and the linker-zone problem stay deleted);
  design/nv-journal.md NEW records the two-spellings ruling (NvRecord
  speaks real EEPROM, NvJournal speaks flash - unification waits for
  the G0 or a cross-target app). A METHOD TRAP FOR THE MD5 GATE, now
  on record: pinning mtimes makes sources OLDER than existing objects
  and ninja relinks stale ones - the gate is valid only with the build
  dir wiped after pinning (the reviewer's worktree gate always did).
  At review Fable also retired nvm.md's stale wish for DSU/PAC drivers
  (both exist since the debug campaign). Gates: md5 30/30 + the
  declared mover, check_samc (1 positive + 4 negative TUs),
  check_family, host 24/24, canary test_samc_nvm z 52/52 - agent AND
  Fable.
  **MTB INTO THE POST-MORTEM DONE 2026-08-30 (Opus delegation,
  REVIEWED BY FABLE and COMMITTED same day, all eight judgment calls
  accepted; memory samc-session-2026-08-30-postmortem). Group 5's
  second item, and the DIVAS ruling (359396a) closed its third the
  same day.** samc/postmortem.hpp NEW: MtbPostMortem<trace_bytes,
  keep_packets> - a rolling MTB buffer in .bss and a SEPARATE .noinit
  record (magic + CRC-16 + count + source + packets) that crosses the
  reset, written by capture() which FREEZES FIRST and refuses to
  overwrite a standing record (hard_fault_reset's own rule), read once
  by take(). The two entry paths are PURE COMPOSITION - TracingReporter
  <Store, source, Next> and hard_fault_trace_reset<P, Store>() - with
  NOT ONE LINE of reset.hpp, platform_sam.hpp, kernel/ or util/
  changed; mtb.hpp grew freeze() and the oldest-first snapshot()
  additively (32/32 pre-existing images byte-identical, Fable's
  worktree gate). NEW SUITE test_samc_postmortem z 36/36 (agent x4 +
  Fable x3 incl. cold) with the reboot letters f 13/13 and p 10/10
  OUTSIDE z. THE PROOF: a UDF three calls deep is LEGIBLE AFTER THE
  RESET - 6..7 packets, the three calls in the order made, the
  exception entry last, the PanicRecord and the trace read side by
  side; the entry packet's source is +8 bytes into the dying leaf and
  its destination the HardFault handler's linked address. A NEW
  SILICON FINDING: BIT 0 OF THE SOURCE WORD MARKS THE EXCEPTION ENTRY
  - exactly one packet per fault trace carries it, settling what the
  debug campaign's letter could only report as absent. The capture
  itself costs 2 packets; FREEZE-FIRST is measured, not asserted (read
  stopped: 9 packets, 3/3 chain leaves; read running: zero packets in
  common with the stopped read and a run of 4 identical packets - the
  copy loop's own backward branch flooding the tail); one three-deep
  chain costs 9 packets, which is the measured justification for
  keeping 16 of the 32-packet buffer (136 bytes of .noinit).
  THE ORDERLY-PANIC PATH IS THE FAULT PATH ON THIS BOARD (BKPT
  escalates with C_DEBUGEN cleared - the journal campaign's finding
  confirmed): an app that wants a trace must bind the fault body, and
  the reporter's own capture is proven separately against a
  non-resetting Next. The journal letter was DECLINED WITH ARITHMETIC
  (136 bytes needs 3 cells and (1+2)x3 = 9 > the attic half's 8 -
  recorded in mtb.md's gaps; Fable re-checked the numbers). Docs:
  mtb.md's integration-with-panic gap CLOSED, platform.md's breadcrumb
  gains its trace sibling, reset.md points at the composition.
  Canary test_samc_debug z 117/117 both hands; check_samc,
  check_family, host 24/24.
  **THE STANDBY-SURVIVING TIMEBASE DONE 2026-08-30 (BY FABLE'S OWN
  HAND on the user's blessing - group 5's last item, WHICH CLOSES THE
  SINGLE-BOARD ROSTER ENTIRELY; what remains needs hardware).** The
  design the user blessed, minus one piece that proved unnecessary:
  the RTC is the ALARM and the WITNESS while SysTick stays the ticker
  - and the SleepSite concept trait proposed alongside was NOT needed,
  because the manager's deadline guard already admits far deadlines
  and everything else fits inside arm()/disarm(): THE POWER MODEL IS
  UNTOUCHED, which design/power.md now records as the second
  same-target validation (a site can LIFT a target restriction with
  the model unchanged). samc/ticker.hpp grew advance(n) (the resync's
  landing point, guard-held, with the caller owing the FROZEN span
  only); samc/sleep.hpp grew SamTimedSleepSite<P, cfg> - arm() places
  a COMP0 alarm on ticks_to_next() rounded UP, disarm()/isr() hand the
  frozen span (RTC-elapsed converted DOWN minus what SysTick itself
  counted) to advance(), the baseline consumed once under the guard,
  and the rate rule is DIRECTIONAL (state a rate not below the true
  one; the default 33500 over-estimates OSCULP32K so every error lands
  LATE - the kernel's own "at least"). THE FINDING THAT COST THE FIRST
  VERSION A WEDGE, caught by halt-and-dump plus a RAM read of the
  counters: the never-early bias GUARANTEES kernel time is still short
  of the deadline at the alarm (advance 490 against a deadline 491
  ticks out, deterministically), so resync alone re-entered the
  still-armed standby behind a spent alarm - THE ALARM'S ISR MUST DO
  THREE THINGS: acknowledge, resync, and HAND THE MACHINE BACK TO A
  TICKING SLEEP (SLEEPCFG to IDLE0; the residual ticks mature on
  SysTick in milliseconds). A foreign wake leaves the alarm STANDING,
  and the model's after-a-wake convention (speak to the manager, even
  with SleepRequested{none}) is LOAD-BEARING with this site - stated
  on it. NEW SUITE test_samc_timebase z 15/15 x5 (one cold), 4
  letters, wireless, inside a REAL kernel with the crystal TC pair as
  the judge: the alarm arithmetic EXACT (16750 counts for 500 ms at
  the stated rate); a 500 ms event through a standby matures at 507 ms
  of wall (band nominal..+3.5%), resync ~473 ticks, millis() honest
  against the crystal (502 over 510); the watchdog's early warning as
  a mid-sleep intruder, the convention re-requesting and the alarm
  re-placed for the remainder, deadline still met; six 150 ms rounds
  all at 152 ms and NOT ONE EARLY. A suite lesson: the pump must DRAIN
  the convention's tail before judging counters (the wake report and
  the reply are still queued when the blip lands). 33/33 pre-existing
  images byte-identical (the additive claim proven by worktree gate);
  canaries sleep z 87, sleepwalk z 76, platform z 34; check_samc (new
  family coverage + a negative: a sub-1024 Hz rate refused),
  check_family, host 24/24. platform.md's timebase gap CLOSED,
  power.md carries the model-unchanged finding, ticker.hpp's and
  sleep.hpp's standing caveats rewritten to the two-site truth.
  **SERCOM SPI DONE 2026-08-31 (PHASE F's FIRST HALF, and the first
  CROSS-ARCHITECTURE campaign: an Opus delegation the user stopped
  mid-flight, taken over and finished BY FABLE'S OWN HAND - four suite
  defects diagnosed at the bench, one driver verb added, the peer
  hardened). COMMITTED same day.** samc/spi.hpp NEW: the whole of ch. 32
  in the two strata - Spi<n> (both roles over ONE register view, SPIM =
  SPIS asserted field by field against the header's own SPIS macros;
  every enable-protection and all three SYNCBUSY bits spelled per
  register, incl. 32.8.2's enable-raises-SYNCBUSY.CTRLB trap; DOPO
  refused as the TRIPLE it is - four rows, and WHICH SIGNAL IS WHICH
  DEPENDS ON THE ROLE, so one harness is a host on row 0x0 and a client
  on row 0x2, both proven on the same seven wires) + SpiHost (the avrdx
  Request shape verbatim - cs/dc PinRefs, two-phase cmd + full-duplex
  data, Borrowed reply loans, per-request BAUD/mode, polled or ISR pump
  on RXC-never-DRE - so THE VICTORY CONDITION HELD: util/spi_bus.hpp =
  BusMaster ran the kernel letter with NOT ONE LINE of util/ or kernel/
  changed) + SpiClient (preload, SSDE, address match, the dark-listener
  drive_output). sercom.hpp grew instance()/spi_regs() additively;
  spi_link.hpp/spi_peer grew host_burst (THE ROLES INVERT: the
  instrument becomes the bus host so the DUT's client half is
  exercisable at all - old ops untouched). NEW SUITE test_samc_spi z
  61/61 x TEN consecutive (cold + nine warm) on the SEVEN-WIRE bench
  this session verified conductor by conductor over SWD + UPDI. THE
  BENCH FINDINGS: errata 1.17.16 AND 1.17.19 both NOT REPRODUCED at rev
  F in SPI mode (SWRST resets from the disabled state - measured from
  three states incl. really-clockless, where it lands once the channel
  returns - and DBGCTRL survives SWRST; both disciplines KEPT, one
  enable of cost); A MODE CHANGE IS A CPOL FLIP ON THE WIRE - flipped
  inside an open select window it is one extra edge and a selected
  client counts it: an exact ONE-BIT SLIP both directions, modes 2/3
  only, which is why the engine applies before its own cs falls and why
  SpiHost::prime() now exists for callers framing CS by hand; THE
  THREE-SCK-CYCLE RULE'S CYCLES ELAPSE ONLY WHILE SCK RUNS, so a client
  answering on RXC (in the gap) is ONE CHARACTER LATE every time and
  the working pump is ONE AHEAD (preload b0, park b1, write next+1 per
  RXC - the peer then read 12/12 from the first character, PLOADEN's
  whole promise); CTRLB.MSSEN measured raising SS between EVERY
  character (4 rises in a 4-char burst - hardware SS frames a character,
  never a transaction, hence the GPIO chip select); the receive buffer
  is TWO deep with BUFOVF/IBON behaving per 32.6.2.7; loop-back through
  the pad and nine-bit characters loop whole; all four modes x both
  orders byte-exact both ways, a DORD mismatch an exact two-way bit
  reversal; back-to-back characters (no gap - one engine request) bind
  at the PEER'S POLLED TURNAROUND, not its CLK_PER/6 electrical ceiling
  (exact to 500 kHz always, 1 MHz a measured coin toss - 4/10 runs -
  2 MHz never: a 10 us byte against a 5..9 us loop). FOUR SUITE DEFECTS
  FIXED BY HAND: a function-local static made MSSEN's letter flaky on
  every second run; the 1.17.16 control read PCHCTRL.CHEN one
  instruction after disconnect() and measured its own race (CHEN is
  write-synchronized - the suite now waits, and clock.hpp's
  fire-and-forget disconnect() is a caveat to know); the boundary
  verdict claimed the peer's electrical ceiling where the binding limit
  is its software turnaround; and THE PEER'S SELECT-WAIT WEDGE - about
  once in five z-runs, persistent until board A reset, the peer's
  exchange window spun on a select READ that never fired while its SPI
  hardware demonstrably shifted (preloads + echo on the wire, the pad
  reading LOW over SWD even in the wedged state's own status) -
  NEUTRALIZED by making run_exchange poll RXC directly (the wait added
  only the mechanism that failed; a byte can only arrive selected) with
  the select kept as Report TELEMETRY (aux1..aux3) so a recurrence
  names itself; the mechanism stays unhunted, an avrdx-side question
  for an AVR bench. Family fixture + SEVEN negatives; gates: worktree
  md5 33/35 byte-identical + the two declared build-id movers (2 and 4
  bytes, same sizes), check_samc, check_family, host 24/24; canaries
  test_samc_uart z 27/27 (sercom.hpp moved) and test_samc_dma z
  112/112. Docs: spi.md NEW PROVISIONAL (gaps: DMA engines, sleep,
  SSDE/address-match on silicon, 1.17.3's dummy, the wedge's cause),
  sercom.md's SPI gap closed, bench.md's desk truth rewritten (the
  seven-wire table, A = spi_peer, C = test_samc_spi). JUDGMENT CALLS
  QUEUED - see memory samc-session-2026-08-31-spi. I2C (ch. 33) is
  phase F's open half.
  **SERCOM I2C DONE 2026-08-31 (PHASE F CLOSED - BY FABLE'S OWN HAND,
  same day as the SPI half). COMMITTED.** samc/i2c.hpp NEW: ch. 33 as
  TWO resources, because unlike the SPI's views I2CM and I2CS really
  differ - I2cm<n> (bus state machine, the THIRD SYNCBUSY bit SYSOP
  with the wait-before-store discipline, force_idle as the dependable
  UNKNOWN exit, BAUD with the chapter's rise-time arithmetic solved
  both ways and pinned by static_asserts, Fm+ split 1:2 per the note)
  and I2cs<n> (AMATCH/DRDY/PREC, the three address modes, GENCEN) -
  plus I2cHost (the avrdx TwiHost Request VERBATIM, always
  asynchronous, one interrupt per byte, the i2c_* vocabulary produced
  ON THE WIRE, per-speed cached register pairs with speed_ok() and the
  refused-never-slowed rule, and unstick() - the avrdx verb's twin
  that now leaves a healthy wire untouched) and I2cClient (the
  erratum discipline built in: answer_address() sweeps 1.17.11's
  leftovers and arms 1.17.22's first_drdy() gate). Errata as code:
  1.17.8 - the W1C masks CANNOT NAME CLKHOLD by construction; 1.17.10
  - no ten-bit client knob exists; 1.17.13 refused both ways; 1.17.21
  - no AACKEN knob (the workaround IS an AMATCH handler and I2cClient
  is one); 1.17.16 NOT REPRODUCED in I2C mode either; HS refused
  (1.17.7/9 break its repeated starts with no workaround). sercom.hpp
  grew i2cm_regs()/i2cs_regs()/gclk_slow_id() additively. NEW SUITE
  test_samc_i2c z 39/39 FIVE TIMES incl. cold, 10 letters, every
  command itself TWO tenures of the engine under test (twi_link
  included by relative path, the spi_link ruling). THE CAMPAIGN'S
  HEADLINE IS A WIRE FINDING WITH A LADDER: the C21's I2C - host
  monitor AND client machinery - samples the wire on GCLK_CORE with NO
  input filter, and the phase F seven-wire bundle's per-edge crosstalk
  (~100 ns) reads as false Starts/Stops: a hand-driven SWD tenure dies
  BUSERR+ARBLOST at 48/24/12 MHz core AT EVERY SCL RATE, is clean
  six-for-six at 6 MHz, and at a 32 kHz core the address was WATCHED
  crossing the wire and the peer ACKed - so the suite runs the core
  from generator 6 (OSC48M/8) and I2cHost::init gained the stated
  core_hz (the freqm reference_hz pattern), with Fm+ therefore
  unreachable-and-refused on this desk; the CLIENT, which cannot slow
  edges it does not own, matched a bit-banged address perfectly
  (AMATCH + stretch, registers identical) and stayed DEAF to the
  peer's real 100 kHz at BOTH 6 and 48 MHz core while that host read
  the AVR campaign's own nobody-home signature (MSTATUS 0x72) - the
  client letter DECLINES its data verdicts with the finding printed
  (the TC-1.20.2 precedent) and the standing fix is PHYSICAL: take the
  I2C pair out of the bundle. The AVR's filtered TWI ran this very
  node at 1 MHz - the filter, not the wire, was the difference all
  along. ALSO MEASURED: one wire fault raises MB AND ERROR TOGETHER
  and the engine's first version left the second standing - an ISR
  storm caught by halt-and-dump (IPSR = the SERCOM's IRQ, main
  starved), now structurally over (finish() sweeps every exit, the
  idle guard sweeps stray levels); a tenure into a held-low wire PARKS
  (BUSSTATE busy for the hold's whole length, not a byte moved) and ON
  THIS SILICON the parked START does NOT fire when the phantom-Start
  hold releases - recovery is the engine's re-init, the suite prints
  the timeline; whether INACTOUT walks UNKNOWN->IDLE by itself came
  out BOTH WAYS (recorded, not judged; force_idle is the dependable
  exit); the repeated start of a write-then-read tenure is TWO address
  matches from the client's side (addr_hits 4 for 2 combined tenures);
  commanded 2 ms/byte stretching = exactly 16 ms for 8 bytes, data
  intact; deaf peer answers write AND empty probe i2c_nack_addr, a
  commanded 3rd-byte refusal answers i2c_nack_data; unstick 0 clean /
  4 at the peer's 4th-edge release; the kernel letter ran I2cBus (=
  BusMaster) over the REAL wire - ordered replies, the NACK delivered
  IN ITS PLACE as a reply, immediate rejection, both sleep votes - so
  BOTH bus vocabularies now hold cross-architecture WITH NOT ONE LINE
  of util/ or kernel/ changed. Suite-craft paid for: a serve ended by
  exact count cuts a combined tenure in half (deadline + settle is the
  clean instrument exit); a mid-letter peer_act under a kernel-mode
  handler starves the bare spin wrapper (one serve per letter);
  tenure()'s deadline path re-inits the engine so no letter inherits a
  parked START. Family fixture + SIX negatives; gates: worktree md5
  34/36 byte-identical + the two declared build-id movers (the
  identity of every pre-existing image doubling as the sercom.hpp
  canary), check_samc, check_family, host 24/24. Docs: i2c.md NEW with
  the headline, sercom.md's I2C gap closed, bench.md's bundle finding
  + suite row. JUDGMENT CALLS QUEUED - see memory
  samc-session-2026-08-31-i2c. PHASE F IS CLOSED; of the whole SAM
  plan only CAN (a second C21) remains.
  **SAM-SAM SPI DMA CAMPAIGN DONE 2026-09-02 (by Fable's own hand on
  the user's blessing - the two-C21 desk's first campaign, run
  autonomously).** The desk became TWO SAM boards behind a USB hub
  (positions C and D, manifest re-verified; D carries a 32.768 kHz
  crystal on PA00/PA01, recorded for a future 32 kHz pass) with the
  five-wire straight-through SPI link verified conductor by conductor
  over SWD alone. samc/src/apps/spi_peer.cpp NEW - the avrdx peer
  ported over the SAME spi_link.hpp (one wire format, two
  architectures, two peers): dark listener, one-ahead pumps, raw-host
  host_burst, regimes mapped onto PLOADEN, ident label = die serial -
  and exchanges SERVED THROUGH ITS OWN DMA ENGINES by default, the new
  protocol bit spilink::spare_polled_pump forcing the polled loop so
  both boundaries stay measurable (the AVR peer ignores the bit by
  construction). samc/spi.hpp: SpiHost grew TWO OPTIONAL DMA ENGINE
  SLOTS (the Uart shape, NoDmaEngine default, engineless build
  byte-identical): the DATA PHASE rides the DMAC - RX drains on RXC
  and its block's completion IS the transaction's, TX feeds on DRE, a
  null tx sends 0xFF from a held source, a null rx drains into a held
  sink - the command phase stays on the byte pump with the handover
  inside isr(), dma_isr(channel, flags) on the DMAC vector, status()
  carrying spi_ok or spi_dma_fault (the first engine-defined BusDone
  code, target-local), the polled spin bounded with abandon-and-report,
  engines both-or-neither/byte-only/distinct-channels (three new
  negatives), Dmac::init() the app's. dmac.hpp grew start_fixed()/
  start_discard() as SIBLING VERBS - a defaulted argument was tried
  first and MOVED three pre-existing images; the worktree gate caught
  it and the siblings restored byte-identity, which is the ruling:
  byte-identity outranks API economy. THE HEADLINE FINDING INVERTS THE
  KICK DOCTRINE PER SERCOM MODE: in SPI host mode, enabling a TX
  channel with DRE already standing FIRES THE FIRST BEAT BY ITSELF,
  and a kick on top is one extra beat whose byte the full transmit
  buffer discards in silence - measured three ways with the lost byte
  moving exactly as the model predicts (the OPPOSITE of the UART
  campaign's rise-latch wedge; dmac.md carries the qualification, the
  SPI launch kicks nothing). Also paid for: stop() disarms a channel's
  interrupts, so every error path that stops an engine re-arms it.
  test_samc_spi grew to 8 letters / 71 verdicts, z 71/71 THREE TIMES
  (two warm, one cold): letter d is now a TWO-LEG ladder to 24 MHz -
  polled pumps exact to 2..3 MHz (the peer's answer reload; at the
  first failing rung the peer still hears every byte exact), BOTH ENDS
  ON ENGINES exact to 6 MHz breaking at 8 STILL in the reload, the
  client's receive side clean to 24 MHz - and letter h is the DMA host
  wireless (polled + ISR-style, the mid-window handover, dummies and
  discard, loop-back exact to 12 MHz with the 24 MHz rung recorded not
  judged). AN OPERATIONAL FINDING WITH TEETH: behind the hub the
  Atmel-ICEs' usb_bulk CMSIS-DAP transport desynchronizes by one
  packet under sustained traffic - bench.py now forces `cmsis-dap
  backend hid` on every OpenOCD invocation, recovery is two
  USBDEVFS_RESET ioctls 5 s apart or a replug, and a failed program
  leaves PARTIAL FLASH: always check "Verified OK" (a silent failure
  ran one z against stale firmware before the rule was learned).
  Gates: md5 worktree 34/37 byte-identical + the two declared build-id
  movers (uart/dma/serial_speed provably identical = the canaries' job
  done by the gate), check_samc, check_family, host 24/24, avrdx
  test_avr_spi/spi_peer compile with the grown protocol. Docs: spi.md
  (DMA sections, the three-boundary ladder, gaps moved), dmac.md (the
  per-mode doctrine + sibling verbs), bench.md (the two-board desk,
  the hub finding, both rows). Nine judgment calls in memory
  samc-session-2026-09-02-spi-dma. Deliberately not built: SpiClient
  engine slots (the peer uses raw engines - a driver slot waits for a
  device-shaped user), erratum 1.17.3's staging (now possible, not
  done), SSDE/SSL and address-match on silicon (gaps stand).
  **samc/delay.hpp BORN 2026-09-02 (same session's tail, user-prompted):
  the microsecond busy-wait over SysTick VAL - "at least" never early,
  and CAPPED BELOW ONE KERNEL TICK by contract (a tick or more is
  TimeEvent territory and is REFUSED with false and no time spent; the
  boundary avrdx/delay.hpp never had, drawn right at birth). Reads VAL
  only (SysTick stays the Ticker's in writing), delta accumulation with
  the wrap folded in, correct inside SysTickInterruptGuard windows; NO
  DIVISION AT WAIT TIME with a compile-time Clock - the M0+ has no high
  multiply, so gcc soft-divides even by constants (~4 us/call, measured:
  the first version paid it every entry) and both quotients fold
  instead. test_samc_platform grew letter d (z 34 -> 39, 39/39 x4 warm +
  cold, letter i 20/20 across its six reboots): 5..900 us at-least with
  ~1 us of true call overhead, 200 x 50 us not one early, the cap and
  the no-Ticker refusals at bracket cost. TWO SUITE DEFECTS FOUND AND
  FIXED ON THE WAY: a warm second z failed letter a because letters b
  AND c reprogram WDT CONFIG/EWCTRL and nothing restored the fuse-loaded
  values (restore now shared, called by both, AFTER the disable's sync -
  an unsynchronized store there is discarded); and the measurement
  bracket of two back-to-back synchronized TC reads costs ~6..10 us of
  its own, which the letter's first version charged to the delay (zero
  measured first, min-vs-min for the refusal legs). SpiHost's
  cs_setup_us Request knob stays future growth with its first device
  user; the bench suites keep their proven calibrated spins (comments
  updated to say the facility now exists). Family TU delay.cpp; gates:
  md5 worktree 35/38 + the two build-id movers + the platform suite
  (the declared mover), check_samc, check_family, host 24/24, spi
  canary z 71/71.
  **SAM-SAM I2C CAMPAIGN DONE 2026-09-02 (by Fable's own hand,
  autonomous, same desk-day as the SPI campaign): THE FILTERLESS-I2C
  FINDING CLOSED FROM BOTH SIDES.** The I2C pair got its own short
  separated wires with external pull-ups (verified present and stronger
  than an internal pull-down by the SWD wire check - continuity and
  isolation open-drain style, no firmware). samc/src/apps/twi_peer.cpp
  NEW - the avrdx peer ported over the same twi_link.hpp: polled client
  on the samc verbs (AMATCH resets the decoder, DRDY moves bytes, the
  closing NACK gated past 1.17.22's first-DRDY blind spot), commanded
  stretch = the wait before answering (this silicon stretches by
  construction), serve/coll/hold_sda/quiet/arb - the last a MODE SWITCH
  because one SERCOM is host OR client, never the AVR's combined both -
  ident label = die serial, AND THE BET IN ITS BANNER: core at 48 MHz.
  THE BET WON: on the clean pair the peer's client serves the command
  channel at the very rate the bundle deafened, so the suite was
  promoted - core_gen 6 -> 0, BOTH ends at 48 MHz, z 39/39 three times
  incl. cold - and the two doors the bundle had closed opened at once:
  letter h TIGHTENED BACK into its data verdicts (the foreign 100 kHz
  host's 10-byte burst byte-exact through the 48 MHz-core client), and
  letter f ran FAST-MODE-PLUS ON THE WIRE - 25 tenures x 16 bytes in
  6 ms at 1 MHz, the stratum's first Fm+, with the refused-never-slowed
  rule keeping its proof on a throwaway 6 MHz core claim. samc/i2c.hpp
  grew ONE additive verb, I2cs/I2cClient::end_transaction() (table
  33-3's CMD 0x2: after the host's closing NACK of a read the client
  goes back to waiting for a start) - born because the samc peer is the
  stratum's FIRST CLIENT-TRANSMIT user (nothing had ever SERVED a read
  before), and the md5 gate proves the growth pure (every pre-existing
  image but the edited suite byte-identical). THE ARBITRATION GAP GOT
  ITS REASONED WALL instead of a letter: the AVR's deterministic race
  armed held STARTs against a bit-banged Busy and released them on one
  edge, but this silicon's parked START DOES NOT FIRE on a phantom
  release (letter g's measured timeline) - the rendezvous primitive is
  absent, a live race wants a third node; i2c.md says so. SMBus
  time-outs still open, with board D's 32 kHz crystal named as their
  designated future source. Gates: md5 worktree 35/38 + the two
  build-id movers + the suite, twi_peer new, check_samc (the family
  fixture covers the new verb), check_family, host 24/24. Docs: i2c.md
  headline gains its second half (the bundle ladder stands as the
  measured hazard, the clean pair as the measured absence), bench.md
  wiring + rows + end state. Judgment calls in memory
  samc-session-2026-09-02-i2c.
  **THE SMBUS TIME-OUT LETTER DONE 2026-09-02 (same desk-day's third
  campaign, user-prompted) - AND THE OBVIOUS READING OVERTURNED.**
  test_samc_i2c grew letter j (z 39 -> 47; 47/47 x3 incl. cold): the
  meter is OSC32K with its factory trim on a generator of its own,
  WEIGHED ON FREQM before being trusted (32.59 kHz), the shared SLOW
  channel moved with the PCHCTRL.CHEN synchronization waited out (the
  fire-and-forget disconnect() race, met again and coded around).
  THREE FINDINGS, none in the chapter: (1) the host's SMBus time-outs
  POLICE THE HOST'S OWN CLOCK HOLD, NOT THE WIRE - 80 ms of client
  stretch under both enables completes i2c_ok with no time-out bit
  ever rising (the CTRLA descriptions foretell it: every remedy is an
  automatic STOP, physically impossible under a client's hold), while
  the host's own unserviced MB trips LOWTOUT at a measured 30 ms
  (25..35 window to the letter) with the exact documented signature
  (STATUS.LOWTOUT+BUSERR, INTFLAG.ERROR beside the standing MB);
  (2) THE COUNTER ARMS AT configure() - enables left in CTRLA by an
  earlier configuration time nothing until a fresh
  disable/write/enable cycle; (3) A WRONG-RATE METER IS MUTE, NOT
  SCALED - with the SLOW channel at 48 MHz the same hold never trips
  at all (a clock-domain limit), which is why the letter weighs its
  meter first. CONSEQUENCE, stated in i2c.md: a bus hung BY A CLIENT
  stays software's to bound (deadline + unstick) - the hardware
  time-outs bound only this host's own software, a case a live ISR
  never produces; the earlier doc line that promised LOWTOUT as "the
  hardware answer to BusMaster's no-timeout" was falsified by the
  bench and rewritten. Gates: md5 36/39 + the two build-id movers +
  the suite, check_samc, host 24/24. Docs i2c.md (findings + the gap
  closed, MEXT stated) and bench.md moved in the same change.
  **THE PER-BUS TIMEOUT DONE 2026-09-02 (same desk-day's fourth
  campaign, the letter-j consequence built where it belongs - a util/
  change BY FABLE'S OWN HAND, ruled per-bus by the user with the
  post-fault client re-verification left to the application).**
  util/bus_master.hpp grew `timeout_ticks` (fifth template argument,
  default 0 = today's arbiter BYTE FOR BYTE - the never_retries
  discipline again, and the gate PROVED it: 36/39 samc + 40/41 avrdx
  images byte-identical, the only movers the new letter and the three
  standing build-id defsyms at their usual 2/4-byte signature, so
  test_samc_spi/uart/dma's identity doubles as the canary): every
  transfer that goes asynchronous arms a ONE-SHOT TimeEvent (a raw
  TimeEvents<P>::Base node with its own fire glue, because the posted
  BusTimeout must carry the SEQUENCE NUMBER at fire time - kernel/
  untouched); if it matures first the engine is declared dead,
  Bus::recover() (static_asserted at the spelling; a new samc neg TU
  refuses an engine without it) puts the PERIPHERAL back where
  start() is legal, the requester is answered bus_timeout (255, top
  of range - engine codes grow UP from 2 and can never collide) IN
  ITS PLACE, and the queue moves on; the retry Policy is NOT
  consulted (the engine never spoke). THE RACE WITH THE REAL
  COMPLETION IS CLOSED BY CONSTRUCTION both ways: a stale BusTimeout
  is dropped by seq mismatch, and a straggler TransferDone posted in
  the window recover() closes necessarily precedes the self-posted
  BusFlushed marker in the queue (recover() silences the engine), so
  a DRAINING state between them is deterministic, not probabilistic -
  both halves staged exactly there in the host suite (the fake posts
  the straggler FROM INSIDE recover(); test_bus_master 18/18, host
  24/24). ENGINES: avrdx TwiHost's recover() already existed (the
  errata's ENABLE cycle - reused untouched, and it is the verb the
  contract names as its model); samc I2cHost::recover() re-runs the
  init() tail from the cached config as its OWN body (no init()
  refactor - an uncalled template verb costs nothing, byte-identity
  over code economy) because a parked START never fires on release
  and re-init is the only exit; samc SpiHost::recover() closes the
  select window FIRST, puts the engines away and re-claims them,
  resets and reconfigures; avrdx SpiHost::recover() is NEW (silence +
  IF clear, restore_host() for the mid-transfer demotion, CS up) -
  both avrdx verbs compile-proven on every package and STATED as
  not-bench-run in their docs. SILICON WITNESS test_samc_i2c letter l
  (z 47 -> 52, 52/52 x3 incl. cold): a tenure into the peer's 60 ms
  SDA hold - the wedge letter j proved the silicon cannot see - came
  back i2c_timeout AT 35 MS EXACTLY (the arbiter's own limit) WITH
  THE SDA PAD STILL LOW AT THE REPLY (the pad is the witness;
  recover()'s force_idle has just rewritten the monitor), zero stale,
  and the SAME bus AO carried the next tenure i2c_ok after the
  release; a 1 ms/byte stretcher completed i2c_ok in 9 ms under the
  same limit (stretching is flow control - the limit sits above the
  tenure by design). TWO LETTER LESSONS: the timed_ao_live flag is
  raised ONLY around the kernel pumps (the peer commands ride the
  bare engine path, and the first version starved them into a link
  failure); and the wedge witness is the PAD, never the monitor.
  I2cBus/SpiBus aliases pass Policy + timeout through; i2c_timeout/
  spi_timeout name the code per vocabulary. Family: timed
  instantiations in all four bus TUs (avrdx twi/spi x8 packages, samc
  i2c/spi x3 headers) + the neg; check_family, check_samc green.
  Docs: design/i2c-bus.md's "still missing" paragraph became "The
  per-bus timeout" (three rulings recorded), design/spi-bus.md points
  at it (the SPI wedge is a dead engine, not a wire), samc/i2c.md's
  consequence line now names the mechanism and letter l, samc/spi.md
  + avrdx twi.md/spi.md state their recover() and the not-staged
  gaps, bench.md's suite row moved. Judgment calls in memory
  samc-session-2026-09-02-timeout.
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
- **Energy experiment - experiments/ tier born, first map point DONE
  2026-08-31 (by Fable's own hand at the office bench, one day from
  question to data).** The question: was deferring DynamicClock on the
  SAM right? NEW top-level `experiments/` (ruled: not "examples" - no
  maintenance promise), self-contained per-experiment dirs globbed by
  both build projects; experiments/energy/ holds the whole thing and
  DOCUMENTS ITSELF (README = rationale, pre-registered predictions,
  wiring, instrument constants, results). The SAM C21 is world + judge
  + meter for an AVR DUT: DAC seeded stimulus, witness on an AC
  comparator (C21 VIH = 0.7xVDD rules out the EIC at 3.3 V), SDADC
  free-running energy windows (R_shunt 10.18 ohm ratio-calibrated
  in place, offset zero-cal, zero overruns). FIRST POINT: prediction
  refuted then repaired - the naive free-running watcher loses to
  DynamicClock pace by 22%, the frugal one-conversion-per-tick watch
  beats pace by 27% (168.6 vs 232.4 mJ) - HOW you watch is worth 1.7x,
  more than any clock choice; the SAM deferral HOLDS on this point.
  Board facts: floating input buffers 0.49 mA, crystal 0.51 mA, the
  AliExpress ADuM clone ~2 mA quiescent (the board rev 1.2 list lives
  in memory). Open: run.py + two-layer logs, the map sweep, brackets,
  the cap instance. Full story: experiments/energy/README.md + memory
  power-experiment-brief.
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
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>)          # STM32G0 release build
(cd stm32g0 && cmake --build --preset stm32g0b1re-release --target <app>-upload)   # flash via OpenOCD (ST-LINK)
tools/check_stm32g0.sh [name]                                      # same for the stm32g0 stratum (g0b1/g071/g031 headers)
# apps are auto-discovered from <project>/src/apps/*.cpp - plus
# experiments/*/{avrdx,samc}/*.cpp, each experiment's per-arch app
# halves - at every configure; no generation step; a new/removed app
# or a changed "// build: opt = value" line takes effect on the next
# configure

python3 tools/bench.py list                  # serial devices, USB probes, the bench manifest
python3 tools/bench.py flash A test_avr_pin  # cmake --build --target <app>, then avrdude/UPDI
python3 tools/bench.py flash C test_samc_dma # ... or OpenOCD/SWD - the BOARD TYPE decides both
python3 tools/bench.py flash E console       # ... or OpenOCD/ST-LINK (the Nucleo-G0B1RE, position E)
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
stm32g0/                 the STM32G0 build project, same shape again (the
                         part number selects define + ld + crt; ST-LINK
                         upload target; svd/STM32G0B1.svd)
test/CMakeLists.txt      the host test project (independent - one CMake
                         configure has exactly one compiler):
                         one executable + ctest entry per test_*/main.cpp
test/CMakePresets.json   the "host" configure/build/test preset (native g++, UBSan)
test/test_*/main.cpp     host unit tests (doctest), cd test && ctest --preset host
test/family_samc/        samc family smoke TUs + neg/, tools/check_samc.sh runs them
third_party/doctest/     vendored doctest.h (MIT, upstream doctest/doctest)
third_party/samc21-dfp/  vendored Microchip.SAMC21_DFP include tree (Apache-2.0)
third_party/cmsis-device-g0/  vendored ST cmsis-device-g0 v1.4.5 Include/ (Apache-2.0)
test/family_stm32g0/     stm32g0 family smoke TUs + neg/, tools/check_stm32g0.sh
third_party/cmsis-core/  vendored ARM CMSIS-Core headers (Apache-2.0)
tools/check_family.sh    family compile check over test/family/ (see above) -
                         zero CMake coupling, calls avr-g++ directly
tools/check_samc.sh      the samc twin over test/family_samc/
tools/bench_boards.py    the bench MANIFEST: the physical boards on the desk
                         (type, console by-path, programmer) - not a target list
tools/uart_stress.py     the host end of test_samc_uart: the same xorshift the
                         firmware generates, plus the baud and frame changes only
                         an OUTSIDE sender can make - the suite's streaming
                         letters cannot be run without it
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
experiments/             one SELF-CONTAINED directory per cross-cutting bench
                         experiment (ruled 2026-08-31; deliberately not
                         "examples" - no maintenance promise): both
                         architectures' app halves (<name>/avrdx/*.cpp and
                         <name>/samc/*.cpp, globbed by the respective build
                         projects; app names unique per arch), the shared
                         wire-protocol header beside them, its own README
                         (rationale, pre-registered predictions, wiring,
                         protocol, log format - docs/ never references it),
                         python driver + analysis, logs/ git-ignored.
                         energy/ = the clock-strategy energy experiment
                         (DynamicClock-deferral verdict; the SAM as
                         stimulus + judge + meter for an AVR DUT)
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
    nvic.hpp               "sam.h" + armv6m/nvic.hpp (the guard and Nvic live there)
    platform_sam.hpp       SamPlatform (idle takes whatever PM.SLEEPCFG holds -
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
                           and its own guards
  stm32g0/               everything that knows stm32g0xx.h (STM32G0, Cortex-M0+)
    device_tables.hpp      THE RESERVE: GPIO ports, USART instances, APB
                           enables, CCIPR multiplexers and the SHARED
                           VECTORS, read off the device header (the last
                           off the device-select macro)
    nvic.hpp               "stm32g0xx.h" + armv6m/nvic.hpp
    ticker.hpp             armv6m/ticker.hpp + the Ticker alias (1000 Hz)
    platform_stm32.hpp     Stm32Platform (WFI = Sleep mode, SLEEPDEEP never
                           written; BKPT; .noinit breadcrumb; atomic_width 4)
    flash.hpp              FlashWaitStates (table 13, the read-back rule),
                           FlashAccel, flash_size_kb - the FLASH campaign's
                           future home
    clock.hpp              Rcc (HSI16/HSIDIV, the PLL, SYSCLK switch, bus
                           prescalers, the per-peripheral ENABLES, CCIPR),
                           Pwr::range, Clock<internal|pll, hz> with the
                           compile-time PLL ratio search - the third clock
                           model
    pin.hpp                Pin<'A',5> / Port<'A'> / PinRef over GPIOx: the
                           port clock opened by every configuring verb,
                           MODER/OTYPER/OSPEEDR/PUPDR/AFR, BSRR/BRR values
    usart.hpp              Usart<n> resource + Uart<n, pins> task, the
                           other two targets' Uart surface verbatim
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
