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
  `ring.md`, `analog.md` (the sampler usage type + arithmetic).
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
Includes carry the stratum prefix (`#include "avrdx/uart.hpp"`). The
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
Handlers dispatch with `brio::match(e, lambdas...)`. Drivers expose
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
- **Payload/ownership pass.** kernel.md section 4 states the rule in
  three lines; a dedicated pass over util/ producers (naming the lease
  at every field, spans of `reply` loans) is pending.
- **HSM.** The FSM contract is HSM-ready (`unhandled` = future
  bubble-to-parent); parent pointers, bubbling and LCA entry/exit
  chains get built only when a real AO demands them.
- **Second target.** Candidates STM32G0x0/x1, ATSAMC/D, CH32V00x
  (SysTick timebases at 1000 Hz). It will exercise the Platform
  concept, the per-target ticker/driver strata and the layering rule
  for real; expect radical revision of util/ and drivers then.
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
  17 mV). TCD deferred. Multislope next when wanted. A 2026-08-20
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
  with their first user. -> CCL -> AC ->
  ADC/DAC/VREF -> CLKCTRL (DA must compile) -> EVSYS tables. Phase 2,
  never-reviewed: USART
  (jumper cross-loopback, new suite test_avr_serial) -> SPI (host ->
  client on SPI0/SPI1, 4 jumpers) -> TWI (host -> client TWI0/TWI1) ->
  delay/platform sweep. Jumper tests
  always check bench.md collisions first. TCD stays deferred.
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
pio run -e <app> -t upload    # flash via Atmel-ICE (UPDI)
pio debug -e <app>-debug      # ALWAYS debug the -debug env
python tools/gen_apps.py      # after adding/removing src/apps/*.cpp (or a "// pio: opt = value" line)

PY=~/.platformio/penv/bin/python         # pyserial + pio live in PlatformIO's venv
$PY tools/bench.py list                  # serial devices, USB probes, the bench manifest
$PY tools/bench.py flash A test_avr_pin  # build the app's env + avrdude over UPDI
$PY tools/bench.py run A a               # drive the console, judge "ALL: N pass, M fail"
$PY tools/bench.py console A             # device path + speed, for your own monitor
$PY tools/bench.py duo A:a B:script.txt  # instrument peer scripted, then the DUT
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
tools/pio_flags.py       per-language AVR flags (build-type aware) +
                         IntelliSense include paths (skips [env:native])
tools/gen_lst.py         post-build: firmware.lst (disassembly) + firmware.map
src/apps/<app>.cpp       one main() per app (ISR vector bindings live HERE)
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
    ring.hpp               Ring<T, size, P> SPSC FIFO, lock-free when the
                           index fits P::atomic_width, guarded otherwise
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
                           (set<hz>()/set(hz) rebases the users then switches)
    delay.hpp              delay_us(clock, us) "at least": folded cycles when
                           constant, 4-cycle loop otherwise; cycles_per_us
    pin.hpp                Pin<'A',5> compile-time GPIO (also a PwmChannel,
                           max 1) + PinRef descriptor + PinSet<Pins...> mask
                           + port_by_letter/pinctrl_of (run-time port lookup)
    uart.hpp               Uart<n, Route, rx, tx> interrupt-driven transport
    spi.hpp                Spi<n> master engine (two-phase descriptor,
                           per-byte ISR pump, CS owned by the engine)
    twi.hpp                Twi<n, Route> I2C master engine
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
```

## Build artifacts

`.pio/build/<env>/`: ELF / HEX / MAP / `firmware.lst` (source-
interleaved disassembly from tools/gen_lst.py).
