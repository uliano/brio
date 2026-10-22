# Documentation rewrite: progress record

A temporary work record for the rewrite described in
DOCUMENTATION_REWRITE_BRIEF.md. Not reader documentation; not linked from
any public page. Both files are to be removed before the final merge
unless the user keeps them.

## Baseline

- Branch `docs-rewrite`, base commit for the first batch: 4d20f8f (the
  brief and the revised editorial paragraphs of CLAUDE.md and
  docs/README.md). Main at launch: c6509ff.
- Checks available in the execution environment: `bin/brio prose` (runs,
  no missing dependency) and `bin/brio gate --tokens --against 4d20f8f
  <files>` (needs a host g++ to lex; `/sw/gcc/bin/g++` 16.2 is present
  here, so the check runs). No build and no host test was attempted, as
  the brief instructs; the cross toolchains under /sw are present on this
  machine but were not used. Everything that needs a compiler or a board
  is in the local handoff below.
- Pre-existing `brio prose` state of the files touched: clean (0 errors,
  0 review items on README.md and docs/boards/nucleo-g0b1re.md before
  editing).

## First batch: scope and file list

Documented route: the maintained teaching example `blink`
(stm32g0/src/apps/blink.cpp) on the ST Nucleo-G0B1RE. Chosen because the
board page (docs/boards/nucleo-g0b1re.md) names the LED and its pin, the
launch configuration "Debug STM32G0B1RE (OpenOCD, Nucleo-G0B1RE)" exists
in .vscode/launch.json, the board needs no wiring (LED and probe on the
board), the flash and debug tools are one OpenOCD release, and the same
blink source builds for the two other G0 Nucleos. The identical launch
shape was performed interactively on the Nucleo-G071RB
(docs/stm32g0/README.md, "Debugging"), which is the strongest evidence
available for the G0B1RE configuration without a desk.

Created:
- docs/learn/README.md (the route's table of contents and where next)
- docs/learn/introduction.md (main loop -> events -> queues -> active
  objects -> cooperative scheduling, with the scheduling diagram)
- docs/learn/first-program.md (blink read in stages)
- docs/learn/cpp-notation.md (the C++ used by blink, at the point of use)
- docs/learn/command-line.md (build, flash, observe, debug from a
  terminal; verification status per step)
- docs/learn/vscode.md (the VS Code guide and its status matrix)
- DOCUMENTATION_REWRITE_PROGRESS.md (this file)

Rewritten:
- README.md (root)
- docs/README.md (the learning route's rows in the map; nothing else)

Comments only (token-identity checked against 4d20f8f):
- brio/kernel/fsm.hpp, active_object.hpp, tenuto.hpp, event_queue.hpp,
  time_event.hpp, time.hpp, post.hpp, platform.hpp, borrowed.hpp,
  panic.hpp (opening comments and the public operations' comments)

Not touched on purpose: stm32g0/src/apps/blink.cpp (the quoted source
stays as it compiles; its `// build:` line is tool-read), every target
header, docs/design/*.md (a later batch), the `// build:` lines of every
app, CLAUDE.md beyond what the brief already revised.

## Incoming references to docs/design/kernel.md's numbered sections

Inventoried before any heading was touched; no heading of kernel.md was
changed in this batch, so every reference below still resolves. The
list is the dependency map for the batch that rewrites kernel.md: each
must be updated in that same change (a path check does not validate a
section number).

Markdown: CLAUDE.md (sections 1, 11, 12), docs/avrxt/README.md (1),
docs/ch32vx03/platform.md (1), docs/ch32x035/platform.md (1),
docs/ch32x035/README.md (1), docs/ch32v00x/platform.md (1),
docs/design/power.md (11), docs/design/runtime.md (10),
docs/design/overview.md (9, 10, 11), docs/design/serial.md (4),
docs/design/spi-bus.md (4, 5), docs/design/ring.md (12),
docs/rp2040/multicore.md (12), docs/rp2040/README.md (12),
docs/rp2350/multicore.md (12, twice), docs/stm32g0/platform.md (11),
docs/stm32f4/platform.md (11).

Sources: avrxt/src/apps/test_avr_platform.cpp (11),
brio/ch32vx03/platform.hpp (1), brio/ch32vx03/pfic.hpp (1),
brio/ch32x035/platform.hpp (1), brio/ch32x035/spi.hpp (1),
brio/ch32v00x/spi.hpp (1), brio/rp2040/nvic.hpp (11),
brio/rp2350/core_m33.hpp (1, 11), brio/rp2350/core_hazard3.hpp (1, 11),
brio/rp2350/multicore.hpp (12), rp2350/src/glue/startup_rp2350_riscv.S
(1, 11), rp2350/src/apps/test_rp2350_multicore.cpp (12),
brio/stm32f4/spi.hpp (1), brio/stm32f4/nvic.hpp (11),
brio/stm32f4/reset.hpp (11), brio/util/bench.hpp (1),
samc21/src/apps/test_samc_platform.cpp (8),
test/test_event_queue/main.cpp (5).

The learning pages link kernel.md by section heading text, never by
number.

## Facts resolved while writing

- "Never blocks" applies to post(): a full queue drops the event and
  counts it. A handler must not wait, but library calls can: print()
  blocks until the sink accepts every byte (util/print.hpp, "Delivery
  policy"). blink prints nothing; the introduction says both things.
- panic.hpp's opening said break_here() is "free NOP otherwise"; the
  Platform contract (platform.hpp) and kernel.md say the core decides
  (a NOP on the AVR, a HardFault on Cortex-M, a breakpoint exception on
  the QingKe cores). The panic.hpp comment now says what the contract
  says.
- Stm32g0Platform::idle() sleeps first and unmasks after (WFI wakes on
  a pending interrupt even under PRIMASK); the kernel's wording
  "re-enable then sleep" is the generic promise. The learning pages say
  the platform closes the window and point at the platform header.
- The .vscode/extensions.json recommends cortex-debug (the G0 route's
  debugger) and not ms-vscode.cpptools, which the cppdbg entries (AVR,
  CH32V00x, CH32V203, CH32V303) need. Recorded in vscode.md's matrix.
- The Serial Monitor extension (ms-vscode.vscode-serial-monitor) is
  configured by one setting (customBaudRates) and not recommended;
  blink has no serial output, so the route does not depend on it.

## Reader review (fresh subagents)

Two fresh readers per round, one on README + introduction +
first-program + cpp-notation, one on command-line + vscode. Each
received the audience profile, the file list and the brief's questions
only.

Round 1, concept route. Supported directly: the purpose, the costs, the
six-step Toggle path, the scheduling scenario (an interrupt posting to a
high-priority object while a low-priority handler runs), the CRTP
explanation, the compile-time checks, the full-queue behaviour.
Substantive gaps: `const Event& e` (the reference `&`) never explained;
brace initialization (`Toggle{}`, `heartbeat{Toggle{}}`) never
explained; `Fsm` never expanded and `Status`'s origin not said; whether
one interrupt handler can interrupt another not stated (so the safety
of two handlers posting to one queue could not be judged); how the
kernel sleeps with interrupts masked without losing a wake not
explained; "guarded by the platform P" used before P was defined; the
verification-status sentence pointing at a pull request the reader
cannot see; "no useful space" for empty events ambiguous; whether the
event being dispatched occupies a slot. All addressed in the pages
(introduction: the no-nesting promise and the sleep mechanism;
first-program: Fsm expanded, Status inherited, the reference
parameter, braces, P defined at first use, depth rule stated,
verification sentence rewritten; cpp-notation: two new sections,
references and braces; README: the no-nesting rule in the model
paragraph).

Round 1, workflow guides. Supported directly: the prerequisites with
their locations and overrides, the terminal and VS Code sequences with
expected results, the three-way distinction build target / upload
target / debug executable, the wrong-folder symptoms, the status of
every VS Code row. Two factual errors found: the pages told the reader
to check USB permission first on "no device connected", where the probe
page says to replug the board first and distinguishes that status from
an open failure; and the pages attributed the DBG_STOP leftover to a
debug session alone, where docs/stm32g0/README.md says the reset that
ends a `program` re-examines the chip, so a plain `-upload` leaves the
bit set too. Both corrected in both pages, and C_DEBUGEN (DHCSR) is
told apart from DBG_STOP (DBGMCU_CR). Other gaps addressed: no per-step
status in command-line.md (added to every step); "the pull request that
introduced this page" (removed); how a cache variable combines with
`--preset`, and how CMake Tools takes one (added); the udev rules file
by name (added); the size table described; `cd` back to the root before
gdb; what `print Blinker::queue` prints; expected results of the build-
and launch-target selections; the Debug "Expected result" separated into
what the G071RB session recorded and what the configuration should
allow beyond it; the launch.json comment located; a `.code-workspace`
file said absent.

Round 2: see below once run.

## Local verification handoff (after the batch)

- Compile: `(cd stm32g0 && cmake --preset stm32g0b1re-debug && cmake
  --build --preset stm32g0b1re-debug --target blink)` and the release
  preset; `brio check stm32g0 blink` if a filter fits; nothing in blink
  changed, but the pages quote it and the quotes must match the file.
- Token identity: `bin/brio gate --tokens --against 4d20f8f
  brio/kernel/*.hpp` (run here, results in the PR); the full `brio gate`
  on a machine with the toolchains confirms every image unmoved.
- VS Code, on the Nucleo-G0B1RE: the sequence in docs/learn/vscode.md,
  every row marked "configured" to be performed once and the matrix
  updated to "verified" or amended; the command-palette labels checked
  against the installed CMake Tools.
- Host tests: `(cd test && ctest --preset host)` unchanged by this
  batch; confirms nothing (no code moved) but is the route's own step.

## Next batches (not started)

1. docs/design/*.md, kernel.md first, with the reference inventory above.
2. One representative target stratum (stm32g0: README, selected
   peripheral pages, header comments).
3. The remaining target strata in groups.
4. Remaining services (serial_port.hpp and print.hpp first), devices,
   host, boards and probes.
