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

Round 2 (fresh readers, after the revisions). Concept route: every
round-1 gap answered "supported directly" (two handlers posting to one
queue, the sleep window, the reference parameter, the braces, Status's
origin, P, the depth rule). New, smaller gaps: the consumer side's
freedom from masking not explained; `start(&running)` is address-of
three lines after `&` was said not to be; what the machine does with
`unhandled()`; why `Entry` carries `brio::`; whether five posts fit in
an idle queue; the AVR's idle sequence (left to the platform header,
the route being the G0's). All but the last addressed in
first-program.md and introduction.md. Workflow guides: the two factual
errors confirmed fixed; the three status labels were not used literally
(now they are, on every step and on the bench-tool section); the
"observe" step named no board (now cites the target page's opening
record and says what is read from the source); whether
ARM_TOOLCHAIN_DIR takes the root or bin/ (now says the root, with an
example); "status 5" unexplained (dropped); the debug paragraph filed
Console details under "recorded" (now split: recorded on the G071RB =
load, reset-halt, breakpoint at main, stepping; everything else
expected). Known residue the pages state rather than hide: no VS Code
label is confirmed against the installed extension; no step is
performed on the G0B1RE as written; a relocated toolchain has not been
exercised through CMake Tools.

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
- Host tests: `(cd test && cmake --preset host && cmake --build
  --preset host && ctest --preset host)` unchanged by this batch;
  confirms nothing (no code moved) but is the route's own step, and the
  bare `ctest` on a fresh tree should report every suite not run, as
  command-line.md now says.
- After the revision, on a FRESH checkout (no build-cmake/): the clangd
  procedure of vscode.md (the two configures, then a header under
  brio/stm32g0/ and one under brio/kernel/ opened with no include
  errors); the full VS Code sequence on the Nucleo-G0B1RE as written;
  the full `brio gate` against 4d20f8f for the ten headers' images.

## Revision of the first batch

The independent review (DOCUMENTATION_REVIEW_BATCH1.md) applied to the
first batch's pages and headers, in a cloud session on a branch made
from `docs-rewrite`; one pull request against `docs-rewrite`. Scope:
the batch's files only. Not touched: the scheduler and the timers
(documented as they work), every code token (`brio gate --tokens`
identical on all ten headers), `// build:` lines, CMake, `.clangd`,
`.vscode`, docs/design/*.md, blink.cpp.

Environment: no cross toolchain, no `/sw/gcc`; the token check lexed
with the system g++ 13.3 (`cli/gate.py` calls `g++` from the PATH), so
it ran. The clone was shallow and was unshallowed before the check
(4d20f8f is an ancestor of the branch). No build and no host test was
run.

### Corrections needed in this batch

- clangd on a fresh checkout: vscode.md's clangd section now says the
  fragments name databases by path and not by the active preset, with
  a table of the four fragments the route reads (`stm32g0/.clangd` and
  `brio/stm32g0/.clangd` -> `build-cmake/stm32g0b1re-release/
  compile_commands.json`; `brio/.clangd` and `test/.clangd` ->
  `build-cmake/host/compile_commands.json`), the two configures a
  fresh checkout runs once (release preset, host project), in the
  terminal and in the status bar, and that the debug preset is selected
  afterwards while the editor keeps reading the release database. The
  matrix row says "two databases to create once". No fragment changed.
- The host-test build step: command-line.md step 6 configures, builds,
  runs, and says the test preset does not build; README.md's one-liner
  carries the three commands and a sentence on the bare `ctest`;
  docs/host/README.md's "Running" block carries the configure and the
  build (the one exception the review allows).
- Priority vs response time: README.md's model paragraph,
  introduction.md (the execution rule, the "Keep handlers short"
  bullet, the priority section after the diagram, which now says the
  diagram's "served next" is a fact of the scenario and states the
  three accepted limits: no preemption, no response-time promise, no
  fairness), first-program.md (stage 4 and the latency paragraph, which
  lists what an event waits for and marks the microsecond figure as
  read from the code, not measured) and tenuto.hpp (the rule paragraph
  and a new one on the scan order as a selection precedence).
- Absolute claims: introduction.md's active-object section replaces
  "never blocks" with the absence of a suspension mechanism and the
  application's duty to know which calls wait; the communication
  section replaces "no deadlock and no ordering puzzle" with what
  posting promises (the poster does not wait) and what it does not
  (a protocol can still stall; a completion event can be dropped), and
  links the payload rules; private state stated as the design rule.
  README.md's "ideas" bullet qualified the same way.
- Alignment: first-program.md stage 1 and cpp-notation.md say the
  slot is the largest alternative, the tag and the padding, give
  sizeof(SetPeriod) 2 and sizeof(Event) 4 under arm-none-eabi-gcc 16.2
  for the Cortex-M0+ (the author's local measurement), and point at
  `sizeof(Event)`.
- Tick conversion vs physical time: first-program.md stage 6 (the
  beginner wording: between N - 1 and N tick intervals, then the loop);
  introduction.md's time section points there; time.hpp's header and
  ticks_from_ms's comment say the rounding protects the count and
  nothing more; time_event.hpp's header gains "When a timer matures"
  (arm() stores now() + delay, process() posts at now() >= deadline),
  the catch-up paragraph says one post per turn per timer under
  post()'s drop rule, and a "Limits of the deadline arithmetic"
  paragraph states the half-range: delays and periods strictly below
  2^31 ticks, a deadline unserved for 2^31 ticks reads as future, a
  delay above 2^31 is posted at once. Exactly 2^31 is excluded from the
  contract (the second review): process() does not post it, but
  ticks_to_next() reports it due and next_deadline() prefers it to a
  nearer deadline, the two queries taking the difference the other way
  round. arm()'s and arm_every()'s comments carry the phase and the
  limit.
- Debugger observations: command-line.md step 5 says cursors and old
  slots are nonzero after events have passed, that `overflows_` is the
  one field that must stay zero, read as a stored field while halted
  rather than through the accessor; the WFI caveat says halt before any
  read since blink sleeps in WFI, and DBG_STOP is told apart as the Stop
  modes' matter. vscode.md's debug section carries the same two facts
  in one paragraph.
- GUI coverage: vscode.md's last section says what the configuration
  covers and that the bench operations are documented through the
  command line today, without deciding their future.

### Editorial refinements

- VS Code as a route: docs/learn/README.md separates the ideas (in
  order) from the two routes (alternatives), cpp-notation.md as a
  companion; first-program.md links both routes at its top; docs/
  README.md's learn rows say the same.
- Verification notes: command-line.md has one status table at the top
  (every step's status, the docs/README.md rule kept) and no status
  paragraph in the steps; the DBG_STOP explanation is one short
  paragraph with a link in step 3 and a sentence in step 5. vscode.md
  keeps the matrix, folds the recorded/expected split of the debug step
  into one sentence, and reduces the DBG_STOP text to a note and a link
  in the flash and debug sections.
- A readable internal comment: event_queue.hpp's implementation block
  rewritten (ordinary headings; slot, position and cursor defined
  before use; depth stated once; the runs, the cursor ownership, the
  fence ordering, the release rule and the one-core assumption kept;
  the queue diagram cited). borrowed.hpp drops the planned epoch check
  from the API description; post.hpp defines "thunk" where ReplyTo
  uses it.
- Navigation summaries: docs/README.md's map rows shortened to one
  purpose each, every link kept, the rules section untouched;
  README.md's RP2350 row says "every driver's test suite passing on
  both" instead of "every chapter green on both".

### The author's local findings

- README.md's excerpt is two blocks copied from blink.cpp as it stands
  (every line of every excerpt in README.md and first-program.md
  checked by a script against stm32g0/src/apps/blink.cpp: all present).
- panic.hpp: a ResetReporter is in the reset header of the strata that
  have one; the avrxt and ch32x035 strata have none (grep: samc21,
  stm32g0, stm32f4, ch32v00x, ch32vx03, rp2040, rp2350).
- command-line.md step 2 explains the three sections and gives one
  debug build's figures (.text 6244, .data 48, .bss 76) as that build's.
- The variant slot: 4 bytes, sizeof(SetPeriod) 2, as above.

### Checks

- `bin/brio prose README.md docs/README.md docs/learn/*.md
  docs/host/README.md brio/kernel/*.hpp`: 19 files, 0 errors, 0 for
  review (before and after).
- `bin/brio gate --tokens --against 4d20f8f brio/kernel/*.hpp`: all ten
  headers token-identical (lexed with the system g++ 13.3).
- Links and anchors of the touched pages checked by a script: every
  relative link and heading anchor resolves; the only hits are lambda
  notation (`[](Toggle)`) read as links, the parser false positive the
  review noted.
- Not run, no toolchain: any build, the host tests, the full
  `brio gate`.

### Reader review of the revision (fresh subagents, two rounds)

Three readers per round, each given only its files and the questions:
the concept route (README, introduction, first-program, cpp-notation)
with the brief's questions plus the review's scenarios; the two
workflow guides with the fresh-checkout scenarios; and a maintainer
profile on event_queue.hpp's and time_event.hpp's comments.

Round 1, concept route. Every scenario answered "supported directly":
the older event in the same queue first; a still-higher queue served
first and starvation possible; the full queue (dropped, counted,
nothing blocks); the one-tick timer maturing in under a tick; no
response-time promise; two interrupt handlers posting to one queue
safe under the no-nesting rule. The pointer-payload reuse scenario was
answerable only through the kernel.md link, as intended. Residue
fixed: the timer bound was stated in milliseconds although the same
stage names the AVR's 1024 Hz tick (now tick intervals); the kernel's
re-scheduling of a periodic timer and a program's `arm_every` on a
running one read as the same "re-arm" with opposite reference points
(now told apart); the catch-all was praised for exhaustiveness while
silently taking a forgotten listed event (now stated in first-program
and cpp-notation); the stage-4 scenario implied a Toggle per tick (now
says its tick is the heartbeat's); `post` returning nothing on a drop,
`arm` beside `arm_every`, the mask in post being against the other
producers, sizing by the depth and not the spare slot, and the
verification sentence naming the G071RB where the route is the G0B1RE
(now says what the record says). Left as is: the includes and the file
skeleton of a new app (the page points at the full file), `print`'s
exact stall (the header's).

Round 1, workflow guides. Every scenario "supported directly" (the
clangd databases, the host-test sequence on an unbuilt tree, the wrong
folder's symptoms, the queue's nonzero cursors, the WFI read). Residue
fixed: the two once-only configures lived in the clangd section while
the numbered procedure said "debug preset" alone (now inside step 2 of
the procedure as well); leaving `blink-upload` selected reflashes at
every build (now said); the root `.clangd` was not named (now is, as
rules without a database); `plugdev` membership; no recovery for step
5 (added); the DBG_STOP paragraph still long (now two lines and a
link). Observed and kept: the exact CMake Tools labels remain to be
confirmed, no step is on record from a fresh checkout.

Round 1, the queue comment. All eight answers "supported directly",
two with an inference the comment did not spell: why a compiler-only
fence suffices (now stated: the other side is on the same core and a
core sees its own stores in program order), and the counts and the
`fence_` name (now tied to "the free end" in the run table). One real
contradiction found and fixed: the outstanding run was "read by
nobody" while hold() reads its last position. Also fixed: "the
consumer's verbs" carry interleave points where only take() and
release() do (platform.hpp's pointer corrected too); "one writer" per
cursor where alloc_ has one side and two contexts (now "one side",
with the critical section's purpose, producer against producer,
stated); `held` and serve_one() before their definitions; exactly
2^31 uncovered (now "at most 2^31"). Noted and kept: the depth rule
also appears on push()'s and capacity()'s own comments (function
comments, not the model).

Round 2 (fresh readers on the revised pages, the review's scenarios
again). Concept route: all nine scenarios "supported directly" (the
older event first, the higher queue and starvation, the burst of five
against a depth of four with the sizing rule and the silent sender,
the three-tick timer's bounds in tick intervals, the two re-arms, the
forgotten alternative, no response-time promise, a waiting handler, the
stage-4 tick); what it could not establish is what the pages defer to
time_event.hpp by name (the catch-up, the exact phase). Residue fixed:
stage 3's "re-arms it one period later" (now "one period after the one
that passed"); "holds four waiting events" before the rule (now "a
depth of four"); "bounds"/"promise" for what is an order and no
duration (reworded in both pages); README's "interrupt handlers do one
thing: post", which primed a Toggle per tick (now "may do", and the
tick only counts); the AVR's tick interval named. Workflow guides: all
seven scenarios "supported directly"; residue fixed: the release
configure was appended after the debug instruction it must precede and
the host configure was outside the procedure (both now stated before
the numbered steps); the host-test parenthesis read as if the build
button ran the suites (reworded); "configured" defined differently on
the two pages (defined once on the command-line page as on the VS Code
page); the VS Code page's caveats repeated after the table (three
removed). The queue comment: no contradiction with the code left; the
eight answers "supported directly" but the exact-2^31 boundary ("within
half the range" read as exclusive; now "2^31 included"), hold()'s
absence from the interleave-point sentence (added), a misplaced hold()'s
handle being the double release release() warns of (connected), and
why overflows() takes the critical section (said: sixteen bits, two
accesses on an 8-bit core). Noted and kept: `take_` written plainly
where the other cursors go through store_shared (it publishes nothing
to a producer), the AO terms assumed by the header (the kernel pages
define them).

## Second batch: the design pages

Scope: every page of `docs/design/` (twenty Markdown files; the two
SVGs untouched), rewritten for the reader the brief names, in the
voice of the first batch, every decision, contract and measurement
kept. Base: `docs-rewrite` at
8ba6ab5dc4af50110483f830b6cee7b85c90f2f9, the reference of every token
check. Branch `claude/docs-design-batch-2-i1j28h`, one pull request
against `docs-rewrite`. Not touched: every source file, `// build:`
lines, CMake, `.clangd`, `.vscode`, CLAUDE.md, docs/README.md, the
learning pages, the target folders. No build and no host test was run
(no cross toolchain and no `/sw` in the cloud); the token check lexed
with the system g++ 13.3.

### Each page, and what changed

Every page now opens with what it decides and for whom, states its
smallest use and the prerequisite that decides correctness, defines
the project's terms where it first uses them (active object,
run-to-completion, contract, lease, monostate, stratum, dispatch), and
uses no capitals for emphasis; every link names its destination;
every "section N" cross-reference inside the batch became a link by
heading text. The realizations tables are kept whole (two blank lines
had cut each of eleven tables in two, so the rows after the cut
rendered as plain text) and their rows follow the fixed order
overview.md now states: avrxt, samc21, stm32g0, stm32f4, ch32v00x,
ch32vx03, ch32x035, rp2040, rp2350, host.

- overview.md: the stale opening (one testbed, the next target
  decided) replaced by today's truth; the strata table lists every
  stratum of `brio/`; the stability hierarchy stated; the realizations
  index links kernel.md by heading; the pins and PWM tables whole.
- kernel.md: the thirteen numbered headings kept word for word (every
  anchor and every "section N" reference inside and outside the batch
  still resolves); the reading guide names its parts; the overflow
  knob corrected (below); the timer limits aligned with
  time_event.hpp; the two-core section names both RP realizations.
- runtime.md, serial.md, spi-bus.md, i2c-bus.md, power.md, nv-heap.md,
  nv-journal.md, gfx.md (the pages the learning route links): the
  orientation added, the prose rewritten, the realization accounts of
  power.md turned from bold paragraphs into prose under the table; the
  bus pages' stale pointer to "the top-level README bench map" now
  names the board pages and the app's header comment; the "not
  bench-verified" items of the NV pages say what would measure them.
- clock.md, ring.md, dma.md, can.md, analog.md, meters.md, usb.md,
  block-stream.md, simulation.md, benchmark.md: the same pass;
  clock.md's reference table lists the util entities and the
  families' headers instead of four families; benchmark.md's tables
  are untouched measurements, its parenthesized figures explained
  once as the image before an engine's rework, and the names of
  review rounds dropped from its prose.

### kernel.md's references

The numbering is kept, so nothing outside the batch had to move. Inside
the batch the references that said "section N" (overview.md's index
rows, power.md, runtime.md, serial.md, spi-bus.md, ring.md) are links
by heading text. The seventeen Markdown pages and nineteen sources of
the inventory above still point at valid numbers; the sources were not
edited, and `bin/brio gate --tokens --against 8ba6ab5` on the kernel
headers, util/bench.hpp and rp2350/multicore.hpp reports every one
token-identical (trivially: no source changed).

### Facts resolved while writing

- kernel.md claimed a kernel-level overflow knob (`count` in release,
  `panic` in debug). event_queue.hpp leaves that to the layer above
  and no such knob exists; `PanicCode::queue_overflow` is a code a
  program may use. The page now says what the code does.
- kernel.md said `ticks_from_ms` has "at least this long" semantics
  (a timeout never fires early); the page now carries the maturity
  window of time_event.hpp and the strict half-range limit.
- kernel.md said `interleave_point()` is called by "the queue's
  consumer verbs"; it is called by `take()` and `release()`.
- overview.md's `EventSystem<Route...>` is "not built" per
  avrxt/evsys.hpp (planned as sugar); said so.
- power.md's deadline-guard default was "measured on the first
  target (a crystal's 1.77 ms restart)"; the prose net refuses an
  ordinal silicon, docs/avrxt/platform.md gives about 1.5 ms for the
  crystal's restart on the AVR128DB48 board, and neither figure is in
  power.hpp. The page names the board and says "under 2 ms" with a
  link to the AVR page. The 1.77 figure's origin is open.
- serial.md's "worst case about 2 ms at 460800 baud" for a print's
  wait is underivable from the stated defaults (a full 256-byte
  transmit ring is 5.5 ms at that rate); replaced by the derivation
  (one byte time per byte that does not yet fit).
- kernel.md's queue cost paragraph explained a "`depth + 2` bytes
  more" RAM figure as "the ring of numbers and the hold flag"; the
  ring alone is `depth + 2` positions, so the parenthesis cannot be
  right as written and was dropped; the figure itself is kept as the
  author's and is open.
- gfx.md's smallest use names `clear`, `line`, `fill_rect` and
  `text_field` as the headers spell them.

### Reader review (fresh subagents)

Round 1, three readers, each given only its files and the questions
(the brief's four, the reviews' scenarios, and a per-page list). Every
scenario was answered "supported directly" on overview.md (the
governing rule, the run as the contract's unit, the chapter rule, the
realizations tables, the hot-path rules, the traps), on kernel.md (the
interrupt posting to a high-priority object during a low-priority
dispatch, with the older event and the still-higher queue ahead of it
and starvation possible; the three leases and who enforces each; the
depth-4 queue's counts, the overflow with no panic, the double
release; the one-tick timer's window and the 2^31 bound; the two idle
promises and no nesting; two kernels on two cores) and on serial.md,
power.md and nv-journal.md (the sink's place and lifetime, print's
wait, the line guarantee; who sleeps and what the manager does, the
three request outcomes; the panic reserve, mount, the current value
and a cut mid-collection). Residue, all applied: overview.md's active
object and cooperative undefined, "the two loans" and "the loan made
physical" unexplained, "binding" in two senses, "pending wave", the
cortexm row's include direction, test letter and verdict prose
undefined, the families named only as "the two ARMv6-M strata" and
"the sister family"; kernel.md's critical section and fold expression
undefined at first use, QV unexpanded, slot and hold used in the
contract before their section, the dispatch loan's sender unnamed, the
order check's "why" not joined to the scan rule, the contexts of
publish, send and hold() unstated, the idle_until sentences reading as
contradictory, the CH32V003's late edge not carved out of the no-lost-
wake promise, the RAM parenthesis above; serial.md's saved-pointer
consequence and compile-time refusal unstated, the 2 ms bound
underived, RxActivity used from two senders before being defined, a
fragment delivered as a line against "every line delivered is a line
sent"; power.md's SleepRequested{none} rule decided per target, rung,
witness and resync undefined before the table, the guard refusal's
reply unstated; nv-journal.md's "map pair's atomicity argument"
pointing off-page, a second save_reserved() unsaid, the geometry
notation undefined. Noted and kept: the realization tables remain
dense for a first-time reader (they are the cross-target reference
the rules prescribe), and `Tenuto`, `Lease` and `Borrowed` are
defined on kernel.md, which every page links.

Round 2 (two fresh readers on the revised overview.md and kernel.md,
the same questions plus the first round's scenarios). Every scenario
"supported directly": the cooperative definition, the two loans, the
cortexm include order, the hot-path rule's per-operation writes, the
suite-size rule, rt/ on the AVR; the three-way order of service, the
dispatch loan's sender and the why of the order check, the contexts
of post, publish, send, hold and arm, the depth-4 counts, idle_until's
contract, the timer bounds, QV. Residue, applied: overview.md's "a
handler" in the cooperative definition, the two loans only a
parenthesis, "reads CMSIS-Core symbols only" against "may include
util/", the per-operation "address", test letter and bench chip used
before defined, engine, concept, traits type and preset undefined, the
count "ten targets" against nine families and the host; kernel.md's
Tenuto named only in its own section, whether an interrupt handler
writes a lent buffer (it writes the driver's ring; the active object
fills the lent buffer in its own dispatch), publish's "any context"
scoped to the core, the idle promise absolute in the scheduler and
holed in the platform (now cross-referenced), stratum, rung, sleep
site, event latch and Tickless undefined. Left as is: the project
objects the examples cite (SerialPort, the bus arbiter, MeterLatch),
defined on their own pages; the realization tables' density; the
index entries (CoreAware, through) with no body text, which name
header entities the index exists to map.

### Checks

- `bin/brio prose docs/design/*.md`: 20 files, 0 errors, 1 for review
  (clock.md: "not built yet", the power-manager active object that
  asks the bus objects before a rate change, a true claim of absence).
- Links and anchors of every page of docs/design, docs/learn and the
  two READMEs, checked by a script with GitHub's anchor rules: every
  relative link and heading anchor resolves (0 problems).
- `git diff --stat 8ba6ab5 -- . ':!docs/design'`: empty; no source,
  build or configuration file changed.
- Not run, no toolchain: any build, the host tests, the full
  `brio gate`.

### Local verification handoff

- The full `brio gate` against 8ba6ab5 confirms every image unmoved
  (no source changed, so it should report every preset identical).
- Two figures to settle on the desk: the deadline guard's default
  (power.md: the AVR128DB48 crystal restart, 1.77 ms in the old page
  against about 1.5 ms on the AVR platform page) and the queue's
  "`depth + 2` bytes more" RAM accounting (kernel.md's cost
  paragraph), to be read off an image's map.
- The realizations tables carry hardware findings this pass could only
  lowercase and link, not re-measure; a re-read of each row against
  its family's page by the desk is the check this batch cannot make.

## Next batches (not started)

1. One representative target stratum (stm32g0: README, selected
   peripheral pages, header comments).
2. The remaining target strata in groups.
3. Remaining services (serial_port.hpp and print.hpp first), devices,
   host, boards and probes.
4. Before the public merge: remove DOCUMENTATION_REWRITE_BRIEF.md,
   DOCUMENTATION_REWRITE_PROGRESS.md and the two review files, unless
   the user keeps them.
