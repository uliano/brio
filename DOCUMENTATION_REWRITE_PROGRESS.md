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

## Third batch: the stm32g0 stratum

Scope: the model target stratum, as the pattern the other target
strata will follow: `docs/stm32g0/README.md` rewritten; the
twenty-two peripheral documents of `docs/stm32g0/` in the brief's
additive pass (the reader's orientation added, local fixes only); the
comments of all thirty-one headers of `brio/stm32g0/` (openings and
public operations, token identity on every one). Base: `docs-rewrite`
at 240c14e4f76e43ee62d463c2d526b71d2cb83af2, the reference of every
token check. Branch `claude/stm32g0-stratum-docs-m008or`, eleven
commits (the README; the route's six documents; the other sixteen;
blink's lines quoted; the readers' residue; the headers in five
groups plus the two pattern headers), one pull request against
`docs-rewrite`. Not touched: every code token, `// build:` and
`// mcu:` lines (none exist in the headers), apps, CMake, presets, ld,
svd, `.vscode`, `.clangd`, the other target strata, the design pages
and the learning route (no link of theirs broke: the README kept the
section names they cite, "Toolchain", "Upload", "Debugging", and
nothing links it by anchor), CLAUDE.md and docs/README.md.

Environment: no cross toolchain and no `/sw` in the cloud; the token
check lexed with the system g++ 13.3; the clone was shallow and was
unshallowed before the first check. No build and no host test was run.
`docs/stm32g0/vendor/README.md` is unchanged in content (the prose net
ran over it; it needed nothing).

### docs/stm32g0/README.md

Rewritten whole: the orientation (what the stratum is, for whom, the
learning route pointed at rather than repeated), the three boards in
one table (part, memory, LED, console, die identity) with the
bench-verified facts of the G0B1RE kept word for word, the toolchain,
the build (the six presets and the G031K8's commands, the build lines,
the outputs per preset, the bench tool with its three board types),
the upload with the two probe caveats (the WFI read and DBG_STOP, with
"HLA" expanded and one tick spelled as a millisecond), the debugger,
the editor, the console, the family coverage (the reserve defined for
a reader meeting the word, the two spellings of an absence, the sweep
without the stale count), the two smaller parts with their suite
tables whole (self-link, peer, letter and all-key defined before
them), and the document map at one line per document. Before a map
cell was shortened, every number and every backticked identifier of
the old cell was checked for presence in its document by a script; the
exceptions are listed under the open facts.

### The peripheral documents

Each of the twenty-two opens with: whom the document is for and what
the driver does in a program; the smallest use, an excerpt of the
document's own "How to use it" block (platform.md's quoted from
blink.cpp as it stands); the prerequisite that decides correctness even
there; and, on the long pages, a reading guide. "Documents of record"
became a heading. Local fixes: the reserve, a suite's letter and the
all-key `z`, a monostate, a pull-walk, an active object and a task
defined at first use in each document that uses the word; kernel.md
linked by heading instead of "section 11"; a path in prose turned into
a link; insistence capitals lowered in the sections the reader meets
first. Nothing reordered, no finding, figure, gap list or realization
cut; the bench findings are untouched. Four orientation sentences
that overstated what the code enforces were corrected against the
headers before the commit (the DAC's data order, one engine per DMA
channel as a transport's check, the IRTIM claim order, the plain sleep
site's restriction as stated and not checked).

### The headers

Comments only, every one of the thirty-one token-identical to the base
(`bin/brio gate --tokens --against 240c14e brio/stm32g0/*.hpp`).
platform.hpp and pin.hpp were rewritten first, as the pattern; the
other twenty-nine were done by five fresh agents under one written
brief (keep every fact, token identity, the opening's order: what the
caller gets, which context calls what, then the why), each reporting
what it changed and what it could not establish; their diffs were
spot-read (spi.hpp, crc.hpp, lptim_ticker.hpp, device_tables.hpp) and
their reports checked. Each opening now says first what the component
does for a caller, which context calls what (what waits, what masks,
who owns a buffer until when), then the facts the old comment gave;
the public operations the documents' smallest uses call gained the
sentence they lacked. Stale claims corrected to the code's truth: a
controller `Dma<n>::init()` that does not exist (spi.hpp and i2c.hpp:
an engine's `arm()` opens the clock); a `DmaMux::synchronize()` that
does not exist (dma.hpp: the two request verbs named); verbs named by
a spelling they do not have (adc.hpp's `temperature_c`,
`conversion_cycles`; dac.hpp's `set(ch, code)`); an erratum's
statement located on `result()` where it is; rtc.hpp's tamper block
described as read-only and RTC_REFIN as unmeasured where the code arms
tampers and docs/stm32g0/rtc.md measures both; exti.hpp's two claims
of absence (no PWR driver, nothing enters Standby) against
stm32g0/pwr.hpp and sleep.hpp; usart.hpp's UartOptions as the "last"
template argument (Placement follows it) and "ES0548 rev Z" (Rev 3 is
the document's revision, Z the silicon's); lse_tracker.hpp's `hz()`
described as the code has it. Two detached doc comments in rtc.hpp and
dac.hpp moved onto the declarations they describe. clock.hpp fell
between the five groups and was done last, by hand: its opening now
leads with the smallest use and the contexts (init first in main and
why a driver initialized before it is wrong by the ratio of the rates;
set() from a handler or main with its bounded waits, restore() the one
verb legal from an interrupt handler, the Rcc verbs unguarded), the
rest kept with capitals lowered; set<hz>() and set(hz) say their
context and what false means.

### Facts resolved while writing

- The README's "22 positive TUs x 12 headers": `test/family_stm32g0/`
  holds 28 positive files today; the page states the rule and no count.
- "Fourteen of the seventeen suites run" on the G071RB and the G031K8:
  seventeen `test_stm32_*` apps exist; the build lines give `nvm`,
  `journal` and `fdcan` to the G0B1RE alone and `spi` to all three, so
  the page says fourteen build for each; the SPI row's "-" on those
  boards is a count not recorded (its peer letters want a paired
  board), not a suite that does not build.
- The old spi.md cell's "a 3-byte command at 399 cycles above the wire
  instead of 998": spi.md's own table reads 413 (`InFlash`) and 381
  (`InRam`) against 998; the 399 is in no document and its origin is
  open; the table stands.
- The old i2c.md cell's readback "pinned against every cell of tables
  172..174": i2c.md names tables 171 and 176 only; the claim is open.
  Its "478 kHz" is the document's 477.6 kHz.
- `test_stm32_serial` on the G0B1RE is 120 verdicts in one README table
  and 118 in the other, and usart.md's `t` count is 171 where the table
  says 174; `test_stm32_tim` is 128 in the tables where tim.md's own
  text says "the G0B1RE's 118". Kept as the desk's figures; open.
- The plain sleep site's "no Stop with an armed time event" is a stated
  restriction (sleep.hpp), not a check; pwr.md's orientation says so.
- `Adc::select()` waits for the CCRDY handshake, bounded;
  `select_sync()` is the same with an answer (adc.hpp).
- serial.md says `write_byte` does not mask interrupts in one place and,
  of the STM32G0, that its byte path masks around the register write in
  another (lines 37 and 76): found by a reader, outside this batch's
  files, open for the services batch.

### Reader review (fresh subagents, one round)

Two readers, each given only its files and the questions: one on the
README, port.md and pin.hpp with a Nucleo-G031K8 scenario; one on
usart.md, platform.hpp and the README's Upload and Debugging sections.
Every scenario was answered "supported directly": the G031K8's LED
fragment and what the angle brackets mean, the four GPIO traps (a
toggle from an interrupt beside a configuring call, PA8 under its
pull-up after a power-on, a pad not handed over, a first store to an
unclocked port), the 1 ms Stop after a flash, the console line by line,
the unbound vector, the multi-board handler, a 4000-byte print in a
handler, the receive slack on the G0B1RE against the G031K8, the
platform's contract and the pending interrupt between the empty check
and the WFI, the WFI read and C_DEBUGEN against DBG_STOP. Residue, all
applied: the README lacked the G031K8's preset in the commands, the
output directory per preset, the bench tool's three board types, the
literal "no device connected", and self-link and peer before the suite
tables; port.md and pin.hpp named the active object without defining
it and port.md said "left in output mode" for a pad never handed over;
usart.md's orientation left the tag object, `extern "C"`, `clock`,
`crlf` and "task" unexplained, did not spell how the multi-board
handler name is written, what a port initialized before the clock task
does (a quarter of the rate, nothing refuses it), what `isr()` returns,
or that the transmit ring has one producer and both write verbs copy.
Left as is, with the reason: the C++ notation itself (`using`,
`constexpr`, templates) is the learning route's and the reference pages
point there; a complete minimal program is first-program.md's. No
second round: nothing the first found was broken in substance.

### Checks

- `bin/brio prose brio/stm32g0/*.hpp docs/stm32g0/*.md
  docs/stm32g0/vendor/README.md`: 55 files, 0 errors, 0 for review
  (before the batch: 0 errors, 1 for review on exti.hpp, a claim of
  absence that was false and is gone).
- `bin/brio gate --tokens --against 240c14e brio/stm32g0/*.hpp`: all
  thirty-one token-identical (lexed with the system g++ 13.3).
- Links and anchors of every page of docs/stm32g0, the learning pages
  and the two READMEs, by the script with GitHub's anchor rules: 0
  problems on the batch's pages; the three hits are the learning
  pages' lambda notation read as links, the known false positive.
- `grep -P '[^\x00-\x7F]'` over the touched files: nothing; no
  `// build:` or `// mcu:` line in any header.
- `git diff --stat 240c14e -- . ':!docs/stm32g0' ':!brio/stm32g0'`:
  empty; no source outside the stratum, no build or configuration file
  changed.
- Not run, no toolchain: any build, `brio check stm32g0`, the host
  tests, the full `brio gate`.

### Local verification handoff

- The full `brio gate` against 240c14e on a machine with the toolchains
  confirms every image unmoved (comments only; the token check says
  so, the gate proves it).
- `brio check stm32g0`, for the family fixtures' sake (nothing in them
  changed).
- The open figures above, to settle from the desk's records: the SPI
  request's 399 against the table's 413/381, the I2C readback's table
  numbers, the serial and tim suite counts.
- The excerpts in the orientations are copies of each document's own
  "How to use it" block; a re-read of each pair after any API change is
  the maintained-example rule applied to them.

## Fourth batch: the route's services

Scope: eight headers of `brio/util/`, comments only, the services the
learning route and the design pages send a reader to:
`serial_port.hpp`, `print.hpp`, `stream.hpp`, `bench.hpp`, `ring.hpp`,
`bus_master.hpp`, `spi_bus.hpp`, `i2c_bus.hpp`. Base: `docs-rewrite`
at 66e5bbcfbeea90b4001007ebd1aa5f42282e0afa, the reference of every
token check. Branch `claude/util-header-comments-sgjxs0`, one pull
request against `docs-rewrite`. Not touched: every code token (the
token check on every file), every other header of `brio/util/`, every
other stratum, the design pages and the learning route (no link of
theirs broke: `docs/learn/introduction.md` cites `util/print.hpp`'s
"Delivery policy" by that phrase and the phrase is kept), CLAUDE.md,
docs/README.md, apps, CMake, `.vscode`, `.clangd`. No `// build:` or
other tool-read line exists in these headers.

Environment: no cross toolchain and no `/sw` in the cloud; the token
check lexed with the system g++ 13.3; the clone was shallow and was
unshallowed before the first check. No build and no host test was run.

### Each file, and what changed

Each opening now says first what the service does for a caller and
its smallest use (an excerpt in the shape the apps spell it), then
which context calls what (what waits, what masks, who owns a buffer
until when), then every fact the old comment gave with its citation
(a design page by name and heading, a measurement by its suite
letter). Capitals used for emphasis are lowered; a term of the
project's own (run, epoch, monostate, tenure, lease, NACK) is defined
where the header first uses it. Internal comments were touched only to
lower the same capitals in the two flavour blocks of `ring.hpp`.

- `serial_port.hpp`: the opening leads with what SerialPort delivers
  (lines as events), the console as the smallest use (the SerialPort
  alias, the Tenuto list with the sink first, the interrupt glue
  posting RxActivity, as `stm32g0/src/apps/console.cpp` spells them),
  then the three contexts (the transport's handler, SerialPort's
  dispatch, the sink's dispatch), the loan and its lifetime in plain
  words (read and write during the LineReceived dispatch, never kept),
  the list order and why it is enough, then the transmit paragraph
  with the wait derived (below) and every run, release, skip and
  silence paragraph kept whole. RxActivity, LineReceived, the queue
  depth, LendsTo, init(), line_overflows() and torn_lines() say what
  the caller must know (a line holds up to max_line - 1 characters and
  a NUL, `util/proto/line_parser.hpp`).
- `print.hpp`: the opening leads with the one function and the
  smallest use, keeps the argument types and the extension point
  (ADL expanded), and rewrites the delivery policy around the
  contexts: main context only, why a print from an interrupt handler
  spins for ever (the handler is the ring's consumer and no handler
  nests over another, kernel.md section 1), the wait derived, the two
  consequences (every other active object held; a print before init
  and interrupts spins for ever), then the run mechanics and the
  conversions as before. write_blocking(), print() and print_scan_run's
  paragraph carry their context and the copy of the caller's text.
- `stream.hpp`: the opening says whom the concepts serve (a program
  through the services, a driver author directly), defines concept
  and try semantics, states the context rule as the transports state
  it and that nothing checks it, keeps the run paragraph and cites its
  measured cost; every concept's comment says whether its verb waits
  (none does) and who owns the buffer (the caller's run is copied).
- `bench.hpp`: the opening leads with what the instrument measures and
  the smallest use as `samc21/src/apps/bench_samc.cpp` spells it
  (BenchIdle, IsrMeter, Stopwatch, bench_counters, bench_sample,
  bench_line), the contexts of each (main, the bound vector, the
  kernel loop), then the line's fields, the ruler, busy, the
  instrument's cost and the quiescent read as before; Stopwatch,
  IsrMeter and bench_line() say their context.
- `ring.hpp`: the opening leads with what Ring is and the smallest use
  (a transport's transmit ring), states the one rule with which side
  is which in a transmit and a receive ring and that two producers
  break it silently (nothing checks it), then the lock-free and guarded
  paths, the fence, the no-doubling rule, the two granularities, the
  capacity and the two flavours as before; push(), pop(), read_span(),
  write_span() and clear() say their side and whether they wait. The
  SkipRing and HardwareRing blocks keep every sentence with their
  capitals lowered and gain a link to their ring.md sections.
- `bus_master.hpp`: the opening leads with what the arbiter does for a
  client (one request, one reply, transactions in arrival order),
  points at the two alias headers for the smallest use, lists who
  calls what (the client, the interrupt glue, the power manager, the
  arbiter's own dispatch, what waits where), states the buffer rule
  as a lifetime (untouched from the post to the BusDone, race-free by
  run-to-completion, checked by nothing), then every paragraph of the
  old opening (the hold, the engine contract, the policy, the timeout,
  the race, the envelope, the authority) kept whole with the
  envelope guideline cited to kernel.md. BusDone, TransferDone,
  BusTimeout, the class comment (pending_depth defined, its bound of
  124 from the static assertion), init(), rejected_count() and
  stale_events() say what the caller must know.
- `spi_bus.hpp`, `i2c_bus.hpp`: each opening says what the alias is
  for a program and shows the smallest use (the host, the bus, the
  post with reply_to, in `stm32g0/spi.hpp`'s and `stm32g0/i2c.hpp`'s
  spelling), points at bus_master.hpp for the contract and at the
  design page for the stack; the alias comments define pending_depth
  and timeout_ticks for a reader who meets them there; NACK and tenure
  defined in i2c_bus.hpp.

### Facts resolved while writing

- The "~2 ms worst case at 460800" of serial_port.hpp's transmit
  paragraph: no code, document or measurement in the repository
  supports it (a grep for the figure finds only unrelated 2 ms
  findings in the target documents). The header now says what
  serial.md derives: zero while the message fits the ring, otherwise
  one byte time per byte that does not yet fit, a byte being ten bit
  times (21.7 us at 460800, 87 us at 115200), so a full 256-byte ring
  is about 5.5 ms at 460800. print.hpp carries the same derivation.
- A line holds up to max_line - 1 characters: `LineAssembler::push`
  overflows when the max_line-th character arrives (line_parser.hpp),
  so "lines longer than max_line" was off by one; the header says
  "did not fit the buffer (max_line - 1 characters)".
- Whether print() may be called from an interrupt handler was stated
  nowhere; the headers now say main context only, with the reason
  (the transport's handler is the ring's consumer; no handler nests
  over another on any target, kernel.md section 1).
- serial.md's two masking sentences (lines 37 and 76, found by the
  third batch): not a contradiction. Line 37 is about the rings (a
  lock-free ring masks nothing); line 76 is about the STM32G0's
  transmit arming, which reads, modifies and writes a control
  register under a guard (stm32g0/usart.hpp's opening says so). The
  page's line 37 still reads as a claim about `write_byte` whole;
  outside this batch's files, open for the page's owner.
- bench.hpp's reference to "design/kernel.md section 1" is kept as a
  number (the inventory above lists it) and now carries the heading
  text too.

### Reader review (fresh subagents, one round)

Two readers, each given only its files, the audience profile and the
questions: one on serial_port.hpp and ring.hpp, one on print.hpp and
bus_master.hpp, with eight and nine scenarios. Supported directly: the
wrong list order refused at compile time and the loan's end at the
sink's return; the saved pointer as a contract violation; the sink's
stall under a long print and who is held; the two-lines-in-flight
stop and the self-post; the lock-free call from either context and
the guarded path at a 512-byte ring on an 8-bit core; clear()'s
condition on all three rings; HardwareRing's count() from an
interrupt; the print before init; the print from a receive handler
and why it hangs; the 1000-byte print's wait and its formula; the
float's default shape and fixed(); the fifth request's rejection; the
buffer rule until the BusDone; the untimed bus wedged by a lost
completion and the timed bus's recovery; the sleep vote in flight.
Residue, all applied: ring.hpp's "a print from a handler and one from
the loop" read as two active objects' prints (now: an interrupt
handler's beside the loop's, the objects' prints being one producer);
two conditions for clear() in one file (now one: one party calls it
while the other is quiescent); "valid until its own side's next
operation" leaving count() in doubt (now: until the side next moves
its index; a look ends nothing) and stated as general where
HardwareRing's run can be written over (now said at the rule); SkipRing's
producer rule against "always safe from either context" (now: a
receive ring's shape, a ring the main loop produces is a Ring);
serial_port.hpp's "the buffer is reused by the next line" against the
ping-pong (now: the next dispatch may assemble into it again, the
buffers alternate), "nothing here waits" against print's wait (now:
SerialPort itself waits for nothing; print is the one wait), the
receive ring called lock-free where a wide index is guarded (dropped),
quiet_ticks as 100 ms where a slow tick makes it one tick (said),
priority never defined (now: list position), whether print may be
called from an interrupt handler (now: main context only, print.hpp
says why), the sink's queue needing room for two LineReceived and a
dropped one being a lost line (said), a byte dropped under a transport
that reports no gaps joining two fragments (said); print.hpp's "empty
tag instance of its all-static class" (now defined), the early print
stated as a hang whether or not the text fits (now: a print that does
not fit the room before init spins; a shorter one's bytes are the
transport's business), "never from an interrupt handler" given one
reason where there are two (now: the one-producer rule and the drain
that cannot run), the ten bit times without 8N1 (said), write_byte's
false unstated (said); bus_master.hpp's three counts of one budget
(now: the 2 x pending_depth + 2 outstanding requests itemized against
the queue's depth), "never silent" against the queue's drop past the
budget (now scoped to the budget), "answered at once" against "the
reply comes later" (now: sent in the receiving dispatch, delivered as
an event), post() from the interrupt glue with no word on its context
(now: legal from any context, kernel/post.hpp), the timeout armed per
wire transfer and so per retry (said). Noted and kept: the C++
notation itself (templates, std::optional, std::span, lambdas,
`static inline`, Fsm, Subscribers, Borrowed's accessor) is the
learning route's and the kernel headers', which the comments point
at; the engine's freedom to copy or read the lent request is a stated
open point of the contract, not a gap of the comment. No second
round, by the batch's cost rule.

### Checks

- `bin/brio prose` over the eight files: 8 files, 0 errors, 0 for
  review (before and after).
- `bin/brio gate --tokens --against 66e5bbc` over the eight files: all
  eight token-identical (lexed with the system g++ 13.3).
- `grep -P '[^\x00-\x7F]'` over the eight files: nothing.
- The test directories the headers cite (`test/test_serial_port`,
  `test/test_print`, `test/test_ring`, `test/test_bus_master`) exist;
  the app files the smallest uses are drawn from
  (`stm32g0/src/apps/console.cpp`, `samc21/src/apps/bench_samc.cpp`,
  `stm32g0/src/apps/test_stm32_spi.cpp`, `test_stm32_i2c.cpp`) were
  read for the spelling.
- Not run, no toolchain: any build, the host tests, the full
  `brio gate`.

### Local verification handoff

- The full `brio gate` against 66e5bbc on a machine with the
  toolchains confirms every image unmoved (comments only; the token
  check says so, the gate proves it).
- The host tests, for the suites the headers cite (nothing in them
  changed).
- serial.md line 37 ("neither the handlers nor `write_byte` mask
  interrupts") against the STM32G0's byte path, to be reworded by
  whoever next edits the page.

## Fifth batch: the entry pages

Scope, capped: every page of `docs/boards/` and `docs/probes/` (their
README files included), `docs/host/README.md`,
`docs/cortexm/README.md`, and the operational README of each remaining
target stratum (`docs/avrxt/`, `docs/samc21/`, `docs/stm32f4/`,
`docs/ch32v00x/`, `docs/ch32vx03/`, `docs/ch32x035/`, `docs/rp2040/`,
`docs/rp2350/`): thirty-five Markdown files, nothing else. Base:
`docs-rewrite` at acfb48515662b239196f5d26ae11a3b2da745d60. Branch
`claude/docs-rewrite-batch-5-fhw1ie`, five commits (boards and probes;
host and cortexm; the READMEs in three groups), one pull request
against `docs-rewrite`. Not touched: every header, every peripheral
document, the vendor pages, the design pages, the learning route and
the stm32g0 pages (no link of theirs broke: the only heading they cite
by name in this batch's files, "How a board joins the bench" of
docs/boards/README.md, is kept), code, CMake, `.vscode`, `.clangd`,
CLAUDE.md, docs/README.md. No build and no host test was run (no cross
toolchain in the cloud, and no header changed, so no token check
applies); `git diff --stat acfb485 -- . ':!docs'` is empty.

### The boards and the probes

Each board page now opens with what the board is (the part, its
memories, the manual), what it offers a brio program and which stratum
it belongs to, then "The board" (the clock, the supply, the LED and the
button, the console, the probe, the identity), the wires to its peer
where it has one, "What a program meets on this board" (the traps),
"Flashing" (the board type and the mechanism) and the documents; the
pages with bench records (the AVR64EA48 board) keep their findings
whole. Each probe page opens with what the probe is and which targets
it reaches, then how `bin/brio` drives it and the traps, under
ordinary headings. The boards README defines the board type and the
three concerns of the bench (the build, the manifest, the tool) in
prose, with a table of types by project and flash mechanism; the probes
README's table is by programmer kind, with the RP2350's two-session
mechanism told apart as the board type's. Every fact, figure, pin,
part number and quirk of the previous text was kept and checked token
by token (every number, backticked identifier and register-like word
of the old page present in the new, the exceptions listed under the
facts below). The two pages the learning route links,
docs/boards/nucleo-g0b1re.md and docs/probes/st-link.md, keep every
fact and the LED, the pin and the "no device connected" rule the route
cites.

### host and cortexm

docs/host/README.md opens with whom the host target serves (the
suites' user, a driver author against a chip made of RAM, a host
program with a viewer), defines the Platform concept where it names
it, keeps the `HostPlatform` facts, the running commands as the first
batch set them, the simulations' licence, the two simulators' seams and
the simulator page, and adds one line on the `host/` project of
programs that run. docs/cortexm/README.md names its four files one by
one with every measured figure, then why the stratum exists, what
stays per family, the three architectures, the include-order contract
and the editor fragment.

### The eight READMEs

Each on the stm32g0 README's pattern: the orientation (what the
stratum is and for whom, the bench parts, what the kernel and util do
there, whether the maintained example `blink` builds there), the
boards as a table with the board pages linked, the toolchain, the
build (the presets by name as `CMakePresets.json` spells them, the
commands, the `// build:` lines, the outputs, the roster, the bench
tool), the upload with its caveats, the debugger with what is verified
and what only configured, the editor, the console, the family coverage
(the check script's sweep and the link guard), the stratum's own
sections (what the silicon taught, not covered yet) and the document
map at one line per document. Every document stays listed; the maps of
`docs/stm32f4/` and `docs/ch32v00x/` gain the vendor row the model has.
Every map cell was shortened under a script check: each number,
backticked identifier and register-like word of the old cell is in the
new cell or in the document the row links, and none is missing on any
of the eight (the two figures the regular expression flagged on the SAM
page, 1.024/2.048/4.096 V and the prescaler ratios 1000/1249/2000/5999,
are in supc.md and sdadc.md in other spellings).

### Facts resolved while writing

- docs/avrxt/README.md quoted a launch entry that `.vscode/launch.json`
  does not contain (`debugServerPath` at `~/.local/bin/pyavrocd`, a
  `-u <probe serial>` flag, three setup commands): the file launches
  PyAvrOCD through `/usr/bin/env` with `PYTHONUNBUFFERED=1`, names no
  serial, sets `stopAtEntry` and connects by hand in five setup
  commands. The page now quotes the entry as the file has it and says
  why (the file's own comment records both findings).
- docs/boards/README.md said the example manifest holds "one entry per
  board type"; `cli/bench/bench_boards.py` holds entries for ten of the
  twenty-three types (one or more per kind of probe the tool drives).
  The page says so. docs/stm32g0/README.md makes the same claim and is
  out of scope: open for its owner.
- docs/probes/README.md listed `rp2350_openocd` as a programmer
  mechanism; it is a `BOARD_TYPES` flash mechanism in
  `cli/bench/common.py`, and the RP2350's programmer entry is an
  `openocd_cmsisdap`. The table is by programmer kind and says so.
- docs/probes/README.md said the 0.12.0 release serves "every SWD path
  of the older targets"; the AVR has no SWD path. Now: the SAM C21 and
  STM32 targets and the Pico.
- `.vscode/launch.json` has no entry for the RP2040, the RP2350 and the
  CH32X035 (the RP2040 page said so; the other two now do), and its
  CH32V203 and CH32V303 entries are marked untested from the editor in
  the file itself: the ch32vx03 page says configured, not verified.
- `brio/ch32vx03/` and `ch32vx03/` have no `.clangd` fragment; the page
  had no editor section and still has none (nothing stated either way).
- `avrxt/src/apps/blink.cpp` has no `// build: boards` line, so it
  builds for the AVR128DB48 alone; the AVR page says so. The RP2040,
  CH32V203/CH32V303 and CH32X035 projects have no `blink`: each page
  states the gap, as docs/README.md's rule asks.
- Bench tools that are neither a test suite nor a peer are no longer
  named on a board page (`pin_check`, `family_probe`, `serial_speed`,
  `wire_check`, `console_usb`, `bench_ch32`; `bench_avr` stays as a row
  of the AVR64EA48 board's results table, a record of a run); the
  finding each carried is kept and attributed to "the pin-check tool",
  "a wiring check", "the benchmark skeleton", "measured".
- Ordinals dropped from the public voice: "the second-silicon board"
  (Nucleo-F446RE), "the fourth board" (32F469IDISCOVERY), "brio's first
  ARMv7-M family"; the desk positions in the CH32V00x page's commands
  (`brio flash I console`) replaced by `<board>`.
- The three READMEs that cited docs/design/kernel.md by section number
  (avrxt: 1; rp2040: 12; ch32x035: 1) now cite the heading text ("The
  model in one page", "Two cores: two kernels and a bridge"); the
  dependency map of the first batch loses those three entries, and the
  headers' numbered references are untouched.
- "a sibling and peer of the other five" (stm32f4, rp2040) and "the
  RISC-V twin of the other three scripts" (ch32v00x) were counts that
  had aged; now "the other cross projects" and the script by name.
- The LED of the AVR128DB48 board: no page names one, and
  `avr_board.hpp` gives that board no LED fact; the AVR boards table
  says "none wired by the board". Open: whether the board has one.
- Pre-existing and out of scope: `docs/stm32f4/vendor/README.md` links
  `../boards/blackpill-f411ce.md`, which resolves to a path under
  `docs/stm32f4/` that does not exist (one `../` short); and the
  comment of `cli/bench/bench_boards.py` lists the STM32F4 board types
  without `f469ni`.

### Reader review (fresh subagents, one round)

Two readers, each given only its files, the audience profile and the
brief's questions, with scenarios: one on three board pages
(weact-rp2040, ch32x035-evt-f8u6, nucleo-g031k8) and the WCH-Link
page, five scenarios; one on the AVR and the CH32V203/CH32V303
READMEs, eight scenarios. Every scenario was answered "supported
directly" (the SPI pads, the side the pull-ups hang from and why, and
what the other board runs; the PC17 pull-up selecting the boot loader
and the bench's answer; "no device connected" and the mass-storage
fallback, reached through the link the Nucleo page supplies; the
2.16 LinkE's dropped runs above 115200 and the rate that carries whole;
the firmware check before the pins; the EA's group images and the tier
guard; BOOTSIZE 0 and the fuse command, reached through the board
page's link; the folded ISRs and `no_icf`; the DWARF caveat's cause and
fix; `reset run` and the dead-looking board; USB under the kernel's
idle and the bus-master count; the trampoline, the leaf and the vector
guard; the two parts run and what stands in for the eleven). Residue,
all applied: on the board pages "dark image", "function 2", "port D",
"the pin-check wave", "the letters that judged one run by name only",
the 47 Ohm without its reason, no `brio flash` form on the RP2040 page,
"the platform suite's first letter's", "the desk", "the console's LED
command", AF1/AF6, "a basic instance", "the oscillator bridges",
"stratum" undefined on the boards README; on the probe page "the fork",
"the hart", "the ModeS key", "letter w", "the hands-free replug", a
suite's "letter"; on the AVR README "smoke TU", "a negative", "the link
guard", "letters" and `z`/`y`, "the benchmark skeleton", `-s nop` and
the other flags named by their switch, which boards the launch entry
serves and that only the AVR128DB48's SVD is committed, the firmware
1.41 sentence scoped to writes over UPDI, `<app>` as a file name,
`brio run` in the commands, the `ALL: 0 pass, 0 fail` line said to be
no verdict, "the run-time twin", the packages that have no board; on
the CH32V203/CH32V303 README "both ways the hardware prologue can be
built" (the `CH32VX03_HPE` option explained), the three vector macros
named (`BRIO_CH32_VECTOR_FLOAT` was the unnamed one), "the bus-master
count", "the desk pair", "the fork's own console", P14, "the latch",
"zero-wait window" and "the tail", the upload's exact command, `brio
run`, the lot note's digit. Left as is, with the reason: the
peripheral documents' vocabulary in the map cells (the host demotion,
pin-level bonding, the data-space window, the mask's shadow) belongs
to the documents the cells link and is defined there; the exact
spelling of an `ISR()` with `no_icf` on the AVR is not written anywhere
(no image carries one) and the page states the rule without inventing
the spelling; how a panic shows itself is platform.md's. No second
round, by the batch's cost rule.

### Checks

- `bin/brio prose` over the thirty-five files: 0 errors, 0 for review
  (before the batch: 0 errors, 2 for review, the two true claims of
  absence on the 32F469IDISCOVERY page, now worded without the
  pattern).
- Links and anchors of every page under docs/, README.md and CLAUDE.md
  by the script with GitHub's anchor rules: 0 problems on the batch's
  files; the eight hits of the whole tree are the seven lambda
  notations of kernel.md and the learning pages (the known false
  positive) and the stm32f4 vendor page's pre-existing link above.
- The token check of the batch (every number, backticked identifier
  and register-like word of each old page present in the new): the
  only absences are the dropped bench-tool names, the desk letters, the
  stale launch snippet and command forms re-spelled, each listed
  above; the map check of the eight READMEs reports no figure missing.
- `grep -P '[^\x00-\x7F]'` over the files: nothing.
- `git diff --stat acfb485 -- . ':!docs'`: empty.
- Not run, no toolchain: any build, the host tests, the full
  `brio gate` (no source changed, so it should report every preset
  identical).

### Local verification handoff

- The full `brio gate` against acfb485 on a machine with the
  toolchains, for the record (docs only; every image should be
  identical).
- The AVR launch entry as quoted: one F5 on the AVR128DB48 board
  confirms the quoted `debugServerArgs` and setup commands still land
  on `main()` as the file's comment records.
- The two claims of the batch that the desk can settle in a minute:
  whether the AVR128DB48 board has a LED (the page says none wired),
  and the stm32g0 README's "example entry per board type".
- The stm32f4 vendor page's link, one `../` to add, by whoever next
  edits that page.

## Sixth batch: the ch32x035 stratum

Scope, capped: the nine peripheral documents of `docs/ch32x035/` (every
page but README.md and vendor/README.md) in the brief's additive pass,
and the comments of all fifteen headers of `brio/ch32x035/` (the seven
part tables under `parts/` untouched). Base: `docs-rewrite` at
03902163821e258f2ea9f6ff1febc78c4cdb04e9, the reference of every token
check. Branch `claude/ch32x035-docs-99jwgr`, seven commits (the four
documents the console's glue touches first; the five others; the nine
platform, interrupt, timebase, clock, pin and map headers; the six
driver headers; this record; the two readers' residue), one pull
request against `docs-rewrite`. Not touched:
every code token (the token check on every header), apps, CMake,
presets, ld, the crt, `.vscode`, `.clangd`, every other stratum, the
design pages and the learning route, docs/ch32x035/README.md and
vendor/README.md, CLAUDE.md, docs/README.md. No `// build:` or `// mcu:`
line exists in the headers. No build, no host test and no cross
toolchain in the cloud; the token check lexed with the system g++
13.3; the clone was shallow and was unshallowed before the first check.

### The documents

Each of the nine opens with: whom the document is for and what the
driver does in a program; the smallest use, an excerpt of the
document's own "How to use it" block (platform.md's the glue the
console and the suites spell; tim.md's the opening of the edge list,
not the one-pulse prose, which names no channel enable and would have
been a new example), explained line by line; the prerequisite that
decides correctness even there; and a reading guide. "Documents of
record" became a heading on every page, with a suite's letter and the
all-key defined beside the suite's name. The terms defined where the
pages use them: a concept, the crt, the hardware prologue and its
option, an active object, a task, a monostate, a beat, a column, MOE.
The one reference to kernel.md by section number (platform.md) is a
link by heading. Capitals used for emphasis lowered across the nine
pages, acronyms and register names kept. Nothing reordered, no finding,
figure, register name, bit name, gap item or realization cut.

The prerequisites stated, each checked against the header: the clock
task first, because a divisor or a compare is computed from the rate
the type claims (a port or a ticker opened at the 8 MHz reset rate with
a 48 MHz type runs six times too slowly, nothing refuses it); the
vector bound by the program under `BRIO_CH32_INTERRUPT`, said the
operational page's way (the project's `CH32X035_HPE` option, on by
default, the crt setting INTSYSCR.HWSTKEN and the attribute becoming
WCH's fast one; one switch for the image); a DMA channel at work
holding the program awake (`idle()` returns at once over
`Dma::any_working()`); a pad read after its port is configured and a
level written before the first configuration going through
`output(level)` or `input(pull)`; an I2C or SPI buffer lent until the
reply; a timer pad claimed after the counter's first run.

### The headers

Comments only, every one of the fifteen token-identical to the base.
Each opening now says first what the component does for a caller and
the smallest use (written out, or pointed at the task below), which
context calls what (the setup verbs in main after the clock task; the
vectors' bodies called from the program's handlers and nowhere else;
the rings' one producer and one consumer; what waits and its bound;
what masks: the USART's transmit arming under a receive engine alone,
the transmit engine's claim; the register verbs with no guard of their
own), who owns a lent buffer until when, then every fact the old
comment gave with its citation. The public operations the smallest
uses call gained the sentence they lacked. Stale claims corrected to
the code's truth: device.hpp listed "the timers" among the blocks with
no driver (TIM1 and TIM2 are tim.hpp's; TIM3 is the one without) and
said no verb writes AFIO_CTLR (afio.hpp's `usb_pad_pull_ups()` writes
its two pull-up mode fields); the I2C host's class comment named four
template parameters where the class has five (`Placement`). The
kernel's rule is cited by section number and heading text in
platform.hpp, pfic.hpp and spi.hpp (the inventory above lists the
numbered references; the numbers stand).

### Facts resolved while writing

- platform.hpp's "a 4096-item copy ... had moved 68 items when the
  tick woke the core 19 000 cycles later" and dma.md's "3862 to 4037
  items left ... 17 000 to 47 000 cycles later" are one run and a
  range; the run is inside the range. Both kept.
- The USART's "one load with the gate closed, two a channel with it
  open" (platform.hpp, dma.hpp) and platform.md's "four instructions
  ... six a channel" count loads and instructions respectively; both
  kept.
- spi.md's smallest use drives the select high after `init()`; the
  orientation says the select is driven high before the first request
  is posted, not before `init()`, which does not claim that pad.
- pwr.md's 200 us settle is the suites' choice: the manual gives no
  settling time, and the orientation says so.
- i2c.hpp's "the Request is the other strata's" was the prose net's
  one review item on the stratum; now "the same struct on every
  stratum".
- pin.hpp said a configuring verb races an interrupt handler "that
  configures the same port"; `Port::clock_on()` is a read-modify-write
  of RCC_APB2PCENR, which the three ports' gate bits share, so two
  ports race on the gate too. The comment now says any port.
- i2c.hpp called the read's last-byte wait for RxNE "bounded";
  `receive_step()` discards `wait_until()`'s answer and reports i2c_ok
  with what DATAR holds when the bound runs out. The comment says so;
  whether the code should report something else is the driver owner's
  call (open).
- "the four i2c_bus.hpp holds" counts the engine's own codes; the
  arbiter's three (i2c_ok, i2c_rejected, i2c_timeout) sit beside them.
  The comment now lists them.
- A port opened before the clock task speaks at a sixth of the rate
  asked until the clock moves, and at the rate asked after: BRR is a
  divisor of HCLK, written once. Derived from the register arithmetic,
  not measured.

### Reader review (fresh subagents, one round)

Two readers, each given only its files, the audience profile and the
brief's questions with scenarios: one on platform.md and usart.md (nine
scenarios), one on the comments of pin.hpp and i2c.hpp (eleven). Every
scenario was answered "supported directly" (a port opened before the
clock task; the vector left unbound; what decides `BRIO_CH32_INTERRUPT`
and why one spelling per image; the kernel never sleeping over a
receive ring; a 4000-byte print from a handler; the interrupt
transport's ceiling by placement; `Uart<1, P>` on the QFN20; the
suite's letter; a store through a shut gate; an output on a shorted
pad; a pull-down refused; a toggle beside a configure; the I2C init()'s
refusals; the lent buffer's lifetime; a START into a held SDA and its
vector; start()'s context and bounds; the repeated START's timing; the
channel being the request), except the panic with no debugger
(platform.md's own gap: whether an `ebreak` with no debugger spins or
raises the core deadlock reset is unmeasured and stays listed) and the
ring size behind the interrupt transport's measured ceilings (open,
below). Residue, applied: platform.md's `<>` as the default timebase,
the `...` of the Tenuto list, what `take_panic_record<P>()` returns is
for, where `CH32X035_HPE` is chosen, a panic's record surviving a reset
something else causes, `z` as this suite's all-key where usart.md's
differ; usart.md's "an index shared with a handler bare" (now the
platform's `atomic_width`), how a program with no line sink writes the
handler, where `init()` and `print` sit relative to `main()` and
`run()`, the reset-rate port's rate once the clock moves, "handler"
told apart from an interrupt handler; pin.hpp's pad, pin and package
pin defined, the cross-port gate race, the STM32F1 expanded, the lock's
silent refusal on `output()`, a port's reset line and the CFGHR copy;
i2c.hpp's active object, arbiter, tenure and lending defined at first
use, the five I2cBus arguments, the DMA vectors' names, the status
codes, the two blocks sharing channels 6 and 7, the unbounded last-byte
read. Left as is, with the reason: the C++ notation itself (`using`,
templates, `constexpr`, `static_assert`, an enum's spelling, the digit
separator) is the learning route's, which the pages link; the `#include`
lines and a program's skeleton are not carried by a reference page (the
first batch's rule); how `print` formats numbers is util/print.hpp's;
how a console key is sent is the operational page's. No second round,
by the batch's cost rule.

Open: the receive ring size of the suite's interrupt-transport port
behind usart.md's "whole to 2 Mbaud" (the page states the ceiling
without the ring that bounded it); whether the I2C host's last-byte
read should report a code of its own when its wait runs out.

### Checks

- `bin/brio prose brio/ch32x035/*.hpp docs/ch32x035/*.md`: 25 files, 0
  errors, 0 for review (before the batch: 0 errors, 1 for review,
  i2c.hpp line 16).
- `bin/brio gate --tokens --against 0390216 brio/ch32x035/*.hpp`: all
  fifteen token-identical (lexed with the system g++ 13.3).
- Links and anchors of the ten pages of docs/ch32x035 and the vendor
  page, by the script with GitHub's anchor rules: 0 problems; the
  three hits of the run are README.md's lambda notation, the known
  false positive outside this batch.
- `grep -P '[^\x00-\x7F]'` over the touched files: nothing; no
  `// build:` or `// mcu:` line in any header.
- `git diff --stat 0390216 -- . ':!docs/ch32x035' ':!brio/ch32x035'`:
  empty before this record was written; no source outside the stratum,
  no build or configuration file changed.
- Not run, no toolchain: any build, `brio check ch32x035`, the host
  tests, the full `brio gate`.

### Local verification handoff

- The full `brio gate` against 0390216 on a machine with the
  toolchains confirms every image of the seven presets unmoved
  (comments only; the token check says so, the gate proves it).
- `brio check ch32x035`, for the family fixtures' sake (nothing in them
  changed).
- The excerpts in the orientations are copies of each document's own
  "How to use it" block, and the headers' smallest uses copies of those
  or of the console's glue (`ch32x035/src/apps/console.cpp`); a re-read
  of each pair after any API change is the maintained-example rule
  applied to them.
- The one claim of the batch the desk can settle in a minute: that a
  port or a ticker initialized before the clock task runs six times too
  slowly (derived from the divisor arithmetic, not measured).

## Local batch: the stm32f4 stratum

Scope: the twenty-five peripheral documents of `docs/stm32f4/` (every
page but README.md and vendor/README.md) in the brief's additive pass,
and the comments of all thirty-three headers of `brio/stm32f4/`. Base:
`docs-rewrite` at a70d4b2 (a70d4b23b8e46be0fb89bff8890f1b8b8214bcd2),
the reference of every token check; local branch `docs-stm32f4`, no
commit, delivered as a patch. Not touched: every code token, apps,
CMake, presets, ld, the crt, svd, `.vscode`, `.clangd`, every other
stratum, the design pages and the learning route (no link of theirs
broke), docs/stm32f4/README.md and vendor/README.md (no link or figure
moved), CLAUDE.md, docs/README.md. No `// build:` or `// mcu:` line
exists in the headers. Local machine, no board touched; the token check
lexed with the host g++ on the PATH; no build, no host test.

Method: platform.md, port.md and their headers (platform.hpp, pin.hpp,
ticker.hpp, delay.hpp, nvic.hpp, dwt.hpp) written first by hand as the
stratum's pattern; the other twenty-three documents and twenty-seven
headers by ten parallel agents, each on a document and its headers,
under the same written rules, each reporting its stale claims with
evidence and running the checks itself; every report read and the
claims the reports leaned on spot-checked against the code (spi.md's
orientation, clock.hpp's ready_spins bound and restore()'s context,
usart.hpp's bit-band and the dozen cycles masked, ltdc.hpp's corrected
figure, i2c.md's orientation).

### The documents

Each of the twenty-five opens with: whom the document is for and what
the driver does in a program; the smallest use, a verbatim excerpt of
the document's own "How to use it" block (a script checked each line's
presence), explained line by line; the prerequisite that decides
correctness even there, checked against the header (the clock task
first, with the consequence derived from the divisor or reload
arithmetic and marked as derived; a vector the program must bind, the
crt's weak `Default_Handler` spin being what an unbound one runs; a
buffer lent until the reply or until `busy()`/`wait()` answers; a
`false` that must be read; a pad to hand over); and, on the long pages,
a reading guide. "Documents of record" became a heading on every page;
the reserve, a suite's letter and the all-keys (`z`, `t`), a monostate,
a task, an engine, an active object, a tenure, a sleep site and a rung
defined where each page first uses them; kernel.md linked by heading
(platform.md's "section 11"); insistence capitals lowered in the
orientation, the documents of record and "What the silicon does".
Nothing reordered; every "Bench findings" section byte-identical to
the base (a script compared them); the gap lists changed only where an
item was stale (below). One rendering fault fixed: usart.md's second
code block closed with "``` A mode beyond the", so the rest of the page
rendered as code.

### The headers

Comments only; all thirty-three token-identical to a70d4b2. Each
opening now says what the component does for a caller and its smallest
use, then which context calls what (what waits and its bound, what
masks interrupts, who owns a buffer until when), then every fact the
old comment gave with its citation, capitals lowered; the public
operations the smallest uses call gained the sentence they lacked. The
numbered kernel.md references in nvic.hpp, reset.hpp (section 11) and
spi.hpp (section 1) keep their numbers and gain the heading text. The
prose net's six review items on the stratum's headers and pages (the
Request "verbatim" of i2c.hpp and spi.hpp, "the other strata's rule",
dma.md's "there is no SDIO driver" twice) are gone: the first three
were false and are corrected, the last two were true and are reworded.

### Stale or false claims corrected

- platform.md, platform.hpp, ticker.hpp, nvic.hpp, pin.hpp: the power,
  reset and EXTI chapters spoken of in the future ("the power chapter
  will own", "until the fault-handling pass", "the EXTI chapter's driver
  is what will own them", "stm32f4/pwr.hpp's to grow"): pwr.hpp,
  sleep.hpp, reset.hpp's `Faults` and `hard_fault_reset<P>()`, and
  exti.hpp exist; the platform's gap items now point there, and "no
  suite has faulted the part yet" now says `test_stm32f4_reset` takes
  the fault vectors with a body bound and the crt's spins are unrun.
- pin.hpp, port.md: "every configuring verb opens the port's clock":
  `pull()` writes PUPDR alone and opens nothing; said in both.
- spi.md, spi.hpp: the Request "verbatim", "field for field, in that
  order": the stm32g0 order is cs, dc, cmd, tx, rx, reply, len,
  cmd_len, cs_setup_us, clock, mode, bits, polled, and this family's
  ends with bits, polled, cs_setup_us, clock, mode, reply; now "the
  shared fields by name, in this family's types and order". spi.hpp's
  "the bus AO asserts the chip select": the engine does (`start()`).
- i2c.md, i2c.hpp, fmpi2c.md, fmpi2c.hpp: the Request "the other
  strata's verbatim": same field names, but each stratum's `I2cSpeed`
  (and the FMPI2C's `FmpI2cSpeed`) is its own, so a program naming a
  speed the block lacks does not compile; "init() refuses both" (it
  refuses below 2 MHz and marks the fast row unreachable below 4);
  fmpi2c.hpp's "always asynchronous", "no i2c_bus_error here" and "the
  one synchronous answer" against `start()`'s two synchronous refusals
  and its `i2c_bus_error`.
- i2c.md gains a gap item, i2c.hpp a comment: the read's last byte
  polls RxNE up to 400 000 times inside the event handler and, when the
  bound runs out, reads DR regardless and reports `i2c_ok` (the
  ch32x035 finding, present here too).
- usart.hpp: "the same surface the other five targets expose" (an aged
  count): now util/stream.hpp's ByteTransport.
- clock.md, clock.hpp: Stop's restore "the power chapter's to restore"
  (sleep.hpp's `resume_clock()` does it); "spi.hpp and ltdc.hpp are what
  call" the audio-PLL verbs (spi.hpp names them only; ltdc.hpp calls
  PLLSAI's); "LSI, LSE, the SAI PLL ... declared and refused"
  (`ClockSource` names none of the three); the gap items on PLLI2S,
  PLLSAI, TIMPRE, the Q output and the AHB2/AHB3 gates now say which
  verbs exist and who calls them. device_tables.hpp: "one class of
  exception, the frequency ladders" (the DMA request mapping, the I2S
  face and the ADC's internal channels are keyed on the part class
  too). syscfg.hpp: two history sentences.
- dma.hpp: `load()`'s "false when EN did not stay up" (`enable()` reads
  nothing back; false is the refusal alone, a dead stream shows in
  TEIF). rng.md, rng.hpp: the F410's gate "the same bit" (bit 31 of
  AHB1ENR against bit 6 of AHB2ENR); "a compile error naming the reason"
  (the type is not declared); the letter run "on the STM32F429" alone
  (the STM32F469 runs it too, per the page's own findings).
- exti.md, exti.hpp: "no SleepSite in this stratum yet", "the lines
  above 15 have no owners", "SLEEPDEEP is never written by this
  stratum", `Rtc::exti_line` (does not exist; the RTC publishes
  `alarm_exti_line`, `wakeup_exti_line`, `tamper_stamp_exti_line`).
  tim.md, tim.hpp: TIMx_SR "the one place" a flag register is rc_w0
  (the ADC, RNG, RTC_ISR, WWDG, USART and I2C have rc_w0 flags);
  "staged at the bench" for an erratum tim.md says is not staged;
  `vector_flags()` returning `all_flags` (it returns `interrupt_flags`);
  the not-verified item's "neither board is on the desk" against the
  page's own F469 findings. reset.md: one history-voice sentence.
- adc.hpp "six facts" (seven listed), dac.hpp "five facts" (six);
  can.hpp's erratum missing the F469/F479 (ES0321 2.15.1, as can.md and
  the reserve's `can_ttcm_erratum()` have it) and `ack_spins` "a hundred
  thousand" (200 000).
- fmc.hpp: "memory_clock(off) is what really leaves a device
  unrefreshed" (only `Fmc::reset()` does, per the comment beside it and
  the measurement); fmc.md's "the LTDC has no driver yet". ltdc.hpp: the
  usage snippet's `configure(layer)` (the function takes a layer config
  and a timing); "some 80 core cycles a read" (seven PCLK2 periods and
  five LCD_CLK periods at 6 MHz are some 165; 216 to 219 measured).
  ltdc.md's "one panel, one rate" scoped to its suite's board. dma2d.md:
  the 32F469IDISCOVERY's fills under `test_stm32f4_dsi`.
- pwr.md, pwr.hpp: the BOR levels' "provisioning verb" of the flash
  chapter (none exists; flash.md declines every BOR write). rtc.hpp:
  fact 7's private EXTI helpers "until exti.hpp exists" (it includes
  exti.hpp and uses `Exti::`); an LSI row under `RtcDomain` (the LSI is
  `Rcc`'s). rtc.md's two gap items brought to what pwr.md measures.
  flash.md, flash.hpp: the dual-bank exception "the F42x/F43x class"
  (the F469/F479 class too, per `flash_facts()` and the page's own
  STM32F469NI findings); "the errata sheets of the three bench parts"
  (four parts). quadspi.hpp: `window()` before `map()` as "the bus
  error 13.3.13 describes" (with the block disabled; in indirect mode
  it is erratum 2.4.5's case).

### Facts resolved while writing

- A ticker started before the clock task under a 180 MHz type at the
  16 MHz reset rate ticks about eleven times too slowly (reload 179999
  on a 16 MHz HCLK), and `Ticker::init()` accepts it; a USART1 opened
  then speaks at 16/90 of the rate asked (PCLK2's 90 MHz divisor on a
  16 MHz bus), an I2C bus at about a third (APB1's 45 MHz). Derived
  from the register arithmetic, not measured; platform.md's bench note
  of a console at the wrong baud after a failed clock init is the
  measured cousin.
- An unbound vector is the crt's weak alias of `Default_Handler`, a
  silent spin (stm32f4/src/glue/startup_stm32f4*.cpp): an unbound
  SysTick stops the program at its first tick.
- The kernel never enters Stop by itself: `Stm32f4Platform::idle()`
  writes no sleep depth; a Stop is a sleep site's arming.

### Reader review (fresh subagents, one round)

Two readers, each given only its files, the audience profile and the
brief's four questions with scenarios: one on platform.md and usart.md
(eleven scenarios), one on the comments of pin.hpp and spi.hpp (eleven).
Supported directly: the ticker before the clock and its eleven-fold
slowness, the forgotten SysTick handler, the pending interrupt between
the empty check and the WFI, no nesting and no BASEPRI, the port before
the clock at 16/90 of the rate, the 4000-byte print stalling the other
active objects, write_byte and print never from an interrupt handler,
the caller's buffer free after write_bulk, the letter and `z`, the
unbound USART vector; a value function before any configuring call, the
reset state, the polled transaction's context, wait, masking and buffer
lifetime, "true is ended, not succeeded", the arbiter's buffer lifetime
and who drives the select, the select parked high first, the
non-portable Request order. Residue, applied: usart.md's two
definitions of `isr()`'s true (now: one per empty-to-non-empty
transition on the plain transport, the consumer's idle-to-busy gate
under a receive engine), where `init()` and `print` sit relative to
`main()`, `enable_interrupts()` and `run()`, the NVIC line `init()`
enables, the line sink's place in the Tenuto list (pointed at
design/serial.md), a print that cannot drain (before init and
interrupts, or in a handler: for ever, util/print.hpp), the one
producer being the main context whatever the number of active
objects, "the clock task's init" named `SysClock::init()`;
platform.md's list order as priority, the HSE crystal as the default
fourth argument, the sleep site defined and the kernel never stopping
by itself, `delay_us`'s false; pin.hpp's G0 paragraph read as the G0's
facts (now: the three facts are this family's), the main context named
for the configuring functions, AO expanded, PwmChannel explained,
"static_asserts" and "stays open" reworded; spi.hpp's `clock` tag, the
post's destination, the two senses of "engine", the lending "nothing
checks", the Request object's own lifetime (free when start() returns;
a posted one in the arbiter's slot until the dispatch ends),
`brio::lend` named. Left as is, with the reason: the C++ notation
(`using`, `constexpr`, digit separators, designated initializers) is
the learning route's; includes and a program's skeleton are not a
reference page's; the console's command keys (`CLK`, `ERR`) live in
the bench findings, kept word for word; the board-file mode in letter
`e`'s finding, likewise. No second round: nothing the first found was
broken in substance.

### Checks

- `bin/brio prose docs/stm32f4/*.md brio/stm32f4/*.hpp`: 59 files, 0
  errors, 0 for review (before the batch: 0 errors, 7 for review).
- `bin/brio gate --tokens --against a70d4b2 brio/stm32f4/*.hpp`: all
  thirty-three token-identical.
- Links and anchors of every page of docs/stm32f4 and its vendor page,
  by a script with GitHub's anchor rules: 0 problems. Code fences
  balanced on every page; `## Documents of record` on all twenty-five.
- `grep -P '[^\x00-\x7F]'` over the stratum's files: nothing; no
  `// build:` or `// mcu:` line in any header.
- Preservation, by script, old file against new: no number and no
  backticked identifier lost but the stale ones corrected above
  (`all_flags`, `Rtc::exti_line`, `SleepSite`, ltdc.hpp's 80); the
  ALL-CAPS words absent are lowered insistence capitals, register and
  bit names all present (clock.hpp's opening citation of RM0090 ch.
  6/7, RM0390 ch. 6 and RM0383 ch. 6, dropped by its rewrite, was found
  this way and restored); every bench findings section byte-identical.
- `git diff --stat a70d4b2 -- . ':!docs/stm32f4' ':!brio/stm32f4'
  ':!DOCUMENTATION_REWRITE_PROGRESS.md'`: empty.
- Not run: any build, `brio check stm32f4`, the host tests, the full
  `brio gate`.

### Open

- tim.hpp cites RM0090 table 61 for TIM1/TIM8's vector split, where
  platform.md names the vector table as table 62; the PDF settles it.
- dac.hpp's fact 3 says an unbuffered output cannot drive the ADC's
  sampling capacitor, dac.md says an LED's resistor; which was measured
  is not recorded.
- dma2d.hpp calls `abort()` and `suspend()` legal from a handler; both
  are read-modify-writes of CR, and a race with a main-context write is
  not established either way.
- `Rtc::read()` from two contexts at once in shadow mode: not
  established; the header claims no context for it.
- usart.md's gap item sends a pending receive across a Stop to the
  power chapter, and pwr.md covers the transmit side only.
- design/spi-bus.md says an 8-bit request "is spelled identically on
  every row"; true for code assigning fields by name, false for a
  designated initializer across the G0's and the F4's orders: for the
  design page's owner.
- usb.hpp keeps "brio's second realization of that contract, after the
  RP2040's": an ordinal of realizations, not of a silicon; the public
  voice rule's reach over it is the user's call.
- `FmpI2cHost::apply()` discards `disable()`'s answer; a PE that never
  reads back clear leaves the tenure at the previous speed. Read from
  the code, written nowhere, unmeasured.

### Local handoff

- The full `brio gate --against a70d4b2` with the toolchains confirms
  every image of the stratum's presets unmoved (comments only;
  the token check says so, the gate proves it); `brio check stm32f4`
  for the family fixtures' sake.
- The excerpts in the orientations are copies of each page's own "How
  to use it" block; a re-read of each pair after any API change is the
  maintained-example rule applied to them.
- The derived figures the desk can settle in a minute: a ticker and a
  console opened before the clock task (eleven-fold slow; 16/90 of the
  baud).

   pages' links; the `ch32x035` stratum is done (the sixth batch), and
   the `stm32f4` stratum's documents and headers by the local batch
   above (uncommitted, a patch).
++ b/brio/stm32f4/adc.hpp
 * ch. 13, RM0383 ch. 11): what a program uses to measure a voltage on a
 * pad, the supply rail through VREFINT, or the die's temperature, polled
 * or under an interrupt, one channel or a sequence into memory through
 * the DMA. The smallest use, one conversion of a pad (docs/stm32f4/adc.md
 * has it with every other use):
 *     using Adc1 = brio::Adc<1>;
 *     using In4 = brio::AnalogIn<brio::Pin<'A', 4>>;   // channel 4
 *     In4::claim();                                    // analog mode
 *     Adc1::init(clock, brio::AdcConfig{});            // after SysClock::init()
 *     Adc1::sample_time(In4::channel, brio::AdcSampleTime::cycles480);
 *     Adc1::select(In4{});
 *     const uint16_t counts = Adc1::read();
 *
 * Who calls what, and from where. Every verb here is a register verb with
 * no guard of its own: the configuring ones (init(), configure(),
 * sample_time(), the sequence verbs, watchdog(), interrupts()) are setup
 * calls or an active object's, never two contexts on one converter at
 * once, since several are read-modify-writes of CR1, CR2 and the SMPRs;
 * nothing here masks interrupts. What waits, and for how long: init() and
 * power_on() spend the converter's stabilization time through delay_us
 * (4 us); read() and read_settled() poll each conversion's end for at
 * most `spins` turns (0x100000 by default) - read(uint16_t&) answers
 * false when the bound runs out, the plain read() returns 0 then, a value
 * a real input can also read. select() and start() are void because
 * util/analog_sampler.hpp's AnalogConverter concept asks for them so, and
 * select() refuses in silence while a conversion is in flight
 * (select_sync() is the same with the answer). isr() is the body the
 * program's ADC_IRQHandler calls, one vector for every converter; it
 * clears the armed flags it finds and never reads the data. A DMA
 * sequence writes into a buffer the caller owns, which stays the
 * caller's to keep in scope while the stream runs.
 *
 * The faces:
 *
 *   AdcCommon        the block - one per device: the prescaler every
 *                    common status and data registers, and the one reset
 *   Adc<n>           one converter, n in 1..3. `Adc<1>` is the master -
 *   AdcInput         the three sources that are not pads, as tags: their
 *                    channel numbers differ across the family and come
 *                    from the reserve (stm32f4/device_tables.hpp, the
 *                    table of what each part has, read off the device
 *                    header).
 * Seven facts that shape this file.
 * 1. There are up to three converters and one clock. ADCCLK is PCLK2
 *    in the common register, so it is the whole block's (13.3.3). fADC
 * 2. There is no calibration and no regulator. Unlike the STM32G0's
 *    measurement, not a trim: nothing is written back (see `AdcFactory`).
 * 3. Two groups, and the injected one preempts. A regular sequence is up
 *    to sixteen conversions into one data register (so a sequence longer
 *    than one needs the DMA or a reader fast enough), and an injected
 *    subtractable offset and a sign. An injected trigger interrupts a
 *    from the tail (13.13.12's note: with JL = 0 the converter runs JSQ4
 * 4. The status register is rc_w0, and that decides how it is cleared.
 *    ADC_SR's six flags are "cleared by software" by writing zero
 *    else, which keep - and never a read-modify-write, which would clear
 * 5. EOC is cleared by reading the data. 13.13.1: EOC goes down on a
 *    and that is a choice the chapter offers (13.8.3), not an oversight.
 * 6. The multi-ADC modes are the master's. Dual and triple modes are
 *    multiplexer and the slaves' triggers must be off (13.9), and the
 * 7. A sequence longer than one needs the DMA, and the request is a cell
 *    against it; there is no engine slot here, because a converter has no
 * The reference is a pad and nothing else. This family has no reference
 * Errata (ES0206 Rev 24 for the F42x/F43x, ES0298 Rev 8 for the F446,
 *    ES0298 2.6.1, ES0287 2.4.1, every silicon revision): with a software
 *    that conversion and the converter does not restart by itself.
 *    Answered structurally: every sequence verb here refuses while a
 *    accuracy whatever the power mode. The workaround is a system one -
 *    the flash accelerator's prefetch off with both caches on, plus
    /// One conversion, polled: clear EOC, start, wait, read. Waits for at
    /// most `spins` polls of EOC; false, `out` untouched, when the bound
    /// runs out (a converter never powered is one way there). Main
    /// context or an active object's.
    /// The same, the counts as the return: 0 when the bound ran out, which
    /// a real input can also read - read(uint16_t&) tells the two apart.
++ b/brio/stm32f4/can.hpp
 * ch. 30): what a program uses to put a node on a CAN bus, or to test
 * one with no bus at all in loopback. `Can<1|2|3>` is the resource over
 * the whole chapter, with `CanFrame`, `CanTiming` and `CanFilter` as its
 * vocabulary. The smallest use, a node at 500 kbit/s
 * (docs/stm32f4/can.md has it with every other use):
 *     using Bus = brio::Can<1>;
 *     constexpr brio::CanPins pads{.rx = {'A', 11, brio::PinFunction::af9},
 *                                  .tx = {'A', 12, brio::PinFunction::af9}};
 *     constexpr auto timing = brio::can_timing_for(brio::apb_hz(clock, false), 500'000);
 *     static_assert(brio::can_timing_valid(timing), "no exact 500 kbit/s from this PCLK1");
 *
 *     Bus::clock(true);
 *     Bus::filter_clock(true);
 *     Bus::claim_pads<pads>();
 *     (void)Bus::init_mode(true);
 *     (void)Bus::options({.auto_bus_off = true});
 *     (void)Bus::timing(timing);
 *     (void)Bus::filter_init(true);
 *     (void)Bus::filter(brio::can_filter_accept_all(0));
 *     (void)Bus::filter_init(false);
 *     if (!Bus::start()) {
 *         // eleven recessive bits never came: nothing is driving the bus
 *     }
 *
 * Who calls what, and from where. Every verb is a register verb with no
 * guard of its own, and nothing here masks interrupts: the configuring
 * ones are setup calls or an active object's, never two contexts at once
 * on one instance. What waits: init_mode(), start() and sleep() wait for
 * the controller's acknowledge, bounded at `ack_spins` reads of CAN_MSR,
 * and answer false when the bound runs out - leaving initialization or
 * sleep needs eleven recessive bits on the receive pad, which a bus with
 * no transceiver and no other node never gives. transmit() and receive()
 * never wait: transmit() answers nothing when no mailbox is free and
 * copies the frame into the mailbox, so the caller's frame is free when
 * it returns; receive() copies the output mailbox and releases it.
 * tx_isr(), rx_isr(fifo) and sce_isr() are the bodies of the program's
 * handlers for the vectors tx_irq(), rx_irq(fifo) and sce_irq() name,
 * and nothing else calls them. The bit time is computed from the APB1
 * rate the clock type claims, so the clock task runs first.
 *
 * The vocabulary is shared, the bus active object is not. `CanFrame`, `CanTiming`
 * Three instances, one filter block. CAN1 and CAN2 come as a pair on every
 * F401, F410 and F411 have none. CAN1 is the master: the filter registers
 * CAN1's clock on, which `filter_clock()` is for. CAN3, where it exists,
 * Seven facts that shape the file.
 * 1. Three modes, and each is a request with an acknowledge. INRQ and
 *    also costs a synchronization - eleven consecutive recessive bits on
 *    wait here is bounded and reports rather than hangs. The reset state
 *    is sleep, not normal: a block whose clock was just turned on
 * 2. The bit time is counted in PCLK1 periods. tq = (BRP + 1) x tPCLK, the
 *    compile time for an exact bit rate - a CAN bus with a rounded bit
 *    application. CAN_BTR is writable only in initialization mode
 * 3. Loopback and silent are the wireless instruments - and loopback still
 *    wants its receive pad. In loop back mode (LBKM) the node receives its
 *    the CANRX input pin" - which is true of the data and not of the
 *    synchronization: measured, a node in loopback whose receive pad is
 *    held dominant never leaves initialization mode, because leaving it
 *    dominant bit and cannot start a transmission at all (30.5.1) - a
 * 4. Three transmit mailboxes and two orders. With more than one pending,
 *    the scheduler picks by identifier (lowest wins, ties by mailbox
 *    number) or by request order when MCR.TXFP is set (30.7.1). A mailbox
 * 5. Two receive FIFOs of three, and the overrun policy is a choice. FMP
 * 6. The filter banks are a scarce shared resource with four shapes. Each
 *    active or not (FA1R) - and the filter match index the receiver
 *    reports is a number over the active filters of that FIFO in the
 *    deactivates reception while it stands (30.4.1).
 * 7. Four vectors per instance. Transmit (the three mailboxes together),
 * The one erratum, and it is live. ES0206 2.13.1, ES0298 2.15.1 and ES0321
 * 2.15.1, every revision of the F427/F437/F429/F439, of the F446 and of
 * the F469/F479 (the reserve's can_ttcm_erratum()): "bxCAN
 * `options()` refuses a configuration that asks for TTCM on those part
    /// rate - so two hundred thousand register reads is a fault and not a
    /// RESET bit, which also puts it in sleep - the transmit status
    /// excepted, master_reset() says why) and its clock off. The
++ b/brio/stm32f4/clock.hpp
 * The STM32F4's clock for a program: the one place the system clock's
 * rate is stated, and the task that brings the chip from its 16 MHz
 * reset state to that rate. A program names its rate as a type and
 * calls init() first thing in main():
 *     using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000,
 *                                  8'000'000>;      // the PLL on an 8 MHz crystal
 *     constexpr SysClock clock;
 *     const bool clock_ok = SysClock::init();      // false: a root did not come up
 *
 * Every driver then takes `clock` (an empty tag object of the type) in
 * its own init and derives its divisors from the rates the type states -
 * `hz`, `pclk1_hz`, `pclk2_hz`, through `apb_hz(clock, on_apb2)` - and
 * never from the hardware, which is why init() comes first: a driver set
 * up while the chip still runs at 16 MHz is wrong by the ratio of the
 * rates, and nothing refuses it. The model is docs/design/clock.md, the
 * reference docs/stm32f4/clock.md.
 *
 * Who calls what. Clock::init() and DynamicClock's init(), set() and
 * set_index() are main-context verbs: they mask no interrupt, and each
 * of their waits on a ready flag or on SWS is bounded by
 * Rcc::ready_spins (a million polls, hundreds of milliseconds at the
 * reset rate - a hardware fault, not a slow start) and turns into a
 * false answer. A set() also calls every listed user's rebase(), in list
 * order, before anything moves. DynamicClock::restore() is the one verb
 * also legal from an interrupt handler (a sleep site's wake). The Rcc
 * verbs are register accessors with no guard of their own: setup verbs,
 * and a read-modify-write of a register another context also writes is
 * the caller's to serialize.
 *
 * The header is the STM32F4 clock tree (RCC, RM0090 ch. 6/7, RM0390
 * ch. 6, RM0383 ch. 6) in the two layers every brio target uses
 * (docs/design/clock.md):
 *
 *  Resources - monostates (all-static types with no instances), thin
 *  typed views with the discipline built in:
 *             AHB/APB prescalers, the peripheral clock enables (one bit
 *             and resets, the two MCO outputs, and the audio PLL with the
 *  Tasks - what an application names:
 *    Clock<source, hz, hse_hz, hse_mode>   the static main clock: one
 *             runtime regime: a pack of static Clock tasks, R0 the boot
 * The clock model: the STM32G0's, with the prescalers no longer pinned.
 * This family has one SYSCLK, shared prescalers below it (HPRE for AHB,
 * PPRE1 and PPRE2 for the two APBs) and an enable bit per peripheral
 * have ceilings (45 and 90 MHz on the F42x/F43x and F446, 42 and 84 on
 * the F405 class, 50 and 100 on the F411 - the reserve's ladder, the
 * reserve being stm32f4/device_tables.hpp, the table of what each part
 * has) and a core at 180 MHz cannot feed them undivided. So `hz` is
 * SYSCLK = HCLK (HPRE stays 1), and beside it the task states
 * `pclk1_hz` and `pclk2_hz`, each the largest power-of-two division of
 * HCLK under its bus's ceiling - and a driver on an APB asks
 * `apb_hz(clock, on_apb2)` for the rate that is really its own (the
 * USART's divisor divides its bus clock, not SYSCLK). What crosses the
 * util contract is unchanged: `clock_hz(clock)` is SYSCLK = HCLK. The
 * timers' kernel clock (twice the APB clock when the APB is divided,
 * RM0090 7.3.3's TIMPRE rule) is the timer chapter's to state.
 * Three things on the way up, in the chapter's order. Raising this
 *   1. the regulator scale (PWR_CR.VOS) for the target rate, written
 *   2. the PLL configured and started; where the rate needs over-drive
 *   3. the flash latency for the target HCLK, read back; the ART
 * state, and every rate a program names is a rise from 16 MHz. The fall
 * is `DynamicClock`'s, and on this family it is one move and not a
 * The PLL ratio is searched at compile time under the chapter's limits
 * 50..432, P in {2, 4, 6, 8}; an exact ratio or a compile error naming
 * What is built: `hsi` (HSI undivided, 16 MHz - the reset state, the
 * PLL from either root). LSI, LSE and the SAI PLL are not roots this
 * task offers (ClockSource names none of them), and the AHB prescaler is
 * declared and refused: a divided `hsi` does not compile.
 * Neither of the other two PLLs is a SYSCLK root, and neither is
 * PLLSAICFGR and the two selectors in RCC_DCKCFGR. stm32f4/ltdc.hpp
 * calls PLLSAI's; the audio PLL's are called by the program that uses
 * stm32f4/spi.hpp's I2s, which names them (docs/stm32f4/spi.md shows the
 * calls). Where the header declares no PLLI2SON or no PLLSAION that PLL
 * is not there and every one of its verbs answers false; and PLLSAI's R
 * output, the pixel clock's, exists only where a display does.
 * Board facts the apps of this project state: 8 MHz from the ST-LINK's
    // with the block that owns the registers, and a program using
    // stm32f4/spi.hpp's I2s calls them. Absent on the parts whose header declares no PLLI2SON, where
    /// readback does not agree - the program is then not at the rate this
    /// type states, so the answer is read. Main context, once, first thing
    /// in main() and before any driver's init(clock); it masks nothing,
    /// and every wait is bounded by Rcc::ready_spins.
++ b/brio/stm32f4/crc.hpp
 * ch. 3) for a program that wants a 32-bit checksum over a block of
 * memory computed by the hardware: `Crc`, a monostate resource (a class
 * of static members standing for the one unit the chip has) over the
 * whole chapter - which is three registers and no options at all. The
 * smallest use:
 *     brio::Crc::init();                       // the gate, then a reset
 *     const uint32_t sum = brio::Crc::compute(words, count);
 *
 * Who calls what. Every verb is a few register accesses, callable from
 * main, an active object's handler or an interrupt handler; none masks
 * interrupts, and the one that waits is reset() (with init() and
 * compute(), which call it), for at most `reset_spins` extra reads of
 * CRC_DR. A word array handed to feed() or compute() is read during the
 * call and free when it returns. The running result lives in the unit
 * and nothing guards it: two contexts checksumming at once corrupt each
 * other's answer, and the arrangement is the program's (see Crc below).
 *
 * What this block is. A hardware CRC-32 with the Ethernet polynomial
 * 0x04C11DB7 wired in: no polynomial register, no initial-value register,
 * Five facts that shape the file.
 * 1. The data register is the operation. One register, CRC_DR: a write
 *    feeds a 32-bit word into the calculator, a read returns the running
 * 2. A word is the grain, and the program decides its byte order. 4.3:
 *    access, so a byte stream has no direct answer here. `word_be()` packs
 * 3. The reset bit is the only control - and it is not instant. CRC_CR has
 *    that the reset takes effect a few cycles after the store: a read of
 *    word written in the next instruction is swallowed. `reset()` therefore
 * 4. CRC_IDR is not part of the checksum. Eight bits of scratch that the
 * 5. No interrupt, no DMA request, no event. The chapter has no IER, the
 *    and nothing else. What a DMA stream can do is memory-to-memory into
 * Errata. No item of ES0206 Rev 24, ES0298 Rev 8 or ES0287 Rev 6 is filed
 * Present on every part. All twenty-three device headers of the pack
 * this file has no absence to handle and nothing in the reserve (the
 * reserve: stm32f4/device_tables.hpp, the table of what each part has,
 * read off the device header).
// The function is not this family's, so it does not live here.
// What stays here is the packing, which is the block's own: how a byte
 * No ownership and no guard. There is one unit and it holds the running
 * is nothing in the silicon to arbitrate with - so a program that
    /// program calls once before its first checksum. Waits as reset()
    /// does, and discards its count.
     * read-modify-write, and CRC_IDR is deliberately not touched.
     * The reset is not instant, and the chapter does not say so. Measured:
     * old running value, and a word fed in the instruction after the store
     * the initial value, and returns the number of extra reads that took:
     * a measurement the bench suite prints and a caller can ignore. The
     * wait is bounded by `reset_spins`: a return equal to it means the
     * initial value had not appeared when the bound ran out, and the verb
     * returns regardless.
    /// A whole checksum in one call: reset, feed, read. `words` is read
    /// during the call only; the wait is reset()'s.
    /// The access width is the header's and not the manual's. 4.4 says the
    /// three reserved bytes after it - so the header asks for a byte store
++ b/brio/stm32f4/dac.hpp
 * ch. 14): what a program uses to put a voltage on PA4 or PA5 - a DC
 * level, a value held until a trigger, a triangle or noise wave, or a
 * table a timer plays through the DMA. The smallest use, a DC level
 * (docs/stm32f4/dac.md has it with every other use):
 *
 *     using Pa4 = brio::Pin<'A', 4>;
 *     brio::Dac::init();                               // clock, block reset
 *     brio::Dac::claim_pad<Pa4>();                     // analog mode, first
 *     brio::Dac::configure(0, brio::DacChannelConfig{});   // left disabled
 *     brio::Dac::enable(0, true);
 *     brio::delay_us(clock, 20);                       // tWAKEUP
 *     brio::Dac::write(0, brio::dac_code(1650, brio::Dac::steps, 3300));
 *
 * Who calls what, and from where. Every verb is a register verb with no
 * guard of its own and nothing here masks interrupts or waits: init(),
 * release(), configure(), enable() and wave() are read-modify-writes of
 * DAC_CR (or a reset), setup calls or an active object's, never two
 * contexts at once on the one CR both channels share; write() and its
 * siblings are single stores into a holding register, which reaches
 * DAC_DORx one APB1 cycle later with no trigger. The pad is the caller's
 * to claim: nothing in init(), configure() or enable() puts it in analog
 * mode. isr() is the body the program's TIM6_DAC handler calls - the
 * vector is TIM6's too, so it answers 0 when the DAC did not speak. A
 * DMA table is the caller's buffer, kept in scope while the stream
 * plays it.
 *
 * `Dac` is a monostate resource (a class used through its type alone,
 * every member static), because a part either has one DAC
 * What this converter is. Two 12-bit channels, DAC_OUT1 on PA4 and
 * And where its output can go: to a pad, and nowhere else. This family's
 * the only way from this converter to the ADC is the bond pad the two
 * first" a real obligation and not hygiene: a pad still in its reset
 * Six facts that shape this file.
 * 1. Channels are 0-based here and 1-based in the manual. `ch = 0` is
 * 2. The data register is not the output register. A write lands in a
 *    software trigger (14.3.4 and 14.3.6's note). So `output(ch)` reads
 *    confuses them measures its own write. And DOR is read-only - 14.3.4's
 * 3. The buffer is disabled by a one. BOFFx is an output buffer disable
 * 4. A trigger is an edge and the wave generators need one. 14.5.1:
 * 5. The DMA request comes from a hardware trigger and from nothing
 *    else - 14.3.7's own parenthesis, "an external trigger (but not a
 * 6. The DMA request is not queued. 14.3.7: a second trigger arriving
 *    channel keeps converting the old datum - so the flag is the count of
 * What this family's DAC has not, and the STM32G0's has: an offset
 * Errata (ES0206 Rev 24 2.6, ES0298 Rev 8 2.7 - the F411's ES0287 files
 *    the moment the DAC is enabled again. The workaround is a sequence
    /// Bus clock on, block reset, both channels off - a setup verb, no
    /// wait. The pads are not touched: which of them this program wants in analog mode is the
    /// 12-bit right-aligned (DAC_DHR12Rx), the natural format: one store,
    /// any context, the code masked to twelve bits; false only for a
    /// channel this part has not got. With no trigger DAC_DORx follows
    /// one APB1 cycle later; with one, after the trigger (fact 2).
++ b/brio/stm32f4/delay.hpp
 * Microsecond busy-waits for the STM32F4 stratum: a short wait inside a
 * dispatch or a driver's setup, `brio::delay_us(clock, 20);`, for at
 * least that many microseconds. It spins on SysTick's VAL: it needs the
 * counter running (Ticker::init() or SysTickCounter::start()), never its
 * interrupt, so it is correct inside a masked window too; it waits, by
 * design, and is capped below one SysTick period - a millisecond or
 * more is a TimeEvent and is refused with false. The code and the
 * reasoning are cortexm/delay.hpp's `delay_us` / `delay_rate` /
 * `DelayRate`, the core-stratum file (docs/cortexm/README.md).
 *
 * What is the STM32F4's here is what holds on it: VAL is 5.6 ns of
 * resolution at 180 MHz; the core has a divide instruction (ARMv7-M's
 * UDIV), so the no-division-at-wait-time rule costs this family nothing
 * and buys it nothing - it is kept for the one code path; a counted loop
 * would be poorly deterministic behind the ART accelerator's prefetch
 * and up to five flash wait states (stm32f4/flash.hpp); and nothing here
 * survives a Stop, where SysTick stops with HCLK. The at-least contract
 * is the core file's; its numbers on this silicon are the platform
 * suite's to measure (docs/stm32f4/platform.md).
++ b/brio/stm32f4/device_tables.hpp
 * The reserve of the stm32f4 stratum: the table of what each part of the
 * family has, read off its device header, and the one file where the
 * preprocessor is allowed to ask the device header what exists. A driver
 * of this stratum asks it instead of the header - whether a port is
 * bonded, which serial instances exist and on which bus, where a DMA
 * request sits, how fast a part may run - and a program meets it through
 * the drivers' compile-time refusals, or asks it directly:
 *     static_assert(brio::gpio_port_present('G'), "this program needs port G");
 *     constexpr bool on_apb2 = brio::usart_on_apb2(6);   // the bus USART6's divisor divides
 *
 * Every entry is constexpr data or a constexpr function, read when the
 * program is compiled: nothing here touches a register, waits or masks,
 * so the question of a calling context does not arise.
 *
 * The standing rule (docs/design/overview.md, "Generalization rule"): a
 * probe that yields a value lives here and is exported as constexpr
 * data; a driver keeps `#ifdef` only to select per-instance code, never
 * to learn a fact. Nothing here is a copied table - every entry is the
 * device header's own symbol, so a variant the pack adds tomorrow is
 * described by its header and not by a list someone has to keep - with
 * one class of exception, stated where each is made: the facts of the
 * reference manual that no header carries. The frequency ladders are the
 * first (below); the DMA request mapping, which instance wears the I2S
 * face and the ADC's internal channel numbers are others, each keyed on
 * the device-select define in its own section.
 *
 * What differs across the STM32F4 family, as far as the strata reach
 *  - Which instances are full. RM0090 table 148 (and its RM0390 and
 *    the header - the U(S)ART distinction in the name is what carries
 *  - Interrupt lines: on this family a vector is mostly one
 *    F417, whose PWR_CR carries one (PWR_CR_VOS_1 is what the header
 *    SAI needs it): what stm32f4/clock.hpp's audio-PLL and PLLSAI verbs
 *    need of them is asked here (plli2s_has_m(), pllsai_has_r()), with
 *    the I2S clock selector a part has (i2s_clock_select()).
 * The frequency ladders are the exception. How fast a part may run at
 * the reference manual, not of the header, and they differ per part
 * thing about the part the preprocessor can know - and stated only for
++ b/brio/stm32f4/dma.hpp
 * RM0383 ch. 9 - one chapter, three manuals, the same IP), for a program
 * that wants data moved without the CPU: a block of memory copied or
 * filled, a peripheral register fed from a run or drained into one, a
 * serial line landing in a ring. Most programs meet this file only as the
 * engine template arguments of a transport (stm32f4/usart.hpp's Uart, the
 * SPI and I2C hosts); the smallest use of its own is a copy:
 *     using Copier = brio::DmaCopyEngine<2, 4>;   // any DMA2 stream
 *     Copier::arm(brio::DmaPriority::high);
 *     (void)Copier::copy(frame_out, frame_in, 1024);   // 1024 uint32_t
 *     while (Copier::busy()) { }
 *
 * It holds four layers, in the two strata every brio driver has:
 *
 *   Dma<n>                the block: its AHB1 gate, its reset, the two
 *   DmaStream<n, s>       the resource: one stream's six registers
 *                         the two tasks a transport wants: drain a run
 *   DmaCopyEngine         the task memory wants: copy and fill, memory to
 * (An engine is a driver layer that owns one stream and moves a run - a
 * buffer and its length - per block; a task, in brio, is a driver layer
 * with state of its own that a program or a transport calls.)
 *
 * Who calls what. The bindings - an engine's arm(), DmaStream's
 * configure(), prepare() and load(), Dma<n>'s init() and reset() - are
 * setup verbs, for main or the owner's setup, and the stream verbs that
 * reconfigure refuse while EN stands. What waits: abort(), stop() and the
 * engines' abandon() and stop() wait for EN to come down, at most 200 000
 * reads (abort()'s `spins`), and nothing else here waits - a block start
 * never polls, and refuses instead while the stream still runs. What
 * masks: DmaStream::arm() alone, a read-modify-write of SxCR and SxFCR
 * under InterruptGuard; an engine's claim() is the test-and-set a
 * transport with two racing contexts masks itself. The ISR bodies
 * (DmaStream::isr(), the engines' service() and service_ring()) are
 * called by the program's handler for the stream's vector and nowhere
 * else. A run handed to a start, and the cell of a fill or a fixed start,
 * is the caller's and is lent to the stream until the block ends - the
 * engine's complete() or busy() answering false, a receive engine's
 * take() for what has arrived - and nothing checks it; a buffer the
 * stream writes is read back by the program through a volatile access
 * (docs/stm32f4/dma.md, the compiler finding).
 *
 * A stream is not a channel, and the difference is the whole shape of
 * stream - eight per controller, each with its own FIFO, its own
 * priority and its own vector - and a stream chooses between exactly
 * eight request lines with CHSEL (10.3.3). Which peripheral sits on
 * a stream and a channel (where the G0's name a channel and a request
 * id), and why the check that the pair really carries the peripheral is
 * The level an engine arms at is its peripheral's (stm32f4/dma_engine.hpp's
 * The driver owns the fabric and not the request vocabulary (the
 * division the STM32G0 stratum draws too). This file takes a plain
 * channel number and knows nothing about USARTs; a
 * Four facts of this controller shape everything below.
 *  - EN is not a switch, it is a request. 10.3.17 step 1 and 10.3.14:
 *    finished and, on a peripheral-to-memory or memory-to-memory stream,
 *    reads 1 rather than store into a protected field. And the set side
 *  - The FIFO is a configuration, not a buffer the caller sees. Four
 *    accepts - the rest raise FEIF at the enable and disable the stream
 *  - Memory-to-memory is DMA2's alone, and no register says so. DMA1's
 *    and 34, note 1), so a DMA1 stream cannot read a memory through the
 *  - A request is a level served on enable. 10.3.2's handshake is
 *    level-driven, as the G0's is: the peripheral drives its request, the controller
 * Errata. Neither ES0206 (F427/F437/F429/F439) nor ES0298 (F446) has a
 *    FSMC or a GPIO register through the peripheral port can be
 *    destination on the memory port instead - which this driver can
    /// stream running. False when anything in the transfer was refused
    /// (nothing is enabled then); enable() reads nothing back and answers
    /// true, so a stream that dies after its enable says so in its flags
    /// (TEIF), not here.
     * this reason: HTIF is set by hardware whether or not HTIE is on,
    /// The binding: `data` is the register the runs are poured into,
    /// stops the stream (abort()'s bounded wait), writes everything a block
    /// does not change, and enables or disables the stream's NVIC line by
    /// `irqs`. A setup verb, called once by the engine's owner.
    /// The claim: take the stream for one block, or answer false because a
    /// caller with two racing contexts masks; it masks nothing itself.
    /// The binding: the gate, the stream stopped (abort()'s bounded
    /// wait), the FIFO at a full threshold, the configuration word kept. A
    /// setup verb, called once. The completion is polled (busy()), so the
    /// stream's interrupt stays off. `low` by default: no
    /// the element the beat. Returns once the block is started, without
    /// waiting for it: both arrays are lent to the stream until busy()
    /// answers false. False, and nothing started, while a block runs,
    /// transfer error is counted in faults(). Waits for nothing: a poll
    /// loop is the caller's.
++ b/brio/stm32f4/dma2d.hpp
 * knows what a pixel is: it fills, copies, converts between eleven
 * in runs, and it is a bus master of its own. A program uses it to draw
 * into a frame buffer without the CPU moving the pixels.
 * Who calls what, and from where. init() and the configuring verbs are
 * setup or an active object's handler, one context at a time. The four
 * mode verbs (fill, copy, convert, blend) and start() write the
 * registers and start the engine, then return: they do not wait, and
 * they answer false, starting nothing, on a refused description or
 * while a transfer runs. The destination belongs to the engine from
 * that start until wait() answers true or the transfer-complete event
 * is seen, and a CPU access to it in between races the engine; a
 * source must hold still for as long.
 * wait() and wait_clut() spin, bounded at busy_spins (20 000 000 polls),
 * and answer false on a stopped block. Nothing here masks interrupts.
 * isr() is the body of the program's handler for irq_line. A buffer the
 * engine writes must be volatile to the CPU, because its address
 * reaches the engine as an integer the compiler sees nothing dereference
 * (docs/stm32f4/dma2d.md, bench findings).
 *
 * The four modes are one engine with parts switched off (11.3.11).
 * buffer with no conversion, so the format field says only how wide a
 * The offsets are in pixels and the addresses are bytes. FGOR, BGOR and
 * OOR count pixels of their own format (11.5.5), added at the end of
 * The silicon checks the configuration and says so in a flag, not in a
 * refusal. 11.3.11 lists a dozen ways to be wrong - an address not
 * One transfer at a time, and a CLUT load is a transfer. CR.START is
 * It is a master and it competes. The AHB dead time (11.3.15) inserts a
 * Concurrency. Every configuring verb writes registers the silicon
    /// once before its first transfer. A setup verb; it does not wait.
    /// stopped block and not a slow one. True is the moment the
    /// transfer's buffers are the caller's again.
    /// packed into the output's own format here. Starts the engine and
    /// returns without waiting; false, nothing started, on a refused
    /// output or area or while a transfer runs.
++ b/brio/stm32f4/dma_engine.hpp
 * at, what it is armed to interrupt for, and the longest block. A program
 * meets it as a default template argument it never spells: `Uart<1,
 * console_pins>` has `NoDmaEngine` in both slots, and naming
 * `stm32f4/dma.hpp`'s `DmaTxEngine` / `DmaRxEngine` there instead is what
 * puts a transport on DMA (docs/stm32f4/dma.md). Nothing here runs code:
 * these are types and constants, read at compile time.
 * It is a tag, not a base class: `present` is the only thing a task asks
 * It lives in a file of its own and not in a dma.hpp, because a driver
 * hardware half is the stream index and cannot be configured: between
 * The level is the peripheral's (docs/design/dma.md). An engine whose
 * peripheral overruns when its stream is starved - a receive ring, an SPI
 * This arbiter's own facts bound what a copy can do. Each controller
 * What an engine's stream raises an interrupt for, chosen once at arm().
 *    proved by another. The SPI host's transmit stream is the case: every
++ b/brio/stm32f4/dsi.hpp
 * The STM32F4's DSI Host (RM0386 ch. 18) for a program that drives a
 * display over MIPI DSI: the link's physical layer powered and clocked,
 * the LTDC's frame turned into packets for the panel, and the panel's own
 * commands and reads carried beside it. On the F469/F479 class alone (the
 * exists there): `Dsi`, a monostate resource (an all-static type with no
 * instances), because a part carries one host or none.
 * That is the video-mode shape; a panel with a frame memory, such as the
 * 32F469IDISCOVERY's, takes the adapted command mode instead
 * (command_mode() in place of video(), a refresh launched by
 * ltdc_flow(true)), and docs/stm32f4/dsi.md writes that bring-up out
 * line by line. The order is the block's: the configuring verbs are
 * refused while the host is enabled (fact 4), so they come before
 * enable(), and the generic interface works only after it. The PLL's
 * input is the HSE (fact 1): the clock task has it running, and init()'s
 * second template argument states its rate, which nothing reads back. The
 * clock lane goes to high speed only after the panel's reset (fact 11),
 * and `DsiPhyConfig::clock_lane_hs` is true by default.
 *
 * Who calls what. Every verb but isr() is the main context's - setup, or
 * an active object's handler (an active object is a component of a brio
 * program with its own event queue, served one event at a time by the
 * kernel) - and the generic interface's verbs are for one context at a
 * time. Every wait is bounded (`default_spins`, two million polls: the
 * regulator's and the PLL's start, a FIFO flag, a read's answer) and ends
 * in false or `DsiStatus::timeout`; nothing here masks interrupts. A
 * write's parameters are copied into the block's FIFO before the verb
 * returns (dcs_write() and generic_write() answer `ok` when the packet is
 * queued, not when the panel has it), and a read's buffer is written
 * inside the call. isr() is the body of the program's DSI_IRQHandler and
 * reads, so clears, both error registers.
 *
 * What the block is. Three things under one register file and one
 * vector: the DSI wrapper (the regulator and the PLL that make the D-PHY
 * effect line), the DSI host (Synopsys' MIPI DSI controller: the packet
 * ways to a panel: video mode, where the LTDC's frame is packed into
 * has nothing to do but pass it on; and adapted command mode, where each
 * The facts that shape the file.
 * 1. The PLL and the PHY have ranges that two documents state
 *    differently. RM0386 18.12.5 gives the PLL's input after IDF as
 *    `dsi_pll_for()` keeps to the intersection, 8..25 MHz, and the rest
 *    output is the bit rate of one lane (the RCC's figure 17 draws the
 * 2. The unit interval is the one field the D-PHY cannot do without
 *    rounded down - `dsi_uix4()`. Every D-PHY timing the wrapper lets a
 * 3. The host counts in lane byte clocks. HSA, HBP and the line in
 *    ratio of the lane byte clock to the pixel clock and rounded
 * 4. Configuration happens with the host disabled. DSI_CR.EN = 0 holds
 *    refuses while `enabled()`, and `disable()` is the way back. What the
 * 5. The host's error registers clear on read (18.13.2): DSI_ISR0 and
 * 6. A read is a write of two packets and a turn-around. The host has no
 * 7. The data type codes are the MIPI DSI specification's, not the
 *    manual's: 18.14 assumes them and prints none, and the two panel
 * 8. Two ways to a panel, and the panel decides. Video mode streams
 * 9. In video mode the generic interface lives inside the stream. The
 * 10. What the panel reports is the panel's. The acknowledge-with-error
 *    appear, and they describe what happened since the previous read.
 * 11. The panel's receiver locks onto the clock lane's entry into high
 *    speed. Measured: a panel reset with the clock lane already running
 * The errata (ES0321 Rev 14, 2.8.1 .. 2.8.3), two of them code here:
 *    writes both clock-lane fields with the larger of the two values,
 *    over the link raises TEIF on every acknowledge trigger. Not code:
 * Three small facts the registers taught, none of them a workaround:
 * WRPCR.PLLEN drops PLLLS and raises no PLLUIF - the unlock flag is a
 * Not here, by design: the panel's own command set beyond the DCS
 * Concurrency. Every configuring verb is a plain store into a register
    /// numbers. False and nothing further when a step fails (the
    /// regulator's or the PLL's wait running out, a timing refused); a
    /// configuration outside the rules is refused at compile time. A
    /// setup verb, main context, with the HSE already running.
    /// clock lane module enabled, then the host, then the wrapper. After
    /// it every configuring verb refuses and the generic interface works;
    /// it waits for nothing.
    /// byte). `ok` means queued in the command FIFO, the parameters
    /// copied; `refused` before enable() or past 255 parameters, `timeout`
    /// when the FIFO did not free in time.
++ b/brio/stm32f4/dwt.hpp
 * The cycle counter of this family's Cortex-M4, for a program or a suite
 * that times a span in core cycles: init() once (false where the unit
 * has no counter), then a difference of two now() reads, each one load
 * and legal from any context. The device header comes first, then the
 * core stratum's CycleCounter (cortexm/dwt.hpp), whose contract and
 *  - The unit is there. RM0390 33.13 (and its RM0090 / RM0383 / RM0386
 *  - It counts HCLK, 5.6 ns a count at 180 MHz and 10 ns at the F411's
 *  - It counts through a Sleep - measured. A WFI with SLEEPDEEP clear
 *    (stm32f4/pwr.hpp, fact 6) - so the bit was cleared at run time on
 *  - Stop stops it: every clock of the 1.2 V domain is off (5.3.5), so a
++ b/brio/stm32f4/exti.hpp
 * Interrupts from pins, and from the peripheral wake-up lines, on the
 * STM32F4: the external interrupt/event controller (RM0090 ch. 12,
 * RM0390 ch. 10, RM0383 ch. 10). This family keeps its pin interrupts
 * here - there are none in GPIO, stm32f4/pin.hpp says so and stops
 * there - so every edge sense, every pending bit and every wake-up line
 * lives in this file.
 *                 the line number is the pin number, and the port is what
 *  ExtiLine<n>    one line named as a constant, the way a peripheral
 *                 wake-up above 15 is named; a line the part has not got
 *                 is a compile error.
 *
 * The smallest use, a falling edge on a button (docs/stm32f4/exti.md
 * carries it whole):
 *
 *   using ButtonInt = brio::ExtInt<brio::Pin<'C', 13>>;   // line 13
 *   ButtonInt::claim(brio::PinPull::up);   // input + pull + EXTICR
 *   ButtonInt::configure(brio::ExtiSense::falling);
 *   ButtonInt::clear();                    // before arming
 *   ButtonInt::arm(true);                  // EXTI_IMR
 *   brio::Nvic::enable(ButtonInt::irq());  // EXTI15_10
 *
 * and in the program's EXTI15_10_IRQHandler, `Exti::isr(
 * Exti::vector_lines(EXTI15_10_IRQn))` for the mask of lines that fired,
 * `ButtonInt::served(fired)` to ask it about line 13, then a post.
 *
 * Who calls what, and from where. The configuring verbs - sense(),
 * interrupt(), event(), select(), steal(), claim(), release() and their
 * line-level faces - are setup verbs: main(), or an active object's
 * handler, never an interrupt handler. Each is a read-modify-write of a
 * register every line shares (RTSR, FTSR, IMR, EMR, SYSCFG_EXTICR) with
 * no guard of its own, so two contexts configuring at once can lose one
 * another's bit. isr() is the interrupt body: the program's handler for
 * an EXTI vector calls it and nothing else should; it reads and clears
 * the pending bits of the lines it is given, one load and one store, and
 * masks nothing. pending(), clear() and clear_lines() are single loads
 * and stores (PR is rc_w1), legal from any context. trigger() is a
 * read-modify-write of SWIER, so a trigger from an interrupt handler
 * racing one from the loop can be lost. Nothing here waits; the
 * multiplexer verbs open SYSCFG's clock gate first. Every verb that takes
 * a line answers false (or reads nothing) for a line the device does not
 * implement.
 *
 * Wake-up is stated here and not built here: an EXTI line with its IMR or
 * EMR bit set wakes the CPU sub-system and, through PWR, the system
 * clocks (12.2.3). What that costs and how the clocks come back is the
 * power chapter's (stm32f4/pwr.hpp, and stm32f4/sleep.hpp's sites, which
 * arm the deep sleeps) - so the only sleep this file has anything to say
 * about is the one the core can enter unaided: WFE, which returns on an
 * EXTI CPU event (an EMR bit set, no interrupt, no NVIC line, no pending
 * bit and nothing to clear).
 * ERRATA. No item of ES0206 Rev 24, ES0298 Rev 8, ES0287 Rev 6 or ES0321
 * Rev 14 is filed against the EXTI or SYSCFG. One item filed under the
 * RTC is about this peripheral all the same, and it applies to every
 * revision of all four parts (ES0206 2.9.3, ES0298 2.10.3, ES0287 2.8.3,
 * ES0321 2.11.3, "RTC interrupt can be masked by another RTC interrupt"):
 * the effective clear of a pending bit is delayed with respect to the
 * store that clears it. A handler that
 * (`Rtc::wakeup_exti_line`, `Pwr::pvd_exti_line` and the like) and an
 * application spells it once:
++ b/brio/stm32f4/flash.hpp
 * RM0383 ch. 3): the read side the clock task needs - the access
 * latency and the ART accelerator's three switches - and the write
 * What a program uses it for: the read side only through the clock task,
 * which programs the wait states and the accelerator itself; the write
 * side to erase a sector and program a block into it, to read or
 * write-protect through the option bytes, and to read the device's
 * identity (DeviceIdcode, DeviceUid). The smallest write, the round trip
 * docs/stm32f4/flash.md explains line by line:
 *
 *     const std::optional<FlashSector> s = Flash::sector_of(0x08060000UL);
 *     if (s && Flash::unlock()) {
 *         if (Flash::erase_sector(s->number) == 0u) {
 *             FlashAccel::flush_caches();
 *             Flash::program(s->base, std::span<const uint8_t>{data, sizeof(data)});
 *             FlashAccel::flush_caches();   // now a read-back is the array's
 *         }
 *         Flash::lock();
 *     }
 *
 * Who calls what. Flash, FlashOptions, FlashAccel and FlashWaitStates are
 * monostates (static members only, never instantiated) and every verb of
 * theirs a main-context call with no guard of its own; nothing here masks interrupts. Every wait is bounded by a
 * count of poll turns, not by a time (Flash::wait_spins says why), and a
 * bound that runs out is a false or FlashFlag::refused. An erase or a
 * program on a single-bank part freezes the core for its whole duration,
 * interrupts included (below), so the caller picks a moment when nothing
 * has to be served. The bytes program() writes are the caller's span,
 * read during the call and free again when it returns. isr() is the body
 * of the program's FLASH_IRQHandler and nothing else calls it.
 *
 * Why the wait states live here and not in clock.hpp: they are the
 * What this file is not: a FlashMedia. There is no storage backend on
 * geometry pushes back - the small sectors are at the bottom of the
 * The chapter, in the shape the code takes:
 *  - One array, uneven sectors, one erase grain. A bank's sectors are
 *    the map below is computed from the flash size register and not
 *  - The program unit is the access width, and it is declared twice: in
 *    FLASH_CR.PSIZE and in the store instruction, which must agree or
 *    template parameter here - it decides which instruction the compiler
 *  - PSIZE is a claim about the supply (table 6 / table 13): x8 down to
 *    1.7 V, x16 from 2.1, x32 from 2.7, and x64 only with an external
 *    are the caller's word about its board, which no register can check.
 *  - A cell is not written once. 3.5.4: "Successive write operations are
 *  - The core is frozen while the flash works. 3.5: "Any attempt to read
 *    to stall" - and on a single-bank part the code is in the flash, so
 *    F42x/F43x class is the exception, and so is the F469/F479 class
 *    (RM0386 3.3.1, the same organization; the reserve's flash_facts()
 *    gives both two banks): a read of one bank runs while the other is
 *    erased (3.6.5), measured on the STM32F469NI. `last_wait_turns()` is what
 *  - The unlock is a keyed sequence and a wrong one is fatal until the
 *    next reset. 3.5.1: KEY1 then KEY2 into FLASH_KEYR; "any wrong
 *    sequence (provoke_wrong_key() stages a wrong one on purpose, for a
 *    suite).
 *  - EOP exists only if its interrupt is enabled. 3.8.4 bit 0: EOP "is
 *  - The option bytes are a second engine with its own key, its own lock
 *    and its own start bit, and writing them erases and reprograms the
 *    whole user configuration sector (3.6.2's note). RDP is read here
 *    and never written by any verb: level 2 is irreversible and takes
 *    FlashOptions::protect() - reaches the register through half-word
 *  - The OTP area (3.7) is 512 bytes in sixteen blocks with a lock byte
 *    each, writable once and never erasable. It is memory-mapped and
 * Errata. Nothing in ES0206 Rev 24, ES0298 Rev 8 or ES0287 Rev 6 is
 * touch the chapter from outside and both are dual-bank only, so
 *  - the order: to increase the frequency, program the new latency,
 *  - the ART accelerator (3.5.2): an instruction prefetch buffer
 *    cache (DCEN), all three clear at reset. With them off a 5-wait-
 *    turns all three on when it raises the rate. A cache may be reset
 * The three errata sheets named above carry no item against the
     * STRT, then BSY. Main context, the engine unlocked first; the answer
     * is a FlashFlag mask, zero for success, and `refused` when the
     * driver stopped before the flash was touched (no such sector on a
     * known map, the lock standing) or when BSY never fell within
     * wait_spins.
++ b/brio/stm32f4/fmc.hpp
 * wire but adds address space: memory soldered outside the die, mapped
 * into the Cortex's own map and reached by a plain load or store. A
 * program uses it to bring up an SDRAM (a frame buffer's home) or a
 * static memory once, at boot, and then treats the window as memory.
 * The smallest use, an SDRAM on bank 2 (docs/stm32f4/fmc.md has it line
 * by line, with the geometry and the timing of the STM32F429I-DISC1's
 * device):
 *
 * with the pads handed to the block on AF12 before initialize(), and
 * nothing touching the window until initialize() has answered true.
 *
 * Who calls what, and from where. initialize(), configure(), timing(),
 * memory_clock() and the static banks' init()/configure() are setup
 * verbs: main(), after the clock task (the refresh count and the
 * power-up delay are computed from the rates the clock type claims).
 * initialize() runs before SysTick and the kernel if the program wants
 * its memory first: its power-up delay is spun on a cycle count
 * (Fmc::spin_at_least()), not slept. command() and every verb built on
 * it - initialize(), self_refresh(), power_down(), normal(),
 * memory_clock() - wait for SDSR.BUSY with a bounded spin (busy_spins,
 * 100 000 polls) and answer false on a time-out rather than hanging.
 * Nothing here masks interrupts. refresh_isr() is the body of the
 * program's handler for Fmc::irq_line and nothing else calls it. The
 * memory itself, once a bank is initialized, is ordinary storage
 * reached through the volatile pointers at<T>() returns, from any
 * context - subject to erratum 2.3.5 below for a CPU read.
 *
 * Two blocks wear this chapter, and they are not the same peripheral.
 * it exists at all: `fsmc_present()` in stm32f4/device_tables.hpp (the
 * reserve: the table of what each part has, read off the device header)
 * is what a program asks to learn which block the part really has, so
 * that an absent FMC never reads as "no external memory". The rest of
 * the pack - the F401, the F410, the F411 and the 48-pin F412 - has
 * What an external memory costs that an internal one does not. Three
 * 1. The window is live before the memory is. The address decoder
 *    controller has not run its initialization sequence hangs the
 *    system with no fault (ES0206 2.3.3) - not a bus error, not a hard
 * 2. The controller is shared and serial. One access at a time, one set
 * 3. The CPU is not the only master, and the errata know it. ES0206
 *    faces of the read FIFO - one says enable RBURST (a 16- or 8-bit
 *    the FIFO off), the other says the FIFO on can serve a stale line
 *    FIFO on (SdramConfig::read_burst), because 2.3.4 has no other
 * The SDRAM clock is not a clock of the RCC. It is HCLK divided by two
 * cycles under it. This driver is deliberately not a ClockUser: the
 * refresh rate and the seven SDTR fields are the device's nanoseconds
 * What the pads are is not in this file. Every FMC signal is AF12 on
 * board actually wires - is a datasheet table and a schematic, not a
 * Concurrency. configure(), timing() and the command verbs are
 * for setup. command() and initialize() busy-wait on SDSR.BUSY with a
    /// cycles. Nothing here checks that the bank is initialized - the
    /// sentence; neither this nor memory_clock(off) leaves a device
    /// unrefreshed - only Fmc::reset() does (memory_clock() below).
    /// before SysTick and before the kernel. A setup verb, main context;
    /// it masks nothing, and each command's wait is wait_ready()'s
    /// bounded spin. The pads must already be the block's (AF12).
++ b/brio/stm32f4/fmpi2c.hpp
 * The Fast-mode Plus I2C of the STM32F4 (RM0390 ch. 23) as a bus for a
 * program: a second and entirely different I2C block some parts of this
 * family (the F410, F412, F413/F423 and F446) carry beside the three of
 * stm32f4/i2c.hpp, the one to choose for a 1 MHz bus or for a rate that
 * survives a change of the core's clock (it has a kernel clock of its
 * own). It comes in the two layers every brio bus driver has
 *   FmpI2c<n>                  the resource: the register block, its APB1
 *                              gate and reset, its kernel clock
 *                              multiplexer, its two vectors, its DMA
 *   FmpI2cHost<n, pins, ...>   the task util/i2c_bus.hpp's arbiter drives
 *                              (I2cBus, an active object that takes one
 *                              request at a time and replies with an
 *                              event), carrying the same Request fields as
 *                              this stratum's I2cHost, the speed an
 *                              FmpI2cSpeed: a program moves from one block
 *                              to the other by changing the host type, the
 *                              pads' type and any speed it names.
 * The smallest use, a bus host under the arbiter (docs/stm32f4/fmpi2c.md,
 * "How to use it", on the Nucleo-F446RE):
 *
 *     constexpr FmpI2cPins pins{.scl = {'C', 6, PinFunction::af4},
 *                               .sda = {'C', 7, PinFunction::af4}};
 *     using Bus = FmpI2cHost<1, pins>;
 *     using BusAo = I2cBus<Bus, P, 4>;   // an active object in the Tenuto
 *     Bus::init(clock);                  // after the clock task
 *     extern "C" void FMPI2C1_EV_IRQHandler() {
 *         if (Bus::isr()) { post<BusAo>(TransferDone{Bus::status()}); }
 *     }
 *     extern "C" void FMPI2C1_ER_IRQHandler() {
 *         if (Bus::error_isr()) { post<BusAo>(TransferDone{Bus::status()}); }
 *     }
 *
 * Who calls what, and from where. init(), rebase(), unstick(), recover()
 * and release() are main-context verbs; init(), unstick() and recover()
 * take the instance's two NVIC lines down while they work and enable them
 * again, release() leaves them down, and none of them masks interrupts
 * globally. init() solves TIMINGR from the kernel clock, which on the APB
 * or SYSCLK choice is the rate the clock type claims, so it runs after the
 * clock task's init(). start() is the arbiter's, from its dispatch (main
 * context), one tenure at a time - the arbiter serializes; the one wait
 * in this file on its path is a speed change's PE cycle (disable(): CR1
 * read back up to a thousand times). isr(), error_isr() and dma_isr() are the bodies the
 * program's handlers for the event vector, the error vector and the two
 * DMA streams call, and nothing else calls them. The Request's two
 * buffers are lent from the post to the I2cDone the arbiter replies with:
 * untouched and in scope until then, which the Borrowed type records and
 * nothing checks. The Fm+ pad drive is the program's to turn on
 * (fast_plus_drive()); nothing turns it on by itself. The resource's
 * configuring verbs refuse behind their own write gates (below) and have
 * no lock of their own.
 *
 * It is the STM32G0's I2C under another name. CR1/CR2 with SADD, NBYTES,
 * drives. So this file is that file, with the same verb names and the same
 * family's own: two vectors instead of one (RM0390 table 142 splits the
 * Fm+ pad drive in SYSCFG_CFGR instead of SYSCFG_CFGR1, DMA streams
 * instead of channels - and no wake from Stop (below).
 * Why every name here carries the Fmp prefix. This stratum already has an
 * to the other block: the F446 carries both, a program may use both at
 * still the arbiter's own - the descriptor's fields are the contract, and
 * One instance, and the index is there for the second. Every part of this
 * (stm32f4/device_tables.hpp: the table of what each part has, read off
 * the device header) publishes its facts without an index and
 * `FmpI2c<n>` static_asserts n == 1. The index exists so the day a part
 * arrives with two, no user of this file is renamed - and so that the host
 * task's parameter list is this stratum's I2cHost's, position for position.
 * The name is two names. ES0298 2.12.10 records it as a documentation
 * What table 127 gives this part and what it withholds. The F446's
 * independent clock and SMBus/PMBus - and a dash against wakeup from Stop
 * mode. The device header agrees in the only way it can: there is no
 * has nothing to gate here. It is the one feature of that chapter this
 * The timing register is the chapter, and the arithmetic goes both ways:
 * fmpi2c_timing_for() solves 23.4.5's conditions for a register value and
 * fmpi2c_scl_hz() prices one, both pinned at the bottom of this file
 *   - tSCL is not tSCLL + tSCLH. 23.4.9's own formula adds tSYNC1 + tSYNC2
 *     500..1000 ns of it. Its slopes are a bus fact and not a chip one, so
 *     chip's own floor under it (the filters and two kernel periods an
 *     edge, no slope), so a requested rate is a ceiling no wire exceeds -
 *     more than a short wire takes, run such a wire faster than asked.
 *   - SDADEL has no +1 and the other three do - and here the manual
 *     0x2 at PRESC 1 and 8 MHz, which is the register description's
 *   - The kernel clock has a floor and it is an erratum's. ES0298 2.12.2
 *     samples the previous SDA value; there is no register workaround, so
 *     the chooser refuses. 23.4.3's own two conditions are checked beside
 * Four more errata shape the code (ES0298 2.12, ten items in all):
 *  - 2.12.3, spurious bus error in controller mode: "Detection of bus
 *    any such transfer continues normally." So the host clears BERR,
 *    counts it (spurious_bus_errors()) and never reports i2c_bus_error
 *    client's BERR is untouched by the erratum and is reported.
 *  - 2.12.8, transmission stalled after the first byte: when the first
 *    cycles of its request, and the ratio between the APB clock and the
 *    cycled. The ratio is arithmetic this file can do, so it is published
 *    takes the first workaround by construction: the engines are started
 *  - 2.12.4, last-received byte loss in reload mode: with RELOAD set and
 *  - 2.12.1, a 10-bit address whose first byte is NACKed wedges the
 * The one verb that disables is 23.4.6's procedure. Clearing PE releases
 * PECBYTE and every status flag - and keeps every configuration register.
 * Every field the register description gates is a verb that refuses, and
 * The pads are the application's, as every bus task of this stratum has
 * alternate function the datasheet gives that signal on that pad - AF4 on
 * bench is the check. Both lines go out open drain - that is the bus - and
 * Not covered yet is at the end of docs/stm32f4/fmpi2c.md.
 * The transaction descriptor is the tenure shape util/i2c_bus.hpp
 * describes - {addr, tx span, tx_len, rx span, rx_len, reply, speed}, the
 * speed an FmpI2cSpeed - and the parameter list is this stratum's
 * I2cHost's, position for position, so a program moves from the
 * F1-lineage block to this one by changing the host type (and the pads'
 * type and any speed it names). One request is one bus tenure in the four
 * shapes I2C devices use:
 * Asynchronous whenever the wire moves: start() returns false and a
 * TransferDone follows from the ISR - even the probe, whose address phase
 * is the transaction. The two refusals that move nothing (a speed the
 * kernel clock cannot produce, a START still standing) are answered at
 * once, start() returning true (its comment below).
 *   i2c_rejected     a speed this kernel clock cannot produce - a
 *   i2c_bus_error    a START still standing at start(), the other
 *                    synchronous answer (below)
 * And no BERR ends a tenure in i2c_bus_error here, which is the
 * erratum's doing. ES0298 2.12.3: in controller mode a BERR can be raised
 * spuriously and "any such transfer continues normally"; the workaround is
 * to clear it and go on. So this engine clears BERR, counts it
 * (spurious_bus_errors()) and does not let it decide anything; the one
 * i2c_bus_error this engine reports is start()'s synchronous answer for a
 * START still standing from a tenure the peripheral never closed. A tenure that never answers is the ARBITER's to
++ b/brio/stm32f4/i2c.hpp
 * one chapter, three manuals, the same registers) as a bus for a
 * program: a sensor, a touch controller or a converter on two wires,
 * this chip the controller, or this chip answering as a target. Up to
 * three instances of the F1 lineage's event machine, in the layers
 * every brio bus driver has (docs/design/i2c-bus.md):
 *  I2c<n>                the resource: the register block, its APB1 gate
 *                        and its reset, its two vectors, and the whole
 *                        with the sequence that clears it, SMBus and PEC,
 *  I2cHost<n, pins, ...> the engine util/i2c_bus.hpp's I2cBus drives (the
 *                        arbiter, an active object that takes one request
 *                        at a time and replies with an event): one Request
 *                        is one tenure, the time the controller holds the
 *                        bus from START to STOP. The Request is
 *                        {addr, tx, tx_len, rx, rx_len, reply, speed}, the
 *                        tenure shape util/i2c_bus.hpp describes, the
 *                        speed an I2cSpeed of this block's two rows
 *                        (standard_100k, fast_400k; no Fast-mode Plus): a
 *                        program written over I2cBus compiles against this
 *                        engine unchanged as long as it names no speed
 *                        this block lacks.
 * The smallest use, a bus with one device driven from the loop
 * (docs/stm32f4/i2c.md, "How to use it", on the STM32F429I-DISC1):
 *
 *     constexpr brio::I2cPins pins{.scl = {'A', 8, brio::PinFunction::af4},
 *                                  .sda = {'C', 9, brio::PinFunction::af4}};
 *     using Bus = brio::I2cHost<3, pins>;
 *     Bus::init(clock);   // after the clock task: both rows solved for PCLK1
 *     extern "C" void I2C3_EV_IRQHandler() { if (Bus::isr()) { done = true; } }
 *     extern "C" void I2C3_ER_IRQHandler() { if (Bus::error_isr()) { done = true; } }
 *     ...
 *     Bus::start(r);      // r: one Request, its buffers lent until done
 *
 * and under the arbiter the vectors post the completion instead:
 * `if (Bus::isr()) { post<Arb>(TransferDone{Bus::status()}); }`.
 *
 * Who calls what, and from where. init(), rebase(), unstick(), recover()
 * and release() are main-context verbs: init(), unstick() and recover()
 * take the instance's two NVIC lines down while they work and enable them
 * again, release() leaves them down, rebase() touches neither, and none
 * of them masks interrupts globally. init()
 * computes the timing from the APB1 rate the clock type claims, so it
 * runs after the clock task's init(); it answers false when not even
 * standard mode can be solved for that rate. start() is the arbiter's,
 * from its dispatch (main context), one tenure at a time - it does not
 * check that the previous tenure has completed, I2cBus is what
 * serializes. It waits in its caller, bounded: about stop_drain_us
 * (5 ms) for the last tenure's STOP to leave the wire and about
 * bus_free_us (100 us) for BUSY to fall; a STOP still standing past the
 * first bound parks the tenure, no START and no vector, for the per-bus
 * timeout to answer. isr(), error_isr() and dma_isr() are the bodies the
 * program's handlers for the event vector, the error vector and the two
 * DMA streams call, and nothing else calls them; each runs to completion
 * like every interrupt handler in brio. The one wait inside them: on the
 * last byte of a read of three or more, isr() polls RxNE up to 400 000
 * times and, if that bound runs out, reads DR regardless and reports
 * i2c_ok with what it holds (receive_step(); docs/stm32f4/i2c.md lists
 * it as open). The Request's two buffers are lent from the post (or from
 * start()) to the completion: untouched and in scope until then, which
 * the Borrowed type records and nothing checks. The resource's verbs
 * have no guard of their own: a program that drives I2c<n> by hand owns
 * the sequencing.
 *
 * The machine this chapter prescribes is a set of software sequences,
 * STOP or repeated START), EV7 (RxNE: read DR) - and a receive procedure
 * that depends on the count: one byte wants ACK cleared before ADDR is
 * on and off during a tenure: TxE and RxNE interrupt only while the byte
 * because the flags that end a tenure are levels that outlive it - BTF
 * What the engine takes from the chapter beyond the sequences, each
 * two bytes itself (the data register and the shifter, TxE rising within
 * A refused last byte is closed from the Sr, not at the NACK. The NACK of
 * the Sr, and its SB is answered with the address and the write bit and a
 * The rate is the instance's own APB1 clock, never SYSCLK, and this
 * chapter needs it stated three times over: FREQ carries it in whole
 * 4 MHz Fm mode is not: init() refuses the first and marks the fast row
 * unreachable for the second, speed_ok()), CCR divides it into the
 * 3 x CCR at DUTY 2 and by 25 x CCR at DUTY 16/9, rounded up here so a bus
 * never runs faster than asked. TRISE is an argument and not an
 * assumption, as the bus edges are on the AVR's TWI: it is the wire's
 * The noise filters are not on every part. I2C_FLTR - the analog filter's
 * every part of the pack but the F405 class, and the reserve (the table
 * of what each part has, read off the device header) asks the header for
 * it (stm32f4/device_tables.hpp's `i2c_has_filter()`); the verb refuses
 * where the register is not there. Table 122's ceiling on the digital
 * filter is published as `i2c_digital_filter_max()` and is advice, not a
 * refusal: it is the bound that keeps the data hold time inside the bus
 * specification with the analog filter off, and the chapter itself allows
 * higher values to a system that can take the violation.
 * Three errata of this chapter shape the code (ES0206 2.10, ES0298 2.11,
 *  - A bus error can be detected spuriously in controller mode, and
 *    controller mode". The manual says as much of a real one (27.3.4: in
 *    error body clears it, counts it - `spurious_bus_errors()` - and lets
 *  - A START cannot be generated after a misplaced STOP, and the
 *  - The repeated START's setup time can be violated in standard mode
 *    between 88 and 100 kHz. A write-then-read Request is a repeated
 * Two optional DMA engine slots, the shape every brio bus task has: name
 * The pads are the application's, as `SpiPins` has them: SCL, SDA and the
 * datasheet gives that signal on that pad - AF4 for I2C1, I2C2 and I2C3,
 * Both lines go out open drain - that is the bus - and the pull-ups are
 * Not covered yet is at the end of docs/stm32f4/i2c.md.
    ///
    /// One tenure at a time: nothing here checks that the previous one has
    /// completed, the arbiter serializes. The Request's buffers stay lent
    /// until the completion. Waits in the caller, bounded (the STOP drain
    /// and the BUSY wait below); a drain that runs out parks the tenure
    /// with no START and no vector, and returns false.
        // Bounded (wait_until's 400 000 polls) and its answer discarded: a
        // bound that runs out reads DR regardless and the tenure reports
        // i2c_ok (docs/stm32f4/i2c.md lists it as open).
++ b/brio/stm32f4/ltdc.hpp
 * picture. A program brings it up once (the pixel clock, the panel's
 * timings, a layer over a frame buffer, a reload, the enable) and from
 * then on draws by writing the frame buffer; the controller fetches it
 * every frame with no CPU work.
 *                    two vectors, and the position counter that is the
 *                    pixel type is the beat, and the CPU writes it and
 * The smallest use (docs/stm32f4/ltdc.md has the STM32F429I-DISC1's
 * panel line by line):
 *
 *   Panel::pixel_clock(pll);                   // PLLSAI, solved at compile time
 *   brio::LtdcLayer<1>::configure(layer, qvga);   // window, format, buffer
 *   brio::LtdcLayer<1>::enable();
 * with the pads handed over and the frame buffer's memory up before it
 * (stm32f4/fmc.hpp's initialize() for an SDRAM).
 *
 * Who calls what, and from where. The configuring verbs - pixel_clock(),
 * timing(), background(), dither(), the layer's configure() and its
 * field verbs - are setup verbs, main() or an active object's handler,
 * one context at a time. pixel_clock() waits for PLLSAI to stop and to
 * lock, each wait Rcc::ready_spins polls at most, and answers false
 * rather than hanging; wait_reload() spins, bounded, for a pending
 * reload. Nothing here masks interrupts. isr() is the body of the
 * program's handler for each of the two vectors, given the mask of the
 * events that vector serves. A layer's frame buffer belongs to the
 * controller for as long as the layer is enabled: it is fetched every
 * frame, so the memory must stay valid and in place until the layer is
 * disabled and a reload has taken that.
 *
 * The clock is a PLL of its own. LCD_CLK comes off PLLSAI's R output
 * Three clock domains, and the bus stalls. 16.3.2: the layer's address
 * seven PCLK2 periods plus five LCD_CLK periods. At a 6 MHz pixel clock
 * and a 90 MHz APB2 that is some 165 core cycles a read by that
 * arithmetic, and 216 to 219 measured (docs/stm32f4/ltdc.md), which is
 * why nothing here polls a register of that domain in a loop without
 * saying so, and why `position()` costs what the bench findings say it
 * costs.
 * The shadow registers are the model. Every layer register but the CLUT
 * blanking (SRCR.VBR), and until then a read returns the active value
 * and not what was written. So a read-modify-write of a layer register
 * by software and cleared by hardware, which is what `reload_pending()`
 * watches - and a read issued straight after the reload store races the
 * What the pads are is not in this file. Twenty-eight signals at most -
 * carries which, and how many bits of each channel a board really
 * panel narrower than 24 bits takes the most significant bits of each
 * pixel format says what is in memory, never what is on the wire.
 * What this file does not do is drive a panel's own controller. A TFT
 * Concurrency. Every configuring verb here is a read-modify-write of a
    /// Bring LCD_CLK up at `lcd_hz` off the root the main PLL is using,
    /// stopped, the triple written, the PLL started and waited for (each
    /// wait Rcc::ready_spins polls at most). A setup verb, main context.
    /// GCR.LTDCEN. Everything else is configured first - the pixel
    /// clock, the timings, the layers and their reload; this is what
    /// starts the timing generator. A setup verb; it does not wait.
    /// order, with the layer left disabled and no reload asked for: the
++ b/brio/stm32f4/nvic.hpp
 * Interrupt control on the STM32F4: the critical section a program or a
 * driver opens around a decision (`brio::InterruptGuard guard;` masks
 * every interrupt until the scope ends, nests, and may be opened from any
 * context), enable_interrupts() once in main() before the kernel runs,
 * and the Nvic resource that enables, pends and reads a peripheral's line
 * by its IRQn. The device header comes first, then the core stratum.
 * InterruptGuard, the enable/disable/readback verbs, the Nvic resource
 * and irq_priority_levels are the Cortex-M's and not ST's, so they live
 * in cortexm/nvic.hpp - the file the three Cortex-M0+ families share and
 * this Cortex-M4 family includes unchanged: SysTick, the NVIC's
 * enable/pend/priority registers and PRIMASK have the same programmer's
 * model on ARMv7-M (PM0214 4.3, 4.4). This header is the STM32F4's
 * include of it, after "stm32f4xx.h" has declared the IRQn enumerators
 * and the priority width (four bits here: sixteen levels) the core file
 * is written against.
 * What ARMv7-M adds and this stratum does not use: BASEPRI, the mask
 * promise (docs/design/kernel.md, section 11, "Platform: what the machine
 * provides") is that no interrupt nests over another, and this family
 * keeps it the way the SAM C21 does: every NVIC line at the same priority
 * (the reset value, 0), PRIMASK the one mask, `Nvic::priority()` the one
 * door and no driver opens it. BASEPRI is what a preemptive kernel would
 * build on; that kernel is another type and another day.
 * It sits at the bottom of the stm32f4/ stratum: ticker.hpp and
 * ticker. Not here: VTOR (the crt's) and the fault configuration
 * (stm32f4/reset.hpp's `Faults`), the software reset (reset.hpp's, beside
 * the flags that say a reset happened).
++ b/brio/stm32f4/pin.hpp
 * The STM32F4's I/O pins (GPIO, RM0090 ch. 8, RM0390 ch. 7, RM0383
 * ch. 8): what a program uses to drive an LED, read a button, or hand a
 * pad to a peripheral. A pin is a type, `Pin<'A', 5>` for PA5, with the
 * letter and the number fixed when the program is compiled; its
 * functions configure the pin and drive or read it. The header has the
 * two faces every target's pin header offers:
 *  Port<'A'>       the port as a resource - one GPIOx block: 16-bit mask
 *                  written for a mask of pins at once;
 *  Pin<'A', 5>     the per-pin face: direction and value, the
 *                  configuring functions, the alternate-function
 *                  handoff, a PwmChannel (util/pwm_channel.hpp's
 *                  one dimmable output, here of one step: max 1, so
 *                  duty(0) is low and duty(1) high) and a PinRef
 *                  factory.
 * Which context may call what. The value functions (set, clear, toggle,
 * read) are single BSRR stores or IDR loads (toggle reads ODR and makes
 * one BSRR store), atomic by the silicon against a handler touching
 * other pins of the port, and are safe from any context, which is what
 * PinRef's set()/clear() inside a bus active object's request depend on
 * (an active object is a component of a brio program with its own event
 * queue, served one event at a time by the kernel,
 * kernel/active_object.hpp). The configuring functions (output, input,
 * analog, function, release, pull, configure_mask) are read-modify-write
 * sequences on MODER/OTYPER/OSPEEDR/PUPDR/AFR, fields with no set/clear
 * twins, so configuring two pins of one port from two contexts (a
 * handler and the loop) can lose a field: they are meant for the main
 * context - setup in main() before the kernel runs, and an active
 * object's handlers and its state machine's entry and exit actions,
 * which run there one at a time - never for an interrupt handler that
 * may interrupt another configuring call. None of them
 * waits beyond the port clock's enable readback, and none masks
 * interrupts.
 * Three facts of this family's GPIO shape the code below. The register
 * block is the STM32G0's register for register (MODER, OTYPER, OSPEEDR,
 * PUPDR, IDR, ODR, BSRR, LCKR, AFRL/AFRH), and the three facts, each the
 * STM32F4's, are where the two families differ:
 *
 * 1. There is no BRR. Bitwise reset goes through the upper half of BSRR
 *    store, still atomic against a handler touching other pins.
 * 2. The port's clock is RCC_AHB1ENR.GPIOxEN (7.3.10), off at reset:
 *    writes, silently. So every configuring verb here but pull() switches
 *    the port's clock on first (an idempotent read-modify-write of one
 *    RCC bit plus the readback that is the dummy access ES0206 2.2.7
 *    asks for); the value verbs (set/clear/toggle/read) do not, because
 *    a pin one has configured has a clocked port by construction, and
 *    pull() rewrites one field of a pin configured before it. Nothing
 *    here ever turns a port clock off - a second pin of the same port
 *    would lose its block; releasing a port is a program-wide decision
 * 3. The reset state is input floating, not analog (8.4.1: MODER resets
 *    which is not this family's reset state and is said so here.
 * Alternate functions are a per-pin 4-bit number (AFRL/AFRH), and which
 * peripheral signal AFn means on a given pad is a table of the datasheet
 * no static_assert, the program's or a driver's, can check it against a
 * header symbol, because the device header does not carry the pin table
 * at all. The bench is the check. Two rows of those tables every serial console
 * There is no pin interrupt in GPIO. Edge and level senses are the
 * EXTI's (RM0090 ch. 12), reached through SYSCFG's multiplexer, and
 * stm32f4/exti.hpp is what owns them.
 * The package fact is a port-level one, read off the device header in
 * pins of a present port a package bonds is finer than that, is not
 * tabulated, and so is checked by nothing (a Pin on an unbonded pad
 * configures a register nobody wired).
/// the bus active object (AO) and not by its client). A null PinRef (the default) means
    /// then type, speed, pull, and the mode last - the store that hands
    /// Push-pull output, low speed, no pull, driving whatever ODR holds
    /// (zero on a pad nothing has written). Opens the port's clock first;
    /// a setup or kernel-time verb (the opening comment).
    /// Output that starts at a known level: the port's clock is opened
    /// first (a BSRR store into an unclocked port is dropped in silence
    /// Change the pull alone: one read-modify-write of PUPDR. It opens no
    /// clock, so it follows a configuring call that has.
++ b/brio/stm32f4/platform.hpp
 * What the brio kernel stands on when it runs on an STM32F4: the
 * STM32F4 (Cortex-M4F) realization of the kernel's Platform concept,
 * the compiler-checked set of requirements kernel/platform.hpp states.
 * A program names it once, as the first argument of its Tenuto, and
 * every kernel template is instantiated with it:
 *     using P = brio::Stm32f4Platform<>;
 *     brio::Tenuto<P, Blinker, Supervisor>::run();
 *
 * It gives the kernel a critical section (interrupts masked for a
 * scope), an idle that sleeps until the next interrupt without losing
 * one that arrives at the wrong moment, the program's source of time, a
 * breakpoint for a panic and the storage a panic record survives a reset
 * in. Who calls what: the kernel loop calls idle() in main-loop context,
 * with interrupts already masked, and nothing else should; now() and
 * ticks_per_second may be read from any context; CriticalSection may be
 * opened from any context and nests; break_here() is the panic path's
 * and panic_record() the panic path's and the next boot's. A program
 * binds the timebase's interrupt vector itself (SysTick_Handler to
 * Ticker::tick()); the kernel binds no vector, and the crt's
 * SysTick_Handler is a weak alias of its Default_Handler spin, so a
 * program that forgets the binding stops at its first tick.
 *
 * One class, its timebase a parameter. The template parameter is the
 * stops in Stop mode, so on that timebase kernel time stands still
 * across a Stop; stm32f4/sleep.hpp's timed site repairs it afterwards
 * from the RTC's wake-up timer. The parameter is there for a timebase
 * that counts through Stop; this family has none today, so there is no
 * Tickless timebase here and the kernel's optional idle_until() hook is
 * not offered.
 * This core has BASEPRI, and this platform does not use it: the kernel
 * cause one is stm32f4/reset.hpp, and the stopping half - PWR's
 * stm32f4/pwr.hpp plus stm32f4/sleep.hpp. This silicon selects the
 * sleep depth in SCB->SCR.SLEEPDEEP and PWR_CR's PDDS/LPDS, and the idle
 * hook here writes none of them, so a WFI here takes whatever somebody
 * else armed. This header provides the storage the panic record lives
 * in, and the linker script the .noinit section it needs.
    /// The kernel's idle: entered by the loop, in main-loop context, with
    /// interrupts masked and nothing to do; sleeps until the next
    /// interrupt and returns with interrupts enabled. Not for a program
    /// to call.
    /// WFI first, unmask after. On Cortex-M a pending interrupt wakes
    /// Whatever is armed. This family selects the deeper modes with
++ b/brio/stm32f4/pwr.hpp
 * the block's bus clock, the main regulator's voltage scale and the
 * over-drive pair - the part the clock task needs, and the reason this
 * a probe. One chapter, one owner; util/power.hpp's sites over it are
 * What a program uses it for. Most programs never call it directly: the
 * clock task calls the regulator half, and a program that sleeps deeper
 * than Sleep names one of stm32f4/sleep.hpp's sites, which arms the mode
 * through `Pwr`. What a program calls by hand is the one-shot Standby and
 * the debugger's bits:
 *
 *     brio::Pwr::debug_low_power(false);           // at boot, before a sleep is weighed
 *     brio::Pwr::enter(brio::PwrMode::standby);    // prepare_standby() is inside
 *
 * Who calls what. `Pwr` is a monostate (static members only, never
 * instantiated) and every verb is a setup or main-context call: the
 * read-modify-writes of PWR_CR and SCB->SCR carry no guard of their own,
 * so one context owns them - the clock task at its init and restore, the
 * sleep site in the power manager's handler. Nothing here masks
 * interrupts. Three verbs wait, each for a ready flag and each bounded by
 * `ready_spins` (a million polls), answering false when the flag never
 * moved: over_drive_enter(), over_drive_exit() and backup_regulator().
 * arm() arms and does not sleep; enter() arms and executes the WFI (or
 * WFE) itself, and comes back only from a mode a program resumes from or
 * from a Standby whose entry conditions were not met.
 *
 * The mode is two registers in two places, and that is the first thing to
 * next WFI is a deep sleep"; PWR_CR.PDDS says which deep sleep (Stop or
 * agree before a WFI does anything but Sleep. `Pwr` owns them both -
 * Six facts of this chapter, besides the regulator's own below.
 * 1. Three modes, and only two of them are a ladder a program can resume
 *    from (5.3). Sleep stops the CPU clock and nothing else; Stop stops
 *    HSE, and keeps SRAM and registers; Standby powers the 1.2 V domain
 *    off and comes back through the reset vector with everything but the
 * 2. What comes back from a Stop is not what went in. 5.3.5: "When
 *    the wake. Everything outside the clock tree survives: the flash
 *    whole of SRAM. (Measured, docs/stm32f4/pwr.md: the VOS field itself
 *    still reads the scale the program wrote, VOSRDY down - what scale 3
 *    names is the regulator's active output, not the register.)
 * 3. A Stop is only entered if nothing is pending. Table 18's own note:
 *    happen is not an error condition anywhere in the silicon: the
 *    this stratum therefore judges it by time elapsed and never by a flag.
 * 4. Standby needs its own flag clear to be entered at all (table 19):
 *    cause. And there is an erratum on top of it: ES0298 2.2.4 (ES0206
 * 5. There is one wake-up flag for every source. Unlike the STM32G0's
 *    or any of the RTC's five events (5.4.2), so it says that the device
 *    was woken and never by what. SBF, beside it, says the device was in
 * 6. The debug bits decide whether a low-power mode happens. DBGMCU_CR's
 * What is not here. PWR_CR.DBP, the backup domain's write gate, is
 * levels are option bytes: stm32f4/flash.hpp's FlashOptions reads them
 * and writes none (provisioning is a tool's, over the debug port), so
 * they are not a register here.
 * Why the scale is a clock question. RM0090 5.1.4: the regulator's
 * that reach it - is reached only in over-drive, entered by a sequence
 * One VOS bit or two. The F405/F407 class has a single VOS bit (scale 2
 * and 1). The reserve (stm32f4/device_tables.hpp: the table of what each
 * part has, read off the device header) says which (pwr_vos_two_bits()),
 * and the code for the scale is written once against that fact.
 * The over-drive verbs are preprocessor-guarded, not if-constexpr'd:
 * The block is behind a gate. PWR sits on APB1 behind RCC_APB1ENR.PWREN,
     * Arming is not sleeping. Nothing stops here; the CPU stops when
     * Arm `m` and stop, here and now, in main context - the deliberate
     * one-shot for the mode a SleepSite will not touch (Standby, which
     * comes back through the reset vector) and for a program with no
     * kernel under it.
++ b/brio/stm32f4/quadspi.hpp
 * The STM32F4's Quad-SPI memory interface (RM0386 ch. 13, RM0390 ch. 12)
 * for a program whose board carries a serial flash on those pads: read
 * its identity, erase and program it a command at a time, and read it
 * back either through the FIFO or as plain memory in the window at
 * 0x9000 0000. One register file under one name on the F412, the
 * F413/F423, the F446 and the F469/F479, which the device header says
 * with QSPI_R_BASE; the rest of the pack has no block and nothing
 * register-facing in this file exists there: `Quadspi`, a monostate
 * resource (an all-static type with no instances), because a part
 * carries one interface or none.
 * (`fast_read_quad` is the program's read command; docs/stm32f4/
 * quadspi.md writes out a whole sequence for the MT25QL128: write
 * enable, page program, the status poll, the window.) The pads are the
 * program's: init() hands none of them to the block, so the board's six
 * are put in their alternate function first. The prescaler is applied
 * to whatever HCLK runs, so a device's ceiling holds at the rate the
 * divisor was computed for.
 *
 * Who calls what. Every verb is the main context's - setup, or an
 * active object's handler (an active object is a component of a brio
 * program with its own event queue, served one event at a time by the
 * kernel) - and one context at a time: the block holds one command. Every
 * verb waits, with a bound (`default_spins` polls, generous for a
 * device's erase), and answers what happened: init() false and nothing
 * written for a configuration outside the fields; command(), write(),
 * read() and poll() a QspiStatus (`refused` with nothing written,
 * `error` for TEF, `timeout` for a wait that ran out, a failed command
 * aborted so the block is at rest); a verb before init() is refused.
 * Nothing here masks interrupts. A caller's buffer is read or written
 * inside the call and is the caller's again when it returns. isr() is
 * the body of the program's handler for QUADSPI_IRQHandler and is called
 * from nowhere else; a program that arms the interrupt does not also
 * rely on the flags it clears (fact 7 below), which is why the blocking
 * verbs complete on BUSY.
 *
 * What the block is. A command engine in front of a serial flash: every
 * the whole thing in one of three functional modes: indirect, where the
 * program writes the registers and pumps a 32-byte FIFO; automatic
 * status-polling, where the block repeats a read on its own and raises a
 * flag when the masked answer matches; and memory-mapped, where a load
 * The facts that shape the file.
 * 1. A command starts on the last register it needs (13.3.5): on the CCR
 * 2. The clock is HCLK divided, not a clock of the RCC: PRESCALER makes
 * 3. The flash's size is part of the contract. FSIZE + 1 is the number of
 *    memory-mapped mode the same mistake is a bus error to the master
 * 4. NCS's high time is a device parameter. CSHT + 1 cycles between two
 * 5. The memory-mapped window is the manual's and not the header's. No
 *    the two classes whose manuals are on the desk and `map()` refuses on
 * 6. BUSY means different things per mode (13.3.14): in indirect mode it
 * 7. The poller and the handler must not want the same flag. TCF is what
 *    isr() reports and clears; a blocking verb that waited for TCF would
 *    be robbed by that handler and time out (measured, finish() below),
 *    so the blocking verbs complete on BUSY falling.
 *
 * The errata are the same five on both sheets read - ES0321 Rev 14
 *    indirect write with dummy cycles loses its first nibble. write()
 *    refuses a command with dummy cycles; a device that wants latency
 *    no code can cover a load a program makes from the window while the
 * Not here, by design: the flash's own command set. This file speaks the
    /// when the configuration breaks qspi_config_ok(). A setup verb, main
    /// context; it waits only for an abort of a command in flight, bounded,
    /// and hands no pad over (the pads are the program's).
     * the chapter's rules or names a data phase, or the block is not
     * enabled. Main context; it waits, bounded by `spins`, for the block
     * to be idle and then for the command to end.
     * with the block busy. Main context; it returns when the block has
     * sent the last byte (BUSY down), not when the device has written
     * them - that is the device's status, poll()'s to wait for. `data` is
     * read inside the call and is the caller's again on return.
     * Main context; it waits, bounded, and `data` is written inside the
     * call only.
    /// unmap(); a load through it with the block disabled is the bus
    /// error 13.3.13 describes, and one with the block in indirect mode is
    /// erratum 2.4.5's case (the opening comment). Null where the window
    /// is not known.
++ b/brio/stm32f4/reset.hpp
 * on the APB1 clock (ch. 22) - the four fault vectors a Cortex-M4 has
 * The smallest use (docs/stm32f4/reset.md carries it whole): the fault
 * body bound to the HardFault vector, the flags read once at boot, and
 * an independent watchdog armed and fed by the loop:
 *
 *     extern "C" void HardFault_Handler() {
 *         brio::hard_fault_reset<brio::Stm32f4Platform<>>();
 *     }
 *     const uint32_t why = brio::Reset::take_flags();   // once, at boot
 *     (void)brio::Iwdg::arm(brio::IwdgConfig{
 *         .prescaler = brio::IwdgPrescaler::div8, .reload = 0x0EEE});
 *     for (;;) { brio::Iwdg::refresh(); ... }
 *
 * Who calls what, and from where. take_flags() and clear_flags() are the
 * boot's, in main() before anything else clears the register; flags() is
 * one load, legal anywhere. The configuring verbs - Iwdg::arm(),
 * Iwdg::configure(), Wwdg::configure(), Wwdg::start(), Faults::enable()
 * and the two traps - are setup verbs for main(): each is a store or a
 * read-modify-write with no guard of its own, and nothing in this file
 * masks interrupts. Iwdg::arm() and Iwdg::configure() wait, bounded, for
 * each update to cross into the IWDG's clock domain (sync(): a poll
 * count, not a time; on a stopped watchdog the wait runs out, measured
 * at 355 ms, and the answer is false). The two refresh() verbs are one
 * store each, legal from any context. Wwdg::isr() is the body the
 * program's WWDG_IRQHandler calls. hard_fault_reset<P>() is the body the
 * program's fault handlers call; it never returns. Faults::read() and
 * take() are for a fault handler, before it hands over to that body.
 * This file owns no storage: the panic record is the platform's
 * (Stm32f4Platform::panic_record()), and a FaultRecord a handler banks
 * goes in storage of the program's own.
 *
 * full story. `Reset::take_flags()` is that cross-check,
 * Seven facts of this silicon shape everything below.
 * 1. The flags accumulate, and there is no "the cause". RCC_CSR's seven
 *    power reset only". So this register is a history and not a single
 * 2. RCC_CSR is a shared register, and the two halves have two owners.
 * 3. The IWDG of this family has no window. Four registers - KR, PR,
 * 4. The IWDG is one way in software - and a reset does stop it. 21.3
 *    cannot be stopped except by a Reset". Measured, and worth stating
 * 5. The keyed registers do not update until the watchdog is started,
 *    bit in IWDG_SR that hardware raises at the store (measured: the
 *    returns the old value (21.4.2 and 21.4.3 both say so). With the
 *    watchdog stopped the bit never drops, LSION or no LSION - 7.2.9 is
 * 6. The WWDG is the opposite peripheral in every way. It counts PCLK1
 *    four codes, /1 to /8 - it has a bus clock enable that is clear at
 *    hardware after a reset" (22.6.1). Its down-counter is free-running
 *    whole timing path measurable with WDGA never set - the way this
 * 7. Four fault vectors, three of them disabled at reset. MemManage,
 *    reset) that escalate to HardFault while they are off - which is why
 *    twelve-byte `FaultRecord` a handler can bank. No storage is owned
 *    here: where the record goes across the reset is the application's
 * Errata: the IWDG's update bits are the one item all four sheets carry
 * 2.7.1..2.7.4, ES0321 Rev 14 2.10.1..2.10.4, on every silicon revision
 * of every part: RVU and PVU are
 * Not built (docs/stm32f4/reset.md carries the list): the option bytes
 * bricks a board; and the debug freeze bits, which are read only here
    /// loop: one store, legal from any context. Also re-locks PR and RLR, which is 21.3.2's own sentence
     * One way until the next reset. After this the device reboots unless
     *
     * A setup verb for main(); it waits, bounded, inside configure().
     * False before anything is written when the setting is invalid;
     * false after the start when an update never crossed, and then the
     * watchdog is running with whatever timing it had, so the program
     * refreshes it all the same.
 * nests over another (docs/design/kernel.md section 11, "Platform: what
 * the machine provides") and a fault is
++ b/brio/stm32f4/rng.hpp
 * says of itself that it "applies to the whole STM32F4xx family"), for a
 * program that needs random words from the hardware rather than a
 * sequence it could reproduce: `Rng`, a monostate resource (a class of
 * static members standing for the one generator), because a part either
 * carries one generator or none. The smallest use:
 *     if (!brio::Rng::init(clock)) { ... }          // the domain is not running
 *     if (const auto v = brio::Rng::read()) { use(*v); }
 *     else if (brio::Rng::last_error() == brio::RngError::seed_error) {
 *         (void)brio::Rng::recover();
 *     }
 *
 * The F401, the F411 and the F446 carry none, and their device header
 * says so by declaring no RNG_BASE; on those parts the register-facing
 * half of this file is not compiled and an `Rng` spelled there is a
 * compile error (the name is not declared; the family fixture's
 * neg/rng_absent_block.cpp is the proof).
 *
 * Who calls what. init(), recover() and release() are setup verbs, for
 * main or an active object's handler, after the clock task: init() and
 * recover() wait for the first word, at most `spins` status reads
 * (100 000 by default), and answer false when it never came. read() does
 * not wait: it answers a word or nothing, the reason kept for
 * last_error(); read_blocking() is the bounded wait over it. isr() is the
 * interrupt body the program's handler for irq() calls; it clears the
 * latched error flags and leaves the word for the handler to take. read()
 * keeps the previous word as the comparison value (one word of state, no
 * guard), so the words are taken from one context. Nothing here masks
 * interrupts.
 *
 * What this block is. Several ring oscillators whose outputs are XORed
 * register handed over 32 bits at a time - a true generator with an analog
 * Six facts that shape the file.
 * 1. It has a clock of its own, and the 48 MHz domain is where it comes
 *    from. The LFSR is clocked by RNG_CLK "at a constant frequency, so
 *    `Clock::usb_hz` reports. The manual's only stated constraint on it is
 *    a ratio, not a rate: the clock monitor raises CECS when
 * 2. The clock gate is on a different bus on one part. RNGEN is bit 6 of
 *    has no AHB2 at all and puts the enable in RCC_AHB1ENR, at bit 31
 *    (`rng_clock_on_ahb1()` in the reserve, stm32f4/device_tables.hpp: the
 *    table of what each part has, read off the device header). Neither the
 *    register nor the bit is the same.
 * 3. The first value is not a random number. FIPS PUB 140-2 asks that "the
 * 4. The two errors are not the same kind. A clock error (CECS) means the
 *    used" - fix the clock tree, clear CEIS, carry on. A seed error (SECS)
 *    entropy": clear SEIS, then clear and set RNGEN to reinitialize
 * 5. The two status flags are rc_w0, not rc_w1. SEIS and CEIS are
 *    word with the bit zero and every other bit of the register as it
 * 6. One vector, and it may be shared. The interrupt is raised by DRDY,
 *    vector's name differs across the family (HASH_RNG_IRQn on the
 * Errata. No item of ES0206 Rev 24, ES0298 Rev 8 or ES0287 Rev 6 is filed
 * The one rate rule the chapter states (24.4.2, CECS and CEIS): the clock
/// because it is the design point, not enforced: the chapter's constraint
 * read() is the whole protocol: it checks the two error statuses, checks
    // The register and the bit differ by part (bit 6 of AHB2ENR, bit 31 of
    // AHB1ENR on the F410), so the store is selected by the header's own
    // symbol: per-part code, which is what a driver may keep a
    // preprocessor branch for.
    /// SECS / CECS, the current statuses: what is wrong now, as opposed to
    /// Clear one latched flag. rc_w0: the bit is cleared by writing a zero
     * 140-2 asks for - before it is handed out. Does not wait: a
     * register read or two and a compare, then the answer; the reason for
     * an empty one is last_error()'s.
     * program runs checked against 24.4.2's ratio at compile time for a
     * static clock (a DynamicClock is not checked here). Waits for the
     * first word, discard_first()'s bounded wait. False when the first
     * word never arrived - which on this block means the clock domain is
     * not running at all - or when an error status (SECS or CECS) stood
     * while waiting for it. Call it after the clock task's init().
    /// it, and the two error flags are latched, so the body reports them
++ b/brio/stm32f4/rtc.hpp
 *             (LSE with its bypass and drive mode, RTCSEL, RTCEN, BDRST)
 *             and RCC_CFGR's RTCPRE - the HSE divider that only this
 *             chapter has a use for. The LSI it may select is
 *             stm32f4/clock.hpp's Rcc (below).
 * What a program uses it for: a calendar that keeps time across resets,
 * an alarm or a periodic wake-up a sleeping chip can be woken by, a
 * timestamp or a tamper input, and a few words of memory that outlive
 * the program. The smallest use brings the domain up on the crystal and
 * sets a calendar only if none survived (docs/stm32f4/rtc.md explains it
 * line by line):
 *     brio::RtcDomain::pwr_bus_clock(true);
 *     brio::RtcDomain::unlock(true);
 *     if (brio::RtcDomain::selected() != brio::RtcClockSource::lse) {
 *         brio::RtcDomain::reset();     // RTCSEL is one-way; costs the backup words
 *     }
 *     brio::RtcDomain::lse_enable(true);
 *     if (!brio::RtcDomain::lse_wait_ready()) { ... no crystal ... }
 *     brio::RtcDomain::open(brio::RtcClockSource::lse);
 *     if (!brio::Rtc::calendar_set()) {
 *         brio::Rtc::init(brio::RtcPrescalers{}, brio::RtcDateTime{12, 0, 0, 1, 6, 24, 6});
 *     }
 *     brio::Rtc::wait_sync();
 *
 * Who calls what. Both classes are monostates (static members only, never
 * instantiated). The configuring verbs of both are setup and
 * main-context calls with no guard of their own: their read-modify-writes
 * of RCC_BDCR, RTC_CR and the EXTI's registers are one context's, and
 * nothing in this file masks interrupts. Every wait is bounded - the
 * domain's register waits by RtcDomain::ready_spins, the crystal's start
 * by lse_ready_spins (seconds at most, an argument of lse_wait_ready()),
 * every RTC synchronization by Rtc::sync_spins - and a bound that runs
 * out is a false, which is what a stopped RTCCLK looks like; read()
 * makes at most four tries before it answers false. The three ISR
 * bodies (alarm_isr(), wakeup_isr(), tamper_stamp_isr()) are called from
 * the program's handlers for the three vectors and from nowhere else; a
 * configuring verb opens the EXTI line its interrupt needs, and the NVIC
 * line stays the program's to enable. A program that uses
 * stm32f4/sleep.hpp's timed sleep site hands the RTC to it whole and
 * calls none of this itself.
 *
 * Eight facts of this silicon shape everything below.
 *
 * 1. The domain is locked at every reset, and there are two locks, not
 *    of it most RTC registers carry a second lock that only the key
 *    reset". Three things sit outside the key and inside DBP alone -
 *    Writing DBP needs a read back before its effect can be relied on
 * 2. RTCSEL is one-way, and the way back is a domain reset. 6.2.8: "Once
 *    calibrations, the timestamp, TAFCR, the twenty backup registers and
 * 3. The RTC keeps running under system reset, and that is the point of
 *    domain reset; what a system reset clears is the three shadow
 * 4. The calendar is read through shadows, or not at all - and on this
 *    silicon the shadow's locking is errata-bound. With BYPSHAD = 0 the
 *    supposed to lock the higher-order ones until DR is read (17.3.6).
 *    And the shadow is not updated in Stop or Standby: after such a sleep
 *    One more precondition, and it is arithmetic: a shadow read is legal
 * 5. Initialization mode is a stopped calendar, and entering it twice in
 *    a row is an erratum. ES0287 2.8.4 (ES0206 2.9.4, ES0298 2.9.4): INIT
 *    reading discipline, and this file applies it unconditionally on the
 *    way out: exit_init() clears INIT, then clears BYPSHAD if it was set,
 *    so that the erratum can be staged at a bench.
 * 6. Every armed thing has a write window, and the window is a flag. The
 *    flag with a bound, writes and re-enables - and returns false if the
 * 7. The interrupts reach the NVIC through the EXTI, and the lines are
 *    configurable ones. 17.5: alarm A and B share EXTI line 17 and the
 *    bit: the rising edge must be selected, the mask opened, and the
 *    pending bit cleared in the handler or the vector re-enters for ever.
 *    The three EXTI operations this needs (IMR, RTSR, PR) are
 *    stm32f4/exti.hpp's: wake_line_open() selects the edge and opens the
 *    mask through Exti::sense() and Exti::interrupt(), and serve() clears
 *    the pending bit through Exti::clear().
 *    And a shared line can swallow an event: ES0287 2.8.3 (ES0206 2.9.3,
 *    after clearing, which is why serve() below is a loop over the
 * 8. Two calibrators, and they are not to be used together. Smooth
 *    +488.5 ppm, writable on the fly; coarse calibration (RTC_CALIBR,
 * Why RCC_BDCR and RCC_CFGR's RTCPRE live here and not in clock.hpp
 * (and RCC_CSR's LSI does not). One register, one owner - and the owner
 * is this chapter.
 * the RTC domain's and not the clock tree's; RTCSEL is one-way, and a
 * sequence. The LSI is not here: it is the RTC's and the watchdog's shared
 * wake_line_* verbs below are this chapter's sequence over its verbs.
 * Errata, ES0287 Rev 6 on the bench part's revision (REV_ID 0x1000,
 *  - 2.8.2 (the calendar shadow lock) is live and is coded, see fact 4;
 *  - 2.8.3 (one RTC interrupt masked by another) is live and is coded,
 *  - 2.8.4 (calendar initialization on consecutive INIT entries) is live
 *    disabled with its pin high) has no workaround: tamper_disarm() says
 *    packages) is live and unfixable in software: PC13 is RTC_AF1, the
 *    domain's registers unpredictable) is live, and its workaround is
 * Not built: one field of the chapter, and it is RTC_CR's FMT. The
 * measured, which is a different list.
     *
     * A setup verb, main context. The calendar stands still from the
     * entry to the exit; the two waits, INITF on the way in and RSF on
     * the way out, are bounded by sync_spins. False for an invalid
     * prescaler pair or date, or when a wait ran out (a stopped RTCCLK).
++ b/brio/stm32f4/sleep.hpp
 * util/power.hpp's depth ladder on STM32F4 silicon: the sleep sites a
 * program hands to its PowerManager so that the kernel's idle WFI can be
 * a Stop and not only a Sleep. A sleep site is the target's half of the
 * power model (docs/design/power.md): it arms a depth just before the
 * kernel's idle path executes its WFI, and puts the machine back after
 * the wake. `Stm32f4SleepSite` arms one of this family's modes and puts
 * the clock tree back afterwards; `Stm32f4TimedSleepSite` additionally
 * keeps kernel time honest across a Stop by putting the RTC's wake-up
 * timer where the deadline is and handing the frozen span back to the
 * SysTick ticker. The smallest use names the plain site over the
 * program's clock task and gives it to the manager:
 *     using Site = brio::Stm32f4SleepSite<SysClock>;
 *     using Pm = brio::PowerManager<P, Site, brio::PowerConfig{}, Engine>;
 *
 * and a program whose time events must survive a Stop takes the timed
 * site instead, calls its init() and binds its vector:
 *
 *     using Site = brio::Stm32f4TimedSleepSite<P, SysClock>;
 *     extern "C" void RTC_WKUP_IRQHandler() { Site::isr(); }
 *
 * Who calls what. arm(), disarm() and armed() are the manager's, called
 * from its handler in main context: arm() just before the kernel's idle
 * path, disarm() at the manager's next dispatch after the wake. The plain
 * site's arm() writes registers and waits for nothing; its disarm()
 * re-runs the clock task when a Stop took the PLL away, which waits on
 * the oscillators' and the PLL's ready flags, each wait bounded by
 * Rcc::ready_spins (stm32f4/clock.hpp). Until that disarm(), code runs on
 * the 16 MHz HSI with dividers computed for the task's rate - the
 * interrupt handler that woke the chip among it. The timed site's init()
 * is main()'s, once, after the clock init and before the manager's first
 * round: it waits, bounded, for the LSE crystal's start
 * (RtcDomain::lse_ready_spins, seconds at most) and for the RTC's
 * synchronizations, and answers false when any of them refuses. Its
 * arm() also reads the RTC (Rtc::read(), at most four tries) and places
 * the alarm through Rtc::set_wakeup(), which polls WUTWF up to
 * Rtc::sync_spins; its isr() is the body of the RTC's wake-up vector,
 * and nothing else calls it - it runs the clock restore inside the
 * interrupt, first. resync(), shared by isr() and disarm(), holds the
 * platform's critical section (interrupts masked) for the RTC read and
 * the arithmetic that consume the baseline; nothing else here masks.
 *
 * The mechanism - which modes exist, what they gate, what wakes them - is
 * stm32f4/pwr.hpp's. The policy - when a program may stop - is
 * util/power.hpp's. What is here is the mapping between them, and the
 * Five things to read before using it.
 * 1. `light` and `none` are one code, and that is the honest answer rather
 *    asked". Between Sleep and Stop this family has nothing: 5.3 lists
 *    `armed()` - which is a pure read of the silicon - answers `none` for
 * 2. The two deep rungs are one mode with two prices. This family's Stop
 *    them apart from the registers alone. It reports the depth this site
 * 3. Standby is off the ladder on purpose, and the site enforces it: every
 *    clears PDDS - so no rung can reach Standby and a PDDS left standing
 *    util/power.hpp's model is built on the program resuming: "the
 * 4. A Stop entered with the kernel's tick armed lasts - on a board no
 *    debugger has touched. 5.3.3: a low-power mode entered through WFI is
 *    coin toss. Unless DBGMCU_CR.DBG_STOP is set, in which case the debug
 *    stm32f4x.cfg writes it at every connection. `arm()` therefore pauses
 *    the ticker for the deep rungs and `disarm()` resumes it: it costs
 *    nothing (a Stop stops SysTick anyway and kernel time was going to
 * 5. What comes back from a Stop is not what went in. 5.3.5: SYSCLK on
 *    meant for eleven times that. The site is therefore templated on the
 *    clock task and restores it - and the place it does so is the model's
 *    disarm() on the first event after the wake, which is exactly "put the
 * The plain site keeps an honest restriction: with kernel time frozen for
 * the whole Stop, a program with armed time events must not take one
 * (nothing refuses it; the events mature late by the Stop's length).
 * `Stm32f4TimedSleepSite` lifts it, inside arm() and disarm() alone, with
 *  - the alarm is the periodic wake-up timer (17.3.5), placed on
 *    TimeEvents<P>::ticks_to_next() rounded up. 5.3.7 calls this the
 *  - the witness is the calendar plus the sub-second counter, read through
 *    subtracting what SysTick itself counted leaves the frozen span, which
 * The rate rule is directional, and both halves want the same direction:
 * state an RTCCLK rate not below the true one. Over-estimating it makes
 * under-reports the span and the resync under-advances; and it makes the
 * late. Both errors land on the side the kernel's time contract allows: at
 * The other prescaler split. 17.3.1 advises a high asynchronous factor to
 * counter is this site's only way of measuring a span the kernel's tick
 * repairs can advance a tick too many and mature an event early.
 * The ISR has four acts, and every one of them is load-bearing:
 *   0. Restore the clock - fact 5 above. First, so that everything after
 *   1. Acknowledge - the EXTI line's pending bit and the RTC's own flag,
 *   2. Resync - hand the frozen span to the ticker.
 *   3. Hand the machine back to a ticking sleep. The never-early bias
 *      guarantees that kernel time is still a shade short of the deadline
 * A foreign wake - any other interrupt - does not run this body. Its
 * the Stop stays armed with the alarm still standing. That is why
 * SleepRequested{none}) is load-bearing with this site.
 * The RTC, whole: the domain gate, the clock select, the prescalers, the
 * init() enables that vector's NVIC line itself; without the line a Stop
 * is never left (docs/stm32f4/pwr.md, measured).
 *
     * Arm a rung - the manager's verb, from its handler, just before the
     * kernel's idle path; it writes registers and waits for nothing. False
     * only for a Stop configuration Pwr refuses, nothing armed then. For
     * the two deep ones this also pauses the kernel's tick
    /// The manager's verb at its next dispatch after the wake. Back to the
    /// kernel's own idle behaviour: the tick running again
     * shadow registers and enable the NVIC line. Call once, from main(),
     * after the clock init, before the manager's first round. It waits,
     * bounded, for the LSE (or the LSI) to report ready and for the RTC's
     * own synchronizations.
    /// The four-act ISR body an app's RTC_WKUP_IRQHandler binds, and
    /// nothing else calls. Act 0 re-runs the clock task in the interrupt,
    /// with its bounded waits. See the file header for why each act is
    /// there.
++ b/brio/stm32f4/spi.hpp
 * RM0383 ch. 20 - one chapter, three manuals, the same registers) as a
 * program uses them: a device on three or four wires with this chip as
 * the host that drives the clock, the same wires with this chip as the
 * client that answers, or an I2S audio stream. Up to six instances of
 * one block that wears two faces, chosen by one bit of I2SCFGR, in the
 * layers every brio bus driver has (docs/design/spi-bus.md):
 *  Spi<n>                the resource in its SPI face: the register
 *                        instance's own APB clock, the three NSS
 *  I2s<n> / I2sExt<n>    the resource in its I2S face: I2SMOD, the four
 *  SpiHost<n, pins, ...> the engine util/spi_bus.hpp's SpiBus drives:
 *                        one transaction at a time (a select, a command
 *                        phase, a data phase), polled or on the
 *                        instance's interrupt. Its Request has the
 *                        shared Request's fields by name
 *                        (docs/design/spi-bus.md, "Realizations"), the
 *                        rate and the frame size in this family's types
 *                        and in this family's declaration order, so code
 *                        that sets the fields by name over SpiBus needs
 *                        no change here.
 * The smallest use, a device that owns its bus, polled, is written out
 * in docs/stm32f4/spi.md: the pads named with the alternate function the
 * datasheet gives them (`SpiPins`), `using Bus = brio::SpiHost<5,
 * spi5>`, `Bus::init(clock)` after the clock task (`clock` is the
 * program's tag object of its stm32f4/clock.hpp Clock type, the rates
 * init() reads), the select pad an output parked high, then a
 * `Bus::Request` with `polled = true` handed to `Bus::start()`. A bus
 * several active objects share is a SpiBus over the same host: the
 * client posts the Request with a reply channel and gets a SpiDone back,
 * and the program binds the instance's vector to `Bus::isr()` and posts
 * TransferDone to the SpiBus active object when it answers true.
 * "Engine" names two things here: SpiHost is the bus's engine, the task
 * SpiBus hands one transaction at a time; its two optional DMA engines
 * (the TxEngine and RxEngine slots, stm32f4/dma.hpp) move its frames
 * without the CPU.
 *
 * Who calls what, and from where. init(), release(), rebase(),
 * bit_order(), sck_speed() and mosi_speed() are setup verbs of the main
 * context, the last four on an idle bus. start() is called from main
 * context (setup, or an active object's dispatch, which run there) - by
 * SpiBus's dispatch, or by a program driving the engine alone. A polled request completes inside it: it spins with interrupts
 * enabled under one budget for the whole transaction (400 000 polls
 * plus 4096 a frame), the instance's interrupt never armed, and a
 * Request's `cs_setup_us` is spent there too; on a stall it resets the
 * block, sets `spi_stalled` and still returns true, so status() is the
 * answer. An asynchronous request returns false at once and its frames
 * are served by isr() (and, with engines, dma_isr()), the bodies the
 * program's handlers for the instance's vector and the two streams'
 * vectors call and nothing else does; each runs to completion like every
 * interrupt handler in brio. The pump's waits are bounded spins: at most
 * 64 polls for TXE behind a frame in flight in isr(), 1000 when it
 * primes two frames (start() and isr()), and 400 000 for the shifter to
 * drain after an overrun. This header takes no critical section of its
 * own; the engines' arming is stm32f4/dma.hpp's. The Request's three
 * buffers are lent (Lease::reply: each field is a Borrowed pointer,
 * made with brio::lend<brio::Lease::reply>(buf), kernel/borrowed.hpp): a
 * polled request's are the caller's again when start() returns; an
 * asynchronous one's stay untouched and in scope from the post to the
 * SpiDone reply, a lending nothing checks. The Request object itself is
 * read through the reference, and the asynchronous path copies the
 * fields its tenure (the time the transaction holds the bus) needs, so
 * a program driving the engine alone has its Request back when start()
 * returns; a posted one lies in the arbiter's slot until the dispatch
 * ends (util/bus_master.hpp). The resource's
 * verbs carry no guard of their own: setup verbs refuse what
 * spi_config_valid() refuses, and the flag and data verbs are single
 * register accesses.
 *
 * The F1 lineage, like the USART of this family. There is no FIFO and no
 * The rate is the instance's own APB clock over a power of two, /2 to
 * Four errata of this chapter are live on every part with a manual on
 * the desk (ES0206 2.12, ES0287 2.11, ES0298 2.14), and three of them
 *  - BSY may stay high when the SPI is disabled. `disable()` waits for
 *    workaround says, and in master receive-only (RXONLY, or
 *  - Anticipated communication upon a transit from slave receiver to
 *    master: the clock starts on the MSTR write even with SPE clear.
 *    a host configuration over a client receive-only one - the first of
 *  - A wrong CRC when the polynomial is even: `spi_config_valid()`
 *  - The last received bit may be wrong when the SCK feedback is slow.
 *    The internal loop back from the SCK pad must return inside two APB
 *    `medium`, 25 at `low`. It is a pad fact and not a rate fact - the
 *    at 90 MHz of PCLK2 is above the ceiling at every pad speed, and
 * Two optional DMA engine slots, the shape every brio bus task has: name
 * the reserve's SPI slice of those tables at compile time (the reserve:
 * stm32f4/device_tables.hpp, the table of what each part has), and
 * refused on a part class whose manual was not read.
 * The pads are the application's, as `UartPins` has them: SCK, MISO,
 * datasheet gives that signal on that pad - AF5 for SPI1, SPI2, SPI4,
/// nests over another (design/kernel.md section 1, "The model in one
/// page"), so a handler that
 * The engine SpiBus (util/spi_bus.hpp = BusMaster) drives. Its Request
 * has the shared Request's fields by name (docs/design/spi-bus.md,
 * "Realizations"), the rate and frame size in this family's types and in
 * this family's declaration order: the engine asserts the request's
 * per 16-bit frame, as frame() and store() below read and write it.
        PinRef cs;   ///< asserted low around the transaction: an output
                     ///< the program parks high before the first request
     * the NVIC line. A setup verb: main context, after the clock task's
     * init (the rates below are read from the Clock type, not from the
     * hardware) and before the first request. `clock` is the app's Clock tag and the rate the BR
    /// True is "ended", not "succeeded": a polled request whose budget ran
    /// out returns true with the block reset and status() = spi_stalled,
    /// so a caller driving the engine without the arbiter reads status().
++ b/brio/stm32f4/syscfg.hpp
 *           the EXTI's sixteen pin lines, the I/O compensation cell, the
 *           Ethernet PHY interface select and the FMPI2C1's Fm+ pad drive.
 * A program rarely calls it by name: stm32f4/exti.hpp's Exti::select()
 * is what points a pin line at a port, and stm32f4/fmpi2c.hpp's
 * fast_plus_drive() what sets the Fm+ bits. The one verb a program uses
 * directly is the compensation cell's, once its pads run at the 50 or
 * 100 MHz speed class:
 *
 *     brio::Syscfg::compensation_cell(true);   // compensation_ready() then
 *                                              // says when it has settled
 *
 * Who calls what: every verb is a setup verb for main context (a
 * configuring read-modify-write, below); none waits, none masks
 * interrupts, and each opens the block's clock gate itself.
 *
 * Why it is its own file and not part of exti.hpp. On the STM32G0 the pin
 * of another peripheral, in another chapter, behind another clock gate -
 * and the switches beside it have nothing to do with interrupts.
 * The gate is the one thing to know. SYSCFG sits on APB2 behind
 * RCC_APB2ENR.SYSCFGEN, which is clear out of reset (RM0090 7.3.14): with
 * dropped in silence - the same trap as an unclocked GPIO port. So every
 * What this file does not carry. SYSCFG_PMC's ADCxDC2 bits are struct
 * members and bit masks that exist on some headers and not others; they
 * bear on the ADC, whose driver (stm32f4/adc.hpp) does not use them, and
 * they will be born with their first user. SYSCFG_CFGR - the FMPI2C1 Fm+
 * pad drive - is here, with stm32f4/fmpi2c.hpp as its user. MEMRMP is
 * read only here on purpose: moving what lives at address 0 under a
 * running program is not a verb, it is a boot decision the BOOT pins
 * already made.
 * Concurrency. Every verb is a read-modify-write of one field of one
++ b/brio/stm32f4/ticker.hpp
 * The kernel timebase of this target, on the Cortex-M4 SysTick: the
 * 1000 Hz tick every time event of a program is counted in, and the
 * default timebase of Stm32f4Platform (stm32f4/platform.hpp). A program
 * starts it once in main(), after the clock task and before interrupts
 * are enabled, and binds SysTick_Handler to its tick() - the Usage block
 * below is the whole of it. Who calls what: init() is main()'s, after
 * SysClock::init(), because the reload is computed from the rate the
 * clock type claims and not read from the hardware (init() refuses only
 * a reload that does not fit 24 bits); tick() is the interrupt body and
 * runs in the SysTick handler and nowhere else; ticks(), millis() and
 * secs() are single loads, legal from any context, and now() masks
 * interrupts for the two loads that must belong to one instant. The
 * class's own contract is documented where the class is
 * (cortexm/ticker.hpp).
 * Why SysTick and not a TIM. SysTick is core-private: no application can
 * The ticker itself is the core stratum's: `BasicTicker` lives in
 * modes do to a core-clocked timebase.
 * The reload fits. SysTick's RELOAD is 24 bits; at 1000 Hz the reload is
 * The caveat that outlives this file: Stop freezes this timebase. RM0090
 * stops and kernel time stands still for exactly as long as the sleep
 * sleep sites, and the timed site that repairs a frozen span through
 * advance(), are stm32f4/sleep.hpp's, not this file's.
++ b/brio/stm32f4/tim.hpp
 * ch. 12..14) for a program that needs a PWM output, a measured period
 * or interval, an event counter, a periodic interrupt, a one-shot pulse
 * or a quadrature encoder - in the two strata every brio target uses
 *  Tim<n>          the resource - one TIMx block: the time base
 *                  datasheet's - see below).
 *  Tasks           TimPwm / TimPairPwm (util/pwm_channel.hpp's
 * The smallest use, a 10 kHz PWM on PA6 (docs/stm32f4/tim.md, "How to
 * use it", has one example per task):
 *   constexpr brio::PinSel wave{'A', 6, brio::PinFunction::af2};  // TIM3_CH1
 *   using Wave = brio::Tim<3>;
 *   using Lamp = brio::TimPwm<Wave, 0, 999>;                      // 1000 steps
 *   brio::TimPad<wave>::claim();
 *   Wave::init();                                                 // clock on, reset
 *   Lamp::setup(Wave::clock_hz(clock) / 1000u / 10'000u - 1u);    // 10 kHz
 *   Lamp::duty(250);                                              // a quarter
 *
 * clock_hz() is the counting rate the clock type claims (fact 5), so the
 * clock task runs first: a timer counting at the 16 MHz reset rate under
 * a faster clock type runs slow by the ratio of the rates, and nothing
 * here can see it.
 *
 * Who calls what, and from where. init(), release(), configure(), the
 * channel, slave, master, break and option verbs, and every task's
 * setup(), are setup verbs: main(), or an active object's handler,
 * never an interrupt handler. They are read-modify-writes of the block's
 * registers, and init()/release() of the RCC's shared enable and reset
 * registers, with no guard of their own; nothing in this file masks
 * interrupts and nothing waits. isr(mask) is the interrupt body: the
 * program's handler for a timer vector calls it, with vector_flags() of
 * that vector on a timer that has four, and it reads and clears in two
 * accesses. set_compare() - and so a task's duty() - set_count(),
 * flags(), flag() and clear_flags() are single loads or stores, legal
 * from any context; interrupts(mask, on) is a read-modify-write of DIER,
 * so a handler and the loop that both change enables race on it.
 *
 * Six facts that shape this file.
 *
 * 1. There is one TIM_TypeDef and fourteen different timers. The device
 *    and no ETR (14.4.2), TIM6/7 have no channel (RM0090 20.4). Nothing
 *    in the header says so. What a timer is - counter width, channels,
 *    as what they are: the manuals', keyed by instance, and reached only
 *    register an instance does not implement refuses - returns false and
 * 2. The status register is rc_w0, not W1C. A flag of TIMx_SR is cleared
 *    by writing zero to it and a write of one has no effect (17.4.5), so
 *    a flag that arrives between the read and the store survives. The
 *    stratum's flag registers are cleared more than one way - the EXTI's
 *    PR is rc_w1, a one in a plain store; RTC_ISR is rc_w0 but carries
 *    its read-write INIT bit over in the value written - so the reflex
 *    learned on one of them is wrong here.
 * 3. The prescaler and the auto-reload are shadowed. PSC is copied into
 *    the working register at the next update event and never before
 *    update raised. A caller that changes the period of a running timer
 * 4. A compare register is preloaded by default here, and that is a
 *    choice. CCyPE is clear out of reset (a write to CCRy acts at once,
 * 5. The counter's clock is not the bus clock. RM0383 6.2 and RM0090
 *    with it divided the timer counts twice its APB clock - and on every
 *    `Tim<n>::clock_hz(clock)` derives the rate from the clock task's own
 * 6. The pad map is the datasheet's and nothing checks it. Which AF
 *    once for the whole stratum. So a timer pad is a `PinSel` the caller
 * Measuring with no wire. Three of this chapter's own features make a
 *  - The internal triggers. Every four-channel timer publishes TRGO and
 *    is: `tim_internal_trigger()` below (RM0090 tables 94, 98 and 101).
 *  - The option registers. TIM5_OR routes the LSI, the LSE or the RTC
 *  - A pad the timer itself drives. The chapter's forced output mode
 *    here. A GPIO in output mode does not do it: the alternate-function
 * Errata - ES0287 2.6, ES0206 2.7 and ES0298 2.6 carry the same four
 * items, so they are the family's (ES0321's 2.9.1..2.9.4 for the
 * F469/F479, in the same order):
 *    control, so TimBreakDeadTime::automatic_output_enable defaults false
 *  - "TRGO and TRGO2 trigger output failure": the slave's clock must
 *    matches in adjacent counter cycles lose the second. No workaround,
 *    and not staged at the bench (docs/stm32f4/tim.md).
     * this is `interrupt_flags` for any of its four enumerators; on TIM1 and
    ///
    /// A setup verb, after init() and after the clock task; waits for
    /// nothing. False, with the counter not started by this call, when
    /// the time base or the channel does not fit this instance.
    /// The duty, in counts out of `max` (a larger value is clamped to
    /// it). A plain store into a preloaded compare register, legal from
    /// any context: it is taken at the next update, so a change never
    /// cuts the pulse that is being produced.
++ b/brio/stm32f4/usart.hpp
 * The STM32F4's USART (RM0090 ch. 30, RM0390 ch. 25, RM0383 ch. 19 -
 * one chapter, three manuals, the same registers) as a serial port for a
 * program: the console a program prints to, a line-oriented link, a
 * byte transport to another chip, or one of the protocol shapes the
 * peripheral offers (LIN, IrDA, smartcard, a synchronous master, mute
 * mode). The header has the two strata every brio serial driver has
 * (docs/design/serial.md is the stack above them):
 *  Usart<n>              the resource: which instance (USART1..3, UART4/5,
 *  Uart<n, pins, ...>    the task: the interrupt-driven byte transport
 *                        print() and SerialPort (util/stream.hpp's
 *                        concept, which is what lets util/serial_port.hpp
 *                        and print() compile here untouched). Its options
 *                        ride one trailing template parameter and cost
 *                        nothing when left at their defaults.
 * The smallest use, the console on USART1's PA9/PA10, is written out at
 * Uart below (docs/stm32f4/usart.md carries it line by line): the two
 * pads named with the alternate function the datasheet gives them,
 * `using Serial = brio::Uart<1, console_pins>`, the instance's interrupt
 * vector bound to Serial::isr() by the program, `Serial::init(clock,
 * 115200)` after the clock task, then print(). init() computes the
 * divisor from the instance's APB clock as the clock type claims it, so
 * a port initialized before the clock task, with every bus still at the
 * 16 MHz reset rate, speaks at the wrong rate until the bus is raised,
 * and nothing refuses it.
 *
 * Who calls what, and from where. init() and release() are setup verbs:
 * main(), after the clock task and before interrupts are enabled
 * globally; init() waits for nothing beyond the bus gate's readback,
 * and release() with an engine waits, bounded, for each stream's EN to
 * read 0 (stm32f4/dma.hpp's abort()). isr() is the interrupt body: the
 * program's handler for the instance's vector calls it and nothing else
 * does, and with engines dma_isr() is the body of
 * each engine's stream vector; both run to completion like every
 * interrupt handler in brio. The transmit verbs (write_byte(),
 * write_bulk(), and print() over them) and the receive verbs
 * (read_byte(), read_bulk(), read_span() with consume(), rx_pending(),
 * harvest()) belong to the main context - setup, or the handler of an
 * active object, a brio object with an event queue the kernel serves one
 * event at a time - and never to an interrupt handler: each ring has one
 * producer and one consumer, and the interrupt body is the other end of
 * both. On the plain transport nothing masks interrupts: TXEIE is armed
 * through CR1's bit-band alias, one store a handler cannot split; with a
 * transmit engine the claim of the stream is taken masked, a dozen
 * cycles, and the block starts with interrupts on (pump_tx() below).
 * Beyond release(), nothing in the task waits except rebase() and
 * set_baud(), main-context verbs that spin until the queued bytes have
 * left the wire, each wait bounded so a wedged line cannot hang them
 * (print()'s own wait for room in the ring is util/print.hpp's).
 * Storage: the rings are the
 * task's own static arrays; write_bulk() copies the caller's span, which
 * is free again when it returns; read_span() hands out the ring's own
 * bytes, valid until consume() releases them. The resource's verbs have
 * no guard of their own: a program that drives Usart<n> beside a running
 * Uart on the same instance owns the ordering.
 *
 * The F1 lineage. This is the classic STM32 USART - SR/DR/BRR/CR1/CR2/
 * and not the STM32G0's (ISR/ICR/TDR/RDR, FIFOs, a prescaler, a
 * cleared by read sequences and not by a clear register (PE, FE, NE,
 * handler is written around those sequences: it reads SR once, decides
 * Two rings and two flags. Bytes leave through a TX ring drained by
 * an RX ring filled by RXNE. isr() returns the "RX went non-empty" edge
 * The baud divisor (30.3.4): USARTDIV = fCK / (8 x (2 - OVER8) x baud),
 * A byte that arrived with an error is dropped rather than pushed: a
 * Full and not. UART4, UART5, UART7 and UART8 have no synchronous mode,
 * The modes exclude each other the way the chapter says (30.3.8 .. 30.3.11
 * Two optional DMA engine slots, the shape every brio Uart has: name a
 * The receive engine runs circular over the whole ring, and the ring is
 * runs - there are no runs; what the consumer can lose is a lap it did
 * A stream and a channel are not free here. On this family's controller
 * not read has no table, and an engine on it is refused rather than run
 * Under the receive engine the CPU never reads DR. IDLE, ORE, NE, FE and
 * the CPU would take a byte the stream was owed. So the channel's read is
 * flags that status read saw, a flag raised after it surviving that read.
 * IDLE that way also forgets the idle the clearing frame armed - a burst
 * The burst edge comes from a vector, never from a poll. With the receive
 * clears. Waiting for the end: IDLEIE, PEIE and EIE armed; an idle line
 * clear, the edge is reported, and the vector turns to waiting for a
 * frame: those three disarmed, RXNEIE armed - the stream still takes the
 * What the channel's read cannot do: an error on the frame right after one
 * The pads are the application's: TX and RX as PinSel with the AF the
 * RTS likewise through the options. init() hands them over before the
 * interrupt-driven byte transport. The console on the Nucleo-F446RE's
 * USART2 (docs/stm32f4/usart.md has the USART1 one of the other boards):
 * `SerialLines` stands for the program's SerialPort active object
 * (util/serial_port.hpp), told of received bytes by the edge isr()
 * returns. The shared Uart surface - same verbs, same return contracts:
    /// the receive interrupt and its NVIC line. A setup verb, main
    /// context; it waits for nothing beyond the bus gate's readback.
    /// Call after the main clock is set up and before interrupts are
    /// divisor comes from the instance's APB clock as that type states
    /// it - never from a second statement of the rate, and never from
    /// SYSCLK by assumption. Nothing reads the hardware's rate back, so a
    /// call before the clock task's init() programs a divisor for a bus
    /// that is not running yet. False when the rate cannot be produced at
    /// that clock, or for nine data bits (the rings carry bytes): the
    /// caller then knows the transport is not up, rather than printing
    /// into a ring nothing will drain.
    /// The instance's one interrupt body - call from its vector, and from
    /// nowhere else.
     * Queue one byte; false when the ring is full (print() retries). Main
     * context only - the ring's one producer - and it does not wait; on
     * the plain transport it arms TXEIE with one bit-band store.
    /// (util/stream.hpp's BulkSink). Returns the number queued, which is
    /// short of the run when the ring fills; it does not wait. Main
    /// context only, like write_byte(); the bytes are copied, so the
    /// caller's span is free again on return.
++ b/brio/stm32f4/usb.hpp
 * The STM32F4's USB OTG controller in device mode (RM0383 ch. 22,
 * RM0390 ch. 31, RM0090 ch. 34 and 35), for a program that wants the
 * chip to be a USB device - most often a CDC ACM serial port a host sees
 * as a COM port and the program prints to as to a UART. This is the
 * endpoint controller util/usb/device.hpp's stack runs on: `UsbOtg<core>`,
 * realizing the `UsbController` contract at the packet, with `UsbFs` and
 * `UsbHs` the two cores this family may carry. brio's second realization
 * of that contract, after the RP2040's own controller
 * (docs/design/usb.md).
 *   using Usb = brio::UsbFs;                       // the full-speed core
 *   using Port = brio::UsbCdcAcm<Usb, P>;          // util/usb/cdc.hpp
 *   using Device = brio::UsbDevice<Usb, Descriptors, Port>;
 *   Usb::init(clock);                              // after SysClock::init()
 *   Device::start();                               // the pull-up: the host enumerates
 *   extern "C" void OTG_FS_IRQHandler() { Device::isr(); }
 *
 * (docs/stm32f4/usb.md writes the descriptors out and explains each
 * line.) The clock type must give the PLL's Q output 48 MHz, which init()
 * checks at compile time (the clock paragraph below), and the clock task
 * runs first; the vector is bound by the program, because init() unmasks
 * the core's interrupts at the block and at the NVIC.
 *
 * Who calls what. init() and release() are setup verbs, main context;
 * init() waits at least the 25 ms the chapter asks for after device mode
 * is forced, and each of the core's resets and FIFO flushes for a bounded
 * number of polls, false meaning a step did not answer. take_events() is
 * the interrupt body, reached through the stack's isr() from the
 * program's handler and from nowhere else; the stack and the class call
 * the contract's other verbs from that body and, under the platform's
 * critical section, from the main context. submit_in() is the one verb
 * that masks interrupts itself - an InterruptGuard across its register
 * writes and its push, for ES0287 2.12.1 below - and it copies the
 * caller's packet before it returns (into the FIFO, or into the
 * endpoint's own buffer when the FIFO has no room), so the caller's
 * storage is free again on return. out_data() hands out the driver's own
 * copy of the last packet, valid until the next one on that endpoint.
 *
 * What the block is. A Synopsys DWC2 core with ST's glue: a full-speed
 * FIFOs, reached through push and pop registers and never addressed
 * window per FIFO, and the packet's length travels in a transfer-size
 *   - One shared receive FIFO for every OUT endpoint. Each packet is
 *     stacked behind the last with a status entry on top of it; the
 *   - One transmit FIFO per IN endpoint, each a slice of the same RAM
 *     core, 1024 on the HS one - and refuses a configuration that does
 *   - A transfer, not a packet, is what an endpoint register describes:
 * Why the OUT side counts packets. A device that arms one OUT packet at
 * shared FIFO with no NAK in between. So `submit_out` counts credits -
 * One packet reported per pass. The receive FIFO is one queue for every
 * per endpoint). It therefore pops entries until the first data packet,
 * hands that one over and leaves the rest in the FIFO: RXFLVL is a level
 * The address goes in before the status stage, and it is the one place
 * SET_ADDRESS travels at the old address and the new one takes effect
 * puts the two steps the other way round, and measured: with the address
 * The errata that are live in device mode, and both are answered here:
 *    sequence into a transmit FIFO that is interrupted by an access to
 * The clock: the core wants 48 MHz +/- 0.25% on its own domain, which on
 * What is not here: host mode and the OTG role switch (brio has no host
 * HS core (this driver drives that core's embedded full-speed PHY and
     * block and at the NVIC. The pull-up stays down - the stack raises
     * it with connect(true) when it is ready to answer. A setup verb,
     * main context, after the clock task's init() (the 48 MHz is the
     * PLL's) and with the core's vector bound; it waits at least 25 ms.
++ b/docs/stm32f4/adc.md
This document is for a programmer who needs to measure a voltage on a
pad, the supply rail or the die's temperature on an STM32F4. `Adc<n>` is
one converter (`Adc<1>` the master), one function per register fact,
with polled conversions, the two sequences, the analog watchdog and the
interrupt body on top; `AdcCommon` is what the converters share (the
clock prescaler, the internal sources, the multi-ADC modes, the one
reset line); `AnalogIn<Pin, channel>` is a pad handed to a channel. The
converter is also the `AnalogConverter` that `util/analog_sampler.hpp`'s
sampler drives, the last example under "How to use it".

The smallest use, one conversion of a pad, the first block of "How to
use it" below:

```cpp
using Adc1 = brio::Adc<1>;
using In4 = brio::AnalogIn<brio::Pin<'A', 4>>;   // channel 4 by the datasheet's map

In4::claim();                                    // analog mode, input buffer off
brio::AdcConfig cfg{};                           // 12 bits, one conversion when asked
Adc1::init(clock, cfg);                          // the prescaler comes from the clock
Adc1::sample_time(In4::channel, brio::AdcSampleTime::cycles480);
Adc1::select(In4{});
const uint16_t counts = Adc1::read();
const uint16_t mv = brio::adc_mv(counts, Adc1::result_steps(), 3300);
```

Line by line: `Adc1` names the first converter and `In4` the pad PA4 as
channel 4, the channel derived from the map every datasheet of this
family gives (a pad off that map does not compile until its channel is
stated); `claim()` puts the pad in analog mode, which turns its digital
input buffer off; `AdcConfig{}` is the default configuration, 12 bits,
right-aligned, one conversion each time one is asked for; `init()` takes
the clock task's tag (`clock`, [clock.md](clock.md)), opens the
converter's bus clock, picks the shared prescaler from the APB2 rate
the clock type claims, writes the configuration and powers the
converter up, waiting out its stabilization time; `sample_time()` gives
channel 4 the longest sampling window, 480 converter cycles;
`select()` makes channel 4 the sequence; `read()` starts one conversion,
polls for its end and returns the counts; `adc_mv()`
(`util/analog.hpp`) turns them into millivolts against a 3300 mV
reference, the board's VREF+.

What decides correctness even there. `init()` answers false, and powers
nothing, when no prescaler code keeps the converter's clock between
0.6 MHz and its 36 MHz ceiling at the APB2 rate the clock type claims;
the smallest use ignores the answer, and a program that does the same
on a clock with no legal division reads 0 from a converter that was
never powered. `select()` and `read()` without an
argument answer nothing: `select()` is void because the sampler's
contract asks for it, and it refuses in silence while a conversion is in
flight (`select_sync()` is the same with the answer); `read()` returns 0
when its bounded poll (0x100000 turns) ends with no conversion, a value
a real input can also read, so a caller that must tell the two apart
uses `read(uint16_t&)`, which answers false. The sampling time is the
caller's to choose: every channel comes out of reset at 3 cycles, the
shortest, and nothing in `init()` changes it. And the reset line is not `init()`'s: one line resets every
converter and the common block together, so `AdcCommon::reset()` is a
separate verb the program calls when no sibling converter is at work.

The page is long because the chapter is. In order: the documents of
record; what the silicon does, with the two errata and how they are
answered; the types and verbs (the reserve's facts, the vocabulary, the
input types, the common block, the converter); how to use it, one
example per use; the bench findings; what is not covered.


## Local batch: the samc21 stratum

Scope: the twenty-seven peripheral documents of `docs/samc21/` (every
page but README.md and vendor/README.md) in the brief's additive pass,
and the comments of all thirty-four headers of `brio/samc21/`. Base:
`docs-rewrite` at a70d4b2 (a70d4b23b8e46be0fb89bff8890f1b8b8214bcd2),
the reference of every token check; local branch `docs-samc21`, no
commit, delivered as a patch. Not touched: every code token, apps,
CMake, presets, ld, the crt, svd, `.vscode`, `.clangd`, every other
stratum, the design pages and the learning route, docs/samc21/README.md
and vendor/README.md, CLAUDE.md, docs/README.md. No `// build:` or
`// mcu:` line exists in the headers. Environment: a local machine with
the toolchains under /sw, none used (no build, no host test, no board);
the token check lexed with the host g++.

Method: the route's pages and headers first, by hand (platform.md,
clock.md, port.md, evsys.md, eic.md, osc32kctrl.md, freqm.md;
platform.hpp, ticker.hpp, nvic.hpp, delay.hpp, sleep.hpp, clock.hpp,
pin.hpp, evsys.hpp, eic.hpp, osc32kctrl.hpp, freqm.hpp,
device_tables.hpp) as the pattern; the other twenty documents and
twenty-one headers by six forks of the same session in groups (SERCOM
USART + DMAC; SPI + I2C; TC + TCC + RTC; ADC + DAC + SDADC; AC + TSENS +
SUPC + CCL; RSTC + NVM + PAC + DSU + DIVAS + MTB), each under the same
written instruction, each reporting its edits, its corrections and what
it could not establish; their diffs checked by the scripts below and
spot-read (sercom.md, i2c.md, supc.md, tcc.md, tc.hpp, spi.hpp,
pac.md).

### The documents

Each of the twenty-seven opens with: whom it is for and what the driver
does in a program; the smallest use, an excerpt of the page's own "How
to use it" (or "One example per use") block, explained line by line;
the prerequisite that decides correctness even there, each claim
checked against the header; and a reading guide on the long pages.
"Documents of record:" became a heading on every page, with a suite's
letter (and the all-key `z` where the page uses it) defined there. The
terms defined where the pages use them: a concept, a task, an active
object (and AO), a monostate, the reserve, the pull-walk, a tenure, the
arbiter, a lend, a meter, the panic breadcrumb. Capitals used for
emphasis lowered in the orientation, the documents of record and the
first section (and in "Types and verbs" / "How to use it" prose where
cheap); acronyms, register names and quotations kept. Nothing
reordered; every "Bench findings" section byte-identical to the base
(checked by script); "Not covered yet" untouched but for two
parenthetical definitions (port.md: the reserve) and one stale name
(reset.md, below); platform.md's "Sleep, peripheral by peripheral"
gained one parenthetical definition of the pull-walk. Excerpts: every
one is the page's own block line for line (checked by script; four are
indented blocks in the source, identical modulo the indentation),
except dmac.md, which has no "How to use it" of its own and quotes the
engined console block of sercom.md.

The prerequisites stated, each checked against the code: the clock task
first, because the drivers compute their dividers from the rate the
clock type claims and the CPU runs at 4 MHz out of reset (a ticker or a
port set up before it runs twelve times too slowly until the clock
moves, and at the rate asked after - derived from the arithmetic, not
measured); every vector the program does not define is a weak alias of
the crt's silent spin (`Default_Handler`), so an unbound, misspelled or
C++-named handler stops the program at the first interrupt with no
build error; which bus clocks are on out of reset (read from the
device header's MCLK_*MASK reset values: PORT, OSC32KCTRL, FREQM, the
NVMCTRL on; EVSYS off); what each `init()` refuses and that its false
is the only warning; what waits, with its bound in polls, and what
masks; the buffers lent until the reply; the enable-protected verbs
answering false while a block runs; a TCC's second `duty()` inside one
period dropped; the plain sleep site's rule (standby only with no time
event armed).

### The headers

Comments only, every one of the thirty-four token-identical to the
base. Each opening now says first what the component does for a
caller and the smallest use, then which context calls what (setup
verbs in main after the clock task, the vector bodies called from the
program's handlers and nowhere else, the ring verbs' one producer, the
waits and their bounds, what masks), who owns a lent buffer until when,
then every fact the old comment gave with its citation, capitals
lowered. The public operations the smallest uses call gained the
sentence they lacked (context, what false means).

### Stale claims corrected to the code's or the page's truth

- osc32kctrl.hpp said XOSC32K "has not run on silicon"; osc32kctrl.md
  measures its start and its frequency on a board with the crystal and
  the false return on one without. Now said, with what has not run.
  osc32kctrl.md's "the RTC driver when it exists" (it exists, and
  states the RTCSEL ordering as its caller's obligation) and the
  header's "the ordering is samc21/rtc.hpp's to keep" said that way.
- `Osc32k::init()`: "false - and nothing written" also for a ready flag
  that never rose; the register is written then. Corrected.
- freqm.hpp: "`measure()` returns nothing rather than a wrong number"
  for a reference faster than the measurand; the code returns the zero
  count. Corrected.
- device_tables.hpp's consumer list lacked samc21/tsens.hpp, which
  includes it. Added.
- ticker.hpp's "the `advance()` verb below": it is BasicTicker's, in
  cortexm/ticker.hpp. Said so.
- sercom.hpp's `rx_skips()` "modulo 2^32, one per character":
  `SkipRing::skips()` is a `uint8_t` stepping once per skip event.
- spi.hpp: erratum 1.17.16 not reproduced in SPI mode and 1.17.19 not
  reproduced (DBGCTRL reads 0x1 across SWRST), as spi.md measures; a
  garbled duplicate comment line (`// The pads go        status_ =
  spi_ok;`) dropped, the statement above it untouched.
- i2c.hpp: `I2cHost` and `I2cClient` have a third template parameter
  (`generator`); erratum 1.17.16 in I2C mode is measured not
  reproduced; "always asynchronous" against `start()`'s synchronous
  returns.
- tc.hpp / tcc.hpp: "sleep, which samc21/sleep.hpp owns" - sleep.hpp
  writes no TC or TCC bit; RUNSTDBY is `TcConfig`/`TccConfig::
  run_standby`, ONDEMAND has no owner. tc.hpp's erratum 1.20.2
  "carries the measurement": tc.md declines the verdict. One READSYNC
  described where `read_sync()` issues two.
- adc.hpp: ADC0's SYNCBUSY "which the erratum does not touch" - it is
  the bit erratum 1.4.10 sticks.
- dac.hpp: `Dac::adc_input_pad_hint` does not exist
  (`adc_input_pad_port`, `adc_input_pad_pin`); sdadc.hpp:
  `sdadc_gain_of()` and `sdadc_shift_for_gain()` do not exist
  (`sdadc_gain_permille()`, `sdadc_corrected()`).
- supc.hpp: VREFOE said to be what the comparator's bandgap input
  needs; supc.md measures it routing the bandgap to an ADC channel only.
- reset.md's gap list named `ResetCause::bodvdd`/`bodcore`; the enum
  spells `brown_out_vdd`/`brown_out_core`. reset.hpp's gap pointer named
  platform.md; the list is reset.md's.
- pac.md's "How to use it": `Pac::lock(Nvm::pac_id)` does not compile
  (no such member; now `ID_NVMCTRL`, 34 in the device header) and
  `PAC_Handler` binds nothing (no such vector; now `SYSTEM_Handler`,
  line 0).
- divas.hpp named a `divide()` that does not exist
  (`divide_unsigned()`, `divide_signed()`).
- eic.md/eic.hpp, clock.md: no stale claim; the excerpt of
  device_tables.hpp's own example says PB22 is line 6 on the G and J
  and absent on the E (the E's pin header has no PB22).

### Open facts

- Erratum 1.4.10's bring-up order: its workaround says ADC0 before
  ADC1, adc.md's bench finding says ADC1 first; adc.md's orientation
  points at both and picks neither.
- PM on IRQ 0: pac.hpp (citing table 10-4) lists PM among the blocks on
  line 0; the device header's vector comment and sleep.hpp (the PM has
  no interrupt) do not. Left as written.
- spi.hpp says enabling the peripheral clears CTRLB.RXEN (32.8.2),
  spi.md that it raises SYNCBUSY.CTRLB; both kept, 32.8.2 not re-read.
- What `Ac::init()` does on a generator that is not running (bounded
  waits, false; the failure mode unmeasured).
- The twelfth-rate figure for a driver set up before the clock task is
  derived, not measured.
- The two prose-net review items, ac.md:91 and ac.hpp:117 ("no PTC
  driver"), are true claims of absence and stay.

### Reader review (fresh subagents)

Round 1, two readers, each given only its files, the audience profile
and the brief's four questions with scenarios: one on platform.md and
sercom.md (eleven scenarios), one on the comments of pin.hpp and
i2c.hpp (eleven). Supported directly: the unbound SysTick vector and
when it bites, the pending interrupt before the WFI, the time event
across a plain-site standby (199 ms late) and the timed site as the
answer, the 4000-byte print and who is held, the unbound console
vector, `write_byte` from a handler, the port initialized before the
clock, the caller's buffer after `write_bulk`, the TxD pads and the
compile-time check; output() after an input pull-up, a toggle beside a
configure and the shared PMUX byte, `Pin<'C', n>`, no clock before the
first pin call, the I2C buffers' lifetime, what `init()` refuses, the
vector and its body. Residue, applied: platform.md's "twelve times too
slowly" read as permanent (now: until the clock moves, right after);
`extern "C"` not said to be what keeps the vector's name (now, with the
silent consequence of a dropped or misspelled one, in platform.md and
sercom.md); the power block's `Pm` beside the PM driver's `brio::Pm`
and what `ticks_to_next()` answers (now said); sercom.md's smallest use
not ending in the kernel (now said) and the write verbs' returns (now
stated); pin.hpp's undefined `Rx`, "PwmChannel"/"PinRef factory",
"context" and "not built"; i2c.hpp's active object, AO, arbiter, timed
arbiter, `P`, lend, "the post", "generator 0 on a clean wire", the
crt's weak spin, `start()`'s inverted-looking return, how an
unreachable speed reaches the requester, the pads' missing internal
pull and the NVIC line `init()` enables. Left as is, with the reason:
the C++ notation (templates, `constexpr`, the digit separator) is the
learning route's; how to declare an active object and the Request's
fields are the kernel's and util/i2c_bus.hpp's pages; the full pin
table per package is a stated gap; the timed site has its own section.

Round 2, one fresh reader on the revised passages only (platform.md's
and sercom.md's orientations, i2c.hpp's opening), eight scenarios:
seven supported directly (the tick rate temporary and why, `extern
"C"`, `Pm` against `brio::Pm` and the empty `ticks_to_next()`, the
write verbs' returns, the six terms of i2c.hpp, the pads' pull and the
NVIC line, the crt's weak spin), one with a small inference (that the
arbiter's reply to a synchronous `start()` is the same `I2cDone`).
Residue, applied: sercom.md's "printed from a handler" (now an active
object's handler, and `print` never from an interrupt handler);
i2c.hpp's "bundled wire", "the owner" (now the lender),
`measured_rise_ns` and SYSOP before their definitions. Left as is: the
power block's vocabulary (`PowerConfig`, the voters, `SleepVote`,
`reply_to`) is util/power.hpp's and design/power.md's; "the stopping
half", "the flash placement" and "the engine's claim" are explained in
the sections the orientations point at.

### Checks

- `bin/brio prose docs/samc21/*.md brio/samc21/*.hpp`: 62 files, 0
  errors, 2 for review (before the batch: 0 errors, 2 for review; the
  same two true claims of absence, moved to ac.md:91 and ac.hpp:117).
- `bin/brio gate --tokens --against a70d4b2 brio/samc21/*.hpp`: all
  thirty-four token-identical.
- Links and anchors of the twenty-eight pages of docs/samc21 and the
  vendor page, by a script with GitHub's anchor rules: 0 problems. The
  one heading renamed (i2c.md's headline, split over two `##` lines in
  the base and so rendered as two headings) has no incoming link.
- A removed-token script (every number, backticked identifier and
  register-like word of a removed line present in the new text): no
  loss but the deliberate corrections above.
- A section script: every "Bench findings" section identical to the
  base; "Not covered yet" changed in port.md and reset.md only, as
  listed.
- `grep -P '[^\x00-\x7F]'` over the touched files: nothing; no
  `// build:` line in any header.
- `git diff --stat a70d4b2 -- . ':!docs/samc21' ':!brio/samc21'
  ':!DOCUMENTATION_REWRITE_PROGRESS.md'`: empty.
- Not run: any build, `brio check samc21`, the host tests, the full
  `brio gate`.

### Local handoff

- The full `brio gate` against a70d4b2 confirms every image of the
  samc21 presets unmoved (comments only; the token check says so, the
  gate proves it); `brio check samc21` for the fixtures' sake.
- The open facts above, for the desk and the datasheet: erratum 1.4.10's
  order, PM on IRQ 0, 32.8.2's RXEN note.
- The excerpts are copies of each page's own block; pac.md's block now
  compiles in principle (`ID_NVMCTRL`, `SYSTEM_Handler`) but no fixture
  compiles it - a re-read after any API change is the
  maintained-example rule applied to them.

++ b/brio/samc21/ac.hpp
 * The SAM C21 Analog Comparators (DS60001479M ch. 40): whether a
 * voltage is above or below a threshold, with no conversion - a pad
 * against a fraction of VDD or against another pad, a signal inside or
 * outside a window - read by the program, raised as an interrupt, or
 * sent on the event system. The smallest use, shortened from
 * docs/samc21/ac.md's "How to use it" (which also routes the output to
 * the CMP0 pad):
 *     brio::Ac::init(1);                        // GCLK_AC from generator 1
 *     using Comp = brio::AcComparator<0>;
 *     Comp::configure({.positive = brio::AcPositive::pin0,
 *                      .negative = brio::AcNegative::vscale});
 *     Comp::scaler(31);                         // threshold VDD/2
 *     Comp::enable(true);
 *     while (!Comp::ready()) { }
 *     bool above = Comp::state();
 *
 * Who calls what. init(), release(), configure(), enable(),
 * event_config() and the window's configure() are setup verbs, for
 * main() or an active object's handler (an active object is a brio
 * component with its own event queue, served one event at a time by the
 * kernel): each waits on SYNCBUSY with a bound (0xFFFF polls by default)
 * and answers false when the wait runs out, and none masks interrupts.
 * The generator init() is given must already run - it clocks every
 * digital stage of the block. scaler(), start(), ready(), state() and
 * the flag and arming verbs are single loads or stores, legal from any
 * context. take_flags() is the interrupt body: the program's AC_Handler
 * calls it (the crt's default for an unbound vector is a silent spin);
 * init() leaves the NVIC line disabled and the program enables it.
 *
 * Three faces:
 *
 *  Ac              the block: APB clock, the one GCLK channel
 *                  (GCLK_AC clocks all the digital - sampling, filter,
 *                  only while its enable is low (every COMPCTRL field
 *  AcWindow<w>     the comparator pair as a window: WSTATE, the four
 * What the chapter says about time, collected here because the bench
 *    the filter section states the sampling rate is GCLK_AC;
 *    no cycle count given for the synchronous path anywhere;
 * Window mode (40.6.4) is the second face of every comparator pair:
 * mode (COMPCTRLn.SINGLE) and the same I/O pin as positive input - so
 * The event surface (40.6.13) runs both ways and this header publishes
 *  - Out: `comparator_generator(n)` (a copy of the comparator status)
 *  - In: `start_user(n)`, the SOCn user that starts a comparison,
 *    29-3 marks all four SOC users asynchronous path only, which is a
 * EVCTRL is enable-protected (block level), so `event_config()` refuses
 * Per-package input legality: the pair, not the comparator, owns the
 * Errata, DS80000740S, read on the E/G/J row at revision F. Of six AC
 * items plus one device-level one, two are this silicon and neither can
 *    workaround is to use the VDD scaler instead. This driver does not
 *  - Not this silicon: 1.5.1 and 1.5.4 (hysteresis, revisions B..E and
 *    B), 1.5.2 (low-power mode with hysteresis, B..E - the pairing is
 *    AC must borrow GCLK_ADC1's channel - revision B only, so `clock()`
 * Sleep is measured, not built here: the 40.6.14 sequences both ran
 * its own edge, and COMPCTRL.RUNSTDBY gates the comparator, not merely
    /// A setup verb; `gclk_generator` must already run (its rate is the
    /// sampling rate of everything digital here). False when a channel
    /// or synchronization wait ran out; the NVIC line is left disabled.
++ b/brio/samc21/adc.hpp
 * A voltage read on the SAM C21: a pad, the supply, the bandgap or the
 * DAC's output, once or paced, averaged or oversampled, watched against
 * a window. The chip's two 12-bit SAR converters (DS60001479M ch. 38)
 * are one resource template, `Adc<n>` - ADC0 and ADC1 are the same
 * peripheral at two addresses, so there is one type and the instance
 * number carries every difference the silicon has (which pads, which
 * GCLK channel, which EVSYS codes, which half of the host/client pair).
 * The smallest use, a single reading:
 * Who calls what. init() and release() are setup calls in main(), after
 * the clock task's init() (the generator's rate passed to init() is a
 * claim the prescaler check trusts); init() answers false for a
 * configuration the chapter forbids, a prescaler out of range for the
 * rate given, or a synchronization that never settled. Every wait is
 * bounded: a synchronization 0xFFFF polls, a conversion in read() and
 * discard() 0xFFFFF. read(), select_sync() and the configuring verbs are
 * main-context calls (setup, or an active object's handler - an active
 * object being a brio component with its own event queue, served one
 * event at a time by the kernel); select(), start() and flush() are
 * single stores that wait for nothing. isr() and resrdy() are the bodies
 * the program's ADCn_Handler calls, and nothing else does. Nothing here
 * masks interrupts and nothing keeps a caller's storage. The pad is not
 * touched by select(): claim_pad() is 38.5.1's analog configuration for
 * a program that wants it.
 *
 * One type with knobs, because every use - a single reading, a paced
 * The four register disciplines, spelled per register because this
 * 1. Enable-protected (38.6.2.1) - writable only with CTRLA.ENABLE = 0,
 *    and not synchronized: CTRLB (the prescaler), REFCTRL, EVCTRL,
 *    CALIB. Every verb here that touches one of them refuses while the
 * 2. Write-synchronized (38.6.8) - CTRLA.SWRST, CTRLA.ENABLE,
 * 3. Double-buffered (38.6.3.3) - and this is the one that surprises:
 *    OFFSETCORR are also buffered, so a write made during a conversion
 *    is held until the next RESULT and its SYNCBUSY bit stays set for
 * 4. Neither - DBGCTRL, which is also not reset by CTRLA.SWRST
 * The RESSEL / AVGCTRL interplay, the chapter's classic trap.
 * Accumulating or averaging more than one sample requires
 *  - above 16 samples the hardware already right-shifts the sum to keep
 *  - ADJRES is an additional right shift the software asks for, and it
 * Calibration is not optional. 38.5.10: "The BIAS and LINEARITY
 * The clock. CLK_ADCn = GCLK_ADCn / 2^(1 + CTRLB.PRESCALER), minimum
 * so at a 48 MHz generator DIV2 is illegal (24 MHz) and DIV4 is the
 * no ClockUser here: on this family the converter has its own generic
 * INTREF needs a long sample. 38.8.9's MUXPOS description: "If the
 * 10 us minimum. `adc_samplen_for_ns()` turns a wanted sampling time
 * Errata, DS80000740S, read on the E/G/J row at revision F. Ten ADC
 * items; five are this silicon.
 *  - 1.4.4 Synchronized Event (all revisions): a synchronized event
 *    arriving during a conversion is never acknowledged and stalls the
 *    event channel. The workaround is "only the asynchronous path from
 *    requires for both ADC users anyway. This is code here:
 *    SYNCBUSY.SWTRIG becomes stuck at one after a wake from standby, and
 *    stores that wait for nothing, and `ready()`/`result()` are the only
 *    the first conversion of a sequence that exits standby. Stated on
 *    it is stated on `enable()`: each instance waits on its own
 *    SYNCBUSY, with a bound, so nothing here hangs - but ADC0's own bit
 *    is the one the erratum sticks, and on the bench the state, once
 *    entered, makes `Adc<0>::enable()` and `init()` answer false and a
 *    software reset does not clear it (docs/samc21/adc.md, "Bench
 *    findings"); the order the converters come up in is the caller's.
 *  - Not this silicon: 1.4.1 (SWTRIG.START never clears), 1.4.2 (the LSB
 * The host/client pair (38.6.3.1) is real and typed: ADC1 is the only
 * Not built (docs/samc21/adc.md carries the list): the converter as a
 * wake source - table 38-4 is measured, but RESRDY, WINMON and OVERRUN
     * Erratum 1.4.10 (all revisions): enabling ADC1 while ADC0 is
     * disabled can leave ADC0.SYNCBUSY.ENABLE stuck at one. This waits
     * on its own instance's bit, bounded by `spins`, so nothing here
     * hangs - but on ADC0 that bit is the one the erratum sticks, and
     * then this answers false with the converter not enabled (measured,
     * docs/samc21/adc.md). A caller that reads ADC0's SYNCBUSY after
    /// The convenience form, for a converter known to be running: 0 when
    /// no result came within the bound. Main context; it spins.
++ b/brio/samc21/ccl.hpp
 * that turn a LUT pair into a flip-flop or a latch. Glue logic with no
 * with no interrupt and no DMA (37.5.4 and 37.5.5 are both "Not
 * applicable"), so the only ways out are a pad and an event. The
 * smallest use, an exclusive OR of two pads on a third
 * (docs/samc21/ccl.md, "How to use it"):
 *
 *     using A = brio::CclIn<brio::Pin<'A', 16>>;    // CCL0/IN[0]
 *     using B = brio::CclIn<brio::Pin<'A', 17>>;    // CCL0/IN[1]
 *     using Y = brio::CclOut<brio::Pin<'A', 19>>;   // CCL0/OUT[0]
 *     brio::Ccl::init();                            // no generic clock needed
 *     A::claim(); B::claim(); Y::claim();
 *     brio::Lut<0>::configure({.in0 = brio::LutInput::io,
 *                              .in1 = brio::LutInput::io,
 *                              .truth = brio::lut_truth(
 *                                  [](bool a, bool b, bool) { return a != b; })});
 *     brio::Ccl::enable(true);
 *
 * Who calls what. The verbs here are setup verbs, for main() or an
 * active object's handler (an active object is a brio component with
 * its own event queue, served one event at a time by the kernel), bar
 * the readbacks (CclOut::read(), config(), enabled()), which are single
 * loads; none masks interrupts. init() waits only for the generic clock
 * channel, bounded (0xFFFF polls), and answers false when that runs
 * out; the configuring verbs (configure(), truth(), the sequencer
 * selector, run_standby()) answer false and write nothing while the
 * block is enabled (below), so the program configures first and enables
 * last; Lut<n>::enable() alone works on a running block. There is no
 * interrupt body: the block has no vector.
 * A filter, a synchronizer, an edge detector, an event input or a
 * sequencer needs init(generator) with a generator that runs; a purely
 * combinational LUT needs none.
 * Four shapes of this block that are easy to guess wrong:
 *  - there is one generic clock for the whole block, not a per-LUT
 *  - a LUT has one event input line (LUTEI + INSEL = EVENT), and the
 *  - the input menu is per-LUT and not per-input: the whole LUT n sees
 *    SERCOM gives SERCOM[n]) with the one exception that TCC hands
 * The enable-protection story, which is why every configuring verb here
 * refuses while the block is enabled. Three documents give three
 *   37.6.2.1  SEQCTRLx.SEQSEL is protected by the even LUT's ENABLE,
 *   errata 1.7.3, every revision of E/G/J: "the SEQCTRLx and LUCTRLx
 * Errata, silicon rev F (E/G/J row - never the N row):
 *    revision B only. Not coded around, and the bench checks that the
 *  - 1.7.2 Sequential Logic, every revision: after an even LUT has been
 *    logic stays under reset until CTRL.ENABLE is written again. The
 *    workaround is code here: Lut<n>::enable(true) on an even LUT
 *  - 1.7.3 Enable Protected Registers, every revision: see above.
 *  - 1.7.4 PAC Protection Error, every revision: writing CTRL.SWRST is
 *    revision B only, and the bench confirms the documented mapping on
 * Not on this family (37.8.3): INSEL's ALT2TC (0xA) and ASYNCEVENT
    /// A setup verb; false only when the generic clock channel did not
    /// go quiet within `spins` (GclkChannel::connect()).
++ b/brio/samc21/clock.hpp
 * The rate the CPU and the peripherals run at on the SAM C21, and the
 * means to change it. A program names its main clock once and passes
 * the tag object to every driver's init(), which computes its dividers
 * from that rate:
 *     using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
 *     constexpr SysClock clock;
 *     SysClock::init();             // first in main(), before any driver
 *
 * Who calls what. Clock::init() is a setup call, first in main() and
 * before any driver's init(): the drivers trust `hz` and not the
 * hardware, and out of reset the CPU runs at 4 MHz. It waits for the
 * oscillator and each synchronization, every wait bounded (0xFFFF polls
 * by default), and answers false when one did not complete - the rate is
 * then not the one `hz` states. The resources' configuring verbs are
 * setup verbs as well, with no guard of their own (read-modify-write or
 * whole-register stores), and their waits are bounded the same way
 * (Xosc::init() polls up to 40 000 000 times for a crystal, Fdpll::init()
 * up to 4 000 000 for each synchronization and for CLKRDY); nothing here
 * masks interrupts. Oscctrl::isr() is an interrupt body for IRQ 0, which
 * five blocks share: the program's SYSTEM_Handler calls it beside the
 * other blocks' bodies. Stopping an oscillator has one ordering rule:
 * every generator is moved off it first (Xosc::stop(), Fdpll::stop()).
 *
 * The clock tree in the two strata docs/design/clock.md describes:
 *
 *  Resources - one monostate per block (a class whose state and
 *  functions are all static: one block, no instances), a thin typed
 *  view of its registers with the synchronization discipline built in:
 *    Oscctrl      the block (OSCCTRL, ch. 20): the STATUS register all
 *  Task - what an application names:
 *    Clock<source, hz>   the static main clock: one constexpr truth
 * Scope, honestly. All three OSCCTRL roots are now resources with their
 * full register description; what is still single-rooted is the task.
 * pointing CLK_MAIN at the crystal or at the DPLL is a main-clock design
 *    orders the wait states adapted before a frequency rise (and after a
 *  - erratum 1.2.2: writing OSC48MDIV while OSC48M runs unrequested
 *  - erratum 1.25.1, live on every revision: below 25 C the FDPLL
 *    reports spurious unlocks, and because the lock signal gates the
 *    output - `FdpllConfig::lock_bypass` defaults to true for exactly
 *  - erratum 1.3.3: an on-the-fly ratio change does not set
 * A note on names. Chapter 20's register summary calls the XOSC failure
    /// then knows the rate is not the one `hz` claims. Call first in
    /// main(), before any driver init: a driver set up before it computes
    /// its dividers for `hz` while the CPU still runs at the 4 MHz reset
    /// rate. Each wait is bounded (0xFFFF polls); nothing is masked.
++ b/brio/samc21/dac.hpp
 * A voltage made on the SAM C21: a level on the VOUT pad, a threshold
 * the comparator, the ADC or the SDADC reads internally, or a waveform
 * paced by events. The 10-bit digital-to-analog converter (DS60001479M
 * ch. 41) is a whole small chapter in one monostate resource (a class
 * whose state and functions are all static) - `Dac`, not `Dac<n>`:
 * this family has exactly one instance on every variant, so an index
 * stratum, against `Adc<n>`'s two real instances). The smallest use:
 * Who calls what. init() and release() are setup calls in main(), after
 * the clock task's init(); init() answers false for a configuration
 * dac_config_valid() refuses or a wait that never ended. Every wait is
 * bounded (0xFFFF polls by default): set() and set_mv() wait for DATA's
 * synchronization and answer false when the write was discarded (a
 * value standing in DATABUF, below); buffer() waits for nothing. The
 * configuring verbs (control_b, external_output, event_config,
 * start_on) refuse while the converter is enabled. isr() is the body
 * the program's DAC_Handler calls, and an EMPTY handler feeding the
 * converter writes with buffer(). Nothing here masks interrupts and
 * nothing keeps a caller's storage.
 *
 * What the silicon does. One channel, 10 bits, up to 350 ksps
 * (CTRLB.IOEN). There is no conversion-done flag - "as there is no
 * (41.6.2.4) - so STATUS.READY is about the startup time (3 us,
 * The four register disciplines, spelled per register the way
 * 1. Enable-protected - CTRLB, and 41.6.2.1 also names EVCTRL while
 *    41.8.3's own property line does not ("PAC Write-Protection" alone).
 *    The two statements disagree; `event_config()` refuses while the
 * 2. Write-synchronized - CTRLA.SWRST, CTRLA.ENABLE, DATA, DATABUF
 *    (41.6.7). Nothing needs synchronization when read. But DATA and
 *    say so: SYNCBUSY.DATABUF stands until a start event consumes the
 * 3. Neither - EVCTRL, INTENSET/CLR, INTFLAG, STATUS, and DBGCTRL,
 * 4. Write-only - DATA and DATABUF are `__O` in the device header and
 *    drawn with W access in 41.8.8/41.8.9. There is no code readback,
 *    so `code()` reports what this driver last wrote and says so.
 * Not PAC-protected (41.5.8): INTFLAG and DATABUF, the pair an EMPTY
 * The reference, and a name that lies in the device header.
 * internal reference, which this family does not have: here the
 * another peripheral's register. `DacRef` follows the datasheet for
 * `DacRef` is this converter's own vocabulary and deliberately not
 * Where the data sits in the register is decided by two bits together
 * DATA[13:4] + 1 so the average carries 14 bits, and it is not a mode a
 * Millivolts. `set_mv()` is a thin wrapper over util/analog.hpp's
 * The linear range is not the whole range. Table 45-30 gives it as
 * Errata, DS80000740S, read on the E/G/J row at revision F.
 *  - 1.8.9 DAC Output (device level, all revisions): selecting the DAC
 *    output as the ADC's positive multiplexer input makes both the DAC
 *    pad instead. On this silicon that wire has zero length: PA02 is
 *    selecting `AnalogIn<Pin<'A',2>>` on ADC0 is the workaround.
 *    `Dac::adc_input_pad_port` and `adc_input_pad_pin` name that pad
 *    and the bench measures
 *  - 1.9.2 Standby Sleep Mode (all revisions): with CTRLA.RUNSTDBY = 0
 *    and is answered from the other side: its workaround is
 *  - Not this silicon: 1.9.1 (dithering with right-adjusted data giving
 *    16 LSB of INL) is revision B only - the E/G/J row carries one X
 * Not built (docs/samc21/dac.md carries the list): VREFA, which needs a
 * never switches on; and the DAC as a wake source - EMPTY and UNDERRUN
++ b/brio/samc21/delay.hpp
 * `delay_us` / `delay_rate` / `DelayRate` (docs/cortexm/README.md). The
 * smallest use is `brio::delay_us(clock, us)` with the clock task's tag:
 * it spins the caller for at least `us` microseconds, timed on SysTick's
 * counter, and answers false, spending no time, for a wait of a kernel
 * tick or more or when SysTick is not running - a wait of a tick is a time
 * event's (docs/samc21/platform.md, "Types and verbs"). It holds the
 * caller, whatever its context, for the whole wait, and masks nothing.
++ b/brio/samc21/device_tables.hpp
 * The reserve: the table of what each SAM C21 variant has, read off the
 * device header. A driver asks it which EXTINT line a pad carries, which
 * waveform output a pad and a peripheral function give, which analog
 * inputs, pad pairs and LUT pads a package bonds, and the instance
 * parameters of each block; a program meets it only through those
 * drivers, as a compile error naming the absence when it names a pad or
 * an instance its variant does not have:
 *   static_assert(extint_exists<'B', 22>);           // what ExtInt<> checks
 *   constexpr int line = eic_extint_line('B', 22);   // 6 on the G and J,
 *                                                    // -1 on the E (no PB22)
 *
 * Everything here is constexpr, evaluated by the compiler: no verb runs
 * on the target, nothing waits, nothing is called from any context.
 *
 * Everything in this file is a probe of the device header's own per-pad
 * and per-instance symbols - `#ifdef` walls that turn vendor macros into
 * constexpr data - and nothing else is allowed to live here.
 *
 * The rule this file enforces (docs/design/overview.md, "Target
 * strata"): a preprocessor conditional whose product is a value - a
 * here, quarantined, one entry per header symbol; a driver may keep an
 * Why probes and not per-variant tables. The alternative - three
 * device macro - would be a copy of the DFP's bonding information,
 * able to drift from it silently. These probes are that information,
 *  - TCC waveform outputs: the same pad carries two of them, on two
 * samc21/adc.hpp, samc21/dac.hpp, samc21/sdadc.hpp, samc21/tsens.hpp,
 * samc21/ccl.hpp, samc21/pac.hpp, samc21/dsu.hpp, samc21/mtb.hpp. Each
 * declares the meaning of its numbers (what an EXTINT line is, what a WO
 * pad does); this file only says which numbers exist on this device.
++ b/brio/samc21/divas.hpp
 * neither. The smallest use (docs/samc21/divas.md, "How to use it"):
 *     Divas::configure(false);                       // once: unsigned
 *     const auto r = Divas::divide_unsigned(n, d);   // r.result, r.remainder
 *
 * Who calls what. Every verb is a few stores and loads with no guard and
 * nothing masked. The engine has one set of operand registers, so one
 * context divides at a time: a division started in an interrupt handler
 * between another's two operand writes corrupts both. On the default AHB
 * path the RESULT read waits for the engine in hardware; on the IOBUS
 * path the operation verbs poll STATUS.BUSY with no bound, and
 * wait_idle() is the bounded poll (0xFFFF reads). The bus clock is on out
 * of reset, so nothing needs opening first.
 *
 * Why it is interesting and why it is only a driver. Every `a / b` in
 * toolchain use it (gcc lets `__aeabi_uidiv` be overridden), and that
 * step is deliberately not taken here. It is a whole-image decision with
 * re-entrant against interrupts (the block has one set of operand
 * real. So this driver exposes and measures; docs/samc21/divas.md carries
 * What the silicon does.
 * The operation starts on the operand write. Writing DIVIDEND does
 * Two buses, and the difference matters. The block answers at
 * 0x48000000 on the high-speed bus and at 0x60000200 on the CPU's local
 * is busy inserts wait states and returns the right answer, so no
 * polling is needed; the IOBUS cannot wait, so a caller there must poll
 * The IOBUS address comes from the data sheet, not the header. The device
 * it takes the same rule: the header wins where both documents speak, and
 * where the header is silent the datasheet is spelled out and the bench
 * Writing an operand while busy is an error. 14.5.8 write-protects
 * is the only candidate. The verbs here do not poll before writing,
 * Divide by zero is not a fault. 14.6.2.5: the quotient comes back zero,
 * and stays set until written back. Nothing traps. `divide_unsigned()`
 * and `divide_signed()` therefore return a result like any other and
 * the caller reads `divide_by_zero()` - the same shape the silicon has,
 * rather than an optional that would hide the remainder the chapter
 * promises.
 * The signed overflow has no indication either. 14.6.2.4: the most
 * Leading-zero optimization is on by default and costs determinism.
 * No interrupt, no events, no DMA, and no sleep. 14.5.4/5/6 are all
 * No errata. DS80000740S has no DIVAS section.
 * Not built (docs/samc21/divas.md carries the list): adopting the block as
++ b/brio/samc21/dmac.hpp
 * for one use and refused for every other: a single channel draining a
 * Uart takes in its one optional slot. A program does not call the engine:
 * it names it in the Uart and binds the DMAC's vector to the Uart's
 * completion body, and the transmitter then moves the ring in blocks with
 * no interrupt per byte (docs/samc21/sercom.md, "How to use it"):
 *
 *     #include "samc21/dmac.hpp"
 *     using Serial = brio::Uart<5, console_pads, 64, 256, brio::DmaTxEngine>;
 *     extern "C" void DMAC_Handler() { Serial::dma_isr(); }
 *
 * Who calls what. The Uart's init() calls arm() (main context, a setup
 * verb), which claims the engine, resets and
 * configures the block and the channel with bounded waits, and opens the
 * DMAC's NVIC line; the Uart's release() calls release(). Blocks are
 * started from the Uart's producer verbs in main context and from its
 * completion in the DMAC handler: the claim between the two, reserve(),
 * is a test-and-set taken under the platform's critical section, and the
 * channel's programming, launch(), runs unmasked. take_interrupt(),
 * complete() and abandon() are the completion body's, reached through the
 * Uart's dma_isr() in the program's DMAC_Handler and nowhere else; a
 * DMAC_Handler left unbound is the crt's weak alias of its silent spin,
 * where the program stops at the first block's end. The run a block
 * carries is the ring's own storage, lent to the channel until its
 * completion consumes it.
 *               run, its completion the end of the run - claimed by one
 * Why one. Erratum 1.10.4 (DS80000740S, "Concurrent channels triggers",
 * descriptor is fetched from it (25.6.2.6), so a corrupted write-back is a
 * The one-user rule, and how it is enforced. Structurally first: there is
 * may still name the engine in two Uarts - two owners of one channel,
 * which would reprogram each other's live block - so the engine is claimed
 * arm() while the first holds the claim is refused with a panic, the
 * release(), so two transports that take the engine in turn (a suite
 * image is one translation unit, but "two different instantiations name
 * The level is the rule's for a transmit: the next of the arbiter's four
 * arms at level 2, the one below an overrunning receive's 3; with one
 * Two moments. arm() configures the binding once - the block reset and
 * CHID is written once. The channel registers sit behind a selector
 * nothing else writes it, so every channel access below is one access and
 * The end address. For an incrementing side SRCADDR holds the address one
 * beat past the last one - start plus the run's length in bytes
 * Disabling is not instantaneous, and the edges behind it are silent: a
 * enable-protected and CHCTRLA.SWRST is ignored - the bit never reads back
 * The standing request. A SERCOM's transmit request is a level - "set
 * is a second beat into a full DATA (measured, docs/samc21/sercom.md).
 * A release leaves the next owner nothing. The channel keeps no request
 * Zero when absent. The descriptor sections are static members, present
 * The claim and the programming are two steps (reserve(), launch()), so an
 * No write-back is ever read. The engine knows the run it programmed and
     * code (Sercom<n>::dma_tx_trigger()). A setup verb, main context: the
     * Uart's init() is its caller.
     * A claim held by another owner is a panic - the one-user rule
     * The claim on the channel: false when a block is already in flight,
     * otherwise the engine is the caller's until launch() or cancel(). An
     * owner that starts blocks from two contexts calls it under the
     * platform's critical section (it is a test-and-set of busy_ and
     * masks nothing itself).
     * Send one contiguous run on a reserved engine: three stores into the
    /// from one context. False when a block is in flight or the run is
    /// The channel is enabled with no trigger pending and no beat moving
    /// trigger that stands - the owner's proof - a channel that cannot
     * The caller decides the block is dead - only the peripheral's owner
     * True when a block was abandoned. Out of line and cold: the rare
++ b/brio/samc21/dsu.hpp
 * this?". The smallest use, a checksum of the image (docs/samc21/dsu.md,
 * "How to use it"):
 *     Dsu::init();                  // clears the reset-time PAC protection
 *     if (const auto crc = Dsu::crc32(0x0000, image_words)) { ... }
 *     Dsu::release();               // protected again, as reset left it
 *
 * Who calls what. Every verb is main context with no guard of its own:
 * ADDR, LENGTH and DATA are one interface for CRC32 and MBIST, and a
 * command issued while another runs is discarded, so one caller at a
 * time. crc32(), crc32_raw() and mbist() spin the caller until
 * STATUSA.DONE, bounded (20 000 000 polls by default); the optional comes
 * back empty, or the result says not done, when the bound ran out.
 * mbist() writes over the caller's range, which the caller owns and
 * gives up for good. Nothing here masks interrupts, and the block has
 * no interrupt and no event to bind.
 *
 * What it is for here. Three of its services are useful to code running
 * on the device rather than to a probe attached to it, and those are
 *  1. Device identification. DID decodes into processor, family,
 *  2. A hardware CRC32 over any memory the bus matrix reaches. The
 *     IEC 60730 Class B asks for one. It destroys what it tests - the
 * What is deliberately not here: chip erase. CTRL.CE erases the whole
 * The protection surprise. PAC.STATUSB's reset value is 0x00000002
 * comes out of reset already write-protected, alone among the
 * The internal and external address ranges. 13.9: the first 0x100 bytes
 * internal range (the device header's `DSU_REGS`), which 13.12.2.3 is
 * Two registers the chapter does not describe. The device header
 * No errata. DS80000740S has no DSU section at all. The one item that
 * and Debug Interface Disable lock of 13.10 is not available on E/G/J
 * Not built (docs/samc21/dsu.md carries the list): chip erase, the DAP
++ b/brio/samc21/eic.hpp
 * peripheral where this family keeps its pin interrupts. There are none
 * event outputs. What a program uses it for: an interrupt or an event
 * from a pad. The smallest use, a button on a line (docs/samc21/eic.md,
 * "How to use it"):
 *   using ButtonInt = brio::ExtInt<brio::Pin<'B', 22>>;   // EXTINT6
 *   Eic::init();                          // bus clock, reset, left disabled
 *   Eic::clock_select(EicClock::ulp32k);
 *   ButtonInt::claim(brio::PinPull::up);
 *   Eic::configure_line(ButtonInt::line, {.sense = EicSense::falling});
 *   Eic::enable(true);                    // false: the block is not running
 *   ButtonInt::arm(true);
 *   Nvic::enable(Eic::irq());
 *
 * and the program's EIC_Handler calling Eic::isr().
 *
 * Who calls what. init(), release(), clock_select(), configure_line(),
 * release_line() and nmi_configure() are setup verbs, with no guard of
 * their own (read-modify-write of the block's registers): main, or one
 * active object's handler (an active object is a brio component with its
 * own event queue, served one event at a time by the kernel). The
 * enable-protected ones answer false and write nothing while the block
 * is enabled. init(), reset() and enable() wait for the block's
 * synchronization, bounded (0xFFFF polls), and answer false when it did
 * not complete. isr() is the body the program's EIC_Handler calls for
 * all sixteen lines, take_nmi() the body of NonMaskableInt_Handler; a
 * vector left undefined is the crt's silent spin. The flag and arming
 * verbs are single loads and stores. Nothing here masks interrupts.
 *
 *  Eic              the block: APB clock, the clock choice (GCLK_EIC or
 *  ExtInt<Pin>      one line reached through the pad that carries it:
 * Three facts that shape the file.
 * 1. The pad-to-line map is not a formula. PA16 is line 0, PA24 is line
 * 2. One NVIC vector for all sixteen lines. 26.6.6 says "The EIC has
 * 3. The clock is optional and knowing when is the whole trick
 *    asynchronously and needs no clock at all; so does edge detection
 *    GCLK_EIC or CLK_ULP32K - and in those modes the pin is sampled, so
 *    And the consequence the chapter never draws, measured here
 *    and CTRLA.ENABLE is write-synchronized against that requested
 *    clock. So whether the block can be enabled at all depends on what
 *    its lines asked for. With GCLK_EIC disconnected, a block whose
 *    detects and disables perfectly; the moment one line asks to be
 *    nothing detected at all. The write is pending rather than lost:
 * Enable-protection is the structural rule here. CONFIGn, ASYNCH,
 * (26.6.2.1). Every verb that touches one of them therefore refuses
 * Errata, DS80000740S, read on the E/G/J row at revision F - and the
 * row matters more here than in any chapter so far, because five of the
 * six EIC items look alarming and only one of them is this silicon:
 *    chip, so `enable()` does not quietly clear INTFLAG behind the
 *    before edge detection actually works): the N family only. This is
 *  - 1.11.6 Edge Detection in Standby: the matrix marks every revision
 *    refuted it at rev F (measured): an
 *    asynchronous line detected 100 edges of 100 offered inside one
 * Not built (docs/samc21/eic.md carries the list): the debouncer
++ b/brio/samc21/evsys.hpp
 * CPU in the path - what a program uses to let one peripheral act on
 * another's signal (a pin edge starting a conversion, a compare moving a
 * pad). The smallest use routes one generator to one user:
 *     Evsys::bus_clock(true);
 *     GclkChannel::connect(Evsys::gclk_id(0), 5);   // clocked paths only
 *     Evsys::connect(user, 0, EventChannelConfig{
 *         .generator = code, .path = EventPath::synchronous,
 *         .edge = EventEdge::rising});
 *
 * the generator code and the user index being the ones the two
 * peripherals' drivers publish. The bus clock is off out of reset
 * (MCLK_APBCMASK), and bus_clock(true) is what opens it.
 *
 * Who calls what. Every verb here is one store or one load: configure(),
 * connect(), attach(), disconnect() and the release verbs are setup
 * verbs (main, or one active object's handler - an active object being
 * a brio component with its own event queue, served one event at a time
 * by the kernel); configure(), connect() and attach() answer false and
 * write nothing for a user, a channel or a configuration the rules below
 * refuse; trigger() is a single store into
 * a write-only register and may be called from any context; isr() is
 * the body the program's EVSYS_Handler calls. Nothing here waits or
 * masks interrupts - the two waits the errata ask for are the caller's,
 * because this header does not know the channel clock's rate.
 *
 * It is an allocator and not a typed table, and the difference is worth
 * So this header owns the fabric and not the vocabulary. It moves
 * The four rules of the chapter, each with code behind it.
 * 1. The user multiplexer is written before the channel. "The user
 * 2. USER.CHANNEL is the channel plus one. Zero means "no channel
 * 3. The path decides whether edge detection is legal, and both ways.
 * 4. The asynchronous path has no status at all. No interrupts, and
 * Errata: three of the four are live on this silicon, which is a lot
 *  - 1.12.1 (all revisions): in synchronous mode a channel whose
 *    generic clock is always on (ONDEMAND = 0) can raise spurious
 *    overrun interrupts. The workaround is ONDEMAND = 1, so that is the
 *  - 1.12.3 (all revisions): a software event on a resynchronized path
 *    that window is lost with no overrun flag - the worst kind of
 *    busy for one GCLK tick without CHBUSY showing it, so a trigger
 *  - 1.12.2 is not this silicon (E/G/J revisions B..E only).
 * Not built (docs/samc21/evsys.md carries the list): the generator and
 * user tables, deliberately - see above; and the channel interrupt as a
++ b/brio/samc21/freqm.hpp
 * generators. The smallest use measures the main clock against a
 * generator on OSCULP32K (docs/samc21/freqm.md, "How to use it"):
 *   const uint8_t refnum = Freqm::refnum_for(48'000'000 / 32768);
 *   Freqm::init(FreqmConfig{.measured_generator = 0,
 *                           .reference_generator = 5, .refnum = refnum});
 *   if (auto count = Freqm::measure()) {
 *       const uint32_t hz = Freqm::to_hz(*count, 32768, refnum);
 *   }
 *   Freqm::release();
 *
 * Who calls what. init() and release() are setup verbs; init() opens the
 * bus clock, connects both generic clock channels, resets and enables,
 * each synchronization bounded (0xFFFF polls), and answers false for a
 * configuration config_valid() refuses or a synchronization that never
 * completed. measure() holds its caller for the whole window - REFNUM
 * reference periods - bounded in polls (20 000 000 by default), and
 * answers nothing for a disabled block, an overflow or a wait that ran
 * out; it belongs to main or to one active object's handler (an active
 * object is a brio component with its own event queue, served one event
 * at a time by the kernel), which it holds for that long. start(),
 * value() and the flag verbs are single stores and loads; isr() is the
 * body the program's FREQM_Handler calls. Nothing here masks
 * interrupts.
 *
 * What the silicon does, in one formula and three constraints.
 * The block counts periods of the measured clock for REFNUM periods of
 * the reference clock and leaves the count in VALUE (44.6.1). Both
 * 1. The reference must be slower than the measured clock. The chapter
 *    refuses it, and nothing here either: `measure()` hands the zero
 *    back as a count, and a VALUE of zero and a genuinely stopped clock
 *    are the same reading, so the caller reads a zero as no answer.
 * 2. VALUE is 24 bits, and the overflow is the caller's to avoid.
 * 3. Erratum 1.24.1, live on every silicon revision including this one,
 *    with no workaround offered: reading CTRLB raises a PAC protection
 *    error. So CTRLB is never read here - not in a read-modify-write,
 *    consequence worth stating is that there is no way to ask the block
 * Precision, since a frequency meter that does not say how well it
 * part in 12 million and overflow long before. The reference's own
 * this block measures a ratio, and the absolute number it is turned into
 * Not built (docs/samc21/freqm.md carries the list): the GCLK_IO pins as
 * claim this header does not make. The DONE interrupt is built - flags,
++ b/brio/samc21/i2c.hpp
 * The SAM C21 SERCOM in I2C mode (DS60001479M ch. 33), for a program
 * that reads and writes I2C devices through the bus arbiter, or answers
 * another host as a client. The smallest use, a device client on the
 * arbitrated bus (docs/samc21/i2c.md, "How to use"):
 *     using I2cHw = brio::I2cHost<3, my_pads>;   // generator 0 on a clean wire
 *     using I2c = brio::I2cBus<I2cHw, P>;        // an active object
 *     extern "C" void SERCOM3_Handler() {
 *         if (I2cHw::isr()) { brio::post<I2c>(brio::TransferDone{I2cHw::status()}); }
 *     }
 *     I2cHw::init(clock, measured_rise_ns);      // in main(), after the clock task
 *
 * then an active object posts a Request ({addr, tx, tx_len, rx, rx_len,
 * reply, speed}, the buffers as lend<Lease::reply>(...)) to I2c and gets
 * an I2cDone back with i2c_ok or the wire's answer (i2c_rejected for a
 * speed the core cannot make, i2c_bus_error, a NACK code).
 *
 * The words. An active object (AO) is a brio component with its own
 * event queue, served one event at a time by the kernel; `P` is the
 * program's platform type (samc21/platform.hpp's SamPlatform). The
 * arbiter is I2cBus, the active object that owns the bus on behalf of
 * every other one and runs their requests one at a time; a timed
 * arbiter is one whose timeout_ticks is not zero (util/i2c_bus.hpp),
 * the one that calls recover() when a tenure outlives it. A tenure is
 * one ownership of the bus from START to STOP. Lending a buffer
 * (lend<Lease::reply>, kernel/borrowed.hpp) passes its address in the
 * event, not a copy: the lender, the posting active object, keeps it
 * untouched until the reply. "Generator 0 on a clean wire" is the
 * template's default generator, the CPU's own rate: init()'s comment
 * says when a wire run in a bundle with other signals, whose crosstalk
 * this unfiltered input reads as false Starts, wants a slower one.
 * measured_rise_ns is the bus's SCL rise time in nanoseconds as the
 * bench measures it, 300 when left out; it enters the baud arithmetic.
 * SYSOP is the host's SYNCBUSY bit that a store to CTRLB.CMD,
 * STATUS.BUSSTATE, ADDR or DATA raises while enabled (the facts below). The task is the I2cHost engine, the driver layer the
 * arbiter drives.
 *
 * Who calls what, and from where. init() is a setup call in main(),
 * after the clock task's init() and before interrupts are enabled
 * globally; it waits, bounded, for the peripheral's synchronizations and
 * the forcing of the bus state to IDLE, and answers false when one did
 * not complete or when not even standard_100k can be produced at the
 * core rate; it hands the pads to the SERCOM with no internal pull (the
 * bus's pull-ups are the board's) and enables the instance's NVIC line
 * itself. start() is the arbiter's, from its dispatch in main
 * context, and waits, bounded, only for synchronizations (the
 * disable/enable pair and the forcing to IDLE a change of speed costs,
 * the SYSOP wait before its ADDR store): every tenure that moves the
 * wire runs on the byte pump (the engine's interrupt-driven byte
 * loop), and start() returns false, meaning "still running, the
 * completion comes on the interrupt"; it returns true - "already over" -
 * only for a speed the core cannot produce (i2c_rejected) or a SYSOP
 * that never settled (i2c_bus_error), with nothing moved, and the
 * arbiter then replies with that status. isr() is the interrupt body: the
 * program's handler for the instance's vector calls it and nothing else
 * does, and it answers true on the completion edge, when the handler
 * posts TransferDone{status()}; a vector left unbound is the crt's weak
 * spin (the start-up code's default handler, a silent loop the program's
 * own definition replaces), where the program stops at the first byte.
 * The Request's two buffers are lent from the moment the Request is
 * posted until the I2cDone the arbiter replies with: untouched and in scope until then, which nothing checks.
 * recover() is the timed arbiter's, from its dispatch; unstick() is the
 * application's, main context, and spins for its nine pulses and a Stop
 * with the SERCOM's interrupt line disabled. I2cClient's polled verbs
 * belong to the context that calls them, its isr() to the program's
 * handler. The resources' configuring verbs are setup calls with no
 * guard of their own. The parts:
 *
 *  I2cm<n>     the host resource - a typed view of one instance's I2C
 *  I2cs<n>     the client resource - the other register set (unlike the
 *  I2cHost<n, pads, generator>
 *              the task util/i2c_bus.hpp (= util/bus_master.hpp) drives:
 *              the transfer engine. A Request is one bus tenure - write,
 *  I2cClient<n, pads, generator>
 * Pads are fixed by the chapter: SDA is PAD[0] and SCL is PAD[1] (33.4),
 * so I2cPads carries only the two pins - there is no DOPO here. Which
 * pins may carry I2C at all is table 6-7's per-package list (on the
 * PB30/31) and no device-header symbol encodes it, so it stays a stated
 *    client's ADDR are enable-protected (33.6.2.1): a write while
 *    enabled is discarded. Everything here configures disabled;
 *  - the host's SYNCBUSY has a third bit beside SWRST/ENABLE: SYSOP,
 *    enabled (33.6.6). Every such store here waits first (the sdadc
 *  - MB and SB hold SCL low (STATUS.CLKHOLD) until software answers
 *    flags automatically (33.10.7), which is why the engine starts a
 *  - erratum 1.17.8 (live): STATUS.CLKHOLD is documented read-only and
 *    is writable, and writing it corrupts the clock-hold state. Every
 *    STATUS store in this file is a mask that excludes bit 7 by
 *  - erratum 1.17.10 (live): 10-bit addressing in CLIENT mode is not
 *  - erratum 1.17.11 (live): the client's error status bits are not
 *  - erratum 1.17.13 (live): Quick Command with SCLSM = 1 raises a bus
 *  - erratum 1.17.16: SWRST claimed non-functional while ENABLE = 0 on
 *    every revision. Not reproduced in SPI mode on this silicon
 *    (samc21/spi.hpp) nor in I2C mode (measured: SWRST from the
 *    disabled state reset the block with its synchronization
 *    completing, docs/samc21/i2c.md); the enable-first discipline is
 *    kept here too, the sheet marking every revision;
 *  - erratum 1.17.21 (live): automatic address acknowledge (AACKEN)
 *    feature". This driver has no AACKEN knob at all - an AMATCH
 *  - erratum 1.17.22 (live): the client's STATUS.RXNACK is invalid at
 *    at 32.768 kHz (33.6.3.1) and is shared by SERCOM0..4 (one
 * The transfer engine, driven by util/bus_master.hpp: a Request is one
 * bus tenure - write, read, or write-then-read joined by a repeated START
 * probe. Asynchronous whenever the wire moves: start() returns false and
 * a TransferDone{status()} follows from the ISR, which is the engine
 * contract every target's I2cHost keeps; the two failures that move
 * nothing (a speed the core cannot produce, a SYSOP that never settled)
 * complete inside start() (start()'s comment).
    /// Bring the instance up as a client: clocks, reset, configuration,
    /// pads, the NVIC line. Setup's call, main context, waits bounded;
    /// false when the resource refuses the configuration or a
    /// synchronization did not complete.
++ b/brio/samc21/mtb.hpp
 * and with write priority over the processor. What a program uses it
 * for: a trace window over a suspect piece of code, or (through
 * samc21/postmortem.hpp) the last branches before a fault, read at the
 * next boot. The smallest use (docs/samc21/mtb.md, "How to use it"):
 *     alignas(1024) uint32_t trace[256];          // 1024 bytes, 128 packets
 *     Mtb::configure(trace, sizeof(trace));       // false: not a trace buffer
 *     Mtb::enable(true);
 *     suspect();
 *     Mtb::enable(false);
 *     const uint32_t n = Mtb::packets_written(trace, sizeof(trace));
 *     // Mtb::packet(trace, i) for i < n
 *
 * Who calls what. Every verb here is a few loads and stores of the
 * block's registers or of the caller's buffer: nothing waits, nothing
 * masks, and there is no clock to enable. configure(), enable() and
 * release() write MASTER and FLOW with no guard of their own, so one
 * context owns the block at a time; freeze() and snapshot() are legal
 * with interrupts dead and from a fault handler. The buffer is the
 * caller's storage: the block writes into it from enable(true) until
 * the trace stops, and a reader stops it first (freeze() or
 * enable(false)), or reads its own branches.
 *
 * The interesting case is the one with no debugger. Every vendor
 * through the Debug Access Port. But the buffer is ordinary SRAM chosen
 * run, stop it, and read its own trace - a hardware backtrace of the
 * What the silicon does.
 * Four registers, and the data sheet describes none of them. 10.3.2
 * documents of record. The device header is therefore the only local
 * The buffer is a power of two, aligned to itself. MASTER.MASK holds
 * buffer that is not aligned to its own size does not fail: it traces
 * somewhere else, which is why `configure()` refuses one.
 * POSITION and FLOW hold offsets from BASE, not addresses. BASE is a
 * A packet is two words: a source address and a destination address,
 * both with bit 0 carrying a flag whose names belong to the TRM. This
 * flags do on this silicon - see docs/samc21/mtb.md.
 * What needs a debugger and what does not. MASTER.EN, MASTER.MASK,
 * halt, which on ARMv6-M requires DHCSR.C_DEBUGEN - a bit
 * `bin/brio` deliberately clears after every SAM flash, precisely
 * Two event users, and the two documents disagree about their numbers.
 * the header's, per the house rule, and the bench settles which is
 * No clock of its own. The MTB has no bit in any MCLK mask and no row
 * its identifier is one the data sheet never prints: table 12-3's PAC
 * No errata. DS80000740S has no MTB section.
 * Reading the tail back is a race against the reader itself. The MTB
 * `freeze()` is therefore the first verb of any post-mortem path - not
 * Not built (docs/samc21/mtb.md carries the list): any decoder above the
     * Point the block at a buffer and write MASTER and FLOW - without
     * enabling it. False, and nothing written, for a buffer that is not
     * a trace buffer (buffer_valid()) or a watermark past its end.
     * `enable(true)` is a separate verb so that the moment tracing
     * starts is a line of the caller's own, which is what makes a trace
     * window mean anything.
++ b/brio/samc21/nvic.hpp
 * Interrupt control on the SAM C21: what a program uses to turn
 * interrupts on (`brio::enable_interrupts()`, once in main() after the
 * drivers' init() calls and before the kernel's run()), to mask them
 * for a scope (`InterruptGuard`, the platform's CriticalSection, legal
 * from any context and nesting), and to enable, pend or set the
 * priority of one NVIC line (`Nvic`, which refuses a level this core
 * does not have). The header is the device header, then the core
 * It still sits at the bottom of the samc21/ stratum: ticker.hpp and
 * The SAM's own facts about interrupts: line 0 is shared by MCLK,
++ b/brio/samc21/nvm.hpp
 * described. What a program uses it for: a record kept across a power
 * cycle (usually through samc21/nvm_flash.hpp's heap and journal
 * backends rather than raw pages), the factory calibration and serial
 * number, the fuse row read back. The smallest raw use
 * (docs/samc21/nvm.md, "How to use it"):
 *     if (Nvm::erase_row(NvmArray::rwwee, Nvm::rwwee_base) == NvmError::none) {
 *         (void)Nvm::program_page(NvmArray::rwwee, Nvm::rwwee_base, page);
 *     }
 *
 * Who calls what. The command verbs (erase_row, program_page, the lock
 * and cache commands) wait for the command, bounded
 * (command_spin_limit), and answer an NvmError - `none` the only
 * success, `timed_out` a command that never finished. They are main
 * context verbs with no guard: the page buffer, ADDR and CTRLA are one
 * interface for both arrays, so one caller at a time, and nothing here
 * masks interrupts. A command on the main array stalls every read of
 * the main array, instruction fetches included, until it ends; on the
 * RWWEE array the CPU runs on. program_page() reads the caller's span
 * during the call and keeps nothing. isr() is the body the program's
 * NVMCTRL_Handler calls. The readers (read, read_word, the factory
 * views) are loads, legal from any context. No clock needs opening
 * first: NVMCTRL's AHB and APB clocks are on out of reset.
 *
 *  Nvm            the block - geometry (from the device header, checked
 *                 reporting, erase/program on either array, the region
 * What the silicon does, in the four facts that shape every verb below.
 * 1. Two programmable arrays, one page buffer. The main array (256 KB at
 *    register - what differs is the command code and, decisively, who
 *    stalls: reading the main array while the main array is programmed or
 *    CPU completes about 3950 polling turns on RWWEE and one on the main
 *    array. The RWWEE array is in exchange not cached, so reads from it
 *    is the more durable of the two, 100k cycles against the main array's
 * 2. Erase by row, program by page. A page is 64 bytes, a row is four
 * 3. The page buffer is loaded by writing to the memory-mapped array, and
 *    it has two traps. Writes must be 16 or 32 bits wide - an 8-bit write
 *    raises a system exception (27.6.4.3), which on this core is a
 *    crosses a 64-bit boundary, which means a random-access write pattern
 *    The real rule is narrower than "write ascending", and it was
 *    section needs is that its two words be written back to back.
 *    Ascending satisfies that; so does descending, which crosses a
 *    A separate limit, and it is not in chapter 27 at all but in table
 *    45-42's footnote: at most 8 consecutive writes are allowed per row
 * 4. ADDR is a half-word offset from the section base, not a byte address
 *    Start address of the section + 2*ADDR" (27.8.8). Measured on the
 * Errata: NVMCTRL has none. Item 1.14.1 of DS80000740S reads "Reserved"
 * Register access protection: CTRLA, CTRLB and ADDR are PAC
 * Not built (docs/samc21/nvm.md carries the list and the reasons):
 *  - The SSB command (Set Security Bit). It is one-way - only a debugger
 *    the user row is the fuses - BOOTPROT, EEPROM size, BODVDD level and
++ b/brio/samc21/nvm_flash.hpp
 * value journal run on this silicon. A program names one as the media of
 * its heap or journal (docs/samc21/nvm.md, "How to use it"):
 *     NvHeap<RwweeFlash, 8, 2> heap;                  // blocks
 *     NvJournal<RwweeJournalZone, 6, 32> journal;     // small values
 *
 * and calls their mount() first; it never calls these types itself.
 *
 * Who calls what. read() is a copy out of memory-mapped flash; program()
 * and erase() go through samc21/nvm.hpp's command verbs, each command
 * waited for with that header's bound, and answer false for a misaligned
 * address or length or a command that did not end clean. They inherit
 * that header's rule: one caller of the controller at a time, nothing
 * masked, and on the RWWEE array the CPU keeps running through every
 * write. The caller's span is read during the call and not kept.
 *
 * It lives in the RWWEE array, and that choice is the whole design.
 * 0x00400000. Writing or erasing the main array stalls the AHB - every
 * array stalls nothing: the main array can be read throughout (27.6.4.1),
 * linked image has: its free flash is bands whose inner edges are
 * zone is a constant - the whole array - and no linker symbol is read
 * The array is partitioned, because this family has two storage classes
 * Both bounds stay constants - that is the property the RWWEE choice
 * a build-dependent edge. It is anchored at the top of the array so that
 * The granularities are not the same number, which is what the concept's
 * erase_size / write_cell split is for: here an erase takes down a row of
 * 256 bytes and a program writes a page of 64, so on this silicon "page"
 * is the small unit. Other targets spell the two words the other way
 * Addresses are absolute. NvHeap works in flat byte addresses and derives
 * The build id is a link-time constant: samc21/CMakeLists.txt passes
 * symbol's address is the value. It is recorded in every map version and
 * nothing decides on it - a block's validity is its checksum's business.
++ b/brio/samc21/osc32kctrl.hpp
 * clock, and the crystal's failure detector - what a program uses for a
 * slow clock that a generator, the RTC or a standby can run on. The
 * smallest use starts OSC32K with its production trim and gives it to a
 * generator (docs/samc21/osc32kctrl.md, "How to use it"):
 *   Osc32k::init(Osc32kConfig{.calib = Osc32k::factory_calib(),
 *                             .enable_32k = true});
 *   Gclk<5>::configure(GclkConfig{.source = GclkSource::osc32k});
 *
 * Who calls what. Every verb is a setup verb with no guard of its own,
 * for main or one active object's handler (an active object is a brio
 * component with its own event queue, served one event at a time by
 * the kernel); nothing here masks interrupts. The two init()s and
 * retrim() wait for the oscillator's ready flag, bounded (2 000 000
 * polls for OSC32K, 40 000 000 for a crystal), and answer false when it
 * never rose; they and Osculp32k::calib(value) answer false and write
 * nothing for an impossible configuration or a locked oscillator. isr() is the
 * body the program's SYSTEM_Handler calls beside the other blocks on
 * IRQ 0. The block's bus clock is on out of reset and nothing here
 * touches it.
 *
 *   Osc32kctrl   the block - status, the three interrupt sources, and
 *   Osc32k       the accurate internal RC: off at reset, and it must be
 * The fact that most needs saying: OSC32K is not calibrated until
 * software calibrates it. 21.5.9 is blunt - "the OSC32K calibration
 * Four more rules of this chapter, each with code behind it.
 * 1. An oscillator's output is separate from the oscillator. EN32K and
 *    wants to be enabled before the GCLK or the RTC is pointed at it.
 * 2. CALIB is committed on ready. "When writing the calibration bits,
 *    the trim and immediately measuring reads the old frequency.
 * 3. WRTLOCK is one-way until a power-on reset, on both internal
 * 4. The RTC's clock is chosen here, not in the RTC. RTCCTRL.RTCSEL
 *    register; samc21/rtc.hpp states the ordering as its caller's
 *    obligation.
 * Errata: neither item touching this chapter applies to this silicon,
 * is requested by the GCLK) is marked for E/G/J revision B only, and
 * Not built (docs/samc21/osc32kctrl.md carries the list): the CFD's event
 * output (EVCTRL.CFDEO). XOSC32K's start and its frequency have run on a
 * board that carries the 32 kHz crystal, and its absence is a false
 * return on one that does not (docs/samc21/osc32kctrl.md, "Bench
 * findings"); the rest of it - the startup codes other than 0, the
 * external-clock mode, the failure detector - is written and
 * family-compiled and has not run on silicon.
     * False - and nothing written - for an impossible configuration or a
     * locked oscillator; false after the write for a ready flag that
     * never came.
++ b/brio/samc21/pac.hpp
 * block that can write-protect any peripheral on any of the APB bridges,
 * running, an access to an address no client answers. The smallest use
 * protects a peripheral by the identifier it publishes and puts it back
 * (docs/samc21/pac.md, "How to use it"):
 *     Pac::protect(Tsens::pac_id);
 *     ...
 *     Pac::unprotect(Tsens::pac_id);   // once: a double clear is an error
 *
 * Who calls what. Every verb is one store or one load, with no guard and
 * nothing waited for or masked; write_control() and the three verbs over
 * it answer false only for an identifier this device has no bridge for,
 * and what the silicon made of a request is read back with is_protected()
 * and flagged(). Protection, unlike the flags, is a program-wide state:
 * two contexts protecting and clearing the same peripheral break the
 * balance rule below. isr() is a body for IRQ 0, the system blocks'
 * shared line: the program's SYSTEM_Handler calls it beside theirs. The APB
 * clock is on out of reset, so nothing needs opening first.
 *
 * The position this header takes: mechanism only, no concept.
 *  1. The framework has no user. Nothing in brio turns protection
 *  2. The double-write rule makes nesting a design decision, not a
 *     detail. 11.5.2.6: setting protection on an already-protected
 *     peripheral is an error, and so is clearing it twice. A naive
 *  3. Protection is not protection from everything. Erratum 1.13.3
 * What the silicon does.
 * WRCTRL is a keyed, word-wise store. One 32-bit write carries the
 * only a word-wise write takes effect and that any other access is
 * The lock is one-way until a reset. Key 3 sets protection and locks
 * The flags are four banks. INTFLAGAHB reports client-bus errors (an
 * all are outside PAC write protection themselves (11.4.8), together
 * One interrupt, and it is the shared line. PAC_IRQn is line 0, shared
 * The errata. There is no errata section for the PAC itself in
 * exists instead is a set of items in other chapters, all live at
 *    not raise the protection error 11.5.2.4 promises for an illegal
 *    the bench found it dropped in silence, with no flag raised.
 *  - 1.24.1 (FREQM): reading FREQM.CTRLB raises a protection error, with
 * flag is good evidence, an absent flag is not. docs/samc21/pac.md carries
 * Not built (docs/samc21/pac.md carries the list): no guard type and no
++ b/brio/samc21/pin.hpp
 * The SAM C21 I/O pins (PORT, DS60001479M ch. 28): what a program uses
 * to drive an LED, read a button, hand a pad to a peripheral, or let an
 * event move a pad. A pin is a type - the group letter and the number
 * are template arguments - and its functions are static:
 * (Rx being another Pin alias, PB31 for the console's SERCOM5 - the
 * function letter is the package's I/O multiplexing table's.)
 *
 * Two faces, register-level:
 *
 *  Port<'B'>       the port resource - one group of the PORT peripheral:
 *                  32-bit mask operations on DIR/OUT/IN and the
 *                  multi-pin configuration engine (WRCONFIG);
 *  Pin<'B', 23>    the per-pin face: direction and value, configure()
 *                  as one PINCFG store, the peripheral-function handoff
 *                  (PMUX), a PwmChannel (max 1: a pin a generic actuator
 *                  can drive like a timer channel) and a PinRef factory
 *                  (ref(): a run-time handle for a request event).
 *
 * Which context may call what. The value verbs (set, clear, toggle,
 * read, Port's out_set/out_clear/out_toggle and dir_* masks, PinRef's
 * set and clear) are single stores into the write-one registers or a
 * single load, legal from any context and needing no guard. The
 * configuring verbs (input_enable, pull, strong_drive, function,
 * release, and output()/input(), which call two of them) read, modify
 * and write the pin's PINCFG byte, function() also the PMUX byte the
 * pin shares with its neighbour, and Port::configure_event the group's
 * EVCTRL: they belong to setup or to one active object's handler (an
 * active object is a brio component with its own event queue, served one
 * event at a time by the kernel), never to two contexts at once on the
 * same byte - a context being main, an active object's dispatch (all of
 * them one context, since the kernel runs one dispatch at a time) or an
 * interrupt handler. Nothing here waits or masks
 * interrupts, and nothing needs a clock first: the PORT's bus clock is
 * on out of reset and no verb here touches it. output() leaves OUT as
 * it is, so it drives the level OUT holds - 0 out of reset, 1 after an
 * input(PinPull::up), whose pull direction is that same bit.
 *
 * Two facts that shape everything below.
 * 1. The input buffer is off by default. PINCFG.INEN gates it, and IN
 * 2. There is no pin interrupt in PORT. Edge and level sensing is a
 *    it. What PORT does have is the opposite direction: it is an event
 *    user (EVCTRL, 28.6.4), four inputs per group that set, clear,
 *    toggle or follow one pin each - and the follow ("out") action is
 * One package fact, exhaustively. This family has exactly two port
 * it stays true if a relative ever grows a group C. Which pins of an
 * existing group are bonded on a given package is a finer question - a
/// pin missing from an existing group is the per-package device tables'
/// job, and that table is not built - such a pin compiles, and nothing
/// says the package does not bond it.
++ b/brio/samc21/platform.hpp
 * What the brio kernel stands on when it runs on a SAM C21: the SAM C21
 * (Cortex-M0+) realization of the kernel's Platform concept, the
 * compiler-checked set of requirements kernel/platform.hpp states, and
 * the one header of this stratum the kernel templates are instantiated
 * with. A program includes it and names it once, as the first argument
 * of its Tenuto:
 *
 *     using P = brio::SamPlatform;
 *     brio::Tenuto<P, Blinker, Supervisor>::run();
 *
 * It gives the kernel a critical section (interrupts masked for a
 * scope), an idle that sleeps until the next interrupt without losing
 * one that arrives at the wrong moment, the program's source of time
 * (samc21/ticker.hpp's 1000 Hz SysTick `Ticker`), a breakpoint for a
 * panic and the storage a panic record survives a reset in. Who calls
 * what: the kernel loop calls idle() in main-loop context, with
 * interrupts already masked, and nothing else should; now() and
 * ticks_per_second may be read from any context; CriticalSection may be
 * opened from any context and nests; break_here() is the panic path's
 * and panic_record() the panic path's and the next boot's. A program
 * binds the timebase's vector itself (SysTick_Handler calling
 * Ticker::tick(), after SysClock::init() and Ticker::init(clock) in
 * main()); the kernel binds no vector, and a SysTick_Handler left
 * undefined is the crt's weak spin, where the program stops at its
 * first tick.
 * The stopping half is samc21/sleep.hpp (PM) plus util/power.hpp's
    /// The kernel's idle: entered by the loop, in main-loop context, with
    /// interrupts masked and nothing to do; sleeps until the next
    /// interrupt and returns with interrupts enabled. Not for a program
    /// to call.
    /// WFI first, unmask after. On Cortex-M a pending interrupt wakes WFI
    /// The armed mode stands. This family selects the sleep depth in
    /// use it, and out of reset it is 0. With SLEEPCFG at its reset value
    /// The one test a program with no power manager pays for is the
    /// SLEEPCFG read below, and it buys erratum 1.8.13's workaround: with the standby
    /// rather than a flag kept, because SLEEPCFG is the state: it is what
    /// the guard, whatever SysTick's counter does in there - and on the
    /// bench it runs (docs/samc21/platform.md), so the guard is also what
    /// Halt in the debugger: the panic path's breakpoint
    /// (kernel/panic.hpp), callable from any context.
    /// Caveat, ARMv6-M. Unlike v7-M, this core cannot ask whether a
    /// The kernel's clock: the ticker's tick count, readable from any
    /// context (what kernel/time_event.hpp arms deadlines against).
    /// survival is promised nowhere: table 18-1 lists what each reset
    // gcc 16 emits the COMDAT section for an inline variable with a
    // custom section attribute as `"awG"` without the group name, and gas
++ b/brio/samc21/postmortem.hpp
 * Where the program died, not just what killed it: the last branches
 * before a fault or a panic, kept across the reset and read at the next
 * boot, with no debugger attached. The smallest use is the two entry
 * paths below plus the boot side (docs/samc21/mtb.md, "How to use it"):
 * `Trace::arm()` once in main(), the HardFault body bound to
 * hard_fault_trace_reset<P, Trace>(), and at boot take_panic_record<P>()
 * and Trace::take() read side by side.
 * Who calls what. arm() and disarm() are setup verbs in main(), with no
 * guard of their own. capture() is the fault path's and the panic
 * reporter's: bounded by `keep_packets`, no allocation, nothing touched
 * but the MTB's registers and the store's own memory, legal with
 * interrupts dead. pending(), packets(), take(), clear() and raw() are
 * the boot side's, plain reads and stores in main context. Nothing here
 * waits or masks. The span take() hands back points into the .noinit
 * record and stays readable until the next capture().
 *
 * The breadcrumb this stratum already has answers what: kernel/panic.hpp
 * boot reads the record back. What it cannot say is where from: which
 * of the disaster the last N branches are in memory - they simply do not
 *   1. Freeze first. `capture()` stops the trace before it does anything
 *   2. Copy, checksum, and keep. The last `keep_packets` packets are
 *      copied out of the rolling buffer into a separate .noinit record
 *      the reset is a bounded, validated object, not a buffer somebody
 *   3. Read and invalidate. `take()` hands the packets over once and
 * It is a sibling of the panic record, not an extension of it. The
 * The same rule as the fault handler's: an existing record is not
 * overwritten. samc21/reset.hpp's hard_fault_reset() refuses to clobber a
 * PanicRecord because a fault after a panic is a consequence of
 * The two entry paths, and which one really runs.
 * On a board with DHCSR.C_DEBUGEN cleared - which is what
 * `bin/brio` leaves after every SAM flash - the second path is the
 * one that runs even for an orderly panic: panic() calls
 * P::break_here() before any reporter, break_here() is a BKPT, and a
 * Costs nothing it is not asked for. The trace buffer is ordinary .bss
++ b/brio/samc21/reset.hpp
 * The smallest use asks, once at boot (docs/samc21/reset.md):
 *
 *     switch (Reset::cause()) { case ResetCause::watchdog: ...; }
 *     if (auto record = take_panic_record<SamPlatform>()) { ... }
 *
 * and a program that wants a fault to leave that record binds
 *
 *     extern "C" void HardFault_Handler() {
 *         brio::hard_fault_reset<brio::SamPlatform>();
 *     }
 *
 * Who calls what. Reset's readers are single loads of a read-only
 * register, legal from any context and at any time. The Watchdog's
 * arm(), disable() and write_config() are setup verbs (main, or one
 * active object's handler - an active object being a brio component with
 * its own event queue, served one event at a time by the kernel), with
 * no guard of their own; each waits for the 1.024 kHz domain's
 * synchronization, bounded (sync_spin_limit, a million polls), and
 * answers false when it did not complete. clear() is one posted store
 * and may be called from any context; force_reset() is one store too,
 * and returns before the reset lands. isr() is the body the program's
 * WDT_Handler calls; hard_fault_reset() is the body of
 * HardFault_Handler, run in the fault, and Reset::software() never
 * returns. Nothing here masks interrupts. Left unbound, HardFault_Handler
 * is the crt's weak spin of its own and WDT_Handler the default spin.
 * the full story. `Reset::cause()` is that cross-check,
 * Four facts of this silicon shape everything below.
 * 1. RCAUSE is exclusive, not cumulative. "When a Reset occurs, the bit
 * 2. Not every reset resets everything (table 18-1). Only a power-supply
 *    a WDT reset or a system reset request spares those and the debug
 *    SRAM appears in no row of that table, for any source including
 * 3. The WDT runs on its own oscillator and its reset values are fuses.
 * 4. The CLEAR register is a loaded gun. Writing 0xA5 restarts the
 * Errata: neither RSTC nor the WDT has an item in DS80000740S - there is
 * Register access protection: RCAUSE and every writable WDT register but
 * Not built (docs/samc21/reset.md carries the list): the SUPC side of
 * the story - BODVDD and BODCORE are reset sources named here and
++ b/brio/samc21/rtc.hpp
 * behind a 10-bit prescaler, wearing three different faces - what a
 * program uses for time that keeps counting through every sleep, a
 * calendar with an alarm that wakes the device, or a periodic event with
 * no CPU in its path. The smallest use, a 32-bit tick counter
 * (docs/samc21/rtc.md, "How to use it"):
 *   brio::Rtc::init();                                     // disabled first
 *   brio::Osc32kctrl::rtc_clock(brio::RtcClock::ulp_32k);  // the source
 *   brio::Rtc::init();                                     // reset on it
 *   brio::Rtc::configure({.mode = brio::RtcMode::count32,
 *                         .prescaler = brio::RtcPrescaler::div1});
 *   brio::Rtc::enable(true);
 *   const uint32_t ticks = brio::Rtc::count32();
 *
 * Which context calls what. init(), release(), configure() and
 * event_config() are setup verbs, from main() or an active object's
 * handler (an active object is a brio component with its own event
 * queue, served one event at a time by the kernel); the counter, compare
 * and calendar verbs belong to one such context at a time. Each write is
 * a store followed by a bounded wait on SYNCBUSY (0xFFFF polls by
 * default) into the RTC's own clock domain, answering false when it runs
 * out; the synchronized reads wait the same way and load the register
 * whether or not the wait completed. Nothing here has a guard of its own
 * or masks interrupts. init() leaves the RTC disabled and its NVIC line
 * off; isr() is the interrupt body the program's RTC_Handler calls, once
 * the program has armed the flags it wants and enabled the line.
 * samc21/sleep.hpp's SamTimedSleepSite drives this same counter as its
 * alarm and witness.
 *
 *   Rtc              the resource and the whole chapter: the three modes
 * There are no tasks here on purpose. An alarm clock, a slow periodic
 * Seven facts that shape the file.
 * 1. The clock is not this driver's to choose, and that is not an
 *    selection is changed" - and take the chosen rate as an argument
 *    synchronized into that clock's domain, so resetting the RTC with
 * 2. The register map is three overlaid views, exactly as the TC's is,
 * 3. Erratum 1.16.3 is live on every revision of this family, this one
 *    included: "an 8-bit or 16-bit write access for a 32-bit register,
 *    structural: there is no verb that writes a byte or a half of any
 * 4. Reading the counter is not free, and the bit that makes it
 *    possible is not on at reset. COUNT (and CLOCK) are
 *    read-synchronized, gated by CTRLA.COUNTSYNC / CTRLA.CLOCKSYNC:
 *    `RtcConfig::read_sync` defaults true, and `count32()` and friends
 *    costs, and what a read without it actually shows, is measured -
 *    Two errata sit on this bit and neither applies to this silicon,
 *    B only, and 1.16.2 (the first COUNT read after enabling COUNTSYNC
 *    that item belong to the N-family row. The bench suite looks for
 * 5. Enable-protection and write-synchronization are different things,
 *    enable-protected and not synchronized; ENABLE and SWRST are
 *    synchronized and explicitly not enable-protected (24.8.1), which
 * 6. The prescaler is not only a divider - it is the periodic event
 *    source, and turning it off turns those off too. PEREO[7:0] tap
 *    combination; the same is true of the periodic interrupts and the
 *    The arithmetic consequence worth knowing before any calendar
 *    code: mode 2 needs a 1 Hz counter clock, and the prescaler stops
 *    at /1024, so a 32.768 kHz source cannot reach 1 Hz. The
 * 7. FREQCORR needs a prescaler. "Frequency correction requires that
 *    field value 1, i.e. DIV2 or slower, since both OFF and DIV1 pass
 * Not built (docs/samc21/rtc.md carries the list): tasks of any kind (see
 * above); erratum 1.8.7's caveat that a DMA write to RTC.COUNT during
 * samc21/sleep.hpp's SamTimedSleepSite uses this counter as an alarm and
 * a witness beside it instead).
     * disabled and unconfigured, because CTRLA's fields and EVCTRL are
     * enable-protected and the caller has them to write. A setup verb;
     * false when the reset's bounded waits did not complete.
     * The clock must already be chosen and running: this driver never
++ b/brio/samc21/sdadc.hpp
 * A slow, fine voltage reading on the SAM C21: a differential pair of
 * pads read to sixteen specified bits and more, once or free-running,
 * with hardware offset, gain and window post-processing. The 16-bit
 * sigma-delta converter (DS60001479M ch. 39) is this family's third and
 * strangest converter. A monostate `Sdadc` (a class whose state and
 * functions are all static), not `Sdadc<n>`: there is exactly one
 * instance on every C21 variant, so an index would be a parameter with
 * a single legal value (the `Rtc` and `Dac` shape in this stratum,
 * against `Adc<n>`'s two real instances). The smallest use:
 *   if (brio::Sdadc::read(v)) { ... }         // signed, -VREF .. +VREF
 *
 * Who calls what. init() and release() are setup calls in main(), after
 * the clock task's init() (the generator's rate passed to init() is a
 * claim the prescaler check trusts); init() answers false for a
 * configuration sdadc_config_valid() refuses, a prescaler out of range
 * for the rate given, or a synchronization that never settled. Every
 * synchronized write here waits before it stores (the bus error below),
 * and every wait is bounded - 0xFFFF polls for a synchronization,
 * 0xFFFFFF for a conversion in read(), next() and discard(); read() and
 * next() answer false when theirs runs out, with nothing stored. Those
 * three spin for whole conversions, SKPCNT + 1 decimation windows each
 * for a single one, and belong to main context (setup, or an active
 * object's handler - an active object being a brio component with its
 * own event queue, served one event at a time by the kernel). isr() and
 * resrdy() are the bodies the program's SDADC_Handler calls, and
 * nothing else does. Nothing here masks interrupts and nothing keeps a
 * caller's storage.
 * What the silicon does, and how little of it is like the SAR.
 * A sigma-delta modulator running at CLK_SDADC_FS feeds a third-order
 * SINC decimation filter (39.6.3), and the filter - not a successive
 * filter's order and never the modulator's, so neither does this file.)
 *  - The input is always differential. MUXSEL names a pair of pads
 *    the negative pad grounded externally (table 45-26 note 3), which
 *  - The result is signed, spanning -VREF .. +VREF. 39.1 and 39.8.19
 *    say signed; 39.6.1, 39.6.3.1 and 39.6.3.4 all say unsigned, and
 *    39.6.3.4 even saturates to [0 : 2^16-1]. The chapter contradicts
 *    itself four times; the bench settles it - a rail-to-rail
 *  - And the datapath is twenty-four bits wide, which the chapter never
 *    says. 39.8.19 calls RESULT "a signed integer value with 24-bit
 *    register saturates at +/-2^23 (0x7FFFFF and 0x800000, not 0x7FFF00
 *    and 0x800000), the corrections of 39.6.3.4 act in raw 24-bit units -
 *    an OFFSETCORR of 25600 moves the reading by exactly 100 of the
 *    real filter output. So this file has three result verbs: `result()` is the
 *    and `result_raw()` is the register. Anything about the corrections
 *  - There is no sample length and no resolution knob. The output rate
 *  - SKPCNT is a warm-up, not an average. A sigma-delta's filter has to
 *    fill before its output means anything, so a single conversion
 *    refuses a single-conversion configuration with fewer than two
 * The clock, which has three stages and is easy to get wrong by a factor
 * PRESCALER is eight bits and linear, and this is where the device
 * all. Measured, crystal-ruled: PRESCALER 3/4/7/23 give conversion
 * The four register disciplines, spelled per register - and this chapter
 * disagrees with itself about three of them, so each disagreement is
 * 1. Enable-protected. 39.6.2.1's list is CTRLA.ONDEMAND/RUNSTDBY,
 *    Measured, by writing each raw under a running converter: REFCTRL,
 *    CTRLB and EVCTRL discard the write and CTRLC and ANACTRL take it -
 *    while the converter is disabled, which is correct under either
 * 2. Write-synchronized, and the bus error that makes this chapter
 *    different. 39.6.8: "If an operation that require synchronization is
 *    every synchronized write in this file waits before writing, not
 *    Note REFCTRL: 39.6.8's prose lists it among the registers needing
 *    write synchronization, and SYNCBUSY has no bit for it. One of the
 * 3. Write-only. SWTRIG's two bits are `W` in 39.8.17 and self-clearing;
 * 4. Neither. EVCTRL, INTENSET/CLR, INTFLAG, SEQSTATUS, SEQCTRL, and
 * Not PAC-protected (39.5.8): INTFLAG, and nothing else.
 * The post-processing, which is the reason a Multislope wants this part.
 * which print the formula three more times). It runs in 24-bit units,
 * OFFSETCORR of 1 moves the reading by one 256th of a 16-bit count.
 *  - GAINCORR's reset value is one, not zero, and a GAINCORR of zero
 *    the reset value was zero; the reset values printed in this revision
 *    of the chapter - GAINCORR 0x0001 and CTRLB.SKPCNT 2 - are that
 *  - The gain is an integer over a power of two, not a fixed-point
 *    SHIFTCORR 1. `sdadc_gain_permille()` and `sdadc_corrected()` do
 *    that arithmetic once.
 * Chopping (ANACTRL.ONCHOP) is the analog half of the same idea: 39.6.3.4
 * What the header has and the chapter does not: `REFCTRL.REFRANGE`, two
 * bits at 5:4, present in the header's register mask (0xB3) and in no
 * header wins on what exists, and it is documented as unknown because
 * Errata, DS80000740S, read on the E/G/J row at revision F. Four SDADC
 * items and one device-level item; one is this silicon.
 *  - 1.8.10 DAC Output Reference Selection (device level, all
 *    revisions): with REFCTRL.REFSEL = DAC, starting a conversion makes
 *    the DAC's output voltage noisy. The workaround is
 *    REFCTRL.ONREFBUF = 1, which `sdadc_config_valid()` requires for the
 *  - 1.8.7 DMA Write Access in standby (all revisions) names
 *  - Not this silicon, and each is a read-the-row trap: 1.18.1 (the APB
 *    conversion of a sequence is invalid) is revision B only; 1.18.3
 *    (GAINCORR and SKPCNT resetting to zero) is revision B only, and
 *    1.18.4 (power consumption) are revisions B..E. Note that table
 *    45-26 has absorbed half of 1.18.2 as a specification: for
 *    VREF >= VDDANA - 0.3 V the input conversion range is +/-0.7 x VREF
 * Not built (docs/samc21/sdadc.md carries the list): the converter as a
 * wake source; ANACTRL's CTLSDADC and
    /// The convenience form, for a converter known to be running: 0 when
    /// no result came within the bound. Main context; it spins.
++ b/brio/samc21/sercom.hpp
 * generator and the pad matrix of the shared ch. 30) as a serial port for
 * a program: the console a program prints to, a line-oriented link, a
 * byte transport to another chip, or one of the shapes the USART offers
 * (RS-485, LIN, IrDA, a synchronous host or client, collision detection).
 * The smallest use, the console on SERCOM5's PB30/PB31 (written out in
 * docs/samc21/sercom.md, "How to use it"): the two pads named with the
 * pins and PMUX function they come out on in a UartPads value,
 * `using Serial = brio::Uart<5, console_pads>`, the instance's vector
 * bound by the program to Serial::isr() (below), `Serial::init(clock,
 * 115200)` in main() after the clock task's init(), then print().
 *
 * Who calls what, and from where. init() and release() are setup verbs:
 * main(), after the clock task and before interrupts are enabled
 * globally; init() waits, bounded, for the instance's reset, enable and
 * CTRLB synchronizations and answers false when one did not complete, the
 * rate cannot be made at the clock's rate, or (with the engine) the DMAC
 * block or channel would not reset. isr() is the interrupt
 * body: the program's handler for the instance's one vector calls it and
 * nothing else does, and it runs to completion like every interrupt
 * handler in brio; dma_isr() is the DMAC vector's body, for the transport
 * that holds the transmit engine. The transmit verbs (write_byte(),
 * write_bulk(), and print() over them) and the receive verbs (read_byte(),
 * read_bulk(), read_span() with consume()) belong to the main context -
 * setup, or the handler of an active object, a brio object with an event
 * queue the kernel serves one event at a time - and never to an interrupt
 * handler: each ring has one producer and one consumer, and the interrupt
 * body is the other end of both. Without an engine nothing here masks
 * interrupts (the transmit interrupt is armed by a plain INTENSET store,
 * which cannot race the handler's INTENCLR); with the engine, the
 * engine's claim is taken under the platform's critical section and the
 * channel programmed unmasked (pump_tx()). Nothing in the task waits
 * except init() above and rebase() and set_baud(), main-context verbs
 * that spin, bounded, until the queued bytes have left the wire.
 * Storage: the rings are the task's own static arrays; write_bulk()
 * copies the caller's span, which is free again when it returns;
 * read_span() hands out the ring's own bytes, valid until consume()
 * releases them. The resource's configuring verbs are setup verbs with
 * no guard of their own.
 *
 * The header has the two strata the rest of this target uses:
 *
 *  Sercom<n>   the resource - a typed view of one instance's USART_INT
 *              the flag verbs, and the one combined interrupt question
 *              the task - the interrupt-driven full-duplex byte
 *              error counters, an init() that speaks hertz, and the
 * Scope. USART mode only. The resource speaks the chapter's
 * directions switched at run time (CTRLB.TXEN/RXEN, 31.8.2). The task
 * One interrupt vector, not two, whatever a two-vector transport would
 * lead one to expect. A SERCOM has exactly one line in the NVIC
 * ERROR - so the task offers one ISR body, Uart::isr(), which reads
 * genuinely pending. The app binds exactly one vector (a vector left
 * unbound is the crt's weak alias of its silent spin, where the program
 * stops at the first received byte or the first armed transmit):
 * Synchronization is real here. SWRST, ENABLE and CTRLB cross into the
 * peripheral's own clock domain, and 31.8.10 promises an APB error for
 * such write in this file is followed by a bounded wait (clock.hpp's
 * it). A wait that times out is reported, never hung on.
 * Pads are not pins. The SERCOM knows four pads and nothing else: which
 * TxD cannot go anywhere. CTRLA.TXPO (31.8.1) offers TxD on SERCOM
 * console's PAD[0]/PAD[1] pair, PAD[0] is the transmitter and PAD[1] the
 * Note the device header's own naming trap: it calls TXPO = 0x1
 * enumerator names the code, not the pad. Everything below is named
 * after the pad the signal lands on.
 *  - CTRLA, CTRLB and BAUD are enable-protected (31.6.2.1): a write
 *    while the peripheral is enabled is discarded. Everything that
 *  - the input pull of a SERCOM input pin can only be a pull-down
 *  - STATUS carries the error bits of the character at the head of the
 *    two-deep receive FIFO and must be read before DATA (31.8.11);
 *  - INTFLAG.TXC is cleared by writing DATA and set when the shifter
 *  - erratum 1.17.16: CTRLA.SWRST does nothing while CTRLA.ENABLE = 0.
 *    the write-back): the Uart has one optional engine slot, the
 *    transmit one, filled by samc21/dmac.hpp's DmaTxEngine - the one DMA
 * Where the hot paths run from. The flash runs behind two wait states at
 * in the vector that binds them, so a program that places that vector in
 * template parameter (util/placement.hpp), moves one path for a program
    /// the receive interrupt and its NVIC line. A setup verb, main
    /// context; it waits, bounded, for the instance's synchronizations,
    /// and it starts the transport afresh (the rings and the error
    /// counters cleared, the skip epoch kept).
    /// Call after the main clock is set up and before interrupts are
    /// then knows the transport is not up, rather than printing into a
    /// The instance's one interrupt body - call from SERCOMn_Handler(),
    /// the program's handler for the instance's vector, and from nowhere
    /// else.
    /// full. Main context only (the ring's one producer), never from an
    /// interrupt handler; it never waits. Without an engine, arming DRE
    /// is a plain store to INTENSET, so it cannot race the handler's INTENCLR; with one, the DRE
    /// Fetch one received byte; false when nothing is pending. Main
    /// context only (the receive ring's one consumer); it never waits.
     * many were queued (short of `src.size()` when the ring filled). Main
     * context only, never from an interrupt handler; it never waits, and
     * the caller's span is free again when it returns.
     * The skip epoch (util/stream.hpp's SkippingSource, docs/design/
     * serial.md): a count, modulo 2^8 (the SkipRing's own byte, returned
     * widened) and never cleared - neither by clear_errors() nor by
     * init() -, that moves by one at each skip of the characters the line
     * carried and the receive ring does not deliver clean: a step an
     * event, not a character. A reader that
++ b/brio/samc21/sleep.hpp
 * How deep a brio program on the SAM C21 sleeps: PM, the Power Manager
 * (DS60001479M ch. 19) - the whole of it, which is two registers - plus
 * `SamSleepSite`, the adapter that puts `util/power.hpp`'s depth ladder
 * on this silicon, and `SamTimedSleepSite`, the site that keeps kernel
 * time across a standby. A program that never sleeps deeper than the
 * default needs none of it: the kernel's idle WFI takes IDLE0, the
 * reset mode. A program that wants standby lists util/power.hpp's
 * manager over a site in its Tenuto (docs/samc21/platform.md, "How to
 * use it"):
 *
 *     using Pm = brio::PowerManager<P, brio::SamSleepSite, brio::PowerConfig{},
 *                                   Voters...>;
 *     using K = brio::Tenuto<P, AOs..., Pm>;
 *
 * (Voters... and AOs... standing for the program's own lists) and asks the manager for a depth with a SleepRequested event; the
 * manager arms the site, and the kernel loop's next idle() is the sleep.
 *
 * Who calls what. A site's arm(), disarm() and armed() are the
 * manager's, called from its handler; arm() spins on the SLEEPCFG
 * readback, bounded (0xFFFF reads), and answers false when the mode did
 * not land. Pm::sleep() and Pm::enter() are for a caller that sleeps
 * outside the kernel loop, in main context. SamTimedSleepSite::init() is
 * a setup call in main(), after the clock task's init() and before the
 * manager's first round; its isr() is the body the program's RTC_Handler
 * calls and nothing else does; its resync() takes the platform's
 * critical section (interrupts masked) for the few instructions that
 * consume the baseline, and nothing else here masks. The rule the plain
 * site leaves to the caller: kernel time stops in standby, so a program
 * on SamSleepSite asks for standby only when no time event is armed
 * (below).
 *
 * The mechanism only: which sleep mode the next WFI will take, what the
 * erratum that makes standby entry a hazard. The policy - when a program
 * IDLE0 (0x0) - the reset value, so it is also what a machine that has
 * IDLE2 (0x2) - IDLE0 except that the CAN is not clocked and therefore
 * ## The tick stands still in standby
 *   kernel time stops for exactly as long as the standby lasts, and
 *   every armed time event matures late by that amount.
 * Two sites answer it, and the choice is the application's:
 *    legitimate when the kernel has no armed time event
 *  - `SamTimedSleepSite` (end of this file) lifts it: the RTC on an
 * coincides exactly with the standby entry." Live on every silicon
 * preconditions are the default state of a brio program: BBIASHS reads
 * is the kernel tick.
 * nothing here because of the paragraph above: kernel time stands still
 * Erratum 1.8.14, live on this silicon: entering standby with
 * and keeps requesting GCLK0 - i.e. the setting achieves the opposite
 * therefore accepts performance mode and the comment on the enumerator
 * knowing before clearing or keeping it: with the RAM back-biased the
 * DMAC cannot access it (19.6.4.1's own note - a sleepwalking DMA
 * The PM has no interrupt, no event, no DMA connection and no software
 * reset (19.5.4 is about the interrupt controller, 19.6.6 and 19.6.7 say
 * Its bus clock is one-way. 19.5.2: "If this clock is disabled, it can
 * Debug changes what standby is. 19.5.6: with the CPU halted in debug
 * is attached does not turn the power domains off. A sleep measured with
 * Live: 1.8.13 (above, coded), 1.8.14 (above, the obligation stated),
 * 1.8.7 (a DMA write performed while sleepwalking may not land on some
 * Not this silicon (revision B only - the read-the-row trap the whole
     * Arm `m`, and do not return until the register says so.
     * Warning (19.6.3.3): the wake-up source must be configured,
     * enabled, and reachable in the armed mode before this runs.
    /// established its wake condition. False - and no sleep - when the
 * Two things to read before using it.
 * The never-deeper rule earns its keep here, because the ladder's rungs
 * The register is the state, and that is why `light` is IDLE2 rather
 * It only arms. The WFI is still the kernel loop's: SamPlatform::idle()
 * (this target's own addition) that kernel time stops across a standby,
++ b/brio/samc21/spi.hpp
 * generator and the pad matrix of the shared ch. 30), for a program that
 * talks to SPI devices through the bus arbiter, or answers another
 * board's host as a client. The smallest use, a device client on the
 * arbitrated bus (docs/samc21/spi.md, "How to use"):
 *     using SpiHw = brio::SpiHost<1, my_pads>;
 *     using SpiBus = brio::SpiBus<SpiHw, P>;     // an active object
 *     extern "C" void SERCOM1_Handler() {
 *         if (SpiHw::isr()) { brio::post<SpiBus>(brio::TransferDone{brio::spi_ok}); }
 *     }
 *     SpiHw::init(clock);                        // in main(), after the clock task
 *
 * then an active object posts a Request (the chip select, the buffers as
 * lend<Lease::reply>(...), baud = *SpiHw::baud_for(hz), the mode) to
 * SpiBus and gets an SpiDone back.
 *
 * Who calls what, and from where. init() is a setup call in main(),
 * after the clock task's init() and before interrupts are enabled
 * globally; it waits, bounded, for the peripheral's synchronizations and
 * answers false when one did not complete or a stated SCK ceiling cannot
 * be produced. start() is the arbiter's, from its dispatch in main
 * context: a polled request runs whole inside it, with only the SERCOM's
 * own interrupt silenced and global interrupts enabled, bounded by one
 * spin budget, and the request's cs_setup_us is spent spinning there for
 * both completion styles; an interrupt-driven one returns once its
 * first characters are written. A change of mode or rate costs start()
 * a disable/enable pair and its bounded synchronization waits first.
 * isr() is the interrupt body: the program's handler for the instance's
 * vector calls it and nothing else does, and it answers true on the
 * completion edge, when the handler posts TransferDone; a vector left
 * unbound is the crt's weak spin, where an interrupt-driven request
 * stops the program at its first character. The Request's three buffers
 * are lent from the post until the SpiDone the arbiter replies with:
 * untouched and in scope until then, which nothing checks. The chip
 * select is a GPIO the device client configures (init() claims DO, SCK
 * and, when the pads name one, DI, and nothing else). SpiClient's polled runs belong to the context that
 * calls them and spin, bounded by the caller's budget, until the run
 * completes, the host raises the select, or the budget runs out. The
 * resource's configuring verbs are setup calls with no guard of their
 * own. The parts:
 *
 *  Spi<n>      the resource - a typed view of one instance's SPI
 *              the one combined interrupt question this core's single
 *              the task util/spi_bus.hpp (= util/bus_master.hpp) drives:
 *              the transfer engine. A Request is a two-phase transaction
 * One instance, one address, one set of facts. Everything that belongs
 * to the SERCOM instance rather than to a mode - the APB mask, the GCLK
 * and is used from here; only the register view differs, and that comes
 * The header's two SPI views are the same registers. sercom.h declares
 * file uses one (SPIM) for both roles and the role is CTRLA.MODE and
 * Pads are not pins, and DOPO is a triple. CTRLA.DIPO names the pad the
 * data input is read from and takes any of the four. CTRLA.DOPO does
 * not name a pad: its four codes each fix a whole (DO, SCK, SS) triple
 * the task. Note the device header's naming trap, the same one TXPO has
 * puts DO on PAD[2]; the enumerator names the code, not the pad.
 * Everything below is named after the pad a signal lands on.
 * Which signal is which depends on the role (32.8.1, table 32-2):
 * so on one fixed four-wire harness the two roles are two different
 *  - CTRLA, CTRLB, BAUD and ADDR are enable-protected (32.6.2.1): a
 *    write while the peripheral is enabled is discarded, not refused.
 *    Writing CTRLB while SYNCBUSY.CTRLB stands is an APB error, so
 *  - enabling the peripheral clears CTRLB.RXEN (32.8.2) and raises
 *    enable() here waits out both synchronizations;
 *  - a client's DATA write needs three SCK cycles to reach the shift
 *  - RXC is set when a character has been fully shifted in, and reading
 *  - STATUS carries one error bit, BUFOVF (32.6.2.7), and CTRLA.IBON
 *  - there is no event surface at all: 32.5.6 and 32.6.4.3 are both
 *  - erratum 1.17.16: CTRLA.SWRST does nothing while CTRLA.ENABLE = 0,
 *    on every revision by the sheet; not reproduced in SPI mode on this
 *    silicon (measured: SWRST from the disabled state resets the block,
 *    its synchronization completing; reset()'s comment). reset() keeps
 *    the discipline and enables the instance first when it finds it
 *    disabled - in SPI host mode, because the enable synchronizes
 *  - erratum 1.17.3 (live on every revision): with CTRLB.PLOADEN set,
 *    holds SS low for the whole transmission. The driver cannot enforce
 *  - erratum 1.17.19 (live on the sheet): DBGCTRL is reset by
 *    CTRLA.SWRST although 32.6.2.2 promises it is the one register a
 *    reset spares. Not reproduced on this silicon (measured: DBGCTRL
 *    reads 0x1 across SWRST from both states, docs/samc21/spi.md);
 *    configure() writes DBGCTRL after any reset, correct under either
 *    answer;
 *  - erratum 1.17.20 (live): a client left with a preloaded character on
 *  - erratum 1.17.1 is revision B only on the E/G/J row (the spurious
 *    without checking the row, and it does not apply to this silicon.
    /// pads, pins, the NVIC line. Setup's call, main context; it waits,
    /// bounded, for the peripheral's synchronizations.
    /// Call after the main clock is set up and before interrupts are
    /// statement of the rate. The pads handed over are DO, SCK and DI
    /// (when the pads name one); the chip selects are each device
    /// client's own GPIO outputs, which it configures (high) before its
    /// first request.
++ b/brio/samc21/supc.hpp
 * analog block on this die eventually asks for. What a program gets
 * from it: whether VDD is below a threshold and what a crossing does,
 * which regulator feeds the core in standby, and the bandgap's level.
 * The smallest use (docs/samc21/supc.md, "How to use it"):
 *     brio::BodVdd::configure({.level = 40, .action = brio::BodAction::none});
 *     const bool below = brio::BodVdd::detected();
 *
 * `action` defaults to `reset`, so a configuration that leaves it out
 * resets the device whenever the supply is below the level.
 *
 * Who calls what. BodVdd::configure() and enable() are setup verbs, for
 * main() or an active object's handler (an active object is a brio
 * component with its own event queue, served one event at a time by the
 * kernel): they wait on BVDDSRDY before every store, each wait bounded
 * (0xFFFF polls), and answer false when one runs out. Vref::configure()
 * and the regulator's run_standby() are single stores or
 * read-modify-writes with no guard of their own, setup verbs too. The
 * status reads are single loads, legal from any context. Supc::isr() is
 * an interrupt body for IRQ 0, which five blocks share: the program's
 * SYSTEM_Handler calls it beside the others' bodies. Nothing here masks
 * interrupts.
 *
 *   Supc        the block - the status register, the interrupt surface
 *               ENABLE bit needs. Its reset value is a fuse.
 *   BodCore     the VDDCORE detector, read-only here on purpose.
 *               that routes it to an ADC input channel, and its sleep
 * What makes this chapter different from its neighbours: three of these
 * registers come up with values nobody in the program wrote. The BODVDD
 * Four rules, each with code behind it.
 * 1. BODVDD is enable-protected and write-synchronized at once, which
 *    the protected fields are discarded while ENABLE is 1 and raise an
 *    configuration, wait, set ENABLE on its own, wait. `configure()`
 *    is that sequence and nothing else. The last step is not
 *    cosmetic: a store carrying the configuration and ENABLE = 1
 *    were - the protection is judged on the value being written, not
 * 2. The level is a field and not a voltage. Table 45-18 gives three
 * 3. BODCORE is not yours. 22.6.3.4: it is calibrated in production,
 *    is exposed read-only, and the header offers no verb that writes
 *    it. Note that the register exists at all only according to the
 * 4. The main regulator cannot be disabled. 22.8.6 says VREG.ENABLE
 * Errata. 1.8.14 is live on this silicon and belongs to PM
 * Not built (docs/samc21/supc.md carries the list): nothing here forces a
 * observed across a sleep, because a detection is a supply crossing
 * Two things to know before using it. Its reset value comes from the
    /// STATUS.BODVDDDET: the supply is below the threshold right now.
    /// A single load, legal from any context.
    /// input channel", and that is what the bench finds it does: the
    /// ADC's bandgap channel is dead without it, while the analog
    /// comparator's MUXNEG bandgap selection and the DAC's reference reach
    /// the bandgap internally and behave the same with it clear or set
    /// (measured, docs/samc21/supc.md).
++ b/brio/samc21/tc.hpp
 * modes, one event input and three event outputs - what a program uses
 * for a PWM output on a pad, a counter that times an interval, or a
 * capture of an input's period and pulse width. The smallest use, a PWM
 * on the bench board's LED (docs/samc21/tc.md, "How to use it"):
 *   using LedWave = brio::TcWo<brio::Pin<'B', 23>>;   // TC3, WO1
 *   using LedPwm = brio::TcPwm8<brio::Tc<3>, 1, 199>;
 *   brio::Tc<3>::init(0);                       // generator 0
 *   LedWave::claim();
 *   LedPwm::setup(brio::TcPrescaler::div256);
 *   LedPwm::duty(100);
 *
 * Which context calls what. init(), release(), configure(),
 * event_config() and the tasks' setup() are setup verbs, from main() or
 * an active object's handler (an active object is a brio component with
 * its own event queue, served one event at a time by the kernel); the
 * value verbs (duty(), the set_* writes, the commands, the synchronized
 * reads) belong to one such context at a time. Each is a store or a
 * read-modify-write followed by a bounded wait on SYNCBUSY (0xFFFF
 * polls by default) that answers false when it runs out; nothing here
 * has a guard of its own or masks interrupts. init() leaves the NVIC
 * line disabled and the timer disabled. isr() is the interrupt body:
 * the program arms the flags it wants, enables the line, and calls isr()
 * from its TCn_Handler, nowhere else. The counter counts its generic
 * clock generator, not Clock::hz, and the generator is the caller's to
 * name.
 *
 *  Tc<n>            the resource: the whole chapter's register surface -
 * Six facts that shape the file.
 * 1. The geometry is the device header's, all of it. How many
 *    each uses, which instances can be paired into a 32-bit counter -
 *      - TC0 and TC1 share generic clock channel 30, TC2 and TC3 share
 * 2. The pad-to-(TC, WO) map is not a formula either, and it is
 * 3. Reading COUNT is a command, not a load. COUNT is
 *    read-synchronized "on demand through the READSYNC command"
 *    what `count16()` and its siblings do (twice: the comment above
 *    read_sync() says why) - the raw accessors are
 * 4. The register map is three overlaid views. The device header gives
 * 5. Enable-protection and write-synchronization are different things
 *    EVCTRL are enable-protected: `configure()` and `event_config()`
 *    CC/CCBUF are write-synchronized: every verb that touches one
 * 6. The period register exists only in 8-bit mode (35.6.2.4). In
 * Errata, DS80000740S, read on the E/G/J row at revision F - and once
 *    as generator): revision B only. This is the item most likely to be
 *    applied by mistake - CTRLA.COPEN is implemented here, and
 *    docs/samc21/tc.md says why the bench declines a verdict on it (a
 *    controlled edge cannot reach a muxed WO pad with no wire).
 *  - 1.20.3 SYNCBUSY Flag: every revision of E/G/J, so live here. When
 *    before the buffer register has been restored, and the remedy is to
 * Not built (docs/samc21/tc.md carries the list): the minimum and maximum
 * header does not declare CTRLA.CAPTMODE at all), and CTRLA.ONDEMAND's
 * clock-request behaviour, which `TcConfig::on_demand` writes and
 * nothing owns; CTRLA.RUNSTDBY, the bit that keeps a TC counting through
 * a standby, is `TcConfig::run_standby`.
    /// the TC is left disabled because CTRLA, WAVE, DRVCTRL and EVCTRL
    /// are enable-protected and the caller has them to write, and its
    /// NVIC line is left disabled. A setup verb; false when the channel
    /// connection or the reset's bounded wait did not complete. Connecting
    /// the channel routes the sibling's clock too (TC0/TC1, TC2/TC3).
    /// Bring the timer up as an 8-bit NPWM generator with PER = top and
    /// the channel at duty 0, and enable it. The caller has already
    /// called T::init() and claimed the pad. False when configure()
    /// refuses (the timer already enabled) or a bounded wait runs out.
    /// The buffered duty, clamped to `max`, taken at the next UPDATE;
    /// the bounded wait's answer is discarded.
++ b/brio/samc21/tcc.hpp
 * What a program uses for a PWM of any period, a complementary pair with
 * dead time, or outputs a fault clamps in hardware. The smallest use, a
 * PWM channel on PA08 (docs/samc21/tcc.md, "How to use it"):
 *   using OutWave = brio::TccWo<brio::Pin<'A', 8>, brio::PinFunction::e>;
 *   using Pwm = brio::TccPwm<brio::Tcc<0>, 0, 1999>;   // TCC0 / WO0
 *   brio::Tcc<0>::init(0);                             // generator 0
 *   OutWave::claim();
 *   Pwm::setup(brio::TccPrescaler::div256);
 *   Pwm::duty(500);
 *
 * Which context calls what. init(), release(), the configuring verbs and
 * the tasks' setup() are setup verbs, from main() or an active object's
 * handler (an active object is a brio component with its own event
 * queue, served one event at a time by the kernel); the value verbs
 * belong to one such context at a time. The direct setters and the
 * commands are a store followed by a bounded wait on SYNCBUSY (0xFFFF
 * polls by default) that answers false when it runs out; the buffered
 * setters never wait and answer false when the previous buffered value
 * has not been taken yet (fact 8), so a second TccPwm::duty() inside one
 * period is dropped. Nothing here has a guard of its own or masks
 * interrupts. init() leaves the timer and its NVIC line disabled; isr()
 * is the interrupt body the program's TCCn_Handler calls, once the
 * program has armed the flags and enabled the line. The counter counts
 * its generic clock generator, not Clock::hz, and the generator is the
 * caller's to name.
 *
 *  Tcc<n>            the resource: the whole chapter's register surface.
 *                    carries it - and the function is part of the key,
 * Nine facts that shape the file - the last two of which are not in the
 * 1. The three instances are not copies of each other, which is the
 *    instance for a unit it does not have is refused at compile time
 * 2. The pad map is keyed by pad **and function**. A TC waveform output
 * 3. Enable-protection and write-synchronization split differently here
 *    than in the TC, and the difference is useful rather than pedantic.
 *    refuse while the TCC is enabled. Write-synchronized and not
 *    circular-buffer bits can all be changed under a running timer,
 * 4. Reading COUNT is a command. COUNT is read-synchronized "on demand
 *    SYNCBUSY.COUNT, then read - and, as in the TC, the command is
 *    issued twice, because the COUNT shadow lands after SYNCBUSY clears
 *    with no bit advertising it (samc21/tc.hpp's read_sync() comment
 *    carries the measurement). `count()` does it; `count_raw()` skips
 * 5. The fault inputs are the event inputs. Recoverable Fault A and B
 *    on asynchronous event channels - the chapter says so for the fault
 *    caller obligation this header states and cannot enforce: evsys.hpp
 *    needs its channel's incoming event enabled (EVCTRL.MCEIx), because
 * 6. The period register is always there, unlike the TC's, and PER is
 * 7. Dithering shares the low bits of COUNT, PER and CCx. With
 * 8. A buffered write's SYNCBUSY bit does not clear until the update
 *    consumes the buffer - measured, and nowhere in the chapter.
 *    of a 1333 us period, measured) and for ever while CTRLB.LUPD is set.
 *    A second write into that window is discarded by the silicon - also
 *    measured. So every buffered setter here refuses instead of waiting:
 *    Its twin, from the same measurement: a read of CCx while a buffered
 *    write is pending returns the buffered value, not the one the
 * 9. The two CTRLB registers are not a set/clear pair for the command
 *    fields. DIR, LUPD and ONESHOT behave as one expects - a '1' to
 *    written as a value, and "writing zero to this bit group has no
 *    effect" on both halves (36.8.2, 36.8.3). So a command is issued
 *    through CTRLBSET and cancelled through CTRLBCLR, and a driver that
 * Errata, DS80000740S, read on the E/G/J row at revision F. This chapter
 * has eleven items and seven of them are live here.
 *  Live:
 *    do not work unless every upper channel is also in one of those
 *    clears twice, which is the whole workaround (and the exact twin of
 *  - 1.21.8 PERBUF: in down-counting mode CTRLB.LUPD is said not to
 *    It did not reproduce here: with the waveform as the witness - the
 *    refused by `tcc_config_valid()` - the one erratum in this chapter
 *    broken for RAMP2C and RAMP2CS only. Those ramp modes are variant-L
 *  Not this silicon (revision B only - do not code around them):
 * Not built (docs/samc21/tcc.md carries the list): the debug-fault state
 * something a bench suite can stage). CTRLA.RUNSTDBY, the bit that keeps
 * a TCC counting through a standby, is `TccConfig::run_standby` here;
 * the fault inputs in a standby are implemented and not bench-verified.
    /// APB clock, generic clock, software reset; left disabled, because
    /// caller has them all to write, and with its NVIC line disabled. A
    /// setup verb; false when the channel connection or the reset's
    /// bounded wait did not complete.
    /// Hand everything back: the line, a reset, the generic clock
    /// channel and the bus clock. TCC0 and TCC1 share one channel (the
    /// header's TCCn_GCLK_ID), so the disconnect stops the sibling too.
    /// has already called T::init() and claimed the pad. False when a
    /// configuration is refused (the timer already enabled) or a bounded
    /// wait runs out.
    /// The buffered duty, clamped to `max`. Never waits: a value written
    /// while the previous one has not been taken yet is refused by
    /// set_cc_buffer() (fact 8), and that refusal is discarded here.
++ b/brio/samc21/ticker.hpp
 * The kernel timebase of this target, on the Cortex-M0+ SysTick: the
 * `Ticker` a program's SamPlatform reads its time from (samc21/
 * platform.hpp), counting 1000 ticks a second, and the guard that keeps
 * the SysTick interrupt off across a standby entry. The smallest use is
 * the Usage block at the end of this comment: the program binds
 * SysTick_Handler to Ticker::tick(), then calls Ticker::init(clock) in
 * main() after the clock task's init() and before interrupts are
 * enabled.
 *
 * Who calls what. init() is a setup call in main(), and it answers
 * false, starting nothing, when the reload the clock's rate gives does
 * not fit SysTick's 24 bits; it computes that reload from the rate the
 * clock type claims, so an init() before the clock task's runs the tick
 * at the wrong rate and nothing refuses it. tick() is the interrupt
 * body: the program's SysTick_Handler calls it and nothing else does.
 * The getters may be read from any context (the Concurrency note
 * below); now() masks interrupts for the two reads it makes. advance()
 * is the timed sleep site's (samc21/sleep.hpp), pause() and resume()
 * the escape hatch the class's own comment describes.
 * SysTickInterruptGuard is the platform's idle() and Pm::sleep()'s.
 *
 * The ticker itself is the core stratum's: `BasicTicker` lives in
 * Why SysTick and not a TC. SysTick is core-private: no application can
 * Monostate (a class whose state and functions are all static, with no
 * instance to create): every member is a static inline, there is one
 * timebase per program because there is one SysTick, and the ISR body
 * reaches its counters with no pointer indirection. State lives in
 * .bss, zeroed before main().
 * (atomic_width 4), so no getter masks interrupts for a single counter.
 * But atomicity is not visibility: the getters read through a volatile
 * ## The same caveat, in its second half: standby stops kernel time
 * in the PM's STANDBY sleep mode kernel time stands still for exactly as
 * the debug probe attached, it was measured running through a standby:
 * resynchronizing this counter from the RTC after every wake -
 * BasicTicker's `advance()` (cortexm/ticker.hpp) is that resync's
 * landing point.
 * Hold the SysTick interrupt off for a scope, and put it back exactly as
 * This exists for one reason: erratum 1.8.13 (DS80000740S, live on every
 * set - STDBYCFG.BBIASHS, whose reset value is 1 - a SysTick interrupt
 * state of a brio program, since the SysTick interrupt is the kernel
++ b/brio/samc21/tsens.hpp
 * The SAM C21's on-die temperature sensor (DS60001479M ch. 43): the
 * die's temperature in hundredths of a degree Celsius, measured once,
 * averaged, watched by the window monitor, or paced by an event. A whole
 * small chapter in one monostate resource (a class whose state and
 * functions are all static) - `Tsens`, not `Tsens<n>`: this family has
 * exactly one instance on every variant, so an index would be a
 * parameter with a single legal value (the `Rtc` / `Dac` / `Sdadc` shape
 * in this stratum). The smallest use:
 * with `generator` running OSC48M undivided (generator 0 under a 48 MHz
 * internal main clock), the one rate the factory calibration holds for.
 *
 * Who calls what. init(), release(), calibration(), window(),
 * event_config() and start_on() are setup verbs, for main() or an active
 * object's handler (an active object is a brio component with its own
 * event queue, served one event at a time by the kernel); their waits on
 * SYNCBUSY are bounded (0xFFFF polls) and a false answer means one ran
 * out or the configuration was refused. read(), measure() and
 * measure_average() spin the caller until a result is ready, each wait
 * bounded at 2 000 000 polls, and answer nothing on a timeout or an
 * overflow: one measurement is about 3.7 ms at 48 MHz, so the average of
 * ten holds its caller about 37 ms. start(), value() and the flag verbs
 * are single loads or stores. isr() is the interrupt body the program's
 * TSENS_Handler calls; init() leaves the NVIC line disabled. Nothing
 * here masks interrupts.
 *
 * This is not an ADC channel - not a voltage into a converter with a
 * correction applied - and everything else follows from that.
 * It is a clock ratio. A temperature-dependent oscillator (TOSC) is run
 * difference of the two periods, amplified over GAIN periods of
 * so the generic clock is the measurement's ruler, not merely its pace.
 * 1. The factory GAIN and OFFSET are calibrated for one clock - "the
 *    and only under it, VALUE is a temperature in hundredths of a degree
 *    Celsius: 43.8.10's own example gives 2500 for 25 C and -2500 for
 * 2. Any other GCLK_TSENS rate rescales the answer, by exactly
 *    offered as constexpr arithmetic that takes the rate as a caller
 *    argument (the samc21/freqm.hpp `reference_hz` pattern - a ratio
 *      - `tsens_rescale(value, offset, f)` scales the reading, for a
 * 3. And the reference's own error is in the answer. OSC48M on the bench
 *    small error on a temperature and a large one on the span from
 * Calibration is the chapter's heart (43.5.9). Four factory values live
 * must be copied in by software: GAIN and OFFSET into their own
 * Without them the peripheral is not merely inaccurate, and its reset
 * value is a trap. GAIN reads zero out of reset, and a zero GAIN is not
 * the value that reads as 0 behaves as 2^24. Measured, twice over
 * The register disciplines, spelled per register the way this stratum's
 * 1. Enable-protected (43.6.2.1): CTRLC, EVCTRL, WINLT, WINUT, GAIN,
 *    OFFSET, CAL - and CTRLA.RUNSTDBY, which is a bit and not a
 * 2. Write-synchronized (43.6.7): CTRLA.SWRST and CTRLA.ENABLE, and
 *    nothing else - SYNCBUSY has exactly two bits and the device header
 *    masks it 0x3. The wait is on the near side of every store, and
 * 3. Not PAC-protected (43.5.8): CTRLB and INTFLAG - the pair an ISR
 *    needs. See the erratum below: this is exactly the
 * 4. Not reset by a software reset (43.8.1): GAIN, OFFSET, CAL and
 * 5. Write-only: CTRLB is `__O` in the device header and drawn with W
 * How wide is VALUE? The chapter says both. 43.6.4 introduces
 * the VALUE register"; 43.8.7's own bit description says twenty-four,
 * The window monitor (43.6.2.4) compares VALUE against WINLT and WINUT,
 * both signed 24-bit. Six modes plus a Reserved seventh. One of them is
 * described twice and differently: 43.8.3's table prints OUTSIDE as
 * agree) and refuses nothing for OUTSIDE; the bench says which reading
 * Errata, DS80000740S, read on the E/G/J row at revision F.
 *  - 1.19.1 PAC Write-protection, live on every revision: "when PAC
 *    registers PAC protection does not cover. The obligation is the
 *    caller's and this header states it rather than coding around it:
 *    PAC write protection is off
 * Not built (docs/samc21/tsens.md carries the list): the block as a wake
++ b/docs/samc21/ac.md
This document is for a programmer who needs to know whether a voltage
is above or below a threshold without a conversion: a pad against a
fraction of the supply, two pads against each other, a signal inside
or outside a window, or a crossing that starts or ends something on
the event system with no CPU in the loop. The AC holds four analog
comparators; `Ac` is the block (its clocks, reset, enable and the one
interrupt flag register), `AcComparator<n>` one comparator, and
`AcWindow<w>` a pair of them as a window comparator.

The smallest use, from "How to use it" below - PA04 against half the
supply:

```cpp
brio::Ac::init(1);                        // GCLK_AC from generator 1
using Comp = brio::AcComparator<0>;
Comp::configure({
    .positive = brio::AcPositive::pin0,   // AIN[0] = PA04
    .negative = brio::AcNegative::vscale, // its own VDD ladder
    .speed = brio::AcSpeed::high,
    .out = brio::AcOut::synchronous,      // sampled, on the CMP0 pad
});
Comp::scaler(31);                         // threshold VDD/2
Comp::enable(true);
while (!Comp::ready()) { }
bool above = Comp::state();
```

Line by line: `Ac::init(1)` opens the block's bus clock, connects its
generic clock channel (GCLK_AC, which clocks the sampling, the filter,
the edge detection and the synchronized output) to generator 1, resets
the block and enables it, with no comparator running yet; `Comp` names
comparator 0; `configure()` writes the comparator's whole control
register - the positive input is AIN[0], the pad PA04, the negative
input the comparator's own 64-step divider of VDD, the fast bias, and
the sampled output routed towards the CMP0 pad - and leaves the
comparator disabled; `scaler(31)` sets that divider to VDD x 32/64;
`enable(true)` starts the comparator; the loop waits for STATUSB.READY,
before which the state is undefined; `state()` is the sampled output,
true when the positive input is above the negative.

What decides correctness even there. The generator `init()` is given
must already run (`Gclk<1>::configure()`, [clock.md](clock.md)): every
digital stage of the block samples at it, and the block's reset and
enable are write-synchronized waits that `init()` bounds and reports
as false when one does not complete. `configure()` answers false, and
writes nothing, for an input the package does not bond (COMP2/3 bond
only AIN[5:4] on the E and G variants), for hysteresis in single-shot
mode and for the end-of-comparison interrupt in continuous mode; every
comparator field is writable only while that comparator is disabled,
which is why `configure()` disables first and `enable()` is a step of
its own. The ready loop is the example's own wait, unbounded: READY
arrives about 6.7 GCLK_AC periods after a continuous-mode enable
("What the silicon does"), so a stopped generator holds the loop for
ever. A pad routed by the output field still needs its PMUX (the note
under "How to use it"). With the bandgap as the negative input, a
spurious flag can rise at the enable (erratum 1.5.6, below), so the
flag is cleared before an interrupt is armed. An armed interrupt
arrives on `AC_Handler`, which the program binds to `Ac::take_flags()`;
the crt's default for an unbound vector is a silent spin.

The page, in order: the documents of record and the errata; what the
silicon does (the sampling clock and what each path costs, window
mode, the event surface, the per-package inputs, enable protection);
the types and verbs; how to use it; the bench findings of the
chapter's own suite and of the DAC-driven suite; the second pair, the
bandgap and the hysteresis; what is not covered.


## Local batch: the avrxt stratum

Scope: the nineteen peripheral documents of `docs/avrxt/` (every page
but README.md and vendor/README.md) in the additive pass, and the
comments of all twenty-five headers of `brio/avrxt/`. Base:
`docs-rewrite` at a70d4b23b8e46be0fb89bff8890f1b8b8214bcd2, the
reference of every token check; local branch `docs-avrxt`, nothing
committed, the work handed over as a patch. Not touched: every code
token, apps, CMake, presets, ld, crt, svd, `.vscode`, `.clangd`, every
other stratum, the design pages, the learning route,
docs/avrxt/README.md and vendor/README.md (no link or figure moved),
CLAUDE.md, docs/README.md. No `// build:` or `// mcu:` line exists in
the headers. Environment: a local machine with the toolchains under
`/sw`; the token check lexed with `/sw/gcc/bin/g++` 16.2; no image was
built, and no `brio check`, host test or board was used (none was
needed); spi.md's two examples and spi.hpp's smallest use were
compiled in a scratch translation unit (below).
The local vendor PDFs (DS40002247B, DS40002443A, DS80001048D; the DA's
data sheet and the DA/DB errata are not on the machine) were rendered
to text and used to check a claim before it was written, never to add
a fact the old text did not carry, with the exceptions named below.

The work was split: the platform and pin pages and the platform, pin,
sleep, reset and device-tables headers by hand, the rest by nine
agents under one written brief (the additive pass, comments only, the
previous batches' slips as the list of what not to do, a removed-token
comparison per file), each paired document with its header; their
reports were checked and their diffs spot-read against the code.

### The documents

Each of the nineteen opens with whom it is for and what the driver
does in a program; the smallest use, an excerpt of the page's own "How
to use it" block (platform.md's the platform line and target glue of
the maintained example `blink`, quoted from
`avrxt/src/apps/blink.cpp`), explained line by line; the prerequisites
that decide correctness even there, each checked against the header
(what the code checks, what it leaves to the program); and, on the
pages over about 300 lines, a reading guide. "Documents of record"
became a heading on every page, its paragraph kept with every document
number, section, errata item, driver and suite name. A suite's letter
is defined beside the suite's name, with that suite's own keys read
from its app header: `z`, and `y` for the partner-board set, where the
suite has them, `a` where its all-key is that (`test_avr_pin`, `test_avr_timer`,
`test_avr_rtc`, `test_avr_analog`, `test_avr_clock`), and the step keys
where there is no all-key set. Terms defined where a page first uses
them: a concept, a monostate, a task, a resource, an active object
(linked to the learning route), a run, the reserve, a bus tenure, a
ClockUser, the power manager, the benchmark skeleton (the app file's
name replaced by that phrase outside the bench findings). The kernel.md
links are by heading. Capitals used for emphasis are lowered in the
orientation, "Documents of record" and "What the silicon does" (and, on
some pages, "Types and verbs"); acronyms, register, bit and enumerator
names are kept. The "Bench findings" sections and every subsection
under them, and the "Not covered yet" sections, are byte-identical to
the base on all nineteen pages (checked by a script that compares each
such section of the base with the new page).

- platform.md: the orientation (the Platform concept and `AvrPlatform`,
  the three services beside it, the power manager and an active object
  defined), `blink`'s platform lines, four prerequisites (the tick's
  vector, with what an unbound one does; `idle()` taking an armed
  mode; no handler nesting, and `CPUINT.LVL1VEC` the one way to break
  it; `break_here()` a NOP without a debugger), the reading guide; the
  "These are not peripheral drivers" paragraph moved, word for word,
  from under the documents of record into the orientation; a new
  paragraph in "What the silicon does" on interrupt priority (below).
- port.md: the orientation (a pin as a type, `Port`, `PinSet`,
  `PinRef`), the button excerpt, four prerequisites (the handler clears
  its flags, one vector per port, which pins wake which sleep on which
  edge, the read-modify-write configuring functions against the single
  instructions).
- clkctrl.md, usart.md, spi.md, twi.md, nvm.md, userrow.md, rtc.md,
  evsys.md, ccl.md, tca.md, tcb.md, tcd.md, adc.md, dac.md, vref.md,
  ac.md, opamp.md: the same shape; each die's facts kept with its die
  (the TCD the DA's and DB's, the OPAMP the DB's, nvm.md's and adc.md's
  two generations each in its half).

Sentences corrected in the documents' existing sections, each against
the code or the page's own findings:

- usart.md, spi.md: the routes are "two bits per instance" on the DA/DB
  only; the AVR EA's USART0 field is three bits (`#if
  PORTMUX_USART0_gm == 0x07`) and its SPI0 field three bits with one
  instance.
- spi.md "How to use it": the bus example passed raw pointers into the
  Request's `Borrowed<..., Lease::reply>` fields, which `Borrowed(T*)`
  (explicit) refuses; it now lends them with `brio::lend`, as the apps
  do, and posts `SpiHw::status()`; the by-hand example's designated
  initializers follow `SpiConfig`'s declaration order. Both old forms
  were compiled with avr-g++ for the AVR128DB48 and fail, both new
  forms compile.
- twi.md "How to use it": the same `lend` fix, and the Request names
  `fast_400k` (one that names no speed runs at `standard_100k` whatever
  `init()` programmed); the `TwiRoute` row gains `alt3`; "noticing a
  stuck transaction is not built" gives that to a timed `I2cBus`
  (`i2c_timeout` and `recover()`, the page's own letter t finding) and
  leaves the decision to spend `unstick()` with the application.
- ac.md "How to use it": the window example posted `Band{...}`, the
  `Window` type whose constructor is deleted; it posts the program's
  own `BandState{...}`.
- clkctrl.md: "all clock-control registers are protected" against
  `Oschf::tune()` and `ClockFailure::clear()`, plain stores: three
  registers have no CCP (DS40002247B 12.5.5, 12.5.6, 12.5.8, read in
  the local PDF); `Xoschf` is the DB's and the EA's, `Pll` the DA's
  and DB's, `ClockFailure` the DB's and the EA's; "What the silicon
  does" says it is the DA/DB's chapter, the EA's being its own section.
- adc.md: the DA/DB `rebase()` aims at the CLK_ADC in force before the
  switch (`clk_adc_hz_ = adc_clock_hz(hz, p)` on every rebase), not at
  init's, which stays the stated intent and its gap; the EA's
  accumulated sum is 22 bits and a differential one a signed 22
  (4095 x 1024 = 4193280 < 2^22; -2048 x 1024 = -2^21), where the page
  said a signed 23; the EA's `start()` is one load and one store to
  COMMAND beside the store of `start_` (the release listing).
- vref.md: a duplicate header row that broke the level table deleted
  (no data row removed); the EA's `Adc<0>::init(clock,
  AdcConfig{.reference = Ref::v2048})` returns false as written
  (`adc_clock_valid()` asks a `sample_duration` of at least 8 at 2 MHz
  with an internal reference, the default being 2), said beside it.
- dac.md: the 1 uA sink is DS40002247B 39.19's, the EA's table 35-22
  giving no drive figure while its findings show milliamperes; the
  INL/DNL line attributed to the DB data sheet.
- tcb.md: the TCB4 ALT1 erratum named "errata DB 2.13.2".
- tcd.md: the DA/DB errata sheets each with its die.
- evsys.md, ccl.md: the hardware-event paragraph moved into the
  orientation; "the static allocation is its natural companion" said as
  the gap evsys.md lists.
- nvm.md: `flash_section_block` is the driver's own constant (512,
  11.3.1.1), not the device header's; the flash read verbs named as the
  header spells them (`flash_read`, `flash_read_word`).

### The headers

Comments only; every one of the twenty-five token-identical to the
base. Each opening now says what the component does for a caller and
the smallest use in the apps' spelling (the clock alias and tag,
`ISR(<vector>)`, `SysClock::init()` first, `sei()` before `run()`),
then which context calls what (the setup verbs; the interrupt bodies,
called from the program's `ISR()` and from nowhere else; what may run
from an active object's handler and what from any context; what waits
and its bound; what masks and around what; who owns a buffer until
when), then every fact the old comment gave with its citation, the
capitals lowered. The public operations the smallest uses call gained
the sentence they lacked.

Stale claims corrected to the code's truth:

- platform.hpp: "the one header of the kernel family allowed to touch
  AVR headers" (every header of the stratum includes `avr/io.h`)
  replaced by what the header is; the opening now states how this core
  keeps the kernel's no-nesting promise (below).
- pin.hpp: the opening named the AVR DA/DB alone (the EA's
  `event_selection()` is in the file); `Pin::port()`'s comment cited a
  `static_assert(false)` in an instantiated branch that the code does
  not have (the refusal is `Port<L>`'s `static_assert` on
  `port_exists(L)`).
- ticker.hpp: "pit() runs with interrupts disabled (AVR default)" was
  wrong (DS40002247B and DS40002443A 7.6.3: the I bit is not cleared on
  entry); `init()` takes XOSC32K when `RtcClock::preferred()` finds it
  enabled and stable, not "when the clock init found" one; the waits'
  answers are discarded, said so.
- usart.hpp: the routes' "two bits per instance" scoped to the DA/DB;
  `Uart::init()`'s "DRE enabled by write_byte()" names `write_bulk()`
  too; `rebase()`'s wait has no bound, said so.
- spi.hpp: `clear_write_collision` does not exist
  (`clear_flags_by_data_access()` does); `SpiHost::apply_mode` does not
  exist (`reprogram()` behind `apply()`); the glue posted to the alias
  template `SpiBus`, which `post` cannot take (now `Bus`, the alias
  spelled out); "no DMA on AVR Dx" is the DA, DB and EA.
- twi.hpp: `speed_ok()` is whether `twi_baud_for` reaches the speed and
  the host refuses an unreachable one (`start()` true with
  `i2c_rejected`), not a readback the tasks only report; tOF "bench:
  125 ns on slew-limited pads" against the page's measured "tOF = 166
  ns in Standard and Fast mode but 0 in Fast-mode Plus" (now the
  page's figure); "three shapes" listing four; the two-step arithmetic
  listing three; the `twi_baud_for` block moved onto its function; the
  "not built" timeout claim as in twi.md.
- clock.hpp: the DA's own data sheet named (DS40002183A ch. 11, as
  clkctrl.md has it); the EA's OSCHFCTRLA reset value carries the
  page's bench reading (0x00 against 0x0C on paper); on the DA/DB the
  OSCHF waits' result is ignored, said so instead of "every bound that
  runs out returns false".
- delay.hpp: "some 35 cycles for 1 us at 24 MHz where the fixed-point
  tail below costs 171 (counted in the listing: docs/avrxt/platform.md)"
  against platform.md's 39 counted and 38 measured, 122 at a static
  rate and 157..160 under a dynamic clock (now those figures, with the
  tail as `delay_us_runtime` reaches it said unmeasured).
- nvm.hpp: version 2's "every verb waits and clears the command on its
  way out" is not true of the byte EEPROM verbs (`eeprom_store` returns
  with the command selected, `EepromStore::finish()` clears it);
  "never a write to FLMAP" against `set_flmap()`.
- nvm_flash.hpp: the build id is the newest source's timestamp
  (`AVR_NVHEAP_BUILD_ID`, avrxt/CMakeLists.txt), not "the epoch of the
  link".
- tca.hpp: `TcaPwm::init()` leaves the compare outputs off
  (`CTRLB = 0`, PORT driving the pins low), not "all six on";
  `Tca::restart()`'s erratum is DB 2.12.1 and DA DS80000882C 2.11.2;
  `Heartbeat`'s `pulse<ch>(ticks)` is `pulse_us<ch>(us)`; `TcaPwm`
  refuses every TCA1 three-channel route, not only PORTC; the TCD
  alternate-route erratum is not "stated in the route table" (tcd.hpp
  and tcd.md).
- evsys.hpp: `EvOut::listen()`/`unlisten()` and the slotted `source()`
  are read-modify-writes, not "one register write each, callable from
  any handler"; the EVOUT ALT1 pins the code allows; the static
  allocator "planned" said as not built, with evsys.md's gap.
- ccl.hpp: `init()` writes CTRLA = 0, then CTRLB, CTRLC and TRUTH, then
  CTRLA with ENABLE, not the order the comment gave.
- ac.hpp, opamp.hpp: READY in the both-bits regime (the page's bench
  finding: no READY event with both bits set); `InstrumentationAmp`
  exists on a part without OP2 and fails its static_assert there.
- adc.hpp, dac.hpp: the `rebase()` and 22-bit corrections as in
  adc.md; two doc comments moved onto the functions they describe
  (`resrdy()`, `set()`); `Dac::init()`'s "call after clock init" (it
  reads no clock) replaced by what it does.

### Facts resolved while writing

- How the AVRxt keeps the kernel's no-nesting promise was stated
  nowhere though kernel/platform.hpp asks every platform to say it: the
  I bit is not cleared on ISR entry nor set by RETI (DS40002247B and
  DS40002443A 7.6.3); every vector is level 0 out of reset
  (15.3.2.4.3), a level-0 handler is interrupted only by the vector
  `CPUINT.LVL1VEC` raises or an NMI (15.3.2.4.2), `STATUS.LVL0EX` holds
  the others off until RETI (15.5.2), and nothing in brio writes
  `LVL1VEC` (grep). platform.hpp's opening and platform.md's "What the
  silicon does" now say so; the README's folded-ISR finding is the one
  way a handler opens the window, cross-referenced.
- An unbound vector: avr-libc's `__bad_interrupt` is a `jmp 0`, read in
  `build-cmake/avr128db48-release/blink.lst`; the pages say the program
  restarts from its first instruction without a hardware reset.
- `-Werror=misspelled-isr` (docs/avrxt/README.md): no such flag is in
  `avrxt/CMakeLists.txt`, but `-Wall -Wextra -Werror` is and
  `-Wmisspelled-isr` is on by default, so a misspelled vector does fail
  the build; the README stands.
- The CCP period holds interrupts off (DS40002247B 7.4.6: "interrupts
  will be ignored for the duration of the configuration change enable
  period", at most four instructions): clock.hpp and clkctrl.md say so
  (a fact the old text did not carry, supported by the data sheet).
- A port, a ticker-free timer or an op amp initialized before the clock
  task runs at the ratio of the reset rate to the claimed one (OSCHF 4
  MHz on the DA/DB, a sixth of 24 MHz; the fuse's rate over 6 on the
  EA, DS40002443A 12.5.2): derived from the divisor arithmetic, not
  measured.

### Open

- The origin of three figures the corrections replaced: twi.hpp's
  "125 ns" tOF, delay.hpp's "some 35 cycles" and "171" (no document
  carries them; the pages' measured figures stand).
- What a level-0 interrupt flag does after `__bad_interrupt`'s `jmp 0`:
  the jump executes no RETI, so `STATUS.LVL0EX` should stay set and the
  restarted program take no further level-0 interrupt (15.5.2); not
  measured, not written.
- `Window::init()` (ac.hpp) casts both comparators' `init()` answers to
  void: for an input pin the package does not bond it still writes the
  window bits and returns true. Stated in its comment; the code is the
  driver owner's call, and it is in no gap list.
- `TcaPwm`'s split-mode endpoint: the header says "CMP = 0 still gives
  one clock of output", DS40002247B 23.3.3.6 and the page's split-mode
  paragraph say CMPn = BOTTOM is a static low; the policy holds either
  way, a measurement settles it.
- `dac.hpp`'s static_assert message "AVR DA/DB have one DAC: DAC0"
  fires on the EA too (a string literal, out of a comments-only pass).
- `nvm_flash.hpp`'s "2048 and 8" for "elsewhere" unverified, kept.
- ticker.hpp's "Written by Uliano Guerrini; rewritten with Claude
  (Anthropic)." attribution line kept; whether it stays is the user's.

### Reader review (fresh subagents)

Round 1, two readers, each given only its files, the audience profile
and the brief's four questions with scenarios: one on platform.md and
usart.md (ten scenarios), one on the comments of clock.hpp and
ticker.hpp (ten). Supported directly: the unbound tick vector and what
it does; no handler nesting and what decides it (CPUINT, not the I
bit); an armed STANDBY taken by every idle; a panic with no debugger
running on past `break_here()`; a port initialized before the clock;
print from an interrupt handler forbidden (one producer), and the
deeper reason inferred correctly from the no-nesting rule; an unbound
DRE vector restarting at the first queued byte; the console's pads
coming with the route; the letter and `z`/`y`; `init()`'s true and
false on each die; the EA's fuse mismatch; `set()` main context only
and its unbounded wait; the masking (four instructions after each CCP
key); the ticker's three lines; the getters from any context; the
busy-flag waits not reported; OSC32K as CLK_RTC and its measured
error; `millis()`'s correction. Residue, applied: usart.md's print
rule read as "a short text works but is forbidden on paper" (now with
print.hpp's reason: what it does is promised by no family); what rate
a port initialized before the clock speaks at, and that the divisor is
right once the clock runs (usart.md, clkctrl.md, twi.md, clock.hpp,
derived from the arithmetic); whether the kernel's tick wakes STANDBY
(platform.md, now with the measurement it rests on); the default
reporter (`HaltReporter`, then `panic()`'s loop with interrupts
masked); `Tenuto` and its template arguments; RAII; "the pair";
"with the same breadcrumb" read as a watchdog writing a record (it
writes none); clock.hpp's "every Clock init writes" MCLKTIMEBASE,
which the EA's fuse-mismatch path does not (corrected), and what
false means for `ClockSource::osc32k`; ticker.hpp's OSC1K path, which
leaves kernel time 32 times slow (now stated), the 32 kHz crystal
passed over in silence when started without its force (now stated,
pointing at `Xosc32k`), "long-term exact" qualified as against the
tick, "the clock task" reworded. Left as is, with the reason: the C++
notation (`using`, `constexpr`, `::`, the digit separator, the tag
object) is the learning route's; the register and oscillator acronyms
(CLK_PER, OSCHF, RUNSTDBY, CCP, ...) are the data sheet's and the pages
cite their sections; the program skeleton (includes, `SysClock`,
`Blinker`) is `blink`'s, which platform.md names; `ISR_FLATTEN`'s
origin (avr-libc's) and the cost table's column notation in usart.md
belong to sections this pass keeps; the wall-clock length of a bounded
poll is not computed anywhere.

Round 2, one fresh reader on the passages round 1 found broken or
silent (usart.md's and platform.md's openings, clock.hpp's opening and
`Clock::init`, ticker.hpp's opening and its two `init()`s), eight
scenarios. Supported directly: the short print before `sei()` (banned,
its outcome deliberately unpromised), the crystal passed over for
OSC32K, OSC1K's kernel time 32 times slow, the tick waking an armed
STANDBY, the default reporter's loop, `Tenuto`. With an inference: the
port before the clock (the rates derived, the scenario itself
impossible to write legally, since a print needs `sei()` and the clock
wants to come before it), the EA fuse mismatch (no MCLKTIMEBASE write,
`hz` untrue). Residue, applied: the priority direction of `Tenuto`'s
list (first is highest, as kernel/tenuto.hpp states); usart.md's "the
rate is right from then on" scoped to an EA whose fuse matched the
claim; clock.hpp's "start Xosc32k here before Ticker::init()" now says
forced on and past `wait_stable()`; ticker.hpp's accuracy line says
"unless init() finds a crystal running". Left as is: "task" read as
"the type an application uses" across the clock and the drivers (the
pages' definition covers the drivers' tasks; the clock's two are the
header's own word for "what an application names"), and the silicon
acronyms. No third round: nothing the second found was broken in
substance.

### Checks

- `bin/brio prose docs/avrxt/*.md docs/avrxt/vendor/README.md
  brio/avrxt/*.hpp`: 46 files, 0 errors, 0 for review (before the
  batch: the same 46 files, 0 errors, 0 for review).
- `bin/brio gate --tokens --against a70d4b2 brio/avrxt/*.hpp`: all
  twenty-five token-identical (lexed with `/sw/gcc/bin/g++` 16.2).
- The removed-token comparison of every touched file against the base
  (every number, backticked identifier, register-like word and document
  number counted before and after): nothing lost but the figures and
  names corrected on purpose, listed above (adc.md's and adc.hpp's
  signed 23, pin.hpp's C++23, delay.hpp's 35 and 171, twi.hpp's 125,
  usart.md's and twi.md's `bench_avr` outside the findings, spi.md's
  driver path now link text) and the lowered emphasis words.
- The "Bench findings" sections (with every subsection under them) and
  the "Not covered yet" sections of all nineteen pages: identical to
  the base.
- Links and anchors of the nineteen pages, README.md and vendor/README.md,
  by a script with GitHub's anchor rules (the kernel.md anchors
  included): 21 files, 0 problems.
- `grep -P '[^\x00-\x7F]'` over the touched files: nothing; no
  `// build:` or `// mcu:` line in any header.
- `git diff --stat a70d4b2 -- . ':!docs/avrxt' ':!brio/avrxt'
  ':!DOCUMENTATION_REWRITE_PROGRESS.md'`: empty.
- The two corrected examples of spi.md were compiled in a scratch
  translation unit with avr-g++ 16.2 for the AVR128DB48 (old forms
  refused, new forms built), and spi.hpp's smallest use likewise; no
  other build, no `brio check`, no host test, no full `brio gate`.

### Local handoff

- The full `brio gate` against a70d4b2 confirms every image unmoved
  (comments only; the token check says so, the gate proves it), and
  `brio check avrxt` for the family fixtures' sake (nothing in them
  changed).
- The corrected "How to use it" examples of twi.md and ac.md (the
  `lend` form and the speed; `BandState`) compiled once in a scratch
  translation unit, as spi.md's were.
- Three claims the desk can settle in minutes: that a port initialized
  before the clock speaks at a sixth of the rate and at the right one
  once the clock runs (derived); that a restart through
  `__bad_interrupt` takes no further level-0 interrupt (`LVL0EX`
  left set, open above); `TcaPwm`'s split-mode endpoint at CMP = 0.
- The patch, `docs_avrxt.patch` at the scratchpad path the brief names,
  is against a70d4b2 and applies with `git apply` on a tree at that
  commit; nothing is committed.

   pages' links; the `ch32x035` stratum is done (the sixth batch), the
   `avrxt` stratum in its local batch.
++ b/brio/avrxt/ac.hpp
 * the AVR EA (DS40002443A ch. 30): what a program uses to learn that an
 * analog input crossed a level, or on which side of a band it sits,
 * without running a conversion, and to hand that level to a pin, an
 * event channel or the CCL. One IP on the three dies; the EA's has two
 * instances, a fifth positive and a fourth negative pin, two power
 * profiles and the window pairing ACn+1 only. docs/avrxt/ac.md is the
 * document.
 * The smallest use, a threshold at 1.000 V on PD6 (AINP3 of every
 * comparator) reported to an active object (a component of the program
 * with its own event queue, served one event at a time by the kernel);
 * Monitor, its event Crossed and the platform P are the program's own:
 *
 *     using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *     using Level = brio::Threshold<brio::Ac<0>>;
 *     ISR(AC0_AC_vect) { brio::post<Monitor>(Crossed{Level::crossed()}); }
 *
 *     int main() {
 *         SysClock::init();
 *         Level::init(brio::AcPos::ainp3, 1000, brio::Ref::v2048);
 *         sei();
 *         brio::Tenuto<P, Monitor>::run();
 *     }
 *
 * Threshold::init() writes the shared reference VREF.ACREF, the DACREF
 * code for 1000 mV, the medium hysteresis, the sense of both edges and
 * the CMP interrupt enable; it returns false, writing no register, for
 * a reference whose millivolts it does not know, a level above the
 * reference or an input pin this package does not bond. The comparator
 * itself needs no rate from the clock task.
 *
 * Who calls what, and from where. init<cfg>(), init(cfg),
 * Threshold::init(), Window::init(), window<partner>() and window_off()
 * are setup verbs: main() before sei(), or an active object's handler
 * that reconfigures; the init verbs read-modify-write PINCTRL and
 * PORTMUX.ACROUTEA, which other drivers share, with no guard, so none
 * of them is for an interrupt handler. cmp(), Threshold::crossed() and
 * Window::changed() are interrupt bodies, called from the program's
 * ISR() for the comparator's vector (Window's: Lower's) and from
 * nowhere else; each clears the CMP flag and reads the state, and an
 * interrupt body runs to completion, no other handler interrupting it.
 * The interrupt bodies write STATUS alone, so enable(), disable(),
 * sense() and enable_interrupt() - unguarded read-modify-writes of
 * CTRLA and INTCTRL - are safe from main and from an active object's
 * handler. state(), flag(), window_state(), Threshold::above(),
 * Window::state() (one load) and dacref(), threshold_mv(),
 * Threshold::set_mv(), clear_flag() (one store) may be called from any
 * context. Nothing here waits, nothing masks interrupts, and no buffer
 * is lent: a new input, reference or DACREF code settles on its own
 * time (below), and a state read before that may be stale.
 *
 *  The resource - Ac<n>, n = 0..2 (0..1 on the EA), a monostate (a
 *  class whose state is static, with no instances): a config struct owns the
 *  config names it (`Ref`, vref.hpp) like the DAC's config does, and an
 *  init() with DACREF as its negative input rewrites it for every
 *  comparator.
 *  The tasks - what an application names:
 *    - INITVAL holds the output meanwhile, and init() does not wait;
 *    the same pin, the partner holds the upper limit on its negative
 *    included, without CLK_PER (32.3.5; the CTRLA field description,
 *    32.5.1, says interrupts and status flags are not updated with the
 *    clock stopped - the bench measured the interrupt waking a standby,
 *    docs/avrxt/ac.md).
    /// Configure A as a threshold on `pin` at `mv` millivolts of `ref`
    /// (`ref_known_mv` gives the millivolts of Ref::vdd or Ref::vrefa),
    /// DACREF as the negative input, then enable the CMP interrupt when
    /// `interrupt`. False, writing no register, when the reference's
    /// millivolts are unknown, `mv` is above them, or A::init() refuses
    /// the configuration. A setup verb (main or an active object).
    /// Interrupt body for A's vector: clears the CMP flag and returns the
    /// state, true = above the threshold.
    /// Configure Upper at `high_mv` and Lower at `low_mv` on the same
    /// `pin`, pair them, and enable Lower's interrupt when `interrupt`.
    /// False, writing no register, when the reference's millivolts are
    /// unknown, `high_mv` is above them or `low_mv` above `high_mv`. The
    /// two comparators' own init() answers are not checked: for an input
    /// pin this package does not bond each writes nothing, the window
    /// and interrupt bits are written all the same, and the answer is
    /// true. A setup verb.
++ b/brio/avrxt/adc.hpp
 * Adc<n>: voltages measured - a sensor on a pin, a differential pair,
 * the supply, the die's temperature, a stream of samples paced by a
 * timer - with the analog-to-digital converter of an AVR DA, DB or EA.
 * A program configures the one converter once with an AdcConfig, names
 * an input, and either reads it (blocking) or lets the RESRDY interrupt
 * hand each result to an active object (a component of a brio program
 * with its own event queue, served one event at a time by the kernel).
 * The smallest use, a blocking reading of PD1 on the DA/DB:
 *
 *     using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *     constexpr SysClock clock;
 *     using Meter = brio::Adc<0>;
 *     using Wire = brio::AnalogIn<brio::Pin<'D', 1>>;            // AIN1
 *
 *     SysClock::init();                                          // first in main()
 *     Meter::init<brio::AdcConfig{.reference = brio::Ref::v2048}>(clock);   // checked at compile time
 *     Meter::select(Wire{});
 *     const uint16_t raw = Meter::read();                        // start, wait, read
 *
 * On the AVR EA (its board's 20 MHz crystal, CLK_ADC 2 MHz by default)
 * the same reading states the sampling an internal reference needs,
 * which init<cfg>() refuses to go without, and claims the pad once:
 *
 *     Meter::init<brio::AdcConfig{.reference = brio::Ref::v2048,
 *                                 .sample_duration = 8}>(clock);   // 4 us x 2 MHz
 *     Meter::claim(Wire{});
 *     Meter::select(Wire{});
 *     const uint16_t raw = Meter::read();
 *
 * Results as events: the program binds the vector and posts what the
 * interrupt body returns (the ISR condenses, the active object decides;
 * Monitor is the program's active object, AdcResult its event), the
 * interrupt enabled by enable_resrdy_interrupt(true) before sei():
 *
 *     ISR(ADC0_RESRDY_vect) { brio::post<Monitor>(AdcResult{Meter::resrdy()}); }
 *
 * Which context calls what. init() (both forms), reconfigure(),
 * disable(), the window and interrupt-enable verbs, start_on() and
 * start_on_events() are setup verbs of the converter's one owner:
 * main() after the clock task, or the handler of the active object that
 * owns the converter. The interrupt-enable verbs are a read-modify-write
 * of INTCTRL with no guard, so they are not called from an interrupt
 * body while the owner configures. resrdy(), wcmp() (the DA/DB),
 * samprdy() and error() (the EA) are interrupt bodies, called from the
 * program's handler for their vector; error() is also the polled form.
 * Nothing in this file masks interrupts. What waits: the DA/DB's init()
 * waits 10 us of warm-up (delay_us on the clock's rate), the EA's waits
 * for nothing; read(), read_sum() (the EA) and flush() (the DA/DB) spin
 * until RESRDY with no time-out, a conversion's length on an enabled
 * converter (on the EA plus the initialization table 31-5 times, up to
 * 60 us with LOWLAT off), and a disabled converter never raises RESRDY;
 * reconfigure() and the DA/DB's rebase() spin on busy() until the
 * converter is idle; the EA's select under an event pace spins the
 * output latency, 36 CLK_PER at most, with interrupts on. One converter, one owner: the window match a
 * result read captures, and on the EA the TEMP registers its 16- and
 * 32-bit registers share, are state for one context at a time. The
 * driver holds no buffer.
 *
 * Adc<n> is the analog-to-digital converter of the stratum as one brio
 * task with knobs (a task: a driver layer with state of its own over a
 * resource, served from its peripheral's interrupt or called by an
 * active object) - see docs/avrxt/adc.md for why not several: every use
 * Ownership of the one converter belongs to one active object.
 * Two converters, two engines, one file. The AVR DA and DB carry a
 * (DS40002247B ch. 33); the AVR EA carries the 12-bit differential ADC
 * selection, and an unbuffered input mux. The two share the surface
 * writes computed on the fly - reconfigure from a console command;
 * false, touching nothing, for what the template form refuses). Both
 * take the clock tag: the prescaler's range (and the DA/DB's warm-up
 * wait) is computed from the rate the clock type claims, so init() comes
 * after the clock task's, and a DynamicClock that does not list this
 * Adc among its users fails to compile at init().
// The AVR DA/DB: the 12-bit SAR ADC (DS40002247B ch. 33)
// the sum, truncated to 16 bits above 16), left_adjust, free_running,
// a reference change takes effect only after one conversion. init()
// that does not. Adc is a ClockUser (a driver a DynamicClock tells of
// every rate change through its rebase(hz)): on a DynamicClock change
// rebase() re-picks the prescaler (stopped, idle, rewritten,
// free-running resumed); the owner pauses event-started conversions
// around the switch. The intent is the CLK_ADC chosen at init; the code
// aims at the one in effect before the switch (clk_adc_hz_ is rewritten
// by every rebase), so a round trip of the clock can leave it slower -
// a gap docs/avrxt/adc.md lists.
// Bench facts (the AVR DB's silicon A5): the converter needs its warm-up (t_ADC_INIT
// conversion after enable (not per start); the unbuffered
// tells about the last result read.
    /// RES holds the sum of the accumulated samples truncated to 16 bits:
    /// Run-time form. Writes the mux (GND) and every knob before enabling
    /// as close as possible to the one in effect before the switch,
    /// within range - init's on the first rebase, the last rebase's
    /// after it (clk_adc_hz_ below is rewritten here; aiming at init's
    /// is the intent, a gap docs/avrxt/adc.md lists).
    /// idle, rewrites, and restarts free-running if it was on. The owner
    /// The result (RES: a sample, or the sum of `accumulate` samples,
    /// waiting on busy() alone reads the previous result), read. The
    /// wait has no time-out: it ends when RESRDY is set, one conversion
    /// on an enabled converter; a disabled converter never sets it.
    /// The thresholds in the differential conversion's format (33.5.16:
    /// Did the last result read by result()/read() match the window?
    /// ISR body for ADCn_RESRDY_vect: the result (flags cleared by the read).
// The AVR EA: the 12-bit differential ADC with PGA (DS40002443A ch. 31)
// The engine, from the chapter:
//  - One COMMAND store is a conversion (31.5.11): DIFF | MODE | START.
//    image with IMMEDIATE ORed in - a load and a store to COMMAND, beside
//    the store that notes the start mode (start_). MODE is the
//    _SCALING forms (table 31-1); DIFF is the input's (an AdcDiff pair
//  - Two outputs (31.3.3.3): SAMPLE, 16 bits, every conversion; RESULT,
//    32 bits, the sum of an accumulation (4095 x 1024 = 4193280, 22
//    bits - adc_result_bits() below; in DIFF -2048 x 1024 at its lowest,
//    a signed 22) or the single conversion. result() reads RESULT[15:0]
//  - The start modes (31.3.3.2): IMMEDIATE (start()), EVENT_TRIGGER
//    (start_on_select(): the select is the start, one store a
//  - The input mux is not buffered (31.3.3.5: "Changing any of these
//    the event pace a select can land on an event-started conversion of
//  - Initialization is timed in hardware, per conversion (table 31-5):
//    from every conversion but the first; it is off by default - the
//  - The reference is ADC.CTRLC's own (31.5.3): VDD 0, VREFA 2, 1.024 V
//  - The PGA (31.3.3.5.1, 31.5.12): gain 1..16, bias 100/75/50/25 %
//    reference, table 31-6); the VIA bits are shared by both muxes, so
//  - Sampling (31.5.9): SAMPDUR + 0.5 CLK_ADC cycles (+1 with the PGA);
//  - The window (31.3.3.8, 31.5.4): WINCM and WINSRC (RESULT[15:0] or
//    vector) and is cleared only by writing 1 (31.5.6): result() and
//  - The flags (31.5.6, table 31-11): RESRDY and SAMPRDY clear when
//  - The temporary registers are shared by RESULT, SAMPLE, WINLT and
// the start mode noted in RAM (a store), the image's load and one store
// to COMMAND; the RESRDY body 14 - one load of
/// TEMPSENSE are positive only. The DAC0 input is the DAC's unbuffered
/// The comparators' DAC references, negative inputs only (31.5.14): the
/// A differential pair as an input value: `AdcDiff<AnalogIn<Pin<'D', 1>>{},
    /// close as possible to the one chosen at init - not to the last
    /// event pace, the select start). The owner pauses an event pace
    /// The digital input buffer of an input's pins turned off, once (the
    /// The wait has no time-out: the conversions and the initialization
    /// table 31-5 times on an enabled converter; a disabled converter
    /// never raises SAMPRDY and the wait never ends.
    /// Did the last result()/sum()/sample() match the window?
    /// signature-row factors signed (TEMPSENSE1 the offset, TEMPSENSE0 the
    /// Under the event pace: STOP with the old input's DIFF, a wait of the
/// A converter whose configuration is a constant of its type: init(clock)
++ b/brio/avrxt/ccl.hpp
 * The configurable custom logic (CCL) of the AVR DA, DB and EA: small
 * look-up tables (LUTs) that compute a logic function of three inputs -
 * pins, event channels, other peripherals' outputs - and drive a pin, an
 * event channel or an interrupt with it, with no CPU, and in sleep as
 * far as the LUT's clock runs there (docs/avrxt/ccl.md): glue logic on
 * pins, a divide-by-two of a timer's event, a one-clock pulse
 * on an edge, a filtered input that wakes the device. The chapter is
 * DS40002247B ch. 31 on the AVR DA/DB (six LUTs and three sequencers on
 * 48/64 pins, four and two on 28/32) and DS40002443A ch. 29 on the AVR
 * EA (four LUTs and two sequencers on every package, the same IP but for
 * the input menu). The header has two strata, as docs/avrxt/ccl.md
 * describes:
 *  the resources - Ccl: the block (enable/disable, RUNSTDBY, the three
 *  LUTn output generator are its event vocabulary (evsys.hpp);
 *  the task - ToggleFlipFlop<pair>: a JK flip-flop on a LUT pair toggled
 * The smallest use, PB3 = PB0 AND PB1 on a 48/64-pin AVR DA/DB (LUT4's
 * port is PORTB):
 *
 *   using And = brio::Lut<4>;
 *   brio::Ccl::disable();
 *   And::init<brio::LutConfig{.in0 = brio::LutInput::pin, .in1 = brio::LutInput::pin,
 *                             .truth = brio::lut_truth([](bool a, bool b, bool) { return a && b; }),
 *                             .output_pin = true}>();
 *   brio::Ccl::enable();
 *
 * Ccl::disable() stops the whole block, the state a configuration starts
 * from; init<cfg>() checks the configuration at compile time, takes the
 * LUT's own pins 0 and 1 as inputs 0 and 1, writes the truth table
 * lut_truth() computes from the lambda (input 2 is `mask`, low) and,
 * with output_pin, makes pin 3 an output carrying the result;
 * Ccl::enable() starts the block. LUT4 exists on the 48/64-pin AVR
 * DA/DB alone; elsewhere a lower LUT does the same on its own port
 * (lut_port()). What decides correctness even there: on the AVR DB's
 * silicon A4/A5 a LUT is reconfigured only with the whole block
 * disabled (errata 2.4.1, below), which the disable/enable bracket does;
 * the driver configures the output pin and nothing else of the port, so
 * the input pins are read as PORT leaves them; and an interrupt sense
 * other than none needs the program's ISR(CCL_CCL_vect) calling
 * Ccl::take_flags().
 *
 * Who calls what, and from where. Nothing here waits or masks
 * interrupts. Ccl::enable() and disable() are one store each; flag()
 * and clear_flag() one load or one store, legal from any context.
 * Ccl::sequencer(), Lut<n>::init(), sense(), enable() and disable() of
 * one LUT, and ToggleFlipFlop::init() read, modify and write registers
 * shared between LUTs (INTCTRL, PORTMUX.CCLROUTEA) or the pins' port,
 * and a Lut keeps a note of the pin it owns: they are setup, in main()
 * or in the handler of the one active object that owns the wiring (an
 * active object is a component of a brio program with its own event
 * queue, served one event at a time by the kernel,
 * docs/learn/introduction.md). Ccl::take_flags() is the interrupt body,
 * called from the program's ISR(CCL_CCL_vect) and from nowhere else.
 *
 *    (init does both: CTRLA = 0 first, then CTRLB, CTRLC, TRUTH while
 *    the LUT is disabled, then CTRLA's own fields together with
 *    ENABLE). On silicon A4/A5 (2.4.1) reconfiguring any LUT wants the
 *    protocol is: Ccl::disable(); Ccl::sequencer<pair>(...) (before the
 *    entries select the instance by input index - input 0/1/2 of a LUT
 *    stable over two clocks and delays four; the edge detector requires
 *  - the sequencer of a pair is clocked by the even LUT's clock, takes
 *    the even LUT as D/J/D/S and the odd as G/K/G/R, and its output is
/// peripheral entries mean depends on which input (0/1/2) they sit on.
    d_flip_flop = CCL_SEQSEL_DFF_gc,    ///< D = even LUT, EN = odd, clocked by the even LUT's clock
    /// Enable the block (after every LUT and sequencer is configured):
    /// one store to CTRLA, RUNSTDBY with it when asked.
    /// even LUT: written while LUT 2p is disabled, before its init()
    /// Compile-time form: lut_config_valid()'s refusals become compile
    /// errors, then the run-time form below does the writes. Setup.
    /// Run-time form. Call with the block disabled (Ccl::disable(); on the
    /// for a port this package lacks are compiled out (a runtime `if`
++ b/brio/avrxt/clock.hpp
 * The clock of a program on an AVR DA, DB or EA: what sets the rate the
 * CPU and the peripherals run at (CLK_PER, the same rate as CLK_CPU on
 * these parts), and the one number every clocked driver of this stratum
 * computes its divisors from. A program names its clock once, as a type;
 * calls that type's init() as the first line of main(); and hands the
 * type's empty tag object to every driver's init, which reads the rate
 * from the type - never from the hardware, never from F_CPU:
 *     using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *     constexpr SysClock clock;          // the tag a driver's init(clock) takes
 *
 *     int main() {
 *         SysClock::init();              // first: CLK_PER = 24 MHz, the PA0/PA1 crystal
 *         ...                            // each driver's init(clock), then sei()
 *     }
 *
 * SysClock::hz is the constant 24'000'000 whether or not init() has run;
 * init() is what makes it true. It returns true when the requested
 * source runs, and false when that source did not start and the
 * fallback runs instead: OSCHF at the same CLK_PER for a high-frequency
 * source, so hz stays true to OSCHF's accuracy; OSC32K for the 32 kHz
 * crystal. For ClockSource::osc32k, false means the switch to OSC32K did
 * not complete in its bounded wait. On the AVR EA a false can also mean that the OSCCFG fuse does
 * not run OSCHF at the rate the type claims; then init() writes nothing,
 * the part stays at its reset clock and hz is not true (Clock::init
 * below).
 *
 * The chapter is the clock controller, CLKCTRL, of the AVR DB
 * (DS40002247B ch. 12), the AVR DA (DS40002183A ch. 11) and the AVR EA
 * (DS40002443A ch. 12), in two strata (docs/avrxt/clkctrl.md is the
 * document, docs/design/clock.md the model of one rate truth):
 *
 *  the resources - one monostate (a class whose state is static, with no
 *  instances) per block of the controller, each a thin typed view of its
 *  registers (the control registers written through CCP, the status
 *  bits read):
 *                 20/24 MHz by FRQSEL, on the EA one rate the OSCCFG fuse
 *                 every Clock init that passes the fuse check and every
 *                 DynamicClock switch
 *  the tasks - what an application names:
 *    Clock<source, source_hz, div, oschf_hz>   the static main clock: one
 *                 constexpr truth `hz` every driver derives from; init()
 *                 composes the resources and reports whether the
 *                 requested source runs
 * Who calls what, and from where.
 *  - Clock::init() (and DynamicClock::init(), which runs Boot's) is
 *    setup: the first line of main(), before every driver's init(clock)
 *    and before sei(). Nothing checks that order: a driver initialized
 *    first computes its divisors for hz while the part still runs at its
 *    reset clock (OSCHF at 4 MHz on the DA/DB, the fuse's rate over 6 on
 *    the EA), so it runs at the ratio of the two rates until init()
 *    brings CLK_PER to hz, and at the right rate from then on (its
 *    divisor was computed for hz). init() masks nothing itself; each protected write holds
 *    interrupts off for the four instructions after the CCP key, which
 *    the hardware does (DS40002247B 7.4.6, DS40002443A 7.4.6). Its waits
 *    are bounded polls: OSCHF's status and each source switch (0xFFFF
 *    polls), XOSCHF's start-up for a crystal or a clock
 *    (Xoschf::wait_stable(), 0xFFFF polls), on the DA the external clock
 *    asked of TCD0 (extclk_seen() below), and for the 32 kHz crystal
 *    Xosc32k::wait_stable()'s 0x00FFFFFF polls. None hangs; an external
 *    source's wait that runs out is the false above, while the answer of
 *    the wait for OSCHF itself is not looked at.
 *  - DynamicClock::set<hz>() and set(hz) belong to the main context -
 *    main(), or the handler of an active object (a component of a brio
 *    program with its own event queue, served one event at a time by the
 *    kernel) - never to an interrupt handler: each listed user's
 *    rebase() runs first and may wait (a Uart's waits, with no bound,
 *    until its transmit side is idle, avrxt/usart.hpp).
 *  - hz, hz(), rate_index() and matches_fuse() read a constant, a
 *    variable or a fuse: any context.
 *  - The resources' verbs are setup verbs with no guard of their own;
 *    several read, modify and write a register (Oschf::run_standby(),
 *    autotune(), Xosc32k::force_on(), MainClock::select(), clkout(),
 *    ClockFailure::test()), so they belong to one context at a time.
 *    ClockFailure::cfd() is an interrupt body: the program's
 *    ISR(CLKCTRL_CFD_vect) (or its NMI handler) calls it and nothing else
 *    does. Nothing here owns a buffer.
 *
 *  - internal sources start when requested; crystals must be enabled
 *    before it is selected;
 *  - MCLKSTATUS follows the request: OSCHFS reads 0 while OSCHF is not
 *  - after any reset CLK_MAIN is OSCHF at the fuse's rate behind the
 *  - OSCHFCTRLA resets to 0x0C with bits 3:2 undocumented (12.5.8; the
 *    bench AVR64EA48 reads 0x00 at reset, docs/avrxt/clkctrl.md): it is
 * Xosc32k here before Ticker::init() when a 32k crystal is fitted,
 * forced on (start_crystal()'s run_standby true) and past
 * Xosc32k::wait_stable(): a crystal nothing requests reads as not
 * running otherwise, and Ticker::init() takes OSC32K in its place.
/// the main clock. EXTS is set only while EXTCLK is requested (DS40002183A
    /// and the copy lands after the protected write: measured on an
/// Enable alone does not run it. With RUNSTDBY clear the oscillator runs
/// only while something requests it - the RTC, the main clock, the PLL -
/// a second). So a program that wants to know the crystal runs before
/// Errata 2.5.4 as code (DB A4/A5): with SOURCE = XOSCHF the PLL runs
/// only if XOSCHF is configured for an external clock (SELHF = 1) - it
/// therefore refuses that combination and writes nothing: a bench
/// header), so SOURCE = 1 there is the external clock and the check
/// discrete set behind the runtime regime: a DynamicClock can only ever
/// index into this list - which is what lets avrxt/delay.hpp expand
/// source must be running (and an external one stable) before it is
/// one microsecond or more, rounded up (DS40002443A 12.3.6, 12.5.7: a
/// CLK_PER, so the clock owns it: every Clock init writes it, every
/// optionally an interrupt (regular or NMI), and - when the main clock is
/// do: a brio app posts to a supervising active object; the kernel keeps running
///    claim about the fuse - 16 or 20 MHz - which the type cannot check
    /// claim - and then nothing is written: the part keeps its reset clock
    /// main(), before any driver's init(clock) and before sei(): a setup
    /// verb that masks nothing and waits in bounded polls only (the
    /// opening comment lists them).
 * its rate; set<hz>() / set(hz) name the new rate (the app speaks Hz -
 * order, synchronously, and then reprogram the main prescaler - so a
 * the new one. Call set() from the main context, and only when nothing
 * that depends on the rate is mid-transfer (a driver's rebase() may
 * wait for its own hardware to go idle; a bus transaction in flight is
 * the caller's problem - in a program of active objects, ask the
 * active objects that own the buses before switching). F_CPU must not
 * be defined for this type: no rate is fixed any more.
    /// The discrete-rate surface (what avrxt/delay.hpp dispatches on).
    /// whole set is known from the type, so per-rate arithmetic can be
    /// runtime fact is which index is current - a byte, not a division.
    /// it has in flight at the old rate), then switch the prescaler.
++ b/brio/avrxt/dac.hpp
 * Dac<n>: a voltage out of the chip - a bias, a test level, a slow
 * waveform, or a level fed inside the chip to the ADC or a comparator.
 * The 10-bit DAC of the AVR DA/DB (DS40002247B ch. 34) and of the AVR
 * EA (DS40002443A ch. 32) - one IP - as a brio actuator of the rank of
 * Pin: synchronous, set-and-forget, no events, no interrupt. One
 * instance (DAC0); the template parameter is there for the symmetry
 * every other driver has and for targets with several. init() selects
 * the reference and configures the output, set() writes a code and the
 * output follows:
 * Which context calls what. init(), disable() and enable() are setup
 * verbs of the one owner (main(), or the active object that owns the
 * converter): register writes that return at once - init() writes
 * VREF.DAC0REF, PD6's pin control, DATA and CTRLA, disable() and
 * enable() rewrite CTRLA (a read-modify-write on the DA/DB) - and none
 * takes the clock, waits or masks interrupts. set() is one 16-bit store
 * and code() one 16-bit load, from the owner's context. Nothing waits
 * for the reference's start-up or the buffer's settling: they follow in
 * the analog domain, and nothing checks that VDD has the headroom the
 * reference needs (docs/avrxt/vref.md).
 *
 * full scale - the DB data sheet's figures (39.19); the EA's table 35-22
 * gives no drive figure and its buffer was measured sinking milliamperes
 * (docs/avrxt/dac.md); the unbuffered output feeds the ADC (MUXPOS DAC0) and
 * Bench facts (the AVR DB's A5 silicon, bare pin; the AVR DA's and the
 * AVR EA's are in docs/avrxt/dac.md): the buffered output rises to a
 * new code within ~10-20 us but falls at the sink limit - about 1 uA
    /// output is code 0. A setup verb that takes no clock and reads none;
    /// it returns at once, and the reference start-up (10-200 us on the
    /// DA/DB, 130 us typ. and 180 max on the EA) and the buffer settling
    /// follow in the analog domain. With `output_pin`, PD6's digital
    /// input buffer is turned off and the pad is the DAC's.
    /// Output code 0..1023 (clamped). One 16-bit write, DATAL then
    /// DATAH: the output updates on the high byte.
++ b/brio/avrxt/delay.hpp
 * A short busy-wait for a driver or for a program's setup code:
 * delay_us(clock, us) spins for at least `us` microseconds at the rate
 * the program's clock type states (avrxt/clock.hpp), never at F_CPU -
 * the AVR implementation of the short-wait role. The smallest use, with
 * the clock tag every driver's init takes:
 *
 *     using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *     constexpr SysClock clock;
 *     ...
 *     brio::delay_us(clock, 10);     // at least 10 us; folded to an exact cycle loop
 * double-digit microseconds. What it is not for: waiting in an active
 * object (a component of a brio program with its own event queue, served
 * one event at a time by the kernel). Anything long enough to be
 * measured in milliseconds is a time event (kernel/time_event.hpp): the
 * loop keeps serving the other active objects and the CPU may sleep.
 * Only pre-kernel init code (before Tenuto::run) may legitimately wait
 * milliseconds here.
 *
 * Who may call it: any context. Every verb here touches no register,
 * masks nothing and owns nothing; it reads the clock type's constant, or
 * a DynamicClock's current rate index once at the call. An interrupt
 * taken during the wait lengthens it, and "at least" still holds. A wait
 * inside an interrupt handler holds every other handler off for its
 * whole length, since a handler runs to completion with no other handler
 * interrupting it. The wait is SysClock::hz's: called before
 * SysClock::init() has brought CLK_PER there, it lasts what the reset
 * clock makes of the count.
 * No division ever runs at wait time. The rate is compile-time
 * prescalers, so its whole rate set is in the type too and the one
 * runtime fact is which rate is current - an index byte. delay_us
 *  - `us` a runtime value: the branch selects a per-rate fixed-point
 *    loop. The factor is rounded up at compile time and the product is
 *    "at least" holds by the two roundings, not by a measured constant.
 * which makes the microsecond arithmetic exact at every rate the
 * time: one loop turn is 4 cycles, so at 32.768 kHz it is 122 us and
 * cache or wait states, so counting cycles is timing. On a Cortex-M or
/// 2^12). The compile-time half of the fixed-point path; rounded up so
/// always_inline on purpose: with a compile-time mult (a static clock's
/// The same wait as one shared out-of-line function: what the dynamic
///    turns are its quarter rounded up, and the wait is _delay_loop_2's
///    ceiling, and the whole path is 39 cycles for 1 us at 24 MHz,
///    counted in the SPI host's listing (38 measured), where the
///    fixed-point paths measure 122 cycles over the nominal at a static
///    rate and 157..160 under a dynamic clock (docs/avrxt/platform.md,
///    "Bench findings"; the tail as this function reaches it, its
///    factor computed at run time, is not measured there). The test on
///    `us` folds away where the caller hands a byte, as the SPI host
///    does.
///    more would overflow that factor; it saturates instead (a longer
///    wait - the safe side), and no AVR DA, DB or EA clock gets there.
///
/// Any context: no register, no mask, no state.
/// Busy-wait at least `us` microseconds at a compile-time rate: the
/// branch contributes - into the shared out-of-line tail when the call
/// discrete rate set, each branch a delay_us_at of its own compile-time
/// proven by the static path) and a hit returns: the current rate's
/// rate index into per-rate branches expanded from the clock's discrete
/// `clock` is the program's empty tag object; any context (the opening
/// comment), the wait lengthened by every interrupt taken during it.
++ b/brio/avrxt/device_tables.hpp
 * What the part an image is compiled for has: the table of what each
 * AVR DA, DB and EA part has, read off the device header, which the
 * stratum's documents call the reserve. A driver of this stratum, or a
 * program that adapts to its part, asks it a question and gets a
 * compile-time answer:
 *
 *     static_assert(brio::port_exists('E'), "this board needs PORTE");
 *     if constexpr (brio::has_xoschf) { ... }        // a crystal can drive PA0/PA1
 *     if constexpr (brio::avr_die == brio::AvrDie::avr128da) { ... }   // a DA erratum
 *
 * Every entry is a constexpr value or a constexpr function, so there is
 * nothing to call at run time and no context to respect. A part the
 * table does not list is a compile error (the #error below), which is
 * how the stratum's claim of the parts it builds for is checked.
 *
 * It is the one file of the avrxt stratum where the preprocessor is
 * allowed to ask the device header what exists. The standing rule
 * a value lives here and is exported as constexpr data; a driver keeps
 * `#ifdef` only to select per-instance code (a register-struct
 * Three kinds of entry, each asked of the thing that is the answer:
 *  - Features and instances. Each a probe of the header symbol that is
 *    the header names a feature by an enumerator rather than a macro (an
 *    the enum type; where it names it by a struct member (an event
 *    feature is never borrowed to mean a family: "has MVIO" is a fact
 *  - The part: its die and its package. What only a die can key - the
 *  - The errata and the family facts no header carries, as values keyed
 * instance, from the header's `_gm`/`_gp`): their width and which of a
/// a pin missing from an existing port (PF4 on a 28-pin part) is a
/// this to keep an instance usable when only one of its pin positions
/// without it takes an external clock on PA0 directly; a program
/// rather than a hard error: a requires-expression over a non-dependent
// The route codes each instance's field takes, per layout. Each table is
// packages that do). Whether this package bonds a route's pins is the
/// The TCB's CCMPL and CCMPH as two independent 8-bit registers in PWM8
++ b/brio/avrxt/evsys.hpp
 * The event system (EVSYS) of the AVR DA, DB and EA: hardware wiring
 * between peripherals, with no CPU and in every sleep mode - a pin's
 * level into a timer's capture input, the PIT's divided clock starting
 * the ADC, any internal signal mirrored on a pin. An event here is the
 * data sheet's hardware event, a signal on an internal wire, not a
 * kernel event; the bridge between the two is an interrupt body that
 * posts. The chapter is DS40002247B ch. 16 on the AVR DA/DB and
 * DS40002443A ch. 16 on the AVR EA; docs/avrxt/evsys.md holds the
 * analysis and the model:
 *  - a generator is a type: its CHANNELn code and the channels it may
 *    generator that is a slot of another peripheral (the AVR EA's pin
 *  - a user is a type that knows its USERxxx register (EvOut<Pin<'D',2>>,
 *  - a channel is a resource handle, EventChannel<n>.
 * The smallest use, the PIT's 512 Hz on channel 1 and mirrored on PD2
 * (EVOUTD), then torn down:
 * source() writes the generator's code into CHANNEL1; listen() selects
 * the EVOUT position in PORTMUX.EVSYSROUTEA, makes PD2 an output and
 * writes the channel into the user's register; unlisten() undoes all
 * three; off() idles the channel; pulse() inverts the channel's signal
 * for one CLK_PER cycle. What decides correctness even there: a PIT
 * divider carries a signal only while the RTC's prescaler chain runs,
 * which in a kernel program the Ticker's init() has seen to by enabling
 * the PIT (and on the AVR EA a PIT slot needs PITEN itself, measured,
 * docs/avrxt/evsys.md); nothing here starts the RTC. A pin generator
 * reads zero while the pin's input driver is disabled.
 *
 * Who calls what, and from where. These run-time primitives are the
 * layer that is the peripheral, and none of them waits or masks
 * interrupts. source() of an ordinary generator, off(), pulse() and a
 * plain user's listen() and unlisten() are one register store each,
 * legal from any context. source() of a slotted generator (the AVR EA's
 * EvPin and EvPitDiv) first reads, modifies and writes the slot's
 * register, and EvOut's listen() and unlisten() read, modify and write
 * PORTMUX.EVSYSROUTEA, a register every port's EVOUT shares: those
 * belong to one owner - setup in main(), or the handler of the active
 * object that owns the route (an active object being a component of a
 * brio program with its own event queue, served one event at a time by
 * the kernel, docs/learn/introduction.md). A rewire is an action of a
 * state: Entry routes, Exit disconnects.
 *
 * The compiler checks legality (this generator on this channel, this
 * pin as an EVOUT); it does not check exclusivity of a channel that
 * several states rewire - that is ownership, the business of one active
 * object's state machine. The tables below grow on demand: a missing
 * is not built: sugar over these primitives, docs/avrxt/evsys.md lists
 * it among its driver gaps.
 * The two models. On the AVR DA/DB every pin and eight fixed PIT
 * parities (16.5.2). On the AVR EA (16.4, 16.5.2) six channels take every
 * generator, but pins and PIT dividers reach them through slots: two per
 * Facts worth remembering (16.3.2): a pin generator is the pin level
 * device tables' too (avrxt/device_tables.hpp, the reserve: the table of
 * what each part has, read off the device header).
/// the slot carries, and EventChannel::source() calls it before writing
/// Software event on a channel chosen at run time (for drivers that
    /// One store into CHANNELn, preceded for a slotted generator by its
    /// prepare() (a read-modify-write of the slot's register: one owner).
/// AVR DA/DB: eight fixed generators, 8192/4096/2048/1024 on even channels
/// only, 512/256/128/64 on odd channels only (the two families share
/// AVR EA: two slots in the RTC, each carrying one divider 4..32768
/// ZCD n zero-cross output level (async). All channels. 0x30 + n.
/// USART n XCK level (sync host mode). All channels. 0x60 + n.
/// SPI n SCK level (host mode). All channels. 0x68 + n.
/// CCL LUT n output level (async). 0x10 + n.
/// Analog comparator n output level (async). 0x20 + n.
/// A port pin's level as an event (async; zero if the input driver is
/// AVR EA: a port has two event slots (PORTx.EVGENCTRLA, DS40002443A
/// 18.5.17), each carrying one of its pins, on any channel; the channel
/// (PORTMUX.EVSYSROUTEA): Px2 in the default position on every port
/// present (PA2, PB2, PC2, PD2, PE2, PF2, PG2); Px7 (ALT1) on every
/// port but PORTF on the AVR DA/DB (PA7, PB7, PC7, PD7, PE7, PG7), on
/// PORTA, PORTC and PORTD on the AVR EA (PA7, PC7, PD7; its PF7 refused
/// below), and on an AVR DA not PB7 or PE7 (DS80000882C 2.6.1) - the
/// static assertions enforce exactly that. listen() selects the pin
/// position, drives the pin as output, connects. Any generator becomes a signal on a pin: the
    /// Disconnect and tear down what listen() set up: the pin back to
/// ENABLEn and DISABLEn are edge-detected, DUMPn and DRIVEn are levels
/// (table 35-2): a channel driving a dump or a drive must hold its
++ b/brio/avrxt/nvm.hpp
 * NVMCTRL (chapter 11): what a program uses to keep data across a
 * reset, a power cut or a reflash - a setting in the EEPROM, a block in
 * the flash, the board's label in the User Row - and the one peripheral
 * that changes the program itself. Flash, EEPROM, the User Row, the
 * Signature Row and the fuses all sit behind it, and every one of them
 * has a different granularity, a different command and a different way
 * of going wrong. The header holds Nvm, the controller (a monostate: a
 * class whose state is static, with no instances; one function per
 * operation, each erase or write checking its address before it
 * touches the controller, the plain-load reads checking none), Sigrow, the read-only Signature Row, and EepromStore,
 * the EEPROM as the util/nv_record.hpp store that the typed record
 * (NvRecord), the interrupt-paced writer active object
 * (util/nv_writer.hpp) and the persistent panic record
 * (util/persistent_panic.hpp) run on - the way most programs reach this
 * driver. avrxt/nvm_flash.hpp presents the AVR DA/DB's flash to the
 * flash block allocator over the same verbs.
 * The smallest use, one EEPROM byte stored and read back, the same
 * lines on every part:
 *
 *     brio::Nvm::eeprom_erase_write(16, 0x5A);   // starts the operation, returns
 *     brio::Nvm::wait_eeprom();                  // a bounded poll until it is done
 *     brio::Nvm::clear_command();                // release the command, idle now
 *     const uint8_t v = brio::Nvm::eeprom_read(16);
 *
 * Who calls what, and from where. Every verb runs in its caller's
 * context, polls where it waits, and masks no interrupt. The verbs that
 * start an operation, wait for one or write CTRLA or CTRLB belong to
 * setup in main() or to an active object's handler (an active object is
 * a component of a brio program with its own event queue, served one
 * event at a time by the kernel): nothing here guards the controller
 * against two contexts, so one context owns the operations and an
 * interrupt handler starts none. eeready() is the one interrupt body,
 * called from the handler the program binds to the NVMCTRL vector
 * (ISR(NVMCTRL_EE_vect) on the AVR DA/DB, ISR(NVMCTRL_EEREADY_vect) on
 * the AVR EA) and nowhere else. The flag reads and the array reads are
 * single accesses, legal from any context - but on the AVR EA an EEPROM
 * read while an operation runs holds the CPU, and every interrupt with
 * it, until the operation ends (the version 3 notes below).
 *
 * What waits, and for how long. Every wait here is a poll of at most
 * wait_limit (4'000'000) turns that answers false when the controller
 * never went idle: a stuck controller, not a slow one. On the AVR DA/DB
 * a flash erase or write halts the CPU for the whole operation,
 * interrupts included - a page erase about 10 ms, a word about 83 us
 * measured on an AVR128DB48 (docs/avrxt/nvm.md) - and the flash and User
 * Row verbs return when it is over, the command cleared; the EEPROM byte
 * verbs (eeprom_write, eeprom_erase_write, eeprom_erase) return as soon
 * as the operation has started, its command still selected, so the
 * caller waits (wait_eeprom) and then calls clear_command(). On the AVR
 * EA every verb that starts an operation returns once it has started,
 * 2.6 to 5.4 ms before it ends, and the command is cleared only after
 * it. On both, eeprom_write_block() blocks for the whole run. No buffer
 * outlives a call: every verb has read its source before it returns.
 *
 * Two generations of the controller, two engines, one file. The AVR DA
 * selected in CTRLA and carried out by a store, the Flash is written a
 * NVM version 3 (DS40002443A ch. 11): stores load a page buffer and a
 * (STATUS.FLBUSY, NVMCTRL_FLBUSY_bm, is version 3's own bit).
 * What the AVR EA keeps: the EEPROM. Its flash and User Row
 * self-programming are declined - a 64 KB part with a 512-byte EEPROM
 * NVM version 2 - the AVR DA and DB (DS40002247B chapter 11).
 * Three memories, three jobs (the division this driver is built around):
 *  - USERROW is 32 bytes that survive a chip erase always, fuse or no
 * How the flash is reached, and why. The Flash is visible twice: as
 * code space through LPM/SPM with RAMPZ:Z, and as a 32 KB window in
 * data space whose position is chosen by NVMCTRL.CTRLB.FLMAP. The
 * driver's flash verbs use the code space only - ELPM to read, SPM to
 * write, never an LD or ST through the window - and none of them
 * writes FLMAP (set_flmap() below is the one verb that does, for a
 * caller's own reads). Two reasons, and both are load-bearing:
 *      section is mirrored into every mapped section and a write that
 *      With no verb writing through the window, the whole item is
 *      inapplicable here rather than something to remember.
 * want the window for its own reads (that half of the DA erratum is
 * The errata that are code here. DS80000915F 2.7.1 / DS80000882C 2.7.2,
 * both families: a multi-page erase checks only the first page of the
 * is no work-around in silicon, so the driver validates the whole
 * is the one worth remembering: the EEPROM on this family has no write
 * What is deliberately not here. CHER (chip erase) and EECHER (EEPROM
 * Command discipline (11.3.2.3). CTRLA is CCP-protected with the SPM
 * selected - selecting a new command over a live one raises an error in
 * STATUS.ERROR instead of doing anything. Every verb here follows the
 * chapter's sequence: wait for the busy flags, clear the command,
 * select the new one, do the store - and the flash, User Row and
 * eeprom_write_block() verbs then wait and clear the command again.
 * The EEPROM byte verbs (eeprom_write, eeprom_erase_write,
 * eeprom_erase) return with the operation running and its command
 * still selected; the caller clears it once the EEPROM is idle
 * (EepromStore::finish(), which the record and the writer call there).
 * CTRLB's other half is CCP-protected too, with the IOREG key, except
 * FLMAP[1:0], which the chapter says explicitly is not (11.5.2).
 * Who stalls whom (table 11-1). A Flash erase or write halts the CPU
 * call. An EEPROM write does not halt the CPU; the CPU is halted only
    /// The same bounded wait for EEBUSY alone: true once the EEPROM
    /// operation a verb started is over, false when it never ended. The
    /// CPU spins in the caller's context meanwhile.
    /// Release the selected command (11.3.2.3 step 6). The flash, User
    /// Row and eeprom_write_block() verbs do it on their way out; the
    /// EEPROM byte verbs leave their command selected while the
    /// operation runs, and the caller clears it here once the EEPROM is
    /// idle. It is public too because a caller that armed a run of
    /// writes through the store model closes it explicitly.
    /// One byte of the EEPROM at `offset`, a plain load from any
    /// context. The offset is not checked: the caller keeps it under
    /// eeprom_size.
    /// Write one byte without erasing it first (EEWR, 11.3.2.3.4). Only
    /// zeros. Returns as soon as the operation has started - the CPU
    /// instead of the 70 us of a bare write. True when the operation has
    /// started: it returns at once, the command still selected (wait
    /// with wait_eeprom(), then clear_command()); false, nothing
    /// started, for an offset outside the EEPROM or a controller that
    /// never went idle.
 * NVM version 3 - the AVR EA (DS40002443A chapter 11; errata
 * DS80001048D). The EEPROM engine.
 * The memory. 512 bytes in 64 pages of 8 (table 8-3), mapped in data
 * byte, any subset of one page in one operation (11.3.1.2, table 11-4):
 * The command model, as the silicon runs it (rev B1, measured by
 * test_avr_nvm's letter b). The chapter states it two ways - "fill the page
 *  - a store into the EEPROM's range loads its page buffer (ANDed with
 *  - writing a page command to CTRLA starts it at once, on the page of
 *    the last load, over the bytes loaded; the buffer clears when it
 *    in two pages land in one page, the last load's, at the other's
 *    offsets. A run is page-local: eeprom_run() refuses one that is
 *    not;
 *  - CTRLA written while an operation runs is refused with
 *    itself into the next operation, so every operation here starts with
 * One operation, its cost. Wait until idle, EEPBCLR, one store per
 * shrink with a faster clock. The EEPROM's unit of cost is the page:
 * Who stalls whom. An EEPROM operation leaves the CPU running
 * on time - but any EEPROM read or buffer load meanwhile is a bus wait
 * The errata as code.
    /// True when the operation has started: it returns at once, EEPERW
    /// left in CTRLA (wait with wait_eeprom(), then clear_command());
    /// false, nothing started, for an offset outside the EEPROM, EEWP
    /// set or a controller that never went idle.
++ b/brio/avrxt/nvm_flash.hpp
 * that lets the target-independent block allocator, NvHeap, keep blocks
 * of flash that survive a reset and a page-selective reflash - a
 * calibration table, a font - in the flash the image does not occupy.
 * NvmFlash is a monostate (static functions, no instances: there is one
 * flash); a program names it as the heap's medium and calls the heap,
 * never these verbs. On the AVR EA this header compiles to a refusal
 * (below).
 * The smallest use, a table found again or stored once (util/nv_heap.hpp
 * has the verbs; docs/avrxt/nvm.md, "NvmFlash - this flash as a block
 * store", has the whole example):
 *
 *     brio::NvHeap<brio::NvmFlash> heap;     // RAM only until it is mounted
 *     const auto& r = heap.mount();          // read-only: no cycle spent
 *     if (!r.mounted()) { ... }              // the image has grown over the map home
 *     if (const auto block = heap.find(cal_id)) {
 *         block->read(0, std::span<uint8_t>(table, block->length));
 *     }
 *
 * What it needs and what it costs. The fuses must give the image a
 * section above BOOT to write into: under the shipping BOOTSIZE = 0 the
 * whole flash is BOOT, no software can write any of it, and the middle
 * zone below is empty. program() and erase() halt the CPU for the whole
 * operation, interrupts included - about 83 us a word and about 10 ms a
 * page erase, measured (docs/avrxt/nvm.md) - so they belong to setup in
 * main() or to an active object's handler that accepts that stall,
 * never to an interrupt handler; read() and zones() are reads, legal
 * from any context. Every verb returns when its operation is over, and
 * none keeps the caller's buffer.
 *
 * The granularities are not the same number. An erase takes down a
 * 512-byte page; a program writes one 16-bit word, and a word may only
 * Addresses are 24-bit and go through ELPM/SPM, never through the
 * the reason the DA's FLMAP-vs-protection erratum (DS80000882C 2.7.1)
 * is inapplicable here: no verb writes through the window. Every verb
 * below is one call into that driver, which keeps its own validation
 * (alignment, device bounds, the section geometry from the fuses, the
 * protection bits, the whole-range check the multi-page-erase erratum
 * makes necessary).
 * The zones, and where their bounds come from. gcc puts the code and the
 * linked image is not one remainder at the top but two bands, and both
 *           floor    __data_load_end rounded up to a page, raised to the
 *                    BOOT boundary - SPM cannot write inside BOOT
 * The symbols are the load addresses, which on this toolchain are real
 * flash addresses; the run-time twins of the same data (__rodata_start
 * The build id is a link-time constant, not a compile-time one:
 * reflashing an unchanged one). It is read as a 32-bit immediate (the
 * four relocation bytes of the symbol's value), because the value is a
 * number and not an address - taking &symbol would truncate it to the
 * 16 bits a pointer has here. It is recorded in every map version and
 * nothing decides on it: a block's validity is its checksum's business.
/// The build id, the newest source timestamp as an epoch, injected by
/// CMakeLists.txt's avr_add_app(). Declared as an array so that no code
/// can be tempted to dereference it: it is a value, and only its
/// relocation bytes are ever read.
    /// The build id (the newest source timestamp, not the time of the
    /// link), as four relocation bytes. Diagnostic only.
++ b/brio/avrxt/opamp.hpp
 * ch. 35): what a program uses to buffer a pad or the DAC, amplify by
 * one of the resistor ladder's exact gains, invert about VDD/2, take a
 * difference with the three-op-amp instrumentation amplifier, or start
 * and stop an op amp from the event system. docs/avrxt/opamp.md is the
 * document. The AVR DB only: the DA and the EA have no such peripheral,
 * so this whole header is gated on the device header's `OPAMP` symbol
 * and `opamp_count` (avrxt/device_tables.hpp) is 0 there. Include it
 * anywhere; use it inside `#ifdef OPAMP`.
 * The smallest use, a unity-gain buffer from PD1 to PD2:
 *
 *     using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *     constexpr SysClock clock;
 *     using Buf = brio::OpampFollower<brio::Opamp<0>>;
 *
 *     int main() {
 *         SysClock::init();
 *         brio::OpampSystem::init(clock);   // TIMEBASE for 24 MHz, then the block's ENABLE
 *         Buf::init();                      // PD1 in, PD2 out, ALWAYSON
 *         Buf::wait_settled();              // warm-up + 127 us of settle time
 *         ...
 *     }
 *
 * OpampSystem::init() computes TIMEBASE from the rate the clock type
 * claims, not from the hardware, so it comes after the clock task's
 * init(); a DynamicClock must list OpampSystem among its users, which
 * its static_assert checks. Buf::init() returns false, writing no
 * register, for a configuration this instance or package cannot make.
 *
 * Who calls what, and from where. OpampSystem::init(), the Opamp<n>
 * init forms, release() and the tasks' init() and release() are setup
 * verbs: main(), after the clock task, or an active object's handler
 * (a component of the program with its own event queue, served one
 * event at a time by the kernel) that reconfigures. The peripheral has
 * no interrupt, so nothing here is an interrupt body. OpampSystem's
 * rebase() is the DynamicClock's to call, from its set(), before the
 * rate changes. The run-time verbs (enable(), mode(), restart(),
 * events(), output(), positive(), wiper(), set(), the *_on() event
 * bindings) are unguarded read-modify-writes of one op amp's registers,
 * for main and active objects; settled(), cal() and the register
 * readers are single loads, legal from any context. wait_settled() is
 * the one wait: it spins on STATUS.SETTLED with no bound, never
 * returning while the op amp is not running (disabled, in the event
 * regime before its ENABLE event, or the block's ENABLE off), and an
 * active object that calls it holds the kernel for the warm-up plus the
 * settle time. Nothing masks interrupts and no buffer is lent.
 *
 *  The resources - OpampSystem: the block every op amp shares (the one
 *  the READY event generator, the offset calibration byte. Both are
 *  monostates: classes whose state is static, with no instances.
 *  The tasks - what an application names (each takes the Opamp<n> it
 *  runs on: OpampPga<Opamp<0>>):
 *  The integrator (figure 35-6) is deliberately not a task: it needs an
 *  - the DAC entries mean the buffered DAC output: the DAC's OUTEN must
 *    be on (34.3.2.3) and PD6 is then the DAC's; nothing here checks it;
 *    The settle timer restarts on every write to OPnCTRLA, OPnINMUX or
 *    generated in the EVENTEN-alone regime only: with both bits set the
 *    bench saw SETTLED rise and no READY event (35.3.2.6;
 *    docs/avrxt/opamp.md, "Bench findings");
 *    DISABLEn are edge-detected, DUMPn and DRIVEn are levels - a DUMP
 *    or DRIVE channel must hold its level, a pulse does nothing;
 *    positive, one step = 0.5 mV (39-27) - on the bench die one step
 *    measured 594 uV and a rising CAL moved the output down
 *    (docs/avrxt/opamp.md, "Bench findings");
 *    2.8.2 makes that bit read-only on silicon A4 - the verb therefore
    software_with_events,   ///< ALWAYSON = 1, EVENTEN = 1: on, DUMPn/DRIVEn heard, no READYn (measured)
    /// Write TIMEBASE for the rate the clock type claims, the debug and
    /// power bits, and enable the block: a setup verb, after the clock
    /// task's init(). A DynamicClock must list OpampSystem (the
    /// static_assert below). Configure the op amps first (35.3.1) or right
    /// Blocking wait, with no bound: spins on SETTLED. Never returns while
    /// the op amp is disabled - the timer only runs when it is on, which
    /// is the caller's business.
    /// Configure Op as a follower of `source` (MUXNEG = OUT, the ladder
    /// off), `mode` its enable regime. False, writing no register, for a
    /// source this instance cannot take or a pad this package does not
    /// bond (Op::init). A setup verb.
/// Only on packages with OP2 - using the type on a package without it
/// fails the static_assert below, so a 28/32-pin build fails to compile
/// rather than half-working.
++ b/brio/avrxt/pin.hpp
// The I/O pins of the AVR DA, DB and EA (PORT, DS40002247B ch. 18,
// DS40002443A ch. 18): what a program uses for an LED, a button, a row
// of keys, a pad an analog input reads, a select line a request carries.
// A pin is a type: Pin<'A', 5> names PA5 when the program is compiled
// (the letter and the number are template arguments), and its static
// functions drive it; there is no object to construct. The smallest use:
//   using Led = brio::Pin<'A', 5>;   // PA5
//   Led::output();                 // PORTA.DIRSET = PIN5_bm
//   Led::toggle();                 // PORTA.OUTTGL = PIN5_bm
//   Led::configure({.pull = brio::PinPull::up, .sense = brio::PinSense::falling});
//   ISR(PORTA_PORT_vect) { const uint8_t who = brio::Port<'A'>::take_flags(); ... }
//
// output() and toggle() are one store each; configure() writes the pin's
// whole control byte PINnCTRL (here: the pull-up and an interrupt on the
// falling edge) in one store; ISR(PORTA_PORT_vect) is avr-libc's spelling
// of the handler of port A's one vector, which the program binds (the
// driver binds none), and take_flags() is its body: which pins raised it,
// those flags cleared. A flag the handler leaves set keeps the request
// active (18.3.3), so the handler is entered again at once, for ever.
//
// Who calls what, and from where. set() and clear() are one SBI or CBI on
// the pin's VPORT alias (a constant address in the bit-accessible I/O
// space; read in the release listings), read() and flag() one load of it;
// toggle(), output(), Port's mask verbs, clear_flag() and take_flags()'s
// clearing store are single stores into the PORT's set/clear/toggle and
// flag registers: all of them are legal from any context, an interrupt
// handler included, and none masks interrupts. configure() is one store
// too. The single-field verbs - input() (its pull half), pull(),
// invert(), sense(), input_enable() - read, modify and write the pin's
// PINnCTRL, so two contexts configuring the same pin at once can lose a
// field: call them from setup or from an active object's handler (an
// active object is a component of a brio program with its own event
// queue, served one event at a time by the kernel). Nothing here waits.
//
// The parts, register-level:
//
//  Port<'C'>       the port resource - what a pin does not possess:
//                  mask operations, and on the AVR EA the pin event
//                  slots' read-back;
//                  CBI), configure(PinConfig) as one PINnCTRL store
//                  rides the multi-pin engine;
//  PinRef          a pin as a run-time value (3 bytes) that can travel
//                  inside a request event.
// Which ports exist depends on the package (no PORTB or PORTE on the 28-
// and 32-pin parts, PORTG on the 64-pin ones alone): a pin on a port the
// package has not got is a compile error (Port's static_assert over
// port_exists(), avrxt/device_tables.hpp, the reserve: the table of what
// each part has, read off the device header). Which pins of an existing
// port a package bonds is not checked (docs/avrxt/port.md, "Not covered
// yet").
/// It holds the PORT, not the VPORT: an edge is one store into OUTSET
/// Named in a branch only an absent port letter reaches: a call that
/// The PORT of a letter chosen at run time (for drivers whose pin is a
/// to one address when the letter is a constant. A constant letter
/// `input_disable` turns the digital input buffer off - IN freezes,
/// The whole PINnCTRL as one value, written in one store by
/// The internal pull: one direction on this family (PORT has PULLUPEN
/// Port<'C'>: the typed view of one port's own registers - what a pin
/// (several pins can fire together: the ISR reads the mask), the
    // Interrupt flags: write-1-to-clear - always plain stores (an RMW
    /// PORTCTRL.SRL: slew-rate limitation for every pin of this port.
    /// The pin an event slot of this port carries (PORTx.EVGENCTRLA,
    // Which ports exist depends on the package (e.g. no PORTB/PORTE on
    // 28/32-pin parts, PORTG only on 64-pin), hence the #ifdef guards in
    // Port<L>::regs(). Requesting a port the package lacks is a compile
    // error: Port<L>'s static_assert on port_exists(L).
    /// Whether this pin senses fully asynchronously: it wakes from every
    // Basic I/O. toggle() and output() are one store each into the
    // PORT's OUTTGL and DIRSET, set() and clear() an SBI and a CBI on the
    // VPORT's OUT: legal from any context. output() drives the level OUT
    // already holds (0 out of reset), so a pin that must come up high is
    // set() first.
    /// The whole PINnCTRL in one store (invert + pull + input level
    /// (and freezes IN at its last value), on is the reset state. On
    /// this family the buffer's switch is a code of the ISC field, the
    /// engine (18.3.2.4): the pins are grouped by port at compile
++ b/brio/avrxt/platform.hpp
 * What the brio kernel stands on when it runs on an AVR DA, DB or EA:
 * this target's realization of the kernel's Platform concept, the
 * compiler-checked set of requirements kernel/platform.hpp states. A
 * program names it once and passes it to every kernel template - its
 * queues, its time events and its Tenuto - as the maintained example
 * `blink` (avrxt/src/apps/blink.cpp) spells it:
 *
 *     using P = brio::AvrPlatform;
 *     ...
 *     ISR(RTC_PIT_vect) { brio::Ticker::pit(); }
 *
 *     int main() {
 *         SysClock::init();
 *         brio::Ticker::init();
 *         sei();
 *         brio::Tenuto<P, Blinker, Supervisor>::run();
 *     }
 *
 * It gives the kernel a critical section (interrupts masked for a
 * scope), an idle that sleeps until the next interrupt without losing
 * one that arrives at the wrong moment, the program's source of time
 * (avrxt/ticker.hpp's Ticker, 1024 Hz on the RTC's periodic interrupt),
 * a breakpoint for a panic and the storage a panic record survives a
 * reset in. Who calls what: the kernel loop calls idle() in main-loop
 * context, with interrupts already masked, and nothing else should;
 * now() may be read from any context (Ticker::ticks() masks for the
 * four loads that copy the count); CriticalSection may be opened from
 * any context and nests; break_here() is the panic path's and
 * panic_record() the panic path's and the next boot's. A program binds
 * the timebase's vector itself (RTC_PIT_vect to Ticker::pit(), above);
 * the kernel binds no vector, and a program whose handler does not call
 * pit() never advances time.
 *
 * The one promise the kernel relies on and no member states - no
 * interrupt handler interrupts another (docs/design/kernel.md, "The
 * model in one page") - this core keeps in its interrupt controller,
 * not in SREG: the I bit is not cleared on entry to a handler, nor set
 * by RETI (DS40002247B and DS40002443A 7.6.3). Every vector but the NMI
 * is at priority level 0 out of reset (15.3.2.4.3), a level-0 handler is
 * interrupted only by the one vector CPUINT.LVL1VEC raises to level 1 or
 * by an NMI (15.3.2.4.2), and CPUINT.STATUS.LVL0EX holds the others off
 * until the handler's RETI (15.5.2). Nothing in brio writes LVL1VEC.
 * only provides the storage the record lives in. The sleep modes deeper
 * than IDLE, which idle() takes when something above the kernel armed
 * one, are avrxt/sleep.hpp's.
    /// The kernel's idle: entered by the loop, in main-loop context, with
    /// interrupts masked and nothing to do; sleeps until the next
    /// interrupt and returns with interrupts enabled. Not for a program
    /// to call. sei takes effect after the following instruction,
    /// IDLE by default: the CPU stops, every peripheral and every
    /// But an already-armed mode stands. If SEN is set when this runs,
    /// manager's to clear: it disarms on the first event it dispatches
        // SLPCTRL.CTRLA is not under CCP (13.3.5): only VREGCTRL is, and
    /// Halt in the debugger when the OCD is active, plain NOP otherwise:
    /// the panic path's breakpoint (kernel/panic.hpp), callable from any
    /// context. With no debugger the panic continues into its reporter.
    /// The kernel's clock: the Ticker's count, readable from any context
    /// (what kernel/time_event.hpp arms deadlines against).
    // custom section attribute as `"awG"` without the group name, and gas
++ b/brio/avrxt/reset.hpp
 * The reset controller and the watchdog of the AVR DA, DB and EA
 * (RSTCTRL and WDT: DS40002247B ch. 14 and 22 on the DA/DB, DS40002443A
 * ch. 14 and 21 on the AVR EA, the same bits, SWRR's one bit named
 * SWRE): why the program is running, and how to end it on purpose. A
 * program reads the cause once at boot, and may arm the watchdog so that
 * a hang ends in a reset; the smallest use (docs/avrxt/platform.md, "How
 * to use it"):
 *
 *     const brio::ResetFlags why = brio::Reset::take_flags();      // read + clear
 *     brio::Watchdog::arm(brio::WdtTime::ms250);   // then WDR from the main loop
 *     brio::Watchdog::clear();
 *
 * Who calls what. Reset::take_flags() is main's, once, first thing at
 * boot (below); flags() and clear_flags() are one load and one store.
 * Reset::software() is legal from any context, a panic reporter's
 * included, and never returns. Watchdog::clear() is the WDR instruction,
 * legal from any context; where the program calls it from is what the
 * watchdog then judges. Watchdog::arm() and off() wait for an earlier CTRLA write's
 * synchronization before they write (wait_sync(), bounded at 2 000 000
 * polls so a stopped OSC32K cannot hang the caller, false when it runs
 * out) and not after it; sync() is that wait on its own. None of these
 * masks interrupts: the CCP-protected writes are the avr-libc
 * _PROTECTED_WRITE sequence.
 * register on the target for the full story". `Reset::take_flags()` is
 *  - RSTFR accumulates. A flag is set by its reset source and cleared
 *  - SRAM is not promised across a reset. The data sheet states that
 * the same encoding, 8 ms to 8 s. CTRLA and the LOCK bit are CCP
    /// returns is the reset that started this run, and the register is
/// data sheet's nominal times, the values are the register codes.
    /// window mode this is also the instruction that must land inside
    /// does. Two details the code around it has to respect: the first
    /// is what activates the window and is never judged (22.3.3.2), and
    /// write asks for is in force only once this returns: the register
    /// which the old configuration is still the one running. That is
    /// is judged by the configuration being replaced. arm() does not
++ b/brio/avrxt/rtc.hpp
 * The RTC peripheral of the AVR DA, DB and EA: a 16-bit counter on the
 * RTC's own slow clock, counting in idle sleep and, when asked, in
 * standby - for a periodic interrupt or event at a rate of the
 * program's own (a second, a sampling epoch), an alarm inside that
 * period, a trimmed crystal - and the periodic interrupt timer (PIT),
 * alive in every sleep mode, whose interrupt is the kernel's tick and
 * whose divided clocks are event-system signals. In a brio program the PIT
 * belongs to the timebase, avrxt/ticker.hpp's BasicTicker, which a
 * program binds and starts and never configures through this header; a
 * program comes here for the counter, the PIT's event signals or a trim.
 * The header holds three resources (a resource, in brio, is one function
 * per register fact, for a program that drives the block by hand), as
 * docs/avrxt/rtc.md describes them; the RTC chapter is DS40002247B
 * ch. 26 on the AVR DA/DB and DS40002443A ch. 24 on the AVR EA, the
 * event slots, 24.13.17.
 * The smallest use, a counter that overflows once a second, its
 * interrupt bound by the program (`seconds` is the program's own
 * counter, `SysClock` its clock alias as avrxt/ticker.hpp spells it):
 *
 *     ISR(RTC_CNT_vect) { if (brio::Rtc::take_flags().ovf) ++seconds; }
 *
 *     int main() {
 *         SysClock::init();
 *         brio::RtcClock::select(brio::RtcSource::osc32k);   // if no Ticker owns it
 *         brio::Rtc::init<brio::RtcConfig{.prescaler = brio::RtcPrescaler::div1,
 *                                         .period = 32767}>();   // one second
 *         brio::Rtc::enable_ovf_interrupt(true);
 *         sei();
 *         ...
 *     }
 *
 * A program with a Ticker leaves out the select(): Ticker::init() has
 * made it. The first overflow lands anywhere inside the first period
 * (the first fact below); every one after it, a period apart.
 *
 * Who calls what, and from where. RtcClock::select() is setup, once,
 * before either function is enabled. Rtc::init() and Pit::init() are
 * setup too (main(), or an active object's handler); Rtc::init() leaves
 * the counter's interrupts off for the caller to enable. take_flags()
 * and Pit::take_flag() are interrupt bodies, called from the program's
 * ISR(RTC_CNT_vect) and ISR(RTC_PIT_vect) and from nowhere else. The flag
 * reads and clears and Pit::enable_interrupt() are one load or one store,
 * legal from any context. The other writing verbs (the setters, the
 * enables, prescaler(), calibrate(), the interrupt enables of the
 * counter, Pit::event_divider<slot>()) are read-modify-write sequences
 * or synchronized writes: call them from main() or an active object's
 * handler, never from two contexts at once. What waits: every write to
 * CTRLA, CNT, PER, CMP or PITCTRLA first polls its busy flag, at most
 * 0xFFFF times and a last read (rtc_wait, rtc_pit_wait); the flags clear
 * in about 2.8 CLK_RTC periods, measured (docs/avrxt/rtc.md), and of
 * the verbs only sync() returns whether the wait ran out. Nothing here masks
 * interrupts: a 16-bit access racing an ISR's is the caller's to guard
 * (the TEMP fact below).
 *
 * One block, one clock. The RTC peripheral has a single clock select
 * owner of CLKSEL in a brio program is the timebase - `BasicTicker`
 *    runs as soon as either is enabled: the first PIT interrupt and the
 *    to write; the wait is bounded, so a CLK_RTC that no peripheral
 *  - crystal error correction is a trim, not a cure: CALIB carries a
 *    sees the correction quantized). A negative correction requires
/// CLKSEL: the source of CLK_RTC for both functions.
    /// (clock.hpp's Xosc32k) - and the oscillator still enabled: the
    /// Setup (main() or an active object's handler), after CLKSEL is
    /// selected; the run-time form below does the work.
    /// False - and nothing written - outside +-127 ppm, or for a
    /// The OVF and CMP interrupt enables (INTCTRL, a read-modify-write):
    /// both reach the program through RTC_CNT_vect and take_flags().
    /// default. The first interrupt lands anywhere inside one period
++ b/brio/avrxt/sleep.hpp
 * The sleep controller of the AVR DA, DB and EA (SLPCTRL, DS40002247B
 * ch. 13, DS40002443A ch. 13): what a program uses to stop the chip
 * deeper than the kernel does by itself. The kernel's idle hook already
 * sleeps in IDLE whenever every queue is empty (AvrPlatform::idle(),
 * avrxt/platform.hpp), so a program that only wants "nothing to do right
 * now" needs nothing from this header. STANDBY and POWER-DOWN stop
 * clocks and shorten the list of what can wake the chip, and this header
 * is the mechanism for them, in two shapes. Code that owns the moment
 * enters a mode itself, its wake source configured and enabled first
 * (the shape docs/avrxt/platform.md, "How to use it", spells out):
 *
 *     brio::Pit::init(brio::PitPeriod::cyc4096);        // 125 ms, interrupt on
 *     brio::Sleep::enter(brio::SleepMode::standby);     // interrupts on, wake armed
 *
 * and an active-object program names AvrSleepSite, at the end of this
 * file, in its power manager (util/power.hpp), which arms a mode after
 * every stakeholder agreed and leaves the SLEEP instruction to the
 * kernel loop's idle().
 *
 * Who calls what. Sleep::arm() and disarm() are one store of
 * SLPCTRL.CTRLA each, preceded by the erratum's NOP (below); armed() and
 * armed_mode() one load: none masks or waits. Sleep::sleep() and enter()
 * stop the CPU until an enabled interrupt the armed mode keeps comes,
 * and touch SREG not at all: global interrupts must already be on, and
 * with no wake source that reaches the armed mode only a reset or a
 * debugger comes back (13.3.1). They are for main context: a program's
 * own code before the kernel runs, or code that owns the moment - under
 * a kernel the loop's idle() executes the instruction. Vreg's verbs are
 * setup verbs: a read and a CCP-protected write of VREGCTRL each, and
 * high_temp_low_leakage(true) refuses (false, nothing written) while a
 * TWI client or the CCL is enabled. AvrSleepSite's arm() and disarm()
 * are the power manager's, called from its handler.
 *
 * VREGCTRL and the regulator control exist on the DA and the DB alone.
 * The AVR EA's SLPCTRL (DS40002443A ch. 13) is CTRLA alone, the same
 * three modes: no VREGCTRL and no regulator control anywhere, so Vreg
 * and VregPower do not exist there (device tables' has_vreg). What
 * follows is the AVR DB's chapter unless it names the EA. The mechanism
 * only: which mode is armed, what a SLEEP instruction then does, and
 * what the regulator is told to do while the CPU is stopped. The policy
 * - when a program may stop its clocks - is not here and is not the
 * kernel's either (see the design note at the end of this comment).
 * rewritten by errata DS80000915F clarification 3.4.1; what follows is
 * peripheral's flag alone is that request (measured): a TCB with
 * the source stays alive unrequested, so the wake-up pays no start-up
 * fast wake-up for the other source too (see "Wake time" below). Wake-up
 * client's address match is the only wake-up its peripheral offers: the
 * - the RTC's counter stops even with RUNSTDBY, the PIT half does not.
 * high_temp_low_leakage(true) refuses while a TWI client or the CCL is
 * Wake time. Six CLK_PER cycles from IDLE. From standby or power-down
 *   the oscillator - OSCHF comes back in 24 us, a 24 MHz crystal in
 *   the regulator - 290 us more, but only when the device really let
 *   regulator at full drive even when it is not the main clock source,
 * Two register facts. SLPCTRL.CTRLA is not under CCP; VREGCTRL is the
 * A debugger break wakes the device to Active mode whether or not an
 * On the AVR EA (DS40002443A ch. 13; docs/avrxt/platform.md has the
 * from every mode - and every pin of the EA senses fully asynchronously,
 * running in standby. The wake pays six CLK_PER cycles and one
 * oscillator start-up (table 13-4) and, measured, nothing else: there is
 * Design position. The kernel sleeps by itself in IDLE and only in IDLE
/// SLPCTRL.CTRLA.SMODE (13.5.1). The values are the register codes (the
    /// Arm `m`: one store of SMODE + SEN, preceded by the erratum's NOP
    /// Warning (13.3.1): the wake-up sources must be configured and
    /// enabled, and global interrupts on, before this runs. With no
    /// Enabling is refused while a TWI client or the CCL is enabled:
    /// Any TWI instance this package has, with its client half enabled
 * It only arms. The SLEEP instruction is still the kernel loop's:
 * source must be configured, enabled and reachable in the armed mode
++ b/brio/avrxt/spi.hpp
 * The Serial Peripheral Interface of the AVR DA and DB (DS40002247B
 * ch. 28) and of the AVR EA (DS40002443A ch. 26, the DB's chapter word
 * for word, SPI0 alone): what a program uses to drive a display, an ADC,
 * a touch controller or a memory on an SPI bus as the host, or to answer
 * another host as a client. The header has two strata, as
 * docs/avrxt/spi.md describes:
 *  the resource - Spi<n>: the typed view of one instance. A config
 *  struct owns the whole configuration (route, host/client role,
 *  transfer mode, data order, bit rate, the Client Select policy, the
 *  buffer bits); init<cfg>() folds it and refuses at compile time what
 *  this package cannot bond, init(cfg) computes it at run time and
 *  returns false instead. Then the verbs the roles share: enable/disable,
 *  the role and its host demotion readback, the rate, the mode, the data
 *  register, the two INTFLAGS layouts with the clear discipline of each
 *  flag, the interrupt enables, the two ISR bodies, and release() -
 *  route back to NONE with every pin handed back to PORT. The class is a
 *  monostate: its state is static and it has no instances.
 *  the tasks - what an application names (a task, in brio, is a driver
 *  layer with state of its own over a resource, served from its
 *  peripheral's interrupt or called by an active object):
 *    SpiHost<n, route>    the transfer engine: two-phase descriptor
 *                         instance in buffer mode with two bytes in
 *                         read by count, CS and DC owned by the engine.
 *    SpiClient<n, route>  the client side: the runs answer(), exchange()
 *                         bodies. A run is the unit a transport moves:
 *                         a buffer and a length, a single byte its
 *                         degenerate case.
 * The smallest use, a full-duplex transaction on SPI0's DEFAULT route
 * (PA4 MOSI, PA5 MISO, PA6 SCK) through the arbiter of util/spi_bus.hpp,
 * an active object (a component of a brio program with its own event
 * queue, served one event at a time by the kernel) that runs the
 * transactions other active objects post to it, one at a time:
 *
 *   using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *   constexpr SysClock clock;
 *   using SpiHw = brio::SpiHost<0>;
 *   using Bus = brio::SpiBus<SpiHw, brio::AvrPlatform>;
 *
 *   ISR(SPI0_INT_vect) {
 *       if (SpiHw::isr()) brio::post<Bus>(brio::TransferDone{SpiHw::status()});
 *   }
 *
 *   // in a handler of the active object Me, tx and rx its own buffers:
 *   brio::post<Bus>(SpiHw::Request{
 *       CsPin::ref(), {}, {}, 0,
 *       brio::lend<brio::Lease::reply>(tx), brio::lend<brio::Lease::reply>(rx), len,
 *       brio::reply_to<Me, brio::SpiDone>()});
 *
 *   int main() {
 *       SysClock::init();
 *       CsPin::set();                  // the select: the program's pin, idle high
 *       CsPin::output();
 *       SpiHw::init(clock);            // host, buffer mode, SSD set
 *       sei();
 *       brio::Tenuto<brio::AvrPlatform, Me, Bus>::run();
 *   }
 *
 * The request leaves its clock (CLK_PER/16), mode (0) and completion
 * style (the interrupt pump) at their defaults; the reply, an SpiDone,
 * reaches Me when the transaction is over, and tx and rx are the
 * engine's until then.
 *
 * Who calls what, and from where. Spi<n>::init() and release(),
 * SpiHost::init() and SpiClient::init() are setup verbs: main(), after
 * the clock task and before interrupts are enabled globally; none of
 * them waits (SpiHost::init() empties the receive FIFO with at most four
 * DATA reads). SpiHost::start() and recover() are the arbiter's, called
 * from its dispatch in main context; prime() is a main-context verb for
 * a caller that frames the select itself. start() of a polled request
 * spins inside the dispatch until the transaction ends - not bounded,
 * because with SSD set nothing can take the clock generator from this
 * host and a transfer written always completes - and before the first
 * clock it spins the request's cs_setup_us; global interrupts stay
 * enabled, masked only around the burst's last DATA store and the TXCIF
 * clear behind it (AvrPlatform::CriticalSection). A pumped request's
 * launch masks its two DATA stores and the flag clear behind them the
 * same way and returns. recover() waits for the bytes in flight, at most
 * 512 polls of TXCIF. SpiHost::isr() is the interrupt body: the
 * program's ISR(SPIn_INT_vect) calls it and nothing else does; it runs
 * to completion, no other handler interrupting it (avrxt/platform.hpp),
 * and masks nothing of its own. rebase() is the DynamicClock's fan-out,
 * with the bus idle. SpiClient's runs and per-byte verbs are polled
 * main-context verbs (setup, or an active object's handler): a run
 * spins until its bytes have moved, the host raises the select, a turn
 * comes late or its `spins` budget runs out, and wait() and the per-byte
 * exchange() are bounded by their `spins` (500000 by default). Storage:
 * a Request's cmd, tx and rx are lent until its SpiDone (Lease::reply);
 * the polled path reads every field through the reference it was lent
 * and copies nothing, the pump copies the pins, the data phase's spans
 * and its length at launch. A client run's spans are the caller's again
 * when the call returns. Spi<n>'s configuring verbs (enable(), clock(),
 * mode(), lsb_first(), client_select_disable(), buffer_mode(), the
 * interrupt enables) are read-modify-writes of CTRLA, CTRLB or INTCTRL
 * with no guard: one owner calls them, from setup or one active object.
 * Its flag reads are single loads and its flag clears single stores.
 *
 * clarifications 3.5.1/3.5.2/3.7.3, and DS80000882C 2.10.1; the AVR EA's
 * route table DS40002443A 17.3.5, at SpiRoute below):
 *  - on the AVR DA and DB, SPI0 and SPI1 exist on every package (28, 32,
 *    48 and 64 pins alike): there are no instance tiers here, only route
 *    ones; the AVR EA has SPI0 alone (device_tables.hpp's spi_count, the
 *    reserve: the table of what each part has, read off the device
 *    header);
 *  - the routes are PORTMUX.SPIROUTEA, two bits per instance on the AVR
 *    DA and DB (three for the AVR EA's SPI0), with a NONE code that
 *    disconnects every pin. Which positions a package bonds is the
 *    device header's route enums, and they are the table below -
 *    28/32-pin AVR DA/DB parts list DEFAULT and NONE only for both
 *  - an erratum beats the header: the 48-pin header lists SPI1 ALT2
 *    (MOSI PB4, MISO PB5, no SCK, no SS position), and DB errata 2.11.1
 *    says that position is non-functional on 48-pin devices (rev. A4/A5;
 *    fixed in B0). The DA errata (DS80000882C) has no twin item, but the
 *    48-pin DA header bonds no SCK for that route either - a host cannot
 *    clock and a client cannot be clocked - so the route is refused on
 *    48-pin parts of both families, at compile time and at run time;
 *    line must be disabled (CTRLB.SSD = 1) for Host mode to survive. It
 *  - two INTFLAGS layouts at the same address, chosen by CTRLB.BUFEN,
 *    with different clear disciplines (28.5.4, 28.5.5):
 *      normal mode: IF is cleared by writing a one to it or by reading
 *      INTFLAGS while it is set and then accessing DATA; WRCOL has only
 *      the read-then-DATA sequence documented, so
 *      clear_flags_by_data_access() performs it;
 *      plain store of the single bit - an RMW would write back every
 *      writing DATA - never by a store to INTFLAGS;
 *  - a read that empties the FIFO as a byte enters it is not taken
 *    RXCIF falls; the next read returns the same byte again and brings
/// SPI pin routing (PORTMUX.SPIROUTEA), by name. On the AVR DA/DB the
/// values are the device header's group values, `none` included (DEFAULT
/// Where one signal of one route sits, and whether this package bonds
/// The position of one signal of one route on this package.
/// The position of one signal of one route on this package.
/// The values are the device header's codes.
/// The seven host bit rates, named by what they divide CLK_PER by
/// order is the register's: its two high bits are PRESC (DIV4 for div2
/// The fastest rate whose SCK does not exceed `max_sck_hz` - the
    /// CTRLA.MASTER. The hardware clears it on a demotion (28.3.2.1.3),
    /// and the AVR only refreshes the SCK output while the peripheral is
    /// still low 10 us after a switch to mode 3). SpiHost's reprogram(),
    /// behind its apply(), does the disable/preset/enable dance itself; a caller
    /// Write DATA: in host mode this starts a transfer; in client mode
    /// The documented W1C half of the normal layout: IF alone (a plain
    /// set, then access DATA. It is the only way the chapter gives for
    /// DREIF: room in the transmit buffer. Cleared by writing DATA -
    /// does not do this: its store of one to RXCIF drops the flag and
    /// ISR body for SPIn_INT_vect in normal mode: read INTFLAGS, then
    /// ISR body for SPIn_INT_vect in buffer mode: the flags, with the
    /// RXCIF and DREIF are not cleared here - they follow the data: the
 * The transfer engine: the target-side half of the SPI stack, driven by
 * byte pump under the SPI interrupt (the AVR DA, DB and EA have no DMA
 * controller).
 * Transaction descriptor (Request) - two phases in one chip-select
 *   phase 1 (optional): cmd[cmd_len] transmitted with DC low
 *   phase 2 (optional): len bytes with DC high, full duplex -
 * CS is active low and asserted/released by the engine around the whole
 * The engine runs the instance in buffer mode (28.3.2.1.2): the
 *    (rule 3). The engine reads by count: it keeps how many bytes it has
 *    written and not yet read and reads what the transmitter's flags
 *  - TXCIF says shifter and buffer are both empty: a condition the
 *    write only when it is cleared behind that write, while the byte
 * The polled path completes inside start() and reads every field of the
 * The pump (polled false) copies at launch what its tenure needs - the
 * ISR wiring (app glue, as usual), with `using Bus = brio::SpiBus<SpiHw,
 * brio::AvrPlatform>;`:
 *       if (SpiHw::isr()) { brio::post<Bus>(brio::TransferDone{SpiHw::status()}); }
    /// The layout: 21 bytes and no padding on this core (every type here
    /// settings. The first fifteen are the tenure's - what the pump
        /// Phase 1, sent with DC low; lent until the reply lands.
        /// Phase 2 out, null = 0xFF dummies; lent until the reply lands.
        /// Phase 2 in, null = discard; lent until the reply lands.
        // Per-transaction bus configuration: on a shared bus every
        // change - nothing per byte. The rate is a division of CLK_PER,
        // right for slow clocks and short transfers); true = polled
        // polling is that this dispatch blocks for the whole transfer
        // 5.2) - measured: SDO drives ~4 us after CS falls, and a frame
     * chip selects are the engine's, not the peripheral's), buffer mode
     * `max_sck_hz` is an optional ceiling for the whole bus: with it set
     *
     * True with the instance enabled as a host, the receive FIFO empty
     * and the pump idle. False, no register written, when this package
     * does not bond the route's SCK (`available`), when even CLK_PER/128
     * is faster than the ceiling, or when the resource refuses the
     * configuration.
    /// that still honours it. The bus must be idle: a transfer in
    /// For callers that frame the select window themselves (Request.cs
    /// by hand inverts that order, and a mode change is a CPOL flip on
    /// the wire (the SCK pad moves to the new idle level while the
    /// Prime first, then assert the select. Main context.
    /// The request is lent for this call and read through `r`: the
     * SPI interrupt body - call from ISR(SPIn_INT_vect) and from nowhere
     * else. The pump runs
 * host sets the pace, and the only thing this side controls is what it
 *    written last goes out. A write during a transfer is ignored and
 *    decides whether the first byte the host clocks out is the one the
 *    shifter while SS is high) or a dummy equal to whatever the shift
 * reset and a partial byte is lost (28.3.2.2.3). selected() reads it.
 * Its unit is the run - answer(), exchange(), receive() serve a
 * degenerate case. In buffer mode the runs read by count and never by
    /// The SS pin's level: low means this client is selected and the
    /// How many answers must be queued ahead of the frame the host is
    /// other strata's. One here: the byte written last goes out with the
    /// during a transfer it is ignored and sets WRCOL.
    // A client's unit is the run: the answers to a transaction, or what it
    // the byte of transfer t + 1 only once the older is read. So a turn -
    // A run is called with the select high and the transmitter empty (a
     * Answer a transaction with `out`, polled on DREIF alone, reading
     * The run ends when the host raises the select, however many bytes it
     * left with. Nothing is read while it runs, so the FIFO fills and what
     * Exchange: answer with `out` and keep what the host sends in `in`,
     * answer()'s; in buffer mode each turn then reads the byte before the
     * Receive a run into `in`. In buffer mode the run still writes, the
    /// the clocking - but the fastest SCK it can follow is CLK_PER/6
++ b/brio/avrxt/tca.hpp
 * word; its route table differs): what a program uses for PWM on up to
 * six pins, a square wave of a chosen frequency, a period with pulses at
 * fixed points inside it, or a 16-bit count of events. The header has
 * the two strata of every brio peripheral header, as docs/avrxt/tca.md
 *  the resource - Tca<n>: the typed view of one instance in normal mode
 *  interrupt enables, the interrupt bodies (one vector per flag: ovf(),
 *  the tasks - what an application names (a task is a driver layer with
 *  state of its own over the resource; the tasks, like the resource, are
 *  monostates: classes whose state is static, with no instances):
 *    TcaPwmCentered<n, port, steps>
 *                                 dual-slope: three 16-bit centre-aligned
 *                                 PWM channels of one period, the OVF
 *                                 event at a chosen instant of it
 *  A task owns its instance: nothing checks that two tasks are not
 *  started on one Tca<n>, and the second init() resets the instance
 *  under the first.
 *
 * The smallest use, six LED channels on PC0..PC5:
 *
 *   using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *   using Bar = brio::TcaPwm<0, 'C'>;      // TCA0 -> PC0..PC5
 *
 *   int main() {
 *       SysClock::init();                  // CLK_PER = 24 MHz
 *       Bar::init();                       // split mode, div16: 5859 Hz, all six dark
 *       Bar::duty<2>(64);                  // PC2 high for 64 counts of every 256
 *       ...
 *   }
 *
 * TcaPwm::init() takes no clock: its frequency is CLK_PER / 16 / 256 at
 * whatever rate CLK_PER runs, so the clock task comes first, and it
 * enables no interrupt. The tasks that take the clock tag (`constexpr
 * SysClock clock;`) - FrequencyGenerator and Heartbeat - compute their
 * divisors from the rate the clock type claims, and a DynamicClock that
 * does not list them is a compile error at their init(). A task that
 * enables an interrupt (Heartbeat, by default) needs its vector bound by
 * the program: `ISR(TCA0_OVF_vect) { Beat::beat(); }` for a
 * `using Beat = brio::Heartbeat<0, 'D'>;`.
 *
 * Which context may call what. Nothing here waits, nothing masks
 * interrupts, nothing holds a caller's buffer.
 *  - The configuring verbs - init(), init<cfg>(), the tasks' init(),
 *    reset(), route(), clock(), enable()/disable(), output<ch>(),
 *    output_value<ch>(), the interrupt enables, event_a_on/event_b_on,
 *    EventCounter::direction_on() - are setup verbs: main(), after the
 *    clock task and before sei(), or an active object's handler (an
 *    active object is a component of a brio program with its own event
 *    queue, served one event at a time by the kernel, so its handlers
 *    never overlap). Most of them read-modify-write a register with no
 *    set/clear twin (CTRLA, CTRLB, CTRLC, INTCTRL, EVCTRL, and
 *    PORTMUX.TCAROUTEA, which holds both instances' fields), so two
 *    contexts racing on one of them can lose a field: never call them
 *    from an interrupt handler that may interrupt another such call.
 *  - The interrupt bodies - ovf(), cmp<ch>(), lunf(), hunf(), lcmp<ch>(),
 *    Heartbeat::beat(), EventCounter::ovf() - are called from the
 *    program's ISR() for that vector and nowhere else; each is one store
 *    that clears its flag.
 *  - The flag verbs, the commands through CTRLESET/CTRLECLR (restart(),
 *    update(), direction_down(), lock_update()) and the 8-bit reads are
 *    single loads or stores, legal from any context. The 16-bit accesses
 *    - count(), period(), compare<ch>(), their buffered writes, and
 *    through them TcaPwm16::duty(), TcaPwmCentered::duty(),
 *    FrequencyGenerator::set_hz(), Heartbeat::pulse_us() - pass through
 *    the instance's one TEMP register (23.5.13): a main-context access
 *    racing an interrupt handler's 16-bit access of the same instance
 *    corrupts both, so such an access belongs in a critical section
 *    (AvrPlatform::CriticalSection, avrxt/platform.hpp) or in the
 *    handler alone. TcaPwm::duty() and TcaPwmCentered::duty() also
 *    read-modify-write CTRLB, and FrequencyGenerator::set_hz() CTRLA:
 *    one context per instance.
 *  - rebase() is called by the DynamicClock that lists the task, from
 *    its set().
 *    - in split mode CMP > TOP is low, hence TcaPwm's PORT endpoints);
 *    no workaround: refused here), and less TCA1's PORTB on the 28/32-pin
 *    AVR EA, whose header keeps that code as "not connected to any pins".
 *    WOn drives the pin only if CMPnEN and the pin is an output (PORT
 *    INVEN inverts it).
/// route does not exist on this package - the device header is the
/// route's shape is this one expression (written out here rather than
/// One TCA route on this package, read whole from the table above
    /// Split view's INTCTRL: set or clear the interrupt-enable bits in
    /// `bit` (a read-modify-write; the enable_lunf/hunf/lcmp verbs below).
    /// PORTMUX: move the WO group to port p (no check: init() checks).
    /// The command goes through the split view with CMDEN = both: in
    /// CTRLC: WOn's output value - readable any time, writable only
    /// CNT = 0, all WO low, the period restarts (DIR back to up on the
    /// DB's A4/A5, DS80000915F 2.12.1, and on every DA, DS80000882C
    /// 2.11.2; kept on the AVR EA).
    /// Event input A / B from channel ch; the action is in the config.
/// TcaPwm<n, port>: TCA n in split mode as six independent 8-bit PWM
/// (TCA1's three-channel ones, on PORTC among others) are not offered.
/// Not a ClockUser: the frequency follows CLK_PER and a LED does not
/// care - an app that does can call clock() itself. Channel<ch> is the
/// PwmChannel type (max 255) that generic actuators (RgbLamp) take.
///   Bar::init();                       // split mode, div16, all six dark
    /// both halves, all six channels at duty 0 (compare outputs off,
    /// CTRLB = 0, so PORT drives the pins), pins as outputs driven low,
    /// counter running from the chosen prescaler. Setup context; no
    /// interrupt enabled.
    /// A read-modify-write of CTRLB and one store: one context per
    /// instance.
/// written buffered (lands at BOTTOM: glitch-free) with the hardware's
/// TcaPwmCentered<n, port, steps>: three 16-bit center-aligned PWM
/// buffered (land at BOTTOM, glitch-free); the endpoints leave the
/// not a ClockUser: the duty survives a clock change, the frequency
/// from BOTTOM for the width pulse_us<ch>(us) sets) that mark moments
/// inside the period - the shape of a synchronized sequence (fire a one-shot
/// direction_on(channel) input B's level sets the direction (down while

++ b/brio/avrxt/tcb.hpp
 * ch. 24) and of the AVR EA (DS40002443A ch. 23, the same IP): what a
 * program uses to measure, count or time one signal - the period, the
 * width or the duty of a pulse train, a count of edges (16 bits on one
 * TCB, 32 on two), a single pulse of a set width started by an event,
 * an interrupt at a rate of its own, a watch that fires when the next
 * edge comes too late, or one 8-bit PWM channel. The signals arrive
 * through the event system (evsys.hpp): a pin's level is an event. The
 * header has the two strata of every brio peripheral header, as
 * docs/avrxt/tcb.md describes:
 *  the resource - Tcb<n>: the typed view of one instance. A config
 *  struct owns the whole configuration (mode, clock, compare, event
 *  input and its edge, filter, async, cascade, sync-update, output pin
 *  and its route, run-standby); init<cfg>() folds it, init(cfg)
 *  computes it at run time. Then the verbs the modes share:
 *  count/compare/capture, running, the two flags, the two interrupt
 *  enables, the interrupt body (take_flags: one vector per instance,
 *  CAPT and OVF share it), the event hooks capture_on / count_on over
 *  evsys.hpp.
 *  the tasks - what an application names (a task is a driver layer with
 *  state of its own over the resource), each a mode with its knobs
 *    DutyMeter<Tcb>        FRQPW: period and width in one sequence
 *  The resource and the tasks are monostates (classes whose state is
 *  static, with no instances). A task owns its instance; two tasks on
 *  one Tcb<n> is the app's bug, and nothing checks it.
 * The smallest use, a frequency meter on the pin PD0, its period posted
 * to an active object (a component of a brio program with its own event
 * queue, served one event at a time by the kernel; `Display` and
 * `Period` stand for the program's own active object and event type):
 *
 *   using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *   constexpr SysClock clock;
 *   using P = brio::AvrPlatform;
 *   using Meter = brio::FrequencyMeter<brio::Tcb<0>>;
 *   using In = brio::EventChannel<2>;      // PORTC/PORTD pins: channels 2-3 on the DA/DB
 *
 *   ISR(TCB0_INT_vect) { brio::post<Display>(Period{Meter::period_ticks()}); }
 *
 *   int main() {
 *       SysClock::init();
 *       In::source(brio::EvPin<brio::Pin<'D', 0>>{});
 *       Meter::init(clock, In{});          // FRQ mode, CLK_PER, CAPT interrupt on
 *       sei();
 *       brio::Tenuto<P, Display>::run();   // Display converts with Meter::hz(ticks)
 *   }
 *
 * The meter's init() takes the clock tag and keeps its rate for hz(),
 * us() and tick_hz(), so it comes after SysClock::init(); a DynamicClock
 * that does not list the meter is a compile error there. It enables the
 * CAPT interrupt, so the program binds TCBn_INT_vect, and the handler
 * reads the capture: that read is what clears CAPT. A channel a pin's
 * event cannot reach is refused at compile time by source().
 *
 * Which context may call what. Nothing here masks interrupts and nothing
 * holds a caller's buffer; one verb waits, below.
 *  - The configuring verbs - init(), init<cfg>(), the tasks' init(),
 *    init_us() and init_ticks(), snapshot_on(), capture_on(),
 *    capture_on_events(), count_on(), the interrupt enables,
 *    enable()/disable(), PeriodicTick::start()/stop(),
 *    CascadedCounter::reset() - are setup verbs: main(), after the clock
 *    task and before sei(), or an active object's handler. Several are
 *    read-modify-writes of a register with no set/clear twin (CTRLA,
 *    EVCTRL, INTCTRL, and PORTMUX.TCBROUTEA, which holds every
 *    instance's bit), so two contexts racing on one of them can lose a
 *    bit: never call them from an interrupt handler that may interrupt
 *    another such call.
 *  - The interrupt bodies - take_flags(), PeriodicTick::tick(),
 *    Timeout::expired(), OneShotPulse::pulse_done(),
 *    PulseCounter::captured(), CascadedCounter::captured() (Lsb's
 *    vector), FrequencyMeter::period_ticks(),
 *    PulseWidthMeter::width_ticks(), DutyMeter::reading() - are called
 *    from the program's ISR(TCBn_INT_vect) and nowhere else.
 *  - The flag verbs, running(), busy(), pending() and
 *    OneShotPulse::fire() (one store to the software-event register) are
 *    single loads or stores, legal from any context; the meters' hz(),
 *    us(), duty_permille() and tick_hz() are arithmetic over the rate
 *    kept at init() (and moved by rebase()). The 16-bit accesses - count(), compare(),
 *    capture() and the reads above built on them, Pwm8::duty() on the
 *    DA/DB - pass through the instance's one TEMP register: a
 *    main-context access racing an interrupt handler's 16-bit access of
 *    the same TCB corrupts both, so such an access belongs in a critical
 *    section (AvrPlatform::CriticalSection, avrxt/platform.hpp) or in the
 *    handler alone. Pwm8::duty() also read-modify-writes CTRLB: one
 *    context per instance.
 *  - CascadedCounter::read() pulses the snapshot channel and polls both
 *    CAPT flags at most 16 times: a miswired pair returns stale values
 *    instead of hanging. Main context or an active object's handler.
 *  - rebase() is called by the DynamicClock that lists the task, from
 *    its set().
 *
 * Facts that shape the code (24.3, 24.5, errata DS80000915F 2.13.x and,
 * for the DA, DS80000882C 2.12.1):
 *    is the acknowledge; in FRQPW read CNT (period) before CCMP
 *    it is for captures of other signals in phase with the TCA period.
 *    The 16-bit accesses go through the instance's one TEMP register:
 *    reads of CNT/CCMP from the main context while the ISR reads them
 *    too want a critical section (the task bodies below run in the ISR);
 *  - errata DB 2.13.1 (A4/A5) / DA 2.12.1 (every revision): in PWM8
 *    CCMP and CNT are 16-bit only - Pwm8 writes CCMP as one word
 *    (period and duty together); on the AVR EA (DS40002443A ch. 23, the
 *    same IP, no TCB erratum) the two bytes are independent as the
 *    chapter says, and Pwm8 writes CCMPH alone;
 *    PF4, TCB1 PA3 / PF5, TCB2 PC0 / PB4, TCB3 PB5 / PC1, TCB4 PG3 /
 *    PC6 (64-pin parts).
/// CNTMODE, in the datasheet's order (the values are the codes).
    /// table, 17.3.8; whether this package bonds a position is
    /// position this package lacks is compiled out (a runtime `if`
    /// Change the width for the next pulse (takes effect at once: do it
/// interval minus one (the restart cycle is not counted: 24000 ticks
/// half tick lost rounds either way - measure with div1 or a TCA).
/// Range: 65535 ticks of the chosen clock (div2 doubles it;
    /// Setup: FRQ mode on `source`'s event, the counter on `tclk`, the
    /// clock tag's rate kept for hz()/us()/tick_hz(), the CAPT
    /// interrupt enabled - the program binds TCBn_INT_vect. Main, after
    /// the clock task, or an active object's handler.
    /// Interrupt body: the period in ticks of the counter clock, one
    /// added at div1 (the restart cycle); the read clears CAPT.
/// Ratiometric on purpose - not a ClockUser: the duty survives a clock

++ b/brio/avrxt/tcd.hpp
 * The AVR DA/DB 12-bit timer/counter type D (TCD, DS40002247B ch. 25):
 * what a program uses to drive a power stage - a complementary pair of
 * outputs with a dead time between them, fault inputs that act on the
 * pins with no CPU in the path - or to run a counter from the PLL, above
 * CLK_MAIN's 24 MHz. The AVR EA has no TCD: the whole header is gated on
 * the device header's TCD0 (below), so it is includable on every part
 * and names nothing there. It has the two strata of every brio
 * peripheral header, as docs/avrxt/tcd.md describes:
 *  the resource - Tcd<0>: the typed view of the one instance. A config
 *  the three synchronization disciplines of this peripheral (below),
 *  read-L-then-H rule, the strobes, the flags, the two interrupt bodies
 *  and release() - every claimed pin handed back to PORT.
 *  the task - TcdPwm<route>: the complementary output pair with dead
 *  time, the reason this peripheral exists (a task is a driver layer
 *  with state of its own over the resource). One ramp mode, WOA and WOB
 *  (and optionally WOC/WOD mirroring either), a symmetric dead time on
 *  both edges, duty updates through the double buffers. The other usage
 *  not built: they belong with their first real user.
 *
 *  Both are monostates (classes whose state is static, with no
 *  instances).
 *
 * The smallest use, the pair on the default route at 20 kHz:
 *
 *   using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *   constexpr SysClock clock;
 *   using Pwm = brio::TcdPwm<brio::TcdRoute::def>;   // WOA on PA4, WOB on PA5
 *
 *   int main() {
 *       SysClock::init();
 *       const bool ok = Pwm::init(clock, {.clock = brio::TcdClock::clkper,
 *                                         .hz = 20'000, .dead_time_ticks = 60,
 *                                         .auto_update = true});
 *       if (ok) Pwm::duty(Pwm::max() / 4);   // lands at the end of the cycle
 *       ...
 *   }
 *
 * With TcdClock::clkper the counter rate comes from the clock tag, so
 * the clock task runs first, and a DynamicClock that does not list
 * TcdPwm is a compile error at init(); with another source the program
 * states the rate in source_hz and nothing checks it (the PLL is started
 * first, avrxt/clock.hpp). TcdPwm::init() claims PA4 and PA5 as outputs
 * and enables no interrupt; a program that enables TCD0_OVF or TCD0_TRIG
 * on the resource binds the vector with ISR() to ovf() or
 * take_triggers().
 *
 * Which context may call what. Nothing here masks interrupts and
 * nothing holds a caller's buffer.
 *  - The configuring verbs - init(), init<cfg>(), release(), the
 *    discipline-3 setters, fault_control(), the interrupt enables,
 *    input_a_on/input_b_on and their _off twins, and TcdPwm's init(),
 *    start(), stop(), stop_at_end() and release() - are setup verbs:
 *    main(), after the clock task and before sei(), or an active
 *    object's handler (an active object is a component of a brio
 *    program with its own event queue, served one event at a time by
 *    the kernel). INTCTRL, CTRLA and PORTMUX.TCDROUTEA are
 *    read-modify-written with no guard: never from an interrupt handler
 *    that may interrupt another such call.
 *  - The verbs of disciplines 1 and 2 wait: each polls STATUS.ENRDY or
 *    STATUS.CMDRDY at most `spins` times (tcd_sync_spins, 65535, by
 *    default) and returns false when the window never opened, a CLK_TCD
 *    source that is not running being the usual cause. On CLK_PER both
 *    windows reopen within one further read (measured,
 *    docs/avrxt/tcd.md); on a slow CLK_TCD the wait is that clock's.
 *    The compare and dither setters, TcdPwm::duty(), sync() and
 *    sync_at_end() are among them; a sequence of them (TcdPwm::duty()
 *    writes two or three compares) is not atomic against another caller,
 *    so one context drives an instance's compares.
 *  - The interrupt bodies - ovf() for TCD0_OVF_vect, take_triggers() for
 *    TCD0_TRIG_vect - are called from the program's ISR() for that
 *    vector and nowhere else; each clears its flags with one store.
 *  - The flag verbs and the 8-bit read-backs are single loads or
 *    stores, take_pwm_activity() a load and a store, all legal from any
 *    context. The 12-bit read-backs read a low byte then a high byte,
 *    and capture_a() and capture_b() do the same with the high read
 *    releasing the buffer, so each capture register has one reader.
 * asynchronous to CLK_PER:
 *  3. The static registers (CTRLB, CTRLC, CTRLD, EVCTRLA/B, INPUTCTRLA/
 *     while the TCD is enabled. The setters enforce it: they return
 *    0x0FFF. CAPTUREA/B are read low byte first, and the read of the
 *    high byte is what releases the buffer for the next capture - the
 *    silicon: 16-24 MHz in, 32-48 MHz out, and it runs only while a
 *  - input mode validity (table 25-5) is a real constraint, not advice:
 *  - dithering is not supported in dual slope (25.3.3.5): refused;
 *  - errata 2.14.1 / 2.13.1 (DB A4/A5, DA every revision): asynchronous
 *    input events (CFG = ASYNC) miss events when CNTPRES is not DIV1.
 *    It is fixed in DB rev. B0, so it is not refused here - it is
 *  - errata 2.14.2 / 2.13.2 (DB A4/A5, DA every revision): on any
 *    alternate route, CMPAEN in FAULTCTRL gates all four outputs - a
 *    but "also set CMPAEN". Rev-dependent again, so it is stated here
 *    and in docs/avrxt/tcd.md and not refused (TcdPwm always enables
 *    WOA);
 *  - errata 2.14.3 / 2.13.3 (every revision of both families): input
 *    work with CMPASET = 0 or in dual slope. Both are refused here, at
 *      ALT1     PB4 PB5 -   -     48-pin (WOC/WOD pinless)
 *      ALT2     PF0 PF1 -   -     28-pin (WOC/WOD pinless)
 *  - the TCD has two interrupt vectors, TCD0_OVF and TCD0_TRIG, and
 *  - STATUS.PWMACTA/B are toggle detectors: hardware sets them on every
 *    pin - and what they watch is the waveform generator behind the
 *    fault override, not the pad (measured, take_pwm_activity() below).
/// TCD pin routing (PORTMUX.TCDROUTEA). The values are the device
/// Where one output of one route sits, and whether this package bonds
/// The position of one output of one route on this package, with the
/// CTRLA.CLKSEL: what CLK_TCD is. The values are the header's codes.
/// CTRLA.SYNCPRES: divides CLK_TCD into the synchronizer clock, which
/// CTRLA.CNTPRES: divides the synchronizer clock into the counter
/// clock. Errata DB 2.14.1 / DA 2.13.1: anything but DIV1 loses
/// asynchronous input events on the affected revisions (DB A4/A5, every
/// DA) - divide with SYNCPRES instead.
/// values are the header's codes.
/// EVCTRLx.ACTION: a fault only, or a fault and a counter capture.
/// DLYCTRL.DLYSEL: the one function the delay block serves. Input
/// Extra TCD clock cycles added to the cycle when the dither
/// is taken out of the following output state instead, so the cycle
/// slope on every silicon revision of both families.
///  - every enabled output must have a bonded pin on that route;
/// Deliberately not refused: errata 2.14.1 (async events with a counter
    /// stay off: the task or the app enables them. False also when the
    /// disable's ENRDY window never opens (nothing written) and when the
    /// final enable's does not (everything written, the TCD left off);
    /// with cfg.enable false, true once written. Setup context.
    /// route back to DEFAULT, so nothing init() set up is left behind.
    /// Setup context; false when disable() did not complete (one of its
    /// ENRDY waits ran out), the rest done anyway.
    // CMPBCLR's high byte is itself the end-of-cycle synchronization
    // auto-updating application calls last.
    // Read low byte then high byte: the read of the high byte is what
    // Each of these returns false and writes nothing while the TCD is
    /// are also driven as PORT outputs by claim_pins()/init().
    /// Read and clear in one verb, so "did this waveform move since I
    /// Measured: the chapter says "each time the WOx output toggles",
    /// and the pad is not what it watches. With an input mode holding
    /// and PWMACT keeps setting - it follows the waveform generator
    /// behind the fault override. The W1C clear itself is sound (with
    /// the TCD disabled the bits stay down). To ask about the pad, read
    /// the pin.
    /// this package lacks is compiled out (a runtime `if` would
    /// The rate of the selected clock source, before the prescalers.
    /// The dead time inserted on both edges, in counter ticks.
 * Not a PwmChannel: `max` here is a run-time value (period minus the two
    /// resource refuses the configuration - Tcd<0>::init(), whose false
    /// includes an ENRDY window that never opened. Setup context, after
    /// the clock task; it claims WOA, WOB and the asked WOC/WOD pins.
    /// sync_at_end(). False when v exceeds max() or when a CMDRDY wait
    /// (tcd_sync_spins polls before each of the two or three compare
    /// writes) runs out. The writes are not atomic against another
    /// caller: one context drives the duty.

        // With AUPDATE the write of CMPBCLR's high byte is the request,
++ b/brio/avrxt/ticker.hpp
 * The time of a brio program on the AVR DA, DB and EA: a counter that
 * the RTC's periodic interrupt timer (PIT) advances 1024 times a second.
 * It is the kernel's timebase - AvrPlatform::now() (avrxt/platform.hpp)
 * returns Ticker::ticks(), and every time event a program arms counts
 * these ticks - and it also gives a program milliseconds, seconds and a
 * timestamp to read. A program does two things with it: it binds the
 * PIT's interrupt vector to pit(), and it calls init() once at start-up.
 * The smallest use, as the maintained example `blink` spells it:
 *
 *     using P = brio::AvrPlatform;
 *     using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *     constexpr SysClock clock;
 *
 *     ISR(RTC_PIT_vect) { brio::Ticker::pit(); }
 *
 *     int main() {
 *         SysClock::init();
 *         brio::Ticker::init();     // RTC clock select, PIT period, its interrupt on
 *         sei();
 *         brio::Tenuto<P, Blinker, Supervisor>::run();   // blink's two active objects
 *     }
 *
 * `ISR(RTC_PIT_vect)` is avr-libc's way of writing the interrupt handler
 * of that vector; `brio::Ticker` is `BasicTicker<1024>`, the alias at the
 * bottom of this file; `init()` picks the RTC's clock, programs the PIT
 * period for 1024 Hz and enables the PIT interrupt; `sei()` lets the first
 * interrupt in; from then on every PIT interrupt runs pit(), which
 * advances the counters, and the kernel loop that run() enters reads
 * them through P (Blinker and Supervisor are blink's active objects:
 * components of a brio program with their own event queues, served one
 * event at a time by the kernel, docs/learn/introduction.md). init()
 * enables the interrupt and does not check that the vector is bound:
 * the binding is the program's. A vector left unbound is avr-libc's
 * __bad_interrupt, a `jmp 0` in the release listings, so the first tick
 * restarts the program from its first instruction, without a hardware
 * reset; a handler that does not call pit() leaves the counters still,
 * and no time event ever fires.
 *
 * Who calls what, and from where:
 *  - init() and init(source) are setup: main(), after the clock's
 *    SysClock::init() and before sei(). They reset the counters and wait
 *    for the RTC's busy flags (avrxt/rtc.hpp's rtc_wait and rtc_pit_wait,
 *    each at most 0xFFFF polls and a last read; the flags clear in about
 *    2.8 CLK_RTC periods, measured, docs/avrxt/rtc.md). A wait that runs
 *    out (CLK_RTC not running) is not reported: init() writes on;
 *  - pit() is the interrupt body, called from the program's
 *    ISR(RTC_PIT_vect) and from nowhere else; it runs to completion with
 *    no other handler interrupting it, because no vector is raised to
 *    CPUINT's level 1 (the mechanism is avrxt/platform.hpp's), and it
 *    waits for nothing;
 *  - ticks(), millis(), secs() and now() may be called from any context:
 *    each masks interrupts (ATOMIC_BLOCK with ATOMIC_RESTORESTATE, so the
 *    previous state comes back) for the few loads that copy a counter;
 *  - pause() and resume() are one store each to PITINTCTRL: the interrupt
 *    off and on, the PIT itself (PITEN) left running.
 * The tick is as accurate as CLK_RTC, which is the internal OSC32K
 * unless init() finds a 32.768 kHz crystal running (init() below says
 * what "running" takes); that oscillator's
 * error, measured per die, is in docs/avrxt/rtc.md.
 *
 * Why a monostate: a monostate is a class whose state is static, with no
 * instances. Here every data member is a `static inline` (one shared
 * copy, no instances, header-only) and every method is static, so the
 * ISR body reaches its counters with direct lds/sts on absolute
 * addresses instead of loading a pointer first, there is no lazy-init
 * branch or guard to pay for, and the class needs no translation unit
 * of its own.
 * is the tick arithmetic above them, and it owns the block's one clock
 *  - millis(): approximate milliseconds, long-term exact against the tick
 *              (wraps ~49.7 days); the tick itself is as accurate as
 *              CLK_RTC, above.
 * every second. The ISR therefore skips the millis increment at positions
 * pit() is the ISR body and runs to completion: the AVRxt core does not
 * clear the I bit on entering a handler (DS40002247B and DS40002443A
 * 7.6.3), and what keeps another handler out is CPUINT - every vector at
 * level 0, none raised to level 1, an NMI aside (avrxt/platform.hpp).
     * The Ticker owns the RTC block's one clock select (rtc.hpp): it
     * takes what RtcClock::preferred() names - XOSC32K when a 32k crystal
     * was started and is still enabled and running, the internal OSC32K
     * otherwise. "Running" is XOSC32KS, which a crystal nothing requests
     * yet sets only when it was started forced on (Xosc32k's run_standby
     * argument, avrxt/clock.hpp): one started without it, or still in its
     * start-up, is passed over in silence for OSC32K. Resets all counters, waits for the RTC's pending writes
     * (bounded, the answer not checked), programs the PIT period for
     * ticks_per_second and enables its interrupt. The RTC counter half
     * (brio::Rtc) is left alone and shares whatever source is selected
     * here.
     * Setup: call once in main(), after clock init and before sei(). The
     * RTC_PIT_vect ISR must call pit().
    /// 32.768 kHz CLK_RTC, so OSC1K divides every rate by 32 - and
    /// ticks_per_second, millis() and secs() do not follow: they keep
    /// counting as if CLK_RTC were 32.768 kHz, so kernel time runs 32
    /// times slow on OSC1K.
     * PIT interrupt body - call from ISR(RTC_PIT_vect) and from nowhere
     * else.
    /// Raw 32-bit tick count since init() (wraps: 49 days @ 1024 Hz);
    /// what AvrPlatform::now() returns. Any context.
++ b/brio/avrxt/twi.hpp
 * The AVR DA/DB/EA two-wire interface (DS40002247B ch. 29, DS40002443A
 * ch. 27) as an I2C bus for a program: a host that talks to sensors,
 * converters and memories, a client that answers a host on another
 * chip, or both at once on one instance. The header has the two strata
 * every brio target's peripheral header has (docs/avrxt/twi.md is the
 * document):
 *  the resource - Twi<n>: the typed view of one instance, a class whose
 *  state is static, with no instances. A config struct owns the whole
 *  configuration (route, the shared CTRLA knobs, the host half, the
 *  client half, the dual-mode client); init<cfg>() folds it and refuses
 *  at compile time what this package cannot bond, init(cfg) computes it
 *  at run time and returns false instead. Then the verbs of the
 *  chapter's register description: CTRLA, DUALCTRL, DBGCTRL,
 *  MCTRLA/MCTRLB/MSTATUS/MBAUD/MADDR/MDATA on the host side,
 *  two interrupt bodies (one per vector), and release().
 *  the tasks - what an application names:
 *    I2cHost<n, route>    the transfer engine: one Request = one bus
 *                         surface and the interrupt body.
 *  Host and client of one instance may run together - combined mode
 *  (same pins) or dual mode (the client on the route's second pin pair).
 * The smallest use, a host on TWI0's default route (PA2 SDA, PA3 SCL)
 * under the bus arbiter of util/i2c_bus.hpp - an active object, a
 * component of a brio program with its own event queue that the kernel
 * serves one event at a time - which queues the requests of every
 * active object sharing the bus and answers each with an I2cDone:
 *
 *   using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *   constexpr SysClock clock;
 *   using P = brio::AvrPlatform;
 *   using I2cHw = brio::I2cHost<0>;
 *   using I2c = brio::I2cBus<I2cHw, P>;
 *
 *   ISR(TWI0_TWIM_vect) {
 *       if (I2cHw::isr()) brio::post<I2c>(brio::TransferDone{I2cHw::status()});
 *   }
 *
 *   int main() {
 *       SysClock::init();
 *       I2cHw::init(clock);              // the default Options: 100 kHz, Smart mode
 *       brio::Ticker::init();
 *       sei();
 *       brio::Tenuto<P, Me, I2c>::run();
 *   }
 *
 *   // in a handler of the active object Me: write a register index,
 *   // read two bytes back after a repeated START
 *   brio::post<I2c>(I2cHw::Request{addr, brio::lend<brio::Lease::reply>(tx), 1,
 *                                  brio::lend<brio::Lease::reply>(rx), 2,
 *                                  brio::reply_to<Me, brio::I2cDone>()});
 *
 * The reply's status is i2c_ok or the code of what went wrong
 * (util/i2c_bus.hpp). The bus's pull-ups are the board's: the driver
 * enables no internal pull-up.
 *
 * Which context calls what. The init() verbs (Twi<n>::init,
 * I2cHost::init, I2cClient::init) and release() are setup: main(), after
 * the clock task - MBAUD is computed from the rate the clock type
 * claims, never read from the hardware - and before sei(); none of them
 * waits. I2cHost::start() is the arbiter's, called from its handler in
 * the main context, and never waits: a START told to go out on a Busy
 * bus is held by the silicon and leaves by itself when the bus goes
 * Idle, and a request whose speed is not the one in force pays an
 * ENABLE cycle there, between tenures. I2cHost::isr() is the body of the
 * host vector (TWIn_TWIM_vect) and I2cClient::isr() of the client vector
 * (TWIn_TWIS_vect): the program's ISR calls it and nothing else does,
 * and like every handler in brio it runs to completion, no other
 * handler interrupting it (avrxt/platform.hpp). The client's protocol
 * verbs (respond, receive, transmit, complete) answer a flag the client
 * half raised, from its handler or from a polled loop; wait_address()
 * and wait_data() spin on that flag for at most `spins` turns of their
 * loop (a count, not a time). recover() is an ENABLE cycle and returns
 * at once; unstick() busy-waits through its bit-banged pulses, at least
 * 10 us a pulse and 20 us for the closing STOP, with both halves down,
 * so it belongs between tenures in the main context. Nothing in this
 * header masks interrupts: the read-modify-write verbs on CTRLA, MCTRLA
 * and SCTRLA carry no guard, and the interrupt bodies write none of
 * those three registers, so only two callers of those verbs in two
 * contexts can race. Buffers: a Request lends its tx and rx
 * (Lease::reply) until its I2cDone lands - the host interrupt reads tx
 * and writes rx meanwhile - and nothing checks at run time that the
 * requester leaves them alone.
 *
 * The two names are deliberate. The resource is named for the chapter -
 * this silicon calls the peripheral a TWI - and the tasks for the bus
 * they speak. I2cHost and I2cClient are spelled the same on every
 * target, so an application reads unchanged when it moves; the resource
 * stays in its own silicon's words, where a datasheet in hand is what a
 * reader needs. docs/design/i2c-bus.md holds the contract the tasks
 * implement.
 *  - on the AVR DA/DB, TWI0 exists on every package and TWI1 appears at
 *    32 pins (the AVR EA has TWI0 alone, below). Unlike SPI and USART
 *    the TWI has no pinless route: every PORTMUX code names a real
 *    SDA/SCL pair, so an instance always costs two pins. The route table
 *    below is the device headers' own route enums, per package;
 *  - each route also names a dual pair - where the client sits when
 *  - errata as code, all three of them:
 *        enabled the pin override covers the driver but not the value -
 *        expose FLUSH at all - recover() is the documented work-around
 *        300 ns selections are swapped in the silicon. TwiSdaHold names
 *        true nanoseconds and twi_sdahold_code() swaps the encoding on
 *  - recover() and unstick() fix different things and neither replaces
 *    the other: recover() puts this peripheral back into a known state,
 *    unstick() bit-bangs the wire free of a client that is holding SDA
 *    low (the classic nine-clocks-and-a-STOP remedy). Noticing a stuck
 *    transaction is the bus active object's: util/bus_master.hpp's
 *    per-bus timeout_ticks, when it is not 0, answers the requester
 *    bus_timeout and calls recover() in its place. Noticing a stuck wire, and deciding to
 *    spend unstick() on it, stays the application's (design/i2c-bus.md);
 *    500 ns). The bus's rise and fall times are arguments, because that
 *    the speed back. Charging nothing for the fall is the one unsafe
 *    BAUD + 5 clocks minus the fall, so it would land below the floor by
 *    exactly tOF (measured: 166 ns at Sm and Fm on the AVR128DB48's bus,
 *    docs/avrxt/twi.md). MBAUD "must be written while the host is
 *    disabled" (29.5.7), so set_baud() performs the ENABLE cycle and
 *    re-declares the bus idle;
 *  - FMPEN is what Fast-mode Plus means to the pins (x10 drive instead
 *    interrupt (PIEN, 29.5.10). The host side asks the stronger question
 *    and enforces it: speed_ok() says whether twi_baud_for() has a BAUD
 *    for the speed at the clock and timing in force, and since the
 *    fastest the divider goes is CLK_PER/10 a reachable speed meets the
 *    four-times condition; the host engine answers a request at an
 *    unreachable speed i2c_rejected. The client keeps the condition as a
 *    readback (max_scl_hz(), can_follow()) and does not enforce it: a
 *    slower main clock is legal there, it only blinds those two
 *    features;
 *  - the AVR EA (DS40002443A ch. 27: the DB's register description but
 *  - two vectors, two interrupt bodies: TWIn_TWIM_vect (host: RIF/WIF)
 *    and TWIn_TWIS_vect (client: DIF/APIF). Neither body clears
 *    anything - the host's flags clear when the handler writes
 *    MADDR/MDATA/MCMD and the client's when it writes SCMD or touches
 *    SDATA (29.5.6, 29.5.12), so a body that cleared them would eat the
 *    state its caller has to read.
/// TWI pin routing (PORTMUX.TWIROUTEA). The values are the device
/// Where one signal of one route sits, and whether this package bonds
/// The main pin pair of a route - where the host, and a client that is
/// The dual pin pair of a route - where the client sits when
/// Does this package bond both dual pins of this route? (What a Dual
/// CTRLA.SDASETUP: how many peripheral clocks the client stretches SCL
/// of this driver is eight, not the reset value's four: above 16 MHz four
/// client whose SDA must fall as it releases its stretch (it holds SDA
/// CTRLA/DUALCTRL.SDAHOLD, named in true nanoseconds. The DA silicon
/// swaps the 50 ns and 300 ns selections (DS80000882C 2.14.2) and
/// CTRLA.FMPEN as well - it is what changes the pads (x10 drive instead
/// The rise-time budget the BAUD calculation charges to the bus: the
/// with stiff pull-ups, which makes the real SCL run a little above the
/// The fall-time budget the calculation charges to the bus: the I2C
/// low time the pins really show is the register's (BAUD + 5) clocks
/// minus the fall (equation 29-4) - charge nothing for it and the low
/// The SCL low phase the host drives for this MBAUD at a speed, in
/**
 * MBAUD for a speed at a peripheral clock - the chapter's own three-step
 * calculation (29.3.2.2.1), with the bus's rise and fall times as
 * arguments because that is what the equations take (on the AVR DA/DB;
 * the AVR EA's own duty takes the branch the comment inside describes):
 *   1. equation 29-3, BAUD = f_CLK/(2 f_SCL) - (5 + f_CLK tR / 2);
 *   2. equation 29-4, tLOW = (BAUD + 5)/f_CLK - tOF;
 *   3. if that tLOW is below the mode's floor, equation 29-5,
 *      BAUD = f_CLK (tLOW_mode + tOF) - 5.
 *
 * `rise_ns` and `fall_ns` describe this bus; 0 means "charge the mode's
 * specification maximum", which is the safe default and the only choice
 * a driver can make on its own. Declaring the real numbers of a stiff
 * bus buys speed back: they only ever appear as terms that lengthen the
 * period, so a smaller pair means a faster bus - and the floor check of
 * step 3 keeps it legal. The rounding is deliberately one-sided: the
 * first term and step 3 round up, the rise term down, all three of
 * which lengthen the period.
 *
 * Empty when the register cannot hold the solution: a BAUD below 0
 * (the divider cannot go that fast at this clock - even BAUD = 0 gives
 * a slower SCL than asked, and a speed is never approximated in
 * silence) or above 255 (nor that slow). Empty too at a clock of 0,
 * and for Fm+ at a CLK_PER at or below twi_fm_plus_min_clk_hz where
 * that floor is not 0 (the AVR EA's 12 MHz).
 */
/// The SCL period this MBAUD produces, in peripheral clock ticks, on a
    // CTRLA - shared by the host and by a client that is not in Dual mode.
    bool quick_command = false;         ///< QCEN: the address packet is the transaction
    bool second_address = false;        ///< ADDREN: the mask field is a second address
    // DUALCTRL - the client on the route's second pin pair, with its own
    /// The shared knobs. The chapter wants them written before the
    /// The hold time in force, in true nanoseconds (the DA swap undone).
    /// CTRLA.FMEN, the AVR EA's Fast-mode duty cycle for the host
    /// Dual mode: the client moves to the route's second pin pair and
    /// MDATA is read, so a receiving handler needs no MCMD at all.
    /// ACKACT and MCMD in one store - the register description says they
    // ---- the stuck wire (not the peripheral) -------------------------------
     * The classic bus recovery, and the one thing recover() cannot do.
     * the wire: a client interrupted in the middle of a byte (a reset
     * again, and the bit-bang is open drain by hand: DIRSET over a clear
     * The policy above this is split. Noticing that a transaction is
     * stuck is the timed arbiter's (util/bus_master.hpp's timeout_ticks,
     * which calls recover(), not this); deciding to spend this on the
     * wire is the application's and is not built (design/i2c-bus.md).
    /// The last acknowledge bit the client sent: true = NACK.
    /// Write-one-to-clear, as a plain store of exactly the named bits -
    /// MDATA is the shift register itself: a write commands a byte
    /// transmit, a read takes the received byte (and, in Smart mode,
    /// PMEN: answer every address on the bus (29.3.2.3).
    /// The last acknowledge bit the host sent: true = NACK.
    /// match logic ignores - an address range.
    /// SADDRMASK with ADDREN = 1: a second exact address.
    /// What a host interrupt found. Nothing is cleared here: RIF/WIF
    /// What a client interrupt found. Same discipline: DIF and APIF
    /// not is refused by the host engine, never approximated: the
    /// it is the pad half of Fm+).
    /// Errata DB 2.15.1 / DA 2.14.1: with the TWI enabled the pin
    /// override covers the driver but not the value, so a PORTx.OUT bit
 * The transfer engine: the target-side half of the I2C stack, driven by
 * Transaction descriptor (Request) - one bus tenure, from START to STOP,
 * in the four shapes I2C devices actually use:
 * register index, repeated START, read the value) and it must be one
 * i2c_arb_lost, i2c_bus_error, and i2c_rejected for a request whose
 * speed the divider cannot make (start() below). i2c_nack_addr is any
 * address nobody acknowledged - a write's, a read's, the repeated
 * START's - and i2c_nack_data a written byte refused. On any NACK the
 * engine still issues the STOP so the bus is released; on arbitration
 * lost the bus belongs to the other host and no STOP is sent; on bus
 * error the peripheral is forced back to the idle bus state.
 * change costs an ENABLE cycle and a force-idle - paid at start(), only
 * A stuck transaction is the bus active object's to notice:
 * util/bus_master.hpp's timeout_ticks, when it is not 0, answers the
 * requester bus_timeout in its place and calls recover() below. A stuck wire is a different fault and the remedy is
    /// One bus tenure, posted to the arbiter. The speed is the tenure's
    /// own and defaults to Sm: a request that names none runs at 100 kHz
    /// whatever init() programmed (start() reprograms MBAUD when it
    /// differs from the speed in force).
        /// Bytes written after START, lent until the reply lands (may be
        /// lent until the reply lands.
    /// the wire, not of one transaction.
        /// The rise and fall times of this bus in nanoseconds; 0 charges
        /// MCTRLA.SMEN: the ACK rides the MDATA read (29.3.3.3). On by
        /// per byte (the benchmark skeleton's letter i, docs/avrxt/twi.md).
        /// A write is the same either way: the action is not taken on a
        /// data write.
     * Host, interrupt driven: routes the pins, programs MBAUD for
     * o.speed at the clock's rate, enables the host with both its
     * interrupts (RIEN, WIEN) and declares the bus idle. Setup: call
     * after the clock task's init, before sei(). The
     * be reached at this clock. When a client of this instance is
     * already enabled (combined mode), only the host half and the shared
     * CTRLA knobs are written.
    /// The bus must be idle - the rewrite costs an ENABLE cycle.
    /// The per-byte state is set up here, once: a cursor into the phase's
    /// Outcome of the last transaction: an i2c_* code of util/i2c_bus.hpp,
    /// valid once isr() returned true, or at once when start() returned
    /// true (i2c_rejected). The program's host handler posts it to the
    /// arbiter as TransferDone{status()}.
     * One interrupt per byte is the silicon's (29.3.2.2.4, 29.3.2.2.5):
     * access is time the bus waits, every byte. The two common cases are
            // read NACK before MDATA is read - with ACK standing the read
    /// The other half of recovery: the wire, not the peripheral. See
 * the General Call address (SADDR bit 0), an address range (SADDRMASK
 * with ADDREN = 0 masks the bits to ignore), a second exact address
 * `on_dual_pins` puts the client on the route's second pin pair through
 * Combined mode (a host and a client on the same instance and the same
 * knobs are then one register shared by the two halves and the later
        uint8_t address_mask = 0;           ///< bits to ignore, or a second address
        bool second_address = false;        ///< ADDREN: address_mask is a second address
    /// host shares. False when the package cannot bond the pins. Setup:
    /// after the clock task's init, before sei(); when the host of this
    /// instance is already enabled (combined mode), only the client half
    /// and the shared knobs are written.
    /// What the last address packet asked for: true = the host reads.
    /// with PMEN or a mask this is the only way to learn which address
    /// The last acknowledge bit the host sent (valid after a transmit):
    /// Interrupt body (TWIn_TWIS_vect): called from the program's ISR,
    /// it returns the SSTATUS it found for the handler to decide on.
    /// Clears nothing: DIF and APIF clear when the handler responds
    /// (SCMD) or touches SDATA.
++ b/brio/avrxt/usart.hpp
 * The USART of the AVR DA/DB (DS40002247B ch. 27) and of the AVR EA
 * (DS40002443A ch. 25, the DB's chapter register for register under a
 * route table of another shape) as a serial port for a program: the
 * console a program prints to and reads lines from, a byte transport to
 * another chip, or one of the other shapes the peripheral offers (a
 * one-wire half-duplex line, RS-485, a synchronous link, a host SPI,
 * IrDA, auto-baud). docs/avrxt/usart.md is the document. The header has
 * two strata:
 *  the resource - Usart<n>: the typed view of one instance. A config
 *  struct owns the whole configuration (route, communication mode, frame
 *  to-clear discipline, the interrupt enables, the interrupt bodies
 *  the tasks - what an application names, each a mode with its knobs in
 *  a configuration field - the errata below say why.
 * The smallest use, the console on USART2's ALT1 route (TXD PF4, RXD
 * PF5), in the shape the apps spell it:
 *
 *   using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *   constexpr SysClock clock;
 *   using Serial = brio::Uart<2, brio::Route::alt1>;
 *   constexpr Serial serial;                 // the tag print() takes
 *
 *   ISR(USART2_RXC_vect) { Serial::rxc(); }
 *   ISR(USART2_DRE_vect) { Serial::dre(); }
 *
 *   int main() {
 *       SysClock::init();
 *       Serial::init(clock, 460800);
 *       sei();
 *       brio::print(serial, "hello", brio::crlf);
 *   }
 *
 * The route brings its pads: init() writes PORTMUX and the pins'
 * directions itself. The baud divisor is computed from Clock::hz, the
 * rate the clock type claims, so init() runs after the clock's; and it
 * is not checked there - a rate the register cannot express writes
 * BAUD 0, and can_baud() is the check. Both vectors are the program's
 * to bind: an unbound one is avr-libc's __bad_interrupt, a jump to
 * address 0. A console that assembles lines posts on rxc()'s answer
 * (docs/avrxt/usart.md, "How to use it").
 *
 * Which context calls what.
 *  - Setup, in main() after the clock's init() and before sei(): every
 *    init() and release(), of the resource and of the tasks. Uart::init()
 *    busy-waits 10 ms (delay_us) holding TXD idle; the resource's init()
 *    waits for nothing.
 *  - Interrupt bodies, called from the program's ISR() for the vector
 *    and from nowhere else: Uart::rxc() (USARTn_RXC_vect) and Uart::dre()
 *    (USARTn_DRE_vect). The resource names receive_as() and clear_txc()
 *    as the bodies of its RXC and TXC vectors for a program that drives
 *    it from interrupts; the polled verbs read the same registers from
 *    main context where no vector is bound to them. A handler runs to
 *    completion, no other handler interrupting it (avrxt/platform.hpp).
 *  - Main context - setup, or the handler of an active object (a
 *    component of a brio program with its own event queue, served one
 *    event at a time by the kernel) - and never an interrupt handler:
 *    Uart's write_byte(), write_bulk() and print() over them, read_byte(),
 *    read_span() with consume(), rx_pending(), tx_idle(), rx_skips(),
 *    rebase() and set_baud(). Each ring has one producer and one
 *    consumer, and the interrupt body is the other end of both; nothing
 *    checks for a second one. The error counters are single bytes,
 *    readable from any context.
 *  - What waits: Uart's transmit and receive verbs never do (print()
 *    spins on a full ring, util/print.hpp). Uart::rebase() and
 *    Uart::set_baud() spin, with no bound, until tx_idle(): the ring is
 *    drained by dre(), so they are called with interrupts enabled once
 *    anything was queued. The resource's send(), wait(), wait_line_idle()
 *    and the polled tasks' verbs over them spin a bounded count (500'000
 *    turns by default) and answer false or an empty optional when it
 *    runs out.
 *  - What masks: with ring sizes of at most 256 (the defaults) the rings
 *    take no critical section on this core (the comment at Uart's ring
 *    defaults). write_byte() and write_bulk() set DREIE with a
 *    read-modify-write of CTRLA that dre() may interrupt; DREIE is the
 *    only bit either side changes there and the main side's store sets
 *    it, so an interleaving costs one DRE entry that finds the ring
 *    empty and clears it again. The resource's knob verbs are
 *    read-modify-writes of CTRLA/CTRLB with no guard: setup verbs, one
 *    context at a time.
 *  - Buffers: the rings are the task's own static arrays. write_bulk()
 *    copies the caller's span, free again when it returns; read_span()
 *    hands out the ring's own bytes, valid until consume() releases them.
 *
 * Facts that shape the code (DS40002247B 27.3, 27.5; errata DS80000915F
 * 2.16.x for the DB, DS80000882C 2.15.x for the DA, DS80001048D 2.5.1
 * for the EA):
 *    FIFO and the read of one of the two data registers shifts the
 *    plain store of just that bit (an RMW would read every pending
 *    hands TXD back to PORT as an input; disabling the receiver is
 *    config - is to leave TXD's direction at input and let the pull-up
 *    automatically set to output by hardware". DS80001048D has no twin
 *    of it; the work around is applied on every part;
 *    the transition to standby. DS80001048D has no twin of it either;
 *    the verb pair stands on every part;
 *  - errata 2.16.3 (auto-baud; DS80001048D 2.5.1 on the EA, word for
 *    word): once ISFIF is set the receiver is dead and clearing the
 *    flag does not revive it - RXEN must be written 0 then 1. Every
 *    auto-baud path here goes through recover_from_isf();
 *  - the routes are PORTMUX.USARTROUTEA/B, a field per instance - two
 *    bits on the AVR DA/DB, three for the EA's USART0 (DS40002443A
 *    17.3.3-4) - with a NONE code that disconnects every pin: an
 *    instance still runs pinless (the baud generator, the transmitter's
 *    flags and its timing are all live) but LBME and ODME are not
 *    available there - both act on the TXD pad, and a pinless loop-back
 *    receives nothing (bench). Which positions a package bonds is the
 *    device header's route enums - the tables below are those enums,
 *    package by package, and the EA's route table of its data sheet.
 *    running before main): hardware is touched only by the explicit
 * The empty default constructor is intentionally available: an instance
 * TX policy: write_byte() has try semantics (false when the TX ring is
 * block; print.hpp blocks), and write_bulk() is the same for a run (a
 * buffer and a length, the unit a transport moves): as
 * DREIE set behind each. tx_idle() is the wire's: nothing queued and the
 * full, byte lost) is counted, as are the hardware error flags.
     * Configure pins, PORTMUX, baud rate and enable the USART: 8N1 on
     * `route`, both directions on, the rings and the error counters
     * emptied, then TXD held idle for 10 ms (a busy-wait) before the
     * first frame can start.
     * Call after the main clock is set up and before sei(); `clock` is
     * divisor folds to a constant. The rate is not checked: one the
     * register cannot express makes usart_baud_reg() 0, and that 0 is
     * written into BAUD - can_baud() is the check. The RXC interrupt is
     * enabled here; the DRE interrupt is enabled on demand by
     * write_byte() and write_bulk().
     * a half-changed rate; a byte being received during the switch may
     * be garbled - the caller picks a quiet moment. Main context only, and
     * the wait has no bound: dre() drains the ring, so interrupts are on.
     * Called from the program's ISR for the vector and nowhere else.
     *
     * One entry takes every frame the buffer holds. The receive buffer is
     * Nothing is cleared beside the frame: the error flags travel with
     * before it, and it is delivered. Every loss is reported to the ring -
     * first byte, and a line's end is the console's terminator. A program
     * that reads no input binds the vector to rxc() and discards the
     * answer; one that does posts on it to the active object that
     * assembles lines (util/serial_port.hpp's SerialPort).
     * Called from the program's ISR for the vector and nowhere else.
     *
     * drains (write_byte() and write_bulk() re-enable it). The run's last
     * byte also clears TXCIF, written after the byte is in TXDATA: the flag rises
     * Clearing it before the write would race the previous frame's end
    /// The first byte goes before the copy: it is pushed and DREIE set
    /// while the rest is copied. The rest goes in parts of at most
    /// The received bytes in place: the contiguous run ready to be read,
    /// The wire is idle: nothing queued, and the last byte's stop bit
    /// never cleared (modulo 2^8): util/stream.hpp's SkippingSource, the
++ b/brio/avrxt/userrow.hpp
//  What it does for a caller: board_id() returns the label written into
//  the User Row when the board was provisioned, so a program - every
//  test_avr_* suite's banner - can say which physical board it runs on.
//  The smallest use:
//
//      auto board = brio::board_id();      // "brio-a", or empty
//      if (board.empty()) board = "?";
//
//  Who calls it: any context, at any time. It reads the memory-mapped
//  row with plain loads, at most one row's length of them, waits for
//  nothing and masks nothing. The view it returns points into the row
//  itself, not into a copy: it stays valid for the life of the program,
//  and shows other bytes only if the CPU rewrites the row (on the AVR
//  DA/DB, Nvm::userrow_write or Nvm::userrow_erase).
//
//  that a chip erase does not touch: whatever is written there survives
//  every reflash of the part. brio uses it to name the physical board -
//  identical chips on identical boards are otherwise indistinguishable
//  once the USB cabling is in doubt.
//  (avrdude -U userrow:w:0x62,...,0x00:m) and this header only reads
//  written with the flash commands, byte by byte, the erase taking all
//  32 bytes at once (avrxt/nvm.hpp: Nvm::userrow_write,
//  userrow_write_block, userrow_erase) - it is not something an
//  identity label should be reached through; on the AVR EA the CPU's
//  USERROW write is declined with its flash self-programming (the row
//  is written through the flash's page buffer, avrxt/nvm.hpp refuses the
//  verbs by name; docs/avrxt/nvm.md), Nvm::userrow_read() reads it, and
//  UPDI writes the row even on a locked part (DS40002443A 8.7).
/// Callable from any context; the view points into the mapped row.
++ b/brio/avrxt/vref.hpp
 * What full scale means for the analog blocks of an AVR DA, DB or EA:
 * the reference each converter and the comparators measure against. A
 * program names its reference where it configures the ADC, the DAC or
 * a comparator (avrxt/adc.hpp, avrxt/dac.hpp, avrxt/ac.hpp), whose
 * init() writes the selection through the setters below, and converts
 * counts to millivolts with ref_mv() and util/analog.hpp. Called
 * directly, to keep a reference powered or to read its level:
 *
 *     brio::Vref::ac(brio::Ref::v1024, true);                  // the ACs' reference, always on
 *     constexpr uint16_t fs_mv = brio::ref_mv(brio::Ref::v2048);  // 2048
 *
 * This is a vocabulary plus three setters, not a device: the enum Ref
 * names this silicon's levels (another target's vref header defines
 * its own brio::Ref - same name, its levels), ref_mv() gives their
 * millivolts to whoever converts (util/analog.hpp holds the
 * target-independent arithmetic); the setters are called by the
 * consumers' init() - an app names its reference where it configures
 * the ADC or the DAC and never writes VREF itself.
 *
 * Which context calls what. ref_mv() and refsel() are constant
 * arithmetic, legal anywhere and folded at compile time. Each setter is
 * one store into its VREF register: it waits for nothing, masks
 * nothing, and is a setup verb of the consumer that owns that register
 * (its init(), from main() or from the active object that owns the
 * converter). Nothing here checks that VDD has the headroom a level
 * needs, nor that the reference a program converts with is the one it
 * selected.
 *
    /// The DAC's reference (VREF.DAC0REF), one store; `always_on` sets
    /// ALWAYSON, the source powered whether the DAC asks for it or not.
    /// The analog comparators' shared reference (VREF.ACREF, the one
    /// their DACREF divides), one store; `always_on` as above.
++ b/docs/avrxt/ac.md
This document is for a programmer who needs to know when an analog
voltage crosses a level, or on which side of a band it sits, without
running a conversion: a threshold detector, a window detector, or a
comparator output on a pin or on an event channel for a timer or the
CCL. `Ac<n>` is the resource, one comparator: an `AcConfig` describes
it and one function per register fact drives it. It is a monostate, a
class whose state is static, with no instances: a program names the
type and never makes an object of it. `Threshold<Ac>` and
`Window<Lower, Upper>` are the tasks over it (a task, in brio, is a
driver layer with state of its own over a resource, served from its
peripheral's interrupt or called by an active object; an active object
is a component of a brio program with its own event queue, served one
event at a time by the kernel, [../learn/introduction.md](../learn/introduction.md)).
`Threshold` sets the comparator's internal reference level, DACREF,
from millivolts and reports each crossing; `Window` pairs two
comparators on one pin and reports above, inside or below.

The smallest use, a threshold at 1.000 V on PD6, trimmed from "How to
use it" below:

```cpp
#include "avrxt/ac.hpp"
using Level = brio::Threshold<brio::Ac<0>>;
Level::init(brio::AcPos::ainp3, 1000, brio::Ref::v2048);     // medium hysteresis, both edges
ISR(AC0_AC_vect) { post<Monitor>(Crossed{Level::crossed()}); }
```

Line by line: `Level` names the threshold task over comparator AC0,
which every part of the three dies has. `init()` takes the positive
input (`ainp3`, PD6 on every comparator), the threshold in millivolts
and the reference: it writes VREF.ACREF to the internal 2.048 V
reference, sets the negative input to DACREF with the code nearest
1000 mV (125), turns PD6's digital input buffer off, and with its
defaults selects the medium hysteresis, an interrupt on both crossing
directions and the CMP interrupt enabled. It returns false, writing no
register, when the reference's millivolts are unknown (`Ref::vdd` or
`Ref::vrefa` without the `ref_known_mv` argument), when the threshold
is above the reference, or when this package does not bond the input's
pin; the excerpt ignores that answer. The `ISR()` line binds the
comparator's vector, with avr-libc's macro: `crossed()` clears the CMP
flag and returns the new state (true = above), and `post<Monitor>`
hands it to an active object; `Monitor` and its event `Crossed` are
the program's own names.

What decides correctness even there. The vector is the program's to
bind: the driver binds none and nothing checks that `AC0_AC_vect` has
a handler while the interrupt is enabled, and the CMP flag stays set
until the body clears it, which `crossed()` does. Interrupts must be
enabled globally (`sei()`) before a crossing can be served. VREF.ACREF
is one register for all the comparators: every `init()` whose negative
input is DACREF writes it, so a second comparator initialized with
another `Ref` moves the first one's threshold, and the first
`Threshold`'s `mv()` then reports a level computed for the old
reference. `init()` waits for nothing: on the DB a DACREF change settles in
tDACREF = 25 us with the bandgap already running (DS80000915F 3.7.4),
the comparator's start-up after enable has no figure (DS40002247B
39.17 gives none), and a `state()` read right after
`init()` may still show the start-up value INITVAL holds.


## Next batches (not started)

The credit of the cloud sessions ended with the sixth batch. What it
did not reach, each on the stm32g0 batch's pattern (the documents'
additive pass: the reader's orientation, the smallest use, the
prerequisite, local fixes; the headers' comments under the same brief:
what the caller gets, which context calls what, then every fact kept
with its citation, token identity on every file):

1. The peripheral documents and the headers of the seven target
   strata whose README the fifth batch rewrote and whose documents no
   batch has reached: `docs/avrxt/` (20 documents, `brio/avrxt/`),
   `docs/samc21/` (27, `brio/samc21/`), `docs/stm32f4/` (25,
   `brio/stm32f4/`), `docs/ch32v00x/` (12, `brio/ch32v00x/`),
   `docs/ch32vx03/` (19, `brio/ch32vx03/`), `docs/rp2040/` (18,
   `brio/rp2040/`), `docs/rp2350/` (22, `brio/rp2350/`), and the vendor
   pages' links; the `ch32x035` stratum is done (the sixth batch).
2. The rest of `brio/util/` (the twenty-five other headers: the analog
   converter and sampler, block streams, CAN, the clock and placement
   vocabularies, the NV heap, journal, record and writer, power, the
   meter sampler, the inbox, the USB stack under `usb/`, the test-bench
   grammar, the line parser under `proto/`, the smaller services), then
   `gfx/`, `devices/` (its README and four documents), the headers of
   `brio/host/` and `docs/host/simulator.md`, and the IP strata's
   documents (`docs/pl011/`, `docs/pl022/`, `docs/dw_apb_i2c/`) with
   their headers.
3. Before the public merge: remove DOCUMENTATION_REWRITE_BRIEF.md,
   DOCUMENTATION_REWRITE_PROGRESS.md and the two review files, unless
   the user keeps them.
