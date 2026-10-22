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
check. Branch `claude/ch32x035-docs-99jwgr`, four commits (the four
documents the console's glue touches first; the five others; the nine
platform, interrupt, timebase, clock, pin and map headers; the six
driver headers), one pull request against `docs-rewrite`. Not touched:
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
