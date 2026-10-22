# Review of the first documentation rewrite batch

Review target: [PR 1](https://github.com/uliano/brio/pull/1), branch
`docs-rewrite`, commit `f3f91b537382000d1d24da1b74c1dfdddf2ed4d4`, compared
with main at `c6509ffef92bfb4252214825bc8834449c3e4c6e`.

This report is feedback for the author and the user, not public learning
material. No review, comment, approval, or change request was posted to
GitHub. The PR files have not been edited by this review.

## Assessment

The direction is substantially better. Keep the learning route and refine
this batch before using it as the model for subsequent batches. It is a
real reorganization around a reader, rather than a cosmetic vocabulary pass.
However, some simplified explanations make stronger promises than the
implementation supports, and two first-time setup procedures have concrete
gaps. These need correction before the model is propagated.

The strongest parts are:

- The introduction now proceeds from a familiar main loop to events,
  queues, active objects, and scheduling before introducing C++ syntax.
- The blink walkthrough explains real, existing source. It does not pretend
  that a button demonstration or a multi-state application already exists.
- Reference parameters, braces, inherited names, static members, variants,
  and lambdas are explained through their actual use. The explanation of
  the derived type parameter correctly describes why Fsm specializations
  must have separate state.
- The separation of the tick interrupt, timer processing in the main loop,
  and event dispatch is useful and should remain central.
- The build target, upload target, debug executable, and launch
  configuration are distinguished. This is essential practical guidance.
- The shortened root target table is considerably easier to use.
- Several opening header comments now tell a caller what a component does
  before explaining its internal representation.
- The author records what was actually checked instead of treating a launch
  configuration as proof of a successful interactive session.

## Corrections needed in this batch

### Make the clangd setup work on a fresh checkout

Location: [vscode.md, clangd](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/docs/learn/vscode.md#L103),
and the following project/preset selection instructions.

The guide says to configure the project and then use the debug preset. But
the existing fragments `stm32g0/.clangd` and `brio/stm32g0/.clangd` both
point specifically at `build-cmake/stm32g0b1re-release`. Configuring
`stm32g0b1re-debug` does not create that database. In addition,
`brio/.clangd` routes kernel and util headers to `build-cmake/host`.

On a developer's existing machine, those databases may already exist. On a
fresh checkout, following the new guide does not establish the editor
configuration it promises. This is an inspected configuration mismatch,
not a speculative problem with the extension.

Correct the procedure without changing the configuration in this editorial
batch: explain the required release and host configuration steps once, then
return to the debug preset for debugging. Alternatively, explicitly document
the supported way to change the fragments. Name the exact database paths.
Do not imply that changing the active CMake preset automatically redirects
clangd. Distinguish source navigation checked against installed configuration
from an interactive editor session that still needs testing.

### Add the missing build step before running host tests

Location: [command-line.md, host tests](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/docs/learn/command-line.md#L240).

The sequence configures and immediately runs CTest:

```bash
cd test
cmake --preset host
ctest --preset host
```

The test preset only describes test execution. It does not build the
executables. I reproduced the failure in a separate fresh build directory:
CTest could not find `test_fsm`, reported it as Not Run, and returned 8.
Building that target first made the same test pass.

Use the complete sequence, from an explicitly stated repository root:

```bash
cd test
cmake --preset host
cmake --build --preset host
ctest --preset host
```

Also clarify the root README's shorter CTest command: it is for an already
configured and built test tree, or should link directly to this complete
procedure. The existing host reference has the same omission; correcting
its short instruction is a small dependency fix worth making now, even if
the rest of that page remains assigned to a later batch.

### Separate priority selection from response-time guarantees

Design intent clarified by the user: Tenuto deliberately stays simple.
Applications accept its scheduling limitations and keep run-to-completion
handlers as short as possible. More elaborate scheduling belongs to a
possible future kernel, not to this documentation batch. Nothing in this
finding requests preemption, fairness, priority aging, a timing analysis
framework, or a change to the implementation.

Keep two different matters explicit: interrupt handlers have equal priority
under the platform's no-nesting rule; active objects are scanned in the
fixed order supplied to Tenuto. That scan order is a selection precedence,
not a preemptive hierarchy or an equal-share service guarantee. Explain
the accepted limits plainly, without presenting them as defects to fix.

Locations:
[README.md](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/README.md#L24),
[introduction.md, execution rule](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/docs/learn/introduction.md#L101),
[introduction.md, priority](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/docs/learn/introduction.md#L159),
[first-program.md, latency](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/docs/learn/first-program.md#L397),
and [tenuto.hpp](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/brio/kernel/tenuto.hpp#L27).

An event for an object with higher priority than the currently running
object is not necessarily the next event served. Another object may have
still higher priority, and older events may already be waiting in the same
queue. Nor is general response time bounded by the longest handler alone.
Interrupt execution, timer processing, queued work, and newly arriving
higher-priority work also matter. Under sustained higher-priority load a
lower-priority object can starve.

The specific two-object diagram is useful, but its assumptions must remain
specific to that example. Do not turn it into a general timing guarantee.
The later bound in first-program.md also omits older events in the same
queue and higher-priority events arriving while the event waits.

I exercised the unchanged scheduler with a high-priority object that posts
its next event while handling the current one. A lower-priority event waited
through five one-tick handlers; continuing that pattern would keep delaying
it. This illustrates why the maximum individual handler duration alone is
not a response-time bound.

Suggested general wording:

> A handler runs to completion before another active object is selected.
> On the next turn the kernel chooses the highest-priority non-empty queue
> and takes its oldest event. Priority affects which handler runs next;
> it does not interrupt a handler already running.
>
> A long handler delays other active objects. Short handlers reduce that
> delay, but response time also depends on interrupts and pending work.
> Continually busy higher-priority objects can prevent lower-priority
> objects from running.

Keep any microsecond claims explicitly tied to a measurement or label them
as expectations for this small example, not measured bounds.

### Remove the absolute claims about blocking, shared data, and deadlock

Location: [introduction.md, active objects](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/docs/learn/introduction.md#L71)
and [communication](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/docs/learn/introduction.md#L192).

The introduction first says an active object never blocks, then correctly
explains that a handler can block in print. Present the rule consistently:
there is no per-object blocking/suspension mechanism in the scheduler, and
application code must account for any function that waits while a handler
is running. A waiting function stalls the other active objects on that core.

The sentence claiming that asynchronous posting means no deadlock and no
ordering puzzle is too strong. Posting itself does not wait for the
receiver. Application protocols can still stop making progress, for example
if two components each wait for an event the other will never send or a
required completion event is dropped. Avoid advertising a system-wide
property that follows from neither the API nor the scheduler.

Likewise, describe private object state as the design rule, not a language
enforcement mechanism. brio deliberately supports buffers shared under
explicit lifetime protocols; its borrowed-buffer ordering requirements are
precisely an example of ordering that matters.

Suggested replacement:

> Each active object manages its own application state. To request work
> from another object, it posts an event and returns without waiting for
> the receiver. The receiver handles that event in a later dispatch.
> When an event refers to a shared buffer, the program must also follow
> the buffer's ownership and lifetime rules.

There is no need to teach every advanced lifetime rule here; link to it.

### Account for alignment when explaining event size

Locations: [first-program.md, event types](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/docs/learn/first-program.md#L64)
and [cpp-notation.md, variant](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/docs/learn/cpp-notation.md#L169).

The claim that a slot is the largest event plus one tag byte omits padding
and makes the tag representation sound universal. With the actual ARM GCC
16.2 toolchain and `-mcpu=cortex-m0plus -mthumb`, a probe using the exact
variant alternatives gave:

```text
sizeof(SetPeriod) = 2
sizeof(std::variant<Entry, Exit, Toggle, SetPeriod>) = 4
```

Explain that the variant stores the largest alternative, a discriminator,
and any alignment padding required by the implementation. The queue stores
that Event type. Empty event alternatives do not increase this particular
variant's size, but that is not a general formula that computes all event
sizes. Use `sizeof(Event)` if exact memory use is important.

### Distinguish tick conversion from physical timeout accuracy

Locations: [first-program.md, time](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/docs/learn/first-program.md#L294),
[time.hpp](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/brio/kernel/time.hpp#L15),
and [time_event.hpp](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/brio/kernel/time_event.hpp#L29).

Rounding the conversion upward avoids shortening the requested number of
tick intervals through arithmetic truncation. It does not guarantee a
wall-clock delay of at least the requested milliseconds from an arbitrary
arming instant. TimeEvents::arm stores `now() + delay_ticks`; now() is an
integer tick count. With a 1 kHz tick, a one-tick timer armed just before
the next tick can mature almost immediately in physical time. Later loop
processing or queue service can delay handling further.

The blanket early-expiry wording already existed in the old header; this
review is not attributing a new implementation defect to the rewrite. But
the new tutorial repeats it as a teaching guarantee, so this is the right
time to qualify it consistently. Do not change timer behavior in this PR.

Also qualify two current reference claims:

- Signed wrap-safe comparison requires the usual half-range restrictions;
  it is not valid for every uint32_t delay. An actual probe with
  `arm(0x80000001u)` caused an immediate post when process ran at the same
  current tick. State supported delay/overdue limits in the reference.
- A periodic timer preserves its sequence of scheduled deadlines and
  attempts catch-up posts. It cannot promise the delivered event count
  when an owner's queue fills, because post drops on overflow.

Keep the beginner explanation short:

> The conversion rounds up to whole ticks. A timer is scheduled against
> the platform's tick counter; the tick's resolution and the time spent
> waiting for the kernel affect when its event is actually handled.

Put the exact phase, wrap, and catch-up details in the timer reference.

### Fix the expected debugger observations and keep the WFI caveat accurate

Location: [command-line.md, debugging](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/docs/learn/command-line.md#L213).

The wording says that printing the queue shows slots, cursors, and the
overflow counter all zero in a healthy blink. After normal event handling,
queue cursors and old slot contents need not be zero. Only the absence of
overflow is expected here. A queue can be empty while its bookkeeping and
stored bytes are nonzero. A small host probe confirmed that condition with
zero overflows.

Make the observation actionable: the overflow count should remain zero;
the cursor values change as events pass, and a complete queue dump includes
variant implementation details. Do not lead beginners to interpret a
nonzero cursor as a fault. Reading an accessor through GDB can require an
inferior function call; for a first observation, use the already documented
stored counter while halted and explain it precisely.

The following paragraph correctly says that memory reads through ST-LINK
while the core is in WFI can be unreliable, but then says that neither this
nor DBG_STOP matters for blink because it uses plain Sleep. The existing
target reference records bad reads from a running console in WFI; this is
not limited to deep Stop. Distinguish the two caveats:

- For reliable memory inspection, halt the core first, including when the
  application uses ordinary WFI Sleep.
- DBG_STOP changes the behavior of deeper Stop modes; blink does not use
  those modes.

### Describe current GUI coverage without deciding its future

Location: [vscode.md, final paragraph](https://github.com/uliano/brio/blob/f3f91b537382000d1d24da1b74c1dfdddf2ed4d4/docs/learn/vscode.md#L256).

The statement that bench verbs have no graphical counterpart and are not
meant to have one makes a product decision that is not established by this
editorial assignment. The user specifically wants the graphical workflow
to receive proper attention. Document existing coverage and limitations,
without declaring that missing GUI actions are intentionally excluded from
future work.

Suggested wording:

> The repository currently documents these bench operations through the
> command line. This guide covers the graphical build, upload, debug,
> and host-test actions supported by its VS Code configuration.

Do not add GUI tooling as part of this correction.

## Editorial refinements before copying the pattern

### Make VS Code a first-class route in the navigation

docs/learn/README.md says to read every page in order, but the command-line
and VS Code guides are alternatives for performing the same task. A reader
using VS Code should not be required to read the terminal walkthrough first.

Keep the conceptual route in order. Present cpp-notation as a companion
reference and explicitly offer a choice of operating route. At the top of
the first-program page, add a direct link for readers who want to run the
unchanged program first and study it afterward. That requires no new example.

### Do not let verification notes take over the user procedure

Honest verification status is valuable. But command-line.md repeatedly
describes the circumstances of authorship, comparisons with a sibling
board, and whether a sequence was walked through on the desk. The VS Code
page has a matrix, per-step discussion, and long recorded/expected
qualifications. Some of this now resembles the internal work record the
rewrite was intended to keep out of the learning path.

Keep one concise evidence/status summary and explicit exceptions where they
affect a user's choice. In the steps themselves, foreground action, expected
result, and recovery. Keep the detailed verification account in the progress
record or PR. An accurate warning need not be repeated as a paragraph at
every stage.

The detailed C_DEBUGEN/DBG_STOP explanation is valuable reference material,
but it appears at length in both workflow pages before the beginner has
finished running blink. For this program, a short note and a descriptive
link are sufficient; retain the full explanation in the target reference.

### Finish one internal comment as a genuinely readable model

event_queue.hpp has a clearer new opening, followed by a large retained
implementation block with headings such as INDEX CAROUSEL, THE PRODUCERS,
and A STATIC QUEUE IS ALL ZERO. Depth is explained more than once. Other
retained passages still use unexplained shorthand, for example an epoch
check in borrowed.hpp or a thunk in post.hpp.

This is not a reason to delete the algorithm's invariants. Preserve the
slot/ring distinction, cursor ownership, publication ordering, release
rules, and one-core assumptions. Rewrite the prose so a maintainer can
follow it: ordinary headings, one meaning per paragraph, a definition before
a specialized term, and no repeated explanation where a local reference
suffices. Reuse the existing queue diagram when helpful.

The first batch should establish at least one complete model for internal
comments, not only a model for introductory header paragraphs. Otherwise the
next batch may add readable prefaces while leaving most difficult comments
unchanged. The existing sample already proves that this can be improved
without changing code tokens.

### Keep navigation summaries shorter than the documents they advertise

Most of docs/README.md's inherited reference map still has paragraph-sized
table entries. This is explicitly acknowledged as inherited content, not a
claim that the author secretly rewrote it badly. Nevertheless, this page is
the new learning route's entry point, and it remains a poor navigation model.

Shorten those descriptions to a reader's question or one concrete purpose,
preserving links to all detailed reference material. This small navigation
change need not become the full design/peripheral rewrite. Also recheck
shortened root-table wording for remaining internal phrases such as "every
chapter green on both"; say what was tested instead.

## Verification performed for this review

- Fetched the changed files at the pinned PR commit into a separate snapshot
  under /tmp and inspected the learning route, root/readme rules, ten kernel
  headers, and the author's progress report. Compared claims against existing
  source, presets, launch/settings files, board/reference pages, and tests.
- Repeated compiler-based comment stripping and normalized comparison for
  all ten changed headers against main: all ten were identical after comment
  removal. This is consistent with the author's token-identity result.
- Ran the existing prose checker on the 18 reader/source files, using
  snapshot-aware path resolution for the new learn pages: zero errors and
  zero review items. The adaptation only lets paths resolve against the
  fetched PR snapshot as well as unchanged files in the local repository.
- Checked the relative Markdown destinations and heading anchors against
  the changed-file snapshot plus unchanged base files. The basic parser
  initially mistook lambda notation for links; those are parser false
  positives, not documentation defects. No actual broken link was established.
- Inspected installed CMake Tools 1.24.42 command metadata. The documented
  Select Active Folder and Set Launch/Debug Target titles exist there.
  This does not verify their interactive behavior or every status-bar label.
- Configured a fresh host-test directory outside the project build trees.
  Running test_fsm before building reproduced the missing-executable failure.
  Built test_fsm, reran it, and it passed.
- Built the unchanged STM32G0 blink application with the existing debug
  configuration into /tmp, using ARM GCC 16.2. The build passed. This was
  the local base source, whose changed kernel files are token-identical to
  the PR; it was not a complete build of every PR target or preset.
- Used ARM GCC for Cortex-M0+ to establish the exact variant size above.
- Compiled and ran small temporary host probes against the PR headers for
  priority/starvation, nonzero bookkeeping in a healthy drained queue,
  tick-deadline posting, and the unsupported signed half-range example.
  These are review probes under /tmp, not new examples or committed tests.
- No board was programmed. No interactive VS Code or hardware debug session
  was performed. No full image-identity gate or full host suite was run.

## Guidance for the next batches

Before starting the next batch, correct the concrete setup and guarantee
issues above and repeat the focused reader review on the changed passages.
Do not restart the introduction from scratch: its structure is useful.

For subsequent work:

1. Separate selection rules from timing guarantees. Test absolute words
   such as next, never, always, exactly, and no against a counterexample.
2. Start task verification from a fresh build/configuration state, not from
   a developer's machine with release and host databases already generated.
3. Give independent readers scenario questions as well as definitions:
   queued older events, a still-higher-priority object, a full completion
   queue, and an arming instant between ticks are useful cases.
4. A reader agent finding a supporting sentence only establishes that the
   document says something. It does not establish that the sentence is
   technically true. Keep an implementation-based review alongside it.
5. Preserve measured and architectural detail, while keeping the default
   learning/procedure path focused on the reader's immediate task.
6. Teach only through existing applications during this pass. Missing
   examples remain follow-up work, not reasons to expand or delay this PR.
7. Keep the corrections editorial. Any implementation problem discovered
   during fact checking should be recorded separately; do not change the
   scheduler, timers, queue API, or editor configuration to rescue prose.

The review supports continuing with this approach after these focused
corrections. It does not recommend merging the batch unchanged or treating
the unperformed GUI validation as complete.
