# Rewrite brio documentation for embedded programmers

This is an execution brief for Claude. Read it completely, then carry out
the work. The user wants a substantial rewrite of the public documentation
and, where useful, source comments. Deliver the revised material, not just
an assessment or a proposal. Work autonomously within each agreed batch.
Deliver the first batch for user review before extending its pattern across
the repository. The full rewrite remains the objective; one batch is not
completion of the whole assignment.

This brief is an editorial work instruction. It is not part of the reader's
documentation and should not be linked from the public learning path.

## Immediate priority: improve what already exists

The user has clarified the time constraint: this pass must use the expiring
Claude token allowance to improve existing documentation and comments.
Missing teaching examples are acknowledged, but creating them is deferred.
Do not block the rewrite on designing, implementing, or compiling examples.
Do not create new demo applications, button firmware, portable board files,
or executable draft examples in this pass, even outside the build globs.

Teach from existing source. Excerpts are allowed when they faithfully show
existing behavior and are clearly identified as excerpts, not standalone
programs. Conceptual diagrams and prose scenarios are welcome; they must
not imply that a missing application already exists. Record missing examples
briefly as follow-up work and continue with the material available now.
This priority overrides any broader permission elsewhere in this brief to
introduce examples. Preserve the long-term learning goal without making its
complete implementation a prerequisite for useful work today.

## Repository state at launch

This brief is committed on the branch `docs-rewrite` together with the
editorial rules it asks for: the relevant paragraphs of CLAUDE.md and
docs/README.md are ALREADY revised there (the audience, the reader's
orientation first, plain language, the verified/configured/unavailable
rule for workflow steps, maintained teaching examples versus disposable
bench apps, the source-comment scope, the identity check for a
comments-only edit). Read them, work under them and do not rewrite them
again; refine a sentence only where executing the batch proves it wrong,
and say so in the PR.

The home of the learning route is `docs/learn/` (its own README.md is
the route's table of contents), already named by docs/README.md and
CLAUDE.md; create it in the first batch. `blink` is the one maintained
teaching example today. Work on this branch; base the first PR on main.
The repository's local checks are `bin/brio prose <paths>` and
`bin/brio gate --tokens --against <ref> <files>`; both run without the
cross toolchains.

## Purpose and intended reader

Make brio understandable and usable by an embedded enthusiast who knows C
and has programmed microcontrollers, but may know little C++ and nothing
about active objects or the approach associated with Miro Samek.

Assume familiarity with functions, structs, pointers, registers, interrupts,
and a simple main loop. Do not assume familiarity with templates, lambdas,
variants, concepts, CRTP, ownership terminology, cooperative scheduling,
state machines, or the QP framework. Introduce these as needed.

The reader is technically capable. Explain unfamiliar ideas without baby
talk, motivational filler, or a general C++ course. Show why each idea is
useful in a microcontroller program and what the programmer must do.

VS Code is the intended interactive working environment. The documentation
must explain the available graphical workflow as carefully as the command
line workflow. Establish what actually works before describing it.

Success means that a reader can:

- Explain how brio differs from their familiar interrupt-and-main-loop
  program, including the costs and obligations of its cooperative model.
- Follow an event from an interrupt or timer to application behavior.
- Understand the essential C++ used in a small brio application.
- Build, program, observe, and debug a suitable example using the supported
  VS Code workflow, with explicit prerequisites and honest limitations.
- Find the exact technical reference when their application needs more.
- Read a source comment without reconstructing private development history.

## Scope, authority, and boundaries

Rewrite documentation and explanatory comments. Preserve executable
behavior, public names, signatures, build behavior, hardware configuration,
and existing technical guarantees. Do not refactor code while rewriting
comments. Do not change register operations, timing, queue sizes, or API
design to make an explanation easier.

You may add and reorganize documentation, repair links, explain existing
applications as teaching material, and revise documentation-related instructions in
CLAUDE.md and docs/README.md. Keep public repository text in English and
ASCII, as the repository requires. Communicate with the user in Italian.

The user explicitly authorizes replacing the editorial assumptions that
prevent this work. In particular, a rule that excludes all application
examples from the main documentation must be narrowed to exclude disposable
bench applications, while allowing maintained teaching examples. Explain
that distinction in the revised rules. Do not ask the user to authorize
routine editorial decisions already covered by this brief.

Preserve valid engineering requirements, technical evidence, and the
separation of public information from private bench records. Do not publish
private manifests, probe serials, local incidents, or personal memory.
Never replace precision with an unsupported simplification.

Source inspection, configuration inspection, and relevant verification are
part of the work. Hardware access, installing software, changing the VS Code
configuration, and changing executable examples are not automatically
required by an editorial rewrite. Use existing capabilities; record concrete
limitations when verification is unavailable. Do not turn this task into a
tooling migration. In a local workspace, do not commit or publish without
Git delivery authorization. In the cloud execution covered by this brief,
use a dedicated working branch, make coherent commits, push that branch,
and open a reviewable PR for each batch. Never work directly on main, merge
the PR, force-push shared history, or publish a release. If PR creation is
unavailable, deliver the pushed branch and an exact PR title/description.
Do not confuse a locally saved file with output delivered from a cloud run.

### Cloud execution profile

The reported cloud environment lacks the cross toolchains and the required
host compiler installed locally under /sw. Do not attempt cross builds,
host test builds, toolchain installation, or repeated compiler discovery in
that environment. The available checks reported for this task are
`bin/brio prose` and `bin/brio gate --tokens`; inspect their actual options
and dependencies once before using them. If a dependency is missing, record
that limitation and continue with source/configuration review. Do not label
an unexecuted check as passing.

Compilation and hardware/GUI validation are a later local handoff. Reuse
existing applications, preserve their behavior, and say explicitly that
their compilation was not repeated in the cloud. Do not write new executable
examples in this pass. A tutorial can explain an existing application and
quote its source without creating a new build target. Future examples need
their own implementation and local compilation work after this pass.

Keep this brief and a short progress record committed on the working branch
so they survive context compaction. Before each batch is handed off, check
that both are actually tracked. They are temporary review/work artifacts;
exclude them from the eventual public merge unless the user wants to retain
them. Do not remove the only durable continuation record while work remains.
Record their required removal in the PR handoff if it is deferred.

## Diagnosis: why the current writing is hard to use

The observations below come from a sample, not a complete technical audit.
Inspect the current files before editing: line numbers and details may move.

### The introduction requires too much prior knowledge

README.md opens with a dense collection of architectural and C++ properties:
cooperative active objects, queues, flat state machines, a shared stack,
allocation choices, virtual functions, compile-time resolution, C++23, and
the runtime. Its first example then combines templates, static members,
inheritance, variants, and lambdas.

Each fact may be useful. Together they give an unfamiliar reader too many
questions before answering the first one: what problem does brio help me
solve? Establish the problem and the execution model before the mechanism.

### Private shorthand has become public vocabulary

Look for phrases such as "the reserve", "the silicon's offer", "the two
loans", "pack order", "types and verbs", "the run", and "realizations".
They compress meanings that the author already knows. Some deserve a
definition; others should be replaced with ordinary words.

Keep actual API identifiers unchanged. Translate the prose around them.
For example, explain an ordered list of active objects before naming the
C++ parameter pack that represents it.

### Sentences and tables accumulate unrelated levels of detail

The target table in README.md contains cells large enough to be chapters:
compiler details, instruction sets, driver coverage, board measurements,
silicon exceptions, unresolved work, and development chronology.

A target selection table should help someone choose a target. Put detailed
coverage and hardware findings in the target's guide and reference pages.
Keep any limitation that materially affects the choice visible in the table.
Do not hide a blocking limitation behind a cheerful status label.

### Emphasis often expresses insistence rather than importance

Repeated capitals such as ONE, SECOND, GROUPS, and CONTRACT flatten the
hierarchy of information. Bold a consequence the reader must remember,
especially one that determines correctness. Do not emphasize every noun.

For example, the cooperative scheduling rule matters more to a first-time
user than the stylistic rule for extracting a shared peripheral layer.

### References rely on memory

docs/design/kernel.md uses numerical section references extensively.
docs/boards/README.md describes short board codes and letter-based groups of
tests. A reader should not have to remember what a number or letter means.

Use descriptive links. When a tool requires a letter or short identifier,
show it beside the full meaning. Do not invent a replacement identifier or
remove one needed to operate an existing tool.

Numbered steps are useful for an actual sequence. The problem is unexplained
codes and references, not numbering itself.

Numerical references into kernel.md also occur outside that document, in
CLAUDE.md, other Markdown pages, and source comments. Before changing its
headings or numbers, inventory all incoming references using searches for
both the filename and section wording. Preserve stable destinations or
update every incoming reference in the same batch. A path-existence check
does not establish that a heading anchor or a section number is valid.
If an incoming reference belongs to a later batch, updating that reference
now is a necessary dependency repair, not permission to rewrite that file.

### The documentation rules favor completeness over learning

docs/README.md requires detailed peripheral inventories and puts source
documents and implementation references before the reader's use case.
It also excludes individual applications from the main documentation,
apart from measurement apparatus. CLAUDE.md reinforces these rules.

These rules support engineering reference material, but leave no adequate
home for a guided introduction. Add that home and adjust the rules so the
next assistant does not erase it. Keep measurement traceability and precise
references in the technical material.

### There are good foundations to retain

docs/design/kernel.md defines an active object in useful terms and has
explicit C++ notes. Hardware documents contain valuable measured behavior
and restrictions. Source comments often preserve the reason for a subtle
implementation decision. Keep this knowledge and improve its presentation.

Do not assume that every long explanation is unnecessary. Buffer lifetime,
interrupt exclusion, queue overflow, sleep/wakeup behavior, and errata can
require substantial detail. Make the detail navigable and place it where
the reader needs it.

### VS Code support exists, but configuration is not proof of usability

.vscode/settings.json defines multiple CMake source directories, status-bar
controls, and separate build/upload and debug workflows. .vscode/launch.json
contains target-specific debugger configurations. .vscode/extensions.json
recommends relevant extensions.

Inspect all three alongside CMake presets and the target guides. Existing
comments describe some verification, but they are evidence to examine, not
a reason to claim that every board's full workflow works now. Do not infer
universal support from a single successful configuration.

## Editorial principles to apply everywhere

### Start with purpose, then behavior, then mechanism

For each subject, answer these questions in a natural order:

1. What problem does this solve in an embedded application?
2. What does the program do, and when does it do it?
3. What must the application programmer provide or avoid?
4. What is the smallest useful example?
5. Where are the exact API and implementation details?

Reference pages can be more compact, but should still begin with orientation.
Do not force the reader through register provenance before explaining what
the driver is for. Include an immediate safety or correctness prerequisite
when it affects even the simplest use.

For peripheral reference pages, the default first pass is additive: put a
short purpose, minimal-use example, and essential prerequisite section
before the existing technical structure. Keep the reference material
intact unless specific passages need rewriting. This limits unnecessary
churn across many pages. It is not permission to leave the rest unreadable:
rewrite dense explanations locally and propose a separately scoped broader
reorder when the page still fails its reader's task. The first batch does
not include a mass reordering of peripheral documents.

### Give every page a clear job

Organize the material around four reader needs:

- Learning: a coherent first encounter with the ideas and a working example.
- Doing: a procedure for a concrete task, with prerequisites and results.
- Looking up: exact capabilities, options, contracts, and limitations.
- Understanding design: reasons, costs, alternatives, and architecture.

Use descriptive navigation labels. Do not create a rigid new taxonomy that
the reader must learn. A small subject can fit on one page; a large subject
needs linked pages. Reuse existing locations when they serve the reader.

Keep one authoritative statement of volatile facts such as platform status.
Short local reminders of essential rules are welcome. Avoid making the
reader follow five links to recover a prerequisite for the current action.

### Use ordinary technical English

Prefer direct subjects and verbs. Replace metaphors when they obscure the
mechanism. Use consistent terms: event, event handler, queue, interrupt
handler, active object, state, transition, timer, buffer owner, and caller.

Explain "contract" as requirements and guarantees when first introduced.
Use ordinary words for general explanation, but retain repository terms
where they name an actual structure. Explain a stratum as a directory
representing a defined architectural layer, then use the term consistently
in that context. In CLI instructions, a verb is the action after `brio`,
such as `flash` or `run`; retain that vocabulary and define it. In C++ API
explanations, "function" or "operation" is usually clearer. Do not create
competing names for the same directory or tool concept.

Do not ban necessary vocabulary. Define active object, cooperative,
run-to-completion, variant, and concept, then use them consistently.
Retain standard abbreviations such as GPIO and DMA where appropriate.
Expand less familiar abbreviations at their first relevant use.

Avoid authorial claims such as "obviously", "simply", "for free", "the code
already reads right", and "correct by construction" without stating exactly
which property is checked and what still depends on the programmer.

### Control information density

Aim for one main idea per paragraph. Use short sentences where the subject
is unfamiliar. Split a sentence if understanding its end requires holding
several nested qualifications in memory. Treat length as a diagnostic, not
a mechanical word limit.

Use a table when readers compare short, parallel facts. Use prose when they
must understand a causal explanation. If a table cell contains a long
paragraph, reconsider the table. Lists should expose a genuine set or
sequence, not turn every sentence into a bullet.

Use headings that answer recognizable questions or name concrete tasks.
Make links meaningful out of context. Replace "section 7" and "see above"
with links whose labels name the subject.

### Preserve boundaries and qualify guarantees

Distinguish the conceptual rule, its implementation, compiler checks,
runtime checks, programmer obligations, and measurements on specific hardware.
Do not collapse these into a single promise.

Examples of claims requiring care:

- Cooperative does not mean that interrupts cannot interrupt a handler.
- An event copied into a queue does not necessarily copy a buffer pointed
  to by that event.
- A type that names a buffer lifetime does not automatically enforce all
  uses of a raw pointer extracted from it.
- Static storage and inline variables do not by themselves prove the
  absence of every initialization-order issue in arbitrary C++ programs.
- Compile-time configuration does not mean the application has no runtime
  state or decisions.
- A configured debugger is not proof of a successful graphical session.
- A measured implementation cost is not a universal guarantee for every
  compiler, build option, clock, and part.

Verify the exact brio behavior before choosing wording. When two existing
statements disagree, inspect the implementation and tests. Record an
unresolved technical question rather than selecting the easier sentence.

In particular, reconcile broad statements that handlers "never block" with
the documented blocking transmit path used by print. Explain the actual
cost and constraint; do not silently invent asynchronous output or promise
that all library calls return immediately.

## The learning path to build

Use a small example that remains recognizable throughout the introduction.
Use the existing blink application: Blinker toggles an LED from a periodic
time event, and Supervisor changes its period by posting SetPeriod every
three seconds. For example, stm32g0/src/apps/blink.cpp already implements
this interaction. There is no existing button application to reuse; do not
invent one or make its creation a prerequisite for this tutorial.

Explain the existing application in stages so the reader can associate each
mechanism with its purpose. This is a sequence of explanations of the same
source, not a request to produce new application variants. The longer-term
learning path below includes topics that may need future examples. In this
pass cover them conceptually or link to existing implementations, and state
where a hands-on example is still missing. A useful progression:

1. Recall the familiar main loop and interrupt model. Explain where flags,
   polling, and waiting become difficult to coordinate as features grow.
   Do not imply that a main loop or C is inherently inadequate.
2. Explain an event as a small record describing something that happened.
   Separate an event's data from the function that handles it.
3. Explain the queue: an event can wait while another handler finishes.
4. Introduce the active object as a responsibility with its own queue and
   persistent state. Explain that it is not an operating-system thread.
5. Show one complete event path, including interrupt glue where applicable.
6. Explain cooperative scheduling and priority. A higher priority selects
   the next eligible handler; it does not interrupt the current handler.
7. Introduce states through a behavior that actually depends on state.
   Explain entry, exit, and transitions using the supported flat model.
8. Replace a delay or busy wait with a timer that results in a later event.
   Explain where the timer is checked and where user code executes.
9. Show communication between two active objects. Introduce post before
   publish, replies, or borrowed buffers.
10. Explain capacities, overload, latency, and how to observe a failure.
11. Link to advanced lifetime rules, services, power behavior, and multicore
    operation after the single-core model is secure.

Explain the scope of the single-core model explicitly. Do not present it
as a universal claim that the repository cannot run work on two cores.

Use a small diagram where it removes uncertainty. For example, a sequence
showing interrupt, queued event, current handler completing, and next
dispatch makes scheduling easier to understand. Prefer plain Markdown,
ASCII diagrams, or the repository's existing diagram format. Do not add
site-generator-specific markup just to render a learning diagram.

Give prerequisites, complete code or a link to a maintained complete
example, expected visible behavior, and one troubleshooting route. State
its verification status; in the cloud follow the compilation handoff rules
above rather than presenting an uncompiled draft as a verified program.
Mark fragments and pseudocode explicitly. Do not present an illustrative
snippet with missing aliases, hardware setup, or an undeclared active object
as a complete program.

Teaching examples must have a stable documented purpose and be checked when
their APIs change. They are separate from disposable experiments and bench
probes. A historical experiment can inform a tutorial without becoming its
unverified foundation.

## Explain C++ through the code the reader is using

Introduce language features at first use, with an optional deeper link.
Avoid placing a catalogue of modern C++ features before the first result.
The table below is a menu of possible explanations, not a syllabus. Derive
what you teach from the actual example and inspected implementation. Omit
features absent from the selected route.

| Construct | Useful first explanation | Necessary boundary |
| --- | --- | --- |
| namespace | A prefix that groups names; brio::Pin names Pin in brio. | Do not teach namespace design before use. |
| using alias | A local name for a type, often used to select a board or driver. | An alias is not a variable or hardware initialization. |
| template argument | A choice supplied when the compiler creates a type or function. | Explain fixed choices separately from runtime values. |
| static member | State or a function associated with the type rather than each instance. | Static state can still change while the program runs. |
| inline variable | A way to define shared program storage in a header. | Do not infer arbitrary initialization guarantees. |
| lambda | A small function written where it is passed to another function. | Explain parameter type, return value, and any capture used. |
| std::variant | One value that can hold one of a listed set of event types. | Distinguish the current alternative from all alternatives at once. |
| match | Selects the handler for the event type currently stored. | Explain catch-all handling and compatible return types. |
| concept | A compiler-checked set of requirements on a type. | It cannot prove every behavioral obligation. |
| Derived type as template argument | In Fsm, passing the active object type gives each specialization its own state. | Explain this concrete purpose before the CRTP name; do not invent callbacks into Derived. |
| span or borrowed view | Access to existing elements without taking ownership. | State who owns the storage and when access expires. |

Use C comparisons to establish a foothold, not to claim equivalence. For
example, comparing a variant to a tagged union is useful if you also explain
that its interface tracks and selects the active alternative. A template is
not just a macro; a concept is not a runtime validation function.

## VS Code: document the actual supported workflow

Build a matrix for the supported operations and target configurations while
inspecting the repository. Use it as working evidence; publish a compact
reader-facing version if useful. For each operation distinguish:

- Implemented and verified interactively for the named configuration.
- Configured, with interactive verification still needed.
- Available through the terminal, without a documented graphical action.
- Unavailable or blocked, with the specific reason.

Cover opening the correct repository folder, obtaining required extensions,
toolchain prerequisites, selecting the active CMake project, choosing the
board/build preset, choosing the build target, compiling, uploading,
selecting the debug executable, starting a debug session, observing serial
output, and running host tests. Some of these may need separate pages.

Pay particular attention to the fact that the repository root is not itself
a CMake project. Explain the active-project choice in the interface before
asking the user to build. Distinguish the build target, the upload target,
and the executable selected for debugging; do not assume they change together.

Inspect the relationship between extension recommendations and debugger
types actually used. A cppdbg launch configuration and a cortex-debug
configuration have different providers. Verify prerequisites rather than
assuming a recommended extension pack supplies everything. Inspect serial
monitor support separately from debugger support.

For each procedure provide:

1. Exact prerequisites for the named board, probe, tools, and host system.
2. The action using the verified interface label or command-palette entry.
3. The expected result: selected preset, produced executable, upload
   completion, stop at main, visible output, or test result.
4. A short recovery path for the most likely failure at that step.
5. A command line equivalent where useful for diagnosis or automation.

Do not invent menu entries or describe an untested sequence as verified.
If no graphical session is available, inspect local extension contributions
and configuration where possible, label what remains unverified, and
complete the rest of the rewrite. Do not make the user confirm every menu.

Absolute paths such as /sw/... describe an existing installation; they are
not self-explanatory setup instructions for a new user. Explain what must
be installed, where the project expects it, and what can be configured.
Do not expand operating-system support claims beyond the evidence.

## Source comments: make obligations easy to find

Review public headers and relevant application comments, not just Markdown.
Prioritize kernel entry points, event handling, timers, buffer lifetimes,
serial services, bus requests, and representative peripheral drivers.
Then cover the rest of the public framework and reader-facing examples.
Do not spend this task rewriting third-party sources or private records.

At a header's beginning, explain its responsibility, when an application
uses it, and its essential constraints. Follow with clearly separated
technical detail where needed. Do not move a warning away from the API
whose correct use depends on it.

A public operation's comment should answer whichever of these matter:

- What does it do, and what does success or failure mean?
- From which execution context may it be called?
- Can it wait, block, invoke user code, or disable interrupts?
- Who owns input and output storage, and how long must it remain valid?
- What changes before return, and what completes later?
- What setup or ordering must already be in place?
- Which requirement is checked, and which remains the caller's duty?

Internal comments should explain a non-obvious reason or invariant. Keep
the explanation near a subtle register sequence, memory-order assumption,
or ownership transfer. A link can supplement the local reason but should
not replace it. Do not comment every assignment or translate syntax into
English line by line.

Retain the technical explanation for compact representations and hardware
workarounds. Present supported behavior before implementation history.
Remove speculative future features from current API descriptions, or place
them in clearly separate planning material when still useful.

Comments in fsm.hpp, active_object.hpp, tenuto.hpp, borrowed.hpp, and
serial_port.hpp are useful early candidates. Several currently mix user
rules, implementation details, future possibilities, metaphors, and
performance evidence in the same opening block.

## Worked editorial examples

These examples demonstrate the desired writing. They are not substitutes
for checking current behavior. Preserve actual API spelling when applying
them. Fragments below are illustrative, not complete applications.

### Introduce the execution model before its C++ representation

Current wording in README.md includes:

> Everything is a type, resolved at compile time. AOs, drivers and the
> kernel are monostate classes selected by type.

A beginner-facing explanation could read:

> A brio application is divided into components called active objects.
> Each one has a queue of events waiting to be handled. The kernel chooses
> an object with pending work and lets it handle one event.
>
> **One handler must finish before another active object can run on that
> core.** Interrupts may still interrupt a handler. Keep handlers short;
> when work must continue later, arrange another event instead of waiting
> for it inside the handler.
>
> The application lists its active objects in priority order. brio uses
> that list at compile time to generate the scheduling code. The following
> example shows how the C++ notation expresses the list.

Follow with an existing maintained example or a clearly marked excerpt from
it, and state its verification status. Explain the cost and behavior of any
blocking library call used by the example rather than repeating an absolute
slogan.

### Explain a template at the point of use

Instead of:

> The monostate type is the one idiom; identity and invariants go in the
> template parameters.

Write:

> In `Pin<'A', 2>`, the letter and number select pin PA2 when the program
> is compiled. The angle brackets contain template arguments: fixed choices
> used to form this pin's type. An alias gives that type a convenient name:
>
> ```cpp
> using OutputPin = brio::Pin<'A', 2>;
> ```
>
> This line names the pin; it does not configure the hardware. A later
> call configures the pin as an output. The output level can then change
> while the program runs.

Use this only with a target whose Pin interface matches it; identify that
target and link to a complete setup example. A physical pin number on a
connector may differ from the GPIO name PA2.

### Explain the actual use of the derived type

The current kernel documentation explicitly calls this a CRTP use, and
fsm.hpp declares `Fsm<Derived, Alts...>`. Do not say that the pattern is
absent, but do not teach a generic static-polymorphism story either.

A suitable explanation after showing the real declaration is:

> In `Fsm<Buttons, Tick>`, Buttons is the class being defined. Passing its
> own type to the base template gives this state machine a specialization
> distinct from `Fsm<Traffic, Tick>`. Each specialization has its own stored
> current state, even when both machines accept the same event types.
> This form is often called CRTP. Here its purpose is to keep their state
> separate; the base does not use it to call functions on Buttons.

This is more useful than either requiring prior CRTP knowledge or removing
its name while leaving the self-reference unexplained.

### Keep tool names while explaining their meaning

Instead of replacing every occurrence of "verb" or "stratum", write:

> The `flash` verb builds and programs the selected application. A verb is
> the action name immediately after `brio` in a command.

And, where the architecture is introduced:

> The project groups code into directories called strata. Each stratum has
> rules about which other parts it may include. For example, kernel code
> cannot include a microcontroller-specific driver.

Explain the exact arguments of a real command alongside that command. Do
not invent a board letter and expect the reader to know a private manifest.

### Explain borrowed storage as a lifetime rule

Current borrowed.hpp terminology includes:

> The reply IS the return of the loan.

Write the practical rule first:

> The event contains a pointer to the buffer, not a copy of its contents.
> The sender still owns the buffer. For a `Lease::reply` request, it must
> keep the storage valid and follow the transfer protocol's access rules
> until the receiver sends the agreed completion event. After completion,
> the sender can reuse the buffer.
>
> `Borrowed<T, Lease::reply>` records that agreement in the type. It does
> not prevent every misuse of a raw pointer obtained from the wrapper.

In an actual bus example, specify the exact completion event, read/write
rules, and behavior if submission fails. In a dispatch-lifetime example,
state that the pointer must not be retained after the receiving handler
returns and explain the required ordering separately. Do not conflate the
two lifetimes.

### Rewrite a scheduling constraint in a source comment

Current serial_port.hpp wording begins the relevant rule with:

> SCHEDULING CONTRACT - consumer above producer: LineSink MUST precede
> SerialPort in the Tenuto pack.

A revised comment could read:

```cpp
// Put LineSink before SerialPort in Tenuto's list of active objects.
// This gives the sink higher priority, so it consumes each completed
// line before SerialPort runs again and reuses the line buffers.
// The sink may use the line only while handling LineReceived; it must
// not save the pointer for a later event. Tenuto checks the ordering.
```

Retain the surrounding explanation of the two buffers, how many lines may
be pending, and what happens when reception outruns consumption. The shorter
rule is an entry point, not permission to delete the algorithm's invariants.

### Preserve implementation rationale without its development story

fsm.hpp's Status comment explains a pointer-sized representation through a
previous representation and its calling-convention cost. A clearer form is:

```cpp
// Store the result in one handler pointer: null means handled, the
// private marker means unhandled, and any other value names the next
// state. Keeping Status pointer-sized avoids the larger return-value
// representation that can require memory transfers on supported ABIs.
```

Keep more exact measured costs in the relevant design explanation with
their build and target context. Check the claimed ABI behavior before
restating it as a general property.

### Make the target table serve a choice

Replace paragraph-sized notes with a compact structure such as:

| Family | Verified boards | Support status | Guide |
| --- | --- | --- | --- |
| Family name | Named board or a short linked list | Existing verified status and important restriction | Target guide |

This is a structural example, not factual replacement data. Preserve the
repository's actual maturity categories and the meaningful distinction
between compilation coverage and hardware verification.

### Give a procedure an observable result

Weak instruction:

> Select the preset and debug as usual.

Better structure:

> Select the CMake project for your board's family, then its debug preset.
> Select the application executable for the debug session. Start the named
> launch configuration in Run and Debug. A successful session stops at the
> documented initial breakpoint and displays the source location.
>
> If the session selects an executable from another family, check the active
> CMake project and debug executable before troubleshooting the probe.

Replace the general labels with the actual verified interface names and
the expected behavior of that launch configuration. Do not promise a stop
at main if that configuration does not request one.

## Execution plan: deliver reviewable batches

### Inspect and establish the baseline

Read applicable instructions and inspect the working tree. Preserve unrelated
user edits. Inventory Markdown, public headers, examples, target guides,
build presets, and VS Code configuration. Read representative pages in full;
use code and tests to resolve claims without attempting unavailable builds.

Record the cloud baseline, the starting commit, the batch scope, checks
available, and existing failures. Keep a short persistent progress record
outside reader-facing docs, committed with the temporary brief. Track areas
reviewed and rewritten, unresolved facts, reader-test findings, and next
steps. Use descriptive names rather than new stage codes.

The reported inventory is approximately 269 Markdown files and 348 headers;
recount rather than treating these numbers as fixed. Plan around coherent
reader tasks and reviewable changes, not a target number of edited files.
Do not use the available token budget as a reason to generate an enormous
unreviewed diff. Spend additional effort on evidence and reader testing.

### First batch: establish the complete introductory model

Include the root README, documentation entry page, a coherent learning route,
a VS Code guide, and the opening/API comments of the kernel and services
that route uses. Do not rewrite every kernel header merely because it is
nearby. Record an explicit file scope and explain necessary dependency fixes.

Use existing blink as the first tutorial, including its Supervisor rather
than an imaginary button input. Choose and name one concrete board/application
combination based on the available source,
board description, and setup evidence. Explain that choice in the work
record; a routine evidence-based choice does not need another permission
round. Do not assume all Nucleo, WeAct, or WCH EVT boards have equivalent
buttons, LEDs, pin mappings, or debug arrangements.

Do not settle the open portable board-file architecture as part of this
editorial task. No new executable drafts are requested in this pass.
If no existing example supports a claimed action, narrow the tutorial to
what exists and identify the missing teaching step honestly. Do not defer
useful documentation work while waiting for that future example.

The Nucleo-G0B1RE is a reasonable default for the STM32G0 blink route if its
existing board guide and launch configuration support the described steps.
This is a documentation choice, not a claim that the user has selected or
verified this board interactively. Name the assumed configuration explicitly;
do not ask for a board choice merely to begin explaining existing source.
The optional board button is not part of this application's behavior.

Revise only the relevant editorial paragraphs in CLAUDE.md and
docs/README.md: audience, teaching examples versus disposable bench apps,
plain language, comments, and verification status. Preserve bench operations,
engineering policy, and unrelated decisions. If those paragraphs have
already been revised locally, use them and avoid duplicating the work.

Run the independent reader review below, revise the weak passages, perform
available checks, and deliver a pushed branch/PR. Stop for the user's review
of this first batch. This checkpoint is intentional: it calibrates the model
before hundreds of files are changed. Do not start subsequent batches until
the first-batch approach is accepted. Preserve an actionable continuation
record so review does not require restarting the analysis.

### Subsequent batches after the first review

Use a separately reviewable PR per coherent batch, based on the accepted
writing pattern. A reasonable sequence is:

- Conceptual design pages, split further by subject when needed.
- One representative target stratum, including its entry page, selected
  peripheral pages, and associated comments, as a model for target material.
- Remaining target strata in manageable groups, retaining their hardware
  differences rather than mechanically copying the model.
- Remaining services, devices, host material, board/probe guides, and
  reader-facing examples not already covered.

Keep each PR independently understandable. If PRs depend on one another,
state the dependency and use the appropriate base; do not duplicate changes
or assume an unmerged PR has reached main. Do not merge any PR yourself.
Later batches can continue under the user's agreed review cadence without
requesting approval for each paragraph. The first-batch review is the
required calibration point; further stops follow user steering.

Track coverage so the introductory batch is not confused with completion of
the repository-wide task. Record justified exclusions and unresolved facts.
Keep precise technical reference reachable throughout the migration.

### Validate each batch and hand it off

Inspect and use `bin/brio prose` and the token check for changed source
files. Do not run bare `bin/brio gate`: its image-comparison mode requires
builds unavailable in the cloud. The token check needs the correct reference
commit: compare against the batch base, not only the latest intermediate
commit. Inspect syntax for `--against` and explicit file arguments before
running it. Do not use `--strings` for a comments-only identity claim.

Review tool-interpreted comments separately. Build annotations, test
selection directives, documentation directives, macro continuations, and
metadata can change behavior even if a token comparison passes. Preserve
these during editorial edits. List any deliberate exception; do not conceal
one as harmless prose.

Check links and heading destinations, all incoming references to renamed
sections, example/API consistency, navigation, ASCII, and the final diff.
Do not weaken a checker merely to obtain a pass. Report pre-existing failures
separately from failures introduced by the batch.

Prepare a compact local validation handoff: which existing examples and
configurations need compilation, which GUI sequences need an interactive
check, the expected outcomes, and any excerpt needing a local consistency check. Do not claim
that existing historical verification validates freshly changed material.

Finish each batch with an Italian summary linking starting pages and the
PR/branch, giving scope, actual check results, reader-review results, and
remaining local verification. Preserve the temporary work record until the
continuation is safe, and identify work artifacts to exclude before merge.

## Independent reader review and calibration

Self-review is necessary but insufficient: the author remembers meanings
that the page may never explain. Use a fresh subagent as a simulated reader.
This brief explicitly requests that limited delegation. It is a diagnostic
proxy for the intended reader, not proof that a real beginner understands.

Give the reader only the intended audience profile, the page or small
learning route under test, and the questions below. Do not provide author
notes, the development conversation, this brief, implementation files, or
answers. Use a fresh context without inherited conversation. Ask the reader
to avoid filling gaps from its own knowledge and to cite supporting passages
for every answer. If it needs another document, it should follow the links
actually supplied to the user and record that navigation.

Ask, in ordinary language:

1. What is this component or feature, and why would I use it?
2. How would I use it for the task the page promises? Give the steps and
   explain the unfamiliar notation needed by the example.
3. Where is the trap: what mistake could cause incorrect behavior, waiting,
   lost data, invalid buffer access, or a failed setup?
4. Which facts or steps could you not establish from the supplied route?

For a scheduling page, add a scenario: an interrupt posts an event for a
high-priority object while a lower-priority handler is running. Ask what
happens next and what information supports that answer. For a buffer page,
ask when storage can be reused. For a GUI guide, ask how to recognize the
wrong active project or executable and how success is observed.

Record whether each essential answer is supported directly, requires an
unexplained inference, or cannot be established. Record the exact confusing
phrase and its consequence. Do not turn this into an arbitrary readability
score. Missing prerequisites, unsafe lifetime answers, or a false scheduling
model are substantive failures even if the prose sounds fluent.

Revise and repeat with a fresh reader when failures are substantive. Keep
iterations bounded: normally two or three focused rounds per sample are
more informative than repeated whole-repository reviews. If a technical
fact remains unresolved, disclose it instead of coaching the reader to pass.
If subagents are unavailable, report the missing independent review; do not
label author self-review as independent.

Use the first-batch user review to calibrate tone by contrast. Invite concrete
examples of a passage the user finds clear and one they find obscure. If
such examples are already supplied, use them; otherwise perform the first
batch without blocking on that optional input. Record what makes the clear
passage work and revise the obscure one. An assistant's preferred style is
not a substitute for the user's intended audience.

For example, if a reader repeats "the sink precedes the producer" but cannot
say which object goes first in Tenuto's list or when the buffer expires, the
revision has failed. Add the ordering and lifetime explanation at the point
of use, then test the scenario again without telling the next reader the
answer.

## Acceptance review

Assess pages by what the reader can do after reading them. A passing prose
check and a shorter word count do not establish success.

For the introduction and tutorial, verify that:

- The first paragraphs explain the purpose and audience without a wall of
  architecture or C++ terminology.
- The execution model is understandable before its template implementation.
- The reader can trace one concrete event from origin to handler to output.
- Priority, interrupt behavior, waiting, and persistent state are explained.
- Every unfamiliar C++ feature needed for the example is introduced locally.
- The tutorial example has all required setup and expected results, with
  compilation and hardware verification status stated accurately.
- The hardware and tool prerequisites are explicit and reproducible.
- The available VS Code route is documented with honest verification status.
- A fresh reader can answer the essential questions from the supplied pages
  without reconstructing the development conversation.

For reference pages and source comments, verify that:

- A reader can find purpose, configuration, context restrictions, failure
  behavior, ownership, and lifetime without searching through a manifesto.
- Exact names and important technical qualifications remain intact.
- Tables support comparison rather than contain miniature chapters.
- Links name their destinations; tool codes are paired with meanings.
- Measured findings retain their relevant conditions and provenance.
- Current guarantees are separated from plans and assumptions.
- A simplified introduction still leads to the full technical explanation.

For the overall change, verify that:

- The rewritten rules will guide the next contributor toward the same
  audience instead of restoring the old style.
- The agreed batch scope has been reviewed, and overall coverage, remaining
  batches, exclusions, and unresolved facts are explicitly recorded.
- Existing useful technical information remains discoverable.
- No executable behavior or tool-interpreted comment metadata changed as
  an incidental effect of rewriting prose.
- Validation results and interactive verification limits are stated plainly.

## Common ways this task can fail

- Cosmetic editing: shorter sentences with the same unexplained concepts
  and the same implementation-first order.
- A glossary used to avoid explaining a concept in context.
- A polished README leading into unchanged, inaccessible chapters.
- A tutorial presented as runnable although its quoted application is incomplete
  or outdated, or a new example developed despite this pass being editorial.
- A claimed graphical workflow inferred from a command line test.
- A beginner guide that hides blocking, ownership, overflow, or timing costs.
- A generic C++ lesson that never helps the reader understand brio.
- A simplified source comment that deletes the reason for a critical
  ordering constraint or hardware workaround.
- New internal codes and elaborate editorial checklists appearing in the
  reader's documentation.
- A plan or sample delivered as if it completed the repository-wide rewrite.

The desired result is a technically reliable body of documentation that
teaches an embedded programmer enough to use brio, then gives them a clear
route to the details they need. Make every editorial choice serve that
reader.
