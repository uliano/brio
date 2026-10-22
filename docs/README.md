# brio documentation

Documentation for the `brio` framework and the multi-app testbed around
it, as they are today. It is written for an embedded programmer who
knows C and microcontrollers and may know little C++ and nothing about
active objects; it serves two readings. The LEARNING route (`learn/`)
takes that reader from the main loop they know to a working brio
program, one idea at a time, and to the graphical and command-line
workflows that build, program and debug it. The REFERENCE (`design/`,
one folder per target and per shared stratum, `boards/`, `probes/`)
holds the rationale, the contracts between layers and the measured
facts, and is where the learning route sends a reader who needs more.

## Map

The learning route: the ideas in order, then the running of the
example by the route you prefer.

| Document | Content |
|----------|---------|
| [learn/README.md](learn/README.md) | The route's table of contents, where it leads afterwards, and the teaching examples it does not have yet |
| [learn/introduction.md](learn/introduction.md) | What problem brio solves and how its program runs, before any C++: events, queues, active objects, the one scheduling rule, what the model costs |
| [learn/first-program.md](learn/first-program.md) | The maintained example `blink` read in stages, from the events to the target glue |
| [learn/cpp-notation.md](learn/cpp-notation.md) | The C++ the example uses, explained through its own lines; a companion to the first program |
| [learn/command-line.md](learn/command-line.md) | Build, flash, observe and debug `blink` on the ST Nucleo-G0B1RE from a terminal, and the host tests |
| [learn/vscode.md](learn/vscode.md) | The same steps in VS Code, each marked verified, configured, terminal-only or blocked |

The reference mirrors the strata of `brio/`: `design/` is the
target-independent framework; one folder per target (`avrxt/`,
`samc21/`, `stm32g0/`, `stm32f4/`, `ch32v00x/`, `ch32vx03/`,
`ch32x035/`, `rp2040/`, `rp2350/`, `host/`) holds that target's
operational page (`README.md`), one document per peripheral driver and
its vendor documents; and one folder per stratum that is not a target
(the core stratum `cortexm/`, the IP strata `pl011/`, `pl022/`,
`dw_apb_i2c/`, and `devices/` for what sits off the chip). Within each,
the kernel's ideas are settled enough to build on, the services and
drivers are here to stay but will change as targets are added, and
the bench is disposable.

Target-independent design, one page per decision:

| Document | Content |
|----------|---------|
| [design/overview.md](design/overview.md) | The philosophy, the governing rule, the layering, naming and style: what every other page is written under |
| [design/kernel.md](design/kernel.md) | The active-object kernel by intent: the contract an active object meets, events and payloads, the queue, the state machine, delivery, the scheduler, time, panic, the platform |
| [design/runtime.md](design/runtime.md) | What a freestanding 32-bit image needs of a runtime and why brio writes those seven functions itself, with no C library; why the AVR stays on avr-libc |
| [design/benchmark.md](design/benchmark.md) | The instrument every driver's cost is measured with: the one line per operation, the rulers, the protocol against the vendor's library |
| [design/clock.md](design/clock.md) | The clock model: one rate truth, static and dynamic regimes, and how a rate change reaches every driver |
| [design/serial.md](design/serial.md) | The serial stack: the UART driver below, line events and `print` above, and the run as the unit of a transport |
| [design/spi-bus.md](design/spi-bus.md) | The shared SPI bus: the engine descriptor, arbitration between active objects, the rules for several devices on one bus |
| [design/i2c-bus.md](design/i2c-bus.md) | The I2C bus over the same arbiter: the engine descriptor and the status vocabulary |
| [design/can.md](design/can.md) | The CAN vocabulary: the frame, the bit timing in human units, the error state, and why there is no bus active object |
| [design/nv-heap.md](design/nv-heap.md) | NvHeap: blocks of flash allocated at run time that outlive the program, and how a power cut during a write is survived |
| [design/nv-journal.md](design/nv-journal.md) | NvJournal: a handful of small values kept in flash on a part with no EEPROM, with a reserve a panic handler can still write |
| [design/usb.md](design/usb.md) | USB, the device side: the contract at the packet, the control endpoint and the standard requests written once, CDC ACM as a byte transport |
| [design/power.md](design/power.md) | The power model: the sleep-depth ladder, the votes among stakeholders, and what a program may assume on the first event after a wake |
| [design/dma.md](design/dma.md) | What a program is owed of a DMA engine whatever the controller: the priority levels and the release contract |
| [design/ring.md](design/ring.md) | Ring: the single-producer single-consumer FIFO, and the reading side of a ring a DMA channel fills |
| [design/analog.md](design/analog.md) | Analog sampling in the kernel: the converter concept, attribution of results, the counts-to-millivolts arithmetic |
| [design/meters.md](design/meters.md) | Meters: bridging a capture interrupt to the loop, and pacing publication rather than capture |
| [design/block-stream.md](design/block-stream.md) | Block streams: caller-owned buffers moved in blocks, and the relay that lends filled blocks for one dispatch |
| [design/gfx.md](design/gfx.md) | Graphics: the three kinds of surface, the write-only base every surface satisfies, the two verbs panels are good at, and how a renderer is checked |
| [design/simulation.md](design/simulation.md) | Simulation: the model of the world a program runs against on the host, and what the host does not tell you |
| [design/architecture.svg](design/architecture.svg) | The strata diagram |
| [design/event-queue.svg](design/event-queue.svg) | The event queue in one state: the slots, the ring of slot numbers, its three runs and three cursors |

The targets, each with its front page (toolchain, board, probe,
debugger and their quirks, then the map of that target's own
documents), and the strata that sit between `util/` and the families:

| Target | Front page |
|--------|------------|
| AVR DA, DB and EA (`brio/avrxt/`) | [avrxt/README.md](avrxt/README.md) - the self-built boards, the Atmel-ICE, PyAvrOCD debugging, the clock fixture |
| SAM C21 (`brio/samc21/`) | [samc21/README.md](samc21/README.md) - the vendored device pack, the self-built board, OpenOCD over SWD, the family smoke check |
| STM32G0 (`brio/stm32g0/`) | [stm32g0/README.md](stm32g0/README.md) - the three Nucleo boards, ST-LINK upload and debug and their caveats, the family coverage over the x1 and x0 lines |
| CH32V00x (`brio/ch32v00x/`) | [ch32v00x/README.md](ch32v00x/README.md) - WCH's compiler, the two boards and the part table, the WCH-Link, the console on the probe's own serial, what the QingKe V2 core taught the stratum |
| CH32V203 / CH32V303 (`brio/ch32vx03/`) | [ch32vx03/README.md](ch32vx03/README.md) - the two series under two ABIs, the thirteen parts the build states, the two-wire debug port and its reset verb, what the silicon taught the stratum |
| CH32X035 (`brio/ch32x035/`) | [ch32x035/README.md](ch32x035/README.md) - one die in seven packages, WCH's evaluation board, the debug port on PC18/PC19, what the documents and the silicon taught the stratum |
| RP2040 (`brio/rp2040/`) | [rp2040/README.md](rp2040/README.md) - the vendored device description, the boot stage, the Debug Probe and which OpenOCD the flash chip demands, two cores with a kernel each |
| RP2350 (`brio/rp2350/`) | [rp2350/README.md](rp2350/README.md) - two processor architectures over one set of peripherals: the two toolchains, the four presets, the image block that decides which pair runs, the one OpenOCD that examines all four cores |
| STM32F4 (`brio/stm32f4/`) | [stm32f4/README.md](stm32f4/README.md) - the hard-float ABI, the four boards, ST-LINK upload, the family coverage over twenty-three headers |
| the Cortex-M core stratum (`brio/cortexm/`) | [cortexm/README.md](cortexm/README.md) - what the Cortex-M families share whatever the vendor, and what stays per family |
| the PL011 IP stratum (`brio/pl011/`) | [pl011/README.md](pl011/README.md) - ARM's PrimeCell UART written once: what is ARM's, what a family owes it, and the rule that births an IP stratum |
| the PL022 IP stratum (`brio/pl022/`) | [pl022/README.md](pl022/README.md) - ARM's PrimeCell SSP the same way: the SPI host engine, the client, the framings |
| the DesignWare I2C IP stratum (`brio/dw_apb_i2c/`) | [dw_apb_i2c/README.md](dw_apb_i2c/README.md) - Synopsys's DW_apb_i2c the same way: the command FIFO, the host engine, the client |
| the off-chip devices (`brio/devices/`) | [devices/README.md](devices/README.md) - what sits off the chip over a link the chip provides: the DCS vocabulary, a traits type per display controller, a driver judged against a simulator and against the bench |
| host (`brio/host/`) | [host/README.md](host/README.md) - the native test target: HostPlatform, the doctest suites, what a simulation is allowed; and [host/simulator.md](host/simulator.md), the contract between a host program and a viewer watching it |
| the boards | [boards/README.md](boards/README.md) - how a board joins the bench, then one page per board brio is tested on |
| the probes | [probes/README.md](probes/README.md) - the flash mechanisms `bin/brio` knows, then one page per probe |

## Rules of this directory

- **Written for the reader named above, in ordinary technical
  English.** A page answers, in this order: what problem this solves in
  an embedded program, what the program does and when, what the
  programmer must provide or avoid, the smallest useful example, where
  the exact API and implementation details are. A reference page may be
  compact but still opens with that orientation; a learning page builds
  it in full. Direct subjects and verbs; one idea per paragraph; a term
  of the project's own (active object, run-to-completion, contract,
  stratum, a CLI verb) is defined where it first appears and then used
  consistently, and a private shorthand that compresses a meaning only
  its author knows is replaced by the meaning. Capitals mark nothing;
  bold marks a consequence the reader must remember, a correctness
  rule first of all. A link names its destination, never "section 7"
  or "see above"; where a tool needs a letter or a code, the code
  stands beside its meaning. A table compares short parallel facts; a
  causal explanation is prose. Precision is never traded for a
  simplification the code does not support: the conceptual rule, what
  the compiler checks, what a runtime check catches, what stays the
  programmer's duty and what was measured on which hardware are kept
  apart, and a measured cost names its build, clock and part.
- **A step is verified, configured or unavailable - and says which.**
  A workflow page states, for every step, whether it was performed on
  the named configuration, is configured but awaits an interactive
  check, exists only on the command line, or is blocked and why. A
  configuration file is evidence to examine, not proof that a session
  succeeds; a result obtained on one board is not a claim about another.
- **Teaching examples are maintained applications; bench apps are
  disposable.** A MAINTAINED TEACHING EXAMPLE is an application the
  documentation reads and explains (`blink` today, in every project
  that builds it): it has a stable documented purpose, keeps compiling
  on every preset of its project, and is re-read when an API it uses
  changes - the documentation that quotes it moves in the same change.
  A BENCH APP (a test suite, a probe, a console, an experiment's half)
  is disposable and free to change without touching the foundations;
  a document never depends on one, with the apparatus exception below.
  A page may name and quote a maintained teaching example and shows the
  reader where its source is; an example a page would like to have and
  has not got is stated as a gap in that page's own words, never
  described as if it were available, and the learning route documents
  the applications that exist rather than inventing ones that do not.
- **Keep it true or delete it.** A design doc that lags the code is
  worse than no doc. Whoever changes a documented decision updates the
  doc in the same change (same commit when practical).
- **Docs hold the WHY and the contracts; headers hold the API.** The
  canonical reference for any type or function is its header comment -
  do not duplicate signatures or parameter lists here, link to the
  header instead. If a browsable API reference is ever wanted, Doxygen
  over the headers generates it without touching this directory.
- **First principles only in design/ and the target folders.** They
  state principles, contracts and tradeoffs - they never describe or
  reference a bench app (disposable, free to change without touching
  the foundations); a maintained teaching example may be named and
  quoted, under the rule above. Apps document themselves in their own
  header comment; `brio apps` lists them.
  THE ONE EXCEPTION FOR BENCH APPS IS APPARATUS: a document may name the firmware a
  reader needs in order to reproduce a measurement it reports - a
  reference suite (`test_<target>_<subject>`, which must keep passing)
  and the peer firmware that suite talks to. A probe written to answer
  one question is NOT apparatus: the answer is in the document and the
  probe may be deleted tomorrow, so the document states the measurement
  and not the binary that took it.
- **One folder per target, mirroring `brio/<name>/`.** Its
  `README.md` is the operational page - toolchain, probe, debugger,
  quirks, clock fixture; next to it one document per peripheral driver
  and `vendor/` with the datasheets of record. What is
  target-independent (models every target realizes) stays in design/;
  a reader of `docs/<target>/` sees at a glance what is that
  target's.
- **Plain Markdown, ASCII only, English** (project-wide rules). No
  generator-specific syntax: every file must render on GitHub as-is.
- **MkDocs-ready by construction.** If/when a website is wanted:
  `pip install mkdocs-material`, drop a 10-line `mkdocs.yml` at the
  repo root pointing at this directory, `mkdocs serve`. Nothing here
  needs rewriting for that - which is exactly why nothing here may
  depend on it.
- **Every peripheral driver has its document, and every document ends
  with what it does not cover.** Maturity is a property of the
  PLATFORM, stated once in the repository README's target table
  (`supported` / `in bring-up`): no document carries a banner of its
  own. Each document closes with "Not covered yet" as two lists -
  DRIVER GAPS (the chapter's features the driver does not implement,
  each with its REASON: needs a wire, a peer board, a meter, a supply;
  born with its first user; declined because ...) kept distinct from
  IMPLEMENTED BUT NOT BENCH-VERIFIED (each with what would measure
  it). A gap with no reason is the only kind a reader should worry
  about; a "gap" the document's own findings already cover is not a
  gap and is deleted; and a document with nothing in either list has
  no such section - that absence is its statement of completeness.
- **One document per peripheral, in this shape.** It opens with the
  reader's orientation: what the driver is for in a program, the
  smallest use, and any prerequisite that decides correctness even in
  that smallest use (a clock to bring up first, a pin to hand over, an
  ordering in the active-object list). Then the documents of record
  with their revision (data sheet, errata), the driver header, the
  reference test suite - no chapter lists, no history. Then: *what
  the silicon does* (the behaviour and the
  physical facts that matter to code, measured ones marked as such -
  and THE SILICON'S OFFER: every feature that bears on cost, FIFOs and
  their thresholds, DMA requests and circular mode, the edges that end
  a run, byte counters and automatic bus conditions, set/clear aliases,
  timer widths, the instruction fetch, each one used by the driver or
  declined with its reason, design/overview.md's "A driver is written
  from its chapter"),
  *types and verbs* (a systematic inventory of the configuration knobs
  - name, values, default, effect - the input/resource types, and the
  verbs by purpose: names and meaning, never signatures), *how to use
  it* (one example per way of using it - what to write, since readers
  want the call, not the header), *bench findings* (the
  facts the test suite established, with its name - among them THE
  COST of each transport or engine: cycles per byte or frame,
  interrupts per byte, cycles masked, beside the wire's or the bus's
  own figure; a measured rate far from that figure is explained there
  or listed in "Not covered yet" as a gap to explain). Tracks and
  history live in CLAUDE.md and memory, never here.
- **A contract with more than one realization carries a realizations
  table.** In the design page that owns the contract, `### Realizations`
  right after the contract's statement: one sentence for what is
  common, then one row per stratum in a fixed order with `host` last -
  the realization (header and type) and
  ONLY what lies beyond the contract there, `-` for nothing, an absent
  realization a row too, with its reason. Names are strata, never
  boards. A spelling of the same function is recorded as a spelling;
  a different function under a shared name as a trap. The index of
  every table is in `design/overview.md` ("One interface where it
  can"); headers never cite another stratum's - the table is the one
  home of the cross-target view.
- **Today's truth only, no change history.** A doc says what is,
  never what it used to be, when it was reorganized or what something
  was called before. Rationale and rejected alternatives are welcome
  (they are the WHY); dates, renames and "since ..." notes are not.
  When the project earns versioning, changelogs will be their own
  documents.

## What does NOT belong here

- The desk itself - which board is plugged in where, its incidents
  and end states: the user's own private notes, never this directory.
  What a board IS lives in `boards/`, what a probe does in `probes/`,
  what a suite needs wired in the suite's own header comment.
- The assistant's working notes: `CLAUDE.md`.
