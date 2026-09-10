# brio documentation

Design documentation for the `brio` framework and the multi-app testbed
around it. This directory is the reference companion to the code: the
rationale and the contracts between layers, as they are today.

## Map

The directory mirrors the strata of `brio/`: `design/` is the
target-independent framework (kernel, services, the models every
target realizes); one folder per target (`avrdx/`, `samc21/`, `stm32g0/`, `host/`) holds
that target's operational page (`README.md`), one document per
peripheral driver, and its vendor documents. Within each, ordered by
stability - the kernel's ideas are settled enough to build on, the
services and drivers are here to stay but will change as targets are
added, the bench is disposable.

Target-independent design:

| Document | Content |
|----------|---------|
| [design/overview.md](design/overview.md) | Philosophy, governing rules, layering, naming and style |
| [design/kernel.md](design/kernel.md) | The active-object kernel, by intent: model, AO contract, events and payloads, queues, FSM, delivery, scheduler, time, panic, platform - with C++ notes |
| [design/clock.md](design/clock.md) | The clock model: one rate truth, static and dynamic regimes, the synchronous rebase fan-out and its two compile-time checks; the SAM's position (no dynamic clock, measured) and the STM32G0's dynamic clock as built: a rate as a TUPLE (root, VCORE range, regulator) in an explicit pack, the direction-aware switch around the regulator, the small fan-out CCIPR buys, the Stop restored to the current rate, the SysTick ticker's rebase and why a rescaling program is a tickless one |
| [design/serial.md](design/serial.md) | The serial stack: Uart driver below, SerialPort line events above |
| [design/spi-bus.md](design/spi-bus.md) | The shared SPI bus: engine descriptor, AO arbitration, multi-device rules |
| [design/i2c-bus.md](design/i2c-bus.md) | The I2C bus: BusMaster generalized, the TWI engine descriptor, status vocabulary |
| [design/nv-heap.md](design/nv-heap.md) | NvHeap: runtime-allocated blocks of flash that outlive the program - the FlashMedia contract, the ping-pong map pair and its atomicity, survival-aware mount, the placement rule, the wear accounting |
| [design/nv-journal.md](design/nv-journal.md) | NvJournal: a handful of small values kept in FLASH on a part with no EEPROM - two halves that ping-pong wholesale, sequence numbers deciding and CRCs judging, a read-only mount, and the panic reserve an ordinary save always leaves behind so a panic handler can write one bounded entry with no erase |
| [design/power.md](design/power.md) | The power model: the sleep-depth ladder, the site that only arms (so the kernel loop's own idle path does the sleeping), the round of votes among stakeholders, standing restrictions for the ones that live in interrupts, the deadline guard, and the first-event-after-wake contract |
| [design/ring.md](design/ring.md) | Ring: the SPSC FIFO, lock-free where the platform allows, guarded elsewhere |
| [design/analog.md](design/analog.md) | Analog in the kernel: the AnalogSampler usage type (converter concept, attribution by reported code, two paces, owner's duties) and the counts <-> mV arithmetic |
| [design/meters.md](design/meters.md) | Meters in the kernel: the MeterLatch that bridges a capture interrupt to the loop (last value wins, overwrites counted) and the MeterSampler that paces PUBLICATION instead of capture - a stale source publishes nothing |
| [design/block-stream.md](design/block-stream.md) | Block streams: the BlockSource/BlockPlayer concepts (caller-owned buffers, the accounting IS the API, blocks not DMA) and the BlockRelay AO that lends filled blocks for one dispatch - built BEFORE the second implementation on purpose, as the fixed point the next platform is measured against |
| [design/architecture.svg](design/architecture.svg) | The strata diagram |

The targets, each with its front page - the operational side
(toolchain, board, probe, debugger and their quirks) and the map of
that target's own documents, one per peripheral driver:

| Target | Front page |
|--------|------------|
| AVR DA/DB (`brio/avrdx/`) | [avrdx/README.md](avrdx/README.md) - Toolchain, board, Atmel-ICE upload, PyAvrOCD debugging and its quirks, clock/timebase; then its documents |
| SAM C21 (`brio/samc21/`) | [samc21/README.md](samc21/README.md) - Toolchain (vendored DFP/CMSIS, no device headers in arm-none-eabi-gcc), board, OpenOCD upload over SWD, cortex-debug, the clangd routing, the family smoke check; then its documents |
| STM32G0 (`brio/stm32g0/`) | [stm32g0/README.md](stm32g0/README.md) - Toolchain, the three Nucleo boards, ST-LINK upload and debug, the SWD-under-WFI caveat, and FAMILY COVERAGE: both the x1 line and the x0 value line, the stratum compiling on all twelve headers of the pack with every vector derived from peripheral presence; then its documents |
| the ARMv6-M core stratum (`brio/armv6m/`) | [armv6m/README.md](armv6m/README.md) - what the two Cortex-M0+ families share: NVIC + PRIMASK, the SysTick ticker and the microsecond busy-wait on SysTick's counter, the include-order contract, what stays per family |
| host (`brio/host/`) | [host/README.md](host/README.md) - The native test target: HostPlatform, doctest suites |
| the boards | [boards/README.md](boards/README.md) - how a board joins the bench (build by type, the manifest, `bin/brio`), then one page per board brio is tested on |
| the probes | [probes/README.md](probes/README.md) - the flash mechanisms `bin/brio` knows, then one page per probe |

## Rules of this directory

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
  reference individual apps (apps are disposable and must be free to
  change without touching the foundations). Apps document themselves
  in their own header comment; `brio apps` lists them.
  THE ONE EXCEPTION IS APPARATUS: a document may name the firmware a
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
- **One document per peripheral, in this shape.** First paragraph:
  the documents of record with their revision (data sheet, errata),
  the driver header, the reference test suite - no chapter lists, no
  history. Then: *what the silicon does* (the behaviour and the
  physical facts that matter to code, measured ones marked as such),
  *types and verbs* (a systematic inventory of the configuration knobs
  - name, values, default, effect - the input/resource types, and the
  verbs by purpose: names and meaning, never signatures), *how to use
  it* (one example per way of using it - what to write, since readers
  want the call, not the header), *bench findings* (the
  facts the test suite established, with its name). Tracks, guiding
  applications and history live in CLAUDE.md and memory, never here.
- **A contract with more than one realization carries a realizations
  table.** In the design page that owns the contract, `### Realizations`
  right after the contract's statement: one sentence for what is
  common, then one row per stratum in a fixed order (avrdx, samc21,
  stm32g0, host) - the realization (header and type) and ONLY what
  lies beyond the contract there, `-` for nothing, an absent
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
