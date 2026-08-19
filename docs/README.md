# brio documentation

Design documentation for the `brio` framework and the multi-app testbed
around it. This directory is the narrative companion to the code: the
decisions, the rationale, the contracts between layers.

## Map

Ordered by stability - the kernel's ideas are settled enough to build
on, the services and drivers are here to stay but will change with
the second target, the bench is disposable:

| Document | Content |
|----------|---------|
| [design/overview.md](design/overview.md) | Philosophy, governing rules, layering, naming and style |
| [design/kernel.md](design/kernel.md) | The active-object kernel, by intent: model, AO contract, events and payloads, queues, FSM, delivery, scheduler, time, panic, platform - with C++ notes |
| [design/serial.md](design/serial.md) | The serial stack: Uart driver below, SerialPort line events above |
| [design/spi-bus.md](design/spi-bus.md) | The shared SPI bus: engine descriptor, AO arbitration, multi-device rules |
| [design/i2c-bus.md](design/i2c-bus.md) | The I2C bus: BusMaster generalized, the TWI engine descriptor, status vocabulary |
| [design/ring.md](design/ring.md) | Ring: the SPSC FIFO, lock-free where the platform allows, guarded elsewhere |
| [design/events.md](design/events.md) | The event system: what the silicon offers, typed vocabulary + run-time connect/disconnect + static allocation as sugar (design fixed, tables on demand) |
| [design/clock.md](design/clock.md) | The clock model: one rate truth, static and dynamic regimes, the synchronous rebase fan-out and its two compile-time checks |
| [targets/avrdx.md](targets/avrdx.md) | AVR DA/DB: toolchain, board, Atmel-ICE upload, PyAvrOCD debugging and its quirks, clock/timebase |
| [targets/host.md](targets/host.md) | The native test target: HostPlatform, doctest suites |
| [bench.md](bench.md) | The board, the wiring and the apps as they are today (volatile) |
| [vendor/README.md](vendor/README.md) | Which datasheets/errata the target strata are written against, by document number (PDFs kept local, not in git) |

## Rules of this directory

- **Keep it true or delete it.** A design doc that lags the code is
  worse than no doc. Whoever changes a documented decision updates the
  doc in the same change (same commit when practical).
- **Docs hold the WHY and the contracts; headers hold the API.** The
  canonical reference for any type or function is its header comment -
  do not duplicate signatures or parameter lists here, link to the
  header instead. If a browsable API reference is ever wanted, Doxygen
  over the headers generates it without touching this directory.
- **First principles only in design/.** Design docs state principles,
  contracts and tradeoffs - they never describe or reference individual
  apps (apps are disposable and must be free to change without touching
  the foundations). Apps document themselves in their own header
  comment and are listed in `bench.md`.
- **One page per target in targets/.** Everything operational about a
  target - toolchain, probe, debugger, quirks, clock fixture - lives
  in `targets/<name>.md`, mirroring `lib/brio/src/<name>/`. Design
  reasons stay in design/.
- **Plain Markdown, ASCII only, English** (project-wide rules). No
  generator-specific syntax: every file must render on GitHub as-is.
- **MkDocs-ready by construction.** If/when a website is wanted:
  `pip install mkdocs-material`, drop a 10-line `mkdocs.yml` at the
  repo root pointing at this directory, `mkdocs serve`. Nothing here
  needs rewriting for that - which is exactly why nothing here may
  depend on it.
- **Today's truth only, no change history.** A doc says what is,
  never what it used to be, when it was reorganized or what something
  was called before. Rationale and rejected alternatives are welcome
  (they are the WHY); dates, renames and "since ..." notes are not.
  When the project earns versioning, changelogs will be their own
  documents.

## What does NOT belong here

- Bench diary and hardware bring-up state beyond the current wiring
  (`bench.md`): session memory.
- The assistant's working notes: `CLAUDE.md`.
