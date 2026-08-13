# brio documentation

Design documentation for the `brio` framework and the multi-app testbed
around it. This directory is the narrative companion to the code: the
decisions, the rationale, the contracts between layers.

## Map

| Document | Content |
|----------|---------|
| [design/overview.md](design/overview.md) | Philosophy, governing rules, layering, naming and style |
| [design/kernel.md](design/kernel.md) | The active-object kernel: events, queues, scheduler, FSM, time, panic |
| [design/spi-bus.md](design/spi-bus.md) | The shared SPI bus: engine descriptor, AO arbitration, multi-device rules |
| [design/serial.md](design/serial.md) | The serial stack: Uart driver below, SerialPort line events above |

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
  contracts and tradeoffs - they never describe individual apps or
  walk through examples. Apps are demos and diagnostics that document
  themselves in their own header comment; device-specific bench facts
  live in the top-level README.
- **Plain Markdown, ASCII only, English** (project-wide rules). No
  generator-specific syntax: every file must render on GitHub as-is.
- **MkDocs-ready by construction.** If/when a website is wanted:
  `pip install mkdocs-material`, drop a 10-line `mkdocs.yml` at the
  repo root pointing at this directory, `mkdocs serve`. Nothing here
  needs rewriting for that - which is exactly why nothing here may
  depend on it.
- Dated decisions keep their dates (`2026-08-13`), matching the style
  of the decision log this directory grew out of.

## What does NOT belong here

- Bench diary, wiring tables, hardware bring-up state: top-level
  `README.md` (bench map) and session memory.
- Build/debug workflow (PlatformIO envs, PyAvrOCD, toolchain gotchas):
  top-level `README.md` and `CLAUDE.md`.
