# brio documentation

Design documentation for the `brio` framework and the multi-app testbed
around it. This directory is the narrative companion to the code: the
decisions, the rationale, the contracts between layers.

## Map

The directory mirrors the strata of `lib/brio/src/`: `design/` is the
target-independent framework (kernel, services, the models every
target realizes); one folder per target (`avrdx/`, `host/`, ...) holds
that target's operational page (`README.md`), one document per
peripheral driver, and its vendor documents. Within each, ordered by
stability - the kernel's ideas are settled enough to build on, the
services and drivers are here to stay but will change with the second
target, the bench is disposable.

Target-independent design:

| Document | Content |
|----------|---------|
| [design/overview.md](design/overview.md) | Philosophy, governing rules, layering, naming and style |
| [design/kernel.md](design/kernel.md) | The active-object kernel, by intent: model, AO contract, events and payloads, queues, FSM, delivery, scheduler, time, panic, platform - with C++ notes |
| [design/clock.md](design/clock.md) | The clock model: one rate truth, static and dynamic regimes, the synchronous rebase fan-out and its two compile-time checks |
| [design/serial.md](design/serial.md) | The serial stack: Uart driver below, SerialPort line events above |
| [design/spi-bus.md](design/spi-bus.md) | The shared SPI bus: engine descriptor, AO arbitration, multi-device rules |
| [design/i2c-bus.md](design/i2c-bus.md) | The I2C bus: BusMaster generalized, the TWI engine descriptor, status vocabulary |
| [design/ring.md](design/ring.md) | Ring: the SPSC FIFO, lock-free where the platform allows, guarded elsewhere |
| [design/analog.md](design/analog.md) | Analog in the kernel: the AnalogSampler usage type (converter concept, attribution by reported code, two paces, owner's duties) and the counts <-> mV arithmetic |
| [design/architecture.svg](design/architecture.svg) | The strata diagram |

Target AVR DA/DB (`lib/brio/src/avrdx/`):

| Document | Content |
|----------|---------|
| [avrdx/README.md](avrdx/README.md) | Toolchain, board, Atmel-ICE upload, PyAvrOCD debugging and its quirks, clock/timebase |
| [avrdx/clkctrl.md](avrdx/clkctrl.md) | CLKCTRL (exhaustive): oscillators, PLL, main clock mux/prescaler/CLKOUT, clock failure detection as resources; Clock/DynamicClock as tasks |
| [avrdx/evsys.md](avrdx/evsys.md) | EVSYS (exhaustive): typed vocabulary + run-time connect/disconnect + static allocation as sugar (tables on demand) |
| [avrdx/vref.md](avrdx/vref.md) | VREF (exhaustive): the reference selector - levels, headroom, how the ADC/DAC name it |
| [avrdx/dac.md](avrdx/dac.md) | DAC (exhaustive): the 10-bit actuator - buffered/unbuffered outputs, the slow fall on a bare pin, usage |
| [avrdx/adc.md](avrdx/adc.md) | ADC (exhaustive): one task with knobs - inputs as types, triggers, accumulation, window, results as events, every usage pattern |
| [avrdx/port.md](avrdx/port.md) | PORT (provisional): Pin, PinSet, PinRef; not covered: pin interrupts, slew, thresholds |
| [avrdx/usart.md](avrdx/usart.md) | USART (provisional): the 8N1 byte transport; not covered: sync, one-wire/RS-485, IrDA, LIN, auto-baud |
| [avrdx/spi.md](avrdx/spi.md) | SPI (provisional): the host engine; not covered: client mode, buffer mode |
| [avrdx/twi.md](avrdx/twi.md) | TWI (provisional): the host engine; not covered: client/dual mode, SMBus, FM+ |
| [avrdx/rtc.md](avrdx/rtc.md) | RTC/PIT (provisional): the PIT as timebase; not covered: the RTC counter |
| [avrdx/tca.md](avrdx/tca.md) | TCA (provisional): chapter reviewed, split-mode PWM task; not covered: the Tca resource, 16-bit modes, events, the other tasks |
| [avrdx/tcb.md](avrdx/tcb.md) | TCB (provisional): chapter reviewed - the eight modes, event clock/capture, 32-bit cascade, routes, errata; no driver yet |
| [avrdx/ccl.md](avrdx/ccl.md) | CCL (provisional): chapter reviewed - LUT inputs menu, truth table, filter/edge, sequencers, clocks, pins, the whole-block reconfiguration erratum; no driver yet |
| [avrdx/ac.md](avrdx/ac.md) | AC (provisional): chapter reviewed - inputs and DACREF, hysteresis/power, output pin/event/interrupt, window mode, the pin table; no driver yet |
| [avrdx/vendor/README.md](avrdx/vendor/README.md) | The datasheets/errata the stratum is written against, by document number (PDFs kept local, not in git) |

Target host (`lib/brio/src/host/`):

| Document | Content |
|----------|---------|
| [host/README.md](host/README.md) | The native test target: HostPlatform, doctest suites |

The bench:

| Document | Content |
|----------|---------|
| [bench.md](bench.md) | The board, the wiring and the apps as they are today (volatile) |

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
  in their own header comment and are listed in `bench.md`.
- **One folder per target, mirroring `lib/brio/src/<name>/`.** Its
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
- **Every peripheral driver has its document, and says whether it is
  exhaustive or provisional.** Exhaustive = written from a systematic
  review of the data sheet chapter and the errata, with a bench test
  suite; provisional = covers what the bench needed so far - it opens
  with a PROVISIONAL banner and closes with "Not covered yet", the
  chapter's features the driver does not implement. That is the state
  of the work, readable in one place.
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
