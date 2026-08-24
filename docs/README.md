# brio documentation

Design documentation for the `brio` framework and the multi-app testbed
around it. This directory is the reference companion to the code: the
rationale and the contracts between layers, as they are today.

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
| [avrdx/platform.md](avrdx/platform.md) | Platform (provisional): what the kernel stands on - `AvrPlatform` (critical section, idle and the SLPCTRL erratum, timebase, atomic_width, the `.noinit` breadcrumb), the short-wait role with both delay paths measured, and `Reset`/`Watchdog` over RSTCTRL and the WDT; not covered: the Standby/Power-Down modes and the regulator knobs that go with them |
| [avrdx/clkctrl.md](avrdx/clkctrl.md) | CLKCTRL (provisional): oscillators, PLL, main clock mux/prescaler/CLKOUT, clock failure detection (DB) as resources; Clock/DynamicClock as tasks, DA external clock datasheet-trusted; not covered: the unbenched paths (XOSC32K, PLL, DA silicon) |
| [avrdx/evsys.md](avrdx/evsys.md) | EVSYS (provisional): the full typed vocabulary (generators and users, package-gated) + run-time connect/disconnect; not covered: static allocation, the vocabulary no driver exercises yet |
| [avrdx/vref.md](avrdx/vref.md) | VREF: the reference selector - levels, headroom, how the ADC/DAC name it |
| [avrdx/dac.md](avrdx/dac.md) | DAC: the 10-bit actuator - buffered/unbuffered outputs, the slow fall on a bare pin, usage |
| [avrdx/adc.md](avrdx/adc.md) | ADC (provisional): one task with knobs - inputs as types, triggers, accumulation, window (signed too), results as events, DB-only inputs gated; not covered: pin-level input legality, the standby paths |
| [avrdx/userrow.md](avrdx/userrow.md) | USERROW: the board identity label - survives chip erase, written once over UPDI, read into every suite banner by board_id() |
| [avrdx/port.md](avrdx/port.md) | PORT (provisional): Port<L> resource, Pin (one-store PinConfig, senses, flags), PinSet across ports on the multi-pin engine, PinRef; not covered: INLVL/slew measurements, the fully-async wake |
| [avrdx/usart.md](avrdx/usart.md) | USART (provisional): the `Usart<n>` resource - routes incl. the pinless one, every frame format, the receiver modes, the errata verbs - and the tasks over it (`Uart`, `OneWire`, `Rs485`, `SyncHost`/`SyncClient`, `MspiHost`, `IrdaLink`, `AutoBaud`); not bench-verified: everything that needs a second board |
| [avrdx/spi.md](avrdx/spi.md) | SPI (provisional): the `Spi<n>` resource - the per-package route table with the errata that beats it, both roles, the seven rates, both INTFLAGS layouts and their clear disciplines, the host demotion - and the tasks over it (`SpiHost` transfer engine, `SpiClient`); not bench-verified: everything that needs a second device on the wire |
| [avrdx/twi.md](avrdx/twi.md) | TWI (provisional): the `Twi<n>` resource - the per-package route table with its dual pin pairs, the three errata as code, the chapter's own baud arithmetic with the bus's edges as arguments, both halves - and the tasks over it (`TwiHost` transfer engine, `TwiClient` including Dual mode); not bench-verified: everything that needs a second, independent device on the wire |
| [avrdx/rtc.md](avrdx/rtc.md) | RTC/PIT (provisional): RtcClock/Rtc/Pit resources (one clock for both functions, counter with period and compare, crystal error correction, the busy flags) + BasicTicker over the PIT; not covered: the crystal and external-clock sources, the standby and debug-run paths |
| [avrdx/tca.md](avrdx/tca.md) | TCA (provisional): the Tca resource (normal mode, buffered compares, event inputs, commands) + tasks TcaPwm/TcaPwm16/FrequencyGenerator/Heartbeat/EventCounter; not covered: the split halves' counters as verbs |
| [avrdx/tcb.md](avrdx/tcb.md) | TCB (provisional): the Tcb resource (eight modes, event clock/capture, cascade, routes) + tasks PeriodicTick/Timeout/OneShotPulse/PulseCounter/CascadedCounter/meters/Pwm8; not covered: pin-level bonding within a port |
| [avrdx/tcd.md](avrdx/tcd.md) | TCD (provisional): the `Tcd<0>` resource - the full chapter with its three synchronization disciplines enforced by the verbs, the per-package route table, the input-mode validity table and the errata that shrink it, the 12-bit captures and their read discipline, dithering - plus `TcdPwm`, the complementary pair with dead time, and the PLL made observable through it; not covered: the usage types waiting for their first user, the external clock source, two errata that would not reproduce on this die |
| [avrdx/ccl.md](avrdx/ccl.md) | CCL (provisional): Ccl + Lut<n> resources (inputs menu, truth table, filter/edge, clocks, pins, the whole-block reconfiguration erratum) + ToggleFlipFlop; not covered: typed per-input instance legality |
| [avrdx/ac.md](avrdx/ac.md) | AC (provisional): the Ac<n> resource (inputs and DACREF, hysteresis/power, pin/event/interrupt, window) + Threshold/Window; not covered: pin-level bonding (PD0, PC6 on small packages) |
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
- **Every peripheral driver has its document; only incomplete ones
  are marked.** A driver that does not yet cover its chapter's full
  option space opens with a PROVISIONAL banner and closes with "Not
  covered yet" - the chapter's features the driver does not implement
  (driver gaps) kept distinct from what is implemented but not yet
  bench-verified. A complete doc carries no banner and no gap list:
  absence of the banner IS the statement of completeness. Never mark
  a doc complete while it still lists gaps.
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
