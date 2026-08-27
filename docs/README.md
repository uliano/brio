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
| [design/nv-heap.md](design/nv-heap.md) | NvHeap (provisional): runtime-allocated blocks of flash that outlive the program - the FlashMedia contract, the ping-pong map pair and its atomicity, survival-aware mount, the placement rule, the wear accounting |
| [design/power.md](design/power.md) | The power model: the sleep-depth ladder, the site that only arms (so the kernel loop's own idle path does the sleeping), the round of votes among stakeholders, standing restrictions for the ones that live in interrupts, the deadline guard, and the first-event-after-wake contract |
| [design/ring.md](design/ring.md) | Ring: the SPSC FIFO, lock-free where the platform allows, guarded elsewhere |
| [design/analog.md](design/analog.md) | Analog in the kernel: the AnalogSampler usage type (converter concept, attribution by reported code, two paces, owner's duties) and the counts <-> mV arithmetic |
| [design/meters.md](design/meters.md) | Meters in the kernel: the MeterLatch that bridges a capture interrupt to the loop (last value wins, overwrites counted) and the MeterSampler that paces PUBLICATION instead of capture - a stale source publishes nothing |
| [design/architecture.svg](design/architecture.svg) | The strata diagram |

Target AVR DA/DB (`lib/brio/src/avrdx/`):

| Document | Content |
|----------|---------|
| [avrdx/README.md](avrdx/README.md) | Toolchain, board, Atmel-ICE upload, PyAvrOCD debugging and its quirks, clock/timebase |
| [avrdx/platform.md](avrdx/platform.md) | Platform (provisional): what the kernel stands on - `AvrPlatform` (critical section, idle and the SLPCTRL erratum, timebase, atomic_width, the `.noinit` breadcrumb), the short-wait role with both delay paths measured, `Sleep`/`Vreg` over all three sleep modes and the regulator plus `AvrSleepSite`, the adapter that carries the power model onto them, and `Reset`/`Watchdog` over RSTCTRL and the WDT; not covered: idle detection and the RUNSTDBY policy, the BOD, the sleep current, and the wake-up sources a lone board cannot produce |
| [avrdx/clkctrl.md](avrdx/clkctrl.md) | CLKCTRL (provisional): oscillators, PLL, main clock mux/prescaler/CLKOUT, clock failure detection (DB) as resources; Clock/DynamicClock as tasks, DA external clock datasheet-trusted; not covered: the unbenched paths (XOSC32K, PLL, DA silicon) |
| [avrdx/evsys.md](avrdx/evsys.md) | EVSYS (provisional): the full typed vocabulary (generators and users, package-gated) + run-time connect/disconnect; not covered: static allocation, the vocabulary no driver exercises yet |
| [avrdx/vref.md](avrdx/vref.md) | VREF: the reference selector - levels, headroom, how the ADC/DAC name it |
| [avrdx/dac.md](avrdx/dac.md) | DAC: the 10-bit actuator - buffered/unbuffered outputs, the slow fall on a bare pin, usage |
| [avrdx/adc.md](avrdx/adc.md) | ADC (provisional): one task with knobs - inputs as types, triggers, accumulation, window (signed too), results as events, DB-only inputs gated; not covered: pin-level input legality, the standby paths |
| [avrdx/nvm.md](avrdx/nvm.md) | NVMCTRL (provisional): the `Nvm` resource - the four memories and what each is for, the Flash sections as a compile-time claim cross-checked against the fuses, ELPM/SPM only (never the data-space window), the command model, the one-way protections, the vector-table invariant every image carries, and the multi-page-erase erratum guarded page by page - plus the services over it (typed record, interrupt-paced writer AO, persistent panic record); not covered: a boot loader, any flash policy, DA silicon |
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
| [avrdx/opamp.md](avrdx/opamp.md) | OPAMP (provisional, **DB only**): the `OpampSystem` block (the one ENABLE, the TIMEBASE that makes a settle time mean microseconds, and the ClockUser hook that keeps it true across a rebase) + `Opamp<n>` - both input multiplexers with their per-instance link codes, the 16R ladder and its eight exact gains, the output driver, the three enable regimes, the internal timer and READY, the four event users and the offset trim - plus the tasks `OpampFollower`, `OpampPga`, `OpampInvertingPga` and the chapter's three-op-amp `InstrumentationAmp`; not covered: the integrator usage type (external R and C, and a DUMP policy), RUNSTBY, IRSEL's electrical effect |
| [avrdx/vendor/README.md](avrdx/vendor/README.md) | The datasheets/errata the stratum is written against, by document number (PDFs kept local, not in git) |

Target SAM C21 (`lib/brio/src/samc/`):

| Document | Content |
|----------|---------|
| [samc/README.md](samc/README.md) | Toolchain (vendored DFP/CMSIS, no device headers in arm-none-eabi-gcc), board, OpenOCD upload over SWD, cortex-debug, the clangd routing, the family smoke check |
| [samc/platform.md](samc/platform.md) | Platform (provisional): `SamPlatform` (PRIMASK critical section, WFI-then-enable idle, BKPT and its ARMv6-M caveat, the `.noinit` breadcrumb, atomic_width 4), `BasicTicker` over SysTick (rates dividing 1000, exact millis, the visibility clause), `Nvic`, and the hand-written crt (47-entry vector table, app-binds-the-vector, the abort() story); not covered: the sleep and reset halves |
| [samc/clock.md](samc/clock.md) | Clock (provisional): OSCCTRL/GCLK/MCLK resources with bounded waits and the two OSC48M errata as code, flash wait states ordered around the change, `Clock<internal, hz>` over the exact OSC48M ratios; not covered: every other root of the tree (XOSC, 32 kHz, FDPLL), DynamicClock |
| [samc/port.md](samc/port.md) | PORT (provisional): two groups on every variant, the off-by-default input buffer, the directional pull, the PMUX handoff, WRCONFIG; not covered: the EIC (senses and pin interrupts are another peripheral), PORT events |
| [samc/sercom.md](samc/sercom.md) | SERCOM USART (provisional): the one-vector reality and the INTFLAG-AND-INTENSET discipline, pads vs pins (TxD on PAD0/PAD2 only), the LSB-first default and its measured cautionary tale, enable-protection and both sync waits, the 16x-arithmetic baud math byte-exact on the wire, `Uart` with the AVR edge-return contract and its two OPTIONAL DMA engines (zero when absent, measured); not covered: the SPI/I2C personalities, the USART long tail |
| [samc/dmac.md](samc/dmac.md) | DMAC (provisional): the selector-guarded channel registers and the INTPEND dispatch that needs no selector, the SRAM descriptor tables, the end-address quirk decided by data against the data sheet's own disagreement, the silent disable/SWRST edge, harvest as the one window into progress, erratum 1.10.4 measured and VALIDATED AGAINST rather than trusted, the serial engines; not covered: the CRC engine, linked lists, the event hooks, standby |
| [samc/ac.md](samc/ac.md) | AC (provisional, minimal by design): the four comparators, the VDD scaler, filters, both output routings - and the measured answer ch. 40 does not give: a synchronized output edge costs the fraction to the next GCLK_AC edge PLUS TWO whole periods (STATE is the sampled path whatever OUT says, INTFLAG raises on the same period, a mid-stream edge through the majority filter pays (N-1)/2 periods, not N-1); not covered: window mode, events, sleep, per-package input legality |
| [samc/vendor/README.md](samc/vendor/README.md) | The datasheets/errata the stratum is written against, the bench chip's silicon revision, the targeted errata pass (PDFs kept local, not in git) |

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
