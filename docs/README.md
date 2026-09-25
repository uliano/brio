# brio documentation

Design documentation for the `brio` framework and the multi-app testbed
around it. This directory is the reference companion to the code: the
rationale and the contracts between layers, as they are today.

## Map

The directory mirrors the strata of `brio/`: `design/` is the
target-independent framework (kernel, services, the models every
target realizes); one folder per target (`avrdx/`, `samc21/`,
`stm32g0/`, `stm32f4/`, `ch32v00x/`, `ch32vx03/`, `rp2040/`,
`rp2350/`, `host/`) holds
that target's operational page (`README.md`), one document per
peripheral driver, and its vendor documents; and one folder per
stratum that is not a target - those that sit between `util/` and the
targets, the core stratum `cortexm/` and the IP strata `pl011/`,
`pl022/`, `dw_apb_i2c/`, and `devices/` for the parts that sit OFF the
chip - each with a page of its own. Within each, ordered by
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
| [design/can.md](design/can.md) | The CAN vocabulary: the classic frame, the bit timing in human units with a controller's limits as a value and the exact search, the error codes and the error state as one observable - and why there is no bus AO |
| [design/nv-heap.md](design/nv-heap.md) | NvHeap: runtime-allocated blocks of flash that outlive the program - the FlashMedia contract, the ping-pong map pair and its atomicity, survival-aware mount, the placement rule, the wear accounting |
| [design/nv-journal.md](design/nv-journal.md) | NvJournal: a handful of small values kept in FLASH on a part with no EEPROM - two halves that ping-pong wholesale, sequence numbers deciding and CRCs judging, a read-only mount, and the panic reserve an ordinary save always leaves behind so a panic handler can write one bounded entry with no erase |
| [design/usb.md](design/usb.md) | USB, the device side: the contract drawn at the PACKET (an endpoint controller offers one packet per endpoint and direction, reports completions, the setup packet and the reset), the control-endpoint machine and chapter 9's requests written once above it, the class contract, and CDC ACM as a byte transport with USB's own NAK as its flow control; proven on the host against a scripted controller before silicon; no host side, ever |
| [design/power.md](design/power.md) | The power model: the sleep-depth ladder, the site that only arms (so the kernel loop's own idle path does the sleeping), the round of votes among stakeholders, standing restrictions for the ones that live in interrupts, the deadline guard, and the first-event-after-wake contract |
| [design/ring.md](design/ring.md) | Ring: the SPSC FIFO, lock-free where the platform allows, guarded elsewhere |
| [design/analog.md](design/analog.md) | Analog in the kernel: the AnalogSampler usage type (converter concept, attribution by reported code, two paces, owner's duties) and the counts <-> mV arithmetic |
| [design/meters.md](design/meters.md) | Meters in the kernel: the MeterLatch that bridges a capture interrupt to the loop (last value wins, overwrites counted) and the MeterSampler that paces PUBLICATION instead of capture - a stale source publishes nothing |
| [design/block-stream.md](design/block-stream.md) | Block streams: the BlockSource/BlockPlayer concepts (caller-owned buffers, the accounting IS the API, blocks not DMA) and the BlockRelay AO that lends filled blocks for one dispatch - built BEFORE the second implementation on purpose, as the fixed point the next platform is measured against |
| [design/gfx.md](design/gfx.md) | Graphics: the three kinds of surface, told apart by where the truth about a pixel lives (nowhere the program can reach, inside the panel, in the program's own memory); the write-only base every surface satisfies and the library draws through whatever lies beneath; the two verbs panels are good at (a filled rectangle, a window fed a run) with a single pixel as the degenerate case and not the foundation; the conventions that are free before a primitive exists and are one-pixel errors after; monospaced opaque text as a consequence of write-only erasing; no compositing; and the three planes of truth - a reference renderer derived from the definition for the primitives, a differential oracle for the pipeline, the bench for the silicon |
| [design/simulation.md](design/simulation.md) | Simulation: the model of the world a program runs against when the machine under it is the host - the world never speaks to the program, so every device's seam is a level and not a gesture; the channel table in both directions, whose columns are either stimulated or COMPUTED (six rules that make a computed one deterministic, and what separates a model from an empty stub); the world as a type with two verbs; the three time policies and who may join each; the scenario file, where outcomes are stimuli and the degrees of freedom are declared; goldens against invariants; and what the host does not tell you |
| [design/architecture.svg](design/architecture.svg) | The strata diagram |

The targets, each with its front page - the operational side
(toolchain, board, probe, debugger and their quirks) and the map of
that target's own documents, one per peripheral driver - and among
them the strata that sit between `util/` and the families, whose
pages say what is theirs and what a family owes them:

| Target | Front page |
|--------|------------|
| AVR DA/DB (`brio/avrdx/`) | [avrdx/README.md](avrdx/README.md) - Toolchain, board, Atmel-ICE upload, PyAvrOCD debugging and its quirks, clock/timebase; then its documents |
| SAM C21 (`brio/samc21/`) | [samc21/README.md](samc21/README.md) - Toolchain (vendored DFP/CMSIS, no device headers in arm-none-eabi-gcc), board, OpenOCD upload over SWD, cortex-debug, the clangd routing, the family smoke check; then its documents |
| STM32G0 (`brio/stm32g0/`) | [stm32g0/README.md](stm32g0/README.md) - Toolchain, the three Nucleo boards, ST-LINK upload and debug, the SWD-under-WFI caveat, and FAMILY COVERAGE: both the x1 line and the x0 value line, the stratum compiling on all twelve headers of the pack with every vector derived from peripheral presence; then its documents |
| CH32V00x (`brio/ch32v00x/`) | [ch32v00x/README.md](ch32v00x/README.md) - Toolchain (WCH's gcc 15 with its xw extension, each part's full ISA), the two boards and the part table the build states, the WCH-Link and WCH's OpenOCD fork, the console on the probe's own serial, and what the QingKe V2 core taught the stratum (a WFI that wakes only for an interrupt it can take, the MIE not cleared on entry); then its documents - one per chapter of the reference manual, each with what the CH32V006K8 measured with no wire and what waits for a jumper or a peer |
| CH32V203 / CH32V303 (`brio/ch32vx03/`) | [ch32vx03/README.md](ch32vx03/README.md) - Toolchain (WCH's gcc 15 again, the full register file, the ilp32 ABI on the V4B parts and ilp32f on the V4F ones), the WeAct core board and WCH's CH32V303 evaluation board, the thirteen parts of the two series in one table the build states under three device classes, the two-wire debug port of this family and the reset verb that does NOT start the program, the console on the probe's own serial and on the chip's own USB, and what the silicon taught the stratum - the clock task parking on the HSI, the PLL divider that lives in another block, the USB pads that are still GPIO pads, the bus matrix that serves the core alone in a sleep of any depth, the CH32V303 with no USB device controller of its own - its console on the host/device block - and a floating-point unit an interrupt pays for, the lot-keyed registers the bench's CH32V303 has not got; then its documents - one per chapter of the reference manual, each with what the CH32V203C8 and the CH32V303VC measured on their boards and what waits for a wire, a peer board, another part or another lot - and what the reference manual's V2.5 changes, beside the silicon |
| RP2040 (`brio/rp2040/`) | [rp2040/README.md](rp2040/README.md) - Toolchain (the pico-sdk's CMSIS header and register definitions vendored, no SDK runtime, the boot stage checked in as bytes), the WeAct board, the Debug Probe and which OpenOCD the flash chip demands, and the two cores with a kernel on each; then its documents |
| RP2350 (`brio/rp2350/`) | [rp2350/README.md](rp2350/README.md) - TWO PROCESSOR ARCHITECTURES OVER ONE SET OF PERIPHERALS and what that costs a build: the two toolchains (arm-none-eabi for the Cortex-M33 pair, an upstream riscv32-unknown-elf for the Hazard3 one), the four presets, the IMAGE_DEF block in the image as the only thing that decides which pair runs, the WeAct core board, the Debug Probe and the third OpenOCD - Raspberry Pi's fork, the only build that examines all four cores - the state-independent flash verb built on the rescue reset, and the four facts of this silicon that shape every chapter (a processor reset that leaves the clock tree standing, pads that come up isolated, bit maps that are not the RP2040's, erratum RP2350-E9); then its documents |
| STM32F4 (`brio/stm32f4/`) | [stm32f4/README.md](stm32f4/README.md) - Toolchain (the hard-float ABI, the FPU enabled by the crt), the four boards (an STM32F429I-DISC1, a Nucleo-F446RE, an STM32F411CE black pill on a standalone STLINK-V3, a 32F469IDISCOVERY), ST-LINK upload, the HLA caveat, and FAMILY COVERAGE: twenty-three headers, the frequency ladders keyed on the part class and refused where no manual was read; then its documents |
| the Cortex-M core stratum (`brio/cortexm/`) | [cortexm/README.md](cortexm/README.md) - what the Cortex-M families share whatever the vendor: NVIC + PRIMASK, the SysTick ticker and the microsecond busy-wait on SysTick's counter, the include-order contract, what stays per family and which families take which of the three files |
| the PL011 IP stratum (`brio/pl011/`) | [pl011/README.md](pl011/README.md) - ARM's PrimeCell UART written once for every family that carries it: what is ARM's (the frame, the divisor arithmetic, the registers, the resource and the transport) and what a family owes it (the `Pl011Chip` traits - registers, reset, interrupt line, clock, pads), and the rule that births an IP stratum |
| the PL022 IP stratum (`brio/pl022/`) | [pl022/README.md](pl022/README.md) - ARM's PrimeCell SSP the same way: the host engine `util/spi_bus.hpp` drives, the client, the framings and the prescaler pair, with the chip's half behind a concept |
| the DesignWare I2C IP stratum (`brio/dw_apb_i2c/`) | [dw_apb_i2c/README.md](dw_apb_i2c/README.md) - Synopsys's DW_apb_i2c the same way: the command FIFO whose entries carry the bus conditions, the host engine, the client, and what a family states about pads, requests and its reset controller |
| the off-chip devices (`brio/devices/`) | [devices/README.md](devices/README.md) - the stratum for what sits OFF the chip and is reached over a link the chip provides: the DCS vocabulary every command panel shares, a traits type per controller, and how a driver here is judged twice - against a host simulator and against the bench |
| host (`brio/host/`) | [host/README.md](host/README.md) - The native test target: HostPlatform, doctest suites, what a simulation is allowed (the whole standard library, heap included), and the simulator's own page - [host/simulator.md](host/simulator.md), the contract between a host program and a viewer watching it: one shared region in each direction, why input is a snapshot and not a socket, and the three rules that keep it portable |
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
