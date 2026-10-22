# brio: philosophy, rules, layering

This page decides the rules every other page of brio is written
under: what is allowed to change, how the code is layered into
directories called strata, what a target's driver is and is not, how a
driver is judged done, and the style the code keeps. It is written for
an embedded programmer who knows C and microcontrollers and may know
little C++; each term of brio's own is defined where it first appears.
The kernel's own contracts are in [kernel.md](kernel.md), and each
service and each target has a page of its own
([the documentation map](../README.md)).

brio takes its name from the musical marking "con brio"; the name is
tied to no silicon on purpose. It is a C++ framework for bare-metal
microcontrollers built around a cooperative active-object kernel, the
model described in Miro Samek's book, written clean-room: the concepts
come from the book, never from the QP source. An active object is a
component of the application with its own queue of events and its own
state, which the kernel serves one event at a time; cooperative means
that an active object's event handler runs to completion before any
other active object runs on that core, and that interrupt handlers,
not the kernel, are the only thing that preempts it ([kernel.md](kernel.md) states the model
and its contracts). It runs today on nine
families of microcontroller, from an 8-bit AVR to dual-core ARM and
RISC-V chips, and on the host, where its tests run ([the target table](../../README.md)).

![brio strata](architecture.svg)

<sub>[open the diagram full size](https://raw.githubusercontent.com/uliano/brio/main/docs/design/architecture.svg) (zoomable in the browser)</sub>

*The strata, the contracts between them (dashed) and the two
directions of traffic: events go up, by value, condensed by interrupt
handler bodies; calls go down, inside a dispatch; the timebase runs
sideways, independent of the CPU clock.*

## Governing rule

**Nothing is settled.** Reusing existing code is fine only as long as
it does not limit the design above it. Whenever a limitation could be
removed by rewriting what sits below, the rewrite wins. Every stage of
work asks for critical analysis at every level of the stack, not only
at the layer being added.

Within that rule, three levels of stability say how much a change may
disturb. The kernel's ideas are fairly stable: the active-object
contract, events by value, the two loans (a buffer lent through a
pointer inside an event, either for the one dispatch that delivers
the event, that is one call of the receiver's handler, or until a
reply event returns it), the three ways of
delivering an event (post to one object, publish to a list, reply to
whoever asked), priority as list order, timers that post events, the
panic record a failure leaves for the next boot, the platform concept
(what the kernel requires of a chip), and the rule that there are two
execution contexts and one boundary between them; each is stated in
[kernel.md](kernel.md). The services of `util/` and the target
drivers are here to stay but are expected to change, possibly
radically, as targets are added. Bench applications (test suites,
probes, consoles) are disposable: nothing in the foundations depends on
one. The one kind of application the documentation does depend on is
a maintained teaching example (`blink` today), which keeps compiling on
every preset of its project and is re-read when an API it uses changes
([docs/README.md](../README.md), the rules).

## Design pillars

- **Everything resolves at compile time; a type stands for each
  thing.** A peripheral is a singleton by nature: there is one SPI0 and
  one pin PA2. brio therefore gives each one a type, `Spi<0>` or
  `Pin<'A', 2>`, with static functions as the access, instead of an
  object that would model something which does not exist twice. The
  compiler folds those functions to the instruction or the constant
  address. The rule for what goes where: identity and invariants go in
  the template parameters (which instance, which route or port, fixed
  sizes and pins: what never changes, and should be checked and folded
  when the program is compiled); what changes goes in run-time
  arguments (a duty cycle, a byte, a rate; also what could be constant
  but earns no type of its own, since a baud rate still folds when the
  argument is a literal); state lives in `static inline` members (a
  ring, a queue, the dynamic clock's rate: one per type, in `.bss`, no
  constructor and no initialization order). Such a class is called a
  monostate: the wiring is decided when the program is compiled, the
  state is static and changes while it runs. Active objects, the
  kernel, clocks and the tasks over peripherals all follow this one
  sentence. There are no virtual interfaces and no run-time
  singletons; where a type must travel as an argument, an empty tag
  object does it (`constexpr Serial serial;`, `constexpr SysClock
  clock;`). The one deliberate return to run time is a small
  descriptor a type produces when a request must travel by value
  (`PinRef` from `Pin<>::ref()`), never a parallel abstraction.
- **Drivers move bytes; they do not format.** Text output is
  `brio::print(sink, ...)`, a set of free functions over any byte sink;
  a new type becomes printable through a `print_one` overload found by
  argument-dependent lookup. `print` waits until the sink has accepted
  every byte and never truncates (`util/print.hpp`, "Delivery
  policy"), so a handler that prints a long message on a slow port
  stalls every other active object for that long.
- **The protocol layer is push-based.** `LineAssembler`
  (`util/proto/line_parser.hpp`) is fed bytes and returns completed
  lines; it refers to no transport, so it is tested on the host.
- **One flat namespace, `brio`.** Namespaces prevent name clashes;
  they are not architecture. Types are PascalCase, functions and
  constants snake_case.
- **The freestanding libstdc++ the toolchain ships** (type_traits,
  concepts, bit, span, optional, expected; no chrono, charconv or
  iostream) instead of hand-rolled traits.
- **gnu++23 project-wide.** C++26 features already in gcc 16 may be
  used when genuinely needed. Two idioms in active use:
  `static_assert(false, ...)` in a discarded `if constexpr` branch for
  "not present on this device" errors, and `std::to_underlying` for
  enum codes.

## Generalization rule

The kernel must never know which silicon it runs on. Where a template
or a concept boundary can do the job, no `#ifdef` is used. No target
include leaves the target strata. A fact that differs per target lives
behind the `Platform` concept or inside a driver. Differences inside
one family (the AVR DA against the DB, package sizes) are handled with
guards on the vendor header's macros, so one set of headers builds
everywhere.

The preprocessor's one legitimate question is whether a vendor macro
exists, the question no template can ask. Even that is split by what
the answer produces. A probe whose product is a value (a pad's line
number, an instance's clock id, a trigger code, a bonding fact) belongs
in the family's device-tables header (`samc21/device_tables.hpp` was
the first), one entry per header symbol, exported as plain `constexpr`
data. Probes are preferred to per-variant tables because a probe
re-reads the vendor header at every compile and cannot drift, where a
copied table can. A driver keeps an `#ifdef` only where it selects
code that differs per instance, a register-struct reference or an
`if constexpr` tier, which no `constexpr` table can hold. Everything
else is C++.

**Authority of util/.** A `util/` type is target-independent by
contract: it includes no target, it is parametrized on a concept (a
compiler-checked list of requirements on a type), and
it runs on the host against a simulated chip. Its concept, however,
was written looking at the targets that exist, and the authority to
call the type general is exactly the list of those targets. Every
`util/` type therefore states in its header on which targets it has
been validated and which assumption of theirs its contract may carry.
Examples: `SerialPort` assumes a byte stream with a receive edge;
`BusMaster` a transaction that runs on interrupts and completes later;
`Ring` an index that fits the platform's atomic access width;
`AnalogSampler` one result per interrupt with the selected input
readable, and a select that cannot mislabel a result, which an
unbuffered input multiplexer owes to its driver's select. A type
validated on one target carries that line as a warning: the second
target is expected to force a synthesis, possibly a rewrite of the
concept. That is the plan, not a failure, and the reason the
stability hierarchy puts `util/` below the kernel's ideas. The right
order is two nearly complete low-level strata before generalizing;
until then, `util/` generalizes from one and says so.

**A contract's unit is the run.** A contract is drawn against the most
capable silicon in view, never against the first chip. A transfer
function takes a span of bytes and treats a single byte as the
degenerate case, because every peripheral with a FIFO or a DMA
request moves runs, and every vendor's library takes a buffer and a
length. The first chip's constraint (one data register, one byte per
interrupt) belongs to that chip's realization; a contract that speaks
it charges it to every chip after. Before a contract is taken as
settled, it is realized on two opposite shapes of silicon, one with a
FIFO or DMA and one without, and read in both images. The byte sink
of `util/stream.hpp` is this rule applied late: its single-byte
function stays for a sink that has nothing else (a test capture, a
simulated port), and every transport carries the run through
`write_bulk` (the `BulkSink` concept), which `print` hands every
string and every number, except a C string's first byte, which goes
through the byte function so that an idle transmitter starts before
the rest is measured. The source side mirrors it: a `SpanSource` lends
its received run in place.

## Layering: the strata

The code is grouped into directories under `brio/`, one per layer,
called strata. Each stratum may include only the strata below it, and
an include always carries the stratum's name
(`#include "avrxt/usart.hpp"`), so a file's portability can be read
from its includes.

| Stratum | May include | Content |
|---------|-------------|---------|
| `kernel/` | nothing of brio | pure logic: queues, the scheduler, state machines, time events, panic, delivery ([kernel.md](kernel.md)) |
| `rt/` | nothing of brio | the runtime every 32-bit image compiles: the seven functions the compiler and libstdc++ expect of a freestanding program, in the framework's one source file; the AVR keeps avr-libc instead ([runtime.md](runtime.md)) |
| `util/` | `kernel/` | services over the kernel: streams and `print`, the benchmark instrument, rings, the bus arbiter with the SPI, I2C and CAN vocabularies, the serial port, the flash heap and journal, power, meters, block streams, the two-core inbox, the USB device stack, the test-bench grammar |
| `gfx/` | nothing of brio | drawing: the surface contract, the primitives, fonts and text ([gfx.md](gfx.md)) |
| `devices/` | `kernel/`, `util/`, `gfx/` | what sits off the chip and is reached over a link the chip provides: the DCS vocabulary, a traits type per display controller, the panel driver over a link concept ([gfx.md](gfx.md), "The command tier") |
| `cortexm/` | `util/`, and the CMSIS-Core symbols a family's device header brings (the family includes that header first) | what every Cortex-M family shares whatever the vendor: the interrupt controller and its guard, the SysTick ticker, the microsecond delay, the cycle counter |
| `pl011/`, `pl022/`, `dw_apb_i2c/` | `kernel/`, `util/` | peripheral blocks licensed to more than one vendor (ARM's PrimeCell UART and SSP, Synopsys's DesignWare I2C), written once over a traits type each family supplies |
| `avrxt/`, `samc21/`, `stm32g0/`, `stm32f4/`, `ch32v00x/`, `ch32vx03/`, `ch32x035/`, `rp2040/`, `rp2350/` | `kernel/`, `util/`, and `cortexm/` or an IP stratum where the family carries the block | one stratum per family: everything that knows the family's device header, the platform type, the clock, the pins and every peripheral driver |
| `host/` | `kernel/`, `util/`, `gfx/`, `devices/` | the native target: the host platform, the chips made of RAM, the shared-memory display, the graphics reference renderer |

Two targets never meet in one binary, so every target stratum is a
sibling of the others and the flat namespace stays free of
collisions. There is one library, not one per stratum: the discipline
comes from the include rule, and several libraries would add ceremony
without enforcement. A pure type produced by a target and consumed by
`util/` lives in `util/` (`util/timestamp.hpp` is one).

A stratum is named for exactly the family it has been proven on, and
widens only when a real chip proves it shares the stratum. One name
per architecture holds on three axes: `brio/<arch>/` for the code,
`docs/<arch>/` for the documents and `<arch>/` for the build project;
chip precision lives in preset names, per-chip linker scripts and the
`*_MCU` cache variables.

## Target strata: drivers by role, not a HAL

Pins, timers, PWM and clocks stay files of their target, each with its
own peculiarities stated in the open. brio does not build a hardware
abstraction layer, and never a multi-platform one. What the targets
have in common is how a driver is made and what it produces for the
layers above, not what the peripheral is.

- **A driver is a task over a resource.** Two thin layers. The
  resource handle (`Tcb<0>`, `Tca<1>`, a `Dma<3>` elsewhere) knows only
  that the instance exists on this package, where its registers are,
  and its identity as a type: no policy, no `init`. It is the name with
  which the application says "this one is mine". The task
  (`PeriodMeter<Tcb<0>, Pin<'A', 2>>`, `TcaPwm<0, 'C'>`, `Ticker`) is
  named for what it does, owns the whole configuration of the
  peripheral for that use, produces events by value for the layers
  above (`PeriodMeasured{ticks}`) and takes commands as static calls.
  One resource can enter one task at a time; a second task on the same
  peripheral is another type, not a flag. Task names and their events
  are the same on every target; resources and implementations are per
  target. Where a task still takes a bare number (`TcaPwm<0, ...>`),
  the explicit handle is introduced at the second task over the same
  peripheral, never earlier.
- **What is common lives at the level of the role, and it is small.**
  A timebase is the platform's `now()` and `ticks_per_second`. A PWM
  channel is the `PwmChannel` concept: `max` and `duty()` in raw
  counts, with no frequency and no polarity, which belong to the timer
  instance and the target. A measurement is an event posted by value
  with its timestamp. Register layouts, modes, clock trees, prescaler
  sets and event-system wiring are never unified: the target speaks
  its own vocabulary.
- **The properties every target driver has.** A monostate type chosen
  when the program is compiled; existence and routing checked with
  `static_assert(false)` in the instantiated branch; one `init()` that
  owns the whole configuration of the role, so no register is touched
  by halves around the application; interrupt handler bodies exposed
  for the application to bind; the model explained in the header
  comment.
- **No driver allocates a resource the application did not name.** The
  kernel tick runs on the RTC or the periodic interrupt timer precisely
  so that every general-purpose timer stays available to the
  application. A framework that silently takes a timer for a
  millisecond counter is what brio refuses to be.
- **Interrupts condense; DMA is an interrupt handler made of silicon.**
  An interrupt handler body (exposed by the driver, bound by the
  application) lives in hardware time. It does the minimum bookkeeping
  (a byte into a ring, one step of an engine, the part of a driver that
  moves the bytes of one transfer), plus whatever has a
  deadline measured in cycles (a counter snapshot-and-reset that must
  land within one heartbeat period, for instance 64 CPU cycles at
  375 kHz), and, on an edge only (a complete line, a transaction done,
  the end of a window), one `post()`. The active object above lives in
  kernel time and holds the logic. What has a deadline in cycles never
  goes through the queue; what has a deadline in milliseconds always
  does. Bytes at a high rate, events at a low rate; only the queue
  crosses between them. A DMA channel does the handler's byte-moving
  without the CPU and raises the edge itself. It plugs in inside an
  engine: `start(request)` programs the channel with the request's
  buffers (lent by the requester under the second of the two loans,
  until the completion event, so the channel may read and write them
  meanwhile), and the completion
  interrupt posts `TransferDone`. Nothing above the engine changes; the
  DMA channel is a resource like a timer, and a receive stream fed by
  DMA is a ring whose producer index is the hardware's counter
  ([dma.md](dma.md), [ring.md](ring.md)). The AVR DA and DB have no
  DMA; their engine boundary is already where a channel would go.
- **Hardware routing (the event system) is behaviour, not only
  configuration.** On the AVR, generators, users and their per-channel
  legality are types, checked when the program is compiled; connecting
  and disconnecting are run-time primitives callable from a state's
  `Entry` and `Exit`, so a rewire is an action of a state, like arming
  a time event. A static `EventSystem<Route...>` allocating channels at
  compile time would be optional sugar over the same primitives; it is
  not built, and the tables grow on demand (`avrxt/evsys.hpp`).
  Contention for a channel rewired at run time is ownership: it belongs
  to one active object's state machine. The word "event" is overloaded
  here and the overload is kept on purpose, because both names belong
  to others, Samek's and the silicon vendors'. A kernel event is a
  value, produced, queued and delivered to an active object in its
  turn; it lives in software time. A hardware event is an edge on an
  internal wire from one peripheral to another, with no CPU and no
  datum; it lives in hardware time, and becomes a kernel event only
  through an interrupt handler that posts. The types never meet:
  `Event`, `EventQueue` and `TimeEvent` are the kernel's;
  `EventChannel`, `EvPitDiv`, `EvPin` and their kin live in the
  target's event-system header, and the kernel knows none of them. In
  prose, "hardware event" and "kernel event" wherever the context does
  not tell.
- **PWM is an actuator, not a bus.** Continuous state, set and forget,
  synchronous, no completion, no contention worth an arbiter: it has
  the rank of `Pin` and is called from handlers. Fades and sequences
  are active objects in `util/` above the channel, never inside the
  driver.
- **The clock is a type with two regimes** (the full model:
  [clock.md](clock.md)). At compile time, `Clock<...>` owns the tree
  and `Clock::hz` is the one truth the same target's drivers derive
  their divisors from. No vendor `F_CPU`-style macro exists in the
  build: a second truth would be unflagged, so vendor headers that
  need one stop compiling on purpose. "Always at maximum" is not a
  rule; a low rate is a legitimate choice. At run time, only when an
  application needs it, a rate change is a synchronous fan-out to every
  clocked driver (`Users::rebase(hz)`, applied to all before the next
  byte: publish semantics, fold mechanics, never a queued event that
  would arrive late), coordinated with the bus active objects so
  nothing changes in the middle of a transfer. The RTC-based tick does
  not move. The kernel never knows the clock.
- **Clock and delay, concretely.** `Clock` is templated on the
  parameters of the tree (source, multiplier, dividers), never on a
  list of allowed frequencies. `hz` (one per clock domain where the
  target has several) is `constexpr` arithmetic on those parameters,
  with `static_assert`s carrying the datasheet's limits (the VCO range,
  the maximum system clock, the flash wait states): the calculation a
  vendor's configuration tool does, done by the compiler, with errors
  instead of red boxes. The complexity of a rich tree stays inside that
  target's `clock.hpp`. `Clock::is_static` says whether `hz` is a
  compile-time constant or a run-time value. `delay_us<Clock>(us)`
  ("at least `us` microseconds") is a per-target driver of the
  short-wait role, for hardware setup times inside drivers, never a
  substitute for time events: a cycle-calibrated loop where the core
  is deterministic (the AVR), a hardware counter (DWT CYCCNT, SysTick,
  a free timer, `mcycle`) where pipelines, prefetch and wait states
  make cycle counting a lottery. The same name and semantics, no
  shared algorithm.
- **Every addition to a family is designed with the other targets in
  mind.** Before a new type or a change to one, ask what its shape
  would be on a Cortex-M0+ with a rich clock tree, on a RISC-V core, on
  a chip with per-pin alternate functions instead of per-timer routing.
  The answer decides what goes above the concept boundary (little) and
  what stays in the target file (most). Nothing written for one family
  may make another target harder than it already is.
- **A core stratum sits between util/ and the families that share a
  core.** `cortexm/` holds what ARM designed and every Cortex-M vendor
  ships unchanged (the NVIC and the PRIMASK guard, the SysTick ticker)
  and nothing of any vendor: of hardware it reads CMSIS-Core symbols
  only, and it refuses to be included before a family's device header, which is
  what brings those symbols. It was factored at the second ARM family,
  with both copies in hand and every image of both families
  byte-identical before and after; the platform, the clock, the pins,
  the startup code and the errata stay per family, because that is
  where they differ. A RISC-V family gets its own core stratum the same
  way, at its second member and never earlier: Raspberry Pi's Hazard3
  is not WCH's QingKe, so the RP2350's RISC-V core file stays its own,
  and a `qingke/` core stratum is due at the CH32X035, the third QingKe
  family.
- **An IP stratum sits in the same place, for a peripheral.** A block
  licensed by more than one vendor and dropped into their chips
  register for register earns a directory of its own, named for the
  design and not for a silicon (`pl011/`, `pl022/`, `dw_apb_i2c/`),
  above `util/` and below the families, under exactly the core
  stratum's rule: born at the second family that carries it, with both
  copies in hand, gated by the images. What earns one is a register
  description that is not merely similar between the chips but
  identical, the same names, offsets and bits, so there is nothing to
  reconcile and nothing to gain by writing it twice. A peripheral that
  is only a relative (the STM32F1's event machine under two vendors'
  names, say) does not qualify and stays one file per family. An IP
  file knows no chip and includes nothing of a family. What a family
  owes it is stated as a concept and satisfied by a traits type of the
  family's own (a small type whose members state the family's facts): where the registers are, the reset, the interrupt line
  and its controller, the critical section, which pads are legal,
  which rate clocks it, which DMA requests it raises. The family's own
  header keeps the public names an application writes. The claim "this
  driver knows no chip" is itself proved by a second realization with
  no silicon under it: a block made of RAM in `host/`, which the host
  suite and a family's compile check both build.
- **Board facts against device facts.** Which timer reaches which
  port, and which USART sits on which pins per route, are facts of the
  device: on the AVR, tables inside `tca.hpp` and `usart.hpp` over the
  stratum's device tables (`avrxt/device_tables.hpp`); on the two
  Cortex-M0+ strata (the SAM C21 and the STM32G0), the device-tables
  header itself
  (`samc21/device_tables.hpp`, `stm32g0/device_tables.hpp`), the one
  file where the preprocessor may probe a vendor macro and export what
  it finds as `constexpr` data. Which timer drives which LEDs, which
  pins are buttons, which vectors bind to which driver bodies, are
  facts of the board. They belong in a per-board unit that can also
  list the resources an application claims and reject a double use
  when the program is compiled; that unit is born with the first
  portable example application, a stated gap of the learning route.
- **A driver covers its family, not its bench chip.** The bench chip
  is the one part of a family on the desk. brio is a
  framework: the target is the whole device range, every instance,
  every mode, every routing option of the chapter's register
  description and every family's errata, not the subset today's
  application exercises. What is knowingly left out is declared in the
  document's "Not covered yet", never silently absent. A driver is done
  only when it compiles for every package of the family (one smoke
  translation unit per package, plus negative tests proving that what
  must be refused fails to compile) and its `test_<target>_<subject>`
  suite passes on the bench. The family compile takes seconds and
  needs no hardware; the bench chip alone masks half the family.
- **A driver is written from its chapter, not from its sibling.** What
  travels between strata is the surface: the task's name, the request,
  the functions, the events. It is copied on purpose. The engine under
  it is written from the chapter of this silicon's reference manual,
  starting from an inventory of what the silicon offers that bears on
  cost: FIFO depths and thresholds, DMA requests and circular mode, the
  edges that end a run (an idle line, a receive time-out), byte
  counters with an automatic STOP or RESTART, set/clear/toggle
  aliases, timer widths and compare registers, the instruction fetch
  (wait states, prefetch, cache, code placed in RAM). Each entry is
  used, or declined with its reason, and the driver's document says
  which ([docs/README.md](../README.md)). A shape that is right on the
  chip it was born on (one byte per interrupt on a block with one data
  register, a whole channel programmed per block on a descriptor
  controller) is wrong on a chip with a FIFO or a register file, and
  no suite that judges behaviour will say so. A comment therefore
  justifies an engine choice from this chip's manual, by section; "as
  the other strata do" is a statement about another chip, not a
  reason. The vendor's library is read before the engine is written,
  as an oracle of shape and cost and never as a source: how it
  performs each operation and what that costs. brio need not win every
  comparison; a gap of more than about a fifth is a finding to settle
  before the driver is done.
- **A driver's cost is part of what it is, judged against the
  silicon's limit.** A transport or an engine has three numbers: the
  cycles per byte or per frame on its hot path, the interrupts taken
  per byte, the cycles run with interrupts masked. They are counted
  once in the disassembly of a release image, because the source hides
  what the compiler adds (a lambda outlined on one core and inlined on
  another, a copy, a call per byte), measured by a letter of the test
  suite, and stated in the document beside the wire's or the bus's own
  figure. A measured rate far from that figure (more than about half
  again the wire's time) is not a fact to record but a finding to
  explain; until it is explained it sits in "Not covered yet". Byte
  identity (`brio gate`) is the proof for a change that claims to move
  nothing; it is never a reason to leave a better default off. A
  better default is switched on by a change of its own, and measured.
- **Three rules for a hot path.** One: nothing on a per-byte or
  per-frame path is a call. The register accessors and the ring's
  functions inline, the request's fields are read once outside the
  loop, the loop is specialized on the request's shape (no data out, no
  data in, the frame width), and the disassembly is where this is
  checked. Two: what is constant for as long as a driver stays
  configured (widths, direction, the peripheral's address, the
  validation of all of it) is written when it is configured (`init`,
  `arm`) or checked when the program is compiled; an operation writes
  only what changes (a
  buffer address, a count, an enable). Three: a critical section covers a
  decision (a claim, a test-and-set), never a sequence of register
  stores, a copy or a spin; a masked window longer than a few tens of
  cycles states its count in its comment.
- **These rules ask questions and demand numbers; they fix no answer.**
  DMA is not better everywhere: the CH32V203's bus matrix serves the
  core alone in a sleep of any depth, so a block in flight forbids the
  sleep, and an interrupt per byte is the cheaper transport in energy.
  A FIFO threshold is wrong for a console: the tail below the threshold
  waits for something else to happen. What the rules forbid is a
  choice made without asking.
- **A loop proves the bytes, never the rate.** A suite's loop (a
  loop-back, a single wire, a jumper, a crossed pair, a self-link)
  shares one divisor at both ends, so a wrong divisor is a slower or a
  faster loop and every byte still comes back. Every test letter (one
  test of a bench suite, selected by a letter on its console) that
  loops or sets a rate therefore times a run of frames against the
  wire's time at the rate asked, never at the rate the divisor
  produced, and judges the reading in thousandths: a UART run within
  -1 % to +3 % (or the divisor's own rounding where that is larger,
  stated), an SPI frame inside a block within the same of its clock
  periods and the block's own gap, an I2C clock never above the rate
  asked and within a quarter below it, or, where the block's own clock
  shape puts its fastest legal rate lower (the AVR's TWI, whose clock
  is 50/50), within that rate computed from the chapter, less 2 %. A
  reading outside its window is a finding to explain, never a window
  to widen; a rate the instrument cannot time is said so, with what
  would time it.
- **A suite's image fits the family's smallest chip.** A bench suite is
  a test program that offers its tests as a menu over the console, one
  letter per test, and prints a verdict for each; its image is a sum:
  the base every image carries (the startup code, the transport, the
  test bench, `print`), the driver under test, the letters, and the
  verdict prose (the text each test prints to say what it measured and
  whether it passed), which weighs as much as the code it judges. The
  unit of that
  sum is the group of letters an image carries, and the size of a
  group is bounded by the smallest chip of the family, flash and RAM
  alike. Letters that exercise what every part has are grouped so that
  each image links on the smallest part (the linker script is the
  guard: an image over the part's memory does not link), while letters
  that exercise what only the larger parts have (a third timer, a
  second USART) build only where the peripheral exists and may be
  grouped as large as that part allows. The verdict prose is never
  shortened to make room: bytes come from regrouping, not from a
  console that says less. The mechanism keeps one source per suite.
  The suite declares its groups in its build header; the build of a
  part that cannot hold it whole emits one image per group, with the
  group's letters named to the test bench, which registers those
  alone, so the others are never referenced and the linker drops them
  with their prose; every other part's build emits the whole suite as
  one image, unless the suite names a larger part's own groups
  (`groups.<board type>`) because its whole image comes within a few
  per cent of that part's memory too. A suite is regrouped when an
  image has under about five per cent of its flash or its RAM left,
  the RAM's margin being the stack's room. The guard is a link, not a
  fixture: `brio check` builds every application on every preset (one
  configured build of a project, one per bench chip) of its family, the smallest part's among them, and fails on an image
  that does not compile or fit; `brio fit` is the census of every
  image's margins. On the CH32V00x, whose smallest part has 16 KB of
  flash and 2 KB of RAM, this rule decides the shape of every suite; on
  the CH32V203 it is the 32 KB tier (four parts with no board) that a
  suite is grouped for; on the STM32G0 it is the G031K8's 64 KB, on the
  SAM C21 its one part's 32 KB of RAM; on the AVR it is the smallest
  chip in view rather than the smallest the project builds, the
  ATmega4809's 48 KB and 6 KB, below the AVR64EA48's 64 KB, so the
  AVR64EA48's build holds every image to that tier after its link, the
  guard standing in for a part no preset builds yet. On the other
  families no image comes near its part's memory yet.
- **Package variability, the pattern.** The device header is the
  authority, at three granularities. A missing instance is compiled out
  in tiers on its header symbol (`#if defined(TCB4)`). An instance
  whose pin position the package does not bond stays fully usable:
  `port_exists` and `if constexpr` compile the missing branch out (a
  run-time `if` on a missing `Pin` kills the whole instance at compile
  time), the compile-time `init<cfg>` refuses the configuration with a
  `static_assert`, and the run-time `init(cfg)` returns false and
  programs nothing. A missing register or enum value is gated on its
  header symbol (`EVSYS_SWEVENTB_gm`, `TCA1`). Pin-level bonding inside
  an existing port waits for the device tables. Corollary: a fact
  stated in a document or a header comment ("this LUT has no ALT1")
  either has a guard in the code or is listed as a driver gap.
  Knowledge the code does not enforce is a bug deferred.

### One interface where it can, its exceptions where a reader looks

The rule above, task names the same on every target and resources per
target, is the claim; the realizations tables are where it is checked.
Every design page whose contract has more than one realization
carries a `### Realizations` section right after the contract it
documents: one sentence saying what is common, then one row per
stratum in a fixed order (avrxt, samc21, stm32g0, stm32f4, ch32v00x,
ch32vx03, ch32x035, rp2040, rp2350, host) with the realization, header
and type, and only what lies beyond the contract there (`-` for
nothing; an absent realization is a row too, with its reason). The
names in those rows are strata, never boards. A difference that is a
spelling of the same function is recorded as such; a difference of
function under a shared name is recorded as a trap. This is the one
home of the cross-target view: a header never cites another
stratum's.

| contract | its table | the exception in one phrase |
|---|---|---|
| the platform | [kernel.md, "Platform: what the machine provides"](kernel.md#11-platform-what-the-machine-provides-kernelplatformhpp) | `atomic_width` 1 on the AVR; `idle()` takes the armed mode three different ways, and is a WFE on the QingKe cores; a breakpoint is a NOP, a HardFault or an `ebreak`; `idle_until` on the tickless STM32G0 alone; one platform type per core on the two RP families, and on the RP2350 one type for both processor architectures |
| the timebase | [kernel.md, "Time"](kernel.md#9-time-kerneltimehpp-kerneltime_eventhpp) | the AVR's tick runs through every sleep; SysTick stops in standby and in a Stop; the LPTIM one counts through and is tickless; the RP2350's is SysTick or a platform timer by architecture, under one surface and one vector name |
| panic, reset, watchdog | [kernel.md, "Failures: overflow and panic"](kernel.md#10-failures-overflow-and-panic-kernelpanichpp) | the record's survival (EEPROM, RWWEE journal, bank 2); the fault body as the panic path on the ARM strata; the watchdog kick under three names and as many contracts |
| the ring | [ring.md](ring.md) | the atomic width alone |
| the clock | [clock.md](clock.md) | prescalers (AVR), no dynamic clock by position (SAM), a pack of rate tuples with a regime (STM32G0), one divider under one root (CH32V00x; the CH32X035 with no bus prescaler either), a generator per clock domain and no bus prescaler at all (the RP families); `delay_us` capped on the 32-bit strata, and on a timer that is neither core's on the RP2350 |
| the UART | [serial.md](serial.md) | seventeen functions in common; four ways to name pins, one or three vectors, DMA slots on most strata and bulk functions on some; on the two RP families the driver itself is shared, being the PL011's IP stratum |
| the SPI bus | [spi-bus.md](spi-bus.md) | the rate's unit (an enum or a divisor), a frame size on the STM32G0 and the CH32V00x, the client a surface per position with one published integer in common, and the PL022's driver shared by the two RP families |
| the I2C bus | [i2c-bus.md](i2c-bus.md) | the rate arithmetic's shape; `actual_scl_hz`/`scl_hz` and `bus_state`/`idle` are not one function under two names; the DesignWare block's driver shared by the two RP families |
| CAN | [can.md](can.md) | the frame's reach (classic against the FD superset), the timing's units and the search's rule, the error state as three flags or as one observable |
| the power model | [power.md](power.md) | each family's ladder mapping; where the tick stops and which site resyncs it |
| the USB device stack | [usb.md](usb.md) | each family's endpoint controller behind the packet contract, and what its RAM or FIFO adds |
| flash storage | [nv-heap.md](nv-heap.md), [nv-journal.md](nv-journal.md) | the geometries; the AVR keeps its small values in the EEPROM; a family may have the medium and mount nothing on it, by a decision those pages record |
| DMA engines | [dma.md](dma.md) | the levels and the release are the WCH strata's, the STM32G0's, the STM32F4's, the SAM C21's and the two RP families' (the RP2040 and the RP2350), the AVR having no controller; four levels or one bit, the arbiter's own tie rule |
| block streams | [block-stream.md](block-stream.md) | the engine names identical wherever block engines exist, and several strata's transfer engines waiting for a block user; the circular mode serves a player and not a source |
| meters | [meters.md](meters.md) | `TimIntervalMeter` is not a pulse-width meter; a family with no capture unit has no meter at all |
| analog | [analog.md](analog.md) | `Ref` is several vocabularies because a reference is several different things, one of them the rail alone; `set`/`write` on the DAC |
| pins and PWM channels | below | - |

Two contracts have no design page of their own and keep their tables
here.

**Pins.** Common to all: `Pin<'A', 5>` and `PinRef`; `set`, `clear`,
`toggle`, `read`, `output`, `input(PinPull)`, `port`, `ref`; and
`duty()`, since a `Pin` is a `PwmChannel` of one step. Beyond that,
`pull(PinPull)` and `is_output` exist on every stratum whose pad
carries its pull in a register of its own; the QingKe families put the
pull in the output register and have neither. A `PinConfig` carries
whatever else a pad can be told, as a `configure(...)` function on the
AVR, the SAM and the two ST families, and as an argument to every
configuring function on the two RP families (the RP2040 and the RP2350), where the pad register is
written whole. What differs is what a pad can be told to do, and how
it is handed to a peripheral.

| stratum | realization | beyond the contract |
|---|---|---|
| avrxt | `Pin` in `avrxt/pin.hpp` | `pull(PinPull)` and `input(PinPull)` with a `PinPull` that has no `down`: the family has no pull-down, and the missing enumerator is the refusal; `sense()`, `flag()`, `clear_flag()` (the pin interrupts are the port's here), `invert()`, `input_enable(bool)` (the input buffer, in a field shared with the sense: it leaves an armed sense at INTDISABLE); no `function()` or `release()`, since a pad is handed to a peripheral by that peripheral's PORTMUX route; `PinSet` |
| samc21 | `Pin` in `samc21/pin.hpp` | `function(PinFunction)` hands the pad to a peripheral function letter and `release()` takes PMUXEN off, the pin back to PORT as it was; `input_enable(bool)` (INEN, the same buffer with no sense beside it); `strong_drive()`; the WRCONFIG multi-pin engine (`configure_mask`); no `PinSet` |
| stm32g0 | `Pin` in `stm32g0/pin.hpp` | `function(PinFunction)` with the AF number, and `release()` = `analog()`, the reset state: the same name as the SAM's with a different landing; `output(level)` before the mode (the port-clock rule); `analog()` is a mode, not the input buffer alone; `read_out()`; speed and open drain in `PinConfig`; `PinSet` |
| stm32f4 | `Pin` in `stm32f4/pin.hpp` | the STM32G0's block register for register, with three differences a caller can see: no BRR, so `clear()` is a store into the upper half of BSRR, still one store; the port clock is on AHB1 and every configuring function opens it; and the reset state is input floating, so `release()` = `analog()` parks a pad in a state that is not the one it was born in, said in the header, because the G0's equation of the two does not hold here either; `function(PinFunction)` with the AF number, whose meaning on a pad is the datasheet's table and not a header symbol; `output(level)`, `read_out()`, `pull(PinPull)`, `is_output`, `has_function`; speed and open drain in `PinConfig`; `PinSet` |
| ch32v00x | `Pin` in `ch32v00x/pin.hpp` | `function(PinDrive)` takes no selector: a pad has one default alternate function on this family and the remaps that move it are AFIO's (`ch32v00x/afio.hpp`, a function of the resource that owns the pad); `release()` is the reset state, a floating input, and `analog()` a separate landing, so the G0's equation of the two does not hold here; `input(PinPull)` pulls through the output register (the one-bit mode of this family); no `pull()`, `is_output` or `PinConfig` |
| ch32vx03 | `Pin` in `ch32vx03/pin.hpp` | `function(PinDrive, PinSpeed)` takes no selector, as on the CH32V00x: which peripheral a pad carries is the remap register's (`ch32vx03/afio.hpp`); the MODE field is the speed, so every output function takes one (50 MHz by default); `input(PinPull)` pulls through the output register; `release()` is the reset state, a floating input, and `analog()` a separate landing; `nibble()` reads a pin's four bits back; the whole-port `out_write` and `configure_pins(mask, nibble)`; the configuration lock no other stratum has (`lock<Mask>()` / `lock(mask)` / `locked()` / `locked_pins()`, one way until a reset); `PinRef` with `read`/`write`/`toggle` beside `set`/`clear`; no `pull()`, `is_output` or `PinConfig` |
| ch32x035 | `Pin` in `ch32x035/pin.hpp` | `function()` takes no argument: MODE carries no speed on this series and there is no open-drain output, so neither the CH32V203's drive nor its speed exists, and which peripheral a pad carries is the remap register's (`ch32x035/afio.hpp`); `input(PinPull)` pulls through the output register and answers, false for a pull-down on a pad that has none (all but PA0..PA15, PC16, PC17), `input<PinPull>()` refusing it at compile time; `output()` and `function()` refused at compile time on a pad sharing its package pin with another (the datasheet's notes 4 to 7); `release()` the floating input and `analog()` a separate landing; `nibble()`, the whole-port `out_write` and `configure_pins(mask, nibble)`, the one-way lock; ports of up to twenty-four pins, so `out_set` is two stores across the halves (BSHR and BSXR), and CFGHR is written only from a RAM copy and never read to configure (`Port::cfghr_copy()`) |
| rp2040 | `Pin` in `rp2040/pin.hpp` | there is no port letter: the bank is one flat numbering, `Pin<25>`, and `Gpio` beside it offers the word-wide functions SIO gives the whole bank at once; `function(PinFunction)` hands the pad to a peripheral by the function column the pad table names and `release()` takes it back to SIO, `analog()` is a landing of its own (the digital input buffer off, the output driver off) for an ADC pad, `pull(PinPull)`, `input(PinPull)`, `is_output`; every configuring function releases the two pad blocks from reset first, because on this chip they come up held there; no `PinSet` |
| rp2350 | `Pin` in `rp2350/pin.hpp` | the RP2040's flat numbering over a bank that may be 48 pins wide, so `Gpio`'s word-wide functions speak two words; `PinConfig` carries the pad whole and every configuring function writes it, and the isolation latch is what makes that necessary: the pad comes up isolated (datasheet 9.7), the latch freezing the output enable, the level and the pulls, so a pad configured the RP2040's way would do nothing at all, and "writing the pad is taking it out of isolation" is this file's rule; `isolate(bool)` / `isolated()` are the latch as a function, and `input_enable(bool)` with `read_pulsed()` are the floating-pad erratum as a pair, the input buffer kept off and enabled for the length of one read, because a pad nothing drives leaks high against its own pull-down on this stepping |
| host | none | - |

**PWM channels.** Common to all: the `PwmChannel` concept (`max` and
`duty(v)`, `util/pwm_channel.hpp`), `RgbLamp` over any three, and
`Pin` as the one-step channel. The realizations are named for their
timer, and an application picks one in its board file:

| stratum | realizations | beyond the contract |
|---|---|---|
| avrxt | `TcaPwm<n, port>::Channel<ch>` (six 8-bit channels of a split TCA), `TcaPwm16`, `TcaPwmCentered` (dual slope, OVF at the centre), `Pwm8<Tcb>`, `TcdPwm<route>` (the complementary pair with dead time, on the TCD, which the AVR EA has not got) | the TCA and TCB channels are ratiometric on purpose (the duty is a fraction of the period, so it survives a clock change while the frequency moves with it) and not `ClockUser`s, the types a dynamic clock notifies ([clock.md](clock.md)); `TcdPwm` is one when the TCD runs on CLK_PER and was asked for a rate in hertz, and follows nothing otherwise |
| samc21 | `TcPwm`, `TcPwm8`, `TccPwm` (a caller-chosen max), `TccPairPwm` (the complementary pair with dead time) | - |
| stm32g0 | `TimPwm`, `TimPairPwm` (the complementary pair, on the timers with a break and dead-time unit), `LptimPwm` (on a timer that keeps counting in Stop) | the timers refuse a dynamic clock: their periods are PCLK cycles and no rebase can keep them |
| stm32f4 | `TimPwm<Tim, ch, top>`, `TimPairPwm<Tim, ch, top>` (the complementary pair with dead time, on TIM1 and TIM8 alone) (`stm32f4/tim.hpp`) | the G0's two names on this family's timers; the dead time is counted in tDTS, the timer clock divided by CR1.CKD and not by the counter's prescaler, so a pair holds the same absolute dead time at every PWM frequency; the timers take a static clock alone, as on the G0, and what they count is HCLK or twice their APB clock (RCC_DCKCFGR's TIMPRE), never the bus rate |
| ch32v00x | `TimPwm<Tim, ch, top>`, `TimPairPwm<Tim, ch, top>` (the complementary pair: under TIM1's break unit as OCx/OCxN, or on TIM2 as channel ch with channel ch + 2 under DTCR, the dead time in each block's own unit) (`ch32v00x/tim.hpp`) | the G0's two names on the F1's timers; a TIM2 pair costs two channels where a TIM1 pair costs one; the timers take a static clock alone, as on the G0 |
| ch32vx03 | `TimPwm<Tim, ch, top>`, `TimPairPwm<Tim, ch, top>` (the complementary pair with dead time, on the advanced timers alone: TIM1 everywhere, TIM8, TIM9 and TIM10 on the CH32V303RC and VC; the general-purpose and basic timers of this family have no break unit and no complementary output) (`ch32vx03/tim.hpp`) | the G0's two names on the F1's timers; the dead time is counted in tDTS, TIMxCLK divided by CTLR1.CKD and not by the counter's prescaler, so a pair holds the same absolute dead band at every PWM frequency; the timers take a static clock alone, as on the G0, and what they count is HCLK at every rate this stratum makes |
| ch32x035 | none | the timers are not written in this stratum; `Pin` is the one-step channel |
| rp2040 | `PwmOutput<pin, top>`, `PwmPair<pin_a, pin_b, top>` (a slice's A output and its complement on B) (`rp2040/pwm.hpp`); `PioPwm<n, sm, pin, period>` (`rp2040/pio.hpp`) | the channel is named by its pad and not by a timer (eight slices of two outputs, GPIO n belonging to slice (n / 2) mod 8), and `max` is TOP + 1; the pair's dead time is arithmetic, two levels and an inversion, because no dead-time unit exists here, so it lands on one edge in the plain mode and on both in the phase-correct one; and the second realization is not a timer at all but a state machine counting the period, the only stratum where a PWM channel is a program |
| rp2350 | `PwmOutput<pin, top>`, `PwmPair<pin_a, pin_b, top>` (`rp2350/pwm.hpp`); `PioPwm<n, sm, pin, period>` (`rp2350/pio.hpp`) | the RP2040's two realizations over twelve slices instead of eight, the pad still naming the channel and `max` still TOP + 1, with the four highest slices reaching a pad only in the package that bonds the upper pads and being repeating timers in the other; the dead time is arithmetic here too; and the divider's last step is a whole 256, the chapter forbidding a fraction there, which is the one arithmetic difference a caller can see |
| host | none | - |

## Style rules

- No `Ao` suffix on active-object class names. In an active-object
  framework every service is an active object, so the suffix is noise;
  the class is named for what it is (`SpiBus`, `SerialPort`,
  `Blinker`). The term stays where it is information: the
  `ActiveObject` concept and the prose.
- Private members carry a trailing underscore (`head_`), not `m_`.
- Queues speak push and pop. A consumer handed an element in place
  takes it, may hold it, and releases it: `EventQueue::take()` hands
  over the slot's number and the element is read where it lies with
  `at()`; `hold()` keeps the slot past the dispatch and `release()`
  gives it back, in any order ([kernel.md, "Queues"](kernel.md#5-queues-kernelevent_queuehpp)).
- No `*_from_isr` doubling of an API: one always-safe operation,
  revisited only with measurements ([ring.md](ring.md)).
- No redundant `inline` on in-class definitions.
- `std::optional` returns instead of a bool and an out-parameter.
- A header's comment explains the concurrency model and the reason
  for each tradeoff; that comment is the API documentation.
- **Integer widths.** `int` is whatever the target's ABI says (16 bits
  on the AVR, 32 on the others), and the language's integer promotions
  bring it back invisibly into every expression. The rules are
  therefore about data and arithmetic, not about banning a keyword.
  First, every stored or exchanged value (struct fields, event
  payloads, register values, protocol bytes) uses an explicit-size
  type (`uint8_t`, `int16_t`, ...). Second, any arithmetic whose
  result can exceed 16 bits names its width itself: a `UL` suffix on
  the literal, an explicit-width accumulator, or a cast on an operand
  (`uint32_t{a} * b`), never on the result, which wraps before the cast
  reaches it. Unsigned wrap is defined behaviour, so no warning level
  reports it; only this rule and the host and AVR double-width builds
  stand between a 16-bit product and a silent zero. Plain `int` and
  `auto` stay welcome for short-lived locals and loop indices, where a
  ban would be noise the promotions bypass anyway. The native test
  build runs under UBSan (`-fsanitize=undefined`, non-recovering), so
  the signed forms of this mistake fail the test run loudly.
- **Applications never touch registers.** `PORTx.`, `VPORTx.`,
  peripheral structs and vendor bit masks appear only inside the
  target stratum's drivers. An application that needs an operation
  the driver lacks adds it to the driver (`Pin::pullup`,
  `PinSet::read`) and never works around it. The only vendor glue an
  application may contain is the interrupt vector binding. This does
  not mean the target stratum abstracts every peripheral in advance:
  it grows on demand, one operation or one table row at a time, and
  knob-heavy peripherals (ADC, TCD, CCL) get a `constexpr`
  configuration struct with designated initializers, not an API that
  abstracts their meaning. The rule holds as long as adding to the
  driver stays easier than bypassing it. Bench-only probes are
  declared outside it, and what they find goes into a driver
  afterwards.

## ISR binding pattern

Drivers expose the handler body (`rxc()`, `dre()`, `pit()`, `isr()`);
the application binds the vector (`ISR(USART2_RXC_vect) {
Serial::rxc(); }`). Vector names are target-specific glue and live in
the application, never in portable code.

The bodies are `[[gnu::always_inline]]`. A body has exactly one call
site by construction, so inlining costs no flash, and with the body
visible the compiler saves only the registers actually used (measured
on the AVR: 16 pushes before, 8 or 9 per handler after). A binding
that posts is flattened (`ISR(vector, ISR_FLATTEN)` with avr-libc,
`[[gnu::flatten]]` on a plain handler): `post()` is a template an
image usually calls from more than one place, so the compiler keeps it
out of line, and a call inside a vector costs a core that saves the
caller's registers in software the whole set. Flattened, the post is
inlined into that vector alone and the kernel stays as it is
(measured on the AVR128DB48's console receive: 16 pushes to 10, 108 to
91 cycles a byte, 60 bytes of flash).

On a core whose hardware prologue saves no floating-point register
(the CH32V303's QingKe V4F), a binding in an image built with the F
extension is a naked trampoline, a call of an ordinary function and
then `mret`, so a call in the body costs no f-register save; the
stratum's check walks every such body in the linked image and fails on
a floating-point instruction it reaches. A leaf, which saves nothing
under the attribute, keeps the attributed handler, held to being a
leaf by the same check ([the CH32V203/CH32V303 platform
page](../ch32vx03/platform.md)).

A family whose linker script carries a `.fast_text` section may place
a binding in SRAM; the SAM C21's page says what it buys
([the SAM C21 platform page](../samc21/platform.md), "A handler in
SRAM"). Whether a driver's hot path runs from SRAM is the program's
choice, per driver (`util/placement.hpp`): the driver takes a placement
tag, `InFlash` by default (its body inline where it was, not a byte of
SRAM spent) or `InRam` (each hot path one flattened instance in
`.fast_text.brio`), and the costs it states follow the tag.
`.fast_text` is code that may run from SRAM, for speed, and a program
names it on a function of its own; `.ram_text` keeps the name of code
that must, for correctness (the RP2040's flash engine). A family whose
linker scripts collect `.fast_text` says so on its platform type, and
`InRam` anywhere else does not compile. The CH32X035 is the family the
tag was born on ([the CH32X035 platform page](../ch32x035/platform.md),
"A handler in SRAM"). On the SAM C21 the Cortex-M0+'s long-branch
veneer from the flash into SRAM makes a program-placed vector the
cleaner way on most paths ([the SAM C21 platform
page](../samc21/platform.md)). The STM32G0 places it too, its SPI host
and USART task gaining behind a vector left in the flash, and the
STM32F4 is measured to lose ([the STM32G0 platform
page](../stm32g0/platform.md), [the STM32F4 platform
page](../stm32f4/platform.md)). On the RP2040 and the RP2350, which
execute in place behind a cache, the placement buys the cold path and
costs the warm one its call ([the RP2040 platform
page](../rp2040/platform.md), [the RP2350 platform
page](../rp2350/platform.md)). The CH32V203 and the CH32V303 run from a
zero-wait window SRAM never beats, and do not offer it
([the CH32V203/CH32V303 platform page](../ch32vx03/platform.md)).
