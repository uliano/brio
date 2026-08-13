# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Interaction

Claude will interact with the user in italian but all the edits in the files, including comments, will be done in english.

## Allowed text symbols

Only ASCII <=127 characters will be allowed in the documents.

## Documentation

Design documentation lives in `docs/` (plain Markdown, GitHub-rendered,
MkDocs-ready; see docs/README.md for the rules). Any change that alters
a documented design decision updates the matching docs/design/*.md in
the same change. Headers remain the canonical API reference - docs hold
the WHY and the contracts, never duplicated signatures. New design
decisions land in docs/design/ (dated), not as further growth of the
decision log below; the log below is the historical seed and stays
authoritative for me until explicitly migrated.

## Project Overview

Bare-metal C++ experiments on an **AVR128DB48** (48-pin, 128 KB flash, 16 KB
SRAM), programmed/debugged with an **Atmel-ICE** over UPDI. No Arduino
framework: just avr-gcc + avr-libc.

Long-term goal: grow `lib/brio` into a small **modern-C++ framework covering
at least the AVR DA and DB families**. Design decided on 2026-07-21 (after a
full critical review of the AVR-Multislope heritage):

- **everything resolves at compile time**: peripheral drivers are static
  (monostate) class templates (`Uart<2, Route::alt1>`, `BasicTicker<1024>`),
  services are templated on the transport and constrained by concepts
  (`ByteSink`/`ByteSource` in stream.hpp) - NO virtual interfaces, no
  runtime singletons (the old ByteStream base class is gone);
- **drivers move bytes, they do not format**: text output is
  `brio::print(sink, ...)` in print.hpp, variadic free functions over any
  ByteSink; new types become printable via a `print_one(sink, value)`
  overload (found by ADL). print BLOCKS until the sink accepts (no silent
  truncation) - so print only after init + sei();
- **protocol layer is push-based**: proto/line_parser.hpp's LineAssembler
  is fed bytes and returns completed lines - no reference to any transport,
  host-testable; parsers/router are plain static code;
- **one flat namespace `brio`**, no nesting (namespaces prevent clashes, they
  are not architecture); apps may `using namespace brio;`, headers always
  qualify. Types PascalCase, functions/constants snake_case;
- **monostate drivers double as zero-cost tags**: `constexpr Serial serial;`
  is an empty object usable as `print(serial, ...)` argument;
- the toolchain ships the **freestanding libstdc++** (type_traits, concepts,
  bit, span, optional, expected, ... - no chrono/charconv/iostream): use it
  instead of hand-rolled traits;
- the project standard is **gnu++23** (set in tools/pio_flags.py); C++26
  features already implemented by gcc 16 may be used when genuinely needed
  (bump the -std flag in that case). C++23 idioms in active use:
  `static_assert(false, ...)` in discarded if-constexpr branches turns
  "peripheral/port not present on this device" into a clear compile error
  (pin.hpp, uart.hpp), `std::to_underlying` for enum codes (clock.hpp).

Family differences are handled with device-macro guards (e.g.
`CLKCTRL_XOSCHFCTRLA` exists only on DB, `PORTB`/`PORTE`/`PORTG` depend on
the package) so the same headers build for both; the USART pin/PORTMUX
table inside uart.hpp is the designated seed of a future per-family device
header.

Next stage (decided 2026-08-13): evolve the framework toward a QP/QV-style
**active-object kernel** - cooperative run-to-completion scheduler, event
queues, state-machine-based active objects, time events - written from
scratch (clean room: the concepts from Samek's book, never the QP source,
which is GPL/commercial), with event-driven non-blocking drivers at least
for I2C and SPI on top: a kind of modern Arduino. Development happens in
THIS repo (multi-app system as testbed, lib/brio as foundation).

**Governing rule for that evolution - nothing is settled.** Reusing the
existing code is fine only as long as it does not limit the design above
it: whenever a limitation could be overcome by rewriting the architecture
of what sits below, the rewrite wins. No layer is taken for granted or
considered done - clock, pin, ring, uart, ticker, print, everything up to
and including the design decisions of 2026-07-21 is up for continuous,
radical, deep revision, and every stage of the work requires critical
analysis at ALL levels of the stack, not just the one being added.

AO kernel decisions taken so far:

- **Name (2026-08-13): the framework is `brio`** (from "con brio" -
  Italian musical marking for liveliness - deliberately not tied to the
  AVR Dx silicon so nothing prevents generalization to other targets,
  and deliberately far from QP's q-naming for clean-room distance). One
  flat namespace `brio` for everything, library folder lib/brio; the
  former `dx` namespace and lib/core folder were renamed on the same
  day (cheapest possible moment: eight headers, three apps, no external
  users). "avr-dx" as a library.json keyword still legitimately refers
  to the chip family.

- **Events (2026-08-13): value semantics, per-AO variant.** Events are
  COPIED into per-AO queues - no pools, no reference counting (QP-style
  pools reconsidered only if large events or heavy multicast ever
  materialize). Each AO declares its own event type: a `std::variant` of
  small trivially-copyable structs (enforced via static_assert). The
  8-byte size target is a GUIDELINE for the event ENVELOPE, not a law
  (refined 2026-08-13): every slot of a queue pays that AO's largest
  alternative and the push copies it with interrupts masked (~1 us per
  8 bytes at 24 MHz), so keep the max alternative small - but per-AO
  deviations are legal with numbers in hand (the queues are per-AO,
  nobody else pays). Payloads above the budget travel BY REFERENCE
  (pointer/span, with an ownership rule) when the data already lives in
  a structurally necessary buffer (rings, line buffers); BY VALUE when
  lifetime simplicity is worth the RAM. Nothing is ever chopped into
  multiple events: one event = one envelope, the cargo stays put.
  Plain shared structs (e.g. `TempChanged`) are
  the lingua franca between publishers and subscribers - no global
  signal enum, no system-wide event type. `publish()` is layered on top
  of `post()` with compile-time subscriber lists, one copy per
  subscriber ("commands are addressed, facts are published"; a concept
  checks the subscriber's variant accepts the notification). Queue depth
  is a per-AO template parameter sized on that AO's real burst; the slot
  size is that AO's own largest alternative, not a system-wide max.
  Measured on avr-gcc 16.2 -Os: std::visit on a 4-alternative variant
  compiles to a plain switch (zero indirect calls), +30 bytes flash and
  identical RAM vs a hand-tagged union.
- **Event queues (2026-08-13): one MPSC queue per AO, short critical
  sections; Ring stays a driver tool.** Under the cooperative QV
  scheduler the only real concurrency is ISR vs main loop (AVR has no
  CAS - the honest primitive IS the brief cli section), so the per-AO
  queue is a ring whose push saves SREG, disables interrupts, copies the
  event (<= 8 bytes, ~1 us at 24 MHz) and restores; the scheduler's pop
  protects only the index update. Posting from an ISR must never block.
  Per-producer SPSC fan-in was rejected (RAM x sources, scheduler scans,
  loses cross-source event ordering) unless a jitter-critical ISR ever
  demands it. `Ring` (SPSC, no cli, byte indices) is NOT the event
  queue: it stays at the BYTE level inside drivers (UART now, I2C/SPI
  next) - the ISR pushes bytes lock-free, the driver AO condenses them
  into few events on the MPSC queue (bytes at high rate, events at low
  rate).
- **Queue overflow (2026-08-13): count always, policy knob for the
  reaction.** A full queue is a sizing mistake, not a runtime condition
  to handle. Per-queue overflow counters are ALWAYS maintained (same
  pattern as the uart RXDATAH error counters - named static symbols,
  inspectable from gdb, nearly free in release). The reaction is a
  kernel compile-time knob: OverflowPolicy::count (drop the event, keep
  running - the release default) or OverflowPolicy::panic (sensible in
  -debug envs, where the BREAK inside panic lands gdb right on the
  undersized queue). post() from ISR never blocks; no bool-returning
  post spreading untested error branches.
- **Panic facility (2026-08-13): one hook, stock reporters.**
  brio::panic(code, ctx) is [[noreturn]]: cli -> breadcrumb (cause +
  queue/AO id) into a .noinit variable that SURVIVES reset (cross-check
  RSTCTRL at next boot) -> asm("break") (halts in gdb if the OCD is
  active, plain NOP otherwise - a free breakpoint that does not consume
  the 2 hw slots) -> app-pluggable Reporter. The kernel knows no LED:
  the app plugs one reporter from a small stock library - halt (default:
  cli+loop), blink_forever<Led> (bench), reset_now (field: watchdog
  reset, the breadcrumb is reported at next boot by serial print or by
  blinking the code at startup), blink_then_reset<Led, cycles> - or
  writes its own. The breadcrumb is written BEFORE any reporter runs,
  so the information is safe whatever the manifestation does.
- **Scheduler (2026-08-13): compile-time QV loop.** `Kernel<Ao1, Ao2,
  ...>`: AOs are monostate classes like the brio drivers, visited via fold
  expressions - no virtual base (it would also force the global event
  type rejected above). Priority IS the pack order, first = highest; no
  separate priority table to keep coherent. One event per iteration:
  pop the highest-priority non-empty queue, dispatch run-to-completion,
  rescan from the top (an urgent event arriving during a slow dispatch
  is served right after it). Starvation of low-priority AOs under fixed
  priority is by definition a sizing error - the overflow counters make
  it visible. All queues empty -> IDLE sleep via the cli-check then
  sei+sleep_cpu idiom (sei takes effect after the following instruction,
  so the lost-wakeup race is closed by the silicon); IDLE keeps
  UART/RTC/peripherals alive, deeper sleep modes are app policy, not
  kernel business.
- **State machines (2026-08-13): HSM-ready contract, flat (QP-style)
  implementation.** The dispatch contract is fixed NOW so hierarchy is
  a later additive extension, never a breaking change: the current
  state is a handler function `Status handler(Ao&, const Event&)`; the
  Status outcomes are handled | unhandled | transition(target)
  (`unhandled` today means "ignore", tomorrow "bubble to parent" - it
  IS the HSM hook); entry/exit are reserved events the transition
  machinery delivers systematically (never hand-written at call sites);
  the kernel delivers an initial `init` event so every AO starts by
  transitioning into its first real state (no half-initialized states).
  On this contract the implementation starts FLAT (each state = one
  readable function, transition = returned to the dispatcher, one
  2-byte indirect call per dispatch); full HSM (parent pointers, event
  bubbling, LCA entry/exit chains) gets built only when a real AO
  demands it - and if the contract then turns out wrong, the governing
  rule already says the rewrite wins. Trivial AOs may legally keep a
  plain switch inside one handler. States-as-types (sml-style,
  compile-time-checked transitions) noted as a possible future probe,
  rejected as foundation: heaviest machinery, hostile errors, app-author
  readability drops. IMPLEMENTED in kernel/fsm.hpp (2026-08-13):
  `Fsm<Derived, Alts...>` CRTP-monostate base building the AO's Event
  as std::variant<Entry, Exit, Alts...> - the reserved events are
  kernel structs PREPENDED to the variant, received through the same
  visit as ordinary events, empty so queue slots do not grow, delivered
  synchronously by the transition machinery and never posted. start()
  arms the initial state and delivers the first Entry (the
  "kernel-delivered init": Kernel will call start() for every AO before
  the loop). Entry may chain transition() - pass-through states for
  free; Exit's return is deliberately ignored (exit is an action, not a
  decision). Derived exists so two AOs with identical alternatives get
  distinct machines.
- **Time events (2026-08-13): timers post events; expiry runs in the
  loop, not in the ISR.** In the AO world a timer never runs user code:
  it posts an event to its owner AO ("in 500 ms post EvTimeout to X"),
  so the logic stays serialized in the AO's dispatch. The old
  callback-based `Timer<Unit>` is superseded for kernel apps (a
  callback fires outside the AO serialization - the antithesis of the
  model); `Ticker` survives as the timebase (RTC/PIT @ 1024 Hz, alive
  in IDLE sleep). Expiry processing is in the KERNEL LOOP (T2): the PIT
  ISR just ticks and, by firing, wakes the CPU; each loop turn compares
  the tick with armed deadlines and posts matured events in main
  context. The ISR-side alternative (T1) was rejected: a periodic ISR
  whose duration grows with armed timers is the antithesis of the
  short-dumb-ISR style, and buys almost no real precision since any
  posted event waits for the current run-to-completion step anyway.
  TimeEvents are static objects owned by their AO (arm/disarm explicit,
  one-shot or periodic), linked into an intrusive list when armed - no
  allocation. Periodic re-arm is drift-free: next = previous deadline +
  period, never now + period. Units are Ticker ticks, with constexpr
  millis/secs conversions.
- **Host-testability (2026-08-13): kernel templated on a Platform
  concept, no #ifdef.** Queues, scheduler, FSM contract and time-event
  bookkeeping are pure logic; the only hardware touchpoints are wrapped
  in a Platform policy declared by a concept (platform.hpp): a RAII
  critical-section type (whose enter/leave are also compiler memory
  barriers - so shared data needs no volatile), idle(), break_here(),
  now(). AvrPlatform (platform_avr.hpp - the only kernel-family header
  allowed to include AVR headers) implements them with intrinsics;
  HostPlatform (platform_host.hpp) gives a depth-counting critical
  section (single-threaded tests), a test-controlled virtual clock and
  recording idle()/break_here() - time becomes deterministic arithmetic
  in tests. There is NO platform default in kernel templates (a default
  would hardwire an AVR include - see the generalization rule): the app
  names its platform once. Preprocessor #ifdef branches were rejected:
  implicit contract, host path rots unseen. Host tests run in
  [env:native] (pio test -e native, doctest; test_ignore=* in the base
  env keeps `pio test` off the probe) and first cover what is hard to
  provoke on silicon: queue overflow + counters, entry/exit ordering,
  drift-free re-arm, scan priority, MPSC stress with simulated
  producers.
- **Generalization rule (2026-08-13): today it is AVR Dx, tomorrow it
  could be anything else.** Always leave the door open to porting: no
  #ifdef where a template/concept boundary can do the job, no AVR
  includes outside the platform_avr/driver layer, target-specific facts
  live behind the Platform concept or in the drivers. brio the kernel
  must never know which silicon it runs on.
- **Time and tick rate (2026-08-13): the tick is opaque, the rate is a
  target constant.** The power-of-two tick rates are a truth of the AVR
  PIT (32768 Hz dividers), not of the world: the next target candidates
  (STM32G0x0/x1, ATSAMC/D, CH32V00x) tick from SysTick at any rate,
  canonically 1000 Hz. Therefore: the Platform concept carries
  `ticks_per_second` as a positive compile-time constant; the kernel
  reasons in opaque ticks and assumes NOTHING about the rate (no pow2,
  no 1-tick-=-1-ms); conversions are constexpr in kernel/time.hpp with
  CEIL semantics ("at least this long" - a timeout never fires early;
  identity folding when tps=1000). TimeStamp's fraction changed from
  ticks to MILLISECONDS (0..999): a tick-based fraction would change
  meaning with the silicon, ms are self-describing; the producing
  driver converts with its known rate, sub-ms measurement is what raw
  Platform::now() ticks are for. Printed as "12.045s" (zero-padded).
  BasicTicker stays declaredly AVR (pow2 asserts, PIT, skip
  correction) in avrdx/ - future tickers are sibling drivers with
  their own truths; no preemptive "generic ticker", the concept IS the
  abstraction (generalize on the second real specimen). Deadline
  arithmetic over wrapping counters (signed difference) is documented
  where it will live: the time-event code.
- **ReplyTo (2026-08-13): replies return to sender.** The third
  delivery primitive after post/publish, in kernel/post.hpp:
  `ReplyTo<Payload>` is a one-function-pointer capsule (2 bytes on AVR,
  trivially copyable - it travels INSIDE the request event) built at
  compile time via `reply_to<RequesterAo, Payload>()`, same thunk
  technique as TimeEvent firing. A service (SPI/I2C bus AO) calls
  `req.reply.send(payload)` without knowing the requester; a
  default-constructed capsule is null and send() is a no-op
  (fire-and-forget for free). A requester whose variant cannot hold
  Payload fails to compile at the reply_to site. This is the return
  channel every bus AO uses; the bus AO's request QUEUE doubles as the
  bus arbiter (contending clients are serialized by the kernel).
- **SPI AO (2026-08-13): the bus AO owns arbitration, an internal
  pending FIFO owns the waiting.** util/spi_ao.hpp is generic over the
  Bus engine (avrdx-side driver moving bytes under interrupts; a fake
  in host tests). Design discovery worth remembering: the event queue
  ALONE cannot arbitrate a bus, because a transaction OUTLIVES the
  dispatch that starts it (it completes on interrupts later) - while
  busy the kernel still delivers the next request, which therefore
  waits in a small internal pending FIFO (main-context only, no
  critical sections). Full FIFO = immediate SpiDone{spi_rejected}
  reply + counter: never silent, never blocking. Engine handshake
  mirrors the uart edge pattern: the app ISR glue posts
  TransferDone{status} when the engine finishes; the AO replies
  through the request's ReplyTo capsule and starts the next pending
  transfer. idle/busy as a real 2-state FSM. The request event
  (~14-byte descriptor) exceeds the 8-byte envelope guideline: a
  recorded legal deviation - the request IS the arbitration token.
  Transaction descriptor (target side, upcoming avrdx/spi.hpp):
  {cs, dc, cmd span (DC low), tx/rx spans (full-duplex capable)} -
  covers write-only displays (cmd+data, DC toggling inside one CS
  window), rx-only ADCs (MCP3550), true loopback (tx+rx), SD cards.
- **Serial AO (2026-08-13): bytes below, line events above, ownership
  by reference.** The Uart driver stays the low level (rings + ISR
  bodies, untouched roles); `SerialAo<Transport, P, LineSink>`
  (util/serial_ao.hpp) is the kernel citizen above: Uart::rxc() now
  returns the RX ring's empty->non-empty EDGE and the app ISR glue
  posts RxActivity on true (no event flood, no lost wakeup: only
  draining empties the ring, so the next byte is an edge again).
  SerialAo drains, feeds two ping-pong LineAssemblers, posts
  LineReceived{char*} (reference: the 80-byte line never enters a
  queue; valid and mutable only during the sink's dispatch). With both
  buffers in flight it stops draining (the ring absorbs - that is its
  job) and SELF-POSTS RxActivity. SCHEDULING CONTRACT: the line
  consumer must precede SerialAo in the Kernel pack - the kernel then
  consumes every posted line before SerialAo runs again, which is why
  in_flight can reset at dispatch entry and two buffers are exactly
  sufficient. TX stays the blocking try_put print: the drain side is
  an ISR (preempts the loop, so the spin always progresses - stall,
  not deadlock), worst case ~2 ms at 460800, zero when the ring has
  room; measured cost of write_byte ~45-50 cycles/byte (<10% of the
  21.7 us wire time). The full-queue policy costs nothing on the
  non-full path (the check exists anyway), so it is pure failure
  semantics: RX drops+counts (the world cannot be paused), TX blocks
  (we can wait, and half-messages are worse than late ones); a
  message-atomic drop ("say it all or say nothing" + counter) is the
  noted future option for telemetry. Uart ring defaults resized
  512/256 -> 64/256 (8-bit indices on both, ~450 bytes RAM back).
- **Layering (2026-08-13): four strata as directories under
  lib/brio/src/, includes always carry the stratum prefix.** kernel/
  (pure logic, includes nothing of brio), util/ (pure services - may
  include kernel/, NEVER a target), avrdx/ (everything that knows
  avr/io.h: drivers + AvrPlatform; implements kernel/util concepts,
  depends on them, never the reverse), host/ (the test "target").
  Two targets never meet in one binary, so a future ch32v00x/ is a
  sibling of avrdx/ and the flat `brio` namespace stays collision-free.
  The include prefix (`#include "avrdx/uart.hpp"`) makes an app's
  portability readable at a glance. One PlatformIO library, NOT one per
  stratum: discipline comes from this rule, multi-library would add
  ceremony without enforcement. Shared pure types produced by a target
  and consumed by util go in util/ (e.g. util/timestamp.hpp, extracted
  from ticker.hpp when this rule caught print.hpp -> ticker.hpp as a
  layering violation).
- **Style rulings (2026-08-13)** (critical re-read of the inherited
  ring.hpp; good choices kept, questionable ones dropped): private
  members use a trailing underscore (head_), not m_; queues speak
  push/pop (put/get remains legacy in Ring); no *_from_isr API doubling
  in the kernel - one always-safe operation, revisit only with
  measurements; no redundant `inline` on in-class definitions;
  std::optional returns instead of bool + out-parameter; keep the
  header-comment style that explains the concurrency model and the
  WHY of each tradeoff.
- **ISR binding pattern (2026-08-13): drivers expose the handler BODY,
  the app binds the vector - and the body is [[gnu::always_inline]].**
  Vector names (ISR(USART2_RXC_vect), RISC-V attributes...) are
  irreducibly target-specific glue: they live in the app (or a future
  board/target file), never in portable code; the driver provides
  rxc()/dre()/pit() functions. The always_inline attribute is what
  makes the pattern FREE: an ISR body has exactly one call site by
  construction, so inlining costs no flash, and with the body visible
  the compiler saves only the registers actually used instead of the
  full ABI call-clobbered set. Measured on the serial app: 16 pushes ->
  8/9 per ISR. Without the attribute, -Os refuses to inline the larger
  bodies and every interrupt pays ~40 wasted cycles of push/pop.
  Survives -fno-inline debug builds (like _delay_ms).

The general structure (toolchain wiring, custom board JSON, Atmel-ICE upload,
the disassembly post-build script) and the `lib/brio` library are inherited
from [uliano/AVR-Multislope](https://github.com/uliano/AVR-Multislope).

The **multi-app** system is inherited from
[uliano/blackpill-experiments](https://github.com/uliano/blackpill-experiments):
every `src/apps/<app>.cpp` has its own `main()` and becomes its own pair of
envs `[env:<app>]` / `[env:<app>-debug]`. Shared code lives in `lib/brio`
(a PlatformIO private library: the LDF compiles and links it automatically
for every env whose app includes one of its headers - no src_filter entry).

There are currently NO external peripherals on the board except the serial
link (CH340 on USART2 ALT1, PF4/PF5). Device drivers for external chips will
be (re)added one by one when the corresponding hardware gets wired up.

## Toolchain

Self-built avr-gcc 16.2 + avr-libc + avr-gdb 17.2 + avrdude 8.1 (built by
/sw/src/build-avr.sh), used in place via `symlink://`:

    /sw/avr        (symlink to /sw/avr-16.2)

A minimal `package.json` manifest inside /sw/avr-16.2 makes the folder usable
as a PlatformIO `symlink://` package; build-avr.sh's finalize stage generates
it on every (re)build, so toolchain upgrades keep it (without it, PlatformIO
fails with MissingPackageManifestError). PlatformIO's bundled toolchain-atmelavr /
avrdude 7.1 are NOT used (too old for AVR-Dx).

## Debugging (PyAvrOCD + Atmel-ICE over UPDI)

Debugging uses [PyAvrOCD](https://pyavrocd.io) as GDB server (installed via
`uv tool install --python-preference only-managed --python 3.12 pyavrocd`,
exposed at ~/.local/bin/pyavrocd, venv on a uv-managed CPython 3.12 so it is
independent of the distro Python) together with the toolchain's avr-gdb 17.2. It is wired in platformio.ini as `debug_tool = custom`
(server on port 40044), mirroring felias-fogg's platform-atmelavr wiring;
switch to `debug_tool = pyavrocd` once the integration lands in the
platform-atmelmegaavr fork referenced by `platform =`.

- Always debug the `<app>-debug` env: start from the IDE (Run and Debug,
  after selecting the `<app>-debug` project env) or with
  `pio debug -e <app>-debug`. The plain `<app>` env is the release build.
- udev rules for the EDBG probes: /etc/udev/rules.d/99-edbg-debuggers.rules
  (unplug/replug the probe after installing them).
- Do NOT add `-mrelax` to the build flags: PyAvrOCD refuses ELF files built
  with it (distorted line-number info).
- Useful GDB console commands: `monitor info`, `monitor reset`,
  `monitor ioregister <name>` (read/write an I/O register by name).
- Breakpoints: the UPDI OCD has 2 hardware breakpoints; GDB borrows one for
  its temporary breakpoints (tbreak main, next over a call, finish), so plan
  for ONE free user breakpoint. Single-stepping is native OCD, costs nothing.
  The server runs with `--breakpoints hardware` (set in debug_server): extra
  breakpoints are refused ("Cannot insert breakpoint") instead of becoming
  software breakpoints that rewrite flash (1000-cycle endurance on UPDI
  parts). Session escape hatch: `monitor breakpoints all`.

### Toolchain DWARF caveat (GCC 16.x vs gdb)

GCC 16.x (verified on 16.1 and 16.2) emits a DWARF5 line table with a
duplicate file entry for the main source and switches file mid-sequence
around inlined code. Both avr-gdb 17.2
and host gdb 15.1 then silently drop part of the line table: file:line
breakpoints fail to bind ("No compiled code for line N") at ANY -O level
above 0, while function breakpoints (e.g. `tbreak main`) still work. This is
why platformio.ini sets:

- `build_unflags = -flto -fuse-linker-plugin` (LTO makes it worse and buys
  nothing at this firmware size), and
- `debug_build_flags = -Og -g3 -ggdb3 -fno-inline` (no inlining -> no
  mid-function file switches -> line breakpoints bind; always_inline code
  such as `_delay_ms` stays inlined and keeps correct timing).

Debug and release flags are fully separated: `debug_build_flags` only applies
to `build_type = debug`, i.e. to the generated `[env:<app>-debug]` envs, and
tools/pio_flags.py adds `-Os -g` only to release builds (it checks
`env.GetBuildType()`). Release firmware therefore never carries `-Og` or
`-fno-inline`; the only global concession is the LTO disable, which is a
deliberate choice (unreadable .lst, zero size benefit) rather than a debug
leftover.

Residual limitation: a line whose only content is a call into an
always_inline system-header function (e.g. a bare `_delay_ms(500);`) still
cannot take a line breakpoint; break on a neighbouring line instead. Worth
re-testing after any avr-gcc / avr-gdb rebuild; candidate for an upstream
bug report (minimal repro: any -O1 build of blink.cpp).

## Multi-app workflow

Each experiment is one `src/apps/<name>.cpp` with its own `main()`. The env
blocks are AUTO-GENERATED into `apps.ini`, TWO per app:

- `[env:<name>]` - release build (`-Os`), for production uploads;
- `[env:<name>-debug]` - `build_type = debug` (`-Og -g3 -ggdb3 -fno-inline`
  via `debug_build_flags`), for every debug session.

```bash
# After adding/removing a file in src/apps/, regenerate the env list:
python tools/gen_apps.py
# (or the VS Code task "PIO: regen apps"), then reload the PlatformIO project.
```

`apps.ini` is committed so a fresh clone already has the envs.

## Build and Development Commands

```bash
# Build the default app (blink, release)
pio run

# Build a specific app (release / debug)
pio run -e blink
pio run -e blink-debug

# Build and upload via Atmel-ICE (UPDI)
pio run -e blink -t upload

# Debug (builds and flashes the -debug env, then attaches)
pio debug -e blink-debug

# Host unit tests (brio kernel logic, no hardware, doctest)
pio test -e native

# Clean
pio run -t clean
```

`pio run` (build only) and `pio test -e native` need NO hardware connected;
the Atmel-ICE is only touched on `-t upload`. Tests run ONLY on the native
env (test_ignore = * in the base [env] keeps a bare `pio test` away from the
probe: AVR envs just skip).

Upload gotcha: connect the Atmel-ICE **AVR** port (not the SAM port). Wrong port
-> avrdude still detects the ICE on USB but `Vtarget` reads ~1.71 V and the UPDI
sign-on fails with `Bad response to AVR sign-on command: 0xa0`.

## Layout

```
platformio.ini          base [env], toolchain, Atmel-ICE upload, debug wiring
apps.ini                generated: [env:<app>] + [env:<app>-debug] per app
boards/AVR128DB48.json   custom bare-metal board (128K flash / 16K RAM)
tools/gen_apps.py        scans src/apps/*.cpp -> apps.ini
tools/pio_flags.py       per-language AVR flags (build-type aware) +
                         IntelliSense include paths (skips [env:native])
tools/gen_lst.py         post-build: firmware.lst (disassembly) + firmware.map
                         (skips [env:native])
src/apps/<app>.cpp       one main() per experiment (ISR vector bindings
                         live HERE, not in portable code)
test/test_*/main.cpp     host unit tests (doctest), run via pio test -e native
lib/brio/                the brio framework (auto-linked by the LDF), all in
                         namespace brio, header-only, four strata (see the
                         layering rule; includes carry the stratum prefix):
  src/kernel/            pure kernel logic - includes NOTHING of brio
    platform.hpp           the Platform concept: what the kernel needs from
                           the machine (CriticalSection, idle, break_here,
                           now, ticks_per_second)
    time.hpp               constexpr tick conversions parameterized on the
                           platform rate (ceil semantics: never early)
    fsm.hpp                Fsm<Derived, Alts...> HSM-ready flat state
                           machines: state = handler function, Entry/Exit
                           reserved variant alternatives, transition
                           chaining, start() = initial entry
    post.hpp               ActiveObject concept + post<Ao>(ev) (addressed,
                           reserved events excluded) + publish(Subscribers
                           <A,B...>{}, ev) fan-out + ReplyTo<Payload>
                           request/reply return capsule
    time_event.hpp         TimeEvent<P, Ao, Ev> posts payload to its AO on
                           expiry; intrusive armed list, drift-free
                           periodics, wrap-safe deadlines, RAII disarm;
                           processed by the kernel loop (T2)
    kernel.hpp             Kernel<P, Aos...>: init_all/step/idle_if_empty/
                           run - the QV loop (priority = pack order, one
                           event per turn, race-free IDLE sleep)
    panic.hpp              panic<P, Reporter>() [[noreturn]] + PanicCode +
                           HaltReporter + take_panic_record<P>() boot-side
                           fetch-and-clear
    event_queue.hpp        EventQueue<E, depth, Platform> - per-AO MPSC
                           queue, saturating overflow counter, optional pop
  src/util/              pure services - may include kernel/, never a target
    stream.hpp             ByteSink / ByteSource / ByteTransport concepts
    print.hpp              print(sink, ...) variadic formatting, hex/fixed/
                           sci wrappers, crlf; extend via print_one + ADL
    timestamp.hpp          TimeStamp value type, ms fraction (produced by
                           the timebase driver, printed by print.hpp)
    serial_port.hpp        SerialPort<Transport, P, LineSink>: RX bytes ->
                           LineReceived events (ping-pong buffers,
                           self-post backpressure, consumer-above-producer
                           scheduling contract; born SerialAo, Ao suffix
                           dropped 2026-08-13)
    spi_bus.hpp            SpiBus<Bus, P>: SPI bus arbiter - pending FIFO,
                           reject-when-full, ReplyTo completion, engine
                           handshake via TransferDone (born SpiAo, Ao
                           suffix dropped 2026-08-13)
    proto/line_parser.hpp  LineAssembler (push) + console/SCPI parsers +
                           CommandRouter<Sink>
  src/avrdx/             everything that knows avr/io.h
    platform_avr.hpp       AvrPlatform: SREG-save critical section, IDLE
                           sleep via sei+sleep, BREAK, Ticker-backed now()
    clock.hpp              DA/DB clock init: init_clocks() probing generic +
                           init_clock_24mhz() deterministic DB crystal path
    pin.hpp                Pin<'A',5> compile-time GPIO (VPORT fast paths) +
                           PinRef runtime descriptor (CS/DC in requests)
    ring.hpp               Ring<T,size> SPSC ring (index type auto-derived)
    uart.hpp               Uart<n, Route, rx, tx> static interrupt-driven
                           byte transport, RXDATAH error counters
    spi.hpp                Spi<n> master engine: two-phase descriptor
                           (cmd @ DC low, full-duplex data @ DC high),
                           per-byte ISR pump, CS owned by the engine
    ticker.hpp             BasicTicker<tps> static RTC/PIT timebase
                           (alias Ticker = BasicTicker<1024>)
  src/host/              the test "target"
    platform_host.hpp      HostPlatform: depth-counting critical section,
                           virtual clock, idle/break call recorders
```

## Build Artifacts

- ELF / HEX / MAP / LST: `.pio/build/<env>/`
- `firmware.lst`: source-interleaved disassembly (from tools/gen_lst.py)

## Clock note

The board has a **24 MHz crystal on PA0/PA1** (XOSCHF, a DB-family feature;
PA0/PA1 are therefore not available as GPIO). `lib/brio/src/clock.hpp`
(ported from AVR-Multislope's src/clocks.h) offers two entry points:

- `init_clocks()` - generic DA/DB probing init: OSCHF 24 MHz baseline, then
  probes DB crystal on PA0/PA1, EXTCLK on PA0, and a 32k crystal on PF0/PF1
  (with OSCHF autotune); returns a `ClockInitCode` describing what it found.
  For boards whose clock fixture is unknown.
- `init_clock_24mhz()` - deterministic DB path used by the apps in THIS repo:
  the crystal is a known fixture, start it (SELHF_XTAL, FRQRANGE 24M,
  CSUTHF 4k) and switch CLK_PER, falling back to the internal OSCHF @ 24 MHz.
  Returns true when running from the crystal. It does NOT touch XOSC32K.

Each app calls one of them as the first line of main(). `F_CPU=24000000`
comes from the board JSON and is correct for both sources.

There is NO 32.768 kHz crystal on this board: `Ticker::init()` selects the
RTC clock automatically (XOSC32K only if the clock init reports it running,
internal OSC32K otherwise). Do NOT enable XOSC32K (PF0/PF1) unless a 32k
crystal is actually fitted; the serial link uses USART2 ALT1 on PF4/PF5, so
PF0/PF1 are free.

## Ticker / timers note

`brio::Ticker` is a STATIC (monostate) class template - `BasicTicker<tps>`
with `using Ticker = BasicTicker<1024>` - not a runtime singleton: all state
is C++17 `static inline` in .bss, all methods are static, header-only, no
init order issues and no pointer indirection in the ISR. Apps wire it as:

    ISR(RTC_PIT_vect) { brio::Ticker::pit(); }
    ...
    brio::Ticker::init();   // after clock init, before sei()

Two bugs of the original AVR-Multislope ticker are fixed here (worth
back-porting): the H/L union word order made ticks() advance by 65536 per
tick, and millis() ran 0.7% fast because window position 0x00 was not
skipped. See the NOTE in lib/brio/src/ticker.hpp.

The legacy callback `Timer<Unit>` (avrdx/timer.hpp) and its `brio::bind`
trampoline were REMOVED on 2026-08-13 together with their last user (the
pre-kernel `console` app): kernel time events are the one way to wait.

## Toolchain std gotcha

The platform's `_bare.py` appends `-std=gnu++11` AFTER the flags added by
extra_scripts `pre:` scripts, so it would override the `-std=gnu++23` from
tools/pio_flags.py (the last -std on the command line wins). This is why
`build_unflags` also lists `-std=gnu++11`. Symptom if it regresses:
"'concept' only available with '-std=c++20'" errors from stream.hpp.
