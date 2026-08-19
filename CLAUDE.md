# CLAUDE.md

Guidance for Claude Code when working in this repository. This file is
the assistant's operating manual: how to interact, where the truth
lives, what is settled, what is open. It does NOT duplicate the design
docs - it points at them.

## Interaction

Claude interacts with the user in Italian; every edit in the files -
code, comments, docs, commit messages - is in English.

## Allowed text symbols

Only ASCII <= 127 in every file of the repo (code, docs, this file).

## Where the truth lives

- `README.md` - brio's shopfront: what it is, an application snippet,
  the ideas, layering and the target table. No history, no apps.
- `docs/design/*.md` - the target-independent design, by intent: WHY
  and contracts (`architecture.svg` = the strata diagram, hand-written
  SVG). `overview.md` (philosophy, governing rule, layering, style),
  `kernel.md` (the AO kernel: model, contract, events, payloads,
  queues, FSM, delivery, scheduler, time, panic, platform, index),
  `clock.md` (the clock model), `serial.md`, `spi-bus.md`, `i2c-bus.md`,
  `ring.md`, `analog.md` (the sampler usage type + arithmetic).
- `docs/<target>/` - one folder per target, mirroring
  `lib/brio/src/<target>/` (`avrdx/`, `host/`): `README.md` is the
  operational page (toolchain, board, probe, debugger and their
  quirks); next to it ONE document per peripheral in the shape
  docs/README.md prescribes (documents of record -> what the silicon
  does -> types and verbs -> how to use it, one example per use ->
  bench findings -> for provisional ones, "Not covered yet"), each
  saying EXHAUSTIVE (systematic review + bench suite: clkctrl.md,
  evsys.md, vref.md, dac.md, adc.md) or PROVISIONAL (port.md,
  usart.md, spi.md, twi.md, rtc.md, tca.md) - the state of the driver
  work, readable in the docs map. The Multislope assessment (every
  acrobatic piece maps to fixed routes + tasks on resources + config
  structs; the 64-cycle snapshot stays in the ISR body) lives in
  memory and in the track entry below, not in docs.
- `docs/bench.md` - the board, the wiring and the apps as they are
  today. The volatile end.
- `docs/avrdx/vendor/README.md` - the datasheets/errata by document
  number and the chapters we use; PDFs are local symlinks
  (git-ignored), cite by SECTION as "DS40002247B 16.5.2" (pages move
  between revisions). Documents of record: datasheet DS40002247B
  (2023) and errata DS80000915F (2025), fetched into
  docs/avrdx/vendor/ (git-ignored); the copies in
  ~/Documenti/Elettronica/AVR/ are the older rev. A of both (and the
  file named AVR64DB...Errata there IS the AVR128DB errata). Check
  revisions before trusting a local PDF.
- Headers - the canonical API reference; header comments explain the
  concurrency model and the WHY of each tradeoff.

Rules (full text in `docs/README.md`): any change that alters a
documented decision updates the matching doc in the same change; docs
say today's truth only (no change history, no dates, no renames);
design/ and the target folders never reference individual apps; never duplicate
signatures into docs; new decisions go into `docs/design/`, not here.
This file has no decision log any more: the former log was migrated to
`docs/design/` and `docs/design/*` is authoritative.

## The project in one paragraph

`brio` (`lib/brio/`) is a header-only C++23 (gnu++23) framework for
bare-metal MCUs built around a cooperative active-object kernel,
written clean-room after Samek's book (never the QP source). One flat
namespace `brio`; four strata under `lib/brio/src/` - `kernel/` (pure
logic, includes nothing of brio), `util/` (services over the kernel),
`avrdx/` (everything that knows `avr/io.h`, the only target today:
AVR DA/DB, bench chip AVR128DB48), `host/` (the native test target).
Includes carry the stratum prefix (`#include "avrdx/uart.hpp"`). The
repo is a PlatformIO project: one `main()` per `src/apps/<app>.cpp`
becomes envs `<app>` (release) and `<app>-debug`; host tests in
`test/` run on `[env:native]`.

## Governing rule and stability hierarchy

**Nothing is settled.** Reusing existing code is fine only while it
does not limit the design above it; when a limitation can be overcome
by rewriting what sits below, the rewrite wins. Every stage of work
requires critical analysis at ALL levels of the stack, not just the
one being added.

Within that, three levels of stability guide how much a change should
disturb:

- **kernel ideas** - fairly stable: the AO contract (`Event`, `queue`,
  `init`, `dispatch`), value events per-AO variant, the two loans
  (`Lease::dispatch` / `Lease::reply`), post/publish/reply, priority =
  pack order, timers post events, panic breadcrumb, Platform concept;
- **util/ services and target drivers** - important, here to stay,
  but expected to change (possibly radically) when the second real
  target arrives;
- **apps** - incidental test tools; they will not survive in their
  current form. Nothing in the foundations or their docs may depend on
  an app; apps document themselves in their own header comment.

## Standing style rulings

Types PascalCase, functions/constants snake_case; private members
trailing underscore; no `Ao` suffix on AO class names; queues speak
push/pop; `std::optional` returns instead of bool + out-param; no
`*_from_isr` API doubling; no redundant `inline` on in-class
definitions; concepts instead of virtual interfaces; use the
freestanding libstdc++ (variant, optional, span, concepts, bit,
expected...) instead of hand-rolled traits; C++26 features already in
gcc 16 may be used when genuinely needed (bump the -std flag then).
Handlers dispatch with `brio::match(e, lambdas...)`. Drivers expose
ISR handler BODIES (`[[gnu::always_inline]]`), the app binds the
vector - vector names never appear in portable code. No `#ifdef` where
a template/concept boundary can do the job; no target includes outside
the target strata; the kernel must never know which silicon it runs
on. Apps never touch registers (PORTx/VPORTx/peripheral structs live
only in target drivers; the ISR vector binding is the one vendor glue an
app may contain). Full text: `docs/design/overview.md`.

## Open items and horizon

Roughly ordered by proximity. None of these is a decision yet; each
gets its dated home in `docs/design/` when taken.

- **Borrowed, phase 2 (debug epoch).** `Borrowed<T, Lease::dispatch>`
  is a plain pointer today. Planned: in debug builds an 8-bit lender
  epoch travels with the loan (on 8-bit targets it costs one byte and
  no padding; on 32-bit ones it sits in the pointer's padding for
  free) and is compared on every access - a stale loan panics on the
  guilty instruction. To be built together with the first host test
  that simulates preemption; not before.
- **Payload/ownership pass.** kernel.md section 4 states the rule in
  three lines; a dedicated pass over util/ producers (naming the lease
  at every field, spans of `reply` loans) is pending.
- **HSM.** The FSM contract is HSM-ready (`unhandled` = future
  bubble-to-parent); parent pointers, bubbling and LCA entry/exit
  chains get built only when a real AO demands them.
- **Second target.** Candidates STM32G0x0/x1, ATSAMC/D, CH32V00x
  (SysTick timebases at 1000 Hz). It will exercise the Platform
  concept, the per-target ticker/driver strata and the layering rule
  for real; expect radical revision of util/ and drivers then.
- **QK-style preemption (far horizon, probably not on AVR).** A
  preemptive non-blocking kernel (higher-priority AO preempts a
  lower one mid-dispatch, single stack, LIFO nesting) would be an
  EXTENSION of the present kernel, not a rewrite of what sits above
  it. What already holds: the AO contract and the three delivery
  primitives are unchanged; `Lease::dispatch` loans stay correct
  because the borrower-precedes-lender order (static_asserted by
  Kernel) makes the borrower preempt the lender right at the post;
  `Lease::reply` loans are ordering-independent. What would change:
  `post()` must trigger the scheduler when it readies a higher AO;
  the Platform gains an ISR-exit / pending-scheduler hook (PendSV-
  like: irreducibly target-specific); the loop becomes the idle-
  priority context; time-event maturation in the loop (T2) must be
  revisited (loop is idle priority then - candidates: tick ISR, or a
  top-priority pseudo-AO woken by the tick). Discipline to keep NOW
  so the door stays open: AOs share nothing but events (an AO's own
  statics are safe; a global touched by two AOs outside events is a
  one-way race under preemption).
- **Exhaustive-driver track (started 2026-08-19).** Done: EVSYS, VREF/
  DAC/ADC, CLKCTRL (clock.hpp rewritten as resources + tasks; `test_avr_clock`
  14/14 on the scope via CLKOUT/PA7: tune curve asymmetric, CFD
  fallback really 4 MHz, status follows the request - in clkctrl.md).
  TCA/TCB: chapters reviewed, tca.hpp (Tca resource + TcaPwm/TcaPwm16/
  FrequencyGenerator/Heartbeat/EventCounter) and tcb.hpp (Tcb resource +
  PeriodicTick/Timeout/OneShotPulse/PulseCounter/CascadedCounter/
  meters/Pwm8) written, `test_avr_timer` (9 tests, closed loop through
  EvPin generators, no wires) written and compiled, NOT yet run on the
  bench (hardware away until the user is back) - docs tca.md/tcb.md
  stay PROVISIONAL until its first green run. CCL/AC: chapters
  reviewed (ccl.md, ac.md), drivers next. TCD deferred. Multislope
  parked until the hardware is back. Remaining provisional: PORT,
  USART, SPI, TWI, RTC, TCA, TCB, CCL, AC - each doc's "Not covered yet"
  is the shopping list. Original order:
  EVSYS (docs/avrdx/evsys.md; avrdx/evsys.hpp primitives built and
  `events0` VERIFIED on the scope 2026-08-19: 512 Hz PIT/64 on PD2,
  button level, off, 4 Hz LED with no CPU; EventSystem static sugar
  when an app has several fixed routes) -> VREF, DAC,
  ADC exhaustively (docs/avrdx/adc.md; vref.hpp, dac.hpp, adc.hpp
  and `test_avr_analog`, the bench test SUITE (54/54 on rev A5 at
  3.3 V; measures VDD at start, run it at 5 V too) - a suite named
  test_<target>_<subject> is a reference test to keep passing through
  every restructuring; docs vref.md/dac.md/adc.md; 68/68 at 5 V too;
  `sampler` = the ADC inside the kernel via util/analog_sampler.hpp,
  bench-verified 512 samples/s no drops) -> TCB/TCA (done as above) ->
  CCL/AC tasks (LUT flip-flops; EventSystem static sugar gets its first
  user there) -> the Multislope app (the 64-cycle snapshot stays in the
  ISR body). Datasheet DS40002247B chapters: EVSYS 16, PORTMUX 17,
  VREF 21, TCA 23, TCB 24, TCD 25, CCL 31, AC 32, ADC 33, DAC 34
  (errata F: ADC, DAC, CCL 2.4, TCA 2.12, TCB 2.13, TCD 2.14 items).
- **Target strata, positions taken (overview.md "Target strata" and
  the diagram docs/design/architecture.svg).** Tasks over resources
  (thin handles + task types named for what they do; explicit handle
  stratum on the second task per peripheral); PwmChannel concept +
  generic actuators (done); Clock as a type (done, both regimes);
  interrupts condense / DMA inside the engines (not on AVR); event
  system = typed vocabulary + runtime connect/disconnect primitives +
  optional static allocation (not built, tables on demand); per-family
  device tables and per-board claim files on the second target.
  Nothing of this is a HAL.
- **Clock: dynamic regime - built and bench-verified (24->12->2 MHz
  under the running console).**
  `DynamicClock<Boot, Users...>` (avrdx/clock.hpp): set<hz>()/set(hz)
  (the app speaks Hz, the prescaler is resolved by div_for) fan
  rebase(hz) out to the users, then switch; Uart/Twi/Spi have
  rebase(); the users list is constrained by the ClockUser concept
  (has rebase) and a driver init(clock)'d with a DynamicClock that does
  not list it is a compile error (clock_follows) - both directions
  checked; delay_us takes the runtime path; `clock_hz(clock)` reads
  either kind. Coordination with bus AOs (switch only when idle) is
  the caller's job - a power-manager AO with request/reply is the
  designed shape, not built. `clock_console` is the bench test
  (CLOCK 4M / 2M / 24M at 115200). F_CPU is not defined in this build
  (see below).
- **Design rule for all AVR work: think the other targets first.**
  Before adding/changing anything in avrdx/, ask what shape it takes
  on Cortex-M0+ (rich clock tree, hardware cycle counters, per-pin
  AF), RISC-V (mcycle, group remap): that decides what crosses the
  concept boundary. Written in overview.md "Target strata".
- **C++ modules: considered, not now.** The real prize would be macro
  isolation (`import brio.avrdx` would not leak `avr/io.h` macros
  above the target stratum - the layering rule made mechanical), not
  build speed. Blockers: the language server (PlatformIO's IntelliSense
  does not follow modules; clangd only partially) and SCons has no
  module dependency scanning. Not a cure for the ISR glue either: the
  vector bindings are configuration (which USART, which route, which
  sink) and would live in a per-board unit under modules exactly as
  under headers. Revisit with the second target / board files.
- **Housekeeping.** No LICENSE file yet (repo is public at
  github.com/uliano/brio).

## Build, test, debug (the must-knows)

```bash
pio test -e native            # host tests (doctest); no hardware needed
pio run -e <app>              # release build (-Os)
pio run -e <app> -t upload    # flash via Atmel-ICE (UPDI)
pio debug -e <app>-debug      # ALWAYS debug the -debug env
python tools/gen_apps.py      # after adding/removing src/apps/*.cpp (or a "// pio: opt = value" line)
```

- Toolchain: self-built avr-gcc 16.2 at `/sw/avr` via `symlink://`;
  never PlatformIO's bundled one. Never add `-mrelax` (PyAvrOCD
  refuses the ELF). `build_unflags` must keep `-std=gnu++11` (the
  platform appends it after our `-std=gnu++23`), `-flto`, and
  `-DF_CPU` (by name: the clock rate has one truth, `Clock::hz`;
  avr-libc's util/delay.h / setbaud.h do not compile here, on purpose -
  use `brio::delay_us(clock, us)`).
- Atmel-ICE: cable in the **AVR** port, not SAM (symptom: Vtarget
  ~1.71 V, sign-on `0xa0`).
- Debugging: PyAvrOCD as GDB server (`debug_tool = custom`, port
  40044); effectively ONE free hardware breakpoint (GDB borrows the
  second); `--breakpoints hardware` refuses extras instead of wearing
  flash; line breakpoints need `-fno-inline` (GCC 16 DWARF caveat) -
  hence the debug flags in platformio.ini. `monitor ioregister <name>`
  reads/writes peripheral registers.
- The bench board: 24 MHz crystal on PA0/PA1 (not GPIO) -
  `Clock<ClockSource::crystal, 24'000'000>` - no 32k crystal (do not
  enable XOSC32K), serial on USART2 ALT1 PF4/PF5.
- Full detail and rationale: `docs/avrdx/README.md`,
  `docs/host/README.md`, `docs/bench.md`.

## Layout

```
platformio.ini          base [env], toolchain, Atmel-ICE upload, debug wiring
apps.ini                generated: [env:<app>] + [env:<app>-debug] per app
boards/AVR128DB48.json   custom bare-metal board (128K flash / 16K RAM)
tools/gen_apps.py        scans src/apps/*.cpp -> apps.ini; copies each app's
                         "// pio: <option> = <value>" header lines into its envs
tools/pio_flags.py       per-language AVR flags (build-type aware) +
                         IntelliSense include paths (skips [env:native])
tools/gen_lst.py         post-build: firmware.lst (disassembly) + firmware.map
src/apps/<app>.cpp       one main() per app (ISR vector bindings live HERE)
test/test_*/main.cpp     host unit tests (doctest), pio test -e native
docs/                    README (map + rules), design/, <target>/ (avrdx/, host/), bench.md
lib/brio/src/            the framework, four strata:
  kernel/                pure kernel logic - includes NOTHING of brio
    platform.hpp           Platform concept (CriticalSection, idle,
                           break_here, now, ticks_per_second, atomic_width,
                           panic_record) + PanicRecord (hosted by the platform)
    active_object.hpp      ActiveObject concept: what Kernel requires of an
                           AO (Event, queue, init, dispatch) + the informal
                           half of the contract
    event_queue.hpp        EventQueue<E, depth, P>: per-AO MPSC queue,
                           saturating overflow counter, optional pop
    fsm.hpp                Fsm<Derived, Alts...> flat HSM-ready machines
                           (Entry/Exit reserved, transition chaining, start);
                           match(e, lambdas...) + Overloaded
    post.hpp               post<Ao>(ev), publish(Subscribers<...>{}, ev),
                           ReplyTo<Payload> / reply_to<Ao, Payload>()
    borrowed.hpp           Lease {dispatch, reply}, Borrowed<T, Lease>:
                           pointer payloads with the lease in the type
    time.hpp               constexpr tick conversions (ceil, never early)
    time_event.hpp         TimeEvents<P> armed list + TimeEvent<P, Ao, Ev>
                           (drift-free periodics, wrap-safe, RAII disarm)
    kernel.hpp             Pack<Aos...> (index, lends_ok) + Kernel<P, Aos...>:
                           init_all/step/idle_if_empty/run, static_asserts
                           borrowers before lenders
    panic.hpp              panic<P, Reporter>(), PanicCode, HaltReporter,
                           take_panic_record<P>()
  util/                  pure services - may include kernel/, never a target
    stream.hpp             ByteSink / ByteSource / ByteTransport concepts
    print.hpp              print(sink, ...) + hex/fixed/sci wrappers, crlf;
                           extend via print_one + ADL
    timestamp.hpp          TimeStamp (ms fraction)
    wire.hpp               constexpr big-endian load/store (16/24/32, be24s)
    analog.hpp             adc_mv/adc_mv_signed, dac_code/dac_mv: pure counts<->mV
                           arithmetic (host-tested); Ref/ref_mv are each target's
    analog_sampler.hpp     AnalogSampler<Converter, P, Subscribers, inputs...>: the
                           owner AO walking a list of inputs, Sampled in (ISR glue,
                           labelled by the converter's selected code), AnalogSample
                           published; software pace or any hardware generator
    clock.hpp              ClockUser concept, clock_hz(clock), clock_follows:
                           the target-independent clock contracts
    pwm_channel.hpp        PwmChannel concept: max + duty(v), the role-level
                           "one dimmable output" (Pin satisfies it, max 1)
    rgb_lamp.hpp           RgbLamp<R, G, B> over three PwmChannels, levels
                           scaled per channel max; Rgb triple
    ring.hpp               Ring<T, size, P> SPSC FIFO, lock-free when the
                           index fits P::atomic_width, guarded otherwise
    serial_port.hpp        SerialPort<Transport, P, LineSink>: RX bytes ->
                           LineReceived (Lease::dispatch loan, LendsTo)
    bus_master.hpp         BusMaster<Bus, P>: bus arbiter (pending FIFO,
                           reject-when-full, ReplyTo completion, BusDone)
    spi_bus.hpp            SPI vocabulary: SpiBus/SpiDone/spi_*
    i2c_bus.hpp            I2C vocabulary: I2cBus/I2cDone/i2c_* + outcomes
    proto/line_parser.hpp  LineAssembler + console/SCPI parsers +
                           CommandRouter<Sink>
  avrdx/                 everything that knows avr/io.h (AVR DA/DB)
    platform_avr.hpp       AvrPlatform
    clock.hpp              CLKCTRL: resources Oschf/Osc32k/Xosc32k/Xoschf/Pll/
                           MainClock/ClockFailure (typed register views) +
                           tasks Clock<source, hz, div> (constexpr hz, the ONE
                           rate truth: no F_CPU) and DynamicClock<Boot, Users...>
                           (set<hz>()/set(hz) rebases the users then switches)
    delay.hpp              delay_us(clock, us) "at least": folded cycles when
                           constant, 4-cycle loop otherwise; cycles_per_us
    pin.hpp                Pin<'A',5> compile-time GPIO (also a PwmChannel,
                           max 1) + PinRef descriptor + PinSet<Pins...> mask
    uart.hpp               Uart<n, Route, rx, tx> interrupt-driven transport
    spi.hpp                Spi<n> master engine (two-phase descriptor,
                           per-byte ISR pump, CS owned by the engine)
    twi.hpp                Twi<n, Route> I2C master engine
    tca.hpp                TCA: Tca<n> resource (normal mode: PER, three
                           buffered CMP, waveform modes, event inputs A/B,
                           commands, ISR bodies) + tasks TcaPwm<n, port>
                           (split mode, six 8-bit PwmChannels), TcaPwm16,
                           FrequencyGenerator, Heartbeat, EventCounter
    tcb.hpp                TCB: Tcb<n> resource (eight modes, event clock/
                           capture, cascade, routes) + tasks PeriodicTick,
                           Timeout, OneShotPulse, PulseCounter,
                           CascadedCounter, Frequency/PulseWidth/DutyMeter,
                           Pwm8
    ticker.hpp             BasicTicker<tps> RTC/PIT timebase (Ticker = 1024)
    vref.hpp               Ref + ref_mv (this silicon's levels) + Vref::adc0/dac0/ac
    dac.hpp                Dac<0>: init(DacConfig), set(code)/set_mv - actuator
    adc.hpp                Adc<0>: init<cfg>()/init(cfg)/reconfigure, AnalogIn<Pin>
                           + AdcInput, select/start/stop/read/result, window,
                           resrdy()/wcmp() ISR bodies, start_on(channel),
                           ClockUser (rebase keeps CLK_ADC in range)
    evsys.hpp              EVSYS: EventChannel<n> (source/off/pulse), generators
                           EvPitDiv/EvRtcOvf/EvRtcCmp/EvPin/EvTcaOvf/EvTcaCmp/
                           EvTcbCapt/EvTcbOvf (code + legality), users EvOut<Pin>/
                           EvAdc0Start/EvTcaCntA/B/EvTcbCaptIn/CountIn +
                           EventUserBase (listen/unlisten); concepts
                           EventGenerator/EventUser; tables on demand
  host/                  the test target
    platform_host.hpp      HostPlatform (virtual clock, recording idle/break)
```

## Build artifacts

`.pio/build/<env>/`: ELF / HEX / MAP / `firmware.lst` (source-
interleaved disassembly from tools/gen_lst.py).
