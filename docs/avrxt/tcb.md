# TCB - the 16-bit timer/counter type B (AVR DA, DB and EA)

This document is for a programmer who needs to measure, count or time
one signal: the period, the width or the duty of a pulse train, a count
of edges (16 bits on one TCB, 32 on two), a single pulse of a set width
started by an event, an interrupt at a rate of its own, or a watch that
fires when the next edge comes too late. `Tcb<n>` is the resource, the
typed view of one instance, one function per register fact. Over it sit
the tasks (a task, in brio, is a driver layer with state of its own
over a resource, served from its peripheral's interrupt or called by an
active object; an active object is a component of a brio program with
its own event queue, served one event at a time by the kernel,
[learn/introduction.md](../learn/introduction.md)): `PeriodicTick`,
`Timeout`, `OneShotPulse`, `PulseCounter`, `CascadedCounter`, the three
meters `FrequencyMeter`, `PulseWidthMeter` and `DutyMeter`, and `Pwm8`,
one 8-bit PWM channel. Each takes the application's units (a clock tag
and microseconds or hertz) at `init()` and hands ticks to the interrupt
handler, and each is a monostate (a class whose state is static, with
no instances): the program names the type and calls its functions. The
signals reach a TCB through the event system ([evsys.md](evsys.md)): a
pin's level is an event, so a meter needs no wire beyond the pin
itself.

The smallest use, a frequency meter on a pin, from "How to use it"
below:

```cpp
#include "avrxt/evsys.hpp"
#include "avrxt/tcb.hpp"
using Meter = brio::FrequencyMeter<brio::Tcb<0>>;
using In = brio::EventChannel<2>;               // PORTC/PORTD pins: channels 2-3

In::source(brio::EvPin<brio::Pin<'D', 0>>{});
Meter::init(clock, In{});                       // FRQ mode, CLK_PER, CAPT interrupt on
ISR(TCB0_INT_vect) { post<Display>(Period{Meter::period_ticks()}); }   // the read clears CAPT
```

Line by line: the two includes bring in the event system and the
driver; `Meter` names the frequency meter over TCB0; `In` names event
channel 2; `In::source(...)` makes the level of the pin PD0 the
channel's event; `Meter::init(clock, In{})` takes the clock task's tag
(`constexpr SysClock clock;`, [clkctrl.md](clkctrl.md)), puts TCB0 in
FRQ mode clocked by CLK_PER, connects its capture input to channel 2
and enables the CAPT interrupt, so the counter restarts at every rising
edge of PD0 and its count at that edge is the period in CLK_PER ticks;
the handler, bound to TCB0's one vector with `ISR()`, reads the period
with `period_ticks()` and posts it to an active object (`brio::post`,
`Display` and `Period` standing for the program's own active object and
event type), where `Meter::hz(ticks)` converts it to hertz.

What decides correctness even there:

- **The clock task first.** `init()` reads the rate from the clock tag,
  and `hz()`, `us()` and `tick_hz()` convert with that number: a meter
  initialized before `SysClock::init()` converts with the rate the clock
  type claims while the counter runs at another. A `DynamicClock` that
  does not list the meter is a compile error at `init()`.
- **The vector, and the read inside it.** `init()` enables the CAPT
  interrupt, so the program binds `TCBn_INT_vect` (one vector per
  instance, CAPT and OVF share it); avr-libc's start-up code sends an
  unbound vector to the reset address, and the program restarts with
  CPUINT's LVL0EX still set, deaf to every level-0 interrupt until a
  real reset ([platform.md](platform.md)). The
  handler must read the capture: that read is what clears CAPT, and a
  handler that leaves it standing is entered again as soon as it
  returns.
- **The channel's legality is checked at compile time.** On the AVR DA
  and DB a pin's event reaches only the two channels of its port pair
  (PORTC and PORTD on channels 2 and 3), and `source()` refuses any
  other; on the AVR EA every pin event goes on any channel. The pin
  itself is read as it stands: nothing here configures PD0, an input out
  of reset.
- **The capture register goes through TEMP.** `period_ticks()` in the
  handler and a read of the same TCB's CNT or CCMP in the main context
  share the instance's one TEMP register, so such a main-context read
  belongs in a critical section ("What the silicon does").
- **The first capture after arming may be false.** A capture input
  armed while the channel is already high detects a rising edge that
  never happened ("What the silicon does"): discard the first period, or
  arm while the pin is low.
- **The range is 65535 ticks.** A signal slower than that (366 Hz at
  24 MHz CLK_PER) wraps the counter, and `overflowed()` says so.

The rest of this page, in order: the documents of record; what the
silicon does (the eight modes, the facts that matter to code, the
events, what the TCB is not); the types and verbs; how to use it, four
examples; the bench findings on the AVR64EA48, then the AVR128DB48 and
the AVR128DA48; and what is not covered.

## Documents of record

AVR128DB28/32/48/64 data sheet DS40002247B (TCB
chapter 24, PORTMUX 17.3.8, EVSYS 16 generators 0xA0-0xA9 / users
0x1E-0x27), errata DS80000915F (2.13.1, 2.13.2) and, for the DA parts,
DS80000882C (2.12.1); for the AVR EA, AVR64EA28/32/48 data sheet
DS40002443A (TCB chapter 23 - the same IP, the same CLKSEL, CNTMODE,
CASCADE, SYNCUPD, ASYNC and FILTER - PORTMUX 17.3.8) and errata
DS80001048D (no TCB item). Complements: TB3214
"Getting Started with TCB" and the Microchip examples
`avr128da48-getting-started-with-tcb`,
`avr128da48-tcb-frequency-dutycycle-measurement` (see
[vendor/README.md](vendor/README.md)). Driver:
[`avrxt/tcb.hpp`](../../brio/avrxt/tcb.hpp) (the `Tcb<n>` resource and
the tasks), the TCB event vocabulary in `avrxt/evsys.hpp`. Reference
test: `test_avr_timer` (a letter is one command key of the suite's
console and the group of verdicts it runs; this suite's `a` runs every
letter its image carries, and on the AVR64EA48 the suite is two images,
[README.md](README.md), "The group axis").

## What the silicon does

Five instances on the family, TCB0..TCB3 on the 48-pin parts (TCB4
only on 64 pins), TCB0..TCB3 on every AVR EA package (TCB3's default
PB5 and TCB2's ALT1 PB4 bonded on its 48-pin part only: `has_default_
pin` / `has_alt_pin` say so, the driver unchanged). Each is a 16-bit counter with one capture/compare
register (`CCMP`), one waveform output (`WO`), two flags (`CAPT`,
`OVF`) that are interrupt sources and event generators alike, and a
mode field that turns the same counter into eight different tools:

| Mode (`CNTMODE`) | The counter | `CCMP` is | `CAPT` fires | Needs the event input |
|------------------|-------------|-----------|--------------|-----------------------|
| `INT` periodic interrupt | counts 0..TOP, restarts | TOP | at TOP | no |
| `TIMEOUT` time-out check | starts on one edge, stops on the next (freeze) | TOP | if TOP is reached before the stop edge | yes |
| `CAPT` capture on event | free-running 0..MAX | capture of CNT | at each selected edge | yes |
| `FRQ` frequency | restarts on each selected edge | capture = period | at each edge | yes |
| `PW` pulse width | restarts on one edge, captures on the other | capture = width | at the capturing edge | yes |
| `FRQPW` frequency + width | starts on edge 1, captures at edge 2, stops at edge 3 | width; period in CNT | at edge 3 (frozen until CCMP is read) | yes |
| `SINGLE` single-shot | starts on an edge, counts 0..TOP, stops | pulse length | at TOP | yes |
| `PWM8` 8-bit PWM | counts 0..CCMPL, restarts | CCMPL = period - 1, CCMPH = high time | at CCMPH | no |

Facts that matter to code:

- **Clock (`CLKSEL`)**: CLK_PER, CLK_PER/2, the prescaled clock of
  TCA0 or TCA1 (`CLK_TCA`, so a TCB runs in step with a TCA - and
  `SYNCUPD` restarts it when that TCA restarts/overflows), or the
  positive edge of an event (`COUNT` user, 0x1F/0x21/0x23/0x25): the
  TCB as an **event counter**. When the count source is the event,
  the capture edge must come from the other user (`CAPT`).
- **Event input (`CAPT` user)** enabled by `CAPTEI`; `EDGE` selects
  which edge does what (the table in 24.5.3 is mode-dependent:
  TIMEOUT start/stop, CAPT/FRQ which edge captures, PW/FRQPW which
  edge restarts, SINGLE positive-only or any edge). The event must
  last at least one CLK_PER (in PW the datasheet asks a minimum edge
  separation of two clock cycles); a `FILTER` (four equal samples at
  CLK_PER) is available - measured: it adds 3 CLK_PER over the bare
  synchronous path (the fourth sample decides, three cycles after
  the first; the datasheet's four count from the event). In SINGLE mode `ASYNC`
  drives WO high on the event itself (before the counter starts,
  2-3 CLK_TCB later) and the event may be shorter than one CLK_PER.
- **TIMEOUT mode fires at TOP ticks and re-fires at every wrap**
  (bench, A5): CAPT fires when CNT reaches TOP - exactly TOP tick
  periods after the synchronized start edge (measured: TOP + a
  constant 2-tick event-path offset, identical to single-shot's) -
  and if the stop edge has not arrived the counter free-runs past
  TOP, wraps, and CAPT fires again at every pass through TOP (every
  65536 ticks): a watch on a long-silent signal keeps shouting, once
  per wrap.
- **Arming a synchronous CAPT-in on a channel already high reads a
  spurious rising edge** (bench, A5): the user's edge detector starts
  from zero, so enabling CAPTEI while the event level is 1 detects an
  edge that never happened on the wire. Arm while the channel is low
  (or discard the first capture).
- **Reading a capture clears `CAPT`** (on the low byte of CCMP):
  read CCMP in the ISR and the flag is gone; in FRQPW read CNT
  (period) before CCMP (width), since reading CCMP re-arms the
  sequence. The 16-bit registers go through `TEMP` (low byte first,
  on read and on write - 8.10.3) - the compiler's 16-bit access does
  this, but an ISR that reads CNT and CCMP shares the one TEMP with
  the main context: those reads belong in a critical section or in
  the ISR only.
- **Single-shot**: WO is low when stopped, high while counting; a
  new event during the pulse is ignored; after TOP the output stays
  low at least one CLK_TCB and a new event is honoured two CLK_PER
  later. Enabling the peripheral (or changing `EDGE` while enabled)
  starts a spurious pulse - writing TOP to CNT before enabling
  prevents it. `STATUS.RUN` tells whether the pulse is in progress.
- **Output**: `CCMPEN` makes WO override PORT.OUT (pin must be an
  output); `CCMPINIT` is the idle level in the modes that do not
  define one (not PWM8, not SINGLE). Routes (PORTMUX `TCBROUTEA`, one
  bit per instance): TCB0 PA2 / PF4, TCB1 PA3 / PF5, TCB2 PC0 / PB4,
  TCB3 PB5 / PC1, TCB4 PG3 / PC6 (the TCB4 ALT1 is dead on every
  silicon revision, errata DB 2.13.2). TCB2's default PC0 collides with
  TCA0's PORTC WO0 route; which of the two a board uses is the
  application's to decide, pad by pad.
- **32-bit capture**: two TCBs, the LSB one clocked from the source
  with `CASCADE = 0`, the MSB one clocked from the LSB's `OVF` event
  (`COUNT` user) with `CASCADE = 1` (delays its CAPT input by one
  CLK_PER to absorb the carry), both in the same capture mode, the
  same CAPT event routed to both; the value is MSB.CCMP:LSB.CCMP.
  Any pair works (the datasheet example is TCB0/TCB1); it costs two
  event channels.
- **Do not change mode while enabled** (unpredictable output); a flag
  may set during configuration - clear INTFLAGS after configuring.
  Periodic interrupt with TOP lowered below CNT: the counter goes on
  to MAX, OVF fires, restarts - an OVF ISR is the safety net when TOP
  is changed on the fly.
- `RUNSTDBY` keeps it counting in standby, and that one bit is the
  whole chain: the request it makes keeps CLK_PER - and the oscillator
  behind it - running through the sleep, whatever the oscillator's own
  `RUNSTDBY` says, and a TCB interrupt then wakes the CPU by itself.
  Without it the counter freezes however the clock is configured
  (measured both ways: [platform.md](platform.md)). `DBGRUN` keeps it
  running under a debugger halt (default: halted and blind to events -
  a single-shot under PyAvrOCD stops with the CPU).
- **Errata DB 2.13.1 (A4/A5) / DA 2.12.1 (every revision)**: in PWM8
  the CCMP and CNT bytes are not independent - write CCMP as one 16-bit
  value (period and duty together), never byte-wise; the driver does,
  on both families. The AVR EA has no such item, and its chapter's
  CCMPL and CCMPH are "two independent registers" in PWM8 (DS40002443A
  23.5.10) - measured so on rev B1 - so there `Pwm8::duty` writes CCMPH
  alone (the device tables' `tcb_pwm8_bytes`): one store instead of
  two, and no pass through the instance's TEMP register, which an ISR
  reading CNT or CCMP of the same TCB would share.

As event generator: `TCBn CAPT` (0xA0 + 2n) and `TCBn OVF` (0xA1 +
2n), one CLK_PER pulses, same conditions as the flags. As event user:
`TCBn CAPT` (0x1E + 2n, edge; synchronous - asynchronous too only in
single-shot, 24.3.4) and `TCBn COUNT` (0x1F + 2n, sync).

What the TCB is not: there is one compare register, so no PWM with an
independent period and duty beyond 8 bits, no dual-slope, no
direction control, no period buffering (writing CCMP takes effect at
once) - that is TCA's job ([tca.md](tca.md)). TCB is the precise
one-channel tool: measure, count, pulse, tick.

## Types and verbs

The resource, `Tcb<n>` (n up to the package's instances: 2 on
28/32-pin, 3 on 48, 4 on 64 - the device header decides, one past the
end is a compile error), and the tasks over it. A pin POSITION the
package does not bond (TCB2's ALT1 is PB4, absent on 28/32-pin) does
not disable the instance: `has_default_pin`/`has_alt_pin` say what is
bonded, the compile-time `init<cfg>` refuses a config asking for a
missing position, the run-time `init(cfg)` returns false for it
(programming nothing) and true otherwise.
All static (monostate); a task owns its instance.

| Entity | Verbs |
|--------|-------|
| `TcbConfig` | `mode` (`TcbMode`: periodic, timeout, capture, frequency, pulse_width, frequency_pulse_width, single_shot, pwm8), `clock` (`TcbClock`: div1, div2, tca0, tca1, event), `compare` (CCMP: TOP / pulse width / PWM word), `event_input` (CAPTEI), `edge` (EDGE, meaning per mode), `filter`, `async`, `cascade`, `sync_update`, `output` (CCMPEN, the pin driven as output), `output_init_high`, `alt_pin` (PORTMUX), `run_standby`, `debug_run` |
| `Tcb<n>` | `init<cfg>()` / `init(cfg)` (disable, route, configure, park CNT, clear flags, enable); `enable`/`disable`/`enabled`; `count`/`count(v)`, `compare`/`compare(v)`, `capture` (CCMP read: clears CAPT), `running` (STATUS.RUN); `capt_flag`/`ovf_flag`/`clear_capt`/`clear_ovf`; `enable_capt_interrupt`/`enable_ovf_interrupt`; `take_flags` (the ISR body of the one vector: both flags, cleared); `capture_on(channel)` (CAPT user + CAPTEI), `capture_on_events(on)`, `count_on(channel)` (COUNT user); `OutDefault`/`OutAlt` pin types; `CaptEvent`/`OvfEvent` generators, `CaptIn`/`CountIn` users |
| `PeriodicTick<Tcb>` | `init(clock, hz)`, `init_us(clock, us)` (div1 or div2 picked to fit, false if neither), `rebase` (ClockUser), `tick` (ISR body), `start`/`stop` |
| `Timeout<Tcb>` | `init(clock, us, channel, edge, filter)` (start on one edge, stop on the next, CAPT if us elapse first - TOP = ticks, never early), `rebase` (ClockUser), `expired` (ISR body), `pending` |
| `OneShotPulse<Tcb>` | `init(clock, width_us, trigger_channel, {any_edge, async, alt_pin, filter})` (this form is a ClockUser: `rebase` keeps the width), `init_ticks(clock, ticks, ...)` (tick domain, rebase leaves it alone), `width_ticks(t)`, `fire` (a software event on the trigger channel), `busy`, `pulse_done` (ISR body) |
| `PulseCounter<Tcb>` | `init(source_channel)` (clock = event, free-running capture mode), `snapshot_on(channel, falling, filter)`, `count`, `reset`, `overflowed`, `captured` (the latched value; ISR body) |
| `CascadedCounter<Lsb, Msb>` | `init(source_clock, carry_channel, snapshot_channel)`, `count_on(channel)`, `reset`, `read` (software snapshot + the two captures: 32 bits), `captured` (ISR body after an external snapshot event) |
| `FrequencyMeter<Tcb>` | `init(clock, source_channel, tcb_clock, falling, filter)`, `period_ticks` (ISR body), `hz(ticks)`, `us(ticks)`, `tick_hz`, `overflowed`, `rebase` (ClockUser: the conversions follow, the divisor stays) |
| `PulseWidthMeter<Tcb>` | `init(clock, source_channel, tcb_clock, low, filter)`, `width_ticks` (ISR body), the same helpers |
| `DutyMeter<Tcb>` | `init(clock, source_channel, tcb_clock, inverted, filter)`, `reading` (ISR body: period then width, in that order), `duty_permille(reading)`, the same helpers |
| `Pwm8<Tcb, period>` | `init(tcb_clock, alt_pin)`, `duty(v)` (PwmChannel, `max` = period; max = static high from PORT); ratiometric on purpose, NOT a ClockUser: the duty survives a clock change, only the frequency moves |
| helpers | `tcb_tick_hz(clk_per, clock)`, `tcb_ticks_for_us(tick_hz, us)`, `tcb_timing_for_us/hz(clk_per, x)` (the div1/div2 choice) |

The conversions speak the application's units (a clock, microseconds,
hertz) at init and hand ticks in the ISR: the meters' `hz()`/`us()` are
pure arithmetic over the tick rate fixed at init (0 when the TCB is
clocked by a TCA: then the caller knows the rate).

## How to use it

A frequency meter on a pin (the pin's level is the event, the TCB
captures the period between rising edges):

```cpp
#include "avrxt/evsys.hpp"
#include "avrxt/tcb.hpp"
using Meter = brio::FrequencyMeter<brio::Tcb<0>>;
using In = brio::EventChannel<2>;               // PORTC/PORTD pins: channels 2-3

In::source(brio::EvPin<brio::Pin<'D', 0>>{});
Meter::init(clock, In{});                       // FRQ mode, CLK_PER, CAPT interrupt on
ISR(TCB0_INT_vect) { post<Display>(Period{Meter::period_ticks()}); }   // the read clears CAPT
...  Meter::hz(ticks)                           // in the AO, pure arithmetic
```

A pulse of 100 us on PB5 whenever an event arrives (and by software):

```cpp
using Shot = brio::OneShotPulse<brio::Tcb<3>>;
using Trigger = brio::EventChannel<1>;
Shot::init(clock, 100, Trigger{});             // single-shot, WO on the default pin
Trigger::source(brio::EvPin<brio::Pin<'A', 4>>{});   // a button's rising edge fires it...
Shot::fire();                                   // ...and so does this
```

A 32-bit capture of CLK_PER (the datasheet's TCB0/TCB1 example):

```cpp
using Wide = brio::CascadedCounter<brio::Tcb<0>, brio::Tcb<1>>;
Wide::init(brio::TcbClock::div1, brio::EventChannel<4>{}, brio::EventChannel<5>{});
uint32_t t = Wide::read();                      // latched by a software snapshot event
```

The resource directly, for a shape no task has (here: TCB in time-out
mode clocked by TCA0's prescaled clock, restarting with it):

```cpp
using T = brio::Tcb<2>;
T::init<brio::TcbConfig{.mode = brio::TcbMode::timeout, .clock = brio::TcbClock::tca0,
                        .compare = 1000, .event_input = true, .sync_update = true}>();
T::capture_on(brio::EventChannel<3>{});
T::enable_capt_interrupt(true);
ISR(TCB2_INT_vect) { auto f = T::take_flags(); if (f.capt) ...; }
```

The ISR vector binding is the app's (one vector per instance, CAPT
and OVF share it).

## Bench findings

On the **AVR64EA48** (rev B1, 5 V, the board's 20 MHz quartz;
`test_avr_timer`, 208 verdicts, twenty runs of twenty; the TCB is the
instrument of every letter): every TCB verdict of the DA/DB holds with
its ticks derived from the rate - the meters exact to the tick (40000
.. 200 ticks a period, 14000 low of a 700 us low time), the capture's
interval-minus-one at `div1` corrected by the meters (and at `div2` the
same half tick: 9999 for 10000, as the DB's 11999), the one-shot widths
200 / 2000 / 20000 ticks and the async pulse 2004 for 2000 (inside the
letter's four ticks), Pwm8 at period 256, the cascade's second 20 M ticks
within 0.2 %, Timeout firing TOP + 2 after its start with the filter's
+3, and the CAPT gate. And the EA's own (letter x): **PWM8's CCMPH takes
a lone byte write with CCMPL kept** - written after a read of CNT that
left 0 in TEMP, the period stays 256 and the width follows (64, then
192) - which is what `Pwm8::duty` does there (above).

`test_avr_timer` (silicon rev A5, 24 MHz crystal, 3.3 V), all
measured against the same crystal - and the same suite green on the
AVR128DA48 (rev A7, its 24 MHz external clock, 5 V; 197 verdicts):

- **The capture is the interval minus one CLK_PER cycle** in FRQ, PW and
  FRQPW (24000 ticks read 23999; Pwm8 with CCMPL = 255 reads 255 for
  its 256-tick period): the restart cycle is not counted. Clocked by a
  TCA's prescaled clock (div64) the capture is exact (750 for 750) -
  the lost cycle is a fraction of a tick. The meters add one tick at
  `div1`, nothing otherwise (at `div2` the half tick rounds either
  way).
- **SYNCUPD** restarts the TCB at the clocking TCA's TOP: a capture
  aligned with that TCA's own edge (its FRQ output) reads 0 - it is
  for signals in phase with the TCA period, not for the TCA itself.
- One-shot widths of 10 / 100 / 1000 us measure exactly (240 / 2400 /
  24000 ticks); `ASYNC` adds four CLK_PER (2404 for 2400): the output
  rises on the event, the counter starts 2-3 CLK_TCB later, as
  documented. A second trigger during the pulse is ignored; `RUN` is
  set two CLK_PER after `fire()`.
- FRQPW (DutyMeter) at 1 kHz for 25/50/75 % duty: period 24000, width
  exact; PW agrees; static low/high never capture. Pwm8 duty words
  written as 16 bits (errata 2.13.1) give period 256 and width = duty.
- The 32-bit cascade (TCB1 + TCB2, two event channels) counts CLK_PER
  coherently through a software snapshot: 24017038 in a one-second
  `delay_us` hold (+0.07 %: the hold is stretched by the test's ISRs).
  The **Ticker's second** (RTC from the internal OSC32K) measured
  23774238 crystal ticks: the OSC32K runs **+0.94 % fast** on this
  board (spec +-10 %).
- Event counting (COUNT user) and the TIMEOUT mode behave as the
  datasheet says: 200 events in 200 ms; a 2 ms high times out a 1 ms
  watch once per period, a 500 us high never.
- **Timeout duration measured to the tick** (T0 free-running stamps
  the start edge and the CAPT event through the same sync user; the
  one-shot, exact by test 4, calibrates the path): the event-path
  offset is a constant 2 ticks, deterministic across runs; the
  timeout then reads TOP + 2 at both 12000 and 24000 - CAPT fires
  exactly TOP ticks after the start. With the stop edge 9.5 ms away
  and TOP = 1 ms, CAPT re-fires at every CNT wrap through TOP (42
  in 105 ms = 4 per period).
- The meter variants are exact like their defaults: PW low = 16800,
  FRQ falling = 24000, inverted duty = 24000/16800 on a 1 kHz, 300 us
  high signal. The OVF safety net works: TOP lowered below a running
  CNT -> OVF at MAX, then the periodic resumes at the new TOP.
  `capture_on_events(false)` silences a Timeout and re-arming
  restores it.
- The other counter clocks are as stated: `div2` reads 12000 - 1 on
  the 1 kHz signal (the half tick rounds down there), a TCA1 CLK_TCA
  at div8 reads 3000 exact (and `tick_hz()` honestly 0). `CCMPINIT`
  drives the idle WO level both ways (read back from the pin). The
  ALT1 route verified on TCB2 -> PB4 (Pwm8 at 256 ticks measured
  through EvPin); TCB0/TCB1 ALT1 sit on the console and stay
  unverified.
- **Live rebase** (DynamicClock 24 -> 12 -> 24 MHz under a running
  Heartbeat + FrequencyMeter, console rebased too): the ticks track
  CLK_PER (24000 -> 12000 -> 24000), the microseconds stand still
  (1000 at every step) - the whole fan-out works under fire.

## Not covered yet

Driver gaps (the chapter's features the driver does not implement,
or implements wrongly):

- Pin-level bonding within an existing port is not modeled (the
  guards are port-level): on the 28-pin part PORTF exists but
  PF4/PF5 (TCB0/TCB1 ALT1) are not bonded, and `alt_pin` there
  compiles and programs a mute position. Deferred to the family
  device tables.

Implemented but not bench-verified:

- DBGRUN under a halt; TCB4 and the 28/32-pin builds
  (compile-verified for every package, no such part on the bench).
- The ALT1 positions of TCB0/TCB1 (PF4/PF5 are the console; the ALT1
  mechanism itself is verified on TCB2 - moving the console to free
  them is a wiring job).
