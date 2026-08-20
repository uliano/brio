# TCB - the 16-bit timer/counter type B (AVR DA/DB)

> **PROVISIONAL.** The resource covers the chapter's full option
> space on every instance of every package (TCB4 included, gated by
> the device header), the microsecond-speaking tasks honor the clock
> contract (ClockUsers), and the whole set is bench-verified on
> TCB0..TCB3; what remains is in "Not covered yet". Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B (TCB
> chapter 24, PORTMUX 17.3.8, EVSYS 16 generators 0xA0-0xA9 / users
> 0x1E-0x27), errata DS80000915F (2.13.1, 2.13.2). Complements: TB3214
> "Getting Started with TCB" and the Microchip examples
> `avr128da48-getting-started-with-tcb`,
> `avr128da48-tcb-frequency-dutycycle-measurement` (see
> [vendor/README.md](vendor/README.md)). Driver: `avrdx/tcb.hpp` (the
> `Tcb<n>` resource and the tasks), the TCB event vocabulary in
> `avrdx/evsys.hpp`. Reference test: `test_avr_timer`.

## What the silicon does

Five instances on the family, TCB0..TCB3 on the 48-pin parts (TCB4
only on 64 pins). Each is a 16-bit counter with ONE capture/compare
register (`CCMP`), ONE waveform output (`WO`), two flags (`CAPT`,
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
  and if the stop edge has not arrived the counter FREE-RUNS past
  TOP, wraps, and CAPT fires again at every pass through TOP (every
  65536 ticks): a watch on a long-silent signal keeps shouting, once
  per wrap.
- **Arming a synchronous CAPT-in on a channel already high reads a
  spurious rising edge** (bench, A5): the user's edge detector starts
  from zero, so enabling CAPTEI while the event level is 1 detects an
  edge that never happened on the wire. Arm while the channel is low
  (or discard the first capture).
- **Reading a capture clears `CAPT`** (on the LOW byte of CCMP):
  read CCMP in the ISR and the flag is gone; in FRQPW read CNT
  (period) BEFORE CCMP (width), since reading CCMP re-arms the
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
  silicon revision, errata 2.13.2). TCB2's default PC0 collides with
  TCA0's PORTC WO0 route; the board-level collisions are in
  [bench.md](../bench.md).
- **32-bit capture**: two TCBs, the LSB one clocked from the source
  with `CASCADE = 0`, the MSB one clocked from the LSB's `OVF` event
  (`COUNT` user) with `CASCADE = 1` (delays its CAPT input by one
  CLK_PER to absorb the carry), both in the SAME capture mode, the
  same CAPT event routed to both; the value is MSB.CCMP:LSB.CCMP.
  Any pair works (the datasheet example is TCB0/TCB1); it costs two
  event channels.
- **Do not change mode while enabled** (unpredictable output); a flag
  may set during configuration - clear INTFLAGS after configuring.
  Periodic interrupt with TOP lowered below CNT: the counter goes on
  to MAX, OVF fires, restarts - an OVF ISR is the safety net when TOP
  is changed on the fly.
- `RUNSTDBY` keeps it counting in standby; `DBGRUN` keeps it running
  under a debugger halt (default: halted and blind to events - a
  single-shot under PyAvrOCD stops with the CPU).
- **Errata 2.13.1 (A4/A5)**: in PWM8 the CCMP and CNT bytes are not
  independent - write CCMP as one 16-bit value (period and duty
  together), never byte-wise.

As event generator: `TCBn CAPT` (0xA0 + 2n) and `TCBn OVF` (0xA1 +
2n), one CLK_PER pulses, same conditions as the flags. As event user:
`TCBn CAPT` (0x1E + 2n, edge; synchronous - asynchronous too only in
single-shot, 24.3.4) and `TCBn COUNT` (0x1F + 2n, sync).

What TCB is NOT: there is one compare register, so no PWM with an
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
#include "avrdx/evsys.hpp"
#include "avrdx/tcb.hpp"
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

`test_avr_timer` (silicon rev A5, 24 MHz crystal, 3.3 V; 82/82), all
measured against the same crystal:

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

- RUNSTDBY under a real standby (no sleeping app yet); DBGRUN under
  a halt; TCB4 and the 28/32-pin builds (compile-verified for every
  package, no such part on the bench).
- The ALT1 positions of TCB0/TCB1 (PF4/PF5 are the console; the ALT1
  mechanism itself is verified on TCB2 - moving the console to free
  them is a wiring job, queued).
- Event-paced captures inside the kernel (an AO owning a meter and
  publishing Hz/period/duty readings as events, the way AnalogSampler
  owns the ADC) - queued as a util/ usage type for when an app needs
  it.
