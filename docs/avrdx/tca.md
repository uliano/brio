# TCA - the 16-bit timer/counter type A (AVR DA/DB)

> **PROVISIONAL.** Normal mode, the PORTMUX routes (the device
> header's exact set, per package), CTRLC and the split-mode
> flag/interrupt surface are covered and bench-verified, and the
> dual-slope modes have their task (TcaPwmCentered); the split
> halves' counters as an API are not - the gaps are in "Not covered
> yet". Documents of
> record: AVR128DB28/32/48/64 data sheet DS40002247B (TCA chapter
> 23, PORTMUX 17.3.7, EVSYS 16 generators 0x80-0x8E / users 0x1A-0x1D),
> errata DS80000915F (2.12.1). Complements: TB3217 "Getting Started
> with TCA", AN2434 (quadrature decoding with CCL + TCA + TCB) - see
> [vendor/README.md](vendor/README.md). Driver: `avrdx/tca.hpp` (the
> `Tca<n>` resource and the tasks), the TCA event vocabulary in
> `avrdx/evsys.hpp`. Reference test: `test_avr_timer`.

## What the silicon does

Two instances, TCA0 and TCA1 (48/64-pin parts). One 16-bit counter
with a period register (`PER`), three compare channels (`CMP0..2`)
each with its waveform output (`WO0..2`), ALL double-buffered
(`PERBUF`, `CMPnBUF` with buffer-valid flags in `CTRLF`, copied at the
UPDATE condition unless `LUPD` locks it), direction control (`DIR`),
and a split mode that makes it two 8-bit timers with three compare
channels each (`WO0..5`).

Modes (`WGMODE`) and what defines TOP / the update point:

| Mode | TOP | WO behaviour | UPDATE at |
|------|-----|--------------|-----------|
| `NORMAL` | PER | none (compare flags/events only) | TOP |
| `FRQ` frequency | CMP0 | WOn toggles at each CMPn match: f = CLK_PER / (2 N (CMP0+1)), max CLK_PER/2; CMP1/CMP2 give phase-offset copies | TOP |
| `SINGLESLOPE` | PER | set at BOTTOM, cleared at match (CMPn = 0 -> static low, CMPn > TOP -> static high); f = CLK_PER / (N (PER+1)); up-count only | BOTTOM |
| `DSTOP` / `DSBOTH` / `DSBOTTOM` dual-slope | PER | up then down; cleared at match up, set at match down; f = CLK_PER / (2 N PER); OVF at TOP / both / BOTTOM | BOTTOM |

Split mode: two independent 8-bit down-counters (`LCNT`/`HCNT`,
`LPER`/`HPER`, `LCMPn`/`HCMPn`), single-slope only (cleared at BOTTOM,
set at match; max duty TOP/(TOP+1); CMPn = 0 or > TOP -> static low),
byte access, no buffering, no event actions; LUNF/HUNF underflow
interrupts, compare interrupts on the low half only. Switching
between normal and split needs ENABLE = 0 and a RESET command
(register meanings change, values do not).

Facts that matter to code:

- **Clock**: CLK_PER through a prescaler N = 1/2/4/8/16/64/256/1024
  (`CLKSEL`); the prescaled clock is also exported to the TCBs
  (`CLK_TCA`, [tcb.md](tcb.md)). Or events: `CNTA` user (0x1A/0x1C)
  with `EVACTA` = count on positive / any edge, count clock while the
  event is high, or direction from the event level (up when low);
  `CNTB` user (0x1B/0x1D) with `EVACTB` = restart on positive / any
  edge / while high, or direction too (both ORed: count up only when
  both low). Level actions are reliable only when the event is slower
  than the timer clock. Event inputs do nothing in split mode.
- **Double buffering is the way to change period/duty on the fly**:
  an unbuffered PER write below CNT makes the counter wrap to MAX
  first (odd waveform); buffered writes wait for UPDATE. `ALUPD`
  auto-locks the update until all used buffers are written (PER and
  the enabled CMPn), so a multi-register change lands in one period.
  The FRQ mode has no double buffering of its own period beyond CMP0BUF.
- **Commands (`CTRLESET.CMD`)**: UPDATE (forces the buffer copy,
  ignores LUPD), RESTART (CNT = 0 and all WO low - restarts the
  period), RESET (all registers to reset, only when disabled).
  Errata 2.12.1 (A4/A5): a RESTART command or restart EVENT in
  NORMAL/FRQ mode resets DIR to up - a down-counting FRQ user must
  rewrite DIR after every restart.
- **Output**: `CMPnEN` makes WOn override PORT.OUT in any waveform
  mode (pin must be an output; PORT `INVEN` inverts it); the endpoints
  of single-slope (0 = low, > TOP = high) are clean in normal mode -
  in split mode CMP > TOP is LOW, which is why the split-mode driver
  drives the endpoints from PORT. Routes (PORTMUX `TCAROUTEA`, 3 bits
  per instance, the whole group moves): TCA0 to PORTA/B/C/D/E/F/G
  pins 0..5; TCA1 to PORTB pins 0..5 (full), PORTC PC4..PC6 and PORTE
  PE4..PE6 (WO0..2 only), PORTG.
- **CTRLC parks the waveform pins**: the CMPnOV bits (split:
  LCMPnOV/HCMPnOV) read the WO level any time and WRITE it while the
  timer is disabled - the one handle on a parked pin between runs.
  Toward the CCL the output bypasses CMPnEN: a parked value feeds a
  LUT too.
- **The 16-bit registers of an instance share one TEMP register**
  (23.5.13): a main-context `count()`/`compare()` racing an ISR's
  16-bit access corrupts through it - such reads belong in a critical
  section or in the ISR alone.
- **Interrupts / events**: normal mode OVF (at TOP or BOTTOM
  depending on the mode) and CMP0..2 match, each a vector, each an
  event generator (`OVF_LUNF` 0x80/0x88, `CMPn_LCMPn` 0x84-86/0x8C-8E,
  one CLK_PER pulses); split mode adds HUNF (0x81/0x89) and loses the
  high-half compare flags. The event conditions are the flag
  conditions.
- `RUNSTDBY` keeps it counting in standby; `DBGRUN` under a debugger
  halt. Writing CNT has priority over count/clear/reload and is
  immediate.

What TCA is NOT: no input capture of any kind (a TCB captures,
optionally clocked by `CLK_TCA` and restarted with the TCA via
SYNCUPD, which is how a TCA period and a TCB capture stay in phase);
no fault/blanking inputs (TCD). TCA is the waveform and period
engine: PWM in 16 bits, frequency generation, a period that may be
counted in events, a heartbeat with up to three phase-related outputs
and an overflow event for the rest of the chip.

## Types and verbs

The resource, `Tca<n>` (n = 0, 1), in normal mode, and the tasks over
it; split mode is reached by the `TcaPwm` task through the resource's
`split()` view. All static (monostate); a task owns its instance.

| Entity | Verbs |
|--------|-------|
| `TcaConfig` | `mode` (`TcaMode`: normal, frequency, single_slope, dual_slope_top/both/bottom), `clock` (`TcaClock`: div1..div1024), `period` (PER), `compare0..2`, `outputs` (CMPnEN mask, pins driven as outputs), `route` (the PORTMUX group; the routes the driver knows are below), `event_a` (`TcaEventA`: none, count_posedge, count_anyedge, count_while_high, direction), `event_b` (`TcaEventB`: none, direction, restart_posedge/anyedge/while_high), `auto_lock_update`, `count_down`, `run_standby`, `debug_run` |
| `Tca<n>` | `init<cfg>()` / `init(cfg)` (disable, hard reset, route, drive outputs, configure, clear flags, enable; the compile-time form static_asserts the route), `enable`/`disable`, `clock(c)` (prescaler under run), `route(p)`, `reset` (disable + CMD RESET); `count`/`count(v)`, `period`/`period(v)`, `compare<ch>()`/`compare<ch>(v)` (immediate), `period_buffered(v)`, `compare_buffered<ch>(v)` (land at UPDATE), `update_pending`, `output<ch>(on)`; `restart`, `update`, `direction_down(b)`, `counting_down`, `lock_update(b)`; `ovf_flag`/`cmp_flag<ch>`/`clear_ovf`/`clear_cmp<ch>`, `enable_ovf_interrupt`/`enable_cmp_interrupt<ch>`, `ovf()`/`cmp<ch>()` (ISR bodies, one vector per flag), `take_flags`; `output_value<ch>(high)`/`output_value<ch>()` (CTRLC parking); split-mode `lunf_flag`/`hunf_flag`/`lcmp_flag<ch>` with their clears, `enable_lunf/hunf/lcmp_interrupt`, ISR bodies `lunf()`/`hunf()`/`lcmp<ch>()` (LUNF shares the OVF vector, HUNF has its own); `event_a_on(channel)`/`event_b_on(channel)` (the action is in the config); `single()`/`split()` register views, `drive_outputs(port, mask)`, `port_of(p)`; `OvfEvent`/`HunfEvent`/`CmpEvent<ch>` generators, `EventA`/`EventB` users |
| `TcaPwm<n, port>` | split mode, six 8-bit channels WO0..5 on pins 0..5: `init(TcaClock)`, `clock(c)`, `duty<ch>(uint8_t)` (0/255 from PORT), `Channel<ch>` (PwmChannel, max 255); `channels = 6` |
| `TcaPwm16<n, port, steps>` | single-slope, three 16-bit channels of one period: `init(TcaClock, outputs)`, `clock(c)`, `duty<ch>(v)` (buffered; 0 = low, steps = high), `Channel<ch>` (PwmChannel, max = steps). The clean endpoints use CMP > TOP, so PER = MAX (65536 steps) is deliberately not expressible: the last step buys glitch-free rails |
| `TcaPwmCentered<n, port, steps>` | dual-slope (center-aligned): three 16-bit PwmChannels of one period, pulse symmetric around BOTTOM, f = CLK_PER / N / (2 steps); `init(TcaClock, outputs, EventAt::top/both/bottom)` places the OVF event at the electrically quiet instants (pulse centre / low centre) - the ADC-pacing knob (EvTcaOvf -> EvAdc0Start); `duty<ch>(v)` buffered, endpoints from PORT; ratiometric, not a ClockUser; works on the three-channel TCA1 routes |
| `FrequencyGenerator<n, port>` | FRQ on WO0: `init(clock, hz)`, `set_hz(hz)` (buffered), `actual_hz`, `rebase` (ClockUser), `start`/`stop` |
| `Heartbeat<n, port>` | a period at a rate with pulses: `init(clock, hz, outputs, interrupt)`, `pulse_us<ch>(us)` (from the start of each period, buffered), `beat` (ISR body), `rebase` (ClockUser), `tick_hz` |
| `EventCounter<n>` | `init(source_channel, action)`, `direction_on(channel)` (input B level: down while high), `count`, `reset`, `overflowed`, `ovf` (ISR body) |
| helpers | `tca_divisor(c)`, `tca_route_code(n, p)`, `tca_wo_pin(n, p, wo)`, `tca_pin_mask`, `tca_timing(ticks_at_div1)` (the smallest prescaler that fits), `tca_period_ticks(clk_per, hz)` |

Routes are checked where the port is a template argument (the tasks)
and at run time in `init(cfg)`; `tca_route_code` mirrors exactly the
codes THIS package's device header defines (PORTG and TCA1 -> PORTE
on 64-pin parts only; no PORTB/PORTE routes on 28/32-pin). TCA1 on
PORTC or PORTE is a three-channel route (WO0..2 on pins 4..6):
`TcaPwm` refuses it - six channels or nothing. One pin-level caveat
survives the port-level gates: TCA0 -> PORTE on 48-pin parts bonds
only PE0..PE3 (WO4/5 land nowhere).

## How to use it

Six LED channels (split mode) and a generic actuator over them:

```cpp
#include "avrdx/tca.hpp"
using PwmC = brio::TcaPwm<0, 'C'>;           // TCA0 -> PC0..PC5
PwmC::init();                                // split mode, div16 (~5.9 kHz), all dark
PwmC::duty<2>(64);                           // PC2 at 25 %
using Lamp3 = brio::RgbLamp<PwmC::Channel<0>, PwmC::Channel<1>, PwmC::Channel<2>>;
Lamp3::show({255, 40, 0});
```

A servo (50 Hz, 1..2 ms) with 16-bit resolution, duty changes landing
at the period boundary:

```cpp
using Servo = brio::TcaPwm16<1, 'B', 60000>;    // 24 MHz / 8 / 60000 = 50 Hz
Servo::init(brio::TcaClock::div8, 0x01);        // WO0 on PB0
Servo::duty<0>(4500);                           // 1.5 ms
```

A square wave, a heartbeat with a pulse and an event counter:

```cpp
using Gen = brio::FrequencyGenerator<0, 'D'>;   // WO0 = PD0
Gen::init(clock, 1000);  Gen::set_hz(440);

using Beat = brio::Heartbeat<0, 'D'>;
Beat::init(clock, 100, 0x01);                   // 100 Hz, pulse on WO0
Beat::pulse_us<0>(500);                         // 500 us high from each period start
ISR(TCA0_OVF_vect) { Beat::beat(); post<Sequencer>(Tick{}); }

using Edges = brio::EventCounter<1>;
Edges::init(brio::EventChannel<4>{});           // counts the channel's positive edges
```

Center-aligned PWM with the ADC paced at the pulse centre (no CPU in
the loop):

```cpp
using Motor = brio::TcaPwmCentered<0, 'D', 12000>;   // 1 kHz at div1
Motor::init(brio::TcaClock::div1, 0x01, Motor::EventAt::bottom);
Motor::duty<0>(3000);                                // 25 %, centred
brio::EventChannel<4>::source(brio::Tca<0>::OvfEvent{});
brio::EvAdc0Start::listen(brio::EventChannel<4>{});  // convert at the quiet instant
```

The resource directly (a dual-slope PWM with compare interrupts - the
interrupts are what the task does not name):

```cpp
using T = brio::Tca<0>;
T::init<brio::TcaConfig{.mode = brio::TcaMode::dual_slope_both, .clock = brio::TcaClock::div4,
                        .period = 1000, .compare0 = 250, .outputs = 0x01, .route = 'A'}>();
T::compare_buffered<0>(500);                    // lands at BOTTOM
T::enable_cmp_interrupt<0>(true);
ISR(TCA0_CMP0_vect) { T::cmp<0>(); ... }
```

Clock change: `FrequencyGenerator` and `Heartbeat` are ClockUsers
(list them in the DynamicClock); the PWM tasks are not - their
frequency follows CLK_PER and a LED does not care; an app that does
calls `clock(c)` itself.

## Bench findings

- Twelve split-mode channels on two timers drive four RGB LEDs on the
  bench; colour mixing is limited by the LEDs' dies, not by the PWM.
- The split -> normal escape works (RESET carries CMDEN through the
  split view): a `TcaPwm` at 50 % on PC0 measures 4096 ticks, a
  `FrequencyGenerator` initialized right after it runs exact (24000).
  HUNF is alive in split mode: the flag sets within a period and the
  event counts 117 +-2 per 20 ms at div16/256. `CTRLC` parks WO0 high
  and low while disabled, read back from the pin.
- `TcaPwmCentered` measured (1 kHz from 12000 steps at div1): period
  2 x steps and width 2 x CMP exact, buffered duty lands at BOTTOM;
  the OVF event sits at the pulse centre - the rising-edge-to-OVF
  distance scales exactly with CMP (delta 3000 for delta-CMP 3000),
  with a small constant skew (-3: the edge is stamped through the
  pin's input synchronizer, the OVF event is internal).
- `test_avr_timer` (A5, 24 MHz crystal), measured by the TCBs against
  the same crystal: FRQ generation at 500 Hz .. 100 kHz exact to the
  tick (48000 / 24000 / 2400 / 240); `set_hz` under a running output
  lands through CMP0BUF at the next TOP - and **CMP0 keeps reading the
  old value** afterwards even though the output already follows the
  buffer (FrequencyGenerator::actual_hz computes from what it set, not
  from the register); single-slope 16-bit PWM (24000 steps) at 25/50/
  75 % duty exact, endpoints 0 and steps static; Heartbeat at 100 Hz
  (div4, 60000 ticks) gives 100 OVF per second and a 500 us pulse of
  exactly 12000 CLK_PER ticks; count-on-event and direction-from-level
  count 200 up in 200 ms and 100 down in the next 100 ms.

## Not covered yet

Driver gaps:

- The split halves' counters/periods/compares as verbs (LCNT/HCNT,
  LPER/HPER, LCMPn/HCMPn - today only through the raw `split()`
  view; the flags, interrupts and ISR bodies are verbs already).
- Pin-level bonding within an existing port (TCA0 -> PORTE on
  48-pin parts bonds only PE0..PE3) - deferred to the family device
  tables; the route-level legality is already the device header's.
- No guarded 16-bit read for the TEMP hazard (the constraint is
  documented above; the caller owns it).

Implemented but not bench-verified:

- The dual_slope_top/both OVF placements (bottom is bench-verified
  by the centred task); the split ISR bodies (the flags and the HUNF
  event are bench-verified, the vectors are not bound by any test).
- RUNSTDBY under a real standby (queued, LOW priority - needs a
  sleeping app); DBGRUN under a halt.
- Event input B restart actions; count_anyedge/count_while_high on
  input A; FRQ phase-offset outputs on CMP1/CMP2; ALUPD multi-register
  updates; the UPDATE command.
- TCA1 on PORTC (WO0..2) - not testable on this bench; the PORTG and
  TCA1 PORTE routes (compile-verified on the 64-pin headers, no such
  part on the bench).
- Errata 2.12.1 is documented, not worked around (no down-counting
  FRQ user; B0 silicon is not affected).
