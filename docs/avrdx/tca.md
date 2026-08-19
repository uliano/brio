# TCA - the 16-bit timer/counter type A (AVR DA/DB)

> **PROVISIONAL.** The systematic review of the chapter and errata is
> done (this page records it); the driver is still one task on the
> peripheral - six 8-bit PWM channels in split mode - and the
> `Tca<n>` resource with the other tasks is not written yet. Documents
> of record: AVR128DB28/32/48/64 data sheet DS40002247B (TCA chapter
> 23, PORTMUX 17.3.7, EVSYS 16 generators 0x80-0x8E / users 0x1A-0x1D),
> errata DS80000915F (2.12.1). Complements: TB3217 "Getting Started
> with TCA", AN2434 (quadrature decoding with CCL + TCA + TCB) - see
> [vendor/README.md](vendor/README.md). Driver: `avrdx/pwm.hpp`
> (`TcaPwm`). Reference tests: `traffic2` (twelve PWM channels on
> TCA0/PORTC and TCA1/PORTB).

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

## What the driver does today

`TcaPwm<n, port>`: TCA n in SPLIT mode, both 8-bit halves in single-
slope PWM with PER = 255, routed to one port (PORTMUX: TCA0 to PORTA/
B/C/D/F, TCA1 to PORTB for six channels), pins 0..5 = WO0..5, one
shared prescaler (`TcaClock`, div16 -> ~5.9 kHz at 24 MHz). `duty<ch>
(v)` is one store; 0 and 255 leave the waveform and drive the pin from
PORT.OUT (clean endpoints, DxCore's policy). `Channel<ch>` is the
`PwmChannel` type generic actuators use (`RgbLamp`). The task owns the
whole timer: no other use of that TCA instance coexists.

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `TcaPwm<n, port>` | `init(TcaClock)`, `duty<ch>(uint8_t)`, `Channel<ch>` (PwmChannel: `max = 255`, `duty(v)`); `channels = 6` |
| `TcaClock` | `div1 .. div1024` |

## How to use it

```cpp
using PwmC = brio::TcaPwm<0, 'C'>;           // TCA0 -> PC0..PC5
PwmC::init();                                // split mode, div16, all dark
PwmC::duty<2>(64);                           // PC2 at 25 %
using Lamp3 = brio::RgbLamp<PwmC::Channel<0>, PwmC::Channel<1>, PwmC::Channel<2>>;
Lamp3::show({255, 40, 0});
```
Clock change: the PWM frequency scales with CLK_PER (not a ClockUser
today - a fact to decide in the exhaustive pass: keep the frequency, or
the prescaler?).

## Bench findings

- Twelve channels on two timers drive four RGB LEDs; colour mixing is
  limited by the LEDs' dies, not by the PWM.

## Not covered yet

In the driver: everything of "What the silicon does" except split-
mode PWM - the `Tca<n>` resource (normal mode: period and three
buffered compare channels, FRQ, single- and dual-slope 16-bit PWM,
the CNTA/CNTB event actions, OVF/CMPn ISR bodies and event
generators, the commands, RUNSTDBY, the partial TCA1 routes, errata
2.12.1) and the tasks over it (16-bit PWM channel, frequency
generator, event counter, heartbeat with an overflow event; `TcaPwm`
becomes the split-mode task on the same resource), the ClockUser
question above, the reference suite (`test_avr_timer`, with the TCBs
measuring what the TCA generates).
