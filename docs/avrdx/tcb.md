# TCB - the 16-bit timer/counter type B (AVR DA/DB)

> **PROVISIONAL.** The systematic review of the chapter and errata is
> done (this page records it); the driver and its bench suite are not
> written yet. Documents of record: AVR128DB28/32/48/64 data sheet
> DS40002247B (TCB chapter 24, PORTMUX 17.3.8, EVSYS 16 generators
> 0xA0-0xA9 / users 0x1E-0x27), errata DS80000915F (2.13.1, 2.13.2).
> Complements: TB3214 "Getting Started with TCB" and the Microchip
> examples `avr128da48-getting-started-with-tcb`,
> `avr128da48-tcb-frequency-dutycycle-measurement` (see
> [vendor/README.md](vendor/README.md)). Driver: none yet. Reference
> test: none yet.

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
  last at least one CLK_PER (two in PW/FRQPW: minimum edge
  separation of two clock cycles); a `FILTER` (four equal samples at
  CLK_PER, +4 cycles of delay) is available. In SINGLE mode `ASYNC`
  drives WO high on the event itself (before the counter starts,
  2-3 CLK_TCB later) and the event may be shorter than one CLK_PER.
- **Reading a capture clears `CAPT`** (on the LOW byte of CCMP):
  read CCMP in the ISR and the flag is gone; in FRQPW read CNT
  (period) BEFORE CCMP (width), since reading CCMP re-arms the
  sequence. The 16-bit registers go through `TEMP` (low byte first on
  read, high byte first on write) - the compiler's 16-bit access does
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
  TCB3 PB5 / PC1 (TCB4 PG3 / PC6 - the ALT1 is dead, errata 2.13.2,
  not our package). Note the collisions with the bench: TCB0/TCB1 ALT1
  sit on PF4/PF5 = the console USART2; TCB2 default on PC0 = TCA0
  PORTC WO0.
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
`TCBn CAPT` (0x1E + 2n, edge, both sync and async) and `TCBn COUNT`
(0x1F + 2n, sync).

What TCB is NOT: there is one compare register, so no PWM with an
independent period and duty beyond 8 bits, no dual-slope, no
direction control, no period buffering (writing CCMP takes effect at
once) - that is TCA's job ([tca.md](tca.md)). TCB is the precise
one-channel tool: measure, count, pulse, tick.

## Types and verbs

None yet. The intended shape (overview.md "Target strata"): a
resource `Tcb<n>` - the typed view of one instance (config struct
with mode, clock, edge, filter, async, cascade, sync-update,
run-standby, output enable/initial level, route; `init<cfg>()` /
`init(cfg)`, enable/disable, `count()`, `compare()`/`capture()`,
`running()`, `capt()`/`ovf()` ISR bodies, the CAPT/COUNT event users
and CAPT/OVF generators for [evsys.md](evsys.md)) - and tasks named
for what they do over it: one-shot pulse, pulse/event counter,
cascaded 32-bit counter, frequency / period / pulse-width meter,
periodic tick and time-out, 8-bit PWM channel.

## How to use it

Nothing to show yet.

## Bench findings

None yet.

## Not covered yet

Everything above: the resource, the tasks, the suite (`test_avr_timer`
is the intended name: TCA generates a known waveform, a TCB measures
it back - frequency, duty, 32-bit capture against the PIT), the
EVSYS vocabulary for TCB, the ClockUser question (a TCB timing from
CLK_PER changes meaning when the main clock moves: timeouts and PWM8
need a rebase, captures need the rate at capture time).
