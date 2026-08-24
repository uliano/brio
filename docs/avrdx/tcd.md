# TCD - the 12-bit timer/counter type D (AVR DA/DB)

> **PROVISIONAL.** The whole chapter-25 register description is
> exposed, the three synchronization disciplines are enforced by the
> verbs rather than left to the caller, the package route table carries
> the pin-level bonding facts, and the input-mode validity table plus
> two of the three TCD errata are refusals at compile time and at run
> time. What remains is in "Not covered yet": one usage type is built
> (the complementary pair) and the others wait for their first user,
> the external clock source has no wiring here, and two errata could
> not be provoked on this die. Documents of record: AVR128DB28/32/48/64
> data sheet DS40002247B (TCD chapter 25, PORTMUX 17.5, EVSYS 16
> generators 0xB0-0xB3 / users, CLKCTRL 12.3.5, electricals 39.10.5),
> errata DS80000915F (2.14.1-2.14.3, 2.5.3, 2.5.4) and DS80000882C
> (2.13.1-2.13.3, 2.4.1). Driver: `avrdx/tcd.hpp` (`Tcd<0>`,
> `TcdPwm<route>`), the TCD event vocabulary in `avrdx/evsys.hpp`, the
> PLL in `avrdx/clock.hpp`. Reference test: `test_avr_tcd`.

## What the silicon does

One instance, TCD0, on every package of both families. A 12-bit
counter in a clock domain ASYNCHRONOUS to CLK_PER, two compare/capture
units each with a waveform output (WOA, WOB), two more outputs that
copy one of them (WOC, WOD), two event inputs with ten fault-handling
modes, two capture registers, a fractional-frequency (dither)
accumulator, four event generators and two interrupt vectors. It is
the only consumer of the PLL on this silicon, and therefore the only
way to run a timer above CLK_MAIN's 24 MHz ceiling.

**The clock.** CLK_TCD comes from OSCHF, the PLL, EXTCLK or CLK_PER,
then through SYNCPRES (1/2/4/8) into the SYNCHRONIZER clock and
through CNTPRES (1/4/32) into the COUNTER clock, so the counter runs
at `source / (SYNCPRES x CNTPRES)`. The synchronizer clock is what the
input processing logic and the delay block run on; a third prescaler
(DLYPRESC) divides it again for blanking and the programmable output
event. A TCD clocked from anything but CLK_PER does not move when the
main prescaler does.

**The four waveform modes** (25.3.3.2), each a walk of the same four
states - dead-time A, on-time A, dead-time B, on-time B - with the
four compare values marking the boundaries:

| Mode | Counter | Cycle in counter ticks | WOA high | WOB high |
|------|---------|------------------------|----------|----------|
| One ramp | 0..CMPBCLR, one reset | CMPBCLR + 1 | CMPACLR - CMPASET | CMPBCLR - CMPBSET |
| Two ramp | 0..CMPACLR, 0..CMPBCLR | (CMPACLR + 1) + (CMPBCLR + 1) | CMPACLR - CMPASET | CMPBCLR - CMPBSET |
| Four ramp | 0..each compare in turn | the four compares plus four | CMPACLR + 1 | CMPBCLR + 1 |
| Dual slope | CMPBCLR down to 0 and up | 2 x (CMPBCLR + 1) | 2 x CMPASET | 2 x (CMPBCLR - CMPBSET + 1) |

Every one of those columns is measured, to the tick, by the reference
test. The dual-slope row is the one place where the bench CORRECTS the
chapter: 25.3.3.2.4 prints `T = (2 x CMPBCLR + 1) / f`; the silicon
runs `2 x (CMPBCLR + 1)` counter ticks at every geometry tried - the
ramp down and the ramp up are each CMPBCLR + 1 ticks long, like every
other mode's ramp. `tcd_cycle_ticks()` returns the measured number.

Ordering facts of One Ramp mode: a match with CMPBCLR clears EVERY
output, so a compare bigger than CMPBCLR never has its effect (WOA
then runs to the end of the cycle); a CMPACLR SMALLER than CMPASET has
no effect at all (same result); and CMPBSET below CMPASET makes the two
on-times overlap, which is exactly what the complementary pair must
avoid. In Dual Slope mode CMPACLR is unused - it can be written, it
reads back, and nothing moves.

**Three synchronization disciplines** (25.3.3.1), and they are the
whole shape of the driver:

1. **ENABLE** (CTRLA bit 0) may only be written while STATUS.ENRDY is
   1. Writing it closes ENRDY until the value has crossed into the TCD
   domain.
2. **The CTRLE strobes** (SYNC, SYNCEOC, RESTART, SCAPTUREA/B, DISEOC)
   and the AUPDATE path may only be issued while STATUS.CMDRDY is 1;
   the double-buffered registers (CMPxSET/CLR, DLYCTRL/DLYVAL,
   DITCTRL/DITVAL, DBGCTRL) are written under the same flag. CTRLE is
   a strobe register: it clears itself when the command is sent.
3. **The static registers** (CTRLB, CTRLC, CTRLD, EVCTRLA/B,
   INPUTCTRLA/B, FAULTCTRL and every CTRLA bit except ENABLE) cannot
   be written while the TCD is enabled.

FAULTCTRL is additionally under Configuration Change Protection (IOREG
key). The two CAPTURE registers are read LOW BYTE FIRST, and the read
of the HIGH byte is what releases the buffer for the next capture.

**Double buffering and AUPDATE.** The four compares reach the TCD
domain on a SYNC strobe, at the end of the next cycle on SYNCEOC, or -
with CTRLC.AUPDATE set - automatically at the end of the cycle the
moment CMPBCLR's HIGH byte is written. That last one is the PWM
update path: write the new duty compares, then write CMPBCLR (even
unchanged) and the whole set lands together on a cycle boundary.
CTRLC.FIFTY mirrors a write to either SET register into both, and
either CLR register into both.

**The inputs.** Two event users, each with its own EVCTRL (enable,
capture-as-well-as-fault, edge/level polarity, and a qualifier: plain,
a four-counter-cycle digital filter, or ASYNC) and its own INPUTCTRL
(one of eleven input modes). The modes are not all legal in all
waveform modes - table 25-5 - and the driver enforces the table.
Modes 1/2/3/5/6 execute "the opposite compare cycle", which One Ramp
mode does not have; Dual Slope mode keeps only 0, 4 and 7, and errata
2.14.3 takes 7 away there as well, leaving 0 and 4.

**Input blanking and the programmable output event** share DLYCTRL -
one trigger (a compare match), one prescaler of the synchronizer
clock, one 8-bit value - so they are ONE choice, not two features. The
driver makes that a single enum field, which is the refusal.

**Dithering** adds DITHER/16 of a counter tick to a chosen part of the
cycle at every accumulator overflow. Where table 25-7 says the
addition to the CYCLE is 0 (one/two-ramp with a dead-time selection)
the compensation is taken out of the following output state instead:
the cycle length does not move, the duty does. Not supported at all in
Dual Slope mode.

**Outputs.** FAULTCTRL.CMPxEN enables an output on its pin and
FAULTCTRL.CMPx is the level that output takes while a fault is active.
CTRLC.CMPCSEL/CMPDSEL pick whether WOC and WOD copy waveform A or B.
With CTRLC.CMPOVR the four cycle states drive the two waveforms
directly from CTRLD (tables 25-12 and 25-13) - which also makes each
dead time individually visible on a pin, and is how the reference test
measures them.

**Routes** (PORTMUX.TCDROUTEA), exactly the device headers' own enums
with the pin-level bonding facts their comments carry. There is no
pinless route code: a TCD with no output enabled in FAULTCTRL is the
pinless configuration.

| Route | WOA | WOB | WOC | WOD | Packages |
|-------|-----|-----|-----|-----|----------|
| DEFAULT | PA4 | PA5 | PA6 | PA7 | all |
| ALT1 | PB4 | PB5 | - | - | 48-pin (WOC/WOD pinless) |
| ALT1 | PB4 | PB5 | PB6 | PB7 | 64-pin |
| ALT2 | PF0 | PF1 | - | - | 28-pin (WOC/WOD pinless) |
| ALT2 | PF0 | PF1 | PF2 | PF3 | 32/48/64-pin |
| ALT3 | PG4 | PG5 | PG6 | PG7 | 64-pin only |

**Events.** Generators: CMPBCLR, CMPASET and CMPBSET (a pulse one
CLK_TCD_CNT period long, at the matching compare) and PROGEV (one
CLK_TCD_SYNC period, from the delay block). Users: input A and input
B. Two interrupt vectors: TCD0_OVF (once per TCD cycle) and TCD0_TRIG
(TRIGA and TRIGB, one flag each, both write-one-to-clear).

**The errata, and what this driver does with each.**

| Item (DB / DA) | What it says | Here |
|----------------|--------------|------|
| 2.14.1 / 2.13.1 | ASYNC input events are missed when CNTPRES != DIV1 (DB A4/A5, fixed on B0; DA every revision) | NOT refused (it is fixed on B0): documented in the header, the workaround named - divide with SYNCPRES instead. NOT REPRODUCED on this die, see below |
| 2.14.2 / 2.13.2 | on ANY alternate route CMPAEN gates ALL FOUR outputs, no workaround (DB A4/A5, fixed on B0; DA every revision) | NOT refused (rev-dependent): stated loudly in the route table and here. MEASURED: on ALT2 a WOB-only configuration drives nothing at all |
| 2.14.3 / 2.13.3 | input mode 7 (WAITSW) does not work with CMPASET = 0 or in Dual Slope mode - EVERY revision of both families | REFUSED, at compile time by `init<cfg>()` and at run time by `init(cfg)`. Neither half could be provoked on this die, see below |
| CLKCTRL 2.5.3 / 2.4.1 | PLLS never sets with RUNSTDBY = 1 and no requester | MEASURED (see [clkctrl.md](clkctrl.md)) |
| CLKCTRL 2.5.4 (DB only) | the PLL will not run from an XOSCHF CRYSTAL, only from an external clock | `Pll::start` REFUSES that combination and writes nothing |

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `TcdConfig` | `route` (`TcdRoute`: def, alt1, alt2, alt3); `clock` (`TcdClock`: oschf, pll, extclk, clkper), `sync_prescaler` (`TcdSyncPrescaler` div1/2/4/8), `count_prescaler` (`TcdCountPrescaler` div1/4/32); `waveform` (`TcdWaveform`: one_ramp, two_ramp, four_ramp, dual_slope); the four compares `compare_a_set`/`compare_a_clear`/`compare_b_set`/`compare_b_clear`; `compare_override`, `auto_update`, `fifty_percent`, `wo_c`/`wo_d` (`TcdWaveformSelect`), `compare_a_value`/`compare_b_value` (CTRLD nibbles); `input_a`/`input_b` (`TcdEventInput`); `enable_woa..wod` and `fault_woa..wod`; `delay` (`TcdDelaySelect`: off, input_blanking, output_event), `delay_trigger`, `delay_prescaler`, `delay_value`; `dither_select`, `dither`; `debug_run`, `fault_on_debug`, `enable` |
| `TcdEventInput` | `enable` (TRIGEI), `action` (`TcdEventAction`: fault, capture), `rising` (EDGE), `config` (`TcdEventConfig`: neither, filter, async), `mode` (`TcdInputMode`: none, jump_wait, exec_wait, exec_fault, freq, exec_dead_time, wait, wait_sw, edge_trig, edge_trig_freq, level_trig_freq) |
| `Tcd<0>` configuration | `init<cfg>()` / `init(cfg)` -> bool (disables under ENRDY, writes every static register with the peripheral down, loads the double buffers, claims the enabled output pins, enables under ENRDY), `release()` (down, every output released in FAULTCTRL, the pins back to PORT inputs, the route back to DEFAULT) |
| discipline 1 | `enable()`, `disable()`, `enabled()`, `enable_ready()`, `wait_enable_ready(spins)` |
| discipline 2 | `command_ready()`, `wait_command_ready(spins)`, `sync()`, `sync_at_end()`, `restart()`, `software_capture_a()`/`_b()`, `disable_at_end()` |
| discipline 3 (static, refused while enabled) | `waveform(w)` / `waveform()`, `clock(c, s, p)` / `clock()`, `input_mode_a(m)` / `_b(m)`, `event_input_a(e)` / `_b(e)`, `output_control(override, auto_update, fifty, c, d)`, `output_values(a, b)`, `delay(sel, trig, pre, value)`; `fault_control(...)` (CCP) is writable any time but only means anything while the TCD is down |
| double buffers | `compare_a_set(v)`/`compare_a_clear(v)`/`compare_b_set(v)`/`compare_b_clear(v)` and their readbacks (values masked to 12 bits), `dither(v)` / `dither()` |
| captures | `capture_a()`, `capture_b()` (low byte then high byte, the pair performed inside) |
| status, flags, interrupts | `take_pwm_activity()` / `clear_pwm_activity()`, `ovf_flag`/`trig_a_flag`/`trig_b_flag`, `clear_ovf`/`clear_trig_a`/`clear_trig_b`/`clear_flags`, `enable_ovf_interrupt`/`enable_trig_a_interrupt`/`enable_trig_b_interrupt`, ISR bodies `ovf()` (TCD0_OVF_vect) and `take_triggers()` (TCD0_TRIG_vect) |
| events | `input_a_on(channel)` / `input_b_on(channel)` / `input_a_off()` / `input_b_off()`; the types `CmpBClrEvent`, `CmpASetEvent`, `CmpBSetEvent`, `ProgEvent`, `InputA`, `InputB` |
| readback | `route()`, `claimed_outputs()`, `input_blanking_enabled()`, `output_event_enabled()`, `delay_cycles()`, `fault_control()` |
| helpers | `tcd_route_exists`, `tcd_pin(route, output)`, `tcd_package_pins`, `tcd_sync_divisor`/`tcd_count_divisor`/`tcd_delay_divisor`, `tcd_sync_hz`/`tcd_counter_hz`, `tcd_cycle_ticks`, `tcd_input_mode_valid`, `tcd_dither_cycle_cost`, `tcd_config_valid`, `tcd_compare_max` |
| `TcdPwm<route>` | `init(clock, TcdPwmConfig)` -> bool, `rebase(hz)` (a ClockUser), `duty(ticks)` -> bool / `duty()`, `max()`, `period_ticks()`, `dead_ticks()`, `counter_hz()`, `cycle_hz()`, `sync()`, `sync_at_end()`, `start()`, `stop()`, `stop_at_end()`, `release()` |

`tcd_config_valid()` is the one place the legality lives: the route
must exist on this package, every ENABLED output must have a bonded
pin, both input modes must be valid for the waveform mode, input mode
7 needs a non-zero CMPASET, dithering is refused in Dual Slope mode,
and no compare may exceed 12 bits. `init<cfg>()` turns each of those
into its own diagnostic; `init(cfg)` returns false and writes nothing.
The runtime compare setters MASK to 12 bits instead of refusing - they
are the hot path of a PWM loop.

## How to use it

**A complementary pair with dead time** - the task, and the reason
this timer exists:

```cpp
#include "avrdx/tcd.hpp"
using Pwm = brio::TcdPwm<brio::TcdRoute::def>;          // WOA on PA4, WOB on PA5
Pwm::init(clock, {.clock = brio::TcdClock::clkper,
                  .hz = 20'000, .dead_time_ticks = 60, .auto_update = true});
Pwm::duty(Pwm::max() / 4);        // with auto_update, it lands at the end of the cycle
```

**The same pair above CLK_MAIN**, clocked by the PLL (the PLL's only
consumer; OSCHF must be at 16-24 MHz first, and its rate is what the
task is told):

```cpp
brio::Oschf::set_hz(24'000'000);
brio::Pll::start(brio::PllSource::oschf, brio::PllMultiplier::x2);   // 48 MHz
Pwm::init(clock, {.clock = brio::TcdClock::pll, .source_hz = 48'000'000,
                  .hz = 100'000, .dead_time_ticks = 24});
```

**A fast emergency stop with no CPU in the path** - an event input in
mode 4 deactivates both outputs to their FAULTCTRL levels for as long
as the event is active, and ASYNC makes it immediate:

```cpp
brio::EventChannel<2>::source(brio::EvAcOut<0>{});      // a comparator, say
brio::Tcd<0>::init({.clock = brio::TcdClock::clkper,
                    .compare_a_set = 60, .compare_a_clear = 600,
                    .compare_b_set = 660, .compare_b_clear = 1199,
                    .input_a = {.enable = true, .config = brio::TcdEventConfig::async,
                                .mode = brio::TcdInputMode::freq},
                    .enable_woa = true, .enable_wob = true});
brio::Tcd<0>::input_a_on(brio::EventChannel<2>{});
```

**Reading the counter** - it lives in another clock domain, so it is
captured, never read:

```cpp
Tcd<0>::software_capture_a();          // strobe, then wait for CMDRDY (both inside)
const uint16_t now = Tcd<0>::capture_a();
```

**The chapter's PWM capture** (example 25-3): both event inputs on ONE
channel, opposite edges, and the two captures bracket the on-time.

```cpp
Tcd<0>::event_input_a({.enable = true, .action = brio::TcdEventAction::capture,
                       .rising = true, .mode = brio::TcdInputMode::none});
Tcd<0>::event_input_b({.enable = true, .action = brio::TcdEventAction::capture,
                       .rising = false, .mode = brio::TcdInputMode::none});
Tcd<0>::input_a_on(ch); Tcd<0>::input_b_on(ch);
const uint16_t on_time = Tcd<0>::capture_b() - Tcd<0>::capture_a();
```

**The two vectors**, bound by the app as always:

```cpp
ISR(TCD0_OVF_vect)  { brio::Tcd<0>::ovf(); ... }
ISR(TCD0_TRIG_vect) { const auto t = brio::Tcd<0>::take_triggers(); if (t.a) ... }
```

## Bench findings (`test_avr_tcd`, rev A5, 250/250, no wires)

The measurement rig: WOA..WOD on the DEFAULT route are read back as
pin EVENTS into TCB meters, the TCD's own CMPBCLR event is the cycle
counter, and PD3/PD4 driven from PORT are the two input-event sources.
For every formula test the TCD is clocked from CLK_PER with both
prescalers at DIV1, so one counter tick IS one CLK_PER tick and the
chapter's equations are checked in whole ticks.

**The waveform formulas are exact, with zero spread.** Every row of
the table above measured min = max = the predicted value over eight
consecutive cycles: one ramp 1200/300/499, two ramp 1400/399/699,
four ramp 1000/200/400, dual slope 1200/400/400 and 2000/300/1400.
Same for the ordering cases: CMPACLR > CMPBCLR gives WOA = CMPBCLR -
CMPASET = 1099, CMPACLR < CMPASET gives 799, CMPBSET < CMPASET gives
WOB = 1149 overlapping WOA's 300.

**Dual slope runs one tick longer than the chapter says**, at both
geometries tried: `2 x (CMPBCLR + 1)`, not `2 x CMPBCLR + 1`. WOB's
high time carries the same +1: `2 x (CMPBCLR - CMPBSET + 1)`.

**The dead times are directly measurable** through CMPOVR + CTRLD:
CMPAVAL = 0b0001 puts WOA high in dead-time A only and CMPBVAL =
0b0100 puts WOB high in dead-time B only. Two ramp measured DTA = 101
= CMPASET + 1 and DTB = 201 = CMPBSET + 1; four ramp 100 and 300.

**The prescalers multiply exactly**: SYNCPRES x CNTPRES over 1, 2, 4,
8, 4, 8, 32 and 128 gives a 300-tick cycle measured at 300, 600, 1200,
2400, 1200, 2400, 9600 and 38400 CLK_PER ticks.

**A TCD on OSCHF is immune to a main-clock rebase and a TCD on
CLK_PER is not** - both sides measured under a live 24 -> 12 -> 24 MHz
switch: on OSCHF the period stays 50 us and the tick count halves;
on CLK_PER the tick count stays 1200 and the period doubles to 100 us.
(OSCHF measured 1202 ticks against the crystal's 1200: -0.17 %.)

**The PLL, at last on the wire.** Every leg runs from the same
oscillator, so the ratios are the multipliers themselves, free of
OSCHF's own error. A 3200-tick TCD cycle measured in CLK_PER ticks:
OSCHF 16 MHz 4808, x2 2404, x3 1603; OSCHF 24 MHz 3205, x2 1602. That
is MULFAC 2x = 2.000, 3x = 2.999 and 2.001 - and 16 MHz x3 and 24 MHz
x2 land on the same 1603/1602 ticks, the same 48 MHz. PLLS sets within
one read of the TCD being enabled on the PLL, and reads 0 whenever
nothing requests it.

**CLKCTRL errata 2.5.3 and 2.5.4 both observed.** With RUNSTDBY = 1
and the TCD down, PLLS stays 0 through 65535 polls; enabling the TCD
on the PLL sets it immediately. With this board's XOSCHF in crystal
mode, `Pll::source_ok(xoschf)` is false and `Pll::start(xoschf, ...)`
returns false leaving PLLCTRLA untouched.

**TCD errata 2.14.2 is real and route-specific.** On the DEFAULT route
a CMPBEN-only configuration drives WOB normally (423 edges in 20 ms).
The same configuration on ALT2 drives NOTHING (0 edges, the pad held
by its pull-up); adding CMPAEN brings WOB back (402 edges).

**Capture.** Software captures 2 ms apart advance by 188 counter ticks
at 93750 Hz, to the count - after the first one or two intervals,
which run twenty-odd counts long. A capture register is BLOCKED for a
new value until its high byte is read, so the first read of a fresh
sequence hands back whatever it was blocked on (the suite's priming
read returned 203, a value captured in the previous test): a capture
sequence needs one discarded read. The chapter's PWM-capture example
works exactly as printed - both inputs on one channel with opposite
edges gave CAPTUREA = 203 for CMPASET = 200 and CAPTUREB = 803 for
CMPACLR = 800, four times running: the SAME +3 counter-tick offset on
both (the pin event's trip through EVSYS and the capture
synchronizer), so their difference is the on-time to the tick, 600,
agreeing with the pulse-width meter.

**Input modes.** Mode 0 leaves the counter alone and still captures.
Mode 4 stops both pins dead while the level is held and keeps the
cycle rate (17 vs 16 cycles per 20 ms). Mode 10 stops WOA alone and
leaves WOB running. Modes 8 and 9 cut a 12000-tick on-time to 3032 on
the cycle that takes the edge; mode 9 leaves the cycle count untouched
where mode 8 does not. Mode 7 halts the counter (0 cycles per 20 ms),
STAYS halted after the input is released, and only a RESTART strobe
brings it back.

**STATUS.PWMACT follows the waveform generator, not the pad.** With
mode 4 holding both pins provably still for 12 ms, PWMACTA/B still
read 1 after a clear; with the TCD disabled the same clear holds at 0.
The W1C is sound - the detector simply watches the generated waveform
behind the fault override. To ask about the pad, read the pin.

**The asynchronous override is immediate.** With CLK_TCD_SYNC =
CLK_PER / 8, the synchronous path takes 14 CLK_PER ticks from the PORT
store that raises the level to WOA's falling edge; ASYNCON takes -3 -
the override lands before the counter read that follows the store. The
differential is 17 ticks, about two synchronizer cycles, which is the
chapter's "two/three clock cycles on the TCD synchronizer clock".

**The digital filter and input blanking work as specified.** With
CLK_TCD_CNT = 93750 Hz (four cycles = 43 us) a 5 us pulse is rejected
and a 400 us pulse passes. A blanking window of 1200 CLK_TCD_SYNC
cycles (400 us) triggered from CMPASET swallowed 4 out of 4 events
placed inside it and honoured 4 out of 4 placed after it.

**Dithering is exact.** One ramp, DITHERSEL = on-time B, DITHER = 8:
32 consecutive cycles summed to 19216 ticks = 32 x 600 + 16, and the
individual cycles are 600 or 601 - the accumulator adds exactly
8/16 of a tick per cycle. DITHERSEL = dead-time B on the same
geometry: the 32 cycles sum to exactly 32 x 600 and it is on-time B
that alternates 399/398, which is table 25-7's "0 additional cycles,
compensated by shortening the following output state" measured.

**The output plumbing.** CMPOVR + CTRLD inverts WOA (900 = cycle -
on-time) while leaving WOB alone; WOC and WOD carry whichever waveform
CMPCSEL/CMPDSEL name, and swapping the two selectors swaps the two
measured on-times. A held fault drives each pin to its FAULTCTRL CMPx
level, both ways round. DISEOC takes the TCD down at the end of the
cycle with the output stopping cleanly and ENRDY open again
afterwards. AUPDATE is exactly what the register says: a new CMPACLR
alone changes nothing (WOA stays 300 ticks over 5 ms), and writing
CMPBCLR - even to the value it already had - is what makes the pair
land (800 ticks).

**The two vectors.** 100 CMPBCLR events in 100 ms at a 1 kHz cycle and
100 OVF interrupts, one per cycle; 10 edges on input A gave exactly 10
TRIGA interrupts and 6 on input B exactly 6 TRIGB; with the TCD
disabled and the flag cleared, zero interrupts in 20 ms - the flags do
not storm.

**The synchronization windows are short.** With the TCD clocked from
CLK_PER, both ENRDY (after an ENABLE write) and CMDRDY (after a SYNC
strobe) read closed on the very next instruction and open again within
one further read: the discipline is real but costs nothing at this
rate. It is at a slow CLK_TCD that the bounded waits earn their keep,
which is why every verb that touches them returns a bool.

## Not covered yet

Driver gaps:

- **Usage types beyond the complementary pair.** `TcdPwm` is the one
  task built. The four-ramp bridge drive, the dual-slope resonant
  converter and a capture instrument over the two capture units are
  deliberately NOT built: they are born with their first user (the
  Multislope application is the expected one). Everything they need is
  on the resource.
- **Pin-level bonding inside an existing port** is encoded from the
  device headers' own route comments (48-pin ALT1 and 28-pin ALT2 stop
  at WOB); anything finer than that waits for the per-family device
  tables, as everywhere else in this stratum.

Implemented but not bench-verified:

- **`TcdClock::extclk`.** The board boots from a crystal on PA0/PA1
  and the TCD's external-clock input has no wiring of its own here.
- **The ALT1, ALT2 and ALT3 routes as waveform sources.** ALT1's pin
  claim and release are verified and ALT2 carries the errata 2.14.2
  measurement, but no waveform is measured on PB4/PB5 or PF0..PF3
  (the console owns PF4/PF5 and the desk's LED sits on PF2); ALT3 is
  PORTG, absent from the 48-pin bench chip.
- **The programmable output event (PROGEV)** and its DLYVAL delay. The
  other half of the same register - input blanking - is bench-verified,
  and the generator is in `evsys.hpp`, but nothing measures PROGEV.
- **Input modes 1, 2, 3, 5 and 6.** Exposed and validity-checked
  against table 25-5; their signatures need per-state discrimination
  that the wireless rig does not have, so none of them is measured.
- **`fifty_percent` together with `auto_update`** (25.5.3 says a write
  to either CLR high byte then requests the sync); FIFTY's mirroring
  is measured, that interaction is not.
- **Errata 2.14.1 could NOT be provoked on this die.** With CFG =
  ASYNC and CNTPRES at 4 and at 32, sixteen events of every length
  down to a back-to-back set/clear (~125 ns) reached BOTH the capture
  path (16/16 TRIGA) and the output-override path (8/8 on-times
  blocked in input mode 9). The erratum lists rev. A5 as affected and
  the bench chip is A5; the driver carries the item and names the
  workaround, and this page records the result as NOT REPRODUCED, not
  as absent.
- **Errata 2.14.3 could not be provoked either.** With mode 7 and
  CMPASET forced to 0 through the raw verbs, the halt worked, it
  survived the input being released, and the RESTART strobe brought
  the counter back - the same behaviour as with CMPASET = 300. The
  refusal STANDS anyway: the item is listed for every revision of both
  families, including B0, and a driver that only works on the die in
  front of it is the failure mode this project is written against.
- **Errata 2.14.2 on B0 silicon.** It is fixed there; this bench
  cannot show the fixed behaviour, which is precisely why the driver
  does not refuse alternate routes.
- **DBGRUN and FAULTDET.** Both need a CPU halted in an OCD session;
  neither is reachable from a running suite.
- **Sleep.** The TCD runs in Idle and stops in Standby and Power-Down
  (25.3.6); the sleep pass will cover it, together with RUNSTDBY on
  the clock sources the TCD requests.
