# AC - the analog comparators (AVR DA/DB)

> **PROVISIONAL.** The register option space is fully exposed with
> the port-level input guards in place (the PORTE positives refused
> where the package lacks them), and pin-vs-pin, all three
> instances, INVERT, both OUT pins and the window senses are
> bench-verified; what remains is pin-level bonding and the analog
> timing measurements - the gaps are in "Not covered yet". Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B (AC
> chapter 32, electricals 39.17, PORTMUX 17.3.10, EVSYS 16 generators
> 0x20-0x22, I/O multiplexing chapter 3), errata DS80000915F (no AC
> silicon erratum; the data sheet clarification 3.7.4 adds
> tDACREF). Complements: TB3211 "Getting Started with AC" (see
> [vendor/README.md](vendor/README.md)). Driver: `avrdx/ac.hpp`
> (`Ac<n>`, `Threshold`, `Window`); the reference is `Vref::ac` in
> `avrdx/vref.hpp` ([vref.md](vref.md)); the OUT event in
> `avrdx/evsys.hpp`. Reference test: `test_avr_timer` (test m).

## What the silicon does

Three comparators, AC0..AC2. Each compares a positive input (one of
four analog pins) with a negative input (one of three pins or
`DACREF`, an 8-bit divider of the comparators' shared reference:
V = DACREF x VREF / 256, VREF = `Vref::ac`), with selectable
hysteresis (none / 10 / 25 / 50 mV typ.) and three power profiles
(response 85 / 250 / 460 ns typ. rising at VDD/2).
The output is a level: `STATUS.CMPSTATE`, an event generator (0x20 +
n, async level), a pin (`OUTEN`: PA7 or ALT1 PC6 for all three - the
PA7 default collides with CLKOUT), the CCL input menu (ACn OUT on LUT
input n, [ccl.md](ccl.md)) and one interrupt per instance (`CMP`,
sense both / negative / positive edge). `INVERT` flips the output;
`INITVAL` sets it during start-up (the AC takes an unspecified
start-up time after enable - 39.17 gives none; a DACREF change
settles in tDACREF = 25 us, errata document clarification 3.7.4, with
the bandgap already running). `RUNSTDBY` keeps it alive in standby:
32.3.5 says event, interrupt and pin keep working without CLK_PER,
but the CTRLA field description (32.5.1) says interrupts and status
flags are NOT updated with the clock stopped - the datasheet
contradicts itself and the bench has not yet decided.

Input pins, identical on the 48- and 64-pin parts of both families
(chapter 3; on 28/32-pin parts the PORTE entries do not exist, and
on DB 28/32 neither does PD0 - see "Not covered yet"):

| | AINP0 | AINP1 | AINP2 | AINP3 | AINN0 | AINN1 | AINN2 |
|---|---|---|---|---|---|---|---|
| AC0 | PD2 | PE0 | PE2 | PD6 | PD3 | PD0 | PD7 |
| AC1 | PD2 | PD3 | PD4 | PD6 | PD5 | PD0 | PD7 |
| AC2 | PD2 | PD4 | PE1 | PD6 | PD7 | PD0 | PD7 |

PD6 (AINP3 on all three) is the DAC output pin, so the DAC can drive
any comparator from outside; the pins must be set as analog inputs
(input buffer off) in PORT.

**Window mode**: `CTRLB.WINSEL` pairs ACn with ACn+1 or ACn+2 (the
upper limit on the partner, the lower on ACn, both `MUXPOS` on the
SAME pin - the user configures both MUXCTRLs); `STATUS.WINSTATE`
reports above / inside / below and `INTMODE` (above / inside / below
/ outside) selects which state raises the interrupt and the event;
`CMPSTATE` is then "the window state matches INTMODE".

The errata document has no AC silicon erratum (its clarification
3.7.4 is the tDACREF figure above). Electricals: offset +-5 mV typ.
(15 max), input leakage 5 nA, CMRR 70 dB, input range -0.2 V .. VDD.

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `AcConfig` | `positive` (`AcPos`: ainp0..3), `negative` (`AcNeg`: ainn0..2, dacref), `reference` (`Ref`, VREF.ACREF) + `reference_always_on`, `dacref` (code), `hysteresis` (`AcHysteresis`: none, small, medium, large), `power` (`AcPower`: fast, medium, low), `invert`, `init_high`, `output_pin`, `alt_pin` (PC6 instead of PA7), `run_standby`, `sense` (`AcSense`: both, falling, rising) |
| `Ac<n>` | `init<cfg>()` / `init(cfg)` (disable, reference, the pins to analog, mux/DACREF/sense/route, enable; the compile-time form static_asserts `ac_config_valid`, the run-time form returns false, touching nothing, for an input pin this package does not bond), `enable`/`disable`, `state`, `dacref(code)`, `threshold_mv(mv, ref_mv)`, `sense`, `enable_interrupt`, `flag`/`clear_flag`, `cmp` (ISR body: the state, flag cleared), `window<partner>(AcWindowSense: above, inside, below, outside)`, `window_off`, `window_state` (`AcWindowState`: above, inside, below); `OutDefault`/`OutAlt` pin types, `OutEvent` generator |
| `Threshold<Ac>` | `init(pin, mv, ref, ref_known_mv, hysteresis, sense, interrupt)`, `set_mv`, `mv`, `above`, `crossed` (ISR body) |
| `Window<Lower, Upper>` | `init(pin, low_mv, high_mv, when, ref, ref_known_mv, hysteresis, interrupt)`, `state`, `changed` (ISR body, Lower's vector) |
| helpers | `ac_dacref_code(mv, ref_mv)`, `ac_dacref_mv(code, ref_mv)` (thin names over util/analog.hpp's `dac_code`/`dac_mv`, steps = 256), `ac_pos_pin(n, pos)`, `ac_neg_pin(n, neg)` (the family pin table), `ac_config_valid(n, cfg)` |

## How to use it

A threshold on PD6 (the DAC pin, AINP3 of every comparator) at 1.000 V:

```cpp
#include "avrdx/ac.hpp"
using Level = brio::Threshold<brio::Ac<0>>;
Level::init(brio::AcPos::ainp3, 1000, brio::Ref::v2048);     // medium hysteresis, both edges
ISR(AC0_AC_vect) { post<Monitor>(Crossed{Level::crossed()}); }
```

A window 0.5 .. 1.5 V with the interrupt when the signal leaves it:

```cpp
using Band = brio::Window<brio::Ac<0>, brio::Ac<2>>;
Band::init(brio::AcPos::ainp3, 500, 1500, brio::AcWindowSense::outside, brio::Ref::v2048);
ISR(AC0_AC_vect) { post<Monitor>(Band{Band::changed()}); }
```

The resource directly - two pins compared, output on PC6 and as an
event for a timer capture:

```cpp
using C = brio::Ac<1>;
C::init<brio::AcConfig{.positive = brio::AcPos::ainp1, .negative = brio::AcNeg::ainn0,
                       .hysteresis = brio::AcHysteresis::small, .output_pin = true, .alt_pin = true}>();
brio::EventChannel<6>::source(C::OutEvent{});
```

## Bench findings

`test_avr_timer` test m (A5, 3.3 V), the DAC on PD6 driving AINP3 of
AC0 and AC2 (no wire: the pin is shared): Threshold at 1000 mV (ref
2.048 V, DACREF 125, medium hysteresis) reads below at 500 mV and
above at 1500 mV with one interrupt on the crossing; sweeping the DAC
1 mV at a time the output **rises at 1007 mV and falls at 990 mV**:
centre +0.5 mV (DAC + comparator offsets cancel to within the
DAC's LSB), **hysteresis 17 mV** for the medium setting (spec 25 typ.);
Window 500..1500 mV on AC0 + AC2 reads below / inside / above at 300 /
1000 / 1800 mV.

Pin-vs-pin and the rest, with no wires (the positive is AINP3 = PD6,
the DAC's pin; the negative a GPIO the test drives): AC0 and AC1
compare a 1.5 V positive against a toggled negative correctly, and
INVERT flips the same comparison; AC1's OUT event is consumed by a
pulse counter (3 crossings = 3 counts); the OUT pin follows the
comparison on PA7 (default; CLKOUT off) and on PC6 (ALT1, AC2); each
window sense (below / inside / above) raises exactly ONE interrupt on
entering its state.

## Not covered yet

Driver gaps:

- Pin-level bonding within a bonded port (PD0 = AINN1 on DB 28/32,
  PC6 = the ALT1 output on 28/32-pin parts): the port-level guards
  cannot see it - deferred to the family device tables.
- `Window` cannot move its limits after init (the resource can, via
  each comparator's `dacref`).

Implemented but not bench-verified:

- Response time and start-up (a TCB capturing the OUT event against
  a DAC step is swamped by the DAC's own slew - a faster edge is
  needed); the three power profiles; the small/large hysteresis
  widths; tDACREF; `INITVAL` (its window is the start-up itself).
- RUNSTDBY under a real standby (queued, LOW priority - it would
  also settle the 32.3.5 vs 32.5.1 contradiction above).
