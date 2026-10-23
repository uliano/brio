# AC - the analog comparators (AVR DA/DB, AVR EA)

This document is for a programmer who needs to know when an analog
voltage crosses a level, or on which side of a band it sits, without
running a conversion: a threshold detector, a window detector, or a
comparator output on a pin or on an event channel for a timer or the
CCL. `Ac<n>` is the resource, one comparator: an `AcConfig` describes
it and one function per register fact drives it. It is a monostate, a
class whose state is static, with no instances: a program names the
type and never makes an object of it. `Threshold<Ac>` and
`Window<Lower, Upper>` are the tasks over it (a task, in brio, is a
driver layer with state of its own over a resource, served from its
peripheral's interrupt or called by an active object; an active object
is a component of a brio program with its own event queue, served one
event at a time by the kernel, [../learn/introduction.md](../learn/introduction.md)).
`Threshold` sets the comparator's internal reference level, DACREF,
from millivolts and reports each crossing; `Window` pairs two
comparators on one pin and reports above, inside or below.

The smallest use, a threshold at 1.000 V on PD6, trimmed from "How to
use it" below:

```cpp
#include "avrxt/ac.hpp"
using Level = brio::Threshold<brio::Ac<0>>;
Level::init(brio::AcPos::ainp3, 1000, brio::Ref::v2048);     // medium hysteresis, both edges
ISR(AC0_AC_vect) { post<Monitor>(Crossed{Level::crossed()}); }
```

Line by line: `Level` names the threshold task over comparator AC0,
which every part of the three dies has. `init()` takes the positive
input (`ainp3`, PD6 on every comparator), the threshold in millivolts
and the reference: it writes VREF.ACREF to the internal 2.048 V
reference, sets the negative input to DACREF with the code nearest
1000 mV (125), turns PD6's digital input buffer off, and with its
defaults selects the medium hysteresis, an interrupt on both crossing
directions and the CMP interrupt enabled. It returns false, writing no
register, when the reference's millivolts are unknown (`Ref::vdd` or
`Ref::vrefa` without the `ref_known_mv` argument), when the threshold
is above the reference, or when this package does not bond the input's
pin; the excerpt ignores that answer. The `ISR()` line binds the
comparator's vector, with avr-libc's macro: `crossed()` clears the CMP
flag and returns the new state (true = above), and `post<Monitor>`
hands it to an active object; `Monitor` and its event `Crossed` are
the program's own names.

What decides correctness even there. The vector is the program's to
bind: the driver binds none and nothing checks that `AC0_AC_vect` has
a handler while the interrupt is enabled, and the CMP flag stays set
until the body clears it, which `crossed()` does. Interrupts must be
enabled globally (`sei()`) before a crossing can be served. VREF.ACREF
is one register for all the comparators: every `init()` whose negative
input is DACREF writes it, so a second comparator initialized with
another `Ref` moves the first one's threshold, and the first
`Threshold`'s `mv()` then reports a level computed for the old
reference. `init()` waits for nothing: on the DB a DACREF change settles in
tDACREF = 25 us with the bandgap already running (DS80000915F 3.7.4),
the comparator's start-up after enable has no figure (DS40002247B
39.17 gives none), and a `state()` read right after
`init()` may still show the start-up value INITVAL holds.

## Documents of record

AVR128DB28/32/48/64 data sheet DS40002247B (AC
chapter 32, electricals 39.17, PORTMUX 17.3.10, EVSYS 16 generators
0x20-0x22, I/O multiplexing chapter 3), errata DS80000915F (no AC
silicon erratum; the data sheet clarification 3.7.4 adds tDACREF);
AVR64EA28/32/48 data sheet DS40002443A (chapter 30, I/O multiplexing
chapter 3), errata DS80001048D (no AC silicon erratum; its
clarification 3.8 fills table 35-27). One IP on the three dies.
Complements: TB3211 "Getting Started with AC" (see
[vendor/README.md](vendor/README.md)). Driver: `avrxt/ac.hpp` (`Ac<n>`,
`Threshold`, `Window`); the reference is `Vref::ac` in `avrxt/vref.hpp`
([vref.md](vref.md)); the OUT event in `avrxt/evsys.hpp`. Reference
test: `test_avr_timer` (test m). A letter is one command key of a
suite's console and the group of verdicts it runs: in
`test_avr_timer`, `m` judges the comparators against the DAC (the
threshold, its crossings, the window) and `w` the pin-vs-pin
comparisons, AC1, INVERT, the OUT pins and the window senses; this
suite's all-key is `a`, which runs every letter its image carries.

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
flags are not updated with the clock stopped. The bench sides with
32.3.5 for the interrupt (below); whether CLK_PER runs for the
comparator meanwhile is not measured, so the two sentences may yet
both hold.

**The AVR EA's** comparators are two, AC0 and AC1, with a fifth
positive pin (AINP4 = PC3) and a fourth negative (AINN3 = PC2) on both
and on every package, the DACREF negative at code 4 (the header's), two
power profiles (no third: `AcPower::low` refused), the window pairing
ACn with ACn+1 only (no UPSEL2), and the CCL's AC input 2 carrying AC1
(no AC2, [ccl.md](ccl.md)). Electricals (DS80001048D 3.8): offset +-10
mV typ., +-30 max; hysteresis 10 / 20 / 30 mV typ.; response 50 ns
rising, 150 ns falling at profile 0; CMRR 70 dB. Its DAC reaches the
comparators only through PD6 (AINP3) - no internal route
([dac.md](dac.md)).

Input pins, identical on the 48- and 64-pin parts of the three dies
(chapter 3; on 28/32-pin parts the PORTE entries do not exist, and
on DB 28/32 neither does PD0 - see "Not covered yet"; the EA's AINP4
and AINN3 are PC3 and PC2 on both its instances):

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
same pin - the user configures both MUXCTRLs); `STATUS.WINSTATE`
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
#include "avrxt/ac.hpp"
using Level = brio::Threshold<brio::Ac<0>>;
Level::init(brio::AcPos::ainp3, 1000, brio::Ref::v2048);     // medium hysteresis, both edges
ISR(AC0_AC_vect) { post<Monitor>(Crossed{Level::crossed()}); }
```

A window 0.5 .. 1.5 V with the interrupt when the signal leaves it:

```cpp
using Band = brio::Window<brio::Ac<0>, brio::Ac<2>>;
Band::init(brio::AcPos::ainp3, 500, 1500, brio::AcWindowSense::outside, brio::Ref::v2048);
ISR(AC0_AC_vect) { post<Monitor>(BandState{Band::changed()}); }
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

On the **AVR128DA48** (rev A7, 5 V; the whole of `test_avr_timer`, 197
verdicts, green): the crossings sit at **1005 mV up and 978 mV down**,
hysteresis 27 mV (the spec's 25 typ.), the window and pin legs as above
- the OUT pin on PA7 read as a level 2 ms after each DAC step (twelve
runs of twelve).

**RUNSTDBY in a real STANDBY** (`test_avr_sleep` o, 5 V): AC0 on AINP0 =
PD2 against DACREF, the pad driven by EVOUT from a PIT divider at 32 Hz.
With `run_standby` the CMP interrupt ends a 125 ms standby at the
first edge, the PIT backstop never firing; without it the comparator
sleeps and the PIT ends the sleep - in every run of twelve. On the
**AVR64EA48** (rev B1, ten runs of ten) the same, which settles the
EA's chapter in favour of 30.3.5 and table 13-3 (the AC runs in standby
with its RUNSTDBY, its interrupt a wake source) against table 13-1,
which leaves the AC off its standby list.

On the **AVR64EA48** (rev B1, 5 V, `test_avr_timer`'s second image, the
board's 96 nF net on PD6): the crossings sit at **999 mV up and 968 mV
down** (1003 / 970 in one run of twelve), hysteresis 31..33 mV for the
medium setting (the clarification's 20 typ.), the window, pin-vs-pin, INVERT, the OUT pins and the window
senses as above; AC1 on the CCL's input 2 counts its three crossings.
The comparators' DACREF0/1 as the ADC's negative input read their
level within 11 mV once sampled for 64 us ([adc.md](adc.md)).

## Not covered yet

Driver gaps:

- Pin-level bonding within a bonded port (PD0 = AINN1 on DB 28/32,
  PC6 = the ALT1 output on 28/32-pin parts): the port-level guards
  cannot see it - deferred to the family device tables.
- `Window` cannot move its limits after init (the resource can, via
  each comparator's `dacref`).

Implemented but not bench-verified:

- The AVR EA's AINP4 (PC3) and AINN3 (PC2): on its board both pins
  carry the I2C pair's dual-mode wires to the partner board, and
  nothing drives an analog level on them; a test there needs those
  wires lifted or a level from the partner.

- Response time and start-up (a TCB capturing the OUT event against
  a DAC step is swamped by the DAC's own slew - a faster edge is
  needed); the three power profiles; the small/large hysteresis
  widths; tDACREF; `INITVAL` (its window is the start-up itself).
- The OUT event and pin under a real standby (the interrupt is measured
  above): a counter that runs in standby on the event, or a peer reading
  the pin, would measure them.
