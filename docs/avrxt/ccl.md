# CCL - the configurable custom logic (AVR DA, DB and EA)

This document is for a programmer who wants logic done in hardware,
with no CPU: glue logic between pins, a timer's event divided by two, a
one-clock pulse on an edge, a filtered input that wakes the device from
sleep. The CCL is a set of look-up tables (LUTs), each a logic function
of three inputs - pins, event channels, other peripherals' outputs -
driving a pin, an event channel or the CCL's interrupt. `Ccl` is the
block and `Lut<n>` one table, both resources (one function per register
fact); a `LutConfig` describes a whole table and `Lut<n>::init()` writes
it. `ToggleFlipFlop<pair>` is a task over a pair of LUTs (a task, in
brio, is a ready-made shape over a resource that a program names and
runs): a flip-flop toggled by one event.

The smallest use, from "How to use it" below - PB3 = PB0 AND PB1 on a
48/64-pin AVR DA/DB, whose LUT4 sits on PORTB:

```cpp
using And = brio::Lut<4>;
brio::Ccl::disable();
And::init<brio::LutConfig{.in0 = brio::LutInput::pin, .in1 = brio::LutInput::pin,
                          .truth = brio::lut_truth([](bool a, bool b, bool) { return a && b; }),
                          .output_pin = true}>();
brio::Ccl::enable();
```

Line by line: `And` names LUT4 as a type; `Ccl::disable()` stops the
whole block, the state every configuration starts from; `init<...>()`
takes the configuration as a compile-time value, so an illegal one (an
edge detector with no filter, an output pin on a port the package does
not have, LUT3's missing ALT1) is a compile error - it makes inputs 0
and 1 the LUT's own pins (PB0, PB1), input 2 left at `mask` (low),
writes the truth table that `lut_truth()` computes from the lambda (bit
k is the output for input pattern k), and with `output_pin` makes PB3
an output carrying the result; `Ccl::enable()` starts the block.

What decides correctness even there. On the AVR DB's silicon A4 and A5
a LUT may be reconfigured only with the whole block disabled (errata
2.4.1, "What the silicon does" below), which is why the configuration
is bracketed by `disable()` and `enable()`; the AVR DA has no such
erratum, and on the AVR EA one LUT re-initialised under a running block
leaves the others undisturbed (measured on an AVR64EA48). A pair's sequencer is set
before its even LUT's `init()`, or the silicon ignores it. The driver
configures the output pin and nothing else of the port: the input pins
are read as PORT leaves them. LUT4 exists on the 48/64-pin AVR DA/DB
alone; on the other parts a lower LUT does the same on its own port
("Pins" under "What the silicon does"). The calls wait for nothing and mask nothing;
several of them read, modify and write registers the LUTs share, so the
wiring is made in setup or by the one active object that owns it (an
active object is a component of a brio program with its own event
queue, served one event at a time by the kernel,
[../learn/introduction.md](../learn/introduction.md)). An interrupt
sense other than `none` needs the program's `ISR(CCL_CCL_vect)` calling
`Ccl::take_flags()`.

The rest of this page, in order: the documents of record; what the
silicon does, the input menu and the facts that matter to code; the
types and verbs and the configuration protocol; how to use it; the
bench findings, the AVR64EA48's and the CCL as a wake-up source among
them; what is not covered.

## Documents of record

AVR128DB28/32/48/64 data sheet DS40002247B (CCL
chapter 31, PORTMUX 17.3.2, EVSYS 16 generators 0x10-0x15 / users
0x00-0x0B, I/O multiplexing chapter 3), errata DS80000915F (2.4.1,
2.4.2) and, for the DA parts, DS80000882C (2.3.1); for the AVR EA,
AVR64EA28/32/48 data sheet DS40002443A (CCL chapter 29, PORTMUX
17.3.2) and errata DS80001048D (no CCL item). Complements: TB3218 "Getting Started with CCL", AN2434, the
Microchip example `avr128db48-blink-led-ccl` (see
[vendor/README.md](vendor/README.md)). Driver: `avrxt/ccl.hpp` (`Ccl`,
`Lut<n>`, `ToggleFlipFlop<pair>`), the LUT event vocabulary in
`avrxt/evsys.hpp`. Reference test: `test_avr_timer` (test c; a letter
is one command key of the suite's console and the group of verdicts it
runs - `c`, `l`, `i`, `k` and the AVR EA's `x` are the CCL's, `a` runs
every letter the image carries).

## What the silicon does

One CCL with six look-up tables (LUT0..LUT5 on the 48/64-pin AVR DA/DB,
LUT0..3 on their 28/32-pin parts and on every AVR EA - the driver takes
the count from the device header) grouped in pairs (0-1, 2-3, 4-5),
each pair owning one sequencer. A LUT is: three inputs chosen from a menu, an
8-bit truth table (`TRUTHn[k]` = the output for input pattern k, IN2
the MSB), then optionally a synchronizer or a glitch filter, then
optionally a rising-edge detector (one-clock pulse), then the output
- to a pin, to the event system (level generator, async), to the
neighbouring LUT (LINK), to the CCL interrupt (one vector for all six,
per-LUT sense rising/falling/both) and, through the pair's sequencer,
back as FEEDBACK.

Input menu, the same for the three inputs of a LUT except that the
peripheral entries pick the instance by input index (`INSEL0/1/2`):
`MASK` (tied low), `FEEDBACK` (the pair's sequencer output),
`LINK` (LUT[n+1]'s direct output; LUT0 is linked into the last LUT),
`EVENTA`/`EVENTB` (the two event users of this LUT, 0x00 + 2n and
0x01 + 2n, no synchronization), `INn` (the LUT's own pin: IN0/IN1/IN2
= pins 0/1/2 of the LUT's port, below), `ACn` (AC0/1/2 OUT on
input 0/1/2), `ZCDn`, `USARTn TXD` (host modes only), `SPI0` (MOSI on
input 0 and 1, SCK on input 2; host mode only), `TCA0`/`TCA1` (WO0/
WO1/WO2 on input 0/1/2), `TCBn` (TCB0/1/2 WO on input 0/1/2), `TCD0`
(WOA/WOB/WOC). So "which timer output can feed which LUT input" is a
fixed table: TCB3 and TCA WO3..5 are not reachable except through an
event channel, and AC2 only on input 2.

The AVR EA's menu (DS40002443A 29.2.2.1) is the same IP's with its
peripherals: no ZCD and no TCD (those inputs are not names there), the
same codes for every kind it has - the driver takes them from the
header's enumerators and asserts the three inputs agree - and the AC
code on input 2 selects **AC1** (the part has no AC2): `lut_input_ac
(input)` says which comparator an input's AC code is, from the device
tables' probe of the header's INSEL2 enumerators. Two contradictions in
the EA's own documents are settled on the bench (findings): the
register summary 29.5.8 calls INSEL2's AC code reserved, and the
header's comments call SPI0 "MISO" on all three inputs.

Facts that matter to code:

- **Enable protection**: `LUTnCTRLA/B/C`, `TRUTHn` and `SEQCTRL` can
  be written only while the (even) LUT is disabled - together with
  ENABLE = 1 in the same write, never together with ENABLE = 0; a
  `SEQCTRL` written after the even LUT is enabled is silently ignored
  (measured: the JK never engages, the LUT outputs the bare truth
  table). On top of that, **errata 2.4.1 (DB rev A4/A5; fixed on B0;
  the DA has no such erratum)**: reconfiguring one
  LUT requires the whole CCL disabled (`CTRLA.ENABLE = 0`), which
  drops every other LUT meanwhile - a driver must build the complete
  configuration first and enable once; a runtime change of one LUT is
  a glitch on all of them. Changing a clock source also wants the CCL
  disabled. The AVR EA has no such erratum: its protection is the
  LUT's own (the even LUT's for a sequencer, DS40002443A 29.3.1.1), and
  one LUT re-initialised under a running block takes its new table at
  once with the others undisturbed (measured, below). The errata document's sleep clarification adds that the CCL
  interrupt wakes the device from power-down only when the LUT path is
  fully asynchronous (no filter, no edge detector); the bench refines
  that into a statement about the LUT's clock - see the findings.
- **Clock (`CLKSRC`)** per LUT, used by the filter, the edge detector
  and (from the even LUT) the sequencer: CLK_PER, the LUT's own
  TRUTHSEL[2] input (then input 2 is seen as low in the truth table),
  OSCHF (before the prescaler), OSC32K, OSC1K. `RUNSTDBY` keeps the
  chosen clock alive in standby; without it a filtered/sequenced LUT
  outputs 0 in standby (the bare truth table keeps working in idle).
- **Filter (`FILTSEL`)**: SYNCH = two clock delay, FILTER = an input
  must hold for more than two clocks, four clocks delay. The edge
  detector requires a filter option. Disabling a LUT clears its
  filter/edge logic one clock later.
- **Sequencer (`SEQSEL`)** per pair, clocked with the even LUT: D
  flip-flop (D = even, EN = odd, clocked by the even LUT's clock),
  JK (J = even, K = odd, toggles on 1/1), D latch (D = even, G =
  odd: the transparent one), RS latch (S = even, R = odd, 1/1
  forbidden). Disabling the even LUT clears the
  flip-flop asynchronously (reset held one clock). The sequencer
  output is what FEEDBACK returns and what the pair's output carries.
- **Pins** (PORTMUX `CCLROUTEA`, one bit per LUT moves only the
  OUT): LUTn inputs IN0/IN1/IN2 are pins 0/1/2 and OUT is pin 3
  (ALT1: pin 6) of port A/C/D/F/B/G for LUT0/1/2/3/4/5; LUT3 has no
  ALT1 (PF6 is RESET) and the driver refuses it; LUT5's port is
  PORTG (64-pin only) - elsewhere the LUT works on internal signals
  and events, and the driver refuses only its pin faces
  (`has_pins`/`has_alt_pin` are the facts; a re-init away from
  `output_pin` releases the previously taken pin). LUT1's pins PC0..PC3 coincide with TCA0's
  PORTC route WO0..3 (which of the two a board uses is the
  application's to decide, pad by pad). `OUTEN`
  drives the pin; the pin's direction must be set by PORT.
- **Errata 2.4.2**: LINK into LUT3 dead - on DB only rev A4 and the
  28/32-pin parts; on DA the same item is never fixed on 28/32-pin
  parts (all revisions).
- The CCL is a level generator (0x10 + n) for the event system, async;
  its event users take the channel without detection (a level or a
  pulse reaches the truth table as is).

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `LutConfig` | `in0`/`in1`/`in2` (`LutInput`: mask, feedback, link, event_a, event_b, pin, ac, zcd, usart_txd, spi0, tca0, tca1, tcb, tcd0 - the instance follows the input index), `truth` (TRUTHn; build it with `lut_truth(lambda of three bools)`), `filter` (`LutFilter`: none, sync, filter), `edge_detect` (needs a filter), `clock` (`LutClock`: clk_per, input2, oschf, osc32k, osc1k), `output_pin`, `alt_pin`, `interrupt` (`LutSense`: none, rising, falling, both) |
| `Ccl` | `enable(run_standby)`, `disable` (the state every reconfiguration starts from), `enabled`, `sequencer<pair>(Sequencer)` (none, d_flip_flop, jk_flip_flop, d_latch, rs_latch), `take_flags` (ISR body of CCL_CCL_vect: bit n per LUT, cleared) |
| `Lut<n>` | `init<cfg>()` / `init(cfg)` (block disabled: inputs, truth, route, pin, sense, then CTRLA with ENABLE), `enable`/`disable` (this LUT), `sense(LutSense)`, `flag`/`clear_flag`, `event_a_on(channel)`/`event_b_on(channel)`, the register accessors; `In0`/`In1`/`In2`/`OutDefault`/`OutAlt` pin types, `port`, `has_pins`/`has_alt_pin` (this package's facts); `OutEvent` generator, `EventA`/`EventB` users |
| `ToggleFlipFlop<pair>` | `init(toggle_channel, output_pin, alt_pin)`: JK on LUT 2p/2p+1, J = K = the event: one toggle per pulse (a divide-by-two, no CPU); `Even`/`Odd` |
| helpers | `lut_truth(f)`, `lut_config_valid`, `lut_port(n)`, `lut_input_ac(input)` |

The protocol, dictated by errata 2.4.1 and the enable protection:
`Ccl::disable()`, `Ccl::sequencer<p>(...)` (before the even LUT's
init - `sequencer` disables that LUT itself), every `Lut<n>::init(...)`,
`Ccl::enable()` - the whole wiring at once. On the AVR EA a lone
`Lut<n>::init(...)` under a running block is legal too (it disables and
re-enables that LUT alone), and the others keep running.
`lut_input_ac(input)` names the comparator an input's AC code selects
(AC2 on input 2 of the DA/DB, AC1 on the EA's). A CCL application is a fixed wiring: the
EventSystem static allocation would be its natural companion, which
[evsys.md](evsys.md) lists as a driver gap, not built.

## How to use it

Glue logic on pins - PB3 = PB0 AND PB1 (LUT4's port is PORTB):

```cpp
#include "avrxt/ccl.hpp"
using And = brio::Lut<4>;
brio::Ccl::disable();
And::init<brio::LutConfig{.in0 = brio::LutInput::pin, .in1 = brio::LutInput::pin,
                          .truth = brio::lut_truth([](bool a, bool b, bool) { return a && b; }),
                          .output_pin = true}>();
brio::Ccl::enable();
```

A timer event divided by two, on a pin and as an event, with no CPU:

```cpp
using Half = brio::ToggleFlipFlop<0>;              // LUT0/LUT1, output LUT0 = PA3
brio::EventChannel<4>::source(brio::EvTcbCapt<1>{});
brio::Ccl::disable();
Half::init(brio::EventChannel<4>{}, true);          // JK toggled by TCB1's CAPT
brio::Ccl::enable();
brio::EventChannel<5>::source(Half::Even::OutEvent{});   // the square wave as an event
```

A one-clock pulse from an event's rising edge (sync + edge detector),
and the CCL interrupt:

```cpp
using Edge = brio::Lut<2>;
Edge::init({.in0 = brio::LutInput::event_a, .truth = brio::lut_truth([](bool a, bool, bool) { return a; }),
            .filter = brio::LutFilter::sync, .edge_detect = true, .interrupt = brio::LutSense::rising});
Edge::event_a_on(brio::EventChannel<3>{});
ISR(CCL_CCL_vect) { const uint8_t who = brio::Ccl::take_flags(); ... }
```

## Bench findings

`test_avr_timer` tests l and i (A5, and green again on the AVR128DA48,
rev A7, whose 48 pins keep LUT3's LINK - DS80000882C 2.3.1 breaks it on
28/32 pins only): LINK verified (LUT4 passes
LUT5's constant both ways onto PB3); the DFF follows a 1 kHz D
exactly (EN high, CLK_PER clock); the RS latch sets and resets from
two software channels, read on LUT2's pin; LUT1's ALT1 drives PC6
and a re-init without the pin releases its direction. TCA0 WO0 as a
LUT input reads 24000 ticks exact; AC0 as a LUT input counts 3 DAC
crossings; the filter delays measured differentially (the stamp
technique): sync +2, filter +4 CLK_PER exact, and the same filter on
OSC32K delays ~4 cycles (2871 CLK_PER measured) - the low-power
debouncer is real.

`test_avr_timer` test c (A5): LUT4 as AND and then OR of PB0/PB1 on
PB3 follows the truth table on the pins (reconfigured with the block
disabled, per 2.4.1); the JK flip-flop on LUT0/1 with J = K = a TCB's
1 kHz CAPT event toggles exactly once per one-CLK_PER pulse (500 Hz on
LUT0's output event, measured 48000 ticks); a LUT with sync + edge
detector on the same event yields one pulse per event (200 in 200 ms,
counted by a TCB clocked by LUT4's output event - a one-CLK_PER pulse
is a valid COUNT input); `SEQCTRL` must be written before the even LUT
is enabled (see above).

`test_avr_timer` test k (A5, no wires), every run of twelve: the
peripheral inputs - TCB2's WO on input 2 (the Pwm8 period, 256 ticks,
through the LUT), USART1's TXD on input 1 (four 0x55 frames, five
rising edges each: 20), SPI0's SCK on input 2 (a pinless host, 32
rising edges for four bytes); the D LATCH on pair 0, read on EVOUT
PD2: Q follows D while G is high and holds 0 and 1 while G is low;
FEEDBACK closing a DFF on itself (D = NOT Q, EN constant 1) divides its
clock by two - clocked from the LUT's own input 2 (TCA0's 1 kHz square
as event B) it runs at 500 Hz exactly (48000 ticks), from OSC32K its
period is 1451..1454 ticks and from OSC1K 46439..46509 - the 32:1 of
the two clocks within the capture's ticks and the oscillator's wander
between the two legs.

On the **AVR64EA48** (rev B1, 5 V, 20 MHz quartz; `test_avr_timer`,
twenty runs of twenty) with its four LUTs - letter c's pin LUT is
LUT2 (PD0/PD1 -> PD3), the pinless one LUT3, LINK LUT3 -> LUT2 -
every CCL verdict of the DA/DB holds: the truth tables on pins, the JK
toggle (40000 ticks, 500 Hz), the edge detector (one pulse per event),
the DFF and RS sequencers, LUT1's ALT1 on PC6, TCA0 WO0, AC0, TCB2,
USART1's TXD and SPI0's SCK as inputs, the D latch, FEEDBACK, the
filter delays (sync +2, filter +4 CLK_PER) and the OSC32K filter (1917..
2470 ticks, 3 to 4 cycles of this die's 618-tick OSC32K cycle). And the
EA's own (letter x):

- **Input 2's AC code is AC1**, as 29.2.2.1 and the header say and the
  register summary 29.5.8 ("reserved") does not: AC1 crossing the DAC
  three times gives three rising edges through LUT2's input 2 with AC0
  off.
- **SPI0 on inputs 0 and 1 is MOSI**, as the data sheet says and the
  header's "MISO" comments do not: a pinless host sending four 0x55
  gives 16 rising edges through each (MOSI rises four times a byte;
  SCK would give 32, an undriven MISO none); on input 2 it is SCK (32,
  letter k).
- **LINK reaches LUT3**: LUT3 passes LUT0's constant both ways (the DA/
  DB's 28/32-pin erratum has no EA twin).
- **One LUT reconfigures under a running block**: LUT2 re-initialised
  fifty times with the block enabled shows each new table at once on
  EVOUT PD2, while LUT0, passing TCA0's 1 kHz WO0, rises exactly once
  per two CMP0 events across the whole sequence (52 against 103) - no
  gap, no glitch the counters see.
- The OSC32K-clocked DFF of letter k needs the mean of 32 periods here:
  this die's OSC32K jitters by a few CLK_PER from cycle to cycle
  ([rtc.md](rtc.md)), and one two-cycle period, multiplied by 32 against
  OSC1K, read up to 132 ticks off the ratio; the mean (rounded to the
  tick, so +-16 once multiplied) reads within 29 in twenty runs.

### The CCL as a wake-up source

`test_avr_sleep` test n (A5, no wires): a PIT divider through the event
system is the one signal that survives every sleep mode, so a LUT fed
from it and interrupting on its own output edge is a wake-up source the
board builds for itself. A 1 s PIT interrupt is the backstop, and which
source ENDED each sleep is read off the counts taken inside the sleep.

- **From STANDBY, a clocked and FILTERED LUT wakes the device** with
  `Ccl::enable(true)` (RUNSTDBY): the backstop never fires.
- **From POWER-DOWN, the fully asynchronous path wakes it** - FILTSEL =
  0, EDGEDET = 0 - which is the errata clarification's own case.
- **AND SO DOES A FILTERED LUT CLOCKED FROM OSC32K.** The restriction is
  really about which clock the LUT was given: the 32 kHz domain is
  exactly what power-down keeps running for the PIT, so a filter on it
  keeps filtering. The same LUT clocked from CLK_PER does NOT wake -
  that clock is gone and the backstop ends the sleep, with one CCL
  interrupt following once CLK_PER is back and the frozen filter catches
  up.
- **On the AVR64EA48** (rev B1; letter n ten runs of ten) the three
  shapes behave as on the DB, the filtered LUT on OSC32K waking from
  power-down too; and a LUT's asynchronous OUTPUT drives its pad through
  power-down (letter p): LUT0 on PA6 (ALT1) and LUT2 on PD3, each
  copying the PIT divider on its event input, wake the device on a
  single edge of the pad they drive ([port.md](port.md)).

## Not covered yet

Driver gaps:

- Typed per-input legality of the peripheral instances (today: the
  enum + the table above, an input whose peripheral the part has not
  got at all - `tca1` without TCA1, `zcd` without a ZCD - not named):
  `zcd` on input 1 or 2 where the package carries only ZCD0 (28/32
  pins), or on input 2 of the AVR DA48 (ZCD0..1), selects a reserved
  code and is not refused, because the refusal is a run-time check every
  image that builds a `LutConfig` at run time would pay; born with its
  first user.

Implemented but not bench-verified:

- The D-latch/DFF distinction under a slow LUT clock (the latch and
  the DFF are each measured, never side by side on one slow clock).
- The ZCD and TCD0 inputs, and SPI0's MOSI on inputs 0/1 on the AVR
  DA/DB (SCK is measured above, MOSI on the EA): typed, no letter drives
  them on those dies - the same pass-through on a TCD waveform, a ZCD
  crossing or the EA's MOSI leg run on a DB would measure them.
- The 28/32-pin AVR EA (compile-verified, no such part on the bench):
  its four LUTs as the 48-pin part's, TCA1's input on parts whose TCA1
  has only PORTA/PORTD routes. `LutClock::oschf`, likewise: the FEEDBACK divider above on it
  would.
- A slow-domain-proof stamp protocol (the OSC32K delay measurement
  is single-shot today: a back-to-back repeat races the re-arm).
