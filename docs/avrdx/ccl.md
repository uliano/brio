# CCL - the configurable custom logic (AVR DA/DB)

> **PROVISIONAL.** The register option space is fully exposed with
> the package and legality guards in place (LUT counts and pin
> existence from the device header, LUT3-ALT refused), and the
> sequencers, LINK, peripheral inputs and filter delays are
> bench-verified; the typed per-input instance legality is not - the
> gaps are in "Not covered yet". Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B (CCL
> chapter 31, PORTMUX 17.3.2, EVSYS 16 generators 0x10-0x15 / users
> 0x00-0x0B, I/O multiplexing chapter 3), errata DS80000915F (2.4.1,
> 2.4.2). Complements: TB3218 "Getting Started with CCL", AN2434, the
> Microchip example `avr128db48-blink-led-ccl` (see
> [vendor/README.md](vendor/README.md)). Driver: `avrdx/ccl.hpp` (`Ccl`,
> `Lut<n>`, `ToggleFlipFlop<pair>`), the LUT event vocabulary in
> `avrdx/evsys.hpp`. Reference test: `test_avr_timer` (test c).

## What the silicon does

One CCL with six look-up tables (LUT0..LUT5 on 48/64-pin parts,
LUT0..3 on 28/32 - the driver takes the count from the device
header) grouped in three pairs (0-1, 2-3, 4-5), each pair
owning one sequencer. A LUT is: three inputs chosen from a menu, an
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
fixed table: TCB3 and TCA WO3..5 are NOT reachable except through an
event channel, and AC2 only on input 2.

Facts that matter to code:

- **Enable protection**: `LUTnCTRLA/B/C`, `TRUTHn` and `SEQCTRL` can
  be written only while the (even) LUT is disabled - together with
  ENABLE = 1 in the same write, never together with ENABLE = 0; a
  `SEQCTRL` written after the even LUT is enabled is silently ignored
  (measured: the JK never engages, the LUT outputs the bare truth
  table). On top of that, **errata 2.4.1 (DB rev A4/A5; fixed on B0;
  the DA has no such erratum)**: reconfiguring ONE
  LUT requires the whole CCL disabled (`CTRLA.ENABLE = 0`), which
  drops every other LUT meanwhile - a driver must build the complete
  configuration first and enable once; a runtime change of one LUT is
  a glitch on all of them. Changing a clock source also wants the CCL
  disabled. On A4/A5 the CCL interrupt wakes the device from standby
  only when the LUT path is fully asynchronous (no filter, no edge
  detector - the errata document's sleep clarification).
- **Clock (`CLKSRC`)** per LUT, used by the filter, the edge detector
  and (from the even LUT) the sequencer: CLK_PER, the LUT's own
  TRUTHSEL[2] input (then input 2 is seen as low in the truth table),
  OSCHF (before the prescaler), OSC32K, OSC1K. `RUNSTDBY` keeps the
  chosen clock alive in standby; without it a filtered/sequenced LUT
  outputs 0 in standby (the bare truth table keeps working in idle).
- **Filter (`FILTSEL`)**: SYNCH = two clock delay, FILTER = an input
  must hold for more than two clocks, four clocks delay. The edge
  detector REQUIRES a filter option. Disabling a LUT clears its
  filter/edge logic one clock later.
- **Sequencer (`SEQSEL`)** per pair, clocked with the even LUT: D
  flip-flop (D = even, EN = odd, CLOCKED by the even LUT's clock),
  JK (J = even, K = odd, toggles on 1/1), D latch (D = even, G =
  odd: the TRANSPARENT one), RS latch (S = even, R = odd, 1/1
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
  PORTC route WO0..3 (the board-level collisions are in
  [bench.md](../bench.md)). `OUTEN`
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
| helpers | `lut_truth(f)`, `lut_config_valid`, `lut_port(n)` |

The protocol, dictated by errata 2.4.1 and the enable protection:
`Ccl::disable()`, `Ccl::sequencer<p>(...)` (before the even LUT's
init - `sequencer` disables that LUT itself), every `Lut<n>::init(...)`,
`Ccl::enable()` - the whole wiring at once. A CCL application is a fixed wiring: the
EventSystem static allocation of [evsys.md](evsys.md) is its natural
companion.

## How to use it

Glue logic on pins - PB3 = PB0 AND PB1 (LUT4's port is PORTB):

```cpp
#include "avrdx/ccl.hpp"
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

`test_avr_timer` tests l and i (A5): LINK verified (LUT4 passes
LUT5's constant both ways onto PB3); the DFF follows a 1 kHz D
exactly (EN high, CLK_PER clock); the RS latch sets and resets from
two software channels, read on LUT2's pin; LUT1's ALT1 drives PC6
and a re-init without the pin releases its direction. TCA0 WO0 as a
LUT input reads 24000 ticks exact; AC0 as a LUT input counts 3 DAC
crossings; the filter delays measured DIFFERENTIALLY (the stamp
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

## Not covered yet

Driver gaps:

- Typed per-input legality of the peripheral instances (today: the
  enum + the table above).

Implemented but not bench-verified:

- The D latch as a sequencer (DFF, JK and RS are bench-verified);
  FEEDBACK as an input; the D-latch/DFF distinction under a slow LUT
  clock.
- The peripheral inputs beyond events, pins, TCA WO and AC (TCB WO,
  USART, SPI: the menu is typed, no test drives them).
- `LutClock::input2` and OSC1K; RUNSTDBY (queued, LOW priority -
  including the A4/A5 wake-only-if-asynchronous clarification).
- A slow-domain-proof stamp protocol (the OSC32K delay measurement
  is single-shot today: a back-to-back repeat races the re-arm).
