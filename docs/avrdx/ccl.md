# CCL - the configurable custom logic (AVR DA/DB)

> **PROVISIONAL.** The chapter and errata are reviewed and the driver
> is written against them; the bench suite is written but has not run
> on silicon yet - the page becomes EXHAUSTIVE with its first green run.
> Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B (CCL
> chapter 31, PORTMUX 17.3.2, EVSYS 16 generators 0x10-0x15 / users
> 0x00-0x0B, I/O multiplexing chapter 3), errata DS80000915F (2.4.1,
> 2.4.2). Complements: TB3218 "Getting Started with CCL", AN2434, the
> Microchip example `avr128db48-blink-led-ccl` (see
> [vendor/README.md](vendor/README.md)). Driver: `avrdx/ccl.hpp` (`Ccl`,
> `Lut<n>`, `ToggleFlipFlop<pair>`), the LUT event vocabulary in
> `avrdx/evsys.hpp`. Reference test: `test_avr_timer` (test c).

## What the silicon does

One CCL with six look-up tables (LUT0..LUT5 on 48/64-pin parts,
LUT0..3 on 28/32) grouped in three pairs (0-1, 2-3, 4-5), each pair
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
  ENABLE = 1 in the same write, never together with ENABLE = 0. On
  top of that, **errata 2.4.1 (A4/A5, our A5)**: reconfiguring ONE
  LUT requires the whole CCL disabled (`CTRLA.ENABLE = 0`), which
  drops every other LUT meanwhile - a driver must build the complete
  configuration first and enable once; a runtime change of one LUT is
  a glitch on all of them. Changing a clock source also wants the CCL
  disabled.
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
  flip-flop (D = even, G = odd: transparent when G = 1), JK (J = even,
  K = odd, toggles on 1/1), D latch (D = even, G = odd), RS latch (S =
  even, R = odd, 1/1 forbidden). Disabling the even LUT clears the
  flip-flop asynchronously (reset held one clock). The sequencer
  output is what FEEDBACK returns and what the pair's output carries.
- **Pins** (PORTMUX `CCLROUTEA`, one bit per LUT moves only the
  OUT): LUTn inputs IN0/IN1/IN2 are pins 0/1/2 and OUT is pin 3
  (ALT1: pin 6) of port A/C/D/F/B/G for LUT0/1/2/3/4/5; LUT3 has no
  ALT1 (PF6 is RESET). Note the bench collisions: LUT0 owns PA0..PA3
  (PA0/PA1 are the crystal), LUT3 PF0..PF3, LUT1 PC0..PC3 = TCA0 PORTC
  WO0..3, LUT2 PD0..PD3 (PD1/PD2 are the analog suite's wires). `OUTEN`
  drives the pin; the pin's direction must be set by PORT.
- **Errata 2.4.2** (A4 and 28/32-pin only): LINK into LUT3 dead - not
  our silicon.
- The CCL is a level generator (0x10 + n) for the event system, async;
  its event users take the channel without detection (a level or a
  pulse reaches the truth table as is).

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `LutConfig` | `in0`/`in1`/`in2` (`LutInput`: mask, feedback, link, event_a, event_b, pin, ac, zcd, usart_txd, spi0, tca0, tca1, tcb, tcd0 - the instance follows the input index), `truth` (TRUTHn; build it with `lut_truth(lambda of three bools)`), `filter` (`LutFilter`: none, sync, filter), `edge_detect` (needs a filter), `clock` (`LutClock`: clk_per, input2, oschf, osc32k, osc1k), `output_pin`, `alt_pin`, `interrupt` (`LutSense`: none, rising, falling, both) |
| `Ccl` | `enable(run_standby)`, `disable` (the state every reconfiguration starts from), `enabled`, `sequencer<pair>(Sequencer)` (none, d_flip_flop, jk_flip_flop, d_latch, rs_latch), `take_flags` (ISR body of CCL_CCL_vect: bit n per LUT, cleared) |
| `Lut<n>` | `init<cfg>()` / `init(cfg)` (block disabled: inputs, truth, route, pin, sense, then CTRLA with ENABLE), `enable`/`disable` (this LUT), `sense(LutSense)`, `flag`/`clear_flag`, `event_a_on(channel)`/`event_b_on(channel)`, the register accessors; `In0`/`In1`/`In2`/`OutDefault`/`OutAlt` pin types, `port`; `OutEvent` generator, `EventA`/`EventB` users |
| `ToggleFlipFlop<pair>` | `init(toggle_channel, output_pin, alt_pin)`: JK on LUT 2p/2p+1, J = K = the event: one toggle per pulse (a divide-by-two, no CPU); `Even`/`Odd` |
| helpers | `lut_truth(f)`, `lut_config_valid`, `lut_port(n)` |

The protocol, dictated by errata 2.4.1: `Ccl::disable()`, every
`Lut<n>::init(...)` and `Ccl::sequencer<p>(...)`, `Ccl::enable()` -
the whole wiring at once. A CCL application is a fixed wiring: the
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

None yet.

## Not covered yet

The first bench run of `test_avr_timer` (test c). In the driver: the
other sequencers as tasks (D flip-flop, latches - the resource does
them, no task names them), LINK chains, the peripheral inputs beyond
events and pins (TCA/TCB WO, AC, USART, SPI as LUT inputs: the menu is
typed, no test drives them), the alternate clocks (OSC32K/OSC1K for a
low-power filter), RUNSTDBY, the filter delay measured in clocks,
typed per-input legality of the instance (today: the enum + the doc
table).
