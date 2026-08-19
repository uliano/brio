# CCL - the configurable custom logic (AVR DA/DB)

> **PROVISIONAL.** The systematic review of the chapter and errata is
> done (this page records it); the driver and its bench suite are not
> written yet. Documents of record: AVR128DB28/32/48/64 data sheet
> DS40002247B (CCL chapter 31, PORTMUX 17.3.2, EVSYS 16 generators
> 0x10-0x15 / users 0x00-0x0B, I/O multiplexing chapter 3), errata
> DS80000915F (2.4.1, 2.4.2). Complements: TB3218 "Getting Started
> with CCL", AN2434, the Microchip example `avr128db48-blink-led-ccl`
> (see [vendor/README.md](vendor/README.md)). Driver: none yet.
> Reference test: none yet.

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

None yet. The intended shape: a resource `Ccl` (enable/disable the
block, the three sequencers, the interrupt vector body with the
per-LUT flags) and `Lut<n>` (a config struct: three typed inputs from
the menu above with the instance checked per input index, the truth
table as eight bits or a `constexpr` function of three bools, filter,
edge, clock, output pin, interrupt sense); tasks named for what a
pair does (a flip-flop or latch between two logic functions, a glitch
filter/edge detector on an event, an AND/OR/XOR of timer outputs).
The static-allocation sugar of [evsys.md](evsys.md) (EventSystem)
gets its first real user here: a CCL application is a fixed wiring.

## How to use it

Nothing to show yet.

## Bench findings

None yet.

## Not covered yet

Everything above; the suite (`test_avr_timer`: a LUT as AND of two
pins toggled by the test and read back on the OUT pin, a JK flip-flop
toggled by a TCB event checked with the TCB counting its own output,
a filter checked against a short pulse); the EVSYS vocabulary for CCL.
