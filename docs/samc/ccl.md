# CCL - Configurable Custom Logic (SAM C21)

> **PROVISIONAL.** The whole register file is implemented and
> bench-verified - every INSEL source, both output stages, the edge
> detector, all four sequential modules, both event directions and all
> four errata. What is not covered is standby operation (CTRL.RUNSTDBY
> is written and read back but no LUT has been watched across a sleep),
> the E and G variants (compile-only), and a `util/` usage type. The
> list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet **DS60001479M** ch. 37
(the CCL), ch. 29 (the event system's tables), ch. 6 (the I/O
multiplexing) and silicon errata **DS80000740S** 1.7.1..1.7.4 and
1.8.3. Driver: `samc/ccl.hpp`, over `samc/device_tables.hpp`'s CCL
probes. Family fixture `test/family_samc/ccl.cpp` plus ten negatives
under `test/family_samc/neg/`. Bench suite `test_samc_ccl`.

## What the silicon does

**Four look-up tables and two sequencers, on every variant.** Each LUT
takes three inputs through independent multiplexers, decodes them
through an eight-bit TRUTH table, and optionally passes the result
through a synchronizer or a glitch filter and then an edge detector.
Adjacent LUTs form a PAIR, and each pair has a sequential sub-module
that can be a gated D flip-flop, a JK flip-flop, a gated D latch or an
RS latch - the even LUT driving D/J/D/S, the odd one G/K/G/R.

**There is no interrupt and no DMA request** (37.5.4 and 37.5.5 are
both "Not applicable"). The only ways out of this peripheral are a pad
and an EVSYS generator - which is what makes it a *core-independent*
block rather than another thing the CPU services.

**One generic clock for the whole peripheral.** GCLK_CCL is peripheral
channel 38, shared by every filter, edge detector and sequential module
in the block, so slowing one down slows all of them. 37.5.3 calls it
"optionally required": a purely combinational LUT needs no clock at
all, and this driver takes that literally (`Ccl::init()` with no
generator).

**The input menu is per-LUT, not per-input.** Each peripheral source
selects its instance from the LUT's own number: LUT *n* sees CMP[n],
TC[n] (default) and TC[n+1] (alternative), TCC[n mod 3] and SERCOM[n].
The one exception is TCC, which hands WO[0], WO[1] and WO[2] to inputs
0, 1 and 2 of the same LUT. `Lut<n>::ac_source` and its four siblings
evaluate 37.6.2.4's formulas so a caller can `static_assert` what it is
wiring. LINK is the *next* LUT's output (the last wrapping to LUT0) and
FEEDBACK is the pair's sequencer output.

**Enable-protection here is an AND of two gates**, which is neither
what the chapter says nor quite what the erratum says - see "Bench
findings". In practice: **the block must be disabled to configure
anything**, which means reconfiguring one LUT drops every other LUT's
output for the duration. That is the silicon's design, and the driver's
verbs refuse rather than write where the silicon will not look.

**The pad map is a package fact, and one LUT loses its pins.** The CCL
sits on peripheral function **I**. LUT3's inputs IN[9..11] (PB14, PB15,
PB16) and its output OUT[3] (PB17) are bonded on the **J alone**, so on
the E and the G, LUT3 exists in silicon - `CCL_LUT_NUM` is 4 everywhere
- with no pin of its own, reachable only through events, a link or a
sequencer. The E bonds no PORT B pad to the CCL at all. Two pads also
carry the SAME input line: PA04 and PA16 are both CCL0/IN[0], PA08 and
PA30 both CCL1/IN[3].

## The errata

Of five items touching this chapter at silicon rev F (E/G/J **row**,
never the N row):

- **1.7.1 RS Latch Reset** ("the reset of the RS latch is not
  functional; the latch can only be cleared by disabling the LUT"):
  **revision B only**. Not coded around, and the bench checks that the
  reset really works here.
- **1.7.2 Sequential Logic**, **every revision, so live**: after an
  even LUT has been disabled to clear its flip-flop or latch and
  enabled again, the sequential logic **stays under reset** until
  CTRL.ENABLE is written again. The workaround is code:
  `Lut<n>::enable(true)` on an even LUT re-states CTRL.ENABLE, and
  `Ccl::restate_enable()` is the same thing by name for a caller that
  drove the bit itself. Reproduced at the bench with a control on both
  sides.
- **1.7.3 Enable Protected Registers**, **every revision**:
  "the SEQCTRLx and LUCTRLx registers are enable-protected by the
  CTRL.ENABLE bit, whereas they must be enable-protected by the
  LUTCTRLx.ENABLE bits." Measured, it is not a swap but an AND (below).
  Every configuring verb refuses while the block is enabled.
- **1.7.4 PAC Protection Error**, **every revision**: writing
  CTRL.SWRST raises a PAC protection error. There is no workaround and
  no other way to reset the peripheral, so `reset()` writes it anyway
  and says so. Observed: the flag really is raised, and nothing else
  happens.
- **1.8.3 TC Selection** ("the default TC selection as CCL input is not
  TC0, but TC4"): **revision B only**, and confirmed by measurement
  rather than by reading the row.

## Types and verbs

**`LutInput`** - `masked`, `feedback`, `link`, `event`, `io`, `ac`,
`tc`, `alt_tc`, `tcc`, `sercom`, plus `alt2_tc` and `async_event`,
which exist only on the C20/C21 **N** variants and are refused here.
`lut_input_available()` answers that from the device header, through
the reserve.

**`LutFilter`** - `none`, `sync`, `filter` (0x3 is Reserved and has no
name).

**`LutSequencer`** - `none`, `d_flip_flop`, `jk_flip_flop`, `latch`,
`rs_latch`.

**`LutConfig`** - the whole LUTCTRLn bar its ENABLE bit: the three
inputs, `truth`, `filter`, `edge_detect`, `event_in` (LUTEI),
`invert_event_in` (INVEI) and `event_out` (LUTEO). ENABLE is not a
field here because it is decided by WHICH verb runs.

**`lut_truth(predicate)`** builds the table from a lambda of three
bools, with IN[0] as the LSB (table 37-1) - the same convention as
`avrdx/ccl.hpp`, so a table written for one family reads correctly on
the other. `lut_truth_pass(i)` / `lut_truth_invert(i)` are the two
tables every probe needs.

**`ccl_lut_config_valid(lut, cfg)`** carries four refusals: an INSEL
code this device does not implement; the edge detector with no filter
and no synchronizer (37.6.2.6); an event input SOURCE with LUTEI clear;
and INVEI with LUTEI clear.

**`Ccl`** - `lut_count`/`sequencer_count`/`input_count`/`gclk_id`/
`pac_id`; `output_generator(lut)` and `input_user(lut)` (the EVSYS
codes this peripheral publishes - `evsys.hpp` owns the fabric, not the
vocabulary); `bus_clock`, `clock`/`unclock`, `reset`, `enable`,
`enabled`, `run_standby`, `restate_enable`, `init(generator =
no_clock)`, `release`; `sequencer(pair, sel)` / `sequencer<pair, sel>()`
/ `sequencer(pair)`.

**`Lut<n>`** - `index`, `pair`, `is_even`, `event_generator`,
`event_user`, `has_input_pad`/`has_output_pad` (the per-package gate),
the five source formulas `ac_source`/`tc_source`/`alt_tc_source`/
`tcc_source`/`sercom_source`/`link_source`; `configure(cfg, enable)` /
`configure<cfg>(enable)`, `config()`, `enable`/`enabled`,
`truth(t)`/`truth()`, `ctrl()`, `listen(channel, cfg)`/`unlisten()`.

**`CclIn<Pin>` / `CclOut<Pin>`** - one input line or one output reached
through a pad that carries it, from the reserve's own map; a pad this
device does not bond fails to compile. `CclIn` publishes `line`, `lut`
and `input`; `CclOut` publishes `lut` and a `read()` that reports what
the pad is actually at.

## How to use it

**Glue logic on three pins, with no clock at all:**

```cpp
using A = brio::CclIn<brio::Pin<'A', 16>>;    // CCL0/IN[0]
using B = brio::CclIn<brio::Pin<'A', 17>>;    // CCL0/IN[1]
using Y = brio::CclOut<brio::Pin<'A', 19>>;   // CCL0/OUT[0]

brio::Ccl::init();                             // bus clock + reset, block down
A::claim();
B::claim();
Y::claim();
brio::Lut<0>::configure({
    .in0 = brio::LutInput::io,
    .in1 = brio::LutInput::io,
    .truth = brio::lut_truth([](bool a, bool b, bool) { return a != b; }),
});
brio::Ccl::enable(true);                       // the LUT decodes from here on
```

**A comparator synchronized to a chosen clock, one whole period cheaper
than the AC's own synchronizer:**

```cpp
brio::Ccl::init(gclk_generator);               // GCLK_CCL = the wanted clock
brio::Lut<0>::configure({
    .in1 = brio::LutInput::ac,                 // CMP0, with COMPCTRL0.OUT = ASYNC
    .truth = brio::lut_truth_pass(1),
    .filter = brio::LutFilter::sync,
});
brio::Ccl::enable(true);
```

**A divide-by-two with no CPU: a JK flip-flop toggled by GCLK_CCL.**
Note the order - the sequencer is written BEFORE the even LUT, and the
block goes up last:

```cpp
brio::Ccl::enable(false);
brio::Ccl::sequencer(0, brio::LutSequencer::jk_flip_flop);
brio::Lut<0>::configure({.truth = 0xFF});      // J = 1
brio::Lut<1>::configure({.truth = 0xFF});      // K = 1
brio::Ccl::enable(true);                       // OUT[0] now toggles per GCLK_CCL
```

**A LUT output as an event, and an event as a LUT input:**

```cpp
brio::Lut<0>::configure({.in0 = brio::LutInput::io,
                         .truth = brio::lut_truth_pass(0),
                         .event_out = true});
// ... elsewhere: Evsys::connect(some_user, channel,
//                   {.generator = brio::Lut<0>::event_generator,
//                    .path = brio::EventPath::asynchronous});

// and the other direction - ASYNCHRONOUS ONLY (table 29-3):
brio::Lut<1>::configure({.in0 = brio::LutInput::event,
                         .truth = brio::lut_truth_pass(0),
                         .event_in = true});
brio::Lut<1>::listen(channel, {.generator = some_generator,
                               .path = brio::EventPath::asynchronous});
```

## Bench findings

From `test_samc_ccl` (7 letters, 141 verdicts, **141/141** four times -
three warm and one cold from a fresh flash - in about two seconds).
**Nothing to wire.** The stimuli are a free pad walked between the rails
by its own internal pull (which survives PMUXEN where the output driver
does not - [port.md](port.md), [eic.md](eic.md)), TC and TCC waveforms
the CCL takes internally, and the analog comparator fed from a
GPIO-driven pad. The observers are the CCL's own OUT pads read back
through PORT.IN, a DMA block that either moved or did not, and the
SysTick cycle stopwatch.

### Enable protection: two gates, ANDed

Four cells of a truth table - a raw TRUTH write attempted with each
ENABLE bit in each state - settle three documents that disagree:

| CTRL.ENABLE | LUTCTRLn.ENABLE | the write lands |
|---|---|---|
| 0 | 0 | **yes** |
| 0 | 1 | no |
| 1 | 0 | no |
| 1 | 1 | no |

- **37.6.2.1** (LUTCTRLn protected by its own ENABLE) is right as far
  as it goes; **37.8.2**'s note (SEQCTRL protected by CTRL.ENABLE) is
  right too; **erratum 1.7.3** describes a *swap* and it is not a swap -
  **both gates are live, and a write lands only with both clear.**
  SEQCTRL behaves the same way (0/0 accepted, block enabled refused,
  even LUT enabled refused).
- **37.6.2.1's escape is real**: configuration and ENABLE = 1 in ONE
  store land together (`0x5A000402` written, `0x5A000402` read back).
  That is a 0 -> 1 *transition* of ENABLE, not a store into an already
  enabled LUT - which is what makes a one-store `configure()` legal.
- **A store into an enabled LUT is dropped in complete silence** - no
  flag, no fault, the old table still decoding. This is what the first
  version of `ccl.hpp` got wrong and this suite caught: `configure()`
  and `truth()` now drop LUTCTRLn.ENABLE in a store of its own first,
  because 37.6.2.1 forbids writing the protected bits together with
  ENABLE = 0.
- **The price, measured:** with the block taken down to reconfigure
  LUT0, a running LUT1's pad drops and comes back with the block. There
  is no way around it on this silicon.
- **CTRL.RUNSTDBY is enable-protected too**, and 37.8.1's "this bit must
  be written before enabling the CCL" is enforced: a raw store clearing
  it under a running block leaves the bit unchanged.
- CTRL implements ENABLE and RUNSTDBY and nothing else (0xFF stored,
  0x00 read back once SWRST has self-cleared).

### The truth table and the input multiplexer

- **All eight rows, twice.** TRUTH 0x96 (three-input XOR - it changes on
  every input, so no wiring mistake can hide) reads back off the pad as
  0x96, and TRUTH 0xE8 (majority) as 0xE8.
- **A masked input is tied low** whatever its pad does, and the same
  table with IO selected follows the same pad - the control that makes
  the first statement mean something.
- **The output pad is driven by the peripheral, not by PORT**: PA19 is
  high with PORT's own OUT and DIR bits for it both at 0.
- **LINK really is the NEXT LUT's output**: a pad edge crosses LUT1 and
  comes out of LUT0.
- **ERRATUM 1.8.3 IS REVISION B, measured with two controls.** TC0's
  WO[0] drives LUT0's default TC input; with TC0 stopped the output
  stands still; and TC4 running the same waveform does **not** reach it.
  ALTTC is TC1, checked the same way.
- **THE DEVICE HEADER IS MISSING A CODE, AND THE CHAPTER IS RIGHT.**
  INSEL 0x8 (TCC) has **no enumerator in any header of this pack**,
  though 37.8.3 lists it for every variant and marks only ALT2TC and
  ASYNCEVENT as N-only. Written as a literal, it works: TCC0's WO[0]
  moves LUT0's input 0 (with the waveform read off TCC0's own pad as a
  control), the field reads back as 8, and stopping TCC0 stops the LUT.
  `LutInput::tcc` is therefore spelled from the datasheet.
- **SERCOM is a real input source**: an idle SERCOM0 transmitter holds
  LUT0's input high, and a stream of 0x00 bytes moves it.

### The stages, and what each one costs

- **37.5.3 is exact.** With GCLK_CCL **disconnected**, a purely
  combinational LUT still decodes; a LUT with the synchronizer passes
  nothing at all, and comes alive the moment the channel is connected.
  The generic clock is needed for events, filters, edges and sequencers
  and for nothing else.
- **The edge detector's strobe is one GCLK_CCL period**, exactly:
  measured **4086..4108 CPU cycles** against a period of 4096, caught
  8/8 times. Held high, the output goes back low - it is an edge
  detector, not a level pass. A disabled LUT drives nothing.

The latency table below is the campaign's headline and has its own
section.

### The sequencers

- **Tables 37-2 to 37-5, all four, all rows** (bar the RS latch's
  forbidden 1/1): the DFF sets, clears and holds in both states; the JK
  sets, clears, holds and **toggles** (a divide-by-two with no CPU in
  it); the D latch follows D with G high and holds with G low; the RS
  latch sets and holds.
- **THE GATE MUST COME DOWN FIRST.** Both stimuli here are pads walking
  between the rails through their own pulls, so moving D and G in the
  same breath is a race the *pads* decide - the first version of the
  D-latch test lost its hold state that way. Close the gate, let it
  settle, then move D.
- **ERRATUM 1.7.1 IS REVISION B**: the RS latch's reset works on this
  silicon.
- **The odd LUT's own output stays available** on OUT[1] while the
  sequencer takes over the even LUT's OUT[0].
- **FEEDBACK participates, proven by a difference**: a JK whose K comes
  from FEEDBACK oscillates on a held J, where the same JK with K forced
  low sets once and stays.
- **Disabling the even LUT clears the module asynchronously** - and
  **ERRATUM 1.7.2 IS REPRODUCED WITH A CONTROL ON BOTH SIDES**: after a
  *raw* re-enable the latch is still under reset and will not set, and
  writing CTRL.ENABLE again - the errata's own workaround, and what
  `Lut<n>::enable(true)` does for an even LUT - brings it back.

### Events, both ways

- **A LUT output edge moves a block of memory**: a pad, a truth table,
  LUTEO, an asynchronous EVSYS channel and a DMA channel armed with *no
  hardware trigger at all*. With LUTEO clear the same edge moves
  nothing.
- **An event reaches the truth table**, and it arrives as the one-GCLK
  strobe 37.6.2.4 describes rather than as a level. With LUTEI clear the
  same events reach nothing. A synchronous or resynchronized channel
  into this user is refused by the driver, because table 29-3 grants it
  the asynchronous path alone.
- **A SOFTWARE EVENT *DOES* CROSS AN ASYNCHRONOUS CHANNEL - and this
  corrects [evsys.md](evsys.md).** Sixteen of sixteen single, spaced
  software events reach this LUT on an asynchronous channel; the same
  sixteen with the user disconnected reach nothing; and **one** of them
  moves a whole DMA block *through* the LUT, which is a second witness
  of a different kind. `test_samc_evsys` found that eight of them moved
  nothing through a DMA channel on the same path - so the limit belongs
  to the **user's input stage**, not to the path: a register write has
  no width for the DMAC's trigger, and the CCL's own event edge detector
  catches every one.

### THE LATENCY TABLE - what a CCL output costs, and the answer to ac.md

The question [ac.md](ac.md) left open: the comparator's *synchronized*
output costs the fraction to the next GCLK_AC edge **plus two whole
periods**, which killed an application that wanted a clock-synchronized
comparator cheaply. Two leads were named and never measured - a LUT as a
one-stage synchronizer, and a combinational LUT on the comparator's
*asynchronous* flavour. Both are measured here.

The instrument is `ac_sync_probe`'s: GCLK_AC and GCLK_CCL both on
generator 1 at OSC48M/4096 = **11.719 kHz**, so one period is exactly
**4096 CPU cycles** and a SysTick stopwatch resolves a fraction of one.
PA04 is driven as a plain GPIO into COMP0's positive input against the
comparator's own VDD scaler; 32 anchored phase steps plus 20 randomized
shots per row, minimum of three per step.

| path | cycles | in GCLK periods |
|---|---|---|
| AC pad, OUT = ASYNC (the chain's own floor) | 199..199 | 0.04 |
| **LUT, combinational** | **207..207** | **0.05** |
| **LUT pair as a DFF** | **207..4217** | **fraction + 0** |
| **LUT, FILTSEL = SYNCH** | 4348..8407 | **fraction + 1** |
| AC pad, OUT = SYNC | 8407..12343 | fraction + 2 |
| **LUT, FILTSEL = FILTER** | 12499..16433 | **fraction + 3** |

- **40.8.13's note is about COMPCTRL.OUT and not about a pad.** With
  OUT = ASYNC the CCL sees the comparator's *raw* output; the note's
  "this bit must be 0x1 or 0x2" is the whole requirement, and no pad
  need be muxed.
- **A combinational LUT costs no clock edge at all** - 207 cycles flat
  across the whole phase sweep, eight CPU cycles above the comparator's
  own asynchronous pad. Lead (b) confirmed: a LUT dodges every sampler
  in the chain.
- **A LUT PAIR AS A DFF COSTS NO WHOLE PERIOD**, just the fraction to
  the next GCLK_CCL edge. That is a true one-stage synchronizer and the
  cheapest clocked path on this die.
- **One LUT with the synchronizer costs the fraction plus exactly one
  period** - half what the comparator's own sampler charges, out of a
  single LUT.
- **The filter costs the fraction plus exactly three periods.** So
  37.6.2.5's "delayed by two to five GCLK cycles" is a range over the
  two *options* and their phase; each option's own cost is exact.
- **The application the AC killed does not revive as "no CCL at all"** -
  the comparator's +2 is silicon and no COMPCTRL setting removes it.
  What the numbers say instead is that **the CCL is strictly cheaper
  than the AC's own synchronizer**: one LUT buys fraction + 1, a LUT
  pair buys fraction + 0, and a bare LUT buys no synchronization at all
  for eight CPU cycles.

## Not covered yet

Driver gaps (not built):
- **Usage types.** There is no task layer over `Lut<n>` yet - no
  `ToggleFlipFlop` equivalent of the AVR's, no debounce, no PWM gate.
  They are born with their first user, as everywhere in this stratum.
- **A `util/` contract.** Nothing in `util/` speaks logic fabric, and
  nothing here proposes one until a second family's CCL says what the
  portable shape is.
- **The N-variant input codes** ALT2TC and ASYNCEVENT are named in the
  vocabulary and refused on this family. If a C20/C21 N ever arrives,
  `ccl_has_alt2_tc()` / `ccl_has_async_event()` are the switches and
  ASYNCEVENT's five-step procedure in 37.6.2.4 is unwritten.

Implemented but not bench-verified:
- **Standby.** CTRL.RUNSTDBY is written and read back and its
  enable-protection is proven, but no LUT has been watched across a
  sleep, so 37.6.4's promise - a combinational LUT keeps working while a
  filtered, edged or sequential one has its output **forced to zero** -
  is datasheet-trusted. It belongs with the power pass.
- **The E and G variants**, compile-only: the pad map is asserted per
  variant out of the device header (`test/family_samc/ccl.cpp`) and two
  negatives refuse the pads those packages lack, but no such board
  exists here. LUT3 with no pins has never been *run*.
- **The glitch filter as a glitch filter.** Its delay is measured
  exactly; its rejection is not, because nothing on this bench produces
  a glitch narrower than a GCLK_CCL period into a CCL input.
- **The whole pad map.** Four input pads and two output pads of the
  thirty-three this package bonds have been driven; the rest are the
  reserve's word.
