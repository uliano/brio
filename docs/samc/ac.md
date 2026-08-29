# AC (SAM C21)

> **PROVISIONAL.** The block, all four comparators in continuous and
> single-shot mode, the VDD scaler, filters, hysteresis, the DAC and
> bandgap negative inputs, both output routings, flags and interrupts,
> BOTH WINDOWS, both event directions with their inversion, the 40.6.10
> offset procedure and per-package input legality are built and
> bench-verified. What remains is a window across a sleep, DBGCTRL
> policy, and the two J-only pad inputs of the COMP2/3 pair. The list is
> in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 40 and
the electrical characteristics table 45-34 (whose note 4 states the
propagation delay is measured on "ACOUT (AC direct output)" and
covers ONLY the analog path - the digital path's cost in cycles is
stated nowhere, which is what the probe below measured). Driver:
`samc/ac.hpp` (`Ac` block + `AcComparator<n>` + `AcWindow<w>`).
Family fixture `test/family_samc/ac.cpp` plus four negatives under
`tools/check_samc.sh`. Two bench SUITES and one probe, and the
distinction matters: `test_samc_ac` (6 letters, 94 verdicts, wireless)
is the chapter's own, `test_samc_analog` carries what needed the DAC as
a swept source (letters g to j), and `ac_sync_probe` remains a PROBE -
it answered one timing question and is not a reference test.

The errata, read on the E/G/J row at silicon revision F: of six AC
items plus one device-level one, **two apply** and neither is
fixable in a register. **1.5.3 Analog Pins** (all revisions): the AC
and the PTC share pads, so give the AC a non-pin input while the PTC
measures - a system-level choice, and there is no PTC driver here.
**1.5.6 Spurious COMP Interrupt** (all revisions): enabling with
MUXNEG = bandgap can raise a spurious COMPn flag, so clear that flag
after enabling and before arming; the driver does not refuse the
bandgap, which is a real input, and `AcNegative::bandgap` carries the
obligation. Not this silicon: 1.5.1 and 1.5.4 (hysteresis, revisions
B and B..E), **1.5.2 (low-power mode WITH hysteresis, B..E - so the
pairing is legal here)**, 1.5.5 (power figures, B..E), and the
device-level **1.8.2, which says GCLK_AC is not functional and the AC
must borrow GCLK_ADC1's channel - revision B only**, so `clock()`
uses AC_GCLK_ID as the device header declares.

## What the silicon does

**Everything digital samples at GCLK_AC; the comparator itself is
analog and free-running.** The filter section states the sampling
rate is the GCLK_AC frequency, and every figure of 40.6.2.4 draws a
"Sampled Comparator Output". COMPCTRLn.OUT routes either the raw
comparator (`ASYNC`) or the sampled version including filtering
(`SYNC`) to the CMPn pad. The AC has an APB clock for its registers
and the GCLK_AC peripheral channel (AC_GCLK_ID) for all of this
digital machinery - slow the generator down and every digital
latency becomes visible to a software stopwatch, which is the
probe's whole method.

**THE MEASURED ANSWER, stated first because it is why this driver
exists: a synchronized output edge costs the fraction of a period to
the next GCLK_AC edge PLUS TWO WHOLE PERIODS** - a two-stage
synchronizer, in the classic shape. Measured as a phase-anchored
64-step staircase (a clean sawtooth spanning exactly one period) and
as 1000 randomized shots, on two independent clockings: OSC48M/4096
on generator 1 (reached with the linear divider, and with DIVSEL and
DIV = 11, which is the same 4096) and OSCULP32K, a clock the CPU does
not share.
Floor 2.05 periods, ceiling 3.03, the ~0.05 being the measurement
chain's own constant (~210 CPU cycles, bounded by the ASYNC control
run). Both edges behave identically.

**AND THE CCL IS STRICTLY CHEAPER**, which is what an application
wanting a clock-synchronized comparator should do instead. 40.8.13's
note - "for internal use of the comparison results by the CCL, this bit
must be 0x1 or 0x2" - is about COMPCTRL.OUT and not about a pad, so a
LUT can take the comparator's **asynchronous** flavour and dodge this
sampler entirely. Measured on the same instrument at the same 11.719
kHz ([ccl.md](ccl.md)): a combinational LUT costs **0.05 periods**
(eight CPU cycles above the comparator's own ASYNC pad), a LUT with
FILTSEL = SYNCH costs the **fraction + 1**, a LUT PAIR as a D flip-flop
costs the **fraction + 0**, and this chapter's own synchronized output -
the fraction + 2 - is the slowest clocked path of the four.

**STATUSA.STATE is the sampled output too, whatever OUT says.** With
the pad routed ASYNC, the STATE readback still pays the same
fraction + 2 periods. STATE is declared valid only while
STATUSB.READY is one; after a continuous-mode enable, READY itself
arrived in ~6.7 GCLK_AC periods on the bench (the electrical
t_STARTUP of 2-3 us is invisible at a slow clock - the domain
crossings dominate).

**INTFLAG raises on the SAME period as the output flip.** Edge
detection compares the current and previous sample (40.6.2.4.1), and
the flag was consistently observed a few tens of CPU cycles BEFORE
the pad readback (the pad round-trip pays PORT input sampling that
the APB flag read does not) - not one period later.

**A mid-stream edge through the majority filter costs (N-1)/2 extra
periods, not the chapter's N-1.** Measured exactly: MAJ3 adds one
period (+4080 cycles on a 4096-cycle period), MAJ5 adds two (+8220).
The chapter's "N-1 sampling cycles from when a comparison is
started" times the FIRST VALID output from idle - a different
question. The majority arithmetic explains the mid-stream number: a
2-of-3 vote flips one sample after the edge's first new sample, a
3-of-5 two samples after.

**Single-shot START to READY measured 5.1..6.5 periods** where
figure 40-4 draws "2-3 cycles" plus t_STARTUP: the figure's cycles
are the command's domain crossing alone; the comparison itself and
READY's own journey back to the APB face pay the rest.

**A GPIO-driven pad reaches the comparator's input mux.** The AVR
suites' wireless trick holds on this family too: PA04 driven by PORT
as a plain output (no PMUX, no function B) is seen by MUXPOS = PIN0,
both levels, proven before anything else relied on it. The
comparator's negative input never needs a pad at all: each
comparator has its own 64-step VDD scaler (SCALERn), and the probe's
threshold is scaler 31 = VDD/2.

**Window mode is the second face of every PAIR** (40.6.4): COMP0/COMP1
are window 0 and COMP2/COMP3 window 1. The pair shares one positive
input - the signal - and its two negative inputs are the window's
limits, **in either order**; STATUSA.WSTATEw reports above / inside /
below and WINCTRL.WINTSELw picks which of four conditions raises
INTFLAG.WINw. The individual comparators keep working throughout.
The chapter asks for two things no register enforces - both
comparators of the pair in the same measurement mode, and the same
positive input on both - so `AcWindow<w>::pair_consistent()` asks the
silicon whether they hold. In single-shot mode either comparator's
START starts both measurements.

**The event surface runs both ways** (40.6.13), and this driver
publishes both vocabularies because `evsys.hpp` owns only the fabric.
Out: `comparator_generator(n)` (0x49..0x4C, a copy of the comparator
status) and `window_generator(w)` (0x4D/0x4E, a copy of the
inside/outside status - generated **whether or not the window's
interrupt is enabled**), gated by EVCTRL.COMPEOx / WINEOx. In:
`start_user(n)` (users 34..37, SOCn, "start a comparison"), gated by
EVCTRL.COMPEIx with an optional INVEIx inversion - and **table 29-3
marks all four SOC users ASYNCHRONOUS PATH ONLY**, which is a
constraint the channel's configuration must honour. EVCTRL is
enable-protected at BLOCK level, so `event_config()` refuses while
the block is enabled.

**Per-package input legality: the PAIR owns the pads.** COMP0/1 take
AIN[0..3] and COMP2/3 take AIN[4..7], so the same `pin2` code means
AIN2 on one comparator and AIN6 on another - and 40.1 bonds only
AIN[5:4] for COMP2/3 on the E and G variants. The device header says
the same in symbols (`PIN_PB05B_AC_AIN6` and `PIN_PB06B_AC_AIN7`
exist on the J alone), so that is the authority: `ac_config_valid()`
refuses a pin the package does not bond, and the comparator index is
part of the question.

**Two more chapter rules are refusals now**: hysteresis is
"available only in continuous mode" (40.6.6), and the
end-of-comparison interrupt is "single-shot mode only" (40.8.12).

**Enable-protection is per comparator.** Every COMPCTRLn field is
writable only while that comparator's own ENABLE is low; ENABLE
itself is write-synchronized (SYNCBUSY.COMPCTRLn), as are
CTRLA.SWRST/ENABLE and WINCTRL. Disabling the whole block
(CTRLA.ENABLE = 0) stops every comparator but leaves their
COMPCTRLn.ENABLE bits standing.

**A GCLK fact this probe needed** (it lives in
[clock.md](clock.md) and in `samc/clock.hpp`, where the config is):
GENCTRL.DIVSEL divides by **2^(DIV+1)** - the DIV value counts, and
the width of the field does not. The linear divisor is what this
probe uses, and only generator 1's linear field reaches past 255
(table 16-3).

## Types and verbs

- **Vocabulary** - `AcPositive`/`AcNegative` (the input muxes; PIN0..3
  are AIN[0..3] for COMP0/1 and AIN[4..7] for COMP2/3),
  `AcSpeed` (bias current: propagation delay AND startup),
  `AcFilter` (off/majority3/majority5), `AcOut`
  (off/asynchronous/synchronous), `AcInterrupt` (toggle/rising/
  falling/end-of-comparison).
- **`AcConfig` / `ac_compctrl()`** - the whole COMPCTRLn as a value,
  ENABLE deliberately excluded (configure() owns it); constexpr, so
  the family fixture pins shapes without a chip.
- **`Ac`** - the block: `init(generator)` (APB mask, the GCLK_AC
  channel onto the named generator, reset, enable - each wait bounded
  on SYNCBUSY), `take_flags()` (the read-and-clear ISR body; one
  INTFLAG serves all four comparators and both windows), `release()`.
- **`AcComparator<n>`** - one of the four: `config_valid(cfg)`
  (constexpr, and the refusals above run through it),
  `configure(AcConfig)` (refuses an illegal configuration, then
  disables first - every field is enable-protected), `enable()` with
  the synchronized wait, `scaler(v)` (VDD x (v+1)/64, writable live),
  `start()` (single-shot), `ready()`, `state()`, `single_shot()` and
  `positive()` (read back out of the silicon, for the window's
  consistency check), flag and arming verbs, plus its own
  `event_generator` and `start_event_user` codes. A fifth comparator
  is refused at compile time.
- **`AcWindow<w>`** - a comparator PAIR: `configure(on, condition)`
  (write-synchronized, but NOT enable-protected, so a window may be
  turned on under a running block), `enabled()`, `interrupt_on()`,
  `state()` (`AcWindowState` above/inside/below), `ready()` (both
  comparators), `pair_consistent()`, `start()`, its `flag`/
  `event_generator` and the flag/arming verbs. A third window is
  refused at compile time.
- **`AcEventControl` / `ac_evctrl()` / `Ac::event_config()`** - the
  whole EVCTRL as one value: the four comparator outputs, the two
  window outputs, the four start-a-comparison inputs and their
  inversions. `ac_event_control_valid()` refuses an inversion for an
  input nothing listens to.
- **`ac_ain_of()` / `ac_ain_exists()` / `ac_pin_index()`** - the
  per-package legality machinery, readable on its own.

## How to use it

```cpp
brio::Ac::init(1);                        // GCLK_AC from generator 1
using Comp = brio::AcComparator<0>;
Comp::configure({
    .positive = brio::AcPositive::pin0,   // AIN[0] = PA04
    .negative = brio::AcNegative::vscale, // its own VDD ladder
    .speed = brio::AcSpeed::high,
    .out = brio::AcOut::synchronous,      // sampled, on the CMP0 pad
});
Comp::scaler(31);                         // threshold VDD/2
Comp::enable(true);
while (!Comp::ready()) { }
bool above = Comp::state();
```

The CMP0 pad is PA12 (or PA18), function H; a pad routed by OUT
still needs its PMUX, and reading it back needs its input buffer
(`CmpPad::function(PinFunction::h, {.input_enable = true})`).

## Bench findings

### From `test_samc_ac` (6 letters, 94 verdicts, **94/94 three times**)

Nothing to wire. The stimulus is a pad driven by PORT and read by the
comparator, and the second voltage is each comparator's own VDD
scaler.

- **STATE and the CMP0 pad agree with the pad, both ways**, at scaler
  31 (~VDD/2), and a pad at VDD reads above scaler steps 0, 31 and 55
  while a pad at ground reads below all three - the 64-step ladder is
  a real divider, not a fixed threshold.
- **All three window states, and how to reach them on a board with no
  analog source.** A rail-driven pad cannot sit between two limits, so
  the roles are swapped: the SIGNAL is each comparator's VDD scaler
  (both at step 31, so the pair really does see one level) and the two
  LIMITS are the two rail-driven pads. That reaches `inside`
  (STATE0=0, STATE1=1), `above` (both limits at ground: STATE0=1,
  STATE1=1) and `below` (both at VDD: both 0) - and confirms that the
  mapping is symmetric in which comparator holds which limit, exactly
  as 40.6.4 says. **The chapter's own shape** - one shared input PIN,
  the two scalers as limits at 49/64 and 17/64 of VDD - was exercised
  too and reads `above` at VDD and `below` at ground; `inside` is
  unreachable in that shape here, and that is a board limitation, not
  a silicon one.
- **The comparators keep answering independently under window mode**,
  which 40.6.4 promises.
- **All four window interrupt selections fire on their own condition
  and stay silent on others**: `inside`, `below`, `above` and
  `outside` each raise INTFLAG.WIN0 on entering the state they name,
  and a window selecting `above` stays quiet through a below-to-inside
  move.
- **A comparator flip moves a DMA block** - pad to AC to EVSYS to
  DMAC, with no CPU in the path - and so does **a window transition**,
  which is 40.6.13's "copy of the inside/outside status" measured:
  the window's event is generated from the state regardless of WINTSEL
  (the window was selecting `above` while the event carried the move
  into `inside`).
- **A PIN EDGE STARTS A COMPARISON.** An EIC line's edge, through an
  **asynchronous** EVSYS channel - the only path table 29-3 allows for
  SOC0 - starts a single-shot comparison on COMP0: before the edge
  READY=0 and the end-of-comparison flag is clear, after it READY=1,
  the flag is set and STATE answers correctly. With EVCTRL.COMPEI0
  cleared the same edge starts nothing, so the enable bit is the gate.
  This is also the first thing in this stratum to use a hardware
  generator on an asynchronous channel for something other than a DMA
  transfer.
- **Re-pointing an event channel at a new generator leaves an event
  standing.** Measured, reproducibly: after `Evsys::connect()` moves
  channel 0 from COMP0 to WIN0, the first DMA arming that follows
  consumes a block even though the window has not moved. The reading
  that fits is that the re-route looks like an edge to the channel's
  detector and, with the user not yet ready, the channel HOLDS the
  event (29.2's USRRDY handshake) until it is. The suite arms twice
  and rests its verdict on the second, and prints the first as the
  fact it is.
- **A board fact**: PA04 and PA05 do NOT follow their own weak
  internal pull, though they reach both rails under PORT's output
  driver - so the suite's precondition is "does the pad go where PORT
  drives it", which is the right question for an analog input anyway.

### `AcNegative::dac`, from `test_samc_dac`

With COMP0's positive input on its own 64-step VDD scaler and its
negative on the DAC's internal output, the comparator flips at DAC code
**255 / 512 / 769** for scaler steps 15 / 31 / 47, against 255 / 511 /
767 predicted by the two dividers. The **gaps are 257 and 257 codes**
against 256 predicted, and differencing them cancels the comparator's
own offset - so that pair is the two ladders agreeing and nothing else.
The comparator also follows the DAC to both ends of its range. Details
in [dac.md](dac.md).

### From `ac_sync_probe` (a PROBE, not a suite)

The numbers behind the timing statements above, all
(GCLK_AC = OSC48M/4096 = 11.719 kHz unless said otherwise, one
period = 4096 CPU cycles at 48 MHz, stopwatch = SysTick):

- Synchronized output, rising edge: staircase min 8402 / max 12422 /
  span 4020 across 64 anchored phase steps; 1000 randomized shots
  min 8402 max 12482, 959 of them in whole-period bucket 2 and the
  rest in 3. Falling edge min 8479 max 12358. On OSCULP32K (period
  ~1465 cycles): min 3102 max 4542 - the same 2..3 periods on a
  clock the CPU does not share.
- ASYNC control: pad-to-pad 202..374 cycles - the measurement
  chain's constant (store + analog TPD + PORT input sampling + poll
  + stopwatch), far below one period, subtracted mentally from
  everything else.
- STATE0 with OUT=ASYNC: 8510..12474 cycles - the readback is the
  sampled path even when the pad is not.
- Filters, mid-stream edge vs the unfiltered floor: MAJ3 +4080,
  MAJ5 +8220 cycles (+1 and +2 periods).
- Single-shot START to READY: 20805..26692 cycles = 5.1..6.5
  periods. Continuous-mode enable to READY: 27489 cycles (~6.7
  periods).
- The GPIO-driven-pad proof and the scaler threshold sanity pass on
  every run; two consecutive full runs at 30/30.

**40.6.14, both sequences.** With a pad walked by hardware while the
CPU is stopped (the chain is in [platform.md](platform.md), "Sleep,
peripheral by peripheral") and the comparator's own VDD scaler at half
supply as the threshold: a CONTINUOUS comparator with RUNSTDBY set woke
the device from STANDBY on its own edge in eight rounds of eight, at
about 14 ms - the moment the pad was due to move - against a 91 ms RTC
backstop. With RUNSTDBY CLEAR it woke nothing, in eight rounds of
eight, **whether GCLK_AC stopped with the CPU or was force-fed by a
generator that runs in standby**: COMPCTRL.RUNSTDBY gates the
COMPARATOR and not merely its clock. The force-fed arrangement did
produce ONE stray wake in thirty-two rounds across four runs, recorded
here and not rounded away. And 40.6.14.2's single-shot SleepWalking
runs: an RTC periodic event on the ASYNCHRONOUS path - all table 29-3
grants the SOC users - starts a comparison DURING a standby and its
INTFLAG is read at the wake.

## COMP2, COMP3, window 1, the bandgap and the hysteresis

From `test_samc_analog` letters g to j, 41 verdicts, with the DAC on
PA02 as the swept source - PA02 being AIN4, which is COMP2/3's PIN0.

**COMP2 and COMP3 run, and window 1 gets the arrangement 40.6.4
actually describes.** Where `test_samc_ac` had to swap the roles (the
scaler as the signal, two rail-driven pads as the limits) because no pad
on this board can sit between the rails, the DAC IS a signal between
them: both comparators share PA02 as the positive input and their own
VDD scalers - steps 16/64 and 40/64 - are the two limits.

| DAC code | COMP2 / COMP3 STATE | WSTATE1 |
|---|---|---|
| 100 | no / no | below |
| 500 | yes / no | inside |
| 900 | yes / yes | above |

`pair_consistent()` holds, WINCTRL takes the window under a running
block, WINTSEL = INSIDE fires on entering the band and stays silent on
leaving it upward, and the two comparators keep their own STATE
throughout.

**THE BANDGAP AS A NEGATIVE INPUT WORKS, AND IT DOES NOT NEED
SUPC.VREF.VREFOE.** The comparator flips at DAC code **203 with the bit
clear and 203 with it set**: 22.6.2.2's sentence is about routing the
reference to an ADC INPUT CHANNEL, which is the one path that really is
dead without it ([adc.md](adc.md)), while the AC's negative multiplexer
takes the bandgap internally exactly as the ADC's and the DAC's
REFERENCE paths do ([dac.md](dac.md)). `ac.hpp`'s own comment said
otherwise and is corrected.

Weighed by the DAC, the three SUPC levels cross at codes **203 / 405 /
812**, i.e. 1044 / 2083 / 4178 mV against a supply this suite locates at
5269 mV - a THIRD independent route to the same bandgap, after the AC's
own scaler and the ADC.

**ERRATUM 1.5.6 reproduces, rarely, with its own workaround as the
control.** Sixty-four enables of both the block and the comparator, with
the DAC held far below the threshold so the output cannot legitimately
change: MUXNEG = bandgap raised **1** COMP flag with STATE still clear;
MUXNEG = the VDD scaler at the same level raised **0**. Some runs see
zero on both. So the item is real and infrequent on this die, and the
driver's stated obligation - clear the flag after enabling, before
arming - is a real one.

**THE HYSTERESIS, in DAC codes and then in millivolts** (one DAC code is
about 5.1 mV here), sweeping the DAC on MUXNEG against the comparator's
own scaler at mid supply. Four repeats of one sweep land on the same
code, so the gaps below are signal:

| | up-flip | down-flip | gap |
|---|---|---|---|
| high speed, HYSTEN 0 | 513 | 512 | 1 code = 5 mV |
| high speed, HYSTEN 1 | 524 | 501 | 23 codes = **118 mV** |
| low power, HYSTEN 1 | 525 | 503 | 22 codes = **113 mV** |

Both are inside table 45-34's own bands (29..190 and 25..248, typical
100). **ERRATUM 1.5.1** - hysteresis present only for a falling
transition - is **revision B alone** on the E/G/J row and the bench
agrees with the row: against the hysteresis-free crossing at 513 the two
edges move out by **11 and 12 codes**, both of them. **ERRATUM 1.5.2**
makes the low-power/hysteresis pairing legal at this revision (it is
marked B..E) and the pairing behaves. **ERRATUM 1.5.4** - the
hysteresis specification itself being wrong - is B..E too, so table
45-34 as printed is the right band to judge against.

**40.6.10'S SWAP PROCEDURE RUNS, AND WHAT IT RETURNS IS A BOUND.**
Swapping the terminals inverts the output as well, so the two cancel and
the SENSE of the output is unchanged - which is what makes the recipe an
offset measurement rather than a polarity change, and what the first
version of this test got wrong. Unswapped and swapped, the crossing is
the same code (513, spread 0 on four repeats each), midpoint 513 against
a nominal 512: **half the difference is under one DAC code, so this
comparator's input offset is smaller than the 5 mV step of the only
source that can sweep it** - consistent with table 45-34's typical
-0.1/+1 mV, and the honest outcome of a procedure run with an instrument
coarser than the quantity.

**The directed INTSEL flavours are directed**: rising fires on the
output's rise and stays silent on its fall, falling does the opposite,
toggle takes both.

**INVEIx inverts a real event**, and like TSENS's STARTINV it needs a
LEVEL to be measurable at all. COMP0's output is one; COMP2 in
single-shot mode is started by it through SOC2 on the asynchronous path:

| | the level rises | the level falls |
|---|---|---|
| INVEI clear | a comparison starts | nothing |
| INVEI set | nothing | a comparison starts |

**`Ac::take_flags()` runs from a real handler.** The AC vector is bound
under the name the device header declares (`AC_Handler`); eight
crossings of the bandgap threshold produce eight vector entries and the
read-and-clear body returns the comparator's own bit.

## Not covered yet

Driver gaps (not built):
- **The window monitor in sleep.** 40.6.14's two sequences are measured
  for a single comparator (see "Bench findings"); a WINDOW across a
  standby, and 40.6.14's rule that both comparators of a pair must
  share RUNSTDBY, are stated and unexercised.
- **DBGCTRL policy.**

Implemented but not bench-verified:
- **AIN6 and AIN7** (PB05 and PB06), the two J-only inputs of the
  COMP2/3 pair: their per-package legality is compile-asserted per
  variant and neither pad has ever carried a comparison - the pair's
  measured facts all come through AIN4, which is the DAC's own pad.
- **Operation on the E and G variants**: compile-checked only.
