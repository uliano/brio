# AC (SAM C21)

> **PROVISIONAL.** A deliberately MINIMAL driver, built to answer one
> timing question at the bench (the latency of the synchronized output
> path, which ch. 40 never quantifies) and shaped so the full AC
> campaign can grow on it: the block, the four comparators in
> continuous and single-shot mode, the VDD scaler, filters, both
> output routings, flags. Window mode, events, sleep and per-package
> input legality are declared, not built - the list is in "Not
> covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 40 and
the electrical characteristics table 45-34 (whose note 4 states the
propagation delay is measured on "ACOUT (AC direct output)" and
covers ONLY the analog path - the digital path's cost in cycles is
stated nowhere, which is what the probe below measured). Driver:
`samc/ac.hpp` (`Ac` block + `AcComparator<n>`). Family fixture
`test/family_samc/ac.cpp` plus a negative under
`tools/check_samc.sh`; the bench instrument is the probe app
`ac_sync_probe` (30 verdicts, wireless).

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
as 1000 randomized shots, on three different clockings: OSC48M/4096
on the 16-bit generator, the fixed /512 the 8-bit generators' DIVSEL
turns out to be, and OSCULP32K (a clock the CPU does not share).
Floor 2.05 periods, ceiling 3.03, the ~0.05 being the measurement
chain's own constant (~210 CPU cycles, bounded by the ASYNC control
run). Both edges behave identically.

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

**Enable-protection is per comparator.** Every COMPCTRLn field is
writable only while that comparator's own ENABLE is low; ENABLE
itself is write-synchronized (SYNCBUSY.COMPCTRLn), as are
CTRLA.SWRST/ENABLE and WINCTRL. Disabling the whole block
(CTRLA.ENABLE = 0) stops every comparator but leaves their
COMPCTRLn.ENABLE bits standing.

**A GCLK fact found on the way** (recorded in `samc/clock.hpp` where
the config lives): GENCTRL.DIVSEL does NOT mean "divide by
2^(DIV+1)" on this family - the divisor becomes 2^(N+1) where N is
the WIDTH of that generator's DIV field, a FIXED 512 for the 8-bit
generators (measured exactly: DIV = 11 with DIVSEL set gave /512),
131072 for generator 1. Any other ratio needs the linear divider,
and only generator 1's reaches past 255 (table 16-3).

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
- **`AcComparator<n>`** - one of the four: `configure(AcConfig)`
  (disables first - every field is enable-protected), `enable()` with
  the synchronized wait, `scaler(v)` (VDD x (v+1)/64, writable live),
  `start()` (single-shot), `ready()`, `state()`, flag and arming
  verbs. A fifth comparator is refused at compile time.

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

The numbers behind the statements above, all from `ac_sync_probe`
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

## Not covered yet

Driver gaps (not built):
- Window mode: WINCTRL, WSTATE, the WIN flags and interrupts.
- The event surface: EVCTRL is never written (COMPEO/WINEO outputs,
  COMPEI/INVEI inputs) - no EVSYS driver exists on this target yet.
- Sleep: RUNSTDBY is a config field and nothing more; the 40.6.14
  SleepWalking sequences have no owner until the power pass.
- Per-package input legality as compile-time refusals: on the E and
  G packages COMP2/3 bond only AIN[5:4] (40.1) - the device-table
  job the full campaign owns. The driver compiles on E/G/J; only
  COMP0 is bench-proven.
- DAC as the negative input (no DAC driver on this target), INTREF
  level selection (SUPC.VREF is untouched), offset compensation via
  SWAP as a procedure.
- DBGCTRL policy.

Implemented but not bench-verified:
- Comparators 1..3 (compile-checked; COMP0 carries every measured
  fact), hysteresis, low-power speed, SWAP, rising/falling INTSEL
  flavours (toggle and end-of-comparison are exercised),
  `Ac::take_flags()` from a real handler (the probe polls).
