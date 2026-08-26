# OPAMP - analog signal conditioning (AVR DB only)

> **PROVISIONAL.** The whole chapter's register description is
> exposed and the whole option space that needs no external component
> is bench-measured - every ladder gain in both topologies, every
> internal source, both op-to-op links, the instrumentation recipe,
> the internal timer to the tick, the offset trim and all four event
> users. What remains is the integrator as a usage type (it needs an
> external R and C and a DUMP policy), standby behaviour, and the
> current figures the A4-only errata talk about; the list is in
> "Not covered yet". Documents of record: AVR128DB28/32/48/64 data
> sheet DS40002247B (OPAMP chapter 35, electricals 39.20 and typical
> characteristics 40.11, EVSYS 16 - generators 0x34-0x36 and users
> 0x2A-0x35, I/O multiplexing chapter 3), errata DS80000915F
> (items 2.8.1 and 2.8.2, both rev. A4 only). The DA family has no
> OPAMP peripheral and its errata document has no such section.
> Driver: `avrdx/opamp.hpp` (`OpampSystem`, `Opamp<n>`,
> `OpampFollower`, `OpampPga`, `OpampInvertingPga`,
> `InstrumentationAmp`); the event vocabulary is `EvOpampReady` and
> `EvOpampCtl` in `avrdx/evsys.hpp` ([evsys.md](evsys.md)); the DAC
> entries mean the BUFFERED DAC output ([dac.md](dac.md)). Reference
> test: `test_avr_opamp`.

## What the silicon does

Up to three operational amplifiers, OP0..OP2, wired to each other and
to the rest of the chip through analog multiplexers, so that a large
family of conditioning circuits needs no external component at all.
**The peripheral is DB-only**, and OP2 exists on the 48- and 64-pin
packages only - the device header is the authority (`opamp_count`
reads the presence of the OP2 registers in `OPAMP_t`, and is 0 on the
DA family, where the whole driver compiles to nothing).

Each op amp has three dedicated pads, one global block serves all
three:

| | INP (+) | OUT | INN (-) | present on |
|---|---|---|---|---|
| OP0 | PD1 | PD2 (AIN2) | PD3 | every DB package |
| OP1 | PD4 | PD5 (AIN5) | PD7 | every DB package |
| OP2 | PE1 | PE2 (AIN10) | PE3 | 48- and 64-pin |

PD6, between OP1's INP and OUT, is the DAC's output pad, and the ADC
reads every OUT pad as an ordinary analog input - which is what makes
the whole peripheral measurable from inside the chip. The driver turns
every pad it uses into an analog input (digital buffer off) and
`release()` gives back exactly the ones it took.

**The two input multiplexers** (`OPnINMUX`). MUXPOS chooses INP, the
op amp's own ladder wiper, the DAC, ground or VDD/2; OP1 and OP2 add
LINKOUT (OP[n-1]'s output) and OP2 alone adds LINKWIP (OP0's wiper).
MUXNEG chooses INN, the wiper, the op amp's own output (unity gain) or
the DAC - the same four everywhere. The DAC entries mean the
**buffered** DAC output, so `DAC.CTRLA.OUTEN` must be on and PD6 is
then the DAC's (34.3.2.3).

**The resistor ladder** (`OPnRESMUX`) is 16 units of R (4 kOhm each,
39-27). MUXTOP hangs its top on nothing, on OPnOUT or on VDD; MUXBOT
hangs its bottom on nothing, on INP, on INN, on the DAC, on ground or
on LINKOUT - which is OP[n-1]'s output and, **for OP0, OP2's**
(35.5.7 note 1), so that entry needs a package with OP2. MUXWIP splits
the 16R into R1 (below the wiper) and R2 (above it):

| MUXWIP | R1 | R2 | non-inverting 1 + R2/R1 | inverting -R2/R1 |
|--------|----|----|--------------------------|-------------------|
| WIP0 | 15R | 1R | 16/15 | -1/15 |
| WIP1 | 14R | 2R | 8/7 | -1/7 |
| WIP2 | 12R | 4R | 4/3 | -1/3 |
| WIP3 | 8R | 8R | 2 | -1 |
| WIP4 | 6R | 10R | 8/3 | -5/3 |
| WIP5 | 4R | 12R | 4 | -3 |
| WIP6 | 2R | 14R | 8 | -7 |
| WIP7 | 1R | 15R | 16 | -15 |

The gains are exact rationals and the driver keeps them so
(`OpampGain{num, den}`): calling the first one "1" would be a lie, and
`opamp_noninverting_wiper(5, 1)` returns nothing rather than a
neighbour, because the ladder has no gain of five.

**The output driver** (`OPnCTRLA.OUTMODE`) is OFF or NORMAL - the
other two codes are reserved, and this die stores only the low bit of
the two-bit field (see the findings). While a driver is on, the OUT
pad belongs to the op amp and no other peripheral can drive it.

**The internal timer.** `OPAMP.TIMEBASE` is one less than the number
of CLK_PER cycles that reach one microsecond (35.5.3), so one timebase
tick is a microsecond and `OPnSETTLE` counts 0..127 of them. When an
op amp starts, the timer waits the WARM-UP (the op amp's own
circuitry) and then the programmed settle time, and only then does
`OPnSTATUS.SETTLED` rise and the READYn event go out. **Any write to
OPnCTRLA, OPnINMUX or OPnRESMUX restarts the timer** - which is why
`init()` writes CTRLA last and why `restart()` exists as a verb.
Because TIMEBASE is a CLK_PER divider, `OpampSystem` is the chapter's
ClockUser: a `DynamicClock` that carries op amps lists it ONCE (there
is one TIMEBASE register, not three) and `rebase()` rewrites it, so a
settle time keeps meaning the microseconds it says.

**Three enable regimes** (35.3.2.7), which the driver names as one
enum: `software` (ALWAYSON alone - on, deaf to events, silent),
`event` (EVENTEN alone - the ENABLEn and DISABLEn events own it, and
READYn is issued), `software_with_events` (both - on by software,
DUMPn and DRIVEn heard, ENABLEn/DISABLEn ignored), plus `off`.

**Four event users and one generator.** READYn is a generator (0x34 +
n, a one-CLK_PER pulse). ENABLEn and DISABLEn are EDGE-detected users,
DUMPn and DRIVEn are LEVELS (table 35-2) - a channel driving a dump or
a drive must HOLD its level, and a software pulse does nothing there.
DUMPn closes a switch from VOUT to VINN (the integrator's reset,
figure 35-6); DRIVEn raises the output driver whatever OUTMODE says.
**The peripheral has no interrupt at all** (35.3.4).

**Offset calibration** (`OPnCAL`, 35.3.2.8). The register comes out of
reset holding the production value from the fuses; the chapter's
procedure is to make the op amp a voltage follower of a known source,
measure both with the ADC and move CAL by the difference.

**Power and range.** `PWRCTRL.IRSEL` trades the rail-to-rail input
common mode (-0.3 V .. VDD + 0.3 V) for VDD - 0.7 V and less current.
Errata DS80000915F **2.8.2** makes that bit read-only on silicon rev.
A4, with no work-around; **2.8.1** (also A4 only) says the peripheral
draws up to three times the specified current when the output sits
near a rail. Both are rev. A4 items and neither has a register face a
driver could work around, so the driver reports rather than promises:
`reduced_input_range(bool)` returns what the silicon actually took.

`DBGCTRL.DBGRUN` keeps the DIGITAL interface running while the CPU is
halted; the analog half runs regardless (35.3.6). `RUNSTBY` (the data
sheet spells it RUNSTDBY, the header does not) keeps an op amp alive
in standby sleep, with its enable behaviour unchanged.

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `OpampSystem` | `init(clock, reduced_range, debug_run)` (TIMEBASE for the clock, PWRCTRL, DBGCTRL, then the block's ENABLE; static_asserts `clock_follows`), `enable`/`disable`/`enabled`, `timebase()`/`timebase(v)`, `rebase(hz)` (the ClockUser hook) + `clock_hz_timebase()`, `reduced_input_range(bool)` -> what the silicon took / `reduced_input_range()`, `debug_run(bool)` |
| `OpampConfig` | `positive` (`OpampPos`: inp, wiper, dac, gnd, vdd_div2, link_out, link_wip), `negative` (`OpampNeg`: inn, wiper, out, dac), `top` (`OpampTop`: off, out, vdd), `bottom` (`OpampBot`: off, inp, inn, dac, link_out, gnd), `wiper` (`OpampWiper`: wip0..wip7), `output` (`OpampOutput`: off, normal), `mode` (`OpampMode`: off, software, event, software_with_events), `settle_us` (0..127), `run_standby` |
| `Opamp<n>` | `init<cfg>()` / `init(cfg)` (pads to analog, INMUX/RESMUX/SETTLE, then CTRLA last so the settle timer restarts with everything in place; the compile-time form static_asserts `opamp_config_valid`, the run-time form returns false, touching nothing), `release()` (op amp off, its pads back to PORT), `enable`/`disable`/`always_on`, `mode()`/`mode(m)`, `restart()` (rewrite CTRLA: re-arm SETTLED and READY without changing anything), `events(bool)`, `output()`/`output(m)`, `run_standby(bool)`, `positive(p)` -> bool / `negative(p)` / `wiper()`/`wiper(w)` / `ladder(top, bot, wip)`, `settle_us()`/`settle_us(v)`, `settled()`, `wait_settled()`, `cal()`/`cal(v)`, `enable_on(ch)` / `disable_on(ch)` / `dump_on(ch)` / `drive_on(ch)` and the four `*_off()`, the raw register references `ctrla()`/`status()`/`resmux()`/`inmux()`/`settle_reg()`/`cal_reg()`; types `ReadyEvent`, `EnableIn`/`DisableIn`/`DumpIn`/`DriveIn`, `inp_pin`/`out_pin`/`inn_pin` |
| `OpampFollower<Op>` | `init(source, settle_us, out, mode)`, `gain()` (1/1), `settled`/`wait_settled`, `release` |
| `OpampPga<Op>` | `init(wiper, source, settle_us, out, mode)`, `set(wiper)`, `gain()`/`gain_of(w)`, `for_gain(num, den)` -> `optional<OpampWiper>`, `settled`/`wait_settled`, `release` |
| `OpampInvertingPga<Op>` | `init(wiper, source as an OpampBot, settle_us, out, mode)`, `set`, `gain`/`gain_of`/`for_gain`, `settled`/`wait_settled`, `release` |
| `InstrumentationAmp<Op0, Op1, Op2>` | `init(gain, v2_source, v1_source, settle_us, observe_stages)`, `set(gain)`, `gain()`/`gain_of(g)`, `settled`/`wait_settled`, `release` - the type needs OP2 |
| helpers | `opamp_count` (evsys.hpp), `opamp_ladder_r1/r2(w)`, `opamp_noninverting_gain(w)`, `opamp_inverting_gain(w)`, `opamp_gain_x1000(g)`, `opamp_noninverting_wiper(num, den)`, `opamp_inverting_wiper(num, den)`, `opamp_inp_pin/out_pin/inn_pin(n)`, `opamp_config_valid(n, cfg)`, `opamp_timebase(hz)`, `opamp_timebase_ns(hz)`, `instrumentation_gain(g)`, `instrumentation_wiper_op0/op2(g)` |

What `opamp_config_valid` refuses, at compile time in `init<cfg>()`
and at run time in `init(cfg)`: an instance this package does not have,
MUXPOS LINKOUT on OP0, MUXPOS LINKWIP on anything but OP2, MUXBOT
LINKOUT on OP0 where there is no OP2 to feed it, a settle time above
127, and any pad the package does not bond.

## How to use it

A unity-gain buffer from PD1 to PD2:

```cpp
#include "avrdx/opamp.hpp"
using Buf = brio::OpampFollower<brio::Opamp<0>>;
brio::OpampSystem::init(clock);        // TIMEBASE, then the block's ENABLE
Buf::init();                           // MUXPOS = INP, MUXNEG = OUT, ladder off
Buf::wait_settled();
```

A non-inverting amplifier of exactly four, driven by the DAC and read
back by the ADC without a single wire:

```cpp
using Amp = brio::OpampPga<brio::Opamp<0>>;
constexpr auto w = *Amp::for_gain(4);          // WIP5, or a compile error
Amp::init(w, brio::OpampPos::dac);
```

An inverting stage about VDD/2, its input on the INN pad:

```cpp
using Inv = brio::OpampInvertingPga<brio::Opamp<1>>;
Inv::init(brio::OpampWiper::wip5);             // gain -3, input on PD7
```

The chapter's instrumentation amplifier, V2 on PE1's neighbour pads
and the difference on PE2:

```cpp
using Instr = brio::InstrumentationAmp<>;      // OP0, OP1, OP2
Instr::init(brio::InstrumentationGain::x7);    // V2 on PD1, V1 on PD4
Instr::wait_settled();
```

Started and stopped by events, with READY telling the rest of the chip
when the output is trustworthy - a timer capture, an ADC start:

```cpp
using Op = brio::Opamp<0>;
Op::init<brio::OpampConfig{.positive = brio::OpampPos::inp,
                           .negative = brio::OpampNeg::out,
                           .mode = brio::OpampMode::event,
                           .settle_us = 30}>();
Op::enable_on(brio::EventChannel<1>{});        // edge
Op::disable_on(brio::EventChannel<2>{});       // edge
brio::EventChannel<0>::source(Op::ReadyEvent{});
```

The integrator's mechanism (figure 35-6): the op amp open-loop with
its inverting input on the INN pad, an external R into that pad and an
external C from the pad to OUT, and a DUMPn LEVEL to discharge.

```cpp
Op::init({.positive = brio::OpampPos::vdd_div2, .negative = brio::OpampNeg::inn,
          .mode = brio::OpampMode::software_with_events});
Op::dump_on(brio::EventChannel<3>{});          // a LEVEL: while high, VOUT -> VINN
```

## Bench findings

`test_avr_opamp` on rev. A5 at a measured VDD of 4980 mV, 96 verdicts,
no wires: the DAC's buffered output is the source, the ADC reads every
OUT pad, a TCB latches the READY event through EVSYS and a PORT pin
supplies the level the DUMP and DRIVE users want. Both converters run
on VDD, so a DAC code c aims at ADC count 4c whatever the rail is.

**The register faces.** TIMEBASE comes out of reset at 1, as 35.5.3
says, and is seven bits. `OPnINMUX` written 0xFF reads back **0x77**:
bits 3 and 7 are unimplemented, as the two three-bit fields imply.
`OPnRESMUX` written 0xFF reads back 0xFF - the reserved MUXTOP code
0x3 and MUXBOT codes 0x6/0x7 are all storable. But `OPnCTRLA` written
with the reserved OUTMODE 0x3 reads back **0x4**: only the low bit of
the field sticks, so **OUTMODE is one implemented bit drawn as two**.
The three instances sit eight bytes apart and do not alias. The
production `OPnCAL` bytes on this die are 0x81, 0x7E and 0x86.

**IRSEL is WRITABLE on this silicon** - written 1 it reads 1, written
0 it reads 0. Errata 2.8.2 is listed for rev. A4 only, and rev. A5
confirms it: the erratum does not apply here, and the code's refusal
to promise anything about the bit is a statement about A4, not about
this board.

**The follower is exact.** OP0 following the DAC over codes 100..900
(0.48 V .. 4.38 V) tracks its own source to **0 mV at every point** -
within the ADC's own count, the op amp's offset does not show. VDD/2
as a source measures 2486 mV against a rail half of 2490 mV, well
inside the +-3 % of 39-27. A follower of GROUND lands at **13 mV**,
which is the output swing's real floor at a near-zero load - the
39-27 figure of 0.15 V is quoted at 1.5 mA.

**The non-inverting ladder is exact to a permille.** Measured as the
slope of OP0OUT against the DAC's own measured output, every wiper:

| MUXWIP | gain | measured x1000 | error |
|--------|------|----------------|-------|
| WIP0 | 16/15 | 1066 | 0 permille |
| WIP1 | 8/7 | 1143 | 0 |
| WIP2 | 4/3 | 1334 | 0 |
| WIP3 | 2 | 2004 | +2 |
| WIP4 | 8/3 | 2664 | -1 |
| WIP5 | 4 | 4002 | 0 |
| WIP6 | 8 | 7998 | 0 |
| WIP7 | 16 | 16013 | 0 |

against the +-3 % typical / +-10 % maximum "system gain accuracy with
internal resistor ladder" of 39-27. Two x2 stages cascaded through
LINKOUT measure 3.96 .. 3.98 instead of 4.

**The inverting ladder is a few percent off, and one-sided.** Same
method, MUXBOT on the DAC: -1/15 reads +29 permille, then -143 (0),
-336 (-9), -1020 (-20), -1701 (-20), -3116 (-38), -7376 (-53), -14997
(0). The magnitudes run high in the middle of the range, which the
non-inverting sweep of the same ladder does not show; the difference
between the two topologies is that here the ladder's bottom is driven
by the DAC's buffer instead of by ground. Still inside the +-10 %.
The stage pivots on VDD/2 to 11 mV.
**The sweep must stay ABOVE VDD/2**: the ladder's bottom is the DAC's
buffered output, which sources a milliampere but sinks about a
microamp ([dac.md](dac.md)), and only with the output below VDD/2 does
the ~75 uA of ladder current flow out of the DAC rather than into it.

**Both links carry what they say.** LINKOUT: OP1 following OP0's
output reproduces it to 1 mV (two offsets in series). LINKWIP: OP0
with its ladder from OP0OUT to ground at WIP3, OP2 following that
wiper, gives 1461 mV of OP0's 2921 - the 8/16 tap, exactly.

**The instrumentation amplifier makes all seven of table 35-14's
gains** (V2 = the DAC, V1 = OP1 following ground at 13-15 mV, slope
taken against the measured V2 - V1 so V1's own position and any
movement under the ladder's load are measured rather than assumed):
1/15 -> 0.063 (-59 permille), 1/7 -> 0.142 (-6), 1/3 -> 0.333 (0),
1 -> 1.002 (+2), 3 -> 3.041 (+13), 7 -> 7.163 (+23), 15 -> 15.516
(+34). At unity gain the end-to-end value is the difference itself:
V2 3894 mV, V1 2486 mV, output 1402 mV against a difference of 1408.
The seven are not a selection - the recipe needs
R1(OP2) = 16R/(1 + G) and R1(OP0) = 16R x G/(1 + G) to BOTH be wiper
positions, and exactly these seven satisfy it (a gain of 5/3 would
want an R1 of 10R, which the ladder does not tap).

**The internal timer is exact, and the warm-up is not what 39-27
suggests.** Timed ENABLE event to READY event, entirely in hardware:

| SETTLE | ENABLE -> READY | delta | expected |
|--------|-----------------|-------|----------|
| 10 us | 605 ticks (25 us) | | |
| 40 us | 1328 ticks (55 us) | 723 | 720 |
| 80 us | 2288 ticks (95 us) | 960 | 960 |
| 127 us | 3416 ticks (142 us) | 1128 | 1128 |

One SETTLE unit is **exactly one TIMEBASE microsecond** - two of the
three deltas land on the tick and the third is three ticks long. What
sits on top is a fixed **365 CLK_PER ticks = 15.2 us of WARM-UP**,
where 39-27's TON, the electrical turn-on, is 1 us typical. And the
warm-up is charged to STARTING an op amp, not to re-arming its timer:
a `restart()` of an op amp that is already running costs its SETTLE
plus **21 ticks**, not 365.

**READY is issued in EVENT_ENABLED mode ONLY.** With ALWAYSON set and
EVENTEN set (SW_ENABLED_WITH_EVENTS) the op amp hears DUMP and DRIVE
but generates nothing: SETTLED rises on time (80 us programmed, 80 us
measured) and no READY event ever reaches the channel. 35.3.2.6 is
exact where 35.3.3 reads wider. With EVENTEN clear there is no event
either, as expected.

SETTLED is cleared and the timer restarted by a write to **any** of
OPnCTRLA, OPnINMUX and OPnRESMUX - all three confirmed.

**TIMEBASE follows a clock rebase.** 24 -> 12 -> 24 MHz under a
running op amp: TIMEBASE reads 23, 11, 23 and a SETTLE of 80 us
measures 2272 / 1148 / 2288 ticks, i.e. 94 / 95 / 95 microseconds -
the tick count halves with the ruler and the TIME does not move.

**Offset calibration.** At the production CAL of 0x81 the follower's
residual (averaged over five source levels, eight extra bits of ADC
averaging) is **-470 to -490 uV**. Amplified by a x16 PGA, 128 CAL
steps move the input by 76 mV, so **one step is 594 uV** where 39-27
says 500. The DIRECTION is the opposite of the plain reading of
35.5.10, which calls 0x00 "the most negative value of offset
adjustment": on this die a RISING CAL moves the output DOWN, i.e. the
trim carries the sign of the inverting input. Scanning CAL coarsely
and then finely finds 0x80 as the best value on this part, and the
residual falls from -490 uV to **about 100 uV** - inside a tenth of an
ADC count. CAL is a plain RAM register: a reset reloads the fuse
value.

**A RUNNING op amp holds its OUT pad even with OUTMODE OFF.** With
every op amp disabled, a pull-up takes PD2 to the rail as expected;
with OP0 running, OUTMODE OFF and the same pull-up, the pad reads
**0 mV**. So "the output driver is disabled" does not mean the pad is
released - the pad is held, and the DRIVEn event is what raises the
NORMAL driver on top of that (level high: 2486 mV, the VDD/2 the op
amp was told to follow; level low: back to 0 mV). A software PULSE on
the same channel changes nothing, which is the level-vs-edge rule of
table 35-2 observed.

**ENABLE and DISABLE behave as 35.3.2.7 describes.** In EVENT_ENABLED
mode SETTLED is down until the first ENABLE event, the output then
holds VDD/2 to 4 mV, a DISABLE event drops SETTLED, and an ENABLE
event on an op amp that has already settled re-issues READY at once.

**DUMP, and the integrator without an integrator.** OP0 open loop (+
on VDD/2, - on the floating INN pad, ladder off) sits at the top rail,
4976 mV. With the DUMP level high the switch closes VOUT onto VINN and
the op amp becomes a follower of VDD/2: 2486 mV. When the level drops
the output does NOT snap back - milliseconds later it is still in
mid-range, because the floating INN pad has kept the dumped charge,
and over hundreds of milliseconds it walks away again at a rate that
differs from run to run. That is an integrator whose only capacitor is
the pad's own stray, and it is the clearest demonstration the DUMP
mechanism can give without external components.

## Not covered yet

Driver gaps:

- **The integrator as a usage type.** Figure 35-6 needs an external R
  and an external C, and the interesting part is the DUMP POLICY - who
  resets, on what event, how the reset is timed against a conversion.
  It is born with its first user (the Multislope work is the candidate);
  the mechanism is all in the resource today.
- The cascaded PGA configurations (figures 35-8 to 35-11) have no task
  of their own: they are two or three `OpampPga` / `OpampInvertingPga`
  chained through `OpampPos::link_out`, and one bench leg proves the
  chaining. A `CascadedPga` type waits for a user who wants the gain
  arithmetic done for it.
- Pin-level bonding inside a bonded port: the pad table is
  port-level, like every other driver's, and awaits the family device
  tables.

Implemented but not bench-verified:

- **RUNSTBY**: the bit is exposed and never exercised - it wants a
  sleeping app, and belongs with the standby matrix in
  [platform.md](platform.md).
- **DBGCTRL.DBGRUN**: set and read back, never observed under a halted
  CPU.
- **IRSEL's electrical effect**: writable on this die, and the input
  common-mode range it selects is not something this bench can measure
  (it needs an input driven above VDD - 0.7 V with the output watched).
  The current saving it buys is likewise unmeasured.
- **Errata 2.8.1** (up to 3x supply current with the output near a
  rail) is rev. A4 only and is a current figure: it cannot be observed
  on the A5 boards of this bench, and there is no work-around to
  encode.
- **DA silicon**: there is none - the family has no OPAMP - so the
  gating is verified by compilation on all four DA packages and by
  nothing else.
- The inverting ladder's few-percent gain error is measured but not
  explained; the suspicion is the DAC buffer driving the ladder's
  bottom, and separating it needs a source the bench does not have.
