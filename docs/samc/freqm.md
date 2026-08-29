# FREQM - Frequency Meter (SAM C21)

> **PROVISIONAL.** The chapter is small and fully built; what is left out
> is the DONE interrupt as a wake source and the GCLK_IO pins as
> measurement inputs. The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 44 - and
errata DS80000740S item **1.24.1, live on every silicon revision
including this one, with no workaround offered**. Driver:
`samc/freqm.hpp`. Family fixture `test/family_samc/freqm.cpp` plus one
negative under `tools/check_samc.sh`; the bench suite is
`test_samc_freqm`.

There is no AVR analog: on that family a clock is measured by counting
its edges in a timer with the CPU in the loop. This block does it in
hardware, and it earns its place because everything the clock work ahead
must characterize - the 32 kHz oscillators, the FDPLL, a crystal against
the internal RC - is exactly a ratio between two generators.

## What the silicon does

**One formula.** The block counts periods of the measured clock for
REFNUM periods of the reference clock and leaves the count in VALUE
(44.6.1):

    f_msr = VALUE / REFNUM x f_ref

Both clocks are generic clock channels of their own - GCLK_FREQM_MSR is
channel 3, GCLK_FREQM_REF channel 4 - so what gets measured, and against
what, is a GCLK routing question. The driver therefore takes GENERATOR
NUMBERS, not sources: anything a generator can be pointed at is
something this can measure.

**The reference must be slower than the measured clock.** The chapter
says so in a call-out box (44.6.2.1) and the arithmetic says why: VALUE
counts measured periods per reference period, so a faster reference
counts zero - which is indistinguishable from a stopped measurand.

**VALUE is 24 bits and the overflow is the caller's to avoid.**
REFNUM x (f_msr/f_ref) must stay under 2^24, and STATUS.OVF says when it
did not. Longer measurements are more precise and closer to overflowing;
that trade is the whole of configuring this block.

**Reading CTRLB raises a PAC protection error** (erratum 1.24.1, every
revision, no workaround). START is therefore written and never read - not
in a read-modify-write, not to check whether a measurement began. The
device header agrees, declaring CTRLB write-only. The consequence worth
stating is that **there is no way to ask the block whether a start was
accepted**: STATUS.BUSY and INTFLAG.DONE are the only evidence a
measurement is running or finished.

**CFGA is eight bits wide, whatever 44.8.3 draws.** The chapter draws
CFGA as sixteen bits with a DIVREF field at bit 15 that "divides the
reference clock by 8"; the device header's `FREQM_CFGA_Msk` is 0x00FF.
Measured, the header is right - see "Bench findings". The driver refuses
a configuration that asks for the divider rather than accepting one that
would silently do nothing.

**Precision is one part in VALUE.** The count is a whole number of
measured periods over a whole number of reference periods, so the
granularity is about f_msr/VALUE. The reference's own accuracy is a
separate matter and counting longer does not improve it: this block
measures a RATIO, and the absolute number it becomes is only ever as
good as the f_ref the caller names.

## Types and verbs

**`FreqmConfig`** - `measured_generator`, `reference_generator` (GCLK
generator numbers), `refnum` (1..255, the measurement's duration in
reference periods) and `divide_reference` (kept as vocabulary and
refused - the silicon has no such field).

**`Freqm`** - the block, monostate.

- *Constants*: `gclk_measured` (3) and `gclk_reference` (4) from the
  device header, `value_max`, `cfga_divref` (the bit the chapter draws
  and the silicon does not implement), `irq()`.
- *Budget*: `refnum_for(expected_ratio)` returns the largest REFNUM
  whose count still fits in VALUE; `config_valid()` refuses a zero
  REFNUM, a generator that does not exist, one generator asked to be
  both clocks, and the absent divider.
- *Claim and release*: `init(cfg)` connects both channels, resets,
  writes CFGA and enables - **in that order**, because CFGA is
  enable-protected and because the reset's own synchronization runs on
  the clocks the channels supply. `release()` hands everything back.
- *Plumbing*: `bus_clock`, `reset`, `enable`, `enabled`, `busy_sync`,
  `wait_sync`.
- *Status*: `status`, `running` (BUSY), `overflowed` (OVF),
  `clear_overflow`, plus `flags`/`armed`/`clear_flags`/`arm`/`disarm`/
  `done_flag`/`isr`.
- *Measuring*: `start()` (a write to CTRLB and nothing else),
  `value()`, `measure()` (start, wait, return the count - nothing on an
  overflow or a wait that never ended), and `to_hz(count, reference_hz,
  refnum)`, whose product is formed in 64 bits because a 24-bit count
  times a kilohertz reference leaves 32.

Note the division of labour with `samc/clock.hpp`: this header enables
the block's APB clock and connects its two channels, but never
configures a generator. What a generator is sourced from is the clock
driver's business.

## How to use it

**Measure a fast clock against a slow one**, which is the shape the
block imposes:

```cpp
Gclk<5>::configure(GclkConfig{.source = GclkSource::osculp32k});

const uint8_t refnum = Freqm::refnum_for(48'000'000 / 32768);
Freqm::init(FreqmConfig{.measured_generator = 0,
                        .reference_generator = 5,
                        .refnum = refnum});
if (auto count = Freqm::measure()) {
    const uint32_t hz = Freqm::to_hz(*count, 32768, refnum);
}
Freqm::release();
```

**Characterize the slow clock instead**, by reading the same ratio the
other way up - the block always needs the reference to be the slower of
the two, so a slow oscillator is measured by making it the measurand and
inverting:

```cpp
// count = REFNUM x f_fast / f_slow  =>  f_slow = REFNUM x f_fast / count
const uint32_t slow_hz = (uint64_t(refnum) * 48'000'000) / *count;
```

## Bench findings

From `test_samc_freqm` (4 letters, 25 verdicts, 25/25). Nothing to wire,
and this suite could not need wires in principle: every clock it
measures is inside the chip. It is also the first thing in this stratum
to run a GCLK generator other than 0 on silicon.

- **The measurement agrees with a second route to three parts in ten
  thousand - but the two routes SHARE THE SCALE.** OSCULP32K measured
  here against OSC48M reads **32957 Hz**; `test_samc_platform` letter c
  measured the same oscillator through the watchdog's early warning
  timed against SysTick and implies 32960 Hz, 3 Hz apart. This page
  first read that as two witnesses sharing no mechanism. IT IS NOT:
  SysTick runs on CLK_MAIN, which is OSC48M, so both numbers are ratios
  against the SAME RC times a nominal 48 MHz. What the 3 Hz agreement
  proves is the CONSISTENCY of the two measurement chains, not the
  frequency - and the crystal, measured later, says OSC48M is about
  5100 ppm SLOW on this die ([clock.md](clock.md)), which rescales both
  readings to about **32907 Hz**.
- **OSCULP32K runs fast of its nominal 32768 Hz** - about 5 to 6 per
  mille on the OSC48M-nominal scale these suites report in, about 4 per
  mille on the crystal's scale ([clock.md](clock.md)). Every timeout
  built on it - the watchdog above all - inherits that.
- **CFGA has no DIVREF, twice over.** Written with bit 15 set and
  REFNUM 1, CFGA **reads back 0x0001** - the bit does not even stay
  written - and setting it changes no measurement. The register is eight
  bits wide as the device header says, and 44.8.3's sixteen-bit drawing
  is wrong for this silicon.
- **REFNUM scales the count, to within the reference's own wander.**
  Doubling REFNUM from 64 to 128 doubles the count to within 0..9 parts
  in 10000 across runs, occasionally more. A permille band on that
  verdict was too tight and made it FLAKY; the band is the oscillator's
  and not the meter's, and an arithmetic fault would miss by a factor
  rather than by a third of a percent. Through a FOUR-cycle window the
  same test misses by anywhere from 0 to 34 parts in 10000 between runs,
  which is the same wander seen through too few of the reference's
  cycles to average it. A ratio test is immune to the reference's absolute error but
  not to its drift between the two measurements.
- **`refnum_for()`'s budget is where the overflow really is**: a
  measurement at the computed REFNUM completes with its count under
  2^24, and OSC48M against OSCULP32K saturates the field at 255 before
  the overflow edge is reachable at all.
- **An initialization order that looks arbitrary is not.** Resetting the
  block before its GCLK channels are connected leaves SYNCBUSY.SWRST
  standing forever, and every measurement then returns nothing - which
  is exactly how the first version of this driver failed here. SWRST is
  synchronized into a clock domain those channels feed.


- **A measurement finishes while the CPU sleeps.** This block has no
  RUNSTDBY bit and does not need one: with the measured clock on the
  crystal generator and the REFERENCE on OSCULP32K - the opposite of
  every other use of the meter here, and the point, since the window is
  REFNUM *reference* periods - a measurement started awake ran through
  a STANDBY and its DONE interrupt was the wake. REFNUM 128 gives a
  3878 us window; the sleep lasted 3895 us and the answer (24004949 Hz)
  agreed with the same measurement taken awake (23987157 Hz). Details
  in [platform.md](platform.md), "Sleep, peripheral by peripheral".

## Not covered yet

Driver gaps (not built):

- **GCLK_IO pins as measurement inputs** (44.5.1): measuring an external
  clock needs a pin claim this header does not make.
- **A `DynamicClock` consumer.** The obvious use of this block is to
  verify a clock switch actually happened, and there is no dynamic clock
  on this target to verify.

Implemented but not bench-verified:

- Operation on the E and G variants: compile-checked only. Neither the
  block nor its channel numbers vary by package.
- References other than OSCULP32K, and measurands other than generator 0
  and generator 5. The routing is generator-agnostic by construction,
  but only those have run.
