# energy - was deferring DynamicClock on the SAM the right call?

An energy experiment on the question brio answered differently on its
two targets: the AVR stratum has `DynamicClock` (rebase fan-out, the
app speaks Hz); the SAM stratum deliberately does not (ruling: per-
peripheral GCLK channels make "one rate for everything" an AVR
assumption - the question reopens with its first real consumer). This
experiment measures, on the AVR - the target that CAN do both - whether
adapting the CPU clock to a variable compute load ever beats a fixed
clock plus sleep, in joules. The verdict feeds the SAM ruling and the
third target's design.

This directory is SELF-CONTAINED: both firmware halves, their shared
wire contract, the campaign driver, the analysis, and this document.
Nothing under docs/ references it (house rule: experiments document
themselves).

## The physics prior, written before any measurement

Neither family scales voltage with frequency, so active current is
I(f) = I0 + k*f and the energy for N cycles at frequency f is
V*N*(k + I0/f): SLOWER CLOCKS COST MORE PER CYCLE - the static term
smears over more time. From the AVR128DB datasheet (DS40002247B table
39-4, typ at 3.0 V): 4.4 mA at 24 MHz, 1.0 mA at 4 MHz, hence
k = 0.17 mA/MHz, I0 = 0.32 mA, and per-cycle energy at 3.3 V:

    24 MHz: 0.605 nJ   8 MHz: 0.693 nJ   4 MHz: 0.825 nJ
     2 MHz: 1.09 nJ    1 MHz: 1.62 nJ

Clock adaptation therefore cannot win on compute; its only theoretical
asset is the avoided cost of waking from sleep. But waking from STANDBY
was measured at +10..12 cycles on this bench (test_avr_power) - free -
while the standby watch-floor is the real bill (DS40002247B 39-5:
ADC0REF alone 175 uA in standby). Pre-registered prediction: with a
stimulus a peripheral can watch, the clock-adapting strategy never wins
a single point; the map's value is the margins and the measured floors.
The experiment tests the fixed-clock philosophy ON ITS HOME TURF - a
quiet regime no peripheral can watch (e.g. parsing a byte stream) is
the adapting strategy's honest best case and a possible follow-up.

Verified along the way: the DB has NO frequency-vs-VDD derating (24 MHz
is legal over the whole 1.8-5.5 V; the "Maximum Frequency vs. VDD"
table the NVMCTRL chapter cites does not exist in section 39).

## The scaffold: an event logger

A plausible LiPo product with two genuinely different compute regimes:

- QUIET (almost always): watch an analog line for activity.
- BURST (rate lambda): capture M samples at 1 kHz, process W cycles
  (filter + CRC-16), deadline T_proc. A missed deadline DISQUALIFIES
  the run - it is not "efficient".

Three strategies, one DUT binary (`avrdx/energy_logger.cpp`):

1. `sprint` - fixed 24 MHz; CPU in standby, the ADC free-running with
   its window comparator as the only wake. Peripherals watch.
2. `pace` - DynamicClock; quiet at 1-2 MHz always awake (software
   threshold), set(24M) on detection, back down after. Pays the
   switch/rebase cost (measured by its own letter) and the idle floor.
3. `static_low` - the minimum fixed clock that meets T_proc (4-8 MHz),
   standby + window-compare watching like sprint. The true SAM-side
   alternative: if it beats pace everywhere, the ruling stands.

Map axes: lambda in 0.2..50 Hz x W in 1e4..1e6 cycles, only points
feasible for ALL strategies. Corner letters: lambda->0 (the watching
floors) and duty-100% (measures the true I0 and k of THIS die).

## The instrument: the SAM C21 is three things at once

`samc/energy_meter.cpp` on bench board C:

- THE WORLD: its DAC (PA02) plays a seeded burst schedule as a real
  analog stimulus into the DUT's ADC pin - same seed, identical event
  history for every strategy, every voltage, both supply instances. The
  DUT is deliberately BLIND to the schedule: detection is real.
- THE JUDGE: the witness pin (DUT toggles once per processed burst,
  plus small pulse-count signatures for phase marks) is received on an
  AC COMPARATOR against the internal 1.024 V bandgap, hysteresis on,
  AC event -> EVSYS -> TC counting and timestamps. NOT on the EIC: the
  C21's VIH is 0.7*VDD (DS60001479M ch. 45, I/O Pins) = 3.57 V at its
  5 V supply - out of spec for a 3.0-3.3 V witness. Both board-to-board
  signals are therefore ANALOG, specified in absolute volts on internal
  references at both ends, immune to either supply.
- THE METER: two modes, two letters each with an in-place calibration.
  - `psu` mode (the map): a ~10 ohm Kelvin-sensed shunt HIGH-SIDE in
    the DUT's supply, read differentially by the SDADC (pair 0: INP0 =
    PA07 = supply side, INN0 = PA06 = DUT side; 1.024 V ref, chopper
    on). Free-running sigma-delta has no dead time and the SINC is
    linear, so averages are unbiased whatever the burst timing; the
    DUT's own bypass capacitance plus the shunt already integrate
    (charge is conserved when windows open and close in the same
    phase). Zero-calibration at window open AND close with the DUT
    parked in power-down (0.65 uA = a perfect zero at mV-scale offset;
    the SDADC's offset is common-mode driven, so it is constant at
    fixed V and subtracted). Sleep-phase energy is reintegrated
    analytically from static characterization runs - the one
    assumption, which the cap instance exists to validate. Calibration
    letter: known resistor loads in the DUT's place (also calibrates
    R_shunt in place, better than any DMM band).
  - `cap` mode (the cross-check): the DUT runs from a charged capacitor
    4.2 -> 3.0 V (a LiPo's full-to-empty window; no shunt - the sliding
    common mode would fake current through the SDADC's finite CMRR).
    The SAR watches V(t); scoring closes when V crosses a MEASURED
    3.00 V (the DUT's BOD stays armed at 2.85 V, SAMPLED mode, as a
    flash-safety net only - its guaranteed band is +-150 mV wide,
    unusable as an endpoint). Energy = 1/2 C (V0^2 - Vf^2) with Vf
    measured. Calibration letters: discharge through a known R (fits
    tau = RC, ~1% where the electrolytic's band is +-20%) and
    self-discharge (leakage vs V, subtracted).

Wires between the boards: supply (+ shunt in psu mode), stimulus,
witness, GND. Nothing else - the UARTs are per-board consoles to the
PC, silent on the DUT during measurement windows.

### Bench wiring, psu mode at 3.3 V

Both boards on USB. On the DUT: JP13's jumper OUT and the probe in its
place - upstream to the +3.3 V post, the probe's TWO downstream taps to
the VDD post and to VDDIO2 (JP10 out); JP2/JP3 (LEDs) open; the
Atmel-ICE attached only to flash, then unplugged. Flying wires, four:

    shunt sense upstream   -> meter PA07 (INP0)   } twisted pair
    shunt sense downstream -> meter PA06 (INN0)   }
    DUT PD2 (witness)      -> meter PA04 (AC AIN0)
    DUT GND                -> meter GND (short, explicit)

Positive current into the DUT then reads positive. For the offset
letter, both sense wires sit temporarily on the SAME shunt terminal
(a true differential zero at the operating common mode), then return
to their places.

A fifth wire is permanent: the probe's downstream node -> meter PB09
(ADC0/AIN3), the SAR's V tap - VMEAS reads it against INTREF at
4.096 V, where one 12-bit count is exactly one millivolt (the letter
switches SUPC.VREF.SEL for its duration and restores the 1.024 V the
SDADC and the AC live on).

Instrument constants, measured in place on this probe:
- R_shunt = 10.18 ohm +-1% (ratio calibration against a 1% 1.5k load:
  R = 1500 x V_shunt/V_load, no DMM and no supply assumption involved)
- SDADC offset at the 3.3 V common mode: +4.39 mV (raw 35926), noise
  21-24 uV rms with shorted sense - re-zeroed per window in real runs
- the board's AliExpress ADuM1201 clone draws ~1.3-1.5 mA quiescent on
  its VDD2 side (a genuine part is far below) - constant, so zero-cal
  absorbs it in psu mode; the rev 1.2 jumper takes it out for the
  uA-sensitive runs

## The DUT board (rev 1.1) and what the meter bills

The board's supply chain: USB VBUS (or Vin via an AMS1117-5.0) -> +5 V
-> AMS1117-3.3 -> +3.3 V; JP13 "cpu voltage" selects the micro's VDD
from either rail, and THE SHUNT SITS IN JP13's place. The meter bills
the DOMAIN downstream of it; the roster, from the schematic:

BILLED: the WHOLE micro - VDD, AVDD through the R3 10-ohm filter, AND
VDDIO2: the probe's downstream side has two taps and the second feeds
VDDIO2, so the MVIO domain (PORTC) is inside the shunt too and
VDDIO2 = VDD (a legal configuration, 1.62-5.5 V independent range).
Also billed: the ADuM1201's micro side (VDD2 + C21) - a constant
pedestal, dissected by the first two IMEAS letters and calibrated out
by the zero pair; the LED networks R4/R5 + D1/D2 only when their pins
drive (JP2/JP3 stay OPEN during runs); the RESET pull only while
pressed.

NOT billed, by construction: the power LED (on +5 V, upstream), the
CH340 (VBUS only), the 74LVC crystal buffer (unpopulated on the DB
board - it exists for the crystal-less DA). No unbilled path into the
micro remains.

JP14/JP15 open the console's TX/RX between the ADuM and the micro: the
signal-level console detach is a jumper, no wires pulled. The ADuM's
quiescent stays on the rail either way; board rev 1.2 plans a
dedicated jumper for the ADuM's VDD2 (from VDD as today, or from the
upstream 3.3 V for the uA-sensitive runs - selectable, NOT fixed
upstream, because a micro at 5 V via JP13 would put a 3.3 V-fed ADuM
output below the AVR's VIH and kill the console in that configuration).

CLOCKS: the DUT runs on internal oscillators ONLY (OSCHF, and OSC32K
for the tick) - `ClockSource::internal` never touches the crystal, so
it never draws; timing precision is the meter's job, and the meter's
board has the crystal. No framework change was needed for this: the
crystal-or-not decision is already the Clock type's compile-time
source parameter, on both architectures.

## Voltages

Map at 3.3 V (the regulated-LiPo rail), brackets at 4.2 and 3.0 V on a
few points to measure the V-dependence of the RANKING (expected
invariant - V multiplies all strategies alike; measured, not assumed).
Cap instance spans 4.2 -> 3.0. Datasheet typicals are quoted at 3.0 V,
so the cap endpoint sits on the characterization curves' own reference
point.

## The wire contract: energy_link.hpp

One header, included by both firmwares (the usart_link/spi_link
precedent): the witness vocabulary (both sides) and the seeded schedule
(METER ONLY - the DUT must not know the future). The Python side
reimplements the xorshift (the uart_stress precedent) and coherence is
VERIFIED per run: the meter prints the first schedule arrivals derived
from the seed, run.py compares them against its own generator - a
golden vector in every log; divergence invalidates the run.

## Campaign driver and analysis

- `run.py` - drives both consoles via tools/bench.py + the bench
  manifest. Human-centric: live per-slice progress (mean/max current,
  bursts seen/expected, deadlines), clean Ctrl-C (current run marked
  aborted, prior runs stay valid), resumable point lists. Cap mode
  walks the operator through the manual steps (charge, detach, enter).
- `logs/` (git-ignored) - one file per run, TWO layers: the raw console
  transcripts of both ends (the full truth, never summarized away) and
  parsed JSONL (slices, witness events, zero-cals; metadata: strategy,
  seed, lambda, W, V, R_shunt or C, timestamps, firmware id).
- `analyze.py` - works ONLY on logs, never on end-of-run summaries:
  the crossover map, per-phase attribution, offset drift, and the
  map<->cap cross-check joined on (lambda, W, strategy, seed).

## Status

Instrument and choreography complete and bench-validated; the first
map point is measured. The meter speaks IMEAS / VMEAS / ZERO / WIT /
WOPEN-WSTAT-WCLOSE (free-running energy windows, zero overruns) /
JARM-JLOG (signature + toggle decoding) / STIM-SSTAT (seeded DAC
bursts with the golden vector) / JVERDICT (per-burst latency or MISS).
The logger runs the full choreography with strategies 0 sprint,
1 pace, 2 static_low, 3 rehearsal, 4 sprint_duty. First point (seed
12345, 15 bursts, 50 ms, gaps 500+700 ms, 300 CRC iterations, 18.2 s
window, all deadlines met by all strategies):

    sprint (ADC free-running watch)  284.3 mJ   73 ms latency
    pace   (3 -> 24 MHz per burst)   232.4 mJ   79 ms
    static_low (6 MHz fixed)         234.0 mJ  296 ms
    sprint_duty (1 conv per tick)    168.6 mJ   74 ms

The pre-registered prediction survived one refutation and one repair:
the naive free-running watcher loses to clock adaptation (the watching
floor dominates), the frugal watcher beats it by 27% - HOW you watch
is worth a factor ~1.7, more than any clock choice. Two protocol bugs
were bench-caught and fixed on the way: a START swallowed right after
the SDADC re-enable (WOPEN now start-and-verifies) and the VMEAS
SUPC.SEL excursion gliching the DAC's shared bandgap into fake bursts
(the DAC parks at zero during V measurements).

Not built yet: run.py and the two-layer logs (a hand-driven
first_point.py stands in), the map sweep, the 4.2/3.0 V brackets, the
capacitor instance. The ADuM-on-the-domain problem gates the cap
instance only PARTLY: ranking cross-checks work with the clone
subtracted analytically (its I(V) characterized at the bracket
voltages, the correction integrated over the SAR's recorded V(t)),
but the lambda->0 corner - the sleep-assumption validation, the cap
instance's main job - needs the ADuM physically out: a ten-minute
bodge on rev 1.1 (lift VDD2, strap it upstream of the jumper), which
rev 1.2 merely turns into a jumper. Cap-mode command-and-config goes
through EEPROM anyway (configure under USB, persist, boot autonomous
from the cap): a depowered ADuM costs cap runs nothing.
