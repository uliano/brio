# CLKCTRL - the clock controller (AVR DA, DB and EA)

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B
(CLKCTRL chapter, electricals 39.10 and 39.3), errata DS80000915F
(2.5.1-2.5.4); for the DA parts DS40002183A (CLKCTRL chapter 11,
electricals 37.5.1) and DS80000882C (2.4.1); for the AVR64EA28/32/48
DS40002443A (chapter 12, the OSCCFG fuse 8.8.2.3, electricals 35.10) and
DS80001048D (no CLKCTRL item) - the EA's chapter is its own section
below. Driver: `avrdx/clock.hpp` (resources `Oschf`, `Osc32k`,
`Xosc32k`, `Xoschf`, `Pll`, `MainClock`, `ClockTimebase` (EA),
`ClockFailure`; tasks `Clock`, `DynamicClock`), `avrdx/delay.hpp`; the target-independent contracts in
`util/clock.hpp`. The clock MODEL (one rate truth, static and dynamic
regimes, the rebase fan-out) is [clock.md](../design/clock.md); this page is the
peripheral. Reference test: `test_avr_clock` (CLKOUT on PA7 to an
oscilloscope, on the board whose PA7 is free); the dynamic regime is bench-verified under a running
console (24 -> 12 -> 2 MHz at 115200).

## What the silicon does

The clock controller feeds CLK_MAIN, and from it CLK_CPU and CLK_PER
(the same rate on this family), from one of four sources through a
prescaler; other clocks run asynchronously to it (the RTC's, the WDT's,
the BOD's, the TCD's). All clock-control registers are protected
(CCP: a key written to CPU.CCP within four instructions of the access
- `_PROTECTED_WRITE`). After any reset the device runs from OSCHF at
4 MHz (or from OSC32K, per the OSCCFG fuse).

**Sources.**

| Source | What | Rates | Notes |
|--------|------|-------|-------|
| OSCHF | internal HF oscillator | 1, 2, 3, 4, 8, 12, 16, 20, 24 MHz | calibrated +-2 % (60 C), +-3 % (85 C), +-5 % over VDD and temperature at >= 4 MHz; 1-3 MHz +-6..10 %, uncalibrated. Manual tune -32..+31 steps of ~0.4 %; auto-tune against a 32 kHz crystal. Starts in 24-30 us from idle/standby, 220-600 us from power-down |
| OSC32K | internal 32.768 kHz ULP | 32.768 kHz | 29.5 .. 36 kHz (+-10 %); also feeds RTC/WDT/BOD; 950 us start-up from power-down |
| XOSC32K | crystal on PF0/PF1, or a 32.768 kHz clock on PF0 | 32.768 kHz | start-up 1k..64k cycles selectable (300 ms typ., 1 s in low-power mode); load 18 pF (8 pF low-power); the RTC's precise clock and OSCHF's auto-tune reference |
| XOSCHF | crystal on PA0/PA1 (DB only), or a clock on PA0 | crystal 4-24 MHz, clock up to 32 MHz | frequency-range setting (drive), start-up 256/1k/4k cycles; the pins leave the PORT while enabled |
| PLL | x2 or x3 from OSCHF or XOSCHF | in 16-24 MHz, out 32-48 MHz | lock ~10 us; clocks the TCD ONLY - it is not a main-clock source; runs only while the TCD requests it. With SOURCE = XOSCHF it runs only from an external CLOCK, never a crystal (errata 2.5.4): `Pll::start` refuses that combination |

**Main clock.** `CLKSEL` picks OSCHF / OSC32K / XOSC32K / EXTCLK (the
XOSCHF block: crystal or external clock per its source bit); the
prescaler divides by 1, 2, 4, 6, 8, 10, 12, 16, 24, 32, 48, 64;
CLK_MAIN must not exceed 24 MHz (at any VDD from 1.8 to 5.5 V).
Switching is safe at run time: the source to switch TO must be running
(an external one stable) before it is selected - a selected source
that never toggles leaves a switch pending that only a reset clears;
SOSC says a switch is in progress. `CLKOUT` puts CLK_PER on PA7.
Each oscillator has a RUNSTDBY bit: on, it runs even when nothing
requests it (so a later request sees no start-up time) and in standby
sleep; off, it runs only when requested. The main clock itself always
runs in active and idle, and in standby only if requested.

**Clock failure detection.** The CFD watches one source - the main
clock, XOSCHF or XOSC32K - for edges; after 8 reference-clock cycles
of silence it sets a flag, raises an interrupt (regular `CLKCTRL_CFD`
or the NMI) and, if the main clock is the one watched, forces CLKSEL
back to the start-up source (OSCHF at its reset rate, 4 MHz) and
disables CLKOUT. A test bit forces the condition. The interrupt
re-triggers every ten OSC32K cycles while the condition holds. Once
enabled with the NMI type, the CFD registers are read-only until a
reset - a lock-in for safety-critical use.

**Errata (DS80000915F).** 2.5.1 (A4): EXTS not set for an external
clock with RUNSTDBY and no requester - request it (RTC/TCD) before
checking. 2.5.2 (A4): RUNSTDBY does not keep external sources on in
sleep. 2.5.3 (A4/A5): PLLS never set with RUNSTDBY and no requester.
2.5.4 (A4/A5): the PLL does not run from an XOSCHF crystal, only from
an external clock. Bench silicon: A5.

## Types and verbs

Resources - thin typed views of the controller's blocks (all writes
through CCP):

| Resource | Verbs |
|----------|-------|
| `Oschf` | `set_hz(hz)` (one of the nine rates; immediate, also while it is the main clock; false, touching nothing, for a rate OSCHF does not produce), `run_standby(bool)`, `autotune(bool)` (needs a running XOSC32K; the clear waits 5 us for the auto-tune's value to land in TUNE, so a `tune()` right after it holds), `tune(-32..31)` / `tune()`, `stable()` |
| `Osc32k` | `run_standby(bool)`, `stable()` |
| `Xosc32k` | `start_crystal(Xosc32kStartup, low_power, run_standby)`, `start_external(run_standby)` (both stop a running oscillator first: CSUT/SEL are latched while enabled, so the new configuration always lands - at the price of the start-up time again), `stop()`, `enabled()`, `stable()`, `wait_stable(spins)`, `force_on(bool)` / `forced_on()` (RUNSTDBY on a running oscillator). `stable()` answers only while something REQUESTS the oscillator or it is forced on (below) |
| `Xoschf` (DB) | `start_crystal(hz, XoschfStartup, run_standby)`, `start_external(hz, run_standby)` (both stop first: SELHF/FRQRANGE/CSUTHF are latched while enabled), `stop()`, `enabled()`, `stable()`, `wait_stable(spins)`, `range_for(hz)` |
| `Pll` | `start(PllSource::oschf/xoschf, PllMultiplier::x2/x3, run_standby)` -> bool (false, writing nothing, when the source cannot drive the PLL on this silicon: errata 2.5.4), `source_ok(PllSource)`, `stop()`, `locked()`, `multiplier()`, `source()`; the arithmetic `pll_output_hz(in_hz, mul)` (0 outside the specified 16-24 MHz in / 32-48 MHz out) |
| `MainClock` | `select(MainSource::oschf/osc32k/xosc32k/extclk)` -> bool (switch completed), `source()`, `switching()`, `prescale(ClockDiv)`, `clkout(bool)` (PA7) |
| `ClockFailure` | `watch(CfdSource::main/xoschf/xosc32k)`, `stop()`, `interrupt(on, nmi)`, `test(force)`, `failed()`, `clear()`, ISR body `cfd()` -> the main source now in effect |

Tasks - what an application names ([clock.md](../design/clock.md)):

| Task | Verbs |
|------|-------|
| `Clock<ClockSource, source_hz, ClockDiv, oschf_hz>` | `init()` -> bool (running from the requested source), `matches_fuse()` (true where FRQSEL sets OSCHF's rate; the EA's one fuse read), `hz`, `is_static = true`; `oschf_hz` the fallback's OSCHF rate, the source's own on the DA/DB and the fuse's claim on the EA (below); sources `internal`, `crystal` (DB, EA), `external` (DB via XOSCHF; DA directly on PA0, asked of TCD0 first because EXTS answers only while the clock is requested, and selected only when it answers), `osc32k`, `xosc32k` (forced on while it is waited for, the force dropped once the main clock requests it) |
| `DynamicClock<Boot, Users...>` | `init()`, `hz()`, `set<hz>()` / `set(hz)` -> bool, `can_run_at(hz)`, `rebases<U>`; `is_static = false` |
| vocabulary | `ClockDiv`, `clock_divisor()`, `div_for()`, `oschf_frqsel()` (DA/DB), `clock_timebase_for()`, `MainSource`, `CfdSource`, `PllSource`, `PllMultiplier`, `Xosc32kStartup`, `XoschfStartup` |
| waits | `delay_us(clock, us)`, `delay_us_runtime(cycles_per_us, us)`, `cycles_per_us(hz)`, `delay_cycles(n)` (raw cycles: for clocks below 1 MHz) (`avrdx/delay.hpp`) |

## How to use it

**The static main clock** - the one line every application has:

```cpp
using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;   // PA0/PA1 crystal
constexpr SysClock clock;
int main() {
    const bool from_crystal = SysClock::init();   // false: OSCHF at 24 MHz instead
    Serial::init(clock, 460800);
    ...
}
```
Other fixtures: `Clock<ClockSource::internal, 4'000'000>` (no external
parts, the reset rate), `Clock<ClockSource::external, 16'000'000>` (a
clock on PA0), `Clock<ClockSource::internal, 24'000'000, ClockDiv::div6>`
(24 MHz oscillator, 4 MHz CLK_PER), `Clock<ClockSource::osc32k, 32'768>`
(a 32 kHz main clock for a sleepy application - the USART cannot
follow it, and a 1024 Hz tick interrupt cannot be served at 32 cycles
per tick: `Ticker::pause()` first, `delay_cycles()` to wait, the RTC
hardware keeps counting).

**The dynamic regime** (rate changes under a running program):

```cpp
using Boot = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
using SysClock = brio::DynamicClock<Boot, Serial, Adc<0>>;   // the users it rebases
SysClock::set<4'000'000>();          // users rebased, then the prescaler
```

**A 32 kHz crystal** for the RTC and for OSCHF auto-tune, started
FORCED ON - nothing requests it yet, and unrequested it does not run:

```cpp
Xosc32k::start_crystal(Xosc32kStartup::cycles64k, false, true);   // forced on
...
if (Xosc32k::wait_stable()) Oschf::autotune(true);   // OSCHF trimmed against it
Ticker::init();                      // the RTC driver picks XOSC32K when it is stable
```

**Trimming OSCHF by hand** (a few tenths of a percent per step):

```cpp
Oschf::tune(+3);                     // ~+1.2 %
```

**CLK_PER on a pin** (PA7) to measure it:

```cpp
MainClock::clkout(true);
```

**Clock failure detection** in a safety-minded application: watch the
crystal, take an interrupt, let the hardware fall back to OSCHF, and
let a supervisor decide.

```cpp
ClockFailure::interrupt(true);              // regular interrupt (nmi=true locks the config)
ClockFailure::watch(CfdSource::main);
ISR(CLKCTRL_CFD_vect) { brio::post<Supervisor>(ClockLost{ClockFailure::cfd()}); }
// in Supervisor: every clocked driver is now off-rate (OSCHF 4 MHz):
// rebase or re-init, or degrade gracefully; the kernel tick (RTC) is unaffected.
```

**The PLL** - the TCD is its only consumer on this silicon, and the
only way to clock a timer above CLK_MAIN's 24 MHz ceiling. Set OSCHF
to 16-24 MHz first (it is the PLL's input, not the main clock's), then
start the PLL and point the TCD at it ([tcd.md](tcd.md)):

```cpp
Oschf::set_hz(24'000'000);                                    // the PLL's input
Pll::start(PllSource::oschf, PllMultiplier::x2);              // 48 MHz out
TcdPwm<TcdRoute::def>::init(clock, {.clock = TcdClock::pll, .source_hz = 48'000'000,
                                    .hz = 100'000, .dead_time_ticks = 24});
// PLLS sets as soon as the TCD requests it; it stays 0 until then.
```

## Bench findings (`test_avr_clock`, rev A5, CLKOUT on PA7)

- Every OSCHF rate (24, 20, 16, 12, 8, 4, 3, 2, 1 MHz) and every main
  prescaler (24 MHz / 1 .. 64, down to 375 kHz) appears on CLKOUT as
  expected; the console (9600 baud, retuned before each switch)
  follows all of them.
- The 24 MHz crystal reads 23.995 MHz on the bench scope (the
  instrument's 200 ppm, not the crystal's).
- OSCHF manual tune at 16 MHz is NOT 0.4 %/step over the range: -32 ->
  14.56 MHz (-8.8 %), -16 -> 15.22 (-4.7 %), 0 -> 15.97 (-0.2 %), +31 ->
  17.96 (+12.4 %): about 0.28 %/step downward, 0.4 %/step upward (the
  data sheet's "0.4 % typ." holds upward). The suite retunes its
  console to these measured rates and stays readable at every step.
- OSC32K as the main clock runs (and the Ticker must be paused: a
  1024 Hz ISR cannot be served at 32 cycles per tick).
- XOSC32K never reports stable on the AVR128DB48 board (no 32 kHz
  crystal fitted), forced on or not.
- On the AVR128DA48 board (REVID 0x17), with a 32.768 kHz crystal and an
  external 24 MHz clock fitted: THE TWO EXTERNAL SOURCES ANSWER ONLY
  WHEN REQUESTED. XOSC32K enabled with RUNSTDBY clear never reported
  stable; forced on (RUNSTDBY) it did, a little over a second at 64k
  cycles of start-up. With a clock on PA0 nobody had requested, EXTS
  read 0; with TCD0 on EXTCLK and enabled, EXTS was set within three
  polls - the probe `Clock<external>::init` now makes on the DA before
  it selects the source, and the main clock then runs from it (the
  console at 460800 on the external 24 MHz).
- Clock failure, forced with the test bit while watching the main
  clock: CLKSEL falls back to OSCHF AND the OSCHF frequency is reset to
  4 MHz (FRQSEL reads 0x3 afterwards - the data sheet's "Reset
  frequency" is literal), CLKOUT is disabled by hardware, the regular
  interrupt fires every 305 us (= 10 OSC32K cycles: 1638 hits in 500
  ms) while the condition holds; clearing the test bit, the flag, and
  re-initialising the clock recovers fully.
- MCLKSTATUS: OSCHFS reads 0 while OSCHF is not the main clock and
  nobody requests it, RUNSTDBY notwithstanding - the status follows the
  REQUEST, as the register note says of the signal; it reads 1 whenever
  OSCHF is the main clock. PLLS stays 0 without a requester (the TCD).
- **An oscillator's RUNSTDBY, measured through sleep** (`test_avr_sleep`,
  numbers in [platform.md](platform.md)): with the bit CLEAR the
  oscillator really does run only when requested, so an enabled XOSCHF
  whose RUNSTDBY is 0 never reports stable while another source drives
  CLK_PER - selecting it IS the request, and the switch is what starts
  it. With the bit SET it costs nothing to wake from a standby sleep;
  with it clear the restart costs 1.45 ms on this board's 24 MHz
  crystal and 302 us on OSCHF - of which ~270 us is the voltage
  regulator, not the oscillator (`SLPCTRL.VREGCTRL.PMODE = FULL`
  removes it and leaves the data sheet's 24-30 us). That is what the
  task's hard-wired RUNSTDBY buys, and what it costs is standby
  current.
- `Uart::rebase` needs TWO frame times after DREIF before the clock
  may change (measured: one frame plus 1 us corrupts the last byte at
  several rates) - the contract is stated in [usart.md](usart.md).

**The PLL, measured through its only consumer** (`test_avr_tcd` test
`f`, rev A5): a TCD cycle of 3200 counter ticks, timed in CLK_PER
ticks against the 24 MHz crystal, gives OSCHF 16 MHz 4808 ticks, x2
2404, x3 1603, and OSCHF 24 MHz 3205, x2 1602. Because every leg runs
from the same oscillator the ratios ARE the multipliers, free of
OSCHF's own error: 2x measures 2.000 and 2.001, 3x measures 2.999, and
16 MHz x3 lands on the same 48 MHz as 24 MHz x2. PLLS sets within one
read of the TCD being enabled on the PLL. Errata 2.5.3 observed
directly: with RUNSTDBY = 1 and no requester, PLLS stays 0 through
65535 polls. Errata 2.5.4 observed as a refusal: with this board's
XOSCHF in crystal mode `Pll::source_ok(xoschf)` is false and
`Pll::start(xoschf, ...)` returns false with PLLCTRLA untouched.

**On the AVR128DA48** (REVID 0x17, rev A7, the board's 24 MHz external
clock into PA0, 5 V; `test_avr_clock` 25 verdicts with the two letters
only that board builds, CLKOUT never enabled - its PA7 carries 100 nF):

- The OSCHF sweep, the twelve prescalers of the external clock and the
  manual tune behave as on the DB, the console following at 9600; the
  clock failure detector is absent from the part and its letter says so.
- **XOSC32K** forced on comes up within 1.5 s (ten runs of ten).
- **`Clock<xosc32k>` as the main clock** (letter 9, ten runs): `init()`
  reports the crystal, MCLKCTRLA reads XOSC32K, and CLK_PER timed by a
  TCB across 32 periods of a TCD on EXTCLK (1048576 external-clock
  cycles each) reads 45815 ticks against 45813 nominal - **+22 to +43
  ppm**, the two crystals' difference (`test_avr_rtc` letter 9 measures
  it at 38..39 ppm) in steps of the 21 ppm a tick is there.
- **OSCHF runs at EXACTLY the external clock's rate while that clock is
  CLK_PER.** A TCD on OSCHF timed by a TCB at CLK_PER (32768 OSCHF
  cycles a period, eight periods): at TUNE 0, 0 ppm in nine runs of ten
  and -15 in the tenth, where TUNE -1 reads -3436..-3788 ppm and TUNE +1
  +3302..+3664 - a step either side, the oscillator runs free. With OSCHF
  itself as CLK_PER, timed against the 32.768 kHz crystal through the RTC,
  the same TUNE 0 reads -156..-277 ppm whether or not a TCD requests the
  external clock. So OSCHF is pulled onto the external clock when that
  clock is CLK_PER and OSCHF's free-running rate is within a step of it -
  on this die's calibration, at this bench's temperature. An OSCHF
  measured against the DA's external clock near 24 MHz measures the
  lock, not the oscillator (`test_avr_tcd` letter f: OSCHF 24 MHz
  reads 3200 ticks of 3200).
- **Auto-tune** (letter 8, ten runs of ten): TUNE -16 takes OSCHF to
  -49154..-49344 ppm of the crystal; AUTOTUNE then brings it to
  -126..-338 ppm (TUNE settles at 0), keeps it there with a TCD
  requesting the external clock (-126..-300), and OSCHF keeps that rate
  once AUTOTUNE is cleared (-129..-257). With no crystal running the
  auto-tune moves nothing (-48634 -> -48720): 11.5.5's "the crystal must
  be enabled".
- **Clearing AUTOTUNE copies the tune AFTER the write returns**: TUNE
  read in the next instructions still held the -16 written before
  AUTOTUNE was set, and 3 us later the auto-tune's 0. A `tune()` written
  in between was overwritten by the late copy - measured: TUNE 0
  written right after the clear left OSCHF at -4.9 %. `autotune(false)`
  now waits the copy out (120 CPU cycles, 5 us at 24 MHz; the copy
  crosses from the auto-tune's 1 MHz clock); a TUNE of 5 written at once
  after it is kept, eight of eight in each of ten runs.
- **The PLL from the external clock** (`test_avr_tcd` letter f, twelve
  runs): with SOURCE = 1 - the DA's EXTCLK, its only HF input - a TCD
  on EXTCLK counts 3200 CLK_PER ticks per 3200-tick cycle and the same
  TCD on the PLL x2 counts 1600: the multiplier against its own source,
  exact. Errata 2.4.1 (the DA's twin of the DB's 2.5.3) as on the DB:
  PLLS stays 0 with RUNSTDBY and no requester, sets at once with the TCD
  requesting.

The task's two restrictions are DELIBERATE positions, not gaps: it
accepts only source rates OSCHF can also produce, so the CFD
fallback keeps `Clock::hz` true (a 7.3728/11.0592/14.7456 MHz UART
crystal goes through the resources - `Xoschf::start_crystal(hz)`
takes any 4-24 MHz - at the price of a false fallback rate); and it
hard-wires RUNSTDBY on for OSCHF/OSC32K, trading standby power for
no start-up wait - an opt-out knob appears when a sleepy application
asks for it.

## The AVR EA (DS40002443A ch. 12)

The same controller in shape - CLKSEL, the twelve prescalers with the
SAME codes (12.5.2; the header names them `CLKCTRL_PDIV_DIV2_gc` where
the DA/DB's say `CLKCTRL_PDIV_2X_gc`, so `ClockDiv` is written as the
chapter's numbers and `test/family/clock.cpp` holds them against both
sets of names), OSC32K, XOSC32K, the clock failure detector, CCP on
every control register - with four differences that make it an engine
of its own:

- **OSCHF runs at ONE rate, chosen by a fuse.** `FUSE.OSCCFG.OSCHFFRQ`
  (8.8.2.3): 0 = 20 MHz, 1 = 16 MHz; OSCHFCTRLA has RUNSTDBY and
  AUTOTUNE and no frequency field (12.5.8). The CPU reads the fuse at
  0x1052 and only UPDI writes it. The `Clock` type names the rate as a
  CLAIM - an internal source's own rate, or the fourth argument
  `oschf_hz` beside a crystal or a clock (defaulting to the source's
  rate) - asserted to be 16 or 20 MHz; `init()` checks it against the
  fuse with one read (`matches_fuse()`) and on a mismatch writes
  NOTHING and returns false: the part keeps its reset clock.
- **The reset clock is OSCHF over /6** (12.3.3; MCLKCTRLB resets to
  0x11): 3.33 MHz from a 20 MHz fuse.
- **CLK_MAIN tops at 20 MHz, and only from 4.5 V** (35.10.5, table
  35-14: 5 MHz from 1.8 V, 10 MHz from 2.7 V, 20 MHz from 4.5 V,
  -40..85 C; 8 / 16 MHz to 125 C). brio asserts the part's 20 MHz
  (`clk_main_max_hz`, the device tables) and no supply: the rows are
  the reader's.
- **MCLKTIMEBASE** (12.3.6, 12.5.7; not under CCP): the count of CLK_PER
  cycles that is one microsecond or more, rounded UP - the time base the
  ADC's and the PGA's internal timings and the TWI's bus time-out count
  in (31.3.2, 31.3.3.4, 27.3.3.1). Too small (the reset 0 included) makes
  those delays short. It is a CLKCTRL register and a pure function of
  CLK_PER, so the CLOCK owns it: `ClockTimebase` is its resource, every
  `Clock::init()` writes `clock_timebase_for(hz)`, and every
  `DynamicClock` switch rewrites it - BEFORE the prescaler when the rate
  rises, AFTER it when the rate falls, so it never covers less than a
  microsecond. No driver writes it.

**The silicon's offer, item by item:**

| Item | Section | In brio |
|------|---------|---------|
| OSCHF at the fuse's 16 or 20 MHz | 8.8.2.3, 12.3.4.1.1 | used: `ClockSource::internal` with the rate as a claim checked by `matches_fuse()` |
| The main prescaler, /1../64 | 12.5.2 | used: `ClockDiv`, `MainClock::prescale`, the fallback's prescaler resolved at compile time (`div_for(oschf_hz, hz)`) |
| XOSCHF crystal 4-20 MHz on PA0/PA1, CSUTHF 256/1k/4k, RUNSTDBY | 12.5.12, 35.10.3 | used: `Xoschf::start_crystal` (4k cycles by default), `ClockSource::crystal`; no FRQRANGE field on this die (`range_for` is 0) |
| XOSCHF external clock on PA0, up to 20 MHz | 12.3.4.2.1, 35.10.4 | used: `Xoschf::start_external`, `ClockSource::external` (not on the bench board: a crystal sits on PA0) |
| XOSC32K crystal or clock, CSUT 1k..64k, LPMODE | 12.5.11 | used: `Xosc32k`, the DA/DB's resource unchanged (same register) |
| OSC32K | 12.5.10 | used: `Osc32k` |
| Auto-tune against XOSC32K | 12.3.7, 12.5.8 | used: `Oschf::autotune` - the field is two bits in the header (code XOSC32K = 1), one documented: written with the code, cleared whole (the device tables' `oschf_autotune_on/field`); the clear waits 120 cycles for the TUNE copy (0.75 us + 3 CLK_MAIN, 12.5.9) |
| Manual tune, -32..+31, 0.2 % a step typical | 12.5.9, 35.10.1 | used: `Oschf::tune`; bit 5 mirrored into 7:6 by the hardware |
| OSCHFCTRLA bits 3:2 (reset 0x0C on paper, undocumented) | 12.5.8 | declined: never written whole, every write a read-modify-write of the bit it means |
| MCLKTIMEBASE | 12.3.6, 12.5.7 | used: `ClockTimebase`, owned by the clock (above) |
| Clock failure detection: main / XOSCHF / XOSC32K, INT or NMI, test bit | 12.3.5 | used: `ClockFailure`, the DB's resource unchanged; the fallback is OSCHF at the fuse's rate ("at its reset frequency") |
| CLKOUT on PA7 | 12.5.1 | used: `MainClock::clkout`; never on the bench board, whose PA7 carries 100 nF |
| The supply grade | 35.10.5 | declined as a parameter (the reader's; above) |
| PLL, FRQSEL | - | absent on this die: `Pll` and `Oschf::set_hz` do not exist here (refused by name) |

**Types and verbs on the EA.** As above, less `Pll`, `oschf_frqsel()`
and `Oschf::set_hz` (absent), plus `ClockTimebase::write(cycles)` /
`read()`, `clock_timebase_for(hz)`, and on `Clock` the fourth argument
`oschf_hz` and `matches_fuse()`. `Clock::init()` sequence: the fuse
check; RUNSTDBY on OSCHF (read-modify-write); the fallback prescaler;
CLKSEL to OSCHF and its status waited; TIMEBASE; then, for a crystal or
a clock, XOSCHF started and EXTS waited (0xFFFF polls), and the switch -
where the fallback's prescaler differs from the source's, the one that
lowers CLK_PER in between goes first.

**How to use it on the EA:**

```cpp
// The bench board: a 20 MHz quartz, the part fused at 20 MHz (the factory OSCCFG 0x00).
using SysClock = brio::Clock<brio::ClockSource::crystal, 20'000'000>;
// The internal oscillator claimed at the fuse's 20 MHz, CLK_PER 5 MHz:
using Slow = brio::Clock<brio::ClockSource::internal, 20'000'000, brio::ClockDiv::div4>;
// An 8 MHz crystal on a part fused at 16 MHz: the fallback is OSCHF / 2.
using Xtal8 = brio::Clock<brio::ClockSource::crystal, 8'000'000, brio::ClockDiv::div1, 16'000'000>;
```

A `DynamicClock` over a 20 MHz source reaches the rates 20 MHz over a
divider gives in whole hertz - /1, /2, /4, /8, /10, /16, /32, /64 - and
refuses /6, /12, /24, /48 (3.33, 1.67, 0.83, 0.42 MHz are not whole);
`delay.hpp`'s per-rate dispatch still expands those four branches, dead
flash on this source (a cost note, not a fault).

**Cost.** `Clock::init()` on the EA: one fuse read, five protected
writes (RUNSTDBY, prescaler, CLKSEL, XOSCHFCTRLA, CLKSEL), one store of
TIMEBASE, and the status polls. A `DynamicClock::set<hz>()` adds ONE
store of a constant (TIMEBASE) to the prescaler's protected write; the
run-time `set(hz)` adds a lookup of the rate's index and a 32-bit
compare with the current rate to place that store - counted in
`test_avr_clock`'s listing, against the run-time `div_for`'s divisions
it already pays.

**Bench findings (AVR64EA48 rev B1, REVID 0x21, 5 V; `test_avr_clock`'s
EA letters driven and read through the debugger, the board's console
path being open and its 32 kHz crystal not running - its page - so
letters 2 to 6, which time on that crystal, stop at their first verdict
there, and letter 1 runs its ladder untimed).**

- Reset state, read at the reset vector over UPDI: MCLKCTRLA 0x00,
  **MCLKCTRLB 0x11** (DIV6, PEN, as 12.5.2 says), MCLKSTATUS 0x02
  (OSCHFS), **MCLKTIMEBASE 0x00**, **OSCHFCTRLA 0x00 - not the 0x0C of
  12.5.8**: bits 3:2 read 0 at reset on this silicon; NVMCTRL.CTRLB
  0x30 (FLMAP 3).
- `Clock<crystal, 20 MHz>::init()` (pin_check, the console): CLKSEL
  EXTCLK with EXTS set on the board's 20 MHz quartz, MCLKCTRLB 0,
  MCLKTIMEBASE 20, XOSCHFCTRLA 0xA1 (CSUTHF 4k, RUNSTDBY, ENABLE),
  OSCHFCTRLA 0x80 after the RUNSTDBY read-modify-write; the console's
  USART2 at BAUD 174 for 460800 (64 x 20 MHz / (16 x 460800) = 173.6).
- `Clock<internal, 20 MHz, div4>` (family_probe): CLKSEL OSCHF,
  MCLKCTRLB 0x03 (DIV4, PEN), MCLKTIMEBASE 5 - 5 MHz, PA7 toggling.
- Letter r (ten runs of ten, 9 verdicts each): the reset state as above,
  read by the program itself before its init - OSCHFCTRLA 0x00 at every
  reset - the fuse's 20 MHz matching the type's claim, TIMEBASE 20 and
  the prescaler off after the init, OSCHFCTRLA's bits 3:2 left as found.
- **TIMEBASE follows every rate** (letter 1, ten runs of ten): across the
  twelve prescalers from the 20 MHz crystal MCLKTIMEBASE reads
  ceil(CLK_PER / 1 MHz) after every switch - through `DynamicClock::set()`
  for the eight whole-hertz rates, which it reaches, and by hand for the
  four fractional ones, which it refuses - and 20 again at the end.
- Letter 7 (ten of ten, 6 verdicts): MCLKSTATUS with the crystal as the
  main clock has EXTS set and no switch pending; RUNSTDBY toggles through
  CCP without touching bits 3:2.
- **XOSCHF's start-up per CSUTHF** (letter 8, twelve runs of twelve):
  from the protected write to EXTS, with OSCHF (untuned) as the ruler,
  15 / 52 / 203..208 us for 256 / 1k / 4k cycles - the counted cycles at
  20 MHz (12.8 / 51.2 / 204.8 us) and a microsecond or two, the board's
  quartz up at once; EXTS reads 0 a millisecond after `Xoschf::stop()`
  in every run. The driver's `wait_stable(0xFFFF)` spins for 52.7 ms at
  20 MHz (1054080 cycles, 16 a poll) - 250 times the slowest start-up
  seen. The floor of that verdict is the counted cycles less OSCHF's 3 %
  (an exact floor failed by 2 us: the ruler, not the oscillator).

## Not covered yet

Driver gaps:

- The AVR EA's supply grade (5 / 10 / 20 MHz from 1.8 / 2.7 / 4.5 V,
  35.10.5): declined as a parameter - the type asserts the part's 20 MHz
  and the reader of the table above owns the supply. A 20 MHz `Clock` on
  a 3.3 V EA board compiles and runs out of specification.
- The AVR EA's `DynamicClock` booted below its source rate: `Boot` must
  be the source at /1, so an EA board under 4.5 V cannot boot a dynamic
  regime from a 20 MHz source at a legal rate (a static `Clock` at its
  prescaled rate can) - born with the first sleepy EA application.
- Errata 2.5.1 (A4 silicon): `Clock<external>::init` waits for EXTS
  before anything requests the source, which on A4 never sets - the
  init as written always falls back to OSCHF there (documented
  above, not worked around; B0 and A5 unaffected).

Implemented but not bench-verified:

- On the AVR EA, the letters of `test_avr_clock` that time on the 32 kHz
  crystal (the bench board's does not run - its page): the prescaler
  ladder's rates, OSCHF's accuracy at the fuse's
  rate, the tune curve, the auto-tune and the late TUNE copy, the 32 kHz
  main clocks, the clock failure detector on XOSC32K and on the main
  clock (what the prescaler does at the fallback) - each measures on a
  board whose 32 kHz crystal runs; `ClockSource::external` on the EA (the board puts a
  crystal on PA0); `Clock<internal, 16 MHz>` (the bench part is fused
  at 20 MHz).
- `ClockSource::external` on the DB and the PLL from its XOSCHF (no
  external clock on the DB board, and errata 2.5.4 makes the PLL need
  one); `run_standby` on the PLL beyond the 2.5.3 / 2.4.1 observation;
  A4 silicon (the DB board is A5); the auto-tune's accuracy under a
  temperature change (one bench temperature); the clock failure detector
  watching XOSC32K (the crystal now runs, on the DA board, which has no
  detector - the DB board, which has one, fits no crystal).
