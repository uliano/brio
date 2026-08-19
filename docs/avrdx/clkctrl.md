# CLKCTRL - the clock controller (AVR DA/DB)

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B
(CLKCTRL chapter, electricals 39.10 and 39.3), errata DS80000915F
(2.5.1-2.5.4). Driver: `avrdx/clock.hpp` (resources `Oschf`, `Osc32k`,
`Xosc32k`, `Xoschf`, `Pll`, `MainClock`, `ClockFailure`; tasks `Clock`,
`DynamicClock`), `avrdx/delay.hpp`; the target-independent contracts in
`util/clock.hpp`. The clock MODEL (one rate truth, static and dynamic
regimes, the rebase fan-out) is [clock.md](../design/clock.md); this page is the
peripheral. Reference test: `test_avr_clock` (CLKOUT on PA7 to an
oscilloscope), `clock_console` (the dynamic regime under a console).

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
| PLL | x2 or x3 from OSCHF or XOSCHF | in 16-24 MHz, out 32-48 MHz | lock ~10 us; clocks the TCD ONLY - it is not a main-clock source; runs only while the TCD requests it |

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
| `Oschf` | `set_hz(hz)` (one of the nine rates; immediate, also while it is the main clock), `run_standby(bool)`, `autotune(bool)` (needs a running XOSC32K), `tune(-32..31)` / `tune()`, `stable()` |
| `Osc32k` | `run_standby(bool)`, `stable()` |
| `Xosc32k` | `start_crystal(Xosc32kStartup, low_power, run_standby)`, `start_external(run_standby)`, `stop()`, `enabled()`, `stable()`, `wait_stable(spins)` |
| `Xoschf` (DB) | `start_crystal(hz, XoschfStartup, run_standby)`, `start_external(hz, run_standby)`, `stop()`, `enabled()`, `stable()`, `wait_stable(spins)`, `range_for(hz)` |
| `Pll` | `start(PllSource::oschf/xoschf, PllMultiplier::x2/x3, run_standby)`, `stop()`, `locked()` |
| `MainClock` | `select(MainSource::oschf/osc32k/xosc32k/extclk)` -> bool (switch completed), `source()`, `switching()`, `prescale(ClockDiv)`, `clkout(bool)` (PA7) |
| `ClockFailure` | `watch(CfdSource::main/xoschf/xosc32k)`, `stop()`, `interrupt(on, nmi)`, `test(force)`, `failed()`, `clear()`, ISR body `cfd()` -> the main source now in effect |

Tasks - what an application names ([clock.md](../design/clock.md)):

| Task | Verbs |
|------|-------|
| `Clock<ClockSource, source_hz, ClockDiv>` | `init()` -> bool (running from the requested source), `hz`, `is_static = true`; sources `internal`, `crystal`, `external`, `osc32k`, `xosc32k` |
| `DynamicClock<Boot, Users...>` | `init()`, `hz()`, `set<hz>()` / `set(hz)` -> bool, `can_run_at(hz)`, `rebases<U>`; `is_static = false` |
| vocabulary | `ClockDiv`, `clock_divisor()`, `div_for()`, `oschf_frqsel()`, `MainSource`, `CfdSource`, `PllSource`, `PllMultiplier`, `Xosc32kStartup`, `XoschfStartup` |
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

**A 32 kHz crystal** for the RTC and for OSCHF auto-tune (not fitted
on the bench board):

```cpp
Xosc32k::start_crystal();            // early: ~300 ms to stabilise
...
if (Xosc32k::stable()) Oschf::autotune(true);   // OSCHF trimmed against it
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

**The PLL** (for the TCD, when that driver exists): `Pll::start(
PllSource::oschf, PllMultiplier::x2)` with OSCHF at 16-24 MHz; it runs
when the TCD requests it.

## Bench findings (`test_avr_clock`, rev A5, CLKOUT on PA7, 14/14)

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
- XOSC32K correctly never reports stable on this board (no 32 kHz
  crystal).
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
- `Uart::rebase` needs TWO frame times after DREIF before the clock may
  change: one frame plus 1 us corrupted the last byte (the line feed)
  at several rates.
