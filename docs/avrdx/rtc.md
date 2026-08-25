# RTC / PIT - the real-time counter and periodic interrupt timer (AVR DA/DB)

> **PROVISIONAL.** The chapter's register description is covered in
> full and bench-verified on the internal oscillator, standby and
> power-down included; what remains is what this bench cannot reach -
> the 32.768 kHz crystal and the external clock (neither is fitted on
> the board), the debug-run paths (no halted CPU in a suite) and the
> tasks that would drive an application through a sleep. The list is
> in "Not covered yet".

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B (RTC
chapter 26), errata DS80000915F and, for the DA parts, DS80000882C -
neither lists an RTC item; the one that touches this peripheral is
CLKCTRL 2.5.2 (DB rev. A4), where `RUNSTDBY` in `XOSC32KCTRLA` fails
to hold an external 32 kHz source alive through sleep, so a
standby-running RTC on the crystal needs another user of that clock.
Drivers: `avrdx/rtc.hpp` (`RtcClock`, `Rtc`, `Pit`) and
`avrdx/ticker.hpp` (`BasicTicker`, `Ticker`). Reference test:
`test_avr_rtc`; the PIT dividers as event generators are also
exercised by `test_avr_timer`, and both halves in sleep by
`test_avr_sleep`.

## What the silicon does

One peripheral, two timing functions over one clock and one prescaler
chain: a 16-bit **real-time counter** (CNT against PER for the
overflow, against CMP for the compare, both an interrupt and an event)
and a **periodic interrupt timer** that raises an interrupt every
PERIOD cycles of CLK_RTC. Either can run without the other; both are
asynchronous to CLK_PER, which is why every configuration write costs
a synchronization.

Facts that matter to code:

- **One clock select for both.** `CLKSEL` picks the internal OSC32K
  (32.768 kHz), the same oscillator divided by 32 (OSC1K, 1.024 kHz),
  the 32.768 kHz crystal / external clock on XTAL32K1-2, or an
  external clock on the EXTCLK pin. There is no per-function choice:
  whoever owns the timebase owns CLKSEL (26.4.1.1).
- **The counter's prescaler is the counter's alone.** `CTRLA.PRESCALER`
  divides CLK_RTC by 1..32768 before CNT; the PIT's PERIOD counts
  CLK_RTC cycles and does not see it (measured: PIT_DIV64 unchanged
  when the prescaler goes from DIV1 to DIV32).
- **The first tick is unknowable.** The prescaler's internal counter is
  stopped only while BOTH functions are disabled, and it starts when
  either is enabled, so the first PIT interrupt and the first RTC count
  fall anywhere inside one full period (26.5.2.2). A period boundary is
  the only instant an application may build on - never the enable.
- **Compare lands at CMP + 1.** The overflow resets CNT to zero and the
  compare fires at the first count after CNT equals CMP, so the compare
  event is exactly CMP + 1 counter ticks after the overflow (measured
  exact).
- **Every write crosses a clock domain.** CTRLA, CNT, PER and CMP each
  have a busy flag in STATUS, PITCTRLA one in PITSTATUS, and the flag
  must be clear before the next write to that register (26.10). The
  driver waits for the flag of the register it is about to write; the
  wait is bounded, so a CLK_RTC that no peripheral has requested cannot
  hang the caller.
- **CLK_PER must be at least 4x CLK_RTC** to read CNT (26.3) - trivially
  true at any sane main clock, worth remembering when the dynamic clock
  regime is about to drop the CPU to 32 kHz.
- **The 16-bit registers share one TEMP register.** Reading or writing
  CNT/PER/CMP from the main context while an ISR touches them too wants
  a critical section, exactly as for the TCB.
- **Crystal error correction is a trim, not a cure.** CALIB carries a
  sign and seven bits of ppm, so the whole range is +-127 ppm, applied
  by adding or removing whole CLK_RTC cycles spread over a million-cycle
  interval (30.5 s at 32.768 kHz). A NEGATIVE correction requires the
  prescaler at DIV2 or slower (26.6) - the driver refuses the illegal
  pair instead of programming a wrong trim, at compile time in the
  config form and with a `false` at run time.
- **Sleep**: the counter runs in idle, and in standby only with
  RUNSTDBY; the PIT runs in every sleep mode, power-down included
  (26.9). Both halves are bench-proven there, and so is the asymmetry:
  in POWER-DOWN the counter stops even with RUNSTDBY set while its PIT
  goes on interrupting. One trap comes with it - CNT read at the
  INSTANT of a wake still carries the value it had at the SLEEP
  instruction, because the read is synchronized into CLK_PER (26.10)
  and that path needs the clock back plus a CLK_RTC edge; a millisecond
  later it is right. Under a halted CPU each half keeps running only
  with its own DBGRUN; with the PIT's clear, a break taken while its
  output is high costs one extra interrupt on resume (26.11).
- **Events** (26.7): the counter's OVF and CMP are pulses on the
  conditions that raise the flags; the PIT's divided clocks
  (PIT_DIV64..DIV8192) are free-running levels off the same chain. Both
  families live in [evsys.md](evsys.md) as `EvRtcOvf` / `EvRtcCmp` /
  `EvPitDiv<n>`.
- The peripheral is identical on every DA/DB package: one instance, the
  same registers and the same four clock sources everywhere - nothing
  to gate.

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `RtcSource` | the four CLK_RTC sources: `osc32k`, `osc1k`, `xosc32k`, `extclk`; `rtc_source_hz()` gives the nominal rate (0 for `extclk`, which only the application knows) |
| `RtcClock` | the shared clock select: `select()`, `selected()`, `hz()`, `preferred()` (the crystal if the clock init started one, the internal oscillator otherwise) |
| `RtcPrescaler`, `rtc_prescaler_div()` | DIV1..DIV32768 and the divisor each stands for |
| `RtcConfig` | the counter's whole configuration: prescaler, period (PER), compare (CMP), correction in ppm, run-standby, debug-run |
| `rtc_correction_valid()` | the legality rule of 26.6, shared by the config and `calibrate()` |
| `Rtc` | the counter: `init<cfg>()` / `init(cfg)`, `enable`/`disable`/`enabled`, `count`, `period`, `compare` (each a getter and a busy-waiting setter), `prescaler`, `tick_hz`, `calibrate`/`calibration_ppm`/`correcting`, `run_standby`, `debug_run`, the flags `ovf_flag`/`cmp_flag`/`clear_ovf`/`clear_cmp`, the enables `enable_ovf_interrupt`/`enable_cmp_interrupt`, the ISR body `take_flags` (OVF and CMP share `RTC_CNT_vect`), the synchronization state `ctrla_busy`/`count_busy`/`period_busy`/`compare_busy`/`sync`, and the event types `OvfEvent`/`CmpEvent` |
| `PitPeriod`, `pit_cycles()` | OFF, CYC4..CYC32768 and the CLK_RTC cycles each stands for |
| `Pit` | the periodic timer: `init(period, interrupt)`, `period`, `enable`/`disable`/`enabled`, `tick_hz`, `enable_interrupt`/`interrupt_enabled`, `flag`/`clear_flag`, the ISR body `take_flag` (`RTC_PIT_vect`), `ctrl_busy`, `debug_run` |
| `BasicTicker<tps>`, `Ticker` | the kernel's timebase over the PIT: `init()` (or `init(source)`), `pit()` (ISR body), `ticks()`, `millis()`, `secs()`, `now(TimeStamp&)`, `pause()`/`resume()`; `ticks_per_second` |
| kernel side | `AvrPlatform::now()` = ticks; `ticks_from_ms<P>()`, `TimeEvent` ([kernel.md](../design/kernel.md)) |

`Rtc` and `Pit` are resources - registers with names. Tasks over them
(an alarm at a wall-clock instant, a slow periodic below the Ticker's
floor) are not built: they are born with their first user.

## How to use it

**As the kernel's timebase** - the common case, and the one that owns
CLKSEL:

```cpp
ISR(RTC_PIT_vect) { brio::Ticker::pit(); }
...
brio::Ticker::init();        // after clock init, before sei()
brio::TimeStamp ts; brio::Ticker::now(ts); brio::print(serial, ts);   // "12.045s"
```
Waiting in an AO is a `TimeEvent`, never a busy loop. `init(source)`
names the clock instead of letting the Ticker guess (`RtcSource::osc1k`
divides every tick rate by 32).

**As a counter with a periodic overflow** - a second, a minute, a
sampling epoch:

```cpp
ISR(RTC_CNT_vect) { if (brio::Rtc::take_flags().ovf) ++seconds; }
...
brio::RtcClock::select(brio::RtcSource::osc32k);      // if no Ticker owns it
brio::Rtc::init<brio::RtcConfig{.prescaler = brio::RtcPrescaler::div1,
                                .period = 32767}>();  // one second
brio::Rtc::enable_ovf_interrupt(true);
```

**As an alarm inside the period** - CMP raises its own flag and its own
event, CMP + 1 ticks after the overflow:

```cpp
brio::Rtc::compare(8192);                             // a quarter of the way in
brio::Rtc::enable_cmp_interrupt(true);
brio::EventChannel<5>::source(brio::Rtc::CmpEvent{}); // ... or with no CPU at all
```

**Trimming the clock**: `calibrate()` takes the error in ppm, positive
to slow the prescaler down (the source runs fast), and enforces the
DIV2 rule.

```cpp
if (!brio::Rtc::calibrate(-100)) { /* prescaler is DIV1: a negative trim is illegal */ }
```

**As a free event source with no interrupt**: the PIT's divided clocks
are levels a channel can carry to any user - a hardware pace for the
ADC, a LED that blinks with the CPU asleep.

```cpp
brio::EventChannel<1>::source(brio::EvPitDiv<64>{});  // 512 Hz from a 32.768 kHz CLK_RTC
```

## Bench findings

`test_avr_rtc` (AVR128DB48 rev. A5, 24 MHz crystal, CLK_RTC = OSC32K;
78 verdicts). The stopwatch is a TCB cascade at CLK_PER whose snapshot
channel is sourced from the RTC's own OVF and CMP events, so every
interval below is latched by hardware, not by an ISR.

- **The OSC32K on this part runs about +0.9 % fast** and wanders: a
  counter second measures 23.76..23.79 M crystal ticks against a
  nominal 24 M (+9000..+9800 ppm), and consecutive one-second
  measurements differ by 100..300 ppm. That wander is the noise floor
  of every ppm-level measurement here.
- **The prescaler divides exactly**: DIV1/PER=32767, DIV2/PER=16383 and
  DIV32/PER=1023 - all 32768 CLK_RTC cycles - give the same period
  within the oscillator's own wander.
- **The compare is exact**: with PER = 32767, CMP = 99 measured 100 RTC
  ticks after the overflow and CMP = 8191 measured 8192 (+-1 tick, the
  precision of converting crystal ticks back with a wandering source).
- **Crystal error correction works, granularly.** +-127 ppm requested
  measured +105..+148 ppm and -110..-150 ppm, the two signs spanning
  215..293 ppm where 254 is asked for. The granularity is the reason
  for the scatter: the correction inserts or removes ONE WHOLE CLK_RTC
  cycle (732 crystal ticks) every 1e6 / ERROR cycles - one every 240 ms
  at 127 ppm - so a quarter-second period is one or two corrections
  long, never 1.04 of one, and only the mean over many periods
  reproduces the trim. The trim is measurable at all only by
  alternating trimmed and untrimmed periods and averaging; blocks
  compared tens of seconds apart measure the oscillator's drift
  instead.
- **The busy flags cost about 2.8 CLK_RTC periods**: from the write to
  the flag clearing, CNTBUSY 2095..2107, PERBUSY 2005..2065, CMPBUSY
  2005..2071, CTRLABUSY 2055 and the PIT's CTRLBUSY 2001..2061 crystal
  ticks, where one CLK_RTC period is 732 nominal ticks. The data
  sheet's "two RTC clock cycles" is the latency of the value, not the
  life of the flag.
- **The first tick is unknowable, and the two cases differ.** Timing
  32 enable cycles of a CYC32 period (23424 crystal ticks): with the
  prescaler stopped (both functions disabled first) the first interrupt
  came at 14484..16525 ticks, mean 16434 - repeatable but not the full
  period; with the prescaler free-running (the counter left enabled) it
  came anywhere from 4845 to 15902, mean 7735. Every one inside a
  period, none of them predictable.
- **The two functions do not disturb each other**: the counter's period
  is the same with the PIT and its 1024 Hz ISR running as without it
  (within the oscillator's wander), and the ratio is exact - 1024 PIT
  ticks per 32768-cycle counter period, every time.
- **The PIT does not see the counter's prescaler**: PIT_DIV64 measured
  46428..46448 crystal ticks with PRESCALER = DIV1 and 46433..46457
  with DIV32 - the same 64 CLK_RTC cycles.
- **OSC1K is the same oscillator divided by 32**: a second built from
  1023 counts of a 1.024 kHz CLK_RTC measures the same crystal ticks as
  one built from 32767 counts of the 32.768 kHz one, and a Ticker asked
  for 1024 ticks/s on OSC1K delivers exactly 32.

## Not covered yet

Driver gaps: none. Every register of chapter 26 - CTRLA, STATUS,
INTCTRL, INTFLAGS, DBGCTRL, CALIB, CLKSEL, CNT, PER, CMP, PITCTRLA,
PITSTATUS, PITINTCTRL, PITINTFLAGS, PITDBGCTRL - is exposed by a verb,
and the four clock sources, sixteen prescaler settings and fifteen PIT
periods are the enums' full range. TEMP is the compiler's, not the
driver's.

Implemented but not bench-verified:

- `RtcSource::xosc32k` and `RtcSource::extclk`: the bench board fits no
  32.768 kHz crystal and no external 32 kHz clock (the EXTCLK pin
  carries the 24 MHz crystal), so both codes are driver surface and
  family-compile coverage only. `Ticker::preferred()` picks the crystal
  when CLKCTRL reports one running - that branch has never fired here.
- The counter's `RUNSTDBY` and the PIT as a wake-up source are proven
  in standby and in power-down by `test_avr_sleep` (findings in
  [platform.md](platform.md)); what no suite has staged is the RTC
  driving an application THROUGH a sleep - an alarm or a slow periodic
  that survives standby is a task this driver deliberately does not
  have yet.
- `DBGCTRL` / `PITDBGCTRL`: the bits read back as written; the extra
  PIT interrupt on resuming from a break with DBGRUN clear (26.11) is
  a debugger-in-the-loop experiment, not a suite verdict.
