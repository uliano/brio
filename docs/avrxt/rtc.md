# RTC / PIT - the real-time counter and periodic interrupt timer (AVR DA, DB and EA)

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B (RTC
chapter 26), errata DS80000915F and, for the DA parts, DS80000882C -
neither lists an RTC item; for the AVR EA, AVR64EA28/32/48 data sheet
DS40002443A (RTC chapter 24, the same chapter but for one register,
PITEVGENCTRLA, 24.13.17) and errata DS80001048D (no RTC item); the one that touches this peripheral is
CLKCTRL 2.5.2 (DB rev. A4), where `RUNSTDBY` in `XOSC32KCTRLA` fails
to hold an external 32 kHz source alive through sleep, so a
standby-running RTC on the crystal needs another user of that clock.
Drivers: `avrxt/rtc.hpp` (`RtcClock`, `Rtc`, `Pit`) and
`avrxt/ticker.hpp` (`BasicTicker`, `Ticker`). Reference test:
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
  `EvPitDiv<n>`. On the AVR EA the PIT's dividers reach the event system
  through two SLOTS (PITEVGENCTRLA, 24.13.17): each carries OFF or
  CLK_RTC / 4 .. / 32768, chosen by the event route
  (`EvPitDiv<div, slot>`) or from the RTC's side
  (`Pit::event_divider<slot>`); a slot carries its divider only while
  PITEN is set (measured below).
- The peripheral is identical on every DA/DB package: one instance, the
  same registers and the same four clock sources everywhere - nothing
  to gate.

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `RtcSource` | the four CLK_RTC sources: `osc32k`, `osc1k`, `xosc32k`, `extclk`; `rtc_source_hz()` gives the nominal rate (0 for `extclk`, which only the application knows) |
| `RtcClock` | the shared clock select: `select()`, `selected()`, `hz()`, `preferred()` (the crystal if one was started and is still enabled and running, the internal oscillator otherwise) |
| `RtcPrescaler`, `rtc_prescaler_div()` | DIV1..DIV32768 and the divisor each stands for |
| `RtcConfig` | the counter's whole configuration: prescaler, period (PER), compare (CMP), correction in ppm, run-standby, debug-run |
| `rtc_correction_valid()` | the legality rule of 26.6, shared by the config and `calibrate()` |
| `Rtc` | the counter: `init<cfg>()` / `init(cfg)`, `enable`/`disable`/`enabled`, `count`, `period`, `compare` (each a getter and a busy-waiting setter), `prescaler`, `tick_hz`, `calibrate`/`calibration_ppm`/`correcting`, `run_standby`, `debug_run`, the flags `ovf_flag`/`cmp_flag`/`clear_ovf`/`clear_cmp`, the enables `enable_ovf_interrupt`/`enable_cmp_interrupt`, the ISR body `take_flags` (OVF and CMP share `RTC_CNT_vect`), the synchronization state `ctrla_busy`/`count_busy`/`period_busy`/`compare_busy`/`sync`, and the event types `OvfEvent`/`CmpEvent` |
| `PitPeriod`, `pit_cycles()` | OFF, CYC4..CYC32768 and the CLK_RTC cycles each stands for |
| `Pit` | the periodic timer: `init(period, interrupt)`, `period`, `enable`/`disable`/`enabled`, `tick_hz`, `enable_interrupt`/`interrupt_enabled`, `flag`/`clear_flag`, the ISR body `take_flag` (`RTC_PIT_vect`), `ctrl_busy`, `debug_run`; on the AVR EA `event_divider<slot>(PitEventDiv)` (a read-modify-write of the slot's nibble) and `event_divider<slot>()` (read back) |
| `PitEventDiv`, `pit_event_cycles()` | the AVR EA's slot values: `off`, `div4`..`div32768` (the header's codes) and the CLK_RTC cycles each stands for |
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

On the AVR EA a slot can also be re-tuned from the RTC's side under a
route made once - the channel names the slot, not the divider - and
it carries nothing unless the PIT is enabled:

```cpp
brio::EventChannel<3>::source(brio::EvPitDiv<4, 1>{});   // slot 1, CLK_RTC / 4
brio::Pit::init(brio::PitPeriod::cyc1024, false);         // PITEN: the slots run
brio::Pit::event_divider<1>(brio::PitEventDiv::div2048); // the same channel at 16 Hz
brio::Pit::event_divider<1>(brio::PitEventDiv::off);     // and still
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

On the **AVR128DA48** (REVID 0x17, rev A7, the board's 24 MHz external
clock as CLK_PER, 5 V; 88 verdicts with letter 9, which only the board
with a 32.768 kHz crystal builds):

- Letters 1 to 8 run as on the DB. This die's OSC32K runs about
  **+0.15 % fast** (a counter second 23.96 M external-clock ticks,
  -1453..-1580 ppm of interval, eight seconds 1487 ppm fast); the busy
  flags clear in 2023..2172 ticks, the same 2.8 CLK_RTC periods; the
  trim measured +123 and -136 ppm for +-127 asked.
- **XOSC32K as CLK_RTC** (letter 9, twelve runs of twelve): forced on,
  stable within 1.5 s and named by `preferred()`; the counter's four
  seconds measure 95996274..95996275 ticks of the external clock - the
  crystal **38..39 ppm fast** against it - forced on and again with the
  force dropped and the RTC as its only requester; `Ticker::init()`
  with no argument takes the crystal and counts exactly 1024 ticks to
  its second.
- **XOSC32KS outlives the oscillator by up to a millisecond**: stopped,
  the crystal's status bit still read 1 at the next poll and was down
  within 1 ms. A timebase started in that window on the status alone
  lands on the stopped crystal and the PIT never ticks again
  (`test_avr_clock` letter 8 froze its holds that way), so
  `preferred()` asks `Xosc32k::enabled()` as well: letter 9 asks it
  right after the stop and gets OSC32K, and letter 8 re-initialises its
  Ticker with no source named straight after stopping the crystal.

On the **AVR64EA48** (REVID 0x21, rev B1, 5 V, the board's 20 MHz
quartz as CLK_PER; `test_avr_rtc`, 91 verdicts, letters 1-8 and p ten
runs of ten; letter 9 owes the board's 32.768 kHz crystal, which does
not oscillate - [../boards/avr64ea48.md](../boards/avr64ea48.md)):

- **This die's OSC32K runs about 1.3 % slow**: a counter second is
  20.25..20.27 M quartz ticks against a nominal 20 M (+12600..+13300
  ppm of interval), inside the data sheet's +-10 % (DS40002443A 35.10.1;
  under 1 % only at 25 C and 3.0 V). It **wanders more than the DB's**:
  over twenty-four runs one second against the mean of the next eight
  differs by up to 391 ppm, two seconds a moment apart through
  different prescalers by up to 663, and a second from OSC1K against one
  from OSC32K by up to 345, where the AVR128DB48's stayed within 300 -
  the three verdicts' bound is the die's own (800 ppm on the EA, 500
  and 333 on the DA/DB). Its cycle-to-cycle jitter shows in a single
  period of four cycles (one DIV4 slot period read 0.24 % short once in
  ten runs; sixteen consecutive ones spread over 5..6 ticks of 2473),
  hence letter p's mean of sixteen.
- The prescaler divides exactly, the compare lands at CMP + 1 (100 for
  CMP = 99 every run, 8190..8193 for 8191 - the conversion's wander),
  OSC1K is the same oscillator divided by 32, the PIT does not see the
  counter's PRESCALER (PIT_DIV64 39552..39596 ticks at DIV1, 39559..
  39593 at DIV32) and 1024 PIT ticks fall in every 32768-cycle counter
  period - all as on the DB.
- The trim works as on the DB, granularly: +127 ppm measured +66..+189,
  -127 measured -67..-166, the signs spanning 133..355 ppm.
- **The busy flags live 2.8 CLK_RTC periods here too** (DB/DA 2.8): CNTBUSY
  1722..1824, PERBUSY 1698..1752, CMPBUSY 1698..1806, CTRLABUSY
  1688..1796 and the PIT's CTRLBUSY 1688..1742 quartz ticks, one CLK_RTC
  period being 610 nominal.
- The first PIT interrupt after an enable, CYC32 (19520 ticks): with
  the prescaler stopped 12283..14105 ticks, mean 13990; free-running
  4103..13440, mean 6567 - the DB's shape, at this clock.
- **The PIT's event slots** (letter p): every divider DIV4..DIV32768 on
  each slot, both slots at once, and the slots following PITEN rather
  than the prescaler - [evsys.md](evsys.md)'s findings.

## Not covered yet

Every register of chapter 26 - CTRLA, STATUS, INTCTRL, INTFLAGS,
DBGCTRL, CALIB, CLKSEL, CNT, PER, CMP, PITCTRLA, PITSTATUS, PITINTCTRL,
PITINTFLAGS, PITDBGCTRL - is exposed by a verb, and the four clock
sources, sixteen prescaler settings and fifteen PIT periods are the
enums' full range. TEMP is the compiler's, not the driver's. On the
AVR EA (DS40002443A ch. 24, the same chapter) the one register more,
PITEVGENCTRLA, is `Pit::event_divider<slot>`'s and the event routes'
(`EvPitDiv<div, slot>`, [evsys.md](evsys.md)).

Implemented but not bench-verified:

- XOSC32K as CLK_RTC on the AVR EA (letter 9): the bench board's 32.768
  kHz crystal does not oscillate (its page) - the letter runs and fails
  at its first verdict there, and the Ticker runs from OSC32K; a board
  whose crystal starts runs it as it stands.

- `RtcSource::extclk` and an external 32 kHz CLOCK on XTAL32K1: neither
  board feeds a clock to XTAL32K1, and the EXTCLK pin carries 24 MHz on
  the DA board (the DB's carries its crystal) - six times past 26.3's
  CLK_PER >= 4 x CLK_RTC for reading CNT - so both are driver surface
  and family-compile coverage only.
- The counter's `RUNSTDBY` and the PIT as a wake-up source are proven
  in standby and in power-down by `test_avr_sleep` (findings in
  [platform.md](platform.md)); what no suite has staged is the RTC
  driving an application THROUGH a sleep - an alarm or a slow periodic
  that survives standby is a task this driver deliberately does not
  have yet.
- `DBGCTRL` / `PITDBGCTRL`: the bits read back as written; the extra
  PIT interrupt on resuming from a break with DBGRUN clear (26.11) is
  a debugger-in-the-loop experiment, not a suite verdict.
