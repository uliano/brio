# Platform - what the kernel stands on (AVR DA/DB)

> **PROVISIONAL.** The four blocks this page covers - SLPCTRL, its
> voltage regulator, RSTCTRL and the WDT - are described in full and
> every claim below is measured, wake-up latencies included. What is
> missing is not mechanism but POLICY and the measurements this desk
> cannot make: the power-manager active object that would decide when
> an application may stop its clocks, the BOD's voltage-level monitor
> and MVIO as wake-up sources, and the sleep CURRENT, which needs a
> bench supply and not a stopwatch. The list is in "Not covered yet".

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B -
SLPCTRL chapter 13, RSTCTRL chapter 14, WDT chapter 22, the CPU's
Configuration Change Protection in 7.4.6 - plus errata DS80000915F
(item 2.2.4 is code here; clarifications 3.3.1 and 3.4.1 rewrite two
of those chapters' tables) and, for the DA parts, DS80000882C, which
lists no SLPCTRL, WDT or CPU item at all. Drivers:
`avrdx/platform_avr.hpp` (`AvrPlatform`, this target's realization of
the kernel's `Platform` concept), `avrdx/delay.hpp` (the short-wait
role), `avrdx/sleep.hpp` (`Sleep`, `Vreg`) and `avrdx/reset.hpp`
(`Reset`, `Watchdog`). Reference tests: `test_avr_platform` and
`test_avr_sleep`.

These are not peripheral drivers in the tasks-over-resources sense:
they are the services the kernel names in `kernel/platform.hpp` and
`kernel/panic.hpp`, and the two silicon blocks that make the panic
breadcrumb mean something. They live in one document because they are
one story - how a program on this silicon waits, how it stops, and how
the next boot finds out why.

## What the silicon does

**Sleep (SLPCTRL, 13).** One register arms it: `CTRLA.SMODE` picks
IDLE / STANDBY / PDOWN and `CTRLA.SEN` enables the SLEEP instruction.
Arming alone does nothing - "the SLEEP instruction must be executed to
make the device go to sleep" (13.3.1), and the wake-up sources must be
configured and enabled, with global interrupts on, BEFORE it runs: with
no enabled interrupt that reaches the armed mode, only a reset comes
back. `CTRLA` is **not** under CCP - only `VREGCTRL` is (13.3.5). The
register file and SRAM are kept through sleep (13.2), and a debugger
break wakes the device whether or not an interrupt is pending (13.3.4)
- which is why a sleep observed under a debug session is not the sleep
the silicon does on its own.

The three modes differ in what keeps its clock and in what may wake it.
**Tables 13-2, 13-3 and 13-4 are rewritten by errata DS80000915F
clarification 3.4.1**; what follows is the clarified version.

| | IDLE | STANDBY | POWER-DOWN |
|---|---|---|---|
| Runs | everything but the CPU | WDT, BOD, EVSYS, NVM always; RTC, CCL, AC, ADC, DAC, OPAMP, TCA, TCB only with their own `RUNSTDBY`; never the TCD's clock | PIT, WDT, sampled BOD, EVSYS - and nothing else, the RTC's COUNTER included |
| Main clock | runs | only if something requests it | stopped |
| Wakes on | any interrupt | PORT pin (async configuration), BOD VLM, MVIO, RTC (counter functions with `RUNSTDBY`), TWI address match, CCL (`RUNSTDBY`), USART start-of-frame, TCA/TCB/ADC/AC left running | PORT pin, BOD VLM, MVIO, PIT, TWI address match, CCL only when fully asynchronous (`FILTSEL` = 0, `EDGEDET` = 0) |
| Wake time | 6 CLK_PER cycles | + oscillator and regulator start-up | + the longer power-down start-ups |

`VREGCTRL.HTLLEN` (high-temperature low leakage, Power-Down only) cuts
that last column further, to the PORT pin, BOD VLM, MVIO and the PIT:
chapter 13 requires the TWI address-match and CCL wake-ups to be
disabled while it is set, "to avoid unpredictable behavior".
`VREGCTRL.PMODE` chooses the regulator's profile - AUTO ("Normal": the
regulator drops to low power in standby and power-down, and whenever
OSC32K is the only clock running) or FULL ("Performance": full drive in
every mode, so no regulator start-up on the way out).

**RUNSTDBY: the peripheral's flag is the one that decides.** Two
different bits carry that name - a peripheral's, and each oscillator's -
and the bench says they do different jobs. A peripheral's `RUNSTDBY` is
a REQUEST: it keeps that peripheral clocked in standby and, through the
request, keeps the main clock and the oscillator behind it running,
whatever the oscillator's own flag says. An oscillator's `RUNSTDBY`
keeps that oscillator running when NOTHING requests it - which never
makes a stopped peripheral count, and buys exactly one thing: no
start-up time on the way out of the sleep (measured below, and it is
worth 1.5 ms on this board's crystal). The corollary for a driver
writer: a peripheral that must survive standby needs its OWN
`RUNSTDBY`; an oscillator with the bit clear is not even startable
while something else drives CLK_PER, because it only runs when
requested - selecting it as the main clock IS the request.

The one erratum that reaches into ordinary code lives here.
**DS80000915F 2.2.4**, all silicon revisions: a store to an address
>= 64 immediately followed by a write to `SLPCTRL.CTRLA` loses that
write, and the documented workaround is a NOP before the CTRLA write.
The same item has a second half - a store to an address >= 64 followed
by an `ST`/`STD` to an address **< 64** (the low I/O space: VPORT,
GPIOR) loses that write too, where the workaround is a NOP or using
`OUT` instead of `ST`. Nothing in the stratum guards that half, and
nothing needs to yet: for a compile-time address in the low I/O space
GCC emits `out`/`sbi`/`cbi`, which IS the documented workaround, and a
scan of the built images finds zero `sts` stores below 0x40 in any of
them. A runtime-computed pointer into that space would be the
exposure, and the stratum has none. The
DA errata of record (DS80000882C, 10/2021) carries no twin item, but
it predates the DB one (first published in DS80000915E, 04/2024), so
its silence is not evidence: the NOP costs one cycle and is emitted on
both families.

**Reset (RSTCTRL, 14).** Six sources - power-on, brown-out, the RESET
pin, the watchdog, a software write, the UPDI - each with a flag in
`RSTFR`. Two facts decide how the flags must be used: they are
**write-one-to-clear**, and they **accumulate**. Nothing but software
clears them, except that a power-on reset clears everything but PORF
and a brown-out clears everything but PORF and BORF (14.5.1). Software
that never clears `RSTFR` therefore reads the history since power-up,
not the last reset. A software reset is a CCP-protected write of
`SWRR.SWRST` and starts about 150 ns later (14.3.2.1.5).

**SRAM across a reset is not promised.** The data sheet states that
RAM is kept in sleep (13.2) and lists what each reset clears (table
14-1, corrected by errata 3.3.1 to add a "reset of BOD configuration"
column and to mark BOD as resetting the UPDI); SRAM appears in neither
list, for any source including power-on. The C runtime is what leaves
`.noinit` alone. That is why `take_panic_record()` checks a magic word
before believing what it finds: cold RAM makes the check necessary,
not merely prudent.

**Watchdog (WDT, 22).** A counter on the 1.024 kHz output of OSC32K,
asynchronous to CLK_PER, running in Active mode and every sleep mode,
and surviving a main-clock failure. `CTRLA.PERIOD` (non-zero enables
it) and `CTRLA.WINDOW` share one encoding, 8 ms to 8 s in eleven
steps; the nominal times are 7.8125 ms .. 8 s, not the round numbers.
`WDR` restarts the period. In Window mode the closed period comes
first and a WDR inside it resets the device exactly as a missing one
does - but **the window is activated by the first WDR after it is
enabled** (22.3.3.2), so that first one is never judged. Both `CTRLA`
and `STATUS.LOCK` are CCP-protected (22.3.7); `CTRLA` must not be
written while `STATUS.SYNCBUSY` is set (22.3.6), and a WDR needs two
to three WDT cycles to synchronize. `LOCK` is one-way in software:
once set, `CTRLA` is read-only until a reset or a debugger, so a
locked watchdog cannot be disabled by the program it is watching.
`FUSE.WDTCFG` supplies `CTRLA`'s reset value, and a non-zero PERIOD
there sets `LOCK` at boot.

**Waiting on this core.** The AVR executes one instruction in a known
number of cycles from flash, with no prefetch, cache or wait states,
so counting cycles IS timing - which is why the short-wait role is a
cycle loop here and a hardware counter (DWT CYCCNT, SysTick, mcycle)
on the cores that have one.

## Types and verbs

`AvrPlatform` (`avrdx/platform_avr.hpp`) is the `Platform` concept for
this target: the RAII `CriticalSection` (save SREG, `cli`, restore -
so guards nest and a guard entered with interrupts masked leaves them
masked), `idle()`, `break_here()`, `now()` and the compile-time
`ticks_per_second` over `Ticker`, `atomic_width` = 1, and
`panic_record()` - a `PanicRecord` in `.noinit`. Two readbacks beyond
the concept let a program (and the suite) check what the others did:
`sleep_armed()` reads `SLPCTRL.CTRLA.SEN`, `interrupts_enabled()`
reads `SREG.I`. `idle()` sleeps in IDLE unless a deeper mode is already
armed, in which case it takes that one instead - see `AvrSleepSite`
below.

`avrdx/delay.hpp` is the short-wait role: `delay_us(clock, us)` reads
the rate from the `Clock` type and never from `F_CPU`, and NO DIVISION
EVER RUNS AT WAIT TIME. A static clock's rate folds; a `DynamicClock`
can only run at one of the discrete rates its type enumerates
(`rate_count` / `rate_hz(i)` / `rate_index()`, from the twelve main
prescalers), so `delay_us` dispatches on the current rate's INDEX into
per-rate branches expanded at compile time: a constant `us` folds to
the exact loop inside its branch, a runtime `us` picks the rate's
Q4.12 loops-per-us factor (`delay_mult`, rounded up at compile time)
and runs one shared 16x16-multiply tail - exact at every reachable
rate, sub-MHz included. `delay_us_runtime(cycles_per_us, us)` stays as
the stored-byte pattern for code that holds a rate but not a clock
type (whole cycles per us: gross below 1 MHz), `delay_cycles(n)`
counts raw cycles for rates below 1 MHz. Every path rounds UP: a setup
time is "no less than", by construction and not by a tuned constant.

`avrdx/sleep.hpp` is the mechanism of chapter 13, and only the
mechanism. `SleepMode` names the three modes; `Sleep` arms one
(`arm`), disarms (`disarm`), reads back what is armed (`armed`,
`armed_mode`), executes the instruction (`sleep`) and offers the
bounded verb an application usually wants, `enter(mode)` = arm + sleep
+ disarm. The pair exists as separate verbs for the callers that must
close the lost-wake-up window themselves: arm first, then mask, test
the condition and put `sei` and `sleep` back to back - which is exactly
the sequence `AvrPlatform::idle()` emits for IDLE. `Vreg` covers
`VREGCTRL` through its CCP key: `power(VregPower)` / `power()` for the
regulator profile, `high_temp_low_leakage(bool)` / `()` for HTLLEN -
and **enabling HTLLEN returns false, writing nothing, while any TWI
client or the CCL is enabled**. That refusal IS chapter 13's warning:
a rule stated in a comment and not enforced is a rule the next program
breaks.

`AvrSleepSite`, in the same header, is this target's `SleepSite` (the
power model of [../design/power.md](../design/power.md)): it maps the
model's depth ladder onto SMODE - `none` disarms, `light` = IDLE,
`standby` = STANDBY, `deep` = PDOWN - and does nothing else. This family
realizes every rung, so the model's "map an absent rung to the nearest
shallower one" rule is the identity here and `armed()` reads back
exactly what was asked.

**The site only ARMS; the SLEEP instruction stays the kernel loop's.**
That is why `AvrPlatform::idle()` is IDLE by DEFAULT rather than IDLE
only: if `SEN` is already set when it runs, something above the kernel
armed a mode on purpose, so `idle()` leaves that arming alone and takes
it - `sei` + `SLEEP`, and no disarming store on the way out, because the
arming is the power manager's to clear (it does so on the first event it
dispatches after the wake). With nothing armed it behaves exactly as
before: arm IDLE, sleep, disarm. This one branch is the whole of what
the power model needs from the target; there is no new kernel hook.

The POLICY otherwise stays out of this header. Standby and power-down
gate clock domains and shorten the wake-up list, so entering them is a
decision about the whole application - which peripherals must survive,
which oscillator must stay up for them, what is allowed to wake the
program. That negotiation is `util/power.hpp`'s; this header is the
mechanism underneath it.

`avrdx/reset.hpp` has two resources. `Reset` reads why we are running
- `flags()` peeks, `take_flags()` reads and clears in one verb (the
one to call first at boot), `clear_flags()` wipes - and `software()`
performs a software reset, never returning. `ResetFlags` names the six
sources and keeps the raw byte. `Watchdog` covers chapter 22:
`arm(period, window)`, `off()`, `clear()` (the WDR), `sync()` (the
bounded wait that says the configuration is now in force),
`busy()`, `locked()`, `lock()`, `enabled()`, `period()`, `window()`,
and the free `wdt_time_us()` for the nominal duration of a `WdtTime`.

## How to use it

**Wait for a hardware setup time.** The common case, with a constant:

```cpp
brio::delay_us(clock, 10);            // folded to an exact cycle loop
```

**Wait by an amount computed at run time**, or under a `DynamicClock`:

```cpp
brio::delay_us(clock, cs_setup_us);   // fixed-point loops, no division
brio::delay_cycles(200);              // when the clock is below 1 MHz
```

**Guard data shared with an ISR** (and let the guard nest):

```cpp
{
    brio::AvrPlatform::CriticalSection cs;
    snapshot = shared_counter;
}                                     // SREG restored, not blindly sei'd
```

**Report the previous reset at boot**, breadcrumb and cause together:

```cpp
const brio::ResetFlags why = brio::Reset::take_flags();      // read + clear
const auto record = brio::take_panic_record<brio::AvrPlatform>();
if (record) {
    brio::print(serial, "panic code ", record->code,
                " context ", record->context,
                why.watchdog ? " (watchdog reset)" : " (other reset)",
                brio::crlf);
}
```

**End a panic in a reset instead of a halt** - the composition
`kernel/panic.hpp` advertises, since the breadcrumb is written before
any reporter runs:

```cpp
struct ResetReporter {
    [[noreturn]] static void report(brio::PanicCode, uint8_t) {
        brio::Reset::software();
    }
};
brio::panic<brio::AvrPlatform, ResetReporter>(brio::PanicCode::kernel_fault, id);
```

**Let the watchdog catch a hang**, with the same breadcrumb:

```cpp
brio::Watchdog::arm(brio::WdtTime::ms250);   // then WDR from the main loop
brio::Watchdog::clear();
```

**Use Window mode** (a WDR too early is a fault too). The window is
armed only after the configuration has synchronized and the first WDR
has activated it:

```cpp
brio::Watchdog::arm(brio::WdtTime::ms250, brio::WdtTime::ms64);
(void)brio::Watchdog::sync();     // the new configuration is in force
brio::Watchdog::clear();          // activates the window; never judged
```

**Sleep when there is nothing to do.** The kernel does this for you;
the shape is what makes it race-free:

```cpp
cli();
if (nothing_to_do()) {
    brio::AvrPlatform::idle();    // sei + SLEEP: no wakeup can slip in
} else {
    sei();
}
```

**Sleep deeper than IDLE, deliberately.** The shape is: arm what must
wake you, make sure its clock survives the mode, then enter. A standby
sleep around a PIT wake-up, with a TCB that has to go on counting
through it:

```cpp
brio::Pit::init(brio::PitPeriod::cyc4096);        // 125 ms, interrupt on
brio::Tcb<1>::init({.mode = brio::TcbMode::capture,
                    .clock = brio::TcbClock::div1,
                    .run_standby = true});        // ITS OWN flag: the request
brio::Sleep::enter(brio::SleepMode::standby);     // interrupts on, wake armed
```

Nothing else is needed to keep CLK_PER alive: the TCB's request does
it. Set the oscillator's own `RUNSTDBY` when the point is a fast
wake-up instead (`brio::Oschf::run_standby(true)`), and keep
`Vreg::power(VregPower::performance)` for the case where even the
regulator's start-up matters.

**Sleep deeper than IDLE, under a kernel.** `Sleep::enter` is for code
that owns the moment. An active-object program instead asks its power
manager, which asks everyone else - the negotiation is
[../design/power.md](../design/power.md); what belongs to this target is
one type name in the declaration and one branch in `idle()`:

```cpp
using Pm = brio::PowerManager<brio::AvrPlatform, brio::AvrSleepSite,
                              brio::PowerConfig{}, Bus, Sensors>;
using K = brio::Kernel<brio::AvrPlatform, Sensors, Bus, Pm>;
...
brio::post<Pm>(brio::SleepRequested{brio::SleepDepth::standby,
                                    brio::reply_to<Supervisor, brio::SleepVote>()});
```

The wake source is still armed by the application, and it must be one
the armed mode can reach: the manager negotiates whether the program may
stop, not whether it can come back. The kernel loop's `idle()` then
takes the armed mode by itself, and the manager disarms on the first
event it dispatches afterwards.

## Bench findings

Established by `test_avr_platform`, on the 24 MHz crystal. Every
figure is CLK_PER cycles between two hardware-latched stamps of a
32-bit TCB pair, with the cost of two back-to-back stamps (70 cycles)
subtracted and interrupts masked.

- **The folded path is EXACT.** `delay_us(clock, N)` with a static
  clock and a constant `N` measures the nominal cycle count with zero
  overhead, at 1, 2, 5, 10, 50 and 100 us alike: 24, 48, 120, 240,
  1200, 2400 cycles. The compiler emits an in-line down-counting loop
  (`ldi r24,0x08 / dec / brne` for 1 us) - no call, no arithmetic.
- **The runtime path costs a constant 122 cycles** at a static rate,
  whatever the length: nominal + 122 at 1, 2, 5, 10, 50 and 100 us
  alike - the Q4.12 multiply, the slice bookkeeping and the one extra
  loop turn that replaces an exact ceil. `delay_us_runtime(24, 1)`
  called directly (the out-of-line stored-byte tail) is 171 cycles, of
  which the loop is the nominal 24. Every path honours "at least" at
  every length tested.
- **A dynamic clock costs 157 cycles per call with a runtime `us`, and
  SIX with a constant one.** The rate is dispatched by INDEX into
  branches folded per rate, so no arithmetic derives the rate at wait
  time: a constant `us` lands in its branch's exact folded loop (24006
  measured for a nominal 24000 - the six cycles are the index chain),
  a runtime `us` selects the branch's Q4.12 factor into the shared
  tail (157..160 cycles, identical at 24, 12 and 1.5 MHz; the suite
  caps it at 200 so the old runtime division - it cost 693 - cannot
  come back unnoticed). The per-microsecond cost stays exact: 24000
  cycles per 1000 us at 24 MHz, 12000 at 12 MHz.
- **Sub-MHz rates are exact now, not 4/3 long.** At 1.5 MHz (24 MHz
  divided by 16, a rate the prescaler really reaches) the per-rate
  factor is exactly 0.375 loops/us (1536 in Q4.12): 1000 us measures a
  1502-cycle slope = 1001 us. The old whole-cycles-per-us rounding
  (`cycles_per_us(1'500'000)` = 2, still what the stored-byte helper
  does) ran every delay 4/3 long. What no arithmetic fixes at such
  rates is granularity and overhead in TIME: one loop turn is 4 cycles
  and the 157-cycle dispatch is ~105 us of them - below roughly 1 MHz
  a microsecond-denominated busy-wait is out of its domain, and the
  honest tools are the folded path and `delay_cycles`.
- **`delay_cycles` rounds up to a multiple of four and costs 39
  cycles** on top: 43, 47, 139, 1039 cycles for 4, 8, 100, 1000.
  Crossing the 16-bit chunk boundary is honest - 262152 cycles (65538
  loops, one full 0xFFFF chunk plus a remainder of 3) measures 262201,
  i.e. the same 39 plus 10 cycles for the extra loop's arithmetic.
- **A critical section is three cycles** (`in` / `cli` / `out`), two
  nested are eight. Entered with interrupts already masked it leaves
  them masked; the inner exit of a nested pair restores rather than
  enables; the outer exit gives the caller its own state back.
- **Waking from IDLE costs six CLK_PER cycles.** With the same alarm
  stamping the clock inside its own ISR, the interrupt lands 24043
  cycles after the arming when the CPU slept and 24037 when it spun -
  a six-cycle difference, exactly 13.3.3.2's figure.
- **The CPU really stops.** Over the same 32 ticks a counter turns
  ~13600 times awake and exactly 32 times asleep, one per wake.
  `idle()` returns only after an interrupt has fired, leaves
  `SLPCTRL.CTRLA` at 0 and interrupts enabled, and waits at most one
  tick period. A caveat the suite had to learn: an unfinished console
  line leaves the USART's DRE interrupt armed, and THAT is then what
  does the waking - one wake per frame, every ~250 cycles at 460800.
- **The generated sleep sequence is right.** In the disassembly the
  `sei` is immediately followed by `sleep` (nothing can slip into the
  lost-wakeup window), the disarming store to `SLPCTRL.CTRLA` is
  immediately preceded by the erratum's NOP, and the arming store is
  preceded by an `ldi` - not by another store, which is what 2.2.4
  actually forbids.
- **The ring is lock-free on this silicon and loses nothing.**
  50000 elements pushed by a 20 kHz TCB interrupt and popped in the
  main loop arrive in order, uncorrupted, in 625 ms with zero pushes
  refused. Overrun on purpose and a full ring refuses instead of
  overwriting: exactly 63 (capacity) queued, the OLDEST 63 in order,
  1129 refusals counted. The kernel's `EventQueue` counts its
  overflows and saturates at 65535 rather than wrapping.
- **The timebase measures +8843 ppm fast** against the crystal (1024
  ticks in 23786556 CLK_PER cycles instead of 24000000) - the same
  OSC32K figure the RTC campaign found. `now()` never goes backwards
  and advances one tick at a time across low-byte wraps.
- **The breadcrumb survives real resets, both kinds.** After a
  watchdog reset `RSTFR` reads exactly 0x08 and `take_panic_record()`
  returns the magic, code and context `panic()` wrote, with a second
  take returning nothing; after a software reset from inside a
  reporter it reads 0x10 and the record is equally intact. A boot
  after a clean take reports no record at all. The `.noinit` token the
  suite carries for itself crosses all four resets unchanged.
- **The watchdog behaves as chapter 22 says, including the trap.** The
  WDTCFG fuse is off on this board, so `LOCK` is clear and `CTRLA`
  starts at zero. `arm()` returns before its configuration is running:
  a WDR issued immediately after enabling Window mode does not violate
  it. And even after `sync()`, ONE WDR is not a violation - it is what
  activates the window (22.3.3.2). Two WDRs, 5 ms apart, inside a
  62.5 ms closed window: watchdog reset, `RSTFR` 0x08. `lock()` reads
  back, makes `arm()` refuse with nothing written, and is released by
  the next reset.
- **`break_here()` falls through as a plain NOP** with no debugger
  attached - the suite goes on running past it.
- **A UPDI flash shows as `RSTFR` 0x20**, which is how a fresh boot
  after `bench.py flash` is told apart from every other start.

### The sleep modes

Established by `test_avr_sleep`, whose single-board set `z` is 72
verdicts on rev A5 at 5 V with nothing wired. Two instruments carry
every number below: a 32-bit TCB pair counting
CLK_PER says whether the main clock ran, and the RTC counter says
whether the 32 kHz domain did, both read across one PIT period of 4096
CLK_RTC cycles. The two rulers measure the same interval, which is what
makes the third number possible: 4096 CLK_RTC cycles measure 2966514
CLK_PER ticks against the 24 MHz crystal, so this board's OSC32K runs
at 33137 Hz - **+11 000 ppm**, one tick being 30.18 us.

- **The erratum's NOP is where it must be.** Every store to
  `SLPCTRL.CTRLA` in a built image is preceded by the NOP, with only
  `ldi`/`mov`/`ori` ever between the two - never another store, which
  is what 2.2.4 forbids. A deliberate high-address store immediately
  before `arm()` still arms.
- **HTLLEN is refused, not merely deprecated.** With the CCL enabled,
  and again with a TWI client enabled, `high_temp_low_leakage(true)`
  returns false and `VREGCTRL` is untouched; with both off it is
  accepted; disabling is never refused. `PMODE` survives an HTLLEN
  write and vice versa (they share one CCP-protected byte).
- **`Sleep::enter(idle)` is the platform's `idle()` under application
  control.** Sixteen sleeps on a 2 ms TCB alarm return only after the
  interrupt, leave `SEN` clear and interrupts enabled; a counter that
  only turns while the CPU runs turns 16 times across those sixteen
  periods and 23937 times over the same span awake.
- **Standby really stops CLK_PER.** Over one 125 ms PIT period the
  stopwatch counts 2972823 ticks awake and **178 asleep** - the 7 us of
  code between the wake-up and the read. The PIT interrupt is what
  comes back, and `SLPCTRL` is disarmed on the way out.
- **The peripheral's RUNSTDBY is the whole chain.** Same sleep, four
  configurations, CLK_PER ticks counted through it: crystal with the
  TCB's flag clear **178**; crystal with it set **2970833** (its own
  oscillator flag on) and **2971371** (that flag off); OSCHF with it
  set **2966515** (oscillator flag off) and **2967628** (on). The
  oscillator's own `RUNSTDBY` changes nothing while a peripheral is
  requesting; without the peripheral's flag, nothing counts. The two
  sources are told apart by the count itself - this board's OSCHF is
  ~0.15 % slower than its crystal.
- **A TCB wakes the CPU from standby by itself** when its chain is
  complete: its 2 ms period ends the sleep after 47909 CLK_PER ticks
  (nominal 48000, the missing 91 being the counter's restart). The same
  TCB with `RUNSTDBY` clear never fires - the PIT has to end that sleep.
- **A pin wakes from standby AND from power-down, with no wire.** EVSYS
  is alive in every sleep mode, so a PIT divider routed to `EVOUT` on
  PD2 drives that pad while the CPU sleeps and PD2's own (fully
  asynchronous) edge sense picks it up. One pad interrupt ends the
  sleep in both modes, with the stopwatch frozen at 176 / 161 ticks and
  a 1 s PIT backstop that never gets to fire. The event system
  therefore keeps ROUTING while every clock but the 32 kHz one is
  stopped.
- **Power-down stops the RTC counter and keeps its PIT.** With
  `RUNSTDBY` set on the counter: across a standby sleep it advances
  4129 CLK_RTC ticks (the PIT period), across a power-down sleep **33**
  - which is the 1 ms of settling after the wake, i.e. nothing at all
  during the sleep. CLK_PER is dead in power-down whatever any
  `RUNSTDBY` says (162 ticks, the post-wake tail), and the PIT
  interrupt still arrives.
- **`RTC.CNT` read at the instant of a wake is STALE.** It returns the
  value it had when the SLEEP instruction ran, in both deep modes; read
  1 ms later it is right. The read is synchronized into CLK_PER (26.10)
  and that path needs the clock back and a CLK_RTC edge. Anything
  timing a sleep by the RTC counter must settle first - the suite does.
- **What an oscillator's RUNSTDBY buys is the wake-up.** Measured as
  CLK_RTC ticks (30.18 us) between the PIT's own edge and the woken
  program, with a spinning baseline subtracted: crystal with its
  `RUNSTDBY` set **0 ticks**, cleared **+48**; OSCHF with it set **0**,
  cleared **+10**. The second board's 41.7 ns ruler puts real numbers
  on those three classes below.
- **`PMODE` shortens the OSCHF restart in that sweep** (+10 ticks in
  Normal against **+1 in Performance**) and changes nothing on the
  crystal (+48 either way). WHY it helps there and not always is the
  next section's finding. With HTLLEN set, the PIT still wakes the
  device from power-down.

### Wake-up latency, measured from a second board

A sleeping chip cannot time its own return: the only clock power-down
leaves running is the PIT's, and the counter that would measure the
restart is the one the mode stops. So the ruler is off-chip.
`test_avr_sleep`'s two-board set `y` (49 verdicts) drives board B
(`sleep_peer`) over a one-wire link: B zeroes a 32-bit CLK_PER
stopwatch and raises a stimulus pin in the same instruction pair, this
board's PORT ISR raises an echo pin as its first statement, and that
edge CAPTURES the stopwatch through B's event system. One tick is
41.7 ns (B's OSCHF at 24 MHz nominal, a per-cent-class reference -
ample for these figures). Medians of eight shots.

- **The fixed cost of the measurement is 23 ticks - 958 ns.** That is
  the AWAKE baseline: B's zero-to-edge gap, the wire, this chip's
  interrupt latency and the ISR prologue up to the store. Every number
  below includes it and is read against it.
- **IDLE costs 30 ticks, 7 more than awake** - 292 ns against the six
  CLK_PER cycles (250 ns) 13.3.3.2 promises and `test_avr_platform` f
  counts on-chip.
- **STANDBY, five clock configurations** (medians, and the whole point
  is the last two):

  | main clock | other oscillators | `PMODE` | ticks | us |
  |---|---|---|---|---|
  | crystal, its `RUNSTDBY` clear | - | Normal | 42452 | 1768 |
  | crystal, its `RUNSTDBY` set | - | Normal | 43 | 1.8 |
  | OSCHF | crystal still oscillating | Normal | 567 | 23.6 |
  | OSCHF | crystal still oscillating | Performance | 567 | 23.6 |
  | OSCHF | all stopped | Normal | 7510 | 313 |
  | OSCHF | all stopped | Performance | 568 | 23.7 |

- **THE REGULATOR IS PAID FOR SEPARATELY, AND ONLY WHEN THE DEVICE
  REALLY LETS GO.** `VREGCTRL`'s AUTO profile drops to low power "in
  Standby and Power-Down, and whenever OSC32K is the only clock
  running" (13.3.5) - and the bench reads that sentence strictly: an
  oscillator left running by its own `RUNSTDBY` keeps the regulator at
  full drive EVEN WHEN IT IS NOT THE MAIN CLOCK SOURCE. Beside a
  running crystal, an OSCHF restart out of standby is 23.6 us and
  `PMODE` moves it by nothing; stop the crystal and the same restart
  becomes 313 us, of which `PMODE = FULL` removes 290. The 24-30 us of
  table 13-5 is the oscillator; the other 290 us is the regulator.
- **A crystal main clock costs 1.77 ms to restart**, in standby and in
  power-down alike, and no regulator profile touches it: its own
  start-up buries everything else. Kept alive by its `RUNSTDBY` it
  costs 43 ticks - 20 ticks, 833 ns, over the awake baseline, which is
  standby's own six cycles plus the synchronization.
- **POWER-DOWN has no oscillator to hold the regulator up**, so the
  Performance profile earns its keep unconditionally there:

  | main clock | `PMODE` | ticks | us |
  |---|---|---|---|
  | crystal | Normal | 42615 | 1775 |
  | OSCHF | Normal | 7518 | 313 |
  | OSCHF | Performance | 567 | 23.6 |

  Same three classes as standby, and the OSCHF figures are the same
  313 us / 23.6 us: power-down is not slower than a standby that has
  been allowed to drop everything - it is that same case, made
  unavoidable.
- **A pin edge wakes from power-down every time.** The stimulus arrives
  on a Px2 pin, one of the two fully asynchronous positions of every
  port (port.md), which is what makes an edge a wake-up source with
  every clock in the device stopped. Eight shots per configuration,
  three configurations, no misses.
- **A TWI address match wakes from both deep modes, and the wire pays.**
  This board as a client at 0x42 on the office bus, board B writing
  three bytes at 100 kHz: the tenure lasts 418.7 us with this board
  awake, **418.7 us out of standby** with the main clock kept alive
  (the 1.8 us wake is invisible under a 10 us SCL bit) and **2197 us
  out of power-down** - exactly the 1.78 ms crystal restart measured
  above, held on SCL by the client from the address match until the
  first serviced byte. The frame arrives intact in all three cases.
  Only the ADDRESS MATCH is a wake-up source: the data interrupts that
  carry the rest of the frame are not, so a program that goes back to
  sleep after the match stalls the tenure with the host holding SCL.
- **A USART start bit wakes from standby, and what it costs the frame
  is a rule** - see usart.md, whose start-of-frame section this suite
  closes.
- **The CCL wakes from standby when it has RUNSTDBY, and from
  power-down when its clock survives the mode** - see ccl.md.

### The power manager on this target

Established by `test_avr_power`, 5 letters / 44 verdicts on board B at
5 V with nothing wired. It is the only suite here that runs the KERNEL:
the object under test is an active object, so the rounds go through real
queues and real dispatch, and only the loop is the suite's.

- **The ladder maps one-to-one onto SMODE.** `SLPCTRL.CTRLA` reads
  0x00, 0x01, 0x03, 0x05 for `none`, `light`, `standby`, `deep`, and
  the site and the manager both read back the depth that was asked -
  this family has every rung, so the model's map-it-shallower rule is
  the identity here.
- **The kernel's own idle hook takes the armed mode.** With `standby`
  armed by the manager and nothing else changed, `Kernel::idle_if_empty()`
  stops the CPU for real: over 32 ticks a counter that only turns while
  the CPU runs turns ~13500 times awake and **exactly 32 times asleep**,
  one per PIT wake. The mode STAYS armed across a wake that says nothing
  to the manager, so the program stops again on the next empty turn.
- **A standby wake beside a clock the sleeper keeps alive costs 10-12
  CLK_PER cycles.** Measured as a whole PIT period from a stamp taken
  with interrupts masked to the wake ISR's own stamp: 23052 cycles
  asleep against 23042 spinning. The stopwatch has `RUNSTDBY` on both
  halves - which is what lets it time the sleep at all, and which by
  the chain rule above keeps the crystal running - so this is the
  "kept alive" row of the latency table (1.8 us there includes a wire
  and a second board's ISR), not a cold restart.
- **A full round costs 3771 CLK_PER cycles - 157 us** from the `post()`
  of the request to the site being armed, with two voters answering
  through the kernel. That is the price of the negotiation, and it is
  paid once per sleep, not once per wake.
- **The deadline guard works on the real timebase.** With a periodic
  time event one tick away, a `deep` request is refused and no voter is
  even asked; the same request with the nearest deadline 1000 ticks away
  is accepted, as is one with nothing armed at all. A `light` request is
  never guarded.
- **A `BusMaster` mid-transfer really refuses.** With a transfer in
  flight the round aborts and nothing is armed; the other stakeholder is
  still asked first, because the manager waits for unanimity rather than
  stopping at the first no. Posting the engine's completion makes the
  identical request succeed.
- **Restrictions clamp, and the voters see the clamp.** Under
  `restrict(light)` a `deep` request arms `SLPCTRL.CTRLA` = 0x01 and the
  voters are asked at `light`, not at `deep`. Nested locks: the
  shallowest live one wins, releasing it falls back to the other,
  releasing twice does not release someone else's, and a moved lock
  keeps the right alive until the mover's scope ends.
- **The first event after a wake disarms and reports.** `CTRLA` returns
  to 0x00, `sleep_armed()` reads false, the manager forgets the round
  and `WakeReport{standby}` reaches the stakeholders that declare it -
  and the `SleepRequested{none}` that carried the wake still replies ok.
- **The timebase's own error shows up in the sleeping window.** The
  RUNSTDBY stopwatch counts 743000-744000 CLK_PER ticks across 32 PIT
  ticks where 1024 Hz exactly would be 749984: OSC32K running ~0.8 %
  fast, the same +8000-to-+11000 ppm this desk measures everywhere else.

## Not covered yet

Driver gaps:

- **Idle detection and the `RUNSTDBY` policy.** The power manager
  (`util/power.hpp`, `AvrSleepSite`) is built and the negotiation runs,
  but nothing decides WHEN a program has nothing left to do - something
  has to post the request - and no driver-wide policy says which
  peripherals must survive which depth. Both are application knowledge
  by design ([../design/power.md](../design/power.md), "What is
  deliberately not here").
- **`FUSE.WDTCFG`.** The driver reads the consequences (`LOCK`,
  `CTRLA`'s reset value) but writes no fuse: fuse programming is a
  UPDI-time concern, not a runtime one.
- **The BOD**, and with it the voltage-level monitor that tables 13-4
  lists as a wake-up source in both deep modes. Chapter 15 has no
  driver at all yet.

Implemented but not bench-verified:

- **`break_here()` with the OCD active.** The NOP half is proven; the
  half that halts in the debugger needs a debug session, which no
  console suite can be.
- **Three of the six reset flags.** POR, BOR and EXTRF are decoded and
  printed but this suite causes none of them - it can reach WDRF,
  SWRF and (through the programmer) UPDIRF only. A power cycle and a
  RESET-pin pulse are desk actions, not verdicts.
- **The watchdog through sleep and a clock failure.** 22.2 promises
  both; neither has been staged here.
- **The two wake-up sources no second board can produce**: MVIO and the
  BOD's voltage-level monitor. The suites prove the PORT pin, the PIT,
  a TCB, the CCL, a USART start bit and a TWI address match.
- **HTLLEN's other half.** That a TWI address match and the CCL STOP
  waking the device once HTLLEN is set is an ABSENCE, and the driver
  refuses the combination that would need it - asserted in situ, with
  a client really enabled, rather than measured.
- **The sleep CURRENT.** Every figure here is about time, not
  microamps: what `PMODE`, `HTLLEN` and a stopped oscillator actually
  save is a bench-supply measurement this desk cannot make. It is the
  one thing the power manager cannot be judged on from inside the chip.
- **A power-manager round into POWER-DOWN.** The manager arms PDOWN and
  the register readback proves it, but no letter of `test_avr_power`
  then executes the sleep: from power-down the only wake sources are
  the PIT, a pin and a TWI match, and the suite's own stopwatch stops
  there, so the round is verified up to the arming and the sleeping
  itself is `test_avr_sleep`'s.
- **A `PowerLock` taken from a real ISR.** The counters are guarded and
  the verbs are exercised from the main context; that an interrupt may
  take and release one mid-burst is asserted by construction, not
  measured.
- **Every WDT period but 8 ms, 64 ms and 8 s**, and the timing of a
  time-out itself: the encoding is exercised through `arm()`'s
  readback and `wdt_time_us()`'s arithmetic, but only the three used
  by the reset legs have been timed against the silicon.
