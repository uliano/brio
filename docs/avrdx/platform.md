# Platform - what the kernel stands on (AVR DA/DB)

> **PROVISIONAL.** The three blocks this page covers - SLPCTRL,
> RSTCTRL and the WDT - are described in full and every claim below is
> measured, but only ONE of the three sleep modes is exposed: `idle()`
> uses IDLE, and Standby / Power-Down (with the voltage regulator knobs
> that go with them) belong to a power-management pass that does not
> exist yet. The list is in "Not covered yet".

Documents of record: AVR128DB28/32/48/64 data sheet DS40002247B -
SLPCTRL chapter 13, RSTCTRL chapter 14, WDT chapter 22, the CPU's
Configuration Change Protection in 7.4.6 - plus errata DS80000915F
(item 2.2.4 is code here; clarifications 3.3.1 and 3.4.1 rewrite two
of those chapters' tables) and, for the DA parts, DS80000882C, which
lists no SLPCTRL, WDT or CPU item at all. Drivers:
`avrdx/platform_avr.hpp` (`AvrPlatform`, this target's realization of
the kernel's `Platform` concept), `avrdx/delay.hpp` (the short-wait
role) and `avrdx/reset.hpp` (`Reset`, `Watchdog`). Reference test:
`test_avr_platform`.

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
make the device go to sleep" (13.3.1). In IDLE the CPU stops and
nothing else does: every peripheral runs and every interrupt source
wakes (13.3.3.1). Waking costs six CLK_PER cycles (13.3.3.2, measured
below). `CTRLA` is **not** under CCP - only `VREGCTRL` is (13.3.5).
The register file and SRAM are kept through sleep (13.2), and a
debugger break wakes the device whether or not an interrupt is pending
(13.3.4).

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
reads `SREG.I`.

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

## Not covered yet

Driver gaps:

- **Standby and Power-Down.** `idle()` arms IDLE only. The deeper
  modes gate clock domains and shrink the wake-up list (tables 13-2
  and 13-4, both corrected by errata 3.4.1), which makes entering them
  a power-management decision an application takes deliberately - not
  something the kernel's "nothing to do" hook may do behind its back.
  There is no verb for them yet, and no `RUNSTDBY` policy across the
  drivers that would need one.
- **`SLPCTRL.VREGCTRL`.** `PMODE` (AUTO / FULL regulator drive) and
  `HTLLEN` (the high-temperature low-leakage bit, Power-Down only,
  incompatible with TWI address-match and CCL wake-up) are not
  exposed. They belong with the sleep modes that give them meaning.
- **`FUSE.WDTCFG`.** The driver reads the consequences (`LOCK`,
  `CTRLA`'s reset value) but writes no fuse: fuse programming is a
  UPDI-time concern, not a runtime one.

Implemented but not bench-verified:

- **`break_here()` with the OCD active.** The NOP half is proven; the
  half that halts in the debugger needs a debug session, which no
  console suite can be.
- **Three of the six reset flags.** POR, BOR and EXTRF are decoded and
  printed but this suite causes none of them - it can reach WDRF,
  SWRF and (through the programmer) UPDIRF only. A power cycle and a
  RESET-pin pulse are desk actions, not verdicts.
- **The watchdog through sleep and a clock failure.** 22.2 promises
  both; neither has been staged here (they wait on the sleep pass).
- **Every WDT period but 8 ms, 64 ms and 8 s**, and the timing of a
  time-out itself: the encoding is exercised through `arm()`'s
  readback and `wdt_time_us()`'s arithmetic, but only the three used
  by the reset legs have been timed against the silicon.
