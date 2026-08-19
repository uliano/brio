# RTC / PIT - the real-time counter and periodic interrupt timer (AVR DA/DB)

> **PROVISIONAL.** Not the result of a systematic review of the RTC
> chapter and errata; the driver uses only the PIT half, as the kernel
> timebase. The exhaustive pass is pending. Documents consulted:
> AVR128DB28/32/48/64 data sheet DS40002247B (RTC), errata
> DS80000915F (no RTC items). Driver: `avrdx/ticker.hpp` (`BasicTicker`,
> `Ticker`). Reference tests: every kernel app (time events), `events0`
> (PIT dividers as event generators: 512 Hz and 4 Hz measured).

## What the driver does today

`BasicTicker<tps>` (`Ticker` = 1024 Hz): the PIT of the RTC, clocked
from the 32.768 kHz source the clock init left running (XOSC32K if a
crystal was started, the internal OSC32K otherwise; RTC prescaler 1),
raises the periodic interrupt at a power-of-two rate 16..1024 Hz; the
`pit()` body advances the tick count and the millisecond count (with
the 1000/1024 correction), and by firing wakes the CPU from idle. It
is the kernel's `now()` and `ticks_per_second`, alive in idle and
standby, independent of the CPU clock. The PIT's divided clocks are
also event generators (`EvPitDiv<n>`, [events.md](events.md)).

## Types and verbs

| Entity | Verbs |
|--------|-------|
| `BasicTicker<tps>`, `Ticker` | `init()`, `pit()` (ISR body for `RTC_PIT_vect`), `ticks()`, `millis()`, `secs()`, `now(TimeStamp&)`; `tps`, `millis_per_tick` |
| kernel side | `AvrPlatform::now()` = ticks; `ticks_from_ms<P>()`, `TimeEvent` ([kernel.md](kernel.md)) |

## How to use it

```cpp
ISR(RTC_PIT_vect) { brio::Ticker::pit(); }
...
brio::Ticker::init();        // after clock init, before sei()
brio::TimeStamp ts; brio::Ticker::now(ts); brio::print(serial, ts);   // "12.045s"
```
Waiting in an AO is a `TimeEvent`, never a busy loop.

## Bench findings

- OSC32K-derived rates within ~0.2 % on this part (513 event-started
  conversions per second for a nominal 512).
- The RTC prescaler is left at 1, so `EvPitDiv<n>` = 32768 / n Hz.

## Not covered yet

The RTC counter itself (16-bit CNT, PER, CMP, overflow and compare
interrupts and events), its 15-bit prescaler, crystal error correction
(CALIB), clock source selection beyond "what clock init started"
(external clock on the XOSC32K pin, OSC32K/1k), RUNSTDBY choices,
PIT periods as wake-up sources for standby, the synchronization
(STATUS busy bits) rules for register writes, the `BasicTicker`
limitation to powers of two (a truth of this PIT, the reason the
kernel tick is opaque).
