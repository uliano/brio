# Meters in the kernel

A hardware meter - a timer in a capture mode, an encoder counter, a
frequency divider with a readable register - produces a number per
EDGE of the signal it watches. The rate of those numbers is the
signal's, not the program's: a 10 kHz input is ten thousand readings a
second, each one an interrupt. Two target-independent pieces sit
between such a meter and the active objects that want its numbers, and
between them they separate the two paces:

- **`MeterLatch`** (`util/meter_sampler.hpp`) - the one-cell bridge out
  of the capture interrupt;
- **`MeterSampler`** (same header) - the active object that walks a
  fixed list of sources on ITS own period and publishes what is fresh.

The meters themselves are each target's (`docs/avrdx/tcb.md`:
`FrequencyMeter`, `PulseWidthMeter`, `DutyMeter`). Nothing here
configures hardware or names a timer.

```
edge -> capture ISR -> MeterLatch::store()      as fast as the wire
tick -> MeterSampler -> publish(MeterSample)    as fast as the app
```

```cpp
using Period = MeterLatch<uint16_t, P, 0>;
using Width  = MeterLatch<uint16_t, P, 1>;
using Meters = MeterSampler<P, Subscribers<Display, Governor>, Period, Width>;

ISR(TCB0_INT_vect) { Period::store(FrequencyMeter<Tcb<0>>::period_ticks()); }
ISR(TCB1_INT_vect) { Width::store(PulseWidthMeter<Tcb<1>>::width_ticks()); }
...
Meters::init(ticks_from_ms<P>(100));      // publish ten times a second
```

## Why a latch and not a queue

What a periodic reading is worth is its LAST value; the ones before it
are history nobody asked for. A single cell costs one value and one
flag, cannot overflow, and always holds the most recent measurement. A
queue would offer a backlog of stale numbers and a hard question about
what to do when it fills.

The overwrites are not silent: `missed()` counts every `store()` that
landed on a value nobody had taken - which is exactly "the signal is
faster than my pace". That is a diagnostic, not an error; a program
that wants every edge wants a counter in hardware, not a queue in RAM.

- **`store(v)` is the ISR half** and runs under the platform's critical
  section, so a main-context `take()` never reads a half-written wide
  value on a machine whose store is not atomic (every 8-bit one).
- **`take()` is read-and-clear**: the value if one arrived since the
  last call, `nullopt` otherwise.
- **The type is the object**, as it is for an AO: the ISR glue names a
  latch with no pointer to plumb. Two latches of the same width are
  told apart by their `id` template argument.
- **The drivers stay untouched.** A meter exposes an ISR handler BODY
  that returns a reading and re-arms the capture; the application's
  vector binding is what joins it to a latch - the same division the
  kernel makes everywhere between a driver's body and an app's
  binding.

## Why the sampler publishes only fresh values

A subscriber that receives an event learns a FACT: this meter read
this. A repeat of the previous reading is not a fact about the signal,
it is a fact about the sampler having nothing to say - and once the two
look identical the subscriber cannot tell them apart. So **a stale
source publishes nothing**, and silence carries the information: no
edges arrived.

A subscriber that must act on that silence (a stall detector, a
watchdog) times it with its own time event, which is the only place
where "how long is too long" is known.

- **The AO paces PUBLICATION, not capture.** The capture rate is the
  wire's and the interrupt's; the period given to `init()` is the
  program's. An event stream at the capture rate would flood every
  subscriber's queue for no gain.
- **`MeterSample{index, value}`** carries the source's position in the
  pack and a 32-bit value; a narrower source widens. What the number
  MEANS - ticks, edges, a period to be turned into Hz - is the
  subscriber's business, exactly as raw counts are in
  [analog.md](analog.md).
- **A source is anything with a `take()`** (`MeterSource`). A latch is
  the usual one; a peripheral with a readable register satisfies the
  concept with a three-line adapter and no interrupt at all.
- **`init(period)` is defaulted to no pace** because the kernel's AO
  contract calls `init()` with no arguments: `Kernel::init_all()`
  leaves the sampler quiet and the application arms it right after,
  where it configures the hardware the sources read.
- **`missed(index)`** passes a source's own overwrite count through for
  diagnostics, and answers 0 for a source that keeps none.

## What is deliberately not here

- **Any arithmetic on the value.** Turning a period in ticks into Hz is
  the meter driver's (`FrequencyMeter::hz()`) or the subscriber's; a
  sampler that divided would need to know the timer's clock, which is
  precisely the thing it must not know.
- **Averaging, medians, rate limiting.** A subscriber that wants a
  smoothed number smooths it: what to do with outliers depends on what
  the number is for, and a filter in the sampler would impose one
  answer on every subscriber.
- **A stall timeout.** See above: silence is the signal, and the
  deadline belongs to whoever has one.

## Reference

| Entity | Header | Role |
|--------|--------|------|
| `MeterLatch<T, P, id>` | `util/meter_sampler.hpp` | the ISR bridge: `store`, `take`, `missed`, `fresh`, `clear` |
| `MeterSource` (concept) | `util/meter_sampler.hpp` | what the sampler needs of a source: a read-and-clear `take()` |
| `MeterSampler<P, Subscribers<...>, Sources...>` | `util/meter_sampler.hpp` | the owner AO: `init(period)`, `start_every`, `stop`, `published()`, `missed(i)`, `queue` |
| `MeterSample{index, value}` | `util/meter_sampler.hpp` | published to the subscribers |
| `FrequencyMeter`, `PulseWidthMeter`, `DutyMeter` | each target's timer header | the silicon that produces the readings |

Target page: [../avrdx/tcb.md](../avrdx/tcb.md).
