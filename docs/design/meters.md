# Meters in the kernel

This page decides how a measurement taken on every edge of a signal
reaches active objects at the program's own pace: `MeterLatch`
(`util/meter_sampler.hpp`), the one-cell bridge out of the capture
interrupt that keeps the latest reading and counts the ones it
overwrote, and `MeterSampler` (the same header), the active object that
walks a fixed list of such sources on its own period and publishes
what is fresh. It is written for an embedded programmer who has
measured a frequency with a timer's capture interrupt and found the
interrupt rate dictating the program's. The meters themselves are each
target's; nothing here configures hardware or names a timer.

A hardware meter (a timer in a capture mode, an encoder counter, a
frequency divider with a readable register) produces a number per
edge of the signal it watches. The rate of those numbers is the
signal's, not the program's: a 10 kHz input is ten thousand readings a
second, each one an interrupt. The two pieces here separate the two
paces.

The smallest use is the excerpt below. The prerequisite that decides
correctness: **a stale source publishes nothing**, so a subscriber
that must act on silence (a stall detector) times it with its own time
event; the sampler will not repeat a reading.

### Realizations

Common to all: the `MeterSource` concept is satisfied by the
`MeterLatch` itself, and `MeterSampler` walks latches, so where a
target has meters at all they are of one shape: a task over a timer
in a capture mode whose reading functions a capture handler calls to
fill the latch (`period_ticks()`, `width_ticks()`). What differs is
which timers offer which capture, one meter that measures a different
thing under a similar name, and whether the silicon has a capture unit
in the first place.

| stratum | realization | beyond the contract |
|---|---|---|
| avrxt | `FrequencyMeter<Tcb>`, `PulseWidthMeter<Tcb>`, `DutyMeter<Tcb>` (`avrxt/tcb.hpp`) | one TCB per meter (frequency, pulse width, or both as one `reading()` of period then width); the tasks are `ClockUser`s, their `hz()`/`us()` conversions following a rebase |
| samc21 | `TcPeriodMeter<Tc>` (period and width, EVACT = PPW), `TcPulseWidthMeter<Tc>` (the high time alone, EVACT = PW) (`samc21/tc.hpp`) | the input reaches the capture through the event system (an EIC line or a CCL LUT as generator), never a pad directly |
| stm32g0 | `TimPeriodMeter<Tim>` (period and width, PWM input mode: a slave controller and two channels) and `TimIntervalMeter<Tim, ch>` (`stm32g0/tim.hpp`) | `TimIntervalMeter` is not a pulse-width meter: it measures the interval between consecutive edges on one channel of a free-running counter, by subtraction, which is what a single-channel timer can offer (`interval()`, one value of state, for the capture handler alone) |
| stm32f4 | `TimPeriodMeter<Tim>` (period and width, PWM input mode: a slave controller and two channels) and `TimIntervalMeter<Tim, ch>` (`stm32f4/tim.hpp`) | the G0's two names again, `TimIntervalMeter` the same interval-between-edges meter; what this family adds is a capture input that needs no pad at all: TIM5's option register puts the LSI, the LSE or the RTC wake-up on its channel 4 and TIM11's puts HSE_RTC on its channel 1, so an oscillator is weighed against the core clock with nothing attached ([the STM32F4 timer page](../stm32f4/tim.md)) |
| ch32v00x | `TimPeriodMeter<Tim>` (period and width, the PWM input mode on TI1 costing both channels) and `TimIntervalMeter<Tim, ch>` (`ch32v00x/tim.hpp`) | the G0's two names on the F1's timers, `TimIntervalMeter` the same interval-between-edges meter and not a pulse-width one; the capture can carry the captured level in bit 16 of the channel register (CAPLVL) and reads 0xFFFF after an overflow (CAPOV), two facts the F1 never had ([the CH32V00x timer page](../ch32v00x/tim.md)) |
| ch32vx03 | `TimPeriodMeter<Tim>` (period and width, PWM input mode on TI1, both channels) and `TimIntervalMeter<Tim, ch>` (`ch32vx03/tim.hpp`) | the G0's two names on the F1's timers again, with none of the sister family's additions (no CAPLVL, no CAPOV, no dual-edge capture on the CH32V203: that register is the CH32V30x_D8's, and a lot's, since `dual_edge_capture()` asks the die and the CH32V303VCT6 measured had none); what this family offers a meter with nothing attached is a pad the port can drive, the alternate-function input being the pad's own input buffer, so the CPU makes the edges a capture measures ([the CH32V203/CH32V303 timer page](../ch32vx03/tim.md)) |
| ch32x035 | none | TIM1, TIM2 and TIM3 (RM ch. 12, 13) are not written in this stratum; TIM2 here is an advanced timer with four vectors |
| rp2040 | none | this silicon has no capture unit: the PWM block's counter can be clocked or gated by its own B pin but nothing latches it on an edge, so there is nothing for a capture handler to read and a `MeterSource` over it would be a fiction. What this family offers instead is `PwmEdgeCounter` / `PwmLevelCounter`, a frequency and a duty obtained by counting over a known window and read by the loop, not by the wire's own edges ([the RP2040 PWM page](../rp2040/pwm.md)) |
| rp2350 | none | the same absence for the same reason, over twelve slices instead of eight ([the RP2350 PWM page](../rp2350/pwm.md)) |
| host | a scripted latch (`test_meter_sampler`) | the sampler's discard-stale and labelling tested to the tick |

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

What a periodic reading is worth is its last value; the ones before it
are history nobody asked for. A single cell costs one value and one
flag, cannot overflow, and always holds the most recent measurement. A
queue would offer a backlog of stale numbers and a hard question about
what to do when it fills.

The overwrites are not silent: `missed()` counts every `store()` that
landed on a value nobody had taken, which is exactly "the signal is
faster than my pace". That is a diagnostic, not an error; a program
that wants every edge wants a counter in hardware, not a queue in RAM.

- **`store(v)` is the interrupt half** and runs under the platform's
  critical section, so a main-context `take()` never reads a
  half-written wide value on a machine whose store is not atomic
  (every 8-bit one).
- **`take()` is read-and-clear**: the value if one arrived since the
  last call, `nullopt` otherwise.
- **The type is the object**, as it is for an active object: the
  interrupt glue names a latch with no pointer to plumb. Two latches
  of the same width are told apart by their `id` template argument.
- **The drivers stay untouched.** A meter exposes an interrupt handler
  body that returns a reading and re-arms the capture; the
  application's vector binding is what joins it to a latch, the same
  division the kernel makes everywhere between a driver's body and an
  application's binding.

## Why the sampler publishes only fresh values

A subscriber that receives an event learns a fact: this meter read
this. A repeat of the previous reading is not a fact about the signal;
it is a fact about the sampler having nothing to say, and once the two
look identical the subscriber cannot tell them apart. So **a stale
source publishes nothing**, and silence carries the information: no
edges arrived.

A subscriber that must act on that silence (a stall detector, a
watchdog) times it with its own time event, which is the only place
where "how long is too long" is known.

- **The active object paces publication, not capture.** The capture
  rate is the wire's and the interrupt's; the period given to `init()`
  is the program's. An event stream at the capture rate would flood
  every subscriber's queue for no gain.
- **`MeterSample{index, value}`** carries the source's position in the
  list and a 32-bit value; a narrower source widens. What the number
  means (ticks, edges, a period to be turned into hertz) is the
  subscriber's business, exactly as raw counts are in
  [analog.md](analog.md).
- **A source is anything with a `take()`** (`MeterSource`). A latch is
  the usual one; a peripheral with a readable register satisfies the
  concept with a three-line adapter and no interrupt at all.
- **`init(period)` defaults to no pace** because the kernel's
  active-object contract calls `init()` with no arguments:
  `Tenuto::init_all()` leaves the sampler quiet and the application
  arms it right after, where it configures the hardware the sources
  read.
- **`missed(index)`** passes a source's own overwrite count through for
  diagnostics, and answers 0 for a source that keeps none.

## What is deliberately not here

- **Any arithmetic on the value.** Turning a period in ticks into
  hertz is the meter driver's (`FrequencyMeter::hz()`) or the
  subscriber's; a sampler that divided would need to know the timer's
  clock, which is precisely the thing it must not know.
- **Averaging, medians, rate limiting.** A subscriber that wants a
  smoothed number smooths it: what to do with outliers depends on what
  the number is for, and a filter in the sampler would impose one
  answer on every subscriber.
- **A stall timeout.** As above: silence is the signal, and the
  deadline belongs to whoever has one.

## Reference

| Entity | Header | Role |
|--------|--------|------|
| `MeterLatch<T, P, id>` | `util/meter_sampler.hpp` | the interrupt bridge: `store`, `take`, `missed`, `fresh`, `clear` |
| `MeterSource` (concept) | `util/meter_sampler.hpp` | what the sampler needs of a source: a read-and-clear `take()` |
| `MeterSampler<P, Subscribers<...>, Sources...>` | `util/meter_sampler.hpp` | the owner active object: `init(period)`, `start_every`, `stop`, `published()`, `missed(i)`, `queue` |
| `MeterSample{index, value}` | `util/meter_sampler.hpp` | published to the subscribers |
| `FrequencyMeter`, `PulseWidthMeter`, `DutyMeter` | each target's timer header | the silicon that produces the readings |

Target page: [the AVR TCB page](../avrxt/tcb.md).
