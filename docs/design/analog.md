# Analog in the kernel

Two target-independent pieces sit between an analog converter driver
and the active objects that want its numbers: the arithmetic
(`util/analog.hpp`: counts <-> millivolts, DAC codes - pure,
host-tested) and the **AnalogSampler** (`util/analog_sampler.hpp`): the
active object that owns one converter, walks a fixed list of inputs
and publishes every result as a value event. The converter itself, its
configuration knobs and its reference levels are each target's
(`docs/avrdx/adc.md`, `vref.md`, `dac.md`).

## The sampler is a usage type, not an application

What it knows: "a list of inputs, a pace, samples out". What it does
not know: what the samples mean (the subscribers scale them with
`adc_mv` and their reference), where the pace comes from, which
silicon converts. It is the analog sibling of `SerialPort<Transport>`
and `BusMaster<Bus>`: a util AO template over a concept the target
driver satisfies.

```cpp
using Sampler = AnalogSampler<Adc<0>, P, Subscribers<Monitor, Alarm>,
                              AnalogIn<Pin<'D', 1>>{}, AdcInput::temp, AdcInput::vdd_div10>;
ISR(ADC0_RESRDY_vect) { post<Sampler>(Sampled{Adc<0>::resrdy(), Adc<0>::selected()}); }
```

- **Converter contract** (`AnalogConverter` + `SamplerInput` per
  input): `start()`, `selected()` (the input code in effect),
  `select(input)` and a constexpr `input_code(input)` for each input
  of the list. The inputs are values of the driver's own vocabulary
  given as template arguments - pin tags, internal sources - so
  selection is an overload resolved at compile time and an input the
  converter cannot take is a compile error.
- **Result path** = the kernel's ISR contract: the driver's
  result-ready ISR body returns the value, the app's vector binding
  posts `Sampled{value, input}`; the sampler publishes
  `AnalogSample{index, value}` (index = position in the list) to the
  subscribers, by value, and selects the next input.
- **Attribution by the reported code, never by timing.** `Sampled`
  carries the input code the converter had selected for that
  conversion, read in the ISR with the value. A late dispatch, a pace
  faster than the dispatch, a queue overflow can delay or lose
  samples - they cannot mislabel them. A code outside the list is
  dropped and counted.
- **Two paces, one sampler.** Software: `start_every(ticks)` (a
  TimeEvent of the sampler; kernel-tick granularity). Hardware: the
  application routes ANY event generator - a PIT divider, a timer
  overflow, a pin - to the converter's start input and never calls
  `start_every`; the sampler only receives results. Which generator
  is the app's choice, made where the hardware is wired: the sampler
  does not name the timer.
- **The owner's duties** stay with the application: configuring the
  converter before the sampler's `init()` (reference, resolution,
  accumulation - the sampler does not reconfigure, and the
  subscribers must know the meaning of what they get), pausing a
  hardware pace across a dynamic clock change ([clock.md](clock.md)),
  never selecting an input behind the sampler's back.
- **The queue is the ammortizer, and says when it is too small.**
  Results arrive from the ISR at the pace; a long dispatch elsewhere
  queues a few; the queue's saturating overflow counter is the
  measurement (bench: 512 samples/s, two subscribers, no drops on AVR
  DA/DB at 24 MHz).

## What is deliberately not here

- A multi-consumer arbiter (requests `{input, ReplyTo<...>}` served
  one conversion at a time): the shape of `BusMaster`, with the
  difference that the reply must carry the value. Built when an
  application has two AOs wanting channels with DIFFERENT converter
  configurations; until then one owner AO and `publish` cover the
  need. Note that an arbiter and a hardware pace exclude each other.
- A threshold watcher (the converter's window comparator posting an
  alarm with no sample traffic, the CPU asleep): a second usage
  type, separate from the sampler - the window is agnostic of the
  selected input, so it does not belong inside a walk.
- A sequencer in util: on silicon with a hardware scan (a sequencer
  and DMA) the walk moves behind the concept, the interface - list,
  pace, samples - does not change. Until that target exists, the walk
  is the sampler's and nothing pretends otherwise.

## Reference

| Entity | Header | Role |
|--------|--------|------|
| `adc_mv`, `adc_mv_signed`, `dac_code`, `dac_mv` | `util/analog.hpp` | counts <-> millivolts for any converter |
| `AnalogConverter`, `SamplerInput` (concepts) | `util/analog_sampler.hpp` | what the sampler needs of a converter and of each input |
| `AnalogSampler<C, P, Subscribers<...>, inputs...>` | `util/analog_sampler.hpp` | the owner AO: `init`, `start_every(ticks)`, `stop`, `unknown_inputs()`, `queue` |
| `Sampled{value, input}` | `util/analog_sampler.hpp` | posted by the ISR glue |
| `AnalogSample{index, value}` | `util/analog_sampler.hpp` | published to the subscribers |
| `Ref`, `ref_mv` | each target's vref header | this silicon's reference levels |

Target pages: [../avrdx/adc.md](../avrdx/adc.md), [../avrdx/vref.md](../avrdx/vref.md), [../avrdx/dac.md](../avrdx/dac.md).
