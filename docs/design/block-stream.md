# Block streams

This page decides the vocabulary of data that moves in caller-owned
buffers at a rate the CPU never touches per sample (`util/block_stream.hpp`):
the two concepts a capture source and a playback player meet, and
`BlockRelay`, the active object that lends each filled buffer to its
subscribers for exactly one dispatch. It is written for an embedded
programmer who has captured a burst of samples by DMA into a pair of
buffers and wants the hand-over to the consumer to be correct by
construction. The contract is about blocks, not about DMA; on a
machine with no DMA an interrupt handler filling the same buffers
satisfies it.

The smallest use: a family's block source (a `DmaPingPongEngine` over
a converter), a `BlockRelay<P, Subscribers<...>, Source>` in Tenuto's
list, the engine's completion posted to the relay as `BlockDone`, and
a subscriber handling `BlockReady<T>`. The prerequisite that decides
correctness: **the block is a `Lease::dispatch` loan**, valid during
the subscriber's dispatch only, so the subscriber must precede the
relay in Tenuto's list (Tenuto refuses the other order) and must not
keep the pointer.

## Why a fixed point

Waveform playback and burst capture are generic applications, not one
project's need, and every platform brings its own DMA with its own
shapes. The contract is the fixed point those shapes are measured
against: the friction a controller meets with these concepts is the
measurement, and "this concept does not fit" is a finding instead of a
silent divergence. If the contract has to move, it moves in the open.

## The two concepts

The contract is about blocks, not about DMA. Nothing below asks how a
buffer gets full or drained; on every stratum that has block engines
both concepts are satisfied by DMA engines, and a machine with no DMA
can satisfy them from an interrupt handler filling the same buffers.

- **`BlockSource`**, the capture shape: `element` (the sample type),
  `ready()` (the filled block, or null; a pointer to volatile, because
  whatever filled it is invisible to the compiler), `ready_length()`,
  `release()` (hand the buffer back; restarts a stalled stream), and
  the accounting: `laps()`, `overruns()`, `stalled()`. The accounting
  is the API: a consumer that falls behind cannot be made correct by
  cleverness, so the only honest design is one that says exactly what
  was traded. An overrun means the source skipped a block rather than
  tear one; how many samples that cost is the underlying peripheral's
  to report, never the source's to invent.
- **`BlockPlayer`**, the playback shape: `element`, `laps()`,
  `faults()`, `running()`, `stop()`. No active object consumes it: a
  player is paced by its peripheral's own request, publishes nothing
  per lap, and `laps()` moving is the one fact that says the stream is
  alive. The concept exists so the next platform's playback has a
  contract to meet, exactly as `BlockSource` does for capture.

### Realizations

Common to all: the two concepts, `BlockRelay`, and the engine names;
every stratum that has block engines spells all four identically
(`DmaTxEngine`, `DmaRxEngine`, `DmaLoopEngine`, `DmaPingPongEngine`),
with the element type as the beat (the unit one transfer moves, the
engines' word) and the same
`kick`/`abandon`/`faults`/`harvest` hardening. What differs is what
the controller under them can do.

| stratum | realization | beyond the contract |
|---|---|---|
| avrxt | none | this family has no DMA; an interrupt-fed source satisfying `BlockSource` from a handler is the stated shape and is born with its first user |
| samc21 | none | the DMAC is driven for the UART's transmitter alone: erratum 1.10.4 corrupts the live descriptor of concurrently triggered channels ([the SAM C21 DMAC page](../samc21/dmac.md)); an interrupt-fed source is the stated shape and is born with its first user |
| stm32g0 | the same two names in `stm32g0/dma.hpp` | the player rides the controller's hardware circular mode (the lap interrupt only counts); the source cannot (skip-rather-tear is undecidable after the edge, below) and stops itself at every block |
| stm32f4 | none yet: `DmaTxEngine` and `DmaRxEngine` alone in `stm32f4/dma.hpp` | this controller's double buffer is a better `BlockSource` than either shape above (the hardware swaps the memory pointer at every end of transaction, so there is no re-arm window and no race between a handler and a running stream, and CT names the half the caller may hold) while the player is plain circular mode as on the G0; born with their first block user, and the re-arm gap the transfer engines pay for meanwhile is measured ([the STM32F4 DMA page](../stm32f4/dma.md)) |
| ch32v00x | none yet: `DmaTxEngine` and `DmaRxEngine` alone in `ch32v00x/dma.hpp` | the controller has a circular mode (CFGR.CIRC) and no multiplexer (the channel is the request), so the two block engines would be the G0's shape on seven fixed channels; born with their first block user on this family, a source the ADC's stall makes worth measuring first ([the CH32V00x DMA page](../ch32v00x/dma.md)) |
| ch32vx03 | the same two names in `ch32vx03/dma.hpp`, each naming its controller first (`<c, ch, Elem>`) | the player rides the controller's hardware circular mode as on the STM32G0, and on the CH32V303 it has the peripheral it was written for, the DAC, played for ever from DMA2's channel 3 and read back level by level; the source does not, for the same rule, and this stratum measured it again (a handler whose whole body is read-CNTR-and-disable found two or three of the next thirty-two items already written at the controller's own speed), so the source stops itself at every block and re-arms the other buffer from the completion. One family fact holds above both: in Sleep no bus master but the core gets a cycle, on either of the CH32V303's controllers, so a stream here holds the program awake or it stops ([the CH32V203/CH32V303 ADC page](../ch32vx03/adc.md), whose converter is these engines' first user) |
| ch32x035 | none yet: `DmaTxEngine` and `DmaRxEngine` alone in `ch32x035/dma.hpp` | eight channels where the channel is the request and a circular mode (CFGR.CIRC), the CH32V203's controller, so the two block engines would be that stratum's shape; in the Sleep the controller moves nothing here too, so a stream holds the program awake; born with their first block user, the ADC, which has no driver in this stratum ([the CH32X035 DMA page](../ch32x035/dma.md)) |
| rp2040 | none yet: `DmaTxEngine` and `DmaRxEngine` alone in `rp2040/dma.hpp` | this controller reaches the two shapes by a road none of the others has: a channel may chain to another at completion, and an address may wrap at a power-of-two boundary, so a player is a pair of channels chained to each other, or one channel with a ring on its read side, and a ping-pong source is two channels chained the same way with no handler in the loop at all; born with their first block user on this family |
| rp2350 | none yet: the same two engine names in `rp2350/dma.hpp` | the RP2040's chaining and its address wrap, plus a mode in the top nibble of the transfer count that makes a channel re-arm itself or run forever, which is the first hardware circular mode on this lineage and would carry a player with no second channel and no handler; two of the chapter's errata bear on exactly that road (a chain fired by an abort, a chain lost after a zero-length transfer) and are answered in the driver, so a block user here starts from the engines and not from the controller |
| host | a scripted ping-pong source (`test_block_stream`) | honest to the contract (an overrun skips the lap, a release restarts), so the relay's loan timing and stall drain are tested to the dispatch |

## BlockRelay

`BlockRelay<P, Subs, Sources...>` is the active object between a
`BlockSource` and its subscribers. Three positions, each the opposite
of `MeterSampler`'s ([meters.md](meters.md)) for a stated reason:

- **Event-driven, not paced.** `MeterSampler` exists to discard: a
  meter's latest value is the only one worth publishing. A block
  stream is the opposite economy: every block must reach its consumer
  exactly once, because the samples inside it exist nowhere else. So
  the source's completion is the wakeup (the interrupt glue posts
  `BlockDone` after the engine's own completion function) and the
  source's two buffers are the slack that absorbs dispatch latency.
- **The block travels as a `Lease::dispatch` loan.** A block is too
  big to copy through a queue; `BlockReady<T>` carries
  `Borrowed<const volatile T, Lease::dispatch>`, valid during the
  receiving dispatch only. The relay declares `LendsTo = Subs` and the
  kernel refuses a list where a borrower does not precede it, which
  makes the release timing correct by construction: when a relay
  dispatch starts, the kernel has already served every borrower, so
  every block lent in the previous dispatch has been consumed, and
  returning those loans is the first thing the new dispatch does
  (`LineReader`'s two-buffer contract, generalized). Between lend and
  release the source fills its other buffer, so the consumer reads
  memory nothing is writing.
- **A dispatch scans every source and lends at most one block per
  source**, then re-posts itself if it lent anything. Scanning
  everything makes a coalesced or dropped wakeup delay work, never
  lose it (the queue's overflow is harmless by construction and still
  counted). One block per source because the second block a stall can
  hold needs this dispatch's loan returned first; the self-post is
  what comes back for it, and it is also what guarantees the returning
  dispatch when the stream has gone quiet: a stalled source emits no
  further completions, and the self-posted dispatch's `release()` is
  what restarts it.

The relay configures no hardware and starts no stream; it does not
know any. It keeps no accounting of its own beside `published()`:
`laps(i)` and `overruns(i)` are the source's numbers passed through,
because a second truth beside the source's would have to be kept in
step and would say nothing new.

## What is deliberately absent

- **A playback active object.** Nothing to pace, nothing to publish;
  an owner that wants laps as events arms its own `TimeEvent`.
- **A one-shot burst vocabulary.** A finite capture is a source
  started and stopped, or (below `util/`) a bare engine block taken
  once; it earns a concept when a second user shapes one.
- **A restart and gap policy.** `release()` restarting a stalled source
  is the source's contract; whether the gap warrants an application
  response is the subscriber's business, told by the overrun count.
- **An AVR implementation, and the two halves part ways there.**
  `BlockSource` is the plausible half: an ADC interrupt filling
  ping-pong buffers at a few thousand samples a second is realistic,
  the concept was designed to allow it, and it is born with its first
  user. `BlockPlayer` is not: an interrupt reloading a DAC costs
  microseconds per sample on that core, and, more to the point, a
  waveform is not that family's shape at all. There a waveform is
  hardware (the timers' PWM, already a `PwmChannel`) and the DAC serves
  levels; a degraded interrupt player would be an emulation of the
  wrong platform, and a concept is not a hardware abstraction layer: a
  target that cannot meet it implements nothing, and a program that
  names it there fails to compile, which is the honest answer.
- **A sequencer.** A table of levels walked at a slow pace (an
  envelope, a profile, a staircase), which is what a machine with no
  stream machinery does instead of playback. It is a different
  vocabulary, not a degraded player: a `TimeEvent` plus a table plus an
  actuator the concepts already name, pure `util/` over existing
  contracts, portable by construction. That is exactly why it is not
  built early: the block-stream contract was, because every platform
  implements it with its own machinery and the fixed point is what
  measures the friction; a sequencer has no per-target half to
  measure, so pre-building buys nothing and it is born with its first
  user.

## Validation

Host: `test_block_stream` (a scripted ping-pong source honest to the
contract, where an overrun skips the lap and a release restarts; loan
timing, stall drain, coalesced wakeups, accounting pass-through).

The STM32G0's `stm32g0/dma.hpp`, over a controller with a hardware
circular mode, is where the fixed point earned its keep. `BlockPlayer`
fits the circular channel exactly and gains by it (the lap interrupt
only counts; there is no re-arm window). `BlockSource` does not fit
it, and the reason is this rule and not the API: a circular channel
never stops, so "skip rather than tear" can only be decided after the
edge, by which time the controller is already writing the half the
caller holds (measured: 0 to 6 elements had landed before a handler
whose whole body was disable-and-read could act). So that engine stops
itself at every block, and nothing above it changed: `test_stm32_dma`
runs the relay over a timer-paced capture stream in a real kernel, and
the family fixture concept-checks both engines. The contract is about
blocks and not about DMA, proven the hard way: the controller's
natural streaming mode is the one the contract cannot use for a
source.

The CH32V203's and the CH32V303's (two controllers on the CH32V303),
with the same circular mode, where the source's refusal of it was
measured a second time (three items of the next half already written
by the time the channel's own handler could disable it, two or three
on the CH32V303), and `test_vx03_adc`'s letter d runs the relay over a
live four-channel capture stream in a real kernel on both parts. The
CH32V303 gave the player the peripheral it was written for:
`test_vx03_dac` plays a table into the DAC for ever from DMA2's third
channel, paced by a timer, and finds it on the pad level by level.
