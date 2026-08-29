# Block streams

`util/block_stream.hpp`. The vocabulary of data that moves in
caller-owned buffers at a rate the CPU never touches per sample, and the
active object that hands filled buffers to subscribers.

## Why this exists before its second implementation

Every other util contract earned its shape the same way: built against
one target, then validated by a second silicon implementing it untouched
(the ring, the clock contracts, `FlashMedia`, `SleepSite`, `PwmChannel`,
`MeterSource`, the analog pair). Block streams invert the order on
purpose. Waveform playback and burst capture are generic applications -
not one project's need - and the next platform (an STM32G0 is the named
candidate) will bring its own DMA with its own shapes. Fixing the
contract NOW, against the SAM C21's engines, plants the reference point:
when that platform's stream machinery is built, the friction against
these concepts is the measurement, and "this concept does not fit" is a
finding instead of a silent divergence. If the contract has to move
then, it moves in the open.

## The two concepts

The contract is about BLOCKS, not about DMA. Nothing below asks how a
buffer gets full or drained; on the SAM C21 both concepts are satisfied
by DMA engines (`samc/dmac.hpp`), and a machine with no DMA can satisfy
them from an interrupt handler filling the same buffers.

- **`BlockSource`** - the capture shape: `element` (the sample type),
  `ready()` (the filled block, or null - pointer-to-volatile, because
  whatever filled it is invisible to the compiler), `ready_length()`,
  `release()` (hand the buffer back; restarts a stalled stream), and the
  accounting - `laps()`, `overruns()`, `stalled()`. The accounting IS
  the API: a consumer that falls behind cannot be made correct by
  cleverness, so the only honest design is one that says exactly what
  was traded. An overrun means the source SKIPPED a block rather than
  tear one; how many samples that cost is the underlying peripheral's to
  report, never the source's to invent.

- **`BlockPlayer`** - the playback shape: `element`, `laps()`,
  `faults()`, `running()`, `stop()`. No AO consumes it: a player is
  paced by its peripheral's own request, publishes nothing per lap, and
  `laps()` moving is the one fact that says the stream is alive. The
  concept exists so the next platform's playback has a contract to meet,
  exactly as `BlockSource` does for capture.

## BlockRelay

`BlockRelay<P, Subs, Sources...>` is the AO between a `BlockSource` and
its subscribers. Three positions, each the opposite of `MeterSampler`'s
for a stated reason:

- **Event-driven, not paced.** `MeterSampler` exists to DISCARD - a
  meter's latest value is the only one worth publishing. A block stream
  is the opposite economy: every block must reach its consumer exactly
  once, because the samples inside it exist nowhere else. So the
  source's completion is the wakeup (the ISR glue posts `BlockDone`
  after the engine's own completion verb) and the source's two buffers
  are the slack that absorbs dispatch latency.

- **The block travels as a `Lease::dispatch` loan.** A block is too big
  to copy through a queue; `BlockReady<T>` carries
  `Borrowed<const volatile T, Lease::dispatch>`, valid during the
  receiving dispatch only. The relay declares `LendsTo = Subs` and the
  kernel refuses a pack where a borrower does not precede it - which
  makes the release timing correct BY CONSTRUCTION: when a relay
  dispatch starts, the kernel has already served every borrower, so
  every block lent in the PREVIOUS dispatch has been consumed, and
  returning those loans is the first thing the new dispatch does
  (`SerialPort`'s two-buffer contract, generalized). Between lend and
  release the source fills its other buffer, so the consumer reads
  memory nothing is writing.

- **A dispatch scans every source and lends at most one block per
  source**, then re-posts itself if it lent anything. Scanning
  everything makes a coalesced or dropped wakeup delay work, never lose
  it (the queue's overflow is harmless by construction and still
  counted). One block per source because the second block a stall can
  hold needs this dispatch's loan returned first - the self-post is
  what comes back for it, and it is also what guarantees the returning
  dispatch when the stream has gone quiet: a stalled source emits no
  further completions, and the self-posted dispatch's `release()` is
  what restarts it.

The relay configures no hardware and starts no stream - it does not
know any. It keeps no accounting of its own beside `published()`:
`laps(i)` and `overruns(i)` are the source's numbers passed through,
because a second truth beside the source's would have to be kept in
step and would say nothing new.

## What is deliberately absent

- **A playback AO** - nothing to pace, nothing to publish; an owner
  that wants laps as events arms its own TimeEvent.
- **One-shot burst vocabulary** - a finite capture is a source started
  and stopped, or (below util) a bare engine block taken once; it earns
  a concept when a second user shapes one.
- **Restart and gap policy** - `release()` restarting a stalled source
  is the source's contract; whether the gap warrants an app response is
  the subscriber's business, told by the overrun count.
- **An AVR implementation, and the two halves part ways there.**
  `BlockSource` is the plausible half: an ADC interrupt filling
  ping-pong buffers at a few ksps is realistic, the concept was
  designed to allow it, and it is born with its first user.
  `BlockPlayer` is NOT: an interrupt reloading a DAC costs microseconds
  per sample on that core, and - more to the point - a waveform is not
  that family's shape at all. There a waveform is hardware (the timers'
  PWM, already `PwmChannel`) and the DAC serves LEVELS; a degraded
  interrupt player would be an emulation of the wrong platform, and a
  concept is not a HAL - a target that cannot meet it implements
  nothing, and a program that names it there fails to compile, which is
  the honest answer.
- **A sequencer** - a table of LEVELS walked at a slow pace (an
  envelope, a profile, a staircase), which is what a machine with no
  stream machinery does instead of playback. It is a different
  vocabulary, not a degraded player: a TimeEvent plus a table plus an
  actuator the concepts already name, i.e. pure util over existing
  contracts, portable by construction. That is exactly why it is NOT
  built early: the block-stream contract was, because every platform
  implements it with its own machinery and the fixed point is what
  measures the friction - a sequencer has no per-target half to
  measure, so pre-building buys nothing and it is born with its first
  user.

## Validation

Host: `test_block_stream` (a scripted ping-pong source honest to the
SAM C21 engine's contract - overrun skips the lap, release restarts;
loan timing, stall drain, coalesced wakeups, accounting pass-through).
Silicon: `test_samc_analog_dma`'s kernel letter runs the relay over the
live DAC-to-ADC chain with `DmaPingPongEngine` as the source; the
family fixture concept-checks both SAM engines against both concepts.
