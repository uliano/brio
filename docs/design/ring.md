# Ring: the SPSC FIFO

`util/ring.hpp` - `Ring<T, size, P>`; `GapRing<T, size, P>`, a Ring
whose producer marks where it lost elements ([its own section](#gapring-a-ring-that-says-where-it-lost));
and `HardwareRing<storage, Counter>`, the consumer half of a ring whose
producer is the hardware ([its own section](#hardwarering-the-ring-whose-producer-is-the-hardware)).

## What it is for

A bounded FIFO between exactly two parties: one producer, one
consumer. Driver byte rings (UART), sample
buffers, logs - anything that needs a queue but not the MPSC
semantics of the kernel `EventQueue`. It is a general tool, not a
driver detail: whoever needs a two-party FIFO names the platform and
uses it, in `avrdx/`, in `util/`, in an app, in a host test.

## Why no interrupt masking

The tempting implementation wraps every main-side operation in
`ATOMIC_BLOCK` and offers `*_from_isr` twins for the ISR side. But an
SPSC ring with single-byte indices needs no interrupt masking at all
on AVR: the producer only writes `head_`, the consumer only writes
`tail_`, each reads the other's index with one atomic `lds`, and a
stale read errs on the safe side (the producer underestimates room,
the consumer underestimates data). The masking would be pure cost - a
few cycles per byte on the print path and, worse, added interrupt
latency on every `write_byte` - and the API doubling it justifies is
noise.

## The decision: the platform states the atomic width, Ring picks the path

Two candidates were rejected:

- `static_assert(size <= 256)` and drop the guard entirely - simplest,
  but a hard limit chosen for a case that does not exist today, on a
  target family where 32-bit indices will be the norm;
- keep the guard as `ATOMIC_BLOCK` behind `if constexpr` - keeps Ring
  in `avrdx/`, keeps `util/atomic.h`, keeps it host-untestable.

Adopted: Ring is templated on the Platform like every other brio
service, and the Platform concept gains one constant, `atomic_width`,
"the widest naturally aligned load/store the CPU performs as one
uninterruptible access, in bytes" (1 on AVR Dx, 4 on the 32-bit
candidates, 4 on the host). Ring derives its index type from `size`
(`uint8_t` up to 256 slots, `uint16_t` up to 65536, `uint32_t`
above - a 16-bit ceiling would be an AVR habit, RAM is the only real
limit) and then decides, with `if constexpr`:

- `sizeof(index_t) <= P::atomic_width` -> **lock-free**: indices are
  shared bare, ordering between slot copy and index publish enforced
  with `std::atomic_signal_fence` (compiler-only, free on single-core
  targets) plus a volatile read of the other side's index (never
  hoisted out of a drain loop);
- otherwise -> **guarded**: every operation runs inside
  `P::CriticalSection`, selected by a fact of the target instead of
  by hand.

So `Ring<uint8_t, 1024, AvrPlatform>` silently takes the guard and
`Ring<uint8_t, 1024, SamPlatform>` is lock-free, with the same source.
Nobody chooses; the platform states a truth, generic code draws the
consequence. This is the pattern the generalization rule asks for: no
`#ifdef`, no per-use knob, no hidden default (a default platform in
Ring would smuggle an AVR include into `util/`).

### Realizations

The ring is one header on every stratum; the only thing that varies is
the one constant it reads and therefore the path a given size takes.

| stratum | `atomic_width` | the path a ring takes |
|---|---|---|
| avrdx | 1 | lock-free up to 256 slots (an 8-bit index), guarded above - the console's 256-byte rings are lock-free, a 1024-slot one is not |
| samc21 | 4 | lock-free at every size this framework declares (a 32-bit index is one access) |
| stm32g0 | 4 | the same |
| ch32v00x | 4 | the same - a 32-bit index is one access on this core too |
| ch32vx03 | 4 | the same on the bigger QingKe cores, the V4B and the V4F alike (`ch32vx03/platform.hpp`), whose guard is a `csrrci` on mstatus.MIE |
| ch32x035 | 4 | the same on the QingKe V4C (`ch32x035/platform.hpp`), whose guard is a `csrrci` on mstatus.MIE |
| rp2040 | 4 | the same - but the guard is PRIMASK, which is PER CORE, so a ring shared BETWEEN the two cores is `util/inbox.hpp`'s and not this one ([kernel.md](kernel.md), section 12) |
| rp2350 | 4 | the same, whichever of this chip's two processor architectures the image is built for - the guard is PRIMASK on the Cortex-M33 half and `mstatus.MIE` on the Hazard3 one, and PER CORE in both spellings, so a ring shared BETWEEN the cores is `util/inbox.hpp`'s and not this one |
| stm32f4 | 4 | the same |
| host | 4, and a second test platform stating 1 | both paths run under the same suite (`test_ring`), the guarded one on the platform that states 1 |

The extra template parameter is the honest price, and it is the same
price `EventQueue`, `SerialPort`, `SpiBus` and `Tenuto` already pay:
the app names its platform once (`using P = AvrPlatform;`) and every
service reads it from there.

### What a verb compiles to

Each verb picks its path with `if constexpr` and runs its body in
place: the lock-free verb IS its body - its own index loaded, the other
side's read fresh, a compare, the slot, a store behind a fence - and
the guarded verb is the same body inside `P::CriticalSection`. No lambda
is handed to a guard helper: at `-Os` gcc keeps such a lambda as a
function of its own on the QingKe cores, its closure built on the stack
at every call.

The ELEMENT verbs - `push`, `pop`, `count` and `empty`/`full` over it -
are `always_inline` themselves, because they sit on the per-byte paths:
a transport's interrupt body, its blocking write, a drain loop. Left to
`-Os`, gcc keeps a verb out of line as soon as the ring's type has two
call sites for it - `pop` in a transmit vector and in `read_byte()` on
the AVR and the QingKe cores, `empty` at the two places the PL011's
vector asks it on the RP2350 - and one call in an interrupt body makes
it a non-leaf function that saves every register a callee may clobber:
sixteen on the AVR's DRE vector where the inline verb leaves six, the
twenty f-registers on top under the CH32V303's ilp32f. Inline, a verb
is about the size of the call it replaces - a few tens of bytes in most
images, up to about two hundred in a suite with many call sites, in
either direction - and no per-byte path calls into the ring. On the
CH32V203C8T6 a transmitted byte's handler body is 31 instructions with
no call, 84 cycles an interrupt between the bench's stamps
([benchmark.md](benchmark.md), letter p). The span verbs are left to
the compiler: they run once per run, and a call per run is the run's
own price.

## API and rules

- One API, always safe from either side: `push(v) -> bool` (false when
  full, nothing written), `pop() -> std::optional<T>`, `count()`,
  `empty()`, `full()`, `capacity()`. No `*_from_isr` twins (style
  rule honoured; measured on the uart ISRs: `std::optional` folds
  away completely and the DRE/RXC bodies are one instruction shorter
  than the bool + out-param form).
- `clear()` is the ONE non-concurrent operation: it rewrites both
  indices and is legal only while the other party is quiescent (init,
  or after masking its interrupt). Documented, not guarded.
- **Two granularities, one contract.** `push`/`pop` move one element -
  the shape an ISR handed one byte at a time wants. The bulk half -
  `read_span()`/`consume(n)` for the consumer, `write_span()`/
  `publish(n)` for the producer - hands a party the CONTIGUOUS run the
  SPSC invariant already made private to it, so a whole block can be
  moved at once: a DMA transfer, a memcpy, a bulk write. Nothing about
  the concurrency model changes - each side still writes only its own
  index and reads only the other's - and the DMA case is the shape
  that motivated it: a hardware channel writes the slots while only
  the software side ever moves an index. Three rules make it safe:
  - a span NEVER wraps: it stops at the end of the buffer and the
    rest comes on the next call - a caller handing the run to a DMA
    block needs ONE contiguous region, and two calls are cheaper than
    pretending otherwise;
  - a span is valid until its OWN side's next operation; the other
    side can only make it more conservative than it needed to be;
  - `consume`/`publish` CLAMP to what is actually available, so an
    over-long release cannot walk an index past the other side's.
  Nothing is visible to the consumer until `publish(n)` - written
  slots before that are the producer's private business. The index
  arithmetic is 32-bit where it must be: at the 65536-slot boundary
  the whole-buffer run does not fit the index type.
- No overwrite-oldest push: it would make the producer move `tail_`
  and break the SPSC rule that makes the lock-free path correct. A
  full ring says false; dropping and counting, or blocking, is the
  caller's policy (uart: RX drops + counts, TX blocks).
- Capacity is `size - 1`: the spare slot tells full from empty without
  a shared counter (a counter would be written by both sides).
- `T` must be trivially copyable: slots are copied byte-wise, possibly
  in an ISR.
- Sizes: power of two (bit-mask wrap), at least 2, no upper bound.

## Testing

`test/test_ring` covers both paths on the host: HostPlatform (width 4)
exercises the lock-free code for every size; a local `NarrowPlatform`
(width 1, entry-counting guard) checks that rings up to 256 slots
never touch the critical section and that wider ones wrap every
operation and leave the guard released. FIFO order, wrap-around, full
rejection, the 65536-slot ring using all 65535 slots, and a simulated
producer/consumer interleaving over a small ring are all covered
without hardware - the point of moving Ring to `util/`. The span half
has its own cases: the never-wraps rule (a straddling ring served in
two calls, both sides), the spare slot never handed out, publish and
consume clamping, spans interleaved with push/pop without losing
order, the guarded path wrapping the span operations too, and the
whole-buffer span at the 65536-slot boundary where the index-width
trap lives. The first hardware consumer is the SAM C21 Uart's DMA TX
engine (`test_samc_dma` at the bench).

## Measured on the AVR's uart driver (-Os, avr-gcc 16.2)

The console's USART2 at its default rings (64/256, 8-bit indices: the
lock-free path), against the same image with the guard forced on at
the same sizes, so that the guard is the only difference:
`write_blocking` (print's byte path) 15 instructions against 23 (no
SREG save / cli / restore); the DRE vector 36 against 50 and the RXC
vector 83 against 113, prologue and epilogue included (the DRE vector
saves six registers lock-free and seven guarded; the RXC vector's
sixteen are the kernel's post on the empty -> non-empty edge); 156
bytes of flash; RAM identical.

## GapRing: a ring that says where it lost

### What it is for

A receive interrupt loses bytes on rare paths - one that finds the ring
full, one the receiver flags as framed or failing its parity, the frames
a hardware overrun swallowed - and a consumer that carries state from
one byte to the next (SerialPort's line begun) must learn WHERE the
stream jumped. A count of the losses does not say: a byte a full ring
refuses is lost behind everything the ring holds, up to a whole ring
ahead of the consumer. `GapRing<T, size, P>` is Ring with the place: the
producer marks each gap on the slot its next element fills, and the
consumer is handed no run across a gap ([serial.md](serial.md), "The
count moves where the gap falls").

### The contract

- Every slot carries a GAP BYTE beside its element: how many elements
  were lost just before the one the slot holds. The producer's verbs are
  Ring's (`push`, `write_span`/`publish`) and one more for its rare
  paths, `lost()` - one element lost here - or `lost(n)`, n at once (a
  drain that counted them): the byte of the free slot its next push will
  fill is bumped, saturating at 255.
- The consumer's verbs are Ring's under Ring's names. `read_span()` stops
  before a slot a gap precedes; with the tail standing on one, it CROSSES
  its gap first and lends from it; `consume(n)` never passes a gap not
  crossed; `pop()` is a run of one. `skips()` counts every element lost,
  NEVER CLEARED (modulo 2^32), and moves at the crossing - so a reader
  comparing it after each `read_span()` sees it move between the last
  element before a gap and the first after it. Every gap is kept, however
  many stand: N elements with K lost deliver the other N - K, in order.
- SPSC as Ring is: a slot's gap byte belongs to whoever owns the slot -
  the producer writes it on its free slot and publishes it with the
  element, the consumer reads and clears it on a slot of its own. The one
  exception is a loss with nothing after it yet on an EMPTY ring, whose
  gap sits on the producer's free slot: the consumer crosses it at its
  next look under `P::CriticalSection`, the decision only.
- Whether any gap stands is two counts of gap slots, the producer's marks
  against the consumer's crossings - wide enough for a ring whose every
  slot holds a gap (a byte up to 255 slots), and read under the guard
  where that is wider than the platform's atomic width. The run is taken
  BEFORE they are compared: a gap marked after the run's head was read
  lies at or beyond that head, one marked before it is seen by the look
  that follows - no run lent crosses a gap however the producer
  interleaves.
- The producer's tests are against immediates, never zero: on the AVR a
  test against zero takes the zero register, which a receive vector that
  did not use it then saves and restores at every entry. A new gap slot
  is told by its byte BECOMING one.

### What it costs

The producer: nothing on `push()`; a load, a test and two or three
stores a loss. The consumer: one load pair and a branch in `read_span()`
and in `consume()` on the common path; only while a gap stands does it
read the run's gap bytes, a byte apiece, out of line, returning a scalar
- a span returned from a call travels through memory on a 32-bit core,
and merged with the common path's it was copied there by a call to
`memcpy` (seen in the STM32G0's listing, and gone). The memory: a byte a
slot. Counted on the consoles in [serial.md](serial.md).

### Testing

`test/test_ring`: a ring with no loss behaves as Ring and its epoch stays;
a byte lost to a full ring is a gap behind everything queued, crossed
only when the tail reaches it; a discard into an empty ring crossed at
the next look; `consume()` clamped at a gap and `pop()` crossing it;
every gap kept when a second stands before the first is crossed; a
stream with one loss in four delivering the other three in order, each
gap seen at the run it opens; `lost(n)`; `clear()` forgetting the gaps
and keeping the epoch; the guarded path and the saturating gap byte on
a byte-atomic platform; and a random lossy numbered stream in which
every run lent is consecutive, every jump is seen at the run it opens
and only there, and every element sent is delivered or counted.

## HardwareRing: the ring whose producer is the hardware

### What it is for

A receive stream that a DMA channel writes in CIRCULAR mode: the channel
writes the storage lap after lap with no CPU in the path, and its own
count register says how far into the lap it has got. Nobody stores a
producer index - the counter IS it, head = size - remaining - and
`HardwareRing<storage, Counter>` is the consumer half of that ring:
Ring's consumer verbs under Ring's names (`read_span()`/`consume(n)`,
`pop()`, `count()`, `empty()`, `capacity()`, `clear()`), so a transport
whose receive engine turns circular changes a type and not its code.
With a byte element it is `util/stream.hpp`'s `SpanSource` itself. What
it adds is what a producer that cannot be stopped forces: the overrun
count and the skip epoch (`overruns()`, `skips()`), and one look that
writes nothing (`waiting()`) for a context that is not the consumer's.

It is a second flavour and not Ring with a hook, because the hardware
producer breaks the two things Ring's consumer stands on. Ring's
producer never writes the consumer's run - the SPSC invariant makes the
run private - and it refuses to push into a full ring. A circular
channel does neither: it never stops, never learns where the tail is,
and writes over the oldest elements when the consumer falls a lap
behind. So the view does what Ring never needs to: it ACCOUNTS for the
laps it did not keep up with, and it judges a run at its release.

### The contract: `RingCounter`

The producer is a type with two static functions, each ONE READ of its
state:

- `remaining()` - the elements still to be written in the current lap,
  in [0, size]: the channel's count register, which the hardware reloads
  to the storage's length at every wrap. It counts ELEMENTS, the
  channel's beats, never bytes; and it counts an element as written only
  once a read of its slot returns it.
- `laps()` - the laps completed since the producer started at the
  storage's first element: the count the channel's completion interrupt
  increments at every wrap. It may LAG the counter - the handler runs
  after the wrap it counts - and must never LEAD it.

The storage is the CALLER's array, named in the view's type (a reference
template parameter, as `JournalPanic` names its journal): the caller
places and aligns it where the channel can reach, and the array's type
gives the element and the size - a power of two, at least 2. The view
is a monostate like the engines and transports it sits between; two
views of one storage are one view.

### Positions, and why the lap count is read first

The view keeps two 32-bit POSITIONS counted from the producer's start,
modulo 2^32: the consumer's tail, and the producer's head as last looked
at. A look computes

    head = laps() * size + (size - remaining()) mod size

- a shift and a mask, the size being a power of two (no multiply: the
CH32V003 has none). The two reads are not one atomic snapshot, and the
ORDER decides which way a look can err:

| read first | a wrap between the two reads, or a completion not yet run | what the view can do |
|---|---|---|
| `laps()` (the view's order) | the count is past the wrap, the laps are not: the head is a lap TOO FEW | a head behind the last look is impossible, so the lap is added back |
| `remaining()` | the count is before the wrap, the laps after it: a lap TOO MANY | nothing - a phantom lap reads exactly like a real overrun |

So the view reads the laps first, with a compiler fence before the count
so that no counter implementation can reorder them, and its only error
is a lap low - which it corrects from the head it saw last. That
correction is exact while the consumer looks MORE OFTEN THAN ONCE A
LAP: a head that has moved a whole lap or more since the last look and
is then a lap low cannot be told from one that has not moved.

### The overrun: skip rather than tear

The lap count is the only witness of a consumer that fell behind. Every
look judges `head - tail`:

- `>= size` - the oldest unread element is the producer's next write,
  or already written over: ONE OVERRUN is counted and the view SKIPS -
  the tail jumps to the head, everything unread discarded, because what
  is left within a lap of the producer is racing it. Skip rather than
  tear, the block streams' doctrine ([block-stream.md](block-stream.md)).
  The capacity is therefore size - 1, as Ring's: a full lap unread is
  already an overrun.

  The `>=` is load-bearing beyond the race. A lap missed while its
  completion is still pending, by a consumer that has not looked for a
  whole lap, reads as a lap less than it is; with `>`, the next look
  after the handler can find exactly a lap unread and hand it out again,
  REPEATING elements the consumer already had. With `>=` that look
  counts the overrun and skips: the late corner is a gap, never a
  repeat.

- At the RELEASE, `consume(n)` looks again - after a fence, so the run's
  reads stay before it - and if the slot at the tail was written over
  while the consumer held the run (`head - tail > size`), the run was
  TORN: the overrun is counted, the view skips, and `consume()` answers
  false. It is the one place a consumer can learn that what it just read
  was overwritten under it; a run released with true was intact when it
  was read. `util/stream.hpp`'s `SpanSource` puts no type on `consume()`'s
  return, so the answer costs a consumer that ignores it nothing.

`overruns()` counts both, each one a skip, until `clear_overruns()`;
what a skip cost in elements is not invented here - the producer's
peripheral, if it can tell, is the one to say.

**A skip is an epoch.** Whichever look skips - the drain's own
`read_span()`, a `count()` asked between two drains (a transport's
`rx_pending()`), a release refused - the stream the consumer reads jumps
there, and the run handed out next is the stream after a gap. A reader
that carries state ACROSS its runs - `SerialPort`'s partial line
([serial.md](serial.md)) - must learn of a jump its own calls did not
make, and the empty run a skipping `read_span()` returns cannot tell it
(an empty run is also a drained ring). So `skips()` counts every skip
and is NEVER cleared: the reader keeps the value it saw last and
compares it at every run, one load and one compare a run. `overruns()`
is the same count less its value at the last `clear_overruns()`, so the
two verbs that existed keep their meaning and a reader's epoch survives
a `clear_errors()`. `clear()` does not move it: a restarted producer is
not a skip.

### No critical section, no platform

The view's state - the two positions and the skip count - belongs to
the consumer alone, and the producer's two numbers arrive in one read
each, so there is nothing to mask and no `Platform` parameter to read an
atomic width from.

Every verb that looks may skip, and a skip moves the tail, so every verb
is the CONSUMER's - but one. An interrupt body that wants the edge test
(a transport's `harvest()` called from its vectors, [serial.md](serial.md))
asks `waiting()`: the same look, laps first and the lap a pending
completion has not counted added back against the head last recorded,
that WRITES NOTHING - neither position moves, nothing is counted, a lap
missed reads as `size` or more and is left for the consumer's next look.
A `count()` there would be a defect, not a cost: made while the consumer
holds a run and the producer has lapped it, it skips the tail under the
run, and the release that follows finds a small queue and answers TRUE
for a run the producer wrote over. `waiting()` reads the two positions
whole - one word on every core with a DMA ring - and at worst stale,
which can only make its answer larger: an edge reported early, never
one missed. A 32-bit lap count that a byte-atomic core would tear
is the producer's to read whole, because the producer is the one that
knows its platform. The fences are Ring's: compiler-only, which is
enough on the in-order cores with no data cache between a channel and
the core that every family with DMA has today - the assumption this
contract carries ([overview.md](overview.md), "Authority of util/").

### What it cannot know

- An element landing between the count read and the run's use is not
  in the run: it is the next call's.
- A lap missed while the completion that counts it is still pending, by
  a consumer that has not looked for a whole lap, is counted at the
  first look after the handler has run; the run handed in between holds
  the stream's real elements, from after the gap.
- A completion handler held off for a whole lap loses that lap from the
  count - one flag latches one wrap - and the view cannot see a lap the
  count never had.
- A consumer that does not look for 2^31 elements outruns the positions'
  arithmetic.
- `clear()` is for a producer (re)started at the storage's first element
  with its lap count at zero, and like Ring's it is not concurrent.

### What it costs

A look is the lap word, the count register and the two positions:
counted at `-Os` on a translation unit instantiating the view over a
256-byte storage and a counter shaped like a circular channel's (a
volatile count register, a volatile lap word), `read_span()` is 41-46
instructions on the 32-bit cores (Cortex-M0+, M4, QingKe V2C and V4B,
Hazard3) against Ring's 16-20, and `consume()` 30-37 against Ring's
10-18; on the Cortex-M0+ each runs about 30 instructions on the path
that hands or releases a run, with one load of the count register. The
run is the unit: `pop()` pays two looks for one element (54-62
instructions), so a consumer that can take a run takes it. `waiting()`
is 17-18 instructions on the Cortex-M0+, the M4 and the QingKe V4B, one
load of the count register, no store and no call; the skip epoch moves
nothing on the paths that hand or release a run (`read_span()` and
`consume()` are the same size with it), and `overruns()` and
`clear_overruns()` pay a subtraction and a load each.

### Testing

`test/test_ring` drives the view with a SCRIPTED CIRCULAR CHANNEL honest
to the contract and to the silicon's habits: one element at a time, each
holding its own position, a count that reloads at a lap's end (or reads
0 there, both conventions covered), a lap count kept by a completion
handler fed by one latched flag - held off on demand, so it lags and a
second wrap while held is lost - and a burst that can land BETWEEN the
two reads of a look. The cases: a run up to the head; a run cut by the
end of the storage and the rest on the next call; a consume across the
wrap; every wrap over thousands of laps with the handler held at random,
every element once and in order; a pending completion inferred from the
head going back; a wrap between the two reads of a look; a lap missed,
counted, skipped and the stream resumed; a run written over while held
refused at its release, and one whose tail slot was not yet reached
released; `pop()` refusing an element written over under it; the byte
view as a `SpanSource` and the 16-bit view counting beats; `clear()`
with a restarted producer; positions running through 2^32; lossy
traffic where every gap is counted and every released run is a slice of
the stream; the late corner, where a gap may be counted late but is
never a repeat; the skip epoch moved by every look that skips and by a
refused release, through `clear_overruns()` and `clear()` unmoved; and
`waiting()` asked while a run is held and while a lap is missed,
moving no position and counting nothing, so the release that follows
still refuses a run written over. On the silicon, the circular receive engines of the
STM32F4, the STM32G0 and the CH32V203 produce into it and their
transports read through it: a host's stream arrives whole at every rate
the bridge carries where the run engine lost up to a tenth of it, a
burst the consumer sleeps through is one overrun counted and the next
run exact ([../stm32f4/dma.md](../stm32f4/dma.md),
[../stm32g0/dma.md](../stm32g0/dma.md),
[../ch32vx03/dma.md](../ch32vx03/dma.md)).
