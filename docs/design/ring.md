# Ring: the SPSC FIFO

`util/ring.hpp` - `Ring<T, size, P>`.

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
