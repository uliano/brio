# Ring: the SPSC FIFO

`util/ring.hpp` - `Ring<T, size, P>`.

## What it is for

A bounded FIFO between exactly two parties: one producer, one
consumer. Driver byte rings (UART today, I2C/SPI next), sample
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
`Ring<uint8_t, 1024, SomeArmPlatform>` is lock-free, with the same
source. Nobody chooses; the platform states a truth, generic code
draws the consequence. This is the pattern the generalization rule
asks for: no `#ifdef`, no per-use knob, no hidden default (a default
platform in Ring would smuggle an AVR include into `util/`).

The extra template parameter is the honest price, and it is the same
price `EventQueue`, `SerialPort`, `SpiBus` and `Kernel` already pay:
the app names its platform once (`using P = AvrPlatform;`) and every
service reads it from there.

## API and rules

- One API, always safe from either side: `push(v) -> bool` (false when
  full, nothing written), `pop() -> std::optional<T>`, `count()`,
  `empty()`, `full()`, `capacity()`. No `*_from_isr` twins (style
  ruling honoured; measured on the uart ISRs: `std::optional` folds
  away completely, the DRE/RXC bodies got one instruction shorter).
- `clear()` is the ONE non-concurrent operation: it rewrites both
  indices and is legal only while the other party is quiescent (init,
  or after masking its interrupt). Documented, not guarded.
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
without hardware - the point of moving Ring to `util/`.

## Measured on the uart driver (-Os, avr-gcc 16.2)

`write_blocking` (print's byte path): 20 -> 13 instructions, no
SREG save / cli / restore; RXC ISR 41 -> 39, DRE ISR 37 -> 36; flash
7056 -> 7034 bytes; RAM identical.
