# The serial stack

Bytes below, line events above, ownership by reference.

## The Uart task

`Uart<n, ...>` - the static, interrupt-driven byte transport every
stratum spells the same way: SPSC rings on both sides
([ring.md](ring.md)), ISR bodies the app binds to its vectors, the
error counters as named statics (inspectable from gdb). Ring defaults
64/256: 8-bit indices on both sides, hence lock-free on every stratum,
the AVR included - neither the ISRs nor `write_byte` mask interrupts.
The driver moves bytes and nothing else; `brio::print` formats on top
of any ByteSink.

The receive-side ISR body returns the RX ring's empty -> non-empty
EDGE: the app ISR glue posts one `RxActivity` event on true. No event
flood, no lost wakeup - only draining empties the ring, so the next
byte is an edge again.

### Realizations

Common to the three: fifteen verbs spelled identically - `init(clock,
baud)`, `write`, `write_byte`, `read_byte`, `rx_pending`, `tx_idle`,
`can_baud`, `actual_baud`, `min_hz_for`, `rebase`, `clear_errors` and
the four counters `frame_errors`, `parity_errors`, `rx_overruns`,
`hw_overruns` - the two rings, and the edge contract above. What
differs is how the pins are named, how many vectors the silicon gives
the port, and what each family's port has that the others' has not.

| stratum | realization | beyond the contract |
|---|---|---|
| avrdx | `Uart<n, Route, rx_size, tx_size>` (`avrdx/usart.hpp`) | the pins are a PORTMUX `Route`; THREE vectors, so the bodies are `rxc()` (the edge) and `dre()`; no `release()` - the resource's teardown is `Usart<n>`'s |
| samc21 | `Uart<n, UartPads, rx_size, tx_size, TxEngine, RxEngine>` (`samc21/sercom.hpp`) | the pins are SERCOM pads with their pins; ONE vector, `isr()` returns the edge; two OPTIONAL DMA engine slots (`NoDmaEngine` by default, compiling to nothing) with `dma_isr()`, `dma_faults()`, `harvest()`, `write_bulk()`/`read_bulk()`; `release()` |
| stm32g0 | `Uart<n, UartPins, rx_size, tx_size, TxEngine, RxEngine, opts>` = `UartTask<Usart<n>, ...>` (`stm32g0/usart.hpp`) | the pins carry their AF; one vector and `isr()`; the same engine slots and bulk verbs; `UartOptions` as one trailing parameter (FIFO thresholds, single wire, ...); `set_baud()` on the task, `kernel_hz()` (the kernel-clock multiplexer), `noise_errors()` (NE exists here alone), `wakes()` (the wake from Stop); the same task over an LPUART is `LpUart` (`stm32g0/lpuart.hpp`) |
| host | none | `SerialPort` is host-tested over a scripted `ByteTransport` |

## SerialPort (`util/serial_port.hpp`)

`SerialPort<Transport, P, LineSink>` is the kernel citizen above the
driver: drains the ring, feeds two ping-pong LineAssemblers, posts
`LineReceived{char*}` to the sink. The 80-byte line travels BY
REFERENCE (never enters a queue) and is valid and mutable only during
the sink's dispatch. With both buffers in flight it stops draining
(the ring absorbs - that is its job) and SELF-POSTS RxActivity to
resume later.

**Scheduling contract**: the line consumer must precede SerialPort in
the Kernel pack. The kernel then consumes every posted line before
SerialPort runs again, which is why `in_flight` can reset at dispatch
entry and two buffers are exactly sufficient. The line is a
`Lease::dispatch` loan (`Borrowed<char, Lease::dispatch>`, see
[kernel.md](kernel.md) section 4): SerialPort declares
`LendsTo = Subscribers<LineSink>` and the Kernel refuses a pack that
violates the order.

## TX policy: block, don't drop

TX stays the blocking push print: the drain side is an ISR (it
preempts the loop, so the spin always progresses - a stall, not a
deadlock), worst case ~2 ms at 460800, zero when the ring has room.
Measured cost of write_byte: ~45-50 cycles/byte with a guarded ring,
under 10% of the 21.7 us wire time per byte; with the lock-free ring
the hot path is 13 instructions vs 20 for a guarded one (no SREG
save/cli/restore).

Full-queue semantics cost nothing on the non-full path (the check
exists anyway), so the policy is pure failure semantics, chosen per
direction:

- **RX drops + counts** - the world cannot be paused;
- **TX blocks** - we can wait, and half-messages are worse than late
  ones.

A message-atomic drop ("say it all or say nothing" + counter) is the
noted future option for telemetry streams.
