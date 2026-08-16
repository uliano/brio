# The serial stack

Bytes below, line events above, ownership by reference. Decisions of
2026-08-13.

## Uart driver (`avrdx/uart.hpp`)

`Uart<n, Route, rx_size, tx_size>` - static interrupt-driven byte
transport: SPSC rings on both sides ([ring.md](ring.md)), ISR bodies
(`rxc()`, `dre()`) bound by the app, RXDATAH error counters (named
statics, inspectable from gdb). Ring defaults 64/256: 8-bit indices on
both sides, hence lock-free on AVR - neither the ISRs nor `write_byte`
mask interrupts (2026-08-16; before that every main-side ring op paid
an SREG save/cli/restore). The driver
moves bytes and nothing else; `brio::print` formats on top of any
ByteSink.

`rxc()` returns the RX ring's empty -> non-empty EDGE: the app ISR
glue posts one `RxActivity` event on true. No event flood, no lost
wakeup - only draining empties the ring, so the next byte is an edge
again.

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
entry and two buffers are exactly sufficient.

## TX policy: block, don't drop

TX stays the blocking push print: the drain side is an ISR (it
preempts the loop, so the spin always progresses - a stall, not a
deadlock), worst case ~2 ms at 460800, zero when the ring has room.
Measured cost of write_byte: ~45-50 cycles/byte with the old guarded
ring, under 10% of the 21.7 us wire time per byte; the lock-free ring
of 2026-08-16 shortens the hot path from 20 to 13 instructions (no
SREG save/cli/restore) - re-measure when convenient.

Full-queue semantics cost nothing on the non-full path (the check
exists anyway), so the policy is pure failure semantics, chosen per
direction:

- **RX drops + counts** - the world cannot be paused;
- **TX blocks** - we can wait, and half-messages are worse than late
  ones.

A message-atomic drop ("say it all or say nothing" + counter) is the
noted future option for telemetry streams.
