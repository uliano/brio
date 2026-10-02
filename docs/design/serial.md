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

**The run is the unit, the byte its degenerate case.** Every transport
offers `write_bulk(run)` beside `write_byte` (`util/stream.hpp`'s
`BulkSink`): as many of the bytes as the transmit ring has room for,
copied into the ring's own free run and published with one index store
a contiguous part, never blocking, and returning how many it took.
Where the transmitter starts on a byte, it is started on the run's
FIRST byte, before the rest is copied. A transport whose interrupt
drains its ring - the SAM C21's, the STM32G0's, the STM32F4's, the
three CH32 strata's, the AVR's - pushes that byte and arms its
interrupt exactly as `write_byte` does, copies the rest and arms again
behind it, because every one of those handlers disarms on the ring that
one byte emptied (the SAM C21's and the AVR's as they pop it, the
STM32G0's FIFO in the same entry, the others on the entry after): two
nudges a run at most, the second idempotent. The PL011's writes an idle
FIFO directly. Where the transmitter starts on a block - a DMA engine,
a USB packet - the run is queued whole and nudged once behind it. A run
that finds no room does what a refused byte does on the same transport.
`print` hands a `string_view` over as one run and a C string - every
formatted number is one - as its first byte through `write_byte`, then
in stretches of up to 64 bytes, measured as it goes (`util/print.hpp`),
so an idle transmitter starts before anything is measured or copied; on
an engined transport that first byte is a block of its own. A sink with
the byte verb alone - a test capture, a simulated port - is fed a byte
at a time. Counted in the release listings, a printed byte that finds
room in the ring costs the thread 10 instructions on the CH32V203 (a
scan and a copy of five each) against 21 through `write_byte` - a call
and a read-modify-write of the USART's control register every byte -
12 against 22 on the SAM C21, 12 against 27 on the STM32G0 (its byte
path masking interrupts around the register write every byte), 9
against 19 on the STM32F446, plus a run's own fifty-odd (the CH32V203)
to seventy-odd (the SAM C21). On a block with one data register and no
FIFO the line does not show it: a print is wire-bound, the interrupt
count is the block's shape, and a print longer than the ring's room is
a spin either way - what moves is the thread's cost of a line that
fits. What the line does show is the
start: measured with `bench_*`'s letter p, a print's first frame leaves
within a few tens of cycles of where the byte verb alone sends it, on the
SAM C21 at 48 MHz and the STM32G0 at 64 MHz alike, and the start costs
the interrupts the byte verb's start costs - the handler entered for
the first byte alone, so the SAM C21 takes one entry more than a print
has bytes and the STM32G0's FIFO serves the first two bytes one an
entry before its refills of eight.

The receive-side ISR body returns the RX ring's empty -> non-empty
EDGE: the app ISR glue posts one `RxActivity` event on true. No event
flood, no lost wakeup - only draining empties the ring, so the next
byte is an edge again.

### Realizations

Common to all: fifteen verbs spelled identically - the run and the
byte both ways, `write_bulk`, `write_byte`, `read_span` with `consume`
and `read_byte`, then `init(clock, baud)`, `rx_pending`, `tx_idle`,
`actual_baud`, `rebase`, `clear_errors` and the four counters
`frame_errors`, `parity_errors`, `rx_overruns`, `hw_overruns` - the two
rings, and the edge contract above; and four more everywhere but the
CH32V00x, whose task has not got them: `set_baud(hz, baud)`, `release`,
`can_baud` and `min_hz_for`. What differs is how the pins are named,
how many vectors the silicon gives the port, and what each family's
port has that the others' has not.

| stratum | realization | beyond the contract |
|---|---|---|
| avrdx | `Uart<n, Route, rx_size, tx_size>` (`avrdx/usart.hpp`) | the pins are a PORTMUX `Route`; THREE vectors, so the bodies are `rxc()` (the edge) and `dre()`; `set_baud()` and `release()` as on the other two |
| samc21 | `Uart<n, UartPads, rx_size, tx_size, TxEngine, RxEngine>` (`samc21/sercom.hpp`) | the pins are SERCOM pads with their pins; ONE vector, `isr()` returns the edge; two OPTIONAL DMA engine slots (`NoDmaEngine` by default, compiling to nothing) with `dma_isr()`, `dma_faults()`, `harvest()`, and `read_bulk()`, the receive run copied out |
| stm32g0 | `Uart<n, UartPins, rx_size, tx_size, TxEngine, RxEngine, opts>` = `UartTask<Usart<n>, ...>` (`stm32g0/usart.hpp`) | the pins carry their AF; one vector and `isr()`; the same engine slots and `read_bulk()`; `UartOptions` as one trailing parameter (FIFO thresholds, single wire, ...); `kernel_hz()` (the kernel-clock multiplexer), `noise_errors()` (NE exists here alone), `wakes()` (the wake from Stop); the same task over an LPUART is `LpUart` (`stm32g0/lpuart.hpp`) |
| ch32v00x | `Uart<n, P, rx_size, tx_size, TxEngine, RxEngine, remap, opts>` = the task over `Usart<n>` (`ch32v00x/usart.hpp`) | the pins are the instance's own under a REMAP CODE (`afio.hpp`'s tables), the code a template parameter; one vector and `isr()`; the same engine slots with `dma_isr()`, `dma_faults()` and `harvest()`, on the channels table 8-2 gives the instance (the channel IS the request on this family, an engine elsewhere refused); USART2 refused at code 0, whose TX is the K8 package's reset pin; `UartOptions` as one trailing parameter (the frame, a single wire that is a BUS and not a loop on this silicon, the flow-control pair); BRR is PCLK over the baud, whole - no separate fractional field and no kernel-clock multiplexer |
| stm32f4 | `Uart<n, pins, rx_size, tx_size, TxEngine, RxEngine, opts>` (`stm32f4/usart.hpp`) | the classic SR/DR/BRR block (the CH32V00x's under ST's names): flags cleared by read sequences, the divisor in sixteenths or eighths (OVER8); the pads as `PinSel`s with the datasheet's AF; ten instances at most with a vector each, the U(S)ART name deciding FULL or not; the divisor from the instance's OWN APB clock (`apb_hz`), because the two buses run below HCLK; two OPTIONAL DMA engine slots (`stm32f4/dma.hpp`'s `DmaTxEngine`/`DmaRxEngine`) with `dma_isr()`, `dma_faults()` and `harvest()`, named by (controller, STREAM, channel) because a stream is the unit here and a peripheral reaches only the cells the request mapping gives it - checked at compile time against the reserve's per-part-class table and refused on a class whose manual was not read; clearing a receive error costs the byte in DR, this block's flags going away only by a read sequence ([../stm32f4/usart.md](../stm32f4/usart.md), [../stm32f4/dma.md](../stm32f4/dma.md)) |
| rp2040 | `Uart<n, pins, rx_size, tx_size, TxEngine, RxEngine>` (`rp2040/uart.hpp`), the family's alias of the IP stratum's `Pl011Transport` (`pl011/uart.hpp`) | the PL011 with its 32-deep FIFOs: the pins are GPIO numbers under function 2, and table 279 decides which of them carry which instance; one vector and `isr()`; `write_byte()` and `write_bulk()` WRITE AN IDLE FIFO DIRECTLY and arm the transmit interrupt only behind a full one, because the PL011's transmit interrupt is an edge that stays latched and not a level - so an idle transmitter takes 32 bytes with no interrupt, a long print takes one per FIFO level, and a refused byte writes nothing; the divisor's integer and fractional halves are latched by the LCR_H write that follows them; the same engine slots (`rp2040/dma.hpp`'s `DmaTxEngine`/`DmaRxEngine`, on any two distinct channels) with `dma_isr()`, `dma_faults()`, `harvest()`, `read_bulk()` - the receive run read off TRANS_COUNT by `harvest()`, the FIFO emptied and the credits cleared before a run ([../rp2040/uart.md](../rp2040/uart.md)) |
| ch32vx03 | `Uart<n, P, rx_size, tx_size, format, TxEngine, RxEngine, remap, opts>` = the task over `Usart<n>` (`ch32vx03/usart.hpp`), n = 1..8 | the pins are the instance's own under a REMAP CODE (`afio.hpp`'s columns, the code a template parameter, a code of 0 writes no register at all, and a column on the debug port's pads is refused by `init()` while the probe's port is alive); the FRAME is a template parameter of its own here, ahead of the engines, where the CH32V00x carries it inside its options; one vector and `isr()`; the same engine slots with `dma_isr()`, `dma_faults()` and `harvest()`, on the SLOT - controller and channel - the request table gives the instance (UART4..UART8's are DMA2's on the CH32V303) (the channel IS the request on this family too, an engine elsewhere refused at compile time); `UartOptions` as the trailing parameter (a single wire that is a BUS and not a loop, the flow-control pair on a FULL instance); the divisor is the instance's OWN bus over the baud, whole - USART1 asks PCLK2 and every other port PCLK1 - and WHICH instances a part offers, and which are FULL, is the datasheet's table and not a header's: the fourth port is a USART4 on the CH32V203C8 alone, so the smartcard and the synchronous clock exist there and are compile errors on a UART; the CH32V303's lot-keyed MARK/SPACE parity and short words are verbs that ask the die ([../ch32vx03/usart.md](../ch32vx03/usart.md)) |
| ch32x035 | `Uart<n, P, rx_size, tx_size, format, TxEngine, RxEngine, remap, opts>` = the task over `Usart<n>` (`ch32x035/usart.hpp`), n = 1..4 | the CH32V203's parameter list and surface, verbatim: the pins the instance's own under a REMAP CODE (`afio.hpp`'s columns, a code of 0 writes no register, a column whose TX or RX sits on the debug port's pads refused by `init()` while the probe owns them, one whose TX pad shares its package pin with another refused at compile time); the frame a template parameter of its own; one vector and `isr()`; the two engine slots take `NoDmaEngine` alone - the DMA chapter is not written here - so `dma_isr()`, `harvest()` and `dma_faults()` answer false and zero; every instance counts its divisor in HCLK, the series having no bus prescaler, and which instances a part offers is read off its pins (the CH32X035F8U6 has no USART1) |
| rp2350 | `Uart<n, pins, rx_size, tx_size, TxEngine, RxEngine>` (`rp2350/uart.hpp`) | THE SAME PL011, AND THE SAME DRIVER: the resource and the transport are the RP2040's verbatim because they are neither family's - they live in the IP stratum (`pl011/uart.hpp`), and what each family writes is a TRAITS type. So everything the row above says of the transport holds here, and what this file adds is the chip's: the pads march in groups of four as on the RP2040 but over a longer bank, and EVERY GROUP'S FLOW-CONTROL PADS CARRY DATA TOO under a SECOND function code - the CTS pad is also that instance's TX, the RTS pad also its RX - so a pad's function code is not a choice but a property of the pad, which is why the IP file keeps that code opaque and this file makes it a TYPE; UARTCLK is clk_peri as before; and the ONE interrupt line per instance is bound under ONE NAME on both of this chip's processor architectures, the interrupt numbering being shared between them ([../rp2350/uart.md](../rp2350/uart.md), [../pl011/README.md](../pl011/README.md)) |
| host | none | `SerialPort` is host-tested over a scripted `ByteTransport` and over a transport lending a real ring's run, every case through both drains (`test_serial_port`); the IP stratum's own driver is tested against a PL011 made of RAM (`test_pl011`, `host/sim_pl011.hpp`), which is the second realization that proves it knows no chip |

## SerialPort (`util/serial_port.hpp`)

`SerialPort<Transport, P, LineSink>` is the kernel citizen above the
driver: drains the ring, feeds two ping-pong LineAssemblers, posts
`LineReceived{char*}` to the sink. The 80-byte line travels BY
REFERENCE (never enters a queue) and is valid and mutable only during
the sink's dispatch. With both buffers in flight it stops draining
(the ring absorbs - that is its job) and SELF-POSTS RxActivity to
resume later.

**The drain takes a run where the transport lends one.** A transport
that offers its receive ring's consumer half in place - `SpanSource`
(`util/stream.hpp`): `read_span()`, the contiguous run ready to be
read, and `consume(n)`, the ring's own two verbs - is drained a run at a
time: the bytes are fed to the assembler where the ring holds them, the
inner loop carries the byte, the active assembler and the end of the run
and nothing else, and one index store releases every byte taken. Any
other `ByteSource` is drained a byte at a time through `read_byte()`.
Both stop at the byte that completes the second line in flight and
leave every byte after it queued, so they deliver the same lines in the
same dispatches. Every interrupt-driven transport lends the pair - each
stratum's `Uart` and the USB CDC class - so every console is drained by
the run. Counted on the CH32V203's console (`-Os`): 14 instructions an
ordinary byte by the run, against 28 through `read_byte()`, the ring's
`pop` inline in the loop.

**Scheduling contract**: the line consumer must precede SerialPort in
the Tenuto pack. The kernel then consumes every posted line before
SerialPort runs again, which is why `in_flight` can reset at dispatch
entry and two buffers are exactly sufficient. The line is a
`Lease::dispatch` loan (`Borrowed<char, Lease::dispatch>`, see
[kernel.md](kernel.md) section 4): SerialPort declares
`LendsTo = Subscribers<LineSink>` and Tenuto refuses a pack that
violates the order.

## TX policy: block, don't drop

TX stays the blocking push print: the drain side is an ISR (it
preempts the loop, so the spin always progresses - a stall, not a
deadlock), worst case ~2 ms at 460800, zero when the ring has room.
Where the drain side is a DMA engine, nothing re-arms it once it has
stopped - a block abandoned on a transfer error leaves it idle with
the ring full - so every engined transport pumps its engine on a
REFUSED `write_byte` or `write_bulk` too, and the spin stays a stall.
(A plain
transport needs no such pump: the push that filled the ring armed a
level that cannot be missed. The PL011's transmit interrupt is an edge
that stays latched, so its transport writes an idle FIFO directly and
arms the interrupt only behind a full one - every queued byte then has
an edge coming or one latched, and a refused byte or run writes
nothing, [../pl011/README.md](../pl011/README.md).)
Measured cost of write_byte: ~45-50 cycles/byte with a guarded ring,
under 10% of the 21.7 us wire time per byte; with the lock-free ring
the hot path is 15 instructions against 23 for a guarded one (no SREG
save/cli/restore; the AVR's console, [ring.md](ring.md)). A printed
string takes the run instead (above).

Full-queue semantics cost nothing on the non-full path (the check
exists anyway), so the policy is pure failure semantics, chosen per
direction:

- **RX drops + counts** - the world cannot be paused;
- **TX blocks** - we can wait, and half-messages are worse than late
  ones.

A message-atomic drop ("say it all or say nothing" + counter) is the
noted future option for telemetry streams.
