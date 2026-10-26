# The serial stack

This page decides how a program talks over a UART: the transport
every family writes under one surface (bytes in a run, interrupt
handler bodies the application binds, counters), the two services over
it (`LineReader`, which turns received bytes into `LineReceived` events
for an active object, and `LineWriter`, the one writer of a port whose
printers preempt one another, which puts every line on the wire whole),
the output policy (a line is written whole or refused and counted,
received bytes are dropped and counted), and how numbers become text. It is
written for an embedded programmer who wants a console on a board:
lines typed by a person or a script in, `print` out.

The smallest use is three types and two rules. A `Uart` of the family,
with the sizes of its two rings as template arguments; a `LineReader`
over it, an active object (a component with its own event queue,
[kernel.md](kernel.md)) which drains the receive ring and posts each
complete line to another active object of the program's, the line
sink; and either `print(uart, ...)` straight into the transport, or,
under the preemptive kernel, a `LineWriter` over it and
`print(Writer{}, ...)` wherever text goes out. The first rule that
decides correctness: **the line sink must come before `LineReader` in
the list of active objects the application gives the kernel**, because
the line is lent to the sink for one dispatch (one call of the sink's
handler) and `LineReader` overwrites the buffer with the next line it
assembles once it runs again; a sink that saved the pointer would read
that next line. The kernel refuses a list in the wrong order when the
program is compiled. The second: **a port with a writer has one
writer**, and every line goes through it. And one cost to know: `print`
straight into a transport waits until the transport has accepted every
byte, so a long message on a slow port stalls every other active object
for that time; through a writer only main context and the one object
the writer names wait, and every other line that finds no room is
refused and counted.

## The Uart task

`Uart<n, ...>` is the static, interrupt-driven byte transport every
stratum spells the same way: a single-producer single-consumer ring on
each side ([ring.md](ring.md)), interrupt handler bodies the
application binds to its vectors, and the error counters as named
statics, inspectable from gdb. The rings default to 64 bytes for
receive and 256 for transmit, with 8-bit indices on both sides, so
publishing an index needs no interrupt masking on any stratum, the AVR
included ([ring.md](ring.md)). That is a property of the ring, not of
the whole transport: `write_byte` may still mask interrupts briefly -
the STM32G0's does on every byte, around the control register its
handler writes too, and every family's does under a transmit engine,
around the claim of the DMA channel (the costs counted after "The run
is the unit" below) - and the target's UART document names those
critical sections. The driver moves
bytes and nothing else; `brio::print` formats on top of any `ByteSink`.

### The run is the unit, the byte its degenerate case

Every transport offers `write_bulk(run)` beside `write_byte`
(`util/stream.hpp`'s `BulkSink`): as many of the bytes as the transmit
ring has room for, copied into the ring's own free run and published
with one index store per contiguous part, never waiting, and returning
how many it took. Where the transmitter starts on a byte, it is
started on the run's first byte, before the rest is copied. A
transport whose interrupt drains its ring (the SAM C21's, the
STM32G0's, the STM32F4's, the three CH32 strata's, the AVR's) pushes
that byte and arms its interrupt exactly as `write_byte` does, copies
the rest and arms again behind it, because every one of those
handlers disarms on the ring that one byte emptied (the SAM C21's and
the AVR's as they pop it, the STM32G0's FIFO in the same entry, the
others on the entry after): two nudges a run at most, the second
idempotent. The AVR copies the rest in parts of at most 32 bytes and
publishes each with a nudge behind it, so that at 3 Mbaud the wire is
not left idle while a whole run is copied
([the AVR USART page](../avrxt/usart.md)). The PL011's transport
writes an idle FIFO directly. Where the transmitter starts on a block
(a DMA engine, a USB packet) the run is queued whole and nudged once
behind it. A run that finds no room does what a refused byte does on
the same transport.

`print` hands a `string_view` over as one run, and a C string (every
formatted number is one) as its first byte through `write_byte`, then
in stretches of up to 64 bytes measured as it goes
(`util/print.hpp`'s `print_scan_run`), so an idle transmitter starts
before anything is measured or copied; on an engined transport that
first byte is a block of its own. A sink with the byte function alone
(a test capture, a simulated port) is fed a byte at a time.

Counted in the release listings, a printed byte that finds room in
the ring costs the thread 10 instructions on the CH32V203 (a scan and
a copy of five each) against 21 through `write_byte` (a call and a
read-modify-write of the USART's control register every byte), 12
against 22 on the SAM C21, 12 against 27 on the STM32G0 (its byte path
masking interrupts around the register write every byte), 9 against
19 on the STM32F446, plus a run's own fifty-odd (the CH32V203) to
seventy-odd (the SAM C21). On a block with one data register and no
FIFO the line does not show it: a print is wire-bound, the interrupt
count is the block's shape, and a print longer than the ring's room is
a wait either way; what moves is the thread's cost of a line that
fits. What the line does show is the start: measured with the bench
suites' letter p, a print's first frame leaves within a few tens of
cycles of where the byte function alone sends it, on the SAM C21 at
48 MHz and the STM32G0 at 64 MHz alike, and the start costs the
interrupts the byte function's start costs: the handler entered for
the first byte alone, so the SAM C21 takes one entry more than a print
has bytes and the STM32G0's FIFO serves the first two bytes one an
entry before its refills of eight.

**The copy into the ring has a crossover.** `write_bulk()` copies the
run into the ring's free run with a byte loop up to a length, and with
the image's `memcpy` above it (the runtime's, [runtime.md](runtime.md);
avr-libc's on the AVR), where its fixed cost, the call and the
alignment prologue, is paid back by its word moves. The length is each
family's, counted in its release listing and stated in its serial
document; below it the byte loop wins, and a console's typical print
is below it.

### The burst edge comes from a vector

The receive side's interrupt bodies (`isr()`, or the AVR's `rxc()`, its
port having a vector a side, and `dma_isr()` where a receive engine
has a vector of its own) return `true` when the receive ring holds
bytes the consumer has not been told of, and the application's
interrupt glue posts one `RxActivity` on `true`. `RxActivity` is the
one event that tells the consumer (`LineReader`) that bytes are
waiting; the interrupt glue posts it on that edge, and `LineReader`
posts it to itself when it had to stop a drain early. Told once: the next
`true` waits until the consumer has found the ring empty, so there is
no flood of events and no lost wakeup. No owner polls for that edge on
a timer, with one exception: the tail of a burst under a receive
engine where no flag of the silicon ends it, the PL011, whose receive
time-out does not rise while a channel owns the FIFO
([the PL011 page](../pl011/README.md)); that tail reaches the consumer
when the owner asks `harvest()` for it.

Which event of the silicon makes the edge is the family's, chosen from
its chapter and stated in its serial document with its cost in the
handler: a byte arriving in an empty ring on a receiver that takes one
byte an interrupt, a FIFO level with the receiver's time-out for the
tail below it, the line going idle after a burst, a circular engine's
half and full marks. Wherever the silicon has an idle flag, a time-out
or a level, the edge reaches the consumer within two frame times of a
burst's last stop bit, except on the PL011, whose fixed time-out of 32
bit periods tells a burst's tail 3.2 frames after it. A stream with no
silence is never held back: on a circular engine the half and full
marks deliver the edge before the producer laps the ring. The idle
flag is no ruler of the gap that raises it: the receiver counts its
idle frame from the stop bit as it sampled it, so its threshold moves
with the sender's rate error (ten bit times from an exact sender,
10.25 at +1 %, 11 at +3 %, measured alike on the STM32G0 and the
STM32F4, [the STM32G0 USART page](../stm32g0/usart.md) and
[the STM32F4 USART page](../stm32f4/usart.md)), so a protocol that
marks the boundary between bursts with silence leaves more than one of
the receiver's idle frames between them.

`harvest()` stays a public function on an engined transport (the
receive errors read and counted, a stream a transfer error stopped
bound again), and the vectors call it; it is idempotent, so an owner
that still calls it changes nothing. Its edge test, made in an
interrupt body over a `HardwareRing`, asks `waiting()`, the one look
that writes nothing ([ring.md](ring.md)).

### The transmit edge, and the room a writer asks

The transmit side has the mirror of that edge, for a writer that queues
a piece of output only where it fits whole and waits otherwise
(`util/stream.hpp`'s `RoomSink` and `EdgeSink`; every transport of every
family is both). `tx_room()` is the bytes `write_bulk()` would take now:
the transmit ring's free count, read from main context with one load of
the consumer's index, never waiting. It grows behind the caller's back
as the transmitter drains, a byte at a time under an interrupt
transmitter and a whole run at a time where the ring's consumer is an
engine: a DMA block holds its run in the ring until its completion, and
the USB serial port's stack pops a packet's 64 bytes out of it at once.
`tx_capacity()` is its largest value, the ring's capacity, a constant
that `LineWriter` keeps its stages within (below).
`tx_drained()` is the transmit edge: true once each time the transmitter
ran the ring dry, the read clearing it. It is
raised in the transport's own interrupt body, at the one place that
finds the ring empty and disarms - an interrupt transmitter's dry
branch, a transmit engine's completion that finds no next run, the USB
serial port's IN completion with nothing left to send, or its
configuration by the host while the ring is empty, the moment its room
goes from none to the whole ring - and the application's interrupt glue
reads it right after that body, in the same vector, as it reads the
receive edge:

```cpp
// excerpt: a console's vector; Lines is a LineReader, Edge an object of
// the program's whose event TxDrained says the ring ran dry
if (Serial::isr()) { brio::post<Lines>(brio::RxActivity{}); }
if (Serial::tx_drained()) { brio::post<Edge>(TxDrained{}); }
```

Where the transmit engine's completion runs under a vector of its own
(the DMA line's), that vector's glue reads it. An edge follows every
write that queued bytes into the ring, so a writer waiting for room
always gets one; a write the transmitter took whole into its FIFO
queued nothing and raises none, the room it found being still there. A
spare edge is possible (a run whose first byte the transmitter took
before the rest was queued, the zero-length packet behind a full USB
packet), a missing one is not. The cost is one store in the dry
branch, taken once a burst, and in the glue a load and a test per
entry.

### No byte is stolen by a clear

While a receive engine owns the data register, the CPU never reads
that register to clear an error flag: where the chapter clears a flag
by a data-register read, the channel's own read is that read, and
where a byte can complete between the status read and the data read,
the family's header names the race and bounds it. So errors injected
into a stream under an engine cost exactly the bytes they hit: N bytes
sent with K hit deliver N - K, in order, and the counters say K. On
the PL011, whose byte-wide beat drops an entry's error flags, all N
arrive, a break's zero among them, while the error interrupts count K.

### `tx_idle()` means the wire is idle

On every family: nothing queued in the transmit ring, no block in
flight on a transmit engine, and the last stop bit off the pad (the
chapter's transmission-complete flag, never answering from a frame
before the current block; an engine's writes do not run the flag's
software clear, so the transport clears it when a block starts, as
its chapter says). It turns true no earlier than the last stop bit and
within a bit time after it, so a caller that switches a direction pin,
a baud rate or a power state on it cuts no frame.

### The interrupt receiver is the console's default

A console reads lines a human or a script types: a few bytes a burst,
with silence between. The interrupt receiver, a byte or a FIFO level
an entry, is every family's receive path unless the program asks for
more. A receive engine with its edge is a stream's opt-in: a
transport whose bytes arrive faster than its interrupts can take them,
or a program that wants the CPU off the per-byte path, names an engine
in the transport's slot. `LineReader` runs over either.

### Realizations

Common to all: fifteen functions spelled identically. The run and the
byte both ways (`write_bulk`, `write_byte`, `read_span` with `consume`,
and `read_byte`), then `init(clock, baud)`, `rx_pending`, `tx_idle`
(the wire's, on every family), `actual_baud`, `rebase`, `clear_errors`
and the four counters `frame_errors`, `parity_errors`, `rx_overruns`,
`hw_overruns`; the two rings and the edge contract above; and five
more, `set_baud(hz, baud)`, `release`, `can_baud`, `min_hz_for` and
`rx_skips`, every byte the receive ring will not deliver, never
cleared (`LineReader`'s epoch, below); and the transmit pair,
`tx_room` and `tx_drained` (above). What differs is how the pins
are named, how many vectors the silicon gives the port, and what each
family's port has that the others' has not.

| stratum | realization | beyond the contract |
|---|---|---|
| avrxt | `Uart<n, Route, rx_size, tx_size>` (`avrxt/usart.hpp`) | the pins are a PORTMUX `Route`; three vectors, so the bodies are `rxc()` (the edge) and `dre()`; `rxc()` takes every frame the two-level buffer holds in one entry, the edge the burst's first byte (chapter 27 has no idle flag, time-out or FIFO level); `tx_idle()` is TXCIF, which the task owns: `dre()` clears it behind a run's last byte, and a resource user who clears it makes `tx_idle()` false until the next run ends; `write_bulk()` copies with avr-libc's `memcpy` from two bytes up; `set_baud()` and `release()` as on the other two |
| samc21 | `Uart<n, UartPads, rx_size, tx_size, TxEngine>` (`samc21/sercom.hpp`) | the pins are SERCOM pads with their pins, one pin for both directions being the loop through the pad (31.6.3.8); one vector, `isr()` returns the edge and takes every level of the receive buffer an entry; one optional DMA engine slot, the transmit one (`NoDmaEngine` by default, compiling to nothing; `DmaTxEngine`, the family's one DMA user, claimed by one transport at a time, [the SAM C21 DMAC page](../samc21/dmac.md)) with `dma_isr()` and `dma_faults()`, the receiver always the interrupt receiver; `read_bulk()`, the receive run copied out; a transmit engine's block half the ring at most; `tx_idle()` is TXC, which every write to DATA clears, a channel's beat included; the run copy the runtime's `memcpy` from 20 bytes whose two ends share their word alignment, the byte loop otherwise; the transmit engine at level 2 of four, the rule's for a transmit ([dma.md](dma.md)); `release()` resets the SERCOM before its clocks are gated, a gated instance left enabled holding its transmit request |
| stm32g0 | `Uart<n, UartPins, rx_size, tx_size, TxEngine, RxEngine, opts>` = `UartTask<Usart<n>, ...>` (`stm32g0/usart.hpp`) | the pins carry their AF; one vector and `isr()`; the same engine slots and `read_bulk()`; `UartOptions` as one trailing parameter (FIFO thresholds, single wire, the receive engine's level `rx_priority`, very_high by default, [dma.md](dma.md), ...); the interrupt receiver paced where the instance has the FIFO (RXFT at half its depth, one entry for four characters or more) with the burst's tail by the receiver time-out (RTOF, ten bit times) on a full USART and by IDLE on an LPUART (`rx_paced`, `rx_tail`); the receive engine's edge from the vectors, IDLE in `isr()` and the ring's half and full marks in `dma_isr()`, its errors entering the vector through EIE and PEIE and every flag cleared through ICR, the receive FIFO kept on under it with a kick (DMAR dropped and raised when IDLE, an error or RXFT at seven eighths finds a character still standing, the cure for a FIFO that wedges behind a starved channel, [the STM32G0 USART page](../stm32g0/usart.md)), `rx_kicks()` its count; `tx_idle()` is TC, cleared when a transmit block starts; the run copy the runtime's `memcpy` from 16 bytes whose ends share their word alignment; `kernel_hz()` (the kernel-clock multiplexer), `noise_errors()` (NE exists here alone), `wakes()` (the wake from Stop); the same task over an LPUART is `LpUart` (`stm32g0/lpuart.hpp`) |
| stm32f4 | `Uart<n, pins, rx_size, tx_size, TxEngine, RxEngine, opts>` (`stm32f4/usart.hpp`) | the classic SR/DR/BRR block (the CH32V00x's under ST's names): flags cleared by read sequences, the divisor in sixteenths or eighths (OVER8); the pads as `PinSel`s with the datasheet's AF; ten instances at most with a vector each, the U(S)ART name deciding full or not; the divisor from the instance's own APB clock (`apb_hz`), because the two buses run below HCLK; two optional DMA engine slots (`stm32f4/dma.hpp`'s `DmaTxEngine`/`DmaRxEngine`) with `dma_isr()`, `dma_faults()` and `harvest()`, named by (controller, stream, channel) because a stream is the unit here and a peripheral reaches only the cells the request mapping gives it, checked at compile time against the device tables' per-part-class table and refused on a class whose manual was not read; under a receive engine the CPU never reads DR: the vector's one status read and the stream's next read of DR make each clear, which costs no byte and suppresses the IDLE of a burst of one frame, so the vector takes RXNE once a burst beside the stream to learn the clear finished, two USART entries a burst, its edge IDLE or that frame and the lap's half and full marks; `tx_idle()` is TC, written 0 at every transmit block's start; the receive engine at `UartOptions::rx_priority` (very_high by default), the transmit at high, and an overrun left with nothing to take cleared by the vector; the single-bit interrupt enables are bit-band stores ([the STM32F4 USART page](../stm32f4/usart.md), [the STM32F4 DMA page](../stm32f4/dma.md)) |
| ch32v00x | `Uart<n, P, rx_size, tx_size, TxEngine, RxEngine, remap, opts>` = the task over `Usart<n>` (`ch32v00x/usart.hpp`) | the pins are the instance's own under a remap code (`afio.hpp`'s tables), the code a template parameter; one vector and `isr()`; the same engine slots with `dma_isr()`, `dma_faults()` and `harvest()`, on the channels table 8-2 gives the instance (the channel is the request on this family, an engine elsewhere refused); USART2 refused at code 0, whose TX is the K8 package's reset pin; `UartOptions` as one trailing parameter (the frame, a single wire that is a bus and not a loop on this silicon, the flow-control pair, the receive engine's level `rx_priority`, very_high by default, [dma.md](dma.md)); BRR is PCLK over the baud, whole, with no separate fractional field and no kernel-clock multiplexer; the receive engine circular over a `HardwareRing`, and under it the CPU never reads DATAR (the clear by a status read and the channel's next read, as the CH32V203's), the edge the vector's, IDLE or the first frame, and the ring's half and full marks; `tx_idle()` is TC, cleared at every transmit block's start |
| ch32vx03 | `Uart<n, P, rx_size, tx_size, format, TxEngine, RxEngine, remap, opts>` = the task over `Usart<n>` (`ch32vx03/usart.hpp`), n = 1..8 | the pins are the instance's own under a remap code (`afio.hpp`'s columns, the code a template parameter; a code of 0 writes no register at all, and a column on the debug port's pads is refused by `init()` while the probe's port is alive); the frame is a template parameter of its own here, ahead of the engines, where the CH32V00x carries it inside its options; one vector and `isr()`; the same engine slots with `dma_isr()`, `dma_faults()` and `harvest()`, on the slot (controller and channel) the request table gives the instance (UART4..UART8's are DMA2's on the CH32V303); the channel is the request on this family too, an engine elsewhere refused at compile time; `UartOptions` as the trailing parameter (a single wire that is a bus and not a loop, the flow-control pair on a full instance, the receive engine's level `rx_priority`, very_high by default, [dma.md](dma.md)); the divisor is the instance's own bus over the baud, whole (USART1 asks PCLK2 and every other port PCLK1), and which instances a part offers, and which are full, is the datasheet's table and not a header's: the fourth port is a USART4 on the CH32V203C8 alone, so the smartcard and the synchronous clock exist there and are compile errors on a UART; the CH32V303's lot-keyed MARK/SPACE parity and short words are functions that ask the die; under a receive engine the CPU never reads DATAR: a status read here arms a clear of every flag standing at the next data read, the channel's, so no byte is taken and the IDLE of a one-frame burst is forgotten with it; the vector takes RXNE once a burst beside the channel, two USART entries a burst, its edge IDLE or that frame and the ring's half and full marks; `tx_idle()` is TC, cleared at every transmit block's start ([the CH32V203/CH32V303 USART page](../ch32vx03/usart.md)) |
| ch32x035 | `Uart<n, P, rx_size, tx_size, format, TxEngine, RxEngine, remap, opts>` = the task over `Usart<n>` (`ch32x035/usart.hpp`), n = 1..4 | the CH32V203's parameter list and surface, verbatim: the pins the instance's own under a remap code (`afio.hpp`'s columns, a code of 0 writes no register, a column whose TX or RX sits on the debug port's pads refused by `init()` while the probe owns them, one whose TX pad shares its package pin with another refused at compile time); the frame a template parameter of its own; one vector and `isr()`; the two engine slots take `NoDmaEngine` alone, the DMA chapter not being written here, so `dma_isr()`, `harvest()` and `dma_faults()` answer false and zero; every instance counts its divisor in HCLK, the series having no bus prescaler, and which instances a part offers is read off its pins (the CH32X035F8U6 has no USART1) |
| rp2040 | `Uart<n, pins, rx_size, tx_size, TxEngine, RxEngine>` (`rp2040/uart.hpp`), the family's alias of the IP stratum's `Pl011Transport` (`pl011/uart.hpp`) | the PL011 with its 32-deep FIFOs: the pins are GPIO numbers under function 2, and table 279 decides which of them carry which instance; one vector and `isr()`; `write_byte()` and `write_bulk()` write an idle FIFO directly and arm the transmit interrupt only behind a full one, because the PL011's transmit interrupt is an edge that stays latched and not a level, so an idle transmitter takes 32 bytes with no interrupt, a long print takes one per FIFO level, and a refused byte writes nothing; the divisor's integer and fractional halves are latched by the LCR_H write that follows them; the same engine slots (`rp2040/dma.hpp`'s `DmaTxEngine`/`DmaRxEngine`, on any two distinct channels) with `dma_isr()`, `dma_faults()`, `harvest()`, `read_bulk()`, the receive run read off TRANS_COUNT by `harvest()`, the FIFO emptied and the credits cleared before a run, a run's completion onto a drained ring the engine's edge; the receive level's entry reads its sixteen entries without asking the flag register, the receive time-out (RT) the burst's tail; the four error interrupts armed and counted in `isr()` under an engine too, an overrun's sticky UARTRSR cleared there and its source masked until the next publish; the receive engine at HIGH_PRIORITY, the transmit at normal, and no level option, one bit, nothing above the ring ([dma.md](dma.md)); the run copy's crossover a member of the chip's traits (`copy_crossover`, 12 on both RP families) ([the RP2040 UART page](../rp2040/uart.md)) |
| rp2350 | `Uart<n, pins, rx_size, tx_size, TxEngine, RxEngine>` (`rp2350/uart.hpp`) | the same PL011 and the same driver: the resource and the transport are the RP2040's verbatim because they are neither family's; they live in the IP stratum (`pl011/uart.hpp`), and what each family writes is a traits type. So everything the row above says of the transport holds here, and what this file adds is the chip's: the pads march in groups of four as on the RP2040 but over a longer bank, and every group's flow-control pads carry data too under a second function code (the CTS pad is also that instance's TX, the RTS pad also its RX), so a pad's function code is not a choice but a property of the pad, which is why the IP file keeps that code opaque and this file makes it a type; UARTCLK is clk_peri as before; and the one interrupt line per instance is bound under one name on both of this chip's processor architectures, the interrupt numbering being shared between them ([the RP2350 UART page](../rp2350/uart.md), [the PL011 page](../pl011/README.md)) |
| host | none | `LineReader` is host-tested over a scripted `ByteTransport` and over a transport lending a real ring's run, every case through both drains (`test_line_reader`); the IP stratum's own driver is tested against a PL011 made of RAM (`test_pl011`, `host/sim_pl011.hpp`), which is the second realization that proves it knows no chip |

## LineReader (`util/line_reader.hpp`)

`LineReader<Transport, P, LineSink>` is the active object above the
driver: it drains the receive ring, feeds two `LineAssembler`s that
alternate, and posts `LineReceived{char*}` to the line sink. The line
(80 bytes by default) travels by reference and never enters a queue;
it is valid and mutable only during the sink's dispatch. With both
buffers in flight `LineReader` stops draining (the ring absorbs, that
is its job) and posts an `RxActivity` to itself to resume later.

**The scheduling rule.** The line sink must precede `LineReader` in
Tenuto's list of active objects. The kernel then serves every posted
line before `LineReader` runs again, which is why its count of lines
in flight can reset at the start of each dispatch and two buffers are
exactly sufficient. The line is a `Lease::dispatch` loan
(`Borrowed<char, Lease::dispatch>`,
[kernel.md, "Payloads"](kernel.md#4-payloads-what-travels-inside-the-event-kernelborrowedhpp)):
`LineReader` declares `LendsTo = Subscribers<LineSink>` and Tenuto
refuses a list that violates the order. The sink may use the line only
while handling `LineReceived`; it must not save the pointer for a
later event.

**The drain takes a run where the transport lends one.** A transport
that offers its receive ring's consumer half in place (`SpanSource` in
`util/stream.hpp`: `read_span()`, the contiguous run ready to be read,
and `consume(n)`, the ring's own two functions) is drained a run at a
time: the bytes are fed to the assembler where the ring holds them,
the inner loop carries the byte, the active assembler and the end of
the run and nothing else, and one index store releases every byte
taken. Any other `ByteSource` is drained a byte at a time through
`read_byte()`. Both stop at the byte that completes the second line in
flight and leave every byte after it queued, so they deliver the same
lines in the same dispatches. Every interrupt-driven transport lends
the pair (each stratum's `Uart` and the USB CDC class), so every
console is drained by the run. Counted on the CH32V203's console
(-Os): 14 instructions an ordinary byte by the run, against 28 through
`read_byte()`, the ring's `pop` inline in the loop.

**A line is posted only from a run released clean.** The lines
completed from a run are posted after the run's `consume()`, never
before. Where the receive ring's producer is a DMA channel
(`HardwareRing`, [ring.md](ring.md)) the channel can write over a run
while the drain reads it, and `consume()` answers `false`: then the
lines completed from that run are dropped, so is the line the
assemblers had begun when it came, both counted in `torn_lines()`, and
the drain skips the stream to its next end of line (the bytes before
it end a line whose beginning the ring skipped). A transport passes
that answer up by returning `consume()`'s `bool`; one whose
`consume()` returns nothing (every Ring-backed transport) has a
release that cannot refuse, and none of this is compiled for it: the
byte loop and its 14 instructions are the same either way. Validated
on the host (`test_line_reader`, a real `HardwareRing` over a scripted
channel that writes while a run is held).

**A gap between two runs is seen.** The stream a ring hands out can
jump. A `HardwareRing` skips when a look finds a lap unread (in the
drain's own `read_span()`, which then lends nothing, or in a `count()`
asked between two drains), and an interrupt receiver drops a byte that
finds its ring full, that the receiver flags as framed or failing its
parity, or that a hardware overrun swallowed. The run handed out after
such a gap is the stream after it, which the line begun in an
assembler would swallow as its continuation: a line delivered with a
hole in it. So every transport reports the gaps in its stream
(`util/stream.hpp`'s `SkippingSource`: `rx_skips()`, an epoch that
moves whenever the stream jumps, never cleared, moved on those rare
paths alone), and the drain compares it at every run with the value it
saw last, over every transport that has it, whether its release can
refuse or not. A change ends the line begun as torn, counted in
`torn_lines()`, and skips the stream to its next end of line: the
bytes before it end a line whose beginning the gap took.

**A drop is skipped, not placed.** A count bumped where a byte is lost
does not say where: a byte a full ring refuses is lost behind
everything the ring holds, up to a whole ring ahead of the consumer,
and the run in hand when such a count moves was received before the
loss, so the drain would tear the line it holds and deliver the cut
one spliced (measured on the host: `[LINE-TWO] [LINE-333] [FOURTAIL]`,
the first line thrown away, the splice clean). The consumer does not
need the place, only the fact. So an interrupt receiver's ring keeps a
drop epoch (`util/ring.hpp`'s `SkipRing`, two bytes, [ring.md](ring.md))
that its handler moves on its rare paths (a byte dropped for its
flags, refused by a full ring, or swallowed by an overrun, reported
where the handler meets it), and the consumer's look that finds it
moved skips: the tail jumps to the head, everything queued is
discarded with the drop, which is necessarily behind the head, and the
epoch moves, between two runs and never inside one, the meaning
`HardwareRing`'s skip has. No run handed out holds a drop, so no line
is ever delivered across one: the line begun is torn, and the stream
resumes at the next end of line or after a silence (below). What the
skip throws away beside the drop, the whole lines queued with it, is
lost only while bytes are being lost anyway. Under an overload so
steady that a drop falls between every two looks, nothing is handed
out at all ([ring.md](ring.md), "What a skip costs the stream"). A
consumer that looks between two losses is handed every byte between
them (the CH32V203's interrupt receiver, read between its breaks: 40
data bytes with 10 breaks, 40 delivered, FE 10, ten skips; read only
after them, none of what the ring held). Under a receive engine the
losses (an overrun, a frame the channel does not take, which is the
STM32G0's FE and PE with DDRE clear, a stream restarted after a
transfer error) are counted where the vector or the look sees them,
beside the ring's own skips: the line torn is the one begun when the
consumer next looks, which is the one the loss cut when the consumer
keeps up with its edges. The USB CDC class has no `rx_skips()`: its
receive ring cannot drop, since an OUT packet is armed only when the
ring has its room, and over a transport without the function (a test
capture, a simulated port) nothing of this is compiled.

**What the transport guarantees, and what it does not.** A line the
transport delivers is intact: no byte of it was dropped, refused or
flagged, and no two pieces of the stream were joined to make it. That
is all. It can still be a fragment: where a drop took a line's own end
of line, the bytes after the drop up to the next end of line are
delivered as a line whose beginning is lost (the undecidable cases
below), contiguous but not what the sender meant as one line. And the
UART's flags do not see a bit flipped inside a frame whose stop bit
and parity come out right, and nothing in the receive path can. Data
that must arrive intact over a long or noisy link carries a
CRC in its own protocol; such a protocol need not read `rx_skips()` at
all, since a skip is one more hole its CRC catches, and pays nothing
for it.

**A silence ends the skip.** A drop can take the cut line's own end of
line, and the skip can land in the middle of a line, so the bytes
after it up to the next end of line are taken for the tail of a line
whose beginning was lost. But where the drain found the ring empty
after the gap and the next run arrives 100 ms or more after that look
(`quiet_ticks`: longer than a USB serial adapter holds back a short
packet, shorter than a person or a script waits before the next
command), the line is taken to have ended in the silence and the run
is drained as a line's beginning: a burst that overflows the ring,
silence, then one command, and the command is answered. What stays
undecidable: a skip with the next line close behind it (that next line
is skipped as the cut line's tail), a sender pausing longer than the
silence in the middle of a line after a drop took its end (its tail is
taken for a line), and a drop that lands on a line's start (the drain
cannot tell it from a cut, and drops that whole line with the fragment
it expects).

The cost, counted at -Os: nothing a byte (the byte loop instruction
for instruction the same, 24 an ordinary byte on the STM32G0's console
on the Cortex-M0+, as without the check), and on that console sixteen
instructions a run: `SkipRing`'s compare of its two bytes eight (an add
each for an offset past the 64-byte ring's slots), the epoch compare
four, the skip's flag four; `consume()` tests nothing. Everything after
a gap (the tear, the skip to the next end of line, its release) is one
out-of-line function, and so is the ring's own skip: inline, their
registers and their calls cost the Cortex-M0+'s byte loop two to three
instructions a byte. In the receive vectors the reports sit on the
rare paths: the per-byte loops of the STM32G0's and the RP2040's
consoles are the plain `Ring`'s instruction for instruction (23 a
character on the STM32G0's FIFO drain, 20 on the PL011's level loop),
and the AVR's RXC vector saves no register for them (7 saved in the
serial suite's image, 10 in the console's, whose binding flattens the
kernel's post). A transport whose release can refuse and that does not
report its skips is refused at compile time
(`test/family_ch32x035/neg/`). Validated on the host
(`test_line_reader`: the scripted channel lapping a `HardwareRing`
between two drains, the skip made by the drain's own look and by one
outside it; a `SkipRing` dropping bytes between two runs, while a run
is held, on a full ring with lines queued before the drop, while both
lines are in flight, and the silence that ends a skip and the burst
that does not; a random lossy stream of numbered, checked lines in
which every line delivered and passing its check is a line sent,
whole and in order;
`test_ring`'s `SkipRing` cases, a random lossy stream among them in
which every run handed out is consecutive and every jump comes after a
skip; `test_pl011`'s receive FIFO, the framed entry, the full ring and
the overrun).

## LineWriter (`util/line_writer.hpp`)

`LineWriter<Transport, P, options>` is the one writer of a transport for
a program in which one printer can run in the middle of another's line:
the objects of a Rubato kernel ([kernel.md](kernel.md)), where a higher
object's dispatch preempts a lower one's, and an interrupt handler that
prints. Every line printed to it reaches the wire whole, the lines in
the order they were completed, or is refused whole and counted. It is a
sink type like the transport itself, not an active object: `print()`
formats each argument at the call, with the very `print_one()` verbs
`print(uart, ...)` uses, into a buffer that belongs to the caller's
line, its STAGE, and the line's end hands the stage to the transport as
one run. The bytes on the wire are the bytes `print(uart, ...)` writes
(the host suite holds twenty lines of every printable type to that,
byte for byte). The smallest use, an excerpt of a program whose clock,
transport and platform `P` are set up and whose objects `App` and
`Console` print:

```cpp
using Log = brio::LineWriter<Serial, P>;   // a long stage of 192 bytes, two of 96

int main() {
    // ... the clock, Serial::init(), interrupts enabled
    Log::init();                           // the transport claimed, before the kernel runs
    brio::Rubato<P, App, Console>::run();
}

// in a dispatch, in main context, in a handler:
brio::print(Log{}, "count = ", count, brio::crlf);
```

Under Tenuto the same program writes `using Log = Serial;` and keeps
its `print(Log{}, ...)` calls: one dispatch runs at a time there, so a
line printed straight into the transport is whole by construction, and
the image compiles nothing of the writer (a handler does not print to a
transport, `util/print.hpp` says why).

**What decides correctness.** Four duties, none of which the release
build checks:

- One writer. `init()`, called once from `main()` before the kernel
  runs, claims the transport; in a debug build (one without `NDEBUG`,
  which every cross project's release configuration defines) the
  transport's `write_byte()` and `write_bulk()` panic at the first write
  from anywhere but the writer. What stays direct is a banner printed
  before `init()` and the panic path.
- Who may wait. A line is written when the transport's room holds it
  whole. Main context and the one object the `wait_from` option names (a
  bench console whose dispatch runs main-context work) wait for that
  room, spinning on `tx_room()` as `print` to a transport spins, the
  transmit interrupt draining under them; every other line that finds
  no room is refused whole and counted. The transport's ring is the only
  queue, so a program whose other objects print faster than the wire
  sizes the ring for their bursts.
- Every `crlf` ends a line, wherever it stands in the argument list (a
  verb cannot know its position): `print(Log{}, "a", crlf, "b", crlf)`
  is two lines, and a line break inside one line is written as text,
  `"\r\n"`. `flush_line` ends a line with no bytes, the shape of a
  prompt that must leave before an end of line comes.
- A handler ends the line it starts. Every interrupt handler is one
  source with one stage, so two handlers composing pieces of lines would
  mix them.

A text is consumed at the call: a literal, a constant table and a buffer
on the caller's stack are all fine.

**Sources and stages.** A line belongs to its SOURCE: the object being
dispatched (the kernel's `dispatching<P>`), main context outside every
dispatch, or every interrupt handler alike (`P::in_handler()`). A table
of a byte per source (`max_sources` entries, 16 by default: the pack's
indices from 0, main context and the handlers in the last two) says
which stage holds the source's open line; a line from an index past the
table is refused and counted. The stages are a fixed pool: one long
stage (`long_size`, 192 bytes by default) kept for the sources allowed
to wait, which take it before a small one, and `small_stages` small
ones (two of `small_size`, 96 bytes, by default) for every source. A
line's first byte takes a free stage, the one decision made with
interrupts masked, since a preemptor landing between the look and the
claim would take the same stage; a line that finds none is refused
whole, its later bytes dropped up to its end. A line may be printed in
several calls, across dispatches too: the stage stays its source's
until the end. Each stage is at most 255 bytes and at most the
transport's `tx_capacity()`, `small_stages` one to seven and
`max_sources` at least three; a writer that breaks one of these does
not compile.

**The line's end publishes.** The stage's bytes go into the transport
as one `write_bulk()` under a LOCK, a flag meaning "a publisher is
copying". A line completed while another publisher holds the lock - by
a higher object that preempted it, or by a handler - is marked pending
with a sequence number and its caller returns: under the kernel's
nesting the preemptor finishes before the holder resumes, so the holder
writes its own line, then the pending ones oldest first, and releases
the lock. The order on the wire is the order in which lines were
completed: a higher object's whole line goes out before the open line
of the object it preempted, which follows at its end, and a handler's
line lands between two lines, never inside one. The holder waits for a
relayed line's room when it may wait itself or the line's source may,
and refuses it otherwise; a handler holding the lock relays nothing,
because nothing runs inside a handler. The source's slot is freed at
its line's end, so its next line takes a fresh stage while the last one
pends. Three sections run with interrupts masked - the stage's take,
the lock's take and the release-and-relay step that frees a written
stage and picks the next pending one, each a few stores and a scan of
the pool - and every copy into the transport runs with interrupts
enabled (the host suite reads the mask's depth in the copy).

**The counters and the mark.** `lost()` counts every line refused - no
stage free, no room, a line that outgrew its stage, a source past the
table - in lines, saturating, never retried. The wire shows where lines
are missing: the next holder of the lock writes `lost N` and CRLF before
its own line, N the losses since the last mark, when the room holds
both; a source that may wait waits for it, and for any other the mark
stays owed to the next line. `util/testbench.hpp`'s bench grammar prints
through a writer as through a transport, its prompt ended with
`flush_line` and its tallies carrying `L lost` when the writer lost
lines during the letter or the run, and `bin/brio run --strict` fails
such a run.

**Sizing.** With the `peak` option the writer also counts the most
stages held at once (`peak_stages()`) and the lines and bytes it
published (`lines_written()`, `bytes_written()`); without it none of
this is compiled. The pool is sized by `peak_stages()` against the
stages a program's deepest preemption holds at once (each source with a
line open, plus every line pending behind a waiting holder) and the
stages by the longest line of each source; `lost()` says when either is
short, and when the ring is.

**The panic path.** `drain_to_port(pump)` writes what the writer holds
into the transport by plain code - the line being published (from its
start, so a line a panic cut inside its copy shows its first bytes
twice), the pending lines in their order, then the open lines as they
stand - calling `pump` while the ring is short; `LineWriterReporter<Writer,
pump, Then>` is a panic reporter ([kernel.md](kernel.md)) that does it
before handing over. A panic masks interrupts for good, so the pump the
program names runs the transport's interrupt body by hand (on the
STM32F4, `Serial::isr()`); and the reporter runs where panic()'s
reporter runs, after `break_here()`, which on the Cortex-M families
faults when no debugger holds the core, so there the drain is reached
only under a debugger. Validated on the host alone (`test_line_writer`).

**The cost**, measured on the STM32F446RE at 180 MHz (five wait states,
the ART on), the console at 115200 baud, over a verdict-shaped line of
60 bytes in four texts and the end (the Rubato suite's letter j, each
figure the least and the most of 16, the union over ten passes; the
direct figures from `bench_stm32f4`'s letter l, the same line on the
same chip in an image with no writer, [benchmark.md](benchmark.md)):

| | cycles |
|---|---|
| the line printed from an object's dispatch, cold (the transport's three TXE entries inside: the run starts the idle transmitter) / warm, the line right behind the cold one | 2004 to 2244 / 1918 to 2118 |
| the same, warm, with interrupts masked: the core's own cycles, and what a handler's print costs | 1785 to 1937 |
| of a warm one: the four texts into the stage / the line's end, its publish | 1102 to 1108 / 667 to 864 |
| `print()` straight into the transport, warm and masked, in the same image | 1436 to 1644 |
| `print()` straight into the transport in an image with no writer, cold (two handlers inside) / warm | 2041 to 2153 / 1702 to 1750 |
| the transport's run alone, warm and masked: 2 bytes / 60 bytes | 188 to 196 / 376 to 612 |

The writer's line is the whole cost: nothing runs after the call. It
costs some 300 cycles more than the same line printed straight into the
transport, warm, a tenth to a quarter by which direct figure it is set
against: the stage's resolution at each text
(the source, its slot), the stage's take and the lock, and a copy of
every byte into the stage before the copy into the ring, at some twelve
cycles a byte from flash and eleven from RAM (gcc lays the scan-and-copy
loop out with two taken branches a byte). The line's end is mostly the transport's run of
60 bytes, whose own cost is bimodal by the ring's alignment (376 to 504
where the runtime's `memcpy` copies words, 564 to 612 where it copies
bytes); the line right behind the cold one costs some 180 cycles more
than the same line a moment later, which [benchmark.md](benchmark.md)
lists among what is not explained. A handler's print of this line holds every other interrupt for
the masked figure, about 10 us; a handler that must stay short posts a
value to an object instead. The masked sections, counted in the release
listing at the core's table counts with each taken branch's refill at
its three cycles: the stage's take about 50 cycles at most, the lock's
take about 16, the release-and-relay step about 80 at most (a refused
line and a scan of three stages); none copies. Flash: no code per
argument list, the call sites within a few bytes of the direct print's
(a call per argument), 1046 bytes of the writer's own code and 88 of
the bench grammar's tally over it - 1132 bytes more than the same
suite's letters printing straight into the transport. RAM beside the
ring: 444 bytes, the three stages' 384, their descriptors' 36 and 24 of
state with the `peak` counters.

## Output policy: whole or refused, counted

A line goes out whole or not at all, and a line that does not is
counted. Through a writer this is its own rule (above): a line is
written only when its end has arrived and the ring's room holds it
whole, and a line with no stage free, no room, or more bytes than its
stage is refused through its end, counted and marked on the wire. The
one wait left is the waiting sources' (main context and the object the
writer names), which poll the ring's room as `print` does.

Straight into a transport, `print` keeps its wait: the drain side is
an interrupt handler, which runs even while the caller spins, so the
wait always makes progress (a stall, not a deadlock). The wait is zero
when the ring has room for the whole message, and otherwise the wire
time of the bytes that do not yet fit: a byte is ten bit times, 21.7 us
at 460800 baud and 87 us at 115200, so a message that finds the
transmit ring full waits one byte time per byte it still has to place:
the ring's room over again at worst, about 5.5 ms at 460800 where the
default ring is 256 bytes (the AVR, the STM32 and the RP families) and
about 1.4 ms where it is 64 (the three WCH families). Where the drain
side is a DMA engine, nothing re-arms it once it has stopped (a block
abandoned on a transfer error leaves it idle with the ring full), so
every engined transport pumps its engine on a refused `write_byte` or
`write_bulk` too, and the wait stays a stall. A plain transport needs
no such pump: the push that filled the ring armed a level that cannot
be missed. The PL011's transmit interrupt is an edge that stays
latched, so its transport writes an idle FIFO directly and arms the
interrupt only behind a full one; every queued byte then has an edge
coming or one latched, and a refused byte or run writes nothing
([the PL011 page](../pl011/README.md)). Measured cost of `write_byte`
on the AVR's console: about 45 to 50 cycles per byte with a guarded
ring, under 10 % of the 21.7 us wire time per byte at 460800 baud;
with the lock-free ring the hot path is 15 instructions against 23 for
a guarded one, with no status-register save, `cli` and restore
([ring.md](ring.md)). A printed string takes the run instead (above).

Full-queue semantics cost nothing on the non-full path (the check
exists anyway), so the policy is pure failure semantics, chosen per
direction:

- **Receive drops and counts**: the world cannot be paused.
- **Transmit writes whole lines or refuses and counts them**: half a
  line is worse than a missing one the wire marks, and a caller that
  waits on the wire inside a dispatch holds every object below it.

## Numbers as text (`util/print.hpp`)

`print` formats on top of any `ByteSink` through conversions under
avr-libc's names and call shapes: `ltoa`, `ultoa`, `dtostrf`,
`dtostre` and the float twins `ftostrf`, `ftostre`. On the AVR they are
avr-libc's. Every other image links no C library
([runtime.md](runtime.md)), so there they are `print.hpp`'s own,
written in C++ over nothing: the integers by a divide by ten in shifts
and adds (no division and no multiply, the same instructions on a core
with neither), a 64-bit integer without a 64-bit division, `hex()` at
32 bits and at 64 (sixteen digits at most, no leading zeros, the
letters `ultoa`'s on each target).

**Off the AVR the floats are an exact binary split, not a float
routine.** A value travels as what it is: a float as a float, through
`ftostrf` and `ftostre`, a double as a double. Its bits are taken apart
and placed in a fixed-point number of 128 integer bits over 256
fraction bits, and every digit is a multiply or a divide by ten on
32-bit words: no floating-point operation, not even a float's widening
to double, and no libgcc call on any core. So the digits are the
value's own, rounded once, an exact half to even: the text the host's
`snprintf` prints (`"%*.*f"`, `"%.*e"` under each flag), which
`test_print` holds all four against over every binade of the float and
of the double from 2^-204 to 2^128, random fields and precisions, the
exact halves and the carries.

- The range: every finite float, and every double below 2^128, held
  exactly from 2^-204 up; a double below that keeps its bits from
  2^-256 up (`fixed()` is still exact there at every precision up to
  61) and reads as zero under 2^-256. A double of 2^128 or more prints
  as an infinity of its sign.
- The letters are avr-libc's: `dtostrf` writes `NAN`, `INF`, `-INF` in
  its field; `dtostre` writes `nan` and `inf` (upper case under
  `DTOSTR_UPPERCASE`), a NaN taking no `-`.
- `fixed(v, width, precision)` keeps at most 18 decimals and a field as
  wide as its 60-byte buffer, either alignment; `sci(v, precision)` and
  a bare float or double print signed (`+1.230e-03`), at most 7
  decimals, avr-libc's own bound.
- The cost, counted on two consoles at -Os: one `fixed()` of a float
  adds 1068 bytes of text on the STM32G0's (Cortex-M0+), the conversion
  inlined into its caller, the fraction's digit step, the divide by ten
  and the runtime's `memset` for the split's words, and 1208 on the
  CH32V003's (no multiplier); of a double 1168 and 1312; `fixed()` and
  `sci()` of a float together 1592 on the STM32G0.
