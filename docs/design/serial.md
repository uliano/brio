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
nudges a run at most, the second idempotent - but on the AVR, which
copies the rest in parts of at most 32 bytes and publishes each with a
nudge behind it, so that at 3 Mbaud the wire is not left idle while a
whole run is copied ([../avrdx/usart.md](../avrdx/usart.md)). The PL011's writes an idle
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

**The copy into the ring is a crossover.** `write_bulk()` copies the
run into the ring's free run with a byte loop up to a length, and with
the image's `memcpy` above it - the runtime's ([runtime.md](runtime.md)),
avr-libc's on the AVR - where its fixed cost, the call and the alignment
prologue, is paid back by its word moves. The length is each family's,
counted in its release listing and stated in its serial document; below
it the byte loop wins, and a console's typical print is below it.

**The burst edge comes from a vector.** The receive side's interrupt
bodies - `isr()` (the AVR's `rxc()`, its port having a vector a side),
and `dma_isr()` where a receive engine has a vector of its own - return
`true` when the receive ring holds
bytes the consumer has not been told of, and the app's ISR glue posts
one `RxActivity` on `true`. Told once: the next `true` waits until the
consumer has found the ring empty, so there is no event flood and no
lost wakeup. No owner polls for that edge on a timer - save the tail of
a burst under a receive engine where no flag of the silicon ends it -
the PL011, whose receive time-out does not rise while a channel owns the
FIFO ([../pl011/README.md](../pl011/README.md)) - which reaches the
consumer when the owner asks `harvest()` for it.
WHICH event of the
silicon makes it is the family's, chosen from its chapter and stated in
its serial document with its cost in the handler: a byte arriving in an
empty ring on a receiver that takes one byte an interrupt, a FIFO level
with the receiver's time-out for the tail below it, the line going idle
after a burst, a circular engine's half and full marks. Wherever the
silicon has an idle flag, a time-out or a level, the edge reaches the
consumer within two frame times of a burst's last stop bit - but on the
PL011, whose fixed time-out of 32 bit periods tells a burst's tail 3.2
frames after it; and a stream
with no silence is never held back - on a circular engine the half and
full marks deliver the edge before the producer laps the ring.
`harvest()` stays a public verb on an engined transport - the receive
errors read and counted, a stream a transfer error stopped bound again -
and the vectors call it; it is idempotent, so an owner that still calls
it changes nothing. Its edge test, made in an interrupt body over a
`HardwareRing`, asks `waiting()`, the one look that writes nothing
([ring.md](ring.md)).

**No byte is stolen by a clear.** While a receive engine owns the data
register, the CPU never reads that register to clear an error flag:
where the chapter clears a flag by a data-register read, the channel's
own read is that read, and where a byte can complete between the status
read and the data read the family's header names the race and bounds
it. So errors injected into a stream under an engine cost exactly the
bytes they hit: N bytes sent with K hit deliver N - K, in order, and the
counters say K - or, on the PL011, whose byte-wide beat drops an
entry's error flags, all N arrive, a break's zero among them, while the
error interrupts count K.

**`tx_idle()` means the wire is idle**, on every family: nothing queued
in the transmit ring, no block in flight on a transmit engine, and the
last stop bit off the pad - the chapter's transmission-complete flag,
never answering from a frame before the current block (an engine's
writes do not run the flag's software clear, so the transport clears it
when a block starts, as its chapter says). It turns true no earlier than
the last stop bit and within a bit time after it, so a caller that
switches a direction pin, a baud rate or a power state on it cuts no
frame.

**The interrupt receiver is the console's default.** A console reads
lines a human or a script types - a few bytes a burst, with silence
between - and the interrupt receiver, a byte or a FIFO level an entry,
is every family's receive path unless the program asks for more. A
receive engine with its edge is a STREAM's opt-in: a transport whose
bytes arrive faster than its interrupts can take them, or a program
that wants the CPU off the per-byte path, names an engine in the
transport's slot. `SerialPort` runs over either (below).

### Realizations

Common to all: fifteen verbs spelled identically - the run and the
byte both ways, `write_bulk`, `write_byte`, `read_span` with `consume`
and `read_byte`, then `init(clock, baud)`, `rx_pending`, `tx_idle` -
the wire's, on every family -, `actual_baud`, `rebase`, `clear_errors`
and the four counters `frame_errors`, `parity_errors`, `rx_overruns`,
`hw_overruns` - the two rings, and the edge contract above - and five
more: `set_baud(hz, baud)`, `release`, `can_baud`, `min_hz_for` and
`rx_skips`, every byte the receive ring will not deliver, never cleared
(SerialPort's epoch, below).
What differs is how the pins are named,
how many vectors the silicon gives the port, and what each family's
port has that the others' has not.

| stratum | realization | beyond the contract |
|---|---|---|
| avrdx | `Uart<n, Route, rx_size, tx_size>` (`avrdx/usart.hpp`) | the pins are a PORTMUX `Route`; THREE vectors, so the bodies are `rxc()` (the edge) and `dre()`; `rxc()` takes every frame the two-level buffer holds in one entry, the edge the burst's FIRST byte (chapter 27 has no idle flag, time-out or FIFO level); `tx_idle()` is TXCIF, which the task OWNS - `dre()` clears it behind a run's last byte, and a resource user who clears it makes `tx_idle()` false until the next run ends; `write_bulk()` copies with avr-libc's `memcpy` from two bytes up; `set_baud()` and `release()` as on the other two |
| samc21 | `Uart<n, UartPads, rx_size, tx_size, TxEngine>` (`samc21/sercom.hpp`) | the pins are SERCOM pads with their pins, one pin for both directions being the loop through the pad (31.6.3.8); ONE vector, `isr()` returns the edge and takes every level of the receive buffer an entry; ONE optional DMA engine slot, the TRANSMIT one (`NoDmaEngine` by default, compiling to nothing; `DmaTxEngine`, the family's one DMA user, claimed by one transport at a time - [the target's document](../samc21/dmac.md)) with `dma_isr()` and `dma_faults()`, the receiver always the interrupt receiver; `read_bulk()`, the receive run copied out; a transmit engine's block half the ring at most; `tx_idle()` is TXC, which every write to DATA clears, a channel's beat included; the run copy the runtime's `memcpy` from 20 bytes whose two ends share their word alignment, the byte loop otherwise |
| stm32g0 | `Uart<n, UartPins, rx_size, tx_size, TxEngine, RxEngine, opts>` = `UartTask<Usart<n>, ...>` (`stm32g0/usart.hpp`) | the pins carry their AF; one vector and `isr()`; the same engine slots and `read_bulk()`; `UartOptions` as one trailing parameter (FIFO thresholds, single wire, the receive engine's level `rx_priority` - very_high by default, [dma.md](dma.md), ...); the interrupt receiver PACED where the instance has the FIFO - RXFT at half its depth, one entry for four characters or more - with the burst's tail by the receiver time-out (RTOF, ten bit times) on a FULL USART and by IDLE on an LPUART (`rx_paced`, `rx_tail`); the receive engine's edge from the vectors, IDLE in `isr()` and the ring's half and full marks in `dma_isr()`, its errors entering the vector through EIE and PEIE and every flag cleared through ICR, the receive FIFO kept on under it with a KICK - DMAR dropped and raised when IDLE, an error or RXFT at seven eighths finds a character still standing, the cure for a FIFO that wedges behind a starved channel ([../stm32g0/usart.md](../stm32g0/usart.md)), `rx_kicks()` its count; `tx_idle()` is TC, cleared when a transmit block starts; the run copy the runtime's `memcpy` from 16 bytes whose ends share their word alignment; `kernel_hz()` (the kernel-clock multiplexer), `noise_errors()` (NE exists here alone), `wakes()` (the wake from Stop); the same task over an LPUART is `LpUart` (`stm32g0/lpuart.hpp`) |
| ch32v00x | `Uart<n, P, rx_size, tx_size, TxEngine, RxEngine, remap, opts>` = the task over `Usart<n>` (`ch32v00x/usart.hpp`) | the pins are the instance's own under a REMAP CODE (`afio.hpp`'s tables), the code a template parameter; one vector and `isr()`; the same engine slots with `dma_isr()`, `dma_faults()` and `harvest()`, on the channels table 8-2 gives the instance (the channel IS the request on this family, an engine elsewhere refused); USART2 refused at code 0, whose TX is the K8 package's reset pin; `UartOptions` as one trailing parameter (the frame, a single wire that is a BUS and not a loop on this silicon, the flow-control pair, the receive engine's level `rx_priority` - very_high by default, [dma.md](dma.md)); BRR is PCLK over the baud, whole - no separate fractional field and no kernel-clock multiplexer; the receive engine CIRCULAR over a `HardwareRing`, and under it the CPU never reads DATAR (the clear by a status read and the channel's next read, as the CH32V203's), the edge the vector's - IDLE or the first frame, and the ring's half and full marks; `tx_idle()` is TC, cleared at every transmit block's start |
| stm32f4 | `Uart<n, pins, rx_size, tx_size, TxEngine, RxEngine, opts>` (`stm32f4/usart.hpp`) | the classic SR/DR/BRR block (the CH32V00x's under ST's names): flags cleared by read sequences, the divisor in sixteenths or eighths (OVER8); the pads as `PinSel`s with the datasheet's AF; ten instances at most with a vector each, the U(S)ART name deciding FULL or not; the divisor from the instance's OWN APB clock (`apb_hz`), because the two buses run below HCLK; two OPTIONAL DMA engine slots (`stm32f4/dma.hpp`'s `DmaTxEngine`/`DmaRxEngine`) with `dma_isr()`, `dma_faults()` and `harvest()`, named by (controller, STREAM, channel) because a stream is the unit here and a peripheral reaches only the cells the request mapping gives it - checked at compile time against the reserve's per-part-class table and refused on a class whose manual was not read; under a receive engine the CPU NEVER READS DR - the vector's one status read and the stream's next read of DR make each clear, which costs no byte and suppresses the IDLE of a burst of one frame, so the vector takes RXNE once a burst beside the stream to learn the clear finished: two USART entries a burst, its edge IDLE or that frame and the lap's half and full marks; `tx_idle()` is TC, written 0 at every transmit block's start; the single-bit interrupt enables are bit-band stores ([../stm32f4/usart.md](../stm32f4/usart.md), [../stm32f4/dma.md](../stm32f4/dma.md)) |
| rp2040 | `Uart<n, pins, rx_size, tx_size, TxEngine, RxEngine>` (`rp2040/uart.hpp`), the family's alias of the IP stratum's `Pl011Transport` (`pl011/uart.hpp`) | the PL011 with its 32-deep FIFOs: the pins are GPIO numbers under function 2, and table 279 decides which of them carry which instance; one vector and `isr()`; `write_byte()` and `write_bulk()` WRITE AN IDLE FIFO DIRECTLY and arm the transmit interrupt only behind a full one, because the PL011's transmit interrupt is an edge that stays latched and not a level - so an idle transmitter takes 32 bytes with no interrupt, a long print takes one per FIFO level, and a refused byte writes nothing; the divisor's integer and fractional halves are latched by the LCR_H write that follows them; the same engine slots (`rp2040/dma.hpp`'s `DmaTxEngine`/`DmaRxEngine`, on any two distinct channels) with `dma_isr()`, `dma_faults()`, `harvest()`, `read_bulk()` - the receive run read off TRANS_COUNT by `harvest()`, the FIFO emptied and the credits cleared before a run, a run's completion onto a drained ring the engine's edge; the receive level's entry reads its sixteen entries without asking the flag register, the receive time-out (RT) the burst's tail; the four error interrupts armed and counted in `isr()` under an engine too; the run copy's crossover a member of the chip's traits (`copy_crossover`, 12 on both RP families) ([../rp2040/uart.md](../rp2040/uart.md)) |
| ch32vx03 | `Uart<n, P, rx_size, tx_size, format, TxEngine, RxEngine, remap, opts>` = the task over `Usart<n>` (`ch32vx03/usart.hpp`), n = 1..8 | the pins are the instance's own under a REMAP CODE (`afio.hpp`'s columns, the code a template parameter, a code of 0 writes no register at all, and a column on the debug port's pads is refused by `init()` while the probe's port is alive); the FRAME is a template parameter of its own here, ahead of the engines, where the CH32V00x carries it inside its options; one vector and `isr()`; the same engine slots with `dma_isr()`, `dma_faults()` and `harvest()`, on the SLOT - controller and channel - the request table gives the instance (UART4..UART8's are DMA2's on the CH32V303) (the channel IS the request on this family too, an engine elsewhere refused at compile time); `UartOptions` as the trailing parameter (a single wire that is a BUS and not a loop, the flow-control pair on a FULL instance, the receive engine's level `rx_priority` - very_high by default, [dma.md](dma.md)); the divisor is the instance's OWN bus over the baud, whole - USART1 asks PCLK2 and every other port PCLK1 - and WHICH instances a part offers, and which are FULL, is the datasheet's table and not a header's: the fourth port is a USART4 on the CH32V203C8 alone, so the smartcard and the synchronous clock exist there and are compile errors on a UART; the CH32V303's lot-keyed MARK/SPACE parity and short words are verbs that ask the die; under a receive engine the CPU never reads DATAR - a status read here arms a clear of every flag standing at the NEXT data read, the channel's, so no byte is taken and the IDLE of a one-frame burst is forgotten with it: the vector takes RXNE once a burst beside the channel, two USART entries a burst, its edge IDLE or that frame and the ring's half and full marks; `tx_idle()` is TC, cleared at every transmit block's start ([../ch32vx03/usart.md](../ch32vx03/usart.md)) |
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

**A line is posted only from a run released clean.** The lines
completed from a run are posted after the run's `consume()`, never
before. Where the receive ring's producer is a DMA channel
(`HardwareRing`, [ring.md](ring.md)) the channel can write over a run
while the drain reads it, and `consume()` answers `false`: then the
lines completed from that run are dropped, so is the line the
assemblers had begun when it came, both counted in `torn_lines()`, and
the drain skips the stream to its next end of line - the bytes before
it end a line whose beginning the ring skipped. A transport passes that
answer up by returning `consume()`'s `bool`; one whose `consume()`
returns nothing - every Ring-backed transport - has a release that
cannot refuse, and none of this is compiled for it: the byte loop and
its 14 instructions are the same either way. Validated on the host
(`test_serial_port`, a real `HardwareRing` over a scripted channel that
writes while a run is held).

**A gap between two runs is seen.** The stream a ring hands out can
jump. A `HardwareRing` skips when a look finds a lap unread - in the
drain's own `read_span()`, which then lends nothing, or in a `count()`
asked between two drains - and an interrupt receiver drops a byte that
finds its ring full, that the receiver flags as framed or failing its
parity, or that a hardware overrun swallowed. The run handed out after
such a gap is the stream after it, which the line begun in an assembler
would swallow as its continuation: a line delivered with a hole in it. So
every transport reports the gaps in its stream - `util/stream.hpp`'s
`SkippingSource`: `rx_skips()`, an epoch that moves whenever the stream
jumps, NEVER CLEARED, moved on those rare paths alone - and the drain
compares it at every run with the value it saw last, over every
transport that has it, whether its release can refuse or not. A change
ends the line begun as torn, counted in `torn_lines()`, and skips the
stream to its next end of line: the bytes before it end a line whose
beginning the gap took.

**A drop is skipped, not placed.** A count bumped where a byte is lost
does not say where: a byte a FULL ring refuses is lost behind everything
the ring holds, up to a whole ring ahead of the consumer, and the run in
hand when such a count moves was received before the loss - the drain
would tear the line it holds and deliver the cut one spliced (measured on
the host: `[LINE-TWO] [LINE-333] [FOURTAIL]`, the first line thrown away,
the splice clean). The consumer does not need the place, only the fact.
So an interrupt receiver's ring keeps a DROP EPOCH (`util/ring.hpp`'s
`SkipRing`, two bytes, [ring.md](ring.md)) that its handler moves on its
rare paths - a byte dropped for its flags, refused by a full ring, or
swallowed by an overrun, reported where the handler meets it - and the
consumer's look that finds it moved SKIPS: the tail jumps to the head,
everything queued is discarded with the drop, which is necessarily behind
the head, and the epoch moves - between two runs, never inside one, the
meaning `HardwareRing`'s skip has. No run handed out holds a drop, so no
line is ever delivered across one: the line begun is torn, and the
stream resumes at the next end of line or after a silence (below). What
the skip throws away beside the drop - the whole lines queued with it -
is lost only while bytes are being lost anyway. Under an overload so
steady that a drop falls between every two looks, nothing is handed out
at all ([ring.md](ring.md), "What a skip costs the stream"). A consumer
that looks between two losses is handed every byte between them (the
CH32V203's interrupt receiver, read between its breaks: 40 data bytes
with 10 breaks, 40 delivered, FE 10, ten skips; read only after them,
none of what the ring held). Under a receive engine the losses - an
overrun, a frame the channel does not take (the STM32G0's FE and PE,
DDRE clear), a stream restarted after a transfer error - are counted
where the vector or the look sees them, beside the ring's own skips: the
line torn is the one begun when the consumer next looks, which is the one
the loss cut when the consumer keeps up with its edges. The USB CDC class
has no `rx_skips()`: its receive ring cannot drop - an OUT packet is armed
only when the ring has its room - and over a transport without the verb
(a test capture, a simulated port) nothing of this is compiled.

**What the transport guarantees, and what it does not.** A LINE the
transport delivers is intact: no byte of it was dropped, refused or
flagged, and no two pieces of the stream were joined to make it. That is
all: the UART's flags do not see a bit flipped inside a frame whose stop
bit and parity come out right, and nothing in the receive path can. Data
that must arrive intact over a long or noisy link carries a CRC in its
own protocol; such a protocol need not read `rx_skips()` at all - a skip
is one more hole its CRC catches - and pays nothing for it.

**A silence ends the skip.** A drop can take the cut line's own end of
line, and the skip can land in the middle of a line, so the bytes after
it up to the next end of line are taken for the tail of a line whose
beginning was lost. But where the drain found the ring EMPTY after the
gap and the next run arrives 100 ms or more after that look
(`quiet_ticks`: longer than a USB serial adapter holds back a short
packet, shorter than a person or a script waits before the next
command), the line is taken to have ended in the silence and the run is
drained as a line's beginning - a burst that overflows the ring, silence,
then one command: the command is answered. What stays undecidable: a skip
with the next line close behind it (that next line is skipped as the cut
line's tail), a sender pausing longer than the silence in the middle of a
line after a drop took its end (its tail is taken for a line), and a drop
that lands on a line's start (the drain cannot tell it from a cut, and
drops that whole line with the fragment it expects).

The cost, counted at `-Os`: nothing a byte - the byte loop instruction
for instruction the same, 24 an ordinary byte on the STM32G0's console
(Cortex-M0+) as without the check - and on that console sixteen
instructions a run: `SkipRing`'s compare of its two bytes eight (an add
each for an offset past the 64-byte ring's slots), the epoch compare
four, the skip's flag four; `consume()` tests nothing. Everything after
a gap - the tear, the skip to the next end of line, its release - is
one out-of-line function, and so is the ring's own skip: inline, their
registers and their calls cost the Cortex-M0+'s byte loop two to three
instructions a byte. In the receive vectors the reports sit on the rare
paths: the per-byte loops of the STM32G0's and the RP2040's consoles are
the plain Ring's instruction for instruction (23 a character on the
STM32G0's FIFO drain, 20 on the PL011's level loop), and the AVR's RXC
vector saves no register for them (7 saved in the serial suite's image,
10 in the console's, whose binding flattens the kernel's post). A transport whose release can refuse and that does
not report its skips is refused at compile time
(`test/family_ch32x035/neg/`). Validated on the host (`test_serial_port`:
the scripted channel lapping a `HardwareRing` between two drains, the
skip made by the drain's own look and by one outside it; a `SkipRing`
dropping bytes between two runs, while a run is held, on a full ring
with lines queued before the drop, while both lines are in flight, and
the silence that ends a skip and the burst that does not; a random lossy
stream of numbered, checked lines in which every line delivered is a line
sent, whole and in order; `test_ring`'s `SkipRing` cases, a random lossy
stream among them in which every run handed out is consecutive and every
jump comes after a skip; `test_pl011`'s receive FIFO, the framed entry,
the full ring and the overrun).

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

## Numbers as text (`util/print.hpp`)

`print` formats on top of any `ByteSink` through conversions under
avr-libc's names and call shapes - `ltoa`, `ultoa`, `dtostrf`,
`dtostre` and the float twins `ftostrf`, `ftostre`. On the AVR they are
avr-libc's. Every other image links no C library
([runtime.md](runtime.md)), so there they are print.hpp's own, written
in C++ over nothing: the integers by a divide by ten in shifts and adds
(no division, no multiply - the same instructions on a core with
neither), a 64-bit integer without a 64-bit division, `hex()` at 32 bits
and at 64 (sixteen digits at most, no leading zeros, the letters
`ultoa`'s on each target).

**Off the AVR the floats are an exact binary split, not a float
routine.** A value travels as what it is - a float as a float, through
`ftostrf` and `ftostre`, a double as a double -; its bits are taken
apart and placed in a fixed-point number of 128 integer bits over 256
fraction bits, and every digit is a multiply or a divide by ten on
32-bit words - no floating-point operation, not even a float's widening
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
  wide as its 60-byte buffer, either alignment; `sci(v, precision)` and a
  bare float or double print signed (`+1.230e-03`), at most 7 decimals,
  avr-libc's own bound.
- The cost, counted on two consoles at `-Os`: one `fixed()` of a float
  adds 1068 bytes of text on the STM32G0's (Cortex-M0+) - the conversion
  inlined into its caller, the fraction's digit step, the divide by ten
  and the runtime's `memset` for the split's words - and 1208 on the
  CH32V003's (no multiplier); of a double 1168 and 1312; `fixed()` and
  `sci()` of a float together 1592 on the STM32G0.
