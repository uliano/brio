# IP stratum: the ARM PrimeCell UART (`pl011/`)

A directory under `brio/` named for a peripheral DESIGN instead of for a
silicon: the ARM PrimeCell UART (PL011), a block licensed by more than
one vendor and dropped into their chips unchanged, register for register.
It sits exactly where a core stratum sits - above `util/`, below the
families ([../design/overview.md](../design/overview.md), "A core stratum
sits between util/ and the families that share a core") - and it holds
the whole of the chapter that is ARM's: the resource over the block's
programmer's model and the interrupt-driven byte transport above it,
with their measured facts and their ISR body.

It exists under the discipline that governs the core stratum: what more
than one family carries is factored out when the SECOND one carries it,
with both copies in hand, and the extraction is gated by the images -
every release image of the family that had it first is byte-identical
before and after. What makes the PL011 worth a stratum of its own, where
a merely similar peripheral would not be, is that its register
description is not similar between the chips that carry it but
IDENTICAL: the same names, the same offsets, the same bits, so there is
nothing to reconcile and nothing to gain by writing it twice.

## The document of record

ARM DDI 0183, the *PrimeCell UART (PL011) Technical Reference Manual*,
which is what a family's data sheet cites when it says its UART is a
PL011 (the RP2040 data sheet does, in its 4.2). That manual is not on
this project's desk, so nothing here is cited by its section: the facts
below are named by REGISTER, and each family's own document carries its
data sheet's chapter and verse for the same thing. A family's
peculiarities - how deep the FIFOs were synthesized, which pads a signal
reaches, which interrupt controller carries the line - are not in that
manual either, which is exactly why they are the concept below.

## What lives here, and what does not

- Here: the frame vocabulary (`UartBits`, `UartParity`, `UartFormat`),
  the fractional baud divider as pure arithmetic (`UartDivisor`,
  `uart_divisor`, `uart_actual_baud`, `uart_min_hz`), every register's
  bit layout as constants (`UartFlag`, `UartInterrupt`, `UartDataError`,
  `UartReceiveStatus`, `UartControl`, `UartLineControl`,
  `UartTriggerField`, `UartBaudField`, `UartDmaControl`,
  `UartFifoLevel`), the resource `Pl011Uart<Chip, n>` and the task
  `Pl011Transport<Chip, n, pins, ...>` with its two ring buffers, its
  error counters (each saturating at its top: a count that wrapped would
  read as a few errors after a storm of them), its two optional DMA engine
  slots and its two ISR bodies.
- Not here, by design: which pads carry the signals and under which
  function, how the block is taken out of reset, which interrupt
  controller owns the line and what it calls it, which rate of the
  family's clock tree is UARTCLK, what a DMA request is called, how deep
  the FIFOs are. Those are the concept, and each family answers them in
  its own header - where the PUBLIC NAMES also live, so an application
  never spells a chip-traits type.

## What a family owes: the `Pl011Chip` concept

One traits type, stated in the family's own `uart.hpp` and checked there
with a `static_assert`. Every member is here because a line of the driver
needs it; nothing is here for symmetry.

The types:

| Member | Why it exists |
|--------|---------------|
| `Regs` | the register block of one instance, with the PL011's own member names (`UARTDR`, `UARTFR`, `UARTLCR_H`, ...) - the driver reads and writes them by name, so a family whose device description spells them otherwise hands over a struct of its own laid over the block, which costs nothing. What the FIELDS inside them are is not asked: the bit layout is the IP's and is stated here |
| `Irq` | what this family's interrupt controller calls a line: an enumerator on one target, a number on another |
| `Interrupts` | that controller, with `enable` / `disable` - the driver never names an NVIC, because a family may carry one under one architecture and something else under another; and it never raises the line by hand, so every entry is a cause the block's own status shows |
| `Guard` | the RAII critical section: the transport takes it where it shares the receive ring's producer side with the line's handler |
| `Platform` | the brio Platform the two byte rings are built on (`util/ring.hpp` asks it for the atomic width and for the same guard) |
| `Pins` | the family's pin-set type, one pad per direction - a pin table is a chip's, never an IP's |
| `DmaRequest` | what this family's DMA calls a peripheral request |

The constants:

| Member | Why it exists |
|--------|---------------|
| `instances` | how many of the block this family carries: the resource refuses a number past it |
| `fifo_depth` | the two FIFOs' depth, which is a SYNTHESIS parameter of the PL011 and therefore the family's fact, not this file's; the receive level the transport programs, half of it, is what an entry for the level reads blind |
| `copy_crossover` | the run length from which `write_bulk()` copies into the transmit ring with the runtime's `memcpy` (its two ends aligned alike) rather than its byte loop: a fact of the core and of the memory the code runs from, measured by the family's bench app (letter `u`) |

The per-instance facts, each a template on the instance number so the
answer is a compile-time constant - a register address, a line, a
request:

| Member | Why it exists |
|--------|---------------|
| `regs<i>()` | where instance i's block sits |
| `irq<i>()` | its interrupt line |
| `reset<i>()`, `released<i>()`, `hold<i>()` | the block from its reset state, whether it is out, and back into it: a reset controller on one family, a clock gate on another |
| `tx_request<i>()`, `rx_request<i>()` | the two requests a transport hands its engines |

The verbs over a register and over the pads:

| Member | Why it exists |
|--------|---------------|
| `set_bits(reg, bits)`, `clear_bits(reg, bits)` | one write that sets or clears bits with no read-modify-write: the interrupt mask is touched from two contexts, and the enable must leave the rest of `UARTCR` alone. A family with atomic register aliases uses them; one without does the read-modify-write under its own guard |
| `pins_valid(n, pins)` | is this pin set legal for this instance - the family's pin table, as a constexpr predicate the transport's `static_assert` calls |
| `tx_pad(pins)`, `rx_pad(pins)` | which pad number carries each direction |
| `Pad<pin>` | the pad AS A TYPE, with the two verbs the driver calls on one: hand it over (`function(code, config)`) and take it back (`release()`) |
| `pad_function` | the function code that routes a UART to a pad, opaque to this file |
| `tx_pad_config()`, `rx_pad_config()` | the electrical setup each direction wants. They are FUNCTIONS returning a value rather than constants, so the driver materializes the configuration at the call, exactly as a family's own driver would have written it there - no object's address is taken and the call folds away. The receive one carries a decision: where a pad comes up pulled DOWN, the receiver must find a pulled-up line, or its first act is to report a break |
| `engines_distinct<Tx, Rx>()` | two engines of one transport must not name one DMA channel; what "the same channel" means is the family's DMA's |

And one member the concept names but cannot check, because it is a
template over the family's own clock types, of which this file knows
none:

| Member | Why it exists |
|--------|---------------|
| `uartclk_hz(clock)` | WHICH rate of this family's tree is UARTCLK. It has a concept of its own, `Pl011ChipClock<Chip, Clock>`, checked by a `static_assert` inside `init()` - the one place a clock is in hand |

## The include contract

`pl011/uart.hpp` includes `kernel/platform.hpp`, `util/clock.hpp` and
`util/ring.hpp`, and nothing else: no vendor header, no family header,
no device include. It may therefore be included anywhere, in any order,
and a family's `uart.hpp` includes it beside its own headers. Apps and
family drivers keep including the FAMILY's header, never this one - the
family is where the public names are.

It follows that this directory needs no `.clangd` fragment of its own:
the framework default (`brio/.clangd`, the host test project's database)
is the right one, because this file really does compile on the host -
which is also what makes the host suite below possible.

## What stays per family

The pin table and its function code; the reset or clock gate; the
interrupt line and its controller; the crt's handler name an app binds;
the DMA request numbers and the engine types that fill the slots; which
rate is UARTCLK; the FIFO depth; and THE PUBLIC NAMES - `Pl011<n>` and
`Uart<n, pins, rx_size, tx_size, TxEngine, RxEngine>`, the family's
aliases of the two templates here, with the family's own defaults for
the ring sizes and the empty engine slots. An application that says
`brio::Uart<0, console_pins>` has no idea this stratum exists, which is
the point.

Each family's document is where its measurements live
([../rp2040/uart.md](../rp2040/uart.md) is the first).

## The transmit side: an edge that stays latched

ARM's text, which both Raspberry Pi data sheets reproduce word for word
(RP2040 4.2.6.3, RP2350 12.1.6.3, "UARTTXINTR"): the transmit interrupt
is asserted when the transmit FIFO is at or below its trigger level,
cleared by writing the FIFO above the level or by `UARTICR`, and "based
on a transition through a level, rather than on the level itself".
Measured on the RP2350 over the debug port
([../rp2350/uart.md](../rp2350/uart.md)): `UARTRIS.TXRIS` is SET when
the FIFO falls through the level, masked or not; it STAYS set until a
write takes the FIFO above the level or `UARTICR` clears it; and the
level alone never sets it again - a `TXIM` armed over a FIFO at or below
its level with `TXRIS` clear never fires.

The transport without a transmit engine is built on exactly that, in
three rules:

- A byte with nothing queued ahead of it and room in the FIFO is written
  STRAIGHT INTO `UARTDR` by the caller (`write_byte`, and `write_bulk` a
  run at a time): an idle transmitter takes a FIFO's depth with no
  interrupt.
- A byte behind a full FIFO, or behind bytes already queued, goes into
  the ring and ARMS `TXIM` - after the push, so a handler that empties
  the ring in between leaves at worst a mask armed over an empty ring.
  WHENEVER BYTES ARE QUEUED THE FIFO IS ABOVE ITS LEVEL OR `TXRIS` IS
  LATCHED: the first byte queues only behind a full FIFO, and from there
  the FIFO draining latches `TXRIS` while a write either stays above the
  level or leaves `TXRIS` as it was. The one store that could break it
  is a clear of `TXRIS` in `UARTICR`, and the transmit path writes none:
  the handler disarms `TXIM` when the ring runs dry and leaves `TXRIS`
  as it stands. So the handler runs once per FIFO level, on the fall
  through it, and refills the FIFO from the ring.
- A byte the full ring REFUSES writes nothing at all: the ring is not
  empty, so `TXIM` is armed and its edge is what drains the ring. A
  caller spinning on a full ring costs no interrupt.

The caller writes `UARTDR` only while the ring is empty, and a handler
runs to completion, so a ring the caller sees empty has every byte
queued before it in the FIFO already: the wire carries the bytes in the
order they were written. The trigger level is the transport's: an
eighth of the FIFO for the transmitter (so each entry refills
seven-eighths of it), half for the receiver.

`tx_idle()` is the wire's: the ring empty, no transmit block in flight,
and `UARTFR.BUSY` clear - which the block holds set "until the complete
byte, including all the stop bits, has been sent". Proven on the pad on
both families (their serial and DMA suites' letter `t`): `tx_idle()`
turns true within a microsecond of the last stop bit's end, timed off
the TX pad's own start bits.

## The receive side

The receive interrupt stands while the FIFO holds its trigger level or
more, and the receive TIME-OUT rises when the FIFO holds a character and
the line has been silent for 32 bit periods; both clear as the FIFO is
read. The interrupt receiver is built on exactly that: it drains the
FIFO whole on either, and AN ENTRY FOR THE LEVEL READS THE LEVEL BLIND -
half the FIFO's depth with no `UARTFR` read between, the handler being
the FIFO's one reader - because on this block a character's cost is its
peripheral loads, and the flag test made two of each one. One entry per
sixteen characters plus the time-out's, the tail of a burst delivered 32
bit periods - 3.2 frames - after its last stop bit: the block's own
time-out, which nothing programs.

EVERY LOSS IS REPORTED TO THE RING (util/ring.hpp's `SkipRing`), whose
consumer's next look skips everything queued, and `rx_skips()` counts
the skips, never cleared - util/serial_port.hpp's epoch, moving between
two runs: an entry dropped for its flags or refused by a full ring,
reported on its rare path where the drain meets it, and an overrun read
from `UARTRSR` after the drain - where in the drain a loss fell does not
matter, since the skip takes everything the ring holds. An overrun
standing at an entry fell with the FIFO full: the 32 entries the FIFO
kept are taken by the drain and skipped with the rest (measured: 48
frames into the masked FIFO, one OE, nothing handed out, one skip, and
the stream after it whole). Under a receive engine an overrun is
reported when the error vector sees it; the run in flight and the
FIFO's content, which precede the loss, are published after the
consumer's look (below, "Not covered yet").

THE OVERRUN UNDER AN ENGINE has a trap of the block's: its interrupt
rises from `UARTRSR`'s OE, which is STICKY - set "if data is received and
the receive FIFO is already full", cleared only by a write of the
register. Cleared through `UARTICR` alone, as the other errors are, the
first overrun of a life leaves OE standing and EVERY LATER ONE RAISES
NOTHING: measured on both families, three overrun episodes, one entry,
one count, and the losses after the first never reported to the ring - a
starved receive channel's gaps handed to the consumer joined. Cleared
with `UARTRSR` too, a FIFO that stays full under a consumer that does not
read raises it again with every frame that lands: 4058 entries for 5120
frames at 3 Mbaud. So the vector clears both and MASKS the source until
the next publish (`dma_isr()`'s completion or `harvest()`), which reports
any frame lost while it slept before waking it: one entry and one count
an episode (their DMA suites' letter o), and every loss before the
consumer's next look - which skips them all - reported.

UNDER A RECEIVE ENGINE NO VECTOR ENDS A BURST. The time-out needs a
character waiting in the FIFO, and the channel's single request moves
each one as it lands: measured on both families, 17 characters taken by
the engine and ten frames of silence after them leave `UARTRIS.RTRIS`
clear and the FIFO empty. A run that FILLS is published and re-armed by
its completion, whose `dma_isr()` answers the ring's edge - so a stream
that fills runs is told once a run - and a run still filling is read
off the engine's count by `harvest()`, the owner's verb. The engine
moves bytes for a stream an owner reads by count or by its own pace;
the burst path with an edge is the interrupt receiver, at one entry a
level. Neither a circular shape (the RP2350's endless transfer count
completes never, so it would take even the completion away) nor a chain
of two channels (half-ring marks, no burst end) would give the engine a
burst's end, and both are declined for that reason.

THE ERRORS UNDER AN ENGINE are the vector's: the channel moves an
entry's byte and not its flags (bits 8..11 of `UARTDR`), and `UARTRSR`
speaks for the last character READ - the channel's - so the transport
arms the four error interrupts with a receive engine and counts each
received error in `isr()`, clearing it through `UARTICR` (and an overrun
through `UARTRSR` as well, above). No clear reads `UARTDR`, so none takes
a byte the channel was owed; what a byte beat cannot do is drop the
errored entry's own byte - a break arrives as a zero among the data.
Measured on both families (their DMA suites' letter `v`, 64 slots of
which four a break, the consumer looking once a slot - the ring skips
what it holds at the look after a drop): through the interrupt receiver
60 bytes intact and in order, four BE; through the engine all 60 in
order with the four zeros among them, four BE.

## The engine slots

With a family's DMA engine in the transmit slot the handler never
touches the transmit FIFO: the transport queues into its ring and starts
a block over the ring's contiguous run, and the block's completion on the
DMA line releases exactly that run and starts the next. Two contexts
start blocks - the thread that queued and the completion - and THE
MASK COVERS THE CLAIM AND NOTHING ELSE: under the family's guard
`pump_tx()` takes the engine's `claim()`, its test-and-set; with the mask
down it reads the run and `launch()`es it, or gives the claim back with
`unclaim()` when the run is empty. The claim comes first because a
claimed engine has no block in flight, so nothing consumes the ring or
starts another block under the loader, and the run read after it never
holds bytes an ended block already sent. A receive engine fills the
ring's free run and is re-armed when it has ended, by the line's handler
or by `harvest()` under the guard.

What the transport asks of an engine, so a family's engines are written
to it: `present`, `channel`, `arm(data, request, high_priority)` - the
receive engine at the family's high level, its FIFO being the one that
overruns when the channel is starved, the transmit engine at the normal
one (docs/design/dma.md) -, `claim()`,
`unclaim()`, `launch(span)`, `complete()`, `busy()`, `service()` with
`flag_complete` and `flag_error`, `abandon()`, `stop()`; and of a
receive engine `start(pointer, count)`, `take()`, `idle()`,
`capacity()`. The RP2040's and the RP2350's engines are both written to
it ([../rp2350/dma.md](../rp2350/dma.md)).

## The proof that it knows no chip

`brio/host/sim_pl011.hpp` is a SECOND realization with no silicon under
it: the register block as an array in RAM (at the PL011's offsets, with
the block's reset values, so a bring-up's drain of the receive FIFO
terminates), a reset that memsets it, an interrupt controller that
keeps its enables, pads that remember what they were handed to and
when, and THE TRANSMIT FIFO with the rule above - entries, the flags
that follow them, `TXRIS` set on the fall through the level and cleared
by a write above it or by `UARTICR`, `UARTMIS` as `UARTRIS` under the
mask, and the wire as a verb the test calls. It is
compiled twice - by the host suite `test_pl011`, which judges the exact
words a bring-up leaves in the block and the ORDER of the acts that make
it, and by a family's compile check (`test/family_rp2040/pl011_ip.cpp`),
which names every verb of the resource and of the transport with both
engine slots empty and both filled. A driver that needed a silicon would
fail one of the two. The negative TUs beside it stage a traits type with
one member taken away and require the concept to refuse it by name.

The host suite plays the wire and the core over that FIFO: a print of
4096 bytes takes one handler entry per FIFO level - 145 to 146, the
first bytes straight in and then one entry per 28 - whether the caller
spins twice or a thousand times as fast as the wire, and none at all
while the wire keeps up with it; every byte reaches the wire in order,
and none is written into a full FIFO; and a receive run's completion
answers the ring's empty -> non-empty edge from `dma_isr()`, a transmit
completion none.

The fake models THE RECEIVE FIFO as far as the interrupt receiver reads
it: the wire lands a frame with its error bits as an entry, a read of
`UARTDR` takes the oldest, `RXFE`/`RXFF` and `RXRIS` (the receive level)
follow the entries, and a frame on a full FIFO is lost with `UARTRSR`'s
OE set - and `OERIS` raised at OE's ONSET alone, as the silicon raises
it; the time-out is raised by hand. Over it the suite drains a burst by
the level, and skips a framed entry, a full ring's refusals and an
overrun with everything queued beside them - nothing handed out joined
across a loss, the stream resuming after each; and under an engine it
checks the overrun's source asleep from the loss to the next publish,
the loss reported at that publish, the next one an entry of its own, and
the count held at 255 after three hundred. What it does
not model is time: the time-out's 32 bit periods, and the engine's view
of the FIFO, stay the bench's to judge.

## Not covered yet

Driver gaps, each with its reason. They are the same in every family
that carries the block, because they are the block's; what each of them
would COST on a given chip - a wire, a transceiver, a pad - is in that
family's own document.

- Hardware flow control (`UARTCR`'s RTSEn/CTSEn and the RTS/CTS pads,
  with the modem-status interrupts behind them): no program over this
  driver has needed it, and proving it takes two more wires on a link
  that has only two.
- IrDA (`UARTCR`'s SIREN and the low-power counter `UARTILPR`): an IrDA
  transceiver, which this project has not got.
- The modem status inputs (DSR, DCD, RI) and the outputs (DTR, OUT1,
  OUT2): their bits are named in `UartInterrupt` because `UARTIMSC`'s
  layout needs them, and no chip this stratum has run on brings any of
  them to a pad.
- The stick-parity bit (`UARTLCR_H`'s SPS): born with a first protocol
  that uses it - a nine-bit address mark is the usual one.
- An errored entry's byte under a receive engine: the channel's byte
  beat moves a framed, parity-failed or break entry's data byte into the
  ring with the good ones (its flags are counted, its byte is not
  dropped). A half-word beat would carry each entry's flags with its
  byte, into a ring of 16-bit entries the transport has not got - born
  with a program that needs the errored bytes out of the stream under
  an engine; the interrupt receiver drops them today.
- An overrun's place under a receive engine: the error vector reports
  the loss to the ring when it sees it, and the bytes before the loss
  still in the run in flight and in the FIFO are published after the
  consumer's look has skipped, so a line among them can reach the
  consumer joined to the stream after the loss. Exact would be the
  report deferred until the FIFO's depth has been moved - born with a
  program whose engine-fed lines must survive the channel's own stall;
  the interrupt receiver is exact.
