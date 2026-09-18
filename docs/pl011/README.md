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
  error counters, its two optional DMA engine slots and its two ISR
  bodies.
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
| `Interrupts` | that controller, with `enable` / `disable` / `set_pending` - the driver never names an NVIC, because a family may carry one under one architecture and something else under another |
| `Guard` | the RAII critical section: the transport takes it where it shares the receive ring's producer side with the line's handler |
| `Platform` | the brio Platform the two byte rings are built on (`util/ring.hpp` asks it for the atomic width and for the same guard) |
| `Pins` | the family's pin-set type, one pad per direction - a pin table is a chip's, never an IP's |
| `DmaRequest` | what this family's DMA calls a peripheral request |

The constants:

| Member | Why it exists |
|--------|---------------|
| `instances` | how many of the block this family carries: the resource refuses a number past it |
| `fifo_depth` | the two FIFOs' depth, which is a SYNTHESIS parameter of the PL011 and therefore the family's fact, not this file's |

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

## The proof that it knows no chip

`brio/host/sim_pl011.hpp` is a SECOND realization with no silicon under
it: the register block as an array in RAM (at the PL011's offsets, with
the block's reset values, so a bring-up's drain of the receive FIFO
terminates), a reset that memsets it, an interrupt controller that
counts, pads that remember what they were handed to and when. It is
compiled twice - by the host suite `test_pl011`, which judges the exact
words a bring-up leaves in the block and the ORDER of the acts that make
it, and by a family's compile check (`test/family_rp2040/pl011_ip.cpp`),
which names every verb of the resource and of the transport with both
engine slots empty and both filled. A driver that needed a silicon would
fail one of the two. The negative TUs beside it stage a traits type with
one member taken away and require the concept to refuse it by name.

What the fake does NOT model is the FIFOs: nothing fills the receive
side, so the receive path has nothing to read there and stays the bench's
to judge.

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
