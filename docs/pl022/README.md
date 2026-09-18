# IP stratum: the ARM PrimeCell SSP (`pl022/`)

A directory under `brio/` named for a peripheral DESIGN instead of for a
silicon: the ARM PrimeCell Synchronous Serial Port (PL022), a block
licensed by more than one vendor and dropped into their chips unchanged,
register for register. It sits exactly where a core stratum sits - above
`util/`, below the families ([../design/overview.md](../design/overview.md),
"A core stratum sits between util/ and the families that share a core") -
and it holds the whole of the chapter that is ARM's: the resource over
the block's programmer's model, the host engine `SpiBus` drives and the
client surface, with their measured facts and their ISR bodies.

It exists under the discipline that governs the core stratum: what more
than one family carries is factored out when the SECOND one carries it,
with both copies in hand, and the extraction is gated by the images -
every release image of the family that had it first is byte-identical
before and after. What makes the PL022 worth a stratum of its own, where
a merely similar peripheral would not be, is that its register
description is not similar between the chips that carry it but
IDENTICAL: the same names, the same offsets, the same bits, so there is
nothing to reconcile and nothing to gain by writing it twice.

## The documents of record

ARM DDI 0194, the *PrimeCell Synchronous Serial Port (PL022) Technical
Reference Manual*, which is what a family's data sheet excerpts when it
says its SPI is a PL022 (the RP2040 data sheet does, in its 4.4, and
names the revision it carries: r1p4). That manual is not on this
project's desk, so nothing here is cited by its section: the facts below
are named by REGISTER, and each family's own document carries its data
sheet's chapter and verse for the same thing. A family's peculiarities -
how deep the FIFOs were synthesized, which pads a signal reaches, which
interrupt controller carries the line, what SOD does to a pad - are not
in that manual either, which is exactly why they are the concept below.

The bus VOCABULARY is not this stratum's and is not restated here: the
Request's arbitration, its reply and its status codes are
[../design/spi-bus.md](../design/spi-bus.md)'s, realized in
`util/spi_bus.hpp` over `util/bus_master.hpp`, and shared verbatim by
every family that drives an SPI. This file takes them.

## What lives here, and what does not

- Here: the mode and framing vocabulary (`SpiMode` with `spi_mode_cpol` /
  `spi_mode_cpha`, `SpiFormat`, `SpiDataSize` with `spi_data_bits`,
  `spi_frame_mask` and `spi_frame_is_halfword`, `SpiRole`), the
  prescaler pair as pure arithmetic (`SpiClock`, `SpiClocks`,
  `spi_sck_hz`, `spi_clock_for`), the whole configuration and the two
  control words it becomes (`SpiConfig`, `spi_config_valid`,
  `spi_cr0_of`, `spi_cr1_of`), every register's bit layout as constants
  (`SpiControl0`, `SpiControl1`, `SpiDataField`, `SpiFlag`,
  `SpiInterrupt`, `SpiDmaControl`), the engine's own status code
  `spi_dma_fault`, the resource `Pl022Ssp<Chip, n>`, the host engine
  `Pl022Host<Chip, n, pins, TxEngine, RxEngine>` with its Request, its
  two optional DMA engine slots and its two ISR bodies, and the client
  `Pl022Client<Chip, n, pins>`.
- Not here, by design: which pads carry the signals and under which
  function, how the block is taken out of reset, which interrupt
  controller owns the line and what it calls it, which rate of the
  family's clock tree is SSPCLK, what a DMA request is called, how deep
  the FIFOs are, what a pin driven at run time is, and what a
  microsecond busy-wait is made of. Those are the concept, and each
  family answers them in its own header - where the PUBLIC NAMES also
  live, so an application never spells a chip-traits type.

## What a family owes: the `Pl022Chip` concept

One traits type, stated in the family's own `spi.hpp` and checked there
with a `static_assert`. Every member is here because a line of the driver
needs it; nothing is here for symmetry.

The types:

| Member | Why it exists |
|--------|---------------|
| `Regs` | the register block of one instance, with the PL022's own member names (`SSPCR0`, `SSPCR1`, `SSPDR`, `SSPSR`, ...) - the driver reads and writes them by name, so a family whose device description spells them otherwise hands over a struct of its own laid over the block, which costs nothing. What the FIELDS inside them are is not asked: the bit layout is the IP's and is stated here |
| `Irq` | what this family's interrupt controller calls a line: an enumerator on one target, a number on another |
| `Interrupts` | that controller, with `enable` / `disable` - the driver never names an NVIC, because a family may carry one under one architecture and something else under another. It asks for no third verb: nothing here ever pends a line |
| `Pins` | the family's pin-set type, one pad per signal - a pin table is a chip's, never an IP's |
| `PinRef` | a pin named at RUN TIME, with `set()` and `clear()`: a Request's chip select and its D/C line are the APPLICATION's pins, chosen per transaction, and never the peripheral's own |
| `DmaRequest` | what this family's DMA calls a peripheral request |
| `DelayRate` | the family's microsecond busy-wait as the precomputed factor a caller stores once, so that no division runs while a chip select settles. The driver calls the verb that BELONGS TO THAT TYPE - written unqualified and found where the type is - which is how a busy-wait made of a core's own counter reaches a file that must know no core. The verb itself is the second concept below |

The constants:

| Member | Why it exists |
|--------|---------------|
| `instances` | how many of the block this family carries: the resource refuses a number past it |
| `fifo_depth` | the two FIFOs' depth, which is a SYNTHESIS parameter of the PL022 and therefore the family's fact, not this file's. It is the pump's window (how many frames may be in flight) and the client's `frames_ahead` |
| `no_pad` | what the family's pin set puts in a signal it does not route: a host with no receive pad is a write-only bus, and a client with no transmit pad is a listener |

The per-instance facts, each a template on the instance number so the
answer is a compile-time constant - a register address, a line, a
request:

| Member | Why it exists |
|--------|---------------|
| `regs<i>()` | where instance i's block sits |
| `irq<i>()` | its interrupt line |
| `reset<i>()`, `released<i>()`, `hold<i>()` | the block from its reset state, whether it is out, and back into it: a reset controller on one family, a clock gate on another |
| `tx_request<i>()`, `rx_request<i>()` | the two requests a host hands its engines |

The verbs over a register, over the pads and over time:

| Member | Why it exists |
|--------|---------------|
| `set_bits(reg, bits)`, `clear_bits(reg, bits)` | one write that sets or clears bits with no read-modify-write: `SSPCR1` carries the enable and the two live bits, and the interrupt mask is touched from two contexts. A family with atomic register aliases uses them; one without does the read-modify-write under its own guard |
| `pins_valid(n, pins)` | is this pin set legal for this instance - the family's pin table, as a constexpr predicate the host's and the client's `static_assert` call |
| `sck_pad(pins)`, `tx_pad(pins)`, `rx_pad(pins)`, `cs_pad(pins)` | which pad number carries each signal, or `no_pad` |
| `Pad<pin>` | the pad AS A TYPE, with the three verbs the driver calls on one: hand it over (`function(code, config)`), take it back (`release()`) and read its level (`read()`, which is how a client answers `selected()`) |
| `pad_function` | the function code that routes an SPI to a pad, opaque to this file |
| `sck_pad_config()`, `tx_pad_config()`, `host_rx_pad_config()`, `client_rx_pad_config()`, `cs_pad_config()` | the electrical setup each signal wants. They are FUNCTIONS returning a value rather than constants, so the driver materializes the configuration at the call, exactly as a family's own driver would have written it there - no object's address is taken and the call folds away. THE TWO RECEIVE PADS ARE ASKED SEPARATELY because they are two electrical situations and not one: a host's answer line may have nothing driving it (no client on the bus, or one listening dark), while a client's command line is driven by the host whenever it matters |
| `engines_distinct<Tx, Rx>()` | two engines of one host must not name one DMA channel; what "the same channel" means is the family's DMA's |
| `delay_rate(hz)` | the busy-wait's factor for a rate, recomputed at every clock change so the `cs_setup_us` of a Request costs no division |

And two things the concept above names but cannot check itself, each
with a concept of its own and a `static_assert` where the thing is in
hand - both of them on the host, which is the only party that needs
either:

| Member | Why it exists |
|--------|---------------|
| `sspclk_hz(clock)` | WHICH rate of this family's tree is SSPCLK - the rate the bit rate is divided from. It is a template over the family's own clock types, of which this file knows none, so it is `Pl022ChipClock<Chip, Clock>`, checked inside `init()` where a clock is in hand |
| `delay_us(rate, us)` | the verb over `DelayRate`. It is not a member of the traits type at all but a FREE function beside it, found by argument-dependent lookup on the type the traits hand over, so it is `Pl022ChipDelay<Chip>`, checked in the host's own body |

## The include contract

`pl022/spi.hpp` includes `kernel/borrowed.hpp`, `kernel/post.hpp`,
`util/clock.hpp` and `util/spi_bus.hpp`, and nothing else: no vendor
header, no family header, no device include, and in particular no core
stratum - a PL022 may sit on a core `brio/cortexm/` knows nothing about,
which is why the microsecond busy-wait arrives as a type of the family's
rather than as an include. It may therefore be included anywhere, in any
order, and a family's `spi.hpp` includes it beside its own headers.
Apps and family drivers keep including the FAMILY's header, never this
one - the family is where the public names are.

It follows that this directory needs no `.clangd` fragment of its own:
the framework default (`brio/.clangd`, the host test project's database)
is the right one, because this file really does compile on the host -
which is also what makes the host suite below possible.

## What stays per family

The pin table and its function code; the reset or clock gate; the
interrupt line and its controller; the crt's handler name an app binds;
the DMA request numbers and the engine types that fill the slots; which
rate is SSPCLK; the FIFO depth; the run-time pin type and the busy-wait;
what SOD does to a pad on that silicon; and THE PUBLIC NAMES -
`Pl022<n>`, `SpiHost<n, pins, TxEngine, RxEngine>` and `SpiClient<n,
pins>`, the family's aliases of the three templates here, with the
family's own defaults for the empty engine slots. An application that
says `brio::SpiHost<0, pins>` has no idea this stratum exists, which is
the point.

Each family's document is where its measurements live
([../rp2040/spi.md](../rp2040/spi.md) is the first).

## The proof that it knows no chip

`brio/host/sim_pl022.hpp` is a SECOND realization with no silicon under
it: the register block as an array in RAM (at the PL022's offsets, with
the block's reset values, so a bring-up's flush of the receive FIFO
terminates and a pump can write its first batch), a reset that memsets
it, an interrupt controller that counts, pads that remember what they
were handed to and when, a run-time pin that records which way it was
driven, and a busy-wait that COUNTS the microseconds it is asked for
instead of spending them. It is compiled twice - by the host suite
`test_pl022`, which judges the exact words a configuration leaves in the
block, the frames a pump wrote and the ORDER of the acts of a bring-up,
and by a family's compile check (`test/family_rp2040/pl022_ip.cpp`),
which names every verb of the resource, of the host with both engine
slots empty and both filled, and of the client. A driver that needed a
silicon would fail one of the two. The negative TUs beside it stage a
traits type with one member taken away and require the concept to refuse
it by name.

What the fake does NOT model is the FIFOs: a register block that is
plain memory cannot, since a store to `SSPDR` is a store and nothing
watches it. So nothing ever comes back, and the receive half of a
transfer - the pump's batching, the phase boundary, the frames on the
wire - stays the bench's to judge.

## Not covered yet

Driver gaps, each with its reason. They are the same in every family that
carries the block, because they are the block's; what each of them would
COST on a given chip - a wire, a device, a pad - is in that family's own
document.

- The block's own frame signal (`SSPFSSOUT`) as a host select: in
  Motorola format it PULSES BETWEEN FRAMES, which a device reading a
  multi-frame transaction inside one select window cannot take, so the
  Request carries a pin of the application's instead. A program that
  wants the pulsed select - a device whose every frame is its own
  transaction - is the first user.
- The National Semiconductor Microwire transaction: the framing is a
  code `SpiConfig` takes and the resource reads back, but the host
  engine's Request is full duplex and one frame wide, where Microwire is
  a control frame followed by the block's answer. A device with a
  Microwire interface is the first user.
- Frame widths other than 8 and 16 in the Request: the resource takes
  the whole 4..16 the block offers and a client is configured with any
  of them, but `SpiDataSize` is the two widths the other strata's
  Request speaks (`docs/design/spi-bus.md`). A device with a 12-bit
  frame is the first user.
- A client on the DMA engines: the slots are the host's, and a client
  answers from its own interrupt. A client that streams is born with a
  program that needs one.
