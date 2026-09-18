# IP stratum: the Synopsys DesignWare I2C (`dw_apb_i2c/`)

A directory under `brio/` named for a peripheral DESIGN instead of for a
silicon: the Synopsys DW_apb_i2c, a block licensed by more than one
vendor and dropped into their chips unchanged, register for register. It
sits exactly where a core stratum sits - above `util/`, below the
families ([../design/overview.md](../design/overview.md), "A core stratum
sits between util/ and the families that share a core") - and it holds
the whole of the chapter that is Synopsys's: the resource over the
block's programmer's model, the host engine `util/i2c_bus.hpp`'s `I2cBus`
drives, the client, their measured facts and their ISR bodies.

It exists under the discipline that governs the core stratum: what more
than one family carries is factored out when the SECOND one carries it,
with both copies in hand, and the extraction is gated by the images -
every release image of the family that had it first is byte-identical
before and after. What makes this block worth a stratum of its own,
where a merely similar peripheral would not be, is that its register
description is not similar between the chips that carry it but
IDENTICAL: the same names, the same offsets, the same bits, so there is
nothing to reconcile and nothing to gain by writing it twice.

## The document of record

The *DesignWare DW_apb_i2c Databook*, which is what a family's data sheet
cites when it says its I2C is a DW_apb_i2c. That databook is not on this
project's desk, so nothing here is cited by its section: the facts below
are named by REGISTER, and each family's own document carries its data
sheet's chapter and verse for the same thing. A family's peculiarities -
how deep the FIFOs were synthesized, which pads a line reaches, which
interrupt controller carries the line, which rate of the clock tree is
ic_clk - are not in that databook either, which is exactly why they are
the concept below.

## What lives here, and what does not

- Here: the speed vocabulary (`I2cSpeed`, `i2c_speed_hz`,
  `i2c_speed_code`, `i2c_min_clk_hz`, the specification's minimum high,
  low, hold and setup times per speed, `i2c_spike_ns`), the SCL
  arithmetic as pure functions (`I2cTiming`, `i2c_cycles_ceil`,
  `i2c_timing_for`, `i2c_scl_hz`), every register's bit layout as
  constants (`I2cControl`, `I2cData`, `I2cEnable`, `I2cEnableStatus`,
  `I2cFlag`, `I2cInterrupt`, `I2cAbort`, `I2cHoldField`,
  `I2cAddressField`, `I2cLevelField`, `I2cDmaControl`), the
  configuration and command vocabulary (`I2cRole`, `I2cConfig`,
  `i2c_con_of`, `I2cCmd`, `i2c_write_entry`, `i2c_read_entry`,
  `i2c_status_of_abort`, `i2c_dma_fault`), the resource
  `DwApbI2cBlock<Chip, n>`, the engine `DwApbI2cHost<Chip, n, pins,
  ...>` with its two optional DMA slots, its two ISR bodies and its
  hand-driven unstick, and `DwApbI2cClient<Chip, n, pins>` with
  `I2cClientEvent`.
- Not here, by design: which pads carry the two lines and under which
  function, how the block is taken out of reset, which interrupt
  controller owns the line and what it calls it, which rate of the
  family's clock tree is ic_clk, what a DMA request is called, how deep
  the FIFOs are, and how a pad is driven low by hand. Those are the
  concept, and each family answers them in its own header - where the
  PUBLIC NAMES also live, so an application never spells a chip-traits
  type.
- Not here either, and not anywhere: the bus REQUEST and its outcome
  codes. They are `util/i2c_bus.hpp`'s, shared verbatim by every stratum
  that drives an I2C ([../design/i2c-bus.md](../design/i2c-bus.md)); this
  file takes them and restates nothing.

## What a family owes: the `DwApbI2cChip` concept

One traits type, stated in the family's own `i2c.hpp` and checked there
with a `static_assert`. Every member is here because a line of the driver
needs it; nothing is here for symmetry.

The types:

| Member | Why it exists |
|--------|---------------|
| `Regs` | the register block of one instance, with the block's own member names (`IC_CON`, `IC_DATA_CMD`, `IC_TX_ABRT_SOURCE`, ...) - the driver reads and writes them by name, so a family whose device description spells them otherwise hands over a struct of its own laid over the block, which costs nothing. What the FIELDS inside them are is not asked: the bit layout is the IP's and is stated here |
| `Irq` | what this family's interrupt controller calls a line: an enumerator on one target, a number on another |
| `Interrupts` | that controller, with `enable` / `disable` - the driver masks the line around a bring-up, a recover and the unstick, and never names an NVIC, because a family may carry one under one architecture and something else under another |
| `Pins` | the family's pin-set type, one pad per line - a pin table is a chip's, never an IP's |
| `DmaRequest` | what this family's DMA calls a peripheral request |
| `SpinRate` | the family's microsecond ruler as a VALUE, computed once per clock change: the unstick drives the wire by hand and needs half a bit of standard mode between its edges |

The constants:

| Member | Why it exists |
|--------|---------------|
| `instances` | how many of the block this family carries: the resource refuses a number past it |
| `fifo_depth` | the two FIFOs' depth, which is a SYNTHESIS parameter of the block and therefore the family's fact, not this file's; the engine puts the transmit threshold at its half |

The per-instance facts, each a template on the instance number so the
answer is a compile-time constant - a register address, a line, a
request:

| Member | Why it exists |
|--------|---------------|
| `regs<i>()` | where instance i's block sits |
| `irq<i>()` | its interrupt line |
| `reset<i>()`, `released<i>()`, `hold<i>()` | the block from its reset state, whether it is out, and back into it: a reset controller on one family, a clock gate on another. A command under way with no STOP holds the block enabled for ever, so the family's reset is the engine's one way out and `recover()` is built on it |
| `tx_request<i>()`, `rx_request<i>()` | the two requests the engine hands its DMA slots |

The verbs over a register, over the pads and over time:

| Member | Why it exists |
|--------|---------------|
| `set_bits(reg, bits)`, `clear_bits(reg, bits)` | one write that sets or clears bits with no read-modify-write: the enable, the abort and the command block are single bits of one register, and the interrupt mask is touched from two contexts. A family with atomic register aliases uses them; one without does the read-modify-write under its own guard |
| `pins_valid(n, pins)` | is this pin set legal for this instance - the family's pin table, as a constexpr predicate the engine's `static_assert` calls |
| `scl_pad(pins)`, `sda_pad(pins)` | which pad number carries each line |
| `Pad<pin>` | the pad AS A TYPE, with the verbs the driver calls on one: hand it over (`function(code, config)`), take it back (`release()`), and the five the unstick drives an open-drain line with by hand - `input(pull)`, `clear()`, `read()`, `drive_low()`, `release_drive()`. A line is driven by its output ENABLE over an output register already holding a zero, which is what open drain is on every family that has no such mode |
| `pad_function` | the function code that routes an I2C to a pad, opaque to this file |
| `pad_config` | the electrical setup both lines want - the pull-up, the slew limit, the input hysteresis. It is the family's OWN constant handed over, not a value this file composes: the driver passes it exactly as the family's own driver would have |
| `open_drain_pull` | what a line released by hand idles at while the unstick owns it |
| `engines_distinct<Tx, Rx>()` | two engines of one host must not name one DMA channel; what "the same channel" means is the family's DMA's |
| `spin_rate(hz)`, `spin_us(rate, us)` | the ruler: the factor for a rate, recomputed at every clock change, and the wait itself |

And one member the concept names but cannot check, because it is a
template over the family's own clock types, of which this file knows
none:

| Member | Why it exists |
|--------|---------------|
| `ic_clk_hz(clock)` | WHICH rate of this family's tree clocks the block. It has a concept of its own, `DwApbI2cChipClock<Chip, Clock>`, checked by a `static_assert` inside each `init()` - the one place a clock is in hand |

## The include contract

`dw_apb_i2c/i2c.hpp` includes `kernel/borrowed.hpp`, `kernel/post.hpp`,
`util/clock.hpp` and `util/i2c_bus.hpp`, and nothing else: no vendor
header, no family header, no device include. It may therefore be included
anywhere, in any order, and a family's `i2c.hpp` includes it beside its
own headers. Apps and family drivers keep including the FAMILY's header,
never this one - the family is where the public names are.

It follows that this directory needs no `.clangd` fragment of its own:
the framework default (`brio/.clangd`, the host test project's database)
is the right one, because this file really does compile on the host -
which is also what makes the host suite below possible.

## What stays per family

The pin table and its function code; the pad configuration; the reset or
clock gate; the interrupt line and its controller; the crt's handler name
an app binds; the DMA request numbers and the engine types that fill the
slots; which rate is ic_clk; the FIFO depth; the microsecond ruler; and
THE PUBLIC NAMES - `DwApbI2c<n>`, `I2cHost<n, pins, TxEngine, RxEngine>`
and `I2cClient<n, pins>`, the family's aliases of the three templates
here, with the family's own defaults for the empty engine slots. An
application that says `brio::I2cHost<0, pins>` has no idea this stratum
exists, which is the point.

A family whose device description names the same bits holds the two
against each other where it defines its traits: this file spells every
bit itself because it may not read a vendor header, and a `static_assert`
per register is what keeps the two spellings one fact
([../rp2040/i2c.md](../rp2040/i2c.md)'s driver does it).

Each family's document is where its measurements live
([../rp2040/i2c.md](../rp2040/i2c.md) is the first).

## The proof that it knows no chip

`brio/host/sim_dw_apb_i2c.hpp` is a SECOND realization with no silicon
under it: the register block as an array in RAM (at the block's offsets,
with the reset values a data sheet gives them, so a tenure's drain of the
receive FIFO terminates), a reset that fills it, an interrupt controller
that counts, pads over a MODELLED WIRE - an undriven line reads high
through its pull-up and SDA is held low for as many hand-driven SCL
pulses as a test asks - and a ruler that counts microseconds instead of
waiting. It is compiled twice: by the host suite `test_dw_apb_i2c`, and
by a family's compile check (`test/family_rp2040/dw_apb_i2c_ip.cpp`),
which names every verb of the resource, of the engine and of the client
with both engine slots empty and both filled. A driver that needed a
silicon would fail one of the two. The negative TUs beside it stage a
traits type with one member taken away and require the concept to refuse
it by name.

What the host suite judges is what a bench cannot see cheaply - the exact
words a bring-up leaves in the block and the order of the acts that make
it, the entry a tenure ends with, the row of the timing table a speed
uses - and what a bench cannot stage at all: a block that will not
disable (answered `i2c_bus_error` instead of waited on), an abort of
every reason, the engines' hand-over in the middle of a read phase, and a
client that releases SDA on the n-th pulse of the unstick.

What the fake does NOT model is the FIFOs: nothing moves an entry out of
`IC_DATA_CMD` and nothing ever fills the receive side, so the transmit
path runs and the register holds the LAST entry the pump wrote, while the
receive path stays the bench's to judge.

## Not covered yet

Driver gaps, each with its reason. They are the same in every family that
carries the block, because they are the block's; what each of them would
COST on a given chip - a wire, a peer board, a device - is in that
family's own document.

- The high-speed mode (3.4 Mbit/s, `IC_HS_SCL_*` and the master code):
  no chip this stratum has run on synthesizes it, and the speed
  vocabulary would grow a row that no silicon here can produce.
- SMBus and its two timeouts, the UDID and the alert: the registers exist
  in the databook's full block, not in the chips this stratum has met,
  and no program over this driver has spoken SMBus.
- The stuck-bus timeouts (`IC_SCL_STUCK_*`, `IC_SDA_STUCK_*`) and the
  bus clear the blocks that have them can run by themselves: they are
  synthesis options, absent on the silicon here, which is why the
  unstick is by hand and why the arbiter's own timeout is what answers a
  client that holds the clock ([../design/i2c-bus.md](../design/i2c-bus.md)).
- 10-bit addressing on the host side: the resource has the mode and
  `IC_TAR` takes ten bits; the Request's `addr` is 7 bits on every
  stratum, and a 10-bit device is the first user.
- A write phase on the DMA engines: an entry is a command word with a
  flag, which a byte buffer cannot carry; a halfword staging copy the
  size of the request is declined until a program needs a bulk write on
  the engines.
- The client on the DMA engines: the requests exist; a streaming client
  is born with a program that needs one.
