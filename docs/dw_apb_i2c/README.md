# IP stratum: the Synopsys DesignWare I2C (`dw_apb_i2c/`)

This page is for a programmer who uses an I2C bus on the RP2040 or the
RP2350, as a host or as a client, and wants to know what the driver
under it does, for whoever reads or changes that driver, and for
whoever brings up another chip that carries the same peripheral. It is both
the stratum's operational page and the document of its one header,
`brio/dw_apb_i2c/i2c.hpp`. A stratum is a directory of `brio/` that is
one architectural layer; this one is an IP stratum, named for a
peripheral design that several vendors license into their chips (an
"IP block") rather than for a chip, and written once for every family
that carries it.

An application never names this stratum. It writes the family's names,
which are aliases of the three templates here kept in the family's own
`i2c.hpp`, and the family supplies the chip's facts through a traits
type - a struct of types, constants and static functions the templates
read, so that they know the chip without including any of its headers:

| Who | Traits type | Where | The public names, aliases of the templates here |
|-----|-------------|-------|---------------------------------------------------|
| the RP2040 | `Rp2040DwApbI2c` | `brio/rp2040/i2c.hpp` | `DwApbI2c<n>` = `DwApbI2cBlock<Rp2040DwApbI2c, n>`; `I2cHost<n, pins, TxEngine, RxEngine, Placement>` = `DwApbI2cHost<Rp2040DwApbI2c, n, pins, ...>`; `I2cClient<n, pins>` = `DwApbI2cClient<Rp2040DwApbI2c, n, pins>` |
| the RP2350, both architectures | `Rp2350DwApbI2c` | `brio/rp2350/i2c.hpp` | the same three names over `Rp2350DwApbI2c` |
| the host suite | `SimDwApbI2c` | `brio/host/sim_dw_apb_i2c.hpp` | none: the suite names the templates itself |

On both chips the traits give two instances, 16-entry FIFOs and ic_clk
as `clk_sys`, the program's own clock rate (RP2040 4.3.14.1, RP2350
12.2.1.2). The smallest use, a host under the bus arbiter on the RP2040
as its own document spells it (an excerpt, the code that opens
[../rp2040/i2c.md](../rp2040/i2c.md)):

```cpp
constexpr brio::I2cPins pins{.scl = 13, .sda = 12};
using Bus = brio::I2cHost<0, pins>;
using I2c = brio::I2cBus<Bus, P, 4, brio::BusPassThrough, brio::ticks_from_ms<P>(20)>;

if (!Bus::init(clock)) {
    // no bus: a clock under 2.7 MHz, or the peripheral did not come up
}

Bus::Request r{};
r.addr = 0x48;
r.tx = brio::lend<brio::Lease::reply>(reg);   r.tx_len = 1;    // the register
r.rx = brio::lend<brio::Lease::reply>(value); r.rx_len = 2;    // then its value
r.speed = brio::I2cSpeed::fast_400k;
r.reply = brio::reply_to<Sensor, brio::I2cDone>();
brio::post<I2c>(r);

extern "C" void isr_i2c0() {
    if (Bus::isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
}
```

It assumes the program's platform `P`, its clock task's tag `clock`,
`static uint8_t reg[1]` and `static uint8_t value[2]`, and an active
object `Sensor` that handles `I2cDone`; `init()` runs in `main()` and the
post in one of `Sensor`'s handlers.

These words are used one way below, as on the design page. A **tenure** is
one START to its STOP, the bus held for one request; the **requester** is the
active object that posts it; a **client** is a device on the wire that answers
a host. `I2cHost` is the **host**, the task `I2cBus` drives. The **engines** are
the two DMA channels a host may be given, and a **DMA block** is one transfer of
one of them. `Request::engine` names what moves a request's bytes: `interrupt`,
the **pump** (the host's interrupt moving them), or `dma`, those channels.

Line by line: `pins` names the clock and data pads; `Bus` is the
family's `I2cHost` on instance 0; `I2c` is the arbiter, an
active object (a component with its own event queue, served one event
at a time by the kernel, here over the program's platform `P`) that
takes requests from every active object that uses the bus in arrival order, with room
for four waiting requests, retries no failure (`BusPassThrough`: every
outcome goes back to the requester as it is) and recovers the host
when a tenure has not answered within 20 ms (the per-bus timeout,
in kernel ticks, `i2c_timeout` its answer). A request names the 7-bit
address, lends one byte to write (the register number) and two bytes to
read back (a loan under
`Lease::reply`, kernel/borrowed.hpp: the arbiter and the host may use
them until the reply), the speed, and the active object the `I2cDone`
reply goes to; it is posted to the arbiter, and `post` copies the
request into the arbiter's queue, so `r` itself may go out of scope -
what it lends may not. The handler is the program's, under the name the
family's startup code gives I2C0's vector (`isr_i2c0` on both RP
chips): when the host's interrupt body returns true, the tenure
has just ended, and the handler posts the host's status to the
arbiter, which sends the reply.

What decides correctness even there:

- **The buffers a request lends stay untouched by the requester until its
  `I2cDone` arrives.** An `i2c_timeout` is such a reply: `recover()` stops the
  channels before the arbiter answers, so nothing still writes a lent buffer.
- **The vector is bound and posts `TransferDone`.** Left unbound, the line's first
  interrupt parks the core in the crt's default spin, where no timeout can answer.
  Bound but not posting, the arbiter never learns that a tenure ended, and its
  timeout answers.
- The arbiter is in the program's list of active objects.
- `init()` runs after the clock's, since the SCL counts are computed from the rate
  the clock type claims for ic_clk.

The pads carry the internal pull-up the chips'
chapters ask for, and the bus still needs its external resistors. The
20 ms must exceed the longest request the program makes at its speed: a
request of 255 bytes is 23 ms of wire at 100 kHz and 5.8 ms at 400 kHz. The
bus vocabulary itself (the Request, the
reply, the status codes) is [../design/i2c-bus.md](../design/i2c-bus.md)'s;
what an active object is and how a program lists them is the learning
route's ([../learn/introduction.md](../learn/introduction.md)). Every
tenure that moves the wire completes in an interrupt handler - the
I2C line's, or the DMA line's when an engine faults: the host has no
polled path. What moves nothing is answered at once, inside `start()`:
`i2c_rejected` for a speed the clock cannot produce, for an address
outside 0x08..0x77 that no host may name
([../design/i2c-bus.md](../design/i2c-bus.md)) and for a request naming
the DMA engines on a host that has none bound; `i2c_bus_error` for a
peripheral still not disabled after 100 000 reads of `IC_ENABLE_STATUS` when a
new address needs the disable, which a correct program does not meet (a
tenure's end leaves nothing that stalls the disable).

## What the stratum is

A directory under `brio/` named for a peripheral design instead of for a
silicon: the Synopsys DW_apb_i2c, a peripheral licensed by more than one
vendor and dropped into their chips unchanged, register for register. It
sits exactly where a core stratum sits - above `util/`, below the
families ([../design/overview.md](../design/overview.md), "A core stratum
sits between util/ and the families that share a core") - and it holds
the whole of the chapter that is Synopsys's: the resource over the
peripheral's programmer's model, the host `util/i2c_bus.hpp`'s `I2cBus`
drives, the client, their measured facts and their ISR bodies.

It exists under the discipline that governs the core stratum (the
stratum a processor core's families share, `cortexm/`): what more than
one family carries is factored out when a second one carries it,
with both copies in hand, and the extraction is gated by the images -
every release image of the family that had it first is byte-identical
before and after. What makes this peripheral worth a stratum of its own,
where a merely similar peripheral would not be, is that its register
description is not similar between the chips that carry it but
identical: the same names, the same offsets, the same bits, so there is
nothing to reconcile and nothing to gain by writing it twice.

## The document of record

The *DesignWare DW_apb_i2c Databook*, which is what a family's data sheet
cites when it says its I2C is a DW_apb_i2c. This project has not got that
databook, so nothing here is cited by its section: the facts below
are named by register, and each family's own document carries its data
sheet's chapter and verse for the same thing. A family's peculiarities -
how deep the FIFOs were synthesized, which pads a line reaches, which
interrupt controller carries the line, which rate of the clock tree is
ic_clk - are each chip's data sheet's to state, which is why they are
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
  `DwApbI2cBlock<Chip, n>`, the host `DwApbI2cHost<Chip, n, pins,
  ...>` with its two optional DMA engine parameters, its two ISR bodies and its
  hand-driven unstick, and `DwApbI2cClient<Chip, n, pins>` with
  `I2cClientEvent`. The host claims each engine's channel for this instance's
  request when its type is instantiated (`util/dma_owner.hpp`, through the DMA slot the
  family's engine publishes as `dma_slot`): a second owner on either channel, or this
  request on a second one, does not compile. An engine of a chip with no DMA
  controller behind it, the simulated peripheral's, publishes no DMA slot and claims nothing.
- Not here, by design: which pads carry the two lines and under which
  function, how the peripheral is taken out of reset, which interrupt
  controller owns the line and what it calls it, which rate of the
  family's clock tree is ic_clk, what a DMA request is called, how deep
  the FIFOs are, and how a pad is driven low by hand. Those are the
  concept, and each family answers them in its own header - where the
  public names also live, so an application never spells a chip-traits
  type.
- Not here either, and not anywhere: the bus request and its outcome
  codes. They are `util/i2c_bus.hpp`'s, shared verbatim by every stratum
  that drives an I2C ([../design/i2c-bus.md](../design/i2c-bus.md)); this
  file takes them and restates nothing.

## What a family owes: the `DwApbI2cChip` concept

One traits type, stated in the family's own `i2c.hpp` and checked there
with a `static_assert` (`Rp2040DwApbI2c` and `Rp2350DwApbI2c`; the
host's `SimDwApbI2c` is checked the same way in its own header, the
table at the top of this page). Every member is here because a line of
the driver needs it; nothing is here for symmetry.

The types:

| Member | Why it exists |
|--------|---------------|
| `Regs` | the registers of one instance, with the peripheral's own member names (`IC_CON`, `IC_DATA_CMD`, `IC_TX_ABRT_SOURCE`, ...) - the driver reads and writes them by name, so a family whose device description spells them otherwise hands over a struct of its own laid over the peripheral, which costs nothing. What the fields inside them are is not asked: the bit layout is the IP's and is stated here |
| `Irq` | what this family's interrupt controller calls a line: an enumerator on one target, a number on another |
| `Interrupts` | that controller, with `enable` / `disable` - the driver masks the line around a bring-up, a recover and the unstick, and never names an NVIC, because a family may carry one under one architecture and something else under another |
| `Pins` | the family's pin-set type, one pad per line - a pin table is a chip's, never an IP's |
| `DmaRequest` | what this family's DMA calls a peripheral request |
| `SpinRate` | the family's microsecond ruler as a value, computed once per clock change: the unstick drives the wire by hand and needs half a bit of standard mode between its edges |

The constants:

| Member | Why it exists |
|--------|---------------|
| `instances` | how many of the peripheral this family carries: the resource refuses a number past it |
| `fifo_depth` | the two FIFOs' depth, which is a synthesis parameter of the peripheral and therefore the family's fact, not this file's; the host puts both thresholds at its half |
| `byte_beat_alias` | the offset from a register's address at which a byte beat lands with the unused byte lanes zero, 0 where the chip's bus fabric has none: what decides whether a write phase can ride a transmit engine (below, "A write phase under `dma`") |

The per-instance facts, each a template on the instance number so the
answer is a compile-time constant - a register address, a line, a
request:

| Member | Why it exists |
|--------|---------------|
| `regs<i>()` | where instance i's registers sit |
| `irq<i>()` | its interrupt line |
| `reset<i>()`, `released<i>()`, `hold<i>()` | the peripheral from its reset state, whether it is out, and back into it: a reset controller on one family, a clock gate on another. A command under way with no STOP holds the peripheral enabled for ever: `IC_ENABLE.ABORT` ends such a tenure, and `recover()`, which also rewrites the configuration, is built on the family's reset |
| `tx_request<i>()`, `rx_request<i>()` | the two DMA requests the host hands its engines |

The verbs over a register, over the pads and over time:

| Member | Why it exists |
|--------|---------------|
| `set_bits(reg, bits)`, `clear_bits(reg, bits)` | one write that sets or clears bits with no read-modify-write: the enable, the abort and TX_CMD_BLOCK are single bits of one register, and the interrupt mask is touched from two contexts. A family with atomic register aliases uses them; one without does the read-modify-write under its own guard |
| `pins_valid(n, pins)` | is this pin set legal for this instance - the family's pin table, as a constexpr predicate the host's `static_assert` calls |
| `scl_pad(pins)`, `sda_pad(pins)` | which pad number carries each line |
| `Pad<pin>` | the pad as a type, with the verbs the driver calls on one: hand it over (`function(code, config)`), take it back (`release()`), and the five the unstick drives an open-drain line with by hand - `input(pull)`, `clear()`, `read()`, `drive_low()`, `release_drive()`. A line is driven by its output enable over an output register already holding a zero, which is what open drain is on every family that has no such mode |
| `pad_function` | the function code that routes an I2C to a pad, opaque to this file |
| `pad_config` | the electrical setup both lines want - the pull-up, the slew limit, the input hysteresis. It is the family's own constant handed over, not a value this file composes: the driver passes it exactly as the family's own driver would have |
| `open_drain_pull` | what a line released by hand idles at while the unstick owns it |
| `engines_distinct<Tx, Rx>()` | two engines of one host must not name one DMA channel; what "the same channel" means is the family's DMA's |
| `spin_rate(hz)`, `spin_us(rate, us)` | the ruler: the factor for a rate, recomputed at every clock change, and the wait itself |

And one member the concept names but cannot check, because it is a
template over the family's own clock types, of which this file knows
none:

| Member | Why it exists |
|--------|---------------|
| `ic_clk_hz(clock)` | which rate of this family's tree clocks the peripheral. It has a concept of its own, `DwApbI2cChipClock<Chip, Clock>`, checked by a `static_assert` inside each `init()` - the one place a clock is in hand |

## The include contract

`dw_apb_i2c/i2c.hpp` includes `kernel/borrowed.hpp`, `kernel/post.hpp`,
`util/clock.hpp`, `util/dma_owner.hpp`, `util/i2c_bus.hpp` and
`util/placement.hpp`, and
nothing else: no vendor
header, no family header, no device include. It may therefore be included
anywhere, in any order, and a family's `i2c.hpp` includes it beside its
own headers. Apps and family drivers keep including the family's header,
never this one - the family is where the public names are.

It follows that this directory needs no `.clangd` fragment of its own:
the framework default (`brio/.clangd`, the host test project's database)
is the right one, because this file really does compile on the host -
which is also what makes the host suite below possible.

## What stays per family

The pin table and its function code; the pad configuration; the reset or
clock gate; the interrupt line and its controller; the crt's handler name
an app binds; the DMA request numbers and the engine types the host's
parameters take; which rate is ic_clk; the FIFO depth; the microsecond ruler;
whether the family can run code from SRAM (its traits'
`places_fast_text`); and
the public names - `DwApbI2c<n>`, `I2cHost<n, pins, TxEngine, RxEngine, Placement>`
and `I2cClient<n, pins>`, the family's aliases of the three templates
here, with the family's own defaults for the empty engine parameters and the
placement. The traits' constant is true when the linker script copies the
`.fast_text` sections into SRAM before `main()`. The host's last
template parameter is util/placement.hpp's tag: under `InFlash`, the default,
`start()`, `isr()` and `dma_isr()` run inline where they are called, and under
`InRam` each is one out-of-line copy in `.fast_text.brio`, in SRAM
([../rp2040/platform.md](../rp2040/platform.md), "A handler in SRAM", has what
it buys). An
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

`brio/host/sim_dw_apb_i2c.hpp` is a second realization with no silicon
under it: the registers as an array in RAM (at the peripheral's offsets,
with the reset values a data sheet gives them, so a tenure's drain of the
receive FIFO terminates), a reset that fills it, an interrupt controller
that counts, pads over a modelled wire - an undriven line reads high
through its pull-up and SDA is held low for as many hand-driven SCL
pulses as a test asks - and a ruler that counts microseconds instead of
waiting. It is compiled twice: by the host suite `test_dw_apb_i2c`, and
by a family's compile check (`test/family_rp2040/dw_apb_i2c_ip.cpp`),
which names every verb of the resource, of the host and of the client
with both engine parameters empty and both filled. A driver that needed a
silicon would fail one of the two. The negative TUs beside it stage a
traits type with one member taken away and require the concept to refuse
it by name.

What the host suite judges is what a bench cannot see cheaply - the exact
words a bring-up leaves in the peripheral and the order of the acts that make
it, the entry a tenure ends with, the row of the timing table a speed
uses - and what a bench cannot stage at all: a peripheral that will not
disable (answered `i2c_bus_error` instead of waited on), an abort of
every reason, the engines' hand-over in the middle of a read phase, the
path each value of a request's `engine` takes (a short read on the
receive engine alone, a write that stays on the pump, a write on the transmit
engine of a chip with the alias, `dma` refused on a
host without engines), and a
client that releases SDA on the n-th pulse of the unstick.

What the fake does not model is the FIFOs: nothing moves an entry out of
`IC_DATA_CMD` and nothing fills the receive side by itself, so the
register holds the last entry the pump wrote. The two level registers are
plain memory a test writes, and the driver reads each once and then moves
that many entries - so a test stages half a FIFO of bytes by setting
`IC_RXFLR` and raising RX_FULL, and judges the batch, the refill, the
tail at the STOP and the masks the handler leaves, while the bytes' values
and the wire's timing stay the bench's to judge.

## The host: what the peripheral offers that bears on cost

Each item of the peripheral's programmer's model that decides what a tenure
costs, used or declined, with its reason; the measured figures are each
family's ([../rp2040/i2c.md](../rp2040/i2c.md),
[../rp2350/i2c.md](../rp2350/i2c.md)).

- **The command FIFO and its level (`IC_TXFLR`)** - used: the pump writes
  a run into the room the FIFO has, one level read and then stores, the
  request's fields held in registers for the run; no status read per
  entry. A tenure of a FIFO or less is written whole by `start()`.
- **The transmit threshold (`IC_TX_TL`, TX_EMPTY) with `TX_EMPTY_CTRL`** -
  used at half the FIFO: a long write refills eight entries an interrupt,
  and the eight still in the FIFO are what the wire spends while the next
  interrupt comes. TX_EMPTY is armed only while entries
  remain and, on a read, only while fewer than half a FIFO of read
  commands are out. **The handler masks it before it pushes** and arms it
  again only when room is still wanted. The reason: TX_EMPTY is a level
  of the FIFO, and an entry pushed into an empty FIFO that the peripheral
  takes at once lowers it and raises it again while the handler runs. With
  the source left armed during the push, the line is still pending when the
  handler returns and the handler is entered once more for nothing: on the
  RP2040 a third host entry reading IC_INTR_STAT zero, on
  `test_rp2040_i2c` letter g's second engined read after every boot. Reading
  the mask back after the store does not cure it. The cost: one
  store to mask it on an entry that finds it armed, and one to arm it again
  when room is still wanted.
- **The receive threshold (`IC_RX_TL`, RX_FULL) and its level
  (`IC_RXFLR`)** - used at half the FIFO, written once at `init()`: a
  long read takes eight bytes an interrupt (one level read, then loads)
  and pours the commands their room leaves in the same entry, so the
  bytes coming back pace the refill and a 255-byte read is 32
  interrupts, one for each eight bytes received and the STOP's. RX_FULL
  is masked once half a FIFO of bytes or fewer remains: the STOP's entry
  reads those last bytes. A deeper level (three quarters: TX_TL 4,
  RX_TL 11) was measured too, on the RP2040 - 21 interrupts for a write
  and 22 for a read of 255 bytes, about a sixth fewer handler cycles - and
  declined: its longest entry grows by a fifth, a latency every other vector
  waits out since no handler nests over another, and the byte times a late
  handler may take halve to four.
- **STOP_DET as every tenure's end** - used: a write, a read, the probe
  and an engined read all complete on their STOP (or on TX_ABRT), so the
  requester hears when the bus is free and the next `start()` finds the
  peripheral idle with nothing standing. A host's STOP_DET is raised for
  any STOP on the bus (`STOP_DET_IF_MASTER_ACTIVE` reads 0 on these
  chips), and a tenure written while another host holds the bus waits
  for that host's STOP - the multi-master rule - with its entries
  still queued. So a STOP ends a tenure only when one read of `IC_STATUS` at
  the STOP finds the transmit FIFO empty (TFE set) and the host state machine
  idle (MST_ACTIVITY clear), as a tenure's own STOP leaves them.

  Without the FIFO's test a two-byte write
  to an absent address, written inside a window another node held, ended
  `i2c_ok` on that window's STOP before it had clocked a bit (measured
  between the WeAct RP2040 and the WeAct RP2350B, each the other's peer).
  The machine's test is for a tenure of one entry: the probe,
  a one-byte read or write. The peripheral takes the waiting entry the moment
  the other node's STOP frees the bus, so a vector that comes a little late reads
  the FIFO empty while that entry's START is still to come. A probe is a read
  with no byte yet, so without that test the false end reads `i2c_bus_error`
  (measured on the RP2040 under the preemptive kernel, a probe parked behind a
  client holding SDA, at the hold's release: `IC_TXFLR` read 1 at the vector's
  entry and TFE set at the handler's read;
  [../rp2040/i2c.md](../rp2040/i2c.md)).

  An abort leaves both at rest too, since it flushes the FIFOs and idles
  the machine. So a tenure whose START waited for another node's STOP, and
  which loses the arbitration between the handler's read of `IC_INTR_STAT`
  and its read of `IC_STATUS`, would pass for ended on its own STOP. The
  handler therefore reads `IC_RAW_INTR_STAT` after `IC_STATUS`, and a
  standing TX_ABRT leaves the tenure to the abort's own entry. On the
  RP2040, a control build without that read ended an engined read inside a
  window held by hand `i2c_ok` with the host's code cold in the XIP cache
  (`test_rp2040_i2c` letter k run first after a boot, four boots of four), `i2c_ok`
  in that suite's whole key `z` (six runs of six, the cache's state there not
  counted), and `i2c_arb_lost` with that code just run (twelve of twelve). The test is one
  mask and one compare in the handler's STOP path, and one register read
  more where it passes (5 to 8 cycles measured on both chips).
- **`RX_FIFO_FULL_HLD_CTRL`** - used: a full receive FIFO holds SCL
  instead of overrunning, and an empty command FIFO with no STOP holds it
  too (RP2040 4.3.7, RP2350 12.2.7), so a handler that comes late stretches the
  bus and loses nothing. A preemption of `start()` does the same:
  `start()` writes its first entry before the rest, so the tenure is on
  the wire while the fill goes on. Held off after its k-th entry, the
  fill has left the peripheral k entries to clock behind the address: the
  address byte and k data bytes, (1 + k) byte times of nine SCL periods
  each, before it needs entry k + 1. A hold-off longer than that
  empties the command FIFO with no STOP in it and the peripheral holds SCL
  until the fill resumes - a stretch inside the tenure, the bytes
  unchanged. Two byte
  times are 45 us at 400 kHz and 18 us at 1 MHz, so a preempting dispatch
  of 3000 cycles (22 to 41 us with the kernel's way in and out, the longest
  preemption of `start()` measured on these chips)
  stretched no tenure at 400 kHz on either chip and, at 1 MHz, 32 in
  6000 writes on the RP2040, where the RP2350's preemptions all but never
  landed inside `start()` (measured under the preemptive kernel,
  [../rp2040/i2c.md](../rp2040/i2c.md) and
  [../rp2350/i2c.md](../rp2350/i2c.md)). The client takes it as a field
  (`hold_when_rx_full`, on by default); off, the peripheral still acknowledges
  every byte, keeps a FIFO's worth and loses the rest, RX_OVER raised
  once a byte lost (measured on both RP chips, their suites' letter m: 24
  bytes into a 16-deep FIFO nobody drained, the first 16 exact and eight
  `overrun` events; with the hold, the same write stretched until the
  vector drained and all 24 exact).
- **`IC_TAR` and `IC_CON` across tenures** - used: they take a write only
  with ENABLE clear, and they are kept across a disable and between
  tenures, so the disable/IC_TAR/enable cycle runs only when the address
  or the speed moves, or when the last tenure did not end on its own STOP
  (an abort, a recover). That cycle's disable waits for the peripheral's own
  shutdown, which follows the bus timing, and is paid in the dispatch: the
  whole cycle costs 6.0 to 7.7 us at 100 kHz and 1.4 us at 400 kHz on the RP2040,
  10 to 12 us and 1.5 or 1.6 us on the RP2350, measured on an idle bus over the
  images each family's page names.
- **`IC_TAR` written while enabled** - not offered: the register
  description of the chips here takes a write only with `IC_ENABLE[0]`
  clear, so a new address is always the disable/enable cycle above.
- **The mechanism a request names (`Request::engine`)** - the program's,
  never the host's: `I2cEngine::interrupt`, the default and what a
  Request that names nothing gets, runs the pump above at any length and
  touches no channel; `I2cEngine::dma` puts the read half on the engines
  at any length - the receive engine collects every byte, and the
  transmit engine pours the commands between the first and the last
  when there are any (a read of three bytes or more). A read of one or
  two bytes has none: the pump writes its one or two commands with the
  rest of the tenure and the receive engine alone runs. The probe stays
  on the pump under either value. `dma`
  on a host with no engines bound (`has_dma` false) is answered
  `i2c_rejected` inside `start()`, nothing on the wire, as an address
  outside the range is: the host's own refusal. The request names it in one field,
  `r.engine = brio::I2cEngine::dma;`, on a host whose two engine parameters
  are filled (each family's "How to use it" has the host type and the
  vector). On a host with engines a wrong choice costs core cycles or a
  sleep (each family's page, on the SLEEP state), never correctness. Each family's document gives the measured cost of both,
  and the length from which `dma` costs the core less:

  - a read on the RP2040: nine bytes with the host in SRAM; from the flash
    the same at 400 kHz and 1 MHz, later at 100 kHz, where the XIP cache
    spreads it;
  - a read on the RP2350: nine bytes on the Cortex-M33, eight on Hazard3;
  - a write: fourteen bytes on the RP2350's Cortex-M33, thirteen on its
    Hazard3, and never on the RP2040, whose writes stay on the pump.
- **A write phase under `dma`, the chip's question** - a write entry is
  the request's byte with the command bits above it clear, and a byte beat
  at the register's own address lands replicated across the word on both
  chips here, its copy in bit 8 a read command, in bit 9 a STOP and in bit
  10 a RESTART. So the traits' `byte_beat_alias` decides. Zero, the RP2040's
  (its fabric has no other behaviour, RP2040 2.1.4): the write half stays on
  the pump under either value and the request completes; a halfword staging
  copy the size of the request would carry it, and is declined: it spends a
  second buffer's RAM to save the pump's few per cent of the core
  during a long write ([../rp2040/i2c.md](../rp2040/i2c.md)). Not zero,
  the RP2350's +0x4000 (RP2350 2.1.5, measured there): the host's
  `dma_carries_writes` is true, a pure write of three bytes or more under
  `dma` takes the read half's shape - the pump writes its first entry,
  which starts the tenure, the transmit engine pours the bytes between in
  byte beats from the request's own buffer with no copy, and the DMA block's end
  on the DMA line hands the pump the last entry with its STOP. A write
  before a read stays on the pump whichever the chip, because the
  channel's DMA block in that tenure is the read's commands. It is usually a
  register index, a few bytes `start()` writes with no interrupt of their
  own; a longer one takes TX_EMPTY entries like any write on the pump. A
  write of one or two bytes stays there too, having no byte between its
  first and its last entry.

  One declaration and one address serve both widths. A program declares the
  transmit engine `uint16_t` and nothing else for writes: each DMA block's beat
  comes from the buffer it is handed. The channel is bound at the alias when
  `init()` or `recover()` arms it, and a read's halfword commands go through it
  too: they land with the same low sixteen bits, and zeros in the upper
  lanes, which the peripheral does not read. So no request writes the address. A
  request pays what any engine start pays: one compare against RAM, and one
  store of CTRL when the beat or the step differs from the last DMA block's. The
  figures, and the length from which
  the channel costs the core less, are the family's
  ([../rp2350/i2c.md](../rp2350/i2c.md)).
- **The DMA requests (`IC_DMA_CR`) and watermarks (`IC_DMA_TDLR`,
  `IC_DMA_RDLR`)** - used for the read phase under `dma`, and for a
  write phase where the chip carries one, watermarks at
  their reset value (single transfers, RP2040 4.3.15.3). Both
  engines arm at the family's normal priority, the rule for a peripheral
  that waits ([../design/dma.md](../design/dma.md)): a starved command channel leaves the
  transmit FIFO empty and a starved receive channel leaves the receive
  FIFO full, and the peripheral holds SCL either way
  (`RX_FIFO_FULL_HLD_CTRL`, set by `I2cConfig`'s default) - the bus
  slows, nothing is lost. The receive engine is armed to report a bus
  error alone, and the STOP ends its tenure. So a read alone under
  `dma` costs these interrupts, a write half before it adding the
  pump's own:
  - a read of one or two bytes: one, the STOP;
  - a read of three bytes or more: two - the transmit DMA block's end, which
    hands the pump the STOP entry, and the STOP;
  - the same, when that end finds no room in the FIFO for the
    entry: three, TX_EMPTY bringing it.

  A write on the transmit engine costs the same two or three, whatever its
  length.

  A bus error on either channel ends the tenure `i2c_dma_fault` in
  `dma_isr()`, the peripheral's abort asked and the channels stopped: the
  stratum's own status, beyond the four wire codes util/i2c_bus.hpp
  defines. An abort under the engines stops them before the abort is
  read away: the FIFOs stay flushed until `IC_CLR_TX_ABRT` is read, so a
  command the transmit engine still pours meanwhile is dropped instead
  of starting a tenure of its own.
- **`IC_ENABLE.ABORT`** - used where an engine fails; a NACK or a lost
  arbitration is the peripheral's own abort, read off `IC_TX_ABRT_SOURCE` once.
- **`IC_ENABLE.TX_CMD_BLOCK`** - declined for tenures: entries written
  ahead of a START are what the FIFO is for, and the fast path already
  writes them into an idle, enabled peripheral.
- **The multi-master rule** - used as the peripheral gives it: a host's
  START waits while the bus is busy (a START another node made, not yet
  closed by its STOP), so two hosts whose STARTs wait on one STOP leave
  together and the wired-AND decides; the loser's abort is ARB_LOST,
  `i2c_arb_lost`. Measured both ways between an RP2040 and an RP2350
  (each chip's document). A new address written while another node holds
  the bus pays a slower disable: 94 us at 100 kHz on the RP2040, where
  the whole cycle takes 6.0 to 7.7 us on an idle bus.
- **The interrupt decode** - one `IC_INTR_STAT` read an entry; the
  sources not armed for the tenure never reach the handler, and a mask
  bit is written only when it changes (a mirror of TX_EMPTY's and
  RX_FULL's, written by `start()` with every source masked and by the
  handler after).

## The client: what the peripheral does that the event stream must hide

- **No STOP_DET for a general call** under `STOP_DET_IFADDRESSED` (the
  register's own note: the interrupt follows a match of `IC_SAR` alone),
  which `I2cConfig` sets: a general call's end is not an event, its
  bytes are.
- **GEN_CALL stands through the address's acknowledge**: a read of
  `IC_CLR_GEN_CALL` while the bit stands does not clear it, and the line
  re-enters (a trace on the RP2040, in the 8 us after the address at 100 kHz).
  No suite re-runs it, since every brio host refuses
  address 0x00 ([../design/i2c-bus.md](../design/i2c-bus.md)), so how many times
  is not established.
  `service()` masks the source at the first report and gives it back at
  the first data byte: one `general_call` event a call. A foreign host's call
  with no data byte leaves it masked until this client's next received byte.
- **The peripheral holds SCL as a transmitter only**: RD_REQ holds it until
  the transmit FIFO has a byte; a receiver holds it only with its
  receive FIFO full. So a client's stretch happens on a read, and a byte
  already in the transmit FIFO when the host asks for it is taken with no RD_REQ.
- **A disable refuses the byte in flight**: "if the module is receiving,
  it stops the current transfer at the end of the current byte and does
  not acknowledge" (RP2040 4.3.17, `IC_ENABLE`), and
  `IC_ENABLE_STATUS.SLV_RX_DATA_LOST` says so afterwards. It is the only
  data NACK the peripheral can give mid-tenure: `IC_SLV_DATA_NACK_ONLY` takes
  a write with it disabled and its client idle; so a refusal of
  the n-th byte is a disable once byte n - 1 is taken (measured, the two
  RP suites' letter e). `SLV_DISABLED_WHILE_BUSY` is set beside it, and
  the far host's tenure ends on a data NACK (letter m: four bytes taken,
  the fifth refused).
- **A disable at a read request never completes**: the peripheral holds SCL
  until a byte is given, and disabled it holds its FIFOs erased, so
  the byte cannot come - `IC_ENABLE_STATUS.IC_EN` stays 1 (2 ms, then
  50 ms more with a byte written after the disable, untaken), neither
  status bit set, SCL held. `IC_ENABLE_STATUS`'s description names one
  transmit-side case, a disable during a transmitter's address byte, and not
  this one. `release()` ends it: its disable is bounded, and the reset
  lets SCL go (letter m: the far host's tenure then completes on the
  pull-up's 0xFF).

## Not covered yet

Driver gaps, each with its reason. They are the same in every family that
carries the peripheral, because they are its own; what each of them would
cost on a given chip - a wire, a peer board, a device - is in that
family's own document.

- The engined read's command cell in RAM: the transmit engine pours the
  plain read command from a constant, which on a chip that executes in
  place lives in the flash and is read through its cache once an entry.
  The PL022 host's dummy moved to RAM for that reason, measured
  ([../pl022/README.md](../pl022/README.md)); at an I2C rate - an entry
  every 22 us at 400 kHz - the contention was not measured, and the move
  costs every engined image RAM, so it waits for a measurement that shows
  the cost.
- The high-speed mode (3.4 Mbit/s, `IC_HS_SCL_*` and the master code):
  no chip this stratum has run on synthesizes it, and the speed
  vocabulary would grow a row that no silicon here can produce.
- SMBus and its two timeouts, the UDID and the alert: the register lists of
  the chips this stratum has met do not carry them,
  and no program over this driver has spoken SMBus.
- The stuck-bus timeouts (`IC_SCL_STUCK_*`, `IC_SDA_STUCK_*`) and the
  bus clear a peripheral that has them can run by itself: absent on the
  silicon here, which is why the
  unstick is by hand and why the arbiter's own timeout is what answers a
  client that holds the clock ([../design/i2c-bus.md](../design/i2c-bus.md)).
- The general call as a host verb: `start()` refuses address 0x00 with
  every reserved address ([../design/i2c-bus.md](../design/i2c-bus.md));
  a broadcast is a feature of its own, born with its first user.
- 10-bit addressing on the host side: the resource has the mode and
  `IC_TAR` takes ten bits; the Request's `addr` is 7 bits on every
  stratum, and a 10-bit device is the first user.
- The client on the DMA engines: the requests exist; a streaming client
  is born with a program that needs one.
