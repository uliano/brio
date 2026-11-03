# The I2C bus

This page decides how a program drives I2C devices through the same
arbiter as the SPI bus: one request is one bus tenure from START to
STOP (a write, a read, a write-then-read or a probe), the active object
that wants it (the requester) posts it and receives an `I2cDone` with a
status that says what the wire did, and the bus active object runs one
tenure at a time on the family's host. It also decides the arbiter
itself, `BusMaster`, which both buses share, and its per-bus timeout. It
is written for an embedded programmer who has read a register over I2C
with a vendor library and wants the same from several active objects
without collisions. [spi-bus.md](spi-bus.md) is the reference form of
the pattern; this page repeats nothing it says. Three words are used one
way throughout: the host is the chip's controller and the `I2cHost` type
that drives it, a client is a device on the wire that answers a host,
and the engines are the DMA channels bound to a host.

The smallest use has four parts. An `I2cHost<n, ...>` of the family. An
`I2cBus<Host, P>` in the list of active objects given to `Tenuto`, the
cooperative kernel (`P` is the target's platform type,
[kernel.md](kernel.md)).
The family's vectors bound to the host's handler bodies, posting
`TransferDone{Host::status()}` to the bus when a body answers true. And
a requester that posts a request and handles `I2cDone` in its own
dispatch. An excerpt, from a handler of an active object `MyAo`, with
`Host` and `Bus` the two types above:

```cpp
// Read a register: a write and a read joined by a repeated START.
// The two bytes are static: they are lent until I2cDone.
static uint8_t reg = 0x0F;
static uint8_t id = 0;
brio::post<Bus>(Host::Request{
    .addr = 0x1D,
    .tx = brio::lend<brio::Lease::reply>(&reg), .tx_len = 1,
    .rx = brio::lend<brio::Lease::reply>(&id),  .rx_len = 1,
    .reply = brio::reply_to<MyAo, brio::I2cDone>(),
    .speed = brio::I2cSpeed::fast_400k,
});
```

`lend` names the loan and `reply_to` the requester's return address
([kernel.md](kernel.md), "Payloads" and "Delivery"). Two things decide
correctness.
**The request's buffers are lent until `I2cDone` arrives**, and the
requester must not touch them before. And **the bare `I2cBus<Host, P>`
has no timeout**: a device that wedges the wire can stop that bus for
good, unless the program gives the bus a limit ("The per-bus timeout",
below).

## BusMaster: the arbiter generalized

The arbiter never looks at a byte. It owns the pending FIFO, the
reject-when-full policy, the `ReplyTo` return channel and the handshake
with the host (`Bus::start`, then `TransferDone`). Nothing in that is
specific to a bus, so one class serves both: `BusMaster<Bus, P,
pending_depth>` in `util/bus_master.hpp`, with `BusDone{status}`,
`bus_ok`, `bus_rejected` and `TransferDone` as its own vocabulary.

The per-bus names are aliases that cost nothing, on purpose:
`util/spi_bus.hpp` (`SpiBus`, `SpiDone`, `spi_ok`, `spi_rejected`) and
`util/i2c_bus.hpp` (`I2cBus`, `I2cDone`, `i2c_*`). A display driver
that reads `SpiDone` says which wire its bytes took; a `BusDone` in
the same place would be less information for no gain. The aliases are
type aliases and named constants, so they compile to no code of their
own.

Status codes are one byte, split by ownership: `bus_ok = 0` and
`bus_rejected = 1` are the arbiter's, `bus_timeout = 255` is the
timeout's, and every value from `bus_engine_status = 2` up belongs to
the host's vocabulary and travels untouched from `TransferDone` to
the requester's reply. The I2C host refuses three kinds of request
on its own: a speed the clock in force cannot produce, an address no
request may name, and, on a host that takes `Request::engine`, `dma`
with no engines bound (below). For each it answers `i2c_rejected`,
which is the arbiter's own `bus_rejected`, because the requester learns
the same thing from the host's refusal as from the arbiter's: nothing
was executed and no byte moved. SPI has no
wire-level outcome to report (no ACK, no arbitration), so the codes it
adds to the arbiter's are its hosts' own ([spi-bus.md](spi-bus.md)); I2C
adds the four an I2C master can observe, declared in
`util/i2c_bus.hpp`:

| Code | Meaning |
|------|---------|
| `i2c_nack_addr` | no ACK on the address: nobody home (the probe result an address scanner reads) |
| `i2c_nack_data` | no ACK on a written byte: the device refused or finished early |
| `i2c_arb_lost` | lost arbitration to another master: not our bus, no STOP sent |
| `i2c_bus_error` | a protocol violation on the wire, or a block `start()` found not its own: the tenure is over ("What a host does on the wire", "The transaction descriptor") |

A host with DMA engines adds a fifth, its own: `i2c_dma_fault`, the
value `bus_engine_status + 4`, when a DMA channel reports a transfer
error in the middle of a tenure. The host then ends the tenure and
releases the bus (a STOP, or on the DesignWare block an abort, which
sends one). `util/i2c_bus.hpp` does not declare it: each stratum whose
host has engine parameters defines it, with the same value, and the STM32F4's
second design names it `fmpi2c_dma_fault` (`stm32f4/fmpi2c.hpp`). The
AVR's host, which has no DMA, never produces it. A requester that
compares against it names its family's constant; a portable one treats
any code it does not know as a failure.

## The stack

```
     requester AO                       scanner / DAC driver
        \  post(Request{...})              /
         v                                v
     I2cBus<Bus, P>          util/i2c_bus.hpp   = BusMaster: arbitration + replies
         |  Bus::start(req)
         v
     I2cHost<n>              <stratum>/{twi,i2c}.hpp   START/Sr/STOP, ACK policy, the per-byte interrupt
         |
     ISR glue in the app     posts TransferDone{I2cHw::status()} on completion
```

Layering as for SPI: `I2cBus` and `BusMaster` are `util/`'s, pure and
tested on the host through the SPI alias in `test_spi_bus` (the same
class); the `I2cHost<n>` belongs to a target stratum, as a task
over that silicon's own resource. The application's interrupt handler
binds the vector.

### Realizations

Common to all but the simulated host: the `Request` field for field and
in this order (`addr`, `tx`, `tx_len`, `rx`, `rx_len`, `reply`, `speed`);
the speed's enumerators, under the type name `I2cSpeed` on every host
but the STM32F4's FMPI2C1, whose type is `FmpI2cSpeed`, so a request
naming `I2cSpeed` does not compile against that one host (the CH32V00x's,
the CH32V203's and CH32V303's, the CH32X035's and the STM32F4's classic
host stop at `fast_400k`, those peripherals having no Fm+); the
host's functions `init`, `start`, `isr`, `status`, `rebase`,
`recover`, `release`, `unstick`; the empty probe (both lengths zero)
served by every row, on the DesignWare block as a one-byte read, its
command FIFO having no address-only entry; `start()`'s refusal of an
address outside 0x08..0x77 (below); and the vocabulary `I2cDone` and
`i2c_*`, produced on the wire. Every silicon row's host is measured on
the bench against a partner its family's I2C page names: another board's
`twi_peer`, the chip's own other instance or client on a self-link, or
a device its board carries. What differs is the resource
under the task and the shape of the rate arithmetic each chapter
imposes. What each stratum adds is in its paragraph under ["The hosts,
family by family"](#the-hosts-family-by-family), at the end of this page;
the table holds the one fact that decides a reader's choice.

`addr` is a 7-bit address from 0x08 to 0x77, the range the I2C
specification leaves to devices (NXP UM10204 3.1.12), and every host's
`start()` answers any other value `i2c_rejected` at once, before
anything reaches the wire - one compare against the one definition of
the range, `i2c_address_in_range()` in `util/i2c_bus.hpp`. The sixteen
values outside it are reserved: 0x00 is the general call and, with the
read bit, the START byte; 0x01 is CBUS, 0x02 other bus formats, 0x03 a
future purpose and 0x04 to 0x07 the Hs-mode codes; 0x78 to 0x7B are the
first byte of a 10-bit address, and 0x7C to 0x7F the device ID and a
future purpose. The reason is the wire and not tidiness. The controller
of the STM32F1's event machine, which the STM32F4's classic I2C and the
three WCH families carry, has no setting for 7-bit or 10-bit addressing:
it goes by the byte it is told to send. A 7-bit address of 0x78 to 0x7B
with its write bit is a byte of the form 1111 0xx0, and that machine
sends it as a 10-bit header, where every other block sends it as a plain
7-bit address, so the same request would mean two different things on
two families. The general call is refused with the rest until it is a
feature of its own, and 10-bit addressing is offered by no host. Behind
the contract, each host of that lineage is also total in its address
phase: an event that phase does not serve, ADD10 above all, ends the
tenure with `i2c_bus_error` instead of re-entering the vector for ever
(measured on the STM32F446, the CH32V303, the CH32X035 and the
CH32V006 with the refusal compiled out, each family's I2C page).

| stratum | realization | what decides |
|---|---|---|
| avrxt | `I2cHost<n, route>` over `Twi<n>` (`avrxt/twi.hpp`), a resource that carries the client half too | no DMA controller, so no `engine` field |
| samc21 | `I2cHost<n, pads, generator, TxEngine>` over `I2cm<n>` (`samc21/i2c.hpp`; `I2cs<n>` is the client's resource, two over one SERCOM) | a transmit engine alone, inside the DMA controller's budget of two channels an image; a read is always the pump's |
| stm32g0 | `I2cHost<n, pins, TxEngine, RxEngine>` over `I2c<n>` (`stm32g0/i2c.hpp`, one TIMINGR word) | a byte counter and AUTOEND, no FIFO; two optional engines |
| stm32f4 | `I2cHost<n, pins, TxEngine, RxEngine>` over `I2c<n>` (`stm32f4/i2c.hpp`, the F1's event machine with the rise-time register the CH32V00x's has not) | `start()` waits for the last STOP; two speeds; a bus error never ends a tenure |
| stm32f4 (2) | `FmpI2cHost<n, pins, TxEngine, RxEngine>` over `FmpI2c<n>` (`stm32f4/fmpi2c.hpp`, the second I2C design of this family: the STM32G0's register file under the name FMPI2C1, on the F410, F412, F413/F423 and F446) | three speeds under `FmpI2cSpeed`; `start()` does not wait for the last STOP |
| ch32v00x | `I2cHost<1, pins, TxEngine, RxEngine>` over `I2c<1>` (`ch32v00x/i2c.hpp`, the F1's event machine with no rise-time register) | `start()` waits for the last STOP; two speeds; the channels fixed |
| ch32vx03 | `I2cHost<n, pins, TxEngine, RxEngine>` over `I2c<n>` (`ch32vx03/i2c.hpp`, the F1's event machine with a rise-time register, two instances on PB1) | `start()` waits for the last STOP; the bus clock refused outside 4..60 MHz; the channels fixed per instance |
| ch32x035 | `I2cHost<1, pins, TxEngine, RxEngine, Placement>` over `I2c<1>` (`ch32x035/i2c.hpp`, the F1's event machine with no rise-time register, one instance, every bus at HCLK) | `start()` waits for the last STOP; the block asked for at `init()`, some lots having none; no open-drain GPIO |
| rp2040 | `I2cHost<n, pins, TxEngine, RxEngine, Placement>` over `DwApbI2c<n>` (`rp2040/i2c.hpp`, the family's aliases of the IP stratum's templates in `dw_apb_i2c/i2c.hpp`) | a command FIFO, one interrupt per eight bytes; `dma` carries the read phase alone |
| rp2350 | `I2cHost<n, pins, TxEngine, RxEngine, Placement>` over `DwApbI2c<n>` (`rp2350/i2c.hpp`, the same IP stratum's templates) | as the RP2040, and `dma` also carries a pure write of three bytes or more |
| host | none | `I2cBus` = `BusMaster`, host-tested through the SPI alias (`test_spi_bus`, `test_bus_master`, the same class); the IP stratum's own driver against a DW_apb_i2c made of RAM (`test_dw_apb_i2c`, `host/sim_dw_apb_i2c.hpp`) |

None of the differences in those paragraphs is a spelling of the same
function: `actual_scl_hz` and `scl_hz` answer different questions (the
register in force against a cached speed), `bus_state()` and `idle()`
look at different things (the wire against the host), and `quick_command`
is a hardware mode the other families do not have (their empty probe
is the same tenure with no data, which the AVR serves too).
`speed_ok(speed)` is one function on every row that caches its rates.

The client side is a surface per position (`I2cClient` on each
stratum, the application's protocol over `Twi<n>`, `I2cs<n>`,
`I2c<n>`, `I2c<1>` or `DwApbI2c<n>`). Every stratum but the AVR speaks
one vocabulary back (`addressed`, `data_ready`, `stop_seen`,
`give`/`take`, `host_reads`; the CH32V00x, the CH32V203, the CH32X035,
the STM32F4 and the DesignWare families add `service()`, one
`I2cClientEvent` per interrupt body; the DesignWare block has no
`addressed`, raising no address-match event, so a tenure is known by its
first byte or its first read request, and the clock stretch of a read
request is the block's own). The AVR's is its own (`addressed`,
`data_ready`, `receive`/`respond`, `complete`). A convergence, not a task.

The peripherals share almost nothing below the contract: a TWI with
its own baud three-step, a SERCOM with a select-then-use register
file, an I2C with one timing word, the F1's event machine with a
clock register and no rise-time one, and a command FIFO whose entries
carry the bus conditions as bits. That one descriptor serves them all is
the point worth recording. Where two families carry the same block the
descriptor stops being a claim: the DesignWare's driver is one file
both of them alias ([the DesignWare I2C page](../dw_apb_i2c/README.md)).

## The transaction descriptor (`I2cHost<n>::Request`)

`addr`, `tx` and `tx_len`, `rx` and `rx_len`, `reply`, `speed`, and
`engine` in the padding after `speed` on a host that takes it: 10 bytes
on the AVR and 28 on a 32-bit target, over the kernel's guideline of
about 8 bytes for an event, legal as on SPI because the request is the
arbitration token. One request is one bus tenure from START to STOP, in
the shapes I2C devices actually use:

| Shape | Fields | On the wire |
|-------|--------|-------------|
| write | tx set, rx_len 0 | S addr+W data... P |
| read | tx_len 0, rx set | S addr+R data... P |
| write-then-read | both | S addr+W tx... **Sr** addr+R rx... P |
| probe | both empty | S addr+W P -> `i2c_ok` / `i2c_nack_addr` |

Write-then-read is the register-access idiom (index, repeated START,
value) and it must be one request: the repeated START is what keeps
another requester from slipping in between, so the SPI rule "the request
is the complete script of one bus tenure" holds word for word. The
probe is what a scanner sends and, unlike the SPI zero-length request,
it does touch the wire (its address phase is the transaction), so
`start()` is asynchronous on I2C whenever the wire moves: such a request
ends in a `TransferDone` from the host's vector, unless the wire wedges,
which is the per-bus timeout's case below. What completes inside
`start()` is what never reached the wire. The refusals: a request naming
a speed the clock in force cannot produce (`speed_ok(speed)` says so),
an address outside the range above, or `dma` with no engines bound is
answered `i2c_rejected` inside `start()`, through the host's `status()`,
with no byte moved; a tenure is never run at a rate nobody asked for and
never approximated in silence. And on the blocks where `start()` can
find the peripheral not its own, `i2c_bus_error` the same way: a system
operation that never settled on the SAM
C21, a block that would not disable on the two DesignWare families, and a
START still standing on the STM32G0, which only a program's own use of the
resource can leave. The
arbiter replies with `status()` on every synchronous
return (`util/bus_master.hpp`'s contract), so the requester sees these
in the same `I2cDone` as any outcome of the wire; the
completion policy does not judge them, and no retry would change a
refusal. Every host completes them the same way: status set, nothing
armed, true returned.

The `tx` and `rx` buffers are `Lease::reply` loans and say so in their
field types (`Borrowed`, [kernel.md](kernel.md), "Payloads"): the
requester keeps them alive and untouched until its `I2cDone` arrives.

Bus speed travels per request (`I2cSpeed::standard_100k`,
`fast_400k`, `fast_plus_1m`), like clock and mode on SPI: a shared bus
can carry a 100 kHz sensor and a 400 kHz DAC. A speed change costs an
enable cycle and a forced idle at `start()` (the AVR's MBAUD, for one,
may only be written with the host disabled), paid only when the speed
actually moves and never per byte. How MBAUD is solved, and why the
bus's rise and fall times are arguments rather than assumptions,
belongs to the host ([the AVR TWI page](../avrxt/twi.md)).

## Which engine moves a half (`Request::engine`)

A tenure has up to two data halves, the bytes it writes and the bytes it
reads. Everything else in it - the address, the repeated START, the end -
belongs to the host's interrupt on every block, and a host may move the
data the same way: the interrupt pump, one entry per byte or per FIFO
level, is present on every host. Where DMA channels are bound to the
host (its engines, each a template argument of the host), each half can
ride them instead, and the program names which, per request:

| `I2cEngine` | what it asks |
|---|---|
| `interrupt` | the pump, at any length; what a request that does not name the field gets |
| `dma` | the engines, at any length, for every half they can carry |

**The host never chooses.** No value hides a threshold set by cost: under
`dma`, a half the engines can carry goes on them at one byte as readily
as at 255. The lengths that stay off them are the silicon's, listed
below (a DesignWare write of one or two bytes; on the FMPI2C1 and the
STM32G0 a write of one starts no engine). The host could count only its own core cycles,
while the program knows what else must run, and whether the core should
sleep, while the wire does. A wrong choice costs core cycles or a sleep.
The one case where it costs correctness is a buffer no DMA master
reaches: under `dma`, a half in the core-coupled memory of an STM32F4 part
that has one should end in `i2c_dma_fault` where `interrupt` would succeed
([the STM32F4 DMA page](../stm32f4/dma.md); read from that page's
measured transfer error, not staged on I2C).

**A half the engines cannot carry completes on the interrupt**, under
`dma` too, and is not refused. Here I2C parts from SPI, whose host refuses
a request its engines cannot carry ([spi-bus.md](spi-bus.md), "Two ways to
run a request"): there the fallback would be the polled loop inside
`start()`, which holds the arbiter's dispatch for the request's whole
wire time, so whether the dispatch is held would depend on details such
as the memory a buffer sits in. On I2C the interrupt path is itself
asynchronous - `start()` returns false and the tenure ends on an
interrupt either way - so carrying a half on the pump changes the core's
cycles and nothing of the program's shape. Each family's document names
its own; the halves the engines leave to the pump are:

- on the CH32V303's DMA1, a half whose run crosses a 64 KB boundary, found
  before the half moves;
- on the STM32G0 and the STM32F4's FMPI2C1, a half with no buffer;
- on the DesignWare block, the read half's first and last commands, the
  probe, a write of one or two bytes and a write before a read, and on the
  RP2040 every write half, its bus fabric landing a byte beat replicated
  across the command register ([the DesignWare I2C
  page](../dw_apb_i2c/README.md), "A write phase under `dma`");
- on the SAM C21, the read half, a write of 255 bytes and a write with no
  buffer ([the SAM C21 I2C page](../samc21/i2c.md)).

A half with no buffer is a length with a null pointer. The `Request` does
not offer it (a buffer may be null only with a length of zero); the hosts
named above tolerate it, writing a filler or draining what they read.

One host asks a channel's start whether it took: the CH32V203's and
CH32V303's, because that is how the 64 KB boundary is found, a refusal
leaving the half to the event vector. Every other host starts the channel
without asking: it is the host's alone (the ownership rule, [dma.md](dma.md)).

**`dma` on a host with no engines bound is refused**, a request that
cannot be served as asked: `start()` returns true with `i2c_rejected` in
`status()`, nothing armed and nothing on the wire, the path the
out-of-range address and the unproducible speed take. Every host that
takes the field publishes `static constexpr bool has_dma`, so a program
that depends on the engines can say so in a `static_assert`.

**Which to name, by the cycles.** As guidance the host never reads, each
family's document states from which length the engines cost the core
fewer cycles than the pump, for a write and for a read: the smallest
length N with

```
E - F <= N x b
```

E is the engines' fixed cost to the core for that half: programming and
starting the channel, any interrupt its completion needs, and completing
the channel at the tenure's end. F is what the pump spends on the same
half beyond its work per byte. b is the pump's cost of a byte: its entry,
the byte, its exit. Only the difference E - F decides, so a family may
count the part of the tenure both paths share (`start()`'s common work,
the address phase) in both E and F or in neither, and its document says
which. A pump that takes no interrupt for a half's first byte saves one
b, and its family's inequality is E - F <= (N - 1) x b. The figures may
be measured, or estimated by counting cycles in the release listing,
whichever is cheaper on the family: the right order of magnitude decides
a count this small.

The wire's speed is not in that inequality. A pump whose service of a
byte outlasts that byte on the wire stretches the clock, which costs the
wire's time and no core cycle, so the tenure runs slower than the speed
asked. Where a family's document has measured it, it says whether this
happens at the family's top speed; a program that needs the rate names
`dma`. The measured count can still differ at 100 kHz, for reasons other
than the stretch: on the RP2040 from the flash, through the XIP cache's
misses on the engined path, and on the CH32V006's and the CH32X035's
written half by the run. Each family's document gives its count per speed.

What the cycles do not count is time awake. A platform whose idle cannot
sleep while a DMA channel works keeps the core awake for the whole half
under `dma`, and then the time awake is the engines' cost, far above any
count of cycles. The case is the CH32V003, the CH32V203 and CH32V303 and
the CH32X035, in whose Sleep the DMA controller moves nothing (measured,
each family's DMA page): their `idle()` asks the chip traits'
`bus_masters_working()` before it sleeps (`qingke/platform.hpp`) and
does not sleep while a channel works. The CH32V006 shares the CH32V003's
stratum but not its core: its QingKe V2C's Sleep lets the DMA work
(measured, [the CH32V00x DMA page](../ch32v00x/dma.md)), so there the
question is the constant false (`ch32v00x/parts/ch32v006.hpp`) and
`idle()` sleeps through a block. On the parts that do not sleep over a
channel, `interrupt` lets the core sleep between bytes, and `dma` is
how a program that wants the core free for other work during the half
asks for the engines anyway.

**Which hosts take the field**: every host with engine parameters - the
STM32G0's, both designs of the STM32F4, the CH32V00x's, the CH32V203's and
CH32V303's, the CH32X035's, the DesignWare host of the RP2040 and the
RP2350, and the SAM C21's - on every instantiation, engines bound or
not. The AVR's host does not carry it: its `has_dma` is false (the AVR
DA, DB and EA have no DMA controller), and the AVR lays a struct out
with no padding (every type aligned to one byte), so the field would
make every request a byte longer. The field is absent there, and a
request that names it does not compile
(`test/family_avrxt/neg/twi_request_names_engine.cpp`).

## What a host does on the wire

- On any NACK the host still ends the tenure with a STOP. The STM32G0 and
  the FMPI2C1 report the failure at the STOP's own edge (STOPF), so there
  the bus is released first. The F1 lineage, the AVR and the SAM C21's
  pump report when the STOP is requested, and the DesignWare block at
  TX_ABRT, so there the STOP may still be going out when the requester
  hears; the next `start()` copes (the F1 lineage waits for it, the AVR's
  silicon holds the START).
- Arbitration lost: the flag is cleared, no STOP (the other master
  owns the bus), `i2c_arb_lost` reported.
- Bus error: the flag is cleared, the tenure ends and `i2c_bus_error` is
  reported. What goes to the wire differs: the AVR forces its bus state
  to idle, the SAM C21 sends nothing, and the three WCH hosts request a
  STOP. On the DesignWare block the same code is any TX_ABRT reason other
  than a NACK or a lost arbitration. On the STM32G0 and the STM32F4's two
  blocks a controller's bus error is no evidence (an erratum's spurious
  flag): it is counted in `spurious_bus_errors()` and the tenure goes on.
- The ACK policy is the host's: each received byte is answered with
  ACK and receive-next, or NACK and STOP. Smart mode, where the
  silicon has it, is an option of the host (the acknowledge then
  rides the data read), invisible here.
- Pull-ups are external, their value the board's: the internal ones are
  far too weak for I2C edges. The RP2040's and RP2350's pads still take
  theirs, as their chapters' setup asks, and the FMPI2C1 offers its own
  as an option. On the AVR, the SAM C21 and the CH32X035 the peripheral
  drives the pins open-drain by itself and `init` only routes them; the
  STM32s, the CH32V00x, the CH32V203 and the CH32V303 configure their
  pads open-drain at `init`.

## The per-bus timeout

A client holding a line low for ever (SDA, or the clock in the middle
of a byte) leaves a tenure in flight: the kernel keeps running, since
nothing waits, but the bus active object stays busy and later requests
pile up until rejected. Loud, but not recovered. The silicon does not
close this. Only the SAM C21 ends a tenure on a client's held clock by
itself, and only with its time-outs armed; no silicon time-out watches a
held SDA, though some blocks answer a START into one. Measured, each row
from its family's I2C page, the arbiter's limit at 20 ms in those stagings,
a dash where this page has no staging to report:

| block | a client holding SCL | a START into a held SDA |
|---|---|---|
| [AVR TWI](../avrxt/twi.md) | no time-out at all | parks with BUSY standing |
| [SAM C21](../samc21/i2c.md) | SMBus time-outs, armed, bound it as 33.6.3.1 reads: a 40 ms stretch ends the tenure at 29 ms, `i2c_bus_error` | parks; after a forced IDLE (`recover()`, or the enable a speed change costs) it goes out and loses its address, `i2c_arb_lost`, or `i2c_bus_error` where BUSERR stands beside the loss |
| [STM32G0](../stm32g0/i2c.md) | the arbiter answers: the host's own unserved hold trips TIMEOUTA, a peer's 6 ms hold trips neither TIMEOUTA nor TIMEOUTB, though RM0444 32.4.12 reads otherwise ("if SCL is tied low for longer than ...") | parks with BUSY standing; after a PE cycle (a speed change, or `recover()`), which clears BUSY, it goes out and loses its address, `i2c_arb_lost` |
| [STM32F4's FMPI2C1](../stm32f4/fmpi2c.md) | the arbiter answers: TIMEOUTA armed at 25 ms reaches the error vector 25 ms into a partner's 40 ms hold, the host sweeps it and the tenure goes on | as the STM32G0's |
| [STM32F4's classic I2C](../stm32f4/i2c.md) | the arbiter answers a 40 ms hold | parks at the speed in force and after a speed change alike, a PE cycle not forgetting the held line; the arbiter answers both |
| [CH32V00x](../ch32v00x/i2c.md) | no clock-low time-out: a 45 ms hold stands until the arbiter answers | comes back ARLO at once, `i2c_arb_lost` well before any limit |
| [CH32V203, CH32V303](../ch32vx03/i2c.md) | no clock-low time-out outside SMBus mode: a 45 ms hold stands until the arbiter answers | the first goes out at the controller's tick and loses, `i2c_arb_lost` in under 0.4 ms; one set after that loss inside the same hold parks until the arbiter answers |
| [CH32X035](../ch32x035/i2c.md) | a 45 ms hold stands until the arbiter answers | goes out and loses at the address's first one bit, `i2c_arb_lost`; a block brought up over a line already held keeps its START pending until the arbiter answers |
| [RP2040](../rp2040/i2c.md), [RP2350](../rp2350/i2c.md) | no clock-low time-out: a hold through a read request stands until the arbiter answers | - |

The right-hand column on the STM32G0, the STM32F4's two blocks, the SAM
C21 and the CH32V203 and CH32V303 was measured under the preemptive
kernel. The STM32G0's and the FMPI2C1's left-hand cells disagree over one
register file, on different stagings (a 4 ms limit and a 6 ms hold, a
25 ms limit and a 40 ms hold at the address). What separates them is not
established, and on both the tenure stands until the arbiter answers.

So one held SDA has two honest replies, parked and timed out on some
strata and refused by the silicon on others. A requester reads both as
"the bus is not mine", and the recovery ladder ("Not built, noted")
stays the application's. So the timeout is the arbiter's: the one object
that knows a completion is owed, living in the kernel that has time
events.

`BusMaster`'s `timeout_ticks` template argument (surfaced here as
`I2cBus`'s) arms a one-shot `TimeEvent` for every tenure that goes
asynchronous. If it matures first, the host is declared dead:
`Bus::recover()` puts the peripheral back where `start()` is legal
(the AVR host's enable-cycle errata workaround, the SAM's cached
re-init, the G0's PE cycle, which RM0444 32.4.9 offers for a deadlock at
the START, "restores the normal operation ... by toggling the PE bit",
the RP2040's block reset through the reset controller, because a host
holding SCL with no STOP in sight never disables); the requester is
answered `i2c_timeout` in its place, and the queue moves on. The races
with the real completion are closed by construction (a sequence number
and a drain state; the whole story is in `util/bus_master.hpp`, staged
deterministically in the host suite).

Three decisions worth their line:

- **Per bus, not per request.** One wedged device starves every requester
  of the wire, so the limit is a property of the bus. It must sit
  above legal clock stretching, which is flow control and not a fault,
  so size it to a whole worst-case tenure at the slowest device, and
  convert with `ticks_from_ms<P>()`. A 255-byte request at 100 kHz is
  23 ms of wire: the 20 ms of the families' examples and of the stagings
  above covers shorter tenures only.
- **The wire stays the application's.** The timeout guarantees only
  that the bus active object and its queue survive to be asked;
  whether to `unstick()`, power-cycle a client or re-probe the bus is
  the recovery ladder below, still policy. After a fault, re-verifying
  the clients one by one is the application's decision.
- **A timeout is not a completion.** The retry `Policy` is never
  consulted, since the host never spoke.

With `timeout_ticks = 0` (the default) none of this exists: the
generated code is byte-identical to the untimed arbiter's.

## Not built, noted

The arbiter carries the hook a recovery policy would hang from:
`BusMaster`'s `Policy` template argument, whose `on_done(status,
attempt)` can ask for the same request to be started again
(`util/bus_master.hpp`). The concrete I2C ladder below and the
multi-host backoff remain on demand.

- **The recovery ladder.** The mechanical functions exist at host
  level (`unstick()` clocks SCL up to nine times until a stuck client
  releases SDA, [the AVR TWI page](../avrxt/twi.md)) and the timeout
  above notices the wedge and reports it; the policy connecting them,
  when to unstick, when to retry, when to take the bus out of service,
  is the application's, or a future policy type's, born with the
  device that needs it.
- **Multi-host policy.** The host reports `i2c_arb_lost` and the
  arbiter passes it to the requester untouched. A bus active object
  that knows it shares the wire (retrying a lost tenure, backing off,
  refusing to start while another host is in the middle of a
  transaction) is not designed.
- 10-bit addressing: the arbiter's descriptor has no shape for it.
- A DAC or sensor active object with word semantics:
  `util/wire.hpp` already gives the big-endian load and store; the
  MCP47CVB22 driver is the first candidate.

Device-specific facts (the DAC's address strapping, the pull-up value)
live in the page of the board that carries the device
([docs/boards/](../boards/README.md)) and in the header comment of the
application that talks to it, never here.

## The hosts, family by family

What each host adds beyond the contract; each family's I2C page has the
detail. A pump is the host's interrupt moving data byte by byte. Smart
mode is the block's acknowledge riding the data read. A row that caches
its rates takes the rate its block is clocked from at `init` and answers
`speed_ok(speed)` and `scl_hz(speed)`.

**AVR (`avrxt`).** The rate readback is `actual_scl_hz(t_rise_ns)`, what
the register in force gives under the chapter's rise-time budget, with
`speed()`, `baud()` and `speed_ok()` for the speed in force.
`quick_command(bool)` is the TWI's own hardware mode (QCEN: every request
an address-only frame). `bus_state()` is the wire's state from the bus
monitor. The handler calls nothing, since one call in an AVR vector saves
the whole call-clobbered set. Smart mode is the default (`Options::smart`).
The client's SDASETUP is eight clocks in every default, four losing a smart
client in a long read on the AVR EA ([the AVR TWI page](../avrxt/twi.md)).

**SAM C21 (`samc21`).** `init` takes the stated core rate
(`reference_hz()`), with `baud_of(speed)`. `idle()` is the host's phase,
not the wire's. Smart mode is always on. The one engine slot takes a
transmit engine (a receive engine there does not compile), with
`dma_isr()`, `Request::engine` and `has_dma`. Under `dma` a write of 1 to
254 bytes rides the channel on three interrupts whatever its length; the
read half, a write of 255 bytes (LEN is eight bits, 33.10.9) and a write
with no buffer stay on the pump, one interrupt a byte. The channel is one
of the family's budget of two an image, checked at compile time, and the
DMAC's vector is the program's to bind ([the SAM C21 DMAC
page](../samc21/dmac.md)). `configuration(speed)` is for code that
reconfigures the peripheral behind the host.

**STM32G0 (`stm32g0`).** `timing_of` and `kernel_hz()` for the kernel-clock
multiplexer; two optional engine slots with `dma_isr()`; `fast_plus_drive()`
(SYSCFG's Fm+ pad drive); `spurious_bus_errors()` (an erratum's counter);
`idle()` as the SAM's. The byte counter with AUTOEND carries the tenure, a
write half's first byte preloaded into TXDR before the START on either engine
(ES0418 2.11.6's first workaround on the STM32G071, one path in and out of its
band, a write of one starting no channel), a read's last byte
taken at STOPF, a NACK named at STOPF with no NACKIE armed (the address's
when TXE is clear and nothing refilled TXDR, the byte pump's position or the
transmit channel's count saying so). A tenure's interrupt and DMA
enables go in one CR1 store each way (`enables()`), so an engined tenure
leaves no enable behind. `dma` carries a half at any length, a half with no
buffer staying on the pump.

**The F1 lineage: the STM32F4's classic host and the three WCH hosts.**
These four share an event machine and its consequences:

- Two vectors, `isr()` for the events and `error_isr()`; `timing_of` and
  `reference_hz()`; `unstick()` returns the clocks it took; `dma_isr()`,
  `Request::engine` and `has_dma`, `dma` carrying every half at any length,
  the one-byte read included.
- **`start()` spends dispatch time on the bus.** This lineage cannot queue
  a START behind a STOP and nothing in its chapters signals the STOP's
  leaving, so `start()` waits for it: about one bit period when a tenure
  is started at the last one's completion, bounded at 5 ms. A clock held
  past that parks the tenure, no START and no vector, for the per-bus
  timeout to answer.
- The event line is raised by `start()` and dropped at the completion, so
  the BTF a STOP leaves standing does not re-enter the vector.
- A write-then-read requests its repeated START on the TxE of the last
  written byte, while that byte shifts, and not at BTF. One measurement
  decided it: with the START at BTF, a CH32V303VCT6 and a CH32V203C8T6 each
  lost that byte when the chip's own two controllers were host and target
  and the target was polled by the core that also served the host's
  vectors. The CH32V203C8T6's target served from its own vector took every
  byte, and so did a target polled by a core of its own on another board
  (both those parts, and the CH32X035). So the loss needs one core serving
  both ends, and the TxE order, where WCH's interrupt example also puts it,
  costs nothing against a target that does not lose it ([the CH32V203/CH32V303
  I2C page](../ch32vx03/i2c.md), [the CH32V00x I2C page](../ch32v00x/i2c.md)).
- The price is a NACK of that byte arriving with the START pending. No host
  stores to the control register there: the controller generates the
  repeated START anyway, and a withdrawal races it (measured: it hung the
  event vector on the STM32F4 with the core at 48 MHz, wedged the bus on the
  CH32V00x at 400 kHz, and lost on the CH32V203/V303 with the core at 8 MHz).
  The host answers `i2c_nack_data` and closes behind that repeated START with
  a void write: the address with the write bit and its STOP, one address byte
  on the wire when a byte is refused.
- An ADD10, or any event the address phase does not serve, ends the tenure
  `i2c_bus_error` with the STOP and then a data-register store that clears
  ADD10, which the STOP alone does not (measured).

**STM32F4's classic host (`stm32f4`).** `reference_hz()` is the instance's
own APB1 clock. An `I2cHostConfig` at init holds fast mode's duty shape, the
wire's two rise times and the noise filters. `digital_filter_max()` is advice
and not a refusal; `repeated_start_setup_at_risk()` publishes an erratum's
window a write-then-read sits in. `spurious_bus_errors()` is as the G0's, and
here a bus error never ends a tenure at all, the manual and the erratum
agreeing that a controller's transfer is unaffected. The engine slots sit on
the request mapping's own cells. On the pump the first two data bytes load in
the ADDR entry, so a one- or two-byte write and a register index take no TxE
interrupt. `claim_smba_pad()` is for the SMBus alert.

**STM32F4's FMPI2C1 (`stm32f4 (2)`).** The STM32G0's surface almost entire,
`recover()` being 23.4.6's PE cycle and nothing more, and three speeds where
the family's other host has two. `reference_hz()` is the APB clock the
registers are on while `kernel_hz()` is the one the arithmetic divides, here
two different rates. Two vectors, with a NACK on the event one (it is
not an error on this block). A NACK is the address's when nothing of the half
reached the wire, told from the byte pump's position or, under the engines,
from TXE and the transmit stream's count. A controller's BERR is counted and
never acted on. The engine slots sit on the mapping's single cell per
direction, `dma` carrying both phases, a half with no buffer apart, with the
engines armed before the START (23.4.16's order) and a write half's first
byte in TXDR ahead of the transmit stream (ES0298 2.12.8's first workaround, one path in and
out of the band, a write of one byte starting no stream; `interrupt` stores
outside the erratum's window). `start()` does not wait for the last STOP,
since this block queues a START behind one itself. `claim_smba_pad()` is for
the SMBus alert.

**CH32V00x (`ch32v00x`).** `reference_hz()` is HCLK; a `duty` at init gives
fast mode's shape. The engine slots are fixed to DMA channels 6 and 7, the
transmit block's completion handing the tenure back to the event vector. On
the pump the ADDR entry stores no data but raises the buffer interrupt, so a
one-byte write takes four entries. `unstick()` also clears the STOPF its own
STOP leaves.

**CH32V203 and CH32V303 (`ch32vx03`).** `reference_hz()` is PB1, with a
`duty` and the wire's rise time at init. The bus clock is refused outside
19.12.2's 4..60 MHz window, narrower than the stratum's own PB1 ceiling. The
engine slots are fixed to the instance's own channels (I2C1 on DMA1's 6 and
7, I2C2 on its 4 and 5). After the wait for the STOP, a BUSY standing with
both lines high takes 19.12.1's SWRST, counted (`unwedges()`). `busy()` is
for a sleep site. On the pump the first two data bytes load in the ADDR
entry. The client half takes `I2cClientOptions` (a polled client with its
three enables down) and has `flush()`, the PE cycle for a byte the
controller never clocked.

**CH32X035 (`ch32x035`).** `reference_hz()` is HCLK, with a `duty` at init.
The block is asked for first: the datasheet withdraws the I2C from some lots
and no register names one, so `I2c<1>::present()` and both `init()`s check
the block holds its timing. Six columns, the part's first off the debug port
the default, the debug port's refused while the probe owns it and the USB
pads while USB_IOEN does. The engine slots are fixed to DMA channels 6 and 7
(ORs shared with USART2). On the pump the first two bytes load in the ADDR
entry. After the wait for the STOP, a BUSY over an idle wire takes 15.9.1's
SWRST, counted (`unwedges()`). `unstick()` drives a line low push-pull and
releases it as an input, this series having no open-drain GPIO. The client is
polled (`I2cClientOptions`) or on its vectors, with `flush()`. `Placement`,
the last template argument (`util/placement.hpp`), can put the hot path in
SRAM.

**RP2040 (`rp2040`).** The Synopsys command FIFO, where an entry carries its
RESTART and STOP bits and one TX_ABRT carries every failure. `reference_hz()`
is clk_sys; `timing_of`; `idle()` as the SAM's. `start()` answers a second
refusal synchronously, `i2c_bus_error` for a block that would not disable to
take the address. `dma` puts the read phase alone on the engine slots at any
length, with `dma_isr()`: the receive engine collects every byte and a
transmit engine of `uint16_t` entries pours the read commands between the
first and the last, so a read of one or two takes the receive engine alone.
The write half stays on the pump, a byte beat landing in the command register
replicated across the word, its copy in bit 8 a read command (RP2040 2.1.4).
The pump reads at half the FIFO, so a tenure of eight bytes or less is one
interrupt and a longer one one per eight. A tenure that completes ends on its
own STOP_DET and a failure on TX_ABRT. The disable / IC_TAR / enable cycle is
skipped while the address and the speed stand and the last tenure ended on
its STOP, which takes the block's bus-timed disable (the whole cycle 1.4 to
1.6 us at 400 kHz, 7.6 to 12 at 100) out of the dispatch. `unstick()` returns the clocks it took; `recover()`
resets the block through the reset controller, the one way out of a host
holding SCL with no STOP in sight. `Placement` as the CH32X035's.

**RP2350 (`rp2350`).** The same block and the same driver, over a traits type
this family supplies: the RP2040's paragraph holds but for the write half.
Under `dma` a pure write of three bytes or more rides the transmit engine
(`dma_carries_writes`), in byte beats from the request's own buffer, through
the alias of the command register that zero-fills the unused byte lanes
(RP2350 2.1.5, measured on both architectures); its first entry and its last,
with the STOP, are the pump's, as a read's commands are, and a write before a
read and a write of one or two bytes stay on the pump. What is the chip's lies
around the block: the pad functions, the GPIO bank, the request table, the
reset controller and the microsecond ruler a bus clear by hand is paced by
([the RP2350 I2C page](../rp2350/i2c.md), [the DesignWare I2C
page](../dw_apb_i2c/README.md)).
