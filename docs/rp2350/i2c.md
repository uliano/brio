# I2C (RP2350)

This document is for a programmer who puts a device on an I2C bus of an
RP2350 - a sensor, a converter, a memory - or who makes this chip answer
as a device on another chip's bus, on either of the chip's two
processor architectures. The chip carries two Synopsys DW_apb_i2c
controllers. Their driver is written once for every family that carries
the peripheral, in the DW_apb_i2c IP stratum
([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md)); this page and
`brio/rp2350/i2c.hpp` are what the RP2350 adds to it - the pads and
their setup, the reset bits, the interrupt lines, the DMA request
numbers, which clock the bus timing is counted in, and the open-drain
verbs a bus clear by hand needs - and the names a program writes:
`DwApbI2c<n>`, the instance; `I2cHost<n, pins, ...>`, the host;
`I2cClient<n, pins>`, the other end. A host is not usually driven
directly: a program names one arbiter per bus, `I2cBus` (an active
object - a component of a brio program with its own event queue, served
one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md) - that takes the
requests of every active object that posts one, runs them one tenure at a
time, a tenure being one START to its STOP, and answers each with
an event, [design/i2c-bus.md](../design/i2c-bus.md)), and posts it a
request.

The smallest use, a register read from a device at 0x48 through the
arbiter:

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

An excerpt: it assumes the program's platform `P`, its clock task's tag
`clock`, `static uint8_t reg[1]` and `static uint8_t value[2]`, and an
active object `Sensor` that handles `I2cDone`; `init()` runs in `main()`
and the post in one of `Sensor`'s handlers.

Line by line: `pins` names I2C0's clock and data pads by number (a pair
that is not the instance's does not compile); `Bus` names the host
with every other template argument at its default (no DMA
engines, the code in the flash); `I2c` is the arbiter for that bus, an
active object of the program's `Tenuto` (the cooperative kernel,
[../design/kernel.md](../design/kernel.md)) over platform `P`, with four
requests of queue, every completion sent straight back to its
requester with no retry (`BusPassThrough`, the default policy) and a
timeout of 20 ms for a tenure that never
completes (a request of 255 bytes is 23 ms of wire at 100 kHz, 5.8 ms at
400 kHz, so the figure must exceed the program's longest request at its
speed); `Bus::init()` takes the clock task's tag
([clock.md](clock.md)); the request names the device's address, lends
the register number to write and the buffer to read into
(`brio::lend<brio::Lease::reply>` - the buffers stay the caller's until
the reply), the speed, and where the reply goes (`Sensor`, the active
object that posts it, receives an `I2cDone`); `post` hands it to the
arbiter; and the `extern "C"` function is the instance's interrupt
handler, under the crt's name for I2C0's line, the same on both
architectures, which tells the arbiter when the tenure has
completed.

What decides correctness even there. `init()` comes after the clock
task: the bus timing is counted in clk_sys cycles at the rate the clock
type claims, not read from the hardware. `init()` answers false when not
even standard mode can be produced (a clock under 2.7 MHz) or the peripheral
did not come up; a speed the
clock cannot produce is answered `i2c_rejected` at the request, and so
is an address outside 0x08..0x77 - the general call and the 10-bit
prefixes among them, which every host refuses
([../design/i2c-bus.md](../design/i2c-bus.md)) - and a request naming
`I2cEngine::dma` on a host with no DMA engines bound, as this `Bus`
is. The
program binds `isr_i2c0` itself; left unbound, the line's first
interrupt goes into the crt's default spin and the program stops there
(no timeout can answer from inside a spinning handler). `reg` and `value` must
stay untouched and in scope from the post to the `I2cDone`; nothing
checks it. An `i2c_timeout` hands them back too: `recover()` stops the channels
before the arbiter answers, so nothing still writes them. The request is posted from an active object's handler, which
is what the reply is sent back to. `init()` hands both pads over itself,
the isolation latch dropped and the pull-up 12.2.1.3 asks for on, but
the pulling is the board's ("What the silicon does" below). And a
program that calls `unstick()`, the bus clear by hand, must first start
the microsecond ruler that paces it (`Mtime::start(clock)`, "How to use it"
below): on the Cortex-M33 nothing else starts it.



## Documents of record

The RP2350 datasheet (build 80d627281ed5), 12.2 (the Synopsys DW_apb_i2c
v2.03a: 12.2.1 the feature list with the FIFO depths and the pad setup,
12.2.5 the behaviour - START and STOP generation, the combined formats
-, 12.2.7 the transmit FIFO and the RESTART / STOP bits of a command,
12.2.10 the operation modes, 12.2.11 the spike filter, 12.2.12
fast-mode-plus, 12.2.13 the bus clear, 12.2.14 the SCL counts with their
floors and the cycles the peripheral adds, 12.2.15 the DMA interface, 12.2.16
the interrupt registers, 12.2.17 the registers), 9.4 (the GPIO function
table), 12.6.4.1 (the system DREQ table), 7.5 (the reset controller),
3.2 (the interrupt lines), 2.1.3 (the atomic register aliases), 2.1.5
(narrow writes replicated across a register, and the alias that zero-fills
them), 3.1.3 and 9.8 (the
single-cycle IO the unstick drives an open-drain line through), 9.7 (the
pad isolation latches); [../design/i2c-bus.md](../design/i2c-bus.md) for
the Request and the vocabulary. The driver is Synopsys's and not this
chip's, so it lives in the IP stratum: `brio/dw_apb_i2c/i2c.hpp` holds
the resource, the host and the client
([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md)), and
`brio/rp2350/i2c.hpp` holds what this chip owes it - the pin table of
9.4 with its pad configuration, the `I2cPad` type that adds the two
open-drain verbs to a `Pin`, the `Rp2350DwApbI2c` chip traits over
`pin.hpp`, `resets.hpp`, `core.hpp`, `delay.hpp` and `dma_engine.hpp`,
this chip's device description held against the bits the IP file spells
itself, and the public names `DwApbI2c<n>` (the resource), `I2cHost`
(the host `I2cBus` drives) and `I2cClient` (the other end), this
chip's aliases of the three templates there. The reference suite:
`test_rp2350_i2c`, which runs on both of this chip's architectures from
one source: the peripheral with nothing on the wire, its pads, and the wire to a second board, the partner - I2C0 on GP36 (SDA) / GP37 (SCL), 1.5 kOhm to the WeAct
RP2040's 3V3 on each
([../boards/weact-rp2350b.md](../boards/weact-rp2350b.md)), whose test firmware,
the peer, is commanded over the bus under test; that
board's own suite, `test_rp2040_i2c`, judges this chip's peripheral as that
peer. A letter of a suite is one command key of its console and the
group of verdicts it runs; `z` is the key that runs every letter but
`x`, the hand-made window traced, which runs by name.

This page says what is the RP2350's. What the peripheral itself does - the
command FIFO, the counts and their floors, the single TX_ABRT, the
client's clock stretch, the engine parameters - is the IP stratum's page and
is not restated here.

## What the silicon does

Two DW_apb_i2c controllers, the design the RP2040 carries with the same
synthesis - 12.2 names IP version v2.03a and the RP2040's 4.3 v2.01, and both
sheets give IC_COMP_VERSION the one reset value 0x3230312a: a host or a client and never
both, standard, fast and fast-mode-plus (no high-speed mode), 7-bit
addresses on both sides and 10-bit ones, two FIFOs of sixteen entries
(12.2.1), one interrupt line per instance with thirteen sources, a DMA
request per FIFO. 12.2 has no "Changes from RP2040" section, where the
SPI, the ADC and the DMA chapters all do, and the two chips' register
descriptions agree define for define - which is why the driver is one
file in an IP stratum and this one is a page of differences. There are
six, and none of them is inside the peripheral:

**The pins are one column, over a longer bank.** Every fourth pad from
GP0 carries an I2C signal - SDA on the even pads, SCL on the odd, the
instance alternating in pairs - so I2C0 has SDA on GP0, 4, 8, ... 44 and
SCL on 1, 5, 9, ... 45, and I2C1 has SDA on 2, 6, 10, ... 46 and SCL on
3, 7, 11, ... 47. That is the RP2040's rule over forty-eight pads instead
of thirty: twelve SDA pads and twelve SCL pads per instance against eight
and eight. What the UART gained here, the I2C did not - the UART's
flow-control pads carry data too under a second function code
([uart.md](uart.md)), while 9.4 gives every one of the forty-eight I2C
pads exactly one I2C entry, all of them function 3. So a pin set stays a
pair of pad numbers and carries no column beside them.

**The pads come up isolated** (9.7, 9.11, and [README.md](README.md)):
PADS_BANK0's reset value is 0x116, the ISO latch set and the input buffer
disabled, and while the latch stands the pad is cut off from the digital
logic in both directions. A pad configured as on the RP2040 would do
nothing at all. Every configuring verb of `pin.hpp` writes the whole pad
register with ISO clear, so the driver's hand-over - the same call the
RP2040's makes - is what drops the latch. What it writes is 12.2.1.3's
own list: pull-up enabled, slew rate limited, Schmitt trigger enabled,
with the chapter's note that the pad's pull-up is not meant to do the
pulling and the board's should.

**The bank is two words**, and that is the one line of the family header
that is not the RP2040's. An open-drain line is driven by its output
enable over an output register already holding a zero, and SIO holds
GPIO0..31 in GPIO_OE and GPIO32..47 in GPIO_HI_OE (3.1.3). `I2cPad` picks
the half its pad falls in with an `if constexpr` on the pin number, so a
line on GP32..GP47 is driven through the right register and the choice
costs nothing.

**The DREQ numbers and the reset bits are not the RP2040's.** A third PIO
and twelve PWM slices against eight move every DREQ row above the first
PIO's: the I2Cs sit at 44..47 where they sat at 32..35. The reset
controller governs twenty-nine peripherals against twenty-seven, and I2C0 and
I2C1 sit at bits 4 and 5 where they sat at 3 and 4. Both come from this
chip's own headers, through `dma_engine.hpp` and `resets.hpp`.

**The interrupt numbering is shared between the architectures** (3.8.4.2),
so I2C0_IRQ is line 36 and I2C1_IRQ line 37 whichever processor pair is
running - not the RP2040's 23 and 24 - and an app binds `isr_i2c0` /
`isr_i2c1` once for both. The controller behind those names is not the
same object, an NVIC on the Cortex-M33 and Hazard3's own on the
other architecture, which is exactly why the IP stratum's concept takes the controller
as a type.

**A byte beat can land as a plain write entry.** A write entry is the
request's byte with the command bits above it clear, and a byte written at
IC_DATA_CMD's own address reaches it replicated across the word (2.1.5, and
12.6: the DMA has one transfer size for both sides), its copy in bit 8 a read
command, in bit 9 a STOP, in bit 10 a RESTART. This chip's fabric offers the
way out the RP2040's does not: at +0x4000, bit 14 of the address, "invalid byte
lanes [are] driven to zero" (2.1.5). 2.1.3 says the I2C's atomic aliases are
served by a bus interposer and does not list this one among them, so what it
does on this peripheral is a measurement, on both architectures (letter y of
`test_rp2350_i2c`): byte beats through it moved 3 to 255 bytes at 400 kHz and
255 at the two other speeds with every one at its position on the partner; the same run at the
register's own address delivers other bytes (the control below, "A pure write
on the transmit engine"). So the transmit engine, one of the DMA channels a host may be given, is
bound there and a pure write can ride it from the request's own buffer, with
no copy. A read's halfword commands go through the same address: the upper
lanes are zero instead of a copy, in a part of the word the peripheral does not
read.

And one thing that is not a difference in the peripheral but changes what a
program must do: **the unstick is paced by the platform timer.** The bus
clear of 12.2.13 has no enable in this chip's IC_CON, whose writable bits
are 0x7FF and stop at the eleventh, so the nine clocks and the STOP
are driven by hand, half a bit of standard mode at a time, and the ruler
they are paced by is `rp2350/delay.hpp`'s. That file counts the platform
timer of 3.1.8 and not CPU cycles, because that counter is the one ruler
both architectures have; it follows that a program which calls
`unstick()` must have started it (`Mtime::start(clock)`), which
Hazard3's ticker already does and the Cortex-M33's does not.
With the counter stopped every wait is refused and spends no time, and
the nine pulses go out at the speed of two register writes, far faster than
half a bit of standard mode.

**Erratum RP2350-E9 changes how a wire is proven, not how it works.** On
stepping A2 a pad whose input buffer is enabled and which nothing drives
leaks enough current to sit high against its own pull-down, so an idle
level is history and not a measurement. An I2C line idles high anyway,
pulled there by the board, and the driver hands both pads over with the
pull-up the chapter asks for, so nothing the peripheral does is coloured by
the leak: a line that reads low is genuinely being driven low, which is
all `unstick()` asks. What the erratum does change is a program's ability
to ask "is a pull-up fitted at all", because a bare pad answers the same
as a pulled-up one. The errata sheet's own workaround is the instrument:
the pad's pull-down on with the input buffer off, the buffer enabled for
the read alone (`Pin::read_pulsed()`, [pin.md](pin.md)) - the leak needs
the buffer on, so only an external pull-up reads high that way.

## Types and verbs

What the IP stratum names (the speed vocabulary, the timing arithmetic,
the register bit layouts, the resource's verbs, the host's and the
client's) is in [../dw_apb_i2c/README.md](../dw_apb_i2c/README.md). This
stratum adds:

- `I2cPins` - the two pad numbers, SCL and SDA. One column, so no
  function travels with them.
- `i2c_sda_pin(n, pin)`, `i2c_scl_pin(n, pin)` - the table of 9.4 as
  constexpr facts over the whole bank, and `i2c_pins_valid(n, pins)`,
  which a host's `static_assert` calls. The package is not asked
  there: the table is the die's, and a pad the QFN-60 has not bonded is
  refused one level down by `Pin<n>`'s own static_assert, which says so
  in those words.
- `i2c_pad_config` - the pad setup 12.2.1.3 asks for, as this stratum's
  `PinConfig`. Writing it is what takes the pad out of isolation.
- `I2cPad<pin>` - a `Pin<pin>` with the two verbs an open-drain drive by
  hand needs, each choosing the half of the bank its pad falls in.
- `Rp2350DwApbI2c` - the chip traits, which nothing above this file
  names: the registers, the reset bits, the interrupt lines and
  their controller, the DREQ numbers, the atomic aliases, the pad type
  and its function, the microsecond ruler, and ic_clk = clk_sys
  (12.2.1.2).
- `DwApbI2c<n>`, `I2cHost<n, pins, TxEngine, RxEngine, Placement>` and
  `I2cClient<n, pins>` - the public names, this chip's aliases of the
  three IP templates, with `NoDmaEngine` in both engine parameters and `InFlash` by
  default (`Placement`: util/placement.hpp's tag, [platform.md](platform.md),
  "A handler in SRAM"). The host's `Request` names what moves its bytes in
  `engine`, `I2cEngine::interrupt` by default or `I2cEngine::dma`,
  and `has_dma` says whether engines are bound (the IP stratum's page,
  "The mechanism a request names"). `dma_carries_writes` is true on an engined
  host of this chip, and `dma_write_min`, three, is the shortest write it
  carries: a pure write of that length or more under `dma` rides the
  transmit engine. A write before a read and a write of one or two bytes
  stay on the interrupt.

The family header also holds the vocabulary check: every bit the IP file
spells for itself, held by `static_assert` against the macro this chip's
own device description gives it. That the two agree register for register
is a measurement and not an assumption a driver may make.

## How to use it

The host on the interrupt is the code that opens this page. The same source builds for both architectures; `isr_i2c0` is a
vector-table slot on one architecture and a dispatch entry on the other.

With the engines (the DMA channels a host may be given): `I2cHost<0, pins, DmaTxEngine<4, uint16_t>,
DmaRxEngine<5>>`, `brio::Dma::init()` before `Bus::init()` ([dma.md](dma.md)),
the DMA line's handler below, and every request that should ride them naming
`dma` (an excerpt: the request
above on that host, reading 64 bytes into a buffer `data` of
the program's, lent like `value`):

```cpp
r.rx = brio::lend<brio::Lease::reply>(data); r.rx_len = 64;    // a read worth the engines
r.engine = brio::I2cEngine::dma;                               // the read half on the DMA channels
```

```cpp
extern "C" void isr_dma_0() {   // line 0: an engine's default interrupt line
    if (Bus::dma_isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
}
```

`dma_isr()` answers true when a bus error on a channel has just ended the tenure,
`i2c_dma_fault` in `status()`; the transmit DMA block's end, its usual entry,
answers false, and the STOP then ends the tenure on `isr_i2c0`. **This vector
must be bound.** `init()` arms the engines, and that enables the line, and the crt
gives `isr_dma_0` the same weak default as `isr_i2c0`. Left unbound, a read of one or
two bytes completes, the receive engine reporting errors alone; the first read or
pure write of three bytes or more parks the core in that spin at the transmit DMA
block's end, where no timeout can answer.

A long write the same way (an excerpt, the same host, `page` a buffer of the
program's, lent until the `I2cDone`; a request with no read half):

```cpp
Bus::Request w{};
w.addr = 0x50;
w.tx = brio::lend<brio::Lease::reply>(page); w.tx_len = 64;    // from this buffer, no copy
w.speed = brio::I2cSpeed::fast_400k;
w.reply = brio::reply_to<Sensor, brio::I2cDone>();
w.engine = brio::I2cEngine::dma;                               // the bytes between the first and the last on the channel
brio::post<I2c>(w);
```

This chip has sixteen channels and four interrupt lines. An engine's second
parameter is its beat's width and its third the line, 0 by default, so a host
whose engines ring on line 2 is `DmaTxEngine<6, uint16_t, 2>` and
`DmaRxEngine<7, uint8_t, 2>` with `isr_dma_2` ([dma.md](dma.md)).

The program chooses, request by request. A request that names nothing runs on the host's
interrupt, at any length, and touches no channel; one that names `dma`
has its read half (the bytes it reads) on the engines at any length - the receive engine
collecting every byte, the transmit engine pouring the commands of a
read of three bytes or more - and, with no read half, a write of three
bytes or more on the transmit engine. Every other write half stays on the
interrupt and the request completes. On a host
with no engines bound, `dma` is answered `i2c_rejected`. On a host with engines
a wrong choice costs core cycles or a sleep (below), never correctness.

The core cycles each costs at 150 MHz, Cortex-M33 / Hazard3, are `bench_rp2350`
letter i's `i2c.cost` lines: `start()` and the vectors together, on the host with
engines bound, in the flash with its vector. Each line is the least of five, and
each figure is the same at the three speeds and in five runs of five unless a cell
says otherwise:

| read of | 1 | 2 | 3 | 8 | 9 | 16 | 64 | 255 |
|---|---|---|---|---|---|---|---|---|
| `interrupt` | 367 / 347 | 378 / 362 | 396 / 383 | 486 / 488 | 609 / 593 | 735 / 740 | 2540 / 2431 | 9698 / 9155 |
| `dma` | 384 / 357 | 386 / 361 | 586 / 580 | 522 / 480 | 531 / 485 | 531 / 485 | 618 / 556 | 618 / 556 |

| write of | 1 | 2 | 3 | 8 | 13 | 14 | 16 | 17 | 32 | 64 | 255 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `interrupt` | 314 / 303 | 323 / 313 | 333 / 323 | 383 / 373 | 433 / 423 | 443 / 433 | 463 / 453 | 590 / 555 | 874 / 799 | 1734 / 1523 | 6884 / 5858 |
| `dma` | 320 / 308 | 329 / 318 | 507 / 516 | 443 / 416 | 443 / 416 | 443 / 416 | 443 / 416 | 443 / 416; 525 / 481 at 100 kHz | 525 / 481 | 525 / 481 | 525 / 481 |

F, b and E are [the design page's](../design/i2c-bus.md) ("Which to name, by the
cycles"): the pump's fixed cost and its cost a byte, and the engines' fixed cost.
For a read of sixteen bytes or more, the pump, the host's interrupt moving the
bytes, costs F + b x n, F 135 / 177 cycles and b 37.5 / 35.2
a byte, in steps of eight (one entry for each half FIFO); the engines cost E,
531 / 485 for a read of nine to sixteen bytes and
618 / 556 from sixty-four (a third entry, the transmit DMA block's end
finding no room for the STOP entry). So `dma` costs the core fewer cycles from
**nine bytes** on the Cortex-M33 and from eight on Hazard3, by eight cycles
there (the pump's second entry comes at nine on both); a read of one
or two costs the same within seventeen cycles either way, and one of three
to eight costs the engines a second entry (the transmit DMA block's end).

For a write, `start()` alone writes up to sixteen entries with the one
interrupt at the STOP, so the pump costs 304 / 293 cycles and 10 a byte, and its
second entry comes at seventeen; the channel costs E, 443 / 416 for a write of
eight to sixteen bytes and 525 / 481 from thirty-two, whatever the length. Its
third entry is the same TX_EMPTY as a read's; at seventeen it comes at 100 kHz and
not at the two faster speeds, by how full the FIFO stands at the DMA block's end. The
design page's inequality puts E - F, 139 / 123 cycles, over 10 a byte: `dma` costs
the core fewer cycles from **fifteen bytes** on the Cortex-M33, level at fourteen,
and from **thirteen** on Hazard3. A write of one or two is the pump's under either
value, `dma` adding the 6 / 5 cycles of its tests.

A write of three to eight costs the channel a second entry. At three and four its
figure reads 64 or 65 / 100 or 103 cycles higher than at eight, in `start()`: a DMA block
of one or two beats ends before `start()` returns, so the DMA line's entry and
return fall inside `start()`'s span (inferred from those lengths, not counted apart;
a read of three and four shows the same, 64 or 65 / 100 or 103).

What the channel buys past the break-even is the core free on a long write. A write
of 255 at 400 kHz, from the same image, keeps the core busy 16 693 / 7328 cycles on the
interrupt and 2119 / 891 on the channel, 31 entries against 3, its longest
host entry 105 / 77 cycles, at the same wall time (x 1.00). Busy is every cycle the
core is not idle ([../design/benchmark.md](../design/benchmark.md)): beyond the
handlers' 5878 / 5463 cycles and `start()`, some 340 / 50 cycles an entry
remain, which no letter splits (`bench_rp2350` letter r's `window` line reads one
idle turn at 150 / 42 cycles).

Under either value the core waits in its sleep instruction between entries, and the
chip's SLEEP state asks in addition that the DMA have no outstanding transfer on any
channel (6.5.2), which a `dma` tenure's channels hold for its data: what that costs is
not measured ("Not covered yet").

A program that means to call `unstick()` starts the ruler its half-bits
are paced by, which on the Cortex-M33 nothing else does:

```cpp
brio::Mtime::start(clock);      // keeps the count; the Hazard3 ticker does too
```

A client (an excerpt: `clock` as above, and `heard()`, `next_answer()` and
`tenure_over()` the program's, each short, since it runs in the vector and no
handler nests over another):

```cpp
constexpr brio::I2cPins client_pins{.scl = 15, .sda = 14};
using Client = brio::I2cClient<1, client_pins>;
if (!Client::init(clock, {.address = 0x42, .speed = brio::I2cSpeed::fast_400k})) {
    // no client: the peripheral did not come up, or the clock cannot meet the speed
}
Client::interrupts(Client::events, true);
extern "C" void isr_i2c1() {
    switch (Client::service()) {
        case brio::I2cClientEvent::byte_received: while (Client::data_ready()) { heard(Client::take()); } break;
        case brio::I2cClientEvent::byte_wanted:   Client::give(next_answer()); Client::clear_read_request(); break;
        case brio::I2cClientEvent::stop:          tenure_over(); break;
        default: break;
    }
}
```

## Bench findings

All of them on an RP2350 in the QFN-80 package, **stepping A2**, clk_sys
at 150 MHz, and all of them on both architectures: `test_rp2350_i2c`
reports **101 pass, 0 fail** on the Cortex-M33 pair and on the Hazard3
pair, from one source, against the WeAct RP2040's peer over the I2C wires
between the two boards. Every verdict is the same on the two, which is what a
peripheral outside the processor should look like; the cycles differ, and a few
races between the core and the wire, each stated where it is met. The same
peripheral as the peer, the RP2040 the host, is judged by `test_rp2040_i2c`.
[../rp2040/i2c.md](../rp2040/i2c.md) holds the wire findings this peripheral
shares - the reserved addresses refused by the host both ways, a STOP_DET for every
STOP, the refusal by a disable - and they are not restated here. The
cost figures are `bench_rp2350` letter i's, against the same peer. A letter below
with no suite named is `test_rp2350_i2c`'s.

- **The counts at this clk_sys.** 100 kHz solves to HCNT 585 and LCNT 899,
  400 kHz to 135 and 224, 1 MHz to 45 and 89 - each with SPKLEN 8 - and
  each gives its asked rate exactly on an ideal wire once the SPKLEN + 8
  cycles the peripheral adds are taken out of the counts. Table 1106's floors
  come back as the chapter states them: fast mode at 12 MHz is (LCNT 15,
  HCNT 6), standard mode at 2.7 MHz is (12, 6), fast-mode-plus needs 32
  MHz, and a hertz below each is refused.
- **The reset state.** IC_CON reads **0x65** - a host with its client half
  disabled, the fast speed code, restarts enabled - with the spike filter
  at 7, the hold at 1, both FIFOs empty and the peripheral disabled.
  **IC_COMP_PARAM_1 reads zero**, as it does on the RP2040: the FIFO
  depths are the chapter's word and not the register's. The transmit FIFO
  is **sixteen** entries deep, measured under TX_CMD_BLOCK - full at
  sixteen, TX_OVER on the seventeenth, flushed by the disable - and the
  disable of an idle peripheral is reported by IC_ENABLE_STATUS within **3 us**.
  The interrupt lines are **36 and 37**, not the RP2040's 23 and 24.
- **The pull-ups read the way E9 forces.** A pad of this stepping whose
  input buffer is on holds history against its own pull-down, so an idle
  level says nothing: GP36 and GP37 are read with the pull-down on and
  the buffer pulsed for the read alone (`Pin::read_pulsed()`,
  [pin.md](pin.md)), and both read high: the 1.5 kOhm to the partner's
  3V3.
- **The pads and the isolation latch, above GP31.** After the hand-over
  GP36 and GP37 read **0x5A** - exactly what 12.2.1.3 asks for: the
  pull-up, the slew limit, the Schmitt trigger, ISO clear, the input
  buffer on - with FUNCSEL 3, and they carry an address phase: a probe of
  an address not acknowledged is `i2c_nack_addr`. The unstick,
  on that healthy bus, reads 0 and takes the pads to SIO through the high
  half's registers and hands them back with **the same 0x5A**, the next
  probe unchanged. `release()` leaves them at 9.11's reset value
  **0x116**, isolated with the input buffer off and a pull-down, and
  FUNCSEL at none.

- **On the wire, above GP31.** All of this runs through GP36/GP37 and the
  unstick's verbs on the high half of SIO. The scan finds the peer's 0x6B alone,
  an address not acknowledged `i2c_nack_addr` in 109 to 111 us at 100 kHz. Every tenure
  shape, and the vocabulary: a deaf client, a data NACK at the first and at the
  third byte, 0x7A and the general call refused by the host inside
  `start()` with the peer answering the general call and matching only
  the write to its own address. The three speeds byte-exact both ways, 99, 392 or
  393 and 952 to 966 kHz measured on the timer for 100, 400
  and 1000 asked. The engines with eight 255-byte reads at 1 MHz and
  TX_OVER never raised. I2cBus with its timed bus (`i2c_timeout` at the
  arbiter's 20 ms, the peer holding SCL). The stuck bus: four pulses for
  a client releasing at its fourth fall, 0xFF in 122 or 123 us for a
  line nine pulses cannot free. Arbitration both ways, and this board
  as the peer host's client, written to and read from. The client's stretch prices at 755 to 765 us for eight holds of
  100 us, a write under it within 3 us of the plain one.
- **The client's edges** (letter m, the peer's host at 100 kHz, both
  architectures): without the hold, 24 bytes written into a receive FIFO nobody
  drains are all acknowledged, the first 16 kept exact and eight
  `overrun` events raised, one a byte lost; with the hold the same write
  stretches until the vector drains it, all 24 exact. A disable after the
  fourth byte of a write refuses the fifth - the far host's data NACK -
  with SLV_DISABLED_WHILE_BUSY and SLV_RX_DATA_LOST both set. A disable at
  a read request the client holds SCL for never completes: IC_EN stays 1
  through 2 ms and 50 ms more, a byte written after it untaken, neither
  bit set; `release()` lets SCL go and the far host's read finishes on the
  pull-up - the RP2040's figures exactly
  ([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md)). Ten runs of z on
  each architecture.
- **The host's cost**, `bench_rp2350` letter i against the peer's
  client, the same image built for both architectures, the host's vector placed
  in SRAM (the V image of [platform.md](platform.md), "A handler in SRAM",
  `BENCH_VECTORS_IN_SRAM` in bench_rp2350.cpp; the tables under "How to use it"
  are the default image's, the vector in the flash), nothing of the far end on
  this core; the wire's time at the SCL
  period measured (378.9 or 379.0 cycles at 400 kHz against the counts' 375:
  the pull-ups' rise, 26 or 27 ns a period at every speed). Each line of the
  letter is the best of five; each figure below its range over five runs,
  clk_sys cycles at 150 MHz, Cortex-M33 / Hazard3:

  | at 400 kHz | wall | interrupts | handler cycles | busy | fixed cost |
  |---|---|---|---|---|---|
  | write of 1 | 7757 / 7764 (x 1.05) | 1 | 96 / 70 | 680 / 371 | 405 / 412 |
  | write of 255 | 874 145 to 874 149 / 874 154 to 874 157 (x 1.00) | 31 | 6178 / 5196 | 17 358 to 17 506 / 7038 to 7122 | 404 to 406 / 411 to 414 |
  | read of 16 | 59 028 or 59 029 / 59 030 to 59 032 (x 1.00) | 2 | 375 / 328 | 1358 to 1506 / 776 to 818 | 511 or 512 / 513 to 515 |
  | read of 255 | 874 233 to 874 247 / 874 243 to 874 248 (x 1.00) | 32 | 9109 / 8573 | 23 470 / 10 501 to 10 543 | 494 to 503 / 497 to 506 |
  | register read, 1 + 1 | 15 158 to 15 160 / 15 162 or 15 163 (x 1.03) | 1 | 133 / 102 | 650 to 798 / 401 | 453 to 455 / 457 or 458 |
  | read of 255 on the engines | 874 163 to 874 170 / 874 171 to 874 185 (x 1.00) | 3 | 333 / 266 | 2242 to 2351 / 923 to 965 | 424 to 426 / 430 to 441 |

  A read of half a FIFO or less, and a write of a FIFO or less, is one interrupt,
  its STOP; a long read is one every eight bytes. Busy can hold one idle turn more
  than the tenure's own, a tick's wake: the core's way through the idle loop and
  back, 150 / 42 cycles by `bench_rp2350` letter r's `window` line, its handler taken out.
  The write of 1's 680 / 371 holds one, in the run with the shortest wall of the five, and
  it reads 532 / 329 at 1 MHz without. The same step of 148 / 42 cycles, the
  handler's unmoved, comes and goes at other lengths.

  The long tenures' walls agree within some fifteen cycles over these five runs.
  Other sessions' 255-byte tenures have read up to some 700 cycles shorter, three
  tenths of a cycle a period, reads against writes as well, so a long read's fixed
  cost is known less well than the table's range says (254 to 506 across sessions).
  The host's code is not what moves them: two images of the host and two of the
  peer that differ in the write path read alike within one session. What moves them
  is not established.

  The address and the speed stay in the
  peripheral between tenures, so a one-byte write's fixed cost is the same at
  400 kHz and 1 MHz (405 on the Cortex-M33, 411 or 412 on Hazard3). At
  100 kHz it reads 571 on the Cortex-M33 and 410 to 575 on Hazard3, and the
  excess is not split. The disable, IC_TAR and enable that a new address still
  pays costs 1563 to 1653 / 1671 to 1836 cycles more at 100 kHz, 226 /
  238 or 239 at 400 kHz and 91 / 103 to 105 at 1 MHz - the disable
  waits on the peripheral's own shutdown, which follows the bus timing. The
  longest host entry is 301 / 288 cycles, a 255-byte read's. With the
  vector in the flash too (`bench_rp2350`, the default image) the
  one-byte write's fixed cost reads 403 / 411 and its
  handler 96 / 70, a 255-byte write 31 interrupts on both architectures at
  400 kHz. **Warm, the vector's placement moves nothing in this image**: the
  one-STOP handler reads the same 96 / 70 cycles from the flash and from SRAM.
  The letter counts no miss of the XIP cache, through which the core executes the
  flash, over the Cortex-M33's tenure and one miss over Hazard3's, in
  both images; which line, it does not say. That is this
  layout's: a warm figure from the flash holds whatever set conflicts the
  image's code makes ([platform.md](platform.md), "A handler in SRAM"). Cold, it
  matters on both: the same handler after every line of the XIP cache is
  invalidated reads 2880 / 1649 cycles from the flash and 96 / 70 placed.
  Where it runs is the program's: the vector it binds, and `I2cHost`'s placement
  tag for `start()` and the bodies. The one-byte write's longest entry, warm then
  cold, reads 96 then 2880 on the Cortex-M33 and 70 then 1649 on Hazard3 with both in the
  flash, and 119 both times and 80 both times with both placed. `bench_rp2350` runs the handler from
  the flash, `bench_rp2350_ram` from SRAM.
- **The vendor's tenure on the same wire:** the pico-sdk 2.3.1's polled
  `i2c_write_blocking` / `i2c_read_blocking` (its `i2c.c` compiled from
  the pinned archive for each architecture, `clock_get_hz` answered with
  clk_sys, brio's crt, clock, console and ruler around it) on I2C0,
  GP36/GP37, against the same peer, commanded once to answer a run of bytes that
  holds each tenure; the
  SDK's own `i2c_init`, then brio's counts written over its own, so the
  SCL period measures the same (378.9 cycles at 400 kHz on both). At
  400 kHz, fixed cost, the SDK against the default image (the table's
  V image in brackets), Cortex-M33 / Hazard3: a one-byte write 362 / 354
  against 403 / 411 (405 / 412), a 255-byte write 366 / 357 against
  403 / 411 or 412, a 16-byte read 767 / 853 against 505 to 508 / 513 or 514, a
  255-byte read 13 675 / 15 203 (x 1.02) against 493 to 499 / 501 to 506
  (x 1.00), a register read of 1 + 1 -16 / -45 against 451 or 452 / 456; an
  address not acknowledged 348 / 317 against 563 / 592. The one-byte write's gap over the
  SDK's fixed cost is within a fifth, the size the project calls a finding, from either
  image (11 and 12 % on the Cortex-M33, 16 % on Hazard3); it is
  what the SDK does not do - no interrupt, no wake from the idle loop. For it
  the SDK holds the core the whole wire, busy 7714 / 7706 cycles against
  677 / 329 from the default image (529 on the Cortex-M33 at 1 MHz, the step above not
  in it). Its fixed cost for a register read of 1 + 1 is
  below zero because `i2c_read_blocking` returns at the last byte,
  before the NACK and the STOP are on the wire; its long read keeps one
  read command in flight, waiting for each byte before the next command,
  where the pump pours them by the FIFO.
- **The engines** (letter g, every request naming the mechanism, ten runs of z
  on each architecture, the same counts in each):
  the shortest reads under `dma`, one byte and two, on the receive
  engine, its run the length and every byte of it moved, the transmit
  engine never started, one host entry and no DMA line entry; a write of
  1 under `dma` on the interrupt, no DMA line entry, and one of 32 on the
  transmit engine, one DMA line entry and two host entries, the
  peer's count and sum exact; `interrupt` at 64 bytes on the engined
  host touches no channel - no DMA line entry, the receive run and its
  channel's write address unchanged - in eight host entries; a write-then-read of
  6 + 16 under `dma` whose third written byte the client refuses, the
  read half already on the engines, is `i2c_nack_data` with two bytes
  stored and the next engined read exact; and an engined read losing the
  arbitration inside a window held by hand is `i2c_arb_lost` (letter k,
  [../rp2040/i2c.md](../rp2040/i2c.md) for the race it closes).
- **A pure write on the transmit engine** (letters y, n, o and v, each with its
  control, ten runs of each on each architecture outside `z`, all green, and in
  the ten runs of `z`). The bytes: 3, 4, 17, 18, 64 and 255 at 400 kHz and 255 at
  100 kHz and 1 MHz, each read back by the partner against the pattern at its
  position, none off in 871 bytes, the sum and the count exact; one DMA line
  entry each, one host entry for a DMA block that left room for the last entry (3,
  4 and 17) and two where it did not, TX_OVER never raised; a read of 16 between
  them on the same channel exact, its halfword commands through the same address.
  A write of 255 takes 5843 to 5854 us at 400 kHz, 23 141 to 23 153 at 100 kHz
  and 2386 to 2398 at 1 MHz, within 1.4 % of the wire's time. The control, the
  channel bound at the register's own address, 32 bytes of the same pattern:
  the partner receives 17, 16 of them off the pattern, and serves 15, the read
  commands the replicated lanes made.
  A NACK in mid-write: the twentieth of 255 refused at 100 kHz is `i2c_nack_data`,
  nineteen bytes stored exact, one host entry and no DMA line entry, the channel
  stopped, and the next engined read exact. The abort flushes the transmit FIFO and
  holds it flushed until it is read away, and that leaves the DMA request standing,
  so what remains of the DMA block pours into nothing while the host's vector comes. A
  DMA block of 253 entries, some 217 to go at the twentieth byte, is still running then, and
  the vector stops it before the abort is read away, which keeps the rest from starting a
  tenure of its own. A shorter DMA block can end first: on the Cortex-M33 a write of 64,
  some 26 of its 62 to go, ended before the vector, its end on the DMA line a harmless
  entry; on Hazard3 it did not.
  The first byte refused is
  `i2c_nack_data` with nothing stored, a deaf address on a write of 32 `i2c_nack_addr`.
  The control, nothing refused: `i2c_ok`, the 255 stored exact.
  A lost arbitration: a write of 255 aimed at the higher address, inside a window held
  by hand with the first entry and the DMA block already in the FIFO, is `i2c_arb_lost`,
  the channel stopped, the peer's host keeping the bus. The control, aimed at the lower
  address: this host keeps it, `i2c_nack_addr`, the peer's reporting the loss.
  A bus error: a write of 16 from the last byte of SRAM (`start()` reads the first
  entry, the channel's first beat reads past SRAM_END, 2.2.3) is `i2c_dma_fault`, on the
  DMA line alone with no host entry, the channel stopped, the partner storing the one
  byte the peripheral ever held. The control: the next write from RAM `i2c_ok` and exact.
- **The engines' fault path** (letter g): a 64-byte engined read whose
  receive run points into a hole in the map (0x40100000) is a bus error
  on the receive channel; `dma_isr()` aborts the tenure and answers
  `i2c_dma_fault` with no host interrupt, the peer having served 3 bytes by
  the abort in twenty-five runs on each architecture, and the next engined
  read is clean.
- **A released DMA request, the same owner again** (letter r, ten runs of four rounds on each architecture,
  the second chip serving): the host released the worst way, both DMA enables of IC_DMA_CR
  set, its channels stopped, the transmit FIFO empty and so its DMA request standing, the
  peripheral held in reset. Then `init()`: a channel bound on the transmit DMA request, pouring
  into a cell of RAM and not into the peripheral, moves none of four in 20 us, and after `init()`
  again an engined read of 16 bytes at 400 kHz is exact. The control, the reset removed: the
  peripheral disabled and enabled with the enables left set, the same channel moves one item at
  once, the DMA request standing, and no more, since a cell of RAM gives the peripheral nothing
  to clear the request with; nothing of the control reaches the wire. That is the DMA request a
  reset at `init()` withdraws ([dma.md](dma.md)).
- **The engines' first read is the XIP cache's fill.** A 64-byte engined
  read at 400 kHz takes 1484 to 1486 us (393 or 394 kHz) with two host
  interrupts. Letter g's first takes 1529 to 1541 / 1527 to 1532 us
  (Cortex-M33 / Hazard3) inside the key `z`, whose other letters run
  before it, and 1493 to 1548 / 1510 to 1532 with the letter run alone.
  The same read with every line of the cache invalidated first,
  printed third, takes 1535 to 1542 / 1522 to 1525. Two host interrupts in
  every case, over ten runs of `z` and fifteen of the letter on each
  architecture; the RP2040's reading in a larger size ([../rp2040/i2c.md](../rp2040/i2c.md)). The letter
  judges the second.

### The receive channel at the STOP (`test_rp2350_i2c` letters `u` and `s`)

A `dma` read has no completion of its own: the receive engine reports errors alone, and
STOP_DET ends the tenure. STOP_DET says the wire is over and not that the channel is, and on
this chip a load a correct program may run leaves the channel far behind it. What the STOP's
entry does about that is the IP stratum's
([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md), "The receive channel at the STOP"): one load
of CTRL.BUSY, and a channel still counting stopped by the abort, what stands in the FIFO taken by
hand. This section is what the channel does here. **Late** below is BUSY up at the vector's
first load. The host's channel arms at normal; HIGH is a copy armed
`DmaCopyEngine::arm(report, true)`, a public verb.

**A load makes a channel late by itself.** Letter `u`: reads of 1, 8, 16, 64 and 255 at each
speed, thirty of each a run, beside copies started again as they run out, and between them
writes of 8 and 64 under `dma`. Late reads of 1800 in a cell, over 12 runs,
Cortex-M33 / Hazard3; byte beats, blocks of 16 KB unless the row says otherwise.

| load | late at 100 kHz | at 400 kHz | at 1 MHz | most beats owed |
|---|---|---|---|---|
| nothing else; one HIGH, SRAM or the flash; two normal, SRAM or the flash | 0 | 0 | 0 | 0 |
| two HIGH, SRAM | 1678 / 1664 | 1772 / 1776 | 1796 / 1793 | 16 / 16 |
| two HIGH, SRAM, blocks of 1 KB | 628 / 529 | 1118 / 1288 | 1503 / 1461 | 3 / 2 |
| four HIGH, SRAM | 1762 / 1769 | 1800 / 1800 | 1800 / 1800 | 16 / 16 |
| two HIGH, SRAM, core 1 writing SRAM | 1621 / 1651 | 1795 / 1683 | 1796 / 1716 | 16 / 16 |
| two HIGH, an APB register read across the bridge | 1746 / 1758 | 1799 / 1785 | 1799 / 1795 | 12 / 16 |
| four HIGH, the same | 1800 / 1791 | 1800 / 1800 | 1800 / 1800 | 16 / 16 |
| four HIGH, the same, core 1 writing SRAM | 1762 / 1778 | 1769 / 1778 | 1754 / 1777 | 16 / 16 |
| two HIGH, out of the uncached flash, blocks of 256 bytes | 1706 / 1614 | 1690 / 1692 | 1788 / 1799 | 16 / 16 |
| four HIGH, the same | 1792 / 1797 | 1799 / 1780 | 1708 / 1780 | 16 / 16 |
| four HIGH, the same, core 1 writing SRAM | 1760 / 1800 | 1800 / 1788 | 1800 / 1800 | 16 / 16 |

Late were 50941 of 81000 / 50842 of 81000 reads, and every one of them was exact at the reply, every cell, with
nothing written after it (a millisecond later, the load still running), nothing standing in the FIFO
and the channel stopped; the partner's tally agrees on every row. One HIGH copy, or two normal ones,
never made a channel late. A wait of this size is not the round 12.6.10 describes, and
[dma.md](dma.md) has why that is still open: a normal channel beside two or more HIGH byte
copies waits about as long as a block of them takes.

**How near, polled.** The vector's line off, a read of one with the request held down until
the byte stands in the FIFO and then let up, a masked poll of BUSY and STOP_DET, twenty tenures a
row. A turn of the poll is three register loads, at most 783 / 609 cycles, and 4255 / 3550 with a load
reading the flash the core fetches from; every figure is good to that.

| load, on 150 MHz, Cortex-M33 / Hazard3 | BUSY down, at most, cycles from the let-up | STOP_DET, at least, at 100 kHz, 400 kHz, 1 MHz |
|---|---|---|
| nothing else | 689 / 508 | 1337 / 1382, 331 / 378, 44 / 126 |
| two HIGH, SRAM, blocks of 1 KB | 2462 / 2634 | 1200 / 1453, 324 / 260, 41 / 127 |
| two HIGH, SRAM, blocks of 16 KB | 50580 / 50533 | 1312 / 1447, 321 / 346, 41 / 127 |
| four HIGH, SRAM | 99712 / 95303 | 1276 / 1437, 265 / 259, 41 / 83 |
| four HIGH across the bridge | 203548 / 160649 | 1267 / 718, 328 / 334, 100 / 94 |
| two normal, the flash, blocks of 256 bytes | 4967 / 3875 | 640 / 799, 640 / 547, 640 / 107 |
| four HIGH, the flash, the same | 73239 / 71003 | 716 / 877, 716 / 1033, 716 / 955 |

Blocks of 1 KB beside blocks of 16 KB are the wait following the block. With 16 KB read out of the
uncached flash, a beat there some 150 cycles, two HIGH copies took BUSY's fall to 2.4 million
cycles (16 ms) and four to 5.2 million (35 ms) in one run, and a read of 255 did not finish in
200 ms, the FIFO full and SCL held. The letter's own flash blocks are 256 bytes, to keep its rows
short. A program under such a load gets a slow bus: every byte, or `i2c_timeout` from the arbiter's
per-bus timeout if the load outlasts it.

**The control, a build whose entry answers `i2c_ok` with no read of the channel.** One run of letter
`u` over the same rows: 4011 of 6750 reads wrong under `i2c_ok` on the Cortex-M33 and 3862 on Hazard3,
2909 and 2610 with a cell written after the reply, 4184 and 4003 with the channel still running at
the reply, up to sixteen cells stale. None on a load the table has at zero.

**The stage** (letter `s`, 12 runs on each half). Eight shapes (reads of 1, 2, 3, 8, 16, 17 and 64, a
write-then-read of 1 + 8) under four tails (the last 1, 2, 8 or 16 beats), at the three speeds, each
followed by a read of eight and a write of eight. The tail is staged five ways: the request taken
down, the channel's EN taken down, EN let up behind the reply, EN let up in the vector 0, 4 or 16
turns ahead of the host's body so that the beats land across it, and a byte popped ahead of the body.

| stage, 32 reads a speed | the control, one run, whose entry reads no channel | the entry |
|---|---|---|
| the request down for the tail | every read wrong under `i2c_ok`, 139 cells at each speed, every channel left running, the FIFO not empty; the read behind wrong, every one | every byte in its place, the channel stopped, FIFO empty, the requests behind whole |
| EN down for the tail | the same | the same |
| EN let up behind the reply | every read wrong at the reply and written after it; the read behind wrong in 21 of 32 | the same, and nothing moves |
| EN let up 0 or 4 turns ahead of the body | 3 of 32 wrong on the Cortex-M33, up to 5 on Hazard3 | the same |
| a byte popped ahead of the body | `i2c_ok` with the byte missing, every read; the read behind wrong in 21 of 32 | `i2c_dma_fault`, the channel stopped, the requests behind whole |

BUSY polled down, then the cell: stale in none of 400 rows, half under two HIGH copies, and the same
for TRANS_COUNT polled to zero ("updating automatically each time a write transfer completes",
12.6.10). A channel paused over a read of four banks one request, it stands behind `IC_DMA_CR`
going down, and with EN let up a beat runs on it: the counter "decrements when the transfer is issued"
(12.6.4.2), which is why the entry stops the channel before touching the FIFO. That is the stray beat's
answer too: `IC_DATA_CMD` read over a stopped channel with the request up, or down, 200 rows of each,
and the next block's start moved nothing and the read behind was exact.

**Reads and writes of seventeen at 100 kHz** fill the FIFO at their block's end, and the entry with
the STOP then waits for TX_EMPTY. That end can overtake `start()`, whose write of the mask would then
take back the arm the DMA line just made, and nothing would write the entry; the host looks at the
arm behind that write ([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md), "The DMA requests"). Without the
look a read of 17 stalled once, unstaged, over fifteen runs of letter `s` on Hazard3 (`i2c_timeout` under the
arbiter). Staged, TX_CMD_BLOCK holding the first entry and the XIP cache emptied over `start()`: the
control left 200 of 200 tenures unanswered on both halves, and with the look none.

**What it costs.** In the release listing the entry adds one load of the channel's CTRL, a test and a
branch to every engined tenure's STOP, and `start()` on an engined host one load and a test of the
TX_EMPTY mirror. The tables above hold both. Letter i's read of one on `dma`, against a control build
without them: 384 cycles against 373 on the Cortex-M33, `start()` 279 against 271 and the vector 105
against 102; 357 against 349 on Hazard3. The late path, out of line in the flash, read at most 4927 cycles on
the Cortex-M33 and 4913 on Hazard3 between stamps in the vector over letter `s`, the abort's
poll and the XIP's misses in it. It waits for nothing else.

A write takes the same two loads. The transmit channel's BUSY at the STOP entry reads down in every row
of letter `u`, reads and writes alike: the last entry is the pump's, written behind the block's end.

The same body on the RAM image (`test_rp2350_i2c_ram`, the engined host and the two vectors in
SRAM, the late path called out to the flash): letters `u` and `s` the same on both halves, 3 runs.

### The stack under the preemptive kernel (`test_rp2350_rubato_i2c`)

Measured by `test_rp2350_rubato_i2c`, one source and two images:
`util/i2c_bus.hpp`'s arbiter (pending depth 4, the per-bus timeout at
20 ms) over the engined `I2cHost<0>` on GP36/GP37 (DMA channels 4 and 5
on DMA_IRQ_0, clk_sys at 150 MHz) inside `Rubato`, brio's preemptive kernel
([../design/kernel.md](../design/kernel.md)), on core 0, against the
WeAct RP2040's `twi_peer`, while TIMER1's alarm 0 posts to the highest of the
suite's five active objects every 97, 193 or 389 us and its dispatch spins 300 or
3000 cycles - the pressure, at 97 us x 3000 21 % of the core; letter `s` adds a
dispatch that spins 160 us every 389 us. Every letter runs once with
the alarm silent and six times under it. **209 verdicts in the key `z`,
green in ten runs of ten on each architecture**, the suite's pure writes of
three bytes or more on the transmit engine but for a write of exactly sixteen,
which letters `s` and `x` measure on the interrupt; the letters `x` and `s`, outside
it, green in fifteen runs each on each architecture. A letter in this section is
this suite's. The five objects,
the command channel through the arbiter and the letters are the I2C
suites' shared body (`avrxt/src/apps/rubato_i2c.hpp`); what the two
Raspberry Pi chips share over this peripheral is
`rp2040/src/apps/rp_rubato_i2c.hpp`, whose header derives every bound,
and the RP2040's run of it is [../rp2040/i2c.md](../rp2040/i2c.md)'s.

**A preemption inside `start()`'s fill: no stretch at 400 kHz.**
`start()`'s first entry starts the tenure, so a preemption after its
k-th entry leaves the peripheral (1 + k) byte times before it needs the next,
and only a longer one holds SCL ([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md),
"The host"). Letter `s`, Cortex-M33 / Hazard3, on `start()` as
the suite wraps it (one copy of the Request and up to four compares to name the
mechanism, `rp2040/src/apps/rp_rubato_i2c.hpp`'s `EngineByLength`, the host's own cost
of the choice being 5 or 6 cycles in `bench_rp2350` letter i's warm lines) with the
host in the flash: `start()` averages 404 to 463 / 363 to 385 cycles for a
one-byte write and 484 to 513 / 456 to 458 for a write of sixteen over fifteen runs.
Those averages hold the XIP misses of the image's layout, so their difference (38 to
90 / 71 to 93 by the run) is no measure of the fifteen entries after the first: by
the write table's 10 cycles a byte those are 150 cycles, where 400
writes under independent ticks of 97 us would land some 4 times. Under
97 us x 3000, the longest preemption of `start()` 3243 to 3471 / 3289
to 3442 cycles against two byte times' 6750 at 400 kHz: 10 to 21 / 15 to
26 landings in `start()` - its whole span counted - in 400 writes, 4 to
9 / 4 to 14 with the host active on the wire, and **no stretch in
fifteen runs on either architecture**. At 1 MHz (two byte times 2700 cycles)
the Cortex-M33 landed once in `start()` in fifteen runs at 97 us x 3000 and
Hazard3 four times, once each with the host active and no stretch; neither
landed under the 160 us spin at 389 us. Independent ticks would land some twelve
times in a run of 400 writes. The tenure cycle there is not independent of the
alarm's period, and which part of it sets the phase is not established;
the letter's long-spin case is judged here only by its exactness, and
its stretches are the RP2040's measurement. Letter `x`, 400 writes of
sixteen at 400 kHz (`start()` 386 to 388 / 457 to 460 cycles silent, letter `s`'s
hundred more on the Cortex-M33 not split): 15
to 22 / 15 to 27 landings against the 10 / 12 independent ticks
would give, 10 to 17 / 1 to 13 with the host active, no stretch. The
key `z` stretched no tenure in ten runs on either architecture.

**The held wire parks the START.** Letter `h`, both architectures: the suite's
requester's write into the peer's 60 ms hold, after the disable, IC_TAR and enable
cycle its address takes, answered `i2c_timeout` 19.95 to 20.06 ms
after its `start()` in every pass of ten runs, and the same met at
400 kHz after commands at 100 kHz, ten of ten; the flood of
probes answered
`i2c_rejected` while the queue is full, one `i2c_timeout` a pass, and
after the release `i2c_ok` or, once a pass, `i2c_nack_addr` from a peer
not yet back. The parked one-entry tenure's false end on another node's
STOP that the RP2040 measured ([../rp2040/i2c.md](../rp2040/i2c.md)) was
not seen on this chip: none in twelve passes under pressure on the Cortex-M33
of a build whose STOP test lacked the state machine's idle, and none in
120 on the two architectures with it (ten runs of `z` each).

**A client disabled to refuse needs some time before it answers again.** Letter
`q`'s control pass writes six bytes to a peer refusing the third, then probes it at
once. The peer enables its client again once the disable is reported done, after the
host's STOP, and that path is in SRAM in `twi_peer` (`refusal_end()`): letter `q` is
green on both architectures in ten first runs after both boards are flashed and in ten
later runs. With that path in the peer's flash, the Cortex-M33's probe met the client
still disabled, `i2c_nack_addr`, in nine first runs of nine and in none of nine later
runs; a cold cache in the peer is inferred from that, no log reading the cache. Hazard3 and
the RP2040 as the host never met it, and why is not established.

**Every shape, and three requesters, read under every pressure as in the
control.** Letter `o` at 400 kHz, one commanded run of bytes from the peer counted to its end - the
suite naming `dma` for a request with a half of two bytes or more and
`interrupt` otherwise, so the shapes take every path the host has
(`rp2040/src/apps/rp_rubato_i2c.hpp`'s table names each) - every
shape `i2c_ok` and byte-exact, the peer's count, sum and seventeen
address matches exact; post to reply over ten runs, the control's
and the worst under pressure, Cortex-M33 / Hazard3: the probe 68 to 75
and 72 to 95 / 73 to 77 and 72 to 95 us, the 32-byte write, on the transmit
engine, 764 to 766 and 769 to 803 / 761 to 763 and 763 to 805, the 32-byte read
763 to 765 and 769 to 804 / 762 or 763 and 763 to 804. Letter `w`: every reply
`i2c_ok`, to its requester, in its order, the peer's tallies exact, 5.4
to 6.1 / 5.3 to 6.0 ms a pass. Letter `p`: the peer's commanded stretch
of 1000 us a byte on a read of 8 in 8788 to 8813 / 8789 to 8799 us
against the 8762
its eight holds take at least.

## Not covered yet

Driver gaps, each with its reason. The ones that belong to the peripheral
rather than to this chip - the high-speed mode, SMBus, the stuck-bus
timeouts, 10-bit addressing on the host side, the client on the engines -
are in
[../dw_apb_i2c/README.md](../dw_apb_i2c/README.md) and are not repeated
here.

- The START byte and the general call as host verbs: `target(addr,
  general_call)` is in the resource, and `start()` refuses both
  addresses with every reserved one
  ([../design/i2c-bus.md](../design/i2c-bus.md)); a broadcast is a
  feature of its own, born with its first user.
- Two clients on one address with a third party reading: the suites' wire has
  no third node, and a DesignWare peripheral is a host or a client, never both.
- This chip as the bus's target, served from an active object, under
  the preemptive kernel: a target whose bytes come from an object (its
  vector posting "the host reads" and leaving SCL held, a dispatch giving
  the byte) is the one place a held-off reader would stretch the clock,
  and no util service serves a target that way (`I2cClient` is served
  from its vector). The partner's `twi_peer` could read it (its
  `host_read`). Born with the first util target object.

Implemented but not bench-verified, each with what would measure it:

- What a `dma` tenure costs the chip's SLEEP state: a meter on the rail, the same
  tenure under either value, would measure it.
- The client's general call (`ack_general_call`): no brio host sends a
  general call, the partner's included, so no suite reaches this
  peripheral's GEN_CALL handling ([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md)
  says what the RP2040 showed of it); a controller that sends one would
  measure it.

- `rebase()`: this stratum has no dynamic clock; measured when the
  clock chapter grows one, a tenure exact after a switch.
- The QFN-60's pin table: the stratum compiles for that package and
  refuses the pads it has not got, but no QFN-60 part is on the bench.
