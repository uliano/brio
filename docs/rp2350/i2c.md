# I2C (RP2350)

This document is for a programmer who puts a device on an I2C bus of an
RP2350 - a sensor, a converter, a memory - or who makes this chip answer
as a device on another chip's bus, on either of the chip's two
processor architectures. The chip carries two Synopsys DW_apb_i2c
controllers. Their driver is written once for every family that carries
the block, in the DW_apb_i2c IP stratum
([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md)); this page and
`brio/rp2350/i2c.hpp` are what the RP2350 adds to it - the pads and
their setup, the reset bits, the interrupt lines, the DMA request
numbers, which clock the bus timing is counted in, and the open-drain
verbs a bus clear by hand needs - and the names a program writes:
`DwApbI2c<n>`, the instance; `I2cHost<n, pins, ...>`, the host engine;
`I2cClient<n, pins>`, the other end. A host is not usually driven
directly: a program names one arbiter per bus, `I2cBus` (an active
object - a component of a brio program with its own event queue, served
one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md) - that takes the
transactions of every client of the bus in turn and answers each with
an event, [design/i2c-bus.md](../design/i2c-bus.md)), and posts it a
request.

The smallest use, a register read from a device at 0x48 through the
arbiter, is the first block under "How to use it" below:

```cpp
constexpr brio::I2cPins pins{.scl = 13, .sda = 12};
using Bus = brio::I2cHost<0, pins>;
using I2c = brio::I2cBus<Bus, P, 4, brio::BusPassThrough, brio::ticks_from_ms<P>(20)>;

Bus::init(clock);

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

Line by line: `pins` names I2C0's clock and data pads by number (a pair
that is not the instance's does not compile); `Bus` names the host
engine with every other template argument at its default (no DMA
engines, the code in the flash); `I2c` is the arbiter for that bus, an
active object of the program's `Tenuto` over platform `P`, with four
requests of queue, every completion sent straight back to its
requester with no retry (`BusPassThrough`, the default policy) and a
timeout of 20 ms for a transaction that never
completes; `Bus::init()` takes the clock task's tag
([clock.md](clock.md)); the request names the device's address, lends
the register number to write and the buffer to read into
(`brio::lend<brio::Lease::reply>` - the buffers stay the caller's until
the reply), the speed, and where the reply goes (`Sensor`, the active
object that posts it, receives an `I2cDone`); `post` hands it to the
arbiter; and the `extern "C"` function is the instance's interrupt
handler, under the crt's name for I2C0's line, the same on both
architectures, which tells the arbiter when the transaction has
completed.

What decides correctness even there. `init()` comes after the clock
task: the bus timing is counted in clk_sys cycles at the rate the clock
type claims, not read from the hardware. `init()` answers false when not
even standard mode can be produced (a clock under 2.7 MHz) or the block
did not come up, and the example discards that answer; a speed the
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
checks it. The request is posted from an active object's handler, which
is what the reply is sent back to. `init()` hands both pads over itself,
the isolation latch dropped and the pull-up 12.2.1.3 asks for on, but
the pulling is the board's ("What the silicon does" below). And a
program that calls `unstick()`, the bus clear by hand, must first start
the microsecond ruler that paces it (`Mtime::start(clock)`, the second
block below): on the Cortex-M33 half nothing else starts it.

The rest of this page, in order: the documents of record; what the
silicon does (the one function column, the isolated pads, the bank in
two words, the DREQ numbers and the reset bits, the shared interrupt
numbering, the alias a write rides the DMA through, the bus clear by hand
and its ruler, erratum RP2350-E9); the
types and verbs this stratum adds; how to use it, the host, its
engines, the ruler and the client; the bench findings, against the desk
partner; and what is not covered. What the block itself does is the IP
stratum's page.

## Documents of record

The RP2350 datasheet (build d126e9e), 12.2 (the Synopsys DW_apb_i2c
v2.03a: 12.2.1 the feature list with the FIFO depths and the pad setup,
12.2.5 the behaviour - START and STOP generation, the combined formats
-, 12.2.7 the transmit FIFO and the RESTART / STOP bits of a command,
12.2.10 the operation modes, 12.2.11 the spike filter, 12.2.12
fast-mode-plus, 12.2.13 the bus clear, 12.2.14 the SCL counts with their
floors and the cycles the block adds, 12.2.15 the DMA interface, 12.2.16
the interrupt registers, 12.2.17 the registers), 9.4 (the GPIO function
table), 12.6.4.1 (the system DREQ table), 7.5 (the reset controller),
3.2 (the interrupt lines), 2.1.3 (the atomic register aliases), 2.1.5
(narrow writes replicated across a register, and the alias that zero-fills
them), 3.1.3 and 9.8 (the
single-cycle IO the unstick drives an open-drain line through), 9.7 (the
pad isolation latches); [../design/i2c-bus.md](../design/i2c-bus.md) for
the Request and the vocabulary. The driver is Synopsys's and not this
chip's, so it lives in the IP stratum: `brio/dw_apb_i2c/i2c.hpp` holds
the resource, the host engine and the client
([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md)), and
`brio/rp2350/i2c.hpp` holds what this chip owes it - the pin table of
9.4 with its pad configuration, the `I2cPad` type that adds the two
open-drain verbs to a `Pin`, the `Rp2350DwApbI2c` chip traits over
`pin.hpp`, `resets.hpp`, `core.hpp`, `delay.hpp` and `dma_engine.hpp`,
this chip's device description held against the bits the IP file spells
itself, and the public names `DwApbI2c<n>` (the resource), `I2cHost`
(the engine `I2cBus` drives) and `I2cClient` (the other end), this
chip's aliases of the three templates there. The reference suite:
`test_rp2350_i2c`, which runs on both of this chip's architectures from
one source: the block wireless, its pads, and the wire to the desk
partner - I2C0 on GP36 (SDA) / GP37 (SCL), 1.5 kOhm to the WeAct
RP2040's 3V3 on each
([../boards/weact-rp2350b.md](../boards/weact-rp2350b.md)), whose peer
of the twi_link protocol is commanded over the bus under test; that
board's own suite, `test_rp2040_i2c`, judges this chip's block as that
peer. A letter of a suite is one command key of its console and the
group of verdicts it runs; `z` is the key that runs every letter but
`x`, the hand-made window traced, which runs by name.

This page says what is the RP2350's. What the block itself does - the
command FIFO, the counts and their floors, the single TX_ABRT, the
client's clock stretch, the engine parameters - is the IP stratum's page and
is not restated here.

## What the silicon does

Two DW_apb_i2c controllers, the same design at the same IP version the
RP2040 carries and with the same synthesis: a host or a client and never
both, standard, fast and fast-mode-plus (no high-speed mode), 7-bit
addresses on both sides and 10-bit ones, two FIFOs of sixteen entries
(12.2.1), one interrupt line per instance with thirteen sources, a DMA
request per FIFO. 12.2 has no "Changes from RP2040" section, where the
SPI, the ADC and the DMA chapters all do, and the two chips' register
descriptions agree define for define - which is why the driver is one
file in an IP stratum and this one is a page of differences. There are
six, and none of them is inside the block:

**The pins are one column, over a longer bank.** Every fourth pad from
GP0 carries an I2C signal - SDA on the even pads, SCL on the odd, the
instance alternating in pairs - so I2C0 has SDA on GP0, 4, 8, ... 44 and
SCL on 1, 5, 9, ... 45, and I2C1 has SDA on 2, 6, 10, ... 46 and SCL on
3, 7, 11, ... 47. That is the RP2040's rule over forty-eight pads instead
of thirty: twelve SDA pads and twelve SCL pads per instance against eight
and eight. What the UART gained here, this block did not - the UART's
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
controller governs twenty-nine blocks against twenty-seven, and I2C0 and
I2C1 sit at bits 4 and 5 where they sat at 3 and 4. Both come from this
chip's own headers, through `dma_engine.hpp` and `resets.hpp`.

**The interrupt numbering is shared between the architectures** (3.8.4.2),
so I2C0_IRQ is line 36 and I2C1_IRQ line 37 whichever processor pair is
running - not the RP2040's 23 and 24 - and an app binds `isr_i2c0` /
`isr_i2c1` once for both. The controller behind those names is not the
same object, an NVIC on the Cortex-M33 half and Hazard3's own on the
other, which is exactly why the IP stratum's concept takes the controller
as a type.

**A byte beat can land as a plain write entry.** A write entry is the
request's byte with the command bits above it clear, and a byte written at
IC_DATA_CMD's own address reaches it replicated across the word (2.1.5, and
12.6: the DMA has one transfer size for both sides), its copy in bit 8 a read
command, in bit 9 a STOP, in bit 10 a RESTART. This chip's fabric offers the
way out the RP2040's does not: at +0x4000, bit 14 of the address, "invalid byte
lanes [are] driven to zero" (2.1.5). 2.1.3 says the I2C's atomic aliases are
served by a bus interposer and does not list this one among them, so what it
does on this block is a measurement, on both architectures (letter y of
`test_rp2350_i2c`): byte beats through it moved 3 to 255 bytes at the three
speeds with every one at its position on the partner, and the same run at the
register's own address does not move those bytes. So the transmit engine is
bound there and a pure write can ride it from the request's own buffer, with
no copy. A read's halfword commands go through the same address: the upper
lanes are zero instead of a copy, in a part of the word the block does not
read.

And one thing that is not a difference in the block but changes what a
program must do: **the unstick is paced by the platform timer.** The bus
clear of 12.2.13 has no enable in this chip's IC_CON, whose writable bits
are 0x7FF and stop at the eleventh, so the nine clocks and the STOP
are driven by hand, half a bit of standard mode at a time, and the ruler
they are paced by is `rp2350/delay.hpp`'s. That file counts the platform
timer of 3.1.8 and not CPU cycles, because that counter is the one ruler
both architectures have; it follows that a program which calls
`unstick()` must have started it (`Mtime::start(clock)`), which the
Hazard3 half's ticker already does and the Cortex-M33 half's does not.
With the counter stopped every wait is refused and spends no time, and
the nine pulses go out at the speed of two register writes - legal on the
wire, far faster than a bus, and measurable.

**Erratum RP2350-E9 changes how a wire is proven, not how it works.** On
stepping A2 a pad whose input buffer is enabled and which nothing drives
leaks enough current to sit high against its own pull-down, so an idle
level is history and not a measurement. An I2C line idles high anyway,
pulled there by the board, and the driver hands both pads over with the
pull-up the chapter asks for, so nothing the block does is coloured by
the leak: a line that reads low is genuinely being driven low, which is
all `unstick()` asks. What the erratum does change is a program's ability
to ask "is a pull-up fitted at all", because a bare pad answers the same
as a pulled-up one. The errata sheet's own workaround is the instrument:
the pad's pull-down on with the input buffer off, the buffer enabled for
the read alone (`Pin::read_pulsed()`, [pin.md](pin.md)) - the leak needs
the buffer on, so only an external pull-up reads high that way. It drives
nothing, which is why it is also how an image here asks whether the desk
partner is powered.

## Types and verbs

What the IP stratum names (the speed vocabulary, the timing arithmetic,
the register bit layouts, the resource's verbs, the host engine's and the
client's) is in [../dw_apb_i2c/README.md](../dw_apb_i2c/README.md). This
stratum adds:

- `I2cPins` - the two pad numbers, SCL and SDA. One column, so no
  function travels with them.
- `i2c_sda_pin(n, pin)`, `i2c_scl_pin(n, pin)` - the table of 9.4 as
  constexpr facts over the whole bank, and `i2c_pins_valid(n, pins)`,
  which an engine's `static_assert` calls. The package is not asked
  there: the table is the die's, and a pad the QFN-60 has not bonded is
  refused one level down by `Pin<n>`'s own static_assert, which says so
  in those words.
- `i2c_pad_config` - the pad setup 12.2.1.3 asks for, as this stratum's
  `PinConfig`. Writing it is what takes the pad out of isolation.
- `I2cPad<pin>` - a `Pin<pin>` with the two verbs an open-drain drive by
  hand needs, each choosing the half of the bank its pad falls in.
- `Rp2350DwApbI2c` - the chip traits, which nothing above this file
  names: the register block, the reset bits, the interrupt lines and
  their controller, the DREQ numbers, the atomic aliases, the pad type
  and its function, the microsecond ruler, and ic_clk = clk_sys
  (12.2.1.2).
- `DwApbI2c<n>`, `I2cHost<n, pins, TxEngine, RxEngine, Placement>` and
  `I2cClient<n, pins>` - the public names, this chip's aliases of the
  three IP templates, with `NoDmaEngine` in both engine parameters and `InFlash` by
  default (`Placement`: util/placement.hpp's tag, [platform.md](platform.md),
  "A handler in SRAM"). The host's `Request` carries the engine it runs
  on, `engine`, `I2cEngine::interrupt` by default or `I2cEngine::dma`,
  and `has_dma` says whether engines are bound (the IP stratum's page,
  "The engine a request names"). `dma_carries_writes` is true on an engined
  host of this chip, and `dma_write_min`, three, is the shortest write it
  carries: a pure write of that length or more under `dma` rides the
  transmit engine. A write before a read and a write of one or two bytes
  stay on the interrupt.

The family header also holds the vocabulary check: every bit the IP file
spells for itself, held by `static_assert` against the macro this chip's
own device description gives it. That the two agree register for register
is a measurement and not an assumption a driver may make.

## How to use it

```cpp
constexpr brio::I2cPins pins{.scl = 13, .sda = 12};
using Bus = brio::I2cHost<0, pins>;
using I2c = brio::I2cBus<Bus, P, 4, brio::BusPassThrough, brio::ticks_from_ms<P>(20)>;

Bus::init(clock);

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

The same source builds for both architectures; `isr_i2c0` is a
vector-table slot on one half and a dispatch entry on the other.

With the engines: `I2cHost<0, pins, DmaTxEngine<4, uint16_t>,
DmaRxEngine<5>>`, `isr_dma_0` calling `Bus::dma_isr()` the same way, and
every request that should ride them naming them (an excerpt: the request
above on that host, reading a block of 64 bytes into a buffer `block` of
the program's, lent like `value`):

```cpp
r.rx = brio::lend<brio::Lease::reply>(block); r.rx_len = 64;   // a block worth the engines
r.engine = brio::I2cEngine::dma;                               // the read half on the DMA channels
```

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

This chip has sixteen channels and four interrupt lines, and the line is
the engine's third parameter, so a host whose engines ring on line 2 is
`DmaTxEngine<6, uint16_t, 2>` and `DmaRxEngine<7, uint8_t, 2>` with
`isr_dma_2` ([dma.md](dma.md)).

The host never chooses. A request that names nothing runs on the host's
interrupt, at any length, and touches no channel; one that names `dma`
has its read half on the engines at any length - the receive engine
collecting every byte, the transmit engine pouring the commands of a
read of three bytes or more - and, with no read half, a write of three
bytes or more on the transmit engine. Every other write half stays on the
interrupt and the request completes. On a host
with no engines bound, `dma` is answered `i2c_rejected`. A wrong choice
costs core cycles, never correctness. The core cycles each costs,
`bench_rp2350` letter i's `i2c.cost` lines (start() and the vectors, the
least of five, the host and its vector in the flash, the same at the
three speeds and in three runs but where a range is given), at 150 MHz,
Cortex-M33 / Hazard3:

| read of | 1 | 2 | 3 | 8 | 9 | 16 | 64 | 255 |
|---|---|---|---|---|---|---|---|---|
| `interrupt` | 365 / 342 | 377 / 356 | 396 / 376 | 491 / 476 | 618 / 581 | 751 / 721 | 2628 / 2418 | 10 073 / 9166 |
| `dma` | 373 / 349 | 375 / 353 | 576 / 572 | 512 / 473 | 522 / 476 | 522 / 476 | 610 / 547 | 610 / 547 |

| write of | 1 | 2 | 3 | 8 | 13 | 14 | 16 | 17 | 32 | 64 | 255 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `interrupt` | 310 / 299 | 320 / 309 | 330 / 319 | 380 / 369 | 430 / 419 | 440 / 429 | 460 / 449 | 593 / 553 | 880 / 796 | 1752 / 1516 | 6974 / 5827 |
| `dma` | 316 / 304 | 326 / 314 | 497 / 509 | 433 / 409 | 433 / 409 | 433 / 409 | 433 / 409 | 433 or 516 / 409 or 475 | 516 / 475 | 516 / 475 | 516 / 475 |

For a read, the pump, the interrupt-driven engine, costs F + b x n, F 127 / 156 cycles and b 39.0 / 35.3 a byte,
in steps of eight (one entry for each half FIFO); the engines cost E,
522 / 476 for a read of nine to sixteen bytes and
610 / 547 from sixty-four (a third entry, the transmit block's end
finding no room for the STOP entry). So `dma` costs the core fewer cycles from
**nine bytes** on the Cortex-M33 and from eight on Hazard3, by three cycles
there (the pump's second entry comes at nine on both); a read of one
or two costs the same within eight cycles either way, and one of three
to eight costs the engines a second entry (the transmit block's end).

For a write, `start()` alone writes up to sixteen entries with the block's one
interrupt at the STOP, so the pump costs 300 / 289 cycles and 10 a byte, and its
second entry comes at seventeen; the channel costs E, 433 / 409 for a write of
eight to sixteen bytes and 516 / 475 from thirty-two, whatever the length (the
third entry is the same TX_EMPTY as a read's; at seventeen it comes or not,
as the FIFO stands at the block's end). So, by the inequality of the design
page (E - F <= N x b, here 133 / 120 over 10), `dma` costs the core fewer
cycles from **fourteen bytes** on the Cortex-M33 and **thirteen** on Hazard3,
level at twelve there. A write of one or two is the pump's under either value,
`dma` adding its one compare. A write of three to eight costs the channel a
second entry; at three and four its figure reads 64 / 100 cycles higher than at
eight, in `start()`: a block of one or two beats ends before `start()` returns, so
the DMA line's entry and return fall inside `start()`'s span (inferred from
those lengths, not counted apart; a read of three and four shows the same 64 /
100). What the channel buys past the break-even is the core free on a long
write: a write of 255 at 400 kHz keeps the core busy 17 383 / 7020 cycles on the
interrupt and 2112 / 882 on the channel, 31 entries against 3, its longest host
entry 102 / 74 cycles, at the same wall time (x 1.00). The energy side: under either
value the core waits in its sleep instruction between entries, and the
chip's SLEEP state asks in addition that the DMA have no outstanding
transfer on any channel (6.5.2), which a `dma` tenure's channels hold for
its data - not measured here.

A program that means to call `unstick()` starts the ruler its half-bits
are paced by, which on the Cortex-M33 half nothing else does:

```cpp
brio::Mtime::start(clock);      // keeps the count; the Hazard3 ticker does too
```

A client:

```cpp
constexpr brio::I2cPins client_pins{.scl = 15, .sda = 14};
using Client = brio::I2cClient<1, client_pins>;
Client::init(clock, {.address = 0x42, .speed = brio::I2cSpeed::fast_400k});
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
at 150 MHz, and all of them on BOTH architectures: `test_rp2350_i2c`
reports **93 pass, 0 fail** on the Cortex-M33 pair and on the Hazard3
pair, from one source, against the WeAct RP2040's peer over the pair's
I2C wires. Nothing in this chapter reads differently between the halves,
which is what a block outside the processor should look like. The same
block as the PEER, the RP2040 the host, is judged by `test_rp2040_i2c`
(docs/rp2040/i2c.md, whose wire findings - the reserved addresses
refused both ways, a STOP_DET for every STOP, the refusal by a disable -
this block shares and are not restated here). The
cost figures are `bench_rp2350` letter i's, against the same peer.

- **THE COUNTS AT THIS clk_sys.** 100 kHz solves to HCNT 585 and LCNT 899,
  400 kHz to 135 and 224, 1 MHz to 45 and 89 - each with SPKLEN 8 - and
  each gives its asked rate exactly on an ideal wire once the SPKLEN + 8
  cycles the block adds are taken out of the counts. Table 1053's floors
  come back as the chapter states them: fast mode at 12 MHz is (LCNT 15,
  HCNT 6), standard mode at 2.7 MHz is (12, 6), fast-mode-plus needs 32
  MHz, and a hertz below each is refused.
- **THE RESET STATE.** IC_CON reads **0x65** - a host with its client half
  disabled, the fast speed code, restarts enabled - with the spike filter
  at 7, the hold at 1, both FIFOs empty and the block disabled.
  **IC_COMP_PARAM_1 reads zero**, as it does on the RP2040: the FIFO
  depths are the chapter's word and not the register's. The transmit FIFO
  is **sixteen** entries deep, measured under TX_CMD_BLOCK - full at
  sixteen, TX_OVER on the seventeenth, flushed by the disable - and the
  disable of an idle block is reported by IC_ENABLE_STATUS within **3 us**.
  The interrupt lines are **36 and 37**, not the RP2040's 23 and 24.
- **THE PULL-UPS READ THE WAY E9 FORCES.** A pad of this stepping whose
  input buffer is on holds history against its own pull-down, so an idle
  level says nothing: GP36 and GP37 are read with the pull-down on and
  the buffer pulsed for the read alone (`Pin::read_pulsed()`,
  [pin.md](pin.md)), and both read HIGH - the 1.5 kOhm to the partner's
  3V3, which is also the proof that the partner is powered before a pad
  of the link is driven.
- **THE PADS AND THE ISOLATION LATCH, ABOVE GP31.** After the hand-over
  GP36 and GP37 read **0x5A** - exactly what 12.2.1.3 asks for: the
  pull-up, the slew limit, the Schmitt trigger, ISO clear, the input
  buffer on - with FUNCSEL 3, and they carry an address phase: a probe of
  an address nobody on the bus answers is `i2c_nack_addr`. The unstick,
  on that healthy bus, reads 0 and takes the pads to SIO through the HIGH
  half's registers and hands them back with **the same 0x5A**, the next
  probe unchanged. `release()` leaves them at 9.11's reset value
  **0x116**, isolated with the input buffer off and a pull-down, and
  FUNCSEL at none.

- **ON THE WIRE, ABOVE GP31.** The scan finds the peer's 0x6B alone,
  nobody home `i2c_nack_addr` in 110 to 111 us at 100 kHz. Every tenure
  shape, the vocabulary (deaf, 0x7A and the general call refused inside
  `start()` with the peer answering the general call and matching only
  the write to its own address, a data NACK at the first and at the
  third byte), the three speeds byte-exact both ways -
  99, 393 to 394 and 965 to 970 kHz measured on the timer for 100, 400
  and 1000 asked -, the engines with eight 255-byte reads at 1 MHz and
  TX_OVER never raised, I2cBus with its timed bus (`i2c_timeout` at the
  arbiter's 20 ms, the peer holding SCL), the stuck bus (four pulses for
  a client releasing at its fourth fall; 0xFF in 125 to 132 us for a
  line nine pulses cannot free), arbitration both ways, and this board
  as the peer host's client, written to, read from and general-called -
  all of it through GP36/GP37 and the unstick's verbs on the HIGH half
  of SIO. The client's stretch prices at 747 to 760 us for eight holds of
  100 us, a write under it within 3 us of the plain one.
- **THE CLIENT'S EDGES** (letter m, the peer's host at 100 kHz, both
  halves): without the hold, 24 bytes written into a receive FIFO nobody
  drains are all acknowledged, the first 16 kept exact and EIGHT
  `overrun` events raised, one a byte lost; with the hold the same write
  stretches until the vector drains it, all 24 exact. A disable after the
  fourth byte of a write refuses the fifth - the far host's data NACK -
  with SLV_DISABLED_WHILE_BUSY and SLV_RX_DATA_LOST both set. A disable at
  a read request the client holds SCL for NEVER COMPLETES: IC_EN stays 1
  through 2 ms and 50 ms more, a byte written after it untaken, neither
  bit set; `release()` lets SCL go and the far host's read finishes on the
  pull-up - the RP2040's figures exactly
  ([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md)). Thirteen runs of z on
  each half.
- **THE HOST'S COST**, `bench_rp2350` letter i against the peer's
  client, the same image built for both halves, the host's vector placed
  in SRAM (the V image of [platform.md](platform.md), "A handler in SRAM",
  `BENCH_VECTORS_IN_SRAM` in bench_rp2350.cpp), nothing of the far end on
  this core; the wire's time at the SCL
  period measured (378.7 cycles at 400 kHz against the counts' 375: the
  pull-ups' rise, 24 to 26 ns a period at every speed). Each line of the
  letter is the best of five; each figure below the least of three runs,
  clk_sys cycles at 150 MHz, Cortex-M33 / Hazard3:

  | at 400 kHz | wall | interrupts | handler cycles | busy | fixed cost |
  |---|---|---|---|---|---|
  | write of 1 | 7752 / 7761 (x 1.05) | 1 | 96 / 71 | 532 / 327 | 404 / 413 |
  | write of 255 | 873 647 / 873 527 (x 1.00) | 31 | 6055 / 5404 | 17 142 / 7312 | 406 / 415 |
  | read of 16 | 58 975 / 58 981 (x 1.00) | 2 | 358 / 326 | 1325 / 772 | 491 / 503 |
  | read of 255 | 873 588 / 873 480 (x 1.00) | 32 | 8556 / 8541 | 22 395 / 10 467 | 346 / 357 |
  | register read, 1 + 1 | 15 149 / 15 151 (x 1.03) | 1 | 132 / 103 | 648 / 396 | 453 / 456 |
  | read of 255 on the engines | 873 478 / 873 438 (x 1.00) | 3 | 343 / 265 | 2264 / 922 | 237 / 245 |

  A tenure of half a FIFO or less is ONE interrupt, its STOP; a long
  read is one every eight bytes. The address and the speed stay in the
  block between tenures, so a one-byte write's fixed cost is the same at
  400 kHz and 1 MHz (404 to 406 on the M33, 413 to 415 on Hazard3); at
  100 kHz it reads higher, 572 or 573 / 469 to 575, and what the excess is
  is not split. The disable, IC_TAR and enable that a new address still
  pays costs about 1650 / 1680 to 1780 cycles more at 100 kHz, 210 to
  230 / 240 at 400 kHz and 92 to 94 / 104 to 107 at 1 MHz - the disable
  waits on the block's own shutdown, which follows the bus timing. The
  longest host entry is 282 / 285 cycles, a 255-byte read's. With the
  vector in the flash too (`bench_rp2350`, the default image) the
  one-byte write's fixed cost reads 400 to 404 / 522 to 524 and its
  handler 96 / 180, a 255-byte write 31 interrupts on both halves at
  400 kHz. **WARM, THE PLACEMENT MATTERS ON HAZARD3 ALONE**: the one-STOP
  handler reads the same 96 cycles on the Cortex-M33 from the flash and
  from SRAM, and 180 against 71 on Hazard3, whose tenure misses the XIP
  cache twice even warm from the flash image (the letter's count over the
  tenure; which lines, not counted). COLD, it matters on both:
  the same handler after every line of the XIP cache is invalidated reads
  2848 / 1649 cycles from the flash and 96 / 71 placed. Where it runs is
  the program's: the vector it binds, and `I2cHost`'s placement tag for
  `start()` and the bodies ([platform.md](platform.md), "A handler in
  SRAM", has the one-byte write's longest entry WARM / COLD: 96 / 2848 on
  the M33 and 180 / 1649 on Hazard3 with both in the flash, 119 / 119
  and 80 / 80 with both placed). `bench_rp2350` runs the handler from
  the flash, `bench_rp2350_ram` from SRAM.
- THE VENDOR'S TENURE ON THE SAME WIRE: the pico-sdk 2.3.1's polled
  `i2c_write_blocking` / `i2c_read_blocking` (its `i2c.c` compiled from
  the pinned archive for each half, `clock_get_hz` answered with
  clk_sys, brio's crt, clock, console and ruler around it) on I2C0,
  GP36/GP37, against the same peer between the same serve commands; the
  SDK's own `i2c_init`, then brio's counts written over its own, so the
  SCL period measures the same (378.9 cycles at 400 kHz on both). At
  400 kHz, fixed cost, the SDK against the default image (the table's
  V image in brackets), Cortex-M33 / Hazard3: a one-byte write 362 / 354
  against 400 / 522 (404 / 413), a 255-byte write 366 / 357 against
  400 / 525, a 16-byte read 767 / 853 against 485 / 509, a 255-byte
  read 13 675 / 15 203 (x 1.02) against 301 / 385 (x 1.00), a register
  read of 1 + 1 -16 / -45 against 449 / 566; a refused address 348 / 317
  against 559 / 591. The one-byte write's gap is within a fifth on the
  M33 from either image (11 and 12 %) and on Hazard3 against the V image
  (17 %): what the SDK does not do - no interrupt, no wake from the idle
  loop; Hazard3's default image adds some 110 cycles more, its handler's
  difference between the flash and SRAM (the bullet above). For it the
  SDK holds the core the whole wire, busy 7714 / 7706 cycles against
  527 / 436 from the default image. Its short reads' fixed cost is
  below zero because `i2c_read_blocking` returns at the last byte,
  before the NACK and the STOP are on the wire; its long read keeps one
  read command in flight, waiting for each byte before the next command,
  where the pump pours them by the FIFO.
- THE ENGINES (letter g, every request naming its engine, three runs of z
  on each half, the same figures in each):
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
- A PURE WRITE ON THE TRANSMIT ENGINE (letters y, n, o and v, each with its
  control, ten runs of each on each half outside `z`, all green, and in the
  three runs of `z`). The bytes: 3, 4, 17, 18, 64 and 255 at 400 kHz and 255 at
  100 kHz and 1 MHz, each read back by the partner against the pattern at its
  position, none off in 874 bytes, the sum and the count exact; one DMA line
  entry each, one host entry for a block that left room for the last entry (3,
  4 and 17) and two where it did not, TX_OVER never raised; a read of 16 between
  them on the same channel exact, its halfword commands through the same address.
  A write of 255 takes 5841 to 5854 us at 400 kHz, 23 139 to 23 152 at 100 kHz
  and 2384 to 2397 at 1 MHz, within 1.4 % of the wire's time. THE CONTROL, the
  channel bound at the register's own address, 32 bytes of the same pattern:
  the partner receives 17, 16 of them off the pattern, and serves 15, the read
  commands the replicated lanes made.
  A NACK in mid-write: the twentieth of 255 refused at 100 kHz is `i2c_nack_data`,
  nineteen bytes stored exact, one host entry and no DMA line entry, the channel
  stopped, and the next engined read exact. The abort flushes the transmit FIFO and
  holds it flushed until it is read away, which leaves the request standing, so what
  remains of the block pours into nothing while the host's vector comes: a block with
  217 entries to go is still running then, and the vector stops it before the abort is
  read away, which is what keeps the rest from starting a tenure of its own. (One with
  44 to go, a write of 64, had ended before the vector on the Cortex-M33, its end on the
  DMA line a harmless entry, and on Hazard3 had not.) The first byte refused is
  `i2c_nack_data` with nothing stored, a deaf address on a write of 32 `i2c_nack_addr`.
  THE CONTROL, nothing refused: i2c_ok, the 255 stored exact.
  A lost arbitration: a write of 255 aimed at the higher address, inside a window held
  by hand with the first entry and the block already in the FIFO, is `i2c_arb_lost`,
  the channel stopped, the peer's host keeping the bus. THE CONTROL, aimed at the lower
  address: this host keeps it, `i2c_nack_addr`, the peer's reporting the loss.
  A bus error: a write of 16 from the last byte of SRAM (`start()` reads the first
  entry, the channel's first beat reads past SRAM_END, 2.2.3) is `i2c_dma_fault`, on the
  DMA line alone with no host entry, the channel stopped, the partner storing the one
  byte the block ever held. THE CONTROL: the next write from RAM i2c_ok and exact.
- THE ENGINES' FAULT PATH (letter g): a 64-byte engined read whose
  receive run points into a hole in the map (0x40100000) is a bus error
  on the receive channel; `dma_isr()` aborts the tenure and answers
  `i2c_dma_fault` with no host interrupt, the peer having served 3 or 4
  bytes by the abort on the Cortex-M33 and 3 on the Hazard3, in
  twenty-five runs each, and the next engined read is clean.
- **THE ENGINES' FIRST READ IS THE XIP CACHE'S FILL.** A 64-byte engined
  read at 400 kHz takes 1483 to 1489 us (392 to 394 kHz) with two host
  interrupts; letter g's first takes 1540 to 1544 / 1527 to 1532 us
  (Cortex-M33 / Hazard3) inside the key `z`, whose other letters run
  before it, and 1513 to 1535 / 1505 to 1520 with the letter run alone,
  and the same read with every line of the cache invalidated first,
  printed third, 1531 to 1539 / 1520 to 1530 - two host interrupts in
  every case, over twenty-five runs on each half; the RP2040's reading
  in a smaller size ([../rp2040/i2c.md](../rp2040/i2c.md)). The letter
  judges the second.

### The stack under the preemptive kernel (`test_rp2350_rubato_i2c`)

Measured by `test_rp2350_rubato_i2c`, one source and two images:
`util/i2c_bus.hpp`'s arbiter (pending depth 4, the per-bus timeout at
20 ms) over the engined `I2cHost<0>` on GP36/GP37 (DMA channels 4 and 5
on DMA_IRQ_0, clk_sys at 150 MHz) inside Rubato on core 0, against the
WeAct RP2040's `twi_peer`, while TIMER1's alarm 0 posts to the pack's
highest object every 97, 193 or 389 us and its dispatch spins 300 or
3000 cycles - at 97 us x 3000, 21 % of the core. The host is brought up
at every pass's opening, once the RP2040's rail reads up on the two
pads, and given back at every letter's end. Every letter runs once with
the alarm silent and six times under it. **209 verdicts in the key `z`,
green in every run on Hazard3 and in every run but the first after a
flash on the Cortex-M33** - the key thirteen times on each half, its
vocabulary letter `q` ten more on Hazard3 and eighteen more on the
Cortex-M33; the letters `x` and `s`, outside it, green in thirteen runs
each on each half; three runs of the key on each half with the suite's
pure writes of three bytes or more on the transmit engine, green. The one verdict the Cortex-M33 fails there is the
peer's and not the host's (below, "The probe after a refusal"). The pack,
the command channel through the arbiter and the letters are the I2C
suites' shared body (`avrxt/src/apps/rubato_i2c.hpp`); what the two
Raspberry Pi chips share over this block is
`rp2040/src/apps/rp_rubato_i2c.hpp`, whose header derives every bound,
and the RP2040's run of it is [../rp2040/i2c.md](../rp2040/i2c.md)'s.

**A preemption inside `start()`'s fill: no stretch at 400 kHz.**
`start()`'s first entry starts the tenure, so a preemption after its
k-th entry leaves the block (1 + k) byte times before it needs the next,
and only a longer one holds SCL ([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md),
the host's engine). Letter `s`, Cortex-M33 / Hazard3, on `start()` as
the suite wraps it (one copy of the Request and two compares to name its
engine, `rp2040/src/apps/rp_rubato_i2c.hpp`'s `EngineByLength`) with the
host in the flash: `start()` takes 424 to 460 / 364 to 386 cycles for a
one-byte write and 504 to 516 / 456 for a write of sixteen, the
fifteen entries after the first 44 to 92 / 70 to 92 cycles, where 400
writes under independent ticks would land 1 or 2 times (three runs; `start()`'s
averages hold the XIP misses of the image's layout, the host's own cost of the
choice being 5 or 6 cycles in `bench_rp2350` letter i's warm lines). Under
97 us x 3000, the longest preemption of `start()` 3243 to 3471 / 3289
to 3510 cycles against two byte times' 6750 at 400 kHz: 3 to 13 / 15 to
25 landings in `start()` - its whole span counted - in 400 writes, 1 to
7 / 3 to 12 with the host active on the wire, and **no stretch in
sixteen runs on either half**. At 1 MHz (two byte times 2700 cycles)
the Cortex-M33 never landed in `start()` in sixteen runs, nor under
the 160 us spin at 389 us, and Hazard3 landed three times in sixteen runs at
97 us x 3000 (twice with the host active, no stretch) and never under the
long spin - where independent ticks would land some twelve times in a
run of 400 writes. The tenure cycle there is not independent of the
alarm's period, and which part of it sets the phase is not established;
the letter's long-spin case is judged here only by its exactness, and
its stretches are the RP2040's measurement. Letter `x`, 400 writes of
sixteen at 400 kHz (`start()` 387 or 388 / 459 or 460 cycles silent): 7
to 23 / 13 to 24 landings against the 10 / 11 or 12 independent ticks
would give, 3 to 14 / 5 to 11 with the host active, no stretch. The
key `z` stretched no tenure in thirteen runs on either half.

**THE HELD WIRE PARKS THE START.** Letter `h`, both halves: the Client's
write into the peer's 60 ms hold, after the disable, IC_TAR and enable
cycle its address takes, answered `i2c_timeout` 19.97 to 20.07 ms
after its `start()` in every pass of thirteen runs, and the same met at
400 kHz after commands at 100 kHz, thirteen of thirteen; the flood of
probes answered
`i2c_rejected` while the queue is full, one `i2c_timeout` a pass, and
after the release `i2c_ok` or, once a pass, `i2c_nack_addr` from a peer
not yet back. The parked one-entry tenure's false end on another node's
STOP that the RP2040 measured ([../rp2040/i2c.md](../rp2040/i2c.md)) was
not seen on this chip: none in twelve pressed passes on the Cortex-M33
of a build whose STOP test lacked the state machine's idle, and none in
156 on the two halves with it (thirteen runs of `z` each).

**The probe after a refusal races the peer, and the SRAM placement of the peer's path cures it.**
Letter `q`'s control pass writes six bytes to a peer refusing the third, then probes it at
once: the peer refuses by disabling its client once the second byte is taken, and enables
it again once the block reports the disable done, after the host's STOP. With that end of
the refusal in the RP2040 peer's flash, on the Cortex-M33 the probe met the client still
disabled in the first run after both boards were flashed, answered `i2c_nack_addr`: nine
first runs of nine, and in none of nine later runs. That build is no longer in the tree, so
this is a measured fact, not a letter. With the same path in SRAM (`twi_peer`'s
`refusal_end()`), as the peer is today, thirteen first runs of thirteen are clean, and six
later runs of six. A cold cache in the peer is inferred from that cure and that only the
first run meets it; no log reads the cache. The host's side of the
race is its abort: a tenure on the interrupt touches no channel at its abort, so the probe
follows sooner. Hazard3 and the RP2040 as the host never met it. The verdict judges the peer's
recovery and was not loosened.

**Every shape, and three requesters, under every pressure as in the
control.** Letter `o` at 400 kHz, one serve counted to its end - the
suite naming `dma` for a request with a half of two bytes or more and
`interrupt` otherwise, so the shapes take every path the host has
(`rp2040/src/apps/rp_rubato_i2c.hpp`'s table names each; a write of
exactly sixteen stays on the interrupt, letters `s` and `x` measuring that
fill) - every
shape `i2c_ok` and byte-exact, the peer's count, sum and seventeen
address matches exact; post to reply over three runs, the control's
and the worst under pressure, Cortex-M33 / Hazard3: the probe 73 to 86
and 65 to 89 / 75 to 80 and 69 to 91 us, the 32-byte write, on the transmit
engine, 763 to 766 and 764 to 802 / 761 to 763 and 761 to 805, the 32-byte read
764 or 765 and 763 to 805 / 762 and 761 to 770. Letter `w`: every reply
`i2c_ok`, to its requester, in its order, the peer's tallies exact, 5.4
to 5.9 / 5.3 to 5.9 ms a pass. Letter `p`: the peer's commanded stretch
of 1000 us a byte on a read of 8 in 8788 to 8810 / 8789 to 8813 us
against the 8762
its eight holds take at least.
- **A RELEASED REQUESTER, THE SAME OWNER AGAIN** (letter r, ten runs of four rounds on each half,
  the second chip serving): the host released the worst way, both DMA enables of IC_DMA_CR
  set, its channels stopped, the transmit FIFO empty and so its request standing, the block
  held in reset. Then `init()`: a channel bound on the transmit request, pouring into a cell
  of RAM and not into the block, moves none of four in 20 us, and after `init()` again an
  engined read of 16 bytes at 400 kHz is exact. THE CONTROL, the reset removed: the block
  disabled and enabled with the enables left set, the same channel moves one item at once,
  the request standing, and no more, a cell of RAM giving the block nothing to clear it with;
  nothing of the control reaches the wire. That is the request a reset at `init()` withdraws
  ([dma.md](dma.md)).

## Not covered yet

Driver gaps, each with its reason. The ones that belong to the block
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
- Two clients on one address with a third party reading (twi_link's
  `coll`): this desk has no third node on the wire, and a DW block is a
  host or a client, never both.
- This chip as the bus's target, served from an active object, under
  the preemptive kernel: a target whose bytes come from an object (its
  vector posting "the host reads" and leaving SCL held, a dispatch giving
  the byte) is the one place a held-off reader would stretch the clock,
  and no util service serves a target that way (`I2cClient` is served
  from its vector). The partner's `twi_peer` could read it (its
  `host_read`). Born with the first util target object.

Implemented but not bench-verified, each with what would measure it:

- The client's general call (`ack_general_call`): no brio host sends a
  general call, the partner's included, so no suite reaches this
  block's GEN_CALL handling ([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md)
  says what the RP2040 showed of it); a controller that sends one would
  measure it.

- `rebase()`: this stratum has no dynamic clock; measured when the
  clock chapter grows one, a tenure exact after a switch.
- The QFN-60's pin table: the stratum compiles for that package and
  refuses the pads it has not got, but no QFN-60 part is on the bench.
