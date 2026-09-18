# I2C (RP2350)

Documents of record: the RP2350 datasheet (build d126e9e), 12.2 (the
Synopsys DW_apb_i2c v2.03a: 12.2.1 the feature list with the FIFO depths
and the pad setup, 12.2.5 the behaviour - START and STOP generation, the
combined formats -, 12.2.7 the transmit FIFO and the RESTART / STOP bits
of a command, 12.2.10 the operation modes, 12.2.11 the spike filter,
12.2.12 fast-mode-plus, 12.2.13 the bus clear, 12.2.14 the SCL counts
with their floors and the cycles the block adds, 12.2.15 the DMA
interface, 12.2.16 the interrupt registers, 12.2.17 the registers), 9.4
(the GPIO function table), 12.6.4.1 (the system DREQ table), 7.5 (the
reset controller), 3.2 (the interrupt lines), 2.1.3 (the atomic register
aliases), 2.1.5 (narrow writes replicated across a register), 3.1.3 and
9.8 (the single-cycle IO the unstick drives an open-drain line through),
9.7 (the pad isolation latches);
[../design/i2c-bus.md](../design/i2c-bus.md) for the Request and the
vocabulary. The driver is Synopsys's and not this chip's, so it lives in
the IP stratum: `brio/dw_apb_i2c/i2c.hpp` holds the resource, the host
engine and the client
([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md)), and
`brio/rp2350/i2c.hpp` holds what this chip owes it - the pin table of 9.4
with its pad configuration, the `I2cPad` type that adds the two
open-drain verbs to a `Pin`, the `Rp2350DwApbI2c` chip traits over
`pin.hpp`, `resets.hpp`, `core.hpp`, `delay.hpp` and `dma_engine.hpp`,
this chip's device description held against the bits the IP file spells
itself, and the PUBLIC NAMES `DwApbI2c<n>` (the resource), `I2cHost` (the
engine `I2cBus` drives) and `I2cClient` (the other end), this chip's
aliases of the three templates there. The reference suite:
`test_rp2350_i2c`, which runs on BOTH of this chip's architectures from
one source, on two wires between the chip's two instances.

This page says what is the RP2350's. What the block itself does - the
command FIFO, the counts and their floors, the single TX_ABRT, the
client's clock stretch, the engine slots - is the IP stratum's page and
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
five, and none of them is inside the block:

**THE PINS ARE ONE COLUMN, OVER A LONGER BANK.** Every fourth pad from
GP0 carries an I2C signal - SDA on the even pads, SCL on the odd, the
instance alternating in pairs - so I2C0 has SDA on GP0, 4, 8, ... 44 and
SCL on 1, 5, 9, ... 45, and I2C1 has SDA on 2, 6, 10, ... 46 and SCL on
3, 7, 11, ... 47. That is the RP2040's rule over forty-eight pads instead
of thirty: twelve SDA pads and twelve SCL pads per instance against eight
and eight. WHAT THE UART GAINED HERE, THIS BLOCK DID NOT - the UART's
flow-control pads carry data too under a second function code
([uart.md](uart.md)), while 9.4 gives every one of the forty-eight I2C
pads exactly one I2C entry, all of them function 3. So a pin set stays a
pair of pad numbers and carries no column beside them.

**THE PADS COME UP ISOLATED** (9.7, 9.11, and [README.md](README.md)):
PADS_BANK0's reset value is 0x116, the ISO latch set and the input buffer
disabled, and while the latch stands the pad is cut off from the digital
logic in both directions. A pad configured as on the RP2040 would do
nothing at all. Every configuring verb of `pin.hpp` writes the whole pad
register with ISO clear, so the driver's hand-over - the same call the
RP2040's makes - is what drops the latch. What it writes is 12.2.1.3's
own list: pull-up enabled, slew rate limited, Schmitt trigger enabled,
with the chapter's note that the pad's pull-up is not meant to do the
pulling and the board's should.

**THE BANK IS TWO WORDS**, and that is the one line of the family header
that is not the RP2040's. An open-drain line is driven by its output
ENABLE over an output register already holding a zero, and SIO holds
GPIO0..31 in GPIO_OE and GPIO32..47 in GPIO_HI_OE (3.1.3). `I2cPad` picks
the half its pad falls in with an `if constexpr` on the pin number, so a
line on GP32..GP47 is driven through the right register and the choice
costs nothing.

**THE DREQ NUMBERS AND THE RESET BITS ARE NOT THE RP2040'S.** A third PIO
and twelve PWM slices against eight move every DREQ row above the first
PIO's: the I2Cs sit at 44..47 where they sat at 32..35. The reset
controller governs twenty-nine blocks against twenty-seven, and I2C0 and
I2C1 sit at bits 4 and 5 where they sat at 3 and 4. Both come from this
chip's own headers, through `dma_engine.hpp` and `resets.hpp`.

**THE INTERRUPT NUMBERING IS SHARED BETWEEN THE ARCHITECTURES** (3.8.4.2),
so I2C0_IRQ is line 36 and I2C1_IRQ line 37 whichever processor pair is
running - not the RP2040's 23 and 24 - and an app binds `isr_i2c0` /
`isr_i2c1` once for both. The controller behind those names is not the
same object, an NVIC on the Cortex-M33 half and Hazard3's own on the
other, which is exactly why the IP stratum's concept takes the controller
as a TYPE.

And one thing that is not a difference in the block but changes what a
program must do: **THE UNSTICK IS PACED BY THE PLATFORM TIMER.** The bus
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

**ERRATUM RP2350-E9 CHANGES HOW A WIRE IS PROVEN, NOT HOW IT WORKS.** On
stepping A2 a pad whose input buffer is enabled and which nothing drives
leaks enough current to sit HIGH against its own pull-down, so an idle
level is history and not a measurement. An I2C line idles high anyway,
pulled there by the board, and the driver hands both pads over with the
pull-up the chapter asks for, so nothing the block does is coloured by
the leak: a line that reads LOW is genuinely being driven low, which is
all `unstick()` asks. What the erratum does change is a program's ability
to ask "is a pull-up fitted at all", because a bare pad answers the same
as a pulled-up one. The other half of the erratum is the instrument: a
pad driven low and then released STAYS low, so the honest test is to
drive each line low, release it onto the pad's own pull-down, and read -
only an external pull-up brings it back up.

## Types and verbs

What the IP stratum names (the speed vocabulary, the timing arithmetic,
the register bit layouts, the resource's verbs, the host engine's and the
client's) is in [../dw_apb_i2c/README.md](../dw_apb_i2c/README.md). This
stratum adds:

- `I2cPins` - the two pad numbers, SCL and SDA. One column, so no
  function travels with them.
- `i2c_sda_pin(n, pin)`, `i2c_scl_pin(n, pin)` - the table of 9.4 as
  constexpr facts over the whole bank, and `i2c_pins_valid(n, pins)`,
  which an engine's `static_assert` calls. The PACKAGE is not asked
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
- `DwApbI2c<n>`, `I2cHost<n, pins, TxEngine, RxEngine>` and
  `I2cClient<n, pins>` - the public names, this chip's aliases of the
  three IP templates, with `NoDmaEngine` in both slots by default.

The family header also holds THE VOCABULARY CHECK: every bit the IP file
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
DmaRxEngine<5>>` and `isr_dma_0` calling `Bus::dma_isr()` the same way.
This chip has sixteen channels and four interrupt lines, and the line is
the engine's third parameter, so a host whose engines ring on line 2 is
`DmaTxEngine<6, uint16_t, 2>` and `DmaRxEngine<7, uint8_t, 2>` with
`isr_dma_2` ([dma.md](dma.md)).

A program that means to call `unstick()` starts the ruler its half-bits
are paced by, which on the Cortex-M33 half nothing else does:

```cpp
brio::Mtime::start(clock);      // idempotent; the Hazard3 ticker does it too
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

## Not covered yet

Driver gaps, each with its reason. The ones that belong to the block
rather than to this chip - the high-speed mode, SMBus, the stuck-bus
timeouts, 10-bit addressing on the host side, a write phase on the DMA
engines, the client on the engines - are in
[../dw_apb_i2c/README.md](../dw_apb_i2c/README.md) and are not repeated
here.

- Multi-master arbitration on the wire: ARB_LOST is decoded to
  `i2c_arb_lost`, but the self-link's other instance is the client, and a
  second host on these two pads needs a peer board that this desk's wires
  do not reach.
- An I2C on the pads above GP31: the table covers them and `I2cPad`
  drives them through the right half of the bank, but no wire of this
  desk reaches GP32..GP47 and the QFN-60 has not got them at all.
- The general call and the START byte as host verbs: `target(addr,
  general_call)` is in the resource; no tenure shape asks for either.

Implemented but not bench-verified, each with the letter of
`test_rp2350_i2c` that will measure it. Nothing in this chapter is
expected to differ between the two architectures - the block is the same
block whichever processor pair is running - so every letter is run twice
and the two transcripts are compared; only the interrupt counts and the
service latencies are printed rather than judged tightly.

- The pin table over the whole bank, the timing arithmetic at this
  clk_sys, table 1053's floors reproduced, the reset state (IC_CON 0x65,
  the spike filter, the hold, the component parameter register at zero),
  the transmit FIFO's depth measured under TX_CMD_BLOCK, the refusals
  while enabled, the disable of an idle block timed, and the two
  interrupt lines being 36 and 37 (letter a).
- THE PULL-UPS PROVEN THE WAY E9 FORCES - each line driven low, released
  onto the pad's own pull-down and read back high - and the two readings
  printed side by side, the bare pad's and the pulled-up one's; then the
  scan of 0x08..0x77 finding the one client, nobody home as
  `i2c_nack_addr` and timed, and the probe served as one read request
  (letter b).
- The four tenure shapes - a write, a read, a write-then-read with the
  repeated START counted from the far end, the probe - and a 200-byte
  write in one tenure through the pump (letter c).
- The vocabulary on the wire: `i2c_nack_addr` from a deaf client on all
  three shapes, `i2c_nack_data` from a client refusing data,
  the client's clock stretch priced, and the bytes queued beyond what a
  host takes flushed at its NACK (letter d).
- The three speeds byte-exact both ways and the SCL rate MEASURED on the
  ruler against what the counts should give on an ideal wire (letter e).
- THE TWO DMA ENGINE SLOTS over this chip's controller ([dma.md](dma.md)),
  kept apart from every bus verdict so that a fault in the newer chapter
  cannot mask one here: a 64-byte read, a write-then-read, the short
  reads that stay on the pump, and eight 255-byte reads at 1 MHz with
  TX_OVER never raised (letter f).
- `util/i2c_bus.hpp`'s arbiter over the engine with nothing changed: four
  tenures answered, a NACK in its place, the rejection when the queue is
  full, both sleep votes, and THE TIMED BUS - a client holding SCL
  answered `i2c_timeout` at the arbiter's 20 ms, with the `recover()`ed
  engine carrying the next tenures (letter g).
- `unstick()` on a healthy wire, on one with SDA held low by a GPIO, and
  on a healthy one again - AND ITS HALF-BIT, the nine clocks and the STOP
  timed on the system timer, which is what states whether the platform
  timer was running under them (letter h).
- The roles inverted on the same two wires, I2C1 hosting and I2C0
  listening (letter i).
- THE PADS AND THE ISOLATION LATCH, which is where this chip differs
  most: the whole pad register read back after the hand-over against
  12.2.1.3, a tenure carried on pads so configured, the pads taken by
  hand for an unstick and given back unchanged, and `release()` leaving
  them at 9.11's reset value - isolated, the input buffer off, a
  pull-down (letter j).
- `rebase()`: this stratum has no dynamic clock yet; measured when the
  clock chapter grows one, a tenure exact after a switch.
- The client's general call (IC_ACK_GENERAL_CALL, the `general_call`
  event) and the client's `overrun` event: a host tenure to address 0x00
  through the resource's `target(0, true)`, and a client configured
  without the hold when its receive FIFO is full.
- `dma_isr()`'s fault path (`i2c_dma_fault`): a bus error on a channel,
  staged with an address the fabric refuses.
- The client tenure cut by a disable (`client_disabled_while_busy`,
  `client_rx_data_lost`): a disable timed inside a host's write.
- The QFN-60's pin table: the stratum compiles for that package and
  refuses the pads it has not got, but no QFN-60 part is on the bench.
