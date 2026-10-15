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
one source: the block wireless, its pads, and the wire to the desk
partner - I2C0 on GP36 (SDA) / GP37 (SCL), 1.5 kOhm to the WeAct
RP2040's 3V3 on each ([../boards/weact-rp2350b.md](../boards/weact-rp2350b.md)),
whose peer of the twi_link protocol is commanded over the bus under
test; that board's own suite, `test_rp2040_i2c`, judges this chip's block
as that peer.

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
as a pulled-up one. The errata sheet's own workaround is the instrument:
the pad's pull-down on with the input buffer OFF, the buffer enabled for
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
- `DwApbI2c<n>`, `I2cHost<n, pins, TxEngine, RxEngine, Placement>` and
  `I2cClient<n, pins>` - the public names, this chip's aliases of the
  three IP templates, with `NoDmaEngine` in both slots and `InFlash` by
  default (`Placement`: util/placement.hpp's tag, [platform.md](platform.md),
  "A handler in SRAM").

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

## Bench findings

All of them on an RP2350 in the QFN-80 package, **stepping A2**, clk_sys
at 150 MHz, and all of them on BOTH architectures: `test_rp2350_i2c`
reports **63 pass, 0 fail** on the Cortex-M33 pair and on the Hazard3
pair, from one source, against the WeAct RP2040's peer over the pair's
I2C wires. Nothing in this chapter reads differently between the halves,
which is what a block outside the processor should look like. The same
block as the PEER, the RP2040 the host, is judged by `test_rp2040_i2c`
(docs/rp2040/i2c.md, whose wire findings - the general call both ways,
GEN_CALL standing through the acknowledge, a STOP_DET for every STOP, the
refusal by a disable - this block shares and are not restated here). The
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
  disable of an idle block is reported by IC_ENABLE_STATUS in **1 us**.
  The interrupt lines are **36 and 37**, not the RP2040's 23 and 24.
- **THE PULL-UPS READ THE WAY E9 FORCES.** A pad of this stepping whose
  input buffer is on holds history against its own pull-down, so an idle
  level says nothing: GP36 and GP37 are read with the pull-down on and the
  buffer pulsed for the read alone (`rp2350/src/apps/pair_power.hpp`), and
  both read HIGH - the 1.5 kOhm to the partner's 3V3, which is also the
  proof that the partner is powered before a pad of the link is driven.
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
  shape, the general call, the vocabulary (deaf, a data NACK at the
  first and at the third byte), the three speeds byte-exact both ways -
  99, 393 to 394 and 965 to 968 kHz measured on the timer for 100, 400
  and 1000 asked -, the engines with eight 255-byte reads at 1 MHz and
  TX_OVER never raised, I2cBus with its timed bus (`i2c_timeout` at the
  arbiter's 20 ms, the peer holding SCL), the stuck bus (four pulses for
  a client releasing at its fourth fall; 0xFF in 122 to 124 us for a
  line nine pulses cannot free), arbitration both ways, and this board
  as the peer host's client, written to, read from and general-called -
  all of it through GP36/GP37 and the unstick's verbs on the HIGH half
  of SIO. The client's stretch prices at 754 to 763 us for eight holds of
  100 us, a write under it within 4 us of the plain one.
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
  ([../dw_apb_i2c/README.md](../dw_apb_i2c/README.md)). Ten runs of z on
  each half.
- **THE HOST'S COST**, `bench_rp2350` letter i against the peer's
  client, the same image built for both halves, the host's vector placed
  in SRAM (the V image of [platform.md](platform.md), "A handler in SRAM",
  `BENCH_VECTORS_IN_SRAM` in bench_rp2350.cpp), nothing of the far end on
  this core; the wire's time at the SCL
  period measured (379.0 cycles at 400 kHz against the counts' 375: the
  pull-ups' rise, 26 ns a period at every speed). The best of five each,
  clk_sys cycles at 150 MHz, Cortex-M33 / Hazard3:

  | at 400 kHz | wall | interrupts | handler cycles | busy | fixed cost |
  |---|---|---|---|---|---|
  | write of 1 | 7744 / 7755 (x 1.05) | 1 | 89 / 72 | 767 / 385 | 392 / 403 |
  | write of 255 | 874 138 / 874 149 (x 1.00) | 31 | 5444 / 4774 | 15 938 / 6756 | 392 / 403 |
  | read of 16 | 59 012 / 59 017 (x 1.00) | 2 | 362 / 317 | 1461 / 817 | 495 / 500 |
  | read of 255 | 874 231 / 874 236 (x 1.00) | 32 | 9008 / 8417 | 23 141 / 10 434 | 485 / 490 |
  | register read, 1 + 1 | 15 146 / 15 152 (x 1.02) | 1 | 124 / 102 | 625 / 405 | 441 / 447 |
  | read of 255 on the engines | 874 122 / 874 130 (x 1.00) | 3 | 345 / 697 | 2348 / 1812 | 376 / 384 |

  A tenure of half a FIFO or less is ONE interrupt, its STOP; a long
  read is one every eight bytes. The address and the speed stay in the
  block between tenures, so a one-byte write's fixed cost is the same at
  the three speeds (390, 392, 392 on the M33); the disable, IC_TAR and
  enable that a new address still pays costs 1839 cycles more at
  100 kHz, 240 at 400 kHz and 105 at 1 MHz on the M33 (1861, 251, 116 on
  the Hazard3) - the disable waits on the block's own shutdown, which
  follows the bus timing. The longest host entry is 298 / 279 cycles, a
  255-byte read's. With the vector in the flash too (`bench_rp2350`, the
  default image) the one-byte write's fixed cost reads 491 / 671 and its
  handler 87 / 236, a 255-byte write 31 / 35 interrupts. **A HANDLER IN FLASH IS A LOTTERY ON THE
  CORTEX-M33**: the one-STOP handler measured 188 to 365 cycles across
  three flash builds of the bench, with up to six XIP cache misses of
  about seventy cycles each in a tenure, against 84 from SRAM; the
  Hazard3 builds of the same source ranged 57 to 287. Where it runs is the
  program's: the vector it binds, and `I2cHost`'s placement tag for
  `start()` and the bodies ([platform.md](platform.md), "A handler in
  SRAM", has the one-byte write's longest entry WARM / COLD: 88 / 2960 on
  the M33 and 226 / 1669 on Hazard3 with both in the flash, 110 / 110
  and 71 / 71 with both placed). `bench_rp2350` runs the handler from
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
  against 491 / 549 (392 / 403), a 255-byte write 366 / 357 against
  498 / 678, a 16-byte read 767 / 853 against 492 / 621, a 255-byte
  read 13 675 / 15 203 (x 1.02) against 587 / 882 (x 1.00), a register
  read of 1 + 1 -16 / -45 against 539 / 830; a refused address 348 / 317
  against 560 / 583. Against the V image the write's gap is within a
  fifth (8 and 12 %): what the SDK does not do - no interrupt, no wake
  from the idle loop; the default image adds 100 to 150 cycles more, the
  difference the bullet above states between its two conditions (the
  vector in the flash or in SRAM). For it
  the SDK holds the core the whole wire, busy 7714 / 7706 cycles
  against 599 / 302. Its short reads' fixed cost is
  below zero because `i2c_read_blocking` returns at the last byte,
  before the NACK and the STOP are on the wire; its long read keeps one
  read command in flight, waiting for each byte before the next command,
  where the pump pours them by the FIFO.
- THE ENGINES' FAULT PATH (letter g): a 64-byte engined read whose
  receive run points into a hole in the map (0x40100000) is a bus error
  on the receive channel; `dma_isr()` aborts the tenure and answers
  `i2c_dma_fault` with no host interrupt, the peer having served 3 or 4
  bytes by the abort (21 and 9 of 30 runs over the three images of the
  pair), and the next engined read is clean.
- **THE ENGINES' FIRST READ IS THE XIP CACHE'S FILL.** A 64-byte engined
  read at 400 kHz takes 1485 to 1492 us (392 to 393 kHz) with two host
  interrupts; letter g's first after a flash takes 1543 / 1530 us
  (Cortex-M33 / Hazard3), and the same read with every line of the cache
  invalidated first, printed third, 1529 / 1518 - two passes on each
  half; the RP2040's reading in a smaller size
  ([../rp2040/i2c.md](../rp2040/i2c.md)). The letter judges the second.

## Not covered yet

Driver gaps, each with its reason. The ones that belong to the block
rather than to this chip - the high-speed mode, SMBus, the stuck-bus
timeouts, 10-bit addressing on the host side, a write phase on the DMA
engines, the client on the engines - are in
[../dw_apb_i2c/README.md](../dw_apb_i2c/README.md) and are not repeated
here.

- The START byte as a host verb: `target(addr, general_call)` is in the
  resource; no tenure shape asks for it.
- Two clients on one address with a third party reading (twi_link's
  `coll`): this desk has no third node on the wire, and a DW block is a
  host or a client, never both.

Implemented but not bench-verified, each with what would measure it:

- `rebase()`: this stratum has no dynamic clock yet; measured when the
  clock chapter grows one, a tenure exact after a switch.
- The QFN-60's pin table: the stratum compiles for that package and
  refuses the pads it has not got, but no QFN-60 part is on the bench.
