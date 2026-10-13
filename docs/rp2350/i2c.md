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
one source: the block wireless, and its pads on the desk partner's I2C
wires - I2C0 on GP36 (SDA) / GP37 (SCL), 1.5 kOhm to the WeAct RP2040's
3V3 on each ([../boards/weact-rp2350b.md](../boards/weact-rp2350b.md)).

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

## Bench findings

All of them on an RP2350 in the QFN-80 package, **stepping A2**, clk_sys
at 150 MHz, and all of them on BOTH architectures: `test_rp2350_i2c`
reports **14 pass, 0 fail** on the Cortex-M33 pair and on the Hazard3
pair, from one source, the far board running a dark image. Nothing in
this chapter reads differently between the halves, which is what a block
outside the processor should look like. The cost figures are
`bench_rp2350` letter i's, on a SELF-LINK the pair desk does not carry
(GP12 to GP14 and GP13 to GP15, 4.7 kOhm to 3V3, I2C0 the host and I2C1
the client at 0x42): the letter runs by name when it is fitted.

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

- **THE HOST'S COST**, `bench_rp2350` letter i on that self-link and
  the same image built for both halves, the I2C vectors in SRAM, the
  client served from its own vector and kept out of the counts; the
  wire's time at the SCL period measured (379.0 cycles at 400 kHz: 4.0
  cycles, 26 ns, over the counts, the pull-ups' rise, which keeps
  fast-mode-plus inside its 120 ns). The best of five each, clk_sys
  cycles at 150 MHz, Cortex-M33 / Hazard3:

  | at 400 kHz | wall | interrupts | handler cycles | busy | fixed cost |
  |---|---|---|---|---|---|
  | write of 1 | 7827 / 7837 (x 1.06) | 1 | 84 / 65 | 583 / 302 | 474 / 484 |
  | write of 255 | 874 305 / 874 327 (x 1.00) | 31 | 5409 / 4795 | 23 866 / 8009 | 455 / 461 |
  | read of 16 | 59 082 / 59 079 (x 1.00) | 2 | 361 / 310 | 2052 / 817 | 558 / 554 |
  | read of 255 | 874 526 / 874 500 (x 1.00) | 32 | 9007 / 8441 | 30 017 / 11 017 | 676 / 634 |
  | register read, 1 + 1 | 15 216 / 15 219 (x 1.03) | 1 | 120 / 96 | 1528 / 492 | 510 / 512 |
  | read of 255 on the engines | 874 206 / 874 214 (x 1.00) | 3 | 337 / 255 | 9174 / 1588 | 356 / 348 |

  A tenure of half a FIFO or less is ONE interrupt, its STOP; a long
  read is one every eight bytes, where one a byte took 285 interrupts
  and three and a half times the handler cycles for 255. The address and
  the speed stay in the block between tenures, so a one-byte write's
  fixed cost is the same at the three speeds (474 on the M33); the
  disable, IC_TAR and enable that a new address still pays costs 916
  cycles more at 100 kHz, 241 at 400 kHz and 106 at 1 MHz - the disable
  waits on the block's own shutdown, which follows the bus timing.
  **A HANDLER IN FLASH IS A LOTTERY ON THE CORTEX-M33**: the one-STOP
  handler measured 188 to 365 cycles across three flash builds of the
  bench (558 with stamps added inside it), with up to six XIP cache
  misses of about seventy cycles each in a tenure, against 84 from SRAM;
  the Hazard3 builds of the same source ranged 57 to 287. The driver
  places nothing (the application binds the vector); the bench measures
  its handlers from SRAM so that a line reads the code.
- **THE VENDOR**, the pico-sdk 2.3.1's `i2c_write_blocking` /
  `i2c_read_blocking` built for each half, on the same wire with this
  driver's counts written over its own: a one-byte write 7811 / 7838
  cycles, fixed 459 / 486 - within a twentieth of this driver's - with
  busy the whole wall where this driver's is 583 / 302. A read of 255
  bytes keeps one command in flight and holds the bus a turn of its loop
  every byte: 887 459 / 888 884 cycles, a fixed cost of 13 713 / 15 138
  against 676 / 634.

## Not covered yet

Driver gaps, each with its reason. The ones that belong to the block
rather than to this chip - the high-speed mode, SMBus, the stuck-bus
timeouts, 10-bit addressing on the host side, a write phase on the DMA
engines, the client on the engines - are in
[../dw_apb_i2c/README.md](../dw_apb_i2c/README.md) and are not repeated
here.

- Multi-master arbitration on the wire: ARB_LOST is decoded to
  `i2c_arb_lost`; a second host on the wire is the desk partner's,
  through a peer of the twi_link protocol.
- The general call and the START byte as host verbs: `target(addr,
  general_call)` is in the resource; no tenure shape asks for either.

Implemented but not bench-verified, each with what would measure it:

- The host and the client on a wire: the scan and the probe as this
  silicon serves it, the tenure shapes with the repeated START counted
  from the far end, the vocabulary (nack_addr, nack_data, the stretch,
  the flush), the three speeds' rate on the wire, the engines, I2cBus
  with its timed bus, the roles inverted - all of it on the pads above
  GP31: the desk partner's peer over the pair's I2C wires.
- The unstick's pulse train against a held SDA, and the ruler its
  half-bits take on the Cortex-M33 half (the platform timer, which a
  program there starts itself): the partner's peer holding SDA low.

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
