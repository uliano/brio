# SPI / I2S (STM32G0)

This document is for a programmer who needs a device on an SPI bus: a
display, a flash chip, a converter, several of them sharing the four
wires through the bus arbiter ([design/spi-bus.md](../design/spi-bus.md)
is the model), or this chip answering as a client; the same block is an
I2S audio interface. `Spi<n>` is the instance, one function per register
fact; `SpiHost<n, pins, ...>` the host engine `util/spi_bus.hpp` drives,
whose Request carries the other targets' fields by name, the rate, the
mode and the frame size as this family's types and in this family's
order ("Types and verbs" below); `SpiClient<n, pins>` the
client role.

The smallest use, a device on a bus through the arbiter, the chip
select a GPIO the engine owns for the transaction:

```cpp
constexpr brio::SpiPins bus_pins{
    .sck  = {'B', 3, brio::PinFunction::af0},   // SPI1_SCK  (DS13560 table 15)
    .miso = {'B', 4, brio::PinFunction::af0},   // SPI1_MISO
    .mosi = {'B', 5, brio::PinFunction::af0},   // SPI1_MOSI
    .nss  = {},                                 // software: the select is a GPIO
};
using Bus = brio::SpiHost<1, bus_pins>;
using Arb = brio::SpiBus<Bus, P, 4>;
using Cs  = brio::Pin<'A', 15>;

Cs::output(true);                 // deasserted
(void)Bus::init(clock, 8'000'000);   // 8 MHz ceiling for the whole bus

brio::post<Arb>(Bus::Request{
    .cs = Cs::ref(),
    .cmd = brio::lend<brio::Lease::reply>(cmd),
    .tx = {}, .rx = brio::lend<brio::Lease::reply>(rx),
    .reply = brio::reply_to<Device, brio::SpiDone>(),
    .len = 8, .cmd_len = 2,
    .clock = brio::SpiClock::div8, .mode = brio::SpiMode::mode0,
});

extern "C" void SPI1_IRQHandler() {
    if (Bus::isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
}
```

What decides correctness even there: the three pads named with the
alternate-function numbers the datasheet gives for that instance, which
nothing in the code can check; the chip select as a GPIO (the block's
hardware select frames the peripheral's lifetime and not a transaction,
a finding below); `init()` after the clock task's; and the instance's
vector bound to `isr()`, with the completion posted to the bus active
object. The request's buffers are lent until `SpiDone` arrives
([design/kernel.md](../design/kernel.md), the loans). On long or
jumpered wires the slew class of the driven pads is a correctness
parameter, set once in the configuration `init()` takes. And this
silicon enforces none of the chapter's configuration rule, so the
driver's refusals are the only protection there is.

## Documents of record

RM0444 Rev 6 - SPI/I2S ch. 35 (35.4's table 205 is
the implementation table, 35.5 the SPI functional description with the
configuration, enable and disable procedures of 35.5.7..35.5.9, the
status and error flags of 35.5.10/35.5.11, NSS pulse mode 35.5.12, TI
mode 35.5.13, the CRC 35.5.14, the interrupts 35.6, the I2S 35.7 and
the registers 35.9), the DMAMUX request table 56 (ch. 11), the APB
enables and resets 5.4.11..5.4.19 and the I2S kernel-clock multiplexers
5.4.21/5.4.22 (RCC_CCIPR and RCC_CCIPR2), the interrupt table 12.3
(table 61); DS13560 Rev 5 tables 13..19 (the AF numbers of the SPI and
I2S pads); errata ES0548 Rev 3 items 2.12.1 and 2.12.2, read on the
bench chip's revision Z column. Driver: `stm32g0/spi.hpp`; the shared
bus vocabulary above it is `util/spi_bus.hpp` over
`util/bus_master.hpp` ([../design/spi-bus.md](../design/spi-bus.md)).
The per-instance presence, APB register, vector, I2S capability, I2S
clock selector and DMAMUX facts come from `stm32g0/device_tables.hpp`.
Bench suite: `test_stm32_spi`, and it carries two instruments on one set
of pads because a desk carries either (the suite probes for the wire at
boot; its header comment is the map; a letter is one command key of the suite's console and the group of verdicts it runs):
letters `a`..`m` (89 verdicts) run on the Nucleo's own SPI1-to-SPI2
self-link, letter `s` (18 verdicts) on no wire at all - the host's two
loops in every shape, judged by what the block holds after each, the
receive shapes under a stretched tick handler, and the rate on the wire
at every BR code -,
letters `n`..`r` (20 verdicts) and `u`..`w` (18) on the link to a PEER
BOARD running `spi_peer`, commanded in band over
`avrxt/src/apps/spi_link.hpp` - `u`..`w` through its EXTENDED EXCHANGE
(16-bit frames, a client that only listens, both ends' CRC units), with
`t` and `x` beside them outside `z`. The peer may be any of the three ports of
that app - the AVR's, the SAM C21's, this stratum's own, which answers
on SPI1 AF0 at the same pin names this board hosts on, or the STM32F4's,
on the F446's SPI2 - and the peer's `ident` names which one answered
(firmware 0x01xx, 0x02xx, 0x03xx, 0x04xx). The
two wirings exclude each other, the suite PROBES which one is fitted
before any letter runs, and the letters whose instrument is absent skip
themselves with the reason printed and no verdict claimed. Family
fixture `test/family_stm32g0/spi.cpp` plus eighteen negatives under
`brio check stm32g0`.

## What the silicon does

Three instances of one block on the G0B1/G0C1 and two everywhere else,
each of which is an SPI **or** an I2S depending on one bit
(`I2SCFGR.I2SMOD`), with 32-bit transmit and receive FIFOs, 4-to-16-bit
frames, a hardware CRC and the enhanced NSSP and TI modes. Table 205 is
the roster and it has exactly two entries that move across the family:
**SPI3 exists only on the G0B1/G0C1, and SPI2 carries an I2S only
there** - one condition, so the reserve (`stm32g0/device_tables.hpp`,
the table of what each part has, read off the device header) derives
both from `SPI3_BASE`
and the device header's own `IS_I2S_ALL_INSTANCE` is the second opinion
the bench compares it against.

Five facts shape the driver.

**35.5.7 is a with-SPE-clear procedure, and the silicon does not enforce
it.** "Configuration of SPI" writes CR1 and CR2 with the peripheral
disabled; four fields say so in their own register description (CRCEN,
CRCL, FRF, NSSP, plus LDMA_TX/LDMA_RX), four more are gated by the
softer "should not be changed when communication is ongoing" (BR, MSTR,
LSBFIRST, CPOL/CPHA - and 35.5.6's Note hardens the last pair into "the
SPI must be disabled by resetting the SPE bit"). Measured on the bench:
**a raw write with SPE set lands on every one of the seventeen
configuration fields**, the four "only when disabled" ones included.
So the driver refuses all of them while enabled - one rule instead of
two, so no caller has to remember which sentence a field got - and that
refusal is the only thing standing between a program and a mid-transfer
reconfiguration.

**The disable is a PROCEDURE, and it is the only way this driver turns
the peripheral off.** 35.5.9: wait `FTLVL = 00`, wait `BSY = 0`, clear
SPE, read DR until `FRLVL = 00`. That makes ES0548 2.12.1's workaround
structural for the case it names, and the case it does not - a
receive-only master, whose clock stops only when SPE does - has its own
verb whose contract is the erratum's own advice (ignore BSY there).

**BSY is not a witness on the client side.** ES0548 2.12.2 says it may
sporadically stay high at the end of a slave transfer and names RXNE and
the NSS pin as the measures to use instead. Nothing on the client half of
this file waits on BSY, and every wait in the file is bounded and
reports rather than spinning.

**The FIFO is the host's write-ahead, and RXNE drives its pump.** TXE
means "the transmit FIFO has room", which is true well before a frame is
on the wire and true again immediately after; RXNE means "a frame has
been shifted in", which on a full-duplex bus is exactly "one frame
moved, both ways" - and reading DR is both the capture and the
acknowledgement. The host keeps the TXFIFO's worth of frames in flight -
three byte frames or two half-words, the capacity 35.5.9 gives the
TXFIFO under its own TXE rule - drains what RXNE announces and refills
by that count, so the clock runs continuously while the software keeps
up and the transfer ends when the last frame has come BACK, which is the
only edge that means the wire is idle. A phase that reads nothing back
takes the chapter's transmit-only arrangement instead: frames in as TXE
says, the unread echoes left to raise OVR, the tail waiting for FTLVL
and BSY and then clearing the flag by 35.5.11's sequence. No frame in
flight is ever lost to an overrun under any interrupt, because what is
in flight never exceeds the RXFIFO's own capacity. **FRLVL cannot count
three byte frames**: its two bits read 01, 10 and 11 for one, two or
three, and four bytes (measured), so the level is a witness of empty and
full and the pump drains on RXNE, frame by frame.

**The FIFO access width is part of the frame size.** Figure 363: with
DS <= 8 a frame is right-aligned in a byte and DR must be accessed a
byte at a time, or the silicon PACKS two frames into one 16-bit access;
with DS > 8 a frame is right-aligned in a half-word. FRXTH must agree
with the read width. Every read and write verb comes in both flavours
and `data(size, v)` picks for the caller.

## Types and verbs

`Spi<n>` - the resource. `regs()`, `irq()`, `bus_clock()`, `reset()`,
`dma_rx_request()` / `dma_tx_request()` / `data_address()` (the DMAMUX
pair and the address an engine is armed on), `i2s_kernel_clock()`,
`has_i2s_mode` (the reserve's constant) beside `has_i2s()` (the device
header's own macro at run time).

`enable()` and `disable()` are not a pair: the first is a store, the
second is 35.5.9's procedure and returns false when a bounded wait ran
out. `disable_receive_only()` is the chapter's other procedure and
`flush_rx()` its last step alone.

Configuration is `configure(SpiConfig)` - CR1, CR2 and CRCPR in three
stores - or the individual verbs `role`, `mode`, `clock`, `bit_order`,
`data_size`, `direction`, `nss`, `frame_format`, `crc`,
`crc_polynomial`, `dma_transmit`, `dma_receive`,
`last_dma_transmit_odd`, `last_dma_receive_odd`, each of which
**refuses while SPE is set and writes nothing**. Three verbs stay open
under a running SPI because they must: `software_select` (SSI - a
software client selects itself with it and a program stages a mode fault
with it), `crc_next` (35.9.1 has it written as the last data is), and
`rx_threshold` (35.5.9's note moves FRXTH mid-transfer to read the last
odd frame of a packed reception).

`SpiConfig` carries the whole option space and `spi_config_valid()`
refuses what the chapter forbids and no register checks: a CRC on a
frame size that is neither 8 nor 16 bits, an even polynomial, NSS pulse
mode with CPHA = 1 or in TI mode or on a client, hardware NSS output on
a client, and an LDMA bit without its DMA enable or with a frame wider
than a byte.

The vocabulary: `SpiMode` (mode0..mode3, bit 1 CPOL and bit 0 CPHA -
spelled the same way in all three targets), `SpiClock` (div2..div256,
the eight BR codes named by their DIVISION), `SpiDataSize` (bits4..
bits16), `SpiNss` (software / hardware_input / hardware_output /
hardware_pulse - a reading of SSM, SSI and SSOE that makes the four
legal arrangements nameable), `SpiDirection`, `SpiFrameFormat`,
`SpiCrcLength`, `SpiRxThreshold`, `SpiFlag`, `SpiInterrupt`.
`spi_rate_for(pclk_hz, max_sck_hz)` picks the fastest code at or below a
ceiling and REFUSES one the prescaler cannot reach.

`SpiHost<n, pins, TxEngine, RxEngine, Placement>` - the transfer engine
`util/spi_bus.hpp` drives. Its `Request` carries the other targets'
fields by name (cs and dc `PinRef`s, `cs_setup_us`, a command phase and
a full-duplex data phase with `Borrowed<..., Lease::reply>` loans,
`len`, `reply`, and the per-transaction bus configuration) with the
rate as a `SpiClock`, the mode as a `SpiMode` and the frame size as a
`SpiDataSize` defaulting to eight - so an 8-bit request looks exactly
like the other targets'. The ORDER is this family's: the two pins, the
three buffers and the reply first, the lengths and the bytes last -
forty bytes with no padding, and a designated initializer follows it.
A polled request is read through the reference and copied nowhere
(the arbiter lends it for the call); an asynchronous one is copied once,
ten word moves, because the pump reads it for its whole tenure.
`init(clock, max_sck_hz = 0)` (the shared surface) and `init(clock,
Config)` - `Config` being `max_sck_hz`, `sck_speed` and `mosi_speed`,
the classes of the two pads a host drives ("The pads") -, `sck_speed()`
and `mosi_speed()` (each moving its class between transactions),
`start(req)` (true = completed inside
start, false = the ISR posts `TransferDone`), `isr()`, `dma_isr()`,
`status()` (`spi_ok`, `spi_dma_fault`, or `spi_stalled` when a polled
wait ran out of its budget - the peripheral clocked nothing, the select
raised and the FIFOs drained), `prime()`, `bit_order()`, `recover()`,
`rebase(hz)`, `clock_for(hz)`, `sck_hz(code)`, `release()`. Three
constants state the engine's numbers: `ahead_bytes` / `ahead_halfwords`
(3 and 2, the frames the pump and the receiving polled loop keep in
flight), `packed_min_frames` (6, the byte phase from which the polled
receive packs two frames an access), and `dma_min_frames` (8, the data
phase from which an engined host hands the data to its engines - the
quotient of `dma_fixed_cycles`, what an engined transaction costs above
the wire, 1093 by letter d, and `pump_cycles_per_frame`, the pump's busy
per frame, 138 by letter e; a shorter data phase takes the pump even
with engines bound). `Placement` is util/placement.hpp's tag, `InFlash`
by default: under `InRam` the pump's entry and the polled transaction
each run as one instance in SRAM, `placed_in_ram` says so, and the
pump's cost follows it - 122 cycles a frame, `dma_min_frames` 9 ("The
host's placement" below).

`LEN IS A FRAME COUNT.` With the default 8-bit frames a frame is a byte
and `len` is a byte count. With `bits` above eight a frame occupies TWO
bytes of the caller's buffer, low byte first, and `len` still counts
frames. Below eight bits a frame is still one byte, right-aligned, with
the unused top bits ignored on the way out and read back as zero.

`SpiClient<n, pins>` - the other end: the polled RUNS and, as their
degenerate case, the per-frame verbs. `init(clock, Config)` (the
`SpiConfig` fields a client can be told, `drive_output`, and
`miso_speed`, the class of the one pad a client drives), `enable()`
(SPE up, the FIFO left for a run to fill) and `enable(first, second)`
(SPE up with the first TWO answers already in the FIFO),
`answer(out, spins)`, `exchange(out, in, spins)`, `receive(in, spins)` -
each returning a `Run` (`count`, and a `RunEnd`: `complete`, `closed`,
`stalled`) over `uint8_t` frames of 4..8 bits, moved two to an access,
or `uint16_t` frames of 9..16, a wider element refused at compile
time - `frames_ahead` (TWO, the FIFO's own number: how many answers a
per-frame pump must keep queued ahead of the host's clock), `write`,
`poll`, `selected()` (a live read of the NSS PAD - the peripheral
publishes no such status bit), `drive_output` (the dark listener), the
error accessors, `isr()`, `disable()`, `release()`.

## The client run

A client's unit is the run: the answers to a transaction, or what it
brings, in one polled call. What this block offers a run, each item used
or declined:

- **The 32-bit TXFIFO under its half-full TXE** (35.5.9: TXE "triggered
  when the TXFIFO level is less than or equal to half of its capacity",
  three 8-bit frames, or four in two half-word accesses) - USED: the run
  fills it with four answers while the select is still high (35.5.8:
  "the data register of the slave must already contain data to be sent
  before starting communication with the master"), and writes on TXE,
  which rises with two frames still queued behind the one shifting - a
  write has three frames of window.
- **Data packing** (35.5.9: "used automatically when any read or write
  16-bit access is performed" with a frame of a byte or less, the LOW
  half leaving first) - USED: two 8-bit frames a write and two a read,
  one turn for two frames. The caller's run need not be aligned (two
  byte loads make the pair; this core takes no unaligned access).
- **FRXTH** - USED: the half threshold (RXNE at two frames) for the
  pairs, the quarter for the odd last frame, moved when one frame is
  left (35.5.9's note); the run leaves the quarter, where `poll()` reads.
- **The 32-bit RXFIFO**, four frames - USED: the reads trail the writes
  by the TXFIFO's four and the RXFIFO holds four, so the steady state
  never overruns. FRLVL drains what a close or a stall left.
- **No end-of-transaction flag on a slave, BSY no witness** (ES0548
  2.12.2) - DECLINED BSY: the NSS PAD is the window. A run reads the pad
  once every four polls of the flag, waits for it to fall before a high
  pad means the close, and without a pad ends on its count or budget.
- **The TXE and RXNE interrupts** - DECLINED for the runs: a run is a
  polled loop; the ISR body stays for a program that wants an interrupt
  client.
- **The DMA requests** - DECLINED in the driver: SpiClient has no engine
  slots (the gap list); a program binds the channels raw, as the stm32g0
  `spi_peer`'s exchange serve does.
- **The overrun** (35.5.10) - `answer()` reads nothing, so OVR rises
  after four frames; its tail drains the RXFIFO, whose last read is the
  sequence's DR read, and reads SR (35.5.11) - no read of an empty FIFO.
- **The CRC unit** (35.5.14) - NOT in the runs: a run has no CRC phase
  (the gap list). The stm32g0 `spi_peer` serves spi_link's CRC beside
  them two ways, both measured from the F446 (below): on its DMA
  channels with the chapter's own "CRC transfer managed by DMA" - the
  transmit channel counting the data frames alone, the receive channel
  the host's CRC frame too, CRCNEXT never written - and with a run of
  its own over the resource, CRCNEXT the store after its last answer
  ("as soon as the last data is written", 35.9.1) and a frame read an
  access, the reception buffer being "a single 16-bit buffer used to
  receive only one data frame at a time" in CRC mode (35.5.14).
- **TI frames, NSS pulse** - not in the runs.

COST, counted in the release listing: the block's base held in a
register by an empty asm, so SR, both views of DR and CR2 are offsets
from one base (folded to constants, each address was a literal-pool load
in the turn); `answer()`'s steady state is twelve instructions a PAIR -
the TXE poll that hits, two byte loads combined, one half-word store, the
loop test - and `exchange()`'s twenty-one a pair, four peripheral
accesses (the two flag reads that hit, the store, the half-word read) and
one stack load, no call in either loop. The vendor's slave loop (HAL
v1.4.7 `HAL_SPI_TransmitReceive`, 8-bit) moves a frame an access with one
queued and calls `HAL_GetTick()` on every pass. The runs are always
inline and the program places their caller: here the flash, behind its
two wait states and the prefetch.

MEASURED by `test_stm32f4_spi` on the STM32F446RE it is paired with
(letter p, the stm32g0 `spi_peer` asked for its polled run with
`spare_polled_pump`): `exchange()` byte-exact both ways at every code up
to the F446's PCLK1/2 = 22.5 MHz - the host's own ceiling, the wire's
fastest on this desk, and inside this client's 32 MHz (DS13560 table 76,
2.7..3.6 V). As the client of the F446's `host_burst` (letter r below,
twelve frames at 1.406 MHz) the exchange run is exact both ways. The
same suite's extended letters judge the rest of the surface at every
code to 22.5 MHz: `exchange()` over `uint16_t` frames, 32 a window
(letter `u`); `receive()` with the answer line dark at 7 and 33 frames
in both widths - the pairs under the half threshold and the odd last
frame under the quarter one - and `exchange()` at 7 and 33 byte frames,
the odd last frame read under the moved threshold (letter `v`).

## The pads

The SPI's pads are GPIO pads in an alternate function, and what the
silicon offers there that bears on a wire is the output stage's SLEW
CLASS - OSPEEDR, four classes (RM0444 7.5.3 "very low, low, high, very
high", brio's `PinSpeed` `low`, `medium`, `high`, `very_high`), their
reach in the data sheet's output timing (DS13560 table 58, 2.7..3.6 V):
2 MHz at 50 pF, 10 MHz at 50 pF, 30 MHz at 50 pF, 60 MHz at 30 pF. Each
item used or declined:

- **The class of every DRIVEN pad, as configuration** - USED.
  `SpiHost::Config` names `sck_speed` and `mosi_speed`,
  `SpiClient::Config` names `miso_speed`. `init()` writes each with its
  pad's alternate function, once; the client's `drive_output(true)`
  re-applies its class wherever it hands the answer line back; no
  transaction touches OSPEEDR. The host's `sck_speed()` and
  `mosi_speed()` move a class between transactions for a program that
  weighs it, and the next `init()` writes the `Config`'s again. The
  default is `very_high` on all three, the class the SPI's timing is
  characterized at (OSPEEDR = 11 at 30 pF: DS13560 table 76, DS12232 and
  DS12992 the same); no erratum of ES0548 names a pad.
- **An input's class** - none to give: OSPEEDR acts on the output stage,
  so a host's MISO and a client's SCK, MOSI and NSS have no field, and a
  class named for one does not compile (two negative TUs).
- **The select as a hardware output** - `claim_nss_pad()` hands it at
  `very_high` and has no field: SSOE and NSSP are measured on the
  self-link, where the default holds, and no program drives them on a
  wire.
- **The FT_c pads** (RM0444 7.5.3's note: they "cannot be set to high
  speed and to very high speed") - NOT CHECKED: which pad is one is the
  data sheet's pin table (on the G0B1, PD0 and PD2, the UCPD's CC lines,
  PD0 carrying SPI2_NSS), and no header of the pack carries it (the gap
  list).

COST, read in the release listings: `init()` stores each class in the
task's state (one byte store) and hands it to the pad's
read-modify-write; `drive_output()` loads the client's once; `start()`,
the pump, the engines and the runs contain no GPIO access but the
select's BSRR store.

ON A BUS OF WIRES THE CLASS IS A CORRECTNESS PARAMETER OF EVERY PAD, and
which class is the BUS's: a fast edge on SCK is seen twice at a far
client, a fast edge on MOSI or MISO couples into the clock beside it, and
either puts the far client's stream one bit out of place from that frame
on. On the desk's jumper wires to the STM32F446RE this host runs MOSI at
`high`, and this board's `spi_peer` answers at `very_high` in modes 0
and 3 and at `medium` in 1 and 2 ("The peer link"); the self-link's
short wires keep the default.

## How to use it

**A device on a bus, through the arbiter.** The chip select is a GPIO
the engine owns for the transaction's duration, exactly as on the other
two targets:

```cpp
constexpr brio::SpiPins bus_pins{
    .sck  = {'B', 3, brio::PinFunction::af0},   // SPI1_SCK  (DS13560 table 15)
    .miso = {'B', 4, brio::PinFunction::af0},   // SPI1_MISO
    .mosi = {'B', 5, brio::PinFunction::af0},   // SPI1_MOSI
    .nss  = {},                                 // software: the select is a GPIO
};
using Bus = brio::SpiHost<1, bus_pins>;
using Arb = brio::SpiBus<Bus, P, 4>;
using Cs  = brio::Pin<'A', 15>;

Cs::output(true);                 // deasserted
(void)Bus::init(clock, 8'000'000);   // 8 MHz ceiling for the whole bus
// ... or, on a bus of long wires, the two driven pads a class down:
// (void)Bus::init(clock, {.max_sck_hz = 8'000'000, .mosi_speed = brio::PinSpeed::high});

brio::post<Arb>(Bus::Request{
    .cs = Cs::ref(),
    .cmd = brio::lend<brio::Lease::reply>(cmd),
    .tx = {}, .rx = brio::lend<brio::Lease::reply>(rx),
    .reply = brio::reply_to<Device, brio::SpiDone>(),
    .len = 8, .cmd_len = 2,
    .clock = brio::SpiClock::div8, .mode = brio::SpiMode::mode0,
});

extern "C" void SPI1_IRQHandler() {
    if (Bus::isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
}
```

**The data phase on DMA.** Name both engine slots and bind the channels'
vectors as well; the command phase stays on the frame pump and the
engines take the data phase at its end. Engines of `uint16_t` serve a
frame of either width, each with its own beat - a byte access to DR for
eight bits or fewer, a half-word one above (35.5.9) - and a wide request
whose buffers are not half-word aligned goes to the frame pump, decided
before the select. The transmit engine is armed for its errors alone:
the receive block's completion is the transaction's, and it proves the
transmit block's, so a transaction takes ONE interrupt; its channel's
vector is still bound, for an error. THE RECEIVE CHANNEL ARMS AT
`very_high` AND THE TRANSMIT ONE AT `high` (docs/design/dma.md): the
host clocks a frame out whenever the transmit channel fills the TXFIFO,
and a receive channel it outran would find the 32-bit RXFIFO full - OVR,
the frame discarded and every one after it lost (35.5.11) - where a
transmit channel that waits only stops the clock between two frames.
`release()` on the host and on the client pulses the block's reset
before the gate: a DMA enable left set raises its request again when the
clock returns ([dma.md](dma.md), "A released requester holds nothing
here").

```cpp
using Tx = brio::DmaTxEngine<1, 1, uint16_t>;
using Rx = brio::DmaRxEngine<1, 2, uint16_t>;
using Bus = brio::SpiHost<1, bus_pins, Tx, Rx>;
(void)Bus::init(clock);            // arms both engines: the gate, the requests, DR
extern "C" void DMA1_Channel1_IRQHandler()   { if (Bus::dma_isr()) { /* post */ } }
extern "C" void DMA1_Channel2_3_IRQHandler() { if (Bus::dma_isr()) { /* post */ } }
```

**The host's hot paths in SRAM.** The placement is the last template
argument; the vectors stay as they are:

```cpp
using Display = brio::SpiHost<1, bus_pins, brio::NoDmaEngine, brio::NoDmaEngine, brio::InRam>;
```

The pump's entry runs from SRAM behind SPI1's vector and every polled
request calls its transaction there - 624 bytes and 1644 of SRAM, an
eighth off the pump's entry and a twentieth off the receive loop at
PCLK/2 ("The host's placement" below).

**A client that answers.** Load the first two answers before the host's
clock can arrive and write the next-plus-one on every received frame -
the pump runs ONE AHEAD:

```cpp
using Peer = brio::SpiClient<2, peer_pins>;
(void)Peer::init(clock, {.mode = brio::SpiMode::mode0});
Peer::rxne_interrupt(true);
Peer::enable(answers[0], answers[1]);

extern "C" void BRIO_STM32G0_SPI2_HANDLER() {
    if ((Peer::isr() & brio::SpiFlag::rxne) != 0u) {
        const auto in = Peer::poll();
        Peer::write(next_answer());
    }
}
```

**An I2S pair.** The kernel clock must be at least PCLK (35.7.4's
warning), which at 64 MHz means SYSCLK:

```cpp
(void)brio::Spi<1>::i2s_kernel_clock(brio::I2sClock::sysclk);
(void)brio::Spi<1>::i2s_configure({.mode = brio::I2sMode::master_transmit,
                                   .standard = brio::I2sStandard::philips,
                                   .divider = 21});
(void)brio::Spi<1>::i2s_enable();
brio::Spi<1>::data_halfword(sample);
...
(void)brio::Spi<1>::i2s_stop_transmit();   // TXE = 1 then BSY = 0, then I2SE down
```

`i2s_sampling_hz()` and `i2s_prescaler_for()` are 35.7.4's four formulas
and their inverse.

## Bench findings

Measured by `test_stm32_spi`. The findings down to "The dynamic clock"
are the SELF-LINK's (SPI1 host to SPI2 client, four wires): 89 verdicts.
The ones under "The peer link" are a SECOND CHIP's, on the same four
SPI1 pads: 38 verdicts, measured against the STM32F446RE paired with
this board - and this board's `spi_peer` measured from that one.

**The block, and the reserve against the header.** SPI1 on line 25 and
SPI2/SPI3 sharing line 26; the DMAMUX pairs 16/17, 18/19, 66/67; the
reserve's I2S table and `IS_I2S_ALL_INSTANCE` agreeing on all three
instances. Every register comes out of an RCC reset at table 209's own
value (CR1 0, CR2 0x0700, SR 0x0002, CRCPR 0x0007, I2SCFGR 0, I2SPR
0x0002) - and **the reset CR2 is the one misaligned combination the
chapter warns about**: DS = 0111 (8-bit) with FRXTH CLEAR, i.e. a
byte frame size with a half-word read threshold, so a driver that
trusted the reset value would read DR at the wrong granularity.

**35.9.2's own sentence, measured: a "Not used" DS code is FORCED to
0111 and not ignored** - `DS = 0010` written, `DS = 0111` read back. The
register lies about what it holds, which is why the driver refuses the
code rather than trusting a readback.

**THE SILICON ENFORCES NONE OF THE CHAPTER'S CONFIGURATION RULE.** With
SPE set, a raw write lands on all seventeen of CPOL, CPHA, BR, MSTR,
LSBFIRST, CRCEN, CRCL, SSM, RXONLY, BIDIMODE, DS, FRXTH, FRF, NSSP,
SSOE, LDMATX and LDMARX - the four whose register description says "only
when the SPI is disabled" included. The driver's refusals are therefore
the only protection there is.

**A PAD THAT IS NOT AN ALTERNATE FUNCTION READS LOW AT THE PERIPHERAL'S
NSS INPUT.** Clearing SSM on a master whose NSS pin is an ordinary GPIO
output raises MODF instantly - the peripheral's NSS input does not see
the pad's level, it sees the (low) alternate-function input - and the
silicon then clears SPE and MSTR and BLOCKS THEIR RETURN until MODF is
cleared (35.5.11's last sentence). Found by a probe that meant to
measure something else. It is why the transfer engine's boot
configuration is `SpiNss::software` and why `SpiHost::init()` never
claims the NSS pad.

**The link.** Sixteen 8-bit frames byte-exact in both directions with
the host polled and the client one frame ahead on its own interrupt;
the same on the host's RXNE pump with exactly ONE completion; a
two-phase request arriving as one select window of three command frames
then eight data ones. All four CPOL/CPHA combinations and both bit
orders byte-exact, with a DORD mismatch producing an exact two-way bit
reversal as the control. **Every frame size from 4 to 16 bits crosses
exact in both directions** with a pattern that uses every bit, which is
also the proof of figure 363's access rule; the unused top bits of a
short frame come back ZERO (a 4-bit frame carrying 0xFF reads 0x0F at
both ends).

**THE RATE LADDER SAYS MORE ABOUT THE CPU THAN ABOUT THE WIRE.** Paced
by the engine's own polled pump - one frame written, waited for, read -
**all eight BR codes carry the link byte-exact in both directions, up to
PCLK/2 = 32 MHz**, because the gap between frames is the HOST's loop and
the client has all of it to answer in. Driven as a BURST instead, by a
hand loop that keeps the transmit FIFO full and reads every frame back
while the client answers each frame on its own vector on the SAME core,
the measured frame period **floors at about 320 CPU cycles** (322 at
PCLK/2 where the wire asks 16, 316 at PCLK/16 where it asks 128), so
that ladder never presses the far end however fast BR is set. The floor
is not the host loop's own: the driver's transmit-only loop, with no
client to serve, holds the clock continuous at PCLK/2 (below, "The rate
on the wire"), so what the burst pays is the polls and the loads beside
the client's per-frame vector; how the 320 cycles divide between them
is owed a measurement on the self-link. The client's receive side is byte-exact at all eight
rungs with OVR never rising. What slips is the client's ANSWER, and it
slips NON-MONOTONICALLY (a faster rung passes above a slower one that
failed), which is a late interrupt and not a ceiling - so the
turnaround ceiling is declined there and measured in the DMA letter
instead.

**A POLLED BURST LOOP ERASES THE EVIDENCE OF ITS OWN OVERRUN.**
35.5.11's clear sequence is "a read of DR followed by a read of SR" -
exactly what a loop that reads the frame and then polls TXE/RXNE does,
every turn. When the burst does lose a frame (intermittently, at the
rung where the wire's frame period and the loop's are within a quarter
of each other) the host's OVR flag reads CLEAR afterwards and the only
witness left is that the frames do not add up.

**NSS, all four ways.** The client's hardware NSS input really is the
transaction: four frames clocked with NSS high reach it not at all, and
the next four with the select low all arrive. **Hardware NSS output is
literal in 35.5.5, read from the far end of the wire**: it falls when
SPE is SET - measured at 77 CPU cycles after the CR1 store, so a program
that read the level in the next instruction would read the old one -
stays low across every frame of a burst, and rises only when the
peripheral is disabled. It frames the peripheral's LIFETIME, not a
transaction. **NSS pulse mode frames a DATA FRAME**: eight frames raise
the far pad eight times, counted on EXTI line 12 - a pulse per frame,
which is what a latching slave wants and the opposite of what a device
driver's chip select needs. Between them they are the whole reason the
transfer engine's select is a GPIO.

**The CRC is the arithmetic.** Over six 8-bit frames with the reset
polynomial the host's TXCRCR is 0x5A and so is a bitwise software loop
over the same polynomial; the client's RXCRCR over the same six frames
is the same number; the other direction likewise, and the frame the host
reads back where the checksum belongs is the client's own TXCRCR. Also
0x5204 for a 16-bit CRC over 8-bit frames and 0xABD0 for 16-bit frames -
every one matching the software reference exactly. **The checksum frame
is the one NEITHER SIDE WRITES**: after CRCNEXT the shifter is fed from
the CRC register and a store there is one more DATA frame instead, at
whichever end makes it.
35.5.14's arithmetic confirmed: a 16-bit CRC over 8-bit frames costs
TWO extra frames on the wire, a matched pair costs one. CRCERR is staged
by giving the client a different polynomial - what can be corrupted on a
wire that carries exactly what was put on it is the RECEIVER'S
EXPECTATION - and 35.9.3's rc_w0 (a write of ZERO, not a W1C) clears it.

**TI mode** carries eight frames byte-exact in both directions with FRF
set at both ends, the polarity and phase forced by the protocol. FRE
staged by an 8-bit master against a 16-bit TI slave DID NOT REPRODUCE:
the slave resynchronized on the next pulse and the flag stayed clear.
Recorded, claimed neither way.

**ES0548 2.12.1 REPRODUCES.** A raw SPE clear with three frames in the
transmit FIFO leaves BSY standing; the same state disabled through
35.5.9's procedure leaves BSY low and both FIFOs empty. 2.12.2's own
subject did not reproduce in 32 rounds (BSY was never still high when
RXNE rose) - which proves nothing about a sporadic clock coincidence,
and the honest witness the erratum names, RXNE, rose on every one of the
32.

**The overrun is literal.** Six frames into a client nothing drains fill
a FOUR-frame RXFIFO and raise OVR, and "the newly received value does
not overwrite the previous one" means the FIFO keeps the FIRST four, not
the last four.

**The DMA engines, and the ceiling the CPU could not find.** Thirty-two
frames out and back byte-exact with no CPU between them and ONE
completion - the receive block's. The command-phase handover loses and
repeats nothing at the seam. A read-only request feeds 0xFF from a held
cell and a write-only one drains into a held sink (the two sibling verbs
`start_fixed()` and `start_discard()` of `stm32g0/dma.hpp`). **With BOTH
ends on DMA channels the ladder is monotone and has a real ceiling:
byte-exact to PCLK/4 = 16 MHz, slipping at PCLK/2 = 32 MHz** - a channel
sustains frames an interrupt entry cannot, which is the distinction the
CPU-driven ladder could only name.

**The engined transaction's cost, with nothing on the wire**
(`bench_stm32` letter `d`: SPI1 on PB3/PB4/PB5 with MISO floating, the
engines `uint16_t` on DMA1 channels 2 and 3, the completion awaited on
the idle path; [../design/benchmark.md](../design/benchmark.md)'s
grammar, n in bytes, the wire SCK/8 bytes a second):

| op | n | PCLK/ | wall | busy | irq | isr | wire cycles | rest |
|---|---|---|---|---|---|---|---|---|
| spi.dma (8-bit) | 16 | 2 | 1352 | 1028 | 1 | 267 | 256 | 1096 |
| spi.dma (8-bit) | 256 | 2 | 5192 | 1028 | 1 | 267 | 4096 | 1096 |
| spi.dma (8-bit) | 16 | 8 | 2117 | 1028 | 1 | 267 | 1024 | 1093 |
| spi.dma (8-bit) | 256 | 8 | 17477 | 1028 | 1 | 267 | 16384 | 1093 |
| spi.dma.w16 | 32 | 2 | 1627 | 1047 | 1 | 267 | 512 | 1115 |
| spi.dma.w16 | 512 | 2 | 9307 | 1047 | 1 | 267 | 8192 | 1115 |
| spi.dma.w16 | 32 | 8 | 3160 | 1046 | 1 | 267 | 2048 | 1112 |
| spi.dma.w16 | 512 | 8 | 33880 | 1046 | 1 | 267 | 32768 | 1112 |

Every frame beyond the first costs exactly the wire's time - 16.0 HCLK
cycles a byte at PCLK/2, 64.0 at PCLK/8, either width - so the engines
keep the FIFO fed at the top rate and the per-byte time is the wire's.
What a transaction adds is a constant some 1100 cycles: `start()`
itself (452 measured alone, a ruler read of ~62 in it: the compare of
the settings, the select, the engines' five stores each), ONE completion
interrupt whose handler is 267 cycles with the meter's stamps, and the
idle path's wake with the bench's own idle-window reads. That constant
- the 8-bit line at PCLK/8, 1093 - is `dma_fixed_cycles`, the numerator
of `dma_min_frames`. The completion's vector is the bench's DMA vector,
which also binds letter u's ports: in `bench_stm32_ram`, whose ports'
bodies are calls into SRAM, the same vector's entry is 246 cycles and
the transaction 1069..1083 above the wire - none of the engines' path
is placed, so the constant is one for both placements.

**The host above the wire** (`bench_stm32` letter `e`: the engineless
`SpiHost<1>` on the same pads, MISO floating, each op the best of 8 runs
of `start()` alone with the Request built outside the clock; the write
shape reads nothing back, the `.rx` one reads every frame into a
buffer; the levels and OVR read after every run were zero and clear on
every line). The polled loops and the pump, before and after this
round, 256 frames, both placements of today's host:

| op | PCLK/ | before: wall, x | today, `InFlash`: wall, x | today, `InRam`: wall, x | the vendor's loop on the same board |
|---|---|---|---|---|---|
| spi.poll (8-bit write) | 2 | - | 4687, 1.14 | 4467, 1.09 | - |
| spi.poll | 4 | 60760, 7.41 | 8558, 1.04 | 8563, 1.04 | 43368, 5.29 |
| spi.poll | 16 | 83791, 2.55 | 33136, 1.01 | 33121, 1.01 | 58732, 1.79 |
| spi.poll.rx (8-bit read) | 2 | - | 6470, 1.57 | 6189, 1.51 | - |
| spi.poll.rx | 4 | 60760, 7.41 | 8609, 1.05 | 8535, 1.04 | 43368, 5.29 |
| spi.poll.rx | 16 | 83791, 2.55 | 33113, 1.01 | 33099, 1.01 | 58732, 1.79 |
| spi.poll16 (16-bit write) | 2 | - | 8524, 1.04 | 8514, 1.03 | - |
| spi.poll16 | 4 | 68411, 4.17 | 16706, 1.01 | 16712, 1.02 | 45395, 2.77 |
| spi.poll16 | 16 | 117002, 1.78 | 65852, 1.00 | 65856, 1.00 | 96848, 1.47 |
| spi.poll16.rx | 2 | - | 10536, 1.28 | 10028, 1.22 | - |
| spi.poll16.rx | 4 | 68411, 4.17 | 16691, 1.01 | 16697, 1.01 | 45395, 2.77 |
| spi.poll16.rx | 16 | 117002, 1.78 | 65848, 1.00 | 65840, 1.00 | 96848, 1.47 |
| spi.pump (8-bit, interrupts per frame; busy per frame) | 4 | 13.32 (1.00; 318) | 5.34 (0.34; 171) | 4.77 (0.34; 153) | - |
| spi.pump | 16 | 3.74 (1.00; 636) | 1.35 (0.34; 135) | 1.21 (0.34; 119) | - |
| spi.pump.rx | 4 | 13.32 (1.00; 318) | 5.44 (0.34; 174) | 4.86 (0.34; 156) | - |
| spi.pump.rx | 16 | 3.74 (1.00; 636) | 1.37 (0.34; 138) | 1.24 (0.34; 122) | - |
| spi.pump16 | 4 | 6.61 (1.00; 315) | 3.65 (0.50; 234) | 3.23 (0.50; 207) | - |
| spi.pump16 | 16 | 2.45 (1.00; 464) | 1.29 (0.67; 285) | 1.22 (0.50; 234) | - |

Before, the polled loop wrote a frame, spun on RXNE and read it back
through two out-of-line verbs - 237 cycles a frame whatever the rate -
and the pump took one interrupt a frame at 315 cycles each with the
bench's stamps. After: the write shape is the chapter's transmit-only
loop (frames in on TXE, two byte frames an access, the unread echoes'
overrun cleared at the tail), wire-bound from PCLK/4 at both widths;
the receive shape keeps three frames in flight frame by frame below six
and PACKS above it (two frames an access each way, two pairs in flight,
the odd last one by a byte with FRXTH moved for it and back), 33.6
cycles a frame against the wire's 32 at PCLK/4 and wire-bound at
PCLK/16 - at PCLK/2 25.3 against 16, the one rate the core's loop does
not keep up with, at either placement; the pump drains what RXNE holds
and refills three, so it takes an interrupt every three frames at
PCLK/4 and at PCLK/16 alike (six for sixteen frames, 86 for 256) and
keeps the bus busy where the handler returns within three frame times:
at PCLK/16 that is 384 cycles against the bench's stamped handler of
397..406 (x 1.35), at PCLK/4 96 cycles against a handler no vector can
run in (x 5.3, the bus idle between refills - the engines' rate, not
the pump's). The 16-bit pump keeps two frames in flight, 512 cycles at
PCLK/16, and its flash handler of 329 cycles takes one frame an entry
as often as two there (0.67 interrupts a frame; from SRAM 0.50). The pump's busy per frame at PCLK/16, 138 cycles with the
stamps inside (122 under `InRam`), is `pump_cycles_per_frame`.

The vendor's loop is ST's HAL v1.4.7 `HAL_SPI_TransmitReceive`,
compiled from the pinned archive into a scratch program with brio's
crt, clock and console around it and measured with the same ruler on
the same pads: one frame in flight (a `txallowed` flag alternates a
TXE-gated store with an RXNE-gated load) and a `HAL_GetTick()` test per
turn, 169 cycles a frame at PCLK/4 whatever the width - it does NOT pack
in its polled path (the packing is its interrupt receiver's) - so brio's
loops are four to five times faster at PCLK/4 and within a tenth at
PCLK/16 where both are the wire's.

**A request's fixed cost** - the price of a DCS command - from the same
letter: a polled command of one byte and 0, 2 and 15 bytes of data,
write-only, the select on PA15 and the D/C on PB6, at PCLK/16:

| op | n | before: wall, rest | today, `InFlash`: wall, rest | today, `InRam`: wall, rest |
|---|---|---|---|---|
| spi.req | 1 | 781, 653 | 444, 316 | 429, 301 |
| spi.req | 3 | 1382, 998 | 797, 413 | 765, 381 |
| spi.req | 16 | 5269, 3221 | 2504, 456 | 2476, 428 |

"Rest" is wall minus the wire's cycles, the Stopwatch's ~62 inside it;
the 3-byte request's 351 net cycles (319 under `InRam`) are: the compare of the settings
and the clamp (~15 instructions), the select and the D/C edges (one
store each, inline), the command phase (one frame in, the tail's wait
on FTLVL and BSY, the overrun's clear and the drain - two reads), the
D/C flip, the data phase (one half-word access for the pair, the same
tail), the select raised, and the silicon's own latencies at each
phase's two ends (the first edge after the write, BSY after the last) -
some 180 instructions and 14 APB accesses from a flash at two wait
states, and one call - into the transaction's own function, where the
placement's flatten puts every loop (polled_transaction() in the
header): the same transaction unflattened and inline in `start()`,
measured in a scratch build, took 413, 768 and 2460 cycles - some 32
less a request - and 9576 for the 256-frame receive at PCLK/2, where
the flattened function takes 6470. What the round took out: the 48-byte copy
of the request (12 word moves), `flush_rx()` (18 instructions and a
call; the loops read back what they wrote, so nothing is stale - the
abnormal exits drain), the RXNE interrupt's disarm under a guard (never
armed outside an asynchronous tenure), two `PinRef` calls, and per
frame two out-of-line `data()` calls and a bounded spin of its own.

**The host's placement** ([platform.md](platform.md), "Code in SRAM"):
`SpiHost<..., InRam>` runs the pump's entry and the polled transaction
each as ONE flattened instance in SRAM - 624 and 1644 bytes on the
engineless host (both widths, every shape), 684 and 1612 on an engined
one - behind the vector and the `start()` that stay in the flash, each
a call through the linker's long-branch veneer. The engines' path takes
no placement (once a transaction), nor does `SpiClient` (its runs are
inline in their caller, which the program places). Measured by
`bench_stm32` and `bench_stm32_ram` (the tables above): the pump's
entry 397 cycles against 347 for three byte frames and 356 against 308
for two half-words, so the pump's 256 frames at PCLK/2 take 39125
cycles against 43824 and its busy a frame at PCLK/16 is 122 against
138; the polled loops a twentieth at PCLK/2 (the receive 6470 against
6189, 16-bit 10536 against 10028) and nothing at PCLK/4 and below,
where the wire is the bound; a polled request 15 to 32 cycles. The
pump's cost per frame follows the tag, and the crossover to the
engines with it: `dma_min_frames` 8 under `InFlash`, 9 under `InRam`.

**Three facts the letter found.** (1) ARM RXNE AFTER THE PRIME, not
before: with the interrupt armed first and three frames written after
it, at PCLK/4 the first frame came back before the pump's count was
stored - the handler ran between the second write and the store,
refilled from a count of zero and overran the FIFO; the first run left
three frames behind and every run after it inherited them (FTLVL 1,
FRLVL 2 after a "completed" transaction, four frames an interrupt where
three were in flight). The frames written before the arming wait in the
RXFIFO and raise the line the moment it is armed. (2) FRLVL's two bits
cannot count three byte frames (above): a drain paced by the level left
one frame an entry behind at PCLK/4, eight interrupts for sixteen
frames instead of six. (3) A two-byte phase packed costs 35 cycles MORE
than frame by frame: the two accesses to CR2 for the threshold moved
and moved back outweigh the two accesses to DR a pair saves, which is
where `packed_min_frames` comes from.

**The longest handler in the image** bounds nothing here but the bus's
continuity: what is in flight on every path - written and not yet read
back - never exceeds the RXFIFO's capacity, the TXFIFO drains into it
and the clock STOPS, so a handler of any length delays the loop or the
pump and loses no frame. Measured by `test_stm32_spi` letter `s`, a
wireless letter: with the tick's handler stretched to some 1500 cycles
(four byte frames at PCLK/2 are 64), a hundred receive runs of 256
bytes each at PCLK/2, PCLK/4 and PCLK/16 polled and at PCLK/4 on the
pump all complete - a lost frame never completes a run, the count never
arriving - with OVR never standing and both FIFOs empty after each, 16
to 55 stretched ticks landing in each batch. The same letter judges
every shape of the two loops by the FIFO levels, the flag and the count
of completions after it. The handlers of the bench's image, for the
record: the tick's 52 instructions (~80 cycles), the console's receive
path 87, the stamped SPI vector ~370..400 cycles.

**THE RATE ON THE WIRE, TIMED WITH NO WIRE.** A client - the self-link's
SPI2 or a peer board - is clocked by the host's SCK and cannot tell a
wrong divisor: every byte lands at whatever rate BR made. So letter `s`
times the host's frame against its SCK periods at the rate ASKED - PCLK
over the divisor the request named - from the difference of two blocks
of 2048 and 256 frames (a transaction's fixed cost cancels), neither
buffer named, on the ticker's cycle count, the best of three of each,
judged -1 % to +3 %. 35.5.9 keeps the clock continuous while TXFIFO
holds a frame ("provided continuously by the master until TXFIFO becomes
empty"), so a frame inside a block carries no gap and the frame is its
SCK periods alone. SCK is the host's whatever is wired, so the reading
stands for letters `d`, `i`, `l` and `p`, which set the same divisors
over wires this desk does not carry.

| run | STM32G0B1RE | STM32G071RB |
|---|---|---|
| the engines, 8-bit frames, PCLK/2 .. PCLK/256 | 999 1000 1000 1000 1000 1000 999 1000 | 999 1000 1000 999 1000 1000 1000 1000 |
| the polled transmit-only loop, PCLK/2 .. PCLK/256 | 1000 1000 1000 1000 1000 999 999 1000 | 999 1000 1000 1000 999 1000 1000 1000 |
| the engines, 16-bit frames, PCLK/2 and PCLK/16 | 999, 999 | 999, 999 |
| a 1 MHz ceiling on the engines at a 16 and a 2 MHz core | 1000, 1000 | 1000, 1000 |

Every code is its wire, and the polled transmit-only loop too, PCLK/2
included: two byte frames packed into each half-word store on TXE, which
stands while TXFIFO is at most half full (35.5.9), outrun sixteen cycles
a frame. At the dynamic clock's other rungs the stated ceiling is 1 MHz
on the wire, PCLK/16 at 16 MHz and PCLK/2 at 2 MHz - the code re-resolved
and the rate the one asked.

**35.9.2's LDMA_TX, measured**: three 16-bit DMA accesses to an 8-bit
frame size carry SIX frames with the bit clear and FIVE with it set, the
odd count told to the silicon being what stops the dummy half of the
last access reaching the wire.

**The kernel letter.** Four transactions queued from one dispatch come
back in order, every one `spi_ok`, with the bytes on the wire exactly
what each request lent; a request past the arbiter's pending depth is
answered `bus_rejected` immediately; an idle bus votes ok on a
`PrepareSleep` and a busy one votes against it; and **the per-bus
timeout answers a completion that never came with `spi_timeout` and
`recover()` puts the engine back** - the very next four transactions run
to `spi_ok` on the same bus AO. The staging is the honest one for a bus
whose host clocks itself: the ISR body runs and acknowledges the frames
but the `TransferDone` is never posted, which is the lost interrupt the
timeout was written for.

**I2S, on three of the same four wires.** I2S1 master transmitter to
I2S2 slave receiver, sixteen 16-bit samples value for value, with CHSIDE
alternating left/right frame by frame. 35.7.4's formula TIMED: I2SDIV 21
with ODD clear predicts 47619 Hz and the eight stereo frames really took
44172..45133 Hz of wall. **MCKOE is measured by what it does to the
rate**, not by a counter: the same divider with the master clock out
divides by 256 instead of 32, an eightfold drop (5952 Hz predicted, 5891
measured) - and the link still carries every sample with MCK sitting on
the MISO wire, PB4 being I2S1_MCK and PC2 I2S2_MCK, the pad pair this
self-link already has. The three STEREO standards - Philips,
MSB-justified, LSB-justified - each carry eight samples value-exact; all
three data lengths do in a 32-bit channel frame, with the 24-bit
length's second packet judged on the eight bits that are really on the
wire because the other eight are padding the receiver zeroes.
**35.7.8's UNDERRUN on silicon**: a slave transmitter whose data
register was never written raises UDR when the master's clock asks for a
sample it has not got - the one error SPI mode has no counterpart for -
**and the flag is gone by the next read**, 35.7.8's own clearing rule
being a trap with teeth: a poll loop that ASKS whether UDR is set is the
thing that clears it.

**The dynamic clock.** With a stated 1 MHz SCK ceiling the BR code is
re-resolved at every rung - PCLK/64 at 64 MHz, PCLK/16 at 16 MHz, PCLK/2
at 2 MHz, three different registers for one stated SCK - and the link is
byte-exact at every one, in both directions.

**Table 205's last row, on silicon**: an SPI interrupt returns a core
from a WFI in Sleep mode, eight rounds of eight and the FIRST wake in
all eight (the kernel tick being the other candidate, 1 ms against the
frame's 32 us).

### The peer link (letters n..r)

The same four SPI1 pads reach a PEER BOARD running `spi_peer` when the
jumpers go there instead of to SPI2, and the topologies EXCLUDE each
other: the suite probes for the self-link at boot and letters b..l skip
themselves when it is absent. The desk pairs this board with an
STM32F446RE running the stm32f4 `spi_peer` on its SPI2 (PB13 SCK, PB14
MISO, PB15 MOSI, PB12 NSS at AF5, wired by function), and against it `z`
scores **74 of 74** (letters `a` and `m` and `s` wireless, letters
`n`..`r` and `u`..`w` on the wire). The same board measures this one's `spi_peer`
from its own SPI2 suite (`test_stm32f4_spi`, docs/stm32f4/spi.md), and
those figures close this section.

**One wire format, every architecture.** `avrxt/src/apps/spi_link.hpp`
is compiled by every port of `spi_peer` and this suite speaks it
unchanged: the
command channel runs at PCLK/256 = 250 kHz, one frame per chip-select
window, ten of ten, and `ident` names the peer's unique ID and its
`spi_peer` sanity byte (firmware 0x0402). The protocol owns the chip
select here (the Request carries a null `PinRef` and the letter frames
the window by hand with 30 us of hold on each side), and `prime()` is
what makes that legal - a mode change inside an open select window is
one extra edge for the selected client.

**Four modes, both bit orders, byte-exact between two different
silicons**, and a DORD mismatch is an EXACT two-way bit reversal with
both ends checking the other's bytes reversed - zero mismatches either
way. The bit order is a BUS-level verb on this target
(`SpiHost::bit_order()`, CR1.LSBFIRST), and the LSb-first exchange rides
that verb.

**THE HOST'S DATA PAD IS AN AGGRESSOR ON THE CLOCK AT THE FAR END, AND
THE RISING-EDGE MODES ARE ITS VICTIMS.** With MOSI at the very-high
class `SpiHost`'s default gives it, letter `x` (the slip statistics,
outside `z`) slipped **9 and 10 bursts of 40** in two runs, every one in
mode 0 or mode 3 - the two that SAMPLE on the rising edge, the host's
MOSI changing while SCK is LOW - and always the same way: both ends
disagreeing from the same frame on, the rest of the stream one bit out
of place, one clock edge too many counted by the client's shifter. At
the command rate itself letter `p`'s first rung failed in three runs of
four.
With the pad ONE NOTCH DOWN, at `high`, the slip is gone - **0 of 40**
in two runs, and 0 of 40 at `medium` and at `low` too - and the clock pad keeps its
class, being the edge the client samples on. So every peer letter's
init is handed MOSI at `high` (`peer_link_config`); the self-link
letters and letter `s` keep the driver's very-high. x is 0 of 40, and
letter p ten runs of ten exact at every judged rung. The BACK-TO-BACK
rung - letter `x`'s second half, thirty bursts of 32 frames at PCLK/4 =
16 MHz through the polled host, whose TXFIFO runs the frames without a
gap, the rung where the far F446 host's own MOSI class showed - measured
**0 of 60** at `very_high`, `medium` and `high` in two runs and 1 of 60
at `low` (the client wrong from frame 3): the link keeps `high`.

**AND THE PEER'S OWN ANSWER LINE IS THE OTHER AGGRESSOR**, walked with
the stm32f4 peer's console (its MISO class) and this end's MOSI at
`high`, ten bursts a mode: at `very_high` 12 of 40 (modes 0, 1 and 3),
at `high` 7 of 40 and at `medium` 12 of 40 (all in modes 1 and 2, the
falling-edge modes), at `low` 0 of 40 - the class that peer names in its
client's `Config`.

**THE LADDER CLIMBS TO WHERE THE PEER'S CLIENT SPECIFICATION ENDS.** The
F446 peer serves every window as one call of its driver's run
(`SpiClient::answer`/`exchange`, written on TXE) and an exchange on its
DMA1 streams by default (its report's `serve=dma`), the run with
`spilink::spare_polled_pump`: both are byte-exact in both directions at
every code from PCLK/256 to PCLK/4 = 16 MHz, twelve runs of twelve,
where the RXNE reload that peer ran before broke at 1 MHz (500 kHz
exact, the floor verdict failing). The top rung, PCLK/2 = 32 MHz, is
OUTSIDE that client's specification - DS10693 table 63 gives an F446
slave 22.5 MHz, its tv(SO) and this host's setup inside half a clock -
so letter p asks the peer's ident and prints a rung above the ceiling
its kind names without judging it (the stm32f4 peer 22.5 MHz, the
stm32g0 one the 32 MHz of DS13560 table 76; a kind whose data sheet the
suite has not read is judged at every rung). There the answer path
fails on both serves: the DMA serve hears all eight characters exact
and this end reads all eight wrong, in twelve runs of twelve; the run
loses one. It is a figure, not a fault to fix. Longer windows find
PCLK/4 itself marginal on this fitting: thirty-two frames on the
engines, or sixteen 16-bit frames on any path, slip in up to one window
in ten of a run, with nothing else running as well as under a
preempting kernel ([platform.md](platform.md), "The drivers under the
preemptive kernel"); no shape slipped at PCLK/8.

**The arbiter over a SECOND CHIP.** Letter `q` runs `SpiBus`
(= `BusMaster`) over `SpiHost` with the F446 answering: four
transactions queued from one dispatch come back in order, all `spi_ok`,
and the 24 bytes on the wire are exactly what each request lent - read
back and reported by the other board, not by a second peripheral of
this one. The four share ONE select window because the peer's
`exchange` is one burst; the rejection past the pending depth, both
sleep votes and the per-bus timeout with `recover()` follow, and the
next four transactions run to `spi_ok` on the same bus AO.
`util/spi_bus.hpp`, `util/bus_master.hpp` and `kernel/` are realized
here exactly as they are written.

**THE VECTOR IS THE HAZARD HERE, NOT THE WIRE**: collecting the peer's
report is itself a command, and the command path re-inits the host -
which hands SPI1's vector back to the bare pump. The arbiter's claim on
that vector has to be RE-STATED afterwards or every later completion is
consumed by the wrong branch and the arbiter answers `spi_timeout` on
transactions that ran perfectly
(measured: four of them, at exactly the arbiter's limit each, with the
engine's registers proven healthy).

**THE ROLES INVERT ON THE PADS THIS BOARD HOSTS WITH.** Letter `r` makes
SPI1 a `SpiClient` on the very four pins it hosts with - MISO driven,
SCK, MOSI and NSS taken as inputs, the peer's GPIO-driven select the
hardware NSS - and serves the burst with ONE CALL of `exchange()`, while
the peer becomes the bus host for twelve frames. The burst's rate is the
instrument's own: the command carries a DIVISION of the peer's clock, so
the same 32 is 750 kHz from an AVR, 1.5 MHz from a SAM, 2 MHz from a G0
and **1.406 MHz from the F446** (its PCLK1 of 45 MHz). Exact both ways,
in 10 runs of 10 alone and in every run of `z` - with the far host's MOSI
at `SpiHost`'s default very-high and its SCK at MEDIUM: at very-high
this client sees an SCK edge twice and slips a bit, both ends wrong from
that frame on (5 runs of 5), at high in 1 run of 3 ([the STM32F4's SPI
page](../stm32f4/spi.md), "`SpiClient` against the STM32G0B1RE host").
THE SELECT WIRE RUNS APART FROM SCK AND MOSI. The F446 holds the select
low from GPIO for the whole burst, and a select run in the bundle beside
the burst's lines picks up their edges: PA15 sees 4 to 19 edges a burst
where 2 are sent (an EXTI line on the pad in its alternate function,
both edges; the pad's pull-up changes nothing, so the line is driven and
coupled, not floating), and a client that takes NSS from the pad is
DESELECTED MID-FRAME and its bit count starts again at the next edge: it
loses a frame or slips a bit, both ends wrong from one frame on - this
exchange, letter `y`'s single line out and its receive-only run alike,
65 to 79 bursts of 100 - while the client under SSM (letter `y`, below),
which reads the pad only to see the select fall, is exact through the
same glitches. Routed apart, the select counts exactly 2 edges a burst.
The rate is not the remedy: a larger division keeps the same edges.

**16-BIT FRAMES ON ALL THREE OF THIS HOST'S ENGINES, AT THE WIRE'S TIME**
(letter `u`, spi_link's `spare_wide`): thirty-two 16-bit frames a window,
the F446 serving them in half-word beats, byte-exact both ways at every
judged code to 8 MHz from the polled loop, the RXNE pump and the
half-word engines - a frame on the engines a half-word beat, the beat
the frame - and at 16 MHz in seven runs of `z` of ten: in the other
three one of the letter's PCLK/4 verdicts failed (the engines' wire
time, the F446's 16-bit client run, the polled loop's half-words), a
frame lost or slipped there, the fitting's PCLK/4 margin above. Timed from two windows of 16 and 48 frames at the same rung
(the transaction's fixed cost cancels), a frame inside the window takes
998..1000 permille of its sixteen SCK periods on the polled loop and
996..999 on the engines, at PCLK/16 and at PCLK/4; the pump's takes
1148 and 2789, its bus waiting for the handler ("The host above the
wire"). The F446's own 16-bit client run is exact at every judged code.
At 32 MHz, above that client's ceiling, the answers are read wrong in
every frame while the client hears them all - the 8-bit signature.

**THE TRANSMIT-ONLY SHAPES, WITNESSED** (letter `v`, `spare_receive_only`):
33 frames, 8- and 16-bit, through the polled transmit-only loop, the
pump's write shape and the engines' held sink, every one heard exact by
the F446 listening with its answer line dark (its receive DMA, then its
`receive()` run at 7 and 33 frames), at every judged code - and at 32
MHz too, printed: a client that only listens holds above the ceiling
its answers break at. Odd counts both ways, 7 and 33 byte frames, exact
against both of the peer's serves.
ON THE DESK AS IT STANDS the PCLK/4 rung of these windows does not hold
in every run: in ten runs of `z` the letter failed one of its verdicts
in four, the F446 reading the 16 MHz transmit-only windows wrong from a
frame part-way through - the polled loop in two, the pump's write shape
and the F446's `receive()` run once each - every slower code exact; the
open finding below.

**THE CRC AGAINST A FOREIGN UNIT** (letter `w`, `spare_crc`), this end's
unit driven by hand on the resource (`SpiHost`'s Request has no CRC
phase), sixteen data frames a window, CRC-8 0x07 on 8-bit frames and
CRC-16 0x1021 on 16-bit ones: at every judged code this end's TXCRCR is
the bitwise arithmetic of what it sent and its RXCRCR of what came, the
CRC frame it reads is the arithmetic of the F446's stream, CRCERR stands
at neither end, against the F446's streams managing the CRC by
themselves (RM0390 26.4.2) and against its CRC run alike. THE CHECK IS A
REAL ONE: this end's unit off and the CRC frame sent as DATA with its
last bit flipped, the F446 reports CRCERR (`report_crcerr`) in both
widths, on both serves, at PCLK/256 and PCLK/4, every data frame still
exact and its own CRC frame still the arithmetic.

**THE CRCNEXT WINDOW** (letter `t`, outside `z`): with one frame in
flight - each written into an idle shifter - a CRCNEXT store landing
16 to 26 core cycles before the last data frame's RXNE (as a poll of
about ten cycles a turn first sees it) corrupts BOTH calculators: the transmit one shifts once more,
so the CRC frame sent is the right CRC moved up a bit (0xD3 went out as
0xA6), and the receive one is not frozen, the CRC frame summed into it
(RXCRCR 0x00); CRCERR stands at both ends, every data frame exact. A
store before the window is clean. A STORE AFTER THE FRAME HAS ENDED IS
NEVER ACTED ON: no CRC frame is clocked at all - CRCNEXT stays set, TXE
stays low, the far client waits out the select for a frame that never
comes and reports nothing wrong - and letter `t`'s T, R and P stay true
there only because nothing was exchanged (the frame its walk reads in
that region is not the peer's CRC); its F/f column says whether the CRC
frame came at all - f past the frame's end at every code, and at 8-bit
PCLK/2 (a 16-cycle frame) at every point of the walk, one frame in
flight never sending a CRC there. That region is the one RM0444
35.5.14's rule ("The CRCNEXT bit must be set before the end of the last
data frame transaction") excludes; the window before the end is what it
does not, and ES0548 lists nothing. The window is in CORE CYCLES, not
in bits - the same 16 to 26 before RXNE for 8- and 16-bit frames from
PCLK/4 to PCLK/16 - and 6 to 12 cycles wide, and where a store lands
against it moves with the code's placement: the same one-in-flight
procedure, CRCNEXT the store after the last data store, sat inside it at
8-bit PCLK/4 in every window of one image and just before it in another
whose code differs only elsewhere. At 8-bit PCLK/2, a frame of 16 core
cycles, one frame in flight cannot set the bit in time at all. So the
host writes the last TWO data frames as one burst into the TXFIFO and
CRCNEXT right after them: the bit lands while the second-to-last frame
is still in the shifter, a whole frame and more before the window, and
letter `w` holds at every code.

**THE VENDOR'S LOOP MEETS THE SAME WINDOW, AND THE REGION PAST IT.** ST's
HAL v1.4.7 `HAL_SPI_Init` and `HAL_SPI_TransmitReceive` with its CRC,
compiled from the pinned archive at every optimisation level (-O0, -O1,
-O2, -O3, -Os, -Og) into a scratch program with brio's crt, clock and
console around it, hosting on SPI1 against the F446's client: sixteen
data frames a window, six windows a cell, every code from PCLK/256 to
PCLK/4 in both widths. Its loop keeps one frame in flight and sets
CRCNEXT by a read-modify-write of CR1 after the last DR store and the
handle's bookkeeping - some twenty instructions between the two stores
at -Os, about 37 core cycles by the listing, and about 60 at -O0. Built
at -O0 its store lands in the window at 8-bit PCLK/8 in every window,
and at 16-bit PCLK/4 in the images whose placement puts it a few cycles
later, with this suite's signature to the byte for the same streams
(TXCRCR 0x0D where the arithmetic is 0xDA, RXCRCR 0x00, CRCERR at both
ends; the call returns HAL_ERROR with HAL_SPI_ERROR_CRC). Built at any
level, at 8-bit PCLK/4 its store comes after the last data frame has
ended: no CRC frame is clocked, and the call returns HAL_TIMEOUT with
HAL_SPI_ERROR_CRC once its timeout runs out waiting for the CRC's RXNE.
Every other cell is clean at every level (at 16-bit PCLK/256 the
vendor's call, and not this suite's loop, slipped a bit now and then: a
fact of the oracle's loop, not pursued). The same
sequence with its CRCNEXT moved to the store right after the DR store, then walked away
from it a loop turn at a time, is clean before the window, corrupts both
calculators inside it and clocks no CRC frame past it, at the distances
from the DR store this suite's walk finds - so nothing else in the
vendor's sequence (CRCEN cleared and set before SPE, SPE set inside the
call, FRXTH, the TXE test, the tick read every turn) bears on it. The
store's distance from the last frame's end decides, and the window is the
silicon's.

**THE SIMPLEX LINES, THE SOFTWARE SELECT AND `prime()`** (letter `y`,
every window judged at the far end by the F446's `spi_peer`, its SCK and
MOSI at medium; twenty runs of the letter alone, all green):

- A CLIENT WITH NO SELECT PAD (`SpiClient` over pads whose `nss` is
  empty, `SpiNss::software`) is deaf with SSI high: the far board's
  twelve-frame `host_burst` at 1.406 MHz clocked, not one frame arrived,
  the run ended `stalled` on its budget, and none of the twelve frames
  the far host read was an answer (the first 0xFF). Selected by `select(true)` once the program has seen the
  far select fall (PA15 read as a plain input), both streams are
  byte-exact from the first frame and the run ends `complete` on its
  count - the one end a run without a pad has.
- THE HOST'S SINGLE LINE OUT (BIDIMODE + BIDIOE, through the resource:
  `direction()`, TXE-paced writes, `disable()`): 32 frames heard exact by
  the far end's receive-only run at PCLK/64 and PCLK/16, the RXFIFO empty
  after every window - with BIDIOE set the receiver takes nothing.
- THE HOST'S RECEIVE-ONLY CLOCK (RXONLY): sixteen frames of the far
  answer run read exact at both rates; `disable_receive_only()` at the
  sixteenth RXNE stops the clock inside the seventeenth frame (the far
  end counts 18 answers loaded: the 17 frames clocked and its one ahead)
  and BSY falls.
- THE CLIENT'S SINGLE LINE OUT (BIDIMODE + BIDIOE on MISO, `answer()`):
  twelve answers read exact by the far host from the first frame, the
  run `closed` on the select. THE CLIENT'S RECEIVE-ONLY RUN (RXONLY,
  `drive_output = false`, `receive()`): twelve frames exact, `complete`.
  Both take NSS from the pad, so both are deselected mid-frame by the
  select glitches the far host's MOSI at very-high puts on that wire, as
  letter `r`'s exchange is ("THE ROLES INVERT", above), while the SSM
  client of the first item is not.
- `prime()`, STAGED BY ITS ABSENCE over every ordered pair of modes (the
  host left at the first with the select high, the select then falling
  before `start()` applies the second): a pair that keeps CPOL moves no
  edge; a pair that FLIPS it and lands in CPHA = 0 (modes 0 and 2) slips
  EXACTLY ONE BIT at both ends from the first frame - the F446 client's
  frames one place right, this host's read of its answers one place
  early - in four pairs of four; the same flip landing in CPHA = 1
  (modes 1 and 3) is not counted by this client at all, four of four
  exact. Which flips a client counts is the client's silicon (the SAM
  C21's slips into modes 2 and 3 from mode 0, docs/samc21/spi.md), so
  the verb is for every caller that frames the select by hand. Primed,
  the four slipping pairs are exact.

**THE G0 `spi_peer` MEASURED FROM THE F446** (`test_stm32f4_spi`, SPI2
at PCLK1 = 45 MHz hosting, its SCK and its MOSI at medium): the
exchange on the peer's DMA serve byte-exact both ways at every code to
PCLK1/2 = 22.5 MHz, from the polled host (eight frames) and from the
F446's DMA1 engines (thirty-two), and the peer's polled run to 22.5 MHz
as well. ITS ANSWER LINE IS AN AGGRESSOR IN THE FALLING-EDGE MODES, the
slip instrument (that suite's letter x, ten bursts a mode at the command
rate) walking the peer's MISO class: 11 of 40 at very-high, 8 and 11 of
40 at high, every one in mode 1 or 2 with both ends wrong from the same
frame - the edge landing on an SCK just settled high - and 0 of 40 at
medium and at low. AND THE TOP RUNG ASKS FOR THE FAST EDGE in the
rising-edge modes: at 22.5 MHz in mode 0 (twenty windows of 32 frames
on that board's polled host and on its streams, a diagnostic run once
per class; letter `u`'s top rung judges the class chosen) the answer
line at very-high is exact in 8-bit and 16-bit frames alike; at high
every 8-bit window holds and 16 and 20 of 20 16-bit ones read wrong,
always in bit 2 of the frame and, on the streams, bit 15 - the data
valid time against the far host's setup - while the peer hears
everything; at medium every window is wrong in both widths, at low every
bit. So the peer drives MISO at very-high in modes 0 and 3 and at medium
in modes 1 and 2 (its header), and that suite's x is 0 of 40 and its
back-to-back rung 0 of 30.

THE EXTENDED EXCHANGE FROM THAT SIDE (its letters `u`, `v`, `w`, every
code to 22.5 MHz judged): the peer's DMA serve in half-word beats and
its 16-bit run exact both ways - but for that host's streams at 22.5 MHz,
where back-to-back 16-bit frames put this end's MSB past the answer
line's budget (10 of 7200 frames read with bit 15 late, none of 9600
8-bit ones; docs/stm32f4/spi.md, letter `u`), judged there to 11.25 MHz; its `receive()` run and its receive
channel hearing every frame of the F446's transmit-only shapes at 7 and
33 frames in both widths, the odd last frame under the quarter
threshold; the CRC through 35.5.14's "CRC transfer managed by DMA" (the
transmit channel counting the data frames, the receive channel the
host's CRC frame too, no CRCNEXT written) and through the peer's CRC run
(CRCNEXT after its last FIFO store, a frame an access) both clean in
both widths - and THIS UNIT CATCHES A CORRUPTED WORD: the F446's CRC
frame sent as data with its last bit flipped raises CRCERR here
(`report_crcerr`) on both serves, both widths, at PCLK1/256 and PCLK1/4
and on the engines at PCLK1/2, every data frame exact. The peer's CRC
run reads a frame an access: at 22.5 MHz it is printed, not judged - its
8-bit frames are 22.7 core cycles each, and it heard all sixteen in three
runs of four.

## On the STM32G071RB

`test_stm32_spi` builds for the Nucleo-G071RB (DEV_ID 0x460, REV_ID
0x2000); no board is paired with it on the desk, so its peer letters have
no partner to run against (the gap list), and its self-link letters skip
after the probe as they do on the G0B1RE.

**THERE ARE TWO INSTANCES AND NOT THREE.** `spi_present(3)` is false, so
`Spi<3>` does not compile there; SPI2's line is SPI2_IRQn rather than
SPI2_3_IRQn (the reserve deriving the name from SPI3's presence, and
`BRIO_STM32G0_SPI2_HANDLER` binding whichever it is), and **the I2S is
SPI1's alone** - `IS_I2S_ALL_INSTANCE` names one instance on this header
where the G0B1's names two. The suite's roster and I2S-row verdicts are
written as the reserve's own statements and hold both ways; the I2S
letter itself is a self-link letter and skips on this desk.

ES0418's SPI items are **2.14.1** and **2.14.2**, word for word the
G0B1's 2.12.1 and 2.12.2. Both are staged by letter `h`, which is a
SELF-LINK letter: NOT STAGED on this board, and said so rather than
assumed from the other die.

## On the STM32G031K8

`test_stm32_spi` builds for the Nucleo-G031K8 (DEV_ID 0x466, REV_ID
0x1003), and so does `spi_peer`; no board is paired with it on the desk,
so its peer letters have no partner to run against (the gap list). What
it cannot have at any wiring is a SELF-link: SPI2's four pads
(PB10/PC2/PD4/PB12) are bonded to no pin of the LQFP32 (DS12992 table
12), so those eleven letters are COMPILED OUT rather than skipped at run
time - each printing its own reason and claiming nothing - and there is
no probe to run either, which keeps the image at 31104 bytes inside a
64 KB part. THE PEER LETTERS ARE COMPILED EVERYWHERE, because SPI1's own
four pads are bonded on every package this stratum has met: what settles
them is the desk, and an absent peer with the wires in place fails
loudly.

What the census still proves on this die: the reserve's SPI roster and
the shared-vector derivation (SPI2's line is SPI1's no more, and it is
shared with a third instance exactly where the part has one); table 209's
reset values to the bit, the misaligned reset DS/FRXTH pair, 35.9.2's
forced DS code; the mode fault a non-alternate-function NSS pad raises;
**that the silicon enforces none of 35.5.7 - all seventeen CR1/CR2 fields
take a write with SPE set** - and that every configuring verb refuses
anyway, with SSI, CRCNEXT and FRXTH open; 35.5.9's disable procedure; and
`spi_rate_for()` at all three rates of the ladder.

The console this suite runs on is **LPUART1** on PA2/PA3 at AF6, for the
reason [clock.md](clock.md) gives: its subject moves the clock and this
part's USART2 has no kernel-clock multiplexer to keep the report out of
it.

## Not covered yet

**Driver gaps** - implemented nowhere, and a program that wants them
must reach the registers itself:

- **no half-duplex or receive-only TASK.** `SpiDirection` and its verbs
  are on the resource and `spi_config_valid()` knows the rules, but
  `SpiHost`'s Request is full-duplex only: a bidirectional device wants
  a direction turn in the middle of a transaction, which is a second
  Request shape and belongs with its first real user.
- **no DMA engine slots on `SpiClient`.** Left for a device-shaped
  user; the bench drives the client's channels raw, which is what a
  peer does anyway.
- **no CRC phase in `SpiHost`'s Request or in `SpiClient`'s runs.** The
  unit is the resource's (`crc`, `crc_polynomial`, `crc_next`, the two
  accumulators, CRCERR), measured on the wire by hand from both ends
  (letters `w` and `t`, the stm32g0 `spi_peer`); a CRC phase in the
  tasks - the last two frames in one burst and CRCNEXT after them, or
  the engines' own "CRC transfer managed by DMA" - is born with the first
  device that checksums its frames.
- **no check that a pad can take the class asked.** RM0444 7.5.3's
  FT_c pads take very low and low alone, and which pad is one is the
  data sheet's pin table, which no header of the pack carries (on the
  G0B1, PD0 - SPI2_NSS - and PD2): `claim_nss_pad()` on PD0 asks it for
  very-high. Born with the device tables' pin-level bonding
  (design/overview.md's open item).
- **a wide frame on byte engines, or from an odd address, runs on the
  frame pump.** Engines of `uint8_t` cannot carry a 9..16-bit frame and
  a half-word beat cannot read a buffer that is not half-word aligned
  (10.4.3's note), so `dma_serves()` sends such a request to the pump
  before the select; the pump is correct at a frame an interrupt.
- **packed 8-bit DMA is a RESOURCE fact and not the task's.** LDMA_TX
  and LDMA_RX are verbs and the rule is measured, but the task never
  packs: packing needs the caller's own buffer layout.
- **`SpiClient` has no address-recognition or wake feature** because
  this peripheral has none - the SAM's `FORM`/`ADDR` and its SSDE have
  no counterpart here. Table 205's "wake-up capability from Low-power
  Sleep" is the plain interrupt, measured.
- **the I2S receiver's shutdown sequence is the caller's.** 35.7.5 gives
  three sequences keyed by DATLEN, CHLEN and I2SSTD, each of them "wait
  for the (second to) last RXNE, then wait N I2S clock cycles using a
  software loop" - and how long an I2S clock cycle is depends on a
  prescaler the slave side does not own. `i2s_disable()` clears I2SE and
  says so; a driver that guessed would be stating a fact it cannot
  enforce.
- **no I2S task.** The chapter's own DMA note (35.7.9: "the DMA works in
  exactly the same way as it does in SPI mode") makes a streaming I2S a
  `util/block_stream.hpp` user, and that is born with its first real
  audio device.

**Implemented but not bench-verified** - the code is there and compiles
on every header of the pack, but no silicon has run it:

- **an engined host's polled transaction under `InRam`** (1612 bytes):
  `bench_stm32_ram` places the engined host, and its letter d runs the
  engines alone; a polled request below `dma_min_frames` on that host is
  what would measure it, the engineless host's figures above being the
  same loops.

- **the transmit-only loop's tail against a select or a D/C the TASK
  moves** (FTLVL then BSY before the edge): the peer letters frame their
  windows by hand with 30 us of hold on each side, so the tail's timing
  is never pressed - the loop, the packed receive with its odd last
  frame and the pump's write-ahead are judged on the wire by the F446
  (letters `u` and `v`), and by the FIFO levels, OVR and the count of
  completions with nothing on MISO (letter `s`, `bench_stm32` letter
  `e`). A device on the wire with a D/C line, or the self-link's letters
  `b`..`d`, would measure it.
- **The peer letters on the STM32G071RB and the STM32G031K8**: no board
  is paired with either on the desk; a partner running `spi_peer` would
  run them.

- **SPI3, on any board.** It exists only on the G0B1/G0C1 and its pads
  (PC10/PC11/PC12 at AF4) are not wired on this desk; the family fixture
  instantiates it, `spi_irq(3)` is checked against the header, and its
  DMAMUX pair is stated from table 56.
- **the single line RECEIVING (BIDIMODE, BIDIOE clear), in either
  role.** A host receiving on its MOSI pad needs the far end to drive
  that pad, and a client receiving on MISO needs the far host to drive
  its own input: the desk partner's MOSI is an input and its MISO the
  host's input on every `spi_link` op. A three-wire device on the bus
  (a sensor with one SDIO line) would measure it; the single line OUT is
  measured both ways (letter `y`).
- **the I2S asynchronous start (ASTRTEN)** and the PCM long-frame
  synchronization: written and read back, never staged.
- **the self-link letters on the STM32G071RB and the STM32G031K8**:
  ES0418 2.14.1 and 2.14.2 (the two BSY items), the frame matrix, the
  eight BR codes host-to-host, NSS four ways, the CRC, TI mode, the OVR
  and the whole I2S personality (SPI1's alone on the G071) - the G071RB's
  desk carries the peer link on those pads and the LQFP32 bonds no SPI2
  (above). All of them are measured on the STM32G0B1RE.

**Findings left open, with what would settle them:**

- **letters `u`'s and `v`'s PCLK/4 rung on the desk as it stands**
  (above): a frame lost or slipped at 16 MHz with this end hosting, in
  four runs of `z` of ten. Letter `x`'s walk of the
  MOSI pad's class against the F446's listening client at 16 MHz, an
  edge count on the F446's select pad as the one above, and a look at
  the four wires would say whether this end's edges reach the far
  select as the far host's reach this one's.

- **the CRCNEXT window's cause** (letter `t`): a store 16 to 26 core
  cycles before the last data frame's RXNE corrupts both calculators,
  in core cycles at every rate measured, where RM0444 asks only that the
  bit be set before the frame's end. The vendor's own polled loop meets
  it on this board, with this suite's signature (the vendor's loop,
  above), so the window is the silicon's and no sequence of ours makes
  it; what the block does there - when it samples CRCNEXT against the
  last frame's end, and why a store a few cycles early shifts the
  transmit calculator once more and leaves the receive one running - is
  ST's to describe, and that description would settle it.

- **the burst ladder's 320-cycle floor** (letter d) is not the host
  loop's own - the transmit-only loop holds PCLK/2's clock continuous
  with no client to serve (letter s) - so it divides between the hand
  loop's polls and loads and the client's per-frame vector on the same
  core, in proportions not measured: the self-link's four wires and the
  same burst with the client on its raw engines (letter i's arrangement)
  would split them.

- **the packed receive at PCLK/2 runs at 25.3 cycles a frame against the
  wire's 16** (x 1.58 on 256 frames; 24.2 and x 1.51 from SRAM, so the
  fetch is not what holds it): a loop turn of some 26 instructions with
  four taken branches and three APB accesses per pair. What would move
  it: the pairs-only loop unrolled by two. At PCLK/4 it is the wire's
  within a twentieth. The write shape, which is what a display's pixel
  path runs, is on the wire's time.
- **a 3-byte polled request costs 351 cycles above the wire** (319 from
  SRAM): the breakdown above names the silicon's own latencies at the
  four phase ends (~50), ~180 instructions from the flash and the 32
  cycles the flattened transaction costs a short request. A ceiling
  kept as one byte instead of an optional, and the budget's arithmetic
  hoisted, are the next ten cycles; the rest is the shape of two phases
  around a D/C pin.
- **the pump's continuity above PCLK/16** is the handler's, not the
  pump's: three frames in flight are 96 cycles at PCLK/4 and no vector
  of this core refills inside them, so the bus idles between refills;
  the engines serve that rate and `dma_min_frames` is where they take
  over.

**Implemented, and the measurement declined with the reason:**

- **the answer-turnaround ceiling of the ISR-driven client** (letter d):
  a ceiling needs a clock nothing gaps, and the burst loop reading every
  frame back beside a client answering on the same core gaps it at every
  rung, so what that ladder measures is the pair's own pace. Measured
  where it can be - both ends on DMA, letter i.
- **the TI frame-format error** did not reproduce with the one staging a
  single board affords (a frame-size mismatch); printed, not claimed.
- **ES0548 2.12.2** is a sporadic coincidence of two clocks and 32
  rounds cannot disprove it; the honest witness it names is measured
  instead.
- **the I2S master clock's own frequency**: MCK is 256 x Fs (about 12
  MHz here) and no free pad on this desk carries a counter for it. Its
  effect on the sampling rate is measured instead, which is the same
  bit under a different question.
- **PCM's frame count**: with I2SSTD = 11 the WS line is a frame
  synchronization pulse and not a channel side, so the slave collects
  one sample fewer in the same window than the stereo standards do.
  Closing that would need a receiver that counts PCM frames rather than
  half-words; the count is printed and the samples that arrive are
  value-exact.
