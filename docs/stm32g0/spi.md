# SPI / I2S (STM32G0)

Documents of record: RM0444 Rev 6 - SPI/I2S ch. 35 (35.4's table 205 is
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
Bench suite: `test_stm32_spi`, and it carries TWO INSTRUMENTS on one set
of pads because a desk carries either (the suite probes for the wire at
boot; its header comment is the map):
letters `a`..`m` (89 verdicts) run on the Nucleo's own SPI1-to-SPI2
self-link, letter `s` (18 verdicts) on no wire at all - the host's two
loops in every shape, judged by what the block holds after each, the
receive shapes under a stretched tick handler, and the rate on the wire
at every BR code -,
letters `n`..`r` (20 verdicts) on the link to a PEER BOARD
running `spi_peer`, commanded in band over
`avrdx/src/apps/spi_link.hpp`. The peer may be any of the three ports of
that app - the AVR's, the SAM C21's, this stratum's own, which answers
on SPI1 AF0 at the same pin names this board hosts on, or the STM32F4's,
on the F446's SPI2 - and the peer's `ident` names which one answered
(firmware 0x01xx, 0x02xx, 0x03xx, 0x04xx). The
two wirings exclude each other, the suite PROBES which one is fitted
before any letter runs, and the letters whose instrument is absent skip
themselves with the reason printed and no verdict claimed. Family
fixture `test/family_stm32g0/spi.cpp` plus sixteen negatives under
`brio check stm32g0`.

## What the silicon does

Three instances of one block on the G0B1/G0C1 and two everywhere else,
each of which is an SPI **or** an I2S depending on one bit
(`I2SCFGR.I2SMOD`), with 32-bit transmit and receive FIFOs, 4-to-16-bit
frames, a hardware CRC and the enhanced NSSP and TI modes. Table 205 is
the roster and it has exactly two entries that move across the family:
**SPI3 exists only on the G0B1/G0C1, and SPI2 carries an I2S only
there** - one condition, so the reserve derives both from `SPI3_BASE`
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

`SpiHost<n, pins, TxEngine, RxEngine>` - the transfer engine
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
`init(clock, max_sck_hz = 0)`, `start(req)` (true = completed inside
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
the wire, 1060 by letter d, and `pump_cycles_per_frame`, the pump's busy
per frame, 137 by letter e; a shorter data phase takes the pump even
with engines bound).

`LEN IS A FRAME COUNT.` With the default 8-bit frames a frame is a byte
and `len` is a byte count. With `bits` above eight a frame occupies TWO
bytes of the caller's buffer, low byte first, and `len` still counts
frames. Below eight bits a frame is still one byte, right-aligned, with
the unused top bits ignored on the way out and read back as zero.

`SpiClient<n, pins>` - the other end: the polled RUNS and, as their
degenerate case, the per-frame verbs. `init(clock, Config)`, `enable()`
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
- **CRC, TI frames, NSS pulse** - not in the runs.

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
twelve frames at 1.406 MHz) the exchange run is exact both ways.

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
SPI1 pads: 38 verdicts, measured against an STM32F446RE and against a
second STM32G0.

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
| spi.dma (8-bit) | 16 | 2 | 1316 | 1015 | 1 | 234 | 256 | 1060 |
| spi.dma (8-bit) | 256 | 2 | 5156 | 1015 | 1 | 234 | 4096 | 1060 |
| spi.dma (8-bit) | 16 | 8 | 2081 | 1016 | 1 | 234 | 1024 | 1057 |
| spi.dma (8-bit) | 256 | 8 | 17441 | 1016 | 1 | 234 | 16384 | 1057 |
| spi.dma.w16 | 32 | 2 | 1576 | 1019 | 1 | 234 | 512 | 1064 |
| spi.dma.w16 | 512 | 2 | 9256 | 1019 | 1 | 234 | 8192 | 1064 |
| spi.dma.w16 | 32 | 8 | 3109 | 1018 | 1 | 234 | 2048 | 1061 |
| spi.dma.w16 | 512 | 8 | 33829 | 1018 | 1 | 234 | 32768 | 1061 |

Every frame beyond the first costs exactly the wire's time - 16.0 HCLK
cycles a byte at PCLK/2, 64.0 at PCLK/8, either width - so the engines
keep the FIFO fed at the top rate and the per-byte time is the wire's.
What a transaction adds is a constant some 1060 cycles: `start()`
itself (471 measured alone, a ruler read of ~62 in it: the compare of
the settings, the select, the engines' five stores each), ONE completion
interrupt whose handler is 234 cycles with the meter's stamps, and the
idle path's wake with the bench's own idle-window reads. That constant
is `dma_fixed_cycles`, the numerator of `dma_min_frames`.

**The host above the wire** (`bench_stm32` letter `e`: the engineless
`SpiHost<1>` on the same pads, MISO floating, each op the best of 8 runs
of `start()` alone with the Request built outside the clock; the write
shape reads nothing back, the `.rx` one reads every frame into a
buffer; the levels and OVR read after every run were zero and clear on
every line). The polled loops and the pump, before and after this
round, 256 frames:

| op | PCLK/ | before: wall, x | after: wall, x | the vendor's loop on the same board |
|---|---|---|---|---|
| spi.poll (8-bit write) | 4 | 60760, 7.41 | 8554, 1.04 | 43368, 5.29 |
| spi.poll | 16 | 83791, 2.55 | 33124, 1.01 | 58732, 1.79 |
| spi.poll.rx (8-bit read) | 4 | 60760, 7.41 | 9609, 1.17 | 43368, 5.29 |
| spi.poll.rx | 16 | 83791, 2.55 | 33165, 1.01 | 58732, 1.79 |
| spi.poll16 (16-bit write) | 4 | 68411, 4.17 | 16701, 1.01 | 45395, 2.77 |
| spi.poll16 | 16 | 117002, 1.78 | 65844, 1.00 | 96848, 1.47 |
| spi.poll16.rx | 4 | 68411, 4.17 | 16701, 1.01 | 45395, 2.77 |
| spi.poll16.rx | 16 | 117002, 1.78 | 65832, 1.00 | 96848, 1.47 |
| spi.pump (8-bit, interrupts per frame; busy per frame) | 4 | 13.32 (1.00; 318) | 5.31 (0.34; 170) | - |
| spi.pump | 16 | 3.74 (1.00; 636) | 1.34 (0.34; 134) | - |
| spi.pump.rx | 4 | 13.32 (1.00; 318) | 5.40 (0.34; 173) | - |
| spi.pump.rx | 16 | 3.74 (1.00; 636) | 1.36 (0.34; 137) | - |
| spi.pump16 | 4 | 6.61 (1.00; 315) | 3.58 (0.50; 229) | - |
| spi.pump16 | 16 | 2.45 (1.00; 464) | 1.30 (0.50; 249) | - |

Before, the polled loop wrote a frame, spun on RXNE and read it back
through two out-of-line verbs - 237 cycles a frame whatever the rate -
and the pump took one interrupt a frame at 315 cycles each with the
bench's stamps. After: the write shape is the chapter's transmit-only
loop (frames in on TXE, two byte frames an access, the unread echoes'
overrun cleared at the tail), wire-bound from PCLK/4 at both widths;
the receive shape keeps three frames in flight frame by frame below six
and PACKS above it (two frames an access each way, two pairs in flight,
the odd last one by a byte with FRXTH moved for it and back), 37.5 cycles
a frame against the wire's 32 at PCLK/4 - the one line of the table
above 1.1 - and wire-bound at PCLK/16; the pump drains what RXNE holds
and refills three, so it takes an interrupt every three frames at
PCLK/4 and at PCLK/16 alike (six for sixteen frames, 86 for 256) and
keeps the bus busy where the handler returns within three frame times:
at PCLK/16 that is 384 cycles against the bench's stamped handler of
372..402 (x 1.34), at PCLK/4 96 cycles against a handler no vector can
run in (x 5.3, the bus idle between refills - the engines' rate, not
the pump's). The pump's busy per frame at PCLK/16, 137 cycles with the
stamps inside, is `pump_cycles_per_frame`.

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

| op | n | before: wall, rest | after: wall, rest |
|---|---|---|---|
| spi.req | 1 | 781, 653 | 413, 285 |
| spi.req | 3 | 1382, 998 | 768, 384 |
| spi.req | 16 | 5269, 3221 | 2460, 412 |

"Rest" is wall minus the wire's cycles, the Stopwatch's ~62 inside it;
the 3-byte request's 322 net cycles are: the compare of the settings
and the clamp (~15 instructions), the select and the D/C edges (one
store each, inline), the command phase (one frame in, the tail's wait
on FTLVL and BSY, the overrun's clear and the drain - two reads), the
D/C flip, the data phase (one half-word access for the pair, the same
tail), the select raised, and the silicon's own latencies at each
phase's two ends (the first edge after the write, BSY after the last) -
some 180 instructions and 14 APB accesses from a flash at two wait
states, and nothing called. What the round took out: the 48-byte copy
of the request (12 word moves), `flush_rx()` (18 instructions and a
call; the loops read back what they wrote, so nothing is stale - the
abnormal exits drain), the RXNE interrupt's disarm under a guard (never
armed outside an asynchronous tenure), two `PinRef` calls, and per
frame two out-of-line `data()` calls and a bounded spin of its own.

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
scores **56 of 56** (letters `a` and `m` and `s` wireless, letters
`n`..`r` on the wire). The same board measures this one's `spi_peer`
from its own SPI2 suite (`test_stm32f4_spi`, docs/stm32f4/spi.md), and
those figures close this section.

**One wire format, every architecture.** `avrdx/src/apps/spi_link.hpp`
is compiled by every port of `spi_peer` and this suite speaks it
unchanged: the
command channel runs at PCLK/256 = 250 kHz, one frame per chip-select
window, ten of ten, and `ident` names the peer's unique ID and its
`spi_peer` sanity byte (firmware 0x0401). The protocol owns the chip
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
class `SpiHost::init()` gives it, letter `x` (the slip statistics,
outside `z`) slipped **9 and 10 bursts of 40** in two runs, every one in
mode 0 or mode 3 - the two that SAMPLE on the rising edge, the host's
MOSI changing while SCK is LOW - and always the same way: both ends
disagreeing from the same frame on, the rest of the stream one bit out
of place, one clock edge too many counted by the client's shifter. At
the command rate itself letter `p`'s first rung failed in three runs of
four.
With the pad ONE NOTCH DOWN, at `high`, the slip is gone - **0 of 40**
in two runs, and 0 of 40 at `medium` and at `low` too - and the clock pad keeps its
class, being the edge the client samples on. So the suite re-states the
peer link's MOSI at `high` after every init (`peer_link_pads()`); the
self-link letters and letter `s` keep the driver's very-high.

**AND THE PEER'S OWN ANSWER LINE IS THE OTHER AGGRESSOR**, walked with
the stm32f4 peer's console (its MISO class) and this end's MOSI at
`high`, ten bursts a mode: at `very_high` 12 of 40 (modes 0, 1 and 3),
at `high` 7 of 40 and at `medium` 12 of 40 (all in modes 1 and 2, the
falling-edge modes), at `low` 0 of 40 - the class that peer re-states.

**THE LADDER CLIMBS TO WHERE THE PEER'S CLIENT SPECIFICATION ENDS.** The
F446 peer serves every window as one call of its driver's run
(`SpiClient::answer`/`exchange`, written on TXE) and an exchange on its
DMA1 streams by default (its report's `serve=dma`), the run with
`spilink::spare_polled_pump`: both are byte-exact in both directions at
every code from PCLK/256 to PCLK/4 = 16 MHz, seven runs of seven, where
the RXNE reload that peer ran before broke at 1 MHz (500 kHz exact, the
floor verdict failing). The top rung, PCLK/2 = 32 MHz, is OUTSIDE that
client's specification - DS10693 table 63 gives an F446 slave 22.5 MHz,
its tv(SO) and this host's setup inside half a clock - and there the
answer path fails on both serves: the DMA serve still hears all eight
characters exact (the signature verdict passes), the run loses one. It
is a figure, not a fault to fix.

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
and **1.406 MHz from the F446** (its PCLK1 of 45 MHz). Exact both ways -
AND THE CURE WAS THE HOST'S CLOCK PAD: with the F446 driving SCK at
very-high this client gained or lost one edge at a random frame in every
run (its stream one bit out of place from there, the host's read of the
answers wrong with it - in 12 runs of 12, on this run and on the
per-frame loop it replaced alike, the ground moved onto the two chips'
own pins changing nothing), at high in 2 runs of 4, and at medium in 0
of 14 (0 of 6 on the per-frame loop), at low in 0 of 4. The F446 peer
now hosts at medium.

**THE G0 `spi_peer` MEASURED FROM THE F446** (`test_stm32f4_spi`, SPI2
at PCLK1 = 45 MHz hosting, its SCK at medium and its MOSI at high): the
exchange on the peer's DMA serve byte-exact both ways at every code to
PCLK1/2 = 22.5 MHz, from the polled host (eight frames) and from the
F446's DMA1 engines (thirty-two), and the peer's polled run to 22.5 MHz
as well. ITS ANSWER LINE IS AN AGGRESSOR IN THE FALLING-EDGE MODES, the
slip instrument (that suite's letter x, ten bursts a mode at the command
rate) walking the peer's MISO class: 11 of 40 at very-high, 8 and 11 of
40 at high, every one in mode 1 or 2 with both ends wrong from the same
frame - the edge landing on an SCK just settled high - and 0 of 40 at
medium and at low; but medium costs letter p's 22.5 MHz rung, in mode 0.
So the peer drives MISO at high in modes 0 and 3 and one notch slower,
medium, in modes 1 and 2 (its header), and x is 0 of 40.

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
- **the tasks claim their pads at very-high speed with no knob.**
  `SpiHost` takes SCK and MOSI and `SpiClient::drive_output()` takes
  MISO at `PinSpeed::very_high`, and the bench measured that a client's
  MISO at that speed slips the falling-edge modes on a jumper-wire bus,
  and a host's SCK and MOSI their own modes (above, and the F446's
  suite). The pad speed is the WIRE's electrical question, not the
  peripheral's; today a program that wants another re-states the pad
  after `init()` (the stm32g0 `spi_peer` does, by mode), and a `PinSpeed`
  field on the two tasks' configs is the driver's answer, born with its
  second user.
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

- **the two polled shapes and the pump's write-ahead, judged on the
  wire.** The transmit-only loop's tail (the D/C or the select moved on
  FTLVL then BSY), the packed receive (two frames an access, the odd
  last one under a moved threshold) and the pump's three frames in
  flight are judged by the FIFO levels, OVR and the count of
  completions with nothing on MISO (`test_stm32_spi` letter `s`,
  `bench_stm32` letter `e`); that every byte lands where it belongs is
  the self-link's question - letters `b`, `c` (every frame size, which
  is the packed path's sub-byte frames) and `d` - and that link is not
  on this desk.
- **16-bit frames on the DMA engines, judged on the wire.** Letter `i`
  carries the leg - sixteen half-word frames out and back on the
  half-word engines with one completion, and the same request from an
  odd address on the frame pump - and it needs the self-link, which is
  not on this desk; `bench_stm32`'s letter `d` times the path with MISO
  floating (the table above) and judges no data.
- **`SpiHost`'s engined data phase and the reset in `release()`, on the
  wire.** They ride letter `i`, a self-link letter, and the self-link is
  not on this desk; `test_stm32_dma` letter `r` measures the release on
  SPI1 with no wire ([dma.md](dma.md)). The same channel levels (receive
  at `very_high`, transmit at `high`) on the raw engines of the
  `spi_peer`'s exchange serve are measured on the wire from the F446,
  exact to 22.5 MHz (above).
- **`SpiClient::receive()`, the runs in 9..16-bit frames, a run without
  an NSS pad, and the odd last frame of an exchange under the moved
  threshold**: compiled on every header, none driven - every exchange on
  this desk is eight or twelve frames. A peer op with an odd count, a
  wide-frame window and a client under SSM would measure them.
- **The peer letters on the STM32G071RB and the STM32G031K8**: no board
  is paired with either on the desk; a partner running `spi_peer` would
  run them.

- **SPI3, on any board.** It exists only on the G0B1/G0C1 and its pads
  (PC10/PC11/PC12 at AF4) are not wired on this desk; the family fixture
  instantiates it, `spi_irq(3)` is checked against the header, and its
  DMAMUX pair is stated from table 56.
- **half-duplex (BIDIMODE) and receive-only (RXONLY) on the wire.** The
  register surface is refused and read back correctly by letter a; no
  letter clocks a transfer in either.
- **the I2S asynchronous start (ASTRTEN)** and the PCM long-frame
  synchronization: written and read back, never staged.
- **`SpiHost::prime()`** - the CPOL-flip-before-the-select verb. Its
  reason is a one-bit slip measured on the SAM C21; the same slip has
  not been staged here. What HAS been measured on this target is its
  mirror image at the other end: a peer client that frames on the clock
  alone (SSM/SSI) counts a host's CPOL settling edge and is one bit
  early from the first frame, which is the same fact seen from the
  client's side.
- **the self-link letters on the STM32G071RB and the STM32G031K8**:
  ES0418 2.14.1 and 2.14.2 (the two BSY items), the frame matrix, the
  eight BR codes host-to-host, NSS four ways, the CRC, TI mode, the OVR
  and the whole I2S personality (SPI1's alone on the G071) - the G071RB's
  desk carries the peer link on those pads and the LQFP32 bonds no SPI2
  (above). All of them are measured on the STM32G0B1RE.

**Findings left open, with what would settle them:**

- **the burst ladder's 320-cycle floor** (letter d) is not the host
  loop's own - the transmit-only loop holds PCLK/2's clock continuous
  with no client to serve (letter s) - so it divides between the hand
  loop's polls and loads and the client's per-frame vector on the same
  core, in proportions not measured: the self-link's four wires and the
  same burst with the client on its raw engines (letter i's arrangement)
  would split them.

- **the packed receive at PCLK/4 runs at 37.5 cycles a frame against the
  wire's 32** (x 1.17 on 256 frames): a loop turn of some 26
  instructions with four taken branches and three APB accesses per
  pair, on a core that refetches a branch target from a flash at two
  wait states. What would move it: the pairs-only loop unrolled by two,
  or the loop placed in SRAM - the retrospective's SRAM measurement is
  the SAM's, not this family's. The write shape, which is what a
  display's pixel path runs, is on the wire's time.
- **a 3-byte polled request costs 322 cycles above the wire** against
  the round's 300: the breakdown above names the silicon's own
  latencies at the four phase ends (~50) and ~180 instructions from the
  flash. A ceiling kept as one byte instead of an optional, and the
  budget's arithmetic hoisted, are the next ten cycles; the rest is the
  shape of two phases around a D/C pin.
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
- **CRCERR by a corrupted word**: two peripherals joined by six
  centimetres of wire carry exactly what is put on them, so the
  receiver's EXPECTATION is what the letter corrupts (a different
  polynomial), said plainly rather than dressed up.
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
