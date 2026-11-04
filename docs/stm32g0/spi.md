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
using Bus = brio::SpiHost<1, bus_pins,
                          brio::DmaTxEngine<1, 1, uint16_t>,   // DMA1 channel 1 out
                          brio::DmaRxEngine<1, 2, uint16_t>>;  // DMA1 channel 2 in
using Arb = brio::SpiBus<Bus, P, 4>;
using Cs  = brio::Pin<'A', 15>;

Cs::output(true);                 // deasserted
(void)Bus::init(clock, 8'000'000);   // 8 MHz ceiling for the whole bus

Bus::Request r{};
r.cs      = Cs::ref();
r.cmd     = brio::lend<brio::Lease::reply>(cmd);
r.cmd_len = 2;
r.rx      = brio::lend<brio::Lease::reply>(rx);
r.len     = 8;
r.clock   = brio::SpiClock::div8;
r.mode    = brio::SpiMode::mode0;
r.reply   = brio::reply_to<Device, brio::SpiDone>();
r.engine  = brio::SpiEngine::dma;
brio::post<Arb>(r);               // returns at once; `cmd` and `rx` stay lent

extern "C" void DMA1_Channel1_IRQHandler() {
    if (Bus::dma_isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
}
extern "C" void DMA1_Channel2_3_IRQHandler() {
    if (Bus::dma_isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
}
```

An excerpt: `cmd` and `rx` are the device's buffers, `Device` its active
object, `P` the platform. The request names the DMA engines (its `engine`
is `SpiEngine::dma`), and runs on them at any length: the host names its two
engines and the program binds their channels' vectors to `dma_isr()`, the
completion posted to the bus active object. Here the two command frames run in
the loop inside `start()` and the eight data frames ride the engines after them,
the one fixed rule for a command before data ("The engines", below). The request
runs on the engine it names and on no other: which costs the core less at a
length and a rate is guidance below ("Which engine costs the core less"), and
nothing in the host reads it. The post returns at once whichever engine the
request names; the outcome comes in the reply, `SpiDone`'s field `status`
([design/spi-bus.md, "What follows each code"](../design/spi-bus.md#what-follows-each-code-and-who-acts)
says who acts on each). What decides correctness even
there: the three
pads named with the alternate-function numbers the datasheet gives for
that instance, which nothing in the code can check; the chip select as a
GPIO (the block's hardware select frames the peripheral's lifetime and
not a transaction, a finding below); `init()` after the clock task's;
the engines named, or a request naming them is refused
(`spi_rejected`, nothing on the wire); a bus timeout on `SpiBus` where a channel
that never completes must not leave the bus busy for ever, the default being none.
The request's buffers are lent
until `SpiDone` arrives
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
letters `a`..`m` (90 verdicts) run on the Nucleo's own SPI1-to-SPI2
self-link, letter `s` (15 verdicts) on no wire at all - the host's two
loops in every shape, judged by what the block holds after each, a request
naming the engines refused on the engineless host, the receive shapes under
a stretched tick handler, and the rate on the wire at every BR code - and
letter `E` (1) on no wire but the SCK pad, when a request ends,
letters `n`..`r` (20 verdicts) and `u`..`w` (16) on the link to a PEER
BOARD running `spi_peer`, commanded in band over
`avrxt/src/apps/spi_link.hpp` - `u`..`w` through its EXTENDED EXCHANGE
(16-bit frames, a client that only listens, both ends' CRC units), with
`t` and `x` beside them outside `z`, `K` (12 verdicts) on the same link, the next
request after a stall or a transfer error, `S` (7) on the same link, the engine a
request names, and `B` (3) on the same link, the first block behind polled frames. The peer may be any of the three ports of
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
this file waits on BSY. Every wait of a transaction and of a client run is bounded
and reports; a rate, mode or width change's disable has bounded waits of its own,
outside the transaction's budget, and the host drops their answer.

**The FIFO is the host's write-ahead, and RXNE paces its receive
loop.** TXE means "the transmit FIFO has room", which is true well
before a frame is on the wire and true again immediately after; RXNE
means "a frame has been shifted in", which on a full-duplex bus is
exactly "one frame moved, both ways" - and reading DR is both the
capture and the acknowledgement. The receive loop keeps the TXFIFO's
worth of frames in flight - three byte frames or two half-words, the
capacity 35.5.9 gives the TXFIFO under its own TXE rule - reads one back
on RXNE and writes the next, so the clock runs continuously while the
loop keeps up and the transfer ends when the last frame has come BACK,
which is the only edge that means the wire is idle (the DMA engines end
a block on the same edge, the receive channel's last beat). A phase that reads nothing back
takes the chapter's transmit-only arrangement instead: frames in as TXE
says, the unread echoes left to raise OVR, the tail waiting for FTLVL
and BSY and then clearing the flag by 35.5.11's sequence. No frame in
flight is ever lost to an overrun under any interrupt, because what is
in flight never exceeds the RXFIFO's own capacity. **FRLVL cannot count
three byte frames**: its two bits read 01, 10 and 11 for one, two or
three, and four bytes (measured), so the level is a witness of empty and
full and the loop reads on RXNE, frame by frame.

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
`len`, `reply`, `engine`, and the per-transaction bus configuration)
with the rate as a `SpiClock`, the mode as a `SpiMode` and the frame size
as a `SpiDataSize` defaulting to eight - so an 8-bit request looks
exactly like the other targets'. The ORDER is this family's: the two
pins, the three buffers and the reply first, the lengths and the bytes
last - forty bytes with no padding, and a designated initializer
follows it. A request is read through the reference and copied nowhere
(the arbiter lends it for the call), but for the select and the D/C of one
on the engines, the two pads its completion moves. `init(clock, max_sck_hz = 0)` (the
shared surface) and `init(clock, Config)` - `Config` being
`max_sck_hz`, `sck_speed` and `mosi_speed`, the classes of the two pads
a host drives ("The pads") -, `sck_speed()` and `mosi_speed()` (each
moving its class between transactions), `start(req)` (true = ended
inside start: a polled request, the empty one, a refused one, one whose
command loop stalled; false = on the engines, `dma_isr()` posts
`TransferDone`), `dma_isr()` (the channels'
interrupt body; the host arms no interrupt of the block and binds no
vector of its own), `status()` (`spi_ok`; `spi_rejected`, a request while this
host's own block is on the engines, or one naming the engines on a host with none
or one they cannot carry; `spi_dma_fault`, both channels
put away and the select and the D/C raised in the vector, which does no more, the
peripheral's reset left to the next `start()`; or `spi_stalled` when a polled wait
ran out of its budget, no frame back within it, the select and the D/C raised and
then the peripheral reset and reconfigured, inside `start()`; the next paragraph says what follows both), `prime()`,
`bit_order()`, `recover()`, `rebase(hz)`, `clock_for(hz)`,
`sck_hz(code)`, `release()`. The constants state the engine's numbers:
`has_dma` (whether engines are bound: a request naming them needs them,
`static_assert(Bus::has_dma)`), `ahead_bytes` / `ahead_halfwords` (3 and
2, the frames the receiving polled loop keeps in flight), and
`packed_min_frames` (6, the byte phase from which the polled receive
packs two frames an access). `Placement` is util/placement.hpp's tag,
`InFlash` by default: under `InRam` the polled transaction runs as one
instance in SRAM, `placed_in_ram` says so, and the polled loop's figures
follow it ("The host's placement" below).

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
client answering frame by frame must keep queued ahead of the host's clock), `write`,
`poll`, `selected()` (a live read of the NSS PAD - the peripheral
publishes no such status bit), `drive_output` (the dark listener), the
error accessors, `isr()`, `disable()`, `release()`.

**After `spi_stalled` and `spi_dma_fault` the program calls nothing.** The next request
starts on a clean block whoever calls it, in direct use or behind the arbiter, which goes
on to the next waiting request before the requester has run. The write-ahead makes the
reset the cure and not a courtesy: frames written to the TXFIFO stand there when the block
does not clock, and they would go out in front of the next request. After a stall the reset
runs inside `start()`, the select released first, because this block drives SCK only while
SPE is set: the reset leaves the pad to whatever else holds the wire and the enable after it
drives the pad again, an edge a device still selected would count. After a transfer error the
channels' vector decides and does not work, and until the next `start()` tests its flag
and pays the reset, frames can stand in the FIFOs and be on the wire. A request with a null
`cs` has its select in the program's hands, so the host cannot release it and the reset
happens under it; `prime()` pays an owed reset too, so a program framing the select itself
primes first, as it must for a mode change. `recover()` remains, for the arbiter on
`spi_timeout`, its one use there, and for a program with the bus idle; it waits for nothing,
and answers false when the block did not come back.

**What the reset cannot do: open a closed clock gate.** With the gate closed the next request
stalls too, on its own budget, and `recover()` answers false. The first request after the
gate reopens finds the block at its reset values and stalls as well, its own reset the way
back, unless the program calls `recover()` when it opens the gate (Bench findings).

**The reset, once, cold and out of line.** The stall, the owed reset and `recover()` call one
function in the flash under either placement: the RCC's pulse (RM0444 5.4.11 and 5.4.12), the
applied configuration written back, the enable, under a thousand cycles. It declines
35.5.9's disable procedure: that lets a last frame out, behind a released select nobody wants
it, and with frames standing and SPE down it waits out its 200 000 polls on FTLVL, 28.4 ms,
and leaves them there. On a host with engines `start()` pays for it a literal load, a byte load,
a compare and a branch a request (the release listing), the call only when a reset is owed; on a
host without engines the test compiles away.

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
- **The overrun** (35.5.11) - `answer()` reads nothing, so OVR rises
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
the polled loops, the engines and the runs contain no GPIO access but
the select's BSRR store.

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
// ... or, on a bus of long wires, the MOSI pad a class down:
// (void)Bus::init(clock, {.max_sck_hz = 8'000'000, .mosi_speed = brio::PinSpeed::high});

Bus::Request r{};
r.cs      = Cs::ref();
r.cmd     = brio::lend<brio::Lease::reply>(cmd);
r.cmd_len = 2;
r.rx      = brio::lend<brio::Lease::reply>(rx);
r.len     = 8;
r.clock   = brio::SpiClock::div8;
r.mode    = brio::SpiMode::mode0;
r.reply   = brio::reply_to<Device, brio::SpiDone>();
r.engine  = brio::SpiEngine::polled;   // the loop, in the arbiter's dispatch: this host has no engines
brio::post<Arb>(r);               // returns at once; `cmd` and `rx` stay lent
```

On this engineless host every request runs `SpiEngine::polled`: one naming
`SpiEngine::dma` would be refused (`spi_rejected`, `has_dma` false). **A polled request
needs no interrupt vector, on any host**: it runs an explicit loop at every
length, on a host with engines too, and arms no interrupt of the block or
of a channel.

**The DMA engines.** Name both engine parameters and bind the channels' vectors,
and a request naming `SpiEngine::dma` rides them at any length: a command
phase before data runs in the loop inside `start()` and the data on the
engines, a command alone is the block ("The engines" under Bench findings).
Engines of `uint16_t` serve a frame of either width, each with its own beat - a
byte access to DR for eight bits or fewer, a half-word one above
(35.5.9) - and a wide request whose buffers are not half-word aligned
is refused, decided before the select. The transmit
engine is armed for its errors alone: the receive block's completion is
the block's, and it proves the transmit block's, so a block takes ONE
interrupt; its channel's vector is still bound, for an error. THE RECEIVE CHANNEL ARMS AT
`very_high` AND THE TRANSMIT ONE AT `high` (docs/design/dma.md): the
host clocks a frame out whenever the transmit channel fills the TXFIFO,
and a receive channel it outran would find the 32-bit RXFIFO full - OVR,
the frame discarded and every one after it lost (35.5.11) - where a
transmit channel that waits only stops the clock between two frames.
`release()` on the host and on the client disables the block and gates
its clock, no reset pulse; `init()` starts from the reset line, before it
configures the block or arms a channel: a DMA enable a release left set
raises its request again when the clock returns, and the reset takes it
away ([dma.md](dma.md), "A released requester holds nothing here").

```cpp
using Tx = brio::DmaTxEngine<1, 1, uint16_t>;
using Rx = brio::DmaRxEngine<1, 2, uint16_t>;
using Bus = brio::SpiHost<1, bus_pins, Tx, Rx>;
static_assert(Bus::has_dma);      // a request naming the engines needs them
(void)Bus::init(clock);            // arms both engines: the gate, the requests, DR
extern "C" void DMA1_Channel1_IRQHandler()   { if (Bus::dma_isr()) { /* post */ } }
extern "C" void DMA1_Channel2_3_IRQHandler() { if (Bus::dma_isr()) { /* post */ } }
```

**The host's hot paths in SRAM.** The placement is the last template
argument; the vectors stay as they are:

```cpp
using Display = brio::SpiHost<1, bus_pins, brio::NoDmaEngine, brio::NoDmaEngine, brio::InRam>;
```

Every request that runs polled calls its transaction there - 1736 bytes
of SRAM on an engineless host and on an engined one, a tenth off the
receive loop at PCLK/2 ("The host's placement" below); the polled command phase
ahead of an engined block stays in the flash under either placement.

**A client that answers.** Load the first two answers before the host's
clock can arrive and write the next-plus-one on every received frame -
the client runs ONE AHEAD:

```cpp
using Peer = brio::SpiClient<2, peer_pins>;
(void)Peer::init(clock, {.mode = brio::SpiMode::mode0});
Peer::rxne_interrupt(true);
Peer::enable(answers[0], answers[1]);

extern "C" void BRIO_STM32G0_SPI2_HANDLER() {
    if ((Peer::isr() & brio::SpiFlag::rxne) != 0u) {
        if (const auto in = Peer::poll()) {
            take_frame(*in);          // the program's: it may run in a handler
        }
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
are the SELF-LINK's (SPI1 host to SPI2 client, four wires): 90 verdicts.
The ones under "The peer link" are a SECOND CHIP's, on the same four
SPI1 pads: 36 verdicts, measured against the STM32F446RE paired with
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
the host polled and the client one frame ahead on its own interrupt.
All four CPOL/CPHA combinations and both bit
orders byte-exact, with a DORD mismatch producing an exact two-way bit
reversal as the control. **Every frame size from 4 to 16 bits crosses
exact in both directions** with a pattern that uses every bit, which is
also the proof of figure 363's access rule; the unused top bits of a
short frame come back ZERO (a 4-bit frame carrying 0xFF reads 0x0F at
both ends).

**THE RATE LADDER SAYS MORE ABOUT THE CPU THAN ABOUT THE WIRE.** Paced
by the host's polled loop, whose write-ahead and whose refill are its pace,
**all eight BR codes carry the link byte-exact in both directions, up to
PCLK/2 = 32 MHz**. Driven as a BURST instead, by a
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
completion - the receive block's. A command in the loop before its data
block loses and repeats nothing at the seam (letter `q` against the peer,
below). A read-only request feeds 0xFF from a held
cell and a write-only one drains into a held sink (the two sibling verbs
`start_fixed()` and `start_discard()` of `stm32g0/dma.hpp`). **With BOTH
ends on DMA channels the ladder is monotone and has a real ceiling:
byte-exact to PCLK/4 = 16 MHz, slipping at PCLK/2 = 32 MHz** - a channel
sustains frames an interrupt entry cannot, which is the distinction the
CPU-driven ladder could only name.

**The engined transaction's cost, with nothing on the wire**
(`bench_stm32` letter `d`: SPI1 on PB3/PB4/PB5 with MISO floating, the
engines `uint16_t` on DMA1 channels 2 and 3, the request on them, the
completion awaited on the idle path;
[../design/benchmark.md](../design/benchmark.md)'s grammar, n in bytes,
the wire SCK/8 bytes a second):

| op | n | PCLK/ | wall | busy | irq | isr | wire cycles | rest |
|---|---|---|---|---|---|---|---|---|
| spi.dma (8-bit) | 64 | 2 | 2018 | 934 | 1 | 255 | 1024 | 994 |
| spi.dma (8-bit) | 256 | 2 | 5090 | 934 | 1 | 255 | 4096 | 994 |
| spi.dma (8-bit) | 64 | 8 | 5087 | 934 | 1 | 255 | 4096 | 991 |
| spi.dma (8-bit) | 256 | 8 | 17375 | 934 | 1 | 255 | 16384 | 991 |
| spi.dma.w16 | 128 | 2 | 3064 | 954 | 1 | 255 | 2048 | 1016 |
| spi.dma.w16 | 512 | 2 | 9208 | 954 | 1 | 255 | 8192 | 1016 |
| spi.dma.w16 | 128 | 8 | 9205 | 953 | 1 | 255 | 8192 | 1013 |
| spi.dma.w16 | 512 | 8 | 33781 | 953 | 1 | 255 | 32768 | 1013 |

Every frame beyond the first costs exactly the wire's time - 16.0 HCLK
cycles a byte at PCLK/2, 64.0 at PCLK/8, either width - so the engines
keep the FIFO fed at the top rate and the per-byte time is the wire's.
What a transaction adds is a constant 991 to 1016 cycles of wall, the same
in three runs: `start()`
itself (the engine's test, the refusals' tests, the compare of the
settings, the select, the two pads kept, the engines' five stores each),
ONE completion interrupt
whose handler is 255 cycles with the meter's stamps, and the idle
path's wake with the bench's own idle-window reads; of it the core is
busy 934 to 954. In `bench_stm32_ram`, whose ports' bodies are calls into SRAM, the
same vector's entry is 240 cycles and the transaction 982 to 1003 above the
wire - none of the engines' path is placed, so the figure is one for
both placements. What the core pays of it against a polled transfer is E - F
("Which engine costs the core less", below).

**The host above the wire** (`bench_stm32` letter `e`: the engineless
`SpiHost<1>` on the same pads, MISO floating, each op the best of 8 runs
of `start()` alone with the Request built outside the clock; the write
shape reads nothing back, the `.rx` one reads every frame into a buffer;
the levels and OVR read after every run were zero and clear on every
line, and since the loop's own reads clear OVR that witness is the bytes and the
count on the receive shapes). x is the wall over the wire's, so larger is slower.
The polled loops, 256 frames, both placements, beside the vendor's loop on the same
board:

| op | PCLK/ | `InFlash`: wall, x | `InRam`: wall, x | the vendor's loop |
|---|---|---|---|---|
| spi.poll (8-bit write) | 2 | 4639, 1.13 | 4445, 1.08 | 44709, 10.91 |
| spi.poll | 4 | 8541, 1.04 | 8541, 1.04 | 44775, 5.46 |
| spi.poll | 16 | 33116, 1.01 | 33099, 1.01 | 60398, 1.84 |
| spi.poll.rx (8-bit read) | 2 | 6480, 1.58 | 6185, 1.51 | 44709, 10.91 |
| spi.poll.rx | 4 | 8623, 1.05 | 8531, 1.04 | 44775, 5.46 |
| spi.poll.rx | 16 | 33117, 1.01 | 33095, 1.00 | 60398, 1.84 |
| spi.poll16 (16-bit write) | 2 | 8499, 1.03 | 8493, 1.03 | 48589, 5.93 |
| spi.poll16 | 4 | 16688, 1.01 | 16688, 1.01 | 48589, 2.96 |
| spi.poll16 | 16 | 65829, 1.00 | 65832, 1.00 | 103121, 1.57 |
| spi.poll16.rx | 2 | 11313, 1.38 | 10025, 1.22 | 48589, 5.93 |
| spi.poll16.rx | 4 | 16769, 1.02 | 16694, 1.01 | 48589, 2.96 |
| spi.poll16.rx | 16 | 65851, 1.00 | 65838, 1.00 | 103121, 1.57 |

The write shape is the chapter's transmit-only loop (frames in on TXE,
two byte frames an access, the unread echoes' overrun cleared at the
tail), at the wire at PCLK/4 and slower at both widths and within an eighth of
the wire at PCLK/2; the receive shape keeps three frames in flight frame by
frame below six and PACKS above it (two frames an access each way, two
pairs in flight, the odd last one by a byte with FRXTH moved for it and
back) - 24.0 cycles a frame against the wire's 16 at PCLK/2, byte frames from
the flash running over the wire at PCLK/4 too, and at the wire at PCLK/16 and
slower; its PCLK/2 and /4 figures from the flash may move with where the image
places the transaction (the open finding below). From the difference of
the 256- and 16-frame walls at PCLK/2, the loop's frame is 16.8 cycles
written, 24.0 read, 32.0 and 43.0 in half-words (16.0, 23.0, 31.95 and
38.0 from SRAM; the walls above are the best of three runs of the letter on each
image, which agree within 6 cycles). A request naming the engines on this host is refused at once - `start()` true,
`spi_rejected`, both FIFOs empty and the select high after it.

The vendor's loop is ST's HAL v1.4.7 `HAL_SPI_TransmitReceive`,
compiled from the pinned archive into a scratch program with brio's
crt, clock and console around it and measured with the same ruler on
the same pads: one frame in flight (a `txallowed` flag alternates a
TXE-gated store with an RXNE-gated load) and a `HAL_GetTick()` test per
turn, 175 cycles a frame in byte frames and 190 in half-words at PCLK/4 - it does
NOT pack in its polled path (the packing is its interrupt receiver's). So brio's loops
take 4.8 to 5.2 times less of the time at PCLK/4 in bytes and 2.9 in half-words, and at
PCLK/16, where brio's are the wire's, the vendor's stays 1.84 of it in bytes and 1.57 in
half-words. Its fixed cost at PCLK/16 is 788 and 1063 cycles above the wire for a request
of one and three frames, against brio's 310 and 394 (below).

**A request's fixed cost** - the price of a DCS command - from the same
letter: a polled command of one byte and 0, 2 and 15 bytes of data,
write-only, the select on PA15 and the D/C on PB6, at PCLK/16:

| op | n | `InFlash`: wall, rest | `InRam`: wall, rest |
|---|---|---|---|
| spi.req | 1 | 438, 310 | 413, 285 |
| spi.req | 3 | 778, 394 | 745, 361 |
| spi.req | 16 | 2481, 433 | 2456, 408 |

"Rest" is wall minus the wire's cycles, the Stopwatch's ~62 inside it;
the 3-byte request's some 332 net cycles (299 under `InRam`) are: the compare of the settings
and the clamp (~15 instructions), the select and the D/C edges (one
store each, inline), the command phase (one frame in, the tail's wait
on FTLVL and BSY, the overrun's clear and the drain - two reads), the
D/C flip, the data phase (one half-word access for the pair, the same
tail), the select raised, and the silicon's own latencies at each
phase's two ends (the first edge after the write, BSY after the last) -
some 180 instructions and 14 APB accesses from a flash at two wait
states, and one call - into the transaction's own function, where the
placement's flatten puts every loop (polled_transaction() in the
header, whose comment says why it is out of line).

**The engines** ([design/spi-bus.md](../design/spi-bus.md), "Two ways to run a
request"). A request naming `SpiEngine::dma` rides the host's two engines at any
length: `start()` keeps the select and the D/C, starts both channels and returns
false; the receive channel's completion is the one interrupt a request, and
`dma_isr()` answering true is the edge the program's glue posts `TransferDone` on.
The block's own interrupts are never armed, and `init()` leaves the instance's NVIC
line as it finds it - SPI2 and SPI3 share one line where the part has a SPI3, and
the other instance may be a client using it. In this order:

- THE ONE BUSY CHECK, first of all and the same for a polled request: while
  this host's own block is still on the engines, `start()` returns true at once
  with `spi_rejected`, before the select, the D/C or the block moves - the only
  busy check there is, no other owner holding the channels (docs/design/dma.md,
  "The ownership rule"). Letter `S` starts a block of 64 frames at PCLK/256 and,
  under it, one request naming the engines and one polled: both refused, their
  select never falling, D/C and CR1 untouched, and the block in flight ending as
  it would have, one completion, `spi_ok`.
- A host with no engines REFUSES it: `start()` returns true at once with
  `spi_rejected`, before the block, the select or the D/C is touched (letter `s`:
  the select's own pad sees no falling edge, CR1 keeps its polarity, both FIFOs
  stay empty), and `has_dma` says so at compile time.
- A reset a transfer error left owed is paid next, on a host with engines: the
  block alone, before any of the tests below.
- An empty request completes on the spot, `spi_ok`.
- What the engines cannot carry is refused the same way, before the select, the D/C
  or the wire moves: a
  9- to 16-bit frame on byte engines, or one whose buffer, out or in, is not
  half-word aligned (10.4.3's note). Letter `S` judges every case: the select never
  falls, D/C, CR1 and CR2 untouched, no edge on SCK, and the next request exact. The
  SCK count is an EXTI counter, whose handler does not keep pace with SCK's edges
  at this core's rate (which is why D/C's witness is the wire's state in D/C's own
  handler), so its zero says that no frame was clocked rather than that no single
  edge was.
- A command phase followed by a data phase runs in the transmit-only loop inside
  `start()`, then the data phase on the engines: one fixed rule, no arithmetic - a
  command is short by nature, and a block of its own would cost a second launch and
  a second interrupt. D/C rises on a wire at rest between the two (letter `S`: the
  handler of D/C's own edge, which runs before the data block is launched, finds BSY
  low, the TXFIFO empty and SCK idle). A stall in that loop ends the request as a
  polled one's does: `start()` true, `spi_stalled`, the select and the D/C released,
  the block reset.
- A command with no data phase is the block, the whole request on the engines.
- The channels then take the block: neither can refuse it, the busy check having
  left none of this host's in flight, the refusals above covering the run, and no
  other owner holding them.

**At its end, ok or failed, a request leaves the select and the D/C high**
(docs/design/spi-bus.md, "When a request ends"): a polled stall, a stall in the
command phase ahead of an engines' block, and a transfer error met in the vector
all release both, as a clean end does. Letter `K` makes three requests fail with a
command phase holding D/C low - a polled command that stalls, an engines'
request whose command stalls in the loop, a command alone on the engines whose
block reads reserved space - and reads D/C high after each.

**When a request ends, the wire has ended.** The select rises, and `start()` or
`dma_isr()` answers true, after the last SCK edge - also for a caller that frames
the select itself (`cs` null), whichever engine the request named. The receive shapes
end on their last RXNE, and in the CPHA = 0 modes (0 and 2) that comes half a bit
before the frame's last edge: letter `E`'s control, the same letter over a receive
tail that ends on the last RXNE, finds the SCK pad still at the active level after a
polled read at PCLK/256 in modes 0 and 2, every time. Nothing of the block marks that
edge for a full-duplex master: ES0548 2.12.1 is about BSY after the SPI is disabled
(a transmitting master with TXE low, a receive-only master, a client losing its
select) and 2.12.2 about a client's BSY at the end of a transfer, and this receive
tail reads no BSY. So the tail waits for the SCK pad's own input to read the
mode's idle level - the pad is sampled into IDR every AHB clock in its alternate
function (RM0444 7.3.12) - once a request, on the transaction's budget. The
transmit-only tail needs nothing more: FTLVL and then BSY already put it after the
edge (letter `E`: never early, in every mode, at PCLK/256, the one rate where the
control shows an early end). The engines' completion needs
no wait either: half a bit is at most 128 core cycles here (PCLK is HCLK, /256 the
slowest code), and the vector's own body from entry to `dma_isr()`'s answer is longer
(258 to 260 cycles with the ruler's read), so letter `E` finds the pad idle when `dma_isr()`
answers at every code, in every mode, read and write, command alone and command before
data. The instrument's reach, from the control: an early end shows at /256 (half a
bit 128 cycles) and not at /128 (64), so on the polled path the pad's read comes
between 64 and 128 cycles after the flag, and at /128 and faster the half bit is over
before the read.

**Which engine costs the core less** - guidance for the program that names one; the
host reads none of it. On the engines a request costs the core a fixed E - `start()`,
the two channels programmed, the completion's vector - whatever its length; polled, a
fixed part F and each frame's time: max(W, the receive loop's turn) for a frame read,
max(W, the transmit-only loop's turn) for a frame written, W the frame's PCLK cycles at
the code and size (bits x the division) - the FIFO keeps frames in flight on both
loops, so no frame pays an idle bus. So the engines cost the core less from N =
(E - F) / frame frames, rounded up. `bench_stm32` letter `e` measures E - F as ONE
difference: 64 frames at PCLK/16 on the engined host on the engines and polled through
the same call, the least busy of the first's eight runs less the second's wall above
the wire, so the ruler's own cycles cancel - **E 851 for a read and 863 for a
write, F 429 and 400, E - F 422 and 463** in three runs,
from the flash at 64 MHz, and from `bench_stm32_ram` E - F 467 and 474. The figures
move with the image's layout: other layouts of this host read E - F up to 448 for a
read and up to 486 for a write from the flash, which moves a cell of the table by up to
one. The turns are the same letter's polled
lines at PCLK/2, where the loop and not the wire is the bound: **24.0 cycles a frame
read in byte frames, 43.0 in half-words, 16.75 a frame written** (the wire's 16 and 32),
23.0, 38.0 and 16.0 from SRAM; at PCLK/4, byte frames read from the flash run at 32.25
against the wire's 32. What that gives, from the flash:

| the engines cost the core less from | /2 | /4 | /8 | /16 | /32 | /64 | /128 and slower |
|---|---|---|---|---|---|---|---|
| 8-bit frames read | 18 | 14 | 7 | 4 | 2 | 1 | 1 |
| 8-bit frames written | 28 | 15 | 8 | 4 | 2 | 1 | 1 |
| 16-bit frames read | 10 | 7 | 4 | 2 | 1 | 1 | 1 |

In words: a single frame already costs the core more polled than on the engines at
PCLK/64 and slower, at PCLK/32 for 16-bit frames read; at PCLK/2 a request under
eighteen byte frames read, or twenty-eight written, costs it less in the loop. A cycle count at 64 MHz: at a slower clock the same code takes fewer cycles and
the edge sits a little lower. Through the arbiter the engines pay its second dispatch
besides, which this arithmetic does not count. A request's length is the program's to
know; the choice is its own.

**The host's placement** ([platform.md](platform.md), "Code in SRAM"):
`SpiHost<..., InRam>` runs the polled transaction as ONE flattened
instance in SRAM - 1736 bytes, both widths and every shape, on an
engineless host and an engined one alike - behind the `start()` that
stays in the flash, a call through the linker's long-branch veneer. The
engines' path takes no placement (once a block), nor does `SpiClient` (its runs
are inline in their caller, which the program places). Measured by
`bench_stm32` and `bench_stm32_ram` (the tables above): the receive
loop 5 % faster at PCLK/2 in byte frames (6480 against 6185), 11 % in half-words (11313
against 10025), and nothing that matters at PCLK/4 (8623 against 8531 in byte frames)
or at PCLK/16 and slower, where the wire
is the bound; a polled request 25 to 33 cycles; and the loop's turns with it, so the
lengths of "Which engine costs the core less" move by a frame at PCLK/2.

**Two facts the letter found.** (1) FRLVL's two bits cannot count three
byte frames (above), so a drain paced by the level falls a frame behind;
every loop here reads on RXNE. (2) A two-byte phase packed costs 35
cycles MORE than frame by frame: the two accesses to CR2 for the
threshold moved and moved back outweigh the two accesses to DR a pair
saves, which is where `packed_min_frames` comes from.

**The longest handler in the image** bounds nothing here but the bus's
continuity: what is in flight on the receive shapes - written and not yet read
back - never exceeds the RXFIFO's capacity, the TXFIFO drains into it
and the clock STOPS, so a handler of any length delays the loop and
loses no frame - and on the engines the receive channel outranks the
transmit one, with no handler between them. Measured by `test_stm32_spi` letter `s`, a
wireless letter: with the tick's handler stretched to some 1500 cycles
(four byte frames at PCLK/2 are 64), a hundred receive runs of 256
bytes each at PCLK/2, PCLK/4 and PCLK/16 polled and at PCLK/4 on the
engines all complete - a lost frame never completes a run, the count never
arriving - with OVR never left standing (the loop's own reads clear it, so the
count is the witness) and both FIFOs empty after each, 13 to 15 stretched ticks
landing in each batch at the faster rates and 54 at PCLK/16. The same letter judges
every shape of the two loops by the FIFO levels, the flag and the count
of completions after it. The handlers of the bench's image, as letter `e` meters them:
the tick 36 cycles and the bench's stamped DMA vector 272, the console's receive path 87
instructions.

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

**35.9.2's LDMA_TX, measured** with the transmit engine on channel 3:
three 16-bit DMA accesses to an 8-bit frame size carry SIX frames with the
bit clear and FIVE with it set, the odd count told to the silicon being
what stops the dummy half of the last access reaching the wire. Letter
`i`'s leg rides the host's transmit engine on channel 1 and has not run on
that engine ("Not covered yet").

**The kernel letter.** Four transactions queued from one dispatch come
back in order, every one `spi_ok`, with the bytes on the wire exactly
what each request lent; a request past the arbiter's pending depth is
answered `bus_rejected` immediately; an idle bus votes ok on a
`PrepareSleep` and a busy one votes against it; and **the per-bus
timeout answers a completion that never came with `spi_timeout` and
the arbiter's own `recover()` puts the engine back** - the very next four transactions run
to `spi_ok` on the same bus AO. The staging is the honest one for a bus
whose host clocks itself: the DMA vector's body runs and ends the block
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
scores **83 of 83** (letters `a`, `m` and `s` wireless, letters
`n`..`r`, `u`..`w` and `y` on the wire). The same board measures this one's `spi_peer`
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
(= `BusMaster`) over the engined `SpiHost` with the F446 answering: four
transactions queued from one dispatch - two on the engines, one polled
inside `start()` and answered in the arbiter's own dispatch, one with a
three-frame command in the loop before its data block - come
back in order, all `spi_ok`, and the 24 bytes on the wire are exactly
what each request lent - read back and reported by the other board, not
by a second peripheral of this one; ten runs, every verdict green. The four share ONE select window because the peer's
`exchange` is one burst; the rejection past the pending depth, both
sleep votes and the per-bus timeout with `recover()` follow, and the
next four transactions run to `spi_ok` on the same bus AO.
`util/spi_bus.hpp`, `util/bus_master.hpp` and `kernel/` are realized
here exactly as they are written.

**THE INSTANCE IS SHARED BY TWO HOSTS HERE, AND THAT IS THE HAZARD,
NOT THE WIRE**: collecting the peer's report is itself a command, and
the command path brings the engineless host up on the same instance - a
reset of the block, its DMA requests with it - and takes the vectors'
routing with it. The letter brings the engined host up again and
routes the vectors to the arbiter again before the next leg; without that, the
next completion would be consumed by the wrong branch and the arbiter
would answer `spi_timeout` on a transaction that never reached the
engines.

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

**16-BIT FRAMES ON BOTH OF THIS HOST'S WAYS, AT THE WIRE'S TIME**
(letter `u`, spi_link's `spare_wide`): thirty-two 16-bit frames a window,
the F446 serving them in half-word beats, byte-exact both ways at every
judged code to 16 MHz from the polled loop and the half-word engines - a
frame on the engines a half-word beat, the beat the frame - in ten runs
of the letter. Timed from two windows of 16 and 48 frames at the same
rung (the transaction's fixed cost cancels), a frame inside the window
takes 999 and 993 permille of its sixteen SCK periods on the polled loop at
PCLK/16 and PCLK/4 and 1000 and 1002 on the engines. The F446's own
16-bit client run is exact at every judged code.
At 32 MHz, above that client's ceiling, the answers are read wrong in
every frame while the client hears them all - the 8-bit signature.

**THE TRANSMIT-ONLY SHAPES, WITNESSED** (letter `v`, `spare_receive_only`):
33 frames, 8- and 16-bit, through the polled transmit-only loop and the
engines' held sink, every one heard exact by
the F446 listening with its answer line dark (its receive DMA, then its
`receive()` run at 7 and 33 frames), at every judged code - and at 32
MHz too, printed: a client that only listens holds above the ceiling
its answers break at. Odd counts both ways, 7 and 33 byte frames, exact
against both of the peer's serves; ten runs of the letter, every verdict
green, the 16 MHz rung included.

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
exchange in 8-bit frames on the peer's DMA serve byte-exact both ways at every
code to PCLK1/2 = 22.5 MHz, from the polled host (eight frames) and from the
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

**RXDMAEN stays up under polled frames, and the block behind them is whole** (letter `B`, twenty
windows a row, fifteen runs and three of `z`). An engined host keeps the request's enable up from
`init()` on (`boot_config()`), so a polled frame or a command's echo raises a DMA request the
channel is not there to serve, and the core reads DR over it. This chip's I2C remembers such a
request and takes a stray beat at the channel's next start ([i2c.md](i2c.md), "A DMA request a
stopped channel never served is remembered"); this block's does not. On the link, eight frames
ahead and the rest of 32 on the engines, in one select window at PCLK/256 and PCLK/32: a polled
read first after `init()`, a polled write first, a command in the loop first, a transfer error
and then a polled read, a block and then a polled read. Every row 300 windows of 300 exact at
both ends, the channel's count whole and the block's first cell unwritten right behind `start()`,
no frame left in the RXFIFO, and the same with RXDMAEN taken down by hand around the polled frames.
By hand with nothing on the select: a frame left standing up to 2 ms, two frames, half-word frames,
and the I2C page's own order - the frame standing, then RXDMAEN up, then DR read - all whole, 300 of
300 a row. A frame left unread with RXDMAEN up does show, the count short and the first cell
written 300 of 300: that is what a beat at the channel's start would look like here.

**The next request after a stall or a transfer error** (letter `K`, twelve runs and three of `z`;
`recover()` called only where named). The peer's window is commanded first, and the engined
host is then not initialized again, so the request judged is the first the host has seen
since the failure and a frame left behind is a window out of place at both ends. A stall is
staged with SPE taken down under the host, which leaves the frames written standing in the
TXFIFO, on each path: a polled write and a polled read of nine frames and the command
phase of a request naming the engines, one frame, which runs in the loop. Each ends `spi_stalled` with the select reading
high and the next window, 32 frames at PCLK/32, polled or on the engines, exact at both ends;
with no reset the same windows are wrong in all 32 frames. The wall in `start()` is the budget,
the request's frames and two more at the applied rate, counted in polls: 2816 polls for the
nine-frame write and read at PCLK/32 (256 cycles a frame), 768 for the one-frame command phase. At
64 MHz that is 1.07 ms for the write and 1.47 ms for the read in this image, 24 and 34 cycles a
poll, and 136 us for the command phase, 11 cycles a poll. The two loops in the polled body keep
the budget on the stack and a poll's turn there moves with the image's layout; the command
phase's own function does not move. A poll takes at least a cycle, so a wait never gives up before
the wire could have ended, and a handler landing on the loop costs a poll at most: it is not a
time the loop can run out of behind a preemption. At the slowest rate the same arithmetic holds a
dead bus longest: 2048 frames of 16 bits at PCLK/256 are 8.4 million polls, some 3 s at 24 cycles
a poll, by arithmetic. `recover()` over three frames standing in the TXFIFO with SPE down measures 1010 cycles
(15 us).

**The order is on the pad** (letter `K`). This block drives SCK only while SPE is set: with the
pad's pull-down on in mode 3 it falls when SPE falls and rises on the enable, and with no pull it
held its level through the reset and no edge came within 1 ms on these wires. So the letter
stages mode 3 with that pull-down, as a bus with a pull on the wire would be, counts SCK's edges
through EXTI by where the select's output latch stands, and judges none with it low and at
least one with it high in `start()`, both stall paths.

**A transfer error, staged and survived** (letter `K`): a buffer at 0x3000 0000, reserved on
every part of the family (RM0444 2.2.2, the three maps), gives the 10.4.7 transfer error and no
fault of the core. A transmit buffer there stops the channel on its first beat, no frame
clocked; a receive buffer there stops the receive channel on the first frame back, the
transmit channel ahead of it, and three frames stand in the RXFIFO behind it every run. `dma_isr()`
answers true once, in 342 and 348 cycles on the ticker's ruler, `status()` is `spi_dma_fault` and
the select the request named is high. The next window, polled or on the engines, is exact at both
ends with no `recover()`; with the reset taken out, the window on the engines reads the frames
standing in front of it, all 32 frames out of place.

**Queued behind a failure, through the arbiter** (letter `K`): two requests posted from one
dispatch, the first bound to fail, the second holding the peer's window. The replies come in
order, `spi_stalled` then `spi_ok` and `spi_dma_fault` then `spi_ok`, the second's window exact at
both ends, polled and on the engines.

**The clock gate closed under the host** (letter `K`): the reset's limit. A polled write stalls
in 1073 to 1078 us, the next in 1077 or 1078 us, `recover()` answers false; the gate opened and `recover()`
called, it answers true and the next window is exact. With the gate opened and no `recover()`, CR1
reads 0x0000 (the pulse reached the block through the closed gate), one more request stalls in
1074 us and its own reset is the way back, the next window exact.

THE EXTENDED EXCHANGE FROM THAT SIDE (its letters `u`, `v`, `w`; that host's reads
judged to 11.25 MHz and printed at 22.5 MHz, where on this cable they are not exact, and
what this end hears judged at 22.5 MHz too: [the STM32F4 SPI page](../stm32f4/spi.md),
"Findings left open"): the peer's DMA serve in half-word beats and its 16-bit run exact both
ways at every judged code, and hearing every frame at 22.5 MHz; its `receive()` run and its receive
channel hearing every frame of the F446's transmit-only shapes at 7 and
33 frames in both widths, the odd last frame under the quarter
threshold; the CRC through 35.5.14's "CRC transfer managed by DMA" (the
transmit channel counting the data frames, the receive channel the
host's CRC frame too, no CRCNEXT written) and through the peer's CRC run
(CRCNEXT after its last FIFO store, a frame an access) both clean in
both widths - and THIS UNIT CATCHES A CORRUPTED WORD: the F446's CRC
frame sent as data with its last bit flipped raises CRCERR here
(`report_crcerr`) on both serves, both widths, at PCLK1/256 and PCLK1/4, every data
frame exact, and on the engines at PCLK1/2, every frame heard. The peer's CRC
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

- **the disable procedure's own waits stay a count of turns.** `Spi<n>::disable()`
  bounds its two waits at 200 000 polls, a resource verb below the host, which has
  no rate of its own to derive from and whose answer the host drops ("the
  transaction's budget" above). A rate change behind a stalled block pays them once;
  deriving them means handing the resource the applied rate.
- **no half-duplex or receive-only TASK.** `SpiDirection` and its verbs
  are on the resource and `spi_config_valid()` knows the rules, but
  `SpiHost`'s Request is full-duplex only: a bidirectional device wants
  a direction turn in the middle of a transaction, which is a second
  Request shape and belongs with its first real user.
- **no DMA engine parameters on `SpiClient`.** Left for a device-shaped
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

- **the self-link letters as the engines reshaped them**: `b`'s
  sixteen frames on the engines and its command in the loop before the
  data block, `i`'s 16-bit request from an odd address refused, its LDMA_TX leg on the
  host's transmit engine and its ladder on the engines, `j`'s arbiter over the engined host. The desk
  carries the peer link on those pads; the same paths are judged with no
  wire by letter `s` and against the F446 by letter `q`, and the
  self-link's four wires would run them.

- **the transmit-only loop's tail against a select or a D/C the TASK
  moves** (FTLVL then BSY before the edge): the peer letters frame their
  windows by hand with 30 us of hold on each side, so the tail's timing
  is never pressed - the loop and the packed receive with its odd last
  frame are judged on the wire by the F446
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

- **letters `u`'s and `v`'s PCLK/4 rung**: ten runs of each letter
  green on the desk as it stands, where an earlier fitting of the same
  four wires slipped a frame at 16 MHz in four runs of `z` of ten. What
  moved it is not separated: letter `x`'s walk of the MOSI pad's class
  against the F446's listening client at 16 MHz, run on both fittings,
  would.

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

- **the packed receive at PCLK/2 runs at 24.0 cycles a frame against the
  wire's 16** (x 1.58 on 256 frames; 23.0 and x 1.51 from SRAM): a loop
  turn of some 26 instructions with four taken branches and three APB
  accesses per pair. What would move it: the pairs-only loop unrolled by
  two. The write shape, which is what a display's pixel path runs, is on
  the wire's time.
- **whether the receive loop's flash figures move with where the image places
  it**: `bench_stm32`'s first image reads 256 byte frames in 6480 and 8623 cycles at
  PCLK/2 and /4, x 1.58 and 1.05, the engineless host's polled transaction 20 bytes into
  a 32-byte line. With the same loops 28 bytes into a line an image read 6463 and 8602,
  and at the start of a line 6867 and 9235, x 1.67 and 1.12; no build reproduces those
  two today. The flash's 64-bit lines behind the
  prefetch are the candidate; an alignment attribute on the transaction, measured on both
  placements of every host the stratum's images carry, would say.
- **a 3-byte polled request costs some 332 cycles above the wire** (299 from
  SRAM), net of the ruler: the breakdown above names the silicon's own latencies
  at the four phase ends (~50), ~180 instructions from the flash and the call into
  the flattened transaction. A ceiling
  kept as one byte instead of an optional, and the budget's arithmetic
  hoisted, are the next ten cycles; the rest is the shape of two phases
  around a D/C pin.

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
