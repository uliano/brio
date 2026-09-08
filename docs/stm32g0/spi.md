# SPI / I2S (STM32G0)

> **PROVISIONAL.** Chapter 35 is implemented whole - both roles, all
> four modes, both bit orders, the eight baud prescalers, data sizes 4
> to 16 bits with the FIFO threshold and the access-width rule, every
> NSS arrangement, TI frame format, the hardware CRC in both lengths,
> half-duplex and receive-only, the error flags with their clear
> sequences, the DMA enables with their packing bits, and the I2S
> personality with its four standards, three data lengths, prescaler
> and master clock - and `SpiHost` gives `util/spi_bus.hpp` and
> `util/bus_master.hpp` their THIRD silicon with not one line of
> `util/` or `kernel/` changed. What is still missing is in "Not
> covered yet".

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
of pads because the desk has held both (see [../bench.md](../bench.md)):
letters `a`..`m` (89 verdicts) run on the Nucleo's own SPI1-to-SPI2
self-link, letters `n`..`r` (20 verdicts) on the link to a PEER BOARD
running `spi_peer`, commanded in band over
`avrdx/src/apps/spi_link.hpp`. The peer may be any of the three ports of
that app - the AVR's, the SAM C21's or this stratum's own, which answers
on SPI1 AF0 at the same pin names this board hosts on - and the peer's
`ident` names which one answered (firmware 0x01xx, 0x02xx, 0x03xx). The two wirings exclude each other, the
suite PROBES which one is fitted before any letter runs, and the letters
whose instrument is absent skip themselves with the reason printed and
no verdict claimed. Family fixture `test/family_stm32g0/spi.cpp` plus
fourteen negatives under `tools/check_stm32g0.sh`.

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

**RXNE and not TXE drives the host pump.** TXE means "the transmit FIFO
has room", which is true well before a frame is on the wire and true
again immediately after; RXNE means "a frame has been shifted in", which
on a full-duplex bus is exactly "one frame moved, both ways" - and
reading DR is both the capture and the acknowledgement. One interrupt
per frame, and the transfer ends when the last frame has come BACK,
which is the only edge that means the wire is idle.

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
`util/spi_bus.hpp` drives. Its `Request` is the other two targets'
verbatim (cs and dc `PinRef`s, `cs_setup_us`, a command phase and a
full-duplex data phase with `Borrowed<..., Lease::reply>` loans, `len`,
`reply`, and the per-transaction bus configuration) with the rate as a
`SpiClock`, the mode as a `SpiMode` and the frame size as a
`SpiDataSize` defaulting to eight - so an 8-bit request looks exactly
like the other targets'. `init(clock, max_sck_hz = 0)`, `start(req)`
(true = completed inside start, false = the ISR posts `TransferDone`),
`isr()`, `dma_isr()`, `status()`, `prime()`, `bit_order()`,
`recover()`, `rebase(hz)`, `clock_for(hz)`, `sck_hz(code)`,
`release()`.

`LEN IS A FRAME COUNT.` With the default 8-bit frames a frame is a byte
and `len` is a byte count. With `bits` above eight a frame occupies TWO
bytes of the caller's buffer, low byte first, and `len` still counts
frames. Below eight bits a frame is still one byte, right-aligned, with
the unused top bits ignored on the way out and read back as zero.

`SpiClient<n, pins>` - the other end: `init(clock, Config)`,
`enable(first, second)` (SPE up with the first TWO answers already in
the FIFO), `write`, `poll`, `selected()` (a live read of the NSS PAD -
the peripheral publishes no such status bit), `drive_output` (the dark
listener), the error accessors, `isr()`, `disable()`, `release()`.

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
    .cmd = brio::lend<brio::Lease::reply>(cmd), .cmd_len = 2,
    .tx = {}, .rx = brio::lend<brio::Lease::reply>(rx), .len = 8,
    .reply = brio::reply_to<Device, brio::SpiDone>(),
    .clock = brio::SpiClock::div8, .mode = brio::SpiMode::mode0,
});

extern "C" void SPI1_IRQHandler() {
    if (Bus::isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
}
```

**The data phase on DMA.** Name both engine slots and bind the channels'
vectors as well; the command phase stays on the frame pump and the
engines take the data phase at its end.

```cpp
using Tx = brio::DmaTxEngine<1, 1>;
using Rx = brio::DmaRxEngine<1, 2>;
using Bus = brio::SpiHost<1, bus_pins, Tx, Rx>;
brio::Dma<1>::bus_clock(true);     // the CONTROLLER is the app's
(void)Bus::init(clock);
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
are the SELF-LINK's (SPI1 host to SPI2 client, four wires): 89 verdicts,
three green `z` runs including one from a cold flash. The ones under
"The peer link" are a SECOND CHIP's, on the same four SPI1 pads: 38
verdicts, measured first against a SAM C21 (two green `z` runs including
one from a cold flash) and then against a second STM32G0.

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
the client has all of it to answer in. Driven as a BURST instead, with
the transmit FIFO kept full, the measured frame period **floors at about
320 CPU cycles** (322 at PCLK/2 where the wire asks 16, 316 at PCLK/16
where it asks 128): a poll of TXE, a store, a poll of RXNE, a load, each
with its APB stall. **This core cannot make a continuous SPI clock out
of software**, so a CPU-driven ladder never presses the far end however
fast BR is set. The client's receive side is byte-exact at all eight
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
the CRC register and a store there is one more DATA frame instead (the
trap the first version of the letter fell into, on both ends at once).
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
`start_fixed()` and `start_discard()` this campaign added to
`stm32g0/dma.hpp`). **With BOTH ends on DMA channels the ladder is
monotone and has a real ceiling: byte-exact to PCLK/4 = 16 MHz, slipping
at PCLK/2 = 32 MHz** - a channel sustains frames an interrupt entry
cannot, which is the distinction the CPU-driven ladder could only name.
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
themselves when it is absent. On the peer desk `z` scores **38 of 38**
(letter `a` wireless, letter `m` wireless, letters `n`..`r` on the
wire) - green twice including a run from a cold flash against a SAM
C21, and green twice including a run from a cold flash against a second
STM32G0 once that peer's answer line had its edge rate set (the last
finding below: at the driver's very-high pad speed the falling-edge
modes slipped in one burst of eight).

**The wire format is the AVR campaign's, unchanged, on a third
architecture.** `avrdx/src/apps/spi_link.hpp` is compiled by three apps
now and this suite is the third to speak it: the command channel came up
first try at PCLK/256 = 250 kHz, one frame per chip-select window, ten
of ten, and `ident` names the peer's die serial and its `spi_peer`
sanity byte. The protocol owns the chip select here (the Request carries
a null `PinRef` and the letter frames the window by hand with 30 us of
hold on each side), and `prime()` is what makes that legal - a mode
change inside an open select window is one extra edge, which is the
finding the samc21 bench paid for and this end simply obeys.

**Four modes, both bit orders, byte-exact between two different
silicons**, and a DORD mismatch is an EXACT two-way bit reversal with
both ends checking the other's bytes reversed - zero mismatches either
way. The bit order is a BUS-level verb on this target
(`SpiHost::bit_order()`, CR1.LSBFIRST) where the samc21 suite has to go
round its own task for the same leg, and the LSb-first exchange rides
that verb.

**WHERE THE LADDER STOPS IS THE PEER'S ANSWER RELOAD AND NOT THE WIRE,
AND WITH A G0 PEER IT DOES NOT STOP AT ALL.** Against the SAM C21 every
BR code from PCLK/256 up to PCLK/8 carries eight frames byte-exact in
both directions and PCLK/4 = 16 MHz breaks with **the peer's own count
full and its mismatches zero** - it heard all eight characters exactly
and could not put its answers on the shifter in time (the SAM serves
through its DMA engines here, which is what its report's `serve=dma`
says). The samc21-to-samc21 bench measured the same boundary at 6 MHz
with both ends engined; a G0 host paced by its own frame pump gives that
peer one rung more. **AGAINST A SECOND STM32G0 SERVING ON ITS OWN DMA
CHANNELS THE WHOLE REGISTER VOCABULARY IS EXACT** - every code down to
PCLK/2 = 32 MHz, eight frames byte-exact both ways at each, which is the
same ceiling the self-link's own two peripherals reach on letter `i`
with both ends engined. So the top rung is now ASKED (the letter walks
it only when everything below it was exact - the ladder's job is to find
a boundary, not to assume one) and this pairing has no boundary left
inside the vocabulary.

**AND THE SAME LETTER PRINTS THE OTHER BOUNDARY BESIDE IT**, without
claiming a verdict: a second climb sets `spilink::spare_polled_pump`,
which asks the peer for its SOFTWARE serve instead of its engine. The G0
peer's software pump - one answer written per RXNE interrupt, one frame
ahead - is exact to **PCLK/8 = 8 MHz in every run, exact at PCLK/4 =
16 MHz in some runs and one frame out in others, and slips at 32 MHz
every time**: an interrupt entry per frame sits right at the edge of
16 MHz on this core. Two boundaries of one instrument, measured in one
letter: what a channel sustains and what an interrupt entry per frame
sustains, a factor of two to four apart.

**The arbiter's THIRD silicon, measured against a SECOND CHIP.** Letter
`q` runs `SpiBus` (= `BusMaster`) over `SpiHost` with the SAM answering:
four transactions queued from one dispatch come back in order, all
`spi_ok`, and the 24 bytes on the wire are exactly what each request
lent - read back and reported by the other board, not by a second
peripheral of this one. The four share ONE select window because the
peer's `exchange` is one burst; the rejection past the pending depth,
both sleep votes and the per-bus timeout with `recover()` follow, and
the next four transactions run to `spi_ok` on the same bus AO. Not one
line of `util/spi_bus.hpp`, `util/bus_master.hpp` or `kernel/` moved.

**A SUITE LESSON WITH TEETH, and it is about the vector and not the
wire**: collecting the peer's report is itself a command, and the
command path re-inits the host - which hands SPI1's vector back to the
bare pump. The arbiter's claim on that vector has to be RE-STATED
afterwards or every later completion is consumed by the wrong branch and
the arbiter answers `spi_timeout` on transactions that ran perfectly
(measured: four of them, at exactly the arbiter's limit each, with the
engine's registers proven healthy).

**THE ROLES INVERT ON THE PADS THIS BOARD HOSTS WITH.** Letter `r` makes
SPI1 a `SpiClient` on the very four pins it hosts with - MISO driven,
SCK, MOSI and NSS taken as inputs, the peer's GPIO-driven select the
hardware NSS - while the peer becomes the bus host for a bounded burst:
twelve frames received byte-exact, and the peer read this end's answer
stream byte-exact FROM THE FIRST FRAME, which is what the two preloads
before the select edge buy (35.5.8). An asymmetric bus on symmetric
wiring needs no re-jumpering to swap roles. The burst's rate is the
instrument's own: the command carries a DIVISION of the peer's clock,
so the same 32 is 750 kHz from an AVR, 1.5 MHz from a SAM and **2 MHz
from a G0**, and the peer resolves it to the BR code that produces at
most that.

**A CLIENT'S ANSWER LINE AT VERY-HIGH PAD SPEED IS AN AGGRESSOR ON A
JUMPER-WIRE BUS, AND THE FALLING-EDGE MODES ARE ITS VICTIMS.** With the
G0 peer's MISO pad at the speed `SpiClient::drive_output()` gives it
(very high), letter `o`'s four-mode matrix slipped in one burst of eight
- 20 bursts of 160 under letter `x`, the statistics letter outside `z` -
and always the same way: the burst byte-exact for its first frames, then
BOTH ENDS DISAGREE FROM THE SAME FRAME ON, the rest of the stream one
bit out of place - one clock edge too many, counted by the client's
shifter and therefore seen in the host's read of its answers too. Every
one of the twenty was mode 1 (CPOL 0, CPHA 1) or mode 2 (CPOL 1, CPHA 0)
- **the two modes that SAMPLE on the falling edge**, in which the data
lines change on the RISING edge - and modes 0 and 3 never slipped (0 of
80). The mechanism follows from that split: the client's MISO edge lands
its own propagation delay AFTER the clock edge, on an SCK that has just
settled HIGH in modes 1/2, and a fast falling edge coupled into the
clock wire beside it dips it through VIL - a spurious falling edge, one
sampling edge too many. In modes 0/3 the same edge lands on a LOW clock
and nothing happens. The proof is the knob: ONE NOTCH DOWN, at HIGH, the
slip is gone - **0 bursts of 120** - and the ladders are UNCHANGED (the
DMA serve exact to PCLK/2 = 32 MHz, the software pump the same 8..16
MHz); medium costs the top rung (16 MHz) and low two (8 MHz), on both
serves alike, which prices the trade the desk offers. What was excluded
before the cause was found still stands: the peer's serve path (engines
and pump slipped alike), the select arrangement (the pad, and SSM/SSI
which is worse for a reason of its own - a client framing on the clock
alone counts the host's CPOL settling edge and is then one bit early
from frame zero), the enable timing and the rate. The SAM peer never
showed it on the same E-side pads: its pads are slower. So the stm32g0
`spi_peer` re-states its MISO pad at HIGH after every init that drives
it, letter `o` is deterministic (38/38 twice including a cold flash),
and letter `x` stays as the instrument that measures it. The DUT's own
pads keep the driver's very-high: only the slave's edge lags the clock.

## On the second silicon

`test_stm32_spi` runs on the Nucleo-G071RB (DEV_ID 0x460, REV_ID 0x2000)
with the Nucleo-G0B1RE as its peer - THE ROLES OF THE G0-TO-G0 LINK
EXCHANGED, the same six wires, the same pin names at both ends - and
scores **38/38**, the same 38 the G0B1RE scores hosting the other way.
The self-link letters skip after the probe on this desk, as they do on E.

**THERE ARE TWO INSTANCES AND NOT THREE.** `spi_present(3)` is false, so
`Spi<3>` does not compile there; SPI2's line is SPI2_IRQn rather than
SPI2_3_IRQn (the reserve deriving the name from SPI3's presence, and
`BRIO_STM32G0_SPI2_HANDLER` binding whichever it is), and **the I2S is
SPI1's alone** - `IS_I2S_ALL_INSTANCE` names one instance on this header
where the G0B1's names two. The suite's roster and I2S-row verdicts are
written as the reserve's own statements and hold both ways; the I2S
letter itself is a self-link letter and skips on this desk.

MEASURED WITH THE G071 AS THE HOST: the BR ladder holds to **PCLK/2 = 32
MHz**, byte-exact both ways at every one of the eight codes, on the DMA
engines AND on the software pump - the same ceiling the G0B1RE reaches
hosting in the other direction, so neither die is the limit and the wire
is not either. The four modes and both orders are byte-exact, the DORD
mismatch is the exact two-way bit reversal, and the kernel letter runs
`SpiBus` over `SpiHost` against the far board with four ordered replies,
two rejections, both sleep votes and a `spi_timeout` recovered.

ES0418's SPI items are **2.14.1** and **2.14.2**, word for word the
G0B1's 2.12.1 and 2.12.2. Both are staged by letter `h`, which is a
SELF-LINK letter: NOT STAGED on this board, and said so rather than
assumed from the other die.

## Not covered yet

**Driver gaps** - implemented nowhere, and a program that wants them
must reach the registers itself:

- **no half-duplex or receive-only TASK.** `SpiDirection` and its verbs
  are on the resource and `spi_config_valid()` knows the rules, but
  `SpiHost`'s Request is full-duplex only: a bidirectional device wants
  a direction turn in the middle of a transaction, which is a second
  Request shape and belongs with its first real user.
- **no DMA engine slots on `SpiClient`.** The samc21 left them for a
  device-shaped user and so does this; the bench drives the client's
  channels raw, which is what a peer does anyway.
- **the tasks claim their pads at very-high speed with no knob.**
  `SpiHost` takes SCK and MOSI and `SpiClient::drive_output()` takes
  MISO at `PinSpeed::very_high`, and the bench measured that a client's
  MISO at that speed slips the falling-edge modes on a jumper-wire bus
  (above). The pad speed is the WIRE's electrical question, not the
  peripheral's; today a program that wants another re-states the pad
  after `init()` (the stm32g0 `spi_peer` does), and a `PinSpeed` field
  on the two tasks' configs is the driver's answer, born with its second
  user.
- **the host's DMA path serves frame sizes of eight bits and below.**
  The engine slots carry `uint8_t`, so a request with `bits` above eight
  on an engined host runs on the frame pump. A second pair of half-word
  slots would double the task's template surface for a case the pump
  serves correctly; declined, and stated in the header.
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
  reason is the samc21 bench's measured one-bit slip; the same slip has
  not been staged here. What HAS been measured on this target is its
  mirror image at the other end: a peer client that frames on the clock
  alone (SSM/SSI) counts a host's CPOL settling edge and is one bit
  early from the first frame, which is the same fact seen from the
  client's side.
- **the second silicon runs the PEER and not the suite.** A
  Nucleo-G071RB serves `spi_peer`'s stm32g0 port on the far end of the
  link - the same driver's `SpiClient`, `SpiHost` and the two DMA
  engines on a second die, exercised from the outside by every peer
  letter - but `test_stm32_spi` itself has never been run ON it, so the
  self-link half and the wireless letter `a` are one G0B1RE's.

**Declined, with the reason:**

- **the answer-turnaround ceiling of the ISR-driven client** (letter d):
  a ceiling needs a clock the CPU cannot gap and this core cannot make
  one, so what a CPU-driven ladder measures is its own loop. Measured
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

On the second silicon (the Nucleo-G071RB), NOT COVERED because the letters
that would cover it are the SELF-LINK's and this desk does not carry it
there: **ES0418 2.14.1 and 2.14.2** (the two BSY items), the frame matrix,
the eight BR codes host-to-host, NSS four ways, the CRC, TI mode, the OVR
and the whole I2S personality - which that part has on SPI1 alone. All of
them are measured on the G0B1RE; none is claimed for the G071.
