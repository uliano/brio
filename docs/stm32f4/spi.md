# SPI and I2S - one block, two faces (STM32F4)

Documents of record: RM0090 Rev 22 ch. 28 (RM0390 Rev 6 ch. 26 and
RM0383 Rev 4 ch. 20 are its twins: one register description, three
manuals, and the memory maps differ on which instances wear the audio
face), the datasheets' alternate-function tables for the pads, RM0090
tables 43 and 44 with their RM0390 and RM0383 twins for the DMA request
mapping, and the errata's SPI/I2S items - ES0206 2.12, ES0287 2.11,
ES0298 2.14: "BSY bit may stay high when SPI is disabled", "anticipated
communication upon SPI transit from slave receiver to master", "wrong
CRC calculation when the polynomial is even", "corrupted last bit of
data and/or CRC received in Master mode with delayed SCK feedback",
"BSY flag may stay high at the end of a data transfer in Slave mode".
Driver: `stm32f4/spi.hpp` (`Spi<n>` the resource, `I2s<n>` and
`I2sExt<n>` the audio face, `SpiHost<n, pins, TxEngine, RxEngine>` the
bus engine, `SpiClient<n, pins>` the other end, the vocabulary
`SpiConfig`, `SpiMode`, `SpiClock`, `SpiDataSize`, `SpiNss`,
`SpiDirection`, `SpiFrameFormat`, `SpiFlag`, `SpiPins`, `I2sConfig`,
`I2sPins`, and the two arithmetics), over the reserve's instance,
audio and request-mapping facts (`stm32f4/device_tables.hpp`), with
the audio PLL and the I2S clock selector on `Rcc`
([clock.md](clock.md)) and the streams from [dma.md](dma.md). The
family fixture is `test/family_stm32f4/spi.cpp` with the negatives
that refuse an absent instance, one pad twice, a host with no MOSI, a
pad on an absent port, an engine off the request map, an engine on a
part class whose manual was not read, one engine of two, two engines
on one stream, engines a word wide, the audio face on an instance that
has none, on a part class that is not known, an extension block that
does not exist, a client run of 32-bit words, and a slew class named
for a pad the task does not drive. Bench:
`test_stm32f4_spi` on the STM32F429I-DISC1, against the gyroscope and
the display controller the board carries on SPI5, and on the
Nucleo-F446RE's SPI2 against the STM32G0B1RE it is paired with on the
desk (that board's `spi_peer`); the stm32f4 `spi_peer` on the same SPI2
is the G0B1RE suite's client; on the Nucleo-F446RE's SPI1 with MISO
floating, the engined request's cost (letter d of `bench_stm32f4`) and
the host's own loops against the wire and against the vendor's (letter
e).

## What the silicon does

**The F1 lineage, as the USART of this family is.** CR1/CR2/SR/DR/
CRCPR/RXCRCR/TXCRCR/I2SCFGR/I2SPR, no FIFO and no frame size beyond
eight or sixteen bits: one transmit buffer, one receive buffer, one
shift register (26.3.9). TXE means "the buffer moved into the shifter"
- at the first bit of the frame, so the next frame can be stored behind
it and the stream is continuous (figure 311) - and RXNE "a frame has
been shifted BOTH ways", on the last sampling edge. An overrun (26.3.13)
leaves the receive buffer holding the frame before it and loses every
frame received after it, the transmit side going on regardless; its
clearing sequence is a DR read and then an SR read. The host's loops
are written on those facts: "The host above the wire", below.

**Up to six instances, one vector each, and the bus decides the rate.**
SPI1 on every part; SPI2 on every part but the 36-pin F410Tx; SPI3 on
every part but the F410 line; SPI4 on the F401, F411, F412, F413,
F42x/F43x, F446 and F469; SPI5 on the F410's larger packages, the F411,
F412, F413, F42x/F43x and F469; SPI6 on the F42x/F43x and F469 classes
alone. SPI1, SPI4, SPI5 and SPI6 sit
on APB2 and SPI2 and SPI3 on APB1, so BR[2:0] divides ITS bus's clock
and not SYSCLK - 90 against 45 MHz at 180 MHz of core
([clock.md](clock.md)). Every instance has a vector of its own, shared
with nothing.

**Enable-protected and not.** DFF and CRCEN "should be written only
when SPE is 0" (28.5.1), and BR, CPOL, CPHA, LSBFIRST and MSTR "should
not be changed when communication is ongoing" - so a request that
changes the frame size, the mode or the rate costs a disable/enable
pair, and one that changes nothing costs nothing. BIDIOE is the
exception the chapter allows while the block runs, which is what makes
a three-wire turnaround possible at all.

**Three chip-select arrangements** (28.3.1). SSM with SSI is the
software one, and it leaves the NSS pad free - which is what a bus AO's
GPIO select needs; a host under it must hold SSI HIGH, because a low
SSI on a master IS the mode fault. SSOE drives the pad low while SPE is
set and gives up multi-master. The hardware input is the multi-master
arrangement, where another master pulling NSS low demotes this one.

**The flags are cleared by sequences, not by a clear register**
(28.3.10): the overrun by reading DR and THEN SR - the DR read alone
does not do it; the mode fault by an SR access while MODF stands and
then a CR1 write; the CRC error by writing a zero into it, the one
rc_w0 bit of the register; the TI framing error and the I2S
desynchronization by reading SR; the I2S underrun by reading SR.

**Four errata are live on every part with a manual on the desk**, and
three of them are answered as code:
- *BSY may stay high when the SPI is disabled* (2.12.1). The disable
  procedure waits for TXE and then for BSY in the transmitting
  configurations, and in MASTER RECEIVE-ONLY - RXONLY, or bidirectional
  with the output off - it does not look at BSY at all: the errata's own
  instruction, and 28.3.7 already says the flag is kept low there.
- *Anticipated communication upon a transit from slave receiver to
  master* (2.12.2): the clock starts on the MSTR write even with SPE
  clear. `configure()` pulses the RCC reset line before writing a HOST
  configuration over a client receive-only one - the errata's first
  workaround, free on the ordinary path.
- *A wrong CRC when the polynomial is even* (2.12.3): an even
  polynomial is refused, so the unit is never armed with a divisor that
  computes nonsense. The reset value 0x0007 is odd.
- *The last received bit may be wrong when the SCK feedback is slow*
  (2.12.4). The internal loop back from the SCK PAD must return inside
  two APB periods, and the errata's table gives the ceiling per pad
  class at a 30 pF load: 84 MHz of APB at `high` or `very_high`, 75 at
  `medium`, 25 at `low` - halved for I2S. It is a PAD fact and not a
  rate fact: the SPI's own bit rate does not enter it. Every pad this
  driver hands over goes out at `very_high` unless the task's `Config`
  names another class ("The pads").

**The audio face is one bit of I2SCFGR**, and which instances have it
is the manual's memory map and not the header - every SPI_TypeDef of
the pack declares I2SCFGR and I2SPR, on instances the manual gives no
I2S at all. It is SPI2 and SPI3 on the F405 class and the F42x/F43x
class; SPI1, SPI2 and SPI3 on the F446 (RM0390 table 165 - SPI4 has
none); every instance from SPI1 to SPI5 on the F411. Full duplex
is a SECOND BLOCK on the classes that have one (I2S2ext, I2S3ext -
always slaves, on the instance's own clock and word select) and two
whole instances paired on the F446, which has none.

**The I2S kernel clock is not the APB's**: the audio PLL's R output, an
external clock on the I2S_CKIN pad, and on the parts with the
RCC_DCKCFGR pair the main PLL's R or the root itself. THE PAIR IS
NUMBERED BY BUS AND NOT BY INSTANCE - I2S1SRC is the APB1 instances'
and I2S2SRC the APB2 ones' (RM0390 6.3.24) - a trap the names invite.
On the F42x/F43x class the audio PLL has no input divider of its own
and shares the MAIN PLL's M, so its VCO input is whatever the system
clock already fixed.

**The clock generator's arithmetic** (28.4.4): the whole divisor is
`2 x I2SDIV + ODD`, and a frame is two channels of 16 or 32 bits -
except with the master clock out, where the divisor becomes 256 x that
whatever the channel width, which is the MCK = 256 x FS rule. I2SDIV 0
and 1 are forbidden. A channel is 32 bits by hardware whenever the data
is wider than 16, whatever CHLEN was written with.

**A request reaches one, two or three cells of the fabric.** The DMA
request mapping gives each instance a short list of (controller,
stream, channel) triples and no others, and the lists differ by part -
the F411's SPI1_TX, SPI4_RX and SPI5_TX have a third cell no other
manual shows, which is why this chapter's placement list is three deep
where the serial and analog ones are two.

## Types and verbs

`Spi<n>` - the resource, one instance in its SPI face. Facts:
`number`, `on_apb2`, `irq`, `has_i2s`. The gate and the reset:
`bus_clock`, `reset`. The configuration: `configure` (the whole
`SpiConfig` with SPE clear, refusing what `spi_config_valid` refuses),
`enable`, `enabled`, `disable` (the errata-aware drain),
`master_receive_only`, `bidirectional_output`, `bidirectional`. The
data: `data` in both directions, `data8`, `data_address`, `rx_ready`,
`tx_empty`, `busy`, `status`, `flag`, `flush_rx`. The errors:
`overrun`/`clear_overrun`, `mode_fault`/`clear_mode_fault`,
`crc_error`/`clear_crc_error`, `frame_error`/`clear_frame_error`. The
CRC unit: `crc_polynomial` both ways, `crc_enable`, `crc_enabled`,
`crc_next`, `crc_next_pending`, `rx_crc`, `tx_crc`. The select:
`software_select`, `software_selected`, `nss_output`. DMA and
interrupts: `dma_requests`, `dma_transmit`, `dma_receive`,
`rxne_interrupt`, `txe_interrupt`, `error_interrupt`, and the ISR body
`isr()` which reports the raised-and-enabled sources and consumes
nothing.

The configuration knobs (`SpiConfig`): `role` (host, client - default
host), `mode` (the four Motorola modes, default 0), `clock` (`SpiClock`
div2..div256, default div16), `bits` (8 or 16, default 8), `lsb_first`
(default false), `nss` (software, hardware_output, hardware_input -
default software), `direction` (full_duplex, receive_only,
half_duplex_out, half_duplex_in - default full duplex), `frame_format`
(motorola, ti - default motorola), `crc` with `crc_polynomial` (default
off, 0x0007), `dma_transmit` and `dma_receive` (default off). The
refusals: an even polynomial, SSOE on a client, TI mode beside a
software select or LSB-first.

The arithmetic, all constexpr: `spi_sck_hz(pclk, code)`,
`spi_rate_for(pclk, max_hz)` (the coarsest code at or below a limit,
nothing when even PCLK/256 is too fast), `spi_frame_is_halfword`,
`spi_data_bits`, `spi_frame_mask`, `spi_mode_cpol`, `spi_mode_cpha`,
`spi_cr1_of`, `spi_cr2_of`, `spi_master_receive_only`, and
`spi_errata_apb_ceiling_hz(pad_speed, for_i2s)` - the errata's own
table as a function.

`I2s<n>` and `I2sExt<n>` - the same block in its audio face, the second
being the full-duplex extension where the part has one. `configure`
(the whole `I2sConfig` with I2SE clear, a master mode refused on an
extension block), `select_spi_mode`, `enable`, `enabled`, `disable`,
`mode`, `i2s_mode_selected`, the generator (`prescaler`,
`prescaler_div`, `prescaler_odd`, `master_clock_out`), the data
(`data`, `data_address`), the flags (`tx_empty`, `rx_ready`, `busy`,
`status`, `right_channel`, `underrun`, `overrun`, `frame_error` with
their clearing verbs), the DMA and interrupt enables, and `isr()`.

The audio knobs (`I2sConfig`): `mode` (client/host x transmit/receive),
`standard` (philips, msb_justified, lsb_justified, pcm), `pcm_long_frame`
(only under PCM), `data` (16, 24, 32 bits), `channel` (16 or 32, forced
to 32 by hardware above 16-bit data), `clock_idle_high`,
`master_clock_out`, `div` (2..255) and `odd`, and the two DMA enables.
The arithmetic: `i2s_frame_factor`, `i2s_fs_hz`, `i2s_prescaler_for`,
`i2s_channel_is_32`, `i2s_config_valid`, `i2s_i2scfgr_of`,
`i2s_i2spr_of`.

`SpiHost<n, pins, TxEngine, RxEngine, hold_off_cycles>` - the engine
[the shared SPI bus](../design/spi-bus.md) drives, with the other
strata's `Request` field for field: `cs`, `dc`, `cmd`, `tx`, `rx`,
`len`, `cmd_len`, `bits`, `polled`, `cs_setup_us`, `clock`, `mode`,
`reply` - in THAT order, 40 bytes with no padding, the first nine words
being what the asynchronous path keeps for its tenure. `init(clock,
max_sck_hz)` (the shared surface) and `init(clock, Config)` - `Config`
being `max_sck_hz`, `sck_speed` and `mosi_speed`, the classes of the two
pads a host drives ("The pads") - `rebase(sysclk)`, `clock_for`,
`sck_hz`, `max_sck_hz`,
`ceiling_clock`, `reference_hz` (the instance's APB clock), `prime`,
`bit_order`, `lsb_first`, `start`, `isr`, `dma_isr`, `status` (`spi_ok`,
`spi_dma_fault`, `spi_overrun`, `spi_stalled`), `overruns` (the
pump's count), `two_in_flight(clock, bits)` (what the write-ahead
threshold says of a rate), `recover`, `release`, `claim_nss_pad`; the
constants a program reads the thresholds off - `hold_off` (the image's
declared hold-off, below), `write_ahead_min_frame_cycles`,
`dma_fixed_cycles`, `dma_min_frames`, `polled_write_turn_cycles`,
`polled_read_gap_cycles` - and the arithmetic behind the first two at
namespace level: `spi_isr_entry_cycles`, `spi_isr_to_read_cycles`,
`spi_default_hold_off_cycles`, `spi_write_ahead_min_frame_cycles()`,
`spi_frame_hclk_cycles()`, `spi_write_ahead_code()`; with engines, the
transmit stream armed for its errors alone and the receive block's
completion the transaction's one interrupt - THE RECEIVE STREAM AT
`very_high`, THE TRANSMIT AT `high` (docs/design/dma.md's rule): the
receive buffer holds one frame, and a receive stream the transmit one
outran finds it full - OVR, "all other subsequently transmitted bytes are
lost" (RM0090 28.3.10) - where a waiting transmit only stops the clock
between two frames; `release()` stops both streams, disables the block
and pulses its RCC reset before the gate, the release contract (a raised
request is held through a gated clock on this family, [dma.md](dma.md))
- and this stratum's three
of its own: `sck_speed` and
`errata_apb_ceiling_hz`/`within_errata_ceiling`, because on this
family the SCK pad's slew class is a correctness parameter and not a
taste, and `mosi_speed`, because on a bus of wires the data pad's edge
is one too and no erratum speaks for it (the breadboard finding
below) - each verb moving between transactions the class `init()`
wrote from the `Config`.

`SpiClient<n, pins>` - the polled RUNS and, as their degenerate case,
the per-frame verbs, with an ISR body: `init(clock, Config)` (the
`SpiConfig` fields a client can be told, `drive_output`, and
`miso_speed`, the class of the one pad a client drives), `enable()` (SPE up, the
transmit buffer left empty for a run) and `enable(first)` (SPE up with
the first answer written), `answer(out, spins)`, `exchange(out, in,
spins)`, `receive(in, spins)` - each returning a `Run` (`count`, and a
`RunEnd`: `complete`, `closed`, `stalled`) over `uint8_t` or `uint16_t`
frames, a wider element refused at compile time - `disable`, `write`,
`writable`, `poll`, `selected`, `select`, `drive_output` (the dark
listener), the error verbs, `isr()`, `release` (the block reset before
the gate, as the host's), and `frames_ahead` = 1, this silicon's answer
to the one number a portable client would need.

`SpiPins` and `I2sPins` name the pads with the alternate function the
DATASHEET gives each signal there; the device header carries no pin
table, so nothing can check an AF and the bench is the check.

## The host above the wire

What a transaction costs beyond its frames is the host's own, and each
piece of it is written from the chapter and counted in the release
listing (`bench_stm32f4.lst`; the numbers are under "Bench findings").

**The request is lent for the call** (util/bus_master.hpp's contract),
so the polled path reads every field through the reference and copies
nothing, and the asynchronous path copies the nine words its tenure
needs - the two pins, the three buffers, the two lengths, the width and
the completion style, laid out contiguously at the head of the Request
- as inline loads and stores: nine words, no call. The rate, the mode,
the setup time and the reply are spent before `start()` returns.

**`apply()` is one compare.** CR1's CPOL and CPHA are the mode's two
bits in the mode's own order, BR the rate's three at bit 3 and DFF the
width's at bit 11 (26.7.1), so the request's three fields fold into the
CR1 word with two shifts and two ors and are compared with the word
applied - 13 instructions on the unchanged path, the ceiling's clamp 6
more - and only a change pays 26.3.10's disable (TXE, then BSY) and the
enable, DFF being enable-protected and BR, CPOL and CPHA "not to be
changed when communication is ongoing".

**The select and D/C edges are one BSRR store each**, the null D/C a
predictable branch, inline through the family's own `PinRef::set()` and
`clear()` (pin.hpp forces the two verbs inline: left to -Os they are
calls, four a transaction). The setup time is tested before any call. **The receive buffer is not
flushed per request**: every path reads back every frame it clocks, so
nothing stands in it when a transaction ends well; the two ends that
leave a frame there - an overrun, a stall - and `recover()` flush it
themselves, `init()` once.

**Three polled shapes, by the request's fields.** A handler can land on
a polled loop at any instruction, and on this platform no interrupt
nests, so every handler in the image is a window the loop cannot see
through - the tick's, the console's, the bench's stamped ones. That
decides the shapes:

- *Transmit-only* (`rx` null: every command phase, a display's pixels):
  paced on TXE, the answers never read; at the tail the last frame is
  let out of the shifter - TXE, then BSY, 26.3.10 - and the receive side
  cleared by DR then SR, the overrun the unread answers raised included
  (26.3.13: the transmit side is untouched). Nothing is waited for on
  the receive side, so a handler delays the loop and loses nothing:
  wire-bound at every rate, 16 instructions and two APB2 accesses a
  frame.
- *Receive, one frame in flight* (`rx` set, below the threshold):
  nothing is queued behind the frame shifting, so nothing can be lost;
  the turnaround is the minimum - the next frame fetched inside the wire
  time and pinned there with an empty asm (the compiler would sink the
  load past the volatile poll), then the DR read and the DR write
  adjacent. The bus idles for that turnaround every frame.
- *Receive, two frames in flight* (`rx` set, above the threshold):
  figure 311's procedure - the next frame written as soon as TXE allows,
  the one that came back read on RXNE - so the bus never idles; a
  handler longer than a frame landing before the read would lose the
  queued frame, which above the threshold none does, and the SR read
  after every DR read (the overrun's own clearing sequence) is what
  says so: an overrun ends the transaction with `spi_overrun`, counted.

One budget bounds the whole transaction's spins; when it runs out the
block is reset and reconfigured and the transaction ends with
`spi_stalled`.

**The pump keeps one or two frames in flight, by the rate.** The handler
reads frame k FIRST - the SR read, the RXNE test, the DR read: four
instructions - then tests OVR in a SECOND SR read, after the DR read,
and writes the next frame owed. The order is 26.3.13's: the frame in
flight behind frame k may complete at any moment before the DR read,
the buffer then keeps frame k and drops that one, and a status copy
taken before the read would miss the OVR its completion raises - frame
k stored as if nothing happened, the loss seen a handler late inside a
run and, on the phase's last pair, by no handler at all: a transaction
that never completes (measured, below). The SR read after the DR read
sees every overrun up to the read and is the flag's clear. With two in
flight the write is frame k + 2, and frame k + 1 is NOT yet shifting at
RXNE: RXNE rises on the last sampling edge of frame k (26.3.9), half a
bit before frame k + 1 leaves the Tx buffer at its own first bit, and a
write in that half bit overwrites it ("the data waiting for transaction
is overwritten") - measured on SPI2 at PCLK1/256 (half a bit = 512 core
cycles): one frame of six never clocked, the transaction never
completing, every arbitrated request answered `spi_timeout`. So the pump
keeps two in flight only where the half bit is shorter than the
handler's own entry and read (`spi_isr_entry_cycles` +
`spi_isr_to_read_cycles` = 27 cycles), and its write waits for TXE
anyway, a spin the rule keeps inside that entry; elsewhere one frame is
in flight, and a write at RXNE lands inside the half bit, the bus
continuous with one frame queued. Two in flight overruns the receive buffer if
the read comes later than one frame time after RXNE, and the latest it
can come is THE IMAGE'S HOLD-OFF - the longest it keeps this vector from
running: a handler that started just before RXNE running its whole body
(no interrupt nests over another) or a masked window as long - then its
exit, this vector's entry and its read:

    write_ahead_min_frame_cycles = (hold_off_cycles + 2 x spi_isr_entry_cycles + spi_isr_to_read_cycles) x 5 / 4
                                 = (150 + 24 + 15) x 5 / 4 = 236 HCLK cycles at the default

with the inputs counted in the listing: the Cortex-M4's exception entry
and exit 12 cycles each at zero wait states (the Cortex-M4 TRM, DDI
0439B 3.9.1), the handler's four instructions with two APB2 accesses at
HCLK/2 about 15, and a quarter on top for the flash's wait states.

**The hold-off is the image's, and the image declares it**: SpiHost's
last template parameter, `hold_off_cycles`, in core cycles at the clock
the host runs at - the longer of the image's longest handler and its
longest masked window. Its default, `spi_default_hold_off_cycles` = 150,
is the bench images' own: letter d's DMA completion vector with its
stamps in `bench_stm32f4` (125 cycles of body, its entry and exit on
top); the console's transmit handler is 71, the tick's 28, the kernel's
longest masked window - `post()` of a Request into a bus AO's queue, 32
instructions six of which are four-word multiples - about 62. An image
measures its own the way `bench_stm32f4` does: an `IsrMeter`
(util/bench.hpp) around each handler it binds, its `isr` over `irq` a
handler's body and the entry and exit added on top, and the masked
windows counted in its release listing; the longest of them is the
figure it declares. A hold-off of 0 is refused at compile time: an
image with no handler of its own still has the tick's. The witness of
a figure declared too short is `overruns()` - and `spi_overrun` in the
transaction's status - at a rate just above the threshold it derives
(measured, letter e's `spi.short` below); a figure declared too long
only keeps one frame in flight where two would have held. A frame at BR code c is
`bits x 2^(c + 1)` PCLK cycles, twice that in HCLK on APB2 at 180 MHz:
so 8-bit frames keep two in flight from /16 (256 cycles) and 16-bit
frames from /8, and faster than that one frame is in flight, the bus
idle for the handler; a hold-off of 300 makes the threshold 423 and moves
both a code slower, /32 and /16 (pinned beside the default's in the
header). The half bit then bounds the pump from the other side: two in
flight at /16 and faster on APB2, at /8 and faster on APB1 (a frame
there is four HCLK a PCLK cycle, the half bit 2^(c + 2)), so on APB2 at
the default only /16 in 8-bit frames and /8../16 in 16-bit ones keep
two, and on APB1 /8 alone in 8-bit frames and /4../8 in 16-bit ones;
the polled loop, which waits for TXE before every write, keeps the
hold-off's rule alone.
`two_in_flight()` answers the pump's arithmetic for a program. The D/C boundary drains to one frame either
way: the first data frame is written only after the last command frame
came back, because a frame queued in the transmit buffer goes out the
moment the one before it ends.

**The engines take a data phase worth their fixed cost**, two sums for
the two completion styles (the numbers are letter d's and letter e's):

- asynchronous: the engines cost 547 cycles a transaction (wall minus
  the wire's time, the completion interrupt and the idle turn it ends
  inside it); the pump costs the core 151 a frame at 5.625 MHz (the
  handler's 94, its entry and exit, the idle turn each interrupt ends;
  185 at 22.5 MHz where the bus waits for the handler) - so
  `dma_min_frames` = 4, and below it the pump serves even with engines
  bound;
- polled: the thread spins on the engines' completion as it would spin
  in the loop, so the engines buy no core time, only the wall time the
  loop's shape leaves on the table - the transmit-only loop's 37-cycle
  turn against a 32-cycle frame at /2 (5 cycles a frame; nothing from
  /4 down, nothing in 16-bit frames), the one-in-flight read's 29-cycle
  turnaround at any rate below the threshold, nothing with two in
  flight - and a phase goes to the engines where that deficit over its
  frames exceeds 547: a write of 8-bit frames at /2 from 110 frames up,
  a one-in-flight read from 19 frames up, never otherwise.

So a DCS command of one to five bytes takes the pump on an engined host
too, and a display's pixel rows at /4 take the loop (wire-bound) while
a bulk read-back takes the engines.

## The client run

A client's unit is the run: the answers to a transaction, or what it
brings, in one polled call. What this block offers a run, each item used
or declined:

- **One transmit buffer in front of one shifter** (26.3.9) - USED, and
  it fixes the shape. The buffer moves into the shifter at a frame's
  first bit and TXE rises then (ES0298 2.14.6: "once the second bit
  transaction is ongoing"), so the next answer may be written at any
  moment from that TXE to the next frame's first bit: SEVEN BITS of
  window. The runs write on TXE - the buffer refilled as soon as the
  shifter takes it - where an answer written at the RXNE of the frame
  before has half a bit.
- **RXNE on the last sampling edge, a receive buffer one frame deep**
  (26.3.9) - USED: `exchange()` reads each frame on its RXNE, one turn
  of two polls, a store and a read inside the next frame.
- **No FIFO, no frame packing** - the chapter has none: the run's beat is
  the frame, 8 or 16 bits (DFF), its element type.
- **No end-of-transaction flag on a slave, BSY no witness** (ES0298
  2.14.6) - DECLINED BSY: the NSS PAD is the window. A run reads the pad
  once every four polls of the flag, waits for it to fall before a high
  pad means the close, and without a pad ends on its count or budget.
- **The TXE and RXNE interrupts** - DECLINED for the runs: a run is a
  polled loop and the ISR body stays for a program that wants an
  interrupt client.
- **The DMA requests** (SPI2_TX on DMA1 stream 4 and SPI2_RX on stream 3,
  channel 0 both, RM0390 table 28) - DECLINED in the driver: SpiClient
  has no engine slots (the gap list); a program binds the streams raw, as
  the stm32f4 `spi_peer`'s exchange serve does, started before SPE in
  26.3.11's order so the transmit stream's first beat is the preload.
- **The overrun** (26.3.10) - `answer()` reads nothing, so OVR rises from
  the second frame; its tail runs the DR-then-SR sequence. `exchange()`
  and the start of a run clear it the same way.
- **The CRC unit** (26.4.2) - NOT in the runs: a run has no CRC phase
  (the gap list). The stm32f4 `spi_peer` serves spi_link's CRC beside
  them two ways, both measured from the G0B1RE (below): on its streams
  with the chapter's own "CRC transfer managed by DMA" - both streams
  counting the data frames, the block appending its CRC after the
  transmit stream's last and checking the host's, CRCNEXT never written,
  the received CRC frame then read from DR, where it "is always loaded"
  - and with a run of its own over the resource in the runs' shape,
  CRCNEXT written "as soon as the last data is written" (26.7.1), so the
  bit lands while the frame before the last is still shifting.
- **TI frames** - not in the runs: no peer on the desk speaks them.

COST, counted in the release listing (the stm32f4 `spi_peer`'s, SPI2):
`answer()`'s turn is the TXE poll that hits - a load, a shift, a branch -
then the answer loaded and stored, seven instructions and two peripheral
accesses a frame; `exchange()`'s is thirteen instructions and four
peripheral accesses (the two flag reads that hit, the DR store and the
DR read), no call in either loop. The vendor's own slave loop (HAL
v1.8.5 `HAL_SPI_TransmitReceive`, 8-bit, -Os) alternates TXE and RXNE
through its handle's fields - some forty-five instructions a frame - and
calls `HAL_GetTick()` on every pass. The runs are always inline and the
program places their caller: here the flash, which the ART accelerator
serves at zero wait states.

MEASURED against the STM32G0B1RE host (`test_stm32_spi` letter p, eight
frames back to back, the peer's DMA serve and its polled run side by
side): exact both ways at every code to PCLK/4 = 16 MHz on both serves,
where the RXNE reload this peer ran broke at 1 MHz. The top rung, 32 MHz,
is outside this client's specification - DS10693 table 63 gives the
slave 22.5 MHz on SPI1..4, and its note 2 the reason, tv(SO) plus the
host's setup inside half a clock - and there the answer path fails on
both serves (the DMA serve still receives every frame exact, the run
loses one); it is a figure, never a fault to fix. As the client of the
G0's `host_burst` (letter r of this stratum's suite, twelve frames at 2
MHz) the exchange run is exact both ways. The same host's extended
letters judge the rest of the surface at every code to 16 MHz:
`exchange()` over `uint16_t` frames (letter `u`), and `receive()` with
the answer line dark at 7 and 33 frames in both widths (letter `v`) -
which holds at 32 MHz too, printed: a client that only listens is not
bound by its answer path.

## The pads

The SPI's pads are GPIO pads in an alternate function, and what the
silicon offers there that bears on a wire is the output stage's SLEW
CLASS - OSPEEDR, four classes (RM0390 7.4.3 "low, medium, fast, high",
brio's `PinSpeed` `low`, `medium`, `high`, `very_high`), their reach in
DS10693 table 58 at 2.7 V and up: 4 MHz at 50 pF, 25 MHz at 50 pF, 50
MHz at 40 pF, 100 MHz at 30 pF. Each item used or declined:

- **The class of every DRIVEN pad, as configuration** - USED.
  `SpiHost::Config` names `sck_speed` and `mosi_speed`,
  `SpiClient::Config` names `miso_speed`. `init()` writes each with its
  pad's alternate function, once; the client's `drive_output(true)`
  re-applies its class wherever it hands the answer line back; no
  transaction touches OSPEEDR. The host's `sck_speed()` and
  `mosi_speed()` move a class between transactions for a program that
  weighs it, and the next `init()` writes the `Config`'s again. The
  default is `very_high` on all three: ES0298 2.14.5 (table 5, and its
  ES0206 / ES0287 twins) asks a host's SCK for high or very-high above
  75 MHz of APB, and table 63 characterizes the SPI's timing at no
  slower than high (OSPEEDR = 10, 30 pF).
- **An input's class** - none to give: OSPEEDR acts on the output stage,
  so a host's MISO and a client's SCK, MOSI and NSS have no field, and a
  class named for one does not compile (two negative TUs).
- **The select as a hardware output** - `claim_nss_pad()` hands it at
  `very_high` and has no field: no program drives SSOE on a wire (the
  gap list).
- **The erratum's table** - `spi_errata_apb_ceiling_hz()` and
  `within_errata_ceiling()`: `init()` accepts a class below the
  erratum's for its APB clock, as the verb does, and the predicate
  answers - letter l of the F446 suite stages exactly that.
- **The I/O compensation cell** (RM0390 8.1, `Syscfg::compensation_cell`)
  - DECLINED in the driver: a chip-wide measure for the supply's
  switching noise, the application's to own (stm32f4/syscfg.hpp), and on
  this desk's wires it changes nothing measured (Bench findings).

COST, read in the release listings: `init()` stores each class in the
task's state (one byte store) and hands it to the pad's
read-modify-write; `drive_output()` loads the client's once; `start()`,
the pump, the engines and the runs contain no GPIO access but the
select's BSRR store.

ON A BUS OF WIRES THE CLASS IS A CORRECTNESS PARAMETER OF EVERY PAD, and
which class is the BUS's: a fast edge on SCK is seen twice at a far
client, a fast edge on MOSI or MISO couples into the clock beside it, and
either puts the far client's stream one bit out of place from that frame
on. On the desk's jumper wires to the STM32G0B1RE this host runs SCK
and MOSI at `medium` and this board's client answers at `low` (Bench
findings); a printed board's short traces keep the default.

## How to use it

A bus with one device on it, no arbiter, polled - the shape a device
that owns its bus wants:

```cpp
constexpr brio::SpiPins spi5{.sck  = {'F', 7, brio::PinFunction::af5},
                             .miso = {'F', 8, brio::PinFunction::af5},
                             .mosi = {'F', 9, brio::PinFunction::af5}};
using Bus = brio::SpiHost<5, spi5>;
Bus::init(clock);                       // no ceiling: every rate legal

const uint8_t cmd[1] = {0x8F};          // read, register 0x0F
uint8_t who = 0;
Bus::Request r{};
r.cs      = brio::Pin<'C', 1>::ref();
r.cmd     = brio::lend<brio::Lease::reply>(cmd);
r.cmd_len = 1;
r.rx      = brio::lend<brio::Lease::reply>(&who);
r.len     = 1;
r.mode    = brio::SpiMode::mode3;
r.clock   = brio::SpiClock::div16;
r.polled  = true;
(void)Bus::start(r);                    // true: it is done, `who` is filled
```

The same bus on jumper wires, its two driven pads a class or two down -
the wire's property, written once at `init()`:

```cpp
Bus::init(clock, {.max_sck_hz = 10'000'000,
                  .sck_speed = brio::PinSpeed::medium,
                  .mosi_speed = brio::PinSpeed::medium});
```

and a client's answer line the same way:

```cpp
using Peer = brio::SpiClient<5, spi5>;
Peer::init(clock, {.mode = brio::SpiMode::mode0, .miso_speed = brio::PinSpeed::low});
```

An image whose handlers or masked windows run longer than the bench's
declares its own hold-off - here 300 cycles, measured as "The host
above the wire" says - and the pump keeps two frames in flight only
where a frame outlasts it:

```cpp
using Bus = brio::SpiHost<5, spi5, brio::NoDmaEngine, brio::NoDmaEngine, 300>;
static_assert(Bus::write_ahead_min_frame_cycles == 423);
```

The same bus shared, through the arbiter - the client posts and gets a
`SpiDone` back, and the app's ISR glue is the one vendor line:

```cpp
using Arb = brio::SpiBus<Bus, Platform, 4>;
extern "C" void SPI5_IRQHandler() {
    if (Bus::isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
}
r.polled = false;
r.reply  = brio::reply_to<MyAo, brio::SpiDone>();
brio::post<Arb>(r);
```

With the data phase on the DMA streams - the cells are the request
mapping's and the reserve checks them at compile time; an element of
`uint16_t` (DR's width) lets the engines carry 16-bit frames too, and
`uint8_t` keeps those on the pump:

```cpp
using Tx = brio::DmaTxEngine<2, 4, 2, uint16_t>;   // SPI5_TX: DMA2 stream 4, channel 2
using Rx = brio::DmaRxEngine<2, 3, 2, uint16_t>;   // SPI5_RX: DMA2 stream 3, channel 2
using FastBus = brio::SpiHost<5, spi5, Tx, Rx>;
extern "C" void DMA2_Stream4_IRQHandler() { (void)FastBus::dma_isr(); }
extern "C" void DMA2_Stream3_IRQHandler() { (void)FastBus::dma_isr(); }
brio::Dma<2>::init();
FastBus::init(clock);
```

A three-wire device, whose answer comes back on the pad the command
went out on. The pad must be let go for the receive window and taken
back after - the peripheral does not do it:

```cpp
brio::Spi<5>::configure({.mode = brio::SpiMode::mode3,
                         .direction = brio::SpiDirection::half_duplex_out});
brio::Spi<5>::enable();
cs.clear();
brio::Spi<5>::data(0xCF);                       // the command, one line out
while (!brio::Spi<5>::tx_empty()) {}
while (brio::Spi<5>::busy()) {}
Mosi::function(brio::PinFunction::af5,          // let the output stage go
               {.pull = brio::PinPull::up, .open_drain = true});
brio::Spi<5>::bidirectional_output(false);      // the clock starts here
// ... 28.3.8's stop: drop SPE after the second-to-last frame
```

The audio face, as a master transmitter at 48 kHz:

```cpp
constexpr auto pll = brio::plli2s_config_for(8'000'000, 76'800'000, SysClock::pll.m);
brio::Rcc::plli2s_configure(pll);
brio::Rcc::plli2s_enable(true);
brio::Rcc::plli2s_wait(true);
brio::Rcc::i2s_source(brio::I2sSource::plli2s_r);

constexpr auto pre = brio::i2s_prescaler_for(76'800'000, 48'000, false, false);
brio::I2s<3>::bus_clock(true);
brio::I2s<3>::configure({.mode = brio::I2sMode::host_transmit,
                         .standard = brio::I2sStandard::philips,
                         .div = pre->div, .odd = pre->odd});
Ck::function(brio::PinFunction::af6);
Ws::function(brio::PinFunction::af6);
Sd::function(brio::PinFunction::af6);
brio::I2s<3>::enable();
brio::I2s<3>::data(sample);          // and one per TXE from there
```

## Bench findings

`test_stm32f4_spi` on the STM32F429I-DISC1, SPI5 at PCLK2 = 90 MHz with
the board's gyroscope (chip select PC1, four wires, mode 3) and its
display controller (chip select PC2, D/CX PD13) on the same three pads.

- **The reset values are the chapter's but one.** CR1 0x0000, CR2
  0x0000, SR 0x0002, CRCPR 0x0007, I2SCFGR 0x0000 - and I2SPR reads
  **0x0000**, at boot and again behind an RCC reset pulse, where 28.5.9
  states 0x0002. Printed and not judged: one part, one instance.
- **The eight rates, and what the polled pump costs.** SCK 45000, 22500,
  11250, 5625, 2812, 1406, 703 and 351 kHz off PCLK2. Thirty-two frames
  timed at each: the measured frame time exceeds eight bits at the
  stated rate by **241 to 300 ns (43 to 54 core cycles) at every code
  of the ladder** - a constant, which is what says it is the pump's own
  cost (one write, one RXNE spin, one read) and not a share of the
  rate. At PCLK2/256 that cost is 12 parts per thousand of a frame and
  the measurement IS the wire: 23042 ns against the arithmetic's 22755.
- **A mode fault needs no pad and no second master**: SSI driven low
  under software management raises MODF, and the silicon has cleared
  SPE and MSTR with it - the host is a client until the sequence is
  run. **The flag is not up on the bus access that follows the store**:
  it appeared **60 core cycles** later, and the clearing sequence's
  effect landed **46 core cycles** after its CR1 write. A peripheral on
  a half-rate APB answers a beat late, and a handler must read the flag
  rather than assume it from what it just wrote.
- **The overrun's sequence is two reads and both are needed**: three
  frames clocked with nothing reading DR leaves SR at 0x43 (OVR, TXE,
  RXNE); a DR read alone leaves OVR standing; the DR read followed by
  an SR read clears it.
- **The device answers.** WHO_AM_I reads **0xD3** at 5625 kHz in mode 3,
  eight times running, where a register the part does not implement
  reads 0x7E - so the byte is the device's and not the line's.
- **The rate ladder against a real device**: exact at 22500 kHz and
  below; at 45000 kHz exact in three runs of four and 0xFB, 0xDB in the
  fourth (the device's own datasheet stops at 10 MHz, and what an
  over-clocked device returns is its business).
- **Two of the four modes read it**: mode 3 (the datasheet's) and mode
  0, the pair with CPOL and CPHA both flipped. What modes 1 and 2 return
  is printed and not judged - 0xD3 and 0xFF here, a timing accident of
  one specimen.
- **Auto-increment is byte-exact**: five control registers read one at a
  time and the same five in one transaction agree; a register written
  and read back agrees for every pattern; a two-register auto-incremented
  WRITE lands in both.
- **The arbiter is unchanged on this architecture.** Four transactions
  posted from one dispatch come back four `spi_ok` replies with an
  ISR-pumped and a polled one interleaved; a two-phase write-then-read
  Request reads the control block; six posted into a four-deep queue are
  all answered and the surplus rejected; an idle bus votes for a sleep.
- **The DMA engines carry the data phase byte-exact, in ONE
  interrupt.** Against the gyroscope, a pattern planted through the
  polled loop in the registers the device only stores (REFERENCE and the
  interrupt thresholds, 0x25 and 0x32..0x38): an ISR-style read of a
  command and six data frames at 5.625 MHz comes back byte-exact with
  one SPI5 interrupt (the command frame, on the pump) and one stream
  completion (the receive stream's; none from the transmit stream), and
  seven data-only frames come back in one interrupt in all. A polled
  read of 24 frames at 11.25 and at 22.5 MHz - where the polled rule
  sends the phase to the engines, the receive stream's completion seen -
  is byte-exact against the loop's 24 at 5.625 MHz (the device's
  auto-increment rolling over from 0x2D to 0x28).
- **The pump in both shapes against the same device**: a command and
  seven data frames ISR-style, one frame in flight at 22.5 and 11.25 MHz
  and two at 5.625, byte-exact, one interrupt a frame, no overrun
  counted.
- **16-bit frames carry the device's bytes, the high byte first on the
  wire**: four data-only frames read two registers a frame back, stored
  low-first as the Request's rule says, through the polled loop (two in
  flight at 5.625 MHz, one at 22.5), the pump at both rates (one
  interrupt a frame) and the engines with a half-word element at 5.625
  MHz (one interrupt).
- **An engined request costs 3.0 us more than its wire, and ONE
  interrupt.** Measured on the F446's SPI1 at 180 MHz with MISO floating
  (`bench_stm32f4` letter d, the data phase alone, no command frame):
  wall minus the wire's time is 547 to 559 cycles at SCK 22.5 MHz and
  535 to 547 at 5.625 MHz across the round's images (code placement
  moves it by about ten), for 16 frames and for 256 alike, and the core
  is busy 540 cycles of it - `start()`, the two block starts and the requests raised
  in one CR2 store, then one completion handler of 120 cycles (the
  receive block's; the transmit stream is armed for its errors alone,
  because every frame the receive stream took was clocked out first). A
  256-frame request runs at 1.03 x the wire at 22.5 MHz. The same
  request cost 2472 cycles (13.7 us) and two interrupts while each block
  start validated and rebuilt its stream (dma.md), and 606 with the
  Request copied whole into the engine and `apply()` comparing three
  bytes (the SPI round took those 59).
- **The host's own loops against the wire** (letter e, the same SPI1,
  the engineless host, 256 frames, the best of six runs with the
  console's and the tick's handlers live, the worst run and the overrun
  count printed beside it). The transmit-only shape: x = 1.14 at SCK 45
  MHz (37 cycles a turn against a 32-cycle frame), 1.01 at 22.5, 1.00
  at 11.25 and 5.625, and 1.00 to 1.01 in 16-bit frames at every rate.
  The receive shape with one frame in flight: x = 2.33 at 45 MHz, 1.60
  at 22.5, 1.23 at 11.25 (43, 39 and 31 cycles of idle bus a frame: the
  turnaround and the block's restart from an idle shifter; the same loop
  read 1.44 at 22.5 MHz in an image that placed it elsewhere in the
  flash, design/benchmark.md's gap on placement), 1.60 and 1.23 in
  16-bit frames at 45 and 22.5 MHz; with two in flight, from
  5.625 MHz in 8-bit frames and from 11.25 in 16-bit ones, x = 1.00 -
  and no overrun counted in any run, at any rate, in either width. The
  loop as it was - `xfer()` per frame, one frame in flight, four calls a
  frame - ran at x = 2.85 at 22.5 MHz and 1.41 at 5.625 (107 to 119
  cycles a frame over the wire).
- **The pump, one or two frames in flight.** The handler is 95 cycles
  with the bench's stamps (145 before: the Request's `bits` reloaded,
  three calls a frame, the next frame written last). With two in flight
  a 256-frame request runs at x = 1.00 at 5.625 MHz in 8-bit frames and
  from 11.25 in 16-bit ones, 408 cycles over the wire whatever the
  length - the bus never idle - and no overrun counted; with one, below
  the threshold, x = 1.76 at 11.25 MHz, 2.57 at 22.5, 4.28 at 45: the
  bus idle for the handler, as the arithmetic above says it must be.
- **The hold-off's witness** (letter e's `spi.short`): the same
  engineless host declaring a hold-off of one cycle - threshold 50, two
  8-bit frames in flight from /4 where the default keeps one until /16 -
  pumping 256 frames at SCK 22.5 MHz (a 64-cycle frame against the
  pump's own 95-cycle handler) ends every one of six runs in
  `spi_overrun`, the host counting six, while the default host's runs
  at every rate and width beside it count none.
- **THE PUMP READS OVR AFTER THE DATA, and the window that reading it
  before left is closed** (letter e's `spi.window`, the Nucleo-F446RE,
  MISO floating): a pumped write of 2 and of 3 8-bit frames at SCK 22.5
  and 11.25 MHz by the same short host - two in flight - SPI1's line
  masked at the NVIC around `start()` and unmasked after 0 to 127 empty
  turns, four sweeps, so that the handler for frame 0 enters before
  frame 1 completes, after it, and on it. At 22.5 MHz the handler
  cannot enter before frame 1 completes (512 of 512 runs lost it, every
  one completed `spi_overrun`); at 11.25 MHz the boundary sits at the
  sixteenth turn. With OVR tested in a status copy taken BEFORE the DR
  read, 4 two-frame runs of 512 never completed, all at that turn - the
  last frame lost with no interrupt to follow - while the three-frame
  runs completed, the next frame's RXNE rising and its handler finding
  the OVR a frame late. With the SR read after the DR read: 512 of 512
  completed on every line, 453 of them `spi_overrun` with the last
  frame lost and counted, no OVR standing after any; a scratch probe in
  the handler counted the completions that fell between the two reads,
  5 and 3 at 11.25 MHz, every one completed. The second SR read costs
  the handler one or two cycles: 95 to 96 cycles with the stamps
  against 93 to 94 the same session.
- **A short polled request's fixed cost: 198, 251 and 246 cycles** for
  a command byte and 0, 2 and 15 data bytes at 5.625 MHz (wall minus
  the wire's time; D/C and the select on two pads, the data phase in the
  transmit-only shape), where the same requests cost 328, 517 and 1712
  before: the Request copied into the engine, `apply()`'s three bytes,
  four pin calls, a flush, and four calls a frame. Over the ENGINED host
  the same three requests cost 229, 284 and 280 and take the pump, as
  `dma_min_frames` and the polled rule say; and 256 frames polled at 45
  MHz take the engines (x = 1.06, one interrupt) where at 22.5 MHz a
  write takes the loop (1.01) and a read the engines (1.03).
- **The vendor's loop on the same board.** ST's HAL v1.8.5
  `HAL_SPI_TransmitReceive`, its two-lines branch, compiled at -Os with
  brio's crt, clock and console around it on the same pads and BR codes:
  256 frames at x = 2.97 at 22.5 MHz and 1.30 at 5.625 in 8-bit frames,
  1.78 and 1.18 in 16-bit ones, and 482 and 634 cycles over the wire for
  a request of one and three frames - its loop tests TXE and RXNE
  independently as figure 311 does, and calls `HAL_GetTick()` on every
  turn for its timeout. brio's receive shape at the same rates is
  wire-bound where the HAL's is not, and its one-in-flight loop (x =
  1.60 at 22.5 MHz) is under a third of the HAL's distance from the wire; a short
  request costs brio under half the HAL's.
- **The tenure's copy and the listing.** `start()` is 348 instructions
  in all with the three shapes inlined (the receive-two shape out of
  line) and four calls, every one cold or above the threshold; the
  asynchronous path's copy is nine word loads and stores; the polled
  3-byte path executes about 130 instructions and no call, where before
  it executed about 205 with eleven calls. The host's code is 1.6 KB a
  host against 0.5 before - the price of the shapes, and of the write
  loop inlined once per phase.
- **16-bit frames ride the engines**, a half-word a beat out of the
  Request's bytes low-first, when the engines' element is `uint16_t`:
  256 frames in 1.01 x the wire at 22.5 MHz with one interrupt, where
  the pump took one interrupt a frame and 2.35 x the wire. A 16-bit
  request whose buffer is not half-word aligned goes to the pump.
- **The SCK pad's slew class does not decide the answer here.** PCLK2 at
  90 MHz is above ES0206 2.12.4's ceiling at every class, and the
  driver says so; the device nevertheless reads exactly at all four
  classes, `low` included, at SCK 5625 kHz on this board's short
  traces. One board, one load: printed.
- **THE BIDIRECTIONAL LINE NEEDS THE PAD TURNED ROUND BY HAND.**
  Measured three ways against the gyroscope moved to its own three-wire
  interface: with the MOSI pad left an alternate-function PUSH-PULL
  output the read comes back all ones whatever the device drives -
  BIDIOE stops the shifter, not the pad's output stage; with the pad
  moved to a plain INPUT it comes back all ZEROES, because the
  peripheral's input is not taken from there; with the pad kept in its
  alternate function but made OPEN DRAIN with a pull-up the device's
  answer arrives exactly (0xD3 then 0x7E, the same two bytes the
  four-wire path reads). The same one-line write moves the device back,
  and the full-duplex bus is intact after it.
- **The display controller's read path is not reachable on this board.**
  The identity command answers 0xFF on every frame - the pull-up on a
  line nobody drove, through the same transaction that had just read
  the gyroscope, so it is the device and not the driver: the board
  straps the controller's interface mode so that its replies leave on a
  pad the MCU is not wired to.
- **The audio PLL and the word select.** PLLI2S from the same 8 MHz
  crystal, its input divider the MAIN PLL's M (4, so a 2 MHz VCO input),
  N = 192 over R = 5: 76.8 MHz, locked; PLLI2SCFGR refused while it
  runs. I2SDIV 25 with ODD clear puts the arithmetic's 48000 Hz on the
  word select, and the pad's own rising edges counted over 200 ms give
  **47805 to 47950 Hz** - within a per cent, the window's resolution.
  CHSIDE alternates as the frame does.
- **The master clock output** moves the divisor to 256 x (2 x I2SDIV +
  ODD): I2SDIV 3 gives 50000 Hz by the arithmetic and **49835 to 49840
  Hz** measured on the word select, with MCK at 256 x that - 12.8 MHz,
  too fast for a polling loop to count, and its pad read high on 1974 to
  1977 of 4000 samples, which is what says it is a clock and not a
  level.

- **On the STM32F411CE's SPI1 at PCLK2 = 96 MHz, against an ILI9481
  panel on a breadboard: THE INTERRUPT-PUMPED REQUEST AT PCLK2/8 NEEDS
  THE SCK PAD AT `medium` OR `low`.** Polled requests at 12 MHz are
  byte-exact at every slew class, interrupt-pumped ones at 6 MHz too;
  at 12 MHz with the pad at `high` or `very_high` the device takes
  neither the command nor the data of an interrupt-pumped request (a
  block written reads back as what was there before, a read comes back
  all ones), and with the pad at `medium` or `low` every byte lands -
  through the software pump and through the DMA engines alike, whose
  interrupt-style request pumps its command byte the same way. IT IS
  THE WIRE AND NOT THE DRIVER: slowing the MOSI pad alone to `medium`
  with SCK left at `very_high` lands every byte too, the data pattern
  makes no difference (all zeros fail as the pattern does), and with a
  logic analyser's probes hung on the lines every case passes at every
  slew class - the probes' capacitance slows the edges the way the pad
  setting does. The analyser then said which wire: with its probe on
  SCK alone the case still fails and the clock at the connector is
  CLEAN (eight rising edges a byte, 82 to 84 ns periods, no pulse under
  40 ns, no extra edge); with its probe on MOSI alone the case passes.
  MOSI's fast edge is the actor and the damage is beyond the connector
  - in the module's traces or the controller's input - where a logic
  analyser cannot look. ES0287 2.11.4 is not involved (it asks for the
  FASTER pad at a high APB). The rule for a breadboard: at PCLK2/8 keep
  SCK or MOSI at `medium`, and `mosi_speed()` is the lever to pull
  first: no erratum names the data pad, so slowing it costs the errata's
  table nothing, where `sck_speed()` at `medium` lowers the APB ceiling
  2.12.4 allows. Measured through the verb: with MOSI at `medium` and
  SCK at `very_high`, every 12 MHz case of the same probe passes,
  through the pump and through the engines. Why the polled loop survives the
  same edges on long requests and not on short ones was not resolved,
  and does not need to be: the fix is on the pads, or on a printed
  board's short traces.

**`test_stm32f4_spi` on the Nucleo-F446RE**, SPI2 at PCLK1 = 45 MHz
against the STM32G0B1RE it is paired with on the desk (the stm32g0
`spi_peer`, firmware 0x0302, on its SPI1), scores **76 of 76** in `z`:

- **The block**: four instances (SPI1 and SPI4 on APB2, SPI2 and SPI3 on
  APB1), one vector each; the I2S face on SPI1, SPI2 and SPI3 and no
  extension block (table 165, 26.1.3); SPI2's requests on DMA1 stream 4
  and stream 3 channel 0 and on no DMA2 cell. The reset values are the
  chapter's, I2SPR included: 0x0002 as 26.7.9 states, at boot and behind
  an RCC reset pulse, where the F429's SPI5 reads 0x0000. The refusals
  are the F429's, the even polynomial ES0298 2.14.4's.
- **The rates, with the select high**: the eight codes 22500 down to 175
  kHz; the polled one-frame loop costs a CONSTANT TURN after RXNE - 361
  to 373 ns (64 to 67 core cycles: the DR read, the next write, the
  restart, four APB1 accesses at HCLK/4) - visible where it outlasts the
  half bit RXNE leaves (the last sampling edge, 26.3.9), at /2../16, and
  hidden by it from /32 down, where the frame time IS eight bits.
- **THE MODE FAULT BY A SECOND CHIP**: SSI driven low raises MODF 66 core
  cycles after the store, SPE and MSTR cleared; then with NSS a HARDWARE
  INPUT (SSM clear, PB12 handed to SPI2) the peer pulls the shared
  select low from its own pad (`ss_pulse`) and MODF rises on this host,
  SPE and MSTR cleared by the silicon - 26.3.4's multi-master demotion,
  measured. The overrun's DR-then-SR sequence as on the F429.
- **The peer link**: the command channel at PCLK1/256 = 175.8 kHz, ten
  of ten; the four modes and both bit orders byte-exact both ways, a
  DORD mismatch an exact two-way reversal.
- **THE LADDER TO PCLK1/2 = 22.5 MHz**: eight frames byte-exact both
  ways at every code on the polled host against the peer's DMA serve,
  thirty-two on the ENGINED host (DMA1 streams 4 and 3, asynchronous, one
  interrupt a transaction), and the peer's polled client run to 22.5 MHz
  too (informational) - with the MOSI pad at medium (the pads, below);
  at high the engined climb slipped at 11.25 MHz in about one run in
  six.
- **THE ARBITER OVER A SECOND CHIP** (letter q): four transactions
  queued from one dispatch, ISR-pumped at the command rate, come back in
  order, spi_ok, the 24 bytes read back exact by the peer; six into a
  three-deep queue answer two `bus_rejected`; an idle bus votes for a
  sleep and a busy one against it; a completion swallowed is answered
  `spi_timeout` at the per-bus limit and `recover()` puts the engine back
  - the next four run to spi_ok on the same bus AO.
- **THE PUMP'S TWO-IN-FLIGHT OVERWRITE, measured and closed**: before
  `pump_two_ahead()`, letter q's four transactions at PCLK1/256 never
  completed - six frames written, five RXNE interrupts, the arbiter
  answering `spi_timeout` four times - because the handler wrote frame k
  + 2 at frame k's RXNE, half a bit (512 core cycles) before frame k + 1
  left the Tx buffer ("The host above the wire"). With the rule, one
  frame in flight at that rate, every transaction completes.
- **THE ROLES INVERT** (letter r): `SpiClient::exchange()` on SPI2 as
  the client of the peer's `host_burst`, twelve frames at 2 MHz under a
  GPIO-held select, byte-exact both ways, the run ending complete.
- **ES0298 2.14.5 does not reproduce on this wire**: the host's SCK pad
  at medium (75 MHz of APB ceiling in table 5) reads the peer's stream
  exact at 22.5 MHz; at low (25 MHz of ceiling against APB1's 45) it is
  exact at 2.8 MHz too - the load is lighter than the table's 30 pF - and
  at 22.5 MHz the low class fails for another reason (DS10693 table 58:
  4 MHz of fmax at 50 pF), its errors not in the last bit.
- **THE PADS ARE CORRECTNESS PARAMETERS ON THESE WIRES**, walked with
  letter x (ten bursts a mode at the command rate) and letter p:
  SCK at very-high or high is seen twice at the G0's input - x slips 15
  bursts of 40 in every mode at high, p breaks from 351 kHz, and letter r
  of the G0's suite (this board's `host_burst`) slips in every run at
  very-high and in two of four at high - and is exact at medium and at
  low. MOSI's edge slips the far client where the frames run back to
  back: thirty bursts of 32 frames on the DMA host at PCLK1/4 = 11.25
  MHz slip in 6 to 13 of 30 at high and 16 to 23 of 30 at very-high, the
  same signature (both ends wrong from one frame on), and in none of 270
  at medium (PCLK1/2, /4 and /8, polled and engined host, the peer's DMA
  serve and its run) nor of 60 at low; with MOSI at medium SCK at high
  holds that rung too (0 of 30), SCK staying medium for r and x. The
  I/O compensation cell (RM0390 8.1) changes nothing: 10 of 30 at high
  with it on, 16 of 30 at very-high. So the suite hosts at SCK medium,
  MOSI medium, and letter x's second part counts that rung (0 of 30).
  It is the rung letter p's engined climb is judged on: with MOSI at
  high, p ALONE slips there in 2 runs of 12 (and z in 1 of 4) - the pad,
  not a state an earlier letter leaves; at medium p is exact in 12 runs
  of 12 and z in 3 of 3. The far client's
  MISO is the aggressor in modes 1 and 2 (the falling-edge samplers):
  11 of 40 at very-high, 8 and 11 at high, 0 at medium and low - while
  medium costs p's 22.5 MHz rung - which is why the stm32g0 peer answers
  at high in modes 0 and 3 and at medium in 1 and 2 (its header); x is
  then 0 of 40, three runs.

- **16-BIT FRAMES ON ALL THREE OF THE HOST'S ENGINES** (letter `u`,
  spi_link's `spare_wide`): thirty-two 16-bit frames a window, the G0
  serving them in half-word beats, byte-exact both ways at every code to
  PCLK1/2 = 22.5 MHz from the polled loop, the RXNE pump and the
  half-word streams - a frame on the streams one half-word beat, the
  beat the frame - and the G0's own 16-bit client run likewise. Timed
  from two windows of 16 and 48 frames at the same rung (the
  transaction's fixed cost cancels), a frame inside the window takes
  999..1001 permille of its sixteen SCK periods on the streams at
  PCLK1/8, /4 and /2, and 999..1000 on the polled loop at /8 and /4,
  where it keeps two frames in flight ("The host above the wire"); at /2,
  one in flight, the loop's frame is 1531 permille and the pump's 2031.
  The 22.5 MHz rung needed the G0's answer line at very-high (its
  header): at high, 16 and 20 of 20 16-bit windows read wrong in bit 2
  (and bit 15 on the streams) while every 8-bit one held.
- **THE TRANSMIT-ONLY SHAPES, WITNESSED** (letter `v`,
  `spare_receive_only`): 33 frames, 8- and 16-bit, through the polled
  transmit-only loop, the pump's write shape and the streams' (the
  receive side discarded), every one heard exact by the G0 listening
  with its answer line dark - its receive channel, then its `receive()`
  run at 7 and 33 frames - at every code to 22.5 MHz; odd counts both
  ways, 7 and 33 byte frames, against both of the G0's serves.
- **THE CRC AGAINST A FOREIGN UNIT** (letter `w`, `spare_crc`), this
  end's unit by hand on the resource (`SpiHost`'s Request has no CRC
  phase), sixteen data frames, CRC-8 0x07 and CRC-16 0x1021: at every
  code to PCLK1/4 TXCRCR is the bitwise arithmetic of what went and
  RXCRCR of what came, the CRC frame read is the arithmetic of the G0's
  stream, CRCERR at neither end - against the G0's channels managing the
  CRC by themselves (RM0444 35.5.14) and against its CRC run alike. THE
  CHECK IS A REAL ONE: this end's unit off and the CRC frame sent as
  data, its last bit flipped, the G0 reports CRCERR (`report_crcerr`)
  in both widths, on both serves, at PCLK1/256 and PCLK1/4 and on its
  channels at PCLK1/2, every data frame exact.
- **ES0298 2.14.5's CRC SIGNATURE AT PCLK1/2, at every SCK class**: at
  22.5 MHz the far side holds - TXCRCR the arithmetic, the G0's check
  clean, every data frame exact both ways - but THIS END'S RECEIVE CRC
  does not: RXCRCR misses the arithmetic in 10 of 10 windows with the SCK
  pad at medium and at high and in 8 or 9 of 10 at very-high, both
  widths (letter `t`), and with the last data frame queued behind the
  one before, the 8-bit CRC frame itself is read wrong (0x04 for 0x4B)
  where the same frame sent as data reads right. The erratum's own words
  - "the CRCERR flag can be unduly asserted ... if any data under
  checksum, and/or just the CRC pattern is wrongly captured" - with two
  differences measured: the data frames are captured right, and the code
  matters (PCLK1/4 and slower are clean) where the erratum says the
  communication speed has no impact. Letter `w` judges the far side at
  /2 and prints this end's. THE VENDOR'S OWN LOOP READS IT THE SAME WAY
  (ST's HAL v1.8.5 `HAL_SPI_TransmitReceive` with its CRC, below): at
  PCLK1/2 in both widths, at every optimisation level, the call returns
  HAL_ERROR with HAL_SPI_ERROR_CRC in every window - RXCRCR off the
  arithmetic, TXCRCR on it, the G0's check clean, every data frame exact
  - and from PCLK1/4 to PCLK1/256 every level is clean.
- **THE CRCNEXT WINDOW** (letter `t`, outside `z`): one frame in flight,
  the CRCNEXT store walked away from the last data store, the same
  hazard as the STM32G0's (docs/stm32g0/spi.md): a store landing from 33
  to some 60 core cycles before the last data frame's RXNE (as a poll
  first sees it), a window under half a bit wide that widens with the
  bit, corrupts both calculators and the far check (TXCRCR 0x0D where the
  arithmetic is 0xDA, RXCRCR 0x00, the STM32G0's signature to the byte);
  a store before it is clean. A STORE AFTER THE FRAME HAS ENDED IS NOT
  ACTED ON IN ITS SLOT: CRCNEXT stays set, no RXNE comes for a CRC frame,
  this end's accumulators keep the data's CRC - and the G0 client takes
  one more frame only when this end gives up waiting (its run time is
  this end's wait), sums it into its RXCRCR and reports CRCERR; letter
  `t`'s T and R stay true there because no CRC phase ran (the frame its
  walk reads is not the peer's CRC). That region is the one RM0390
  26.4.2's rule ("The CRCNEXT bit must be set before the end of the last
  data frame transaction") excludes; the window before the end is what it
  does not, and ES0298 lists nothing. So the host queues the last data
  frame behind the one before it - the second-to-last into the idle
  shifter, the last on its TXE, CRCNEXT the store after - and the bit
  lands a frame before the window at every code.
- **The vendor's loop on the CRC.** ST's HAL v1.8.5 `HAL_SPI_Init` and
  `HAL_SPI_TransmitReceive` with its CRC, compiled from the pinned
  archive at every optimisation level (-O0, -O1, -O2, -O3, -Os, -Og) into
  a scratch program with brio's crt, clock and console around it,
  hosting on SPI2 against the G0's client: sixteen data frames a window,
  six windows a cell, every code PCLK1/2 to PCLK1/256 in both widths. Its
  loop keeps one frame in flight and sets CRCNEXT by a read-modify-write
  of CR1 after the last DR store and the handle's bookkeeping, which at
  this core's speed lands before the window from PCLK1/4 down: every
  level is clean there. At PCLK1/2 every level reads 2.14.5's signature
  (above), and the -O0 build, in 8-bit frames, gets no RXNE for a CRC
  frame the G0 receives and rejects as promptly as in a clean window: the
  call returns HAL_TIMEOUT with HAL_SPI_ERROR_CRC and CRCERR stands at
  both ends. No unmodified build lands in the window on this board;
  the same sequence with its CRCNEXT moved to the store right after the
  DR store, then walked away from it a loop turn at a time, is clean
  before the window, corrupts both calculators inside it (HAL_ERROR,
  HAL_SPI_ERROR_CRC) and times out past it, at the distances this
  suite's walk finds - so the window is the silicon's, reproduced inside
  the vendor's own sequence, and nothing else in that sequence bears on
  it.

**`SpiClient` against the STM32G0B1RE host** (the stm32f4 `spi_peer` on
this board, firmware 0x0402, `test_stm32_spi` on the G0B1RE, 74 of 74):
a dark client that drives MISO only inside an answer window; every
window ONE CALL of the run - `answer()` for the protocol's answer windows,
`exchange()` (or the DMA1 serve, its default) for an exchange - the four
transfer modes and both bit orders byte-exact both ways, four arbitrated
transactions in one select window read back exact. ES0298 2.14.6's BSY
is never looked at - the NSS pad is the window. THE ANSWER LINE'S SLEW
CLASS IS A CORRECTNESS PARAMETER here too, walked against the host's
slip statistics: 12 of 40 slip at `very_high`, 7 at `high` and 12 at
`medium` (the falling-edge modes), 0 at `low`, the class the app keeps -
and the ladder is exact to 16 MHz at low ("The client run"). AS THE BUS
HOST (`host_burst`, twelve frames at PCLK1/32 = 1.406 MHz to the G0's
client) the peer drives SCK at MEDIUM: at very-high the far client's
stream went one bit out of place from a random frame in 12 runs of 12
(an edge gained or lost; the ground moved onto the chips' own pins
changed nothing), at high in 2 of 4, at medium in none of 14, at low in
none of 4.
THE EXTENDED EXCHANGE FROM THAT SIDE (its letters `u`, `v`, `w`, every
code to 16 MHz judged, 32 MHz printed): the peer's streams in half-word
beats and its 16-bit `exchange()` run exact both ways, the answers at 32
MHz all wrong while every frame is heard (the 8-bit signature); its
receive stream (33 frames) and its `receive()` run (7 and 33) hearing
every frame of the G0's transmit-only shapes in both widths, to 32 MHz;
the CRC
through 26.4.2's "CRC transfer managed by DMA" (both streams counting
the data frames, the CRC frame read from DR) and through the peer's CRC
run both clean in both widths - and THIS UNIT CATCHES A CORRUPTED WORD:
the G0's CRC frame sent as data with its last bit flipped raises CRCERR
here (`report_crcerr`) on both serves, both widths, at PCLK/256 and
PCLK/4, every data frame exact.

## Not covered yet

Driver gaps:
- No CRC phase in `SpiHost`'s Request or in `SpiClient`'s runs: the unit
  is the resource's (`crc_polynomial`, `crc_enable`, `crc_next`, the two
  accumulators, CRCERR), measured on the wire by hand from both ends
  (letters `w` and `t`, the stm32f4 `spi_peer`); a CRC phase in the tasks
  - the last data frame queued behind the one before and CRCNEXT after
  it, or the streams' own "CRC transfer managed by DMA" - is born with
  the first device that checksums its frames.
- TI frame format: the configuration is written and refused where the
  chapter says it must be, but nothing on this board frames its
  transactions that way. Born with its first device.
- No DMA engine slots on `SpiClient`: a program that wants its client
  on the streams binds them raw (the stm32f4 `spi_peer` does, "The
  client run"); a slot pair is born with a device-shaped client.
- The I2S beyond a master transmitter: no codec on the board, so
  reception, the slave modes, the full-duplex extension block on the
  wire (the F42x/F43x class's: the F446 has none, RM0390 26.1.3 "only
  transmitter or receiver") and the DMA-fed audio stream have no peer. The extension block's
  refusals and its register surface are exercised; its data path is not.
- The I2S clock sources other than the audio PLL's R output: the
  I2S_CKIN pad has no clock on it, and the F446's four-way selector
  (with the main PLL's R and the root among them) is a part this
  stratum has one of, on another desk position.
- A display driver over this bus: the board's controller is a peer whose
  reads cannot answer, and what a display wants above the transaction
  descriptor is the first portable example's business and not this
  chapter's.

Implemented, not bench-verified (each with what would measure it):
- A run without an NSS pad (a client under SSM): compiled on every
  header, never driven - every peer op frames on the pad; a peer
  configuration under SSM would measure it.
- The instances other than SPI5, SPI2 and SPI1 (SPI3, SPI4, SPI6 -
  compiled on every header that has them, none driven): a wire between
  two of them, or a device on one. SPI1 is driven on the STM32F411CE
  against a display and a touch controller (the finding above).
- The receive-only and half-duplex-in configurations as a Request's
  `direction`: the engine's transactions are full duplex by
  construction, and the resource's simplex modes were driven by hand in
  the bidirectional letter alone.
- A stall ending a transaction with `spi_stalled`: the block always
  clocks on this bench; the gate closed under a polled request would
  measure it.
- A hold-off declared HONESTLY above the default: an image binding a
  handler longer than 150 cycles beside the pump at /16 (two in flight
  by the default, a 256-cycle frame) would count overruns with the
  default and none with its own figure declared - a timer handler of a
  few hundred cycles at a rate that keeps landing it in the frames is
  that bench line. The too-short side is measured (`spi.short`).
- The DCS link's commands and pixel rows over the transmit-only shape
  against a panel: the black pill's display (the breadboard finding)
  wants a run of its probe over this host.
- The hardware NSS output (SSOE) as the engine's select: the bus AO's
  select is a GPIO on purpose; the hardware INPUT is measured through
  `claim_nss_pad()` (letter d's demotion), the output is not.
- `rebase()` and the SCK ceiling across a clock change: this family has
  no dynamic clock yet ([clock.md](clock.md)).
- The error interrupt (ERRIE) and the transmit interrupt (TXEIE): the
  engine arms RXNE alone, and the errors were measured by polling.
- The frame error (FRE): it is a TI slave's and an I2S slave's, and
  neither role has a peer here.
