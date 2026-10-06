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
has none, on a part class that is not known, and an extension block that
does not exist. Bench: `test_stm32f4_spi` on the STM32F429I-DISC1,
against the gyroscope and the display controller the board carries on
SPI5; on the Nucleo-F446RE's SPI1 with MISO floating, the engined
request's cost (letter d of `bench_stm32f4`) and the host's own loops
against the wire and against the vendor's (letter e).

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
  driver hands over goes out at `very_high`.

**The audio face is one bit of I2SCFGR**, and which instances have it
is the manual's memory map and not the header - every SPI_TypeDef of
the pack declares I2SCFGR and I2SPR, on instances the manual gives no
I2S at all. It is SPI2 and SPI3 on the F405 class, the F42x/F43x class
and the F446; every instance from SPI1 to SPI5 on the F411. Full duplex
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
max_sck_hz)`, `rebase(sysclk)`, `clock_for`, `sck_hz`, `max_sck_hz`,
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
completion the transaction's one interrupt - and this stratum's three
of its own: `sck_speed` and
`errata_apb_ceiling_hz`/`within_errata_ceiling`, because on this
family the SCK pad's slew class is a correctness parameter and not a
taste, and `mosi_speed`, because on a bus of wires the data pad's edge
is one too and no erratum speaks for it (the breadboard finding
below).

`SpiClient<n, pins>` - a polled surface with an ISR body: `init`,
`enable(first)`, `disable`, `write`, `writable`, `poll`, `selected`,
`select`, `drive_output` (the dark listener), the error verbs,
`isr()`, `release`, and `frames_ahead` = 1, this silicon's answer to
the one number a portable client would need.

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
reads frame k FIRST - the SR read, the RXNE and OVR tests, the DR read:
five instructions - and writes the next frame owed; with two in flight
frame k + 1 is already shifting and the write is k + 2, so the bus stays
busy through the handler. Two in flight overruns the receive buffer if
the read comes later than one frame time after RXNE, and the latest it
can come is THE IMAGE'S HOLD-OFF - the longest it keeps this vector from
running: a handler that started just before RXNE running its whole body
(no interrupt nests over another) or a masked window as long - then its
exit, this vector's entry and its read:

    write_ahead_min_frame_cycles = (hold_off_cycles + 2 x spi_isr_entry_cycles + spi_isr_to_read_cycles) x 5 / 4
                                 = (150 + 24 + 16) x 5 / 4 = 237 HCLK cycles at the default

with the inputs counted in the listing: the Cortex-M4's exception entry
and exit 12 cycles each at zero wait states (the Cortex-M4 TRM, DDI
0439B 3.9.1), the handler's five instructions with two APB2 accesses at
HCLK/2 about 16, and a quarter on top for the flash's wait states.

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
idle for the handler; a hold-off of 300 makes the threshold 425 and moves
both a code slower, /32 and /16 (pinned beside the default's in the
header). `two_in_flight()` answers the same arithmetic for a program. The D/C boundary drains to one frame either
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

An image whose handlers or masked windows run longer than the bench's
declares its own hold-off - here 300 cycles, measured as "The host
above the wire" says - and the pump keeps two frames in flight only
where a frame outlasts it:

```cpp
using Bus = brio::SpiHost<5, spi5, brio::NoDmaEngine, brio::NoDmaEngine, 300>;
static_assert(Bus::write_ahead_min_frame_cycles == 425);
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
  The receive shape with one frame in flight: x = 2.14 at 45 MHz, 1.44
  at 22.5, 1.19 at 11.25 (37, 29 and 25 cycles of idle bus a frame: the
  turnaround and the block's restart from an idle shifter), 1.63 and
  1.28 in 16-bit frames at 45 and 22.5 MHz; with two in flight, from
  5.625 MHz in 8-bit frames and from 11.25 in 16-bit ones, x = 1.00 -
  and no overrun counted in any run, at any rate, in either width. The
  loop as it was - `xfer()` per frame, one frame in flight, four calls a
  frame - ran at x = 2.85 at 22.5 MHz and 1.41 at 5.625 (107 to 119
  cycles a frame over the wire).
- **The pump, one or two frames in flight.** The handler is 94 cycles
  with the bench's stamps (145 before: the Request's `bits` reloaded,
  three calls a frame, the next frame written last). With two in flight
  a 256-frame request runs at x = 1.00 at 5.625 MHz in 8-bit frames and
  from 11.25 in 16-bit ones, 408 cycles over the wire whatever the
  length - the bus never idle - and no overrun counted; with one, below
  the threshold, x = 1.76 at 11.25 MHz, 2.57 at 22.5, 4.28 at 45: the
  bus idle for the handler, as the arithmetic above says it must be.
- **The hold-off's witness** (letter e's `spi.short`): the same
  engineless host declaring a hold-off of one cycle - threshold 51, two
  8-bit frames in flight from /4 where the default keeps one until /16 -
  pumping 256 frames at SCK 22.5 MHz (a 64-cycle frame against the
  pump's own 94-cycle handler) ends every one of six runs in
  `spi_overrun`, the host counting six, while the default host's runs
  at every rate and width beside it count none.
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
  1.44 at 22.5 MHz) is half the HAL's distance from the wire; a short
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

## Not covered yet

Driver gaps:
- The CRC unit beyond its verbs and the even-polynomial refusal: no
  device on the bench speaks a CRC-checked SPI, and a CRC measured
  against nothing is a register read-back. What would close it is a peer
  board running `SpiClient` with the same polynomial - the AVR and the
  SAM strata's suites do exactly that over a wire this board has not
  got.
- TI frame format: the configuration is written and refused where the
  chapter says it must be, but nothing on this board frames its
  transactions that way. Born with its first device.
- **`SpiClient` is driven on SPI2 against a second board.** The peer
  instrument this stratum carries answers a far board's host ONE FRAME
  AHEAD over PB12..PB15 at 281.25 kHz of SCK: a dark client that drives
  MISO only inside an answer window, the four transfer modes and both
  bit orders byte-exact in both directions, a client that never drains
  keeping ONE frame and raising the overrun, the roles inverted with
  this board clocking the far one as host, and a ten-second stress of
  23 exchanges with no error at either end. ES0206 2.12.5's BSY is
  never looked at on this side - RXNE is the witness. And THE ANSWER
  LINE'S SLEW CLASS IS A CORRECTNESS PARAMETER: at the driver's
  `very_high` the far board's falling-edge modes slip and an engined
  exchange comes back nine bytes of sixteen wrong; at `low`, with the
  far board's own pads left at its driver's fastest, everything is
  byte-exact and the rate ladder is unchanged.
- The I2S beyond a master transmitter: no codec on the board, so
  reception, the slave modes, the full-duplex extension block on the
  wire and the DMA-fed audio stream have no peer. The extension block's
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
- The hardware NSS arrangements as the ENGINE's select (`claim_nss_pad`,
  SSOE, the hardware input): the bus AO's select is a GPIO on purpose,
  and a multi-master bus is what would exercise the input.
- `recover()` after a wedged transaction, and the arbiter's per-bus
  timeout with it: a lost completion has to be staged, as the other
  strata's suites stage it, and that wants a second device to hold the
  bus.
- `rebase()` and the SCK ceiling across a clock change: this family has
  no dynamic clock yet ([clock.md](clock.md)).
- The error interrupt (ERRIE) and the transmit interrupt (TXEIE): the
  engine arms RXNE alone, and the errors were measured by polling.
- The frame error (FRE): it is a TI slave's and an I2S slave's, and
  neither role has a peer here.
