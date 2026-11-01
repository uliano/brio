# SPI and I2S - one block, two faces (STM32F4)

This document is for a programmer who needs an SPI bus on an STM32F4: a
sensor, a display controller, a memory or another chip on three or four
wires, with this chip as the host that drives the clock or as the client
that answers it - or, on the instances that have it, an I2S audio
stream. `Spi<n>` is the instance in its SPI face, one function per
register fact, and `I2s<n>` the same block in its audio face.
`SpiHost<n, pins, ...>`, the host engine, is the driver over it: it runs
one transaction at a time - a chip select, an optional command phase, a data
phase - either polled inside the call or on its DMA engines (the plural
always means the two DMA streams bound to the host), and it is what the
shared SPI bus of
[design/spi-bus.md](../design/spi-bus.md) drives. That bus is an
arbiter: an active object (AO: a component of a brio program with its
own event queue, served one event at a time by the kernel,
[docs/learn/introduction.md](../learn/introduction.md)) that takes the
requests of the active objects sharing the wire, runs them one after
another on the engine and replies to each when its transaction is done.
`SpiClient<n, pins>` is the other end of the wire, a polled surface.

The smallest use, the first block of "How to use it" below: a bus with
one device on it, no arbiter, polled (the STM32F429I-DISC1's SPI5). An
excerpt: `clock` is the program's clock type, already initialized, and the
lines from `const uint8_t cmd` down run in the program's main context:

```cpp
constexpr brio::SpiPins spi5{.sck  = {'F', 7, brio::PinFunction::af5},
                             .miso = {'F', 8, brio::PinFunction::af5},
                             .mosi = {'F', 9, brio::PinFunction::af5}};
using Bus = brio::SpiHost<5, spi5>;
using Cs  = brio::Pin<'C', 1>;

Cs::output(true);                       // the select, parked high
(void)Bus::init(clock);                 // no ceiling: nothing for it to refuse

const uint8_t cmd[1] = {0x8F};          // read, register 0x0F
uint8_t who = 0;
Bus::Request r{};
r.cs      = Cs::ref();
r.cmd     = brio::lend<brio::Lease::reply>(cmd);
r.cmd_len = 1;
r.rx      = brio::lend<brio::Lease::reply>(&who);
r.len     = 1;
r.mode    = brio::SpiMode::mode3;
r.clock   = brio::SpiClock::div16;
r.polled  = true;
(void)Bus::start(r);                    // a polled request is always done on return
const bool good = Bus::status() == brio::spi_ok;   // only then is `who` the device's
```

Line by line: `spi5` names the three pads with the alternate function
the datasheet gives SPI5 on them (AF5), which nothing in the code can
check; `Bus` is the engine over SPI5 on those pads, with no DMA engines;
`init()` takes the clock type's tag (`constexpr SysClock clock;`,
[clock.md](clock.md)) and, with no ceiling named, accepts every rate.
The `Request` describes the transaction: `cs` the select pad as a
`PinRef` (a port and a mask the engine drives low around the
transaction), `cmd` and `cmd_len` the command phase, `rx` and `len` the
data phase's input buffer and its length in frames (no `tx`: the data
phase clocks out 0xFF), `mode` the SPI mode, `clock` the rate as a
division of the instance's own APB clock (`div16`: SPI5 is on APB2, so
90 MHz over 16 at a 180 MHz core) and `polled` the way to run it.
`brio::lend<brio::Lease::reply>` marks each buffer as lent to the
engine until the transaction ends ([design/kernel.md, "Payloads: what
travels inside the event"](../design/kernel.md#4-payloads-what-travels-inside-the-event-kernelborrowedhpp)).
A polled request runs inside `start()`, which returns true because the
transaction has ended when it returns: that it ended, not how, which is
what `status()` says.

What decides correctness even there. The select pad must already be an
output parked high: `start()` drives it only through BSRR stores, so a pad
still in its reset state, an input, never selects the device. `init()` after
the clock's: it reads the bus rate from the clock type and not from the
hardware, and the ceiling and the select's setup time are computed from that
rate; a nonzero `cs_setup_us` also needs SysTick's counter running, since with
it stopped the delay returns at once and the setup time is skipped in silence
(`cortexm/delay.hpp`). And "it is done" is not "it succeeded": `start()`
returns true on a polled request that stalled too (a flag the loop waited for
never rose within its budget), with the peripheral reset and `status()`
answering `spi_stalled` instead of `spi_ok`, so a program that must trust `who`
reads `status()`, as the block does.

Two more decide correctness before the first frame. **On a bus of wires the
slew class of every driven pad is a correctness parameter**: the default,
`very_high`, puts a far client one bit out of place ("The pads"), so name the
classes in the `Config` `init()` takes. And at a 180 MHz core PCLK2 is 90 MHz,
above ES0206 2.12.4's ceiling of 84 MHz at every pad class, so on an APB2
instance the erratum no longer promises the last received bit: a core at 168
MHz, or an APB1 instance at 45 MHz, stays inside it, and
`within_errata_ceiling()` answers for the bus as it runs. This example above
the ceiling read its device exact on the board's short traces (Bench findings);
that is one board and one load.

A polled request runs an explicit loop at every length, on a host with DMA
engines too: it arms no interrupt, of the instance or of a stream, so **it
needs no interrupt vector bound**, and its loop waits with interrupts enabled;
the buffers are read and written inside `start()` and are the caller's again
when it returns. **It holds the core for its whole wire time**: under the
cooperative kernel (`Tenuto`) every other active object waits with it, only
handlers running, and a stall holds them for the budget (400 000 spins and 4096
a frame: 781.7 ms on a request of 2048 frames at PCLK1/256). Through the
arbiter a request is posted instead (the block "The same bus shared, through the
arbiter" below): the post returns at once, the loop runs later in the arbiter's
dispatch, and the buffers are lent from the post until the `SpiDone` reply
arrives, untouched and in scope until then, which nothing checks. A request that
is not polled is asynchronous, and asynchronous is DMA: the host names its two DMA
engines, `Dma<n>::init()` comes before the host's `init()`, the streams' NVIC lines
are opened when the host arms them, and the program binds the two streams' vectors
to `Bus::dma_isr()`, whose true answer is the edge the glue posts `TransferDone` on;
on a host with no engines such a request is refused (`spi_rejected`, nothing on the
wire), and one too short to repay the engines runs polled inside `start()`. Ask
asynchronously when the other objects must run through the transfer, or when a long
read wants the wire's rate.

The page is long because the block has two faces and two ends. In
order: the documents of record; what the silicon does; the types and
verbs; the host above the wire (what a transaction costs beyond its
frames, the polled loop's one frame in flight and what it costs, the
asynchronous way and the short request); the
client run; the pads and their slew classes; how to use it, one example
per use; the bench findings, most of the page, on the STM32F429I-DISC1
and against a second chip on the Nucleo-F446RE; and what is not
covered.

## Documents of record

RM0090 Rev 22 ch. 28 (RM0390 Rev 6 ch. 26 and
RM0383 Rev 4 ch. 20 are its twins, and a section cited here by number alone is
RM0090's when it starts 28 and RM0390's when it starts 26: one register description, three
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
audio and request-mapping facts (`stm32f4/device_tables.hpp`, the
reserve: the table of what each part has, read off the device header
where the header can say it and keyed on the part class where only the
manual can), with
the audio PLL and the I2S clock selector on `Rcc`
([clock.md](clock.md)) and the streams from [dma.md](dma.md). The
family fixture is `test/family_stm32f4/spi.cpp` with the negatives
that refuse an absent instance, one pad twice, a host with no MOSI, a
pad on an absent port, an engine off the request map, an engine on a
part class whose manual was not read, one engine of two, two engines
on one stream, engines a word wide, the audio face on an instance that
has none, on a part class that is not known, an extension block that
does not exist, a client run of 32-bit words, and a slew class named
for a pad the host or client does not drive. Bench:
`test_stm32f4_spi` on the STM32F429I-DISC1, against the gyroscope and
the display controller the board carries on SPI5, and on the
Nucleo-F446RE's SPI2 against the STM32G0B1RE it is paired with on the
desk (that board's `spi_peer`); the stm32f4 `spi_peer` on the same SPI2
is the G0B1RE suite's client; on the Nucleo-F446RE's SPI1 with MISO
floating, the engined request's cost (letter d of `bench_stm32f4`) and
the host's own loops against the wire and against the vendor's (letter
e). A letter is one command key of a suite's console (or of the
benchmark's) and the group of verdicts it runs; `z` is the key that
runs a suite's whole set, and a letter outside `z` is run by its own
key.

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
on APB2 and SPI2 and SPI3 on APB1, so BR[2:0] divides its bus's clock
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
GPIO select needs; a host under it must hold SSI high, because a low
SSI on a master is the mode fault. SSOE drives the pad low while SPE is
set and gives up multi-master. The hardware input is the multi-master
arrangement, where another master pulling NSS low demotes this one.

**The flags are cleared by sequences, not by a clear register**
(28.3.10): the overrun by reading DR and then SR - the DR read alone
does not do it; the mode fault by an SR access while MODF stands and
then a CR1 write; the CRC error by writing a zero into it, the one
rc_w0 bit of the register; the TI framing error and the I2S
desynchronization by reading SR; the I2S underrun by reading SR.

**Four errata are live on every part with a manual on the desk**, and
three of them are answered as code:
- *BSY may stay high when the SPI is disabled* (2.12.1). The disable
  procedure waits for TXE and then for BSY in the transmitting
  configurations, and in master receive-only - RXONLY, or bidirectional
  with the output off - it does not look at BSY at all: the errata's own
  instruction, and 28.3.7 already says the flag is kept low there.
- *Anticipated communication upon a transit from slave receiver to
  master* (2.12.2): the clock starts on the MSTR write even with SPE
  clear. `configure()` pulses the RCC reset line before writing a host
  configuration over a client receive-only one - the errata's first
  workaround, free on the ordinary path.
- *A wrong CRC when the polynomial is even* (2.12.3): an even
  polynomial is refused, so the unit is never armed with a divisor that
  computes nonsense. The reset value 0x0007 is odd.
- *The last received bit may be wrong when the SCK feedback is slow*
  (2.12.4). The internal loop back from the SCK pad must return inside
  two APB periods, and the errata's table gives the ceiling per pad
  class at a 30 pF load: 84 MHz of APB at `high` or `very_high`, 75 at
  `medium`, 25 at `low` - halved for I2S. It is a pad fact and not a
  rate fact: the SPI's own bit rate does not enter it. Every pad this
  driver hands over goes out at `very_high` unless the host's or client's
  `Config` names another class ("The pads").

**The audio face is one bit of I2SCFGR**, and which instances have it
is the manual's memory map and not the header - every SPI_TypeDef of
the pack declares I2SCFGR and I2SPR, on instances the manual gives no
I2S at all. It is SPI2 and SPI3 on the F405 class and the F42x/F43x
class; SPI1, SPI2 and SPI3 on the F446 (RM0390 table 165 - SPI4 has
none); every instance from SPI1 to SPI5 on the F411. Full duplex
is a second block on the classes that have one (I2S2ext, I2S3ext -
always slaves, on the instance's own clock and word select) and two
whole instances paired on the F446, which has none.

**The I2S kernel clock is not the APB's**: the audio PLL's R output, an
external clock on the I2S_CKIN pad, and on the parts with the
RCC_DCKCFGR pair the main PLL's R or the root itself. The pair is
numbered by bus and not by instance - I2S1SRC is the APB1 instances'
and I2S2SRC the APB2 ones' (RM0390 6.3.24) - a trap the names invite.
On the F42x/F43x class the audio PLL has no input divider of its own
and shares the main PLL's M, so its VCO input is whatever the system
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

`SpiHost<n, pins, TxEngine, RxEngine>` - the engine
[the shared SPI bus](../design/spi-bus.md) drives, with the fields of
the shared `Request` by name (that page's "Realizations") and the rate
and frame size in this family's types, in this family's order: `cs`,
`dc`, `cmd`, `tx`, `rx`, `len`, `cmd_len`, `bits`, `polled`,
`cs_setup_us`, `clock`, `mode`, `reply` - in that order (a designated
initializer follows it), 40 bytes with no padding, its head - `cs` to
`bits` - being what a request that goes to the engines keeps for its
tenure (its time on the bus).

The verbs: `init(clock, max_sck_hz)` (the shared surface) and `init(clock,
Config)` - `Config` being `max_sck_hz`, `sck_speed` and `mosi_speed`, the
classes of the two pads a host drives ("The pads") - `rebase(sysclk)`,
`clock_for`, `sck_hz`, `max_sck_hz`, `ceiling_clock`, `reference_hz`
(the instance's APB clock), `prime`, `bit_order`, `lsb_first`, `start`,
`dma_isr` (the streams' interrupt body; the host arms no interrupt of
the peripheral and binds no vector of its own), `status`,
`async_min_frames(clock, bits, read)` (the short request's frame count on the
bus as it runs, "The host above the wire"), `recover`, `release`,
`claim_nss_pad`. The constants a program reads: `has_dma` (whether engines are
bound: an asynchronous request needs them, `static_assert(Bus::has_dma)`),
`costs` with `async_read_budget_cycles` and `async_write_budget_cycles`. The
arithmetic behind them is at namespace level: `spi_frame_hclk_cycles()`,
`SpiHostCosts` and `spi_costs`, `spi_polled_frame_cost()`,
`spi_async_budget()`, `spi_async_min_frames()`.

`status` answers one of four, and each has what follows it
([design/spi-bus.md, "What follows each code"](../design/spi-bus.md#what-follows-each-code-and-who-acts)):
`spi_ok`; `spi_rejected`, an asynchronous request on a host with no engines,
nothing on the wire and nothing to do but change the program; `spi_stalled`, the
peripheral already reset and reconfigured, so the next request needs nothing of the
program; `spi_dma_fault`, the requests dropped, both streams put away, the receive buffer read and
the select released, but the peripheral not reset and a frame still on the wire can land
behind that read, so a program calls `recover()` with the bus idle before the next request
(no letter provokes one: "Not covered yet"). The arbiter calls `recover()` itself on
`spi_timeout` and on nothing else.

With engines, the transmit stream is armed for its errors alone and the receive
block's completion is the DMA block's one interrupt. **The receive stream arms at
`very_high` and the transmit at `high`** (docs/design/dma.md's rule): the receive
buffer holds one frame, and a receive stream the transmit one outran finds it full -
OVR, "all other subsequently transmitted bytes are lost" (RM0090 28.3.10) - where a
waiting transmit only stops the clock between two frames. `release()` stops both
streams, disables the peripheral and pulses its RCC reset before the gate, the release
contract (a raised request is held through a gated clock on this family,
[dma.md](dma.md)).

Three verbs are this stratum's own: `sck_speed` and
`errata_apb_ceiling_hz`/`within_errata_ceiling`, because on this family the SCK pad's
slew class is a correctness parameter and not a taste, and `mosi_speed`, because on a
bus of wires the data pad's edge is one too and no erratum speaks for it (the
breadboard finding below). Each speed verb moves between transactions the class `init()`
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
so a request that completes inside `start()` is read through the
reference and copied nowhere, and one that goes to the engines has what
its tenure needs copied - the two pins, the three buffers, the two
lengths and the width, laid out contiguously at the head of the Request
- as inline loads and stores, no call. The rate, the mode, the setup
time and the reply are spent before `start()` returns.

**`apply()` is one compare.** CR1's CPOL and CPHA are the mode's two
bits in the mode's own order, BR the rate's three at bit 3 and DFF the
width's at bit 11 (26.7.1), so the request's three fields fold into the
CR1 word with two shifts and two ors and are compared with the word
applied - 13 instructions on the unchanged path, the ceiling's clamp 4
more - and only a change pays 26.3.10's disable (TXE, then BSY) and the
enable, DFF being enable-protected and BR, CPOL and CPHA "not to be
changed when communication is ongoing".

**The select and D/C edges are one BSRR store each**, the null D/C a
predictable branch, inline through the family's own `PinRef::set()` and
`clear()` (pin.hpp forces the two verbs inline: left to -Os they are
calls, four a transaction). The setup time is tested before any call. **The receive buffer is not
flushed per request**: the receive loop and the receive stream read every
frame they clock and the transmit-only loop clears what stands at its tail, so
nothing stands in it when a transaction ends well. A stall, `spi_dma_fault` and
`recover()` flush it themselves, `init()` once ("Types and verbs").

**Two polled shapes, by the request's fields.** A request with `polled`
set runs one of them at every length and on every host, and never the
DMA engines. A handler can land on a polled loop at any instruction, and
on this platform no interrupt nests, so every handler in the image is a
window the loop cannot see through - the tick's, the console's, the
bench's stamped ones - and under the preemptive kernel so is every
dispatch above the caller. That decides the shapes:

- *Transmit-only* (`rx` null: every command phase, a display's pixels):
  paced on TXE, the answers never read; at the tail the last frame is
  let out of the shifter - TXE, then BSY, 26.3.10 - and the receive side
  cleared by DR then SR, the overrun the unread answers raised included
  (26.3.13: the transmit side is untouched). Nothing is waited for on
  the receive side, so a handler delays the loop and loses nothing. It runs
  at the wire at 22.5 MHz and slower, and at SCK 45 MHz its turn of 38 cycles
  is the bound against the wire's 32: 16 instructions and two APB2 accesses a
  frame.
- *Receive* (`rx` set): one frame in flight at every rate. Nothing is
  queued behind the frame shifting, so nothing can be lost: this host
  owns the clock, the clock waits for the read, and a handler or a
  preemption is a gap on the bus. The turnaround is the minimum - the
  next frame fetched inside the wire time and pinned there with an empty
  asm (the compiler would sink the load past the volatile poll), then
  the DR read and the DR write adjacent.

One budget, 400 000 spins and 4096 a frame, bounds the whole transaction's
spins; when it runs out the peripheral is reset and reconfigured and the
transaction ends with `spi_stalled`. A request that changes the rate, the mode or
the width runs the disable of 26.3.10 first, with bounded waits of its own outside
that budget and an answer the host drops: the budget of the transaction that follows
shows what it left as a stall of its own.

**The transmit buffer's write-ahead (26.3.9, figure 311) is used by the
transmit-only shape and declined by the receive shape.** Storing the next
frame while the previous one shifts keeps the wire continuous, and the
transmit-only shape does it because it reads nothing. A receive shape
doing it would put two frames in flight, and it would overrun the receive
buffer whenever anything held the loop from reading frame k for longer
than one frame time after its RXNE (26.3.13); no bound on the image's
handlers, or on the dispatches above a preempted one, can be enforced
here. The price of declining it is the turn between two frames read,
measured below ("The host's own loops"): at most 68 HCLK cycles a frame
at 180 MHz in the lines run, modes 0 and 1 at /4 to /256. A read that wants the
wire's own rate asks asynchronously, and the streams keep it.

The D/C boundary needs nothing more: the first data frame is written
only after the last command frame came back, because a frame queued in
the transmit buffer goes out the moment the one before it ends.

**Asynchronous is DMA** ([design/spi-bus.md](../design/spi-bus.md),
"Two ways to run a request"). A request whose `polled` is false rides
the host's two engines: `start()` copies what the tenure keeps,
starts both streams and raises both requests in one CR2 store, and
returns false; the receive stream's completion is the one interrupt a
block, and `dma_isr()` answering true is the edge the program's glue
posts `TransferDone` on. The instance's own interrupt is never armed,
and `init()` leaves its NVIC line as it finds it. A host with no engines
REFUSES such a request: `start()` returns true at once with
`spi_rejected`, before the block, the select or the D/C is touched
(measured: the select's pad sees no falling edge, letter `q`), and
`has_dma` says so at compile time. Once a request rides the engines, its
command phase rides a DMA block of its own when it is the whole request or would
repay one (the rule below), the data block launched from the vector once the last
command frame is back and D/C has risen - two interrupts and one
completion; a shorter command phase runs polled inside `start()`, in
the transmit-only shape, before the data block goes to the streams. A
16-bit request the streams cannot carry - over byte engines, or with a
buffer off a half-word - runs polled inside `start()` whichever way it
asked: the streams cannot move it, and a refusal would make the answer
depend on where a buffer happens to sit.

**The short request runs polled.** On the engines a request costs the
core a fixed E - `start()`, the two streams started, the completion
interrupt - whatever its length; polled, it costs a fixed part F and
each frame's time: W + the receive loop's idle bus for a frame read,
max(W, the transmit-only turn) for a frame written, W being the frame's
HCLK cycles at the code, the width and the instance's bus. So an
asynchronous request rides the engines from the smallest count N with
F + N x frame >= E and runs polled below it, completing inside `start()`
- the synchronous completion the arbiter already answers. Only E - F enters, one
for a read and one for a write, and `bench_stm32f4` letter `e` measures each as
one difference. It times an asynchronous request of 64 frames at SCK 5.625 MHz on
the engined host, its least busy run. From that it takes F, found by timing the
polled request through the same call at 16 and at 64 frames, whole, and
extrapolating to no frames - **473 for a read, 443 for a write** in three runs on
this image, 461 and 428 on another whose `start()` differs by two instructions, the
difference being where the flash fetch lands (`spi_costs` holds 460 and 430, and
that much drift moves a count by at most a frame). The same letter's polled lines
on the engineless host give the frame figures, from the difference of 256 and 16
frames: the transmit-only turn **38 cycles** at SCK 45 MHz (the wire's 32; at 22.5
MHz and slower the wire is the bound; `spi_costs` keeps 36 for it, and that moves no
count, (430 + 35) / 36 and (430 + 37) / 38 both being 12), and the receive loop's
idle bus
in the table of "The host's own loops" below, which the host keeps as
**15 bus cycles** (30 HCLK on APB2, 60 on APB1): the DR read, the next
write and the shifter's restart, accesses at the APB's own clock. In the
CPHA = 0 modes that overstates a read at the slower rates, where the half
bit RXNE leaves before the frame's end hides the turn, and it moves no
count: where any of the turn is hidden, in a frame of 256 cycles or longer,
the count with the idle bus is the count with the wire's frame alone. What
the figures give, a number of cells pinned by the header's `static_assert`s and
the rest from the same arithmetic:

| an asynchronous 8-bit request rides the engines from | /2 | /4 | /8 | /16 | /32 and slower |
|---|---|---|---|---|---|
| a read, APB2 (SPI1, SPI4, SPI5, SPI6) | 8 | 5 | 3 | 2 | 1 |
| a write, APB2 | 12 | 7 | 4 | 2 | 1 |
| a read, APB1 (SPI2, SPI3) | 4 | 3 | 2 | 1 | 1 |
| a write, APB1 | 7 | 4 | 2 | 1 | 1 |

`async_min_frames()` answers for the code, the width and the bus in
force: the host computes the two frame costs where the configuration or
the bus's rate changes and never per request, so `start()` pays two
multiplies and a compare on the asynchronous path and nothing on a
polled one. Two cases the rule approximates: a command phase on the engines is
a DMA block judged by the same E, though a block chained from the interrupt costs
less than one `start()` does; and E counts the host's work and not the arbiter's,
whose second dispatch for the completion an asynchronous request pays besides, so
through the arbiter the true edge sits later than `async_min_frames()` (by how much is
not measured here).

So a display command (a DCS command, the display serial interface's command and its
parameters) runs polled inside `start()` on an engined host too, when it is short for
its rate: on APB2 up to 11 bytes at /2, 6 at /4, 3 at /8, one at /16 and none at /32
and slower, where even one byte repays a block. A display's pixel rows, a bulk
read-back and every long block ride the streams.

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
  interrupt client - one that answers on TXEIE alone through the
  resource's `isr()` is measured against the STM32G0B1RE (letter `g`).
- **The DMA requests** (SPI2_TX on DMA1 stream 4 and SPI2_RX on stream 3,
  channel 0 both, RM0390 table 28) - DECLINED in the driver: SpiClient
  has no engine slots (the gap list); a program binds the streams raw, as
  the stm32f4 `spi_peer`'s exchange does, started before SPE in
  26.3.11's order so the transmit stream's first beat is the preload.
- **The overrun** (26.3.13) - `answer()` reads nothing, so OVR rises from
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
side): exact both ways at every code to PCLK/4 = 16 MHz on both serves. The top rung, 32 MHz,
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
  `very_high` and has no field; SSOE is the resource's (`nss_output()`,
  `SpiNss::hardware_output`), measured framing the STM32G0B1RE's client
  (letter `f`, below), and never the engine's select, which is a GPIO the
  Request carries so one host serves many devices.
- **The erratum's table** - `spi_errata_apb_ceiling_hz()` and
  `within_errata_ceiling()`: `init()` accepts a class below the
  erratum's for its APB clock, as the verb does, and the predicate
  answers - letter l of the F446 suite stages exactly that.
- **The I/O compensation cell** (RM0390 8.1, `Syscfg::compensation_cell`)
  - DECLINED in the driver: a chip-wide measure for the supply's
  switching noise, the application's to own (stm32f4/syscfg.hpp), and on
  this desk's wires it changes nothing measured (Bench findings).

COST, read in the release listings: `init()` stores each class in the
driver's state (one byte store) and hands it to the pad's
read-modify-write; `drive_output()` loads the client's once; `start()`,
the polled loops, the engines and the runs contain no GPIO access but
the select's BSRR store.

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
using Cs  = brio::Pin<'C', 1>;

Cs::output(true);                       // the select, parked high
(void)Bus::init(clock);                 // no ceiling: nothing for it to refuse

const uint8_t cmd[1] = {0x8F};          // read, register 0x0F
uint8_t who = 0;
Bus::Request r{};
r.cs      = Cs::ref();
r.cmd     = brio::lend<brio::Lease::reply>(cmd);
r.cmd_len = 1;
r.rx      = brio::lend<brio::Lease::reply>(&who);
r.len     = 1;
r.mode    = brio::SpiMode::mode3;
r.clock   = brio::SpiClock::div16;
r.polled  = true;
(void)Bus::start(r);                    // a polled request is always done on return
const bool good = Bus::status() == brio::spi_ok;   // only then is `who` the device's
```

The same bus on jumper wires, its two driven pads a class or two down -
the wire's property, written once at `init()`. False means the ceiling cannot be
made at this clock, and the bus is not up:

```cpp
const bool up = Bus::init(clock, {.max_sck_hz = 10'000'000,
                                  .sck_speed = brio::PinSpeed::medium,
                                  .mosi_speed = brio::PinSpeed::medium});
```

and a client's answer line the same way. This shows the class alone: a client must
also be selected, so its pins name an `nss` pad, or it takes `SpiNss::software` and
`select(true)`, or it stays deaf (Bench findings, "A client with no select pad"):

```cpp
using Peer = brio::SpiClient<5, spi5>;
const bool up = Peer::init(clock, {.mode = brio::SpiMode::mode0,
                                   .miso_speed = brio::PinSpeed::low});
```

The same bus with its two DMA engines - the cells are the request
mapping's and the reserve checks them at compile time; an element of
`uint16_t` (DR's width) lets the engines carry 16-bit frames too, and
with `uint8_t` a 16-bit request runs polled inside `start()`:

```cpp
using Tx = brio::DmaTxEngine<2, 4, 2, uint16_t>;   // SPI5_TX: DMA2 stream 4, channel 2
using Rx = brio::DmaRxEngine<2, 3, 2, uint16_t>;   // SPI5_RX: DMA2 stream 3, channel 2
using FastBus = brio::SpiHost<5, spi5, Tx, Rx>;
brio::Dma<2>::init();
FastBus::init(clock);
```

and shared, through the arbiter - the client posts and gets a `SpiDone`
back; a request that is not polled is asynchronous, which needs the
engines (`has_dma`), and the app's ISR glue on the two streams' vectors
is the one vendor line. An excerpt. `Platform` is the program's platform type
(`brio::Stm32f4Platform<brio::Ticker>`, say), `MyAo` the client's active object;
`Arb` goes in the list of active objects given to the kernel; `cmd` and `block` are
the client's static storage, since the loan runs past the function that posts:

```cpp
static_assert(FastBus::has_dma);   // an asynchronous request needs the engines
using Arb = brio::SpiBus<FastBus, Platform, 4>;
extern "C" void DMA2_Stream4_IRQHandler() {
    if (FastBus::dma_isr()) { brio::post<Arb>(brio::TransferDone{FastBus::status()}); }
}
extern "C" void DMA2_Stream3_IRQHandler() {
    if (FastBus::dma_isr()) { brio::post<Arb>(brio::TransferDone{FastBus::status()}); }
}

static const uint8_t cmd[1] = {0xE8};   // read from 0x28, auto-incrementing
static uint8_t block[6];

FastBus::Request a{};
a.cs      = Cs::ref();
a.cmd     = brio::lend<brio::Lease::reply>(cmd);
a.cmd_len = 1;
a.rx      = brio::lend<brio::Lease::reply>(block);
a.len     = sizeof block;
a.mode    = brio::SpiMode::mode3;
a.clock   = brio::SpiClock::div16;
a.reply   = brio::reply_to<MyAo, brio::SpiDone>();
brio::post<Arb>(a);                // returns at once; `block` stays lent

// In MyAo's dispatch, on the reply:
//     [](const brio::SpiDone& d) { if (d.status == brio::spi_ok) { /* read `block` */ } }
```

By the table above this request rides the streams: its one command frame is too short
for a block of its own and runs polled inside `start()`, then the six data frames go on
the DMA. With `timeout_ticks` left at its default a channel that never completes leaves
the bus busy for ever: on a bus that must survive it, `brio::SpiBus<FastBus, Platform, 4,
brio::BusPassThrough, ticks>` answers `spi_timeout` at the limit
([design/spi-bus.md](../design/spi-bus.md), "The per-bus timeout").

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

In these findings x is a transaction's duration over the wire's, so larger is
slower; a *turn* is one round of a polled loop; *wall* is the whole time from the
call to its return; a *rung* is one rate of a ladder of rates; a *slip* is a stream
one bit out of place from some frame on; the *peer* is the far board, and its *serve*
the way it answers.

`test_stm32f4_spi` on the STM32F429I-DISC1, SPI5 at PCLK2 = 90 MHz with
the board's gyroscope (chip select PC1, four wires, mode 3) and its
display controller (chip select PC2, D/CX PD13) on the same three pads.
These are the silicon's and the devices' facts; the polled loop's cost and the
asynchronous way on this board are in the second list of "Not covered yet".

- **The reset values are the chapter's but one.** CR1 0x0000, CR2
  0x0000, SR 0x0002, CRCPR 0x0007, I2SCFGR 0x0000 - and I2SPR reads
  **0x0000**, at boot and again behind an RCC reset pulse, where 28.5.9
  states 0x0002. Printed and not judged: one part, one instance.
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
On the Nucleo-F446RE's SPI1 at 180 MHz with MISO floating, the host's costs:

- **An engined request costs some 3.5 us more than its wire, and ONE
  interrupt.** Measured
  (`bench_stm32f4` letter d, the data phase alone, no command frame, the
  request asynchronous and long enough to ride the streams): wall minus
  the wire's time is 673 and 637 cycles at SCK 22.5 MHz and 661 and 625
  at 5.625 MHz in 8-bit frames, 659 to 675 in 16-bit ones, for 16 frames
  and for 256, the same in three runs, and the core is busy 626 to 664 cycles of it -
  `start()` (the settings compared, the short request's decision, the
  tenure's copy, the two block starts and the requests raised in
  one CR2 store), then one completion handler of 156 to 167 cycles with the
  bench's stamps (the receive block's; the transmit stream is armed for
  its errors alone, because every frame the receive stream took was
  clocked out first). A 256-frame request runs at x 1.03 at 22.5 MHz and 1.00 at
  5.625. THE FIGURE MOVES WITH THE IMAGE'S LAYOUT, not with the code: the same path
  reads 601 to 631 in an image whose `start()` is two instructions longer, and 670 to
  700 in one where two nops at `start()`'s end move what follows it; the fetch behind
  the ART is the candidate (not traced).
- **The host's own loops against the wire** (letter e, the same SPI1,
  the engineless host, 256 frames, the best of five runs with the
  console's and the tick's handlers live, the worst run printed beside
  it; three runs of the letter). The transmit-only shape: x = 1.21 at SCK
  45 MHz (38 cycles a turn against a 32-cycle frame), 1.01 at 22.5, 1.00 at
  11.25 and 5.625, and 1.00 to 1.01 in 16-bit frames at every rate. The
  receive shape in mode 0: x = 2.20 at 45 MHz, 1.47 at 22.5, 1.20 at 11.25
  and 1.07 at 5.625 in 8-bit frames, 1.47, 1.30, 1.11 and 1.05 in 16-bit
  ones; the same loop over the engined host, placed elsewhere in the flash,
  reads 2.33 and 1.60 at 45 and 22.5 MHz (design/benchmark.md's gap on
  placement). What the receive loop's one frame in flight costs at every
  rate, the letter's `spi.frame` lines, the receive frame's HCLK cycles over
  the wire's from 256 less 16 frames, each run with interrupts masked, the
  same figures to a tenth of a cycle in three runs of the letter:

  | over the wire, 180 MHz | /4 | /8 | /16 | /32 | /64 and slower |
  |---|---|---|---|---|---|
  | SPI1 (APB2), mode 0, 8-bit | 30 | 26 | 18 | 2 | 0 |
  | SPI1, mode 0, 16-bit | 38 | 30 | 26 | 6 | 0 |
  | SPI1, mode 1, 8-bit | 42 | 38 | 42 | 38 | 38 to 42 |
  | SPI1, mode 1, 16-bit | 38 | 42 | 38 | 42 | 38 to 42 |
  | SPI2 (APB1), mode 0, 8-bit | 44 | 36 | 20 | 0 | 0 |
  | SPI2, mode 0, 16-bit | 56 | 40 | 28 | 0 | 0 |
  | SPI2, mode 1, 8-bit | 64 | 56 | 60 | 68 | 56 to 64 |
  | SPI2, mode 1, 16-bit | 56 | 60 | 68 | 64 | 56 to 68 |

  The wire's frame is 8 bits x the division x HCLK/PCLK: 64 cycles at /4 on
  SPI1 in 8-bit frames, four times that on SPI2 in 16-bit ones. In mode 0
  RXNE rises half a bit before the frame's end (26.3.9), and the turn hides
  in that half bit at the slow rates; in mode 1 the last sampling edge is
  the frame's end, and the turn shows at every rate. At /2, 8-bit frames
  read at 70 cycles on SPI1 against the wire's 32 and 16-bit frames at 94
  against 64. The figures' alternation by 4 cycles from code to code in
  mode 1 was not looked into: on SPI1 it is two cycles of its APB clock.
- **A short polled request's fixed cost: 194, 237 and 247 cycles** for
  a command byte and 0, 2 and 15 data bytes at 5.625 MHz (wall minus
  the wire's time; D/C and the select on two pads, the data phase in the
  transmit-only shape), and 196, 239 and 249 over the engined host, the
  same loops. 256 frames polled at 45 and 22.5 MHz over the engined host
  run the loop too, with no completion (irq 0), a read and a write alike.
- **The vendor's loop on the same board.** ST's HAL v1.8.5
  `HAL_SPI_TransmitReceive`, its two-lines branch, compiled at -Os with
  brio's crt, clock and console around it on the same pads and BR codes:
  256 frames at x = 3.02 at 22.5 MHz and 1.32 at 5.625 in 8-bit frames,
  1.81 and 1.19 in 16-bit ones, and 530 and 748 cycles over the wire for
  a request of one and three frames - its loop tests TXE and RXNE
  independently as figure 311 does, and calls `HAL_GetTick()` on every
  turn for its timeout. brio's receive shape at the same rates is x =
  1.47 at 22.5 MHz (1.60 in the instance over the engined host), under a
  third of the HAL's distance from the wire, and 1.07 at 5.625, a quarter
  of it; a short request costs brio under half the HAL's.
- **The tenure's copy and the listing.** `start()` is 243 instructions
  (652 bytes) over the engined host and 217 (588) over the engineless one,
  both shapes inlined, with three calls on the engineless host and four on the
  engined one, every one cold: the rate's change, the select's setup time, the
  stall and, with engines, the asynchronous path. With
  `polled` set it reaches the loop past one byte test, no engine rule on
  the way; no call in either loop, every field of the Request in a
  register. The width and the null out buffer are tested a frame, in
  registers, and in the receive loop inside the wire's time: the turn runs
  from the RXNE poll's load to the DR store, the DR load between them. The
  asynchronous path's copy is inline loads and stores. A polled 3-byte
  request executes about 135 instructions and no call.
- **16-bit frames ride the engines**, a half-word a beat out of the
  Request's bytes low-first, when the engines' element is `uint16_t`:
  256 frames at x 1.02 at 22.5 MHz with one interrupt. A 16-bit
  request whose buffer is not half-word aligned runs polled inside
  `start()`, whichever way it asked.
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
  panel on a breadboard: AT PCLK2/8 THE MOSI PAD'S EDGE IS THE WIRE'S FAULT.**
  Measured with a probe that wrote each frame of a request, the command byte
  included, from the instance's interrupt, not this host's way of running one (the
  second list says what remains to run). At 6 MHz every slew class is byte-exact. At
  12 MHz with SCK and MOSI at `high` or `very_high` the device takes neither the
  command nor the data (a block written reads back as what was there before, a read
  comes back all ones), and with the pads at `medium` or `low` every byte lands. It
  is the wire and not the driver: slowing the MOSI pad alone to `medium`, SCK left at
  `very_high`, lands every byte too, the data pattern makes no difference, and with a
  logic analyser's probes on the lines every case passes at every class, their
  capacitance slowing the edges as the pad setting does. The analyser then said which
  wire: with its probe on SCK alone the case still fails and the clock at the
  connector is clean (eight rising edges a byte, 82 to 84 ns periods, no pulse under
  40 ns, no extra edge); with its probe on MOSI alone the case passes. MOSI's fast edge
  is the actor and the damage is beyond the connector, in the module's traces or the
  controller's input, where an analyser cannot look. ES0287 2.11.4 is not involved (it
  asks for the faster pad at a high APB). The rule for a breadboard: at PCLK2/8 keep
  SCK or MOSI at `medium`, and `mosi_speed()` is the lever to pull first: no erratum
  names the data pad, so slowing it costs the errata's table nothing, where
  `sck_speed()` at `medium` lowers the APB ceiling 2.12.4 allows. Measured through the
  verb: with MOSI at `medium` and SCK at `very_high`, every 12 MHz case of the same
  probe passes.

**`test_stm32f4_spi` on the Nucleo-F446RE**, SPI2 at PCLK1 = 45 MHz
against the STM32G0B1RE it is paired with on the desk (the stm32g0
`spi_peer`, firmware 0x0302, on its SPI1), runs **97 verdicts** in `z`, all
green in three runs. What they judge and what they print at the top rate: every
read of 16-bit frames and of the CRC is judged to PCLK1/4 = 11.25 MHz and printed
at PCLK1/2 = 22.5 MHz, where this cable's read-back is not exact ("Findings left
open" under "Not covered yet"); what the peer hears, and the 8-bit windows of the
ladder, are judged at PCLK1/2 too:

- **The block**: four instances (SPI1 and SPI4 on APB2, SPI2 and SPI3 on
  APB1), one vector each; the I2S face on SPI1, SPI2 and SPI3 and no
  extension block (table 165, 26.1.3); SPI2's requests on DMA1 stream 4
  and stream 3 channel 0 and on no DMA2 cell. The reset values are the
  chapter's, I2SPR included: 0x0002 as 26.7.9 states, at boot and behind
  an RCC reset pulse, where the F429's SPI5 reads 0x0000. The refusals
  are the F429's, the even polynomial ES0298 2.14.4's.
- **The rates, with the select high**: the eight codes 22500 down to 175
  kHz, timed with the suite's own one-frame loop on the resource, not `SpiHost`'s:
  a constant turn after RXNE of 363 to 364 ns (65 core cycles: the DR read, the next
  write, the restart, four APB1 accesses at HCLK/4), visible where it outlasts the
  half bit RXNE leaves (the last sampling edge, 26.3.9), at /2 to /16, and hidden by
  it at /32 and slower, where the frame time is eight bits. `SpiHost`'s own loop is
  the `spi.frame` table above.
- **THE MODE FAULT BY A SECOND CHIP**: SSI driven low raises MODF 74 to 76 core
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
  queued from one dispatch at the command rate - two asynchronous on the
  streams, one polled inside `start()` and answered in the arbiter's own
  dispatch, one with a three-frame command block chained to its data
  block - come back in order, spi_ok, the 24 bytes read back exact by
  the peer; six into a
  three-deep queue answer two `spi_rejected`; an idle bus votes for a
  sleep and a busy one against it; a completion swallowed is answered
  `spi_timeout` at the per-bus limit, the arbiter calling `recover()` itself
  - the next four run to spi_ok on the same bus AO. AND AN ASYNCHRONOUS
  REQUEST ON THE ENGINELESS HOST IS REFUSED: `start()` answers at once
  with `spi_rejected`, and the select's own pad (PB12, its falling edges
  counted on EXTI line 12) never falls - once for the same request
  polled.
- **THE SHORT ASYNCHRONOUS REQUEST** (letter `s`): on the engined host,
  one frame under its own count (`async_min_frames()`: 3 frames of an
  8-bit read at PCLK1/4, 2 at PCLK1/8, APB1's figures in "The host above
  the wire") completes inside `start()` with no completion on a stream,
  and the count itself rides the streams, one block and one completion -
  every window byte-exact at both ends; a four-frame command at
  PCLK1/256 rides a block of its own, the data block chained to it from
  the vector (two blocks, one completion), and a one-frame command at
  PCLK1/8 runs polled inside `start()` before the data block (one block,
  one completion).
- **THE COUNT PAST SIXTEEN BITS** (letter `j`): `cmd_len` is a byte and `len` sixteen
  bits, and their sum is taken in thirty-two. One command frame and 65535 data frames
  at PCLK1/8, 65536 frames, end `spi_ok` in 16 777 731 to 16 777 743 core cycles against
  the wire's 16 777 216 (93.2 ms), 515 to 527 above it: the fixed cost, the ticks landing
  on it absorbed by the transmit buffer. Judged: at least the wire's less half a bit, ten
  runs of the letter and three of `z`.
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
  bursts of 40 in every mode at high and p breaks at 351 kHz and faster - and is
  exact at medium and at low (the peer's `host_burst` hosts at SCK medium
  for the same doubled edge: "`SpiClient` against the STM32G0B1RE host"
  below). MOSI's edge slips the far client where the
  frames run back to back: thirty bursts of 32 frames on the DMA host at PCLK1/4 = 11.25
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

- **16-BIT FRAMES ON BOTH OF THE HOST'S WAYS** (letter `u`,
  spi_link's extended exchange in 16-bit frames): thirty-two frames a window, the G0
  serving them in half-word beats, a frame on the streams one half-word beat. Judged:
  byte-exact both ways at every code to PCLK1/4 from the polled loop, from the
  half-word streams and against the G0's own 16-bit client run; at PCLK1/2 every frame
  heard exact by the G0 and this end's read printed, not exact on this cable. Timed from
  two windows of 16 and 48 frames at the same rung (the transaction's fixed cost
  cancels, a pair a tick landed in run again), a frame inside the window takes 999 to
  1001 permille of its sixteen SCK periods on the streams at PCLK1/8, /4 and /2, and on
  the polled loop 1077 at /8 and 1218 at /4, the wire and the loop's turn ("The host's own
  loops"); 1437 at /2, printed. The 22.5 MHz rung needed the G0's answer line at very-high
  (its header): at high, more 16-bit windows read wrong while every 8-bit one held.
  Why PCLK1/4 is where the read is judged, by the answer line's budget: a bit of the
  G0's answer must cross the wire inside one SCK low phase, this end's tw(SCKL) at least
  division / 2 x TPCLK - 1.5 ns (DS10693 table 63), against the G0B1's tv(SO) up to 14 ns
  (DS13560 table 76) and this end's tsu(MI) of 4 ns, the SCK pad's edge at medium and the
  wire both ways besides. At PCLK1/2 that leaves 2.7 ns, at PCLK1/4 24.9.
- **THE TRANSMIT-ONLY SHAPES, WITNESSED** (letter `v`,
  `spare_receive_only`): 33 frames, 8- and 16-bit, through the polled
  transmit-only loop and the streams' write shape (the receive side
  discarded), every one heard exact by the G0 listening
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
  in both widths, on both serves, at PCLK1/256 and PCLK1/4, every data frame exact;
  and on its channels at PCLK1/2 the G0's check and every frame it heard are judged,
  this end's read printed.
- **ES0298 2.14.5's CRC SIGNATURE AT PCLK1/2, at every SCK class**: at
  22.5 MHz the far side holds - TXCRCR the arithmetic, the G0's check
  clean, every data frame heard exact by the G0 - but THIS END'S RECEIVE CRC
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
  walk reads is not the peer's CRC), its F/f column saying whether the
  CRC frame came at all - and at 8-bit PCLK1/2 no RXNE comes for it at
  nearly any point of the walk. That region is the one RM0390
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
  this core's speed lands before the window at PCLK1/4 and slower: every
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

- **A CLIENT WITH NO SELECT PAD, AND THE SELECT AS THE BLOCK'S OUTPUT**
  (letter `f`): `SpiClient` over pads whose `nss` is
  empty, `SpiNss::software`, is deaf with SSI high - the G0's twelve-frame
  `host_burst` at 2 MHz clocked, nothing arrived, the run ended `stalled`
  on its budget and none of the twelve frames the far host read was an
  answer (the first 0xFF) - and, selected
  by `select(true)` once the program has seen the far select fall on
  PB12 (a plain input), byte-exact both ways from the first frame, the
  run `complete` on its count. SSOE (26.3.5): the pad reads high before
  SPE, LOW as soon as SPE rises with no frame written, low through eight
  frames at PCLK1/64 and high again once SPE is cleared - and that window
  frames the G0 peer's exchange byte-exact both ways.
- **THE STALL A SECOND CHIP MAKES** (letter `g`, three runs of `z`): a 2048-frame
  polled request at PCLK1/256 (93.2 ms of wire) with this host's NSS a
  hardware INPUT - SSM and SSI cleared under the host's configuration -
  and the G0 pulling the shared select low some 36 ms in (`ss_pulse`):
  MODF clears SPE and MSTR, the RXNE the loop waits for never comes, and
  the request ends `spi_stalled` when its one budget (400000 + 4096 spins
  a frame) runs out, at 781.7 ms, the core and every other active object held
  that long, with the peripheral reset and reconfigured (SPE, MSTR and SSM back).
  ERRIE's vector runs ONCE on that fault, SR 0x22 (MODF, TXE standing beside it). The request is full
  duplex on purpose: on a transmit-only one OVR would come first, since nothing reads
  the receive buffer.
- **A CLIENT ON TXEIE, AND ERRIE ON A CLIENT THAT READS NOTHING** (letter
  `g`): enabling TXEIE with the buffer empty and the select high runs the
  vector at once, and that first entry is the preload; then one entry
  per frame, twelve for twelve answers, read byte-exact by the G0 host
  from the first frame while the main loop reading RXNE heard every frame
  exact. A client that reads nothing raises OVR at the end of its second
  frame and ERRIE's vector sees it once (SR 0x43) - `isr()` consumes
  nothing, so the flag stands until its clearing sequence.
- **A HANDLER LONGER THAN A FRAME IS A GAP ON THE BUS** (letter `h`, ten
  windows a run, ten runs and every figure the same in each): a TIM7
  update handler holding the core 350 cycles of its own (374 to 429 with
  its entry, its exit and the rest of its body, the suite's arithmetic)
  every 5 us, beside the polled receive loop at PCLK1/8 - 256 cycles a
  frame, 64 frames. With no handler a window takes 19141 cycles; with it,
  about 32 runs landing in each, every window is exact both ways, `spi_ok`,
  the far client's count right and OVR never standing, in 28650 to 29374
  cycles: 9510 to 10234 longer, where the suite's arithmetic allows 2478 (the
  runs that must land, times the 118 cycles a run longer than the frame
  must idle the bus) to 18398 (every run whole). The letter's handler is the only interrupt the loop meets there
  but the tick: the host arms none of the block's.
- **A DISPATCH ABOVE THE ARBITER IS A GAP ON THE BUS TOO**
  (`test_stm32f4_rubato_link` under the preemptive kernel, against the
  STM32G0B1RE's `spi_peer`, eighteen runs of letter `p`): the polled loop
  runs inside the arbiter's dispatch and an object of the highest priority
  preempts it every 97 to 389 us for 300 cycles, longer than a frame at /8
  and faster, or for 3000, longer than one at /64 and faster. Every rung
  from PCLK1/256 to PCLK1/2, eight frames in each of the seven passes - 1008
  rungs in all - is exact both ways: zero lost frames, against the bound of
  zero. The engines and the short asynchronous request hold the same way.

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
client) the peer drives MOSI at `SpiHost`'s default very-high and SCK
at MEDIUM: at very-high the G0's client sees an SCK edge twice and its
stream slips one bit, both ends wrong from that frame on (letter r wrong
in 5 runs of 5, letter y in 3 of 3), at high in 1 run of 3, and at
medium letters r and y are exact in 10 runs of 10 each. Medium stays
inside ES0298 2.14.5's table 5 for APB1's 45 MHz (75 MHz of APB), and
DS10693 table 58's 25 MHz at 50 pF for medium is above the fastest
burst this instance can clock (PCLK1/2 = 22.5 MHz; the suites ask for a
division of 32 or more). THE SELECT WIRE RUNS APART FROM SCK AND MOSI.
The peer holds PB12 low from GPIO for the whole burst, and a select run
in the bundle beside the burst's lines picks up their edges: the G0's
select pad sees 4 to 19 edges a burst where 2 are sent (an EXTI line on
the pad, both edges; its pull-up changes nothing - the line is driven,
and coupled), and a client that takes NSS from the pad is deselected
mid-frame and its bit count starts again - a frame lost or a bit
slipped, both ends wrong from one frame on, in 65 to 79 bursts of 100,
whatever its direction - while a client under SSM, which reads the pad
only to see the select fall, is exact through the same glitches. With
the select routed apart, SCK and MOSI moved together through the debug
port at every class and both select levels raise no edge on the G0's
pad in 19200 steps, and the bursts above count exactly 2. A larger
division keeps the same edges: the rate is not the remedy.
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
  (letters `w` and `t`, the stm32f4 `spi_peer`); a CRC phase in the host or
  the client - the last data frame queued behind the one before and CRCNEXT after
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
- The instances other than SPI5, SPI2 and SPI1 (SPI3, SPI4, SPI6 -
  compiled on every header that has them, none driven): a wire between
  two of them, or a device on one. SPI1 is driven on the STM32F411CE
  against a display and a touch controller (the finding above).
- The receive-only and half-duplex-in configurations (`SpiConfig`'s
  `direction`): the host's transactions are full duplex by construction, its
  Request has no such field, and the resource's simplex modes were driven by hand in
  the bidirectional letter alone.
- The DCS link's commands and pixel rows over the transmit-only shape
  against a panel: the STM32F411CE's display (the breadboard finding)
  wants a run of its probe over this host.
- `rebase()` and the SCK ceiling across a clock change: the family's
  `DynamicClock` ([clock.md](clock.md)) exists, and no program here makes
  an `SpiHost` one of its users. `rebase()` derives the APB rate from
  the new SYSCLK (`apb_hz_at`) - reading the divider back, the I2C hosts
  measured, reads the rate being left. What would measure it is a
  transfer timed against the partner at every rate of a pack.
- The frame error (FRE): it is a TI slave's and an I2S slave's, and
  neither role has a peer here.
- The receive loop's frame in modes 2 and 3, and on SPI2 at /2: letter e's
  `spi.frame` lines run modes 0 and 1 at /4 to /256, so the header's CPHA = 0 and
  CPHA = 1 figures extrapolate from one mode each; the same lines over the other two
  would measure them.
- `spi_dma_fault`: no letter makes a stream report a transfer error, so the path
  (requests dropped, streams put away, the receive buffer read, select released)
  is read in the code. A stream pointed at an unmapped address would provoke it.
- `SpiHost` on the STM32F429I-DISC1: that half of the suite compiles over the host as
  it runs a request here and was not run with these loops, so nothing above states the
  polled loop's cost against the two soldered devices on SPI5 (letter b's ladder, and
  auto-increment through it), the arbiter over them (letter j: four replies in order, a
  write-then-read, the surplus rejected, the sleep vote), the asynchronous way (letter
  k's block timing and its one interrupt) or 16-bit frames through both ways. A run of
  that half on the board would measure them.
- An engined request on the STM32F411CE's breadboard panel at 12 MHz as
  this host runs it - a short command byte polled inside `start()`,
  the data block on the streams: the pad finding above was measured with
  every frame written from the instance's interrupt. A run of the probe
  over this host would measure it.

Findings left open, with what would settle them:
- This cable's read-back at PCLK1/2 = 22.5 MHz is not exact. With this end hosting the
  G0's client on SPI2, 16-bit frames read wrong at that rate from the polled loop and the
  streams alike: 5 of 64 bytes in a window of 32 frames from the polled loop, in bit 2 of a
  frame (0x80 for 0x84), and 12 from the streams, in bit 15 (0x65 for 0xE5), in every run,
  while the G0 hears every frame exact. 8-bit windows hold there and stay judged (letters p
  and l), though the CRC frame after them does not. So the suite judges this end's 16-bit and
  CRC reads to PCLK1/4 and prints PCLK1/2, the bench proving the software and not the top
  rate of a set of jumper wires. It does not follow this host's code: the two ways are
  different loops and PCLK1/4 and slower are exact. The cause is not established. The answer
  line's budget above leaves 2.7 ns at that rate for the pad and the wire, and letter `x`'s
  walk of this end's SCK and MOSI classes and of the peer's MISO class in 16-bit frames, or a
  printed board's short traces, would say which edge it is.
