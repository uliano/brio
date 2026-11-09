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
r.engine  = brio::SpiEngine::polled;
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
90 MHz over 16 at a 180 MHz core) and `engine` the way to run it.
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
never rose within its budget), with the select and the D/C released, the peripheral reset
and `status()` answering `spi_stalled` instead of `spi_ok`, so a program that
must trust `who` reads `status()`, as the block does, and calls nothing else:
the next request starts on a clean block.

Two more decide correctness before the first frame. **On a bus of wires the
slew class of every driven pad is a correctness parameter**: the default,
`very_high`, puts a far client one bit out of place ("The pads"), so name the
classes in the `Config` `init()` takes. And at a 180 MHz core PCLK2 is 90 MHz,
above ES0206 2.12.4's ceiling of 84 MHz at every pad class, so on an APB2
instance the erratum no longer promises the last received bit: a core at 168
MHz, or an APB1 instance at 45 MHz, stays inside it, and
`within_errata_ceiling()` answers for the bus as it runs. Above the ceiling the
board's gyroscope read exact on its short traces (Bench findings), one board and
one load; this host's present loops have not been run on that board, so the
example as written is compiled and not bench-verified ("Not covered yet").

A polled request runs an explicit loop at every length, on a host with DMA
engines too: it arms no interrupt, of the instance or of a stream, so **it
needs no interrupt vector bound**, and its loop waits with interrupts enabled;
the buffers are read and written inside `start()` and are the caller's again
when it returns. **It holds the core for its whole wire time**: under the
cooperative kernel (`Tenuto`) every other active object waits with it, only
handlers running, and a stall holds them for the budget. The budget is one poll
for each core cycle of the request's wire time and two frames more, and a poll
takes 16 cycles or more, so a request of 2048 frames at PCLK1/256, whose wire is
93 ms, stalls for 1.49 s. Through the
arbiter a request is posted instead (the block "The same bus shared, through the
arbiter" below): the post returns at once, the loop runs later in the arbiter's
dispatch, and the buffers are lent from the post until the `SpiDone` reply
arrives, untouched and in scope until then, which nothing checks.

A request naming `SpiEngine::dma` runs on the DMA engines at any length. Three
things make that work. The host's type names its two DMA engines. The host's
`init()` arms both streams, which opens the controller's clock gate and their NVIC
lines (`Dma<n>::init()` before it is the program's choice, a controller at its reset
values; after another engine's arming it would undo that arming). The program binds the two streams' vectors to `Bus::dma_isr()`,
whose true answer is the edge the glue posts `TransferDone` on. What the engines
cannot carry - on a host with none, everything - is refused with `spi_rejected`
before the select falls, and so is any request while this host's own block is still
on the streams, the one busy check ("The engines" below). **The request runs on the engine it names and
on no other**: name the engines when the other objects must run through the transfer
or a long read wants the wire's rate, and the loop otherwise; "Which engine costs the
core less" below gives the lengths, as guidance, and nothing in the host reads them.
**When a request ends, the wire has ended**, on both engines: the select rises and the
answer comes after the last SCK edge ("When a request ends" below).

The page is long because the block has two faces and two ends. In
order: the documents of record; what the silicon does; the types and
verbs; the host above the wire (what a transaction costs beyond its
frames, the polled loop's one frame in flight and what it costs, the
engines, when a request ends, which engine costs the core less); the
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
`dc`, `cmd`, `tx`, `rx`, `len`, `cmd_len`, `bits`, `engine`,
`cs_setup_us`, `clock`, `mode`, `reply` - in that order (a designated
initializer follows it), 40 bytes with no padding. Of these a request on the
engines keeps two, `cs` and `dc`, for its tenure (its time on the bus): the
completion moves those pads.

The verbs: `init(clock, max_sck_hz)` (the shared surface) and `init(clock,
Config)` - `Config` being `max_sck_hz`, `sck_speed` and `mosi_speed`, the
classes of the two pads a host drives ("The pads") - `rebase(sysclk)`,
`clock_for`, `sck_hz`, `max_sck_hz`, `ceiling_clock`, `reference_hz`
(the instance's APB clock), `prime`, `bit_order`, `lsb_first`, `start`,
`dma_isr` (the streams' interrupt body; the host arms no interrupt of
the peripheral and binds no vector of its own), `status`, `recover`, `release`,
`claim_nss_pad`. The constant a program reads: `has_dma` (whether engines are
bound: a request naming them needs them, `static_assert(Bus::has_dma)`).

`status` answers one of four, and each has what follows it
([design/spi-bus.md, "What follows each code"](../design/spi-bus.md#what-follows-each-code-and-who-acts)):
`spi_ok`; `spi_rejected`, a request naming the engines on a host with none or one
they cannot carry (the list under "The engines" below), nothing on the wire and
nothing to do but change the program - or any request while this host's own block
is on the streams, the busy check, the block in flight going on untouched;
`spi_stalled`, the select and the D/C released and then the
peripheral reset and reconfigured, inside `start()` - or, on the engines, the vector's
wait for the wire's end run out, the select and the D/C released and the reset left
to the next `start()`;
`spi_dma_fault`, the DMA requests dropped, both streams put away and the select and the
D/C released in the streams' vector, which does no more: the peripheral's reset is left
to the next `start()`, which tests a flag first and pays it before anything else, so
until then a frame can still be on the wire and another stand in the receive buffer.

**After `spi_stalled` and `spi_dma_fault` the program calls nothing** - but for a
closed clock gate (the paragraph after next). The next request
starts on a clean block whoever calls it, in direct use or behind the arbiter, which goes
on to the next waiting request before the requester has run. The select goes first
because this block drives SCK only while SPE is set: the reset leaves the pad to whatever
else holds the wire and the enable after it drives the pad again, an edge a device still
selected would count. A request with a null `cs` has its select in the program's hands, so
the host cannot release it and the reset happens under it; `prime()` pays an owed reset
too, so a program framing the select itself primes first, as it must for a mode change.
`recover()` remains, for the arbiter on `spi_timeout`, its one use there, and for a program
with the bus idle; it waits for nothing, and answers false when the block did not come back -
its clock gate closed, the one case measured: the program opens the gate and calls `recover()`
again, which then answers true.

**What the reset cannot do: open a closed clock gate.** With the gate closed the next request
stalls too, on its own budget, and `recover()` answers false. The first request after the
gate reopens finds the block at its reset values and stalls as well, its own reset the way
back, unless the program calls `recover()` when it opens the gate (Bench findings).

With engines, the transmit stream is armed for its errors alone and the receive
block's completion is the DMA block's one interrupt. **The receive stream takes the
controller's priority `very_high` and the transmit stream `high`** (DMA arbitration
priorities, not pad slew classes; docs/design/dma.md's rule): the receive
buffer holds one frame, and a receive stream the transmit one outran finds it full -
OVR, "all other subsequently transmitted bytes are lost" (RM0090 28.3.10) - where a
waiting transmit only stops the clock between two frames. `release()` stops both
streams, disables the peripheral and gates its clock, no reset pulse; `init()` starts
from the RCC reset, which withdraws a DMA request held through the gated clock before the
streams are armed - the release contract ([dma.md](dma.md)).

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
listener), the error verbs, `isr()`, `release` (the block disabled
and its clock gated, no reset pulse, `init` starting from the reset, as
the host's), and `frames_ahead` = 1, this silicon's answer
to the one number a portable client would need.

`SpiPins` and `I2sPins` name the pads with the alternate function the
DATASHEET gives each signal there; the device header carries no pin
table, so nothing can check an AF and the bench is the check.

## The host above the wire

What a transaction costs beyond its frames is the host's own, and each
piece of it is written from the chapter and counted in the release
listing (`bench_stm32f4.lst`; the numbers are under "Bench findings").

**The request is lent for the call** (util/bus_master.hpp's contract),
so it is read through the reference and copied nowhere, but for the select
and the D/C of a request on the engines, the two pads its completion moves:
two inline stores of two words. The rest is spent before `start()` returns.

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
nothing stands in it when a transaction ends well. A stall, `spi_dma_fault`,
`recover()` and `init()` reset the block, which empties it.

**Two polled shapes, by the request's fields.** A request naming
`SpiEngine::polled` runs one of them at every length and on every host, and never the
DMA engines. A handler can land on a polled loop at any instruction, and
on this platform no interrupt nests, so every handler in the image is a
window the loop cannot see through - the tick's, the console's, the
bench's stamped ones - and under the preemptive kernel so is every
dispatch above the caller. That decides the shapes:

- *Transmit-only* (`rx` null: every command phase, a display's pixels):
  paced on TXE, the answers never read; at the tail the last frame is
  let out of the shifter - TXE, then BSY, 26.3.10 - and the receive side
  cleared by DR then SR, the overrun the unread answers raised included
  (26.3.13: the transmit side is untouched); then the SCK pad's idle level,
  because BSY falls half a bit early ("When a request ends"). Nothing is waited for on
  the receive side, so a handler delays the loop and loses nothing. It runs
  at the wire at 22.5 MHz and slower, and at SCK 45 MHz its turn of 38 cycles
  (40 in an image laid out otherwise, Bench findings) is the bound against the
  wire's 32: 16 instructions and two APB2 accesses a frame.
- *Receive* (`rx` set): one frame in flight at every rate. Nothing is
  queued behind the frame shifting, so nothing can be lost: this host
  owns the clock, the clock waits for the read, and a handler or a
  preemption is a gap on the bus. The turnaround is the minimum - the
  next frame fetched inside the wire time and pinned there with an empty
  asm (the compiler would sink the load past the volatile poll), then
  the DR read and the DR write adjacent; after the last frame, the SCK pad's
  idle level.

One budget bounds the whole transaction's spins, a count of polls derived from the
clocks and not one chosen by hand: the request's frames and two more at the applied rate,
a frame being the division times its bits in PCLK cycles and log2(HCLK/PCLK) more in
core cycles, in polls of at least a cycle each, so a wait ends no sooner than the wire
could have (the shift is worked out wherever the rate, the width or the clock
changes, and a request pays one load, an add and a shift). When it runs out the select
and the D/C are released, the peripheral reset and reconfigured and the transaction
ends with `spi_stalled`. A request that changes the rate, the mode or
the width runs the disable of 26.3.10 first, with bounded waits of its own outside
that budget and an answer the host drops: the budget of the transaction that follows
shows what it left as a stall of its own.

**The reset, once, cold and out of line.** The stall, the owed reset and `recover()` call one
function: the RCC's pulse (RM0390 6.3.8 and 6.3.9), the applied configuration written back,
the enable; `recover()` around it measures 934 to 1091 cycles (Bench findings). It declines 26.3.10's disable: that procedure lets a last
frame out, behind a released select nobody wants it, and on a block with SPE down or its
gate closed it returns at once anyway. On a host with engines `start()` pays for it one
byte load and one branch a request (`ldrb`, `cbz` in the release listing), the call only
when a reset is owed; on a host without engines the test compiles away.

**The transmit buffer's write-ahead (26.3.9, figure 311) is used by the
transmit-only shape and declined by the receive shape.** Storing the next
frame while the previous one shifts keeps the wire continuous, and the
transmit-only shape does it because it reads nothing. A receive shape
doing it would put two frames in flight, and it would overrun the receive
buffer whenever anything held the loop from reading frame k for longer
than one frame time after its RXNE (26.3.13); no bound on the image's
handlers, or on the dispatches above a preempted one, can be enforced
here. The price of declining it is the turn between two frames read,
measured below ("The host's own loops"): at most 72 HCLK cycles a frame
at 180 MHz in the lines run, modes 0 and 1 at /4 to /256. A read that wants the
wire's own rate names the engines, and the streams keep it.

The D/C boundary needs nothing more: the first data frame is written
only after the last command frame came back, because a frame queued in
the transmit buffer goes out the moment the one before it ends.

**The engines** ([design/spi-bus.md](../design/spi-bus.md),
"Two ways to run a request"). A request naming `SpiEngine::dma` rides the
host's two engines at any length: `start()` keeps the select and the D/C,
starts both streams and raises both DMA requests in one CR2 store, and returns
false; the receive stream's completion is the one interrupt a request, and
`dma_isr()` answering true is the edge the program's glue posts `TransferDone`
on. The instance's own interrupt is never armed, and `init()` leaves its NVIC
line as it finds it. In this order:

- THE ONE BUSY CHECK, first of all and the same for a polled request: while
  this host's own block is still on the streams, `start()` returns true at once
  with `spi_rejected`, before the select, the D/C or the block moves - the only
  busy check there is, no other owner holding the streams (docs/design/dma.md,
  "The ownership rule"). Letter `s` starts a block of 64 frames at PCLK1/256
  and, under it, one request naming the streams and one polled: both refused,
  their select never falling, D/C and CR1 untouched, and the block in flight
  ending as it would have, one completion, `spi_ok`.
- A host with no engines REFUSES it: `start()` returns true at once with
  `spi_rejected`, before the block, the select or the D/C is touched
  (measured: the select's pad sees no falling edge, letter `q`), and `has_dma`
  says so at compile time.
- A reset a transfer error left owed is paid next, on a host with engines: the
  block alone, before any of the tests below.
- An empty request (no command and no data) completes on the spot, `spi_ok`.
- What the engines cannot carry is refused the same way, before the select, the
  D/C or the wire moves: 16-bit frames on byte engines; a 16-bit frame whose
  buffer, out or in, is off a half-word (a half-word memory address must sit on a
  half-word, RM0390 9.3.7); and a buffer anywhere in 0x1000 0000 to 0x1FFF FFFF,
  one compare of the address's top four bits. That region opens with the CCM,
  64 KB at 0x1000 0000 on the parts that have one (RM0090 2.3.1), which no stream
  reaches: the core-coupled memory sits on the CPU's D-bus alone, and on a part
  without it nothing answers there, a bus error on the first beat
  ([dma.md](dma.md)). The same compare refuses the region's top too - the system
  memory at 0x1FFF 0000, the OTP area and the option bytes - where the reason does
  not hold; whether a stream reads them is not measured, and a program sending a
  constant from there names the loop. Measured: letter `s`, every case with the
  select never falling, D/C and CR1 untouched, no edge on SCK.
- A command phase followed by a data phase runs in the transmit-only loop inside
  `start()`, then the data phase on the streams: one fixed rule, no arithmetic -
  a command is short by nature, and a block of its own would cost a second launch
  and a second interrupt. D/C rises after the command's last SCK edge and before
  the data's first (letter `s` counts them: 64 of 64 edges at PCLK1/256 for a
  four-frame command). A stall in that loop ends the request as a polled one's
  does: `start()` true, `spi_stalled`, the select and the D/C released, the block
  reset.
- A command with no data phase is the block, the whole request on the streams.
- The streams then take the block: neither can refuse it, the busy check having
  left none of this host's running, the refusals above covering the run, and no
  other owner holding them.

**At its end, ok or failed, a request leaves the select and the D/C high**
(docs/design/spi-bus.md, "When a request ends"): a polled stall, a stall in the
command phase ahead of a streams' block, a transfer error and a wait for the wire's
end that ran out in the vector all release both, as a clean end does. Letter `k`
makes three requests fail with a command phase holding D/C low - a polled command
that stalls, a streams' request whose command stalls in the loop, a command alone
on the streams whose block reads reserved space - and reads D/C high after each.

**When a request ends, the wire has ended.** The select rises, and `start()` or
`dma_isr()` answers true, after the last SCK edge - also for a caller that frames
the select itself (`cs` null), whichever engine the request named. RXNE rises on
the last sampling edge (26.3.9), which in the CPHA = 0 modes (0 and 2) is half a bit
before the frame's last edge, and **BSY falls with it** (measured: in all four modes,
at every rate, on both instances, BSY is low in the same poll as the first RXNE; in
modes 0 and 2, at the slow rates where the poll comes inside the half bit, the pad is
still at the active level then). So neither flag marks the end, and no interrupt
does. The SCK pad's own input does: in its alternate function
the pad is sampled into IDR every AHB1 cycle (RM0390 7.3.11), and the edge that ends
the frame is the one that brings it back to the mode's idle level. Each polled
phase's tail, after its last RXNE or its TXE-then-BSY, waits for that level - once a
phase, never a frame - on the transaction's budget; the receive stream's completion
waits for it in the vector, on a bound counted in polls. Half a bit at the applied
rate is 2^code PCLK cycles, which is 2^(code + log2(HCLK/PCLK)) core cycles; the
bound is that many polls of IDR, plus 16 for the pad's climb, and since a poll takes
at least one core cycle the bound is never shorter than the half bit. When the edge
comes the wait ends at it, within the half bit; when it never comes the vector spends
the whole count, each poll an IDR load, a compare, a count and a branch, so several
times the half bit. Then the request ends `spi_stalled`, the select released and the
reset owed to the next `start()`, the D/C high with it (letter `E` stages it by taking the pad from the
block). A transfer error does not wait: the vector drops the requests, puts the
streams away, releases the select and the D/C and marks the reset owed, which the
next `start()` pays. The cost by rate, in the vector with every other interrupt held off, measured
by letter `E` (the vector's cycles from entry to `dma_isr()`'s answer, mode 0 against
mode 1, which waits for nothing):

| the wait in the vector, core cycles at 180 MHz | /2 to /32 | /64 | /128 | /256 |
|---|---|---|---|---|
| half a bit, SPI2 (APB1, 45 MHz) | 4 to 64 | 128 | 256 | 512 |
| measured wait, SPI2 | 0 | 0 | about 30 | about 290 |
| half a bit, SPI1 (APB2, 90 MHz) | 2 to 32 | 64 | 128 | 256 |
| measured wait, SPI1 | 0 | 0 | 0 | about 80 |

The vector's own body, some 220 cycles on SPI2 and 185 on SPI1 from entry with
nothing to wait for, covers half a bit at /64 and faster on APB1 and at /128 and
faster on APB2: there the wait is one IDR read. The longest the vector holds every
other interrupt off when the edge comes is the /256 row, 512 cycles from entry on
SPI2.

What letter `E` can see. Its control is the same letter over tails that end on the
flags alone - the last RXNE for a read, TXE then BSY for a write, the completion for
the streams - and it finds the pad still at the active level in modes 0 and 2: polled
at /64, /128 and /256 on every way (SPI1's eight-frame read at /128 and /256), on the
streams at /256 (but for SPI1's command alone, never early), and SPI2's command loop
ahead of a block at /128 and /256. At the faster rates the turn from the flag to the pad's read outlasts the half bit, and the
letter cannot see an early end there.

**Which engine costs the core less** - guidance for the program that names one; the
host reads none of it. On the engines a request costs the core a fixed E -
`start()`, the two streams started, the completion's vector - whatever its length;
polled, a fixed part F and each frame's time: W + the receive loop's idle bus for a
frame read, max(W, the transmit-only turn) for a frame written, W being the frame's
HCLK cycles at the code, the width and the instance's bus (8 bits x the division x
HCLK/PCLK). So the engines cost the core less from N = (E - F) / frame frames, rounded
up. `bench_stm32f4` letter `e` measures E as the least busy of a 64-frame request at
SCK 5.625 MHz on the engined host and F as the polled wall of 16 and 64 frames through
the same call extrapolated to no frames, so the ruler's cycles cancel: **E 570 for a
read and 571 for a write, F 177 and 195, E - F 393 and 376**, at 180 MHz, SPI1, the
same in three runs. These figures move with the image's layout behind the ART (the
finding under Bench findings): an image of the same host code laid out otherwise reads
E 603 and 656, F 173 and 195, E - F 430 and 461, so the table below gives the range the
two layouts span. The frame figures are the same letter's: the transmit-only turn at
SCK 45 MHz, 38 cycles in the one image and 40 in the other against the wire's 32 (the
wire is the bound at 22.5 MHz and slower); the receive frame 72 cycles at /2 against
the wire's 32, and at /4 and slower the `spi.frame` lines of "The host's own loops"
below - mode 0's, the shorter, so a cell errs toward the loop. E and F are measured on
SPI1 alone. The APB1 rows use SPI1's E - F: SPI2's vector runs some 35 cycles longer
(letter `E`), and E - F that much larger moves none of their cells out of its range;
SPI2's own E and F are not measured. What that gives, 8-bit frames:

| the engines cost the core less from | /2 | /4 | /8 | /16 | /32 and slower |
|---|---|---|---|---|---|
| a read, APB2 (SPI1, SPI4, SPI5, SPI6) | 6 | 5 | 3 | 2 | 1 |
| a write, APB2 | 10 to 12 | 6 to 8 | 3 to 4 | 2 | 1 |
| a read, APB1 (SPI2, SPI3) | at most 8 | 3 | 2 | 1 | 1 |
| a write, APB1 | 6 to 8 | 3 to 4 | 2 | 1 | 1 |

The read on APB1 at /2 has no measured frame (SPI2's receive frame at /2, the gap
below); with no idle bus at all the frame would be the wire's 64 cycles, which bounds
the cell at 8.

In words: on APB2 a single frame already costs the core more polled than on the engines
at /32 and slower, and on APB1 at /16 and slower; at the fast rates a short request -
a display command and its few parameters, a register read - costs the core less in the
loop. Through the arbiter the engines pay its second dispatch besides, which this
arithmetic does not count. A request's length is the program's to know; the choice is
its own.

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
  has no engine parameters (the gap list); a program binds the streams raw, as
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
r.engine  = brio::SpiEngine::polled;
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
with `uint8_t` a 16-bit request naming them is refused (`spi_rejected`):

```cpp
using Tx = brio::DmaTxEngine<2, 4, 2, uint16_t>;   // SPI5_TX: DMA2 stream 4, channel 2
using Rx = brio::DmaRxEngine<2, 3, 2, uint16_t>;   // SPI5_RX: DMA2 stream 3, channel 2
using FastBus = brio::SpiHost<5, spi5, Tx, Rx>;
brio::Dma<2>::init();
(void)FastBus::init(clock);             // no ceiling: nothing for it to refuse
```

and shared, through the arbiter - the client posts and gets a `SpiDone`
back; a request naming `SpiEngine::dma` runs on the engines, which must be
bound (`has_dma`), and the app's ISR glue on the two streams' vectors
is the one vendor line. An excerpt. `Platform` is the program's platform type
(`brio::Stm32f4Platform<brio::Ticker>`, say), `MyAo` the client's active object;
`Arb` goes in the list of active objects given to the kernel; `cmd` and `block` are
the client's static storage, since the loan runs past the function that posts:

```cpp
static_assert(FastBus::has_dma);   // a request naming the engines needs them
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
a.engine  = brio::SpiEngine::dma;
brio::post<Arb>(a);                // returns at once; `block` stays lent

// In MyAo's dispatch, on the reply:
//     [](const brio::SpiDone& d) { if (d.status == brio::spi_ok) { /* read `block` */ } }
```

This request names the engines: its one command frame runs in the loop inside
`start()`, then the six data frames go on the streams, one interrupt at their end. With `timeout_ticks` left at its default a channel that never completes leaves
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
engines on this board are in the second list of "Not covered yet".

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

- **An engined request costs some 3.1 us more than its wire, and ONE
  interrupt.** Measured
  (`bench_stm32f4` letter d, the data phase alone, no command frame, the
  request on the streams): wall minus
  the wire's time is 568 cycles at SCK 22.5 MHz and 556 at 5.625 MHz in
  8-bit frames, 572 and 560 in 16-bit ones, for 16 frames and for 256, the
  same in three runs, and the core is busy 574 to 579 cycles of it -
  `start()` (the engine's test, the refusals' tests, the settings compared,
  the two pads kept, the two block starts and the requests raised in
  one CR2 store), then one completion handler of 138 cycles with the
  bench's stamps (the receive block's; the transmit stream is armed for
  its errors alone, because every frame the receive stream took was
  clocked out first). A 256-frame request runs at x 1.03 at 22.5 MHz and 1.00 at
  5.625. THE FIGURE MOVES WITH THE IMAGE'S LAYOUT, not with the code: the same
  host code in an image laid out otherwise reads 560 to 648 above the wire, its
  16-frame requests and its 16-bit ones the dearer, and its letter `e` the E of
  "Which engine costs the core less"; the fetch behind the ART is the candidate (not
  traced).
- **The host's own loops against the wire** (letter e, the same SPI1,
  the engineless host, 256 frames, the best of five runs with the
  console's and the tick's handlers live, the worst run printed beside
  it; three runs of the letter). The transmit-only shape: x = 1.21 at SCK
  45 MHz (38 cycles a turn against a 32-cycle frame; 1.27 and 40 cycles in the
  image laid out otherwise), 1.01 at 22.5, 1.00 at 11.25 and 5.625, and 1.00 to
  1.01 in 16-bit frames at every rate. The receive shape in mode 0: x = 2.26 at
  45 MHz, 1.50 at 22.5, 1.22 at 11.25 and 1.08 at 5.625 in 8-bit frames, 1.50,
  1.22, 1.12 and 1.03 in 16-bit ones; the same loop in the image's other host, the
  engined one, reads 1.95 and 1.63 at 45 and 22.5 MHz, and which of an image's two
  hosts gets which moves with its layout, the loop's instructions unchanged
  (design/benchmark.md's gap on placement). What the receive loop's one frame in flight costs at every
  rate, the letter's `spi.frame` lines, the receive frame's HCLK cycles over
  the wire's from 256 less 16 frames, each run with interrupts masked, the
  same figures to a tenth of a cycle in three runs of the letter:

  | over the wire, 180 MHz | /4 | /8 | /16 | /32 | /64 and slower |
  |---|---|---|---|---|---|
  | SPI1 (APB2), mode 0, 8-bit | 32 | 28 | 20 | 4 | 0 |
  | SPI1, mode 0, 16-bit | 28 | 32 | 16 | 8 | 0 |
  | SPI1, mode 1, 8-bit | 32 | 40 | 32 | 40 | 32 to 40 |
  | SPI1, mode 1, 16-bit | 40 | 32 | 40 | 32 | 32 to 40 |
  | SPI2 (APB1), mode 0, 8-bit | 52 | 44 | 28 | 0 | 0 |
  | SPI2, mode 0, 16-bit | 64 | 48 | 36 | 0 | 0 |
  | SPI2, mode 1, 8-bit | 72 | 64 | 68 | 56 | 64 to 72 |
  | SPI2, mode 1, 16-bit | 64 | 68 | 56 | 72 | 56 to 68 |

  The image laid out otherwise reads the same table to a hundredth of a cycle: what moved
  between the two layouts is E and the transmit-only turn, not this loop.

  The wire's frame is 8 bits x the division x HCLK/PCLK: 64 cycles at /4 on
  SPI1 in 8-bit frames, four times that on SPI2 in 16-bit ones. In mode 0
  RXNE rises half a bit before the frame's end (26.3.9), and the turn hides
  in that half bit at the slow rates; in mode 1 the last sampling edge is
  the frame's end, and the turn shows at every rate. At /2, 8-bit frames
  read at 72 cycles on SPI1 against the wire's 32 and 16-bit frames at 96
  against 64. The figures' alternation by 4 cycles from code to code in
  mode 1 was not looked into: on SPI1 it is two cycles of its APB clock.
- **A short polled request's fixed cost: 199, 256 and 266 cycles** for
  a command byte and 0, 2 and 15 data bytes at 5.625 MHz (wall minus
  the wire's time; D/C and the select on two pads, the data phase in the
  transmit-only shape, each phase's tail waiting for the SCK pad's idle level),
  and 216, 273 and 283 over the engined host, the same loops (210, 267 and 278 in
  the image laid out otherwise). 256 frames polled at 45 and 22.5 MHz over the engined host
  run the loop too, with no completion (irq 0), a read and a write alike.
- **The vendor's loop on the same board.** ST's HAL v1.8.5
  `HAL_SPI_TransmitReceive`, its two-lines branch, compiled at -Os with
  brio's crt, clock and console around it on the same pads and BR codes:
  256 frames at x = 3.02 at 22.5 MHz and 1.32 at 5.625 in 8-bit frames,
  1.81 and 1.19 in 16-bit ones, and 530 and 748 cycles over the wire for
  a request of one and three frames - its loop tests TXE and RXNE
  independently as figure 311 does, and calls `HAL_GetTick()` on every
  turn for its timeout. brio's receive shape at the same rates is x =
  1.50 at 22.5 MHz (1.63 in the instance over the engined host), under a
  third of the HAL's distance from the wire, and 1.08 at 5.625, a quarter
  of it; a short request costs brio under half the HAL's. The HAL's tail waits
  for BSY alone (`SPI_EndRxTxTransaction`), so in modes 0 and 2 its request ends
  half a bit before the wire does.
- **The listing.** `start()` is 662 bytes over the engined host and
  648 over the engineless one, both shapes inlined. The engineless host's has three
  calls, every one cold: the rate's change, the select's setup time and the stall.
  The engined host's has those three and two more, the owed reset and the engines'
  path. With `SpiEngine::polled` it reaches the loop past one byte test of `engine`;
  no call in either loop or in the tails' wait for the SCK pad (an IDR load, a compare,
  a counter), every field of the Request in a register. The width and the null out
  buffer are tested a frame, in registers, and in the receive loop inside the wire's
  time: the turn runs from the RXNE poll's load to the DR store, the DR load between
  them. The engines' path keeps the two pads with inline stores.
- **16-bit frames ride the engines**, a half-word a beat out of the
  Request's bytes low-first, when the engines' element is `uint16_t`:
  256 frames at x 1.02 at 22.5 MHz with one interrupt. A 16-bit
  request on the engines whose buffer is not half-word aligned is refused.
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
`spi_peer`, firmware 0x0302, on its SPI1), runs **120 verdicts** in `z`, all
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
  a constant turn after RXNE of 466 to 468 ns (83 to 84 core cycles: the poll's last
  turn, the DR read, the next write, the restart), visible where it outlasts the
  half bit RXNE leaves (the last sampling edge, 26.3.9), at /2 to /16, and hidden by
  it at /32 and slower, where the frame time is eight bits: 74 ns off them at /256,
  inside the half bit over 32 frames the letter allows (90 ns with the arithmetic's
  truncations). The turns' spread is
  judged against one turn of the poll, measured in the same letter, PCLK1's edge and
  the arithmetic's truncations; a code a kernel tick landed in is printed and left
  out. `SpiHost`'s own loop is the `spi.frame` table above.
- **THE MODE FAULT'S CLEARING SEQUENCE, FROM A FRESH FAULT** (letter `d`, ten runs):
  under a mask, the SR read followed by the CR1 write of the value kept clears MODF, the
  first SR read after the write already finds it down, and a write raising MSTR, SSI and
  SPE takes after it; with a CR1 read, a CR2 read or a second SR read between the two
  steps the flag comes down all the same, on the same first read. `clear_mode_fault()`
  clears a fresh fault and answers true.
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
  thirty-two on the ENGINED host (DMA1 streams 4 and 3, one interrupt a
  transaction), and the peer's polled client run to 22.5 MHz
  too (informational) - with the MOSI pad at medium (the pads, below);
  at high the engined climb slipped at 11.25 MHz in about one run in
  six.
- **THE ARBITER OVER A SECOND CHIP** (letter q): four transactions
  queued from one dispatch at the command rate - two on the streams, one
  polled inside `start()` and answered in the arbiter's own dispatch, one
  with a three-frame command in the loop before its data block - come back
  in order, spi_ok, the 24 bytes read back exact by
  the peer; six into a
  three-deep queue answer two `spi_rejected`; an idle bus votes for a
  sleep and a busy one against it; a completion swallowed is answered
  `spi_timeout` at the per-bus limit, the arbiter calling `recover()` itself
  - the next four run to spi_ok on the same bus AO. AND A REQUEST NAMING
  THE ENGINES ON THE ENGINELESS HOST IS REFUSED: `start()` answers at once
  with `spi_rejected`, and the select's own pad (PB12, its falling edges
  counted on EXTI line 12) never falls - once for the same request
  polled.
- **THE ENGINE A REQUEST NAMES** (letter `s`, against the peer, ten runs): one
  frame read and one written on the streams at PCLK1/4 and PCLK1/8, where the loop
  costs the core less - `start()` false, one block, one completion, exact at both
  ends; the refusals of "The engines" above with every witness; the one busy check,
  two requests under a block in flight refused with their select never falling and D/C
  and CR1 untouched, the block ending `spi_ok` with its one completion; the next
  request exact; a four-frame command before eight data frames at
  PCLK1/256 and a one-frame command before 32 at PCLK1/128, each one block and one
  completion with D/C rising at exactly the command's last edge (64 and 16 of SCK's
  edges counted); a three-frame command alone on the streams, D/C high at its end.
  Its control, the same letter over a host that ran a short request naming the engines
  in the loop and refused nothing before the select fell, fails four of the seven
  verdicts.
- **WHEN A REQUEST ENDS THE WIRE HAS ENDED** (letter `E`, no wire but SCK's own pad,
  SPI2 and SPI1, ten runs): 1344 requests on each instance - seven ways (polled reads
  of one and eight frames, a polled write, the engines' read, write, command alone and
  command then data), four modes, eight rates, six times each, through a select and a
  null one - every one with the SCK pad at its idle level when it ended; the vector's
  wait as in the table of "When a request ends", mode 0's longest vector at every rung
  at most 8 cycles past mode 1's least plus half a bit, judged against mode 1's own
  spread there plus 16 (one turn of the pad's poll, IDR's sampling and the pad's edge,
  DS10693 table 58); a wait staged to run out ends
  `spi_stalled`, the select up, and the next request `spi_ok`.
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
  1000 permille of its sixteen SCK periods on the streams at PCLK1/8, /4 and /2, judged
  against 1000 plus or minus 3, 4 and 7: the 24 core cycles by which the two windows'
  fixed costs can differ (PCLK1's edge at the start, the completing vector's two APB1
  accesses, the poll that sees its end) over 32 frames. The polled loop's frame is
  printed, not judged, being the wire less the hidden half bit plus a turn of code
  whose APB1 accesses the manual gives no time for: 1085 at /8, 1234 at /4 and 1468 at
  /2 ("The host's own loops"). The 22.5 MHz rung needed the G0's answer line at very-high
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
  the request ends `spi_stalled` when its one budget (2050 frames of 8192 core
  cycles, 16.8 million polls) runs out, at 1.49 s, 16 cycles a poll, the core and
  every other active object held that long, with the peripheral reset and reconfigured (SPE, MSTR and SSM back).
  ERRIE's vector runs ONCE on that fault, SR 0x22 (MODF, TXE standing beside it). The request is full
  duplex on purpose: on a transmit-only one OVR would come first, since nothing reads
  the receive buffer.
- **The next request after a stall or a transfer error** (letter `k`, twelve runs and three of `z`;
  `recover()` called only where named). The peer's window is commanded first, and the engined host
  is then not initialized again, so the request judged is the first the host has seen
  since the failure and a frame left behind is a window out of place at both ends. A
  stall is staged with SPE taken down under the host, on each path: a polled write and a
  polled read of nine frames and the command phase of a request naming the engines, one
  frame at PCLK1/8, which runs in the loop. Each ends `spi_stalled` with the select
  reading high, the next window, 32 frames at PCLK1/32 (PCLK1/8 after the command phase),
  polled or on the streams, exact at both ends. The wall in `start()` is the budget at 180
  MHz: 11264 polls at PCLK1/32 (1024 core cycles a frame), 1.01 ms for the nine-frame write and
  for the read, 16 cycles a poll on APB1, and 768 polls for the command phase at PCLK1/8, 75
  us, 18 cycles a poll; a poll's turn moves with the image's layout. `recover()` over a frame standing
  with SPE down measures 934 cycles in twelve of the fifteen runs and 1083 or 1091 in three (5 to 6 us).
- **The order is on the pad** (letter `k`). This block drives SCK only while SPE is set: with
  the pad's pull-down on in mode 3 it falls when SPE falls and rises on the enable, and with no
  pull it held its level through the reset and no edge came within 1 ms on these wires. So
  the letter stages mode 3 with that pull-down, as a bus with a pull on the wire would be,
  counts SCK's edges through EXTI by where the select's output latch stands, and judges none
  with it low and one with it high in `start()`, both stall paths. With the reset ahead of
  the release the same stage counts that edge with the select low, and the verdict falls.
- **A transfer error, staged and survived** (letter `k`): a buffer at 0x3000 0000, in RM0390
  2.2.2's reserved 0x2003 0000 to 0x3FFF FFFF, gives a bus error and no fault of the core. A
  transmit buffer there stops the stream on its first beat, no frame clocked; a receive buffer
  there stops the receive stream on the first frame back, and a frame stands in the receive
  buffer behind it every run. `dma_isr()` answers true once, in some 650 cycles on the transmit
stream and 683 on the receive, `status()` is
  `spi_dma_fault` and the select the request named is high. The next window, polled or on the
  streams, is exact at both ends with no `recover()`; with the reset taken out, the frame standing
  is read in front of a window on the streams, all 32 frames out of place.
- **Queued behind a failure, through the arbiter** (letter `k`): two requests posted from one
  dispatch, the first bound to fail, the second holding the peer's window. The replies come in
  order, `spi_stalled` then `spi_ok` and `spi_dma_fault` then `spi_ok`, the second's window exact
  at both ends, polled and on the streams.
- **The clock gate closed under the host** (letter `k`): the reset's limit. A polled write
  stalls in 1014 us, the next in 1014 us, `recover()` answers false; the gate opened and
  `recover()` called, it answers true and the next window is exact. With the gate opened and no
  `recover()`, CR1 reads 0x0000 (the pulse reached the block through the closed gate), one more
  request stalls in 1015 us and its own reset is the way back, the next window exact.
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
  frame, 64 frames, the console drained before the twenty windows. With no
  handler a window takes 19415 to 19463 cycles; with it, about 32 runs landing
  in each, every window is exact both ways, `spi_ok`, the far client's count
  right and OVR never standing, in 28669 to 29373 cycles: 9206 to 9958 longer,
  where the suite's arithmetic allows 2478 (the runs that must land, times the
  118 cycles a run longer than the frame must idle the bus) to some 18700
  (every run whole). The letter's handler is the only interrupt the loop meets there
  but the tick: the host arms none of the block's.
- **A DISPATCH ABOVE THE ARBITER IS A GAP ON THE BUS TOO**
  (`test_stm32f4_rubato_link` under the preemptive kernel, against the
  STM32G0B1RE's `spi_peer`, eighteen runs of letter `p`): the polled loop
  runs inside the arbiter's dispatch and an object of the highest priority
  preempts it every 97 to 389 us for 300 cycles, longer than a frame at /8
  and faster, or for 3000, longer than one at /64 and faster. Every rung
  from PCLK1/256 to PCLK1/2, eight frames in each of the seven passes - 1008
  rungs in all - is exact both ways: zero lost frames, against the bound of
  zero. The engines, a two-frame request on them included, hold the same way.

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
- The resource's own waits stay a count of turns: `Spi<n>`'s `wait_until()`, under
  `disable()`, bounds at 400 000 polls, a verb below the host, which has no rate of
  its own to derive from and whose answer the host drops. A rate change behind a
  stalled block pays it once; deriving it means handing the resource the applied rate.
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
- No DMA engine parameters on `SpiClient`: a program that wants its client
  on the streams binds them raw (the stm32f4 `spi_peer` does, "The
  client run"); a pair is born with a device-shaped client.
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
- The wait for the wire's end in modes 2 and 3 is measured (letter `E`); its cost
  in the vector is measured in modes 0 and 1 alone, mode 2 waiting the same half
  bit for the opposite level. A run of the vector's ruler over modes 2 and 3 would
  measure it.
- E and F on an APB1 instance: letter `e` measures them on SPI1 alone, so the APB1
  rows of "Which engine costs the core less" use SPI1's E - F (a vector some 35
  cycles longer on SPI2 moves none of their cells). The same two requests over SPI2
  would measure them.
- The receive loop's frame in modes 2 and 3, and on SPI2 at /2: letter e's
  `spi.frame` lines run modes 0 and 1 at /4 to /256, so the header's CPHA = 0 and
  CPHA = 1 figures extrapolate from one mode each; the same lines over the other two
  would measure them.
- `SpiHost` on the STM32F429I-DISC1: that half of the suite compiles over the host as
  it runs a request here and was not run with these loops, so nothing above states the
  polled loop's cost against the two soldered devices on SPI5 (letter b's ladder, and
  auto-increment through it), the arbiter over them (letter j: four replies in order, a
  write-then-read, the surplus rejected, the sleep vote), the engines (letter
  k's block timing and its one interrupt) or 16-bit frames through both ways. A run of
  that half on the board would measure them.
- The mode fault's clearing sequence from a fresh fault on the STM32F429I-DISC1 (letter
  `d`'s SPI5 half): the strict sequence, the three accesses between the steps and
  `clear_mode_fault()`'s answer, as the Nucleo-F446RE runs them. A run of letter `d` on
  that board would measure them.
- A request on the engines on the STM32F411CE's breadboard panel at 12 MHz as
  this host runs it - the command byte in the loop inside `start()`, the data
  block on the streams: the pad finding above was measured with
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
