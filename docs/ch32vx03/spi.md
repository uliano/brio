# SPI and I2S (CH32V203, CH32V303)

Up to three synchronous serial ports, both roles, 8- or 16-bit frames,
the four transfer modes, three chip-select arrangements, the simplex and
one-wire line modes, a hardware CRC over the traffic and two DMA
requests each - the STM32F1's SPI under WCH's register names, with one
register of its own (HSCR, a high-speed read mode) - and on the
CH32V303RC and VC the I2S face of the second and third instances.
Documents of record: the CH32F/V20x_V30x_V31x reference manual V2.3
(20.1 for what the block claims, 20.2 for the SPI face's behaviour,
20.3 for the I2S face's, 20.4 for the registers with tables 20-1 to 20-3
for the three instances' maps and 20.4.10's lot notes for HSCR, 3.3.5.8
and figure 3-3 for the I2S clock, 10.2.11 with tables 10-5, 10-32 and
10-33 for the pads, 11.2.3 with tables 11-2 to 11-5 for the DMA slots,
3.4.7 and 3.4.8 for the gates and 9.5.1 with table 9-2 for the vectors),
the CH32V203 datasheet V2.8 (table 2-1 for how many SPI a part has,
section 3 for which pads it bonds) and the CH32V303 datasheet V3.5
(table 2-1-1 for the SPI and I2S counts, table 3-4 for the audio pads).
Three device classes read the chapter: CH32V20x_D6 for every CH32V203 up
to the CH32V203C8, CH32V20x_D8 for the CH32V203RB and CH32V30x_D8 for
the four CH32V303. Driver:
[brio/ch32vx03/spi.hpp](../../brio/ch32vx03/spi.hpp). Reference suites:
`test_vx03_spi`, which talks to a peer board running `spi_peer` on the
CH32V203 and to its own third instance on the CH32V303, and
`test_vx03_i2s` for the audio face.

## What the silicon does

### The instances and their buses - and that is the rate table

SPI1 is the PB2 instance and SPI2 and SPI3 the PB1 ones, which decides
three things at once: the gate bit, the reset line, and - the one that
surprises - THE CLOCK BR DIVIDES. PB2 runs at HCLK on this family and
PB1 is capped at 72 MHz by this stratum, so at the 144 MHz the bench
runs, `SpiClock::div2` is 72 MHz of SCK on SPI1 and 36 MHz on SPI2 and
SPI3. A program asks the INSTANCE for its reference and never the system
clock.

WHICH INSTANCES a part has is its datasheet's table and nothing else,
held as a mask (`device::spi_instances`): every CH32V203 below the
CH32V203C8 carries SPI1 alone, the CH32V203C8 and RB and the 128 KB
CH32V303 carry SPI1 and SPI2, and the CH32V303RC and VC all three. An
instance a part has not got does not compile. SPI3 answers at 0x4000
3C00 with its gate at PB1PCENR's bit 15 and its vector at entry 67 of
the CH32V303's tail.

### No FIFO: one buffer each way

Figure 20-1 is one transmit buffer, one receive buffer and one shift
register. That single fact shapes both roles:

- **TXE means "the buffer moved into the shifter"** - a frame EARLY -
  and **RXNE means "a frame has been shifted both ways"**, which is the
  one moment the bus is provably idle. A host's pump therefore runs on
  RXNE: one write, one interrupt, one read per frame.
- **A client answers ONE FRAME AHEAD.** 20.2.3: the slave starts to
  transmit on the first sampling edge, moving the buffer into the
  shifter and raising TXE. So the first answer must be in the buffer
  before the host's first clock, and every received frame loads the
  next. A client that falls behind does not stall the bus - the host
  reads whatever the shifter held, a wrong answer and never a missing
  one (measured: see the dummy byte below).
- **A client that never drains keeps ONE frame** and raises the overrun
  for every frame after it (measured).

### What may be written, and when

- **DFF and CRCEN "can only be written when SPE is 0"** (20.4.1), which
  is what makes a change of frame size or CRC cost a disable/enable
  pair. BR, LSBFIRST, MSTR, CPOL and CPHA "cannot be modified during
  communication".
- **The disable is a drain**, not a bit: clearing SPE while a frame is
  in flight truncates it on the wire, so the driver waits for TXE and
  for BSY to fall first, each wait bounded.
- **A RECEIVE-ONLY HOST NEVER STOPS BY ITSELF.** With RXONLY set (or
  BIDIMODE with BIDIOE clear) the host's clock runs as long as SPE
  stands and frames keep arriving with nothing written, so BSY is the
  wrong thing to wait for. Such a host is stopped by dropping SPE in the
  gap between two frames and reading the last one out; the driver's
  `disable()` recognizes the configuration and does not wait, and
  `stop_receive_only()` is the sequence. The chapter does not spell it -
  the bench does.

### The flags, and the sequence that clears each

STATR's eight bits, and what each costs (20.2.7, 20.4.3):

| Flag | How it goes down |
|------|------------------|
| RXNE | a DATAR read |
| TXE | a DATAR write |
| OVR | a DATAR read followed by a STATR read |
| MODF | nothing software can do (measured below); the block's RCC reset pulse |
| CRCERR | a ZERO into its bit and ones elsewhere (the one rc_w0 bit) |
| BSY | hardware's, never software's |
| UDR | the I2S face's: a read of STATR (20.3.7.1) |
| CHSIDE | the I2S face's: refreshed by the hardware, never cleared |

**MODF IS A DEMOTION**, not a report: the silicon clears SPE and MSTR
with it (measured). That is why this driver's bus engine keeps its chip
select as an ordinary GPIO and never hands the NSS pad to a host. AND
20.2.7's RECIPE FOR CLEARING IT DOES NOT WORK HERE - six sequences
measured in this driver and eight more with WCH's own peripheral library
on the same board, its documented one among them, and only the RCC
reset pulse puts the flag down (the findings below list them).

Five interrupt sources over ONE vector per instance in the SPI face:
TXE and RXNE under their own enables, and CRCERR, OVR and MODF together
under ERRIE; in the I2S face ERRIE gates OVR and UDR.

### A data line's edge is a correctness parameter

On a bus made of jumper wires the fastest pad is the loudest one, and an
edge coupled from a data line into the clock is ONE SAMPLING EDGE TOO
MANY - which shows in exactly the two modes that sample on the FALLING
edge while the data lines change on the rising one, modes 1 and 2. So
both tasks carry `pad_speed()`, the slew class of the pads they drive,
and the default is the port's fastest with the measurement in the
findings below.

### The pads are a column

Table 10-32 gives SPI1 two columns - PA4/PA5/PA6/PA7 and
PA15/PB3/PB4/PB5 - selected by one bit of AFIO_PCFR1, and gives SPI2
NONE: the second instance has no remap field at all and its four pads
(PB12..PB15) come from the datasheet's pin table. Table 10-33 gives SPI3
two behind PCFR1's bit 28: the default PA15/PB3/PB4/PB5 - the very pads
of SPI1's second column - and PA4/PC10/PC11/PC12, and the same bit moves
I2S3's pads with them. Which pads a package brings out is the part's own
table, and three of the CH32V203 parts bond nothing of SPI1's second
column, so naming it there is refused.

### The DMA slots

Tables 11-2 to 11-5, and on this family THE CHANNEL IS THE REQUEST, on
one of two controllers where the part has two: SPI1 receives on DMA1's
channel 2 and transmits on 3, SPI2 on DMA1's 4 and 5, and SPI3 - and
I2S3, whose requests are the same two - on DMA2's 1 and 2. An engine
named on any other slot is refused at compile time, because a wrong one
would move nothing.

IN SLEEP THE BUS MATRIX SERVES THE CORE ALONE on this family
([dma.md](dma.md)), so a transport with engines holds the program awake
while a block is in flight.

### The high-speed read mode: one code means the same on every lot

HSCR.HSRXEN moves a HOST's sampling of MISO late enough to catch an
answer at the top rate, and 20.4.10 tells its story four ways by class
and by lot. On the CH32V20x_D6 it "is only valid at clock division 2"
and the bit is write-only. On the CH32V20x_D8 - the CH32V203RB - the bit
is write-only and the mode valid at /2 alone on lots whose fifth digit
from the end is below two, anywhere on the others. On the CH32V30x_D8
lots whose penultimate sixth digit is not zero the bit reads back, BR
takes a SECOND LADDER under it (20.4.1: FPCLK/2, /3, /4 ... /9 in place
of the powers of two) and a second mode, HSRXEN2, exists - "used only
when HSRXEN is turned on and SPI PCLK is greater than or equal to 120M".
On that class's lots whose sixth digit is zero the bit is write-only
again, and the mode valid at /2 alone where the fifth digit is below
two. BR = 000 is FPCLK/2 in BOTH ladders,
so /2 is the one code at which the mode means the same thing on every
die of every class: `high_speed_read()` refuses at any other, at run
time against the code in the register and at compile time where the
configuration is a constant (`configure_high_speed_read<cfg>()`), and
HSCR is written whole because a write-only bit cannot be read back for a
read-modify-write. HSRXEN2 is a verb that asks the die - the CH32V303's
alone, refused below 120 MHz of the instance's bus, which leaves it to
SPI1, PB1 being capped at 72 - reading HSCR's word before it writes,
because an address that answers for another register has been met on
this class ([opa.md](opa.md)'s EXTEN_CTR2): a word with a bit outside
HSCR's two is taken as no, and the two bits written must come back
exactly. The mode outlives a reconfiguration (HSCR
is not CTLR1), so a host that moves off /2 turns it off first.

What the mode buys is measured on the CH32V303VCT6: SPI2 hosting SPI3
at /2 - 36 MHz of SCK, over the evaluation board's jumper wires, every
frame a DMA channel's - read 254 of 256 bytes wrong without it, 253 of
them the client's byte ONE BIT LATE (shifted one place toward the least
significant end with the previous byte's last bit on top: a sample
taken before the answer's edge), and every byte right with it, in both
8- and 16-bit frames; the client received every byte both times. That
die kept no HSRXEN2.

### The I2S face (the CH32V303RC's and VC's)

The register file carries SPIx_I2S_CFGR on every instance and SPIx_I2SPR
on SPI2 and SPI3, and chapter 20 is written for four families at once;
what decides whether a part HAS the face is its datasheet - table 2-1-1
gives two I2S to the 256 KB CH32V303 and none to the 128 KB ones, and
the CH32V203's table 2-1 has no I2S row at all. So `I2s<n>` compiles
where the part has the face and for SPI2 and SPI3 alone, and elsewhere
the SPI face keeps `i2s_config_writable()`, the one question it asks of
that register.

- **One block, two faces.** I2SMOD decides which answers. The gate, the
  reset, the vector, DATAR, STATR and CTLR2's interrupt and DMA enables
  are the one block's; CTLR1, the CRC registers, SSOE, MODF and CRCERR
  are not used in the I2S face (20.3.1). I2SMOD "can only be set when
  SPI or I2S is disabled" and every other field of I2S_CFGR and I2SPR
  "should be set when I2S is turned off", so the configuration is
  written with I2SE and SPE clear, in 20.3.4.1's order: I2SPR, then
  I2S_CFGR whole, then CTLR2's DMA bits.
- **The four standards** (20.3.2): Philips, with WS one clock ahead of
  the MSB; MSB-justified, WS with the MSB; LSB-justified, the datum's
  last bit against the channel's end; PCM, with no channel side and a
  frame PCMSYNC makes short (WS one bit wide) or long (thirteen bits).
- **The widths**: a 16-bit datum in a 16- or a 32-bit channel, a 24-bit
  or 32-bit one in a 32-bit channel - DATLEN's fourth code "not allowed",
  and CHLEN "only meaningful when DATLEN = 00, otherwise the channel
  length is fixed to 32 bits by hardware". A wider datum in a 16-bit
  channel is therefore REFUSED rather than quietly given a 32-bit one.
  A 16-bit datum is ONE data-register access a channel, a 24- or 32-bit
  one TWO (the high half first, a 24-bit datum's last byte forced to zero
  on reception) - and two DMA items.
- **The clock a master divides is SYSCLK.** Figure 3-3, the simple tree
  this class has, draws SYSCLK to both I2S interfaces, and RCC_CFGR2's
  I2S2SRC and I2S3SRC (3.3.5.8) only choose between SYSCLK and a PLL3
  the CH32V30x_D8C has and this class has not. 20.3.3's generator
  divides it by a linear 2 x I2SDIV + ODD (I2SDIV 2..255) and by the
  frame - two channels of 16 or 32 bits, or a fixed 256 with the master
  clock out, MCK being 256 x FS on its own pad (PC6 for I2S2, PC7 for
  I2S3, pads no remap moves). At a 144 MHz SYSCLK a two-channel 16-bit
  frame reaches 47872 Hz for 48 kHz and nothing below some 8.8 kHz.
  Measured: 16015 frames a second counted against the core's own counter
  for the 16014 that arithmetic gives at 16 kHz - the generator divides
  SYSCLK, and nothing else.
- **The flags of the face**: CHSIDE (the side of the datum a transmitter
  is about to send or a receiver has just taken; no meaning under PCM
  nor after an error), UDR (a client transmitter clocked before its
  data register was written, cleared by a STATR read), OVR (as the SPI
  face's), and BSY - which 20.3.6.1 says stays LOW throughout a master
  reception, and does (measured: never read set through one).
- **TXE READS 1 WITH I2SE CLEAR**, against 20.3.6.2's "when I2S is
  turned off (I2SE bit is 0), this flag is also 0": on a configured
  transmitting face, before the enable, through it and after the
  disable, TXE read 1 - and a datum written before the enable is the
  FIRST WORD ON THE WIRE once the face is enabled (measured on the
  CH32V303VCT6). So a program may load its first word before it
  enables, and nothing waits for TXE to rise at the enable.
- **The first word is the left channel's.** Under all four standards a
  client receiver's first word was the host's first datum, on the left
  (CHSIDE clear); and a client TRANSMITTER fed one datum ahead put that
  datum out as the host's first word, on the left, under Philips -
  20.3.5.1's closing sentence, "a WS signal of 1 means that the left
  channel is sent first", describes no level this pair shows.
- **A receiver is stopped on a frame's edge** by 20.3.4.3's sequences:
  a 16-bit datum in a 32-bit channel waits for the second-to-last RXNE
  and then seventeen I2S clock periods when LSB-justified, and for the
  last RXNE and one period under the other three standards; every other
  combination of widths waits for the second-to-last RXNE and one
  period - and a client clears I2SE at the last RXNE (20.3.5.2).
  `i2s_receive_stop()` states which for a configuration, and the timing
  is the caller's.

## Types and verbs

### The vocabulary

`SpiMode` (the four Motorola modes, with `spi_mode_cpol` /
`spi_mode_cpha` splitting one into its two bits), `SpiClock` (the eight
BR codes, with `spi_division` and `spi_sck_hz` turning one into a
divisor and a frequency against a given bus rate), `SpiDataSize` (the
two frame widths, with `spi_data_bits` and `spi_frame_mask`), `SpiRole`,
`SpiNss` (software, hardware output, hardware input), `SpiDirection`
(full duplex, receive only, the two halves of the one-wire mode), and
`SpiFlag` (the STATR bits as an ISR body hands them back, with `errors`
the three ERRIE gates of the SPI face and `i2s_errors` the two of the
I2S face, beside `underrun` and `channel_side`).

`spi_rate_for(pclk, max_hz)` is the runtime chooser - the coarsest code
at or below a ceiling, an empty optional where even the bus over 256 is
faster, never rounded up - and `SpiRateOf<pclk, max_hz>::clock` is the
same answer as a compile-time constant, REFUSED rather than reported
where the bus cannot make it. `spi_bus_hz<n>(clock)` is the rate that
instance's BR field divides, out of a static or a dynamic clock tag.

`SpiConfig` collects everything the two control words hold, and
`spi_config_valid` is the chapter's refusals: a field holding a value
its register field does not encode, the CRC asked for outside full
duplex or with a zero polynomial, and a CLIENT asking for the NSS output
(SSOE is a host's verb).

`SpiPins` is a column - the four pads and the remap code - and
`spi_pins_for(instance, code)` builds one out of
[afio.hpp](../../brio/ch32vx03/afio.hpp)'s tables;
`spi_default_pins<n>` is each instance's own first column, and
`spi_pins_valid` refuses a link with no clock, two signals on one pad, a
code the instance has not got, and any pad this package does not bond.
`spi_present(n)`, `spi_base_for()`, `spi_bus_for()`, `spi_gate_for()`,
`spi_irq_for()`, `spi_column_count()` and `spi_dma_rx_slot()` /
`spi_dma_tx_slot()` are the instance's facts.

The I2S face's own: `I2sMode` (client or host, transmit or receive),
`I2sStandard`, `I2sDataLength`, `I2sChannelLength`, `I2sConfig` with
`i2s_config_valid()` (a field its register field does not encode, a
host's divider of 0 or 1, PCM's long frame outside PCM, a wider datum in
a 16-bit channel, the master clock asked of a client) and the two
register words `i2s_i2scfgr_of()` / `i2s_i2spr_of()`;
`i2s_channel_is_32()` and `i2s_accesses_per_channel()`; 20.3.3's
arithmetic as `i2s_frame_factor()`, `i2s_fs_hz()`, `i2s_bit_clock_hz()`
and the chooser `i2s_prescaler_for()` returning an `I2sPrescaler`;
`i2s_clock_hz(clock)` - a static Clock's SYSCLK; `i2s_receive_stop()`
with `I2sReceiveStop`; and `I2sPins` with `i2s_pins_for(n, code,
with_mck)`, `i2s_default_pins<n>` and `i2s_pins_valid()` (WS, CK and SD
the column's own, the master clock the instance's own pad), and
`i2s_present(n)`.

### The resource

`Spi<n>`: `bus_clock()`, `reset()`, `bus_hz(clock)`, `remap(code)`;
`configure(SpiConfig)` and `configure<cfg>()` - the runtime one
reporting the refusal as false, the template one as a compile error on
the line that wrote the value; `enable()`, `enabled()`, `disable()` (the
drain), `receive_only_host()` and `stop_receive_only()`; the data verbs
(`data`, `data8`, `rxne`, `txe`, `busy`, `status`, `flush_rx`) and the
read-backs `role()`, `mode()`, `clock()`, `bits()`; the error trio with
the sequence each needs (`overrun` / `clear_overrun`, `mode_fault` /
`clear_mode_fault`, `crc_error` / `clear_crc_error`); the CRC
(`crc_polynomial`, `crc_next`, `rx_crc`, `tx_crc`); `software_select`,
`half_duplex_output`, `high_speed_read`, `configure_high_speed_read<cfg>()`,
`high_speed_read2(on, bus_hz)` (the CH32V303's), `i2s_config_writable`,
`dma_requests`; the three interrupt enables and `isr()`, the body that
hands back the raised-and-ENABLED sources and consumes nothing.

`I2s<n>` for n = 2 and 3 on the parts that have the face: `bus_clock()`,
`reset()`, `remap()` (the instance's), `configure(I2sConfig)` and
`configure<cfg>()`, `select_spi_mode()` / `i2s_mode_selected()`,
`enable()` / `enabled()` / `disable()` (a transmitter drained, a
receiver stopped where it stands), `mode()`, `standard()`, the generator
(`prescaler(div, odd)` refused while enabled, `prescaler_div()`,
`prescaler_odd()`, `master_clock_out()`), the data and flag verbs
(`data`, `tx_empty`, `rx_ready`, `busy`, `status`, `right_channel`,
`underrun` / `clear_underrun`, `overrun` / `clear_overrun`), the
enables and requests shared with the SPI face (`dma_requests`,
`rxne_interrupt`, `txe_interrupt`, `error_interrupt`), `isr()`,
`data_address()`, and `claim_pads<pins>(mode)` / `release_pads<pins>()`
(table 10-5's configuration for each mode); the constants `number`,
`irq`, `dma_rx_slot`, `dma_tx_slot`.

### The host engine

`SpiHost<n, pins, TxEngine, RxEngine>` is the engine
[util/spi_bus.hpp](../../brio/util/spi_bus.hpp)'s `SpiBus` drives, and
its `Request` is the other strata's VERBATIM: a chip select the bus AO
asserts, a display D/C line, a CS setup time, a command phase and a data
phase with optional out and in buffers, the mode/rate/frame size per
request, and a `polled` flag choosing between the per-frame ISR pump and
a synchronous spin inside `start()`. Beside it: `init(clock, max_sck_hz)`
with an optional bus-wide SCK ceiling, `rebase()` for a dynamic clock's
fan-out, `clock_for()` / `sck_hz()` / `reference_hz()`, `prime()` (mode
and rate now, moving no data - a CPOL flip is an edge, so prime BEFORE
the select falls), `bit_order()` (a property of the WIRE and so a
bus-level verb), `pad_speed()` (the slew class of SCK and MOSI, a
choice that survives the next `init()`), `start()`, `isr()`,
`dma_isr()`, `status()`, `busy()`, `recover()`, `release()` and
`claim_nss_pad()`.

### The client

`SpiClient<n, pins>`: `init(clock, Config)`, `enable(first)` (SPE up
with the first answer already in the buffer), `write` / `writable` /
`poll`, `selected()` (a live read of the NSS pad, since the peripheral
publishes no select status), `select()` for a client under software
management, `drive_output()` and `output_driven()` - THE DARK LISTENER,
the answer line handed to the peripheral only for the window it answers
in - `pad_speed()` for that one pad, the error verbs, `isr()`,
`release()`, and `frames_ahead` = 1, this silicon's answer to the one
number a portable client would need.

## How to use it

One device, polled, on SPI2's own column:

```cpp
using Bus = brio::SpiHost<2>;
Bus::init(clock);                        // no ceiling: every rate legal

const uint8_t cmd[1] = {0x9F};
uint8_t id[3] = {};
Bus::Request r{};
r.cs      = brio::Pin<'B', 12>::ref();
r.cmd     = brio::lend<brio::Lease::reply>(cmd);
r.cmd_len = 1;
r.rx      = brio::lend<brio::Lease::reply>(id);
r.len     = 3;
r.mode    = brio::SpiMode::mode0;
r.clock   = brio::SpiClock::div16;
r.polled  = true;
(void)Bus::start(r);                     // true: it is done, `id` is filled
```

A shared bus under the arbiter, with the transactions on the interrupt:

```cpp
using Arb = brio::SpiBus<Bus, Platform, 4>;
extern "C" BRIO_CH32_INTERRUPT void spi2_handler() {
    if (Bus::isr()) { brio::post<Arb>(brio::TransferDone{Bus::status()}); }
}
r.polled = false;
r.reply  = brio::reply_to<MyAo, brio::SpiDone>();
brio::post<Arb>(r);
```

The DMA engines, on the two slots the tables wire to the instance - DMA1
for SPI1 and SPI2, DMA2 for the CH32V303's SPI3:

```cpp
using Fast = brio::SpiHost<1, brio::spi_default_pins<1>,
                           brio::DmaTxEngine<1, 3>, brio::DmaRxEngine<1, 2>>;
brio::Dma<1>::open();
Fast::init(clock);
extern "C" BRIO_CH32_INTERRUPT void dma1_channel2_handler() { (void)Fast::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel3_handler() { (void)Fast::dma_isr(); }

using Third = brio::SpiHost<3, brio::spi3_default_pins,
                            brio::DmaTxEngine<2, 2>, brio::DmaRxEngine<2, 1>>;
```

A device with a datasheet limit, refused at compile time where the bus
cannot make it; and the high-speed read, refused at compile time at any
code but /2:

```cpp
constexpr auto rate = brio::SpiRateOf<SysClock::pclk1_hz, 4'000'000>::clock;
brio::Spi<2>::configure_high_speed_read<brio::SpiConfig{.clock = brio::SpiClock::div2}>();
```

The client, answering one frame ahead and dark between windows:

```cpp
using Peer = brio::SpiClient<2>;
Peer::init(clock, {.nss = brio::SpiNss::hardware_input, .drive_output = true});
Peer::enable(first_answer);              // before the host's first clock
while (const auto got = Peer::poll()) { Peer::write(next_answer(*got)); }
Peer::drive_output(false);               // the answer line back to the bus
```

An I2S master transmitting 16-bit Philips frames at 48 kHz, the divider
found at compile time against SYSCLK, and a client receiving on the
other instance:

```cpp
constexpr auto pre = *brio::i2s_prescaler_for(brio::i2s_clock_hz(clock), 48'000, false, false);
using Out = brio::I2s<2>;
using In  = brio::I2s<3>;
In::bus_clock(true);
(void)In::claim_pads<brio::i2s_default_pins<3>>(brio::I2sMode::client_receive);
(void)In::configure({.mode = brio::I2sMode::client_receive});
In::enable();                            // listening before the master's clock starts
Out::bus_clock(true);
(void)Out::claim_pads<brio::i2s_default_pins<2>>(brio::I2sMode::host_transmit);
Out::configure<brio::I2sConfig{.mode = brio::I2sMode::host_transmit,
                               .div = pre.div, .odd = pre.odd}>();
Out::enable();                           // then feed Out::data() on TXE, left channel first
```

## Bench findings

`test_vx03_spi` on the CH32V203C8T6 at 144 MHz of HCLK (PB2 = 144 MHz,
PB1 = 72 MHz), with two instruments: a strap from PA7 to PA6 that makes
SPI1's own MOSI its MISO, and a peer board on SPI2's four wires running
`spi_peer`. The CH32V303VCT6's findings follow in a section of their
own.

- **The clock is right on the pad that carries it.** SPI1 on its second
  column puts SCK on PB3, which is TIM2's channel 2 under that timer's
  partial remap, and the pad's own edges reach the capture input while
  the SPI drives it: at every one of the eight BR codes a 64-frame burst
  produced exactly 512 rising edges, the fastest (72 MHz of SCK counted
  by a 144 MHz timer) included.
- **The two instances' rate tables differ**, as their buses do: SPI1's
  ladder runs 72 MHz down to 562.5 kHz, SPI2's 36 MHz down to
  281.25 kHz.
- **The polled loop, not the wire, is the limit at the fast end.** On
  SPI1's strap a 256-byte polled burst costs 101 core cycles a frame at
  /2 where the wire alone is 16, and 2133 at /256 where the wire is
  2048; the two meet around /64. The DMA engines close that gap: the
  same block at /4 costs 36 cycles a frame against the wire's 32.
- **THE STRAP IS CLEAN TO 36 MHz AND BREAKS AT 72.** Every BR code from
  /4 down carried 256 bytes byte-exact; /2 did not. Four kilobytes at
  /4 (36 MHz of SCK), sent as sixteen chunks, came back byte-exact.
- **Every mode, both widths and both bit orders round-trip** on the
  strap, and the pump takes exactly one interrupt per frame.
- **The hardware CRC is the arithmetic.** Over six frames with the reset
  polynomial the transmit CRC register held what a bitwise loop over the
  same polynomial computes, the receive register held the same number,
  the CRC frame the loop read back was its low byte, and CRCERR stood
  down.
- **A host clocks itself.** With MOSI released and MISO held by the
  port, every frame comes back all ones or all zeros with the level, at
  both widths, both bit orders and all four modes - the wireless proof
  that the receive path is the pad and not a leftover.
- **The mode fault takes the role away, AND THE CHAPTER'S WAY BACK IS
  NOT ONE.** Clearing SSI on a software-managed host raises MODF and the
  block loses MSTR and SPE with it (CTLR1 0x228, STATR 0x22 at the
  fault), and the error reaches the instance's vector under ERRIE - one
  entry carrying the mode-fault bit. But 20.2.7's "a read or write
  operation to STATR, then a write to CTLR1" leaves the flag standing
  under every shape of those two steps the bench tried: the CTLR1 write
  rewriting the register as it stands, the write that RAISES SPE (which
  is the shape WCH's own library documents, and the one the chapter's
  words leave open), the write that restores SSI, the sequence with SSI
  raised BEFORE it, and a STATR WRITE in place of the read. The block's
  RCC reset pulse is what puts MODF down. WCH's own peripheral library,
  built with the vendor's compiler and run on the same board, does no
  better: its documented sequence (`SPI_I2S_GetFlagStatus` then
  `SPI_Cmd`) with SSI still low and with SSI raised first, its
  `SPI_I2S_ClearFlag`, a CTLR1 rewrite, a disable-then-enable, a whole
  `SPI_Init` again - eight sequences, the flag standing through every
  one, and `RCC_APB2PeriphResetCmd` the one thing that clears it. And
  the register shows why no sequence can work: while MODF stands a
  CTLR1 write LANDS (SSI moves) but SPE NEVER TAKES - the bit reads
  zero after every write that sets it - so a demoted host cannot be
  re-enabled at all without the reset line. Since MODF has taken SPE
  and MSTR with it, a demoted host must be reconfigured anyway, and
  that reset is the first step of it.
- **The one-wire line mode turns around on the same pad**: BIDIMODE with
  BIDIOE drives a frame out of MOSI, and clearing BIDIOE makes that same
  pad the receiver, with the clock running on either side of the turn.
- **The overrun is a level with a sequence**: frames arriving on a full
  buffer raise OVR, and only a DATAR read followed by a STATR read
  clears it.
- **Between two chips**, over SPI2's four wires: all four transfer modes
  and both bit orders carry a burst byte-exact in both directions; a
  bit-order mismatch is an exact two-way bit reversal; the command
  channel answered ten pings of ten at 281.25 kHz. Sixteen bytes through
  SPI2's DMA ENGINES (channels 5 and 4) came back byte-exact at both
  ends, and a TEN-SECOND STRESS ran 23 exchanges of sixteen frames with
  every one exact at both ends.
- **THE PEER LINK IS EXACT TO 1.125 MHz AND BREAKS AT 2.25 MHz**, and
  the break is the far end's ANSWER RELOAD and not the wire: at every
  rate above the boundary the client still received all eight characters
  with no mismatch of its own while what this end read back was wrong.
- **THE ANSWER LINE'S SLEW CLASS IS A CORRECTNESS PARAMETER.** With the
  far board's MISO pad at its driver's fastest class the four-mode
  matrix slips - mode 2 on one pass, mode 1 on another, the two that
  sample on the falling edge - the engined exchange comes back nine
  bytes of sixteen wrong and the stress loses five exchanges of
  twenty-three. Walking THIS end's own SCK and MOSI down the three
  classes this port has cures none of it; walking the far end's MISO
  down to its slowest cures all of it, with this end's pads left at the
  driver's fastest. So the aggressor is the CLIENT's answer edge, and
  the rate ladder above is unchanged at every class - the slow pad costs
  nothing on this bus.
- **The select is a real wire**: the peer pulling it low from its own
  port is seen at this end, which is what frames a client's transaction.
- **The dummy byte is measured and not assumed.** Told not to preload,
  the client sends the shifter's leftover in the first frame and its own
  stream one place late from there - which is the one-ahead rule seen
  from the other end.
- **A client that never drains keeps one frame**, not a FIFO's worth,
  and raises the overrun.
- **The roles invert.** With the peer as the bus host, this chip's
  `SpiClient` answered one frame ahead under a select the peer framed,
  and both ends agreed byte for byte; the answer line was released again
  afterwards.

### On the CH32V303VCT6

`test_vx03_spi` (26 verdicts in `z`) and `test_vx03_i2s` (15) on WCH's
evaluation board at the same 144 MHz, over four jumper wires between
SPI2's column and SPI3's default one - PB12-PA15, PB13-PB3, PB14-PB4,
PB15-PB5, each looked for before a letter uses it - which carry I2S2 to
I2S3 as well; there is no strap on SPI1's own pads and no peer board, so
the letters that want either decline by name.

- **SPI3 in both roles, against SPI2**: as the client of SPI2's host
  engine, served one frame ahead from its own vector, and as the host
  over SPI2's client, all four modes and both bit orders in 8-bit frames
  and a run of 16-bit ones - thirty-two frames exact both ways in every
  combination.
- **The high-speed read** (above): at SPI2's /2, 36 MHz, 254 bytes of
  256 wrong without HSRXEN, 253 of them one bit late, none with it, in
  8272 core cycles for the block against the wire's 8192. SPI1's /2 is
  72 MHz of SCK, reached here through SPI1's second column, whose pads
  are SPI3's default ones and so ride the same wires to SPI2 - and there
  the far end is SPI2 as a CLIENT on a 72 MHz bus, taking a clock at its
  own bus rate: the host read 254 or 255 bytes wrong with the mode and
  without it, and the client itself received 254 to 256 wrong in some
  runs and none in another, so that pairing says nothing about the
  host's read.
  At /8 the same path is exact both ways.
- **The clock counted on PB3 at /2** - 72 MHz of SCK, half the counting
  timer's clock - came to 233 and 271 of 512 edges on this board, where
  PB3 carries the jumper to PB13; every slower code counted exactly 512.
  Whether the lost edges are the wire's load or the timer's input at its
  own ceiling is not separable here (the CH32V203C8T6, with nothing on
  PB3, counted all 512).
- **Half-words through all four channels** (DMA1's 4 and 5 for SPI2,
  DMA2's 1 and 2 for SPI3, every channel moving 16-bit items into 16-bit
  frames) against the same bytes as 8-bit frames, a 256-byte block each
  way, in core cycles: a 16-bit frame 64.6 at /2 (the wire's 64), 128.6
  at /4, 256.6 at /8 and 512.6 at /16; a byte 32.3, 64.3, 128.3 and 256.3
  cycles in either frame width. The engines add under one per cent to the
  wire at every rate, so a half-word frame halves the number of requests
  and costs nothing on the wire; the block was exact both ways at /4, /8
  and /16 and at /2 with HSRXEN, and one bit late at /2 without it, in
  both widths.
- **The I2S face**, I2S2 against I2S3 on WS, CK and SD: every field of
  I2S_CFGR and I2SPR written and read back on both instances; Philips,
  MSB- and LSB-justified and PCM in both frames carrying 96 words word for
  word with the channel side flipping at every word where the standard
  has one; a 16-bit datum in a 32-bit channel in one access, 24- and
  32-bit data in two, the 24-bit datum's low byte reading zero in 48 of
  48 second accesses; the roles swapped; a host RECEIVING from a client
  fed one datum ahead, stopped on the word 20.3.4.3 names; OVR on a
  receiver nobody read and UDR on a client transmitter nobody fed, each
  reaching its instance's vector under ERRIE, and TXE and RXNE through
  both vectors; and a block of 256 half-words at 48 kHz - 2674 us, the
  frame rate's own - through I2S2's transmit request on DMA1's channel 5
  and I2S3's receive request on DMA2's channel 1.
- **SPI1's I2SMOD reads back zero** on this part, whose datasheet gives
  the face to SPI2 and SPI3 alone.

## Not covered yet

Driver gaps:

- **The hardware NSS arrangements as the ENGINE's select** (SSOE's
  output and the multi-host input). The bus AO's select is a GPIO on
  purpose - a hardware input's low level takes the host's role away, and
  the measurement above shows exactly that - so what is left unproven is
  a MULTI-HOST bus, which wants a second host and an arbitration to
  observe.
- **`Spi<2>` as a CLIENT under software management** (SSM with SSI
  driven low by the program instead of by the wire): the peer's select
  is a real wire here, so the software-selected client has no user.
- **The CRC through the DMA engines**: the engines carry the data phase
  and the CRC frame is CRCNEXT's, one frame after the block ends. Born
  with the first device that checks one over a block.
- **16-bit frames through the host engine's DMA slots**: the engines of
  `SpiHost` carry bytes and a 16-bit request falls back to the pump - the
  other strata's rule, and the transfer-granularity question the first
  portable example is meant to settle. What the silicon costs for
  half-words through all four channels is measured above, at the
  resource, and decides nothing.
- **The BR ladder under HSRXEN on the CH32V30x_D8 lots that have it**
  (FPCLK/3, /5 ... /9): declined, because a program cannot read its lot
  and /2, the one code both ladders share, is what the driver offers.
- **An audio task over the I2S face**: the resource and its verbs are
  here, and a stream of samples with its double buffer is an application
  that has not been born.
- **The master clock on its pad, and a codec**: MCKOE is written and read
  back, and `claim_pads()` hands PC6 or PC7 to a host - but the
  evaluation board's PC6 carries the timers' wire, and no codec is on the
  desk.
- **The I2S face's full duplex**: this class has no extension block, so
  a two-way audio link is two whole instances, one transmitting and one
  receiving on a shared WS and CK - which is how the bench suite pairs
  them.

Implemented but not bench-verified, each with what would measure it:

- **The high-speed read at 72 MHz of SCK**, SPI1's /2 - the rate the
  mode is for - and **HSRXEN2**: on the CH32V303 board the only far end
  at that rate is SPI2 as a client on a 72 MHz bus, which does not keep
  up, and the die kept no HSRXEN2. A strap from SPI1's MOSI to its own
  MISO, as the CH32V203C8T6's board carries, would measure the first on
  a CH32V303; a CH32V303 of a lot that has the bit, the second.
- **SPI1's clock counted at /2 on a CH32V303**: PB3 carries the jumper
  to PB13 on the evaluation board; the pad free of it would say whether
  the count at half the timer's clock is the silicon's or the wire's.
- **SPI3's second column (PA4/PC10/PC11/PC12), and I2S3's on it**: the
  code is written by `remap()` and refused where the package bonds none
  of its pads, and no suite drives it - on the evaluation board PC10 and
  PC11 carry the crossed pair between USART2 and UART4. A strap from
  that column to SPI2's four pads, with the pair lifted, would measure
  it.
- **SPI1's hardware NSS input and output on its own pad.** On the
  CH32V203's board PA4 is strapped to PA5, so a hardware NSS input on
  SPI1 would read its own clock and demote the host at the first edge -
  a strap's consequence and not a silicon fact. A bench with PA4 free
  would measure it.
- **`recover()` after a wedged transaction, and the arbiter's per-bus
  timeout with it**: a lost completion has to be staged, and that wants
  a second device that can hold the bus.
- **`rebase()` and the SCK ceiling across a clock change**: the fan-out
  is written and compiled, and no suite drives this chapter under a
  `DynamicClock` yet.
- **The transmit interrupt (TXEIE) on a host**: the host engine arms
  RXNE alone, and a transmit-only stream with no reader would be its
  first user; on a client, answers written from TXE one frame ahead are
  measured above.
