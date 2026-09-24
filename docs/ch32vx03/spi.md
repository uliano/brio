# SPI (CH32V203)

Up to two synchronous serial ports, both roles, 8- or 16-bit frames, the
four transfer modes, three chip-select arrangements, the simplex and
one-wire line modes, a hardware CRC over the traffic and two DMA
requests each - the STM32F1's SPI under WCH's register names, with one
register of its own and without the I2S the four-family chapter also
describes. Documents of record: the CH32F/V20x_V30x_V31x reference
manual V2.3 (20.1 for what the block claims, 20.2 for the behaviour,
20.4 for the registers with tables 20-1 and 20-2 for the two instances'
maps, 10.2.11 and table 10-32 for the pads, 11.2.3 with table 11-5 for
the DMA rows, 3.4.7 and 3.4.8 for the gates and 9.5.1 with table 9-2 for
the two vectors) and the CH32V203 datasheet V2.8 (table 2-1 for how many
SPI a part has, section 3 for which pads it bonds). Ours is the
CH32V20x_D6 device class for every part up to the CH32V203C8 and
CH32V20x_D8 for the CH32V203RB. Driver:
[brio/ch32vx03/spi.hpp](../../brio/ch32vx03/spi.hpp). Reference suite:
`test_vx03_spi`, which talks to a peer board running `spi_peer`.

## What the silicon does

### Two instances on two buses, and that is the rate table

SPI1 is the PB2 instance and SPI2 the PB1 one, which decides three
things at once: the gate bit, the reset line, and - the one that
surprises - THE CLOCK BR DIVIDES. PB2 runs at HCLK on this family and
PB1 is capped at 72 MHz by this stratum, so at the 144 MHz the bench
runs, `SpiClock::div2` is 72 MHz of SCK on SPI1 and 36 MHz on SPI2. A
program asks the INSTANCE for its reference and never the system clock.

HOW MANY INSTANCES a part has is the datasheet's table 2-1 and nothing
else: every part below the CH32V203C8 carries SPI1 alone, and `Spi<2>`
does not compile there.

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
| UDR, CHSIDE | the I2S face's; they do not move in SPI mode |

**MODF IS A DEMOTION**, not a report: the silicon clears SPE and MSTR
with it (measured). That is why this driver's bus engine keeps its chip
select as an ordinary GPIO and never hands the NSS pad to a host. AND
20.2.7's RECIPE FOR CLEARING IT DOES NOT WORK HERE - six sequences
measured in this driver and eight more with WCH's own peripheral library
on the same board, its documented one among them, and only the RCC
reset pulse puts the flag down (the findings below list them).

Five interrupt sources over ONE vector per instance: TXE and RXNE under
their own enables, and CRCERR, OVR and MODF together under ERRIE.

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
(PB12..PB15) come from the datasheet's pin table. Which pads a package
brings out is the part's own table, and three of the nine parts bond
nothing of SPI1's second column, so naming it there is refused.

### The DMA rows

Table 11-5, and on this family THE CHANNEL IS THE REQUEST: SPI1 receives
on channel 2 and transmits on 3, SPI2 receives on 4 and transmits on 5.
An engine named on any other channel is refused at compile time, because
a wrong one would move nothing.

IN SLEEP THE BUS MATRIX SERVES THE CORE ALONE on this family
([dma.md](dma.md)), so a transport with engines holds the program awake
while a block is in flight.

### The high-speed read mode is not a second rate table here

HSCR.HSRXEN exists on every instance, but 20.4.10's note makes the
alternate BR ladder (FPCLK/2, /3, /4 ... instead of the powers of two)
the property of the CH32F20x_D8/D8C, CH32V30x_D8/D8C and CH32V31x_D8C
alone; for the CH32F20x_D6 and CH32V20x_D6 series "this mode is only
valid at clock division 2". The bit is also WRITE-ONLY here, so nothing
reads it back. The driver's `high_speed_read()` therefore refuses at any
BR code but /2, and HSRXEN2 - another class's bit outright - is not
spelled.

### There is no I2S on this series

Chapter 20 describes an I2S face for four families at once, and the
register maps carry SPIx_I2S_CFGR on both instances (SPIx_I2SPR, the
prescaler a master I2S needs, on SPI2 alone). The CH32V203 DATASHEET
gives this series no I2S: no row in table 2-1, no audio signal in any
package's pin table, and the clock a master would need - RCC_CFGR2's
I2S2SRC and a PLL3 - belongs to another class. The driver carries no I2S
verbs; `i2s_config_writable()` is the one question it asks of that
register, and what the silicon answers is in the findings below.

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
the three ERRIE gates).

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
`half_duplex_output`, `high_speed_read`, `i2s_config_writable`,
`dma_requests`; the three interrupt enables and `isr()`, the body that
hands back the raised-and-ENABLED sources and consumes nothing.

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

The DMA engines, on the two channels table 11-5 wires to the instance:

```cpp
using Fast = brio::SpiHost<1, brio::spi_default_pins<1>,
                           brio::DmaTxEngine<3>, brio::DmaRxEngine<2>>;
brio::Dma::open();
Fast::init(clock);
extern "C" BRIO_CH32_INTERRUPT void dma1_channel2_handler() { (void)Fast::dma_isr(); }
extern "C" BRIO_CH32_INTERRUPT void dma1_channel3_handler() { (void)Fast::dma_isr(); }
```

A device with a datasheet limit, refused at compile time where the bus
cannot make it:

```cpp
constexpr auto rate = brio::SpiRateOf<SysClock::pclk1_hz, 4'000'000>::clock;
```

The client, answering one frame ahead and dark between windows:

```cpp
using Peer = brio::SpiClient<2>;
Peer::init(clock, {.nss = brio::SpiNss::hardware_input, .drive_output = true});
Peer::enable(first_answer);              // before the host's first clock
while (const auto got = Peer::poll()) { Peer::write(next_answer(*got)); }
Peer::drive_output(false);               // the answer line back to the bus
```

## Bench findings

`test_vx03_spi` on the CH32V203C8T6 at 144 MHz of HCLK (PB2 = 144 MHz,
PB1 = 72 MHz), with two instruments: a strap from PA7 to PA6 that makes
SPI1's own MOSI its MISO, and a peer board on SPI2's four wires running
`spi_peer`.

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
- **16-bit frames through the DMA engines**: the engines carry bytes and
  a 16-bit request falls back to the pump - the other strata's rule, and
  the transfer-granularity question the first portable example is meant
  to settle.
- **The high-speed read mode's effect**: on this device class 20.4.10
  confines it to BR = /2, which is the one code the strap does not carry
  cleanly, so there is no clean baseline to compare it against. What
  would close it is a short wire between two instances of one board, or
  a device that answers at 72 MHz.

Implemented, not bench-verified (each with what would measure it):

- **SPI1's hardware NSS input and output on its own pad.** On this board
  PA4 is strapped to PA5, so a hardware NSS input on SPI1 would read its
  own clock and demote the host at the first edge - a strap's
  consequence and not a silicon fact. A bench with PA4 free would
  measure it.
- **SPI1's second column as a working link.** It is measured as a CLOCK
  (the edge count above) but never as a four-wire transfer: PB4 and PB5
  carry no strap on this board. A jumper from PB5 to PB4 would make it
  the same loopback the first column has.
- **`recover()` after a wedged transaction, and the arbiter's per-bus
  timeout with it**: a lost completion has to be staged, and that wants
  a second device that can hold the bus.
- **`rebase()` and the SCK ceiling across a clock change**: the fan-out
  is written and compiled, and no suite drives this chapter under a
  `DynamicClock` yet.
- **The transmit interrupt (TXEIE)**: the engine arms RXNE alone, and
  the client's one-ahead pump is driven by the received frame. A
  transmit-only stream with no reader would be its first user.
